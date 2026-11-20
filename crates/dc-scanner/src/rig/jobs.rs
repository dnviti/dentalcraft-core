//! Rig jobs: `scan.auto` and `turntable.calibrate`, each on its own thread.
//!
//! A job owns nothing of the controller: it moves the table through the
//! shared [`TableHandle`] and steers the acquisition through the
//! [`RigLink`] (hold the frames out of the model, capture settled stops,
//! close loops). The controller polls [`JobShared::status`] and, once the
//! thread ended, finalises it (see `rig::control`).
//!
//! Frames reach the host late (0.7 to 1.1 s on a MINI 2), so after every
//! move the job waits [`settle`]: the requested settle time, and at least
//! the device latency plus [`SETTLE_MARGIN`]; the stop capture then drops
//! one more frame.
//!
//! **`scan.auto`**, for each tilt (the frames held out of the model):
//!
//! * *stops* (default): tilt, then for every `step_deg` around the
//!   rotation: move, settle, read the angle at rest, capture the median of
//!   `frames` frames; the acquisition registers it from the turntable pose
//!   and fuses it (a refused registration fuses the stop at the turntable
//!   pose and flags it). After a full revolution the loop is closed. The
//!   first views of the first pass check the rig calibration
//!   ([`CalibrationCheck`]): when they are not where it puts them, the
//!   scanner or the turntable moved since it was measured and the job
//!   fails at once with `calibration-outdated` (calibrate again).
//! * *continuous*: tilt, settle, release the fusion and rotate by
//!   `rotation_deg` at a speed of about [`CONTINUOUS_DEG_PER_FRAME`] per
//!   frame unless given.
//!
//! A user pause (`scan.pause`) stops the table; `scan.resume` sends the
//! rest of the motion again. At the end the job reports a [`ScanQuality`]
//! and fails with `scan-quality-low` when too little was registered.
//!
//! **Finish** (`finish`, stop-and-capture): once the stops are in, the job
//! ends the acquisition ([`RigLink::end_acquisition`]: the scanner stops
//! streaming, the fill light goes off, nothing is tracked, the session
//! reports `stopped` and the table is free) with a snapshot of the kept
//! stops, and, still on its thread (cancellable, one step per phase),
//! rebuilds the fine mesh from them ([`crate::reconstruct`]) and finishes
//! it ([`dc_geom::finish`]); the status carries both reports, and the
//! controller, finalising the job, puts the cloud and the two meshes (raw,
//! then finished: two undo steps) in the scan editor.
//!
//! **`turntable.calibrate`**, with a static object on the platter: settled
//! stops every `step_deg` (and at `+t` / `-t` on a dual-axis table), then
//! [`calib::measure`] (see there). A refused calibration fails the job with
//! its code and leaves the residuals in the status.
//!
//! **Link drops**: when the link to the table drops (a Bluetooth
//! disconnection, or a protocol error or a timeout followed by the drop
//! within [`LINK_LOSS_GRACE`]) the job reconnects the table
//! ([`Turntable::reconnect`], at most [`RECONNECT_TIMEOUT`]), waits for it
//! to stand still and does again what failed: a move is sent again from
//! where the table is (the targets are absolute angles of the pass), a
//! reading is taken again, so the pass goes on at the stop where it was.
//! The status counts the reconnections (`reconnects`); a link that does
//! not come back, or more than [`MAX_RECONNECTS`] drops, fails the job
//! (`ble-disconnected`), and the controller keeps the stops fused so far.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, MutexGuard};
use std::thread::JoinHandle;
use std::time::{Duration, Instant, SystemTime};

use dc_geom::finish::{FinishParams, FinishStage, finish_mesh};
use dc_mesh::TriMesh;
use dc_turntable::{AxisKind, LinkProbe, Turntable, TurntableError};

use super::calib::{self, CalibrationStop};
use super::{
    RigCalibration, RigJobKind, RigJobPhase, RigJobState, RigJobStatus, ScanQuality,
    save_calibration,
};
use crate::reconstruct::{ReconstructParams, ReconstructStage, reconstruct};
use crate::session::{CaptureRequest, RigLink, StopOutcome};
use crate::{Result, ScanError};

/// The opened table, shared by the controller, the jobs and the settle
/// watcher.
pub(crate) type TableHandle = Arc<Mutex<Box<dyn Turntable>>>;

/// Locks the table (a panicked holder does not poison it for good).
pub(crate) fn lock(t: &TableHandle) -> MutexGuard<'_, Box<dyn Turntable>> {
    t.lock().unwrap_or_else(|e| e.into_inner())
}

/// Longest wait for a frame.
const FRAME_TIMEOUT: Duration = Duration::from_secs(10);
/// Longest tilt.
const TILT_TIMEOUT: Duration = Duration::from_secs(60);
/// Slowest rotation assumed for the timeouts (deg/s; the Revopoint table
/// turns at 4 deg/s at its slowest).
const SLOWEST_DEG_PER_S: f32 = 3.0;
/// Rotation speed of the moves between stops (deg/s): nothing is captured
/// while the table turns.
pub(crate) const STOP_MOVE_DEG_PER_S: f32 = 30.0;
/// Tilt speed of the jobs (deg/s): the firmware value 60, measured at
/// about 6.7 deg/s.
pub(crate) const TILT_DEG_PER_S: f32 = 6.7;
/// Tilt amplitude of the calibration (degrees), at most half the range.
pub(crate) const CALIBRATION_TILT_DEG: f32 = 15.0;
/// Settle time beyond the device latency (seconds).
pub(crate) const SETTLE_MARGIN: Duration = Duration::from_millis(400);
/// Continuous `scan.auto`: rotation per frame when no speed is given
/// (degrees), clamped to [`CONTINUOUS_DEG_PER_S`].
pub(crate) const CONTINUOUS_DEG_PER_FRAME: f32 = 2.0;
/// Speed range of the continuous `scan.auto` default (deg/s; 4 is the
/// slowest the Revopoint table turns).
pub(crate) const CONTINUOUS_DEG_PER_S: (f32, f32) = (4.0, 30.0);
/// Stops: a scan fusing less than this fraction of its stops fails (a
/// flagged stop is fused at the turntable pose, which the recorded data
/// show exact: a full turn ends on the very first view).
const MIN_FUSED_STOPS: f32 = 0.5;
/// Stops: a mean residual of the fused stops above this (mm) is a warning.
/// On the rig the stops fused at the turntable pose show 0.27-0.29 mm (the
/// registration refinement is refused on most real stops: the turntable
/// is exact, the single-view registration is not, so a flagged stop is
/// not a weakness by itself).
const WEAK_RMS_MM: f32 = 0.4;
/// A loop error beyond this (degrees) is a warning.
const WEAK_LOOP_DEG: f32 = 1.0;
/// Continuous: a scan fusing less than this fraction of its frames fails.
const MIN_FUSED_FRACTION: f32 = 0.25;
/// Continuous: fusing less than this fraction is a warning.
const WEAK_FUSED_FRACTION: f32 = 0.6;
/// Warning code of a usable but weak scan.
pub const WARN_SCAN_QUALITY_WEAK: &str = "scan-quality-weak";
/// Steps of the finish of `scan.auto`: refine, fuse, extract, clean, fill
/// holes, optimise.
pub(crate) const FINISH_STEPS: u32 = 6;
/// Longest wait for a dropped link to the table to come back, per drop.
pub(crate) const RECONNECT_TIMEOUT: Duration = Duration::from_secs(30);
/// Link drops a job recovers from; one more fails it.
pub(crate) const MAX_RECONNECTS: u32 = 10;
/// After a protocol error or a timeout with the link still reported up,
/// how long the job waits for the system to report a drop: a radio drop
/// is known only after the supervision timeout of the link (seconds).
const LINK_LOSS_GRACE: Duration = Duration::from_secs(6);

/// The raw and the finished mesh of a `scan.auto` finish.
pub(crate) type FinishedMeshes = (TriMesh, TriMesh);

/// Status and cancellation of a job, shared with the controller.
#[derive(Debug)]
pub(crate) struct JobShared {
    cancel: AtomicBool,
    status: Mutex<RigJobStatus>,
    result: Mutex<Option<RigCalibration>>,
    meshes: Mutex<Option<FinishedMeshes>>,
    /// The job is done with the table (it only computes from here on).
    table_free: AtomicBool,
}

impl JobShared {
    fn new(kind: RigJobKind, steps: u32) -> Arc<Self> {
        Arc::new(Self {
            cancel: AtomicBool::new(false),
            table_free: AtomicBool::new(false),
            status: Mutex::new(RigJobStatus {
                kind,
                state: RigJobState::Running,
                step: 0,
                steps,
                phase: (kind == RigJobKind::Auto).then_some(RigJobPhase::Acquire),
                tilt_deg: None,
                error: None,
                error_detail: None,
                calibration: None,
                quality: None,
                reconstruction: None,
                finish: None,
                reconnects: 0,
                reconnecting: false,
                kept_stops: None,
            }),
            result: Mutex::new(None),
            meshes: Mutex::new(None),
        })
    }

    fn lock(&self) -> MutexGuard<'_, RigJobStatus> {
        self.status.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// Progress of the job.
    pub fn status(&self) -> RigJobStatus {
        self.lock().clone()
    }

    /// Asks the job to stop.
    pub fn cancel(&self) {
        self.cancel.store(true, Ordering::SeqCst);
    }

    fn cancelled(&self) -> bool {
        self.cancel.load(Ordering::SeqCst)
    }

    /// The calibration measured by a finished calibrate job.
    pub fn take_result(&self) -> Option<RigCalibration> {
        self.result.lock().unwrap_or_else(|e| e.into_inner()).take()
    }

    /// The raw and the finished mesh of a `scan.auto` finish.
    pub fn take_meshes(&self) -> Option<FinishedMeshes> {
        self.meshes.lock().unwrap_or_else(|e| e.into_inner()).take()
    }

    /// Marks a job that ended well as failed after all (`e`): the
    /// controller could not hand its result over.
    pub fn fail(&self, e: &ScanError) {
        let mut s = self.lock();
        s.state = RigJobState::Failed;
        s.error = Some(e.code().to_owned());
        s.error_detail = Some(e.to_string());
    }

    /// The controller finalised a stop-and-capture `scan.auto`: `n` stops
    /// are kept for the reconstruction and the export.
    pub fn set_kept_stops(&self, n: Option<u32>) {
        self.lock().kept_stops = n;
    }

    /// The job still uses the table (it is acquiring).
    pub fn holds_table(&self) -> bool {
        !self.table_free.load(Ordering::SeqCst)
    }
}

/// A running job.
#[derive(Debug)]
pub(crate) struct Job {
    pub shared: Arc<JobShared>,
    handle: JoinHandle<()>,
}

impl Job {
    /// The thread ended.
    pub fn finished(&self) -> bool {
        self.handle.is_finished()
    }

    /// Waits for the thread (after [`JobShared::cancel`] or once finished).
    pub fn join(self) -> Arc<JobShared> {
        // the thread catches its own errors; a panic leaves the status as is
        let _ = self.handle.join();
        let mut s = self.shared.lock();
        if s.state == RigJobState::Running || s.state == RigJobState::Paused {
            s.state = RigJobState::Failed;
            s.error = Some("device".into());
            s.error_detail = Some("the rig job thread panicked".into());
        }
        drop(s);
        self.shared
    }
}

/// What a job works with.
struct Ctx {
    table: TableHandle,
    /// Whether the link to the table is up, when it can drop.
    table_link: Option<LinkProbe>,
    link: RigLink,
    job: Arc<JobShared>,
}

/// Absolute target of a move.
#[derive(Clone, Copy)]
enum Target {
    Rotate(f32),
    Tilt(f32),
}

impl Ctx {
    fn cancelled(&self) -> bool {
        self.job.cancelled()
    }

    fn check(&self) -> Result<()> {
        if self.cancelled() {
            Err(ScanError::Cancelled)
        } else {
            Ok(())
        }
    }

    /// Waits until the table stands still on every axis: a manual move or
    /// a homing may still be under way when a job starts (starting the
    /// controller's job stops only the watch of that motion), and a stop
    /// taken while the table turns ruins the registration of its pair.
    fn wait_still(&self) -> Result<()> {
        self.on_table(|t| self.settle_axes(t))
    }

    /// Waits for every axis of `t` to stand still.
    fn settle_axes(&self, t: &mut dyn Turntable) -> dc_turntable::Result<()> {
        for axis in t.axes() {
            t.wait_settled(axis.kind, rotation_timeout(360.0, None), &|| {
                self.cancelled()
            })?;
        }
        Ok(())
    }

    /// Runs `f` on the table; when the link to the table dropped,
    /// reconnects it and runs `f` again, so `f` must re-read where the
    /// table is rather than assume it.
    fn on_table<T>(
        &self,
        mut f: impl FnMut(&mut dyn Turntable) -> dc_turntable::Result<T>,
    ) -> Result<T> {
        loop {
            let r = f(lock(&self.table).as_mut());
            match r {
                Err(e) if !self.cancelled() && self.lost_link(&e) => self.reconnect(e)?,
                r => return Ok(r?),
            }
        }
    }

    /// `e` comes from a dropped link: a disconnection, or a protocol error
    /// or a timeout while the link drops (it shows only after the
    /// supervision timeout of the radio, [`LINK_LOSS_GRACE`]).
    fn lost_link(&self, e: &TurntableError) -> bool {
        if e.link_lost() {
            return true;
        }
        let Some(up) = &self.table_link else {
            return false;
        };
        if !up() {
            return true;
        }
        if !matches!(e, TurntableError::Protocol(_) | TurntableError::Timeout(_)) {
            return false;
        }
        let end = Instant::now() + LINK_LOSS_GRACE;
        while Instant::now() < end && !self.cancelled() {
            std::thread::sleep(Duration::from_millis(50));
            if !up() {
                return true;
            }
        }
        false
    }

    /// The link dropped (`cause`): reconnects the table and waits for it to
    /// stand still (the move under way at the drop goes on), again after
    /// every new drop meanwhile.
    ///
    /// # Errors
    /// [`ScanError::Cancelled`]; `ble-disconnected`
    /// ([`TurntableError::Reconnect`]) when the link does not come back
    /// within [`RECONNECT_TIMEOUT`] or dropped more than
    /// [`MAX_RECONNECTS`] times.
    fn reconnect(&self, mut cause: TurntableError) -> Result<()> {
        loop {
            if self.job.lock().reconnects >= MAX_RECONNECTS {
                return Err(TurntableError::Reconnect(format!(
                    "the link dropped more than {MAX_RECONNECTS} times ({cause})"
                ))
                .into());
            }
            self.job.lock().reconnecting = true;
            let mut t = lock(&self.table);
            let back = t.reconnect(RECONNECT_TIMEOUT, &|| self.cancelled());
            self.job.lock().reconnecting = false;
            match back {
                Ok(()) => self.job.lock().reconnects += 1,
                Err(TurntableError::Cancelled) => return Err(ScanError::Cancelled),
                Err(e) => {
                    return Err(TurntableError::Reconnect(format!("{e} (dropped: {cause})")).into());
                }
            }
            match self.settle_axes(t.as_mut()) {
                Ok(()) => return Ok(()),
                Err(e) => {
                    drop(t);
                    if self.cancelled() || !self.lost_link(&e) {
                        return Err(e.into());
                    }
                    cause = e;
                }
            }
        }
    }

    /// Starts the next step; `tilt` updates the tilt of the pass.
    fn step(&self, tilt: Option<f32>) {
        let mut s = self.job.lock();
        s.step = (s.step + 1).min(s.steps);
        if tilt.is_some() {
            s.tilt_deg = tilt;
        }
    }

    /// Starts the next step, a phase of the finish.
    fn enter(&self, phase: RigJobPhase) {
        let mut s = self.job.lock();
        s.step = (s.step + 1).min(s.steps);
        s.phase = Some(phase);
    }

    /// Waits while the user paused the scan.
    fn wait_unpaused(&self) -> Result<()> {
        if !self.link.paused() {
            return Ok(());
        }
        self.job.lock().state = RigJobState::Paused;
        while self.link.paused() {
            self.check()?;
            std::thread::sleep(Duration::from_millis(10));
        }
        self.job.lock().state = RigJobState::Running;
        Ok(())
    }

    /// Sleeps `d`, interrupted by a cancel (and, when `pausable`, extended
    /// by a pause).
    fn sleep(&self, d: Duration, pausable: bool) -> Result<()> {
        let end = Instant::now() + d;
        while Instant::now() < end {
            self.check()?;
            if pausable {
                self.wait_unpaused()?;
            }
            std::thread::sleep(Duration::from_millis(5).min(end - Instant::now()));
        }
        Ok(())
    }

    fn rotation(&self) -> Result<f32> {
        self.on_table(|t| t.position(AxisKind::Rotate))?
            .ok_or_else(|| ScanError::Calibration("the turntable reports no rotation".into()))
    }

    /// Sets the rotation speed, and the tilt speed of the jobs when `tilt`.
    fn set_speeds(&self, rotate_deg_per_s: f32, tilt: bool) -> Result<()> {
        self.on_table(|t| {
            t.set_speed(AxisKind::Rotate, rotate_deg_per_s)?;
            if tilt {
                t.set_speed(AxisKind::Tilt, TILT_DEG_PER_S)?;
            }
            Ok(())
        })
    }

    /// Moves to `target` and waits for the end of the motion, each attempt
    /// within `timeout`. When `pausable`, a user pause stops the table and
    /// the rest of the motion is sent again on resume; after a dropped
    /// link it is sent again from where the table stands.
    fn run_move(&self, target: Target, timeout: Duration, pausable: bool) -> Result<()> {
        loop {
            if pausable {
                self.wait_unpaused()?;
            }
            self.check()?;
            let interrupted = || self.cancelled() || (pausable && self.link.paused());
            let arrived = self.on_table(|t| {
                let axis = match target {
                    Target::Rotate(to) => {
                        let now = t.position(AxisKind::Rotate)?.unwrap_or(to);
                        if (to - now).abs() > 0.05 {
                            t.move_by(AxisKind::Rotate, to - now)?;
                        }
                        AxisKind::Rotate
                    }
                    Target::Tilt(to) => {
                        t.move_to(AxisKind::Tilt, to)?;
                        AxisKind::Tilt
                    }
                };
                match t.wait_settled(axis, timeout, &interrupted) {
                    Ok(()) => Ok(true),
                    // paused: stop now, send the rest after the resume
                    Err(TurntableError::Cancelled) if !self.cancelled() => t.stop().map(|()| false),
                    Err(e) => Err(e),
                }
            })?;
            if arrived {
                return Ok(());
            }
        }
    }

    /// Tilts the table back to 0 and waits for it: a scan leaves it flat.
    /// Best effort and not cancellable (it also runs after a cancel or a
    /// failure): a table without a tilt axis or a link that is down is left
    /// as it is.
    fn tilt_home(&self) {
        let mut t = lock(&self.table);
        if t.move_to(AxisKind::Tilt, 0.0).is_ok() {
            let _ = t.wait_settled(AxisKind::Tilt, TILT_TIMEOUT, &|| false);
        }
    }

    /// Captures the stop at `angles` from the frames after the settle.
    fn capture(&self, angles: (f32, f32), frames: usize, fuse: bool) -> Result<StopOutcome> {
        let request = CaptureRequest {
            angles,
            frames,
            skip: 1,
            fuse,
        };
        let timeout = FRAME_TIMEOUT * (frames as u32 + 2);
        self.link.capture(request, timeout, &|| self.cancelled())
    }
}

/// The wait after a move: `requested`, and at least the device latency
/// plus [`SETTLE_MARGIN`] (the frames arriving meanwhile still show the
/// motion).
pub(crate) fn settle(requested: Duration, latency: Duration) -> Duration {
    if latency.is_zero() {
        requested
    } else {
        requested.max(latency + SETTLE_MARGIN)
    }
}

/// Timeout of a rotation by `deg` at `deg_per_s`: generous, since the
/// device may clamp the speed (never assumed faster than the slowest).
fn rotation_timeout(deg: f32, deg_per_s: Option<f32>) -> Duration {
    let speed = deg_per_s.map_or(SLOWEST_DEG_PER_S, |s| s.min(SLOWEST_DEG_PER_S));
    Duration::from_secs_f32(deg.abs() / speed.max(0.1) + 30.0)
}

/// Starts `body` on a job thread, once the table stands still. Whatever
/// happens the frames end up held out of the model; on an error or a
/// cancel the table is stopped.
fn spawn(
    kind: RigJobKind,
    steps: u32,
    table: TableHandle,
    link: RigLink,
    body: impl FnOnce(&Ctx) -> Result<()> + Send + 'static,
) -> Result<Job> {
    let shared = JobShared::new(kind, steps);
    let table_link = lock(&table).link();
    let ctx = Ctx {
        table,
        table_link,
        link,
        job: Arc::clone(&shared),
    };
    let handle = std::thread::Builder::new()
        .name(format!("dc-rig-{kind:?}").to_lowercase())
        .spawn(move || {
            let r = ctx.wait_still().and_then(|()| body(&ctx));
            ctx.link.set_hold(true);
            let mut s = ctx.job.lock();
            match r {
                Ok(()) => {
                    s.state = RigJobState::Done;
                    s.step = s.steps;
                }
                Err(e) => {
                    s.state = if matches!(e, ScanError::Cancelled) {
                        RigJobState::Cancelled
                    } else {
                        RigJobState::Failed
                    };
                    if s.state == RigJobState::Failed {
                        s.error = Some(e.code().to_owned());
                        s.error_detail = Some(e.to_string());
                    }
                    drop(s);
                    // best effort: the table may be the thing that failed;
                    // a table the job let go of is left alone
                    if ctx.job.holds_table() {
                        let mut t = lock(&ctx.table);
                        let _ = t.stop();
                        drop(t);
                        // and flat again, when the link is up
                        ctx.tilt_home();
                    }
                }
            }
        })
        .map_err(|e| ScanError::Device(format!("cannot start the rig job: {e}")))?;
    Ok(Job { shared, handle })
}

/// How `scan.auto` turns the table.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) enum AutoMode {
    /// Stop every `step_deg`, capture `frames` frames.
    Stops { step_deg: f32, frames: usize },
    /// Turn continuously, fusing the tracked frames.
    Continuous,
}

/// The finish of `scan.auto`.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct FinishPlan {
    /// Mesh resolution of the fine reconstruction (mm).
    pub resolution_mm: f32,
    /// Refine the poses of the stops first.
    pub refine: bool,
}

/// `scan.auto` plan.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct AutoPlan {
    pub tilts_deg: Vec<f32>,
    pub rotation_deg: f32,
    pub deg_per_s: Option<f32>,
    pub settle: Duration,
    pub has_tilt: bool,
    pub mode: AutoMode,
    /// Rebuild and finish the mesh at the end (stop-and-capture only).
    pub finish: Option<FinishPlan>,
}

impl AutoPlan {
    /// Stops of one pass, the first and the last included.
    fn stops_per_pass(&self) -> u32 {
        match self.mode {
            AutoMode::Stops { step_deg, .. } => {
                (self.rotation_deg.abs() / step_deg).ceil() as u32 + 1
            }
            AutoMode::Continuous => 1,
        }
    }

    /// Steps of the job: per pass one tilt, then one step per stop (or one
    /// rotation); then one per phase of the finish.
    pub fn steps(&self) -> u32 {
        let finish = if self.finish.is_some() {
            FINISH_STEPS
        } else {
            0
        };
        (1 + self.stops_per_pass()) * self.tilts_deg.len() as u32 + finish
    }
}

/// Starts `scan.auto` (the acquisition is running and held).
pub(crate) fn start_auto(table: TableHandle, link: RigLink, plan: AutoPlan) -> Result<Job> {
    spawn(RigJobKind::Auto, plan.steps(), table, link, move |ctx| {
        let acquired = match plan.mode {
            AutoMode::Stops { step_deg, frames } => auto_stops(ctx, &plan, step_deg, frames),
            AutoMode::Continuous => auto_continuous(ctx, &plan),
        };
        // the acquisition is over: the table goes back to 0 (the rotation
        // stays) before the surface is built; a failed or cancelled job
        // does the same when it ends
        if acquired.is_ok() && plan.has_tilt {
            ctx.tilt_home();
        }
        let quality = acquired?;
        let verdict = judge(&quality, plan.mode);
        ctx.job.lock().quality = Some(quality);
        verdict?;
        match &plan.finish {
            Some(f) => finish_scan(ctx, f),
            None => Ok(()),
        }
    })
}

/// The finish of a stop-and-capture `scan.auto`: the fine mesh from the
/// kept stops, then cleaned, closed and optimised; the reports go to the
/// status, the meshes to the controller.
fn finish_scan(ctx: &Ctx, plan: &FinishPlan) -> Result<()> {
    // everything is captured: the acquisition ends and the table is free
    let stops = ctx
        .link
        .end_acquisition(FRAME_TIMEOUT * 6, &|| ctx.cancelled())?;
    ctx.job.table_free.store(true, Ordering::SeqCst);
    let params = ReconstructParams {
        resolution_mm: plan.resolution_mm,
        refine: plan.refine,
        ..ReconstructParams::default()
    };
    let rec = reconstruct(
        &stops,
        &params,
        &|s| {
            ctx.enter(match s {
                ReconstructStage::Refine => RigJobPhase::Refine,
                ReconstructStage::Fuse => RigJobPhase::Fuse,
                ReconstructStage::Extract => RigJobPhase::Extract,
            })
        },
        &|| ctx.cancelled(),
    )?;
    drop(stops);
    ctx.job.lock().reconstruction = Some(rec.report.clone());
    let (finished, report) = finish_mesh(
        &rec.mesh,
        &FinishParams::default(),
        &|s| {
            ctx.enter(match s {
                FinishStage::Clean => RigJobPhase::Clean,
                FinishStage::Holes => RigJobPhase::FillHoles,
                FinishStage::Optimise => RigJobPhase::Optimise,
            })
        },
        &|| ctx.cancelled(),
    )?;
    ctx.job.lock().finish = Some(report);
    *ctx.job.meshes.lock().unwrap_or_else(|e| e.into_inner()) = Some((rec.mesh, finished));
    Ok(())
}

/// The verdict on a finished `scan.auto`: an error when unusable, a
/// warning in `quality` when weak.
fn judge(quality: &ScanQuality, mode: AutoMode) -> Result<()> {
    match mode {
        AutoMode::Stops { .. } => {
            let stops = quality.stops.max(1) as f32;
            if quality.fused == 0 || (quality.fused as f32) < MIN_FUSED_STOPS * stops {
                return Err(ScanError::ScanQuality(format!(
                    "{} of {} stops fused, {} only at the turntable pose",
                    quality.fused, quality.stops, quality.flagged_stops
                )));
            }
        }
        AutoMode::Continuous => {
            let fused = quality.fused as f32 / quality.frames.max(1) as f32;
            if quality.frames > 0 && fused < MIN_FUSED_FRACTION {
                return Err(ScanError::ScanQuality(format!(
                    "{} of {} frames fused",
                    quality.fused, quality.frames
                )));
            }
        }
    }
    Ok(())
}

/// The warning of a usable `quality`, if weak.
fn warning(quality: &ScanQuality, mode: AutoMode) -> Option<String> {
    let weak = match mode {
        AutoMode::Stops { .. } => {
            quality.mean_rms_mm.is_some_and(|r| r > WEAK_RMS_MM)
                || quality
                    .loop_error_deg
                    .iter()
                    .any(|e| e.abs() > WEAK_LOOP_DEG)
        }
        AutoMode::Continuous => {
            quality.frames > 0
                && (quality.fused as f32) < WEAK_FUSED_FRACTION * quality.frames as f32
        }
    };
    weak.then(|| WARN_SCAN_QUALITY_WEAK.to_owned())
}

/// Tilts to `tilt` when the table tilts, and waits for the scene to settle.
fn tilt_to(ctx: &Ctx, plan: &AutoPlan, tilt: f32) -> Result<()> {
    if plan.has_tilt {
        ctx.run_move(Target::Tilt(tilt), TILT_TIMEOUT, true)?;
    }
    Ok(())
}

/// Views registered after the world view that the first pass of a
/// stop-and-capture `scan.auto` judges against the rig calibration
/// ([`CalibrationCheck`]).
pub const CHECK_VIEWS: u32 = 6;
/// When at least this many of those views contradict the calibration, the
/// scan stops (`calibration-outdated`).
pub const CHECK_MIN_CONTRADICTING: u32 = 5;
/// A view whose registration was refused contradicts the calibration when
/// less than this fraction of its samples lies on the model at the
/// turntable pose (within the finest association gate, 0.5 mm).
pub const CHECK_MIN_PRIOR_FIT: f32 = 0.1;

/// Whether the rig calibration still fits the rig, judged on the first
/// views of a stop-and-capture `scan.auto` (the first pass, the views
/// registered after the world view). Each view is registered from the
/// turntable pose that the calibration predicts; it contradicts the
/// calibration when its registration was refused and almost none of it
/// lies on the model at that pose: the view is millimetres away from where
/// the calibration puts it. Measured on the user's rig:
///
/// * a calibration made before the turntable was turned on the desk and
///   moved (2026-11-20: axis 16 mm away, tilt reversed): every view
///   refused, 0.2-3% of each on the model at the turntable pose; the scan
///   went on and rejected 241 of its 245 views at the end;
/// * the same views with a calibration measured on the rig as it stood:
///   none refused;
/// * a fresh calibration with the field-of-view intrinsics (no factory
///   calibration, a non-metric scanner): every view refused but 17-42% of
///   each on the model, a usable scan;
/// * a scan whose object slid on the platter at the start of the pass: 2
///   of the first 6 contradict.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct CalibrationCheck {
    /// Views judged so far, at most [`CHECK_VIEWS`].
    views: u32,
    /// Of those, the views that contradict the calibration.
    contradicting: u32,
}

impl CalibrationCheck {
    /// Notes a view registered after the world view: `flagged` when its
    /// registration was refused, `prior_fit` the fraction of its samples
    /// on the model at the turntable pose (see `StopOutcome::prior_fit`).
    /// Views beyond the first [`CHECK_VIEWS`] are not judged.
    pub fn note(&mut self, flagged: bool, prior_fit: Option<f32>) {
        if self.views >= CHECK_VIEWS {
            return;
        }
        self.views += 1;
        if flagged && prior_fit.is_some_and(|f| f < CHECK_MIN_PRIOR_FIT) {
            self.contradicting += 1;
        }
    }

    /// The first [`CHECK_VIEWS`] views are judged and at least
    /// [`CHECK_MIN_CONTRADICTING`] of them contradict the calibration.
    pub fn outdated(&self) -> bool {
        self.views == CHECK_VIEWS && self.contradicting >= CHECK_MIN_CONTRADICTING
    }

    /// Technical detail of the verdict, for the pass at `tilt_deg`.
    pub fn detail(&self, tilt_deg: f32) -> String {
        format!(
            "{} of the first {} views of the pass at {tilt_deg}° are not where the calibration \
             puts them (registration refused, under {:.0}% of each on the model at the turntable \
             pose): the scanner or the turntable moved since it was measured",
            self.contradicting,
            self.views,
            CHECK_MIN_PRIOR_FIT * 100.0
        )
    }
}

fn auto_stops(ctx: &Ctx, plan: &AutoPlan, step_deg: f32, frames: usize) -> Result<ScanQuality> {
    ctx.set_speeds(plan.deg_per_s.unwrap_or(STOP_MOVE_DEG_PER_S), plan.has_tilt)?;
    let wait = settle(plan.settle, ctx.link.latency());
    let speed = plan.deg_per_s.unwrap_or(STOP_MOVE_DEG_PER_S);
    let timeout = rotation_timeout(step_deg, Some(speed));
    let sign = plan.rotation_deg.signum();
    let n = plan.stops_per_pass() - 1;
    let mut quality = ScanQuality::default();
    let mut rms = Vec::new();
    let mut check = CalibrationCheck::default();
    for (pass, &tilt) in plan.tilts_deg.iter().enumerate() {
        ctx.link.set_hold(true);
        ctx.step(Some(tilt));
        tilt_to(ctx, plan, tilt)?;
        let start = ctx.rotation()?;
        let mut first = None;
        for i in 0..=n {
            if i > 0 {
                let to = (i as f32 * step_deg).min(plan.rotation_deg.abs());
                ctx.run_move(Target::Rotate(start + sign * to), timeout, true)?;
            }
            ctx.sleep(wait, true)?;
            ctx.wait_unpaused()?;
            let angles = (ctx.rotation()?, if plan.has_tilt { tilt } else { 0.0 });
            let out = ctx.capture(angles, frames, true)?;
            ctx.step(None);
            quality.stops += 1;
            if let Some(k) = out.index {
                quality.fused += 1;
                // the first pass checks the calibration on the views
                // registered after the world view (the first fused stop)
                if pass == 0 && k > 0 {
                    check.note(out.flagged, out.prior_fit);
                }
                first.get_or_insert(k);
            }
            if out.flagged {
                quality.flagged_stops += 1;
            }
            rms.extend(out.rms_mm);
            ctx.job.lock().quality = Some(quality.clone());
            if check.outdated() {
                return Err(ScanError::CalibrationOutdated(check.detail(tilt)));
            }
        }
        if let Some(first) = first
            && plan.rotation_deg.abs() >= 359.0
        {
            let closed = ctx
                .link
                .close_loop(first, FRAME_TIMEOUT * 6, &|| ctx.cancelled())?;
            quality.loop_error_deg.extend(closed.error_deg);
        }
    }
    quality.mean_rms_mm = (!rms.is_empty()).then(|| rms.iter().sum::<f32>() / rms.len() as f32);
    quality.warning = warning(&quality, plan.mode);
    Ok(quality)
}

fn auto_continuous(ctx: &Ctx, plan: &AutoPlan) -> Result<ScanQuality> {
    let speed = match plan.deg_per_s {
        Some(s) => s,
        None => {
            // about two degrees per frame at the frame rate of the device
            let deadline = Instant::now() + Duration::from_secs(5);
            while ctx.link.fps() <= 0.0 && Instant::now() < deadline {
                ctx.sleep(Duration::from_millis(50), true)?;
            }
            let fps = ctx.link.fps();
            let fps = if fps > 0.0 { fps } else { 2.0 };
            (CONTINUOUS_DEG_PER_FRAME * fps).clamp(CONTINUOUS_DEG_PER_S.0, CONTINUOUS_DEG_PER_S.1)
        }
    };
    ctx.set_speeds(speed, plan.has_tilt)?;
    let wait = settle(plan.settle, ctx.link.latency());
    let timeout = rotation_timeout(plan.rotation_deg, Some(speed));
    let mut quality = ScanQuality::default();
    for &tilt in &plan.tilts_deg {
        ctx.link.set_hold(true);
        ctx.step(Some(tilt));
        tilt_to(ctx, plan, tilt)?;
        ctx.sleep(wait, true)?;
        ctx.wait_unpaused()?;
        let start = ctx.rotation()?;
        let (frames, fused) = (ctx.link.frames(), ctx.link.fused_frames());
        ctx.link.set_hold(false);
        ctx.step(None);
        ctx.run_move(Target::Rotate(start + plan.rotation_deg), timeout, true)?;
        ctx.link.set_hold(true);
        quality.frames += ctx.link.frames() - frames;
        quality.fused += ctx.link.fused_frames() - fused;
        ctx.job.lock().quality = Some(quality.clone());
    }
    quality.warning = warning(&quality, plan.mode);
    Ok(quality)
}

/// `turntable.calibrate` plan.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct CalibrationPlan {
    pub steps: u32,
    pub step_deg: f32,
    /// Tilt amplitude; `None` for a table without tilt.
    pub tilt_deg: Option<f32>,
    /// Frames per stop.
    pub frames: usize,
    /// Requested wait after each move.
    pub settle: Duration,
    pub scanner_serial: Option<String>,
    pub turntable_id: String,
    /// Where the result is saved (`None`: kept in memory only).
    pub dir: Option<std::path::PathBuf>,
}

impl CalibrationPlan {
    /// Steps of the job: tilt to zero, `steps + 1` stops, tilt stops and
    /// the tilt back.
    pub fn steps(&self) -> u32 {
        let tilt = u32::from(self.tilt_deg.is_some());
        tilt + self.steps + 1 + 3 * tilt
    }
}

/// Starts `turntable.calibrate` (the acquisition is running and held,
/// without region of interest).
pub(crate) fn start_calibration(
    table: TableHandle,
    link: RigLink,
    plan: CalibrationPlan,
) -> Result<Job> {
    spawn(
        RigJobKind::Calibrate,
        plan.steps(),
        table,
        link,
        move |ctx| {
            let c = calibrate(ctx, &plan)?;
            if let Some(dir) = &plan.dir {
                save_calibration(dir, &c)?;
            }
            *ctx.job.result.lock().unwrap_or_else(|e| e.into_inner()) = Some(c);
            Ok(())
        },
    )
}

fn calibrate(ctx: &Ctx, plan: &CalibrationPlan) -> Result<RigCalibration> {
    ctx.set_speeds(STOP_MOVE_DEG_PER_S, plan.tilt_deg.is_some())?;
    ctx.link.set_hold(true);
    if plan.tilt_deg.is_some() {
        ctx.step(Some(0.0));
        ctx.run_move(Target::Tilt(0.0), TILT_TIMEOUT, false)?;
    }
    let wait = settle(plan.settle, ctx.link.latency());
    let step_timeout = rotation_timeout(plan.step_deg, Some(STOP_MOVE_DEG_PER_S));

    // settled stops around the axis
    let base = ctx.rotation()?;
    let mut rotation = Vec::with_capacity(plan.steps as usize + 1);
    let mut intrinsics = None;
    for k in 0..=plan.steps {
        if k > 0 {
            ctx.run_move(
                Target::Rotate(base + k as f32 * plan.step_deg),
                step_timeout,
                false,
            )?;
        }
        ctx.sleep(wait, false)?;
        let angle = ctx.rotation()?;
        let out = ctx.capture((angle, 0.0), plan.frames, false)?;
        intrinsics = Some(out.intrinsics);
        rotation.push(CalibrationStop {
            rotate_deg: angle,
            tilt_deg: 0.0,
            depth: out.depth,
        });
        ctx.step(None);
    }
    // the up of the IMU with the platter level (the scanner never moves:
    // smoothed over every frame of the turn)
    let imu_up = ctx.link.imu_up();
    // tilt stops at the last angle
    let mut tilt = Vec::new();
    if let Some(t) = plan.tilt_deg {
        let angle = rotation.last().map_or(base, |s| s.rotate_deg);
        for target in [t, -t] {
            ctx.step(Some(target));
            ctx.run_move(Target::Tilt(target), TILT_TIMEOUT, false)?;
            ctx.sleep(wait, false)?;
            let out = ctx.capture((angle, target), plan.frames, false)?;
            tilt.push(CalibrationStop {
                rotate_deg: angle,
                tilt_deg: target,
                depth: out.depth,
            });
        }
        ctx.step(Some(0.0));
        ctx.run_move(Target::Tilt(0.0), TILT_TIMEOUT, false)?;
    }
    let Some(intrinsics) = intrinsics else {
        return Err(ScanError::Calibration("no stop captured".into()));
    };
    let outcome = match calib::measure(&intrinsics, &rotation, &tilt) {
        Ok(o) => o,
        Err((e, report)) => {
            ctx.job.lock().calibration = Some(report);
            return Err(e);
        }
    };
    ctx.job.lock().calibration = Some(outcome.report.clone());
    let axis = outcome.axis;
    Ok(RigCalibration {
        axis_point: outcome.platter_point.into(),
        axis_dir: axis.dir.into(),
        platter_up: outcome.up.into(),
        tilt_axis_point: outcome.tilt.map(|a| a.point.into()),
        tilt_axis_dir: outcome.tilt.map(|a| a.dir.into()),
        scanner_serial: plan.scanner_serial.clone(),
        turntable_id: plan.turntable_id.clone(),
        created: super::rfc3339(SystemTime::now()),
        fit_rms_deg: axis.rms_deg,
        fit_rms_mm: axis.rms_mm,
        angle_gain: outcome.angle_gain,
        fit_pairs: axis.pairs as u32,
        imu_up: imu_up.map(Into::into),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn plans_count_their_steps() {
        let plan = AutoPlan {
            tilts_deg: vec![-20.0, 0.0, 20.0],
            rotation_deg: 360.0,
            deg_per_s: None,
            settle: Duration::ZERO,
            has_tilt: true,
            mode: AutoMode::Continuous,
            finish: None,
        };
        assert_eq!(plan.steps(), 6);
        let stops = AutoPlan {
            mode: AutoMode::Stops {
                step_deg: 10.0,
                frames: 3,
            },
            ..plan
        };
        assert_eq!(stops.stops_per_pass(), 37);
        assert_eq!(stops.steps(), 3 * 38);
        let finished = AutoPlan {
            finish: Some(FinishPlan {
                resolution_mm: 0.1,
                refine: true,
            }),
            ..stops.clone()
        };
        assert_eq!(finished.steps(), 3 * 38 + FINISH_STEPS);
        let c = CalibrationPlan {
            steps: 24,
            step_deg: 15.0,
            tilt_deg: Some(15.0),
            frames: 3,
            settle: Duration::ZERO,
            scanner_serial: None,
            turntable_id: "t".into(),
            dir: None,
        };
        assert_eq!(c.steps(), 1 + 25 + 3);
        assert!(rotation_timeout(360.0, Some(30.0)) >= Duration::from_secs(42));
        assert!(rotation_timeout(360.0, None) >= Duration::from_secs(120));
        assert_eq!(
            settle(Duration::from_millis(500), Duration::from_millis(800)),
            Duration::from_millis(1200)
        );
        assert_eq!(settle(Duration::ZERO, Duration::ZERO), Duration::ZERO);
    }

    #[test]
    fn quality_fails_or_warns_below_the_thresholds() {
        let stops = AutoMode::Stops {
            step_deg: 10.0,
            frames: 3,
        };
        let mut q = ScanQuality {
            stops: 37,
            fused: 37,
            ..ScanQuality::default()
        };
        assert!(judge(&q, stops).is_ok());
        assert_eq!(warning(&q, stops), None);
        q.flagged_stops = 10;
        q.mean_rms_mm = Some(0.28);
        assert_eq!(warning(&q, stops), None);
        q.mean_rms_mm = Some(0.5);
        assert_eq!(warning(&q, stops).as_deref(), Some(WARN_SCAN_QUALITY_WEAK));
        q.mean_rms_mm = Some(0.28);
        q.loop_error_deg = vec![0.06, -1.4];
        assert_eq!(warning(&q, stops).as_deref(), Some(WARN_SCAN_QUALITY_WEAK));
        q.flagged_stops = 30;
        assert!(judge(&q, stops).is_ok());
        q.fused = 10;
        assert_eq!(judge(&q, stops).unwrap_err().code(), "scan-quality-low");
        let q = ScanQuality {
            frames: 245,
            fused: 21,
            ..ScanQuality::default()
        };
        assert_eq!(
            judge(&q, AutoMode::Continuous).unwrap_err().code(),
            "scan-quality-low"
        );
    }

    /// The check over `views` (flagged, fit at the turntable pose).
    fn checked(views: &[(bool, Option<f32>)]) -> CalibrationCheck {
        let mut c = CalibrationCheck::default();
        for &(flagged, fit) in views {
            c.note(flagged, fit);
        }
        c
    }

    #[test]
    fn the_calibration_check_needs_its_first_views_far_from_the_turntable_pose() {
        // the first views of the Windows scan of 2026-11-20 (calibration
        // made before the turntable was turned and moved)
        let moved = [0.03, 0.026, 0.013, 0.002, 0.003, 0.015].map(|f| (true, Some(f)));
        let c = checked(&moved);
        assert!(c.outdated(), "{c:?}");
        assert!(
            c.detail(-20.0).contains("6 of the first 6"),
            "{}",
            c.detail(-20.0)
        );
        // judged only once its window is full
        assert!(!checked(&moved[..5]).outdated());
        // a fitting calibration: views registered (not flagged)
        assert!(!checked(&[(false, None); 6]).outdated());
        // a fresh calibration on a non-metric scanner: views refused, yet
        // well on the model at the turntable pose
        let fov = [0.421, 0.302, 0.26, 0.224, 0.201, 0.174].map(|f| (true, Some(f)));
        assert!(!checked(&fov).outdated());
        // an object that slid at the start of the pass, then held
        let slid = [
            (true, Some(0.002)),
            (true, Some(0.029)),
            (true, Some(0.24)),
            (false, None),
            (false, None),
            (false, None),
        ];
        assert!(!checked(&slid).outdated());
        // four views far off are not enough, five are
        let mut four = moved;
        four[1] = (false, None);
        four[4] = (true, Some(0.5));
        assert!(!checked(&four).outdated());
        let mut five = moved;
        five[2] = (false, None);
        assert!(checked(&five).outdated());
        // a refused view with no samples is no evidence
        let mut empty = moved;
        empty[0] = (true, None);
        empty[3] = (true, None);
        assert!(!checked(&empty).outdated());
        // the views after the first ones are not judged
        let mut late = vec![(false, None); 6];
        late.extend(moved);
        assert!(!checked(&late).outdated());
    }
}
