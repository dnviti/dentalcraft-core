//! Rig jobs: `scan.auto` and `turntable.calibrate`, each on its own thread.
//!
//! A job owns nothing of the controller: it moves the table through the
//! shared [`TableHandle`] and steers the acquisition through the
//! [`RigLink`] (hold the frames out of the model, read poses, grab depth
//! images). The controller polls [`JobShared::status`] and, once the thread
//! ended, finalises it (see `rig::control`).
//!
//! **`scan.auto`**, for each tilt: hold the fusion, tilt, wait `settle`,
//! release the fusion, rotate by `rotation_deg`, hold again. A user pause
//! (`scan.pause`) stops the table; `scan.resume` sends the rest of the
//! motion again.
//!
//! **`turntable.calibrate`**, with a static object on the platter:
//!
//! 1. *Background*: the frames are held out of the model while the table
//!    turns in half steps; the depth images at each half step give the
//!    static background ([`Background`]), removed from the frames from then
//!    on, so the enclosure does not pull the tracking.
//! 2. *Rotation*: the frames are fused and tracked while the table turns
//!    `steps` times by `step_deg`; the tracked pose after each step and the
//!    table angle give the rotation axis ([`fit_axis`]).
//! 3. *Tilt* (dual-axis tables): poses at `+t` and `-t` give the tilt axis.
//! 4. *Platter*: the lowest heights of the fused points along the axis give
//!    the platter plane, where the axis point is put.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, MutexGuard};
use std::thread::JoinHandle;
use std::time::{Duration, Instant, SystemTime};

use dc_turntable::{AxisKind, Turntable, TurntableError};
use glam::Vec3;

use super::fit::{AxisFit, fit_axis};
use super::{Background, RigCalibration, RigJobKind, RigJobState, RigJobStatus, save_calibration};
use crate::session::RigLink;
use crate::{Result, ScanError};

/// The opened table, shared by the controller, the jobs and the settle
/// watcher.
pub(crate) type TableHandle = Arc<Mutex<Box<dyn Turntable>>>;

/// Locks the table (a panicked holder does not poison it for good).
pub(crate) fn lock(t: &TableHandle) -> MutexGuard<'_, Box<dyn Turntable>> {
    t.lock().unwrap_or_else(|e| e.into_inner())
}

/// Longest wait for a frame, a pose or a depth image.
const FRAME_TIMEOUT: Duration = Duration::from_secs(10);
/// Longest tilt.
const TILT_TIMEOUT: Duration = Duration::from_secs(60);
/// Slowest rotation assumed for the timeouts (deg/s; the Revopoint table
/// turns at 4 deg/s at its slowest).
const SLOWEST_DEG_PER_S: f32 = 3.0;
/// Rotation speed of the calibration (deg/s): slow enough for the
/// tracking to follow without a motion prior.
pub(crate) const CALIBRATION_DEG_PER_S: f32 = 30.0;
/// Rotation speed of the background pass (deg/s): nothing is tracked, so
/// as fast as the table goes.
const BACKGROUND_DEG_PER_S: f32 = 90.0;
/// Tilt speed of the calibration (deg/s).
const CALIBRATION_TILT_DEG_PER_S: f32 = 15.0;
/// Tilt amplitude of the calibration (degrees), at most half the range.
pub(crate) const CALIBRATION_TILT_DEG: f32 = 15.0;
/// Two depths closer than this are the same background (mm).
const BACKGROUND_TOL_MM: f32 = 1.5;
/// Frames tried for a tracked pose after a step.
const POSE_TRIES: usize = 5;

/// Status and cancellation of a job, shared with the controller.
#[derive(Debug)]
pub(crate) struct JobShared {
    cancel: AtomicBool,
    status: Mutex<RigJobStatus>,
    result: Mutex<Option<RigCalibration>>,
}

impl JobShared {
    fn new(kind: RigJobKind, steps: u32) -> Arc<Self> {
        Arc::new(Self {
            cancel: AtomicBool::new(false),
            status: Mutex::new(RigJobStatus {
                kind,
                state: RigJobState::Running,
                step: 0,
                steps,
                tilt_deg: None,
                error: None,
                error_detail: None,
            }),
            result: Mutex::new(None),
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

    /// Starts the next step; `tilt` updates the tilt of the pass.
    fn step(&self, tilt: Option<f32>) {
        let mut s = self.job.lock();
        s.step = (s.step + 1).min(s.steps);
        if tilt.is_some() {
            s.tilt_deg = tilt;
        }
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
        lock(&self.table)
            .position(AxisKind::Rotate)?
            .ok_or_else(|| ScanError::Calibration("the turntable reports no rotation".into()))
    }

    /// Moves to `target` and waits for the end of the motion. When
    /// `pausable`, a user pause stops the table and the rest of the motion
    /// is sent again on resume.
    fn run_move(&self, target: Target, timeout: Duration, pausable: bool) -> Result<()> {
        let deadline = Instant::now() + timeout;
        loop {
            if pausable {
                self.wait_unpaused()?;
            }
            self.check()?;
            let mut t = lock(&self.table);
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
            let interrupted = || self.cancelled() || (pausable && self.link.paused());
            let left = deadline.saturating_duration_since(Instant::now());
            match t.wait_settled(axis, left, &interrupted) {
                Ok(()) => return Ok(()),
                // paused: stop now, send the rest after the resume
                Err(TurntableError::Cancelled) if !self.cancelled() => t.stop()?,
                Err(e) => return Err(e.into()),
            }
        }
    }

    /// The tracked pose of a frame taken now.
    fn pose(&self, what: &str) -> Result<glam::Affine3A> {
        for _ in 0..POSE_TRIES {
            let t = self.link.next_pose(FRAME_TIMEOUT, &|| self.cancelled())?;
            if t.ok {
                return Ok(t.pose);
            }
        }
        Err(ScanError::Calibration(format!(
            "tracking lost at {what}: keep a static, textured object on the platter"
        )))
    }
}

/// Timeout of a rotation by `deg` at `deg_per_s`: generous, since the
/// device may clamp the speed (never assumed faster than the slowest).
fn rotation_timeout(deg: f32, deg_per_s: Option<f32>) -> Duration {
    let speed = deg_per_s.map_or(SLOWEST_DEG_PER_S, |s| s.min(SLOWEST_DEG_PER_S));
    Duration::from_secs_f32(deg.abs() / speed.max(0.1) + 30.0)
}

/// Starts `body` on a job thread. Whatever happens the frames end up held
/// out of the model; on an error or a cancel the table is stopped.
fn spawn(
    kind: RigJobKind,
    steps: u32,
    table: TableHandle,
    link: RigLink,
    body: impl FnOnce(&Ctx) -> Result<()> + Send + 'static,
) -> Result<Job> {
    let shared = JobShared::new(kind, steps);
    let ctx = Ctx {
        table,
        link,
        job: Arc::clone(&shared),
    };
    let handle = std::thread::Builder::new()
        .name(format!("dc-rig-{kind:?}").to_lowercase())
        .spawn(move || {
            let r = body(&ctx);
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
                    // best effort: the table may be the thing that failed
                    let _ = lock(&ctx.table).stop();
                }
            }
        })
        .map_err(|e| ScanError::Device(format!("cannot start the rig job: {e}")))?;
    Ok(Job { shared, handle })
}

/// `scan.auto` plan.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct AutoPlan {
    pub tilts_deg: Vec<f32>,
    pub rotation_deg: f32,
    pub deg_per_s: Option<f32>,
    pub settle: Duration,
    pub has_tilt: bool,
}

impl AutoPlan {
    /// Steps of the job: one tilt and one rotation per pass.
    pub fn steps(&self) -> u32 {
        2 * self.tilts_deg.len() as u32
    }
}

/// Starts `scan.auto` (the acquisition is running and held).
pub(crate) fn start_auto(table: TableHandle, link: RigLink, plan: AutoPlan) -> Result<Job> {
    spawn(RigJobKind::Auto, plan.steps(), table, link, move |ctx| {
        if let Some(s) = plan.deg_per_s {
            lock(&ctx.table).set_speed(AxisKind::Rotate, s)?;
        }
        let timeout = rotation_timeout(plan.rotation_deg, plan.deg_per_s);
        for &tilt in &plan.tilts_deg {
            ctx.link.set_hold(true);
            ctx.step(Some(tilt));
            if plan.has_tilt {
                ctx.run_move(Target::Tilt(tilt), TILT_TIMEOUT, true)?;
            }
            ctx.sleep(plan.settle, true)?;
            ctx.wait_unpaused()?;
            let start = ctx.rotation()?;
            ctx.link.set_hold(false);
            ctx.step(None);
            ctx.run_move(Target::Rotate(start + plan.rotation_deg), timeout, true)?;
            ctx.link.set_hold(true);
        }
        Ok(())
    })
}

/// `turntable.calibrate` plan.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct CalibrationPlan {
    pub steps: u32,
    pub step_deg: f32,
    /// Tilt amplitude; `None` for a table without tilt.
    pub tilt_deg: Option<f32>,
    pub roi_radius_mm: f32,
    pub scanner_serial: Option<String>,
    pub turntable_id: String,
    /// Where the result is saved (`None`: kept in memory only).
    pub dir: Option<std::path::PathBuf>,
}

impl CalibrationPlan {
    /// Steps of the job: tilt to zero, background half steps, measured
    /// steps, tilt moves.
    pub fn steps(&self) -> u32 {
        let tilt = u32::from(self.tilt_deg.is_some());
        tilt + 2 * self.steps + self.steps + 3 * tilt
    }
}

/// Starts `turntable.calibrate` (the acquisition is running and held, on a
/// fresh volume, without region of interest).
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
    let cancel = || ctx.cancelled();
    {
        let mut t = lock(&ctx.table);
        t.set_speed(AxisKind::Rotate, BACKGROUND_DEG_PER_S)?;
        if plan.tilt_deg.is_some() {
            t.set_speed(AxisKind::Tilt, CALIBRATION_TILT_DEG_PER_S)?;
        }
    }
    ctx.link.set_hold(true);
    if plan.tilt_deg.is_some() {
        ctx.step(Some(0.0));
        ctx.run_move(Target::Tilt(0.0), TILT_TIMEOUT, false)?;
    }
    let half = 0.5 * plan.step_deg;
    let step_timeout = rotation_timeout(plan.step_deg, Some(CALIBRATION_DEG_PER_S));

    // 1. background: depth images at every half step, nothing fused
    let start = ctx.rotation()?;
    let mut frames = Vec::with_capacity(2 * plan.steps as usize);
    for k in 1..=2 * plan.steps {
        frames.push(ctx.link.grab_depth(FRAME_TIMEOUT, &cancel)?);
        ctx.step(None);
        ctx.run_move(Target::Rotate(start + k as f32 * half), step_timeout, false)?;
    }
    let background = Background::from_frames(&frames, BACKGROUND_TOL_MM);
    drop(frames);
    ctx.link.set_background(Some(Arc::new(background)));

    // 2. rotation: fuse and track, one pose per step
    lock(&ctx.table).set_speed(AxisKind::Rotate, CALIBRATION_DEG_PER_S)?;
    ctx.link.set_hold(false);
    ctx.link.wait_fused(FRAME_TIMEOUT, &cancel)?;
    let base = ctx.rotation()?;
    let mut samples = vec![(base, ctx.pose("the start")?)];
    for k in 1..=plan.steps {
        ctx.step(None);
        ctx.run_move(
            Target::Rotate(base + k as f32 * plan.step_deg),
            step_timeout,
            false,
        )?;
        let angle = ctx.rotation()?;
        samples.push((angle, ctx.pose(&format!("rotation step {k}"))?));
    }
    let axis = fit_axis(&samples)?;
    // the platter comes from the cloud of the rotation: the tilt below is
    // only tracked (it uncovers enclosure the background never saw)
    ctx.link.set_hold(true);
    let cloud = ctx.link.next_preview(FRAME_TIMEOUT, &cancel)?.cloud;

    // 3. tilt: poses at +t and -t from the last pose at tilt 0
    let tilt_axis = match plan.tilt_deg {
        Some(t) => {
            let reference = samples[samples.len() - 1].1;
            let mut tilts = vec![(0.0, reference)];
            for target in [t, -t] {
                ctx.step(Some(target));
                ctx.run_move(Target::Tilt(target), TILT_TIMEOUT, false)?;
                tilts.push((target, ctx.pose(&format!("tilt {target}"))?));
            }
            ctx.step(Some(0.0));
            ctx.run_move(Target::Tilt(0.0), TILT_TIMEOUT, false)?;
            Some(fit_axis(&tilts)?)
        }
        None => None,
    };

    // 4. platter plane: the lowest dense layer of fused points
    let (point, up) = platter(&axis, &cloud.points, plan.roi_radius_mm)?;
    Ok(RigCalibration {
        axis_point: point.into(),
        axis_dir: axis.dir.into(),
        platter_up: up.into(),
        tilt_axis_point: tilt_axis.map(|a| a.point.into()),
        tilt_axis_dir: tilt_axis.map(|a| a.dir.into()),
        scanner_serial: plan.scanner_serial.clone(),
        turntable_id: plan.turntable_id.clone(),
        created: super::rfc3339(SystemTime::now()),
        fit_rms_deg: axis.rms_deg,
        fit_rms_mm: axis.rms_mm,
    })
}

/// Platter point on the axis and platter normal towards the camera, from
/// the heights along the axis of the `points` (camera frame) within
/// `radius_mm` of it: the lowest 0.5 mm layer holding at least a fifth of
/// the points of the densest layer (the base of the object, or the
/// platter), so a few stray points of the enclosure below do not count.
fn platter(axis: &AxisFit, points: &[Vec3], radius_mm: f32) -> Result<(Vec3, Vec3)> {
    const BIN_MM: f32 = 0.5;
    // the camera (origin) looks down on the platter
    let up = if (-axis.point).dot(axis.dir) >= 0.0 {
        axis.dir
    } else {
        -axis.dir
    };
    let heights: Vec<f32> = points
        .iter()
        .filter_map(|p| {
            let v = *p - axis.point;
            let h = v.dot(up);
            ((v - h * up).length_squared() <= radius_mm * radius_mm && h.is_finite()).then_some(h)
        })
        .collect();
    if heights.len() < 100 {
        return Err(ScanError::Calibration(
            "too few points near the axis: put an object on the platter".into(),
        ));
    }
    let low = heights.iter().copied().fold(f32::INFINITY, f32::min);
    let bin = |h: f32| ((h - low) / BIN_MM) as usize;
    let mut counts = vec![0usize; bin(heights.iter().copied().fold(low, f32::max)) + 1];
    for h in &heights {
        counts[bin(*h)] += 1;
    }
    let peak = counts.iter().copied().max().unwrap_or(0);
    let first = counts.iter().position(|c| 5 * c >= peak).unwrap_or(0);
    // the layer may straddle two bins: take the median around it
    let from = low + (first as f32 - 1.0) * BIN_MM;
    let mut layer: Vec<f32> = heights
        .into_iter()
        .filter(|h| (from..from + 3.0 * BIN_MM).contains(h))
        .collect();
    let mid = layer.len() / 2;
    let (_, h, _) = layer.select_nth_unstable_by(mid, f32::total_cmp);
    Ok((axis.point + *h * up, up))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn platter_is_the_lowest_dense_layer_on_the_camera_side() {
        let axis = AxisFit {
            point: Vec3::new(0.0, 0.0, 150.0),
            dir: Vec3::Z,
            rms_deg: 0.0,
            rms_mm: 0.0,
            pairs: 2,
        };
        // a disc 10 mm in front of the axis point (towards the camera), an
        // object on it, far points outside the radius and a few stray
        // enclosure points 25 mm under the disc
        let mut points = Vec::new();
        for i in 0..400 {
            let a = i as f32 * 0.1;
            points.push(Vec3::new(20.0 * a.cos(), 20.0 * a.sin(), 140.0));
            points.push(Vec3::new(5.0 * a.cos(), 5.0 * a.sin(), 130.0));
            points.push(Vec3::new(90.0, 0.0, 170.0));
            if i % 10 == 0 {
                points.push(Vec3::new(30.0, 0.0, 165.0));
            }
        }
        let (p, up) = platter(&axis, &points, 45.0).unwrap();
        assert_eq!(up, -Vec3::Z);
        assert!((p - Vec3::new(0.0, 0.0, 140.0)).length() < 1e-4, "{p}");
        assert!(platter(&axis, &points[..30], 45.0).is_err());
    }

    #[test]
    fn plans_count_their_steps() {
        let plan = AutoPlan {
            tilts_deg: vec![-20.0, 0.0, 20.0],
            rotation_deg: 360.0,
            deg_per_s: None,
            settle: Duration::ZERO,
            has_tilt: true,
        };
        assert_eq!(plan.steps(), 6);
        let c = CalibrationPlan {
            steps: 8,
            step_deg: 45.0,
            tilt_deg: Some(15.0),
            roi_radius_mm: 45.0,
            scanner_serial: None,
            turntable_id: "t".into(),
            dir: None,
        };
        assert_eq!(c.steps(), 1 + 16 + 8 + 3);
        assert!(rotation_timeout(360.0, Some(30.0)) >= Duration::from_secs(42));
        assert!(rotation_timeout(360.0, None) >= Duration::from_secs(120));
    }
}
