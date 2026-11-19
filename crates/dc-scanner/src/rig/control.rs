//! The `turntable.*` commands and `scan.auto` on the [`ScanController`].
//!
//! The controller keeps the opened table behind a [`TableHandle`] shared
//! with the job threads and with a settle watcher: `turntable.move` and
//! `turntable.home` return once the table acknowledged, and the watcher
//! waits for the end of the motion so `busy` clears by itself. Nothing here
//! holds the controller while a table moves.

use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::Duration;

use dc_turntable::sim::{self as table_sim, SimRig, SimTurntableBackend};
use dc_turntable::{
    AngleProbe, AxisInfo, AxisKind, LinkProbe, Turntable, TurntableBackend, TurntableError,
    TurntableInfo, axis_of,
};
use schemars::JsonSchema;
use serde::Deserialize;
use serde_json::{Value, json};

use super::jobs::{
    self, AutoMode, AutoPlan, CalibrationPlan, FinishPlan, Job, JobShared, TableHandle, lock,
};
use super::{
    AUTO_FRAMES_PER_STOP, AUTO_RESOLUTION_MM, AUTO_STEP_DEG, AUTO_TILTS_DEG,
    RECONSTRUCT_RESOLUTION_MM, RigCalibration, RigJobKind, RigJobState, RigJobStatus,
    TurntableStatus, calibration_json,
};
use crate::control::{
    NoParams, SCAN_AUTO, TURNTABLE_CALIBRATE, TURNTABLE_CALIBRATION, TURNTABLE_CONNECT,
    TURNTABLE_DEVICES, TURNTABLE_DISCONNECT, TURNTABLE_HOME, TURNTABLE_MOVE, TURNTABLE_SPEED,
    TURNTABLE_STATUS, TURNTABLE_STOP, parse,
};
use crate::session::ScanState;
use crate::{Result, ScanController, ScanEditor, ScanError};

/// Default discovery time of `turntable.devices` (seconds).
const DEFAULT_DISCOVERY_S: f32 = 3.0;
/// Longest discovery time (seconds).
const MAX_DISCOVERY_S: f32 = 30.0;
/// Longest wait of the settle watcher.
const WATCH_TIMEOUT: Duration = Duration::from_secs(600);

/// Parameters of `turntable.devices`.
#[derive(Debug, Default, Deserialize, JsonSchema)]
#[serde(default, deny_unknown_fields)]
pub(crate) struct DevicesParams {
    /// Bluetooth discovery time in seconds, 0 to 30 (default 3); 0 lists
    /// the simulated table only.
    timeout_s: Option<f32>,
}

/// Options of the simulated table (`turntable.connect` with `backend:
/// "sim"`).
#[derive(Debug, Default, Deserialize, JsonSchema)]
#[serde(default, deny_unknown_fields)]
pub(crate) struct SimTableParams {
    /// Complete motions at once when no simulated scanner paces them
    /// (default false: real time).
    fast: Option<bool>,
}

/// Parameters of `turntable.connect`.
#[derive(Debug, Default, Deserialize, JsonSchema)]
#[serde(default, deny_unknown_fields)]
pub(crate) struct ConnectParams {
    /// Backend id (`revopoint-ble`, `sim`). Default: the first table found
    /// over Bluetooth, else `sim`.
    backend: Option<String>,
    /// Device id as listed by `turntable.devices` (a Bluetooth address).
    device: Option<String>,
    /// Options of the simulated table; only with the `sim` backend.
    options: Option<SimTableParams>,
}

/// Parameters of `turntable.move`: exactly one of `by_deg` and `to_deg`.
#[derive(Debug, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub(crate) struct MoveParams {
    /// `rotate` or `tilt`.
    axis: AxisKind,
    /// Relative move in degrees (signed).
    by_deg: Option<f32>,
    /// Absolute target in degrees.
    to_deg: Option<f32>,
}

/// Parameters of `turntable.speed`.
#[derive(Debug, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub(crate) struct SpeedParams {
    /// `rotate` or `tilt`.
    axis: AxisKind,
    /// Speed in degrees per second (the table clamps it to its range).
    deg_per_s: f32,
}

/// Parameters of `turntable.calibrate`.
#[derive(Debug, Default, Deserialize, JsonSchema)]
#[serde(default, deny_unknown_fields)]
pub(crate) struct CalibrateParams {
    /// Rotation steps, 3 to 36 (default 24).
    steps: Option<u32>,
    /// Degrees per step, 5 to 120 (default 15); steps times step_deg must
    /// be 180 to 1080.
    step_deg: Option<f32>,
    /// Frames averaged (per-pixel median) at each stop, 1 to 10 (default 3).
    frames_per_stop: Option<u32>,
    /// Wait after each move in seconds, 0 to 30 (default 0.5; at least the
    /// scanner latency plus 0.4 s).
    settle_s: Option<f32>,
}

/// How `scan.auto` turns the table.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize, JsonSchema)]
#[serde(rename_all = "kebab-case")]
pub(crate) enum AutoModeParam {
    /// Stop every `step_deg`, capture, register from the turntable pose.
    StopAndCapture,
    /// Turn without stopping, tracking every frame.
    Continuous,
}

/// Parameters of `scan.auto`.
#[derive(Debug, Default, Deserialize, JsonSchema)]
#[serde(default, deny_unknown_fields)]
pub(crate) struct AutoParams {
    /// Tilts of the passes in degrees (default [-30, -15, 0, 15, 30]); one
    /// rotation per tilt.
    tilts_deg: Option<Vec<f32>>,
    /// Rotation of each pass in degrees (default 360).
    rotation_deg: Option<f32>,
    /// `stop-and-capture` (default with a calibration) or `continuous`
    /// (default without).
    mode: Option<AutoModeParam>,
    /// Stop-and-capture: degrees between stops, 2 to 45 (default 6).
    step_deg: Option<f32>,
    /// Stop-and-capture: frames averaged (per-pixel median) at each stop,
    /// 1 to 10 (default 3).
    frames_per_stop: Option<u32>,
    /// Rotation speed in degrees per second (default: 30 between stops;
    /// continuous, about 2 degrees per frame at the scanner's frame rate,
    /// 4 to 30).
    deg_per_s: Option<f32>,
    /// Wait after each move in seconds (default 0.5; at least the scanner
    /// latency plus 0.4 s).
    settle_s: Option<f32>,
    /// Run without a turntable calibration (default false): no region of
    /// interest and no motion prior, so only continuous and only for setups
    /// where the background is beyond the depth range.
    allow_uncalibrated: Option<bool>,
    /// Stop-and-capture: at the end, stop the acquisition, rebuild the mesh
    /// from the stops and finish it (clean, close the holes, optimise), so
    /// the scan editor holds the finished mesh (default true). `false`
    /// leaves the acquisition paused, without a mesh. Continuous scans keep
    /// no stops and are never finished.
    finish: Option<bool>,
    /// Mesh resolution of the finish (voxel of the fine reconstruction),
    /// 0.05 to 1 mm (default 0.1).
    resolution_mm: Option<f32>,
    /// Refine the poses of the stops before the fine fusion (default true).
    refine: Option<bool>,
}

/// The connected table.
struct Connected {
    table: TableHandle,
    info: TurntableInfo,
    axes: Vec<AxisInfo>,
    probe: Option<AngleProbe>,
    /// Whether its link is up, when it can drop.
    link: Option<LinkProbe>,
}

/// Waits for the end of a manual move without holding the controller.
struct Watcher {
    cancel: Arc<AtomicBool>,
    axes: Vec<AxisKind>,
    handle: JoinHandle<()>,
}

impl Watcher {
    fn start(table: &TableHandle, axes: Vec<AxisKind>) -> Result<Self> {
        let cancel = Arc::new(AtomicBool::new(false));
        let (t, c, a) = (Arc::clone(table), Arc::clone(&cancel), axes.clone());
        let handle = std::thread::Builder::new()
            .name("dc-turntable-watch".into())
            .spawn(move || {
                for axis in a {
                    let mut t = lock(&t);
                    // a failed wait only ends the watch: the next command reports
                    let _ = t.wait_settled(axis, WATCH_TIMEOUT, &|| c.load(Ordering::SeqCst));
                }
            })
            .map_err(|e| ScanError::Device(format!("cannot start the turntable watcher: {e}")))?;
        Ok(Self {
            cancel,
            axes,
            handle,
        })
    }

    /// Stops watching (the motion goes on) and returns the watched axes.
    fn stop(self) -> Vec<AxisKind> {
        self.cancel.store(true, Ordering::SeqCst);
        let _ = self.handle.join();
        self.axes
    }
}

/// Turntable side of the [`ScanController`].
pub(crate) struct RigState {
    sim: Arc<SimRig>,
    dir: Option<PathBuf>,
    table: Option<Connected>,
    watcher: Option<Watcher>,
    job: Option<(RigJobKind, Job)>,
    last_auto: Option<Arc<JobShared>>,
    last_calibration: Option<Arc<JobShared>>,
    calibration: Option<RigCalibration>,
    loaded: bool,
}

impl Default for RigState {
    fn default() -> Self {
        Self {
            sim: SimRig::new(),
            dir: super::default_rig_dir(),
            table: None,
            watcher: None,
            job: None,
            last_auto: None,
            last_calibration: None,
            calibration: None,
            loaded: false,
        }
    }
}

impl std::fmt::Debug for RigState {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("RigState")
            .field("table", &self.table.as_ref().map(|t| &t.info))
            .field("job", &self.job.as_ref().map(|j| j.0))
            .field("calibration", &self.calibration)
            .finish_non_exhaustive()
    }
}

impl RigState {
    /// The simulated rig shared by the simulated scanner and table.
    pub fn sim(&self) -> &Arc<SimRig> {
        &self.sim
    }

    fn job_running(&self) -> bool {
        self.job.as_ref().is_some_and(|(_, j)| !j.finished())
    }

    fn table(&self) -> Result<&Connected> {
        self.table
            .as_ref()
            .ok_or(ScanError::Turntable(TurntableError::NotConnected))
    }

    /// Loads the stored calibration once.
    fn ensure_loaded(&mut self) -> Result<()> {
        if !self.loaded {
            self.loaded = true;
            if let Some(dir) = &self.dir {
                self.calibration = super::load_calibration(dir)?;
            }
        }
        Ok(())
    }
}

/// `turntable.devices`: the tables of every backend and the state of each
/// backend. Discovery blocks for `timeout_s`; this needs no controller, so
/// a host can run it without locking one.
///
/// # Errors
/// [`ScanError::InvalidParams`].
pub fn turntable_devices(params: &Value) -> Result<Value> {
    let p: DevicesParams = parse(params)?;
    let secs = p.timeout_s.unwrap_or(DEFAULT_DISCOVERY_S);
    if !(0.0..=MAX_DISCOVERY_S).contains(&secs) {
        return Err(ScanError::InvalidParams(format!(
            "timeout_s {secs} not in [0, {MAX_DISCOVERY_S}]"
        )));
    }
    let timeout = Duration::from_secs_f32(secs);
    let mut devices = Vec::new();
    let mut states = Vec::new();
    for b in dc_turntable::backends() {
        match b.availability().and_then(|()| b.list(timeout)) {
            Ok(list) => {
                devices.extend(list);
                states.push(json!({ "id": b.id(), "available": true }));
            }
            Err(e) => states.push(json!({
                "id": b.id(),
                "available": false,
                "error_code": e.code(),
            })),
        }
    }
    Ok(json!({ "devices": devices, "backends": states }))
}

impl ScanController {
    /// Directory of the rig calibration file (default:
    /// [`super::default_rig_dir`]); the stored calibration is read again
    /// from it.
    pub fn set_rig_dir(&mut self, dir: impl Into<PathBuf>) {
        self.rig.dir = Some(dir.into());
        self.rig.loaded = false;
        self.rig.calibration = None;
    }

    /// The rig shared by the simulated scanner and the simulated table.
    pub fn sim_rig(&self) -> &Arc<SimRig> {
        self.rig.sim()
    }

    /// Same as `turntable.status`, without I/O: the angles are the live
    /// estimate of the table.
    pub fn turntable_status(&self) -> TurntableStatus {
        let calibration_job = self.calibration_job();
        let Some(c) = &self.rig.table else {
            return TurntableStatus {
                connected: false,
                info: None,
                axes: Vec::new(),
                rotate_deg: None,
                tilt_deg: None,
                busy: false,
                calibrated: false,
                calibration_job,
            };
        };
        let angles = c.probe.as_ref().map(|p| p()).unwrap_or_default();
        let watching = self
            .rig
            .watcher
            .as_ref()
            .is_some_and(|w| !w.handle.is_finished());
        TurntableStatus {
            connected: c.link.as_ref().is_none_or(|up| up()),
            info: Some(c.info.clone()),
            axes: c.axes.clone(),
            rotate_deg: angles.rotate_deg,
            tilt_deg: angles.tilt_deg,
            busy: self.rig.job_running() || watching || angles.moving,
            calibrated: self.session.rig_calibration().is_some(),
            calibration_job,
        }
    }

    /// Progress of the latest `scan.auto`.
    pub fn auto_status(&self) -> Option<RigJobStatus> {
        self.rig.last_auto.as_ref().map(|j| j.status())
    }

    /// Progress of the latest `turntable.calibrate`.
    pub fn calibration_job(&self) -> Option<RigJobStatus> {
        self.rig.last_calibration.as_ref().map(|j| j.status())
    }

    /// The stored rig calibration (read at `turntable.connect` and
    /// `turntable.calibration`, replaced by `turntable.calibrate`).
    pub fn calibration(&self) -> Option<RigCalibration> {
        self.rig.calibration.clone()
    }

    /// A rig job (`scan.auto`, `turntable.calibrate`) ended and is not
    /// finalised yet: until then the acquisition still runs held, as the
    /// job left it. Every command finalises it first; a host that only
    /// reads the controller calls [`ScanController::finish_rig_job`].
    pub fn rig_job_ended(&self) -> bool {
        self.rig.job.as_ref().is_some_and(|(_, j)| j.finished())
    }

    /// Finalises a rig job that ended, as any command does first: after a
    /// calibration the acquisition is reset and the new calibration
    /// applied, after `scan.auto` the acquisition goes on paused. Nothing
    /// happens while the job runs.
    pub fn finish_rig_job(&mut self) {
        self.reap_rig(false);
    }

    /// Finalises a finished job: after a calibration the acquisition is
    /// reset and the new calibration applied; after `scan.auto` the
    /// acquisition goes on paused. With `cancel`, a running job is
    /// cancelled first and the settle watcher stopped.
    pub(crate) fn reap_rig(&mut self, cancel: bool) {
        if cancel {
            if let Some((_, job)) = &self.rig.job {
                job.shared.cancel();
            }
            if let Some(w) = self.rig.watcher.take() {
                w.stop();
            }
        }
        if self
            .rig
            .watcher
            .as_ref()
            .is_some_and(|w| w.handle.is_finished())
            && let Some(w) = self.rig.watcher.take()
        {
            w.stop();
        }
        let done = self
            .rig
            .job
            .as_ref()
            .is_some_and(|(_, j)| cancel || j.finished());
        if !done {
            return;
        }
        let Some((kind, job)) = self.rig.job.take() else {
            return;
        };
        let shared = job.join();
        match kind {
            RigJobKind::Calibrate => {
                self.session.reset();
                if let Some(c) = shared.take_result() {
                    self.rig.calibration = Some(c);
                    self.rig.loaded = true;
                }
                self.apply_rig();
            }
            RigJobKind::Auto => {
                self.session.end_hold();
                // a finished scan: the acquisition stops, the editor gets
                // the cloud, the fine mesh and the finished mesh (two undo
                // steps)
                if let Some((raw, finished)) = shared.take_meshes() {
                    let handed = self.session.stop().and_then(|cloud| {
                        let mut editor = ScanEditor::new(cloud);
                        editor.replace_mesh(raw)?;
                        editor.replace_mesh(finished)?;
                        Ok(editor)
                    });
                    match handed {
                        Ok(editor) => self.editor = Some(editor),
                        Err(e) => shared.fail(&e),
                    }
                } else if Self::kept_partial(&shared.status()) {
                    // a stop-and-capture scan that failed or was cancelled
                    // keeps its stops: the acquisition stops, so they can be
                    // rebuilt (`scan.reconstruct`) and exported; the job's
                    // own error stays the one reported
                    if let Ok(cloud) = self.session.stop() {
                        self.editor = Some(ScanEditor::new(cloud));
                    }
                }
                let kept = self.session.kept_stops().map(|k| k.stops.len() as u32);
                shared.set_kept_stops(kept);
            }
        }
    }

    /// A `scan.auto` that ended without a finish (failed or cancelled)
    /// after capturing stops.
    fn kept_partial(s: &RigJobStatus) -> bool {
        matches!(s.state, RigJobState::Failed | RigJobState::Cancelled)
            && s.quality.as_ref().is_some_and(|q| q.stops > 0)
    }

    /// Gives the session the calibration matching the connected scanner
    /// and table, and the table's angle estimate; and, with or without a
    /// table, the orientation of the scanner's IMU from the stored
    /// calibration of this scanner.
    pub(crate) fn apply_rig(&mut self) {
        // an unreadable calibration file must not prevent a connection:
        // turntable.calibration reports it
        let _ = self.rig.ensure_loaded();
        let serial = self.session.status().device.and_then(|d| d.serial);
        let (calibration, probe) = match &self.rig.table {
            Some(c) => {
                let calibration = self
                    .rig
                    .calibration
                    .as_ref()
                    .filter(|cal| cal.matches(serial.as_deref(), &c.info.id))
                    .cloned();
                (calibration, c.probe.clone())
            }
            None => (None, None),
        };
        let imu = self
            .rig
            .calibration
            .as_ref()
            .filter(|cal| serial.is_some() && cal.scanner_serial == serial)
            .and_then(RigCalibration::camera_from_imu);
        self.session.set_rig(calibration, probe);
        self.session.set_imu_alignment(imu);
    }

    /// Runs a `turntable.*` command or `scan.auto`.
    pub(crate) fn execute_rig(&mut self, id: &str, params: &Value) -> Result<Value> {
        match id {
            TURNTABLE_DEVICES => turntable_devices(params),
            TURNTABLE_CONNECT => {
                let p: ConnectParams = parse(params)?;
                self.connect_table(&p)
            }
            TURNTABLE_DISCONNECT => {
                let NoParams {} = parse(params)?;
                self.reap_rig(true);
                self.rig.table = None;
                self.apply_rig();
                Ok(json!({}))
            }
            TURNTABLE_MOVE => {
                let p: MoveParams = parse(params)?;
                let (by, to) = match (p.by_deg, p.to_deg) {
                    (Some(b), None) => (Some(b), None),
                    (None, Some(t)) => (None, Some(t)),
                    _ => {
                        return Err(ScanError::InvalidParams(
                            "give exactly one of by_deg and to_deg".into(),
                        ));
                    }
                };
                self.manual(&[p.axis], |t| match (by, to) {
                    (Some(b), _) => t.move_by(p.axis, b),
                    (_, Some(to)) => t.move_to(p.axis, to),
                    _ => Ok(()),
                })?;
                self.status_json()
            }
            TURNTABLE_SPEED => {
                let p: SpeedParams = parse(params)?;
                self.manual(&[], |t| t.set_speed(p.axis, p.deg_per_s))?;
                self.status_json()
            }
            TURNTABLE_HOME => {
                let NoParams {} = parse(params)?;
                let axes: Vec<AxisKind> = self.rig.table()?.axes.iter().map(|a| a.kind).collect();
                self.manual(&axes, |t| t.home())?;
                self.status_json()
            }
            TURNTABLE_STOP => {
                let NoParams {} = parse(params)?;
                self.reap_rig(true);
                let c = self.rig.table()?;
                lock(&c.table).stop()?;
                self.status_json()
            }
            TURNTABLE_STATUS => {
                let NoParams {} = parse(params)?;
                self.status_json()
            }
            TURNTABLE_CALIBRATE => {
                let p: CalibrateParams = parse(params)?;
                self.calibrate(&p)
            }
            TURNTABLE_CALIBRATION => {
                let NoParams {} = parse(params)?;
                self.rig.ensure_loaded()?;
                Ok(calibration_json(self.rig.calibration.as_ref()))
            }
            SCAN_AUTO => {
                let p: AutoParams = parse(params)?;
                self.auto(&p)
            }
            _ => Err(ScanError::InvalidParams(format!(
                "unknown turntable command {id}"
            ))),
        }
    }

    fn status_json(&self) -> Result<Value> {
        serde_json::to_value(self.turntable_status()).map_err(|e| ScanError::Device(e.to_string()))
    }

    /// Runs a manual command on the table: refused during a job; the
    /// settle watcher is paused around it and then watches the axes it
    /// watched plus `moved`.
    fn manual(
        &mut self,
        moved: &[AxisKind],
        f: impl FnOnce(&mut dyn Turntable) -> dc_turntable::Result<()>,
    ) -> Result<()> {
        if self.rig.job_running() {
            return Err(TurntableError::Busy.into());
        }
        let table = Arc::clone(&self.rig.table()?.table);
        let mut axes = self
            .rig
            .watcher
            .take()
            .map(Watcher::stop)
            .unwrap_or_default();
        let result = f(lock(&table).as_mut());
        if result.is_ok() {
            for a in moved {
                if !axes.contains(a) {
                    axes.push(*a);
                }
            }
        }
        if !axes.is_empty() {
            self.rig.watcher = Some(Watcher::start(&table, axes)?);
        }
        Ok(result?)
    }

    fn connect_table(&mut self, p: &ConnectParams) -> Result<Value> {
        let fast = p.options.as_ref().and_then(|o| o.fast);
        let backend_id = match (&p.backend, fast) {
            (Some(b), _) => Some(b.as_str()),
            (None, Some(_)) => Some(table_sim::BACKEND_ID),
            (None, None) => None,
        };
        if fast.is_some() && backend_id != Some(table_sim::BACKEND_ID) {
            return Err(ScanError::InvalidParams(
                "options only apply to the sim backend".into(),
            ));
        }
        // the old table first: a simulated one must leave the rig
        self.reap_rig(true);
        self.rig.table = None;
        let sim = SimTurntableBackend::new(Arc::clone(&self.rig.sim), fast.unwrap_or(false));
        let open_backend = |id: &str| -> Result<Box<dyn TurntableBackend>> {
            if id == table_sim::BACKEND_ID {
                Ok(Box::new(sim.clone()))
            } else {
                Ok(dc_turntable::backend(id)?)
            }
        };
        let discovery = Duration::from_secs_f32(DEFAULT_DISCOVERY_S);
        let table: Box<dyn Turntable> = match (backend_id, p.device.as_deref()) {
            (Some(b), Some(d)) => open_backend(b)?.open(d)?,
            (Some(b), None) => {
                let b = open_backend(b)?;
                b.availability()?;
                let first =
                    b.list(discovery)?.into_iter().next().ok_or_else(|| {
                        TurntableError::NotFound(format!("{} (no table)", b.id()))
                    })?;
                b.open(&first.id)?
            }
            (None, Some(d)) if d == table_sim::DEVICE_ID => sim.open(d)?,
            (None, Some(d)) => {
                let mut last = TurntableError::NotFound(d.to_owned());
                let mut opened = None;
                for b in dc_turntable::backends()
                    .into_iter()
                    .filter(|b| b.id() != table_sim::BACKEND_ID)
                {
                    match b.open(d) {
                        Ok(t) => {
                            opened = Some(t);
                            break;
                        }
                        Err(e) => last = e,
                    }
                }
                opened.ok_or(last)?
            }
            (None, None) => {
                let mut found = None;
                for b in dc_turntable::backends()
                    .into_iter()
                    .filter(|b| b.id() != table_sim::BACKEND_ID)
                {
                    if b.availability().is_ok()
                        && let Ok(list) = b.list(discovery)
                        && let Some(d) = list.first()
                    {
                        found = Some(b.open(&d.id)?);
                        break;
                    }
                }
                match found {
                    Some(t) => t,
                    None => sim.open(table_sim::DEVICE_ID)?,
                }
            }
        };
        let info = table.info();
        let axes = table.axes();
        let probe = table.probe();
        let link = table.link();
        self.rig.table = Some(Connected {
            table: Arc::new(Mutex::new(table)),
            info: info.clone(),
            axes: axes.clone(),
            probe,
            link,
        });
        self.apply_rig();
        Ok(json!({
            "turntable": info,
            "axes": axes,
            "calibrated": self.session.rig_calibration().is_some(),
        }))
    }

    /// Checks that a job can start: a table, a scanner, no other job.
    fn job_prerequisites(&mut self) -> Result<(TableHandle, Vec<AxisInfo>, TurntableInfo)> {
        if self.rig.job_running() {
            return Err(TurntableError::Busy.into());
        }
        let c = self.rig.table()?;
        let out = (Arc::clone(&c.table), c.axes.clone(), c.info.clone());
        if self.session.status().state == ScanState::Idle {
            return Err(ScanError::NotConnected);
        }
        if let Some(w) = self.rig.watcher.take() {
            w.stop();
        }
        Ok(out)
    }

    fn auto(&mut self, p: &AutoParams) -> Result<Value> {
        let (table, axes, _) = self.job_prerequisites()?;
        if self.session.rig_calibration().is_none() && p.allow_uncalibrated != Some(true) {
            return Err(ScanError::RigUncalibrated);
        }
        let tilt_axis = axis_of(&axes, AxisKind::Tilt).ok();
        let tilts = p.tilts_deg.clone().unwrap_or_else(|| {
            if tilt_axis.is_some() {
                AUTO_TILTS_DEG.to_vec()
            } else {
                vec![0.0]
            }
        });
        if tilts.is_empty() || tilts.len() > 12 {
            return Err(ScanError::InvalidParams(
                "tilts_deg needs 1 to 12 tilts".into(),
            ));
        }
        let has_tilt = match axis_of(&axes, AxisKind::Tilt) {
            Ok(tilt) => {
                for t in &tilts {
                    tilt.check(*t)?;
                }
                true
            }
            Err(e) => {
                if tilts.iter().any(|t| *t != 0.0) {
                    return Err(e.into());
                }
                false
            }
        };
        let rotation_deg = p.rotation_deg.unwrap_or(360.0);
        if !(rotation_deg.is_finite() && rotation_deg != 0.0 && rotation_deg.abs() <= 1080.0) {
            return Err(ScanError::InvalidParams(format!(
                "rotation_deg {rotation_deg} not in [-1080, 1080] or zero"
            )));
        }
        if p.deg_per_s.is_some_and(|s| !(s.is_finite() && s > 0.0)) {
            return Err(ScanError::InvalidParams(
                "deg_per_s must be positive".into(),
            ));
        }
        let settle_s = p.settle_s.unwrap_or(0.5);
        if !(0.0..=30.0).contains(&settle_s) {
            return Err(ScanError::InvalidParams(format!(
                "settle_s {settle_s} not in [0, 30]"
            )));
        }
        let calibrated = self.session.rig_calibration().is_some();
        let mode = match p.mode {
            Some(AutoModeParam::Continuous) => AutoMode::Continuous,
            Some(AutoModeParam::StopAndCapture) if !calibrated => {
                return Err(ScanError::RigUncalibrated);
            }
            None if !calibrated => AutoMode::Continuous,
            _ => {
                let step_deg = p.step_deg.unwrap_or(AUTO_STEP_DEG);
                let frames = p.frames_per_stop.unwrap_or(AUTO_FRAMES_PER_STOP);
                if !(2.0..=45.0).contains(&step_deg) || !(1..=10).contains(&frames) {
                    return Err(ScanError::InvalidParams(format!(
                        "step_deg {step_deg} not in [2, 45] or frames_per_stop {frames} not in \
                         [1, 10]"
                    )));
                }
                AutoMode::Stops {
                    step_deg,
                    frames: frames as usize,
                }
            }
        };
        let resolution_mm = p.resolution_mm.unwrap_or(AUTO_RESOLUTION_MM);
        let (lo, hi) = RECONSTRUCT_RESOLUTION_MM;
        if !(lo..=hi).contains(&resolution_mm) {
            return Err(ScanError::InvalidParams(format!(
                "resolution_mm {resolution_mm} not in [{lo}, {hi}]"
            )));
        }
        // continuous scans keep no stops: nothing to rebuild from
        let finish =
            (p.finish.unwrap_or(true) && matches!(mode, AutoMode::Stops { .. })).then(|| {
                FinishPlan {
                    resolution_mm,
                    refine: p.refine.unwrap_or(true),
                }
            });
        match self.session.status().state {
            ScanState::Connected | ScanState::Stopped => self.session.start_held()?,
            ScanState::Scanning => self.session.rig_link().set_hold(true),
            state => {
                return Err(ScanError::InvalidState {
                    action: "start the automatic scan",
                    state,
                });
            }
        }
        let plan = AutoPlan {
            tilts_deg: tilts,
            rotation_deg,
            deg_per_s: p.deg_per_s,
            settle: Duration::from_secs_f32(settle_s),
            has_tilt,
            mode,
            finish,
        };
        let steps = plan.steps();
        let job = jobs::start_auto(table, self.session.rig_link(), plan)?;
        self.rig.last_auto = Some(Arc::clone(&job.shared));
        self.rig.job = Some((RigJobKind::Auto, job));
        Ok(json!({ "started": true, "steps": steps }))
    }

    fn calibrate(&mut self, p: &CalibrateParams) -> Result<Value> {
        let (table, axes, info) = self.job_prerequisites()?;
        let steps = p.steps.unwrap_or(24);
        let step_deg = p.step_deg.unwrap_or(15.0);
        let total = steps as f32 * step_deg;
        let frames = p.frames_per_stop.unwrap_or(3);
        let settle_s = p.settle_s.unwrap_or(0.5);
        if !(3..=36).contains(&steps)
            || !(5.0..=120.0).contains(&step_deg)
            || !(180.0..=1080.0).contains(&total)
            || !(1..=10).contains(&frames)
            || !(0.0..=30.0).contains(&settle_s)
        {
            return Err(ScanError::InvalidParams(format!(
                "{steps} steps of {step_deg} degrees, {frames} frames per stop, settle {settle_s} \
                 s: steps 3 to 36, step_deg 5 to 120, 180 to 1080 degrees in all, 1 to 10 \
                 frames, settle 0 to 30 s"
            )));
        }
        let status = self.session.status();
        match status.state {
            ScanState::Connected | ScanState::Stopped => {}
            state => {
                return Err(ScanError::InvalidState {
                    action: "calibrate",
                    state,
                });
            }
        }
        let tilt_deg = axis_of(&axes, AxisKind::Tilt).ok().map(|t| {
            let max = t.max_deg.unwrap_or(30.0).min(-t.min_deg.unwrap_or(-30.0));
            jobs::CALIBRATION_TILT_DEG.min(0.5 * max)
        });
        let plan = CalibrationPlan {
            steps,
            step_deg,
            tilt_deg,
            frames: frames as usize,
            settle: Duration::from_secs_f32(settle_s),
            scanner_serial: status.device.and_then(|d| d.serial),
            turntable_id: info.id,
            dir: self.rig.dir.clone(),
        };
        // a fresh volume, no region of interest nor prior while measuring
        self.session.reset();
        let probe = self.rig.table.as_ref().and_then(|c| c.probe.clone());
        self.session.set_rig(None, probe);
        self.session.start_held()?;
        let n = plan.steps();
        let job = match jobs::start_calibration(table, self.session.rig_link(), plan) {
            Ok(job) => job,
            Err(e) => {
                self.session.reset();
                self.apply_rig();
                return Err(e);
            }
        };
        self.rig.last_calibration = Some(Arc::clone(&job.shared));
        self.rig.job = Some((RigJobKind::Calibrate, job));
        Ok(json!({ "started": true, "steps": n }))
    }

    /// Lets go of the devices, as `turntable.disconnect` and
    /// `scan.disconnect` together: a running job (`scan.auto`, a
    /// calibration) is cancelled and the settle watcher stopped, since both
    /// hold the table, then the table and the scanner are disconnected. A
    /// host calls it before it goes away, even while another thread still
    /// holds the controller; dropping the controller calls it too.
    ///
    /// A process killed by a signal runs neither: BlueZ then keeps a
    /// Bluetooth table linked to this computer, unusable by other hosts,
    /// until DentalCraft connects to it again (it takes the link over) or
    /// the link is dropped from the system's Bluetooth settings.
    pub fn close(&mut self) {
        self.reap_rig(true);
        self.rig.table = None;
        self.session.disconnect();
        self.apply_rig();
    }
}

impl Drop for ScanController {
    /// [`ScanController::close`]: left running, the threads of a move or a
    /// job would keep the table past the end of the process, and a
    /// Bluetooth table would stay linked to this computer (BlueZ does not
    /// disconnect it).
    fn drop(&mut self) {
        self.close();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn run(c: &mut ScanController, id: &str, params: Value) -> Value {
        c.execute(id, &params, &|| false)
            .unwrap_or_else(|e| panic!("{id} {params}: {e}"))
    }

    #[test]
    fn the_status_shows_a_dropped_link() {
        let dir = tempfile::tempdir().unwrap();
        let mut c = ScanController::new();
        c.set_rig_dir(dir.path());
        run(&mut c, TURNTABLE_CONNECT, json!({"backend": "sim"}));
        assert!(c.turntable_status().connected);
        c.sim_rig().inject_link_drop(0, true);
        let s = run(&mut c, TURNTABLE_STATUS, json!({}));
        assert_eq!(s["connected"], false, "{s}");
        assert_eq!(s["info"]["id"], "sim-turntable", "{s}");
        let err = c
            .execute(
                TURNTABLE_MOVE,
                &json!({"axis": "rotate", "by_deg": 10.0}),
                &|| false,
            )
            .unwrap_err();
        assert_eq!(err.code(), "ble-disconnected");
        // connecting again gives a working table
        run(&mut c, TURNTABLE_CONNECT, json!({"backend": "sim"}));
        assert!(c.turntable_status().connected);
    }

    #[test]
    fn dropping_the_controller_mid_move_closes_the_table() {
        let dir = tempfile::tempdir().unwrap();
        let mut c = ScanController::new();
        c.set_rig_dir(dir.path());
        let rig = Arc::clone(c.sim_rig());
        run(&mut c, TURNTABLE_CONNECT, json!({"backend": "sim"}));
        assert!(rig.attached());
        // a slow real-time move: the settle watcher holds the table meanwhile
        run(
            &mut c,
            TURNTABLE_SPEED,
            json!({"axis": "rotate", "deg_per_s": 1.0}),
        );
        run(
            &mut c,
            TURNTABLE_MOVE,
            json!({"axis": "rotate", "by_deg": 90.0}),
        );
        assert!(c.turntable_status().busy);
        drop(c);
        assert!(!rig.attached(), "the table outlived its controller");
    }

    #[test]
    fn close_ends_a_running_scan_auto_and_lets_go_of_both_devices() {
        let dir = tempfile::tempdir().unwrap();
        let mut c = ScanController::new();
        c.set_rig_dir(dir.path());
        let rig = Arc::clone(c.sim_rig());
        run(
            &mut c,
            crate::control::SCAN_CONNECT,
            json!({"backend": "sim", "options": {"realtime": false, "width": 64, "height": 40}}),
        );
        run(&mut c, TURNTABLE_CONNECT, json!({"backend": "sim"}));
        // a slow real-time pass: the job holds the table for a minute
        let out = run(
            &mut c,
            SCAN_AUTO,
            json!({"tilts_deg": [0.0], "rotation_deg": 60.0, "deg_per_s": 1.0, "allow_uncalibrated": true}),
        );
        assert_eq!(out["started"], true);
        assert!(c.turntable_status().busy);
        c.close();
        assert!(!rig.attached(), "the job kept the table");
        assert_eq!(
            c.auto_status().unwrap().state,
            crate::rig::RigJobState::Cancelled
        );
        assert!(!c.turntable_status().connected);
        assert_eq!(c.session().status().state, ScanState::Idle);
    }
}
