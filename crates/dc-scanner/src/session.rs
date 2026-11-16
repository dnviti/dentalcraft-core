//! Live acquisition: device, tracking and fusion on a background thread.
//!
//! [`ScanSession`] is a small state machine driven by the UI thread:
//!
//! ```text
//! Idle --connect--> Connected --start--> Scanning <--pause/resume--> Paused
//!                                           |  stop                    | stop
//!                                           v                          v
//!                                        Stopped --start (continue)--> Scanning
//! reset: back to Connected (data discarded); disconnect: back to Idle.
//! ```
//!
//! `start` moves the device into an acquisition thread. Every frame is
//! depth-filtered, tracked against the fused model ([`crate::tracking`]) from
//! a constant-velocity prediction and, when tracking holds, integrated into
//! a sparse TSDF volume. The first fused frame defines the world frame.
//!
//! **Pause keeps the device streaming and the camera tracked, but stops
//! fusion**: the live view keeps following the scanner, so the operator can
//! reposition it and resume without losing track. `stop` joins the thread,
//! stops the stream and returns the fused cloud; the volume is kept, so a
//! later `start` continues the same scan (use `reset` to discard it).
//!
//! The UI polls [`ScanSession::status`] (counters) and
//! [`ScanSession::preview`] (fused cloud, latest frame, camera), refreshed
//! by the thread at most every [`PREVIEW_INTERVAL`]. No method blocks on the
//! device except `stop` / `reset` / `disconnect`, which wait for the thread
//! to finish its current frame.
//!
//! **Turntable rig** ([`crate::rig`]): with a rig calibration
//! ([`ScanSession::set_rig`]) every frame is cropped to the region of
//! interest around the turntable axis, and the motion of the table since
//! the previous tracked frame, predicted from its live angle estimate, is
//! the initial guess of the tracking. Rig jobs may hold the frames out of
//! the model while tracking goes on (tilting), like a pause.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, MutexGuard};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use dc_geom::tsdf::{TsdfParams, TsdfVolume};
use dc_mesh::PointCloud;
use dc_turntable::AngleProbe;
use glam::{Affine3A, Vec3};
use rayon::prelude::*;
use serde::{Deserialize, Serialize};

use crate::rig::{Background, RigCalibration, RigGeometry, RigInput};
use crate::tracking::{TrackParams, sample_frame, track_frame};
use crate::{DepthFrame, DeviceInfo, Result, ScanError, ScannerBackend, ScannerDevice, backend};

/// Shortest time between two preview refreshes.
pub const PREVIEW_INTERVAL: Duration = Duration::from_millis(200);
/// How long the acquisition thread waits for a frame before checking for stop.
const FRAME_WAIT: Duration = Duration::from_millis(50);

/// Acquisition state.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum ScanState {
    /// No device connected.
    Idle,
    /// A device is connected, nothing is being acquired.
    Connected,
    /// Frames are tracked and fused.
    Scanning,
    /// Frames are tracked but not fused.
    Paused,
    /// Acquisition ended; the fused scan is available.
    Stopped,
}

/// Acquisition settings.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(default)]
pub struct ScanSettings {
    /// Voxel size of the live fusion volume (mm).
    pub live_voxel_mm: f32,
    /// Depths closer than this are ignored (mm). The default, 100 mm, is
    /// below the 120 mm where the MINI 2 working range starts: nearer values
    /// are artefacts.
    pub min_depth_mm: f32,
    /// Depths farther than this are ignored (mm).
    pub max_depth_mm: f32,
    /// Exposure time sent to the device at start, when set (µs).
    pub exposure_us: Option<u32>,
    /// Radius of the turntable region of interest around the axis (mm).
    pub roi_radius_mm: f32,
    /// Height of the turntable region of interest above the platter (mm).
    pub roi_height_mm: f32,
    /// Crop the frames to the turntable region of interest when a rig
    /// calibration is in use ([`crate::rig`]).
    pub use_roi: bool,
}

impl Default for ScanSettings {
    fn default() -> Self {
        Self {
            live_voxel_mm: 0.3,
            min_depth_mm: 100.0,
            max_depth_mm: 300.0,
            exposure_us: None,
            roi_radius_mm: crate::rig::DEFAULT_ROI_RADIUS_MM,
            roi_height_mm: crate::rig::DEFAULT_ROI_HEIGHT_MM,
            use_roi: true,
        }
    }
}

impl ScanSettings {
    /// Checks the ranges: voxel in `[0.05, 5]` mm, `0 <= min < max` depth,
    /// region of interest radius in `[5, 200]` mm and height in `[5, 300]`
    /// mm.
    pub fn validate(&self) -> Result<()> {
        if !(0.05..=5.0).contains(&self.live_voxel_mm) {
            return Err(ScanError::InvalidParams(format!(
                "live_voxel_mm {} not in [0.05, 5]",
                self.live_voxel_mm
            )));
        }
        if !(self.min_depth_mm >= 0.0 && self.min_depth_mm < self.max_depth_mm) {
            return Err(ScanError::InvalidParams(format!(
                "depth range [{}, {}] invalid",
                self.min_depth_mm, self.max_depth_mm
            )));
        }
        if !(5.0..=200.0).contains(&self.roi_radius_mm)
            || !(5.0..=300.0).contains(&self.roi_height_mm)
        {
            return Err(ScanError::InvalidParams(format!(
                "roi_radius_mm {} not in [5, 200] or roi_height_mm {} not in [5, 300]",
                self.roi_radius_mm, self.roi_height_mm
            )));
        }
        Ok(())
    }

    fn tsdf(&self) -> TsdfParams {
        TsdfParams {
            voxel_mm: self.live_voxel_mm,
            trunc_mm: (4.0 * self.live_voxel_mm).max(1.0),
            max_weight: 64.0,
        }
    }
}

/// Snapshot of the acquisition, cheap to take.
#[derive(Clone, Debug, Serialize)]
pub struct ScanStatus {
    /// Current state.
    pub state: ScanState,
    /// The connected device.
    pub device: Option<DeviceInfo>,
    /// Frames received since the last reset.
    pub frames: u64,
    /// Frames fused into the model.
    pub fused_frames: u64,
    /// Frames rejected by tracking.
    pub tracking_lost: u64,
    /// The latest frame was tracked.
    pub tracking_ok: bool,
    /// Points of the latest preview cloud.
    pub points: usize,
    /// Frames processed per second (smoothed).
    pub fps: f32,
    /// [`ScanError::code`] of the failure that ended the stream, if any.
    pub error: Option<String>,
    /// Technical detail of that failure (untranslated).
    pub error_detail: Option<String>,
    /// Turntable angles `[rotate, tilt]` (degrees) when the world frame was
    /// set (first fused frame), when a turntable was connected.
    pub world_angles_deg: Option<[f32; 2]>,
}

/// What the live view draws.
#[derive(Clone, Debug)]
pub struct LivePreview {
    /// Fused surface points so far (with normals).
    pub cloud: Arc<PointCloud>,
    /// Points of the latest frame in world coordinates.
    pub frame_points: Arc<Vec<Vec3>>,
    /// Camera-to-world pose of the latest frame.
    pub camera: Affine3A,
    /// Increases at every refresh.
    pub generation: u64,
}

/// State shared with the acquisition thread.
#[derive(Default)]
struct Shared {
    stop: AtomicBool,
    paused: AtomicBool,
    /// A rig job keeps the frames out of the model (tracking continues).
    hold: AtomicBool,
    live: Mutex<Live>,
    rig: Mutex<RigInput>,
    grab: Mutex<Option<Grab>>,
}

impl Shared {
    fn live(&self) -> MutexGuard<'_, Live> {
        // a panicking acquisition thread must not take the UI down with it
        self.live.lock().unwrap_or_else(|e| e.into_inner())
    }

    fn rig(&self) -> MutexGuard<'_, RigInput> {
        self.rig.lock().unwrap_or_else(|e| e.into_inner())
    }

    fn grab(&self) -> MutexGuard<'_, Option<Grab>> {
        self.grab.lock().unwrap_or_else(|e| e.into_inner())
    }
}

/// A depth image requested by a rig job: the first frame numbered `after`
/// or later, range-filtered.
struct Grab {
    after: u64,
    depth: Option<Arc<Vec<f32>>>,
}

/// Camera pose of one processed frame.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct TrackSnapshot {
    /// Number of the frame (1 = first frame since the last reset).
    pub frame: u64,
    /// Camera-to-world pose.
    pub pose: Affine3A,
    /// The frame was tracked.
    pub ok: bool,
}

#[derive(Default)]
struct Live {
    frames: u64,
    fused_frames: u64,
    tracking_lost: u64,
    tracking_ok: bool,
    fps: f32,
    preview: Option<LivePreview>,
    generation: u64,
    error: Option<(&'static str, String)>,
    last_track: Option<TrackSnapshot>,
    world_angles: Option<[f32; 2]>,
}

impl Live {
    fn publish(&mut self, cloud: Arc<PointCloud>, frame_points: Arc<Vec<Vec3>>, camera: Affine3A) {
        self.generation += 1;
        self.preview = Some(LivePreview {
            cloud,
            frame_points,
            camera,
            generation: self.generation,
        });
    }
}

/// The fused model and the camera track; owned by the thread while scanning.
struct Fusion {
    volume: TsdfVolume,
    /// Camera-to-world pose of the latest tracked frame.
    pose: Affine3A,
    /// Motion of the camera between the two latest tracked frames (camera frame).
    velocity: Affine3A,
    /// The volume holds at least one frame.
    started: bool,
    /// Turntable angles `(rotate, tilt)` of the latest tracked frame.
    angles: Option<(f32, f32)>,
}

impl Fusion {
    fn new(settings: &ScanSettings) -> Self {
        Self {
            volume: TsdfVolume::new(settings.tsdf()),
            pose: Affine3A::IDENTITY,
            velocity: Affine3A::IDENTITY,
            started: false,
            angles: None,
        }
    }
}

/// The acquisition seen from a rig job thread: hold, frames, poses. Cheap
/// to clone; it never blocks the acquisition.
#[derive(Clone)]
pub(crate) struct RigLink(Arc<Shared>);

impl RigLink {
    /// Keeps the frames out of the model (`true`) or lets them in.
    pub fn set_hold(&self, hold: bool) {
        self.0.hold.store(hold, Ordering::SeqCst);
    }

    /// The user paused the scan.
    pub fn paused(&self) -> bool {
        self.0.paused.load(Ordering::SeqCst)
    }

    /// Frames processed since the last reset.
    pub fn frames(&self) -> u64 {
        self.0.live().frames
    }

    /// Frames fused since the last reset.
    pub fn fused_frames(&self) -> u64 {
        self.0.live().fused_frames
    }

    /// The latest preview.
    pub fn preview(&self) -> Option<LivePreview> {
        self.0.live().preview.clone()
    }

    /// Sets (or clears) the static background removed from the frames.
    pub fn set_background(&self, background: Option<Arc<Background>>) {
        self.0.rig().background = background;
    }

    /// Fails when the stream ended, waits a little otherwise.
    fn tick(&self, deadline: Instant, cancel: &dyn Fn() -> bool, what: &str) -> Result<()> {
        if cancel() {
            return Err(ScanError::Cancelled);
        }
        if let Some((code, detail)) = self.0.live().error.clone() {
            return Err(ScanError::Device(format!("{code}: {detail}")));
        }
        if self.0.stop.load(Ordering::SeqCst) {
            return Err(ScanError::Device("the acquisition stopped".into()));
        }
        if Instant::now() >= deadline {
            return Err(ScanError::Device(format!("no frame in time ({what})")));
        }
        std::thread::sleep(Duration::from_millis(2));
        Ok(())
    }

    /// The pose of a frame processed after this call (one frame in flight
    /// is skipped).
    pub fn next_pose(&self, timeout: Duration, cancel: &dyn Fn() -> bool) -> Result<TrackSnapshot> {
        let after = self.frames() + 2;
        let deadline = Instant::now() + timeout;
        loop {
            if let Some(t) = self.0.live().last_track
                && t.frame >= after
            {
                return Ok(t);
            }
            self.tick(deadline, cancel, "pose")?;
        }
    }

    /// The range-filtered depth of a frame processed after this call (one
    /// frame in flight is skipped).
    pub fn grab_depth(
        &self,
        timeout: Duration,
        cancel: &dyn Fn() -> bool,
    ) -> Result<Arc<Vec<f32>>> {
        let after = self.frames() + 2;
        *self.0.grab() = Some(Grab { after, depth: None });
        let deadline = Instant::now() + timeout;
        loop {
            let depth = {
                let mut grab = self.0.grab();
                let depth = grab.as_mut().and_then(|g| g.depth.take());
                if depth.is_some() {
                    *grab = None;
                }
                depth
            };
            if let Some(d) = depth {
                return Ok(d);
            }
            if let Err(e) = self.tick(deadline, cancel, "depth") {
                *self.0.grab() = None;
                return Err(e);
            }
        }
    }

    /// Waits until at least one frame was fused.
    pub fn wait_fused(&self, timeout: Duration, cancel: &dyn Fn() -> bool) -> Result<()> {
        let deadline = Instant::now() + timeout;
        while self.fused_frames() == 0 {
            self.tick(deadline, cancel, "first fused frame")?;
        }
        Ok(())
    }

    /// Waits for a preview published after this call.
    pub fn next_preview(
        &self,
        timeout: Duration,
        cancel: &dyn Fn() -> bool,
    ) -> Result<LivePreview> {
        let generation = self.0.live().generation;
        let deadline = Instant::now() + timeout;
        loop {
            if let Some(p) = self.preview()
                && p.generation > generation
            {
                return Ok(p);
            }
            self.tick(deadline, cancel, "preview")?;
        }
    }
}

type Worker = JoinHandle<(Box<dyn ScannerDevice>, Fusion)>;

/// A scanning session: one device, one fused scan. See the module doc.
pub struct ScanSession {
    settings: ScanSettings,
    state: ScanState,
    info: Option<DeviceInfo>,
    device: Option<Box<dyn ScannerDevice>>,
    fusion: Option<Fusion>,
    worker: Option<Worker>,
    shared: Arc<Shared>,
}

impl std::fmt::Debug for ScanSession {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ScanSession")
            .field("state", &self.state)
            .field("device", &self.info)
            .field("settings", &self.settings)
            .finish_non_exhaustive()
    }
}

impl ScanSession {
    /// Session without a device.
    pub fn new(settings: ScanSettings) -> Self {
        Self {
            settings,
            state: ScanState::Idle,
            info: None,
            device: None,
            fusion: None,
            worker: None,
            shared: Arc::default(),
        }
    }

    /// Opens device `device_id` of the registered backend `backend`
    /// (disconnecting the current device first).
    ///
    /// # Errors
    /// [`ScanError::InvalidState`] while scanning; the backend's errors.
    pub fn connect(&mut self, backend_id: &str, device_id: &str) -> Result<DeviceInfo> {
        self.connect_with(backend(backend_id)?.as_ref(), device_id)
    }

    /// [`ScanSession::connect`] with an explicit backend (tests, custom
    /// backends not in the registry).
    pub fn connect_with(
        &mut self,
        backend: &dyn ScannerBackend,
        device_id: &str,
    ) -> Result<DeviceInfo> {
        self.require_idle("connect")?;
        self.disconnect();
        let device = backend.open(device_id)?;
        let info = device.info();
        self.device = Some(device);
        self.info = Some(info.clone());
        self.state = ScanState::Connected;
        Ok(info)
    }

    /// Ends any acquisition, discards the scan and closes the device.
    pub fn disconnect(&mut self) {
        self.reset();
        self.device = None;
        self.info = None;
        self.state = ScanState::Idle;
    }

    /// Starts (from `Connected`) or continues (from `Stopped`) the acquisition.
    ///
    /// # Errors
    /// [`ScanError::NotConnected`], [`ScanError::InvalidState`], or the
    /// device's error when its stream does not start.
    pub fn start(&mut self) -> Result<()> {
        self.start_with(false)
    }

    /// [`ScanSession::start`] with the frames held out of the model until a
    /// rig job releases them (the job's first motion runs unfused).
    pub(crate) fn start_held(&mut self) -> Result<()> {
        self.start_with(true)
    }

    fn start_with(&mut self, hold: bool) -> Result<()> {
        match self.state {
            ScanState::Connected | ScanState::Stopped => {}
            ScanState::Idle => return Err(ScanError::NotConnected),
            state => {
                return Err(ScanError::InvalidState {
                    action: "start",
                    state,
                });
            }
        }
        let mut device = self.device.take().ok_or(ScanError::NotConnected)?;
        let started = self
            .settings
            .exposure_us
            .map_or(Ok(()), |us| device.set_exposure_us(us))
            .and_then(|()| device.start());
        if let Err(e) = started {
            self.device = Some(device);
            return Err(e);
        }
        let fusion = self
            .fusion
            .take()
            .unwrap_or_else(|| Fusion::new(&self.settings));
        self.shared.stop.store(false, Ordering::SeqCst);
        self.shared.paused.store(false, Ordering::SeqCst);
        self.shared.hold.store(hold, Ordering::SeqCst);
        self.shared.live().error = None;
        let shared = Arc::clone(&self.shared);
        let settings = self.settings.clone();
        let worker = std::thread::Builder::new()
            .name("dc-scanner-acquisition".into())
            .spawn(move || acquire(device, fusion, &shared, &settings))
            .map_err(|e| ScanError::Device(format!("cannot start the acquisition thread: {e}")))?;
        self.worker = Some(worker);
        self.state = ScanState::Scanning;
        Ok(())
    }

    /// Stops fusing frames (tracking continues).
    ///
    /// # Errors
    /// [`ScanError::InvalidState`] unless scanning.
    pub fn pause(&mut self) -> Result<()> {
        if self.state != ScanState::Scanning {
            return Err(ScanError::InvalidState {
                action: "pause",
                state: self.state,
            });
        }
        self.shared.paused.store(true, Ordering::SeqCst);
        self.state = ScanState::Paused;
        Ok(())
    }

    /// Resumes fusing frames.
    ///
    /// # Errors
    /// [`ScanError::InvalidState`] unless paused.
    pub fn resume(&mut self) -> Result<()> {
        if self.state != ScanState::Paused {
            return Err(ScanError::InvalidState {
                action: "resume",
                state: self.state,
            });
        }
        self.shared.paused.store(false, Ordering::SeqCst);
        self.state = ScanState::Scanning;
        Ok(())
    }

    /// Ends the acquisition and returns the fused cloud (surface points with
    /// normals, world = first frame camera). From `Stopped` it returns the
    /// same cloud again.
    ///
    /// # Errors
    /// [`ScanError::InvalidState`] when nothing was acquired;
    /// [`ScanError::Device`] when the acquisition thread failed.
    pub fn stop(&mut self) -> Result<PointCloud> {
        match self.state {
            ScanState::Scanning | ScanState::Paused => self.join()?,
            ScanState::Stopped => {}
            state => {
                return Err(ScanError::InvalidState {
                    action: "stop",
                    state,
                });
            }
        }
        self.state = ScanState::Stopped;
        let fusion = self.fusion.as_ref().ok_or(ScanError::Empty("scan"))?;
        let cloud = fusion.volume.extract_cloud();
        let mut live = self.shared.live();
        let frame_points = live
            .preview
            .as_ref()
            .map_or_else(Arc::default, |p| Arc::clone(&p.frame_points));
        live.publish(Arc::new(cloud.clone()), frame_points, fusion.pose);
        Ok(cloud)
    }

    /// Discards the scan (stopping any acquisition); the device stays connected.
    pub fn reset(&mut self) {
        if self.worker.is_some() {
            // a failed thread has nothing left to discard
            let _ = self.join();
        }
        self.fusion = None;
        *self.shared.live() = Live::default();
        *self.shared.grab() = None;
        self.shared.rig().background = None;
        self.shared.hold.store(false, Ordering::SeqCst);
        self.state = if self.device.is_some() {
            ScanState::Connected
        } else {
            ScanState::Idle
        };
    }

    /// A rig job ended: the acquisition goes on paused (fusion off, tracking
    /// on), ready for `stop` or `resume`.
    pub(crate) fn end_hold(&mut self) {
        if self.state == ScanState::Scanning {
            self.shared.paused.store(true, Ordering::SeqCst);
            self.state = ScanState::Paused;
        }
        self.shared.hold.store(false, Ordering::SeqCst);
    }

    /// The acquisition as seen by a rig job thread.
    pub(crate) fn rig_link(&self) -> RigLink {
        RigLink(Arc::clone(&self.shared))
    }

    /// Sets the turntable rig used by the acquisition: its calibration
    /// (region of interest and motion prior, see [`crate::rig`]) and the
    /// live angle estimate of the table. Takes effect from the next frame.
    pub fn set_rig(&mut self, calibration: Option<RigCalibration>, probe: Option<AngleProbe>) {
        let mut rig = self.shared.rig();
        rig.geometry = calibration.as_ref().map(RigGeometry::new);
        rig.calibration = calibration;
        rig.probe = probe;
    }

    /// The rig calibration in use.
    pub fn rig_calibration(&self) -> Option<RigCalibration> {
        self.shared.rig().calibration.clone()
    }

    /// Current state and counters.
    pub fn status(&self) -> ScanStatus {
        let live = self.shared.live();
        ScanStatus {
            state: self.state,
            device: self.info.clone(),
            frames: live.frames,
            fused_frames: live.fused_frames,
            tracking_lost: live.tracking_lost,
            tracking_ok: live.tracking_ok,
            points: live.preview.as_ref().map_or(0, |p| p.cloud.len()),
            fps: live.fps,
            error: live.error.as_ref().map(|(code, _)| (*code).to_owned()),
            error_detail: live.error.as_ref().map(|(_, detail)| detail.clone()),
            world_angles_deg: live.world_angles,
        }
    }

    /// Latest live preview (shared buffers, cheap to clone).
    pub fn preview(&self) -> Option<LivePreview> {
        self.shared.live().preview.clone()
    }

    /// The acquisition settings.
    pub fn settings(&self) -> &ScanSettings {
        &self.settings
    }

    /// Replaces the settings. The fusion voxel size applies to the next new
    /// volume (after [`ScanSession::reset`] or a new connection).
    ///
    /// # Errors
    /// [`ScanError::InvalidState`] while scanning or paused,
    /// [`ScanError::InvalidParams`] for values out of range.
    pub fn set_settings(&mut self, settings: ScanSettings) -> Result<()> {
        self.require_idle("change settings")?;
        settings.validate()?;
        self.settings = settings;
        Ok(())
    }

    fn require_idle(&self, action: &'static str) -> Result<()> {
        match self.state {
            ScanState::Scanning | ScanState::Paused => Err(ScanError::InvalidState {
                action,
                state: self.state,
            }),
            _ => Ok(()),
        }
    }

    /// Stops the thread and takes back the device and the fused model.
    fn join(&mut self) -> Result<()> {
        let Some(worker) = self.worker.take() else {
            return Ok(());
        };
        self.shared.stop.store(true, Ordering::SeqCst);
        match worker.join() {
            Ok((mut device, fusion)) => {
                device.stop();
                self.device = Some(device);
                self.fusion = Some(fusion);
                Ok(())
            }
            Err(_) => {
                // the device went down with the thread
                self.info = None;
                self.state = ScanState::Idle;
                let detail = "the acquisition thread panicked".to_owned();
                self.shared.live().error = Some(("device", detail.clone()));
                Err(ScanError::Device(detail))
            }
        }
    }
}

impl Drop for ScanSession {
    fn drop(&mut self) {
        let _ = self.join();
    }
}

/// Body of the acquisition thread.
fn acquire(
    mut device: Box<dyn ScannerDevice>,
    mut fusion: Fusion,
    shared: &Shared,
    settings: &ScanSettings,
) -> (Box<dyn ScannerDevice>, Fusion) {
    let params = TrackParams::default();
    let mut cloud: Arc<PointCloud> = shared
        .live()
        .preview
        .as_ref()
        .map_or_else(Arc::default, |p| Arc::clone(&p.cloud));
    let mut extracted_at = shared.live().fused_frames;
    let mut last_publish: Option<Instant> = None;
    let mut last_frame: Option<Instant> = None;
    while !shared.stop.load(Ordering::SeqCst) {
        let frame = match device.next_frame(FRAME_WAIT) {
            Ok(Some(frame)) => frame,
            Ok(None) => continue,
            Err(e) => {
                shared.live().error = Some((e.code(), e.to_string()));
                break;
            }
        };
        let k = frame.intrinsics;
        if frame.depth_mm.len() != k.width as usize * k.height as usize {
            let e = ScanError::Device(format!(
                "frame has {} depth values, expected {}x{}",
                frame.depth_mm.len(),
                k.width,
                k.height
            ));
            shared.live().error = Some((e.code(), e.to_string()));
            break;
        }
        let paused = shared.paused.load(Ordering::SeqCst) || shared.hold.load(Ordering::SeqCst);
        let (geometry, probe, background) = {
            let rig = shared.rig();
            (rig.geometry, rig.probe.clone(), rig.background.clone())
        };
        // the turntable angles of this frame, from the live estimate
        let angles = probe
            .map(|p| p())
            .and_then(|a| Some((a.rotate_deg?, a.tilt_deg.unwrap_or(0.0))));
        let roi = geometry
            .filter(|_| settings.use_roi)
            .map(|g| (g, angles.map_or(0.0, |a| a.1)));
        let depth = filter_depth(&frame, settings, background.as_deref(), roi.as_ref());
        {
            let mut grab = shared.grab();
            let number = shared.live().frames + 1;
            if let Some(g) = grab.as_mut()
                && g.depth.is_none()
                && number >= g.after
            {
                g.depth = Some(Arc::new(depth.clone()));
            }
        }
        let sample = sample_frame(&depth, &k, &params);
        let tracked = if !fusion.started {
            // the first frame defines the world
            Some(Affine3A::IDENTITY)
        } else {
            let guess = match (geometry, angles, fusion.angles) {
                // the turntable moved the object: P_k = P_prev M_prev M_k^-1
                (Some(g), Some(now), Some(prev)) if g.has_tilt() || now.1 == prev.1 => {
                    fusion.pose * g.platter(prev.0, prev.1) * g.platter(now.0, now.1).inverse()
                }
                _ => fusion.pose * fusion.velocity,
            };
            let r = track_frame(&fusion.volume, &sample, &k, &fusion.pose, &guess, &params);
            r.ok.then_some(r.pose)
        };
        let fused = match tracked {
            Some(pose) => {
                fusion.velocity = fusion.pose.inverse() * pose;
                fusion.pose = pose;
                fusion.angles = angles;
                if paused || (!fusion.started && sample.points.is_empty()) {
                    false
                } else {
                    fusion.volume.integrate(&depth, &k, &pose);
                    fusion.started = true;
                    true
                }
            }
            None => {
                fusion.velocity = Affine3A::IDENTITY;
                false
            }
        };
        let now = Instant::now();
        let mut live = shared.live();
        live.frames += 1;
        live.tracking_ok = tracked.is_some();
        if tracked.is_none() {
            live.tracking_lost += 1;
        }
        if fused {
            live.fused_frames += 1;
            if live.fused_frames == 1 {
                live.world_angles = angles.map(|(r, t)| [r, t]);
            }
        }
        live.last_track = Some(TrackSnapshot {
            frame: live.frames,
            pose: tracked.unwrap_or(fusion.pose),
            ok: tracked.is_some(),
        });
        if let Some(prev) = last_frame {
            let dt = (now - prev).as_secs_f32().max(1e-6);
            live.fps = if live.fps > 0.0 {
                0.8 * live.fps + 0.2 / dt
            } else {
                1.0 / dt
            };
        }
        last_frame = Some(now);
        if last_publish.is_none_or(|t| now - t >= PREVIEW_INTERVAL) {
            let fused_frames = live.fused_frames;
            drop(live);
            if fused_frames != extracted_at {
                cloud = Arc::new(fusion.volume.extract_cloud());
                extracted_at = fused_frames;
            }
            let camera = tracked.unwrap_or(fusion.pose);
            let frame_points = sample
                .points
                .iter()
                .map(|p| camera.transform_point3(*p))
                .collect();
            shared
                .live()
                .publish(Arc::clone(&cloud), Arc::new(frame_points), camera);
            last_publish = Some(now);
        }
    }
    (device, fusion)
}

/// Depth with the values outside the working range, on the static
/// `background` or outside the turntable region of interest `roi` (geometry
/// and tilt) marked invalid (`NaN`).
fn filter_depth(
    frame: &DepthFrame,
    settings: &ScanSettings,
    background: Option<&Background>,
    roi: Option<&(RigGeometry, f32)>,
) -> Vec<f32> {
    let k = &frame.intrinsics;
    let w = k.width as usize;
    frame
        .depth_mm
        .par_iter()
        .enumerate()
        .with_min_len(1024)
        .map(|(i, &d)| {
            let keep = d >= settings.min_depth_mm
                && d <= settings.max_depth_mm
                && !background.is_some_and(|b| b.is_static(i, d))
                && roi.is_none_or(|(g, tilt)| {
                    let p = k.ray((i % w) as f32, (i / w) as f32) * d;
                    g.roi(p, *tilt, settings.roi_radius_mm, settings.roi_height_mm)
                });
            if keep { d } else { f32::NAN }
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sim::{SimBackend, SimOptions, UPPER_ID};

    fn sim(frames: u64) -> SimBackend {
        SimBackend::new(SimOptions {
            realtime: false,
            max_frames: Some(frames),
            width: 160,
            height: 100,
            ..SimOptions::default()
        })
    }

    #[test]
    fn state_machine_refuses_out_of_order_actions() {
        let mut s = ScanSession::new(ScanSettings::default());
        assert_eq!(s.status().state, ScanState::Idle);
        assert_eq!(s.start().unwrap_err().code(), "not-connected");
        assert_eq!(s.pause().unwrap_err().code(), "invalid-state");
        assert_eq!(s.stop().unwrap_err().code(), "invalid-state");
        assert_eq!(
            s.connect("nope", UPPER_ID).unwrap_err().code(),
            "unknown-backend"
        );
        let info = s.connect_with(&sim(4), UPPER_ID).unwrap();
        assert_eq!(info.id, UPPER_ID);
        assert_eq!(s.status().state, ScanState::Connected);
        assert_eq!(s.resume().unwrap_err().code(), "invalid-state");
        s.start().unwrap();
        assert_eq!(s.start().unwrap_err().code(), "invalid-state");
        assert_eq!(
            s.set_settings(ScanSettings::default()).unwrap_err().code(),
            "invalid-state"
        );
        s.pause().unwrap();
        assert_eq!(s.status().state, ScanState::Paused);
        s.resume().unwrap();
        s.stop().unwrap();
        assert_eq!(s.status().state, ScanState::Stopped);
        s.reset();
        assert_eq!(s.status().state, ScanState::Connected);
        assert_eq!(s.status().frames, 0);
        s.disconnect();
        assert_eq!(s.status().state, ScanState::Idle);
        assert!(s.status().device.is_none());
    }

    #[test]
    fn settings_are_validated() {
        let mut s = ScanSession::new(ScanSettings::default());
        let bad = ScanSettings {
            live_voxel_mm: 0.0,
            ..ScanSettings::default()
        };
        assert_eq!(s.set_settings(bad).unwrap_err().code(), "invalid-params");
        let bad = ScanSettings {
            min_depth_mm: 300.0,
            max_depth_mm: 100.0,
            ..ScanSettings::default()
        };
        assert_eq!(s.set_settings(bad).unwrap_err().code(), "invalid-params");
        let good = ScanSettings {
            live_voxel_mm: 0.5,
            ..ScanSettings::default()
        };
        s.set_settings(good.clone()).unwrap();
        assert_eq!(s.settings(), &good);
    }

    /// Polls until `frames` frames were processed.
    fn wait_frames(s: &ScanSession, frames: u64) {
        let deadline = Instant::now() + Duration::from_secs(60);
        while s.status().frames < frames {
            assert!(Instant::now() < deadline, "{:?}", s.status());
            std::thread::sleep(Duration::from_millis(5));
        }
    }

    #[test]
    fn paused_frames_are_tracked_but_not_fused() {
        let mut s = ScanSession::new(ScanSettings {
            live_voxel_mm: 0.5,
            ..ScanSettings::default()
        });
        let backend = SimBackend::new(SimOptions {
            fps: 40.0,
            max_frames: Some(12),
            width: 160,
            height: 100,
            ..SimOptions::default()
        });
        s.connect_with(&backend, UPPER_ID).unwrap();
        s.pause().unwrap_err();
        s.start().unwrap();
        wait_frames(&s, 2);
        s.pause().unwrap();
        // a frame in flight when the pause lands may still be fused
        wait_frames(&s, (s.status().frames + 1).min(12));
        let before = s.status().fused_frames;
        assert!(before >= 1);
        wait_frames(&s, 12);
        let st = s.status();
        assert_eq!(st.fused_frames, before, "{st:?}");
        assert!(st.frames > st.fused_frames);
        s.resume().unwrap();
        let cloud = s.stop().unwrap();
        assert!(!cloud.is_empty());
        let p = s.preview().unwrap();
        assert_eq!(p.cloud.len(), cloud.len());
    }
}
