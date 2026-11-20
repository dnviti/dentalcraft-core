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
//! the previous tracked frame, predicted from its angle estimate at the
//! moment the frame shows ([`ScannerDevice::latency`] before it arrived),
//! is the initial guess of the tracking; a frame that stays still while
//! the table turns is lost (locked on the static scene). Rig jobs may hold
//! the frames out of the model while tracking goes on (tilting), like a
//! pause, and ask the thread for settled stops: the median of a few frames
//! registered from the turntable pose and fused (stop-and-capture), and
//! loop closures over a revolution of stops. Once everything is captured a
//! job ends the acquisition (`RigLink::end_acquisition`): the stream
//! stops and the session reports `Stopped` while the job computes.

use std::collections::VecDeque;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, MutexGuard};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use dc_geom::icp::rotation_angle;
use dc_geom::tsdf::{CameraIntrinsics, TsdfParams, TsdfVolume};
use dc_mesh::PointCloud;
use dc_turntable::AngleProbe;
use glam::{Affine3A, Quat, Vec3};
use rayon::prelude::*;
use serde::{Deserialize, Serialize};

use crate::reconstruct::{KeptStop, KeptStops, stop_correction};
use crate::rig::fit::{angle_about, to_d};
use crate::rig::{
    Background, CalibrationCheck, RigCalibration, RigGeometry, RigInput, about, camera_world,
    gravity_world,
};
use crate::tracking::{FrameSample, TrackParams, sample_frame, track_frame};
use crate::{
    DepthFrame, DeviceInfo, Result, ScanError, ScannerBackend, ScannerDevice, VendorStatus, backend,
};

/// Shortest time between two preview refreshes.
pub const PREVIEW_INTERVAL: Duration = Duration::from_millis(200);
/// How long the acquisition thread waits for a frame before checking for stop.
const FRAME_WAIT: Duration = Duration::from_millis(50);
/// Weight of the newest frame in the smoothed IMU up: one accelerometer
/// reading is noisy, and a hand-held scanner adds its own accelerations.
const IMU_UP_SMOOTHING: f32 = 0.2;

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
    /// Keep the scanner's fill light steadily on while scanning (on a
    /// device that supports it, see [`crate::revopoint_usb::light`]);
    /// `false` turns it off or leaves it as the device keeps it.
    pub fill_light: bool,
    /// Run the scanner at its fast frame rate (on a device that supports
    /// it, see [`crate::revopoint_usb::exposure`]): about 6.8 instead of
    /// 2.7 frames per second on the MINI 2. `false` sends nothing.
    pub fast_exposure: bool,
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
            fill_light: true,
            fast_exposure: true,
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
    /// Turntable angles `[rotate, tilt]` (degrees, as read: the rotation is
    /// cumulative) when the world frame was set (first fused frame), when a
    /// turntable was connected.
    pub world_angles_deg: Option<[f32; 2]>,
    /// The scan world frame (fused cloud, preview, result) as a rigid map
    /// from the camera frame of the first fused frame, column-major 3×4
    /// (`Affine3A::to_cols_array`), once set.
    pub world_from_camera: Option<[f32; 12]>,
    /// The world frame is the calibrated rig frame: +Z the turntable axis
    /// pointing up from the platter, origin on the axis at the platter,
    /// +X toward the scanner. Otherwise it is [`crate::rig::gravity_world`]
    /// when `imu_world`, else [`crate::rig::camera_world`].
    pub rig_world: bool,
    /// Without a rig calibration in use, the world's +Z is the true up
    /// measured by the scanner's IMU (oriented in the camera by a stored
    /// calibration of this scanner, [`RigCalibration::camera_from_imu`]).
    pub imu_world: bool,
    /// Pose of the scanner at the first fused frame in the world frame:
    /// `[tilt, yaw]` in degrees, tilt > 0 looking down, yaw 0 looking at
    /// the axis (along −X).
    pub scanner_deg: Option<[f32; 2]>,
    /// What the vendor layer of the device did (fill light, fast exposure,
    /// origin of the intrinsics and the intrinsics in use, see
    /// [`VendorStatus`]); `None` without a device or for a device without
    /// one.
    pub vendor: Option<VendorStatus>,
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
    /// The acquisition ended without the session: a rig job ended it
    /// ([`RigLink::end_acquisition`]) or its stream failed (device lost).
    /// The stream is off and the thread gone, its model waits to be
    /// joined.
    ended: AtomicBool,
    live: Mutex<Live>,
    rig: Mutex<RigInput>,
    io: Mutex<RigIo>,
}

impl Shared {
    fn live(&self) -> MutexGuard<'_, Live> {
        // a panicking acquisition thread must not take the UI down with it
        self.live.lock().unwrap_or_else(|e| e.into_inner())
    }

    fn rig(&self) -> MutexGuard<'_, RigInput> {
        self.rig.lock().unwrap_or_else(|e| e.into_inner())
    }

    fn io(&self) -> MutexGuard<'_, RigIo> {
        self.io.lock().unwrap_or_else(|e| e.into_inner())
    }
}

/// A capture asked by a rig job at a settled stop.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct CaptureRequest {
    /// Turntable angles `(rotate, tilt)` read at rest.
    pub angles: (f32, f32),
    /// Frames whose per-pixel median is the stop image.
    pub frames: usize,
    /// Frames dropped first (still showing the motion).
    pub skip: usize,
    /// Register the stop image onto the model and fuse it (`false`: only
    /// return it, for the calibration).
    pub fuse: bool,
}

/// What the acquisition thread did with a [`CaptureRequest`].
#[derive(Clone, Debug)]
pub(crate) struct StopOutcome {
    /// The range-filtered median depth image of the stop.
    pub depth: Arc<Vec<f32>>,
    /// Its intrinsics.
    pub intrinsics: CameraIntrinsics,
    /// Index of the stop among the fused stops, when fused.
    pub index: Option<usize>,
    /// The registration was refused: fused at the turntable pose.
    pub flagged: bool,
    /// Correction of the turntable pose about the axis (degrees).
    pub delta_deg: f32,
    /// RMS residual of the registration (mm), when it ran.
    pub rms_mm: Option<f32>,
    /// A refused registration: the fraction of the stop's samples that lie
    /// on the model, within the finest association gate, at the turntable
    /// pose (low: the view is not where the rig calibration puts it).
    /// `None` for an accepted registration, the first view and a view
    /// without samples.
    pub prior_fit: Option<f32>,
}

/// Outcome of a loop closure.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct LoopOutcome {
    /// Turn about the axis that closes the loop (degrees), when measured.
    pub error_deg: Option<f32>,
    /// The model was rebuilt (loop error spread, clutter removed).
    pub applied: bool,
    /// Static clutter pixels removed from the stops.
    pub clutter_pixels: usize,
}

/// A request of a rig job, served by the acquisition thread.
#[derive(Clone, Copy, Debug, PartialEq)]
enum RigRequest {
    Capture(CaptureRequest),
    CloseLoop {
        first: usize,
    },
    /// A snapshot of the kept stops, then the end of the acquisition: the
    /// device stops streaming and the thread returns.
    End,
}

#[derive(Clone, Debug)]
enum RigReply {
    Captured(StopOutcome),
    Loop(LoopOutcome),
    Stops(Arc<KeptStops>),
}

/// Requests of the rig jobs and their replies.
#[derive(Default)]
struct RigIo {
    request: Option<RigRequest>,
    skipped: usize,
    collected: Vec<Arc<Vec<f32>>>,
    reply: Option<RigReply>,
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
    world_angles: Option<[f32; 2]>,
    /// World frame from the first camera frame, once set.
    world: Option<Affine3A>,
    rig_world: bool,
    imu_world: bool,
    latency: Duration,
    /// The up direction measured by the device IMU, smoothed over the
    /// recent frames (frame of the IMU).
    imu_up: Option<Vec3>,
    /// The vendor status of the device while the acquisition thread holds
    /// it, published when the stream starts, at its first frame and when
    /// it ends.
    vendor: Option<VendorStatus>,
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

/// A fused stop, kept to rebuild the model after a loop closure and for
/// the fine reconstruction ([`crate::reconstruct`]).
struct StopRecord {
    /// Valid pixels of the median image: index and depth.
    pixels: Arc<Vec<(u32, f32)>>,
    intrinsics: CameraIntrinsics,
    angles: (f32, f32),
    pose: Affine3A,
    /// Its registration was refused: fused at the turntable pose.
    flagged: bool,
}

impl StopRecord {
    fn depth(&self) -> Vec<f32> {
        let mut d = vec![f32::NAN; self.intrinsics.pixel_count()];
        for &(i, z) in self.pixels.iter() {
            d[i as usize] = z;
        }
        d
    }
}

/// The kept stops of `fusion` (pixels shared, cheap), with the rig
/// `calibration` of the scan and the region of interest of `settings` when
/// it was in use.
fn kept_stops(
    fusion: &Fusion,
    calibration: Option<RigCalibration>,
    settings: &ScanSettings,
) -> KeptStops {
    let roi = (settings.use_roi && fusion.rig_world && calibration.is_some())
        .then_some((settings.roi_radius_mm, settings.roi_height_mm));
    KeptStops {
        stops: fusion
            .stops
            .iter()
            .map(|s| KeptStop {
                pixels: Arc::clone(&s.pixels),
                intrinsics: s.intrinsics,
                angles: s.angles,
                pose: s.pose,
                flagged: s.flagged,
            })
            .collect(),
        world: fusion.world,
        world_angles: fusion.world_angles,
        calibration,
        roi,
    }
}

/// The fused model and the camera track; owned by the thread while scanning.
/// Poses and the volume are in the camera frame of the first fused frame;
/// `world` maps them to the scan world frame for every output.
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
    /// Turntable angles of the world frame (first fused frame).
    world_angles: Option<(f32, f32)>,
    /// The scan world frame from the first camera frame.
    world: Affine3A,
    /// `world` is the calibrated rig frame.
    rig_world: bool,
    /// `world` stands on the up measured by the IMU.
    imu_world: bool,
    /// The true up in the camera frame of the latest frame, from the IMU,
    /// when known.
    up: Option<Vec3>,
    /// The fused stops of `scan.auto`.
    stops: Vec<StopRecord>,
    /// Frames fused one by one (not as stops).
    loose_frames: u64,
}

impl Fusion {
    fn new(settings: &ScanSettings) -> Self {
        Self {
            volume: TsdfVolume::new(settings.tsdf()),
            pose: Affine3A::IDENTITY,
            velocity: Affine3A::IDENTITY,
            started: false,
            angles: None,
            world_angles: None,
            world: camera_world(),
            rig_world: false,
            imu_world: false,
            up: None,
            stops: Vec::new(),
            loose_frames: 0,
        }
    }

    /// Sets the world frame at the first fused frame (taken at `angles`):
    /// the rig frame with a calibration, else [`gravity_world`] on the up
    /// measured by the IMU, else [`camera_world`].
    fn set_world(&mut self, geometry: Option<&RigGeometry>, angles: Option<(f32, f32)>) {
        self.world_angles = angles;
        (self.world, self.rig_world, self.imu_world) = match (geometry, self.up) {
            (Some(g), _) => (
                g.world_from_camera(angles.map_or(0.0, |a| a.1)),
                true,
                false,
            ),
            (None, Some(up)) => (gravity_world(up), false, true),
            (None, None) => (camera_world(), false, false),
        };
    }
}

/// `cloud` moved rigidly by `m`, in place.
fn moved(mut cloud: PointCloud, m: &Affine3A) -> PointCloud {
    for p in &mut cloud.points {
        *p = m.transform_point3(*p);
    }
    for n in cloud.normals.iter_mut().flatten() {
        *n = m.transform_vector3(*n);
    }
    cloud
}

/// `[tilt, yaw]` (degrees) of the camera placed in the world by `world`:
/// tilt > 0 looking down, yaw 0 looking along −X (at the turntable axis).
fn scanner_angles(world: &Affine3A) -> [f32; 2] {
    let f = world.transform_vector3(Vec3::Z);
    [
        (-f.z).clamp(-1.0, 1.0).asin().to_degrees(),
        f.y.atan2(-f.x).to_degrees(),
    ]
}

/// The acquisition seen from a rig job thread: hold, frames, stops. Cheap
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

    /// Frames processed per second (smoothed; 0 before two frames).
    pub fn fps(&self) -> f32 {
        self.0.live().fps
    }

    /// The up direction measured by the device IMU, smoothed over the
    /// recent frames (frame of the IMU), when the device reports it.
    pub fn imu_up(&self) -> Option<Vec3> {
        self.0.live().imu_up
    }

    /// The latency of the device ([`ScannerDevice::latency`]) when the
    /// acquisition started, before its frame rate is known: the
    /// conservative one the rig jobs settle on (the angle of each frame is
    /// read with the latency of the moment).
    pub fn latency(&self) -> Duration {
        self.0.live().latency
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

    /// Sends `request` and waits for its reply.
    fn ask(
        &self,
        request: RigRequest,
        timeout: Duration,
        cancel: &dyn Fn() -> bool,
    ) -> Result<RigReply> {
        {
            let mut io = self.0.io();
            *io = RigIo::default();
            io.request = Some(request);
        }
        let deadline = Instant::now() + timeout;
        loop {
            if let Some(reply) = self.0.io().reply.take() {
                return Ok(reply);
            }
            if let Err(e) = self.tick(deadline, cancel, "stop") {
                *self.0.io() = RigIo::default();
                return Err(e);
            }
        }
    }

    /// Captures a settled stop (see [`CaptureRequest`]) from the frames
    /// processed after this call.
    pub fn capture(
        &self,
        request: CaptureRequest,
        timeout: Duration,
        cancel: &dyn Fn() -> bool,
    ) -> Result<StopOutcome> {
        match self.ask(RigRequest::Capture(request), timeout, cancel)? {
            RigReply::Captured(s) => Ok(s),
            _ => Err(ScanError::Device("unexpected rig reply".into())),
        }
    }

    /// Closes the loop of the fused stops from index `first` (a full
    /// revolution, see `close_loop`).
    pub fn close_loop(
        &self,
        first: usize,
        timeout: Duration,
        cancel: &dyn Fn() -> bool,
    ) -> Result<LoopOutcome> {
        match self.ask(RigRequest::CloseLoop { first }, timeout, cancel)? {
            RigReply::Loop(l) => Ok(l),
            _ => Err(ScanError::Device("unexpected rig reply".into())),
        }
    }

    /// Ends the acquisition once a rig job has captured everything: a
    /// snapshot of the kept stops is returned, the device stops streaming
    /// (fill light off) and no frame is tracked any more, so nothing shows
    /// a scan under way while the job computes. The session reports
    /// [`ScanState::Stopped`] from then on and keeps the model for
    /// [`ScanSession::stop`].
    pub fn end_acquisition(
        &self,
        timeout: Duration,
        cancel: &dyn Fn() -> bool,
    ) -> Result<Arc<KeptStops>> {
        match self.ask(RigRequest::End, timeout, cancel)? {
            RigReply::Stops(s) => Ok(s),
            _ => Err(ScanError::Device("unexpected rig reply".into())),
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
        match self.live_state() {
            ScanState::Connected | ScanState::Stopped => {}
            ScanState::Idle => return Err(ScanError::NotConnected),
            state => {
                return Err(ScanError::InvalidState {
                    action: "start",
                    state,
                });
            }
        }
        // a stream that ended by itself (device lost, rig job) gives the
        // device and the model back first: the scan goes on from them
        self.join()?;
        let mut device = self.device.take().ok_or(ScanError::NotConnected)?;
        let started = self
            .settings
            .exposure_us
            .map_or(Ok(()), |us| device.set_exposure_us(us))
            .and_then(|()| device.set_fill_light(self.settings.fill_light))
            .and_then(|()| device.set_fast_exposure(self.settings.fast_exposure))
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
        self.shared.ended.store(false, Ordering::SeqCst);
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
        let state = self.live_state();
        if state != ScanState::Scanning {
            return Err(ScanError::InvalidState {
                action: "pause",
                state,
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
        let state = self.live_state();
        if state != ScanState::Paused {
            return Err(ScanError::InvalidState {
                action: "resume",
                state,
            });
        }
        self.shared.paused.store(false, Ordering::SeqCst);
        self.state = ScanState::Scanning;
        Ok(())
    }

    /// Ends the acquisition and returns the fused cloud (surface points with
    /// normals, in the scan world frame, see [`ScanStatus::rig_world`]).
    /// From `Stopped` it returns the same cloud again.
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
        let cloud = moved(fusion.volume.extract_cloud(), &fusion.world);
        let mut live = self.shared.live();
        let frame_points = live
            .preview
            .as_ref()
            .map_or_else(Arc::default, |p| Arc::clone(&p.frame_points));
        live.publish(
            Arc::new(cloud.clone()),
            frame_points,
            fusion.world * fusion.pose,
        );
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
        *self.shared.io() = RigIo::default();
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

    /// The state as it is: a rig job ([`RigLink::end_acquisition`]) or a
    /// failed stream may have ended the acquisition while the session still
    /// holds the thread, which is then `Stopped` already.
    fn live_state(&self) -> ScanState {
        match self.state {
            ScanState::Scanning | ScanState::Paused if self.shared.ended.load(Ordering::SeqCst) => {
                ScanState::Stopped
            }
            state => state,
        }
    }

    /// The stream failed by itself (the device was lost) and the session
    /// still holds that ended acquisition: [`ScanSession::stop`] takes its
    /// model back.
    pub(crate) fn stream_failed(&self) -> bool {
        self.worker.is_some()
            && self.shared.ended.load(Ordering::SeqCst)
            && self.shared.live().error.is_some()
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

    /// The stops kept by a stop-and-capture scan (`scan.auto`) for the fine
    /// reconstruction, once the acquisition stopped; `None` while it runs,
    /// when no stop was kept, or when frames were also fused one by one
    /// (continuous or hand-held scanning: the stops are not the whole scan).
    pub fn kept_stops(&self) -> Option<KeptStops> {
        if self.state != ScanState::Stopped {
            return None;
        }
        let fusion = self.fusion.as_ref()?;
        if fusion.stops.is_empty() || fusion.loose_frames > 0 {
            return None;
        }
        Some(kept_stops(fusion, self.rig_calibration(), &self.settings))
    }

    /// Sets the rotation from the scanner's IMU frame to its camera frame
    /// ([`RigCalibration::camera_from_imu`]): without a rig calibration in
    /// use, the world of the next scan then stands on the true up the IMU
    /// measures ([`gravity_world`]). Takes effect from the next frame.
    pub fn set_imu_alignment(&mut self, camera_from_imu: Option<Quat>) {
        self.shared.rig().camera_from_imu = camera_from_imu;
    }

    /// Current state and counters.
    pub fn status(&self) -> ScanStatus {
        let live = self.shared.live();
        ScanStatus {
            state: self.live_state(),
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
            world_from_camera: live.world.map(|w| w.to_cols_array()),
            rig_world: live.rig_world,
            imu_world: live.imu_world,
            scanner_deg: live.world.as_ref().map(scanner_angles),
            // the acquisition thread publishes it while it holds the device
            vendor: match &self.device {
                Some(device) => device.vendor_status(),
                None => live.vendor.clone(),
            },
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
        match self.live_state() {
            state @ (ScanState::Scanning | ScanState::Paused) => {
                Err(ScanError::InvalidState { action, state })
            }
            _ => Ok(()),
        }
    }

    /// Stops the thread and takes back the device and the fused model.
    fn join(&mut self) -> Result<()> {
        let Some(worker) = self.worker.take() else {
            return Ok(());
        };
        self.shared.stop.store(true, Ordering::SeqCst);
        self.shared.ended.store(false, Ordering::SeqCst);
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
                let mut live = self.shared.live();
                live.error = Some(("device", detail.clone()));
                live.vendor = None;
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

/// Turntable angles of the recent past, to read the angles of a frame at
/// the moment it shows ([`ScannerDevice::latency`] before its arrival).
#[derive(Default)]
struct AngleHistory(VecDeque<(Instant, (f32, f32))>);

impl AngleHistory {
    /// How long the history goes back.
    const SPAN: Duration = Duration::from_secs(4);

    fn push(&mut self, at: Instant, angles: Option<(f32, f32)>) {
        let Some(a) = angles else {
            self.0.clear();
            return;
        };
        self.0.push_back((at, a));
        while self
            .0
            .front()
            .is_some_and(|(t, _)| at.saturating_duration_since(*t) > Self::SPAN)
        {
            self.0.pop_front();
        }
    }

    /// The angles at `at`, interpolated (the oldest known before it).
    fn at(&self, at: Instant) -> Option<(f32, f32)> {
        let after = self.0.iter().position(|(t, _)| *t >= at);
        match after {
            Some(0) => self.0.front().map(|(_, a)| *a),
            Some(k) => {
                let (t0, a0) = self.0[k - 1];
                let (t1, a1) = self.0[k];
                let span = (t1 - t0).as_secs_f32();
                let f = if span > 0.0 {
                    (at - t0).as_secs_f32() / span
                } else {
                    1.0
                };
                Some((a0.0 + f * (a1.0 - a0.0), a0.1 + f * (a1.1 - a0.1)))
            }
            None => self.0.back().map(|(_, a)| *a),
        }
    }
}

/// Largest turn about the axis a stop registration may apply to the
/// turntable pose (degrees).
pub(crate) const STOP_MAX_DELTA_DEG: f64 = 1.5;
/// Largest turn off the axis a stop registration may apply (degrees).
pub(crate) const STOP_MAX_OFF_AXIS_DEG: f64 = 1.0;
/// Largest shift of the axis point a stop registration may apply (mm).
pub(crate) const STOP_MAX_SHIFT_MM: f64 = 1.5;
/// A loop error beyond this (degrees) is not spread: something slipped.
const LOOP_MAX_DEG: f32 = 3.0;
/// A loop error below this (degrees) is left as is.
const LOOP_MIN_DEG: f32 = 0.02;
/// Fewest stops of a revolution to tell static clutter from the object.
const MIN_CLUTTER_STOPS: usize = 8;
/// Static clutter is not removed when it is more than this fraction of
/// the stop pixels (the object itself looks the same at every angle).
const MAX_CLUTTER_FRACTION: f32 = 0.3;
/// Fraction of the stops of a revolution in which a depth must repeat at
/// a pixel to be static clutter: higher than the calibration's
/// background, so the object's own near-axis surfaces stay.
const CLUTTER_MIN_FRACTION: f32 = 0.85;
/// Turntable motion (degrees) above which a frame must show the object
/// turning; a tracked frame that barely moves is locked on the static
/// scene.
const MIN_CHECKED_TURN_DEG: f32 = 1.0;

/// `true` when the table turned by at least [`MIN_CHECKED_TURN_DEG`]
/// between the angles `prev` and `now` but the camera pose `pose` turned
/// by less than a third of it from `prev_pose`: the frame locked onto the
/// static scene (enclosure, desk) instead of the turning object. The
/// rotation angle of a motion does not depend on its axis, so no
/// calibration is needed.
fn locked_on_static(
    prev_pose: &Affine3A,
    pose: &Affine3A,
    prev: Option<(f32, f32)>,
    now: Option<(f32, f32)>,
) -> bool {
    let turn = match (now, prev) {
        (Some(n), Some(p)) => (n.0 - p.0).abs().max((n.1 - p.1).abs()),
        _ => 0.0,
    };
    let moved = rotation_angle(&(to_d(prev_pose).inverse() * to_d(pose))).to_degrees() as f32;
    turn >= MIN_CHECKED_TURN_DEG && moved < 0.3 * turn
}

/// Tracking parameters of the frames and the stops: residuals up to 0.3
/// mm (a MINI 2 frame of a turning object shows 0.18-0.2 mm, a still one
/// 0.1 mm).
fn track_params(prior: bool) -> TrackParams {
    let p = TrackParams {
        max_rms_mm: 0.3,
        ..TrackParams::default()
    };
    if prior { p.with_prior() } else { p }
}

/// Samples `depth` for tracking, the points on the platter (with a
/// calibration) marked as not counting.
fn sample(
    depth: &[f32],
    k: &CameraIntrinsics,
    params: &TrackParams,
    platter: Option<(&RigGeometry, f32)>,
) -> FrameSample {
    let mut s = sample_frame(depth, k, params);
    if let Some((g, tilt)) = platter {
        s.excluded = s.points.iter().map(|p| g.on_platter(*p, tilt)).collect();
    }
    s
}

/// Camera pose of the turntable angles `angles` in the world of `fusion`.
fn rig_pose(g: &RigGeometry, world: (f32, f32), angles: (f32, f32)) -> Affine3A {
    g.platter(world.0, world.1) * g.platter(angles.0, angles.1).inverse()
}

/// Registers, fuses and records the stop image `depth` (see
/// [`CaptureRequest`]).
fn fuse_stop(
    fusion: &mut Fusion,
    geometry: Option<&RigGeometry>,
    depth: Arc<Vec<f32>>,
    k: CameraIntrinsics,
    angles: (f32, f32),
) -> StopOutcome {
    let mut out = StopOutcome {
        depth: Arc::clone(&depth),
        intrinsics: k,
        index: None,
        flagged: false,
        delta_deg: 0.0,
        rms_mm: None,
        prior_fit: None,
    };
    let params = track_params(true);
    let s = sample(&depth, &k, &params, geometry.map(|g| (g, angles.1)));
    let pose = if !fusion.started {
        if s.points.is_empty() {
            return out;
        }
        fusion.set_world(geometry, Some(angles));
        Affine3A::IDENTITY
    } else {
        let prior = match (geometry, fusion.world_angles) {
            (Some(g), Some(w)) => rig_pose(g, w, angles),
            _ => fusion.stops.last().map_or(fusion.pose, |s| s.pose),
        };
        let r = track_frame(&fusion.volume, &s, &k, &prior, &params);
        out.rms_mm = r.rms_mm.is_finite().then_some(r.rms_mm);
        match geometry {
            Some(g) => {
                let (delta, off, shift) = stop_correction(g, &prior, &r.pose, angles.1);
                let accept = r.ok
                    && delta.abs() <= STOP_MAX_DELTA_DEG
                    && off <= STOP_MAX_OFF_AXIS_DEG
                    && shift <= STOP_MAX_SHIFT_MM;
                out.flagged = !accept;
                out.delta_deg = delta as f32;
                if accept {
                    r.pose
                } else {
                    // never drop a stop: the turntable pose is exact to
                    // the recorded data (a full turn ends on the very first
                    // view); its residual is the one reported
                    let at_prior = TrackParams {
                        iterations_per_stage: 0,
                        ..params
                    };
                    let r0 = track_frame(&fusion.volume, &s, &k, &prior, &at_prior);
                    out.rms_mm = r0.rms_mm.is_finite().then_some(r0.rms_mm);
                    let counted = s.counted();
                    out.prior_fit = (counted > 0).then(|| r0.inliers as f32 / counted as f32);
                    prior
                }
            }
            None if r.ok => r.pose,
            None => return out,
        }
    };
    fusion.volume.integrate(&depth, &k, &pose);
    fusion.started = true;
    fusion.pose = pose;
    fusion.velocity = Affine3A::IDENTITY;
    fusion.angles = Some(angles);
    fusion.stops.push(StopRecord {
        pixels: Arc::new(
            depth
                .iter()
                .enumerate()
                .filter(|(_, d)| d.is_finite() && **d > 0.0)
                .map(|(i, d)| (i as u32, *d))
                .collect(),
        ),
        intrinsics: k,
        angles,
        pose,
        flagged: out.flagged,
    });
    out.index = Some(fusion.stops.len() - 1);
    out
}

/// Closes the loop of the stops from `first` (one revolution at one tilt):
///
/// * when the last stop is a full turn after the first (same view), it is
///   registered onto the first one alone; the turn about the axis that
///   this finds is spread linearly over the stops of the revolution (a 1-D
///   pose graph with equal weights on the stop-to-stop edges);
/// * the static clutter of the revolution (a depth repeated at the same
///   pixel in most stops, see [`Background`]: enclosure, stand, artefacts
///   that the turntable pose smears into rings) is removed from its stops,
///   unless that would take more than [`MAX_CLUTTER_FRACTION`] of their
///   pixels (an object turning into itself, like a centred cylinder);
///
/// and the model is rebuilt from the stops.
fn close_loop(
    fusion: &mut Fusion,
    geometry: Option<&RigGeometry>,
    settings: &ScanSettings,
    first: usize,
) -> LoopOutcome {
    let mut out = LoopOutcome {
        error_deg: None,
        applied: false,
        clutter_pixels: 0,
    };
    let Some(g) = geometry else {
        return out;
    };
    let first = first.min(fusion.stops.len());
    let stops = &fusion.stops[first..];
    let n = stops.len().saturating_sub(1);
    let (Some(s0), Some(sn)) = (stops.first(), stops.last()) else {
        return out;
    };
    let mut error = 0.0;
    if n >= 3
        && ((sn.angles.0 - s0.angles.0).abs() - 360.0).abs() <= 1.0
        && (sn.angles.1 - s0.angles.1).abs() <= 0.1
    {
        let mut v0 = TsdfVolume::new(settings.tsdf());
        v0.integrate(&s0.depth(), &s0.intrinsics, &s0.pose);
        let params = track_params(true);
        let depth = sn.depth();
        let s = sample(&depth, &sn.intrinsics, &params, Some((g, sn.angles.1)));
        let r = track_frame(&v0, &s, &sn.intrinsics, &sn.pose, &params);
        if r.ok {
            let (_, dir) = g.axis_at(sn.angles.1);
            let c = to_d(&sn.pose).inverse() * to_d(&r.pose);
            error = angle_about(&c.matrix3, dir.as_dvec3()).to_degrees() as f32;
            out.error_deg = Some(error);
        }
    }
    // a model fused frame by frame cannot be rebuilt from the stops
    if fusion.loose_frames > 0 {
        return out;
    }
    let spread = (LOOP_MIN_DEG..=LOOP_MAX_DEG).contains(&error.abs());
    if spread {
        let (point, dir) = g.axis_at(stops[0].angles.1);
        for (i, s) in fusion.stops[first..].iter_mut().enumerate() {
            s.pose *= about(point, dir, error * i as f32 / n as f32);
        }
    }
    let pass = &fusion.stops[first..];
    if pass.len() >= MIN_CLUTTER_STOPS {
        let frames: Vec<Arc<Vec<f32>>> = pass.iter().map(|s| Arc::new(s.depth())).collect();
        let background = Background::from_frames_with(
            &frames,
            crate::rig::calib::BACKGROUND_TOL_MM,
            CLUTTER_MIN_FRACTION,
        );
        let total: usize = pass.iter().map(|s| s.pixels.len()).sum();
        let clutter: usize = pass
            .iter()
            .map(|s| {
                s.pixels
                    .iter()
                    .filter(|(i, d)| background.is_static(*i as usize, *d))
                    .count()
            })
            .sum();
        if clutter > 0 && (clutter as f32) <= MAX_CLUTTER_FRACTION * total as f32 {
            for s in &mut fusion.stops[first..] {
                Arc::make_mut(&mut s.pixels)
                    .retain(|(i, d)| !background.is_static(*i as usize, *d));
            }
            out.clutter_pixels = clutter;
        }
    }
    if !spread && out.clutter_pixels == 0 {
        return out;
    }
    let mut volume = TsdfVolume::new(settings.tsdf());
    for s in &fusion.stops {
        volume.integrate(&s.depth(), &s.intrinsics, &s.pose);
    }
    fusion.volume = volume;
    if let Some(last) = fusion.stops.last() {
        fusion.pose = last.pose;
    }
    out.applied = true;
    out
}

/// One stop of [`fuse_recorded_stops`].
#[derive(Clone, Debug, PartialEq)]
pub struct ReplayedStop {
    /// The stop was fused.
    pub fused: bool,
    /// Its registration was refused: fused at the turntable pose.
    pub flagged: bool,
    /// Correction of the turntable pose about the axis (degrees).
    pub delta_deg: f32,
    /// RMS registration residual (mm).
    pub rms_mm: Option<f32>,
    /// A refused registration: the fraction of its samples on the model at
    /// the turntable pose (low: the view is not where the rig calibration
    /// puts it).
    pub prior_fit: Option<f32>,
}

/// Outcome of [`fuse_recorded_stops`].
#[derive(Clone, Debug)]
pub struct ReplayedScan {
    /// Per stop.
    pub stops: Vec<ReplayedStop>,
    /// Loop closure error of the revolution (degrees), when measured.
    pub loop_error_deg: Option<f32>,
    /// The model was rebuilt (loop error spread, static clutter removed).
    pub loop_applied: bool,
    /// Static clutter pixels removed from the stops.
    pub clutter_pixels: usize,
    /// The fused cloud, in the scan world frame (`world`).
    pub cloud: PointCloud,
    /// The scan world frame from the camera frame of the first stop.
    pub world: Affine3A,
    /// The fused stops, as `scan.auto` keeps them for the fine
    /// reconstruction ([`crate::reconstruct`]).
    pub kept: KeptStops,
    /// The calibration check of `scan.auto` ([`crate::rig::CalibrationCheck`])
    /// found the calibration outdated on the first views of the first
    /// pass: the live scan would have stopped there with
    /// `calibration-outdated`.
    pub calibration_outdated: bool,
}

/// A recorded stop: turntable angles `(rotate, tilt)` and the
/// range-filtered median depth image (see
/// [`crate::rig::calib::median_depth`]).
pub type RecordedStop = ((f32, f32), Arc<Vec<f32>>);

/// Runs recorded stop images through the stop-and-capture fusion of
/// `scan.auto`, offline: region of interest, registration from the
/// turntable pose of `calibration`, fusion, the calibration check on the
/// first pass (the recording goes on where the live scan would stop),
/// then a loop closure over all the stops. For tests and tuning on
/// recorded data.
pub fn fuse_recorded_stops(
    settings: &ScanSettings,
    calibration: &RigCalibration,
    intrinsics: &CameraIntrinsics,
    stops: &[RecordedStop],
) -> ReplayedScan {
    let g = RigGeometry::new(calibration);
    let mut fusion = Fusion::new(settings);
    let k = *intrinsics;
    let w = k.width as usize;
    let mut out = Vec::new();
    let mut check = CalibrationCheck::default();
    let first_tilt = stops.first().map(|(a, _)| a.1);
    let mut first_pass = true;
    for (angles, depth) in stops {
        first_pass &= Some(angles.1) == first_tilt;
        let cropped: Vec<f32> = depth
            .iter()
            .enumerate()
            .map(|(i, &d)| {
                let p = k.ray((i % w) as f32, (i / w) as f32) * d;
                let keep = d >= settings.min_depth_mm
                    && d <= settings.max_depth_mm
                    && (!settings.use_roi
                        || g.roi(p, angles.1, settings.roi_radius_mm, settings.roi_height_mm));
                if keep { d } else { f32::NAN }
            })
            .collect();
        let s = fuse_stop(&mut fusion, Some(&g), Arc::new(cropped), k, *angles);
        // a view registered after the world view, as `scan.auto` judges it
        if first_pass && s.index.is_some_and(|i| i > 0) {
            check.note(s.flagged, s.prior_fit);
        }
        out.push(ReplayedStop {
            fused: s.index.is_some(),
            flagged: s.flagged,
            delta_deg: s.delta_deg,
            rms_mm: s.rms_mm,
            prior_fit: s.prior_fit,
        });
    }
    let closed = close_loop(&mut fusion, Some(&g), settings, 0);
    ReplayedScan {
        stops: out,
        loop_error_deg: closed.error_deg,
        loop_applied: closed.applied,
        clutter_pixels: closed.clutter_pixels,
        cloud: moved(fusion.volume.extract_cloud(), &fusion.world),
        world: fusion.world,
        kept: kept_stops(&fusion, Some(calibration.clone()), settings),
        calibration_outdated: check.outdated(),
    }
}

/// Body of the acquisition thread.
fn acquire(
    mut device: Box<dyn ScannerDevice>,
    mut fusion: Fusion,
    shared: &Shared,
    settings: &ScanSettings,
) -> (Box<dyn ScannerDevice>, Fusion) {
    let mut cloud: Arc<PointCloud> = shared
        .live()
        .preview
        .as_ref()
        .map_or_else(Arc::default, |p| Arc::clone(&p.cloud));
    let mut extracted_at = shared.live().fused_frames;
    let mut last_publish: Option<Instant> = None;
    let mut last_frame: Option<Instant> = None;
    // what the device did when its stream started; again at the first frame
    // (a restart reopens the device inside `next_frame`) and at the end
    let mut started = false;
    {
        let mut live = shared.live();
        live.latency = device.latency();
        live.vendor = device.vendor_status();
    }
    let mut history = AngleHistory::default();
    let probe_angles = |probe: &Option<dc_turntable::AngleProbe>| {
        probe
            .as_ref()
            .map(|p| p())
            .and_then(|a| Some((a.rotate_deg?, a.tilt_deg.unwrap_or(0.0))))
    };
    while !shared.stop.load(Ordering::SeqCst) {
        let (geometry, probe, camera_from_imu) = {
            let rig = shared.rig();
            (rig.geometry, rig.probe.clone(), rig.camera_from_imu)
        };
        // a loop closure needs no frame (the request is copied out: the
        // lock must not be held while serving it)
        let request = shared.io().request;
        if let Some(RigRequest::CloseLoop { first }) = request {
            let out = close_loop(&mut fusion, geometry.as_ref(), settings, first);
            let mut io = shared.io();
            io.request = None;
            io.reply = Some(RigReply::Loop(out));
            drop(io);
            if out.applied {
                // the next refresh extracts the rebuilt model
                extracted_at = u64::MAX;
            }
        }
        if request == Some(RigRequest::End) {
            let calibration = shared.rig().calibration.clone();
            let stops = Arc::new(kept_stops(&fusion, calibration, settings));
            // the stream ends now (fill light off): nothing is tracked while
            // the job computes
            device.stop();
            shared.live().vendor = device.vendor_status();
            shared.ended.store(true, Ordering::SeqCst);
            let mut io = shared.io();
            io.request = None;
            io.reply = Some(RigReply::Stops(stops));
            break;
        }
        history.push(Instant::now(), probe_angles(&probe));
        let frame = match device.next_frame(FRAME_WAIT) {
            Ok(Some(frame)) => frame,
            Ok(None) => continue,
            Err(e) => {
                let mut live = shared.live();
                live.error = Some((e.code(), e.to_string()));
                live.vendor = device.vendor_status();
                break;
            }
        };
        if !started {
            started = true;
            shared.live().vendor = device.vendor_status();
        }
        let arrived = Instant::now();
        history.push(arrived, probe_angles(&probe));
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
        // the true up of this frame in the camera, from the IMU
        if let Some(u) = frame.imu_up {
            let mut live = shared.live();
            live.imu_up = Some(live.imu_up.map_or(u, |a| {
                a.lerp(u, IMU_UP_SMOOTHING).try_normalize().unwrap_or(u)
            }));
        }
        fusion.up = camera_from_imu
            .zip(shared.live().imu_up)
            .map(|(r, u)| r * u);
        let paused = shared.paused.load(Ordering::SeqCst) || shared.hold.load(Ordering::SeqCst);
        // the turntable angles when the frame was taken (the latency of a
        // device may follow its frame rate)
        let angles = history.at(arrived.checked_sub(device.latency()).unwrap_or(arrived));
        let request = shared.io().request;
        let capture = match request {
            Some(RigRequest::Capture(c)) => Some(c),
            _ => None,
        };
        let tilt = capture.map_or(angles.map_or(0.0, |a| a.1), |c| c.angles.1);
        let roi = geometry.filter(|_| settings.use_roi).map(|g| (g, tilt));
        let depth = filter_depth(&frame, settings, roi.as_ref());
        let mut stop = None;
        if let Some(c) = capture {
            let mut io = shared.io();
            if io.skipped < c.skip {
                io.skipped += 1;
            } else {
                io.collected.push(Arc::new(depth.clone()));
                if io.collected.len() >= c.frames.max(1) {
                    let frames = std::mem::take(&mut io.collected);
                    io.request = None;
                    drop(io);
                    let median = Arc::new(crate::rig::calib::median_depth(&frames));
                    let out = if c.fuse {
                        fuse_stop(&mut fusion, geometry.as_ref(), median, k, c.angles)
                    } else {
                        StopOutcome {
                            depth: median,
                            intrinsics: k,
                            index: None,
                            flagged: false,
                            delta_deg: 0.0,
                            rms_mm: None,
                            prior_fit: None,
                        }
                    };
                    stop = Some(out.index.is_some());
                    shared.io().reply = Some(RigReply::Captured(out));
                }
            }
        }
        let prior = geometry.is_some() && angles.is_some() && fusion.angles.is_some();
        let params = track_params(prior);
        let sample = sample(&depth, &k, &params, geometry.as_ref().map(|g| (g, tilt)));
        let tracked = if stop == Some(true) {
            // the stop was just registered and fused: it is this frame's pose
            Some(fusion.pose)
        } else if !fusion.started {
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
            let r = track_frame(&fusion.volume, &sample, &k, &guess, &params);
            let still = locked_on_static(&fusion.pose, &r.pose, fusion.angles, angles);
            (r.ok && !still).then_some(r.pose)
        };
        let fused = match (tracked, stop) {
            (_, Some(true)) => false,
            (Some(pose), _) => {
                fusion.velocity = fusion.pose.inverse() * pose;
                fusion.pose = pose;
                fusion.angles = angles;
                if paused || (!fusion.started && sample.points.is_empty()) {
                    false
                } else {
                    if !fusion.started {
                        fusion.set_world(geometry.as_ref(), angles);
                    }
                    fusion.volume.integrate(&depth, &k, &pose);
                    fusion.started = true;
                    fusion.loose_frames += 1;
                    true
                }
            }
            (None, _) => {
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
        if fused || stop == Some(true) {
            live.fused_frames += 1;
            if live.fused_frames == 1 {
                live.world_angles = fusion.world_angles.map(|(r, t)| [r, t]);
                live.world = Some(fusion.world);
                live.rig_world = fusion.rig_world;
                live.imu_world = fusion.imu_world;
            }
        }
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
                cloud = Arc::new(moved(fusion.volume.extract_cloud(), &fusion.world));
                extracted_at = fused_frames;
            }
            let camera = fusion.world * tracked.unwrap_or(fusion.pose);
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
    // nothing streams any more: no frame rate, nothing tracked
    let mut live = shared.live();
    live.fps = 0.0;
    live.tracking_ok = false;
    if live.error.is_some() {
        // the stream failed (device lost): the session is no longer
        // scanning; its model waits to be joined, as after a rig job
        shared.ended.store(true, Ordering::SeqCst);
    }
    drop(live);
    (device, fusion)
}

/// Depth with the values outside the working range or outside the
/// turntable region of interest `roi` (geometry and tilt) marked invalid
/// (`NaN`).
fn filter_depth(
    frame: &DepthFrame,
    settings: &ScanSettings,
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

    #[test]
    fn a_lost_device_ends_the_scan_by_itself() {
        let mut s = ScanSession::new(ScanSettings::default());
        let backend = SimBackend::new(SimOptions {
            realtime: false,
            lost_after: Some(6),
            width: 160,
            height: 100,
            ..SimOptions::default()
        });
        s.connect_with(&backend, UPPER_ID).unwrap();
        s.start().unwrap();
        let deadline = Instant::now() + Duration::from_secs(60);
        while s.status().state != ScanState::Stopped {
            assert!(Instant::now() < deadline, "{:?}", s.status());
            std::thread::sleep(Duration::from_millis(5));
        }
        // no scan under way any more: no frame rate, nothing tracked, the
        // reason given
        let st = s.status();
        assert_eq!(st.frames, 6);
        assert_eq!(st.fps, 0.0);
        assert!(!st.tracking_ok);
        assert_eq!(st.error.as_deref(), Some("device-lost"));
        assert!(s.stream_failed());
        assert_eq!(s.pause().unwrap_err().code(), "invalid-state");
        // the settings may change, as with any stopped scan
        s.set_settings(ScanSettings::default()).unwrap();
        // stop takes back what was fused
        let cloud = s.stop().unwrap();
        assert!(!cloud.is_empty());
        assert!(!s.stream_failed());
        assert_eq!(s.status().state, ScanState::Stopped);
        // the scan may go on (the device is still lost: it ends again)
        s.start().unwrap();
        let deadline = Instant::now() + Duration::from_secs(60);
        while !s.stream_failed() {
            assert!(Instant::now() < deadline, "{:?}", s.status());
            std::thread::sleep(Duration::from_millis(5));
        }
        // and start again without a stop in between
        s.start().unwrap();
        s.reset();
        assert_eq!(s.status().state, ScanState::Connected);
    }

    /// A simulated device with a vendor layer: its light follows the stream.
    struct VendorSim {
        inner: Box<dyn ScannerDevice>,
        streaming: bool,
    }

    impl ScannerDevice for VendorSim {
        fn info(&self) -> DeviceInfo {
            self.inner.info()
        }

        fn start(&mut self) -> Result<()> {
            self.inner.start()?;
            self.streaming = true;
            Ok(())
        }

        fn stop(&mut self) {
            self.inner.stop();
            self.streaming = false;
        }

        fn next_frame(&mut self, timeout: Duration) -> Result<Option<DepthFrame>> {
            self.inner.next_frame(timeout)
        }

        fn vendor_status(&self) -> Option<VendorStatus> {
            Some(VendorStatus {
                fill_light: if self.streaming { "on" } else { "off" }.into(),
                fill_light_note: None,
                fast_exposure: "applied".into(),
                fast_exposure_note: None,
                calibration: "factory",
                calibration_note: None,
                fx: 1142.37,
                fy: 1142.37,
                cx: 163.19,
                cy: 329.85,
            })
        }
    }

    struct VendorSimBackend(SimBackend);

    impl ScannerBackend for VendorSimBackend {
        fn id(&self) -> &'static str {
            "vendor-sim"
        }

        fn list(&self) -> Result<Vec<DeviceInfo>> {
            self.0.list()
        }

        fn open(&self, device_id: &str) -> Result<Box<dyn ScannerDevice>> {
            Ok(Box::new(VendorSim {
                inner: self.0.open(device_id)?,
                streaming: false,
            }))
        }
    }

    #[test]
    fn the_status_reports_the_vendor_layer_in_every_state() {
        let light = |s: &ScanSession| s.status().vendor.map(|v| v.fill_light.into_owned());
        let mut s = ScanSession::new(ScanSettings::default());
        // a device without a vendor layer
        s.connect_with(&sim(4), UPPER_ID).unwrap();
        assert_eq!(s.status().vendor, None);
        // connected: the session holds the device
        s.connect_with(&VendorSimBackend(sim(1000)), UPPER_ID)
            .unwrap();
        assert_eq!(light(&s).as_deref(), Some("off"));
        let json = serde_json::to_value(s.status()).unwrap();
        assert_eq!(json["vendor"]["fill_light"], "off");
        assert_eq!(json["vendor"]["calibration"], "factory");
        // scanning: the acquisition thread publishes it
        s.start().unwrap();
        wait_frames(&s, 2);
        let v = s.status().vendor.unwrap();
        assert_eq!(
            (&*v.fill_light, &*v.fast_exposure, v.calibration),
            ("on", "applied", "factory")
        );
        assert_eq!((v.fx, v.cx, v.cy), (1142.37, 163.19, 329.85));
        // stopped: the device is back, its light off
        s.stop().unwrap();
        assert_eq!(light(&s).as_deref(), Some("off"));
        s.disconnect();
        assert_eq!(s.status().vendor, None);
        assert!(serde_json::to_value(s.status()).unwrap()["vendor"].is_null());
    }

    #[test]
    fn a_frame_locked_on_the_static_scene_is_lost() {
        use crate::sim::{SimBackend, SimDevice, typodont_arch};
        use dc_turntable::sim::{RigPose, SimRig};
        let options = SimOptions {
            realtime: false,
            width: 200,
            height: 125,
            ..SimOptions::default()
        };
        let info = SimBackend::default().list().unwrap().remove(0);
        let dev = SimDevice::new(info, &typodont_arch(true), options)
            .unwrap()
            .with_rig(SimRig::new())
            .unwrap();
        let at = |deg: f32| {
            dev.render_rig(
                0,
                RigPose {
                    rotate_deg: deg,
                    tilt_deg: 0.0,
                },
            )
        };
        let (f0, f1) = (at(0.0), at(8.0));
        let k = f0.intrinsics;
        let mut volume = TsdfVolume::new(ScanSettings::default().tsdf());
        volume.integrate(&f0.depth_mm, &k, &Affine3A::IDENTITY);
        // no region of interest, no prior: the enclosure holds the pose
        let params = track_params(false);
        let s = sample_frame(&f1.depth_mm, &k, &params);
        let r = track_frame(&volume, &s, &k, &Affine3A::IDENTITY, &params);
        let moved = rotation_angle(&to_d(&r.pose)).to_degrees();
        assert!(moved < 2.4, "the solve followed the object: {moved} deg");
        assert!(locked_on_static(
            &Affine3A::IDENTITY,
            &r.pose,
            Some((0.0, 0.0)),
            Some((8.0, 0.0))
        ));
        // a pose that turned with the table is not locked; a still table
        // proves nothing
        let turned = Affine3A::from_rotation_y(8f32.to_radians());
        assert!(!locked_on_static(
            &Affine3A::IDENTITY,
            &turned,
            Some((0.0, 0.0)),
            Some((8.0, 0.0))
        ));
        assert!(!locked_on_static(
            &Affine3A::IDENTITY,
            &Affine3A::IDENTITY,
            Some((8.0, 0.0)),
            Some((8.5, 0.0))
        ));
    }

    #[test]
    fn angles_are_read_back_at_the_moment_a_frame_shows() {
        let t0 = Instant::now();
        let mut h = AngleHistory::default();
        for k in 0..=10 {
            h.push(t0 + Duration::from_millis(100 * k), Some((k as f32, 0.0)));
        }
        let a = h.at(t0 + Duration::from_millis(250)).unwrap();
        assert!((a.0 - 2.5).abs() < 1e-3, "{a:?}");
        assert_eq!(h.at(t0).unwrap().0, 0.0);
        assert_eq!(h.at(t0 + Duration::from_secs(5)).unwrap().0, 10.0);
        h.push(t0 + Duration::from_secs(2), None);
        assert_eq!(h.at(t0), None);
    }

    #[test]
    fn the_world_stands_on_the_rig_then_the_imu_then_the_camera() {
        let mut f = Fusion::new(&ScanSettings::default());
        f.set_world(None, None);
        assert_eq!(f.world, camera_world());
        assert!(!f.rig_world && !f.imu_world);
        // the IMU measures a scanner looking 20 degrees down
        let a = 20_f32.to_radians();
        f.up = Some(Vec3::new(0.0, -a.cos(), -a.sin()));
        f.set_world(None, None);
        assert_eq!(f.world, gravity_world(f.up.unwrap()));
        assert!(!f.rig_world && f.imu_world);
        let [tilt, yaw] = scanner_angles(&f.world);
        assert!(
            (tilt - 20.0).abs() < 1e-3 && yaw.abs() < 1e-3,
            "{tilt} {yaw}"
        );
        // a rig calibration wins
        let c = RigCalibration {
            axis_point: [0.0, 0.0, 150.0],
            axis_dir: [0.0, 0.0, -1.0],
            platter_up: [0.0, 0.0, -1.0],
            tilt_axis_point: None,
            tilt_axis_dir: None,
            scanner_serial: None,
            turntable_id: "T".into(),
            created: String::new(),
            fit_rms_deg: 0.0,
            fit_rms_mm: 0.0,
            angle_gain: 1.0,
            fit_pairs: 0,
            imu_up: None,
        };
        let g = RigGeometry::new(&c);
        f.set_world(Some(&g), Some((0.0, 0.0)));
        assert_eq!(f.world, g.world_from_camera(0.0));
        assert!(f.rig_world && !f.imu_world);
    }
}
