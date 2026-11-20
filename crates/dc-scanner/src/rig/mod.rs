//! Turntable scanning rig (alpha): a fixed scanner in an enclosure looking
//! at an object on a motorised turntable ([`dc_turntable`]).
//!
//! * **Calibration** ([`RigCalibration`]): the turntable axes in the
//!   scanner camera frame. It is measured without tools by
//!   `turntable.calibrate` (see [`jobs`] and [`calib`]): a static object on
//!   the platter is turned in settled stops, consecutive stops are
//!   registered onto each other, and the screw axis is fitted to those
//!   motions ([`fit::fit_motions`]); a result that fails its checks is
//!   refused. It is stored as JSON in
//!   `<data dir>/dentalcraft/rig/calibration.json`.
//! * **Region of interest**: with a calibration, every depth frame is
//!   cropped to a cylinder around the turntable axis ([`RigGeometry::roi`]),
//!   from just under the platter up to [`crate::ScanSettings::roi_height_mm`],
//!   so the static enclosure never pulls the tracking.
//! * **Motion prior**: with a calibration and a live angle estimate of the
//!   table ([`dc_turntable::AngleProbe`]), the motion of the object since
//!   the previous frame, predicted from the angles, is the initial guess of
//!   the tracking ([`RigGeometry::platter`]).
//! * **Jobs**: `scan.auto` (for each tilt: stop-and-capture around the
//!   table, or a continuous turn) and `turntable.calibrate` run on their
//!   own thread ([`jobs`]).
//!
//! The camera frame is the frame of the depth images: x right, y down,
//! z forward, millimetres.

pub mod calib;
pub(crate) mod control;
pub mod fit;
pub(crate) mod jobs;

use std::path::{Path, PathBuf};
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use dc_turntable::{AngleProbe, AxisInfo, TurntableInfo};
use glam::{Affine3A, Mat3, Quat, Vec3};
use serde::{Deserialize, Serialize};
use serde_json::{Value, json};

use crate::{Result, ScanError};

pub use control::turntable_devices;
pub use jobs::{CHECK_MIN_CONTRADICTING, CHECK_MIN_PRIOR_FIT, CHECK_VIEWS, CalibrationCheck};

/// Default radius of the region of interest around the axis (mm).
pub const DEFAULT_ROI_RADIUS_MM: f32 = 45.0;
/// Default height of the region of interest above the platter (mm).
pub const DEFAULT_ROI_HEIGHT_MM: f32 = 60.0;
/// The region of interest starts this far under the platter plane (mm),
/// so the base of the object stays in.
pub const ROI_BELOW_PLATTER_MM: f32 = 1.0;
/// Frame points less than this above the platter plane (mm) do not count
/// as evidence of a good pose: a plane pairs under any turn.
pub const PLATTER_BAND_MM: f32 = 1.5;
/// Name of the calibration file in the rig directory.
pub const CALIBRATION_FILE: &str = "calibration.json";

/// Turntable axes in the scanner camera frame (millimetres).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct RigCalibration {
    /// Point of the rotation axis on the platter plane.
    pub axis_point: [f32; 3],
    /// Unit rotation axis: positive turntable angles turn right-handed
    /// about it.
    pub axis_dir: [f32; 3],
    /// Unit normal of the platter towards the scanner (`axis_dir` or its
    /// opposite).
    pub platter_up: [f32; 3],
    /// Point of the tilt axis, when the table tilts.
    #[serde(default)]
    pub tilt_axis_point: Option<[f32; 3]>,
    /// Unit tilt axis: positive tilts turn right-handed about it.
    #[serde(default)]
    pub tilt_axis_dir: Option<[f32; 3]>,
    /// Serial number of the scanner it was measured with.
    pub scanner_serial: Option<String>,
    /// Id of the turntable it was measured with.
    pub turntable_id: String,
    /// When it was measured (RFC 3339, UTC).
    pub created: String,
    /// RMS difference between the fitted and the commanded rotation angles
    /// (degrees): the quality of the fit.
    #[serde(default)]
    pub fit_rms_deg: f32,
    /// RMS residual of the axis position (mm).
    #[serde(default)]
    pub fit_rms_mm: f32,
    /// Measured over commanded rotation angle (1 for an exact table).
    #[serde(default = "one")]
    pub angle_gain: f32,
    /// Registered stop pairs the axis was fitted to.
    #[serde(default)]
    pub fit_pairs: u32,
    /// The up direction measured by the scanner's IMU while the
    /// calibration was measured (unit, frame of the IMU), the platter level
    /// at tilt 0: with `platter_up`, the true up in the camera frame, it
    /// orients the IMU in the camera ([`RigCalibration::camera_from_imu`]).
    /// `None` for a scanner without an IMU or an older calibration.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub imu_up: Option<[f32; 3]>,
}

fn one() -> f32 {
    1.0
}

impl RigCalibration {
    /// `true` when it was measured with this scanner and this turntable (a
    /// missing serial matches any scanner).
    pub fn matches(&self, scanner_serial: Option<&str>, turntable_id: &str) -> bool {
        self.turntable_id == turntable_id
            && match (self.scanner_serial.as_deref(), scanner_serial) {
                (Some(a), Some(b)) => a == b,
                _ => true,
            }
    }

    /// The rotation from the frame of the scanner's IMU to its camera
    /// frame, estimated from `imu_up` and `platter_up`, the same direction
    /// (up) seen by the two. One direction cannot fix the turn about
    /// itself, so the IMU axes are taken along the camera axes: the
    /// rotation between the axes that brings `imu_up` closest to
    /// `platter_up`, then the shortest turn that makes them agree. `None`
    /// without `imu_up`.
    pub fn camera_from_imu(&self) -> Option<Quat> {
        let imu = Vec3::from(self.imu_up?).try_normalize()?;
        let up = Vec3::from(self.platter_up).try_normalize()?;
        let axes = axis_rotations()
            .into_iter()
            .max_by(|a, b| (*a * imu).dot(up).total_cmp(&(*b * imu).dot(up)))?;
        Some((Quat::from_rotation_arc(axes * imu, up) * axes).normalize())
    }
}

/// The 24 rotations that take the coordinate axes onto the coordinate axes.
fn axis_rotations() -> Vec<Quat> {
    let axes = [
        Vec3::X,
        Vec3::Y,
        Vec3::Z,
        Vec3::NEG_X,
        Vec3::NEG_Y,
        Vec3::NEG_Z,
    ];
    let mut out = Vec::with_capacity(24);
    for x in axes {
        for y in axes.into_iter().filter(|y| x.dot(*y) == 0.0) {
            out.push(Quat::from_mat3(&Mat3::from_cols(x, y, x.cross(y))));
        }
    }
    out
}

/// Kind of a rig job.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum RigJobKind {
    /// `scan.auto`.
    Auto,
    /// `turntable.calibrate`.
    Calibrate,
}

/// State of a rig job.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum RigJobState {
    /// Running.
    Running,
    /// Waiting for `scan.resume` (the table is stopped).
    Paused,
    /// Finished.
    Done,
    /// Ended by an error (`error`).
    Failed,
    /// Cancelled (`scan.stop`, `turntable.stop`, a disconnection).
    Cancelled,
}

/// What a rig job is doing.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum RigJobPhase {
    /// Turning the table and capturing (one step per tilt and per stop).
    Acquire,
    /// `scan.auto` finish: refining the poses of the stops.
    Refine,
    /// Fusing the stops into the fine volume.
    Fuse,
    /// Extracting the mesh.
    Extract,
    /// Cleaning the mesh.
    Clean,
    /// Closing its holes.
    FillHoles,
    /// Optimising it.
    Optimise,
}

/// Progress of a rig job.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct RigJobStatus {
    /// The job.
    pub kind: RigJobKind,
    /// Its state.
    pub state: RigJobState,
    /// Step under way, 1-based (`steps` when done).
    pub step: u32,
    /// Steps of the job: one per turntable move, plus one per phase of the
    /// finish of `scan.auto` (refine, fuse, extract, clean, fill holes,
    /// optimise).
    pub steps: u32,
    /// What `scan.auto` is doing (`null` for a calibration).
    #[serde(default)]
    pub phase: Option<RigJobPhase>,
    /// Tilt of the current pass (degrees), when tilting.
    pub tilt_deg: Option<f32>,
    /// Stable error code when failed.
    pub error: Option<String>,
    /// Technical detail of the error (untranslated).
    pub error_detail: Option<String>,
    /// `turntable.calibrate`: the residuals of the measured axes (also when
    /// the calibration was refused).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub calibration: Option<calib::CalibrationReport>,
    /// `scan.auto`: how well the views were registered and fused.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub quality: Option<ScanQuality>,
    /// `scan.auto` with `finish`: the fine reconstruction from the stops
    /// (`null` until done).
    #[serde(default)]
    pub reconstruction: Option<crate::reconstruct::ReconstructionReport>,
    /// `scan.auto` with `finish`: the finish of the mesh (`null` until
    /// done).
    #[serde(default)]
    pub finish: Option<dc_geom::finish::FinishReport>,
    /// Times the link to the table dropped during the job and came back:
    /// the job resumed at the stop where it was. A warning when not zero.
    #[serde(default)]
    pub reconnects: u32,
    /// The link to the table dropped and the job is reconnecting it now
    /// (at most 30 s; the job fails when it does not come back).
    #[serde(default)]
    pub reconnecting: bool,
    /// `scan.auto` stop-and-capture, once ended: the stops kept for
    /// `scan.reconstruct` and `scan.export` `stops`, also when the job
    /// failed or was cancelled (`null` without any).
    #[serde(default)]
    pub kept_stops: Option<u32>,
}

/// Default tilts of `scan.auto` (degrees): five passes, so the occlusal
/// surfaces and the steep walls are seen from above and from both sides,
/// within ±20 degrees, safe for an object that only rests on the platter.
/// On the rig (2026-11-19, a test arch resting on the platter) the passes
/// at ±15 stayed put (per-view corrections 0.2-0.5 degrees, at most 1.7)
/// while at ±30 the object slid and turned on the platter by 6 to 21
/// degrees within the pass; a scan at ±20 kept it in place.
pub const AUTO_TILTS_DEG: [f32; 5] = [-20.0, -10.0, 0.0, 10.0, 20.0];
/// Default degrees between the stops of `scan.auto` (stop-and-capture):
/// 5 × 49 stops, about 2 s each on the rig, keep the whole automatic scan
/// near 10 minutes.
pub const AUTO_STEP_DEG: f32 = 7.5;
/// Default frames averaged (per-pixel median) at each stop of `scan.auto`.
pub const AUTO_FRAMES_PER_STOP: u32 = 3;
/// Default mesh resolution of the finish of `scan.auto` (mm).
pub const AUTO_RESOLUTION_MM: f32 = crate::reconstruct::DEFAULT_RESOLUTION_MM;
/// Range of the resolution of the fine reconstruction (mm).
pub const RECONSTRUCT_RESOLUTION_MM: (f32, f32) = crate::reconstruct::RESOLUTION_RANGE_MM;
/// Default largest distance the finish may move the surface (mm).
pub const FINISH_MAX_ERROR_MM: f32 = dc_geom::finish::DEFAULT_MAX_ERROR_MM;

/// Quality of a `scan.auto` run.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct ScanQuality {
    /// Stop-and-capture: stops captured.
    pub stops: u32,
    /// Stops whose registration was refused, fused at the turntable pose.
    pub flagged_stops: u32,
    /// Continuous: frames received while turning.
    pub frames: u64,
    /// Frames (continuous) or stops fused.
    pub fused: u64,
    /// Mean RMS registration residual of the fused views (mm).
    pub mean_rms_mm: Option<f32>,
    /// Loop closure error of each full revolution (degrees about the axis).
    pub loop_error_deg: Vec<f32>,
    /// Warning code when the scan is usable but weak
    /// (`scan-quality-weak`).
    pub warning: Option<String>,
}

/// What `turntable.status` returns.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct TurntableStatus {
    /// A turntable is connected and its link is up. `false` with `info`
    /// set: the link dropped; a running job reconnects it by itself,
    /// otherwise `turntable.connect` again.
    pub connected: bool,
    /// Its identity.
    pub info: Option<TurntableInfo>,
    /// Its axes.
    pub axes: Vec<AxisInfo>,
    /// Estimated rotation (degrees, cumulative).
    pub rotate_deg: Option<f32>,
    /// Estimated tilt (degrees).
    pub tilt_deg: Option<f32>,
    /// A motion or a job is under way: moves are refused.
    pub busy: bool,
    /// A calibration matching the scanner and the table is in use (region
    /// of interest and motion prior).
    pub calibrated: bool,
    /// The latest `scan.auto` stopped with `calibration-outdated`: its
    /// first views were not where the stored calibration puts them, so the
    /// scanner or the turntable moved since it was measured. Calibrate
    /// again; a new calibration (measured, or a changed calibration file)
    /// clears it.
    pub calibration_outdated: bool,
    /// The latest `turntable.calibrate` job.
    pub calibration_job: Option<RigJobStatus>,
}

/// The rig calibration as glam geometry, used per frame.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RigGeometry {
    axis_point: Vec3,
    axis_dir: Vec3,
    up: Vec3,
    tilt: Option<(Vec3, Vec3)>,
    gain: f32,
}

/// Rigid rotation by `deg` about the line through `point` along `dir`.
pub fn about(point: Vec3, dir: Vec3, deg: f32) -> Affine3A {
    Affine3A::from_translation(point)
        * Affine3A::from_quat(Quat::from_axis_angle(dir, deg.to_radians()))
        * Affine3A::from_translation(-point)
}

/// Rigid map from the camera frame to a frame whose axes are `x`, `y`, `z`
/// (unit, right-handed, in camera coordinates) with its origin at `origin`.
fn frame_from_camera(x: Vec3, y: Vec3, z: Vec3, origin: Vec3) -> Affine3A {
    let r = Mat3::from_cols(x, y, z).transpose();
    Affine3A::from_mat3_translation(r, -(r * origin))
}

/// The scan world frame without a rig calibration, as a map from the
/// camera frame: +Z the camera's up (−y), +X back toward the scanner (−z),
/// Y = Z × X (the camera's +x), origin at the camera. A scanner held level
/// sees a horizontal surface as the XY plane.
pub fn camera_world() -> Affine3A {
    frame_from_camera(Vec3::NEG_Z, Vec3::X, Vec3::NEG_Y, Vec3::ZERO)
}

/// The scan world frame without a rig calibration of a scanner that knows
/// its up direction `up` (camera frame, from its IMU), as a map from the
/// camera frame: +Z the true up, +X back toward the scanner along the
/// horizontal (the camera's −z, or −y when it looks straight down),
/// Y = Z × X, origin at the camera. Equals [`camera_world`] for a scanner
/// held level (`up` the camera's −y).
pub fn gravity_world(up: Vec3) -> Affine3A {
    let z = up.try_normalize().unwrap_or(Vec3::NEG_Y);
    let x = Vec3::NEG_Z
        .reject_from_normalized(z)
        .try_normalize()
        .or_else(|| Vec3::NEG_Y.reject_from_normalized(z).try_normalize())
        .unwrap_or_else(|| z.any_orthonormal_vector());
    frame_from_camera(x, z.cross(x), z, Vec3::ZERO)
}

impl RigGeometry {
    /// The geometry of `c`.
    pub fn new(c: &RigCalibration) -> Self {
        let tilt = match (c.tilt_axis_point, c.tilt_axis_dir) {
            (Some(p), Some(d)) => Some((Vec3::from(p), Vec3::from(d).normalize())),
            _ => None,
        };
        Self {
            axis_point: c.axis_point.into(),
            axis_dir: Vec3::from(c.axis_dir).normalize(),
            up: Vec3::from(c.platter_up).normalize(),
            tilt,
            gain: if c.angle_gain.is_finite() && c.angle_gain > 0.0 {
                c.angle_gain
            } else {
                1.0
            },
        }
    }

    /// The tilt axis is known.
    pub fn has_tilt(&self) -> bool {
        self.tilt.is_some()
    }

    /// The tilt as a rigid motion of the camera frame (identity without a
    /// tilt axis).
    fn tilt(&self, tilt_deg: f32) -> Affine3A {
        self.tilt
            .map_or(Affine3A::IDENTITY, |(p, d)| about(p, d, tilt_deg))
    }

    /// Pose of the platter in the camera frame at the given angles, relative
    /// to the angles zero: `T(tilt) * A(gain * rotate)`. The motion of the
    /// object between two frames is `platter(b) * platter(a)^-1`.
    pub fn platter(&self, rotate_deg: f32, tilt_deg: f32) -> Affine3A {
        self.tilt(tilt_deg) * about(self.axis_point, self.axis_dir, self.gain * rotate_deg)
    }

    /// The rotation axis (point on the platter, direction) in the camera
    /// frame with the platter tilted by `tilt_deg`.
    pub fn axis_at(&self, tilt_deg: f32) -> (Vec3, Vec3) {
        let t = self.tilt(tilt_deg);
        (
            t.transform_point3(self.axis_point),
            t.transform_vector3(self.axis_dir),
        )
    }

    /// Height (mm) of the camera-frame point `p` above the platter tilted
    /// by `tilt_deg`, and its squared distance from the axis.
    fn height(&self, p: Vec3, tilt_deg: f32) -> (f32, f32) {
        let q = if self.tilt.is_some() && tilt_deg != 0.0 {
            self.tilt(tilt_deg).inverse().transform_point3(p)
        } else {
            p
        };
        let v = q - self.axis_point;
        let h = v.dot(self.up);
        (h, (v - h * self.up).length_squared())
    }

    /// `true` when the camera-frame point `p` is within
    /// [`PLATTER_BAND_MM`] of the platter plane or under it.
    pub fn on_platter(&self, p: Vec3, tilt_deg: f32) -> bool {
        self.height(p, tilt_deg).0 < PLATTER_BAND_MM
    }

    /// `true` when the camera-frame point `p` is inside the cylinder of
    /// radius `radius_mm` around the axis, from [`ROI_BELOW_PLATTER_MM`]
    /// under the platter up to `height_mm` above it, the platter tilted by
    /// `tilt_deg`.
    pub fn roi(&self, p: Vec3, tilt_deg: f32, radius_mm: f32, height_mm: f32) -> bool {
        let (h, r2) = self.height(p, tilt_deg);
        (-ROI_BELOW_PLATTER_MM..=height_mm).contains(&h) && r2 <= radius_mm * radius_mm
    }

    /// The scan world frame of a scan whose first view was taken with the
    /// platter tilted by `tilt_deg`, as a rigid map from that camera frame:
    /// +Z the rotation axis pointing up from the platter, origin on the
    /// axis at the platter, +X from the axis toward the scanner along the
    /// platter, Y = Z × X. The object stands along +Z on the XY plane.
    pub fn world_from_camera(&self, tilt_deg: f32) -> Affine3A {
        let t = self.tilt(tilt_deg);
        let origin = t.transform_point3(self.axis_point);
        let z = t.transform_vector3(self.up).normalize();
        let x = (-origin)
            .reject_from_normalized(z)
            .try_normalize()
            .unwrap_or_else(|| z.any_orthonormal_vector());
        frame_from_camera(x, z.cross(x), z, origin)
    }
}

/// Fraction of the frames in which a depth must repeat to be background.
pub const BACKGROUND_MIN_FRACTION: f32 = 0.6;

/// Static background of a depth image: per pixel, the farthest depth seen
/// the same (within `tol_mm`) in at least [`BACKGROUND_MIN_FRACTION`] of
/// the frames (and two), taken at different turntable angles. Pixels at
/// that depth in a later frame show the enclosure (or the platter, the
/// same at every angle), not the turning object; a surface of the object
/// that happens to repeat a depth in a few frames is not background.
#[derive(Clone, Debug, PartialEq)]
pub struct Background {
    depth: Vec<f32>,
    tol_mm: f32,
}

impl Background {
    /// Background of `frames` (same image size, `NaN` / `0` invalid).
    pub fn from_frames(frames: &[std::sync::Arc<Vec<f32>>], tol_mm: f32) -> Self {
        Self::from_frames_with(frames, tol_mm, BACKGROUND_MIN_FRACTION)
    }

    /// [`Background::from_frames`] with the depth repeated in at least
    /// `min_fraction` of the frames (and two).
    pub fn from_frames_with(
        frames: &[std::sync::Arc<Vec<f32>>],
        tol_mm: f32,
        min_fraction: f32,
    ) -> Self {
        use rayon::prelude::*;
        let n = frames.iter().map(|f| f.len()).min().unwrap_or(0);
        let need = ((min_fraction * frames.len() as f32).ceil() as usize).max(2);
        let depth = (0..n)
            .into_par_iter()
            .with_min_len(4096)
            .map(|i| {
                let mut v: Vec<f32> = frames
                    .iter()
                    .map(|f| f[i])
                    .filter(|d| d.is_finite() && *d > 0.0)
                    .collect();
                if v.len() < need {
                    return f32::NAN;
                }
                v.sort_by(|a, b| b.total_cmp(a));
                // the farthest v_k with `need` depths in [v_k - tol, v_k + tol]
                let (mut lo, mut hi) = (0, 0);
                for k in 0..v.len() {
                    while v[lo] - v[k] > tol_mm {
                        lo += 1;
                    }
                    hi = hi.max(k);
                    while hi + 1 < v.len() && v[k] - v[hi + 1] <= tol_mm {
                        hi += 1;
                    }
                    if hi + 1 - lo >= need {
                        return v[k];
                    }
                }
                f32::NAN
            })
            .collect();
        Self { depth, tol_mm }
    }

    /// `true` when depth `d` at pixel `i` is background.
    pub fn is_static(&self, i: usize, d: f32) -> bool {
        self.depth
            .get(i)
            .is_some_and(|b| b.is_finite() && (d - b).abs() <= self.tol_mm)
    }

    /// Pixels with a known background.
    pub fn known(&self) -> usize {
        self.depth.iter().filter(|d| d.is_finite()).count()
    }
}

/// What the acquisition thread needs from the rig, set by the controller.
#[derive(Clone, Default)]
pub(crate) struct RigInput {
    pub calibration: Option<RigCalibration>,
    pub geometry: Option<RigGeometry>,
    pub probe: Option<AngleProbe>,
    /// The rotation from the scanner's IMU frame to its camera frame
    /// ([`RigCalibration::camera_from_imu`]), when known.
    pub camera_from_imu: Option<Quat>,
}

/// Default directory of the rig files: `<data dir>/dentalcraft/rig`
/// (`$XDG_DATA_HOME` or `~/.local/share` on Linux, `%APPDATA%` on Windows,
/// `~/Library/Application Support` on macOS); `None` without a home.
pub fn default_rig_dir() -> Option<PathBuf> {
    let env = |k: &str| {
        std::env::var_os(k)
            .filter(|v| !v.is_empty())
            .map(PathBuf::from)
    };
    let data = if cfg!(windows) {
        env("APPDATA")
    } else if cfg!(target_os = "macos") {
        env("HOME").map(|h| h.join("Library/Application Support"))
    } else {
        env("XDG_DATA_HOME").or_else(|| env("HOME").map(|h| h.join(".local/share")))
    };
    data.map(|d| d.join("dentalcraft").join("rig"))
}

/// Reads the calibration stored in `dir`; `None` when there is none.
///
/// # Errors
/// [`ScanError::Calibration`] for an unreadable or invalid file.
pub fn load_calibration(dir: &Path) -> Result<Option<RigCalibration>> {
    let path = dir.join(CALIBRATION_FILE);
    match std::fs::read(&path) {
        Ok(bytes) => serde_json::from_slice(&bytes)
            .map(Some)
            .map_err(|e| ScanError::Calibration(format!("{}: {e}", path.display()))),
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => Ok(None),
        Err(e) => Err(ScanError::Calibration(format!("{}: {e}", path.display()))),
    }
}

/// Writes `c` into `dir` (created when missing), atomically.
///
/// # Errors
/// [`ScanError::Calibration`] when the file cannot be written.
pub fn save_calibration(dir: &Path, c: &RigCalibration) -> Result<PathBuf> {
    let err = |e: std::io::Error| ScanError::Calibration(format!("{}: {e}", dir.display()));
    std::fs::create_dir_all(dir).map_err(err)?;
    let path = dir.join(CALIBRATION_FILE);
    let tmp = dir.join(format!("{CALIBRATION_FILE}.tmp"));
    let text = serde_json::to_vec_pretty(c).map_err(|e| ScanError::Calibration(e.to_string()))?;
    std::fs::write(&tmp, text).map_err(err)?;
    std::fs::rename(&tmp, &path).map_err(err)?;
    Ok(path)
}

/// `t` as RFC 3339 UTC with second precision (`2026-11-16T14:30:05Z`).
pub fn rfc3339(t: SystemTime) -> String {
    let secs = t
        .duration_since(UNIX_EPOCH)
        .unwrap_or(Duration::ZERO)
        .as_secs();
    let (days, rem) = (secs / 86_400, secs % 86_400);
    // civil from days (Howard Hinnant)
    let z = days as i64 + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z - era * 146_097;
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let day = doy - (153 * mp + 2) / 5 + 1;
    let month = if mp < 10 { mp + 3 } else { mp - 9 };
    let year = yoe + era * 400 + i64::from(month <= 2);
    format!(
        "{year:04}-{month:02}-{day:02}T{:02}:{:02}:{:02}Z",
        rem / 3600,
        rem / 60 % 60,
        rem % 60
    )
}

/// JSON of an optional calibration.
pub(crate) fn calibration_json(c: Option<&RigCalibration>) -> Value {
    c.map_or(Value::Null, |c| json!(c))
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Arc;

    fn calibration() -> RigCalibration {
        RigCalibration {
            axis_point: [0.0, 0.0, 150.0],
            axis_dir: [0.0, 0.0, 1.0],
            platter_up: [0.0, 0.0, -1.0],
            tilt_axis_point: Some([0.0, 0.0, 160.0]),
            tilt_axis_dir: Some([1.0, 0.0, 0.0]),
            scanner_serial: Some("S1".into()),
            turntable_id: "T1".into(),
            created: rfc3339(SystemTime::UNIX_EPOCH),
            fit_rms_deg: 0.1,
            fit_rms_mm: 0.05,
            angle_gain: 1.0,
            fit_pairs: 24,
            imu_up: None,
        }
    }

    #[test]
    fn the_imu_is_oriented_by_the_up_of_the_calibration() {
        // IMU axes along the camera's: x to -x, y to -z, z to -y (the up of
        // a scanner held level), scanner looking 19 degrees down
        let truth = Quat::from_mat3(&Mat3::from_cols(Vec3::NEG_X, Vec3::NEG_Z, Vec3::NEG_Y));
        let looking_down = |deg: f32| {
            let a = deg.to_radians();
            Vec3::new(0.0, -a.cos(), -a.sin())
        };
        let up = looking_down(19.0);
        let c = RigCalibration {
            platter_up: up.into(),
            imu_up: Some((truth.inverse() * up).into()),
            ..calibration()
        };
        let r = c.camera_from_imu().unwrap();
        assert!(r.angle_between(truth).to_degrees() < 0.01, "{r:?}");
        // the scanner moved to 40 degrees down: its IMU tells the new up
        let now = looking_down(40.0);
        assert!((r * (truth.inverse() * now)).angle_between(now) < 1e-4);
        // an IMU slightly off the axes still maps the calibration up exactly
        let off = Quat::from_rotation_x(3_f32.to_radians()) * truth;
        let c = RigCalibration {
            imu_up: Some((off.inverse() * up).into()),
            ..c
        };
        let r = c.camera_from_imu().unwrap();
        assert!((r * Vec3::from(c.imu_up.unwrap())).angle_between(up) < 1e-4);
        assert!(r.angle_between(off).to_degrees() < 0.01, "{r:?}");
        // no IMU: unknown
        assert_eq!(calibration().camera_from_imu(), None);
    }

    #[test]
    fn the_gravity_world_stands_on_the_true_up() {
        let level = gravity_world(Vec3::NEG_Y);
        assert!(level.abs_diff_eq(camera_world(), 1e-6), "{level:?}");
        // looking 19 degrees down: up is +Z, the view points down along -X
        let a = 19_f32.to_radians();
        let up = Vec3::new(0.0, -a.cos(), -a.sin());
        let w = gravity_world(up);
        assert!(w.transform_vector3(up).abs_diff_eq(Vec3::Z, 1e-6));
        let view = w.transform_vector3(Vec3::Z);
        assert!((view.z + a.sin()).abs() < 1e-6 && view.y.abs() < 1e-6 && view.x < 0.0);
        assert_eq!(w.transform_point3(Vec3::ZERO), Vec3::ZERO);
        // looking straight down: +X toward the camera's top edge
        let down = gravity_world(Vec3::NEG_Z);
        assert!(
            down.transform_vector3(Vec3::NEG_Y)
                .abs_diff_eq(Vec3::X, 1e-6)
        );
    }

    #[test]
    fn timestamps_are_rfc3339() {
        assert_eq!(rfc3339(UNIX_EPOCH), "1970-01-01T00:00:00Z");
        let t = UNIX_EPOCH + Duration::from_secs(1_794_839_405);
        assert_eq!(rfc3339(t), "2026-11-16T14:30:05Z");
        let leap = UNIX_EPOCH + Duration::from_secs(951_782_400);
        assert_eq!(rfc3339(leap), "2000-02-29T00:00:00Z");
    }

    #[test]
    fn calibration_round_trips_and_matches() {
        let dir = tempfile::tempdir().unwrap();
        assert_eq!(load_calibration(dir.path()).unwrap(), None);
        let c = calibration();
        save_calibration(&dir.path().join("rig"), &c).unwrap();
        assert_eq!(
            load_calibration(&dir.path().join("rig")).unwrap(),
            Some(c.clone())
        );
        assert!(c.matches(Some("S1"), "T1"));
        assert!(c.matches(None, "T1"));
        assert!(!c.matches(Some("S2"), "T1"));
        assert!(!c.matches(Some("S1"), "T2"));
        std::fs::write(dir.path().join(CALIBRATION_FILE), "{").unwrap();
        assert_eq!(
            load_calibration(dir.path()).unwrap_err().code(),
            "calibration"
        );
        let v = calibration_json(Some(&c));
        assert_eq!(v["created"], "1970-01-01T00:00:00Z");
        assert_eq!(v["axis_dir"], json!([0.0, 0.0, 1.0]));
    }

    #[test]
    fn roi_is_a_cylinder_over_the_platter_that_follows_the_tilt() {
        let g = RigGeometry::new(&calibration());
        // up is -z (towards the camera at the origin)
        assert!(g.roi(Vec3::new(10.0, 0.0, 140.0), 0.0, 45.0, 60.0));
        assert!(g.roi(Vec3::new(0.0, 0.0, 150.5), 0.0, 45.0, 60.0));
        assert!(!g.roi(Vec3::new(0.0, 0.0, 152.0), 0.0, 45.0, 60.0), "under");
        assert!(!g.roi(Vec3::new(0.0, 0.0, 80.0), 0.0, 45.0, 60.0), "above");
        assert!(
            !g.roi(Vec3::new(50.0, 0.0, 140.0), 0.0, 45.0, 60.0),
            "outside"
        );
        // a point on the tilted platter centre line stays inside
        let p = g
            .platter(0.0, 20.0)
            .transform_point3(Vec3::new(0.0, 0.0, 120.0));
        assert!(g.roi(p, 20.0, 1.0, 60.0));
        assert!(!g.roi(p, 0.0, 1.0, 60.0));
        // the platter motion: 90 degrees about +z through the axis point
        let m = g.platter(90.0, 0.0);
        let q = m.transform_point3(Vec3::new(10.0, 0.0, 150.0));
        assert!(q.distance(Vec3::new(0.0, 10.0, 150.0)) < 1e-4, "{q}");
    }

    #[test]
    fn background_is_the_farthest_repeated_depth() {
        let frames: Vec<Arc<Vec<f32>>> = [
            vec![200.0, 150.0, 0.0, 120.0],
            vec![200.2, 200.0, 0.0, 130.0],
            vec![160.0, 200.1, 180.0, f32::NAN],
        ]
        .into_iter()
        .map(Arc::new)
        .collect();
        let b = Background::from_frames(&frames, 0.5);
        assert!(b.is_static(0, 200.1));
        assert!(!b.is_static(0, 160.0));
        assert!(b.is_static(1, 199.8));
        assert!(!b.is_static(2, 180.0), "seen once");
        assert!(!b.is_static(3, 120.0), "never repeated");
        assert_eq!(b.known(), 2);
    }
}
