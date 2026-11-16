//! Turntable scanning rig (alpha): a fixed scanner in an enclosure looking
//! at an object on a motorised turntable ([`dc_turntable`]).
//!
//! * **Calibration** ([`RigCalibration`]): the turntable axes in the
//!   scanner camera frame. It is measured without tools by
//!   `turntable.calibrate` (see [`jobs`]): a static object on the platter is
//!   turned in steps while the scanner tracks it, and the axis is fitted to
//!   the tracked poses ([`fit::fit_axis`]). It is stored as JSON in
//!   `<data dir>/dentalcraft/rig/calibration.json`.
//! * **Region of interest**: with a calibration, every depth frame is
//!   cropped to a cylinder around the turntable axis ([`RigGeometry::roi`]),
//!   from just under the platter up to [`crate::ScanSettings::roi_height_mm`],
//!   so the static enclosure never pulls the tracking.
//! * **Motion prior**: with a calibration and a live angle estimate of the
//!   table ([`dc_turntable::AngleProbe`]), the motion of the object since
//!   the previous frame, predicted from the angles, is the initial guess of
//!   the tracking ([`RigGeometry::platter`]).
//! * **Jobs**: `scan.auto` (tilt, then rotate while fusing, for each tilt)
//!   and `turntable.calibrate` run on their own thread ([`jobs`]).
//!
//! The camera frame is the frame of the depth images: x right, y down,
//! z forward, millimetres.

pub(crate) mod control;
pub mod fit;
pub(crate) mod jobs;

use std::path::{Path, PathBuf};
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use dc_turntable::{AngleProbe, AxisInfo, TurntableInfo};
use glam::{Affine3A, Quat, Vec3};
use serde::{Deserialize, Serialize};
use serde_json::{Value, json};

use crate::{Result, ScanError};

pub use control::turntable_devices;

/// Default radius of the region of interest around the axis (mm).
pub const DEFAULT_ROI_RADIUS_MM: f32 = 45.0;
/// Default height of the region of interest above the platter (mm).
pub const DEFAULT_ROI_HEIGHT_MM: f32 = 60.0;
/// The region of interest starts this far under the platter plane (mm),
/// so the base of the object stays in.
pub const ROI_BELOW_PLATTER_MM: f32 = 1.0;
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

/// Progress of a rig job.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct RigJobStatus {
    /// The job.
    pub kind: RigJobKind,
    /// Its state.
    pub state: RigJobState,
    /// Step under way, 1-based (`steps` when done).
    pub step: u32,
    /// Steps of the job (one per turntable move).
    pub steps: u32,
    /// Tilt of the current pass (degrees), when tilting.
    pub tilt_deg: Option<f32>,
    /// Stable error code when failed.
    pub error: Option<String>,
    /// Technical detail of the error (untranslated).
    pub error_detail: Option<String>,
}

/// What `turntable.status` returns.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct TurntableStatus {
    /// A turntable is connected.
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
}

/// Rigid rotation by `deg` about the line through `point` along `dir`.
fn about(point: Vec3, dir: Vec3, deg: f32) -> Affine3A {
    Affine3A::from_translation(point)
        * Affine3A::from_quat(Quat::from_axis_angle(dir, deg.to_radians()))
        * Affine3A::from_translation(-point)
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
    /// to the angles zero: `T(tilt) * A(rotate)`. The motion of the object
    /// between two frames is `platter(b) * platter(a)^-1`.
    pub fn platter(&self, rotate_deg: f32, tilt_deg: f32) -> Affine3A {
        self.tilt(tilt_deg) * about(self.axis_point, self.axis_dir, rotate_deg)
    }

    /// `true` when the camera-frame point `p` is inside the cylinder of
    /// radius `radius_mm` around the axis, from [`ROI_BELOW_PLATTER_MM`]
    /// under the platter up to `height_mm` above it, the platter tilted by
    /// `tilt_deg`.
    pub fn roi(&self, p: Vec3, tilt_deg: f32, radius_mm: f32, height_mm: f32) -> bool {
        let q = if self.tilt.is_some() && tilt_deg != 0.0 {
            self.tilt(tilt_deg).inverse().transform_point3(p)
        } else {
            p
        };
        let v = q - self.axis_point;
        let h = v.dot(self.up);
        (-ROI_BELOW_PLATTER_MM..=height_mm).contains(&h)
            && (v - h * self.up).length_squared() <= radius_mm * radius_mm
    }
}

/// Static background of a depth image: per pixel, the farthest depth seen
/// the same (within `tol_mm`) in at least two frames taken at different
/// turntable angles. Pixels at that depth in a later frame show the
/// enclosure, not the turning object.
#[derive(Clone, Debug, PartialEq)]
pub struct Background {
    depth: Vec<f32>,
    tol_mm: f32,
}

impl Background {
    /// Background of `frames` (same image size, `NaN` / `0` invalid).
    pub fn from_frames(frames: &[std::sync::Arc<Vec<f32>>], tol_mm: f32) -> Self {
        let n = frames.iter().map(|f| f.len()).min().unwrap_or(0);
        let depth = (0..n)
            .map(|i| {
                let mut v: Vec<f32> = frames
                    .iter()
                    .map(|f| f[i])
                    .filter(|d| d.is_finite() && *d > 0.0)
                    .collect();
                v.sort_by(|a, b| b.total_cmp(a));
                (0..v.len())
                    .find(|&k| {
                        v.iter()
                            .enumerate()
                            .any(|(j, d)| j != k && (d - v[k]).abs() <= tol_mm)
                    })
                    .map_or(f32::NAN, |k| v[k])
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

/// What the acquisition thread needs from the rig, set by the controller
/// and the jobs.
#[derive(Clone, Default)]
pub(crate) struct RigInput {
    pub calibration: Option<RigCalibration>,
    pub geometry: Option<RigGeometry>,
    pub probe: Option<AngleProbe>,
    pub background: Option<std::sync::Arc<Background>>,
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
        }
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
