//! Turntable axis calibration from settled stops (`turntable.calibrate`).
//!
//! The job ([`super::jobs`]) turns a static object on the platter in steps
//! and captures, at every stop, the per-pixel median of a few frames
//! ([`median_depth`]). [`measure`] then works on those images only:
//!
//! 1. **Object**: the static background (enclosure, platter: the same at
//!    every angle) is the depth repeated in most stops ([`Background`]);
//!    what remains, gated around its median, is the turning object.
//! 2. **Coarse axis**: the per-stop centroids of the object turn on a
//!    circle about the axis. Candidate directions (the normal of that
//!    circle, and a grid of "up" directions of a scanner looking down at
//!    the table) with the circle centre as axis point seed a registration
//!    of the first two stops; the best registration gives the coarse
//!    screw axis. The platter itself is nearly invisible to the MINI 2, so
//!    no plane is needed.
//! 3. **Pairs**: every stop is registered onto the next one with
//!    point-to-plane ICP, seeded by the rotation about the current axis
//!    by the commanded angle ([`crate::tracking::track_frame`] on a volume
//!    of the next stop). The screw axis of those motions is fitted
//!    ([`fit_motions`]); a second round is seeded with the fitted axis.
//! 4. **Platter**: the lowest dense layer of the object along the axis
//!    (its base) gives the platter plane, where the axis point is put.
//! 5. **Tilt** (dual-axis tables): the stops at `+t` and `-t` are
//!    registered onto the last stop at tilt 0 and give the tilt axis; a
//!    tilted stop that does not register refuses the calibration with its
//!    own code, `calibration-tilt`.
//! 6. **Checks**: enough registered pairs, angle residual at most
//!    [`MAX_RMS_DEG`], axis residual at most [`MAX_RMS_MM`], axis
//!    [`AXIS_DISTANCE_MM`] from the scanner, platter seen from above,
//!    angle gain within [`MAX_GAIN_ERROR`]. A failed check refuses the
//!    calibration ([`ScanError::CalibrationRefused`]) with the
//!    [`CalibrationReport`] of what was measured.

use std::sync::Arc;

use dc_geom::tsdf::{CameraIntrinsics, TsdfParams, TsdfVolume};
use glam::{Affine3A, DAffine3, DMat3, DVec3, Mat3, Quat, Vec3};
use rayon::prelude::*;
use serde::{Deserialize, Serialize};

use super::fit::{AxisFit, fit_axis, fit_motions};
use super::{Background, about};
use crate::tracking::{TrackParams, TrackResult, sample_frame, track_frame};
use crate::{Result, ScanError};

/// Two depths closer than this are the same background (mm).
pub const BACKGROUND_TOL_MM: f32 = 1.5;
/// Largest accepted RMS angle residual of the axis fit, per stop pair
/// (degrees). Tuned on the recorded MINI 2 revolution (24 x 15 degrees,
/// field-of-view intrinsics): 1.7 there, from a geometry that is not
/// quite metric without the scanner's own intrinsics; the axis itself is
/// averaged over the pairs. A still or locked run fails other checks.
pub const MAX_RMS_DEG: f32 = 2.5;
/// Largest accepted RMS position residual of the axis fit (mm); 1.3 on
/// the recorded revolution.
pub const MAX_RMS_MM: f32 = 2.0;
/// Accepted distance of the axis point from the scanner (mm).
pub const AXIS_DISTANCE_MM: (f32, f32) = (80.0, 300.0);
/// Largest accepted `|angle_gain - 1|`.
pub const MAX_GAIN_ERROR: f32 = 0.05;
/// Fewest registered pairs, as a fraction of the stop pairs.
pub const MIN_PAIR_FRACTION: f32 = 0.6;
/// Object depths further than this from their median are dropped (mm).
const OBJECT_DEPTH_GATE_MM: f32 = 60.0;
/// A pixel whose depth differs by less than this between two stops is
/// static clutter for their registration (mm).
const PAIR_STATIC_MM: f32 = 0.5;
/// Object points further than this from their median point are dropped (mm).
const OBJECT_RADIUS_GATE_MM: f32 = 80.0;
/// Fewest object pixels per stop, on average.
const MIN_OBJECT_PIXELS: usize = 1500;
/// Depths of the tilt axis under the platter (mm) that seed the
/// registrations of the tilt stops. The Revopoint dual-axis table tilts
/// about an axis 47 mm under its platter (measured): a seed 15 mm off that
/// depth starts a ±15° tilt stop about 4 mm off, beyond the first
/// association gate of the registration, and whether it still registers
/// then depends on the side of the object in view.
const TILT_PIVOT_SEEDS_MM: [f32; 8] = [0.0, 10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 70.0];

/// One settled stop of the calibration.
#[derive(Clone, Debug)]
pub struct CalibrationStop {
    /// Table rotation (degrees, cumulative).
    pub rotate_deg: f32,
    /// Table tilt (degrees).
    pub tilt_deg: f32,
    /// Range-filtered median depth image (`NaN` / `0` invalid).
    pub depth: Arc<Vec<f32>>,
}

/// What [`measure`] reports, also when it refuses: the numbers behind the
/// checks, for the UI.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct CalibrationReport {
    /// RMS angle residual of the rotation axis fit (degrees).
    pub rms_deg: Option<f32>,
    /// RMS position residual of the rotation axis fit (mm).
    pub rms_mm: Option<f32>,
    /// Registered stop pairs.
    pub pairs: u32,
    /// Stop pairs tried.
    pub pairs_expected: u32,
    /// Measured over commanded rotation angle (least squares over the
    /// pairs).
    pub angle_gain: Option<f32>,
    /// The last stop, a full turn after the first, shows the same view:
    /// the table turns exactly, so the gain applied is 1.
    pub loop_verified: Option<bool>,
    /// Distance of the axis point (on the platter) from the scanner (mm).
    pub axis_distance_mm: Option<f32>,
    /// Angle between the platter normal and the direction to the scanner
    /// (degrees): below 90 when the scanner looks down on the platter.
    pub view_angle_deg: Option<f32>,
    /// RMS angle residual of the tilt axis fit (degrees).
    pub tilt_rms_deg: Option<f32>,
    /// RMS position residual of the tilt axis fit (mm).
    pub tilt_rms_mm: Option<f32>,
    /// Object pixels per stop, on average.
    pub object_pixels: u32,
}

/// A measured calibration.
#[derive(Clone, Debug)]
pub struct CalibrationOutcome {
    /// The rotation axis.
    pub axis: AxisFit,
    /// The tilt axis, for a dual-axis table.
    pub tilt: Option<AxisFit>,
    /// Point of the rotation axis on the platter plane.
    pub platter_point: Vec3,
    /// Platter normal towards the scanner (`axis.dir` or its opposite).
    pub up: Vec3,
    /// Angle gain to apply: 1 when [`CalibrationReport::loop_verified`],
    /// else the measured one.
    pub angle_gain: f32,
    /// The residuals.
    pub report: CalibrationReport,
}

/// A refused or failed calibration, with what was measured so far.
pub type Refusal = (ScanError, CalibrationReport);

fn refuse(code: &'static str, detail: String, report: &CalibrationReport) -> Refusal {
    (
        ScanError::CalibrationRefused { code, detail },
        report.clone(),
    )
}

/// Per-pixel median of the valid depths of `frames` (a pixel needs a valid
/// depth in more than half of them, both of two). `NaN` where too few.
pub fn median_depth(frames: &[Arc<Vec<f32>>]) -> Vec<f32> {
    let n = frames.iter().map(|f| f.len()).min().unwrap_or(0);
    let need = if frames.len() <= 2 {
        frames.len()
    } else {
        frames.len() / 2 + 1
    };
    (0..n)
        .into_par_iter()
        .with_min_len(4096)
        .map(|i| {
            let mut v: Vec<f32> = frames
                .iter()
                .map(|f| f[i])
                .filter(|d| d.is_finite() && *d > 0.0)
                .collect();
            if v.is_empty() || v.len() < need {
                return f32::NAN;
            }
            v.sort_by(f32::total_cmp);
            let m = v.len() / 2;
            if v.len().is_multiple_of(2) {
                0.5 * (v[m - 1] + v[m])
            } else {
                v[m]
            }
        })
        .collect()
}

/// Tracking parameters of the stop registrations: the seed is within a
/// few millimetres, a correction beyond 5 mm / 5 degrees is a slide.
fn pair_params() -> TrackParams {
    TrackParams {
        max_rms_mm: 0.3,
        min_inliers: 150,
        min_inlier_fraction: 0.2,
        ..TrackParams::default()
    }
    .with_prior()
}

/// A volume holding the single image `depth` seen from the identity.
fn single_view(depth: &[f32], intr: &CameraIntrinsics) -> TsdfVolume {
    let mut v = TsdfVolume::new(TsdfParams {
        voxel_mm: 0.3,
        trunc_mm: 1.2,
        max_weight: 64.0,
    });
    v.integrate(depth, intr, &Affine3A::IDENTITY);
    v
}

/// Screw axis (point nearest the origin, unit direction about which the
/// rotation is positive and below a half turn) of a rigid motion with a
/// rotation of at least a degree.
fn screw_axis(m: &Affine3A) -> Option<(Vec3, Vec3)> {
    let (_, q, _) = m.to_scale_rotation_translation();
    let (mut axis, angle) = q.to_axis_angle();
    if angle > std::f32::consts::PI {
        axis = -axis;
    }
    if q.angle_between(Quat::IDENTITY) < 1f32.to_radians() {
        return None;
    }
    let r = Mat3::from_quat(q).as_dmat3();
    let a = axis.as_dvec3();
    let mut ata = DMat3::from_cols(a * a.x, a * a.y, a * a.z);
    let i_r = DMat3::IDENTITY - r;
    ata += i_r.transpose() * i_r;
    let atb = i_r.transpose() * Vec3::from(m.translation).as_dvec3();
    (ata.determinant().abs() > 1e-12).then(|| ((ata.inverse() * atb).as_vec3(), axis))
}

/// Centre of the circle through the points `c` projected on the plane
/// normal to `dir` (algebraic fit), or their mean when they barely turn.
fn circle_centre(c: &[Vec3], dir: Vec3) -> Vec3 {
    let mean = c.iter().copied().sum::<Vec3>() / c.len().max(1) as f32;
    let u = dir.any_orthonormal_vector();
    let v = dir.cross(u);
    let pts: Vec<(f64, f64)> = c
        .iter()
        .map(|p| (f64::from((*p - mean).dot(u)), f64::from((*p - mean).dot(v))))
        .collect();
    let spread = pts.iter().map(|(x, y)| x * x + y * y).sum::<f64>() / pts.len().max(1) as f64;
    if pts.len() < 3 || spread < 1.0 {
        return mean;
    }
    // x^2 + y^2 + D x + E y + F = 0
    let mut ata = DMat3::ZERO;
    let mut atb = DVec3::ZERO;
    for (x, y) in &pts {
        let row = DVec3::new(*x, *y, 1.0);
        ata += DMat3::from_cols(row * row.x, row * row.y, row * row.z);
        atb += row * -(x * x + y * y);
    }
    if ata.determinant().abs() < 1e-9 {
        return mean;
    }
    let s = ata.inverse() * atb;
    let (cx, cy) = (-0.5 * s.x, -0.5 * s.y);
    let r = (cx * cx + cy * cy - s.z).max(0.0).sqrt();
    // a circle much wider than the spread of the centroids is noise
    if !r.is_finite() || r * r > 25.0 * spread + 100.0 {
        return mean;
    }
    mean + cx as f32 * u + cy as f32 * v
}

/// Camera-frame points of the valid pixels of `depth`, every `stride`.
fn points(depth: &[f32], intr: &CameraIntrinsics, stride: usize) -> Vec<Vec3> {
    let w = intr.width as usize;
    depth
        .iter()
        .enumerate()
        .step_by(stride.max(1))
        .filter(|(_, d)| d.is_finite() && **d > 0.0)
        .map(|(i, d)| intr.ray((i % w) as f32, (i / w) as f32) * *d)
        .collect()
}

/// Points of the image `src` that the object motion `m` puts on the
/// surface of the image `dst` (within a millimetre in depth): the turning
/// object, not the static scene it uncovers.
fn consistent(src: &[f32], dst: &[f32], intr: &CameraIntrinsics, m: &Affine3A) -> Vec<Vec3> {
    let (w, h) = (intr.width as f32, intr.height as f32);
    points(src, intr, 2)
        .into_iter()
        .filter(|p| {
            let q = m.transform_point3(*p);
            if q.z <= 0.0 {
                return false;
            }
            let u = (intr.fx * q.x / q.z + intr.cx).round();
            let v = (intr.fy * q.y / q.z + intr.cy).round();
            if u < 0.0 || v < 0.0 || u >= w || v >= h {
                return false;
            }
            let d = dst[v as usize * intr.width as usize + u as usize];
            d.is_finite() && (d - q.z).abs() <= 1.0
        })
        .collect()
}

fn median(mut v: Vec<f32>) -> Option<f32> {
    if v.is_empty() {
        return None;
    }
    let m = v.len() / 2;
    let (_, x, _) = v.select_nth_unstable_by(m, f32::total_cmp);
    Some(*x)
}

/// The object pixels of `stops` (others `NaN`): not on the static
/// background, within [`OBJECT_DEPTH_GATE_MM`] of the median object depth
/// and [`OBJECT_RADIUS_GATE_MM`] of the median object point.
fn object_depths(
    stops: &[CalibrationStop],
    background: &Background,
    intr: &CameraIntrinsics,
) -> Vec<Vec<f32>> {
    let moving: Vec<Vec<f32>> = stops
        .iter()
        .map(|s| {
            s.depth
                .iter()
                .enumerate()
                .map(|(i, &d)| {
                    if d.is_finite() && d > 0.0 && !background.is_static(i, d) {
                        d
                    } else {
                        f32::NAN
                    }
                })
                .collect()
        })
        .collect();
    let sample: Vec<f32> = moving
        .iter()
        .flat_map(|d| d.iter().step_by(7).copied().filter(|d| d.is_finite()))
        .collect();
    let Some(mid) = median(sample) else {
        return moving;
    };
    let pts: Vec<Vec3> = moving.iter().flat_map(|d| points(d, intr, 11)).collect();
    let centre = Vec3::new(
        median(pts.iter().map(|p| p.x).collect()).unwrap_or(0.0),
        median(pts.iter().map(|p| p.y).collect()).unwrap_or(0.0),
        median(pts.iter().map(|p| p.z).collect()).unwrap_or(mid),
    );
    let w = intr.width as usize;
    moving
        .into_iter()
        .map(|d| {
            d.iter()
                .enumerate()
                .map(|(i, &z)| {
                    let keep = z.is_finite()
                        && (z - mid).abs() <= OBJECT_DEPTH_GATE_MM
                        && (intr.ray((i % w) as f32, (i / w) as f32) * z).distance(centre)
                            <= OBJECT_RADIUS_GATE_MM;
                    if keep { z } else { f32::NAN }
                })
                .collect()
        })
        .collect()
}

/// Registers `source` onto the volume `target` from the object motion
/// `seed` (source camera points to target camera points).
fn register(
    target: &TsdfVolume,
    source: &[f32],
    intr: &CameraIntrinsics,
    seed: &Affine3A,
) -> TrackResult {
    let params = pair_params();
    let sample = sample_frame(source, intr, &params);
    track_frame(target, &sample, intr, seed, &params)
}

/// Better of two registrations: accepted first, then the lower residual.
fn better(a: Option<TrackResult>, b: TrackResult) -> TrackResult {
    match a {
        Some(a) if (a.ok, -a.rms_mm) >= (b.ok, -b.rms_mm) => a,
        _ => b,
    }
}

/// Candidate "up" directions of a scanner looking down at the table: the
/// camera's `-y` turned towards the camera by 0 to 80 degrees and rolled
/// by up to 30 degrees.
fn up_grid() -> Vec<Vec3> {
    let mut v = Vec::new();
    for a in (0..=80).step_by(10) {
        for b in (-30..=30).step_by(10) {
            let (a, b) = ((a as f32).to_radians(), (b as f32).to_radians());
            v.push(Quat::from_rotation_z(b) * Vec3::new(0.0, -a.cos(), -a.sin()));
        }
    }
    v
}

/// Measures the turntable axes from `rotation` (stops at tilt 0, in
/// order, commanded angles strictly monotonic) and `tilt` (stops at the
/// last rotation angle and other tilts, registered onto the last rotation
/// stop). See the module doc.
///
/// # Errors
/// [`ScanError::CalibrationRefused`] (with the report) when a check fails.
pub fn measure(
    intr: &CameraIntrinsics,
    rotation: &[CalibrationStop],
    tilt: &[CalibrationStop],
) -> std::result::Result<CalibrationOutcome, Refusal> {
    let mut report = CalibrationReport {
        pairs_expected: rotation.len().saturating_sub(1) as u32,
        ..CalibrationReport::default()
    };
    if rotation.len() < 3 {
        return Err(refuse(
            "calibration-tracking",
            format!("{} stops, at least 3 needed", rotation.len()),
            &report,
        ));
    }
    let frames: Vec<Arc<Vec<f32>>> = rotation.iter().map(|s| Arc::clone(&s.depth)).collect();
    let background = Background::from_frames(&frames, BACKGROUND_TOL_MM);
    let objects = object_depths(rotation, &background, intr);
    let pixels: usize = objects
        .iter()
        .map(|d| d.iter().filter(|z| z.is_finite()).count())
        .sum::<usize>()
        / objects.len();
    report.object_pixels = pixels as u32;
    if pixels < MIN_OBJECT_PIXELS {
        return Err(refuse(
            "calibration-no-object",
            format!(
                "{pixels} moving pixels per stop (at least {MIN_OBJECT_PIXELS}): nothing turns on \
                 the platter, or the table did not move"
            ),
            &report,
        ));
    }
    let centroids: Vec<Vec3> = objects
        .iter()
        .map(|d| {
            let p = points(d, intr, 3);
            p.iter().copied().sum::<Vec3>() / p.len().max(1) as f32
        })
        .collect();

    // coarse axis: the best registration of the first pair over the seeds
    let step0 = f64::from(rotation[1].rotate_deg - rotation[0].rotate_deg);
    let target0 = single_view(&objects[1], intr);
    let mut dirs = Vec::new();
    if let Some(n) = centroid_plane_normal(&centroids) {
        dirs.push(n);
    }
    dirs.extend(up_grid());
    let mut best: Option<TrackResult> = None;
    for (k, d) in dirs.iter().enumerate() {
        let point = circle_centre(&centroids, *d);
        for sign in [1.0, -1.0] {
            let seed = about(point, *d * sign, step0 as f32);
            let r = register(&target0, &objects[0], intr, &seed);
            best = Some(better(best, r));
        }
        if k == 0 && best.is_some_and(|b| b.ok && b.rms_mm < 0.15) {
            break;
        }
    }
    // signed so that the commanded step turns right-handed about it
    let coarse = best
        .filter(|b| b.ok)
        .and_then(|b| screw_axis(&b.pose))
        .map(|(p, d)| (p, d * step0.signum() as f32, 1.0));
    let Some(mut axis_seed) = coarse else {
        return Err(refuse(
            "calibration-tracking",
            "the first two stops could not be registered: use a matte, asymmetric object of at \
             least 20 mm"
                .into(),
            &report,
        ));
    };

    // pairs, twice: seeded by the coarse axis, then by the fitted one. A
    // pixel at the same depth in both stops of a pair votes for no motion
    // (static clutter the background missed): it is left out of the source
    let targets: Vec<TsdfVolume> = objects[1..]
        .par_iter()
        .map(|d| single_view(d, intr))
        .collect();
    let sources: Vec<Vec<f32>> = (0..rotation.len() - 1)
        .into_par_iter()
        .map(|k| {
            objects[k]
                .iter()
                .zip(&objects[k + 1])
                .map(|(&a, &b)| {
                    if b.is_finite() && (a - b).abs() <= PAIR_STATIC_MM {
                        f32::NAN
                    } else {
                        a
                    }
                })
                .collect()
        })
        .collect();
    let mut fit = None;
    let mut pairs = Vec::new();
    for _ in 0..2 {
        let (point, dir, gain) = axis_seed;
        pairs = (0..rotation.len() - 1)
            .into_par_iter()
            .filter_map(|k| {
                let delta = f64::from(rotation[k + 1].rotate_deg - rotation[k].rotate_deg);
                let seed = about(point, dir, gain * delta as f32);
                let r = register(&targets[k], &sources[k], intr, &seed);
                r.ok.then_some((k, delta, r.pose))
            })
            .collect::<Vec<_>>();
        let motions: Vec<(f64, DAffine3)> = pairs
            .iter()
            .map(|(_, d, m)| (*d, super::fit::to_d(m)))
            .collect();
        report.pairs = motions.len() as u32;
        let min_pairs = ((MIN_PAIR_FRACTION * report.pairs_expected as f32).ceil() as usize).max(3);
        if motions.len() < min_pairs {
            return Err(refuse(
                "calibration-tracking",
                format!(
                    "{} of {} stop pairs registered (at least {min_pairs})",
                    motions.len(),
                    report.pairs_expected
                ),
                &report,
            ));
        }
        let f = fit_motions(&motions)
            .map_err(|e| refuse("calibration-tracking", e.to_string(), &report))?;
        axis_seed = (f.point, f.dir, f.angle_gain);
        fit = Some(f);
    }
    let axis = fit.unwrap_or_else(|| unreachable!());
    report.rms_deg = Some(axis.rms_deg);
    report.rms_mm = Some(axis.rms_mm);
    report.angle_gain = Some(axis.angle_gain);
    // a full turn that ends on the first view proves the table exact: the
    // registration's gain is then a bias of the geometry, not of the table
    let total = rotation[rotation.len() - 1].rotate_deg - rotation[0].rotate_deg;
    let verified = (total.abs() - 360.0).abs() <= 0.5 && {
        let first = single_view(&objects[0], intr);
        let r = register(
            &first,
            &objects[rotation.len() - 1],
            intr,
            &Affine3A::IDENTITY,
        );
        r.ok && dc_geom::icp::rotation_angle(&super::fit::to_d(&r.pose)).to_degrees() < 1.0
    };
    report.loop_verified = Some(verified);
    let angle_gain = if verified { 1.0 } else { axis.angle_gain };

    // platter: the base of the object along the axis, from the points that
    // turn with it (the static scene it uncovers does not)
    let all: Vec<Vec3> = pairs
        .par_iter()
        .flat_map_iter(|(k, _, m)| consistent(&objects[*k], &objects[k + 1], intr, m))
        .collect();
    let (platter_point, up) = platter(&axis, &all)
        .map_err(|e| refuse("calibration-no-object", e.to_string(), &report))?;
    let distance = platter_point.length();
    report.axis_distance_mm = Some(distance);
    let view = up.angle_between(-platter_point).to_degrees();
    report.view_angle_deg = Some(view);

    // tilt
    let tilt_fit = if tilt.is_empty() {
        None
    } else {
        Some(measure_tilt(
            intr,
            rotation,
            tilt,
            &background,
            platter_point,
            up,
            &mut report,
        )?)
    };

    // checks
    if axis.rms_deg > MAX_RMS_DEG || axis.rms_mm > MAX_RMS_MM {
        return Err(refuse(
            "calibration-inaccurate",
            format!(
                "axis residuals {:.2} deg / {:.2} mm (at most {MAX_RMS_DEG} / {MAX_RMS_MM})",
                axis.rms_deg, axis.rms_mm
            ),
            &report,
        ));
    }
    if let Some(t) = &tilt_fit
        && (t.rms_deg > MAX_RMS_DEG || t.rms_mm > MAX_RMS_MM)
    {
        return Err(refuse(
            "calibration-inaccurate",
            format!(
                "tilt axis residuals {:.2} deg / {:.2} mm (at most {MAX_RMS_DEG} / {MAX_RMS_MM})",
                t.rms_deg, t.rms_mm
            ),
            &report,
        ));
    }
    if !(AXIS_DISTANCE_MM.0..=AXIS_DISTANCE_MM.1).contains(&distance)
        || view >= 85.0
        || (angle_gain - 1.0).abs() > MAX_GAIN_ERROR
    {
        return Err(refuse(
            "calibration-implausible",
            format!(
                "axis {distance:.0} mm from the scanner (expected {}..{}), seen at {view:.0} deg \
                 from the platter normal (below 85), angle gain {angle_gain:.3}",
                AXIS_DISTANCE_MM.0, AXIS_DISTANCE_MM.1
            ),
            &report,
        ));
    }
    Ok(CalibrationOutcome {
        axis,
        tilt: tilt_fit,
        platter_point,
        up,
        angle_gain,
        report,
    })
}

/// Normal of the plane of the stop centroids, facing the camera, when
/// they spread enough to define one.
fn centroid_plane_normal(c: &[Vec3]) -> Option<Vec3> {
    if c.len() < 3 {
        return None;
    }
    let mean = c.iter().copied().sum::<Vec3>() / c.len() as f32;
    let mut cov = nalgebra::Matrix3::<f64>::zeros();
    for p in c {
        let d = (*p - mean).as_dvec3();
        let v = nalgebra::Vector3::new(d.x, d.y, d.z);
        cov += v * v.transpose();
    }
    let eig = cov.symmetric_eigen();
    let mut order = [0, 1, 2];
    order.sort_by(|a, b| eig.eigenvalues[*a].total_cmp(&eig.eigenvalues[*b]));
    // the two in-plane spreads must be clear of the normal one
    if eig.eigenvalues[order[1]] < 4.0 * c.len() as f64 {
        return None;
    }
    let n = eig.eigenvectors.column(order[0]);
    let n = Vec3::new(n[0] as f32, n[1] as f32, n[2] as f32).try_normalize()?;
    Some(if n.dot(-mean) >= 0.0 { n } else { -n })
}

/// The tilt axis from the `tilt` stops registered onto the last rotation
/// stop. Without the background (the tilted platter moves), the object is
/// what lies near the platter point.
fn measure_tilt(
    intr: &CameraIntrinsics,
    rotation: &[CalibrationStop],
    tilt: &[CalibrationStop],
    background: &Background,
    platter_point: Vec3,
    up: Vec3,
    report: &mut CalibrationReport,
) -> std::result::Result<AxisFit, Refusal> {
    let w = intr.width as usize;
    let near = |d: &[f32], drop_static: bool| -> Vec<f32> {
        d.iter()
            .enumerate()
            .map(|(i, &z)| {
                let keep = z.is_finite()
                    && z > 0.0
                    && !(drop_static && background.is_static(i, z))
                    && (intr.ray((i % w) as f32, (i / w) as f32) * z).distance(platter_point)
                        <= OBJECT_RADIUS_GATE_MM;
                if keep { z } else { f32::NAN }
            })
            .collect()
    };
    let reference = rotation.last().unwrap_or_else(|| unreachable!());
    let target = single_view(&near(&reference.depth, true), intr);
    let side = Vec3::X - Vec3::X.dot(up) * up;
    let side = side
        .try_normalize()
        .unwrap_or_else(|| up.any_orthonormal_vector());
    let mut samples = vec![(0.0_f32, Affine3A::IDENTITY)];
    for s in tilt {
        let delta = s.tilt_deg - reference.tilt_deg;
        let source = near(&s.depth, true);
        let mut best: Option<TrackResult> = None;
        for depth in TILT_PIVOT_SEEDS_MM {
            for sign in [1.0, -1.0] {
                // object motion from this tilt back to the reference
                let seed = about(platter_point - depth * up, side * sign, -delta);
                best = Some(better(best, register(&target, &source, intr, &seed)));
            }
        }
        match best {
            Some(r) if r.ok => samples.push((delta, r.pose)),
            _ => {
                return Err(refuse(
                    "calibration-tilt",
                    format!("the stop at tilt {} could not be registered", s.tilt_deg),
                    report,
                ));
            }
        }
    }
    let fit = fit_axis(&samples).map_err(|e| refuse("calibration-tilt", e.to_string(), report))?;
    report.tilt_rms_deg = Some(fit.rms_deg);
    report.tilt_rms_mm = Some(fit.rms_mm);
    Ok(fit)
}

/// Platter point on the axis and platter normal towards the camera, from
/// the heights along the axis of the `points` (camera frame) within
/// [`OBJECT_RADIUS_GATE_MM`] of it: the lowest 0.5 mm layer holding at
/// least a fifth of the points of the densest layer (the base of the
/// object), so a few stray points below do not count.
///
/// # Errors
/// [`ScanError::Calibration`] with too few points near the axis.
pub fn platter(axis: &AxisFit, points: &[Vec3]) -> Result<(Vec3, Vec3)> {
    const BIN_MM: f32 = 0.5;
    // the camera (origin) looks down on the platter
    let up = if (-axis.point).dot(axis.dir) >= 0.0 {
        axis.dir
    } else {
        -axis.dir
    };
    let r = OBJECT_RADIUS_GATE_MM;
    let heights: Vec<f32> = points
        .iter()
        .filter_map(|p| {
            let v = *p - axis.point;
            let h = v.dot(up);
            ((v - h * up).length_squared() <= r * r && h.is_finite()).then_some(h)
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
    let layer: Vec<f32> = heights
        .into_iter()
        .filter(|h| (from..from + 3.0 * BIN_MM).contains(h))
        .collect();
    let h = median(layer).unwrap_or(0.0);
    Ok((axis.point + h * up, up))
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
            angle_gain: 1.0,
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
        let (p, up) = platter(&axis, &points).unwrap();
        assert_eq!(up, -Vec3::Z);
        assert!((p - Vec3::new(0.0, 0.0, 140.0)).length() < 1e-4, "{p}");
        assert!(platter(&axis, &points[..30]).is_err());
    }

    #[test]
    fn median_needs_most_frames() {
        let f = |v: Vec<f32>| Arc::new(v);
        let m = median_depth(&[
            f(vec![1.0, 1.0, f32::NAN]),
            f(vec![3.0, f32::NAN, f32::NAN]),
            f(vec![2.0, 0.0, 5.0]),
        ]);
        assert_eq!(m[0], 2.0);
        assert!(m[1].is_nan() && m[2].is_nan());
        assert_eq!(median_depth(&[f(vec![1.0]), f(vec![3.0])])[0], 2.0);
    }

    #[test]
    fn screw_axis_of_a_turn_about_a_line() {
        let m = about(Vec3::new(5.0, 0.0, 100.0), Vec3::Y, 15.0);
        let (p, d) = screw_axis(&m).unwrap();
        assert!(d.cross(Vec3::Y).length() < 1e-4);
        let off = p - Vec3::new(5.0, 0.0, 100.0);
        assert!((off - off.dot(Vec3::Y) * Vec3::Y).length() < 1e-2, "{p}");
        let c = circle_centre(
            &(0..8)
                .map(|k| {
                    let a = k as f32 * 0.6;
                    Vec3::new(3.0 + 10.0 * a.cos(), 7.0, 100.0 + 10.0 * a.sin())
                })
                .collect::<Vec<_>>(),
            Vec3::Y,
        );
        assert!((c - Vec3::new(3.0, 7.0, 100.0)).length() < 1e-2, "{c}");
    }

    /// Settled stops of the simulated rig at the table angles `angles`
    /// (degrees), with the camera-to-scene pose of the rig camera.
    fn sim_stops(angles: &[f32]) -> (CameraIntrinsics, Vec<CalibrationStop>, Affine3A) {
        use crate::ScannerBackend;
        use crate::sim::{SimBackend, SimDevice, SimOptions, rig_camera_pose, typodont_arch};
        use dc_turntable::sim::{RigPose, SimRig};
        let options = SimOptions {
            realtime: false,
            width: 200,
            height: 125,
            ..SimOptions::default()
        };
        let camera = rig_camera_pose(&options);
        let info = SimBackend::default().list().unwrap().remove(0);
        let dev = SimDevice::new(info, &typodont_arch(true), options)
            .unwrap()
            .with_rig(SimRig::new())
            .unwrap();
        let stops: Vec<CalibrationStop> = angles
            .iter()
            .enumerate()
            .map(|(k, a)| {
                let f = dev.render_rig(
                    k as u64,
                    RigPose {
                        rotate_deg: *a,
                        tilt_deg: 0.0,
                    },
                );
                CalibrationStop {
                    rotate_deg: *a,
                    tilt_deg: 0.0,
                    depth: Arc::new(f.depth_mm),
                }
            })
            .collect();
        let intr = crate::sim::intrinsics(200, 125);
        (intr, stops, camera)
    }

    #[test]
    fn calibration_accepts_a_turning_object_and_refuses_a_still_one() {
        let angles: Vec<f32> = (0..=12).map(|k| 30.0 * k as f32).collect();
        let (intr, stops, camera) = sim_stops(&angles);
        let o = measure(&intr, &stops, &[]).unwrap_or_else(|(e, r)| panic!("{e}: {r:?}"));
        let truth = camera.inverse().transform_vector3(Vec3::Z);
        assert!(o.axis.dir.angle_between(truth).to_degrees() < 0.5, "{o:?}");
        assert_eq!(o.report.loop_verified, Some(true));
        assert_eq!(o.angle_gain, 1.0);
        assert!(o.report.rms_deg.unwrap() < 0.2, "{:?}", o.report);

        // the turn is measured, but a tilted stop shows nothing to register:
        // refused with the code of the tilt, not of the turn
        let blank = CalibrationStop {
            rotate_deg: 360.0,
            tilt_deg: 15.0,
            depth: Arc::new(vec![f32::NAN; stops[0].depth.len()]),
        };
        let (e, report) = measure(&intr, &stops, &[blank]).unwrap_err();
        assert_eq!(e.code(), "calibration-tilt", "{e}");
        assert!(report.rms_deg.is_some(), "{report:?}");

        // the table never moved: nothing turns, the run is refused
        let still = vec![0.0; 13];
        let (_, frames, _) = sim_stops(&still);
        let commanded: Vec<CalibrationStop> = frames
            .into_iter()
            .zip(&angles)
            .map(|(s, a)| CalibrationStop {
                rotate_deg: *a,
                ..s
            })
            .collect();
        let (e, report) = measure(&intr, &commanded, &[]).unwrap_err();
        assert_eq!(e.code(), "calibration-no-object", "{e}");
        assert!(report.rms_deg.is_none());
    }
}
