//! Rotation axis of a turntable from rigid motions of the object.
//!
//! With the scanner fixed and the object turning, the tracked camera poses
//! `P_k` (camera to world, world = first fused camera) describe the motion
//! of the object in the camera frame: `Q_ij = P_j^-1 P_i` moves the object
//! from sample `i` to sample `j`, a rotation by the commanded angle
//! difference about the turntable axis. For every pair with a usable angle
//! difference:
//!
//! * the axis direction `a` is the common rotation axis: the eigenvector of
//!   eigenvalue 1 of every `R_ij`, found as the null vector of
//!   `sum (R_ij - I)^T (R_ij - I)`, signed so that positive commanded angles
//!   turn right-handed about `a`;
//! * a point `c` on the axis is fixed by every motion, `R c + t = c`: the
//!   stacked `(I - R_ij) c = t_ij` are solved by least squares, pinned
//!   along `a` by the extra equation `a . c = 0` (the foot of the camera
//!   origin on the axis).

use glam::{Affine3A, DAffine3, DMat3, DVec3, Vec3};

use crate::{Result, ScanError};

/// Smallest and largest commanded angle difference (degrees) of a pair.
const PAIR_RANGE_DEG: (f64, f64) = (5.0, 150.0);

/// A fitted rotation axis (camera frame, millimetres).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AxisFit {
    /// Point of the axis nearest to the camera origin.
    pub point: Vec3,
    /// Unit direction: positive angles turn right-handed about it.
    pub dir: Vec3,
    /// RMS difference between the fitted angles and the commanded ones
    /// times [`AxisFit::angle_gain`] (deg).
    pub rms_deg: f32,
    /// RMS residual of the fixed-point equations (mm).
    pub rms_mm: f32,
    /// Pairs used.
    pub pairs: usize,
    /// Least-squares ratio of the fitted over the commanded angles.
    pub angle_gain: f32,
}

pub(crate) fn to_d(a: &Affine3A) -> DAffine3 {
    DAffine3::from_mat3_translation(a.matrix3.as_dmat3(), a.translation.as_dvec3())
}

/// `a` wrapped to `(-180, 180]` degrees.
fn wrap_deg(a: f64) -> f64 {
    let w = a.rem_euclid(360.0);
    if w > 180.0 { w - 360.0 } else { w }
}

/// `vee((R - R^T) / 2)`: `sin(angle) * axis`.
fn skew_vector(r: &DMat3) -> DVec3 {
    0.5 * DVec3::new(
        r.y_axis.z - r.z_axis.y,
        r.z_axis.x - r.x_axis.z,
        r.x_axis.y - r.y_axis.x,
    )
}

/// Signed rotation angle of `r` about the unit axis `a` (radians).
pub fn angle_about(r: &DMat3, a: DVec3) -> f64 {
    let cos = 0.5 * (r.x_axis.x + r.y_axis.y + r.z_axis.z - 1.0);
    skew_vector(r).dot(a).atan2(cos.clamp(-1.0, 1.0))
}

/// Unit null vector of the symmetric positive semi-definite `m` whose two
/// other eigenvalues are large: the longest cross product of two rows.
fn null_vector(m: &DMat3) -> Option<DVec3> {
    let rows = [m.row(0), m.row(1), m.row(2)];
    let best = [
        rows[0].cross(rows[1]),
        rows[0].cross(rows[2]),
        rows[1].cross(rows[2]),
    ]
    .into_iter()
    .max_by(|a, b| a.length_squared().total_cmp(&b.length_squared()))?;
    best.try_normalize()
}

/// Fits the rotation axis of `samples`: commanded angle (degrees,
/// cumulative) and tracked camera-to-world pose at that angle.
///
/// # Errors
/// [`ScanError::Calibration`] with fewer than two usable pairs (angle
/// differences of 5 to 150 degrees) or degenerate motions.
pub fn fit_axis(samples: &[(f32, Affine3A)]) -> Result<AxisFit> {
    let mut motions = Vec::new();
    for (i, (ai, pi)) in samples.iter().enumerate() {
        for (aj, pj) in &samples[i + 1..] {
            let delta = wrap_deg(f64::from(*aj) - f64::from(*ai));
            if !(PAIR_RANGE_DEG.0..=PAIR_RANGE_DEG.1).contains(&delta.abs()) {
                continue;
            }
            let q = to_d(pj).inverse() * to_d(pi);
            motions.push((delta, q));
        }
    }
    fit_motions(&motions)
}

/// Fits the rotation axis of rigid motions of the object in the camera
/// frame: `(commanded angle difference in degrees, motion)` where the
/// motion moves the object from the first angle to the second.
///
/// # Errors
/// [`ScanError::Calibration`] with fewer than two motions or degenerate
/// ones (the object did not turn).
pub fn fit_motions(motions: &[(f64, DAffine3)]) -> Result<AxisFit> {
    let fail = |m: &str| ScanError::Calibration(m.to_owned());
    if motions.len() < 2 {
        return Err(fail("too few rotations to fit the axis"));
    }
    let mut m = DMat3::ZERO;
    for (_, q) in motions {
        let d = q.matrix3 - DMat3::IDENTITY;
        m += d.transpose() * d;
    }
    // a still object leaves every R = I: nothing to fit
    if m.to_cols_array().iter().map(|v| v.abs()).sum::<f64>() < 1e-6 * motions.len() as f64 {
        return Err(fail("the object did not turn"));
    }
    let mut dir = null_vector(&m).ok_or_else(|| fail("the object did not turn"))?;
    let handed: f64 = motions
        .iter()
        .map(|(delta, q)| delta.signum() * skew_vector(&q.matrix3).dot(dir))
        .sum();
    if handed < 0.0 {
        dir = -dir;
    }
    // (I - R) c = t for every motion, plus a . c = 0
    let mut ata = DMat3::from_cols(dir * dir.x, dir * dir.y, dir * dir.z);
    let mut atb = DVec3::ZERO;
    for (_, q) in motions {
        let a = DMat3::IDENTITY - q.matrix3;
        ata += a.transpose() * a;
        atb += a.transpose() * q.translation;
    }
    if ata.determinant().abs() < 1e-12 {
        return Err(fail("degenerate motions"));
    }
    let point = ata.inverse() * atb;
    let measured: Vec<f64> = motions
        .iter()
        .map(|(_, q)| angle_about(&q.matrix3, dir).to_degrees())
        .collect();
    let (num, den) = motions
        .iter()
        .zip(&measured)
        .fold((0.0, 0.0), |(n, d), ((delta, _), m)| {
            (n + delta * m, d + delta * delta)
        });
    let gain = if den > 0.0 { num / den } else { 1.0 };
    let (mut sq_mm, mut sq_deg) = (0.0, 0.0);
    for ((delta, q), m) in motions.iter().zip(&measured) {
        let res = (DMat3::IDENTITY - q.matrix3) * point - q.translation;
        sq_mm += res.length_squared();
        sq_deg += (m - gain * delta).powi(2);
    }
    let n = motions.len() as f64;
    Ok(AxisFit {
        point: point.as_vec3(),
        dir: dir.as_vec3(),
        rms_deg: (sq_deg / n).sqrt() as f32,
        rms_mm: (sq_mm / (3.0 * n)).sqrt() as f32,
        pairs: motions.len(),
        angle_gain: gain as f32,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::{DQuat, Quat};

    /// Object pose in the camera frame at angle `deg` about the axis.
    fn about(point: DVec3, dir: DVec3, deg: f64) -> DAffine3 {
        DAffine3::from_translation(point)
            * DAffine3::from_quat(DQuat::from_axis_angle(dir, deg.to_radians()))
            * DAffine3::from_translation(-point)
    }

    /// Deterministic pseudo-random numbers in `[-1, 1]`.
    fn noise(seed: &mut u64) -> f64 {
        *seed = seed
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        ((*seed >> 11) as f64 / (1u64 << 53) as f64) * 2.0 - 1.0
    }

    #[test]
    fn recovers_a_known_axis_from_noisy_poses() {
        let point = DVec3::new(4.0, -12.0, 140.0);
        let dir = DVec3::new(0.1, -0.85, -0.5).normalize();
        let mut seed = 7;
        let mut samples = Vec::new();
        // world = camera at the first sample: P_k = M_0 M_k^-1
        let m0 = about(point, dir, 10.0);
        for k in 0..8 {
            let deg = 10.0 + 45.0 * k as f64;
            let pk = m0 * about(point, dir, deg).inverse();
            // tracking noise: 0.05 degree, 0.03 mm
            let axis = DVec3::new(noise(&mut seed), noise(&mut seed), noise(&mut seed))
                .try_normalize()
                .unwrap_or(DVec3::X);
            let wobble = DAffine3::from_quat(DQuat::from_axis_angle(
                axis,
                (0.05 * noise(&mut seed)).to_radians(),
            )) * DAffine3::from_translation(
                0.03 * DVec3::new(noise(&mut seed), noise(&mut seed), noise(&mut seed)),
            );
            let p = pk * wobble;
            let p = Affine3A::from_mat3_translation(p.matrix3.as_mat3(), p.translation.as_vec3());
            samples.push((deg as f32, p));
        }
        let fit = fit_axis(&samples).unwrap();
        let angle = fit.dir.as_dvec3().angle_between(dir).to_degrees();
        assert!(angle < 0.5, "direction off by {angle} degrees");
        // distance of the true point from the fitted line
        let v = point - fit.point.as_dvec3();
        let off = (v - v.dot(dir) * dir).length();
        assert!(off < 0.5, "axis off by {off} mm");
        assert!(fit.rms_deg < 0.5 && fit.rms_mm < 0.5, "{fit:?}");
        assert!(fit.point.as_dvec3().dot(fit.dir.as_dvec3()).abs() < 1e-3);

        // negative commanded angles flip the direction
        let flipped: Vec<(f32, Affine3A)> = samples.iter().map(|(a, p)| (-a, *p)).collect();
        let fit2 = fit_axis(&flipped).unwrap();
        assert!(fit2.dir.dot(-fit.dir) > 0.9999);
    }

    #[test]
    fn refuses_too_few_or_still_samples() {
        let id = Affine3A::IDENTITY;
        assert_eq!(fit_axis(&[(0.0, id)]).unwrap_err().code(), "calibration");
        let still = [(0.0, id), (45.0, id), (90.0, id)];
        assert_eq!(fit_axis(&still).unwrap_err().code(), "calibration");
        let q = Quat::from_rotation_z(0.3);
        let r = Affine3A::from_quat(q);
        // 180 degree pairs are skipped
        assert!(fit_axis(&[(0.0, id), (180.0, r)]).is_err());
        assert_eq!(wrap_deg(315.0), -45.0);
        assert_eq!(wrap_deg(-190.0), 170.0);
    }
}
