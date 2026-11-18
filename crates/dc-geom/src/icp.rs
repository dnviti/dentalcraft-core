//! Rigid registration: weighted Kabsch / Umeyama and trimmed iterative
//! closest point (WP14, port plan B2.4).
//!
//! Port of the add-on's registrations (`visit.py`: `kabsch`,
//! `point_to_plane`, `icp`; `bite.py`: `_icp`, `_ficp_keep`):
//!
//! * [`kabsch`]: least-squares rigid motion (Kabsch 1976), with weights and
//!   an optional uniform scale (Umeyama 1991);
//! * [`point_to_plane_step`]: one linearised point-to-plane solve about the
//!   centroid of the moving points (Chen & Medioni 1992), the rotation
//!   recovered exactly from the rotation vector;
//! * [`icp`]: trimmed ICP of a point set onto a surface. Pairs come from the
//!   exact closest point of the target triangles ([`Bvh`]), optionally gated
//!   by distance and by normal compatibility (Rusinkiewicz & Levoy 2001), and
//!   trimmed either to a fixed fraction (Chetverikov 2002, `visit.icp`) or to
//!   the automatic overlap of FICP (Chetverikov 2005, `bite._icp`).
//!
//! [`IcpParams::default`] matches `visit.icp` (50 iterations, trim 0.8,
//! tolerance 1e-7 on the RMS, normal gate 60°, point-to-plane);
//! [`IcpParams::bite`] gives the variant of `bite._icp`.
//!
//! The reported error of an iteration is measured on its pairs before its
//! update, as in the add-on: the RMS signed point-to-plane distance
//! (point-to-plane) or the RMS pair distance (point-to-point).

use glam::{DAffine3, DMat3, DVec3, Vec3};
use nalgebra::{Matrix3, Matrix6, Vector6};
use rayon::prelude::*;

use dc_mesh::TriMesh;

use crate::bvh::Bvh;
use crate::{GeomError, Result};

/// Fewest pairs an iteration needs (the add-on's `12`).
pub const MIN_PAIRS: usize = 12;

/// Relative cutoff of the singular values in the point-to-plane solve.
const PLANE_RCOND: f64 = 1e-12;

/// Error minimised by [`icp`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IcpMetric {
    /// Distance between paired points, solved with [`kabsch`] (Besl & McKay).
    PointToPoint,
    /// Distance to the tangent plane of the target, linearised (Chen & Medioni).
    PointToPlane,
}

/// Rejection of the worst pairs at every iteration.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Trim {
    /// Keep every pair that passed the gates.
    None,
    /// Keep the pairs whose distance is at most the given quantile of the
    /// distances (linear interpolation, as `numpy.quantile`). `visit.icp`.
    Fraction(f64),
    /// Automatic overlap (FICP, Chetverikov 2005): keep the `xi * n` closest
    /// pairs, `xi >= xi_min` minimising `e(xi) / xi^3` (`n` = number of source
    /// points, `e` = mean squared distance). `bite._icp`.
    Auto {
        /// Smallest overlap considered (the add-on uses 0.2 and 0.1).
        xi_min: f64,
    },
}

/// ICP parameters.
#[derive(Debug, Clone, PartialEq)]
pub struct IcpParams {
    /// Maximum number of iterations.
    pub max_iterations: usize,
    /// Error minimised at every iteration.
    pub metric: IcpMetric,
    /// Trimming of the worst pairs.
    pub trim: Trim,
    /// Source points farther than this from the target get no pair (mm).
    pub max_pair_distance: f64,
    /// Pairs whose normals differ by more than this are rejected (degrees).
    /// Used only when source normals are given.
    pub max_normal_angle: Option<f64>,
    /// Stop when the reported RMS changes by less than this (mm); 0 disables.
    pub tolerance: f64,
    /// Stop when the mean squared pair distance changes by less than this
    /// (mm²); 0 disables. `bite._icp` uses 1e-9.
    pub mse_tolerance: f64,
    /// Stop when an update rotates by less than this (radians) and
    /// translates by less than [`IcpParams::min_step_translation`]; 0
    /// disables. `bite._icp` uses 1e-5.
    pub min_step_rotation: f64,
    /// See [`IcpParams::min_step_rotation`] (mm). `bite._icp` uses 1e-4.
    pub min_step_translation: f64,
    /// Fewest pairs (after the gates, before trimming) an iteration needs;
    /// below this the target is considered lost and the iteration stops.
    pub min_pairs: usize,
    /// Initial transform applied to the source.
    pub initial: DAffine3,
}

impl Default for IcpParams {
    /// The defaults of `visit.icp`.
    fn default() -> Self {
        Self {
            max_iterations: 50,
            metric: IcpMetric::PointToPlane,
            trim: Trim::Fraction(0.8),
            max_pair_distance: f64::INFINITY,
            max_normal_angle: Some(60.0),
            tolerance: 1e-7,
            mse_tolerance: 0.0,
            min_step_rotation: 0.0,
            min_step_translation: 0.0,
            min_pairs: MIN_PAIRS,
            initial: DAffine3::IDENTITY,
        }
    }
}

impl IcpParams {
    /// The variant of `bite._icp(src, src_n, surf, T, iters, gate, cos_min,
    /// plane, xi_min)`: distance gate, normal gate, automatic overlap,
    /// stop on a small update or a still mean squared distance.
    pub fn bite(
        iterations: usize,
        gate: f64,
        max_normal_angle: f64,
        metric: IcpMetric,
        xi_min: f64,
    ) -> Self {
        Self {
            max_iterations: iterations,
            metric,
            trim: Trim::Auto { xi_min },
            max_pair_distance: gate,
            max_normal_angle: Some(max_normal_angle),
            tolerance: 0.0,
            mse_tolerance: 1e-9,
            min_step_rotation: 1e-5,
            min_step_translation: 1e-4,
            min_pairs: MIN_PAIRS,
            initial: DAffine3::IDENTITY,
        }
    }
}

/// Outcome of a registration.
#[derive(Debug, Clone, PartialEq)]
pub struct IcpResult {
    /// Rigid transform mapping the source onto the target.
    pub transform: DAffine3,
    /// Error of the last iteration (mm, see the module doc).
    pub rms: f64,
    /// Iterations performed (updates applied).
    pub iterations: usize,
    /// Fraction of source points with a kept pair in the last iteration.
    pub overlap: f64,
    /// Error of every iteration, in order.
    pub rms_history: Vec<f64>,
    /// A stopping criterion was met before `max_iterations`.
    pub converged: bool,
    /// Too few pairs were left (fewer than `min_pairs`): the target was lost.
    pub lost: bool,
}

/// Target surface of [`icp`]: the BVH and unit normals of its triangles.
#[derive(Debug, Clone)]
pub struct IcpTarget {
    bvh: Bvh,
    normals: Vec<DVec3>,
}

impl IcpTarget {
    /// Prepares `mesh` as a registration target.
    ///
    /// # Errors
    /// [`GeomError::InvalidInput`] for a mesh without triangles or with an
    /// index out of range.
    pub fn new(mesh: &TriMesh) -> Result<Self> {
        if mesh.tris.is_empty() {
            return Err(GeomError::InvalidInput("target has no triangles".into()));
        }
        let bvh = Bvh::build(mesh)?;
        let normals = mesh
            .tris
            .par_iter()
            .map(|t| {
                let [a, b, c] = t.map(|i| mesh.positions[i as usize].as_dvec3());
                (b - a).cross(c - a).normalize_or_zero()
            })
            .collect();
        Ok(Self { bvh, normals })
    }

    /// The hierarchy over the target triangles.
    pub fn bvh(&self) -> &Bvh {
        &self.bvh
    }

    /// Unit normal of target triangle `face` (zero for a degenerate one).
    pub fn face_normal(&self, face: u32) -> DVec3 {
        self.normals[face as usize]
    }
}

/// Least-squares similarity `x -> s R x + t` moving `src` onto `dst`
/// (weighted Kabsch; with `with_scale` the Umeyama scale, else `s = 1`).
/// Reflections are excluded (`det R = +1`).
///
/// # Errors
/// [`GeomError::InvalidInput`] when the sets are empty or of different
/// lengths, or the weights do not match or sum to zero.
pub fn kabsch(
    src: &[DVec3],
    dst: &[DVec3],
    weights: Option<&[f64]>,
    with_scale: bool,
) -> Result<DAffine3> {
    if src.is_empty() || src.len() != dst.len() {
        return Err(GeomError::InvalidInput(format!(
            "kabsch needs two non-empty sets of equal length ({} and {})",
            src.len(),
            dst.len()
        )));
    }
    if let Some(w) = weights
        && w.len() != src.len()
    {
        return Err(GeomError::InvalidInput(format!(
            "{} weights for {} points",
            w.len(),
            src.len()
        )));
    }
    let w = |i: usize| weights.map_or(1.0, |w| w[i]);
    let total: f64 = (0..src.len()).map(w).sum();
    if total.partial_cmp(&0.0) != Some(std::cmp::Ordering::Greater) || !total.is_finite() {
        return Err(GeomError::InvalidInput(
            "weights must sum to a positive value".into(),
        ));
    }
    let ca = (0..src.len()).map(|i| src[i] * w(i)).sum::<DVec3>() / total;
    let cb = (0..dst.len()).map(|i| dst[i] * w(i)).sum::<DVec3>() / total;
    let mut h = Matrix3::<f64>::zeros();
    let mut var_a = 0.0;
    for i in 0..src.len() {
        let a = src[i] - ca;
        let b = dst[i] - cb;
        let wi = w(i);
        var_a += wi * a.length_squared();
        for r in 0..3 {
            for c in 0..3 {
                h[(r, c)] += wi * a[r] * b[c];
            }
        }
    }
    let svd = h.svd(true, true);
    let (Some(u), Some(vt)) = (svd.u, svd.v_t) else {
        return Err(GeomError::InvalidInput("SVD failed".into()));
    };
    // nalgebra sorts the singular values in decreasing order: the reflection
    // fix flips the direction of the smallest one.
    let d = (vt.transpose() * u.transpose()).determinant();
    let d = if d < 0.0 { -1.0 } else { 1.0 };
    let dm = Matrix3::from_diagonal(&nalgebra::Vector3::new(1.0, 1.0, d));
    let r = vt.transpose() * dm * u.transpose();
    let s = if with_scale && var_a > 0.0 {
        let sv = svd.singular_values;
        (sv[0] + sv[1] + d * sv[2]) / var_a
    } else {
        1.0
    };
    let rot = DMat3::from_cols(
        DVec3::new(r[(0, 0)], r[(1, 0)], r[(2, 0)]),
        DVec3::new(r[(0, 1)], r[(1, 1)], r[(2, 1)]),
        DVec3::new(r[(0, 2)], r[(1, 2)], r[(2, 2)]),
    ) * s;
    Ok(DAffine3::from_mat3_translation(rot, cb - rot * ca))
}

/// Rigid motion moving the points `p` towards the planes through `q` with
/// normals `n`: linearised point-to-plane least squares about the centroid
/// of `p` (`visit.point_to_plane`). The rotation is the exact rotation of the
/// solved rotation vector; a rank-deficient system (e.g. a plane, which
/// leaves sliding free) gets the minimum-norm solution.
///
/// # Errors
/// [`GeomError::InvalidInput`] when the slices are empty or of different
/// lengths.
pub fn point_to_plane_step(p: &[DVec3], q: &[DVec3], n: &[DVec3]) -> Result<DAffine3> {
    if p.is_empty() || p.len() != q.len() || p.len() != n.len() {
        return Err(GeomError::InvalidInput(format!(
            "point-to-plane needs equal non-empty sets ({}, {}, {})",
            p.len(),
            q.len(),
            n.len()
        )));
    }
    let c = p.iter().sum::<DVec3>() / p.len() as f64;
    let mut ata = Matrix6::<f64>::zeros();
    let mut atb = Vector6::<f64>::zeros();
    for i in 0..p.len() {
        let rc = (p[i] - c).cross(n[i]);
        let row = [rc.x, rc.y, rc.z, n[i].x, n[i].y, n[i].z];
        let b = (q[i] - p[i]).dot(n[i]);
        for r in 0..6 {
            atb[r] += row[r] * b;
            for k in r..6 {
                ata[(r, k)] += row[r] * row[k];
            }
        }
    }
    for r in 0..6 {
        for k in 0..r {
            ata[(r, k)] = ata[(k, r)];
        }
    }
    let svd = ata.svd(true, true);
    let smax = svd.singular_values.max();
    let x = if smax > 0.0 {
        svd.solve(&atb, smax * PLANE_RCOND)
            .map_err(|e| GeomError::InvalidInput(e.into()))?
    } else {
        Vector6::zeros()
    };
    let rot = rotation_from_vector(DVec3::new(x[0], x[1], x[2]));
    let t = c + DVec3::new(x[3], x[4], x[5]) - rot * c;
    Ok(DAffine3::from_mat3_translation(rot, t))
}

/// Outcome of [`point_to_plane_solve`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PlaneSolve {
    /// The rigid motion (identity along the weak directions).
    pub delta: DAffine3,
    /// Smallest eigenvalue of the scaled normal matrix over its largest
    /// (0 for a rank-deficient system): how well the pairs constrain the
    /// six degrees of freedom.
    pub conditioning: f64,
    /// Directions left unchanged because they were too weakly constrained.
    pub weak: usize,
}

/// Weighted, degeneracy-aware variant of [`point_to_plane_step`].
///
/// The rotation unknowns are scaled by the RMS distance of the points from
/// their centroid, so the six eigenvalues of the normal matrix are
/// comparable (Gelfand et al. 2003, *Geometrically stable sampling*). The
/// system is solved in its eigenbasis and every direction whose
/// eigenvalue is below `min_relative_eigen` times the largest is left at
/// zero: those motions (sliding along a plane, turning about the axis of a
/// surface of revolution) keep the starting pose instead of following the
/// noise. `weights` (one per pair, `None`: all 1) are robust weights, e.g.
/// Tukey's biweight of the residual.
///
/// # Errors
/// [`GeomError::InvalidInput`] when the slices are empty or of different
/// lengths.
pub fn point_to_plane_solve(
    p: &[DVec3],
    q: &[DVec3],
    n: &[DVec3],
    weights: Option<&[f64]>,
    min_relative_eigen: f64,
) -> Result<PlaneSolve> {
    if p.is_empty()
        || p.len() != q.len()
        || p.len() != n.len()
        || weights.is_some_and(|w| w.len() != p.len())
    {
        return Err(GeomError::InvalidInput(format!(
            "point-to-plane needs equal non-empty sets ({}, {}, {})",
            p.len(),
            q.len(),
            n.len()
        )));
    }
    let w = |i: usize| weights.map_or(1.0, |w| w[i].max(0.0));
    let total: f64 = (0..p.len()).map(w).sum();
    if total <= 0.0 {
        return Ok(PlaneSolve {
            delta: DAffine3::IDENTITY,
            conditioning: 0.0,
            weak: 6,
        });
    }
    let c = (0..p.len()).map(|i| w(i) * p[i]).sum::<DVec3>() / total;
    let scale = ((0..p.len())
        .map(|i| w(i) * (p[i] - c).length_squared())
        .sum::<f64>()
        / total)
        .sqrt()
        .max(1e-9);
    let mut ata = Matrix6::<f64>::zeros();
    let mut atb = Vector6::<f64>::zeros();
    for i in 0..p.len() {
        let wi = w(i);
        if wi == 0.0 {
            continue;
        }
        let rc = (p[i] - c).cross(n[i]) / scale;
        let row = [rc.x, rc.y, rc.z, n[i].x, n[i].y, n[i].z];
        let b = (q[i] - p[i]).dot(n[i]);
        for r in 0..6 {
            atb[r] += wi * row[r] * b;
            for k in r..6 {
                ata[(r, k)] += wi * row[r] * row[k];
            }
        }
    }
    for r in 0..6 {
        for k in 0..r {
            ata[(r, k)] = ata[(k, r)];
        }
    }
    let eig = ata.symmetric_eigen();
    let lmax = eig.eigenvalues.max();
    let lmin = eig.eigenvalues.min();
    let mut x = Vector6::<f64>::zeros();
    let mut weak = 0;
    for k in 0..6 {
        let l = eig.eigenvalues[k];
        if lmax <= 0.0 || l < min_relative_eigen.max(PLANE_RCOND) * lmax {
            weak += 1;
            continue;
        }
        let v = eig.eigenvectors.column(k);
        x += v * (v.dot(&atb) / l);
    }
    let rot = rotation_from_vector(DVec3::new(x[0], x[1], x[2]) / scale);
    let t = c + DVec3::new(x[3], x[4], x[5]) - rot * c;
    Ok(PlaneSolve {
        delta: DAffine3::from_mat3_translation(rot, t),
        conditioning: if lmax > 0.0 {
            (lmin / lmax).max(0.0)
        } else {
            0.0
        },
        weak,
    })
}

/// Rotation of angle `|v|` about `v` (Rodrigues).
fn rotation_from_vector(v: DVec3) -> DMat3 {
    let angle = v.length();
    if angle > 0.0 {
        DMat3::from_axis_angle(v / angle, angle)
    } else {
        DMat3::IDENTITY
    }
}

/// Rotation angle of the linear part of `t` (radians).
pub fn rotation_angle(t: &DAffine3) -> f64 {
    let m = t.matrix3;
    let tr = m.x_axis.x + m.y_axis.y + m.z_axis.z;
    ((tr - 1.0) * 0.5).clamp(-1.0, 1.0).acos()
}

/// Registers the points `source` (with optional unit normals) onto `target`.
///
/// # Errors
/// [`GeomError::InvalidInput`] for an empty source, normals of the wrong
/// length or parameters out of range.
pub fn icp(
    source: &[DVec3],
    source_normals: Option<&[DVec3]>,
    target: &IcpTarget,
    params: &IcpParams,
) -> Result<IcpResult> {
    check_params(source, source_normals, params)?;
    let n_src = source.len();
    let cos_min = match (source_normals, params.max_normal_angle) {
        (Some(_), Some(a)) => Some(a.to_radians().cos()),
        _ => None,
    };
    let gate = if params.max_pair_distance.is_finite() {
        params.max_pair_distance as f32
    } else {
        f32::INFINITY
    };
    let mut t = params.initial;
    let mut out = IcpResult {
        transform: t,
        rms: f64::INFINITY,
        iterations: 0,
        overlap: 0.0,
        rms_history: Vec::new(),
        converged: false,
        lost: false,
    };
    let (mut last_rms, mut last_mse) = (f64::INFINITY, f64::INFINITY);
    let mut moved: Vec<DVec3> = Vec::with_capacity(n_src);
    let mut query: Vec<Vec3> = Vec::with_capacity(n_src);
    for _ in 0..params.max_iterations {
        source
            .par_iter()
            .map(|&p| t.transform_point3(p))
            .collect_into_vec(&mut moved);
        moved
            .par_iter()
            .map(|p| p.as_vec3())
            .collect_into_vec(&mut query);
        let hits = target.bvh.closest_points(&query, gate);
        let rot = t.matrix3;
        // pairs passing the gates: (source index, distance, closest point, normal)
        let mut ok: Vec<(u32, f64, DVec3, DVec3)> = hits
            .par_iter()
            .enumerate()
            .filter_map(|(i, h)| {
                let h = h.as_ref()?;
                let nq = target.normals[h.face as usize];
                if let (Some(c), Some(ns)) = (cos_min, source_normals)
                    && (rot * ns[i]).dot(nq) < c
                {
                    return None;
                }
                let q = h.point.as_dvec3();
                Some((i as u32, moved[i].distance(q), q, nq))
            })
            .collect();
        if ok.len() < params.min_pairs {
            out.lost = true;
            break;
        }
        trim_pairs(&mut ok, n_src, params.trim);
        let p: Vec<DVec3> = ok.iter().map(|e| moved[e.0 as usize]).collect();
        let q: Vec<DVec3> = ok.iter().map(|e| e.2).collect();
        let mse = ok.iter().map(|e| e.1 * e.1).sum::<f64>() / ok.len() as f64;
        let (step, rms) = match params.metric {
            IcpMetric::PointToPlane => {
                let nq: Vec<DVec3> = ok.iter().map(|e| e.3).collect();
                let b2 = p
                    .iter()
                    .zip(&q)
                    .zip(&nq)
                    .map(|((p, q), n)| (*q - *p).dot(*n).powi(2))
                    .sum::<f64>();
                (
                    point_to_plane_step(&p, &q, &nq)?,
                    (b2 / p.len() as f64).sqrt(),
                )
            }
            IcpMetric::PointToPoint => (kabsch(&p, &q, None, false)?, mse.sqrt()),
        };
        t = step * t;
        out.iterations += 1;
        out.rms = rms;
        out.rms_history.push(rms);
        out.overlap = ok.len() as f64 / n_src as f64;
        out.transform = t;
        let small = (params.min_step_rotation > 0.0 || params.min_step_translation > 0.0)
            && rotation_angle(&step) < params.min_step_rotation
            && step.translation.length() < params.min_step_translation;
        if small
            || (last_rms - rms).abs() < params.tolerance
            || (last_mse - mse).abs() < params.mse_tolerance
        {
            out.converged = true;
            break;
        }
        last_rms = rms;
        last_mse = mse;
    }
    Ok(out)
}

fn check_params(
    source: &[DVec3],
    source_normals: Option<&[DVec3]>,
    params: &IcpParams,
) -> Result<()> {
    if source.is_empty() {
        return Err(GeomError::InvalidInput("source has no points".into()));
    }
    if let Some(n) = source_normals
        && n.len() != source.len()
    {
        return Err(GeomError::InvalidInput(format!(
            "{} normals for {} source points",
            n.len(),
            source.len()
        )));
    }
    match params.trim {
        Trim::Fraction(f) if !(f > 0.0 && f <= 1.0) => {
            return Err(GeomError::InvalidInput(format!(
                "trim fraction {f} not in (0, 1]"
            )));
        }
        Trim::Auto { xi_min } if !(xi_min > 0.0 && xi_min <= 1.0) => {
            return Err(GeomError::InvalidInput(format!(
                "xi_min {xi_min} not in (0, 1]"
            )));
        }
        _ => {}
    }
    if params.max_pair_distance.is_nan() || params.max_pair_distance < 0.0 {
        return Err(GeomError::InvalidInput(
            "max_pair_distance must be >= 0".into(),
        ));
    }
    Ok(())
}

/// Applies `trim` to the gated pairs (kept in source order, or in distance
/// order for [`Trim::Auto`]); `n_total` is the number of source points.
fn trim_pairs(ok: &mut Vec<(u32, f64, DVec3, DVec3)>, n_total: usize, trim: Trim) {
    match trim {
        Trim::None => {}
        Trim::Fraction(f) => {
            let mut d: Vec<f64> = ok.iter().map(|e| e.1).collect();
            d.sort_unstable_by(f64::total_cmp);
            let h = (d.len() - 1) as f64 * f;
            let lo = h.floor() as usize;
            let hi = (lo + 1).min(d.len() - 1);
            let q = d[lo] + (h - lo as f64) * (d[hi] - d[lo]);
            ok.retain(|e| e.1 <= q);
        }
        Trim::Auto { xi_min } => {
            // stable order: distance, then source index
            ok.sort_unstable_by(|a, b| a.1.total_cmp(&b.1).then(a.0.cmp(&b.0)));
            let n = n_total as f64;
            let xi_last = ok.len() as f64 / n;
            let floor = xi_min.min(xi_last);
            let (mut cs, mut best, mut best_k) = (0.0, f64::INFINITY, ok.len());
            for (k, e) in ok.iter().enumerate() {
                cs += e.1 * e.1;
                let k1 = (k + 1) as f64;
                let xi = k1 / n;
                if xi >= floor {
                    let psi = (cs / k1) / xi.powi(3);
                    if psi < best {
                        best = psi;
                        best_k = k + 1;
                    }
                }
            }
            ok.truncate(best_k);
        }
    }
}

/// Registers the vertices of `source` onto the surface of `target` with the
/// metric of `params`. Source normals are the mesh normals when present,
/// otherwise the area-weighted vertex normals; a source without normals and
/// triangles (a point cloud) is registered without the normal gate.
///
/// # Errors
/// As [`icp`] and [`IcpTarget::new`].
pub fn register(source: &TriMesh, target: &TriMesh, params: &IcpParams) -> Result<IcpResult> {
    let tgt = IcpTarget::new(target)?;
    let pts: Vec<DVec3> = source.positions.iter().map(|p| p.as_dvec3()).collect();
    let normals: Option<Vec<DVec3>> = match &source.normals {
        Some(n) => Some(n.iter().map(|n| n.as_dvec3().normalize_or_zero()).collect()),
        None if source.tris.is_empty() => None,
        None => Some(
            source
                .compute_vertex_normals()
                .iter()
                .map(|n| n.as_dvec3().normalize_or_zero())
                .collect(),
        ),
    };
    icp(&pts, normals.as_deref(), &tgt, params)
}

/// Registers `source` onto `target` with point-to-plane ICP (the metric of
/// `params` is overridden), see [`register`].
///
/// # Errors
/// As [`register`].
pub fn point_to_plane(source: &TriMesh, target: &TriMesh, params: &IcpParams) -> Result<IcpResult> {
    let params = IcpParams {
        metric: IcpMetric::PointToPlane,
        ..params.clone()
    };
    register(source, target, &params)
}

#[cfg(test)]
mod tests;
