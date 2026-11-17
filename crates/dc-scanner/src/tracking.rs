//! Frame-to-model camera tracking.
//!
//! Every new depth frame is registered onto the fused model with projective
//! point-to-plane ICP (Newcombe et al. 2011, *KinectFusion*): the model is
//! ray cast from the previous camera pose ([`TsdfVolume::raycast`], at a
//! reduced resolution), each frame point is projected into that synthetic
//! view to find its partner, and the linearised point-to-plane system is
//! solved with [`dc_geom::icp::point_to_plane_step`]. The caller supplies
//! the initial guess (constant-velocity prediction).
//!
//! The frame is first sampled with [`sample_frame`]: a regular grid over
//! the image whose step is chosen from the number of *valid* pixels, so a
//! frame that sees the object in a corner still gives about
//! [`TrackParams::target_points`] points. Pixels at depth edges (a
//! neighbour missing or much nearer / farther) and isolated speckles are
//! left out, and every sample carries a normal from its neighbours.
//!
//! A pair is kept when the point-to-plane distance is within a shrinking
//! gate, the points are within that gate plus the size of one model pixel
//! (the model is ray cast at a coarser resolution than the frame), and the
//! frame and model normals agree within [`TrackParams::max_normal_deg`]. A
//! frame is accepted when enough of its samples pair up, the plane residual
//! is small and the motion since the previous frame is plausible; otherwise
//! it counts as lost and is not fused.

use dc_geom::icp::{point_to_plane_step, rotation_angle};
use dc_geom::tsdf::{CameraIntrinsics, TsdfVolume};
use glam::{Affine3A, DAffine3, DVec3, Vec3};
use rayon::prelude::*;

/// Tracking parameters.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TrackParams {
    /// Association gates of the coarse-to-fine stages (mm).
    pub gates_mm: [f32; 3],
    /// Iterations per stage at most.
    pub iterations_per_stage: usize,
    /// Fewest pairs for a valid solve.
    pub min_inliers: usize,
    /// Fewest pairs as a fraction of the frame samples for an accepted frame.
    pub min_inlier_fraction: f32,
    /// Largest accepted RMS point-to-plane residual (mm).
    pub max_rms_mm: f32,
    /// Largest accepted camera translation between two frames (mm).
    pub max_step_mm: f32,
    /// Largest accepted camera rotation between two frames (degrees).
    pub max_step_deg: f32,
    /// The model is ray cast at most this many pixels wide.
    pub max_model_width: u32,
    /// Frame samples used for tracking, about (counted on valid pixels).
    pub target_points: usize,
    /// Largest depth step to a neighbour pixel, in pixel footprints
    /// (`depth / fx`) per pixel, for a sample not to be at a depth edge;
    /// 4 rejects surfaces steeper than about 76 degrees to the view.
    pub max_slope: f32,
    /// Largest angle between the frame and model normals of a pair (degrees).
    pub max_normal_deg: f32,
}

impl Default for TrackParams {
    fn default() -> Self {
        Self {
            gates_mm: [3.0, 1.2, 0.5],
            iterations_per_stage: 6,
            min_inliers: 200,
            min_inlier_fraction: 0.25,
            max_rms_mm: 0.2,
            max_step_mm: 20.0,
            max_step_deg: 20.0,
            max_model_width: 220,
            target_points: 8000,
            max_slope: 4.0,
            max_normal_deg: 30.0,
        }
    }
}

impl TrackParams {
    /// Pixel stride that samples about [`TrackParams::target_points`] of
    /// `valid_pixels` valid pixels.
    pub fn stride(&self, valid_pixels: usize) -> u32 {
        (valid_pixels as f32 / self.target_points.max(1) as f32)
            .sqrt()
            .ceil()
            .max(1.0) as u32
    }
}

/// The tracking samples of a depth frame (camera space, mm).
#[derive(Debug, Clone, Default, PartialEq)]
pub struct FrameSample {
    /// Sample points.
    pub points: Vec<Vec3>,
    /// Unit normal of each point, facing the camera.
    pub normals: Vec<Vec3>,
}

/// Offset (pixels) of the neighbours used for the edge test and normals.
const NEIGHBOUR: usize = 2;

fn valid(d: f32) -> bool {
    d.is_finite() && d > 0.0
}

/// Samples a depth image for tracking: a grid of
/// [`TrackParams::stride`] over the valid pixels, without the pixels at
/// depth edges (see [`TrackParams::max_slope`]), each with a normal.
pub fn sample_frame(
    depth_mm: &[f32],
    intr: &CameraIntrinsics,
    params: &TrackParams,
) -> FrameSample {
    let w = intr.width as usize;
    let h = (intr.height as usize).min(depth_mm.len().checked_div(w).unwrap_or(0));
    let valid_pixels = depth_mm[..w * h].iter().filter(|&&d| valid(d)).count();
    let stride = params.stride(valid_pixels) as usize;
    let r = NEIGHBOUR;
    let rows: Vec<usize> = (r..h.saturating_sub(r)).step_by(stride).collect();
    let per_row: Vec<(Vec<Vec3>, Vec<Vec3>)> = rows
        .par_iter()
        .map(|&v| {
            let mut points = Vec::new();
            let mut normals = Vec::new();
            for u in (r..w.saturating_sub(r)).step_by(stride) {
                let d = depth_mm[v * w + u];
                if !valid(d) {
                    continue;
                }
                let tol = params.max_slope * r as f32 * d / intr.fx;
                let at = |du: isize, dv: isize| {
                    let (uu, vv) = (u.wrapping_add_signed(du), v.wrapping_add_signed(dv));
                    let dn = depth_mm[vv * w + uu];
                    (valid(dn) && (dn - d).abs() <= tol)
                        .then(|| intr.ray(uu as f32, vv as f32) * dn)
                };
                let r = r as isize;
                let (Some(left), Some(right), Some(up), Some(down)) =
                    (at(-r, 0), at(r, 0), at(0, -r), at(0, r))
                else {
                    continue;
                };
                let p = intr.ray(u as f32, v as f32) * d;
                let mut n = (down - up).cross(right - left).normalize_or_zero();
                if n == Vec3::ZERO {
                    continue;
                }
                if n.dot(p) > 0.0 {
                    n = -n;
                }
                points.push(p);
                normals.push(n);
            }
            (points, normals)
        })
        .collect();
    let mut sample = FrameSample::default();
    for (p, n) in per_row {
        sample.points.extend(p);
        sample.normals.extend(n);
    }
    sample
}

/// Outcome of [`track_frame`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TrackResult {
    /// Estimated camera-to-world pose of the frame.
    pub pose: Affine3A,
    /// RMS point-to-plane residual of the final pairs (mm).
    pub rms_mm: f32,
    /// Number of final pairs.
    pub inliers: usize,
    /// Solves performed.
    pub iterations: usize,
    /// The frame passed every acceptance test.
    pub ok: bool,
}

/// The model as seen from one camera: per-pixel world points and normals.
struct ModelView {
    intr: CameraIntrinsics,
    world_to_cam: DAffine3,
    points: Vec<Vec3>,
    normals: Vec<Vec3>,
    /// Cosine of [`TrackParams::max_normal_deg`].
    min_normal_cos: f64,
}

impl ModelView {
    fn new(
        volume: &TsdfVolume,
        intr: &CameraIntrinsics,
        cam_to_world: &Affine3A,
        params: &TrackParams,
    ) -> Self {
        let factor = intr.width.div_ceil(params.max_model_width.max(1));
        let intr = intr.downscaled(factor);
        let (points, normals) = volume.raycast(&intr, cam_to_world);
        Self {
            intr,
            world_to_cam: to_d(cam_to_world).inverse(),
            points,
            normals,
            min_normal_cos: (params.max_normal_deg as f64).to_radians().cos(),
        }
    }

    /// Partner of the world point `p` with world normal `pn`: the model
    /// point and normal under its projection, when the pair passes the
    /// gates (see the module doc).
    fn partner(&self, p: DVec3, pn: DVec3, gate: f64) -> Option<(DVec3, DVec3)> {
        let c = self.world_to_cam.transform_point3(p);
        if c.z <= 1e-6 {
            return None;
        }
        let k = &self.intr;
        let u = (k.fx as f64 * c.x / c.z + k.cx as f64).round();
        let v = (k.fy as f64 * c.y / c.z + k.cy as f64).round();
        if u < 0.0 || v < 0.0 || u >= k.width as f64 || v >= k.height as f64 {
            return None;
        }
        let i = v as usize * k.width as usize + u as usize;
        let (q, n) = (self.points[i], self.normals[i]);
        if !q.is_finite() || !n.is_finite() || n == Vec3::ZERO {
            return None;
        }
        let (q, n) = (q.as_dvec3(), n.as_dvec3().normalize());
        // the partner is the model point of the nearest coarse pixel: up to
        // one model pixel away along the surface
        let lateral = c.z / k.fx as f64;
        let ok = (p - q).dot(n).abs() <= gate
            && q.distance(p) <= gate + lateral
            && pn.dot(n) >= self.min_normal_cos;
        ok.then_some((q, n))
    }

    /// Pairs of the frame samples placed by `pose`.
    fn pairs(&self, frame: &FrameSample, pose: &DAffine3, gate: f64) -> Vec<(DVec3, DVec3, DVec3)> {
        frame
            .points
            .par_iter()
            .zip(frame.normals.par_iter())
            .with_min_len(512)
            .filter_map(|(p, n)| {
                let w = pose.transform_point3(p.as_dvec3());
                let wn = pose.transform_vector3(n.as_dvec3());
                self.partner(w, wn, gate).map(|(q, n)| (w, q, n))
            })
            .collect()
    }
}

fn to_d(a: &Affine3A) -> DAffine3 {
    DAffine3::from_mat3_translation(a.matrix3.as_dmat3(), a.translation.as_dvec3())
}

fn to_f(a: &DAffine3) -> Affine3A {
    Affine3A::from_mat3_translation(a.matrix3.as_mat3(), a.translation.as_vec3())
}

/// Registers the frame samples `frame` (camera space, mm, from
/// [`sample_frame`]) onto the model in `volume`, ray cast from `model_pose`
/// with the frame intrinsics `intr`, starting from the camera-to-world
/// guess `initial`.
pub fn track_frame(
    volume: &TsdfVolume,
    frame: &FrameSample,
    intr: &CameraIntrinsics,
    model_pose: &Affine3A,
    initial: &Affine3A,
    params: &TrackParams,
) -> TrackResult {
    let view = ModelView::new(volume, intr, model_pose, params);
    let mut pose = to_d(initial);
    let mut iterations = 0;
    let failed = |pose: &DAffine3, iterations| TrackResult {
        pose: to_f(pose),
        rms_mm: f32::INFINITY,
        inliers: 0,
        iterations,
        ok: false,
    };
    for &gate in &params.gates_mm {
        for _ in 0..params.iterations_per_stage {
            let pairs = view.pairs(frame, &pose, gate as f64);
            if pairs.len() < params.min_inliers.max(dc_geom::icp::MIN_PAIRS) {
                return failed(&pose, iterations);
            }
            let (p, (q, n)): (Vec<DVec3>, (Vec<DVec3>, Vec<DVec3>)) =
                pairs.into_iter().map(|(p, q, n)| (p, (q, n))).unzip();
            let Ok(delta) = point_to_plane_step(&p, &q, &n) else {
                return failed(&pose, iterations);
            };
            pose = delta * pose;
            iterations += 1;
            let moved = delta.translation.length();
            if rotation_angle(&delta) < 1e-5 && moved < 1e-4 {
                break;
            }
        }
    }
    let gate = params.gates_mm[2] as f64;
    let pairs = view.pairs(frame, &pose, gate);
    let inliers = pairs.len();
    let rms = if inliers == 0 {
        f64::INFINITY
    } else {
        (pairs
            .iter()
            .map(|(p, q, n)| (p - q).dot(*n).powi(2))
            .sum::<f64>()
            / inliers as f64)
            .sqrt()
    };
    let step = to_d(model_pose).inverse() * pose;
    let ok = inliers >= params.min_inliers
        && inliers as f32 >= params.min_inlier_fraction * frame.points.len() as f32
        && rms <= params.max_rms_mm as f64
        && step.translation.length() <= params.max_step_mm as f64
        && rotation_angle(&step).to_degrees() <= params.max_step_deg as f64;
    TrackResult {
        pose: to_f(&pose),
        rms_mm: rms as f32,
        inliers,
        iterations,
        ok,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ScannerBackend;
    use crate::sim::{SimBackend, SimDevice, SimOptions, UPPER_ID, typodont_arch};
    use dc_geom::tsdf::TsdfParams;

    #[test]
    fn recovers_a_perturbed_pose() {
        let options = SimOptions {
            realtime: false,
            width: 320,
            height: 200,
            ..SimOptions::default()
        };
        let mesh = typodont_arch(true);
        let info = SimBackend::default().list().unwrap()[0].clone();
        assert_eq!(info.id, UPPER_ID);
        let dev = SimDevice::new(info, &mesh, options).unwrap();
        let (f0, f1) = (dev.render(0), dev.render(3));
        let (p0, p1) = (dev.pose(0), dev.pose(3));
        let mut volume = TsdfVolume::new(TsdfParams {
            voxel_mm: 0.4,
            trunc_mm: 1.6,
            max_weight: 64.0,
        });
        // world = first camera
        volume.integrate(&f0.depth_mm, &f0.intrinsics, &Affine3A::IDENTITY);
        let truth = p0.inverse() * p1;
        let params = TrackParams::default();
        let sample = sample_frame(&f1.depth_mm, &f1.intrinsics, &params);
        let r = track_frame(
            &volume,
            &sample,
            &f1.intrinsics,
            &Affine3A::IDENTITY,
            &Affine3A::IDENTITY,
            &params,
        );
        assert!(r.ok, "{r:?}");
        // displacement of points over the model (about 130 mm in front)
        let err = to_d(&truth).inverse() * to_d(&r.pose);
        for p in [
            DVec3::new(0.0, 0.0, 130.0),
            DVec3::new(25.0, 20.0, 130.0),
            DVec3::new(-25.0, -20.0, 125.0),
        ] {
            let moved = err.transform_point3(p).distance(p);
            assert!(moved < 0.05, "{moved} mm at {p}");
        }
    }

    #[test]
    fn sampling_follows_the_valid_pixels_and_skips_edges() {
        let intr = CameraIntrinsics {
            width: 400,
            height: 300,
            fx: 600.0,
            fy: 600.0,
            cx: 199.5,
            cy: 149.5,
        };
        let params = TrackParams {
            target_points: 1000,
            ..TrackParams::default()
        };
        // a plane at 200 mm over the left quarter only, with a 5 mm step
        // (a depth edge) at column 50 and one speckle
        let mut depth = vec![0.0_f32; 400 * 300];
        for v in 0..300 {
            for u in 0..100 {
                depth[v * 400 + u] = if u < 50 { 200.0 } else { 205.0 };
            }
        }
        depth[150 * 400 + 300] = 150.0;
        let s = sample_frame(&depth, &intr, &params);
        // 30 000 valid pixels -> stride 6 -> about 830 samples, not the
        // ~130 a stride over the whole image would give
        assert_eq!(params.stride(30_000), 6);
        assert!((600..1000).contains(&s.points.len()), "{}", s.points.len());
        assert_eq!(s.points.len(), s.normals.len());
        for (p, n) in s.points.iter().zip(&s.normals) {
            // no sample within 2 pixels before the step, none on the speckle
            let u = (p.x / p.z * intr.fx + intr.cx).round();
            assert!(!(48.0..=51.0).contains(&u), "{p}");
            assert!(p.z < 206.0 && p.z > 199.0);
            // the plane faces the camera
            assert!(n.dot(Vec3::NEG_Z) > 0.999, "{n}");
        }
        // nothing valid: no sample
        assert!(
            sample_frame(&vec![0.0; 400 * 300], &intr, &params)
                .points
                .is_empty()
        );
    }
}
