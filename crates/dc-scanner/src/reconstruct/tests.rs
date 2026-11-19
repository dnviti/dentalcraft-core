use super::*;
use glam::{Mat3, Vec3};

const W: u32 = 160;
const H: u32 = 120;

fn intr() -> CameraIntrinsics {
    CameraIntrinsics {
        width: W,
        height: H,
        fx: 190.0,
        fy: 190.0,
        cx: (W as f32 - 1.0) * 0.5,
        cy: (H as f32 - 1.0) * 0.5,
    }
}

/// The scene: two spheres (centre, radius) and a box (min, max corners)
/// standing on the platter, off the turntable axis (turns about the axis
/// are observable, planes and edges constrain the registration).
const SPHERES: [(Vec3, f32); 2] = [
    (Vec3::new(0.0, 0.0, 6.0), 6.0),
    (Vec3::new(9.0, 3.0, 4.0), 4.0),
];
const BOX: (Vec3, Vec3) = (Vec3::new(-11.0, -7.0, 0.0), Vec3::new(-4.0, 1.0, 5.0));

/// Signed distance to the scene surface.
fn sdf(p: Vec3) -> f32 {
    let (lo, hi) = BOX;
    let c = (lo + hi) * 0.5;
    let q = (p - c).abs() - (hi - lo) * 0.5;
    let boxed = q.max(Vec3::ZERO).length() + q.max_element().min(0.0);
    SPHERES
        .iter()
        .map(|(c, r)| p.distance(*c) - r)
        .fold(boxed, f32::min)
}

/// Camera at `eye` looking at `target` (+Z forward, +Y down), world +Z up.
fn look_at(eye: Vec3, target: Vec3) -> Affine3A {
    let z = (target - eye).normalize();
    let x = Vec3::NEG_Z.cross(z).normalize();
    let y = z.cross(x);
    Affine3A::from_mat3_translation(Mat3::from_cols(x, y, z), eye)
}

/// Depth image of the scene from the camera-to-world pose `pose`.
fn render(k: &CameraIntrinsics, pose: &Affine3A) -> Vec<f32> {
    let o = pose.transform_point3(Vec3::ZERO);
    let mut depth = vec![f32::NAN; k.pixel_count()];
    for v in 0..k.height {
        for u in 0..k.width {
            let ray = k.ray(u as f32, v as f32);
            let d = pose.transform_vector3(ray);
            let mut best = f32::INFINITY;
            for (c, r) in SPHERES {
                let oc = o - c;
                let a = d.length_squared();
                let b = d.dot(oc);
                let disc = b * b - a * (oc.length_squared() - r * r);
                if disc >= 0.0 {
                    let t = (-b - disc.sqrt()) / a;
                    if t > 0.0 {
                        best = best.min(t);
                    }
                }
            }
            // slabs of the box
            let inv = d.recip();
            let (ta, tb) = ((BOX.0 - o) * inv, (BOX.1 - o) * inv);
            let (t0, t1) = (ta.min(tb).max_element(), ta.max(tb).min_element());
            if t0 <= t1 && t0 > 0.0 {
                best = best.min(t0);
            }
            if best.is_finite() {
                // t along `ray` (z = 1 in the camera): the depth
                depth[(v * k.width + u) as usize] = best;
            }
        }
    }
    depth
}

/// Stops every `step` degrees of one revolution at tilt 0, the camera 70
/// mm away and 30 degrees above the platter; poses perturbed by `jitter`
/// (degrees about the axis, mm along X of the world) alternating in sign.
fn scan(step: f32, jitter: (f32, f32)) -> (KeptStops, Vec<Affine3A>) {
    let k = intr();
    let elev = 30f32.to_radians();
    let cam0 = look_at(
        Vec3::new(70.0 * elev.cos(), 0.0, 70.0 * elev.sin()),
        Vec3::new(0.0, 0.0, 5.0),
    );
    let world = cam0;
    let to_cam = cam0.inverse();
    let calibration = RigCalibration {
        axis_point: to_cam.transform_point3(Vec3::ZERO).into(),
        axis_dir: to_cam.transform_vector3(Vec3::Z).into(),
        platter_up: to_cam.transform_vector3(Vec3::Z).into(),
        tilt_axis_point: None,
        tilt_axis_dir: None,
        scanner_serial: None,
        turntable_id: "test".into(),
        created: "2026-11-19T00:00:00Z".into(),
        fit_rms_deg: 0.0,
        fit_rms_mm: 0.0,
        angle_gain: 1.0,
        fit_pairs: 0,
        imu_up: None,
    };
    let mut stops = Vec::new();
    let mut truth = Vec::new();
    let n = (360.0 / step).round() as usize;
    for i in 0..n {
        let theta = i as f32 * step;
        // the object turns by theta: the camera turns by -theta around it
        let cam_world = Affine3A::from_rotation_z((-theta).to_radians()) * cam0;
        let depth = render(&k, &cam_world);
        let pose = world.inverse() * cam_world;
        let sign = if i % 2 == 0 { 1.0 } else { -1.0 };
        let perturbed_world = Affine3A::from_translation(Vec3::new(sign * jitter.1, 0.0, 0.0))
            * Affine3A::from_rotation_z((sign * jitter.0).to_radians())
            * cam_world;
        let perturbed = if i == 0 {
            pose
        } else {
            world.inverse() * perturbed_world
        };
        stops.push(KeptStop {
            pixels: Arc::new(
                depth
                    .iter()
                    .enumerate()
                    .filter(|(_, d)| d.is_finite())
                    .map(|(j, d)| (j as u32, *d))
                    .collect(),
            ),
            intrinsics: k,
            angles: (theta, 0.0),
            pose: perturbed,
            flagged: false,
        });
        truth.push(pose);
    }
    (
        KeptStops {
            stops,
            world,
            world_angles: Some((0.0, 0.0)),
            calibration: Some(calibration),
            roi: Some((40.0, 30.0)),
        },
        truth,
    )
}

/// Mean and largest distance of the mesh vertices to the scene surface.
fn error(mesh: &TriMesh) -> (f32, f32) {
    let d: Vec<f32> = mesh.positions.iter().map(|p| sdf(*p).abs()).collect();
    (
        d.iter().sum::<f32>() / d.len().max(1) as f32,
        d.iter().copied().fold(0.0, f32::max),
    )
}

#[test]
fn exact_stops_reconstruct_within_the_voxel() {
    let (stops, _) = scan(45.0, (0.0, 0.0));
    let params = ReconstructParams {
        resolution_mm: 0.25,
        refine: false,
        ..ReconstructParams::default()
    };
    let r = reconstruct(&stops, &params, &|_| {}, &|| false).unwrap();
    assert!(r.mesh.face_count() > 2000, "{:?}", r.report);
    let (mean, max) = error(&r.mesh);
    assert!(mean < 0.06, "mean {mean}");
    assert!(max < 0.5, "max {max}");
    assert_eq!(r.report.stops, 8);
    assert_eq!(r.report.refined, 0);
    assert!(r.report.mean_rms_mm.is_none());
    assert_eq!(r.report.faces, r.mesh.face_count());
    // the mesh is in the world frame: nothing under the platter but the
    // region-of-interest margin, faces look outward
    assert!(
        r.mesh
            .positions
            .iter()
            .all(|p| p.z > -ROI_BELOW_PLATTER_MM - 0.5)
    );
    let outward = (0..r.mesh.face_count())
        .filter(|&f| {
            let [a, b, c] = r.mesh.triangle(f);
            let p = (a + b + c) / 3.0;
            let e = 1e-2;
            let grad = Vec3::new(
                sdf(p + Vec3::X * e) - sdf(p - Vec3::X * e),
                sdf(p + Vec3::Y * e) - sdf(p - Vec3::Y * e),
                sdf(p + Vec3::Z * e) - sdf(p - Vec3::Z * e),
            );
            r.mesh.face_normal(f).dot(grad) > 0.0
        })
        .count();
    assert!(outward * 100 > 98 * r.mesh.face_count());
}

#[test]
fn refinement_undoes_turntable_errors_and_never_worsens() {
    // every other stop off by 0.6 degrees about the axis and 0.3 mm
    let (stops, truth) = scan(15.0, (0.6, 0.3));
    let params = ReconstructParams {
        resolution_mm: 0.25,
        refine: false,
        ..ReconstructParams::default()
    };
    let raw = reconstruct(&stops, &params, &|_| {}, &|| false).unwrap();
    let refined = reconstruct(
        &stops,
        &ReconstructParams {
            refine: true,
            ..params
        },
        &|_| {},
        &|| false,
    )
    .unwrap();
    let (raw_mean, _) = error(&raw.mesh);
    let (ref_mean, _) = error(&refined.mesh);
    let rep = &refined.report;
    assert!(rep.refined >= 3, "{rep:?}");
    let (before, after) = (rep.mean_rms_before_mm.unwrap(), rep.mean_rms_mm.unwrap());
    assert!(after < before, "{rep:?}");
    assert!(
        ref_mean < 0.7 * raw_mean,
        "raw {raw_mean} refined {ref_mean}"
    );
    // the refined poses are nearer the truth than the perturbed ones
    let off = |a: &Affine3A, b: &Affine3A| {
        let c = (a.inverse() * *b).to_scale_rotation_translation();
        c.1.angle_between(glam::Quat::IDENTITY).to_degrees()
    };
    let before: f32 = stops
        .stops
        .iter()
        .zip(&truth)
        .map(|(s, t)| off(&s.pose, t))
        .sum();
    let after: f32 = refined
        .poses
        .iter()
        .zip(&truth)
        .map(|(p, t)| off(p, t))
        .sum();
    assert!(after < 0.5 * before, "pose error {before} -> {after} deg");
}

#[test]
fn stops_round_trip_through_a_directory() {
    let (stops, _) = scan(90.0, (0.2, 0.1));
    let dir = tempfile::tempdir().unwrap();
    assert_eq!(write_stops(dir.path(), &stops).unwrap(), 4);
    let back = read_stops(dir.path()).unwrap();
    assert_eq!(back, stops);
    assert!(dir.path().join(STOPS_FILE).is_file());
    // no stops: nothing to write; a missing directory: invalid parameters
    let empty = KeptStops {
        stops: Vec::new(),
        ..stops.clone()
    };
    assert_eq!(
        write_stops(dir.path(), &empty).unwrap_err().code(),
        "scan-no-stops"
    );
    assert_eq!(
        read_stops(&dir.path().join("nope")).unwrap_err().code(),
        "invalid-params"
    );
}

#[test]
fn bad_parameters_and_cancel() {
    let (stops, _) = scan(90.0, (0.0, 0.0));
    let bad = ReconstructParams {
        resolution_mm: 0.01,
        ..ReconstructParams::default()
    };
    assert_eq!(
        reconstruct(&stops, &bad, &|_| {}, &|| false)
            .unwrap_err()
            .code(),
        "invalid-params"
    );
    let none = KeptStops {
        stops: Vec::new(),
        ..stops.clone()
    };
    assert_eq!(
        reconstruct(&none, &ReconstructParams::default(), &|_| {}, &|| false)
            .unwrap_err()
            .code(),
        "scan-no-stops"
    );
    assert_eq!(
        reconstruct(&stops, &ReconstructParams::default(), &|_| {}, &|| true)
            .unwrap_err()
            .code(),
        "cancelled"
    );
}

/// Height of the simulated tilt axis under the platter (mm).
const PIVOT_MM: f32 = 40.0;

/// Stops every `step` degrees of one revolution at each tilt of `tilts`
/// (the first stop, at rotation 0 and the first tilt, is the world: give
/// tilt 0 first), with the poses of a calibrated dual-axis turntable.
/// `slip(tilt, k)` places the object on the platter at stop `k` of the pass
/// at `tilt` (identity: where it was at the first stop); the poses do not
/// know about it, like the turntable poses of an object that moved.
fn passes_scan(tilts: &[f32], step: f32, slip: &dyn Fn(f32, usize) -> Affine3A) -> KeptStops {
    let k = intr();
    let elev = 30f32.to_radians();
    let cam0 = look_at(
        Vec3::new(70.0 * elev.cos(), 0.0, 70.0 * elev.sin()),
        Vec3::new(0.0, 0.0, 5.0),
    );
    let to_cam = cam0.inverse();
    let pivot = Vec3::new(0.0, 0.0, -PIVOT_MM);
    let calibration = RigCalibration {
        axis_point: to_cam.transform_point3(Vec3::ZERO).into(),
        axis_dir: to_cam.transform_vector3(Vec3::Z).into(),
        platter_up: to_cam.transform_vector3(Vec3::Z).into(),
        tilt_axis_point: Some(to_cam.transform_point3(pivot).into()),
        tilt_axis_dir: Some(to_cam.transform_vector3(Vec3::X).into()),
        scanner_serial: None,
        turntable_id: "test".into(),
        created: "2026-11-19T00:00:00Z".into(),
        fit_rms_deg: 0.0,
        fit_rms_mm: 0.0,
        angle_gain: 1.0,
        fit_pairs: 0,
        imu_up: None,
    };
    // the platter at the angles, in the scene
    let platter = |r: f32, t: f32| {
        Affine3A::from_translation(pivot)
            * Affine3A::from_rotation_x(t.to_radians())
            * Affine3A::from_translation(-pivot)
            * Affine3A::from_rotation_z(r.to_radians())
    };
    let n = (360.0 / step).round() as usize;
    let mut stops = Vec::new();
    for &t in tilts {
        for i in 0..n {
            let r = i as f32 * step;
            // the camera seen from the object, as the turntable says ...
            let told = platter(r, t).inverse() * cam0;
            // ... and as it was, the object moved on the platter
            let depth = render(&k, &(slip(t, i).inverse() * told));
            stops.push(KeptStop {
                pixels: Arc::new(
                    depth
                        .iter()
                        .enumerate()
                        .filter(|(_, d)| d.is_finite())
                        .map(|(j, d)| (j as u32, *d))
                        .collect(),
                ),
                intrinsics: k,
                angles: (r, t),
                pose: to_cam * told,
                flagged: false,
            });
        }
    }
    KeptStops {
        stops,
        world: cam0,
        world_angles: Some((0.0, 0.0)),
        calibration: Some(calibration),
        roi: Some((40.0, 30.0)),
    }
}

fn reconstruct_at(stops: &KeptStops, refine: bool) -> Reconstruction {
    let params = ReconstructParams {
        resolution_mm: 0.25,
        refine,
        ..ReconstructParams::default()
    };
    reconstruct(stops, &params, &|_| {}, &|| false).unwrap()
}

#[test]
fn a_pass_that_slipped_as_one_body_is_aligned() {
    // the object slipped 1.2 mm and turned 1.5 degrees while the table
    // tilted to +15: beyond the live stop limits, within a pass alignment
    let slip = Affine3A::from_translation(Vec3::new(1.0, -0.6, 0.3))
        * Affine3A::from_rotation_z(1.5f32.to_radians());
    let stops = passes_scan(&[0.0, 15.0, -15.0], 30.0, &|t, _| {
        if t > 0.0 { slip } else { Affine3A::IDENTITY }
    });
    let r = reconstruct_at(&stops, true);
    let rep = &r.report;
    let tilted = rep.pass_details.iter().find(|p| p.tilt_deg > 0.0).unwrap();
    assert!(tilted.corrected_mm > 0.5, "{rep:?}");
    assert_eq!(rep.passes, 3);
    assert_eq!(rep.rejected_stops, 0, "{rep:?}");
    assert_eq!(rep.stops, 36);
    assert!(rep.warning.is_none());
    let (mean, _) = error(&r.mesh);
    assert!(mean < 0.06, "mean {mean} {rep:?}");
}

#[test]
fn views_of_an_object_that_moved_are_left_out() {
    // the object turned 15 degrees on the platter before the -15 pass, and
    // 12 more degrees during the last three stops of the +15 pass
    let big = Affine3A::from_translation(Vec3::new(2.0, 0.0, 0.0))
        * Affine3A::from_rotation_z(15f32.to_radians());
    let late = Affine3A::from_rotation_z((-12f32).to_radians());
    let slip = |t: f32, k: usize| {
        if t < 0.0 {
            big
        } else if t > 0.0 && k >= 9 {
            late
        } else {
            Affine3A::IDENTITY
        }
    };
    let stops = passes_scan(&[0.0, 15.0, -15.0], 30.0, &slip);
    let r = reconstruct_at(&stops, true);
    let rep = &r.report;
    assert_eq!(rep.rejected_passes, 1, "{rep:?}");
    assert_eq!(rep.rejected_stops, 12 + 3, "{rep:?}");
    assert_eq!(rep.stops, 36 - 15);
    assert_eq!(rep.warning.as_deref(), Some(WARN_VIEWS_REJECTED));
    let moved: Vec<bool> = stops
        .stops
        .iter()
        .enumerate()
        .map(|(i, s)| slip(s.angles.1, i % 12) != Affine3A::IDENTITY)
        .collect();
    assert_eq!(r.rejected, moved);
    for p in &rep.pass_details {
        assert_eq!(p.rejected, p.tilt_deg < 0.0, "{p:?}");
    }
    // no double walls: the mesh lies on the scene; fused as they are, the
    // moved views would make them
    let (mean, _) = error(&r.mesh);
    assert!(mean < 0.06, "mean {mean} {rep:?}");
    let (raw_mean, _) = error(&reconstruct_at(&stops, false).mesh);
    assert!(raw_mean > 2.0 * mean, "raw {raw_mean} gated {mean}");
}
