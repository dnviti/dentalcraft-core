use super::*;
use glam::Mat3;

fn intr(width: u32, height: u32, f: f32) -> CameraIntrinsics {
    CameraIntrinsics {
        width,
        height,
        fx: f,
        fy: f,
        cx: (width as f32 - 1.0) * 0.5,
        cy: (height as f32 - 1.0) * 0.5,
    }
}

/// Camera at `eye` looking at `target` (+Z forward, +Y down).
fn look_at(eye: Vec3, target: Vec3, up: Vec3) -> Affine3A {
    let z = (target - eye).normalize();
    let x = (-up).cross(z).normalize();
    let y = z.cross(x);
    Affine3A::from_mat3_translation(Mat3::from_cols(x, y, z), eye)
}

/// Depth image of a sphere seen from `pose` (0 where the ray misses).
fn sphere_depth(i: &CameraIntrinsics, pose: &Affine3A, c: Vec3, r: f32) -> Vec<f32> {
    let o = pose.transform_point3(Vec3::ZERO);
    let mut depth = vec![0.0; i.pixel_count()];
    for v in 0..i.height {
        for u in 0..i.width {
            let d = pose.transform_vector3(i.ray(u as f32, v as f32));
            let oc = o - c;
            let a = d.length_squared();
            let b = d.dot(oc);
            let disc = b * b - a * (oc.length_squared() - r * r);
            if disc >= 0.0 {
                let t = (-b - disc.sqrt()) / a;
                if t > 0.0 {
                    depth[(v * i.width + u) as usize] = t;
                }
            }
        }
    }
    depth
}

#[test]
fn plane_is_reconstructed_within_a_voxel() {
    let cam = intr(64, 48, 60.0);
    let mut vol = TsdfVolume::new(TsdfParams {
        voxel_mm: 0.5,
        trunc_mm: 2.0,
        max_weight: 32.0,
    });
    let mut depth = vec![50.0; cam.pixel_count()];
    depth[10] = 0.0;
    depth[11] = f32::NAN;
    for _ in 0..3 {
        vol.integrate(&depth, &cam, &Affine3A::IDENTITY);
    }
    assert!(vol.block_count() > 0);

    let cloud = vol.extract_cloud();
    assert!(cloud.len() > 500);
    let normals = cloud.normals.as_ref().unwrap();
    for (p, n) in cloud.points.iter().zip(normals) {
        assert!((p.z - 50.0).abs() < 0.5, "{p}");
        assert!(n.dot(Vec3::NEG_Z) > 0.95, "{n}");
    }

    let mesh = vol.extract_mesh();
    assert!(mesh.face_count() > 500);
    assert_eq!(mesh.edge_adjacency().non_manifold_edge_count(), 0);
    for p in &mesh.positions {
        assert!((p.z - 50.0).abs() < 0.5, "{p}");
    }
    for f in 0..mesh.face_count() {
        assert!(mesh.face_normal(f).z < -0.9);
    }

    vol.clear();
    assert_eq!(vol.block_count(), 0);
    assert!(vol.extract_mesh().is_empty());
}

#[test]
fn sphere_from_three_views_is_within_a_voxel() {
    let cam = intr(96, 72, 110.0);
    let c = Vec3::new(0.0, 0.0, 60.0);
    let r = 12.0;
    let vs = 0.4;
    let mut vol = TsdfVolume::new(TsdfParams {
        voxel_mm: vs,
        trunc_mm: 4.0 * vs,
        max_weight: 32.0,
    });
    let poses: Vec<Affine3A> = [-25.0f32, 0.0, 25.0]
        .iter()
        .map(|&a| {
            let a = a.to_radians();
            let eye = c + Vec3::new(a.sin(), 0.0, -a.cos()) * 60.0;
            look_at(eye, c, Vec3::NEG_Y)
        })
        .collect();
    for pose in &poses {
        vol.integrate(&sphere_depth(&cam, pose, c, r), &cam, pose);
    }

    let mesh = vol.extract_mesh();
    assert!(mesh.face_count() > 1000);
    assert_eq!(mesh.edge_adjacency().non_manifold_edge_count(), 0);
    let err: Vec<f32> = mesh
        .positions
        .iter()
        .map(|p| ((*p - c).length() - r).abs())
        .collect();
    let max = err.iter().copied().fold(0.0, f32::max);
    let mean = err.iter().sum::<f32>() / err.len() as f32;
    assert!(max < vs, "max {max}");
    assert!(mean < 0.25 * vs, "mean {mean}");
    // faces look outward
    let outward = (0..mesh.face_count())
        .filter(|&f| {
            let [a, b, cc] = mesh.triangle(f);
            mesh.face_normal(f).dot((a + b + cc) / 3.0 - c) > 0.0
        })
        .count();
    assert_eq!(outward, mesh.face_count());

    // normals: grazing views at the silhouettes of the three cameras give a
    // few tilted gradients, every normal still points outward
    let cloud = vol.extract_cloud();
    let normals = cloud.normals.as_ref().unwrap();
    let mut tilted = 0;
    for (p, n) in cloud.points.iter().zip(normals) {
        assert!(((*p - c).length() - r).abs() < vs, "{p}");
        let cos = n.dot((*p - c).normalize());
        assert!(cos > 0.3, "{p} {n}");
        tilted += usize::from(cos < 0.8);
    }
    assert!(tilted * 50 < cloud.len(), "{tilted} of {}", cloud.len());

    // raycast from the middle view: hits on the sphere, normals facing the camera
    let (pts, nrm) = vol.raycast(&cam, &poses[1]);
    let depth = sphere_depth(&cam, &poses[1], c, r);
    let mut hits = 0;
    let mut tilted = 0;
    let mut beyond_silhouette = 0;
    for (i, p) in pts.iter().enumerate() {
        if p.is_nan() {
            assert!(nrm[i].is_nan());
            continue;
        }
        hits += 1;
        beyond_silhouette += usize::from(depth[i] <= 0.0);
        assert!(((*p - c).length() - r).abs() < vs, "{p}");
        let cos = nrm[i].dot((*p - c).normalize());
        assert!(cos > 0.3, "{p} {}", nrm[i]);
        tilted += usize::from(cos < 0.8);
    }
    let expected = depth.iter().filter(|&&d| d > 0.0).count();
    assert!(hits as f32 > 0.9 * expected as f32, "{hits} of {expected}");
    assert!(tilted * 50 < hits, "{tilted} of {hits}");
    assert!(
        beyond_silhouette * 50 < hits,
        "{beyond_silhouette} of {hits}"
    );
}

#[test]
fn raycast_of_empty_volume_is_all_nan() {
    let cam = intr(8, 6, 10.0);
    let vol = TsdfVolume::new(TsdfParams::default());
    let (p, n) = vol.raycast(&cam, &Affine3A::IDENTITY);
    assert_eq!(p.len(), 48);
    assert!(p.iter().chain(&n).all(|v| v.is_nan()));
}

#[test]
fn wrong_size_frame_is_ignored() {
    let cam = intr(8, 6, 10.0);
    let mut vol = TsdfVolume::new(TsdfParams::default());
    vol.integrate(&[100.0; 10], &cam, &Affine3A::IDENTITY);
    vol.integrate(&[0.0; 48], &cam, &Affine3A::IDENTITY);
    assert_eq!(vol.block_count(), 0);
}

#[test]
fn backproject_skips_invalid_and_strides() {
    let cam = intr(4, 2, 2.0);
    let mut depth = vec![10.0; 8];
    depth[1] = 0.0;
    depth[2] = f32::NAN;
    let (pts, px) = backproject(&depth, &cam, 1);
    assert_eq!(px, vec![0, 3, 4, 5, 6, 7]);
    // pixel 0: u = 0, v = 0 -> ((0 - 1.5) / 2, (0 - 0.5) / 2, 1) * 10
    assert!((pts[0] - Vec3::new(-7.5, -2.5, 10.0)).length() < 1e-5);
    let (_, px2) = backproject(&depth, &cam, 2);
    assert_eq!(px2, vec![0]);
}

#[test]
fn intrinsics_project_and_downscale() {
    let cam = intr(640, 400, 800.0);
    let p = cam.ray(100.0, 50.0) * 200.0;
    let (u, v) = cam.project(p).unwrap();
    assert!((u - 100.0).abs() < 1e-3 && (v - 50.0).abs() < 1e-3);
    let half = cam.downscaled(2);
    assert_eq!((half.width, half.height), (320, 200));
    assert!((half.cx - 159.5).abs() < 1e-4);
}

/// Release-mode performance check (run with `cargo test --release -p dc-geom
/// -- --ignored tsdf_integrate_speed --nocapture`).
#[test]
#[ignore = "timing, release builds only"]
fn tsdf_integrate_speed() {
    let cam = intr(640, 400, 800.0);
    let c = Vec3::new(0.0, 0.0, 200.0);
    let pose = Affine3A::IDENTITY;
    let depth = sphere_depth(&cam, &pose, c, 40.0);
    let mut vol = TsdfVolume::new(TsdfParams::default());
    vol.integrate(&depth, &cam, &pose);
    let runs = 10;
    let t = std::time::Instant::now();
    for _ in 0..runs {
        vol.integrate(&depth, &cam, &pose);
    }
    let ms = t.elapsed().as_secs_f64() * 1000.0 / runs as f64;
    let t = std::time::Instant::now();
    let (pts, _) = vol.raycast(&cam.downscaled(2), &pose);
    let ray_ms = t.elapsed().as_secs_f64() * 1000.0;
    println!(
        "integrate 640x400 @0.3 mm: {ms:.1} ms/frame, {} blocks; raycast 320x200: {ray_ms:.1} ms ({} hits)",
        vol.block_count(),
        pts.iter().filter(|p| !p.is_nan()).count()
    );
    assert!(ms < 30.0, "{ms} ms");
}
