use super::*;

/// Deterministic uniform values in [0, 1).
struct Lcg(u64);

impl Lcg {
    fn next(&mut self) -> f32 {
        self.0 = self
            .0
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        (self.0 >> 40) as f32 / (1u64 << 24) as f32
    }
}

/// `n x n` grid on the plane z = 0 with spacing `s`.
fn grid(n: usize, s: f32) -> Vec<Vec3> {
    (0..n * n)
        .map(|i| Vec3::new((i % n) as f32 * s, (i / n) as f32 * s, 0.0))
        .collect()
}

/// Fibonacci sphere of `n` points.
fn sphere(n: usize, r: f32) -> Vec<Vec3> {
    let ga = std::f32::consts::PI * (3.0 - 5f32.sqrt());
    (0..n)
        .map(|i| {
            let y = 1.0 - 2.0 * (i as f32 + 0.5) / n as f32;
            let rr = (1.0 - y * y).sqrt();
            let t = ga * i as f32;
            Vec3::new(rr * t.cos(), y, rr * t.sin()) * r
        })
        .collect()
}

#[test]
fn voxel_downsample_averages_points_and_attributes() {
    let mut pts = Vec::new();
    for z in 0..10 {
        for y in 0..10 {
            for x in 0..10 {
                pts.push(Vec3::new(x as f32, y as f32, z as f32) * 0.1);
            }
        }
    }
    let n = pts.len();
    let cloud = PointCloud {
        points: pts,
        normals: Some(vec![Vec3::new(0.0, 0.0, 2.0); n]),
        colors: Some(vec![[10, 20, 30]; n]),
    };
    let d = voxel_downsample(&cloud, 0.5);
    assert_eq!(d.len(), 8);
    assert!((d.points[0] - Vec3::splat(0.2)).length() < 1e-5);
    assert!(
        d.normals
            .unwrap()
            .iter()
            .all(|n| (*n - Vec3::Z).length() < 1e-6)
    );
    assert!(d.colors.unwrap().iter().all(|c| *c == [10, 20, 30]));
    assert_eq!(voxel_downsample(&cloud, 0.0), cloud);
}

/// Plane grid plus scattered outliers at the end; returns the cloud and the
/// outlier indices.
fn plane_with_outliers() -> (PointCloud, Vec<u32>) {
    let mut pts = grid(30, 0.5);
    let first = pts.len() as u32;
    let mut rng = Lcg(7);
    for _ in 0..6 {
        pts.push(Vec3::new(
            rng.next() * 15.0,
            rng.next() * 15.0,
            5.0 + rng.next() * 20.0,
        ));
    }
    let outliers = (first..pts.len() as u32).collect();
    (PointCloud::from_points(pts), outliers)
}

#[test]
fn statistical_outliers_finds_injected_points() {
    let (cloud, outliers) = plane_with_outliers();
    assert_eq!(statistical_outliers(&cloud, 8, 2.0), outliers);
    assert!(statistical_outliers(&cloud, 0, 2.0).is_empty());
}

#[test]
fn radius_outliers_finds_injected_points() {
    let (cloud, outliers) = plane_with_outliers();
    assert_eq!(radius_outliers(&cloud, 1.0, 3), outliers);
}

#[test]
fn largest_cluster_keeps_the_main_part() {
    let mut pts = grid(10, 0.5);
    let main = pts.len() as u32;
    pts.extend(
        grid(4, 0.5)
            .into_iter()
            .map(|p| p + Vec3::new(20.0, 0.0, 0.0)),
    );
    pts.push(Vec3::new(-10.0, 0.0, 0.0));
    pts.push(Vec3::new(0.0, -10.0, 0.0));
    let cloud = PointCloud::from_points(pts);
    let removed = largest_cluster(&cloud, 0.6);
    assert_eq!(removed, (main..cloud.len() as u32).collect::<Vec<_>>());
}

#[test]
fn normals_of_a_sphere_point_outward_or_to_the_viewpoint() {
    let mut cloud = PointCloud::from_points(sphere(800, 10.0));
    estimate_normals(&mut cloud, 10, None);
    for (p, n) in cloud.points.iter().zip(cloud.normals.as_ref().unwrap()) {
        assert!(n.dot(p.normalize()) > 0.95, "{p} {n}");
    }
    // viewpoint inside: every normal turns inward
    estimate_normals(&mut cloud, 10, Some(Vec3::ZERO));
    for (p, n) in cloud.points.iter().zip(cloud.normals.as_ref().unwrap()) {
        assert!(n.dot(p.normalize()) < -0.95);
    }
    // no viewpoint, existing normals: their orientation is kept
    estimate_normals(&mut cloud, 10, None);
    for (p, n) in cloud.points.iter().zip(cloud.normals.as_ref().unwrap()) {
        assert!(n.dot(p.normalize()) < -0.95);
    }
}

#[test]
fn normals_of_an_open_plane_are_consistent() {
    let mut cloud = PointCloud::from_points(grid(20, 0.5));
    let mut rng = Lcg(3);
    for p in &mut cloud.points {
        p.z = (rng.next() - 0.5) * 0.02;
    }
    estimate_normals(&mut cloud, 8, None);
    let normals = cloud.normals.unwrap();
    let sign = normals[0].z.signum();
    assert!(normals.iter().all(|n| n.z * sign > 0.95));
}

#[test]
fn smoothing_flattens_noise() {
    let mut rng = Lcg(11);
    let mut cloud = PointCloud::from_points(grid(20, 0.5));
    for p in &mut cloud.points {
        p.z = (rng.next() - 0.5) * 0.2;
    }
    cloud.normals = Some(vec![Vec3::Z; cloud.len()]);
    let rms =
        |c: &PointCloud| (c.points.iter().map(|p| p.z * p.z).sum::<f32>() / c.len() as f32).sqrt();
    let s = smooth_cloud(&cloud, 12, 3, 1.0);
    assert!(rms(&s) < 0.5 * rms(&cloud), "{} {}", rms(&s), rms(&cloud));
    assert!(s.normals.unwrap().iter().all(|n| n.z > 0.9));
    assert_eq!(smooth_cloud(&cloud, 12, 0, 1.0), cloud);
}

#[test]
fn selections() {
    let cloud = PointCloud::from_points(vec![
        Vec3::ZERO,
        Vec3::new(1.0, 1.0, 1.0),
        Vec3::new(2.0, 0.0, 0.0),
        Vec3::new(-1.0, 0.5, 0.0),
    ]);
    assert_eq!(
        select_in_aabb(&cloud, Vec3::splat(1.0), Vec3::splat(-0.5)),
        vec![0, 1]
    );
    assert_eq!(select_in_sphere(&cloud, Vec3::X, 1.0), vec![0, 2]);
    assert_eq!(
        select_by_plane(&cloud, Vec3::new(0.5, 0.0, 0.0), Vec3::X),
        vec![1, 2]
    );
    assert!(select_by_plane(&cloud, Vec3::ZERO, Vec3::ZERO).is_empty());
}
