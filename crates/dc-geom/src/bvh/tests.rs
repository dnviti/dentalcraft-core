use super::*;

/// Tiny deterministic generator (no extra dependency).
struct Lcg(u64);

impl Lcg {
    fn next_f32(&mut self) -> f32 {
        self.0 = self
            .0
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        ((self.0 >> 40) as f32) / ((1u64 << 24) as f32)
    }

    fn range(&mut self, lo: f32, hi: f32) -> f32 {
        lo + (hi - lo) * self.next_f32()
    }

    fn vec(&mut self, lo: f32, hi: f32) -> Vec3 {
        Vec3::new(self.range(lo, hi), self.range(lo, hi), self.range(lo, hi))
    }

    fn unit(&mut self) -> Vec3 {
        loop {
            let v = self.vec(-1.0, 1.0);
            if v.length_squared() > 1e-4 && v.length_squared() <= 1.0 {
                return v.normalize();
            }
        }
    }
}

/// `n` small random triangles inside the cube `[-1, 1]^3`.
fn soup(n: usize, rng: &mut Lcg) -> TriMesh {
    let mut pos = Vec::new();
    let mut tris = Vec::new();
    for i in 0..n as u32 {
        let c = rng.vec(-1.0, 1.0);
        for _ in 0..3 {
            pos.push(c + rng.vec(-0.15, 0.15));
        }
        tris.push([3 * i, 3 * i + 1, 3 * i + 2]);
    }
    TriMesh::new(pos, tris)
}

/// Closed UV sphere with `2 * seg * (rings - 1)` triangles.
fn sphere(radius: f32, seg: u32, rings: u32) -> TriMesh {
    let mut pos = vec![Vec3::new(0.0, 0.0, radius)];
    for r in 1..rings {
        let th = std::f32::consts::PI * r as f32 / rings as f32;
        for s in 0..seg {
            let ph = std::f32::consts::TAU * s as f32 / seg as f32;
            pos.push(radius * Vec3::new(th.sin() * ph.cos(), th.sin() * ph.sin(), th.cos()));
        }
    }
    pos.push(Vec3::new(0.0, 0.0, -radius));
    let south = pos.len() as u32 - 1;
    let ring = |r: u32, s: u32| 1 + (r - 1) * seg + s % seg;
    let mut tris = Vec::new();
    for s in 0..seg {
        tris.push([0, ring(1, s), ring(1, s + 1)]);
    }
    for r in 1..rings - 1 {
        for s in 0..seg {
            let (a, b, c, d) = (
                ring(r, s),
                ring(r, s + 1),
                ring(r + 1, s),
                ring(r + 1, s + 1),
            );
            tris.push([a, c, b]);
            tris.push([b, c, d]);
        }
    }
    for s in 0..seg {
        tris.push([south, ring(rings - 1, s + 1), ring(rings - 1, s)]);
    }
    TriMesh::new(pos, tris)
}

/// Wavy height field of `n * n * 2` triangles over `[0, n]^2`.
fn terrain(n: u32) -> TriMesh {
    let mut pos = Vec::new();
    for j in 0..=n {
        for i in 0..=n {
            let (x, y) = (i as f32, j as f32);
            pos.push(Vec3::new(x, y, (x * 0.7).sin() * (y * 0.4).cos() * 2.0));
        }
    }
    let w = n + 1;
    let mut tris = Vec::new();
    for j in 0..n {
        for i in 0..n {
            let a = j * w + i;
            tris.push([a, a + 1, a + w + 1]);
            tris.push([a, a + w + 1, a + w]);
        }
    }
    TriMesh::new(pos, tris)
}

fn brute_closest(mesh: &TriMesh, p: Vec3, max_d: f32) -> Option<(u32, f64)> {
    let mut best: Option<(u32, f64)> = None;
    let pd = p.as_dvec3();
    for f in 0..mesh.tris.len() {
        let [a, b, c] = mesh.triangle(f).map(|v| v.as_dvec3());
        let (pt, _) = closest_on_triangle(pd, a, b, c);
        let d = (pt - pd).length();
        if best.is_none_or(|(_, bd)| d < bd) {
            best = Some((f as u32, d));
        }
    }
    best.filter(|&(_, d)| d <= max_d as f64)
}

fn brute_rays(mesh: &TriMesh, o: Vec3, d: Vec3, max_t: f32) -> Vec<(u32, f64)> {
    let mut hits = Vec::new();
    for f in 0..mesh.tris.len() {
        let [a, b, c] = mesh.triangle(f).map(|v| v.as_dvec3());
        if let Some((t, _)) = ray_triangle(o.as_dvec3(), d.as_dvec3(), a, b, c)
            && (0.0..=max_t as f64).contains(&t)
        {
            hits.push((f as u32, t));
        }
    }
    hits.sort_by(|x, y| x.1.total_cmp(&y.1).then(x.0.cmp(&y.0)));
    hits
}

fn check_closest(mesh: &TriMesh, queries: usize, spread: f32, seed: u64) {
    let bvh = Bvh::build(mesh).unwrap();
    let mut rng = Lcg(seed);
    let points: Vec<Vec3> = (0..queries).map(|_| rng.vec(-spread, spread)).collect();
    let batch = bvh.closest_points(&points, f32::INFINITY);
    for (i, &p) in points.iter().enumerate() {
        let got = bvh.closest_point(p, f32::INFINITY).unwrap();
        assert_eq!(Some(got), batch[i]);
        let (_, bd) = brute_closest(mesh, p, f32::INFINITY).unwrap();
        assert!(
            (got.distance as f64 - bd).abs() <= 1e-5 * (1.0 + bd),
            "query {i}: bvh {} brute {bd}",
            got.distance
        );
        // the point lies on the reported triangle, with consistent barycentrics
        let t = mesh.triangle(got.face as usize);
        let q = t[0] * got.barycentric.x + t[1] * got.barycentric.y + t[2] * got.barycentric.z;
        assert!((q - got.point).length() < 1e-4, "barycentric mismatch");
        assert!((got.barycentric.element_sum() - 1.0).abs() < 1e-5);
        assert!(got.barycentric.min_element() >= 0.0);
        assert!(((got.point - p).length() - got.distance).abs() < 1e-4);
    }
}

fn check_rays(mesh: &TriMesh, queries: usize, origin_spread: f32, seed: u64) {
    let bvh = Bvh::build(mesh).unwrap();
    let mut rng = Lcg(seed);
    let origins: Vec<Vec3> = (0..queries)
        .map(|_| rng.vec(-origin_spread, origin_spread))
        .collect();
    let dirs: Vec<Vec3> = (0..queries)
        .map(|_| rng.unit() * rng.range(0.5, 2.0))
        .collect();
    let batch = bvh.ray_casts(&origins, &dirs, f32::INFINITY);
    let mut with_hit = 0;
    for i in 0..queries {
        let (o, d) = (origins[i], dirs[i]);
        let brute = brute_rays(mesh, o, d, f32::INFINITY);
        let first = bvh.ray_cast(o, d, f32::INFINITY);
        assert_eq!(first, batch[i]);
        match (first, brute.first()) {
            (Some(h), Some(&(bf, bt))) => {
                with_hit += 1;
                assert_eq!(h.face, bf, "ray {i}");
                assert!((h.t as f64 - bt).abs() <= 1e-5 * (1.0 + bt), "ray {i}");
                let t = mesh.triangle(h.face as usize);
                let q = t[0] * h.barycentric.x + t[1] * h.barycentric.y + t[2] * h.barycentric.z;
                assert!((q - (o + d * h.t)).length() < 1e-3);
            }
            (None, None) => {}
            (a, b) => panic!("ray {i}: bvh {a:?} brute {b:?}"),
        }
        let all = bvh.ray_all(o, d, f32::INFINITY);
        assert_eq!(all.len(), brute.len(), "ray {i}");
        for (h, &(bf, bt)) in all.iter().zip(&brute) {
            assert_eq!(h.face, bf);
            assert!((h.t as f64 - bt).abs() <= 1e-5 * (1.0 + bt));
        }
        assert!(all.windows(2).all(|w| w[0].t <= w[1].t));
    }
    assert!(with_hit > queries / 20, "too few hits: {with_hit}");
}

#[test]
fn closest_point_matches_brute_force_on_random_soup() {
    let mut rng = Lcg(7);
    check_closest(&soup(1500, &mut rng), 10_000, 1.6, 1);
}

#[test]
fn closest_point_matches_brute_force_on_sphere() {
    check_closest(&sphere(1.0, 48, 24), 10_000, 2.0, 2);
}

#[test]
fn rays_match_brute_force_on_random_soup() {
    let mut rng = Lcg(8);
    check_rays(&soup(1500, &mut rng), 10_000, 1.5, 3);
}

#[test]
fn rays_match_brute_force_on_sphere() {
    check_rays(&sphere(1.0, 40, 20), 10_000, 2.0, 4);
}

#[test]
fn sphere_rays_cross_twice_at_analytic_distance() {
    let bvh = Bvh::build(&sphere(1.0, 96, 48)).unwrap();
    let hits = bvh.ray_hits(
        Vec3::new(0.3, 0.2, 5.0),
        Vec3::NEG_Z,
        f32::INFINITY,
        8,
        1e-4,
    );
    assert_eq!(hits.len(), 2);
    let z = (1.0f32 - 0.3 * 0.3 - 0.2 * 0.2).sqrt();
    assert!((hits[0].t - (5.0 - z)).abs() < 5e-3, "{}", hits[0].t);
    assert!((hits[1].t - (5.0 + z)).abs() < 5e-3, "{}", hits[1].t);
}

#[test]
fn closest_point_respects_max_distance() {
    let mesh = sphere(1.0, 32, 16);
    let bvh = Bvh::build(&mesh).unwrap();
    let p = Vec3::new(0.0, 0.0, 3.0);
    assert!(bvh.closest_point(p, 1.5).is_none());
    let h = bvh.closest_point(p, 2.5).unwrap();
    assert!((h.distance - 2.0).abs() < 1e-3);
    assert!(bvh.closest_point(p, -1.0).is_none());
    assert!(bvh.closest_point(p, f32::NAN).is_none());
}

#[test]
fn closest_point_ties_go_to_the_smallest_face() {
    // two coincident triangles
    let pos = vec![Vec3::ZERO, Vec3::X, Vec3::Y];
    let mesh = TriMesh::new(pos, vec![[0, 1, 2], [0, 1, 2], [0, 2, 1]]);
    let bvh = Bvh::build(&mesh).unwrap();
    assert_eq!(
        bvh.closest_point(Vec3::new(0.2, 0.2, 1.0), 10.0)
            .unwrap()
            .face,
        0
    );
    let hits = bvh.ray_all(Vec3::new(0.2, 0.2, 1.0), Vec3::NEG_Z, 10.0);
    assert_eq!(hits.iter().map(|h| h.face).collect::<Vec<_>>(), [0, 1, 2]);
    assert_eq!(
        bvh.ray_hits(Vec3::new(0.2, 0.2, 1.0), Vec3::NEG_Z, 10.0, 5, 1e-4)
            .len(),
        1
    );
}

#[test]
fn empty_and_invalid_meshes() {
    let bvh = Bvh::build(&TriMesh::default()).unwrap();
    assert!(bvh.is_empty());
    assert_eq!(bvh.triangle_count(), 0);
    assert!(bvh.bounds().is_none());
    assert!(bvh.closest_point(Vec3::ZERO, f32::INFINITY).is_none());
    assert!(bvh.ray_cast(Vec3::ZERO, Vec3::Z, 1.0).is_none());
    assert!(bvh.ray_all(Vec3::ZERO, Vec3::Z, 1.0).is_empty());
    assert!(bvh.ray_hits(Vec3::ZERO, Vec3::Z, 1.0, 4, 1e-4).is_empty());
    assert!(bvh.overlap_sphere(Vec3::ZERO, 1.0).is_empty());
    assert!(bvh.overlap_aabb(Vec3::NEG_ONE, Vec3::ONE).is_empty());
    assert!(bvh.closest_points(&[Vec3::ZERO], 1.0)[0].is_none());
    // vertices but no triangles
    let bvh = Bvh::build(&TriMesh::new(vec![Vec3::ZERO], vec![])).unwrap();
    assert!(bvh.is_empty());
    // out-of-range index is an error
    let bad = TriMesh::new(vec![Vec3::ZERO], vec![[0, 1, 2]]);
    assert!(matches!(Bvh::build(&bad), Err(GeomError::InvalidInput(_))));
}

#[test]
fn non_finite_triangles_and_queries_are_ignored() {
    let pos = vec![
        Vec3::ZERO,
        Vec3::X,
        Vec3::Y,
        Vec3::new(f32::NAN, 0.0, 0.0),
        Vec3::X,
        Vec3::Y,
    ];
    let mesh = TriMesh::new(pos, vec![[0, 1, 2], [3, 4, 5]]);
    let bvh = Bvh::build(&mesh).unwrap();
    assert_eq!(bvh.triangle_count(), 1);
    let h = bvh
        .closest_point(Vec3::new(0.2, 0.2, 1.0), f32::INFINITY)
        .unwrap();
    assert_eq!(h.face, 0);
    let nan = Vec3::new(f32::NAN, 0.0, 0.0);
    assert!(bvh.closest_point(nan, f32::INFINITY).is_none());
    assert!(bvh.ray_cast(nan, Vec3::Z, 1.0).is_none());
    assert!(bvh.ray_cast(Vec3::ZERO, nan, 1.0).is_none());
    assert!(
        bvh.ray_cast(Vec3::new(0.2, 0.2, 1.0), Vec3::ZERO, 10.0)
            .is_none()
    );
    assert!(
        bvh.ray_all(Vec3::new(0.2, 0.2, 1.0), Vec3::ZERO, 10.0)
            .is_empty()
    );
}

#[test]
fn degenerate_triangles() {
    // 0: a sliver segment, 1: all three corners equal, 2: a proper triangle far away
    let pos = vec![
        Vec3::ZERO,
        Vec3::new(2.0, 0.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 5.0, 0.0),
        Vec3::new(10.0, 0.0, 0.0),
        Vec3::new(11.0, 0.0, 0.0),
        Vec3::new(10.0, 1.0, 0.0),
    ];
    let mesh = TriMesh::new(pos, vec![[0, 1, 2], [3, 3, 3], [4, 5, 6]]);
    let bvh = Bvh::build(&mesh).unwrap();
    assert_eq!(bvh.triangle_count(), 3);
    // closest point on the collinear triangle: projects on the segment
    let h = bvh
        .closest_point(Vec3::new(1.5, 1.0, 0.0), f32::INFINITY)
        .unwrap();
    assert_eq!(h.face, 0);
    assert!((h.point - Vec3::new(1.5, 0.0, 0.0)).length() < 1e-6);
    assert!((h.distance - 1.0).abs() < 1e-6);
    assert!(h.barycentric.is_finite() && (h.barycentric.element_sum() - 1.0).abs() < 1e-6);
    // the point-like triangle is found
    let h = bvh
        .closest_point(Vec3::new(0.0, 7.0, 0.0), f32::INFINITY)
        .unwrap();
    assert_eq!(h.face, 1);
    assert!((h.distance - 2.0).abs() < 1e-6);
    // degenerate triangles are never hit by a ray, others are
    assert!(
        bvh.ray_cast(Vec3::new(1.0, 0.0, 1.0), Vec3::NEG_Z, 10.0)
            .is_none()
    );
    assert!(
        bvh.ray_cast(Vec3::new(0.0, 5.0, 1.0), Vec3::NEG_Z, 10.0)
            .is_none()
    );
    let h = bvh
        .ray_cast(Vec3::new(10.2, 0.2, 1.0), Vec3::NEG_Z, 10.0)
        .unwrap();
    assert_eq!(h.face, 2);
    // all-degenerate mesh
    let m = TriMesh::new(vec![Vec3::ONE], vec![[0, 0, 0]]);
    let b = Bvh::build(&m).unwrap();
    assert!(
        (b.closest_point(Vec3::ZERO, f32::INFINITY).unwrap().distance - 3f32.sqrt()).abs() < 1e-6
    );
}

#[test]
fn ray_parallel_to_a_face() {
    let mesh = TriMesh::new(vec![Vec3::ZERO, Vec3::X, Vec3::Y], vec![[0, 1, 2]]);
    let bvh = Bvh::build(&mesh).unwrap();
    // parallel above the plane, and inside the plane
    assert!(
        bvh.ray_cast(Vec3::new(-1.0, 0.2, 0.5), Vec3::X, 10.0)
            .is_none()
    );
    assert!(
        bvh.ray_cast(Vec3::new(-1.0, 0.2, 0.0), Vec3::X, 10.0)
            .is_none()
    );
    assert!(
        bvh.ray_all(Vec3::new(-1.0, 0.2, 0.0), Vec3::X, 10.0)
            .is_empty()
    );
    // a slightly tilted ray does hit, from both sides
    let h = bvh
        .ray_cast(Vec3::new(0.1, 0.2, 1.0), Vec3::new(0.0, 0.0, -1.0), 10.0)
        .unwrap();
    assert!((h.t - 1.0).abs() < 1e-6);
    let h = bvh
        .ray_cast(Vec3::new(0.1, 0.2, -1.0), Vec3::new(0.01, 0.0, 1.0), 10.0)
        .unwrap();
    assert_eq!(h.face, 0);
    // axis-aligned rays against a box-shaped mesh (zero direction components)
    let b = Bvh::build(&terrain(8)).unwrap();
    let o = Vec3::new(3.5, 4.25, 50.0);
    let h = b.ray_cast(o, Vec3::NEG_Z, f32::INFINITY).unwrap();
    let br = brute_rays(&terrain(8), o, Vec3::NEG_Z, f32::INFINITY);
    assert_eq!(h.face, br[0].0);
}

#[test]
fn ray_range_and_origin_on_surface() {
    let mesh = TriMesh::new(vec![Vec3::ZERO, Vec3::X, Vec3::Y], vec![[0, 1, 2]]);
    let bvh = Bvh::build(&mesh).unwrap();
    let o = Vec3::new(0.2, 0.2, 1.0);
    assert!(bvh.ray_cast(o, Vec3::NEG_Z, 0.5).is_none());
    assert!(bvh.ray_cast_range(o, Vec3::NEG_Z, 1.5, 3.0).is_none());
    assert!(bvh.ray_cast_range(o, Vec3::NEG_Z, 0.5, 3.0).is_some());
    // behind the origin is not hit
    assert!(bvh.ray_cast(o, Vec3::Z, 10.0).is_none());
    // origin on the surface: t = 0 is a hit
    let h = bvh
        .ray_cast(Vec3::new(0.2, 0.2, 0.0), Vec3::NEG_Z, 1.0)
        .unwrap();
    assert!(h.t.abs() < 1e-6);
    assert!(bvh.ray_cast_range(o, Vec3::NEG_Z, 2.0, 1.0).is_none());
}

#[test]
fn ray_through_shared_edge_is_not_lost() {
    let mesh = terrain(6);
    let bvh = Bvh::build(&mesh).unwrap();
    // the diagonal of cell (2, 3) runs from (2, 3) to (3, 4): aim at its middle
    let o = Vec3::new(2.5, 3.5, 20.0);
    let hits = bvh.ray_hits(o, Vec3::NEG_Z, f32::INFINITY, 4, 1e-4);
    assert_eq!(hits.len(), 1);
    // and through a vertex
    let hits = bvh.ray_hits(
        Vec3::new(3.0, 3.0, 20.0),
        Vec3::NEG_Z,
        f32::INFINITY,
        4,
        1e-4,
    );
    assert_eq!(hits.len(), 1);
}

#[test]
fn ray_hits_multi_hit_semantics() {
    // three parallel sheets at z = 0, 1, 2 and a hit spacing check
    let mut pos = Vec::new();
    let mut tris = Vec::new();
    for k in 0..3u32 {
        let z = k as f32;
        pos.extend([
            Vec3::new(-1.0, -1.0, z),
            Vec3::new(1.0, -1.0, z),
            Vec3::new(0.0, 1.0, z),
        ]);
        tris.push([3 * k, 3 * k + 1, 3 * k + 2]);
    }
    let bvh = Bvh::build(&TriMesh::new(pos, tris)).unwrap();
    let o = Vec3::new(0.0, 0.0, -1.0);
    let hits = bvh.ray_hits(o, Vec3::Z, f32::INFINITY, 10, 1e-4);
    assert_eq!(hits.iter().map(|h| h.face).collect::<Vec<_>>(), [0, 1, 2]);
    assert_eq!(
        hits.iter().map(|h| h.t.round() as i32).collect::<Vec<_>>(),
        [1, 2, 3]
    );
    assert_eq!(bvh.ray_hits(o, Vec3::Z, f32::INFINITY, 2, 1e-4).len(), 2);
    assert_eq!(bvh.ray_hits(o, Vec3::Z, 2.5, 10, 1e-4).len(), 2);
    assert!(bvh.ray_hits(o, Vec3::Z, f32::INFINITY, 0, 1e-4).is_empty());
    // a large gap skips the near sheets
    assert_eq!(bvh.ray_hits(o, Vec3::Z, f32::INFINITY, 10, 1.5).len(), 2);
    // zero gap still terminates
    assert_eq!(bvh.ray_hits(o, Vec3::Z, f32::INFINITY, 10, 0.0).len(), 3);
    // batch version agrees
    let b = bvh.ray_hits_batch(
        &[o, o + Vec3::Z * 1.5],
        &[Vec3::Z, Vec3::Z],
        f32::INFINITY,
        10,
        1e-4,
    );
    assert_eq!(b[0], hits);
    assert_eq!(b[1].len(), 2);
}

#[test]
fn overlap_sphere_matches_brute_force() {
    let mut rng = Lcg(11);
    let mesh = soup(1200, &mut rng);
    let bvh = Bvh::build(&mesh).unwrap();
    let centers: Vec<Vec3> = (0..300).map(|_| rng.vec(-1.3, 1.3)).collect();
    let batch = bvh.overlap_spheres(&centers, 0.3);
    for (i, &c) in centers.iter().enumerate() {
        let got = bvh.overlap_sphere(c, 0.3);
        assert_eq!(got, batch[i]);
        let want: Vec<u32> = (0..mesh.tris.len() as u32)
            .filter(|&f| {
                let [a, b, cc] = mesh.triangle(f as usize).map(|v| v.as_dvec3());
                let (pt, _) = closest_on_triangle(c.as_dvec3(), a, b, cc);
                (pt - c.as_dvec3()).length() <= 0.3f32 as f64
            })
            .collect();
        assert_eq!(got, want, "sphere {i}");
    }
    assert!(bvh.overlap_sphere(Vec3::ZERO, -1.0).is_empty());
}

#[test]
fn overlap_aabb_matches_sampled_brute_force() {
    let mut rng = Lcg(12);
    let mesh = soup(800, &mut rng);
    let bvh = Bvh::build(&mesh).unwrap();
    for _ in 0..200 {
        let c = rng.vec(-1.2, 1.2);
        let h = rng.vec(0.02, 0.4);
        let (lo, hi) = (c - h, c + h);
        let got = bvh.overlap_aabb(lo, hi);
        // reference: clip by the box bounds of each triangle, then require that
        // the closest point of the triangle to the box centre scaled metric
        // (Chebyshev distance) is within the half extents
        let want: Vec<u32> = (0..mesh.tris.len() as u32)
            .filter(|&f| tri_box_reference(&mesh.triangle(f as usize), lo, hi))
            .collect();
        assert_eq!(got, want);
    }
    assert!(bvh.overlap_aabb(Vec3::ONE, Vec3::NEG_ONE).is_empty());
}

/// Reference triangle/box test: dense sampling of the triangle plus the
/// box-edge/triangle crossings, accurate enough for random inputs where the
/// ambiguous configurations have measure zero.
fn tri_box_reference(t: &[Vec3; 3], lo: Vec3, hi: Vec3) -> bool {
    let inside = |p: Vec3| p.cmpge(lo).all() && p.cmple(hi).all();
    let n = 40;
    for i in 0..=n {
        for j in 0..=(n - i) {
            let (u, v) = (i as f32 / n as f32, j as f32 / n as f32);
            if inside(t[0] * (1.0 - u - v) + t[1] * u + t[2] * v) {
                return true;
            }
        }
    }
    // box edges crossing the triangle
    let c = |i: usize| {
        Vec3::new(
            if i & 1 == 0 { lo.x } else { hi.x },
            if i & 2 == 0 { lo.y } else { hi.y },
            if i & 4 == 0 { lo.z } else { hi.z },
        )
    };
    for a in 0..8usize {
        for b in (a + 1)..8 {
            if (a ^ b).count_ones() != 1 {
                continue;
            }
            let (p, q) = (c(a), c(b));
            if let Some((s, _)) = ray_triangle(
                p.as_dvec3(),
                (q - p).as_dvec3(),
                t[0].as_dvec3(),
                t[1].as_dvec3(),
                t[2].as_dvec3(),
            ) && (0.0..=1.0).contains(&s)
            {
                return true;
            }
        }
    }
    false
}

#[test]
fn closest_on_triangle_regions() {
    let (a, b, c) = (
        DVec3::ZERO,
        DVec3::new(2.0, 0.0, 0.0),
        DVec3::new(0.0, 2.0, 0.0),
    );
    let cases = [
        (DVec3::new(0.5, 0.5, 3.0), DVec3::new(0.5, 0.5, 0.0)), // face
        (DVec3::new(-1.0, -1.0, 0.0), a),                       // vertex a
        (DVec3::new(3.0, -1.0, 0.0), b),                        // vertex b
        (DVec3::new(-1.0, 3.0, 0.0), c),                        // vertex c
        (DVec3::new(1.0, -1.0, 0.0), DVec3::new(1.0, 0.0, 0.0)), // edge ab
        (DVec3::new(-1.0, 1.0, 0.0), DVec3::new(0.0, 1.0, 0.0)), // edge ac
        (DVec3::new(2.0, 2.0, 0.0), DVec3::new(1.0, 1.0, 0.0)), // edge bc
    ];
    for (p, want) in cases {
        let (q, w) = closest_on_triangle(p, a, b, c);
        assert!((q - want).length() < 1e-12, "{p:?}: {q:?}");
        assert!((a * w.x + b * w.y + c * w.z - q).length() < 1e-12);
    }
}

#[test]
fn structure_is_sound() {
    let mut rng = Lcg(5);
    let mesh = soup(5000, &mut rng);
    let bvh = Bvh::build(&mesh).unwrap();
    assert_eq!(bvh.triangle_count(), 5000);
    assert!(bvh.node_count() > 1 && bvh.node_count() < 2 * 5000);
    let mut ids = bvh.ids.clone();
    ids.sort_unstable();
    assert!(ids.iter().enumerate().all(|(i, &f)| i as u32 == f));
    // every leaf box contains its triangles; every inner box its children
    fn check(bvh: &Bvh, ni: usize) {
        let n = &bvh.nodes[ni];
        let (lo, hi) = (Vec3::from(n.min), Vec3::from(n.max));
        if n.is_leaf() {
            let s = n.a as usize;
            for slot in s..s + n.count as usize {
                for p in bvh.tris[slot] {
                    assert!(p.cmpge(lo).all() && p.cmple(hi).all());
                }
            }
        } else {
            for c in [ni + 1, ni + n.a as usize] {
                let k = &bvh.nodes[c];
                assert!(Vec3::from(k.min).cmpge(lo).all() && Vec3::from(k.max).cmple(hi).all());
                check(bvh, c);
            }
        }
    }
    check(&bvh, 0);
    let bounds = bvh.bounds().unwrap();
    let want = mesh.aabb().unwrap();
    assert_eq!((bounds.min, bounds.max), (want.min, want.max));
}

#[test]
fn many_identical_triangles_do_not_break_the_build() {
    let pos = vec![Vec3::ZERO, Vec3::X, Vec3::Y];
    let mesh = TriMesh::new(pos, vec![[0, 1, 2]; 20_000]);
    let bvh = Bvh::build(&mesh).unwrap();
    assert_eq!(bvh.triangle_count(), 20_000);
    assert_eq!(
        bvh.closest_point(Vec3::new(0.2, 0.2, 1.0), 5.0)
            .unwrap()
            .face,
        0
    );
    assert_eq!(
        bvh.ray_all(Vec3::new(0.2, 0.2, 1.0), Vec3::NEG_Z, 5.0)
            .len(),
        20_000
    );
}

#[test]
fn large_coordinates_keep_precision() {
    // a scan far from the origin: f64 query math keeps the sub-micron answer
    let off = Vec3::new(2000.0, -3000.0, 1500.0);
    let mut mesh = sphere(10.0, 32, 16);
    for p in &mut mesh.positions {
        *p += off;
    }
    let bvh = Bvh::build(&mesh).unwrap();
    let p = off + Vec3::new(0.0, 0.0, 30.0);
    let h = bvh.closest_point(p, f32::INFINITY).unwrap();
    assert!((h.distance - 20.0).abs() < 0.2);
}

/// Release-only timing: `cargo test -p dc-geom --release -- --ignored build_300k`.
#[test]
#[ignore = "timing test, run with --release"]
fn build_300k_triangles_under_150_ms() {
    // 388 x 388 grid cells = 301 088 triangles
    let mesh = terrain(388);
    assert!(mesh.tris.len() >= 300_000);
    let mut times = Vec::new();
    for _ in 0..7 {
        let t0 = std::time::Instant::now();
        let bvh = Bvh::build(&mesh).unwrap();
        times.push(t0.elapsed());
        assert_eq!(bvh.triangle_count(), mesh.tris.len());
    }
    times.sort();
    eprintln!(
        "bvh build of {} triangles: min {:?}, median {:?}, max {:?}",
        mesh.tris.len(),
        times[0],
        times[3],
        times[6]
    );
    assert!(times[3].as_millis() < 150, "median {:?}", times[3]);

    // query throughput on the same tree
    let bvh = Bvh::build(&mesh).unwrap();
    let mut rng = Lcg(21);
    let pts: Vec<Vec3> = (0..200_000)
        .map(|_| {
            Vec3::new(
                rng.range(0.0, 388.0),
                rng.range(0.0, 388.0),
                rng.range(-3.0, 3.0),
            )
        })
        .collect();
    let t0 = std::time::Instant::now();
    let r = bvh.closest_points(&pts, f32::INFINITY);
    eprintln!("200k closest-point queries: {:?}", t0.elapsed());
    assert!(r.iter().all(|h| h.is_some()));
    let dirs = vec![Vec3::NEG_Z; pts.len()];
    let origins: Vec<Vec3> = pts.iter().map(|p| Vec3::new(p.x, p.y, 10.0)).collect();
    let t0 = std::time::Instant::now();
    let r = bvh.ray_casts(&origins, &dirs, f32::INFINITY);
    eprintln!("200k ray casts: {:?}", t0.elapsed());
    assert!(r.iter().all(|h| h.is_some()));
}
