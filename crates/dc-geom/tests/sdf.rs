//! Narrow-band SDF (WP18): accuracy against analytic shapes, offsets,
//! closing, CSG, meshing quality and the dental-arch performance case.

use std::time::Instant;

use glam::{IVec3, Vec3};

use dc_geom::GeomError;
use dc_geom::bvh::Bvh;
use dc_geom::sdf::{CsgOp, SdfGrid, SdfParams};
use dc_mesh::TriMesh;

// ----- test shapes ----------------------------------------------------------

/// Closed UV ellipsoid, counter-clockwise outward faces.
fn ellipsoid(c: Vec3, r: Vec3, nu: u32, nv: u32) -> TriMesh {
    let mut pos = vec![c + Vec3::new(0.0, 0.0, r.z)];
    for j in 1..nv {
        let th = std::f32::consts::PI * j as f32 / nv as f32;
        for i in 0..nu {
            let ph = std::f32::consts::TAU * i as f32 / nu as f32;
            pos.push(c + r * Vec3::new(th.sin() * ph.cos(), th.sin() * ph.sin(), th.cos()));
        }
    }
    pos.push(c - Vec3::new(0.0, 0.0, r.z));
    let south = pos.len() as u32 - 1;
    let ring = |j: u32, i: u32| 1 + (j - 1) * nu + (i % nu);
    let mut tris = Vec::new();
    for i in 0..nu {
        tris.push([0, ring(1, i), ring(1, i + 1)]);
        tris.push([south, ring(nv - 1, i + 1), ring(nv - 1, i)]);
    }
    for j in 1..nv - 1 {
        for i in 0..nu {
            let (a, b, cc, d) = (
                ring(j, i),
                ring(j, i + 1),
                ring(j + 1, i),
                ring(j + 1, i + 1),
            );
            tris.push([a, cc, d]);
            tris.push([a, d, b]);
        }
    }
    TriMesh::new(pos, tris)
}

fn sphere(c: Vec3, r: f32) -> TriMesh {
    ellipsoid(c, Vec3::splat(r), 128, 64)
}

/// Axis-aligned box, counter-clockwise outward faces.
fn cuboid(lo: Vec3, hi: Vec3) -> TriMesh {
    let p = |i: u32| {
        Vec3::new(
            if i & 1 == 0 { lo.x } else { hi.x },
            if i & 2 == 0 { lo.y } else { hi.y },
            if i & 4 == 0 { lo.z } else { hi.z },
        )
    };
    let quads = [
        [0, 4, 6, 2],
        [1, 3, 7, 5],
        [0, 1, 5, 4],
        [2, 6, 7, 3],
        [0, 2, 3, 1],
        [4, 5, 7, 6],
    ];
    let tris = quads
        .iter()
        .flat_map(|q| [[q[0], q[1], q[2]], [q[0], q[2], q[3]]])
        .collect();
    TriMesh::new((0..8).map(p).collect(), tris)
}

fn small_box() -> TriMesh {
    cuboid(Vec3::ZERO, Vec3::ONE)
}

fn box_sdf(p: Vec3, lo: Vec3, hi: Vec3) -> f32 {
    let c = (lo + hi) * 0.5;
    let h = (hi - lo) * 0.5;
    let q = (p - c).abs() - h;
    q.max(Vec3::ZERO).length() + q.max_element().min(0.0)
}

fn concat(meshes: &[TriMesh]) -> TriMesh {
    let mut out = TriMesh::default();
    for m in meshes {
        let base = out.positions.len() as u32;
        out.positions.extend(&m.positions);
        out.tris.extend(m.tris.iter().map(|t| t.map(|v| v + base)));
    }
    out
}

struct Lcg(u64);

impl Lcg {
    fn f(&mut self) -> f32 {
        self.0 = self
            .0
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        ((self.0 >> 40) as f32) / ((1u64 << 24) as f32)
    }

    fn range(&mut self, lo: f32, hi: f32) -> f32 {
        lo + (hi - lo) * self.f()
    }

    fn point(&mut self, lo: Vec3, hi: Vec3) -> Vec3 {
        Vec3::new(
            self.range(lo.x, hi.x),
            self.range(lo.y, hi.y),
            self.range(lo.z, hi.z),
        )
    }
}

// ----- mesh checks ------------------------------------------------------------

fn signed_volume(m: &TriMesh) -> f64 {
    m.tris
        .iter()
        .map(|t| {
            let [a, b, c] = t.map(|i| m.positions[i as usize].as_dvec3());
            a.dot(b.cross(c)) / 6.0
        })
        .sum()
}

/// Closed, manifold, no degenerate faces, outward, `components` parts.
fn assert_clean(m: &TriMesh, components: usize) {
    let r = m.validate();
    assert!(r.is_watertight(), "not watertight: {r:?}");
    assert!(r.is_clean(), "not clean: {r:?}");
    assert_eq!(r.unreferenced_vertices, 0, "{r:?}");
    assert_eq!(r.components, components, "{r:?}");
    assert!(signed_volume(m) > 0.0, "faces not outward");
}

/// Symmetric vertex-to-surface Hausdorff distance through BVHs.
fn hausdorff(a: &TriMesh, b: &TriMesh) -> f32 {
    let directed = |from: &TriMesh, to: &TriMesh| {
        let bvh = Bvh::build(to).unwrap();
        bvh.closest_points(&from.positions, f32::INFINITY)
            .iter()
            .map(|h| h.unwrap().distance)
            .fold(0.0f32, f32::max)
    };
    directed(a, b).max(directed(b, a))
}

/// Analytic signed distance of a CSG case.
type Exact<'a> = Box<dyn Fn(Vec3) -> f32 + 'a>;

fn params(voxel_size: f32, band: f32) -> SdfParams {
    SdfParams { voxel_size, band }
}

// ----- tests ------------------------------------------------------------------

#[test]
fn sphere_distances_within_half_a_voxel() {
    let (c, r, vs) = (Vec3::new(0.31, -0.17, 0.43), 4.0, 0.2);
    let g = SdfGrid::from_mesh(&sphere(c, r), &params(vs, 1.0)).unwrap();
    let mut worst = 0.0f32;
    let mut n = 0;
    g.for_each_voxel(|ijk, v| {
        if v.abs() < g.band() {
            let exact = (g.voxel_position(ijk) - c).length() - r;
            worst = worst.max((v - exact).abs());
            n += 1;
        }
    });
    assert!(n > 10_000, "{n} band voxels");
    assert!(worst < 0.5 * vs, "worst error {worst}");
    assert!(g.sample(c) < 0.0 && g.sample(c) == -g.band());
    assert!(g.sample(c + Vec3::splat(10.0)) == g.band());
    let mut rng = Lcg(7);
    for _ in 0..500 {
        let p = c + rng.point(Vec3::splat(-5.0), Vec3::splat(5.0));
        let exact = (p - c).length() - r;
        if exact.abs() < 0.6 {
            assert!((g.sample(p) - exact).abs() < 0.5 * vs, "sample at {p}");
            let n = g.gradient(p);
            assert!(n.dot((p - c).normalize()) > 0.95, "gradient {n} at {p}");
        }
    }
}

#[test]
fn box_distances_within_half_a_voxel() {
    let (lo, hi, vs) = (
        Vec3::new(0.13, 0.27, -0.31),
        Vec3::new(6.07, 5.11, 3.93),
        0.15,
    );
    let g = SdfGrid::from_mesh(&cuboid(lo, hi), &params(vs, 0.9)).unwrap();
    let mut worst = 0.0f32;
    g.for_each_voxel(|ijk, v| {
        if v.abs() < g.band() {
            worst = worst.max((v - box_sdf(g.voxel_position(ijk), lo, hi)).abs());
        }
    });
    assert!(worst < 0.5 * vs, "worst error {worst}");
    assert!(g.interior_tile_count() > 0);
    assert!(g.contains((lo + hi) * 0.5));
}

#[test]
fn meshes_are_watertight_and_close_to_the_input() {
    let vs = 0.2;
    for (name, input, comps) in [
        ("sphere", sphere(Vec3::new(0.05, 0.1, 0.0), 4.0), 1),
        (
            "box",
            cuboid(Vec3::new(-2.03, -1.51, -1.17), Vec3::new(2.11, 1.49, 1.9)),
            1,
        ),
        (
            "two spheres",
            concat(&[
                sphere(Vec3::ZERO, 2.0),
                sphere(Vec3::new(6.0, 0.0, 0.0), 2.0),
            ]),
            2,
        ),
    ] {
        let g = SdfGrid::from_mesh(&input, &params(vs, 0.8)).unwrap();
        let m = g.to_mesh().unwrap();
        assert_clean(&m, comps);
        let h = hausdorff(&m, &input);
        assert!(h < vs, "{name}: Hausdorff {h}");
        let (vi, vm) = (signed_volume(&input), signed_volume(&m));
        assert!((vm / vi - 1.0).abs() < 0.02, "{name}: volume {vm} vs {vi}");
    }
}

#[test]
fn offsets_land_at_the_analytic_distance() {
    let (c, r, vs) = (Vec3::new(0.1, 0.2, 0.3), 3.0, 0.15);
    for d in [0.6f32, -0.6, 1.4] {
        let mut g = SdfGrid::from_mesh(&sphere(c, r), &params(vs, 1.0)).unwrap();
        g.offset(d).unwrap();
        let m = g.to_mesh().unwrap();
        assert_clean(&m, 1);
        let worst = m
            .positions
            .iter()
            .map(|p| ((*p - c).length() - (r + d)).abs())
            .fold(0.0f32, f32::max);
        assert!(worst < 0.5 * vs, "sphere offset {d}: worst {worst}");
        let mut v_worst = 0.0f32;
        g.for_each_voxel(|ijk, v| {
            if v.abs() < g.band() {
                let exact = (g.voxel_position(ijk) - c).length() - (r + d);
                v_worst = v_worst.max((v - exact).abs());
            }
        });
        assert!(
            v_worst < 0.5 * vs,
            "sphere offset {d}: band error {v_worst}"
        );
    }
    let (lo, hi) = (Vec3::new(0.07, 0.11, 0.05), Vec3::new(5.03, 3.97, 3.01));
    for d in [0.6f32, -0.5] {
        let mut g = SdfGrid::from_mesh(&cuboid(lo, hi), &params(vs, 1.0)).unwrap();
        g.offset(d).unwrap();
        let m = g.to_mesh().unwrap();
        assert_clean(&m, 1);
        let worst = m
            .positions
            .iter()
            .map(|p| (box_sdf(*p, lo, hi) - d).abs())
            .fold(0.0f32, f32::max);
        assert!(worst < 0.5 * vs, "box offset {d}: worst {worst}");
    }
}

#[test]
fn closing_fills_a_slot_narrower_than_twice_the_radius() {
    let vs = 0.1;
    let p = params(vs, 0.8);
    let (lo, hi) = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(8.0, 4.0, 4.0));
    let mut g = SdfGrid::from_mesh(&cuboid(lo, hi), &p).unwrap();
    // a narrow slot (0.45 mm) and a wide one (1.6 mm), 2 mm deep, across y
    let narrow = SdfGrid::from_mesh(
        &cuboid(Vec3::new(2.0, -1.0, 2.0), Vec3::new(2.45, 5.0, 5.0)),
        &p,
    )
    .unwrap();
    let wide = SdfGrid::from_mesh(
        &cuboid(Vec3::new(5.0, -1.0, 2.0), Vec3::new(6.6, 5.0, 5.0)),
        &p,
    )
    .unwrap();
    g.subtract(&narrow).unwrap();
    g.subtract(&wide).unwrap();
    let in_narrow = Vec3::new(2.225, 2.0, 3.0);
    let in_wide = Vec3::new(5.8, 2.0, 3.0);
    assert!(!g.contains(in_narrow) && !g.contains(in_wide));
    assert_clean(&g.to_mesh().unwrap(), 1);
    g.close(0.3).unwrap();
    assert!(
        g.contains(in_narrow),
        "narrow slot not filled: {}",
        g.sample(in_narrow)
    );
    // the ball of radius 0.3 dips about 0.1 mm into the 0.45 mm mouth
    assert!(g.contains(Vec3::new(2.225, 2.0, 3.75)));
    assert!(
        !g.contains(in_wide),
        "wide slot filled: {}",
        g.sample(in_wide)
    );
    // faces away from the slots stay where they were
    for q in [
        Vec3::new(0.0, 2.0, 1.0),
        Vec3::new(8.0, 2.0, 1.0),
        Vec3::new(4.0, 2.0, 4.0),
    ] {
        assert!(
            g.sample(q).abs() < 0.5 * vs,
            "face moved at {q}: {}",
            g.sample(q)
        );
    }
    assert_clean(&g.to_mesh().unwrap(), 1);
}

#[test]
fn csg_matches_the_analytic_booleans() {
    let vs = 0.15;
    let p = params(vs, 0.9);
    let (ca, cb, r) = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(2.5, 0.3, 0.1), 2.0);
    let a = SdfGrid::from_mesh(&sphere(ca, r), &p).unwrap();
    let b = SdfGrid::from_mesh(&sphere(cb, r), &p).unwrap();
    let da = |q: Vec3| (q - ca).length() - r;
    let db = |q: Vec3| (q - cb).length() - r;
    let cases: [(CsgOp, Exact); 3] = [
        (CsgOp::Union, Box::new(|q| da(q).min(db(q)))),
        (CsgOp::Intersection, Box::new(|q| da(q).max(db(q)))),
        (CsgOp::Difference, Box::new(|q| da(q).max(-db(q)))),
    ];
    for (op, exact) in cases {
        let mut g = a.clone();
        g.csg(&b, op).unwrap();
        let mut rng = Lcg(11);
        for _ in 0..4000 {
            let q = rng.point(Vec3::new(-3.0, -3.0, -3.0), Vec3::new(5.5, 3.0, 3.0));
            let e = exact(q);
            if e.abs() > vs {
                assert_eq!(g.contains(q), e < 0.0, "{op:?} at {q}");
            }
            if e.abs() < 0.5 {
                assert!((g.sample(q) - e).abs() < 0.5 * vs, "{op:?} value at {q}");
            }
        }
        let m = g.to_mesh().unwrap();
        assert_clean(&m, 1);
        let worst = m
            .positions
            .iter()
            .map(|q| exact(*q).abs())
            .fold(0.0f32, f32::max);
        assert!(worst < 0.5 * vs, "{op:?}: surface error {worst}");
    }
    // smooth union: fills the crease, never removes material
    let mut s = a.clone();
    s.csg(&b, CsgOp::SmoothUnion(0.8)).unwrap();
    let crease = Vec3::new(1.25, 0.15 + 1.5, 0.05);
    assert!(s.sample(crease) < da(crease).min(db(crease)) - 0.05);
    let mut rng = Lcg(5);
    for _ in 0..2000 {
        let q = rng.point(Vec3::splat(-3.0), Vec3::new(5.5, 3.0, 3.0));
        if da(q).min(db(q)) < -vs {
            assert!(s.contains(q));
        }
    }
    assert_clean(&s.to_mesh().unwrap(), 1);
    // different grids are refused
    let other = SdfGrid::from_mesh(&sphere(ca, r), &params(0.2, 0.9)).unwrap();
    assert!(matches!(s.union(&other), Err(GeomError::InvalidInput(_))));
}

#[test]
fn fill_interior_fills_an_enclosed_cavity() {
    let p = params(0.15, 0.9);
    let mut g = SdfGrid::from_mesh(&sphere(Vec3::ZERO, 4.0), &p).unwrap();
    let hole = SdfGrid::from_mesh(&sphere(Vec3::ZERO, 2.0), &p).unwrap();
    g.subtract(&hole).unwrap();
    assert!(!g.contains(Vec3::ZERO));
    assert_clean(&g.to_mesh().unwrap(), 2);
    g.fill_interior().unwrap();
    assert!(g.contains(Vec3::ZERO), "{}", g.sample(Vec3::ZERO));
    assert!(g.contains(Vec3::new(1.9, 0.0, 0.0)));
    assert!(g.sample(Vec3::new(4.0, 0.0, 0.0)).abs() < 0.1);
    assert_clean(&g.to_mesh().unwrap(), 1);
}

#[test]
fn overlapping_parts_give_their_union() {
    // a soup of overlapping closed parts, as the teeth of an arch
    let parts = concat(&[
        sphere(Vec3::ZERO, 2.0),
        sphere(Vec3::new(2.2, 0.0, 0.0), 2.0),
        ellipsoid(Vec3::new(1.1, 0.0, -1.0), Vec3::new(3.0, 1.0, 1.0), 96, 48),
    ]);
    let vs = 0.15;
    let mut g = SdfGrid::from_mesh(&parts, &params(vs, 0.9)).unwrap();
    let inside = |q: Vec3| {
        q.length() < 2.0
            || (q - Vec3::new(2.2, 0.0, 0.0)).length() < 2.0
            || ((q - Vec3::new(1.1, 0.0, -1.0)) / Vec3::new(3.0, 1.0, 1.0)).length() < 1.0
    };
    // no buried surface: distances inside are those of the union
    let d = (Vec3::new(1.1, 0.0, 0.0) - Vec3::new(2.2, 0.0, 0.0)).length();
    assert!((g.sample(Vec3::new(1.1, 0.0, 0.0)) + g.band().min(2.0 - d)).abs() < 0.5 * vs);
    let m = g.to_mesh().unwrap();
    assert_clean(&m, 1);
    let mut rng = Lcg(3);
    let mut wrong = 0;
    for v in &m.positions {
        // every vertex lies on the outer boundary of the union
        let out = (0..6).any(|k| {
            let mut o = Vec3::ZERO;
            o[k / 2] = if k % 2 == 0 { 0.3 } else { -0.3 };
            !inside(*v + o)
        });
        if !out {
            wrong += 1;
        }
    }
    assert_eq!(wrong, 0, "vertices of a buried surface");
    let _ = rng.f();
    // eroding does not open the buried surfaces
    g.offset(-0.3).unwrap();
    assert_clean(&g.to_mesh().unwrap(), 1);
    assert!(g.contains(Vec3::new(1.1, 0.0, 0.0)));
}

#[test]
fn small_holes_in_a_scan_stay_closed() {
    let mut m = sphere(Vec3::ZERO, 3.0);
    // open the north cap: the first ring of triangles around the pole
    // (a hole of about 0.15 mm across, under a voxel)
    m.tris.drain(0..1);
    assert!(m.validate().boundary_edges > 0);
    let g = SdfGrid::from_mesh(&m, &params(0.2, 0.8)).unwrap();
    assert!(g.contains(Vec3::ZERO));
    let out = g.to_mesh().unwrap();
    assert_clean(&out, 1);
    assert!(hausdorff(&out, &sphere(Vec3::ZERO, 3.0)) < 0.2);
}

#[test]
fn output_is_deterministic() {
    let input = concat(&[
        sphere(Vec3::new(0.1, 0.0, 0.0), 2.0),
        cuboid(Vec3::new(1.0, -1.0, -1.0), Vec3::new(4.0, 1.3, 1.1)),
    ]);
    let run = || {
        let mut g = SdfGrid::from_mesh(&input, &params(0.12, 0.8)).unwrap();
        g.offset(0.25).unwrap();
        g.to_mesh().unwrap()
    };
    let a = run();
    let b = rayon::ThreadPoolBuilder::new()
        .num_threads(3)
        .build()
        .unwrap()
        .install(run);
    assert_eq!(a, b);
}

#[test]
fn iso_levels_and_empty_results() {
    let p = params(0.2, 1.0);
    let g = SdfGrid::from_mesh(&sphere(Vec3::ZERO, 3.0), &p).unwrap();
    let m = g.to_mesh_at(0.5).unwrap();
    assert_clean(&m, 1);
    let worst = m
        .positions
        .iter()
        .map(|q| (q.length() - 3.5).abs())
        .fold(0.0f32, f32::max);
    assert!(worst < 0.1, "{worst}");
    let mut e = g.clone();
    e.offset(-3.5).unwrap();
    assert!(e.to_mesh().unwrap().is_empty());
    assert_eq!(e.block_count(), 0);
    assert!(SdfGrid::empty(&p).unwrap().to_mesh().unwrap().is_empty());
    assert_eq!(g.voxel(IVec3::new(1000, 0, 0)), g.band());
}

#[test]
fn invalid_input_is_refused() {
    let bad = |r: dc_geom::Result<SdfGrid>| matches!(r, Err(GeomError::InvalidInput(_)));
    assert!(bad(SdfGrid::from_mesh(
        &TriMesh::default(),
        &SdfParams::default()
    )));
    assert!(bad(SdfGrid::from_mesh(&small_box(), &params(0.0, 1.0))));
    assert!(bad(SdfGrid::from_mesh(
        &small_box(),
        &params(0.1, f32::NAN)
    )));
    let mut oob = small_box();
    oob.tris[0][0] = 99;
    assert!(SdfGrid::from_mesh(&oob, &SdfParams::default()).is_err());
    let mut g = SdfGrid::from_mesh(&small_box(), &params(0.1, 0.5)).unwrap();
    assert!(g.offset(f32::INFINITY).is_err());
    assert!(g.close(-1.0).is_err());
    assert!(g.to_mesh_at(f32::NAN).is_err());
}

// ----- performance --------------------------------------------------------------

/// Synthetic arch: tooth-like ellipsoids along a parabola over a horseshoe of
/// gum ellipsoids, about 60 × 50 × 25 mm and 200k triangles.
fn arch() -> TriMesh {
    let mut parts = Vec::new();
    let n = 14;
    for i in 0..n {
        let s = -1.0 + 2.0 * (i as f32 + 0.5) / n as f32;
        let x = 28.0 * s;
        let y = 44.0 * (1.0 - s * s) - 22.0;
        let tangent = Vec3::new(28.0, -88.0 * s, 0.0).normalize();
        let molar = s.abs() > 0.55;
        let r = if molar {
            Vec3::new(5.2, 5.0, 5.5)
        } else {
            Vec3::new(4.2, 3.4, 5.5)
        };
        let c = Vec3::new(x, y, 16.0);
        let tooth = ellipsoid(Vec3::ZERO, r, 84, 42);
        let gum = ellipsoid(Vec3::ZERO, Vec3::new(5.0, 7.0, 7.0), 84, 42);
        let rot = glam::Quat::from_rotation_arc(Vec3::X, tangent);
        for (m, centre) in [(tooth, c), (gum, Vec3::new(x, y, 7.0))] {
            let mut m = m;
            for p in &mut m.positions {
                *p = centre + rot * *p;
            }
            parts.push(m);
        }
    }
    concat(&parts)
}

#[test]
#[ignore = "performance: run in release with --ignored --nocapture"]
fn arch_mesh_to_sdf_offset_and_mesh() {
    let mesh = arch();
    let bb = mesh.aabb().unwrap();
    println!(
        "arch: {} triangles, {:.1} x {:.1} x {:.1} mm, {} threads",
        mesh.face_count(),
        bb.size().x,
        bb.size().y,
        bb.size().z,
        rayon::current_num_threads()
    );
    let t0 = Instant::now();
    let mut g = SdfGrid::from_mesh(&mesh, &params(0.15, 0.6 + 3.0 * 0.15)).unwrap();
    let t1 = Instant::now();
    println!(
        "from_mesh: {:.3} s, {} blocks, {} tiles, {} band voxels, {:.1} MB",
        (t1 - t0).as_secs_f64(),
        g.block_count(),
        g.interior_tile_count(),
        g.band_voxel_count(),
        g.approx_bytes() as f64 / 1e6
    );
    g.offset(0.6).unwrap();
    let t2 = Instant::now();
    println!(
        "offset +0.6: {:.3} s, {} blocks",
        (t2 - t1).as_secs_f64(),
        g.block_count()
    );
    let out = g.to_mesh().unwrap();
    let t3 = Instant::now();
    println!(
        "to_mesh: {:.3} s, {} triangles",
        (t3 - t2).as_secs_f64(),
        out.face_count()
    );
    println!("total: {:.3} s", (t3 - t0).as_secs_f64());
    let r = out.validate();
    assert!(r.is_watertight() && r.is_clean(), "{r:?}");
    // besides the arch, at most a few one-voxel voids where the dilation
    // of three parts almost closes (the sampled field is positive there)
    let mut sizes: Vec<usize> = out.connected_components().iter().map(Vec::len).collect();
    sizes.sort_unstable();
    println!("components (faces): {sizes:?}");
    assert!(
        sizes[..sizes.len() - 1].iter().all(|&n| n <= 16),
        "{sizes:?}"
    );
    assert!((t3 - t0).as_secs_f64() < 10.0);
    // no two triangles of the mesher output cross
    assert_eq!(crossings(&out), 0);
}

// ----- regressions: walls on voxel centres, pockets, shells, sweeps ---------

/// Pairs of triangles that share no vertex and cross (an edge of one meets
/// the other).
fn crossings(m: &TriMesh) -> usize {
    let bvh = Bvh::build(m).unwrap();
    let corner = |t: [u32; 3]| t.map(|i| m.positions[i as usize].as_dvec3());
    let edge_hits = |p: glam::DVec3, q: glam::DVec3, t: &[glam::DVec3; 3]| {
        let d = q - p;
        let (e1, e2) = (t[1] - t[0], t[2] - t[0]);
        let pv = d.cross(e2);
        let det = e1.dot(pv);
        if det.abs() <= 1e-18 {
            return false;
        }
        let s = p - t[0];
        let u = s.dot(pv) / det;
        let qv = s.cross(e1);
        let w = d.dot(qv) / det;
        let h = e2.dot(qv) / det;
        u >= 0.0 && w >= 0.0 && u + w <= 1.0 && (0.0..=1.0).contains(&h)
    };
    let mut n = 0;
    for (i, &ta) in m.tris.iter().enumerate() {
        let a = corner(ta);
        let lo = a[0].min(a[1]).min(a[2]).as_vec3();
        let hi = a[0].max(a[1]).max(a[2]).as_vec3();
        for j in bvh.overlap_aabb(lo, hi) {
            let tb = m.tris[j as usize];
            if j as usize <= i || ta.iter().any(|v| tb.contains(v)) {
                continue;
            }
            let b = corner(tb);
            if (0..3)
                .any(|k| edge_hits(a[k], a[(k + 1) % 3], &b) || edge_hits(b[k], b[(k + 1) % 3], &a))
            {
                n += 1;
            }
        }
    }
    n
}

#[test]
fn walls_through_voxel_centres_stay_inside() {
    // two overlapping boxes; the inner wall of each lies on voxel centres
    // (an exact multiple of the voxel), where the distance is exactly zero
    let vs = 0.125;
    let parts = concat(&[
        cuboid(Vec3::ZERO, Vec3::new(2.0, 2.0, 2.0)),
        cuboid(Vec3::new(1.0, 0.0, 0.0), Vec3::new(3.0, 2.0, 2.0)),
    ]);
    let g = SdfGrid::from_mesh(&parts, &params(vs, 4.0 * vs)).unwrap();
    for (j, k) in [(4, 4), (8, 8), (12, 4)] {
        for i in [8, 16] {
            let v = g.voxel(IVec3::new(i, j, k));
            assert!(v < 0.0, "voxel ({i}, {j}, {k}) on an inner wall reads {v}");
        }
    }
    let m = g.to_mesh().unwrap();
    assert_clean(&m, 1);
    let lo = Vec3::ZERO;
    let hi = Vec3::new(3.0, 2.0, 2.0);
    for p in &m.positions {
        assert!(box_sdf(*p, lo, hi).abs() < 0.01, "vertex {p} off the union");
    }
}

/// A box with a pocket: a floor, four walls and a lid with a slit of width
/// `slit` along y, built from overlapping closed boxes and a prism standing
/// in a corner of the pocket (a soup like the parts of a model).
fn pocket_soup(slit: f32) -> TriMesh {
    let (w, h) = (4.0, 3.0);
    let s = 0.5 * (w - slit);
    let prism = TriMesh::new(
        vec![
            Vec3::new(0.3, 0.3, 0.3),
            Vec3::new(1.4, 0.3, 0.3),
            Vec3::new(0.3, 1.4, 0.3),
            Vec3::new(0.3, 0.3, 2.7),
            Vec3::new(1.4, 0.3, 2.7),
            Vec3::new(0.3, 1.4, 2.7),
        ],
        vec![
            [0, 2, 1],
            [3, 4, 5],
            [0, 1, 4],
            [0, 4, 3],
            [1, 2, 5],
            [1, 5, 4],
            [2, 0, 3],
            [2, 3, 5],
        ],
    );
    concat(&[
        cuboid(Vec3::ZERO, Vec3::new(w, w, 0.6)),
        cuboid(Vec3::ZERO, Vec3::new(0.6, w, h)),
        cuboid(Vec3::new(w - 0.6, 0.0, 0.0), Vec3::new(w, w, h)),
        cuboid(Vec3::ZERO, Vec3::new(w, 0.6, h)),
        cuboid(Vec3::new(0.0, w - 0.6, 0.0), Vec3::new(w, w, h)),
        cuboid(Vec3::new(0.0, 0.0, h - 0.6), Vec3::new(s, w, h)),
        cuboid(Vec3::new(w - s, 0.0, h - 0.6), Vec3::new(w, w, h)),
        prism,
    ])
}

#[test]
fn blockout_closing_fills_a_pocket_behind_a_narrow_mouth() {
    let (vs, r) = (0.1, 0.3);
    let centre = Vec3::new(2.0, 2.0, 1.5);
    let mouth = Vec3::new(2.0, 2.0, 2.7);
    // the union of the parts: an open pocket
    let soup = pocket_soup(0.4);
    let union = SdfGrid::from_mesh(&soup, &params(vs, 0.6)).unwrap();
    assert!(!union.contains(centre) && !union.contains(mouth));
    // the add-on's closing: the slit (narrower than 2r) sealed, the pocket
    // behind it filled, one surface around the outside only
    let g = SdfGrid::from_mesh_closed(&soup, &params(vs, 0.6), r).unwrap();
    assert!(g.contains(centre), "pocket left open: {}", g.sample(centre));
    assert!(g.contains(mouth), "slit left open: {}", g.sample(mouth));
    let m = g.to_mesh().unwrap();
    assert_clean(&m, 1);
    // every vertex on the outer box (the ball dips 0.08 mm into the slit)
    for p in &m.positions {
        assert!(
            box_sdf(*p, Vec3::ZERO, Vec3::new(4.0, 4.0, 3.0)).abs() < 0.1,
            "vertex {p} inside the solid"
        );
    }
    // outside the closed surface the values are the exact distance to it
    for q in [Vec3::new(-0.4, 2.0, 1.5), Vec3::new(2.0, 2.0, 3.45)] {
        let d = box_sdf(q, Vec3::ZERO, Vec3::new(4.0, 4.0, 3.0));
        assert!(
            (g.sample(q) - d).abs() < 0.5 * vs,
            "{q}: {} vs {d}",
            g.sample(q)
        );
    }
    // the exact morphological closing keeps the pocket (a ball of radius r fits)
    let mut exact = union.clone();
    exact.close(r).unwrap();
    assert!(!exact.contains(centre));
    // a slit wider than 2r stays open
    let open = SdfGrid::from_mesh_closed(&pocket_soup(0.9), &params(vs, 0.6), r).unwrap();
    assert!(!open.contains(centre) && !open.contains(mouth));
    assert_clean(&open.to_mesh().unwrap(), 1);
}

#[test]
fn shell_and_shift_read_the_level_sets() {
    let (vs, r) = (0.1, 2.0);
    let g = SdfGrid::from_mesh(&sphere(Vec3::ZERO, r), &params(vs, 1.2)).unwrap();
    let s = g.shell(0.2, 0.8).unwrap();
    let m = s.to_mesh().unwrap();
    assert_clean(&m, 2);
    for p in &m.positions {
        let d = p.length() - r;
        assert!(
            (d - 0.2).abs() < 0.03 || (d - 0.8).abs() < 0.03,
            "shell vertex {d} mm from the sphere"
        );
    }
    assert!(s.contains(Vec3::new(r + 0.5, 0.0, 0.0)));
    assert!(!s.contains(Vec3::ZERO) && !s.contains(Vec3::new(r + 1.0, 0.0, 0.0)));
    assert!(g.shell(0.2, 1.2).is_err() && g.shell(0.5, 0.2).is_err());
    let mut moved = g.clone();
    moved.shift(0.3).unwrap();
    for p in &moved.to_mesh().unwrap().positions {
        assert!((p.length() - r - 0.3).abs() < 0.03);
    }
}

#[test]
fn sweep_down_fills_what_the_solid_hides_from_above() {
    // a mushroom: a cap on a stem on a base
    let vs = 0.1;
    let parts = concat(&[
        cuboid(Vec3::new(-3.0, -3.0, 0.0), Vec3::new(3.0, 3.0, 0.6)),
        cuboid(Vec3::new(-0.5, -0.5, 0.3), Vec3::new(0.5, 0.5, 2.6)),
        cuboid(Vec3::new(-1.5, -1.5, 2.4), Vec3::new(1.5, 1.5, 3.0)),
    ]);
    let mut g = SdfGrid::from_mesh(&parts, &params(vs, 0.6)).unwrap();
    let under = Vec3::new(1.0, 0.3, 1.5);
    assert!(!g.contains(under));
    g.sweep_down(0.0).unwrap();
    assert!(g.contains(under));
    assert!(!g.contains(Vec3::new(2.0, 0.3, 1.5)));
    assert!(!g.contains(Vec3::new(0.3, 0.3, 3.2)));
    assert!(!g.contains(Vec3::new(1.0, 0.3, -0.2)));
    let m = g.to_mesh().unwrap();
    assert_clean(&m, 1);
    // the swept solid is the cap's footprint over the base: every vertex on it
    for p in &m.positions {
        let d = box_sdf(*p, Vec3::new(-3.0, -3.0, 0.0), Vec3::new(3.0, 3.0, 0.6)).min(box_sdf(
            *p,
            Vec3::new(-1.5, -1.5, 0.0),
            Vec3::new(1.5, 1.5, 3.0),
        ));
        assert!(d.abs() < 0.03, "vertex {p} {d} mm off the sweep");
    }
}

#[test]
fn meshes_do_not_cross_themselves() {
    // overlapping parts, offsets and a closing: no two triangles cross
    let parts = concat(&[
        sphere(Vec3::ZERO, 2.0),
        sphere(Vec3::new(2.2, 0.0, 0.3), 1.7),
        ellipsoid(Vec3::new(1.1, 0.0, -1.0), Vec3::new(3.0, 1.0, 1.0), 96, 48),
        pocket_soup(0.4),
    ]);
    let mut g = SdfGrid::from_mesh(&parts, &params(0.1, 0.6)).unwrap();
    assert_eq!(crossings(&g.to_mesh().unwrap()), 0);
    g.offset(0.25).unwrap();
    assert_eq!(crossings(&g.to_mesh().unwrap()), 0);
    let c = SdfGrid::from_mesh_closed(&parts, &params(0.1, 0.6), 0.3).unwrap();
    assert_eq!(crossings(&c.to_mesh().unwrap()), 0);
}
