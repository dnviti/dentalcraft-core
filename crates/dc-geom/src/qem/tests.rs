use std::collections::HashMap;

use glam::{DVec3, Vec3};

use super::*;

/// Unit icosphere with `subdiv` subdivisions (20 * 4^subdiv triangles).
fn icosphere(subdiv: usize) -> (Vec<DVec3>, Vec<[u32; 3]>) {
    let t = (1.0 + 5f64.sqrt()) / 2.0;
    let mut v: Vec<DVec3> = [
        (-1.0, t, 0.0),
        (1.0, t, 0.0),
        (-1.0, -t, 0.0),
        (1.0, -t, 0.0),
        (0.0, -1.0, t),
        (0.0, 1.0, t),
        (0.0, -1.0, -t),
        (0.0, 1.0, -t),
        (t, 0.0, -1.0),
        (t, 0.0, 1.0),
        (-t, 0.0, -1.0),
        (-t, 0.0, 1.0),
    ]
    .iter()
    .map(|&(x, y, z)| DVec3::new(x, y, z).normalize())
    .collect();
    let mut f: Vec<[u32; 3]> = vec![
        [0, 11, 5],
        [0, 5, 1],
        [0, 1, 7],
        [0, 7, 10],
        [0, 10, 11],
        [1, 5, 9],
        [5, 11, 4],
        [11, 10, 2],
        [10, 7, 6],
        [7, 1, 8],
        [3, 9, 4],
        [3, 4, 2],
        [3, 2, 6],
        [3, 6, 8],
        [3, 8, 9],
        [4, 9, 5],
        [2, 4, 11],
        [6, 2, 10],
        [8, 6, 7],
        [9, 8, 1],
    ];
    for _ in 0..subdiv {
        let mut cache = HashMap::new();
        let mut nf = Vec::with_capacity(f.len() * 4);
        let mut mid = |a: u32, b: u32, v: &mut Vec<DVec3>| -> u32 {
            *cache.entry((a.min(b), a.max(b))).or_insert_with(|| {
                v.push((v[a as usize] + v[b as usize]).normalize());
                (v.len() - 1) as u32
            })
        };
        for [a, b, c] in f {
            let ab = mid(a, b, &mut v);
            let bc = mid(b, c, &mut v);
            let ca = mid(c, a, &mut v);
            nf.extend([[a, ab, ca], [b, bc, ab], [c, ca, bc], [ab, bc, ca]]);
        }
        f = nf;
    }
    (v, f)
}

/// Sphere of radius 10 mm.
fn sphere(subdiv: usize) -> TriMesh {
    let (v, f) = icosphere(subdiv);
    TriMesh::new(v.iter().map(|p| (*p * 10.0).as_vec3()).collect(), f)
}

/// Deterministic uniform numbers in [0, 1).
struct Lcg(u64);
impl Lcg {
    fn next(&mut self) -> f64 {
        self.0 = self
            .0
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        (self.0 >> 11) as f64 / (1u64 << 53) as f64
    }
}

/// Sphere with radial noise of `amp` mm.
fn noisy_sphere(subdiv: usize, amp: f64) -> TriMesh {
    let (v, f) = icosphere(subdiv);
    let mut rng = Lcg(3);
    let pos = v
        .iter()
        .map(|p| (*p * (10.0 + amp * (2.0 * rng.next() - 1.0))).as_vec3())
        .collect();
    TriMesh::new(pos, f)
}

/// Open height field `n x n` over 30 x 30 mm: waves plus noise (a scan-like
/// open surface with one boundary loop).
fn noisy_surface(n: usize, noise: f64) -> TriMesh {
    let mut rng = Lcg(11);
    let mut pos = Vec::with_capacity(n * n);
    for j in 0..n {
        for i in 0..n {
            let x = 30.0 * i as f64 / (n - 1) as f64;
            let y = 30.0 * j as f64 / (n - 1) as f64;
            let z = 1.5 * (x / 4.0).sin() * (y / 5.0).cos() + noise * (2.0 * rng.next() - 1.0);
            pos.push(Vec3::new(x as f32, y as f32, z as f32));
        }
    }
    let mut tris = Vec::new();
    for j in 0..n - 1 {
        for i in 0..n - 1 {
            let a = (j * n + i) as u32;
            let b = a + 1;
            let c = a + n as u32;
            let d = c + 1;
            if (i + j) % 2 == 0 {
                tris.extend([[a, b, d], [a, d, c]]);
            } else {
                tris.extend([[a, b, c], [b, d, c]]);
            }
        }
    }
    TriMesh::new(pos, tris)
}

/// Largest distance from the vertices of `from` to the surface of `to` (BVH,
/// exact in f64 for the stored f32 geometry).
fn directed(from: &TriMesh, to: &TriMesh) -> f64 {
    let bvh = Bvh::build(to).unwrap();
    from.positions
        .iter()
        .map(|p| {
            let h = bvh.closest_point(*p, f32::INFINITY).unwrap();
            h.point.as_dvec3().distance(p.as_dvec3())
        })
        .fold(0.0, f64::max)
}

fn hausdorff(a: &TriMesh, b: &TriMesh) -> f64 {
    directed(a, b).max(directed(b, a))
}

/// Manifold, no degenerate or unreferenced vertex, and the same topology.
fn assert_clean(input: &TriMesh, out: &TriMesh) {
    let ri = input.validate();
    let ro = out.validate();
    assert!(ro.is_clean(), "{ro:?}");
    assert_eq!(ro.unreferenced_vertices, 0, "{ro:?}");
    assert_eq!(ro.boundary_loops, ri.boundary_loops, "{ro:?}");
    assert_eq!(ro.components, ri.components, "{ro:?}");
    // closed orientable surfaces keep their Euler characteristic
    let euler = |r: &dc_mesh::ValidationReport, m: &TriMesh| {
        let e = m.edge_adjacency().edge_count() as i64;
        r.vertices as i64 - e + r.faces as i64
    };
    assert_eq!(euler(&ro, out), euler(&ri, input));
}

fn assert_vertex_map(input: &TriMesh, d: &Decimation) {
    assert_eq!(d.vertex_map.len(), input.vertex_count());
    assert!(
        d.vertex_map
            .iter()
            .all(|&v| (v as usize) < d.mesh.vertex_count())
    );
}

#[test]
fn sphere_hits_face_target() {
    let mesh = sphere(6); // 81920 faces
    for target in [20_000, 8_000, 2_000] {
        let d = decimate(&mesh, &DecimateParams::to_faces(target)).unwrap();
        let n = d.mesh.face_count();
        assert!(n.abs_diff(target) <= target / 100, "{n} vs {target}");
        assert_clean(&mesh, &d.mesh);
        assert_vertex_map(&mesh, &d);
        // radial deviation stays at the chord error
        let worst = d
            .mesh
            .positions
            .iter()
            .map(|p| (p.as_dvec3().length() - 10.0).abs())
            .fold(0.0, f64::max);
        assert!(worst < 0.05, "{target}: {worst}");
        // no flipped triangle: every normal points outwards
        for f in 0..d.mesh.face_count() {
            let c: Vec3 = d.mesh.triangle(f).iter().sum::<Vec3>() / 3.0;
            assert!(d.mesh.face_normal_raw(f).dot(c) > 0.0);
        }
    }
}

#[test]
fn ratio_matches_blender_decimate() {
    let mesh = sphere(5); // 20480 faces
    let d = decimate(&mesh, &DecimateParams::to_ratio(0.25)).unwrap();
    assert!(d.mesh.face_count().abs_diff(5120) <= 51);
    assert_clean(&mesh, &d.mesh);
}

#[test]
fn sphere_within_tolerance() {
    let mesh = sphere(5);
    let tol = 0.01;
    let d = decimate(&mesh, &DecimateParams::to_tolerance(tol)).unwrap();
    let h = hausdorff(&mesh, &d.mesh);
    assert!(h <= tol + 1e-5, "hausdorff {h}");
    assert!(
        d.mesh.face_count() < mesh.face_count() / 2,
        "{}",
        d.mesh.face_count()
    );
    assert_clean(&mesh, &d.mesh);
}

#[test]
fn noisy_sphere_within_tolerance() {
    let mesh = noisy_sphere(5, 0.02);
    let tol = 0.05;
    let d = decimate(&mesh, &DecimateParams::to_tolerance(tol)).unwrap();
    let h = hausdorff(&mesh, &d.mesh);
    assert!(h <= tol + 1e-5, "hausdorff {h}");
    assert!(d.mesh.face_count() < mesh.face_count() / 2);
    assert_clean(&mesh, &d.mesh);
}

/// Boundary vertices of `m` and the boundary edges as segments.
fn boundary_of(m: &TriMesh) -> (Vec<DVec3>, Vec<(DVec3, DVec3)>) {
    let adj = m.edge_adjacency();
    let mut pts = Vec::new();
    let mut segs = Vec::new();
    for (e, faces) in adj.iter() {
        if faces.len() == 1 {
            let (a, b) = (
                m.positions[e.0 as usize].as_dvec3(),
                m.positions[e.1 as usize].as_dvec3(),
            );
            pts.push(a);
            pts.push(b);
            segs.push((a, b));
        }
    }
    (pts, segs)
}

fn to_segments(pts: &[DVec3], segs: &[(DVec3, DVec3)]) -> f64 {
    pts.iter()
        .map(|p| {
            segs.iter()
                .map(|(a, b)| segment_dist2(*p, *a, *b))
                .fold(f64::INFINITY, f64::min)
                .sqrt()
        })
        .fold(0.0, f64::max)
}

#[test]
fn open_noisy_surface_keeps_boundary() {
    let mesh = noisy_surface(120, 0.02); // 28322 faces
    let (bp, bs) = boundary_of(&mesh);

    // face target
    let d = decimate(&mesh, &DecimateParams::to_faces(3_000)).unwrap();
    assert!(
        d.mesh.face_count().abs_diff(3_000) <= 30,
        "{}",
        d.mesh.face_count()
    );
    assert_clean(&mesh, &d.mesh);
    let (op, os) = boundary_of(&d.mesh);
    assert!(to_segments(&op, &bs) < 0.05, "{}", to_segments(&op, &bs));
    assert!(to_segments(&bp, &os) < 0.05, "{}", to_segments(&bp, &os));
    // the boundary stays on the boundary
    for (i, &o) in d.vertex_map.iter().enumerate() {
        let on_boundary = bp.contains(&mesh.positions[i].as_dvec3());
        if on_boundary {
            let q = d.mesh.positions[o as usize].as_dvec3();
            assert!(op.contains(&q));
        }
    }

    // tolerance
    let tol = 0.05;
    let d = decimate(&mesh, &DecimateParams::to_tolerance(tol)).unwrap();
    let h = hausdorff(&mesh, &d.mesh);
    assert!(h <= tol + 1e-5, "hausdorff {h}");
    assert_clean(&mesh, &d.mesh);
    let (op, os) = boundary_of(&d.mesh);
    assert!(to_segments(&op, &bs) <= tol + 1e-5);
    assert!(to_segments(&bp, &os) <= tol + 1e-5);
    assert!(d.mesh.face_count() < mesh.face_count() / 2);
}

#[test]
fn free_boundary_still_manifold() {
    let mesh = noisy_surface(60, 0.02);
    let p = DecimateParams {
        preserve_boundary: false,
        ..DecimateParams::to_faces(500)
    };
    let d = decimate(&mesh, &p).unwrap();
    assert!(d.mesh.face_count().abs_diff(500) <= 5);
    assert_clean(&mesh, &d.mesh);
}

#[test]
fn deterministic() {
    let mesh = noisy_sphere(5, 0.05);
    let p = DecimateParams::to_faces(3_000);
    let a = decimate(&mesh, &p).unwrap();
    let b = decimate(&mesh, &p).unwrap();
    assert_eq!(a, b);
    let p = DecimateParams::to_tolerance(0.05);
    assert_eq!(decimate(&mesh, &p).unwrap(), decimate(&mesh, &p).unwrap());
}

/// A mesh over [`PARALLEL_MIN_FACES`] without a face target is decimated in
/// cells: the tolerance, the border and the topology hold as in a
/// sequential run, and the result does not depend on the threads.
#[test]
fn parallel_cells_keep_tolerance_and_topology() {
    let mesh = noisy_surface(185, 0.02); // 67712 faces
    assert!(mesh.face_count() >= PARALLEL_MIN_FACES);
    let (bp, bs) = boundary_of(&mesh);
    let tol = 0.05;
    let p = DecimateParams::to_tolerance(tol);
    let run = |threads: usize| {
        rayon::ThreadPoolBuilder::new()
            .num_threads(threads)
            .build()
            .unwrap()
            .install(|| decimate(&mesh, &p).unwrap())
    };
    let d = run(4);
    assert_eq!(d, run(1));
    let h = hausdorff(&mesh, &d.mesh);
    assert!(h <= tol + 1e-5, "hausdorff {h}");
    assert_clean(&mesh, &d.mesh);
    assert_vertex_map(&mesh, &d);
    let (op, os) = boundary_of(&d.mesh);
    assert!(to_segments(&op, &bs) <= tol + 1e-5);
    assert!(to_segments(&bp, &os) <= tol + 1e-5);
    assert!(
        d.mesh.face_count() < mesh.face_count() / 4,
        "{}",
        d.mesh.face_count()
    );

    // closed: the Euler characteristic holds across the cell seams
    let sphere = noisy_sphere(6, 0.01); // 81920 faces
    let d = decimate(&sphere, &p).unwrap();
    let h = hausdorff(&sphere, &d.mesh);
    assert!(h <= tol + 1e-5, "hausdorff {h}");
    assert_clean(&sphere, &d.mesh);
    assert!(
        d.mesh.face_count() < sphere.face_count() / 4,
        "{}",
        d.mesh.face_count()
    );
}

/// The surface bound keeps the edge midpoints and face centroids within
/// the tolerance, the edge cap every edge, and the normal cap keeps more
/// faces; `decimate_against` holds the bound against the reference.
#[test]
fn surface_bound_normal_and_edge_caps() {
    let mesh = noisy_surface(90, 0.01);
    let tol = 0.05;
    let p = DecimateParams {
        surface_tolerance: true,
        max_normal_angle_deg: 10.0,
        max_edge: 1.5,
        ..DecimateParams::to_tolerance(tol)
    };
    let d = decimate(&mesh, &p).unwrap();
    assert_clean(&mesh, &d.mesh);
    assert!(hausdorff(&mesh, &d.mesh) <= tol + 1e-5);
    let bvh = Bvh::build(&mesh).unwrap();
    let out = &d.mesh;
    for t in &out.tris {
        let [a, b, c] = t.map(|v| out.positions[v as usize]);
        for q in [
            (a + b) * 0.5,
            (b + c) * 0.5,
            (c + a) * 0.5,
            (a + b + c) / 3.0,
        ] {
            let h = bvh.closest_point(q, f32::INFINITY).unwrap();
            assert!(f64::from(h.distance) <= tol + 1e-5, "{}", h.distance);
        }
        for (x, y) in [(a, b), (b, c), (c, a)] {
            assert!(x.distance(y) <= 1.5 + 1e-4, "{}", x.distance(y));
        }
    }
    let fewer = decimate(&mesh, &DecimateParams::to_tolerance(tol)).unwrap();
    assert!(d.mesh.face_count() > fewer.mesh.face_count());
    assert!(d.mesh.face_count() < mesh.face_count() / 2);

    // against a reference: the mesh moved within the tolerance
    let mut moved = mesh.clone();
    for q in &mut moved.positions {
        q.z += 0.02;
    }
    let d = decimate_against(&moved, &mesh, &p).unwrap();
    assert!(directed(&mesh, &d.mesh) <= tol + 1e-5);
    assert!(directed(&d.mesh, &mesh) <= tol + 1e-5);
    for q in &mut moved.positions {
        q.z += 0.04;
    }
    assert!(decimate_against(&moved, &mesh, &p).is_err());
}

#[test]
fn vertex_labels_are_preserved() {
    let mut mesh = sphere(5);
    // three labels: two caps and a band
    let labels: Vec<u16> = mesh
        .positions
        .iter()
        .map(|p| match p.z {
            z if z > 3.0 => 1,
            z if z < -4.0 => 2,
            _ => 0,
        })
        .collect();
    mesh.vertex_labels = Some(labels.clone());
    let d = decimate(&mesh, &DecimateParams::to_ratio(0.2)).unwrap();
    assert_clean(&mesh, &d.mesh);
    let out = d.mesh.vertex_labels.as_ref().unwrap();
    for (i, &o) in d.vertex_map.iter().enumerate() {
        assert_eq!(out[o as usize], labels[i], "vertex {i}");
    }
    // vertices next to another label never move
    let mut nb: Vec<Vec<u32>> = vec![Vec::new(); mesh.vertex_count()];
    for t in &mesh.tris {
        for k in 0..3 {
            nb[t[k] as usize].push(t[(k + 1) % 3]);
        }
    }
    for v in 0..mesh.vertex_count() {
        if nb[v].iter().any(|&w| labels[w as usize] != labels[v]) {
            let o = d.vertex_map[v] as usize;
            assert_eq!(d.mesh.positions[o], mesh.positions[v]);
        }
    }
    assert!(d.mesh.face_count() < mesh.face_count() / 3);
}

#[test]
fn face_labels_are_preserved() {
    let mut mesh = sphere(4);
    let fl: Vec<u16> = (0..mesh.face_count())
        .map(|f| u16::from(mesh.triangle(f)[0].x > 0.0))
        .collect();
    mesh.face_labels = Some(fl.clone());
    let d = decimate(&mesh, &DecimateParams::to_ratio(0.3)).unwrap();
    assert_clean(&mesh, &d.mesh);
    let out = d.mesh.face_labels.as_ref().unwrap();
    assert_eq!(out.len(), d.mesh.face_count());
    // label areas are about the same
    let area = |m: &TriMesh, l: &[u16], k: u16| -> f32 {
        (0..m.face_count())
            .filter(|&f| l[f] == k)
            .map(|f| m.face_area(f))
            .sum()
    };
    for k in 0..2 {
        let (a0, a1) = (area(&mesh, &fl, k), area(&d.mesh, out, k));
        assert!((a0 - a1).abs() / a0 < 0.01, "{k}: {a0} {a1}");
    }
}

#[test]
fn attributes_follow_survivors() {
    let mut mesh = sphere(3);
    mesh.normals = Some(mesh.compute_vertex_normals());
    mesh.vertex_colors = Some(
        (0..mesh.vertex_count())
            .map(|i| [i as u8, 0, 0, 255])
            .collect(),
    );
    let d = decimate(&mesh, &DecimateParams::to_faces(300)).unwrap();
    let m = &d.mesh;
    assert_eq!(m.normals.as_ref().unwrap().len(), m.vertex_count());
    let colors = m.vertex_colors.as_ref().unwrap();
    for (i, &o) in d.vertex_map.iter().enumerate() {
        // survivors keep their own colour
        if m.positions[o as usize] == mesh.positions[i] {
            assert_eq!(colors[o as usize][0], i as u8);
        }
    }
}

#[test]
fn max_error_stops_early() {
    let mesh = noisy_sphere(4, 0.05);
    let p = DecimateParams {
        target_faces: 0,
        max_error: 1e-4,
        ..DecimateParams::default()
    };
    let d = decimate(&mesh, &p).unwrap();
    assert!(d.max_cost <= 1e-4);
    assert!(d.collapses > 0 && d.mesh.face_count() > 100);
}

#[test]
fn small_closed_meshes_stay_valid() {
    // a tetrahedron cannot collapse further
    let t = TriMesh::new(
        vec![Vec3::ZERO, Vec3::X, Vec3::Y, Vec3::Z],
        vec![[0, 2, 1], [0, 1, 3], [0, 3, 2], [1, 2, 3]],
    );
    let d = decimate(&t, &DecimateParams::to_faces(0)).unwrap();
    assert_eq!(d.mesh.face_count(), 4);
    let ico = sphere(0);
    let d = decimate(&ico, &DecimateParams::to_faces(0)).unwrap();
    assert!(d.mesh.validate().is_watertight());
    assert!(d.mesh.face_count() >= 4);
}

#[test]
fn unreferenced_vertices_are_unmapped() {
    let mut mesh = sphere(2);
    mesh.positions.push(Vec3::splat(100.0));
    let d = decimate(&mesh, &DecimateParams::to_faces(100)).unwrap();
    assert_eq!(*d.vertex_map.last().unwrap(), UNMAPPED);
    assert_eq!(d.mesh.validate().unreferenced_vertices, 0);
}

#[test]
fn invalid_input() {
    let m = TriMesh::default();
    assert!(matches!(
        decimate(&m, &DecimateParams::default()),
        Err(GeomError::InvalidInput(_))
    ));
    let s = sphere(1);
    assert!(decimate(&s, &DecimateParams::to_ratio(0.0)).is_err());
    assert!(decimate(&s, &DecimateParams::to_ratio(1.5)).is_err());
    assert!(decimate(&s, &DecimateParams::to_tolerance(-1.0)).is_err());
    let mut bad = s.clone();
    bad.positions[0] = Vec3::NAN;
    assert!(decimate(&bad, &DecimateParams::default()).is_err());
    let mut bad = s;
    bad.tris[0][0] = 10_000;
    assert!(matches!(
        decimate(&bad, &DecimateParams::default()),
        Err(GeomError::Mesh(_))
    ));
}

/// 300k -> 30k triangles; run with
/// `cargo test -p dc-geom --release -- --ignored qem_performance --nocapture`.
#[test]
#[ignore = "performance measurement (release)"]
fn qem_performance() {
    let mesh = noisy_sphere(7, 0.01); // 327680 faces
    let t0 = std::time::Instant::now();
    let d = decimate(&mesh, &DecimateParams::to_faces(30_000)).unwrap();
    let dt = t0.elapsed();
    println!(
        "decimate {} -> {} faces in {:.2} s",
        mesh.face_count(),
        d.mesh.face_count(),
        dt.as_secs_f64()
    );
    assert!(d.mesh.validate().is_clean());
    let t0 = std::time::Instant::now();
    let d = decimate(&mesh, &DecimateParams::to_tolerance(0.02)).unwrap();
    println!(
        "decimate to 0.02 mm: {} -> {} faces in {:.2} s",
        mesh.face_count(),
        d.mesh.face_count(),
        t0.elapsed().as_secs_f64()
    );
}
