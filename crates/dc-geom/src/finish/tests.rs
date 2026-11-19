use super::*;

/// Midpoint vertex of `a b` on the unit sphere (shared between faces).
fn midpoint(
    pos: &mut Vec<Vec3>,
    mid: &mut std::collections::HashMap<(u32, u32), u32>,
    a: u32,
    b: u32,
) -> u32 {
    let key = (a.min(b), a.max(b));
    *mid.entry(key).or_insert_with(|| {
        pos.push(((pos[a as usize] + pos[b as usize]) * 0.5).normalize());
        (pos.len() - 1) as u32
    })
}

/// Upper half (z > 0) of an icosphere of radius `r` around the origin
/// (`subdivisions` times subdivided, nearly equilateral faces like a
/// marching cubes surface), open along a jagged equator, faces
/// counter-clockwise seen from outside.
fn hemisphere(r: f32, subdivisions: u32) -> TriMesh {
    let t = (1.0 + 5f32.sqrt()) / 2.0;
    let mut pos: Vec<Vec3> = [
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
    .map(|&(x, y, z)| Vec3::new(x, y, z).normalize())
    .collect();
    let mut tris: Vec<[u32; 3]> = vec![
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
    for _ in 0..subdivisions {
        let mut mid = std::collections::HashMap::new();
        let mut next = Vec::with_capacity(tris.len() * 4);
        for &[a, b, c] in &tris {
            let ab = midpoint(&mut pos, &mut mid, a, b);
            let bc = midpoint(&mut pos, &mut mid, b, c);
            let ca = midpoint(&mut pos, &mut mid, c, a);
            next.extend([[a, ab, ca], [b, bc, ab], [c, ca, bc], [ab, bc, ca]]);
        }
        tris = next;
    }
    let mut mesh = TriMesh::new(pos.into_iter().map(|p| p * r).collect(), tris);
    let keep: Vec<bool> = (0..mesh.face_count())
        .map(|f| {
            let [a, b, c] = mesh.triangle(f);
            (a + b + c).z > 0.0
        })
        .collect();
    retain_faces(&mut mesh, &keep);
    compact(&mut mesh);
    for f in 0..mesh.face_count() {
        let [a, b, c] = mesh.triangle(f);
        if mesh.face_normal(f).dot((a + b + c) / 3.0) < 0.0 {
            mesh.tris[f].swap(1, 2);
        }
    }
    mesh
}

/// Removes the faces whose centroid is within `radius` of `centre`.
fn punch(mesh: &mut TriMesh, centre: Vec3, radius: f32) {
    let keep: Vec<bool> = (0..mesh.face_count())
        .map(|f| {
            let [a, b, c] = mesh.triangle(f);
            ((a + b + c) / 3.0).distance(centre) > radius
        })
        .collect();
    retain_faces(mesh, &keep);
}

#[test]
fn finish_closes_holes_to_the_curvature_and_keeps_the_base_open() {
    let r = 10.0;
    let mut mesh = hemisphere(r, 5);
    let holes = [
        Vec3::new(0.0, 0.0, 1.0),
        Vec3::new(0.5, 0.0, 0.6).normalize(),
        Vec3::new(-0.3, 0.6, 0.5).normalize(),
    ];
    for h in holes {
        punch(&mut mesh, h * r, 1.5);
    }
    // a small island (a little cap far away) and a spike
    let island = hemisphere(1.0, 2);
    let n = mesh.positions.len() as u32;
    mesh.positions.extend(
        island
            .positions
            .iter()
            .map(|p| *p + Vec3::new(30.0, 0.0, 0.0)),
    );
    mesh.tris
        .extend(island.tris.iter().map(|t| t.map(|i| i + n)));
    let dir = Vec3::new(-0.8, 0.0, 0.5).normalize() * r;
    let spike = (0..n as usize)
        .min_by(|&a, &b| {
            mesh.positions[a]
                .distance(dir)
                .total_cmp(&mesh.positions[b].distance(dir))
        })
        .unwrap();
    mesh.positions[spike] *= 1.1;
    assert_eq!(mesh.boundary_loops().len(), 5);

    // the test mesh is coarser than a scan: no edge or normal cap
    let params = FinishParams {
        max_edge_mm: 0.0,
        max_normal_deg: 0.0,
        ..FinishParams::default()
    };
    let (out, report) = finish_mesh(&mesh, &params, &|_| {}, &|| false).unwrap();
    assert_eq!(report.islands_removed, 1, "{report:?}");
    assert!(report.spikes_fixed >= 1, "{report:?}");
    assert_eq!(report.holes_filled, 3, "{report:?}");
    assert_eq!(report.boundary_loops, 1, "{report:?}");
    assert_eq!(report.holes_left, 0);
    assert_eq!(report.non_manifold_edges, 0);
    assert_eq!(report.components, 1);
    assert_eq!(report.faces_before, mesh.face_count());
    assert!(report.faces_after < report.faces_before / 2, "{report:?}");
    assert_eq!(report.faces_after, out.face_count());
    // the base border stays where it was (around z = 0)
    let loops = out.boundary_loops();
    for &v in &loops[0] {
        assert!(out.positions[v as usize].z.abs() < 0.6);
    }
    // patches follow the sphere: a flat cap of radius 1.5 would sag by
    // r - sqrt(r² - 1.5²) = 0.11 mm
    let sag = r - (r * r - 1.5f32 * 1.5).sqrt();
    let worst = out
        .positions
        .iter()
        .map(|p| (p.length() - r).abs())
        .fold(0.0f32, f32::max);
    assert!(worst < 0.35 * sag, "worst {worst} sag {sag}");
    // faces still look outward
    for f in 0..out.face_count() {
        let [a, b, c] = out.triangle(f);
        assert!(out.face_normal(f).dot((a + b + c) / 3.0) > 0.0);
    }
}

#[test]
fn bowtie_vertex_is_split_and_fins_dropped() {
    // lone triangles are all fins: removed, nothing left
    let positions = vec![
        Vec3::ZERO,
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(-1.0, 0.0, 0.0),
        Vec3::new(0.0, -1.0, 0.0),
    ];
    let mesh = TriMesh::new(positions, vec![[0, 1, 2], [0, 3, 4]]);
    let mut report = FinishReport::default();
    let out = clean(&mesh, 0.0, &mut report);
    assert!(out.is_empty());
    assert_eq!(report.fins_removed, 2);

    // two n x n grids (diagonals from (i, j) to (i + 1, j + 1)) touching at
    // one corner vertex
    let n = 6usize;
    let mut positions = Vec::new();
    let mut tris = Vec::new();
    for (g, o) in [Vec3::ZERO, Vec3::new(n as f32, n as f32, 0.0)]
        .into_iter()
        .enumerate()
    {
        let base = positions.len() as u32;
        for y in 0..=n {
            for x in 0..=n {
                positions.push(o + Vec3::new(x as f32, y as f32, 0.0));
            }
        }
        let id = |x: usize, y: usize| base + (y * (n + 1) + x) as u32;
        for y in 0..n {
            for x in 0..n {
                if g == 1 && x + y > n + 2 {
                    // the second grid is smaller
                    continue;
                }
                tris.push([id(x, y), id(x + 1, y), id(x + 1, y + 1)]);
                tris.push([id(x, y), id(x + 1, y + 1), id(x, y + 1)]);
            }
        }
    }
    // weld the corner (n, n) of the first grid with the origin of the second
    let corner = (n * (n + 1) + n) as u32;
    let origin = ((n + 1) * (n + 1)) as u32;
    for t in &mut tris {
        for v in t.iter_mut() {
            if *v == origin {
                *v = corner;
            }
        }
    }
    let mesh = TriMesh::new(positions, tris);
    let mut report = FinishReport::default();
    let out = clean(&mesh, 0.0, &mut report);
    assert_eq!(report.nonmanifold_fixed, 1, "{report:?}");
    let loops = out.boundary_loops();
    assert_eq!(loops.len(), 2);
    for l in &loops {
        let mut seen = std::collections::HashSet::new();
        assert!(l.iter().all(|v| seen.insert(*v)), "simple loop");
    }
    assert_eq!(out.connected_components().len(), 2);
    // the main component only (the second grid is smaller)
    let mut report = FinishReport::default();
    let out = clean(&mesh, 1.0, &mut report);
    assert_eq!(report.islands_removed, 1);
    assert!(
        out.positions
            .iter()
            .all(|p| p.x <= n as f32 && p.y <= n as f32)
    );
}

#[test]
fn finish_rejects_bad_parameters_and_cancels() {
    let mesh = hemisphere(5.0, 3);
    let bad = FinishParams {
        min_island_fraction: 1.5,
        ..FinishParams::default()
    };
    assert!(finish_mesh(&mesh, &bad, &|_| {}, &|| false).is_err());
    let bad = FinishParams {
        max_error_mm: 0.0,
        ..FinishParams::default()
    };
    assert!(finish_mesh(&mesh, &bad, &|_| {}, &|| false).is_err());
    assert!(matches!(
        finish_mesh(&mesh, &FinishParams::default(), &|_| {}, &|| true),
        Err(GeomError::Cancelled)
    ));
    // stages are reported in order
    let seen = std::sync::Mutex::new(Vec::new());
    finish_mesh(
        &mesh,
        &FinishParams::default(),
        &|s| seen.lock().unwrap().push(s),
        &|| false,
    )
    .unwrap();
    assert_eq!(
        *seen.lock().unwrap(),
        vec![
            FinishStage::Clean,
            FinishStage::Holes,
            FinishStage::Optimise
        ]
    );
}
