use super::*;

/// `n x n` vertex grid on z = 0 (spacing 1), counter-clockwise from +Z,
/// without the faces whose centroid falls in one of the `holes`
/// (centre, radius).
fn punched_grid(n: usize, holes: &[(Vec3, f32)]) -> TriMesh {
    let mut positions = Vec::with_capacity(n * n);
    for y in 0..n {
        for x in 0..n {
            positions.push(Vec3::new(x as f32, y as f32, 0.0));
        }
    }
    let id = |x: usize, y: usize| (y * n + x) as u32;
    let mut tris = Vec::new();
    for y in 0..n - 1 {
        for x in 0..n - 1 {
            for t in [
                [id(x, y), id(x + 1, y), id(x + 1, y + 1)],
                [id(x, y), id(x + 1, y + 1), id(x, y + 1)],
            ] {
                let c = t.iter().map(|&v| positions[v as usize]).sum::<Vec3>() / 3.0;
                if holes.iter().all(|(h, r)| c.distance(*h) > *r) {
                    tris.push(t);
                }
            }
        }
    }
    TriMesh::new(positions, tris)
}

fn assert_flat_and_up(mesh: &TriMesh) {
    for f in 0..mesh.face_count() {
        assert!(mesh.face_normal(f).z > 0.999, "face {f} folds");
    }
    for p in &mesh.positions {
        assert!(p.z.abs() < 1e-4);
    }
}

#[test]
fn punched_hole_is_closed_and_outer_border_kept() {
    let full = (19.0f32) * 19.0;
    let mesh = punched_grid(20, &[(Vec3::new(9.5, 9.5, 0.0), 3.2)]);
    assert_eq!(mesh.boundary_loops().len(), 2);
    let (out, filled) = fill_holes(&mesh, &HoleFillParams::default()).unwrap();
    assert_eq!(filled, 1);
    out.check().unwrap();
    let adj = out.edge_adjacency();
    assert_eq!(adj.non_manifold_edge_count(), 0);
    let loops = out.boundary_loops();
    assert_eq!(loops.len(), 1);
    assert_eq!(loops[0].len(), 4 * 19, "the outer border stays open");
    assert!((out.area() - full).abs() < 1e-3, "{}", out.area());
    assert!(out.vertex_count() > mesh.vertex_count(), "refined patch");
    assert_flat_and_up(&out);
}

#[test]
fn without_refinement_the_patch_uses_only_border_vertices() {
    let mesh = punched_grid(12, &[(Vec3::new(5.5, 5.5, 0.0), 2.0)]);
    let hole = mesh
        .boundary_loops()
        .into_iter()
        .map(|l| l.len())
        .min()
        .unwrap();
    let params = HoleFillParams {
        refine: false,
        fair: false,
        ..HoleFillParams::default()
    };
    let (out, filled) = fill_holes(&mesh, &params).unwrap();
    assert_eq!(filled, 1);
    assert_eq!(out.vertex_count(), mesh.vertex_count());
    assert_eq!(out.face_count(), mesh.face_count() + hole - 2);
    assert!((out.area() - 121.0).abs() < 1e-3);
    assert_eq!(out.edge_adjacency().non_manifold_edge_count(), 0);
    assert_flat_and_up(&out);
}

#[test]
fn hole_size_limit_and_largest_loop() {
    let mesh = punched_grid(
        30,
        &[
            (Vec3::new(7.5, 7.5, 0.0), 1.5),
            (Vec3::new(20.5, 20.5, 0.0), 4.5),
        ],
    );
    let sizes: Vec<usize> = mesh.boundary_loops().iter().map(Vec::len).collect();
    assert_eq!(sizes.len(), 3);
    let small = *sizes.iter().min().unwrap();
    let params = HoleFillParams {
        max_boundary_edges: small,
        ..HoleFillParams::default()
    };
    let (out, filled) = fill_holes(&mesh, &params).unwrap();
    assert_eq!(filled, 1);
    assert_eq!(out.boundary_loops().len(), 2);
    // a single hole is the largest loop: never filled
    let one = punched_grid(8, &[]);
    let (same, filled) = fill_holes(&one, &HoleFillParams::default()).unwrap();
    assert_eq!(filled, 0);
    assert_eq!(same, one);
}

#[test]
fn long_loops_are_ear_clipped() {
    let mesh = punched_grid(64, &[(Vec3::new(31.5, 31.5, 0.0), 26.0)]);
    let hole = mesh
        .boundary_loops()
        .into_iter()
        .map(|l| l.len())
        .min()
        .unwrap();
    assert!(hole > DP_MAX, "{hole}");
    let params = HoleFillParams {
        refine: false,
        fair: false,
        ..HoleFillParams::default()
    };
    let (out, filled) = fill_holes(&mesh, &params).unwrap();
    assert_eq!(filled, 1);
    assert_eq!(out.edge_adjacency().non_manifold_edge_count(), 0);
    assert!((out.area() - 63.0 * 63.0).abs() < 1e-2, "{}", out.area());
    assert_flat_and_up(&out);
}

#[test]
fn faired_patch_spans_a_curved_hole() {
    // bowl z = 0.02 r², hole in the middle: the patch stays within the
    // border heights and keeps the orientation
    let mut mesh = punched_grid(20, &[(Vec3::new(9.5, 9.5, 0.0), 3.2)]);
    for p in &mut mesh.positions {
        let r2 = (p.x - 9.5).powi(2) + (p.y - 9.5).powi(2);
        p.z = 0.02 * r2;
    }
    let (out, filled) = fill_holes(&mesh, &HoleFillParams::default()).unwrap();
    assert_eq!(filled, 1);
    assert_eq!(out.edge_adjacency().non_manifold_edge_count(), 0);
    let ring_max = 0.02 * 3.6f32.powi(2) * 2.0;
    for p in &out.positions[mesh.vertex_count()..] {
        assert!(p.z >= 0.0 && p.z <= ring_max, "{p}");
    }
    for f in mesh.face_count()..out.face_count() {
        assert!(out.face_normal(f).z > 0.9);
    }
}
