use glam::Vec3;

use super::*;

/// Outward cube of side `s` with its lowest corner at `o`.
fn cube(o: Vec3, s: f32) -> TriMesh {
    let v = [
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [1.0, 1.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
        [1.0, 0.0, 1.0],
        [1.0, 1.0, 1.0],
        [0.0, 1.0, 1.0],
    ]
    .map(|p| o + Vec3::from_array(p) * s)
    .to_vec();
    let t = vec![
        [0, 2, 1],
        [0, 3, 2],
        [4, 5, 6],
        [4, 6, 7],
        [0, 1, 5],
        [0, 5, 4],
        [3, 7, 6],
        [3, 6, 2],
        [0, 4, 7],
        [0, 7, 3],
        [1, 2, 6],
        [1, 6, 5],
    ];
    TriMesh::new(v, t)
}

/// `a` and `b` in one mesh, indices of `b` after those of `a`.
fn join(a: &TriMesh, b: &TriMesh) -> TriMesh {
    let n = a.positions.len() as u32;
    let mut m = a.clone();
    m.positions.extend(&b.positions);
    m.tris.extend(b.tris.iter().map(|t| t.map(|i| i + n)));
    m
}

/// Every triangle with its own three vertices (an STL file as triangles).
fn soup(m: &TriMesh) -> TriMesh {
    let positions = m
        .tris
        .iter()
        .flat_map(|t| t.map(|i| m.positions[i as usize]))
        .collect();
    let tris = (0..m.tris.len() as u32)
        .map(|f| [3 * f, 3 * f + 1, 3 * f + 2])
        .collect();
    TriMesh::new(positions, tris)
}

/// Outward square torus (genus 1): `n × n` quads around a ring.
fn torus(n: u32) -> TriMesh {
    let (big, small) = (10.0f32, 3.0f32);
    let mut positions = Vec::new();
    for i in 0..n {
        let u = i as f32 / n as f32 * std::f32::consts::TAU;
        for j in 0..n {
            let w = j as f32 / n as f32 * std::f32::consts::TAU;
            let r = big + small * w.cos();
            positions.push(Vec3::new(r * u.cos(), r * u.sin(), small * w.sin()));
        }
    }
    let at = |i: u32, j: u32| (i % n) * n + (j % n);
    let mut tris = Vec::new();
    for i in 0..n {
        for j in 0..n {
            let (a, b, c, d) = (at(i, j), at(i + 1, j), at(i + 1, j + 1), at(i, j + 1));
            tris.extend([[a, b, c], [a, c, d]]);
        }
    }
    let mut m = TriMesh::new(positions, tris);
    if solid_topology(&m).volume_mm3 < 0.0 {
        for t in &mut m.tris {
            t.swap(1, 2);
        }
    }
    m
}

/// The cube with the top front edge (4, 5) split at `at` on both of its
/// triangles (closed), or only on the top (a T-junction: a crack).
fn split_edge(at: Vec3, both_sides: bool) -> TriMesh {
    let mut m = cube(Vec3::ZERO, 1.0);
    m.positions.push(at);
    let s = 8;
    // top (4, 5, 6), (4, 6, 7) -> fan around the split
    m.tris[2] = [4, s, 7];
    m.tris[3] = [s, 5, 6];
    m.tris.push([s, 6, 7]);
    if both_sides {
        // front (0, 5, 4) -> (0, 5, s), (0, s, 4)
        m.tris[5] = [0, 5, s];
        m.tris.push([0, s, 4]);
    }
    m
}

#[test]
fn a_cube_is_a_closed_outward_solid() {
    let t = solid_topology(&cube(Vec3::ZERO, 2.0));
    assert_eq!(
        t,
        SolidTopology {
            vertices: 8,
            edges: 18,
            triangles: 12,
            left_out: 0,
            boundary_edges: 0,
            nonmanifold_edges: 0,
            flipped_edges: 0,
            pieces: 1,
            genus: Some(0),
            volume_mm3: 8.0,
        }
    );
    assert!(t.is_closed());
}

#[test]
fn duplicated_vertices_are_one_vertex() {
    let t = solid_topology(&soup(&cube(Vec3::ZERO, 1.0)));
    assert_eq!((t.vertices, t.edges, t.left_out), (8, 18, 0));
    assert!(t.is_closed());
    assert_eq!(t.genus, Some(0));
    // -0.0 and 0.0 are one position
    let mut m = soup(&cube(Vec3::ZERO, 1.0));
    m.positions[0] = Vec3::new(-0.0, -0.0, -0.0);
    assert_eq!(solid_topology(&m).vertices, 8);
}

#[test]
fn close_but_distinct_vertices_stay_apart() {
    // a 40 nm edge, as the shell mesher makes: closed, nothing collapses
    let t = solid_topology(&soup(&split_edge(Vec3::new(4e-5, 0.0, 1.0), true)));
    assert_eq!((t.vertices, t.left_out), (9, 0));
    assert!(t.is_closed());
    assert_eq!((t.flipped_edges, t.genus), (0, Some(0)));
    assert!((t.volume_mm3 - 1.0).abs() < 1e-9);
}

#[test]
fn collapsed_triangles_are_left_out() {
    // the split vertex on the corner itself: two triangles have no area
    let t = solid_topology(&split_edge(Vec3::new(0.0, 0.0, 1.0), true));
    assert_eq!(t.left_out, 2);
    assert!(t.is_closed());
    assert_eq!(t.genus, Some(0));
}

#[test]
fn a_t_junction_is_open() {
    let t = solid_topology(&split_edge(Vec3::new(0.5, 0.0, 1.0), false));
    // (4, s), (s, 5) on the top, (4, 5) on the front
    assert_eq!(t.boundary_edges, 3);
    assert_eq!(t.genus, None);
}

#[test]
fn solids_touching_at_a_vertex_are_closed() {
    let a = cube(Vec3::ZERO, 1.0);
    let b = cube(Vec3::ONE, 1.0);
    let t = solid_topology(&soup(&join(&a, &b)));
    assert!(t.is_closed());
    assert_eq!((t.vertices, t.pieces, t.genus), (15, 1, Some(0)));
    assert!((t.volume_mm3 - 2.0).abs() < 1e-9);
}

#[test]
fn solids_sharing_an_edge_are_non_manifold() {
    let a = cube(Vec3::ZERO, 1.0);
    let b = cube(Vec3::new(1.0, 1.0, 0.0), 1.0);
    let t = solid_topology(&soup(&join(&a, &b)));
    assert_eq!((t.boundary_edges, t.nonmanifold_edges), (0, 1));
    assert_eq!(t.genus, None);
}

#[test]
fn flipped_and_inward_triangles_are_counted() {
    let mut one = cube(Vec3::ZERO, 1.0);
    one.tris[0].swap(1, 2);
    let t = solid_topology(&one);
    assert!(t.is_closed());
    assert_eq!(t.flipped_edges, 3);
    let mut inward = cube(Vec3::ZERO, 1.0);
    for t in &mut inward.tris {
        t.swap(1, 2);
    }
    let t = solid_topology(&inward);
    assert_eq!(t.flipped_edges, 0);
    assert!((t.volume_mm3 + 1.0).abs() < 1e-9);
}

#[test]
fn a_tunnel_is_genus_one() {
    let t = solid_topology(&soup(&torus(12)));
    assert!(t.is_closed());
    assert_eq!((t.flipped_edges, t.pieces, t.genus), (0, 1, Some(1)));
    assert!(t.volume_mm3 > 0.0);
    // two pieces: a cube and a torus, one hole
    let t = solid_topology(&join(&torus(12), &cube(Vec3::splat(50.0), 1.0)));
    assert_eq!((t.pieces, t.genus), (2, Some(1)));
}

#[test]
fn empty_and_out_of_range() {
    let t = solid_topology(&TriMesh::default());
    assert_eq!(
        (t.triangles, t.pieces, t.genus, t.volume_mm3),
        (0, 0, None, 0.0)
    );
    let mut m = cube(Vec3::ZERO, 1.0);
    m.tris.push([0, 1, 99]);
    let t = solid_topology(&m);
    assert_eq!(t.left_out, 1);
    assert!(t.is_closed());
}
