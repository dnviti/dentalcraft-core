//! Mesh health report.

use serde::{Deserialize, Serialize};

use crate::TriMesh;

/// Area below which a triangle counts as degenerate (mm²).
pub const DEGENERATE_AREA: f32 = 1e-12;

/// Topological and geometric health of a [`TriMesh`].
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct ValidationReport {
    /// Number of vertices.
    pub vertices: usize,
    /// Number of triangles.
    pub faces: usize,
    /// Triangles referencing a vertex index out of range.
    pub out_of_range_faces: usize,
    /// Triangles with repeated indices or (near) zero area.
    pub degenerate_faces: usize,
    /// Vertices not used by any triangle.
    pub unreferenced_vertices: usize,
    /// Edges used by exactly one triangle.
    pub boundary_edges: usize,
    /// Edges used by more than two triangles.
    pub non_manifold_edges: usize,
    /// Number of boundary loops (holes).
    pub boundary_loops: usize,
    /// Number of connected components.
    pub components: usize,
}

impl ValidationReport {
    /// Computes the report for `mesh`. Out-of-range triangles are counted and
    /// excluded from the topological checks.
    pub fn of(mesh: &TriMesh) -> Self {
        let nv = mesh.positions.len();
        let in_range = |t: &[u32; 3]| t.iter().all(|&i| (i as usize) < nv);
        let clean = TriMesh::new(
            mesh.positions.clone(),
            mesh.tris.iter().copied().filter(in_range).collect(),
        );
        let degenerate_faces = (0..clean.tris.len())
            .filter(|&f| {
                let [a, b, c] = clean.tris[f];
                a == b || b == c || a == c || clean.face_area(f) <= DEGENERATE_AREA
            })
            .count();
        let mut used = vec![false; nv];
        for t in &clean.tris {
            for &i in t {
                used[i as usize] = true;
            }
        }
        let adj = clean.edge_adjacency();
        Self {
            vertices: nv,
            faces: mesh.tris.len(),
            out_of_range_faces: mesh.tris.len() - clean.tris.len(),
            degenerate_faces,
            unreferenced_vertices: used.iter().filter(|u| !**u).count(),
            boundary_edges: adj.boundary_edge_count(),
            non_manifold_edges: adj.non_manifold_edge_count(),
            boundary_loops: adj.boundary_loops().len(),
            components: clean.connected_components().len(),
        }
    }

    /// No out-of-range indices, no edge shared by more than two triangles.
    pub fn is_manifold(&self) -> bool {
        self.out_of_range_faces == 0 && self.non_manifold_edges == 0
    }

    /// Manifold and without boundary edges (closed surface).
    pub fn is_watertight(&self) -> bool {
        self.is_manifold() && self.boundary_edges == 0
    }

    /// Manifold and without degenerate triangles.
    pub fn is_clean(&self) -> bool {
        self.is_manifold() && self.degenerate_faces == 0
    }
}

#[cfg(test)]
mod tests {
    use crate::test_meshes::*;

    #[test]
    fn tetra_is_watertight() {
        let r = tetra().validate();
        assert!(r.is_watertight());
        assert!(r.is_clean());
        assert_eq!(r.components, 1);
        assert_eq!(r.boundary_loops, 0);
    }

    #[test]
    fn quad_is_open_but_manifold() {
        let r = quad().validate();
        assert!(r.is_manifold());
        assert!(!r.is_watertight());
        assert_eq!(r.boundary_edges, 4);
        assert_eq!(r.boundary_loops, 1);
    }

    #[test]
    fn detects_non_manifold_degenerate_and_bad_indices() {
        let mut m = quad();
        m.positions.push(glam::Vec3::new(0.5, 0.5, 1.0));
        m.tris.push([0, 2, 4]); // third face on edge 0-2
        m.tris.push([1, 1, 3]); // repeated index
        m.tris.push([0, 1, 99]); // out of range
        let r = m.validate();
        assert_eq!(r.non_manifold_edges, 1);
        assert_eq!(r.degenerate_faces, 1);
        assert_eq!(r.out_of_range_faces, 1);
        assert!(!r.is_manifold());
    }
}
