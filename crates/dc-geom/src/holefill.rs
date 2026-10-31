//! Hole filling (Liepa: minimum-weight triangulation, refinement, fairing) (WP16).

use dc_mesh::TriMesh;

use crate::{GeomError, Result};

/// Hole filling parameters.
#[derive(Debug, Clone, PartialEq)]
pub struct HoleFillParams {
    /// Holes with more boundary edges than this are left open.
    pub max_boundary_edges: usize,
    /// Refine the patch to match the surrounding edge length.
    pub refine: bool,
    /// Fair the patch (smooth blend with the surroundings).
    pub fair: bool,
}

impl Default for HoleFillParams {
    fn default() -> Self {
        Self {
            max_boundary_edges: 500,
            refine: true,
            fair: true,
        }
    }
}

/// Fills the holes of `mesh`; returns the patched mesh.
pub fn fill_holes(_mesh: &TriMesh, _params: &HoleFillParams) -> Result<TriMesh> {
    Err(GeomError::NotImplemented("hole filling (WP16)"))
}
