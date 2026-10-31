//! Quadric error metric decimation (WP15).

use dc_mesh::TriMesh;

use crate::{GeomError, Result};

/// Decimation parameters.
#[derive(Debug, Clone, PartialEq)]
pub struct DecimateParams {
    /// Target number of triangles.
    pub target_faces: usize,
    /// Do not collapse edges whose error exceeds this (mm²).
    pub max_error: f64,
    /// Keep boundary edges in place.
    pub preserve_boundary: bool,
}

impl Default for DecimateParams {
    fn default() -> Self {
        Self {
            target_faces: 100_000,
            max_error: f64::INFINITY,
            preserve_boundary: true,
        }
    }
}

/// Simplifies `mesh` towards `params.target_faces` triangles.
pub fn decimate(_mesh: &TriMesh, _params: &DecimateParams) -> Result<TriMesh> {
    Err(GeomError::NotImplemented("QEM decimation (WP15)"))
}
