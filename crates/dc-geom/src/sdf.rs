//! Narrow-band signed distance fields: offset, closing and meshing (WP18).

use glam::Vec3;

use dc_mesh::TriMesh;

use crate::{GeomError, Result};

/// SDF sampling parameters.
#[derive(Debug, Clone, PartialEq)]
pub struct SdfParams {
    /// Voxel edge length (mm).
    pub voxel_size: f32,
    /// Half width of the narrow band (mm).
    pub band: f32,
}

impl Default for SdfParams {
    fn default() -> Self {
        Self {
            voxel_size: 0.1,
            band: 1.0,
        }
    }
}

/// Sparse narrow-band signed distance grid (negative inside).
#[derive(Debug, Clone)]
pub struct SdfGrid {
    /// Voxel edge length (mm).
    pub voxel_size: f32,
    /// World position of voxel `(0, 0, 0)`.
    pub origin: Vec3,
}

impl SdfGrid {
    /// Samples the signed distance of a closed `mesh`.
    pub fn from_mesh(_mesh: &TriMesh, _params: &SdfParams) -> Result<Self> {
        Err(GeomError::NotImplemented("SDF sampling (WP18)"))
    }

    /// Offsets the surface by `distance` (mm, positive = outwards).
    pub fn offset(&mut self, _distance: f32) -> Result<()> {
        Err(GeomError::NotImplemented("SDF offset (WP18)"))
    }

    /// Morphological closing (dilate then erode) by `radius` (mm).
    pub fn close(&mut self, _radius: f32) -> Result<()> {
        Err(GeomError::NotImplemented("SDF closing (WP18)"))
    }

    /// Extracts the zero level set as a triangle mesh.
    pub fn to_mesh(&self) -> Result<TriMesh> {
        Err(GeomError::NotImplemented("SDF meshing (WP18)"))
    }
}
