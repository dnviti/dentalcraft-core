//! Bounding volume hierarchy: closest point and ray queries on a triangle mesh (WP13).

use glam::Vec3;

use dc_mesh::TriMesh;

use crate::{GeomError, Result};

/// Closest point on the surface to a query point.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ClosestHit {
    /// Index of the triangle holding the point.
    pub face: u32,
    /// Closest point on the surface.
    pub point: Vec3,
    /// Distance from the query point.
    pub distance: f32,
}

/// First intersection of a ray with the surface.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RayHit {
    /// Index of the triangle hit.
    pub face: u32,
    /// Ray parameter of the hit (`origin + t * dir`).
    pub t: f32,
    /// Barycentric coordinates of the hit inside the triangle.
    pub barycentric: Vec3,
}

/// Acceleration structure over the triangles of a mesh.
#[derive(Debug, Clone)]
pub struct Bvh {
    _private: (),
}

impl Bvh {
    /// Builds the hierarchy over `mesh`.
    pub fn build(_mesh: &TriMesh) -> Result<Self> {
        Err(GeomError::NotImplemented("BVH build (WP13)"))
    }

    /// Closest surface point to `p` within `max_distance`.
    pub fn closest_point(&self, _p: Vec3, _max_distance: f32) -> Option<ClosestHit> {
        None
    }

    /// First hit of the ray `origin + t * dir`, `t` in `[0, max_t]`.
    pub fn ray_cast(&self, _origin: Vec3, _dir: Vec3, _max_t: f32) -> Option<RayHit> {
        None
    }
}
