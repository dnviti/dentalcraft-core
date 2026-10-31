//! Point-to-plane iterative closest point registration (WP14).

use glam::DAffine3;

use dc_mesh::TriMesh;

use crate::{GeomError, Result};

/// ICP parameters.
#[derive(Debug, Clone, PartialEq)]
pub struct IcpParams {
    /// Maximum number of iterations.
    pub max_iterations: usize,
    /// Pairs farther than this are rejected (mm).
    pub max_pair_distance: f64,
    /// Stop when the RMS change between iterations is below this (mm).
    pub tolerance: f64,
    /// Initial transform applied to the source.
    pub initial: DAffine3,
}

impl Default for IcpParams {
    fn default() -> Self {
        Self {
            max_iterations: 50,
            max_pair_distance: 1.0,
            tolerance: 1e-5,
            initial: DAffine3::IDENTITY,
        }
    }
}

/// Outcome of a registration.
#[derive(Debug, Clone, PartialEq)]
pub struct IcpResult {
    /// Rigid transform mapping the source onto the target.
    pub transform: DAffine3,
    /// Final RMS point-to-plane distance (mm).
    pub rms: f64,
    /// Iterations performed.
    pub iterations: usize,
    /// Fraction of source points with an accepted pair.
    pub overlap: f64,
}

/// Registers `source` onto `target`.
pub fn point_to_plane(
    _source: &TriMesh,
    _target: &TriMesh,
    _params: &IcpParams,
) -> Result<IcpResult> {
    Err(GeomError::NotImplemented("point-to-plane ICP (WP14)"))
}
