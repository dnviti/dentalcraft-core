//! Geometry kernels for DentalCraft.
//!
//! Every kernel lives in its own module and is reached through a
//! [`Backend`], so that a GPU implementation can replace the CPU one later
//! (WP36): BVH ([`bvh`]), nearest neighbours ([`knn`]), ICP ([`icp`]), QEM
//! decimation ([`qem`]), hole filling ([`holefill`]), sparse solvers
//! ([`sparse`]), signed distance fields ([`sdf`]). [`metrics`] holds the
//! comparison measures used by the golden tests (Hausdorff distance, label
//! agreement).
//!
//! The 3D scanner pipeline uses [`tsdf`] (depth frame fusion, raycast,
//! extraction), [`cloud`] (point cloud filters, normals, selections),
//! [`meshing`] (oriented point cloud to open mesh) and [`repair`] (island
//! removal, Taubin smoothing); [`tsdf`] and [`meshing`] share one sparse
//! marching cubes implementation.

pub mod bvh;
pub mod cloud;
pub mod cotan;
pub mod holefill;
pub mod icp;
pub mod knn;
mod mc;
pub mod meshing;
pub mod metrics;
pub mod qem;
pub mod repair;
pub mod sdf;
pub mod sparse;
pub mod tsdf;

/// Errors produced by geometry kernels.
#[derive(Debug, thiserror::Error)]
pub enum GeomError {
    /// The kernel is part of a later work package and not available yet.
    #[error("not implemented yet: {0}")]
    NotImplemented(&'static str),
    /// The input is not usable by the kernel (empty mesh, size mismatch...).
    #[error("invalid input: {0}")]
    InvalidInput(String),
    /// An iterative method stopped without reaching its tolerance.
    #[error("did not converge after {iterations} iterations (residual {residual})")]
    NoConvergence {
        /// Iterations performed.
        iterations: usize,
        /// Final residual.
        residual: f64,
    },
    /// The computation was cancelled by the caller.
    #[error("cancelled")]
    Cancelled,
    /// Error from the mesh layer.
    #[error(transparent)]
    Mesh(#[from] dc_mesh::MeshError),
}

/// Result alias for geometry kernels.
pub type Result<T, E = GeomError> = std::result::Result<T, E>;

/// Compute backend executing the geometry kernels.
pub trait Backend: Send + Sync {
    /// Short identifier (e.g. `cpu`, `wgpu`).
    fn name(&self) -> &'static str;

    /// `true` when the backend runs on the GPU.
    fn is_gpu(&self) -> bool {
        false
    }
}

/// Default multi-threaded CPU backend (rayon).
#[derive(Debug, Clone, Copy, Default)]
pub struct CpuBackend;

impl Backend for CpuBackend {
    fn name(&self) -> &'static str {
        "cpu"
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cpu_backend_identity() {
        let b: &dyn Backend = &CpuBackend;
        assert_eq!(b.name(), "cpu");
        assert!(!b.is_gpu());
    }

    #[test]
    fn empty_inputs_are_handled() {
        let m = dc_mesh::TriMesh::default();
        let (out, filled) = holefill::fill_holes(&m, &holefill::HoleFillParams::default()).unwrap();
        assert_eq!((out, filled), (m, 0));
        let a = sparse::CsrMatrix::identity(2);
        assert_eq!(
            sparse::pcg_solve(&a, &[1.0, 1.0], &sparse::PcgParams::default()).unwrap(),
            vec![1.0, 1.0]
        );
    }
}
