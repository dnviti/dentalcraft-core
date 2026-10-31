//! Geometry kernels for DentalCraft.
//!
//! Every kernel lives in its own module and is reached through a
//! [`Backend`], so that a GPU implementation can replace the CPU one later
//! (WP36). In this version all kernels are contracts only: they return
//! [`GeomError::NotImplemented`] until their work package lands
//! (BVH WP13, ICP WP14, QEM WP15, hole filling WP16, sparse/harmonic WP17,
//! SDF WP18). [`metrics`] holds the comparison measures used by the golden
//! tests (Hausdorff distance, label agreement).

pub mod bvh;
pub mod cotan;
pub mod holefill;
pub mod icp;
pub mod knn;
pub mod metrics;
pub mod qem;
pub mod sdf;
pub mod sparse;

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
    fn stubs_report_not_implemented() {
        let m = dc_mesh::TriMesh::default();
        assert!(matches!(
            qem::decimate(&m, &qem::DecimateParams::default()),
            Err(GeomError::NotImplemented(_))
        ));
        assert!(matches!(
            holefill::fill_holes(&m, &holefill::HoleFillParams::default()),
            Err(GeomError::NotImplemented(_))
        ));
        assert!(matches!(
            icp::point_to_plane(&m, &m, &icp::IcpParams::default()),
            Err(GeomError::NotImplemented(_))
        ));
        assert!(matches!(
            sdf::SdfGrid::from_mesh(&m, &sdf::SdfParams::default()),
            Err(GeomError::NotImplemented(_))
        ));
        let a = sparse::CsrMatrix::identity(2);
        assert_eq!(
            sparse::pcg_solve(&a, &[1.0, 1.0], &sparse::PcgParams::default()).unwrap(),
            vec![1.0, 1.0]
        );
    }
}
