//! Sparse linear algebra: CSR matrices, Jacobi-preconditioned conjugate
//! gradient, harmonic solves (WP17).

use crate::{GeomError, Result};

/// Square sparse matrix in compressed sparse row form.
#[derive(Debug, Clone, PartialEq)]
pub struct CsrMatrix {
    /// Number of rows (and columns).
    pub n: usize,
    /// Row start offsets into `cols` / `vals` (length `n + 1`).
    pub row_ptr: Vec<usize>,
    /// Column index of every stored entry.
    pub cols: Vec<usize>,
    /// Value of every stored entry.
    pub vals: Vec<f64>,
}

impl CsrMatrix {
    /// `n × n` identity matrix.
    pub fn identity(n: usize) -> Self {
        Self {
            n,
            row_ptr: (0..=n).collect(),
            cols: (0..n).collect(),
            vals: vec![1.0; n],
        }
    }

    /// Number of stored entries.
    pub fn nnz(&self) -> usize {
        self.vals.len()
    }
}

/// Conjugate gradient parameters.
#[derive(Debug, Clone, PartialEq)]
pub struct PcgParams {
    /// Maximum number of iterations.
    pub max_iterations: usize,
    /// Relative residual tolerance.
    pub tolerance: f64,
}

impl Default for PcgParams {
    fn default() -> Self {
        Self {
            max_iterations: 1000,
            tolerance: 1e-8,
        }
    }
}

/// Solves `a x = b` for a symmetric positive definite `a`.
pub fn pcg_solve(_a: &CsrMatrix, _b: &[f64], _params: &PcgParams) -> Result<Vec<f64>> {
    Err(GeomError::NotImplemented("Jacobi PCG (WP17)"))
}

/// Harmonic interpolation over a mesh: values fixed at `constrained`
/// vertices, smooth (cotangent Laplacian) everywhere else.
pub fn harmonic(
    _mesh: &dc_mesh::TriMesh,
    _constrained: &[(u32, f64)],
    _params: &PcgParams,
) -> Result<Vec<f64>> {
    Err(GeomError::NotImplemented("harmonic solve (WP17)"))
}
