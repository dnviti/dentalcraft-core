//! Sparse linear algebra: CSR matrices, Jacobi-preconditioned conjugate
//! gradient, Dirichlet and harmonic solves (WP17, port plan B1.3).
//!
//! The solver is the reproducible PCG of the plugin (`gpu_solve.py`:
//! `repro_csr_cpu`, `_repro_pcg_rows`, `Csr.apply`, `exact_dot`), with the
//! same operation order, so it gives the same bits as the oracle and the
//! same bits on every thread count:
//!
//! * a matrix-vector product sums each row on its own, `0 + v0 x0 + v1 x1 +
//!   ...` in the column order of the row: rows are independent, so running
//!   them in parallel does not change any bit;
//! * every vector update (`x += alpha p`, `z = r / diag`...) is element-wise;
//! * dot products use [`exact_dot`]: the products are pre-rounded on two
//!   grids set by `max |p|` and `n` (two-fold pre-rounding), after which
//!   every partial sum is exact, so the result does not depend on the
//!   summation order. On top of that the partial sums are taken over fixed
//!   chunks of [`CHUNK`] entries and combined in chunk order, so the
//!   reduction tree does not depend on the rayon pool either.

use rayon::prelude::*;

use crate::{GeomError, Result};

/// Rows (or vector entries) per parallel work item. Fixed, so that the work
/// split never depends on the number of threads.
pub const CHUNK: usize = 4096;

/// Square sparse matrix in compressed sparse row form (rows sorted by
/// column, no duplicate entries when built by [`CsrMatrix::from_triplets`]).
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

    /// `n × n` matrix of the `(row, col, value)` entries, duplicates summed
    /// (as `gpu_solve.csr_from_coo`): entries are sorted by `(row, col)`
    /// with a stable sort and the duplicates of one position are added in
    /// input order starting from `0.0`, so the result is deterministic and
    /// bit-identical to the oracle.
    pub fn from_triplets(n: usize, triplets: &[(usize, usize, f64)]) -> Result<Self> {
        if let Some(&(r, c, _)) = triplets.iter().find(|&&(r, c, _)| r >= n || c >= n) {
            return Err(GeomError::InvalidInput(format!(
                "entry ({r}, {c}) outside a {n} x {n} matrix"
            )));
        }
        let mut order: Vec<usize> = (0..triplets.len()).collect();
        order.sort_by_key(|&k| (triplets[k].0, triplets[k].1));
        let mut row_ptr = vec![0usize; n + 1];
        let mut cols = Vec::with_capacity(triplets.len());
        let mut vals: Vec<f64> = Vec::with_capacity(triplets.len());
        let mut last: Option<(usize, usize)> = None;
        for &k in &order {
            let (r, c, v) = triplets[k];
            if last != Some((r, c)) {
                cols.push(c);
                vals.push(0.0);
                row_ptr[r + 1] += 1;
                last = Some((r, c));
            }
            *vals.last_mut().expect("pushed above") += v;
        }
        for i in 0..n {
            row_ptr[i + 1] += row_ptr[i];
        }
        Ok(Self {
            n,
            row_ptr,
            cols,
            vals,
        })
    }

    /// Number of stored entries.
    pub fn nnz(&self) -> usize {
        self.vals.len()
    }

    /// Checks the structure: `row_ptr` of length `n + 1`, non-decreasing,
    /// ending at `nnz`, and every column below `n`.
    pub fn check(&self) -> Result<()> {
        let bad = |m: &str| Err(GeomError::InvalidInput(format!("CSR matrix: {m}")));
        if self.row_ptr.len() != self.n + 1 || self.row_ptr[0] != 0 {
            return bad("row_ptr must have n + 1 entries starting at 0");
        }
        if self.row_ptr.windows(2).any(|w| w[1] < w[0]) {
            return bad("row_ptr must be non-decreasing");
        }
        if self.row_ptr[self.n] != self.cols.len() || self.cols.len() != self.vals.len() {
            return bad("row_ptr, cols and vals disagree on the number of entries");
        }
        if self.cols.iter().any(|&c| c >= self.n) {
            return bad("column index out of range");
        }
        Ok(())
    }

    /// Stored value at `(row, col)`, summed over duplicates (`0.0` when
    /// absent).
    pub fn get(&self, row: usize, col: usize) -> f64 {
        let (a, b) = (self.row_ptr[row], self.row_ptr[row + 1]);
        self.cols[a..b]
            .iter()
            .zip(&self.vals[a..b])
            .filter(|&(&c, _)| c == col)
            .fold(0.0, |s, (_, &v)| s + v)
    }

    /// Diagonal of the matrix.
    pub fn diagonal(&self) -> Vec<f64> {
        (0..self.n).map(|i| self.get(i, i)).collect()
    }

    /// `out = A x`, rows in parallel. Each row is summed as `0 + v0 x0 + v1
    /// x1 + ...` in its stored order (`gpu_solve.Csr.apply`), so the result
    /// does not depend on the thread count.
    ///
    /// # Panics
    /// When `x` or `out` do not have `n` entries.
    pub fn mul_vec_into(&self, x: &[f64], out: &mut [f64]) {
        assert_eq!(x.len(), self.n, "x must have n entries");
        assert_eq!(out.len(), self.n, "out must have n entries");
        out.par_chunks_mut(CHUNK)
            .enumerate()
            .for_each(|(chunk, rows)| {
                let first = chunk * CHUNK;
                for (k, o) in rows.iter_mut().enumerate() {
                    let i = first + k;
                    let mut s = 0.0;
                    for e in self.row_ptr[i]..self.row_ptr[i + 1] {
                        s += x[self.cols[e]] * self.vals[e];
                    }
                    *o = s;
                }
            });
    }

    /// `A x` (see [`CsrMatrix::mul_vec_into`]).
    pub fn mul_vec(&self, x: &[f64]) -> Vec<f64> {
        let mut out = vec![0.0; self.n];
        self.mul_vec_into(x, &mut out);
        out
    }
}

/// `frexp` exponent: `x = m 2^e` with `0.5 <= |m| < 1`; `0` for zero and
/// non-finite values (as numpy).
fn frexp_exp(x: f64) -> i32 {
    if x == 0.0 || !x.is_finite() {
        return 0;
    }
    let bits = x.to_bits();
    let e = ((bits >> 52) & 0x7ff) as i32;
    if e == 0 {
        // subnormal: scale into the normal range first
        return frexp_exp(x * f64::powi(2.0, 64)) - 64;
    }
    e - 1022
}

/// `x 2^k` with a single rounding (as C `ldexp`).
fn ldexp(mut x: f64, mut k: i32) -> f64 {
    // steps of at most 2^±1000 keep every intermediate step exact; only the
    // last one can round (into the subnormal range or to infinity)
    while k > 1000 {
        x *= f64::powi(2.0, 1000);
        k -= 1000;
    }
    while k < -1000 {
        x *= f64::powi(2.0, -1000);
        k += 1000;
    }
    x * f64::from_bits(((k + 1023) as u64) << 52)
}

/// Dot product of `a` and `b`, independent of the summation order and of
/// the thread count (`gpu_solve.exact_dot`, the same bits): every product
/// `p` is split on two grids set by `max |p|` and `n` (two-fold
/// pre-rounding), the parts are summed exactly and added with one final
/// rounding.
///
/// # Panics
/// When `a` and `b` have different lengths.
pub fn exact_dot(a: &[f64], b: &[f64]) -> f64 {
    assert_eq!(a.len(), b.len(), "exact_dot: length mismatch");
    let n = a.len();
    if n == 0 {
        return 0.0;
    }
    let m = a
        .par_chunks(CHUNK)
        .zip(b.par_chunks(CHUNK))
        .map(|(x, y)| {
            x.iter()
                .zip(y)
                .fold(0.0f64, |m, (&u, &v)| m.max((u * v).abs()))
        })
        .collect::<Vec<f64>>()
        .into_iter()
        .fold(0.0f64, f64::max);
    let nf = n as f64;
    let e1 = frexp_exp(nf * m);
    let c1 = ldexp(1.5, e1 + 1);
    let c2 = ldexp(1.5, frexp_exp(nf * ldexp(1.0, e1 - 52)) + 1);
    let parts: Vec<(f64, f64)> = a
        .par_chunks(CHUNK)
        .zip(b.par_chunks(CHUNK))
        .map(|(x, y)| {
            let (mut s1, mut s2) = (0.0f64, 0.0f64);
            for (&u, &v) in x.iter().zip(y) {
                let p = u * v;
                let q1 = (p + c1) - c1; // first fold
                let q2 = ((p - q1) + c2) - c2; // second fold
                s1 += q1;
                s2 += q2;
            }
            (s1, s2)
        })
        .collect();
    let (s1, s2) = parts
        .into_iter()
        .fold((0.0, 0.0), |(a1, a2), (b1, b2)| (a1 + b1, a2 + b2));
    s1 + s2
}

/// Conjugate gradient parameters.
#[derive(Debug, Clone, PartialEq)]
pub struct PcgParams {
    /// Maximum number of iterations.
    pub max_iterations: usize,
    /// Relative residual tolerance (`|r| / |b|`, every right-hand side).
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

/// Result of [`pcg_solve_many`] and [`dirichlet_solve`].
#[derive(Debug, Clone, PartialEq)]
pub struct PcgSolution {
    /// One solution vector per right-hand side.
    pub x: Vec<Vec<f64>>,
    /// Iterations performed (the plugin reports one less when it stops at
    /// the iteration limit).
    pub iterations: usize,
    /// Final relative residual `|r| / |b|` of every right-hand side.
    pub residuals: Vec<f64>,
    /// `true` when every residual is below the tolerance.
    pub converged: bool,
}

impl PcgSolution {
    fn into_single(self) -> Result<Vec<f64>> {
        if self.converged {
            Ok(self.x.into_iter().next().unwrap_or_default())
        } else {
            Err(GeomError::NoConvergence {
                iterations: self.iterations,
                residual: self.residuals.first().copied().unwrap_or(f64::NAN),
            })
        }
    }
}

/// Solves `a x = b` for a symmetric positive definite `a`, starting from
/// zero. [`GeomError::NoConvergence`] when the tolerance is not reached.
pub fn pcg_solve(a: &CsrMatrix, b: &[f64], params: &PcgParams) -> Result<Vec<f64>> {
    pcg_solve_warm(a, b, None, params)
}

/// [`pcg_solve`] starting from `x0` (warm start) when given.
pub fn pcg_solve_warm(
    a: &CsrMatrix,
    b: &[f64],
    x0: Option<&[f64]>,
    params: &PcgParams,
) -> Result<Vec<f64>> {
    let x0 = x0.map(|x| vec![x.to_vec()]);
    pcg_solve_many(a, &[b.to_vec()], x0.as_deref(), params)?.into_single()
}

fn axpy_into(out: &mut [f64], f: impl Fn(usize, f64) -> f64 + Sync) {
    out.par_chunks_mut(CHUNK)
        .enumerate()
        .for_each(|(chunk, xs)| {
            let first = chunk * CHUNK;
            for (k, x) in xs.iter_mut().enumerate() {
                *x = f(first + k, *x);
            }
        });
}

/// Jacobi-preconditioned conjugate gradient on several right-hand sides at
/// once (`gpu_solve.repro_csr_cpu`, the same bits): all of them iterate
/// together until every residual is below the tolerance or the iteration
/// limit is reached. `x0` gives a warm start per right-hand side. The
/// preconditioner is `1 / diag(a)` (`1` where the diagonal is not
/// positive). Never fails on non-convergence: check
/// [`PcgSolution::converged`].
pub fn pcg_solve_many(
    a: &CsrMatrix,
    bs: &[Vec<f64>],
    x0: Option<&[Vec<f64>]>,
    params: &PcgParams,
) -> Result<PcgSolution> {
    a.check()?;
    let n = a.n;
    if bs.iter().any(|b| b.len() != n) {
        return Err(GeomError::InvalidInput(format!(
            "right-hand side length differs from the matrix size {n}"
        )));
    }
    let mut xs: Vec<Vec<f64>> = match x0 {
        Some(x0) => {
            if x0.len() != bs.len() || x0.iter().any(|x| x.len() != n) {
                return Err(GeomError::InvalidInput(
                    "warm start does not match the right-hand sides".into(),
                ));
            }
            x0.to_vec()
        }
        None => vec![vec![0.0; n]; bs.len()],
    };
    let minv: Vec<f64> = a
        .diagonal()
        .into_iter()
        .map(|d| 1.0 / if d > 0.0 { d } else { 1.0 })
        .collect();
    let l = bs.len();
    let mut rs: Vec<Vec<f64>> = Vec::with_capacity(l);
    let mut zs: Vec<Vec<f64>> = Vec::with_capacity(l);
    let mut rz = Vec::with_capacity(l);
    let mut bn = Vec::with_capacity(l);
    let mut res = Vec::with_capacity(l);
    for (b, x) in bs.iter().zip(&xs) {
        let mut r = a.mul_vec(x);
        axpy_into(&mut r, |i, ax| b[i] - ax);
        let mut z = r.clone();
        axpy_into(&mut z, |i, r| minv[i] * r);
        rz.push(exact_dot(&r, &z));
        let nb = exact_dot(b, b).sqrt();
        let nb = if nb == 0.0 { 1.0 } else { nb };
        bn.push(nb);
        res.push(exact_dot(&r, &r).sqrt() / nb);
        rs.push(r);
        zs.push(z);
    }
    let mut ps = zs.clone();
    let mut ap = vec![0.0; n];
    let mut it = 0usize;
    let converged = loop {
        if res.iter().all(|&r| r < params.tolerance) {
            break true;
        }
        if it >= params.max_iterations {
            break false;
        }
        for c in 0..l {
            let (p, r, z, x) = (&mut ps[c], &mut rs[c], &mut zs[c], &mut xs[c]);
            a.mul_vec_into(p, &mut ap);
            let pap = exact_dot(p, &ap);
            let alpha = rz[c] / if pap.abs() > 1e-300 { pap } else { 1.0 };
            axpy_into(x, |i, x| x + alpha * p[i]);
            axpy_into(r, |i, r| r - alpha * ap[i]);
            {
                let r: &[f64] = r;
                axpy_into(z, |i, _| minv[i] * r[i]);
            }
            res[c] = exact_dot(r, r).sqrt() / bn[c];
            let rzn = exact_dot(r, z);
            let beta = rzn / if rz[c].abs() > 1e-300 { rz[c] } else { 1.0 };
            rz[c] = rzn;
            let z: &[f64] = z;
            axpy_into(p, |i, p| p * beta + z[i]);
        }
        it += 1;
    };
    Ok(PcgSolution {
        x: xs,
        iterations: it,
        residuals: res,
        converged,
    })
}

/// Solves `a x = rhs` with the entries `fixed` of `x` prescribed (Dirichlet
/// constraints), for every field of `fields` (one or more right-hand sides
/// sharing the same constrained set).
///
/// Each field is a full-length vector: its value at a fixed entry is the
/// constraint, its value at a free entry the warm start. The free unknowns
/// solve `A_ff x_f = rhs_f - A_fc x_c` with [`pcg_solve_many`]; `a` must
/// be symmetric and `A_ff` positive definite (e.g. a Laplacian with at
/// least one fixed vertex per connected component). The constraint term of
/// row `i` is summed as in the plugin (`reconstruct.gum_weights`): the
/// fixed columns above `i`, then those below, each in column order, then
/// added. `rhs` is optional (zero: Laplace equation). Returns full-length
/// solutions (fixed entries keep their values).
pub fn dirichlet_solve(
    a: &CsrMatrix,
    fixed: &[usize],
    fields: &[Vec<f64>],
    rhs: Option<&[Vec<f64>]>,
    params: &PcgParams,
) -> Result<PcgSolution> {
    a.check()?;
    let n = a.n;
    if let Some(&i) = fixed.iter().find(|&&i| i >= n) {
        return Err(GeomError::InvalidInput(format!(
            "fixed index {i} out of range ({n} unknowns)"
        )));
    }
    if fields.iter().any(|f| f.len() != n) {
        return Err(GeomError::InvalidInput(
            "field length differs from the matrix size".into(),
        ));
    }
    if let Some(rhs) = rhs
        && (rhs.len() != fields.len() || rhs.iter().any(|r| r.len() != n))
    {
        return Err(GeomError::InvalidInput(
            "right-hand sides do not match the fields".into(),
        ));
    }
    let mut is_fixed = vec![false; n];
    for &i in fixed {
        is_fixed[i] = true;
    }
    let free: Vec<usize> = (0..n).filter(|&i| !is_fixed[i]).collect();
    let mut pos = vec![usize::MAX; n];
    for (k, &i) in free.iter().enumerate() {
        pos[i] = k;
    }
    // A_ff: free rows and columns, column order kept (pos is monotone)
    let mut row_ptr = Vec::with_capacity(free.len() + 1);
    row_ptr.push(0);
    let (mut cols, mut vals) = (Vec::new(), Vec::new());
    for &i in &free {
        for e in a.row_ptr[i]..a.row_ptr[i + 1] {
            let c = a.cols[e];
            if !is_fixed[c] {
                cols.push(pos[c]);
                vals.push(a.vals[e]);
            }
        }
        row_ptr.push(cols.len());
    }
    let aff = CsrMatrix {
        n: free.len(),
        row_ptr,
        cols,
        vals,
    };
    let mut bs = Vec::with_capacity(fields.len());
    let mut x0 = Vec::with_capacity(fields.len());
    for (k, field) in fields.iter().enumerate() {
        let b: Vec<f64> = free
            .par_iter()
            .with_min_len(CHUNK)
            .map(|&i| {
                let (mut hi, mut lo) = (0.0f64, 0.0f64);
                for e in a.row_ptr[i]..a.row_ptr[i + 1] {
                    let c = a.cols[e];
                    if is_fixed[c] {
                        let t = -a.vals[e] * field[c];
                        if c > i {
                            hi += t;
                        } else {
                            lo += t;
                        }
                    }
                }
                let bc = hi + lo;
                match rhs {
                    Some(rhs) => rhs[k][i] + bc,
                    None => bc,
                }
            })
            .collect();
        bs.push(b);
        x0.push(free.iter().map(|&i| field[i]).collect::<Vec<f64>>());
    }
    let sol = pcg_solve_many(&aff, &bs, Some(&x0), params)?;
    let x = fields
        .iter()
        .zip(sol.x)
        .map(|(field, xf)| {
            let mut full = field.clone();
            for (k, &i) in free.iter().enumerate() {
                full[i] = xf[k];
            }
            full
        })
        .collect();
    Ok(PcgSolution { x, ..sol })
}

/// Harmonic interpolation over a mesh: values fixed at `constrained`
/// vertices, smooth (clamped cotangent Laplacian, [`crate::cotan`])
/// everywhere else. A vertex listed twice takes its last value.
/// [`GeomError::NoConvergence`] when the tolerance is not reached.
pub fn harmonic(
    mesh: &dc_mesh::TriMesh,
    constrained: &[(u32, f64)],
    params: &PcgParams,
) -> Result<Vec<f64>> {
    let n = mesh.vertex_count();
    let mut field = vec![0.0; n];
    let mut fixed = Vec::with_capacity(constrained.len());
    for &(v, value) in constrained {
        let v = v as usize;
        if v >= n {
            return Err(GeomError::InvalidInput(format!(
                "constrained vertex {v} out of range ({n} vertices)"
            )));
        }
        field[v] = value;
        fixed.push(v);
    }
    crate::cotan::harmonic_field(mesh, &fixed, &[field], params)?.into_single()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 1-D Laplacian `tridiag(-1, 2, -1)` of size `n`.
    fn lap1d(n: usize) -> CsrMatrix {
        let mut t = Vec::new();
        for i in 0..n {
            t.push((i, i, 2.0));
            if i > 0 {
                t.push((i, i - 1, -1.0));
            }
            if i + 1 < n {
                t.push((i, i + 1, -1.0));
            }
        }
        CsrMatrix::from_triplets(n, &t).unwrap()
    }

    #[test]
    fn triplets_sorted_and_summed() {
        let m = CsrMatrix::from_triplets(
            3,
            &[
                (2, 0, 1.0),
                (0, 1, 2.0),
                (0, 1, 3.0),
                (1, 1, 4.0),
                (0, 0, 5.0),
            ],
        )
        .unwrap();
        assert_eq!(m.row_ptr, vec![0, 2, 3, 4]);
        assert_eq!(m.cols, vec![0, 1, 1, 0]);
        assert_eq!(m.vals, vec![5.0, 5.0, 4.0, 1.0]);
        assert_eq!(m.get(0, 1), 5.0);
        assert_eq!(m.get(2, 2), 0.0);
        assert_eq!(m.diagonal(), vec![5.0, 4.0, 0.0]);
        m.check().unwrap();
        assert!(CsrMatrix::from_triplets(2, &[(2, 0, 1.0)]).is_err());
    }

    #[test]
    fn spmv_matches_dense() {
        let a = lap1d(5);
        let x = [1.0, 2.0, 4.0, 8.0, 16.0];
        assert_eq!(a.mul_vec(&x), vec![0.0, -1.0, -2.0, -4.0, 24.0]);
    }

    #[test]
    fn frexp_ldexp() {
        assert_eq!(frexp_exp(1.0), 1);
        assert_eq!(frexp_exp(0.5), 0);
        assert_eq!(frexp_exp(3.0), 2);
        assert_eq!(frexp_exp(0.0), 0);
        assert_eq!(frexp_exp(f64::MIN_POSITIVE), -1021);
        assert_eq!(frexp_exp(f64::MIN_POSITIVE / 4.0), -1023);
        assert_eq!(ldexp(1.5, 3), 12.0);
        assert_eq!(ldexp(1.0, -1074), f64::from_bits(1));
        assert_eq!(ldexp(1.0, 1023), f64::powi(2.0, 1023));
    }

    #[test]
    fn exact_dot_is_order_independent() {
        let a: Vec<f64> = (0..10_000)
            .map(|i| ((i * 7919) % 1013) as f64 * 1e-3 - 0.4)
            .collect();
        let b: Vec<f64> = (0..10_000).map(|i| (i as f64).sin() * 1e5).collect();
        let d = exact_dot(&a, &b);
        let mut ra = a.clone();
        let mut rb = b.clone();
        ra.reverse();
        rb.reverse();
        assert_eq!(d.to_bits(), exact_dot(&ra, &rb).to_bits());
        let naive: f64 = a.iter().zip(&b).map(|(x, y)| x * y).sum();
        assert!((d - naive).abs() <= 1e-9 * naive.abs().max(1.0));
        assert_eq!(exact_dot(&[], &[]), 0.0);
        assert_eq!(exact_dot(&[0.0, 0.0], &[1.0, 2.0]), 0.0);
    }

    #[test]
    fn pcg_solves_1d_poisson() {
        // -u'' = 0 with u(0)=0 and u(n+1)=1 folded into b: linear solution
        let n = 50;
        let a = lap1d(n);
        let mut b = vec![0.0; n];
        b[n - 1] = 1.0;
        let x = pcg_solve(&a, &b, &PcgParams::default()).unwrap();
        for (i, xi) in x.iter().enumerate() {
            assert!((xi - (i + 1) as f64 / (n + 1) as f64).abs() < 1e-7);
        }
    }

    #[test]
    fn warm_start_and_many() {
        let n = 30;
        let a = lap1d(n);
        let b1: Vec<f64> = (0..n).map(|i| (i as f64 * 0.3).cos()).collect();
        let b2: Vec<f64> = vec![1.0; n];
        let p = PcgParams {
            max_iterations: 500,
            tolerance: 1e-12,
        };
        let s = pcg_solve_many(&a, &[b1.clone(), b2.clone()], None, &p).unwrap();
        assert!(s.converged);
        for (x, b) in s.x.iter().zip([&b1, &b2]) {
            let r = a.mul_vec(x);
            for (ri, bi) in r.iter().zip(b) {
                assert!((ri - bi).abs() < 1e-9);
            }
        }
        // warm start at the solution: converged without iterating
        let w = pcg_solve_many(&a, &[b1], Some(&s.x[..1]), &p).unwrap();
        assert!(w.iterations <= 1);
        // iteration limit reached
        let few = PcgParams {
            max_iterations: 2,
            tolerance: 1e-12,
        };
        assert!(matches!(
            pcg_solve(&a, &b2, &few),
            Err(GeomError::NoConvergence { iterations: 2, .. })
        ));
        assert!(pcg_solve(&a, &[1.0], &p).is_err());
    }

    #[test]
    fn dirichlet_on_a_path() {
        // graph Laplacian of a path, ends fixed: linear interpolation
        let n = 11;
        let mut t = Vec::new();
        for i in 0..n - 1 {
            t.extend([
                (i, i, 1.0),
                (i + 1, i + 1, 1.0),
                (i, i + 1, -1.0),
                (i + 1, i, -1.0),
            ]);
        }
        let a = CsrMatrix::from_triplets(n, &t).unwrap();
        let mut f = vec![0.0; n];
        f[0] = 2.0;
        f[n - 1] = 12.0;
        let p = PcgParams {
            max_iterations: 200,
            tolerance: 1e-13,
        };
        let s = dirichlet_solve(&a, &[0, n - 1], &[f], None, &p).unwrap();
        assert!(s.converged);
        for (i, x) in s.x[0].iter().enumerate() {
            assert!((x - (2.0 + i as f64)).abs() < 1e-10, "{i}: {x}");
        }
        assert!(dirichlet_solve(&a, &[n], &[vec![0.0; n]], None, &p).is_err());
    }
}
