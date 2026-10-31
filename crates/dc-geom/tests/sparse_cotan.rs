//! Sparse solver and cotangent Laplacian (WP17): analytic solutions on a
//! triangular lattice, bit-identical results across rayon pool sizes, and
//! bit-for-bit agreement with the plugin oracle (`gpu_solve.repro_csr_cpu`,
//! `reconstruct.cotan_weights`).
//!
//! `fixtures/sparse_oracle.txt` was written once by running the plugin's
//! own functions (loaded read-only from `blender-ortho-aligner`) on a
//! jittered lattice: cotangent weights, the harmonic system that
//! `reconstruct.gum_weights` builds (two fields, warm start), its
//! `repro_csr_cpu` solutions, and a `csr_from_coo` with duplicates.

use std::collections::HashMap;

use dc_geom::cotan;
use dc_geom::sparse::{self, CsrMatrix, PcgParams};
use dc_mesh::TriMesh;
use glam::Vec3;

/// Equilateral triangular lattice, `rows x cols` triangles strips, edge
/// `h`, optional deterministic jitter. Returns the mesh and a boundary
/// flag per vertex.
fn lattice(rows: usize, cols: usize, h: f32, jitter: f32) -> (TriMesh, Vec<bool>) {
    let idx = |j: usize, i: usize| (j * (cols + 1) + i) as u32;
    let mut pos = Vec::new();
    let mut boundary = Vec::new();
    let mut seed = 0x9e37_79b9_u32;
    let mut rnd = || {
        seed ^= seed << 13;
        seed ^= seed >> 17;
        seed ^= seed << 5;
        seed as f32 / u32::MAX as f32 - 0.5
    };
    for j in 0..=rows {
        for i in 0..=cols {
            let x = (i as f32 + 0.5 * (j % 2) as f32) * h;
            let y = j as f32 * h * 3f32.sqrt() / 2.0;
            pos.push(Vec3::new(
                x + jitter * rnd(),
                y + jitter * rnd(),
                jitter * rnd(),
            ));
            boundary.push(j == 0 || j == rows || i == 0 || i == cols);
        }
    }
    let mut tris = Vec::new();
    for j in 0..rows {
        for i in 0..cols {
            if j % 2 == 0 {
                tris.push([idx(j, i), idx(j, i + 1), idx(j + 1, i)]);
                tris.push([idx(j, i + 1), idx(j + 1, i + 1), idx(j + 1, i)]);
            } else {
                tris.push([idx(j, i), idx(j, i + 1), idx(j + 1, i + 1)]);
                tris.push([idx(j, i), idx(j + 1, i + 1), idx(j + 1, i)]);
            }
        }
    }
    (TriMesh::new(pos, tris), boundary)
}

fn tight() -> PcgParams {
    PcgParams {
        max_iterations: 5000,
        tolerance: 1e-12,
    }
}

#[test]
fn laplace_reproduces_a_harmonic_quadratic() {
    // x^2 - y^2 + linear terms is harmonic; on the equilateral lattice the
    // cotangent Laplacian is exact for quadratics, so the discrete solution
    // equals the analytic one up to f32 positions and solver tolerance
    let (mesh, boundary) = lattice(16, 20, 1.0, 0.0);
    let u = |p: Vec3| {
        let (x, y) = (f64::from(p.x), f64::from(p.y));
        x * x - y * y + 0.5 * x + 0.25 * y
    };
    let constrained: Vec<(u32, f64)> = boundary
        .iter()
        .enumerate()
        .filter(|&(_, &b)| b)
        .map(|(i, _)| (i as u32, u(mesh.positions[i])))
        .collect();
    let x = sparse::harmonic(&mesh, &constrained, &tight()).unwrap();
    let mut worst = 0.0f64;
    for (i, p) in mesh.positions.iter().enumerate() {
        worst = worst.max((x[i] - u(*p)).abs());
    }
    assert!(worst < 1e-4, "max error {worst}");
}

#[test]
fn poisson_with_mass_matrix() {
    // -Δu = -4 for u = x^2 + y^2: L u = M f with the lumped mass
    let (mesh, boundary) = lattice(14, 18, 0.5, 0.0);
    let u = |p: Vec3| f64::from(p.x).powi(2) + f64::from(p.y).powi(2);
    let l = cotan::cotan_laplacian(&mesh);
    let m = cotan::mass_barycentric(&mesh);
    let rhs: Vec<f64> = m.iter().map(|mi| -4.0 * mi).collect();
    let fixed: Vec<usize> = (0..boundary.len()).filter(|&i| boundary[i]).collect();
    let field: Vec<f64> = mesh
        .positions
        .iter()
        .zip(&boundary)
        .map(|(p, &b)| if b { u(*p) } else { 0.0 })
        .collect();
    let s = sparse::dirichlet_solve(&l, &fixed, &[field], Some(&[rhs]), &tight()).unwrap();
    assert!(s.converged);
    let worst = mesh
        .positions
        .iter()
        .zip(&s.x[0])
        .map(|(p, x)| (x - u(*p)).abs())
        .fold(0.0, f64::max);
    assert!(worst < 1e-4, "max error {worst}");
}

#[test]
fn bit_identical_across_thread_counts() {
    // more than two CHUNKs of unknowns, so the parallel paths really split
    let (mesh, boundary) = lattice(90, 110, 0.4, 0.15);
    assert!(mesh.vertex_count() > 2 * sparse::CHUNK);
    let l = cotan::cotan_laplacian(&mesh);
    let fixed: Vec<usize> = (0..boundary.len()).filter(|&i| boundary[i]).collect();
    let fields: Vec<Vec<f64>> = (0..2)
        .map(|k| {
            mesh.positions
                .iter()
                .zip(&boundary)
                .map(|(p, &b)| {
                    if b {
                        (f64::from(p.x) * (k + 1) as f64).sin() + f64::from(p.y)
                    } else {
                        0.0
                    }
                })
                .collect()
        })
        .collect();
    let params = PcgParams {
        max_iterations: 300,
        tolerance: 1e-9,
    };
    let run = |threads: usize| {
        rayon::ThreadPoolBuilder::new()
            .num_threads(threads)
            .build()
            .unwrap()
            .install(|| {
                let lap = cotan::cotan_laplacian(&mesh);
                assert_eq!(lap, l);
                let s = sparse::dirichlet_solve(&lap, &fixed, &fields, None, &params).unwrap();
                let dot = sparse::exact_dot(&s.x[0], &s.x[1]);
                (s, dot)
            })
    };
    let (base, dot) = run(1);
    assert!(base.iterations > 10);
    for threads in [2, 8] {
        let (s, d) = run(threads);
        assert_eq!(s.iterations, base.iterations);
        assert_eq!(d.to_bits(), dot.to_bits());
        for (a, b) in s.x.iter().flatten().zip(base.x.iter().flatten()) {
            assert_eq!(a.to_bits(), b.to_bits(), "{threads} threads");
        }
        for (a, b) in s.residuals.iter().zip(&base.residuals) {
            assert_eq!(a.to_bits(), b.to_bits());
        }
    }
}

struct Fixture(HashMap<String, Vec<String>>);

impl Fixture {
    fn load() -> Self {
        let text = include_str!("fixtures/sparse_oracle.txt");
        let map = text
            .lines()
            .filter(|l| !l.starts_with('#') && !l.trim().is_empty())
            .map(|l| {
                let (k, v) = l.split_once(':').expect("key: values");
                (
                    k.to_string(),
                    v.split_whitespace().map(str::to_string).collect(),
                )
            })
            .collect();
        Self(map)
    }

    fn f64s(&self, key: &str) -> Vec<f64> {
        self.0[key]
            .iter()
            .map(|s| f64::from_bits(u64::from_str_radix(s, 16).unwrap()))
            .collect()
    }

    fn ints(&self, key: &str) -> Vec<usize> {
        self.0[key].iter().map(|s| s.parse().unwrap()).collect()
    }
}

fn assert_bits(got: &[f64], want: &[f64], what: &str) {
    assert_eq!(got.len(), want.len(), "{what}: length");
    for (i, (g, w)) in got.iter().zip(want).enumerate() {
        assert_eq!(g.to_bits(), w.to_bits(), "{what}[{i}]: {g} vs {w}");
    }
}

#[test]
fn matches_the_plugin_oracle_bit_for_bit() {
    let fx = Fixture::load();
    let p = fx.f64s("positions");
    let positions: Vec<Vec3> = p
        .chunks(3)
        .map(|c| Vec3::new(c[0] as f32, c[1] as f32, c[2] as f32))
        .collect();
    let tris: Vec<[u32; 3]> = fx
        .ints("tris")
        .chunks(3)
        .map(|c| [c[0] as u32, c[1] as u32, c[2] as u32])
        .collect();
    let mesh = TriMesh::new(positions, tris);

    let edges = cotan::unique_edges(&mesh);
    let want_edges: Vec<[u32; 2]> = fx
        .ints("edges")
        .chunks(2)
        .map(|c| [c[0] as u32, c[1] as u32])
        .collect();
    assert_eq!(edges, want_edges);
    let w = cotan::cotan_weights(&mesh, &edges, cotan::COT_MIN, cotan::COT_MAX);
    assert_bits(&w, &fx.f64s("cotan_weights"), "cotan weights");

    let fixed = fx.ints("fixed");
    let params = PcgParams {
        max_iterations: fx.ints("maxiter")[0],
        tolerance: fx.f64s("tol")[0],
    };
    let f0 = fx.f64s("field0");
    let f1 = fx.f64s("field1");

    let single = cotan::harmonic_field(&mesh, &fixed, std::slice::from_ref(&f0), &params).unwrap();
    assert!(single.converged);
    assert_eq!(single.iterations, fx.ints("single_it")[0]);
    assert_bits(&single.x[0], &fx.f64s("single_x"), "single solve");
    assert_bits(&single.residuals, &fx.f64s("single_res"), "single residual");

    let many = cotan::harmonic_field(&mesh, &fixed, &[f0, f1], &params).unwrap();
    assert!(many.converged);
    assert_eq!(many.iterations, fx.ints("many_it")[0]);
    assert_bits(&many.x[0], &fx.f64s("many_x0"), "joint solve, field 0");
    assert_bits(&many.x[1], &fx.f64s("many_x1"), "joint solve, field 1");
    assert_bits(&many.residuals, &fx.f64s("many_res"), "joint residuals");

    let rows = fx.ints("coo_rows");
    let cols = fx.ints("coo_cols");
    let vals = fx.f64s("coo_vals");
    let t: Vec<(usize, usize, f64)> = (0..rows.len())
        .map(|k| (rows[k], cols[k], vals[k]))
        .collect();
    let csr = CsrMatrix::from_triplets(6, &t).unwrap();
    assert_eq!(csr.row_ptr, fx.ints("csr_ptr"));
    assert_eq!(csr.cols, fx.ints("csr_cols"));
    assert_bits(&csr.vals, &fx.f64s("csr_vals"), "csr_from_coo values");
}
