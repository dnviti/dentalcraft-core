//! Cotangent Laplacian and mass matrix of a triangle mesh, for harmonic
//! fields and smoothing (WP17, port plan B1.3), solved with [`crate::sparse`].
//!
//! Ported from the plugin (`reconstruct.cotan_weights`,
//! `reconstruct.gum_weights`, `gpu_solve.laplacian_edges`), with the same
//! operation order, so the weights and matrices are bit-identical to the
//! oracle. Laplacians are positive semi-definite: `L = D - W`, with `W` the
//! symmetric edge weights and `D` their row sums.

use dc_mesh::TriMesh;

use crate::Result;
use crate::sparse::{self, CsrMatrix, PcgParams, PcgSolution};

/// Lower clamp of the cotangent edge weights (plugin `COT_MIN`): positive
/// weights keep harmonic fields within the range of their constraints
/// (maximum principle).
pub const COT_MIN: f64 = 0.05;
/// Upper clamp of the cotangent edge weights (plugin `COT_MAX`): bounds the
/// conditioning on slivers.
pub const COT_MAX: f64 = 10.0;

/// Undirected edges of the mesh, each once as `[min, max]`, sorted in
/// lexicographic order (plugin `meshutil.unique_edges`).
pub fn unique_edges(mesh: &TriMesh) -> Vec<[u32; 2]> {
    let mut e: Vec<[u32; 2]> = mesh
        .tris
        .iter()
        .flat_map(|t| [[t[0], t[1]], [t[1], t[2]], [t[2], t[0]]])
        .map(|[a, b]| [a.min(b), a.max(b)])
        .collect();
    e.sort_unstable();
    e.dedup();
    e
}

/// Cotangent weights `(cot a + cot b) / 2` of the `edges` (from
/// [`unique_edges`]), clamped to `[lo, hi]` (Pinkall and Polthier 1993;
/// plugin `reconstruct.cotan_weights`). A boundary edge gets its single
/// cotangent halved. Summed in the plugin's order: one pass per triangle
/// corner, triangles in order.
///
/// # Panics
/// When a triangle edge is missing from `edges` or a vertex index is out of
/// range.
pub fn cotan_weights(mesh: &TriMesh, edges: &[[u32; 2]], lo: f64, hi: f64) -> Vec<f64> {
    let v: Vec<[f64; 3]> = mesh
        .positions
        .iter()
        .map(|p| [f64::from(p.x), f64::from(p.y), f64::from(p.z)])
        .collect();
    let mut w = vec![0.0f64; edges.len()];
    let mut acc = vec![0.0f64; edges.len()];
    for c in 0..3 {
        acc.iter_mut().for_each(|a| *a = 0.0);
        for t in &mesh.tris {
            let (i0, i1, i2) = (t[c], t[(c + 1) % 3], t[(c + 2) % 3]);
            let (p0, p1, p2) = (v[i0 as usize], v[i1 as usize], v[i2 as usize]);
            let u = [p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]];
            let q = [p2[0] - p0[0], p2[1] - p0[1], p2[2] - p0[2]];
            let dot = u[0] * q[0] + u[1] * q[1] + u[2] * q[2];
            let x = [
                u[1] * q[2] - u[2] * q[1],
                u[2] * q[0] - u[0] * q[2],
                u[0] * q[1] - u[1] * q[0],
            ];
            let norm = (x[0] * x[0] + x[1] * x[1] + x[2] * x[2]).sqrt();
            let cot = dot / norm.max(1e-12);
            let key = [i1.min(i2), i1.max(i2)];
            let k = edges
                .binary_search(&key)
                .expect("triangle edge missing from the edge list");
            acc[k] += cot;
        }
        for (wi, a) in w.iter_mut().zip(&acc) {
            *wi += 0.5 * a;
        }
    }
    w.into_iter().map(|x| x.clamp(lo, hi)).collect()
}

/// Laplacian `D - W` of `n` vertices from symmetric edge weights (`edges`
/// sorted as [`unique_edges`]). The degree of a vertex is summed as the
/// plugin does: the edges where it is the first endpoint, then those where
/// it is the second, each in edge order, then added.
///
/// # Panics
/// When `weights` and `edges` differ in length or an index is out of range.
pub fn laplacian_from_edges(n: usize, edges: &[[u32; 2]], weights: &[f64]) -> CsrMatrix {
    assert_eq!(edges.len(), weights.len(), "one weight per edge");
    let (mut da, mut db) = (vec![0.0f64; n], vec![0.0f64; n]);
    for (e, &w) in edges.iter().zip(weights) {
        da[e[0] as usize] += w;
        db[e[1] as usize] += w;
    }
    let mut t = Vec::with_capacity(n + 2 * edges.len());
    t.extend((0..n).map(|i| (i, i, da[i] + db[i])));
    for (e, &w) in edges.iter().zip(weights) {
        t.push((e[0] as usize, e[1] as usize, -w));
    }
    for (e, &w) in edges.iter().zip(weights) {
        t.push((e[1] as usize, e[0] as usize, -w));
    }
    CsrMatrix::from_triplets(n, &t).expect("edge indices within the vertex count")
}

/// Cotangent Laplacian of the mesh, weights clamped to
/// [`COT_MIN`]..[`COT_MAX`] as in the plugin.
pub fn cotan_laplacian(mesh: &TriMesh) -> CsrMatrix {
    let edges = unique_edges(mesh);
    let w = cotan_weights(mesh, &edges, COT_MIN, COT_MAX);
    laplacian_from_edges(mesh.vertex_count(), &edges, &w)
}

/// Uniform (umbrella) Laplacian: every edge weighs 1 (plugin
/// `gpu_solve.laplacian_edges`).
pub fn uniform_laplacian(mesh: &TriMesh) -> CsrMatrix {
    let edges = unique_edges(mesh);
    let w = vec![1.0; edges.len()];
    laplacian_from_edges(mesh.vertex_count(), &edges, &w)
}

/// Lumped barycentric mass matrix (its diagonal): a third of the area of
/// every incident triangle. The plugin has no mass matrix (its harmonic
/// fields solve the plain Laplace equation); this one pairs with
/// [`cotan_laplacian`] for Poisson problems, `L u = M f` discretising
/// `-Δu = f`.
pub fn mass_barycentric(mesh: &TriMesh) -> Vec<f64> {
    let mut m = vec![0.0f64; mesh.vertex_count()];
    for t in &mesh.tris {
        let p: Vec<[f64; 3]> = t
            .iter()
            .map(|&i| {
                let q = mesh.positions[i as usize];
                [f64::from(q.x), f64::from(q.y), f64::from(q.z)]
            })
            .collect();
        let u = [p[1][0] - p[0][0], p[1][1] - p[0][1], p[1][2] - p[0][2]];
        let v = [p[2][0] - p[0][0], p[2][1] - p[0][1], p[2][2] - p[0][2]];
        let x = [
            u[1] * v[2] - u[2] * v[1],
            u[2] * v[0] - u[0] * v[2],
            u[0] * v[1] - u[1] * v[0],
        ];
        let third = (x[0] * x[0] + x[1] * x[1] + x[2] * x[2]).sqrt() / 6.0;
        for &i in t {
            m[i as usize] += third;
        }
    }
    m
}

/// Harmonic fields over the mesh: the Laplace equation with the clamped
/// cotangent Laplacian, values prescribed at the `fixed` vertices
/// (plugin `reconstruct.gum_weights`). Each field is a full-length vector:
/// its values at fixed vertices are the constraints, at free vertices the
/// warm start. Every connected component needs at least one fixed vertex.
/// See [`sparse::dirichlet_solve`].
pub fn harmonic_field(
    mesh: &TriMesh,
    fixed: &[usize],
    fields: &[Vec<f64>],
    params: &PcgParams,
) -> Result<PcgSolution> {
    sparse::dirichlet_solve(&cotan_laplacian(mesh), fixed, fields, None, params)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn edges_of_a_quad() {
        let m = TriMesh::new(
            vec![
                glam::Vec3::ZERO,
                glam::Vec3::X,
                glam::Vec3::ONE,
                glam::Vec3::Y,
            ],
            vec![[0, 1, 2], [0, 2, 3]],
        );
        assert_eq!(
            unique_edges(&m),
            vec![[0, 1], [0, 2], [0, 3], [1, 2], [2, 3]]
        );
    }

    #[test]
    fn right_triangle_weights_and_clamp() {
        // unit right triangle: angles 90 (at 0), 45, 45
        let m = TriMesh::new(
            vec![glam::Vec3::ZERO, glam::Vec3::X, glam::Vec3::Y],
            vec![[0, 1, 2]],
        );
        let e = unique_edges(&m);
        let w = cotan_weights(&m, &e, f64::NEG_INFINITY, f64::INFINITY);
        // edges (0,1) and (0,2) face a 45 degree angle, (1,2) the right angle
        assert!((w[0] - 0.5).abs() < 1e-12);
        assert!((w[1] - 0.5).abs() < 1e-12);
        assert!(w[2].abs() < 1e-12);
        let wc = cotan_weights(&m, &e, COT_MIN, COT_MAX);
        assert_eq!(wc[2], COT_MIN);
        let l = laplacian_from_edges(3, &e, &wc);
        let ones = l.mul_vec(&[1.0; 3]);
        assert!(ones.iter().all(|x| x.abs() < 1e-15));
        assert_eq!(l.get(1, 2), -COT_MIN);
        let mass = mass_barycentric(&m);
        assert!(mass.iter().all(|x| (x - 1.0 / 6.0).abs() < 1e-15));
        let u = uniform_laplacian(&m);
        assert_eq!(u.diagonal(), vec![2.0; 3]);
    }
}
