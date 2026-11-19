//! Feature-preserving denoising of a scanned surface.
//!
//! 1. The face normals are filtered bilaterally (Zheng et al. 2011,
//!    "Bilateral normal filtering for mesh denoising"): every normal becomes
//!    the area-weighted mean of the normals of the faces around its
//!    vertices, weighted by a Gaussian of the distance between the centroids
//!    (spatial, the mean such distance) and of the difference between the
//!    normals (range, [`DenoiseParams::sigma_normal`]). Faces across a crease
//!    differ in normal and barely mix: edges and cusps stay sharp while the
//!    grain of the scan is flattened.
//! 2. The vertices are moved to fit the filtered normals (Sun et al. 2007):
//!    every vertex goes toward the planes of its faces through their
//!    centroids. No vertex moves farther than
//!    [`DenoiseParams::max_displacement`] from where it was, and border
//!    vertices stay.
//!
//! Every step is computed in parallel from the previous one, so the result
//! does not depend on the number of threads.

use dc_mesh::TriMesh;
use glam::Vec3;
use rayon::prelude::*;

use crate::{GeomError, Result};

/// Parameters of [`denoise`].
#[derive(Debug, Clone, PartialEq)]
pub struct DenoiseParams {
    /// Rounds of normal filtering.
    pub normal_iterations: usize,
    /// Rounds of vertex fitting.
    pub vertex_iterations: usize,
    /// Range of the normal weight (difference of unit normals): smaller
    /// keeps more creases.
    pub sigma_normal: f32,
    /// Largest distance a vertex may move (mm).
    pub max_displacement: f32,
}

impl Default for DenoiseParams {
    fn default() -> Self {
        Self {
            normal_iterations: 4,
            vertex_iterations: 8,
            sigma_normal: 0.35,
            max_displacement: 0.01,
        }
    }
}

/// Denoises `mesh` in place (see the module doc); returns the largest
/// distance a vertex moved (mm).
///
/// # Errors
/// [`GeomError::InvalidInput`] for parameters out of range; kernel errors
/// for an inconsistent mesh.
pub fn denoise(mesh: &mut TriMesh, params: &DenoiseParams) -> Result<f32> {
    if !(params.sigma_normal > 0.0 && params.max_displacement >= 0.0) {
        return Err(GeomError::InvalidInput(
            "sigma_normal must be > 0 and max_displacement >= 0".into(),
        ));
    }
    mesh.check()?;
    if mesh.tris.is_empty() || params.max_displacement == 0.0 {
        return Ok(0.0);
    }
    let (offsets, vfaces) = crate::finish::vertex_faces(mesh);
    let ring = |v: u32| &vfaces[offsets[v as usize] as usize..offsets[v as usize + 1] as usize];
    let border = crate::finish::border_vertices(mesh);
    let tris = &mesh.tris;
    let start = mesh.positions.clone();

    // centroids, areas and unit normals
    let geometry = |pos: &[Vec3]| -> (Vec<Vec3>, Vec<f32>, Vec<Vec3>) {
        let mut c = Vec::new();
        let mut a = Vec::new();
        let mut n = Vec::new();
        tris.par_iter()
            .map(|t| {
                let [p, q, r] = t.map(|v| pos[v as usize]);
                let cr = (q - p).cross(r - p);
                ((p + q + r) / 3.0, 0.5 * cr.length(), cr.normalize_or_zero())
            })
            .collect::<Vec<_>>()
            .into_iter()
            .for_each(|(ci, ai, ni)| {
                c.push(ci);
                a.push(ai);
                n.push(ni);
            });
        (c, a, n)
    };
    let (centroid, area, mut normal) = geometry(&mesh.positions);

    // spatial range: the mean distance between neighbouring centroids
    let (sum, count) = tris
        .par_iter()
        .enumerate()
        .map(|(f, t)| {
            let mut s = 0.0f64;
            let mut k = 0usize;
            for &v in t {
                for &g in ring(v) {
                    if g as usize != f {
                        s += f64::from(centroid[f].distance(centroid[g as usize]));
                        k += 1;
                    }
                }
            }
            (s, k)
        })
        .reduce(|| (0.0, 0), |x, y| (x.0 + y.0, x.1 + y.1));
    let sigma_c = if count > 0 {
        (sum / count as f64) as f32
    } else {
        0.0
    };
    if sigma_c <= 0.0 {
        return Ok(0.0);
    }
    let (ws, wr) = (
        -0.5 / (sigma_c * sigma_c),
        -0.5 / (params.sigma_normal * params.sigma_normal),
    );

    // 1. bilateral filtering of the normals (centroids fixed)
    for _ in 0..params.normal_iterations {
        normal = tris
            .par_iter()
            .enumerate()
            .map(|(f, t)| {
                let (cf, nf) = (centroid[f], normal[f]);
                let mut acc = Vec3::ZERO;
                for &v in t {
                    for &g in ring(v) {
                        let g = g as usize;
                        let w = area[g]
                            * (ws * cf.distance_squared(centroid[g])).exp()
                            * (wr * nf.distance_squared(normal[g])).exp();
                        acc += w * normal[g];
                    }
                }
                acc.try_normalize().unwrap_or(nf)
            })
            .collect();
    }

    // 2. the vertices fitted to the filtered normals
    let max2 = params.max_displacement * params.max_displacement;
    for _ in 0..params.vertex_iterations {
        let pos = &mesh.positions;
        let (centroid, _, _) = geometry(pos);
        let moved: Vec<Vec3> = (0..pos.len())
            .into_par_iter()
            .map(|v| {
                let fs = ring(v as u32);
                if border[v] || fs.is_empty() {
                    return pos[v];
                }
                let mut d = Vec3::ZERO;
                for &f in fs {
                    let n = normal[f as usize];
                    d += n * n.dot(centroid[f as usize] - pos[v]);
                }
                let mut p = pos[v] + d / fs.len() as f32;
                // within the bound of the start
                let off = p - start[v];
                if off.length_squared() > max2 {
                    p = start[v] + off * (params.max_displacement / off.length());
                }
                p
            })
            .collect();
        mesh.positions = moved;
    }
    if mesh.normals.is_some() {
        mesh.update_normals();
    }
    Ok(mesh
        .positions
        .par_iter()
        .zip(&start)
        .map(|(p, q)| p.distance(*q))
        .reduce(|| 0.0, f32::max))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `n x n` grid over 10 x 10 mm with a crease (a roof, slope 0.5) and
    /// noise of `amp` mm.
    fn noisy_roof(n: usize, amp: f32) -> (TriMesh, TriMesh) {
        let mut seed = 3u64;
        let mut rnd = || {
            seed = seed.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1);
            (seed >> 33) as f32 / (1u64 << 31) as f32 * 2.0 - 1.0
        };
        let mut clean = Vec::new();
        let mut noisy = Vec::new();
        for j in 0..n {
            for i in 0..n {
                let x = 10.0 * i as f32 / (n - 1) as f32;
                let y = 10.0 * j as f32 / (n - 1) as f32;
                let z = 0.5 * (5.0 - (x - 5.0).abs());
                clean.push(Vec3::new(x, y, z));
                noisy.push(Vec3::new(x, y, z + amp * rnd()));
            }
        }
        let mut tris = Vec::new();
        for j in 0..n - 1 {
            for i in 0..n - 1 {
                let a = (j * n + i) as u32;
                let (b, c, d) = (a + 1, a + n as u32, a + n as u32 + 1);
                tris.extend([[a, b, d], [a, d, c]]);
            }
        }
        (TriMesh::new(clean, tris.clone()), TriMesh::new(noisy, tris))
    }

    fn rms_z(a: &TriMesh, b: &TriMesh) -> f32 {
        let s: f32 = a
            .positions
            .iter()
            .zip(&b.positions)
            .map(|(p, q)| (p.z - q.z).powi(2))
            .sum();
        (s / a.positions.len() as f32).sqrt()
    }

    #[test]
    fn flattens_the_grain_keeps_the_crease_and_the_bound() {
        // 0.1 mm grid with 0.01 mm noise, as the fine scan reconstruction
        let (clean, noisy) = noisy_roof(101, 0.01);
        let mut out = noisy.clone();
        let params = DenoiseParams::default();
        let moved = denoise(&mut out, &params).unwrap();
        assert!(moved <= params.max_displacement + 1e-6, "{moved}");
        let before = rms_z(&noisy, &clean);
        let after = rms_z(&out, &clean);
        assert!(after < 0.6 * before, "rms {before} -> {after}");
        // the crease stays sharp: its vertices stay on the ridge
        for (p, q) in out.positions.iter().zip(&clean.positions) {
            if (q.x - 5.0).abs() < 1e-3 && q.y > 0.5 && q.y < 9.5 {
                assert!((p.z - q.z).abs() < 0.012, "{p} {q}");
            }
        }
        // the border stays
        for (i, (p, q)) in out.positions.iter().zip(&noisy.positions).enumerate() {
            let (x, y) = (i % 101, i / 101);
            if x == 0 || y == 0 || x == 100 || y == 100 {
                assert_eq!(p, q);
            }
        }
        // deterministic
        let mut again = noisy.clone();
        denoise(&mut again, &params).unwrap();
        assert_eq!(again, out);
    }

    #[test]
    fn invalid_params() {
        let (_, mut m) = noisy_roof(5, 0.0);
        let bad = DenoiseParams {
            sigma_normal: 0.0,
            ..DenoiseParams::default()
        };
        assert!(denoise(&mut m, &bad).is_err());
    }
}
