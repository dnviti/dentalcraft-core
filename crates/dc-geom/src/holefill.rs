//! Hole filling (after Liepa 2003: triangulation, refinement, fairing).
//!
//! Every closed boundary loop with at most
//! [`HoleFillParams::max_boundary_edges`] edges is filled, except the
//! largest loop of the mesh (by perimeter): on an open scan that is the scan
//! border, which must stay open.
//!
//! 1. Triangulation: minimum-area triangulation of the loop by dynamic
//!    programming (O(n³), loops up to [`DP_MAX`] vertices); longer loops are
//!    ear-clipped, smallest angle first.
//! 2. Refinement (optional): triangles are split at their centroid while
//!    their size exceeds the local boundary edge length, and interior patch
//!    edges are flipped toward a Delaunay-like triangulation.
//! 3. Fairing (optional): the new vertices are relaxed to the membrane
//!    (uniform Laplacian) surface spanned by the fixed boundary.
//!
//! Patch triangles use the reverse of the boundary half-edges, so the
//! result keeps a consistent orientation.

use std::collections::{HashMap, HashSet};

use dc_mesh::TriMesh;
use glam::Vec3;

use crate::Result;

/// Loops longer than this are ear-clipped instead of triangulated by
/// minimum area.
pub const DP_MAX: usize = 160;

/// Hole filling parameters.
#[derive(Debug, Clone, PartialEq)]
pub struct HoleFillParams {
    /// Holes with more boundary edges than this are left open.
    pub max_boundary_edges: usize,
    /// Refine the patch to match the surrounding edge length.
    pub refine: bool,
    /// Fair the patch (smooth blend with the surroundings).
    pub fair: bool,
}

impl Default for HoleFillParams {
    fn default() -> Self {
        Self {
            max_boundary_edges: 500,
            refine: true,
            fair: true,
        }
    }
}

/// Fills the holes of `mesh` (see the module docs); returns the patched
/// mesh and the number of holes filled. New vertices get label `0`, the mean
/// colour of their hole's border and recomputed normals (when the mesh has
/// those attributes); new faces get label `0`.
///
/// # Errors
/// Propagates [`dc_mesh::MeshError`] for an inconsistent mesh.
pub fn fill_holes(mesh: &TriMesh, params: &HoleFillParams) -> Result<(TriMesh, usize)> {
    mesh.check()?;
    let adj = mesh.edge_adjacency();
    let pos = &mesh.positions;
    let mut loops: Vec<(Vec<u32>, f32)> = adj
        .boundary_loops()
        .into_iter()
        .filter(|l| {
            let closes = l.len() >= 3 && adj.faces_of(l[l.len() - 1], l[0]).len() == 1;
            let mut seen = HashSet::with_capacity(l.len());
            closes && l.iter().all(|v| seen.insert(*v))
        })
        .map(|l| {
            let per = (0..l.len())
                .map(|i| pos[l[i] as usize].distance(pos[l[(i + 1) % l.len()] as usize]))
                .sum();
            (l, per)
        })
        .collect();
    // the largest loop (open scan border) is never filled
    if let Some(big) = (0..loops.len()).max_by(|&a, &b| loops[a].1.total_cmp(&loops[b].1)) {
        loops.swap_remove(big);
    }

    let mut out = mesh.clone();
    let mut filled = 0;
    for (lp, _) in loops {
        if lp.len() > params.max_boundary_edges {
            continue;
        }
        // reversed loop: patch triangles listed counter-clockwise along it
        // use the boundary half-edges in the opposite direction
        let ring: Vec<u32> = lp.iter().rev().copied().collect();
        let ring_pos: Vec<Vec3> = ring.iter().map(|&v| out.positions[v as usize]).collect();
        let local = if ring.len() <= DP_MAX {
            min_area_triangulation(&ring_pos)
        } else {
            ear_clip(&ring_pos)
        };
        let mut tris: Vec<[u32; 3]> = local.iter().map(|t| t.map(|i| ring[i])).collect();
        let first_new = out.positions.len();
        if params.refine {
            refine(&mut out.positions, &mut tris, &ring, &adj);
        }
        if params.fair && out.positions.len() > first_new {
            fair(&mut out.positions, &tris, first_new);
        }
        let added = out.positions.len() - first_new;
        if let Some(labels) = &mut out.vertex_labels {
            labels.resize(out.positions.len(), 0);
        }
        if let Some(colors) = &mut out.vertex_colors {
            let mut acc = [0u32; 4];
            for &v in &ring {
                for (a, c) in acc.iter_mut().zip(colors[v as usize]) {
                    *a += u32::from(c);
                }
            }
            let n = ring.len() as u32;
            let mean = acc.map(|a| ((a + n / 2) / n) as u8);
            colors.extend(std::iter::repeat_n(mean, added));
        }
        if let Some(labels) = &mut out.face_labels {
            labels.extend(std::iter::repeat_n(0, tris.len()));
        }
        out.tris.extend(tris);
        filled += 1;
    }
    if filled > 0 && out.normals.is_some() {
        out.update_normals();
    }
    Ok((out, filled))
}

/// Area of triangle `a b c`.
fn area(a: Vec3, b: Vec3, c: Vec3) -> f32 {
    0.5 * (b - a).cross(c - a).length()
}

/// Minimum-area triangulation of polygon `p` (local indices, triangles
/// `(i, m, j)` with `i < m < j`). A small edge length term breaks ties toward
/// compact triangles.
fn min_area_triangulation(p: &[Vec3]) -> Vec<[usize; 3]> {
    let n = p.len();
    if n < 3 {
        return Vec::new();
    }
    let weight = |i: usize, m: usize, j: usize| {
        let (a, b, c) = (p[i], p[m], p[j]);
        area(a, b, c)
            + 1e-3 * (a.distance_squared(b) + b.distance_squared(c) + c.distance_squared(a))
    };
    let mut cost = vec![0f32; n * n];
    let mut best = vec![0u32; n * n];
    for len in 2..n {
        for i in 0..n - len {
            let j = i + len;
            let mut bc = f32::INFINITY;
            let mut bm = i + 1;
            for m in i + 1..j {
                let c = cost[i * n + m] + cost[m * n + j] + weight(i, m, j);
                if c < bc {
                    bc = c;
                    bm = m;
                }
            }
            cost[i * n + j] = bc;
            best[i * n + j] = bm as u32;
        }
    }
    let mut tris = Vec::with_capacity(n - 2);
    let mut stack = vec![(0usize, n - 1)];
    while let Some((i, j)) = stack.pop() {
        if j < i + 2 {
            continue;
        }
        let m = best[i * n + j] as usize;
        tris.push([i, m, j]);
        stack.push((i, m));
        stack.push((m, j));
    }
    tris
}

/// Ear clipping of polygon `p` (counter-clockwise around its Newell
/// normal), smallest interior angle first.
fn ear_clip(p: &[Vec3]) -> Vec<[usize; 3]> {
    let n = p.len();
    let mut normal = Vec3::ZERO;
    for i in 0..n {
        let (a, b) = (p[i], p[(i + 1) % n]);
        normal += Vec3::new(
            (a.y - b.y) * (a.z + b.z),
            (a.z - b.z) * (a.x + b.x),
            (a.x - b.x) * (a.y + b.y),
        );
    }
    let normal = normal.normalize_or_zero();
    let angle = |a: Vec3, v: Vec3, b: Vec3| {
        let (e1, e2) = (a - v, b - v);
        let s = normal.dot(e2.cross(e1));
        let a = s.atan2(e1.dot(e2));
        if a < 0.0 {
            a + std::f32::consts::TAU
        } else {
            a
        }
    };
    let mut ring: Vec<usize> = (0..n).collect();
    let mut tris = Vec::with_capacity(n.saturating_sub(2));
    while ring.len() > 3 {
        let m = ring.len();
        let t = (0..m)
            .min_by(|&x, &y| {
                let ax = angle(p[ring[(x + m - 1) % m]], p[ring[x]], p[ring[(x + 1) % m]]);
                let ay = angle(p[ring[(y + m - 1) % m]], p[ring[y]], p[ring[(y + 1) % m]]);
                ax.total_cmp(&ay)
            })
            .unwrap_or(0);
        tris.push([ring[(t + m - 1) % m], ring[t], ring[(t + 1) % m]]);
        ring.remove(t);
    }
    if ring.len() == 3 {
        tris.push([ring[0], ring[1], ring[2]]);
    }
    tris
}

/// Directed-edge map of a triangle list: `(a, b)` to the triangle holding it.
fn edge_map(tris: &[[u32; 3]]) -> HashMap<(u32, u32), usize> {
    let mut m = HashMap::with_capacity(tris.len() * 3);
    for (f, t) in tris.iter().enumerate() {
        for k in 0..3 {
            m.insert((t[k], t[(k + 1) % 3]), f);
        }
    }
    m
}

/// Liepa refinement: centroid splits driven by the boundary edge length,
/// with Delaunay-like edge flips after every round.
fn refine(
    positions: &mut Vec<Vec3>,
    tris: &mut Vec<[u32; 3]>,
    ring: &[u32],
    adj: &dc_mesh::EdgeAdjacency,
) {
    let n = ring.len();
    let mut sigma: HashMap<u32, f32> = HashMap::with_capacity(n * 4);
    for i in 0..n {
        let v = ring[i];
        let p = positions[v as usize];
        let a = p.distance(positions[ring[(i + n - 1) % n] as usize]);
        let b = p.distance(positions[ring[(i + 1) % n] as usize]);
        sigma.insert(v, 0.5 * (a + b));
    }
    let alpha = std::f32::consts::SQRT_2;
    // safety cap on the patch size
    let max_tris = 64 * n + 256;
    for _ in 0..32 {
        let mut split = false;
        let count = tris.len();
        for f in 0..count {
            if tris.len() + 2 > max_tris {
                break;
            }
            let t = tris[f];
            let vp = t.map(|v| positions[v as usize]);
            let c = (vp[0] + vp[1] + vp[2]) / 3.0;
            let sc = t.iter().map(|v| sigma[v]).sum::<f32>() / 3.0;
            let ok = (0..3).all(|k| {
                let d = alpha * c.distance(vp[k]);
                d > sc && d > sigma[&t[k]]
            });
            if !ok {
                continue;
            }
            let ci = positions.len() as u32;
            positions.push(c);
            sigma.insert(ci, sc);
            tris[f] = [t[0], t[1], ci];
            tris.push([t[1], t[2], ci]);
            tris.push([t[2], t[0], ci]);
            split = true;
        }
        relax(positions, tris, adj);
        if !split {
            break;
        }
    }
}

/// Flips interior patch edges whose opposite angles sum above π.
fn relax(positions: &[Vec3], tris: &mut [[u32; 3]], adj: &dc_mesh::EdgeAdjacency) {
    let angle = |a: Vec3, v: Vec3, b: Vec3| (a - v).angle_between(b - v);
    let mut edges = edge_map(tris);
    let limit = 16 * tris.len() + 16;
    let mut flips = 0;
    let mut changed = true;
    while changed && flips < limit {
        changed = false;
        for f in 0..tris.len() {
            for k in 0..3 {
                let t = tris[f];
                let (a, b, c) = (t[k], t[(k + 1) % 3], t[(k + 2) % 3]);
                let Some(&g) = edges.get(&(b, a)) else {
                    continue;
                };
                let u = tris[g];
                let d = u.iter().copied().find(|&x| x != a && x != b).unwrap_or(a);
                if d == a || c == d {
                    continue;
                }
                let p = |v: u32| positions[v as usize];
                let sum = angle(p(a), p(c), p(b)) + angle(p(a), p(d), p(b));
                if sum <= std::f32::consts::PI + 1e-4 {
                    continue;
                }
                // the new edge must not exist already, and the new triangles
                // must not fold over
                if edges.contains_key(&(c, d))
                    || edges.contains_key(&(d, c))
                    || !adj.faces_of(c, d).is_empty()
                {
                    continue;
                }
                let nrm = |x: u32, y: u32, z: u32| (p(y) - p(x)).cross(p(z) - p(x));
                let old = nrm(a, b, c) + nrm(b, a, d);
                if nrm(a, d, c).dot(old) <= 0.0 || nrm(d, b, c).dot(old) <= 0.0 {
                    continue;
                }
                for tt in [t, u] {
                    for kk in 0..3 {
                        edges.remove(&(tt[kk], tt[(kk + 1) % 3]));
                    }
                }
                tris[f] = [a, d, c];
                tris[g] = [d, b, c];
                for (fi, tt) in [(f, tris[f]), (g, tris[g])] {
                    for kk in 0..3 {
                        edges.insert((tt[kk], tt[(kk + 1) % 3]), fi);
                    }
                }
                flips += 1;
                changed = true;
                break;
            }
        }
    }
}

/// Relaxes the vertices from `first_new` on to the membrane surface spanned
/// by the fixed patch border (Gauss-Seidel on the uniform Laplacian).
fn fair(positions: &mut [Vec3], tris: &[[u32; 3]], first_new: usize) {
    let count = positions.len() - first_new;
    let mut nbrs: Vec<Vec<u32>> = vec![Vec::new(); count];
    for t in tris {
        for k in 0..3 {
            let v = t[k] as usize;
            if v >= first_new {
                for o in [t[(k + 1) % 3], t[(k + 2) % 3]] {
                    let list = &mut nbrs[v - first_new];
                    if !list.contains(&o) {
                        list.push(o);
                    }
                }
            }
        }
    }
    let scale = nbrs
        .iter()
        .enumerate()
        .flat_map(|(i, l)| l.iter().map(move |&o| (first_new + i, o as usize)))
        .map(|(v, o)| positions[v].distance(positions[o]))
        .fold(0f32, f32::max);
    let tol = (scale * 1e-4).max(f32::EPSILON);
    for _ in 0..500 {
        let mut moved = 0f32;
        for (i, list) in nbrs.iter().enumerate() {
            if list.is_empty() {
                continue;
            }
            let avg = list.iter().map(|&o| positions[o as usize]).sum::<Vec3>() / list.len() as f32;
            let v = first_new + i;
            moved = moved.max(avg.distance(positions[v]));
            positions[v] = avg;
        }
        if moved < tol {
            break;
        }
    }
}

#[cfg(test)]
mod tests;
