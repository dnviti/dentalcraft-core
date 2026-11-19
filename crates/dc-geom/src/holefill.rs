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
//!    (uniform Laplacian) surface spanned by the fixed boundary; then, with
//!    [`HoleFillParams::curvature`], to the thin-plate surface (minimum of
//!    the squared uniform Laplacian over the patch and its border, Liepa's
//!    "curvature" fairing) with the border and the ring of mesh vertices
//!    around it fixed: the patch continues the slope and the bend of the
//!    surrounding surface (C1) instead of spanning it like a flat cap.
//!
//! Patch triangles use the reverse of the boundary half-edges, so the
//! result keeps a consistent orientation.

use std::collections::{HashMap, HashSet};

use dc_mesh::TriMesh;
use glam::{DVec3, Vec3};

use crate::Result;
use crate::sparse::CsrMatrix;

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
    /// With `fair`: fair to the curvature of the surroundings (thin
    /// plate), not only to the membrane spanned by the border.
    pub curvature: bool,
}

impl Default for HoleFillParams {
    fn default() -> Self {
        Self {
            max_boundary_edges: 500,
            refine: true,
            fair: true,
            curvature: true,
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
    loops.retain(|(l, _)| l.len() <= params.max_boundary_edges);
    // mesh neighbours of the border vertices, for the curvature fairing
    let ring_nbrs = if params.fair && params.curvature {
        border_neighbours(mesh, loops.iter().flat_map(|(l, _)| l.iter().copied()))
    } else {
        HashMap::new()
    };

    let mut out = mesh.clone();
    let mut filled = 0;
    for (lp, _) in loops {
        // reversed loop: patch triangles listed counter-clockwise along it
        // use the boundary half-edges in the opposite direction
        let ring: Vec<u32> = lp.iter().rev().copied().collect();
        let ring_pos: Vec<Vec3> = ring.iter().map(|&v| out.positions[v as usize]).collect();
        // a diagonal that is already a mesh edge (a narrow neck between two
        // border vertices) would give that edge a third face
        let forbidden = |a: usize, b: usize| !adj.faces_of(ring[a], ring[b]).is_empty();
        let local = if ring.len() <= DP_MAX {
            min_area_triangulation(&ring_pos, &forbidden)
        } else {
            ear_clip(&ring_pos, &forbidden)
        };
        let Some(local) = local else {
            continue;
        };
        let mut tris: Vec<[u32; 3]> = local.iter().map(|t| t.map(|i| ring[i])).collect();
        let first_new = out.positions.len();
        if params.refine {
            refine(&mut out.positions, &mut tris, &ring, &adj);
        }
        if params.fair && out.positions.len() > first_new {
            fair(&mut out.positions, &tris, first_new);
            if params.curvature {
                fair_curvature(&mut out.positions, &tris, first_new, &ring, &ring_nbrs);
            }
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
/// `(i, m, j)` with `i < m < j`) whose diagonals avoid the `forbidden`
/// ones; `None` when there is no such triangulation. A small edge length
/// term breaks ties toward compact triangles.
fn min_area_triangulation(
    p: &[Vec3],
    forbidden: &dyn Fn(usize, usize) -> bool,
) -> Option<Vec<[usize; 3]>> {
    let n = p.len();
    if n < 3 {
        return None;
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
            // the chord i j is a diagonal unless it closes the loop
            if !(i == 0 && j == n - 1) && forbidden(i, j) {
                cost[i * n + j] = f32::INFINITY;
                continue;
            }
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
    if !cost[n - 1].is_finite() {
        return None;
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
    Some(tris)
}

/// Ear clipping of polygon `p` (counter-clockwise around its Newell
/// normal), smallest interior angle first, never cutting a `forbidden`
/// diagonal; `None` when it gets stuck.
fn ear_clip(p: &[Vec3], forbidden: &dyn Fn(usize, usize) -> bool) -> Option<Vec<[usize; 3]>> {
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
            .filter(|&x| !forbidden(ring[(x + m - 1) % m], ring[(x + 1) % m]))
            .min_by(|&x, &y| {
                let ax = angle(p[ring[(x + m - 1) % m]], p[ring[x]], p[ring[(x + 1) % m]]);
                let ay = angle(p[ring[(y + m - 1) % m]], p[ring[y]], p[ring[(y + 1) % m]]);
                ax.total_cmp(&ay)
            })?;
        tris.push([ring[(t + m - 1) % m], ring[t], ring[(t + 1) % m]]);
        ring.remove(t);
    }
    if ring.len() == 3 {
        tris.push([ring[0], ring[1], ring[2]]);
    }
    Some(tris)
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

/// Mesh neighbours (sharing a triangle) of every vertex in `border`.
fn border_neighbours(mesh: &TriMesh, border: impl Iterator<Item = u32>) -> HashMap<u32, Vec<u32>> {
    let mut out: HashMap<u32, Vec<u32>> = border.map(|v| (v, Vec::new())).collect();
    if out.is_empty() {
        return out;
    }
    for t in &mesh.tris {
        for k in 0..3 {
            if let Some(list) = out.get_mut(&t[k]) {
                for o in [t[(k + 1) % 3], t[(k + 2) % 3]] {
                    if !list.contains(&o) {
                        list.push(o);
                    }
                }
            }
        }
    }
    out
}

/// Thin-plate fairing of the patch vertices from `first_new` on: they
/// minimise the sum of the squared uniform Laplacians of the patch
/// vertices and of the hole border `ring` (whose neighbours in the mesh are
/// `ring_nbrs`), with every other vertex fixed: the patch meets the
/// surrounding surface with its slope and bend. Solved by conjugate
/// gradient from the membrane positions; a solve that does not converge
/// keeps them.
fn fair_curvature(
    positions: &mut [Vec3],
    patch: &[[u32; 3]],
    first_new: usize,
    ring: &[u32],
    ring_nbrs: &HashMap<u32, Vec<u32>>,
) {
    let count = positions.len() - first_new;
    if count == 0 {
        return;
    }
    // neighbours of the patch and border vertices (patch triangles, plus
    // the mesh around the border)
    let mut nbrs: HashMap<u32, Vec<u32>> = HashMap::with_capacity(count + ring.len());
    let mut add = |v: u32, o: u32| {
        let list = nbrs.entry(v).or_default();
        if !list.contains(&o) {
            list.push(o);
        }
    };
    for t in patch {
        for k in 0..3 {
            add(t[k], t[(k + 1) % 3]);
            add(t[k], t[(k + 2) % 3]);
        }
    }
    for v in ring {
        for &o in ring_nbrs.get(v).map_or(&[][..], Vec::as_slice) {
            add(*v, o);
        }
    }
    let unknown = |v: u32| (v as usize >= first_new).then(|| v as usize - first_new);
    // normal equations of the energy sum_v |L_v x|^2 over the free vertices
    let mut triplets = Vec::new();
    let mut rhs = vec![vec![0.0f64; count]; 3];
    let mut rows: Vec<u32> = (first_new as u32..positions.len() as u32).collect();
    rows.extend_from_slice(ring);
    for v in rows {
        let Some(list) = nbrs.get(&v).filter(|l| !l.is_empty()) else {
            continue;
        };
        let c = 1.0 / list.len() as f64;
        let entries = list
            .iter()
            .map(|&o| (o, c))
            .chain(std::iter::once((v, -1.0)));
        let mut fixed = DVec3::ZERO;
        let mut free = Vec::with_capacity(list.len() + 1);
        for (o, w) in entries {
            match unknown(o) {
                Some(k) => free.push((k, w)),
                None => fixed += w * positions[o as usize].as_dvec3(),
            }
        }
        for &(a, wa) in &free {
            for &(b, wb) in &free {
                triplets.push((a, b, wa * wb));
            }
            for (axis, r) in rhs.iter_mut().enumerate() {
                r[a] -= wa * fixed[axis];
            }
        }
    }
    let Ok(a) = CsrMatrix::from_triplets(count, &triplets) else {
        return;
    };
    let mut solved = Vec::with_capacity(3);
    for (axis, b) in rhs.iter().enumerate() {
        let mut x: Vec<f64> = positions[first_new..]
            .iter()
            .map(|p| f64::from(p[axis]))
            .collect();
        conjugate_gradient(&a, b, &mut x, FAIR_TOLERANCE, FAIR_MAX_ITERATIONS);
        solved.push(x);
    }
    for (k, p) in positions[first_new..].iter_mut().enumerate() {
        let q = Vec3::new(
            solved[0][k] as f32,
            solved[1][k] as f32,
            solved[2][k] as f32,
        );
        if q.is_finite() {
            *p = q;
        }
    }
}

/// Relative residual reduction at which the thin-plate fairing stops.
const FAIR_TOLERANCE: f64 = 1e-6;
/// Iterations of the thin-plate fairing at most: every iterate lowers the
/// bending energy from the membrane start, so a cut-off solve is still
/// fairer than the membrane.
const FAIR_MAX_ITERATIONS: usize = 3000;

/// Jacobi-preconditioned conjugate gradient on `a x = b` from `x`, until
/// the residual falls below `tol` times the starting one or after
/// `max_iterations` (plain sums: a patch needs no bit reproducibility).
fn conjugate_gradient(a: &CsrMatrix, b: &[f64], x: &mut [f64], tol: f64, max_iterations: usize) {
    let dot = |u: &[f64], v: &[f64]| u.iter().zip(v).map(|(p, q)| p * q).sum::<f64>();
    let n = a.n;
    let inv: Vec<f64> = a
        .diagonal()
        .into_iter()
        .map(|d| if d > 0.0 { 1.0 / d } else { 1.0 })
        .collect();
    let mut r = a.mul_vec(x);
    for (ri, bi) in r.iter_mut().zip(b) {
        *ri = bi - *ri;
    }
    let start = dot(&r, &r).sqrt();
    if start == 0.0 || !start.is_finite() {
        return;
    }
    let mut z: Vec<f64> = r.iter().zip(&inv).map(|(ri, di)| ri * di).collect();
    let mut p = z.clone();
    let mut rz = dot(&r, &z);
    let mut ap = vec![0.0; n];
    for _ in 0..max_iterations {
        a.mul_vec_into(&p, &mut ap);
        let pap = dot(&p, &ap);
        if pap <= 0.0 || !pap.is_finite() {
            return;
        }
        let alpha = rz / pap;
        for ((xi, ri), (pi, api)) in x.iter_mut().zip(r.iter_mut()).zip(p.iter().zip(&ap)) {
            *xi += alpha * pi;
            *ri -= alpha * api;
        }
        if dot(&r, &r).sqrt() <= tol * start {
            return;
        }
        for ((zi, ri), di) in z.iter_mut().zip(&r).zip(&inv) {
            *zi = ri * di;
        }
        let next = dot(&r, &z);
        let beta = next / rz;
        rz = next;
        for (pi, zi) in p.iter_mut().zip(&z) {
            *pi = zi + beta * *pi;
        }
    }
}

#[cfg(test)]
mod tests;
