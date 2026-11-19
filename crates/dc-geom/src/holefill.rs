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

use std::cmp::Reverse;
use std::collections::{BinaryHeap, HashMap, HashSet};
use std::hash::{BuildHasherDefault, Hasher};
use std::ops::{Index, IndexMut};

use dc_mesh::TriMesh;
use glam::{DVec3, Vec3};
use rayon::prelude::*;

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

/// Multiplicative hash of small integer keys (vertex indices, edges): the
/// maps of a patch are hit millions of times, and the protection of the
/// default hasher against chosen keys is of no use on mesh indices.
#[derive(Default, Clone, Copy)]
struct IndexHasher(u64);

impl IndexHasher {
    fn add(&mut self, x: u64) {
        self.0 = (self.0.rotate_left(5) ^ x).wrapping_mul(0x517c_c1b7_2722_0a95);
    }
}

impl Hasher for IndexHasher {
    fn write(&mut self, bytes: &[u8]) {
        for &b in bytes {
            self.add(u64::from(b));
        }
    }

    fn write_u32(&mut self, i: u32) {
        self.add(u64::from(i));
    }

    fn write_u64(&mut self, i: u64) {
        self.add(i);
    }

    fn write_usize(&mut self, i: usize) {
        self.add(i as u64);
    }

    fn finish(&self) -> u64 {
        self.0
    }
}

/// Map keyed by mesh indices.
type IndexMap<K, V> = HashMap<K, V, BuildHasherDefault<IndexHasher>>;

/// Key of the undirected edge `a b`.
fn edge_key(a: u32, b: u32) -> u64 {
    let (lo, hi) = if a < b { (a, b) } else { (b, a) };
    (u64::from(lo) << 32) | u64::from(hi)
}

/// Edges of a triangle list (sorted keys, with the number of faces using
/// each) and its border half-edges, following the face winding (sorted).
/// Degenerate edges (`a == b`) are ignored. The same answers as
/// [`dc_mesh::EdgeAdjacency`], from flat sorted arrays built in parallel.
pub(crate) struct Edges {
    keys: Vec<u64>,
    counts: Vec<u32>,
    border: Vec<(u32, u32)>,
    vertices: usize,
}

impl Edges {
    /// Edges of `tris`, whose vertices are below `vertices`.
    pub(crate) fn new(tris: &[[u32; 3]], vertices: usize) -> Self {
        let half = |t: &[u32; 3]| {
            let t = *t;
            (0..3)
                .map(move |k| (t[k], t[(k + 1) % 3]))
                .filter(|(a, b)| a != b)
        };
        let mut all: Vec<u64> = tris
            .par_iter()
            .flat_map_iter(|t| half(t).map(|(a, b)| edge_key(a, b)))
            .collect();
        all.par_sort_unstable();
        let mut keys = Vec::with_capacity(all.len() / 2 + 1);
        let mut counts: Vec<u32> = Vec::with_capacity(all.len() / 2 + 1);
        for k in all {
            match (keys.last(), counts.last_mut()) {
                (Some(&last), Some(n)) if last == k => *n += 1,
                _ => {
                    keys.push(k);
                    counts.push(1);
                }
            }
        }
        let mut edges = Self {
            keys,
            counts,
            border: Vec::new(),
            vertices,
        };
        let mut border: Vec<(u32, u32)> = tris
            .par_iter()
            .flat_map_iter(|t| half(t).filter(|&(a, b)| edges.count(a, b) == 1))
            .collect();
        border.par_sort_unstable();
        edges.border = border;
        edges
    }

    /// Faces using the edge `a b` (0 when it does not exist).
    pub(crate) fn count(&self, a: u32, b: u32) -> u32 {
        self.keys
            .binary_search(&edge_key(a, b))
            .map_or(0, |i| self.counts[i])
    }

    /// Edges used by more than two faces.
    pub(crate) fn non_manifold(&self) -> usize {
        self.counts.iter().filter(|&&n| n > 2).count()
    }

    /// Border loops as vertex lists following the face winding, open chains
    /// (around non-manifold vertices) included, in the order of
    /// [`dc_mesh::EdgeAdjacency::boundary_loops`].
    pub(crate) fn boundary_loops(&self) -> Vec<Vec<u32>> {
        // half-edges out of every vertex: a run of the sorted border, taken
        // from its end
        let mut first = vec![u32::MAX; self.vertices];
        let mut left = vec![0u32; self.vertices];
        for (i, &(a, _)) in self.border.iter().enumerate() {
            if first[a as usize] == u32::MAX {
                first[a as usize] = i as u32;
            }
            left[a as usize] += 1;
        }
        let mut pop = |v: u32| {
            let n = &mut left[v as usize];
            (*n > 0).then(|| {
                *n -= 1;
                self.border[(first[v as usize] + *n) as usize].1
            })
        };
        let mut loops = Vec::new();
        for &(start, _) in &self.border {
            let Some(mut n) = pop(start) else { continue };
            let mut lp = vec![start];
            while n != start {
                lp.push(n);
                match pop(n) {
                    Some(next) => n = next,
                    None => break,
                }
            }
            loops.push(lp);
        }
        loops
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
    let adj = Edges::new(&mesh.tris, mesh.positions.len());
    let pos = &mesh.positions;
    let mut loops: Vec<(Vec<u32>, f32)> = adj
        .boundary_loops()
        .into_iter()
        .filter(|l| {
            let closes = l.len() >= 3 && adj.count(l[l.len() - 1], l[0]) == 1;
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
        IndexMap::default()
    };

    // every hole is patched on its own (in parallel, the longest first),
    // its new vertices numbered from the mesh's vertex count, then appended
    // in loop order
    let base = mesh.positions.len();
    let mut order: Vec<usize> = (0..loops.len()).collect();
    order.sort_by_key(|&i| std::cmp::Reverse(loops[i].0.len()));
    let mut patches: Vec<Option<Patch>> = (0..loops.len()).map(|_| None).collect();
    let done: Vec<(usize, Option<Patch>)> = order
        .into_par_iter()
        .with_max_len(1)
        .map(|i| (i, patch(mesh, &adj, &loops[i].0, &ring_nbrs, params)))
        .collect();
    for (i, p) in done {
        patches[i] = p;
    }
    let mut out = mesh.clone();
    let mut filled = 0;
    for patch in patches.into_iter().flatten() {
        let first_new = out.positions.len();
        let added = patch.new.len();
        out.positions.extend(patch.new);
        if let Some(labels) = &mut out.vertex_labels {
            labels.resize(out.positions.len(), 0);
        }
        if let Some(colors) = &mut out.vertex_colors {
            let mut acc = [0u32; 4];
            for &v in &patch.ring {
                for (a, c) in acc.iter_mut().zip(colors[v as usize]) {
                    *a += u32::from(c);
                }
            }
            let n = patch.ring.len() as u32;
            let mean = acc.map(|a| ((a + n / 2) / n) as u8);
            colors.extend(std::iter::repeat_n(mean, added));
        }
        if let Some(labels) = &mut out.face_labels {
            labels.extend(std::iter::repeat_n(0, patch.tris.len()));
        }
        let shift = |v: u32| {
            if v as usize >= base {
                v - base as u32 + first_new as u32
            } else {
                v
            }
        };
        out.tris
            .extend(patch.tris.into_iter().map(|t| t.map(shift)));
        filled += 1;
    }
    if filled > 0 && out.normals.is_some() {
        out.update_normals();
    }
    Ok((out, filled))
}

/// The patch of one hole: its border (the loop reversed), its new
/// vertices and its triangles (new vertices numbered from the vertex count
/// of the mesh).
struct Patch {
    ring: Vec<u32>,
    new: Vec<Vec3>,
    tris: Vec<[u32; 3]>,
}

/// Positions seen while patching one hole: the mesh's (read only), then
/// the new vertices of the patch.
struct PatchPositions<'a> {
    base: &'a [Vec3],
    new: Vec<Vec3>,
}

impl PatchPositions<'_> {
    fn len(&self) -> usize {
        self.base.len() + self.new.len()
    }

    fn push(&mut self, p: Vec3) {
        self.new.push(p);
    }
}

impl Index<usize> for PatchPositions<'_> {
    type Output = Vec3;

    fn index(&self, i: usize) -> &Vec3 {
        match i.checked_sub(self.base.len()) {
            Some(k) => &self.new[k],
            None => &self.base[i],
        }
    }
}

impl IndexMut<usize> for PatchPositions<'_> {
    /// New vertices only: the mesh is read only.
    fn index_mut(&mut self, i: usize) -> &mut Vec3 {
        &mut self.new[i - self.base.len()]
    }
}

/// Triangulates, refines and fairs the hole bounded by `lp`; `None` when
/// it cannot be triangulated.
fn patch(
    mesh: &TriMesh,
    adj: &Edges,
    lp: &[u32],
    ring_nbrs: &IndexMap<u32, Vec<u32>>,
    params: &HoleFillParams,
) -> Option<Patch> {
    // reversed loop: patch triangles listed counter-clockwise along it
    // use the boundary half-edges in the opposite direction
    let ring: Vec<u32> = lp.iter().rev().copied().collect();
    let ring_pos: Vec<Vec3> = ring.iter().map(|&v| mesh.positions[v as usize]).collect();
    // a diagonal that is already a mesh edge (a narrow neck between two
    // border vertices) would give that edge a third face
    let forbidden = |a: usize, b: usize| adj.count(ring[a], ring[b]) > 0;
    let local = if ring.len() <= DP_MAX {
        min_area_triangulation(&ring_pos, &forbidden)
    } else {
        ear_clip(&ring_pos, &forbidden)
    }?;
    let mut tris: Vec<[u32; 3]> = local.iter().map(|t| t.map(|i| ring[i])).collect();
    let mut positions = PatchPositions {
        base: &mesh.positions,
        new: Vec::new(),
    };
    let first_new = positions.len();
    if params.refine {
        refine(&mut positions, &mut tris, &ring, adj);
    }
    if params.fair && positions.len() > first_new {
        fair(&mut positions, &tris, first_new);
        if params.curvature {
            fair_curvature(&mut positions, &tris, first_new, &ring, ring_nbrs);
        }
    }
    Some(Patch {
        ring,
        new: positions.new,
        tris,
    })
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
/// normal), smallest interior angle first (ties to the first vertex of the
/// polygon), never cutting a `forbidden` diagonal; `None` when it gets
/// stuck. The ears wait in a heap; clipping one changes only the angles of
/// its two neighbours.
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
    // the order of `f32::total_cmp` as an integer
    let order = |x: f32| {
        let b = x.to_bits() as i32;
        b ^ (((b >> 31) as u32) >> 1) as i32
    };
    let mut prev: Vec<usize> = (0..n).map(|i| (i + n - 1) % n).collect();
    let mut next: Vec<usize> = (0..n).map(|i| (i + 1) % n).collect();
    // version of every vertex's entry; `u32::MAX` once clipped
    let mut version = vec![0u32; n];
    let ear = |i: usize, prev: &[usize], next: &[usize]| {
        (!forbidden(prev[i], next[i])).then(|| order(angle(p[prev[i]], p[i], p[next[i]])))
    };
    let mut heap: BinaryHeap<Reverse<(i32, usize, u32)>> = (0..n)
        .filter_map(|i| ear(i, &prev, &next).map(|k| Reverse((k, i, 0))))
        .collect();
    let mut tris = Vec::with_capacity(n.saturating_sub(2));
    let mut left = n;
    while left > 3 {
        let Reverse((_, t, v)) = heap.pop()?;
        if v != version[t] {
            continue;
        }
        let (a, b) = (prev[t], next[t]);
        tris.push([a, t, b]);
        next[a] = b;
        prev[b] = a;
        version[t] = u32::MAX;
        left -= 1;
        for w in [a, b] {
            version[w] += 1;
            if let Some(k) = ear(w, &prev, &next) {
                heap.push(Reverse((k, w, version[w])));
            }
        }
    }
    if left == 3 {
        let s = (0..n).find(|&i| version[i] != u32::MAX)?;
        tris.push([s, next[s], next[next[s]]]);
    }
    Some(tris)
}

/// Directed-edge map of a triangle list: `(a, b)` to the triangle holding it.
fn edge_map(tris: &[[u32; 3]]) -> IndexMap<(u32, u32), usize> {
    let mut m = IndexMap::with_capacity_and_hasher(tris.len() * 3, Default::default());
    for (f, t) in tris.iter().enumerate() {
        for k in 0..3 {
            m.insert((t[k], t[(k + 1) % 3]), f);
        }
    }
    m
}

/// Liepa refinement: centroid splits driven by the boundary edge length,
/// with Delaunay-like edge flips after every round.
fn refine(positions: &mut PatchPositions, tris: &mut Vec<[u32; 3]>, ring: &[u32], adj: &Edges) {
    let n = ring.len();
    let mut sigma: IndexMap<u32, f32> =
        IndexMap::with_capacity_and_hasher(n * 4, Default::default());
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
fn relax(positions: &PatchPositions, tris: &mut [[u32; 3]], adj: &Edges) {
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
                if edges.contains_key(&(c, d)) || edges.contains_key(&(d, c)) || adj.count(c, d) > 0
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
fn fair(positions: &mut PatchPositions, tris: &[[u32; 3]], first_new: usize) {
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
fn border_neighbours(mesh: &TriMesh, border: impl Iterator<Item = u32>) -> IndexMap<u32, Vec<u32>> {
    // slot of every border vertex in `lists`
    let mut slot = vec![u32::MAX; mesh.positions.len()];
    let mut verts = Vec::new();
    for v in border {
        if slot[v as usize] == u32::MAX {
            slot[v as usize] = verts.len() as u32;
            verts.push(v);
        }
    }
    let mut lists: Vec<Vec<u32>> = vec![Vec::new(); verts.len()];
    if !verts.is_empty() {
        for t in &mesh.tris {
            for k in 0..3 {
                let s = slot[t[k] as usize];
                if s != u32::MAX {
                    let list = &mut lists[s as usize];
                    for o in [t[(k + 1) % 3], t[(k + 2) % 3]] {
                        if !list.contains(&o) {
                            list.push(o);
                        }
                    }
                }
            }
        }
    }
    verts.into_iter().zip(lists).collect()
}

/// Thin-plate fairing of the patch vertices from `first_new` on: they
/// minimise the sum of the squared uniform Laplacians of the patch
/// vertices and of the hole border `ring` (whose neighbours in the mesh are
/// `ring_nbrs`), with every other vertex fixed: the patch meets the
/// surrounding surface with its slope and bend. Solved by conjugate
/// gradient from the membrane positions; a solve that does not converge
/// keeps them.
fn fair_curvature(
    positions: &mut PatchPositions,
    patch: &[[u32; 3]],
    first_new: usize,
    ring: &[u32],
    ring_nbrs: &IndexMap<u32, Vec<u32>>,
) {
    let count = positions.len() - first_new;
    if count == 0 {
        return;
    }
    // neighbours of the patch and border vertices (patch triangles, plus
    // the mesh around the border)
    let mut nbrs: IndexMap<u32, Vec<u32>> =
        IndexMap::with_capacity_and_hasher(count + ring.len(), Default::default());
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
    let mut rhs = vec![[0.0f64; 3]; count];
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
            for (axis, r) in rhs[a].iter_mut().enumerate() {
                *r -= wa * fixed[axis];
            }
        }
    }
    let Ok(a) = CsrMatrix::from_triplets(count, &triplets) else {
        return;
    };
    let mut x: Vec<[f64; 3]> = positions
        .new
        .iter()
        .map(|p| p.to_array().map(f64::from))
        .collect();
    conjugate_gradient3(&a, &rhs, &mut x, FAIR_TOLERANCE, FAIR_MAX_ITERATIONS);
    for (p, q) in positions.new.iter_mut().zip(&x) {
        let q = Vec3::from_array(q.map(|v| v as f32));
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

/// Patches with fewer unknowns are solved sequentially: the holes already
/// run in parallel, and a fine-grained parallel product per iteration
/// would only wait on them.
const FAIR_PARALLEL_ROWS: usize = 32_768;

/// `out = A x` for the three columns of `x` at once (one pass over the
/// matrix); in parallel from [`FAIR_PARALLEL_ROWS`] rows.
fn mul_vec3(a: &CsrMatrix, x: &[[f64; 3]], out: &mut [[f64; 3]]) {
    let row = |i: usize| {
        let mut s = [0.0; 3];
        for e in a.row_ptr[i]..a.row_ptr[i + 1] {
            let (v, xc) = (a.vals[e], &x[a.cols[e]]);
            for k in 0..3 {
                s[k] += xc[k] * v;
            }
        }
        s
    };
    if a.n >= FAIR_PARALLEL_ROWS {
        out.par_chunks_mut(crate::sparse::CHUNK)
            .enumerate()
            .for_each(|(c, rows)| {
                for (k, o) in rows.iter_mut().enumerate() {
                    *o = row(c * crate::sparse::CHUNK + k);
                }
            });
    } else {
        for (i, o) in out.iter_mut().enumerate() {
            *o = row(i);
        }
    }
}

/// Per column, the dot product of the columns of `u` and `v`; from
/// [`FAIR_PARALLEL_ROWS`] rows summed in parallel by fixed chunks, the
/// partial sums added in chunk order (the same result on any number of
/// threads).
fn dot3(u: &[[f64; 3]], v: &[[f64; 3]]) -> [f64; 3] {
    let sum = |u: &[[f64; 3]], v: &[[f64; 3]]| {
        let mut s = [0.0; 3];
        for (p, q) in u.iter().zip(v) {
            for k in 0..3 {
                s[k] += p[k] * q[k];
            }
        }
        s
    };
    if u.len() < FAIR_PARALLEL_ROWS {
        return sum(u, v);
    }
    let parts: Vec<[f64; 3]> = u
        .par_chunks(crate::sparse::CHUNK)
        .zip(v.par_chunks(crate::sparse::CHUNK))
        .map(|(u, v)| sum(u, v))
        .collect();
    let mut s = [0.0; 3];
    for p in parts {
        for k in 0..3 {
            s[k] += p[k];
        }
    }
    s
}

/// `f(i, &mut a[i], &mut b[i])` for every row, in parallel by chunks from
/// [`FAIR_PARALLEL_ROWS`] rows.
fn rows2(
    a: &mut [[f64; 3]],
    b: &mut [[f64; 3]],
    f: impl Fn(usize, &mut [f64; 3], &mut [f64; 3]) + Sync,
) {
    if a.len() < FAIR_PARALLEL_ROWS {
        for (i, (x, y)) in a.iter_mut().zip(b.iter_mut()).enumerate() {
            f(i, x, y);
        }
        return;
    }
    let chunk = crate::sparse::CHUNK;
    a.par_chunks_mut(chunk)
        .zip(b.par_chunks_mut(chunk))
        .enumerate()
        .for_each(|(c, (a, b))| {
            for (k, (x, y)) in a.iter_mut().zip(b.iter_mut()).enumerate() {
                f(c * chunk + k, x, y);
            }
        });
}

/// Jacobi-preconditioned conjugate gradient on `a x = b` for the three
/// columns of `b` and `x` at once (they share every product with the
/// matrix), from `x`; every column stops on its own, when its residual
/// falls below `tol` times its starting one, or after `max_iterations`
/// (plain sums: a patch needs no bit reproducibility).
fn conjugate_gradient3(
    a: &CsrMatrix,
    b: &[[f64; 3]],
    x: &mut [[f64; 3]],
    tol: f64,
    max_iterations: usize,
) {
    let n = a.n;
    let inv: Vec<f64> = a
        .diagonal()
        .into_iter()
        .map(|d| if d > 0.0 { 1.0 / d } else { 1.0 })
        .collect();
    let mut r = vec![[0.0; 3]; n];
    mul_vec3(a, x, &mut r);
    for (ri, bi) in r.iter_mut().zip(b) {
        for k in 0..3 {
            ri[k] = bi[k] - ri[k];
        }
    }
    let start = dot3(&r, &r).map(f64::sqrt);
    let mut active = start.map(|s| s != 0.0 && s.is_finite());
    let mut z: Vec<[f64; 3]> = r
        .iter()
        .zip(&inv)
        .map(|(ri, di)| ri.map(|v| v * di))
        .collect();
    let mut p = z.clone();
    let mut rz = dot3(&r, &z);
    let mut ap = vec![[0.0; 3]; n];
    for _ in 0..max_iterations {
        if !active.contains(&true) {
            return;
        }
        mul_vec3(a, &p, &mut ap);
        let pap = dot3(&p, &ap);
        let mut alpha = [0.0; 3];
        for k in 0..3 {
            if active[k] {
                if pap[k] <= 0.0 || !pap[k].is_finite() {
                    active[k] = false;
                } else {
                    alpha[k] = rz[k] / pap[k];
                }
            }
        }
        rows2(x, &mut r, |i, xi, ri| {
            for k in (0..3).filter(|&k| active[k]) {
                xi[k] += alpha[k] * p[i][k];
                ri[k] -= alpha[k] * ap[i][k];
            }
        });
        let rr = dot3(&r, &r);
        for k in 0..3 {
            if rr[k].sqrt() <= tol * start[k] {
                active[k] = false;
            }
        }
        rows2(&mut z, &mut p, |i, zi, _| *zi = r[i].map(|v| v * inv[i]));
        let next = dot3(&r, &z);
        let mut beta = [0.0; 3];
        for k in (0..3).filter(|&k| active[k]) {
            beta[k] = next[k] / rz[k];
            rz[k] = next[k];
        }
        rows2(&mut p, &mut z, |_, pi, zi| {
            for k in (0..3).filter(|&k| active[k]) {
                pi[k] = zi[k] + beta[k] * pi[k];
            }
        });
    }
}

#[cfg(test)]
mod tests;
