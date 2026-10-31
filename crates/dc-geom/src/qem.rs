//! Quadric error metric decimation (WP15, port plan B2.4).
//!
//! Greedy edge collapse after Garland & Heckbert (1997). Every vertex holds
//! the sum of the plane quadrics of its original triangles (unweighted, as
//! the add-on's `gpu_simplify`), plus, with boundary preservation, the
//! quadrics of the planes through the boundary edges perpendicular to their
//! triangle (weight [`DecimateParams::boundary_weight`], Blender's Decimate
//! uses 100). Collapses are taken cheapest first; the merged vertex goes to
//! the minimiser of the summed quadric, computed with a truncated
//! pseudo-inverse about the edge midpoint (Lindstrom 2000), so flat and
//! cylindrical regions stay put; when that position is rejected the edge
//! endpoints and midpoint are tried.
//!
//! A collapse is accepted only when the result stays a 2-manifold of the
//! same topology:
//!
//! * link condition (Dey et al. 1999): the common neighbours of the two
//!   vertices are exactly the opposite vertices of the shared triangles, an
//!   interior edge between two boundary vertices is never collapsed, and an
//!   opposite vertex never drops below valence 3 (2 on a boundary);
//! * no fold-over: no surviving triangle turns its normal by more than about
//!   87° or becomes degenerate.
//! * with [`DecimateParams::max_valence`], no vertex ends with more
//!   neighbours than that.
//!
//! Stop criteria (the first one met ends the run): the face target
//! ([`DecimateParams::target_faces`] or [`DecimateParams::ratio`], Blender's
//! Decimate "collapse" by ratio), the quadric error
//! ([`DecimateParams::max_error`]), or no collapse left within the geometric
//! tolerance ([`DecimateParams::tolerance`], like `gpu_simplify.reduce`).
//! With a tolerance every original vertex stays assigned to a triangle of
//! the reduced mesh within the tolerance and every moved vertex is checked
//! against the original surface, so the vertex Hausdorff distance between
//! input and output (both directions) is at most the tolerance.
//!
//! Labels (`vertex_labels`, or `face_labels` when there are no vertex
//! labels) are preserved when [`DecimateParams::preserve_labels`] is set: a
//! vertex next to another label never moves and never disappears, so no
//! collapse crosses or moves a label boundary. Surviving vertices and
//! triangles keep their attributes; [`Decimation::vertex_map`] maps every
//! input vertex to the output vertex it merged into.
//!
//! The run is sequential and deterministic: ties in cost go to the smaller
//! vertex indices, the survivor of a free collapse is the smaller index.

use std::cmp::Ordering;

use glam::DVec3;
use nalgebra::{Matrix3, Vector3};
use rayon::prelude::*;

use dc_mesh::TriMesh;

use crate::bvh::Bvh;
use crate::{GeomError, Result};

/// Output index of an input vertex that is not referenced by any triangle.
pub const UNMAPPED: u32 = u32::MAX;

/// Eigenvalues of the quadric below this fraction of the largest are
/// treated as zero when placing a vertex.
const EIGEN_CUTOFF: f64 = 1e-3;
/// Smallest cosine between a triangle normal before and after a collapse.
const FLIP_COS: f64 = 0.05;
/// A triangle whose doubled area is below this fraction of its squared
/// longest edge is degenerate.
const DEGENERATE: f64 = 1e-6;
/// Bits of the stamp sum kept in a queue entry.
const STAMP_MASK: u32 = 0x3fff_ffff;
/// Rounds of re-seeding the queue with every live edge.
const MAX_ROUNDS: usize = 8;
/// Relative slack of the bounding-sphere test of the tolerance check (a
/// face is skipped only when it is certainly farther than the bound).
const PRUNE_SLACK: f64 = 1.0 + 1e-9;

/// Decimation parameters.
#[derive(Debug, Clone, PartialEq)]
pub struct DecimateParams {
    /// Target number of triangles (0: no face target).
    pub target_faces: usize,
    /// Target as a fraction of the input triangles (overrides
    /// `target_faces`), as the `ratio` of Blender's Decimate modifier.
    pub ratio: Option<f64>,
    /// Do not collapse edges whose quadric error exceeds this (mm²).
    pub max_error: f64,
    /// Keep every vertex of the input within this distance of the output
    /// surface and every output vertex within it of the input (mm).
    pub tolerance: Option<f64>,
    /// Keep boundary edges in place: boundary vertices only collapse along
    /// the boundary, held by boundary quadrics.
    pub preserve_boundary: bool,
    /// Weight of the boundary quadrics.
    pub boundary_weight: f64,
    /// Never move or remove a vertex next to another label.
    pub preserve_labels: bool,
    /// No collapse may leave a vertex with more neighbours than this (0: no
    /// limit). Large flat regions otherwise end in fans of dozens of thin
    /// triangles, whose tolerance check dominates the run.
    pub max_valence: usize,
}

impl Default for DecimateParams {
    fn default() -> Self {
        Self {
            target_faces: 100_000,
            ratio: None,
            max_error: f64::INFINITY,
            tolerance: None,
            preserve_boundary: true,
            boundary_weight: 100.0,
            preserve_labels: true,
            max_valence: 0,
        }
    }
}

impl DecimateParams {
    /// Collapse down to `faces` triangles.
    pub fn to_faces(faces: usize) -> Self {
        Self {
            target_faces: faces,
            ..Self::default()
        }
    }

    /// Collapse down to `ratio` times the input triangles (Blender Decimate).
    pub fn to_ratio(ratio: f64) -> Self {
        Self {
            ratio: Some(ratio),
            ..Self::default()
        }
    }

    /// Fewest triangles within `tolerance` mm (like `gpu_simplify.reduce`).
    pub fn to_tolerance(tolerance: f64) -> Self {
        Self {
            target_faces: 0,
            tolerance: Some(tolerance),
            ..Self::default()
        }
    }
}

/// Result of [`decimate`].
#[derive(Debug, Clone, PartialEq)]
pub struct Decimation {
    /// The simplified mesh (attributes of the surviving vertices and faces;
    /// normals recomputed when the input had normals).
    pub mesh: TriMesh,
    /// Output vertex of every input vertex ([`UNMAPPED`] when the input
    /// vertex is not referenced by any triangle).
    pub vertex_map: Vec<u32>,
    /// Number of collapses performed.
    pub collapses: usize,
    /// Largest quadric error of a performed collapse (mm²).
    pub max_cost: f64,
}

/// Simplifies `mesh` (see the module doc).
///
/// # Errors
/// [`GeomError::InvalidInput`] for a mesh without triangles, with
/// non-finite positions or invalid parameters; [`GeomError::Mesh`] for
/// indices out of range or attributes of the wrong length.
pub fn decimate(mesh: &TriMesh, params: &DecimateParams) -> Result<Decimation> {
    mesh.check()?;
    if mesh.tris.is_empty() {
        return Err(GeomError::InvalidInput("mesh has no triangles".into()));
    }
    if mesh.positions.iter().any(|p| !p.is_finite()) {
        return Err(GeomError::InvalidInput("vertex not finite".into()));
    }
    if let Some(r) = params.ratio
        && !(r > 0.0 && r <= 1.0)
    {
        return Err(GeomError::InvalidInput(format!("ratio {r} not in (0, 1]")));
    }
    if let Some(t) = params.tolerance
        && !(t >= 0.0 && t.is_finite())
    {
        return Err(GeomError::InvalidInput(format!("tolerance {t} invalid")));
    }
    if params.max_error.is_nan() || params.boundary_weight.is_nan() || params.boundary_weight < 0.0
    {
        return Err(GeomError::InvalidInput(
            "max_error and boundary_weight must be numbers (weight >= 0)".into(),
        ));
    }
    let target = match params.ratio {
        Some(r) => (r * mesh.tris.len() as f64).ceil() as usize,
        None => params.target_faces,
    };
    let mut d = Decimator::new(mesh, params)?;
    d.run(target, params.max_error);
    Ok(d.finish(mesh))
}

/// Symmetric 4x4 quadric `[A b; b^T c]`: `a00 a01 a02 b0 a11 a12 b1 a22 b2 c`.
#[derive(Debug, Clone, Copy, Default)]
struct Quadric([f64; 10]);

impl Quadric {
    /// `w * (n.x + d)^2` for the plane `n.x + d = 0`, `n` unit.
    fn plane(n: DVec3, d: f64, w: f64) -> Self {
        Self([
            w * n.x * n.x,
            w * n.x * n.y,
            w * n.x * n.z,
            w * n.x * d,
            w * n.y * n.y,
            w * n.y * n.z,
            w * n.y * d,
            w * n.z * n.z,
            w * n.z * d,
            w * d * d,
        ])
    }

    fn add(&mut self, o: &Self) {
        for (a, b) in self.0.iter_mut().zip(o.0) {
            *a += b;
        }
    }

    fn sum(a: &Self, b: &Self) -> Self {
        let mut s = *a;
        s.add(b);
        s
    }

    fn eval(&self, p: DVec3) -> f64 {
        let q = &self.0;
        let (x, y, z) = (p.x, p.y, p.z);
        q[0] * x * x
            + q[4] * y * y
            + q[7] * z * z
            + 2.0 * (q[1] * x * y + q[2] * x * z + q[5] * y * z)
            + 2.0 * (q[3] * x + q[6] * y + q[8] * z)
            + q[9]
    }

    /// Minimiser closest to `m`: `m - A+ (A m + b)` with the small
    /// eigenvalues of `A` truncated.
    fn optimum(&self, m: DVec3) -> DVec3 {
        let q = &self.0;
        let a = Matrix3::new(q[0], q[1], q[2], q[1], q[4], q[5], q[2], q[5], q[7]);
        let mv = Vector3::new(m.x, m.y, m.z);
        let g = a * mv + Vector3::new(q[3], q[6], q[8]);
        let eig = a.symmetric_eigen();
        let lmax = eig.eigenvalues.iter().fold(0.0_f64, |a, &l| a.max(l));
        if lmax <= 0.0 {
            return m;
        }
        let mut x = mv;
        for i in 0..3 {
            let l = eig.eigenvalues[i];
            if l > EIGEN_CUTOFF * lmax {
                let v = eig.eigenvectors.column(i);
                x -= v * (v.dot(&g) / l);
            }
        }
        DVec3::new(x[0], x[1], x[2])
    }
}

/// Queue entry packed in a `u128` so that the integer order is the queue
/// order: cost (`f32` bits, non-negative), then the vertex indices `a < b`,
/// then the stamp sum of `a` and `b` (30 bits) and the placement (2 bits:
/// 0 optimal, 1 vertex `a`, 2 vertex `b`, 3 midpoint). Stamps only grow, so
/// an unchanged sum means both vertices are unchanged.
#[derive(Debug, Clone, Copy)]
struct Entry {
    cost: f32,
    a: u32,
    b: u32,
    stamps: u32,
    alt: u8,
}

impl Entry {
    fn pack(self) -> u128 {
        (u128::from(self.cost.to_bits()) << 96)
            | (u128::from(self.a) << 64)
            | (u128::from(self.b) << 32)
            | u128::from((self.stamps & STAMP_MASK) << 2 | u32::from(self.alt & 3))
    }

    fn unpack(k: u128) -> Self {
        let low = k as u32;
        Self {
            cost: f32::from_bits((k >> 96) as u32),
            a: (k >> 64) as u32,
            b: (k >> 32) as u32,
            stamps: low >> 2,
            alt: (low & 3) as u8,
        }
    }
}

/// Queue key of a cost: `f32`, finite and non-negative.
fn key_cost(c: f64) -> f32 {
    let c = c as f32;
    if c.is_finite() { c.max(0.0) } else { f32::MAX }
}

/// 4-ary min-heap of packed entries (lazy deletion: stale entries are
/// skipped when popped).
#[derive(Debug, Default)]
struct Queue(Vec<u128>);

impl Queue {
    /// A sorted array is a valid heap.
    fn from_sorted(v: Vec<u128>) -> Self {
        Self(v)
    }

    fn push(&mut self, k: u128) {
        let v = &mut self.0;
        v.push(k);
        let mut i = v.len() - 1;
        while i > 0 {
            let p = (i - 1) / 4;
            if v[p] <= k {
                break;
            }
            v[i] = v[p];
            i = p;
        }
        v[i] = k;
    }

    fn pop(&mut self) -> Option<u128> {
        let v = &mut self.0;
        let last = v.pop()?;
        if v.is_empty() {
            return Some(last);
        }
        let top = v[0];
        let n = v.len();
        let mut i = 0;
        loop {
            let c = 4 * i + 1;
            if c >= n {
                break;
            }
            let mut m = c;
            for j in c + 1..(c + 4).min(n) {
                if v[j] < v[m] {
                    m = j;
                }
            }
            if v[m] >= last {
                break;
            }
            v[i] = v[m];
            i = m;
        }
        v[i] = last;
        Some(top)
    }
}

/// A planned collapse: `gone` merges into `keep`, which moves to `x`.
#[derive(Debug, Clone, Copy)]
struct Plan {
    keep: u32,
    gone: u32,
    x: DVec3,
    cost: f64,
    free: bool,
}

/// A face around a planned collapse, as it would become.
struct NewFace {
    id: u32,
    c: [DVec3; 3],
    centre: DVec3,
    radius: f64,
}

/// State of the tolerance check.
struct Tolerance {
    tol2: f64,
    tol: f32,
    /// Original positions (centred, f64).
    orig: Vec<DVec3>,
    /// Original vertices assigned to every face.
    face_pts: Vec<Vec<u32>>,
    /// BVH of the original mesh (input coordinates).
    bvh: Bvh,
    /// Assignment computed by the last successful check.
    scratch: Vec<(u32, u32)>,
}

struct Decimator<'a> {
    center: DVec3,
    pos: Vec<DVec3>,
    quad: Vec<Quadric>,
    tris: Vec<[u32; 3]>,
    face_alive: Vec<bool>,
    vfaces: Vec<Vec<u32>>,
    alive: Vec<bool>,
    stamp: Vec<u32>,
    boundary: Vec<bool>,
    pinned: Vec<bool>,
    parent: Vec<u32>,
    labels: Option<&'a [u16]>,
    preserve_boundary: bool,
    live_faces: usize,
    collapses: usize,
    max_cost: f64,
    tol: Option<Tolerance>,
    heap: Queue,
    nk: Vec<u32>,
    ng: Vec<u32>,
    nw: Vec<u32>,
    max_valence: usize,
}

/// Sorted distinct neighbours of `v` (excluding `v`) into `out`.
fn neighbours(vfaces: &[Vec<u32>], tris: &[[u32; 3]], v: u32, out: &mut Vec<u32>) {
    out.clear();
    for &f in &vfaces[v as usize] {
        for w in tris[f as usize] {
            if w != v {
                out.push(w);
            }
        }
    }
    out.sort_unstable();
    out.dedup();
}

fn normal(a: DVec3, b: DVec3, c: DVec3) -> DVec3 {
    (b - a).cross(c - a)
}

/// `true` when `abc` is degenerate (relative to its longest edge).
fn degenerate(n: DVec3, a: DVec3, b: DVec3, c: DVec3) -> bool {
    let l2 = (b - a)
        .length_squared()
        .max((c - b).length_squared())
        .max((a - c).length_squared());
    n.length() <= DEGENERATE * l2
}

impl<'a> Decimator<'a> {
    fn new(mesh: &'a TriMesh, params: &DecimateParams) -> Result<Self> {
        let nv = mesh.positions.len();
        let nf = mesh.tris.len();
        let center = mesh.aabb().map_or(DVec3::ZERO, |b| b.center().as_dvec3());
        let pos: Vec<DVec3> = mesh
            .positions
            .iter()
            .map(|p| p.as_dvec3() - center)
            .collect();
        let tris = mesh.tris.clone();
        let face_alive: Vec<bool> = tris
            .iter()
            .map(|&[a, b, c]| a != b && b != c && a != c)
            .collect();
        let mut vfaces: Vec<Vec<u32>> = vec![Vec::new(); nv];
        for (f, t) in tris.iter().enumerate() {
            if face_alive[f] {
                for &v in t {
                    vfaces[v as usize].push(f as u32);
                }
            }
        }
        let live_faces = face_alive.iter().filter(|a| **a).count();

        // edges: (key, face) sorted
        let mut edges: Vec<(u64, u32)> = Vec::with_capacity(3 * nf);
        for (f, t) in tris.iter().enumerate() {
            if !face_alive[f] {
                continue;
            }
            for k in 0..3 {
                let (a, b) = (t[k], t[(k + 1) % 3]);
                edges.push((edge_key(a, b), f as u32));
            }
        }
        edges.par_sort_unstable();
        let mut boundary = vec![false; nv];
        let mut pinned = vec![false; nv];
        let mut bcount = vec![0u8; nv];
        let mut quad = vec![Quadric::default(); nv];
        // boundary quadrics, boundary and non-manifold flags
        let mut i = 0;
        let mut bquads: Vec<(u32, Quadric)> = Vec::new();
        while i < edges.len() {
            let mut j = i + 1;
            while j < edges.len() && edges[j].0 == edges[i].0 {
                j += 1;
            }
            let (a, b) = ((edges[i].0 >> 32) as u32, edges[i].0 as u32);
            match j - i {
                1 => {
                    for v in [a, b] {
                        boundary[v as usize] = true;
                        bcount[v as usize] = bcount[v as usize].saturating_add(1);
                    }
                    if params.preserve_boundary && params.boundary_weight > 0.0 {
                        let [p0, p1, p2] = tris[edges[i].1 as usize].map(|v| pos[v as usize]);
                        let n = normal(p0, p1, p2).normalize_or_zero();
                        let (pa, pb) = (pos[a as usize], pos[b as usize]);
                        let e = (pb - pa).normalize_or_zero();
                        let m = e.cross(n).normalize_or_zero();
                        if m != DVec3::ZERO {
                            let q = Quadric::plane(m, -m.dot(pa), params.boundary_weight);
                            bquads.push((a, q));
                            bquads.push((b, q));
                        }
                    }
                }
                2 => {}
                _ => {
                    pinned[a as usize] = true;
                    pinned[b as usize] = true;
                }
            }
            i = j;
        }
        // face quadrics, accumulated per vertex in face order (deterministic)
        let fq: Vec<Quadric> = tris
            .par_iter()
            .map(|t| {
                let [a, b, c] = t.map(|v| pos[v as usize]);
                let n = normal(a, b, c).normalize_or_zero();
                if n == DVec3::ZERO {
                    Quadric::default()
                } else {
                    Quadric::plane(n, -n.dot(a), 1.0)
                }
            })
            .collect();
        quad.par_iter_mut().enumerate().for_each(|(v, q)| {
            for &f in &vfaces[v] {
                q.add(&fq[f as usize]);
            }
        });
        for (v, q) in &bquads {
            quad[*v as usize].add(q);
        }

        // vertex fans that are not a single disc / half-disc, and labels
        let labels_v = params
            .preserve_labels
            .then_some(mesh.vertex_labels.as_deref())
            .flatten();
        let labels_f = if params.preserve_labels && labels_v.is_none() {
            mesh.face_labels.as_deref()
        } else {
            None
        };
        let mut nb = Vec::new();
        for v in 0..nv {
            if vfaces[v].is_empty() {
                continue;
            }
            neighbours(&vfaces, &tris, v as u32, &mut nb);
            let nfaces = vfaces[v].len();
            let disc = if boundary[v] {
                bcount[v] == 2 && nb.len() == nfaces + 1
            } else {
                nb.len() == nfaces
            };
            if !disc {
                pinned[v] = true;
            }
            if let Some(l) = labels_v
                && nb.iter().any(|&w| l[w as usize] != l[v])
            {
                pinned[v] = true;
            }
            if let Some(l) = labels_f {
                let l0 = l[vfaces[v][0] as usize];
                if vfaces[v].iter().any(|&f| l[f as usize] != l0) {
                    pinned[v] = true;
                }
            }
        }

        let tol = match params.tolerance {
            Some(t) => {
                let mut face_pts = vec![Vec::new(); nf];
                for (v, fs) in vfaces.iter().enumerate() {
                    if let Some(&f) = fs.first() {
                        face_pts[f as usize].push(v as u32);
                    }
                }
                Some(Tolerance {
                    tol2: t * t,
                    tol: t as f32,
                    orig: pos.clone(),
                    face_pts,
                    bvh: Bvh::build(mesh)?,
                    scratch: Vec::new(),
                })
            }
            None => None,
        };

        Ok(Self {
            center,
            pos,
            quad,
            tris,
            face_alive,
            vfaces,
            alive: vec![true; nv],
            stamp: vec![0; nv],
            boundary,
            pinned,
            parent: (0..nv as u32).collect(),
            labels: labels_v,
            preserve_boundary: params.preserve_boundary,
            live_faces,
            collapses: 0,
            max_cost: 0.0,
            tol,
            heap: Queue::default(),
            nk: Vec::new(),
            ng: Vec::new(),
            nw: Vec::new(),
            max_valence: params.max_valence,
        })
    }

    /// Stamp sum of an edge (see [`Entry`]).
    fn stamps(&self, a: u32, b: u32) -> u32 {
        self.stamp[a as usize].wrapping_add(self.stamp[b as usize]) & STAMP_MASK
    }

    /// `u` must survive the collapse of its edge with `other`, in place.
    fn must_keep(&self, u: u32, other: u32) -> bool {
        self.pinned[u as usize]
            || (self.preserve_boundary
                && self.boundary[u as usize]
                && !self.boundary[other as usize])
    }

    /// Placement and cost of collapsing the edge `a < b`.
    fn plan(&self, a: u32, b: u32, alt: u8) -> Option<Plan> {
        let (ka, kb) = (self.must_keep(a, b), self.must_keep(b, a));
        if ka && kb {
            return None;
        }
        if let Some(l) = self.labels
            && l[a as usize] != l[b as usize]
        {
            return None;
        }
        let q = Quadric::sum(&self.quad[a as usize], &self.quad[b as usize]);
        let (pa, pb) = (self.pos[a as usize], self.pos[b as usize]);
        let (keep, gone, x, free) = if ka {
            (a, b, pa, false)
        } else if kb {
            (b, a, pb, false)
        } else {
            let mid = (pa + pb) * 0.5;
            let x = match alt {
                0 => q.optimum(mid),
                1 => pa,
                2 => pb,
                _ => mid,
            };
            (a, b, x, true)
        };
        Some(Plan {
            keep,
            gone,
            x,
            cost: q.eval(x).max(0.0),
            free,
        })
    }

    fn push_edge(&mut self, a: u32, b: u32) {
        let (a, b) = if a < b { (a, b) } else { (b, a) };
        if let Some(p) = self.plan(a, b, 0) {
            let e = Entry {
                cost: key_cost(p.cost),
                a,
                b,
                stamps: self.stamps(a, b),
                alt: 0,
            };
            self.heap.push(e.pack());
        }
    }

    /// Queues every live edge.
    fn seed(&mut self) {
        self.heap.0.clear();
        let mut nb = Vec::new();
        let mut entries = Vec::new();
        for u in 0..self.pos.len() as u32 {
            if !self.alive[u as usize] || self.vfaces[u as usize].is_empty() {
                continue;
            }
            neighbours(&self.vfaces, &self.tris, u, &mut nb);
            for &w in nb.iter().filter(|&&w| w > u) {
                entries.push((u, w));
            }
        }
        let mut planned: Vec<u128> = entries
            .par_iter()
            .filter_map(|&(a, b)| {
                self.plan(a, b, 0).map(|p| {
                    Entry {
                        cost: key_cost(p.cost),
                        a,
                        b,
                        stamps: self.stamps(a, b),
                        alt: 0,
                    }
                    .pack()
                })
            })
            .collect();
        planned.par_sort_unstable();
        self.heap = Queue::from_sorted(planned);
    }

    fn run(&mut self, target: usize, max_error: f64) {
        for _ in 0..MAX_ROUNDS {
            if self.live_faces <= target {
                return;
            }
            self.seed();
            let before = self.collapses;
            while self.live_faces > target {
                let Some(k) = self.heap.pop() else { break };
                let e = Entry::unpack(k);
                let (a, b) = (e.a as usize, e.b as usize);
                if !self.alive[a] || !self.alive[b] || self.stamps(e.a, e.b) != e.stamps {
                    continue;
                }
                let Some(p) = self.plan(e.a, e.b, e.alt) else {
                    continue;
                };
                if p.cost > max_error {
                    return;
                }
                if self.validate(&p) {
                    self.collapse(&p);
                    continue;
                }
                if e.alt != 0 || !p.free {
                    continue;
                }
                let mut alts: Vec<(Plan, u8)> = (1..=3)
                    .filter_map(|k| self.plan(e.a, e.b, k).map(|p| (p, k)))
                    .collect();
                alts.sort_by(|x, y| x.0.cost.total_cmp(&y.0.cost).then(x.1.cmp(&y.1)));
                for (alt, k) in alts {
                    if alt.cost > max_error {
                        break;
                    }
                    if self.validate(&alt) {
                        if alt.cost <= p.cost {
                            self.collapse(&alt);
                        } else {
                            let r = Entry {
                                cost: key_cost(alt.cost),
                                alt: k,
                                ..e
                            };
                            self.heap.push(r.pack());
                        }
                        break;
                    }
                }
            }
            if self.collapses == before {
                return;
            }
        }
    }

    /// Topological, geometric and tolerance checks of `p`. On success with a
    /// tolerance, the new assignment of the original vertices is left in
    /// the scratch buffer for [`Decimator::collapse`].
    fn validate(&mut self, p: &Plan) -> bool {
        let (k, g) = (p.keep, p.gone);
        let (ku, gu) = (k as usize, g as usize);
        // shared faces and their opposite vertices
        let mut opp = [0u32; 2];
        let mut shared = 0;
        for &f in &self.vfaces[gu] {
            let t = self.tris[f as usize];
            if t.contains(&k) {
                if shared == 2 {
                    return false;
                }
                opp[shared] = t.into_iter().find(|&w| w != k && w != g).unwrap_or(k);
                shared += 1;
            }
        }
        if shared == 0 || (shared == 2 && self.boundary[ku] && self.boundary[gu]) {
            return false;
        }
        let opp = &mut opp[..shared];
        opp.sort_unstable();
        // link condition
        let mut nk = std::mem::take(&mut self.nk);
        let mut ng = std::mem::take(&mut self.ng);
        let mut nw = std::mem::take(&mut self.nw);
        neighbours(&self.vfaces, &self.tris, k, &mut nk);
        neighbours(&self.vfaces, &self.tris, g, &mut ng);
        let mut ok = {
            let mut common = 0usize;
            let mut matches = true;
            let (mut i, mut j) = (0, 0);
            while i < nk.len() && j < ng.len() {
                match nk[i].cmp(&ng[j]) {
                    Ordering::Less => i += 1,
                    Ordering::Greater => j += 1,
                    Ordering::Equal => {
                        if common >= opp.len() || opp[common] != nk[i] {
                            matches = false;
                            break;
                        }
                        common += 1;
                        i += 1;
                        j += 1;
                    }
                }
            }
            matches && common == opp.len()
        };
        // the merged vertex: both rings without the two ends, the opposite
        // vertices once
        if ok && self.max_valence > 0 && nk.len() + ng.len() - opp.len() - 2 > self.max_valence {
            ok = false;
        }
        if ok {
            for &w in opp.iter() {
                neighbours(&self.vfaces, &self.tris, w, &mut nw);
                let min = if self.boundary[w as usize] { 3 } else { 4 };
                if nw.len() < min {
                    ok = false;
                    break;
                }
            }
        }
        self.nk = nk;
        self.ng = ng;
        self.nw = nw;
        if !ok {
            return false;
        }
        // fold-over
        for &v in &[k, g] {
            for &f in &self.vfaces[v as usize] {
                let t = self.tris[f as usize];
                if t.contains(&k) && t.contains(&g) {
                    continue;
                }
                let old = t.map(|w| self.pos[w as usize]);
                let new = t.map(|w| {
                    if w == k || w == g {
                        p.x
                    } else {
                        self.pos[w as usize]
                    }
                });
                let n0 = normal(old[0], old[1], old[2]);
                let n1 = normal(new[0], new[1], new[2]);
                if degenerate(n1, new[0], new[1], new[2]) {
                    return false;
                }
                let l0 = n0.length();
                if l0 > 0.0 && n0.dot(n1) <= FLIP_COS * l0 * n1.length() {
                    return false;
                }
            }
        }
        // tolerance
        if self.tol.is_some() {
            return self.check_tolerance(p);
        }
        true
    }

    fn check_tolerance(&mut self, p: &Plan) -> bool {
        let (k, g) = (p.keep, p.gone);
        let Some(tol) = self.tol.as_mut() else {
            return true;
        };
        // a vertex left in place was checked when it got there
        if p.x != self.pos[k as usize]
            && tol
                .bvh
                .closest_point((p.x + self.center).as_vec3(), tol.tol)
                .is_none()
        {
            return false;
        }
        // new faces around the merged vertex, each with a bounding sphere
        let mut new_faces: Vec<NewFace> = Vec::new();
        for &v in &[k, g] {
            for &f in &self.vfaces[v as usize] {
                let t = self.tris[f as usize];
                if t.contains(&k) && t.contains(&g) {
                    continue;
                }
                let c = t.map(|w| {
                    if w == k || w == g {
                        p.x
                    } else {
                        self.pos[w as usize]
                    }
                });
                let centre = (c[0] + c[1] + c[2]) / 3.0;
                let radius = c.iter().map(|q| q.distance(centre)).fold(0.0, f64::max);
                new_faces.push(NewFace {
                    id: f,
                    c,
                    centre,
                    radius,
                });
            }
        }
        tol.scratch.clear();
        for &v in &[k, g] {
            for &f in &self.vfaces[v as usize] {
                if v == g && self.tris[f as usize].contains(&k) {
                    continue; // shared faces are listed once (from k)
                }
                // the face that held the points, when it survives, bounds their distance
                let own = new_faces.iter().find(|nf| nf.id == f);
                for &pt in &tol.face_pts[f as usize] {
                    let o = tol.orig[pt as usize];
                    let mut bound = tol.tol2;
                    if let Some(nf) = own {
                        bound = bound.min(point_triangle_dist2(o, nf.c[0], nf.c[1], nf.c[2]));
                    }
                    // the closest new face (the first one at the smallest distance); a
                    // face whose sphere lies beyond the bound cannot be it
                    let mut best = (f64::INFINITY, u32::MAX);
                    for nf in &new_faces {
                        let lb = (o.distance(nf.centre) - nf.radius).max(0.0);
                        if lb * lb > bound.min(best.0) * PRUNE_SLACK {
                            continue;
                        }
                        let d = point_triangle_dist2(o, nf.c[0], nf.c[1], nf.c[2]);
                        if d < best.0 {
                            best = (d, nf.id);
                        }
                    }
                    if best.0 > tol.tol2 {
                        return false;
                    }
                    tol.scratch.push((pt, best.1));
                }
            }
        }
        true
    }

    fn collapse(&mut self, p: &Plan) {
        let (k, g) = (p.keep, p.gone);
        let (ku, gu) = (k as usize, g as usize);
        if let Some(tol) = self.tol.as_mut() {
            for &v in &[ku, gu] {
                for &f in &self.vfaces[v] {
                    tol.face_pts[f as usize].clear();
                }
            }
            for &(pt, f) in &tol.scratch {
                tol.face_pts[f as usize].push(pt);
            }
            tol.scratch.clear();
        }
        let gf = std::mem::take(&mut self.vfaces[gu]);
        for f in gf {
            let t = &mut self.tris[f as usize];
            if t.contains(&k) {
                self.face_alive[f as usize] = false;
                self.live_faces -= 1;
                for w in *t {
                    if w != g {
                        self.vfaces[w as usize].retain(|&x| x != f);
                    }
                }
            } else {
                for w in t.iter_mut() {
                    if *w == g {
                        *w = k;
                    }
                }
                self.vfaces[ku].push(f);
            }
        }
        self.pos[ku] = p.x;
        let qg = self.quad[gu];
        self.quad[ku].add(&qg);
        self.alive[gu] = false;
        self.parent[gu] = k;
        self.boundary[ku] |= self.boundary[gu];
        self.stamp[ku] = self.stamp[ku].wrapping_add(1);
        self.collapses += 1;
        self.max_cost = self.max_cost.max(p.cost);
        let mut nb = std::mem::take(&mut self.nk);
        neighbours(&self.vfaces, &self.tris, k, &mut nb);
        for &w in &nb {
            self.push_edge(k, w);
        }
        self.nk = nb;
    }

    fn finish(mut self, mesh: &TriMesh) -> Decimation {
        let nv = self.pos.len();
        let mut new_index = vec![UNMAPPED; nv];
        let mut positions = Vec::new();
        let mut old_of_new = Vec::new();
        for (v, ni) in new_index.iter_mut().enumerate() {
            if self.alive[v] && !self.vfaces[v].is_empty() {
                *ni = positions.len() as u32;
                positions.push((self.pos[v] + self.center).as_vec3());
                old_of_new.push(v);
            }
        }
        // follow the collapse chains (with path compression)
        let mut vertex_map = vec![UNMAPPED; nv];
        for (v, out) in vertex_map.iter_mut().enumerate() {
            let mut r = v as u32;
            while self.parent[r as usize] != r {
                r = self.parent[r as usize];
            }
            let mut c = v as u32;
            while self.parent[c as usize] != r && c != r {
                let next = self.parent[c as usize];
                self.parent[c as usize] = r;
                c = next;
            }
            *out = new_index[r as usize];
        }
        let faces: Vec<usize> = (0..self.tris.len())
            .filter(|&f| self.face_alive[f])
            .collect();
        let tris = faces
            .iter()
            .map(|&f| self.tris[f].map(|v| new_index[v as usize]))
            .collect();
        let mut out = TriMesh::new(positions, tris);
        out.vertex_labels = mesh
            .vertex_labels
            .as_ref()
            .map(|l| old_of_new.iter().map(|&v| l[v]).collect());
        out.vertex_colors = mesh
            .vertex_colors
            .as_ref()
            .map(|c| old_of_new.iter().map(|&v| c[v]).collect());
        out.face_labels = mesh
            .face_labels
            .as_ref()
            .map(|l| faces.iter().map(|&f| l[f]).collect());
        if mesh.normals.is_some() {
            out.update_normals();
        }
        Decimation {
            mesh: out,
            vertex_map,
            collapses: self.collapses,
            max_cost: self.max_cost,
        }
    }
}

fn edge_key(a: u32, b: u32) -> u64 {
    let (a, b) = if a < b { (a, b) } else { (b, a) };
    (u64::from(a) << 32) | u64::from(b)
}

/// Squared distance from `p` to triangle `abc` (Ericson, "Real-Time
/// Collision Detection" 5.1.5); degenerate triangles act as segments.
fn point_triangle_dist2(p: DVec3, a: DVec3, b: DVec3, c: DVec3) -> f64 {
    let ab = b - a;
    let ac = c - a;
    let ap = p - a;
    let d1 = ab.dot(ap);
    let d2 = ac.dot(ap);
    if d1 <= 0.0 && d2 <= 0.0 {
        return ap.length_squared();
    }
    let bp = p - b;
    let d3 = ab.dot(bp);
    let d4 = ac.dot(bp);
    if d3 >= 0.0 && d4 <= d3 {
        return bp.length_squared();
    }
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let v = d1 / (d1 - d3);
        return (p - (a + ab * v)).length_squared();
    }
    let cp = p - c;
    let d5 = ab.dot(cp);
    let d6 = ac.dot(cp);
    if d6 >= 0.0 && d5 <= d6 {
        return cp.length_squared();
    }
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let w = d2 / (d2 - d6);
        return (p - (a + ac * w)).length_squared();
    }
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return (p - (b + (c - b) * w)).length_squared();
    }
    let denom = va + vb + vc;
    if denom.abs() <= f64::MIN_POSITIVE {
        return segment_dist2(p, a, b)
            .min(segment_dist2(p, b, c))
            .min(segment_dist2(p, c, a));
    }
    let v = vb / denom;
    let w = vc / denom;
    (p - (a + ab * v + ac * w)).length_squared()
}

fn segment_dist2(p: DVec3, a: DVec3, b: DVec3) -> f64 {
    let ab = b - a;
    let l2 = ab.length_squared();
    let t = if l2 > 0.0 {
        ((p - a).dot(ab) / l2).clamp(0.0, 1.0)
    } else {
        0.0
    };
    (p - (a + ab * t)).length_squared()
}

#[cfg(test)]
mod tests;
