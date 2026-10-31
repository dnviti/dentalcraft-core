//! Bounding volume hierarchy over the triangles of a mesh: closest point,
//! ray and overlap queries (WP13, port plan B1.2).
//!
//! The tree is built with a binned surface-area heuristic (16 bins on the
//! best of the three axes), in parallel for large meshes, and flattened into
//! one `Vec` of 32-byte nodes (the left child directly follows its parent,
//! the right child is reached through a relative offset). Node boxes and
//! triangle corners are stored as `f32`, in tree order for cache locality;
//! every query measures in `f64`, so a hit is exact for the stored (f32)
//! geometry and does not depend on the magnitude of the coordinates.
//!
//! Conventions, matching the CPU oracle of the Blender add-on
//! (`gpu_query.py`):
//!
//! * rays are two-sided (scans are open surfaces and are seen from the back);
//!   `dir` need not be unit length, `t` is the ray parameter
//!   (`origin + t * dir`);
//! * ties (equal distance or equal `t`) go to the smallest face index;
//! * a query without a result returns `None` / an empty `Vec`;
//! * batched queries return one entry per input, in input order.
//!
//! Triangles with non-finite corners are skipped when building; triangles
//! with out-of-range indices are an error. Zero-area triangles are kept for
//! the closest-point and overlap queries (they act as segments or points) and
//! never hit a ray.

use glam::{DVec3, Vec3};
use rayon::prelude::*;

use dc_mesh::{Aabb, TriMesh};

use crate::{GeomError, Result};

/// Number of bins used by the surface-area heuristic.
const BINS: usize = 16;
/// Triangles per leaf above which a node is always split.
const MAX_LEAF: usize = 8;
/// Triangles per leaf below which a node is always a leaf.
const MIN_LEAF: usize = 2;
/// Relative cost of visiting a node against intersecting one triangle.
const TRAVERSAL_COST: f32 = 1.0;
/// Depth after which the build falls back to balanced median splits.
const MAX_SAH_DEPTH: u32 = 40;
/// Subtrees smaller than this are built on a single thread.
const PAR_BUILD: usize = 8192;
/// Minimum batch chunk handled by one rayon task.
const PAR_CHUNK: usize = 256;
/// Size of the traversal stack (depth is bounded by `MAX_SAH_DEPTH` + log2 n).
const STACK: usize = 128;
/// Slack on the barycentric coordinates of a ray hit, so that a ray through
/// a shared edge does not fall between two triangles.
const BARY_EPS: f64 = 1e-9;
/// Smallest step used to move past a hit in [`Bvh::ray_hits`].
const MIN_GAP: f64 = 1e-9;

/// Closest point on the surface to a query point.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ClosestHit {
    /// Index of the triangle holding the point.
    pub face: u32,
    /// Closest point on the surface.
    pub point: Vec3,
    /// Distance from the query point.
    pub distance: f32,
    /// Barycentric coordinates `(w0, w1, w2)` of `point` in the triangle.
    pub barycentric: Vec3,
}

/// Intersection of a ray with the surface.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RayHit {
    /// Index of the triangle hit.
    pub face: u32,
    /// Ray parameter of the hit (`origin + t * dir`).
    pub t: f32,
    /// Barycentric coordinates `(w0, w1, w2)` of the hit inside the triangle.
    pub barycentric: Vec3,
}

#[derive(Debug, Clone, Copy)]
struct Node {
    min: [f32; 3],
    max: [f32; 3],
    /// Leaf: first triangle slot. Inner: offset of the right child from this
    /// node (the left child is the next node).
    a: u32,
    /// Number of triangles for a leaf, `0` for an inner node.
    count: u32,
}

impl Node {
    fn is_leaf(&self) -> bool {
        self.count > 0
    }

    fn min_d(&self) -> DVec3 {
        DVec3::new(self.min[0] as f64, self.min[1] as f64, self.min[2] as f64)
    }

    fn max_d(&self) -> DVec3 {
        DVec3::new(self.max[0] as f64, self.max[1] as f64, self.max[2] as f64)
    }
}

/// Acceleration structure over the triangles of a mesh.
#[derive(Debug, Clone, Default)]
pub struct Bvh {
    nodes: Vec<Node>,
    /// Triangle corners in tree order.
    tris: Vec<[Vec3; 3]>,
    /// Face index of every slot of `tris`.
    ids: Vec<u32>,
}

/// Build item: box of one triangle and its face index.
#[derive(Debug, Clone, Copy)]
struct Item {
    min: Vec3,
    max: Vec3,
    id: u32,
}

impl Item {
    fn centroid(&self) -> Vec3 {
        (self.min + self.max) * 0.5
    }
}

impl Bvh {
    /// Builds the hierarchy over `mesh`.
    ///
    /// An empty mesh gives an empty tree whose queries return nothing.
    ///
    /// # Errors
    /// [`GeomError::InvalidInput`] when a triangle index is out of range.
    pub fn build(mesh: &TriMesh) -> Result<Self> {
        let nv = mesh.positions.len();
        if let Some(f) = mesh
            .tris
            .par_iter()
            .position_any(|t| t.iter().any(|&i| i as usize >= nv))
        {
            return Err(GeomError::InvalidInput(format!(
                "triangle {f} refers to a vertex out of range"
            )));
        }
        let mut items: Vec<Item> = mesh
            .tris
            .par_iter()
            .enumerate()
            .filter_map(|(id, t)| {
                let p = t.map(|i| mesh.positions[i as usize]);
                if !(p[0].is_finite() && p[1].is_finite() && p[2].is_finite()) {
                    return None;
                }
                Some(Item {
                    min: p[0].min(p[1]).min(p[2]),
                    max: p[0].max(p[1]).max(p[2]),
                    id: id as u32,
                })
            })
            .collect();
        if items.is_empty() {
            return Ok(Self::default());
        }
        let nodes = build_par(&mut items, 0, 0);
        let ids: Vec<u32> = items.iter().map(|it| it.id).collect();
        let tris: Vec<[Vec3; 3]> = ids
            .par_iter()
            .map(|&id| mesh.tris[id as usize].map(|i| mesh.positions[i as usize]))
            .collect();
        Ok(Self { nodes, tris, ids })
    }

    /// `true` when the tree holds no triangle.
    pub fn is_empty(&self) -> bool {
        self.nodes.is_empty()
    }

    /// Number of triangles in the tree (skipped non-finite ones excluded).
    pub fn triangle_count(&self) -> usize {
        self.ids.len()
    }

    /// Number of nodes (for tests and diagnostics).
    pub fn node_count(&self) -> usize {
        self.nodes.len()
    }

    /// Bounding box of all triangles; `None` for an empty tree.
    pub fn bounds(&self) -> Option<Aabb> {
        let n = self.nodes.first()?;
        Some(Aabb {
            min: Vec3::from(n.min),
            max: Vec3::from(n.max),
        })
    }

    // ----- closest point -------------------------------------------------

    /// Closest surface point to `p` within `max_distance` (use
    /// `f32::INFINITY` for no limit). `None` when the tree is empty, `p` is
    /// not finite or nothing lies within the limit.
    pub fn closest_point(&self, p: Vec3, max_distance: f32) -> Option<ClosestHit> {
        if self.nodes.is_empty() || !p.is_finite() || max_distance.is_nan() || max_distance < 0.0 {
            return None;
        }
        let pd = p.as_dvec3();
        let mut best_d2 = if max_distance.is_finite() {
            (max_distance as f64) * (max_distance as f64)
        } else {
            f64::INFINITY
        };
        let mut best: Option<(u32, DVec3, DVec3)> = None;
        let mut stack = [0u32; STACK];
        let mut sp = 1;
        while sp > 0 {
            sp -= 1;
            let ni = stack[sp] as usize;
            let n = &self.nodes[ni];
            if box_d2(pd, n) > best_d2 {
                continue;
            }
            if n.is_leaf() {
                let s = n.a as usize;
                for slot in s..s + n.count as usize {
                    let [a, b, c] = self.tris[slot].map(|v| v.as_dvec3());
                    let (pt, bary) = closest_on_triangle(pd, a, b, c);
                    let d2 = (pt - pd).length_squared();
                    let face = self.ids[slot];
                    let better = match best {
                        None => d2 <= best_d2,
                        Some((bf, ..)) => d2 < best_d2 || (d2 == best_d2 && face < bf),
                    };
                    if better {
                        best_d2 = d2;
                        best = Some((face, pt, bary));
                    }
                }
            } else {
                let (l, r) = (ni + 1, ni + n.a as usize);
                let (dl, dr) = (box_d2(pd, &self.nodes[l]), box_d2(pd, &self.nodes[r]));
                // push the farther child first so that the nearer is popped next
                let (far, near, dfar, dnear) = if dl <= dr {
                    (r, l, dr, dl)
                } else {
                    (l, r, dl, dr)
                };
                if dfar <= best_d2 {
                    stack[sp] = far as u32;
                    sp += 1;
                }
                if dnear <= best_d2 {
                    stack[sp] = near as u32;
                    sp += 1;
                }
            }
        }
        best.map(|(face, pt, bary)| ClosestHit {
            face,
            point: pt.as_vec3(),
            distance: best_d2.sqrt() as f32,
            barycentric: bary.as_vec3(),
        })
    }

    /// [`Bvh::closest_point`] for many points, in parallel. One entry per
    /// input point, in input order.
    pub fn closest_points(&self, points: &[Vec3], max_distance: f32) -> Vec<Option<ClosestHit>> {
        points
            .par_iter()
            .with_min_len(PAR_CHUNK)
            .map(|&p| self.closest_point(p, max_distance))
            .collect()
    }

    // ----- rays ----------------------------------------------------------

    /// First hit of the ray `origin + t * dir`, `t` in `[0, max_t]` (use
    /// `f32::INFINITY` for a ray of unlimited length).
    pub fn ray_cast(&self, origin: Vec3, dir: Vec3, max_t: f32) -> Option<RayHit> {
        self.ray_cast_range(origin, dir, 0.0, max_t)
    }

    /// First hit of the ray with `t` in `[t_min, t_max]`.
    pub fn ray_cast_range(
        &self,
        origin: Vec3,
        dir: Vec3,
        t_min: f32,
        t_max: f32,
    ) -> Option<RayHit> {
        self.first_hit(
            origin.as_dvec3(),
            dir.as_dvec3(),
            t_min as f64,
            t_max as f64,
            origin.is_finite() && dir.is_finite(),
        )
    }

    /// Every intersection of the ray with a triangle, `t` in `[0, max_t]`,
    /// sorted by `t` (ties by face). A ray through a shared edge or vertex
    /// reports each adjacent triangle; use [`Bvh::ray_hits`] to merge them.
    pub fn ray_all(&self, origin: Vec3, dir: Vec3, max_t: f32) -> Vec<RayHit> {
        let mut out = Vec::new();
        if self.nodes.is_empty()
            || !origin.is_finite()
            || !dir.is_finite()
            || dir == Vec3::ZERO
            || max_t.is_nan()
            || max_t < 0.0
        {
            return out;
        }
        let (o, d) = (origin.as_dvec3(), dir.as_dvec3());
        let t_max = max_t as f64;
        let inv = inv_dir(d);
        let mut stack = [0u32; STACK];
        let mut sp = 1;
        while sp > 0 {
            sp -= 1;
            let ni = stack[sp] as usize;
            let n = &self.nodes[ni];
            if slab(o, inv, n, 0.0, t_max).is_none() {
                continue;
            }
            if n.is_leaf() {
                let s = n.a as usize;
                for slot in s..s + n.count as usize {
                    let [a, b, c] = self.tris[slot].map(|v| v.as_dvec3());
                    if let Some((t, bary)) = ray_triangle(o, d, a, b, c)
                        && (0.0..=t_max).contains(&t)
                    {
                        out.push(RayHit {
                            face: self.ids[slot],
                            t: t as f32,
                            barycentric: bary.as_vec3(),
                        });
                    }
                }
            } else {
                stack[sp] = (ni + 1) as u32;
                stack[sp + 1] = (ni + n.a as usize) as u32;
                sp += 2;
            }
        }
        out.sort_by(|x, y| x.t.total_cmp(&y.t).then(x.face.cmp(&y.face)));
        out
    }

    /// Up to `max_hits` successive hits along the ray, `t` in `[0, max_t]`.
    ///
    /// Same rule as the oracle: hit `h` is the first hit past `t(h-1) + gap`,
    /// so the two triangles at a shared edge count once and a hit closer than
    /// `gap` to the previous one is skipped. `gap` is in units of `t` (of
    /// length when `dir` is a unit vector); values below `1e-9` are raised to
    /// `1e-9` so that the search always advances.
    pub fn ray_hits(
        &self,
        origin: Vec3,
        dir: Vec3,
        max_t: f32,
        max_hits: usize,
        gap: f32,
    ) -> Vec<RayHit> {
        let mut out = Vec::new();
        let valid = origin.is_finite() && dir.is_finite();
        let (o, d) = (origin.as_dvec3(), dir.as_dvec3());
        let gap = (gap as f64).max(MIN_GAP);
        let mut t_min = 0.0f64;
        while out.len() < max_hits {
            let Some(hit) = self.first_hit(o, d, t_min, max_t as f64, valid) else {
                break;
            };
            // the f32 `t` of the hit is rounded: advance from the exact value
            t_min = hit.t as f64 + gap;
            out.push(hit);
        }
        out
    }

    /// [`Bvh::ray_cast`] for many rays, in parallel. `origins` and `dirs` are
    /// paired; the shorter one decides the number of rays.
    pub fn ray_casts(&self, origins: &[Vec3], dirs: &[Vec3], max_t: f32) -> Vec<Option<RayHit>> {
        origins
            .par_iter()
            .zip(dirs.par_iter())
            .with_min_len(PAR_CHUNK)
            .map(|(&o, &d)| self.ray_cast(o, d, max_t))
            .collect()
    }

    /// [`Bvh::ray_hits`] for many rays, in parallel. `origins` and `dirs` are
    /// paired; the shorter one decides the number of rays.
    pub fn ray_hits_batch(
        &self,
        origins: &[Vec3],
        dirs: &[Vec3],
        max_t: f32,
        max_hits: usize,
        gap: f32,
    ) -> Vec<Vec<RayHit>> {
        origins
            .par_iter()
            .zip(dirs.par_iter())
            .with_min_len(PAR_CHUNK)
            .map(|(&o, &d)| self.ray_hits(o, d, max_t, max_hits, gap))
            .collect()
    }

    fn first_hit(&self, o: DVec3, d: DVec3, t_min: f64, t_max: f64, valid: bool) -> Option<RayHit> {
        if self.nodes.is_empty()
            || !valid
            || d == DVec3::ZERO
            || t_min.is_nan()
            || t_max.is_nan()
            || t_min > t_max
        {
            return None;
        }
        let inv = inv_dir(d);
        let mut limit = t_max;
        let mut best: Option<(u32, DVec3)> = None;
        let mut stack = [0u32; STACK];
        let mut sp = 1;
        while sp > 0 {
            sp -= 1;
            let ni = stack[sp] as usize;
            let n = &self.nodes[ni];
            if slab(o, inv, n, t_min, limit).is_none() {
                continue;
            }
            if n.is_leaf() {
                let s = n.a as usize;
                for slot in s..s + n.count as usize {
                    let [a, b, c] = self.tris[slot].map(|v| v.as_dvec3());
                    let Some((t, bary)) = ray_triangle(o, d, a, b, c) else {
                        continue;
                    };
                    if t < t_min {
                        continue;
                    }
                    let face = self.ids[slot];
                    let better = match best {
                        None => t <= limit,
                        Some((bf, _)) => t < limit || (t == limit && face < bf),
                    };
                    if better {
                        limit = t;
                        best = Some((face, bary));
                    }
                }
            } else {
                let (l, r) = (ni + 1, ni + n.a as usize);
                let dl = slab(o, inv, &self.nodes[l], t_min, limit);
                let dr = slab(o, inv, &self.nodes[r], t_min, limit);
                // visit the nearer child first
                match (dl, dr) {
                    (Some(a), Some(b)) => {
                        let (far, near) = if a <= b { (r, l) } else { (l, r) };
                        stack[sp] = far as u32;
                        stack[sp + 1] = near as u32;
                        sp += 2;
                    }
                    (Some(_), None) => {
                        stack[sp] = l as u32;
                        sp += 1;
                    }
                    (None, Some(_)) => {
                        stack[sp] = r as u32;
                        sp += 1;
                    }
                    (None, None) => {}
                }
            }
        }
        best.map(|(face, bary)| RayHit {
            face,
            t: limit as f32,
            barycentric: bary.as_vec3(),
        })
    }

    // ----- overlap -------------------------------------------------------

    /// Faces with at least one point within `radius` of `center` (the
    /// triangle touches the closed ball), sorted by face index.
    pub fn overlap_sphere(&self, center: Vec3, radius: f32) -> Vec<u32> {
        let mut out = Vec::new();
        if self.nodes.is_empty() || !center.is_finite() || radius.is_nan() || radius < 0.0 {
            return out;
        }
        let c = center.as_dvec3();
        let r2 = (radius as f64) * (radius as f64);
        let mut stack = [0u32; STACK];
        let mut sp = 1;
        while sp > 0 {
            sp -= 1;
            let ni = stack[sp] as usize;
            let n = &self.nodes[ni];
            if box_d2(c, n) > r2 {
                continue;
            }
            if n.is_leaf() {
                let s = n.a as usize;
                for slot in s..s + n.count as usize {
                    let [a, b, cc] = self.tris[slot].map(|v| v.as_dvec3());
                    let (pt, _) = closest_on_triangle(c, a, b, cc);
                    if (pt - c).length_squared() <= r2 {
                        out.push(self.ids[slot]);
                    }
                }
            } else {
                stack[sp] = (ni + 1) as u32;
                stack[sp + 1] = (ni + n.a as usize) as u32;
                sp += 2;
            }
        }
        out.sort_unstable();
        out
    }

    /// Faces that intersect the closed box `[min, max]` (exact
    /// triangle/box test), sorted by face index.
    pub fn overlap_aabb(&self, min: Vec3, max: Vec3) -> Vec<u32> {
        let mut out = Vec::new();
        if self.nodes.is_empty() || !min.is_finite() || !max.is_finite() || min.cmpgt(max).any() {
            return out;
        }
        let (bmin, bmax) = (min.as_dvec3(), max.as_dvec3());
        let (center, half) = ((bmin + bmax) * 0.5, (bmax - bmin) * 0.5);
        let mut stack = [0u32; STACK];
        let mut sp = 1;
        while sp > 0 {
            sp -= 1;
            let ni = stack[sp] as usize;
            let n = &self.nodes[ni];
            if n.min_d().cmpgt(bmax).any() || n.max_d().cmplt(bmin).any() {
                continue;
            }
            if n.is_leaf() {
                let s = n.a as usize;
                for slot in s..s + n.count as usize {
                    let t = self.tris[slot].map(|v| v.as_dvec3());
                    if tri_box_overlap(&t, center, half) {
                        out.push(self.ids[slot]);
                    }
                }
            } else {
                stack[sp] = (ni + 1) as u32;
                stack[sp + 1] = (ni + n.a as usize) as u32;
                sp += 2;
            }
        }
        out.sort_unstable();
        out
    }

    /// [`Bvh::overlap_sphere`] for many balls of the same radius, in parallel.
    pub fn overlap_spheres(&self, centers: &[Vec3], radius: f32) -> Vec<Vec<u32>> {
        centers
            .par_iter()
            .with_min_len(PAR_CHUNK)
            .map(|&c| self.overlap_sphere(c, radius))
            .collect()
    }
}

// ----- build ---------------------------------------------------------------

fn surface_area(min: Vec3, max: Vec3) -> f32 {
    let e = (max - min).max(Vec3::ZERO);
    2.0 * (e.x * e.y + e.y * e.z + e.z * e.x)
}

struct Bounds {
    min: Vec3,
    max: Vec3,
    cmin: Vec3,
    cmax: Vec3,
}

fn bounds_of(items: &[Item]) -> Bounds {
    let mut b = Bounds {
        min: Vec3::splat(f32::INFINITY),
        max: Vec3::splat(f32::NEG_INFINITY),
        cmin: Vec3::splat(f32::INFINITY),
        cmax: Vec3::splat(f32::NEG_INFINITY),
    };
    for it in items {
        b.min = b.min.min(it.min);
        b.max = b.max.max(it.max);
        let c = it.centroid();
        b.cmin = b.cmin.min(c);
        b.cmax = b.cmax.max(c);
    }
    b
}

fn make_node(b: &Bounds, a: u32, count: u32) -> Node {
    Node {
        min: b.min.to_array(),
        max: b.max.to_array(),
        a,
        count,
    }
}

/// Partitions `items` in place and returns the size of the left part, or
/// `None` when the node should be a leaf.
fn split(items: &mut [Item], b: &Bounds, depth: u32) -> Option<usize> {
    let n = items.len();
    if n <= MIN_LEAF {
        return None;
    }
    let extent = b.cmax - b.cmin;
    let longest = if extent.x >= extent.y && extent.x >= extent.z {
        0
    } else if extent.y >= extent.z {
        1
    } else {
        2
    };
    if extent.max_element() <= 0.0 {
        // all centroids coincide: any split is as good as another
        return (n > MAX_LEAF).then_some(n / 2);
    }
    if depth >= MAX_SAH_DEPTH {
        return Some(median_split(items, longest));
    }
    let mut best = (f32::INFINITY, 0usize, 0usize); // cost, axis, bin
    for axis in 0..3 {
        if extent[axis] <= 0.0 {
            continue;
        }
        let scale = BINS as f32 / extent[axis];
        let mut count = [0u32; BINS];
        let mut bmin = [Vec3::splat(f32::INFINITY); BINS];
        let mut bmax = [Vec3::splat(f32::NEG_INFINITY); BINS];
        for it in items.iter() {
            let k = bin_of(it.centroid()[axis], b.cmin[axis], scale);
            count[k] += 1;
            bmin[k] = bmin[k].min(it.min);
            bmax[k] = bmax[k].max(it.max);
        }
        // right-to-left sweep: area and count of everything from bin k on
        let mut right_area = [0.0f32; BINS];
        let (mut rmin, mut rmax) = (Vec3::splat(f32::INFINITY), Vec3::splat(f32::NEG_INFINITY));
        for k in (1..BINS).rev() {
            rmin = rmin.min(bmin[k]);
            rmax = rmax.max(bmax[k]);
            right_area[k] = if rmin.x <= rmax.x {
                surface_area(rmin, rmax)
            } else {
                0.0
            };
        }
        let (mut lmin, mut lmax) = (Vec3::splat(f32::INFINITY), Vec3::splat(f32::NEG_INFINITY));
        let mut nl = 0u32;
        for k in 0..BINS - 1 {
            nl += count[k];
            lmin = lmin.min(bmin[k]);
            lmax = lmax.max(bmax[k]);
            let nr = n as u32 - nl;
            if nl == 0 || nr == 0 {
                continue;
            }
            let cost = surface_area(lmin, lmax) * nl as f32 + right_area[k + 1] * nr as f32;
            if cost < best.0 {
                best = (cost, axis, k);
            }
        }
    }
    if !best.0.is_finite() {
        return Some(median_split(items, longest));
    }
    if n <= MAX_LEAF {
        let sa = surface_area(b.min, b.max).max(f32::MIN_POSITIVE);
        let split_cost = TRAVERSAL_COST + best.0 / sa;
        if n as f32 <= split_cost {
            return None;
        }
    }
    let (_, axis, bin) = best;
    let scale = BINS as f32 / extent[axis];
    let mut i = 0;
    let mut j = n;
    while i < j {
        if bin_of(items[i].centroid()[axis], b.cmin[axis], scale) <= bin {
            i += 1;
        } else {
            j -= 1;
            items.swap(i, j);
        }
    }
    if i == 0 || i == n {
        return Some(median_split(items, longest));
    }
    Some(i)
}

fn bin_of(c: f32, cmin: f32, scale: f32) -> usize {
    (((c - cmin) * scale) as usize).min(BINS - 1)
}

fn median_split(items: &mut [Item], axis: usize) -> usize {
    let mid = items.len() / 2;
    items.select_nth_unstable_by(mid, |a, b| {
        a.centroid()[axis].total_cmp(&b.centroid()[axis])
    });
    mid
}

/// Sequential build of the subtree over `items`; `start` is the slot of
/// `items[0]` in the final triangle order.
fn build_rec(items: &mut [Item], start: usize, depth: u32, out: &mut Vec<Node>) {
    let b = bounds_of(items);
    match split(items, &b, depth) {
        None => out.push(make_node(&b, start as u32, items.len() as u32)),
        Some(mid) => {
            let me = out.len();
            out.push(make_node(&b, 0, 0));
            let (left, right) = items.split_at_mut(mid);
            build_rec(left, start, depth + 1, out);
            out[me].a = (out.len() - me) as u32;
            build_rec(right, start + mid, depth + 1, out);
        }
    }
}

/// Build that forks onto rayon for large subtrees.
fn build_par(items: &mut [Item], start: usize, depth: u32) -> Vec<Node> {
    if items.len() < PAR_BUILD {
        let mut out = Vec::with_capacity(items.len());
        build_rec(items, start, depth, &mut out);
        return out;
    }
    let b = bounds_of(items);
    let Some(mid) = split(items, &b, depth) else {
        return vec![make_node(&b, start as u32, items.len() as u32)];
    };
    let (left, right) = items.split_at_mut(mid);
    let (l, r) = rayon::join(
        || build_par(left, start, depth + 1),
        || build_par(right, start + mid, depth + 1),
    );
    let mut out = Vec::with_capacity(1 + l.len() + r.len());
    out.push(make_node(&b, (1 + l.len()) as u32, 0));
    out.extend(l);
    out.extend(r);
    out
}

// ----- query kernels (f64) ---------------------------------------------------

/// Squared distance from `p` to the box of `n`.
fn box_d2(p: DVec3, n: &Node) -> f64 {
    let d = (n.min_d() - p).max(p - n.max_d()).max(DVec3::ZERO);
    d.length_squared()
}

/// Inverse direction with a huge finite value for zero components, so that
/// `0 * inf = NaN` cannot occur in the slab test.
fn inv_dir(d: DVec3) -> DVec3 {
    let f = |x: f64| if x.abs() < 1e-300 { 1e300 } else { 1.0 / x };
    DVec3::new(f(d.x), f(d.y), f(d.z))
}

/// Entry parameter of the ray into the box of `n`, if it overlaps `[t0, t1]`.
fn slab(o: DVec3, inv: DVec3, n: &Node, t0: f64, t1: f64) -> Option<f64> {
    let a = (n.min_d() - o) * inv;
    let b = (n.max_d() - o) * inv;
    let tn = a.min(b).max_element().max(t0);
    let tf = a.max(b).min_element().min(t1);
    // slack of a few ulps: the box may touch the ray only at its far face
    (tn <= tf + 1e-12 * (1.0 + tf.abs())).then_some(tn)
}

/// Two-sided Möller-Trumbore intersection in `f64`: `(t, barycentric)`.
/// `None` for a ray parallel to the plane (including a ray inside it) and
/// for a zero-area triangle.
fn ray_triangle(o: DVec3, d: DVec3, a: DVec3, b: DVec3, c: DVec3) -> Option<(f64, DVec3)> {
    let e1 = b - a;
    let e2 = c - a;
    let p = d.cross(e2);
    let det = e1.dot(p);
    let scale = d.length() * e1.cross(e2).length();
    if det.abs() <= 1e-12 * scale {
        return None;
    }
    let inv = 1.0 / det;
    let s = o - a;
    let u = s.dot(p) * inv;
    if !(-BARY_EPS..=1.0 + BARY_EPS).contains(&u) {
        return None;
    }
    let q = s.cross(e1);
    let v = d.dot(q) * inv;
    if v < -BARY_EPS || u + v > 1.0 + BARY_EPS {
        return None;
    }
    let t = e2.dot(q) * inv;
    let w = DVec3::new(1.0 - u - v, u, v).max(DVec3::ZERO);
    Some((t, w / (w.x + w.y + w.z)))
}

fn closest_on_segment(p: DVec3, a: DVec3, b: DVec3) -> (DVec3, f64) {
    let ab = b - a;
    let len2 = ab.length_squared();
    let t = if len2 > 0.0 {
        ((p - a).dot(ab) / len2).clamp(0.0, 1.0)
    } else {
        0.0
    };
    (a + ab * t, t)
}

/// Closest point of triangle `abc` to `p` and its barycentric coordinates.
/// Zero-area triangles are treated as their longest segment or a point.
fn closest_on_triangle(p: DVec3, a: DVec3, b: DVec3, c: DVec3) -> (DVec3, DVec3) {
    let ab = b - a;
    let ac = c - a;
    let n2 = ab.cross(ac).length_squared();
    let max_e2 = ab
        .length_squared()
        .max(ac.length_squared())
        .max((c - b).length_squared());
    if n2 <= 1e-20 * max_e2 * max_e2 {
        // degenerate: best of the three segments
        let (p0, t0) = closest_on_segment(p, a, b);
        let (p1, t1) = closest_on_segment(p, b, c);
        let (p2, t2) = closest_on_segment(p, c, a);
        let (d0, d1, d2) = (
            (p0 - p).length_squared(),
            (p1 - p).length_squared(),
            (p2 - p).length_squared(),
        );
        return if d0 <= d1 && d0 <= d2 {
            (p0, DVec3::new(1.0 - t0, t0, 0.0))
        } else if d1 <= d2 {
            (p1, DVec3::new(0.0, 1.0 - t1, t1))
        } else {
            (p2, DVec3::new(t2, 0.0, 1.0 - t2))
        };
    }
    let ap = p - a;
    let d1 = ab.dot(ap);
    let d2 = ac.dot(ap);
    if d1 <= 0.0 && d2 <= 0.0 {
        return (a, DVec3::X);
    }
    let bp = p - b;
    let d3 = ab.dot(bp);
    let d4 = ac.dot(bp);
    if d3 >= 0.0 && d4 <= d3 {
        return (b, DVec3::Y);
    }
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let v = d1 / (d1 - d3);
        return (a + ab * v, DVec3::new(1.0 - v, v, 0.0));
    }
    let cp = p - c;
    let d5 = ab.dot(cp);
    let d6 = ac.dot(cp);
    if d6 >= 0.0 && d5 <= d6 {
        return (c, DVec3::Z);
    }
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let w = d2 / (d2 - d6);
        return (a + ac * w, DVec3::new(1.0 - w, 0.0, w));
    }
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return (b + (c - b) * w, DVec3::new(0.0, 1.0 - w, w));
    }
    let denom = 1.0 / (va + vb + vc);
    let v = vb * denom;
    let w = vc * denom;
    (a + ab * v + ac * w, DVec3::new(1.0 - v - w, v, w))
}

/// Separating-axis test of a triangle against a box (centre, half extents).
fn tri_box_overlap(tri: &[DVec3; 3], center: DVec3, half: DVec3) -> bool {
    let v = tri.map(|p| p - center);
    // the three box axes
    for ax in 0..3 {
        let lo = v[0][ax].min(v[1][ax]).min(v[2][ax]);
        let hi = v[0][ax].max(v[1][ax]).max(v[2][ax]);
        if lo > half[ax] || hi < -half[ax] {
            return false;
        }
    }
    let e = [v[1] - v[0], v[2] - v[1], v[0] - v[2]];
    // nine axes: box axis x triangle edge
    for edge in &e {
        for unit in [DVec3::X, DVec3::Y, DVec3::Z] {
            let axis = unit.cross(*edge);
            let (p0, p1, p2) = (v[0].dot(axis), v[1].dot(axis), v[2].dot(axis));
            let r = half.x * axis.x.abs() + half.y * axis.y.abs() + half.z * axis.z.abs();
            if p0.min(p1).min(p2) > r || p0.max(p1).max(p2) < -r {
                return false;
            }
        }
    }
    // the triangle plane
    let n = e[0].cross(e[1]);
    let r = half.x * n.x.abs() + half.y * n.y.abs() + half.z * n.z.abs();
    n.dot(v[0]).abs() <= r
}

#[cfg(test)]
mod tests;
