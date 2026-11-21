//! Ray casting and distance queries on a triangle mesh: a binary bounding
//! volume hierarchy over the faces. The fitting-side detection, the
//! orientation search and the supports use it.
//!
//! The tree is built by median splits of the face centroids along the
//! longest axis. Ties are broken by face index and every leaf is sorted, so
//! the same mesh always gives the same tree on every platform. Queries
//! return the minimum of (distance, face index), so their result does not
//! depend on the traversal order either.

use dc_mesh::TriMesh;
use glam::Vec3;

/// Faces per leaf, at most.
const LEAF: usize = 4;
/// Barycentric tolerance of the ray test: rays through an edge or a vertex
/// hit the faces that share it.
const BARY_EPS: f32 = 1e-6;
/// Traversal stack size: twice the depth of a median-split tree over
/// 2³² faces, with room to spare.
const STACK: usize = 96;

/// A triangle in leaf order: first corner, the two edges from it, its face.
#[derive(Debug, Clone, Copy)]
struct Tri {
    a: Vec3,
    e1: Vec3,
    e2: Vec3,
    face: u32,
}

/// A node: its box, and either a leaf range of [`Bvh::tris`] (`count` > 0)
/// or an inner node whose first child follows it and whose second child
/// is at `start` (`count` = 0).
#[derive(Debug, Clone, Copy)]
struct Node {
    min: Vec3,
    max: Vec3,
    start: u32,
    count: u32,
}

/// A ray hit: distance along the (unit) ray and the face hit.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct Hit {
    /// Distance from the origin, mm.
    pub t: f32,
    /// Face index in the mesh.
    pub face: u32,
}

/// Bounding volume hierarchy over some faces of a mesh.
#[derive(Debug, Clone, Default)]
pub(crate) struct Bvh {
    nodes: Vec<Node>,
    tris: Vec<Tri>,
}

impl Bvh {
    /// Tree over every face of `mesh`.
    pub fn new(mesh: &TriMesh) -> Self {
        Self::with_faces(mesh, 0..mesh.face_count() as u32)
    }

    /// Tree over the faces `faces` of `mesh` (face indices stay the mesh's).
    pub fn with_faces(mesh: &TriMesh, faces: impl IntoIterator<Item = u32>) -> Self {
        let mut items: Vec<(Vec3, Tri)> = faces
            .into_iter()
            .map(|f| {
                let [a, b, c] = mesh.triangle(f as usize);
                (
                    (a + b + c) / 3.0,
                    Tri {
                        a,
                        e1: b - a,
                        e2: c - a,
                        face: f,
                    },
                )
            })
            .collect();
        if items.is_empty() {
            return Self::default();
        }
        let mut nodes: Vec<Node> = Vec::with_capacity(2 * items.len() / LEAF + 1);
        build(&mut items, 0, &mut nodes);
        Self {
            nodes,
            tris: items.into_iter().map(|(_, t)| t).collect(),
        }
    }

    /// `true` when the tree holds no face.
    pub fn is_empty(&self) -> bool {
        self.tris.is_empty()
    }

    /// Nearest hit of the ray `origin + t·dir` (`dir` unit) with
    /// 0 < t < `t_max`, either side of the faces. Equal distances go to the
    /// lower face index.
    pub fn cast(&self, origin: Vec3, dir: Vec3, t_max: f32) -> Option<Hit> {
        let mut best: Option<Hit> = None;
        self.walk(origin, dir, t_max, &mut |tri, limit| {
            if let Some(t) = ray_tri(origin, dir, tri, t_max) {
                let better = match best {
                    None => true,
                    Some(b) => t < b.t || (t == b.t && tri.face < b.face),
                };
                if better {
                    best = Some(Hit { t, face: tri.face });
                    *limit = t;
                }
            }
            false
        });
        best
    }

    /// Whether the ray `origin + t·dir` hits a face with 0 < t < `t_max`.
    pub fn occluded(&self, origin: Vec3, dir: Vec3, t_max: f32) -> bool {
        let mut hit = false;
        self.walk(origin, dir, t_max, &mut |tri, _| {
            hit = ray_tri(origin, dir, tri, t_max).is_some();
            hit
        });
        hit
    }

    /// Visits the leaf triangles of the nodes the ray reaches within the
    /// current limit, nearer child first. `visit` may lower the limit and
    /// returns `true` to stop. Boxes exactly at the limit are still
    /// visited, so ties are seen whatever the order.
    fn walk(
        &self,
        origin: Vec3,
        dir: Vec3,
        t_max: f32,
        visit: &mut dyn FnMut(&Tri, &mut f32) -> bool,
    ) {
        if self.nodes.is_empty() {
            return;
        }
        let inv = dir.recip();
        let mut limit = t_max;
        // (node, entry distance of its box)
        let mut stack = [(0u32, 0.0f32); STACK];
        let mut sp = 1usize;
        while sp > 0 {
            sp -= 1;
            let (n, entry) = stack[sp];
            if entry > limit {
                continue;
            }
            let node = &self.nodes[n as usize];
            if node.count > 0 {
                let range = node.start as usize..(node.start + node.count) as usize;
                for tri in &self.tris[range] {
                    if visit(tri, &mut limit) {
                        return;
                    }
                }
                continue;
            }
            let (l, r) = (n + 1, node.start);
            let tl = ray_box(origin, inv, &self.nodes[l as usize], limit);
            let tr = ray_box(origin, inv, &self.nodes[r as usize], limit);
            // Push the farther child first so the nearer one is visited first.
            match (tl, tr) {
                (Some(a), Some(b)) => {
                    let (first, second) = if a <= b {
                        ((l, a), (r, b))
                    } else {
                        ((r, b), (l, a))
                    };
                    stack[sp] = second;
                    stack[sp + 1] = first;
                    sp += 2;
                }
                (Some(a), None) => {
                    stack[sp] = (l, a);
                    sp += 1;
                }
                (None, Some(b)) => {
                    stack[sp] = (r, b);
                    sp += 1;
                }
                (None, None) => {}
            }
        }
    }

    /// Nearest face to `p` closer than `radius`: (distance, face).
    pub fn nearest(&self, p: Vec3, radius: f32) -> Option<(f32, u32)> {
        if self.nodes.is_empty() {
            return None;
        }
        let mut best: Option<(f32, u32)> = None;
        let mut limit2 = radius * radius;
        let mut stack = [0u32; STACK];
        let mut sp = 1usize;
        while sp > 0 {
            sp -= 1;
            let n = stack[sp];
            let node = &self.nodes[n as usize];
            let d = (node.min - p).max(p - node.max).max(Vec3::ZERO);
            if d.length_squared() > limit2 {
                continue;
            }
            if node.count > 0 {
                let range = node.start as usize..(node.start + node.count) as usize;
                for tri in &self.tris[range] {
                    let q = closest_on_tri(p, tri.a, tri.a + tri.e1, tri.a + tri.e2);
                    let d2 = (q - p).length_squared();
                    let better = match best {
                        None => d2 <= limit2,
                        Some((b2, f)) => d2 < b2 || (d2 == b2 && tri.face < f),
                    };
                    if better {
                        best = Some((d2, tri.face));
                        limit2 = d2;
                    }
                }
            } else {
                stack[sp] = node.start;
                stack[sp + 1] = n + 1;
                sp += 2;
            }
        }
        best.map(|(d2, f)| (d2.sqrt(), f))
    }
}

/// Builds the subtree over `items` depth first; returns its node index.
fn build(items: &mut [(Vec3, Tri)], offset: usize, nodes: &mut Vec<Node>) -> usize {
    let (mut min, mut max) = (Vec3::splat(f32::MAX), Vec3::splat(f32::MIN));
    let (mut cmin, mut cmax) = (min, max);
    for (c, t) in items.iter() {
        let (b, d) = (t.a + t.e1, t.a + t.e2);
        min = min.min(t.a).min(b).min(d);
        max = max.max(t.a).max(b).max(d);
        cmin = cmin.min(*c);
        cmax = cmax.max(*c);
    }
    let me = nodes.len();
    nodes.push(Node {
        min,
        max,
        start: offset as u32,
        count: items.len() as u32,
    });
    if items.len() <= LEAF {
        items.sort_by_key(|(_, t)| t.face);
        return me;
    }
    let ext = cmax - cmin;
    let axis = if ext.x >= ext.y && ext.x >= ext.z {
        0
    } else if ext.y >= ext.z {
        1
    } else {
        2
    };
    let mid = items.len() / 2;
    items.select_nth_unstable_by(mid, |(ca, ta), (cb, tb)| {
        ca[axis].total_cmp(&cb[axis]).then(ta.face.cmp(&tb.face))
    });
    let (left, right) = items.split_at_mut(mid);
    build(left, offset, nodes);
    let second = build(right, offset + mid, nodes);
    nodes[me].start = second as u32;
    nodes[me].count = 0;
    me
}

/// Slab test: the distance at which the ray enters `node`'s box, if it
/// does within `t_max`.
fn ray_box(origin: Vec3, inv: Vec3, node: &Node, t_max: f32) -> Option<f32> {
    let t1 = (node.min - origin) * inv;
    let t2 = (node.max - origin) * inv;
    let near = t1.min(t2).max_element().max(0.0);
    let far = t1.max(t2).min_element().min(t_max);
    (near <= far).then_some(near)
}

/// Möller–Trumbore ray-triangle test, both sides; distance in (0, t_max).
fn ray_tri(origin: Vec3, dir: Vec3, tri: &Tri, t_max: f32) -> Option<f32> {
    let p = dir.cross(tri.e2);
    let det = tri.e1.dot(p);
    if det.abs() < 1e-12 {
        return None;
    }
    let inv = 1.0 / det;
    let s = origin - tri.a;
    let u = s.dot(p) * inv;
    if !(-BARY_EPS..=1.0 + BARY_EPS).contains(&u) {
        return None;
    }
    let q = s.cross(tri.e1);
    let v = dir.dot(q) * inv;
    if v < -BARY_EPS || u + v > 1.0 + BARY_EPS {
        return None;
    }
    let t = tri.e2.dot(q) * inv;
    (t > 0.0 && t < t_max).then_some(t)
}

/// Closest point to `p` on the triangle `a b c` (Ericson, Real-Time
/// Collision Detection §5.1.5, by Voronoi regions).
pub(crate) fn closest_on_tri(p: Vec3, a: Vec3, b: Vec3, c: Vec3) -> Vec3 {
    let (ab, ac, ap) = (b - a, c - a, p - a);
    let (d1, d2) = (ab.dot(ap), ac.dot(ap));
    if d1 <= 0.0 && d2 <= 0.0 {
        return a;
    }
    let bp = p - b;
    let (d3, d4) = (ab.dot(bp), ac.dot(bp));
    if d3 >= 0.0 && d4 <= d3 {
        return b;
    }
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        return a + ab * (d1 / (d1 - d3));
    }
    let cp = p - c;
    let (d5, d6) = (ab.dot(cp), ac.dot(cp));
    if d6 >= 0.0 && d5 <= d6 {
        return c;
    }
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        return a + ac * (d2 / (d2 - d6));
    }
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && d4 - d3 >= 0.0 && d5 - d6 >= 0.0 {
        return b + (c - b) * ((d4 - d3) / ((d4 - d3) + (d5 - d6)));
    }
    let denom = 1.0 / (va + vb + vc);
    a + ab * (vb * denom) + ac * (vc * denom)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::testutil::default_shell;

    fn brute_cast(mesh: &TriMesh, o: Vec3, d: Vec3, t_max: f32) -> Option<Hit> {
        let mut best: Option<Hit> = None;
        for f in 0..mesh.face_count() {
            let [a, b, c] = mesh.triangle(f);
            let tri = Tri {
                a,
                e1: b - a,
                e2: c - a,
                face: f as u32,
            };
            if let Some(t) = ray_tri(o, d, &tri, t_max)
                && best.is_none_or(|h| t < h.t || (t == h.t && (f as u32) < h.face))
            {
                best = Some(Hit { t, face: f as u32 });
            }
        }
        best
    }

    #[test]
    fn rays_and_distances_match_brute_force() {
        let shell = default_shell();
        let mesh = &shell.mesh;
        let bvh = Bvh::new(mesh);
        let mut s = 0x2545_f491_u32;
        let mut rnd = || {
            s ^= s << 13;
            s ^= s >> 17;
            s ^= s << 5;
            s as f32 / u32::MAX as f32
        };
        let mut hits = 0;
        for _ in 0..300 {
            let o = Vec3::new(rnd() * 70.0 - 35.0, rnd() * 60.0 - 30.0, rnd() * 16.0 - 2.0);
            let d = Vec3::new(rnd() - 0.5, rnd() - 0.5, rnd() - 0.5).normalize();
            let fast = bvh.cast(o, d, 40.0);
            assert_eq!(fast, brute_cast(mesh, o, d, 40.0));
            assert_eq!(bvh.occluded(o, d, 40.0), fast.is_some());
            hits += usize::from(fast.is_some());
            let near = bvh.nearest(o, 3.0);
            let brute = (0..mesh.face_count())
                .map(|f| {
                    let [a, b, c] = mesh.triangle(f);
                    (closest_on_tri(o, a, b, c).distance(o), f as u32)
                })
                .filter(|(d, _)| *d <= 3.0)
                .min_by(|x, y| x.0.total_cmp(&y.0).then(x.1.cmp(&y.1)));
            assert_eq!(near.map(|n| n.1), brute.map(|b| b.1));
        }
        assert!(hits > 50, "too few hits to test: {hits}");
    }

    #[test]
    fn subset_tree_sees_only_its_faces() {
        let shell = default_shell();
        let mesh = &shell.mesh;
        let faces: Vec<u32> = (0..mesh.face_count() as u32)
            .filter(|f| f % 3 == 0)
            .collect();
        let bvh = Bvh::with_faces(mesh, faces.iter().copied());
        for f in (0..mesh.face_count()).step_by(97) {
            let [a, b, c] = mesh.triangle(f);
            let centre = (a + b + c) / 3.0;
            let (d, face) = bvh.nearest(centre, 5.0).unwrap();
            assert_eq!(face % 3, 0);
            if f % 3 == 0 {
                assert!(d < 1e-4);
            }
        }
        assert!(Bvh::with_faces(mesh, []).is_empty());
        assert_eq!(
            Bvh::with_faces(mesh, []).cast(Vec3::ZERO, Vec3::Z, 1.0),
            None
        );
    }
}
