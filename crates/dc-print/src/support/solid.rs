//! Meshes of the support solids: struts (8-sided tapered prisms with a cone
//! tip) and the raft (chamfered pads around the feet, joined by chamfered
//! bands). All are closed and outward-oriented, and the slicer's nonzero
//! rule merges them with the part and with each other.
//!
//! The geometry uses fixed directions (multiples of 22.5° and 15°, as
//! constants) and vector rotations only, never the platform maths library,
//! so the meshes are the same bits on every platform.

use dc_mesh::TriMesh;
use glam::Vec2;

use crate::part::Strut;

/// cos and sin of 22.5°.
const C22: f32 = 0.923_879_5;
const S22: f32 = 0.382_683_43;
/// Ring directions of a strut, 22.5° + k·45°, counter-clockwise.
pub(crate) const RING: [Vec2; 8] = [
    Vec2::new(C22, S22),
    Vec2::new(S22, C22),
    Vec2::new(-S22, C22),
    Vec2::new(-C22, S22),
    Vec2::new(-C22, -S22),
    Vec2::new(-S22, -C22),
    Vec2::new(S22, -C22),
    Vec2::new(C22, -S22),
];
/// cos and sin of 15°: the step of the raft pads' outline.
const C15: f32 = 0.965_925_8;
const S15: f32 = 0.258_819_04;
/// Sides of a raft pad.
const PAD_SIDES: usize = 24;
/// Shortest band between two pads, mm: a shorter one lies inside them.
const MIN_BAND_MM: f32 = 0.01;

/// How far the strut top goes into the part, above the contact, mm.
pub(crate) const PENETRATION_MM: f32 = 0.2;
/// Length of the cone from the shaft to the tip, mm (shorter on short struts).
pub(crate) const CONE_MM: f32 = 2.0;
/// Shaft diameter at the foot over the nominal shaft diameter.
pub(crate) const FOOT_FLARE: f32 = 1.25;
/// How far a strut sinks into the raft, mm.
const RAFT_OVERLAP_MM: f32 = 0.1;

/// Cone length of a strut whose contact is `height` above its foot.
pub(crate) fn cone_len(height: f32) -> f32 {
    CONE_MM.min(0.6 * height.max(0.0))
}

/// The solid of one strut. Its foot ring sits at the foot (sunk slightly
/// into the raft) with the shaft diameter × [`FOOT_FLARE`]. The shaft
/// narrows to the shaft diameter where the cone starts, and the cone ends
/// at the tip diameter [`PENETRATION_MM`] above the contact.
pub(crate) fn strut_mesh(s: &Strut) -> TriMesh {
    let axis = s.tip.truncate();
    let bottom = (s.foot.z - RAFT_OVERLAP_MM).max(0.0);
    let height = s.tip.z - s.foot.z;
    let cone = cone_len(height);
    let (shaft_r, tip_r) = (0.5 * s.shaft_d_mm, 0.5 * s.tip_d_mm);
    let rings = [
        (bottom, shaft_r * FOOT_FLARE),
        (s.tip.z - cone, shaft_r),
        (s.tip.z + PENETRATION_MM, tip_r),
    ];
    let mut positions = Vec::with_capacity(24);
    for (z, r) in rings {
        positions.extend(RING.iter().map(|d| (axis + *d * r).extend(z)));
    }
    let mut tris = Vec::with_capacity(44);
    for k in 1..7u32 {
        tris.push([0, k + 1, k]);
        tris.push([16, 16 + k, 16 + k + 1]);
    }
    for ring in 0..2u32 {
        let (a0, b0) = (8 * ring, 8 * ring + 8);
        for k in 0..8u32 {
            let k1 = (k + 1) % 8;
            tris.push([a0 + k, a0 + k1, b0 + k1]);
            tris.push([a0 + k, b0 + k1, b0 + k]);
        }
    }
    TriMesh::new(positions, tris)
}

/// The raft, from the plate (z = 0) up to `thickness`, following the feet
/// (the U of an arch) instead of filling their hull:
/// - a pad around each foot, `pad` in radius on the top face;
/// - a band along each edge of the feet's minimum spanning tree
///   ([`spanning_tree`]), from foot to foot, `band` (at most `pad`) in
///   half-width on the top face, so its square ends lie inside the pads.
///
/// Every piece is chamfered at 45°: its bottom outline is `thickness`
/// wider all round than its top, so a scraper gets under the edge. The
/// pieces are closed, outward-oriented and overlap: the slicer's nonzero
/// rule merges them, and as they share the 45° slope their union is the
/// union of the outlines, chamfered. Empty without feet or thickness.
pub(crate) fn raft_mesh(feet: &[Vec2], thickness: f32, pad: f32, band: f32) -> TriMesh {
    let mut mesh = TriMesh::default();
    if feet.is_empty() || thickness <= 0.0 {
        return mesh;
    }
    // the same raft whatever order the feet come in
    let mut feet = feet.to_vec();
    feet.sort_by(|a, b| a.x.total_cmp(&b.x).then(a.y.total_cmp(&b.y)));
    feet.dedup();
    for &f in &feet {
        let mut d = Vec2::X;
        let ring: Vec<(Vec2, Vec2)> = (0..PAD_SIDES)
            .map(|_| {
                let here = d;
                d = Vec2::new(d.x * C15 - d.y * S15, d.x * S15 + d.y * C15);
                (f, here)
            })
            .collect();
        push_chamfered(&mut mesh, &ring, pad, thickness);
    }
    let band = band.min(pad);
    for (i, j) in spanning_tree(&feet) {
        let (a, b) = (feet[i], feet[j]);
        let len = a.distance(b);
        if len < MIN_BAND_MM {
            continue;
        }
        // left normal; counter-clockwise: out along the right side, back
        // along the left
        let n = (b - a).perp() / len;
        push_chamfered(
            &mut mesh,
            &[(a, -n), (b, -n), (b, n), (a, n)],
            band,
            thickness,
        );
    }
    mesh
}

/// Appends a closed prism chamfered at 45° to `mesh`. `ring` is its outline
/// as (centre, unit direction) points, convex and counter-clockwise: at
/// z = `thickness` each point lies `radius` from its centre, at z = 0
/// `radius + thickness`.
fn push_chamfered(mesh: &mut TriMesh, ring: &[(Vec2, Vec2)], radius: f32, thickness: f32) {
    let m = ring.len() as u32;
    let lo = mesh.positions.len() as u32;
    let hi = lo + m;
    let bottom = radius + thickness;
    mesh.positions
        .extend(ring.iter().map(|(c, d)| (*c + *d * bottom).extend(0.0)));
    mesh.positions.extend(
        ring.iter()
            .map(|(c, d)| (*c + *d * radius).extend(thickness)),
    );
    for k in 1..m - 1 {
        mesh.tris.push([lo, lo + k + 1, lo + k]);
        mesh.tris.push([hi, hi + k, hi + k + 1]);
    }
    for k in 0..m {
        let k1 = (k + 1) % m;
        mesh.tris.push([lo + k, lo + k1, hi + k1]);
        mesh.tris.push([lo + k, hi + k1, hi + k]);
    }
}

/// Edges `(i, j)` of the Euclidean minimum spanning tree of `points`
/// (Prim's algorithm, O(n²)): the shortest set of segments that joins
/// them all. Squared distances in f64 and ties to the lower index, so the
/// tree is the same on every platform.
pub(crate) fn spanning_tree(points: &[Vec2]) -> Vec<(usize, usize)> {
    let n = points.len();
    if n < 2 {
        return Vec::new();
    }
    let d2 = |i: usize, j: usize| points[i].as_dvec2().distance_squared(points[j].as_dvec2());
    // per point: the squared distance to the tree and the tree point
    let mut best: Vec<(f64, usize)> = (0..n).map(|j| (d2(0, j), 0)).collect();
    let mut in_tree = vec![false; n];
    in_tree[0] = true;
    let mut edges = Vec::with_capacity(n - 1);
    for _ in 1..n {
        let Some(k) = (0..n)
            .filter(|&j| !in_tree[j])
            .min_by(|&a, &b| best[a].0.total_cmp(&best[b].0).then(a.cmp(&b)))
        else {
            break;
        };
        in_tree[k] = true;
        edges.push((best[k].1, k));
        for j in 0..n {
            if !in_tree[j] {
                let d = d2(k, j);
                if d < best[j].0 {
                    best[j] = (d, k);
                }
            }
        }
    }
    edges
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::support::tests::assert_closed_outward;
    use glam::Vec3;

    #[test]
    fn strut_is_closed_and_shaped() {
        let s = Strut {
            tip: Vec3::new(3.0, -2.0, 7.5),
            foot: Vec3::new(3.0, -2.0, 1.0),
            tip_d_mm: 0.35,
            shaft_d_mm: 1.0,
        };
        let m = strut_mesh(&s);
        assert_closed_outward(&m);
        let bb = m.aabb().unwrap();
        assert!((bb.max.z - 7.7).abs() < 1e-5 && (bb.min.z - 0.9).abs() < 1e-5);
        // Short strut: the cone shrinks with it.
        let short = Strut {
            tip: Vec3::new(0.0, 0.0, 1.5),
            foot: Vec3::new(0.0, 0.0, 1.0),
            ..s
        };
        assert_closed_outward(&strut_mesh(&short));
    }

    /// Feet along a U: two arms 30 mm apart, 8 feet each, and a bow of 11
    /// feet joining them.
    fn u_feet() -> Vec<Vec2> {
        let mut feet = Vec::new();
        for i in 0..8 {
            let y = 2.5 * i as f32;
            feet.push(Vec2::new(-15.0, y));
            feet.push(Vec2::new(15.0, y));
        }
        // the bow, every 15° from 15° to 165° on a 15 mm circle centred
        // at the arms' ends
        let rotate = |d: Vec2| Vec2::new(d.x * C15 - d.y * S15, d.x * S15 + d.y * C15);
        let mut d = rotate(Vec2::X);
        for _ in 0..11 {
            feet.push(Vec2::new(0.0, 17.5) + d * 15.0);
            d = rotate(d);
        }
        feet
    }

    #[test]
    fn raft_pads_and_bands_are_closed() {
        let sets: [&[Vec2]; 5] = [
            &[Vec2::new(1.0, 2.0)],
            &[Vec2::new(0.0, 0.0), Vec2::new(10.0, 3.0)],
            &[
                Vec2::new(0.0, 0.0),
                Vec2::new(10.0, 0.0),
                Vec2::new(4.0, 6.0),
            ],
            // a repeated foot and one 5 µm away: one pad each, no band
            &[Vec2::ZERO, Vec2::ZERO, Vec2::new(0.005, 0.0)],
            &u_feet(),
        ];
        // per pad 24 sides: 2 × 24 + 2 × 22 triangles; per band 12
        let tris = [92, 2 * 92 + 12, 3 * 92 + 2 * 12, 2 * 92, 27 * 92 + 26 * 12];
        for (feet, tris) in sets.into_iter().zip(tris) {
            let raft = raft_mesh(feet, 1.0, 2.125, 2.0);
            assert_closed_outward(&raft);
            assert_eq!(raft.face_count(), tris);
            let bb = raft.aabb().unwrap();
            assert!((bb.max.z - 1.0).abs() < 1e-6 && bb.min.z == 0.0);
        }
        assert!(raft_mesh(&[], 1.0, 2.5, 2.0).is_empty());
        assert!(raft_mesh(&[Vec2::ZERO], 0.0, 2.5, 2.0).is_empty());
        // the order of the feet does not matter
        let mut shuffled = u_feet();
        shuffled.reverse();
        assert_eq!(
            raft_mesh(&shuffled, 1.0, 2.125, 2.0),
            raft_mesh(&u_feet(), 1.0, 2.125, 2.0)
        );
    }

    #[test]
    fn the_raft_follows_the_u() {
        let (pad, thickness) = (2.125, 1.0);
        let raft = raft_mesh(&u_feet(), thickness, pad, 2.0);
        // every point of the raft is near a foot or a band: nothing lies
        // between the arms, below the bow
        let reach = pad + thickness + 1e-4;
        for p in &raft.positions {
            if p.y < 17.5 {
                assert!(p.x.abs() >= 15.0 - reach, "{p} fills the U");
            }
        }
        // the top of a pad is flat `pad` around its foot, the bottom
        // `thickness` wider: 45°
        let single = raft_mesh(&[Vec2::new(3.0, 4.0)], thickness, pad, 2.0);
        for p in &single.positions {
            let r = p.truncate().distance(Vec2::new(3.0, 4.0));
            let want = if p.z == 0.0 { pad + thickness } else { pad };
            assert!((r - want).abs() < 1e-4, "{p}");
        }
    }

    #[test]
    fn spanning_tree_joins_neighbours() {
        assert!(spanning_tree(&[]).is_empty());
        assert!(spanning_tree(&[Vec2::ONE]).is_empty());
        let line: Vec<Vec2> = [0.0, 3.0, 1.0, 2.0]
            .iter()
            .map(|&x| Vec2::new(x, 0.0))
            .collect();
        // 0–2 (1 mm), 2–3 (1 mm), 3–1 (1 mm)
        assert_eq!(spanning_tree(&line), vec![(0, 2), (2, 3), (3, 1)]);
        // a U: n − 1 edges, none across the 30 mm opening
        let u = u_feet();
        let edges = spanning_tree(&u);
        assert_eq!(edges.len(), u.len() - 1);
        let total: f32 = edges.iter().map(|&(i, j)| u[i].distance(u[j])).sum();
        // along the U: 2 × 7 steps of 2.5 mm on the arms, then 12 chords
        // of 15° on the 15 mm circle (2 · 15 · sin 7.5°) round the bow
        assert!((total - (35.0 + 12.0 * 3.915_787)).abs() < 0.01, "{total}");
        assert!(edges.iter().all(|&(i, j)| u[i].distance(u[j]) < 4.0));
    }
}
