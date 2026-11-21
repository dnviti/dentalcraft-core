//! Cross-section of one triangle with a sampling plane (print plan §5).
//!
//! A vertex is *above* the plane z = z_c when z > z_c and *below* otherwise
//! (on the plane counts as below), the half-open rule of [`super::z`]. A
//! triangle with vertices on both sides has exactly two edges that change
//! side, and gives one segment between their crossing points.
//!
//! **Exact closure.** A crossing point is always interpolated from the edge's
//! below vertex towards its above vertex, so the two triangles sharing an
//! edge (welded or not, as long as the coordinates are equal) compute it bit
//! for bit the same, and the section contours close exactly. A vertex on the
//! plane is its own crossing point (t = 0), so a triangle that only touches
//! the plane gives a zero-length segment, which never crosses a row.
//!
//! **Direction.** The segment runs from the crossing of the edge going down
//! through the plane to the crossing of the edge coming up. For a triangle
//! listed counter-clockwise seen from outside, that direction is ẑ × n, so
//! the section of an outward-oriented solid runs counter-clockwise around the
//! material (seen from +z), and holes clockwise.

use glam::Vec3;

/// A directed section segment in the plate's XY plane, mm.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct Segment {
    /// Where the surface goes down through the plane.
    pub(crate) from: [f64; 2],
    /// Where the surface comes up through the plane.
    pub(crate) to: [f64; 2],
}

/// The section of triangle `tri` with the plane z = `zc`, if it crosses it.
pub(crate) fn cut(tri: [Vec3; 3], zc: f64) -> Option<Segment> {
    let p = tri.map(|v| [f64::from(v.x), f64::from(v.y), f64::from(v.z)]);
    let above = p.map(|v| v[2] > zc);
    if above[0] == above[1] && above[1] == above[2] {
        return None;
    }
    let mut from = [0.0; 2];
    let mut to = [0.0; 2];
    for k in 0..3 {
        let (u, v) = (k, (k + 1) % 3);
        if above[u] == above[v] {
            continue;
        }
        let (lo, hi) = if above[u] { (p[v], p[u]) } else { (p[u], p[v]) };
        // hi.z > zc ≥ lo.z, so the divisor is positive and t ∈ [0, 1].
        let t = (zc - lo[2]) / (hi[2] - lo[2]);
        let q = [lo[0] + t * (hi[0] - lo[0]), lo[1] + t * (hi[1] - lo[1])];
        if above[u] {
            from = q;
        } else {
            to = q;
        }
    }
    Some(Segment { from, to })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn direction_is_z_cross_normal() {
        // Normal −y (outward), so the segment must run +x.
        let tri = [
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
        ];
        let s = cut(tri, 0.5).unwrap();
        assert_eq!(s.from, [0.0, 0.0]);
        assert_eq!(s.to, [0.5, 0.0]);
        // Normal −x, so the segment must run −y.
        let tri = [
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(0.0, 1.0, 0.0),
        ];
        let s = cut(tri, 0.5).unwrap();
        assert!(s.to[1] < s.from[1]);
    }

    #[test]
    fn shared_edges_give_identical_points() {
        let a = Vec3::new(0.1, 0.2, -0.3);
        let b = Vec3::new(1.7, -0.9, 2.1);
        let c = Vec3::new(0.4, 1.3, 0.9);
        let d = Vec3::new(1.2, 1.1, -0.7);
        // Edge a–b, crossed going up in one triangle and going down in its
        // neighbour, which lists it the other way round.
        let s1 = cut([a, b, c], 0.37).unwrap();
        let s2 = cut([b, a, d], 0.37).unwrap();
        assert_eq!(s1.to, s2.from);
    }

    #[test]
    fn on_plane_and_flat_triangles() {
        // A vertex exactly on the plane counts as below: touching from
        // above gives a zero-length segment at that vertex.
        let tri = [
            Vec3::new(0.0, 0.0, 0.5),
            Vec3::new(1.0, 0.0, 1.0),
            Vec3::new(0.0, 1.0, 1.0),
        ];
        let s = cut(tri, 0.5).unwrap();
        assert_eq!(s.from, s.to);
        assert_eq!(s.from, [0.0, 0.0]);
        // Touching from below, or lying in the plane: no section.
        let tri = [
            Vec3::new(0.0, 0.0, 0.5),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
        ];
        assert_eq!(cut(tri, 0.5), None);
        let flat = [
            Vec3::new(0.0, 0.0, 0.5),
            Vec3::new(1.0, 0.0, 0.5),
            Vec3::new(0.0, 1.0, 0.5),
        ];
        assert_eq!(cut(flat, 0.5), None);
        // Degenerate (collinear, or a point): no panic, no NaN.
        let line = [
            Vec3::ZERO,
            Vec3::new(1.0, 1.0, 1.0),
            Vec3::new(2.0, 2.0, 2.0),
        ];
        let s = cut(line, 0.25).unwrap();
        assert!(s.from.iter().chain(&s.to).all(|v| v.is_finite()));
        assert_eq!(cut([Vec3::ONE; 3], 0.25), None);
    }
}
