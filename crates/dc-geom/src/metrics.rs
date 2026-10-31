//! Comparison measures for golden tests and quality checks (PLAN section 5:
//! vertices Hausdorff < 0.01 mm, labels >= 99 %).
//!
//! Everything is computed in `f64`. The Hausdorff functions are brute force
//! (`O(n * m)`): fine for test fixtures; they move onto the BVH / k-d tree
//! once those land (WP13).

use dc_mesh::TriMesh;
use glam::{DVec3, Vec3};

/// Directed Hausdorff distance between point sets: the largest distance from
/// a point of `from` to its nearest point of `to`. `None` when either set is
/// empty.
pub fn hausdorff_points_directed(from: &[Vec3], to: &[Vec3]) -> Option<f64> {
    if from.is_empty() || to.is_empty() {
        return None;
    }
    let to: Vec<DVec3> = to.iter().map(|p| p.as_dvec3()).collect();
    let worst_sq = from
        .iter()
        .map(|p| {
            let p = p.as_dvec3();
            to.iter()
                .map(|q| p.distance_squared(*q))
                .fold(f64::INFINITY, f64::min)
        })
        .fold(0.0, f64::max);
    Some(worst_sq.sqrt())
}

/// Symmetric Hausdorff distance between point sets (the larger of the two
/// directed distances). `None` when either set is empty.
pub fn hausdorff_points(a: &[Vec3], b: &[Vec3]) -> Option<f64> {
    Some(hausdorff_points_directed(a, b)?.max(hausdorff_points_directed(b, a)?))
}

/// Directed Hausdorff distance from the vertices of `from` to the surface
/// (triangles) of `to`. `None` when `from` has no vertices or `to` no
/// triangles.
pub fn hausdorff_surface_directed(from: &TriMesh, to: &TriMesh) -> Option<f64> {
    if from.positions.is_empty() || to.tris.is_empty() {
        return None;
    }
    let tris: Vec<[DVec3; 3]> = (0..to.face_count())
        .map(|f| to.triangle(f).map(Vec3::as_dvec3))
        .collect();
    let worst_sq = from
        .positions
        .iter()
        .map(|p| {
            let p = p.as_dvec3();
            tris.iter()
                .map(|t| p.distance_squared(closest_point_on_triangle(p, t)))
                .fold(f64::INFINITY, f64::min)
        })
        .fold(0.0, f64::max);
    Some(worst_sq.sqrt())
}

/// Symmetric vertex-to-surface Hausdorff distance between two meshes.
/// `None` when either mesh has no triangles.
pub fn hausdorff_surface(a: &TriMesh, b: &TriMesh) -> Option<f64> {
    Some(hausdorff_surface_directed(a, b)?.max(hausdorff_surface_directed(b, a)?))
}

/// Fraction (0..=1) of positions where the two label arrays agree. `None`
/// when the lengths differ or both are empty.
pub fn label_agreement(a: &[u16], b: &[u16]) -> Option<f64> {
    label_agreement_where(a, b, |_| true)
}

/// Like [`label_agreement`], counting only the positions where the
/// reference label `b[i]` is not `ignore` (e.g. gingiva or "unlabelled").
/// `None` when the lengths differ or no position is counted.
pub fn label_agreement_ignoring(a: &[u16], b: &[u16], ignore: u16) -> Option<f64> {
    label_agreement_where(a, b, |r| r != ignore)
}

fn label_agreement_where(a: &[u16], b: &[u16], count: impl Fn(u16) -> bool) -> Option<f64> {
    if a.len() != b.len() {
        return None;
    }
    let (mut total, mut same) = (0usize, 0usize);
    for (&x, &r) in a.iter().zip(b) {
        if count(r) {
            total += 1;
            same += usize::from(x == r);
        }
    }
    (total > 0).then(|| same as f64 / total as f64)
}

/// Closest point to `p` on the triangle `t` (Ericson, Real-Time Collision
/// Detection, 5.1.5).
fn closest_point_on_triangle(p: DVec3, t: &[DVec3; 3]) -> DVec3 {
    let [a, b, c] = *t;
    let ab = b - a;
    let ac = c - a;
    let ap = p - a;
    let d1 = ab.dot(ap);
    let d2 = ac.dot(ap);
    if d1 <= 0.0 && d2 <= 0.0 {
        return a;
    }
    let bp = p - b;
    let d3 = ab.dot(bp);
    let d4 = ac.dot(bp);
    if d3 >= 0.0 && d4 <= d3 {
        return b;
    }
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        return a + ab * (d1 / (d1 - d3));
    }
    let cp = p - c;
    let d5 = ab.dot(cp);
    let d6 = ac.dot(cp);
    if d6 >= 0.0 && d5 <= d6 {
        return c;
    }
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        return a + ac * (d2 / (d2 - d6));
    }
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        return b + (c - b) * ((d4 - d3) / ((d4 - d3) + (d5 - d6)));
    }
    let denom = 1.0 / (va + vb + vc);
    a + ab * (vb * denom) + ac * (vc * denom)
}

#[cfg(test)]
mod tests {
    use super::*;

    const EPS: f64 = 1e-9;

    /// Unit square made of two triangles.
    fn quad() -> TriMesh {
        TriMesh::new(
            vec![
                Vec3::new(0.0, 0.0, 0.0),
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(1.0, 1.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
            ],
            vec![[0, 1, 2], [0, 2, 3]],
        )
    }

    #[test]
    fn point_hausdorff_is_symmetric_and_exact() {
        let a = [Vec3::ZERO, Vec3::X];
        let b = [Vec3::ZERO, Vec3::X, Vec3::new(0.0, 3.0, 0.0)];
        // every point of a is in b, but b has a point 3 away from a
        assert!(hausdorff_points_directed(&a, &b).unwrap() < EPS);
        assert!((hausdorff_points_directed(&b, &a).unwrap() - 3.0).abs() < EPS);
        assert!((hausdorff_points(&a, &b).unwrap() - 3.0).abs() < EPS);
        assert!((hausdorff_points(&b, &a).unwrap() - 3.0).abs() < EPS);
        assert!(hausdorff_points(&a, &a).unwrap() < EPS);
        assert_eq!(hausdorff_points(&a, &[]), None);
        assert_eq!(hausdorff_points(&[], &b), None);
    }

    #[test]
    fn surface_hausdorff_of_a_translated_mesh() {
        let a = quad();
        let mut b = a.clone();
        for p in &mut b.positions {
            p.z += 0.25;
        }
        assert!(hausdorff_surface(&a, &a).unwrap() < EPS);
        assert!((hausdorff_surface(&a, &b).unwrap() - 0.25).abs() < 1e-6);
        assert!((hausdorff_surface(&b, &a).unwrap() - 0.25).abs() < 1e-6);
        assert_eq!(hausdorff_surface(&a, &TriMesh::default()), None);
    }

    #[test]
    fn surface_hausdorff_sees_points_off_the_vertices() {
        // a vertex above the middle of a big triangle is close to the
        // surface but far from every vertex
        let big = TriMesh::new(
            vec![
                Vec3::new(-10.0, -10.0, 0.0),
                Vec3::new(10.0, -10.0, 0.0),
                Vec3::new(0.0, 10.0, 0.0),
            ],
            vec![[0, 1, 2]],
        );
        let probe = TriMesh::new(
            vec![
                Vec3::new(0.0, 0.0, 0.1),
                Vec3::new(0.1, 0.0, 0.1),
                Vec3::new(0.0, 0.1, 0.1),
            ],
            vec![[0, 1, 2]],
        );
        let d = hausdorff_surface_directed(&probe, &big).unwrap();
        assert!((d - 0.1).abs() < 1e-6, "{d}");
        assert!(hausdorff_points_directed(&probe.positions, &big.positions).unwrap() > 9.0);
    }

    #[test]
    fn closest_point_regions() {
        let t = [DVec3::ZERO, DVec3::X, DVec3::Y];
        let cp = |x, y, z| closest_point_on_triangle(DVec3::new(x, y, z), &t);
        assert_eq!(cp(-1.0, -1.0, 0.0), DVec3::ZERO); // vertex a
        assert_eq!(cp(2.0, -0.5, 0.0), DVec3::X); // vertex b
        assert_eq!(cp(-0.5, 2.0, 0.0), DVec3::Y); // vertex c
        assert!((cp(0.5, -1.0, 0.0) - DVec3::new(0.5, 0.0, 0.0)).length() < EPS); // edge ab
        assert!((cp(-1.0, 0.5, 0.0) - DVec3::new(0.0, 0.5, 0.0)).length() < EPS); // edge ac
        assert!((cp(1.0, 1.0, 0.0) - DVec3::new(0.5, 0.5, 0.0)).length() < EPS); // edge bc
        assert!((cp(0.2, 0.2, 5.0) - DVec3::new(0.2, 0.2, 0.0)).length() < EPS); // face
    }

    #[test]
    fn label_agreement_fractions() {
        assert_eq!(label_agreement(&[1, 2, 3, 4], &[1, 2, 3, 4]), Some(1.0));
        assert_eq!(label_agreement(&[1, 2, 0, 4], &[1, 2, 3, 4]), Some(0.75));
        assert_eq!(label_agreement(&[1], &[1, 2]), None);
        assert_eq!(label_agreement(&[], &[]), None);
        // gingiva (0) in the reference is not counted
        assert_eq!(
            label_agreement_ignoring(&[5, 11, 12, 0], &[0, 11, 13, 0], 0),
            Some(0.5)
        );
        assert_eq!(label_agreement_ignoring(&[1, 1], &[0, 0], 0), None);
    }
}
