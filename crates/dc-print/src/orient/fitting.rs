//! Fitting-side detection from the geometry alone (print plan §4), for
//! parts that come without face labels.
//!
//! A single ray along each face normal is not enough. On a cup-shaped
//! shell, the occlusal intaglio faces the opening, so its normal ray leaves
//! through the opening without a hit; that is a third or more of the
//! fitting surface. What separates the two sides of a thin shell is how
//! enclosed each side is: the fitting side looks into the cavity, which the
//! shell surrounds everywhere but at the opening, and the outer side looks
//! at open space.
//!
//! The method:
//! 1. **Enclosure.** Each face casts [`RAYS`] fixed rays over its outer
//!    hemisphere, at elevations of 10° and more. Its enclosure is the share
//!    of rays that hit the mesh within [`REACH_MM`].
//! 2. **Pairing.** A ray from the face into the material (−normal) meets
//!    the other side of the wall within [`WALL_MM`], on a face whose normal
//!    points the other way. That face is the partner.
//! 3. **Decision.** A paired face is fitting when it is more enclosed than
//!    its partner, i.e. it is the cavity side of the wall. A face without a
//!    partner (rim, thick features) is fitting when at least half its rays
//!    hit.
//! 4. **Clean-up.** Two passes of a majority vote over each face and its
//!    edge neighbours flip faces whose neighbours all disagree. The vote is
//!    by count, not area: next to the trim line, a few large rim triangles
//!    would otherwise outvote a row of small wall faces.
//!
//! Every face is computed independently: results are collected in face
//! order, so they do not depend on the thread count.

use dc_mesh::TriMesh;
use glam::Vec3;
use rayon::prelude::*;

use super::raycast::Bvh;

/// Enclosure rays per face.
pub(crate) const RAYS: usize = 24;
/// Reach of the enclosure rays, mm: about the size of a tooth cavity.
pub(crate) const REACH_MM: f32 = 15.0;
/// Lowest elevation of an enclosure ray above the face plane (sin 10°).
const MIN_ELEVATION_SIN: f32 = 0.173_648_18;
/// Longest pairing ray across the wall, mm.
pub(crate) const WALL_MM: f32 = 3.0;
/// Enclosure from which an unpaired face counts as fitting.
const UNPAIRED_FITTING: f32 = 0.5;
/// Start offset of the rays off the surface, mm.
const EPS_MM: f32 = 1e-3;

/// Per-face fitting flags (see the module docs).
pub(crate) fn detect(mesh: &TriMesh) -> Vec<bool> {
    let n = mesh.face_count();
    if n == 0 {
        return Vec::new();
    }
    let bvh = Bvh::new(mesh);
    let dirs = hemisphere();
    let faces: Vec<(Vec3, Vec3)> = (0..n)
        .map(|f| {
            let [a, b, c] = mesh.triangle(f);
            ((a + b + c) / 3.0, (b - a).cross(c - a).normalize_or_zero())
        })
        .collect();

    let enclosure: Vec<f32> = faces
        .par_iter()
        .map(|&(c, nrm)| {
            if nrm == Vec3::ZERO {
                return 0.0;
            }
            let (t, b) = nrm.any_orthonormal_pair();
            let origin = c + nrm * EPS_MM;
            let hits = dirs
                .iter()
                .filter(|d| bvh.occluded(origin, t * d.x + b * d.y + nrm * d.z, REACH_MM))
                .count();
            hits as f32 / RAYS as f32
        })
        .collect();

    let partner: Vec<Option<u32>> = faces
        .par_iter()
        .map(|&(c, nrm)| {
            if nrm == Vec3::ZERO {
                return None;
            }
            let hit = bvh.cast(c - nrm * EPS_MM, -nrm, WALL_MM)?;
            (faces[hit.face as usize].1.dot(nrm) < -0.5).then_some(hit.face)
        })
        .collect();

    let mut fitting: Vec<bool> = (0..n)
        .map(|f| match partner[f] {
            Some(g) if enclosure[f] != enclosure[g as usize] => {
                enclosure[f] > enclosure[g as usize]
            }
            _ => enclosure[f] >= UNPAIRED_FITTING,
        })
        .collect();

    let neighbours = edge_neighbours(mesh);
    for _ in 0..2 {
        fitting = (0..n)
            .map(|f| {
                let agree = neighbours[f]
                    .iter()
                    .filter(|&&g| fitting[g as usize] == fitting[f])
                    .count();
                let disagree = neighbours[f].len() - agree;
                // The face's own vote counts on its side; ties keep it.
                if disagree > agree + 1 {
                    !fitting[f]
                } else {
                    fitting[f]
                }
            })
            .collect();
    }
    fitting
}

/// The enclosure rays in the face frame (x, y tangent, z along the
/// normal): a Fibonacci spiral, uniform in solid angle above 10°.
fn hemisphere() -> [Vec3; RAYS] {
    const GOLDEN_ANGLE: f64 = 2.399_963_229_728_653;
    std::array::from_fn(|i| {
        let z = MIN_ELEVATION_SIN + (1.0 - MIN_ELEVATION_SIN) * (i as f32 + 0.5) / RAYS as f32;
        let r = (1.0 - z * z).max(0.0).sqrt();
        let (s, c) = super::sin_cos(GOLDEN_ANGLE * i as f64);
        Vec3::new(r * c as f32, r * s as f32, z)
    })
}

/// Faces sharing an edge with each face, sorted.
pub(crate) fn edge_neighbours(mesh: &TriMesh) -> Vec<Vec<u32>> {
    let mut edges: Vec<(u32, u32, u32)> = Vec::with_capacity(mesh.face_count() * 3);
    for (f, t) in mesh.tris.iter().enumerate() {
        for (a, b) in [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
            edges.push((a.min(b), a.max(b), f as u32));
        }
    }
    edges.sort_unstable();
    let mut out = vec![Vec::new(); mesh.face_count()];
    for run in edges.chunk_by(|x, y| x.0 == y.0 && x.1 == y.1) {
        for &(_, _, f) in run {
            for &(_, _, g) in run {
                if f != g {
                    out[f as usize].push(g);
                }
            }
        }
    }
    for list in &mut out {
        list.sort_unstable();
        list.dedup();
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::part::rotate_mesh;
    use crate::testutil::{SyntheticShell, default_shell};
    use glam::Quat;

    /// (share of the true fitting faces found, share of the other faces
    /// wrongly flagged).
    fn score(shell: &SyntheticShell, found: &[bool]) -> (f32, f32) {
        let truth = shell.fitting_truth();
        let (mut tp, mut pos, mut fp, mut neg) = (0usize, 0usize, 0usize, 0usize);
        for (t, f) in truth.iter().zip(found) {
            if *t {
                pos += 1;
                tp += usize::from(*f);
            } else {
                neg += 1;
                fp += usize::from(*f);
            }
        }
        (tp as f32 / pos as f32, fp as f32 / neg as f32)
    }

    #[test]
    fn finds_the_intaglio_of_the_synthetic_shell() {
        let shell = default_shell();
        let found = detect(&shell.mesh);
        let (recall, false_pos) = score(&shell, &found);
        assert!(recall > 0.9, "found {recall:.3} of the fitting faces");
        assert!(
            false_pos < 0.05,
            "{false_pos:.3} of the other faces flagged"
        );
    }

    #[test]
    fn does_not_depend_on_the_pose() {
        let mut shell = default_shell();
        rotate_mesh(
            &mut shell.mesh,
            Quat::from_euler(glam::EulerRot::XYZ, 2.1, -0.7, 0.4),
        );
        let found = detect(&shell.mesh);
        let (recall, false_pos) = score(&shell, &found);
        assert!(recall > 0.9, "found {recall:.3} of the fitting faces");
        assert!(
            false_pos < 0.05,
            "{false_pos:.3} of the other faces flagged"
        );
    }

    #[test]
    fn neighbours_of_a_closed_shell_are_three_per_face() {
        let shell = default_shell();
        let nb = edge_neighbours(&shell.mesh);
        assert!(nb.iter().all(|l| l.len() == 3));
    }
}
