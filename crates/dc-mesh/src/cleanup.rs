//! Import cleanup primitives shared by the scan import pipeline.

use std::collections::HashSet;

use glam::Vec3;

use crate::TriMesh;

/// Counts of the faces dropped by [`remove_bad_faces`].
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct BadFaceReport {
    /// Faces with a repeated vertex index (`a == b`, `b == c` or `a == c`).
    pub degenerate: usize,
    /// Faces whose sorted index triple equals an earlier kept face, so
    /// reversed duplicates count too.
    pub duplicate: usize,
}

impl BadFaceReport {
    /// Total number of removed faces.
    pub fn removed(&self) -> usize {
        self.degenerate + self.duplicate
    }
}

/// Removes faces that Blender's mesh validation drops on import: faces with a
/// repeated index, then faces with the same vertex set as an earlier face (the
/// first occurrence is kept, whatever its winding).
///
/// Collinear zero-area faces with three distinct indices stay, and so do
/// isolated vertices, so the vertex count never changes. `face_labels` is
/// filtered in step with the triangles. Runs in O(faces) with a hash set.
pub fn remove_bad_faces(mesh: &mut TriMesh) -> BadFaceReport {
    let mut report = BadFaceReport::default();
    let mut seen: HashSet<[u32; 3]> = HashSet::with_capacity(mesh.tris.len());
    let mut keep = Vec::with_capacity(mesh.tris.len());
    for &[a, b, c] in &mesh.tris {
        if a == b || b == c || a == c {
            report.degenerate += 1;
            keep.push(false);
            continue;
        }
        let mut key = [a, b, c];
        key.sort_unstable();
        let fresh = seen.insert(key);
        if !fresh {
            report.duplicate += 1;
        }
        keep.push(fresh);
    }
    if report.removed() == 0 {
        return report;
    }
    let mut it = keep.iter();
    mesh.tris.retain(|_| *it.next().unwrap_or(&true));
    if let Some(labels) = &mut mesh.face_labels
        && labels.len() == keep.len()
    {
        let mut it = keep.iter();
        labels.retain(|_| *it.next().unwrap_or(&true));
    }
    report
}

/// Rotates the mesh from a Y-up to a Z-up frame: `(x, y, z)` becomes
/// `(x, -z, y)` for positions and normals (if present). This is a proper
/// rotation (determinant +1, 90 degrees about X), so triangle winding and the
/// sign of the signed volume are unchanged.
pub fn rotate_y_up_to_z_up(mesh: &mut TriMesh) {
    let rot = |v: &mut Vec3| *v = Vec3::new(v.x, -v.z, v.y);
    mesh.positions.iter_mut().for_each(rot);
    if let Some(normals) = &mut mesh.normals {
        normals.iter_mut().for_each(rot);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_meshes::tetra;

    /// Signed volume via the divergence theorem (positive for outward winding).
    fn signed_volume(m: &TriMesh) -> f32 {
        m.tris
            .iter()
            .enumerate()
            .map(|(f, _)| {
                let [a, b, c] = m.triangle(f);
                a.dot(b.cross(c)) / 6.0
            })
            .sum()
    }

    /// Strip of `n` distinct triangles over `n + 2` vertices.
    fn strip(n: u32) -> TriMesh {
        let positions = (0..n + 2)
            .map(|i| Vec3::new(i as f32, (i % 2) as f32, 0.0))
            .collect();
        let tris = (0..n).map(|i| [i, i + 1, i + 2]).collect();
        TriMesh::new(positions, tris)
    }

    #[test]
    fn repeated_index_faces_are_removed() {
        // same shape as the measured fixture: 4788 faces, 50 with repeated indices
        let mut m = strip(4738);
        for i in 0..50u32 {
            let v = i + 3;
            let bad = match i % 3 {
                0 => [v, v, v + 1],
                1 => [v, v + 1, v + 1],
                _ => [v, v + 1, v],
            };
            m.tris.insert((i as usize * 90) % m.tris.len(), bad);
        }
        let nv = m.vertex_count();
        assert_eq!(m.face_count(), 4788);
        let report = remove_bad_faces(&mut m);
        assert_eq!(
            report,
            BadFaceReport {
                degenerate: 50,
                duplicate: 0
            }
        );
        assert_eq!(m.face_count(), 4738);
        assert_eq!(m.vertex_count(), nv);
        assert!(
            m.tris
                .iter()
                .all(|t| t[0] != t[1] && t[1] != t[2] && t[0] != t[2])
        );
    }

    #[test]
    fn duplicates_and_reversed_duplicates_are_removed() {
        // 5020 faces: 4720 distinct + 300 duplicates (half same winding, half reversed)
        let mut m = strip(4720);
        let base = m.tris.clone();
        for (i, t) in base.iter().take(300).enumerate() {
            m.tris
                .push(if i % 2 == 0 { *t } else { [t[2], t[1], t[0]] });
        }
        assert_eq!(m.face_count(), 5020);
        let report = remove_bad_faces(&mut m);
        assert_eq!(
            report,
            BadFaceReport {
                degenerate: 0,
                duplicate: 300
            }
        );
        assert_eq!(report.removed(), 300);
        assert_eq!(m.face_count(), 4720);
        assert_eq!(m.tris, base);
    }

    #[test]
    fn first_occurrence_wins_collinear_and_isolated_vertices_stay() {
        let mut m = TriMesh::new(
            vec![
                Vec3::new(0.0, 0.0, 0.0),
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(2.0, 0.0, 0.0), // collinear with 0 and 1
                Vec3::new(0.0, 1.0, 0.0),
                Vec3::new(9.0, 9.0, 9.0), // isolated
            ],
            vec![
                [0, 1, 2], // collinear, kept
                [2, 1, 0], // reversed duplicate of the collinear face, removed
                [0, 1, 3], // kept
                [3, 1, 0], // reversed duplicate, removed
                [1, 1, 3], // repeated index, removed
                [1, 3, 0], // rotation of the third face, removed
                [1, 2, 3], // kept
            ],
        );
        m.face_labels = Some(vec![10, 11, 12, 13, 14, 15, 16]);
        let report = remove_bad_faces(&mut m);
        assert_eq!(
            report,
            BadFaceReport {
                degenerate: 1,
                duplicate: 3
            }
        );
        assert_eq!(m.tris, vec![[0, 1, 2], [0, 1, 3], [1, 2, 3]]);
        assert_eq!(m.face_labels, Some(vec![10, 12, 16]));
        assert_eq!(m.vertex_count(), 5);
        m.check().unwrap();
    }

    #[test]
    fn clean_mesh_is_untouched() {
        let mut m = tetra();
        let before = m.clone();
        assert_eq!(remove_bad_faces(&mut m), BadFaceReport::default());
        assert_eq!(m, before);
    }

    #[test]
    fn remove_bad_faces_handles_a_large_mesh() {
        let mut m = strip(855_000);
        let base = m.tris.clone();
        m.tris.extend(base.iter().take(100_000).copied());
        let report = remove_bad_faces(&mut m);
        assert_eq!(report.duplicate, 100_000);
        assert_eq!(m.face_count(), 855_000);
    }

    #[test]
    fn y_up_to_z_up_maps_known_vertex_and_normals() {
        let mut m = TriMesh::new(vec![Vec3::new(-33.842, -2.484, -11.998)], Vec::new());
        m.normals = Some(vec![Vec3::new(0.0, 1.0, 0.0)]);
        rotate_y_up_to_z_up(&mut m);
        assert_eq!(m.positions[0], Vec3::new(-33.842, 11.998, -2.484));
        assert_eq!(m.normals.unwrap()[0], Vec3::new(0.0, 0.0, 1.0));
    }

    #[test]
    fn y_up_to_z_up_keeps_winding_and_volume_sign() {
        let mut m = tetra();
        let before = signed_volume(&m);
        let area = m.area();
        let tris = m.tris.clone();
        rotate_y_up_to_z_up(&mut m);
        assert_eq!(m.tris, tris);
        let after = signed_volume(&m);
        assert!(before.abs() > 0.1 && (before - after).abs() < 1e-6);
        assert!((m.area() - area).abs() < 1e-6);
        // inside-out mesh keeps its negative sign
        m.tris.iter_mut().for_each(|t| t.swap(1, 2));
        assert!(signed_volume(&m) < 0.0);
    }
}
