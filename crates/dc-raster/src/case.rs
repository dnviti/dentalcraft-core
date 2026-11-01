//! Scene of a case: its scans in the world frame, coloured by tooth label
//! when labels are known.

use std::collections::BTreeMap;
use std::sync::Arc;

use dc_case::{BlobRef, CaseState, ScanRole};
use dc_scene::{Color, LabelPalette, MeshColoring};

use crate::Item;

/// Palette of tooth labels (FDI numbers; `0` = gingiva): the app's
/// [`LabelPalette::default`], so renders match the viewport.
pub fn label_palette() -> LabelPalette {
    LabelPalette::default()
}

/// The scans of `state` as render items, in the world frame (case world
/// frame × scan import transform, as the app viewport).
///
/// `labels` maps an arch to a mesh blob whose vertex labels hold the tooth
/// numbers (the Aligners module's `labels`). When that mesh has as many
/// vertices as the scan, the scan is coloured by those labels; when it has
/// a different vertex count, the label mesh is drawn instead of the scan.
/// Scans without labels use the tooth colour. `visible` limits the roles
/// drawn (all when `None`).
pub fn case_items<'a>(
    state: &'a CaseState,
    labels: &BTreeMap<ScanRole, BlobRef>,
    visible: Option<&[ScanRole]>,
) -> Vec<Item<'a>> {
    let world = state.case.world_frame.from_case.to_mat4().as_mat4();
    let mut out = Vec::new();
    for scan in &state.case.scans {
        if visible.is_some_and(|v| !v.contains(&scan.role)) {
            continue;
        }
        let Some(mesh) = state.mesh(&scan.mesh) else {
            continue;
        };
        let xform = world * scan.import_xform.to_mat4().as_mat4();
        let labelled = labels
            .get(&scan.role)
            .and_then(|r| state.mesh(r))
            .filter(|m| m.vertex_labels.is_some());
        let item = match labelled {
            Some(lm) if lm.vertex_count() == mesh.vertex_count() => {
                Item::new(mesh).with_coloring(MeshColoring::Labels {
                    labels: Arc::new(lm.vertex_labels.clone().unwrap_or_default()),
                    palette: label_palette(),
                })
            }
            Some(lm) => Item::new(lm).with_coloring(MeshColoring::VertexLabels(label_palette())),
            None => Item::new(mesh).with_coloring(MeshColoring::Uniform(Color::TOOTH)),
        };
        out.push(item.with_xform(xform));
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use dc_case::{Case, Pose, Scan};
    use dc_mesh::{TriMesh, Units};
    use glam::{DVec3, Vec3};

    fn tri(z: f32) -> TriMesh {
        TriMesh::new(
            vec![
                Vec3::new(0.0, 0.0, z),
                Vec3::new(1.0, 0.0, z),
                Vec3::new(0.0, 1.0, z),
            ],
            vec![[0, 1, 2]],
        )
    }

    fn state() -> CaseState {
        let mut s = CaseState::new(Case::new("T"));
        for (role, z) in [(ScanRole::Upper, 1.0), (ScanRole::Lower, -1.0)] {
            let blob = s.blobs.insert(tri(z));
            s.case.scans.push(Scan {
                id: uuid_like(z),
                role,
                role_source: Default::default(),
                source_name: format!("{role:?}.stl"),
                source_units: Units::Millimeters,
                import_xform: Pose::from_translation(DVec3::new(10.0, 0.0, 0.0)),
                mesh: blob,
            });
        }
        s
    }

    fn uuid_like(z: f32) -> uuid::Uuid {
        uuid::Uuid::from_u128(u128::from(z.to_bits()))
    }

    #[test]
    fn scans_become_items_in_the_world_frame() {
        let s = state();
        let items = case_items(&s, &BTreeMap::new(), None);
        assert_eq!(items.len(), 2);
        let p = items[0].xform.transform_point3(Vec3::ZERO);
        assert_eq!(p, Vec3::new(10.0, 0.0, 0.0));
        assert!(matches!(items[0].coloring, MeshColoring::Uniform(_)));
        let only = case_items(&s, &BTreeMap::new(), Some(&[ScanRole::Lower]));
        assert_eq!(only.len(), 1);
        assert_eq!(only[0].mesh.positions[0].z, -1.0);
    }

    #[test]
    fn both_bite_scans_are_drawn() {
        let mut s = state();
        for z in [3.0, 4.0] {
            let blob = s.blobs.insert(tri(z));
            s.case.scans.push(Scan {
                id: uuid_like(z),
                role: ScanRole::Bite,
                role_source: Default::default(),
                source_name: format!("bite{z}.stl"),
                source_units: Units::Millimeters,
                import_xform: Pose::default(),
                mesh: blob,
            });
        }
        let only = case_items(&s, &BTreeMap::new(), Some(&[ScanRole::Bite]));
        assert_eq!(only.len(), 2);
        assert_eq!(only[0].mesh.positions[0].z, 3.0);
        assert_eq!(only[1].mesh.positions[0].z, 4.0);
    }

    #[test]
    fn labels_colour_the_scan() {
        let mut s = state();
        let mut lm = tri(1.0);
        lm.vertex_labels = Some(vec![11, 11, 0]);
        let r = s.blobs.insert(lm);
        let mut other = tri(1.0);
        other.positions.push(Vec3::ONE);
        other.vertex_labels = Some(vec![31, 31, 31, 31]);
        let r2 = s.blobs.insert(other);
        let labels = BTreeMap::from([(ScanRole::Upper, r), (ScanRole::Lower, r2)]);
        let items = case_items(&s, &labels, None);
        match &items[0].coloring {
            MeshColoring::Labels { labels, .. } => assert_eq!(**labels, vec![11, 11, 0]),
            c => panic!("{c:?}"),
        }
        // different vertex count: the label mesh is drawn instead
        assert_eq!(items[1].mesh.vertex_count(), 4);
        assert!(matches!(items[1].coloring, MeshColoring::VertexLabels(_)));
    }
}
