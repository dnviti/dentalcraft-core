//! Case, scans, world frame and the immutable case state.

use std::collections::BTreeMap;
use std::sync::Arc;

use glam::DVec3;
use serde::{Deserialize, Serialize};
use uuid::Uuid;

use dc_mesh::{TriMesh, Units};

use crate::{BlobRef, BlobStore, ModuleData, ModuleState, Pose};

/// What a scan shows.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ScanRole {
    /// Upper (maxillary) arch.
    Upper,
    /// Lower (mandibular) arch.
    Lower,
    /// Bite registration.
    Bite,
}

/// An imported scan.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Scan {
    /// Stable identifier.
    pub id: Uuid,
    /// Arch or bite.
    pub role: ScanRole,
    /// Original file name (no directory, for display and traceability).
    pub source_name: String,
    /// Unit of the source file (positions are converted to mm on import).
    pub source_units: Units,
    /// Rigid transform applied after the unit conversion, from file to case coordinates.
    pub import_xform: Pose,
    /// The mesh, in case coordinates (mm).
    pub mesh: BlobRef,
}

/// Dental world frame of a case.
///
/// Convention (same as the old `worldframe.py`): the occlusal plane is XY,
/// the midline runs along Y with the incisors towards −Y, and the lower arch
/// lies towards +Z.
#[derive(Debug, Clone, Copy, PartialEq, Default, Serialize, Deserialize)]
pub struct WorldFrame {
    /// Transform from case coordinates to the world frame.
    pub from_case: Pose,
}

impl WorldFrame {
    /// Normal of the occlusal plane.
    pub const OCCLUSAL_NORMAL: DVec3 = DVec3::Z;
    /// Direction of the midline (posterior to anterior is −Y).
    pub const MIDLINE: DVec3 = DVec3::Y;
    /// Direction towards the incisors.
    pub const ANTERIOR: DVec3 = DVec3::NEG_Y;
    /// Direction from the upper towards the lower arch.
    pub const TOWARDS_LOWER: DVec3 = DVec3::Z;
}

/// A patient case: scans and world frame. Patient identity is a
/// pseudonymised code only (no names in v1).
#[derive(Debug, Clone, PartialEq, Default, Serialize, Deserialize)]
pub struct Case {
    /// Stable identifier.
    pub id: Uuid,
    /// Pseudonymised patient code.
    pub patient_code: String,
    /// Imported scans.
    pub scans: Vec<Scan>,
    /// Dental world frame.
    pub world_frame: WorldFrame,
}

impl Case {
    /// New empty case with a random id.
    pub fn new(patient_code: impl Into<String>) -> Self {
        Self {
            id: Uuid::new_v4(),
            patient_code: patient_code.into(),
            ..Self::default()
        }
    }

    /// First scan with the given role.
    pub fn scan(&self, role: ScanRole) -> Option<&Scan> {
        self.scans.iter().find(|s| s.role == role)
    }

    /// Scan with the given id.
    pub fn scan_by_id(&self, id: Uuid) -> Option<&Scan> {
        self.scans.iter().find(|s| s.id == id)
    }
}

/// Complete state of an open case at one point in time.
///
/// Treated as immutable once shared (`Arc<CaseState>`): commands build a new
/// state from the previous one. Cloning is cheap for meshes (shared) and
/// deep for module data.
#[derive(Debug, Clone, Default)]
pub struct CaseState {
    /// The case.
    pub case: Case,
    /// Meshes referenced by the case and by the modules.
    pub blobs: BlobStore,
    /// Module data keyed by module id.
    pub modules: BTreeMap<String, ModuleState>,
}

impl CaseState {
    /// State holding `case` and nothing else.
    pub fn new(case: Case) -> Self {
        Self {
            case,
            ..Self::default()
        }
    }

    /// Mesh referenced by `r`.
    pub fn mesh(&self, r: &BlobRef) -> Option<&Arc<TriMesh>> {
        self.blobs.get(r)
    }

    /// State of module `id`.
    pub fn module(&self, id: &str) -> Option<&ModuleState> {
        self.modules.get(id)
    }

    /// Typed data of module `id` (`None` when absent or of another type).
    pub fn module_data<T: ModuleData>(&self, id: &str) -> Option<&T> {
        self.modules.get(id)?.downcast_ref()
    }

    /// Replaces the state of module `id`.
    pub fn set_module(&mut self, id: impl Into<String>, state: ModuleState) {
        self.modules.insert(id.into(), state);
    }

    /// Approximate memory of everything but the blobs (counted separately,
    /// since they are shared between versions).
    pub fn approx_bytes_without_blobs(&self) -> usize {
        std::mem::size_of::<Self>()
            + self.case.scans.len() * std::mem::size_of::<Scan>()
            + self
                .modules
                .values()
                .map(|m| m.data().approx_bytes())
                .sum::<usize>()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::Vec3;

    #[test]
    fn case_serde_round_trip() {
        let mut store = BlobStore::new();
        let mesh = store.insert(TriMesh::new(
            vec![Vec3::ZERO, Vec3::X, Vec3::Y],
            vec![[0, 1, 2]],
        ));
        let mut case = Case::new("PZ-0001");
        case.scans.push(Scan {
            id: Uuid::new_v4(),
            role: ScanRole::Upper,
            source_name: "upper.stl".into(),
            source_units: Units::Millimeters,
            import_xform: Pose::IDENTITY,
            mesh,
        });
        let json = serde_json::to_value(&case).unwrap();
        assert_eq!(json["scans"][0]["role"], "upper");
        assert_eq!(json["scans"][0]["source_units"], "mm");
        let back: Case = serde_json::from_value(json).unwrap();
        assert_eq!(back, case);
        assert!(back.scan(ScanRole::Upper).is_some());
        assert!(back.scan(ScanRole::Lower).is_none());
        let state = CaseState {
            case: back,
            blobs: store,
            ..CaseState::default()
        };
        assert!(state.mesh(&mesh).is_some());
        assert!(state.approx_bytes_without_blobs() > 0);
    }
}
