//! Case, scans, world frame and the immutable case state.

use std::collections::{BTreeMap, HashSet};
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

/// Most scans a case holds with role [`ScanRole::Bite`] (buccal bite, one per
/// side). Each arch role holds one scan.
pub const MAX_BITE_SCANS: usize = 2;

/// Why a scan has its role.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RoleSource {
    /// Chosen by the user or the caller.
    #[default]
    Explicit,
    /// Read from the file name.
    FileName,
    /// Recognised from the geometry.
    Detected,
    /// Recognised from the geometry, from a cue the old plugin marks "to be
    /// checked": the arch is chosen and settled (nothing moves it, as for
    /// [`RoleSource::Detected`]) but the user should look at it. A
    /// `case.set-role` with the same role on both sides confirms it, which
    /// makes it [`RoleSource::Explicit`]. Files written before this variant
    /// never contain it.
    DetectedUnsure,
    /// A guess to be checked: the import may move the scan to the other arch.
    Provisional,
}

/// An imported scan.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Scan {
    /// Stable identifier.
    pub id: Uuid,
    /// Arch or bite.
    pub role: ScanRole,
    /// Why the scan has this role (projects written before this field count
    /// as [`RoleSource::Explicit`]).
    #[serde(default)]
    pub role_source: RoleSource,
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
/// the midline runs along Y with the incisors towards −Y. The lower arch has
/// its teeth up (+Z) and the upper arch stands above it with its teeth down,
/// so the lower arch lies below the upper one (towards −Z).
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
    /// Direction from the upper towards the lower arch (the lower arch lies
    /// below the upper one).
    pub const TOWARDS_LOWER: DVec3 = DVec3::NEG_Z;
}

/// Which jaws a case treats.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Jaws {
    /// Upper jaw only.
    Upper,
    /// Lower jaw only.
    Lower,
    /// Both jaws.
    #[default]
    Both,
}

/// A patient case: scans and world frame. Patient identity is a
/// pseudonymised code only (no names in v1).
#[derive(Debug, Clone, PartialEq, Default, Serialize, Deserialize)]
pub struct Case {
    /// Stable identifier.
    pub id: Uuid,
    /// Pseudonymised patient code.
    pub patient_code: String,
    /// Practice-side case identifier (free text; empty when not set).
    #[serde(default)]
    pub case_id: String,
    /// Free-form notes about the case.
    #[serde(default)]
    pub notes: String,
    /// Jaws being treated.
    #[serde(default)]
    pub jaws: Jaws,
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

    /// First scan with the given role. An arch role holds one scan; for the
    /// bite, up to [`MAX_BITE_SCANS`], see [`Case::scans_of`].
    pub fn scan(&self, role: ScanRole) -> Option<&Scan> {
        self.scans.iter().find(|s| s.role == role)
    }

    /// Every scan with the given role, in case order.
    pub fn scans_of(&self, role: ScanRole) -> impl Iterator<Item = &Scan> {
        self.scans.iter().filter(move |s| s.role == role)
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

    /// Blobs referenced by the case scans and by the module data.
    ///
    /// Anything in [`CaseState::blobs`] outside this set is garbage.
    pub fn referenced_blobs(&self) -> HashSet<BlobRef> {
        let mut refs: HashSet<BlobRef> = self.case.scans.iter().map(|s| s.mesh).collect();
        for module in self.modules.values() {
            refs.extend(module.data().referenced_blobs());
        }
        refs
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
            role_source: RoleSource::FileName,
            source_name: "upper.stl".into(),
            source_units: Units::Millimeters,
            import_xform: Pose::IDENTITY,
            mesh,
        });
        let json = serde_json::to_value(&case).unwrap();
        assert_eq!(json["scans"][0]["role"], "upper");
        assert_eq!(json["scans"][0]["role_source"], "file_name");
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

    #[test]
    fn old_case_json_gets_defaults() {
        let old = serde_json::json!({
            "id": Uuid::from_u128(1),
            "patient_code": "P-1",
            "scans": [],
            "world_frame": {"from_case": {"t": [0.0, 0.0, 0.0], "q": [0.0, 0.0, 0.0, 1.0]}},
        });
        let case: Case = serde_json::from_value(old).unwrap();
        assert_eq!(case.case_id, "");
        assert_eq!(case.notes, "");
        assert_eq!(case.jaws, Jaws::Both);
        let json = serde_json::to_value(&case).unwrap();
        assert_eq!(json["jaws"], "both");
    }

    #[test]
    fn old_scan_json_gets_an_explicit_role() {
        let old = serde_json::json!({
            "id": Uuid::from_u128(1),
            "role": "bite",
            "source_name": "bite.stl",
            "source_units": "mm",
            "import_xform": {"t": [0.0, 0.0, 0.0], "q": [0.0, 0.0, 0.0, 1.0]},
            "mesh": BlobRef::of_bytes(b"x"),
        });
        let scan: Scan = serde_json::from_value(old).unwrap();
        assert_eq!(scan.role_source, RoleSource::Explicit);
    }

    #[test]
    fn detected_unsure_has_its_own_name_and_old_names_still_read() {
        let json = |name: &str| serde_json::json!(name);
        assert_eq!(
            serde_json::to_value(RoleSource::DetectedUnsure).unwrap(),
            json("detected_unsure")
        );
        for (name, source) in [
            ("explicit", RoleSource::Explicit),
            ("file_name", RoleSource::FileName),
            ("detected", RoleSource::Detected),
            ("detected_unsure", RoleSource::DetectedUnsure),
            ("provisional", RoleSource::Provisional),
        ] {
            let back: RoleSource = serde_json::from_value(json(name)).unwrap();
            assert_eq!(back, source);
        }
    }

    #[test]
    fn scans_of_keeps_the_order_of_the_bite_scans() {
        let mesh = BlobRef::of_bytes(b"x");
        let scan = |id, role| Scan {
            id: Uuid::from_u128(id),
            role,
            role_source: RoleSource::Explicit,
            source_name: String::new(),
            source_units: Units::Millimeters,
            import_xform: Pose::IDENTITY,
            mesh,
        };
        let mut case = Case::new("P");
        case.scans = vec![
            scan(1, ScanRole::Lower),
            scan(2, ScanRole::Bite),
            scan(3, ScanRole::Bite),
        ];
        assert_eq!(case.scan(ScanRole::Bite).unwrap().id, Uuid::from_u128(2));
        let bites: Vec<_> = case.scans_of(ScanRole::Bite).map(|s| s.id).collect();
        assert_eq!(bites, [Uuid::from_u128(2), Uuid::from_u128(3)]);
    }

    #[test]
    fn world_frame_convention_matches_the_plugin() {
        // worldframe.py: lower arch teeth up (+Z), upper arch teeth down,
        // lower arch below the upper, incisors towards −Y, midline along Y.
        assert_eq!(WorldFrame::OCCLUSAL_NORMAL, DVec3::Z);
        assert_eq!(WorldFrame::TOWARDS_LOWER, DVec3::NEG_Z);
        assert_eq!(WorldFrame::ANTERIOR, DVec3::NEG_Y);
        assert_eq!(WorldFrame::MIDLINE, DVec3::Y);
        // the lower teeth point up, i.e. away from the upper arch
        assert_eq!(
            WorldFrame::TOWARDS_LOWER.dot(WorldFrame::OCCLUSAL_NORMAL),
            -1.0
        );
    }

    #[test]
    fn referenced_blobs_cover_scans_and_modules() {
        let mut state = CaseState::default();
        let a = state.blobs.insert(TriMesh::new(
            vec![Vec3::ZERO, Vec3::X, Vec3::Y],
            vec![[0, 1, 2]],
        ));
        let b = state.blobs.insert(TriMesh::new(
            vec![Vec3::ZERO, Vec3::X, Vec3::Z],
            vec![[0, 1, 2]],
        ));
        let c = state.blobs.insert(TriMesh::new(
            vec![Vec3::ZERO, Vec3::Y, Vec3::Z],
            vec![[0, 1, 2]],
        ));
        state.case.scans.push(Scan {
            id: Uuid::from_u128(1),
            role: ScanRole::Upper,
            role_source: RoleSource::Explicit,
            source_name: "u.stl".into(),
            source_units: Units::Millimeters,
            import_xform: Pose::IDENTITY,
            mesh: a,
        });
        // an opaque module keeps its blob references alive
        state.set_module(
            "future",
            ModuleState::new(crate::OpaqueModuleData {
                schema_version: 9,
                value: serde_json::json!({"deep": [{"mesh": b.to_hex()}, "not a hash"]}),
            }),
        );
        let refs = state.referenced_blobs();
        assert!(refs.contains(&a) && refs.contains(&b) && !refs.contains(&c));
    }
}
