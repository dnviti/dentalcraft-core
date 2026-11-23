//! Shared types of the print pipeline (print plan §4–§6).
//!
//! Flow: [`PrintPart`] (an aligner shell in model space) →
//! [`crate::orient::orient`] (rotation, fitting side away from the plate) →
//! [`crate::support::supports`] (support solids on the outer side only) →
//! [`crate::layout::nest`] (position on the plate) →
//! [`crate::slice::slice_plate`] (layers as runs) → GOO file.

use std::path::Path;

use dc_mesh::io::ReadOptions;
use dc_mesh::{MeshError, MeshFormat, TriMesh};
use glam::{Quat, Vec2, Vec3};
use serde::{Deserialize, Serialize};

/// One object to print, in model space (mm).
#[derive(Debug, Clone)]
pub struct PrintPart {
    /// Stable id, e.g. `upper-03`, `lower-12-passive`.
    pub id: String,
    /// Closed, outward-oriented triangle mesh.
    pub mesh: TriMesh,
    /// Per-face flag: `true` for faces of the fitting (intaglio) side, which
    /// touches the teeth and must never carry a support. `None`: computed by
    /// [`crate::orient::fitting_faces`].
    pub fitting: Option<Vec<bool>>,
}

/// Rigid orientation chosen for a part: `rotation` is applied about the
/// mesh origin, then the part is lifted so its lowest point sits at
/// `lift_mm` above the plate.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Orientation {
    /// Rotation from model space to plate space (plate up = +Z).
    pub rotation: Quat,
    /// Height of the part's lowest point above the plate, mm (support height).
    pub lift_mm: f32,
}

/// One support: a tapered strut from the plate (or the base raft) to a
/// contact point on the part's outer surface. Plate space, mm.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Strut {
    /// Contact point on the part surface.
    pub tip: Vec3,
    /// Foot on the raft top (z = raft thickness).
    pub foot: Vec3,
    /// Contact tip diameter, mm.
    pub tip_d_mm: f32,
    /// Shaft diameter, mm.
    pub shaft_d_mm: f32,
}

/// Supports of one oriented part, plate space before nesting (part
/// footprint centred on the origin in XY).
#[derive(Debug, Clone, Default)]
pub struct SupportSet {
    /// Struts.
    pub struts: Vec<Strut>,
    /// Raft under the struts, as a closed mesh (empty when none).
    pub raft: TriMesh,
    /// Support solids (struts meshed), closed meshes; sliced with the part.
    pub solids: Vec<TriMesh>,
}

/// A part ready for the plate: oriented, supported, and placed.
#[derive(Debug, Clone)]
pub struct PlacedPart {
    /// Part id.
    pub id: String,
    /// Oriented part mesh, plate space, already translated to its place.
    pub mesh: TriMesh,
    /// Support solids and raft, plate space, translated with the part.
    pub supports: Vec<TriMesh>,
    /// XY translation applied by nesting, mm.
    pub offset: Vec2,
}

/// Footprint of an oriented and supported part, used by nesting.
#[derive(Debug, Clone, PartialEq)]
pub struct Footprint {
    /// Part id.
    pub id: String,
    /// Convex hull of the part + supports projected on XY, counter-clockwise, mm.
    pub hull: Vec<Vec2>,
    /// Height above the plate, mm.
    pub height_mm: f32,
}

/// Rotate a mesh in place.
pub fn rotate_mesh(mesh: &mut TriMesh, rotation: Quat) {
    for p in &mut mesh.positions {
        *p = rotation * *p;
    }
    if let Some(n) = &mut mesh.normals {
        for v in n {
            *v = rotation * *v;
        }
    }
}

/// Reads a part (or a stage model) from a mesh file, vertices identified by
/// their exact position. An STL file has no connectivity: the default
/// tolerance weld of [`dc_mesh::io::read`] would merge the two ends of the
/// very short edges of a fine shell, folding their triangles, and the solid
/// check would see edges the file does not have.
///
/// # Errors
/// The file cannot be read or is not a valid mesh.
pub fn read_mesh(path: &Path) -> Result<TriMesh, MeshError> {
    let format = MeshFormat::from_path(path)?;
    let exact = ReadOptions {
        weld_tolerance: 0.0,
    };
    dc_mesh::io::read_with(path, format, &exact)
}

/// Translate a mesh in place.
pub fn translate_mesh(mesh: &mut TriMesh, by: Vec3) {
    for p in &mut mesh.positions {
        *p += by;
    }
}
