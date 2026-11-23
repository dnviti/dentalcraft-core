//! Triangle mesh container for DentalCraft.
//!
//! [`TriMesh`] is the in-memory representation of every scan, tooth and
//! derived surface: positions in millimetres, triangles as vertex index
//! triples, optional per-vertex normals and optional per-vertex / per-face
//! `u16` labels (FDI numbers, `0` = gingiva / unlabeled).
//!
//! The crate also provides topology queries ([`EdgeAdjacency`], boundary
//! loops, connected components), the topology of a mesh as a solid with
//! vertices identified by exact position ([`solid_topology`], the shared
//! definition of a printable closed solid), a validation report ([`ValidationReport`]),
//! the canonical `DCM1` blob encoding ([`dcm`]) and the mesh file I/O
//! readers and writers ([`io`]: STL, OBJ, PLY, unit detection).
//! [`PointCloud`] is the scanner's point set (positions with optional
//! normals and colours), saved and loaded as PLY by [`io::write_ply`] /
//! [`io::read_ply`].

pub mod cleanup;
pub mod cloud;
pub mod dcm;
pub mod io;
pub mod solid;
mod topology;
mod validate;

use glam::Vec3;

pub use cleanup::{BadFaceReport, remove_bad_faces, rotate_y_up_to_z_up};
pub use cloud::PointCloud;
pub use io::{MeshFormat, ScanFile, Units, read_scan};
pub use solid::{SolidTopology, solid_topology};
pub use topology::{EdgeAdjacency, EdgeKey};
pub use validate::ValidationReport;

/// Errors produced by mesh operations.
#[derive(Debug, thiserror::Error)]
pub enum MeshError {
    /// A triangle references a vertex index that does not exist.
    #[error("triangle {face} references vertex {index}, but the mesh has {vertex_count} vertices")]
    IndexOutOfRange {
        /// Index of the offending triangle.
        face: usize,
        /// Offending vertex index.
        index: u32,
        /// Number of vertices in the mesh.
        vertex_count: usize,
    },
    /// An attribute array does not match the element count it belongs to.
    #[error("attribute `{name}` has {found} entries, expected {expected}")]
    AttributeLength {
        /// Attribute name (`normals`, `vertex_labels`, `face_labels`, `vertex_colors`).
        name: &'static str,
        /// Expected entry count.
        expected: usize,
        /// Actual entry count.
        found: usize,
    },
    /// The data is not a valid encoding of the requested format.
    #[error("invalid {format} data: {reason}")]
    Format {
        /// Format name (e.g. `DCM1`, `STL`).
        format: &'static str,
        /// Human readable reason.
        reason: String,
    },
    /// The file extension or content does not match any supported format.
    #[error("unsupported mesh format: {0}")]
    UnsupportedFormat(String),
    /// Underlying I/O error.
    #[error(transparent)]
    Io(#[from] std::io::Error),
    /// The operation is part of a later work package and not available yet.
    #[error("not implemented yet: {0}")]
    NotImplemented(&'static str),
}

/// Result alias for mesh operations.
pub type Result<T, E = MeshError> = std::result::Result<T, E>;

/// Axis-aligned bounding box.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Aabb {
    /// Component-wise minimum corner.
    pub min: Vec3,
    /// Component-wise maximum corner.
    pub max: Vec3,
}

impl Aabb {
    /// Smallest box containing all `points`; `None` when there are none.
    pub fn from_points<'a>(points: impl IntoIterator<Item = &'a Vec3>) -> Option<Self> {
        let mut it = points.into_iter();
        let first = *it.next()?;
        Some(it.fold(
            Self {
                min: first,
                max: first,
            },
            |b, p| Self {
                min: b.min.min(*p),
                max: b.max.max(*p),
            },
        ))
    }

    /// Edge lengths of the box.
    pub fn size(&self) -> Vec3 {
        self.max - self.min
    }

    /// Centre of the box.
    pub fn center(&self) -> Vec3 {
        (self.min + self.max) * 0.5
    }

    /// Length of the box diagonal.
    pub fn diagonal(&self) -> f32 {
        self.size().length()
    }
}

/// Indexed triangle mesh (millimetres, right-handed, counter-clockwise front faces).
#[derive(Debug, Clone, Default, PartialEq)]
pub struct TriMesh {
    /// Vertex positions.
    pub positions: Vec<Vec3>,
    /// Triangles as vertex index triples.
    pub tris: Vec<[u32; 3]>,
    /// Optional per-vertex normals (same length as `positions`).
    pub normals: Option<Vec<Vec3>>,
    /// Optional per-vertex labels (same length as `positions`).
    pub vertex_labels: Option<Vec<u16>>,
    /// Optional per-face labels (same length as `tris`).
    pub face_labels: Option<Vec<u16>>,
    /// Optional per-vertex colours, RGBA 8 bit (same length as `positions`).
    /// Read from OBJ / PLY files; not stored in the `DCM1` blob.
    pub vertex_colors: Option<Vec<[u8; 4]>>,
}

impl TriMesh {
    /// Mesh with positions and triangles only. Indices are not checked: use
    /// [`TriMesh::check`] or [`TriMesh::validate`] for untrusted data.
    pub fn new(positions: Vec<Vec3>, tris: Vec<[u32; 3]>) -> Self {
        Self {
            positions,
            tris,
            ..Self::default()
        }
    }

    /// Number of vertices.
    pub fn vertex_count(&self) -> usize {
        self.positions.len()
    }

    /// Number of triangles.
    pub fn face_count(&self) -> usize {
        self.tris.len()
    }

    /// `true` when the mesh has no triangles.
    pub fn is_empty(&self) -> bool {
        self.tris.is_empty()
    }

    /// Checks that indices are in range and attribute arrays have the right length.
    pub fn check(&self) -> Result<()> {
        let nv = self.positions.len();
        for (face, tri) in self.tris.iter().enumerate() {
            if let Some(&index) = tri.iter().find(|&&i| i as usize >= nv) {
                return Err(MeshError::IndexOutOfRange {
                    face,
                    index,
                    vertex_count: nv,
                });
            }
        }
        let attr = |name, expected, found: Option<usize>| match found {
            Some(found) if found != expected => Err(MeshError::AttributeLength {
                name,
                expected,
                found,
            }),
            _ => Ok(()),
        };
        attr("normals", nv, self.normals.as_ref().map(Vec::len))?;
        attr(
            "vertex_labels",
            nv,
            self.vertex_labels.as_ref().map(Vec::len),
        )?;
        attr(
            "face_labels",
            self.tris.len(),
            self.face_labels.as_ref().map(Vec::len),
        )?;
        attr(
            "vertex_colors",
            nv,
            self.vertex_colors.as_ref().map(Vec::len),
        )
    }

    /// Bounding box of the vertex positions; `None` for a mesh without vertices.
    pub fn aabb(&self) -> Option<Aabb> {
        Aabb::from_points(&self.positions)
    }

    /// Corner positions of triangle `face`.
    ///
    /// # Panics
    /// When `face` or one of its indices is out of range.
    pub fn triangle(&self, face: usize) -> [Vec3; 3] {
        self.tris[face].map(|i| self.positions[i as usize])
    }

    /// Non-normalised face normal (length = twice the triangle area).
    pub fn face_normal_raw(&self, face: usize) -> Vec3 {
        let [a, b, c] = self.triangle(face);
        (b - a).cross(c - a)
    }

    /// Unit face normal (zero for a degenerate triangle).
    pub fn face_normal(&self, face: usize) -> Vec3 {
        self.face_normal_raw(face).normalize_or_zero()
    }

    /// Triangle area.
    pub fn face_area(&self, face: usize) -> f32 {
        0.5 * self.face_normal_raw(face).length()
    }

    /// Total surface area.
    pub fn area(&self) -> f32 {
        (0..self.tris.len()).map(|f| self.face_area(f)).sum()
    }

    /// Area-weighted vertex normals (zero for isolated vertices).
    pub fn compute_vertex_normals(&self) -> Vec<Vec3> {
        let mut acc = vec![Vec3::ZERO; self.positions.len()];
        for f in 0..self.tris.len() {
            let n = self.face_normal_raw(f);
            for &i in &self.tris[f] {
                acc[i as usize] += n;
            }
        }
        acc.into_iter().map(Vec3::normalize_or_zero).collect()
    }

    /// Computes and stores vertex normals in [`TriMesh::normals`].
    pub fn update_normals(&mut self) {
        self.normals = Some(self.compute_vertex_normals());
    }

    /// Multiplies every position by `factor` (unit conversion).
    pub fn scale(&mut self, factor: f32) {
        for p in &mut self.positions {
            *p *= factor;
        }
    }

    /// Converts positions from `units` to millimetres (see [`io::detect_units`]).
    pub fn scale_to_mm(&mut self, units: Units) {
        if units != Units::Millimeters {
            self.scale(units.to_mm());
        }
    }

    /// Edge-to-face adjacency of the mesh.
    pub fn edge_adjacency(&self) -> EdgeAdjacency {
        EdgeAdjacency::build(self)
    }

    /// Closed boundary loops as ordered vertex index lists.
    pub fn boundary_loops(&self) -> Vec<Vec<u32>> {
        self.edge_adjacency().boundary_loops()
    }

    /// Connected components as lists of face indices (faces sharing a vertex
    /// belong to the same component). Sorted by decreasing size.
    pub fn connected_components(&self) -> Vec<Vec<u32>> {
        topology::connected_components(self)
    }

    /// Topological and geometric health report.
    pub fn validate(&self) -> ValidationReport {
        ValidationReport::of(self)
    }

    /// Rough memory footprint in bytes (used by undo memory budgets).
    pub fn approx_bytes(&self) -> usize {
        use std::mem::size_of;
        self.positions.len() * size_of::<Vec3>()
            + self.tris.len() * size_of::<[u32; 3]>()
            + self
                .normals
                .as_ref()
                .map_or(0, |n| n.len() * size_of::<Vec3>())
            + self.vertex_labels.as_ref().map_or(0, |l| l.len() * 2)
            + self.face_labels.as_ref().map_or(0, |l| l.len() * 2)
            + self.vertex_colors.as_ref().map_or(0, |c| c.len() * 4)
    }
}

#[cfg(test)]
pub(crate) mod test_meshes {
    use super::*;

    /// Closed tetrahedron with outward normals.
    pub fn tetra() -> TriMesh {
        TriMesh::new(
            vec![
                Vec3::new(0.0, 0.0, 0.0),
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
                Vec3::new(0.0, 0.0, 1.0),
            ],
            vec![[0, 2, 1], [0, 1, 3], [0, 3, 2], [1, 2, 3]],
        )
    }

    /// Unit square made of two triangles (open, one boundary loop).
    pub fn quad() -> TriMesh {
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
}

#[cfg(test)]
mod tests {
    use super::test_meshes::*;
    use super::*;

    #[test]
    fn aabb_and_area() {
        let m = quad();
        let b = m.aabb().unwrap();
        assert_eq!(b.min, Vec3::ZERO);
        assert_eq!(b.max, Vec3::new(1.0, 1.0, 0.0));
        assert_eq!(b.center(), Vec3::new(0.5, 0.5, 0.0));
        assert!((m.area() - 1.0).abs() < 1e-6);
        assert!(TriMesh::default().aabb().is_none());
    }

    #[test]
    fn vertex_normals_point_up_on_flat_quad() {
        let m = quad();
        for n in m.compute_vertex_normals() {
            assert!((n - Vec3::Z).length() < 1e-6);
        }
    }

    #[test]
    fn check_detects_bad_index_and_attribute_length() {
        let mut m = quad();
        assert!(m.check().is_ok());
        m.face_labels = Some(vec![1]);
        assert!(matches!(m.check(), Err(MeshError::AttributeLength { .. })));
        m.face_labels = None;
        m.tris.push([0, 1, 9]);
        assert!(matches!(
            m.check(),
            Err(MeshError::IndexOutOfRange { index: 9, .. })
        ));
    }

    #[test]
    fn scale_converts_units() {
        let mut m = quad();
        m.scale(10.0);
        assert_eq!(m.aabb().unwrap().max, Vec3::new(10.0, 10.0, 0.0));
    }
}
