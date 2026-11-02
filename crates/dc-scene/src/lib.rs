//! GPU-agnostic description of what the viewport draws.
//!
//! Modules return [`SceneItem`]s (and named [`Overlay`]s) built from the
//! case state; the app's viewport turns them into GPU resources. Meshes are
//! referenced by [`BlobRef`], so the renderer can cache uploads by content.

use std::sync::Arc;

use glam::Vec3;
use serde::{Deserialize, Serialize};

use dc_case::{BlobRef, Pose, ScanRole};
use dc_mesh::TriMesh;

/// Linear RGBA colour, components in `[0, 1]`.
#[derive(Debug, Clone, Copy, PartialEq, Default, Serialize, Deserialize)]
pub struct Color {
    /// Red.
    pub r: f32,
    /// Green.
    pub g: f32,
    /// Blue.
    pub b: f32,
    /// Alpha (1 = opaque).
    pub a: f32,
}

impl Color {
    /// Opaque white.
    pub const WHITE: Color = Color::rgb(1.0, 1.0, 1.0);
    /// Opaque black.
    pub const BLACK: Color = Color::rgb(0.0, 0.0, 0.0);
    /// Default tooth colour.
    pub const TOOTH: Color = Color::rgb(0.93, 0.91, 0.84);
    /// Default gingiva colour.
    pub const GINGIVA: Color = Color::rgb(0.85, 0.45, 0.47);

    /// Opaque colour.
    pub const fn rgb(r: f32, g: f32, b: f32) -> Self {
        Self { r, g, b, a: 1.0 }
    }

    /// Colour with alpha.
    pub const fn rgba(r: f32, g: f32, b: f32, a: f32) -> Self {
        Self { r, g, b, a }
    }

    /// Parses `#RRGGBB` or `#RRGGBBAA` (the `#` is optional).
    pub fn from_hex(s: &str) -> Option<Self> {
        let s = s.strip_prefix('#').unwrap_or(s);
        if !matches!(s.len(), 6 | 8) || !s.is_ascii() {
            return None;
        }
        let c = |i: usize| {
            u8::from_str_radix(&s[i..i + 2], 16)
                .ok()
                .map(|v| f32::from(v) / 255.0)
        };
        Some(Self {
            r: c(0)?,
            g: c(2)?,
            b: c(4)?,
            a: if s.len() == 8 { c(6)? } else { 1.0 },
        })
    }

    /// Same colour with another alpha.
    pub fn with_alpha(self, a: f32) -> Self {
        Self { a, ..self }
    }

    /// Components as an array.
    pub fn to_array(self) -> [f32; 4] {
        [self.r, self.g, self.b, self.a]
    }
}

/// Colours used to draw per-vertex or per-face labels.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct LabelPalette {
    /// Colour of label `0` (gingiva / unlabeled).
    pub unlabeled: Color,
    /// Colours cycled over labels `1..`.
    pub colors: Vec<Color>,
}

impl Default for LabelPalette {
    fn default() -> Self {
        const HEX: [&str; 12] = [
            "#4E79A7", "#F28E2B", "#59A14F", "#E15759", "#76B7B2", "#EDC948", "#B07AA1", "#FF9DA7",
            "#9C755F", "#BAB0AC", "#86BCB6", "#D37295",
        ];
        Self {
            unlabeled: Color::GINGIVA,
            colors: HEX.iter().filter_map(|h| Color::from_hex(h)).collect(),
        }
    }
}

impl LabelPalette {
    /// Colour of `label`.
    pub fn color(&self, label: u16) -> Color {
        if label == 0 || self.colors.is_empty() {
            self.unlabeled
        } else {
            self.colors[(label as usize - 1) % self.colors.len()]
        }
    }
}

/// How a mesh is coloured.
#[derive(Debug, Clone, PartialEq)]
pub enum MeshColoring {
    /// One colour for the whole mesh.
    Uniform(Color),
    /// One colour per vertex.
    PerVertex(Arc<Vec<Color>>),
    /// The mesh's own vertex labels drawn with a palette.
    VertexLabels(LabelPalette),
    /// Explicit per-vertex labels drawn with a palette.
    Labels {
        /// One label per vertex.
        labels: Arc<Vec<u16>>,
        /// Palette.
        palette: LabelPalette,
    },
}

/// A 3D text label anchored at a point.
#[derive(Debug, Clone, PartialEq)]
pub struct Label3d {
    /// Anchor position (world, mm).
    pub position: Vec3,
    /// Text (already translated, or a tooth number).
    pub text: String,
    /// Text colour.
    pub color: Color,
}

/// One drawable element.
#[derive(Debug, Clone, PartialEq)]
pub enum SceneItem {
    /// A triangle mesh from the blob store.
    Mesh {
        /// The mesh.
        blob: BlobRef,
        /// Colouring.
        coloring: MeshColoring,
        /// Model transform.
        xform: Pose,
        /// Can be picked with the mouse.
        pickable: bool,
    },
    /// Line segments.
    Lines {
        /// Segment end points.
        segments: Vec<[Vec3; 2]>,
        /// Colour.
        color: Color,
        /// Width in screen pixels.
        width: f32,
    },
    /// Points.
    Points {
        /// Positions.
        points: Vec<Vec3>,
        /// Colour.
        color: Color,
        /// Size in screen pixels.
        size: f32,
    },
    /// Text labels in 3D.
    Labels3d(Vec<Label3d>),
}

impl SceneItem {
    /// Uniformly coloured, pickable mesh without transform.
    pub fn mesh(blob: BlobRef, color: Color) -> Self {
        Self::Mesh {
            blob,
            coloring: MeshColoring::Uniform(color),
            xform: Pose::IDENTITY,
            pickable: true,
        }
    }
}

/// A [`SceneItem`] a module contributes to the case view, with the tags the
/// shell needs to filter, pick and compose it. Geometry is in **case
/// coordinates** (the consumer applies the case world frame, as for scans).
#[derive(Debug, Clone, PartialEq)]
pub struct SceneEntry {
    /// What to draw.
    pub item: SceneItem,
    /// Arch (`Upper` or `Lower`) the item belongs to. The arch toggles and
    /// the render's `visible` filter hide it with that arch, and a hidden
    /// item is not pickable. `None`: always shown.
    pub arch: Option<ScanRole>,
    /// FDI number of the tooth this mesh is (stable id for tinting and
    /// picking by tooth); `None` for anything that is not one tooth.
    pub tooth: Option<u8>,
    /// While this entry is shown, the case scan of `arch` is not drawn (the
    /// entry replaces it, e.g. the built teeth and gingiva).
    pub hides_scan: bool,
    /// Geometry drawn instead of the blob of a [`SceneItem::Mesh`] (same
    /// triangles, moved vertices: e.g. the gingiva deformed at a stage, which
    /// is not in the blob store). The blob still names the mesh (cache key,
    /// fallback).
    pub mesh: Option<Arc<TriMesh>>,
}

impl SceneEntry {
    /// Entry for `item` with no tag: always shown, hides nothing.
    pub fn new(item: SceneItem) -> Self {
        Self {
            item,
            arch: None,
            tooth: None,
            hides_scan: false,
            mesh: None,
        }
    }

    /// Drawn with `mesh` instead of the geometry of its blob.
    pub fn with_mesh(mut self, mesh: Arc<TriMesh>) -> Self {
        self.mesh = Some(mesh);
        self
    }

    /// The mesh to draw: the override, else the blob of a
    /// [`SceneItem::Mesh`] looked up with `lookup`; `None` for other items.
    pub fn resolve_mesh<'a>(
        &'a self,
        lookup: impl FnOnce(&BlobRef) -> Option<&'a Arc<TriMesh>>,
    ) -> Option<&'a Arc<TriMesh>> {
        match &self.item {
            SceneItem::Mesh { blob, .. } => self.mesh.as_ref().or_else(|| lookup(blob)),
            _ => None,
        }
    }

    /// Belonging to `arch`.
    pub fn with_arch(mut self, arch: ScanRole) -> Self {
        self.arch = Some(arch);
        self
    }

    /// Being the tooth `fdi`.
    pub fn with_tooth(mut self, fdi: u8) -> Self {
        self.tooth = Some(fdi);
        self
    }

    /// Replacing the scan of its arch.
    pub fn hiding_scan(mut self) -> Self {
        self.hides_scan = true;
        self
    }

    /// `true` when the entry is drawn with the arches in `visible` (all when
    /// `None`; an entry with no arch is always drawn).
    pub fn shown_with(&self, visible: Option<&[ScanRole]>) -> bool {
        match (self.arch, visible) {
            (Some(a), Some(v)) => v.contains(&a),
            _ => true,
        }
    }
}

/// A named group of items drawn on top of the case (measures, seeds,
/// collision maps...), which the user can show or hide.
#[derive(Debug, Clone, PartialEq)]
pub struct Overlay {
    /// Stable id (also used as i18n key for its name).
    pub id: String,
    /// Shown.
    pub visible: bool,
    /// Items.
    pub items: Vec<SceneItem>,
}

impl Overlay {
    /// Visible overlay with no items.
    pub fn new(id: impl Into<String>) -> Self {
        Self {
            id: id.into(),
            visible: true,
            items: Vec::new(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hex_colors() {
        assert_eq!(Color::from_hex("#ff0000"), Some(Color::rgb(1.0, 0.0, 0.0)));
        assert_eq!(Color::from_hex("00000080").unwrap().a, 128.0 / 255.0);
        assert!(Color::from_hex("#fff").is_none());
        assert!(Color::from_hex("#gg0000").is_none());
    }

    #[test]
    fn palette_cycles_and_keeps_zero() {
        let p = LabelPalette::default();
        assert_eq!(p.colors.len(), 12);
        assert_eq!(p.color(0), Color::GINGIVA);
        assert_eq!(p.color(1), p.color(13));
        assert_ne!(p.color(1), p.color(2));
    }

    #[test]
    fn entry_filters_by_arch() {
        let item = SceneItem::mesh(BlobRef::of_bytes(b"m"), Color::TOOTH);
        let e = SceneEntry::new(item.clone())
            .with_arch(ScanRole::Lower)
            .with_tooth(31)
            .hiding_scan();
        assert!(e.hides_scan && e.tooth == Some(31));
        assert!(e.shown_with(None));
        assert!(e.shown_with(Some(&[ScanRole::Lower])));
        assert!(!e.shown_with(Some(&[ScanRole::Upper])));
        assert!(SceneEntry::new(item).shown_with(Some(&[])));
    }

    #[test]
    fn overlay_builder() {
        let mut o = Overlay::new("seeds");
        o.items
            .push(SceneItem::mesh(BlobRef::of_bytes(b"m"), Color::TOOTH));
        o.items.push(SceneItem::Points {
            points: vec![Vec3::ZERO],
            color: Color::BLACK,
            size: 4.0,
        });
        assert!(o.visible);
        assert_eq!(o.items.len(), 2);
    }
}
