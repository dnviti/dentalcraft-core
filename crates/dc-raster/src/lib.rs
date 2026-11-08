//! CPU rasteriser for headless renders (port plan B2.1).
//!
//! Draws triangle meshes into an RGBA image without a GPU and encodes it as
//! PNG, for `dentalcraft-cli render`, the MCP `render` tool and the
//! `screenshot` of the app bridge.
//!
//! - Orthographic camera on one of the standard dental [`View`]s, framed on
//!   the bounding box of the scene ([`Camera::frame`]); the axes follow the
//!   app viewport (`dc-viewport`): front = camera on −Y looking at +Y, up =
//!   +Z, patient's right on −X.
//! - Z-buffer, Lambert shading with a headlight (two-sided, so open scans
//!   are lit from both sides) plus ambient, optional hidden-line wireframe,
//!   optional supersampling.
//! - Colours from [`dc_scene::MeshColoring`]: uniform, per vertex, or
//!   labels drawn with the app's [`LabelPalette`] (FDI tooth numbers).
//!
//! The output is deterministic: the same input gives the same pixels (no
//! threads, fixed evaluation order).
//!
//! ```ignore
//! let items = dc_raster::case_items(&state, &labels, None);
//! let img = dc_raster::render(&items, &RenderOptions::new(1024, 768, View::OcclusalUpper))?;
//! std::fs::write("occlusal.png", img.to_png()?)?;
//! ```

mod camera;
mod case;
mod request;

use dc_mesh::{Aabb, TriMesh};
use glam::{Mat4, Vec3};

pub use camera::{Camera, View};
pub use case::{case_items, case_items_with, label_palette};
pub use dc_scene::{Color, LabelPalette, MeshColoring};
pub use request::{DEFAULT_HEIGHT, DEFAULT_SUPERSAMPLE, DEFAULT_WIDTH, RenderRequest, parse_size};

/// Largest image side accepted, in pixels (after supersampling).
pub const MAX_SIDE: u32 = 16_384;

/// Errors of the rasteriser.
#[derive(Debug, thiserror::Error)]
pub enum RasterError {
    /// Width or height is zero or too large.
    #[error("invalid image size {width}x{height} (each side 1..={max})", max = MAX_SIDE)]
    InvalidSize {
        /// Requested width.
        width: u32,
        /// Requested height.
        height: u32,
    },
    /// PNG encoding failed.
    #[error("PNG encoding failed: {0}")]
    Png(#[from] png::EncodingError),
    /// Writing the file failed.
    #[error(transparent)]
    Io(#[from] std::io::Error),
}

/// A mesh to draw.
#[derive(Debug, Clone)]
pub struct Item<'a> {
    /// The mesh (millimetres).
    pub mesh: &'a TriMesh,
    /// Model transform (rigid, or with uniform scale).
    pub xform: Mat4,
    /// How it is coloured.
    pub coloring: MeshColoring,
}

impl<'a> Item<'a> {
    /// `mesh` with the identity transform and the tooth colour.
    pub fn new(mesh: &'a TriMesh) -> Self {
        Self {
            mesh,
            xform: Mat4::IDENTITY,
            coloring: MeshColoring::Uniform(Color::TOOTH),
        }
    }

    /// With this model transform.
    pub fn with_xform(mut self, xform: Mat4) -> Self {
        self.xform = xform;
        self
    }

    /// With this colouring.
    pub fn with_coloring(mut self, coloring: MeshColoring) -> Self {
        self.coloring = coloring;
        self
    }
}

/// Render settings.
#[derive(Debug, Clone, PartialEq)]
pub struct RenderOptions {
    /// Image width in pixels.
    pub width: u32,
    /// Image height in pixels.
    pub height: u32,
    /// Camera direction.
    pub view: View,
    /// Background colour (alpha 0 for a transparent background).
    pub background: Color,
    /// Colour of the triangle edges; no wireframe when `None`.
    pub wireframe: Option<Color>,
    /// Empty border around the scene, as a fraction of each side.
    pub margin: f32,
    /// Ambient light (0..1); the rest is Lambert from the headlight.
    pub ambient: f32,
    /// Supersampling factor per side (1 = none).
    pub supersample: u32,
}

impl RenderOptions {
    /// Default background (dark grey, as the app viewport).
    pub const BACKGROUND: Color = Color::rgb(0.11, 0.12, 0.14);
    /// Default wireframe colour.
    pub const WIRE: Color = Color::rgb(0.15, 0.15, 0.18);

    /// `width` × `height` image of `view`, default background, no
    /// wireframe, 5% margin, no supersampling.
    pub fn new(width: u32, height: u32, view: View) -> Self {
        Self {
            width,
            height,
            view,
            background: Self::BACKGROUND,
            wireframe: None,
            margin: 0.05,
            ambient: 0.25,
            supersample: 1,
        }
    }
}

/// An RGBA image, 8 bits per channel, rows top to bottom.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Image {
    /// Width in pixels.
    pub width: u32,
    /// Height in pixels.
    pub height: u32,
    /// `width * height * 4` bytes.
    pub rgba: Vec<u8>,
}

impl Image {
    /// Image filled with `color`.
    pub fn filled(width: u32, height: u32, color: [u8; 4]) -> Self {
        let n = width as usize * height as usize;
        Self {
            width,
            height,
            rgba: color.repeat(n),
        }
    }

    /// Pixel at (`x`, `y`).
    pub fn pixel(&self, x: u32, y: u32) -> [u8; 4] {
        let i = (y as usize * self.width as usize + x as usize) * 4;
        [
            self.rgba[i],
            self.rgba[i + 1],
            self.rgba[i + 2],
            self.rgba[i + 3],
        ]
    }

    /// PNG encoding of the image.
    pub fn to_png(&self) -> Result<Vec<u8>, RasterError> {
        let mut out = Vec::new();
        {
            let mut enc = png::Encoder::new(&mut out, self.width, self.height);
            enc.set_color(png::ColorType::Rgba);
            enc.set_depth(png::BitDepth::Eight);
            let mut w = enc.write_header()?;
            w.write_image_data(&self.rgba)?;
            w.finish()?;
        }
        Ok(out)
    }

    /// Writes the image as PNG to `path`.
    pub fn write_png(&self, path: impl AsRef<std::path::Path>) -> Result<(), RasterError> {
        std::fs::write(path, self.to_png()?)?;
        Ok(())
    }
}

/// Bounding box of the items after their transforms; `None` when they have
/// no vertex.
pub fn scene_bounds(items: &[Item<'_>]) -> Option<Aabb> {
    let pts: Vec<Vec3> = items
        .iter()
        .flat_map(|it| {
            it.mesh
                .positions
                .iter()
                .map(|p| it.xform.transform_point3(*p))
        })
        .collect();
    Aabb::from_points(&pts)
}

fn to_u8(v: f32) -> u8 {
    (v.clamp(0.0, 1.0) * 255.0 + 0.5) as u8
}

fn color_u8(c: Color) -> [u8; 4] {
    [to_u8(c.r), to_u8(c.g), to_u8(c.b), to_u8(c.a)]
}

/// Renders `items` with the camera framed on their bounding box.
pub fn render(items: &[Item<'_>], opts: &RenderOptions) -> Result<Image, RasterError> {
    let ss = opts.supersample.max(1);
    let (w, h) = (opts.width, opts.height);
    let valid = |s: u32| s >= 1 && s.checked_mul(ss).is_some_and(|v| v <= MAX_SIDE);
    if !valid(w) || !valid(h) {
        return Err(RasterError::InvalidSize {
            width: w,
            height: h,
        });
    }
    let Some(bounds) = scene_bounds(items) else {
        return Ok(Image::filled(w, h, color_u8(opts.background)));
    };
    let camera = Camera::frame(&bounds, opts.view, w * ss, h * ss, opts.margin);
    let big = render_with_camera(items, &camera, opts, ss as f32);
    Ok(if ss == 1 { big } else { downsample(&big, ss) })
}

/// Renders `items` with `camera` at the camera's size (no supersampling:
/// `opts.width`, `opts.height` and `opts.view` are ignored).
pub fn render_camera(items: &[Item<'_>], camera: &Camera, opts: &RenderOptions) -> Image {
    render_with_camera(items, camera, opts, 1.0)
}

fn downsample(img: &Image, ss: u32) -> Image {
    let (w, h) = (img.width / ss, img.height / ss);
    let n = ss * ss;
    let mut rgba = Vec::with_capacity(w as usize * h as usize * 4);
    for y in 0..h {
        for x in 0..w {
            let mut acc = [0u32; 4];
            for dy in 0..ss {
                for dx in 0..ss {
                    let p = img.pixel(x * ss + dx, y * ss + dy);
                    for (a, v) in acc.iter_mut().zip(p) {
                        *a += u32::from(v);
                    }
                }
            }
            rgba.extend(acc.map(|a| ((a + n / 2) / n) as u8));
        }
    }
    Image {
        width: w,
        height: h,
        rgba,
    }
}

/// Per-vertex colour source of an item.
enum VertexColors {
    Uniform([f32; 3]),
    /// Interpolated across the triangle.
    Smooth(Vec<[f32; 3]>),
    /// Taken from the nearest vertex (labels stay crisp).
    Nearest(Vec<[f32; 3]>),
}

fn rgb(c: Color) -> [f32; 3] {
    [c.r, c.g, c.b]
}

fn vertex_colors(item: &Item<'_>) -> VertexColors {
    let nv = item.mesh.vertex_count();
    let labels = |labels: &[u16], palette: &LabelPalette| {
        VertexColors::Nearest(labels.iter().map(|&l| rgb(palette.color(l))).collect())
    };
    match &item.coloring {
        MeshColoring::Uniform(c) => VertexColors::Uniform(rgb(*c)),
        MeshColoring::PerVertex(cs) if cs.len() == nv => {
            VertexColors::Smooth(cs.iter().map(|c| rgb(*c)).collect())
        }
        MeshColoring::VertexLabels(p) => match &item.mesh.vertex_labels {
            Some(l) if l.len() == nv => labels(l, p),
            _ => VertexColors::Uniform(rgb(p.unlabeled)),
        },
        MeshColoring::Labels { labels: l, palette } if l.len() == nv => labels(l, palette),
        MeshColoring::Labels { palette, .. } => VertexColors::Uniform(rgb(palette.unlabeled)),
        MeshColoring::PerVertex(_) => VertexColors::Uniform(rgb(Color::TOOTH)),
    }
}

/// Twice the signed area of (a, b, p): positive when p is left of a→b.
fn edge(a: Vec3, b: Vec3, p: (f32, f32)) -> f32 {
    (b.x - a.x) * (p.1 - a.y) - (b.y - a.y) * (p.0 - a.x)
}

fn render_with_camera(
    items: &[Item<'_>],
    camera: &Camera,
    opts: &RenderOptions,
    px_scale: f32,
) -> Image {
    let (w, h) = (camera.width, camera.height);
    let mut img = Image::filled(w, h, color_u8(opts.background));
    let mut depth = vec![f32::INFINITY; w as usize * h as usize];
    let light = -camera.dir;
    let ambient = opts.ambient.clamp(0.0, 1.0);
    let wire = opts.wireframe.map(rgb);
    let wire_px = 0.75 * px_scale;
    for item in items {
        let mesh = item.mesh;
        if mesh.tris.is_empty() {
            continue;
        }
        let screen: Vec<Vec3> = mesh
            .positions
            .iter()
            .map(|p| camera.project(item.xform.transform_point3(*p)))
            .collect();
        let normals: Vec<Vec3> = match &mesh.normals {
            Some(n) if n.len() == mesh.vertex_count() => n.clone(),
            _ => mesh.compute_vertex_normals(),
        };
        let shade: Vec<f32> = normals
            .iter()
            .map(|n| {
                let n = item.xform.transform_vector3(*n).normalize_or_zero();
                ambient + (1.0 - ambient) * n.dot(light).abs()
            })
            .collect();
        let colors = vertex_colors(item);
        for tri in &mesh.tris {
            let [i0, i1, i2] = tri.map(|i| i as usize);
            if i0.max(i1).max(i2) >= screen.len() {
                continue;
            }
            let (s0, s1, s2) = (screen[i0], screen[i1], screen[i2]);
            let area = edge(s0, s1, (s2.x, s2.y));
            if !area.is_finite() || area.abs() < 1e-12 {
                continue;
            }
            let min_x = s0.x.min(s1.x).min(s2.x).floor().max(0.0);
            let max_x = s0.x.max(s1.x).max(s2.x).ceil().min(w as f32 - 1.0);
            let min_y = s0.y.min(s1.y).min(s2.y).floor().max(0.0);
            let max_y = s0.y.max(s1.y).max(s2.y).ceil().min(h as f32 - 1.0);
            if min_x > max_x || min_y > max_y {
                continue;
            }
            // pixel distance to the edge opposite vertex k = b_k * |area| / len_k
            let len = [
                (s2 - s1).truncate().length(),
                (s0 - s2).truncate().length(),
                (s1 - s0).truncate().length(),
            ];
            let tri_shade = [shade[i0], shade[i1], shade[i2]];
            for y in min_y as u32..=max_y as u32 {
                for x in min_x as u32..=max_x as u32 {
                    let p = (x as f32 + 0.5, y as f32 + 0.5);
                    let b0 = edge(s1, s2, p) / area;
                    let b1 = edge(s2, s0, p) / area;
                    let b2 = edge(s0, s1, p) / area;
                    if b0 < 0.0 || b1 < 0.0 || b2 < 0.0 {
                        continue;
                    }
                    let z = b0 * s0.z + b1 * s1.z + b2 * s2.z;
                    let di = y as usize * w as usize + x as usize;
                    if z >= depth[di] {
                        continue;
                    }
                    depth[di] = z;
                    let b = [b0, b1, b2];
                    let on_edge = wire.is_some()
                        && (0..3).any(|k| len[k] > 0.0 && b[k] * area.abs() / len[k] < wire_px);
                    let c = match (on_edge, &colors) {
                        (true, _) => wire.unwrap_or_default(),
                        (false, VertexColors::Uniform(c)) => *c,
                        (false, VertexColors::Smooth(cs)) => {
                            let (c0, c1, c2) = (cs[i0], cs[i1], cs[i2]);
                            [0, 1, 2].map(|k| b0 * c0[k] + b1 * c1[k] + b2 * c2[k])
                        }
                        (false, VertexColors::Nearest(cs)) => {
                            let k = if b0 >= b1 && b0 >= b2 {
                                i0
                            } else if b1 >= b2 {
                                i1
                            } else {
                                i2
                            };
                            cs[k]
                        }
                    };
                    let s = if on_edge {
                        1.0
                    } else {
                        b0 * tri_shade[0] + b1 * tri_shade[1] + b2 * tri_shade[2]
                    };
                    let o = di * 4;
                    img.rgba[o] = to_u8(c[0] * s);
                    img.rgba[o + 1] = to_u8(c[1] * s);
                    img.rgba[o + 2] = to_u8(c[2] * s);
                    img.rgba[o + 3] = 255;
                }
            }
        }
    }
    img
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A 2×2×2 mm cube centred on the origin, with labels 11 and 21 on its
    /// halves.
    fn cube() -> TriMesh {
        let p = |x: f32, y: f32, z: f32| Vec3::new(x, y, z);
        let positions = vec![
            p(-1.0, -1.0, -1.0),
            p(1.0, -1.0, -1.0),
            p(1.0, 1.0, -1.0),
            p(-1.0, 1.0, -1.0),
            p(-1.0, -1.0, 1.0),
            p(1.0, -1.0, 1.0),
            p(1.0, 1.0, 1.0),
            p(-1.0, 1.0, 1.0),
        ];
        let tris = vec![
            [0, 2, 1],
            [0, 3, 2],
            [4, 5, 6],
            [4, 6, 7],
            [0, 1, 5],
            [0, 5, 4],
            [1, 2, 6],
            [1, 6, 5],
            [2, 3, 7],
            [2, 7, 6],
            [3, 0, 4],
            [3, 4, 7],
        ];
        let mut m = TriMesh::new(positions, tris);
        m.vertex_labels = Some(vec![11, 21, 21, 11, 11, 21, 21, 11]);
        m
    }

    /// FNV-1a 64 of the pixels.
    fn hash(img: &Image) -> u64 {
        img.rgba.iter().fold(0xcbf2_9ce4_8422_2325, |h, &b| {
            (h ^ u64::from(b)).wrapping_mul(0x0000_0100_0000_01b3)
        })
    }

    #[test]
    fn render_is_deterministic() {
        let m = cube();
        let items = [Item::new(&m).with_coloring(MeshColoring::VertexLabels(label_palette()))];
        let mut opts = RenderOptions::new(64, 48, View::Iso);
        opts.wireframe = Some(RenderOptions::WIRE);
        let a = render(&items, &opts).unwrap();
        let b = render(&items, &opts).unwrap();
        assert_eq!(a, b);
        assert_eq!((a.width, a.height, a.rgba.len()), (64, 48, 64 * 48 * 4));
        // golden hash of this exact image: update it only for an intended
        // change of the rasteriser
        assert_eq!(hash(&a), 0x2fb8_3e16_51c0_7d12, "hash {:#x}", hash(&a));
    }

    #[test]
    fn png_round_trip_header() {
        let img = Image::filled(3, 2, [1, 2, 3, 255]);
        let png = img.to_png().unwrap();
        assert_eq!(&png[1..4], b"PNG");
        let dec = png::Decoder::new(std::io::Cursor::new(png));
        let mut r = dec.read_info().unwrap();
        let mut buf = vec![0; r.output_buffer_size().unwrap()];
        let info = r.next_frame(&mut buf).unwrap();
        assert_eq!((info.width, info.height), (3, 2));
        assert_eq!(&buf[..4], &[1, 2, 3, 255]);
    }

    #[test]
    fn empty_scene_is_background() {
        let img = render(&[], &RenderOptions::new(4, 4, View::Frontal)).unwrap();
        assert!(
            img.rgba
                .chunks(4)
                .all(|p| p == color_u8(RenderOptions::BACKGROUND))
        );
    }

    #[test]
    fn invalid_sizes_are_rejected() {
        assert!(render(&[], &RenderOptions::new(0, 4, View::Frontal)).is_err());
        let mut o = RenderOptions::new(MAX_SIDE, 4, View::Frontal);
        o.supersample = 2;
        assert!(render(&[], &o).is_err());
    }

    #[test]
    fn mesh_covers_the_framed_area_and_labels_colour_it() {
        let mut m = cube();
        // every vertex faces the camera: full light on the front face
        m.normals = Some(vec![Vec3::NEG_Y; 8]);
        let items = [Item::new(&m).with_coloring(MeshColoring::VertexLabels(label_palette()))];
        // frontal: camera on −Y, +X to the right; x<0 has label 11, x>0 label 21
        let img = render(&items, &RenderOptions::new(100, 100, View::Frontal)).unwrap();
        let bg = color_u8(RenderOptions::BACKGROUND);
        assert_eq!(img.pixel(1, 1), bg, "corner is background (margin)");
        let left = img.pixel(25, 50);
        let right = img.pixel(75, 50);
        assert_ne!(left, bg);
        assert_ne!(right, bg);
        assert_ne!(left, right, "two labels, two colours");
        // the face looks straight at the camera: full light
        let c11 = color_u8(label_palette().color(11));
        assert_eq!(&left[..3], &c11[..3]);
    }

    #[test]
    fn supersampling_keeps_the_size() {
        let m = cube();
        let mut o = RenderOptions::new(40, 30, View::OcclusalUpper);
        o.supersample = 3;
        let img = render(&[Item::new(&m)], &o).unwrap();
        assert_eq!((img.width, img.height), (40, 30));
    }

    #[test]
    fn nearer_surfaces_hide_farther_ones() {
        let mut near = cube();
        near.normals = Some(vec![Vec3::NEG_Y; 8]);
        let mut far = near.clone();
        far.vertex_labels = None;
        // the far cube sits behind the near one along +Y (frontal view)
        let items = [
            Item::new(&far)
                .with_xform(Mat4::from_translation(Vec3::new(0.0, 5.0, 0.0)))
                .with_coloring(MeshColoring::Uniform(Color::GINGIVA)),
            Item::new(&near).with_coloring(MeshColoring::Uniform(Color::TOOTH)),
        ];
        let img = render(&items, &RenderOptions::new(50, 50, View::Frontal)).unwrap();
        let c = img.pixel(25, 25);
        assert_eq!(&c[..3], &color_u8(Color::TOOTH)[..3]);
    }
}
