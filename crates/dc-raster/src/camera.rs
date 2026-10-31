//! Standard views and the orthographic camera.

use dc_mesh::Aabb;
use glam::Vec3;

/// Standard dental views in the case world frame ([`dc_case::WorldFrame`],
/// the plugin's `worldframe.py`): incisors towards −Y, patient's right on
/// −X, lower arch with the teeth up (+Z) below the upper arch, whose teeth
/// point down. Directions match the app viewport (`dc-viewport::StdView`):
/// front = camera on −Y, up = +Z (the upper arch at the top of the image).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum View {
    /// Occlusal view of the upper arch: camera below (−Z) looking up,
    /// incisors at the top of the image.
    OcclusalUpper,
    /// Occlusal view of the lower arch: camera above (+Z) looking down,
    /// incisors at the top of the image.
    OcclusalLower,
    /// Frontal view: camera in front of the incisors (−Y), up = +Z.
    Frontal,
    /// From behind (+Y), up = +Z.
    Back,
    /// The patient's left side: camera on +X looking at −X, up = +Z.
    Left,
    /// The patient's right side: camera on −X looking at +X, up = +Z.
    Right,
    /// Plain top view: camera above (+Z), +Y up in the image.
    Top,
    /// Three-quarter view from the front, the patient's right and above.
    Iso,
}

impl View {
    /// Every view.
    pub const ALL: [View; 8] = [
        View::OcclusalUpper,
        View::OcclusalLower,
        View::Frontal,
        View::Back,
        View::Left,
        View::Right,
        View::Top,
        View::Iso,
    ];

    /// Canonical name, as accepted by [`View::parse`].
    pub fn name(self) -> &'static str {
        match self {
            View::OcclusalUpper => "occlusal",
            View::OcclusalLower => "occlusal_lower",
            View::Frontal => "frontal",
            View::Back => "back",
            View::Left => "left",
            View::Right => "right",
            View::Top => "top",
            View::Iso => "iso",
        }
    }

    /// View from its name (case and `-`/`_` insensitive). Accepts the
    /// canonical names and the aliases `occlusal_upper`, `bottom` (=
    /// occlusal), `front` (= frontal) and `isometric`.
    pub fn parse(s: &str) -> Option<View> {
        let s = s.trim().to_ascii_lowercase().replace('-', "_");
        Some(match s.as_str() {
            "occlusal" | "occlusal_upper" | "bottom" => View::OcclusalUpper,
            "occlusal_lower" => View::OcclusalLower,
            "frontal" | "front" => View::Frontal,
            "back" => View::Back,
            "left" => View::Left,
            "right" => View::Right,
            "top" => View::Top,
            "iso" | "isometric" => View::Iso,
            _ => return None,
        })
    }

    /// Viewing direction (camera towards the scene) and image-up direction.
    pub fn direction_and_up(self) -> (Vec3, Vec3) {
        match self {
            View::OcclusalUpper => (Vec3::Z, Vec3::NEG_Y),
            View::OcclusalLower => (Vec3::NEG_Z, Vec3::NEG_Y),
            View::Frontal => (Vec3::Y, Vec3::Z),
            View::Back => (Vec3::NEG_Y, Vec3::Z),
            View::Left => (Vec3::NEG_X, Vec3::Z),
            View::Right => (Vec3::X, Vec3::Z),
            View::Top => (Vec3::NEG_Z, Vec3::Y),
            View::Iso => (Vec3::new(1.0, 1.0, -0.8).normalize(), Vec3::Z),
        }
    }
}

/// Orthographic camera: world millimetres to image pixels.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Camera {
    /// World point at the centre of the image.
    pub center: Vec3,
    /// Unit viewing direction.
    pub dir: Vec3,
    /// Unit world direction of the image +x axis.
    pub right: Vec3,
    /// Unit world direction of the image up (−y) axis.
    pub up: Vec3,
    /// Pixels per millimetre.
    pub scale: f32,
    /// Image width in pixels.
    pub width: u32,
    /// Image height in pixels.
    pub height: u32,
}

impl Camera {
    /// Camera of `view` centred on `center` with `scale` pixels per mm.
    pub fn new(view: View, center: Vec3, scale: f32, width: u32, height: u32) -> Self {
        let (dir, up) = view.direction_and_up();
        let dir = dir.normalize();
        let right = dir.cross(up).normalize();
        let up = right.cross(dir);
        Self {
            center,
            dir,
            right,
            up,
            scale,
            width,
            height,
        }
    }

    /// Camera of `view` that fits the corners of `bounds` in the image,
    /// leaving `margin` (fraction of each side, clamped to 0..0.45) free on
    /// every border, centred on the projected box.
    pub fn frame(bounds: &Aabb, view: View, width: u32, height: u32, margin: f32) -> Self {
        let mut cam = Self::new(view, bounds.center(), 1.0, width, height);
        let (mut lo, mut hi) = ([f32::INFINITY; 2], [f32::NEG_INFINITY; 2]);
        for i in 0..8 {
            let c = Vec3::new(
                if i & 1 == 0 {
                    bounds.min.x
                } else {
                    bounds.max.x
                },
                if i & 2 == 0 {
                    bounds.min.y
                } else {
                    bounds.max.y
                },
                if i & 4 == 0 {
                    bounds.min.z
                } else {
                    bounds.max.z
                },
            ) - cam.center;
            let uv = [c.dot(cam.right), c.dot(cam.up)];
            for k in 0..2 {
                lo[k] = lo[k].min(uv[k]);
                hi[k] = hi[k].max(uv[k]);
            }
        }
        let m = margin.clamp(0.0, 0.45);
        let ext = [(hi[0] - lo[0]).max(1e-6), (hi[1] - lo[1]).max(1e-6)];
        let sx = width as f32 * (1.0 - 2.0 * m) / ext[0];
        let sy = height as f32 * (1.0 - 2.0 * m) / ext[1];
        cam.scale = sx.min(sy);
        cam.center += cam.right * (0.5 * (lo[0] + hi[0])) + cam.up * (0.5 * (lo[1] + hi[1]));
        cam
    }

    /// Image position of world point `p`: x and y in pixels (y down), z =
    /// depth along the viewing direction (larger is farther).
    pub fn project(&self, p: Vec3) -> Vec3 {
        let d = p - self.center;
        Vec3::new(
            self.width as f32 * 0.5 + d.dot(self.right) * self.scale,
            self.height as f32 * 0.5 - d.dot(self.up) * self.scale,
            d.dot(self.dir),
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn bounds() -> Aabb {
        Aabb {
            min: Vec3::new(-30.0, -25.0, -5.0),
            max: Vec3::new(30.0, 25.0, 15.0),
        }
    }

    #[test]
    fn views_follow_the_case_world_frame() {
        use dc_case::WorldFrame;
        let anterior = WorldFrame::ANTERIOR.as_vec3();
        let to_lower = WorldFrame::TOWARDS_LOWER.as_vec3();
        let lower_teeth = WorldFrame::OCCLUSAL_NORMAL.as_vec3();
        let cam = |v: View| Camera::new(v, Vec3::ZERO, 10.0, 200, 200);
        // a lower crown tip below an upper one, a right molar and the incisors
        let (lower_tip, upper_tip) = (Vec3::new(0.0, -20.0, -1.0), Vec3::new(0.0, -21.0, 1.0));
        let right_molar = Vec3::new(-20.0, 10.0, 0.0);
        let f = cam(View::Frontal);
        assert!(
            (f.dir + anterior).length() < 1e-6,
            "front looks from the incisors"
        );
        assert!(
            f.project(upper_tip).y < f.project(lower_tip).y,
            "upper arch on top"
        );
        assert!(
            f.project(right_molar).x < 100.0,
            "patient's right on the image left"
        );
        let lo = cam(View::OcclusalLower);
        assert!((lo.dir - to_lower).length() < 1e-6);
        assert!(lo.dir.dot(lower_teeth) < 0.0, "lower teeth face the camera");
        let up = cam(View::OcclusalUpper);
        assert!((up.dir + to_lower).length() < 1e-6);
        assert!(
            up.dir.dot(-lower_teeth) < 0.0,
            "upper teeth face the camera"
        );
        for v in [View::OcclusalLower, View::OcclusalUpper] {
            let c = cam(v);
            assert!(
                (c.up - anterior).length() < 1e-6,
                "{v:?}: incisors at the top"
            );
        }
        assert!(cam(View::Right).dir.x > 0.9 && cam(View::Left).dir.x < -0.9);
        // the iso view looks from the front, the patient's right and above
        let iso = cam(View::Iso);
        assert!(iso.dir.y > 0.0 && iso.dir.x > 0.0 && iso.dir.z < 0.0);
    }

    #[test]
    fn names_round_trip_and_aliases() {
        for v in View::ALL {
            assert_eq!(View::parse(v.name()), Some(v));
        }
        assert_eq!(View::parse("Front"), Some(View::Frontal));
        assert_eq!(View::parse("occlusal-upper"), Some(View::OcclusalUpper));
        assert_eq!(View::parse("bottom"), Some(View::OcclusalUpper));
        assert_eq!(View::parse("sideways"), None);
    }

    #[test]
    fn bases_are_orthonormal() {
        for v in View::ALL {
            let c = Camera::new(v, Vec3::ZERO, 1.0, 10, 10);
            assert!((c.right.length() - 1.0).abs() < 1e-5, "{v:?}");
            assert!((c.up.length() - 1.0).abs() < 1e-5, "{v:?}");
            assert!(c.right.dot(c.up).abs() < 1e-5, "{v:?}");
            assert!(c.right.dot(c.dir).abs() < 1e-5, "{v:?}");
            assert!(c.up.dot(c.dir).abs() < 1e-5, "{v:?}");
        }
    }

    #[test]
    fn frontal_framing_fits_the_box() {
        // frontal: image x = world X (60 mm), image up = world Z (20 mm)
        let c = Camera::frame(&bounds(), View::Frontal, 800, 600, 0.05);
        // width limits: 800 * 0.9 / 60 = 12 px/mm (height would allow 27)
        assert!((c.scale - 12.0).abs() < 1e-4, "{}", c.scale);
        let p = c.project(Vec3::new(-30.0, 0.0, 15.0));
        assert!((p.x - 40.0).abs() < 1e-3, "{p}");
        // Z 15 is the top of the box: 10 mm above the centre (z = 5)
        assert!((p.y - (300.0 - 120.0)).abs() < 1e-3, "{p}");
        let q = c.project(Vec3::new(30.0, 0.0, -5.0));
        assert!((q.x - 760.0).abs() < 1e-3, "{q}");
        // the camera looks at +Y: points further back are deeper
        assert!(c.project(Vec3::new(0.0, 25.0, 0.0)).z > c.project(Vec3::ZERO).z);
    }

    #[test]
    fn occlusal_framing_puts_incisors_on_top() {
        let c = Camera::frame(&bounds(), View::OcclusalUpper, 600, 600, 0.0);
        // 60 mm wide, 50 mm tall: width limits, 10 px/mm
        assert!((c.scale - 10.0).abs() < 1e-4, "{}", c.scale);
        // incisors towards −Y: top of the image
        let incisor = c.project(Vec3::new(0.0, -25.0, 0.0));
        let molar = c.project(Vec3::new(0.0, 25.0, 0.0));
        assert!(incisor.y < molar.y);
        assert!((incisor.y - 50.0).abs() < 1e-3, "{incisor}");
        // from below, the patient's right (−X) is on the left of the image
        assert!(c.project(Vec3::new(-30.0, 0.0, 0.0)).x < 1e-3);
    }

    #[test]
    fn every_view_keeps_all_corners_inside() {
        let b = bounds();
        for v in View::ALL {
            let c = Camera::frame(&b, v, 320, 200, 0.1);
            for i in 0..8 {
                let p = Vec3::new(
                    if i & 1 == 0 { b.min.x } else { b.max.x },
                    if i & 2 == 0 { b.min.y } else { b.max.y },
                    if i & 4 == 0 { b.min.z } else { b.max.z },
                );
                let s = c.project(p);
                assert!(s.x >= 31.99 && s.x <= 288.01, "{v:?} {s}");
                assert!(s.y >= 19.99 && s.y <= 180.01, "{v:?} {s}");
            }
        }
    }
}
