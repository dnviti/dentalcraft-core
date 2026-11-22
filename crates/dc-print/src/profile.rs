//! Printer profiles (print plan §3.1).

use serde::{Deserialize, Serialize};

use crate::machine::MachineParams;

/// A resin printer as data: how to recognise it, its pixel grid and build
/// volume, and the layer heights it accepts.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PrinterProfile {
    /// Stable profile id (`saturn-4-ultra-16k`).
    pub id: String,
    /// The printer's SDCP `MachineName`, also written as the GOO printer name.
    pub machine_name: String,
    /// LCD resolution along X, in pixels.
    pub resolution_x: u32,
    /// LCD resolution along Y, in pixels.
    pub resolution_y: u32,
    /// Pixel pitch along X, in micrometres.
    pub pixel_pitch_x_um: f64,
    /// Pixel pitch along Y, in micrometres.
    pub pixel_pitch_y_um: f64,
    /// Build volume X, Y, Z in millimetres (the GOO platform size). Z is the
    /// print height limit: the Z axis may travel further (the machine
    /// parameters' I0 A0 B), the rest is room to lift the plate.
    pub build_mm: [f32; 3],
    /// Smallest accepted layer height, in micrometres.
    pub layer_min_um: u32,
    /// Largest accepted layer height, in micrometres.
    pub layer_max_um: u32,
    /// Mirror the plate's X axis when mapping to file pixels (and the GOO
    /// header flag).
    pub mirror_x: bool,
    /// Mirror the plate's Y axis when mapping to file pixels (and the GOO
    /// header flag).
    pub mirror_y: bool,
    /// The printer firmware's own parameters, when known: tilt release
    /// times, vat temperature and volume (see [`crate::machine`]).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub machine: Option<MachineParams>,
    /// `true` once the dimensional scale (pixel pitch) and the left/right
    /// orientation (mirror flags) have been checked against the printer: a
    /// vendor file and a measured test print (plan §9.3, Q2). While `false`,
    /// every `print.prepare` result carries [`PROFILE_NOT_VALIDATED`].
    #[serde(default)]
    pub validated: bool,
}

/// Warning code of a result prepared with a profile whose scale and
/// orientation are not yet checked against the printer.
pub const PROFILE_NOT_VALIDATED: &str = "print-profile-not-validated";

impl PrinterProfile {
    /// Profile id of the built-in Saturn 4 Ultra 16K.
    pub const SATURN_4_ULTRA_16K: &'static str = "saturn-4-ultra-16k";

    /// The Elegoo Saturn 4 Ultra 16K.
    ///
    /// Pixel pitch: Elegoo's spec gives 14 µm (X, 15 120 px) × 19 µm
    /// (Y, 6 230 px), i.e. a 211.68 × 118.37 mm plate, and that is what this
    /// profile uses. The printer itself reports `XYZsize` 218.88 × 128.88 ×
    /// 220 over SDCP, which would mean 14.48 × 20.69 µm pixels. The
    /// discrepancy (plan risk 4) is settled by the platform size Elegoo
    /// SatelLite writes into its GOO header, then by the gated dimensional
    /// print; until then the spec values stay.
    ///
    /// Mirror flags are `false` until the golden SatelLite file says
    /// otherwise (plan Q2).
    ///
    /// Neither the pitch nor the mirror flags are checked against the
    /// printer yet, so the profile is not `validated`: printing is allowed
    /// and every result says so.
    ///
    /// Machine parameters: the DENTAL 45 set prepared with Elegoo
    /// ([`MachineParams::dental_45`]). Its Z travel is 230 mm; the print
    /// limit stays the 220 mm build height of Elegoo's spec.
    pub fn saturn_4_ultra_16k() -> Self {
        Self {
            id: Self::SATURN_4_ULTRA_16K.into(),
            machine_name: "Saturn 4 Ultra 16K".into(),
            resolution_x: 15_120,
            resolution_y: 6_230,
            pixel_pitch_x_um: 14.0,
            pixel_pitch_y_um: 19.0,
            build_mm: [211.68, 118.37, 220.0],
            layer_min_um: 10,
            layer_max_um: 200,
            mirror_x: false,
            mirror_y: false,
            machine: Some(MachineParams::dental_45()),
            validated: false,
        }
    }

    /// The built-in profiles.
    pub fn builtin() -> Vec<Self> {
        vec![Self::saturn_4_ultra_16k()]
    }

    /// Whether `layer_um` is a supported layer height.
    pub fn supports_layer_um(&self, layer_um: u32) -> bool {
        (self.layer_min_um..=self.layer_max_um).contains(&layer_um)
    }

    /// Pixels in one layer image.
    pub fn pixel_count(&self) -> u64 {
        u64::from(self.resolution_x) * u64::from(self.resolution_y)
    }

    /// The one mapping between plate pixels and file pixels.
    ///
    /// Plate pixel (`col`, `row`) is the pixel whose centre lies at
    /// x = (col + ½)·pitch_x, y = (row + ½)·pitch_y in the plate frame. The
    /// file stores pixels row-major, `resolution_x` per row. `mirror_x`
    /// reverses the columns and `mirror_y` the rows. The map is an
    /// involution: applying it twice gives the input back, so the writer,
    /// the reader and the previews all use this function in both directions.
    ///
    /// Returns `None` outside the pixel grid.
    pub fn map_pixel(&self, col: u32, row: u32) -> Option<(u32, u32)> {
        if col >= self.resolution_x || row >= self.resolution_y {
            return None;
        }
        let c = if self.mirror_x {
            self.resolution_x - 1 - col
        } else {
            col
        };
        let r = if self.mirror_y {
            self.resolution_y - 1 - row
        } else {
            row
        };
        Some((c, r))
    }

    /// [`Self::map_pixel`] extended to plate points: the plate point
    /// (`x`, `y`) in mm goes to the file frame (mm from the file's first
    /// column and row) with the same mirror flags. The pixel centre of
    /// (`col`, `row`) maps to the pixel centre of `map_pixel(col, row)`, so
    /// the previews follow the layer images.
    pub fn map_point_mm(&self, x: f64, y: f64) -> (f64, f64) {
        let w = f64::from(self.resolution_x) * self.pixel_pitch_x_um / 1000.0;
        let h = f64::from(self.resolution_y) * self.pixel_pitch_y_um / 1000.0;
        (
            if self.mirror_x { w - x } else { x },
            if self.mirror_y { h - y } else { y },
        )
    }

    /// Codes of the warnings every job prepared with this profile carries:
    /// [`PROFILE_NOT_VALIDATED`] until the profile is validated.
    pub fn warnings(&self) -> Vec<&'static str> {
        if self.validated {
            Vec::new()
        } else {
            vec![PROFILE_NOT_VALIDATED]
        }
    }
}

/// Z of layer `index` (0-based) for a layer height of `layer_um` µm, in mm:
/// exactly (index + 1)·h, from integer micrometres, never accumulated.
pub fn layer_z_mm(index: u32, layer_um: u32) -> f32 {
    ((u64::from(index) + 1) * u64::from(layer_um)) as f64 as f32 / 1000.0
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn saturn_values_match_the_spec() {
        let p = PrinterProfile::saturn_4_ultra_16k();
        assert_eq!((p.resolution_x, p.resolution_y), (15_120, 6_230));
        assert_eq!(p.pixel_count(), 94_197_600);
        let w = p.resolution_x as f64 * p.pixel_pitch_x_um / 1000.0;
        let h = p.resolution_y as f64 * p.pixel_pitch_y_um / 1000.0;
        assert!((w - f64::from(p.build_mm[0])).abs() < 1e-3);
        assert!((h - f64::from(p.build_mm[1])).abs() < 1e-3);
        assert!(p.supports_layer_um(50) && !p.supports_layer_um(5));
    }

    #[test]
    fn mirror_is_an_involution() {
        let mut p = PrinterProfile::saturn_4_ultra_16k();
        p.mirror_x = true;
        p.mirror_y = true;
        for (c, r) in [(0, 0), (15_119, 6_229), (7, 4000)] {
            let (c2, r2) = p.map_pixel(c, r).unwrap();
            assert_eq!(p.map_pixel(c2, r2), Some((c, r)));
        }
        assert_eq!(p.map_pixel(0, 0), Some((15_119, 6_229)));
        assert_eq!(p.map_pixel(15_120, 0), None);
    }

    #[test]
    fn point_map_follows_the_pixel_map() {
        let mut p = PrinterProfile::saturn_4_ultra_16k();
        let centre = |p: &PrinterProfile, c: u32, r: u32| {
            (
                (f64::from(c) + 0.5) * p.pixel_pitch_x_um / 1000.0,
                (f64::from(r) + 0.5) * p.pixel_pitch_y_um / 1000.0,
            )
        };
        for (mx, my) in [(false, false), (true, false), (false, true), (true, true)] {
            p.mirror_x = mx;
            p.mirror_y = my;
            for (c, r) in [(0, 0), (15_119, 6_229), (7, 4000), (9000, 3)] {
                let (x, y) = centre(&p, c, r);
                let (fx, fy) = p.map_point_mm(x, y);
                let (fc, fr) = p.map_pixel(c, r).unwrap();
                let (ex, ey) = centre(&p, fc, fr);
                assert!((fx - ex).abs() < 1e-6 && (fy - ey).abs() < 1e-6);
            }
        }
    }

    #[test]
    fn unvalidated_profiles_warn() {
        let mut p = PrinterProfile::saturn_4_ultra_16k();
        assert!(!p.validated);
        assert_eq!(p.warnings(), [PROFILE_NOT_VALIDATED]);
        p.validated = true;
        assert!(p.warnings().is_empty());
    }

    #[test]
    fn layer_z_is_exact_and_increasing() {
        assert_eq!(layer_z_mm(0, 50), 0.05);
        assert_eq!(layer_z_mm(1734, 30), 52.05);
        let mut last = 0.0;
        for i in 0..4400 {
            let z = layer_z_mm(i, 50);
            assert!(z > last);
            last = z;
        }
        assert_eq!(last, 220.0);
    }
}
