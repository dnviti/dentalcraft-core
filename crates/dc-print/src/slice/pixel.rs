//! Plate frame ↔ pixel grid (print plan §5).
//!
//! The plate frame has its origin at the corner of plate pixel (0, 0), x along
//! the columns and y along the rows: plate pixel (col, row) has its centre at
//! x = (col + ½)·pitch_x, y = (row + ½)·pitch_y, as
//! [`PrinterProfile::map_pixel`] defines it. Pixels need not be square
//! (14 × 19 µm on the Saturn 4 Ultra 16K), so each axis has its own pitch.

use crate::PrinterProfile;
use crate::goo::GooError;

/// One axis of the pixel grid.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct Axis {
    /// Pixel pitch, µm.
    pitch_um: f64,
    /// Number of pixels.
    count: u32,
}

impl Axis {
    /// An axis of `count` pixels of `pitch_um` µm.
    pub(crate) fn new(pitch_um: f64, count: u32) -> Self {
        Self { pitch_um, count }
    }

    /// Number of pixels.
    pub(crate) fn count(&self) -> u32 {
        self.count
    }

    /// Pixel pitch, mm.
    pub(crate) fn pitch_mm(&self) -> f64 {
        self.pitch_um / 1000.0
    }

    /// Centre of pixel `k`, mm: (k + ½)·pitch. The one definition of a pixel
    /// centre; every sampling decision compares against it.
    pub(crate) fn centre_mm(&self, k: u32) -> f64 {
        (2.0 * f64::from(k) + 1.0) * self.pitch_um / 2000.0
    }

    /// The first pixel whose centre is at or after `v` (mm), in `0..=count`
    /// (`count` when there is none).
    ///
    /// Exact with respect to [`Axis::centre_mm`]: for the result `k`,
    /// centre(k − 1) < v ≤ centre(k). So the pixels whose centre lies in the
    /// half-open interval [a, b) are `first(a)..first(b)`, and intervals that
    /// touch never share a pixel.
    pub(crate) fn first_centre_at_or_after(&self, v: f64) -> u32 {
        let guess = (v * 1000.0 / self.pitch_um - 0.5).ceil();
        // `as` saturates (NaN gives 0); the loops below make the result exact.
        let mut k = if guess >= f64::from(self.count) {
            self.count
        } else {
            guess.max(0.0) as u32
        };
        while k > 0 && self.centre_mm(k - 1) >= v {
            k -= 1;
        }
        while k < self.count && self.centre_mm(k) < v {
            k += 1;
        }
        k
    }
}

/// The pixel grid of a profile, in the plate frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct Grid {
    /// Columns (plate x).
    pub(crate) x: Axis,
    /// Rows (plate y).
    pub(crate) y: Axis,
}

impl Grid {
    /// The grid of `profile`; refuses a pitch that is not finite and positive.
    pub(crate) fn new(profile: &PrinterProfile) -> Result<Self, GooError> {
        let valid = |p: f64| p.is_finite() && p > 0.0;
        if !valid(profile.pixel_pitch_x_um) || !valid(profile.pixel_pitch_y_um) {
            return Err(GooError::Field("pixel pitch"));
        }
        Ok(Self {
            x: Axis::new(profile.pixel_pitch_x_um, profile.resolution_x),
            y: Axis::new(profile.pixel_pitch_y_um, profile.resolution_y),
        })
    }

    /// Area of one pixel, mm².
    pub(crate) fn pixel_area_mm2(&self) -> f64 {
        self.x.pitch_mm() * self.y.pitch_mm()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn centres_follow_map_pixel_frame() {
        let g = Grid::new(&PrinterProfile::saturn_4_ultra_16k()).unwrap();
        assert_eq!(g.x.centre_mm(0), 0.007);
        assert_eq!(g.y.centre_mm(0), 0.0095);
        assert_eq!(g.x.centre_mm(15_119), 211.673);
        assert!((g.pixel_area_mm2() - 0.014 * 0.019).abs() < 1e-15);
    }

    #[test]
    fn first_centre_is_exact_and_clamped() {
        let g = Grid::new(&PrinterProfile::saturn_4_ultra_16k()).unwrap();
        for axis in [g.x, g.y] {
            for k in (0..axis.count()).step_by(37) {
                let c = axis.centre_mm(k);
                assert_eq!(axis.first_centre_at_or_after(c), k);
                assert_eq!(axis.first_centre_at_or_after(c - 1e-9), k);
                assert_eq!(axis.first_centre_at_or_after(c + 1e-9), k + 1);
            }
            assert_eq!(axis.first_centre_at_or_after(-5.0), 0);
            assert_eq!(axis.first_centre_at_or_after(1e9), axis.count());
            assert_eq!(axis.first_centre_at_or_after(f64::NAN), 0);
        }
    }

    #[test]
    fn bad_pitch_is_refused() {
        let mut p = PrinterProfile::saturn_4_ultra_16k();
        p.pixel_pitch_y_um = 0.0;
        assert!(matches!(Grid::new(&p), Err(GooError::Field(_))));
    }
}
