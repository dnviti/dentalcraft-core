//! Scanline fill of one layer's section (print plan §5).
//!
//! The section segments become an edge table over the pixel rows. Row j is
//! sampled at its centre y_j ([`super::pixel`]), and a segment crosses row j
//! when y_lo ≤ y_j < y_hi (half-open). A contour vertex on a row centre
//! therefore counts once where the contour passes through it, and zero times
//! or twice (cancelling) where the contour turns; flat segments never count.
//! The crossing x is interpolated from the segment's lower end, so two
//! segments meeting at a vertex on the row give the very same x.
//!
//! Each row's crossings are sorted by x and swept with a winding count: a
//! segment going −y adds 1, going +y subtracts 1, so an outward-oriented
//! solid winds +1 inside ([`super::section`]). Inside means a nonzero count
//! over all meshes together, so overlapping meshes form one solid and cure
//! once. Inside intervals become pixel spans in [`quantise_row`].

use std::ops::Range;

use super::pixel::{Axis, Grid};
use super::section::Segment;

/// A section segment set up for the row sweep.
#[derive(Debug, Clone, Copy)]
struct Edge {
    /// First row it crosses.
    row0: u32,
    /// One past the last row it crosses.
    row1: u32,
    /// x of its lower end (smaller y), mm.
    x_lo: f64,
    /// y of its lower end, mm.
    y_lo: f64,
    /// dx / dy.
    slope: f64,
    /// Winding step: +1 going −y, −1 going +y.
    wind: i32,
}

/// Lit pixel spans of one layer, per plate row.
#[derive(Debug, Default)]
pub(crate) struct RowSpans {
    /// Row r's spans are `spans[offsets[r]..offsets[r + 1]]`.
    offsets: Vec<usize>,
    /// Sorted, disjoint and non-touching within a row.
    spans: Vec<Range<u32>>,
    /// Lit pixels over all rows.
    lit: u64,
}

impl RowSpans {
    /// The lit column spans of plate row `row` (empty outside the grid).
    pub(crate) fn row(&self, row: u32) -> &[Range<u32>] {
        let r = row as usize;
        match (self.offsets.get(r), self.offsets.get(r + 1)) {
            (Some(&a), Some(&b)) => &self.spans[a..b],
            _ => &[],
        }
    }

    /// Lit pixels over all rows.
    pub(crate) fn lit(&self) -> u64 {
        self.lit
    }
}

/// Reusable buffers of the scanline fill.
#[derive(Debug, Default)]
pub(crate) struct Filler {
    edges: Vec<Edge>,
    active: Vec<Edge>,
    crossings: Vec<(f64, i32)>,
}

impl Filler {
    /// Forgets the segments of the previous layer.
    pub(crate) fn clear(&mut self) {
        self.edges.clear();
    }

    /// Adds one section segment. Segments that cross no row centre inside
    /// the grid (flat, too short, or off the plate in y) are dropped here.
    pub(crate) fn add(&mut self, s: Segment, grid: &Grid) {
        let (lo, hi, wind) = if s.from[1] > s.to[1] {
            (s.to, s.from, 1)
        } else if s.from[1] < s.to[1] {
            (s.from, s.to, -1)
        } else {
            return;
        };
        let row0 = grid.y.first_centre_at_or_after(lo[1]);
        let row1 = grid.y.first_centre_at_or_after(hi[1]);
        if row0 >= row1 {
            return;
        }
        // Finite for finite input, except an x step over a subnormal y step.
        let slope = (hi[0] - lo[0]) / (hi[1] - lo[1]);
        self.edges.push(Edge {
            row0,
            row1,
            x_lo: lo[0],
            y_lo: lo[1],
            slope: if slope.is_finite() { slope } else { 0.0 },
            wind,
        });
    }

    /// Fills every row from the added segments into `out`.
    pub(crate) fn fill(&mut self, grid: &Grid, out: &mut RowSpans) {
        out.offsets.clear();
        out.spans.clear();
        out.lit = 0;
        self.edges.sort_unstable_by_key(|e| e.row0);
        self.active.clear();
        let mut next = 0;
        let mut row = 0;
        while next < self.edges.len() || !self.active.is_empty() {
            if self.active.is_empty() {
                row = self.edges[next].row0;
            }
            while let Some(e) = self.edges.get(next)
                && e.row0 == row
            {
                self.active.push(*e);
                next += 1;
            }
            // Rows up to this one without spans start (and end) here.
            out.offsets.resize(row as usize + 1, out.spans.len());
            let y = grid.y.centre_mm(row);
            self.crossings.clear();
            self.crossings.extend(
                self.active
                    .iter()
                    .map(|e| (e.x_lo + (y - e.y_lo) * e.slope, e.wind)),
            );
            self.crossings
                .sort_unstable_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));
            out.lit += quantise_row(&self.crossings, &grid.x, &mut out.spans);
            row += 1;
            self.active.retain(|e| e.row1 > row);
        }
        out.offsets
            .resize(grid.y.count() as usize + 1, out.spans.len());
    }
}

/// Turns one row's sorted crossings into lit pixel spans appended to `out`;
/// returns the pixels lit.
///
/// Nonzero rule: an inside interval [x_a, x_b) runs from where the winding
/// leaves 0 to where it comes back. Pixel k is lit when its centre lies in
/// it (half-open, exact through [`Axis::first_centre_at_or_after`]), so
/// pixel counts are exact and touching intervals never share a pixel. An
/// interval still open at the end of the row (only an open mesh does that)
/// is dropped rather than lighting the rest of the row.
///
/// **Anti-aliasing hook.** v1 is black and white, and this is the only place
/// where exact section geometry becomes pixels. Grey edges would be made
/// here: the crossings give the covered fraction of each edge pixel (averaged
/// over sub-rows for the y direction), and the row would carry grey runs
/// instead of spans. The GOO RLE already encodes grey runs
/// ([`crate::goo::rle`]).
fn quantise_row(crossings: &[(f64, i32)], x: &Axis, out: &mut Vec<Range<u32>>) -> u64 {
    let row_start = out.len();
    let mut wind = 0i32;
    let mut start = 0.0;
    let mut lit = 0u64;
    for &(cx, w) in crossings {
        let before = wind;
        wind += w;
        if before == 0 && wind != 0 {
            start = cx;
        } else if before != 0 && wind == 0 {
            let a = x.first_centre_at_or_after(start);
            let b = x.first_centre_at_or_after(cx);
            if a < b {
                lit += u64::from(b - a);
                let in_row = out.len() > row_start;
                match out.last_mut() {
                    Some(last) if in_row && last.end == a => last.end = b,
                    _ => out.push(a..b),
                }
            }
        }
    }
    lit
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::PrinterProfile;

    fn grid(cols: u32, rows: u32) -> Grid {
        let mut p = PrinterProfile::saturn_4_ultra_16k();
        p.resolution_x = cols;
        p.resolution_y = rows;
        // 1 mm pixels keep the arithmetic readable.
        p.pixel_pitch_x_um = 1000.0;
        p.pixel_pitch_y_um = 1000.0;
        Grid::new(&p).unwrap()
    }

    /// Counter-clockwise polygon → its section segments.
    fn polygon(filler: &mut Filler, g: &Grid, pts: &[[f64; 2]]) {
        for k in 0..pts.len() {
            let s = Segment {
                from: pts[k],
                to: pts[(k + 1) % pts.len()],
            };
            filler.add(s, g);
        }
    }

    /// Row `r`'s spans as (start, end) pairs.
    fn row(spans: &RowSpans, r: u32) -> Vec<(u32, u32)> {
        spans.row(r).iter().map(|s| (s.start, s.end)).collect()
    }

    #[test]
    fn square_lights_centres_half_open() {
        let g = grid(10, 8);
        let mut f = Filler::default();
        let mut out = RowSpans::default();
        // [1.5, 4.5) × [2.5, 5.5): centres 1.5..=3.5 → columns 1..4;
        // y centres 2.5, 3.5, 4.5 → rows 2..5.
        polygon(
            &mut f,
            &g,
            &[[1.5, 2.5], [4.5, 2.5], [4.5, 5.5], [1.5, 5.5]],
        );
        f.fill(&g, &mut out);
        for r in 0..8 {
            let want: &[(u32, u32)] = if (2..5).contains(&r) { &[(1, 4)] } else { &[] };
            assert_eq!(row(&out, r), want, "row {r}");
        }
        assert_eq!(out.lit(), 9);
        assert!(out.row(8).is_empty());
    }

    #[test]
    fn overlaps_union_and_holes_subtract() {
        let g = grid(12, 4);
        let mut f = Filler::default();
        let mut out = RowSpans::default();
        // Two overlapping squares (both counter-clockwise) and, inside the
        // second, a clockwise hole.
        polygon(
            &mut f,
            &g,
            &[[0.0, 0.0], [6.0, 0.0], [6.0, 4.0], [0.0, 4.0]],
        );
        polygon(
            &mut f,
            &g,
            &[[4.0, 0.0], [11.0, 0.0], [11.0, 4.0], [4.0, 4.0]],
        );
        polygon(
            &mut f,
            &g,
            &[[8.0, 1.0], [8.0, 3.0], [10.0, 3.0], [10.0, 1.0]],
        );
        f.fill(&g, &mut out);
        assert_eq!(row(&out, 0), [(0, 11)]);
        assert_eq!(row(&out, 1), [(0, 8), (10, 11)]);
        assert_eq!(row(&out, 2), [(0, 8), (10, 11)]);
        assert_eq!(row(&out, 3), [(0, 11)]);
        assert_eq!(out.lit(), 11 * 2 + 9 * 2);
    }

    #[test]
    fn vertices_on_row_centres_count_once() {
        let g = grid(10, 10);
        let mut f = Filler::default();
        let mut out = RowSpans::default();
        // A diamond with its four vertices on pixel centres.
        polygon(
            &mut f,
            &g,
            &[[4.5, 0.5], [8.5, 4.5], [4.5, 8.5], [0.5, 4.5]],
        );
        f.fill(&g, &mut out);
        // Bottom vertex row: the vertex itself (x = 4.5) is [4.5, 4.5): empty.
        assert!(out.row(0).is_empty());
        // Row 1 (y = 1.5): [3.5, 5.5) holds centres 3.5 and 4.5.
        assert_eq!(row(&out, 1), [(3, 5)]);
        // Through the side vertices: counted once, not doubled or leaked.
        assert_eq!(row(&out, 4), [(0, 8)]);
        assert_eq!(row(&out, 7), [(3, 5)]);
        // Top vertex row: excluded (half-open in y).
        assert!(out.row(8).is_empty());
        assert!((0..10).all(|r| out.row(r).len() <= 1));
    }

    #[test]
    fn open_contour_does_not_leak() {
        let g = grid(10, 4);
        let mut f = Filler::default();
        let mut out = RowSpans::default();
        // A lone edge going −y: winding opens and never closes.
        f.add(
            Segment {
                from: [3.0, 4.0],
                to: [3.0, 0.0],
            },
            &g,
        );
        f.fill(&g, &mut out);
        assert_eq!(out.lit(), 0);
    }

    #[test]
    fn off_grid_geometry_is_clipped() {
        let g = grid(6, 3);
        let mut f = Filler::default();
        let mut out = RowSpans::default();
        polygon(
            &mut f,
            &g,
            &[[-5.0, -5.0], [20.0, -5.0], [20.0, 1.0], [-5.0, 1.0]],
        );
        f.fill(&g, &mut out);
        assert_eq!(row(&out, 0), [(0, 6)]);
        assert!(out.row(1).is_empty());
        assert_eq!(out.lit(), 6);
    }
}
