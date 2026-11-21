//! Nesting of parts on the build plate (print plan §4). Owner: PlateJob.
//!
//! Bottom-left fill on a grid of [`CELL_MM`] cells covering the plate inside
//! its margins:
//! - a footprint is the convex hull of its points, tried at 0, 90, 180 and
//!   270° about +Z (exact quarter turns, no trigonometry);
//! - a part covers every cell its hull touches; once placed it blocks every
//!   cell its hull grown by the gap touches, so two parts never come closer
//!   than the gap and never leave the margins, whatever the cell rounding;
//! - each turn of a part finds its free position, and the part takes the
//!   best one by the fill's [`Rank`] (the lowest top edge, the lowest row
//!   or the leftmost column first; ties: the fewest turns);
//! - parts are taken tallest first (heights in bands of
//!   [`HEIGHT_BAND_MM`]), then largest first, then in input order, and each
//!   goes on the first plate where it fits. Plates fill up before a new one
//!   opens, and since a plate prints as long as its tallest part, the tall
//!   parts share the first plates and the short ones the last;
//! - a greedy fill can turn or shift parts in a way that wastes room
//!   (identical arches nested into each other's gaps leave the row ends
//!   empty), so the fill runs with a few fixed [`STRATEGIES`] (allowed
//!   turns and rank) and keeps the best result: the fewest plates, then the
//!   lowest sum of plate heights, then the most parts on the first plates,
//!   then the earlier strategy.
//!
//! Sequential, and integer arithmetic once each footprint is rasterised, so
//! the same footprints give the same plates on every platform.

use crate::PrinterProfile;
use crate::part::Footprint;
use glam::{DVec2, Vec2};
use serde::{Deserialize, Serialize};

/// Side of a nesting grid cell, mm (print plan §4). A power of two, so cell
/// boundaries are exact in floating point.
pub const CELL_MM: f32 = 0.25;

/// Parts whose heights fall in the same band of this many millimetres are
/// nested as equally tall (≈ 20 layers at 0.05 mm), largest first.
pub const HEIGHT_BAND_MM: f32 = 1.0;

/// Free positions whose rank (top edge, row or column) is within this many
/// millimetres of the best one count as equally good, and the other axis
/// decides: identical parts then line up in rows or columns instead of
/// sinking a fraction of a millimetre into each other's gaps.
pub const RANK_TOLERANCE_MM: f32 = 2.0;

/// Nesting parameters.
#[derive(Debug, Clone, PartialEq)]
pub struct LayoutParams {
    /// Minimum gap between parts, mm.
    pub gap_mm: f32,
    /// Margin from the plate edge, mm.
    pub margin_mm: f32,
}

impl Default for LayoutParams {
    fn default() -> Self {
        Self {
            gap_mm: 3.0,
            margin_mm: 4.0,
        }
    }
}

/// Errors of the nesting, with stable codes ([`LayoutError::code`]).
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum LayoutError {
    /// A part is taller than the printer builds.
    #[error(
        "print-part-too-tall: {id} is {height_mm:.2} mm tall, the printer builds up to {max_mm:.2} mm"
    )]
    TooTall {
        /// Part id.
        id: String,
        /// Height of the part with its supports, mm.
        height_mm: f32,
        /// Build height of the printer, mm.
        max_mm: f32,
    },
    /// A part does not fit an empty plate in any of its four turns.
    #[error(
        "print-part-too-big: {id} needs {size_mm:?} mm, the plate has {usable_mm:?} mm inside the margins"
    )]
    TooBig {
        /// Part id.
        id: String,
        /// Bounding box of the footprint (X, Y), mm.
        size_mm: [f32; 2],
        /// Room on the plate inside the margins (X, Y), mm.
        usable_mm: [f32; 2],
    },
    /// A footprint has no points, or a coordinate or the height is not a
    /// finite number (the height also not negative).
    #[error("print-invalid-footprint: {0}")]
    InvalidFootprint(String),
    /// The gap or the margin is negative or not finite, or the margins leave
    /// no room on the plate.
    #[error("print-invalid-layout: {0}")]
    InvalidParams(String),
}

impl LayoutError {
    /// Stable identifier, the same in every language (kebab-case).
    pub fn code(&self) -> &'static str {
        match self {
            Self::TooTall { .. } => "print-part-too-tall",
            Self::TooBig { .. } => "print-part-too-big",
            Self::InvalidFootprint(_) => "print-invalid-footprint",
            Self::InvalidParams(_) => "print-invalid-layout",
        }
    }
}

/// Where nesting put one part: turn it `quarter_turns` × 90° about +Z
/// (counter-clockwise seen from above, about the origin of its footprint),
/// then move it by `offset`.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Placement {
    /// Index of the part in the input.
    pub index: usize,
    /// Counter-clockwise quarter turns about +Z, 0–3.
    pub quarter_turns: u8,
    /// XY translation applied after the turn, mm.
    pub offset: Vec2,
}

impl Placement {
    /// The turn in degrees (0, 90, 180 or 270).
    pub fn rotation_deg(&self) -> f32 {
        90.0 * f32::from(self.quarter_turns % 4)
    }

    /// Plate position of point `p` of the part's footprint.
    pub fn apply(&self, p: Vec2) -> Vec2 {
        quarter_turn(p, self.quarter_turns) + self.offset
    }
}

/// `p` turned `turns` × 90° counter-clockwise about the origin. Exact: only
/// swaps and sign changes.
pub fn quarter_turn(p: Vec2, turns: u8) -> Vec2 {
    match turns % 4 {
        0 => p,
        1 => Vec2::new(-p.y, p.x),
        2 => -p,
        _ => Vec2::new(p.y, -p.x),
    }
}

/// One plate: where each of its parts goes.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Plate {
    /// One placement per part on this plate, by input index.
    pub parts: Vec<Placement>,
}

/// Checks the gap and the margin against the plate of `profile`, as
/// [`nest`] does before placing anything.
///
/// # Errors
/// [`LayoutError::InvalidParams`].
pub fn check_params(profile: &PrinterProfile, params: &LayoutParams) -> Result<(), LayoutError> {
    Grid::new(profile, params).map(|_| ())
}

/// Packs footprints on as few plates as needed; see the module
/// documentation for the rules. Every footprint is placed exactly once; no
/// input gives an empty result except an empty input.
///
/// # Errors
/// [`LayoutError::TooTall`] or [`LayoutError::TooBig`] for the first such
/// part in input order (never a silent drop),
/// [`LayoutError::InvalidFootprint`] and [`LayoutError::InvalidParams`] for
/// unusable input.
pub fn nest(
    footprints: &[Footprint],
    profile: &PrinterProfile,
    params: &LayoutParams,
) -> Result<Vec<Plate>, LayoutError> {
    let grid = Grid::new(profile, params)?;
    let parts = footprints
        .iter()
        .map(|f| Part::new(f, profile.build_mm[2], params.gap_mm, &grid))
        .collect::<Result<Vec<_>, _>>()?;

    let mut order: Vec<usize> = (0..parts.len()).collect();
    order.sort_by(|&a, &b| {
        let (pa, pb) = (&parts[a], &parts[b]);
        pb.band
            .cmp(&pa.band)
            .then(pb.area.total_cmp(&pa.area))
            .then(a.cmp(&b))
    });

    let mut best: Option<Packing> = None;
    for strategy in &STRATEGIES {
        let packed = pack(&parts, &order, footprints, &grid, strategy);
        if let Some(p) = packed
            && best.as_ref().is_none_or(|b| p.is_better_than(b))
        {
            best = Some(p);
        }
    }
    // the first strategy allows every turn, and every part fits an empty
    // plate in one of them (`Part::new`)
    let best = best.expect("the free strategy packs every part");
    Ok(best
        .plates
        .into_iter()
        .map(|mut p| {
            p.placements.sort_by_key(|pl| pl.index);
            Plate {
                parts: p.placements,
            }
        })
        .collect())
}

/// Which free position a fill takes first.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Rank {
    /// The lowest top edge, then the leftmost.
    LowestTop,
    /// The lowest bottom row, then the leftmost.
    LowestRow,
    /// The leftmost column, then the lowest row: fills column by column.
    Leftmost,
}

/// How one fill places parts.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Strategy {
    /// Allowed quarter turns, bit `k` for `k` turns.
    pub turns: u8,
    /// Which free position comes first (ties: the fewest turns).
    pub rank: Rank,
}

const fn strategy(turns: u8, rank: Rank) -> Strategy {
    Strategy { turns, rank }
}

/// The fills [`nest`] compares, in order of preference on a tie. The first
/// allows every turn, so it always places every part; the ones with one
/// turn lay identical parts out as a grid, which nesting parts into each
/// other's gaps can beat or spoil.
pub const STRATEGIES: [Strategy; 9] = [
    strategy(0b1111, Rank::LowestTop),
    strategy(0b1111, Rank::LowestRow),
    strategy(0b1111, Rank::Leftmost),
    strategy(0b0101, Rank::LowestTop),
    strategy(0b1010, Rank::LowestTop),
    strategy(0b0001, Rank::Leftmost),
    strategy(0b0010, Rank::Leftmost),
    strategy(0b0001, Rank::LowestRow),
    strategy(0b0010, Rank::LowestRow),
];

/// The plates of one fill and what ranks it.
struct Packing {
    plates: Vec<Occupancy>,
    /// Sum over the plates of their tallest part, mm.
    height_sum: f64,
    /// Parts per plate.
    counts: Vec<usize>,
}

impl Packing {
    fn is_better_than(&self, other: &Self) -> bool {
        self.plates
            .len()
            .cmp(&other.plates.len())
            .then(self.height_sum.total_cmp(&other.height_sum))
            .then_with(|| other.counts.cmp(&self.counts))
            .is_lt()
    }
}

/// First-fit of the parts in `order` with `strategy`; `None` when a part
/// fits no plate with the turns it allows.
fn pack(
    parts: &[Part],
    order: &[usize],
    footprints: &[Footprint],
    grid: &Grid,
    strategy: &Strategy,
) -> Option<Packing> {
    let mut plates: Vec<Occupancy> = Vec::new();
    for &index in order {
        let part = &parts[index];
        let placed = plates
            .iter_mut()
            .any(|plate| plate.place(index, part, grid, strategy));
        if !placed {
            let mut plate = Occupancy::new(grid);
            if !plate.place(index, part, grid, strategy) {
                return None;
            }
            plates.push(plate);
        }
    }
    let height = |p: &Occupancy| {
        p.placements
            .iter()
            .map(|pl| footprints[pl.index].height_mm)
            .fold(0.0f32, f32::max)
    };
    Some(Packing {
        height_sum: plates.iter().map(|p| f64::from(height(p))).sum(),
        counts: plates.iter().map(|p| p.placements.len()).collect(),
        plates,
    })
}

/// The nesting grid: `nx` × `ny` cells from the margin corner.
struct Grid {
    /// Plate position of the corner of cell (0, 0), mm.
    origin: DVec2,
    /// Cell side, mm.
    cell: f64,
    /// Columns.
    nx: i32,
    /// Rows.
    ny: i32,
    /// [`RANK_TOLERANCE_MM`] in cells.
    tolerance: i32,
}

impl Grid {
    fn new(profile: &PrinterProfile, params: &LayoutParams) -> Result<Self, LayoutError> {
        let (gap, margin) = (params.gap_mm, params.margin_mm);
        if !(gap.is_finite() && gap >= 0.0 && margin.is_finite() && margin >= 0.0) {
            return Err(LayoutError::InvalidParams(format!(
                "gap {gap} mm and margin {margin} mm must be finite and not negative"
            )));
        }
        let cell = f64::from(CELL_MM);
        let margin = f64::from(margin);
        let cells = |build: f32| ((f64::from(build) - 2.0 * margin) / cell).floor();
        let (nx, ny) = (cells(profile.build_mm[0]), cells(profile.build_mm[1]));
        if !(nx >= 1.0 && ny >= 1.0 && nx < f64::from(i32::MAX) && ny < f64::from(i32::MAX)) {
            return Err(LayoutError::InvalidParams(format!(
                "a margin of {margin} mm leaves no room on a {:?} mm plate",
                profile.build_mm
            )));
        }
        Ok(Self {
            origin: DVec2::splat(margin),
            cell,
            nx: nx as i32,
            ny: ny as i32,
            tolerance: (f64::from(RANK_TOLERANCE_MM) / cell).round() as i32,
        })
    }

    /// Room inside the margins, mm.
    fn usable_mm(&self) -> [f32; 2] {
        [
            (f64::from(self.nx) * self.cell) as f32,
            (f64::from(self.ny) * self.cell) as f32,
        ]
    }
}

/// Inclusive column range of one grid row.
type Span = (i32, i32);

/// A footprint at one quarter turn, rasterised relative to the bottom-left
/// corner of its bounding box.
struct Turned {
    /// Bottom-left corner of the turned hull's bounding box, mm.
    min: DVec2,
    /// Cells the hull touches: `(row, span)`, rows `0..rows`, columns from 0.
    cells: Vec<(i32, Span)>,
    /// Columns covered.
    cols: i32,
    /// Rows covered.
    rows: i32,
    /// Cells blocked once placed (the hull grown by the gap); rows and
    /// columns may be negative.
    keep: Vec<(i32, Span)>,
}

/// One footprint, ready to nest.
struct Part {
    turns: [Turned; 4],
    /// Height band (larger nests first).
    band: i64,
    /// Hull area, mm² (larger nests first within a band).
    area: f64,
}

impl Part {
    fn new(f: &Footprint, max_height: f32, gap: f32, grid: &Grid) -> Result<Self, LayoutError> {
        if f.hull.is_empty() || f.hull.iter().any(|p| !p.is_finite()) {
            return Err(LayoutError::InvalidFootprint(format!(
                "{}: the footprint needs at least one point, all finite",
                f.id
            )));
        }
        if !(f.height_mm.is_finite() && f.height_mm >= 0.0) {
            return Err(LayoutError::InvalidFootprint(format!(
                "{}: height {} mm",
                f.id, f.height_mm
            )));
        }
        if f.height_mm > max_height {
            return Err(LayoutError::TooTall {
                id: f.id.clone(),
                height_mm: f.height_mm,
                max_mm: max_height,
            });
        }
        let hull = convex_hull(f.hull.iter().map(|p| p.as_dvec2()).collect());
        let turns = [0, 1, 2, 3].map(|k| Turned::new(&hull, k, f64::from(gap), grid.cell));
        if !turns.iter().any(|t| t.cols <= grid.nx && t.rows <= grid.ny) {
            let (lo, hi) = bounds(&hull);
            let size = hi - lo;
            return Err(LayoutError::TooBig {
                id: f.id.clone(),
                size_mm: [size.x as f32, size.y as f32],
                usable_mm: grid.usable_mm(),
            });
        }
        Ok(Self {
            turns,
            band: (f.height_mm / HEIGHT_BAND_MM).floor() as i64,
            area: polygon_area(&hull),
        })
    }
}

impl Turned {
    fn new(hull: &[DVec2], turns: u8, gap: f64, cell: f64) -> Self {
        let turned: Vec<DVec2> = hull.iter().map(|&p| quarter_turn_d(p, turns)).collect();
        let (min, _) = bounds(&turned);
        let local: Vec<DVec2> = turned.iter().map(|&p| p - min).collect();
        let cells = rasterise(&local, cell);
        let cols = cells.iter().map(|&(_, (_, c1))| c1 + 1).max().unwrap_or(1);
        let rows = cells.iter().map(|&(r, _)| r + 1).max().unwrap_or(1);
        let keep = if gap > 0.0 {
            rasterise(&grow(&local, gap), cell)
        } else {
            cells.clone()
        };
        Self {
            min,
            cells,
            cols,
            rows,
            keep,
        }
    }
}

/// The cells of one plate taken so far, row by row, as sorted, merged spans.
struct Occupancy {
    rows: Vec<Vec<Span>>,
    placements: Vec<Placement>,
}

impl Occupancy {
    fn new(grid: &Grid) -> Self {
        Self {
            rows: vec![Vec::new(); grid.ny as usize],
            placements: Vec::new(),
        }
    }

    /// Puts `part` at its best free position under `strategy`; `false` when
    /// it does not fit.
    fn place(&mut self, index: usize, part: &Part, grid: &Grid, strategy: &Strategy) -> bool {
        let Some((k, ox, oy)) = self.best_position(part, grid, strategy) else {
            return false;
        };
        let t = &part.turns[usize::from(k)];
        for &(r, (c0, c1)) in &t.keep {
            let row = oy + r;
            let (a, b) = ((ox + c0).max(0), (ox + c1).min(grid.nx - 1));
            if (0..grid.ny).contains(&row) && a <= b {
                insert_span(&mut self.rows[row as usize], (a, b));
            }
        }
        let corner = grid.origin + DVec2::new(f64::from(ox), f64::from(oy)) * grid.cell;
        self.placements.push(Placement {
            index,
            quarter_turns: k,
            offset: (corner - t.min).as_vec2(),
        });
        true
    }

    /// Where `part` goes under `strategy`: over the turns it allows, the
    /// free positions whose [`Rank`] key is within [`RANK_TOLERANCE_MM`] of
    /// the best one compete on the other axis (the leftmost, or for
    /// [`Rank::Leftmost`] the lowest), then on the key itself, then on the
    /// fewest turns. So a part never gives up the edge of its row (or
    /// column) to sink a fraction of a millimetre lower (or further left).
    /// `(turns, column, row)` of its bounding box corner.
    fn best_position(
        &self,
        part: &Part,
        grid: &Grid,
        strategy: &Strategy,
    ) -> Option<(u8, i32, i32)> {
        let tol = grid.tolerance;
        // (key, other axis, turns, column, row)
        let mut found: Vec<(i32, i32, u8, i32, i32)> = Vec::new();
        // the best key so far
        let mut floor: Option<i32> = None;
        for (k, t) in (0u8..).zip(&part.turns) {
            if strategy.turns & (1 << k) == 0 || t.cols > grid.nx || t.rows > grid.ny {
                continue;
            }
            let max_ox = grid.nx - t.cols;
            let mut first_row: Option<i32> = None;
            for oy in 0..=grid.ny - t.rows {
                match strategy.rank {
                    Rank::LowestTop | Rank::LowestRow => {
                        let key = match strategy.rank {
                            Rank::LowestTop => oy + t.rows,
                            _ => oy,
                        };
                        // higher rows only rank worse
                        if floor.is_some_and(|f| key > f + tol)
                            || first_row.is_some_and(|r| oy > r + tol)
                        {
                            break;
                        }
                        if let Some(ox) = self.leftmost(t, oy, max_ox) {
                            first_row.get_or_insert(oy);
                            floor = Some(floor.map_or(key, |f| f.min(key)));
                            found.push((key, ox, k, ox, oy));
                        }
                    }
                    Rank::Leftmost => {
                        let limit = floor.map_or(max_ox, |f| max_ox.min(f + tol));
                        if let Some(ox) = self.leftmost(t, oy, limit) {
                            floor = Some(floor.map_or(ox, |f| f.min(ox)));
                            found.push((ox, oy, k, ox, oy));
                        }
                    }
                }
            }
        }
        let floor = floor?;
        found
            .into_iter()
            .filter(|c| c.0 <= floor + tol)
            .min_by_key(|c| (c.1, c.0, c.2))
            .map(|(_, _, k, ox, oy)| (k, ox, oy))
    }

    /// Leftmost column ≤ `max_ox` where `t` at row `oy` touches no taken
    /// cell. On a conflict it jumps past the taken span, so each try moves
    /// right by at least one cell.
    fn leftmost(&self, t: &Turned, oy: i32, max_ox: i32) -> Option<i32> {
        let mut ox = 0;
        'retry: while ox <= max_ox {
            for &(r, (c0, c1)) in &t.cells {
                let row = &self.rows[(oy + r) as usize];
                let (a, b) = (ox + c0, ox + c1);
                let i = row.partition_point(|&(s, _)| s <= b);
                if i > 0 && row[i - 1].1 >= a {
                    ox = row[i - 1].1 - c0 + 1;
                    continue 'retry;
                }
            }
            return Some(ox);
        }
        None
    }
}

/// Adds `span` to a row of sorted, disjoint spans, merging it with the
/// spans it overlaps or touches.
fn insert_span(row: &mut Vec<Span>, (mut a, mut b): Span) {
    let i = row.partition_point(|&(_, e)| e < a - 1);
    let mut j = i;
    while j < row.len() && row[j].0 <= b + 1 {
        a = a.min(row[j].0);
        b = b.max(row[j].1);
        j += 1;
    }
    row.splice(i..j, [(a, b)]);
}

/// `p` turned `turns` × 90° counter-clockwise, exactly.
fn quarter_turn_d(p: DVec2, turns: u8) -> DVec2 {
    match turns % 4 {
        0 => p,
        1 => DVec2::new(-p.y, p.x),
        2 => -p,
        _ => DVec2::new(p.y, -p.x),
    }
}

/// Bounding box corners of a non-empty point set.
fn bounds(points: &[DVec2]) -> (DVec2, DVec2) {
    points.iter().fold(
        (DVec2::splat(f64::INFINITY), DVec2::splat(f64::NEG_INFINITY)),
        |(lo, hi), &p| (lo.min(p), hi.max(p)),
    )
}

/// Convex hull, counter-clockwise, without collinear points (Andrew's
/// monotone chain). One or two points for degenerate input.
pub(crate) fn convex_hull(mut points: Vec<DVec2>) -> Vec<DVec2> {
    points.sort_by(|a, b| a.x.total_cmp(&b.x).then(a.y.total_cmp(&b.y)));
    points.dedup();
    if points.len() <= 2 {
        return points;
    }
    // one chain of the monotone chain: left turns only
    let chain = |points: &mut dyn Iterator<Item = DVec2>| {
        let mut c: Vec<DVec2> = Vec::new();
        for p in points {
            while c.len() >= 2
                && (c[c.len() - 1] - c[c.len() - 2]).perp_dot(p - c[c.len() - 2]) <= 0.0
            {
                c.pop();
            }
            c.push(p);
        }
        // its last point is the first of the other chain
        c.pop();
        c
    };
    let mut hull = chain(&mut points.iter().copied());
    hull.extend(chain(&mut points.iter().rev().copied()));
    hull
}

/// Area of a simple polygon (absolute shoelace), mm².
fn polygon_area(poly: &[DVec2]) -> f64 {
    let n = poly.len();
    let twice: f64 = (0..n).map(|i| poly[i].perp_dot(poly[(i + 1) % n])).sum();
    0.5 * twice.abs()
}

/// First-quadrant vertices of the regular 16-gon whose edges touch the unit
/// circle, with the edges facing ±X and ±Y: (1, tan 11.25°), then the
/// vertices at 33.75°, 56.25° and 78.75° scaled by 1 / cos 11.25°. Exact
/// literals, so no platform's trigonometry is involved, and the extent
/// along each axis is exactly 1.
const GON_Q1: [(f64, f64); 4] = [
    (1.0, 0.198_912_367_379_658),
    (0.847_759_065_022_573_5, 0.566_454_497_350_521_4),
    (0.566_454_497_350_521_4, 0.847_759_065_022_573_5),
    (0.198_912_367_379_658, 1.0),
];

/// A convex polygon containing `poly` grown by `gap` in every direction:
/// its Minkowski sum with the 16-gon around the disc of radius `gap`.
fn grow(poly: &[DVec2], gap: f64) -> Vec<DVec2> {
    let gon: Vec<DVec2> = (0..4u8)
        .flat_map(|k| {
            GON_Q1
                .iter()
                .map(move |&(x, y)| quarter_turn_d(DVec2::new(x, y) * gap, k))
        })
        .collect();
    convex_hull(
        poly.iter()
            .flat_map(|&p| gon.iter().map(move |&d| p + d))
            .collect(),
    )
}

/// The cells of a convex polygon (or 1–2 points), as `(row, span)`: every
/// cell holding a point of it, point (x, y) being in cell
/// (⌊x / cell⌋, ⌊y / cell⌋). Rows are measured as closed strips, which can
/// add a cell on a cell boundary, never miss one.
fn rasterise(poly: &[DVec2], cell: f64) -> Vec<(i32, Span)> {
    let (lo, hi) = bounds(poly);
    let (r0, r1) = ((lo.y / cell).floor() as i32, (hi.y / cell).floor() as i32);
    (r0..=r1)
        .filter_map(|r| {
            let y0 = f64::from(r) * cell;
            strip_extent(poly, y0, y0 + cell)
                .map(|(a, b)| (r, ((a / cell).floor() as i32, (b / cell).floor() as i32)))
        })
        .collect()
}

/// X extent of a convex polygon (or 1–2 points) within the strip
/// y ∈ [y0, y1]: the vertices inside it and the edge crossings of its two
/// lines.
fn strip_extent(poly: &[DVec2], y0: f64, y1: f64) -> Option<(f64, f64)> {
    let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
    let n = poly.len();
    for i in 0..n {
        let a = poly[i];
        if (y0..=y1).contains(&a.y) {
            lo = lo.min(a.x);
            hi = hi.max(a.x);
        }
        let b = poly[(i + 1) % n];
        for y in [y0, y1] {
            if (a.y < y && b.y > y) || (a.y > y && b.y < y) {
                let x = a.x + (y - a.y) * (b.x - a.x) / (b.y - a.y);
                lo = lo.min(x);
                hi = hi.max(x);
            }
        }
    }
    (lo <= hi).then_some((lo, hi))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn saturn() -> PrinterProfile {
        PrinterProfile::saturn_4_ultra_16k()
    }

    fn rect(id: &str, w: f32, h: f32, height: f32) -> Footprint {
        Footprint {
            id: id.into(),
            hull: vec![
                Vec2::new(-w / 2.0, -h / 2.0),
                Vec2::new(w / 2.0, -h / 2.0),
                Vec2::new(w / 2.0, h / 2.0),
                Vec2::new(-w / 2.0, h / 2.0),
            ],
            height_mm: height,
        }
    }

    /// A horseshoe-like hull: a half disc of radius 30 on a 60 × 20 block,
    /// not symmetric about X, so its turns differ.
    fn arch(id: &str, height: f32) -> Footprint {
        let mut hull = vec![Vec2::new(-30.0, -20.0), Vec2::new(30.0, -20.0)];
        for i in 0..=12 {
            let a = std::f32::consts::PI * i as f32 / 12.0;
            hull.push(Vec2::new(30.0 * a.cos(), 30.0 * a.sin()));
        }
        Footprint {
            id: id.into(),
            hull,
            height_mm: height,
        }
    }

    fn placed_hull(f: &Footprint, p: &Placement) -> Vec<Vec2> {
        f.hull.iter().map(|&q| p.apply(q)).collect()
    }

    /// Separating-axis test for convex polygons (any orientation).
    fn intersect(a: &[Vec2], b: &[Vec2]) -> bool {
        for poly in [a, b] {
            for i in 0..poly.len() {
                let e = poly[(i + 1) % poly.len()] - poly[i];
                let axis = Vec2::new(-e.y, e.x);
                let proj = |s: &[Vec2]| {
                    s.iter()
                        .fold((f32::INFINITY, f32::NEG_INFINITY), |(lo, hi), p| {
                            let d = p.dot(axis);
                            (lo.min(d), hi.max(d))
                        })
                };
                let ((a0, a1), (b0, b1)) = (proj(a), proj(b));
                if a1 < b0 || b1 < a0 {
                    return false;
                }
            }
        }
        true
    }

    fn point_segment(p: Vec2, a: Vec2, b: Vec2) -> f32 {
        let ab = b - a;
        let t = ((p - a).dot(ab) / ab.length_squared()).clamp(0.0, 1.0);
        (a + ab * t).distance(p)
    }

    /// Distance between two disjoint convex polygons.
    fn distance(a: &[Vec2], b: &[Vec2]) -> f32 {
        let mut d = f32::INFINITY;
        for (p, q) in [(a, b), (b, a)] {
            for &v in p {
                for i in 0..q.len() {
                    d = d.min(point_segment(v, q[i], q[(i + 1) % q.len()]));
                }
            }
        }
        d
    }

    /// Every placement is inside the margins and at least the gap away from
    /// every other part of its plate; every part is placed once.
    fn check(plates: &[Plate], fps: &[Footprint], params: &LayoutParams) {
        let p = saturn();
        let (m, tol) = (params.margin_mm, 1e-3);
        let mut seen = vec![0; fps.len()];
        for plate in plates {
            assert!(!plate.parts.is_empty());
            let hulls: Vec<Vec<Vec2>> = plate
                .parts
                .iter()
                .map(|pl| {
                    seen[pl.index] += 1;
                    placed_hull(&fps[pl.index], pl)
                })
                .collect();
            for h in &hulls {
                for q in h {
                    assert!(q.x >= m - tol && q.x <= p.build_mm[0] - m + tol, "{q}");
                    assert!(q.y >= m - tol && q.y <= p.build_mm[1] - m + tol, "{q}");
                }
            }
            for i in 0..hulls.len() {
                for j in i + 1..hulls.len() {
                    assert!(
                        !intersect(&hulls[i], &hulls[j]),
                        "parts {i} and {j} overlap"
                    );
                    let d = distance(&hulls[i], &hulls[j]);
                    assert!(
                        d >= params.gap_mm - tol,
                        "parts {i} and {j} are {d} mm apart"
                    );
                }
            }
        }
        assert!(seen.iter().all(|&n| n == 1), "{seen:?}");
    }

    #[test]
    fn identical_footprints_fill_one_plate() {
        let params = LayoutParams::default();
        // 16 fit on a plain 4 × 4 grid without turning any of them
        let fps: Vec<Footprint> = (0..16)
            .map(|i| rect(&format!("r{i}"), 40.0, 25.0, 30.0))
            .collect();
        let plates = nest(&fps, &saturn(), &params).unwrap();
        assert_eq!(plates.len(), 1);
        check(&plates, &fps, &params);
        // ...and the turns pack more than the plain grid
        let more: Vec<Footprint> = (0..18)
            .map(|i| rect(&format!("r{i}"), 40.0, 25.0, 30.0))
            .collect();
        let plates = nest(&more, &saturn(), &params).unwrap();
        assert_eq!(plates.len(), 1);
        assert!(plates[0].parts.iter().any(|p| p.quarter_turns % 2 == 1));
        check(&plates, &more, &params);
    }

    #[test]
    fn arches_keep_the_gap_and_the_margins() {
        let params = LayoutParams {
            gap_mm: 3.0,
            margin_mm: 5.0,
        };
        let fps: Vec<Footprint> = (0..9).map(|i| arch(&format!("a{i}"), 40.0)).collect();
        let plates = nest(&fps, &saturn(), &params).unwrap();
        check(&plates, &fps, &params);
        // a plain 3 × 2 grid fits 6: the fill must not do worse
        assert!(plates[0].parts.len() >= 6, "{}", plates[0].parts.len());
    }

    #[test]
    fn horseshoes_fill_a_plate_like_a_grid() {
        // 64 × 52 mm horseshoe hulls: 3 per row and 2 rows fit, while the
        // greedy fill alone nests them into each other and keeps only 5
        let shoe = |i: usize| {
            let mut hull = vec![Vec2::new(-32.0, -22.0), Vec2::new(32.0, -22.0)];
            for k in 0..=198 {
                let a = std::f32::consts::PI * k as f32 / 198.0;
                hull.push(Vec2::new(32.0 * a.cos(), 30.0 * a.sin()));
            }
            Footprint {
                id: format!("shoe{i}"),
                hull,
                height_mm: 30.0,
            }
        };
        let params = LayoutParams::default();
        let fps: Vec<Footprint> = (0..12).map(shoe).collect();
        let plates = nest(&fps, &saturn(), &params).unwrap();
        assert_eq!(plates.len(), 2);
        assert_eq!(plates[0].parts.len(), 6);
        check(&plates, &fps, &params);
    }

    #[test]
    fn overflow_opens_a_second_plate() {
        let params = LayoutParams::default();
        let one = nest(&[rect("r", 60.0, 50.0, 30.0)], &saturn(), &params).unwrap();
        assert_eq!(one.len(), 1);
        let fps: Vec<Footprint> = (0..8)
            .map(|i| rect(&format!("r{i}"), 60.0, 50.0, 30.0))
            .collect();
        let plates = nest(&fps, &saturn(), &params).unwrap();
        // 3 per row and 2 rows fit inside 203.68 × 110.37 mm; 8 need 2 plates
        assert_eq!(plates.len(), 2);
        assert_eq!(plates[0].parts.len(), 6);
        assert_eq!(plates[1].parts.len(), 2);
        check(&plates, &fps, &params);
    }

    #[test]
    fn tall_parts_share_the_first_plate() {
        let params = LayoutParams::default();
        let mut fps = Vec::new();
        for i in 0..6 {
            fps.push(rect(&format!("short{i}"), 60.0, 50.0, 20.0));
            fps.push(rect(&format!("tall{i}"), 60.0, 50.0, 45.0));
        }
        let plates = nest(&fps, &saturn(), &params).unwrap();
        assert_eq!(plates.len(), 2);
        for p in &plates[0].parts {
            assert!(fps[p.index].id.starts_with("tall"), "{}", fps[p.index].id);
        }
        check(&plates, &fps, &params);
    }

    #[test]
    fn a_part_that_fits_only_turned_is_turned() {
        let params = LayoutParams::default();
        // 150 mm only fits along X (203.68 mm), not along Y (110.37 mm)
        let fps = [rect("long", 30.0, 150.0, 10.0)];
        let plates = nest(&fps, &saturn(), &params).unwrap();
        assert_eq!(plates[0].parts[0].quarter_turns % 2, 1);
        check(&plates, &fps, &params);
    }

    #[test]
    fn too_big_and_too_tall_are_errors() {
        let params = LayoutParams::default();
        let big = [
            rect("ok", 10.0, 10.0, 10.0),
            rect("huge", 120.0, 120.0, 10.0),
        ];
        match nest(&big, &saturn(), &params) {
            Err(e @ LayoutError::TooBig { .. }) => {
                assert_eq!(e.code(), "print-part-too-big");
                assert!(e.to_string().contains("huge"));
            }
            other => panic!("{other:?}"),
        }
        let tall = [rect("tower", 10.0, 10.0, 221.0)];
        match nest(&tall, &saturn(), &params) {
            Err(e @ LayoutError::TooTall { .. }) => assert_eq!(e.code(), "print-part-too-tall"),
            other => panic!("{other:?}"),
        }
        // exactly the usable room still fits: 203.68 → 814 cells of 0.25 mm,
        // and a 203 mm part touches 813 of them
        assert!(nest(&[rect("wide", 203.0, 20.0, 10.0)], &saturn(), &params).is_ok());
    }

    #[test]
    fn invalid_input_is_refused() {
        let p = saturn();
        let ok = LayoutParams::default();
        let bad = Footprint {
            id: "nan".into(),
            hull: vec![Vec2::new(f32::NAN, 0.0)],
            height_mm: 1.0,
        };
        assert_eq!(
            nest(&[bad], &p, &ok).unwrap_err().code(),
            "print-invalid-footprint"
        );
        let empty = Footprint {
            id: "empty".into(),
            hull: vec![],
            height_mm: 1.0,
        };
        assert!(nest(&[empty], &p, &ok).is_err());
        for params in [
            LayoutParams {
                gap_mm: -1.0,
                margin_mm: 4.0,
            },
            LayoutParams {
                gap_mm: 3.0,
                margin_mm: 60.0,
            },
        ] {
            assert_eq!(
                nest(&[rect("r", 1.0, 1.0, 1.0)], &p, &params)
                    .unwrap_err()
                    .code(),
                "print-invalid-layout"
            );
        }
        assert!(nest(&[], &p, &ok).unwrap().is_empty());
    }

    #[test]
    fn nesting_is_deterministic() {
        let params = LayoutParams::default();
        let mut fps: Vec<Footprint> = (0..10)
            .map(|i| arch(&format!("a{i}"), 30.0 + i as f32))
            .collect();
        fps.extend((0..10).map(|i| rect(&format!("r{i}"), 20.0 + i as f32, 15.0, 25.0)));
        let a = nest(&fps, &saturn(), &params).unwrap();
        let b = nest(&fps.clone(), &saturn(), &params).unwrap();
        assert_eq!(a, b);
        check(&a, &fps, &params);
    }

    #[test]
    fn point_and_segment_footprints_nest() {
        let params = LayoutParams::default();
        let fps = [
            Footprint {
                id: "point".into(),
                hull: vec![Vec2::ZERO],
                height_mm: 1.0,
            },
            Footprint {
                id: "segment".into(),
                hull: vec![Vec2::new(-5.0, 0.0), Vec2::new(5.0, 0.0)],
                height_mm: 1.0,
            },
        ];
        let plates = nest(&fps, &saturn(), &params).unwrap();
        assert_eq!(plates.len(), 1);
        assert_eq!(plates[0].parts.len(), 2);
    }

    #[test]
    fn hull_and_spans_helpers() {
        let pts = [
            (0.0, 0.0),
            (2.0, 0.0),
            (1.0, 1.0),
            (2.0, 2.0),
            (0.0, 2.0),
            (1.0, 0.0),
        ]
        .map(|(x, y)| DVec2::new(x, y));
        let hull = convex_hull(pts.to_vec());
        assert_eq!(hull.len(), 4);
        assert_eq!(polygon_area(&hull), 4.0);
        let mut row = vec![(0, 3), (10, 12)];
        insert_span(&mut row, (4, 6));
        assert_eq!(row, vec![(0, 6), (10, 12)]);
        insert_span(&mut row, (7, 9));
        assert_eq!(row, vec![(0, 12)]);
        insert_span(&mut row, (20, 21));
        assert_eq!(row, vec![(0, 12), (20, 21)]);
        // a 1 × 1 mm square touches 5 × 5 cells of 0.25 mm (closed squares)
        let sq = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)].map(|(x, y)| DVec2::new(x, y));
        let cells = rasterise(&sq, 0.25);
        assert_eq!(cells.len(), 5);
        assert!(cells.iter().all(|&(_, s)| s == (0, 4)));
        assert_eq!(quarter_turn(Vec2::new(1.0, 2.0), 1), Vec2::new(-2.0, 1.0));
        assert_eq!(quarter_turn(Vec2::new(1.0, 2.0), 3), Vec2::new(2.0, -1.0));
    }
}
