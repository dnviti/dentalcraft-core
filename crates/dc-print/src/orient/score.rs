//! Scoring of one candidate orientation (print plan §4).
//!
//! The terms, all in plate space after the candidate rotation. A face
//! counts as facing down when its normal is within the self-supporting
//! angle plus [`MARGIN_DEG`] of straight down. The margin keeps the search
//! from relying on a face standing right at the limit, where rounding would
//! decide (a vertical wall or a flat bottom tilted exactly 45°).
//! - (a) fitting area facing down: it would sag or need supports on the
//!   intaglio, so it is weighted heavily;
//! - (b) outer area facing down: what the supports must hold;
//! - (c) height (layers, time), with a small weight;
//! - (d) largest horizontal cross-section (peel force, large flat
//!   islands), with a small weight.
//!
//! Each cross-section is exact for its plane. It is the shoelace sum of the
//! slice segments, each oriented by its face's outward normal, so overlaps
//! and holes count correctly.

use dc_mesh::TriMesh;
use glam::{DVec2, Quat, Vec3};

use super::sin_cos;

/// Margin on the self-supporting angle in the score, degrees.
const MARGIN_DEG: f64 = 5.0;

/// Weight of (a), per mm²: one mm² of intaglio needing a support costs as
/// much as 100 mm² of outer support area.
const W_FITTING_DOWN: f64 = 100.0;
/// Weight of (b), per mm².
const W_SUPPORT: f64 = 1.0;
/// Weight of (c), per mm of height.
const W_HEIGHT: f64 = 4.0;
/// Weight of (d), per mm² of the largest cross-section.
const W_SECTION: f64 = 1.0;
/// Spacing of the cross-section planes, mm.
const SECTION_STEP_MM: f32 = 0.5;

/// The terms of one candidate and their weighted total (lower is better).
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct Score {
    /// (a) Fitting area facing down, mm².
    pub fitting_down_mm2: f64,
    /// (b) Outer area facing down, mm².
    pub support_mm2: f64,
    /// (c) Height, mm.
    pub height_mm: f64,
    /// (d) Largest horizontal cross-section, mm².
    pub max_section_mm2: f64,
    /// Weighted sum.
    pub total: f64,
}

/// The z below which a unit face normal counts as facing down in the
/// score: −cos(self-supporting angle + [`MARGIN_DEG`]).
pub(crate) fn down_z(overhang_deg: f32) -> f32 {
    -sin_cos((f64::from(overhang_deg) + MARGIN_DEG).to_radians()).1 as f32
}

/// Scores `mesh` rotated by `rotation`; faces whose unit normal has
/// z < `down_z` face down.
pub(crate) fn score(mesh: &TriMesh, fitting: &[bool], rotation: Quat, down_z: f32) -> Score {
    let pos: Vec<Vec3> = mesh.positions.iter().map(|p| rotation * *p).collect();
    let (mut zmin, mut zmax) = (f32::MAX, f32::MIN);
    for p in &pos {
        zmin = zmin.min(p.z);
        zmax = zmax.max(p.z);
    }
    if pos.is_empty() {
        return Score {
            fitting_down_mm2: 0.0,
            support_mm2: 0.0,
            height_mm: 0.0,
            max_section_mm2: 0.0,
            total: 0.0,
        };
    }
    let (mut fitting_down, mut support) = (0.0f64, 0.0f64);
    for (f, t) in mesh.tris.iter().enumerate() {
        let [a, b, c] = t.map(|i| pos[i as usize]);
        let raw = (b - a).cross(c - a);
        let len = raw.length();
        if len == 0.0 || raw.z >= down_z * len {
            continue;
        }
        let area = 0.5 * f64::from(len);
        if fitting.get(f).copied().unwrap_or(false) {
            fitting_down += area;
        } else {
            support += area;
        }
    }
    let height = zmax - zmin;
    let planes = ((height / SECTION_STEP_MM).ceil() as usize).max(1);
    let max_section = section_areas(&pos, &mesh.tris, zmin, SECTION_STEP_MM, planes)
        .into_iter()
        .fold(0.0f64, f64::max);
    let height = f64::from(height);
    Score {
        fitting_down_mm2: fitting_down,
        support_mm2: support,
        height_mm: height,
        max_section_mm2: max_section,
        total: W_FITTING_DOWN * fitting_down
            + W_SUPPORT * support
            + W_HEIGHT * height
            + W_SECTION * max_section,
    }
}

/// Areas of the horizontal cross-sections of a closed, outward-oriented
/// mesh at z = `z0` + (k + ½)·`step`, k < `count`.
pub(crate) fn section_areas(
    pos: &[Vec3],
    tris: &[[u32; 3]],
    z0: f32,
    step: f32,
    count: usize,
) -> Vec<f64> {
    let mut acc = vec![0.0f64; count];
    // Coordinates relative to a centre keep the shoelace terms small.
    let centre = pos.iter().fold(Vec3::ZERO, |s, p| s + *p) / pos.len().max(1) as f32;
    for t in tris {
        let [a, b, c] = t.map(|i| pos[i as usize] - Vec3::new(centre.x, centre.y, 0.0));
        let lo = a.z.min(b.z).min(c.z);
        let hi = a.z.max(b.z).max(c.z);
        let first = ((lo - z0) / step - 0.5).ceil().max(0.0) as usize;
        let last = ((hi - z0) / step - 0.5).floor();
        if last < 0.0 {
            continue;
        }
        let last = (last as usize).min(count.saturating_sub(1));
        let n = (b - a).cross(c - a);
        // Slice segments run with the material on their left: along Z × n.
        let along = DVec2::new(-f64::from(n.y), f64::from(n.x));
        for (k, area) in acc.iter_mut().enumerate().take(last + 1).skip(first) {
            let z = z0 + (k as f32 + 0.5) * step;
            let mut ends = [DVec2::ZERO; 2];
            let mut m = 0;
            for (p, q) in [(a, b), (b, c), (c, a)] {
                if (p.z < z) != (q.z < z) && m < 2 {
                    let s = f64::from(z - p.z) / f64::from(q.z - p.z);
                    let (p2, q2) = (p.truncate().as_dvec2(), q.truncate().as_dvec2());
                    ends[m] = p2 + (q2 - p2) * s;
                    m += 1;
                }
            }
            if m < 2 {
                continue;
            }
            let (mut p, mut q) = (ends[0], ends[1]);
            if (q - p).dot(along) < 0.0 {
                std::mem::swap(&mut p, &mut q);
            }
            *area += 0.5 * p.perp_dot(q);
        }
    }
    acc
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Closed box [0, sx] × [0, sy] × [0, sz], outward.
    fn cuboid(sx: f32, sy: f32, sz: f32) -> TriMesh {
        let p = |x: f32, y: f32, z: f32| Vec3::new(x * sx, y * sy, z * sz);
        let positions = vec![
            p(0.0, 0.0, 0.0),
            p(1.0, 0.0, 0.0),
            p(1.0, 1.0, 0.0),
            p(0.0, 1.0, 0.0),
            p(0.0, 0.0, 1.0),
            p(1.0, 0.0, 1.0),
            p(1.0, 1.0, 1.0),
            p(0.0, 1.0, 1.0),
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
        TriMesh::new(positions, tris)
    }

    #[test]
    fn box_sections_are_exact() {
        let mesh = cuboid(20.0, 10.0, 5.0);
        let areas = section_areas(&mesh.positions, &mesh.tris, 0.0, 0.5, 10);
        for a in areas {
            assert!((a - 200.0).abs() < 1e-6, "{a}");
        }
    }

    #[test]
    fn box_score_terms() {
        let mesh = cuboid(20.0, 10.0, 5.0);
        let down = down_z(45.0);
        let s = score(&mesh, &[false; 12], Quat::IDENTITY, down);
        assert!(
            (s.support_mm2 - 200.0).abs() < 1e-3,
            "bottom face needs support: {s:?}"
        );
        assert_eq!(s.fitting_down_mm2, 0.0);
        assert!((s.height_mm - 5.0).abs() < 1e-6);
        assert!((s.max_section_mm2 - 200.0).abs() < 1e-3);
        let flags: Vec<bool> = (0..12).map(|f| f < 2).collect();
        let s = score(&mesh, &flags, Quat::IDENTITY, down);
        assert!((s.fitting_down_mm2 - 200.0).abs() < 1e-3 && s.support_mm2 == 0.0);
    }
}
