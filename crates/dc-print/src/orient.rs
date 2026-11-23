//! Automatic orientation of aligner shells (print plan §4). Owner: OrientSupport.
//!
//! The search:
//! 1. **Base pose.** The cavity opening (the area-weighted mean normal of
//!    the fitting faces) points straight up, away from the plate. The
//!    occlusal side faces the plate and carries the supports, and resin
//!    drains from the cavity.
//! 2. **Tilts.** Tilts of up to [`OrientParams::max_tilt_deg`] in
//!    [`OrientParams::tilt_step_deg`] steps, about eight horizontal axes
//!    (X, Y and the diagonals). A tilt cuts the wall obliquely, which
//!    shrinks the per-layer cross-section and the peel force, and avoids
//!    large flat islands.
//! 3. **Score** (module `score`). Fitting area facing down is penalised
//!    heavily, then outer area needing support, then, with small weights,
//!    height and the largest cross-section. Ties go to the earlier
//!    candidate.
//! 4. **Yaw.** A turn about Z lays the footprint's smallest enclosing
//!    rectangle with its long side along the plate's X axis, for nesting.
//!    It changes none of the scored terms.
//! 5. **Fitting-side islands.** Candidates are tried best score first,
//!    sliced at the job's layer height ([`OrientParams::layer_um`]): the
//!    first that starts no island on the fitting side wins. Such an island
//!    (the counter of an engraved letter facing down, for one) could never
//!    get a strut. With none free, the one with the fewest is kept and the
//!    hard island check names the part. Only the layers where a fitting
//!    vertex starts the solid are sliced: an island starts at such a vertex
//!    or not at all.
//! 6. **Wide search** ([`wide_poses`]). When the chosen pose fails the
//!    job's hard checks, the whole sphere of directions is searched:
//!    every direction of the opening on a [`WIDE_STEP_DEG`] grid (the
//!    tilts of step 2 and the flipped poses included). A direction
//!    qualifies when it starts no island on the fitting side (counted
//!    before the turn of step 4, which changes no height); with none on
//!    the grid, the [`REFINE_SEEDS`] directions with the fewest such
//!    islands are refined on grids of half and a quarter of the step. The
//!    qualifying directions are turned as in step 4 and ranked as step 3
//!    ranks, the poses whose islands all start away from the fitting side
//!    (farther than [`crate::support::PLACEMENT_CLEARANCE_MM`], where a
//!    strut may stand) first; the job tries at most [`WIDE_POSES`] of them
//!    in turn. A part whose first pose passes costs nothing more.
//!
//! The fitting side of an exported shell comes from its stage model
//! ([`fitting_from_model`]); [`fitting_faces`] guesses it from the geometry
//! alone for other parts.
//!
//! The orientation must not change from one platform to another. Trigonometry
//! therefore goes through [`sin_cos`] (basic IEEE operations only) or
//! through vectors. Candidates are scored in parallel and collected in
//! order, and every ranking breaks ties by candidate order.

mod fitting;
mod hull;
mod model;
pub(crate) mod raycast;
mod score;

pub use model::{
    ExportModel, MIN_NEAR_SHARE, MODEL_DIR, MODEL_TOL_MM, ModelMismatch, SHELL_DIR, export_model,
    fitting_from_model,
};

use crate::part::{Orientation, PrintPart, rotate_mesh, translate_mesh};
use crate::support::islands;
use dc_mesh::TriMesh;
use glam::{DVec3, Quat, Vec2, Vec3};
use rayon::prelude::*;

/// Orientation parameters.
#[derive(Debug, Clone, PartialEq)]
pub struct OrientParams {
    /// Height of the part's lowest point above the plate, mm: the support
    /// height, raft included.
    pub lift_mm: f32,
    /// Largest tilt away from "opening straight up", degrees.
    pub max_tilt_deg: f32,
    /// Tilt step of the search, degrees (0: no tilt).
    pub tilt_step_deg: f32,
    /// Self-supporting angle, degrees. A face needs support when its normal
    /// points down within this angle of straight down
    /// (normal.z < −cos(angle)).
    pub overhang_deg: f32,
    /// Layer height of the print, µm (the job's): an orientation that
    /// leaves an island starting on the fitting side at this layer height
    /// is refused, as such an island can never get a strut.
    pub layer_um: u32,
}

impl Default for OrientParams {
    fn default() -> Self {
        Self {
            lift_mm: 5.0,
            max_tilt_deg: 45.0,
            tilt_step_deg: 15.0,
            overhang_deg: 45.0,
            layer_um: 50,
        }
    }
}

/// Per-face fitting-side flags computed from the geometry: `true` for the
/// faces of the intaglio of a cup-shaped shell. Each face's enclosure
/// (rays over its outer hemisphere) is compared with that of the face
/// facing it across the wall; the cavity side is the more enclosed one.
pub fn fitting_faces(mesh: &TriMesh) -> Vec<bool> {
    fitting::detect(mesh)
}

/// Chooses the orientation of one part: its fitting flags, or computed
/// ones, then [`orient_with`].
pub fn orient(part: &PrintPart, params: &OrientParams) -> Orientation {
    match &part.fitting {
        Some(fitting) => orient_with(&part.mesh, fitting, params),
        None => orient_with(&part.mesh, &fitting_faces(&part.mesh), params),
    }
}

/// Chooses the orientation of `mesh`, whose fitting faces are flagged in
/// `fitting` (per face, as in [`PrintPart::fitting`]): the best-scored
/// candidate whose slices at [`OrientParams::layer_um`] start no island on
/// the fitting side; when every candidate does, the one with the fewest
/// such islands (the hard island check then names the part).
pub fn orient_with(mesh: &TriMesh, fitting: &[bool], params: &OrientParams) -> Orientation {
    let lift_mm = params.lift_mm;
    if mesh.is_empty() {
        return Orientation {
            rotation: Quat::IDENTITY,
            lift_mm,
        };
    }
    let base = opening_up(mesh, fitting);
    let down_z = score::down_z(params.overhang_deg);
    let tilts = tilts(params);
    let scores: Vec<f64> = tilts
        .par_iter()
        .map(|t| score::score(mesh, fitting, *t * base, down_z).total)
        .collect();
    let mut order: Vec<usize> = (0..scores.len()).collect();
    order.sort_by(|&a, &b| scores[a].total_cmp(&scores[b]).then(a.cmp(&b)));
    let on_fitting = islands::vertex_touches(mesh, fitting);
    let mut fallback: Option<(usize, Orientation)> = None;
    for &i in &order {
        let o = pose(mesh, (tilts[i] * base).normalize(), lift_mm);
        let n = fitting_islands(&oriented_mesh(mesh, &o), &on_fitting, params.layer_um);
        if n == 0 {
            return o;
        }
        if fallback.as_ref().is_none_or(|(least, _)| n < *least) {
            fallback = Some((n, o));
        }
    }
    fallback.map_or(
        Orientation {
            rotation: base,
            lift_mm,
        },
        |(_, o)| o,
    )
}

/// Grid step of the wide search ([`wide_poses`]), degrees.
pub const WIDE_STEP_DEG: f64 = 5.0;
/// Directions refined when no direction of the grid qualifies.
pub const REFINE_SEEDS: usize = 4;
/// Refinement grids: half the step, then a quarter.
const REFINE_LEVELS: u32 = 2;
/// Poses of the wide search the job tries at most.
pub const WIDE_POSES: usize = 16;

/// A direction of the opening: its tilt from straight up and the azimuth
/// of the horizontal axis it is tilted about, degrees.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Dir {
    tilt_deg: f64,
    axis_deg: f64,
}

impl Dir {
    fn quat(self) -> Quat {
        tilt_quat(self.tilt_deg.to_radians(), self.axis_deg.to_radians())
    }
}

/// Islands of the oriented part alone, by where they start.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, PartialOrd, Ord)]
struct Starts {
    /// On a vertex of a fitting face: never a strut.
    fitting: usize,
    /// On another vertex within [`crate::support::PLACEMENT_CLEARANCE_MM`]
    /// of a fitting face: a strut only beside the start, if at all.
    near: usize,
}

/// The poses the job tries when the pose of [`orient_with`] fails its hard
/// checks (module documentation, step 6), best first, at most
/// [`WIDE_POSES`], none of `tried`. Empty when no direction, refined or
/// not, starts no island on the fitting side.
pub fn wide_poses(
    mesh: &TriMesh,
    fitting: &[bool],
    params: &OrientParams,
    tried: &[Orientation],
) -> Vec<Orientation> {
    if mesh.is_empty() {
        return Vec::new();
    }
    let base = opening_up(mesh, fitting);
    let on_fitting = islands::vertex_touches(mesh, fitting);
    let near = near_fitting(mesh, fitting, &on_fitting);
    // The turn of step 4 changes no height: the islands are counted before
    // it, and only the qualifying poses are turned.
    let eval = |d: &Dir| {
        let rotation = (d.quat() * base).normalize();
        let o = Orientation {
            rotation,
            lift_mm: params.lift_mm,
        };
        let starts = island_starts(
            &oriented_mesh(mesh, &o),
            &on_fitting,
            Some(&near),
            params.layer_um,
        );
        (rotation, starts)
    };

    // the tilts of step 2, then the grid over the whole sphere
    let mut dirs = tilt_dirs(params);
    dirs.extend(sphere(WIDE_STEP_DEG));
    let mut found: Vec<(Quat, Starts)> = dirs.par_iter().map(eval).collect();
    for level in 1..=REFINE_LEVELS {
        if found.iter().any(|(_, s)| s.fitting == 0) {
            break;
        }
        // the best directions so far, and the grid of the level around them
        let mut best: Vec<usize> = (0..found.len()).collect();
        best.sort_by_key(|&i| (found[i].1, i));
        let step = WIDE_STEP_DEG / f64::from(1u32 << level);
        let around: Vec<Dir> = best
            .iter()
            .take(REFINE_SEEDS)
            .flat_map(|&i| neighbours(dirs[i], step))
            .collect();
        found.extend(around.par_iter().map(eval).collect::<Vec<_>>());
        dirs.extend(around);
    }

    // the qualifying poses, turned, each once, ranked
    let qualifying: Vec<usize> = (0..found.len())
        .filter(|&i| found[i].1.fitting == 0)
        .collect();
    let turned: Vec<Orientation> = qualifying
        .par_iter()
        .map(|&i| pose(mesh, found[i].0, params.lift_mm))
        .collect();
    let mut keep: Vec<(Orientation, bool)> = Vec::new();
    for (o, &i) in turned.iter().zip(&qualifying) {
        if !tried.contains(o) && !keep.iter().any(|(k, _)| k == o) {
            keep.push((*o, found[i].1.near > 0));
        }
    }
    let down_z = score::down_z(params.overhang_deg);
    let scores: Vec<f64> = keep
        .par_iter()
        .map(|(o, _)| score::score(mesh, fitting, o.rotation, down_z).total)
        .collect();
    let mut order: Vec<usize> = (0..keep.len()).collect();
    order.sort_by(|&a, &b| {
        keep[a]
            .1
            .cmp(&keep[b].1)
            .then(scores[a].total_cmp(&scores[b]))
            .then(a.cmp(&b))
    });
    order
        .into_iter()
        .take(WIDE_POSES)
        .map(|k| keep[k].0)
        .collect()
}

/// The pose of `rotation` (module documentation, step 4): turned about Z
/// so that the footprint's long side runs along X.
fn pose(mesh: &TriMesh, rotation: Quat, lift_mm: f32) -> Orientation {
    let footprint: Vec<Vec2> = mesh
        .positions
        .iter()
        .map(|p| (rotation * *p).truncate())
        .collect();
    let turn = hull::long_side_turn(&footprint);
    Orientation {
        rotation: (turn * rotation).normalize(),
        lift_mm,
    }
}

/// Islands of the oriented part alone that start on a vertex of a fitting
/// face (`on_fitting`, per vertex).
fn fitting_islands(oriented: &TriMesh, on_fitting: &[bool], layer_um: u32) -> usize {
    island_starts(oriented, on_fitting, None, layer_um).fitting
}

/// The islands of the oriented part alone that start on a vertex of a
/// fitting face (`on_fitting`) or, when `near` is given, on another vertex
/// it flags. Only the layers where such a vertex starts the solid are
/// sliced: an island starts at a vertex where the solid starts
/// ([`islands::island_bottom`]).
fn island_starts(
    oriented: &TriMesh,
    on_fitting: &[bool],
    near: Option<&[bool]>,
    layer_um: u32,
) -> Starts {
    let watched = near.unwrap_or(on_fitting);
    let minima = islands::material_minima(oriented);
    let starts: Vec<u32> = minima
        .iter()
        .copied()
        .filter(|&v| watched[v as usize])
        .collect();
    let mut out = Starts::default();
    if starts.is_empty() {
        return out;
    }
    let layers = islands::layers_of(oriented, &starts, layer_um);
    for island in islands::islands_at(&[oriented], layer_um, &layers) {
        match islands::island_bottom(oriented, &minima, &island, layer_um) {
            Some(v) if on_fitting[v as usize] => out.fitting += 1,
            Some(v) if watched[v as usize] => out.near += 1,
            _ => {}
        }
    }
    out
}

/// Per vertex: on a fitting face, or within
/// [`crate::support::PLACEMENT_CLEARANCE_MM`] of one (no strut stands
/// there). The same in every pose.
fn near_fitting(mesh: &TriMesh, fitting: &[bool], on_fitting: &[bool]) -> Vec<bool> {
    let fit = raycast::Bvh::with_faces(
        mesh,
        (0..mesh.face_count().min(fitting.len()))
            .filter(|&f| fitting[f])
            .map(|f| f as u32),
    );
    mesh.positions
        .par_iter()
        .zip(on_fitting)
        .map(|(p, &on)| {
            on || fit
                .nearest(*p, crate::support::PLACEMENT_CLEARANCE_MM)
                .is_some()
        })
        .collect()
}

/// The part in plate space: rotated by `o.rotation` about the model
/// origin, lifted so its lowest point sits at `o.lift_mm`, and centred on
/// the origin in XY (bounding box centre).
pub fn oriented_mesh(mesh: &TriMesh, o: &Orientation) -> TriMesh {
    let mut out = mesh.clone();
    rotate_mesh(&mut out, o.rotation);
    if let Some(bb) = out.aabb() {
        let c = bb.center();
        translate_mesh(&mut out, Vec3::new(-c.x, -c.y, o.lift_mm - bb.min.z));
    }
    out
}

/// Rotation that turns the cavity opening, the area-weighted mean normal
/// of the fitting faces, straight up. Identity without fitting faces.
fn opening_up(mesh: &TriMesh, fitting: &[bool]) -> Quat {
    let (mut sum, mut area) = (DVec3::ZERO, 0.0f64);
    for f in 0..mesh.face_count() {
        let raw = mesh.face_normal_raw(f).as_dvec3();
        area += raw.length();
        if fitting.get(f).copied().unwrap_or(false) {
            sum += raw;
        }
    }
    if sum.length() <= 1e-3 * area {
        return Quat::IDENTITY;
    }
    Quat::from_rotation_arc(sum.normalize().as_vec3(), Vec3::Z)
}

/// A tilt of `tilt` radians about the horizontal axis at azimuth `axis`
/// radians.
fn tilt_quat(tilt: f64, axis: f64) -> Quat {
    let (sh, ch) = sin_cos(tilt / 2.0);
    let (sa, ca) = sin_cos(axis);
    Quat::from_xyzw((ca * sh) as f32, (sa * sh) as f32, 0.0, ch as f32)
}

/// The candidate tilts: none, then each step up to the largest tilt about
/// eight horizontal axes (X, the diagonals, Y and so on, 45° apart).
fn tilts(params: &OrientParams) -> Vec<Quat> {
    let mut out = vec![Quat::IDENTITY];
    let (step, max) = (params.tilt_step_deg, params.max_tilt_deg);
    if !(step > 0.0 && max > 0.0) {
        return out;
    }
    let steps = ((max / step).ceil() as usize).min(90);
    for s in 1..=steps {
        let tilt = f64::from((s as f32 * step).min(max)).to_radians();
        for k in 0..8 {
            out.push(tilt_quat(tilt, f64::from(k) * std::f64::consts::FRAC_PI_4));
        }
    }
    out
}

/// The tilts of [`tilts`] as directions.
fn tilt_dirs(params: &OrientParams) -> Vec<Dir> {
    let mut out = vec![Dir {
        tilt_deg: 0.0,
        axis_deg: 0.0,
    }];
    let (step, max) = (params.tilt_step_deg, params.max_tilt_deg);
    if !(step > 0.0 && max > 0.0) {
        return out;
    }
    let steps = ((max / step).ceil() as usize).min(90);
    for s in 1..=steps {
        let tilt_deg = f64::from((s as f32 * step).min(max));
        out.extend((0..8).map(|k| Dir {
            tilt_deg,
            axis_deg: 45.0 * f64::from(k),
        }));
    }
    out
}

/// Directions over the whole sphere about `step` degrees apart: rings of
/// tilt 0, `step`, … 180° (the opening straight down), each ring with as
/// many axes as fit `step` apart along it.
fn sphere(step: f64) -> Vec<Dir> {
    let rings = (180.0 / step).round() as u32;
    let mut out = Vec::new();
    for r in 0..=rings {
        let tilt_deg = 180.0 * f64::from(r) / f64::from(rings);
        let along = 360.0 * sin_cos(tilt_deg.to_radians()).0;
        let n = ((along / step).round() as u32).max(1);
        out.extend((0..n).map(|k| Dir {
            tilt_deg,
            axis_deg: 360.0 * f64::from(k) / f64::from(n),
        }));
    }
    out
}

/// The eight directions `step` degrees around `d`: tilts of ±`step`, axes
/// turned by `step` along the ring, and the four diagonals.
fn neighbours(d: Dir, step: f64) -> Vec<Dir> {
    let ring = sin_cos(d.tilt_deg.to_radians()).0;
    // near a pole the axis is free: eight axes on the first ring
    if ring < 1e-6 {
        let tilt_deg = if d.tilt_deg < 90.0 {
            step
        } else {
            180.0 - step
        };
        return (0..8)
            .map(|k| Dir {
                tilt_deg,
                axis_deg: 45.0 * f64::from(k),
            })
            .collect();
    }
    let turn = step / ring;
    let mut out = Vec::with_capacity(8);
    for dt in [-step, 0.0, step] {
        for da in [-turn, 0.0, turn] {
            if dt == 0.0 && da == 0.0 {
                continue;
            }
            let mut tilt_deg = d.tilt_deg + dt;
            let mut axis_deg = d.axis_deg + da;
            // past a pole: the same direction about the opposite axis
            if tilt_deg < 0.0 {
                tilt_deg = -tilt_deg;
                axis_deg += 180.0;
            } else if tilt_deg > 180.0 {
                tilt_deg = 360.0 - tilt_deg;
                axis_deg += 180.0;
            }
            out.push(Dir {
                tilt_deg,
                axis_deg: axis_deg.rem_euclid(360.0),
            });
        }
    }
    out
}

/// Sine and cosine from basic IEEE operations only, without the platform
/// maths library, so every platform gets the same bits. Range reduction to
/// ±π/4, then Taylor series to the 17th power (error below 1e-15 for
/// moderate angles).
pub(crate) fn sin_cos(x: f64) -> (f64, f64) {
    let k = (x / std::f64::consts::FRAC_PI_2).round();
    let r = x - k * std::f64::consts::FRAC_PI_2;
    let r2 = r * r;
    let mut s = 0.0;
    let mut c = 0.0;
    // Horner from the highest term: sin = r Σ (−r²)^i / (2i+1)!,
    // cos = Σ (−r²)^i / (2i)!.
    for i in (0..9).rev() {
        let (fs, fc) = (FACT[2 * i + 1], FACT[2 * i]);
        let sign = if i % 2 == 0 { 1.0 } else { -1.0 };
        s = s * r2 + sign / fs;
        c = c * r2 + sign / fc;
    }
    let s = s * r;
    match (k as i64).rem_euclid(4) {
        0 => (s, c),
        1 => (c, -s),
        2 => (-s, -c),
        _ => (-c, s),
    }
}

/// n! for n ≤ 17, exact in f64.
const FACT: [f64; 18] = {
    let mut f = [1.0f64; 18];
    let mut i = 1;
    while i < 18 {
        f[i] = f[i - 1] * i as f64;
        i += 1;
    }
    f
};

#[cfg(test)]
mod tests {
    use super::*;
    use crate::testutil::{aligner_shell_with_faces, default_shell, spiked_slab};
    use glam::EulerRot;

    #[test]
    fn sin_cos_matches_the_maths_library() {
        for i in -400..=400 {
            let x = f64::from(i) * 0.0173;
            let (s, c) = sin_cos(x);
            assert!(
                (s - x.sin()).abs() < 1e-14 && (c - x.cos()).abs() < 1e-14,
                "{x}"
            );
        }
    }

    #[test]
    fn the_cavity_opens_up_whatever_the_input_pose() {
        for (k, euler) in [
            (0.0, 0.0, 0.0),
            (3.0, 0.2, -0.4),
            (1.1, -2.0, 0.7),
            (-0.5, 1.4, 2.9),
        ]
        .into_iter()
        .enumerate()
        {
            let mut shell = default_shell();
            let pose = Quat::from_euler(EulerRot::XYZ, euler.0, euler.1, euler.2);
            rotate_mesh(&mut shell.mesh, pose);
            let opening = pose * shell.opening;
            let part = shell.part("upper-01", k % 2 == 0);
            let o = orient(&part, &OrientParams::default());
            let up = (o.rotation * opening).z;
            assert!(up >= 0.70, "pose {k}: opening.z = {up}");
            // The plate holds the long side along X.
            let placed = oriented_mesh(&part.mesh, &o);
            let size = placed.aabb().unwrap().size();
            assert!(size.x >= size.y, "pose {k}: {size:?}");
            assert!((placed.aabb().unwrap().min.z - o.lift_mm).abs() < 1e-3);
        }
    }

    #[test]
    fn the_chosen_tilt_keeps_the_intaglio_up_and_the_sections_small() {
        let shell = default_shell();
        let fitting = shell.fitting_truth();
        let params = OrientParams::default();
        let o = orient_with(&shell.mesh, &fitting, &params);
        let down_z = score::down_z(params.overhang_deg);
        let chosen = score::score(&shell.mesh, &fitting, o.rotation, down_z);
        let flat = score::score(
            &shell.mesh,
            &fitting,
            opening_up(&shell.mesh, &fitting),
            down_z,
        );
        assert_eq!(chosen.fitting_down_mm2, 0.0, "{chosen:?}");
        assert!(
            chosen.total <= flat.total,
            "{chosen:?} vs untilted {flat:?}"
        );
        assert!(
            chosen.max_section_mm2 < flat.max_section_mm2,
            "{chosen:?} vs {flat:?}"
        );
    }

    #[test]
    fn orientation_is_deterministic() {
        let shell = aligner_shell_with_faces(40_000);
        let part = shell.part("lower-07", false);
        let a = orient(&part, &OrientParams::default());
        let b = orient(&part, &OrientParams::default());
        assert_eq!(a, b);
        assert_eq!(oriented_mesh(&part.mesh, &a), oriented_mesh(&part.mesh, &b));
    }

    #[test]
    fn no_tilt_search_keeps_the_base_pose() {
        let shell = default_shell();
        let params = OrientParams {
            tilt_step_deg: 0.0,
            ..OrientParams::default()
        };
        let o = orient_with(&shell.mesh, &shell.fitting_truth(), &params);
        assert!((o.rotation * shell.opening).z > 0.999);
        let empty = orient_with(&TriMesh::default(), &[], &params);
        assert_eq!(empty.rotation, Quat::IDENTITY);
    }

    /// A pose that starts an island on the fitting side is refused when
    /// another pose does not, even when it scores best.
    #[test]
    fn a_pose_with_an_island_on_the_fitting_side_is_refused() {
        let shell = default_shell();
        let mut fitting = shell.fitting_truth();
        let base_only = OrientParams {
            tilt_step_deg: 0.0,
            ..OrientParams::default()
        };
        // A patch of 1.5 mm around the lowest point of the untilted pose (a
        // cusp) is put on the fitting side, as an engraved stamp would be.
        let first = oriented_mesh(&shell.mesh, &orient_with(&shell.mesh, &fitting, &base_only));
        let low = (0..first.vertex_count())
            .min_by(|&a, &b| {
                first.positions[a]
                    .z
                    .total_cmp(&first.positions[b].z)
                    .then(a.cmp(&b))
            })
            .unwrap();
        let spot = shell.mesh.positions[low];
        for (f, flag) in fitting.iter_mut().enumerate() {
            let [a, b, c] = shell.mesh.triangle(f);
            *flag |= ((a + b + c) / 3.0).distance(spot) < 1.5;
        }
        let touches = islands::vertex_touches(&shell.mesh, &fitting);
        let count = |o: &Orientation| fitting_islands(&oriented_mesh(&shell.mesh, o), &touches, 50);

        let base = orient_with(&shell.mesh, &fitting, &base_only);
        assert!(count(&base) > 0, "the untilted pose starts the island");
        // Tilts of 45°: the untilted pose scores best, yet is refused.
        let params = OrientParams {
            tilt_step_deg: 45.0,
            ..OrientParams::default()
        };
        let (up, down) = (
            opening_up(&shell.mesh, &fitting),
            score::down_z(params.overhang_deg),
        );
        let scores: Vec<f64> = tilts(&params)
            .iter()
            .map(|t| score::score(&shell.mesh, &fitting, *t * up, down).total)
            .collect();
        assert!(scores[1..].iter().all(|s| *s > scores[0]), "{scores:?}");
        let chosen = orient_with(&shell.mesh, &fitting, &params);
        assert_ne!(chosen, base);
        assert_eq!(count(&chosen), 0);
    }

    /// The pyramid of [`spiked_slab`] starts an island on the fitting side
    /// in every tilt of up to 45°; the wide search finds the poses that
    /// start none, and only those.
    #[test]
    fn the_wide_search_finds_poses_the_tilts_miss() {
        let (mesh, fitting) = spiked_slab();
        let params = OrientParams {
            layer_um: 100,
            ..OrientParams::default()
        };
        let touches = islands::vertex_touches(&mesh, &fitting);
        let count =
            |o: &Orientation| fitting_islands(&oriented_mesh(&mesh, o), &touches, params.layer_um);
        let base = opening_up(&mesh, &fitting);
        for t in tilts(&params) {
            let o = pose(&mesh, (t * base).normalize(), params.lift_mm);
            assert!(count(&o) > 0, "{o:?}");
        }
        let first = orient_with(&mesh, &fitting, &params);
        let wide = wide_poses(&mesh, &fitting, &params, &[first]);
        assert!(!wide.is_empty());
        for o in &wide {
            assert_ne!(*o, first);
            assert_eq!(count(o), 0, "{o:?}");
            // the opening turned well past the old 45°
            assert!((o.rotation * Vec3::Z).z < 0.5, "{o:?}");
        }
        assert!(wide.len() <= WIDE_POSES);
    }

    /// The same poses, in the same order, whatever the thread count.
    #[test]
    fn the_wide_search_is_deterministic() {
        let (mesh, fitting) = spiked_slab();
        let params = OrientParams::default();
        let a = wide_poses(&mesh, &fitting, &params, &[]);
        let one = rayon::ThreadPoolBuilder::new()
            .num_threads(1)
            .build()
            .unwrap();
        let b = one.install(|| wide_poses(&mesh, &fitting, &params, &[]));
        assert!(!a.is_empty());
        assert_eq!(a, b);
        assert_eq!(a, wide_poses(&mesh, &fitting, &params, &[]));
    }

    /// Every direction of the sphere grid once, the poles included.
    #[test]
    fn the_sphere_grid_covers_every_direction() {
        let dirs = sphere(WIDE_STEP_DEG);
        let ups: Vec<Vec3> = dirs.iter().map(|d| d.quat().inverse() * Vec3::Z).collect();
        // any direction is within the step of one of the grid
        for i in 0..200 {
            let (s, c) = sin_cos(f64::from(i) * 0.731);
            let z = f64::from(i) / 100.0 - 1.0;
            let r = (1.0 - z * z).sqrt();
            let d = Vec3::new((r * c) as f32, (r * s) as f32, z as f32);
            let near = ups.iter().map(|u| u.dot(d)).fold(f32::MIN, f32::max);
            assert!(near > (WIDE_STEP_DEG as f32).to_radians().cos(), "{d:?}");
        }
        assert_eq!(dirs.first().map(|d| d.tilt_deg), Some(0.0));
        assert_eq!(dirs.last().map(|d| d.tilt_deg), Some(180.0));
    }
}
