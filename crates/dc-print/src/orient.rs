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
//!    hard island check names the part.
//!
//! The fitting side of an exported shell comes from its stage model
//! ([`fitting_from_model`]); [`fitting_faces`] guesses it from the geometry
//! alone for other parts.
//!
//! The orientation must not change from one platform to another. Trigonometry
//! therefore goes through [`sin_cos`] (basic IEEE operations only) or
//! through vectors. Candidates are scored in parallel and collected in
//! order.

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
        let rotation = (tilts[i] * base).normalize();
        let footprint: Vec<Vec2> = mesh
            .positions
            .iter()
            .map(|p| (rotation * *p).truncate())
            .collect();
        let turn = hull::long_side_turn(&footprint);
        let o = Orientation {
            rotation: (turn * rotation).normalize(),
            lift_mm,
        };
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

/// Islands of the oriented part alone that start on a vertex of a fitting
/// face (`on_fitting`, per vertex).
fn fitting_islands(oriented: &TriMesh, on_fitting: &[bool], layer_um: u32) -> usize {
    let minima = islands::material_minima(oriented);
    if !minima.iter().any(|&v| on_fitting[v as usize]) {
        return 0;
    }
    islands::islands(&[oriented], layer_um)
        .iter()
        .filter(|i| {
            islands::island_bottom(oriented, &minima, i, layer_um)
                .is_some_and(|v| on_fitting[v as usize])
        })
        .count()
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
        let (sh, ch) = sin_cos(tilt / 2.0);
        for k in 0..8 {
            let (sa, ca) = sin_cos(f64::from(k) * std::f64::consts::FRAC_PI_4);
            out.push(Quat::from_xyzw(
                (ca * sh) as f32,
                (sa * sh) as f32,
                0.0,
                ch as f32,
            ));
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
    use crate::testutil::{aligner_shell_with_faces, default_shell};
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
}
