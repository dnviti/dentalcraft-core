//! Fitting side of an exported aligner shell from its stage model (print
//! plan §2.3).
//!
//! The export writes every shell next to the model it was built on: the
//! shell's intaglio is that model's surface, moved out by the clearance,
//! except where the shell bridges the gaps between the teeth (block-out).
//! The stamp is engraved up to 0.3 mm into the wall from the tooth side.
//!
//! Between the teeth a face-by-face distance misleads: where the shell
//! bridges the gap, the model comes nearer to the outer side of the wall
//! than to the bridge. So the sides are decided per wall surface:
//! 1. **Walls.** A face whose ray into the material meets a face pointing
//!    the other way, at a wall-like distance (0.6–1.6 × the median wall),
//!    and that face's own ray comes back to it, is a wall face. Wall faces
//!    joined by edges form surfaces: the intaglio, the outer side, and
//!    small pieces where creases, rims and holes cut them.
//! 2. **Sides.** A surface is fitting when most of its faces lie on the
//!    model (centre within `tol`).
//! 3. **The rest** (rim bands along the trim line and around the holes
//!    between the teeth, the strokes of the stamp): fitting when the face
//!    lies on the model, its centre within `tol` or its front facing the
//!    model across at most `tol` + [`RAY_EXTRA_MM`]. Supports never touch
//!    them.
//!
//! A shell and a model from different frames or stages share little
//! surface: fewer than [`MIN_NEAR_SHARE`] of the faces on the model is a
//! [`ModelMismatch`].
//!
//! [`export_model`] finds the model of a shell in the export layout:
//! `<export>/stampa_resina/<shell>.stl` and
//! `<export>/modelli_termoformatura/<model>.stl`, paired by the export's
//! `*_manifest_stampa.csv`, else by name. The clearance comes from the
//! export's `manifest.json`.

use std::fs;
use std::path::{Path, PathBuf};

use dc_mesh::TriMesh;
use glam::Vec3;
use rayon::prelude::*;

use super::raycast::Bvh;

/// Distance from the model, beyond the clearance, within which a shell face
/// lies on it, mm: the engraved stamp reaches 0.3 mm into the wall.
pub const MODEL_TOL_MM: f32 = 0.3;
/// Smallest share of the shell's faces that must lie on the model.
pub const MIN_NEAR_SHARE: f32 = 0.3;
/// A face facing the model counts up to this much beyond the tolerance, mm.
pub const RAY_EXTRA_MM: f32 = 0.2;
/// Wall faces are paired at 0.6 to 1.6 times the median wall thickness.
const WALL_SPAN: [f32; 2] = [0.6, 1.6];
/// Longest ray across the wall, mm.
const WALL_REACH_MM: f32 = 3.0;
/// Distances to the model are measured up to this, mm.
const REACH_MM: f32 = 2.5;
/// Start offset of the rays off the surface, mm.
const EPS_MM: f32 = 1e-3;

/// The shell does not lie on the model given for it.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
#[error(
    "print-model-mismatch: {:.1} % of the shell lies on its model, at least {:.0} % expected (another frame or another stage)",
    100.0 * .near_share,
    100.0 * MIN_NEAR_SHARE
)]
pub struct ModelMismatch {
    /// Share of the shell's faces within the tolerance of the model.
    pub near_share: f32,
}

impl ModelMismatch {
    /// Stable kebab-case error code.
    pub fn code(&self) -> &'static str {
        "print-model-mismatch"
    }
}

/// Per-face fitting flags of `shell` from the `model` it was built on, in
/// the same frame; `tol_mm` is the clearance plus [`MODEL_TOL_MM`]. See
/// the module docs.
///
/// # Errors
/// [`ModelMismatch`] when fewer than [`MIN_NEAR_SHARE`] of the faces lie on
/// the model.
pub fn fitting_from_model(
    shell: &TriMesh,
    model: &TriMesh,
    tol_mm: f32,
) -> Result<Vec<bool>, ModelMismatch> {
    let n = shell.face_count();
    if n == 0 {
        return Ok(Vec::new());
    }
    let faces: Vec<(Vec3, Vec3)> = (0..n)
        .map(|f| {
            let [a, b, c] = shell.triangle(f);
            ((a + b + c) / 3.0, (b - a).cross(c - a).normalize_or_zero())
        })
        .collect();
    let near_model = nearby_model(shell, model, tol_mm.max(0.0) + REACH_MM);
    let own = Bvh::new(shell);

    // Distance of each face centre to the model (∞ beyond the reach), and
    // whether its front faces the model just beyond the tolerance.
    let reach = tol_mm + RAY_EXTRA_MM;
    let to_model: Vec<(f32, bool)> = faces
        .par_iter()
        .map(|&(c, nrm)| {
            let d = near_model
                .nearest(c, REACH_MM)
                .map_or(f32::INFINITY, |(d, _)| d);
            let facing = nrm != Vec3::ZERO
                && near_model
                    .cast(c + nrm * EPS_MM, nrm, reach)
                    .is_some_and(|h| model.face_normal_raw(h.face as usize).dot(nrm) < 0.0);
            (d, facing)
        })
        .collect();
    let near_share = to_model.iter().filter(|(d, _)| *d <= tol_mm).count() as f32 / n as f32;
    if near_share < MIN_NEAR_SHARE {
        return Err(ModelMismatch { near_share });
    }
    // Wall surfaces, each fitting when most of its faces lie on the model.
    let surfaces = wall_surfaces(shell, &own, &faces);
    let mut on_model = vec![(0u32, 0u32); surfaces.count];
    for (f, s) in surfaces.of.iter().enumerate() {
        if let Some(s) = s {
            on_model[*s as usize].0 += u32::from(to_model[f].0 <= tol_mm);
            on_model[*s as usize].1 += 1;
        }
    }
    Ok((0..n)
        .map(|f| match surfaces.of[f] {
            Some(s) => 2 * on_model[s as usize].0 > on_model[s as usize].1,
            None => to_model[f].0 <= tol_mm || to_model[f].1,
        })
        .collect())
}

/// Wall faces grouped into surfaces (see the module docs).
struct WallSurfaces {
    /// Surface of each face; `None` for faces that are not wall faces.
    of: Vec<Option<u32>>,
    /// Number of surfaces.
    count: usize,
}

/// Pairs the faces across the wall and joins the paired ones into
/// surfaces by shared edges, in face order.
fn wall_surfaces(shell: &TriMesh, own: &Bvh, faces: &[(Vec3, Vec3)]) -> WallSurfaces {
    let n = faces.len();
    // The face behind each face, across the material, and its distance.
    let behind: Vec<Option<(u32, f32)>> = faces
        .par_iter()
        .map(|&(c, nrm)| {
            if nrm == Vec3::ZERO {
                return None;
            }
            let hit = own.cast(c - nrm * EPS_MM, -nrm, WALL_REACH_MM)?;
            (faces[hit.face as usize].1.dot(nrm) < -0.5).then_some((hit.face, hit.t))
        })
        .collect();
    let mut spans: Vec<f32> = behind.iter().flatten().map(|b| b.1).collect();
    if spans.is_empty() {
        return WallSurfaces {
            of: vec![None; n],
            count: 0,
        };
    }
    spans.sort_by(f32::total_cmp);
    let wall = spans[spans.len() / 2];
    let (lo, hi) = (WALL_SPAN[0] * wall, WALL_SPAN[1] * wall);
    // A wall face's partner is at a wall-like distance and points back.
    let paired: Vec<bool> = (0..n)
        .map(|f| {
            behind[f].is_some_and(|(p, t)| {
                (lo..=hi).contains(&t)
                    && behind[p as usize]
                        .is_some_and(|(q, _)| faces[q as usize].0.distance(faces[f].0) < 0.75 * t)
            })
        })
        .collect();
    let neighbours = super::fitting::edge_neighbours(shell);
    let mut of: Vec<Option<u32>> = vec![None; n];
    let mut count = 0usize;
    let mut stack: Vec<u32> = Vec::new();
    for start in 0..n {
        if !paired[start] || of[start].is_some() {
            continue;
        }
        let id = count as u32;
        count += 1;
        of[start] = Some(id);
        stack.push(start as u32);
        while let Some(f) = stack.pop() {
            for &g in &neighbours[f as usize] {
                if paired[g as usize] && of[g as usize].is_none() {
                    of[g as usize] = Some(id);
                    stack.push(g);
                }
            }
        }
    }
    WallSurfaces { of, count }
}

/// The model's faces within `margin` of the shell's bounding box, as a
/// tree (the base and the rest of the model are never needed).
fn nearby_model(shell: &TriMesh, model: &TriMesh, margin: f32) -> Bvh {
    let Some(bb) = shell.aabb() else {
        return Bvh::default();
    };
    let (lo, hi) = (bb.min - Vec3::splat(margin), bb.max + Vec3::splat(margin));
    let faces = (0..model.face_count() as u32).filter(|&f| {
        let [a, b, c] = model.triangle(f as usize);
        let (fmin, fmax) = (a.min(b).min(c), a.max(b).max(c));
        fmin.cmple(hi).all() && fmax.cmpge(lo).all()
    });
    Bvh::with_faces(model, faces)
}

/// The stage model of an exported aligner shell, and the clearance of the
/// export.
#[derive(Debug, Clone, PartialEq)]
pub struct ExportModel {
    /// The model file.
    pub path: PathBuf,
    /// Clearance between the teeth and the shell, mm.
    pub clearance_mm: f32,
}

/// Folder of the shells in an export.
pub const SHELL_DIR: &str = "stampa_resina";
/// Folder of the stage models in an export.
pub const MODEL_DIR: &str = "modelli_termoformatura";

/// The model of the shell file `shell` when it sits in an export:
/// `<export>/stampa_resina/` with `<export>/modelli_termoformatura/` beside
/// it. The pair comes from a `*_manifest_stampa.csv` of the export (columns
/// `model` and `shell`), else from the names: `{ARCH}_allineatore_NN.stl`
/// is built on `{ARCH}_modello_NN.stl`, and a passive
/// `{ARCH}_allineatore_NN_passivo.stl` on the last stage's model. `None`
/// outside an export or when the model file is missing.
pub fn export_model(shell: &Path) -> Option<ExportModel> {
    let dir = shell.parent()?;
    if dir.file_name()? != SHELL_DIR {
        return None;
    }
    let root = dir.parent()?;
    let models = root.join(MODEL_DIR);
    if !models.is_dir() {
        return None;
    }
    let name = shell.file_name()?.to_str()?;
    let path = model_from_manifests(root, name).or_else(|| model_by_name(&models, name))?;
    Some(ExportModel {
        path,
        clearance_mm: export_clearance(root),
    })
}

/// The model the export's `*_manifest_stampa.csv` files give for the shell
/// file `name`.
fn model_from_manifests(root: &Path, name: &str) -> Option<PathBuf> {
    let mut csvs: Vec<PathBuf> = fs::read_dir(root)
        .ok()?
        .filter_map(Result::ok)
        .map(|e| e.path())
        .filter(|p| {
            p.file_name()
                .and_then(|n| n.to_str())
                .is_some_and(|n| n.ends_with("_manifest_stampa.csv"))
        })
        .collect();
    csvs.sort();
    let wanted = format!("{SHELL_DIR}/{name}");
    csvs.iter().find_map(|csv| {
        let text = fs::read_to_string(csv).ok()?;
        let mut lines = text.lines();
        let header = csv_fields(lines.next()?);
        let col = |c: &str| header.iter().position(|h| h.trim() == c);
        let (model, shell) = (col("model")?, col("shell")?);
        lines.find_map(|line| {
            let f = csv_fields(line);
            let path = root.join(f.get(model)?.trim());
            (f.get(shell)?.trim() == wanted && path.is_file()).then_some(path)
        })
    })
}

/// The fields of one CSV line (RFC 4180 quoting).
fn csv_fields(line: &str) -> Vec<String> {
    let mut fields = vec![String::new()];
    let mut quoted = false;
    let mut chars = line.trim_end_matches('\r').chars().peekable();
    while let Some(c) = chars.next() {
        match (c, quoted) {
            ('"', true) if chars.peek() == Some(&'"') => {
                chars.next();
                fields.last_mut().expect("one field").push('"');
            }
            ('"', _) => quoted = !quoted,
            (',', false) => fields.push(String::new()),
            (c, _) => fields.last_mut().expect("one field").push(c),
        }
    }
    fields
}

/// The model named after the shell file `name`.
fn model_by_name(models: &Path, name: &str) -> Option<PathBuf> {
    let stem = name
        .strip_suffix(".stl")
        .or_else(|| name.strip_suffix(".STL"))?;
    let (stem, passive) = match stem.strip_suffix("_passivo") {
        Some(s) => (s, true),
        None => (stem, false),
    };
    let (arch, number) = stem.split_once("_allineatore_")?;
    if number.is_empty() || !number.bytes().all(|b| b.is_ascii_digit()) {
        return None;
    }
    let path = if passive {
        // The last stage: the model with the highest number.
        let prefix = format!("{arch}_modello_");
        let mut best: Option<(u32, PathBuf)> = None;
        for e in fs::read_dir(models).ok()?.filter_map(Result::ok) {
            let file = e.file_name();
            let Some(n) = file
                .to_str()
                .and_then(|f| f.strip_prefix(&prefix))
                .and_then(|f| f.strip_suffix(".stl"))
                .filter(|n| !n.is_empty() && n.bytes().all(|b| b.is_ascii_digit()))
                .and_then(|n| n.parse::<u32>().ok())
            else {
                continue;
            };
            if best.as_ref().is_none_or(|(b, _)| n > *b) {
                best = Some((n, e.path()));
            }
        }
        best?.1
    } else {
        models.join(format!("{arch}_modello_{number}.stl"))
    };
    path.is_file().then_some(path)
}

/// `settings.clearance_mm` of the export's `manifest.json`, 0 when absent.
fn export_clearance(root: &Path) -> f32 {
    fs::read(root.join("manifest.json"))
        .ok()
        .and_then(|b| serde_json::from_slice::<serde_json::Value>(&b).ok())
        .and_then(|v| v["settings"]["clearance_mm"].as_f64())
        .filter(|c| c.is_finite() && *c >= 0.0)
        .map_or(0.0, |c| c as f32)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::part::translate_mesh;
    use crate::testutil::{Side, aligner_shell_with_faces, default_shell};

    #[test]
    fn the_model_gives_the_true_intaglio() {
        let shell = default_shell();
        let model = shell.model();
        let flags = fitting_from_model(&shell.mesh, &model, MODEL_TOL_MM).unwrap();
        let truth = shell.fitting_truth();
        let wrong = (0..flags.len())
            .filter(|&f| flags[f] != truth[f] && shell.side[f] != Side::Rim)
            .count();
        assert_eq!(wrong, 0, "inner or outer faces wrong");
        // Outer faces right above the gingiva lie within the tolerance of
        // the model, yet the wall's inner side is nearer: they stay outer.
        let bvh = Bvh::new(&model);
        let near_outer = (0..flags.len())
            .filter(|&f| {
                let [a, b, c] = shell.mesh.triangle(f);
                shell.side[f] == Side::Outer
                    && bvh.nearest((a + b + c) / 3.0, MODEL_TOL_MM).is_some()
            })
            .count();
        assert!(near_outer > 20, "{near_outer} outer faces near the gingiva");
        // The rim band lies on the gingiva.
        assert!((0..flags.len()).all(|f| shell.side[f] != Side::Rim || flags[f]));
    }

    #[test]
    fn a_moved_model_is_refused() {
        let shell = aligner_shell_with_faces(8_000);
        let mut model = shell.model();
        // Moved along all three axes: a vertical wall moved only up would
        // still lie on its own plane.
        translate_mesh(&mut model, Vec3::splat(3.0));
        let err = fitting_from_model(&shell.mesh, &model, MODEL_TOL_MM).unwrap_err();
        assert_eq!(err.code(), "print-model-mismatch");
        assert!(err.near_share < MIN_NEAR_SHARE, "{err}");
    }

    #[test]
    fn csv_fields_follow_the_quoting_rules() {
        assert_eq!(csv_fields("a,b,,c\r"), ["a", "b", "", "c"]);
        assert_eq!(csv_fields(r#"1,"x, ""y""",z"#), ["1", r#"x, "y""#, "z"]);
    }

    #[test]
    fn shells_of_an_export_find_their_models() {
        let root = std::env::temp_dir().join(format!("dc-print-export-{}", std::process::id()));
        let (shells, models) = (root.join(SHELL_DIR), root.join(MODEL_DIR));
        fs::create_dir_all(&shells).unwrap();
        fs::create_dir_all(&models).unwrap();
        for f in [
            "SUP_modello_00_iniziale.stl",
            "SUP_modello_01.stl",
            "SUP_modello_02.stl",
            "INF_modello_01.stl",
        ] {
            fs::write(models.join(f), b"").unwrap();
        }
        for f in [
            "SUP_allineatore_01.stl",
            "SUP_allineatore_02.stl",
            "SUP_allineatore_02_passivo.stl",
            "INF_allineatore_01.stl",
        ] {
            fs::write(shells.join(f), b"").unwrap();
        }
        fs::write(
            root.join("manifest.json"),
            br#"{"settings":{"clearance_mm":0.05}}"#,
        )
        .unwrap();
        // The manifest pairs INF 01 with an unusual model name.
        fs::write(models.join("INF_altro.stl"), b"").unwrap();
        fs::write(
            root.join("INF_manifest_stampa.csv"),
            "aligner,plan_stage,passive,model,shell\n1,1,no,modelli_termoformatura/INF_altro.stl,stampa_resina/INF_allineatore_01.stl\n",
        )
        .unwrap();
        let get = |f: &str| {
            export_model(&shells.join(f))
                .map(|m| (m.path.file_name().unwrap().to_owned(), m.clearance_mm))
        };
        assert_eq!(
            get("SUP_allineatore_01.stl"),
            Some(("SUP_modello_01.stl".into(), 0.05))
        );
        assert_eq!(
            get("SUP_allineatore_02_passivo.stl"),
            Some(("SUP_modello_02.stl".into(), 0.05))
        );
        assert_eq!(
            get("INF_allineatore_01.stl"),
            Some(("INF_altro.stl".into(), 0.05))
        );
        assert_eq!(get("SUP_allineatore_07.stl"), None);
        assert_eq!(export_model(&models.join("SUP_modello_01.stl")), None);
        fs::remove_dir_all(&root).unwrap();
    }
}
