//! The `print.*` commands (print plan §8), headless and agent-drivable.
//!
//! Like the scanner's: [`command_infos`] lists them with the i18n keys of
//! their label and documentation (`print-cmd-<name>`, in the host's bundle:
//! dc-print holds no UI text, only stable error codes) and the JSON Schema
//! of their parameters; [`execute`] runs one.
//!
//! | id | parameters | result |
//! |---|---|---|
//! | `print.profiles` | none | `{profiles}`, each with a summary of its machine parameters (`entries`: their count) |
//! | `print.machine-params` | `profile?` | the machine parameters of the profile: the typed values and every entry with its comment |
//! | `print.prepare` | `inputs` (shell files) or `dir`, `out_dir`, the resin values `layer_um`, `exposure_s`, `bottom_exposure_s`, `bottom_layers`, `rest_s`, `resin_name`, `density_g_ml` (all required), `profile?`, `pwm?`, `gap_mm?`, `margin_mm?` | the summary also written to `summary.json`, with `profile_validated` and `warnings` |
//!
//! `print.prepare` reads one part per file, runs [`crate::job::prepare`],
//! then writes `plate_NN.goo` and `plate_NN.stl` per plate (NN from 01)
//! and `summary.json` into `out_dir`. Every file is first written to a
//! fresh temporary file beside it (created new, never through an existing
//! file or link) and synced; only when the whole set is written are the
//! `plate_NN.goo`/`.stl` of an earlier run that this run did not write
//! removed and the new files renamed into place. A failed or cancelled run
//! leaves `out_dir` as it was; nothing but those plate files is ever
//! removed. A missing resin value is refused before anything is read
//! (`print-no-resin-preset`): no value is ever guessed. Parts are read
//! with vertices identified by exact position ([`crate::part::read_mesh`]).
//! A part that is not a closed, consistently oriented solid with a positive
//! volume and no hole through its wall (open or non-manifold edges, flipped
//! or inward triangles, a tunnel: the criteria of the aligners' export
//! print check) is refused (`print-invalid-part`) before it is sliced.
//!
//! The summary's `profile_validated` is the profile's
//! [`PrinterProfile::validated`]; `warnings` lists `{code, message}`
//! objects, today only [`crate::profile::PROFILE_NOT_VALIDATED`]
//! (`print-profile-not-validated`): the profile's dimensional scale and
//! left/right orientation are not yet checked against the printer, so the
//! size of a test piece must be verified before use. Printing is not
//! blocked.
//!
//! A shell of a DentalCraft export (`stampa_resina/` with
//! `modelli_termoformatura/` beside it) takes its fitting side from its
//! stage model ([`crate::orient::export_model`],
//! [`crate::orient::fitting_from_model`]); other parts take it from their
//! geometry in the pipeline. Each part of the summary says which
//! (`fitting_source`: `model` or `geometry`).

use std::fs::{self, File};
use std::io::{BufWriter, Write};
use std::path::{Path, PathBuf};

use rayon::prelude::*;
use schemars::JsonSchema;
use serde::Deserialize;
use serde::de::DeserializeOwned;
use serde_json::{Map, Value, json};

use crate::PrinterProfile;
use crate::goo::JobSettings;
use crate::job::{self, JobError, PrintJob};
use crate::layout::{LayoutError, LayoutParams};
use crate::machine::MachineParams;
use crate::orient::{self, OrientParams};
use crate::part::PrintPart;
use crate::support::SupportParams;

/// `print.profiles`.
pub const PRINT_PROFILES: &str = "print.profiles";
/// `print.prepare`.
pub const PRINT_PREPARE: &str = "print.prepare";
/// `print.machine-params`.
pub const PRINT_MACHINE_PARAMS: &str = "print.machine-params";

/// A print command as listed to agents and the UI.
#[derive(Debug, Clone, Copy)]
pub struct PrintCommandInfo {
    /// Stable id (`print.*`).
    pub id: &'static str,
    /// i18n key of the label (`print-cmd-<name>`).
    pub label_key: &'static str,
    /// i18n key of the documentation (`<label key>-doc`).
    pub doc_key: &'static str,
    /// JSON Schema of the parameters object.
    pub params_schema: fn() -> Value,
}

fn schema_of<T: JsonSchema>() -> Value {
    schemars::schema_for!(T).to_value()
}

macro_rules! info {
    ($id:expr, $name:literal, $params:ty) => {
        PrintCommandInfo {
            id: $id,
            label_key: concat!("print-cmd-", $name),
            doc_key: concat!("print-cmd-", $name, "-doc"),
            params_schema: schema_of::<$params>,
        }
    };
}

/// Every print command, in listing order.
pub fn command_infos() -> Vec<PrintCommandInfo> {
    vec![
        info!(PRINT_PROFILES, "profiles", NoParams),
        info!(PRINT_MACHINE_PARAMS, "machine-params", MachineParamsParams),
        info!(PRINT_PREPARE, "prepare", PrepareParams),
    ]
}

/// `true` for the ids of the print commands (`print.` prefix).
pub fn is_print_command(id: &str) -> bool {
    id.starts_with("print.")
}

/// Errors of the print commands, with stable codes
/// ([`PrintCommandError::code`]); the message starts with the code.
#[derive(Debug, thiserror::Error)]
pub enum PrintCommandError {
    /// The parameters do not fit the command.
    #[error("invalid-params: {0}")]
    InvalidParams(String),
    /// Resin values are missing: every one is required, none is guessed.
    #[error("print-no-resin-preset: missing {}", .0.join(", "))]
    NoResinPreset(Vec<&'static str>),
    /// No printer profile has this id.
    #[error("print-unknown-profile: {0}")]
    UnknownProfile(String),
    /// The profile has no machine parameters.
    #[error("print-no-machine-params: {0}")]
    NoMachineParams(String),
    /// There is no part to read.
    #[error("print-no-inputs: {0}")]
    NoInputs(String),
    /// A part file could not be read.
    #[error("print-read: {path}: {source}")]
    Read {
        /// The file.
        path: String,
        /// Why.
        source: dc_mesh::MeshError,
    },
    /// An output could not be written.
    #[error("print-write: {path}: {source}")]
    Write {
        /// The file or folder.
        path: String,
        /// Why.
        source: std::io::Error,
    },
    /// A shell of an export does not lie on its stage model: another frame
    /// or another stage.
    #[error(
        "print-model-mismatch: {part}: {:.1} % of the shell lies on its model {model}, at least {:.0} % expected (another frame or another stage)",
        100.0 * .near_share,
        100.0 * orient::MIN_NEAR_SHARE
    )]
    ModelMismatch {
        /// The part.
        part: String,
        /// Its model file.
        model: String,
        /// Share of the shell's faces that lie on the model.
        near_share: f32,
    },
    /// The pipeline failed (its code leads the message).
    #[error(transparent)]
    Job(#[from] JobError),
}

impl PrintCommandError {
    /// Stable identifier, the same in every language (kebab-case).
    pub fn code(&self) -> &'static str {
        match self {
            Self::InvalidParams(_) => "invalid-params",
            Self::NoResinPreset(_) => "print-no-resin-preset",
            Self::UnknownProfile(_) => "print-unknown-profile",
            Self::NoMachineParams(_) => "print-no-machine-params",
            Self::NoInputs(_) => "print-no-inputs",
            Self::Read { .. } => "print-read",
            Self::Write { .. } => "print-write",
            Self::ModelMismatch { .. } => "print-model-mismatch",
            Self::Job(e) => e.code(),
        }
    }

    /// `true` when the call itself must change (parameters, settings, the
    /// inputs named), `false` when the work failed.
    pub fn is_invalid_call(&self) -> bool {
        matches!(
            self,
            Self::InvalidParams(_)
                | Self::NoResinPreset(_)
                | Self::UnknownProfile(_)
                | Self::NoMachineParams(_)
                | Self::NoInputs(_)
                | Self::ModelMismatch { .. }
                | Self::Job(JobError::InvalidSettings(_))
                | Self::Job(JobError::Layout(LayoutError::InvalidParams(_)))
        )
    }

    /// `true` when the caller cancelled.
    pub fn is_cancelled(&self) -> bool {
        matches!(self, Self::Job(JobError::Cancelled))
    }
}

/// Runs the print command `id`. `cancel` is polled while slicing;
/// `progress` gets the fraction done, 0 to 1.
///
/// # Errors
/// The [`PrintCommandError`] of the command; `invalid-params` for an
/// unknown id.
pub fn execute(
    id: &str,
    params: &Value,
    cancel: &(dyn Fn() -> bool + Sync),
    progress: &mut dyn FnMut(f32),
) -> Result<Value, PrintCommandError> {
    match id {
        PRINT_PROFILES => {
            let NoParams {} = parse(params)?;
            let profiles = PrinterProfile::builtin()
                .into_iter()
                .map(|p| {
                    let mut v = serde_json::to_value(&p).map_err(json_error)?;
                    if let Some(m) = &p.machine {
                        v["machine"] = machine_summary(m)?;
                    }
                    Ok(v)
                })
                .collect::<Result<Vec<_>, PrintCommandError>>()?;
            Ok(json!({ "profiles": profiles }))
        }
        PRINT_MACHINE_PARAMS => {
            let p: MachineParamsParams = parse(params)?;
            let profile = find_profile(p.profile.as_deref())?;
            let machine = profile
                .machine
                .as_ref()
                .ok_or_else(|| PrintCommandError::NoMachineParams(profile.id.clone()))?;
            let mut v = serde_json::to_value(machine).map_err(json_error)?;
            v["profile"] = json!(profile.id);
            Ok(v)
        }
        PRINT_PREPARE => {
            let p: PrepareParams = parse(params)?;
            prepare_files(&p, cancel, progress)
        }
        _ => Err(PrintCommandError::InvalidParams(format!(
            "{id} is not a print command"
        ))),
    }
}

/// Parses command parameters: `null` is an empty object.
fn parse<T: DeserializeOwned>(params: &Value) -> Result<T, PrintCommandError> {
    let value = if params.is_null() {
        Value::Object(Map::new())
    } else {
        params.clone()
    };
    serde_json::from_value(value).map_err(|e| PrintCommandError::InvalidParams(e.to_string()))
}

/// Parameters of the commands without parameters.
#[derive(Debug, Default, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
struct NoParams {}

/// Parameters of `print.machine-params`.
#[derive(Debug, Default, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
struct MachineParamsParams {
    /// Printer profile id (see `print.profiles`); default
    /// `saturn-4-ultra-16k`.
    profile: Option<String>,
}

/// The built-in profile `id`, the Saturn 4 Ultra 16K when `None`.
fn find_profile(id: Option<&str>) -> Result<PrinterProfile, PrintCommandError> {
    match id {
        None => Ok(PrinterProfile::saturn_4_ultra_16k()),
        Some(id) => PrinterProfile::builtin()
            .into_iter()
            .find(|p| p.id == id)
            .ok_or_else(|| PrintCommandError::UnknownProfile(id.to_owned())),
    }
}

/// The machine parameters without their entries (only their count).
fn machine_summary(m: &MachineParams) -> Result<Value, PrintCommandError> {
    let mut v = serde_json::to_value(m).map_err(json_error)?;
    v["entries"] = json!(m.entries.len());
    Ok(v)
}

/// Parameters of `print.prepare`: exactly one of `inputs` and `dir`. The
/// resin values come from the resin maker for this printer and layer
/// height; every one is required.
#[derive(Debug, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
#[schemars(extend("oneOf" = [{ "required": ["inputs"] }, { "required": ["dir"] }]))]
struct PrepareParams {
    /// Shell files to print, one part each, in millimetres (STL; OBJ and PLY
    /// are read too). The file name without its extension names the part.
    inputs: Option<Vec<String>>,
    /// A folder: every `.stl` file in it, in name order.
    dir: Option<String>,
    /// Folder for `plate_NN.goo`, `plate_NN.stl` and `summary.json`;
    /// created when missing, files of the same name replaced.
    out_dir: String,
    /// Printer profile id (see `print.profiles`); default
    /// `saturn-4-ultra-16k`.
    profile: Option<String>,
    /// Layer height, µm.
    #[schemars(required)]
    layer_um: Option<u32>,
    /// Exposure of the normal layers, s.
    #[schemars(required)]
    exposure_s: Option<f32>,
    /// Exposure of the bottom layers, s.
    #[schemars(required)]
    bottom_exposure_s: Option<f32>,
    /// Number of bottom layers.
    #[schemars(required)]
    bottom_layers: Option<u32>,
    /// Rest before each exposure (wait after the vat settles), s.
    #[schemars(required)]
    rest_s: Option<f32>,
    /// Resin name written into the files (1–31 ASCII characters).
    #[schemars(required)]
    resin_name: Option<String>,
    /// Resin density, g/ml (for the weight).
    #[schemars(required)]
    density_g_ml: Option<f32>,
    /// Light PWM, 1–255; default 255.
    pwm: Option<u16>,
    /// Gap between parts, mm; default 3.
    gap_mm: Option<f32>,
    /// Margin from the plate edges, mm; default 4.
    margin_mm: Option<f32>,
}

impl PrepareParams {
    /// The job, or the names of the missing resin values.
    fn print_job(&self) -> Result<PrintJob, PrintCommandError> {
        let missing: Vec<&'static str> = [
            ("layer_um", self.layer_um.is_none()),
            ("exposure_s", self.exposure_s.is_none()),
            ("bottom_exposure_s", self.bottom_exposure_s.is_none()),
            ("bottom_layers", self.bottom_layers.is_none()),
            ("rest_s", self.rest_s.is_none()),
            ("resin_name", self.resin_name.is_none()),
            ("density_g_ml", self.density_g_ml.is_none()),
        ]
        .into_iter()
        .filter_map(|(name, missing)| missing.then_some(name))
        .collect();
        let (
            Some(layer_um),
            Some(exposure_s),
            Some(bottom_exposure_s),
            Some(bottom_layers),
            Some(rest_s),
            Some(resin_name),
            Some(density_g_ml),
        ) = (
            self.layer_um,
            self.exposure_s,
            self.bottom_exposure_s,
            self.bottom_layers,
            self.rest_s,
            self.resin_name.clone(),
            self.density_g_ml,
        )
        else {
            return Err(PrintCommandError::NoResinPreset(missing));
        };
        let profile = find_profile(self.profile.as_deref())?;
        let defaults = LayoutParams::default();
        let job = PrintJob {
            profile,
            job: JobSettings {
                layer_um,
                exposure_s,
                bottom_exposure_s,
                bottom_layers,
                rest_before_exposure_s: rest_s,
                pwm: self.pwm.unwrap_or(255),
                resin_name,
                volume_mm3: 0.0,
                weight_g: 0.0,
                print_time_s: 0,
            },
            density_g_ml,
            orient: OrientParams {
                layer_um,
                ..OrientParams::default()
            },
            support: SupportParams {
                layer_um,
                ..SupportParams::default()
            },
            layout: LayoutParams {
                gap_mm: self.gap_mm.unwrap_or(defaults.gap_mm),
                margin_mm: self.margin_mm.unwrap_or(defaults.margin_mm),
            },
        };
        job.validate()?;
        Ok(job)
    }

    /// The part files, from exactly one of `inputs` and `dir`. `out_dir`
    /// may not be `dir`: its plate STLs would be read as parts next time.
    fn sources(&self) -> Result<Vec<PathBuf>, PrintCommandError> {
        let paths = match (&self.inputs, &self.dir) {
            (Some(inputs), None) => inputs.iter().map(PathBuf::from).collect(),
            (None, Some(dir)) => {
                let canonical = |p: &str| fs::canonicalize(p).ok();
                if canonical(dir).is_some_and(|d| canonical(&self.out_dir) == Some(d)) {
                    return Err(PrintCommandError::InvalidParams(
                        "out_dir must be another folder than dir".into(),
                    ));
                }
                stl_files(Path::new(dir))?
            }
            _ => {
                return Err(PrintCommandError::InvalidParams(
                    "give exactly one of inputs (a list of files) and dir (a folder)".into(),
                ));
            }
        };
        if paths.is_empty() {
            return Err(PrintCommandError::NoInputs("no file to print".into()));
        }
        Ok(paths)
    }
}

/// The `.stl` files of `dir`, sorted by name (the same on every platform).
fn stl_files(dir: &Path) -> Result<Vec<PathBuf>, PrintCommandError> {
    let entries = fs::read_dir(dir)
        .map_err(|e| PrintCommandError::NoInputs(format!("{}: {e}", dir.display())))?;
    let mut files: Vec<PathBuf> = entries
        .filter_map(Result::ok)
        .map(|e| e.path())
        .filter(|p| p.is_file() && p.extension().is_some_and(|e| e.eq_ignore_ascii_case("stl")))
        .collect();
    files.sort();
    Ok(files)
}

/// Where a part's fitting flags come from.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum FittingSource {
    /// The stage model the shell was built on.
    Model,
    /// The part's own geometry ([`crate::orient::fitting_faces`]).
    Geometry,
}

impl FittingSource {
    fn as_str(self) -> &'static str {
        match self {
            Self::Model => "model",
            Self::Geometry => "geometry",
        }
    }
}

/// Parts read at once: a shell holds its model (over 100 MB while it is
/// read and searched) until its fitting side is known.
const READ_AT_ONCE: usize = 2;

/// One part per file, named by the file name without its extension, in
/// input order.
fn read_parts(paths: &[PathBuf]) -> Result<Vec<(PrintPart, FittingSource)>, PrintCommandError> {
    let mut parts: Vec<(PrintPart, FittingSource)> = Vec::with_capacity(paths.len());
    for chunk in paths.chunks(READ_AT_ONCE) {
        let read: Vec<Result<(PrintPart, FittingSource), PrintCommandError>> =
            chunk.par_iter().map(|p| read_part(p)).collect();
        for r in read {
            let (part, source) = r?;
            if parts.iter().any(|(p, _)| p.id == part.id) {
                return Err(PrintCommandError::InvalidParams(format!(
                    "two parts are named {}: give files with different names",
                    part.id
                )));
            }
            parts.push((part, source));
        }
    }
    Ok(parts)
}

/// One part; a shell of an export gets its fitting flags from its stage
/// model.
fn read_part(path: &Path) -> Result<(PrintPart, FittingSource), PrintCommandError> {
    let read = |p: &Path| {
        crate::part::read_mesh(p).map_err(|source| PrintCommandError::Read {
            path: p.display().to_string(),
            source,
        })
    };
    let mesh = read(path)?;
    let id = path
        .file_stem()
        .map(|s| s.to_string_lossy().into_owned())
        .unwrap_or_default();
    let Some(paired) = orient::export_model(path) else {
        let part = PrintPart {
            id,
            mesh,
            fitting: None,
        };
        return Ok((part, FittingSource::Geometry));
    };
    let model = read(&paired.path)?;
    let tol = paired.clearance_mm + orient::MODEL_TOL_MM;
    let fitting = orient::fitting_from_model(&mesh, &model, tol).map_err(|e| {
        PrintCommandError::ModelMismatch {
            part: id.clone(),
            model: paired.path.display().to_string(),
            near_share: e.near_share,
        }
    })?;
    let part = PrintPart {
        id,
        mesh,
        fitting: Some(fitting),
    };
    Ok((part, FittingSource::Model))
}

/// `print.prepare`: parts in, plate files and the summary out.
fn prepare_files(
    p: &PrepareParams,
    cancel: &(dyn Fn() -> bool + Sync),
    progress: &mut dyn FnMut(f32),
) -> Result<Value, PrintCommandError> {
    let job = p.print_job()?;
    let (parts, sources): (Vec<PrintPart>, Vec<FittingSource>) =
        read_parts(&p.sources()?)?.into_iter().unzip();
    progress(0.0);
    let prepared = job::prepare_cancellable(&parts, &job, cancel)?;
    progress(0.1);
    // The plans are in input order, as the sources.
    let mut part_plans = serde_json::to_value(&prepared.parts).map_err(json_error)?;
    if let Some(list) = part_plans.as_array_mut() {
        for (plan, source) in list.iter_mut().zip(&sources) {
            plan["fitting_source"] = json!(source.as_str());
        }
    }

    let out_dir = Path::new(&p.out_dir);
    fs::create_dir_all(out_dir).map_err(|source| PrintCommandError::Write {
        path: out_dir.display().to_string(),
        source,
    })?;
    let file_time = job::file_time_now();
    let count = prepared.plates.len();
    let mut plates = Vec::with_capacity(count);
    let (mut volume_ml, mut weight_g, mut time_s) = (0.0, 0.0, 0u64);
    // Every file is staged first: a failed or cancelled run leaves the
    // earlier set as it was, never a mix of old and new plates.
    let mut staged: Vec<(PathBuf, PathBuf)> = Vec::with_capacity(2 * count + 1);
    let result = (|| {
        for (i, plate) in prepared.plates.iter().enumerate() {
            let name = format!("plate_{:02}", i + 1);
            let goo = format!("{name}.goo");
            let stl = format!("{name}.stl");
            let goo_path = out_dir.join(&goo);
            let (tmp, summary) = write_staged(&goo_path, |w| {
                job::write_plate_with(
                    plate,
                    &job,
                    w,
                    &mut |done, total| {
                        let plate_done = f64::from(done) / f64::from(total.max(1));
                        progress((0.1 + 0.9 * (i as f64 + plate_done) / count as f64) as f32);
                    },
                    cancel,
                    &file_time,
                )
            })?;
            staged.push((tmp, goo_path));
            let stl_path = out_dir.join(&stl);
            let (tmp, ()) = write_staged(&stl_path, |w| job::write_plate_stl(plate, w))?;
            staged.push((tmp, stl_path));
            volume_ml += summary.volume_ml;
            weight_g += summary.weight_g;
            time_s += u64::from(summary.est_time_s);
            let mut entry = serde_json::to_value(&summary).map_err(json_error)?;
            entry["goo"] = json!(goo);
            entry["stl"] = json!(stl);
            plates.push(entry);
        }
        let summary = json!({
            "out_dir": out_dir.display().to_string(),
            "profile": job.profile.id,
            "profile_validated": job.profile.validated,
            "warnings": warnings(&job.profile),
            "resin_name": job.job.resin_name,
            "layer_um": job.job.layer_um,
            "density_g_ml": job.density_g_ml,
            "plates": plates,
            "parts": part_plans,
            "total": {
                "plates": count,
                "parts": parts.len(),
                "volume_ml": volume_ml,
                "weight_g": weight_g,
                "est_time_s": time_s,
            },
        });
        let summary_path = out_dir.join("summary.json");
        let (tmp, ()) = write_staged(&summary_path, |w| {
            serde_json::to_writer_pretty(&mut *w, &summary).map_err(json_error)?;
            w.write_all(b"\n")?;
            Ok(())
        })?;
        staged.push((tmp, summary_path));
        Ok(summary)
    })();
    let summary = match result {
        Ok(s) => s,
        Err(e) => {
            for (tmp, _) in &staged {
                // best effort: the staged files are ours
                let _ = fs::remove_file(tmp);
            }
            return Err(e);
        }
    };
    let new: Vec<&Path> = staged.iter().map(|(_, path)| path.as_path()).collect();
    remove_stale_plates(out_dir, &new)?;
    for (i, (tmp, path)) in staged.iter().enumerate() {
        if let Err(source) = fs::rename(tmp, path) {
            for (tmp, _) in &staged[i..] {
                let _ = fs::remove_file(tmp);
            }
            return Err(PrintCommandError::Write {
                path: path.display().to_string(),
                source,
            });
        }
    }
    progress(1.0);
    Ok(summary)
}

/// `true` for the plate file names `print.prepare` writes:
/// `plate_<two or more digits>.goo` or `.stl`.
fn is_plate_file(name: &str) -> bool {
    let Some(rest) = name.strip_prefix("plate_") else {
        return false;
    };
    let Some(digits) = rest
        .strip_suffix(".goo")
        .or_else(|| rest.strip_suffix(".stl"))
    else {
        return false;
    };
    digits.len() >= 2 && digits.bytes().all(|b| b.is_ascii_digit())
}

/// Removes the plate files of `out_dir` that are not in `keep`: an earlier
/// run's plates must not sit beside this run's. Nothing else is touched.
fn remove_stale_plates(out_dir: &Path, keep: &[&Path]) -> Result<(), PrintCommandError> {
    let io = |path: &Path| {
        let path = path.display().to_string();
        move |source| PrintCommandError::Write { path, source }
    };
    for entry in fs::read_dir(out_dir).map_err(io(out_dir))? {
        let entry = entry.map_err(io(out_dir))?;
        let path = entry.path();
        let stale = entry.file_name().to_str().is_some_and(is_plate_file)
            && !keep.contains(&path.as_path())
            && !entry.file_type().map_err(io(&path))?.is_dir();
        if stale {
            fs::remove_file(&path).map_err(io(&path))?;
        }
    }
    Ok(())
}

/// The `{code, message}` warnings of a result prepared with `profile`.
fn warnings(profile: &PrinterProfile) -> Vec<Value> {
    profile
        .warnings()
        .into_iter()
        .map(|code| {
            let message = match code {
                crate::profile::PROFILE_NOT_VALIDATED => format!(
                    "the dimensional scale and the left/right orientation of printer profile {} are not yet checked against the printer: verify the size of a test piece before use",
                    profile.id
                ),
                _ => code.to_owned(),
            };
            json!({ "code": code, "message": message })
        })
        .collect()
}

fn json_error(e: serde_json::Error) -> JobError {
    JobError::Io(e.into())
}

/// Writes `path` through a staged temporary file ([`write_staged`]), then
/// renames it over `path`. On failure `path` is left as it was.
#[cfg(test)]
fn write_atomic<T>(
    path: &Path,
    write: impl FnOnce(&mut BufWriter<File>) -> Result<T, JobError>,
) -> Result<T, PrintCommandError> {
    let (tmp, value) = write_staged(path, write)?;
    fs::rename(&tmp, path).map_err(|source| {
        let _ = fs::remove_file(&tmp);
        PrintCommandError::Write {
            path: path.display().to_string(),
            source,
        }
    })?;
    Ok(value)
}

/// Writes a new temporary file beside `path` and syncs it; returns its
/// path, to be renamed over `path`. The file gets a fresh name
/// (`<name>.<pid>.<n>.tmp`) and is created with `create_new`, so an
/// existing file or link of that name is never followed or overwritten.
/// On failure the temporary file is removed.
fn write_staged<T>(
    path: &Path,
    write: impl FnOnce(&mut BufWriter<File>) -> Result<T, JobError>,
) -> Result<(PathBuf, T), PrintCommandError> {
    use std::sync::atomic::{AtomicU64, Ordering};
    static NEXT: AtomicU64 = AtomicU64::new(0);
    let io = |source| PrintCommandError::Write {
        path: path.display().to_string(),
        source,
    };
    let (tmp, file) = loop {
        let mut tmp = path.as_os_str().to_owned();
        tmp.push(format!(
            ".{}.{}.tmp",
            std::process::id(),
            NEXT.fetch_add(1, Ordering::Relaxed)
        ));
        let tmp = PathBuf::from(tmp);
        match File::options().write(true).create_new(true).open(&tmp) {
            Ok(file) => break (tmp, file),
            Err(e) if e.kind() == std::io::ErrorKind::AlreadyExists => continue,
            Err(e) => return Err(io(e)),
        }
    };
    let result = (|| {
        let mut w = BufWriter::new(file);
        let value = write(&mut w)?;
        let file = w.into_inner().map_err(|e| io(e.into_error()))?;
        file.sync_all().map_err(io)?;
        Ok(value)
    })();
    match result {
        Ok(value) => Ok((tmp, value)),
        Err(e) => {
            // best effort: the file is ours
            let _ = fs::remove_file(&tmp);
            Err(e)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn run(id: &str, params: Value) -> Result<Value, PrintCommandError> {
        execute(id, &params, &|| false, &mut |_| {})
    }

    fn resin() -> Value {
        json!({
            "layer_um": 200,
            "exposure_s": 2.5,
            "bottom_exposure_s": 30.0,
            "bottom_layers": 4,
            "rest_s": 1.5,
            "resin_name": "Clear aligner",
            "density_g_ml": 1.1,
        })
    }

    fn with(base: Value, extra: Value) -> Value {
        let mut v = base;
        for (k, x) in extra.as_object().unwrap() {
            v[k] = x.clone();
        }
        v
    }

    /// A fresh folder in the system temp folder, removed on drop.
    struct TempDir(PathBuf);

    impl TempDir {
        fn new(name: &str) -> Self {
            let dir = std::env::temp_dir().join(format!(
                "dc-print-{name}-{}-{:?}",
                std::process::id(),
                std::thread::current().id()
            ));
            let _ = fs::remove_dir_all(&dir);
            fs::create_dir_all(&dir).unwrap();
            Self(dir)
        }
    }

    impl Drop for TempDir {
        fn drop(&mut self) {
            let _ = fs::remove_dir_all(&self.0);
        }
    }

    #[test]
    fn commands_are_listed_with_keys_and_schemas() {
        let infos = command_infos();
        let ids: Vec<&str> = infos.iter().map(|i| i.id).collect();
        assert_eq!(ids, [PRINT_PROFILES, PRINT_MACHINE_PARAMS, PRINT_PREPARE]);
        for i in &infos {
            assert!(is_print_command(i.id));
            assert!(i.label_key.starts_with("print-cmd-"));
            assert_eq!(i.doc_key, format!("{}-doc", i.label_key));
            assert_eq!((i.params_schema)()["type"], "object", "{}", i.id);
        }
        assert!(!is_print_command("scan.start"));
        let schema = (infos[2].params_schema)();
        let required: Vec<&str> = schema["required"]
            .as_array()
            .unwrap()
            .iter()
            .map(|v| v.as_str().unwrap())
            .collect();
        for name in [
            "out_dir",
            "layer_um",
            "exposure_s",
            "bottom_exposure_s",
            "bottom_layers",
            "rest_s",
            "resin_name",
            "density_g_ml",
        ] {
            assert!(
                required.contains(&name),
                "{name} not required: {required:?}"
            );
        }
        assert!(!required.contains(&"pwm") && !required.contains(&"inputs"));
        assert_eq!(schema["oneOf"].as_array().unwrap().len(), 2);
        assert_eq!(schema["properties"]["layer_um"]["type"], "integer");
    }

    #[test]
    fn profiles_are_listed() {
        let v = run(PRINT_PROFILES, Value::Null).unwrap();
        let saturn = &v["profiles"][0];
        assert_eq!(saturn["id"], PrinterProfile::SATURN_4_ULTRA_16K);
        assert_eq!(saturn["resolution_x"], 15_120);
        // the machine parameters in short: their values, not their entries
        assert_eq!(saturn["machine"]["name"], "DENTAL 45");
        assert_eq!(saturn["machine"]["z_travel_mm"], 230.0);
        assert_eq!(saturn["machine"]["vat_max_ml"], 870.0);
        assert_eq!(saturn["machine"]["entries"], 160);
        assert_eq!(
            run(PRINT_PROFILES, json!({"x": 1})).unwrap_err().code(),
            "invalid-params"
        );
    }

    #[test]
    fn machine_params_are_listed_in_full() {
        let v = run(PRINT_MACHINE_PARAMS, Value::Null).unwrap();
        assert_eq!(v["profile"], PrinterProfile::SATURN_4_ULTRA_16K);
        assert_eq!(v["z_travel_mm"], 230.0);
        assert_eq!(v["tank_target_c"], 45.0);
        assert_eq!(v["light_source"], "combined");
        assert_eq!(v["slow"]["first_ms"], 8420.0);
        assert_eq!(v["fast"]["after_ms"], 2430.0);
        let entries = v["entries"].as_array().unwrap();
        assert_eq!(entries.len(), 160);
        assert_eq!(entries[0]["axis"], 0);
        assert_eq!(entries[0]["letter"], "B");
        assert_eq!(entries[0]["comment"], "Corsa asse Z (mm)");
        let same = run(
            PRINT_MACHINE_PARAMS,
            json!({"profile": "saturn-4-ultra-16k"}),
        )
        .unwrap();
        assert_eq!(same, v);
        assert_eq!(
            run(PRINT_MACHINE_PARAMS, json!({"profile": "form-3"}))
                .unwrap_err()
                .code(),
            "print-unknown-profile"
        );
        assert_eq!(
            run(PRINT_MACHINE_PARAMS, json!({"printer": "x"}))
                .unwrap_err()
                .code(),
            "invalid-params"
        );
    }

    #[test]
    fn missing_resin_values_are_refused_first() {
        // nothing is read: the input does not even exist
        let e = run(
            PRINT_PREPARE,
            json!({"inputs": ["/nowhere/a.stl"], "out_dir": "/nowhere/out", "layer_um": 50}),
        )
        .unwrap_err();
        assert_eq!(e.code(), "print-no-resin-preset");
        assert!(e.is_invalid_call());
        let text = e.to_string();
        assert!(
            text.starts_with("print-no-resin-preset: missing exposure_s"),
            "{text}"
        );
        for name in [
            "bottom_exposure_s",
            "bottom_layers",
            "rest_s",
            "resin_name",
            "density_g_ml",
        ] {
            assert!(text.contains(name), "{text}");
        }
        assert!(!text.contains("layer_um"), "{text}");
        let mut each = resin();
        each.as_object_mut().unwrap().remove("density_g_ml");
        let e = run(
            PRINT_PREPARE,
            with(each, json!({"inputs": ["a.stl"], "out_dir": "o"})),
        )
        .unwrap_err();
        assert_eq!(e.to_string(), "print-no-resin-preset: missing density_g_ml");
    }

    #[test]
    fn prepare_parameters_are_checked() {
        let base = with(resin(), json!({"out_dir": "/nowhere/out"}));
        let code = |extra: Value| {
            run(PRINT_PREPARE, with(base.clone(), extra))
                .unwrap_err()
                .code()
        };
        // exactly one of inputs and dir
        assert_eq!(code(json!({})), "invalid-params");
        assert_eq!(
            code(json!({"inputs": ["a.stl"], "dir": "d"})),
            "invalid-params"
        );
        assert_eq!(code(json!({"inputs": []})), "print-no-inputs");
        assert_eq!(
            code(json!({"inputs": ["a.stl"], "colour": "red"})),
            "invalid-params"
        );
        assert_eq!(
            code(json!({"inputs": ["a.stl"], "layer_um": "fifty"})),
            "invalid-params"
        );
        assert_eq!(
            code(json!({"inputs": ["a.stl"], "profile": "form-3"})),
            "print-unknown-profile"
        );
        // settings out of range, before any file is read
        assert_eq!(
            code(json!({"inputs": ["a.stl"], "layer_um": 5})),
            "print-invalid-settings"
        );
        assert_eq!(
            code(json!({"inputs": ["a.stl"], "resin_name": "résine"})),
            "print-invalid-settings"
        );
        assert_eq!(
            code(json!({"inputs": ["a.stl"], "density_g_ml": 0.0})),
            "print-invalid-settings"
        );
        assert_eq!(
            code(json!({"inputs": ["a.stl"], "margin_mm": 80.0})),
            "print-invalid-layout"
        );
        assert_eq!(
            code(json!({"inputs": ["a.stl"], "gap_mm": -1.0})),
            "print-invalid-layout"
        );
        // the files
        assert_eq!(code(json!({"inputs": ["/nowhere/a.stl"]})), "print-read");
        assert_eq!(code(json!({"dir": "/nowhere"})), "print-no-inputs");
        let empty = TempDir::new("empty");
        fs::write(empty.0.join("notes.txt"), "x").unwrap();
        assert_eq!(code(json!({"dir": empty.0})), "print-no-inputs");
        assert_eq!(
            code(json!({"dir": empty.0, "out_dir": empty.0})),
            "invalid-params"
        );
        assert_eq!(
            run("print.nothing", Value::Null).unwrap_err().code(),
            "invalid-params"
        );
    }

    #[test]
    fn failed_writes_leave_no_file() {
        let dir = TempDir::new("atomic");
        let path = dir.0.join("plate_01.goo");
        fs::write(&path, b"old").unwrap();
        let e = write_atomic(&path, |w| {
            w.write_all(b"half")?;
            Err::<(), _>(JobError::Cancelled)
        })
        .unwrap_err();
        assert!(e.is_cancelled());
        assert_eq!(fs::read(&path).unwrap(), b"old");
        assert!(!dir.0.join("plate_01.goo.tmp").exists());
        write_atomic(&path, |w| Ok(w.write_all(b"new")?)).unwrap();
        assert_eq!(fs::read(&path).unwrap(), b"new");
    }

    #[test]
    fn temp_files_are_fresh_and_never_reused() {
        let dir = TempDir::new("fresh");
        let path = dir.0.join("plate_01.goo");
        // a file at the old predictable name is neither followed nor touched
        let old_tmp = dir.0.join("plate_01.goo.tmp");
        fs::write(&old_tmp, b"someone else's").unwrap();
        let (a, ()) = write_staged(&path, |w| Ok(w.write_all(b"a")?)).unwrap();
        let (b, ()) = write_staged(&path, |w| Ok(w.write_all(b"b")?)).unwrap();
        assert_ne!(a, b);
        assert_ne!(a, old_tmp);
        assert_eq!(fs::read(&a).unwrap(), b"a");
        assert_eq!(fs::read(&b).unwrap(), b"b");
        assert_eq!(fs::read(&old_tmp).unwrap(), b"someone else's");
        assert!(!path.exists());
    }

    #[test]
    fn only_stale_plate_files_are_removed() {
        for (name, plate) in [
            ("plate_01.goo", true),
            ("plate_03.stl", true),
            ("plate_123.goo", true),
            ("plate_1.goo", false),
            ("plate_01.goo.tmp", false),
            ("plate_ab.stl", false),
            ("summary.json", false),
            ("myplate_01.goo", false),
        ] {
            assert_eq!(is_plate_file(name), plate, "{name}");
        }
        let dir = TempDir::new("stale");
        for name in [
            "plate_01.goo",
            "plate_01.stl",
            "plate_02.goo",
            "plate_02.stl",
            "plate_03.goo",
            "notes.txt",
            "plate_1.goo",
        ] {
            fs::write(dir.0.join(name), b"x").unwrap();
        }
        fs::create_dir(dir.0.join("plate_04.stl")).unwrap();
        let keep = [dir.0.join("plate_01.goo"), dir.0.join("plate_01.stl")];
        let keep: Vec<&Path> = keep.iter().map(PathBuf::as_path).collect();
        remove_stale_plates(&dir.0, &keep).unwrap();
        let mut left: Vec<String> = fs::read_dir(&dir.0)
            .unwrap()
            .map(|e| e.unwrap().file_name().into_string().unwrap())
            .collect();
        left.sort();
        assert_eq!(
            left,
            [
                "notes.txt",
                "plate_01.goo",
                "plate_01.stl",
                "plate_04.stl",
                "plate_1.goo"
            ]
        );
    }

    /// A part that is not a closed solid is refused before any file is
    /// written, and an earlier run's plates stay as they were.
    #[test]
    fn open_parts_are_refused_and_earlier_plates_kept() {
        let dir = TempDir::new("open");
        let (mut cup, _) = crate::job::tests::cup(glam::Vec3::new(20.0, 12.0, 8.0), 1.0);
        cup.tris.pop();
        let path = dir.0.join("SUP_allineatore_01.stl");
        dc_mesh::io::write(&path, &cup, dc_mesh::MeshFormat::Stl).unwrap();
        let out = dir.0.join("out");
        fs::create_dir_all(&out).unwrap();
        fs::write(out.join("plate_02.goo"), b"old").unwrap();
        let e = run(
            PRINT_PREPARE,
            with(resin(), json!({"inputs": [path], "out_dir": out})),
        )
        .unwrap_err();
        assert_eq!(e.code(), "print-invalid-part", "{e}");
        assert!(e.to_string().contains("SUP_allineatore_01"), "{e}");
        assert_eq!(fs::read(out.join("plate_02.goo")).unwrap(), b"old");
        assert_eq!(fs::read_dir(&out).unwrap().count(), 1);
    }

    /// A cup with the edge `(a, b)` of one triangle split a fifth of the
    /// STL reader's weld tolerance from `a`, on both of its triangles: a
    /// closed solid with an edge far shorter than that tolerance, as the
    /// shell mesher makes them. Written to `path`; the first edge whose
    /// split the tolerance weld folds.
    fn cup_with_a_tiny_edge(path: &Path) -> dc_mesh::TriMesh {
        let (cup, _) = crate::job::tests::cup(glam::Vec3::new(20.0, 12.0, 8.0), 1.0);
        let cell = 1e-6 * cup.aabb().unwrap().diagonal();
        for (f, &[a, b, c]) in cup.tris.iter().enumerate() {
            // the neighbour runs (b, a)
            let Some((g, d)) = cup.tris.iter().enumerate().find_map(|(g, t)| {
                (0..3)
                    .find(|&k| t[k] == b && t[(k + 1) % 3] == a)
                    .map(|k| (g, t[(k + 2) % 3]))
            }) else {
                continue;
            };
            let (pa, pb) = (cup.positions[a as usize], cup.positions[b as usize]);
            let m = pa + (pb - pa).normalize() * (0.2 * cell);
            let mut split = cup.clone();
            split.positions.push(m);
            let s = (split.positions.len() - 1) as u32;
            split.tris[f] = [a, s, c];
            split.tris[g] = [b, s, d];
            split.tris.extend([[s, b, c], [s, a, d]]);
            dc_mesh::io::write(path, &split, dc_mesh::MeshFormat::Stl).unwrap();
            if dc_mesh::io::read(path).unwrap().positions.len() < split.positions.len() {
                return split;
            }
        }
        panic!("no split edge is folded by the weld");
    }

    /// An edge shorter than the reader's weld tolerance is no defect: the
    /// part is read with exact vertex identity and prepared, and even a
    /// mesh read with the tolerance weld (its two triangles folded) passes
    /// the solid check. It was refused as "edges shared by more than two
    /// triangles" (export SUP 01 and 05 of 2026-11-23).
    #[test]
    fn tiny_edges_are_not_folded_into_non_manifold_ones() {
        let dir = TempDir::new("tiny-edge");
        let path = dir.0.join("SUP_allineatore_01.stl");
        let split = cup_with_a_tiny_edge(&path);
        let exact = crate::part::read_mesh(&path).unwrap();
        assert_eq!(exact.positions.len(), split.positions.len());
        assert_eq!(crate::job::solid_defect(&exact), None);
        let welded = dc_mesh::io::read(&path).unwrap();
        assert_eq!(crate::job::solid_defect(&welded), None);
        let out = dir.0.join("out");
        let v = run(
            PRINT_PREPARE,
            with(resin(), json!({"inputs": [path], "out_dir": out})),
        )
        .unwrap();
        assert_eq!(v["total"]["parts"], 1, "{v}");
    }

    /// The whole command on two cups (needs orientation, supports and the
    /// slicer).
    #[test]
    fn prepare_writes_plates_and_the_summary() {
        let dir = TempDir::new("prepare");
        let (cup, _) = crate::job::tests::cup(glam::Vec3::new(20.0, 12.0, 8.0), 1.0);
        for name in ["SUP_allineatore_01", "INF_allineatore_01"] {
            let path = dir.0.join(format!("{name}.stl"));
            dc_mesh::io::write(&path, &cup, dc_mesh::MeshFormat::Stl).unwrap();
        }
        let out = dir.0.join("out");
        // an earlier run's plates and a file of the user's
        fs::create_dir_all(&out).unwrap();
        for name in ["plate_07.goo", "plate_07.stl", "notes.txt"] {
            fs::write(out.join(name), b"old").unwrap();
        }
        let mut fractions = Vec::new();
        let v = execute(
            PRINT_PREPARE,
            &with(resin(), json!({"dir": dir.0, "out_dir": out})),
            &|| false,
            &mut |f| fractions.push(f),
        )
        .unwrap();
        assert_eq!(v["total"]["parts"], 2);
        assert_eq!(fractions.first(), Some(&0.0));
        assert_eq!(fractions.last(), Some(&1.0));
        assert!(fractions.windows(2).all(|w| w[0] <= w[1]));
        let plates = v["plates"].as_array().unwrap();
        assert_eq!(v["total"]["plates"], plates.len());
        let mut ids: Vec<&str> = Vec::new();
        for p in plates {
            let goo = fs::read(out.join(p["goo"].as_str().unwrap())).unwrap();
            let file = crate::goo::GooFile::parse(&goo).unwrap();
            assert_eq!(json!(file.header.layer_count), p["layers"]);
            assert_eq!(file.header.resin_name, "Clear aligner");
            let stl = dc_mesh::io::read(out.join(p["stl"].as_str().unwrap())).unwrap();
            let on_plate = p["parts"].as_array().unwrap();
            // the shells and their supports
            assert!(stl.face_count() > on_plate.len() * cup.face_count());
            ids.extend(on_plate.iter().map(|i| i.as_str().unwrap()));
        }
        ids.sort_unstable();
        assert_eq!(ids, ["INF_allineatore_01", "SUP_allineatore_01"]);
        // the file is the returned summary (compared as text: floats need
        // not come back to the last bit when JSON is read)
        let written = fs::read_to_string(out.join("summary.json")).unwrap();
        assert_eq!(written, serde_json::to_string_pretty(&v).unwrap() + "\n");
        let leftovers: Vec<_> = fs::read_dir(&out)
            .unwrap()
            .filter_map(Result::ok)
            .filter(|e| e.path().extension().is_some_and(|x| x == "tmp"))
            .collect();
        assert!(leftovers.is_empty());
        assert!(!out.join("plate_07.goo").exists() && !out.join("plate_07.stl").exists());
        assert_eq!(fs::read(out.join("notes.txt")).unwrap(), b"old");
        assert_eq!(v["profile_validated"], false);
        assert_eq!(
            v["warnings"][0]["code"],
            crate::profile::PROFILE_NOT_VALIDATED
        );
        assert!(
            v["warnings"][0]["message"]
                .as_str()
                .unwrap()
                .contains("test piece")
        );
        assert_eq!(v["warnings"].as_array().unwrap().len(), 1);
    }

    /// A shell of an export (`stampa_resina` beside
    /// `modelli_termoformatura`) takes its fitting side from its model; a
    /// model from another frame is refused.
    #[test]
    fn export_shells_are_paired_with_their_models() {
        let dir = TempDir::new("export");
        let (shells, models) = (dir.0.join(orient::SHELL_DIR), dir.0.join(orient::MODEL_DIR));
        fs::create_dir_all(&shells).unwrap();
        fs::create_dir_all(&models).unwrap();
        let shell = crate::testutil::aligner_shell_with_faces(6_000);
        let stl = dc_mesh::MeshFormat::Stl;
        dc_mesh::io::write(shells.join("SUP_allineatore_01.stl"), &shell.mesh, stl).unwrap();
        dc_mesh::io::write(models.join("SUP_modello_01.stl"), &shell.model(), stl).unwrap();
        let out = dir.0.join("out");
        let params = with(resin(), json!({"dir": shells, "out_dir": out}));
        let v = run(PRINT_PREPARE, params.clone()).unwrap();
        assert_eq!(v["parts"][0]["fitting_source"], "model");
        assert!(v["parts"][0]["contacts"].as_u64().unwrap() > 0);

        // The same shell elsewhere: its fitting side from its geometry.
        let loose = dir.0.join("loose");
        fs::create_dir_all(&loose).unwrap();
        fs::copy(
            shells.join("SUP_allineatore_01.stl"),
            loose.join("SUP_allineatore_01.stl"),
        )
        .unwrap();
        let v = run(
            PRINT_PREPARE,
            with(resin(), json!({"dir": loose, "out_dir": out})),
        )
        .unwrap();
        assert_eq!(v["parts"][0]["fitting_source"], "geometry");

        // A model moved 3 mm on every axis: refused before anything is
        // written.
        let mut moved = shell.model();
        crate::part::translate_mesh(&mut moved, glam::Vec3::splat(3.0));
        dc_mesh::io::write(models.join("SUP_modello_01.stl"), &moved, stl).unwrap();
        let err = run(PRINT_PREPARE, params).unwrap_err();
        assert_eq!(err.code(), "print-model-mismatch");
        assert!(err.is_invalid_call());
        assert!(err.to_string().contains("SUP_allineatore_01"), "{err}");
    }
}
