//! The scan import pipeline, shared by every surface (app, CLI, MCP, modules).
//!
//! * [`prepare`] reads one file: cleanup, units, axes, warnings. It touches no
//!   state, so a background job can run it. [`prepare_with`] does the same
//!   without the unit conversion, for bite and visit scans.
//! * [`allocate`] decides the roles of a batch. It is pure: no geometry, no I/O.
//! * [`commit`] applies the result as one state transition.
//! * [`import_files`] chains the three and is the single entry point.
//!
//! Geometry detection of the arch stays out of the core: a module passes it to
//! [`import_files`] as a [`Resolver`].

use std::collections::{BTreeMap, BTreeSet, HashSet};
use std::fmt;
use std::path::{Path, PathBuf};
use std::sync::Arc;

use dc_jobs::{Cancelled, JobCtx};
use dc_mesh::io::{ReadOptions, detect_units, normalize_units};
use dc_mesh::{
    BadFaceReport, MeshError, MeshFormat, ScanFile, TriMesh, Units, read_scan, remove_bad_faces,
    rotate_y_up_to_z_up,
};
use serde::ser::SerializeStruct;
use serde::{Serialize, Serializer};
use uuid::Uuid;

use crate::{BlobRef, CaseState, MAX_BITE_SCANS, Pose, RoleSource, Scan, ScanRole, role_from_name};

/// A model whose largest extent is below this (mm, after the unit conversion)
/// gets the [`WarningCode::Tiny`] warning.
pub const TINY_EXTENT_MM: f32 = 5.0;

/// A scan with fewer faces than this gets the [`WarningCode::LowPolygons`]
/// warning. Rejecting it is the job of the arch step, not of the import.
pub const LOW_POLYGON_FACES: usize = 1000;

/// i18n key of the progress message of [`import_files`].
pub const PROGRESS_KEY: &str = "case-import-progress-reading";

/// Exact weld, as in the plugin: only corners with the same coordinates merge.
/// A tolerance can collapse the real small triangles of a dense scan.
const SCAN_READ: ReadOptions = ReadOptions {
    weld_tolerance: 0.0,
};

/// Arch slots in the order a provisional scan fills them (lower first, as in
/// the plugin).
const ARCHES: [ScanRole; 2] = [ScanRole::Lower, ScanRole::Upper];

/// Stable identifier of a warning, the same in every language.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum WarningCode {
    /// The file was not in millimetres.
    Rescaled,
    /// A bite or visit scan, never rescaled, has the size of another unit.
    UnitsSuspect,
    /// Largest extent under [`TINY_EXTENT_MM`].
    Tiny,
    /// Fewer faces than [`LOW_POLYGON_FACES`].
    LowPolygons,
    /// An OBJ with several objects became one scan.
    ObjObjectsMerged,
    /// Degenerate or duplicate faces were dropped.
    BadFacesRemoved,
    /// A new scan took the place of an existing one.
    Replaced,
    /// The detected arch was taken: the scan went to the other one.
    ArchConflict,
    /// The role is a guess, or a provisional scan was moved.
    Provisional,
    /// The bite role is full.
    BiteLimit,
}

/// A structured import warning. Serializes to `code`, `key`, `args` and the
/// `message` in the current language, so JSON and toasts share it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ImportWarning {
    /// Stable identifier.
    pub code: WarningCode,
    /// Fluent key.
    pub key: &'static str,
    /// Fluent arguments.
    pub args: BTreeMap<String, String>,
}

impl ImportWarning {
    fn new(code: WarningCode, key: &'static str, args: &[(&str, &str)]) -> Self {
        let args = args
            .iter()
            .map(|(k, v)| ((*k).to_owned(), (*v).to_owned()))
            .collect();
        Self { code, key, args }
    }

    /// The warning in the current language.
    pub fn message(&self) -> String {
        let args: Vec<(&str, &str)> = self
            .args
            .iter()
            .map(|(k, v)| (k.as_str(), v.as_str()))
            .collect();
        translate(self.key, &args)
    }
}

impl Serialize for ImportWarning {
    fn serialize<S: Serializer>(&self, s: S) -> Result<S::Ok, S::Error> {
        let mut w = s.serialize_struct("ImportWarning", 4)?;
        w.serialize_field("code", &self.code)?;
        w.serialize_field("key", self.key)?;
        w.serialize_field("args", &self.args)?;
        w.serialize_field("message", &self.message())?;
        w.end()
    }
}

/// Why a file was not imported. `Display` gives the text in the current
/// language; [`ImportError::detail`] the untranslated technical cause.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ImportError {
    /// The job was cancelled.
    Cancelled,
    /// Not an STL, OBJ or PLY file.
    Unsupported {
        /// File name.
        name: String,
    },
    /// The file could not be opened or read.
    Unreadable {
        /// File name.
        name: String,
        /// Technical cause.
        detail: String,
    },
    /// Corrupt content: truncated, NaN coordinates, indices out of range...
    Invalid {
        /// File name.
        name: String,
        /// Technical cause.
        detail: String,
    },
    /// No faces, or none left after the cleanup.
    Empty {
        /// File name.
        name: String,
    },
    /// No arch slot left for a file whose role is unknown.
    NeedsRole {
        /// File name.
        name: String,
    },
    /// Several files of the batch claim the same arch.
    RoleClash {
        /// File name.
        name: String,
        /// The arch claimed.
        role: ScanRole,
    },
    /// Replacing or moving an arch would leave old results next to the new
    /// mesh: the state of a module cannot be cleared by this build.
    ArchesLocked {
        /// Id of the module.
        module: String,
    },
}

impl ImportError {
    /// Stable identifier, the same in every language.
    pub fn code(&self) -> &'static str {
        match self {
            Self::Cancelled => "cancelled",
            Self::Unsupported { .. } => "unsupported",
            Self::Unreadable { .. } => "unreadable",
            Self::Invalid { .. } => "invalid",
            Self::Empty { .. } => "empty",
            Self::NeedsRole { .. } => "needs_role",
            Self::RoleClash { .. } => "role_clash",
            Self::ArchesLocked { .. } => "arches_locked",
        }
    }

    /// Fluent key of the message.
    pub fn key(&self) -> &'static str {
        match self {
            Self::Cancelled => "case-import-error-cancelled",
            Self::Unsupported { .. } => "case-import-error-unsupported",
            Self::Unreadable { .. } => "case-import-error-unreadable",
            Self::Invalid { .. } => "case-import-error-invalid",
            Self::Empty { .. } => "case-import-error-empty",
            Self::NeedsRole { .. } => "case-import-error-needs-role",
            Self::RoleClash { .. } => "case-import-error-role-clash",
            Self::ArchesLocked { .. } => "case-import-error-arches-locked",
        }
    }

    /// Technical cause, untranslated, when there is one (logs, JSON).
    pub fn detail(&self) -> Option<&str> {
        match self {
            Self::Unreadable { detail, .. } | Self::Invalid { detail, .. } => Some(detail),
            _ => None,
        }
    }

    /// The error in the current language.
    pub fn message(&self) -> String {
        match self {
            Self::Cancelled => translate(self.key(), &[]),
            Self::RoleClash { name, role } => {
                translate(self.key(), &[("name", name), ("role", role_id(*role))])
            }
            Self::ArchesLocked { module } => translate(self.key(), &[("module", module)]),
            Self::Unsupported { name }
            | Self::Unreadable { name, .. }
            | Self::Invalid { name, .. }
            | Self::Empty { name }
            | Self::NeedsRole { name } => translate(self.key(), &[("name", name)]),
        }
    }

    fn of_mesh(name: &str, e: MeshError) -> Self {
        let name = name.to_owned();
        match e {
            MeshError::UnsupportedFormat(_) => Self::Unsupported { name },
            MeshError::Io(e) => Self::Unreadable {
                name,
                detail: e.to_string(),
            },
            e => Self::Invalid {
                name,
                detail: e.to_string(),
            },
        }
    }
}

impl fmt::Display for ImportError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.message())
    }
}

impl std::error::Error for ImportError {}

impl From<Cancelled> for ImportError {
    fn from(_: Cancelled) -> Self {
        Self::Cancelled
    }
}

fn translate(key: &str, args: &[(&str, &str)]) -> String {
    crate::loader();
    dc_i18n::tr_args(key, args)
}

fn role_id(role: ScanRole) -> &'static str {
    match role {
        ScanRole::Upper => "upper",
        ScanRole::Lower => "lower",
        ScanRole::Bite => "bite",
    }
}

fn file_name(path: &Path) -> String {
    path.file_name().map_or_else(
        || path.display().to_string(),
        |n| n.to_string_lossy().into_owned(),
    )
}

/// A scan file read and cleaned up, not in a case yet.
#[derive(Debug, Clone)]
pub struct PreparedScan {
    /// The file.
    pub path: PathBuf,
    /// File name without directory.
    pub source_name: String,
    /// The mesh in millimetres, Z up.
    pub mesh: Arc<TriMesh>,
    /// Format of the file.
    pub format: MeshFormat,
    /// Unit the file was read in: millimetres when the units were left alone.
    pub source_units: Units,
    /// Faces dropped by the cleanup.
    pub cleanup: BadFaceReport,
    /// Warnings about the file (rescaled, tiny, low polygons...).
    pub warnings: Vec<ImportWarning>,
}

/// How [`prepare_with`] reads a file.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PrepareOptions {
    /// Convert to millimetres, with the unit guessed from the size. The guess
    /// fits a whole arch: a partial scan of 2 to 20 mm would look like
    /// centimetres and grow ten times. With `false` the coordinates stay as
    /// they are, taken as millimetres, and a size that fits another unit gives
    /// [`WarningCode::UnitsSuspect`].
    pub normalize_units: bool,
}

impl PrepareOptions {
    /// For a bite or visit scan: the plugin never rescales those.
    pub const KEEP_UNITS: Self = Self {
        normalize_units: false,
    };
}

impl Default for PrepareOptions {
    fn default() -> Self {
        Self {
            normalize_units: true,
        }
    }
}

/// Reads one arch scan: [`prepare_with`] with the default options.
pub fn prepare(path: &Path, ctx: &JobCtx) -> Result<PreparedScan, ImportError> {
    prepare_with(path, &PrepareOptions::default(), ctx)
}

/// Reads one scan file: read, drop bad faces, reject an empty mesh, convert to
/// millimetres (see [`PrepareOptions`]), turn OBJ files from Y up to Z up,
/// collect the warnings.
///
/// Touches no state and no UI. The OBJ rotation is baked into the positions.
pub fn prepare_with(
    path: &Path,
    options: &PrepareOptions,
    ctx: &JobCtx,
) -> Result<PreparedScan, ImportError> {
    let name = file_name(path);
    ctx.check_cancelled()?;
    let ScanFile {
        mut mesh,
        format,
        obj_object_count,
    } = read_scan(path, &SCAN_READ).map_err(|e| ImportError::of_mesh(&name, e))?;
    ctx.check_cancelled()?;
    let cleanup = remove_bad_faces(&mut mesh);
    if mesh.is_empty() {
        return Err(ImportError::Empty { name });
    }
    let of_mesh = |e| ImportError::of_mesh(&name, e);
    let (source_units, suspect) = if options.normalize_units {
        (normalize_units(&mut mesh).map_err(of_mesh)?, None)
    } else {
        let looks = detect_units(&mesh).map_err(of_mesh)?;
        (
            Units::Millimeters,
            (looks != Units::Millimeters).then_some(looks),
        )
    };
    if format == MeshFormat::Obj {
        rotate_y_up_to_z_up(&mut mesh);
    }
    let warnings = file_warnings(
        &name,
        &mesh,
        source_units,
        suspect,
        cleanup,
        obj_object_count,
    );
    ctx.check_cancelled()?;
    Ok(PreparedScan {
        path: path.to_owned(),
        source_name: name,
        mesh: Arc::new(mesh),
        format,
        source_units,
        cleanup,
        warnings,
    })
}

/// Warnings of a prepared file, measured on the final mesh.
fn file_warnings(
    name: &str,
    mesh: &TriMesh,
    units: Units,
    suspect: Option<Units>,
    cleanup: BadFaceReport,
    obj_objects: Option<usize>,
) -> Vec<ImportWarning> {
    let mut out = Vec::new();
    if units != Units::Millimeters {
        out.push(ImportWarning::new(
            WarningCode::Rescaled,
            "case-import-warn-rescaled",
            &[
                ("name", name),
                ("factor", &units.to_mm().to_string()),
                ("units", units.symbol()),
            ],
        ));
    }
    if let Some(looks) = suspect {
        out.push(ImportWarning::new(
            WarningCode::UnitsSuspect,
            "case-import-warn-units-suspect",
            &[("name", name), ("units", looks.symbol())],
        ));
    }
    let extent = mesh.aabb().map_or(0.0, |b| b.size().max_element());
    if extent < TINY_EXTENT_MM {
        out.push(ImportWarning::new(
            WarningCode::Tiny,
            "case-import-warn-tiny",
            &[("name", name), ("size", &format!("{extent:.2}"))],
        ));
    }
    if mesh.face_count() < LOW_POLYGON_FACES {
        out.push(ImportWarning::new(
            WarningCode::LowPolygons,
            "case-import-warn-low-polygons",
            &[("name", name), ("faces", &mesh.face_count().to_string())],
        ));
    }
    if let Some(objects) = obj_objects.filter(|&n| n > 1) {
        out.push(ImportWarning::new(
            WarningCode::ObjObjectsMerged,
            "case-import-warn-obj-objects-merged",
            &[("name", name), ("objects", &objects.to_string())],
        ));
    }
    if cleanup.removed() > 0 {
        out.push(ImportWarning::new(
            WarningCode::BadFacesRemoved,
            "case-import-warn-bad-faces-removed",
            &[
                ("name", name),
                ("degenerate", &cleanup.degenerate.to_string()),
                ("duplicate", &cleanup.duplicate.to_string()),
            ],
        ));
    }
    out
}

/// Arch recognised from the geometry.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Detection {
    /// [`ScanRole::Upper`] or [`ScanRole::Lower`]; a bite is ignored, since
    /// only the user or the file name can make a scan a bite.
    pub arch: ScanRole,
    /// Higher is surer. Only compared between the files of a batch.
    pub confidence: f32,
}

/// What [`allocate`] knows about one file of a batch.
#[derive(Debug, Clone, PartialEq)]
pub struct FileClaim {
    /// File name, for the messages.
    pub name: String,
    /// Role chosen by the caller.
    pub explicit: Option<ScanRole>,
    /// Role read from the name ([`role_from_name`]).
    pub name_hint: Option<ScanRole>,
    /// Arch from the geometry, if known.
    pub detection: Option<Detection>,
}

/// What happens to one file of a batch.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Decision {
    /// The file enters the case.
    Place {
        /// Its role.
        role: ScanRole,
        /// Why it has that role.
        source: RoleSource,
        /// Existing scans it takes the place of.
        replaces: Vec<Uuid>,
    },
    /// Beyond the bite limit of the batch: left out, with a warning.
    Skip,
    /// The file cannot be placed.
    Fail(ImportError),
}

/// An existing provisional scan moved to the other arch.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
pub struct ScanMove {
    /// The scan.
    pub scan: Uuid,
    /// Its role before.
    pub from: ScanRole,
    /// Its role now.
    pub to: ScanRole,
}

/// Result of [`allocate`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Allocation {
    /// One decision per file, in input order.
    pub decisions: Vec<Decision>,
    /// Provisional scans to move.
    pub moves: Vec<ScanMove>,
    /// Warnings about the roles.
    pub warnings: Vec<ImportWarning>,
}

/// Who holds an arch slot during [`allocate`].
#[derive(Clone)]
enum Holder<'a> {
    /// Scans of the case (normally one).
    Case(Vec<&'a Scan>),
    /// A file of the batch.
    Batch,
}

struct Slots<'a> {
    files: &'a [FileClaim],
    /// Module whose state keeps the arches as they are.
    locked: Option<&'a str>,
    held: BTreeMap<ScanRole, Holder<'a>>,
    /// Arch slots claimed by name or explicitly, not yet placed.
    reserved: Vec<ScanRole>,
    out: Allocation,
}

fn other_arch(role: ScanRole) -> ScanRole {
    match role {
        ScanRole::Upper => ScanRole::Lower,
        _ => ScanRole::Upper,
    }
}

fn moved_warning(scan: &Scan, to: ScanRole) -> ImportWarning {
    ImportWarning::new(
        WarningCode::Provisional,
        "case-import-warn-provisional-moved",
        &[("name", &scan.source_name), ("role", role_id(to))],
    )
}

impl<'a> Slots<'a> {
    fn free(&self, role: ScanRole) -> bool {
        !self.held.contains_key(&role) && !self.reserved.contains(&role)
    }

    /// A scan of the case holds `role`, and it is only a guess.
    fn provisional(&self, role: ScanRole) -> bool {
        matches!(self.held.get(&role), Some(Holder::Case(olds))
            if olds.iter().all(|s| s.role_source == RoleSource::Provisional))
    }

    /// The scan on `role` can step aside: it is provisional and the other
    /// arch is free.
    fn movable(&self, role: ScanRole) -> bool {
        self.free(other_arch(role)) && self.provisional(role)
    }

    /// Puts file `i` on the arch `role`, moving a provisional occupant away
    /// or replacing the occupant. With the arches locked, a file that would
    /// do either fails.
    fn place(&mut self, i: usize, role: ScanRole, source: RoleSource) {
        let olds = match self.held.get(&role) {
            Some(Holder::Case(olds)) => olds.clone(),
            _ => Vec::new(),
        };
        if let Some(module) = self.locked.filter(|_| !olds.is_empty()) {
            let module = module.to_owned();
            return self.fail(i, ImportError::ArchesLocked { module });
        }
        let movable = self.movable(role);
        self.held.insert(role, Holder::Batch);
        self.out.decisions[i] = Decision::Place {
            role,
            source,
            replaces: Vec::new(),
        };
        if movable {
            let to = other_arch(role);
            for s in &olds {
                self.out.moves.push(ScanMove {
                    scan: s.id,
                    from: role,
                    to,
                });
                self.out.warnings.push(moved_warning(s, to));
            }
            self.held.insert(to, Holder::Case(olds));
        } else {
            for s in olds {
                self.replace(i, role, s);
            }
        }
    }

    /// File `i` replaces `s` on `role`. A scan moved earlier in this batch
    /// never gets there: see [`Slots::undo_move`].
    fn replace(&mut self, i: usize, role: ScanRole, s: &Scan) {
        let (i, role) = self.undo_move(s).unwrap_or((i, role));
        if let Decision::Place { replaces, .. } = &mut self.out.decisions[i] {
            replaces.push(s.id);
        }
        self.out.warnings.push(ImportWarning::new(
            WarningCode::Replaced,
            "case-import-warn-replaced",
            &[
                ("name", &self.files[i].name),
                ("old", &s.source_name),
                ("role", role_id(role)),
            ],
        ));
    }

    /// Cancels the move of `s`, which is going away. It leaves from where it
    /// is in the case, so what replaces it is the file that took that arch:
    /// returns that file and the arch.
    fn undo_move(&mut self, s: &Scan) -> Option<(usize, ScanRole)> {
        let k = self.out.moves.iter().position(|m| m.scan == s.id)?;
        let m = self.out.moves.remove(k);
        let moved = moved_warning(s, m.to);
        self.out.warnings.retain(|w| *w != moved);
        let by = self
            .out
            .decisions
            .iter()
            .position(|d| matches!(d, Decision::Place { role, .. } if *role == m.from))?;
        Some((by, m.from))
    }

    fn warn_file(&mut self, i: usize, code: WarningCode, key: &'static str, role: ScanRole) {
        let w = ImportWarning::new(
            code,
            key,
            &[("name", &self.files[i].name), ("role", role_id(role))],
        );
        self.out.warnings.push(w);
    }

    fn fail(&mut self, i: usize, error: ImportError) {
        self.out.decisions[i] = Decision::Fail(error);
    }

    /// Rules 1 to 3: explicit and name-hinted files take the arches first.
    /// One explicit claim beats the name hints on the same arch; several
    /// claims of equal rank all fail, whatever the file order.
    fn claimed_arches(&mut self) {
        let mut winners = Vec::new();
        for role in ARCHES {
            let of = |pick: fn(&FileClaim) -> Option<ScanRole>| -> Vec<usize> {
                let files = self.files.iter().enumerate();
                files
                    .filter(|(_, f)| pick(f) == Some(role))
                    .map(|(i, _)| i)
                    .collect()
            };
            let explicit = of(|f| f.explicit);
            let named = of(|f| f.name_hint.filter(|_| f.explicit.is_none()));
            let rank = if explicit.is_empty() {
                &named
            } else {
                &explicit
            };
            let winner = (rank.len() == 1).then(|| rank[0]);
            for &i in explicit.iter().chain(&named) {
                if Some(i) != winner {
                    let name = self.files[i].name.clone();
                    self.fail(i, ImportError::RoleClash { name, role });
                }
            }
            if let Some(i) = winner {
                winners.push((i, role));
                self.reserved.push(role);
            }
        }
        for (i, role) in winners {
            let source = if self.files[i].explicit.is_some() {
                RoleSource::Explicit
            } else {
                RoleSource::FileName
            };
            self.reserved.retain(|r| *r != role);
            self.place(i, role, source);
        }
    }

    /// Rule 6: explicit bites first, then the named ones, each in file order.
    /// New scans fill the free places, then replace the older bite scans from
    /// the last one; a file of the batch is never replaced.
    fn bites(&mut self, existing: &'a [Scan]) {
        let bite = |f: &FileClaim| f.explicit.or(f.name_hint) == Some(ScanRole::Bite);
        let files = self.files.iter().enumerate();
        let (explicit, named): (Vec<_>, Vec<_>) = files
            .filter(|(_, f)| bite(f))
            .map(|(i, f)| (i, f.explicit.is_some()))
            .partition(|&(_, is_explicit)| is_explicit);
        let mut olds: Vec<&Scan> = existing
            .iter()
            .filter(|s| s.role == ScanRole::Bite)
            .collect();
        let mut free = MAX_BITE_SCANS.saturating_sub(olds.len());
        let max = MAX_BITE_SCANS.to_string();
        for (k, (i, is_explicit)) in explicit.into_iter().chain(named).enumerate() {
            let name = self.files[i].name.clone();
            if k >= MAX_BITE_SCANS {
                self.out.warnings.push(ImportWarning::new(
                    WarningCode::BiteLimit,
                    "case-import-warn-bite-limit-skipped",
                    &[("name", &name), ("max", &max)],
                ));
                continue;
            }
            let mut replaces = Vec::new();
            if free > 0 {
                free -= 1;
            } else if let Some(old) = olds.pop() {
                replaces.push(old.id);
                self.out.warnings.push(ImportWarning::new(
                    WarningCode::BiteLimit,
                    "case-import-warn-bite-limit-replaced",
                    &[("name", &name), ("old", &old.source_name), ("max", &max)],
                ));
            }
            self.out.decisions[i] = Decision::Place {
                role: ScanRole::Bite,
                source: if is_explicit {
                    RoleSource::Explicit
                } else {
                    RoleSource::FileName
                },
                replaces,
            };
        }
    }

    /// Rule 4: unhinted files with a detection, surest first, ties in file
    /// order. Only a provisional scan gives way to them: never a confirmed
    /// scan, never a file of the batch.
    fn detected(&mut self, unhinted: &[usize]) {
        let mut files: Vec<(usize, Detection)> = unhinted
            .iter()
            .filter_map(|&i| Some((i, self.files[i].detection?)))
            .filter(|(_, d)| d.arch != ScanRole::Bite)
            .collect();
        files.sort_by(|a, b| b.1.confidence.total_cmp(&a.1.confidence));
        for (i, d) in files {
            let (want, other) = (d.arch, other_arch(d.arch));
            if self.free(want) || (self.locked.is_none() && self.provisional(want)) {
                self.place(i, want, RoleSource::Detected);
            } else if self.free(other) {
                self.place(i, other, RoleSource::Provisional);
                self.warn_file(
                    i,
                    WarningCode::ArchConflict,
                    "case-import-warn-arch-conflict",
                    other,
                );
            } else {
                let name = self.files[i].name.clone();
                self.fail(i, ImportError::NeedsRole { name });
            }
        }
    }

    /// Rule 5: unhinted files without a detection take a free arch, nothing
    /// else.
    fn undetected(&mut self, unhinted: &[usize]) {
        for &i in unhinted {
            let known = self.files[i]
                .detection
                .is_some_and(|d| d.arch != ScanRole::Bite);
            if known {
                continue;
            }
            if let Some(role) = ARCHES.into_iter().find(|&r| self.free(r)) {
                self.place(i, role, RoleSource::Provisional);
                self.warn_file(
                    i,
                    WarningCode::Provisional,
                    "case-import-warn-provisional",
                    role,
                );
            } else {
                let name = self.files[i].name.clone();
                self.fail(i, ImportError::NeedsRole { name });
            }
        }
    }
}

/// Decides the roles of a batch against the scans already in the case.
///
/// 1. Explicit and name-hinted files take their slots first, so the result
///    does not depend on the file order.
/// 2. An explicit role ignores name and detection, and may replace.
/// 3. A provisional occupant moves to the free arch instead of being replaced.
/// 4. Unhinted files with a detection go by descending confidence. A guess
///    from the geometry never clears a confirmed arch, so only a provisional
///    scan gives way to them:
///
///    | Detected arch holds  | Other arch | Result                                  |
///    | -------------------- | ---------- | --------------------------------------- |
///    | nothing              | any        | taken, detected                         |
///    | a provisional scan   | free       | taken, detected; the scan moves there   |
///    | a provisional scan   | not free   | taken, detected; the scan is replaced   |
///    | anything else (a)    | free       | other arch, provisional, arch conflict  |
///    | anything else (a)    | not free   | [`ImportError::NeedsRole`]              |
///
///    (a) A [`RoleSource::DetectedUnsure`] scan is settled like a detected
///    one: the plugin warns about it, it does not give the arch away.
///
/// 5. Unhinted files without a detection take a free arch as provisional,
///    lower first; with both arches taken they fail with
///    [`ImportError::NeedsRole`].
/// 6. Only an explicit choice or a name token makes a bite. The role holds
///    [`MAX_BITE_SCANS`] scans: a third one replaces the second, and a batch
///    with more keeps the first ones.
///
/// A scan moved by one file and replaced by a later one of the batch is simply
/// replaced, by the file that took its arch: [`Allocation::moves`] never lists
/// a scan that goes away.
pub fn allocate(existing: &[Scan], files: &[FileClaim]) -> Allocation {
    allocate_with(existing, files, None)
}

/// [`allocate`] with the arches locked by the state of the module `locked`
/// (see [`roles_changed`]): a file that would replace or move an arch fails
/// with [`ImportError::ArchesLocked`], and a provisional scan counts as
/// confirmed for rule 4. Free arches and bite scans work as usual.
pub fn allocate_with(existing: &[Scan], files: &[FileClaim], locked: Option<&str>) -> Allocation {
    let mut held = BTreeMap::new();
    for role in ARCHES {
        let olds: Vec<&Scan> = existing.iter().filter(|s| s.role == role).collect();
        if !olds.is_empty() {
            held.insert(role, Holder::Case(olds));
        }
    }
    let mut slots = Slots {
        files,
        locked,
        held,
        reserved: Vec::new(),
        out: Allocation {
            decisions: vec![Decision::Skip; files.len()],
            moves: Vec::new(),
            warnings: Vec::new(),
        },
    };
    slots.claimed_arches();
    slots.bites(existing);
    let unhinted: Vec<usize> = (0..files.len())
        .filter(|&i| files[i].explicit.is_none() && files[i].name_hint.is_none())
        .collect();
    slots.detected(&unhinted);
    slots.undetected(&unhinted);
    slots.out
}

/// A prepared scan with its place in the case.
#[derive(Debug, Clone)]
pub struct Placement {
    /// The scan.
    pub scan: PreparedScan,
    /// Its role.
    pub role: ScanRole,
    /// Why it has that role.
    pub source: RoleSource,
    /// Existing scans it takes the place of.
    pub replaces: Vec<Uuid>,
}

/// A scan that entered the case.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct ImportedScan {
    /// The file.
    pub path: PathBuf,
    /// Id of the new scan.
    pub scan_id: Uuid,
    /// Its role.
    pub role: ScanRole,
    /// Why it has that role.
    pub role_source: RoleSource,
    /// File name without directory.
    pub source_name: String,
    /// Unit the file was read in.
    pub source_units: Units,
    /// The mesh blob.
    pub mesh: BlobRef,
    /// Vertex count.
    pub vertices: usize,
    /// Face count.
    pub faces: usize,
}

/// What [`commit`] did.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Committed {
    /// The new scans, in placement order.
    pub imported: Vec<ImportedScan>,
    /// Roles whose scan was replaced or moved, sorted.
    pub replaced_roles: Vec<ScanRole>,
    /// What the modules cleared, sorted, without repeats.
    pub module_cleared: Vec<&'static str>,
}

/// Applies an allocation as one state transition: moves the provisional
/// scans, inserts the new blobs and scans, drops the replaced scans, resets
/// the world frame when an arch changed, calls
/// [`ModuleData::scan_replaced`](crate::ModuleData::scan_replaced) once per
/// role replaced or moved, then drops the blobs that became unreferenced.
///
/// Without placements it does nothing. Fails with `state` unchanged when the
/// allocation changes an arch and they are locked (see [`roles_changed`]).
pub fn commit(
    state: &mut CaseState,
    placements: &[Placement],
    moves: &[ScanMove],
) -> Result<Committed, ImportError> {
    let scans = &state.case.scans;
    let moved = moves.iter().any(|m| scans.iter().any(|s| s.id == m.scan));
    let replaced = placements.iter().flat_map(|p| &p.replaces).any(|id| {
        scans
            .iter()
            .any(|s| s.id == *id && s.role != ScanRole::Bite)
    });
    if !placements.is_empty() && (moved || replaced) {
        check_arches(state)?;
    }
    Ok(apply(state, placements, moves))
}

fn apply(state: &mut CaseState, placements: &[Placement], moves: &[ScanMove]) -> Committed {
    if placements.is_empty() {
        return Committed::default();
    }
    let before = state.referenced_blobs();
    let mut touched = BTreeSet::new();
    for m in moves {
        if let Some(scan) = state.case.scans.iter_mut().find(|s| s.id == m.scan) {
            scan.role = m.to;
            touched.extend([m.from, m.to]);
        }
    }
    let gone: HashSet<Uuid> = placements
        .iter()
        .flat_map(|p| p.replaces.iter().copied())
        .collect();
    touched.extend(
        state
            .case
            .scans
            .iter()
            .filter_map(|s| gone.contains(&s.id).then_some(s.role)),
    );
    state.case.scans.retain(|s| !gone.contains(&s.id));

    let mut imported = Vec::with_capacity(placements.len());
    for p in placements {
        let mesh = state.blobs.insert_arc(p.scan.mesh.clone());
        let id = Uuid::new_v4();
        state.case.scans.push(Scan {
            id,
            role: p.role,
            role_source: p.source,
            source_name: p.scan.source_name.clone(),
            source_units: p.scan.source_units,
            import_xform: Pose::default(),
            mesh,
        });
        imported.push(ImportedScan {
            path: p.scan.path.clone(),
            scan_id: id,
            role: p.role,
            role_source: p.source,
            source_name: p.scan.source_name.clone(),
            source_units: p.scan.source_units,
            mesh,
            vertices: p.scan.mesh.vertex_count(),
            faces: p.scan.mesh.face_count(),
        });
    }
    state.case.scans.sort_by_key(|s| s.role);

    let module_cleared = settle_roles(state, &touched, &before);

    Committed {
        imported,
        replaced_roles: touched.into_iter().collect(),
        module_cleared,
    }
}

/// What every change of scan roles does to the rest of the case: resets the
/// world frame when an arch is among `touched`, calls
/// [`ModuleData::scan_replaced`](crate::ModuleData::scan_replaced) once per
/// role (in role order), then drops the blobs that were in `before` (see
/// [`CaseState::referenced_blobs`], taken before the change) and are no
/// longer referenced. Returns what the modules cleared, sorted, without
/// repeats.
///
/// [`commit`] does the same; any other command that moves or replaces scans
/// (e.g. `case.set-role`) ends with this.
///
/// An opaque module state (schema newer than this build, or module not
/// installed) has no [`ModuleData::scan_replaced`](crate::ModuleData::scan_replaced)
/// of its own, so its results would stay next to the new mesh. With an arch
/// among `touched` that is refused with [`ImportError::ArchesLocked`], before
/// `state` changes: the caller drops the changes it made so far. Bite scans
/// stay free, and so does adding a scan to an empty arch, which touches no role.
pub fn roles_changed(
    state: &mut CaseState,
    touched: &BTreeSet<ScanRole>,
    before: &HashSet<BlobRef>,
) -> Result<Vec<&'static str>, ImportError> {
    if touched.iter().any(|r| *r != ScanRole::Bite) {
        check_arches(state)?;
    }
    Ok(settle_roles(state, touched, before))
}

/// The module that locks the arches, if any.
fn opaque_module(state: &CaseState) -> Option<&str> {
    let mut modules = state.modules.iter();
    modules.find_map(|(id, m)| m.is_opaque().then_some(id.as_str()))
}

fn check_arches(state: &CaseState) -> Result<(), ImportError> {
    opaque_module(state).map_or(Ok(()), |module| {
        let module = module.to_owned();
        Err(ImportError::ArchesLocked { module })
    })
}

fn settle_roles(
    state: &mut CaseState,
    touched: &BTreeSet<ScanRole>,
    before: &HashSet<BlobRef>,
) -> Vec<&'static str> {
    if touched.iter().any(|r| *r != ScanRole::Bite) {
        state.case.world_frame = Default::default();
    }
    let mut cleared = BTreeSet::new();
    for &role in touched {
        for module in state.modules.values_mut() {
            cleared.extend(module.data_mut().scan_replaced(role));
        }
    }
    drop_unreferenced(state, before);
    cleared.into_iter().collect()
}

/// Drops the blobs that were in `before` (taken before a change) and are no
/// longer referenced; blobs added since stay.
fn drop_unreferenced(state: &mut CaseState, before: &HashSet<BlobRef>) {
    let after = state.referenced_blobs();
    let keep = state
        .blobs
        .iter()
        .map(|(r, _)| *r)
        .filter(|r| after.contains(r) || !before.contains(r))
        .collect();
    state.blobs.retain_only(&keep);
}

/// What [`remove_scan`] did.
#[derive(Debug, Clone, PartialEq)]
pub struct Removed {
    /// The scan that left the case.
    pub scan: Scan,
    /// The world frame was reset (an arch scan left).
    pub world_frame_reset: bool,
    /// What the modules cleared, sorted, without repeats.
    pub module_cleared: Vec<&'static str>,
}

/// Removes the scan `id` from the case as one state transition, as a
/// replacement does with the scan it replaces: resets the world frame when
/// it was an arch scan, calls
/// [`ModuleData::scan_removed`](crate::ModuleData::scan_removed) on every
/// module, then drops the blobs that became unreferenced (its mesh, unless
/// another scan or a module still refers to the same blob).
///
/// `Ok(None)` with `state` unchanged when the case has no scan `id`.
/// Removing an arch scan is refused, as replacing one is, while a module
/// state is opaque ([`ImportError::ArchesLocked`]), before `state` changes;
/// a bite scan stays free.
pub fn remove_scan(state: &mut CaseState, id: Uuid) -> Result<Option<Removed>, ImportError> {
    let Some(pos) = state.case.scans.iter().position(|s| s.id == id) else {
        return Ok(None);
    };
    let world_frame_reset = state.case.scans[pos].role != ScanRole::Bite;
    if world_frame_reset {
        check_arches(state)?;
    }
    let before = state.referenced_blobs();
    let scan = state.case.scans.remove(pos);
    if world_frame_reset {
        state.case.world_frame = Default::default();
    }
    let mut cleared = BTreeSet::new();
    for module in state.modules.values_mut() {
        cleared.extend(module.data_mut().scan_removed(&scan));
    }
    drop_unreferenced(state, &before);
    Ok(Some(Removed {
        scan,
        world_frame_reset,
        module_cleared: cleared.into_iter().collect(),
    }))
}

/// One file to import.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ImportFile {
    /// The file.
    pub path: PathBuf,
    /// Role chosen by the caller.
    pub role: Option<ScanRole>,
}

/// A batch of files.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct ImportRequest {
    /// The files.
    pub files: Vec<ImportFile>,
}

impl ImportRequest {
    /// Batch of files whose role is left to the pipeline.
    pub fn from_paths(paths: impl IntoIterator<Item = impl Into<PathBuf>>) -> Self {
        let files = paths
            .into_iter()
            .map(|p| ImportFile {
                path: p.into(),
                role: None,
            })
            .collect();
        Self { files }
    }
}

/// Recognises the arch of a scan from its geometry. Called only for the files
/// with neither an explicit role nor a name hint.
pub type Resolver<'a> = dyn FnMut(&PreparedScan) -> Option<Detection> + 'a;

/// A file that was not imported.
#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub struct FailedImport {
    /// The file.
    pub path: PathBuf,
    /// Stable identifier ([`ImportError::code`]).
    pub code: &'static str,
    /// The reason in the language current at the time of the import.
    pub reason: String,
    /// Technical cause, untranslated, when there is one.
    pub detail: Option<String>,
}

impl FailedImport {
    fn new(path: &Path, error: &ImportError) -> Self {
        Self {
            path: path.to_owned(),
            code: error.code(),
            reason: error.message(),
            detail: error.detail().map(str::to_owned),
        }
    }
}

/// Faces removed from the imported files.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize)]
pub struct CleanupCounts {
    /// Faces with a repeated vertex.
    pub degenerate: usize,
    /// Faces repeating an earlier one.
    pub duplicate: usize,
}

/// Result of [`import_files`].
#[derive(Debug, Clone, Default, PartialEq, Serialize)]
pub struct ImportReport {
    /// Scans that entered the case, in request order.
    pub imported: Vec<ImportedScan>,
    /// Files that were not imported: unreadable or unusable ones first, then
    /// the ones that found no place.
    pub failed: Vec<FailedImport>,
    /// Warnings about the imported and the skipped files.
    pub warnings: Vec<ImportWarning>,
    /// Roles whose scan was replaced or moved.
    pub replaced_roles: Vec<ScanRole>,
    /// Provisional scans moved to the other arch.
    pub moved: Vec<ScanMove>,
    /// What the modules cleared because of the replacement.
    pub module_cleared: Vec<&'static str>,
    /// Faces removed from the imported files.
    pub cleanup: CleanupCounts,
    /// The job was cancelled: nothing changed, the rest of the report is empty.
    pub cancelled: bool,
}

impl ImportReport {
    fn cancelled() -> Self {
        Self {
            cancelled: true,
            ..Self::default()
        }
    }
}

/// Imports a batch into `state`: the single entry point of the import.
///
/// A failed file does not stop the batch. When nothing is imported or the job
/// is cancelled, `state` stays as it was. Callers holding a shared state clone
/// it first and keep the old one for undo.
///
/// A bite scan, known from the explicit role or the file name, is read with
/// [`PrepareOptions::KEEP_UNITS`].
pub fn import_files(
    state: &mut CaseState,
    request: &ImportRequest,
    mut resolver: Option<&mut Resolver<'_>>,
    ctx: &JobCtx,
) -> ImportReport {
    crate::loader();
    let mut report = ImportReport::default();
    let mut prepared = Vec::with_capacity(request.files.len());
    let total = request.files.len().max(1) as f32;
    for (i, file) in request.files.iter().enumerate() {
        ctx.set_progress(i as f32 / total, Some(PROGRESS_KEY));
        let role = file.role.or_else(|| role_from_name(&file_name(&file.path)));
        let options = if role == Some(ScanRole::Bite) {
            PrepareOptions::KEEP_UNITS
        } else {
            PrepareOptions::default()
        };
        match prepare_with(&file.path, &options, ctx) {
            Ok(scan) => prepared.push((scan, file.role)),
            Err(ImportError::Cancelled) => return ImportReport::cancelled(),
            Err(e) => report.failed.push(FailedImport::new(&file.path, &e)),
        }
    }

    let mut claims = Vec::with_capacity(prepared.len());
    for (scan, explicit) in &prepared {
        let name_hint = role_from_name(&scan.source_name);
        let detection = match (explicit, name_hint, resolver.as_mut()) {
            (None, None, Some(resolve)) => resolve(scan),
            _ => None,
        };
        if ctx.is_cancelled() {
            return ImportReport::cancelled();
        }
        claims.push(FileClaim {
            name: scan.source_name.clone(),
            explicit: *explicit,
            name_hint,
            detection,
        });
    }

    let allocation = allocate_with(&state.case.scans, &claims, opaque_module(state));
    let mut placements = Vec::with_capacity(prepared.len());
    for ((scan, _), decision) in prepared.into_iter().zip(allocation.decisions) {
        match decision {
            Decision::Place {
                role,
                source,
                replaces,
            } => {
                report.warnings.extend(scan.warnings.iter().cloned());
                report.cleanup.degenerate += scan.cleanup.degenerate;
                report.cleanup.duplicate += scan.cleanup.duplicate;
                placements.push(Placement {
                    scan,
                    role,
                    source,
                    replaces,
                });
            }
            Decision::Skip => {}
            Decision::Fail(e) => report.failed.push(FailedImport::new(&scan.path, &e)),
        }
    }
    report.warnings.extend(allocation.warnings);
    // `allocate_with` has already kept the files off the locked arches
    let done = apply(state, &placements, &allocation.moves);
    report.imported = done.imported;
    report.replaced_roles = done.replaced_roles;
    report.module_cleared = done.module_cleared;
    report.moved = allocation.moves;
    ctx.set_progress(1.0, None);
    report
}
