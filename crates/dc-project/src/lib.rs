//! The `.dcprj` project file (format version 1).
//!
//! A ZIP archive whose first entry is the stored (uncompressed) `mimetype`
//! file containing [`MIME_TYPE`], followed by:
//!
//! | entry | content |
//! |---|---|
//! | `manifest.json` | [`Manifest`] |
//! | `case.json` | [`dc_case::Case`] |
//! | `modules/<id>/state.json` | module data (JSON, module schema) |
//! | `history/steps.json` | step history (the undo stack is not saved) |
//! | `blobs/<blake3>.dcm` | `DCM1` mesh blobs (deflate, see below) |
//! | `thumbnail.png` | optional preview |
//!
//! JSON entries and blobs are deflated (pure-Rust zlib-rs); the thumbnail is
//! stored. The format deliberately avoids zstd for now: the `zstd` crate
//! needs a C build and `ruzstd` has no streaming ZIP integration. Hashes in
//! [`Manifest::entries`] cover the uncompressed bytes and are verified on
//! load.
//!
//! # Guarantees
//!
//! * Output is deterministic (fixed entry order, timestamps and permissions):
//!   `save` -> `load` -> `save` yields identical bytes.
//! * [`save`] is atomic: it writes `<path>.tmp`, fsyncs it, hard-links the
//!   previous file to `<path>.bak` (one backup is kept), renames the
//!   temporary file over `<path>` and fsyncs the directory.
//! * [`load`] only fails hard when the container, the manifest or `case.json`
//!   are unusable. Any other problem (corrupt or missing blob, hash mismatch,
//!   newer module schema, ...) is reported in [`LoadOutcome::issues`] and
//!   makes the outcome [`LoadOutcome::read_only`]: callers must not overwrite
//!   such a file.
//! * Older container versions are migrated on `serde_json::Value`s;
//!   module states are migrated by their [`ModuleCodec`]. State of modules
//!   without a codec (or of a newer schema) is kept as
//!   [`dc_case::OpaqueModuleData`] and written back unchanged.

mod codec;
mod migrate;
mod read;
mod write;

use std::collections::BTreeMap;
use std::path::Path;

use serde::{Deserialize, Serialize};
use serde_json::Value;
use uuid::Uuid;

use dc_case::{BlobRef, CaseState};

pub use codec::{ModuleCodec, ModuleCodecs};

/// MIME type, stored as the first ZIP entry `mimetype`.
pub const MIME_TYPE: &str = "application/vnd.dentalcraft.project+zip";
/// File extension (without dot).
pub const FILE_EXTENSION: &str = "dcprj";
/// Value of [`Manifest::format`].
pub const FORMAT: &str = "dcprj";
/// Container format version written by this crate.
pub const FORMAT_VERSION: u32 = 1;

/// Entry holding the MIME type.
pub const MIMETYPE_PATH: &str = "mimetype";
/// Entry holding the manifest.
pub const MANIFEST_PATH: &str = "manifest.json";
/// Entry holding the case.
pub const CASE_PATH: &str = "case.json";
/// Entry holding the step history.
pub const HISTORY_PATH: &str = "history/steps.json";
/// Entry holding the thumbnail.
pub const THUMBNAIL_PATH: &str = "thumbnail.png";

/// Entry holding the state of module `id`.
pub fn module_state_path(id: &str) -> String {
    format!("modules/{id}/state.json")
}

/// Entry holding blob `r`.
pub fn blob_path(r: &BlobRef) -> String {
    format!("blobs/{}.dcm", r.to_hex())
}

/// Errors reading or writing a project.
#[derive(Debug, thiserror::Error)]
pub enum DcprjError {
    /// File system error.
    #[error(transparent)]
    Io(#[from] std::io::Error),
    /// Malformed JSON entry.
    #[error(transparent)]
    Json(#[from] serde_json::Error),
    /// Not a `.dcprj` file (missing or wrong `mimetype`).
    #[error("not a DentalCraft project")]
    NotAProject,
    /// Written by a newer version: open read-only.
    #[error("format version {found} is newer than the supported {supported}")]
    UnsupportedVersion {
        /// Version in the file.
        found: u32,
        /// Newest version this build understands.
        supported: u32,
    },
    /// A required entry is missing.
    #[error("missing entry {0}")]
    MissingEntry(String),
    /// An entry's content does not match its BLAKE3 hash.
    #[error("corrupt entry {0}: hash mismatch")]
    HashMismatch(String),
    /// A schema migration failed.
    #[error("migration of {container} from version {from} failed: {reason}")]
    Migration {
        /// `project` or the module id.
        container: String,
        /// Version migrated from.
        from: u32,
        /// Reason.
        reason: String,
    },
    /// Error from a mesh blob.
    #[error(transparent)]
    Mesh(#[from] dc_mesh::MeshError),
    /// ZIP container error (truncated or malformed archive).
    #[error("invalid project archive: {0}")]
    Zip(String),
    /// A required JSON document has an unusable structure.
    #[error("corrupt project: {0}")]
    Corrupt(String),
    /// Module id that cannot be used as an archive path component.
    #[error("invalid module id {0:?}")]
    InvalidModuleId(String),
}

impl From<zip::result::ZipError> for DcprjError {
    fn from(e: zip::result::ZipError) -> Self {
        match e {
            zip::result::ZipError::Io(e) => Self::Io(e),
            e => Self::Zip(e.to_string()),
        }
    }
}

/// `true` for ids usable in `modules/<id>/state.json`.
pub(crate) fn valid_module_id(id: &str) -> bool {
    !id.is_empty()
        && id
            .bytes()
            .all(|b| b.is_ascii_alphanumeric() || b == b'_' || b == b'-')
}

/// Application that wrote the file.
#[derive(Debug, Clone, PartialEq, Eq, Default, Serialize, Deserialize)]
pub struct AppInfo {
    /// Application version (semver).
    pub version: String,
    /// Git commit of the build.
    pub commit: String,
}

/// A module whose state is stored in the project.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ModuleEntry {
    /// Module id (e.g. `aligners`).
    pub id: String,
    /// Module version that wrote the state.
    pub version: String,
    /// Schema version of `modules/<id>/state.json`.
    pub schema_version: u32,
}

/// Size and hash of one archive entry (uncompressed).
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct EntryInfo {
    /// BLAKE3 of the uncompressed bytes.
    pub blake3: BlobRef,
    /// Uncompressed size.
    pub bytes: u64,
}

/// `manifest.json`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Manifest {
    /// Always [`FORMAT`].
    pub format: String,
    /// Container format version.
    pub format_version: u32,
    /// Stable project id.
    pub project_id: Uuid,
    /// Writer application.
    pub app: AppInfo,
    /// Creation time, RFC 3339 UTC.
    pub created: String,
    /// Last modification time, RFC 3339 UTC.
    pub modified: String,
    /// Length unit of all coordinates; always `mm`.
    pub units: String,
    /// Module states stored in the archive.
    pub modules: Vec<ModuleEntry>,
    /// Every other entry with its hash, keyed by path.
    pub entries: BTreeMap<String, EntryInfo>,
}

impl Manifest {
    /// Manifest for a new project.
    pub fn new(app: AppInfo, now_rfc3339: impl Into<String>) -> Self {
        let now = now_rfc3339.into();
        Self {
            format: FORMAT.to_owned(),
            format_version: FORMAT_VERSION,
            project_id: Uuid::new_v4(),
            app,
            created: now.clone(),
            modified: now,
            units: "mm".to_owned(),
            modules: Vec::new(),
            entries: BTreeMap::new(),
        }
    }
}

/// Everything stored in a project file.
///
/// `manifest.modules` and `manifest.entries` are derived from the content
/// when saving; the other manifest fields (ids, timestamps, writer) are
/// written as given, and `manifest.modules[..].version` is the module
/// version recorded for each module state (the application version when a
/// module has no entry).
#[derive(Debug, Clone)]
pub struct Project {
    /// The manifest.
    pub manifest: Manifest,
    /// Case, meshes and module states. Only the blobs in the store are
    /// written: drop unreferenced ones first ([`dc_case::BlobStore::retain_only`]).
    pub case_state: CaseState,
    /// Step history (JSON); `[]` when empty. The undo stack is not saved.
    pub history: Value,
    /// PNG thumbnail.
    pub thumbnail: Option<Vec<u8>>,
}

impl Project {
    /// New project for `case_state` with a fresh manifest.
    pub fn new(app: AppInfo, now_rfc3339: impl Into<String>, case_state: CaseState) -> Self {
        Self {
            manifest: Manifest::new(app, now_rfc3339),
            case_state,
            history: Value::Array(Vec::new()),
            thumbnail: None,
        }
    }
}

/// A problem found while loading a project.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum LoadIssue {
    /// The file has a newer container version than this build supports.
    NewerFormat {
        /// Version in the file.
        found: u32,
        /// Newest version this build understands.
        supported: u32,
    },
    /// An entry listed in the manifest is not in the archive.
    MissingEntry(String),
    /// An archive entry that the manifest does not list.
    UnlistedEntry(String),
    /// An entry's size or BLAKE3 hash differs from the manifest.
    HashMismatch(String),
    /// An entry could not be read or parsed.
    UnreadableEntry {
        /// Entry path.
        path: String,
        /// Reason.
        reason: String,
    },
    /// A blob passed the hash check but is not a valid `DCM1` mesh.
    BlobDecode {
        /// Entry path.
        path: String,
        /// Reason.
        reason: String,
    },
    /// A scan references a mesh that could not be loaded.
    MissingBlob(BlobRef),
    /// No codec for the module: its state is kept opaque (informational).
    ModuleKeptOpaque {
        /// Module id.
        id: String,
    },
    /// The module state has a newer schema than its codec supports: kept
    /// opaque (informational).
    ModuleTooNew {
        /// Module id.
        id: String,
        /// Schema version in the file.
        found: u32,
        /// Newest schema version the codec understands.
        supported: u32,
    },
    /// Migrating or decoding the module state failed: kept opaque.
    ModuleDecodeFailed {
        /// Module id.
        id: String,
        /// Reason.
        reason: String,
    },
}

impl LoadIssue {
    /// `true` when saving over the file could lose data.
    pub fn blocks_writing(&self) -> bool {
        !matches!(
            self,
            Self::ModuleKeptOpaque { .. } | Self::ModuleTooNew { .. }
        )
    }
}

impl std::fmt::Display for LoadIssue {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::NewerFormat { found, supported } => {
                write!(
                    f,
                    "format version {found} is newer than the supported {supported}"
                )
            }
            Self::MissingEntry(p) => write!(f, "missing entry {p}"),
            Self::UnlistedEntry(p) => write!(f, "entry {p} is not listed in the manifest"),
            Self::HashMismatch(p) => write!(f, "corrupt entry {p}: hash mismatch"),
            Self::UnreadableEntry { path, reason } => write!(f, "cannot read {path}: {reason}"),
            Self::BlobDecode { path, reason } => write!(f, "invalid mesh {path}: {reason}"),
            Self::MissingBlob(r) => write!(f, "scan mesh {r} could not be loaded"),
            Self::ModuleKeptOpaque { id } => write!(f, "module {id} is unknown: state kept as is"),
            Self::ModuleTooNew {
                id,
                found,
                supported,
            } => write!(
                f,
                "module {id} state has schema {found}, newer than the supported {supported}: kept as is"
            ),
            Self::ModuleDecodeFailed { id, reason } => {
                write!(f, "module {id} state could not be decoded: {reason}")
            }
        }
    }
}

/// Result of [`load`].
#[derive(Debug, Clone)]
pub struct LoadOutcome {
    /// The loaded project (corrupt parts are left out).
    pub project: Project,
    /// The file must not be overwritten (newer version or damaged content).
    pub read_only: bool,
    /// Everything that was found wrong, informational issues included.
    pub issues: Vec<LoadIssue>,
}

/// Writes `project` to `path` atomically (see the module docs).
pub fn save(path: &Path, project: &Project) -> Result<(), DcprjError> {
    write::save(path, project)
}

/// Reads, verifies and migrates the project at `path`; module states are
/// decoded with `codecs`.
pub fn load(path: &Path, codecs: &ModuleCodecs) -> Result<LoadOutcome, DcprjError> {
    read::load(path, codecs)
}

#[cfg(test)]
mod tests;
