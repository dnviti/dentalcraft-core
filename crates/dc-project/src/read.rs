//! `.dcprj` reader: verification, migration and decoding.

use std::collections::BTreeSet;
use std::fs::File;
use std::io::{BufReader, Read, Seek};
use std::path::Path;
use std::sync::Arc;

use dc_case::{BlobRef, BlobStore, Case, CaseState, ModuleState, OpaqueModuleData};
use dc_mesh::dcm;
use serde_json::Value;
use zip::ZipArchive;
use zip::result::ZipError;

use crate::migrate::{ContainerDoc, migrate_container};
use crate::{
    CASE_PATH, DcprjError, EntryInfo, FORMAT, FORMAT_VERSION, HISTORY_PATH, LoadIssue, LoadOutcome,
    MANIFEST_PATH, MIME_TYPE, MIMETYPE_PATH, Manifest, ModuleCodecs, Project, THUMBNAIL_PATH,
    module_state_path, valid_module_id,
};

/// Upper bound for an entry that is not listed in the manifest.
const UNLISTED_LIMIT: u64 = 64 << 20;

/// Largest pre-allocation for one entry: sizes come from the file, so a
/// larger entry grows its buffer while it is actually read.
const PREALLOC_CAP: u64 = 64 << 20;

/// Largest uncompressed size of one entry (2 GiB); a file declaring more is
/// refused before anything is read.
pub const MAX_ENTRY_BYTES: u64 = 2 << 30;

/// Largest uncompressed size of all entries together (16 GiB). The total is
/// also bounded by 1100 times the file length (deflate's limit is ~1032:1).
pub const MAX_TOTAL_BYTES: u64 = 16 << 30;

/// Deflate cannot expand more than about 1032:1, so a project declaring
/// more than this many times its file length is lying about its sizes.
const MAX_RATIO: u64 = 1100;

fn zip_err(e: ZipError) -> DcprjError {
    match e {
        ZipError::Io(e) => DcprjError::Io(e),
        e => DcprjError::Zip(e.to_string()),
    }
}

/// Reads at most `limit + 1` bytes of entry `name`. With `exact`, an entry
/// whose ZIP size is not `limit` (the manifest size) is refused.
fn read_entry<R: Read + Seek>(
    ar: &mut ZipArchive<R>,
    name: &str,
    limit: u64,
    exact: bool,
) -> Result<Vec<u8>, String> {
    let entry = ar.by_name(name).map_err(|e| e.to_string())?;
    if exact && entry.size() != limit {
        return Err(format!(
            "size {} differs from the manifest ({limit} bytes)",
            entry.size()
        ));
    }
    let mut buf = Vec::with_capacity(entry.size().min(limit).min(PREALLOC_CAP) as usize);
    entry
        .take(limit.saturating_add(1))
        .read_to_end(&mut buf)
        .map_err(|e| e.to_string())?;
    Ok(buf)
}

/// Reads and verifies a listed entry. Problems become issues and `None`.
fn read_verified<R: Read + Seek>(
    ar: &mut ZipArchive<R>,
    name: &str,
    info: &EntryInfo,
    issues: &mut Vec<LoadIssue>,
) -> Option<Vec<u8>> {
    match read_entry(ar, name, info.bytes, true) {
        Err(reason) => {
            issues.push(LoadIssue::UnreadableEntry {
                path: name.to_owned(),
                reason,
            });
            None
        }
        Ok(bytes)
            if bytes.len() as u64 != info.bytes || BlobRef::of_bytes(&bytes) != info.blake3 =>
        {
            issues.push(LoadIssue::HashMismatch(name.to_owned()));
            None
        }
        Ok(bytes) => Some(bytes),
    }
}

fn corrupt(msg: impl Into<String>) -> DcprjError {
    DcprjError::Corrupt(msg.into())
}

pub(crate) fn load(path: &Path, codecs: &ModuleCodecs) -> Result<LoadOutcome, DcprjError> {
    let file = File::open(path)?;
    let file_len = file.metadata()?.len();
    let mut ar = ZipArchive::new(BufReader::new(file)).map_err(zip_err)?;
    let mut issues = Vec::new();

    // `mimetype` must be the first entry, stored, with the exact content.
    {
        let mut first = ar.by_index(0).map_err(|_| DcprjError::NotAProject)?;
        if first.name_raw() != MIMETYPE_PATH.as_bytes()
            || first.compression() != zip::CompressionMethod::Stored
            || first.size() != MIME_TYPE.len() as u64
        {
            return Err(DcprjError::NotAProject);
        }
        let mut s = String::new();
        first
            .read_to_string(&mut s)
            .map_err(|_| DcprjError::NotAProject)?;
        if s != MIME_TYPE {
            return Err(DcprjError::NotAProject);
        }
    }

    let manifest_bytes = read_entry(&mut ar, MANIFEST_PATH, UNLISTED_LIMIT, false)
        .map_err(|_| DcprjError::MissingEntry(MANIFEST_PATH.to_owned()))?;
    if manifest_bytes.len() as u64 > UNLISTED_LIMIT {
        return Err(corrupt("manifest.json is too large"));
    }
    let mut manifest_value: Value = serde_json::from_slice(&manifest_bytes)?;
    if manifest_value.get("format").and_then(Value::as_str) != Some(FORMAT) {
        return Err(DcprjError::NotAProject);
    }
    let found = manifest_value
        .get("format_version")
        .and_then(Value::as_u64)
        .and_then(|v| u32::try_from(v).ok())
        .ok_or_else(|| corrupt("manifest.format_version is missing or invalid"))?;
    let newer = found > FORMAT_VERSION;
    if newer {
        issues.push(LoadIssue::NewerFormat {
            found,
            supported: FORMAT_VERSION,
        });
    }

    // Entry inventory (hashes are checked on the raw bytes, before migration).
    let entries_value = manifest_value
        .get("entries")
        .cloned()
        .ok_or_else(|| corrupt("manifest.entries is missing"))?;
    let entries: std::collections::BTreeMap<String, EntryInfo> =
        serde_json::from_value(entries_value)?;
    // The zip crate keeps one entry per name: ask it whether some were
    // dropped, then guard against names equal only after lossy decoding.
    if ar.has_duplicate_names() {
        return Err(corrupt("duplicate archive entry names"));
    }
    let raw_names: Vec<String> = ar
        .file_names_raw()
        .map(|n| String::from_utf8_lossy(n).into_owned())
        .collect();
    let names: BTreeSet<String> = raw_names.iter().cloned().collect();
    if names.len() != raw_names.len() {
        return Err(corrupt("duplicate archive entry names"));
    }
    // Declared sizes bound every read below: refuse impossible ones first.
    let mut total: u64 = 0;
    for (name, info) in entries.iter().filter(|(n, _)| names.contains(*n)) {
        if info.bytes > MAX_ENTRY_BYTES {
            return Err(corrupt(format!(
                "{name} declares {} bytes, more than the limit of {MAX_ENTRY_BYTES}",
                info.bytes
            )));
        }
        total = total.saturating_add(info.bytes);
    }
    let total_limit = MAX_TOTAL_BYTES.min(file_len.saturating_mul(MAX_RATIO));
    if total > total_limit {
        return Err(corrupt(format!(
            "entries declare {total} bytes in total, more than the limit of {total_limit}"
        )));
    }
    for name in &names {
        if name != MIMETYPE_PATH && name != MANIFEST_PATH && !entries.contains_key(name) {
            issues.push(LoadIssue::UnlistedEntry(name.clone()));
        }
    }
    for name in entries.keys() {
        if !names.contains(name) {
            issues.push(LoadIssue::MissingEntry(name.clone()));
        }
    }
    let listed =
        |name: &str, names: &BTreeSet<String>| entries.get(name).filter(|_| names.contains(name));

    // Case: unreadable or corrupt means the file is unusable.
    let case_info =
        listed(CASE_PATH, &names).ok_or_else(|| DcprjError::MissingEntry(CASE_PATH.to_owned()))?;
    let case_bytes = match read_entry(&mut ar, CASE_PATH, case_info.bytes, true) {
        Ok(b) => b,
        Err(reason) => return Err(corrupt(format!("{CASE_PATH}: {reason}"))),
    };
    if case_bytes.len() as u64 != case_info.bytes
        || BlobRef::of_bytes(&case_bytes) != case_info.blake3
    {
        issues.push(LoadIssue::HashMismatch(CASE_PATH.to_owned()));
    }
    let case_value: Value = serde_json::from_slice(&case_bytes)?;

    // History (optional).
    let mut history_value = None;
    if let Some(info) = listed(HISTORY_PATH, &names)
        && let Some(bytes) = read_verified(&mut ar, HISTORY_PATH, info, &mut issues)
    {
        match serde_json::from_slice(&bytes) {
            Ok(v) => history_value = Some(v),
            Err(e) => issues.push(LoadIssue::UnreadableEntry {
                path: HISTORY_PATH.to_owned(),
                reason: e.to_string(),
            }),
        }
    }

    // Container migrations.
    let mut doc = ContainerDoc {
        manifest: std::mem::take(&mut manifest_value),
        case: case_value,
        history: history_value,
    };
    if !newer {
        migrate_container(&mut doc, found)?;
    }
    let as_newer = |e: serde_json::Error| {
        if newer {
            DcprjError::UnsupportedVersion {
                found,
                supported: FORMAT_VERSION,
            }
        } else {
            DcprjError::Json(e)
        }
    };
    let manifest: Manifest = serde_json::from_value(doc.manifest).map_err(as_newer)?;
    let case: Case = serde_json::from_value(doc.case).map_err(as_newer)?;
    let history = doc.history.unwrap_or_else(|| Value::Array(Vec::new()));

    // Modules.
    let mut state = CaseState::new(case);
    for module in &manifest.modules {
        if !valid_module_id(&module.id) {
            return Err(corrupt(format!("invalid module id {:?}", module.id)));
        }
        let path = module_state_path(&module.id);
        let Some(info) = listed(&path, &names) else {
            continue; // already reported as missing
        };
        let Some(bytes) = read_verified(&mut ar, &path, info, &mut issues) else {
            continue;
        };
        let value: Value = match serde_json::from_slice(&bytes) {
            Ok(v) => v,
            Err(e) => {
                issues.push(LoadIssue::UnreadableEntry {
                    path,
                    reason: e.to_string(),
                });
                continue;
            }
        };
        let opaque = |value: Value| {
            ModuleState::new(OpaqueModuleData {
                schema_version: module.schema_version,
                value,
            })
        };
        let module_state = match codecs.get(&module.id) {
            None => {
                issues.push(LoadIssue::ModuleKeptOpaque {
                    id: module.id.clone(),
                });
                opaque(value)
            }
            Some(codec) if module.schema_version > codec.current_version() => {
                issues.push(LoadIssue::ModuleTooNew {
                    id: module.id.clone(),
                    found: module.schema_version,
                    supported: codec.current_version(),
                });
                opaque(value)
            }
            Some(codec) => match codec.decode_from(value.clone(), module.schema_version) {
                Ok(data) => ModuleState::from_box(data),
                Err((_, reason)) => {
                    issues.push(LoadIssue::ModuleDecodeFailed {
                        id: module.id.clone(),
                        reason,
                    });
                    opaque(value)
                }
            },
        };
        state.set_module(module.id.clone(), module_state);
    }

    // Blobs.
    let mut blobs = BlobStore::new();
    for (name, info) in &entries {
        let Some(hex) = name
            .strip_prefix("blobs/")
            .and_then(|n| n.strip_suffix(".dcm"))
        else {
            continue;
        };
        if !names.contains(name) {
            continue;
        }
        if hex != info.blake3.to_hex() {
            issues.push(LoadIssue::HashMismatch(name.clone()));
            continue;
        }
        let Some(bytes) = read_verified(&mut ar, name, info, &mut issues) else {
            continue;
        };
        match dcm::decode(&bytes) {
            Ok(mesh) => blobs.insert_verified(info.blake3, Arc::new(mesh)),
            Err(e) => issues.push(LoadIssue::BlobDecode {
                path: name.clone(),
                reason: e.to_string(),
            }),
        }
    }
    // Every mesh a scan or a module refers to (sorted: issues are stable).
    let mut referenced: Vec<BlobRef> = state.referenced_blobs().into_iter().collect();
    referenced.sort_unstable();
    for r in referenced {
        if !blobs.contains(&r) {
            issues.push(LoadIssue::MissingBlob(r));
        }
    }
    state.blobs = blobs;

    // Thumbnail (optional, not essential: corruption is reported, not fatal).
    let thumbnail = listed(THUMBNAIL_PATH, &names)
        .and_then(|info| read_verified(&mut ar, THUMBNAIL_PATH, info, &mut issues));

    let read_only = issues.iter().any(LoadIssue::blocks_writing);
    Ok(LoadOutcome {
        project: Project {
            manifest,
            case_state: state,
            history,
            thumbnail,
        },
        read_only,
        issues,
    })
}
