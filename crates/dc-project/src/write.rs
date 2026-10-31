//! Deterministic, atomic `.dcprj` writer.

use std::collections::BTreeMap;
use std::fs::{self, File};
use std::io::{BufWriter, Write};
use std::path::{Path, PathBuf};

use dc_case::BlobRef;
use dc_mesh::dcm;
use zip::write::SimpleFileOptions;
use zip::{CompressionMethod, DateTime, ZipWriter};

use crate::{
    CASE_PATH, DcprjError, EntryInfo, FORMAT, FORMAT_VERSION, HISTORY_PATH, MANIFEST_PATH,
    MIME_TYPE, MIMETYPE_PATH, Manifest, ModuleEntry, Project, THUMBNAIL_PATH, blob_path,
    module_state_path, valid_module_id,
};

const JSON_LEVEL: i64 = 6;
const BLOB_LEVEL: i64 = 3;

fn options(method: CompressionMethod, level: Option<i64>, bytes: u64) -> SimpleFileOptions {
    SimpleFileOptions::default()
        .compression_method(method)
        .compression_level(level)
        // Fixed timestamp and permissions: the output only depends on the content.
        .last_modified_time(DateTime::default())
        .unix_permissions(0o644)
        .large_file(bytes >= u64::from(u32::MAX))
}

fn json_bytes(v: &impl serde::Serialize) -> Result<Vec<u8>, DcprjError> {
    Ok(serde_json::to_vec(v)?)
}

fn info(bytes: &[u8]) -> EntryInfo {
    EntryInfo {
        blake3: BlobRef::of_bytes(bytes),
        bytes: bytes.len() as u64,
    }
}

/// Atomic file replacement path: `<path>` with `suffix` appended to the name.
fn sibling(path: &Path, suffix: &str) -> PathBuf {
    let mut name = path.file_name().unwrap_or_default().to_os_string();
    name.push(suffix);
    path.with_file_name(name)
}

pub(crate) fn save(path: &Path, project: &Project) -> Result<(), DcprjError> {
    let tmp = sibling(path, ".tmp");
    let result = write_tmp(&tmp, project).and_then(|()| replace(path, &tmp));
    if result.is_err() {
        let _ = fs::remove_file(&tmp);
    }
    result
}

/// Writes the archive to `tmp` and fsyncs it.
fn write_tmp(tmp: &Path, project: &Project) -> Result<(), DcprjError> {
    let state = &project.case_state;

    // Small JSON entries are kept in memory; blobs are encoded one at a time.
    let mut json: Vec<(String, Vec<u8>)> = Vec::new();
    json.push((CASE_PATH.to_owned(), json_bytes(&state.case)?));
    let mut modules = Vec::new();
    for (id, module) in &state.modules {
        if !valid_module_id(id) {
            return Err(DcprjError::InvalidModuleId(id.clone()));
        }
        let version = project
            .manifest
            .modules
            .iter()
            .find(|m| &m.id == id)
            .map_or_else(
                || project.manifest.app.version.clone(),
                |m| m.version.clone(),
            );
        modules.push(ModuleEntry {
            id: id.clone(),
            version,
            schema_version: module.data().schema_version(),
        });
        json.push((module_state_path(id), json_bytes(&module.to_json()?)?));
    }
    json.push((HISTORY_PATH.to_owned(), json_bytes(&project.history)?));

    let mut entries: BTreeMap<String, EntryInfo> = BTreeMap::new();
    for (name, bytes) in &json {
        entries.insert(name.clone(), info(bytes));
    }
    if let Some(t) = &project.thumbnail {
        entries.insert(THUMBNAIL_PATH.to_owned(), info(t));
    }

    // Pass 1: size every blob (the manifest comes before them in the archive).
    // Blobs that no scan and no module references are garbage: not written.
    let referenced = state.referenced_blobs();
    let mut blobs: Vec<(BlobRef, u64)> = Vec::new();
    let mut refs: Vec<_> = state
        .blobs
        .iter()
        .filter(|(r, _)| referenced.contains(*r))
        .collect();
    refs.sort_by_key(|(r, _)| **r);
    for (r, mesh) in refs {
        let enc = dcm::encode(mesh);
        if BlobRef::of_bytes(&enc) != *r {
            return Err(DcprjError::HashMismatch(blob_path(r)));
        }
        entries.insert(
            blob_path(r),
            EntryInfo {
                blake3: *r,
                bytes: enc.len() as u64,
            },
        );
        blobs.push((*r, enc.len() as u64));
    }

    let manifest = Manifest {
        format: FORMAT.to_owned(),
        format_version: FORMAT_VERSION,
        modules,
        entries,
        ..project.manifest.clone()
    };

    let file = File::create(tmp)?;
    let mut zw = ZipWriter::new(BufWriter::new(file));
    let deflate = CompressionMethod::Deflated;

    zw.start_file(MIMETYPE_PATH, options(CompressionMethod::Stored, None, 0))?;
    zw.write_all(MIME_TYPE.as_bytes())?;

    let manifest_bytes = serde_json::to_vec_pretty(&manifest)?;
    zw.start_file(MANIFEST_PATH, options(deflate, Some(JSON_LEVEL), 0))?;
    zw.write_all(&manifest_bytes)?;

    for (name, bytes) in &json {
        zw.start_file(name, options(deflate, Some(JSON_LEVEL), 0))?;
        zw.write_all(bytes)?;
    }

    // Pass 2: encode again and stream each blob straight into the archive.
    for (r, bytes) in blobs {
        let mesh = state.blobs.get(&r).expect("blob listed in pass 1");
        let enc = dcm::encode(mesh);
        zw.start_file(blob_path(&r), options(deflate, Some(BLOB_LEVEL), bytes))?;
        zw.write_all(&enc)?;
    }

    if let Some(t) = &project.thumbnail {
        // PNG data is already compressed.
        zw.start_file(
            THUMBNAIL_PATH,
            options(CompressionMethod::Stored, None, t.len() as u64),
        )?;
        zw.write_all(t)?;
    }

    let file = zw.finish()?.into_inner().map_err(|e| e.into_error())?;
    file.sync_all()?;
    Ok(())
}

/// Keeps the previous file as `<path>.bak` and moves `tmp` into place.
fn replace(path: &Path, tmp: &Path) -> Result<(), DcprjError> {
    if path.exists() {
        let bak = sibling(path, ".bak");
        match fs::remove_file(&bak) {
            Ok(()) => {}
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => {}
            Err(e) => return Err(e.into()),
        }
        // A hard link keeps `path` in place until the final rename; fall back
        // to a copy where links are unsupported.
        if fs::hard_link(path, &bak).is_err() {
            fs::copy(path, &bak)?;
        }
    }
    fs::rename(tmp, path)?;
    sync_dir(path)?;
    Ok(())
}

#[cfg(unix)]
fn sync_dir(path: &Path) -> std::io::Result<()> {
    let dir = match path.parent() {
        Some(p) if !p.as_os_str().is_empty() => p,
        _ => Path::new("."),
    };
    File::open(dir)?.sync_all()
}

#[cfg(not(unix))]
fn sync_dir(_path: &Path) -> std::io::Result<()> {
    Ok(())
}
