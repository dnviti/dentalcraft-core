use std::io::{Read, Write};
use std::path::Path;

use dc_case::{
    BlobRef, Case, CaseState, ModuleState, OpaqueModuleData, RoleSource, Scan, ScanRole,
};
use dc_mesh::{TriMesh, Units};
use glam::Vec3;
use proptest::prelude::*;
use serde_json::{Value, json};
use zip::write::SimpleFileOptions;
use zip::{CompressionMethod, ZipArchive, ZipWriter};

use serde::{Deserialize, Serialize};

use super::*;

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
struct Demo {
    stages: u32,
    name: String,
}
dc_case::impl_module_data!(Demo, 1);

fn codecs() -> ModuleCodecs {
    let mut c = ModuleCodecs::new();
    c.register("demo", ModuleCodec::typed::<Demo>(1));
    c
}

fn mesh(seed: f32) -> TriMesh {
    let mut m = TriMesh::new(
        vec![
            Vec3::new(seed, 0.0, 0.0),
            Vec3::new(1.0, seed, 0.5),
            Vec3::new(0.0, 1.0, seed * 2.0),
            Vec3::new(1.0, 1.0, 1.0),
        ],
        vec![[0, 1, 2], [1, 3, 2]],
    );
    m.vertex_labels = Some(vec![0, 1, 2, 3]);
    m
}

fn app() -> AppInfo {
    AppInfo {
        version: "0.1.0".into(),
        commit: "abc123".into(),
    }
}

fn sample_project() -> Project {
    let mut state = CaseState::new(Case::new("P-0001"));
    for (i, role) in [ScanRole::Upper, ScanRole::Lower].into_iter().enumerate() {
        let r = state.blobs.insert(mesh(i as f32 + 0.25));
        state.case.scans.push(Scan {
            id: uuid::Uuid::from_u128(i as u128 + 1),
            role,
            role_source: RoleSource::Explicit,
            source_name: format!("scan{i}.stl"),
            source_units: Units::Millimeters,
            import_xform: Default::default(),
            mesh: r,
        });
    }
    state.set_module(
        "demo",
        ModuleState::new(Demo {
            stages: 7,
            name: "x".into(),
        }),
    );
    state.set_module(
        "future",
        ModuleState::new(OpaqueModuleData {
            schema_version: 9,
            value: json!({"a": [1, 2, {"b": null}], "z": 1.5}),
        }),
    );
    let mut p = Project::new(app(), "2026-10-31T10:00:00Z", state);
    p.manifest.project_id = uuid::Uuid::from_u128(42);
    p.history = json!([{"step": 1}]);
    p.thumbnail = Some(b"\x89PNG fake".to_vec());
    p
}

fn dir() -> tempfile::TempDir {
    tempfile::tempdir().unwrap()
}

/// All entries as (name, uncompressed content), in archive order.
fn read_all(path: &Path) -> Vec<(String, Vec<u8>)> {
    let mut ar = ZipArchive::new(std::fs::File::open(path).unwrap()).unwrap();
    (0..ar.len())
        .map(|i| {
            let mut f = ar.by_index(i).unwrap();
            let mut b = Vec::new();
            f.read_to_end(&mut b).unwrap();
            (f.name().unwrap().into_owned(), b)
        })
        .collect()
}

/// Writes a raw archive (`mimetype` first), without touching the manifest.
fn write_zip(path: &Path, files: &[(String, Vec<u8>)]) {
    let mut zw = ZipWriter::new(std::fs::File::create(path).unwrap());
    zw.start_file(
        MIMETYPE_PATH,
        SimpleFileOptions::default().compression_method(CompressionMethod::Stored),
    )
    .unwrap();
    zw.write_all(MIME_TYPE.as_bytes()).unwrap();
    for (n, b) in files {
        if n == MIMETYPE_PATH {
            continue;
        }
        zw.start_file(n, SimpleFileOptions::default()).unwrap();
        zw.write_all(b).unwrap();
    }
    zw.finish().unwrap();
}

/// Builds a consistent archive from a manifest (without `entries`) and files.
fn build_zip(path: &Path, mut manifest: Value, files: &[(&str, Vec<u8>)]) {
    let entries: serde_json::Map<String, Value> = files
        .iter()
        .map(|(n, b)| {
            (
                (*n).to_owned(),
                json!({"blake3": BlobRef::of_bytes(b).to_hex(), "bytes": b.len()}),
            )
        })
        .collect();
    manifest["entries"] = Value::Object(entries);
    let mut all = vec![(
        MANIFEST_PATH.to_owned(),
        serde_json::to_vec(&manifest).unwrap(),
    )];
    all.extend(files.iter().map(|(n, b)| ((*n).to_owned(), b.clone())));
    write_zip(path, &all);
}

fn base_manifest(version: u32) -> Value {
    json!({
        "format": "dcprj",
        "format_version": version,
        "project_id": uuid::Uuid::from_u128(1),
        "app": {"version": "0.0.1", "commit": "old"},
        "created": "2026-01-01T00:00:00Z",
        "modified": "2026-01-02T00:00:00Z",
        "units": "mm",
        "modules": [],
    })
}

fn case_json() -> Vec<u8> {
    serde_json::to_vec(&Case::new("P-9")).unwrap()
}

fn blob_entry_of(project: &Project) -> String {
    let r = project.case_state.case.scans[0].mesh;
    blob_path(&r)
}

#[test]
fn two_bite_scans_round_trip_in_order() {
    let d = dir();
    let path = d.path().join("bites.dcprj");
    let mut p = sample_project();
    for (i, side) in ["left", "right"].into_iter().enumerate() {
        let state = &mut p.case_state;
        let mesh = state.blobs.insert(mesh(i as f32 + 5.0));
        state.case.scans.push(Scan {
            id: uuid::Uuid::from_u128(i as u128 + 100),
            role: ScanRole::Bite,
            role_source: RoleSource::FileName,
            source_name: format!("bite_{side}.stl"),
            source_units: Units::Millimeters,
            import_xform: Default::default(),
            mesh,
        });
    }
    save(&path, &p).unwrap();
    let out = load(&path, &codecs()).unwrap();
    assert!(!out.read_only, "{:?}", out.issues);
    let case = &out.project.case_state.case;
    assert_eq!(*case, p.case_state.case);
    let bites: Vec<&str> = case
        .scans_of(ScanRole::Bite)
        .map(|s| s.source_name.as_str())
        .collect();
    assert_eq!(bites, ["bite_left.stl", "bite_right.stl"]);
    assert_eq!(case.scan(ScanRole::Bite).unwrap().id.as_u128(), 100);
    assert_eq!(out.project.case_state.blobs.len(), 4);
}

#[test]
fn detected_unsure_survives_save_and_load() {
    let d = dir();
    let path = d.path().join("unsure.dcprj");
    let mut p = sample_project();
    for (scan, source) in p
        .case_state
        .case
        .scans
        .iter_mut()
        .zip([RoleSource::DetectedUnsure, RoleSource::Detected])
    {
        scan.role_source = source;
    }
    save(&path, &p).unwrap();
    let out = load(&path, &codecs()).unwrap();
    assert!(!out.read_only, "{:?}", out.issues);
    let case = &out.project.case_state.case;
    assert_eq!(*case, p.case_state.case);
    let sources: Vec<RoleSource> = case.scans.iter().map(|s| s.role_source).collect();
    assert_eq!(sources, [RoleSource::DetectedUnsure, RoleSource::Detected]);
    let mut zip = ZipArchive::new(std::fs::File::open(&path).unwrap()).unwrap();
    let mut text = String::new();
    zip.by_name("case.json")
        .unwrap()
        .read_to_string(&mut text)
        .unwrap();
    assert!(text.contains("\"detected_unsure\""), "{text}");
}

#[test]
fn round_trip_is_byte_identical() {
    let d = dir();
    let (a, b) = (d.path().join("a.dcprj"), d.path().join("b.dcprj"));
    let p = sample_project();
    save(&a, &p).unwrap();
    let out = load(&a, &codecs()).unwrap();
    assert!(
        out.issues.iter().all(|i| !i.blocks_writing()),
        "{:?}",
        out.issues
    );
    assert!(!out.read_only);
    let q = &out.project;
    assert_eq!(q.case_state.case, p.case_state.case);
    assert_eq!(q.history, p.history);
    assert_eq!(q.thumbnail, p.thumbnail);
    assert_eq!(q.manifest.project_id, p.manifest.project_id);
    assert_eq!(q.case_state.blobs.len(), 2);
    for s in &q.case_state.case.scans {
        assert_eq!(
            **q.case_state.blobs.get(&s.mesh).unwrap(),
            *p.case_state.blobs.get(&s.mesh).unwrap().as_ref()
        );
    }
    assert_eq!(
        q.case_state.module_data::<Demo>("demo"),
        p.case_state.module_data::<Demo>("demo")
    );
    save(&b, q).unwrap();
    assert_eq!(std::fs::read(&a).unwrap(), std::fs::read(&b).unwrap());
    // saving the same project twice is deterministic too
    save(&b, &p).unwrap();
    assert_eq!(std::fs::read(&a).unwrap(), std::fs::read(&b).unwrap());
}

#[test]
fn mimetype_is_first_and_stored() {
    let d = dir();
    let a = d.path().join("a.dcprj");
    save(&a, &sample_project()).unwrap();
    let mut ar = ZipArchive::new(std::fs::File::open(&a).unwrap()).unwrap();
    let mut first = ar.by_index(0).unwrap();
    assert_eq!(first.name().unwrap(), "mimetype");
    assert_eq!(first.compression(), CompressionMethod::Stored);
    let mut s = String::new();
    first.read_to_string(&mut s).unwrap();
    assert_eq!(s, MIME_TYPE);
    drop(first);
    assert_eq!(ar.by_index(1).unwrap().name().unwrap(), "manifest.json");
    // the raw bytes start with the local header followed by the plain name
    // and MIME type, so file(1)-style sniffing works
    let raw = std::fs::read(&a).unwrap();
    assert_eq!(&raw[30..38], b"mimetype");
    assert_eq!(&raw[38..38 + MIME_TYPE.len()], MIME_TYPE.as_bytes());
}

#[test]
fn manifest_lists_every_entry() {
    let d = dir();
    let a = d.path().join("a.dcprj");
    let p = sample_project();
    save(&a, &p).unwrap();
    let files = read_all(&a);
    let m: Manifest = serde_json::from_slice(&files[1].1).unwrap();
    assert_eq!(m.modules.len(), 2);
    assert_eq!(m.modules[1].schema_version, 9);
    for (n, b) in &files[2..] {
        let e = &m.entries[n];
        assert_eq!(e.bytes, b.len() as u64);
        assert_eq!(e.blake3, BlobRef::of_bytes(b));
    }
    assert_eq!(m.entries.len(), files.len() - 2);
}

#[test]
fn tampered_blob_is_detected() {
    let d = dir();
    let (a, b) = (d.path().join("a.dcprj"), d.path().join("b.dcprj"));
    let p = sample_project();
    save(&a, &p).unwrap();
    let victim = blob_entry_of(&p);
    let mut files = read_all(&a);
    for (n, bytes) in &mut files {
        if *n == victim {
            let i = bytes.len() / 2;
            bytes[i] ^= 0xff;
        }
    }
    write_zip(&b, &files);
    let out = load(&b, &codecs()).unwrap();
    assert!(out.read_only);
    assert!(out.issues.contains(&LoadIssue::HashMismatch(victim)));
    assert_eq!(out.project.case_state.blobs.len(), 1);
    assert!(
        out.issues
            .iter()
            .any(|i| matches!(i, LoadIssue::MissingBlob(_)))
    );
}

#[test]
fn flipped_byte_in_stored_archive_is_detected() {
    let d = dir();
    let a = d.path().join("a.dcprj");
    let p = sample_project();
    save(&a, &p).unwrap();
    let victim = blob_entry_of(&p);
    let start = {
        let mut ar = ZipArchive::new(std::fs::File::open(&a).unwrap()).unwrap();
        ar.by_name(&victim).unwrap().data_start().unwrap()
    };
    let mut raw = std::fs::read(&a).unwrap();
    raw[start as usize + 12] ^= 0x55;
    std::fs::write(&a, raw).unwrap();
    let out = load(&a, &codecs()).unwrap();
    assert!(out.read_only);
    assert!(!out.issues.is_empty());
}

#[test]
fn missing_blob_is_reported() {
    let d = dir();
    let (a, b) = (d.path().join("a.dcprj"), d.path().join("b.dcprj"));
    let p = sample_project();
    save(&a, &p).unwrap();
    let victim = blob_entry_of(&p);
    let files: Vec<_> = read_all(&a)
        .into_iter()
        .filter(|(n, _)| *n != victim)
        .collect();
    write_zip(&b, &files);
    let out = load(&b, &codecs()).unwrap();
    assert!(out.read_only);
    assert!(out.issues.contains(&LoadIssue::MissingEntry(victim)));
    assert_eq!(out.project.case_state.blobs.len(), 1);
}

#[test]
fn unlisted_entry_makes_read_only() {
    let d = dir();
    let (a, b) = (d.path().join("a.dcprj"), d.path().join("b.dcprj"));
    save(&a, &sample_project()).unwrap();
    let mut files = read_all(&a);
    files.push(("extra.bin".into(), vec![1, 2, 3]));
    write_zip(&b, &files);
    let out = load(&b, &codecs()).unwrap();
    assert!(out.read_only);
    assert!(
        out.issues
            .contains(&LoadIssue::UnlistedEntry("extra.bin".into()))
    );
}

#[test]
fn truncated_and_garbage_files_error_without_panic() {
    let d = dir();
    let a = d.path().join("a.dcprj");
    save(&a, &sample_project()).unwrap();
    let raw = std::fs::read(&a).unwrap();
    let t = d.path().join("t.dcprj");
    for cut in [
        0,
        1,
        10,
        30,
        raw.len() / 3,
        raw.len() / 2,
        raw.len() - 30,
        raw.len() - 1,
    ] {
        std::fs::write(&t, &raw[..cut]).unwrap();
        assert!(load(&t, &codecs()).is_err(), "cut at {cut}");
    }
    std::fs::write(&t, b"this is not a zip file at all").unwrap();
    assert!(load(&t, &codecs()).is_err());
    assert!(matches!(
        load(&d.path().join("missing.dcprj"), &codecs()),
        Err(DcprjError::Io(_))
    ));
}

#[test]
fn foreign_zip_is_not_a_project() {
    let d = dir();
    let t = d.path().join("t.zip");
    let mut zw = ZipWriter::new(std::fs::File::create(&t).unwrap());
    zw.start_file("hello.txt", SimpleFileOptions::default())
        .unwrap();
    zw.write_all(b"hi").unwrap();
    zw.finish().unwrap();
    assert!(matches!(load(&t, &codecs()), Err(DcprjError::NotAProject)));
}

#[test]
fn atomic_save_keeps_backup() {
    let d = dir();
    let a = d.path().join("case.dcprj");
    let mut p = sample_project();
    save(&a, &p).unwrap();
    let first = std::fs::read(&a).unwrap();
    assert!(!d.path().join("case.dcprj.bak").exists());
    p.history = json!([{"step": 1}, {"step": 2}]);
    save(&a, &p).unwrap();
    let second = std::fs::read(&a).unwrap();
    assert_ne!(first, second);
    assert_eq!(
        std::fs::read(d.path().join("case.dcprj.bak")).unwrap(),
        first
    );
    assert!(!d.path().join("case.dcprj.tmp").exists());
    // a third save replaces the backup with the previous version
    p.history = json!([]);
    save(&a, &p).unwrap();
    assert_eq!(
        std::fs::read(d.path().join("case.dcprj.bak")).unwrap(),
        second
    );
    let names: Vec<_> = std::fs::read_dir(d.path())
        .unwrap()
        .map(|e| e.unwrap().file_name().to_string_lossy().into_owned())
        .collect();
    assert_eq!(names.len(), 2, "{names:?}");
}

#[test]
fn failed_save_leaves_existing_file_and_no_tmp() {
    let d = dir();
    let a = d.path().join("case.dcprj");
    let mut p = sample_project();
    save(&a, &p).unwrap();
    let before = std::fs::read(&a).unwrap();
    p.case_state.set_module(
        "bad/id",
        ModuleState::new(OpaqueModuleData {
            schema_version: 1,
            value: json!(1),
        }),
    );
    assert!(matches!(save(&a, &p), Err(DcprjError::InvalidModuleId(_))));
    assert_eq!(std::fs::read(&a).unwrap(), before);
    assert!(!d.path().join("case.dcprj.tmp").exists());
    assert!(!d.path().join("case.dcprj.bak").exists());
}

#[test]
fn migrates_v0_container_to_v1() {
    let d = dir();
    let a = d.path().join("old.dcprj");
    let mut manifest = base_manifest(0);
    manifest.as_object_mut().unwrap().remove("units");
    let case = json!({
        "id": uuid::Uuid::from_u128(5),
        "patient": "P-OLD",
        "scans": [],
        "world_frame": {"from_case": {"t": [0.0, 0.0, 0.0], "q": [0.0, 0.0, 0.0, 1.0]}},
    });
    build_zip(
        &a,
        manifest,
        &[(CASE_PATH, serde_json::to_vec(&case).unwrap())],
    );
    let out = load(&a, &codecs()).unwrap();
    assert!(out.issues.is_empty(), "{:?}", out.issues);
    assert!(!out.read_only);
    assert_eq!(out.project.manifest.format_version, FORMAT_VERSION);
    assert_eq!(out.project.manifest.units, "mm");
    assert_eq!(out.project.case_state.case.patient_code, "P-OLD");
    // saving upgrades the file
    let b = d.path().join("new.dcprj");
    save(&b, &out.project).unwrap();
    let again = load(&b, &codecs()).unwrap();
    assert_eq!(again.project.manifest.format_version, 1);
    assert!(again.issues.is_empty());
}

#[test]
fn migrates_module_state_by_schema_version() {
    let d = dir();
    let a = d.path().join("old.dcprj");
    let mut manifest = base_manifest(1);
    manifest["modules"] = json!([{"id": "demo", "version": "0.0.1", "schema_version": 0}]);
    let state0 = json!({"count": 3, "name": "legacy"});
    build_zip(
        &a,
        manifest,
        &[
            (CASE_PATH, case_json()),
            (
                &module_state_path("demo"),
                serde_json::to_vec(&state0).unwrap(),
            ),
        ],
    );
    let mut c = ModuleCodecs::new();
    c.register(
        "demo",
        ModuleCodec::typed::<Demo>(1).with_migration(0, |v| {
            let o = v.as_object_mut().ok_or("not an object")?;
            let n = o.remove("count").ok_or("no count")?;
            o.insert("stages".into(), n);
            Ok(())
        }),
    );
    let out = load(&a, &c).unwrap();
    assert!(out.issues.is_empty(), "{:?}", out.issues);
    assert_eq!(
        out.project.case_state.module_data::<Demo>("demo"),
        Some(&Demo {
            stages: 3,
            name: "legacy".into()
        })
    );
    // without the migration the state is kept as is and the file is protected
    let out = load(&a, &codecs()).unwrap();
    assert!(out.read_only);
    assert!(out.project.case_state.module("demo").unwrap().is_opaque());
}

#[test]
fn migrates_module_state_across_several_versions() {
    let d = dir();
    let a = d.path().join("old.dcprj");
    let mut manifest = base_manifest(1);
    manifest["modules"] = json!([{"id": "demo", "version": "0.0.1", "schema_version": 0}]);
    let state0 = json!({"count": 3, "label": "legacy"});
    build_zip(
        &a,
        manifest,
        &[
            (CASE_PATH, case_json()),
            (
                &module_state_path("demo"),
                serde_json::to_vec(&state0).unwrap(),
            ),
        ],
    );
    let codec = ModuleCodec::typed::<Demo>(2)
        .with_migration(0, |v| {
            let o = v.as_object_mut().ok_or("not an object")?;
            let n = o.remove("count").ok_or("no count")?;
            o.insert("stages".into(), n);
            Ok(())
        })
        .with_migration(1, |v| {
            let o = v.as_object_mut().ok_or("not an object")?;
            let n = o.remove("label").ok_or("no label")?;
            o.insert("name".into(), n);
            Ok(())
        });
    let mut c = ModuleCodecs::new();
    c.register("demo", codec.clone());
    let out = load(&a, &c).unwrap();
    assert!(out.issues.is_empty(), "{:?}", out.issues);
    assert_eq!(
        out.project.case_state.module_data::<Demo>("demo"),
        Some(&Demo {
            stages: 3,
            name: "legacy".into()
        })
    );
    // starting from the middle version only runs the later steps
    let mid = codec.load(json!({"stages": 5, "label": "mid"}), 1).unwrap();
    assert_eq!(
        mid.as_any().downcast_ref::<Demo>(),
        Some(&Demo {
            stages: 5,
            name: "mid".into()
        })
    );
    // newer than the codec is refused
    assert!(codec.load(json!({}), 3).is_err());
}

#[test]
fn old_case_json_without_new_fields_loads() {
    let d = dir();
    let a = d.path().join("old.dcprj");
    // case.json as written before `case_id`, `notes` and `jaws` existed
    let case = json!({
        "id": uuid::Uuid::from_u128(5),
        "patient_code": "P-OLD",
        "scans": [],
        "world_frame": {"from_case": {"t": [0.0, 0.0, 0.0], "q": [0.0, 0.0, 0.0, 1.0]}},
    });
    build_zip(
        &a,
        base_manifest(1),
        &[(CASE_PATH, serde_json::to_vec(&case).unwrap())],
    );
    let out = load(&a, &codecs()).unwrap();
    assert!(out.issues.is_empty(), "{:?}", out.issues);
    let c = &out.project.case_state.case;
    assert_eq!(c.patient_code, "P-OLD");
    assert_eq!(c.case_id, "");
    assert_eq!(c.notes, "");
    assert_eq!(c.jaws, dc_case::Jaws::Both);
}

#[test]
fn save_prunes_unreferenced_blobs() {
    let d = dir();
    let a = d.path().join("gc.dcprj");
    let mut p = sample_project();
    let garbage = p.case_state.blobs.insert(mesh(9.5));
    // referenced only from an opaque module's JSON
    let kept = p.case_state.blobs.insert(mesh(7.5));
    p.case_state.set_module(
        "future",
        ModuleState::new(OpaqueModuleData {
            schema_version: 9,
            value: json!({"mesh": kept.to_hex()}),
        }),
    );
    assert_eq!(p.case_state.blobs.len(), 4);
    save(&a, &p).unwrap();
    let out = load(&a, &codecs()).unwrap();
    let blobs = &out.project.case_state.blobs;
    assert_eq!(blobs.len(), 3);
    assert!(!blobs.contains(&garbage));
    assert!(blobs.contains(&kept));
    assert!(
        out.issues.iter().all(|i| !i.blocks_writing()),
        "{:?}",
        out.issues
    );
}

#[test]
fn newer_format_version_is_read_only() {
    let d = dir();
    let a = d.path().join("new.dcprj");
    let mut manifest = base_manifest(2);
    manifest["shiny_new_field"] = json!({"x": 1});
    build_zip(&a, manifest, &[(CASE_PATH, case_json())]);
    let out = load(&a, &codecs()).unwrap();
    assert!(out.read_only);
    assert!(out.issues.contains(&LoadIssue::NewerFormat {
        found: 2,
        supported: 1
    }));
    assert_eq!(out.project.case_state.case.patient_code, "P-9");

    // an unreadable newer layout is an UnsupportedVersion error
    let b = d.path().join("new2.dcprj");
    build_zip(
        &b,
        base_manifest(2),
        &[(CASE_PATH, b"{\"layout\": 2}".to_vec())],
    );
    assert!(matches!(
        load(&b, &codecs()),
        Err(DcprjError::UnsupportedVersion {
            found: 2,
            supported: 1
        })
    ));
}

#[test]
fn opaque_module_round_trips_unchanged() {
    let d = dir();
    let (a, b) = (d.path().join("a.dcprj"), d.path().join("b.dcprj"));
    save(&a, &sample_project()).unwrap();
    // no codecs at all: both modules stay opaque
    let out = load(&a, &ModuleCodecs::new()).unwrap();
    assert!(!out.read_only);
    assert!(out.project.case_state.module("future").unwrap().is_opaque());
    assert!(out.project.case_state.module("demo").unwrap().is_opaque());
    assert!(out.issues.contains(&LoadIssue::ModuleKeptOpaque {
        id: "future".into()
    }));
    save(&b, &out.project).unwrap();
    assert_eq!(std::fs::read(&a).unwrap(), std::fs::read(&b).unwrap());

    // codec older than the file: kept opaque, noted, still writable
    let mut c = ModuleCodecs::new();
    c.register("future", ModuleCodec::typed::<Demo>(2));
    let out = load(&a, &c).unwrap();
    assert!(!out.read_only);
    assert!(out.issues.contains(&LoadIssue::ModuleTooNew {
        id: "future".into(),
        found: 9,
        supported: 2
    }));
    let opaque = out
        .project
        .case_state
        .module_data::<OpaqueModuleData>("future")
        .unwrap();
    assert_eq!(opaque.schema_version, 9);
    assert_eq!(opaque.value["z"], 1.5);
}

#[test]
fn module_version_string_survives() {
    let d = dir();
    let (a, b) = (d.path().join("a.dcprj"), d.path().join("b.dcprj"));
    let mut p = sample_project();
    p.manifest.modules.push(ModuleEntry {
        id: "demo".into(),
        version: "2.3.4".into(),
        schema_version: 1,
    });
    save(&a, &p).unwrap();
    let out = load(&a, &ModuleCodecs::new()).unwrap();
    let demo = out
        .project
        .manifest
        .modules
        .iter()
        .find(|m| m.id == "demo")
        .unwrap();
    assert_eq!(demo.version, "2.3.4");
    save(&b, &out.project).unwrap();
    assert_eq!(std::fs::read(&a).unwrap(), std::fs::read(&b).unwrap());
}

fn arb_mesh() -> impl Strategy<Value = TriMesh> {
    (1usize..12).prop_flat_map(|nv| {
        let coord = -1000.0f32..1000.0;
        (
            prop::collection::vec((coord.clone(), coord.clone(), coord), nv),
            prop::collection::vec((0..nv as u32, 0..nv as u32, 0..nv as u32), 0..10),
            prop::option::of(prop::collection::vec(any::<u16>(), nv)),
            any::<bool>(),
        )
            .prop_map(|(pos, tris, labels, face_labels)| {
                let mut m = TriMesh::new(
                    pos.into_iter()
                        .map(|(x, y, z)| Vec3::new(x, y, z))
                        .collect(),
                    tris.into_iter().map(|(a, b, c)| [a, b, c]).collect(),
                );
                m.vertex_labels = labels;
                if face_labels {
                    m.face_labels = Some((0..m.tris.len() as u16).collect());
                }
                m
            })
    })
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(24))]

    #[test]
    fn random_meshes_round_trip(meshes in prop::collection::vec(arb_mesh(), 1..4)) {
        let d = dir();
        let (a, b) = (d.path().join("a.dcprj"), d.path().join("b.dcprj"));
        let mut state = CaseState::new(Case::new("P-prop"));
        let refs: Vec<_> = meshes.iter().cloned().map(|m| state.blobs.insert(m)).collect();
        // blobs must be referenced to survive a save
        for (i, r) in refs.iter().enumerate() {
            state.case.scans.push(Scan {
                id: uuid::Uuid::from_u128(i as u128 + 1),
                role: ScanRole::Upper,
                role_source: RoleSource::Explicit,
                source_name: format!("s{i}.stl"),
                source_units: Units::Millimeters,
                import_xform: Default::default(),
                mesh: *r,
            });
        }
        let mut p = Project::new(app(), "2026-10-31T10:00:00Z", state);
        p.manifest.project_id = uuid::Uuid::from_u128(7);
        save(&a, &p).unwrap();
        let out = load(&a, &codecs()).unwrap();
        prop_assert!(out.issues.is_empty());
        prop_assert!(!out.read_only);
        for (r, m) in refs.iter().zip(&meshes) {
            prop_assert_eq!(&**out.project.case_state.blobs.get(r).unwrap(), m);
        }
        save(&b, &out.project).unwrap();
        prop_assert_eq!(std::fs::read(&a).unwrap(), std::fs::read(&b).unwrap());
    }
}

/// File names in `dir`, sorted.
fn listing(dir: &Path) -> Vec<String> {
    let mut names: Vec<_> = std::fs::read_dir(dir)
        .unwrap()
        .map(|e| e.unwrap().file_name().to_string_lossy().into_owned())
        .collect();
    names.sort();
    names
}

#[test]
fn save_refuses_non_finite_case_and_leaves_files_untouched() {
    let d = dir();
    let a = d.path().join("case.dcprj");
    let mut p = sample_project();
    save(&a, &p).unwrap();
    p.history = json!([]);
    save(&a, &p).unwrap(); // now there is a backup too
    let before = std::fs::read(&a).unwrap();
    let bak = std::fs::read(d.path().join("case.dcprj.bak")).unwrap();
    let names = listing(d.path());
    // what `Pose::new` used to produce from a zero quaternion
    p.case_state.case.world_frame.from_case.q = glam::DQuat::from_xyzw(f64::NAN, 0.0, 0.0, 1.0);
    match save(&a, &p) {
        Err(DcprjError::NonFinite { entry, path }) => {
            assert_eq!(entry, CASE_PATH);
            assert_eq!(path, "world_frame.from_case.q[0]");
        }
        other => panic!("expected NonFinite, got {other:?}"),
    }
    assert_eq!(std::fs::read(&a).unwrap(), before);
    assert_eq!(std::fs::read(d.path().join("case.dcprj.bak")).unwrap(), bak);
    assert_eq!(listing(d.path()), names, "no temporary file is left");
}

#[derive(Debug, Clone, Serialize)]
struct Measure {
    gap: Option<f64>,
}
dc_case::impl_module_data!(Measure, 1);

#[test]
fn save_refuses_non_finite_option_instead_of_writing_none() {
    let d = dir();
    let a = d.path().join("case.dcprj");
    let mut p = sample_project();
    p.case_state.set_module(
        "measure",
        ModuleState::new(Measure {
            gap: Some(f64::INFINITY),
        }),
    );
    match save(&a, &p) {
        Err(DcprjError::NonFinite { entry, path }) => {
            assert_eq!(entry, module_state_path("measure"));
            assert_eq!(path, "gap");
        }
        other => panic!("expected NonFinite, got {other:?}"),
    }
    assert!(listing(d.path()).is_empty());
    p.case_state
        .set_module("measure", ModuleState::new(Measure { gap: Some(0.5) }));
    assert!(save(&a, &p).unwrap().is_empty());
}

#[test]
fn temporary_file_name_is_unique() {
    let d = dir();
    let a = d.path().join("case.dcprj");
    // another writer's (or a stale) fixed-name temporary file is not touched
    std::fs::create_dir(d.path().join("case.dcprj.tmp")).unwrap();
    std::fs::write(d.path().join("case.dcprj.tmp/keep"), b"x").unwrap();
    save(&a, &sample_project()).unwrap();
    save(&a, &sample_project()).unwrap();
    assert_eq!(
        std::fs::read(d.path().join("case.dcprj.tmp/keep")).unwrap(),
        b"x"
    );
    assert_eq!(
        listing(d.path()),
        ["case.dcprj", "case.dcprj.bak", "case.dcprj.tmp"]
    );
    assert!(!load(&a, &codecs()).unwrap().read_only);
}

#[test]
fn backup_failure_is_a_warning_not_a_failed_save() {
    let d = dir();
    let a = d.path().join("case.dcprj");
    let mut p = sample_project();
    save(&a, &p).unwrap();
    // a `.bak` that cannot be removed (a non-empty directory, on every OS)
    let bak = d.path().join("case.dcprj.bak");
    std::fs::create_dir(&bak).unwrap();
    std::fs::write(bak.join("x"), b"x").unwrap();
    p.history = json!([{"step": 9}]);
    let warnings = save(&a, &p).unwrap();
    assert!(
        matches!(&warnings[..], [SaveWarning::Backup { path, .. }] if *path == bak),
        "{warnings:?}"
    );
    let out = load(&a, &codecs()).unwrap();
    assert!(!out.read_only);
    assert_eq!(out.project.history, json!([{"step": 9}]));
    assert_eq!(
        listing(d.path()),
        ["case.dcprj", "case.dcprj.bak"],
        "no temporary file is left"
    );
}

/// Archive with `case.json` and `history/steps.json`, whose manifest
/// declares `declared` bytes for the history.
fn lying_history(path: &Path, declared: u64) {
    let case = case_json();
    let history = b"[]".to_vec();
    let mut manifest = base_manifest(1);
    manifest["entries"] = json!({
        CASE_PATH: {"blake3": BlobRef::of_bytes(&case).to_hex(), "bytes": case.len()},
        HISTORY_PATH: {"blake3": BlobRef::of_bytes(&history).to_hex(), "bytes": declared},
    });
    write_zip(
        path,
        &[
            (
                MANIFEST_PATH.to_owned(),
                serde_json::to_vec(&manifest).unwrap(),
            ),
            (CASE_PATH.to_owned(), case),
            (HISTORY_PATH.to_owned(), history),
        ],
    );
}

#[test]
fn hostile_declared_sizes_are_load_errors_not_aborts() {
    let d = dir();
    let a = d.path().join("bomb.dcprj");
    // 1 TiB for one entry: above the per-entry limit
    lying_history(&a, 1 << 40);
    assert!(matches!(load(&a, &codecs()), Err(DcprjError::Corrupt(_))));
    // under the per-entry limit, but far more than the file can expand to
    lying_history(&a, MAX_ENTRY_BYTES);
    match load(&a, &codecs()) {
        Err(DcprjError::Corrupt(m)) => assert!(m.contains("in total"), "{m}"),
        other => panic!("expected Corrupt, got {other:?}"),
    }
    // a size that differs from the ZIP entry by one byte
    lying_history(&a, 3);
    let out = load(&a, &codecs()).unwrap();
    assert!(out.read_only);
    assert!(
        out.issues.iter().any(|i| matches!(
            i,
            LoadIssue::UnreadableEntry { path, reason }
                if path == HISTORY_PATH && reason.contains("differs from the manifest")
        )),
        "{:?}",
        out.issues
    );
}

#[test]
fn duplicate_entry_names_are_refused() {
    let d = dir();
    let a = d.path().join("dup.dcprj");
    // the zip writer refuses duplicates: forge the second name in the bytes
    build_zip(
        &a,
        base_manifest(1),
        &[(CASE_PATH, case_json()), ("case.jsoX", case_json())],
    );
    let mut bytes = std::fs::read(&a).unwrap();
    let mut forged = 0;
    for i in 0..bytes.len() - 9 {
        if &bytes[i..i + 9] == b"case.jsoX" {
            bytes[i + 8] = b'n';
            forged += 1;
        }
    }
    assert!(forged >= 2, "local and central headers");
    std::fs::write(&a, bytes).unwrap();
    match load(&a, &codecs()) {
        Err(DcprjError::Corrupt(m)) => assert!(m.contains("duplicate"), "{m}"),
        other => panic!("expected Corrupt, got {other:?}"),
    }
}

#[test]
fn blob_referenced_only_by_a_module_must_be_present() {
    let d = dir();
    let a = d.path().join("case.dcprj");
    let mut p = sample_project();
    let missing = BlobRef::of_bytes(b"a mesh that is not in the store");
    p.case_state.set_module(
        "extra",
        ModuleState::new(OpaqueModuleData {
            schema_version: 1,
            value: json!({"mesh": missing.to_hex()}),
        }),
    );
    save(&a, &p).unwrap();
    let out = load(&a, &codecs()).unwrap();
    assert!(out.read_only);
    assert!(
        out.issues.contains(&LoadIssue::MissingBlob(missing)),
        "{:?}",
        out.issues
    );
}
