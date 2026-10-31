//! JSON summary of a session for [`crate::Session::inspect`].

use std::collections::HashSet;

use dc_mesh::TriMesh;
use serde_json::{Map, Value, json};

use crate::builtins::role_str;
use crate::{Result, Session, SessionError};

/// Arrays longer than this are abbreviated in the top-level summary.
const MAX_ARRAY: usize = 32;
/// Elements kept at the head of an abbreviated array.
const HEAD: usize = 8;

fn mesh_json(mesh: &TriMesh) -> Value {
    let aabb = mesh.aabb().map(|b| {
        json!({
            "min": [b.min.x, b.min.y, b.min.z],
            "max": [b.max.x, b.max.y, b.max.z],
        })
    });
    json!({
        "vertices": mesh.vertex_count(),
        "faces": mesh.face_count(),
        "aabb": aabb,
    })
}

fn merge(mut a: Value, b: Value) -> Value {
    if let (Value::Object(a), Value::Object(b)) = (&mut a, b) {
        a.extend(b);
    }
    a
}

/// The full summary tree (module states unabridged).
fn full(s: &Session) -> Value {
    let state = s.state();
    let case = &state.case;
    let referenced: HashSet<_> = state.referenced_blobs();
    let scans: Vec<Value> = case
        .scans
        .iter()
        .map(|scan| {
            let base = json!({
                "id": scan.id,
                "role": role_str(scan.role),
                "source_name": scan.source_name,
                "source_units": scan.source_units.symbol(),
                "mesh": scan.mesh.to_hex(),
            });
            match state.mesh(&scan.mesh) {
                Some(m) => merge(base, mesh_json(m)),
                None => merge(base, json!({ "missing": true })),
            }
        })
        .collect();
    let mut meshes: Vec<(String, Value)> = state
        .blobs
        .iter()
        .map(|(r, m)| {
            (
                r.to_hex(),
                merge(
                    json!({ "blob": r.to_hex(), "referenced": referenced.contains(r) }),
                    mesh_json(m),
                ),
            )
        })
        .collect();
    meshes.sort_by(|a, b| a.0.cmp(&b.0));
    let modules: Map<String, Value> = state
        .modules
        .iter()
        .map(|(id, m)| {
            let v = m
                .to_json()
                .unwrap_or_else(|e| json!({ "error": e.to_string() }));
            (id.clone(), v)
        })
        .collect();
    let registered: Vec<Value> = s
        .modules()
        .list()
        .map(|m| {
            json!({
                "id": m.id(),
                "version": m.version(),
                "name": dc_i18n::tr(m.name_key()),
                "enabled": s.modules().is_enabled(m.id()),
            })
        })
        .collect();
    let manifest = s.manifest();
    let undo = s.undo_stack();
    json!({
        "project": {
            "path": s.path().map(|p| p.display().to_string()),
            "read_only": s.is_read_only(),
            "dirty": s.is_dirty(),
            "project_id": manifest.map(|m| m.project_id),
            "created": manifest.map(|m| m.created.clone()),
            "modified": manifest.map(|m| m.modified.clone()),
            "load_issues": s.load_issues(),
        },
        "case": {
            "id": case.id,
            "patient_code": case.patient_code,
            "case_id": case.case_id,
            "notes": case.notes,
            "jaws": case.jaws,
            "world_frame": case.world_frame,
        },
        "scans": scans,
        "meshes": meshes.into_iter().map(|(_, v)| v).collect::<Vec<_>>(),
        "modules": modules,
        "registered_modules": registered,
        "steps": s.steps_json(),
        "undo": {
            "undo_depth": undo.undo_depth(),
            "redo_depth": undo.redo_depth(),
            "undo_label": undo.undo_label(),
            "redo_label": undo.redo_label(),
            "memory_bytes": undo.memory_bytes(),
        },
        "view": s.view(),
    })
}

/// Replaces arrays longer than [`MAX_ARRAY`] with `{"$len": n, "head": [...]}`.
fn abbreviate(v: Value) -> Value {
    match v {
        Value::Array(a) if a.len() > MAX_ARRAY => {
            let len = a.len();
            let head: Vec<Value> = a.into_iter().take(HEAD).map(abbreviate).collect();
            json!({ "$len": len, "head": head })
        }
        Value::Array(a) => Value::Array(a.into_iter().map(abbreviate).collect()),
        Value::Object(m) => Value::Object(m.into_iter().map(|(k, v)| (k, abbreviate(v))).collect()),
        other => other,
    }
}

/// JSON pointer for `a.b.0` (or the path itself when it starts with `/`).
fn pointer(path: &str) -> String {
    if path.starts_with('/') {
        return path.to_owned();
    }
    path.split('.')
        .filter(|p| !p.is_empty())
        .map(|p| format!("/{}", p.replace('~', "~0").replace('/', "~1")))
        .collect()
}

pub(crate) fn inspect(s: &Session, path: Option<&str>) -> Result<Value> {
    let tree = full(s);
    match path.map(str::trim).filter(|p| !p.is_empty() && *p != "/") {
        None => {
            let mut tree = tree;
            if let Some(m) = tree.get_mut("modules") {
                *m = abbreviate(m.take());
            }
            Ok(tree)
        }
        Some(p) => tree
            .pointer(&pointer(p))
            .cloned()
            .ok_or_else(|| SessionError::PathNotFound(p.to_owned())),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pointers() {
        assert_eq!(pointer("scans.0.role"), "/scans/0/role");
        assert_eq!(pointer("/scans/0"), "/scans/0");
        assert_eq!(pointer("a/b"), "/a~1b");
    }

    #[test]
    fn long_arrays_are_abbreviated() {
        let v = abbreviate(json!({ "labels": (0..100).collect::<Vec<_>>(), "short": [1, 2] }));
        assert_eq!(v["labels"]["$len"], 100);
        assert_eq!(v["labels"]["head"].as_array().unwrap().len(), HEAD);
        assert_eq!(v["short"], json!([1, 2]));
    }

    #[test]
    fn summary_of_an_empty_session() {
        let s = Session::new();
        let v = s.inspect(None).unwrap();
        assert_eq!(v["scans"], json!([]));
        assert_eq!(v["undo"]["undo_depth"], 0);
        assert_eq!(v["project"]["dirty"], false);
        assert_eq!(s.inspect(Some("undo.redo_depth")).unwrap(), 0);
        assert!(matches!(
            s.inspect(Some("nope")),
            Err(SessionError::PathNotFound(_))
        ));
    }
}
