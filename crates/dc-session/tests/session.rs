//! Session tests: built-ins, a module command, undo/redo and a `.dcprj`
//! round trip through `project.save` / `project.open`.

use std::path::Path;

use dc_case::{CaseState, ModuleState};
use dc_command::{CommandError, JobCtx, StepDef, StepGraph};
use dc_mesh::TriMesh;
use dc_module_api::{
    CommandDesc, CommandOutput, CommandSpec, MenuTree, Module, ModuleCodec, ModuleCtx,
};
use dc_scene::SceneItem;
use dc_session::{Session, SessionError};
use glam::Vec3;
use serde_json::{Value, json};

#[derive(Debug, Clone, Default, PartialEq, serde::Serialize, serde::Deserialize)]
struct Counter {
    count: u64,
}
dc_case::impl_module_data!(Counter, 1);

fn increment(
    state: &CaseState,
    params: &Value,
    ctx: &JobCtx,
) -> Result<CommandOutput, CommandError> {
    ctx.check_cancelled()?;
    let by = match params.get("by") {
        None => 1,
        Some(v) => v
            .as_u64()
            .ok_or_else(|| CommandError::Invalid("by must be a non-negative integer".into()))?,
    };
    let count = state.module_data::<Counter>("dummy").map_or(0, |c| c.count) + by;
    let mut next = state.clone();
    next.set_module("dummy", ModuleState::new(Counter { count }));
    Ok((next, json!({ "count": count })))
}

struct Dummy;

impl Module for Dummy {
    fn id(&self) -> &'static str {
        "dummy"
    }
    fn name_key(&self) -> &'static str {
        "dummy-name"
    }
    fn version(&self) -> &'static str {
        "0.1.0"
    }
    fn menu(&self) -> MenuTree {
        MenuTree::default()
    }
    fn commands(&self) -> Vec<CommandDesc> {
        Vec::new()
    }
    fn command_specs(&self) -> Vec<CommandSpec> {
        vec![CommandSpec {
            id: "dummy.increment",
            label_key: "dummy-increment",
            doc_key: "dummy-increment-doc",
            params_schema: || json!({"type": "object", "properties": {"by": {"type": "integer", "minimum": 0}}}),
            run: increment,
        }]
    }
    fn inspector(&mut self, _ui: &mut egui::Ui, _ctx: &mut ModuleCtx<'_>) {}
    fn scene(&self, _state: &CaseState) -> Vec<SceneItem> {
        Vec::new()
    }
    fn data_codec(&self) -> ModuleCodec {
        ModuleCodec::typed::<Counter>(1)
    }
}

fn session() -> Session {
    let mut s = Session::new();
    s.register_module(Box::new(Dummy)).unwrap();
    s
}

/// A closed tetrahedron, 10 mm across, written as binary STL.
fn write_tetra(path: &Path) {
    let mesh = TriMesh::new(
        vec![
            Vec3::ZERO,
            Vec3::new(10.0, 0.0, 0.0),
            Vec3::new(0.0, 10.0, 0.0),
            Vec3::new(0.0, 0.0, 10.0),
        ],
        vec![[0, 2, 1], [0, 1, 3], [1, 2, 3], [0, 3, 2]],
    );
    dc_mesh::io::write(path, &mesh, dc_mesh::MeshFormat::Stl).unwrap();
}

fn count(s: &Session) -> Value {
    s.inspect(Some("modules.dummy.count")).unwrap()
}

#[test]
fn lists_and_describes_builtins_and_module_commands() {
    let s = session();
    let list = s.list_commands();
    let ids: Vec<&str> = list.iter().map(|c| c.id.as_str()).collect();
    for id in [
        "project.new",
        "project.open",
        "project.save",
        "case.import",
        "edit.undo",
        "edit.redo",
        "steps.list",
        "steps.rewind",
        "view.set",
        "dummy.increment",
    ] {
        assert!(ids.contains(&id), "{id} missing");
    }
    assert_eq!(list.last().unwrap().source, "dummy");
    let d = s.describe("case.import").unwrap();
    assert_eq!(d.label, "Import scan");
    assert_eq!(d.params_schema["required"], json!(["path"]));
    assert!(d.params_schema["properties"]["role"].is_object());
    let d = s.describe("dummy.increment").unwrap();
    assert_eq!(d.source, "dummy");
    // no bundle for the dummy keys: the key itself
    assert_eq!(d.label, "dummy-increment");
    assert!(matches!(
        s.describe("nope"),
        Err(SessionError::UnknownCommand(_))
    ));
}

#[test]
fn executes_builtins_and_module_commands_with_undo_redo() {
    let dir = tempfile::tempdir().unwrap();
    let stl = dir.path().join("lower_scan.stl");
    write_tetra(&stl);

    let mut s = session();
    assert_eq!(
        s.execute("dummy.increment", json!({"by": 2})).unwrap(),
        json!({"count": 2})
    );
    assert_eq!(
        s.execute("dummy.increment", json!({"by": 3})).unwrap(),
        json!({"count": 5})
    );
    assert!(s.is_dirty());

    // undo / redo through the API and through the built-in commands
    assert!(s.undo());
    assert_eq!(count(&s), 2);
    assert!(s.redo());
    assert_eq!(count(&s), 5);
    let out = s.execute("edit.undo", Value::Null).unwrap();
    assert_eq!(out, json!({"done": true, "undo_depth": 1, "redo_depth": 1}));
    assert_eq!(count(&s), 2);
    s.execute("edit.redo", json!({})).unwrap();
    assert_eq!(count(&s), 5);
    assert_eq!(s.execute("edit.redo", json!({})).unwrap()["done"], false);

    // import a scan: role from the file name, counts in the summary
    let out = s
        .execute("case.import", json!({"path": stl.to_str().unwrap()}))
        .unwrap();
    assert_eq!(out["role"], "lower");
    assert_eq!(
        (out["vertices"].clone(), out["faces"].clone()),
        (json!(4), json!(4))
    );
    let summary = s.inspect(None).unwrap();
    assert_eq!(summary["scans"][0]["role"], "lower");
    assert_eq!(summary["scans"][0]["faces"], 4);
    assert_eq!(summary["meshes"].as_array().unwrap().len(), 1);
    assert_eq!(summary["undo"]["undo_depth"], 3);
    assert_eq!(summary["modules"]["dummy"]["count"], 5);
    // the import is undoable
    assert!(s.undo());
    assert_eq!(s.inspect(Some("scans")).unwrap(), json!([]));
    assert!(s.redo());
    // explicit role
    s.execute(
        "case.import",
        json!({"path": stl.to_str().unwrap(), "role": "bite"}),
    )
    .unwrap();
    assert_eq!(s.state().case.scans.len(), 2);

    // errors leave the state untouched
    let depth = s.undo_stack().undo_depth();
    assert!(matches!(
        s.execute("nope", json!({})),
        Err(SessionError::UnknownCommand(_))
    ));
    assert!(matches!(
        s.execute("dummy.increment", json!({"by": "x"})),
        Err(SessionError::Command { .. })
    ));
    assert!(
        s.execute(
            "case.import",
            json!({"path": dir.path().join("missing.stl")})
        )
        .is_err()
    );
    assert!(s.execute("edit.undo", json!({"extra": 1})).is_err());
    assert_eq!(s.undo_stack().undo_depth(), depth);

    // view parameters
    let v = s
        .execute("view.set", json!({"preset": "front", "width": 640}))
        .unwrap();
    assert_eq!(v, json!({"preset": "front", "width": 640}));
    let v = s.execute("view.set", json!({"width": null})).unwrap();
    assert_eq!(v, json!({"preset": "front"}));
    assert_eq!(s.inspect(Some("/view/preset")).unwrap(), "front");
    assert!(s.execute("view.set", json!([1])).is_err());

    // batch: stops at the first failure, earlier commands stay applied
    let err = s
        .batch(vec![
            ("dummy.increment".into(), json!({})),
            ("nope".into(), json!({})),
            ("dummy.increment".into(), json!({})),
        ])
        .unwrap_err();
    assert!(matches!(err, SessionError::Batch { index: 1, .. }));
    assert_eq!(count(&s), 6);
    let outs = s
        .batch(vec![
            ("dummy.increment".into(), json!({"by": 4})),
            ("edit.undo".into(), Value::Null),
        ])
        .unwrap();
    assert_eq!(outs[0], json!({"count": 10}));
    assert_eq!(count(&s), 6);

    // project.new drops everything
    let out = s
        .execute("project.new", json!({"patient_code": "PZ-9"}))
        .unwrap();
    assert_eq!(out["discarded_unsaved_changes"], true);
    assert_eq!(s.state().case.patient_code, "PZ-9");
    assert_eq!(s.undo_stack().undo_depth(), 0);
    assert!(!s.is_dirty());
}

#[test]
fn steps_list_and_rewind() {
    let mut s = session();
    let out = s.execute("steps.list", Value::Null).unwrap();
    assert_eq!(out["status"], Value::Null);
    assert!(matches!(
        s.execute("steps.rewind", json!({"step": "a"})),
        Err(SessionError::NoStepGraph)
    ));
    s.set_step_graph(
        StepGraph::new(vec![
            StepDef {
                id: "a",
                depends: &[],
            },
            StepDef {
                id: "b",
                depends: &["a"],
            },
        ])
        .unwrap(),
    );
    let out = s.execute("steps.list", Value::Null).unwrap();
    assert_eq!(out["status"].as_array().unwrap().len(), 2);
    let out = s.execute("steps.rewind", json!({"step": "b"})).unwrap();
    assert_eq!(out["removed"], json!([]));
    assert!(matches!(
        s.execute("steps.rewind", json!({"step": "zzz"})),
        Err(SessionError::Step(_))
    ));
}

#[test]
fn module_registration_rejects_duplicates() {
    let mut s = session();
    assert!(matches!(
        s.register_module(Box::new(Dummy)),
        Err(SessionError::Module(_))
    ));
    s.set_module_enabled("dummy", false).unwrap();
    assert!(s.list_commands().iter().all(|c| c.source == "session"));
    assert!(matches!(
        s.execute("dummy.increment", json!({})),
        Err(SessionError::UnknownCommand(_))
    ));
}

#[test]
fn dcprj_round_trip_through_save_and_open() {
    let dir = tempfile::tempdir().unwrap();
    let stl = dir.path().join("upper.stl");
    write_tetra(&stl);
    let target = dir.path().join("case");

    let mut s = session();
    assert!(matches!(
        s.execute("project.save", json!({})),
        Err(SessionError::NoPath)
    ));
    s.execute("project.new", json!({"patient_code": "PZ-0001"}))
        .unwrap();
    s.execute("case.import", json!({"path": stl.to_str().unwrap()}))
        .unwrap();
    s.execute("dummy.increment", json!({"by": 7})).unwrap();
    let out = s
        .execute("project.save", json!({"path": target.to_str().unwrap()}))
        .unwrap();
    let saved = dir.path().join("case.dcprj");
    assert_eq!(out["path"], saved.display().to_string());
    assert!(out["bytes"].as_u64().unwrap() > 0);
    assert!(!s.is_dirty());
    assert_eq!(s.path(), Some(saved.as_path()));
    let before = s.inspect(None).unwrap();

    let mut t = session();
    let out = t
        .execute("project.open", json!({"path": saved.to_str().unwrap()}))
        .unwrap();
    assert_eq!(out["read_only"], false);
    assert_eq!(out["issues"], json!([]));
    assert_eq!(out["scans"], 1);
    assert_eq!(out["modules"], json!(["dummy"]));
    let after = t.inspect(None).unwrap();
    for key in ["case", "scans", "meshes", "modules"] {
        assert_eq!(after[key], before[key], "{key}");
    }
    assert_eq!(
        after["project"]["project_id"],
        before["project"]["project_id"]
    );
    assert_eq!(count(&t), 7);
    assert_eq!(t.state().case.patient_code, "PZ-0001");
    assert_eq!(t.undo_stack().undo_depth(), 0);

    // save again in place, then edit and reopen to drop the change
    t.execute("project.save", Value::Null).unwrap();
    t.execute("dummy.increment", json!({})).unwrap();
    let out = t
        .execute("project.open", json!({"path": saved.to_str().unwrap()}))
        .unwrap();
    assert_eq!(out["discarded_unsaved_changes"], true);
    assert_eq!(count(&t), 7);

    // without the module the state is kept opaque, and still inspectable
    let mut u = Session::new();
    let out = u
        .execute("project.open", json!({"path": saved.to_str().unwrap()}))
        .unwrap();
    assert_eq!(out["read_only"], false);
    assert_eq!(out["issues"].as_array().unwrap().len(), 1);
    assert_eq!(count(&u), 7);

    // a damaged file fails to open and leaves the session as it was
    std::fs::write(dir.path().join("bad.dcprj"), b"not a zip").unwrap();
    assert!(matches!(
        u.execute(
            "project.open",
            json!({"path": dir.path().join("bad.dcprj")})
        ),
        Err(SessionError::Project(_))
    ));
    assert_eq!(count(&u), 7);
}
