//! Session tests: built-ins, a module command, undo/redo and a `.dcprj`
//! round trip through `project.save` / `project.open`.

use std::any::Any;
use std::path::Path;
use std::sync::{Mutex, MutexGuard, PoisonError};

use dc_case::{
    Case, CaseState, ModuleData, ModuleState, OpaqueModuleData, Pose, RoleSource, ScanRole,
};
use dc_command::{CommandError, JobCtx, StepDef, StepGraph};
use dc_i18n::Lang;
use dc_mesh::TriMesh;
use dc_module_api::{
    CommandDesc, CommandOutput, CommandSpec, MenuTree, Module, ModuleCodec, ModuleCtx,
};
use dc_scene::SceneItem;
use dc_session::{Session, SessionError};
use glam::{DVec3, Vec3};
use serde_json::{Value, json};

#[derive(Debug, Clone, Default, PartialEq, serde::Serialize, serde::Deserialize)]
struct Counter {
    count: u64,
}

/// Counts like a module result computed from the scans: replacing a scan
/// clears it.
impl ModuleData for Counter {
    fn schema_version(&self) -> u32 {
        1
    }
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    }
    fn clone_box(&self) -> Box<dyn ModuleData> {
        Box::new(self.clone())
    }
    fn to_json(&self) -> Result<Value, serde_json::Error> {
        serde_json::to_value(self)
    }
    fn scan_replaced(&mut self, _role: ScanRole) -> Vec<&'static str> {
        if self.count == 0 {
            return Vec::new();
        }
        self.count = 0;
        vec!["dummy.count"]
    }
}

/// Moves the world frame away from the identity.
fn shift_frame(state: &CaseState, _: &Value, ctx: &JobCtx) -> Result<CommandOutput, CommandError> {
    ctx.check_cancelled()?;
    let mut next = state.clone();
    next.case.world_frame.from_case = Pose::from_translation(DVec3::X);
    Ok((next, json!({})))
}

/// The language is process-wide: tests that read translated text hold this.
static LANG: Mutex<()> = Mutex::new(());

fn lang(l: Lang) -> MutexGuard<'static, ()> {
    let guard = LANG.lock().unwrap_or_else(PoisonError::into_inner);
    dc_i18n::set_language(l).unwrap();
    guard
}

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
        vec![
            CommandSpec {
                id: "dummy.increment",
                label_key: "dummy-increment",
                doc_key: "dummy-increment-doc",
                params_schema: || json!({"type": "object", "properties": {"by": {"type": "integer", "minimum": 0}}}),
                run: increment,
            },
            CommandSpec {
                id: "dummy.shift-frame",
                label_key: "dummy-shift-frame",
                doc_key: "dummy-shift-frame-doc",
                params_schema: || json!({"type": "object"}),
                run: shift_frame,
            },
        ]
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
    let _g = lang(Lang::En);
    let s = session();
    let list = s.list_commands();
    let ids: Vec<&str> = list.iter().map(|c| c.id.as_str()).collect();
    for id in [
        "project.new",
        "project.open",
        "project.save",
        "case.import",
        "case.set-role",
        "case.remove-scan",
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
    // exactly one of path and files; no single field is required
    assert_eq!(d.params_schema.get("required"), None);
    assert_eq!(
        d.params_schema["oneOf"],
        json!([{"required": ["path"]}, {"required": ["files"]}])
    );
    for key in ["path", "files", "role"] {
        assert!(d.params_schema["properties"][key].is_object(), "{key}");
    }
    let roles = format!("{}", d.params_schema);
    assert!(roles.contains("auto") && roles.contains("bite"));
    assert!(d.doc.contains("files") && d.doc.contains("role_source"));
    let d = s.describe("case.set-role").unwrap();
    assert_eq!(d.label, "Set scan role");
    assert_eq!(d.params_schema["required"], json!(["from", "to"]));
    assert!(d.params_schema["properties"]["scan_id"].is_object());
    let d = s.describe("case.remove-scan").unwrap();
    assert_eq!(d.label, "Remove scan");
    assert_eq!(d.params_schema["required"], json!(["scan_id"]));
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

// ---------------------------------------------------------------- import

fn tetra(dir: &Path, name: &str) -> String {
    let path = dir.join(name);
    write_tetra(&path);
    path.to_str().unwrap().to_owned()
}

fn import(s: &mut Session, params: Value) -> Result<Value, SessionError> {
    s.execute("case.import", params)
}

/// `(role, role_source, source_name)` of every scan, in case order.
fn scans(s: &Session) -> Vec<(String, String, String)> {
    s.inspect(Some("scans"))
        .unwrap()
        .as_array()
        .unwrap()
        .iter()
        .map(|v| {
            let text = |k: &str| v[k].as_str().unwrap().to_owned();
            (text("role"), text("role_source"), text("source_name"))
        })
        .collect()
}

fn row(role: &str, source: &str, name: &str) -> (String, String, String) {
    (role.to_owned(), source.to_owned(), name.to_owned())
}

fn scan_id(s: &Session, role: &str) -> String {
    let v = s.inspect(Some("scans")).unwrap();
    let scan = v.as_array().unwrap().iter().find(|x| x["role"] == role);
    scan.unwrap()["id"].as_str().unwrap().to_owned()
}

#[test]
fn import_output_keeps_the_single_file_fields_and_adds_the_rest() {
    let _g = lang(Lang::En);
    let dir = tempfile::tempdir().unwrap();
    let mut s = session();
    let out = import(&mut s, json!({"path": tetra(dir.path(), "upper.stl")})).unwrap();
    assert_eq!(out["role"], "upper");
    assert_eq!(out["role_source"], "file_name");
    assert_eq!(out["source_name"], "upper.stl");
    // a 10 mm model is read as centimetres, which the summary reports too
    assert_eq!(out["source_units"], "cm");
    assert_eq!(
        out["source_units"],
        s.inspect(Some("scans.0.source_units")).unwrap()
    );
    assert_eq!(out["vertices"], 4);
    assert_eq!(out["faces"], 4);
    assert_eq!(out["replaced"], false);
    assert_eq!(out["mesh"].as_str().unwrap().len(), 64);
    assert_eq!(out["scan_id"], scan_id(&s, "upper"));
    assert_eq!(out["failed"], json!([]));
    assert_eq!(out["moved"], json!([]));
    assert_eq!(out["module_cleared"], json!([]));
    assert_eq!(out["cleanup"], json!({"degenerate": 0, "duplicate": 0}));
    // the batch array holds the same fields, plus the path
    let one = &out["imported"][0];
    assert_eq!(out["imported"].as_array().unwrap().len(), 1);
    for key in [
        "scan_id",
        "role",
        "role_source",
        "source_name",
        "mesh",
        "faces",
    ] {
        assert_eq!(one[key], out[key], "{key}");
    }
    assert!(one["path"].as_str().unwrap().ends_with("upper.stl"));
    // the rescaling and the face count are reported
    let codes: Vec<&str> = out["warnings"]
        .as_array()
        .unwrap()
        .iter()
        .map(|w| w["code"].as_str().unwrap())
        .collect();
    assert_eq!(codes, ["rescaled", "low_polygons"]);

    // a scan of the same role is replaced
    let again = import(
        &mut s,
        json!({"path": tetra(dir.path(), "upper2.stl"), "role": "upper"}),
    )
    .unwrap();
    assert_eq!(again["replaced"], true);
    assert_eq!(again["role_source"], "explicit");
    assert_eq!(
        again["warnings"]
            .as_array()
            .unwrap()
            .iter()
            .filter(|w| w["code"] == "replaced")
            .count(),
        1
    );
    assert_eq!(scans(&s), [row("upper", "explicit", "upper2.stl")]);
}

#[test]
fn import_warnings_are_json_in_both_languages() {
    let _g = lang(Lang::En);
    let dir = tempfile::tempdir().unwrap();
    let stl = tetra(dir.path(), "scan.stl");
    let mut s = session();
    let en = import(&mut s, json!({"path": stl})).unwrap();
    dc_i18n::set_language(Lang::It).unwrap();
    s.execute("project.new", json!({})).unwrap();
    let it = import(&mut s, json!({"path": stl, "role": "auto"})).unwrap();

    let by_code = |v: &Value, code: &str| -> Value {
        let w = v["warnings"].as_array().unwrap();
        w.iter().find(|w| w["code"] == code).unwrap().clone()
    };
    for code in ["low_polygons", "provisional"] {
        let (e, i) = (by_code(&en, code), by_code(&it, code));
        assert_eq!(e["key"], i["key"], "{code}");
        assert_eq!(e["args"], i["args"], "{code}");
        assert_ne!(e["message"], i["message"], "{code}");
        assert_eq!(e["args"]["name"], "scan.stl");
    }
    let p = by_code(&en, "provisional");
    assert_eq!(p["key"], "case-import-warn-provisional");
    assert_eq!(p["args"]["role"], "lower");
    assert_eq!(
        p["message"],
        "scan.stl: the arch could not be recognised, so it is placed as lower arch for now. Please check."
    );
    assert!(
        by_code(&it, "provisional")["message"]
            .as_str()
            .unwrap()
            .contains("non è stato possibile riconoscere")
    );
    assert!(
        by_code(&it, "low_polygons")["message"]
            .as_str()
            .unwrap()
            .contains("facce")
    );
}

#[test]
fn an_unhinted_file_is_provisional_and_a_named_one_makes_it_move() {
    let _g = lang(Lang::En);
    let dir = tempfile::tempdir().unwrap();
    let mut s = session();
    let out = import(&mut s, json!({"path": tetra(dir.path(), "scan.stl")})).unwrap();
    assert_eq!(out["role"], "lower");
    assert_eq!(out["role_source"], "provisional");
    assert_eq!(scans(&s), [row("lower", "provisional", "scan.stl")]);
    assert_eq!(
        s.inspect(Some("scans.0.role_source")).unwrap(),
        "provisional"
    );

    // a name that claims the lower arch moves the guess to the free arch
    let out = import(&mut s, json!({"path": tetra(dir.path(), "lower.stl")})).unwrap();
    assert_eq!(out["role"], "lower");
    assert_eq!(out["replaced"], false);
    let moved = out["moved"].as_array().unwrap();
    assert_eq!(moved.len(), 1);
    assert_eq!(
        (moved[0]["from"].clone(), moved[0]["to"].clone()),
        (json!("lower"), json!("upper"))
    );
    assert_eq!(
        scans(&s),
        [
            row("upper", "provisional", "scan.stl"),
            row("lower", "file_name", "lower.stl"),
        ]
    );
}

#[test]
fn a_batch_imports_several_files_as_one_undo_step() {
    let dir = tempfile::tempdir().unwrap();
    let mut s = session();
    let depth = s.undo_stack().undo_depth();
    let out = import(
        &mut s,
        json!({"files": [
            {"path": tetra(dir.path(), "a.stl"), "role": "bite"},
            {"path": tetra(dir.path(), "b.stl"), "role": "auto"},
            {"path": tetra(dir.path(), "upper.stl")},
        ]}),
    )
    .unwrap();
    assert_eq!(s.undo_stack().undo_depth(), depth + 1);
    let imported = out["imported"].as_array().unwrap();
    assert_eq!(imported.len(), 3);
    assert_eq!(imported[0]["source_name"], "a.stl");
    // the single-file fields describe the first one
    assert_eq!(out["source_name"], "a.stl");
    assert_eq!(out["role"], "bite");
    assert_eq!(
        scans(&s),
        [
            row("upper", "file_name", "upper.stl"),
            row("lower", "provisional", "b.stl"),
            row("bite", "explicit", "a.stl"),
        ]
    );
    assert!(s.undo());
    assert_eq!(s.inspect(Some("scans")).unwrap(), json!([]));
    assert_eq!(s.undo_stack().undo_depth(), depth);
}

#[test]
fn a_partial_batch_succeeds_with_the_failures_listed() {
    let _g = lang(Lang::En);
    let dir = tempfile::tempdir().unwrap();
    let missing = dir.path().join("lower_missing.stl");
    let mut s = session();
    let out = import(
        &mut s,
        json!({"files": [
            {"path": missing.to_str().unwrap()},
            {"path": tetra(dir.path(), "upper.stl")},
        ]}),
    )
    .unwrap();
    assert_eq!(out["imported"].as_array().unwrap().len(), 1);
    let failed = out["failed"].as_array().unwrap();
    assert_eq!(failed.len(), 1);
    assert_eq!(failed[0]["code"], "unreadable");
    assert!(
        failed[0]["path"]
            .as_str()
            .unwrap()
            .ends_with("lower_missing.stl")
    );
    assert_eq!(
        failed[0]["reason"],
        "lower_missing.stl: the file cannot be read."
    );
    assert!(failed[0]["detail"].is_string());
    assert_eq!(scans(&s), [row("upper", "file_name", "upper.stl")]);
    assert_eq!(s.undo_stack().undo_depth(), 1);
}

#[test]
fn nothing_imported_is_an_error_and_leaves_the_state_unchanged() {
    let _g = lang(Lang::En);
    let dir = tempfile::tempdir().unwrap();
    let mut s = session();
    let err = import(&mut s, json!({"path": dir.path().join("nope.stl")})).unwrap_err();
    let text = err.to_string();
    assert!(text.starts_with("case.import: "), "{text}");
    assert!(
        text.contains("nope.stl: the file cannot be read."),
        "{text}"
    );
    assert!(s.state().case.scans.is_empty());
    assert_eq!(s.undo_stack().undo_depth(), 0);

    // both arches taken and an unhinted file: it needs a role
    import(
        &mut s,
        json!({"files": [
            {"path": tetra(dir.path(), "upper.stl")},
            {"path": tetra(dir.path(), "lower.stl")},
        ]}),
    )
    .unwrap();
    let before = s.inspect(None).unwrap();
    let depth = s.undo_stack().undo_depth();
    let err = import(&mut s, json!({"path": tetra(dir.path(), "scan.stl")})).unwrap_err();
    assert!(
        err.to_string().contains(
            "scan.stl: both arches are already taken. Choose its role in the Import scans panel"
        ),
        "{err}"
    );
    assert_eq!(s.undo_stack().undo_depth(), depth);
    assert_eq!(s.inspect(None).unwrap(), before);

    // with a role it replaces, as it was asked to
    let out = import(
        &mut s,
        json!({"path": tetra(dir.path(), "scan.stl"), "role": "lower"}),
    )
    .unwrap();
    assert_eq!(out["replaced"], true);
}

#[test]
fn an_empty_mesh_never_replaces_a_good_scan() {
    let dir = tempfile::tempdir().unwrap();
    let mut s = session();
    import(&mut s, json!({"path": tetra(dir.path(), "upper.stl")})).unwrap();
    let good = scan_id(&s, "upper");
    let before = s.inspect(None).unwrap();

    // binary STL: no triangle at all, then one degenerate triangle
    let mut none = vec![0u8; 80];
    none.extend(0u32.to_le_bytes());
    let mut degenerate = vec![0u8; 80];
    degenerate.extend(1u32.to_le_bytes());
    degenerate.extend([0u8; 50]);
    for (name, bytes) in [("none", none), ("degenerate", degenerate)] {
        let path = dir.path().join(format!("{name}.stl"));
        std::fs::write(&path, bytes).unwrap();
        for role in ["upper", "auto"] {
            let err = import(
                &mut s,
                json!({"path": path.to_str().unwrap(), "role": role}),
            );
            assert!(err.is_err(), "{name} {role}");
        }
        // inside a batch, the good file still comes in and the empty one is listed
        let out = import(
            &mut s,
            json!({"files": [
                {"path": path.to_str().unwrap(), "role": "lower"},
                {"path": tetra(dir.path(), "lower.stl")},
            ]}),
        )
        .unwrap();
        assert_eq!(out["failed"].as_array().unwrap().len(), 1);
        assert_eq!(scan_id(&s, "upper"), good, "{name}");
        assert!(s.undo());
    }
    assert_eq!(s.inspect(None).unwrap()["scans"], before["scans"]);
    assert_eq!(scan_id(&s, "upper"), good);
}

#[test]
fn import_params_need_exactly_one_of_path_and_files() {
    let _g = lang(Lang::En);
    let dir = tempfile::tempdir().unwrap();
    let a = tetra(dir.path(), "a.stl");
    let mut s = session();
    for params in [
        json!({"path": a, "files": [{"path": a}]}),
        json!({}),
        json!({"role": "upper"}),
        json!({"files": []}),
        json!({"files": [{"path": a, "x": 1}]}),
        json!({"path": a, "role": "middle"}),
        json!({"path": a, "extra": true}),
    ] {
        let err = import(&mut s, params.clone());
        assert!(
            matches!(
                err,
                Err(SessionError::Command {
                    source: CommandError::Invalid(_),
                    ..
                })
            ),
            "{params}"
        );
    }
    let err = import(&mut s, json!({"path": a, "files": [{"path": a}]})).unwrap_err();
    assert!(
        err.to_string()
            .contains("Give exactly one of path (one file) or files"),
        "{err}"
    );
    dc_i18n::set_language(Lang::It).unwrap();
    let err = import(&mut s, json!({})).unwrap_err();
    assert!(err.to_string().contains("Indica esattamente uno"), "{err}");
    assert!(s.state().case.scans.is_empty());
    assert_eq!(s.undo_stack().undo_depth(), 0);
}

/// An upper or lower role on several files is a clash whatever the names say,
/// and nothing is imported; bite keeps the first two.
#[test]
fn a_batch_role_applies_to_every_file() {
    let _guard = lang(Lang::En);
    let dir = tempfile::tempdir().unwrap();
    let mut s = session();
    let files = json!([
        {"path": tetra(dir.path(), "a.stl")},
        {"path": tetra(dir.path(), "lower.stl")},
    ]);
    let err = import(&mut s, json!({"files": files, "role": "upper"})).unwrap_err();
    let text = err.to_string();
    assert!(
        text.contains("a.stl:") && text.contains("lower.stl:"),
        "{text}"
    );
    assert!(text.contains("more than one file"), "{text}");
    assert!(scans(&s).is_empty());

    let out = import(&mut s, json!({"files": files, "role": "bite"})).unwrap();
    assert_eq!(out["imported"].as_array().unwrap().len(), 2);
    assert_eq!(
        scans(&s),
        [
            row("bite", "explicit", "a.stl"),
            row("bite", "explicit", "lower.stl"),
        ]
    );
}

/// A bite scan keeps its size: the tetrahedron measures 10, which looks like
/// centimetres, and an arch of the same size grows.
#[test]
fn a_bite_keeps_its_size_with_a_units_warning() {
    let dir = tempfile::tempdir().unwrap();
    let mut s = session();
    let out = import(
        &mut s,
        json!({"path": tetra(dir.path(), "a.stl"), "role": "bite"}),
    )
    .unwrap();
    assert_eq!(out["source_units"], "mm");
    assert_eq!(out["warnings"][0]["code"], "units_suspect");
    assert_eq!(out["warnings"][0]["args"]["units"], "cm");
    let out = import(
        &mut s,
        json!({"path": tetra(dir.path(), "b.stl"), "role": "upper"}),
    )
    .unwrap();
    assert_eq!(out["source_units"], "cm");
    assert_eq!(out["warnings"][0]["code"], "rescaled");
}

/// The state of a module this build cannot read, on a case with both arches.
fn locked_state(dir: &Path) -> (CaseState, CommandSpec, CommandSpec) {
    let specs = dc_session::builtins::specs();
    let find = |id: &str| *specs.iter().find(|s| s.id == id).unwrap();
    let (import, set_role) = (find("case.import"), find("case.set-role"));
    let params = json!({"files": [
        {"path": tetra(dir, "upper.stl")},
        {"path": tetra(dir, "lower.stl")},
    ]});
    let state = CaseState::new(Case::new("T"));
    let (mut state, _) = import
        .execute(&state, &params, &JobCtx::detached())
        .unwrap();
    state.set_module(
        "future",
        ModuleState::new(OpaqueModuleData {
            schema_version: 9,
            value: json!({"results": true}),
        }),
    );
    (state, import, set_role)
}

#[test]
fn an_opaque_module_stops_set_role_on_the_arches() {
    let dir = tempfile::tempdir().unwrap();
    let (state, _, set_role) = locked_state(dir.path());
    let ctx = JobCtx::detached();
    for params in [
        json!({"from": "upper", "to": "lower"}),
        json!({"from": "upper", "to": "bite"}),
    ] {
        let err = set_role.execute(&state, &params, &ctx).unwrap_err();
        assert!(
            matches!(&err, CommandError::Invalid(m) if m.contains("future")),
            "{err}"
        );
    }
    // a role confirmed where it is moves nothing
    let params = json!({"from": "upper", "to": "upper"});
    let (next, out) = set_role.execute(&state, &params, &ctx).unwrap();
    assert_eq!(out["role_source"], "explicit");
    assert_eq!(next.case.scans.len(), 2);
}

#[test]
fn an_opaque_module_fails_an_import_that_replaces_an_arch() {
    let dir = tempfile::tempdir().unwrap();
    let (state, import, _) = locked_state(dir.path());
    let ctx = JobCtx::detached();
    let params = json!({"files": [
        {"path": tetra(dir.path(), "upper2.stl")},
        {"path": tetra(dir.path(), "lower2.stl")},
    ]});
    let err = import.execute(&state, &params, &ctx).unwrap_err();
    let CommandError::Failed(text) = err else {
        panic!("{err}")
    };
    // the reason is about the project: once, not once per file
    assert_eq!(text.matches("future").count(), 1, "{text}");
    // a bite goes in
    let params = json!({"path": tetra(dir.path(), "morso.stl")});
    let (next, _) = import.execute(&state, &params, &ctx).unwrap();
    assert_eq!(next.case.scans.len(), 3);
}

// -------------------------------------------------------------- set-role

fn set_role(s: &mut Session, params: Value) -> Result<Value, SessionError> {
    s.execute("case.set-role", params)
}

fn frame_is_default(s: &Session) -> bool {
    s.state().case.world_frame == dc_case::WorldFrame::default()
}

#[test]
fn set_role_swaps_the_arches_and_undoes() {
    let dir = tempfile::tempdir().unwrap();
    let mut s = session();
    import(
        &mut s,
        json!({"files": [
            {"path": tetra(dir.path(), "upper.stl")},
            {"path": tetra(dir.path(), "lower.stl")},
        ]}),
    )
    .unwrap();
    s.execute("dummy.increment", json!({"by": 3})).unwrap();
    s.execute("dummy.shift-frame", json!({})).unwrap();
    assert!(!frame_is_default(&s));
    let (upper, lower) = (scan_id(&s, "upper"), scan_id(&s, "lower"));
    let depth = s.undo_stack().undo_depth();

    let out = set_role(&mut s, json!({"from": "upper", "to": "lower"})).unwrap();
    assert_eq!(s.undo_stack().undo_depth(), depth + 1);
    assert_eq!(out["scan_id"], upper);
    assert_eq!(
        (out["from"].clone(), out["to"].clone()),
        (json!("upper"), json!("lower"))
    );
    assert_eq!(out["role_source"], "explicit");
    assert_eq!(out["swapped"]["scan_id"], lower);
    assert_eq!(out["swapped"]["from"], "lower");
    assert_eq!(out["swapped"]["to"], "upper");
    assert_eq!(out["world_frame_reset"], true);
    assert_eq!(out["module_cleared"], json!(["dummy.count"]));
    assert_eq!(scan_id(&s, "lower"), upper);
    assert_eq!(scan_id(&s, "upper"), lower);
    assert_eq!(
        scans(&s),
        [
            row("upper", "explicit", "lower.stl"),
            row("lower", "explicit", "upper.stl"),
        ]
    );
    assert!(frame_is_default(&s));
    assert_eq!(count(&s), 0);

    // undo brings back the roles, how they were chosen, the frame and the result
    assert!(s.undo());
    assert_eq!(
        scans(&s),
        [
            row("upper", "file_name", "upper.stl"),
            row("lower", "file_name", "lower.stl"),
        ]
    );
    assert!(!frame_is_default(&s));
    assert_eq!(count(&s), 3);
    assert!(s.redo());
    assert_eq!(scan_id(&s, "lower"), upper);
}

#[test]
fn set_role_moves_to_a_free_arch_and_confirms_a_guess() {
    let dir = tempfile::tempdir().unwrap();
    let mut s = session();
    import(&mut s, json!({"path": tetra(dir.path(), "scan.stl")})).unwrap();
    assert_eq!(scans(&s), [row("lower", "provisional", "scan.stl")]);
    s.execute("dummy.increment", json!({})).unwrap();
    s.execute("dummy.shift-frame", json!({})).unwrap();

    // confirming the guess: same role, now explicit, nothing recomputed
    let out = set_role(&mut s, json!({"from": "lower", "to": "lower"})).unwrap();
    assert_eq!(out["swapped"], Value::Null);
    assert_eq!(out["world_frame_reset"], false);
    assert_eq!(out["module_cleared"], json!([]));
    assert_eq!(scans(&s), [row("lower", "explicit", "scan.stl")]);
    assert_eq!(count(&s), 1);
    assert!(!frame_is_default(&s));

    // moving to the free arch: no swap, the module result goes
    let out = set_role(&mut s, json!({"from": "lower", "to": "upper"})).unwrap();
    assert_eq!(out["swapped"], Value::Null);
    assert_eq!(out["world_frame_reset"], true);
    assert_eq!(out["module_cleared"], json!(["dummy.count"]));
    assert_eq!(scans(&s), [row("upper", "explicit", "scan.stl")]);
    assert!(frame_is_default(&s));
}

#[test]
fn set_role_with_the_same_role_confirms_a_detected_unsure_scan() {
    let dir = tempfile::tempdir().unwrap();
    let specs = dc_session::builtins::specs();
    let find = |id: &str| *specs.iter().find(|s| s.id == id).unwrap();
    let ctx = JobCtx::detached();
    let state = CaseState::new(Case::new("T"));
    let params = json!({"path": tetra(dir.path(), "scan_a.stl"), "role": "lower"});
    let (mut state, _) = find("case.import").execute(&state, &params, &ctx).unwrap();
    state.case.scans[0].role_source = RoleSource::DetectedUnsure;

    let params = json!({"from": "lower", "to": "lower"});
    let (next, out) = find("case.set-role")
        .execute(&state, &params, &ctx)
        .unwrap();
    assert_eq!(out["role_source"], "explicit");
    assert_eq!(out["world_frame_reset"], false);
    assert_eq!(out["module_cleared"], json!([]));
    assert_eq!(next.case.scans[0].role_source, RoleSource::Explicit);
    assert_eq!(next.case.scans[0].id, state.case.scans[0].id);
}

#[test]
fn set_role_follows_the_bite_limit_and_needs_a_scan_id() {
    let _g = lang(Lang::En);
    let dir = tempfile::tempdir().unwrap();
    let mut s = session();
    import(
        &mut s,
        json!({"files": [
            {"path": tetra(dir.path(), "a.stl"), "role": "bite"},
            {"path": tetra(dir.path(), "b.stl"), "role": "bite"},
            {"path": tetra(dir.path(), "lower.stl")},
        ]}),
    )
    .unwrap();
    let ids: Vec<String> = s
        .state()
        .case
        .scans_of(ScanRole::Bite)
        .map(|x| x.id.to_string())
        .collect();
    assert_eq!(ids.len(), 2);
    let before = s.inspect(None).unwrap();
    let depth = s.undo_stack().undo_depth();

    // two bite scans: `from: bite` is ambiguous without scan_id
    let err = set_role(&mut s, json!({"from": "bite", "to": "upper"})).unwrap_err();
    assert!(
        err.to_string()
            .contains("more than one scan with the role bite"),
        "{err}"
    );
    // wrong or malformed ids
    let lower = scan_id(&s, "lower");
    let err = set_role(
        &mut s,
        json!({"from": "bite", "to": "upper", "scan_id": lower}),
    )
    .unwrap_err();
    assert!(
        err.to_string().contains("does not have the role bite"),
        "{err}"
    );
    let err = set_role(
        &mut s,
        json!({"from": "bite", "to": "upper", "scan_id": "x"}),
    )
    .unwrap_err();
    assert!(err.to_string().contains("not a valid scan id"), "{err}");
    // a third scan cannot join the bite
    let err = set_role(&mut s, json!({"from": "lower", "to": "bite"})).unwrap_err();
    assert!(err.to_string().contains("at most 2 bite scans"), "{err}");
    // nothing on the role
    let err = set_role(&mut s, json!({"from": "upper", "to": "lower"})).unwrap_err();
    assert!(
        err.to_string().contains("no scan with the role upper"),
        "{err}"
    );
    assert_eq!(s.undo_stack().undo_depth(), depth);
    assert_eq!(s.inspect(None).unwrap(), before);

    // a bite scan to the free arch: one bite is left
    let out = set_role(
        &mut s,
        json!({"from": "bite", "to": "upper", "scan_id": ids[1]}),
    )
    .unwrap();
    assert_eq!(out["scan_id"], ids[1]);
    assert_eq!(out["swapped"], Value::Null);
    assert_eq!(scan_id(&s, "upper"), ids[1]);
    assert_eq!(s.state().case.scans_of(ScanRole::Bite).count(), 1);
    // a single bite scan needs no scan_id; the arch scan there swaps to the bite
    let out = set_role(&mut s, json!({"from": "bite", "to": "lower"})).unwrap();
    assert_eq!(out["scan_id"], ids[0]);
    assert_eq!(out["swapped"]["source_name"], "lower.stl");
    assert_eq!(
        scans(&s),
        [
            row("upper", "explicit", "b.stl"),
            row("lower", "explicit", "a.stl"),
            row("bite", "explicit", "lower.stl"),
        ]
    );
    // back to two bite scans by moving the upper one onto the bite
    set_role(&mut s, json!({"from": "upper", "to": "bite"})).unwrap();
    assert_eq!(s.state().case.scans_of(ScanRole::Bite).count(), 2);
    assert!(s.state().case.scan(ScanRole::Upper).is_none());
    // two bite scans and an occupied arch: the chosen bite swaps with it
    let b = scan_id(&s, "lower");
    let bite = s.state().case.scans_of(ScanRole::Bite).next().unwrap().id;
    let out = set_role(
        &mut s,
        json!({"from": "bite", "to": "lower", "scan_id": bite.to_string()}),
    )
    .unwrap();
    assert_eq!(out["swapped"]["scan_id"], b);
    assert_eq!(s.state().case.scans_of(ScanRole::Bite).count(), 2);
}

#[test]
fn set_role_rejects_bad_parameters() {
    let mut s = session();
    for params in [
        json!({}),
        json!({"from": "upper"}),
        json!({"from": "auto", "to": "upper"}),
        json!({"from": "upper", "to": "lower", "extra": 1}),
    ] {
        assert!(set_role(&mut s, params.clone()).is_err(), "{params}");
    }
}

// ------------------------------------------------------------ remove-scan

/// A closed tetrahedron `size` wide, written as binary STL: a mesh (and so
/// a blob) of its own for every size.
fn sized_tetra(dir: &Path, name: &str, size: f32) -> String {
    let mesh = TriMesh::new(
        vec![
            Vec3::ZERO,
            Vec3::new(size, 0.0, 0.0),
            Vec3::new(0.0, size, 0.0),
            Vec3::new(0.0, 0.0, size),
        ],
        vec![[0, 2, 1], [0, 1, 3], [1, 2, 3], [0, 3, 2]],
    );
    let path = dir.join(name);
    dc_mesh::io::write(&path, &mesh, dc_mesh::MeshFormat::Stl).unwrap();
    path.to_str().unwrap().to_owned()
}

fn remove(s: &mut Session, id: &str) -> Result<Value, SessionError> {
    s.execute("case.remove-scan", json!({ "scan_id": id }))
}

/// Upper, lower and two bite scans, each with its own mesh, a module result
/// and a world frame moved away from the identity.
fn four_scans(dir: &Path) -> Session {
    let mut s = session();
    import(
        &mut s,
        json!({"files": [
            {"path": sized_tetra(dir, "upper.stl", 10.0)},
            {"path": sized_tetra(dir, "lower.stl", 11.0)},
            {"path": sized_tetra(dir, "a.stl", 12.0), "role": "bite"},
            {"path": sized_tetra(dir, "b.stl", 13.0), "role": "bite"},
        ]}),
    )
    .unwrap();
    s.execute("dummy.increment", json!({"by": 3})).unwrap();
    s.execute("dummy.shift-frame", json!({})).unwrap();
    assert_eq!(s.state().blobs.len(), 4);
    s
}

fn bite_ids(s: &Session) -> Vec<String> {
    let bites = s.state().case.scans_of(ScanRole::Bite);
    bites.map(|x| x.id.to_string()).collect()
}

#[test]
fn remove_scan_takes_a_bite_scan_out_and_undo_brings_it_back() {
    let dir = tempfile::tempdir().unwrap();
    let mut s = four_scans(dir.path());
    let before = s.inspect(None).unwrap();
    let depth = s.undo_stack().undo_depth();
    let bites = bite_ids(&s);
    let mesh = s.state().case.scans_of(ScanRole::Bite).nth(1).unwrap().mesh;

    let out = remove(&mut s, &bites[1]).unwrap();
    assert_eq!(s.undo_stack().undo_depth(), depth + 1);
    assert_eq!(out["scan_id"], bites[1]);
    assert_eq!(out["source_name"], "b.stl");
    assert_eq!(out["role"], "bite");
    // a bite scan leaves the world frame; the module says what it cleared
    assert_eq!(out["world_frame_reset"], false);
    assert_eq!(out["module_cleared"], json!(["dummy.count"]));
    assert!(!frame_is_default(&s));
    assert_eq!(bite_ids(&s), [bites[0].clone()]);
    assert_eq!(s.state().case.scans.len(), 3);
    // its mesh left the case with it
    assert!(!s.state().blobs.contains(&mesh));
    assert_eq!(s.state().blobs.len(), 3);

    assert!(s.undo());
    assert_eq!(s.inspect(None).unwrap()["scans"], before["scans"]);
    assert_eq!(s.inspect(None).unwrap()["meshes"], before["meshes"]);
    assert_eq!(count(&s), 3);
    assert!(s.redo());
    assert_eq!(bite_ids(&s), [bites[0].clone()]);
}

#[test]
fn remove_scan_of_an_arch_resets_the_frame_and_clears_the_modules() {
    let dir = tempfile::tempdir().unwrap();
    for role in ["upper", "lower"] {
        let mut s = four_scans(dir.path());
        let id = scan_id(&s, role);
        let out = remove(&mut s, &id).unwrap();
        assert_eq!(out["role"], role);
        assert_eq!(out["world_frame_reset"], true);
        assert_eq!(out["module_cleared"], json!(["dummy.count"]));
        assert!(frame_is_default(&s));
        assert_eq!(count(&s), 0);
        assert!(s.state().case.scans.iter().all(|x| x.id.to_string() != id));
        assert_eq!(s.state().case.scans.len(), 3);
        assert_eq!(s.state().blobs.len(), 3);
        // undo: the scan, the frame and the module result come back
        assert!(s.undo());
        assert_eq!(scan_id(&s, role), id);
        assert!(!frame_is_default(&s));
        assert_eq!(count(&s), 3);
    }
}

#[test]
fn remove_scan_refuses_unknown_and_malformed_ids() {
    let _g = lang(Lang::En);
    let dir = tempfile::tempdir().unwrap();
    let mut s = four_scans(dir.path());
    let before = s.inspect(None).unwrap();
    let depth = s.undo_stack().undo_depth();
    let unknown = uuid::Uuid::new_v4().to_string();
    let err = remove(&mut s, &unknown).unwrap_err();
    assert!(
        err.to_string()
            .contains(&format!("no scan with the id {unknown}")),
        "{err}"
    );
    let err = remove(&mut s, "x").unwrap_err();
    assert!(
        err.to_string().contains("x is not a valid scan id"),
        "{err}"
    );
    for params in [json!({}), json!({"scan_id": unknown, "extra": 1})] {
        assert!(
            s.execute("case.remove-scan", params.clone()).is_err(),
            "{params}"
        );
    }
    assert_eq!(s.undo_stack().undo_depth(), depth);
    assert_eq!(s.inspect(None).unwrap(), before);
}

#[test]
fn an_opaque_module_stops_removing_an_arch_but_not_a_bite_scan() {
    let dir = tempfile::tempdir().unwrap();
    let (state, import, _) = locked_state(dir.path());
    let specs = dc_session::builtins::specs();
    let remove = *specs.iter().find(|s| s.id == "case.remove-scan").unwrap();
    let ctx = JobCtx::detached();
    let upper = state.case.scan(ScanRole::Upper).unwrap().id.to_string();
    let err = remove
        .execute(&state, &json!({ "scan_id": upper }), &ctx)
        .unwrap_err();
    assert!(
        matches!(&err, CommandError::Invalid(m) if m.contains("future")),
        "{err}"
    );
    let params = json!({"path": tetra(dir.path(), "morso.stl")});
    let (state, _) = import.execute(&state, &params, &ctx).unwrap();
    let bite = state.case.scan(ScanRole::Bite).unwrap().id.to_string();
    let (next, out) = remove
        .execute(&state, &json!({ "scan_id": bite }), &ctx)
        .unwrap();
    assert_eq!(out["role"], "bite");
    assert_eq!(next.case.scans.len(), 2);
}

/// The `blobs/` entries of a saved project.
fn saved_blobs(path: &Path) -> usize {
    let out = dc_project::load(path, &dc_project::ModuleCodecs::new()).unwrap();
    let names = out.project.manifest.entries.keys();
    names.filter(|k| k.starts_with("blobs/")).count()
}

#[test]
fn a_removed_scan_leaves_no_blob_in_the_saved_project() {
    let dir = tempfile::tempdir().unwrap();
    let mut s = four_scans(dir.path());
    let before = dir.path().join("before.dcprj");
    s.execute("project.save", json!({ "path": before }))
        .unwrap();
    let bites = bite_ids(&s);
    remove(&mut s, &bites[0]).unwrap();
    let after = dir.path().join("after.dcprj");
    s.execute("project.save", json!({ "path": after })).unwrap();

    assert_eq!(saved_blobs(&before), 4);
    assert_eq!(saved_blobs(&after), 3);
    let size = |p: &Path| std::fs::metadata(p).unwrap().len();
    assert!(size(&after) < size(&before));
    let mut t = session();
    let out = t.execute("project.open", json!({ "path": after })).unwrap();
    assert_eq!(out["scans"], 3);
    assert_eq!(out["issues"], json!([]));
    assert_eq!(bite_ids(&t), [bites[1].clone()]);
}
