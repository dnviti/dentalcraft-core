//! The `scan.*` commands through a headless session (no module): the
//! simulated scanner, cleanup, mesh, repair and the import into the case.

use std::time::{Duration, Instant};

use dc_case::ScanRole;
use dc_i18n::Lang;
use dc_session::{Session, SessionError};
use serde_json::{Value, json};

fn exec(s: &mut Session, id: &str, params: Value) -> Value {
    s.execute(id, params)
        .unwrap_or_else(|e| panic!("{id} failed: {e}"))
}

#[test]
fn scan_commands_are_listed_with_schemas_and_translated_labels() {
    let mut s = Session::new();
    let ids: Vec<String> = s.list_commands().into_iter().map(|c| c.id).collect();
    for info in dc_scanner::command_infos() {
        assert!(ids.iter().any(|i| i == info.id), "{} not listed", info.id);
        let d = s.describe(info.id).unwrap();
        assert_eq!(d.source, dc_session::SESSION_SOURCE);
        assert_eq!(d.params_schema["type"], "object", "{}", info.id);
    }
    let d = s.describe("scan.mesh-build").unwrap();
    assert_eq!(d.params_schema["required"], json!(["resolution_mm"]));
    assert!(d.params_schema["properties"]["resolution_mm"].is_object());
    let d = s.describe("scan.cloud").unwrap();
    assert_eq!(d.params_schema["required"], json!(["op"]));
    dc_i18n::set_language(Lang::It).unwrap();
    assert_eq!(s.describe("scan.mesh-build").unwrap().label, "Crea mesh");
    assert_eq!(
        s.describe("scan.cloud").unwrap().label,
        "Modifica nuvola di punti"
    );
    dc_i18n::set_language(Lang::En).unwrap();
    assert_eq!(s.describe("scan.mesh-build").unwrap().label, "Build mesh");

    // refused calls are errors of the command, with the stable scan code
    let e = s.execute("scan.start", Value::Null).unwrap_err();
    assert!(
        matches!(&e, SessionError::Command { source, .. } if source.to_string().contains("not-connected")),
        "{e}"
    );
    let e = s
        .execute("scan.import", json!({"role": "upper"}))
        .unwrap_err();
    assert!(
        matches!(&e, SessionError::Command { source, .. } if source.to_string().contains("no-mesh")),
        "{e}"
    );
    assert!(
        s.execute("scan.cloud", json!({"op": {"op": "nope"}}))
            .is_err()
    );
    assert!(s.state().case.scans.is_empty());
}

#[test]
fn simulated_scan_end_to_end_into_the_case() {
    let mut s = Session::new();
    let devices = exec(&mut s, "scan.devices", Value::Null);
    assert!(
        devices["devices"]
            .as_array()
            .unwrap()
            .iter()
            .any(|d| d["id"] == "sim-typodont-upper")
    );

    let out = exec(
        &mut s,
        "scan.connect",
        json!({
            "backend": "sim",
            "device": "sim-typodont-upper",
            "options": {"realtime": false, "frames": 30, "width": 320, "height": 200}
        }),
    );
    assert_eq!(out["device"]["id"], "sim-typodont-upper");
    exec(&mut s, "scan.start", Value::Null);

    let deadline = Instant::now() + Duration::from_secs(300);
    let status = loop {
        let st = exec(&mut s, "scan.status", Value::Null);
        if st["frames"].as_u64().unwrap() >= 30 {
            break st;
        }
        assert!(
            Instant::now() < deadline,
            "the sim delivered no frames: {st}"
        );
        std::thread::sleep(Duration::from_millis(20));
    };
    assert_eq!(status["state"], "scanning");
    assert!(status["fused_frames"].as_u64().unwrap() > 0, "{status}");

    let stopped = exec(&mut s, "scan.stop", Value::Null);
    let points = stopped["points"].as_u64().unwrap();
    assert!(points > 1000, "{stopped}");
    // stopping again keeps the same scan
    assert_eq!(exec(&mut s, "scan.stop", Value::Null)["points"], points);

    let cleaned = exec(
        &mut s,
        "scan.cloud",
        json!({"op": {"op": "remove-statistical", "k": 16, "std_ratio": 2.0}}),
    );
    assert!(cleaned["points"].as_u64().unwrap() <= points);
    assert_eq!(cleaned["editor"]["can_undo"], true);

    let built = exec(&mut s, "scan.mesh-build", json!({"resolution_mm": 0.4}));
    assert!(built["faces"].as_u64().unwrap() > 1000, "{built}");
    let repaired = exec(
        &mut s,
        "scan.mesh",
        json!({"op": {"op": "fill-holes", "max_edges": 60}}),
    );
    assert!(repaired["holes_filled"].is_number(), "{repaired}");
    assert_eq!(repaired["editor"]["mesh_faces"], repaired["faces"]);

    // the scan editor has its own undo; the case does not know about it
    let undone = exec(&mut s, "scan.undo", Value::Null);
    assert_eq!(undone["done"], true);
    assert_eq!(undone["editor"]["mesh_faces"], built["faces"]);
    exec(&mut s, "scan.redo", Value::Null);

    let dir = tempfile::tempdir().unwrap();
    let ply = dir.path().join("scan.ply");
    exec(
        &mut s,
        "scan.export",
        json!({"path": ply.display().to_string(), "what": "mesh"}),
    );
    assert!(std::fs::metadata(&ply).unwrap().len() > 0);

    assert!(s.state().case.scans.is_empty());
    let imported = exec(&mut s, "scan.import", json!({"role": "upper"}));
    assert_eq!(imported["role"], "upper", "{imported}");
    let state = s.state().clone();
    assert_eq!(state.case.scans.len(), 1);
    let scan = &state.case.scans[0];
    assert_eq!(scan.role, ScanRole::Upper);
    let mesh = state.mesh(&scan.mesh).unwrap();
    // the import cleans degenerate faces: the mesh matches within 0.5 %
    let close = |got: usize, want: &Value| {
        let want = want.as_u64().unwrap() as f64;
        got as f64 <= want && got as f64 >= want * 0.995
    };
    assert!(close(mesh.face_count(), &repaired["faces"]));
    assert!(close(mesh.vertex_count(), &repaired["vertices"]));
    assert_eq!(imported["faces"], json!(mesh.face_count()));
    assert_eq!(imported["vertices"], json!(mesh.vertex_count()));
    // the import left nothing behind in the temp directory
    let leftovers = std::fs::read_dir(std::env::temp_dir())
        .unwrap()
        .filter_map(Result::ok)
        .filter(|e| {
            e.file_name()
                .to_string_lossy()
                .starts_with("dentalcraft-scan-")
        })
        .filter(|e| e.path().join("scan-upper.ply").exists())
        .count();
    assert_eq!(leftovers, 0);

    // one undoable step, like a file import
    assert_eq!(exec(&mut s, "edit.undo", Value::Null)["done"], true);
    assert!(s.state().case.scans.is_empty());
    assert_eq!(exec(&mut s, "edit.redo", Value::Null)["done"], true);
    assert_eq!(s.state().case.scans.len(), 1);

    // the controller is shared: the same scan is visible through it
    let controller = s.scan_controller();
    let c = controller.lock().unwrap();
    assert!(c.editor().unwrap().mesh().is_some());
    drop(c);

    exec(&mut s, "scan.reset", Value::Null);
    assert_eq!(
        exec(&mut s, "scan.status", Value::Null)["editor"],
        Value::Null
    );
    exec(&mut s, "scan.disconnect", Value::Null);
}

#[test]
fn turntable_commands_run_through_the_session() {
    let mut s = Session::new();
    let ids: Vec<String> = s.list_commands().into_iter().map(|c| c.id).collect();
    for id in [
        "scan.auto",
        "turntable.devices",
        "turntable.connect",
        "turntable.disconnect",
        "turntable.move",
        "turntable.speed",
        "turntable.stop",
        "turntable.home",
        "turntable.status",
        "turntable.calibrate",
        "turntable.calibration",
    ] {
        assert!(ids.iter().any(|i| i == id), "{id} not listed");
    }
    let d = s.describe("turntable.move").unwrap();
    assert_eq!(d.params_schema["required"], json!(["axis"]));

    // the discovery runs without the scan controller: a locked controller
    // does not block it
    let controller = s.scan_controller();
    let guard = controller.lock().unwrap();
    let out = exec(&mut s, "turntable.devices", json!({"timeout_s": 0}));
    drop(guard);
    assert!(
        out["devices"]
            .as_array()
            .unwrap()
            .iter()
            .any(|d| d["id"] == "sim-turntable")
    );

    let st = exec(&mut s, "turntable.status", Value::Null);
    assert_eq!(st["connected"], false);
    let e = s
        .execute("turntable.move", json!({"axis": "rotate", "by_deg": 10}))
        .unwrap_err();
    assert!(
        matches!(&e, SessionError::Command { source, .. } if source.to_string().contains("turntable-not-connected")),
        "{e}"
    );
    let dir = tempfile::tempdir().unwrap();
    controller.lock().unwrap().set_rig_dir(dir.path());
    exec(
        &mut s,
        "turntable.connect",
        json!({"backend": "sim", "options": {"fast": true}}),
    );
    exec(
        &mut s,
        "turntable.move",
        json!({"axis": "tilt", "to_deg": 10}),
    );
    let deadline = Instant::now() + Duration::from_secs(10);
    loop {
        let st = exec(&mut s, "turntable.status", Value::Null);
        if st["busy"] == false {
            assert_eq!(st["tilt_deg"], 10.0);
            break;
        }
        assert!(Instant::now() < deadline, "{st}");
        std::thread::sleep(Duration::from_millis(10));
    }

    // scan.auto needs a calibration, unless explicitly allowed without
    exec(
        &mut s,
        "scan.connect",
        json!({"backend": "sim", "options": {"realtime": false, "width": 160, "height": 100}}),
    );
    let auto = json!({"tilts_deg": [0], "rotation_deg": 20, "deg_per_s": 60, "settle_s": 0});
    let e = s.execute("scan.auto", auto.clone()).unwrap_err();
    assert!(
        matches!(&e, SessionError::Command { source, .. } if source.to_string().contains("rig-uncalibrated")),
        "{e}"
    );
    assert_eq!(
        exec(&mut s, "scan.status", Value::Null)["state"],
        "connected"
    );
    let mut allowed = auto;
    allowed["allow_uncalibrated"] = json!(true);
    assert_eq!(exec(&mut s, "scan.auto", allowed)["started"], true);
    let deadline = Instant::now() + Duration::from_secs(60);
    loop {
        let st = exec(&mut s, "scan.status", Value::Null);
        if st["auto"]["state"] == "done" {
            assert_eq!(st["state"], "paused");
            break;
        }
        assert_eq!(st["auto"]["state"], "running", "{st}");
        assert!(Instant::now() < deadline, "{st}");
        std::thread::sleep(Duration::from_millis(10));
    }
    assert!(
        exec(&mut s, "scan.stop", Value::Null)["points"]
            .as_u64()
            .unwrap()
            > 0
    );
    exec(&mut s, "scan.disconnect", Value::Null);
    assert_eq!(
        exec(&mut s, "turntable.calibration", Value::Null),
        Value::Null
    );
    exec(&mut s, "turntable.disconnect", Value::Null);
}
