//! The link to the turntable drops in the middle of a stop-and-capture
//! `scan.auto` (simulated rig, link fault injected in the simulated table):
//! the job reconnects and resumes at the stop where it was; when the link
//! does not come back the job fails but keeps its stops, which can be
//! exported and rebuilt.

use std::time::{Duration, Instant};

use dc_scanner::ScanController;
use dc_scanner::rig::{RigCalibration, rfc3339, save_calibration};
use dc_scanner::sim::{self, SimOptions};
use glam::Vec3;
use serde_json::{Value, json};

const WIDTH: u32 = 120;
const HEIGHT: u32 = 75;

fn run(c: &mut ScanController, id: &str, params: Value) -> Value {
    c.execute(id, &params, &|| false)
        .unwrap_or_else(|e| panic!("{id} {params}: {e} ({})", e.code()))
}

/// Polls `scan.status` until the automatic scan leaves `running`.
fn wait_auto(c: &mut ScanController) -> Value {
    let deadline = Instant::now() + Duration::from_secs(240);
    loop {
        let job = run(c, "scan.status", Value::Null)["auto"].clone();
        if job["state"] != "running" && job["state"] != "paused" {
            return job;
        }
        assert!(Instant::now() < deadline, "{job}");
        std::thread::sleep(Duration::from_millis(20));
    }
}

/// The exact calibration of the simulated rig (scene +Z the rotation
/// axis through the origin, tilt about +X under the platter), stored in
/// `dir`, so the stop-and-capture mode runs without measuring it.
fn store_true_calibration(dir: &std::path::Path) {
    let options = SimOptions {
        realtime: false,
        width: WIDTH,
        height: HEIGHT,
        ..SimOptions::default()
    };
    let to_cam = sim::rig_camera_pose(&options).inverse();
    let axis_dir = to_cam.transform_vector3(Vec3::Z).normalize();
    let c = RigCalibration {
        axis_point: to_cam.transform_point3(Vec3::ZERO).into(),
        axis_dir: axis_dir.into(),
        platter_up: axis_dir.into(),
        tilt_axis_point: Some(
            to_cam
                .transform_point3(Vec3::new(0.0, 0.0, -sim::RIG_TILT_PIVOT_MM))
                .into(),
        ),
        tilt_axis_dir: Some(to_cam.transform_vector3(Vec3::X).normalize().into()),
        scanner_serial: None,
        turntable_id: "sim-turntable".into(),
        created: rfc3339(std::time::SystemTime::now()),
        fit_rms_deg: 0.0,
        fit_rms_mm: 0.0,
        angle_gain: 1.0,
        fit_pairs: 24,
        imu_up: None,
    };
    save_calibration(dir, &c).unwrap();
}

/// A controller on the simulated rig, calibrated.
fn rig(dir: &std::path::Path) -> ScanController {
    store_true_calibration(dir);
    let mut c = ScanController::new();
    c.set_rig_dir(dir);
    run(
        &mut c,
        "scan.connect",
        json!({"backend": "sim", "options": {"realtime": false, "width": WIDTH, "height": HEIGHT}}),
    );
    let r = run(
        &mut c,
        "turntable.connect",
        json!({"backend": "sim", "options": {"fast": true}}),
    );
    assert_eq!(r["calibrated"], true, "{r}");
    c
}

/// One pass of ten stops 10 degrees apart, no finish.
fn auto_params() -> Value {
    json!({"mode": "stop-and-capture", "tilts_deg": [0], "rotation_deg": 90, "step_deg": 10,
           "frames_per_stop": 1, "deg_per_s": 90, "settle_s": 0, "finish": false})
}

#[test]
fn a_link_drop_mid_scan_reconnects_and_resumes_at_the_same_stop() {
    let dir = tempfile::tempdir().unwrap();
    let mut c = rig(dir.path());
    let start = run(&mut c, "turntable.status", Value::Null)["rotate_deg"]
        .as_f64()
        .unwrap() as f32;
    // moves: the tilt, then one per stop; the link drops on the move to
    // the fourth stop, which still reaches the table
    c.sim_rig().inject_link_drop(5, true);
    run(&mut c, "scan.auto", auto_params());
    let job = wait_auto(&mut c);
    assert_eq!(job["state"], "done", "{job}");
    assert_eq!(job["reconnects"], 1, "{job}");
    assert_eq!(job["reconnecting"], false, "{job}");
    assert_eq!(job["quality"]["stops"], 10, "{job}");
    let status = run(&mut c, "turntable.status", Value::Null);
    assert_eq!(status["connected"], true, "{status}");
    run(&mut c, "scan.stop", Value::Null);
    // every stop once, 10 degrees apart: none lost, none taken twice, the
    // interrupted move not sent again on top of itself
    let stops = c.session().kept_stops().expect("kept stops");
    let angles: Vec<f32> = stops.stops.iter().map(|s| s.angles.0 - start).collect();
    assert_eq!(angles.len(), 10, "{angles:?}");
    for (i, a) in angles.iter().enumerate() {
        assert!(
            (a - 10.0 * i as f32).abs() < 0.1,
            "stop {i} at {a}: {angles:?}"
        );
    }
    run(&mut c, "turntable.disconnect", Value::Null);
}

#[test]
fn a_link_that_does_not_come_back_fails_the_scan_but_keeps_its_stops() {
    let dir = tempfile::tempdir().unwrap();
    let mut c = rig(dir.path());
    c.sim_rig().inject_link_drop(5, false);
    run(&mut c, "scan.auto", auto_params());
    let job = wait_auto(&mut c);
    assert_eq!(job["state"], "failed", "{job}");
    assert_eq!(job["error"], "ble-disconnected", "{job}");
    let detail = job["error_detail"].as_str().unwrap();
    assert!(detail.contains("reconnecting failed"), "{detail}");
    assert_eq!(job["reconnects"], 0, "{job}");
    // the table shows the dropped link
    let table = run(&mut c, "turntable.status", Value::Null);
    assert_eq!(table["connected"], false, "{table}");
    assert_eq!(table["info"]["id"], "sim-turntable", "{table}");
    // the four stops taken before the drop are kept: the scan stopped
    let status = run(&mut c, "scan.status", Value::Null);
    assert_eq!(status["state"], "stopped", "{status}");
    assert_eq!(status["auto"]["kept_stops"], 4, "{status}");
    assert!(status["editor"]["points"].as_u64().unwrap() > 0, "{status}");
    let path = dir.path().join("partial-stops");
    let out = run(
        &mut c,
        "scan.export",
        json!({"what": "stops", "path": path.to_string_lossy()}),
    );
    assert_eq!(out["stops"], 4, "{out}");
    assert!(path.is_dir());
    let out = run(
        &mut c,
        "scan.reconstruct",
        json!({"resolution_mm": 0.5, "refine": false}),
    );
    assert!(out["faces"].as_u64().unwrap() > 0, "{out}");
    assert_eq!(out["reconstruction"]["stops"], 4, "{out}");
}

#[test]
fn a_cancelled_stop_and_capture_scan_keeps_its_stops() {
    let dir = tempfile::tempdir().unwrap();
    let mut c = rig(dir.path());
    // cancelled (turntable.stop) once it took its first stops
    run(&mut c, "scan.auto", auto_params());
    let deadline = Instant::now() + Duration::from_secs(120);
    while run(&mut c, "scan.status", Value::Null)["auto"]["quality"]["stops"]
        .as_u64()
        .unwrap_or(0)
        < 2
    {
        assert!(Instant::now() < deadline);
        std::thread::sleep(Duration::from_millis(5));
    }
    run(&mut c, "turntable.stop", Value::Null);
    let status = run(&mut c, "scan.status", Value::Null);
    assert_eq!(status["auto"]["state"], "cancelled", "{status}");
    assert_eq!(status["state"], "stopped", "{status}");
    let kept = status["auto"]["kept_stops"].as_u64().unwrap();
    assert!(kept >= 2, "{status}");
    let path = dir.path().join("cancelled-stops");
    let out = run(
        &mut c,
        "scan.export",
        json!({"what": "stops", "path": path.to_string_lossy()}),
    );
    assert_eq!(out["stops"].as_u64(), Some(kept), "{out}");
}
