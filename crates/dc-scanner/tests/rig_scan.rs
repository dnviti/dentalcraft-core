//! End to end on the simulated turntable rig: a fixed simulated scanner
//! and a simulated dual-axis table in a static enclosure. `turntable.calibrate`
//! finds the axes, then `scan.auto` scans the typodont from all sides; the
//! region of interest keeps the enclosure out of the scan.

use std::time::{Duration, Instant};

use dc_geom::bvh::Bvh;
use dc_geom::knn::KnnIndex;
use dc_scanner::sim::{self, SimOptions};
use dc_scanner::{ScanController, ScannerBackend};
use dc_turntable::sim::{RigPose, SimRig};
use glam::{Affine3A, Vec3};
use serde_json::{Value, json};

const WIDTH: u32 = 200;
const HEIGHT: u32 = 125;

fn run(c: &mut ScanController, id: &str, params: Value) -> Value {
    c.execute(id, &params, &|| false)
        .unwrap_or_else(|e| panic!("{id} {params}: {e} ({})", e.code()))
}

/// Polls the job status at `path` of `id` until it leaves `running`.
fn wait_job(c: &mut ScanController, id: &str, key: &str) -> Value {
    let deadline = Instant::now() + Duration::from_secs(240);
    loop {
        let s = run(c, id, Value::Null);
        let job = s[key].clone();
        if job["state"] != "running" && job["state"] != "paused" {
            return job;
        }
        assert!(Instant::now() < deadline, "{s}");
        std::thread::sleep(Duration::from_millis(20));
    }
}

fn options() -> SimOptions {
    SimOptions {
        realtime: false,
        width: WIDTH,
        height: HEIGHT,
        ..SimOptions::default()
    }
}

fn v3(a: &Value) -> Vec3 {
    let f = |i: usize| a[i].as_f64().unwrap() as f32;
    Vec3::new(f(0), f(1), f(2))
}

/// Distance of `p` from the line through `q` along `d`.
fn off_line(p: Vec3, q: Vec3, d: Vec3) -> f32 {
    let v = p - q;
    (v - v.dot(d) * d).length()
}

/// The scan world frame of `scan.status` (from the first camera frame).
fn world_of(status: &Value) -> Affine3A {
    let cols: Vec<f32> = status["world_from_camera"]
        .as_array()
        .unwrap_or_else(|| panic!("no world frame: {status}"))
        .iter()
        .map(|v| v.as_f64().unwrap() as f32)
        .collect();
    Affine3A::from_cols_slice(&cols)
}

#[test]
fn calibrated_turntable_scan_covers_the_model_and_ignores_the_enclosure() {
    let started = Instant::now();
    let dir = tempfile::tempdir().unwrap();
    let mut c = ScanController::new();
    c.set_rig_dir(dir.path());
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
    assert_eq!(r["turntable"]["id"], "sim-turntable");
    assert_eq!(r["calibrated"], false);
    assert_eq!(
        run(&mut c, "turntable.calibration", Value::Null),
        Value::Null
    );

    // the enclosure fills a good part of the view: without the region of
    // interest it would drive the tracking
    let mesh = sim::typodont_arch(true);
    let bounds = mesh.aabb().unwrap();
    let info = sim::SimBackend::default().list().unwrap().remove(0);
    let dev = sim::SimDevice::new(info, &mesh, options())
        .unwrap()
        .with_rig(SimRig::new())
        .unwrap();
    let frame = dev.render_rig(0, RigPose::default());
    let camera = sim::rig_camera_pose(&options());
    let k = frame.intrinsics;
    let (mut enclosure, mut valid) = (0, 0);
    for (i, d) in frame.depth_mm.iter().enumerate() {
        if *d <= 0.0 {
            continue;
        }
        valid += 1;
        let (u, v) = ((i % WIDTH as usize) as f32, (i / WIDTH as usize) as f32);
        let p = camera.transform_point3(k.ray(u, v) * *d);
        if p.z < -1.0 || p.y > sim::RIG_WALL_MM - 1.0 {
            enclosure += 1;
        }
    }
    assert!(
        enclosure * 4 > valid,
        "{enclosure} of {valid} pixels on the enclosure"
    );

    // no calibration yet: the automatic scan is refused
    let refused = c.execute("scan.auto", &json!({}), &|| false).unwrap_err();
    assert_eq!(refused.code(), "rig-uncalibrated");

    // --- calibration ------------------------------------------------------
    let r = run(&mut c, "turntable.calibrate", json!({}));
    assert_eq!(r["started"], true);
    let job = wait_job(&mut c, "turntable.status", "calibration_job");
    assert_eq!(job["state"], "done", "{job}");
    let status = run(&mut c, "turntable.status", Value::Null);
    assert_eq!(status["calibrated"], true, "{status}");
    assert_eq!(status["busy"], false, "{status}");
    let cal = run(&mut c, "turntable.calibration", Value::Null);
    assert!(dir.path().join("calibration.json").is_file());
    assert_eq!(cal["turntable_id"], "sim-turntable");
    assert_eq!(cal["scanner_serial"], "SIM-U-0001");
    // truth: scene +Z through the origin (platter top), tilt +X at z = -10
    let to_cam = camera.inverse();
    let axis_dir = to_cam.transform_vector3(Vec3::Z);
    let dir_err = v3(&cal["axis_dir"]).angle_between(axis_dir).to_degrees();
    assert!(dir_err < 0.5, "axis direction off by {dir_err} deg: {cal}");
    let origin = to_cam.transform_point3(Vec3::ZERO);
    let point_err = off_line(origin, v3(&cal["axis_point"]), v3(&cal["axis_dir"]));
    assert!(point_err < 0.5, "axis off by {point_err} mm: {cal}");
    let plane_err = (v3(&cal["axis_point"]) - origin).dot(axis_dir).abs();
    assert!(plane_err < 0.5, "platter off by {plane_err} mm: {cal}");
    let tilt_dir = to_cam.transform_vector3(Vec3::X);
    let tilt_err = v3(&cal["tilt_axis_dir"])
        .angle_between(tilt_dir)
        .to_degrees();
    assert!(tilt_err < 1.0, "tilt axis off by {tilt_err} deg: {cal}");
    let pivot = to_cam.transform_point3(Vec3::new(0.0, 0.0, -sim::RIG_TILT_PIVOT_MM));
    let pivot_err = off_line(
        pivot,
        v3(&cal["tilt_axis_point"]),
        v3(&cal["tilt_axis_dir"]),
    );
    assert!(pivot_err < 1.0, "tilt axis off by {pivot_err} mm: {cal}");
    println!("calibrated in {:?}: {cal}", started.elapsed());

    // --- automatic scan (stop-and-capture by default) ----------------------
    let r = run(
        &mut c,
        "scan.auto",
        json!({"tilts_deg": [-15, 15], "rotation_deg": 360, "deg_per_s": 90, "settle_s": 0}),
    );
    // per tilt: the tilt, then 37 stops every 10 degrees
    assert_eq!(r["steps"], 76);
    let job = wait_job(&mut c, "scan.status", "auto");
    assert_eq!(job["state"], "done", "{job}");
    let status = run(&mut c, "scan.status", Value::Null);
    assert_eq!(status["state"], "paused", "{status}");
    assert!(status["tracking_lost"].as_u64().unwrap() <= 3, "{status}");
    let world = status["world_angles_deg"].clone();
    let world = RigPose {
        rotate_deg: world[0].as_f64().unwrap() as f32,
        tilt_deg: world[1].as_f64().unwrap() as f32,
    };
    assert!((world.tilt_deg + 15.0).abs() < 1e-3, "{world:?}");
    run(&mut c, "scan.stop", Value::Null);
    let cloud = c.editor().unwrap().cloud().clone();
    println!("{} points after {:?}", cloud.len(), started.elapsed());

    // the world is the rig frame: +Z the axis of the platter as tilted at
    // the first stop, origin on the platter, the scanner on the +X side
    assert_eq!(status["rig_world"], true, "{status}");
    let to_world = world_of(&status);
    let scene_to_world = to_world * camera.inverse();
    let platter = sim::rig_platter(world);
    let up = scene_to_world.transform_vector3(platter.transform_vector3(Vec3::Z));
    let up_err = up.angle_between(Vec3::Z).to_degrees();
    assert!(up_err < 1.0, "platter normal {up_err} deg off +Z");
    let origin = scene_to_world.transform_point3(platter.transform_point3(Vec3::ZERO));
    assert!(origin.length() < 1.0, "platter centre at {origin}");
    // the camera looks down at 60 degrees, the platter tilted by -15 about
    // the scene X axis (across the line of sight): 45 degrees to it
    let [tilt, yaw] = [0, 1].map(|i| status["scanner_deg"][i].as_f64().unwrap() as f32);
    assert!((tilt - 45.0).abs() < 1.0 && yaw.abs() < 1.0, "{status}");
    let eye = to_world.transform_point3(Vec3::ZERO);
    assert!(
        eye.x > 0.0 && eye.y.abs() < 1.0 && eye.z > 0.0,
        "scanner at {eye}"
    );

    // back to model coordinates
    let world_to_model: Affine3A =
        sim::rig_model_to_scene(&bounds, world).inverse() * camera * to_world.inverse();
    let in_model: Vec<Vec3> = cloud
        .points
        .iter()
        .map(|p| world_to_model.transform_point3(*p))
        .collect();
    let bvh = Bvh::build(&mesh).unwrap();
    let d: Vec<f32> = bvh
        .closest_points(&in_model, f32::INFINITY)
        .into_iter()
        .map(|h| h.unwrap().distance)
        .collect();
    let mean = d.iter().sum::<f32>() / d.len() as f32;
    let far = d.iter().filter(|d| **d > 1.0).count();
    assert!(mean < 0.15, "mean {mean} mm");
    // nothing of the enclosure (25 mm under, 60 mm behind) got in
    assert!(
        far * 1000 < d.len(),
        "{far} of {} points over 1 mm away",
        d.len()
    );

    // the typodont is covered from all sides
    let index = KnnIndex::build(&in_model).unwrap();
    let covered = mesh
        .positions
        .iter()
        .filter(|v| index.nearest(**v).is_some_and(|n| n.distance <= 0.5))
        .count();
    let coverage = covered as f32 / mesh.positions.len() as f32;
    assert!(coverage > 0.8, "coverage {coverage}");
    println!(
        "mean {mean} mm, coverage {coverage}, total {:?}",
        started.elapsed()
    );

    // the same scan without the region of interest: the enclosure gets in
    run(&mut c, "scan.reset", Value::Null);
    let s = run(&mut c, "scan.settings", json!({"use_roi": false}));
    assert_eq!(s["use_roi"], false);
    run(
        &mut c,
        "scan.auto",
        json!({"tilts_deg": [0], "rotation_deg": 90, "deg_per_s": 60, "settle_s": 0}),
    );
    let job = wait_job(&mut c, "scan.status", "auto");
    assert_eq!(job["state"], "done", "{job}");
    let status = run(&mut c, "scan.status", Value::Null);
    let world = &status["world_angles_deg"];
    let world = RigPose {
        rotate_deg: world[0].as_f64().unwrap() as f32,
        tilt_deg: world[1].as_f64().unwrap() as f32,
    };
    run(&mut c, "scan.stop", Value::Null);
    let world_to_model =
        sim::rig_model_to_scene(&bounds, world).inverse() * camera * world_of(&status).inverse();
    let in_model: Vec<Vec3> = c
        .editor()
        .unwrap()
        .cloud()
        .points
        .iter()
        .map(|p| world_to_model.transform_point3(*p))
        .collect();
    let far = bvh
        .closest_points(&in_model, f32::INFINITY)
        .into_iter()
        .filter(|h| h.unwrap().distance > 1.0)
        .count();
    assert!(
        far * 10 > in_model.len(),
        "without the region of interest only {far} of {} points are off the model",
        in_model.len()
    );
    println!(
        "without ROI: {far} of {} points off the model, total {:?}",
        in_model.len(),
        started.elapsed()
    );

    run(&mut c, "turntable.disconnect", Value::Null);
    assert!(!c.sim_rig().attached());
}
