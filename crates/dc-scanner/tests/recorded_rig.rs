//! Offline replay of a turntable dataset recorded on the real rig (MINI 2
//! and Revopoint dual-axis table): the settled stops of a 24 x 15 degree
//! revolution go through the calibration and then through the
//! stop-and-capture fusion of `scan.auto`, and the residuals are printed.
//!
//! Ignored by default: it needs `DC_SCAN_DATA`, the directory holding the
//! dataset `b_turn_24x15` (format in its `README.md`: `frames.bin`, 800x600
//! little-endian `u16` per frame in 1/200 mm, `meta.csv` with the phase,
//! the stop and the reported angles of every frame). Run with
//! `DC_SCAN_DATA=/var/tmp/dc-scan-data cargo test --release -p dc-scanner
//! --test recorded_rig -- --ignored --nocapture`. The calibration stops
//! test reads its own dataset (see
//! `recorded_calibration_stops_measure_both_axes`); the reconstruction
//! replay reads a dataset or the stops exported by `scan.export` (see
//! `recorded_stops_reconstruct`); the check of an outdated rig calibration
//! reads two exported scans (see `recorded_scans_check_the_rig_calibration`).

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::Instant;

use dc_geom::bvh::Bvh;
use dc_geom::finish::{FinishParams, finish_mesh};
use dc_geom::tsdf::{TsdfParams, TsdfVolume};
use dc_mesh::TriMesh;
use dc_scanner::reconstruct::{KeptStops, ReconstructParams, read_stops, reconstruct, write_stops};
use dc_scanner::rig::calib::{self, CalibrationStop};
use dc_scanner::rig::{RigCalibration, rfc3339};
use dc_scanner::session::{RecordedStop, fuse_recorded_stops};
use dc_scanner::{CameraIntrinsics, MeshOp, ScanEditor, ScanSettings};

const W: usize = 800;
const H: usize = 600;

/// A settled stop: reported angles and its frames (mm, `NaN` invalid).
struct Stop {
    rotate: f32,
    tilt: f32,
    frames: Vec<Arc<Vec<f32>>>,
}

/// The settled frames of `dataset`, per stop: the "still" frames from the
/// third on (the first two still show the move, the frames arrive about
/// 0.8 s late).
fn load(dataset: &str) -> Option<Vec<Stop>> {
    let dir = PathBuf::from(std::env::var_os("DC_SCAN_DATA")?).join(dataset);
    let meta = std::fs::read_to_string(dir.join("meta.csv")).ok()?;
    let raw = std::fs::read(dir.join("frames.bin")).ok()?;
    let frame = |idx: usize| -> Arc<Vec<f32>> {
        let bytes = &raw[idx * W * H * 2..(idx + 1) * W * H * 2];
        Arc::new(
            bytes
                .as_chunks::<2>()
                .0
                .iter()
                .map(|b| {
                    let v = u16::from_le_bytes(*b);
                    let d = f32::from(v) * 0.005;
                    // the working range of the acquisition
                    if v == 0 || !(100.0..=300.0).contains(&d) {
                        f32::NAN
                    } else {
                        d
                    }
                })
                .collect(),
        )
    };
    let mut lines = meta.lines();
    let head: Vec<&str> = lines.next()?.split(',').collect();
    let col = |name: &str| head.iter().position(|h| *h == name);
    let (i_idx, i_rot, i_tilt, i_phase, i_step) = (
        col("idx")?,
        col("rep_rot_deg")?,
        col("cmd_tilt_deg")?,
        col("phase")?,
        col("step")?,
    );
    let mut stops: BTreeMap<u32, (f32, f32, Vec<usize>)> = BTreeMap::new();
    for line in lines {
        let f: Vec<&str> = line.split(',').collect();
        if f.get(i_phase) != Some(&"still") {
            continue;
        }
        let step: u32 = f[i_step].parse().ok()?;
        let e = stops.entry(step).or_insert((
            f[i_rot].parse().ok()?,
            f[i_tilt].parse().ok()?,
            Vec::new(),
        ));
        e.2.push(f[i_idx].parse().ok()?);
    }
    Some(
        stops
            .into_values()
            .map(|(rotate, tilt, idx)| {
                // DC_SCAN_SKIP: still frames dropped per stop (default 2)
                let skip: usize = std::env::var("DC_SCAN_SKIP")
                    .ok()
                    .and_then(|v| v.parse().ok())
                    .unwrap_or(2);
                let mut settled: Vec<usize> = if idx.len() > skip {
                    idx[skip..].to_vec()
                } else {
                    idx
                };
                // DC_SCAN_FRAMES: settled frames used per stop at most
                if let Some(n) = std::env::var("DC_SCAN_FRAMES")
                    .ok()
                    .and_then(|v| v.parse::<usize>().ok())
                {
                    settled.truncate(n.max(1));
                }
                Stop {
                    rotate,
                    tilt,
                    frames: settled.into_iter().map(frame).collect(),
                }
            })
            .collect(),
    )
}

#[test]
#[ignore = "needs DC_SCAN_DATA with a recorded revolution (DC_SCAN_SET, default b_turn_24x15)"]
fn recorded_revolution_calibrates_and_fuses() {
    let set = std::env::var("DC_SCAN_SET").unwrap_or_else(|_| "b_turn_24x15".into());
    let Some(stops) = load(&set) else {
        panic!("set DC_SCAN_DATA to the directory holding {set}");
    };
    // no scanner calibration file was installed: the field-of-view estimate
    // (DC_SCAN_FX overrides it, to study its effect)
    let f: f32 = std::env::var("DC_SCAN_FX")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(1190.0);
    let intr = CameraIntrinsics {
        width: W as u32,
        height: H as u32,
        fx: f,
        fy: f,
        cx: 399.5,
        cy: 299.5,
    };
    println!("{} stops", stops.len());
    let medians: Vec<CalibrationStop> = stops
        .iter()
        .map(|s| CalibrationStop {
            rotate_deg: s.rotate,
            tilt_deg: s.tilt,
            depth: Arc::new(calib::median_depth(&s.frames)),
        })
        .collect();

    let t = Instant::now();
    let outcome = calib::measure(&intr, &medians, &[]);
    let (outcome, report) = match outcome {
        Ok(o) => {
            let r = o.report.clone();
            (Some(o), r)
        }
        Err((e, r)) => {
            println!("calibration refused: {e}");
            (None, r)
        }
    };
    println!(
        "calibration in {:?}: {}",
        t.elapsed(),
        serde_json::to_string(&report).unwrap()
    );
    let outcome = outcome.expect("the recorded revolution calibrates");
    println!(
        "axis point {:?} dir {:?} up {:?}",
        outcome.platter_point, outcome.axis.dir, outcome.up
    );
    let c = RigCalibration {
        axis_point: outcome.platter_point.into(),
        axis_dir: outcome.axis.dir.into(),
        platter_up: outcome.up.into(),
        tilt_axis_point: None,
        tilt_axis_dir: None,
        scanner_serial: None,
        turntable_id: "recorded".into(),
        created: rfc3339(std::time::SystemTime::now()),
        fit_rms_deg: outcome.axis.rms_deg,
        fit_rms_mm: outcome.axis.rms_mm,
        angle_gain: outcome.axis.angle_gain,
        fit_pairs: outcome.axis.pairs as u32,
        imu_up: None,
    };

    let t = Instant::now();
    // DC_SCAN_STOPS: comma-separated stop indices to fuse (default all)
    let only: Option<Vec<usize>> = std::env::var("DC_SCAN_STOPS")
        .ok()
        .map(|v| v.split(',').filter_map(|k| k.trim().parse().ok()).collect());
    // DC_SCAN_BG: remove the static background of the revolution first
    let background = std::env::var_os("DC_SCAN_BG").map(|_| {
        let frames: Vec<Arc<Vec<f32>>> = medians.iter().map(|s| Arc::clone(&s.depth)).collect();
        dc_scanner::rig::Background::from_frames(&frames, calib::BACKGROUND_TOL_MM)
    });
    let recorded: Vec<RecordedStop> = medians
        .iter()
        .enumerate()
        .filter(|(k, _)| only.as_ref().is_none_or(|o| o.contains(k)))
        .map(|(_, s)| {
            let depth = match &background {
                Some(b) => Arc::new(
                    s.depth
                        .iter()
                        .enumerate()
                        .map(|(i, &d)| if b.is_static(i, d) { f32::NAN } else { d })
                        .collect(),
                ),
                None => Arc::clone(&s.depth),
            };
            ((s.rotate_deg, s.tilt_deg), depth)
        })
        .collect();
    // DC_SCAN_ROI_MM / DC_SCAN_ROI_H_MM: radius and height of the region
    // of interest (default 45 / 60 mm), DC_SCAN_NO_ROI: none
    let env = |k: &str, d: f32| {
        std::env::var(k)
            .ok()
            .and_then(|v| v.parse().ok())
            .unwrap_or(d)
    };
    let settings = ScanSettings {
        roi_radius_mm: env("DC_SCAN_ROI_MM", ScanSettings::default().roi_radius_mm),
        roi_height_mm: env("DC_SCAN_ROI_H_MM", ScanSettings::default().roi_height_mm),
        use_roi: std::env::var_os("DC_SCAN_NO_ROI").is_none(),
        ..ScanSettings::default()
    };
    let scan = fuse_recorded_stops(&settings, &c, &intr, &recorded);
    println!("static clutter removed: {} pixels", scan.clutter_pixels);
    // DC_SCAN_PLY: where to write the fused cloud
    if let Some(path) = std::env::var_os("DC_SCAN_PLY") {
        dc_mesh::io::write_ply(&scan.cloud, std::path::Path::new(&path)).unwrap();
    }
    let fused = scan.stops.iter().filter(|s| s.fused).count();
    let flagged = scan.stops.iter().filter(|s| s.flagged).count();
    let rms: Vec<f32> = scan.stops.iter().filter_map(|s| s.rms_mm).collect();
    let mean_rms = rms.iter().sum::<f32>() / rms.len().max(1) as f32;
    let max_delta = scan
        .stops
        .iter()
        .map(|s| s.delta_deg.abs())
        .fold(0.0, f32::max);
    let bounds = scan.cloud.aabb();
    println!(
        "fusion in {:?}: {fused}/{} stops fused, {flagged} flagged, mean rms {mean_rms:.3} mm, \
         max |dtheta| {max_delta:.2} deg, loop {:?} deg (applied {}), {} points, bounds {bounds:?}",
        t.elapsed(),
        scan.stops.len(),
        scan.loop_error_deg,
        scan.loop_applied,
        scan.cloud.len()
    );
    for (k, s) in scan.stops.iter().enumerate() {
        println!(
            "  stop {k:2}: fused {} flagged {} dtheta {:+.3} rms {:?}",
            s.fused, s.flagged, s.delta_deg, s.rms_mm
        );
    }
    // the world is the rig frame: the platter is the XY plane, the object
    // stands along +Z, the scanner on the +X side looking down at it
    let f = scan.world.transform_vector3(glam::Vec3::Z);
    let eye = scan.world.transform_point3(glam::Vec3::ZERO);
    let (tilt, yaw) = ((-f.z).asin().to_degrees(), f.y.atan2(-f.x).to_degrees());
    let plane: Vec<glam::Vec3> = scan
        .cloud
        .points
        .iter()
        .copied()
        .filter(|p| p.z.abs() < 1.5 && p.truncate().length() > 20.0)
        .collect();
    let normal = plane_normal(&plane);
    let normal_deg = normal.map(|n| n.dot(glam::Vec3::Z).abs().acos().to_degrees());
    println!(
        "world: scanner at {eye:?} tilt {tilt:.1} deg yaw {yaw:.1} deg; {} platter points, \
         normal {normal_deg:?} deg off +Z; object z {:?} mm",
        plane.len(),
        bounds.map(|(lo, hi)| (lo.z, hi.z)),
    );
    assert!(
        yaw.abs() < 5.0 && (0.0..80.0).contains(&tilt),
        "{tilt} {yaw}"
    );
    assert!(eye.x > 0.0 && eye.z > 0.0, "{eye:?}");
    assert_eq!(fused, scan.stops.len());
}

/// Normal of the least-squares plane of `points` (smallest principal axis).
fn plane_normal(points: &[glam::Vec3]) -> Option<glam::Vec3> {
    if points.len() < 3 {
        return None;
    }
    let c = points.iter().copied().sum::<glam::Vec3>() / points.len() as f32;
    let mut m = glam::Mat3::ZERO;
    for p in points {
        let d = *p - c;
        m += glam::Mat3::from_cols(d * d.x, d * d.y, d * d.z);
    }
    // inverse iteration converges on the smallest eigenvector
    let inv = (m + glam::Mat3::IDENTITY * 1e-6).inverse();
    let mut n = glam::Vec3::ONE.normalize();
    for _ in 0..50 {
        n = (inv * n).try_normalize()?;
    }
    Some(n)
}

/// `depth` (pinhole `f`, centre of the image) as seen through a camera
/// with radial distortion `k1` (normalised radius) and depth `z' = z +
/// offset`: resampled into the ideal pinhole image, nearest pixel.
fn remodel(depth: &[f32], f: f32, k1: f32, offset: f32) -> Vec<f32> {
    let (cx, cy) = ((W as f32 - 1.0) * 0.5, (H as f32 - 1.0) * 0.5);
    let mut out = vec![f32::NAN; W * H];
    for v in 0..H {
        for u in 0..W {
            let (x, y) = ((u as f32 - cx) / f, (v as f32 - cy) / f);
            let s = 1.0 + k1 * (x * x + y * y);
            let (ud, vd) = ((cx + x * s * f).round(), (cy + y * s * f).round());
            if ud < 0.0 || vd < 0.0 || ud >= W as f32 || vd >= H as f32 {
                continue;
            }
            let d = depth[vd as usize * W + ud as usize];
            if d.is_finite() {
                out[v * W + u] = d + offset;
            }
        }
    }
    out
}

/// How consistent the recorded revolution `DC_SCAN_SET` (default
/// `b_turn_24x15`) is under other camera models: the per-pair angle
/// residual and the angle gain of the calibration over a grid of focal
/// lengths, radial distortions and depth offsets. The turntable is exact,
/// so the right model gives residuals near the registration noise and a
/// gain of 1.
#[test]
#[ignore = "needs DC_SCAN_DATA with a recorded revolution; slow"]
fn recorded_revolution_under_other_camera_models() {
    let set = std::env::var("DC_SCAN_SET").unwrap_or_else(|_| "b_turn_24x15".into());
    let Some(stops) = load(&set) else {
        panic!("set DC_SCAN_DATA to the directory holding {set}");
    };
    let medians: Vec<(f32, Vec<f32>)> = stops
        .iter()
        .map(|s| (s.rotate, calib::median_depth(&s.frames)))
        .collect();
    // the stops fused at the exact turntable poses: their residual against
    // the model is the consistency of the camera model (and of the axis)
    let consistency =
        |intr: &CameraIntrinsics, o: &calib::CalibrationOutcome, model: &[CalibrationStop]| {
            let c = RigCalibration {
                axis_point: o.platter_point.into(),
                axis_dir: o.axis.dir.into(),
                platter_up: o.up.into(),
                tilt_axis_point: None,
                tilt_axis_dir: None,
                scanner_serial: None,
                turntable_id: "recorded".into(),
                created: String::new(),
                fit_rms_deg: o.axis.rms_deg,
                fit_rms_mm: o.axis.rms_mm,
                angle_gain: o.angle_gain,
                fit_pairs: o.axis.pairs as u32,
                imu_up: None,
            };
            let stops: Vec<RecordedStop> = model
                .iter()
                .map(|s| ((s.rotate_deg, 0.0), Arc::clone(&s.depth)))
                .collect();
            let scan = fuse_recorded_stops(&ScanSettings::default(), &c, intr, &stops);
            let at_prior: Vec<f32> = scan
                .stops
                .iter()
                .filter(|s| s.flagged)
                .filter_map(|s| s.rms_mm)
                .collect();
            at_prior.iter().sum::<f32>() / at_prior.len().max(1) as f32
        };
    let grid: Vec<(f32, f32, f32)> = match std::env::var("DC_SCAN_GRID").as_deref() {
        Ok("coarse") => [1100.0, 1190.0, 1300.0]
            .into_iter()
            .flat_map(|f| [-0.15, 0.0, 0.15].map(move |k| (f, k)))
            .flat_map(|(f, k)| [-4.0, 0.0, 4.0].map(move |o| (f, k, o)))
            .collect(),
        _ => [1100.0, 1190.0, 1300.0]
            .into_iter()
            .flat_map(|f| [-0.3, -0.15, 0.0, 0.15, 0.3].map(move |k| (f, k)))
            .flat_map(|(f, k)| [-8.0, -4.0, 0.0, 4.0, 8.0].map(move |o| (f, k, o)))
            .collect(),
    };
    for (f, k1, offset) in grid {
        let intr = CameraIntrinsics {
            width: W as u32,
            height: H as u32,
            fx: f,
            fy: f,
            cx: (W as f32 - 1.0) * 0.5,
            cy: (H as f32 - 1.0) * 0.5,
        };
        let model: Vec<CalibrationStop> = medians
            .iter()
            .map(|(a, d)| CalibrationStop {
                rotate_deg: *a,
                tilt_deg: 0.0,
                depth: Arc::new(remodel(d, f, k1, offset)),
            })
            .collect();
        let (r, prior_rms) = match calib::measure(&intr, &model, &[]) {
            Ok(o) => {
                let p = consistency(&intr, &o, &model);
                (o.report, Some(p))
            }
            Err((_, r)) => (r, None),
        };
        println!(
            "f {f:6.1} k1 {k1:+.2} offset {offset:+.1}: rms {:?} deg {:?} mm, gain {:?}, pairs \
             {}, at turntable poses {prior_rms:?} mm",
            r.rms_deg, r.rms_mm, r.angle_gain, r.pairs
        );
    }
}

/// The stops of a calibration recorded on the rig, as the job takes them
/// (`DC_SCAN_DATA/<DC_CAL_SET>`, default `g_calib_m125`: `stops.json` with
/// the intrinsics and, per stop, the angles and the file of its median
/// depth image, 800x600 little-endian `f32` mm, `NaN` invalid), measure
/// both axes of the table. `g_calib_m125` started at -125.2 degrees: the
/// stop at tilt -15 shows a side of the test piece that did not register
/// from the shallow tilt pivots tried before (2026-11-19, the calibration
/// was refused with `calibration-tracking`).
#[test]
#[ignore = "needs DC_SCAN_DATA with recorded calibration stops (DC_CAL_SET, default g_calib_m125)"]
fn recorded_calibration_stops_measure_both_axes() {
    let set = std::env::var("DC_CAL_SET").unwrap_or_else(|_| "g_calib_m125".into());
    let dir = PathBuf::from(std::env::var_os("DC_SCAN_DATA").expect("set DC_SCAN_DATA")).join(&set);
    let meta: serde_json::Value =
        serde_json::from_slice(&std::fs::read(dir.join("stops.json")).expect("stops.json"))
            .expect("stops.json is JSON");
    let i = &meta["intrinsics"];
    let num = |v: &serde_json::Value| v.as_f64().expect("a number") as f32;
    let intr = CameraIntrinsics {
        width: W as u32,
        height: H as u32,
        fx: num(&i["fx"]),
        fy: num(&i["fy"]),
        cx: num(&i["cx"]),
        cy: num(&i["cy"]),
    };
    let stops = |key: &str| -> Vec<CalibrationStop> {
        meta[key]
            .as_array()
            .expect("a list of stops")
            .iter()
            .map(|s| {
                let bytes = std::fs::read(dir.join(s["file"].as_str().expect("a file")))
                    .expect("a stop image");
                CalibrationStop {
                    rotate_deg: num(&s["rotate_deg"]),
                    tilt_deg: num(&s["tilt_deg"]),
                    depth: Arc::new(
                        bytes
                            .as_chunks::<4>()
                            .0
                            .iter()
                            .map(|b| f32::from_le_bytes(*b))
                            .collect(),
                    ),
                }
            })
            .collect()
    };
    let (rotation, tilt) = (stops("rotation"), stops("tilt"));
    assert_eq!(rotation.len(), 25);
    assert_eq!(tilt.len(), 2);
    let o = calib::measure(&intr, &rotation, &tilt).unwrap_or_else(|(e, r)| panic!("{e}: {r:?}"));
    println!("{set}: {:?}", o.report);
    let t = o.tilt.expect("the tilt axis");
    assert!(
        t.rms_deg <= calib::MAX_RMS_DEG && t.rms_mm <= calib::MAX_RMS_MM,
        "{t:?}"
    );
    // the tilt axis lies along the platter, under it
    assert!(t.dir.dot(o.up).abs() < 0.1, "{t:?}");
    assert!((t.point - o.platter_point).dot(o.up) < -20.0, "{t:?}");
}

/// The MINI 2 factory calibration (rectified left pinhole of the depth
/// image), read from the device on 2026-11-18.
fn factory_intrinsics() -> CameraIntrinsics {
    CameraIntrinsics {
        width: W as u32,
        height: H as u32,
        fx: 1142.3702,
        fy: 1142.3702,
        cx: 163.18774,
        cy: 329.85217,
    }
}

fn env_f32(key: &str, default: f32) -> f32 {
    std::env::var(key)
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(default)
}

/// The rotation and tilt stops of a dataset in `DC_SCAN_DATA`: a frames
/// dataset (`frames.bin`, `meta.csv`: the per-pixel median of the settled
/// frames of every stop) or a stops dataset (`stops.json` with `rotation`
/// and `tilt` lists of median images, like `g_calib_m125`), with its
/// intrinsics (`DC_SCAN_INTR=fov` for the field-of-view estimate the old
/// recordings used, the factory calibration otherwise).
fn dataset_stops(set: &str) -> (CameraIntrinsics, Vec<CalibrationStop>, Vec<CalibrationStop>) {
    let dir = PathBuf::from(std::env::var_os("DC_SCAN_DATA").expect("set DC_SCAN_DATA")).join(set);
    let fov = std::env::var("DC_SCAN_INTR").is_ok_and(|v| v == "fov");
    let intr = if fov {
        CameraIntrinsics {
            width: W as u32,
            height: H as u32,
            fx: 1190.0,
            fy: 1190.0,
            cx: 399.5,
            cy: 299.5,
        }
    } else {
        factory_intrinsics()
    };
    if dir.join("stops.json").is_file() {
        let meta: serde_json::Value =
            serde_json::from_slice(&std::fs::read(dir.join("stops.json")).unwrap()).unwrap();
        let num = |v: &serde_json::Value| v.as_f64().expect("a number") as f32;
        let stops = |key: &str| -> Vec<CalibrationStop> {
            meta[key]
                .as_array()
                .map(Vec::as_slice)
                .unwrap_or_default()
                .iter()
                .map(|s| {
                    let bytes = std::fs::read(dir.join(s["file"].as_str().unwrap())).unwrap();
                    CalibrationStop {
                        rotate_deg: num(&s["rotate_deg"]),
                        tilt_deg: num(&s["tilt_deg"]),
                        depth: Arc::new(
                            bytes
                                .as_chunks::<4>()
                                .0
                                .iter()
                                .map(|b| f32::from_le_bytes(*b))
                                .collect(),
                        ),
                    }
                })
                .collect()
        };
        return (intr, stops("rotation"), stops("tilt"));
    }
    let stops = load(set).expect("a frames dataset");
    let (mut rotation, mut tilt) = (Vec::new(), Vec::new());
    let first_tilt = stops.first().map_or(0.0, |s| s.tilt);
    for s in stops {
        let stop = CalibrationStop {
            rotate_deg: s.rotate,
            tilt_deg: s.tilt,
            depth: Arc::new(calib::median_depth(&s.frames)),
        };
        if (s.tilt - first_tilt).abs() < 0.05 {
            rotation.push(stop);
        } else {
            tilt.push(stop);
        }
    }
    (intr, rotation, tilt)
}

/// Calibrates the rig on the stops of `set` and runs them through the
/// stop-and-capture fusion of `scan.auto`: the kept stops and the live
/// residual per stop. `DC_SCAN_EVERY=k` fuses every k-th rotation stop
/// only (a coarser step; the calibration still uses them all).
fn dataset_scan(set: &str) -> KeptStops {
    let (intr, rotation, tilt) = dataset_stops(set);
    let o = calib::measure(&intr, &rotation, &tilt).unwrap_or_else(|(e, r)| panic!("{e}: {r:?}"));
    println!("calibration of {set}: {:?}", o.report);
    let c = RigCalibration {
        axis_point: o.platter_point.into(),
        axis_dir: o.axis.dir.into(),
        platter_up: o.up.into(),
        tilt_axis_point: o.tilt.as_ref().map(|t| t.point.into()),
        tilt_axis_dir: o.tilt.as_ref().map(|t| t.dir.into()),
        scanner_serial: None,
        turntable_id: "recorded".into(),
        created: rfc3339(std::time::SystemTime::now()),
        fit_rms_deg: o.axis.rms_deg,
        fit_rms_mm: o.axis.rms_mm,
        angle_gain: o.axis.angle_gain,
        fit_pairs: o.axis.pairs as u32,
        imu_up: None,
    };
    let every = std::env::var("DC_SCAN_EVERY")
        .ok()
        .and_then(|v| v.parse::<usize>().ok())
        .unwrap_or(1)
        .max(1);
    let recorded: Vec<RecordedStop> = rotation
        .iter()
        .enumerate()
        .filter(|(k, _)| k % every == 0)
        .map(|(_, s)| s)
        .chain(tilt.iter())
        .map(|s| ((s.rotate_deg, s.tilt_deg), Arc::clone(&s.depth)))
        .collect();
    let scan = fuse_recorded_stops(&ScanSettings::default(), &c, &intr, &recorded);
    let flagged = scan.stops.iter().filter(|s| s.flagged).count();
    let rms: Vec<f32> = scan.stops.iter().filter_map(|s| s.rms_mm).collect();
    println!(
        "live fusion: {} stops, {flagged} flagged, mean rms {:.3} mm, loop {:?} deg, clutter {}",
        scan.stops.len(),
        rms.iter().sum::<f32>() / rms.len().max(1) as f32,
        scan.loop_error_deg,
        scan.clutter_pixels
    );
    scan.kept
}

/// The model of the live acquisition rebuilt from the kept stops (live
/// voxel, unit weights), as the cloud `scan.stop` hands to the editor.
fn live_cloud(stops: &KeptStops) -> dc_mesh::PointCloud {
    let s = ScanSettings::default();
    let mut v = TsdfVolume::new(TsdfParams {
        voxel_mm: s.live_voxel_mm,
        trunc_mm: (4.0 * s.live_voxel_mm).max(1.0),
        max_weight: 64.0,
    });
    for k in &stops.stops {
        v.integrate(&k.depth(), &k.intrinsics, &k.pose);
    }
    v.extract_cloud().transformed(&stops.world)
}

/// Fraction of the vertices with a second sheet facing the same way
/// within `depth` mm behind them (along the inward normal): double walls
/// of misregistered views.
fn double_wall_fraction(mesh: &TriMesh, depth: f32) -> f32 {
    let Ok(bvh) = Bvh::build(mesh) else {
        return f32::NAN;
    };
    let normals = mesh.compute_vertex_normals();
    let step = (mesh.vertex_count() / 50_000).max(1);
    let picked: Vec<usize> = (0..mesh.vertex_count()).step_by(step).collect();
    let origins: Vec<glam::Vec3> = picked
        .iter()
        .map(|&v| mesh.positions[v] - normals[v] * 0.02)
        .collect();
    let dirs: Vec<glam::Vec3> = picked.iter().map(|&v| -normals[v]).collect();
    let hits = bvh.ray_casts(&origins, &dirs, depth);
    let double = hits
        .iter()
        .zip(&picked)
        .filter(|(h, v)| {
            h.is_some_and(|h| mesh.face_normal(h.face as usize).dot(normals[**v]) > 0.5)
        })
        .count();
    double as f32 / picked.len().max(1) as f32
}

/// Topology and size of `mesh` in one line.
fn describe(mesh: &TriMesh) -> String {
    let v = mesh.validate();
    format!(
        "{} faces, {} vertices, {} border loops, {} non-manifold edges, {} components, double walls {:.4}",
        v.faces,
        v.vertices,
        v.boundary_loops,
        v.non_manifold_edges,
        v.components,
        double_wall_fraction(mesh, 0.8)
    )
}

/// The stops of the passes at the tilts `DC_SCAN_TILTS` (comma separated,
/// default all), every `DC_SCAN_KEEP_EVERY`-th stop of each pass (default
/// 1): the subsets of an exported scan that a scan with fewer tilts or a
/// coarser step would have taken.
fn select_stops(mut stops: KeptStops) -> KeptStops {
    let tilts: Option<Vec<f32>> = std::env::var("DC_SCAN_TILTS")
        .ok()
        .map(|v| v.split(',').filter_map(|t| t.trim().parse().ok()).collect());
    let every = std::env::var("DC_SCAN_KEEP_EVERY")
        .ok()
        .and_then(|v| v.parse::<usize>().ok())
        .unwrap_or(1)
        .max(1);
    let same = |a: f32, b: f32| (a - b).abs() < 0.05;
    let mut seen: Vec<(f32, usize)> = Vec::new();
    stops.stops.retain(|s| {
        let t = s.angles.1;
        if tilts
            .as_ref()
            .is_some_and(|l| !l.iter().any(|x| same(*x, t)))
        {
            return false;
        }
        let k = match seen.iter_mut().find(|(x, _)| same(*x, t)) {
            Some((_, k)) => {
                *k += 1;
                *k - 1
            }
            None => {
                seen.push((t, 1));
                0
            }
        };
        k % every == 0
    });
    stops
}

/// The fine reconstruction and the finish of `scan.auto` on real stops,
/// against the old path (the live 0.3 mm model as a cloud, meshed at 0.3
/// mm, islands removed, holes filled, three Taubin passes). The stops come
/// from `DC_SCAN_STOPS` (a directory written by `scan.export` with `what:
/// "stops"`, subsets by `DC_SCAN_TILTS` and `DC_SCAN_KEEP_EVERY`, see
/// `select_stops`) or from the dataset `DC_SCAN_SET` in `DC_SCAN_DATA`
/// (default `g_calib_m125`), calibrated and fused like `scan.auto` (see
/// `dataset_scan`, `DC_SCAN_EVERY`, `DC_SCAN_FRAMES`, `DC_SCAN_INTR`).
/// `DC_SCAN_RES` (0.1), `DC_SCAN_REFINE` (1), `DC_SCAN_MINW` (1) set the
/// reconstruction; `DC_SCAN_OUT` gets `old.ply`, `raw.ply`,
/// `finished.ply` and the stops (`stops/`).
#[test]
#[ignore = "needs DC_SCAN_STOPS or DC_SCAN_DATA with a recorded scan"]
fn recorded_stops_reconstruct() {
    let (label, stops) = match std::env::var_os("DC_SCAN_STOPS") {
        Some(dir) => (
            PathBuf::from(&dir).display().to_string(),
            select_stops(read_stops(Path::new(&dir)).expect("a stops directory")),
        ),
        None => {
            let set = std::env::var("DC_SCAN_SET").unwrap_or_else(|_| "g_calib_m125".into());
            let stops = dataset_scan(&set);
            (set, stops)
        }
    };
    println!("{label}: {} stops", stops.stops.len());
    let out = std::env::var_os("DC_SCAN_OUT").map(PathBuf::from);
    if let Some(out) = &out {
        std::fs::create_dir_all(out).unwrap();
        write_stops(&out.join("stops"), &stops).unwrap();
    }

    // the old path
    let t = Instant::now();
    let mut editor = ScanEditor::new(live_cloud(&stops));
    editor.build_mesh(0.3, &|| false).unwrap();
    editor
        .apply_mesh(&MeshOp::RemoveIslands { min_fraction: 0.1 })
        .unwrap();
    let holes = editor
        .apply_mesh(&MeshOp::FillHoles { max_edges: 200 })
        .unwrap();
    editor
        .apply_mesh(&MeshOp::Smooth {
            iterations: 3,
            lambda: 0.5,
            mu: -0.53,
        })
        .unwrap();
    let old = editor.mesh().unwrap().as_ref().clone();
    println!(
        "OLD {label}: {:.1} s, {} holes filled; {}",
        t.elapsed().as_secs_f32(),
        holes["holes_filled"],
        describe(&old)
    );

    // the new path
    let params = ReconstructParams {
        resolution_mm: env_f32("DC_SCAN_RES", 0.1),
        refine: std::env::var("DC_SCAN_REFINE").map_or(true, |v| v != "0"),
        min_weight: env_f32("DC_SCAN_MINW", 1.0),
    };
    let t = Instant::now();
    let rec = reconstruct(&stops, &params, &|_| {}, &|| false).unwrap();
    let t_rec = t.elapsed().as_secs_f32();
    let t = Instant::now();
    let (finished, report) =
        finish_mesh(&rec.mesh, &FinishParams::default(), &|_| {}, &|| false).unwrap();
    let t_fin = t.elapsed().as_secs_f32();
    println!("RECONSTRUCTION {label}: {t_rec:.1} s {:?}", rec.report);
    for p in &rec.report.pass_details {
        println!("  pass {p:?}");
    }
    let rejected: Vec<String> = rec
        .rejected
        .iter()
        .enumerate()
        .filter(|(_, r)| **r)
        .map(|(i, _)| {
            let s = &stops.stops[i];
            format!("{i}@{:+.0}/{:.0}", s.angles.1, s.angles.0)
        })
        .collect();
    println!("rejected stops: {}", rejected.join(" "));
    println!("RAW: {}", describe(&rec.mesh));
    println!("FINISH {t_fin:.1} s {report:?}");
    println!("FINISHED: {}", describe(&finished));
    let rms: Vec<String> = rec.stop_rms_mm.iter().map(|r| format!("{r:.3}")).collect();
    println!("per-stop rms against the others (mm): {}", rms.join(" "));
    if let Some(out) = &out {
        let ply = dc_mesh::io::MeshFormat::Ply;
        dc_mesh::io::write(out.join("old.ply"), &old, ply).unwrap();
        dc_mesh::io::write(out.join("raw.ply"), &rec.mesh, ply).unwrap();
        dc_mesh::io::write(out.join("finished.ply"), &finished, ply).unwrap();
    }
    assert_eq!(report.non_manifold_edges, 0);
    assert_eq!(report.components, 1);
}

/// The live stop-and-capture fusion of the stops exported from a scan
/// (`scan.export` `stops`) under the calibration they were taken with: the
/// views, flags and the verdict of the calibration check.
fn replay_exported(dir: &Path) -> (usize, dc_scanner::session::ReplayedScan) {
    let kept = read_stops(dir).expect("a stops directory");
    let calibration = kept
        .calibration
        .clone()
        .expect("stops of a calibrated scan");
    let k = kept.stops[0].intrinsics;
    let stops: Vec<RecordedStop> = kept
        .stops
        .iter()
        .map(|s| {
            let mut depth = vec![f32::NAN; (k.width * k.height) as usize];
            for &(i, z) in s.pixels.iter() {
                depth[i as usize] = z;
            }
            (s.angles, Arc::new(depth))
        })
        .collect();
    let n = stops.len();
    (
        n,
        fuse_recorded_stops(&ScanSettings::default(), &calibration, &k, &stops),
    )
}

/// The calibration check of `scan.auto` on real scans: the Windows scan of
/// 2026-11-20, taken with the calibration of 2026-11-19 after the user had
/// turned and moved the turntable (`DC_SCAN_STOPS_OUTDATED`, the live scan
/// flagged 243 of 245 views and its reconstruction rejected 241), must stop
/// on its first views; the Linux scan of 2026-11-19 with the same
/// calibration on the rig as it stood (`DC_SCAN_STOPS_FITTING`, 19 of 245
/// views flagged, none rejected) must not.
#[test]
#[ignore = "needs DC_SCAN_STOPS_OUTDATED and DC_SCAN_STOPS_FITTING (exported scans)"]
fn recorded_scans_check_the_rig_calibration() {
    for (key, outdated) in [
        ("DC_SCAN_STOPS_OUTDATED", true),
        ("DC_SCAN_STOPS_FITTING", false),
    ] {
        let dir = PathBuf::from(std::env::var_os(key).unwrap_or_else(|| panic!("set {key}")));
        let (n, scan) = replay_exported(&dir);
        let flagged = scan.stops.iter().filter(|s| s.flagged).count();
        let first: Vec<String> = scan
            .stops
            .iter()
            .skip(1)
            .take(dc_scanner::rig::CHECK_VIEWS as usize)
            .map(|s| format!("{}/{:?}", u8::from(s.flagged), s.prior_fit))
            .collect();
        println!(
            "{}: {n} stops, {flagged} flagged, first views (flagged/fit) {}, outdated {}",
            dir.display(),
            first.join(" "),
            scan.calibration_outdated
        );
        assert_eq!(scan.calibration_outdated, outdated, "{}", dir.display());
    }
}
