//! End to end: scan the simulated typodont, check the fused cloud against
//! the source mesh, then clean, mesh and repair it in the editor.

use std::time::{Duration, Instant};

use dc_geom::bvh::Bvh;
use dc_mesh::PointCloud;
use dc_scanner::sim::{self, SimBackend, SimOptions, UPPER_ID};
use dc_scanner::{CloudOp, MeshOp, ScanEditor, ScanSession, ScanSettings, ScanState};
use glam::{Affine3A, Vec3};

const FRAMES: u64 = 40;

fn options() -> SimOptions {
    SimOptions {
        realtime: false,
        max_frames: Some(FRAMES),
        width: 320,
        height: 200,
        period_frames: 160,
        ..SimOptions::default()
    }
}

/// Scans the upper typodont; returns the cloud and the world-to-model
/// transform (world = camera of the first frame turned Z-up).
fn scan() -> (PointCloud, Affine3A) {
    let backend = SimBackend::new(options());
    let mut session = ScanSession::new(ScanSettings::default());
    session.connect_with(&backend, UPPER_ID).unwrap();
    session.start().unwrap();
    let deadline = Instant::now() + Duration::from_secs(120);
    while session.status().frames < FRAMES {
        assert!(Instant::now() < deadline, "{:?}", session.status());
        std::thread::sleep(Duration::from_millis(10));
    }
    let preview = session.preview().expect("live preview");
    assert!(preview.generation >= 1);
    assert!(!preview.frame_points.is_empty());
    let cloud = session.stop().unwrap();
    let status = session.status();
    assert_eq!(status.state, ScanState::Stopped);
    assert!(status.error.is_none(), "{status:?}");
    assert!(status.tracking_lost <= 2, "{status:?}");
    assert!(status.fused_frames >= FRAMES - 2, "{status:?}");
    // without a rig the world is the first camera frame with +Z its up
    assert!(!status.rig_world, "{status:?}");
    let to_world = Affine3A::from_cols_array(&status.world_from_camera.unwrap());
    assert_eq!(to_world, dc_scanner::rig::camera_world());
    assert_eq!(
        to_world.transform_vector3(Vec3::NEG_Y),
        Vec3::Z,
        "camera up"
    );
    assert_eq!(status.scanner_deg, Some([0.0, 0.0]));
    let bounds = sim::typodont_arch(true).aabb().unwrap();
    (
        cloud,
        sim::camera_pose(&bounds, 0, &options()) * to_world.inverse(),
    )
}

#[test]
fn scan_clean_mesh_repair() {
    let (cloud, world_to_model) = scan();
    assert!(cloud.len() > 20_000, "{} points", cloud.len());
    assert!(
        cloud
            .normals
            .as_ref()
            .is_some_and(|n| n.len() == cloud.len())
    );

    // one-sided Hausdorff, cloud -> source surface
    let source = sim::typodont_arch(true);
    let bvh = Bvh::build(&source).unwrap();
    let in_model: Vec<Vec3> = cloud
        .points
        .iter()
        .map(|p| world_to_model.transform_point3(*p))
        .collect();
    let d: Vec<f32> = bvh
        .closest_points(&in_model, f32::INFINITY)
        .into_iter()
        .map(|h| h.unwrap().distance)
        .collect();
    let mean = d.iter().sum::<f32>() / d.len() as f32;
    let max = d.iter().copied().fold(0.0, f32::max);
    assert!(mean < 0.1, "mean {mean} mm");
    assert!(max < 0.5, "max {max} mm");

    // --- editing -------------------------------------------------------
    let model_to_world = world_to_model.inverse();
    let mut ed = ScanEditor::new(cloud);
    let n0 = ed.cloud().len();

    // injected outliers 5 mm above the base plate are removed
    let mut noisy = (**ed.cloud()).clone();
    let floaters: Vec<Vec3> = [(-3.0, 8.0), (2.0, 12.0), (4.0, 5.0)]
        .iter()
        .map(|&(x, y)| model_to_world.transform_point3(Vec3::new(x, y, 5.0)))
        .collect();
    let extra = PointCloud {
        points: floaters.clone(),
        normals: Some(vec![Vec3::Z; floaters.len()]),
        colors: None,
    };
    noisy.extend(&extra);
    let mut ed_noisy = ScanEditor::new(noisy);
    let r = ed_noisy
        .apply_cloud(&CloudOp::RemoveStatistical {
            k: 10,
            std_ratio: 2.0,
        })
        .unwrap();
    let kept = &ed_noisy.cloud().points;
    for f in &floaters {
        assert!(!kept.contains(f), "outlier {f} kept ({r})");
    }
    assert!(r["removed"].as_u64().unwrap() < n0 as u64 / 20, "{r}");

    // crop to the front half of the arch (model y > 0) with a plane
    let r = ed
        .apply_cloud(&CloudOp::DeletePlane {
            point: model_to_world.transform_point3(Vec3::ZERO).into(),
            normal: model_to_world.transform_vector3(-Vec3::Y).into(),
        })
        .unwrap();
    let front_expected = in_model.iter().filter(|p| p.y > 0.0).count();
    let front = ed.cloud().len();
    assert!(
        front.abs_diff(front_expected) <= n0 / 1000,
        "{r}, {front_expected} in front"
    );

    // a box (world axes) around the left canine region
    let c = model_to_world.transform_point3(Vec3::new(-15.0, 20.0, 5.0));
    let (min, max) = (c - Vec3::splat(3.0), c + Vec3::splat(3.0));
    let inside = |p: &Vec3| p.cmpge(min).all() && p.cmple(max).all();
    let expected = ed.cloud().points.iter().filter(|p| inside(p)).count();
    assert!(expected > 0);
    let r = ed
        .apply_cloud(&CloudOp::DeleteAabb {
            min: min.into(),
            max: max.into(),
        })
        .unwrap();
    assert_eq!(r["removed"], expected);
    assert!(!ed.cloud().points.iter().any(inside));
    let cropped = ed.cloud().len();

    // punch a hole in the palate (flat base plate inside the horseshoe)
    let palate = model_to_world.transform_point3(Vec3::new(0.0, 14.0, 0.0));
    let r = ed
        .apply_cloud(&CloudOp::DeleteSphere {
            center: palate.into(),
            radius_mm: 2.0,
        })
        .unwrap();
    assert!(r["removed"].as_u64().unwrap() > 50, "{r}");

    let r = ed.build_mesh(0.3, &|| false).unwrap();
    let mesh = ed.mesh().unwrap().clone();
    assert_eq!(r["faces"], mesh.face_count());
    assert!(mesh.face_count() > 10_000, "{r}");
    let loops = mesh.boundary_loops().len();
    assert!(loops >= 2, "{loops} boundary loops");

    let r = ed
        .apply_mesh(&MeshOp::FillHoles { max_edges: 300 })
        .unwrap();
    assert!(r["holes_filled"].as_u64().unwrap() >= 1, "{r}");
    let filled = ed.mesh().unwrap().clone();
    assert!(filled.boundary_loops().len() < loops);
    assert!(filled.face_count() > mesh.face_count());

    // undo / redo walk the mesh and the cloud history
    assert!(ed.undo());
    assert_eq!(ed.mesh().unwrap().face_count(), mesh.face_count());
    assert!(ed.redo());
    assert_eq!(ed.mesh().unwrap().face_count(), filled.face_count());
    assert!(ed.undo() && ed.undo());
    assert!(ed.mesh().is_none(), "before build_mesh");
    assert!(ed.undo());
    assert_eq!(ed.cloud().len(), cropped);

    // a cloud edit drops the stale mesh
    assert!(ed.redo() && ed.redo());
    assert!(ed.mesh().is_some());
    ed.apply_cloud(&CloudOp::Downsample { voxel_mm: 0.6 })
        .unwrap();
    assert!(ed.mesh().is_none());
    assert!(ed.cloud().len() < cropped);
}
