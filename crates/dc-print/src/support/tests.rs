//! Supports on the synthetic aligner shell.

use std::collections::BTreeMap;

use dc_mesh::TriMesh;
use glam::Vec3;

use super::*;
use crate::orient::{OrientParams, orient_with, oriented_mesh};
use crate::testutil::{Side, SyntheticShell, default_shell};

/// Closed (each directed edge once, its twin once, so every edge is shared
/// by two faces), outward (positive volume), no degenerate face.
pub(super) fn assert_closed_outward(mesh: &TriMesh) {
    let mut count: BTreeMap<(u32, u32), u32> = BTreeMap::new();
    for t in &mesh.tris {
        for e in [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
            *count.entry(e).or_default() += 1;
        }
    }
    for (&(a, b), &n) in &count {
        assert_eq!(n, 1, "edge {a}->{b} used {n} times");
        assert_eq!(count.get(&(b, a)), Some(&1), "edge {a}->{b} has no twin");
    }
    let volume: f64 = (0..mesh.face_count())
        .map(|f| {
            let [a, b, c] = mesh.triangle(f);
            f64::from(a.dot(b.cross(c))) / 6.0
        })
        .sum();
    assert!(volume > 0.0, "inward or empty: {volume}");
    assert!(
        (0..mesh.face_count()).all(|f| mesh.face_area(f) > 1e-7),
        "degenerate face"
    );
}

/// The shell oriented and lifted as the pipeline does it, its fitting
/// flags and its supports.
fn supported(shell: &SyntheticShell) -> (TriMesh, Vec<bool>, SupportSet) {
    let fitting = shell.fitting_truth();
    let o = orient_with(&shell.mesh, &fitting, &OrientParams::default());
    let mesh = oriented_mesh(&shell.mesh, &o);
    let set = supports(&mesh, &fitting, &SupportParams::default());
    (mesh, fitting, set)
}

#[test]
fn no_tip_touches_the_fitting_side() {
    let shell = default_shell();
    let (mesh, fitting, set) = supported(&shell);
    assert!(set.struts.len() >= 20, "{} struts", set.struts.len());
    check_no_fitting_contact(&mesh, &fitting, &set).unwrap();
    // Every tip sits on an outer face, the first surface above the plate
    // there, and its foot stands on the raft below it.
    let bvh = Bvh::new(&mesh);
    for s in &set.struts {
        let hit = bvh
            .cast(s.tip.truncate().extend(-0.01), Vec3::Z, s.tip.z + 0.02)
            .unwrap();
        assert_eq!(
            shell.side[hit.face as usize],
            Side::Outer,
            "tip {:?}",
            s.tip
        );
        assert!((hit.t - 0.01 - s.tip.z).abs() < 1e-3);
        assert_eq!(s.foot, Vec3::new(s.tip.x, s.tip.y, 1.0));
    }
}

#[test]
fn every_island_is_supported() {
    let (mesh, _, set) = supported(&default_shell());
    check_islands(&mesh, &set, 50).unwrap();
    check_islands(&mesh, &set, 100).unwrap();
}

#[test]
fn support_solids_are_closed_and_stand_on_the_raft() {
    let (_, _, set) = supported(&default_shell());
    assert_eq!(set.solids.len(), set.struts.len());
    for s in &set.solids {
        assert_closed_outward(s);
    }
    assert_closed_outward(&set.raft);
    let raft = set.raft.aabb().unwrap();
    assert_eq!((raft.min.z, raft.max.z), (0.0, 1.0));
    for s in &set.struts {
        let foot = s.foot.truncate();
        assert!(foot.cmpge(raft.min.truncate()).all() && foot.cmple(raft.max.truncate()).all());
    }
}

#[test]
fn supports_do_not_depend_on_threads_or_runs() {
    let shell = default_shell();
    let (mesh, _, a) = supported(&shell);
    let one = rayon::ThreadPoolBuilder::new()
        .num_threads(1)
        .build()
        .unwrap();
    let (mesh1, _, b) = one.install(|| supported(&shell));
    assert_eq!(mesh, mesh1);
    assert_eq!(a.struts, b.struts);
    assert_eq!(a.raft, b.raft);
    assert_eq!(a.solids, b.solids);
}

#[test]
fn the_checks_catch_bad_supports() {
    let (mesh, fitting, mut set) = supported(&default_shell());
    // Without struts, the lowest cusp starts in mid-air.
    let bare = SupportSet {
        struts: Vec::new(),
        raft: set.raft.clone(),
        solids: Vec::new(),
    };
    let err = check_islands(&mesh, &bare, 50).unwrap_err();
    assert_eq!(err.code(), "print-island-unsupported");
    // A tip moved onto the intaglio.
    let f = fitting.iter().position(|&x| x).unwrap();
    let [a, b, c] = mesh.triangle(f);
    set.struts[3].tip = (a + b + c) / 3.0;
    let err = check_no_fitting_contact(&mesh, &fitting, &set).unwrap_err();
    assert_eq!(err.code(), "print-fitting-contact");
    assert!(
        matches!(err, SupportError::FittingContact { strut: 3, .. }),
        "{err}"
    );
}

#[test]
fn computed_fitting_flags_work_as_well() {
    let shell = default_shell();
    let part = shell.part("upper-02", false);
    let fitting = crate::orient::fitting_faces(&part.mesh);
    let o = crate::orient::orient(&part, &OrientParams::default());
    let mesh = oriented_mesh(&part.mesh, &o);
    let set = supports(&mesh, &fitting, &SupportParams::default());
    check_no_fitting_contact(&mesh, &shell.fitting_truth(), &set).unwrap();
    check_islands(&mesh, &set, 50).unwrap();
}

#[test]
fn an_empty_part_has_no_supports() {
    let set = supports(&TriMesh::default(), &[], &SupportParams::default());
    assert!(set.struts.is_empty() && set.solids.is_empty() && set.raft.is_empty());
    check_islands(&TriMesh::default(), &set, 50).unwrap();
}

/// A flat-bottomed cup (big flat faces): the search must not tilt its
/// bottom exactly to the self-supporting limit, where rounding would leave
/// the whole bottom unsupported.
#[test]
fn a_flat_bottomed_cup_is_supported_all_over() {
    let (cup, fitting) = crate::job::tests::cup(Vec3::new(20.0, 12.0, 8.0), 1.0);
    let o = orient_with(&cup, &fitting, &OrientParams::default());
    let mesh = oriented_mesh(&cup, &o);
    let set = supports(&mesh, &fitting, &SupportParams::default());
    // About one contact per spacing² of the 20 × 12 mm bottom.
    assert!(set.struts.len() >= 15, "{} struts", set.struts.len());
    check_no_fitting_contact(&mesh, &fitting, &set).unwrap();
    check_islands(&mesh, &set, 50).unwrap();
}

/// Closed box between `lo` and `hi`, outward.
fn cuboid(lo: Vec3, hi: Vec3) -> TriMesh {
    let p = |x: bool, y: bool, z: bool| {
        Vec3::new(
            if x { hi.x } else { lo.x },
            if y { hi.y } else { lo.y },
            if z { hi.z } else { lo.z },
        )
    };
    let positions = vec![
        p(false, false, false),
        p(true, false, false),
        p(true, true, false),
        p(false, true, false),
        p(false, false, true),
        p(true, false, true),
        p(true, true, true),
        p(false, true, true),
    ];
    let tris = vec![
        [0, 2, 1],
        [0, 3, 2],
        [4, 5, 6],
        [4, 6, 7],
        [0, 1, 5],
        [0, 5, 4],
        [1, 2, 6],
        [1, 6, 5],
        [2, 3, 7],
        [2, 7, 6],
        [3, 0, 4],
        [3, 4, 7],
    ];
    TriMesh::new(positions, tris)
}

/// The island rounds hold an island the first pass left alone: a floating
/// block with no strut yet gets one where it starts.
#[test]
fn the_island_rounds_hold_what_is_left() {
    let part = cuboid(Vec3::new(-2.0, -1.0, 6.0), Vec3::new(2.0, 1.0, 7.0));
    let fitting = vec![false; part.face_count()];
    let params = SupportParams::default();
    let (bvh, fit) = (Bvh::new(&part), Bvh::with_faces(&part, []));
    let placer = Placer {
        mesh: &part,
        fitting: &fitting,
        bvh: &bvh,
        fit: &fit,
        raft_top: params.raft_mm,
        tip_r: 0.5 * params.tip_d_mm,
        shaft_r: 0.5 * params.shaft_d_mm,
    };
    let mut tips = Vec::new();
    let bare = support_set(&tips, &params, &placer);
    assert_eq!(
        check_islands(&part, &bare, params.layer_um).map_err(|e| e.code()),
        Err("print-island-unsupported")
    );
    let minima = islands::material_minima(&part);
    let set = hold_islands(&placer, &minima, &params, &mut tips);
    assert!(!set.struts.is_empty());
    check_islands(&part, &set, params.layer_um).unwrap();
    check_no_fitting_contact(&part, &fitting, &set).unwrap();
}

/// An island that starts on the fitting side never gets a strut: the hard
/// check names it (the orientation must avoid it).
#[test]
fn an_island_on_the_fitting_side_gets_no_strut() {
    let part = cuboid(Vec3::new(-2.0, -1.0, 6.0), Vec3::new(2.0, 1.0, 7.0));
    let fitting = vec![true; part.face_count()];
    let set = supports(&part, &fitting, &SupportParams::default());
    assert!(set.struts.is_empty());
    let err = check_islands(&part, &set, 50).unwrap_err();
    assert_eq!(err.code(), "print-island-unsupported");
}
