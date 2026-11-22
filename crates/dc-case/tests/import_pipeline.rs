//! The scan import pipeline end to end. Every mesh is generated here.

use std::any::Any;
use std::path::{Path, PathBuf};

use dc_case::import::{
    Allocation, Decision, Detection, FileClaim, ImportError, ImportFile, ImportReport,
    ImportRequest, ImportWarning, Placement, PrepareOptions, Resolver, WarningCode, allocate,
    allocate_with, commit, import_files, prepare, prepare_with, remove_scan, roles_changed,
};
use dc_case::{
    BlobRef, Case, CaseState, ModuleData, ModuleState, OpaqueModuleData, Pose, RoleSource, Scan,
    ScanRole, WorldFrame, role_from_name,
};
use dc_i18n::Lang;
use dc_jobs::JobCtx;
use dc_mesh::{MeshFormat, TriMesh, Units};
use glam::{DVec3, Vec3};
use serde::Serialize;
use uuid::Uuid;

use RoleSource::{Detected, DetectedUnsure, Explicit, FileName, Provisional};
use ScanRole::{Bite, Lower, Upper};

/// Faceted square of `2 * cells * cells` faces and side `size`.
fn grid(cells: u32, size: f32) -> TriMesh {
    let n = cells + 1;
    let step = size / cells as f32;
    let mut positions = Vec::with_capacity((n * n) as usize);
    for y in 0..n {
        for x in 0..n {
            let z = ((x + y) % 2) as f32 * step * 0.1;
            positions.push(Vec3::new(x as f32 * step, y as f32 * step, z));
        }
    }
    let mut tris = Vec::with_capacity((2 * cells * cells) as usize);
    for y in 0..cells {
        for x in 0..cells {
            let i = y * n + x;
            tris.push([i, i + 1, i + n]);
            tris.push([i + 1, i + n + 1, i + n]);
        }
    }
    TriMesh::new(positions, tris)
}

/// An arch-sized model that raises no file warning.
fn arch(size: f32) -> TriMesh {
    grid(24, size)
}

struct Dir(tempfile::TempDir);

impl Dir {
    fn new() -> Self {
        Self(tempfile::tempdir().unwrap())
    }

    fn path(&self, name: &str) -> PathBuf {
        self.0.path().join(name)
    }

    fn stl(&self, name: &str, mesh: &TriMesh) -> PathBuf {
        let path = self.path(name);
        dc_mesh::io::write(&path, mesh, MeshFormat::Stl).unwrap();
        path
    }

    fn text(&self, name: &str, text: &str) -> PathBuf {
        let path = self.path(name);
        std::fs::write(&path, text).unwrap();
        path
    }
}

fn import(state: &mut CaseState, paths: &[PathBuf]) -> ImportReport {
    let request = ImportRequest::from_paths(paths.iter().cloned());
    import_files(state, &request, None, &JobCtx::detached())
}

fn import_as(state: &mut CaseState, path: PathBuf, role: ScanRole) -> ImportReport {
    let request = ImportRequest {
        files: vec![ImportFile {
            path,
            role: Some(role),
        }],
    };
    import_files(state, &request, None, &JobCtx::detached())
}

fn codes(warnings: &[ImportWarning]) -> Vec<WarningCode> {
    warnings.iter().map(|w| w.code).collect()
}

fn roles(state: &CaseState) -> Vec<(ScanRole, &str)> {
    state
        .case
        .scans
        .iter()
        .map(|s| (s.role, s.source_name.as_str()))
        .collect()
}

// --- allocate ---------------------------------------------------------------

fn old(id: u128, role: ScanRole, source: RoleSource, name: &str) -> Scan {
    Scan {
        id: Uuid::from_u128(id),
        role,
        role_source: source,
        source_name: name.into(),
        source_units: Units::Millimeters,
        import_xform: Pose::IDENTITY,
        mesh: BlobRef::of_bytes(name.as_bytes()),
    }
}

fn file(name: &str) -> FileClaim {
    FileClaim {
        name: name.into(),
        explicit: None,
        name_hint: role_from_name(name),
        detection: None,
    }
}

fn explicit(name: &str, role: ScanRole) -> FileClaim {
    FileClaim {
        explicit: Some(role),
        ..file(name)
    }
}

fn detected(name: &str, arch: ScanRole, confidence: f32) -> FileClaim {
    FileClaim {
        detection: Some(Detection { arch, confidence }),
        ..file(name)
    }
}

fn put(role: ScanRole, source: RoleSource) -> Decision {
    Decision::Place {
        role,
        source,
        replaces: Vec::new(),
    }
}

fn over(role: ScanRole, source: RoleSource, ids: &[u128]) -> Decision {
    Decision::Place {
        role,
        source,
        replaces: ids.iter().copied().map(Uuid::from_u128).collect(),
    }
}

fn needs_role(name: &str) -> Decision {
    Decision::Fail(ImportError::NeedsRole { name: name.into() })
}

fn clash(name: &str, role: ScanRole) -> Decision {
    Decision::Fail(ImportError::RoleClash {
        name: name.into(),
        role,
    })
}

#[track_caller]
fn check(a: &Allocation, decisions: &[Decision], warnings: &[WarningCode]) {
    assert_eq!(a.decisions, decisions);
    assert_eq!(codes(&a.warnings), warnings);
}

#[test]
fn rule_1_named_files_reserve_their_slots_first() {
    let a = allocate(&[], &[file("scan.stl"), file("lower.stl")]);
    check(
        &a,
        &[put(Upper, Provisional), put(Lower, FileName)],
        &[WarningCode::Provisional],
    );
    let a = allocate(&[], &[file("lower.stl"), file("scan.stl")]);
    check(
        &a,
        &[put(Lower, FileName), put(Upper, Provisional)],
        &[WarningCode::Provisional],
    );
}

#[test]
fn rule_2_an_explicit_role_overrules_name_and_detection_and_may_replace() {
    let f = FileClaim {
        explicit: Some(Upper),
        ..detected("lower.stl", Lower, 0.99)
    };
    check(&allocate(&[], &[f]), &[put(Upper, Explicit)], &[]);

    let case = [old(1, Upper, FileName, "u.stl")];
    let a = allocate(&case, &[explicit("x.stl", Upper)]);
    check(&a, &[over(Upper, Explicit, &[1])], &[WarningCode::Replaced]);
    // the name hint replaces too
    let a = allocate(&case, &[file("upper.stl")]);
    check(&a, &[over(Upper, FileName, &[1])], &[WarningCode::Replaced]);
    // an explicit arch beats the bite token of the name
    let a = allocate(&[], &[explicit("bite.stl", Lower)]);
    check(&a, &[put(Lower, Explicit)], &[]);
}

#[test]
fn rule_3_a_provisional_occupant_moves_to_the_free_arch() {
    let case = [old(1, Lower, Provisional, "a.stl")];
    let a = allocate(&case, &[file("lower.stl")]);
    check(&a, &[put(Lower, FileName)], &[WarningCode::Provisional]);
    assert_eq!(a.moves.len(), 1);
    assert_eq!(
        (a.moves[0].scan, a.moves[0].from, a.moves[0].to),
        (Uuid::from_u128(1), Lower, Upper)
    );
    // the detection moves it as well, and it is no arch conflict
    let a = allocate(&case, &[detected("b.stl", Lower, 0.9)]);
    check(&a, &[put(Lower, Detected)], &[WarningCode::Provisional]);
    assert_eq!(a.moves.len(), 1);
}

#[test]
fn rule_4_a_detected_unsure_scan_is_settled_and_never_displaced() {
    // an unhinted file with a surer detection does not move it away: the
    // scan holds its arch like a detected one, the file goes to the other arch
    let case = [old(1, Lower, DetectedUnsure, "a.stl")];
    let a = allocate(&case, &[detected("b.stl", Lower, 0.99)]);
    check(&a, &[put(Upper, Provisional)], &[WarningCode::ArchConflict]);
    assert!(a.moves.is_empty());
    // with the other arch held as well there is no place left
    let case = [
        old(1, Lower, DetectedUnsure, "a.stl"),
        old(2, Upper, Explicit, "u.stl"),
    ];
    let a = allocate(&case, &[detected("b.stl", Lower, 0.99)]);
    check(&a, &[needs_role("b.stl")], &[]);
    assert!(a.moves.is_empty());
    // an unhinted file without detection does not take its slot either
    let a = allocate(&case[..1], &[file("c.stl")]);
    check(&a, &[put(Upper, Provisional)], &[WarningCode::Provisional]);
    assert!(a.moves.is_empty());
}

#[test]
fn rule_3_without_a_free_arch_the_provisional_scan_is_replaced() {
    // the other arch is held by the case
    let case = [
        old(1, Lower, Provisional, "a.stl"),
        old(2, Upper, FileName, "u.stl"),
    ];
    let a = allocate(&case, &[file("lower.stl")]);
    check(&a, &[over(Lower, FileName, &[1])], &[WarningCode::Replaced]);
    assert!(a.moves.is_empty());
    // the other arch is claimed in the same batch, whatever comes first
    let case = [old(1, Lower, Provisional, "a.stl")];
    for files in [
        [file("lower.stl"), file("upper.stl")],
        [file("upper.stl"), file("lower.stl")],
    ] {
        let a = allocate(&case, &files);
        assert!(a.moves.is_empty());
        assert_eq!(codes(&a.warnings), [WarningCode::Replaced]);
    }
}

#[test]
fn rule_4_detected_files_take_their_arch_then_the_other_then_fail() {
    // free
    let a = allocate(&[], &[detected("a.stl", Upper, 0.8)]);
    check(&a, &[put(Upper, Detected)], &[]);
    // taken, other free
    let case = [old(1, Upper, FileName, "u.stl")];
    let a = allocate(&case, &[detected("a.stl", Upper, 0.8)]);
    check(&a, &[put(Lower, Provisional)], &[WarningCode::ArchConflict]);
    // both taken
    let case = [
        old(1, Upper, FileName, "u.stl"),
        old(2, Lower, Explicit, "l.stl"),
    ];
    let a = allocate(&case, &[detected("a.stl", Lower, 0.8)]);
    check(&a, &[needs_role("a.stl")], &[]);
}

#[test]
fn rule_4_a_detection_never_clears_a_confirmed_arch() {
    for (upper, lower) in [
        (FileName, Explicit),
        (Explicit, Detected),
        (Detected, FileName),
    ] {
        let case = [old(1, Upper, upper, "u.stl"), old(2, Lower, lower, "l.stl")];
        for arch in [Upper, Lower] {
            let a = allocate(&case, &[detected("a.stl", arch, 0.99)]);
            check(&a, &[needs_role("a.stl")], &[]);
            assert!(a.moves.is_empty());
        }
    }
    // a file without a detection does no more than that
    let case = [
        old(1, Upper, FileName, "u.stl"),
        old(2, Lower, FileName, "l.stl"),
    ];
    check(
        &allocate(&case, &[file("a.stl")]),
        &[needs_role("a.stl")],
        &[],
    );
}

#[test]
fn rule_4_only_a_provisional_scan_gives_way() {
    // the other arch is taken: it cannot step aside, so it is replaced
    let case = [
        old(1, Lower, Provisional, "a.stl"),
        old(2, Upper, FileName, "u.stl"),
    ];
    let a = allocate(&case, &[detected("b.stl", Lower, 0.9)]);
    check(&a, &[over(Lower, Detected, &[1])], &[WarningCode::Replaced]);
    assert!(a.moves.is_empty());
    // the guess on the other arch is not touched by a file that wants this one
    let a = allocate(&case, &[detected("b.stl", Upper, 0.9)]);
    check(&a, &[needs_role("b.stl")], &[]);
    // both arches are guesses
    let case = [
        old(1, Lower, Provisional, "a.stl"),
        old(2, Upper, Provisional, "b.stl"),
    ];
    let a = allocate(&case, &[detected("c.stl", Upper, 0.9)]);
    check(&a, &[over(Upper, Detected, &[2])], &[WarningCode::Replaced]);
}

#[test]
fn rule_4_goes_by_descending_confidence_ties_in_file_order() {
    let a = allocate(
        &[],
        &[detected("a.stl", Upper, 0.4), detected("b.stl", Upper, 0.9)],
    );
    check(
        &a,
        &[put(Lower, Provisional), put(Upper, Detected)],
        &[WarningCode::ArchConflict],
    );
    let a = allocate(
        &[],
        &[detected("a.stl", Upper, 0.7), detected("b.stl", Upper, 0.7)],
    );
    check(
        &a,
        &[put(Upper, Detected), put(Lower, Provisional)],
        &[WarningCode::ArchConflict],
    );
}

#[test]
fn rule_4_never_replaces_a_file_of_the_same_batch() {
    let a = allocate(
        &[],
        &[
            detected("a.stl", Upper, 0.9),
            detected("b.stl", Upper, 0.8),
            detected("c.stl", Upper, 0.7),
        ],
    );
    check(
        &a,
        &[
            put(Upper, Detected),
            put(Lower, Provisional),
            needs_role("c.stl"),
        ],
        &[WarningCode::ArchConflict],
    );
}

#[test]
fn rule_4_runs_after_the_named_files() {
    // the name reserves the arch, the detection gets the other one
    let a = allocate(&[], &[detected("a.stl", Upper, 0.99), file("upper.stl")]);
    check(
        &a,
        &[put(Lower, Provisional), put(Upper, FileName)],
        &[WarningCode::ArchConflict],
    );
}

/// The scan steps aside for the first file, then a second one wants the arch
/// it went to: it is replaced, by the file that took its place, and never moved.
#[test]
fn a_scan_moved_and_then_replaced_is_only_replaced() {
    let case = [old(1, Lower, Provisional, "p.stl")];
    for files in [
        [file("lower.stl"), detected("seen.stl", Upper, 0.9)],
        [detected("seen.stl", Upper, 0.9), file("lower.stl")],
    ] {
        let a = allocate(&case, &files);
        assert!(a.moves.is_empty());
        assert_eq!(codes(&a.warnings), [WarningCode::Replaced]);
        let i = files.iter().position(|f| f.name == "lower.stl").unwrap();
        assert_eq!(a.decisions[i], over(Lower, FileName, &[1]));
        assert_eq!(a.decisions[1 - i], put(Upper, Detected));
        let w = &a.warnings[0];
        assert_eq!(w.args["name"], "lower.stl");
        assert_eq!(w.args["old"], "p.stl");
        assert_eq!(w.args["role"], "lower");
    }
    // two detections, the surer one first
    let a = allocate(
        &case,
        &[detected("a.stl", Lower, 0.9), detected("b.stl", Upper, 0.8)],
    );
    check(
        &a,
        &[over(Lower, Detected, &[1]), put(Upper, Detected)],
        &[WarningCode::Replaced],
    );
    assert!(a.moves.is_empty());
}

#[test]
fn locked_arches_refuse_to_replace_or_move() {
    let locked = |case: &[Scan], files: &[FileClaim]| allocate_with(case, files, Some("future"));
    let fail = || {
        Decision::Fail(ImportError::ArchesLocked {
            module: "future".into(),
        })
    };
    // a claim on an occupied arch, provisional or not
    let case = [old(1, Upper, FileName, "u.stl")];
    check(&locked(&case, &[file("upper.stl")]), &[fail()], &[]);
    let case = [old(1, Lower, Provisional, "p.stl")];
    let a = locked(&case, &[explicit("a.stl", Lower)]);
    check(&a, &[fail()], &[]);
    assert!(a.moves.is_empty());
    // a detection leaves the provisional scan and takes the free arch
    let a = locked(&case, &[detected("b.stl", Lower, 0.9)]);
    check(&a, &[put(Upper, Provisional)], &[WarningCode::ArchConflict]);
    // free arches and bites work as usual, in the same batch as a refused file
    let case = [
        old(1, Upper, FileName, "u.stl"),
        old(2, Bite, Explicit, "a.stl"),
        old(3, Bite, Explicit, "b.stl"),
    ];
    let a = locked(
        &case,
        &[file("upper.stl"), file("lower.stl"), file("bite.stl")],
    );
    check(
        &a,
        &[fail(), put(Lower, FileName), over(Bite, FileName, &[3])],
        &[WarningCode::BiteLimit],
    );
}

#[test]
fn rule_5_unhinted_files_fill_the_free_arches_lower_first() {
    let a = allocate(&[], &[file("a.stl"), file("b.stl"), file("c.stl")]);
    check(
        &a,
        &[
            put(Lower, Provisional),
            put(Upper, Provisional),
            needs_role("c.stl"),
        ],
        &[WarningCode::Provisional, WarningCode::Provisional],
    );
    let case = [old(1, Lower, Explicit, "l.stl")];
    let a = allocate(&case, &[file("a.stl")]);
    check(&a, &[put(Upper, Provisional)], &[WarningCode::Provisional]);
}

#[test]
fn rule_5_never_replaces_and_never_lands_in_bite() {
    let case = [
        old(1, Lower, Provisional, "a.stl"),
        old(2, Upper, Provisional, "b.stl"),
    ];
    let a = allocate(&case, &[file("c.stl")]);
    check(&a, &[needs_role("c.stl")], &[]);
    assert!(a.moves.is_empty());
    // a detection of "bite" counts as no detection
    let a = allocate(&[], &[detected("c.stl", Bite, 0.99)]);
    check(&a, &[put(Lower, Provisional)], &[WarningCode::Provisional]);
}

#[test]
fn rule_6_bite_comes_from_a_choice_or_a_name_token() {
    let a = allocate(&[], &[explicit("a.stl", Bite), file("morso.stl")]);
    check(&a, &[put(Bite, Explicit), put(Bite, FileName)], &[]);
}

#[test]
fn rule_6_a_third_bite_replaces_the_second() {
    let case = [
        old(1, Bite, Explicit, "left.stl"),
        old(2, Bite, Explicit, "right.stl"),
    ];
    let a = allocate(&case, &[file("bite3.stl")]);
    check(&a, &[over(Bite, FileName, &[2])], &[WarningCode::BiteLimit]);
}

#[test]
fn rule_6_a_batch_over_the_limit_keeps_the_first_two() {
    let a = allocate(
        &[],
        &[
            file("bite1.stl"),
            file("bite2.stl"),
            file("bite3.stl"),
            file("bite4.stl"),
        ],
    );
    check(
        &a,
        &[
            put(Bite, FileName),
            put(Bite, FileName),
            Decision::Skip,
            Decision::Skip,
        ],
        &[WarningCode::BiteLimit, WarningCode::BiteLimit],
    );
}

#[test]
fn rule_6_new_bites_never_replace_each_other() {
    let one = [old(1, Bite, Explicit, "a.stl")];
    let a = allocate(&one, &[file("bite1.stl"), file("bite2.stl")]);
    check(
        &a,
        &[put(Bite, FileName), over(Bite, FileName, &[1])],
        &[WarningCode::BiteLimit],
    );
    let two = [
        old(1, Bite, Explicit, "a.stl"),
        old(2, Bite, Explicit, "b.stl"),
    ];
    let a = allocate(&two, &[file("bite1.stl"), file("bite2.stl")]);
    check(
        &a,
        &[over(Bite, FileName, &[2]), over(Bite, FileName, &[1])],
        &[WarningCode::BiteLimit, WarningCode::BiteLimit],
    );
}

#[test]
fn files_claiming_the_same_arch_never_depend_on_the_order() {
    // same rank: all of them fail and the scan in place stays
    let case = [old(1, Upper, FileName, "u.stl")];
    for names in [
        ["upper_a.stl", "upper_b.stl"],
        ["upper_b.stl", "upper_a.stl"],
    ] {
        let a = allocate(&case, &[file(names[0]), file(names[1])]);
        check(&a, &[clash(names[0], Upper), clash(names[1], Upper)], &[]);
    }
    let a = allocate(&[], &[explicit("a.stl", Lower), explicit("b.stl", Lower)]);
    check(&a, &[clash("a.stl", Lower), clash("b.stl", Lower)], &[]);
    // an explicit claim beats the name
    let a = allocate(&[], &[file("upper.stl"), explicit("a.stl", Upper)]);
    check(&a, &[clash("upper.stl", Upper), put(Upper, Explicit)], &[]);
}

#[test]
fn the_result_does_not_depend_on_the_file_order() {
    let case = [
        old(1, Lower, Provisional, "p.stl"),
        old(2, Bite, Explicit, "b.stl"),
    ];
    let files = [
        explicit("chosen.stl", Bite),
        file("upper.stl"),
        file("lower.stl"),
        file("bite.stl"),
        detected("seen.stl", Lower, 0.5),
    ];
    let result = |order: &[usize]| {
        let batch: Vec<FileClaim> = order.iter().map(|&i| files[i].clone()).collect();
        let a = allocate(&case, &batch);
        let mut by_name: Vec<(String, Decision)> =
            batch.into_iter().map(|f| f.name).zip(a.decisions).collect();
        by_name.sort_by(|x, y| x.0.cmp(&y.0));
        (by_name, a.moves, codes(&a.warnings))
    };
    let want = result(&[0, 1, 2, 3, 4]);
    let mut order = [0, 1, 2, 3, 4];
    // every rotation and every swap of two files
    for _ in 0..5 {
        order.rotate_left(1);
        assert_eq!(result(&order), want, "{order:?}");
        for i in 0..4 {
            order.swap(i, i + 1);
            assert_eq!(result(&order), want, "{order:?}");
            order.swap(i, i + 1);
        }
    }
}

// --- prepare ----------------------------------------------------------------

#[test]
fn files_without_usable_faces_are_rejected() {
    let d = Dir::new();
    let ctx = JobCtx::detached();
    let cases = [
        d.text("nofaces.obj", "v 0 0 0\nv 1 0 0\nv 0 1 0\n"),
        d.stl("zero.stl", &TriMesh::default()),
        // all degenerate: left with no faces after the cleanup
        d.text(
            "degenerate.obj",
            "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 1 2\nf 2 3 3\nf 1 2 1\n",
        ),
    ];
    for path in cases {
        let e = prepare(&path, &ctx).unwrap_err();
        assert_eq!(e.code(), "empty", "{}", path.display());
    }
    // an STL whose triangles collapse on a welded vertex
    let soup = "solid s\nfacet normal 0 0 1\nouter loop\nvertex 0 0 0\nvertex 0 0 0\nvertex 1 0 0\nendloop\nendfacet\nendsolid s\n";
    let e = prepare(&d.text("collapsed.stl", soup), &ctx).unwrap_err();
    assert_eq!(e.code(), "empty");
}

#[test]
fn corrupt_files_are_rejected() {
    let d = Dir::new();
    let ctx = JobCtx::detached();
    let code = |path: PathBuf| prepare(&path, &ctx).unwrap_err().code();

    assert_eq!(code(d.path("missing.stl")), "unreadable");
    assert_eq!(code(d.text("scan.dcm", "x")), "unsupported");
    // NaN
    assert_eq!(
        code(d.text("nan.obj", "v nan 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n")),
        "invalid"
    );
    // index out of range
    assert_eq!(
        code(d.text("range.obj", "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 9\n")),
        "invalid"
    );
    // truncated binary STL
    let path = d.stl("cut.stl", &arch(60.0));
    let bytes = std::fs::read(&path).unwrap();
    std::fs::write(&path, &bytes[..bytes.len() / 2]).unwrap();
    let e = prepare(&path, &ctx).unwrap_err();
    assert_eq!(e.code(), "invalid");
    assert!(e.detail().is_some_and(|s| s.contains("truncated")));
    assert!(!e.message().contains("truncated"));
}

#[test]
fn units_follow_the_dc_mesh_rules() {
    let d = Dir::new();
    let ctx = JobCtx::detached();
    let cases: &[(f32, Units, f32)] = &[
        (60.0, Units::Millimeters, 60.0),
        (0.06, Units::Meters, 60.0),
        (6.0, Units::Centimeters, 60.0),
        // the micrometre rule
        (60_000.0, Units::Micrometers, 60.0),
        // the boundaries of the metre rule are inclusive
        (0.02, Units::Meters, 20.0),
        (0.2, Units::Meters, 200.0),
        (0.019, Units::Millimeters, 0.019),
        (0.21, Units::Millimeters, 0.21),
    ];
    for (i, &(size, units, mm)) in cases.iter().enumerate() {
        // exact coordinates: the boundaries are compared as written in the file
        let text = format!("v 0 0 0\nv {size} 0 0\nv 0 {size} 0\nf 1 2 3\n");
        let path = d.text(&format!("u{i}.obj"), &text);
        let scan = prepare(&path, &ctx).unwrap();
        assert_eq!(scan.source_units, units, "{size}");
        let got = scan.mesh.aabb().unwrap().size().max_element();
        assert!((got - mm).abs() < mm * 1e-3, "{size}: {got}");
        let rescaled = codes(&scan.warnings).contains(&WarningCode::Rescaled);
        assert_eq!(rescaled, units != Units::Millimeters, "{size}");
    }
}

#[test]
fn obj_files_go_from_y_up_to_z_up_on_real_numbers() {
    let d = Dir::new();
    let text = "v -33.842 -2.484 -11.998\nv 25 1 3\nv 0 12 -9\nf 1 2 3\n";
    let scan = prepare(&d.text("scan.obj", text), &JobCtx::detached()).unwrap();
    assert_eq!(scan.source_units, Units::Millimeters);
    assert_eq!(scan.mesh.positions[0], Vec3::new(-33.842, 11.998, -2.484));
    // the axes change for OBJ only
    let stl = "solid s\nfacet normal 0 0 1\nouter loop\nvertex -33.842 -2.484 -11.998\nvertex 25 1 3\nvertex 0 12 -9\nendloop\nendfacet\nendsolid s\n";
    let scan = prepare(&d.text("scan.stl", stl), &JobCtx::detached()).unwrap();
    assert!(
        scan.mesh
            .positions
            .contains(&Vec3::new(-33.842, -2.484, -11.998))
    );
}

#[test]
fn the_tiny_warning_looks_at_the_rescaled_mesh() {
    let d = Dir::new();
    let ctx = JobCtx::detached();
    // 0.06 m: tiny as a raw number, 60 mm once converted
    let scan = prepare(&d.stl("m.stl", &arch(0.06)), &ctx).unwrap();
    assert!(!codes(&scan.warnings).contains(&WarningCode::Tiny));
    assert!(codes(&scan.warnings).contains(&WarningCode::Rescaled));
    // 1 mm stays 1 mm
    let scan = prepare(&d.stl("mm.stl", &arch(1.0)), &ctx).unwrap();
    assert_eq!(codes(&scan.warnings), [WarningCode::Tiny]);
    // just under and at the limit
    let scan = prepare(&d.stl("a.stl", &arch(0.21)), &ctx).unwrap();
    assert_eq!(codes(&scan.warnings), [WarningCode::Tiny]);
    let scan = prepare(&d.stl("b.stl", &arch(20.5)), &ctx).unwrap();
    assert!(scan.warnings.is_empty());
}

/// Largest extent of the mesh of a scan of the case, in mm.
fn extent(state: &CaseState, name: &str) -> f32 {
    let scan = state.case.scans.iter().find(|s| s.source_name == name);
    let mesh = state.blobs.get(&scan.unwrap().mesh).unwrap();
    mesh.aabb().unwrap().size().max_element()
}

#[test]
fn keeping_the_units_leaves_the_coordinates_and_warns() {
    let d = Dir::new();
    let ctx = JobCtx::detached();
    let keep = PrepareOptions::KEEP_UNITS;
    let path = d.stl("visit.stl", &arch(15.0));
    // the default is what `prepare` does: 15 looks like centimetres
    let scan = prepare(&path, &ctx).unwrap();
    assert_eq!(scan.source_units, Units::Centimeters);
    assert_eq!(codes(&scan.warnings), [WarningCode::Rescaled]);
    assert!(PrepareOptions::default().normalize_units);

    let scan = prepare_with(&path, &keep, &ctx).unwrap();
    assert_eq!(scan.source_units, Units::Millimeters);
    assert_eq!(scan.mesh.aabb().unwrap().size().max_element(), 15.0);
    assert_eq!(codes(&scan.warnings), [WarningCode::UnitsSuspect]);
    assert_eq!(scan.warnings[0].args["units"], "cm");

    // metres and micrometres are suspect as well; a size in millimetres is not
    let scan = prepare_with(&d.stl("m.stl", &arch(0.06)), &keep, &ctx).unwrap();
    assert_eq!(scan.warnings[0].args["units"], "m");
    assert!(codes(&scan.warnings).contains(&WarningCode::Tiny));
    let scan = prepare_with(&d.stl("um.stl", &arch(60_000.0)), &keep, &ctx).unwrap();
    assert_eq!(codes(&scan.warnings), [WarningCode::UnitsSuspect]);
    assert_eq!(scan.warnings[0].args["units"], "um");
    let scan = prepare_with(&d.stl("mm.stl", &arch(30.0)), &keep, &ctx).unwrap();
    assert!(scan.warnings.is_empty());

    // the axes still change
    let obj = "v 1 2 3\nv 4 2 3\nv 1 9 3\nf 1 2 3\n";
    let scan = prepare_with(&d.text("v.obj", obj), &keep, &ctx).unwrap();
    assert_eq!(scan.mesh.positions[0], Vec3::new(1.0, -3.0, 2.0));
}

#[test]
fn a_bite_in_a_centimetre_size_is_not_rescaled() {
    let d = Dir::new();
    let mesh = arch(15.0);
    let mut state = CaseState::new(Case::new("T"));
    // from the name token
    let report = import(&mut state, &[d.stl("bite_left.stl", &mesh)]);
    assert_eq!(report.imported[0].role, Bite);
    assert_eq!(report.imported[0].source_units, Units::Millimeters);
    assert_eq!(codes(&report.warnings), [WarningCode::UnitsSuspect]);
    assert_eq!(report.warnings[0].args["units"], "cm");
    assert_eq!(extent(&state, "bite_left.stl"), 15.0);
    // from the explicit choice, whatever the name says
    let report = import_as(&mut state, d.stl("upper.stl", &mesh), Bite);
    assert_eq!(report.imported[0].role, Bite);
    assert_eq!(codes(&report.warnings), [WarningCode::UnitsSuspect]);
    assert_eq!(extent(&state, "upper.stl"), 15.0);

    // an explicit arch beats the bite token: a whole arch, rescaled
    let mut state = CaseState::new(Case::new("T"));
    let report = import_as(&mut state, d.stl("bite.stl", &mesh), Upper);
    assert_eq!(report.imported[0].source_units, Units::Centimeters);
    assert_eq!(codes(&report.warnings), [WarningCode::Rescaled]);
    assert_eq!(extent(&state, "bite.stl"), 150.0);
    // and an unhinted file is treated as an arch
    let mut state = CaseState::new(Case::new("T"));
    let report = import(&mut state, &[d.stl("scan.stl", &mesh)]);
    assert_eq!(report.imported[0].source_units, Units::Centimeters);
    assert_eq!(extent(&state, "scan.stl"), 150.0);
}

#[test]
fn the_low_polygon_limit_warns_and_does_not_reject() {
    let d = Dir::new();
    let ctx = JobCtx::detached();
    // 22 cells: 968 faces; 23 cells: 1058 faces
    let scan = prepare(&d.stl("few.stl", &grid(22, 60.0)), &ctx).unwrap();
    assert_eq!(codes(&scan.warnings), [WarningCode::LowPolygons]);
    let scan = prepare(&d.stl("many.stl", &grid(23, 60.0)), &ctx).unwrap();
    assert!(scan.warnings.is_empty());
}

#[test]
fn cleanup_counts_and_obj_objects_are_reported() {
    let d = Dir::new();
    let text =
        "o a\nv 0 0 0\nv 60 0 0\nv 0 60 0\nv 60 60 0\nf 1 2 3\nf 3 2 1\nf 1 1 2\no b\nf 2 4 3\n";
    let scan = prepare(&d.text("scan.obj", text), &JobCtx::detached()).unwrap();
    assert_eq!(scan.cleanup.degenerate, 1);
    assert_eq!(scan.cleanup.duplicate, 1);
    assert_eq!(scan.mesh.face_count(), 2);
    let got = codes(&scan.warnings);
    assert!(got.contains(&WarningCode::ObjObjectsMerged), "{got:?}");
    assert!(got.contains(&WarningCode::BadFacesRemoved), "{got:?}");
}

#[test]
fn a_cancelled_job_changes_nothing() {
    let d = Dir::new();
    let path = d.stl("upper.stl", &arch(60.0));
    let ctx = JobCtx::detached();
    ctx.cancel_token().cancel();
    assert_eq!(prepare(&path, &ctx).unwrap_err(), ImportError::Cancelled);
    let mut state = CaseState::new(Case::new("T"));
    let request = ImportRequest::from_paths([path]);
    let report = import_files(&mut state, &request, None, &ctx);
    assert!(report.cancelled);
    assert!(report.imported.is_empty() && report.failed.is_empty());
    assert!(state.case.scans.is_empty() && state.blobs.is_empty());
}

// --- import_files -----------------------------------------------------------

#[test]
fn imports_a_batch_with_roles_and_sources() {
    let d = Dir::new();
    let mut state = CaseState::new(Case::new("T"));
    let report = import(
        &mut state,
        &[
            d.stl("scan.stl", &arch(60.0)),
            d.stl("UpperJaw.stl", &arch(61.0)),
        ],
    );
    assert!(report.failed.is_empty());
    assert_eq!(codes(&report.warnings), [WarningCode::Provisional]);
    assert_eq!(report.imported[0].role, Lower);
    assert_eq!(report.imported[0].role_source, Provisional);
    assert_eq!(report.imported[1].role, Upper);
    assert_eq!(report.imported[1].role_source, FileName);
    assert_eq!(report.imported[1].source_name, "UpperJaw.stl");
    assert_eq!(report.imported[1].faces, 1152);
    assert_eq!(
        roles(&state),
        [(Upper, "UpperJaw.stl"), (Lower, "scan.stl")]
    );
    assert_eq!(state.blobs.len(), 2);
    assert!(state.blobs.contains(&report.imported[0].mesh));
    assert_eq!(state.case.scans[0].import_xform, Pose::IDENTITY);
    assert_eq!(state.case.scan(Upper).unwrap().role_source, FileName);
}

#[test]
fn an_empty_file_named_upper_keeps_the_good_scan() {
    let d = Dir::new();
    let mut state = CaseState::new(Case::new("T"));
    import(&mut state, &[d.stl("upper.stl", &arch(60.0))]);
    let good = state.case.scan(Upper).unwrap().clone();
    let frame = WorldFrame {
        from_case: Pose::from_translation(DVec3::X),
    };
    state.case.world_frame = frame;

    let report = import(&mut state, &[d.stl("upper2.stl", &TriMesh::default())]);
    assert!(report.imported.is_empty());
    assert_eq!(report.failed.len(), 1);
    assert_eq!(report.failed[0].code, "empty");
    assert!(report.failed[0].path.ends_with("upper2.stl"));
    assert!(report.replaced_roles.is_empty());
    assert_eq!(state.case.scans, std::slice::from_ref(&good));
    assert!(state.blobs.contains(&good.mesh));
    assert_eq!(state.case.world_frame, frame);
}

#[test]
fn a_failed_file_does_not_stop_the_batch() {
    let d = Dir::new();
    let mut state = CaseState::new(Case::new("T"));
    let report = import(
        &mut state,
        &[
            d.stl("upper.stl", &arch(60.0)),
            d.path("missing.stl"),
            d.text("broken.obj", "v 0 0 0\nf 1 2 3\n"),
            d.stl("lower.stl", &arch(61.0)),
        ],
    );
    assert_eq!(report.imported.len(), 2);
    let failed: Vec<&str> = report.failed.iter().map(|f| f.code).collect();
    assert_eq!(failed, ["unreadable", "invalid"]);
    assert_eq!(roles(&state).len(), 2);
}

#[test]
fn when_nothing_is_imported_the_state_is_unchanged() {
    let d = Dir::new();
    let mut state = CaseState::new(Case::new("T"));
    import(
        &mut state,
        &[
            d.stl("upper.stl", &arch(60.0)),
            d.stl("lower.stl", &arch(61.0)),
        ],
    );
    let (case, blobs) = (state.case.clone(), state.blobs.len());
    // no slot for an unhinted file, a clash, an empty file
    let report = import(
        &mut state,
        &[
            d.stl("scan.stl", &arch(62.0)),
            d.stl("upper_a.stl", &arch(63.0)),
            d.stl("upper_b.stl", &arch(64.0)),
            d.text("lower_empty.obj", "v 0 0 0\n"),
        ],
    );
    assert!(report.imported.is_empty());
    assert_eq!(report.failed.len(), 4);
    assert!(report.warnings.is_empty(), "{:?}", report.warnings);
    assert_eq!(state.case, case);
    assert_eq!(state.blobs.len(), blobs);
}

#[test]
fn the_resolver_runs_only_for_unhinted_files() {
    let d = Dir::new();
    let mut state = CaseState::new(Case::new("T"));
    let mut seen = Vec::new();
    let mut detect = |scan: &dc_case::import::PreparedScan| {
        seen.push(scan.source_name.clone());
        Some(Detection {
            arch: Upper,
            confidence: 0.9,
        })
    };
    let resolver: &mut Resolver<'_> = &mut detect;
    let request = ImportRequest {
        files: vec![
            ImportFile {
                path: d.stl("scan.stl", &arch(60.0)),
                role: None,
            },
            ImportFile {
                path: d.stl("lower.stl", &arch(61.0)),
                role: None,
            },
            ImportFile {
                path: d.stl("other.stl", &arch(62.0)),
                role: Some(Bite),
            },
        ],
    };
    let report = import_files(&mut state, &request, Some(resolver), &JobCtx::detached());
    assert_eq!(seen, ["scan.stl"]);
    assert_eq!(report.imported[0].role, Upper);
    assert_eq!(report.imported[0].role_source, Detected);
    assert_eq!(report.imported[1].role, Lower);
    assert_eq!(report.imported[2].role, Bite);
    assert!(report.warnings.is_empty());
}

/// Module data that remembers the hook calls and holds one blob per role.
#[derive(Debug, Clone, Serialize)]
struct Labels {
    labels: Vec<(ScanRole, BlobRef)>,
    calls: Vec<ScanRole>,
}

impl ModuleData for Labels {
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
    fn to_json(&self) -> Result<serde_json::Value, serde_json::Error> {
        serde_json::to_value(self)
    }
    fn referenced_blobs(&self) -> Vec<BlobRef> {
        self.labels.iter().map(|(_, b)| *b).collect()
    }
    fn scan_replaced(&mut self, role: ScanRole) -> Vec<&'static str> {
        self.calls.push(role);
        self.labels.retain(|(r, _)| *r != role);
        vec!["labels"]
    }
}

/// A case with both arches, a labels blob on the upper one and a blob nothing
/// refers to.
fn labelled_case(d: &Dir) -> (CaseState, BlobRef, BlobRef) {
    let mut state = CaseState::new(Case::new("T"));
    import(
        &mut state,
        &[
            d.stl("upper.stl", &arch(60.0)),
            d.stl("lower.stl", &arch(61.0)),
        ],
    );
    let labels = state.blobs.insert(arch(65.0));
    let loose = state.blobs.insert(arch(66.0));
    state.set_module(
        "m",
        ModuleState::new(Labels {
            labels: vec![(Upper, labels)],
            calls: Vec::new(),
        }),
    );
    state.case.world_frame.from_case = Pose::from_translation(DVec3::X);
    (state, labels, loose)
}

fn hook_calls(state: &CaseState) -> Vec<ScanRole> {
    state.module_data::<Labels>("m").unwrap().calls.clone()
}

#[test]
fn replacing_an_arch_resets_the_frame_clears_the_module_and_drops_blobs() {
    let d = Dir::new();
    let (mut state, labels, loose) = labelled_case(&d);
    let old_scan = state.case.scan(Upper).unwrap().mesh;

    let report = import(&mut state, &[d.stl("upper_new.stl", &arch(70.0))]);
    assert_eq!(report.replaced_roles, [Upper]);
    assert_eq!(report.module_cleared, ["labels"]);
    assert_eq!(codes(&report.warnings), [WarningCode::Replaced]);
    assert_eq!(state.case.world_frame, WorldFrame::default());
    assert_eq!(hook_calls(&state), [Upper]);
    // the replaced scan and what the module let go are gone, the rest stays
    assert!(!state.blobs.contains(&old_scan));
    assert!(!state.blobs.contains(&labels));
    assert!(state.blobs.contains(&loose));
    assert_eq!(state.case.scans.len(), 2);
}

#[test]
fn a_blob_still_in_use_survives_the_replacement() {
    let d = Dir::new();
    let (mut state, _, _) = labelled_case(&d);
    let old_scan = state.case.scan(Lower).unwrap().mesh;
    state
        .modules
        .get_mut("m")
        .unwrap()
        .downcast_mut::<Labels>()
        .unwrap()
        .labels
        .push((Upper, old_scan));
    import(&mut state, &[d.stl("lower_new.stl", &arch(70.0))]);
    assert!(state.blobs.contains(&old_scan));
}

#[test]
fn filling_a_free_slot_keeps_frame_and_modules() {
    let d = Dir::new();
    let mut state = CaseState::new(Case::new("T"));
    import(&mut state, &[d.stl("upper.stl", &arch(60.0))]);
    state.set_module(
        "m",
        ModuleState::new(Labels {
            labels: Vec::new(),
            calls: Vec::new(),
        }),
    );
    let frame = WorldFrame {
        from_case: Pose::from_translation(DVec3::Y),
    };
    state.case.world_frame = frame;
    let report = import(&mut state, &[d.stl("lower.stl", &arch(61.0))]);
    assert!(report.replaced_roles.is_empty());
    assert!(report.module_cleared.is_empty());
    assert!(hook_calls(&state).is_empty());
    assert_eq!(state.case.world_frame, frame);
}

#[test]
fn moving_a_provisional_scan_counts_as_a_change_of_both_arches() {
    let d = Dir::new();
    let mut state = CaseState::new(Case::new("T"));
    import(&mut state, &[d.stl("scan.stl", &arch(60.0))]);
    let moved = state.case.scan(Lower).unwrap().clone();
    assert_eq!(moved.role_source, Provisional);
    state.set_module(
        "m",
        ModuleState::new(Labels {
            labels: Vec::new(),
            calls: Vec::new(),
        }),
    );
    state.case.world_frame.from_case = Pose::from_translation(DVec3::Z);

    let report = import(&mut state, &[d.stl("lower.stl", &arch(61.0))]);
    assert_eq!(report.moved.len(), 1);
    assert_eq!(report.replaced_roles, [Upper, Lower]);
    assert_eq!(codes(&report.warnings), [WarningCode::Provisional]);
    assert_eq!(roles(&state), [(Upper, "scan.stl"), (Lower, "lower.stl")]);
    let now = state.case.scan(Upper).unwrap();
    assert_eq!((now.id, now.mesh), (moved.id, moved.mesh));
    assert_eq!(now.role_source, Provisional);
    assert_eq!(state.blobs.len(), 2);
    assert_eq!(state.case.world_frame, WorldFrame::default());
    assert_eq!(hook_calls(&state), [Upper, Lower]);
}

#[test]
fn removing_an_arch_resets_the_frame_clears_the_module_and_drops_blobs() {
    let d = Dir::new();
    let (mut state, labels, loose) = labelled_case(&d);
    let gone = state.case.scan(Upper).unwrap().clone();
    let snapshot = state.clone();

    let removed = remove_scan(&mut state, gone.id).unwrap().unwrap();
    assert_eq!(removed.scan, gone);
    assert!(removed.world_frame_reset);
    // the default hook is a replacement of the scan's role
    assert_eq!(removed.module_cleared, ["labels"]);
    assert_eq!(hook_calls(&state), [Upper]);
    assert_eq!(state.case.world_frame, WorldFrame::default());
    assert_eq!(roles(&state), [(Lower, "lower.stl")]);
    assert!(!state.blobs.contains(&gone.mesh));
    assert!(!state.blobs.contains(&labels));
    // a blob nothing referred to before is not the removal's business
    assert!(state.blobs.contains(&loose));
    // the state before is untouched: an undo gives everything back
    assert_eq!(snapshot.case.scans.len(), 2);
    assert!(snapshot.blobs.contains(&gone.mesh) && snapshot.blobs.contains(&labels));
}

#[test]
fn removing_a_bite_scan_keeps_the_frame_and_a_shared_blob() {
    let d = Dir::new();
    let (mut state, _, _) = labelled_case(&d);
    import(
        &mut state,
        &[
            d.stl("bite1.stl", &arch(30.0)),
            d.stl("bite2.stl", &arch(31.0)),
        ],
    );
    let frame = state.case.world_frame;
    let bite = state.case.scans_of(Bite).next().unwrap().clone();
    // the module also holds the blob of the bite scan: it stays
    state
        .modules
        .get_mut("m")
        .unwrap()
        .downcast_mut::<Labels>()
        .unwrap()
        .labels
        .push((Lower, bite.mesh));

    let removed = remove_scan(&mut state, bite.id).unwrap().unwrap();
    assert!(!removed.world_frame_reset);
    assert_eq!(state.case.world_frame, frame);
    assert_eq!(hook_calls(&state), [Bite]);
    assert_eq!(state.case.scans_of(Bite).count(), 1);
    assert!(state.case.scan_by_id(bite.id).is_none());
    assert!(state.blobs.contains(&bite.mesh));
}

#[test]
fn removing_an_unknown_scan_changes_nothing() {
    let d = Dir::new();
    let (mut state, _, _) = labelled_case(&d);
    let (case, blobs) = (state.case.clone(), state.blobs.len());
    assert_eq!(remove_scan(&mut state, Uuid::new_v4()).unwrap(), None);
    assert_eq!(state.case, case);
    assert_eq!(state.blobs.len(), blobs);
    assert!(hook_calls(&state).is_empty());
}

#[test]
fn an_opaque_module_refuses_removing_an_arch_but_not_a_bite_scan() {
    let d = Dir::new();
    let mut state = CaseState::new(Case::new("T"));
    import(
        &mut state,
        &[
            d.stl("upper.stl", &arch(60.0)),
            d.stl("bite.stl", &arch(30.0)),
        ],
    );
    lock(&mut state);
    let (case, blobs) = (state.case.clone(), state.blobs.len());
    let upper = state.case.scan(Upper).unwrap().id;
    let err = remove_scan(&mut state, upper).unwrap_err();
    assert!(matches!(&err, ImportError::ArchesLocked { module } if module == "future"));
    assert_eq!(state.case, case);
    assert_eq!(state.blobs.len(), blobs);
    let bite = state.case.scan(Bite).unwrap().id;
    assert!(remove_scan(&mut state, bite).unwrap().is_some());
    assert_eq!(roles(&state), [(Upper, "upper.stl")]);
}

/// A detector that always answers `arch`.
fn import_detected(state: &mut CaseState, paths: &[PathBuf], arch: ScanRole) -> ImportReport {
    let mut detect = |_: &dc_case::import::PreparedScan| {
        Some(Detection {
            arch,
            confidence: 0.9,
        })
    };
    let resolver: &mut Resolver<'_> = &mut detect;
    let request = ImportRequest::from_paths(paths.iter().cloned());
    import_files(state, &request, Some(resolver), &JobCtx::detached())
}

#[test]
fn an_unhinted_file_with_both_arches_confirmed_changes_nothing() {
    let d = Dir::new();
    let (mut state, _, _) = labelled_case(&d);
    let (case, blobs) = (state.case.clone(), state.blobs.len());
    for arch_ in [Upper, Lower] {
        let report = import_detected(&mut state, &[d.stl("scan.stl", &arch(70.0))], arch_);
        assert!(report.imported.is_empty());
        let failed: Vec<&str> = report.failed.iter().map(|f| f.code).collect();
        assert_eq!(failed, ["needs_role"]);
        assert!(report.warnings.is_empty(), "{:?}", report.warnings);
        assert!(report.replaced_roles.is_empty() && report.moved.is_empty());
        assert!(report.module_cleared.is_empty());
    }
    assert_eq!(state.case, case);
    assert_eq!(state.blobs.len(), blobs);
    assert!(hook_calls(&state).is_empty());
}

#[test]
fn an_unhinted_file_detected_on_a_provisional_slot_still_works() {
    let d = Dir::new();
    // the other arch is confirmed: the guess is replaced
    let mut state = CaseState::new(Case::new("T"));
    import(
        &mut state,
        &[
            d.stl("scan.stl", &arch(60.0)),
            d.stl("upper.stl", &arch(61.0)),
        ],
    );
    assert_eq!(state.case.scan(Lower).unwrap().role_source, Provisional);
    let report = import_detected(&mut state, &[d.stl("new.stl", &arch(62.0))], Lower);
    assert_eq!(report.imported[0].role, Lower);
    assert_eq!(report.imported[0].role_source, Detected);
    assert_eq!(codes(&report.warnings), [WarningCode::Replaced]);
    assert_eq!(report.replaced_roles, [Lower]);
    assert_eq!(roles(&state), [(Upper, "upper.stl"), (Lower, "new.stl")]);

    // the other arch is free: the guess steps aside
    let mut state = CaseState::new(Case::new("T"));
    import(&mut state, &[d.stl("scan.stl", &arch(60.0))]);
    let report = import_detected(&mut state, &[d.stl("new.stl", &arch(62.0))], Lower);
    assert_eq!(report.moved.len(), 1);
    assert_eq!(roles(&state), [(Upper, "scan.stl"), (Lower, "new.stl")]);
}

#[test]
fn the_report_never_says_a_scan_was_moved_and_replaced() {
    let d = Dir::new();
    let mut state = CaseState::new(Case::new("T"));
    import(&mut state, &[d.stl("scan.stl", &arch(60.0))]);
    let gone = state.case.scan(Lower).unwrap().clone();
    state.set_module(
        "m",
        ModuleState::new(Labels {
            labels: Vec::new(),
            calls: Vec::new(),
        }),
    );
    let batch = [
        d.stl("lower.stl", &arch(61.0)),
        d.stl("seen.stl", &arch(62.0)),
    ];
    let report = import_detected(&mut state, &batch, Upper);
    assert!(report.moved.is_empty(), "{:?}", report.moved);
    assert_eq!(codes(&report.warnings), [WarningCode::Replaced]);
    let message = report.warnings[0].message();
    assert!(message.contains("lower.stl") && message.contains("scan.stl"));
    // only the arch that held the scan changed: the other one was free
    assert_eq!(report.replaced_roles, [Lower]);
    assert_eq!(hook_calls(&state), [Lower]);
    assert_eq!(roles(&state), [(Upper, "seen.stl"), (Lower, "lower.stl")]);
    assert!(state.case.scan_by_id(gone.id).is_none());
    assert!(!state.blobs.contains(&gone.mesh));
    let json = serde_json::to_value(&report).unwrap();
    assert_eq!(json["moved"], serde_json::json!([]));
}

/// A module state this build cannot read, with a world frame to watch.
fn lock(state: &mut CaseState) {
    state.set_module(
        "future",
        ModuleState::new(OpaqueModuleData {
            schema_version: 9,
            value: serde_json::json!({"results": "of the old scans"}),
        }),
    );
    state.case.world_frame.from_case = Pose::from_translation(DVec3::X);
}

#[test]
fn an_opaque_module_refuses_replacing_or_moving_an_arch() {
    let d = Dir::new();
    let refused = |state: &mut CaseState, name: &str| {
        let (case, blobs) = (state.case.clone(), state.blobs.len());
        let report = import(state, &[d.stl(name, &arch(70.0))]);
        assert!(report.imported.is_empty(), "{name}");
        assert_eq!(report.failed.len(), 1, "{name}");
        assert_eq!(report.failed[0].code, "arches_locked", "{name}");
        assert!(report.failed[0].reason.contains("future"), "{name}");
        assert!(report.warnings.is_empty(), "{:?}", report.warnings);
        assert!(report.replaced_roles.is_empty() && report.moved.is_empty());
        assert_eq!(state.case, case, "{name}");
        assert_eq!(state.blobs.len(), blobs, "{name}");
    };
    // replacing
    let mut state = CaseState::new(Case::new("T"));
    import(&mut state, &[d.stl("upper.stl", &arch(60.0))]);
    lock(&mut state);
    refused(&mut state, "upper_new.stl");
    // moving a provisional scan
    let mut state = CaseState::new(Case::new("T"));
    import(&mut state, &[d.stl("scan.stl", &arch(60.0))]);
    lock(&mut state);
    refused(&mut state, "lower.stl");
}

#[test]
fn an_opaque_module_leaves_free_arches_and_bite_scans_alone() {
    let d = Dir::new();
    let mut state = CaseState::new(Case::new("T"));
    import(
        &mut state,
        &[
            d.stl("upper.stl", &arch(60.0)),
            d.stl("bite1.stl", &arch(30.0)),
            d.stl("bite2.stl", &arch(31.0)),
        ],
    );
    lock(&mut state);
    let frame = state.case.world_frame;
    // a first scan on the empty arch, a bite replacement, and a refused file
    // in the same batch: only that one fails
    let report = import(
        &mut state,
        &[
            d.stl("lower.stl", &arch(61.0)),
            d.stl("bite3.stl", &arch(32.0)),
            d.stl("upper_new.stl", &arch(62.0)),
        ],
    );
    assert_eq!(report.imported.len(), 2);
    assert_eq!(report.failed.len(), 1);
    assert_eq!(report.failed[0].code, "arches_locked");
    assert!(report.failed[0].path.ends_with("upper_new.stl"));
    assert_eq!(report.replaced_roles, [Bite]);
    assert_eq!(state.case.world_frame, frame);
    assert_eq!(state.case.scan(Upper).unwrap().source_name, "upper.stl");
    assert_eq!(state.case.scan(Lower).unwrap().source_name, "lower.stl");
}

#[test]
fn commit_and_roles_changed_refuse_on_their_own() {
    let d = Dir::new();
    let mut state = CaseState::new(Case::new("T"));
    import(
        &mut state,
        &[
            d.stl("upper.stl", &arch(60.0)),
            d.stl("bite.stl", &arch(30.0)),
        ],
    );
    lock(&mut state);
    let (case, blobs) = (state.case.clone(), state.blobs.len());
    let scan = prepare(&d.stl("new.stl", &arch(62.0)), &JobCtx::detached()).unwrap();
    let placement = |role: ScanRole, old: &Scan| Placement {
        scan: scan.clone(),
        role,
        source: Explicit,
        replaces: vec![old.id],
    };
    let upper = state.case.scan(Upper).unwrap().clone();
    let error = commit(&mut state, &[placement(Upper, &upper)], &[]).unwrap_err();
    assert_eq!(error.code(), "arches_locked");
    assert_eq!(state.case, case);
    assert_eq!(state.blobs.len(), blobs);

    let before = state.referenced_blobs();
    let touched = [Upper].into();
    assert!(roles_changed(&mut state, &touched, &before).is_err());
    assert_eq!(state.case, case);

    // the bite replacement goes through both
    let bite = state.case.scan(Bite).unwrap().clone();
    let done = commit(&mut state, &[placement(Bite, &bite)], &[]).unwrap();
    assert_eq!(done.replaced_roles, [Bite]);
    assert_eq!(state.case.world_frame, case.world_frame);
    let touched = [Bite].into();
    assert!(roles_changed(&mut state, &touched, &before).is_ok());
}

#[test]
fn two_bite_scans_live_in_the_state_and_the_third_replaces_the_second() {
    let d = Dir::new();
    let mut state = CaseState::new(Case::new("T"));
    import(
        &mut state,
        &[
            d.stl("bite_left.stl", &arch(30.0)),
            d.stl("bite_right.stl", &arch(31.0)),
            d.stl("upper.stl", &arch(60.0)),
        ],
    );
    state.case.world_frame.from_case = Pose::from_translation(DVec3::X);
    let names = |s: &CaseState| -> Vec<String> {
        s.case
            .scans_of(Bite)
            .map(|x| x.source_name.clone())
            .collect()
    };
    assert_eq!(names(&state), ["bite_left.stl", "bite_right.stl"]);

    let report = import(&mut state, &[d.stl("bite_new.stl", &arch(32.0))]);
    assert_eq!(codes(&report.warnings), [WarningCode::BiteLimit]);
    assert_eq!(report.replaced_roles, [Bite]);
    assert_eq!(names(&state), ["bite_left.stl", "bite_new.stl"]);
    // the bite does not touch the world frame
    assert_ne!(state.case.world_frame, WorldFrame::default());
    assert_eq!(state.blobs.len(), 3);

    // an explicit bite is a bite too; a batch keeps its first two
    let report = import(
        &mut state,
        &[
            d.stl("bite_1.stl", &arch(33.0)),
            d.stl("bite_2.stl", &arch(34.0)),
            d.stl("bite_3.stl", &arch(35.0)),
        ],
    );
    assert_eq!(report.imported.len(), 2);
    assert!(report.failed.is_empty());
    assert_eq!(names(&state), ["bite_1.stl", "bite_2.stl"]);
    let report = import_as(&mut state, d.stl("left.stl", &arch(36.0)), Bite);
    assert_eq!(report.imported[0].role_source, Explicit);
    assert_eq!(names(&state), ["bite_1.stl", "left.stl"]);
}

// --- messages ---------------------------------------------------------------

/// One warning of every kind, produced by real imports.
fn every_warning(d: &Dir) -> Vec<ImportWarning> {
    let mut out = Vec::new();
    let mut run = |state: &mut CaseState, files: Vec<PathBuf>| {
        out.extend(import(state, &files).warnings);
    };
    let mut s = CaseState::new(Case::new("T"));
    run(&mut s, vec![d.stl("m.stl", &arch(0.06))]); // rescaled, provisional
    let mut s = CaseState::new(Case::new("T"));
    run(&mut s, vec![d.stl("t.stl", &arch(1.5))]); // tiny, provisional
    let mut s = CaseState::new(Case::new("T"));
    run(&mut s, vec![d.stl("few.stl", &grid(4, 60.0))]); // low polygons, provisional
    let mut s = CaseState::new(Case::new("T"));
    // merged objects, bad faces, low polygons, provisional
    let obj = "o a\nv 0 0 0\nv 60 0 0\nv 0 60 0\nf 1 2 3\nf 1 1 2\no b\nf 1 3 2\n";
    run(&mut s, vec![d.text("o.obj", obj)]);
    run(&mut s, vec![d.stl("lower.stl", &arch(61.0))]); // provisional moved
    let mut s = CaseState::new(Case::new("T"));
    run(&mut s, vec![d.stl("upper.stl", &arch(60.0))]);
    run(&mut s, vec![d.stl("upper2.stl", &arch(62.0))]); // replaced
    let mut s = CaseState::new(Case::new("T"));
    let batch = (1..=2)
        .map(|i| d.stl(&format!("bite{i}.stl"), &arch(30.0 + i as f32)))
        .collect();
    run(&mut s, batch);
    run(&mut s, vec![d.stl("bite3.stl", &arch(33.0))]); // bite limit, replaced
    let batch = (4..=6)
        .map(|i| d.stl(&format!("bite{i}.stl"), &arch(30.0 + i as f32)))
        .collect();
    run(&mut s, batch); // bite limit, skipped and replaced
    let mut s = CaseState::new(Case::new("T"));
    run(&mut s, vec![d.stl("bite_cm.stl", &arch(15.0))]); // units suspect

    // arch conflict
    let mut s = CaseState::new(Case::new("T"));
    import(&mut s, &[d.stl("upper.stl", &arch(60.0))]);
    let mut detect = |_: &dc_case::import::PreparedScan| {
        Some(Detection {
            arch: Upper,
            confidence: 0.9,
        })
    };
    let resolver: &mut Resolver<'_> = &mut detect;
    let request = ImportRequest::from_paths([d.stl("scan.stl", &arch(61.0))]);
    let report = import_files(&mut s, &request, Some(resolver), &JobCtx::detached());
    out.extend(report.warnings);
    out
}

fn text_of<'a>(warnings: &'a [(&'static str, String)], key: &str) -> &'a str {
    warnings
        .iter()
        .find(|(k, _)| *k == key)
        .map_or_else(|| panic!("no warning {key}"), |(_, m)| m.as_str())
}

/// The language is process-wide: this is the only test that switches it, and
/// the only one that reads translated text.
#[test]
fn warnings_and_errors_read_well_in_both_languages() {
    let d = Dir::new();
    let warnings = every_warning(&d);
    for code in [
        WarningCode::Rescaled,
        WarningCode::UnitsSuspect,
        WarningCode::Tiny,
        WarningCode::LowPolygons,
        WarningCode::ObjObjectsMerged,
        WarningCode::BadFacesRemoved,
        WarningCode::Replaced,
        WarningCode::ArchConflict,
        WarningCode::Provisional,
        WarningCode::BiteLimit,
    ] {
        assert!(warnings.iter().any(|w| w.code == code), "{code:?}");
    }
    let messages = |lang| {
        dc_i18n::set_language(lang).unwrap();
        warnings
            .iter()
            .map(|w| (w.key, w.message()))
            .collect::<Vec<_>>()
    };
    let en = messages(Lang::En);
    let it = messages(Lang::It);

    let want: &[(&str, &str, &str)] = &[
        (
            "case-import-warn-rescaled",
            "m.stl: rescaled × 1000 to millimetres (the file was in m).",
            "m.stl: riscalato × 1000 in millimetri (il file era in m).",
        ),
        (
            "case-import-warn-units-suspect",
            "bite_cm.stl: the size looks like cm, but bite and follow-up scans are never rescaled: it was read as millimetres. Check the file.",
            "bite_cm.stl: le dimensioni fanno pensare a cm, ma le scansioni del morso e di controllo non vengono mai riscalate: è stato letto in millimetri. Verifica il file.",
        ),
        (
            "case-import-warn-tiny",
            "t.stl: the model measures only 1.50 mm. Check that the file is in millimetres.",
            "t.stl: il modello misura solo 1.50 mm. Verifica che il file sia in millimetri.",
        ),
        (
            "case-import-warn-low-polygons",
            "few.stl: only 32 faces, too few for a clinical scan. Check that this is the right file.",
            "few.stl: solo 32 facce, troppo poche per una scansione clinica. Verifica che sia il file giusto.",
        ),
        (
            "case-import-warn-obj-objects-merged",
            "o.obj: the 2 objects of the OBJ file were merged into one scan.",
            "o.obj: i 2 oggetti del file OBJ sono stati uniti in un'unica scansione.",
        ),
        (
            "case-import-warn-bad-faces-removed",
            "o.obj: faces removed: 1 degenerate, 1 duplicate.",
            "o.obj: facce rimosse: 1 degeneri, 1 duplicate.",
        ),
        (
            "case-import-warn-replaced",
            "upper2.stl replaces upper.stl as upper arch.",
            "upper2.stl sostituisce upper.stl come arcata superiore.",
        ),
        (
            "case-import-warn-arch-conflict",
            "The scan scan.stl looks like the other arch, but that place is taken: it was placed as lower arch. Arch conflict, please check.",
            "La scansione scan.stl sembra l'altra arcata, ma quel posto è già occupato: è stata assegnata come arcata inferiore. Conflitto di arcata, verifica.",
        ),
        (
            "case-import-warn-provisional",
            "m.stl: the arch could not be recognised, so it is placed as lower arch for now. Please check.",
            "m.stl: non è stato possibile riconoscere l'arcata, quindi per ora è assegnata come arcata inferiore. Verifica.",
        ),
        (
            "case-import-warn-provisional-moved",
            "o.obj was placed provisionally: it is now upper arch, to make room for the new scan.",
            "o.obj era assegnata in modo provvisorio: ora è arcata superiore, per fare posto alla nuova scansione.",
        ),
        (
            "case-import-warn-bite-limit-replaced",
            "A case holds at most 2 bite scans: bite3.stl replaces bite2.stl.",
            "Un caso può avere al massimo 2 scansioni del morso: bite3.stl sostituisce bite2.stl.",
        ),
        (
            "case-import-warn-bite-limit-skipped",
            "A case holds at most 2 bite scans: bite6.stl was not imported.",
            "Un caso può avere al massimo 2 scansioni del morso: bite6.stl non è stata importata.",
        ),
    ];
    for (key, english, italian) in want {
        assert_eq!(text_of(&en, key), *english);
        assert_eq!(text_of(&it, key), *italian);
    }
    for (key, message) in en.iter().chain(&it) {
        assert!(
            !message.contains('{') && !message.contains("case-import"),
            "{key}: {message}"
        );
    }

    // JSON and toasts share the text
    dc_i18n::set_language(Lang::It).unwrap();
    let json = serde_json::to_value(&warnings[0]).unwrap();
    assert_eq!(json["code"], "rescaled");
    assert_eq!(json["key"], "case-import-warn-rescaled");
    assert_eq!(json["args"]["factor"], "1000");
    assert_eq!(json["args"]["units"], "m");
    assert_eq!(json["message"], warnings[0].message());

    let name = String::from("a.stl");
    let errors: &[(ImportError, &str, &str)] = &[
        (
            ImportError::Empty { name: name.clone() },
            "a.stl: the file contains no usable faces.",
            "a.stl: il file non contiene facce utilizzabili.",
        ),
        (
            ImportError::NeedsRole { name: name.clone() },
            "a.stl: both arches are already taken. Choose its role in the Import scans panel, under Role of the next scans, and import it again (a command takes it in the role parameter).",
            "a.stl: entrambe le arcate sono già occupate. Scegli il ruolo nel pannello Importa scansioni, sotto Ruolo delle prossime scansioni, e importa di nuovo il file (in un comando va nel parametro role).",
        ),
        (
            ImportError::RoleClash {
                name: name.clone(),
                role: Upper,
            },
            "a.stl: more than one file of this import is meant as upper arch. Say which file goes there.",
            "a.stl: più di un file di questa importazione è indicato come arcata superiore. Indica quale file va assegnato.",
        ),
        (
            ImportError::ArchesLocked {
                module: "aligners".into(),
            },
            "This project holds data of the aligners module that this version cannot read, so the arches cannot be replaced, moved or removed: update DentalCraft. A scan can still go on a free arch, and the bite scans can change.",
            "Questo progetto contiene dati del modulo aligners che questa versione non sa leggere, quindi le arcate non si possono sostituire, spostare né rimuovere: aggiorna DentalCraft. Una scansione può comunque andare su un'arcata libera e le scansioni del morso possono cambiare.",
        ),
        (
            ImportError::Invalid {
                name: name.clone(),
                detail: "line 3: bad number".into(),
            },
            "a.stl: the file is damaged or is not a valid scan.",
            "a.stl: il file è danneggiato o non è una scansione valida.",
        ),
        (
            ImportError::Unreadable {
                name: name.clone(),
                detail: "os error 2".into(),
            },
            "a.stl: the file cannot be read.",
            "a.stl: impossibile leggere il file.",
        ),
        (
            ImportError::Unsupported { name },
            "a.stl: unsupported file format. Use STL, OBJ or PLY.",
            "a.stl: formato di file non supportato. Usa STL, OBJ o PLY.",
        ),
        (
            ImportError::Cancelled,
            "Import cancelled.",
            "Importazione annullata.",
        ),
    ];
    for (error, english, italian) in errors {
        dc_i18n::set_language(Lang::En).unwrap();
        assert_eq!(error.to_string(), *english);
        dc_i18n::set_language(Lang::It).unwrap();
        assert_eq!(error.to_string(), *italian);
    }
    assert_eq!(
        dc_i18n::tr(dc_case::import::PROGRESS_KEY),
        "Lettura delle scansioni"
    );

    // a failure in the report carries the reason in the current language
    let mut state = CaseState::new(Case::new("T"));
    let report = import(&mut state, &[d.text("vuoto.obj", "v 0 0 0\n")]);
    assert_eq!(
        report.failed[0].reason,
        "vuoto.obj: il file non contiene facce utilizzabili."
    );
    let json = serde_json::to_value(&report).unwrap();
    assert_eq!(json["failed"][0]["code"], "empty");
    assert_eq!(json["cancelled"], false);

    dc_i18n::set_language(Lang::En).unwrap();
}

#[test]
fn a_path_is_reported_with_its_bare_file_name() {
    let d = Dir::new();
    let dir = d.path("upper_case");
    std::fs::create_dir(&dir).unwrap();
    let path: PathBuf = dir.join("scan.stl");
    dc_mesh::io::write(&path, &arch(60.0), MeshFormat::Stl).unwrap();
    let mut state = CaseState::new(Case::new("T"));
    let report = import(&mut state, std::slice::from_ref(&path));
    // the directory name is no hint
    assert_eq!(report.imported[0].role, Lower);
    assert_eq!(report.imported[0].source_name, "scan.stl");
    assert_eq!(report.imported[0].path, Path::new(&path));
}
