//! The synthetic part of the scan import differential study, in Rust: the
//! groups B (units), C (role from the file name), D (robustness) and E
//! (sequences) of `dentalcraft-aligners/golden/import/expected.json`, run on
//! generated meshes through [`prepare`] and [`import_files`]. Group A (one
//! mesh in six formats) is here as `every_format_gives_the_same_mesh`.
//!
//! The Blender add-on is the oracle. Where the pipeline differs from it, on
//! purpose, the test says so with `DIVERGES`:
//!
//! * Micrometres. An extent of 20 000 to 200 000 is taken as micrometres and
//!   scaled by 0.001. The add-on has no such rule: it keeps a 60 m model.
//! * The 0.02 and 0.2 boundaries. They are inclusive here, compared as f32.
//!   The add-on compares its float32 `dimensions` with double literals, so an
//!   extent of exactly f32(0.02) or f32(0.2) is not rescaled there.
//! * Corrupt files are rejected: a truncated binary STL, random bytes named
//!   `.stl`, a NaN or infinite coordinate, a face index out of range. The
//!   add-on imports several of them as empty or half-empty objects (and
//!   raises an uncaught error on others).
//! * Files with no usable face (an empty OBJ, an ASCII STL without facets, a
//!   PLY with no vertices, a mesh whose faces all collapse) are rejected with
//!   the code `empty`. The add-on turns some of them into an empty object.
//!   Because of that, an empty file named `upper` no longer replaces a good
//!   upper scan.
//! * A multi-object OBJ becomes one scan with the `ObjObjectsMerged` warning.
//!   The add-on takes the first object and leaves the others in the scene.
//! * Weld. STL corners are welded within 1e-6 of the bounding box diagonal
//!   (the add-on welds only bit-identical corners). A weld can collapse a
//!   triangle onto repeated indices, and the cleanup then removes it: on the
//!   demo arch of group A that gives 166390 faces where the add-on has
//!   166392 (two source vertices 7.4e-5 mm apart merge, 83171 vertices
//!   against 83172). The same cleanup removes the 50 repeated-index faces and
//!   the 300 duplicate faces of group D, as Blender's own importers do, so
//!   those rows now agree.
//! * Roles. Same words as the add-on, plus the bite words (`bite`, `morso`).
//!   With no hint the first free arch is taken, lower first, as provisional.
//!   One scan per role: importing a second scan of a role replaces the first
//!   (the add-on keeps every object as its own case).
//!
//! The meshes are generated here, so the numbers are exact. Rows already
//! covered one by one in `import_pipeline.rs` (the allocation rules, the
//! warning texts, the module hook) are not repeated.

use std::path::{Path, PathBuf};

use dc_case::import::{ImportReport, ImportRequest, WarningCode, import_files, prepare};
use dc_case::{Case, CaseState, RoleSource, ScanRole};
use dc_jobs::JobCtx;
use dc_mesh::{MeshFormat, TriMesh, Units};
use glam::Vec3;

use RoleSource::{FileName, Provisional};
use ScanRole::{Bite, Lower, Upper};

/// A flat, faceted square of `2 * cells * cells` faces and side `size`, with
/// a little relief so that no face is collinear.
fn grid(cells: u32, size: f32) -> TriMesh {
    let n = cells + 1;
    let step = size / cells as f32;
    let mut positions = Vec::new();
    for y in 0..n {
        for x in 0..n {
            let z = ((x + y) % 2) as f32 * step * 0.1;
            positions.push(Vec3::new(x as f32 * step, y as f32 * step, z));
        }
    }
    let mut tris = Vec::new();
    for y in 0..cells {
        for x in 0..cells {
            let i = y * n + x;
            tris.push([i, i + 1, i + n]);
            tris.push([i + 1, i + n + 1, i + n]);
        }
    }
    TriMesh::new(positions, tris)
}

/// The arch-sized model most tests import: 2 * 24 * 24 = 1152 faces.
fn arch() -> TriMesh {
    grid(24, 60.0)
}

/// Two triangles whose largest extent is exactly `extent` (the x axis), with
/// the minimum at 0. Written as f32, so the boundaries are the f32 ones.
fn sized(extent: f32) -> TriMesh {
    let h = extent * 0.22;
    TriMesh::new(
        vec![
            Vec3::ZERO,
            Vec3::new(extent, 0.0, 0.0),
            Vec3::new(extent, h, 0.0),
            Vec3::new(0.0, h, 0.0),
        ],
        vec![[0, 1, 2], [0, 2, 3]],
    )
}

struct Dir(tempfile::TempDir);

impl Dir {
    fn new() -> Self {
        Self(tempfile::tempdir().unwrap())
    }

    fn path(&self, name: &str) -> PathBuf {
        self.0.path().join(name)
    }

    /// Writes `mesh` in the format of the extension of `name`.
    fn mesh(&self, name: &str, mesh: &TriMesh) -> PathBuf {
        let path = self.path(name);
        let format = MeshFormat::from_path(&path).unwrap();
        dc_mesh::io::write(&path, mesh, format).unwrap();
        path
    }

    fn bytes(&self, name: &str, bytes: &[u8]) -> PathBuf {
        let path = self.path(name);
        std::fs::write(&path, bytes).unwrap();
        path
    }

    fn text(&self, name: &str, text: &str) -> PathBuf {
        self.bytes(name, text.as_bytes())
    }
}

fn state() -> CaseState {
    CaseState::new(Case::new("T"))
}

fn import(state: &mut CaseState, paths: &[PathBuf]) -> ImportReport {
    let request = ImportRequest::from_paths(paths.iter().cloned());
    import_files(state, &request, None, &JobCtx::detached())
}

fn codes(report: &ImportReport) -> Vec<WarningCode> {
    report.warnings.iter().map(|w| w.code).collect()
}

fn failed_codes(report: &ImportReport) -> Vec<&'static str> {
    report.failed.iter().map(|f| f.code).collect()
}

fn roles(state: &CaseState) -> Vec<(ScanRole, RoleSource)> {
    state
        .case
        .scans
        .iter()
        .map(|s| (s.role, s.role_source))
        .collect()
}

// --- B: units ---------------------------------------------------------------

/// Group B, row by row: the largest extent as written in the STL, the unit
/// the pipeline detects and the size in millimetres afterwards.
#[test]
fn b_every_extent_of_the_unit_table() {
    use Units::{Centimeters as Cm, Meters as M, Micrometers as Um, Millimeters as Mm};
    // (extent in the file, unit, DIVERGES from the add-on)
    let rows: &[(f32, Units, bool)] = &[
        (0.06, M, false),
        (6.0, Cm, false),
        (60.0, Mm, false),
        (60_000.0, Um, true), // the add-on keeps 60 m
        (2.4, Cm, false),     // "inch" scans are read as centimetres on both sides
        (0.02, M, true),      // f32 boundary, inclusive here
        (0.2, M, true),       // f32 boundary, inclusive here
        (2.0, Cm, false),
        (20.0, Cm, false),
        (15.0, Cm, false),
        (3.0, Cm, false),
        (0.019, Mm, false),
        (0.21, Mm, false),
        (1.9, Mm, false),
        (21.0, Mm, false),
        (100.0, Mm, false),
        (19_999.0, Mm, false),
        (20_000.0, Um, true), // micrometre boundary
        (200_000.0, Um, true),
        (200_001.0, Mm, false),
        (0.0199, Mm, false),
        (2.0001, Cm, false),
    ];
    let d = Dir::new();
    let ctx = JobCtx::detached();
    for (i, &(extent, units, _diverges)) in rows.iter().enumerate() {
        let path = d.mesh(&format!("u{i}.stl"), &sized(extent));
        let scan = prepare(&path, &ctx).unwrap();
        assert_eq!(scan.source_units, units, "extent {extent}");
        let mm = extent * units.to_mm();
        let got = scan.mesh.aabb().unwrap().size().max_element();
        assert!((got - mm).abs() <= mm * 1e-5, "extent {extent}: {got} mm");
        assert_eq!(scan.mesh.aabb().unwrap().min, Vec3::ZERO, "extent {extent}");
        let rescaled = codes_of(&scan).contains(&WarningCode::Rescaled);
        assert_eq!(rescaled, units != Mm, "extent {extent}");
    }
}

fn codes_of(scan: &dc_case::import::PreparedScan) -> Vec<WarningCode> {
    scan.warnings.iter().map(|w| w.code).collect()
}

// --- C: role from the file name ---------------------------------------------

/// Group C through the whole import: the name decides the role (and says so),
/// a name without a hint takes the first free arch, lower, as provisional.
/// The names are those of the study; a name with an upper and a lower word
/// (or a longer word that only contains one) gives no hint.
#[test]
fn c_the_name_decides_the_role_in_the_pipeline() {
    let rows: &[(&str, Option<ScanRole>)] = &[
        ("upper.stl", Some(Upper)),
        ("lower.stl", Some(Lower)),
        ("maxilla.stl", Some(Upper)),
        ("mandible.stl", Some(Lower)),
        ("mascellare.stl", Some(Upper)),
        ("mandibolare.stl", Some(Lower)),
        ("superiore.stl", Some(Upper)),
        ("inferiore.stl", Some(Lower)),
        ("sup.stl", Some(Upper)),
        ("inf.stl", Some(Lower)),
        ("Patient_sup.stl", Some(Upper)),
        ("Oberkiefer.stl", Some(Upper)),
        ("Unterkiefer.stl", Some(Lower)),
        ("UpperJaw.stl", Some(Upper)),
        ("lowerJaw.stl", Some(Lower)),
        ("UPPER.STL", Some(Upper)),
        ("maxillary.stl", Some(Upper)),
        ("mandibular.stl", Some(Lower)),
        ("Arcata_Superiore.stl", Some(Upper)),
        ("arcata-inferiore.stl", Some(Lower)),
        ("inferior.stl", Some(Lower)),
        ("superior.stl", Some(Upper)),
        ("maxill.stl", Some(Upper)),
        ("mandib.stl", Some(Lower)),
        ("Lower.ply", Some(Lower)),
        ("case_UPPER_final.obj", Some(Upper)),
        ("p123_Maxillary_Scan.stl", Some(Upper)),
        ("12_upper.stl", Some(Upper)),
        ("UpperJawScan.stl", Some(Upper)),
        ("LowerJawScan.stl", Some(Lower)),
        ("LOWER.STL", Some(Lower)),
        // DIVERGES: the add-on has no bite role (it would call these lower)
        ("bite.stl", Some(Bite)),
        ("morso.stl", Some(Bite)),
        ("UpperJaw_bite.stl", Some(Bite)),
        // a longer word only containing a hint, or two hints: no hint
        ("flower.stl", None),
        ("superman.stl", None),
        ("scan1.stl", None),
        ("scan_upper_lower.stl", None),
        ("upperlower.stl", None),
        ("upper-lower.stl", None),
        ("Mandible_Upper.stl", None),
        ("unterkiefer_oberkiefer.stl", None),
        ("superioreinferiore.stl", None),
        ("lowercase.stl", None),
        ("uppercase.stl", None),
        ("supply.stl", None),
        ("info.stl", None),
        ("insup.stl", None),
        ("Max.stl", None),
        ("MAN.stl", None),
        ("bitescan.stl", None),
        ("occlusion.stl", None),
    ];
    let mesh = arch();
    for &(name, hint) in rows {
        let d = Dir::new();
        let mut s = state();
        let report = import(&mut s, &[d.mesh(name, &mesh)]);
        assert!(
            report.failed.is_empty(),
            "{name}: {:?}",
            failed_codes(&report)
        );
        let got = &report.imported[0];
        match hint {
            Some(role) => assert_eq!((got.role, got.role_source), (role, FileName), "{name}"),
            // the first free arch, lower first
            None => assert_eq!((got.role, got.role_source), (Lower, Provisional), "{name}"),
        }
        assert_eq!(got.source_name, name);
    }
}

#[test]
fn c_the_directory_never_counts() {
    let d = Dir::new();
    for dir in ["upper_case", "lower_dir"] {
        std::fs::create_dir(d.path(dir)).unwrap();
        let path = d.mesh(&format!("{dir}/scan.stl"), &arch());
        let mut s = state();
        let report = import(&mut s, &[path]);
        let got = &report.imported[0];
        assert_eq!((got.role, got.role_source), (Lower, Provisional), "{dir}");
        assert_eq!(got.source_name, "scan.stl");
    }
}

/// Four files without a hint in one batch: they fill the free arches, lower
/// then upper, and the rest find no place (never a replacement, never bite).
/// The add-on makes four separate lower cases of them.
#[test]
fn c_four_unhinted_files_fill_two_arches_and_two_find_no_place() {
    let d = Dir::new();
    let paths: Vec<PathBuf> = (1..=4)
        .map(|i| d.mesh(&format!("scan{i}.stl"), &arch()))
        .collect();
    let mut s = state();
    let report = import(&mut s, &paths);
    let got: Vec<_> = report
        .imported
        .iter()
        .map(|i| (i.source_name.as_str(), i.role, i.role_source))
        .collect();
    assert_eq!(
        got,
        [
            ("scan1.stl", Lower, Provisional),
            ("scan2.stl", Upper, Provisional)
        ]
    );
    assert_eq!(failed_codes(&report), ["needs_role", "needs_role"]);
    assert_eq!(s.case.scans.len(), 2);
}

/// `scan1`, `x_upper`, `scan2`: the named file keeps its arch, the first
/// unhinted one takes the free one, the last finds no place.
#[test]
fn c_a_named_file_between_two_unhinted_ones() {
    let d = Dir::new();
    let paths = [
        d.mesh("scan1.stl", &arch()),
        d.mesh("x_upper.stl", &arch()),
        d.mesh("scan2.stl", &arch()),
    ];
    let mut s = state();
    let report = import(&mut s, &paths);
    let by_name = |n: &str| {
        let i = report.imported.iter().find(|i| i.source_name == n).unwrap();
        (i.role, i.role_source)
    };
    assert_eq!(by_name("x_upper.stl"), (Upper, FileName));
    assert_eq!(by_name("scan1.stl"), (Lower, Provisional));
    assert_eq!(failed_codes(&report), ["needs_role"]);
    assert!(report.failed[0].path.ends_with("scan2.stl"));
}

// --- A and D: formats and file variants -------------------------------------

/// Counts and the bounding box of an imported mesh: (vertices, faces, min, max).
fn shape(path: &Path) -> (usize, usize, Vec3, Vec3) {
    let scan = prepare(path, &JobCtx::detached()).unwrap();
    let b = scan.mesh.aabb().unwrap();
    (
        scan.mesh.positions.len(),
        scan.mesh.face_count(),
        b.min,
        b.max,
    )
}

fn ply_ascii(mesh: &TriMesh) -> String {
    let mut s = format!(
        "ply\nformat ascii 1.0\nelement vertex {}\nproperty float x\nproperty float y\nproperty float z\nelement face {}\nproperty list uchar int vertex_indices\nend_header\n",
        mesh.positions.len(),
        mesh.tris.len()
    );
    for p in &mesh.positions {
        s += &format!("{} {} {}\n", p.x, p.y, p.z);
    }
    for t in &mesh.tris {
        s += &format!("3 {} {} {}\n", t[0], t[1], t[2]);
    }
    s
}

fn ply_big_endian(mesh: &TriMesh) -> Vec<u8> {
    let mut b = format!(
        "ply\nformat binary_big_endian 1.0\nelement vertex {}\nproperty float x\nproperty float y\nproperty float z\nelement face {}\nproperty list uchar uint vertex_indices\nend_header\n",
        mesh.positions.len(),
        mesh.tris.len()
    )
    .into_bytes();
    for p in &mesh.positions {
        for c in [p.x, p.y, p.z] {
            b.extend_from_slice(&c.to_be_bytes());
        }
    }
    for t in &mesh.tris {
        b.push(3);
        for i in t {
            b.extend_from_slice(&i.to_be_bytes());
        }
    }
    b
}

/// Group A: the same mesh as binary STL, ASCII STL, OBJ and three kinds of
/// PLY gives the same vertices and faces; STL and PLY keep the coordinates,
/// OBJ turns Y-up into Z-up (as the add-on's world does).
#[test]
fn every_format_gives_the_same_mesh() {
    let d = Dir::new();
    let mesh = grid(30, 60.0);
    let (v, f) = (mesh.positions.len(), mesh.face_count());
    let want = shape(&d.mesh("a.stl", &mesh));
    assert_eq!(
        (want.0, want.1),
        (v, f),
        "binary STL welds to the source mesh"
    );
    let ascii = d.path("b.stl");
    dc_mesh::io::write_stl_ascii(&ascii, &mesh).unwrap();
    let same = [
        ascii,
        d.mesh("c.ply", &mesh),
        d.text("d.ply", &ply_ascii(&mesh)),
        d.bytes("e.ply", &ply_big_endian(&mesh)),
    ];
    for path in &same {
        assert_eq!(shape(path), want, "{}", path.display());
    }
    // OBJ: (x, y, z) -> (x, -z, y)
    let (ov, of, min, max) = shape(&d.mesh("f.obj", &mesh));
    assert_eq!((ov, of), (v, f));
    let (a, b) = (want.2, want.3);
    assert_eq!(min, Vec3::new(a.x, -b.z, a.y));
    assert_eq!(max, Vec3::new(b.x, -a.z, b.y));
}

/// The letter case of the extension does not matter (the add-on agrees).
#[test]
fn d_extensions_in_any_letter_case() {
    let d = Dir::new();
    let mesh = arch();
    for (lower, name) in [
        ("l.stl", "scan.STL"),
        ("l.obj", "scan.OBJ"),
        ("l.ply", "scan.Ply"),
        ("m.stl", "scan.sTl"),
    ] {
        let want = shape(&d.mesh(lower, &mesh));
        std::fs::copy(d.path(lower), d.path(name)).unwrap();
        assert_eq!(shape(&d.path(name)), want, "{name}");
    }
}

/// A binary STL whose header starts with `solid`, an ASCII STL with CRLF line
/// ends, a small mesh under the low polygon limit: all imported.
#[test]
fn d_stl_variants_are_read() {
    let d = Dir::new();
    let mesh = arch();
    let (v, f, ..) = shape(&d.mesh("plain.stl", &mesh));
    // binary with a text-looking header
    let mut bytes = std::fs::read(d.path("plain.stl")).unwrap();
    bytes[..26].copy_from_slice(b"solid exported by scanner ");
    let got = shape(&d.bytes("solidhdr.stl", &bytes));
    assert_eq!((got.0, got.1), (v, f));
    // ASCII with CRLF
    dc_mesh::io::write_stl_ascii(d.path("lf.stl"), &mesh).unwrap();
    let crlf = String::from_utf8(std::fs::read(d.path("lf.stl")).unwrap())
        .unwrap()
        .replace('\n', "\r\n");
    let got = shape(&d.text("crlf.stl", &crlf));
    assert_eq!((got.0, got.1), (v, f));
    // under 1000 faces: a warning, not a rejection
    let small = grid(10, 60.0);
    let scan = prepare(&d.mesh("small.stl", &small), &JobCtx::detached()).unwrap();
    assert_eq!(scan.mesh.face_count(), 200);
    assert_eq!(codes_of(&scan), [WarningCode::LowPolygons]);
}

/// OBJ with quads and `v/vt/vn` indices, negative indices, and `v x y z r g b`
/// vertex colours; PLY with colours and normals.
#[test]
fn d_obj_and_ply_variants_are_read() {
    let d = Dir::new();
    let n = 21u32; // 20 x 20 quads
    let mut verts = String::new();
    for y in 0..n {
        for x in 0..n {
            verts += &format!(
                "v {} {} {}\n",
                x as f32 * 3.0,
                y as f32 * 3.0,
                (x % 2) as f32
            );
        }
    }
    let at = |x: u32, y: u32| y * n + x + 1;
    // quads with v/vt/vn
    let mut quads = String::from("vt 0 0\nvn 0 0 1\n");
    for y in 0..n - 1 {
        for x in 0..n - 1 {
            let q = [at(x, y), at(x + 1, y), at(x + 1, y + 1), at(x, y + 1)];
            quads += &format!("f {}/1/1 {}/1/1 {}/1/1 {}/1/1\n", q[0], q[1], q[2], q[3]);
        }
    }
    let got = shape(&d.text("quads.obj", &format!("{verts}{quads}")));
    assert_eq!((got.0, got.1), (441, 2 * 400), "a quad is two triangles");
    // negative (relative) indices: each face refers back from the end
    let total = (n * n) as i64;
    let mut rel = String::new();
    for y in 0..n - 1 {
        for x in 0..n - 1 {
            let q = [at(x, y), at(x + 1, y), at(x + 1, y + 1)];
            rel += &format!(
                "f {} {} {}\n",
                q[0] as i64 - total - 1,
                q[1] as i64 - total - 1,
                q[2] as i64 - total - 1
            );
        }
    }
    let got = shape(&d.text("rel.obj", &format!("{verts}{rel}")));
    assert_eq!((got.0, got.1), (441, 400));
    // vertex colours after the coordinates
    let coloured = verts.replace('\n', " 0.5 0.25 0.75\n");
    let got = shape(&d.text("col.obj", &format!("{coloured}{rel}")));
    assert_eq!((got.0, got.1), (441, 400));
    // PLY with colours and normals (the writer keeps them)
    let mut mesh = grid(20, 60.0);
    mesh.normals = Some(vec![Vec3::Z; mesh.positions.len()]);
    let got = shape(&d.mesh("nc.ply", &mesh));
    assert_eq!((got.0, got.1), (441, 800));
}

// --- D: files the import refuses --------------------------------------------

/// Group D, the files with nothing to import and the corrupt ones: each is
/// refused with its own code and imports nothing.
/// DIVERGES: the add-on turns most of them into an empty object or an uncaught
/// error; it also ACCEPTS a truncated STL, garbage named `.stl` and a NaN.
#[test]
fn d_unusable_files_are_refused_with_a_code() {
    let d = Dir::new();
    let mesh = arch();
    // a valid binary STL to cut
    let full = {
        let p = d.mesh("full.stl", &mesh);
        std::fs::read(p).unwrap()
    };
    let faces = mesh.face_count();
    let mut noise = vec![0u8; 4096];
    let mut x = 0x9e37_79b9_u32;
    for b in &mut noise {
        x = x.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        *b = (x >> 24) as u8;
    }
    // the header announces `faces` triangles
    let header_triangles = |bytes: &[u8]| u32::from_le_bytes(bytes[80..84].try_into().unwrap());
    assert_eq!(header_triangles(&full) as usize, faces);

    let ply_header = |v: usize| {
        format!(
            "ply\nformat ascii 1.0\nelement vertex {v}\nproperty float x\nproperty float y\nproperty float z\nend_header\n"
        )
    };
    let mut binary_zero = b"synthetic".to_vec();
    binary_zero.resize(80, b' ');
    binary_zero.extend_from_slice(&0u32.to_le_bytes());

    let cases: Vec<(String, Vec<u8>, &str)> = vec![
        // no bytes at all
        ("empty0.stl".into(), vec![], "invalid"),
        ("empty0.obj".into(), vec![], "empty"),
        ("empty0.ply".into(), vec![], "invalid"),
        // no faces
        ("tri0_bin.stl".into(), binary_zero, "empty"),
        (
            "solid_empty.stl".into(),
            b"solid x\nendsolid x\n".to_vec(),
            "empty",
        ),
        (
            "noface.obj".into(),
            b"# no faces\nv 0 0 0\nv 1 0 0\nv 0 1 0\n".to_vec(),
            "empty",
        ),
        ("empty_comment.obj".into(), b"# nothing\n".to_vec(), "empty"),
        (
            "face0.ply".into(),
            (ply_header(3) + "0 0 0\n1 0 0\n0 1 0\n").into_bytes(),
            "empty",
        ),
        ("vert0.ply".into(), ply_header(0).into_bytes(), "empty"),
        // truncated binary STL: mid-triangle, on a triangle boundary, inside the header
        (
            "trunc_mid.stl".into(),
            full[..84 + 50 * (faces / 2) + 25].to_vec(),
            "invalid",
        ),
        (
            "trunc_aligned.stl".into(),
            full[..84 + 50 * (faces / 2)].to_vec(),
            "invalid",
        ),
        ("trunc_header.stl".into(), full[..60].to_vec(), "invalid"),
        // garbage under each extension
        ("garbage.stl".into(), noise.clone(), "invalid"),
        (
            "garbage_text.stl".into(),
            b"this is not a mesh\nhello world\n".repeat(20),
            "invalid",
        ),
        ("garbage.ply".into(), noise.clone(), "invalid"),
        // not coordinates, indices out of range
        (
            "nan.obj".into(),
            b"v nan 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n".to_vec(),
            "invalid",
        ),
        (
            "inf.obj".into(),
            b"v inf 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n".to_vec(),
            "invalid",
        ),
        (
            "badidx.obj".into(),
            b"v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 9\n".to_vec(),
            "invalid",
        ),
        // not a mesh extension, valid content or not
        ("valid_as_txt.txt".into(), full.clone(), "unsupported"),
        ("model.3mf".into(), b"PK\x03\x04".to_vec(), "unsupported"),
        ("noext".into(), full.clone(), "unsupported"),
    ];
    let mut s = state();
    for (name, bytes, code) in &cases {
        let path = d.bytes(name, bytes);
        let e = prepare(&path, &JobCtx::detached()).unwrap_err();
        assert_eq!(e.code(), *code, "{name}: {e:?}");
        let report = import(&mut s, &[path]);
        assert!(report.imported.is_empty(), "{name}");
        assert_eq!(failed_codes(&report), [*code], "{name}");
    }
    assert!(s.case.scans.is_empty() && s.blobs.is_empty());
    // NaN in a binary STL
    let mut bad = mesh.clone();
    bad.positions[10].x = f32::NAN;
    let path = d.mesh("nan.stl", &bad);
    assert_eq!(
        prepare(&path, &JobCtx::detached()).unwrap_err().code(),
        "invalid"
    );
}

// --- D: cleanup, weld, objects ----------------------------------------------

/// 50 faces with a repeated index and 18 collinear ones with distinct
/// indices, over 20 isolated vertices: the repeated-index faces go, the
/// collinear ones and the isolated vertices stay, in OBJ and PLY. DIVERGES
/// from the files as read: the add-on's importers also drop the 50 (4738
/// against 4788 faces), the pipeline now agrees with it.
#[test]
fn d_degenerate_faces_are_removed_collinear_ones_stay() {
    let d = Dir::new();
    let base = grid(48, 60.0); // 4608 faces, 2401 vertices
    let nv = base.positions.len() as u32;
    let mut mesh = base.clone();
    for i in 0..20 {
        mesh.positions
            .push(Vec3::new(100.0 + i as f32, 100.0, 100.0));
    }
    for i in 0..50u32 {
        mesh.tris.push([i, i, i + 1]);
    }
    for i in 0..18u32 {
        mesh.tris.push([nv + i, nv + i + 1, nv + i + 2]);
    }
    for name in ["degen.obj", "degen.ply"] {
        let path = d.path(name);
        match name.rsplit('.').next().unwrap() {
            "obj" => {
                let mut text = String::new();
                for p in &mesh.positions {
                    text += &format!("v {} {} {}\n", p.x, p.y, p.z);
                }
                for t in &mesh.tris {
                    text += &format!("f {} {} {}\n", t[0] + 1, t[1] + 1, t[2] + 1);
                }
                std::fs::write(&path, text).unwrap();
            }
            _ => dc_mesh::io::write(&path, &mesh, MeshFormat::Ply).unwrap(),
        }
        let scan = prepare(&path, &JobCtx::detached()).unwrap();
        assert_eq!(scan.cleanup.degenerate, 50, "{name}");
        assert_eq!(scan.cleanup.duplicate, 0, "{name}");
        assert_eq!(scan.mesh.face_count(), 4608 + 18, "{name}");
        assert_eq!(scan.mesh.positions.len(), 2401 + 20, "{name}");
        assert!(codes_of(&scan).contains(&WarningCode::BadFacesRemoved));
    }
}

/// 200 identical faces and 100 reversed ones: the first copy stays, whatever
/// its winding, in all three formats (the STL repeats the triangles in the
/// soup, the weld gives them the same indices).
#[test]
fn d_duplicate_and_reversed_faces_are_removed() {
    let d = Dir::new();
    let mut mesh = grid(48, 60.0);
    let base = mesh.tris.clone();
    mesh.tris.extend_from_slice(&base[..200]);
    mesh.tris
        .extend(base[200..300].iter().map(|t| [t[2], t[1], t[0]]));
    for name in ["dup.stl", "dup.obj", "dup.ply"] {
        let path = d.mesh(name, &mesh);
        let scan = prepare(&path, &JobCtx::detached()).unwrap();
        assert_eq!(scan.cleanup.duplicate, 300, "{name}");
        assert_eq!(scan.cleanup.degenerate, 0, "{name}");
        assert_eq!(scan.mesh.face_count(), 4608, "{name}");
    }
}

/// The weld takes bit-identical corners always, and corners within 1e-6 of
/// the bounding box diagonal. DIVERGES: the add-on welds only bit-identical
/// ones, so a triangle of that size stays there and collapses here, and the
/// cleanup removes it (the 166390 against 166392 faces of group A).
#[test]
fn d_the_stl_weld_and_the_faces_it_collapses() {
    let d = Dir::new();
    let mesh = grid(48, 60.0);
    // a soup: every face with its own three corners
    let soup = |extra: &[[Vec3; 3]], jitter: f32| {
        let mut positions = Vec::new();
        let mut state = 7u32;
        let mut noise = || {
            state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            ((state >> 8) as f32 / (1u32 << 24) as f32 * 2.0 - 1.0) * jitter
        };
        for t in &mesh.tris {
            for &i in t {
                let p = mesh.positions[i as usize];
                positions.push(p + Vec3::new(noise(), noise(), noise()));
            }
        }
        for t in extra {
            positions.extend_from_slice(t);
        }
        let tris = (0..positions.len() as u32 / 3)
            .map(|i| [3 * i, 3 * i + 1, 3 * i + 2])
            .collect();
        TriMesh::new(positions, tris)
    };
    // bit-identical corners weld to the source mesh
    let got = shape(&d.mesh("exact.stl", &soup(&[], 0.0)));
    assert_eq!((got.0, got.1), (2401, 4608));

    // one triangle a hundred times smaller than the weld tolerance
    // (1e-6 * 85 mm), far from the grid: its corners merge, the face goes
    let tiny = [
        Vec3::new(30.0, 30.0, 50.0),
        Vec3::new(30.0 + 1e-7, 30.0, 50.0),
        Vec3::new(30.0, 30.0 + 1e-7, 50.0),
    ];
    let path = d.mesh("tiny.stl", &soup(&[tiny], 0.0));
    let scan = prepare(&path, &JobCtx::detached()).unwrap();
    assert_eq!(scan.cleanup.degenerate, 1);
    assert_eq!(scan.mesh.face_count(), 4608);
    assert_eq!(scan.mesh.positions.len(), 2401 + 1);
    assert!(codes_of(&scan).contains(&WarningCode::BadFacesRemoved));

    // corners a thousand times farther apart than the tolerance do not merge
    // (bar a coincidence): the mesh stays a soup
    let apart = shape(&d.mesh("apart.stl", &soup(&[], 1e-2)));
    assert_eq!(apart.1, 4608);
    assert!(apart.0 > 3 * 4608 - 20, "{} vertices", apart.0);
}

/// An OBJ with two objects is one scan, with the merge warning: both objects'
/// vertices and faces are there and the box spans both. DIVERGES: the add-on
/// takes the first object only (and leaves the other in the scene).
#[test]
fn d_a_multi_object_obj_is_merged_with_a_warning() {
    let d = Dir::new();
    let one = grid(20, 60.0); // 441 vertices, 800 faces
    let mut text = String::new();
    for (k, name) in ["a", "b"].iter().enumerate() {
        text += &format!("o {name}\n");
        for p in &one.positions {
            text += &format!("v {} {} {}\n", p.x + 100.0 * k as f32, p.y, p.z);
        }
        let off = k * one.positions.len() + 1;
        for t in &one.tris {
            text += &format!(
                "f {} {} {}\n",
                t[0] as usize + off,
                t[1] as usize + off,
                t[2] as usize + off
            );
        }
    }
    let path = d.text("two.obj", &text);
    let scan = prepare(&path, &JobCtx::detached()).unwrap();
    assert_eq!(scan.mesh.positions.len(), 2 * 441);
    assert_eq!(scan.mesh.face_count(), 2 * 800);
    let b = scan.mesh.aabb().unwrap();
    assert!(
        (b.max.x - 160.0).abs() < 1e-3 && b.min.x.abs() < 1e-3,
        "{b:?}"
    );
    assert!(codes_of(&scan).contains(&WarningCode::ObjObjectsMerged));
    // and as an import: one scan
    let mut s = state();
    let report = import(&mut s, &[path]);
    assert_eq!(report.imported.len(), 1);
    assert!(codes(&report).contains(&WarningCode::ObjObjectsMerged));
}

// --- E: sequences -----------------------------------------------------------

/// Two scans of the same role, imported one after the other (as `case.import`
/// does for each file of a CLI run): the later one replaces the earlier, the
/// old blob is dropped and the report says so. The same file twice is the
/// same case. DIVERGES: the add-on keeps every import as its own case.
#[test]
fn e_the_later_scan_of_a_role_replaces_the_earlier() {
    let d = Dir::new();
    let (a, b) = (
        d.mesh("a_upper.stl", &grid(24, 60.0)),
        d.mesh("b_upper.stl", &grid(20, 61.0)),
    );
    let mut s = state();
    import(&mut s, std::slice::from_ref(&a));
    let first = s.case.scan(Upper).unwrap().clone();
    let report = import(&mut s, std::slice::from_ref(&b));
    assert_eq!(report.replaced_roles, [Upper]);
    assert!(codes(&report).contains(&WarningCode::Replaced));
    assert_eq!(s.case.scans.len(), 1);
    let now = s.case.scan(Upper).unwrap();
    assert_ne!(now.id, first.id);
    assert_eq!(now.source_name, "b_upper.stl");
    assert_eq!(s.blobs.get(&now.mesh).unwrap().face_count(), 800);
    assert!(!s.blobs.contains(&first.mesh));
    assert_eq!(s.blobs.len(), 1);

    // the same file twice: a new scan with the same mesh, still one scan
    let mut s = state();
    import(&mut s, std::slice::from_ref(&a));
    let report = import(&mut s, std::slice::from_ref(&a));
    assert_eq!(report.replaced_roles, [Upper]);
    assert_eq!(s.case.scans.len(), 1);
    assert_eq!(s.blobs.len(), 1);
}

/// Upper, lower, upper again: the third replaces the first and the lower
/// scan stays as it was.
#[test]
fn e_upper_lower_upper() {
    let d = Dir::new();
    let upper = d.mesh("a_upper.stl", &grid(24, 60.0));
    let lower = d.mesh("c_lower.stl", &grid(20, 61.0));
    let upper2 = d.mesh("b_upper.stl", &grid(22, 62.0));
    let mut s = state();
    import(&mut s, &[upper, lower]);
    assert_eq!(roles(&s), [(Upper, FileName), (Lower, FileName)]);
    let lower_scan = s.case.scan(Lower).unwrap().clone();
    let report = import(&mut s, &[upper2]);
    assert_eq!(report.replaced_roles, [Upper]);
    assert_eq!(roles(&s), [(Upper, FileName), (Lower, FileName)]);
    assert_eq!(s.case.scan(Lower).unwrap(), &lower_scan);
    assert_eq!(s.case.scan(Upper).unwrap().source_name, "b_upper.stl");
    assert_eq!(s.blobs.len(), 2);
}

/// A good upper scan, then a file named upper that is empty, garbage or
/// truncated: the good scan stays, with its blob and the case untouched, and
/// the failure is reported. DIVERGES from the old import, where the empty
/// OBJ replaced the good scan; the add-on also keeps the first scan.
#[test]
fn e_a_bad_file_never_replaces_a_good_scan() {
    let d = Dir::new();
    let mesh = grid(24, 60.0);
    let good = d.mesh("a_upper.stl", &mesh);
    let full = std::fs::read(&good).unwrap();
    let bad = [
        ("upper_empty.obj", b"# nothing\n".to_vec(), "empty"),
        ("upper_garbage.obj", vec![0xff; 4096], "invalid"),
        ("upper_garbage.stl", vec![0xff; 4096], "invalid"),
        (
            "upper_trunc.stl",
            full[..84 + 50 * 500 + 25].to_vec(),
            "invalid",
        ),
    ];
    for (name, bytes, code) in bad {
        let mut s = state();
        import(&mut s, std::slice::from_ref(&good));
        let before = (s.case.scans.clone(), s.blobs.len());
        let report = import(&mut s, &[d.bytes(name, &bytes)]);
        assert!(report.imported.is_empty(), "{name}");
        assert_eq!(report.failed.len(), 1, "{name}");
        if name == "upper_garbage.obj" {
            // 0xff bytes are not text: the OBJ reader refuses them as invalid
            // or finds no face in them; either way nothing is imported
            let got = report.failed[0].code;
            assert!(got == "invalid" || got == "empty", "{name}: {got}");
        } else {
            assert_eq!(report.failed[0].code, code, "{name}");
        }
        assert!(report.replaced_roles.is_empty(), "{name}");
        assert_eq!((s.case.scans.clone(), s.blobs.len()), before, "{name}");
    }
}
