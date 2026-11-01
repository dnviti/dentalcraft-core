//! Round-trip, detection, robustness and unit tests for the mesh readers/writers.

use std::path::PathBuf;

use glam::Vec3;

use super::*;
use crate::test_meshes::{quad, tetra};

/// Deterministic xorshift generator (no external dependency).
struct Rng(u64);

impl Rng {
    fn next(&mut self) -> u64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        self.0
    }

    fn unit(&mut self) -> f32 {
        (self.next() >> 40) as f32 / (1u64 << 24) as f32
    }
}

/// Closed UV sphere with vertices in first-use order (so welding an STL of
/// it reproduces the same vertex numbering).
fn sphere(rings: u32, segs: u32, radius: f32) -> TriMesh {
    let mut positions = vec![Vec3::new(0.0, 0.0, radius)];
    for r in 1..rings {
        let phi = std::f32::consts::PI * r as f32 / rings as f32;
        for s in 0..segs {
            let th = std::f32::consts::TAU * s as f32 / segs as f32;
            positions
                .push(radius * Vec3::new(phi.sin() * th.cos(), phi.sin() * th.sin(), phi.cos()));
        }
    }
    positions.push(Vec3::new(0.0, 0.0, -radius));
    let south = positions.len() as u32 - 1;
    let ring = |r: u32, s: u32| 1 + (r - 1) * segs + s % segs;
    let mut tris = Vec::new();
    for s in 0..segs {
        tris.push([0, ring(1, s), ring(1, s + 1)]);
    }
    for r in 1..rings - 1 {
        for s in 0..segs {
            let (a, b, c, d) = (
                ring(r, s),
                ring(r, s + 1),
                ring(r + 1, s),
                ring(r + 1, s + 1),
            );
            tris.push([a, c, b]);
            tris.push([b, c, d]);
        }
    }
    for s in 0..segs {
        tris.push([south, ring(rings - 1, s + 1), ring(rings - 1, s)]);
    }
    TriMesh::new(positions, tris)
}

fn tmp(name: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("dc-mesh-io-{}-{name}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    dir.join(name)
}

/// Removes a file made by [`tmp`] and its private directory.
fn rm(path: &Path) {
    std::fs::remove_file(path).ok();
    if let Some(dir) = path.parent() {
        std::fs::remove_dir(dir).ok();
    }
}

/// Per-triangle geometry equal within `tol`, same topology.
fn assert_same_geometry(a: &TriMesh, b: &TriMesh, tol: f32) {
    assert_eq!(a.face_count(), b.face_count());
    assert_eq!(a.vertex_count(), b.vertex_count());
    for f in 0..a.face_count() {
        let (ta, tb) = (a.triangle(f), b.triangle(f));
        for k in 0..3 {
            assert!((ta[k] - tb[k]).length() <= tol, "face {f} corner {k}");
        }
    }
    assert_eq!(a.validate(), b.validate());
}

#[test]
fn round_trip_every_format() {
    let mesh = sphere(12, 16, 25.0);
    assert!(mesh.validate().boundary_edges == 0);
    for (fmt, file) in [
        (MeshFormat::Stl, "rt.stl"),
        (MeshFormat::Obj, "rt.obj"),
        (MeshFormat::Ply, "rt.ply"),
    ] {
        let path = tmp(file);
        write(&path, &mesh, fmt).unwrap();
        let back = read(&path).unwrap();
        assert_same_geometry(&mesh, &back, 1e-6);
        rm(&path);
    }
    // ASCII STL
    let path = tmp("rt_ascii.stl");
    write_stl_ascii(&path, &mesh).unwrap();
    assert!(std::fs::read(&path).unwrap().starts_with(b"solid"));
    assert_same_geometry(&mesh, &read(&path).unwrap(), 1e-6);
    rm(&path);
}

#[test]
fn stl_weld_merges_duplicates_and_keeps_topology() {
    let mesh = tetra();
    let mut buf = Vec::new();
    write_to(&mut buf, &mesh, MeshFormat::Stl).unwrap();
    assert_eq!(buf.len(), 84 + 4 * 50);
    let back = read_bytes(&buf, MeshFormat::Stl, &ReadOptions::default()).unwrap();
    assert_eq!(back.vertex_count(), 4);
    assert_eq!(back.validate().boundary_edges, 0);
    assert_eq!(back.validate().components, 1);
    assert!(back.normals.is_none());
}

#[test]
fn stl_weld_tolerance() {
    // two triangles sharing an edge, shared vertices perturbed by 1e-5 on a 10 mm box
    let mut bin = vec![b' '; 80];
    bin.extend_from_slice(&2u32.to_le_bytes());
    let tris: [[[f32; 3]; 3]; 2] = [
        [[0.0, 0.0, 0.0], [10.0, 0.0, 0.0], [10.0, 10.0, 0.0]],
        [[0.0, 0.0, 0.0], [10.0, 10.0, 0.0], [0.0, 10.0, 0.0]],
    ];
    for (i, t) in tris.iter().enumerate() {
        bin.extend_from_slice(&[0u8; 12]);
        for v in t {
            let jitter = if i == 1 { 1e-7 } else { 0.0 };
            for x in v {
                bin.extend_from_slice(&(x + jitter * x.signum()).to_le_bytes());
            }
        }
        bin.extend_from_slice(&[0u8; 2]);
    }
    let loose = ReadOptions {
        weld_tolerance: 1e-4,
    };
    let m = read_bytes(&bin, MeshFormat::Stl, &loose).unwrap();
    assert_eq!(m.vertex_count(), 4);
    let exact = ReadOptions {
        weld_tolerance: 0.0,
    };
    let m = read_bytes(&bin, MeshFormat::Stl, &exact).unwrap();
    assert!(m.vertex_count() > 4);
}

#[test]
fn binary_stl_with_solid_header_is_detected_as_binary() {
    let mesh = quad();
    let mut buf = Vec::new();
    write_to(&mut buf, &mesh, MeshFormat::Stl).unwrap();
    buf[..80].fill(0);
    buf[..16].copy_from_slice(b"solid exporter v");
    let back = read_bytes(&buf, MeshFormat::Stl, &ReadOptions::default()).unwrap();
    assert_eq!(back.face_count(), 2);
    assert_eq!(back.vertex_count(), 4);
}

#[test]
fn ascii_stl_with_odd_formatting() {
    let text = "SOLID  my part\r\n facet normal 0 0 1\r\n  outer loop\r\n\
        vertex 0 0 0\r\n vertex 1E0 0 0\r\n  vertex 0 1.0e0 0\r\n endloop\r\n endfacet\r\n\
        endsolid my part\r\n";
    let m = read_bytes(text.as_bytes(), MeshFormat::Stl, &ReadOptions::default()).unwrap();
    assert_eq!((m.vertex_count(), m.face_count()), (3, 1));
}

#[test]
fn obj_polygons_comments_negative_indices_and_colors() {
    let text = "# c\nv 0 0 0 1 0 0\nv 1 0 0 0 1 0\nv 1 1 0 0 0 1\nv 0 1 0\nvt 0 0\nvn 0 0 1\n\
        o thing\nf 1/1/1 2/1/1 3/1/1 4//1\nf -4 -3 -2\n";
    let m = read_bytes(text.as_bytes(), MeshFormat::Obj, &ReadOptions::default()).unwrap();
    assert_eq!(m.tris, vec![[0, 1, 2], [0, 2, 3], [0, 1, 2]]);
    let c = m.vertex_colors.as_ref().unwrap();
    assert_eq!(c[0], [255, 0, 0, 255]);
    assert_eq!(c[2], [0, 0, 255, 255]);
    assert_eq!(c[3], [255; 4]);
    // colour round trip
    let mut buf = Vec::new();
    write_to(&mut buf, &m, MeshFormat::Obj).unwrap();
    let back = read_bytes(&buf, MeshFormat::Obj, &ReadOptions::default()).unwrap();
    assert_eq!(back.vertex_colors, m.vertex_colors);
}

#[test]
fn ply_round_trip_with_colors_and_normals() {
    let mut m = sphere(6, 8, 3.0);
    m.update_normals();
    m.vertex_colors = Some(
        (0..m.vertex_count())
            .map(|i| [i as u8, 255 - i as u8, 7, 200])
            .collect(),
    );
    let mut buf = Vec::new();
    write_to(&mut buf, &m, MeshFormat::Ply).unwrap();
    let back = read_bytes(&buf, MeshFormat::Ply, &ReadOptions::default()).unwrap();
    assert_eq!(back, m);
}

#[test]
fn ply_ascii_and_big_endian() {
    let ascii = "ply\nformat ascii 1.0\ncomment x\nelement vertex 4\nproperty float x\nproperty float y\n\
        property float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\n\
        element face 1\nproperty list uchar uint vertex_index\nproperty uchar flags\nend_header\n\
        0 0 0 255 0 0\n1 0 0 0 255 0\n1 1 0 0 0 255\n0 1 0 9 9 9\n4 0 1 2 3 5\n";
    let m = read_bytes(ascii.as_bytes(), MeshFormat::Ply, &ReadOptions::default()).unwrap();
    assert_eq!(m.tris, vec![[0, 1, 2], [0, 2, 3]]);
    assert_eq!(m.vertex_colors.as_ref().unwrap()[0], [255, 0, 0, 255]);

    let mut be = b"ply\nformat binary_big_endian 1.0\nelement vertex 3\nproperty double x\n\
        property float y\nproperty float z\nelement edge 1\nproperty int a\nproperty int b\n\
        element face 1\nproperty list uchar short vertex_indices\nend_header\n"
        .to_vec();
    for (x, y, z) in [(0.0f64, 0.0f32, 0.0f32), (1.5, 0.0, 0.0), (0.0, 2.5, 0.0)] {
        be.extend_from_slice(&x.to_be_bytes());
        be.extend_from_slice(&y.to_be_bytes());
        be.extend_from_slice(&z.to_be_bytes());
    }
    be.extend_from_slice(&[0, 0, 0, 1, 0, 0, 0, 2]); // edge (skipped)
    be.push(3);
    for i in [0i16, 1, 2] {
        be.extend_from_slice(&i.to_be_bytes());
    }
    let m = read_bytes(&be, MeshFormat::Ply, &ReadOptions::default()).unwrap();
    assert_eq!(m.positions[1], Vec3::new(1.5, 0.0, 0.0));
    assert_eq!(m.positions[2], Vec3::new(0.0, 2.5, 0.0));
    assert_eq!(m.tris, vec![[0, 1, 2]]);
}

#[test]
fn malformed_input_is_an_error() {
    let o = ReadOptions::default();
    let cases: &[(MeshFormat, &[u8])] = &[
        (MeshFormat::Stl, b""),
        (MeshFormat::Stl, b"solid x\nfacet\nvertex 1 2\nendfacet\n"),
        (MeshFormat::Stl, b"solid x\nvertex 0 0 0\nvertex 1 0 0\n"),
        (MeshFormat::Stl, b"solid x\nvertex nan 0 0\nvertex 1 0 0\nvertex 0 1 0\n"),
        (MeshFormat::Obj, b"v 0 0 0\nf 1 2 3\n"),
        (MeshFormat::Obj, b"v 0 0 0\nv 1 0 0\nf 1 2\n"),
        (MeshFormat::Obj, b"v a b c\n"),
        (MeshFormat::Obj, b"v 0 0 0\nv 1 0 0\nv 0 1 0\nf 0 1 2\n"),
        (MeshFormat::Ply, b""),
        (MeshFormat::Ply, b"ply\nformat ascii 1.0\nelement vertex 1\nproperty float x\n"),
        (MeshFormat::Ply, b"ply\nformat ascii 1.0\nelement vertex 99999999999\nproperty float x\nproperty float y\nproperty float z\nend_header\n1 2 3\n"),
        (MeshFormat::Ply, b"ply\nformat binary_little_endian 1.0\nelement vertex 1\nproperty float x\nproperty float y\nproperty float z\nend_header\n\0\0"),
        (MeshFormat::Ply, b"ply\nformat ascii 1.0\nelement vertex 3\nproperty float x\nproperty float y\nproperty float z\nelement face 1\nproperty list uchar int vertex_indices\nend_header\n0 0 0\n1 0 0\n0 1 0\n3 0 1 7\n"),
    ];
    for (i, (fmt, data)) in cases.iter().enumerate() {
        let r = read_bytes(data, *fmt, &o);
        assert!(r.is_err(), "case {i} should fail: {r:?}");
    }
}

#[test]
fn truncated_and_corrupted_buffers_never_panic() {
    let mesh = sphere(5, 7, 10.0);
    let o = ReadOptions::default();
    let mut images: Vec<(MeshFormat, Vec<u8>)> = Vec::new();
    for fmt in [MeshFormat::Stl, MeshFormat::Obj, MeshFormat::Ply] {
        let mut buf = Vec::new();
        write_to(&mut buf, &mesh, fmt).unwrap();
        images.push((fmt, buf));
    }
    let mut ascii = Vec::new();
    write_stl_ascii_to(&mut ascii, &mesh).unwrap();
    images.push((MeshFormat::Stl, ascii));

    let mut rng = Rng(0x9E37_79B9_7F4A_7C15);
    for (fmt, img) in &images {
        for cut in (0..img.len()).step_by(7).chain([img.len() - 1]) {
            let _ = read_bytes(&img[..cut], *fmt, &o);
        }
        for _ in 0..300 {
            let mut copy = img.clone();
            for _ in 0..4 {
                let at = (rng.next() as usize) % copy.len();
                copy[at] = rng.next() as u8;
            }
            if let Ok(m) = read_bytes(&copy, *fmt, &o) {
                m.check().unwrap(); // accepted data must be internally consistent
            }
        }
    }
    // a truncated binary STL is an error, not a short mesh
    let (_, stl) = &images[0];
    assert!(read_bytes(&stl[..stl.len() - 10], MeshFormat::Stl, &o).is_err());
}

#[test]
fn write_rejects_inconsistent_mesh() {
    let mut m = quad();
    m.tris.push([0, 1, 9]);
    let mut buf = Vec::new();
    assert!(write_to(&mut buf, &m, MeshFormat::Stl).is_err());
    assert!(write_to(&mut buf, &m, MeshFormat::Obj).is_err());
    assert!(write_to(&mut buf, &m, MeshFormat::Ply).is_err());
}

#[test]
fn unit_detection_cases() {
    let cube = |edge: f32| {
        let mut m = quad();
        m.scale(edge);
        m
    };
    assert_eq!(detect_units(&cube(0.06)).unwrap(), Units::Meters);
    assert_eq!(detect_units(&cube(6.0)).unwrap(), Units::Centimeters);
    assert_eq!(detect_units(&cube(60.0)).unwrap(), Units::Millimeters);
    assert_eq!(detect_units(&cube(60_000.0)).unwrap(), Units::Micrometers);
    assert_eq!(detect_units(&cube(1.0)).unwrap(), Units::Millimeters);
    assert_eq!(
        detect_units(&TriMesh::default()).unwrap(),
        Units::Millimeters
    );
    let mut bad = quad();
    bad.positions[0].x = f32::NAN;
    assert!(detect_units(&bad).is_err());

    for (edge, expect) in [(0.06f32, 60.0f32), (6.0, 60.0), (60_000.0, 60.0)] {
        let mut m = cube(edge);
        normalize_units(&mut m).unwrap();
        let ext = m.aabb().unwrap().size().max_element();
        assert!((ext - expect).abs() < 1e-3, "{edge}: {ext}");
    }
    let mut m = cube(0.06);
    m.scale_to_mm(Units::Meters);
    assert!((m.aabb().unwrap().size().x - 60.0).abs() < 1e-3);
}

#[test]
#[ignore = "benchmark: run with `cargo test -p dc-mesh --release -- --ignored`"]
fn million_triangle_binary_stl_loads_under_one_second() {
    // ~1M triangles: 708 x 708 grid, 2 triangles per cell
    let n = 708usize;
    let mut rng = Rng(42);
    let pts: Vec<Vec3> = (0..=n * n + 2 * n + 1)
        .map(|i| {
            let (x, y) = ((i % (n + 1)) as f32, (i / (n + 1)) as f32);
            Vec3::new(x * 0.05, y * 0.05, rng.unit() * 0.01)
        })
        .collect();
    let at = |x: usize, y: usize| (y * (n + 1) + x) as u32;
    let mut tris = Vec::new();
    for y in 0..n {
        for x in 0..n {
            tris.push([at(x, y), at(x + 1, y), at(x + 1, y + 1)]);
            tris.push([at(x, y), at(x + 1, y + 1), at(x, y + 1)]);
        }
    }
    let mesh = TriMesh::new(pts, tris);
    assert!(mesh.face_count() >= 1_000_000);
    let path = tmp("million.stl");
    write(&path, &mesh, MeshFormat::Stl).unwrap();

    let t = std::time::Instant::now();
    let back = read(&path).unwrap();
    let dt = t.elapsed();
    rm(&path);
    eprintln!("read {} triangles in {dt:?}", back.face_count());
    assert_eq!(back.face_count(), mesh.face_count());
    assert!(back.vertex_count() <= mesh.vertex_count());
    assert!(dt.as_secs_f64() < 1.0, "took {dt:?}");
}

/// Optional check against real scans: `DC_TEETH3DS=/path/to/dataset`.
#[test]
fn teeth3ds_obj_loads_when_available() {
    let Some(root) = std::env::var_os("DC_TEETH3DS") else {
        return;
    };
    fn first_obj(dir: &Path) -> Option<PathBuf> {
        let mut subdirs = Vec::new();
        for e in std::fs::read_dir(dir).ok()?.flatten() {
            let p = e.path();
            if p.is_dir() {
                subdirs.push(p);
            } else if p.extension().is_some_and(|x| x.eq_ignore_ascii_case("obj")) {
                return Some(p);
            }
        }
        subdirs.iter().find_map(|d| first_obj(d))
    }
    let Some(path) = first_obj(Path::new(&root)) else {
        eprintln!("no OBJ found under DC_TEETH3DS");
        return;
    };
    let mesh = read(&path).unwrap();
    assert!(mesh.face_count() > 1000, "{}", path.display());
    mesh.check().unwrap();
    let u = detect_units(&mesh).unwrap();
    eprintln!(
        "{}: {} v, {} f, units {:?}",
        path.display(),
        mesh.vertex_count(),
        mesh.face_count(),
        u
    );
}

const TWO_OBJECTS: &str = "\
o upper
v 0 0 0
v 1 0 0
v 0 1 0
f 1 2 3
o lower
v 0 0 1
f 1 2 3 4
f 1 3 4
";

#[test]
fn read_scan_counts_obj_objects_with_faces() {
    let opts = ReadOptions::default();
    let two = TWO_OBJECTS;
    let scan = read_scan_bytes(two.as_bytes(), MeshFormat::Obj, &opts).unwrap();
    assert_eq!(scan.format, MeshFormat::Obj);
    assert_eq!(scan.obj_object_count, Some(2));
    assert_eq!(scan.mesh.face_count(), 4);
    assert_eq!(
        scan.mesh,
        read_bytes(two.as_bytes(), MeshFormat::Obj, &opts).unwrap()
    );
}

#[test]
fn read_scan_ignores_objects_without_faces() {
    let obj = "o empty\no a\nv 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\no trailing\nv 5 5 5\n";
    let scan = read_scan_bytes(obj.as_bytes(), MeshFormat::Obj, &ReadOptions::default()).unwrap();
    assert_eq!(scan.obj_object_count, Some(1));
}

#[test]
fn read_scan_plain_obj_is_one_implicit_object() {
    let opts = ReadOptions::default();
    let plain = "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n";
    let scan = read_scan_bytes(plain.as_bytes(), MeshFormat::Obj, &opts).unwrap();
    assert_eq!(scan.obj_object_count, Some(1));
    // faces before the first `o` plus a named object with faces make two
    let mixed = "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\no b\nf 3 2 1\n";
    let scan = read_scan_bytes(mixed.as_bytes(), MeshFormat::Obj, &opts).unwrap();
    assert_eq!(scan.obj_object_count, Some(2));
    // no faces at all
    let scan = read_scan_bytes(b"v 0 0 0\n", MeshFormat::Obj, &opts).unwrap();
    assert_eq!(scan.obj_object_count, Some(0));
}

#[test]
fn read_scan_from_file_reports_format_and_no_count_for_stl() {
    let mesh = tetra();
    let opts = ReadOptions::default();
    for (fmt, file) in [(MeshFormat::Stl, "scan.stl"), (MeshFormat::Ply, "scan.ply")] {
        let path = tmp(file);
        write(&path, &mesh, fmt).unwrap();
        let scan = read_scan(&path, &opts).unwrap();
        assert_eq!(scan.format, fmt);
        assert_eq!(scan.obj_object_count, None);
        assert_eq!(scan.mesh, read(&path).unwrap());
        rm(&path);
    }
    let path = tmp("scan.obj");
    write(&path, &mesh, MeshFormat::Obj).unwrap();
    let scan = read_scan(&path, &opts).unwrap();
    assert_eq!(scan.obj_object_count, Some(1));
    rm(&path);
    assert!(read_scan("nope.xyz", &opts).is_err());
}
