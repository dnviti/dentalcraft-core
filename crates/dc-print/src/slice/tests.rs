//! Slicer tests on synthetic solids (print plan §9.1).

use std::f32::consts::TAU;
use std::hash::{DefaultHasher, Hash, Hasher};

use glam::{Quat, Vec3};

use super::*;
use crate::part::{rotate_mesh, translate_mesh};

fn saturn() -> PrinterProfile {
    PrinterProfile::saturn_4_ultra_16k()
}

/// A closed, outward prism over a counter-clockwise polygon that is
/// star-shaped from its first vertex (the cap fans start there).
fn prism(poly: &[[f32; 2]], z0: f32, z1: f32) -> TriMesh {
    let n = poly.len() as u32;
    let mut positions: Vec<Vec3> = poly.iter().map(|p| Vec3::new(p[0], p[1], z0)).collect();
    positions.extend(poly.iter().map(|p| Vec3::new(p[0], p[1], z1)));
    let mut tris = Vec::new();
    for k in 0..n {
        let j = (k + 1) % n;
        tris.push([k, j, n + j]);
        tris.push([k, n + j, n + k]);
    }
    for k in 1..n - 1 {
        tris.push([0, k + 1, k]);
        tris.push([n, n + k, n + k + 1]);
    }
    TriMesh::new(positions, tris)
}

fn cuboid(min: [f32; 3], max: [f32; 3]) -> TriMesh {
    let poly = [
        [min[0], min[1]],
        [max[0], min[1]],
        [max[0], max[1]],
        [min[0], max[1]],
    ];
    prism(&poly, min[2], max[2])
}

/// The same solid with inward faces (a cavity when nested in another).
fn inverted(mut mesh: TriMesh) -> TriMesh {
    for t in &mut mesh.tris {
        t.swap(1, 2);
    }
    mesh
}

/// Slices and keeps every layer, checking the sink order.
fn slice_all(
    meshes: &[&TriMesh],
    profile: &PrinterProfile,
    layer_um: u32,
) -> (Vec<SlicedLayer>, f64) {
    let mut layers = Vec::new();
    let volume = slice_plate(meshes, profile, layer_um, &mut |i, layer| {
        assert_eq!(i as usize, layers.len(), "sink called in layer order");
        layers.push(layer);
        Ok(())
    })
    .unwrap();
    assert_eq!(layers.len() as u32, layer_count(meshes, layer_um));
    (layers, volume)
}

fn lit(runs: &[Run]) -> u64 {
    runs.iter()
        .filter(|r| r.value == 0xFF)
        .map(|r| u64::from(r.len))
        .sum()
}

fn covered(runs: &[Run]) -> u64 {
    runs.iter().map(|r| u64::from(r.len)).sum()
}

/// Lit file pixels (col, row), row-major.
fn lit_pixels(runs: &[Run], width: u32) -> Vec<(u32, u32)> {
    let width = u64::from(width);
    let mut out = Vec::new();
    let mut at = 0u64;
    for r in runs {
        if r.value == 0xFF {
            out.extend(
                (at..at + u64::from(r.len)).map(|p| ((p % width) as u32, (p / width) as u32)),
            );
        }
        at += u64::from(r.len);
    }
    out
}

/// Pixels whose centre (k + ½)·pitch lies in [a, b): the oracle, written
/// independently of the slicer's own arithmetic.
fn centres_in(a: f64, b: f64, pitch_um: f64, count: u32) -> Vec<u32> {
    (0..count)
        .filter(|&k| {
            let c = (f64::from(k) + 0.5) * pitch_um / 1000.0;
            a <= c && c < b
        })
        .collect()
}

fn count_in(x: (f64, f64), y: (f64, f64)) -> u64 {
    let p = saturn();
    let cols = centres_in(x.0, x.1, p.pixel_pitch_x_um, p.resolution_x).len() as u64;
    let rows = centres_in(y.0, y.1, p.pixel_pitch_y_um, p.resolution_y).len() as u64;
    cols * rows
}

/// Signed volume of a closed mesh (divergence theorem).
fn mesh_volume(mesh: &TriMesh) -> f64 {
    (0..mesh.face_count())
        .map(|f| {
            let [a, b, c] = mesh.triangle(f).map(|v| v.as_dvec3());
            a.dot(b.cross(c)) / 6.0
        })
        .sum()
}

fn digest(layers: &[SlicedLayer]) -> u64 {
    let mut h = DefaultHasher::new();
    for l in layers {
        for r in &l.runs {
            (r.value, r.len).hash(&mut h);
        }
        l.area_mm2.to_bits().hash(&mut h);
    }
    h.finish()
}

#[test]
fn cube_counts_layers_pixels_and_volume() {
    let p = saturn();
    let cube = cuboid([50.0, 30.0, 0.0], [60.0, 40.0, 10.0]);
    let (layers, volume) = slice_all(&[&cube], &p, 50);
    // ⌈10 / 0.05⌉
    assert_eq!(layers.len(), 200);

    let cols = centres_in(50.0, 60.0, 14.0, p.resolution_x);
    let rows = centres_in(30.0, 40.0, 19.0, p.resolution_y);
    // Within one column and one row of the ideal 10 mm / pitch.
    assert!((cols.len() as f64 - 10.0 / 0.014).abs() < 1.0);
    assert!((rows.len() as f64 - 10.0 / 0.019).abs() < 1.0);
    let want = (cols.len() * rows.len()) as u64;
    let pixel_mm2 = 0.014 * 0.019;
    // Within one row and one column of the ideal pixel count.
    let ideal = 100.0 / pixel_mm2;
    assert!((want as f64 - ideal).abs() <= (cols.len() + rows.len()) as f64);
    for (i, layer) in layers.iter().enumerate() {
        assert_eq!(covered(&layer.runs), p.pixel_count(), "layer {i}");
        assert_eq!(lit(&layer.runs), want, "layer {i}");
        assert!((layer.area_mm2 - want as f64 * pixel_mm2).abs() < 1e-9);
    }
    // No mirroring: the first lit pixel is the box's corner pixel.
    let leading = layers[0].runs[0];
    assert_eq!(leading.value, 0);
    assert_eq!(
        u64::from(leading.len),
        u64::from(rows[0]) * u64::from(p.resolution_x) + u64::from(cols[0])
    );
    assert!((volume / 1000.0 - 1.0).abs() < 0.01, "volume {volume}");
    assert!((volume - 200.0 * want as f64 * pixel_mm2 * 0.05).abs() < 1e-6);
}

#[test]
fn top_in_the_lower_half_of_a_layer_adds_no_blank_layer() {
    let cube = cuboid([50.0, 30.0, 0.0], [60.0, 40.0, 10.0]);
    // 10 / 0.03 = 333.3: the 334th centre (10.005) is above the top.
    assert_eq!(layer_count(&[&cube], 30), 333);
    // 10.04 / 0.05 = 200.8: the 201st centre (10.025) is below the top.
    let taller = cuboid([50.0, 30.0, 0.0], [60.0, 40.0, 10.04]);
    assert_eq!(layer_count(&[&taller], 50), 201);
    assert_eq!(layer_count(&[], 50), 0);
    assert_eq!(layer_count(&[&cube], 0), 0);
}

#[test]
fn cylinder_area_per_layer_within_one_percent() {
    let p = saturn();
    let (n, r) = (512usize, 5.0f32);
    let poly: Vec<[f32; 2]> = (0..n)
        .map(|k| {
            let a = TAU * k as f32 / n as f32;
            [100.0 + r * a.cos(), 60.0 + r * a.sin()]
        })
        .collect();
    let cylinder = prism(&poly, 0.0, 3.0);
    let (layers, volume) = slice_all(&[&cylinder], &p, 50);
    assert_eq!(layers.len(), 60);
    let circle = std::f64::consts::PI * 25.0;
    for (i, layer) in layers.iter().enumerate() {
        let err = layer.area_mm2 / circle - 1.0;
        assert!(err.abs() < 0.01, "layer {i}: {err:+.4}");
    }
    assert!((volume / (circle * 3.0) - 1.0).abs() < 0.01);
}

#[test]
fn overlapping_meshes_cure_once() {
    let p = saturn();
    let a = cuboid([50.0, 30.0, 0.0], [60.0, 40.0, 2.0]);
    let b = cuboid([55.0, 35.0, 0.0], [65.0, 45.0, 2.0]);
    let (_, va) = slice_all(&[&a], &p, 50);
    let (_, vb) = slice_all(&[&b], &p, 50);
    let (union, vu) = slice_all(&[&a, &b], &p, 50);
    let want = count_in((50.0, 60.0), (30.0, 40.0)) + count_in((55.0, 65.0), (35.0, 45.0))
        - count_in((55.0, 60.0), (35.0, 40.0));
    for (i, layer) in union.iter().enumerate() {
        assert_eq!(lit(&layer.runs), want, "layer {i}");
    }
    let overlap = count_in((55.0, 60.0), (35.0, 40.0)) as f64 * 0.014 * 0.019 * 2.0;
    assert!((vu - (va + vb - overlap)).abs() < 1e-6);

    // A support strut sunk into a part adds only what sticks out.
    let strut = cuboid([52.0, 32.0, -1.0], [53.0, 33.0, 1.5]);
    let (with_strut, _) = slice_all(&[&a, &strut], &p, 50);
    let (alone, _) = slice_all(&[&a], &p, 50);
    assert_eq!(digest(&with_strut), digest(&alone));

    // Touching parts: one solid, no gap and no pixel counted twice.
    let left = cuboid([50.0, 30.0, 0.0], [55.0, 40.0, 2.0]);
    let right = cuboid([55.0, 30.0, 0.0], [60.0, 40.0, 2.0]);
    let whole = cuboid([50.0, 30.0, 0.0], [60.0, 40.0, 2.0]);
    let (pair, _) = slice_all(&[&left, &right], &p, 50);
    let (one, _) = slice_all(&[&whole], &p, 50);
    assert_eq!(pair, one);
}

#[test]
fn hollow_box_walls_have_exact_pixel_width() {
    let p = saturn();
    // 1 mm walls; the closed cavity is an inward-facing box, so the winding
    // is 0 inside it.
    let outer = cuboid([50.0, 30.0, 0.0], [60.0, 40.0, 1.0]);
    let cavity = inverted(cuboid([51.0, 31.0, 0.2], [59.0, 39.0, 0.8]));
    let (layers, _) = slice_all(&[&outer, &cavity], &p, 50);
    let row = centres_in(35.0, 35.1, 19.0, p.resolution_y)[0];
    let width = p.resolution_x;
    let in_row: Vec<u32> = lit_pixels(&layers[10].runs, width)
        .into_iter()
        .filter(|&(_, r)| r == row)
        .map(|(c, _)| c)
        .collect();
    let left = centres_in(50.0, 51.0, 14.0, width);
    let right = centres_in(59.0, 60.0, 14.0, width);
    assert_eq!(in_row, [left.clone(), right].concat());
    // 1 mm / 14 µm = 71.4 pixels.
    assert!(left.len() == 71 || left.len() == 72);
}

#[test]
fn l_prism_keeps_its_chirality_through_map_pixel() {
    // Saturn pitch (14 × 19 µm) on a small grid; the other fields come from
    // the Saturn profile, and each case below sets both mirror flags.
    let base = PrinterProfile {
        id: "small".into(),
        machine_name: "Small".into(),
        resolution_x: 300,
        resolution_y: 200,
        pixel_pitch_x_um: 14.0,
        pixel_pitch_y_um: 19.0,
        build_mm: [4.2, 3.8, 220.0],
        ..PrinterProfile::saturn_4_ultra_16k()
    };
    // An L: long arm along +x, short arm along +y, joined at the low corner.
    // Listed counter-clockwise from the reflex vertex.
    let l = prism(
        &[
            [1.0, 1.0],
            [1.0, 3.0],
            [0.5, 3.0],
            [0.5, 0.5],
            [3.5, 0.5],
            [3.5, 1.0],
        ],
        0.0,
        0.1,
    );
    let (w, h) = (base.resolution_x, base.resolution_y);
    let rect = |x: (f64, f64), y: (f64, f64)| {
        let cols = centres_in(x.0, x.1, 14.0, w);
        let rows = centres_in(y.0, y.1, 19.0, h);
        let mut v = Vec::new();
        for &r in &rows {
            v.extend(cols.iter().map(|&c| (c, r)));
        }
        v
    };
    let mut want = rect((0.5, 3.5), (0.5, 1.0));
    want.extend(rect((0.5, 1.0), (1.0, 3.0)));
    want.sort_unstable();

    let mut files = Vec::new();
    for (mirror_x, mirror_y) in [(false, false), (true, false), (false, true), (true, true)] {
        let profile = PrinterProfile {
            mirror_x,
            mirror_y,
            ..base.clone()
        };
        let (layers, _) = slice_all(&[&l], &profile, 50);
        let file = lit_pixels(&layers[0].runs, w);
        // Back to plate pixels through the one mapping (an involution).
        let mut plate: Vec<(u32, u32)> = file
            .iter()
            .map(|&(c, r)| profile.map_pixel(c, r).unwrap())
            .collect();
        plate.sort_unstable();
        assert_eq!(plate, want, "mirror {mirror_x} {mirror_y}");
        // The file itself is mirrored as the flags say.
        let mut expect_file: Vec<(u32, u32)> = want
            .iter()
            .map(|&(c, r)| {
                (
                    if mirror_x { w - 1 - c } else { c },
                    if mirror_y { h - 1 - r } else { r },
                )
            })
            .collect();
        expect_file.sort_unstable();
        let mut file_sorted = file.clone();
        file_sorted.sort_unstable();
        assert_eq!(file_sorted, expect_file);
        files.push(file_sorted);
    }
    // Every mirror gives a different image: none is a no-op.
    for (i, a) in files.iter().enumerate() {
        for b in &files[i + 1..] {
            assert_ne!(a, b);
        }
    }
    // Chirality in the plate frame: the short arm rises at low x.
    let lit_at = |x: f64, y: f64| {
        let c = (x / 0.014) as u32;
        let r = (y / 0.019) as u32;
        want.binary_search(&(c, r)).is_ok()
    };
    assert!(lit_at(0.75, 2.5) && lit_at(3.0, 0.75));
    assert!(!lit_at(3.0, 2.5) && !lit_at(0.25, 0.25));
}

#[test]
fn on_plane_vertices_and_degenerate_triangles_do_not_leak() {
    let p = saturn();
    // h = 100 µm: sampling planes at 0.05, 0.15, …; 0.25, 0.75 and 1.25
    // are exact in f32 too.
    let (x0, x1, y0, y1, z0, zm, z1) = (50.0, 52.0, 30.0, 31.0, 0.25, 0.75, 1.25);
    let b = [[x0, y0], [x1, y0], [x1, y1], [x0, y1]];
    let mut positions: Vec<Vec3> = b.iter().map(|q| Vec3::new(q[0], q[1], z0)).collect();
    positions.extend(b.iter().map(|q| Vec3::new(q[0], q[1], z1)));
    // m splits the vertical edge b1–t1 exactly on the plane of layer 7.
    positions.push(Vec3::new(x1, y0, zm));
    let (b0, b1, b2, b3, t0, t1, t2, t3, m) = (0, 1, 2, 3, 4, 5, 6, 7, 8);
    let tris = vec![
        [b0, b2, b1],
        [b0, b3, b2],
        [t0, t1, t2],
        [t0, t2, t3],
        // Front face, split at m.
        [b0, b1, m],
        [b0, m, t1],
        [b0, t1, t0],
        // Zero-area triangle closing the T-junction at m.
        [b1, t1, m],
        [b1, b2, t2],
        [b1, t2, t1],
        [b2, b3, t3],
        [b2, t3, t2],
        [b3, b0, t0],
        [b3, t0, t3],
    ];
    let boxed = TriMesh::new(positions, tris);
    // A pyramid with its base on plane 2 and its apex exactly on plane 7.
    let pyramid = TriMesh::new(
        vec![
            Vec3::new(70.0, 30.0, z0),
            Vec3::new(72.0, 30.0, z0),
            Vec3::new(72.0, 32.0, z0),
            Vec3::new(70.0, 32.0, z0),
            Vec3::new(71.0, 31.0, zm),
        ],
        vec![
            [0, 2, 1],
            [0, 3, 2],
            [0, 1, 4],
            [1, 2, 4],
            [2, 3, 4],
            [3, 0, 4],
        ],
    );
    // Loose degenerate triangles: needles and a point.
    let q = Vec3::new(90.0, 50.0, 0.1);
    let r = Vec3::new(90.0, 50.0, 1.1);
    let s = Vec3::new(91.0, 51.0, 0.6);
    let junk = TriMesh::new(
        vec![q, r, s],
        vec![[0, 0, 1], [1, 2, 2], [2, 2, 2], [1, 1, 0]],
    );

    let (layers, _) = slice_all(&[&boxed, &pyramid, &junk], &p, 100);
    // Centres below the top 1.25: layers 0..12.
    assert_eq!(layers.len(), 12);
    let box_px = count_in((x0.into(), x1.into()), (y0.into(), y1.into()));
    let width = p.resolution_x;
    for (i, layer) in layers.iter().enumerate() {
        let px = lit_pixels(&layer.runs, width);
        // Nothing outside the box and the pyramid's base.
        assert!(px.iter().all(|&(c, r)| {
            let (x, y) = ((f64::from(c) + 0.5) * 0.014, (f64::from(r) + 0.5) * 0.019);
            ((50.0..52.0).contains(&x) && (30.0..31.0).contains(&y))
                || ((70.0..72.0).contains(&x) && (30.0..32.0).contains(&y))
        }));
        let in_box = px.iter().filter(|&&(c, _)| c < 5000).count() as u64;
        let in_pyramid = px.len() as u64 - in_box;
        // Bottom on a plane: lit from layer 2 on. Apex on plane 7: dark.
        assert_eq!(in_box, if i >= 2 { box_px } else { 0 }, "box, layer {i}");
        assert_eq!(in_pyramid > 0, (2..7).contains(&i), "pyramid, layer {i}");
    }
    // The pyramid shrinks layer by layer.
    let pyramid_px = |i: usize| {
        lit_pixels(&layers[i].runs, width)
            .iter()
            .filter(|&&(c, _)| c >= 5000)
            .count()
    };
    assert!((2..6).all(|i| pyramid_px(i) > pyramid_px(i + 1)));

    // Only degenerate triangles: no layer lights anything.
    let (layers, volume) = slice_all(&[&junk], &p, 100);
    assert!(layers.iter().all(|l| lit(&l.runs) == 0));
    assert_eq!(volume, 0.0);
}

#[test]
fn sink_error_stops_slicing_and_is_returned() {
    let cube = cuboid([50.0, 30.0, 0.0], [51.0, 31.0, 1.0]);
    let mut seen = Vec::new();
    let result = slice_plate(&[&cube], &saturn(), 50, &mut |i, _| {
        seen.push(i);
        if i == 3 {
            Err(GooError::Field("stop"))
        } else {
            Ok(())
        }
    });
    assert!(matches!(result, Err(GooError::Field("stop"))));
    assert_eq!(seen, [0, 1, 2, 3]);
}

#[test]
fn bad_input_is_refused() {
    let p = saturn();
    fn ignore(_: u32, _: SlicedLayer) -> Result<(), GooError> {
        Ok(())
    }
    let cube = cuboid([50.0, 30.0, 0.0], [51.0, 31.0, 1.0]);
    let refused = |meshes: &[&TriMesh], profile: &PrinterProfile, layer_um: u32| {
        matches!(
            slice_plate(meshes, profile, layer_um, &mut ignore),
            Err(GooError::Field(_))
        )
    };
    assert!(refused(&[&cube], &p, 0));
    let mut nan = cube.clone();
    nan.positions[3].x = f32::NAN;
    assert!(refused(&[&nan], &p, 50));
    let mut out_of_range = cube.clone();
    out_of_range.tris[0][1] = 99;
    assert!(refused(&[&out_of_range], &p, 50));
    let too_tall = cuboid([50.0, 30.0, 0.0], [51.0, 31.0, 220.1]);
    assert!(refused(&[&too_tall], &p, 50));
    let tallest = cuboid([50.0, 30.0, 0.0], [51.0, 31.0, 220.0]);
    assert!(!refused(&[&tallest], &p, 200));
    // Nothing to slice: no layer, no volume.
    let empty = TriMesh::default();
    assert_eq!(slice_plate(&[&empty], &p, 50, &mut ignore).unwrap(), 0.0);
}

/// The synthetic aligner shell, rim on the plate, placed at `at`.
fn placed_shell(mut mesh: TriMesh, at: Vec3) -> TriMesh {
    let bb = mesh.aabb().unwrap();
    translate_mesh(&mut mesh, at - bb.min);
    mesh
}

#[test]
fn aligner_shell_volume_and_determinism() {
    let p = saturn();
    let shell = placed_shell(
        crate::testutil::default_shell().mesh,
        Vec3::new(20.0, 20.0, 0.0),
    );
    let run = |threads: usize| {
        rayon::ThreadPoolBuilder::new()
            .num_threads(threads)
            .build()
            .unwrap()
            .install(|| slice_all(&[&shell], &p, 50))
    };
    let (one, v1) = run(1);
    let (many, v7) = run(7);
    assert_eq!(digest(&one), digest(&many));
    assert_eq!(v1.to_bits(), v7.to_bits());
    let exact = mesh_volume(&shell);
    assert!(exact > 0.0, "outward shell");
    assert!(
        (v1 / exact - 1.0).abs() < 0.02,
        "sliced {v1:.1} vs mesh {exact:.1} mm³"
    );
}

/// Peak resident memory of this process (Linux only), MiB.
fn peak_rss_mib() -> Option<f64> {
    let status = std::fs::read_to_string("/proc/self/status").ok()?;
    let kib: f64 = status
        .lines()
        .find(|l| l.starts_with("VmHWM:"))?
        .split_whitespace()
        .nth(1)?
        .parse()
        .ok()?;
    Some(kib / 1024.0)
}

/// Slices `meshes` with 1, 4 and all threads, prints the times and a memory
/// estimate, and checks that every thread count gives the same volume.
fn timed(label: &str, meshes: &[&TriMesh], layer_um: u32) {
    let p = saturn();
    let triangles: usize = meshes.iter().map(|m| m.face_count()).sum();
    let mut volumes = Vec::new();
    for threads in [1, 4, rayon::current_num_threads()] {
        let pool = rayon::ThreadPoolBuilder::new()
            .num_threads(threads)
            .build()
            .unwrap();
        let (secs, layers, runs_max, runs_total, volume, batch) = pool.install(|| {
            let start = std::time::Instant::now();
            let (mut layers, mut runs_max, mut runs_total) = (0u32, 0usize, 0usize);
            let volume = slice_plate(meshes, &p, layer_um, &mut |_, layer| {
                layers += 1;
                runs_max = runs_max.max(layer.runs.len());
                runs_total += layer.runs.len();
                Ok(())
            })
            .unwrap();
            let secs = start.elapsed().as_secs_f64();
            (
                secs,
                layers,
                runs_max,
                runs_total,
                volume,
                batch_len() as usize,
            )
        });
        volumes.push(volume.to_bits());
        // Slicer memory: the z index (16 B per triangle), two batches of
        // layers in flight (8 B per run), and per worker the row offsets
        // (8 B per row) plus edge and span buffers (bounded by the busiest
        // layer).
        let index_mib = (triangles * 16) as f64 / 1048576.0;
        let in_flight_mib = (2 * batch * runs_max * 8) as f64 / 1048576.0;
        let per_worker = p.resolution_y as usize * 8 + runs_max * 48;
        let scratch_mib = (threads * per_worker) as f64 / 1048576.0;
        eprintln!(
            "{label}: {triangles} triangles, {layers} layers at {layer_um} µm, \
             {threads} threads (batch {batch}): {secs:.3} s ({:.3} ms/layer), \
             volume {volume:.1} mm³, runs/layer max {runs_max} avg {}; memory estimate \
             {:.1} MiB (index {index_mib:.1}, in flight {in_flight_mib:.1}, scratch \
             {scratch_mib:.1}); peak RSS of the test process so far {}",
            secs * 1000.0 / f64::from(layers.max(1)),
            runs_total / layers.max(1) as usize,
            index_mib + in_flight_mib + scratch_mib,
            peak_rss_mib().map_or("n/a".into(), |m| format!("{m:.0} MiB")),
        );
    }
    assert!(volumes.windows(2).all(|w| w[0] == w[1]));
}

/// The synthetic shell with about `faces` triangles, stood up at 38° as on
/// a print and scaled to `height_mm`, with its lowest point on the plate.
fn standing_shell(faces: usize, height_mm: f32, at: Vec3) -> TriMesh {
    let mut mesh = crate::testutil::aligner_shell_with_faces(faces).mesh;
    rotate_mesh(&mut mesh, Quat::from_rotation_x(38f32.to_radians()));
    let size = mesh.aabb().unwrap().size();
    mesh.scale(height_mm / size.z);
    placed_shell(mesh, at)
}

#[test]
#[ignore = "timing: cargo test -p dc-print --release -- --ignored --nocapture timing_"]
fn timing_200k_triangles_40_mm() {
    let shell = standing_shell(200_000, 40.0, Vec3::new(70.0, 30.0, 0.0));
    timed("one shell", &[&shell], 50);
}

#[test]
#[ignore = "timing: cargo test -p dc-print --release -- --ignored --nocapture timing_"]
fn timing_plate_of_six_shells() {
    let shell = standing_shell(400_000, 40.0, Vec3::ZERO);
    let size = shell.aabb().unwrap().size();
    let plate: Vec<TriMesh> = (0..6)
        .map(|k| {
            let mut m = shell.clone();
            let at = Vec3::new(
                5.0 + (k % 3) as f32 * (size.x + 3.0),
                5.0 + (k / 3) as f32 * (size.y + 3.0),
                0.0,
            );
            translate_mesh(&mut m, at);
            m
        })
        .collect();
    let refs: Vec<&TriMesh> = plate.iter().collect();
    timed("plate of six shells", &refs, 50);
}
