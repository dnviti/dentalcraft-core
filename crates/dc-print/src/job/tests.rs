use super::*;
use crate::goo::GooFile;
use glam::Vec2;

/// Adds the quad `q` (corners in order around it) as two triangles facing
/// `outward`, sharing the vertices already in `m`.
fn quad(m: &mut TriMesh, q: [Vec3; 4], outward: Vec3) {
    let [a, b, c, d] = q.map(|p| match m.positions.iter().position(|&v| v == p) {
        Some(i) => i as u32,
        None => {
            m.positions.push(p);
            (m.positions.len() - 1) as u32
        }
    });
    if (q[1] - q[0]).cross(q[2] - q[0]).dot(outward) >= 0.0 {
        m.tris.extend([[a, b, c], [a, c, d]]);
    } else {
        m.tris.extend([[a, c, b], [a, d, c]]);
    }
}

/// Closed box with outward faces.
fn cuboid(lo: Vec3, hi: Vec3) -> TriMesh {
    let mut m = TriMesh::default();
    let c = |x: f32, y: f32, z: f32| Vec3::new(x, y, z);
    let (a, b) = (lo, hi);
    quad(
        &mut m,
        [
            c(a.x, a.y, a.z),
            c(b.x, a.y, a.z),
            c(b.x, b.y, a.z),
            c(a.x, b.y, a.z),
        ],
        -Vec3::Z,
    );
    quad(
        &mut m,
        [
            c(a.x, a.y, b.z),
            c(b.x, a.y, b.z),
            c(b.x, b.y, b.z),
            c(a.x, b.y, b.z),
        ],
        Vec3::Z,
    );
    quad(
        &mut m,
        [
            c(a.x, a.y, a.z),
            c(b.x, a.y, a.z),
            c(b.x, a.y, b.z),
            c(a.x, a.y, b.z),
        ],
        -Vec3::Y,
    );
    quad(
        &mut m,
        [
            c(a.x, b.y, a.z),
            c(b.x, b.y, a.z),
            c(b.x, b.y, b.z),
            c(a.x, b.y, b.z),
        ],
        Vec3::Y,
    );
    quad(
        &mut m,
        [
            c(a.x, a.y, a.z),
            c(a.x, b.y, a.z),
            c(a.x, b.y, b.z),
            c(a.x, a.y, b.z),
        ],
        -Vec3::X,
    );
    quad(
        &mut m,
        [
            c(b.x, a.y, a.z),
            c(b.x, b.y, a.z),
            c(b.x, b.y, b.z),
            c(b.x, a.y, b.z),
        ],
        Vec3::X,
    );
    m
}

/// A cup: a `size` box with a cavity open at the top and `wall` thick
/// walls and floor, as one closed solid. Returns the mesh and the fitting
/// flags (the cavity's walls and floor).
pub(crate) fn cup(size: Vec3, wall: f32) -> (TriMesh, Vec<bool>) {
    let (l, w, h, t) = (size.x, size.y, size.z, wall);
    let c = |x: f32, y: f32, z: f32| Vec3::new(x, y, z);
    let mut m = TriMesh::default();
    // outside: bottom and four walls
    quad(
        &mut m,
        [c(0., 0., 0.), c(l, 0., 0.), c(l, w, 0.), c(0., w, 0.)],
        -Vec3::Z,
    );
    quad(
        &mut m,
        [c(0., 0., 0.), c(l, 0., 0.), c(l, 0., h), c(0., 0., h)],
        -Vec3::Y,
    );
    quad(
        &mut m,
        [c(0., w, 0.), c(l, w, 0.), c(l, w, h), c(0., w, h)],
        Vec3::Y,
    );
    quad(
        &mut m,
        [c(0., 0., 0.), c(0., w, 0.), c(0., w, h), c(0., 0., h)],
        -Vec3::X,
    );
    quad(
        &mut m,
        [c(l, 0., 0.), c(l, w, 0.), c(l, w, h), c(l, 0., h)],
        Vec3::X,
    );
    // rim: four trapezoids at the top
    quad(
        &mut m,
        [c(0., 0., h), c(l, 0., h), c(l - t, t, h), c(t, t, h)],
        Vec3::Z,
    );
    quad(
        &mut m,
        [c(l, 0., h), c(l, w, h), c(l - t, w - t, h), c(l - t, t, h)],
        Vec3::Z,
    );
    quad(
        &mut m,
        [c(l, w, h), c(0., w, h), c(t, w - t, h), c(l - t, w - t, h)],
        Vec3::Z,
    );
    quad(
        &mut m,
        [c(0., w, h), c(0., 0., h), c(t, t, h), c(t, w - t, h)],
        Vec3::Z,
    );
    let outer = m.tris.len();
    // the cavity: four walls facing inwards and the floor facing up
    let (x0, x1, y0, y1) = (t, l - t, t, w - t);
    quad(
        &mut m,
        [c(x0, y0, t), c(x1, y0, t), c(x1, y0, h), c(x0, y0, h)],
        Vec3::Y,
    );
    quad(
        &mut m,
        [c(x0, y1, t), c(x1, y1, t), c(x1, y1, h), c(x0, y1, h)],
        -Vec3::Y,
    );
    quad(
        &mut m,
        [c(x0, y0, t), c(x0, y1, t), c(x0, y1, h), c(x0, y0, h)],
        Vec3::X,
    );
    quad(
        &mut m,
        [c(x1, y0, t), c(x1, y1, t), c(x1, y1, h), c(x1, y0, h)],
        -Vec3::X,
    );
    quad(
        &mut m,
        [c(x0, y0, t), c(x1, y0, t), c(x1, y1, t), c(x0, y1, t)],
        Vec3::Z,
    );
    let fitting = (0..m.tris.len()).map(|f| f >= outer).collect();
    (m, fitting)
}

/// Signed volume of a closed mesh (positive with outward faces), mm³.
fn volume(m: &TriMesh) -> f32 {
    (0..m.face_count())
        .map(|f| {
            let [a, b, c] = m.triangle(f);
            a.dot(b.cross(c)) / 6.0
        })
        .sum()
}

/// A 60 × 40 mm plate of 0.05 mm pixels, 50 mm tall.
fn small_profile() -> PrinterProfile {
    PrinterProfile {
        id: "test-60x40".into(),
        machine_name: "Test".into(),
        resolution_x: 1200,
        resolution_y: 800,
        pixel_pitch_x_um: 50.0,
        pixel_pitch_y_um: 50.0,
        build_mm: [60.0, 40.0, 50.0],
        layer_min_um: 10,
        layer_max_um: 200,
        mirror_x: false,
        mirror_y: false,
        machine: Some(MachineParams::dental_45()),
    }
}

fn settings() -> JobSettings {
    JobSettings {
        layer_um: 100,
        exposure_s: 2.5,
        bottom_exposure_s: 30.0,
        bottom_layers: 4,
        rest_before_exposure_s: 1.5,
        pwm: 255,
        resin_name: "Clear aligner".into(),
        volume_mm3: 0.0,
        weight_g: 0.0,
        print_time_s: 0,
    }
}

fn print_job(profile: PrinterProfile) -> PrintJob {
    PrintJob {
        profile,
        job: settings(),
        density_g_ml: 1.1,
        orient: OrientParams::default(),
        support: SupportParams::default(),
        layout: LayoutParams::default(),
    }
}

const TIME: &str = "2026-11-21 10:00:00";

#[test]
fn test_meshes_are_closed_and_outward() {
    let b = cuboid(Vec3::ZERO, Vec3::new(2.0, 3.0, 4.0));
    assert!((volume(&b) - 24.0).abs() < 1e-4);
    assert!(b.validate().is_watertight());
    let (c, fitting) = cup(Vec3::new(20.0, 12.0, 8.0), 1.0);
    assert!((volume(&c) - (20.0 * 12.0 * 8.0 - 18.0 * 10.0 * 7.0)).abs() < 1e-2);
    assert_eq!(fitting.iter().filter(|&&f| f).count(), 10);
}

#[test]
fn settings_are_validated() {
    let ok = print_job(small_profile());
    ok.validate().unwrap();
    let mut cases = Vec::new();
    let mut j = ok.clone();
    j.job.layer_um = 5;
    cases.push(j);
    let mut j = ok.clone();
    j.job.exposure_s = 0.0;
    cases.push(j);
    let mut j = ok.clone();
    j.job.bottom_exposure_s = f32::NAN;
    cases.push(j);
    let mut j = ok.clone();
    j.job.rest_before_exposure_s = -1.0;
    cases.push(j);
    let mut j = ok.clone();
    j.job.pwm = 0;
    cases.push(j);
    let mut j = ok.clone();
    j.job.resin_name = String::new();
    cases.push(j);
    let mut j = ok.clone();
    j.job.resin_name = "x".repeat(32);
    cases.push(j);
    let mut j = ok.clone();
    j.density_g_ml = 0.0;
    cases.push(j);
    for j in cases {
        let e = j.validate().unwrap_err();
        assert_eq!(e.code(), "print-invalid-settings", "{e}");
    }
    assert_eq!(prepare(&[], &ok).unwrap_err().code(), "print-no-parts");
}

#[test]
fn time_estimate_uses_the_machine_parameters() {
    let s = JobSettings {
        exposure_s: 2.0,
        bottom_exposure_s: 20.0,
        bottom_layers: 3,
        rest_before_exposure_s: 1.0,
        ..settings()
    };
    let m = MachineParams::dental_45();
    // 25 °C up to 45 °C at 10 °C in 180 s
    assert_eq!(preheat_s(Some(&m)), 450);
    assert_eq!(preheat_s(None), 0);
    let slow = |layers| estimate_time_s(&s, Some(&m), ReleaseMode::Slow, layers);
    let fast = |layers| estimate_time_s(&s, Some(&m), ReleaseMode::Fast, layers);
    // 450 + 3 × (20 + 1 + 8.42) + 7 × (2 + 1 + 8.42): every layer is one of
    // the first 50, tilted at 8.42 s in either mode (I212 A = C)
    assert_eq!(slow(10), 618);
    assert_eq!(fast(10), 618);
    // past layer 50 the tilt takes 4.41 s slow (I212 B), 2.43 s fast (D)
    assert_eq!(slow(60), 1149);
    assert_eq!(fast(60), 1129);
    assert_eq!(slow(0), 450);
    // fewer layers than bottom layers
    assert_eq!(slow(2), 509);
    // the machine's own rest in the first layers (I202 A) replaces the file's
    let mut rest = m.clone();
    rest.first_layers_rest_s = 3.0;
    assert_eq!(estimate_time_s(&s, Some(&rest), ReleaseMode::Slow, 10), 638);
    // without machine parameters: exposures and rests only
    assert_eq!(estimate_time_s(&s, None, ReleaseMode::Slow, 10), 84);
    assert_eq!(ASSUMED_RELEASE, ReleaseMode::Slow);
}

#[test]
fn the_machine_parameters_must_fit_the_job() {
    let ok = print_job(small_profile());
    ok.validate().unwrap();
    let mut machine_only = ok.clone();
    if let Some(m) = &mut machine_only.profile.machine {
        m.light_source = LightSource::MachineOnly;
    }
    assert_eq!(
        machine_only.validate().unwrap_err().code(),
        "print-invalid-settings"
    );
    let mut short_axis = ok.clone();
    if let Some(m) = &mut short_axis.profile.machine {
        m.z_travel_mm = 40.0;
    }
    assert_eq!(
        short_axis.validate().unwrap_err().code(),
        "print-invalid-settings"
    );
    // the Saturn: 230 mm of Z travel, 220 mm printed
    let saturn = PrinterProfile::saturn_4_ultra_16k();
    let m = saturn.machine.as_ref().unwrap();
    assert!(m.z_travel_mm >= f64::from(saturn.build_mm[2]));
    assert_eq!(saturn.build_mm[2], 220.0);
}

#[test]
fn plates_that_overflow_the_vat_are_refused() {
    let profile = small_profile();
    // 870 ml vat, 4 mm × 143 × 244 mm (139.568 ml) must stay in it
    check_vat(&profile, 0, 730.0).unwrap();
    match check_vat(&profile, 2, 731.0) {
        Err(e @ JobError::VatOverflow { plate: 2, .. }) => {
            assert_eq!(e.code(), "print-vat-overflow");
            assert!(e.to_string().contains("870 ml"), "{e}");
        }
        other => panic!("{other:?}"),
    }
    let mut unknown = profile.clone();
    unknown.machine = None;
    check_vat(&unknown, 0, 1e6).unwrap();
}

#[test]
fn file_time_is_utc() {
    assert_eq!(format_utc(0), "1970-01-01 00:00:00");
    assert_eq!(format_utc(951_782_400), "2000-02-29 00:00:00");
    assert_eq!(format_utc(1_793_445_605), "2026-10-31 11:20:05");
    assert_eq!(file_time_now().len(), 19);
}

#[test]
fn footprint_is_the_hull_seen_from_above() {
    let a = cuboid(Vec3::new(-2.0, -1.0, 5.0), Vec3::new(2.0, 1.0, 9.0));
    let b = cuboid(Vec3::new(-0.5, -0.5, 0.0), Vec3::new(0.5, 3.0, 5.0));
    let f = footprint("p", [&a, &b].into_iter());
    assert_eq!(f.height_mm, 9.0);
    assert_eq!(f.hull.len(), 6);
    assert!(f.hull.contains(&Vec2::new(0.5, 3.0)) && f.hull.contains(&Vec2::new(-2.0, -1.0)));
}

#[test]
fn quarter_turns_of_a_mesh_are_exact() {
    let mut m = cuboid(Vec3::new(1.0, 2.0, 0.0), Vec3::new(3.0, 5.0, 1.0));
    m.update_normals();
    let original = m.clone();
    turn_mesh(&mut m, 1);
    assert!(m.positions.iter().any(|p| *p == Vec3::new(-2.0, 1.0, 0.0)));
    for _ in 0..3 {
        turn_mesh(&mut m, 1);
    }
    assert_eq!(m, original);
}

/// A plate made by hand: a 10 × 8 × 4 mm box standing on the plate at
/// (20, 15), and a 2 × 2 × 3 mm post as its "support" beside it.
fn hand_plate() -> PreparedPlate {
    let mesh = cuboid(Vec3::new(15.0, 11.0, 0.0), Vec3::new(25.0, 19.0, 4.0));
    let post = cuboid(Vec3::new(30.0, 14.0, 0.0), Vec3::new(32.0, 16.0, 3.0));
    PreparedPlate {
        index: 0,
        parts: vec![PlacedPart {
            id: "box".into(),
            mesh,
            supports: vec![post],
            offset: Vec2::ZERO,
        }],
        height_mm: 4.0,
        resin_estimate_ml: 0.332,
    }
}

#[test]
fn plate_stl_holds_parts_and_supports() {
    let mut bytes = Vec::new();
    write_plate_stl(&hand_plate(), &mut bytes).unwrap();
    let m = dc_mesh::io::read_bytes(&bytes, MeshFormat::Stl, &Default::default()).unwrap();
    assert_eq!(m.face_count(), 24);
    assert_eq!(m.connected_components().len(), 2);
    assert!((volume(&m) - (320.0 + 12.0)).abs() < 1e-2);
}

#[test]
fn preview_shows_the_parts_from_above() {
    let p = small_profile();
    let img = preview(&hand_plate(), &p, 120);
    // 60 × 40 mm on 120 px: 2 px per mm, letterboxed 20 px from the top
    let at = |x: f32, y: f32| img[((20.0 + (40.0 - y) * 2.0) as usize) * 120 + (x * 2.0) as usize];
    assert_eq!(at(20.0, 15.0), PREVIEW_PART);
    assert_eq!(at(31.0, 15.0), PREVIEW_SUPPORT);
    assert_eq!(at(50.0, 30.0), PREVIEW_PLATE);
    assert_eq!(img[0], 0);
}

#[test]
fn plate_is_written_as_goo_with_volume_weight_and_time() {
    let job = print_job(small_profile());
    let plate = hand_plate();
    let mut calls = Vec::new();
    let mut bytes = Vec::new();
    let summary = write_plate_with(
        &plate,
        &job,
        &mut bytes,
        &mut |done, total| calls.push((done, total)),
        &|| false,
        TIME,
    )
    .unwrap();
    let file = GooFile::parse(&bytes).unwrap();
    assert_eq!(file.header.layer_count, summary.layers);
    assert_eq!(file.layers.len() as u32, summary.layers);
    // 4 mm at 0.1 mm
    assert_eq!(summary.layers, 40);
    assert_eq!(calls.len(), 40);
    assert_eq!(calls.last(), Some(&(40, 40)));
    assert_eq!(summary.parts, vec!["box".to_owned()]);
    assert_eq!(summary.height_mm, 4.0);
    // 320 + 12 mm³, within the pixel sampling
    assert!(
        (summary.volume_ml - 0.332).abs() < 0.332 * 0.02,
        "{}",
        summary.volume_ml
    );
    assert!((summary.weight_g - summary.volume_ml * 1.1).abs() < 1e-6);
    assert!((f64::from(file.header.volume_mm3) - summary.volume_ml * 1000.0).abs() < 1e-2);
    assert!((f64::from(file.header.weight_g) - summary.weight_g).abs() < 1e-4);
    assert_eq!(file.header.print_time_s, summary.est_time_s);
    let machine = MachineParams::dental_45();
    assert_eq!(
        summary.est_time_s,
        estimate_time_s(&job.job, Some(&machine), ASSUMED_RELEASE, 40)
    );
    assert_eq!(summary.tank_target_c, Some(45.0));
    assert_eq!(summary.preheat_s, 450);
    assert_eq!(summary.release, ReleaseMode::Slow);
    assert!((summary.resin_needed_ml - summary.volume_ml - 139.568).abs() < 1e-9);
    assert_eq!(file.header.file_time, TIME);
    assert_eq!(file.header.resin_name, "Clear aligner");
    assert!(file.header.big_preview.contains(&PREVIEW_PART));
    assert!(file.header.small_preview.contains(&PREVIEW_PART));
    assert_eq!(file.layers[0].settings.exposure_s, 30.0);
    assert_eq!(file.layers[39].settings.exposure_s, 2.5);

    // the same plate gives the same bytes
    let mut again = Vec::new();
    write_plate_with(&plate, &job, &mut again, &mut |_, _| {}, &|| false, TIME).unwrap();
    assert_eq!(bytes, again);
    // and stops when cancelled
    let e = write_plate_with(&plate, &job, Vec::new(), &mut |_, _| {}, &|| true, TIME);
    assert!(matches!(e, Err(JobError::Cancelled)));
}

/// The whole pipeline on cups (needs orientation, supports and the slicer).
#[test]
fn cups_are_prepared_checked_nested_and_written() {
    let (mesh, fitting) = cup(Vec3::new(20.0, 12.0, 8.0), 1.0);
    let parts: Vec<PrintPart> = (0..3)
        .map(|i| PrintPart {
            id: format!("cup-{i}"),
            mesh: mesh.clone(),
            fitting: Some(fitting.clone()),
        })
        .collect();
    let mut profile = small_profile();
    profile.build_mm = [60.0, 40.0, 50.0];
    let job = print_job(profile);
    let prepared = prepare(&parts, &job).unwrap();
    assert_eq!(prepared.parts.len(), 3);
    let placed: usize = prepared.plates.iter().map(|p| p.parts.len()).sum();
    assert_eq!(placed, 3);
    for (plan, part) in prepared.parts.iter().zip(&parts) {
        assert_eq!(plan.id, part.id);
        assert!(plan.contacts > 0, "{} has no supports", plan.id);
    }
    // every placed mesh is inside the plate margins
    for plate in &prepared.plates {
        for p in &plate.parts {
            for m in iter::once(&p.mesh).chain(&p.supports) {
                let b = m.aabb().unwrap();
                assert!(
                    b.min.x >= job.layout.margin_mm - 1e-3
                        && b.min.y >= job.layout.margin_mm - 1e-3
                );
                assert!(b.max.x <= 60.0 - job.layout.margin_mm + 1e-3);
                assert!(b.max.y <= 40.0 - job.layout.margin_mm + 1e-3);
                assert!(b.min.z >= -1e-4);
            }
        }
    }
    let mut first = Vec::new();
    let summary = write_plate_with(
        &prepared.plates[0],
        &job,
        &mut first,
        &mut |_, _| {},
        &|| false,
        TIME,
    )
    .unwrap();
    let cup_ml = f64::from(volume(&mesh)) / 1000.0;
    let n = prepared.plates[0].parts.len() as f64;
    assert!(
        summary.volume_ml >= n * cup_ml * 0.98,
        "{}",
        summary.volume_ml
    );
    GooFile::parse(&first).unwrap();

    // deterministic: same parts, same plates, same bytes
    let again = prepare(&parts, &job).unwrap();
    assert_eq!(again.parts, prepared.parts);
    let mut second = Vec::new();
    write_plate_with(
        &again.plates[0],
        &job,
        &mut second,
        &mut |_, _| {},
        &|| false,
        TIME,
    )
    .unwrap();
    assert_eq!(first, second);
}

/// The job's layer height drives the orientation and the supports, not the
/// `layer_um` their own parameters carry.
#[test]
fn the_job_layer_height_drives_orientation_and_supports() {
    let (mesh, fitting) = cup(Vec3::new(20.0, 12.0, 8.0), 1.0);
    let parts = [PrintPart {
        id: "cup".into(),
        mesh,
        fitting: Some(fitting),
    }];
    let mut job = print_job(small_profile());
    job.job.layer_um = 200;
    job.orient.layer_um = 200;
    job.support.layer_um = 200;
    let expected = prepare(&parts, &job).unwrap();
    job.orient.layer_um = 10;
    job.support.layer_um = 10;
    let got = prepare(&parts, &job).unwrap();
    assert_eq!(got.parts, expected.parts);
    let supports = |p: &PreparedPlates| -> Vec<Vec<TriMesh>> {
        p.plates
            .iter()
            .flat_map(|plate| &plate.parts)
            .map(|part| part.supports.clone())
            .collect()
    };
    assert_eq!(supports(&got), supports(&expected));
}

#[test]
fn overlapping_pieces_count_once() {
    // two 10 × 10 × 1 mm slabs overlapping on half their area
    let mut two = cuboid(Vec3::ZERO, Vec3::new(10.0, 10.0, 1.0));
    let other = cuboid(Vec3::new(5.0, 0.0, 0.0), Vec3::new(15.0, 10.0, 1.0));
    let base = two.positions.len() as u32;
    two.positions.extend_from_slice(&other.positions);
    two.tris
        .extend(other.tris.iter().map(|t| t.map(|i| i + base)));
    assert!((mesh_volume_mm3(&two) - 200.0).abs() < 1e-6);
    let union = union_volume_mm3(&two).unwrap();
    assert!((union - 150.0).abs() < 150.0 * 0.01, "{union}");
    assert_eq!(union_volume_mm3(&TriMesh::default()).unwrap(), 0.0);
}

/// The Saturn's plate and height with 0.1 mm pixels: real nesting, quick
/// slicing.
fn coarse_saturn() -> PrinterProfile {
    PrinterProfile {
        id: "saturn-coarse".into(),
        resolution_x: 2117,
        resolution_y: 1184,
        pixel_pitch_x_um: 100.0,
        pixel_pitch_y_um: 100.0,
        build_mm: [211.7, 118.4, 220.0],
        ..PrinterProfile::saturn_4_ultra_16k()
    }
}

/// The whole pipeline on synthetic aligner shells, with and without their
/// fitting flags (needs orientation, supports and the slicer).
#[test]
fn aligner_shells_are_prepared_and_written() {
    let shell = crate::testutil::default_shell();
    let parts = vec![
        shell.part("upper-01", true),
        shell.part("upper-02", true),
        shell.part("upper-03", false),
    ];
    let mut job = print_job(coarse_saturn());
    job.job.layer_um = 200;
    let prepared = prepare(&parts, &job).unwrap();
    let placed: usize = prepared.plates.iter().map(|p| p.parts.len()).sum();
    assert_eq!(placed, 3);
    // every part on one of the plates
    assert!(
        prepared
            .parts
            .iter()
            .all(|p| p.plate < prepared.plates.len())
    );
    for plan in &prepared.parts {
        assert!(plan.contacts > 0, "{} has no supports", plan.id);
        assert!(plan.height_mm <= 220.0);
    }
    let plate = &prepared.plates[0];
    let on_plate = plate.parts.len() as f64;
    let mut bytes = Vec::new();
    let summary =
        write_plate_with(plate, &job, &mut bytes, &mut |_, _| {}, &|| false, TIME).unwrap();
    let file = GooFile::parse(&bytes).unwrap();
    assert_eq!(file.header.layer_count, summary.layers);
    let shell_ml = f64::from(volume(&shell.mesh)) / 1000.0;
    // the shells and their supports
    assert!(
        summary.volume_ml > on_plate * shell_ml * 0.98,
        "{}",
        summary.volume_ml
    );
    let mut stl = Vec::new();
    write_plate_stl(plate, &mut stl).unwrap();
    let m = dc_mesh::io::read_bytes(&stl, MeshFormat::Stl, &Default::default()).unwrap();
    assert!(m.face_count() > plate.parts.len() * shell.mesh.face_count());
}
