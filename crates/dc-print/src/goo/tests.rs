use super::*;

fn tiny_profile() -> PrinterProfile {
    PrinterProfile {
        id: "tiny".into(),
        machine_name: "Tiny".into(),
        resolution_x: 8,
        resolution_y: 4,
        pixel_pitch_x_um: 14.0,
        pixel_pitch_y_um: 19.0,
        build_mm: [0.112, 0.076, 220.0],
        layer_min_um: 10,
        layer_max_um: 200,
        mirror_x: false,
        mirror_y: false,
        machine: None,
    }
}

fn job() -> JobSettings {
    JobSettings {
        layer_um: 50,
        exposure_s: 2.3,
        bottom_exposure_s: 32.0,
        bottom_layers: 1,
        rest_before_exposure_s: 1.2,
        pwm: 255,
        resin_name: "Clear aligner".into(),
        volume_mm3: 1234.5,
        weight_g: 1.42,
        print_time_s: 600,
    }
}

fn be_f32(b: &[u8], at: usize) -> f32 {
    f32::from_be_bytes(b[at..at + 4].try_into().unwrap())
}

fn be_u32(b: &[u8], at: usize) -> u32 {
    u32::from_be_bytes(b[at..at + 4].try_into().unwrap())
}

fn be_u16(b: &[u8], at: usize) -> u16 {
    u16::from_be_bytes(b[at..at + 2].try_into().unwrap())
}

#[test]
fn header_field_offsets_match_the_spec() {
    let profile = PrinterProfile::saturn_4_ultra_16k();
    let mut h = Header::new(&profile, &job(), 77, "0.3.0", "2026-11-20 22:00:00").unwrap();
    h.small_preview[0] = 0xF800;
    h.big_preview[BIG_PREVIEW * BIG_PREVIEW - 1] = 0x07E0;
    let b = h.to_bytes().unwrap();
    assert_eq!(b.len(), HEADER_SIZE);
    assert_eq!(&b[offset::VERSION..4], b"V3.0");
    assert_eq!(b[offset::MAGIC..offset::MAGIC + 8], MAGIC);
    assert_eq!(
        &b[offset::SOFTWARE_NAME..offset::SOFTWARE_NAME + 12],
        b"DentalCraft\0"
    );
    assert_eq!(
        &b[offset::SOFTWARE_VERSION..offset::SOFTWARE_VERSION + 5],
        b"0.3.0"
    );
    assert_eq!(&b[offset::FILE_TIME..offset::FILE_TIME + 4], b"2026");
    assert_eq!(
        &b[offset::PRINTER_NAME..offset::PRINTER_NAME + 18],
        b"Saturn 4 Ultra 16K"
    );
    assert_eq!(&b[offset::PRINTER_TYPE..offset::PRINTER_TYPE + 4], b"DLP\0");
    assert_eq!(&b[offset::RESIN_NAME..offset::RESIN_NAME + 5], b"Clear");
    assert_eq!(be_u16(&b, offset::AA_LEVEL), 1);
    assert_eq!(be_u16(&b, offset::SMALL_PREVIEW), 0xF800);
    assert_eq!(&b[offset::BIG_PREVIEW - 2..offset::BIG_PREVIEW], &CRLF);
    assert_eq!(be_u16(&b, offset::LAYER_COUNT - 4), 0x07E0);
    assert_eq!(&b[offset::LAYER_COUNT - 2..offset::LAYER_COUNT], &CRLF);
    assert_eq!(be_u32(&b, offset::LAYER_COUNT), 77);
    assert_eq!(be_u16(&b, offset::RESOLUTION_X), 15_120);
    assert_eq!(be_u16(&b, offset::RESOLUTION_X + 2), 6_230);
    assert_eq!(&b[offset::MIRROR_X..offset::MIRROR_X + 2], &[0, 0]);
    assert_eq!(be_f32(&b, offset::PLATFORM_X), 211.68);
    assert_eq!(be_f32(&b, offset::PLATFORM_X + 4), 118.37);
    assert_eq!(be_f32(&b, offset::PLATFORM_X + 8), 220.0);
    assert_eq!(be_f32(&b, offset::LAYER_THICKNESS), 0.05);
    assert_eq!(be_f32(&b, offset::EXPOSURE), 2.3);
    assert_eq!(b[offset::DELAY_MODE], 1);
    assert_eq!(be_f32(&b, offset::WAITS + 8), 1.2);
    assert_eq!(be_f32(&b, offset::WAITS + 20), 1.2);
    assert_eq!(be_f32(&b, offset::BOTTOM_EXPOSURE), 32.0);
    assert_eq!(be_u32(&b, offset::BOTTOM_LAYERS), 1);
    assert!(b[offset::LIFT..offset::BOTTOM_PWM].iter().all(|&x| x == 0));
    assert_eq!(be_u16(&b, offset::BOTTOM_PWM), 255);
    assert_eq!(be_u16(&b, offset::BOTTOM_PWM + 2), 255);
    assert_eq!(b[offset::ADVANCE_MODE], 0);
    assert_eq!(be_u32(&b, offset::PRINT_TIME), 600);
    assert_eq!(be_f32(&b, offset::VOLUME), 1234.5);
    assert_eq!(be_f32(&b, offset::WEIGHT), 1.42);
    assert_eq!(&b[offset::CURRENCY..offset::CURRENCY + 4], b"EUR\0");
    assert_eq!(be_u32(&b, offset::LAYER_OFFSET), HEADER_SIZE as u32);
    assert_eq!(b[offset::GREY_RANGE], 1);
    assert_eq!(be_u16(&b, offset::TRANSITION_LAYERS), 0);
    assert_eq!(offset::TRANSITION_LAYERS + 2, HEADER_SIZE);
    assert_eq!(Header::parse(&b).unwrap(), h);
}

/// Three layers of an 8 × 4 grid: a lit square, a diagonal, all lit.
fn three_layers() -> Vec<Vec<Run>> {
    let profile = tiny_profile();
    let square = 2..6;
    let l0 = runs_from_row_spans(&profile, |row| {
        if (1..3).contains(&row) {
            std::slice::from_ref(&square)
        } else {
            &[]
        }
    });
    let diag: Vec<Range<u32>> = (0..4).map(|r| r..r + 1).collect();
    let l1 = runs_from_row_spans(&profile, |row| std::slice::from_ref(&diag[row as usize]));
    let all = 0..8;
    let l2 = runs_from_row_spans(&profile, |_| std::slice::from_ref(&all));
    vec![l0, l1, l2]
}

#[test]
fn write_read_round_trip() {
    let profile = tiny_profile();
    let job = job();
    let layers = three_layers();
    let header = Header::new(&profile, &job, 3, "0.3.0", "2026-11-20 22:00:00").unwrap();
    let mut w = GooWriter::new(Vec::new(), header.clone(), job.clone()).unwrap();
    for runs in &layers {
        w.write_layer(runs.iter().copied()).unwrap();
    }
    let bytes = w.finish().unwrap();
    assert_eq!(&bytes[bytes.len() - ENDING.len()..], &ENDING);

    let file = GooFile::parse(&bytes).unwrap();
    assert_eq!(file.header, header);
    assert_eq!(file.layers.len(), 3);
    for (i, (layer, runs)) in file.layers.iter().zip(&layers).enumerate() {
        assert_eq!(&layer.runs().unwrap(), runs, "layer {i}");
        let s = layer.settings;
        assert_eq!(s.z_mm, layer_z_mm(i as u32, job.layer_um));
        assert_eq!(s.lift, [0.0; 8]);
        assert_eq!(s.pwm, 255);
        assert_eq!(s.wait_after_retract_s, 1.2);
        assert_eq!(s.exposure_s, if i == 0 { 32.0 } else { 2.3 });
    }
    // Layer 0's square: rows 1 and 2, columns 2..6.
    assert_eq!(
        layers[0],
        vec![
            Run::new(0, 10),
            Run::new(0xFF, 4),
            Run::new(0, 4),
            Run::new(0xFF, 4),
            Run::new(0, 10)
        ]
    );
    // Layer record: 64-byte settings then 0D 0A at the first layer.
    assert_eq!(&bytes[HEADER_SIZE + 64..HEADER_SIZE + 66], &CRLF);
    assert_eq!(be_f32(&bytes, HEADER_SIZE + 6), 0.05);
}

#[test]
fn layer_z_is_monotonic_in_the_file() {
    let profile = tiny_profile();
    let job = JobSettings {
        layer_um: 30,
        ..job()
    };
    let n = 200;
    let header = Header::new(&profile, &job, n, "0.3.0", "t").unwrap();
    let mut w = GooWriter::new(Vec::new(), header, job.clone()).unwrap();
    for _ in 0..n {
        w.write_layer([Run::new(0, 32)]).unwrap();
    }
    let file = GooFile::parse(&w.finish().unwrap()).unwrap();
    let z: Vec<f32> = file.layers.iter().map(|l| l.settings.z_mm).collect();
    assert!(z.windows(2).all(|p| p[1] > p[0]));
    assert_eq!(z[0], 0.03);
    assert_eq!(z[n as usize - 1], 6.0);
}

#[test]
fn coverage_and_count_are_enforced() {
    let profile = tiny_profile();
    let header = Header::new(&profile, &job(), 1, "0.3.0", "t").unwrap();
    let mut w = GooWriter::new(Vec::new(), header.clone(), job()).unwrap();
    assert!(matches!(
        w.write_layer([Run::new(0, 31)]),
        Err(GooError::LayerCoverage { got: 31, .. })
    ));
    let w = GooWriter::new(Vec::new(), header, job()).unwrap();
    assert!(matches!(
        w.finish(),
        Err(GooError::LayerCount {
            expected: 1,
            got: 0
        })
    ));
}

#[test]
fn mirror_flags_flip_the_spans() {
    let mut profile = tiny_profile();
    profile.mirror_x = true;
    profile.mirror_y = true;
    let lit = 0..2;
    // Plate row 0, columns 0..2 → file row 3, columns 6..8.
    let runs = runs_from_row_spans(&profile, |row| {
        if row == 0 {
            std::slice::from_ref(&lit)
        } else {
            &[]
        }
    });
    assert_eq!(runs, vec![Run::new(0, 30), Run::new(0xFF, 2)]);
}
