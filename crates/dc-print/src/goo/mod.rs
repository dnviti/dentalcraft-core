//! Elegoo GOO V3 slice format (spec "Goo Format Spec V1.2"; plan §5).
//!
//! Big-endian throughout, IEEE-754 `f32`, fixed-length NUL-padded strings.
//! Layout: a header of exactly [`HEADER_SIZE`] bytes (two RGB565 previews
//! included), one record per layer, then [`ENDING`].
//!
//! A layer record is 64 bytes of settings (pause flag u16, pause Z, layer
//! Z, five timings, eight lift/retract floats, PWM u16), `0D 0A`, the image
//! size u32, the image ([`rle`]) and `0D 0A`. The research table puts the
//! fields after the layer Z one slot off (it lists six timings); the spec
//! and the 64-byte total say five.
//!
//! [`GooWriter`] streams: it takes each layer as runs, encodes only the RLE
//! bytes and writes them out, so no full layer bitmap is ever built.
//! [`GooFile::parse`] reads a whole file back (tests, `print.inspect`, the
//! golden comparison).

pub mod rle;

use std::io::Write;
use std::ops::Range;

pub use rle::Run;

use crate::profile::{PrinterProfile, layer_z_mm};

/// Size of the GOO V3 header, also the offset of the first layer.
pub const HEADER_SIZE: usize = 195_477;
/// Side of the small preview, in pixels.
pub const SMALL_PREVIEW: usize = 116;
/// Side of the big preview, in pixels.
pub const BIG_PREVIEW: usize = 290;
/// Header version string.
pub const VERSION: &[u8; 4] = b"V3.0";
/// Header magic (field 1).
pub const MAGIC: [u8; 8] = [0x07, 0x00, 0x00, 0x00, 0x44, 0x4C, 0x50, 0x00];
/// File ending (11 bytes).
pub const ENDING: [u8; 11] = [
    0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00, 0x44, 0x4C, 0x50, 0x00,
];
/// Size of a layer's settings block.
pub const LAYER_SETTINGS_SIZE: usize = 64;
const CRLF: [u8; 2] = [0x0D, 0x0A];

/// Byte offsets of the header fields (decimal, from the spec's field sizes).
pub mod offset {
    /// Version string (4).
    pub const VERSION: usize = 0;
    /// Magic (8).
    pub const MAGIC: usize = 4;
    /// Software name (32).
    pub const SOFTWARE_NAME: usize = 12;
    /// Software version (24).
    pub const SOFTWARE_VERSION: usize = 44;
    /// File time (24).
    pub const FILE_TIME: usize = 68;
    /// Printer name (32).
    pub const PRINTER_NAME: usize = 92;
    /// Printer type (32).
    pub const PRINTER_TYPE: usize = 124;
    /// Resin profile name (32).
    pub const RESIN_NAME: usize = 156;
    /// Anti-aliasing level u16.
    pub const AA_LEVEL: usize = 188;
    /// Small preview, 116×116 RGB565.
    pub const SMALL_PREVIEW: usize = 194;
    /// Big preview, 290×290 RGB565.
    pub const BIG_PREVIEW: usize = 27_108;
    /// Layer count u32.
    pub const LAYER_COUNT: usize = 195_310;
    /// Resolution X u16, then Y u16.
    pub const RESOLUTION_X: usize = 195_314;
    /// Mirror X u8, then mirror Y u8.
    pub const MIRROR_X: usize = 195_318;
    /// Platform X, Y, Z f32.
    pub const PLATFORM_X: usize = 195_320;
    /// Layer thickness f32.
    pub const LAYER_THICKNESS: usize = 195_332;
    /// Normal exposure f32.
    pub const EXPOSURE: usize = 195_336;
    /// Delay mode u8.
    pub const DELAY_MODE: usize = 195_340;
    /// Six wait times f32.
    pub const WAITS: usize = 195_345;
    /// Bottom exposure f32.
    pub const BOTTOM_EXPOSURE: usize = 195_369;
    /// Bottom layer count u32.
    pub const BOTTOM_LAYERS: usize = 195_373;
    /// Sixteen lift/retract f32 (two stages).
    pub const LIFT: usize = 195_377;
    /// Bottom PWM u16, then PWM u16.
    pub const BOTTOM_PWM: usize = 195_441;
    /// Advance mode u8.
    pub const ADVANCE_MODE: usize = 195_445;
    /// Print time u32 (s).
    pub const PRINT_TIME: usize = 195_446;
    /// Volume f32 (mm³).
    pub const VOLUME: usize = 195_450;
    /// Weight f32 (g).
    pub const WEIGHT: usize = 195_454;
    /// Price f32.
    pub const PRICE: usize = 195_458;
    /// Currency string (8).
    pub const CURRENCY: usize = 195_462;
    /// Offset of the first layer u32.
    pub const LAYER_OFFSET: usize = 195_470;
    /// Grey-scale range u8.
    pub const GREY_RANGE: usize = 195_474;
    /// Transition layer count u16.
    pub const TRANSITION_LAYERS: usize = 195_475;
}

/// Errors of the GOO codec. Stable codes for the `print.*` commands.
#[derive(Debug, thiserror::Error)]
pub enum GooError {
    /// Writing failed.
    #[error("goo-io: {0}")]
    Io(#[from] std::io::Error),
    /// The data ends early.
    #[error("goo-truncated")]
    Truncated,
    /// The header is not GOO V3.
    #[error("goo-bad-header")]
    BadHeader,
    /// A `0D 0A` delimiter or the ending is missing.
    #[error("goo-bad-delimiter at byte {0}")]
    BadDelimiter(usize),
    /// A layer image does not start with `0x55`.
    #[error("goo-bad-image-magic {0:#04x}")]
    BadImageMagic(u8),
    /// A layer image checksum does not match.
    #[error("goo-bad-checksum")]
    BadChecksum,
    /// A layer's runs do not cover exactly the pixel grid.
    #[error("goo-layer-coverage: layer {layer} covers {got} of {expected} pixels")]
    LayerCoverage {
        /// Layer index.
        layer: u32,
        /// Pixels in the grid.
        expected: u64,
        /// Pixels covered.
        got: u64,
    },
    /// More or fewer layers were written than the header announced.
    #[error("goo-layer-count: header says {expected}, got {got}")]
    LayerCount {
        /// Layers announced.
        expected: u32,
        /// Layers written.
        got: u32,
    },
    /// A value does not fit its field (string length, preview size, resolution).
    #[error("goo-field: {0}")]
    Field(&'static str),
}

/// Exposure and timing settings of one print, from the resin preset.
#[derive(Debug, Clone, PartialEq)]
pub struct JobSettings {
    /// Layer height in micrometres.
    pub layer_um: u32,
    /// Normal exposure, s.
    pub exposure_s: f32,
    /// Bottom exposure, s.
    pub bottom_exposure_s: f32,
    /// Number of bottom layers.
    pub bottom_layers: u32,
    /// Rest before exposure (wait after retract), s.
    pub rest_before_exposure_s: f32,
    /// Light PWM, 0–255.
    pub pwm: u16,
    /// Resin preset name (header field 7).
    pub resin_name: String,
    /// Total volume in mm³, supports included (the printer checks the resin
    /// level against it).
    pub volume_mm3: f32,
    /// Total weight in g.
    pub weight_g: f32,
    /// Estimated print time, s.
    pub print_time_s: u32,
}

/// The decoded GOO V3 header (every field, so vendor files round-trip).
#[derive(Debug, Clone, PartialEq)]
pub struct Header {
    /// Software name.
    pub software_name: String,
    /// Software version.
    pub software_version: String,
    /// File time, `YYYY-MM-DD HH:MM:SS`.
    pub file_time: String,
    /// Printer name.
    pub printer_name: String,
    /// Printer type.
    pub printer_type: String,
    /// Resin profile name.
    pub resin_name: String,
    /// Anti-aliasing level.
    pub aa_level: u16,
    /// Grey level.
    pub grey_level: u16,
    /// Blur level.
    pub blur_level: u16,
    /// Small preview, 116×116 RGB565, row-major.
    pub small_preview: Vec<u16>,
    /// Big preview, 290×290 RGB565, row-major.
    pub big_preview: Vec<u16>,
    /// Number of layers.
    pub layer_count: u32,
    /// Resolution X, px.
    pub resolution_x: u16,
    /// Resolution Y, px.
    pub resolution_y: u16,
    /// Mirror X flag.
    pub mirror_x: bool,
    /// Mirror Y flag.
    pub mirror_y: bool,
    /// Platform X, Y, Z, mm.
    pub platform_mm: [f32; 3],
    /// Layer thickness, mm.
    pub layer_thickness_mm: f32,
    /// Normal exposure, s.
    pub exposure_s: f32,
    /// Delay mode: 1 static waits, 0 light-off time.
    pub delay_mode: u8,
    /// Light-off time, s.
    pub light_off_s: f32,
    /// Bottom before-lift, after-lift, after-retract, then the same for
    /// normal layers, s.
    pub waits_s: [f32; 6],
    /// Bottom exposure, s.
    pub bottom_exposure_s: f32,
    /// Bottom layer count.
    pub bottom_layers: u32,
    /// Lift/retract distances and speeds, both stages (16 floats).
    pub lift: [f32; 16],
    /// Bottom light PWM.
    pub bottom_pwm: u16,
    /// Light PWM.
    pub pwm: u16,
    /// Advance (per-layer settings) mode.
    pub advance_mode: bool,
    /// Print time, s.
    pub print_time_s: u32,
    /// Volume, mm³.
    pub volume_mm3: f32,
    /// Weight, g.
    pub weight_g: f32,
    /// Price.
    pub price: f32,
    /// Currency.
    pub currency: String,
    /// Offset of the first layer.
    pub layer_offset: u32,
    /// Grey-scale range: 1 = 0x00–0xFF.
    pub grey_range: u8,
    /// Transition layer count.
    pub transition_layers: u16,
}

impl Header {
    /// The header DentalCraft writes for `profile` and `job`: black
    /// previews (set them before writing), lifts 0, advance mode 0, delay
    /// mode 1 with the rest before exposure, transition 0.
    pub fn new(
        profile: &PrinterProfile,
        job: &JobSettings,
        layer_count: u32,
        software_version: &str,
        file_time: &str,
    ) -> Result<Self, GooError> {
        let res = |v: u32| u16::try_from(v).map_err(|_| GooError::Field("resolution"));
        Ok(Self {
            software_name: "DentalCraft".into(),
            software_version: software_version.into(),
            file_time: file_time.into(),
            printer_name: profile.machine_name.clone(),
            printer_type: "DLP".into(),
            resin_name: job.resin_name.clone(),
            aa_level: 1,
            grey_level: 1,
            blur_level: 0,
            small_preview: vec![0; SMALL_PREVIEW * SMALL_PREVIEW],
            big_preview: vec![0; BIG_PREVIEW * BIG_PREVIEW],
            layer_count,
            resolution_x: res(profile.resolution_x)?,
            resolution_y: res(profile.resolution_y)?,
            mirror_x: profile.mirror_x,
            mirror_y: profile.mirror_y,
            platform_mm: profile.build_mm,
            layer_thickness_mm: layer_z_mm(0, job.layer_um),
            exposure_s: job.exposure_s,
            delay_mode: 1,
            light_off_s: 0.0,
            waits_s: [
                0.0,
                0.0,
                job.rest_before_exposure_s,
                0.0,
                0.0,
                job.rest_before_exposure_s,
            ],
            bottom_exposure_s: job.bottom_exposure_s,
            bottom_layers: job.bottom_layers,
            lift: [0.0; 16],
            bottom_pwm: job.pwm,
            pwm: job.pwm,
            advance_mode: false,
            print_time_s: job.print_time_s,
            volume_mm3: job.volume_mm3,
            weight_g: job.weight_g,
            price: 0.0,
            currency: "EUR".into(),
            layer_offset: HEADER_SIZE as u32,
            grey_range: 1,
            transition_layers: 0,
        })
    }

    /// Pixels in one layer image.
    pub fn pixel_count(&self) -> u64 {
        u64::from(self.resolution_x) * u64::from(self.resolution_y)
    }

    /// Serialises the header (exactly [`HEADER_SIZE`] bytes).
    pub fn to_bytes(&self) -> Result<Vec<u8>, GooError> {
        if self.small_preview.len() != SMALL_PREVIEW * SMALL_PREVIEW
            || self.big_preview.len() != BIG_PREVIEW * BIG_PREVIEW
        {
            return Err(GooError::Field("preview size"));
        }
        let mut b = Vec::with_capacity(HEADER_SIZE);
        b.extend_from_slice(VERSION);
        b.extend_from_slice(&MAGIC);
        put_str(&mut b, &self.software_name, 32)?;
        put_str(&mut b, &self.software_version, 24)?;
        put_str(&mut b, &self.file_time, 24)?;
        put_str(&mut b, &self.printer_name, 32)?;
        put_str(&mut b, &self.printer_type, 32)?;
        put_str(&mut b, &self.resin_name, 32)?;
        for v in [self.aa_level, self.grey_level, self.blur_level] {
            b.extend_from_slice(&v.to_be_bytes());
        }
        for p in &self.small_preview {
            b.extend_from_slice(&p.to_be_bytes());
        }
        b.extend_from_slice(&CRLF);
        for p in &self.big_preview {
            b.extend_from_slice(&p.to_be_bytes());
        }
        b.extend_from_slice(&CRLF);
        b.extend_from_slice(&self.layer_count.to_be_bytes());
        b.extend_from_slice(&self.resolution_x.to_be_bytes());
        b.extend_from_slice(&self.resolution_y.to_be_bytes());
        b.push(u8::from(self.mirror_x));
        b.push(u8::from(self.mirror_y));
        put_f32s(&mut b, &self.platform_mm);
        put_f32s(&mut b, &[self.layer_thickness_mm, self.exposure_s]);
        b.push(self.delay_mode);
        put_f32s(&mut b, &[self.light_off_s]);
        put_f32s(&mut b, &self.waits_s);
        put_f32s(&mut b, &[self.bottom_exposure_s]);
        b.extend_from_slice(&self.bottom_layers.to_be_bytes());
        put_f32s(&mut b, &self.lift);
        b.extend_from_slice(&self.bottom_pwm.to_be_bytes());
        b.extend_from_slice(&self.pwm.to_be_bytes());
        b.push(u8::from(self.advance_mode));
        b.extend_from_slice(&self.print_time_s.to_be_bytes());
        put_f32s(&mut b, &[self.volume_mm3, self.weight_g, self.price]);
        put_str(&mut b, &self.currency, 8)?;
        b.extend_from_slice(&self.layer_offset.to_be_bytes());
        b.push(self.grey_range);
        b.extend_from_slice(&self.transition_layers.to_be_bytes());
        debug_assert_eq!(b.len(), HEADER_SIZE);
        Ok(b)
    }

    /// Parses a header from the first [`HEADER_SIZE`] bytes of `data`.
    pub fn parse(data: &[u8]) -> Result<Self, GooError> {
        if data.len() < HEADER_SIZE {
            return Err(GooError::Truncated);
        }
        let mut r = Reader { data, pos: 0 };
        if r.take(4)? != VERSION || r.take(8)? != MAGIC {
            return Err(GooError::BadHeader);
        }
        let software_name = r.str(32)?;
        let software_version = r.str(24)?;
        let file_time = r.str(24)?;
        let printer_name = r.str(32)?;
        let printer_type = r.str(32)?;
        let resin_name = r.str(32)?;
        let aa_level = r.u16()?;
        let grey_level = r.u16()?;
        let blur_level = r.u16()?;
        let small_preview = r.u16s(SMALL_PREVIEW * SMALL_PREVIEW)?;
        r.crlf()?;
        let big_preview = r.u16s(BIG_PREVIEW * BIG_PREVIEW)?;
        r.crlf()?;
        Ok(Self {
            software_name,
            software_version,
            file_time,
            printer_name,
            printer_type,
            resin_name,
            aa_level,
            grey_level,
            blur_level,
            small_preview,
            big_preview,
            layer_count: r.u32()?,
            resolution_x: r.u16()?,
            resolution_y: r.u16()?,
            mirror_x: r.u8()? != 0,
            mirror_y: r.u8()? != 0,
            platform_mm: r.f32s()?,
            layer_thickness_mm: r.f32()?,
            exposure_s: r.f32()?,
            delay_mode: r.u8()?,
            light_off_s: r.f32()?,
            waits_s: r.f32s()?,
            bottom_exposure_s: r.f32()?,
            bottom_layers: r.u32()?,
            lift: r.f32s()?,
            bottom_pwm: r.u16()?,
            pwm: r.u16()?,
            advance_mode: r.u8()? != 0,
            print_time_s: r.u32()?,
            volume_mm3: r.f32()?,
            weight_g: r.f32()?,
            price: r.f32()?,
            currency: r.str(8)?,
            layer_offset: r.u32()?,
            grey_range: r.u8()?,
            transition_layers: r.u16()?,
        })
    }
}

/// The 64-byte settings block of one layer.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LayerSettings {
    /// 1 = pause at this layer.
    pub pause: u16,
    /// Pause position Z, mm.
    pub pause_z_mm: f32,
    /// Layer position Z, mm.
    pub z_mm: f32,
    /// Exposure, s.
    pub exposure_s: f32,
    /// Light-off time, s.
    pub light_off_s: f32,
    /// Wait before lift, s.
    pub wait_before_lift_s: f32,
    /// Wait after lift, s.
    pub wait_after_lift_s: f32,
    /// Wait after retract (rest before exposure), s.
    pub wait_after_retract_s: f32,
    /// Lift/retract distances and speeds, both stages.
    pub lift: [f32; 8],
    /// Light PWM.
    pub pwm: u16,
}

impl LayerSettings {
    /// The settings DentalCraft writes for layer `index`: Z exactly
    /// (index + 1)·h, bottom or normal exposure, lifts 0, no pause (pause Z
    /// is the machine height, as vendor files carry).
    pub fn for_layer(header: &Header, job: &JobSettings, index: u32) -> Self {
        let bottom = index < job.bottom_layers;
        Self {
            pause: 0,
            pause_z_mm: header.platform_mm[2],
            z_mm: layer_z_mm(index, job.layer_um),
            exposure_s: if bottom {
                job.bottom_exposure_s
            } else {
                job.exposure_s
            },
            light_off_s: 0.0,
            wait_before_lift_s: 0.0,
            wait_after_lift_s: 0.0,
            wait_after_retract_s: job.rest_before_exposure_s,
            lift: [0.0; 8],
            pwm: job.pwm,
        }
    }

    fn write(&self, b: &mut Vec<u8>) {
        b.extend_from_slice(&self.pause.to_be_bytes());
        put_f32s(
            b,
            &[
                self.pause_z_mm,
                self.z_mm,
                self.exposure_s,
                self.light_off_s,
                self.wait_before_lift_s,
                self.wait_after_lift_s,
                self.wait_after_retract_s,
            ],
        );
        put_f32s(b, &self.lift);
        b.extend_from_slice(&self.pwm.to_be_bytes());
    }

    fn read(r: &mut Reader<'_>) -> Result<Self, GooError> {
        Ok(Self {
            pause: r.u16()?,
            pause_z_mm: r.f32()?,
            z_mm: r.f32()?,
            exposure_s: r.f32()?,
            light_off_s: r.f32()?,
            wait_before_lift_s: r.f32()?,
            wait_after_lift_s: r.f32()?,
            wait_after_retract_s: r.f32()?,
            lift: r.f32s()?,
            pwm: r.u16()?,
        })
    }
}

/// Streaming GOO V3 writer: the header first, then one call per layer, then
/// [`GooWriter::finish`].
pub struct GooWriter<W: Write> {
    out: W,
    header: Header,
    job: JobSettings,
    written: u32,
    image: Vec<u8>,
    record: Vec<u8>,
}

impl<W: Write> GooWriter<W> {
    /// Writes `header` and returns a writer for its layers.
    pub fn new(mut out: W, header: Header, job: JobSettings) -> Result<Self, GooError> {
        out.write_all(&header.to_bytes()?)?;
        Ok(Self {
            out,
            header,
            job,
            written: 0,
            image: Vec::new(),
            record: Vec::with_capacity(LAYER_SETTINGS_SIZE + 6),
        })
    }

    /// Writes the next layer from its runs in file pixel order. The runs
    /// must cover exactly resolution X × Y pixels.
    pub fn write_layer(&mut self, runs: impl IntoIterator<Item = Run>) -> Result<(), GooError> {
        if self.written >= self.header.layer_count {
            return Err(GooError::LayerCount {
                expected: self.header.layer_count,
                got: self.written + 1,
            });
        }
        let got = rle::encode(runs, &mut self.image);
        let expected = self.header.pixel_count();
        if got != expected {
            return Err(GooError::LayerCoverage {
                layer: self.written,
                expected,
                got,
            });
        }
        let settings = LayerSettings::for_layer(&self.header, &self.job, self.written);
        self.record.clear();
        settings.write(&mut self.record);
        self.record.extend_from_slice(&CRLF);
        let size = u32::try_from(self.image.len()).map_err(|_| GooError::Field("layer size"))?;
        self.record.extend_from_slice(&size.to_be_bytes());
        self.out.write_all(&self.record)?;
        self.out.write_all(&self.image)?;
        self.out.write_all(&CRLF)?;
        self.written += 1;
        Ok(())
    }

    /// Writes the ending after the last layer and returns the output.
    pub fn finish(mut self) -> Result<W, GooError> {
        if self.written != self.header.layer_count {
            return Err(GooError::LayerCount {
                expected: self.header.layer_count,
                got: self.written,
            });
        }
        self.out.write_all(&ENDING)?;
        self.out.flush()?;
        Ok(self.out)
    }
}

/// One layer read back: its settings and its encoded image.
#[derive(Debug, Clone, PartialEq)]
pub struct GooLayer {
    /// The settings block.
    pub settings: LayerSettings,
    /// The image bytes (`0x55`, chunks, checksum).
    pub image: Vec<u8>,
}

impl GooLayer {
    /// Decodes the image into runs.
    pub fn runs(&self) -> Result<Vec<Run>, GooError> {
        rle::decode(&self.image)
    }
}

/// A whole GOO V3 file in memory.
#[derive(Debug, Clone, PartialEq)]
pub struct GooFile {
    /// The header.
    pub header: Header,
    /// The layers, in order.
    pub layers: Vec<GooLayer>,
}

impl GooFile {
    /// Parses a complete file, checking delimiters, the ending and each
    /// layer's coverage of the pixel grid.
    pub fn parse(data: &[u8]) -> Result<Self, GooError> {
        let header = Header::parse(data)?;
        let mut r = Reader {
            data,
            pos: usize::try_from(header.layer_offset).map_err(|_| GooError::BadHeader)?,
        };
        let mut layers = Vec::with_capacity(header.layer_count.min(100_000) as usize);
        for layer in 0..header.layer_count {
            let settings = LayerSettings::read(&mut r)?;
            r.crlf()?;
            let size = r.u32()? as usize;
            let image = r.take(size)?.to_vec();
            r.crlf()?;
            let got: u64 = rle::decode(&image)?
                .iter()
                .map(|run| u64::from(run.len))
                .sum();
            if got != header.pixel_count() {
                return Err(GooError::LayerCoverage {
                    layer,
                    expected: header.pixel_count(),
                    got,
                });
            }
            layers.push(GooLayer { settings, image });
        }
        if r.take(ENDING.len())? != ENDING || r.pos != data.len() {
            return Err(GooError::BadDelimiter(r.pos));
        }
        Ok(Self { header, layers })
    }
}

/// Builds a layer's runs in file pixel order from lit spans in plate pixel
/// coordinates. `spans(row)` gives the lit column ranges of plate row `row`,
/// sorted and disjoint. Columns and rows go through
/// [`PrinterProfile::map_pixel`], the single plate ↔ file mapping, so the
/// mirror flags are honoured.
pub fn runs_from_row_spans<'a>(
    profile: &PrinterProfile,
    spans: impl Fn(u32) -> &'a [Range<u32>],
) -> Vec<Run> {
    let width = profile.resolution_x;
    let mut runs: Vec<Run> = Vec::new();
    let push = |runs: &mut Vec<Run>, value: u8, len: u32| {
        if len == 0 {
            return;
        }
        match runs.last_mut() {
            Some(last) if last.value == value => last.len += len,
            _ => runs.push(Run::new(value, len)),
        }
    };
    // The mapping is an involution, so it also takes file rows to plate rows.
    for file_row in 0..profile.resolution_y {
        let Some((_, plate_row)) = profile.map_pixel(0, file_row) else {
            continue;
        };
        let lit = spans(plate_row);
        // File columns of each span; reversed (and each span flipped) when
        // the mapping mirrors X.
        let mapped = lit.iter().filter_map(|s| {
            let end = s.end.min(width);
            if s.start >= end {
                return None;
            }
            let (a, _) = profile.map_pixel(s.start, plate_row)?;
            let (b, _) = profile.map_pixel(end - 1, plate_row)?;
            Some((a.min(b), a.max(b) + 1))
        });
        let mut col = 0;
        let mut file_spans: Vec<(u32, u32)> = mapped.collect();
        file_spans.sort_unstable();
        for (start, end) in file_spans {
            push(&mut runs, 0x00, start.saturating_sub(col));
            push(&mut runs, 0xFF, end - start.max(col));
            col = end.max(col);
        }
        push(&mut runs, 0x00, width - col);
    }
    runs
}

fn put_str(b: &mut Vec<u8>, s: &str, len: usize) -> Result<(), GooError> {
    // keep one NUL so readers that expect a C string stop in the field
    if s.len() >= len || !s.is_ascii() {
        return Err(GooError::Field("string too long or not ASCII"));
    }
    b.extend_from_slice(s.as_bytes());
    b.resize(b.len() + len - s.len(), 0);
    Ok(())
}

fn put_f32s(b: &mut Vec<u8>, v: &[f32]) {
    for x in v {
        b.extend_from_slice(&x.to_be_bytes());
    }
}

struct Reader<'a> {
    data: &'a [u8],
    pos: usize,
}

impl<'a> Reader<'a> {
    fn take(&mut self, n: usize) -> Result<&'a [u8], GooError> {
        let end = self.pos.checked_add(n).ok_or(GooError::Truncated)?;
        let s = self.data.get(self.pos..end).ok_or(GooError::Truncated)?;
        self.pos = end;
        Ok(s)
    }
    fn array<const N: usize>(&mut self) -> Result<[u8; N], GooError> {
        let mut a = [0; N];
        a.copy_from_slice(self.take(N)?);
        Ok(a)
    }
    fn u8(&mut self) -> Result<u8, GooError> {
        Ok(self.take(1)?[0])
    }
    fn u16(&mut self) -> Result<u16, GooError> {
        Ok(u16::from_be_bytes(self.array()?))
    }
    fn u32(&mut self) -> Result<u32, GooError> {
        Ok(u32::from_be_bytes(self.array()?))
    }
    fn f32(&mut self) -> Result<f32, GooError> {
        Ok(f32::from_be_bytes(self.array()?))
    }
    fn f32s<const N: usize>(&mut self) -> Result<[f32; N], GooError> {
        let mut a = [0.0; N];
        for x in &mut a {
            *x = self.f32()?;
        }
        Ok(a)
    }
    fn u16s(&mut self, n: usize) -> Result<Vec<u16>, GooError> {
        Ok(self
            .take(n * 2)?
            .as_chunks::<2>()
            .0
            .iter()
            .map(|c| u16::from_be_bytes(*c))
            .collect())
    }
    fn str(&mut self, n: usize) -> Result<String, GooError> {
        let raw = self.take(n)?;
        let end = raw.iter().position(|&c| c == 0).unwrap_or(n);
        Ok(String::from_utf8_lossy(&raw[..end]).into_owned())
    }
    fn crlf(&mut self) -> Result<(), GooError> {
        let at = self.pos;
        if self.take(2)? != CRLF {
            return Err(GooError::BadDelimiter(at));
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests;
