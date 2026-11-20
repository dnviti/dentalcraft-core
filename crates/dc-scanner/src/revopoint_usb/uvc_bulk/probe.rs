//! The video-streaming interface of a UVC function, read from the USB
//! configuration descriptor, and its probe / commit control (UVC 1.5,
//! 3.9 and 4.3.1.1).

/// UVC request `SET_CUR`.
pub const SET_CUR: u8 = 0x01;
/// UVC request `GET_CUR`.
pub const GET_CUR: u8 = 0x81;
/// Video-streaming control selector `VS_PROBE_CONTROL`.
pub const VS_PROBE_CONTROL: u8 = 0x01;
/// Video-streaming control selector `VS_COMMIT_CONTROL`.
pub const VS_COMMIT_CONTROL: u8 = 0x02;

const DT_INTERFACE: u8 = 0x04;
const CS_INTERFACE: u8 = 0x24;
const CLASS_VIDEO: u8 = 0x0e;
const SUBCLASS_CONTROL: u8 = 0x01;
const SUBCLASS_STREAMING: u8 = 0x02;
const VC_HEADER: u8 = 0x01;
const VS_INPUT_HEADER: u8 = 0x01;
const VS_FORMAT_UNCOMPRESSED: u8 = 0x04;
const VS_FRAME_UNCOMPRESSED: u8 = 0x05;
const VS_FORMAT_FRAME_BASED: u8 = 0x10;
const VS_FRAME_FRAME_BASED: u8 = 0x11;

/// Tail of the GUID of the uncompressed formats named by a FOURCC
/// (`{fourcc}-0000-0010-8000-00AA00389B71`, descriptor byte order).
const GUID_TAIL: [u8; 12] = [
    0x00, 0x00, 0x10, 0x00, 0x80, 0x00, 0x00, 0xaa, 0x00, 0x38, 0x9b, 0x71,
];

/// The format GUID of `fourcc`, as the descriptors hold it.
pub fn format_guid(fourcc: u32) -> [u8; 16] {
    let mut guid = [0; 16];
    guid[..4].copy_from_slice(&fourcc.to_le_bytes());
    guid[4..].copy_from_slice(&GUID_TAIL);
    guid
}

/// The frame intervals a frame size offers, 100 ns units.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Intervals {
    /// A list of intervals.
    Discrete(Vec<u32>),
    /// Every interval from `min` to `max` by `step`.
    Continuous {
        /// Shortest interval.
        min: u32,
        /// Longest interval.
        max: u32,
        /// Granularity.
        step: u32,
    },
}

/// A frame descriptor (`VS_FRAME_UNCOMPRESSED` or `VS_FRAME_FRAME_BASED`).
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct FrameDesc {
    /// `bFrameIndex`.
    pub index: u8,
    /// Width in pixels.
    pub width: u16,
    /// Height in pixels.
    pub height: u16,
    /// `dwMaxVideoFrameBufferSize` (0 for frame-based formats, which do not
    /// say).
    pub max_bytes: u32,
    /// `dwDefaultFrameInterval`, 100 ns units.
    pub default_interval: u32,
    /// The intervals offered.
    pub intervals: Intervals,
}

impl FrameDesc {
    /// The interval closest to `fps` frames per second (the default one
    /// for 0), as `uvcvideo` picks it for `VIDIOC_S_PARM`.
    pub fn nearest_interval(&self, fps: u32) -> u32 {
        if fps == 0 {
            return self.default_interval;
        }
        let wanted = 10_000_000 / fps;
        match &self.intervals {
            Intervals::Discrete(list) => list
                .iter()
                .copied()
                .min_by_key(|i| i.abs_diff(wanted))
                .unwrap_or(self.default_interval),
            Intervals::Continuous { min, max, step } => {
                let wanted = wanted.clamp(*min, *max);
                let step = (*step).max(1);
                (min + (wanted - min + step / 2) / step * step).min(*max)
            }
        }
    }
}

/// A format descriptor with its frames.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct FormatDesc {
    /// `bFormatIndex`.
    pub index: u8,
    /// `guidFormat`.
    pub guid: [u8; 16],
    /// Its frame sizes.
    pub frames: Vec<FrameDesc>,
}

/// A video-streaming interface (alternate setting 0).
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct StreamingInterface {
    /// `bInterfaceNumber`.
    pub number: u8,
    /// `bEndpointAddress` of the video data (`VS_INPUT_HEADER`).
    pub endpoint: u8,
    /// `bcdUVC` of the video-control interface that lists it.
    pub uvc_version: u16,
    /// The formats it offers.
    pub formats: Vec<FormatDesc>,
}

impl StreamingInterface {
    /// Reads the video-streaming interface `number` out of the raw
    /// configuration descriptor `config` (with its class-specific
    /// descriptors). `None` when it is not a video-streaming interface with
    /// an input header, or no video-control header lists it.
    pub fn parse(config: &[u8], number: u8) -> Option<Self> {
        let mut uvc_version = None;
        let mut endpoint = None;
        let mut formats: Vec<FormatDesc> = Vec::new();
        // (number, alt setting, subclass) of the current interface
        let mut current: Option<(u8, u8, u8)> = None;
        let mut i = 0;
        while i + 2 <= config.len() {
            let len = usize::from(config[i]);
            if len < 2 || i + len > config.len() {
                break;
            }
            let d = &config[i..i + len];
            i += len;
            match d[1] {
                DT_INTERFACE if len >= 9 => {
                    current = (d[5] == CLASS_VIDEO).then_some((d[2], d[3], d[6]));
                }
                CS_INTERFACE if len >= 3 => match current {
                    Some((_, 0, SUBCLASS_CONTROL)) if d[2] == VC_HEADER && len >= 12 => {
                        let count = usize::from(d[11]);
                        if d.get(12..12 + count).is_some_and(|l| l.contains(&number)) {
                            uvc_version = Some(u16::from_le_bytes([d[3], d[4]]));
                        }
                    }
                    Some((n, 0, SUBCLASS_STREAMING)) if n == number => match d[2] {
                        VS_INPUT_HEADER if len >= 7 => endpoint = Some(d[6]),
                        VS_FORMAT_UNCOMPRESSED | VS_FORMAT_FRAME_BASED if len >= 21 => {
                            let mut guid = [0; 16];
                            guid.copy_from_slice(&d[5..21]);
                            formats.push(FormatDesc {
                                index: d[3],
                                guid,
                                frames: Vec::new(),
                            });
                        }
                        VS_FRAME_UNCOMPRESSED | VS_FRAME_FRAME_BASED => {
                            if let (Some(format), Some(frame)) = (formats.last_mut(), frame(d)) {
                                format.frames.push(frame);
                            }
                        }
                        _ => {}
                    },
                    _ => {}
                },
                _ => {}
            }
        }
        Some(Self {
            number,
            endpoint: endpoint?,
            uvc_version: uvc_version?,
            formats,
        })
    }

    /// The format with FOURCC `fourcc` and its `width x height` frame.
    pub fn find(&self, fourcc: u32, width: u32, height: u32) -> Option<(&FormatDesc, &FrameDesc)> {
        let guid = format_guid(fourcc);
        self.formats
            .iter()
            .filter(|f| f.guid == guid)
            .find_map(|f| {
                f.frames
                    .iter()
                    .find(|fr| u32::from(fr.width) == width && u32::from(fr.height) == height)
                    .map(|fr| (f, fr))
            })
    }
}

/// A frame descriptor (uncompressed or frame-based: the same fields up to
/// the default interval, then frame-based ones carry `dwBytesPerLine`).
fn frame(d: &[u8]) -> Option<FrameDesc> {
    let u32_at = |o: usize| Some(u32::from_le_bytes(d.get(o..o + 4)?.try_into().ok()?));
    let based = d[2] == VS_FRAME_FRAME_BASED;
    let (max_bytes, default_interval, kind) = if based {
        (0, u32_at(17)?, *d.get(21)?)
    } else {
        (u32_at(17)?, u32_at(21)?, *d.get(25)?)
    };
    let intervals = if kind == 0 {
        Intervals::Continuous {
            min: u32_at(26)?,
            max: u32_at(30)?,
            step: u32_at(34)?,
        }
    } else {
        Intervals::Discrete(
            (0..usize::from(kind))
                .map(|k| u32_at(26 + 4 * k))
                .collect::<Option<_>>()?,
        )
    };
    Some(FrameDesc {
        index: *d.get(3)?,
        width: u16::from_le_bytes([*d.get(5)?, *d.get(6)?]),
        height: u16::from_le_bytes([*d.get(7)?, *d.get(8)?]),
        max_bytes,
        default_interval,
        intervals,
    })
}

/// Bytes of the largest probe / commit control (UVC 1.5).
const CONTROL_MAX: usize = 48;
/// Bytes of a UVC 1.0 probe / commit control, the fields every version has.
pub const CONTROL_MIN: usize = 26;

/// The video probe and commit control (`VS_PROBE_CONTROL`,
/// `VS_COMMIT_CONTROL`), little-endian: `bmHint` (0), `bFormatIndex` (2),
/// `bFrameIndex` (3), `dwFrameInterval` (4), ..., `dwMaxVideoFrameSize`
/// (18), `dwMaxPayloadTransferSize` (22); 26 bytes in UVC 1.0, 34 in 1.1,
/// 48 in 1.5.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct StreamingControl {
    bytes: [u8; CONTROL_MAX],
    len: usize,
}

impl StreamingControl {
    /// Bytes of the control of a device of `bcdUVC` `uvc_version`.
    pub fn len_for(uvc_version: u16) -> usize {
        if uvc_version < 0x0110 {
            CONTROL_MIN
        } else if uvc_version < 0x0150 {
            34
        } else {
            CONTROL_MAX
        }
    }

    /// The probe of `format`, `frame` at `interval` (100 ns units) with
    /// only the frame interval fixed (`bmHint` 1); the device fills the
    /// rest.
    pub fn new(uvc_version: u16, format: u8, frame: u8, interval: u32) -> Self {
        let mut bytes = [0; CONTROL_MAX];
        bytes[0] = 1;
        bytes[2] = format;
        bytes[3] = frame;
        bytes[4..8].copy_from_slice(&interval.to_le_bytes());
        Self {
            bytes,
            len: Self::len_for(uvc_version),
        }
    }

    /// The control of `len` bytes read back from a device: `reply` holds at
    /// least the UVC 1.0 fields; a shorter reply than `len` is padded with
    /// zeros. `None` for fewer than [`CONTROL_MIN`] bytes.
    pub fn from_reply(reply: &[u8], len: usize) -> Option<Self> {
        if reply.len() < CONTROL_MIN {
            return None;
        }
        let len = len.clamp(CONTROL_MIN, CONTROL_MAX);
        let mut bytes = [0; CONTROL_MAX];
        let n = reply.len().min(len);
        bytes[..n].copy_from_slice(&reply[..n]);
        Some(Self { bytes, len })
    }

    /// The bytes sent.
    pub fn as_bytes(&self) -> &[u8] {
        &self.bytes[..self.len]
    }

    fn u32_at(&self, offset: usize) -> u32 {
        u32::from_le_bytes([
            self.bytes[offset],
            self.bytes[offset + 1],
            self.bytes[offset + 2],
            self.bytes[offset + 3],
        ])
    }

    /// `bFormatIndex`.
    pub fn format_index(&self) -> u8 {
        self.bytes[2]
    }

    /// `bFrameIndex`.
    pub fn frame_index(&self) -> u8 {
        self.bytes[3]
    }

    /// `dwFrameInterval`, 100 ns units.
    pub fn frame_interval(&self) -> u32 {
        self.u32_at(4)
    }

    /// `dwMaxVideoFrameSize`: bytes of one frame.
    pub fn max_video_frame_size(&self) -> u32 {
        self.u32_at(18)
    }

    /// `dwMaxPayloadTransferSize`: bytes of the largest payload.
    pub fn max_payload_transfer_size(&self) -> u32 {
        self.u32_at(22)
    }
}

#[cfg(test)]
pub(super) mod tests {
    use super::*;

    /// The configuration descriptor of the MINI 2 (`2207:110c`, bcdDevice
    /// 0x0419), one descriptor per line, as Linux read it from the device
    /// (sysfs `descriptors`, 2026-11-20): the depth camera (interfaces 0
    /// and 1), the colour camera (2 and 3), HID (4).
    pub(crate) fn mini2_config() -> Vec<u8> {
        let hex = concat!(
            "0902730205010480fa",
            "080b00020e030005",
            "09040000010e010005",
            "0d240100014f00006cdc020101",
            "1224020101020000000000000000030a0000",
            "0b240502010040025f1f00",
            "092403030101000200",
            "1c2406046d1a21c27e784a3cac067e434b676f9a0e010203ffa2a700",
            "07058103100008",
            "0525031000",
            "09040100010e020006",
            "0f2401020901820003000000010000",
            "1b240401025955593200001000800000aa00389b71100100000000",
            "26240501002003580200f0490200e0930400a60e002a2c0a00032a2c0a0040420f0080841e00",
            "26240502004006b00400c0270900804f1200983a002a2c0a00032a2c0a0040420f0080841e00",
            "1b240402035931362000001000800000aa00389b71100100000000",
            "26240501002003580200f0490200e0930400a60e002a2c0a00032a2c0a0040420f0080841e00",
            "26240502002003b00400e0930400c02709004c1d002a2c0a00032a2c0a0040420f0080841e00",
            "26240503006009580200d0dd0600a0bb0d00f22b002a2c0a00032a2c0a0040420f0080841e00",
            "06240d010104",
            "07058202000200",
            "080b02020e030008",
            "09040200010e010008",
            "0d240100014f00006cdc020103",
            "1224020101020000000000000000030a0000",
            "0b240502010040025f1f00",
            "092403030101000200",
            "1c2406046d1a21c27e784a3cac067e434b676f9a0e010203ffa2a700",
            "07058303100008",
            "0525031000",
            "09040300010e020009",
            "0e24010149008400030000000100",
            "0b24060101000100000000",
            "2a240701004006b00400c0270900804f1200983a0015160500041516050020a107002a2c0a0040420f00",
            "06240d010104",
            "07058402000200",
            "09040400020300000b",
            "092101010001222100",
            "07058503100004",
            "07050103000404",
        );
        (0..hex.len())
            .step_by(2)
            .map(|i| u8::from_str_radix(&hex[i..i + 2], 16).unwrap())
            .collect()
    }

    #[test]
    fn mini2_depth_interface() {
        let config = mini2_config();
        let vs = StreamingInterface::parse(&config, 1).unwrap();
        assert_eq!((vs.number, vs.endpoint, vs.uvc_version), (1, 0x82, 0x0100));
        assert_eq!(vs.formats.len(), 2);
        assert_eq!(&vs.formats[0].guid[..4], b"YUY2");
        let y16 = u32::from_le_bytes(*b"Y16 ");
        let (format, frame) = vs.find(y16, 800, 600).unwrap();
        assert_eq!((format.index, frame.index), (2, 1));
        assert_eq!(frame.max_bytes, 960_000);
        assert_eq!(
            frame.intervals,
            Intervals::Discrete(vec![666_666, 1_000_000, 2_000_000])
        );
        let sizes: Vec<(u8, u16, u16)> = format
            .frames
            .iter()
            .map(|f| (f.index, f.width, f.height))
            .collect();
        assert_eq!(sizes, [(1, 800, 600), (2, 800, 1200), (3, 2400, 600)]);
        assert_eq!(vs.find(y16, 640, 480), None);
        // control and HID interfaces are no streaming interfaces
        for number in [0, 2, 4] {
            assert_eq!(StreamingInterface::parse(&config, number), None);
        }
        // the colour stream (MJPEG, a format this reader does not take)
        let rgb = StreamingInterface::parse(&config, 3).unwrap();
        assert_eq!((rgb.endpoint, rgb.uvc_version), (0x84, 0x0100));
        assert!(rgb.formats.is_empty());
        // the extension unit is in the same descriptor
        assert!(crate::revopoint_usb::xu::has_unit(&config));
    }

    #[test]
    fn interval_choice() {
        let frame = FrameDesc {
            index: 1,
            width: 800,
            height: 600,
            max_bytes: 960_000,
            default_interval: 666_666,
            intervals: Intervals::Discrete(vec![666_666, 1_000_000, 2_000_000]),
        };
        assert_eq!(frame.nearest_interval(15), 666_666);
        assert_eq!(frame.nearest_interval(30), 666_666);
        assert_eq!(frame.nearest_interval(10), 1_000_000);
        assert_eq!(frame.nearest_interval(6), 2_000_000);
        assert_eq!(frame.nearest_interval(0), 666_666);
        let frame = FrameDesc {
            intervals: Intervals::Continuous {
                min: 333_333,
                max: 2_000_000,
                step: 333_333,
            },
            ..frame
        };
        assert_eq!(frame.nearest_interval(15), 666_666);
        assert_eq!(frame.nearest_interval(60), 333_333);
        assert_eq!(frame.nearest_interval(1), 1_999_998);
    }

    #[test]
    fn probe_layout() {
        let p = StreamingControl::new(0x0100, 2, 1, 666_666);
        assert_eq!(p.as_bytes().len(), 26);
        assert_eq!(&p.as_bytes()[..8], &[1, 0, 2, 1, 0x2a, 0x2c, 0x0a, 0x00]);
        assert!(p.as_bytes()[8..].iter().all(|&b| b == 0));
        assert_eq!(StreamingControl::len_for(0x0110), 34);
        assert_eq!(StreamingControl::len_for(0x0150), 48);
        // a reply: format 2, frame 1, 15 fps, 960000-byte frames in payloads
        // of at most 0x8000 bytes
        let mut reply = p.as_bytes().to_vec();
        reply[18..22].copy_from_slice(&960_000u32.to_le_bytes());
        reply[22..26].copy_from_slice(&0x8000u32.to_le_bytes());
        let c = StreamingControl::from_reply(&reply, 26).unwrap();
        assert_eq!((c.format_index(), c.frame_index()), (2, 1));
        assert_eq!(c.frame_interval(), 666_666);
        assert_eq!(c.max_video_frame_size(), 960_000);
        assert_eq!(c.max_payload_transfer_size(), 0x8000);
        assert_eq!(c.as_bytes(), &reply[..]);
        // a UVC 1.1 control answered with the 1.0 fields only: padded
        let c = StreamingControl::from_reply(&reply, 34).unwrap();
        assert_eq!(c.as_bytes().len(), 34);
        assert!(StreamingControl::from_reply(&reply[..20], 26).is_none());
    }
}
