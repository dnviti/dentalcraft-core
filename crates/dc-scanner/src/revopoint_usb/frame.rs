//! Layout of the MINI 2 `Y16` depth stream (observed, not documented by the
//! vendor).
//!
//! Every mode carries the depth image first: 800x600 little-endian `u16`
//! values (960 000 bytes). Row 0 of the depth image is not depth but a
//! header ([`FrameHeader`]). The modes with infrared add the two rectified
//! 8-bit 800x600 images of the stereo pair, one after the other, at the end
//! of the buffer:
//!
//! | mode | `Y16` size | bytes 0..960k | 960k..1.92M | 1.92M..2.88M |
//! |---|---|---|---|---|
//! | [`StreamMode::Full`] | 2400x600 | depth | zeros | IR pair |
//! | [`StreamMode::DepthIr`] | 800x1200 | depth | IR pair | - |
//! | [`StreamMode::Depth`] | 800x600 | depth | - | - |
//!
//! The depth image is registered to the first image of the pair.

use glam::{Quat, Vec3};
use serde::{Deserialize, Serialize};

use crate::{Result, ScanError};

/// Width of the depth image (pixels).
pub const DEPTH_WIDTH: u32 = 800;
/// Height of the depth image (pixels), row 0 (the header) included.
pub const DEPTH_HEIGHT: u32 = 600;
/// Bytes of the depth image (`u16` per pixel).
const DEPTH_BYTES: usize = DEPTH_WIDTH as usize * DEPTH_HEIGHT as usize * 2;
/// Bytes of one infrared image (`u8` per pixel).
const IR_BYTES: usize = DEPTH_WIDTH as usize * DEPTH_HEIGHT as usize;
/// Bytes of the header in row 0 (the rest of the row is zero).
const HEADER_BYTES: usize = 84;
/// The first ten `u16` words of every header seen so far, except word
/// [`HEADER_VARYING_WORD`].
const HEADER_MAGIC: [u16; 10] = [0, 0, 0, 0x100, 0x160, 0, 12, 0, 0, 0];
/// The header word that is not constant: `0x160` on 2026-11-16, `0x148` on
/// 2026-11-18, on the same unit and firmware (meaning unknown, maybe a
/// sensor reading). It is not checked.
const HEADER_VARYING_WORD: usize = 4;

/// A `Y16` format of the depth node. The device offers the three sizes at
/// 5, 10 and 15 fps nominal, but paces the frames itself: on one unit it
/// delivered 6.5 fps in [`Self::Full`] and 10.7 fps in the two others, and
/// later the same day 2.7 fps in every mode (device timestamps 365 ms
/// apart, probably a longer auto exposure on a darker scene).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum StreamMode {
    /// 2400x600: depth, an empty plane, the infrared pair.
    #[default]
    Full,
    /// 800x1200: depth and the infrared pair.
    DepthIr,
    /// 800x600: depth only.
    Depth,
}

impl StreamMode {
    /// `Y16` image size (width, height) to negotiate.
    pub fn size(self) -> (u32, u32) {
        match self {
            Self::Full => (2400, 600),
            Self::DepthIr => (800, 1200),
            Self::Depth => (800, 600),
        }
    }

    /// Bytes of one frame.
    pub fn frame_bytes(self) -> usize {
        let (w, h) = self.size();
        w as usize * h as usize * 2
    }

    /// Byte offset of the infrared pair, when the mode carries it.
    fn ir_offset(self) -> Option<usize> {
        match self {
            Self::Full => Some(2 * DEPTH_BYTES),
            Self::DepthIr => Some(DEPTH_BYTES),
            Self::Depth => None,
        }
    }
}

/// Header of a depth frame (row 0), little endian. Offsets observed on
/// firmware with `bcdDevice` 0x0419:
///
/// * bytes 0..20: ten `u16` words, always [`HEADER_MAGIC`] except word
///   [`HEADER_VARYING_WORD`];
/// * bytes 20..24: `u32` device time in milliseconds;
/// * then `f32`s: orientation quaternion `w, x, y, z` (24..40), gyroscope
///   (40..52), accelerometer in g (52..64, about `(0, 0, 1)` at rest),
///   magnetometer (64..76), one unknown value and a zero.
///
/// The quaternion order is inferred: with the accelerometer along +z at
/// rest, `w, x, y, z` gives a rotation about the vertical (a heading),
/// `x, y, z, w` would tilt the vertical away from gravity.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FrameHeader {
    /// Device time (ms).
    pub timestamp_ms: u32,
    /// Orientation of the device IMU, normalised; `None` when the stored
    /// quaternion is not a rotation.
    pub orientation: Option<Quat>,
    /// Gyroscope reading (device units).
    pub gyro: Vec3,
    /// Accelerometer reading (g).
    pub accel_g: Vec3,
    /// Magnetometer reading (device units).
    pub magnetometer: Vec3,
}

/// An accelerometer reading this close to 1 g (relative) is taken as
/// gravity alone: the device is at rest or moving slowly.
const REST_G_TOLERANCE: f32 = 0.1;

impl FrameHeader {
    /// The up direction in the frame of the IMU (unit): the accelerometer
    /// reading, when it is about 1 g.
    pub fn up(&self) -> Option<Vec3> {
        let g = self.accel_g.length();
        ((g - 1.0).abs() <= REST_G_TOLERANCE).then(|| self.accel_g / g)
    }
}

/// Parses the header in the bytes of row 0. `None` when the row is too
/// short or does not start with the known header words.
pub fn parse_header(row0: &[u8]) -> Option<FrameHeader> {
    let bytes = row0.get(..HEADER_BYTES)?;
    let word = |i: usize| u16::from_le_bytes([bytes[2 * i], bytes[2 * i + 1]]);
    if (0..HEADER_MAGIC.len()).any(|i| i != HEADER_VARYING_WORD && word(i) != HEADER_MAGIC[i]) {
        return None;
    }
    let f = |i: usize| {
        let o = 24 + 4 * i;
        f32::from_le_bytes([bytes[o], bytes[o + 1], bytes[o + 2], bytes[o + 3]])
    };
    let vec3 = |i: usize| Vec3::new(f(i), f(i + 1), f(i + 2));
    let q = Quat::from_xyzw(f(1), f(2), f(3), f(0));
    let orientation = (q.is_finite() && (q.length() - 1.0).abs() < 0.05).then(|| q.normalize());
    Some(FrameHeader {
        timestamp_ms: u32::from_le_bytes([bytes[20], bytes[21], bytes[22], bytes[23]]),
        orientation,
        gyro: vec3(4),
        accel_g: vec3(7),
        magnetometer: vec3(10),
    })
}

/// The two rectified infrared images of a frame, 800x600, one byte per
/// pixel, row-major.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct IrPair<'a> {
    /// The image the depth is registered to.
    pub first: &'a [u8],
    /// The other camera.
    pub second: &'a [u8],
}

/// A frame split into its planes (borrowed from the buffer).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Planes<'a> {
    /// The header of row 0, when it is recognised.
    pub header: Option<FrameHeader>,
    /// The depth image, 800x600 little-endian `u16`, row 0 included.
    pub depth: &'a [u8],
    /// The infrared pair, in the modes that carry it.
    pub ir: Option<IrPair<'a>>,
}

/// Splits a frame of `mode`.
///
/// # Errors
/// [`ScanError::Device`] when the buffer is shorter than a frame of `mode`.
pub fn split_planes(frame: &[u8], mode: StreamMode) -> Result<Planes<'_>> {
    if frame.len() < mode.frame_bytes() {
        let (w, h) = mode.size();
        return Err(ScanError::Device(format!(
            "frame of {} bytes, a Y16 {w}x{h} frame has {}",
            frame.len(),
            mode.frame_bytes()
        )));
    }
    let depth = &frame[..DEPTH_BYTES];
    let ir = mode.ir_offset().map(|o| IrPair {
        first: &frame[o..o + IR_BYTES],
        second: &frame[o + IR_BYTES..o + 2 * IR_BYTES],
    });
    Ok(Planes {
        header: parse_header(&depth[..DEPTH_WIDTH as usize * 2]),
        depth,
        ir,
    })
}

/// The depth image (800x600 little-endian `u16`) in millimetres:
/// `raw * scale_mm`. Raw `0` (no depth) and row 0 (the header) are `0`.
pub fn depth_to_mm(depth: &[u8], scale_mm: f32) -> Vec<f32> {
    let row = DEPTH_WIDTH as usize;
    depth
        .as_chunks::<2>()
        .0
        .iter()
        .enumerate()
        .map(|(i, b)| {
            let raw = u16::from_le_bytes(*b);
            if i < row || raw == 0 {
                0.0
            } else {
                f32::from(raw) * scale_mm
            }
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Row 0 of a real 2400x600 frame of the MINI 2 (serial
    /// `A26307475A6H01AB8`, `bcdDevice` 0x0419), scanner at rest on a desk.
    const ROW0: &[u8] = include_bytes!("../../tests/data/mini2_header_row0.bin");

    /// A synthetic frame of `mode` with the real header, depth `depth(x, y)`
    /// for rows 1.., and infrared images filled with 1 and 2.
    fn frame(mode: StreamMode, depth: impl Fn(u32, u32) -> u16) -> Vec<u8> {
        let mut f = vec![0_u8; mode.frame_bytes()];
        f[..ROW0.len()].copy_from_slice(ROW0);
        for y in 1..DEPTH_HEIGHT {
            for x in 0..DEPTH_WIDTH {
                let o = 2 * (y * DEPTH_WIDTH + x) as usize;
                f[o..o + 2].copy_from_slice(&depth(x, y).to_le_bytes());
            }
        }
        if let Some(o) = mode.ir_offset() {
            f[o..o + IR_BYTES].fill(1);
            f[o + IR_BYTES..o + 2 * IR_BYTES].fill(2);
        }
        f
    }

    #[test]
    fn header_of_a_real_frame() {
        assert_eq!(ROW0.len(), 1600);
        let h = parse_header(ROW0).unwrap();
        assert_eq!(h.timestamp_ms, 35_428_028);
        let q = h.orientation.unwrap();
        // stored (0.7908, -0.0011, 0.0193, 0.6090), norm 0.998, normalised
        assert!(
            (q.w - 0.7922).abs() < 1e-3 && (q.z - 0.6100).abs() < 1e-3,
            "{q:?}"
        );
        assert!(q.is_normalized());
        // at rest: about 1 g, along +z; the orientation keeps +z vertical
        assert!((h.accel_g.length() - 1.0).abs() < 0.02, "{:?}", h.accel_g);
        assert!(h.accel_g.z > 0.99);
        assert!(q.mul_vec3(Vec3::Z).z > 0.99);
        assert!(h.gyro.length() < 0.05);
        assert!(h.magnetometer.length() > 10.0);
        let up = h.up().unwrap();
        assert!(up.is_normalized() && up.z > 0.99, "{up}");
        // a shaken device measures more than gravity: no up
        let shaken = FrameHeader {
            accel_g: 1.5 * h.accel_g,
            ..h
        };
        assert_eq!(shaken.up(), None);

        // word 4 varies between sessions (0x148 on 2026-11-18): still a header
        let mut varying = ROW0.to_vec();
        varying[8..10].copy_from_slice(&0x148_u16.to_le_bytes());
        assert_eq!(parse_header(&varying).unwrap().timestamp_ms, 35_428_028);
        // not a header: other words, or too short
        let mut other = ROW0.to_vec();
        other[6] = 7;
        assert_eq!(parse_header(&other), None);
        assert_eq!(parse_header(&ROW0[..40]), None);
        // a header whose quaternion is not a rotation has no orientation
        let mut zero_q = ROW0.to_vec();
        zero_q[24..40].fill(0);
        assert_eq!(parse_header(&zero_q).unwrap().orientation, None);
    }

    #[test]
    fn planes_of_each_mode() {
        for mode in [StreamMode::Full, StreamMode::DepthIr, StreamMode::Depth] {
            let (w, h) = mode.size();
            assert_eq!(mode.frame_bytes(), (w * h * 2) as usize);
            let f = frame(mode, |x, _| x as u16);
            let p = split_planes(&f, mode).unwrap();
            assert_eq!(p.header.unwrap().timestamp_ms, 35_428_028);
            assert_eq!(p.depth.len(), DEPTH_BYTES);
            assert_eq!(&p.depth[2 * 801..2 * 801 + 2], &1_u16.to_le_bytes());
            match p.ir {
                Some(ir) => {
                    assert_ne!(mode, StreamMode::Depth);
                    assert_eq!((ir.first.len(), ir.second.len()), (IR_BYTES, IR_BYTES));
                    assert!(ir.first.iter().all(|&v| v == 1));
                    assert!(ir.second.iter().all(|&v| v == 2));
                }
                None => assert_eq!(mode, StreamMode::Depth),
            }
            let e = split_planes(&f[..f.len() - 1], mode).unwrap_err();
            assert_eq!(e.code(), "device");
        }
        // the 2400x600 layout: depth, an empty plane, then the pair
        assert_eq!(StreamMode::Full.ir_offset(), Some(1_920_000));
        assert_eq!(StreamMode::DepthIr.ir_offset(), Some(960_000));
    }

    #[test]
    fn depth_conversion_skips_the_header_and_holes() {
        let f = frame(
            StreamMode::Depth,
            |x, y| if x == 5 { 0 } else { (y * 100) as u16 },
        );
        let mm = depth_to_mm(split_planes(&f, StreamMode::Depth).unwrap().depth, 0.01);
        assert_eq!(mm.len(), (DEPTH_WIDTH * DEPTH_HEIGHT) as usize);
        // row 0 is the header, not depth
        assert!(mm[..DEPTH_WIDTH as usize].iter().all(|&d| d == 0.0));
        let at = |x: u32, y: u32| mm[(y * DEPTH_WIDTH + x) as usize];
        assert_eq!(at(5, 10), 0.0);
        assert!((at(6, 10) - 10.0).abs() < 1e-4);
        assert!((at(0, 599) - 599.0).abs() < 1e-3);
    }
}
