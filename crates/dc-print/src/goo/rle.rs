//! GOO V3 layer image RLE (spec V1.2 §"layer content").
//!
//! An encoded image is `0x55`, the chunks, then one checksum byte. Pixels go
//! row-major over the whole layer. Chunk byte 0, bits 7..6:
//! - `00`: run of 0x00; `11`: run of 0xFF; `01`: run of a grey value, the
//!   value in the next byte. Bits 5..4 give 0–3 extra length bytes; the
//!   length is bits 3..0 plus the extra bytes big-endian shifted left by 4
//!   (up to 2^28 − 1). For grey runs the value byte precedes the length bytes.
//! - `10`: difference from the previous pixel value, |diff| in bits 3..0;
//!   bits 5..4: `00` +diff once, `01` +diff with the run in byte 1, `10`
//!   −diff once, `11` −diff with the run in byte 1. Only decoded here.
//!
//! The checksum is the bitwise NOT of the 8-bit sum of every byte between
//! `0x55` and the checksum.

use super::GooError;

/// Start byte of an encoded layer image.
pub const IMAGE_MAGIC: u8 = 0x55;
/// Longest run one chunk can carry (28 length bits).
pub const MAX_RUN: u32 = (1 << 28) - 1;

/// `len` consecutive pixels of grey `value` (0x00 black, 0xFF lit).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Run {
    /// Pixel value.
    pub value: u8,
    /// Number of pixels; never 0 in decoded output.
    pub len: u32,
}

impl Run {
    /// A run of `len` pixels of `value`.
    pub const fn new(value: u8, len: u32) -> Self {
        Self { value, len }
    }
}

/// The GOO checksum of the chunk bytes (between `0x55` and the checksum).
pub fn checksum(chunks: &[u8]) -> u8 {
    !chunks.iter().fold(0u8, |s, b| s.wrapping_add(*b))
}

/// Appends one chunk of `len` (1..=[`MAX_RUN`]) pixels of `value`, with the
/// fewest length bytes.
fn push_chunk(out: &mut Vec<u8>, value: u8, len: u32) {
    debug_assert!((1..=MAX_RUN).contains(&len));
    let kind: u8 = match value {
        0x00 => 0b00,
        0xFF => 0b11,
        _ => 0b01,
    };
    let extra: u8 = match len {
        0..=0xF => 0,
        0x10..=0xFFF => 1,
        0x1000..=0xF_FFFF => 2,
        _ => 3,
    };
    out.push(kind << 6 | extra << 4 | (len & 0xF) as u8);
    if kind == 0b01 {
        out.push(value);
    }
    for k in (0..extra).rev() {
        out.push((len >> (4 + 8 * u32::from(k))) as u8);
    }
}

/// Encodes `runs` as a complete layer image (`0x55`, chunks, checksum) into
/// `out`, which is cleared first. Zero-length runs are skipped, runs of the
/// same value are merged and runs longer than [`MAX_RUN`] are split.
/// Returns the number of pixels covered.
pub fn encode(runs: impl IntoIterator<Item = Run>, out: &mut Vec<u8>) -> u64 {
    out.clear();
    out.push(IMAGE_MAGIC);
    let mut total = 0u64;
    let mut pending: Option<(u8, u64)> = None;
    let flush = |out: &mut Vec<u8>, value: u8, mut len: u64| {
        while len > 0 {
            let n = len.min(u64::from(MAX_RUN));
            push_chunk(out, value, n as u32);
            len -= n;
        }
    };
    for run in runs {
        if run.len == 0 {
            continue;
        }
        total += u64::from(run.len);
        match pending {
            Some((v, ref mut n)) if v == run.value => *n += u64::from(run.len),
            _ => {
                if let Some((v, n)) = pending {
                    flush(out, v, n);
                }
                pending = Some((run.value, u64::from(run.len)));
            }
        }
    }
    if let Some((v, n)) = pending {
        flush(out, v, n);
    }
    let sum = checksum(&out[1..]);
    out.push(sum);
    total
}

/// Decodes a complete layer image into runs (merged where the value
/// repeats). Checks the start byte and the checksum.
pub fn decode(data: &[u8]) -> Result<Vec<Run>, GooError> {
    let (&first, rest) = data.split_first().ok_or(GooError::Truncated)?;
    if first != IMAGE_MAGIC {
        return Err(GooError::BadImageMagic(first));
    }
    let (&sum, chunks) = rest.split_last().ok_or(GooError::Truncated)?;
    if checksum(chunks) != sum {
        return Err(GooError::BadChecksum);
    }
    let mut runs: Vec<Run> = Vec::new();
    let push = |runs: &mut Vec<Run>, value: u8, len: u32| {
        if len == 0 {
            return;
        }
        match runs.last_mut() {
            // saturating: a hostile file then fails the coverage check
            Some(last) if last.value == value => last.len = last.len.saturating_add(len),
            _ => runs.push(Run::new(value, len)),
        }
    };
    let mut prev = 0u8;
    let mut i = 0;
    let byte = |i: usize| chunks.get(i).copied().ok_or(GooError::Truncated);
    while i < chunks.len() {
        let b0 = chunks[i];
        i += 1;
        let kind = b0 >> 6;
        let sub = (b0 >> 4) & 0b11;
        let low = u32::from(b0 & 0xF);
        if kind == 0b10 {
            let value = if sub & 0b10 == 0 {
                prev.wrapping_add(low as u8)
            } else {
                prev.wrapping_sub(low as u8)
            };
            let len = if sub & 0b01 == 0 {
                1
            } else {
                let n = byte(i)?;
                i += 1;
                u32::from(n)
            };
            push(&mut runs, value, len);
            prev = value;
            continue;
        }
        let value = match kind {
            0b00 => 0x00,
            0b11 => 0xFF,
            _ => {
                let v = byte(i)?;
                i += 1;
                v
            }
        };
        let mut high = 0u32;
        for _ in 0..sub {
            high = high << 8 | u32::from(byte(i)?);
            i += 1;
        }
        push(&mut runs, value, high << 4 | low);
        prev = value;
    }
    Ok(runs)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn round_trip(runs: &[Run]) -> Vec<Run> {
        let mut buf = Vec::new();
        encode(runs.iter().copied(), &mut buf);
        decode(&buf).unwrap()
    }

    #[test]
    fn all_black_and_all_white_are_one_chunk() {
        let px = 15_120 * 6_230;
        for value in [0x00, 0xFF] {
            let mut buf = Vec::new();
            assert_eq!(encode([Run::new(value, px)], &mut buf), u64::from(px));
            assert_eq!(buf.len(), 1 + 4 + 1);
            assert_eq!(decode(&buf).unwrap(), vec![Run::new(value, px)]);
        }
    }

    #[test]
    fn alternating_pixels_round_trip() {
        let runs: Vec<Run> = (0..1000)
            .map(|k| Run::new(if k % 2 == 0 { 0 } else { 0xFF }, 1))
            .collect();
        let mut buf = Vec::new();
        encode(runs.iter().copied(), &mut buf);
        assert_eq!(buf.len(), 1000 + 2);
        assert_eq!(decode(&buf).unwrap(), runs);
    }

    #[test]
    fn every_length_class_uses_the_fewest_bytes() {
        for (len, size) in [
            (1, 1),
            (15, 1),
            (16, 2),
            (0xFFF, 2),
            (0x1000, 3),
            (0xF_FFFF, 3),
            (0x10_0000, 4),
            (MAX_RUN, 4),
        ] {
            let mut buf = Vec::new();
            encode([Run::new(0xFF, len)], &mut buf);
            assert_eq!(buf.len() - 2, size, "len {len}");
            assert_eq!(decode(&buf).unwrap(), vec![Run::new(0xFF, len)]);
        }
    }

    #[test]
    fn long_runs_are_split_and_merged_back() {
        let len = MAX_RUN;
        let runs = [Run::new(0, len), Run::new(0, len), Run::new(0, 5)];
        let mut buf = Vec::new();
        encode(runs, &mut buf);
        // split as MAX_RUN, MAX_RUN, 5: two 4-byte chunks and a 1-byte one
        assert_eq!(buf.len(), 1 + 4 + 4 + 1 + 1);
        let back = decode(&buf).unwrap();
        let total: u64 = back.iter().map(|r| u64::from(r.len)).sum();
        assert_eq!(total, 2 * u64::from(len) + 5);
        assert!(back.iter().all(|r| r.value == 0));
    }

    #[test]
    fn grey_runs_and_merging() {
        let runs = [
            Run::new(0x80, 300),
            Run::new(0x80, 2),
            Run::new(0, 0),
            Run::new(0xFF, 7),
            Run::new(0x01, 70_000),
        ];
        assert_eq!(
            round_trip(&runs),
            vec![
                Run::new(0x80, 302),
                Run::new(0xFF, 7),
                Run::new(0x01, 70_000)
            ]
        );
    }

    #[test]
    fn diff_chunks_decode() {
        // 0xFF ×3, then −0x0F once, then −0x0F with run 4
        let chunks = [0xC3, 0b1010_1111, 0b1011_1111, 4];
        let mut data = vec![IMAGE_MAGIC];
        data.extend_from_slice(&chunks);
        data.push(checksum(&chunks));
        assert_eq!(
            decode(&data).unwrap(),
            vec![Run::new(0xFF, 3), Run::new(0xF0, 1), Run::new(0xE1, 4)]
        );
    }

    #[test]
    fn corruption_is_detected() {
        let mut buf = Vec::new();
        encode([Run::new(0, 40), Run::new(0xFF, 40)], &mut buf);
        let mut bad = buf.clone();
        bad[1] ^= 0x01;
        assert!(matches!(decode(&bad), Err(GooError::BadChecksum)));
        bad = buf.clone();
        bad[0] = 0x54;
        assert!(matches!(decode(&bad), Err(GooError::BadImageMagic(0x54))));
        assert!(matches!(decode(&[]), Err(GooError::Truncated)));
    }
}

#[cfg(kani)]
mod kani_proofs {
    use super::*;

    /// One run of any length and colour decodes to itself.
    #[kani::proof]
    #[kani::unwind(8)]
    fn run_length_round_trips() {
        let len: u32 = kani::any();
        kani::assume((1..=MAX_RUN).contains(&len));
        let value: u8 = kani::any();
        let mut buf = Vec::with_capacity(8);
        encode([Run::new(value, len)], &mut buf);
        let back = decode(&buf).unwrap();
        assert!(back.len() == 1 && back[0] == Run::new(value, len));
    }
}
