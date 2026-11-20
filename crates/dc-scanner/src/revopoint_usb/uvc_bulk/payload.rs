//! UVC payload headers and the reassembly of frames from bulk payloads (UVC
//! 1.5, 2.4.3.3), with the rules of the Linux driver `uvcvideo`
//! (`uvc_video_decode_bulk`), so that a frame comes out of this reader when
//! it would come out of V4L2:
//!
//! * a payload starts with a header ([`PayloadHeader`]); on a bulk endpoint
//!   it spans one or more transfers and ends with a short transfer (shorter
//!   than requested, a zero-length packet included) or once it holds
//!   `dwMaxPayloadTransferSize` bytes;
//! * a payload whose header is bad (fewer than 2 bytes, `bHeaderLength`
//!   below 2 or beyond the payload) is dropped whole;
//! * the frame identifier bit ([`FID`]) toggles at every frame: a payload
//!   with another FID than the frame in progress ends that frame; once a
//!   frame ended, payloads that keep its FID are dropped until the FID
//!   toggles (a reader that joined in the middle of a frame);
//! * the end-of-frame bit ([`EOF`]) ends the frame at the end of its
//!   payload;
//! * the error bit ([`ERR`]), more bytes than `dwMaxVideoFrameSize` or a
//!   frame of another size make the frame bad: it is dropped, as the V4L2
//!   backend drops the buffers the driver flags `V4L2_BUF_FLAG_ERROR`.
//!
//! One difference: `uvcvideo` also marks the frame that a payload with the
//! error bit ends (its FID toggled); here the error bit only spoils the
//! frame the payload belongs to.

/// `bmHeaderInfo`: frame identifier, toggles at every frame.
pub const FID: u8 = 0x01;
/// `bmHeaderInfo`: end of frame.
pub const EOF: u8 = 0x02;
/// `bmHeaderInfo`: the header has a presentation time stamp.
pub const PTS: u8 = 0x04;
/// `bmHeaderInfo`: the header has a source clock reference.
pub const SCR: u8 = 0x08;
/// `bmHeaderInfo`: error in this payload.
pub const ERR: u8 = 0x40;

/// The header of one payload.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PayloadHeader {
    /// `bHeaderLength`: bytes of the header; the frame data follows.
    pub len: usize,
    /// `bmHeaderInfo` ([`FID`], [`EOF`], [`ERR`], ...).
    pub info: u8,
    /// Presentation time stamp (device clock), when the header has one.
    pub pts: Option<u32>,
    /// Source clock reference, when the header has one: the device clock
    /// and the 11-bit USB frame counter it was sampled at.
    pub scr: Option<(u32, u16)>,
}

impl PayloadHeader {
    /// Parses the header at the start of `payload`; `None` when it is bad
    /// (the checks of `uvcvideo`). A time field announced by the header
    /// bits but missing from a short header is left out.
    pub fn parse(payload: &[u8]) -> Option<Self> {
        let [hle, info, ..] = *payload else {
            return None;
        };
        let len = usize::from(hle);
        if len < 2 || len > payload.len() {
            return None;
        }
        let mut fields = &payload[2..len];
        let mut take = |n: usize| {
            let (field, rest) = fields.split_at_checked(n)?;
            fields = rest;
            Some(field)
        };
        let pts = if info & PTS != 0 {
            take(4)
                .and_then(|b| b.try_into().ok())
                .map(u32::from_le_bytes)
        } else {
            None
        };
        let scr = if info & SCR != 0 {
            take(6).and_then(|b| {
                let stc = u32::from_le_bytes(b[..4].try_into().ok()?);
                let sof = u16::from_le_bytes(b[4..].try_into().ok()?) & 0x07ff;
                Some((stc, sof))
            })
        } else {
            None
        };
        Some(Self {
            len,
            info,
            pts,
            scr,
        })
    }

    /// The frame identifier bit.
    pub fn fid(&self) -> bool {
        self.info & FID != 0
    }
}

/// Counters of an [`Assembler`].
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Stats {
    /// Payloads with a good header.
    pub payloads: u64,
    /// Payloads dropped for a bad header.
    pub bad_headers: u64,
    /// Zero-length transfers that started no payload.
    pub empty: u64,
    /// Payloads dropped while waiting for the FID to toggle.
    pub out_of_sync: u64,
    /// Payloads with the error bit.
    pub error_bits: u64,
    /// Payloads with a presentation time stamp.
    pub with_pts: u64,
    /// Payloads with a source clock reference.
    pub with_scr: u64,
    /// Frames ended by the end-of-frame bit.
    pub ended_eof: u64,
    /// Frames ended by a FID toggle.
    pub ended_fid: u64,
    /// Frames ended because they outgrew the frame size.
    pub overflows: u64,
    /// Good frames handed over.
    pub frames: u64,
    /// Frames dropped: error bit, overflow or another size.
    pub bad_frames: u64,
}

/// A complete, good frame, handed to the sink of [`Assembler::push`].
#[derive(Debug)]
pub struct Done<'a> {
    /// The frame bytes. The sink may swap the vector for another one: the
    /// assembler clears whatever it gets back and fills it with the next
    /// frame.
    pub data: &'a mut Vec<u8>,
    /// Host time when the first payload of the frame arrived, µs.
    pub timestamp_us: u64,
    /// Presentation time stamp of the first payload of the frame.
    pub pts: Option<u32>,
}

/// Puts frames together from the transfers of a bulk video endpoint (see
/// the module doc).
#[derive(Debug)]
pub struct Assembler {
    /// `dwMaxPayloadTransferSize`.
    max_payload: usize,
    /// `dwMaxVideoFrameSize`: the size of every good frame.
    frame_bytes: usize,
    /// Bytes of the payload in progress, header included.
    payload_bytes: usize,
    /// The payload in progress has a good header: its data goes to the
    /// frame.
    payload_open: bool,
    /// The payload in progress is dropped until it ends.
    payload_skip: bool,
    /// `bmHeaderInfo` of the payload in progress.
    payload_info: u8,
    /// FID of the latest payload taken.
    last_fid: Option<bool>,
    /// A frame is in progress.
    active: bool,
    frame: Vec<u8>,
    /// The frame in progress is spoilt.
    bad: bool,
    timestamp_us: u64,
    pts: Option<u32>,
    stats: Stats,
}

impl Assembler {
    /// An assembler for payloads of at most `max_payload` bytes and frames
    /// of `frame_bytes` bytes. A `max_payload` of 0 (a device that does not
    /// say) leaves the short transfers alone to end the payloads.
    pub fn new(max_payload: usize, frame_bytes: usize) -> Self {
        Self {
            max_payload: if max_payload == 0 {
                usize::MAX
            } else {
                max_payload
            },
            frame_bytes,
            payload_bytes: 0,
            payload_open: false,
            payload_skip: false,
            payload_info: 0,
            last_fid: None,
            active: false,
            frame: Vec::with_capacity(frame_bytes),
            bad: false,
            timestamp_us: 0,
            pts: None,
            stats: Stats::default(),
        }
    }

    /// The counters so far.
    pub fn stats(&self) -> Stats {
        self.stats
    }

    /// Takes one completed transfer: `data` are its bytes, `short` says it
    /// ended before the requested length, `now_us` is the host time. Every
    /// frame it completes goes to `sink`.
    pub fn push(&mut self, data: &[u8], short: bool, now_us: u64, sink: &mut impl FnMut(Done<'_>)) {
        self.payload_bytes = self.payload_bytes.saturating_add(data.len());
        let mut body = data;
        if !self.payload_open && !self.payload_skip {
            match PayloadHeader::parse(data) {
                Some(header) => {
                    self.stats.payloads += 1;
                    self.stats.with_pts += u64::from(header.pts.is_some());
                    self.stats.with_scr += u64::from(header.scr.is_some());
                    if self.begin(&header, now_us, sink) {
                        self.payload_open = true;
                        self.payload_info = header.info;
                        body = &data[header.len..];
                    } else {
                        self.payload_skip = true;
                    }
                }
                None => {
                    if data.is_empty() {
                        self.stats.empty += 1;
                    } else {
                        self.stats.bad_headers += 1;
                    }
                    self.payload_skip = true;
                }
            }
        }
        if self.payload_open {
            self.append(body, sink);
        }
        if short || self.payload_bytes >= self.max_payload {
            if self.payload_open && self.payload_info & EOF != 0 && !self.frame.is_empty() {
                self.stats.ended_eof += 1;
                self.finish(sink);
            }
            self.payload_open = false;
            self.payload_skip = false;
            self.payload_info = 0;
            self.payload_bytes = 0;
        }
    }

    /// A payload with a good header begins; `false` when it is dropped.
    fn begin(
        &mut self,
        header: &PayloadHeader,
        now_us: u64,
        sink: &mut impl FnMut(Done<'_>),
    ) -> bool {
        let fid = header.fid();
        if self.active && self.last_fid != Some(fid) && !self.frame.is_empty() {
            self.stats.ended_fid += 1;
            self.finish(sink);
        }
        if !self.active {
            if self.last_fid == Some(fid) {
                // the rest of a frame already ended (or joined late)
                self.stats.out_of_sync += 1;
                return false;
            }
            self.active = true;
            self.bad = false;
            self.timestamp_us = now_us;
            self.pts = header.pts;
        }
        if header.info & ERR != 0 {
            self.stats.error_bits += 1;
            self.bad = true;
        }
        self.last_fid = Some(fid);
        true
    }

    /// Appends payload data to the frame in progress.
    fn append(&mut self, body: &[u8], sink: &mut impl FnMut(Done<'_>)) {
        if body.is_empty() || !self.active {
            return;
        }
        let room = self.frame_bytes.saturating_sub(self.frame.len());
        if body.len() > room {
            self.frame.extend_from_slice(&body[..room]);
            self.bad = true;
            self.stats.overflows += 1;
            self.finish(sink);
            // the rest of this payload belongs to no frame
            self.payload_open = false;
            self.payload_skip = true;
        } else {
            self.frame.extend_from_slice(body);
        }
    }

    /// Ends the frame in progress: a good one goes to `sink`.
    fn finish(&mut self, sink: &mut impl FnMut(Done<'_>)) {
        self.active = false;
        if self.bad || self.frame.len() != self.frame_bytes {
            self.stats.bad_frames += 1;
        } else {
            self.stats.frames += 1;
            sink(Done {
                data: &mut self.frame,
                timestamp_us: self.timestamp_us,
                pts: self.pts,
            });
        }
        self.frame.clear();
        self.frame.reserve(self.frame_bytes);
        self.bad = false;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A payload: header with `info` (and a PTS when `pts` is set), then
    /// `data`.
    fn payload(info: u8, pts: Option<u32>, data: &[u8]) -> Vec<u8> {
        let mut p = vec![2, info];
        if let Some(pts) = pts {
            p[0] = 6;
            p[1] |= PTS;
            p.extend_from_slice(&pts.to_le_bytes());
        }
        p.extend_from_slice(data);
        p
    }

    /// Feeds `payload` as transfers of at most `transfer` bytes (the last
    /// one short unless the payload fills it, then a zero-length packet
    /// when `zlp`).
    fn feed(
        a: &mut Assembler,
        payload: &[u8],
        transfer: usize,
        zlp: bool,
        frames: &mut Vec<Vec<u8>>,
    ) {
        let mut sink = |d: Done<'_>| frames.push(std::mem::take(d.data));
        let chunks: Vec<&[u8]> = payload.chunks(transfer).collect();
        for chunk in &chunks {
            a.push(chunk, chunk.len() < transfer, 0, &mut sink);
        }
        if zlp && payload.len().is_multiple_of(transfer) {
            a.push(&[], true, 0, &mut sink);
        }
    }

    #[test]
    fn header_fields() {
        assert_eq!(PayloadHeader::parse(&[]), None);
        assert_eq!(PayloadHeader::parse(&[2]), None);
        assert_eq!(PayloadHeader::parse(&[1, 0, 0]), None);
        // bHeaderLength beyond the payload
        assert_eq!(PayloadHeader::parse(&[12, 0x0c, 1, 2, 3]), None);
        let h = PayloadHeader::parse(&[2, EOF | FID, 0xaa]).unwrap();
        assert_eq!((h.len, h.fid(), h.pts, h.scr), (2, true, None, None));
        // PTS and SCR (SOF counter on 11 bits)
        let h = PayloadHeader::parse(&[
            12,
            PTS | SCR | EOF,
            0x78,
            0x56,
            0x34,
            0x12,
            1,
            0,
            0,
            0,
            0xff,
            0xff,
            0x99,
        ])
        .unwrap();
        assert_eq!(h.len, 12);
        assert_eq!(h.pts, Some(0x1234_5678));
        assert_eq!(h.scr, Some((1, 0x07ff)));
        assert!(!h.fid());
        // the bits announce more than the header holds
        let h = PayloadHeader::parse(&[4, PTS | SCR, 1, 2]).unwrap();
        assert_eq!((h.len, h.pts, h.scr), (4, None, None));
    }

    #[test]
    fn eof_frames_over_several_transfers() {
        // two frames of 1000 bytes, one payload each, in 512-byte transfers
        let mut a = Assembler::new(4096, 1000);
        let mut frames = Vec::new();
        let f0 = vec![1u8; 1000];
        let f1 = vec![2u8; 1000];
        feed(&mut a, &payload(EOF, Some(7), &f0), 512, true, &mut frames);
        feed(
            &mut a,
            &payload(FID | EOF, None, &f1),
            512,
            true,
            &mut frames,
        );
        assert_eq!(frames, [f0, f1]);
        let s = a.stats();
        assert_eq!(
            (s.payloads, s.frames, s.ended_eof, s.with_pts),
            (2, 2, 2, 1)
        );
        assert_eq!((s.bad_frames, s.out_of_sync, s.empty), (0, 0, 0));
    }

    #[test]
    fn fid_toggle_ends_frames_without_eof() {
        // three payloads per frame, no EOF bit: the next FID ends a frame
        let mut a = Assembler::new(4096, 300);
        let mut frames = Vec::new();
        for (fid, byte) in [(0, 1u8), (FID, 2), (0, 3)] {
            for _ in 0..3 {
                feed(
                    &mut a,
                    &payload(fid, None, &[byte; 100]),
                    512,
                    true,
                    &mut frames,
                );
            }
        }
        // the third frame waits for the next toggle
        assert_eq!(frames, [vec![1u8; 300], vec![2u8; 300]]);
        assert_eq!(a.stats().ended_fid, 2);
        feed(
            &mut a,
            &payload(FID, None, &[4; 100]),
            512,
            true,
            &mut frames,
        );
        assert_eq!(frames.len(), 3);
        assert_eq!(frames[2], vec![3u8; 300]);
    }

    #[test]
    fn payload_ends_at_the_maximum_size_or_a_short_transfer() {
        // payloads of exactly max_payload bytes (a multiple of the
        // transfer) end without a short transfer
        let mut a = Assembler::new(1024, 2 * 1022);
        let mut frames = Vec::new();
        feed(
            &mut a,
            &payload(0, None, &[5; 1022]),
            512,
            false,
            &mut frames,
        );
        feed(
            &mut a,
            &payload(EOF, None, &[6; 1022]),
            512,
            false,
            &mut frames,
        );
        assert_eq!(frames.len(), 1);
        assert_eq!(&frames[0][..1022], &[5; 1022]);
        assert_eq!(&frames[0][1022..], &[6; 1022]);
        // a zero-length packet after a full payload starts no payload
        let mut a = Assembler::new(1024, 1022);
        let mut frames = Vec::new();
        feed(
            &mut a,
            &payload(EOF, None, &[7; 1022]),
            512,
            true,
            &mut frames,
        );
        assert_eq!(frames.len(), 1);
        assert_eq!(a.stats().empty, 1);
        // a payload filling whole transfers, below the maximum, ends with
        // its zero-length packet
        let mut a = Assembler::new(1 << 20, 1022);
        let mut frames = Vec::new();
        feed(
            &mut a,
            &payload(EOF, None, &[8; 1022]),
            512,
            true,
            &mut frames,
        );
        assert_eq!(frames, [vec![8u8; 1022]]);
        assert_eq!(a.stats().empty, 0);
    }

    #[test]
    fn header_only_payload_carries_eof() {
        // the data, then an empty payload with EOF ("EOF in empty payload")
        let mut a = Assembler::new(4096, 500);
        let mut frames = Vec::new();
        feed(&mut a, &payload(0, None, &[9; 500]), 512, true, &mut frames);
        assert!(frames.is_empty());
        feed(&mut a, &payload(EOF, None, &[]), 512, true, &mut frames);
        assert_eq!(frames, [vec![9u8; 500]]);
    }

    #[test]
    fn bad_and_truncated_headers_drop_their_payload() {
        let mut a = Assembler::new(4096, 200);
        let mut frames = Vec::new();
        feed(&mut a, &payload(0, None, &[1; 100]), 512, true, &mut frames);
        // bHeaderLength 0, then a header cut by a short transfer: dropped
        feed(&mut a, &[0, 0, 7, 7, 7], 512, true, &mut frames);
        feed(&mut a, &[12, PTS | SCR, 1, 2], 512, true, &mut frames);
        feed(
            &mut a,
            &payload(EOF, None, &[1; 100]),
            512,
            true,
            &mut frames,
        );
        assert_eq!(frames, [vec![1u8; 200]]);
        assert_eq!(a.stats().bad_headers, 2);
    }

    #[test]
    fn error_bit_short_frame_and_overflow_drop_the_frame() {
        let mut a = Assembler::new(4096, 200);
        let mut frames = Vec::new();
        // error bit in the second payload of the frame
        feed(&mut a, &payload(0, None, &[1; 100]), 512, true, &mut frames);
        feed(
            &mut a,
            &payload(ERR | EOF, None, &[1; 100]),
            512,
            true,
            &mut frames,
        );
        // a frame that lost a payload: too short
        feed(
            &mut a,
            &payload(FID | EOF, None, &[2; 100]),
            512,
            true,
            &mut frames,
        );
        // a frame larger than the frame size: cut, dropped, and the rest of
        // its payload ignored
        feed(
            &mut a,
            &payload(EOF, None, &[3; 300]),
            512,
            true,
            &mut frames,
        );
        // then a good one
        feed(
            &mut a,
            &payload(FID | EOF, None, &[4; 200]),
            512,
            true,
            &mut frames,
        );
        assert_eq!(frames, [vec![4u8; 200]]);
        let s = a.stats();
        assert_eq!(
            (s.error_bits, s.overflows, s.bad_frames, s.frames),
            (1, 1, 3, 1)
        );
    }

    #[test]
    fn joining_mid_frame_waits_for_the_fid_toggle() {
        // a frame of 3 payloads; the reader sees its last two only, ending
        // in EOF: the cut frame is dropped (too short), then the FID of the
        // next frame resynchronises
        let mut a = Assembler::new(4096, 300);
        let mut frames = Vec::new();
        feed(&mut a, &payload(0, None, &[1; 100]), 512, true, &mut frames);
        feed(
            &mut a,
            &payload(EOF, None, &[1; 100]),
            512,
            true,
            &mut frames,
        );
        assert_eq!(a.stats().bad_frames, 1);
        // a stray payload of the ended frame is dropped
        feed(
            &mut a,
            &payload(EOF, None, &[1; 100]),
            512,
            true,
            &mut frames,
        );
        assert_eq!(a.stats().out_of_sync, 1);
        for i in 0..3 {
            let info = if i == 2 { FID | EOF } else { FID };
            feed(
                &mut a,
                &payload(info, None, &[2; 100]),
                512,
                true,
                &mut frames,
            );
        }
        assert_eq!(frames, [vec![2u8; 300]]);
    }

    #[test]
    fn frame_timestamp_is_its_first_payload() {
        let mut a = Assembler::new(4096, 200);
        let mut got = Vec::new();
        let mut sink = |d: Done<'_>| got.push((d.timestamp_us, d.pts, d.data.len()));
        a.push(&payload(0, Some(11), &[1; 100]), true, 1000, &mut sink);
        a.push(&payload(EOF, Some(12), &[1; 100]), true, 2000, &mut sink);
        a.push(&payload(FID | EOF, None, &[1; 200]), true, 3000, &mut sink);
        assert_eq!(got, [(1000, Some(11), 200), (3000, None, 200)]);
    }

    #[test]
    fn the_sink_may_swap_the_buffer() {
        let mut a = Assembler::new(4096, 100);
        let mut spare = vec![0u8; 7];
        let mut seen = Vec::new();
        let mut sink = |d: Done<'_>| {
            std::mem::swap(d.data, &mut spare);
            seen.push(spare.clone());
        };
        a.push(&payload(EOF, None, &[1; 100]), true, 0, &mut sink);
        a.push(&payload(FID | EOF, None, &[2; 100]), true, 0, &mut sink);
        assert_eq!(seen, [vec![1u8; 100], vec![2u8; 100]]);
    }
}
