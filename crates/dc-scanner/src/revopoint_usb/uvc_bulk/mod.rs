//! A user-space UVC video reader over a bulk endpoint, the capture of
//! systems whose own camera stack cannot deliver the MINI 2's `Y16` frames
//! (macOS, see `macos`). Its rules are those of the Linux driver `uvcvideo`,
//! so that it hands over the same frames as the V4L2 path; on Linux it is
//! only used by tests (the kernel driver must be detached from the
//! streaming interface first).
//!
//! It speaks standard UVC only (UVC 1.5, the 1.0 controls the MINI 2 has):
//!
//! 1. **Negotiation** ([`negotiate`]): `SET_CUR` of the probe control with
//!    the format, frame and interval chosen from the interface descriptors
//!    ([`probe::StreamingInterface`]), `GET_CUR` of what the device made of
//!    it, `SET_CUR` of that on the commit control: the device starts
//!    streaming.
//! 2. **Streaming** ([`BulkStream`]): a reader thread keeps transfers queued
//!    on the bulk endpoint ([`BulkPipe`], e.g. [`NusbPipe`]), puts the
//!    payloads together into frames ([`payload::Assembler`]) and keeps the
//!    newest good frames in the bounded queue of the Media Foundation
//!    capture (`ring`): draining it ends on the newest frame, as draining
//!    the V4L2 queue does.
//! 3. **Stop** ([`BulkStream::stop`]): the transfers are cancelled, then
//!    `CLEAR_FEATURE(ENDPOINT_HALT)` goes to the video endpoint, what Linux
//!    and Windows send to stop a bulk video stream (UVC does not say). The
//!    MINI 2 then leaves the bus and comes back, as after any stream.
//!
//! Errors keep the codes of the other captures: `device-lost` when the
//! device left, `device` otherwise.
//!
//! Measured on the MINI 2 (Linux, 2026-11-20, `uvcvideo` detached from
//! interface 1 only): `dwMaxPayloadTransferSize` 524288 but payloads of
//! 153600 bytes, 7 per 800x600 `Y16` frame, 2-byte headers without PTS,
//! SCR or EOF (frames end at the FID toggle, as in `uvcvideo`), two empty
//! EOF payloads at the stream start; the frames, their row-0 headers and
//! the frame timing matched V4L2 on the same scene.

pub mod payload;
pub mod probe;

use std::convert::Infallible;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Condvar, LazyLock, Mutex, MutexGuard};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use nusb::MaybeFuture;
use nusb::transfer::{
    Buffer, Bulk, ControlIn, ControlOut, ControlType, In, Recipient, TransferError,
};

use self::payload::{Assembler, Done, Stats};
use self::probe::{
    FrameDesc, GET_CUR, SET_CUR, StreamingControl, StreamingInterface, VS_COMMIT_CONTROL,
    VS_PROBE_CONTROL,
};
use super::ring::FrameRing;
use crate::{Result, ScanError};

/// Longest wait of one control request.
const CONTROL_TIMEOUT: Duration = Duration::from_secs(1);
/// Longest wait of the reader thread for one transfer, so it sees a stop
/// in time.
const READ_WAIT: Duration = Duration::from_millis(100);
/// Packets of one transfer at most: `UVC_MAX_PACKETS` of `uvcvideo`, whose
/// bulk transfers are `dwMaxPayloadTransferSize` rounded up to whole
/// packets, at most this many (a payload spans several transfers).
const MAX_PACKETS: usize = 32;
/// Transfers kept queued on the endpoint.
const IN_FLIGHT: usize = 8;
/// How long the cancelled transfers are waited for at a stop.
const CANCEL_WAIT: Duration = Duration::from_millis(500);

/// Class requests to a video-streaming interface on endpoint 0 (the probe
/// and commit controls).
pub trait StreamControl {
    /// A class `request` to the interface, host to device.
    ///
    /// # Errors
    /// `device-lost` when the device left, [`ScanError::Device`] otherwise.
    fn class_out(&self, request: u8, value: u16, index: u16, data: &[u8]) -> Result<()>;
    /// A class `request` to the interface, device to host; returns the
    /// bytes written to `reply`.
    ///
    /// # Errors
    /// `device-lost` when the device left, [`ScanError::Device`] otherwise.
    fn class_in(&self, request: u8, value: u16, index: u16, reply: &mut [u8]) -> Result<usize>;
}

/// The stream parameters the device agreed to.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Committed {
    /// The control committed.
    pub control: StreamingControl,
    /// Bytes of one frame (`dwMaxVideoFrameSize`).
    pub frame_bytes: usize,
    /// `dwMaxPayloadTransferSize` (0 when the device does not say).
    pub max_payload: usize,
}

/// Negotiates `format` (`bFormatIndex`), `frame` at `interval` (100 ns
/// units) on the streaming interface `vs` and commits it: the device starts
/// streaming (see the module doc).
///
/// # Errors
/// A failed request, or a device that answers another format or frame.
pub fn negotiate(
    ctl: &impl StreamControl,
    vs: &StreamingInterface,
    format: u8,
    frame: &FrameDesc,
    interval: u32,
) -> Result<Committed> {
    let probe = StreamingControl::new(vs.uvc_version, format, frame.index, interval);
    let len = probe.as_bytes().len();
    let index = u16::from(vs.number);
    let selector = |s: u8| u16::from(s) << 8;
    ctl.class_out(SET_CUR, selector(VS_PROBE_CONTROL), index, probe.as_bytes())?;
    let mut reply = [0; 48];
    let got = ctl.class_in(
        GET_CUR,
        selector(VS_PROBE_CONTROL),
        index,
        &mut reply[..len],
    )?;
    let current = StreamingControl::from_reply(&reply[..got], len).ok_or_else(|| {
        ScanError::Device(format!(
            "GET_CUR(VS_PROBE_CONTROL) of interface {}: {got} bytes",
            vs.number
        ))
    })?;
    if (current.format_index(), current.frame_index()) != (format, frame.index) {
        return Err(ScanError::Device(format!(
            "interface {} answered format {} frame {} to the probe of format {format} frame {}",
            vs.number,
            current.format_index(),
            current.frame_index(),
            frame.index
        )));
    }
    ctl.class_out(
        SET_CUR,
        selector(VS_COMMIT_CONTROL),
        index,
        current.as_bytes(),
    )?;
    let frame_bytes = match current.max_video_frame_size() {
        0 if frame.max_bytes > 0 => frame.max_bytes as usize,
        0 => usize::from(frame.width) * usize::from(frame.height) * 2,
        n => n as usize,
    };
    Ok(Committed {
        control: current,
        frame_bytes,
        max_payload: current.max_payload_transfer_size() as usize,
    })
}

/// One completed transfer of a [`BulkPipe`].
#[derive(Debug)]
pub struct Read<'a> {
    /// The bytes received.
    pub data: &'a [u8],
    /// The transfer ended before the requested length (a short packet).
    pub short: bool,
}

/// The bulk IN endpoint of a video-streaming interface, owned by the reader
/// thread.
pub trait BulkPipe: Send {
    /// Sizes the transfers for payloads of at most `max_payload` bytes (0
    /// when the device does not say), as [`transfer_len`] does; called
    /// before the first read.
    fn set_payload_size(&mut self, max_payload: usize);
    /// Waits at most `timeout` for the next completed transfer (`Ok(None)`
    /// when none completed), keeping the next ones queued.
    ///
    /// # Errors
    /// `device-lost` when the device left, [`ScanError::Device`] when the
    /// transfer failed.
    fn read(&mut self, timeout: Duration) -> Result<Option<Read<'_>>>;
    /// Cancels the queued transfers and sends `CLEAR_FEATURE(ENDPOINT_HALT)`
    /// to the endpoint: the end of the stream.
    ///
    /// # Errors
    /// The failure of the request (the device may be gone).
    fn halt(&mut self) -> Result<()>;
}

/// Bytes of one transfer for payloads of at most `max_payload` bytes on an
/// endpoint of `packet`-byte packets, as `uvcvideo` sizes them: whole
/// packets, at most [`MAX_PACKETS`].
pub fn transfer_len(max_payload: usize, packet: usize) -> usize {
    let packet = packet.max(1);
    let packets = if max_payload == 0 {
        MAX_PACKETS
    } else {
        max_payload.div_ceil(packet).clamp(1, MAX_PACKETS)
    };
    packets * packet
}

/// A failed transfer, as a [`ScanError`].
fn transfer_error(what: &str, e: TransferError) -> ScanError {
    match e {
        TransferError::Disconnected => ScanError::DeviceLost(format!("{what}: {e}")),
        TransferError::Stall => ScanError::Device(format!("{what}: refused by the device (stall)")),
        e => ScanError::Device(format!("{what}: {e}")),
    }
}

/// A failure of a `nusb` call other than a transfer.
pub(crate) fn usb_error(what: &str, e: &nusb::Error) -> ScanError {
    if e.kind() == nusb::ErrorKind::Disconnected {
        ScanError::DeviceLost(format!("{what}: {e}"))
    } else {
        ScanError::Device(format!("{what}: {e}"))
    }
}

/// The class request of [`StreamControl::class_out`] sent by `send`.
fn class_out_with(
    send: impl FnOnce(ControlOut<'_>) -> std::result::Result<(), TransferError>,
    request: u8,
    value: u16,
    index: u16,
    data: &[u8],
) -> Result<()> {
    send(ControlOut {
        control_type: ControlType::Class,
        recipient: Recipient::Interface,
        request,
        value,
        index,
        data,
    })
    .map_err(|e| transfer_error(&format!("class request {request:#04x} {value:#06x}"), e))
}

/// The class request of [`StreamControl::class_in`] sent by `receive`.
fn class_in_with(
    receive: impl FnOnce(ControlIn) -> std::result::Result<Vec<u8>, TransferError>,
    request: u8,
    value: u16,
    index: u16,
    reply: &mut [u8],
) -> Result<usize> {
    let data = receive(ControlIn {
        control_type: ControlType::Class,
        recipient: Recipient::Interface,
        request,
        value,
        index,
        length: u16::try_from(reply.len()).unwrap_or(u16::MAX),
    })
    .map_err(|e| transfer_error(&format!("class request {request:#04x} {value:#06x}"), e))?;
    let n = data.len().min(reply.len());
    reply[..n].copy_from_slice(&data[..n]);
    Ok(n)
}

/// Requests through a claimed interface.
impl StreamControl for nusb::Interface {
    fn class_out(&self, request: u8, value: u16, index: u16, data: &[u8]) -> Result<()> {
        class_out_with(
            |c| self.control_out(c, CONTROL_TIMEOUT).wait(),
            request,
            value,
            index,
            data,
        )
    }

    fn class_in(&self, request: u8, value: u16, index: u16, reply: &mut [u8]) -> Result<usize> {
        class_in_with(
            |c| self.control_in(c, CONTROL_TIMEOUT).wait(),
            request,
            value,
            index,
            reply,
        )
    }
}

/// Requests through the device (macOS takes class requests to an interface
/// it does not hold; Linux needs it claimed).
impl StreamControl for nusb::Device {
    fn class_out(&self, request: u8, value: u16, index: u16, data: &[u8]) -> Result<()> {
        class_out_with(
            |c| self.control_out(c, CONTROL_TIMEOUT).wait(),
            request,
            value,
            index,
            data,
        )
    }

    fn class_in(&self, request: u8, value: u16, index: u16, reply: &mut [u8]) -> Result<usize> {
        class_in_with(
            |c| self.control_in(c, CONTROL_TIMEOUT).wait(),
            request,
            value,
            index,
            reply,
        )
    }
}

/// The bulk endpoint of a claimed interface, read with `nusb`:
/// [`IN_FLIGHT`] transfers of [`transfer_len`] bytes stay queued, so the
/// host controller reads at every chance.
pub struct NusbPipe {
    endpoint: nusb::Endpoint<Bulk, In>,
    transfer_len: usize,
    /// The buffer of the latest [`BulkPipe::read`], queued again at the
    /// next one.
    current: Option<Buffer>,
}

impl NusbPipe {
    /// The bulk IN endpoint `address` of `interface`.
    ///
    /// # Errors
    /// [`ScanError::Device`] when the interface has no such endpoint or it
    /// is in use.
    pub fn new(interface: &nusb::Interface, address: u8) -> Result<Self> {
        let endpoint = interface
            .endpoint::<Bulk, In>(address)
            .map_err(|e| usb_error(&format!("video endpoint {address:#04x}"), &e))?;
        let transfer_len = transfer_len(0, endpoint.max_packet_size());
        Ok(Self {
            endpoint,
            transfer_len,
            current: None,
        })
    }

    /// Bytes requested by each transfer.
    pub fn transfer_len(&self) -> usize {
        self.transfer_len
    }
}

impl std::fmt::Debug for NusbPipe {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("NusbPipe")
            .field("endpoint", &self.endpoint)
            .field("transfer_len", &self.transfer_len)
            .finish_non_exhaustive()
    }
}

impl BulkPipe for NusbPipe {
    fn set_payload_size(&mut self, max_payload: usize) {
        self.transfer_len = transfer_len(max_payload, self.endpoint.max_packet_size());
    }

    fn read(&mut self, timeout: Duration) -> Result<Option<Read<'_>>> {
        if let Some(buffer) = self.current.take() {
            self.endpoint.submit(buffer);
        }
        while self.endpoint.pending() < IN_FLIGHT {
            let buffer = self.endpoint.allocate(self.transfer_len);
            self.endpoint.submit(buffer);
        }
        let Some(done) = self.endpoint.wait_next_complete(timeout) else {
            return Ok(None);
        };
        done.status
            .map_err(|e| transfer_error("video stream transfer", e))?;
        let short = done.actual_len < done.buffer.requested_len();
        let data: &[u8] = self.current.insert(done.buffer);
        Ok(Some(Read { data, short }))
    }

    fn halt(&mut self) -> Result<()> {
        self.current = None;
        self.endpoint.cancel_all();
        while self.endpoint.pending() > 0 {
            if self.endpoint.wait_next_complete(CANCEL_WAIT).is_none() {
                break;
            }
        }
        self.endpoint
            .clear_halt()
            .wait()
            .map_err(|e| usb_error("CLEAR_FEATURE(ENDPOINT_HALT) of the video endpoint", &e))
    }
}

/// Host time in microseconds since the first call (monotonic).
fn now_us() -> u64 {
    static EPOCH: LazyLock<Instant> = LazyLock::new(Instant::now);
    u64::try_from(EPOCH.elapsed().as_micros()).unwrap_or(u64::MAX)
}

/// A frame handed over by [`BulkStream::next`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Frame {
    /// Bytes of the frame written into the output vector.
    pub bytes: usize,
    /// Host time when its first payload arrived, µs (monotonic).
    pub timestamp_us: u64,
}

/// Why the stream ended, kept for every later call.
#[derive(Clone, Debug)]
enum Failure {
    Lost(String),
    Device(String),
}

impl Failure {
    fn of(e: &ScanError) -> Self {
        match e {
            ScanError::DeviceLost(m) => Self::Lost(m.clone()),
            e => Self::Device(e.to_string()),
        }
    }

    fn error(&self) -> ScanError {
        match self {
            Self::Lost(m) => ScanError::DeviceLost(m.clone()),
            Self::Device(m) => ScanError::Device(m.clone()),
        }
    }
}

#[derive(Debug, Default)]
struct Inbox {
    frames: FrameRing,
    failure: Option<Failure>,
    stats: Stats,
}

#[derive(Debug, Default)]
struct Shared {
    inbox: Mutex<Inbox>,
    ready: Condvar,
    stop: AtomicBool,
}

impl Shared {
    fn lock(&self) -> MutexGuard<'_, Inbox> {
        self.inbox.lock().unwrap_or_else(|e| e.into_inner())
    }
}

/// The streaming part of the reader (see the module doc): a reader thread
/// on a [`BulkPipe`], frames waiting in a short queue.
pub struct BulkStream {
    shared: Arc<Shared>,
    thread: Option<JoinHandle<Box<dyn BulkPipe>>>,
}

impl std::fmt::Debug for BulkStream {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("BulkStream")
            .field("running", &self.thread.is_some())
            .finish_non_exhaustive()
    }
}

impl BulkStream {
    /// Starts reading `pipe` (the device must stream: see [`negotiate`]):
    /// frames of `committed.frame_bytes` bytes, at most `queue` of them
    /// waiting.
    ///
    /// # Errors
    /// [`ScanError::Device`] when the thread cannot start.
    pub fn start(mut pipe: Box<dyn BulkPipe>, committed: &Committed, queue: usize) -> Result<Self> {
        pipe.set_payload_size(committed.max_payload);
        let shared = Arc::new(Shared::default());
        shared.lock().frames.set_capacity(queue);
        let reader = shared.clone();
        let (max_payload, frame_bytes) = (committed.max_payload, committed.frame_bytes);
        let thread = std::thread::Builder::new()
            .name("uvc-bulk-reader".into())
            .spawn(move || read_loop(pipe, &reader, max_payload, frame_bytes))
            .map_err(|e| ScanError::Device(format!("video reader thread: {e}")))?;
        Ok(Self {
            shared,
            thread: Some(thread),
        })
    }

    /// Waits at most `timeout` for a frame and moves it into `out` (the
    /// previous content of `out` is reused). Frames come oldest first;
    /// `Ok(None)` when none came in time.
    ///
    /// # Errors
    /// `device-lost` when the device left, [`ScanError::Device`] when the
    /// stream failed or was stopped.
    pub fn next(&mut self, timeout: Duration, out: &mut Vec<u8>) -> Result<Option<Frame>> {
        let inbox = self.shared.lock();
        let (mut inbox, _) = self
            .shared
            .ready
            .wait_timeout_while(inbox, timeout, |i| {
                i.frames.is_empty() && i.failure.is_none()
            })
            .unwrap_or_else(|e| e.into_inner());
        if let Some(timestamp_us) = inbox.frames.pop_into(out) {
            return Ok(Some(Frame {
                bytes: out.len(),
                timestamp_us,
            }));
        }
        if let Some(failure) = &inbox.failure {
            return Err(failure.error());
        }
        if self.thread.is_none() {
            return Err(ScanError::Device("the video stream is stopped".into()));
        }
        Ok(None)
    }

    /// The counters of the frame reassembly so far (frames handed over by
    /// the reader, whether taken or dropped from the full queue).
    pub fn stats(&self) -> Stats {
        self.shared.lock().stats
    }

    /// Stops the stream (see the module doc) and drops the frames still
    /// waiting, as `VIDIOC_STREAMOFF` does; errors are ignored, the device
    /// may be gone. Idempotent.
    pub fn stop(&mut self) {
        self.shared.stop.store(true, Ordering::Release);
        if let Some(thread) = self.thread.take()
            && let Ok(mut pipe) = thread.join()
        {
            let _ = pipe.halt();
        }
        self.shared.lock().frames.clear();
    }
}

impl Drop for BulkStream {
    fn drop(&mut self) {
        self.stop();
    }
}

/// The reader thread: transfers to frames until a stop or a failure; gives
/// the pipe back for the halt.
fn read_loop(
    mut pipe: Box<dyn BulkPipe>,
    shared: &Shared,
    max_payload: usize,
    frame_bytes: usize,
) -> Box<dyn BulkPipe> {
    let mut assembler = Assembler::new(max_payload, frame_bytes);
    let mut ended = 0;
    let mut deliver = |done: Done<'_>| {
        let mut inbox = shared.lock();
        let _ = inbox.frames.push_with(done.timestamp_us, |data| {
            std::mem::swap(done.data, data);
            Ok::<(), Infallible>(())
        });
        drop(inbox);
        shared.ready.notify_all();
    };
    while !shared.stop.load(Ordering::Acquire) {
        let failure = match pipe.read(READ_WAIT) {
            Ok(None) => continue,
            Ok(Some(read)) => {
                assembler.push(read.data, read.short, now_us(), &mut deliver);
                let stats = assembler.stats();
                if stats.frames + stats.bad_frames != ended {
                    ended = stats.frames + stats.bad_frames;
                    shared.lock().stats = stats;
                }
                continue;
            }
            Err(e) => e,
        };
        let mut inbox = shared.lock();
        inbox.failure = Some(Failure::of(&failure));
        inbox.stats = assembler.stats();
        drop(inbox);
        shared.ready.notify_all();
        return pipe;
    }
    shared.lock().stats = assembler.stats();
    pipe
}

#[cfg(test)]
mod tests {
    use std::cell::RefCell;
    use std::sync::mpsc;

    use super::payload::{EOF, FID};
    use super::*;

    /// What the scripted pipe returns: a transfer (bytes, short) or a
    /// failure (`true`: the device left).
    type Step = std::result::Result<(Vec<u8>, bool), bool>;

    /// A pipe that plays scripted transfers, then waits.
    struct Script {
        reads: mpsc::Receiver<Step>,
        current: Vec<u8>,
        halted: Arc<AtomicBool>,
        payload_size: Arc<Mutex<usize>>,
    }

    impl BulkPipe for Script {
        fn set_payload_size(&mut self, max_payload: usize) {
            *self.payload_size.lock().unwrap() = max_payload;
        }

        fn read(&mut self, timeout: Duration) -> Result<Option<Read<'_>>> {
            match self.reads.recv_timeout(timeout) {
                Ok(Ok((data, short))) => {
                    self.current = data;
                    Ok(Some(Read {
                        data: &self.current,
                        short,
                    }))
                }
                Ok(Err(true)) => Err(ScanError::DeviceLost("gone".into())),
                Ok(Err(false)) => Err(ScanError::Device("fault".into())),
                Err(_) => Ok(None),
            }
        }

        fn halt(&mut self) -> Result<()> {
            self.halted.store(true, Ordering::Release);
            Ok(())
        }
    }

    struct Rig {
        stream: BulkStream,
        feed: mpsc::Sender<Step>,
        halted: Arc<AtomicBool>,
        payload_size: Arc<Mutex<usize>>,
    }

    fn stream(queue: usize, frame_bytes: usize) -> Rig {
        let (feed, reads) = mpsc::channel();
        let halted = Arc::new(AtomicBool::new(false));
        let payload_size = Arc::new(Mutex::new(0));
        let pipe = Script {
            reads,
            current: Vec::new(),
            halted: halted.clone(),
            payload_size: payload_size.clone(),
        };
        let committed = Committed {
            control: StreamingControl::new(0x0100, 2, 1, 666_666),
            frame_bytes,
            max_payload: 4096,
        };
        let stream = BulkStream::start(Box::new(pipe), &committed, queue).unwrap();
        Rig {
            stream,
            feed,
            halted,
            payload_size,
        }
    }

    /// One frame of `fill` bytes as a single payload with EOF.
    fn frame(fid: bool, fill: u8, bytes: usize) -> Step {
        let mut p = vec![2, EOF | if fid { FID } else { 0 }];
        p.resize(2 + bytes, fill);
        Ok((p, true))
    }

    #[test]
    fn frames_in_order_then_newest_kept() {
        let Rig {
            stream: mut s,
            feed,
            halted,
            payload_size,
        } = stream(2, 100);
        assert_eq!(*payload_size.lock().unwrap(), 4096);
        let mut out = Vec::new();
        assert_eq!(s.next(Duration::ZERO, &mut out).unwrap(), None);
        feed.send(frame(false, 1, 100)).unwrap();
        let f = s.next(Duration::from_secs(5), &mut out).unwrap().unwrap();
        assert_eq!((f.bytes, out.as_slice()), (100, &[1u8; 100][..]));
        // four frames while the consumer sleeps: the two newest wait
        for (i, fill) in [2u8, 3, 4, 5].into_iter().enumerate() {
            feed.send(frame(i % 2 == 0, fill, 100)).unwrap();
        }
        let deadline = Instant::now() + Duration::from_secs(5);
        while s.stats().frames < 5 && Instant::now() < deadline {
            std::thread::sleep(Duration::from_millis(5));
        }
        let mut got = Vec::new();
        while let Some(f) = s.next(Duration::ZERO, &mut out).unwrap() {
            assert_eq!(f.bytes, 100);
            got.push(out[0]);
        }
        assert_eq!(got, [4, 5]);
        s.stop();
        assert!(halted.load(Ordering::Acquire));
        assert_eq!(
            s.next(Duration::ZERO, &mut out).unwrap_err().code(),
            "device"
        );
    }

    #[test]
    fn a_lost_device_ends_the_stream_for_good() {
        let Rig {
            stream: mut s,
            feed,
            halted,
            ..
        } = stream(4, 10);
        feed.send(frame(false, 7, 10)).unwrap();
        feed.send(Err(true)).unwrap();
        let mut out = Vec::new();
        // the frame before the loss still comes
        let f = s.next(Duration::from_secs(5), &mut out).unwrap().unwrap();
        assert_eq!(f.bytes, 10);
        for _ in 0..2 {
            let e = s.next(Duration::from_secs(5), &mut out).unwrap_err();
            assert_eq!(e.code(), "device-lost");
        }
        drop(s);
        // the halt is still sent (and ignored when the device is gone)
        assert!(halted.load(Ordering::Acquire));
    }

    #[test]
    fn a_transfer_failure_is_a_device_error() {
        let Rig {
            stream: mut s,
            feed,
            ..
        } = stream(4, 10);
        feed.send(Err(false)).unwrap();
        let mut out = Vec::new();
        let e = s.next(Duration::from_secs(5), &mut out).unwrap_err();
        assert_eq!(e.code(), "device");
    }

    #[test]
    fn stop_drops_the_waiting_frames() {
        let Rig {
            stream: mut s,
            feed,
            halted,
            ..
        } = stream(4, 10);
        feed.send(frame(false, 1, 10)).unwrap();
        let deadline = Instant::now() + Duration::from_secs(5);
        while s.stats().frames < 1 && Instant::now() < deadline {
            std::thread::sleep(Duration::from_millis(5));
        }
        s.stop();
        assert!(halted.load(Ordering::Acquire));
        let mut out = Vec::new();
        let e = s.next(Duration::ZERO, &mut out).unwrap_err();
        assert_eq!(e.code(), "device");
        assert!(out.is_empty());
    }

    #[test]
    fn transfer_sizes_follow_uvcvideo() {
        assert_eq!(transfer_len(524_288, 512), 32 * 512);
        assert_eq!(transfer_len(3000, 512), 6 * 512);
        assert_eq!(transfer_len(512, 512), 512);
        assert_eq!(transfer_len(1, 512), 512);
        assert_eq!(transfer_len(0, 512), 32 * 512);
        assert_eq!(transfer_len(100, 64), 2 * 64);
    }

    /// One recorded request: `bRequest`, `wValue`, `wIndex`, data sent.
    type Request = (u8, u16, u16, Vec<u8>);

    /// The requests of a negotiation, recorded.
    #[derive(Default)]
    struct Recorder {
        sent: RefCell<Vec<Request>>,
        /// What `GET_CUR` answers.
        answer: Vec<u8>,
    }

    impl StreamControl for Recorder {
        fn class_out(&self, request: u8, value: u16, index: u16, data: &[u8]) -> Result<()> {
            self.sent
                .borrow_mut()
                .push((request, value, index, data.to_vec()));
            Ok(())
        }

        fn class_in(&self, request: u8, value: u16, index: u16, reply: &mut [u8]) -> Result<usize> {
            self.sent
                .borrow_mut()
                .push((request, value, index, Vec::new()));
            let n = self.answer.len().min(reply.len());
            reply[..n].copy_from_slice(&self.answer[..n]);
            Ok(n)
        }
    }

    #[test]
    fn negotiation_probes_reads_back_and_commits() {
        let config = probe::tests::mini2_config();
        let vs = StreamingInterface::parse(&config, 1).unwrap();
        let (format, frame) = vs.find(u32::from_le_bytes(*b"Y16 "), 800, 600).unwrap();
        // what the MINI 2 answered (2026-11-20)
        let answer = vec![
            0x01, 0x00, 0x02, 0x01, 0x2a, 0x2c, 0x0a, 0x00, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x00,
            0xa6, 0x0e, 0x00, 0x00, 0x00, 0x08, 0x00,
        ];
        let rec = Recorder {
            answer: answer.clone(),
            ..Recorder::default()
        };
        let c = negotiate(&rec, &vs, format.index, frame, frame.nearest_interval(15)).unwrap();
        assert_eq!((c.frame_bytes, c.max_payload), (960_000, 524_288));
        let sent = rec.sent.borrow();
        let calls: Vec<(u8, u16, u16, usize)> = sent
            .iter()
            .map(|(r, v, i, d)| (*r, *v, *i, d.len()))
            .collect();
        assert_eq!(
            calls,
            [
                (0x01, 0x0100, 1, 26),
                (0x81, 0x0100, 1, 0),
                (0x01, 0x0200, 1, 26)
            ]
        );
        // the probe: bmHint 1, format 2, frame 1, 666666 x 100 ns
        assert_eq!(&sent[0].3[..8], &[1, 0, 2, 1, 0x2a, 0x2c, 0x0a, 0]);
        // the commit is what the device answered
        assert_eq!(sent[2].3, answer);
        drop(sent);
        // a device that picks another frame is refused, nothing committed
        let mut other = answer;
        other[3] = 2;
        let rec = Recorder {
            answer: other,
            ..Recorder::default()
        };
        let e = negotiate(&rec, &vs, format.index, frame, 666_666).unwrap_err();
        assert!(e.to_string().contains("frame 2"), "{e}");
        assert_eq!(rec.sent.borrow().len(), 2);
        // a short answer too
        let rec = Recorder {
            answer: vec![0; 20],
            ..Recorder::default()
        };
        assert!(negotiate(&rec, &vs, format.index, frame, 666_666).is_err());
    }
}
