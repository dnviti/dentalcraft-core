//! Minimal V4L2 memory-mapped capture (Linux), hand-written over
//! `rustix::ioctl` from `linux/videodev2.h`. Only standard V4L2 calls:
//! `QUERYCAP`, `S_FMT`, `S_PARM`, `REQBUFS`, `QUERYBUF`, `QBUF`, `DQBUF`,
//! `STREAMON`, `STREAMOFF`; plus, through `UVCIOC_CTRL_QUERY` of
//! `uvcvideo`, the user-authorised fill light lines of [`FillLight`] and
//! nothing else (see [`super::light`]).

#![allow(unsafe_code)]
#![warn(clippy::undocumented_unsafe_blocks)]

use std::ffi::{c_int, c_long, c_void};
use std::os::fd::OwnedFd;
use std::path::Path;
use std::ptr::NonNull;
use std::time::Duration;

use rustix::event::{PollFd, PollFlags, Timespec, poll};
use rustix::fs::{Mode, OFlags, open};
use rustix::io::Errno;
use rustix::ioctl::{Getter, Opcode, Setter, Updater, ioctl, opcode};
use rustix::mm::{MapFlags, ProtFlags, mmap, munmap};

use super::exposure::ExposureProfile;
use super::factory::{
    CLOSE_PAYLOAD, FILE_PAYLOAD_LEN, FactoryFile, OPEN_SELECTOR, READ_SELECTOR, append_chunk,
};
use super::light::{FillLight, SHELL_PAYLOAD_LEN, SHELL_SELECTOR, SHELL_UNIT};
use crate::{Result, ScanError};

// ----- videodev2.h ---------------------------------------------------------

const BUF_TYPE_VIDEO_CAPTURE: u32 = 1;
const MEMORY_MMAP: u32 = 1;
const FIELD_NONE: u32 = 1;
const CAP_VIDEO_CAPTURE: u32 = 0x0000_0001;
const CAP_STREAMING: u32 = 0x0400_0000;
const CAP_TIMEPERFRAME: u32 = 0x1000;
const BUF_FLAG_ERROR: u32 = 0x40;

/// `v4l2_fourcc('Y', '1', '6', ' ')`: 16-bit greyscale.
pub(super) const PIX_FMT_Y16: u32 = u32::from_le_bytes(*b"Y16 ");

/// Types whose all-zero bit pattern is a valid value.
///
/// # Safety
/// Implementors are `repr(C)` aggregates of integers (and unions of them).
unsafe trait Zeroable: Sized {
    fn zeroed() -> Self {
        // SAFETY: the trait contract: all-zero bits are a valid value.
        unsafe { std::mem::zeroed() }
    }
}

/// `struct v4l2_capability`.
#[repr(C)]
struct Capability {
    driver: [u8; 16],
    card: [u8; 32],
    bus_info: [u8; 32],
    version: u32,
    capabilities: u32,
    device_caps: u32,
    reserved: [u32; 3],
}

/// `struct v4l2_pix_format`.
#[repr(C)]
#[derive(Clone, Copy)]
struct PixFormat {
    width: u32,
    height: u32,
    pixelformat: u32,
    field: u32,
    bytesperline: u32,
    sizeimage: u32,
    colorspace: u32,
    priv_: u32,
    flags: u32,
    enc: u32,
    quantization: u32,
    xfer_func: u32,
}

/// The `fmt` union of `struct v4l2_format` (200 bytes, pointer-aligned
/// because of `struct v4l2_window`).
#[repr(C)]
union FormatUnion {
    pix: PixFormat,
    raw: [u8; 200],
    align: [usize; 200 / std::mem::size_of::<usize>()],
}

/// `struct v4l2_format`.
#[repr(C)]
struct Format {
    type_: u32,
    fmt: FormatUnion,
}

/// `struct v4l2_fract`.
#[repr(C)]
#[derive(Clone, Copy)]
struct Fract {
    numerator: u32,
    denominator: u32,
}

/// `struct v4l2_captureparm`.
#[repr(C)]
#[derive(Clone, Copy)]
struct CaptureParm {
    capability: u32,
    capturemode: u32,
    timeperframe: Fract,
    extendedmode: u32,
    readbuffers: u32,
    reserved: [u32; 4],
}

/// The `parm` union of `struct v4l2_streamparm`.
#[repr(C)]
union ParmUnion {
    capture: CaptureParm,
    raw: [u8; 200],
}

/// `struct v4l2_streamparm`.
#[repr(C)]
struct StreamParm {
    type_: u32,
    parm: ParmUnion,
}

/// `struct v4l2_requestbuffers`.
#[repr(C)]
struct RequestBuffers {
    count: u32,
    type_: u32,
    memory: u32,
    capabilities: u32,
    flags: u8,
    reserved: [u8; 3],
}

/// `struct timeval` (the classic `long` layout of the V4L2 ABI).
#[repr(C)]
#[derive(Clone, Copy)]
struct Timeval {
    sec: c_long,
    usec: c_long,
}

/// `struct v4l2_timecode`.
#[repr(C)]
#[derive(Clone, Copy)]
struct Timecode {
    type_: u32,
    flags: u32,
    frames: u8,
    seconds: u8,
    minutes: u8,
    hours: u8,
    userbits: [u8; 4],
}

/// The `m` union of `struct v4l2_buffer`.
#[repr(C)]
#[derive(Clone, Copy)]
union BufferM {
    offset: u32,
    userptr: usize,
    planes: *mut c_void,
    fd: i32,
}

/// `struct v4l2_buffer`.
#[repr(C)]
struct Buffer {
    index: u32,
    type_: u32,
    bytesused: u32,
    flags: u32,
    field: u32,
    timestamp: Timeval,
    timecode: Timecode,
    sequence: u32,
    memory: u32,
    m: BufferM,
    length: u32,
    reserved2: u32,
    request_fd: i32,
}

// SAFETY: integer aggregates and unions of integers / a raw pointer; null
// and zero are valid values of each.
unsafe impl Zeroable for Capability {}
// SAFETY: as above.
unsafe impl Zeroable for Format {}
// SAFETY: as above.
unsafe impl Zeroable for PixFormat {}
// SAFETY: as above.
unsafe impl Zeroable for StreamParm {}
// SAFETY: as above.
unsafe impl Zeroable for RequestBuffers {}
// SAFETY: as above.
unsafe impl Zeroable for Buffer {}

const VIDIOC_QUERYCAP: Opcode = opcode::read::<Capability>(b'V', 0);
const VIDIOC_S_FMT: Opcode = opcode::read_write::<Format>(b'V', 5);
const VIDIOC_REQBUFS: Opcode = opcode::read_write::<RequestBuffers>(b'V', 8);
const VIDIOC_QUERYBUF: Opcode = opcode::read_write::<Buffer>(b'V', 9);
const VIDIOC_QBUF: Opcode = opcode::read_write::<Buffer>(b'V', 15);
const VIDIOC_DQBUF: Opcode = opcode::read_write::<Buffer>(b'V', 17);
const VIDIOC_STREAMON: Opcode = opcode::write::<c_int>(b'V', 18);
const VIDIOC_STREAMOFF: Opcode = opcode::write::<c_int>(b'V', 19);
const VIDIOC_G_PARM: Opcode = opcode::read_write::<StreamParm>(b'V', 21);
const VIDIOC_S_PARM: Opcode = opcode::read_write::<StreamParm>(b'V', 22);

// ----- errors --------------------------------------------------------------

/// A failed call as a scanner error: a vanished device is `device-lost`.
fn errno_error(call: &str, e: Errno) -> ScanError {
    match e {
        Errno::NODEV | Errno::NXIO => ScanError::DeviceLost(format!("{call}: {e}")),
        Errno::BUSY => ScanError::Device(format!(
            "{call}: {e} (another program is using the scanner)"
        )),
        e => ScanError::Device(format!("{call}: {e}")),
    }
}

/// Text of a NUL-padded byte array.
fn text(bytes: &[u8]) -> String {
    let end = bytes.iter().position(|&b| b == 0).unwrap_or(bytes.len());
    String::from_utf8_lossy(&bytes[..end]).into_owned()
}

// ----- capture -------------------------------------------------------------

/// Identity of a V4L2 node (`VIDIOC_QUERYCAP`).
#[derive(Clone, Debug, PartialEq, Eq)]
pub(super) struct Caps {
    /// Driver name (`uvcvideo`).
    pub driver: String,
    /// Device name.
    pub card: String,
    /// Bus location (`usb-0000:00:14.0-1`).
    pub bus_info: String,
}

/// One mmap'ed driver buffer.
struct Mapping {
    ptr: NonNull<c_void>,
    len: usize,
}

/// A dequeued frame, copied out of the driver buffer.
#[derive(Clone, Copy, Debug)]
pub(super) struct Dequeued {
    /// Bytes of the frame written into the output vector.
    pub bytes: usize,
    /// Driver timestamp (monotonic clock), microseconds.
    pub timestamp_us: u64,
}

/// An open V4L2 capture node, streaming once [`Capture::start`] succeeded.
pub(super) struct Capture {
    fd: OwnedFd,
    buffers: Vec<Mapping>,
    streaming: bool,
}

// SAFETY: the mappings are owned by this value (unmapped on drop) and only
// touched through `&mut self`; the file descriptor is thread-agnostic.
unsafe impl Send for Capture {}

impl Capture {
    /// Opens the node (non-blocking) and checks it is a streaming capture
    /// node.
    ///
    /// # Errors
    /// The `open` / `QUERYCAP` failure (`device-lost` when the device is
    /// gone), [`ScanError::Device`] for a node that cannot stream.
    pub fn open(path: &Path) -> std::result::Result<(Self, Caps), (Errno, ScanError)> {
        let fd = open(
            path,
            OFlags::RDWR | OFlags::NONBLOCK | OFlags::CLOEXEC,
            Mode::empty(),
        )
        .map_err(|e| (e, errno_error(&format!("open {}", path.display()), e)))?;
        // SAFETY: `VIDIOC_QUERYCAP` writes one `v4l2_capability`.
        let cap: Capability = unsafe { ioctl(&fd, Getter::<VIDIOC_QUERYCAP, Capability>::new()) }
            .map_err(|e| (e, errno_error("VIDIOC_QUERYCAP", e)))?;
        let caps = Caps {
            driver: text(&cap.driver),
            card: text(&cap.card),
            bus_info: text(&cap.bus_info),
        };
        let wanted = CAP_VIDEO_CAPTURE | CAP_STREAMING;
        if cap.device_caps & wanted != wanted {
            return Err((
                Errno::NOTTY,
                ScanError::Device(format!(
                    "{} ({}) is not a streaming capture node",
                    path.display(),
                    caps.card
                )),
            ));
        }
        Ok((
            Self {
                fd,
                buffers: Vec::new(),
                streaming: false,
            },
            caps,
        ))
    }

    /// Sets a `width x height` format of `fourcc`; returns the image size in
    /// bytes. `Ok(None)` when the driver picks another size or format (the
    /// mode is not offered).
    pub fn set_format(&mut self, width: u32, height: u32, fourcc: u32) -> Result<Option<usize>> {
        let mut f = Format::zeroed();
        f.type_ = BUF_TYPE_VIDEO_CAPTURE;
        let mut pix = PixFormat::zeroed();
        pix.width = width;
        pix.height = height;
        pix.pixelformat = fourcc;
        pix.field = FIELD_NONE;
        f.fmt.pix = pix;
        // SAFETY: `VIDIOC_S_FMT` reads and updates one `v4l2_format`.
        unsafe { ioctl(&self.fd, Updater::<VIDIOC_S_FMT, Format>::new(&mut f)) }
            .map_err(|e| errno_error("VIDIOC_S_FMT", e))?;
        // SAFETY: the driver filled the `pix` member for a capture type.
        let pix = unsafe { f.fmt.pix };
        Ok(
            (pix.width == width && pix.height == height && pix.pixelformat == fourcc)
                .then_some(pix.sizeimage as usize),
        )
    }

    /// Asks for `fps` frames per second, when the driver supports it.
    pub fn set_fps(&mut self, fps: u32) -> Result<()> {
        let mut p = StreamParm::zeroed();
        p.type_ = BUF_TYPE_VIDEO_CAPTURE;
        // SAFETY: `VIDIOC_G_PARM` reads and updates one `v4l2_streamparm`.
        unsafe { ioctl(&self.fd, Updater::<VIDIOC_G_PARM, StreamParm>::new(&mut p)) }
            .map_err(|e| errno_error("VIDIOC_G_PARM", e))?;
        // SAFETY: the driver filled the `capture` member for a capture type.
        let mut capture = unsafe { p.parm.capture };
        if capture.capability & CAP_TIMEPERFRAME == 0 || fps == 0 {
            return Ok(());
        }
        capture.timeperframe = Fract {
            numerator: 1,
            denominator: fps,
        };
        p.parm.capture = capture;
        // SAFETY: `VIDIOC_S_PARM` reads and updates one `v4l2_streamparm`.
        unsafe { ioctl(&self.fd, Updater::<VIDIOC_S_PARM, StreamParm>::new(&mut p)) }
            .map_err(|e| errno_error("VIDIOC_S_PARM", e))
    }

    /// Allocates and maps `count` buffers, queues them and starts streaming.
    pub fn start(&mut self, count: u32) -> Result<()> {
        if self.streaming {
            return Ok(());
        }
        let mut req = RequestBuffers::zeroed();
        req.count = count;
        req.type_ = BUF_TYPE_VIDEO_CAPTURE;
        req.memory = MEMORY_MMAP;
        // SAFETY: `VIDIOC_REQBUFS` reads and updates one `v4l2_requestbuffers`.
        unsafe {
            ioctl(
                &self.fd,
                Updater::<VIDIOC_REQBUFS, RequestBuffers>::new(&mut req),
            )
        }
        .map_err(|e| errno_error("VIDIOC_REQBUFS", e))?;
        if req.count == 0 {
            return Err(ScanError::Device("VIDIOC_REQBUFS: no buffer".into()));
        }
        for index in 0..req.count {
            let mut b = Self::buffer(index);
            // SAFETY: `VIDIOC_QUERYBUF` reads and updates one `v4l2_buffer`.
            unsafe { ioctl(&self.fd, Updater::<VIDIOC_QUERYBUF, Buffer>::new(&mut b)) }
                .map_err(|e| errno_error("VIDIOC_QUERYBUF", e))?;
            let len = b.length as usize;
            // SAFETY: for MMAP buffers the driver sets `m.offset`.
            let offset = unsafe { b.m.offset };
            // SAFETY: a fresh shared mapping of the driver buffer at the
            // offset and length the driver reported; unmapped in `unmap`.
            let ptr = unsafe {
                mmap(
                    std::ptr::null_mut(),
                    len,
                    ProtFlags::READ | ProtFlags::WRITE,
                    MapFlags::SHARED,
                    &self.fd,
                    u64::from(offset),
                )
            }
            .map_err(|e| errno_error("mmap", e))?;
            let ptr = NonNull::new(ptr).ok_or_else(|| ScanError::Device("mmap: null".into()))?;
            self.buffers.push(Mapping { ptr, len });
            self.queue(index)?;
        }
        let type_ = BUF_TYPE_VIDEO_CAPTURE as c_int;
        // SAFETY: `VIDIOC_STREAMON` reads one `int` (the buffer type).
        unsafe { ioctl(&self.fd, Setter::<VIDIOC_STREAMON, c_int>::new(type_)) }
            .map_err(|e| errno_error("VIDIOC_STREAMON", e))?;
        self.streaming = true;
        Ok(())
    }

    fn buffer(index: u32) -> Buffer {
        let mut b = Buffer::zeroed();
        b.index = index;
        b.type_ = BUF_TYPE_VIDEO_CAPTURE;
        b.memory = MEMORY_MMAP;
        b
    }

    fn queue(&mut self, index: u32) -> Result<()> {
        let mut b = Self::buffer(index);
        // SAFETY: `VIDIOC_QBUF` reads and updates one `v4l2_buffer`; the
        // buffer memory is the driver's own mapping.
        unsafe { ioctl(&self.fd, Updater::<VIDIOC_QBUF, Buffer>::new(&mut b)) }
            .map_err(|e| errno_error("VIDIOC_QBUF", e))
    }

    /// Waits at most `timeout` for a frame and copies it into `out`
    /// (resized to the frame), then gives the buffer back to the driver.
    /// `Ok(None)` when no complete frame came in time.
    ///
    /// # Errors
    /// `device-lost` when the device is gone (`POLLHUP` / `POLLERR`,
    /// `ENODEV`), [`ScanError::Device`] for other failures.
    pub fn next(&mut self, timeout: Duration, out: &mut Vec<u8>) -> Result<Option<Dequeued>> {
        let ts = Timespec {
            tv_sec: timeout.as_secs().try_into().unwrap_or(i64::MAX),
            tv_nsec: timeout.subsec_nanos().into(),
        };
        let mut fds = [PollFd::new(&self.fd, PollFlags::IN)];
        match poll(&mut fds, Some(&ts)) {
            Ok(0) | Err(Errno::INTR) => return Ok(None),
            Ok(_) => {}
            Err(e) => return Err(errno_error("poll", e)),
        }
        let revents = fds[0].revents();
        if revents.intersects(PollFlags::HUP | PollFlags::ERR) {
            return Err(ScanError::DeviceLost(format!(
                "poll: {revents:?} (the scanner left the USB bus)"
            )));
        }
        let mut b = Self::buffer(0);
        // SAFETY: `VIDIOC_DQBUF` reads and updates one `v4l2_buffer`.
        match unsafe { ioctl(&self.fd, Updater::<VIDIOC_DQBUF, Buffer>::new(&mut b)) } {
            Ok(()) => {}
            Err(Errno::AGAIN) => return Ok(None),
            Err(e) => return Err(errno_error("VIDIOC_DQBUF", e)),
        }
        let index = b.index;
        let Some(m) = self.buffers.get(index as usize) else {
            return Err(ScanError::Device(format!(
                "VIDIOC_DQBUF: unknown buffer {index}"
            )));
        };
        let bytes = (b.bytesused as usize).min(m.len);
        let corrupt = b.flags & BUF_FLAG_ERROR != 0;
        if !corrupt {
            // SAFETY: the mapping is `m.len` bytes long and the driver does
            // not write into a dequeued buffer; it is queued again only
            // after this copy.
            let data = unsafe { std::slice::from_raw_parts(m.ptr.as_ptr().cast::<u8>(), bytes) };
            out.clear();
            out.extend_from_slice(data);
        }
        self.queue(index)?;
        if corrupt {
            return Ok(None);
        }
        let timestamp_us = (b.timestamp.sec.max(0) as u64)
            .saturating_mul(1_000_000)
            .saturating_add(b.timestamp.usec.max(0) as u64);
        Ok(Some(Dequeued {
            bytes,
            timestamp_us,
        }))
    }

    /// Stops streaming and releases the buffers (errors ignored: the
    /// device may already be gone).
    pub fn stop(&mut self) {
        if self.streaming {
            let type_ = BUF_TYPE_VIDEO_CAPTURE as c_int;
            // SAFETY: `VIDIOC_STREAMOFF` reads one `int` (the buffer type).
            let _ = unsafe { ioctl(&self.fd, Setter::<VIDIOC_STREAMOFF, c_int>::new(type_)) };
            self.streaming = false;
        }
        for m in self.buffers.drain(..) {
            // SAFETY: `m` is a mapping made in `start`, unmapped once.
            let _ = unsafe { munmap(m.ptr.as_ptr(), m.len) };
        }
    }
}

/// `struct uvc_xu_control_query` (`linux/uvcvideo.h`).
#[repr(C)]
struct XuQuery {
    unit: u8,
    selector: u8,
    query: u8,
    size: u16,
    data: *mut u8,
}

const UVCIOC_CTRL_QUERY: Opcode = opcode::read_write::<XuQuery>(b'u', 0x21);
const UVC_SET_CUR: u8 = 0x01;
const UVC_GET_CUR: u8 = 0x81;
const UVC_GET_LEN: u8 = 0x85;

impl Capture {
    /// One query of `selector` of the vendor extension unit.
    fn xu_query(&self, selector: u8, query: u8, data: &mut [u8]) -> Result<()> {
        let mut q = XuQuery {
            unit: SHELL_UNIT,
            selector,
            query,
            size: data.len() as u16,
            data: data.as_mut_ptr(),
        };
        // SAFETY: `UVCIOC_CTRL_QUERY` reads one `uvc_xu_control_query` and
        // reads or writes `size` bytes at `data`, a live buffer of that
        // length for the duration of the call.
        unsafe { ioctl(&self.fd, Updater::<UVCIOC_CTRL_QUERY, XuQuery>::new(&mut q)) }
            .map_err(|e| errno_error("UVCIOC_CTRL_QUERY", e))
    }

    /// Checks that `selector` of the extension unit is `expected` bytes
    /// long (`what` names the command for the error).
    fn check_len(&self, selector: u8, expected: usize, what: &str) -> Result<()> {
        let mut len = [0u8; 2];
        self.xu_query(selector, UVC_GET_LEN, &mut len)?;
        let len = u16::from_le_bytes(len) as usize;
        if len != expected {
            return Err(ScanError::Device(format!(
                "extension unit {SHELL_UNIT} selector {selector} is {len} bytes, not \
                 {expected}: no {what} sent"
            )));
        }
        Ok(())
    }

    /// Sends shell `payloads` after checking the selector length.
    fn send_shell(&self, payloads: Vec<[u8; SHELL_PAYLOAD_LEN]>, what: &str) -> Result<()> {
        self.check_len(SHELL_SELECTOR, SHELL_PAYLOAD_LEN, what)?;
        for mut p in payloads {
            self.xu_query(SHELL_SELECTOR, UVC_SET_CUR, &mut p)?;
        }
        Ok(())
    }

    /// Sets the fill light (the lines of [`FillLight`], nothing else),
    /// after checking that the shell selector is
    /// [`SHELL_PAYLOAD_LEN`] bytes long.
    ///
    /// # Errors
    /// [`ScanError::Device`] when the selector has another length or a
    /// query fails.
    pub fn set_fill_light(&self, light: FillLight) -> Result<()> {
        self.send_shell(light.payloads(), "light command")
    }

    /// Applies `profile` (its lines, nothing else), with the checks of
    /// [`Capture::set_fill_light`].
    ///
    /// # Errors
    /// As [`Capture::set_fill_light`].
    pub fn set_exposure_profile(&self, profile: ExposureProfile) -> Result<()> {
        self.send_shell(profile.payloads(), "exposure command")
    }

    /// Reads the factory file `file` over the read-only file channel (see
    /// [`super::factory`]): open for reading, read until the end, close.
    /// Call it with the stream idle.
    ///
    /// # Errors
    /// [`ScanError::Device`] when a selector has another length, a query
    /// fails or a reply is malformed.
    pub fn read_factory_file(&self, file: FactoryFile) -> Result<Vec<u8>> {
        self.check_len(OPEN_SELECTOR, FILE_PAYLOAD_LEN, "file read")?;
        self.check_len(READ_SELECTOR, FILE_PAYLOAD_LEN, "file read")?;
        self.xu_query(OPEN_SELECTOR, UVC_SET_CUR, &mut file.open_payload())?;
        let mut data = Vec::new();
        let read = (|| -> Result<()> {
            loop {
                let mut reply = [0u8; FILE_PAYLOAD_LEN];
                self.xu_query(READ_SELECTOR, UVC_GET_CUR, &mut reply)?;
                if !append_chunk(&reply, &mut data)? {
                    return Ok(());
                }
            }
        })();
        // the session is closed whatever happened after it was opened
        let mut close = CLOSE_PAYLOAD;
        let closed = self.xu_query(OPEN_SELECTOR, UVC_SET_CUR, &mut close);
        read.and(closed)?;
        if data.is_empty() {
            return Err(ScanError::Device(format!(
                "{}: empty or missing on the device",
                file.device_path()
            )));
        }
        Ok(data)
    }
}

impl Drop for Capture {
    fn drop(&mut self) {
        self.stop();
    }
}

#[cfg(test)]
mod tests {
    use std::mem::{offset_of, size_of};

    use super::*;

    /// Sizes and offsets of `linux/videodev2.h` on 64-bit Linux, and the
    /// ioctl numbers they produce.
    #[test]
    #[cfg(target_pointer_width = "64")]
    fn layouts_match_videodev2() {
        assert_eq!(size_of::<Capability>(), 104);
        assert_eq!(size_of::<PixFormat>(), 48);
        assert_eq!(size_of::<Format>(), 208);
        assert_eq!(offset_of!(Format, fmt), 8);
        assert_eq!(size_of::<CaptureParm>(), 40);
        assert_eq!(size_of::<StreamParm>(), 204);
        assert_eq!(size_of::<RequestBuffers>(), 20);
        assert_eq!(size_of::<Timecode>(), 16);
        assert_eq!(size_of::<Buffer>(), 88);
        assert_eq!(offset_of!(Buffer, timestamp), 24);
        assert_eq!(offset_of!(Buffer, sequence), 56);
        assert_eq!(offset_of!(Buffer, m), 64);
        assert_eq!(offset_of!(Buffer, length), 72);
        assert_eq!(offset_of!(Buffer, request_fd), 80);

        assert_eq!(VIDIOC_QUERYCAP, 0x8068_5600);
        assert_eq!(VIDIOC_S_FMT, 0xc0d0_5605);
        assert_eq!(VIDIOC_REQBUFS, 0xc014_5608);
        assert_eq!(VIDIOC_QUERYBUF, 0xc058_5609);
        assert_eq!(VIDIOC_QBUF, 0xc058_560f);
        assert_eq!(VIDIOC_DQBUF, 0xc058_5611);
        assert_eq!(VIDIOC_STREAMON, 0x4004_5612);
        assert_eq!(VIDIOC_STREAMOFF, 0x4004_5613);
        assert_eq!(VIDIOC_G_PARM, 0xc0cc_5615);
        assert_eq!(VIDIOC_S_PARM, 0xc0cc_5616);
        assert_eq!(PIX_FMT_Y16, 0x2036_3159);
        assert_eq!(size_of::<XuQuery>(), 16);
        assert_eq!(UVCIOC_CTRL_QUERY, 0xc010_7521);
    }

    #[test]
    fn errno_mapping() {
        assert_eq!(errno_error("x", Errno::NODEV).code(), "device-lost");
        assert_eq!(errno_error("x", Errno::IO).code(), "device");
        let busy = errno_error("VIDIOC_S_FMT", Errno::BUSY).to_string();
        assert!(busy.contains("another program"), "{busy}");
        assert_eq!(text(b"uvcvideo\0\0\0"), "uvcvideo");
    }
}
