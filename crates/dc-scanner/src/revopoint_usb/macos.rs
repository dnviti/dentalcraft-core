//! macOS platform layer of the USB backend: the `2207:110c` devices from
//! `nusb` (IOKit), the depth stream read by the user-space UVC reader of
//! [`super::uvc_bulk`] on the video-streaming interface (interface 1), the
//! vendor extension unit reached with class requests on endpoint 0.
//!
//! macOS has no capture node the backend could open: its camera stack (the
//! `UVCAssistant` extension behind AVFoundation) does not deliver `Y16`
//! frames, and it may hold the streaming interface. [`Capture::start`]
//! takes the interface:
//!
//! 1. with a plain claim (`USBInterfaceOpen`, through `nusb`);
//! 2. when another client holds it, with `USBInterfaceOpenSeize`
//!    ([`seize`]), which asks that client to close it;
//! 3. when that is refused too, the stream fails with
//!    `platform-capture-denied`: scanning is not available on this Mac, the
//!    app offers the import of scan files instead.
//!
//! The vendor requests ([`super::xu`]) are class requests to the
//! video-control interface (interface 0) on endpoint 0
//! ([`XuRequest::setup`]): IOKit takes them through the device without any
//! interface claimed, from a normal user. While no stream runs, the device
//! is opened for each request and closed after it, so that an idle scanner
//! stays free for the system; while streaming, the requests share the
//! stream's handle.
//!
//! Written and compile-checked on Linux (2026-11-20); not run on a Mac yet.
//! The same reader streamed the MINI 2 on Linux with frames equal to the
//! V4L2 ones.

mod seize;

use std::time::Duration;

use nusb::MaybeFuture;
use nusb::transfer::{ControlIn, ControlOut, ControlType, Recipient};

use super::uvc_bulk::probe::{FrameDesc, StreamingInterface};
use super::uvc_bulk::{self, BulkPipe, BulkStream, NusbPipe, usb_error};
use super::xu::{self, Selector, XuRequest, XuTransport};
use super::{BACKEND_ID, USB_PRODUCT_ID, USB_VENDOR_ID};
use crate::{DeviceInfo, Result, ScanError};

/// `v4l2_fourcc('Y', '1', '6', ' ')`: 16-bit greyscale, the FOURCC of its
/// UVC format GUID.
pub(super) const PIX_FMT_Y16: u32 = u32::from_le_bytes(*b"Y16 ");
/// The video-streaming interface of the depth camera.
const DEPTH_STREAMING: u8 = 1;
/// USB class of the video interfaces.
const CLASS_VIDEO: u8 = 0x0e;
/// USB subclass of a video-streaming interface.
const SUBCLASS_STREAMING: u8 = 0x02;
/// Longest wait of one extension-unit request.
const XU_TIMEOUT: Duration = Duration::from_secs(1);

/// IORegistry entry id of the USB device: a new one at every enumeration.
pub(super) type Address = u64;

/// The depth capture "node": the USB device itself (its streaming interface
/// is taken when the stream starts).
pub(super) type Node = nusb::DeviceInfo;

/// IOKit is always there; whether the stream can be taken is only known
/// when it starts (`platform-capture-denied`).
pub(super) fn availability() -> Result<()> {
    Ok(())
}

/// A listed `2207:110c` device.
pub(super) struct Found {
    pub(super) info: DeviceInfo,
    /// IORegistry entry id of this enumeration.
    pub(super) address: Address,
    device: nusb::DeviceInfo,
}

impl Found {
    /// The device, when it has the video-streaming interface of the depth
    /// camera.
    pub(super) fn depth_node(&self) -> Option<Node> {
        self.device
            .interfaces()
            .any(|i| {
                i.interface_number() == DEPTH_STREAMING
                    && i.class() == CLASS_VIDEO
                    && i.subclass() == SUBCLASS_STREAMING
            })
            .then(|| self.device.clone())
    }

    /// Detail of a device without it.
    pub(super) fn no_node(&self) -> String {
        format!(
            "{} (USB location {:#010x}) has no video-streaming interface {DEPTH_STREAMING}",
            self.info.id,
            self.device.location_id()
        )
    }
}

/// Whether the device has the vendor extension unit (see [`super::xu`]),
/// read from its USB configuration descriptor (cached when the capture was
/// opened, no device I/O); the reason when not.
pub(super) fn vendor_unit(_found: &Found, capture: &Capture) -> std::result::Result<(), String> {
    if xu::has_unit(&capture.config) {
        Ok(())
    } else {
        Err(xu::missing_unit("the USB descriptors"))
    }
}

/// The `2207:110c` devices on the bus.
pub(super) fn find_devices() -> Result<Vec<Found>> {
    let devices = nusb::list_devices()
        .wait()
        .map_err(|e| ScanError::Device(format!("USB enumeration: {e}")))?;
    Ok(devices
        .filter(|d| d.vendor_id() == USB_VENDOR_ID && d.product_id() == USB_PRODUCT_ID)
        .map(|d| {
            let serial = d
                .serial_number()
                .filter(|s| !s.is_empty())
                .map(str::to_owned);
            Found {
                info: DeviceInfo {
                    backend: BACKEND_ID.to_owned(),
                    id: serial
                        .clone()
                        .unwrap_or_else(|| format!("usb-{:08x}", d.location_id())),
                    name: super::device_name(d.device_version()).to_owned(),
                    serial,
                },
                address: d.registry_entry_id(),
                device: d,
            }
        })
        .collect())
}

/// Reads the device's configuration (its streaming interface and the
/// vendor unit) and closes it again. The error says whether it may be
/// transient: the device may be leaving or coming back.
pub(super) fn open_capture(node: &Node) -> std::result::Result<Capture, (bool, ScanError)> {
    let device = node
        .open()
        .wait()
        .map_err(|e| (transient(&e), usb_error("open the scanner", &e)))?;
    let config = device
        .active_configuration()
        .map(|c| c.as_bytes().to_vec())
        .map_err(|e| (true, ScanError::Device(format!("USB configuration: {e}"))))?;
    drop(device);
    let vs = StreamingInterface::parse(&config, DEPTH_STREAMING).ok_or_else(|| {
        (
            false,
            ScanError::Device(format!(
                "interface {DEPTH_STREAMING} is no USB video-streaming interface"
            )),
        )
    })?;
    Ok(Capture {
        device: node.clone(),
        config,
        vs,
        choice: None,
        streaming: None,
    })
}

/// A device that is leaving or not back yet.
fn transient(e: &nusb::Error) -> bool {
    matches!(
        e.kind(),
        nusb::ErrorKind::Disconnected | nusb::ErrorKind::NotFound
    )
}

/// A dequeued frame, moved out of the reader's queue.
#[derive(Clone, Copy, Debug)]
pub(super) struct Dequeued {
    /// Bytes of the frame written into the output vector.
    pub bytes: usize,
    /// Host time when its first payload arrived, µs (monotonic).
    pub timestamp_us: u64,
}

/// The format chosen by [`Capture::set_format`] and [`Capture::set_fps`].
#[derive(Clone, Debug)]
struct Choice {
    /// `bFormatIndex`.
    format: u8,
    frame: FrameDesc,
    /// Frame interval, 100 ns units.
    interval: u32,
}

/// A running depth stream. The fields drop in order: the reader stops (and
/// halts the endpoint) before the interface and the device are released.
struct Streaming {
    stream: BulkStream,
    /// The claimed interface; `None` when it was seized (the pipe holds
    /// it then).
    _interface: Option<nusb::Interface>,
    device: nusb::Device,
}

/// The depth camera of one device, streaming once [`Capture::start`]
/// succeeded.
pub(super) struct Capture {
    device: nusb::DeviceInfo,
    /// The active configuration descriptor.
    config: Vec<u8>,
    /// Interface 1, the depth stream.
    vs: StreamingInterface,
    choice: Option<Choice>,
    streaming: Option<Streaming>,
}

impl Capture {
    /// Picks the `width x height` frame of the `fourcc` format of the
    /// streaming interface; returns its size in bytes. `Ok(None)` when the
    /// interface does not offer it. Nothing is sent yet: the device is
    /// probed when the stream starts, once the interface is held.
    pub fn set_format(&mut self, width: u32, height: u32, fourcc: u32) -> Result<Option<usize>> {
        let Some((format, frame)) = self.vs.find(fourcc, width, height) else {
            return Ok(None);
        };
        let bytes = match frame.max_bytes {
            0 => width as usize * height as usize * 2,
            n => n as usize,
        };
        self.choice = Some(Choice {
            format: format.index,
            frame: frame.clone(),
            interval: frame.default_interval,
        });
        Ok(Some(bytes))
    }

    /// Asks for the frame interval closest to `fps` frames per second.
    pub fn set_fps(&mut self, fps: u32) -> Result<()> {
        if let Some(choice) = &mut self.choice {
            choice.interval = choice.frame.nearest_interval(fps);
        }
        Ok(())
    }

    /// Takes the streaming interface (see the module doc), negotiates the
    /// format and starts reading, at most `buffers` frames waiting.
    ///
    /// # Errors
    /// `platform-capture-denied` when the system keeps the interface,
    /// `device-lost` when the device left, [`ScanError::Device`] otherwise.
    pub fn start(&mut self, buffers: u32) -> Result<()> {
        if self.streaming.is_some() {
            return Ok(());
        }
        let choice = self
            .choice
            .clone()
            .ok_or_else(|| ScanError::Device("no video format chosen".into()))?;
        let device = self
            .device
            .open()
            .wait()
            .map_err(|e| usb_error("open the scanner", &e))?;
        let number = self.vs.number;
        let (pipe, interface): (Box<dyn BulkPipe>, _) = match device.claim_interface(number).wait()
        {
            Ok(interface) => {
                let pipe = NusbPipe::new(&interface, self.vs.endpoint)?;
                (Box::new(pipe), Some(interface))
            }
            Err(e) if transient(&e) => {
                return Err(usb_error(&format!("claim interface {number}"), &e));
            }
            Err(claim) => {
                // held by the system camera driver: take it over
                let pipe = seize::seize(self.device.registry_entry_id(), number)
                    .and_then(|s| s.into_pipe(self.vs.endpoint))
                    .map_err(|e| seize_error(&claim, e))?;
                (Box::new(pipe), None)
            }
        };
        let (format, frame, interval) = (choice.format, &choice.frame, choice.interval);
        let committed = match &interface {
            Some(interface) => uvc_bulk::negotiate(interface, &self.vs, format, frame, interval)?,
            // the seized interface is not nusb's: the device carries them
            None => uvc_bulk::negotiate(&device, &self.vs, format, frame, interval)?,
        };
        let stream = BulkStream::start(pipe, &committed, buffers as usize)?;
        self.streaming = Some(Streaming {
            stream,
            _interface: interface,
            device,
        });
        Ok(())
    }

    /// Waits at most `timeout` for a frame and moves it into `out`.
    /// `Ok(None)` when no frame came in time.
    ///
    /// # Errors
    /// `device-lost` when the device left, [`ScanError::Device`] for other
    /// failures.
    pub fn next(&mut self, timeout: Duration, out: &mut Vec<u8>) -> Result<Option<Dequeued>> {
        let Some(streaming) = self.streaming.as_mut() else {
            return Err(ScanError::Device("the video stream is not started".into()));
        };
        Ok(streaming.stream.next(timeout, out)?.map(|f| Dequeued {
            bytes: f.bytes,
            timestamp_us: f.timestamp_us,
        }))
    }

    /// Stops streaming (`CLEAR_FEATURE(ENDPOINT_HALT)`, as on Linux and
    /// Windows) and releases the interface and the device; errors are
    /// ignored, the device may be gone.
    pub fn stop(&mut self) {
        if let Some(mut streaming) = self.streaming.take() {
            streaming.stream.stop();
        }
    }
}

impl Drop for Capture {
    fn drop(&mut self) {
        self.stop();
    }
}

/// The error of a stream start whose claim (`claim`) and seizure (`seized`)
/// both failed.
fn seize_error(claim: &nusb::Error, seized: seize::SeizeError) -> ScanError {
    match seized {
        seize::SeizeError::Denied(why) => ScanError::PlatformCaptureDenied(format!(
            "the video-streaming interface of the scanner is held by the system (claim: \
             {claim}; {why})"
        )),
        seize::SeizeError::Lost(why) => ScanError::DeviceLost(why),
        seize::SeizeError::Failed(why) => {
            ScanError::Device(format!("take the video-streaming interface: {why}"))
        }
    }
}

/// The vendor requests as class requests on endpoint 0 (see the module
/// doc).
impl XuTransport for Capture {
    fn query(&self, selector: Selector, request: XuRequest<'_>) -> Result<()> {
        let opened;
        let device = match &self.streaming {
            Some(streaming) => &streaming.device,
            None => {
                opened = self
                    .device
                    .open()
                    .wait()
                    .map_err(|e| usb_error("open the scanner", &e))?;
                &opened
            }
        };
        exchange(device, selector, request)
    }
}

/// One request of [`super::xu`] on endpoint 0 of `device`.
fn exchange(device: &nusb::Device, selector: Selector, request: XuRequest<'_>) -> Result<()> {
    let setup = request.setup(selector);
    // what `ControlType::Class` and `Recipient::Interface` encode, both ways
    debug_assert_eq!(setup.request_type & 0x7f, 0x21);
    let what = || {
        format!(
            "extension unit {} selector {} request {:#04x}",
            xu::UNIT,
            selector.id(),
            setup.request
        )
    };
    let error = |e: nusb::transfer::TransferError| match e {
        nusb::transfer::TransferError::Disconnected => {
            ScanError::DeviceLost(format!("{}: {e}", what()))
        }
        e => ScanError::Device(format!("{}: {e}", what())),
    };
    let control_in = || {
        device
            .control_in(
                ControlIn {
                    control_type: ControlType::Class,
                    recipient: Recipient::Interface,
                    request: setup.request,
                    value: setup.value,
                    index: setup.index,
                    length: setup.length,
                },
                XU_TIMEOUT,
            )
            .wait()
            .map_err(error)
            .and_then(|reply| {
                if reply.len() == usize::from(setup.length) {
                    Ok(reply)
                } else {
                    Err(ScanError::Device(format!(
                        "{}: {} bytes, not {}",
                        what(),
                        reply.len(),
                        setup.length
                    )))
                }
            })
    };
    match request {
        XuRequest::SetCur(payload) => device
            .control_out(
                ControlOut {
                    control_type: ControlType::Class,
                    recipient: Recipient::Interface,
                    request: setup.request,
                    value: setup.value,
                    index: setup.index,
                    data: payload.as_bytes(),
                },
                XU_TIMEOUT,
            )
            .wait()
            .map_err(error),
        XuRequest::GetCur(reply) => {
            reply.copy_from_slice(&control_in()?);
            Ok(())
        }
        XuRequest::GetLen(len) => {
            let reply = control_in()?;
            *len = u16::from_le_bytes([reply[0], reply[1]]);
            Ok(())
        }
        XuRequest::GetInfo(info) => {
            *info = control_in()?[0];
            Ok(())
        }
    }
}
