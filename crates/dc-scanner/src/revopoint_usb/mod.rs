//! Revopoint MINI 2 over plain USB video, backend `revopoint-usb` (alpha,
//! Linux only).
//!
//! The MINI 2 computes depth on the device and streams it as a standard UVC
//! camera (USB `2207:110c`): the depth node (interface 0) sends `Y16`
//! frames, the colour node (interface 2) `MJPG` 1600x1200. This backend
//! reads the depth node with plain V4L2 calls ([`v4l2`]) and needs neither
//! the vendor SDK nor any vendor command: it sends no UVC extension-unit
//! request, no HID report and no firmware, only format negotiation and
//! streaming. The colour stream is not read (it is not registered to the
//! depth image, and stopping it also resets the depth stream).
//!
//! * **Discovery.** [`ScannerBackend::list`] enumerates the USB devices with
//!   `nusb`, keeps the `2207:110c` devices and finds their V4L2 nodes under
//!   `/sys/bus/usb/devices/<port>:1.0/video4linux/` (depth) and `<port>:1.2`
//!   (colour). It lists, by serial number, only the devices whose depth
//!   node exists: a scanner held by another program (e.g. the USB
//!   redirection of a virtual machine, which unbinds `uvcvideo`) or still
//!   re-enumerating cannot stream and is not offered. The same USB id is
//!   used by the MINI and the POP4, so the name says "MINI 2" only for the
//!   `bcdDevice` seen on the MINI 2 ([`MINI2_BCD_DEVICE`]), else it is
//!   generic.
//! * **Frames.** [`frame`] describes the layout: row 0 of the depth image is
//!   a header (device time, IMU orientation), the rest is depth as `u16`
//!   times the depth scale; `0` is no depth. Frames are copied out of the
//!   driver buffers before these are queued again.
//! * **Calibration.** Intrinsics and depth scale come from a user file or
//!   the alpha defaults, see [`calib`]. Without a calibration file the
//!   geometry is an estimate (focal length from the published field of
//!   view, depth scale measured on one unit).
//! * **Exposure.** The device runs its own auto exposure; V4L2 offers only
//!   brightness and gain, which are not an exposure time.
//!   [`ScannerDevice::set_exposure_us`] is therefore a no-op.
//!
//! # Stop and restart
//!
//! Stopping the stream makes the MINI 2 leave the USB bus and come back
//! 1.5-2.5 s later as a new USB device (same serial); a stream started on
//! the old device within about 3 s hangs. So [`RevopointUsbDevice`]:
//!
//! * `stop` turns the stream off and closes the node;
//! * the first `start` opens the node at once; a `start` after a stop (or
//!   after the device was lost) returns at once and the reopening happens
//!   in [`ScannerDevice::next_frame`], on the acquisition thread: it waits
//!   until the serial shows up again at a new USB address (or the old one
//!   has stayed for [`REENUMERATION_GRACE`]) with its depth node, then
//!   reopens it. Each call waits at most its `timeout` and returns
//!   `Ok(None)` meanwhile, so the scan session can cancel the wait; after
//!   [`UsbOptions::reconnect_timeout`] it fails with `device-not-found`.
//! * a device that disappears while streaming (`POLLHUP`, `ENODEV`) ends
//!   the stream with `device-lost`.
//!
//! Pausing a [`crate::ScanSession`] keeps the stream running, so pause and
//! resume never trigger a reconnection; only stop / start do.
//!
//! On other platforms [`ScannerBackend::availability`] reports
//! `platform-unsupported` and no device is listed.

pub mod calib;
pub mod frame;
#[cfg(target_os = "linux")]
mod v4l2;

use std::time::Duration;

use serde::{Deserialize, Serialize};

pub use calib::{Calibration, CalibrationSource};
pub use frame::StreamMode;

#[cfg(not(target_os = "linux"))]
use crate::{DeviceInfo, Result, ScannerBackend, ScannerDevice};

/// Id of the USB backend.
pub const BACKEND_ID: &str = "revopoint-usb";
/// USB vendor id of the Revopoint scanners (Rockchip).
pub const USB_VENDOR_ID: u16 = 0x2207;
/// USB product id shared by the MINI, MINI 2 and POP4.
pub const USB_PRODUCT_ID: u16 = 0x110c;
/// `bcdDevice` of the MINI 2 seen so far (firmware dependent).
pub const MINI2_BCD_DEVICE: u16 = 0x0419;
/// How long a device that kept its USB address after a stop is waited for
/// before it is reopened anyway (the hang window after a stop is ~3 s).
pub const REENUMERATION_GRACE: Duration = Duration::from_millis(3500);

/// Display name of a device with this `bcdDevice`.
pub fn device_name(bcd_device: u16) -> &'static str {
    if bcd_device == MINI2_BCD_DEVICE {
        "Revopoint MINI 2 (USB)"
    } else {
        "Revopoint scanner (USB)"
    }
}

/// Stream options of the USB backend.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct UsbOptions {
    /// `Y16` mode to negotiate; [`StreamMode::Depth`] is the fallback when
    /// the driver refuses it.
    pub mode: StreamMode,
    /// Frame rate asked for (the device offers 5, 10 and 15).
    pub fps: u32,
    /// Driver buffers.
    pub buffers: u32,
    /// Longest wait for the device to come back after a stop.
    pub reconnect_timeout: Duration,
    /// Keep the infrared pair of the latest frame
    /// ([`RevopointUsbDevice::latest_ir`]).
    pub keep_ir: bool,
}

impl Default for UsbOptions {
    fn default() -> Self {
        Self {
            mode: StreamMode::Full,
            fps: 15,
            buffers: 4,
            reconnect_timeout: Duration::from_secs(8),
            keep_ir: false,
        }
    }
}

/// The Revopoint USB video backend (see the module doc).
#[derive(Clone, Debug, Default)]
pub struct RevopointUsbBackend {
    options: UsbOptions,
}

impl RevopointUsbBackend {
    /// A backend whose devices stream with `options`.
    pub fn with_options(options: UsbOptions) -> Self {
        Self { options }
    }
}

#[cfg(not(target_os = "linux"))]
impl ScannerBackend for RevopointUsbBackend {
    fn id(&self) -> &'static str {
        BACKEND_ID
    }

    fn availability(&self) -> Result<()> {
        Err(crate::ScanError::PlatformUnsupported(format!(
            "{BACKEND_ID} uses Linux V4L2; on this system use revopoint-sdk"
        )))
    }

    fn list(&self) -> Result<Vec<DeviceInfo>> {
        Ok(Vec::new())
    }

    fn open(&self, _device_id: &str) -> Result<Box<dyn ScannerDevice>> {
        Err(self.availability().unwrap_err())
    }
}

#[cfg(target_os = "linux")]
pub use linux::RevopointUsbDevice;

#[cfg(target_os = "linux")]
mod linux {
    use std::path::{Path, PathBuf};
    use std::time::{Duration, Instant};

    use nusb::MaybeFuture;

    use super::calib::{Calibration, load_calibration};
    use super::frame::{self, StreamMode};
    use super::v4l2::{Capture, PIX_FMT_Y16};
    use super::{BACKEND_ID, REENUMERATION_GRACE, USB_PRODUCT_ID, USB_VENDOR_ID, UsbOptions};
    use crate::{
        DepthFrame, DeviceInfo, Result, RevopointUsbBackend, ScanError, ScannerBackend,
        ScannerDevice,
    };

    /// Longest sleep of one reconnection step inside `next_frame`.
    const RECONNECT_STEP: Duration = Duration::from_millis(100);

    /// A listed `2207:110c` device.
    struct Found {
        info: DeviceInfo,
        /// USB (bus, address) of this enumeration.
        address: (u8, u8),
        /// `/sys/bus/usb/devices/<port>`.
        sysfs: PathBuf,
    }

    /// The `2207:110c` devices on the bus.
    fn find_devices() -> Result<Vec<Found>> {
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
                let port = d
                    .sysfs_path()
                    .file_name()
                    .map(|n| n.to_string_lossy().into_owned())
                    .unwrap_or_default();
                Found {
                    info: DeviceInfo {
                        backend: BACKEND_ID.to_owned(),
                        id: serial.clone().unwrap_or_else(|| format!("usb-{port}")),
                        name: super::device_name(d.device_version()).to_owned(),
                        serial,
                    },
                    address: (d.busnum(), d.device_address()),
                    sysfs: d.sysfs_path().to_owned(),
                }
            })
            .collect())
    }

    /// The V4L2 nodes of a device.
    #[derive(Clone, Debug, PartialEq, Eq)]
    struct Nodes {
        /// Depth capture node (interface 0).
        depth: PathBuf,
        /// Colour capture node (interface 2), not streamed in this alpha.
        #[allow(dead_code, reason = "resolved for the colour stream to come")]
        color: Option<PathBuf>,
    }

    /// The capture node of USB interface `interface` of the device at
    /// `sysfs`: the `video4linux` entry with `index` 0 (the others are
    /// metadata nodes).
    fn capture_node(sysfs: &Path, interface: u8) -> Option<PathBuf> {
        let port = sysfs.file_name()?.to_str()?;
        let video4linux = sysfs
            .join(format!("{port}:1.{interface}"))
            .join("video4linux");
        let mut nodes: Vec<String> = std::fs::read_dir(&video4linux)
            .ok()?
            .filter_map(|e| e.ok()?.file_name().into_string().ok())
            .filter(|n| n.starts_with("video"))
            .collect();
        nodes.sort_by_key(|n| n[5..].parse::<u32>().unwrap_or(u32::MAX));
        nodes
            .iter()
            .find(|n| {
                std::fs::read_to_string(video4linux.join(n).join("index"))
                    .is_ok_and(|i| i.trim() == "0")
            })
            .or(nodes.first())
            .map(|n| Path::new("/dev").join(n))
    }

    fn nodes(sysfs: &Path) -> Option<Nodes> {
        Some(Nodes {
            depth: capture_node(sysfs, 0)?,
            color: capture_node(sysfs, 2),
        })
    }

    /// Detail of a device whose depth node is missing.
    fn no_node(sysfs: &Path) -> String {
        let driver = std::fs::read_link(sysfs.join(format!(
            "{}:1.0/driver",
            sysfs.file_name().unwrap_or_default().to_string_lossy()
        )))
        .ok()
        .and_then(|p| p.file_name().map(|n| n.to_string_lossy().into_owned()))
        .unwrap_or_else(|| "none".into());
        format!(
            "no V4L2 depth node under {} (interface 0 driver: {driver}); another program, \
             e.g. a virtual machine USB redirection, may hold the scanner",
            sysfs.display()
        )
    }

    impl ScannerBackend for RevopointUsbBackend {
        fn id(&self) -> &'static str {
            BACKEND_ID
        }

        fn list(&self) -> Result<Vec<DeviceInfo>> {
            Ok(find_devices()?
                .into_iter()
                .filter(|f| nodes(&f.sysfs).is_some())
                .map(|f| f.info)
                .collect())
        }

        fn open(&self, device_id: &str) -> Result<Box<dyn ScannerDevice>> {
            let found = find_devices()?
                .into_iter()
                .find(|f| f.info.id == device_id)
                .ok_or_else(|| ScanError::DeviceNotFound(device_id.to_owned()))?;
            let nodes =
                nodes(&found.sysfs).ok_or_else(|| ScanError::Device(no_node(&found.sysfs)))?;
            let (calibration, _) = load_calibration()?;
            // the node must open; it is closed again until `start`
            Capture::open(&nodes.depth).map_err(|(_, e)| e)?;
            Ok(Box::new(RevopointUsbDevice {
                info: found.info,
                options: self.options.clone(),
                calibration,
                address: found.address,
                capture: None,
                mode: self.options.mode,
                restart: None,
                reconnect_deadline: None,
                buf: Vec::new(),
                latest_ir: None,
            }))
        }
    }

    /// A stream that ended: the device re-enumerates before it can stream
    /// again.
    #[derive(Clone, Copy, Debug)]
    struct Ended {
        /// When the stream ended.
        at: Instant,
        /// USB address of the enumeration that streamed.
        address: (u8, u8),
    }

    /// A Revopoint scanner streaming over USB video (see the module doc).
    pub struct RevopointUsbDevice {
        info: DeviceInfo,
        options: UsbOptions,
        calibration: Calibration,
        /// USB address of the current enumeration.
        address: (u8, u8),
        capture: Option<Capture>,
        /// The negotiated mode of `capture`.
        mode: StreamMode,
        /// Set when a stream ended: the next start waits for the device.
        restart: Option<Ended>,
        /// Set while a start waits for the device to come back.
        reconnect_deadline: Option<Instant>,
        buf: Vec<u8>,
        latest_ir: Option<Vec<u8>>,
    }

    impl std::fmt::Debug for RevopointUsbDevice {
        fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
            f.debug_struct("RevopointUsbDevice")
                .field("info", &self.info)
                .field("streaming", &self.capture.is_some())
                .field("mode", &self.mode)
                .finish_non_exhaustive()
        }
    }

    /// Why a reopen attempt did not stream yet.
    enum NotYet {
        /// The device is not back (or not ready): try again.
        Wait,
        /// A real failure.
        Fail(ScanError),
    }

    impl RevopointUsbDevice {
        /// The calibration in use (loaded when the device was opened).
        pub fn calibration(&self) -> &Calibration {
            &self.calibration
        }

        /// The negotiated stream mode (the requested one until a stream ran).
        pub fn mode(&self) -> StreamMode {
            self.mode
        }

        /// The infrared pair of the latest frame (`first` then `second`
        /// image, 800x600 each), when [`UsbOptions::keep_ir`] is set and the
        /// mode carries it.
        pub fn latest_ir(&self) -> Option<frame::IrPair<'_>> {
            let ir = self.latest_ir.as_deref()?;
            let (first, second) = ir.split_at(ir.len() / 2);
            Some(frame::IrPair { first, second })
        }

        /// Finds the device, opens its depth node and starts streaming.
        fn try_open(&mut self) -> std::result::Result<(), NotYet> {
            let found = find_devices()
                .map_err(NotYet::Fail)?
                .into_iter()
                .find(|f| f.info.id == self.info.id)
                .ok_or(NotYet::Wait)?;
            if let Some(ended) = self.restart
                && found.address == ended.address
                && ended.at.elapsed() < REENUMERATION_GRACE
            {
                // the old enumeration, about to leave the bus
                return Err(NotYet::Wait);
            }
            let nodes = nodes(&found.sysfs).ok_or(NotYet::Wait)?;
            let (mut capture, _) = Capture::open(&nodes.depth).map_err(|(errno, e)| {
                use rustix::io::Errno;
                // udev may not have applied the permissions yet
                if matches!(
                    errno,
                    Errno::NOENT | Errno::ACCESS | Errno::PERM | Errno::NODEV
                ) {
                    NotYet::Wait
                } else {
                    NotYet::Fail(e)
                }
            })?;
            let mode = self.negotiate(&mut capture).map_err(NotYet::Fail)?;
            capture
                .set_fps(self.options.fps)
                .and_then(|()| capture.start(self.options.buffers.max(2)))
                .map_err(NotYet::Fail)?;
            self.mode = mode;
            self.address = found.address;
            self.capture = Some(capture);
            self.restart = None;
            self.reconnect_deadline = None;
            Ok(())
        }

        /// Sets the requested mode, else [`StreamMode::Depth`].
        fn negotiate(&self, capture: &mut Capture) -> Result<StreamMode> {
            for mode in [self.options.mode, StreamMode::Depth] {
                let (w, h) = mode.size();
                if let Some(size) = capture.set_format(w, h, PIX_FMT_Y16)?
                    && size >= mode.frame_bytes()
                {
                    return Ok(mode);
                }
            }
            Err(ScanError::Device(
                "the depth node offers no Y16 800x600 format".into(),
            ))
        }

        /// The stream ended (stopped or lost): forget it.
        fn end_stream(&mut self) {
            if let Some(mut capture) = self.capture.take() {
                capture.stop();
                self.restart = Some(Ended {
                    at: Instant::now(),
                    address: self.address,
                });
            }
            self.reconnect_deadline = None;
        }

        /// One reconnection step: waits at most `timeout`.
        fn reconnect(&mut self, timeout: Duration, deadline: Instant) -> Result<()> {
            match self.try_open() {
                Ok(()) => Ok(()),
                Err(NotYet::Fail(e)) => {
                    self.reconnect_deadline = None;
                    Err(e)
                }
                Err(NotYet::Wait) if Instant::now() >= deadline => {
                    self.reconnect_deadline = None;
                    Err(ScanError::DeviceNotFound(format!(
                        "{} did not come back on USB within {:.0} s after the stream \
                         stopped",
                        self.info.id,
                        self.options.reconnect_timeout.as_secs_f32()
                    )))
                }
                Err(NotYet::Wait) => {
                    std::thread::sleep(timeout.min(RECONNECT_STEP));
                    Ok(())
                }
            }
        }

        fn depth_frame(&mut self, driver_timestamp_us: u64) -> Result<DepthFrame> {
            let planes = frame::split_planes(&self.buf, self.mode)?;
            if self.options.keep_ir {
                self.latest_ir = planes.ir.map(|ir| [ir.first, ir.second].concat());
            }
            Ok(DepthFrame {
                depth_mm: frame::depth_to_mm(planes.depth, self.calibration.depth_scale_mm),
                intrinsics: self.calibration.intrinsics,
                timestamp_us: planes
                    .header
                    .map_or(driver_timestamp_us, |h| u64::from(h.timestamp_ms) * 1000),
                color: None,
                imu_orientation: planes.header.and_then(|h| h.orientation),
            })
        }
    }

    impl ScannerDevice for RevopointUsbDevice {
        fn info(&self) -> DeviceInfo {
            self.info.clone()
        }

        fn start(&mut self) -> Result<()> {
            if self.capture.is_some() || self.reconnect_deadline.is_some() {
                return Ok(());
            }
            if self.restart.is_some() {
                // the device is re-enumerating: `next_frame` reopens it
                self.reconnect_deadline = Some(Instant::now() + self.options.reconnect_timeout);
                return Ok(());
            }
            match self.try_open() {
                Ok(()) => Ok(()),
                Err(NotYet::Fail(e)) => Err(e),
                Err(NotYet::Wait) => Err(ScanError::DeviceNotFound(format!(
                    "{}: the scanner or its depth node is not ready",
                    self.info.id
                ))),
            }
        }

        fn stop(&mut self) {
            self.end_stream();
        }

        fn next_frame(&mut self, timeout: Duration) -> Result<Option<DepthFrame>> {
            if let Some(deadline) = self.reconnect_deadline {
                self.reconnect(timeout, deadline)?;
                return Ok(None);
            }
            let Some(capture) = self.capture.as_mut() else {
                return Err(ScanError::Device("the depth stream is not started".into()));
            };
            match capture.next(timeout, &mut self.buf) {
                Ok(Some(dq)) => {
                    if dq.bytes < self.mode.frame_bytes() {
                        // a short (incomplete) frame: skip it
                        return Ok(None);
                    }
                    self.depth_frame(dq.timestamp_us).map(Some)
                }
                Ok(None) => Ok(None),
                Err(e) => {
                    if e.code() == "device-lost" {
                        self.end_stream();
                    }
                    Err(e)
                }
            }
        }
    }

    impl Drop for RevopointUsbDevice {
        fn drop(&mut self) {
            self.end_stream();
        }
    }

    #[cfg(test)]
    mod tests {
        use super::*;

        #[test]
        fn capture_node_prefers_index_zero() {
            let dir = tempfile::tempdir().unwrap();
            let sysfs = dir.path().join("1-1");
            for (iface, node, index) in [(0, "video1", "1"), (0, "video0", "0"), (2, "video3", "0")]
            {
                let d = sysfs.join(format!("1-1:1.{iface}/video4linux/{node}"));
                std::fs::create_dir_all(&d).unwrap();
                std::fs::write(d.join("index"), format!("{index}\n")).unwrap();
            }
            assert_eq!(
                nodes(&sysfs),
                Some(Nodes {
                    depth: PathBuf::from("/dev/video0"),
                    color: Some(PathBuf::from("/dev/video3")),
                })
            );
            // no depth interface bound to uvcvideo: no node, a clear detail
            let other = dir.path().join("1-2");
            std::fs::create_dir_all(other.join("1-2:1.0")).unwrap();
            assert_eq!(nodes(&other), None);
            assert!(no_node(&other).contains("driver: none"));
        }

        #[test]
        fn listing_never_fails_without_devices() {
            // whatever is plugged in, listing works and only lists 2207:110c
            let list = RevopointUsbBackend::default().list().unwrap();
            assert!(list.iter().all(|d| d.backend == BACKEND_ID));
            assert_eq!(
                RevopointUsbBackend::default()
                    .open("no-such-serial")
                    .err()
                    .unwrap()
                    .code(),
                "device-not-found"
            );
        }
    }
}
