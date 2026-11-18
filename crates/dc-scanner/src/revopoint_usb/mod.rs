//! Revopoint MINI 2 over plain USB video, backend `revopoint-usb` (alpha,
//! Linux and Windows).
//!
//! The MINI 2 computes depth on the device and streams it as a standard UVC
//! camera (USB `2207:110c`): the depth node (interface 0) sends `Y16`
//! frames, the colour node (interface 2) `MJPG` 1600x1200. This backend
//! reads the depth node with the standard capture API of the system, plain
//! V4L2 calls on Linux (`v4l2`) and Media Foundation on Windows (`mf`, on
//! the inbox UVC driver `usbvideo`), and needs no vendor SDK. Its vendor
//! commands are closed lists the user authorised on 2026-11-18, Linux only:
//! the fill light ([`light`]: on while streaming, off after), the fast
//! exposure profile ([`exposure`]) and the read-only factory calibration
//! file read ([`factory`]). It sends no other extension-unit request, no
//! HID report and no firmware. The colour stream is not read (it is not
//! registered to the depth image, and stopping it also resets the depth
//! stream).
//!
//! * **Discovery.** [`ScannerBackend::list`] enumerates the USB devices with
//!   `nusb` and keeps the `2207:110c` devices. On Linux their V4L2 nodes are
//!   under `/sys/bus/usb/devices/<port>:1.0/video4linux/` (depth) and
//!   `<port>:1.2` (colour); on Windows the depth node is the Media
//!   Foundation video source of interface 0 (`...&MI_00`) whose parent
//!   device node is the listed USB device. It lists, by serial number, only
//!   the devices whose depth node exists: a scanner held by another program
//!   (e.g. the USB redirection of a virtual machine, which unbinds
//!   `uvcvideo`) or still re-enumerating cannot stream and is not offered.
//!   The same USB id is used by the MINI and the POP4, so the name says
//!   "MINI 2" only for the `bcdDevice` seen on the MINI 2
//!   ([`MINI2_BCD_DEVICE`]), else it is generic.
//! * **Frames.** [`frame`] describes the layout: row 0 of the depth image is
//!   a header (device time, IMU orientation), the rest is depth as `u16`
//!   times the depth scale; `0` is no depth. Frames are copied out of the
//!   driver buffers before these are queued again.
//! * **Calibration.** Intrinsics and depth scale come from a user file, else
//!   the factory calibration read from the scanner and cached, else the
//!   alpha defaults, see [`calib`] and [`factory`]. The depth scale is an
//!   estimate measured on one unit in every case but a user file.
//! * **Exposure.** The device runs its own auto exposure; V4L2 offers only
//!   brightness and gain, which are not an exposure time, so
//!   [`ScannerDevice::set_exposure_us`] is a no-op. The fast profile
//!   ([`exposure`], [`crate::ScanSettings::fast_exposure`]) shortens the
//!   pattern time: about 6.8 instead of 2.7 frames per second.
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
//! * a device that disappears while streaming (`POLLHUP`, `ENODEV`; on
//!   Windows a failed or ended sample request) ends the stream with
//!   `device-lost`.
//!
//! Pausing a [`crate::ScanSession`] keeps the stream running, so pause and
//! resume never trigger a reconnection; only stop / start do.
//!
//! On Windows without Media Foundation (N editions without the Media
//! Feature Pack) and on other platforms [`ScannerBackend::availability`]
//! reports `platform-unsupported` and no device is listed.

pub mod calib;
#[cfg(any(target_os = "linux", windows))]
mod device;
pub mod exposure;
pub mod factory;
pub mod frame;
pub mod light;
#[cfg(target_os = "linux")]
mod linux;
#[cfg(windows)]
mod mf;
#[cfg(target_os = "linux")]
mod v4l2;
#[cfg(windows)]
mod win;

// The platform layer of `device`: device discovery and the capture of the
// depth node.
#[cfg(target_os = "linux")]
use linux as platform;
#[cfg(windows)]
use win as platform;

use std::time::Duration;

use serde::{Deserialize, Serialize};

pub use calib::{Calibration, CalibrationSource};
#[cfg(any(target_os = "linux", windows))]
pub use device::RevopointUsbDevice;
pub use exposure::ExposureProfile;
pub use factory::FactoryFile;
pub use frame::StreamMode;
pub use light::FillLight;

#[cfg(not(any(target_os = "linux", windows)))]
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
    /// the driver refuses it. The default, `Depth`, moves a third of the
    /// bytes of `Full`; with [`UsbOptions::keep_ir`] a `Depth` mode is
    /// raised to `Full`, the only one known to carry the infrared pair on
    /// every unit.
    pub mode: StreamMode,
    /// Frame rate asked for (the device offers 5, 10 and 15).
    pub fps: u32,
    /// Driver buffers (V4L2; Media Foundation manages its own).
    pub buffers: u32,
    /// Longest wait for the device to come back after a stop.
    pub reconnect_timeout: Duration,
    /// Keep the infrared pair of the latest frame
    /// ([`RevopointUsbDevice::latest_ir`]).
    pub keep_ir: bool,
    /// How long before its arrival a frame shows the scene: 0.7 to 1.1 s
    /// measured on a MINI 2 (2026-11-18), see [`crate::ScannerDevice::latency`].
    pub latency: Duration,
}

impl UsbOptions {
    /// The mode actually negotiated first.
    pub fn stream_mode(&self) -> StreamMode {
        if self.keep_ir && self.mode == StreamMode::Depth {
            StreamMode::Full
        } else {
            self.mode
        }
    }
}

impl Default for UsbOptions {
    fn default() -> Self {
        Self {
            mode: StreamMode::Depth,
            fps: 15,
            buffers: 4,
            reconnect_timeout: Duration::from_secs(8),
            keep_ir: false,
            latency: Duration::from_millis(800),
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

#[cfg(not(any(target_os = "linux", windows)))]
impl ScannerBackend for RevopointUsbBackend {
    fn id(&self) -> &'static str {
        BACKEND_ID
    }

    fn availability(&self) -> Result<()> {
        Err(crate::ScanError::PlatformUnsupported(format!(
            "{BACKEND_ID} uses V4L2 (Linux) or Media Foundation (Windows); on this system use \
             revopoint-sdk"
        )))
    }

    fn list(&self) -> Result<Vec<DeviceInfo>> {
        Ok(Vec::new())
    }

    fn open(&self, _device_id: &str) -> Result<Box<dyn ScannerDevice>> {
        Err(self.availability().unwrap_err())
    }
}
