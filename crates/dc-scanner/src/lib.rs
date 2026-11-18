//! 3D scanner acquisition for DentalCraft (alpha).
//!
//! The crate is headless and GPU-free. It covers the whole in-app scanning
//! workflow up to the mesh handed to the case import:
//!
//! 1. **Devices.** A [`ScannerBackend`] enumerates and opens devices of one
//!    kind; a [`ScannerDevice`] streams [`DepthFrame`]s (depth computed on the
//!    device, millimetres, pinhole intrinsics). [`backends`] is the registry;
//!    the simulated backend [`sim`] is always present.
//! 2. **Acquisition.** [`ScanSession`] owns the device and an acquisition
//!    thread that tracks every frame against the fused model (projective
//!    point-to-plane ICP, see [`tracking`]) and fuses it into a sparse TSDF
//!    volume ([`dc_geom::tsdf::TsdfVolume`]). The UI polls
//!    [`ScanSession::status`] and [`ScanSession::preview`]; no call blocks on
//!    the device.
//! 3. **Editing.** [`ScanEditor`] holds the fused cloud and the mesh built
//!    from it, applies [`CloudOp`]s (cleanup, selection deletes, smoothing)
//!    and [`MeshOp`]s (hole filling, smoothing, island removal, decimation)
//!    with undo / redo.
//! 4. **Turntable rig.** [`rig`] drives a motorised turntable
//!    ([`dc_turntable`]) in front of a fixed scanner: tool-free axis
//!    calibration, automatic multi-tilt scans (`scan.auto`), region of
//!    interest around the platter and turntable motion prior for the
//!    tracking. The `turntable.*` commands live on the same
//!    [`ScanController`].
//!
//! Units are millimetres everywhere. The world frame of a scan is the camera
//! frame of its first fused frame (x right, y down, z forward).
//!
//! # Adding a hardware backend
//!
//! A real scanner (e.g. Revopoint MINI 2) plugs in by implementing the two
//! traits and registering the backend in [`backends`]:
//!
//! * [`ScannerBackend::id`] is a short stable id (`revopoint`), used by the
//!   CLI / MCP commands and stored in [`DeviceInfo::backend`].
//! * [`ScannerBackend::list`] enumerates the connected devices (USB / Wi-Fi
//!   discovery). It must be quick and must not open the devices.
//! * [`ScannerBackend::open`] opens one device by the [`DeviceInfo::id`]
//!   returned by `list` and returns it boxed as a [`ScannerDevice`].
//! * [`ScannerDevice::start`] / [`ScannerDevice::stop`] start and stop the
//!   depth stream; [`ScannerDevice::next_frame`] waits at most `timeout` for
//!   the next frame and returns `Ok(None)` when none arrived in time. It runs
//!   on the acquisition thread, so it may block up to `timeout` but no longer
//!   (the session checks its stop flag between calls).
//! * Frames carry the depth along the optical axis in millimetres, row-major,
//!   `0` or `NaN` for invalid pixels, with the intrinsics of the depth image
//!   (undistorted). Optional per-pixel colour must be registered to the depth
//!   image. Timestamps are device time in microseconds.
//! * Errors are reported as [`ScanError::Device`] with a technical detail;
//!   the app shows the localised message of [`ScanError::code`].
//!
//! The backend must be pure Rust (no C/C++ SDK at build time); vendor
//! protocols are spoken over `std` sockets or a pure-Rust USB / V4L2 stack
//! ([`revopoint_usb`]), or a vendor SDK installed by the user is loaded at
//! run time ([`revopoint_sdk`]).

pub mod control;
pub mod editor;
pub mod revopoint_sdk;
pub mod revopoint_usb;
pub mod rig;
pub mod session;
pub mod sim;
pub mod tracking;

use std::time::Duration;

use serde::{Deserialize, Serialize};

pub use control::{
    SCAN_IMPORT, ScanCommandInfo, ScanController, command_infos, import_role, is_scan_command,
};
pub use dc_geom::tsdf::CameraIntrinsics;
pub use dc_mesh::PointCloud;
pub use dc_turntable::{AxisInfo, AxisKind, TurntableError, TurntableInfo};
pub use editor::{CloudOp, MeshOp, ScanEditor};
pub use rig::{RigCalibration, RigJobKind, RigJobState, RigJobStatus, TurntableStatus};
pub use session::{LivePreview, ScanSession, ScanSettings, ScanState, ScanStatus};

pub use revopoint_usb::RevopointUsbBackend;

/// Errors of the scanner crate. Every variant has a stable kebab-case
/// [`ScanError::code`] that the app maps to a localised message.
#[derive(Debug, thiserror::Error)]
pub enum ScanError {
    /// No backend with this id is registered.
    #[error("unknown scanner backend `{0}`")]
    UnknownBackend(String),
    /// The backend has no device with this id.
    #[error("scanner device `{0}` not found")]
    DeviceNotFound(String),
    /// The device failed (disconnected, protocol error, I/O).
    #[error("scanner device error: {0}")]
    Device(String),
    /// The action needs a connected device.
    #[error("no scanner connected")]
    NotConnected,
    /// The action is not allowed in the current acquisition state.
    #[error("cannot {action} while the scan is {state:?}")]
    InvalidState {
        /// The refused action (`start`, `pause`...).
        action: &'static str,
        /// The current state.
        state: ScanState,
    },
    /// A parameter is out of range.
    #[error("invalid parameter: {0}")]
    InvalidParams(String),
    /// There is nothing to work on (empty cloud, no mesh yet).
    #[error("nothing to process: {0}")]
    Empty(&'static str),
    /// The computation was cancelled by the caller.
    #[error("cancelled")]
    Cancelled,
    /// A geometry kernel failed.
    #[error(transparent)]
    Geom(dc_geom::GeomError),
    /// A mesh operation failed (e.g. loading the simulated scene).
    #[error(transparent)]
    Mesh(#[from] dc_mesh::MeshError),
    /// There is no mesh yet: build it from the cloud first.
    #[error("no mesh built yet")]
    NoMesh,
    /// The vendor SDK library a backend needs is not installed or unusable.
    #[error("scanner SDK missing: {0}")]
    SdkMissing(String),
    /// The vendor SDK refuses the device: it is not authorised for SDK use.
    #[error("scanner not authorised for the SDK: {0}")]
    SdkUnauthorized(String),
    /// The backend does not run on this operating system.
    #[error("scanner backend not supported on this platform: {0}")]
    PlatformUnsupported(String),
    /// The device left while streaming (unplugged, USB reset).
    #[error("scanner disconnected: {0}")]
    DeviceLost(String),
    /// The turntable failed or refused (its code, e.g.
    /// `turntable-not-connected`, `ble-no-adapter`).
    #[error(transparent)]
    Turntable(dc_turntable::TurntableError),
    /// The turntable calibration failed or its file is unusable.
    #[error("turntable calibration: {0}")]
    Calibration(String),
    /// `scan.auto` without a turntable calibration matching the scanner and
    /// the table (allowed with `allow_uncalibrated`).
    #[error("no turntable calibration for this scanner and turntable: calibrate first")]
    RigUncalibrated,
    /// `turntable.calibrate` measured an axis that fails its checks; `code`
    /// says which (`calibration-no-object`, `calibration-tracking`,
    /// `calibration-inaccurate`, `calibration-implausible`).
    #[error("turntable calibration refused ({code}): {detail}")]
    CalibrationRefused {
        /// Stable code of the failed check.
        code: &'static str,
        /// Technical detail with the measured values (untranslated).
        detail: String,
    },
    /// `scan.auto` ended with too few views fused or registered.
    #[error("the automatic scan is unusable: {0}")]
    ScanQuality(String),
}

impl From<dc_turntable::TurntableError> for ScanError {
    fn from(e: dc_turntable::TurntableError) -> Self {
        match e {
            dc_turntable::TurntableError::Cancelled => Self::Cancelled,
            dc_turntable::TurntableError::InvalidParams(m) => Self::InvalidParams(m),
            e => Self::Turntable(e),
        }
    }
}

impl From<dc_geom::GeomError> for ScanError {
    fn from(e: dc_geom::GeomError) -> Self {
        match e {
            dc_geom::GeomError::Cancelled => Self::Cancelled,
            e => Self::Geom(e),
        }
    }
}

impl ScanError {
    /// Stable identifier, the same in every language (kebab-case).
    pub fn code(&self) -> &'static str {
        match self {
            Self::UnknownBackend(_) => "unknown-backend",
            Self::DeviceNotFound(_) => "device-not-found",
            Self::Device(_) => "device",
            Self::NotConnected => "not-connected",
            Self::InvalidState { .. } => "invalid-state",
            Self::InvalidParams(_) => "invalid-params",
            Self::Empty(_) => "empty",
            Self::Cancelled => "cancelled",
            Self::Geom(_) => "geometry",
            Self::Mesh(_) => "mesh",
            Self::NoMesh => "no-mesh",
            Self::SdkMissing(_) => "sdk-missing",
            Self::SdkUnauthorized(_) => "sdk-unauthorized",
            Self::PlatformUnsupported(_) => "platform-unsupported",
            Self::DeviceLost(_) => "device-lost",
            Self::Turntable(e) => e.code(),
            Self::Calibration(_) => "calibration",
            Self::RigUncalibrated => "rig-uncalibrated",
            Self::CalibrationRefused { code, .. } => code,
            Self::ScanQuality(_) => "scan-quality-low",
        }
    }
}

/// Result alias of the scanner crate.
pub type Result<T, E = ScanError> = std::result::Result<T, E>;

/// One depth image streamed by a device.
#[derive(Debug, Clone, PartialEq)]
pub struct DepthFrame {
    /// Depth along the optical axis in millimetres, row-major
    /// (`width * height`); `0` or `NaN` marks an invalid pixel.
    pub depth_mm: Vec<f32>,
    /// Pinhole intrinsics of the depth image.
    pub intrinsics: CameraIntrinsics,
    /// Device timestamp in microseconds.
    pub timestamp_us: u64,
    /// Optional colour per depth pixel (same layout as `depth_mm`).
    pub color: Option<Vec<[u8; 3]>>,
    /// Orientation of the device IMU when the frame was taken, when the
    /// device reports it (frame of the IMU, not of the camera).
    pub imu_orientation: Option<glam::Quat>,
}

/// Identity of a device as listed by its backend.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct DeviceInfo {
    /// Id of the backend that lists the device (`sim`, ...).
    pub backend: String,
    /// Id of the device, unique within the backend.
    pub id: String,
    /// Display name reported by the device (not translated).
    pub name: String,
    /// Serial number, when the device has one.
    pub serial: Option<String>,
}

/// An opened device. See the crate doc for the contract of each method.
pub trait ScannerDevice: Send {
    /// Identity of the device.
    fn info(&self) -> DeviceInfo;
    /// Starts the depth stream.
    fn start(&mut self) -> Result<()>;
    /// Stops the depth stream (idempotent).
    fn stop(&mut self);
    /// Waits at most `timeout` for the next frame; `Ok(None)` when none came.
    fn next_frame(&mut self, timeout: Duration) -> Result<Option<DepthFrame>>;
    /// Sets the exposure time, when the device supports it.
    fn set_exposure_us(&mut self, _us: u32) -> Result<()> {
        Ok(())
    }
    /// Keeps the device's fill light steadily on while streaming (`true`)
    /// or leaves it alone (`false`), when the device has one it can drive.
    fn set_fill_light(&mut self, _on: bool) -> Result<()> {
        Ok(())
    }
    /// How long before its arrival a frame shows the scene, about: the
    /// turntable angle of a frame is read that long before it arrived.
    fn latency(&self) -> Duration {
        Duration::ZERO
    }
}

/// A kind of scanner: enumerates and opens its devices.
pub trait ScannerBackend: Send + Sync {
    /// Short stable id of the backend.
    fn id(&self) -> &'static str;
    /// Whether the backend can be used at all (e.g. its vendor SDK is
    /// installed); `scan.devices` reports the error code when it cannot.
    fn availability(&self) -> Result<()> {
        Ok(())
    }
    /// Devices currently available.
    fn list(&self) -> Result<Vec<DeviceInfo>>;
    /// Opens the device `device_id` (an id returned by [`ScannerBackend::list`]).
    fn open(&self, device_id: &str) -> Result<Box<dyn ScannerDevice>>;
}

/// The registered backends. The simulated backend `sim` is always present;
/// `revopoint-sdk` lists no device until the Revopoint SDK is installed;
/// `revopoint-usb` (Linux, Windows) lists the Revopoint scanners on USB.
pub fn backends() -> Vec<Box<dyn ScannerBackend>> {
    vec![
        Box::new(revopoint_sdk::RevopointSdkBackend::default()),
        Box::new(revopoint_usb::RevopointUsbBackend::default()),
        Box::new(sim::SimBackend::default()),
    ]
}

/// The registered backend with id `id`.
pub fn backend(id: &str) -> Result<Box<dyn ScannerBackend>> {
    backends()
        .into_iter()
        .find(|b| b.id() == id)
        .ok_or_else(|| ScanError::UnknownBackend(id.to_owned()))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn registry_has_sim_with_two_devices() {
        let b = backend("sim").unwrap();
        let ids: Vec<String> = b.list().unwrap().into_iter().map(|d| d.id).collect();
        assert_eq!(ids, ["sim-typodont-upper", "sim-typodont-lower"]);
        assert_eq!(backend("nope").err().unwrap().code(), "unknown-backend");
    }

    #[test]
    fn error_codes_are_kebab_case() {
        let errors = [
            ScanError::NotConnected,
            ScanError::InvalidState {
                action: "start",
                state: ScanState::Idle,
            },
            ScanError::InvalidParams("x".into()),
            ScanError::from(dc_geom::GeomError::Cancelled),
            ScanError::from(dc_geom::GeomError::InvalidInput("x".into())),
            ScanError::PlatformUnsupported("x".into()),
            ScanError::DeviceLost("x".into()),
        ];
        let codes: Vec<&str> = errors.iter().map(ScanError::code).collect();
        assert_eq!(
            codes,
            [
                "not-connected",
                "invalid-state",
                "invalid-params",
                "cancelled",
                "geometry",
                "platform-unsupported",
                "device-lost"
            ]
        );
    }

    #[test]
    fn registry_order() {
        let ids: Vec<&str> = backends().iter().map(|b| b.id()).collect();
        assert_eq!(ids, ["revopoint-sdk", "revopoint-usb", "sim"]);
        let usb = backend("revopoint-usb").unwrap();
        if cfg!(target_os = "linux") {
            usb.availability().unwrap();
        } else if cfg!(windows) {
            // Media Foundation may be missing (Windows N without the Media
            // Feature Pack)
            if let Err(e) = usb.availability() {
                assert_eq!(e.code(), "platform-unsupported");
            }
        } else {
            assert_eq!(
                usb.availability().unwrap_err().code(),
                "platform-unsupported"
            );
            assert!(usb.list().unwrap().is_empty());
        }
    }
}
