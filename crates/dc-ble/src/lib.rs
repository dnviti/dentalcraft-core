//! Blocking Bluetooth LE serial links.
//!
//! A deliberately small surface: find nearby devices by name, connect to one,
//! and exchange bytes over a GATT "serial" service (one writable
//! characteristic, one notifying characteristic), the way HM-10 style BLE
//! modules and the Revopoint turntable do. No async runtime is exposed.
//!
//! Backends (all pure Rust, no C toolchain or system library at build time):
//! - Linux: BlueZ over D-Bus, through `zbus`'s blocking API.
//! - Windows: WinRT (`BluetoothLEDevice`, GATT) through the `windows` crate.
//! - Other platforms: every call fails with [`BleError::Unsupported`].
//!
//! Alpha: the Linux and Windows code paths could not be exercised against real
//! hardware yet; the protocol logic on top ([`chunks`], profiles) is tested
//! with fakes.

use std::time::Duration;

use serde::{Deserialize, Serialize};
use uuid::Uuid;

#[cfg(target_os = "linux")]
mod linux;
#[cfg(target_os = "linux")]
use linux as platform;

#[cfg(windows)]
mod winrt;
#[cfg(windows)]
use winrt as platform;

#[cfg(not(any(target_os = "linux", windows)))]
mod unsupported;
#[cfg(not(any(target_os = "linux", windows)))]
use unsupported as platform;

/// A Bluetooth LE device seen during a scan.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct BleDeviceInfo {
    /// Stable identifier to pass to [`connect_serial`]: the Bluetooth address
    /// (`AA:BB:CC:DD:EE:FF`) on both Linux and Windows.
    pub id: String,
    /// Advertised local name, when known.
    pub name: Option<String>,
    /// Signal strength in dBm, when known.
    pub rssi: Option<i16>,
}

/// Why a Bluetooth operation failed.
#[derive(Debug, thiserror::Error)]
pub enum BleError {
    /// No Bluetooth adapter (or no Bluetooth service) on this machine.
    #[error("no Bluetooth adapter available")]
    NoAdapter,
    /// An adapter exists but is powered off or blocked.
    #[error("the Bluetooth adapter is turned off")]
    AdapterOff,
    /// Bluetooth LE is not implemented on this platform.
    #[error("Bluetooth LE is not supported on this platform")]
    Unsupported,
    /// The device, or the requested GATT service or characteristic, was not found.
    #[error("Bluetooth device or service not found")]
    NotFound,
    /// The operation did not finish in time.
    #[error("Bluetooth operation timed out")]
    Timeout,
    /// The link to the device was lost.
    #[error("Bluetooth device disconnected")]
    Disconnected,
    /// Any other failure reported by the operating system's Bluetooth stack.
    #[error("Bluetooth error: {0}")]
    Platform(String),
}

impl BleError {
    /// Stable machine-readable code (surfaced in command errors and the UI).
    pub fn code(&self) -> &'static str {
        match self {
            Self::NoAdapter => "ble-no-adapter",
            Self::AdapterOff => "ble-adapter-off",
            Self::Unsupported => "ble-unsupported",
            Self::NotFound => "ble-not-found",
            Self::Timeout => "ble-timeout",
            Self::Disconnected => "ble-disconnected",
            Self::Platform(_) => "ble-platform",
        }
    }
}

/// An open BLE serial connection.
pub trait BleLink: Send {
    /// Write bytes to the device, split into chunks of at most
    /// [`SerialProfile::max_chunk`] bytes (write without response).
    fn write(&mut self, data: &[u8]) -> Result<(), BleError>;
    /// Wait up to `timeout` for the next notification payload.
    ///
    /// `Ok(None)` means nothing arrived in time. A lost link is reported as
    /// [`BleError::Disconnected`] (and keeps being reported afterwards).
    fn recv(&mut self, timeout: Duration) -> Result<Option<Vec<u8>>, BleError>;
    /// Stop notifications and disconnect. Idempotent; also done on drop.
    fn close(&mut self);
}

/// The GATT layout of a serial-over-BLE service.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SerialProfile {
    /// GATT service UUID.
    pub service: Uuid,
    /// Characteristic written to send data.
    pub write_char: Uuid,
    /// Characteristic that notifies received data (may equal `write_char`).
    pub notify_char: Uuid,
    /// Largest single write, in bytes (the ATT payload limit; 20 by default).
    pub max_chunk: usize,
}

impl SerialProfile {
    /// The ubiquitous HM-10 / "BLE UART" layout: service `FFE0`, one
    /// characteristic `FFE1` for both write and notify, 20-byte chunks.
    pub fn hm10() -> Self {
        Self {
            service: uuid16(0xFFE0),
            write_char: uuid16(0xFFE1),
            notify_char: uuid16(0xFFE1),
            max_chunk: 20,
        }
    }
}

/// The 128-bit UUID of a Bluetooth SIG 16-bit short UUID
/// (`0000xxxx-0000-1000-8000-00805f9b34fb`).
pub const fn uuid16(short: u16) -> Uuid {
    const BASE: u128 = 0x0000_0000_0000_1000_8000_0080_5f9b_34fb;
    Uuid::from_u128(((short as u128) << 96) | BASE)
}

/// Split `data` into writes of at most `max_chunk` bytes (a zero limit is
/// treated as 1). Empty input yields no chunks.
pub fn chunks(data: &[u8], max_chunk: usize) -> impl Iterator<Item = &[u8]> {
    data.chunks(max_chunk.max(1))
}

/// Succeeds when a powered Bluetooth adapter is present.
pub fn availability() -> Result<(), BleError> {
    platform::availability()
}

/// Scan for advertising devices for `timeout` and return those whose
/// advertised name satisfies `name_filter` (devices without a name are never
/// passed to the filter nor returned).
pub fn scan(
    timeout: Duration,
    name_filter: &dyn Fn(&str) -> bool,
) -> Result<Vec<BleDeviceInfo>, BleError> {
    platform::scan(timeout, name_filter)
}

/// Connect to `device_id` (a [`BleDeviceInfo::id`]), locate the profile's
/// service and characteristics, enable notifications and return the link.
/// `timeout` bounds the whole connection setup. An id listed by an earlier
/// [`scan`] stays valid: a device the system no longer knows (BlueZ forgets
/// unpaired devices about 30 s after they disconnect) is discovered again.
pub fn connect_serial(
    device_id: &str,
    profile: &SerialProfile,
    timeout: Duration,
) -> Result<Box<dyn BleLink>, BleError> {
    platform::connect_serial(device_id, profile, timeout)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn uuid16_expands_to_the_base_uuid() {
        let expected = Uuid::parse_str("0000ffe0-0000-1000-8000-00805f9b34fb").unwrap();
        assert_eq!(uuid16(0xFFE0), expected);
        assert_eq!(
            uuid16(0xFFE1).to_string(),
            "0000ffe1-0000-1000-8000-00805f9b34fb"
        );
    }

    #[test]
    fn hm10_profile_is_ffe0_ffe1_with_20_byte_chunks() {
        let p = SerialProfile::hm10();
        assert_eq!(
            p.service,
            Uuid::parse_str("0000ffe0-0000-1000-8000-00805f9b34fb").unwrap()
        );
        assert_eq!(
            p.write_char,
            Uuid::parse_str("0000ffe1-0000-1000-8000-00805f9b34fb").unwrap()
        );
        assert_eq!(p.write_char, p.notify_char);
        assert_eq!(p.max_chunk, 20);
    }

    #[test]
    fn chunks_split_at_the_limit_and_preserve_order() {
        let data: Vec<u8> = (0..45).collect();
        let parts: Vec<&[u8]> = chunks(&data, 20).collect();
        assert_eq!(parts.len(), 3);
        assert_eq!(parts[0].len(), 20);
        assert_eq!(parts[1].len(), 20);
        assert_eq!(parts[2].len(), 5);
        assert_eq!(parts.concat(), data);
    }

    #[test]
    fn chunks_edge_cases() {
        assert_eq!(chunks(b"", 20).count(), 0);
        assert_eq!(chunks(&[1u8; 20], 20).count(), 1);
        assert_eq!(chunks(&[1u8; 21], 20).count(), 2);
        // A zero limit must not loop forever or panic.
        assert_eq!(chunks(b"abc", 0).count(), 3);
        // A typical Revopoint command is longer than one ATT payload.
        let cmd = b"+CT,TURNANGLE=-360.000000;";
        assert_eq!(chunks(cmd, 20).count(), 2);
    }

    #[test]
    fn error_codes_are_stable() {
        assert_eq!(BleError::NoAdapter.code(), "ble-no-adapter");
        assert_eq!(BleError::AdapterOff.code(), "ble-adapter-off");
        assert_eq!(BleError::Unsupported.code(), "ble-unsupported");
        assert_eq!(BleError::NotFound.code(), "ble-not-found");
        assert_eq!(BleError::Timeout.code(), "ble-timeout");
        assert_eq!(BleError::Disconnected.code(), "ble-disconnected");
        assert_eq!(BleError::Platform("x".into()).code(), "ble-platform");
    }

    #[test]
    fn device_info_round_trips_through_json() {
        let d = BleDeviceInfo {
            id: "AA:BB:CC:DD:EE:FF".into(),
            name: Some("REVO_DUAL_AXIS_TABLE".into()),
            rssi: Some(-61),
        };
        let json = serde_json::to_string(&d).unwrap();
        assert_eq!(serde_json::from_str::<BleDeviceInfo>(&json).unwrap(), d);
    }

    /// On a machine without a Bluetooth adapter (the CI runners, the dev PC)
    /// the probe must fail cleanly with `NoAdapter`; a missing or
    /// non-activatable `bluetoothd` and a missing system bus map to it too. On
    /// a machine that does have an adapter the call may succeed or report it
    /// off. Other platforms report `Unsupported`.
    #[test]
    fn availability_maps_a_missing_stack_to_no_adapter() {
        match availability() {
            Ok(()) | Err(BleError::AdapterOff | BleError::NoAdapter | BleError::Unsupported) => {}
            Err(e) => panic!("unexpected availability error: {e}"),
        }
        if !std::path::Path::new("/sys/class/bluetooth").exists() && cfg!(target_os = "linux") {
            assert!(matches!(availability(), Err(BleError::NoAdapter)));
        }
    }

    /// Scanning without an adapter fails the same way and never hangs.
    #[test]
    fn scan_and_connect_without_adapter_fail_cleanly() {
        if std::path::Path::new("/sys/class/bluetooth").exists() || !cfg!(target_os = "linux") {
            return;
        }
        let e = scan(Duration::from_millis(100), &|_| true).unwrap_err();
        assert_eq!(e.code(), "ble-no-adapter");
        let e = connect_serial(
            "AA:BB:CC:DD:EE:FF",
            &SerialProfile::hm10(),
            Duration::from_secs(1),
        )
        .err()
        .unwrap();
        assert_eq!(e.code(), "ble-no-adapter");
    }
}
