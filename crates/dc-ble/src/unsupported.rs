//! Platforms without a Bluetooth LE backend: every call reports `Unsupported`.

use std::time::Duration;

use crate::{BleDeviceInfo, BleError, BleLink, SerialProfile};

pub(crate) fn availability() -> Result<(), BleError> {
    Err(BleError::Unsupported)
}

pub(crate) fn scan(
    _timeout: Duration,
    _name_filter: &dyn Fn(&str) -> bool,
) -> Result<Vec<BleDeviceInfo>, BleError> {
    Err(BleError::Unsupported)
}

pub(crate) fn connect_serial(
    _device_id: &str,
    _profile: &SerialProfile,
    _timeout: Duration,
) -> Result<Box<dyn BleLink>, BleError> {
    Err(BleError::Unsupported)
}
