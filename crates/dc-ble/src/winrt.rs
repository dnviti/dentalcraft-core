//! Windows backend: WinRT Bluetooth LE (`windows` crate), blocking through
//! `IAsyncOperation::join`.
//!
//! Every public entry point runs its WinRT calls on a short-lived worker
//! thread, so that (a) the caller's COM apartment (egui/winit threads are
//! STA) is never involved and (b) a hung WinRT call can be abandoned after a
//! timeout instead of blocking the caller forever.
//!
//! Alpha: written against the `windows` 0.62 sources and never compiled or run
//! on a Windows machine yet.

use std::collections::HashMap;
use std::sync::mpsc::{self, RecvTimeoutError};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

use uuid::Uuid;
use windows::Devices::Bluetooth::Advertisement::{
    BluetoothLEAdvertisementReceivedEventArgs, BluetoothLEAdvertisementWatcher,
    BluetoothLEAdvertisementWatcherStatus, BluetoothLEScanningMode,
};
use windows::Devices::Bluetooth::GenericAttributeProfile::{
    GattCharacteristic, GattCharacteristicProperties,
    GattClientCharacteristicConfigurationDescriptorValue, GattCommunicationStatus,
    GattDeviceService, GattSession, GattValueChangedEventArgs, GattWriteOption,
};
use windows::Devices::Bluetooth::{
    BluetoothCacheMode, BluetoothConnectionStatus, BluetoothLEDevice,
};
use windows::Devices::Radios::{Radio, RadioKind, RadioState};
use windows::Foundation::TypedEventHandler;
use windows::Storage::Streams::{DataReader, DataWriter};
use windows::core::{GUID, IInspectable, Ref};

use crate::{BleDeviceInfo, BleError, BleLink, LinkState, SerialProfile, chunks};

/// HRESULT_FROM_WIN32(ERROR_DEVICE_NOT_AVAILABLE): the radio is off.
const E_RADIO_UNAVAILABLE: i32 = 0x8007_10DFu32 as i32;
/// E_POINTER: WinRT returned a null object (device not found).
const E_POINTER: i32 = 0x8000_4003u32 as i32;
/// How long to wait for a worker beyond its own deadline.
const WORKER_SLACK: Duration = Duration::from_secs(5);

fn map_err(e: windows::core::Error) -> BleError {
    match e.code().0 {
        E_RADIO_UNAVAILABLE => BleError::AdapterOff,
        E_POINTER => BleError::NotFound,
        _ => BleError::Platform(format!("{} ({:#010x})", e.message(), e.code().0)),
    }
}

fn map_status(status: GattCommunicationStatus, what: &str) -> Result<(), BleError> {
    match status {
        GattCommunicationStatus::Success => Ok(()),
        GattCommunicationStatus::Unreachable => Err(BleError::Disconnected),
        GattCommunicationStatus::ProtocolError => {
            Err(BleError::Platform(format!("{what}: GATT protocol error")))
        }
        GattCommunicationStatus::AccessDenied => {
            Err(BleError::Platform(format!("{what}: access denied")))
        }
        other => Err(BleError::Platform(format!(
            "{what}: GATT status {}",
            other.0
        ))),
    }
}

/// Run `f` on a fresh thread and wait at most `timeout` for its result. A
/// worker that overruns is abandoned: it finishes in the background and its
/// result (e.g. a late link) is dropped, which closes it.
fn on_worker<T: Send + 'static>(
    timeout: Duration,
    f: impl FnOnce() -> Result<T, BleError> + Send + 'static,
) -> Result<T, BleError> {
    let (tx, rx) = mpsc::channel();
    thread::Builder::new()
        .name("dc-ble-winrt".into())
        .spawn(move || {
            let _ = tx.send(f());
        })
        .map_err(|e| BleError::Platform(format!("cannot start the BLE worker thread: {e}")))?;
    match rx.recv_timeout(timeout) {
        Ok(result) => result,
        Err(RecvTimeoutError::Timeout) => Err(BleError::Timeout),
        Err(RecvTimeoutError::Disconnected) => {
            Err(BleError::Platform("the BLE worker thread died".into()))
        }
    }
}

fn guid(u: Uuid) -> GUID {
    GUID::from_u128(u.as_u128())
}

/// `AA:BB:CC:DD:EE:FF` for the 48-bit WinRT address.
fn format_address(address: u64) -> String {
    let b = address.to_be_bytes();
    format!(
        "{:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}",
        b[2], b[3], b[4], b[5], b[6], b[7]
    )
}

fn parse_address(id: &str) -> Option<u64> {
    let digits: String = id.chars().filter(|c| *c != ':' && *c != '-').collect();
    if digits.len() != 12 {
        return None;
    }
    u64::from_str_radix(&digits, 16).ok()
}

// ----------------------------------------------------------------- probe

pub(crate) fn availability() -> Result<(), BleError> {
    on_worker(Duration::from_secs(15), || {
        let radios = match Radio::GetRadiosAsync().and_then(|op| op.join()) {
            Ok(radios) => radios,
            // Radio enumeration can be refused to some apps even though BLE
            // works; the real operations report their own failures.
            Err(_) => return Ok(()),
        };
        let mut found = false;
        for radio in &radios {
            if radio.Kind().map_err(map_err)? == RadioKind::Bluetooth {
                found = true;
                if radio.State().map_err(map_err)? == RadioState::On {
                    return Ok(());
                }
            }
        }
        if found {
            Err(BleError::AdapterOff)
        } else {
            Err(BleError::NoAdapter)
        }
    })
}

// ------------------------------------------------------------------ scan

struct Seen {
    name: Option<String>,
    rssi: i16,
}

pub(crate) fn scan(
    timeout: Duration,
    name_filter: &dyn Fn(&str) -> bool,
) -> Result<Vec<BleDeviceInfo>, BleError> {
    availability()?;
    let seen: Arc<Mutex<HashMap<u64, Seen>>> = Arc::default();
    let collected = {
        let seen = seen.clone();
        on_worker(timeout + WORKER_SLACK, move || {
            let watcher = BluetoothLEAdvertisementWatcher::new().map_err(map_err)?;
            watcher
                .SetScanningMode(BluetoothLEScanningMode::Active)
                .map_err(map_err)?;
            let sink = seen.clone();
            let token = watcher
                .Received(&TypedEventHandler::<
                    BluetoothLEAdvertisementWatcher,
                    BluetoothLEAdvertisementReceivedEventArgs,
                >::new(
                    move |_, args: Ref<BluetoothLEAdvertisementReceivedEventArgs>| {
                        let Some(args) = args.as_ref() else {
                            return Ok(());
                        };
                        let address = args.BluetoothAddress()?;
                        let rssi = args.RawSignalStrengthInDBm()?;
                        let name = args
                            .Advertisement()
                            .and_then(|a| a.LocalName())
                            .map(|n| n.to_string())
                            .ok()
                            .filter(|n| !n.is_empty());
                        if let Ok(mut map) = sink.lock() {
                            let entry = map.entry(address).or_insert(Seen { name: None, rssi });
                            entry.rssi = rssi;
                            // The name often only comes with the scan response.
                            if name.is_some() {
                                entry.name = name;
                            }
                        }
                        Ok(())
                    },
                ))
                .map_err(map_err)?;
            watcher.Start().map_err(map_err)?;

            let deadline = Instant::now() + timeout;
            let mut aborted = false;
            while Instant::now() < deadline {
                thread::sleep(Duration::from_millis(100).min(deadline - Instant::now()));
                if watcher.Status().map_err(map_err)?
                    == BluetoothLEAdvertisementWatcherStatus::Aborted
                {
                    aborted = true;
                    break;
                }
            }
            let _ = watcher.Stop();
            let _ = watcher.RemoveReceived(token);
            if aborted {
                // Usually the radio was switched off while scanning.
                return Err(BleError::Platform(
                    "the Bluetooth scan was aborted by the system".into(),
                ));
            }
            Ok(())
        })
    };
    collected?;

    let map = seen
        .lock()
        .map_err(|_| BleError::Platform("scan results poisoned".into()))?;
    let mut devices: Vec<BleDeviceInfo> = map
        .iter()
        .filter_map(|(address, s)| {
            let name = s.name.as_deref()?;
            name_filter(name).then(|| BleDeviceInfo {
                id: format_address(*address),
                name: Some(name.to_owned()),
                rssi: Some(s.rssi),
            })
        })
        .collect();
    devices.sort_by(|a, b| {
        b.rssi
            .cmp(&a.rssi)
            .then_with(|| a.name.cmp(&b.name))
            .then_with(|| a.id.cmp(&b.id))
    });
    Ok(devices)
}

// --------------------------------------------------------------- connect

enum Event {
    Data(Vec<u8>),
    Lost,
}

struct WinLink {
    device: BluetoothLEDevice,
    session: Option<GattSession>,
    service: GattDeviceService,
    write_char: GattCharacteristic,
    notify_char: GattCharacteristic,
    value_token: i64,
    status_token: i64,
    write_option: GattWriteOption,
    max_chunk: usize,
    rx: mpsc::Receiver<Event>,
    state: LinkState,
    lost: bool,
    closed: bool,
}

impl BleLink for WinLink {
    fn write(&mut self, data: &[u8]) -> Result<(), BleError> {
        if self.closed || self.lost {
            return Err(BleError::Disconnected);
        }
        for chunk in chunks(data, self.max_chunk) {
            let writer = DataWriter::new().map_err(map_err)?;
            writer.WriteBytes(chunk).map_err(map_err)?;
            let buffer = writer.DetachBuffer().map_err(map_err)?;
            let status = self
                .write_char
                .WriteValueWithOptionAsync(&buffer, self.write_option)
                .and_then(|op| op.join())
                .map_err(map_err)?;
            if let Err(e) = map_status(status, "write") {
                if matches!(e, BleError::Disconnected) {
                    self.lost = true;
                    self.state.mark_lost();
                }
                return Err(e);
            }
        }
        Ok(())
    }

    fn recv(&mut self, timeout: Duration) -> Result<Option<Vec<u8>>, BleError> {
        if self.closed || self.lost {
            return Err(BleError::Disconnected);
        }
        match self.rx.recv_timeout(timeout) {
            Ok(Event::Data(bytes)) => Ok(Some(bytes)),
            Ok(Event::Lost) | Err(RecvTimeoutError::Disconnected) => {
                self.lost = true;
                self.state.mark_lost();
                Err(BleError::Disconnected)
            }
            Err(RecvTimeoutError::Timeout) => Ok(None),
        }
    }

    fn close(&mut self) {
        if self.closed {
            return;
        }
        self.closed = true;
        self.state.mark_lost();
        let _ = self.notify_char.RemoveValueChanged(self.value_token);
        let _ = self.device.RemoveConnectionStatusChanged(self.status_token);
        let _ = self
            .notify_char
            .WriteClientCharacteristicConfigurationDescriptorAsync(
                GattClientCharacteristicConfigurationDescriptorValue::None,
            )
            .and_then(|op| op.join());
        if let Some(session) = &self.session {
            let _ = session.Close();
        }
        // Releasing every reference to the device disconnects it.
        let _ = self.service.Close();
        let _ = self.device.Close();
    }

    fn state(&self) -> LinkState {
        self.state.clone()
    }
}

impl Drop for WinLink {
    fn drop(&mut self) {
        self.close();
    }
}

fn first_service(device: &BluetoothLEDevice, service: Uuid) -> Result<GattDeviceService, BleError> {
    let result = device
        .GetGattServicesForUuidWithCacheModeAsync(guid(service), BluetoothCacheMode::Uncached)
        .and_then(|op| op.join())
        .map_err(map_err)?;
    match result.Status().map_err(map_err)? {
        GattCommunicationStatus::Unreachable => return Err(BleError::NotFound),
        status => map_status(status, "service discovery")?,
    }
    result
        .Services()
        .map_err(map_err)?
        .into_iter()
        .next()
        .ok_or_else(|| BleError::Platform("GATT serial service not found on the device".into()))
}

fn find_characteristic(
    service: &GattDeviceService,
    uuid: Uuid,
    what: &str,
) -> Result<GattCharacteristic, BleError> {
    let result = service
        .GetCharacteristicsForUuidWithCacheModeAsync(guid(uuid), BluetoothCacheMode::Uncached)
        .and_then(|op| op.join())
        .map_err(map_err)?;
    map_status(result.Status().map_err(map_err)?, what)?;
    result
        .Characteristics()
        .map_err(map_err)?
        .into_iter()
        .next()
        .ok_or_else(|| BleError::Platform(format!("GATT {what} not found on the device")))
}

fn has(properties: GattCharacteristicProperties, flag: GattCharacteristicProperties) -> bool {
    properties.0 & flag.0 != 0
}

fn connect_blocking(
    device_id: &str,
    profile: &SerialProfile,
) -> Result<Box<dyn BleLink>, BleError> {
    let address = parse_address(device_id).ok_or(BleError::NotFound)?;
    let device = BluetoothLEDevice::FromBluetoothAddressAsync(address)
        .and_then(|op| op.join())
        .map_err(map_err)?;

    let service = first_service(&device, profile.service)?;
    let write_char = find_characteristic(&service, profile.write_char, "write characteristic")?;
    let notify_char = find_characteristic(&service, profile.notify_char, "notify characteristic")?;

    let write_option = if has(
        write_char.CharacteristicProperties().map_err(map_err)?,
        GattCharacteristicProperties::WriteWithoutResponse,
    ) {
        GattWriteOption::WriteWithoutResponse
    } else {
        GattWriteOption::WriteWithResponse
    };
    let cccd = if has(
        notify_char.CharacteristicProperties().map_err(map_err)?,
        GattCharacteristicProperties::Notify,
    ) {
        GattClientCharacteristicConfigurationDescriptorValue::Notify
    } else {
        GattClientCharacteristicConfigurationDescriptorValue::Indicate
    };

    // Keep the link up between GATT operations (best effort).
    let session = device
        .BluetoothDeviceId()
        .and_then(|id| GattSession::FromDeviceIdAsync(&id))
        .and_then(|op| op.join())
        .ok();
    if let Some(session) = &session {
        let _ = session.SetMaintainConnection(true);
    }

    let (tx, rx) = mpsc::channel();
    let value_tx = tx.clone();
    let state = LinkState::new();
    let status_state = state.clone();
    let value_token = notify_char
        .ValueChanged(&TypedEventHandler::<
            GattCharacteristic,
            GattValueChangedEventArgs,
        >::new(
            move |_, args: Ref<GattValueChangedEventArgs>| {
                let Some(args) = args.as_ref() else {
                    return Ok(());
                };
                let buffer = args.CharacteristicValue()?;
                let mut bytes = vec![0u8; buffer.Length()? as usize];
                DataReader::FromBuffer(&buffer)?.ReadBytes(&mut bytes)?;
                let _ = value_tx.send(Event::Data(bytes));
                Ok(())
            },
        ))
        .map_err(map_err)?;
    let status_token = device
        .ConnectionStatusChanged(&TypedEventHandler::<BluetoothLEDevice, IInspectable>::new(
            move |sender: Ref<BluetoothLEDevice>, _| {
                if let Some(device) = sender.as_ref()
                    && device.ConnectionStatus()? == BluetoothConnectionStatus::Disconnected
                {
                    status_state.mark_lost();
                    let _ = tx.send(Event::Lost);
                }
                Ok(())
            },
        ))
        .map_err(map_err)?;

    // From here on dropping the link undoes everything done so far.
    let link = WinLink {
        device,
        session,
        service,
        write_char,
        notify_char,
        value_token,
        status_token,
        write_option,
        max_chunk: profile.max_chunk,
        rx,
        state,
        lost: false,
        closed: false,
    };
    let status = link
        .notify_char
        .WriteClientCharacteristicConfigurationDescriptorAsync(cccd)
        .and_then(|op| op.join())
        .map_err(map_err)?;
    map_status(status, "enabling notifications")?;
    Ok(Box::new(link))
}

pub(crate) fn connect_serial(
    device_id: &str,
    profile: &SerialProfile,
    timeout: Duration,
) -> Result<Box<dyn BleLink>, BleError> {
    availability()?;
    let id = device_id.to_owned();
    let profile = *profile;
    on_worker(timeout, move || connect_blocking(&id, &profile))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn address_round_trip() {
        let a = parse_address("aa:bb:cc:dd:ee:0f").unwrap();
        assert_eq!(a, 0xAABB_CCDD_EE0F);
        assert_eq!(format_address(a), "AA:BB:CC:DD:EE:0F");
        assert!(parse_address("nonsense").is_none());
    }
}
