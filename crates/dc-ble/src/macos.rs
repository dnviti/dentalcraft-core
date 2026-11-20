//! CoreBluetooth backend: the glue between the objc2 bindings and the
//! platform-free logic in [`crate::macos_logic`].
//!
//! Every operation owns one `CBCentralManager` whose delegate runs on a
//! private serial dispatch queue and turns each callback into an [`Event`]
//! on an `mpsc` channel; the calling thread drives CoreBluetooth directly
//! and waits on that channel, the way the BlueZ backend waits on D-Bus.
//!
//! macOS kills a process that touches CoreBluetooth unless its bundle
//! explains why (`NSBluetoothAlwaysUsageDescription`), so every entry point
//! checks that first and fails with [`BleError::AppBundleRequired`].
//!
//! CoreBluetooth never reveals Bluetooth addresses: a device's id is the
//! peripheral identifier CoreBluetooth gives it on this Mac.

// objc2 exposes every CoreBluetooth call as `unsafe`; this module is the only
// place of the crate that makes them.
#![allow(unsafe_code)]

use std::collections::VecDeque;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{self, Receiver, Sender};
use std::time::{Duration, Instant};

use dispatch2::{DispatchQueue, DispatchRetained};
use objc2::rc::Retained;
use objc2::runtime::{AnyObject, ProtocolObject};
use objc2::{AnyThread, DefinedClass, define_class, msg_send};
use objc2_core_bluetooth::{
    CBAdvertisementDataLocalNameKey, CBAttribute, CBCentralManager, CBCentralManagerDelegate,
    CBCentralManagerScanOptionAllowDuplicatesKey, CBCharacteristic, CBCharacteristicWriteType,
    CBManager, CBPeripheral, CBPeripheralDelegate, CBPeripheralState, CBService, CBUUID,
};
use objc2_foundation::{
    NSArray, NSBundle, NSData, NSDictionary, NSError, NSNumber, NSObject, NSObjectProtocol,
    NSString, NSUUID, ns_string,
};
use uuid::Uuid;

use crate::macos_logic::{
    Central, Event, Fail, Link, Policy, Wire, WriteMode, authorization_outcome, cb_uuid_string,
    collect_scan, connect_flow, parse_cb_uuid, parse_peripheral_id, rssi_dbm, state_drops_links,
    usage_described, wait_powered,
};
use crate::{BleDeviceInfo, BleError, BleLink, LinkState, SerialProfile};

/// Label of the dispatch queue the delegate callbacks run on.
const QUEUE_LABEL: &str = "app.dentalcraft.dc-ble";

// --------------------------------------------------------------- guards

/// Refuses, instead of letting macOS kill the process, when this program is
/// not inside a bundle whose `Info.plist` explains its use of Bluetooth (the
/// DentalCraft app bundle does, for both of its programs).
fn ensure_bundle() -> Result<(), BleError> {
    let text = NSBundle::mainBundle()
        .objectForInfoDictionaryKey(ns_string!("NSBluetoothAlwaysUsageDescription"))
        .and_then(|value| value.downcast::<NSString>().ok())
        .map(|value| value.to_string());
    if usage_described(text.as_deref()) {
        Ok(())
    } else {
        Err(BleError::AppBundleRequired)
    }
}

/// The checks before any manager exists. Returns the longest wait for the
/// adapter state: longer the first time, when macOS asks the user whether
/// DentalCraft may use Bluetooth.
fn preflight(policy: &Policy) -> Result<Duration, BleError> {
    ensure_bundle()?;
    // SAFETY: a class property without arguments (macOS 10.15+; the app
    // needs 11).
    let raw = unsafe { CBManager::authorization_class() }.0;
    Ok(if authorization_outcome(raw)? {
        policy.prompt
    } else {
        policy.probe
    })
}

// ------------------------------------------------------------- delegate

/// State shared by the delegate callbacks (run on the dispatch queue).
struct Ivars {
    tx: Sender<Event>,
    /// The state of the link being set up; marked lost the moment the link
    /// drops, before anyone reads the channel.
    state: LinkState,
    /// A connection to the peripheral is up (between `didConnect` and its
    /// loss): only then does an adapter state change drop a link.
    connected: AtomicBool,
    /// The notify characteristic whose values are forwarded.
    notify: Option<Uuid>,
}

impl Ivars {
    fn send(&self, event: Event) {
        // The receiver is gone once the operation ended: nothing to tell.
        let _ = self.tx.send(event);
    }

    fn lost(&self) {
        if self.connected.swap(false, Ordering::AcqRel) {
            self.state.mark_lost();
        }
    }
}

fn fail(error: Option<&NSError>) -> Option<Fail> {
    error.map(|e| Fail {
        domain: e.domain().to_string(),
        code: e.code(),
        text: e.localizedDescription().to_string(),
    })
}

/// The UUID of a service or characteristic.
fn uuid_of(attribute: &CBAttribute) -> Option<Uuid> {
    // SAFETY: property getters without arguments.
    let uuid = unsafe { attribute.UUID() };
    parse_cb_uuid(&unsafe { uuid.UUIDString() }.to_string())
}

fn cb_uuid(uuid: Uuid) -> Retained<CBUUID> {
    // SAFETY: CoreBluetooth accepts the 4-, 8- and 36-character forms that
    // `cb_uuid_string` produces.
    unsafe { CBUUID::UUIDWithString(&NSString::from_str(&cb_uuid_string(uuid))) }
}

define_class!(
    // SAFETY: NSObject has no subclassing requirements and `Delegate` does
    // not implement `Drop`.
    #[unsafe(super(NSObject))]
    #[name = "DentalCraftBleDelegate"]
    #[ivars = Ivars]
    struct Delegate;

    unsafe impl NSObjectProtocol for Delegate {}

    unsafe impl CBCentralManagerDelegate for Delegate {
        #[unsafe(method(centralManagerDidUpdateState:))]
        fn did_update_state(&self, central: &CBCentralManager) {
            // SAFETY: a property getter without arguments.
            let raw = unsafe { central.state() }.0;
            if state_drops_links(raw) {
                // Below PoweredOn every connection is gone, without a
                // didDisconnect callback.
                self.ivars().lost();
            }
            self.ivars().send(Event::State(raw));
        }

        #[unsafe(method(centralManager:didDiscoverPeripheral:advertisementData:RSSI:))]
        fn did_discover(
            &self,
            _central: &CBCentralManager,
            peripheral: &CBPeripheral,
            advertisement: &NSDictionary<NSString, AnyObject>,
            rssi: &NSNumber,
        ) {
            // SAFETY: property getters without arguments; the key is a
            // CoreBluetooth constant.
            let id = Uuid::from_bytes(unsafe { peripheral.identifier() }.as_bytes());
            let name = advertisement
                .objectForKey(unsafe { CBAdvertisementDataLocalNameKey })
                .and_then(|value| value.downcast::<NSString>().ok())
                .or_else(|| unsafe { peripheral.name() })
                .map(|name| name.to_string());
            self.ivars().send(Event::Discovered {
                id,
                name,
                rssi: rssi_dbm(rssi.as_isize()),
            });
        }

        #[unsafe(method(centralManager:didConnectPeripheral:))]
        fn did_connect(&self, _central: &CBCentralManager, _peripheral: &CBPeripheral) {
            self.ivars().connected.store(true, Ordering::Release);
            self.ivars().send(Event::Connected);
        }

        #[unsafe(method(centralManager:didFailToConnectPeripheral:error:))]
        fn did_fail_to_connect(
            &self,
            _central: &CBCentralManager,
            _peripheral: &CBPeripheral,
            error: Option<&NSError>,
        ) {
            self.ivars().send(Event::ConnectFailed(fail(error)));
        }

        #[unsafe(method(centralManager:didDisconnectPeripheral:error:))]
        fn did_disconnect(
            &self,
            _central: &CBCentralManager,
            _peripheral: &CBPeripheral,
            error: Option<&NSError>,
        ) {
            self.ivars().lost();
            self.ivars().send(Event::Disconnected(fail(error)));
        }
    }

    unsafe impl CBPeripheralDelegate for Delegate {
        #[unsafe(method(peripheral:didDiscoverServices:))]
        fn did_discover_services(&self, _peripheral: &CBPeripheral, error: Option<&NSError>) {
            self.ivars().send(Event::ServicesDiscovered(fail(error)));
        }

        #[unsafe(method(peripheral:didDiscoverCharacteristicsForService:error:))]
        fn did_discover_characteristics(
            &self,
            _peripheral: &CBPeripheral,
            _service: &CBService,
            error: Option<&NSError>,
        ) {
            self.ivars()
                .send(Event::CharacteristicsDiscovered(fail(error)));
        }

        #[unsafe(method(peripheral:didUpdateNotificationStateForCharacteristic:error:))]
        fn did_update_notification_state(
            &self,
            _peripheral: &CBPeripheral,
            characteristic: &CBCharacteristic,
            error: Option<&NSError>,
        ) {
            if uuid_of(characteristic) == self.ivars().notify {
                self.ivars().send(Event::NotifyState(fail(error)));
            }
        }

        #[unsafe(method(peripheral:didUpdateValueForCharacteristic:error:))]
        fn did_update_value(
            &self,
            _peripheral: &CBPeripheral,
            characteristic: &CBCharacteristic,
            error: Option<&NSError>,
        ) {
            if error.is_some() || uuid_of(characteristic) != self.ivars().notify {
                return;
            }
            // SAFETY: a property getter without arguments.
            if let Some(value) = unsafe { characteristic.value() } {
                self.ivars().send(Event::Value(value.to_vec()));
            }
        }

        #[unsafe(method(peripheral:didWriteValueForCharacteristic:error:))]
        fn did_write_value(
            &self,
            _peripheral: &CBPeripheral,
            _characteristic: &CBCharacteristic,
            error: Option<&NSError>,
        ) {
            self.ivars().send(Event::Written(fail(error)));
        }

        #[unsafe(method(peripheralIsReadyToSendWriteWithoutResponse:))]
        fn ready_to_send(&self, _peripheral: &CBPeripheral) {
            self.ivars().send(Event::ReadyToSend);
        }
    }
);

impl Delegate {
    fn new(ivars: Ivars) -> Retained<Self> {
        let this = Self::alloc().set_ivars(ivars);
        // SAFETY: NSObject's designated initializer.
        unsafe { msg_send![super(this), init] }
    }
}

// --------------------------------------------------------------- central

/// One central manager with its delegate and, once found, the peripheral and
/// its serial characteristics.
struct CbCentral {
    manager: Retained<CBCentralManager>,
    delegate: Retained<Delegate>,
    // Keeps the delegate's queue alive as long as the manager.
    queue: DispatchRetained<DispatchQueue>,
    peripheral: Option<Retained<CBPeripheral>>,
    write: Option<Retained<CBCharacteristic>>,
    notify: Option<Retained<CBCharacteristic>>,
}

impl CbCentral {
    /// A new manager; its first state arrives as an [`Event::State`].
    fn open(notify: Option<Uuid>, state: LinkState) -> (Self, Receiver<Event>) {
        let (tx, rx) = mpsc::channel();
        let delegate = Delegate::new(Ivars {
            tx,
            state,
            connected: AtomicBool::new(false),
            notify,
        });
        let queue = DispatchQueue::new(QUEUE_LABEL, None);
        // SAFETY: the delegate and the serial queue outlive the manager
        // (all three live in `Self`); the manager is weakly bound to the
        // delegate and only calls it on `queue`.
        let manager = unsafe {
            CBCentralManager::initWithDelegate_queue(
                CBCentralManager::alloc(),
                Some(ProtocolObject::from_ref(&*delegate)),
                Some(&queue),
            )
        };
        let central = Self {
            manager,
            delegate,
            queue,
            peripheral: None,
            write: None,
            notify: None,
        };
        (central, rx)
    }

    fn service(&self, uuid: Uuid) -> Option<Retained<CBService>> {
        let peripheral = self.peripheral.as_ref()?;
        // SAFETY: property getters without arguments.
        let services = unsafe { peripheral.services() }?;
        services
            .iter()
            .find(|service| uuid_of(service) == Some(uuid))
    }

    /// The open link's half of this central. `None` (after cancelling the
    /// connection) when the setup left a part out, which `connect_flow`
    /// rules out.
    fn into_wire(mut self) -> Option<CbWire> {
        match (
            self.peripheral.take(),
            self.write.take(),
            self.notify.take(),
        ) {
            (Some(peripheral), Some(write), Some(notify)) => Some(CbWire {
                manager: self.manager,
                peripheral,
                write,
                notify,
                _delegate: self.delegate,
                _queue: self.queue,
            }),
            (peripheral, ..) => {
                if let Some(peripheral) = peripheral {
                    // SAFETY: a peripheral of this manager.
                    unsafe { self.manager.cancelPeripheralConnection(&peripheral) };
                }
                None
            }
        }
    }
}

impl Central for CbCentral {
    fn state(&mut self) -> isize {
        // SAFETY: a property getter without arguments.
        unsafe { self.manager.state() }.0
    }

    fn retrieve(&mut self, id: Uuid) -> bool {
        let ids = NSArray::from_retained_slice(&[NSUUID::from_bytes(id.into_bytes())]);
        // SAFETY: an array of NSUUID, as the method takes.
        let found = unsafe { self.manager.retrievePeripheralsWithIdentifiers(&ids) };
        let Some(peripheral) = found.iter().next() else {
            return false;
        };
        // SAFETY: the delegate outlives the peripheral's use (both in
        // `self`, then in the link); the property is weak.
        unsafe { peripheral.setDelegate(Some(ProtocolObject::from_ref(&*self.delegate))) };
        self.peripheral = Some(peripheral);
        true
    }

    fn start_scan(&mut self) {
        // Duplicates refresh the signal strength and a name that only comes
        // with a later scan response, like BlueZ's property updates.
        let yes = NSNumber::new_bool(true);
        let value: &AnyObject = &yes;
        // SAFETY: the key is a CoreBluetooth constant and takes an NSNumber.
        unsafe {
            let options = NSDictionary::<NSString, AnyObject>::from_slices(
                &[CBCentralManagerScanOptionAllowDuplicatesKey],
                &[value],
            );
            self.manager
                .scanForPeripheralsWithServices_options(None, Some(&options));
        }
    }

    fn stop_scan(&mut self) {
        // SAFETY: no arguments; harmless when not scanning.
        unsafe { self.manager.stopScan() };
    }

    fn connect(&mut self) {
        if let Some(peripheral) = &self.peripheral {
            // SAFETY: a peripheral of this manager, no options.
            unsafe { self.manager.connectPeripheral_options(peripheral, None) };
        }
    }

    fn cancel(&mut self) {
        if let Some(peripheral) = &self.peripheral {
            // SAFETY: a peripheral of this manager.
            unsafe { self.manager.cancelPeripheralConnection(peripheral) };
        }
    }

    fn discover_services(&mut self, service: Uuid) {
        if let Some(peripheral) = &self.peripheral {
            let services = NSArray::from_retained_slice(&[cb_uuid(service)]);
            // SAFETY: an array of CBUUID, as the method takes.
            unsafe { peripheral.discoverServices(Some(&services)) };
        }
    }

    fn discover_characteristics(&mut self, service: Uuid, chars: &[Uuid]) -> bool {
        let (Some(peripheral), Some(service)) = (&self.peripheral, self.service(service)) else {
            return false;
        };
        let wanted: Vec<Retained<CBUUID>> = chars.iter().map(|&c| cb_uuid(c)).collect();
        let wanted = NSArray::from_retained_slice(&wanted);
        // SAFETY: an array of CBUUID and a service of this peripheral.
        unsafe { peripheral.discoverCharacteristics_forService(Some(&wanted), &service) };
        true
    }

    fn characteristics(&mut self, profile: &SerialProfile) -> (Option<usize>, Option<usize>) {
        let Some(service) = self.service(profile.service) else {
            return (None, None);
        };
        // SAFETY: property getters without arguments.
        let chars = unsafe { service.characteristics() };
        let find = |uuid: Uuid| chars.as_ref()?.iter().find(|c| uuid_of(c) == Some(uuid));
        self.write = find(profile.write_char);
        self.notify = find(profile.notify_char);
        let props = |c: &Option<Retained<CBCharacteristic>>| {
            c.as_ref().map(|c| unsafe { c.properties() }.0)
        };
        (props(&self.write), props(&self.notify))
    }

    fn set_notify(&mut self, on: bool) {
        if let (Some(peripheral), Some(notify)) = (&self.peripheral, &self.notify) {
            // SAFETY: a characteristic of this peripheral.
            unsafe { peripheral.setNotifyValue_forCharacteristic(on, notify) };
        }
    }
}

// ------------------------------------------------------------------ link

/// The CoreBluetooth objects of an open link.
struct CbWire {
    manager: Retained<CBCentralManager>,
    peripheral: Retained<CBPeripheral>,
    write: Retained<CBCharacteristic>,
    notify: Retained<CBCharacteristic>,
    // Keep the callbacks (and with them the link's channel) alive.
    _delegate: Retained<Delegate>,
    _queue: DispatchRetained<DispatchQueue>,
}

// SAFETY: the link is used by one thread at a time (`BleLink` takes
// `&mut self`); CoreBluetooth objects may be messaged from any thread, the
// framework serialising its work on the manager's queue (the way other
// CoreBluetooth clients such as bleak drive it), and objc reference counts
// are atomic. The delegate only touches `Send + Sync` ivars.
unsafe impl Send for CbWire {}

impl Wire for CbWire {
    fn can_send(&self) -> bool {
        // SAFETY: a property getter without arguments.
        unsafe { self.peripheral.canSendWriteWithoutResponse() }
    }

    fn send(&self, chunk: &[u8], mode: WriteMode) {
        let kind = match mode {
            WriteMode::WithoutResponse => CBCharacteristicWriteType::WithoutResponse,
            WriteMode::WithResponse => CBCharacteristicWriteType::WithResponse,
        };
        let data = NSData::with_bytes(chunk);
        // SAFETY: a characteristic of this peripheral, a write type it takes.
        unsafe {
            self.peripheral
                .writeValue_forCharacteristic_type(&data, &self.write, kind)
        };
    }

    fn shutdown(&self) {
        // SAFETY: property getters, and calls on this manager's peripheral
        // and one of its characteristics.
        unsafe {
            if self.peripheral.state() == CBPeripheralState::Connected && self.notify.isNotifying()
            {
                self.peripheral
                    .setNotifyValue_forCharacteristic(false, &self.notify);
            }
            self.manager.cancelPeripheralConnection(&self.peripheral);
        }
    }
}

// ------------------------------------------------------------ entry points

pub(crate) fn availability() -> Result<(), BleError> {
    let policy = Policy::DEFAULT;
    let wait = preflight(&policy)?;
    let (mut central, rx) = CbCentral::open(None, LinkState::new());
    wait_powered(&mut central, &rx, Instant::now() + wait)
}

pub(crate) fn scan(
    timeout: Duration,
    name_filter: &dyn Fn(&str) -> bool,
) -> Result<Vec<BleDeviceInfo>, BleError> {
    let policy = Policy::DEFAULT;
    let wait = preflight(&policy)?;
    let (mut central, rx) = CbCentral::open(None, LinkState::new());
    wait_powered(&mut central, &rx, Instant::now() + wait)?;
    central.start_scan();
    let devices = collect_scan(&rx, Instant::now() + timeout, name_filter);
    central.stop_scan();
    devices
}

pub(crate) fn connect_serial(
    device_id: &str,
    profile: &SerialProfile,
    timeout: Duration,
) -> Result<Box<dyn BleLink>, BleError> {
    let policy = Policy::DEFAULT;
    let wait = preflight(&policy)?;
    // A MAC address (an id from Linux or Windows) names no peripheral here.
    let id = parse_peripheral_id(device_id).ok_or(BleError::NotFound)?;
    let state = LinkState::new();
    let (mut central, rx) = CbCentral::open(Some(profile.notify_char), state.clone());
    // The first use waits for the user's answer to the permission prompt;
    // `timeout` then bounds the connection setup, as on the other platforms.
    wait_powered(&mut central, &rx, Instant::now() + wait)?;
    let mut inbox = VecDeque::new();
    let deadline = Instant::now() + timeout;
    let mode = match connect_flow(
        &mut central,
        &rx,
        id,
        profile,
        deadline,
        &policy,
        &mut inbox,
    ) {
        Ok(mode) => mode,
        Err(e) => {
            central.stop_scan();
            central.cancel();
            return Err(e);
        }
    };
    let Some(wire) = central.into_wire() else {
        state.mark_lost();
        return Err(BleError::Platform(
            "GATT serial characteristics went away".into(),
        ));
    };
    Ok(Box::new(Link::new(
        wire,
        rx,
        inbox,
        mode,
        profile.max_chunk,
        state,
        policy,
    )))
}
