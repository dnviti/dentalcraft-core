//! BlueZ backend: hand-written blocking `zbus` proxies, no libdbus.
//!
//! Layout: [`availability`] / [`scan`] / [`connect_serial`] talk to
//! `org.bluez` on the system bus. A link owns one connection plus a reader
//! thread that turns `PropertiesChanged` signals (characteristic `Value`,
//! device `Connected`) into an `mpsc` channel, so [`BleLink::recv`] can honour
//! a timeout.
//!
//! BlueZ forgets unpaired devices: their object goes `TemporaryTimeout`
//! (30 s) after the discovery that found them or after they disconnect, so
//! [`connect_serial`] discovers a device again when its id is unknown.

use std::collections::HashMap;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{self, RecvTimeoutError};
use std::thread;
use std::time::{Duration, Instant};

use uuid::Uuid;
use zbus::blocking::connection::Builder as ConnectionBuilder;
use zbus::blocking::fdo::ObjectManagerProxy;
use zbus::blocking::{Connection, MessageIterator};
use zbus::fdo::ManagedObjects;
use zbus::message::Type as MessageType;
use zbus::proxy;
use zbus::zvariant::{OwnedObjectPath, OwnedValue, Value};
use zbus::{DBusError, MatchRule};

use crate::{BleDeviceInfo, BleError, BleLink, LinkState, SerialProfile, chunks};

const BLUEZ: &str = "org.bluez";
const ADAPTER_IFACE: &str = "org.bluez.Adapter1";
const DEVICE_IFACE: &str = "org.bluez.Device1";
const SERVICE_IFACE: &str = "org.bluez.GattService1";
const CHAR_IFACE: &str = "org.bluez.GattCharacteristic1";

/// Time allowed for BlueZ calls that are not connection setup. Starting
/// `bluetoothd` through D-Bus activation can be slow, hence not tiny.
const PROBE_TIMEOUT: Duration = Duration::from_secs(10);
/// How often the scan loop re-reads the BlueZ object tree.
const SCAN_POLL: Duration = Duration::from_millis(300);
/// Poll period while waiting for GATT service resolution.
const RESOLVE_POLL: Duration = Duration::from_millis(100);
/// Pause between stopping discovery and `Device1.Connect`: BlueZ aborts a
/// connection started while the controller is still leaving discovery
/// (`le-connection-abort-by-local`).
const CONNECT_SETTLE: Duration = Duration::from_millis(500);
/// Pause before retrying a locally aborted `Connect`.
const CONNECT_RETRY_DELAY: Duration = Duration::from_secs(1);
/// Most `Connect` calls of one connection setup, see [`is_local_abort`].
const CONNECT_ATTEMPTS: u32 = 4;
/// Longest discovery of a device BlueZ no longer knows (see
/// [`discover_device`]); an advertising device shows up within a second.
const REDISCOVER_TIMEOUT: Duration = Duration::from_secs(10);

type Props = HashMap<String, OwnedValue>;

#[proxy(
    interface = "org.bluez.Adapter1",
    default_service = "org.bluez",
    gen_async = false,
    blocking_name = "Adapter1Proxy"
)]
trait Adapter1 {
    fn set_discovery_filter(&self, filter: HashMap<&str, Value<'_>>) -> zbus::Result<()>;
    fn start_discovery(&self) -> zbus::Result<()>;
    fn stop_discovery(&self) -> zbus::Result<()>;
}

#[proxy(
    interface = "org.bluez.Device1",
    default_service = "org.bluez",
    gen_async = false,
    blocking_name = "Device1Proxy"
)]
trait Device1 {
    fn connect(&self) -> zbus::Result<()>;
    fn disconnect(&self) -> zbus::Result<()>;
    #[zbus(property)]
    fn connected(&self) -> zbus::Result<bool>;
    #[zbus(property)]
    fn services_resolved(&self) -> zbus::Result<bool>;
}

#[proxy(
    interface = "org.bluez.GattCharacteristic1",
    default_service = "org.bluez",
    gen_async = false,
    blocking_name = "GattCharacteristic1Proxy"
)]
trait GattCharacteristic1 {
    fn write_value(&self, value: &[u8], options: HashMap<&str, Value<'_>>) -> zbus::Result<()>;
    fn start_notify(&self) -> zbus::Result<()>;
    fn stop_notify(&self) -> zbus::Result<()>;
}

// ---------------------------------------------------------------- errors

/// The D-Bus error name carried by `e`, when it is a remote error reply.
fn error_name(e: &zbus::Error) -> Option<String> {
    match e {
        zbus::Error::MethodError(name, _, _) => Some(name.to_string()),
        zbus::Error::FDO(f) => match &**f {
            zbus::fdo::Error::ZBus(inner) => error_name(inner),
            other => Some(other.name().to_string()),
        },
        _ => None,
    }
}

fn is_io_timeout(e: &zbus::Error) -> bool {
    matches!(e, zbus::Error::InputOutput(io) if io.kind() == std::io::ErrorKind::TimedOut)
}

/// Map a zbus error to the public error type.
fn map_err(e: zbus::Error) -> BleError {
    if is_io_timeout(&e) {
        return BleError::Timeout;
    }
    let Some(name) = error_name(&e) else {
        return BleError::Platform(e.to_string());
    };
    let n = name.as_str();
    if n.ends_with(".ServiceUnknown")
        || n.ends_with(".NameHasNoOwner")
        || n.starts_with("org.freedesktop.systemd1.")
        || n.contains(".Spawn")
    {
        // bluetoothd is not installed, not running and cannot be activated.
        BleError::NoAdapter
    } else if n.ends_with(".NoReply") || n.ends_with(".TimedOut") || n.ends_with(".Timeout") {
        BleError::Timeout
    } else if n == "org.bluez.Error.NotReady" || n == "org.bluez.Error.Blocked" {
        BleError::AdapterOff
    } else if n == "org.bluez.Error.DoesNotExist"
        || n == "org.bluez.Error.NotAvailable"
        || n.ends_with(".UnknownObject")
    {
        BleError::NotFound
    } else if n == "org.bluez.Error.NotConnected" || n.ends_with(".Disconnected") {
        BleError::Disconnected
    } else {
        BleError::Platform(format!("{e}"))
    }
}

/// BlueZ's reason for a connection the controller aborted itself, typically
/// because discovery was still stopping.
const LOCAL_ABORT: &str = "le-connection-abort-by-local";

/// `true` for the `org.bluez.Error.Failed` reply whose reason is
/// [`LOCAL_ABORT`], worth a retry: the StopDiscovery / Connect race, or a
/// link that failed to be established (HCI 0x3e: the peripheral missed its
/// first connection events, about one attempt in four with the Revopoint
/// turntable, sometimes twice in a row). `name` is the D-Bus error name,
/// `text` the error message.
fn is_local_abort(name: Option<&str>, text: &str) -> bool {
    name == Some("org.bluez.Error.Failed") && text.contains(LOCAL_ABORT)
}

/// [`is_local_abort`] for a zbus error.
fn is_local_abort_err(e: &zbus::Error) -> bool {
    is_local_abort(error_name(e).as_deref(), &e.to_string())
}

/// Like [`map_err`], for calls on an established link: a vanished object or a
/// "not connected" reply means the device is gone.
fn map_link_err(e: zbus::Error) -> BleError {
    match map_err(e) {
        BleError::NotFound => BleError::Disconnected,
        other => other,
    }
}

/// Failing to reach the system bus at all means there is no Bluetooth stack
/// to talk to (containers, minimal installs).
fn system_bus(method_timeout: Duration) -> Result<Connection, BleError> {
    ConnectionBuilder::system()
        .and_then(|b| b.method_timeout(method_timeout).build())
        .map_err(|_| BleError::NoAdapter)
}

// --------------------------------------------------------------- objects

fn managed_objects(conn: &Connection) -> Result<ManagedObjects, BleError> {
    let proxy = ObjectManagerProxy::new(conn, BLUEZ, "/").map_err(map_err)?;
    proxy
        .get_managed_objects()
        .map_err(|e| map_err(zbus::Error::from(e)))
}

fn prop_str(props: &Props, key: &str) -> Option<String> {
    props
        .get(key)?
        .downcast_ref::<&str>()
        .ok()
        .map(str::to_owned)
}

fn prop_bool(props: &Props, key: &str) -> Option<bool> {
    props.get(key)?.downcast_ref::<bool>().ok()
}

fn prop_i16(props: &Props, key: &str) -> Option<i16> {
    props.get(key)?.downcast_ref::<i16>().ok()
}

fn prop_path(props: &Props, key: &str) -> Option<OwnedObjectPath> {
    let v = props.get(key)?.try_clone().ok()?;
    OwnedObjectPath::try_from(v).ok()
}

fn prop_strings(props: &Props, key: &str) -> Vec<String> {
    props
        .get(key)
        .and_then(|v| v.try_clone().ok())
        .and_then(|v| Vec::<String>::try_from(v).ok())
        .unwrap_or_default()
}

fn prop_uuid(props: &Props, key: &str) -> Option<Uuid> {
    Uuid::parse_str(&prop_str(props, key)?).ok()
}

/// Properties of `iface` on every object of the tree, sorted by path so that
/// "first match" is deterministic.
fn with_interface<'a>(
    objects: &'a ManagedObjects,
    iface: &'static str,
) -> Vec<(&'a OwnedObjectPath, &'a Props)> {
    let mut found: Vec<_> = objects
        .iter()
        .filter_map(|(path, ifaces)| {
            ifaces
                .iter()
                .find(|(name, _)| name.as_str() == iface)
                .map(|(_, props)| (path, props))
        })
        .collect();
    found.sort_by(|a, b| a.0.as_str().cmp(b.0.as_str()));
    found
}

/// The adapter to use: the first powered one. `NoAdapter` if there is none,
/// `AdapterOff` if all are powered off.
fn pick_adapter(objects: &ManagedObjects) -> Result<OwnedObjectPath, BleError> {
    let adapters = with_interface(objects, ADAPTER_IFACE);
    if adapters.is_empty() {
        return Err(BleError::NoAdapter);
    }
    adapters
        .iter()
        .find(|(_, props)| prop_bool(props, "Powered").unwrap_or(false))
        .map(|(path, _)| (*path).clone())
        .ok_or(BleError::AdapterOff)
}

fn is_below(path: &OwnedObjectPath, parent: &OwnedObjectPath) -> bool {
    path.as_str()
        .strip_prefix(parent.as_str())
        .is_some_and(|rest| rest.starts_with('/'))
}

// ----------------------------------------------------------------- probe

pub(crate) fn availability() -> Result<(), BleError> {
    let conn = system_bus(PROBE_TIMEOUT)?;
    let objects = managed_objects(&conn)?;
    pick_adapter(&objects).map(|_| ())
}

// ------------------------------------------------------------------ scan

fn collect_devices(
    objects: &ManagedObjects,
    adapter: &OwnedObjectPath,
    name_filter: &dyn Fn(&str) -> bool,
    seen: &mut HashMap<String, BleDeviceInfo>,
) {
    for (path, props) in with_interface(objects, DEVICE_IFACE) {
        if !is_below(path, adapter) {
            continue;
        }
        let (Some(address), Some(name)) = (prop_str(props, "Address"), prop_str(props, "Name"))
        else {
            continue;
        };
        if !name_filter(&name) {
            continue;
        }
        let rssi = prop_i16(props, "RSSI");
        // BlueZ keeps devices it saw long ago in its cache. Those have no RSSI
        // any more; only report devices that are around (or already linked).
        if rssi.is_none() && !prop_bool(props, "Connected").unwrap_or(false) {
            continue;
        }
        let entry = seen
            .entry(address.clone())
            .or_insert_with(|| BleDeviceInfo {
                id: address,
                name: Some(name),
                rssi,
            });
        if rssi.is_some() {
            entry.rssi = rssi;
        }
    }
}

/// A discovery session of this client, stopped when dropped.
struct Discovery {
    adapter: Adapter1Proxy<'static>,
    /// `false` when it was already running for this client: not ours to stop.
    started: bool,
}

impl Discovery {
    /// Starts discovering Bluetooth LE devices on `adapter`.
    fn start(conn: &Connection, adapter: &OwnedObjectPath) -> Result<Self, BleError> {
        let adapter = Adapter1Proxy::builder(conn)
            .path(adapter.clone())
            .and_then(|b| b.build())
            .map_err(map_err)?;
        let filter = HashMap::from([
            ("Transport", Value::from("le")),
            ("DuplicateData", Value::from(false)),
        ]);
        adapter.set_discovery_filter(filter).map_err(map_err)?;
        // "InProgress": another client is already discovering. Fine for us, but
        // then it is not ours to stop.
        let started = match adapter.start_discovery() {
            Ok(()) => true,
            Err(e) if error_name(&e).as_deref() == Some("org.bluez.Error.InProgress") => false,
            Err(e) => return Err(map_err(e)),
        };
        Ok(Self { adapter, started })
    }
}

impl Drop for Discovery {
    fn drop(&mut self) {
        if self.started {
            let _ = self.adapter.stop_discovery();
        }
    }
}

pub(crate) fn scan(
    timeout: Duration,
    name_filter: &dyn Fn(&str) -> bool,
) -> Result<Vec<BleDeviceInfo>, BleError> {
    let conn = system_bus(PROBE_TIMEOUT)?;
    let objects = managed_objects(&conn)?;
    let adapter = pick_adapter(&objects)?;
    let discovery = Discovery::start(&conn, &adapter)?;

    let mut seen = HashMap::new();
    let deadline = Instant::now() + timeout;
    collect_devices(&objects, &adapter, name_filter, &mut seen);
    loop {
        let now = Instant::now();
        if now >= deadline {
            break;
        }
        thread::sleep(SCAN_POLL.min(deadline - now));
        collect_devices(&managed_objects(&conn)?, &adapter, name_filter, &mut seen);
    }
    drop(discovery);

    let mut devices: Vec<BleDeviceInfo> = seen.into_values().collect();
    devices.sort_by(|a, b| {
        b.rssi
            .cmp(&a.rssi)
            .then_with(|| a.name.cmp(&b.name))
            .then_with(|| a.id.cmp(&b.id))
    });
    Ok(devices)
}

// --------------------------------------------------------------- connect

/// Where the serial characteristics live in the BlueZ tree.
struct GattTarget {
    write_path: OwnedObjectPath,
    notify_path: OwnedObjectPath,
    /// `"command"` (write without response) when the characteristic offers it,
    /// otherwise `"request"`.
    write_type: &'static str,
}

fn find_device(
    objects: &ManagedObjects,
    adapter: &OwnedObjectPath,
    id: &str,
) -> Option<OwnedObjectPath> {
    with_interface(objects, DEVICE_IFACE)
        .into_iter()
        .filter(|(path, _)| is_below(path, adapter))
        .find(|(path, props)| {
            path.as_str() == id
                || prop_str(props, "Address").is_some_and(|a| a.eq_ignore_ascii_case(id))
        })
        .map(|(path, _)| path.clone())
}

fn find_gatt(
    objects: &ManagedObjects,
    device: &OwnedObjectPath,
    profile: &SerialProfile,
) -> Result<GattTarget, BleError> {
    let missing = |what: &str| BleError::Platform(format!("GATT {what} not found on the device"));
    let service = with_interface(objects, SERVICE_IFACE)
        .into_iter()
        .find(|(path, props)| {
            is_below(path, device) && prop_uuid(props, "UUID") == Some(profile.service)
        })
        .map(|(path, _)| path.clone())
        .ok_or_else(|| missing("serial service"))?;
    let chars = with_interface(objects, CHAR_IFACE);
    let find_char = |uuid: Uuid, what: &str| {
        chars
            .iter()
            .find(|(path, props)| {
                is_below(path, &service)
                    && prop_uuid(props, "UUID") == Some(uuid)
                    && prop_path(props, "Service").is_none_or(|s| s == service)
            })
            .map(|(path, props)| ((*path).clone(), *props))
            .ok_or_else(|| missing(what))
    };
    let (write_path, write_props) = find_char(profile.write_char, "write characteristic")?;
    let (notify_path, _) = find_char(profile.notify_char, "notify characteristic")?;
    let flags = prop_strings(write_props, "Flags");
    let write_type = if flags.is_empty() || flags.iter().any(|f| f == "write-without-response") {
        "command"
    } else {
        "request"
    };
    Ok(GattTarget {
        write_path,
        notify_path,
        write_type,
    })
}

enum Event {
    Data(Vec<u8>),
    Lost,
}

/// Start the thread that converts BlueZ signals into [`Event`]s. The match
/// rule covers the whole device subtree (characteristics and the device
/// itself) and deliberately has no `sender`: only BlueZ publishes there.
/// A lost link is also marked on `state` at once, so observers see it
/// before the next read.
fn spawn_reader(
    conn: &Connection,
    device: &OwnedObjectPath,
    notify_path: OwnedObjectPath,
    tx: mpsc::Sender<Event>,
    stop: Arc<AtomicBool>,
    state: LinkState,
) -> Result<(), BleError> {
    let rule = MatchRule::builder()
        .msg_type(MessageType::Signal)
        .interface("org.freedesktop.DBus.Properties")
        .and_then(|b| b.member("PropertiesChanged"))
        .and_then(|b| b.path_namespace(device.as_str()))
        .map(|b| b.build())
        .map_err(map_err)?;
    let iter = MessageIterator::for_match_rule(rule, conn, Some(1024)).map_err(map_err)?;
    let device = device.clone();
    thread::Builder::new()
        .name("dc-ble-notify".into())
        .spawn(move || {
            reader_loop(iter, &device, &notify_path, &tx, &stop);
            state.mark_lost();
            let _ = tx.send(Event::Lost);
        })
        .map(|_| ())
        .map_err(|e| BleError::Platform(format!("cannot start the BLE reader thread: {e}")))
}

/// Forwards the notifications until the link is lost or stopped.
fn reader_loop(
    iter: MessageIterator,
    device: &OwnedObjectPath,
    notify_path: &OwnedObjectPath,
    tx: &mpsc::Sender<Event>,
    stop: &AtomicBool,
) {
    for msg in iter {
        if stop.load(Ordering::Acquire) {
            return;
        }
        let Ok(msg) = msg else {
            return;
        };
        let header = msg.header();
        let Some(path) = header.path() else { continue };
        let Ok((iface, changed, _invalidated)) =
            msg.body().deserialize::<(String, Props, Vec<String>)>()
        else {
            continue;
        };
        if iface == CHAR_IFACE && path.as_str() == notify_path.as_str() {
            let value = changed
                .get("Value")
                .and_then(|v| v.try_clone().ok())
                .and_then(|v| Vec::<u8>::try_from(v).ok());
            if let Some(bytes) = value
                && tx.send(Event::Data(bytes)).is_err()
            {
                return;
            }
        } else if iface == DEVICE_IFACE
            && path.as_str() == device.as_str()
            && prop_bool(&changed, "Connected") == Some(false)
        {
            return;
        }
    }
    // The connection closed under us.
}

struct BluezLink {
    // Keeps the bus connection (and with it the proxies) alive.
    _conn: Connection,
    device: Device1Proxy<'static>,
    write_char: GattCharacteristic1Proxy<'static>,
    notify_char: GattCharacteristic1Proxy<'static>,
    write_type: &'static str,
    max_chunk: usize,
    rx: mpsc::Receiver<Event>,
    stop: Arc<AtomicBool>,
    state: LinkState,
    lost: bool,
    closed: bool,
}

impl BleLink for BluezLink {
    fn write(&mut self, data: &[u8]) -> Result<(), BleError> {
        if self.closed || self.lost {
            return Err(BleError::Disconnected);
        }
        for chunk in chunks(data, self.max_chunk) {
            let options = HashMap::from([("type", Value::from(self.write_type))]);
            if let Err(e) = self.write_char.write_value(chunk, options) {
                let e = map_link_err(e);
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
        self.stop.store(true, Ordering::Release);
        let _ = self.notify_char.stop_notify();
        // Disconnecting also emits a `Connected=false` signal that wakes the
        // reader thread so that it sees the stop flag and exits.
        let _ = self.device.disconnect();
    }

    fn state(&self) -> LinkState {
        self.state.clone()
    }
}

impl Drop for BluezLink {
    fn drop(&mut self) {
        self.close();
    }
}

/// Everything after `Connect`; on error the caller disconnects the device.
fn open_link(
    conn: &Connection,
    device_path: &OwnedObjectPath,
    device: Device1Proxy<'static>,
    profile: &SerialProfile,
    deadline: Instant,
) -> Result<BluezLink, BleError> {
    while !device.services_resolved().map_err(map_err)? {
        if !device.connected().map_err(map_err)? {
            return Err(BleError::Disconnected);
        }
        if Instant::now() >= deadline {
            return Err(BleError::Timeout);
        }
        thread::sleep(RESOLVE_POLL);
    }

    let objects = managed_objects(conn)?;
    let target = find_gatt(&objects, device_path, profile)?;
    let char_proxy = |path: &OwnedObjectPath| {
        GattCharacteristic1Proxy::builder(conn)
            .path(path.clone())
            .and_then(|b| b.build())
            .map_err(map_err)
    };
    let write_char = char_proxy(&target.write_path)?;
    let notify_char = char_proxy(&target.notify_path)?;

    // Subscribe to signals before enabling notifications so that no early
    // notification is lost.
    let (tx, rx) = mpsc::channel();
    let stop = Arc::new(AtomicBool::new(false));
    let state = LinkState::new();
    spawn_reader(
        conn,
        device_path,
        target.notify_path,
        tx,
        stop.clone(),
        state.clone(),
    )?;
    match notify_char.start_notify() {
        Ok(()) => {}
        // Already notifying (stale session): usable as is.
        Err(e) if error_name(&e).as_deref() == Some("org.bluez.Error.InProgress") => {}
        Err(e) => {
            stop.store(true, Ordering::Release);
            return Err(map_err(e));
        }
    }
    Ok(BluezLink {
        _conn: conn.clone(),
        device,
        write_char,
        notify_char,
        write_type: target.write_type,
        max_chunk: profile.max_chunk,
        rx,
        stop,
        state,
        lost: false,
        closed: false,
    })
}

pub(crate) fn connect_serial(
    device_id: &str,
    profile: &SerialProfile,
    timeout: Duration,
) -> Result<Box<dyn BleLink>, BleError> {
    let deadline = Instant::now() + timeout;
    let conn = system_bus(timeout.max(Duration::from_secs(1)))?;
    let objects = managed_objects(&conn)?;
    let adapter = pick_adapter(&objects)?;
    // discovery (ours just ended, or another client's) must be over before
    // Connect, or the controller aborts the connection
    quiesce_discovery(&conn, &objects, &adapter);
    // looked up only now, right before Connect: BlueZ drops unpaired devices
    let device_path = match find_device(&managed_objects(&conn)?, &adapter, device_id) {
        Some(path) => path,
        None => discover_device(&conn, &adapter, device_id, deadline)?,
    };
    let device = Device1Proxy::builder(&conn)
        .path(device_path.clone())
        .and_then(|b| b.build())
        .map_err(map_err)?;

    if !device.connected().map_err(map_err)? {
        let mut attempts = 1;
        loop {
            match device.connect() {
                Ok(()) => break,
                Err(e) if error_name(&e).as_deref() == Some("org.bluez.Error.AlreadyConnected") => {
                    break;
                }
                Err(e)
                    if attempts < CONNECT_ATTEMPTS
                        && is_local_abort_err(&e)
                        && Instant::now() + CONNECT_RETRY_DELAY < deadline =>
                {
                    attempts += 1;
                    thread::sleep(CONNECT_RETRY_DELAY);
                }
                Err(e) => {
                    // Cancel a connection attempt that may still be pending.
                    let _ = device.disconnect();
                    return Err(map_err(e));
                }
            }
        }
    }
    match open_link(&conn, &device_path, device.clone(), profile, deadline) {
        Ok(link) => Ok(Box::new(link)),
        Err(e) => {
            let _ = device.disconnect();
            Err(e)
        }
    }
}

/// Discovers `id` until BlueZ has an object for it again, for at most
/// [`REDISCOVER_TIMEOUT`] and never past `deadline`, then leaves the
/// controller [`CONNECT_SETTLE`] to stop discovering.
///
/// BlueZ keeps the object of an unpaired, untrusted device only while it is
/// connected or recently seen: a disconnection makes it temporary again
/// (`device_bonding_complete` with status "disconnected") and it is removed
/// `TemporaryTimeout` (30 s by default) later, like a device that was only
/// discovered. An id listed earlier is then unknown to BlueZ although the
/// device is right there and advertising.
fn discover_device(
    conn: &Connection,
    adapter: &OwnedObjectPath,
    id: &str,
    deadline: Instant,
) -> Result<OwnedObjectPath, BleError> {
    let deadline = deadline.min(Instant::now() + REDISCOVER_TIMEOUT);
    let discovery = Discovery::start(conn, adapter)?;
    let path = loop {
        if let Some(path) = find_device(&managed_objects(conn)?, adapter, id) {
            break path;
        }
        let now = Instant::now();
        if now >= deadline {
            return Err(BleError::NotFound);
        }
        thread::sleep(SCAN_POLL.min(deadline - now));
    };
    drop(discovery);
    thread::sleep(CONNECT_SETTLE);
    Ok(path)
}

/// Stops a running discovery on `adapter` (best effort: BlueZ only lets the
/// client that started it stop it) and leaves the controller
/// [`CONNECT_SETTLE`] to finish.
fn quiesce_discovery(conn: &Connection, objects: &ManagedObjects, adapter: &OwnedObjectPath) {
    let discovering = with_interface(objects, ADAPTER_IFACE)
        .into_iter()
        .find(|(path, _)| *path == adapter)
        .and_then(|(_, props)| prop_bool(props, "Discovering"))
        .unwrap_or(false);
    if discovering
        && let Ok(a) = Adapter1Proxy::builder(conn)
            .path(adapter.clone())
            .and_then(|b| b.build())
    {
        let _ = a.stop_discovery();
    }
    thread::sleep(CONNECT_SETTLE);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn subtree_check_requires_a_path_separator() {
        let parent = OwnedObjectPath::try_from("/org/bluez/hci0/dev_AA").unwrap();
        let child = OwnedObjectPath::try_from("/org/bluez/hci0/dev_AA/service0001").unwrap();
        let sibling = OwnedObjectPath::try_from("/org/bluez/hci0/dev_AAB/service0001").unwrap();
        assert!(is_below(&child, &parent));
        assert!(!is_below(&sibling, &parent));
        assert!(!is_below(&parent, &parent));
    }

    #[test]
    fn missing_bluez_maps_to_no_adapter() {
        let e = zbus::Error::from(zbus::fdo::Error::ServiceUnknown("no bluez".into()));
        assert!(matches!(map_err(e), BleError::NoAdapter));
        let e = zbus::Error::from(zbus::fdo::Error::NameHasNoOwner("none".into()));
        assert!(matches!(map_err(e), BleError::NoAdapter));
    }

    #[test]
    fn timeouts_and_vanished_objects_map_sensibly() {
        let io = std::io::Error::from(std::io::ErrorKind::TimedOut);
        assert!(matches!(
            map_err(zbus::Error::InputOutput(io.into())),
            BleError::Timeout
        ));
        let gone = zbus::Error::from(zbus::fdo::Error::UnknownObject("x".into()));
        assert!(matches!(map_err(gone.clone()), BleError::NotFound));
        assert!(matches!(map_link_err(gone), BleError::Disconnected));
    }

    #[test]
    fn only_the_local_abort_is_retried() {
        let failed = Some("org.bluez.Error.Failed");
        assert!(is_local_abort(
            failed,
            "org.bluez.Error.Failed: le-connection-abort-by-local"
        ));
        assert!(is_local_abort(failed, "le-connection-abort-by-local"));
        assert!(!is_local_abort(failed, "le-connection-abort-by-remote"));
        assert!(!is_local_abort(failed, "Software caused connection abort"));
        assert!(!is_local_abort(
            Some("org.bluez.Error.InProgress"),
            LOCAL_ABORT
        ));
        assert!(!is_local_abort(None, LOCAL_ABORT));
        // a zbus error carrying another name is not retried either
        let other = zbus::Error::from(zbus::fdo::Error::Failed(LOCAL_ABORT.into()));
        assert!(!is_local_abort_err(&other));
    }

    /// `true` while BlueZ has an object for the device `id`.
    fn bluez_knows(id: &str) -> bool {
        let conn = system_bus(PROBE_TIMEOUT).expect("system bus");
        let objects = managed_objects(&conn).expect("BlueZ objects");
        let adapter = pick_adapter(&objects).expect("powered adapter");
        find_device(&objects, &adapter, id).is_some()
    }

    /// Live, about a minute, with a powered unpaired HM-10 style device:
    /// `DC_BLE_IT=<address> cargo test -p dc-ble -- --ignored --nocapture`.
    /// A disconnection puts an unpaired device back to "temporary" and
    /// BlueZ removes its object `TemporaryTimeout` (30 s) later; the id
    /// listed before must still connect.
    #[test]
    #[ignore = "needs a powered unpaired HM-10 style device: DC_BLE_IT=<address>"]
    fn reconnects_after_bluez_dropped_the_device() {
        let id = std::env::var("DC_BLE_IT").expect("DC_BLE_IT=<device address>");
        let profile = SerialProfile::hm10();
        let timeout = Duration::from_secs(20);
        if !bluez_knows(&id) {
            scan(Duration::from_secs(10), &|_| true).expect("scan");
        }
        let mut link = connect_serial(&id, &profile, timeout).expect("first connection");
        link.close();
        drop(link);
        let dropped_by = Instant::now() + Duration::from_secs(60);
        while bluez_knows(&id) {
            assert!(
                Instant::now() < dropped_by,
                "BlueZ kept {id} after the disconnection: is it paired or trusted?"
            );
            thread::sleep(Duration::from_secs(1));
        }
        eprintln!("BlueZ dropped {id}; connecting again by its address");
        let mut link =
            connect_serial(&id, &profile, timeout).expect("connection after BlueZ dropped it");
        link.close();
    }
}
