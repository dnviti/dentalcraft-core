//! The platform-free half of the CoreBluetooth backend ([`super::macos`]):
//! the event model, the connection state machine with its timeouts and
//! retry policy, and the link (write flow control, notifications, close).
//!
//! CoreBluetooth only talks through delegate callbacks on a dispatch queue.
//! The macOS glue turns every callback into an [`Event`] on an `mpsc`
//! channel and exposes the few CoreBluetooth calls through [`Central`] and
//! [`Wire`]; everything that decides (what to wait for, when to retry, when
//! a link is lost) lives here and is tested on every platform with fakes.

// Compiled on other platforms only for its tests; there the glue that uses
// part of it does not exist.
#![cfg_attr(not(target_os = "macos"), allow(dead_code))]

use std::collections::{HashMap, VecDeque};
use std::sync::mpsc::{Receiver, RecvTimeoutError};
use std::thread;
use std::time::{Duration, Instant};

use uuid::Uuid;

use crate::{BleDeviceInfo, BleError, BleLink, LinkState, SerialProfile, chunks, uuid16};

/// Timeouts and retries of the backend; [`Policy::DEFAULT`] mirrors BlueZ.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Policy {
    /// Longest wait for the Bluetooth stack to report a settled state (the
    /// BlueZ backend's probe timeout).
    pub probe: Duration,
    /// Longest wait for the state while macOS shows its first-use Bluetooth
    /// permission prompt: the user has to read it and answer.
    pub prompt: Duration,
    /// Longest scan for a device CoreBluetooth does not know (any more).
    pub rediscover: Duration,
    /// Most connection attempts of one connection setup.
    pub attempts: u32,
    /// Pause before retrying a connection that failed to be established.
    pub retry_delay: Duration,
    /// Longest wait for room to write one chunk without response, or for
    /// the answer to a write with response.
    pub write: Duration,
    /// Longest wait for the disconnection to be confirmed on close.
    pub close: Duration,
    /// How often a wait for room to write re-checks the peripheral: some
    /// macOS releases skip `peripheralIsReadyToSendWriteWithoutResponse`.
    pub poll: Duration,
}

impl Policy {
    /// The values the BlueZ backend uses, plus the CoreBluetooth extras.
    pub const DEFAULT: Self = Self {
        probe: Duration::from_secs(10),
        prompt: Duration::from_secs(60),
        rediscover: Duration::from_secs(10),
        attempts: 4,
        retry_delay: Duration::from_secs(1),
        write: Duration::from_secs(5),
        close: Duration::from_secs(2),
        poll: Duration::from_millis(20),
    };
}

// ----------------------------------------------------------------- states

/// `CBManagerState` raw values.
pub(crate) mod manager_state {
    pub const UNKNOWN: isize = 0;
    pub const RESETTING: isize = 1;
    pub const UNSUPPORTED: isize = 2;
    pub const UNAUTHORIZED: isize = 3;
    pub const POWERED_OFF: isize = 4;
    pub const POWERED_ON: isize = 5;
}

/// What a `CBManagerState` means: `Some(Ok)` ready, `Some(Err)` a final
/// answer, `None` still settling (unknown, resetting).
pub(crate) fn state_outcome(raw: isize) -> Option<Result<(), BleError>> {
    use manager_state::*;
    match raw {
        POWERED_ON => Some(Ok(())),
        POWERED_OFF => Some(Err(BleError::AdapterOff)),
        UNAUTHORIZED => Some(Err(BleError::Unauthorized)),
        UNSUPPORTED => Some(Err(BleError::NoAdapter)),
        UNKNOWN | RESETTING => None,
        other => Some(Err(BleError::Platform(format!(
            "unexpected Bluetooth state {other}"
        )))),
    }
}

/// `CBManagerAuthorization` before any manager exists: `Err` when the user
/// (or a policy) refused Bluetooth to this app, `Ok(true)` when macOS has
/// not asked yet and the first manager will show the permission prompt.
pub(crate) fn authorization_outcome(raw: isize) -> Result<bool, BleError> {
    match raw {
        0 => Ok(true),                        // not determined
        1 | 2 => Err(BleError::Unauthorized), // restricted, denied
        _ => Ok(false),                       // allowed always
    }
}

/// `true` when a state change takes a connected link down: below
/// `PoweredOn` CoreBluetooth drops every connection without calling
/// `didDisconnectPeripheral`.
pub(crate) fn state_drops_links(raw: isize) -> bool {
    raw != manager_state::POWERED_ON
}

/// `true` for a usable `NSBluetoothAlwaysUsageDescription` value. Without
/// one, macOS kills the process the moment it touches CoreBluetooth.
pub(crate) fn usage_described(value: Option<&str>) -> bool {
    value.is_some_and(|v| !v.trim().is_empty())
}

// ------------------------------------------------------------- identifiers

/// The id of a peripheral: CoreBluetooth's per-Mac identifier, an
/// uppercase hyphenated UUID like `NSUUID.UUIDString`.
pub(crate) fn format_peripheral_id(bytes: [u8; 16]) -> String {
    Uuid::from_bytes(bytes)
        .hyphenated()
        .to_string()
        .to_ascii_uppercase()
}

/// The peripheral identifier of an id; `None` for anything else (a MAC
/// address from another platform, garbage).
pub(crate) fn parse_peripheral_id(id: &str) -> Option<Uuid> {
    Uuid::parse_str(id.trim()).ok()
}

/// A `CBUUID.UUIDString`: 4 hex digits for a Bluetooth SIG 16-bit UUID, 8
/// for a 32-bit one, else the full 128-bit form.
pub(crate) fn parse_cb_uuid(s: &str) -> Option<Uuid> {
    let base = uuid16(0).as_u128();
    let short = |digits: &str| u32::from_str_radix(digits, 16).ok();
    match s.len() {
        4 => short(s).map(|v| Uuid::from_u128((u128::from(v) << 96) | base)),
        8 => short(s).map(|v| Uuid::from_u128((u128::from(v) << 96) | base)),
        _ => Uuid::parse_str(s).ok(),
    }
}

/// The `CBUUID` string of `uuid`, the inverse of [`parse_cb_uuid`]: the
/// short form for a Bluetooth SIG UUID (`FFE0`), which is how CoreBluetooth
/// names those, else the full uppercase form.
pub(crate) fn cb_uuid_string(uuid: Uuid) -> String {
    let value = uuid.as_u128();
    if value & ((1u128 << 96) - 1) == uuid16(0).as_u128() {
        let short = value >> 96;
        if short <= 0xFFFF {
            return format!("{short:04X}");
        }
        return format!("{short:08X}");
    }
    uuid.hyphenated().to_string().to_ascii_uppercase()
}

/// Signal strength of a discovery: CoreBluetooth reports 127 when it has
/// none.
pub(crate) fn rssi_dbm(raw: isize) -> Option<i16> {
    if raw == 127 {
        return None;
    }
    i16::try_from(raw).ok()
}

// ------------------------------------------------------------------ errors

/// `CBErrorDomain`, the domain of CoreBluetooth's own errors.
pub(crate) const CB_ERROR_DOMAIN: &str = "CBErrorDomain";

/// An `NSError` from a delegate callback.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) struct Fail {
    pub domain: String,
    pub code: isize,
    pub text: String,
}

impl Fail {
    fn cb_code(&self) -> Option<isize> {
        (self.domain == CB_ERROR_DOMAIN).then_some(self.code)
    }

    /// The public error.
    pub fn to_error(&self) -> BleError {
        match self.cb_code() {
            // not connected, connection timeout, peripheral disconnected
            Some(3 | 6 | 7) => BleError::Disconnected,
            // unknown device
            Some(12) => BleError::NotFound,
            _ => BleError::Platform(format!("{} ({} {})", self.text, self.domain, self.code)),
        }
    }

    /// A connection that failed to be established and is worth another
    /// attempt, like BlueZ's `le-connection-abort-by-local`: unknown,
    /// connection timeout, peripheral disconnected, connection failed.
    pub fn transient(&self) -> bool {
        matches!(self.cb_code(), Some(0 | 6 | 7 | 10))
    }
}

fn fail_error(fail: Option<Fail>) -> Result<(), BleError> {
    match fail {
        None => Ok(()),
        Some(f) => Err(f.to_error()),
    }
}

// ------------------------------------------------------------------ events

/// What the delegate reports, in the order CoreBluetooth calls it.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) enum Event {
    /// `centralManagerDidUpdateState`, raw `CBManagerState`.
    State(isize),
    /// A device seen while scanning.
    Discovered {
        id: Uuid,
        name: Option<String>,
        rssi: Option<i16>,
    },
    Connected,
    ConnectFailed(Option<Fail>),
    /// The link dropped (also sent by the glue when the adapter goes down
    /// under a connected link).
    Disconnected(Option<Fail>),
    ServicesDiscovered(Option<Fail>),
    CharacteristicsDiscovered(Option<Fail>),
    NotifyState(Option<Fail>),
    /// A notification of the notify characteristic.
    Value(Vec<u8>),
    /// The answer to a write with response.
    Written(Option<Fail>),
    /// `peripheralIsReadyToSendWriteWithoutResponse`.
    ReadyToSend,
}

/// Next event before `deadline`: `Ok(None)` when it passed, `Err` when the
/// glue (and with it the delegate) is gone. Notifications are kept in
/// `inbox` so that none is lost while waiting for something else.
fn next_event(
    rx: &Receiver<Event>,
    deadline: Instant,
    inbox: &mut VecDeque<Vec<u8>>,
) -> Result<Option<Event>, BleError> {
    loop {
        let now = Instant::now();
        if now >= deadline {
            return Ok(None);
        }
        match rx.recv_timeout(deadline - now) {
            Ok(Event::Value(bytes)) => inbox.push_back(bytes),
            Ok(event) => return Ok(Some(event)),
            Err(RecvTimeoutError::Timeout) => return Ok(None),
            Err(RecvTimeoutError::Disconnected) => {
                return Err(BleError::Platform(
                    "the Bluetooth delegate went away".into(),
                ));
            }
        }
    }
}

// -------------------------------------------------------------------- scan

/// Adds a discovery to `seen` with the BlueZ backend's rules: devices
/// without a name are skipped, the filter decides on the name, a later
/// signal strength replaces an older one.
pub(crate) fn record_device(
    seen: &mut HashMap<String, BleDeviceInfo>,
    id: Uuid,
    name: Option<String>,
    rssi: Option<i16>,
    name_filter: &dyn Fn(&str) -> bool,
) {
    let Some(name) = name.filter(|n| !n.is_empty()) else {
        return;
    };
    if !name_filter(&name) {
        return;
    }
    let id = format_peripheral_id(id.into_bytes());
    let entry = seen.entry(id.clone()).or_insert_with(|| BleDeviceInfo {
        id,
        name: Some(name),
        rssi,
    });
    if rssi.is_some() {
        entry.rssi = rssi;
    }
}

/// The scan result in the BlueZ backend's order: strongest first, then by
/// name and id.
pub(crate) fn sorted_devices(seen: HashMap<String, BleDeviceInfo>) -> Vec<BleDeviceInfo> {
    let mut devices: Vec<BleDeviceInfo> = seen.into_values().collect();
    devices.sort_by(|a, b| {
        b.rssi
            .cmp(&a.rssi)
            .then_with(|| a.name.cmp(&b.name))
            .then_with(|| a.id.cmp(&b.id))
    });
    devices
}

/// Collects discoveries from `rx` until `deadline`.
pub(crate) fn collect_scan(
    rx: &Receiver<Event>,
    deadline: Instant,
    name_filter: &dyn Fn(&str) -> bool,
) -> Result<Vec<BleDeviceInfo>, BleError> {
    let mut seen = HashMap::new();
    let mut inbox = VecDeque::new();
    while let Some(event) = next_event(rx, deadline, &mut inbox)? {
        match event {
            Event::Discovered { id, name, rssi } => {
                record_device(&mut seen, id, name, rssi, name_filter);
            }
            Event::State(raw) => {
                if let Some(Err(e)) = state_outcome(raw) {
                    return Err(e);
                }
            }
            _ => {}
        }
    }
    Ok(sorted_devices(seen))
}

// ----------------------------------------------------------------- connect

/// How the serial characteristic takes writes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum WriteMode {
    WithoutResponse,
    WithResponse,
}

/// `CBCharacteristicProperties` bits used here.
pub(crate) mod props {
    pub const WRITE_WITHOUT_RESPONSE: usize = 0x04;
    pub const WRITE: usize = 0x08;
    pub const NOTIFY: usize = 0x10;
    pub const INDICATE: usize = 0x20;
}

/// The write mode of a characteristic with these properties, as BlueZ's
/// backend picks it: without response when offered (or when nothing is
/// declared), else with response.
pub(crate) fn write_mode(properties: usize) -> WriteMode {
    if properties & props::WRITE != 0 && properties & props::WRITE_WITHOUT_RESPONSE == 0 {
        WriteMode::WithResponse
    } else {
        WriteMode::WithoutResponse
    }
}

/// `true` when the notify characteristic can push values.
pub(crate) fn can_notify(properties: usize) -> bool {
    properties & (props::NOTIFY | props::INDICATE) != 0
}

/// The CoreBluetooth calls of a connection setup, on one central manager
/// and the one peripheral it connects. Calls only start operations; their
/// outcome arrives as an [`Event`].
pub(crate) trait Central {
    /// `CBManager.state`, raw.
    fn state(&mut self) -> isize;
    /// Looks the peripheral up among those the system knows
    /// (`retrievePeripheralsWithIdentifiers`) and keeps it; `false` when
    /// it is unknown.
    fn retrieve(&mut self, id: Uuid) -> bool;
    fn start_scan(&mut self);
    fn stop_scan(&mut self);
    /// `connectPeripheral` on the retrieved peripheral.
    fn connect(&mut self);
    /// `cancelPeripheralConnection`: drops a pending or open connection.
    fn cancel(&mut self);
    fn discover_services(&mut self, service: Uuid);
    /// Starts discovering `chars` of `service`; `false` when the peripheral
    /// has no such service.
    fn discover_characteristics(&mut self, service: Uuid, chars: &[Uuid]) -> bool;
    /// Keeps the write and notify characteristics of `profile` and returns
    /// their properties; `None` for a missing one (`write`, `notify`).
    fn characteristics(&mut self, profile: &SerialProfile) -> (Option<usize>, Option<usize>);
    /// `setNotifyValue` on the kept notify characteristic.
    fn set_notify(&mut self, on: bool);
}

fn missing(what: &str) -> BleError {
    BleError::Platform(format!("GATT {what} not found on the device"))
}

/// Waits for the manager to settle on `PoweredOn`.
pub(crate) fn wait_powered(
    central: &mut dyn Central,
    rx: &Receiver<Event>,
    deadline: Instant,
) -> Result<(), BleError> {
    let mut inbox = VecDeque::new();
    loop {
        if let Some(outcome) = state_outcome(central.state()) {
            return outcome;
        }
        match next_event(rx, deadline, &mut inbox)? {
            None => return Err(BleError::Timeout),
            Some(Event::State(raw)) => {
                if let Some(outcome) = state_outcome(raw) {
                    return outcome;
                }
            }
            Some(_) => {}
        }
    }
}

/// Waits until `pick` takes an event; a disconnection or the deadline
/// end the wait with an error.
fn wait_for<T>(
    rx: &Receiver<Event>,
    deadline: Instant,
    inbox: &mut VecDeque<Vec<u8>>,
    mut pick: impl FnMut(Event) -> Option<T>,
) -> Result<T, BleError> {
    loop {
        match next_event(rx, deadline, inbox)? {
            None => return Err(BleError::Timeout),
            Some(Event::Disconnected(_)) => return Err(BleError::Disconnected),
            Some(Event::State(raw)) if state_drops_links(raw) => {
                return Err(state_outcome(raw)
                    .and_then(Result::err)
                    .unwrap_or(BleError::Disconnected));
            }
            Some(event) => {
                if let Some(value) = pick(event) {
                    return Ok(value);
                }
            }
        }
    }
}

/// Makes the peripheral `id` the central's one: the system's record when
/// it has one, else a scan for at most [`Policy::rediscover`] (never past
/// `deadline`). CoreBluetooth remembers peripherals across launches, but
/// not forever: an id listed long ago must still connect, without any
/// "forget and pair again" by the user.
fn find_peripheral(
    central: &mut dyn Central,
    rx: &Receiver<Event>,
    id: Uuid,
    deadline: Instant,
    policy: &Policy,
    inbox: &mut VecDeque<Vec<u8>>,
) -> Result<(), BleError> {
    if central.retrieve(id) {
        return Ok(());
    }
    central.start_scan();
    let until = deadline.min(Instant::now() + policy.rediscover);
    let seen = wait_for(rx, until, inbox, |e| match e {
        Event::Discovered { id: found, .. } if found == id => Some(()),
        _ => None,
    });
    central.stop_scan();
    match seen {
        Ok(()) if central.retrieve(id) => Ok(()),
        Ok(()) | Err(BleError::Timeout) => Err(BleError::NotFound),
        Err(e) => Err(e),
    }
}

/// `connectPeripheral` until it succeeds, retrying a connection that failed
/// to be established (see [`Fail::transient`]) up to [`Policy::attempts`]
/// times. CoreBluetooth never times a connection out: past `deadline` the
/// pending attempt is cancelled.
fn connect_with_retries(
    central: &mut dyn Central,
    rx: &Receiver<Event>,
    deadline: Instant,
    policy: &Policy,
    inbox: &mut VecDeque<Vec<u8>>,
) -> Result<(), BleError> {
    let mut attempts = 1;
    loop {
        central.connect();
        let outcome = wait_for(rx, deadline, inbox, |e| match e {
            Event::Connected => Some(None),
            Event::ConnectFailed(fail) => Some(Some(fail)),
            _ => None,
        });
        match outcome {
            Ok(None) => return Ok(()),
            Ok(Some(fail)) => {
                let retry = attempts < policy.attempts
                    && fail.as_ref().is_none_or(Fail::transient)
                    && Instant::now() + policy.retry_delay < deadline;
                if !retry {
                    return Err(fail.map_or(BleError::Disconnected, |f| f.to_error()));
                }
                attempts += 1;
                thread::sleep(policy.retry_delay);
            }
            Err(e) => {
                central.cancel();
                return Err(e);
            }
        }
    }
}

/// The whole connection setup on `central`: wait for Bluetooth, find the
/// peripheral, connect (with retries), find the serial service and its
/// characteristics, enable notifications. Returns the write mode; on error
/// the caller cancels the connection. Notifications that arrive early are
/// left in `inbox`.
pub(crate) fn connect_flow(
    central: &mut dyn Central,
    rx: &Receiver<Event>,
    id: Uuid,
    profile: &SerialProfile,
    deadline: Instant,
    policy: &Policy,
    inbox: &mut VecDeque<Vec<u8>>,
) -> Result<WriteMode, BleError> {
    wait_powered(central, rx, deadline)?;
    find_peripheral(central, rx, id, deadline, policy, inbox)?;
    connect_with_retries(central, rx, deadline, policy, inbox)?;

    central.discover_services(profile.service);
    wait_for(rx, deadline, inbox, |e| match e {
        Event::ServicesDiscovered(fail) => Some(fail),
        _ => None,
    })
    .and_then(fail_error)?;
    let mut wanted = vec![profile.write_char];
    if profile.notify_char != profile.write_char {
        wanted.push(profile.notify_char);
    }
    if !central.discover_characteristics(profile.service, &wanted) {
        return Err(missing("serial service"));
    }
    wait_for(rx, deadline, inbox, |e| match e {
        Event::CharacteristicsDiscovered(fail) => Some(fail),
        _ => None,
    })
    .and_then(fail_error)?;
    let (write, notify) = central.characteristics(profile);
    let write = write.ok_or_else(|| missing("write characteristic"))?;
    let notify = notify.ok_or_else(|| missing("notify characteristic"))?;
    if !can_notify(notify) {
        return Err(BleError::Platform(
            "the GATT notify characteristic neither notifies nor indicates".into(),
        ));
    }

    central.set_notify(true);
    wait_for(rx, deadline, inbox, |e| match e {
        Event::NotifyState(fail) => Some(fail),
        _ => None,
    })
    .and_then(fail_error)?;
    Ok(write_mode(write))
}

// -------------------------------------------------------------------- link

/// The CoreBluetooth calls of an open link.
pub(crate) trait Wire {
    /// `canSendWriteWithoutResponse`.
    fn can_send(&self) -> bool;
    /// `writeValue:forCharacteristic:type:` on the write characteristic.
    fn send(&self, chunk: &[u8], mode: WriteMode);
    /// Notifications off, then `cancelPeripheralConnection`.
    fn shutdown(&self);
}

/// An open link over a [`Wire`]: chunked writes with flow control,
/// notifications, the shared [`LinkState`], close on drop.
pub(crate) struct Link<W: Wire> {
    wire: W,
    rx: Receiver<Event>,
    inbox: VecDeque<Vec<u8>>,
    mode: WriteMode,
    max_chunk: usize,
    policy: Policy,
    state: LinkState,
    lost: bool,
    closed: bool,
}

impl<W: Wire> Link<W> {
    pub fn new(
        wire: W,
        rx: Receiver<Event>,
        inbox: VecDeque<Vec<u8>>,
        mode: WriteMode,
        max_chunk: usize,
        state: LinkState,
        policy: Policy,
    ) -> Self {
        Self {
            wire,
            rx,
            inbox,
            mode,
            max_chunk,
            policy,
            state,
            lost: false,
            closed: false,
        }
    }

    fn mark_lost(&mut self) -> BleError {
        self.lost = true;
        self.state.mark_lost();
        BleError::Disconnected
    }

    /// Handles one event that is not the one a write waits for: a lost
    /// link becomes the error.
    fn absorb(&mut self, event: Event) -> Result<(), BleError> {
        match event {
            Event::Disconnected(_) => Err(self.mark_lost()),
            Event::State(raw) if state_drops_links(raw) => Err(self.mark_lost()),
            _ => Ok(()),
        }
    }

    /// Waits until the peripheral takes a write without response.
    fn wait_room(&mut self, deadline: Instant) -> Result<(), BleError> {
        loop {
            if self.wire.can_send() {
                return Ok(());
            }
            let now = Instant::now();
            if now >= deadline {
                return Err(BleError::Timeout);
            }
            // Short slices: the ready callback is not reliable everywhere.
            let slice = deadline.min(now + self.policy.poll);
            match next_event(&self.rx, slice, &mut self.inbox) {
                Ok(None | Some(Event::ReadyToSend)) => {}
                Ok(Some(event)) => self.absorb(event)?,
                Err(_) => return Err(self.mark_lost()),
            }
        }
    }

    /// Waits for the answer to a write with response.
    fn wait_written(&mut self, deadline: Instant) -> Result<(), BleError> {
        loop {
            match next_event(&self.rx, deadline, &mut self.inbox) {
                Ok(None) => return Err(BleError::Timeout),
                Ok(Some(Event::Written(fail))) => {
                    return match fail.map(|f| f.to_error()) {
                        None => Ok(()),
                        Some(BleError::Disconnected) => Err(self.mark_lost()),
                        Some(e) => Err(e),
                    };
                }
                Ok(Some(event)) => self.absorb(event)?,
                Err(_) => return Err(self.mark_lost()),
            }
        }
    }

    /// [`BleLink::close`]: notifications off and disconnect, then wait
    /// (briefly) for CoreBluetooth to confirm, since
    /// `cancelPeripheralConnection` does not block: the device is free for
    /// the next connection when this returns. Idempotent.
    fn shut(&mut self) {
        if self.closed {
            return;
        }
        self.closed = true;
        self.state.mark_lost();
        self.wire.shutdown();
        if self.lost {
            return;
        }
        let deadline = Instant::now() + self.policy.close;
        while let Ok(Some(event)) = next_event(&self.rx, deadline, &mut self.inbox) {
            if matches!(event, Event::Disconnected(_)) {
                break;
            }
        }
        self.lost = true;
    }
}

impl<W: Wire + Send> BleLink for Link<W> {
    fn write(&mut self, data: &[u8]) -> Result<(), BleError> {
        if self.closed || self.lost {
            return Err(BleError::Disconnected);
        }
        for chunk in chunks(data, self.max_chunk) {
            let deadline = Instant::now() + self.policy.write;
            if self.mode == WriteMode::WithoutResponse {
                self.wait_room(deadline)?;
            }
            self.wire.send(chunk, self.mode);
            if self.mode == WriteMode::WithResponse {
                self.wait_written(deadline)?;
            }
        }
        Ok(())
    }

    fn recv(&mut self, timeout: Duration) -> Result<Option<Vec<u8>>, BleError> {
        if self.closed {
            return Err(BleError::Disconnected);
        }
        if let Some(bytes) = self.inbox.pop_front() {
            return Ok(Some(bytes));
        }
        if self.lost {
            return Err(BleError::Disconnected);
        }
        let deadline = Instant::now() + timeout;
        loop {
            let now = Instant::now();
            if now >= deadline {
                return Ok(None);
            }
            match self.rx.recv_timeout(deadline - now) {
                Ok(Event::Value(bytes)) => return Ok(Some(bytes)),
                Ok(event) => self.absorb(event)?,
                Err(RecvTimeoutError::Timeout) => return Ok(None),
                Err(RecvTimeoutError::Disconnected) => return Err(self.mark_lost()),
            }
        }
    }

    fn close(&mut self) {
        self.shut();
    }

    fn state(&self) -> LinkState {
        self.state.clone()
    }
}

impl<W: Wire> Drop for Link<W> {
    fn drop(&mut self) {
        self.shut();
    }
}

#[cfg(test)]
mod tests;
