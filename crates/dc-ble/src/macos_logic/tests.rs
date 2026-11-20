//! The CoreBluetooth backend's logic against a fake central and a fake wire.

use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::mpsc::{self, Sender};
use std::sync::{Arc, Mutex};

use super::*;
use crate::BleLink;

const ID: Uuid = Uuid::from_u128(0x8c1f_4a9e_0b7d_4e21_9f3a_55aa_10c0_ffee);

/// Short timings so that the failure paths finish quickly.
const FAST: Policy = Policy {
    probe: Duration::from_millis(200),
    prompt: Duration::from_millis(200),
    rediscover: Duration::from_millis(150),
    attempts: 4,
    retry_delay: Duration::from_millis(5),
    write: Duration::from_millis(150),
    close: Duration::from_millis(100),
    poll: Duration::from_millis(5),
};

fn soon(d: Duration) -> Instant {
    Instant::now() + d
}

fn cb_error(code: isize) -> Fail {
    Fail {
        domain: CB_ERROR_DOMAIN.into(),
        code,
        text: "test".into(),
    }
}

// ------------------------------------------------------------ pure helpers

#[test]
fn manager_states_map_to_the_public_errors() {
    use manager_state::*;
    assert!(matches!(state_outcome(POWERED_ON), Some(Ok(()))));
    assert!(matches!(
        state_outcome(POWERED_OFF),
        Some(Err(BleError::AdapterOff))
    ));
    assert!(matches!(
        state_outcome(UNAUTHORIZED),
        Some(Err(BleError::Unauthorized))
    ));
    assert!(matches!(
        state_outcome(UNSUPPORTED),
        Some(Err(BleError::NoAdapter))
    ));
    assert!(state_outcome(UNKNOWN).is_none());
    assert!(state_outcome(RESETTING).is_none());
    assert!(matches!(
        state_outcome(42),
        Some(Err(BleError::Platform(_)))
    ));
    assert!(!state_drops_links(POWERED_ON));
    assert!(state_drops_links(POWERED_OFF));
    assert!(state_drops_links(RESETTING));
}

#[test]
fn authorization_refusals_are_unauthorized() {
    assert!(matches!(authorization_outcome(0), Ok(true)));
    assert!(matches!(
        authorization_outcome(1),
        Err(BleError::Unauthorized)
    ));
    assert!(matches!(
        authorization_outcome(2),
        Err(BleError::Unauthorized)
    ));
    assert!(matches!(authorization_outcome(3), Ok(false)));
}

#[test]
fn only_a_real_usage_text_counts() {
    assert!(usage_described(Some("DentalCraft turns the turntable")));
    assert!(!usage_described(Some("  ")));
    assert!(!usage_described(Some("")));
    assert!(!usage_described(None));
}

#[test]
fn peripheral_ids_round_trip_and_reject_mac_addresses() {
    let text = format_peripheral_id(ID.into_bytes());
    assert_eq!(text, "8C1F4A9E-0B7D-4E21-9F3A-55AA10C0FFEE");
    assert_eq!(parse_peripheral_id(&text), Some(ID));
    assert_eq!(parse_peripheral_id(&text.to_lowercase()), Some(ID));
    assert_eq!(
        parse_peripheral_id(" 8C1F4A9E-0B7D-4E21-9F3A-55AA10C0FFEE "),
        Some(ID)
    );
    assert_eq!(parse_peripheral_id("DE:B6:85:06:68:79"), None);
    assert_eq!(parse_peripheral_id(""), None);
}

#[test]
fn cb_uuids_use_the_short_form_for_sig_uuids() {
    assert_eq!(cb_uuid_string(uuid16(0xFFE0)), "FFE0");
    assert_eq!(cb_uuid_string(uuid16(0x2902)), "2902");
    let long = Uuid::from_u128(0x1234_5678 << 96 | uuid16(0).as_u128());
    assert_eq!(cb_uuid_string(long), "12345678");
    let custom = Uuid::parse_str("6e400001-b5a3-f393-e0a9-e50e24dcca9e").unwrap();
    assert_eq!(
        cb_uuid_string(custom),
        "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
    );
    for uuid in [uuid16(0xFFE0), uuid16(0xFFE1), long, custom] {
        assert_eq!(parse_cb_uuid(&cb_uuid_string(uuid)), Some(uuid));
    }
    assert_eq!(parse_cb_uuid("ffe1"), Some(uuid16(0xFFE1)));
    assert_eq!(parse_cb_uuid("XYZ1"), None);
}

#[test]
fn rssi_127_means_unknown() {
    assert_eq!(rssi_dbm(127), None);
    assert_eq!(rssi_dbm(-61), Some(-61));
    assert_eq!(rssi_dbm(isize::MAX), None);
}

#[test]
fn corebluetooth_errors_map_and_classify() {
    assert!(matches!(cb_error(7).to_error(), BleError::Disconnected));
    assert!(matches!(cb_error(3).to_error(), BleError::Disconnected));
    assert!(matches!(cb_error(12).to_error(), BleError::NotFound));
    assert!(matches!(cb_error(14).to_error(), BleError::Platform(_)));
    for transient in [0, 6, 7, 10] {
        assert!(cb_error(transient).transient(), "code {transient}");
    }
    for fatal in [1, 3, 11, 14] {
        assert!(!cb_error(fatal).transient(), "code {fatal}");
    }
    let att = Fail {
        domain: "CBATTErrorDomain".into(),
        code: 7,
        text: "x".into(),
    };
    assert!(!att.transient());
    assert!(matches!(att.to_error(), BleError::Platform(_)));
}

#[test]
fn write_mode_prefers_without_response() {
    use props::*;
    assert_eq!(
        write_mode(WRITE_WITHOUT_RESPONSE),
        WriteMode::WithoutResponse
    );
    assert_eq!(
        write_mode(WRITE | WRITE_WITHOUT_RESPONSE | NOTIFY),
        WriteMode::WithoutResponse
    );
    assert_eq!(write_mode(WRITE), WriteMode::WithResponse);
    assert_eq!(write_mode(0), WriteMode::WithoutResponse);
    assert!(can_notify(NOTIFY));
    assert!(can_notify(INDICATE));
    assert!(!can_notify(WRITE));
}

// -------------------------------------------------------------------- scan

#[test]
fn scan_keeps_named_matches_with_the_latest_rssi_strongest_first() {
    let (tx, rx) = mpsc::channel();
    let other = Uuid::from_u128(7);
    let unnamed = Uuid::from_u128(8);
    let speaker = Uuid::from_u128(9);
    for event in [
        Event::Discovered {
            id: ID,
            name: Some("REVO_DUAL_AXIS_TABLE".into()),
            rssi: Some(-80),
        },
        Event::Discovered {
            id: unnamed,
            name: None,
            rssi: Some(-30),
        },
        Event::Discovered {
            id: speaker,
            name: Some("Speaker".into()),
            rssi: Some(-20),
        },
        Event::Discovered {
            id: other,
            name: Some("REVO_DUAL_AXIS_TABLE".into()),
            rssi: Some(-70),
        },
        // A later report refreshes the strength; one without keeps it.
        Event::Discovered {
            id: ID,
            name: Some("REVO_DUAL_AXIS_TABLE".into()),
            rssi: Some(-50),
        },
        Event::Discovered {
            id: ID,
            name: Some("REVO_DUAL_AXIS_TABLE".into()),
            rssi: None,
        },
        Event::State(manager_state::POWERED_ON),
    ] {
        tx.send(event).unwrap();
    }
    let devices = collect_scan(&rx, soon(Duration::from_millis(50)), &|n| {
        n.starts_with("REVO")
    })
    .unwrap();
    let ids: Vec<&str> = devices.iter().map(|d| d.id.as_str()).collect();
    assert_eq!(
        ids,
        [
            format_peripheral_id(ID.into_bytes()),
            format_peripheral_id(other.into_bytes())
        ]
    );
    assert_eq!(devices[0].rssi, Some(-50));
}

#[test]
fn scan_stops_when_bluetooth_goes_off() {
    let (tx, rx) = mpsc::channel();
    tx.send(Event::State(manager_state::POWERED_OFF)).unwrap();
    let started = Instant::now();
    let e = collect_scan(&rx, soon(Duration::from_secs(5)), &|_| true).unwrap_err();
    assert!(matches!(e, BleError::AdapterOff));
    assert!(started.elapsed() < Duration::from_secs(1));
}

// ----------------------------------------------------------------- connect

/// A scripted CoreBluetooth central: every call answers on the channel the
/// way the delegate would.
struct FakeCentral {
    tx: Sender<Event>,
    state: isize,
    /// `retrievePeripheralsWithIdentifiers` knows the device.
    known: bool,
    /// A scan finds the device (which makes it known).
    advertising: bool,
    /// What each `connect` produces, in order; when empty, nothing (a
    /// connection pending forever, as CoreBluetooth does).
    connects: VecDeque<Event>,
    has_service: bool,
    write_props: Option<usize>,
    notify_props: Option<usize>,
    /// A notification arrives together with the subscription.
    early_value: bool,
    calls: Vec<&'static str>,
}

impl FakeCentral {
    fn new() -> (Self, Receiver<Event>) {
        let (tx, rx) = mpsc::channel();
        let central = Self {
            tx,
            state: manager_state::POWERED_ON,
            known: true,
            advertising: true,
            connects: VecDeque::from([Event::Connected]),
            has_service: true,
            write_props: Some(props::WRITE_WITHOUT_RESPONSE | props::NOTIFY),
            notify_props: Some(props::WRITE_WITHOUT_RESPONSE | props::NOTIFY),
            early_value: false,
            calls: Vec::new(),
        };
        (central, rx)
    }

    fn count(&self, call: &str) -> usize {
        self.calls.iter().filter(|c| **c == call).count()
    }
}

impl Central for FakeCentral {
    fn state(&mut self) -> isize {
        self.state
    }

    fn retrieve(&mut self, id: Uuid) -> bool {
        self.calls.push("retrieve");
        self.known && id == ID
    }

    fn start_scan(&mut self) {
        self.calls.push("start_scan");
        if self.advertising {
            self.known = true;
            let _ = self.tx.send(Event::Discovered {
                id: Uuid::from_u128(1),
                name: Some("Other".into()),
                rssi: None,
            });
            let _ = self.tx.send(Event::Discovered {
                id: ID,
                name: Some("REVO_DUAL_AXIS_TABLE".into()),
                rssi: Some(-60),
            });
        }
    }

    fn stop_scan(&mut self) {
        self.calls.push("stop_scan");
    }

    fn connect(&mut self) {
        self.calls.push("connect");
        if let Some(event) = self.connects.pop_front() {
            let _ = self.tx.send(event);
        }
    }

    fn cancel(&mut self) {
        self.calls.push("cancel");
    }

    fn discover_services(&mut self, service: Uuid) {
        self.calls.push("discover_services");
        assert_eq!(service, uuid16(0xFFE0));
        let _ = self.tx.send(Event::ServicesDiscovered(None));
    }

    fn discover_characteristics(&mut self, service: Uuid, chars: &[Uuid]) -> bool {
        self.calls.push("discover_characteristics");
        assert_eq!(service, uuid16(0xFFE0));
        assert_eq!(chars, [uuid16(0xFFE1)], "write = notify: asked once");
        if self.has_service {
            let _ = self.tx.send(Event::CharacteristicsDiscovered(None));
        }
        self.has_service
    }

    fn characteristics(&mut self, _profile: &SerialProfile) -> (Option<usize>, Option<usize>) {
        (self.write_props, self.notify_props)
    }

    fn set_notify(&mut self, on: bool) {
        self.calls.push(if on { "notify_on" } else { "notify_off" });
        if self.early_value {
            let _ = self.tx.send(Event::Value(b"+OK;".to_vec()));
        }
        let _ = self.tx.send(Event::NotifyState(None));
    }
}

fn run_connect(
    central: &mut FakeCentral,
    rx: &Receiver<Event>,
    timeout: Duration,
) -> (Result<WriteMode, BleError>, VecDeque<Vec<u8>>) {
    let mut inbox = VecDeque::new();
    let outcome = connect_flow(
        central,
        rx,
        ID,
        &SerialProfile::hm10(),
        soon(timeout),
        &FAST,
        &mut inbox,
    );
    (outcome, inbox)
}

#[test]
fn a_known_peripheral_connects_without_scanning() {
    let (mut central, rx) = FakeCentral::new();
    let (outcome, inbox) = run_connect(&mut central, &rx, Duration::from_secs(2));
    assert_eq!(outcome.unwrap(), WriteMode::WithoutResponse);
    assert!(inbox.is_empty());
    assert_eq!(
        central.calls,
        [
            "retrieve",
            "connect",
            "discover_services",
            "discover_characteristics",
            "notify_on"
        ]
    );
}

#[test]
fn a_forgotten_peripheral_is_discovered_again_first_time() {
    let (mut central, rx) = FakeCentral::new();
    central.known = false;
    let (outcome, _) = run_connect(&mut central, &rx, Duration::from_secs(2));
    outcome.unwrap();
    assert_eq!(
        &central.calls[..5],
        ["retrieve", "start_scan", "stop_scan", "retrieve", "connect"]
    );
}

#[test]
fn a_peripheral_that_does_not_advertise_is_not_found() {
    let (mut central, rx) = FakeCentral::new();
    central.known = false;
    central.advertising = false;
    let started = Instant::now();
    let (outcome, _) = run_connect(&mut central, &rx, Duration::from_secs(2));
    assert!(matches!(outcome, Err(BleError::NotFound)));
    // The rediscovery is bounded by its own limit, not the whole timeout.
    assert!(started.elapsed() < Duration::from_secs(1));
    assert_eq!(central.count("stop_scan"), 1);
    assert_eq!(central.count("connect"), 0);
}

#[test]
fn connections_that_fail_to_be_established_are_retried() {
    let (mut central, rx) = FakeCentral::new();
    central.connects = VecDeque::from([
        Event::ConnectFailed(Some(cb_error(10))),
        Event::ConnectFailed(Some(cb_error(6))),
        Event::ConnectFailed(None),
        Event::Connected,
    ]);
    let (outcome, _) = run_connect(&mut central, &rx, Duration::from_secs(2));
    outcome.unwrap();
    assert_eq!(central.count("connect"), 4);
}

#[test]
fn retries_stop_after_the_last_attempt() {
    let (mut central, rx) = FakeCentral::new();
    central.connects = (0..6)
        .map(|_| Event::ConnectFailed(Some(cb_error(10))))
        .collect();
    let (outcome, _) = run_connect(&mut central, &rx, Duration::from_secs(2));
    assert!(matches!(outcome, Err(BleError::Platform(_))));
    assert_eq!(central.count("connect"), FAST.attempts as usize);
}

#[test]
fn a_lasting_failure_is_not_retried() {
    let (mut central, rx) = FakeCentral::new();
    central.connects = VecDeque::from([Event::ConnectFailed(Some(cb_error(14))), Event::Connected]);
    let (outcome, _) = run_connect(&mut central, &rx, Duration::from_secs(2));
    assert!(matches!(outcome, Err(BleError::Platform(_))));
    assert_eq!(central.count("connect"), 1);
}

#[test]
fn a_pending_connection_is_cancelled_at_the_deadline() {
    let (mut central, rx) = FakeCentral::new();
    central.connects.clear();
    let (outcome, _) = run_connect(&mut central, &rx, Duration::from_millis(80));
    assert!(matches!(outcome, Err(BleError::Timeout)));
    assert_eq!(central.calls.last(), Some(&"cancel"));
}

#[test]
fn bluetooth_off_or_refused_ends_the_setup_with_its_code() {
    for (state, code) in [
        (manager_state::POWERED_OFF, "ble-adapter-off"),
        (manager_state::UNAUTHORIZED, "ble-unauthorized"),
        (manager_state::UNSUPPORTED, "ble-no-adapter"),
    ] {
        let (mut central, rx) = FakeCentral::new();
        central.state = state;
        let (outcome, _) = run_connect(&mut central, &rx, Duration::from_secs(2));
        assert_eq!(outcome.unwrap_err().code(), code);
        assert!(central.calls.is_empty(), "nothing is tried");
    }
}

#[test]
fn the_state_settles_through_events() {
    let (mut central, rx) = FakeCentral::new();
    central.state = manager_state::UNKNOWN;
    central
        .tx
        .send(Event::State(manager_state::RESETTING))
        .unwrap();
    central
        .tx
        .send(Event::State(manager_state::POWERED_ON))
        .unwrap();
    wait_powered(&mut central, &rx, soon(Duration::from_millis(100))).unwrap();

    let (mut central, rx) = FakeCentral::new();
    central.state = manager_state::UNKNOWN;
    let e = wait_powered(&mut central, &rx, soon(Duration::from_millis(30))).unwrap_err();
    assert!(matches!(e, BleError::Timeout));
}

#[test]
fn a_disconnection_during_setup_fails_it() {
    let (mut central, rx) = FakeCentral::new();
    // The link connects, then drops before the services are discovered.
    central.connects.clear();
    central.tx.send(Event::Connected).unwrap();
    central
        .tx
        .send(Event::Disconnected(Some(cb_error(7))))
        .unwrap();
    let (outcome, _) = run_connect(&mut central, &rx, Duration::from_secs(1));
    assert!(matches!(outcome, Err(BleError::Disconnected)));
    assert_eq!(central.count("discover_characteristics"), 0);
}

#[test]
fn a_device_without_the_serial_service_is_refused() {
    let (mut central, rx) = FakeCentral::new();
    central.has_service = false;
    let (outcome, _) = run_connect(&mut central, &rx, Duration::from_secs(1));
    assert!(matches!(outcome, Err(BleError::Platform(m)) if m.contains("serial service")));

    let (mut central, rx) = FakeCentral::new();
    central.notify_props = None;
    let (outcome, _) = run_connect(&mut central, &rx, Duration::from_secs(1));
    assert!(matches!(outcome, Err(BleError::Platform(m)) if m.contains("notify")));

    let (mut central, rx) = FakeCentral::new();
    central.notify_props = Some(props::WRITE);
    let (outcome, _) = run_connect(&mut central, &rx, Duration::from_secs(1));
    assert!(matches!(outcome, Err(BleError::Platform(_))));
    assert_eq!(central.count("notify_on"), 0);
}

#[test]
fn an_early_notification_is_kept_for_the_link() {
    let (mut central, rx) = FakeCentral::new();
    central.early_value = true;
    let (outcome, inbox) = run_connect(&mut central, &rx, Duration::from_secs(1));
    outcome.unwrap();
    assert_eq!(inbox, [b"+OK;".to_vec()]);
}

#[test]
fn a_write_only_with_response_characteristic_is_written_with_response() {
    let (mut central, rx) = FakeCentral::new();
    central.write_props = Some(props::WRITE);
    let (outcome, _) = run_connect(&mut central, &rx, Duration::from_secs(1));
    assert_eq!(outcome.unwrap(), WriteMode::WithResponse);
}

// -------------------------------------------------------------------- link

#[derive(Default)]
struct WireLog {
    sent: Mutex<Vec<(Vec<u8>, WriteMode)>>,
    shutdowns: AtomicUsize,
    /// Writes without response the peripheral takes before it is full.
    credits: AtomicUsize,
}

#[derive(Clone)]
struct FakeWire(Arc<WireLog>);

impl Wire for FakeWire {
    fn can_send(&self) -> bool {
        self.0.credits.load(Ordering::SeqCst) > 0
    }

    fn send(&self, chunk: &[u8], mode: WriteMode) {
        if mode == WriteMode::WithoutResponse {
            self.0.credits.fetch_sub(1, Ordering::SeqCst);
        }
        self.0.sent.lock().unwrap().push((chunk.to_vec(), mode));
    }

    fn shutdown(&self) {
        self.0.shutdowns.fetch_add(1, Ordering::SeqCst);
    }
}

fn fake_link(mode: WriteMode, credits: usize) -> (Link<FakeWire>, Arc<WireLog>, Sender<Event>) {
    let log = Arc::new(WireLog::default());
    log.credits.store(credits, Ordering::SeqCst);
    let (tx, rx) = mpsc::channel();
    let link = Link::new(
        FakeWire(log.clone()),
        rx,
        VecDeque::new(),
        mode,
        20,
        LinkState::new(),
        FAST,
    );
    (link, log, tx)
}

#[test]
fn writes_go_out_in_20_byte_chunks_without_response() {
    let (mut link, log, _tx) = fake_link(WriteMode::WithoutResponse, 100);
    link.write(b"+CT,TURNANGLE=-360.000000;").unwrap();
    let sent = log.sent.lock().unwrap();
    assert_eq!(sent.len(), 2);
    assert_eq!(sent[0].0, b"+CT,TURNANGLE=-360.0");
    assert_eq!(sent[1].0, b"00000;");
    assert!(
        sent.iter()
            .all(|(c, m)| c.len() <= 20 && *m == WriteMode::WithoutResponse)
    );
}

#[test]
fn writes_wait_until_the_peripheral_has_room() {
    let (mut link, log, tx) = fake_link(WriteMode::WithoutResponse, 1);
    let feeder = {
        let log = log.clone();
        thread::spawn(move || {
            for _ in 0..2 {
                thread::sleep(Duration::from_millis(20));
                log.credits.fetch_add(1, Ordering::SeqCst);
                let _ = tx.send(Event::ReadyToSend);
            }
            tx
        })
    };
    let started = Instant::now();
    link.write(&[7u8; 45]).unwrap();
    assert!(started.elapsed() >= Duration::from_millis(35));
    let sent = log.sent.lock().unwrap();
    assert_eq!(
        sent.iter().map(|(c, _)| c.len()).collect::<Vec<_>>(),
        [20, 20, 5]
    );
    drop(sent);
    drop(feeder.join().unwrap());
}

#[test]
fn a_peripheral_that_never_has_room_times_out() {
    let (mut link, log, _tx) = fake_link(WriteMode::WithoutResponse, 0);
    assert!(matches!(link.write(b"+CT,STOP;"), Err(BleError::Timeout)));
    assert!(log.sent.lock().unwrap().is_empty());
    assert!(
        link.state().is_connected(),
        "a slow peripheral is not a lost one"
    );
}

#[test]
fn writes_with_response_wait_for_the_answer() {
    let (mut link, log, tx) = fake_link(WriteMode::WithResponse, 0);
    tx.send(Event::Written(None)).unwrap();
    tx.send(Event::Written(None)).unwrap();
    link.write(&[1u8; 30]).unwrap();
    assert_eq!(log.sent.lock().unwrap().len(), 2);

    tx.send(Event::Written(Some(cb_error(7)))).unwrap();
    assert!(matches!(link.write(b"x"), Err(BleError::Disconnected)));
    assert!(!link.state().is_connected());

    let (mut link, _log, _tx) = fake_link(WriteMode::WithResponse, 0);
    assert!(matches!(link.write(b"x"), Err(BleError::Timeout)));
}

#[test]
fn notifications_arrive_in_order_after_the_early_ones() {
    let log = Arc::new(WireLog::default());
    let (tx, rx) = mpsc::channel();
    let mut link = Link::new(
        FakeWire(log),
        rx,
        VecDeque::from([b"early".to_vec()]),
        WriteMode::WithoutResponse,
        20,
        LinkState::new(),
        FAST,
    );
    tx.send(Event::ReadyToSend).unwrap();
    tx.send(Event::Value(b"+OK;".to_vec())).unwrap();
    assert_eq!(
        link.recv(Duration::from_millis(50)).unwrap(),
        Some(b"early".to_vec())
    );
    assert_eq!(
        link.recv(Duration::from_millis(50)).unwrap(),
        Some(b"+OK;".to_vec())
    );
    assert_eq!(link.recv(Duration::from_millis(20)).unwrap(), None);
}

#[test]
fn a_disconnection_is_reported_and_marks_the_state() {
    let (mut link, log, tx) = fake_link(WriteMode::WithoutResponse, 10);
    let observer = link.state();
    tx.send(Event::Value(b"last".to_vec())).unwrap();
    tx.send(Event::Disconnected(Some(cb_error(7)))).unwrap();
    assert_eq!(
        link.recv(Duration::from_millis(50)).unwrap(),
        Some(b"last".to_vec())
    );
    assert!(matches!(
        link.recv(Duration::from_millis(50)),
        Err(BleError::Disconnected)
    ));
    assert!(!observer.is_connected());
    // And keeps being reported, without touching the wire.
    assert!(matches!(
        link.recv(Duration::ZERO),
        Err(BleError::Disconnected)
    ));
    assert!(matches!(link.write(b"x"), Err(BleError::Disconnected)));
    assert!(log.sent.lock().unwrap().is_empty());
}

#[test]
fn bluetooth_going_off_drops_the_link() {
    let (mut link, _log, tx) = fake_link(WriteMode::WithoutResponse, 0);
    tx.send(Event::State(manager_state::POWERED_OFF)).unwrap();
    assert!(matches!(link.write(b"x"), Err(BleError::Disconnected)));
    assert!(!link.state().is_connected());
}

#[test]
fn a_vanished_delegate_counts_as_a_lost_link() {
    let (mut link, _log, tx) = fake_link(WriteMode::WithoutResponse, 0);
    drop(tx);
    assert!(matches!(
        link.recv(Duration::from_millis(50)),
        Err(BleError::Disconnected)
    ));
    assert!(!link.state().is_connected());
}

#[test]
fn close_disconnects_once_and_waits_for_the_confirmation() {
    let (mut link, log, tx) = fake_link(WriteMode::WithoutResponse, 10);
    let observer = link.state();
    tx.send(Event::NotifyState(None)).unwrap();
    tx.send(Event::Disconnected(None)).unwrap();
    let started = Instant::now();
    link.close();
    assert!(
        started.elapsed() < FAST.close,
        "the confirmation ends the wait"
    );
    assert!(!observer.is_connected());
    link.close();
    assert_eq!(log.shutdowns.load(Ordering::SeqCst), 1);
    assert!(matches!(
        link.recv(Duration::ZERO),
        Err(BleError::Disconnected)
    ));
    assert!(matches!(link.write(b"x"), Err(BleError::Disconnected)));
    drop(link);
    assert_eq!(log.shutdowns.load(Ordering::SeqCst), 1);
}

#[test]
fn close_without_a_confirmation_gives_up_after_its_limit() {
    let (mut link, log, _tx) = fake_link(WriteMode::WithoutResponse, 10);
    let started = Instant::now();
    link.close();
    assert!(started.elapsed() >= FAST.close);
    assert_eq!(log.shutdowns.load(Ordering::SeqCst), 1);
}

#[test]
fn dropping_a_link_disconnects_it() {
    let (link, log, tx) = fake_link(WriteMode::WithoutResponse, 10);
    let observer = link.state();
    tx.send(Event::Disconnected(None)).unwrap();
    drop(link);
    assert_eq!(log.shutdowns.load(Ordering::SeqCst), 1);
    assert!(!observer.is_connected());
}
