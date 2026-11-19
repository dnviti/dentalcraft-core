//! Protocol tests against a scripted [`BleLink`] that plays the table.

use std::collections::VecDeque;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use dc_ble::{BleError, BleLink, LinkState};

use super::*;

/// The table as the fake link plays it.
#[derive(Debug, Default)]
struct Table {
    inbox: Vec<u8>,
    outbox: VecDeque<Vec<u8>>,
    writes: Vec<(Instant, String)>,
    /// Rotation since power-on; reported modulo 360 like the firmware.
    angle: f32,
    target: f32,
    /// Degrees covered between two angle queries.
    step: f32,
    tilt: f32,
    silent: bool,
    fail_prefix: Option<&'static str>,
    /// Send `+OK,TURNANGLE=<deg>` (no `;`, like the real firmware) when a
    /// rotation arrives.
    notify_completion: bool,
    /// `+CT,TOZERO;` unwinds every turn instead of taking the short way.
    tozero_unwinds: bool,
    closed: bool,
}

/// What the firmware reports for the rotation `angle`: modulo 360, in
/// (−360, 0].
fn reported(angle: f32) -> f32 {
    angle - 360.0 * (angle / 360.0).ceil()
}

impl Table {
    fn reply(&mut self, text: &str) {
        self.outbox.push_back(text.as_bytes().to_vec());
    }

    fn respond(&mut self, cmd: &str) {
        self.writes.push((Instant::now(), format!("{cmd};")));
        if self.silent {
            return;
        }
        if self.fail_prefix.is_some_and(|p| cmd.starts_with(p)) {
            self.reply("+FAIL,ERR=3;");
            return;
        }
        let value = |prefix: &str| -> Option<f32> { cmd.strip_prefix(prefix)?.parse().ok() };
        if cmd == "+QT,CHANGEANGLE" {
            let d = self.target - self.angle;
            let was_moving = d != 0.0;
            self.angle += d.clamp(-self.step, self.step);
            // the answer arrives split over two notifications
            let text = format!("+DATA={:.1};", reported(self.angle));
            let (a, b) = text.split_at(4);
            self.reply(a);
            self.reply(b);
            if was_moving && self.angle == self.target && self.notify_completion {
                let text = format!("+OK,TURNANGLE={:.2}", reported(self.angle));
                self.reply(&text);
            }
        } else if cmd == "+QR,TILTANGLE" {
            self.reply("+DATA=-30.0,30.0,0.3;");
        } else if cmd == "+QR,TILTVALUE" {
            let text = format!("+DATA={:.1};", self.tilt);
            self.reply(&text);
        } else if let Some(v) = value("+CT,TURNANGLE=") {
            self.target += v;
            self.reply("+OK;");
        } else if let Some(v) = value("+CR,TILTVALUE=") {
            self.tilt = v;
            self.reply("+OK;");
        } else if cmd == "+CT,STOP" {
            self.target = self.angle;
            self.reply("+OK;");
        } else if cmd == "+CT,TOZERO" {
            self.target = if self.tozero_unwinds {
                0.0
            } else {
                360.0 * (self.angle / 360.0).round()
            };
            self.reply("+OK;");
        } else {
            self.reply("+OK;");
        }
    }
}

struct FakeLink {
    table: Arc<Mutex<Table>>,
    state: LinkState,
}

impl FakeLink {
    fn boxed(table: &Arc<Mutex<Table>>) -> Box<Self> {
        Box::new(Self {
            table: Arc::clone(table),
            state: LinkState::new(),
        })
    }
}

impl BleLink for FakeLink {
    fn write(&mut self, data: &[u8]) -> std::result::Result<(), BleError> {
        let mut t = self.table.lock().unwrap();
        if t.closed || !self.state.is_connected() {
            return Err(BleError::Disconnected);
        }
        t.inbox.extend_from_slice(data);
        while let Some(end) = t.inbox.iter().position(|&b| b == b';') {
            let cmd: Vec<u8> = t.inbox.drain(..=end).collect();
            let cmd = String::from_utf8(cmd[..cmd.len() - 1].to_vec()).unwrap();
            t.respond(&cmd);
        }
        Ok(())
    }

    fn recv(&mut self, timeout: Duration) -> std::result::Result<Option<Vec<u8>>, BleError> {
        if !self.state.is_connected() {
            return Err(BleError::Disconnected);
        }
        if let Some(m) = self.table.lock().unwrap().outbox.pop_front() {
            return Ok(Some(m));
        }
        std::thread::sleep(timeout.min(Duration::from_millis(1)));
        Ok(self.table.lock().unwrap().outbox.pop_front())
    }

    fn close(&mut self) {
        self.state.mark_lost();
        self.table.lock().unwrap().closed = true;
    }

    fn state(&self) -> LinkState {
        self.state.clone()
    }
}

fn timing() -> Timing {
    Timing {
        command_gap: Duration::from_millis(5),
        reply_timeout: Duration::from_millis(100),
        poll_interval: Duration::from_millis(2),
        settle_epsilon_deg: 0.05,
        tilt_margin: Duration::from_millis(10),
    }
}

fn info() -> TurntableInfo {
    TurntableInfo {
        backend: BACKEND_ID.into(),
        id: "AA:BB:CC:DD:EE:FF".into(),
        name: DEVICE_NAME.into(),
    }
}

fn open(table: Table) -> (RevopointTurntable, Arc<Mutex<Table>>) {
    let shared = Arc::new(Mutex::new(table));
    let link = FakeLink::boxed(&shared);
    (
        RevopointTurntable::new(link, info(), timing()).unwrap(),
        shared,
    )
}

fn sent(t: &Arc<Mutex<Table>>) -> Vec<String> {
    t.lock()
        .unwrap()
        .writes
        .iter()
        .map(|w| w.1.clone())
        .collect()
}

fn never() -> bool {
    false
}

#[test]
fn profile_name_and_command_strings_are_exact() {
    let p = profile();
    assert_eq!(
        p.service.to_string(),
        "0000ffe0-0000-1000-8000-00805f9b34fb"
    );
    assert_eq!(
        p.write_char.to_string(),
        "0000ffe1-0000-1000-8000-00805f9b34fb"
    );
    assert_eq!(p.notify_char, p.write_char);
    assert_eq!(p.max_chunk, 20);
    assert!(name_matches("REVO_DUAL_AXIS_TABLE"));
    assert!(!name_matches("REVO_TA500"));
    assert_eq!(cmd::turn_angle(-360.0), "+CT,TURNANGLE=-360.000000;");
    assert_eq!(cmd::turn_angle(45.5), "+CT,TURNANGLE=45.500000;");
    assert_eq!(cmd::turn_continue(1), "+CT,TURNCONTINUE=1;");
    assert_eq!(cmd::turn_continue(-1), "+CT,TURNCONTINUE=-1;");
    assert_eq!(cmd::turn_speed(36), "+CT,TURNSPEED=36;");
    assert_eq!(cmd::tilt_value(-30.0), "+CR,TILTVALUE=-30.000000;");
    assert_eq!(cmd::tilt_speed(10), "+CR,TILTSPEED=10;");
    assert_eq!(cmd::TURN_STOP, "+CT,STOP;");
    assert_eq!(cmd::TURN_TO_ZERO, "+CT,TOZERO;");
    assert_eq!(cmd::TILT_STOP, "+CR,STOP;");
    assert_eq!(cmd::TILT_TO_ZERO, "+CR,TOZERO;");
    assert_eq!(cmd::Q_CHANGE_ANGLE, "+QT,CHANGEANGLE;");
    assert_eq!(cmd::Q_TILT_VALUE, "+QR,TILTVALUE;");
    assert_eq!(cmd::Q_TILT_ANGLE, "+QR,TILTANGLE;");
    assert_eq!(cmd::Q_TURN_SPEED, "+QT,TURNSPEED;");
    assert_eq!(cmd::Q_TILT_SPEED, "+QR,TILTSPEED;");
}

#[test]
fn speed_mapping_is_seconds_per_turn() {
    assert_eq!(turn_speed_value(10.0), 36);
    assert_eq!(turn_speed_value(20.0), 18);
    assert_eq!(turn_speed_value(100.0), 18);
    assert_eq!(turn_speed_value(1.0), 90);
    assert_eq!(turn_speed_value(4.0), 90);
    // the firmware takes only multiples of 5 from 40 to 90 (measured)
    assert_eq!(tilt_speed_value(6.0), 55);
    assert_eq!(tilt_speed_value(6.7), 60);
    assert_eq!(tilt_speed_value(100.0), 90);
    assert_eq!(tilt_speed_value(0.1), 40);
    assert!((tilt_speed_deg_per_s(60) - 6.67).abs() < 0.01);
}

#[test]
fn replies_parse_and_reassemble() {
    assert_eq!(parse_reply("+OK"), Reply::Ok(None));
    assert_eq!(
        parse_reply("+OK,TURNANGLE=90.0"),
        Reply::Ok(Some("TURNANGLE=90.0".into()))
    );
    assert_eq!(parse_reply("+DATA=-134.9"), Reply::Data(vec![-134.9]));
    assert_eq!(
        parse_reply("+DATA=-30.0,30.0,0.3"),
        Reply::Data(vec![-30.0, 30.0, 0.3])
    );
    assert_eq!(parse_reply("+FAIL,ERR=2"), Reply::Fail("FAIL,ERR=2".into()));
    assert_eq!(parse_reply("+DATA=x"), Reply::Other("+DATA=x".into()));
    assert_eq!(parse_reply("hello"), Reply::Other("hello".into()));
    let mut b = ReplyBuffer::default();
    assert!(b.push(b"+DA").is_empty());
    assert_eq!(
        b.push(b"TA=-134.9;+OK;\r\n+QT"),
        [Reply::Data(vec![-134.9]), Reply::Ok(None)]
    );
    assert_eq!(b.push(b"X;"), [Reply::Other("+QTX".into())]);
    assert!(b.push(b";\0;").is_empty());
    // the firmware's completion notification has no `;` (seen live)
    assert!(b.push(b"+OK,TURNANGLE=-171.05").is_empty());
    assert_eq!(
        b.push(b"+DATA=-171.0;"),
        [
            Reply::Ok(Some("TURNANGLE=-171.05".into())),
            Reply::Data(vec![-171.0])
        ]
    );
    assert!(b.push(b"+OK,TURNANGLE=-0.02").is_empty());
    assert_eq!(b.pending(), b"+OK,TURNANGLE=-0.02");
    let done = b.flush().unwrap();
    assert_eq!(completion_angle(&done), Some(-0.02));
    assert_eq!(b.flush(), None);
    assert_eq!(completion_angle(&Reply::Ok(Some("TOZERO".into()))), None);
    assert_eq!(completion_angle(&Reply::Ok(None)), None);
}

#[test]
fn connect_reads_limits_and_angle_and_refuses_a_silent_device() {
    let (t, table) = open(Table {
        angle: -12.5,
        target: -12.5,
        step: 10.0,
        ..Table::default()
    });
    assert_eq!(t.tilt_limits(), (-30.0, 30.0));
    // the tilt value query does not follow the tilt: not asked
    assert_eq!(sent(&table), ["+QR,TILTANGLE;", "+QT,CHANGEANGLE;"]);
    let a = t.probe().unwrap()();
    assert_eq!(
        (a.rotate_deg, a.tilt_deg, a.moving),
        (Some(-12.5), Some(0.0), false)
    );
    let tilt = &t.axes()[1];
    assert_eq!((tilt.min_deg, tilt.max_deg), (Some(-30.0), Some(30.0)));
    assert_eq!(tilt.feedback, Feedback::None);
    assert_eq!(t.axes()[0].feedback, Feedback::Notified);

    let silent = Arc::new(Mutex::new(Table {
        silent: true,
        ..Table::default()
    }));
    let err = RevopointTurntable::new(FakeLink::boxed(&silent), info(), timing()).unwrap_err();
    assert_eq!(err.code(), "turntable-protocol");
}

#[test]
fn moves_return_on_the_ack_and_settle_on_the_completion_notification() {
    let (mut t, table) = open(Table {
        step: 30.0,
        notify_completion: true,
        ..Table::default()
    });
    t.move_by(AxisKind::Rotate, -90.0).unwrap();
    // acknowledged, not done: the table has not turned yet
    assert_eq!(table.lock().unwrap().angle, 0.0);
    assert_eq!(sent(&table).last().unwrap(), "+CT,TURNANGLE=-90.000000;");
    assert!(t.probe().unwrap()().moving);
    let before = sent(&table).len();
    t.wait_settled(AxisKind::Rotate, Duration::from_secs(2), &never)
        .unwrap();
    assert_eq!(table.lock().unwrap().angle, -90.0);
    // -30, -60, -90 and the unterminated completion: no fourth poll
    let polls = sent(&table)[before..]
        .iter()
        .filter(|c| *c == "+QT,CHANGEANGLE;")
        .count();
    assert_eq!(polls, 3);
    assert_eq!(t.position(AxisKind::Rotate).unwrap(), Some(-90.0));
    // the completion notification is kept, not mistaken for an
    // acknowledgement
    assert!(
        t.unsolicited()
            .any(|r| *r == Reply::Ok(Some("TURNANGLE=-90.00".into())))
    );
    // writes keep the command gap
    let w = &table.lock().unwrap().writes;
    for pair in w.windows(2) {
        assert!(pair[1].0 - pair[0].0 >= Duration::from_millis(5));
    }
}

#[test]
fn move_to_rotate_is_relative_to_the_reading() {
    let (mut t, table) = open(Table {
        angle: 100.0,
        target: 100.0,
        step: 1000.0,
        ..Table::default()
    });
    t.move_to(AxisKind::Rotate, 40.0).unwrap();
    assert_eq!(sent(&table).last().unwrap(), "+CT,TURNANGLE=-60.000000;");
    t.wait_settled(AxisKind::Rotate, Duration::from_secs(1), &never)
        .unwrap();
    assert_eq!(table.lock().unwrap().angle, 40.0);
    t.home().unwrap();
    assert_eq!(
        sent(&table)[sent(&table).len() - 2..],
        ["+CT,TOZERO;", "+CR,TOZERO;"]
    );
    t.wait_settled(AxisKind::Rotate, Duration::from_secs(1), &never)
        .unwrap();
    assert_eq!(table.lock().unwrap().angle, 0.0);
}

#[test]
fn a_cancelled_or_stalled_wait_reports_it() {
    let (mut t, table) = open(Table {
        step: 1.0,
        ..Table::default()
    });
    t.move_by(AxisKind::Rotate, 360.0).unwrap();
    let err = t
        .wait_settled(AxisKind::Rotate, Duration::from_secs(5), &|| true)
        .unwrap_err();
    assert_eq!(err.code(), "cancelled");
    let err = t
        .wait_settled(AxisKind::Rotate, Duration::from_millis(20), &never)
        .unwrap_err();
    assert_eq!(err.code(), "turntable-timeout");
    t.stop().unwrap();
    let s = sent(&table);
    assert!(s.iter().any(|c| c == "+CT,STOP;") && s.iter().any(|c| c == "+CR,STOP;"));
    // stopped: the next wait ends on two equal readings
    t.wait_settled(AxisKind::Rotate, Duration::from_secs(1), &never)
        .unwrap();
}

#[test]
fn tilt_is_open_loop_and_range_checked() {
    let (mut t, table) = open(Table::default());
    t.set_speed(AxisKind::Tilt, 100.0).unwrap();
    assert_eq!(sent(&table).last().unwrap(), "+CR,TILTSPEED=90;");
    let n = sent(&table).len();
    assert_eq!(
        t.move_to(AxisKind::Tilt, 31.0).unwrap_err().code(),
        "turntable-out-of-range"
    );
    assert_eq!(sent(&table).len(), n, "nothing sent for a refused tilt");
    t.move_to(AxisKind::Tilt, 3.0).unwrap();
    assert_eq!(sent(&table).last().unwrap(), "+CR,TILTVALUE=3.000000;");
    // 3 degrees at 90 / 9 = 10 deg/s: about 0.3 s, then the margin
    let start = Instant::now();
    t.wait_settled(AxisKind::Tilt, Duration::from_secs(5), &never)
        .unwrap();
    let took = start.elapsed();
    assert!(took >= Duration::from_millis(300), "{took:?}");
    assert_eq!(t.position(AxisKind::Tilt).unwrap(), Some(3.0));
    t.move_by(AxisKind::Tilt, -3.0).unwrap();
    assert_eq!(sent(&table).last().unwrap(), "+CR,TILTVALUE=0.000000;");
    let err = t
        .wait_settled(AxisKind::Tilt, Duration::from_millis(1), &never)
        .unwrap_err();
    assert_eq!(err.code(), "turntable-timeout");
}

#[test]
fn refusals_and_lost_links_have_codes() {
    let (mut t, table) = open(Table {
        fail_prefix: Some("+CT,TURNSPEED"),
        ..Table::default()
    });
    assert_eq!(
        t.set_speed(AxisKind::Rotate, 10.0).unwrap_err().code(),
        "turntable-refused"
    );
    assert_eq!(
        t.set_speed(AxisKind::Rotate, -1.0).unwrap_err().code(),
        "invalid-params"
    );
    table.lock().unwrap().closed = true;
    assert_eq!(
        t.move_by(AxisKind::Rotate, 10.0).unwrap_err().code(),
        "ble-disconnected"
    );
    let raw = {
        table.lock().unwrap().closed = false;
        t.exchange_raw(cmd::Q_TURN_SPEED).unwrap()
    };
    assert_eq!(raw, [Reply::Ok(None)]);
}

/// Moves `deg`, waits for the end and returns the position read then.
fn turn(t: &mut RevopointTurntable, deg: f32) -> f32 {
    t.move_by(AxisKind::Rotate, deg).unwrap();
    t.wait_settled(AxisKind::Rotate, Duration::from_secs(5), &never)
        .unwrap();
    t.position(AxisKind::Rotate).unwrap().unwrap()
}

#[test]
fn wrapped_readings_make_a_continuous_angle() {
    assert_eq!(unwrap_deg(-350.0, 0.0), 10.0);
    assert_eq!(unwrap_deg(-10.0, -5.0), -10.0);
    assert_eq!(unwrap_deg(0.0, 715.0), 720.0);
    assert_eq!(unwrap_deg(-10.0, -365.0), -370.0);
    let (mut t, table) = open(Table {
        step: 4.0,
        notify_completion: true,
        ..Table::default()
    });
    // +10 from the power-on zero: the table reads -350.0 and its
    // completion says so too
    assert_eq!(turn(&mut t, 10.0), 10.0);
    assert!(
        t.unsolicited()
            .any(|r| *r == Reply::Ok(Some("TURNANGLE=-350.00".into())))
    );
    assert_eq!(t.probe().unwrap()().rotate_deg, Some(10.0));
    // back across the zero
    assert_eq!(turn(&mut t, -20.0), -10.0);
    assert_eq!(turn(&mut t, 15.0), 5.0);
    // whole turns, both ways
    table.lock().unwrap().step = 30.0;
    assert_eq!(turn(&mut t, 720.0), 725.0);
    assert_eq!(table.lock().unwrap().angle, 725.0);
    assert_eq!(turn(&mut t, -1095.0), -370.0);
    assert_eq!(table.lock().unwrap().angle, -370.0);
    assert_eq!(t.probe().unwrap()().rotate_deg, Some(-370.0));
    // an absolute move counts in the same continuous angle
    t.move_to(AxisKind::Rotate, -400.0).unwrap();
    assert_eq!(sent(&table).last().unwrap(), "+CT,TURNANGLE=-30.000000;");
}

#[test]
fn home_from_a_wrapped_angle_takes_the_closest_turn() {
    // connected 10 degrees short of a whole turn: the table reads -10.0
    let (mut t, table) = open(Table {
        angle: 350.0,
        target: 350.0,
        step: 30.0,
        notify_completion: true,
        ..Table::default()
    });
    assert_eq!(t.position(AxisKind::Rotate).unwrap(), Some(-10.0));
    t.home().unwrap();
    t.wait_settled(AxisKind::Rotate, Duration::from_secs(2), &never)
        .unwrap();
    // forward 10 degrees to the zero, not 350 back
    assert_eq!(table.lock().unwrap().angle, 360.0);
    assert_eq!(t.position(AxisKind::Rotate).unwrap(), Some(0.0));
    // from more than half a turn, on to the next zero: still continuous
    assert_eq!(turn(&mut t, 190.0), 190.0);
    t.home().unwrap();
    assert!(t.probe().unwrap()().moving);
    t.wait_settled(AxisKind::Rotate, Duration::from_secs(2), &never)
        .unwrap();
    assert_eq!(table.lock().unwrap().angle, 720.0);
    assert_eq!(t.position(AxisKind::Rotate).unwrap(), Some(360.0));
}

#[test]
fn home_follows_the_table_whichever_way_it_turns() {
    // a firmware that unwinds every turn: 190 degrees back, not 170 on
    let (mut t, table) = open(Table {
        step: 30.0,
        notify_completion: true,
        tozero_unwinds: true,
        ..Table::default()
    });
    assert_eq!(turn(&mut t, 190.0), 190.0);
    t.home().unwrap();
    t.wait_settled(AxisKind::Rotate, Duration::from_secs(2), &never)
        .unwrap();
    assert_eq!(table.lock().unwrap().angle, 0.0);
    assert_eq!(t.position(AxisKind::Rotate).unwrap(), Some(0.0));
}

/// A connector that opens a new fake link to `table`, after `failures`
/// failed attempts; counts the attempts.
fn connector(table: &Arc<Mutex<Table>>, failures: u32) -> (Connector, Arc<Mutex<u32>>) {
    let attempts = Arc::new(Mutex::new(0));
    let (table, counted) = (Arc::clone(table), Arc::clone(&attempts));
    let connect: Connector = Box::new(move |_timeout| {
        let mut n = counted.lock().unwrap();
        *n += 1;
        if *n <= failures {
            return Err(BleError::NotFound);
        }
        table.lock().unwrap().closed = false;
        Ok(FakeLink::boxed(&table) as Box<dyn BleLink>)
    });
    (connect, attempts)
}

#[test]
fn a_reconnection_keeps_the_angle_continuous_and_the_speeds() {
    let table = Arc::new(Mutex::new(Table {
        step: 30.0,
        notify_completion: true,
        ..Table::default()
    }));
    let first = FakeLink::boxed(&table);
    let radio = first.state();
    let (connect, attempts) = connector(&table, 1);
    let mut t = RevopointTurntable::new(first, info(), timing())
        .unwrap()
        .with_connector(connect);
    let link = t.link().unwrap();
    t.set_speed(AxisKind::Rotate, 20.0).unwrap();
    t.set_speed(AxisKind::Tilt, 6.7).unwrap();
    // almost two turns: the table reads -20
    assert_eq!(turn(&mut t, 700.0), 700.0);
    // the radio drops during a move: the state shows it before any read
    t.move_by(AxisKind::Rotate, 20.0).unwrap();
    assert!(link());
    radio.mark_lost();
    assert!(!link());
    assert_eq!(
        t.position(AxisKind::Rotate).unwrap_err().code(),
        "ble-disconnected"
    );
    let before = sent(&table).len();
    t.reconnect(Duration::from_secs(10), &never).unwrap();
    assert!(link());
    assert_eq!(*attempts.lock().unwrap(), 2, "one failed attempt, then up");
    let resent = sent(&table)[before..].to_vec();
    for speed in ["+CT,TURNSPEED=18;", "+CR,TILTSPEED=60;"] {
        assert!(resent.iter().any(|s| s == speed), "{resent:?}");
    }
    // the move finished meanwhile: 720, the reading 0.0 on the same turn,
    // not mapped back to 0
    t.wait_settled(AxisKind::Rotate, Duration::from_secs(5), &never)
        .unwrap();
    assert_eq!(t.position(AxisKind::Rotate).unwrap(), Some(720.0));
    assert_eq!(t.probe().unwrap()().rotate_deg, Some(720.0));
    assert_eq!(turn(&mut t, 10.0), 730.0);
    assert_eq!(table.lock().unwrap().angle, 730.0);
}

#[test]
fn a_reconnection_gives_up_at_its_deadline_or_on_cancel() {
    let table = Arc::new(Mutex::new(Table::default()));
    let (connect, attempts) = connector(&table, u32::MAX);
    let mut t = RevopointTurntable::new(FakeLink::boxed(&table), info(), timing())
        .unwrap()
        .with_connector(connect);
    let link = t.link().unwrap();
    let start = Instant::now();
    let err = t.reconnect(Duration::from_secs(4), &never).unwrap_err();
    assert_eq!(err.code(), "ble-not-found");
    assert!(start.elapsed() < Duration::from_secs(5));
    assert!(*attempts.lock().unwrap() >= 2);
    assert!(!link(), "the old link was closed");
    let err = t.reconnect(Duration::from_secs(30), &|| true).unwrap_err();
    assert_eq!(err.code(), "cancelled");
    // a table opened without a connector cannot reconnect
    let (mut plain, _) = open(Table::default());
    assert_eq!(
        plain
            .reconnect(Duration::from_secs(1), &never)
            .unwrap_err()
            .code(),
        "turntable-not-connected"
    );
}
