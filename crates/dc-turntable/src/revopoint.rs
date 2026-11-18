//! Revopoint Dual-axis Turntable over Bluetooth LE (`revopoint-ble`), alpha.
//!
//! The protocol is not published by Revopoint; this follows the community
//! reverse engineering of the 2022 firmware (opensourcemanufacturing /
//! Revopoint-Dual-Turntable, SphaeroX / Revopoint-Dual-Axis-Turntable,
//! aderusha / ha-revopoint-turntable), checked live on 2026-11-16 against a
//! table advertising as `REVO_DUAL_AXIS_TABLE`:
//!
//! * The table advertises as [`DEVICE_NAME`] and talks ASCII over the HM-10
//!   style serial service ([`profile`]: service `FFE0`, characteristic
//!   `FFE1` for write without response and notify, writes of at most 20
//!   bytes). One central at a time. Connecting takes about 0.6 s.
//! * Commands are `+...;` strings ([`cmd`]). Each one is acknowledged by
//!   `+OK;` (or `+FAIL,ERR=<n>;`) within about 50 ms; the acknowledgement
//!   is not the end of the motion. Commands are spaced by
//!   [`Timing::command_gap`] (150 ms, like the known clients).
//! * Queries answer `+DATA=<v>[,<v>...];` within about 50 ms.
//!   `+QT,CHANGEANGLE;` is the live rotation, 0.1 degree steps, **modulo
//!   360 in (−360, 0]**: its zero is the power-on position, and +10 from
//!   there reads `-350.0`, +30 `-330.0`, −10 `-10.0`. The backend makes it
//!   continuous ([`unwrap_deg`]): each reading is taken on the turn closest
//!   to where the motion estimate expects the table, and the reading at
//!   connect on the turn closest to 0 (in (−180, 180]). The angles are
//!   continuous from the connection on, never absolute: the calibration
//!   and the motion prior only use differences.
//! * When a rotation ends the table notifies `+OK,TURNANGLE=<deg>` **without
//!   the final `;`** (also after `+CT,TOZERO;`), wrapped like the readings
//!   (`+OK,TURNANGLE=-350.02` after +10 from 0). [`ReplyBuffer`] ends such a
//!   message at the next `+` or when nothing more arrives, and
//!   [`Turntable::wait_settled`] returns on it ([`Feedback::Notified`]);
//!   polling `+QT,CHANGEANGLE;` until two readings agree stays the
//!   fallback. A `+OK,TOZERO;` was also seen about 1.3 s after a tilt to 0.
//! * `+CT,TOZERO;` goes the short way to the zero: from +10 it turned back
//!   in about a second, and from +189 its 8.3 s at 21.4 deg/s fit 171
//!   degrees forward rather than 189 back. The home target is the whole
//!   turn closest to the table; the readings during the wait follow the
//!   table whichever way it turns.
//! * The tilt has no feedback: `+QR,TILTVALUE;` answered `0.5` whatever the
//!   commanded tilt, so it is not used and the tilt is assumed 0 at
//!   connect. A tilt is waited for from its expected duration
//!   ([`Feedback::None`]). `+QR,TILTANGLE;` answers `-30.0,30.0,0.3`.
//!
//! Speeds: the table takes `+CT,TURNSPEED=<v>;` read as seconds per
//! revolution (`+QT,TURNSPEED;` answers `17.5,90.0,2.0`: min, max, step),
//! so `v = 360 / deg_per_s` clamped to 18..90 ([`turn_speed_value`]);
//! live, `v = 18` turned at 21.4 deg/s. The tilt speed unit is not known
//! (`+QR,TILTSPEED;` answers `6.8,90.0,1.0`, but the firmware refuses
//! `+CR,TILTSPEED=` with 7, 8, 10, 15, 20, 30 and 35, `FAIL,ERR=007`, and
//! accepts 40, 45, 50, 55, 60 and 90, measured 2026-11-18); `v = 60`
//! tilted 20 degrees in about 3 s, so the mapping assumes the speed grows
//! with the value, `deg_per_s = v / 9`, `v` a multiple of 5 in 40..90
//! ([`tilt_speed_value`], [`tilt_speed_deg_per_s`]). Both only feed the
//! motion estimate and the timeouts; rotation completion never relies on
//! them.

use std::collections::VecDeque;
use std::sync::{Arc, Mutex, MutexGuard};
use std::time::{Duration, Instant};

use dc_ble::{BleLink, SerialProfile};

use crate::{
    AngleProbe, Angles, AxisInfo, AxisKind, Feedback, Result, Turntable, TurntableBackend,
    TurntableError, TurntableInfo,
};

/// Id of the backend.
pub const BACKEND_ID: &str = "revopoint-ble";
/// Advertised Bluetooth name of the Dual-axis Turntable.
pub const DEVICE_NAME: &str = "REVO_DUAL_AXIS_TABLE";
/// Longest time to connect and resolve the GATT services.
pub const CONNECT_TIMEOUT: Duration = Duration::from_secs(20);
/// Range of the rotation speed value (seconds per revolution).
pub const TURN_SPEED_RANGE: (f32, f32) = (18.0, 90.0);
/// Range of the tilt speed value accepted by the firmware.
pub const TILT_SPEED_RANGE: (f32, f32) = (40.0, 90.0);
/// Tilt range used when the table does not answer `+QR,TILTANGLE;`.
pub const DEFAULT_TILT_LIMITS: (f32, f32) = (-30.0, 30.0);
/// Rotation speed assumed until one is set: value 36, what Revo Scan sends.
const DEFAULT_TURN_DEG_PER_S: f32 = 10.0;
/// Tilt speed assumed until one is set.
const DEFAULT_TILT_DEG_PER_S: f32 = 6.0;
/// A rotation reading within this of the target counts as arrived.
const ARRIVED_DEG: f32 = 0.5;
/// After the expected end of a rotation, two equal readings end the wait
/// even far from the target (a stall, a different firmware).
const OVERDUE: Duration = Duration::from_secs(2);

/// The GATT layout of the table: HM-10 serial service `FFE0`, one
/// characteristic `FFE1` for write and notify, 20-byte writes.
pub fn profile() -> SerialProfile {
    SerialProfile::hm10()
}

/// `true` for the advertised name of the Dual-axis Turntable.
pub fn name_matches(name: &str) -> bool {
    name.trim() == DEVICE_NAME
}

/// Device value of `+CT,TURNSPEED=` for a rotation speed (deg/s), clamped
/// to [`TURN_SPEED_RANGE`].
pub fn turn_speed_value(deg_per_s: f32) -> u32 {
    (360.0 / deg_per_s)
        .clamp(TURN_SPEED_RANGE.0, TURN_SPEED_RANGE.1)
        .round() as u32
}

/// Device value of `+CR,TILTSPEED=` for a tilt speed (deg/s): `9 *
/// deg_per_s` rounded to a multiple of 5 and clamped to
/// [`TILT_SPEED_RANGE`], the values the firmware accepts (see the module
/// doc).
pub fn tilt_speed_value(deg_per_s: f32) -> u32 {
    let v = (9.0 * deg_per_s).clamp(TILT_SPEED_RANGE.0, TILT_SPEED_RANGE.1);
    (5.0 * (v / 5.0).round()) as u32
}

/// Tilt speed (deg/s) of the device value `v` (`v / 9`; `60` measured at
/// about 6.7 deg/s).
pub fn tilt_speed_deg_per_s(v: u32) -> f32 {
    v as f32 / 9.0
}

/// The command strings.
pub mod cmd {
    /// Rotate by `deg` degrees (signed, relative).
    pub fn turn_angle(deg: f32) -> String {
        format!("+CT,TURNANGLE={deg:.6};")
    }
    /// Rotate continuously, `dir` 1 or -1.
    pub fn turn_continue(dir: i8) -> String {
        format!("+CT,TURNCONTINUE={};", if dir < 0 { -1 } else { 1 })
    }
    /// Rotation speed (device value, see [`super::turn_speed_value`]).
    pub fn turn_speed(value: u32) -> String {
        format!("+CT,TURNSPEED={value};")
    }
    /// Stop the rotation.
    pub const TURN_STOP: &str = "+CT,STOP;";
    /// Rotate back to the power-on zero, the short way.
    pub const TURN_TO_ZERO: &str = "+CT,TOZERO;";
    /// Tilt to the absolute angle `deg`.
    pub fn tilt_value(deg: f32) -> String {
        format!("+CR,TILTVALUE={deg:.6};")
    }
    /// Tilt speed (device value, see [`super::tilt_speed_value`]).
    pub fn tilt_speed(value: u32) -> String {
        format!("+CR,TILTSPEED={value};")
    }
    /// Stop the tilt.
    pub const TILT_STOP: &str = "+CR,STOP;";
    /// Tilt back to zero.
    pub const TILT_TO_ZERO: &str = "+CR,TOZERO;";
    /// Query: rotation modulo 360, in (−360, 0], live.
    pub const Q_CHANGE_ANGLE: &str = "+QT,CHANGEANGLE;";
    /// Query: last commanded tilt.
    pub const Q_TILT_VALUE: &str = "+QR,TILTVALUE;";
    /// Query: tilt limits (min, max, step).
    pub const Q_TILT_ANGLE: &str = "+QR,TILTANGLE;";
    /// Query: rotation speed.
    pub const Q_TURN_SPEED: &str = "+QT,TURNSPEED;";
    /// Query: tilt speed.
    pub const Q_TILT_SPEED: &str = "+QR,TILTSPEED;";
}

/// One message from the table (the text between `+` and `;`).
#[derive(Clone, Debug, PartialEq)]
pub enum Reply {
    /// `+OK;` or `+OK,<detail>;`.
    Ok(Option<String>),
    /// `+DATA=<v>,...;`.
    Data(Vec<f32>),
    /// `+FAIL...;`.
    Fail(String),
    /// Anything else (kept verbatim).
    Other(String),
}

/// Parses one message, without its `;`.
pub fn parse_reply(msg: &str) -> Reply {
    let m = msg.trim();
    let body = m.strip_prefix('+').unwrap_or(m);
    if body == "OK" {
        return Reply::Ok(None);
    }
    if let Some(detail) = body.strip_prefix("OK,") {
        return Reply::Ok(Some(detail.trim().to_owned()));
    }
    if body.starts_with("FAIL") {
        return Reply::Fail(body.to_owned());
    }
    if let Some(data) = body.strip_prefix("DATA=") {
        let values: std::result::Result<Vec<f32>, _> =
            data.split(',').map(|v| v.trim().parse::<f32>()).collect();
        if let Ok(values) = values
            && !values.is_empty()
        {
            return Reply::Data(values);
        }
    }
    Reply::Other(m.to_owned())
}

/// Reassembles notifications into messages: a message may span several
/// notifications and one notification may hold several messages. Messages
/// end with `;`, except the completion notifications of the firmware
/// (`+OK,TURNANGLE=-171.05` arrives without it): a notification starting
/// with `+` ends the pending message, and [`ReplyBuffer::flush`] ends it
/// when nothing more arrives.
#[derive(Debug, Default)]
pub struct ReplyBuffer {
    buf: Vec<u8>,
}

impl ReplyBuffer {
    /// Adds received bytes; returns the messages completed by them.
    pub fn push(&mut self, bytes: &[u8]) -> Vec<Reply> {
        let mut out = Vec::new();
        let starts_message = bytes
            .iter()
            .find(|b| !b.is_ascii_whitespace() && **b != 0)
            .is_some_and(|b| *b == b'+');
        if starts_message {
            out.extend(self.flush());
        }
        self.buf.extend_from_slice(bytes);
        while let Some(end) = self.buf.iter().position(|&b| b == b';') {
            let msg: Vec<u8> = self.buf.drain(..=end).collect();
            out.extend(Self::message(&msg[..msg.len() - 1]));
        }
        if self.buf.len() > 4096 {
            // garbage without terminator: drop it rather than grow forever
            self.buf.clear();
        }
        out
    }

    /// Ends the pending unterminated message, if any.
    pub fn flush(&mut self) -> Option<Reply> {
        let msg = std::mem::take(&mut self.buf);
        Self::message(&msg)
    }

    /// Bytes of the message received so far, not yet ended.
    pub fn pending(&self) -> &[u8] {
        &self.buf
    }

    fn message(bytes: &[u8]) -> Option<Reply> {
        let text = String::from_utf8_lossy(bytes);
        let text = text.trim_matches(|c: char| c.is_whitespace() || c == '\0');
        (!text.is_empty()).then(|| parse_reply(text))
    }
}

/// The rotation angle of a completion notification (`+OK,TURNANGLE=<deg>`),
/// as the table sends it (modulo 360, see [`unwrap_deg`]).
pub fn completion_angle(reply: &Reply) -> Option<f32> {
    match reply {
        Reply::Ok(Some(d)) => d.strip_prefix("TURNANGLE=")?.trim().parse().ok(),
        _ => None,
    }
}

/// A rotation angle of the table made continuous: of the angles `raw + 360 k`
/// (the table reports its rotation modulo 360), the one closest to
/// `expected`. Right while the table is less than half a turn from where it
/// is expected.
pub fn unwrap_deg(raw: f32, expected: f32) -> f32 {
    raw + 360.0 * ((expected - raw) / 360.0).round()
}

/// Timings of the protocol.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Timing {
    /// Shortest time between two commands.
    pub command_gap: Duration,
    /// Longest wait for the answer to a command or query.
    pub reply_timeout: Duration,
    /// Time between two position readings while waiting for a rotation.
    pub poll_interval: Duration,
    /// Two readings closer than this (degrees) are equal.
    pub settle_epsilon_deg: f32,
    /// Extra wait after the expected end of a tilt.
    pub tilt_margin: Duration,
}

impl Default for Timing {
    fn default() -> Self {
        Self {
            command_gap: Duration::from_millis(150),
            reply_timeout: Duration::from_millis(1500),
            poll_interval: Duration::from_millis(500),
            settle_epsilon_deg: 0.05,
            tilt_margin: Duration::from_millis(500),
        }
    }
}

/// Constant-speed estimate of one axis.
#[derive(Clone, Copy, Debug)]
struct AxisModel {
    from: f32,
    to: f32,
    start: Instant,
    deg_per_s: f32,
}

impl AxisModel {
    fn at_rest(deg: f32, deg_per_s: f32) -> Self {
        Self {
            from: deg,
            to: deg,
            start: Instant::now(),
            deg_per_s,
        }
    }

    fn at(&self, now: Instant) -> f32 {
        let d = self.to - self.from;
        let travel = self.deg_per_s * now.saturating_duration_since(self.start).as_secs_f32();
        if travel >= d.abs() {
            self.to
        } else {
            self.from + travel.copysign(d)
        }
    }

    fn end(&self) -> Instant {
        self.start + Duration::from_secs_f32((self.to - self.from).abs() / self.deg_per_s)
    }

    /// Starts a motion from the current estimate to `to`.
    fn go(&mut self, to: f32) {
        let now = Instant::now();
        *self = Self {
            from: self.at(now),
            to,
            start: now,
            deg_per_s: self.deg_per_s,
        };
    }

    /// A reading: the axis is at `deg` now, still heading to its target
    /// unless `settled`.
    fn anchor(&mut self, deg: f32, settled: bool) {
        self.from = deg;
        self.start = Instant::now();
        if settled {
            self.to = deg;
        }
    }

    fn set_speed(&mut self, deg_per_s: f32) {
        let now = Instant::now();
        self.from = self.at(now);
        self.start = now;
        self.deg_per_s = deg_per_s;
    }
}

#[derive(Debug)]
struct Model {
    rotate: AxisModel,
    tilt: AxisModel,
}

impl Model {
    fn axis(&mut self, axis: AxisKind) -> &mut AxisModel {
        match axis {
            AxisKind::Rotate => &mut self.rotate,
            AxisKind::Tilt => &mut self.tilt,
        }
    }
}

/// An opened Revopoint Dual-axis Turntable, over any [`BleLink`] (the
/// tests drive it with a scripted link).
pub struct RevopointTurntable {
    link: Box<dyn BleLink>,
    info: TurntableInfo,
    timing: Timing,
    rx: ReplyBuffer,
    pending: VecDeque<Reply>,
    last_write: Option<Instant>,
    tilt_limits: (f32, f32),
    model: Arc<Mutex<Model>>,
    /// Target of the current rotation (continuous degrees).
    rotate_target: Option<f32>,
    /// Messages received outside of a command or query exchange (e.g.
    /// completion notifications), oldest first, capped.
    unsolicited: VecDeque<Reply>,
    /// Latest completion notification: when it arrived and its angle
    /// (continuous).
    completed: Option<(Instant, f32)>,
    /// When the latest rotation was acknowledged.
    rotate_sent: Option<Instant>,
}

impl std::fmt::Debug for RevopointTurntable {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("RevopointTurntable")
            .field("info", &self.info)
            .field("tilt_limits", &self.tilt_limits)
            .finish_non_exhaustive()
    }
}

impl RevopointTurntable {
    /// Talks to the table over `link` (notifications already enabled):
    /// reads the tilt limits (defaults when unanswered) and the rotation
    /// angle (required: it proves the protocol). The tilt is assumed 0.
    ///
    /// # Errors
    /// [`TurntableError::Protocol`] when the rotation angle is not answered;
    /// the link's errors.
    pub fn new(link: Box<dyn BleLink>, info: TurntableInfo, timing: Timing) -> Result<Self> {
        let model = Model {
            rotate: AxisModel::at_rest(0.0, DEFAULT_TURN_DEG_PER_S),
            tilt: AxisModel::at_rest(0.0, DEFAULT_TILT_DEG_PER_S),
        };
        let mut t = Self {
            link,
            info,
            timing,
            rx: ReplyBuffer::default(),
            pending: VecDeque::new(),
            last_write: None,
            tilt_limits: DEFAULT_TILT_LIMITS,
            model: Arc::new(Mutex::new(model)),
            rotate_target: None,
            unsolicited: VecDeque::new(),
            completed: None,
            rotate_sent: None,
        };
        match t.query(cmd::Q_TILT_ANGLE) {
            Ok(v) if v.len() >= 2 && v[0] < v[1] => t.tilt_limits = (v[0], v[1]),
            Ok(_) | Err(TurntableError::Protocol(_)) => {}
            Err(e) => return Err(e),
        }
        let angle = t.read_angle()?;
        t.model().rotate = AxisModel::at_rest(angle, DEFAULT_TURN_DEG_PER_S);
        Ok(t)
    }

    fn model(&self) -> MutexGuard<'_, Model> {
        self.model.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// Tilt limits reported by the table (degrees).
    pub fn tilt_limits(&self) -> (f32, f32) {
        self.tilt_limits
    }

    /// Messages that arrived outside of an exchange so far (diagnostics:
    /// completion notifications of newer firmware show up here).
    pub fn unsolicited(&self) -> impl Iterator<Item = &Reply> {
        self.unsolicited.iter()
    }

    fn note_unsolicited(&mut self, r: Reply) {
        if self.unsolicited.len() >= 32 {
            self.unsolicited.pop_front();
        }
        self.unsolicited.push_back(r);
    }

    /// Next message within `timeout`. Completion notifications are noted
    /// on the way.
    fn next_reply(&mut self, timeout: Duration) -> Result<Option<Reply>> {
        let deadline = Instant::now() + timeout;
        loop {
            if let Some(r) = self.pending.pop_front() {
                if let Some(raw) = completion_angle(&r) {
                    let angle = self.unwrapped(raw);
                    self.completed = Some((Instant::now(), angle));
                }
                return Ok(Some(r));
            }
            let left = deadline.saturating_duration_since(Instant::now());
            match self.link.recv(left)? {
                Some(bytes) => {
                    let replies = self.rx.push(&bytes);
                    self.pending.extend(replies);
                }
                // a completion notification has no `;`: nothing more came,
                // so it is whole
                None if self.rx.pending().starts_with(b"+OK,") => {
                    self.pending.extend(self.rx.flush());
                }
                None => return Ok(None),
            }
            if Instant::now() >= deadline && self.pending.is_empty() {
                return Ok(None);
            }
        }
    }

    /// Writes `text`, keeping the gap since the previous write; messages
    /// that arrived before are kept as unsolicited.
    fn write(&mut self, text: &str) -> Result<()> {
        while let Some(r) = self.next_reply(Duration::ZERO)? {
            self.note_unsolicited(r);
        }
        if let Some(last) = self.last_write {
            let ready = last + self.timing.command_gap;
            let now = Instant::now();
            if ready > now {
                std::thread::sleep(ready - now);
            }
        }
        self.link.write(text.as_bytes())?;
        self.last_write = Some(Instant::now());
        Ok(())
    }

    /// Sends a command and waits for its acknowledgement.
    ///
    /// # Errors
    /// [`TurntableError::Refused`] on `+FAIL`, [`TurntableError::Protocol`]
    /// when nothing answers.
    pub fn command(&mut self, text: &str) -> Result<Option<String>> {
        self.write(text)?;
        let sent = Instant::now();
        let deadline = sent + self.timing.reply_timeout;
        loop {
            let left = deadline.saturating_duration_since(Instant::now());
            match self.next_reply(left)? {
                // a completion notification is not the acknowledgement
                Some(r) if completion_angle(&r).is_some() => self.note_unsolicited(r),
                Some(Reply::Ok(detail)) => return Ok(detail),
                Some(Reply::Fail(f)) => {
                    return Err(TurntableError::Refused(format!("{text} -> {f}")));
                }
                Some(other) => self.note_unsolicited(other),
                // a firmware that acknowledges with the completion itself
                None if self.completed.is_some_and(|(at, _)| at >= sent) => return Ok(None),
                None => {
                    return Err(TurntableError::Protocol(format!("no answer to {text}")));
                }
            }
        }
    }

    /// Sends a query and returns the values of its `+DATA=` answer.
    ///
    /// # Errors
    /// [`TurntableError::Refused`] on `+FAIL`, [`TurntableError::Protocol`]
    /// when no data comes.
    pub fn query(&mut self, text: &str) -> Result<Vec<f32>> {
        self.write(text)?;
        let deadline = Instant::now() + self.timing.reply_timeout;
        loop {
            let left = deadline.saturating_duration_since(Instant::now());
            match self.next_reply(left)? {
                Some(Reply::Data(v)) => return Ok(v),
                Some(Reply::Fail(f)) => {
                    return Err(TurntableError::Refused(format!("{text} -> {f}")));
                }
                Some(other) => self.note_unsolicited(other),
                None => return Err(TurntableError::Protocol(format!("no data for {text}"))),
            }
        }
    }

    /// Sends `text` and returns every message received within
    /// [`Timing::reply_timeout`] (diagnostics: raw protocol exploration).
    ///
    /// # Errors
    /// The link's errors.
    pub fn exchange_raw(&mut self, text: &str) -> Result<Vec<Reply>> {
        self.write(text)?;
        let deadline = Instant::now() + self.timing.reply_timeout;
        let mut out = Vec::new();
        while let Some(r) = self.next_reply(deadline.saturating_duration_since(Instant::now()))? {
            out.push(r);
        }
        Ok(out)
    }

    /// A rotation angle as the table reports it (modulo 360) made
    /// continuous around the motion estimate.
    fn unwrapped(&self, raw: f32) -> f32 {
        unwrap_deg(raw, self.model().rotate.at(Instant::now()))
    }

    fn read_angle(&mut self) -> Result<f32> {
        let v = self.query(cmd::Q_CHANGE_ANGLE)?;
        let raw = v
            .first()
            .copied()
            .ok_or_else(|| TurntableError::Protocol("empty angle".into()))?;
        Ok(self.unwrapped(raw))
    }

    fn rotate_by(&mut self, deg: f32) -> Result<()> {
        if !deg.is_finite() {
            return Err(TurntableError::InvalidParams(format!("rotate by {deg}")));
        }
        self.command(&cmd::turn_angle(deg))?;
        let now = Instant::now();
        let mut m = self.model();
        // a turn sent while another is under way adds to its target
        let base = match self.rotate_target {
            Some(t) if m.rotate.end() > now => t,
            _ => m.rotate.at(now),
        };
        m.rotate.go(base + deg);
        drop(m);
        self.rotate_target = Some(base + deg);
        self.rotate_sent = Some(Instant::now());
        Ok(())
    }

    fn tilt_to(&mut self, deg: f32) -> Result<()> {
        self.axes_info(AxisKind::Tilt).check(deg)?;
        self.command(&cmd::tilt_value(deg))?;
        self.model().tilt.go(deg);
        Ok(())
    }

    fn axes_info(&self, kind: AxisKind) -> AxisInfo {
        match kind {
            AxisKind::Rotate => AxisInfo {
                kind,
                min_deg: None,
                max_deg: None,
                relative_only: true,
                feedback: Feedback::Notified,
            },
            AxisKind::Tilt => AxisInfo {
                kind,
                min_deg: Some(self.tilt_limits.0),
                max_deg: Some(self.tilt_limits.1),
                relative_only: false,
                feedback: Feedback::None,
            },
        }
    }

    /// Sleeps `d` in small slices; `true` when `cancel` fired.
    fn sleep_cancellable(d: Duration, cancel: &dyn Fn() -> bool) -> bool {
        let end = Instant::now() + d;
        loop {
            if cancel() {
                return true;
            }
            let now = Instant::now();
            if now >= end {
                return false;
            }
            std::thread::sleep((end - now).min(Duration::from_millis(20)));
        }
    }

    /// The angle of a completion notification received since the latest
    /// rotation was acknowledged.
    fn rotation_completed(&self) -> Option<f32> {
        let (at, angle) = self.completed?;
        (self.rotate_sent? <= at).then_some(angle)
    }

    /// The rotation ended at `angle`.
    fn settled_at(&mut self, angle: f32) {
        self.model().rotate.anchor(angle, true);
        self.rotate_target = None;
        self.rotate_sent = None;
    }

    /// Reads the notifications for `d` (or until a rotation completes);
    /// `true` when `cancel` fired.
    fn listen(&mut self, d: Duration, cancel: &dyn Fn() -> bool) -> Result<bool> {
        let end = Instant::now() + d;
        loop {
            if cancel() {
                return Ok(true);
            }
            let now = Instant::now();
            if now >= end || self.rotation_completed().is_some() {
                return Ok(false);
            }
            if let Some(r) = self.next_reply((end - now).min(Duration::from_millis(20)))? {
                self.note_unsolicited(r);
            }
        }
    }
}

impl Drop for RevopointTurntable {
    fn drop(&mut self) {
        self.link.close();
    }
}

impl Turntable for RevopointTurntable {
    fn info(&self) -> TurntableInfo {
        self.info.clone()
    }

    fn axes(&self) -> Vec<AxisInfo> {
        vec![
            self.axes_info(AxisKind::Rotate),
            self.axes_info(AxisKind::Tilt),
        ]
    }

    fn move_by(&mut self, axis: AxisKind, deg: f32) -> Result<()> {
        match axis {
            AxisKind::Rotate => self.rotate_by(deg),
            AxisKind::Tilt => {
                let now = self.model().tilt.to;
                self.tilt_to(now + deg)
            }
        }
    }

    fn move_to(&mut self, axis: AxisKind, deg: f32) -> Result<()> {
        match axis {
            AxisKind::Rotate => {
                let now = self.read_angle()?;
                self.model().rotate.anchor(now, false);
                self.rotate_target = Some(now);
                self.rotate_by(deg - now)
            }
            AxisKind::Tilt => self.tilt_to(deg),
        }
    }

    fn set_speed(&mut self, axis: AxisKind, deg_per_s: f32) -> Result<()> {
        if !(deg_per_s.is_finite() && deg_per_s > 0.0) {
            return Err(TurntableError::InvalidParams(format!(
                "speed {deg_per_s} deg/s"
            )));
        }
        match axis {
            AxisKind::Rotate => {
                let v = turn_speed_value(deg_per_s);
                self.command(&cmd::turn_speed(v))?;
                self.model().rotate.set_speed(360.0 / v as f32);
            }
            AxisKind::Tilt => {
                let v = tilt_speed_value(deg_per_s);
                self.command(&cmd::tilt_speed(v))?;
                self.model().tilt.set_speed(tilt_speed_deg_per_s(v));
            }
        }
        Ok(())
    }

    fn stop(&mut self) -> Result<()> {
        self.command(cmd::TURN_STOP)?;
        self.command(cmd::TILT_STOP)?;
        let now = Instant::now();
        {
            let mut m = self.model();
            let tilt = m.tilt.at(now);
            m.tilt.anchor(tilt, true);
        }
        let angle = self.read_angle()?;
        self.model().rotate.anchor(angle, true);
        self.rotate_target = None;
        self.rotate_sent = None;
        Ok(())
    }

    fn home(&mut self) -> Result<()> {
        self.command(cmd::TURN_TO_ZERO)?;
        self.command(cmd::TILT_TO_ZERO)?;
        let mut m = self.model();
        // the table goes the short way: to the closest whole turn
        let zero = 360.0 * (m.rotate.at(Instant::now()) / 360.0).round();
        m.axis(AxisKind::Rotate).go(zero);
        m.axis(AxisKind::Tilt).go(0.0);
        drop(m);
        self.rotate_target = Some(zero);
        self.rotate_sent = Some(Instant::now());
        Ok(())
    }

    fn position(&mut self, axis: AxisKind) -> Result<Option<f32>> {
        match axis {
            AxisKind::Rotate => {
                let a = self.read_angle()?;
                let settled = self.model().rotate.end() <= Instant::now();
                self.model().rotate.anchor(a, settled);
                Ok(Some(a))
            }
            AxisKind::Tilt => Ok(Some(self.model().tilt.at(Instant::now()))),
        }
    }

    fn wait_settled(
        &mut self,
        axis: AxisKind,
        timeout: Duration,
        cancel: &dyn Fn() -> bool,
    ) -> Result<()> {
        let deadline = Instant::now() + timeout;
        match axis {
            AxisKind::Tilt => {
                let end = self.model().tilt.end() + self.timing.tilt_margin;
                let now = Instant::now();
                if end > deadline {
                    return Err(TurntableError::Timeout(format!(
                        "tilt needs {:?}, more than {timeout:?}",
                        end - now
                    )));
                }
                if Self::sleep_cancellable(end.saturating_duration_since(now), cancel) {
                    return Err(TurntableError::Cancelled);
                }
                let tilt = self.model().tilt.to;
                self.model().tilt.anchor(tilt, true);
                Ok(())
            }
            AxisKind::Rotate => {
                let expected_end = self.model().rotate.end();
                let mut previous: Option<f32> = None;
                loop {
                    if cancel() {
                        return Err(TurntableError::Cancelled);
                    }
                    // the completion notification ends the wait at once;
                    // two equal readings are the fallback
                    if let Some(a) = self.rotation_completed() {
                        self.settled_at(a);
                        return Ok(());
                    }
                    let a = self.read_angle()?;
                    let now = Instant::now();
                    let arrived = self
                        .rotate_target
                        .is_none_or(|t| (a - t).abs() <= ARRIVED_DEG);
                    let overdue = now >= expected_end + OVERDUE;
                    let still =
                        previous.is_some_and(|p| (a - p).abs() <= self.timing.settle_epsilon_deg);
                    if still && (arrived || overdue) {
                        self.settled_at(a);
                        return Ok(());
                    }
                    self.model().rotate.anchor(a, false);
                    previous = Some(a);
                    if now >= deadline {
                        return Err(TurntableError::Timeout(format!(
                            "rotation still moving at {a} degrees"
                        )));
                    }
                    if self.listen(self.timing.poll_interval, cancel)? {
                        return Err(TurntableError::Cancelled);
                    }
                }
            }
        }
    }

    fn probe(&self) -> Option<AngleProbe> {
        let model = Arc::clone(&self.model);
        Some(Arc::new(move || {
            let m = model.lock().unwrap_or_else(|e| e.into_inner());
            let now = Instant::now();
            Angles {
                rotate_deg: Some(m.rotate.at(now)),
                tilt_deg: Some(m.tilt.at(now)),
                moving: m.rotate.end() > now || m.tilt.end() > now,
            }
        }))
    }
}

/// The backend: Revopoint Dual-axis Turntables seen over Bluetooth LE.
#[derive(Debug, Clone, Copy, Default)]
pub struct RevopointBackend;

impl TurntableBackend for RevopointBackend {
    fn id(&self) -> &'static str {
        BACKEND_ID
    }

    fn availability(&self) -> Result<()> {
        Ok(dc_ble::availability()?)
    }

    fn list(&self, timeout: Duration) -> Result<Vec<TurntableInfo>> {
        if timeout.is_zero() {
            return Ok(Vec::new());
        }
        Ok(dc_ble::scan(timeout, &name_matches)?
            .into_iter()
            .map(|d| TurntableInfo {
                backend: BACKEND_ID.to_owned(),
                id: d.id,
                name: d.name.unwrap_or_else(|| DEVICE_NAME.to_owned()),
            })
            .collect())
    }

    fn open(&self, id: &str) -> Result<Box<dyn Turntable>> {
        let link = dc_ble::connect_serial(id, &profile(), CONNECT_TIMEOUT)?;
        let info = TurntableInfo {
            backend: BACKEND_ID.to_owned(),
            id: id.to_owned(),
            name: DEVICE_NAME.to_owned(),
        };
        Ok(Box::new(RevopointTurntable::new(
            link,
            info,
            Timing::default(),
        )?))
    }
}

#[cfg(test)]
mod tests;
