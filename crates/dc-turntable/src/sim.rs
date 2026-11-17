//! Simulated dual-axis turntable `sim`.
//!
//! The table lives in a [`SimRig`]: the angles of both axes as functions of
//! a simulation clock. A simulated scanner shares the rig (the dependency
//! goes scanner -> turntable): before rendering each frame it calls
//! [`SimRig::advance`] with the frame period and renders the object at the
//! returned [`RigPose`], so a scan sees the object turn and the motion is
//! paced by the frames, deterministic even when frames are computed faster
//! than real time.
//!
//! While a simulated scanner streams ([`SimRig::set_streaming`]) the clock
//! moves only with its frames. Otherwise the clock follows real time; in
//! fast mode ([`SimTurntableBackend::new`] with `fast`)
//! [`Turntable::wait_settled`] then completes the motion at once.

use std::sync::{Arc, Mutex, MutexGuard};
use std::time::{Duration, Instant};

use crate::{
    AngleProbe, Angles, AxisInfo, AxisKind, Feedback, Result, Turntable, TurntableBackend,
    TurntableError, TurntableInfo,
};

/// Id of the simulated backend.
pub const BACKEND_ID: &str = "sim";
/// Device id of the simulated table.
pub const DEVICE_ID: &str = "sim-turntable";
/// Tilt range of the simulated table: `[-30, 30]` degrees, like the
/// Revopoint Dual-axis Turntable.
pub const TILT_LIMIT_DEG: f32 = 30.0;
/// Rotation speed at power-on (degrees per second).
pub const DEFAULT_ROTATE_DEG_PER_S: f32 = 30.0;
/// Tilt speed at power-on (degrees per second).
pub const DEFAULT_TILT_DEG_PER_S: f32 = 15.0;
/// Speeds are clamped to this range (degrees per second).
const SPEED_RANGE: (f32, f32) = (0.5, 720.0);

/// Angles of the simulated table (degrees).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct RigPose {
    /// Cumulative signed rotation.
    pub rotate_deg: f32,
    /// Tilt.
    pub tilt_deg: f32,
}

/// Constant-speed motion of one axis on the simulation clock.
#[derive(Clone, Copy, Debug)]
struct Motion {
    from: f32,
    to: f32,
    start: f64,
    speed: f32,
}

impl Motion {
    fn at_rest(deg: f32, speed: f32) -> Self {
        Self {
            from: deg,
            to: deg,
            start: 0.0,
            speed,
        }
    }

    fn at(&self, t: f64) -> f32 {
        let d = self.to - self.from;
        let travel = (f64::from(self.speed) * (t - self.start).max(0.0)) as f32;
        if travel >= d.abs() {
            self.to
        } else {
            self.from + travel.copysign(d)
        }
    }

    fn end(&self) -> f64 {
        self.start + f64::from((self.to - self.from).abs() / self.speed)
    }
}

#[derive(Debug)]
struct Inner {
    clock: f64,
    last_real: Instant,
    /// A simulated scanner streams: its frames pace the clock.
    streaming: bool,
    attached: bool,
    fast: bool,
    rotate: Motion,
    tilt: Motion,
}

impl Inner {
    /// Brings the clock up to date with real time when no scanner paces it.
    fn sync(&mut self) {
        let now = Instant::now();
        if !self.streaming {
            self.clock += (now - self.last_real).as_secs_f64();
        }
        self.last_real = now;
    }

    fn motion(&mut self, axis: AxisKind) -> &mut Motion {
        match axis {
            AxisKind::Rotate => &mut self.rotate,
            AxisKind::Tilt => &mut self.tilt,
        }
    }

    fn pose(&self) -> RigPose {
        RigPose {
            rotate_deg: self.rotate.at(self.clock),
            tilt_deg: self.tilt.at(self.clock),
        }
    }

    fn moving(&self, axis: AxisKind) -> bool {
        let m = match axis {
            AxisKind::Rotate => &self.rotate,
            AxisKind::Tilt => &self.tilt,
        };
        self.clock < m.end()
    }

    /// Starts a motion of `axis` from where it is now to `to`.
    fn go(&mut self, axis: AxisKind, to: f32) {
        let clock = self.clock;
        let m = self.motion(axis);
        *m = Motion {
            from: m.at(clock),
            to,
            start: clock,
            speed: m.speed,
        };
    }
}

/// The shared state of a simulated table: its angles over the simulation
/// clock. Shared by the table and by the simulated scanner.
#[derive(Debug)]
pub struct SimRig {
    inner: Mutex<Inner>,
}

impl SimRig {
    /// A rig at rest at zero, no table attached.
    pub fn new() -> Arc<Self> {
        Arc::new(Self {
            inner: Mutex::new(Inner {
                clock: 0.0,
                last_real: Instant::now(),
                streaming: false,
                attached: false,
                fast: false,
                rotate: Motion::at_rest(0.0, DEFAULT_ROTATE_DEG_PER_S),
                tilt: Motion::at_rest(0.0, DEFAULT_TILT_DEG_PER_S),
            }),
        })
    }

    fn lock(&self) -> MutexGuard<'_, Inner> {
        self.inner.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// A simulated table is open on this rig: a simulated scanner then
    /// renders the turntable scene instead of its hand-held orbit.
    pub fn attached(&self) -> bool {
        self.lock().attached
    }

    /// A simulated scanner starts (`true`) or stops streaming: while it
    /// streams, only its frames ([`SimRig::advance`]) move the clock.
    pub fn set_streaming(&self, on: bool) {
        let mut g = self.lock();
        g.sync();
        g.streaming = on;
    }

    /// Advances the clock by one frame period `dt_s` (seconds) and returns
    /// the pose at the new time; called by the simulated scanner before it
    /// renders a frame.
    pub fn advance(&self, dt_s: f64) -> RigPose {
        let mut g = self.lock();
        g.clock += dt_s.max(0.0);
        g.last_real = Instant::now();
        g.pose()
    }

    /// The pose now.
    pub fn pose(&self) -> RigPose {
        let mut g = self.lock();
        g.sync();
        g.pose()
    }

    /// The simulation clock (seconds).
    pub fn clock_s(&self) -> f64 {
        let mut g = self.lock();
        g.sync();
        g.clock
    }
}

/// The simulated backend: one table, [`DEVICE_ID`], on its rig.
#[derive(Debug, Clone)]
pub struct SimTurntableBackend {
    rig: Arc<SimRig>,
    fast: bool,
}

impl Default for SimTurntableBackend {
    fn default() -> Self {
        Self::new(SimRig::new(), false)
    }
}

impl SimTurntableBackend {
    /// Backend whose table moves `rig`; `fast` completes motions at once
    /// when no scanner drives the clock.
    pub fn new(rig: Arc<SimRig>, fast: bool) -> Self {
        Self { rig, fast }
    }

    /// The rig of the table.
    pub fn rig(&self) -> &Arc<SimRig> {
        &self.rig
    }

    fn info() -> TurntableInfo {
        TurntableInfo {
            backend: BACKEND_ID.to_owned(),
            id: DEVICE_ID.to_owned(),
            name: "Simulated dual-axis turntable".to_owned(),
        }
    }
}

impl TurntableBackend for SimTurntableBackend {
    fn id(&self) -> &'static str {
        BACKEND_ID
    }

    fn list(&self, _timeout: Duration) -> Result<Vec<TurntableInfo>> {
        Ok(vec![Self::info()])
    }

    fn open(&self, id: &str) -> Result<Box<dyn Turntable>> {
        if id != DEVICE_ID {
            return Err(TurntableError::NotFound(id.to_owned()));
        }
        let mut g = self.rig.lock();
        if g.attached {
            return Err(TurntableError::Busy);
        }
        g.sync();
        g.attached = true;
        g.fast = self.fast;
        drop(g);
        Ok(Box::new(SimTurntable {
            rig: Arc::clone(&self.rig),
        }))
    }
}

/// The opened simulated table. Dropping it detaches it from the rig.
#[derive(Debug)]
pub struct SimTurntable {
    rig: Arc<SimRig>,
}

impl Drop for SimTurntable {
    fn drop(&mut self) {
        self.rig.lock().attached = false;
    }
}

fn axes() -> Vec<AxisInfo> {
    vec![
        AxisInfo {
            kind: AxisKind::Rotate,
            min_deg: None,
            max_deg: None,
            relative_only: false,
            feedback: Feedback::Polled,
        },
        AxisInfo {
            kind: AxisKind::Tilt,
            min_deg: Some(-TILT_LIMIT_DEG),
            max_deg: Some(TILT_LIMIT_DEG),
            relative_only: false,
            feedback: Feedback::Polled,
        },
    ]
}

impl SimTurntable {
    fn target(&self, axis: AxisKind, to: f32) -> Result<()> {
        crate::axis_of(&axes(), axis)?.check(to)?;
        let mut g = self.rig.lock();
        g.sync();
        g.go(axis, to);
        Ok(())
    }
}

impl Turntable for SimTurntable {
    fn info(&self) -> TurntableInfo {
        SimTurntableBackend::info()
    }

    fn axes(&self) -> Vec<AxisInfo> {
        axes()
    }

    fn move_by(&mut self, axis: AxisKind, deg: f32) -> Result<()> {
        if !deg.is_finite() {
            return Err(TurntableError::InvalidParams(format!("move by {deg}")));
        }
        let now = {
            let mut g = self.rig.lock();
            g.sync();
            let clock = g.clock;
            g.motion(axis).at(clock)
        };
        self.target(axis, now + deg)
    }

    fn move_to(&mut self, axis: AxisKind, deg: f32) -> Result<()> {
        self.target(axis, deg)
    }

    fn set_speed(&mut self, axis: AxisKind, deg_per_s: f32) -> Result<()> {
        if !(deg_per_s.is_finite() && deg_per_s > 0.0) {
            return Err(TurntableError::InvalidParams(format!(
                "speed {deg_per_s} deg/s"
            )));
        }
        let mut g = self.rig.lock();
        g.sync();
        let clock = g.clock;
        let m = g.motion(axis);
        let now = m.at(clock);
        *m = Motion {
            from: now,
            to: m.to,
            start: clock,
            speed: deg_per_s.clamp(SPEED_RANGE.0, SPEED_RANGE.1),
        };
        Ok(())
    }

    fn stop(&mut self) -> Result<()> {
        let mut g = self.rig.lock();
        g.sync();
        let pose = g.pose();
        g.go(AxisKind::Rotate, pose.rotate_deg);
        g.go(AxisKind::Tilt, pose.tilt_deg);
        Ok(())
    }

    fn home(&mut self) -> Result<()> {
        let mut g = self.rig.lock();
        g.sync();
        g.go(AxisKind::Rotate, 0.0);
        g.go(AxisKind::Tilt, 0.0);
        Ok(())
    }

    fn position(&mut self, axis: AxisKind) -> Result<Option<f32>> {
        let p = self.rig.pose();
        Ok(Some(match axis {
            AxisKind::Rotate => p.rotate_deg,
            AxisKind::Tilt => p.tilt_deg,
        }))
    }

    fn wait_settled(
        &mut self,
        axis: AxisKind,
        timeout: Duration,
        cancel: &dyn Fn() -> bool,
    ) -> Result<()> {
        let deadline = Instant::now() + timeout;
        loop {
            {
                let mut g = self.rig.lock();
                g.sync();
                if g.fast && !g.streaming {
                    let end = g.motion(axis).end();
                    g.clock = g.clock.max(end);
                }
                if !g.moving(axis) {
                    return Ok(());
                }
            }
            if cancel() {
                return Err(TurntableError::Cancelled);
            }
            if Instant::now() >= deadline {
                return Err(TurntableError::Timeout(format!(
                    "{} still moving after {timeout:?}",
                    axis.as_str()
                )));
            }
            std::thread::sleep(Duration::from_millis(1));
        }
    }

    fn probe(&self) -> Option<AngleProbe> {
        let rig = Arc::clone(&self.rig);
        Some(Arc::new(move || {
            let mut g = rig.lock();
            g.sync();
            let p = g.pose();
            Angles {
                rotate_deg: Some(p.rotate_deg),
                tilt_deg: Some(p.tilt_deg),
                moving: g.moving(AxisKind::Rotate) || g.moving(AxisKind::Tilt),
            }
        }))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn never() -> bool {
        false
    }

    #[test]
    fn fast_table_settles_at_once_and_detaches_on_drop() {
        let backend = SimTurntableBackend::new(SimRig::new(), true);
        assert_eq!(backend.list(Duration::ZERO).unwrap()[0].id, DEVICE_ID);
        assert_eq!(
            backend.open("x").err().unwrap().code(),
            "turntable-not-found"
        );
        let mut t = backend.open(DEVICE_ID).unwrap();
        assert!(backend.rig().attached());
        assert_eq!(
            backend.open(DEVICE_ID).err().unwrap().code(),
            "turntable-busy"
        );
        t.move_by(AxisKind::Rotate, -90.0).unwrap();
        t.move_to(AxisKind::Tilt, 20.0).unwrap();
        assert_eq!(
            t.move_to(AxisKind::Tilt, 40.0).unwrap_err().code(),
            "turntable-out-of-range"
        );
        let probe = t.probe().unwrap();
        assert!(probe().moving);
        t.wait_settled(AxisKind::Rotate, Duration::from_secs(1), &never)
            .unwrap();
        t.wait_settled(AxisKind::Tilt, Duration::from_secs(1), &never)
            .unwrap();
        assert_eq!(t.position(AxisKind::Rotate).unwrap(), Some(-90.0));
        assert_eq!(t.position(AxisKind::Tilt).unwrap(), Some(20.0));
        let a = probe();
        assert_eq!(
            (a.rotate_deg, a.tilt_deg, a.moving),
            (Some(-90.0), Some(20.0), false)
        );
        t.home().unwrap();
        t.wait_settled(AxisKind::Tilt, Duration::from_secs(1), &never)
            .unwrap();
        t.wait_settled(AxisKind::Rotate, Duration::from_secs(1), &never)
            .unwrap();
        assert_eq!(backend.rig().pose(), RigPose::default());
        drop(t);
        assert!(!backend.rig().attached());
    }

    #[test]
    fn frames_drive_the_motion_at_the_set_speed() {
        let rig = SimRig::new();
        rig.set_streaming(true);
        let mut t = SimTurntableBackend::new(Arc::clone(&rig), true)
            .open(DEVICE_ID)
            .unwrap();
        t.set_speed(AxisKind::Rotate, 40.0).unwrap();
        t.move_by(AxisKind::Rotate, 10.0).unwrap();
        // a scanner at 10 fps: 4 degrees per frame
        let p = rig.advance(0.1);
        assert!((p.rotate_deg - 4.0).abs() < 1e-3, "{p:?}");
        assert!((rig.advance(0.1).rotate_deg - 8.0).abs() < 1e-3);
        assert_eq!(rig.advance(0.1).rotate_deg, 10.0);
        // while a scanner streams, the wait does not jump the clock
        t.move_by(AxisKind::Rotate, 10.0).unwrap();
        rig.advance(0.0);
        let cancelled = t.wait_settled(AxisKind::Rotate, Duration::from_secs(5), &|| true);
        assert_eq!(cancelled.unwrap_err().code(), "cancelled");
        assert_eq!(rig.pose().rotate_deg, 10.0);
        assert_eq!(
            t.set_speed(AxisKind::Tilt, 0.0).unwrap_err().code(),
            "invalid-params"
        );
    }

    #[test]
    fn realtime_table_moves_in_real_time_and_stops() {
        let backend = SimTurntableBackend::new(SimRig::new(), false);
        let mut t = backend.open(DEVICE_ID).unwrap();
        t.set_speed(AxisKind::Rotate, 500.0).unwrap();
        t.move_by(AxisKind::Rotate, 20.0).unwrap();
        let start = Instant::now();
        t.wait_settled(AxisKind::Rotate, Duration::from_secs(5), &never)
            .unwrap();
        assert!(start.elapsed() >= Duration::from_millis(30));
        assert_eq!(t.position(AxisKind::Rotate).unwrap(), Some(20.0));
        t.set_speed(AxisKind::Rotate, 1.0).unwrap();
        t.move_by(AxisKind::Rotate, 90.0).unwrap();
        t.stop().unwrap();
        let p = t.position(AxisKind::Rotate).unwrap().unwrap();
        assert!((20.0..21.0).contains(&p), "{p}");
        t.wait_settled(AxisKind::Rotate, Duration::from_millis(100), &never)
            .unwrap();
    }
}
