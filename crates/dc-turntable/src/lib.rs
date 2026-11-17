//! Motorised turntables of a scanning rig (alpha).
//!
//! A fixed scanner looks at an object standing on a turntable; the table
//! rotates (and, on dual-axis tables, tilts) the object in front of it. This
//! crate is the device layer, headless and GPU-free:
//!
//! * [`Turntable`] is one opened table: relative / absolute moves per
//!   [`AxisKind`], speed in degrees per second, stop, home, position and a
//!   blocking [`Turntable::wait_settled`]. [`AxisInfo`] tells what each axis
//!   can do (range, relative-only, kind of position [`Feedback`]).
//! * [`TurntableBackend`] enumerates and opens the tables of one kind;
//!   [`backends`] is the registry.
//! * [`Turntable::probe`] returns an [`AngleProbe`]: a cheap, thread-safe
//!   estimate of the current angles (no I/O), for live status and for the
//!   motion prior of the scan tracking.
//!
//! Backends:
//!
//! * [`revopoint`] (`revopoint-ble`): the Revopoint Dual-axis Turntable
//!   over Bluetooth LE ([`dc_ble`]), community reverse-engineered protocol.
//! * [`sim`] (`sim`): an in-memory table whose pose a simulated scanner can
//!   read ([`sim::SimRig`]), so a simulated scan sees the object turn.
//!
//! # Adding a turntable
//!
//! Implement [`Turntable`] and [`TurntableBackend`] and register the backend
//! in [`backends`]. `list` may block for its `timeout` (radio discovery);
//! `open` connects. Moves return once the device accepted the command, not
//! when the motion ends: callers wait with [`Turntable::wait_settled`].
//! Angles are degrees, speeds degrees per second, whatever the device
//! units are. Errors carry a stable [`TurntableError::code`].

pub mod revopoint;
pub mod sim;

use std::sync::Arc;
use std::time::Duration;

use dc_ble::BleError;
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};

/// An axis of a turntable.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "kebab-case")]
pub enum AxisKind {
    /// Rotation of the platter about its own axis (unbounded).
    Rotate,
    /// Tilt of the platter (bounded).
    Tilt,
}

impl AxisKind {
    /// Stable lowercase name (`rotate`, `tilt`).
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Rotate => "rotate",
            Self::Tilt => "tilt",
        }
    }
}

/// How the position of an axis is known.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum Feedback {
    /// The device reports the end of a motion by itself.
    Notified,
    /// The position can be read; the end of a motion is found by polling.
    Polled,
    /// No feedback: the motion is timed from the commanded speed.
    None,
}

/// What an axis can do.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct AxisInfo {
    /// The axis.
    pub kind: AxisKind,
    /// Lowest reachable angle (degrees), when bounded.
    pub min_deg: Option<f32>,
    /// Highest reachable angle (degrees), when bounded.
    pub max_deg: Option<f32>,
    /// The device only takes relative moves; [`Turntable::move_to`] is
    /// emulated from the position.
    pub relative_only: bool,
    /// How the position is known.
    pub feedback: Feedback,
}

impl AxisInfo {
    /// Checks that `deg` is within the range of the axis.
    ///
    /// # Errors
    /// [`TurntableError::OutOfRange`].
    pub fn check(&self, deg: f32) -> Result<()> {
        let below = self.min_deg.is_some_and(|m| deg < m - 1e-3);
        let above = self.max_deg.is_some_and(|m| deg > m + 1e-3);
        if !deg.is_finite() || below || above {
            return Err(TurntableError::OutOfRange {
                axis: self.kind,
                deg,
                min: self.min_deg,
                max: self.max_deg,
            });
        }
        Ok(())
    }
}

/// Identity of a turntable as listed by its backend.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct TurntableInfo {
    /// Id of the backend (`revopoint-ble`, `sim`).
    pub backend: String,
    /// Id of the device within the backend (Bluetooth address or platform
    /// id, `sim-turntable`).
    pub id: String,
    /// Display name reported by the device (not translated).
    pub name: String,
}

/// Estimated current angles of a table (degrees).
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Angles {
    /// Cumulative signed rotation, when the table rotates.
    pub rotate_deg: Option<f32>,
    /// Tilt, when the table tilts.
    pub tilt_deg: Option<f32>,
    /// An axis is expected to be moving.
    pub moving: bool,
}

/// Cheap thread-safe estimate of the angles of a table, see
/// [`Turntable::probe`]. It never does I/O.
pub type AngleProbe = Arc<dyn Fn() -> Angles + Send + Sync>;

/// Errors of the turntable layer. Every variant has a stable kebab-case
/// [`TurntableError::code`]; Bluetooth errors keep their `ble-*` code.
#[derive(Debug, thiserror::Error)]
pub enum TurntableError {
    /// No backend with this id is registered.
    #[error("unknown turntable backend `{0}`")]
    UnknownBackend(String),
    /// The backend has no table with this id.
    #[error("turntable `{0}` not found")]
    NotFound(String),
    /// The action needs a connected turntable.
    #[error("no turntable connected")]
    NotConnected,
    /// The table has no such axis.
    #[error("the turntable has no {} axis", .0.as_str())]
    NoAxis(AxisKind),
    /// The target angle is outside the range of the axis.
    #[error("{} {deg} degrees out of range [{min:?}, {max:?}]", axis.as_str())]
    OutOfRange {
        /// The axis.
        axis: AxisKind,
        /// The refused angle.
        deg: f32,
        /// Lowest angle of the axis.
        min: Option<f32>,
        /// Highest angle of the axis.
        max: Option<f32>,
    },
    /// The table is executing a job (automatic scan, calibration).
    #[error("the turntable is busy")]
    Busy,
    /// The device answered something unexpected, or nothing.
    #[error("turntable protocol error: {0}")]
    Protocol(String),
    /// The device refused a command.
    #[error("the turntable refused the command: {0}")]
    Refused(String),
    /// A motion did not end in time.
    #[error("turntable motion timed out: {0}")]
    Timeout(String),
    /// The caller cancelled the wait.
    #[error("cancelled")]
    Cancelled,
    /// A parameter is invalid.
    #[error("invalid parameter: {0}")]
    InvalidParams(String),
    /// The Bluetooth link failed.
    #[error(transparent)]
    Ble(#[from] BleError),
}

impl TurntableError {
    /// Stable identifier, the same in every language (kebab-case).
    pub fn code(&self) -> &'static str {
        match self {
            Self::UnknownBackend(_) => "turntable-unknown-backend",
            Self::NotFound(_) => "turntable-not-found",
            Self::NotConnected => "turntable-not-connected",
            Self::NoAxis(_) => "turntable-no-axis",
            Self::OutOfRange { .. } => "turntable-out-of-range",
            Self::Busy => "turntable-busy",
            Self::Protocol(_) => "turntable-protocol",
            Self::Refused(_) => "turntable-refused",
            Self::Timeout(_) => "turntable-timeout",
            Self::Cancelled => "cancelled",
            Self::InvalidParams(_) => "invalid-params",
            Self::Ble(e) => e.code(),
        }
    }
}

/// Result alias of the turntable crate.
pub type Result<T, E = TurntableError> = std::result::Result<T, E>;

/// An opened turntable. Moves return once the device accepted them.
pub trait Turntable: Send {
    /// Identity of the table.
    fn info(&self) -> TurntableInfo;
    /// The axes of the table.
    fn axes(&self) -> Vec<AxisInfo>;
    /// Moves `axis` by `deg` degrees (signed).
    fn move_by(&mut self, axis: AxisKind, deg: f32) -> Result<()>;
    /// Moves `axis` to the absolute angle `deg`.
    fn move_to(&mut self, axis: AxisKind, deg: f32) -> Result<()>;
    /// Sets the speed of `axis`; the device may clamp it to its range.
    fn set_speed(&mut self, axis: AxisKind, deg_per_s: f32) -> Result<()>;
    /// Stops every axis.
    fn stop(&mut self) -> Result<()>;
    /// Returns every axis to its zero.
    fn home(&mut self) -> Result<()>;
    /// Current angle of `axis`, `None` when it cannot be known.
    fn position(&mut self, axis: AxisKind) -> Result<Option<f32>>;
    /// Blocks until `axis` stops moving, at most `timeout`; `cancel` is
    /// polled and makes the wait fail with [`TurntableError::Cancelled`]
    /// (the motion itself is not stopped).
    fn wait_settled(
        &mut self,
        axis: AxisKind,
        timeout: Duration,
        cancel: &dyn Fn() -> bool,
    ) -> Result<()>;
    /// Live estimate of the angles without I/O, when the backend has one.
    fn probe(&self) -> Option<AngleProbe> {
        None
    }
}

/// A kind of turntable: enumerates and opens its devices.
pub trait TurntableBackend: Send + Sync {
    /// Short stable id of the backend.
    fn id(&self) -> &'static str;
    /// Whether the backend can be used at all (e.g. a Bluetooth adapter is
    /// present and powered).
    fn availability(&self) -> Result<()> {
        Ok(())
    }
    /// Tables currently available; may block up to `timeout` (discovery).
    fn list(&self, timeout: Duration) -> Result<Vec<TurntableInfo>>;
    /// Opens the table `id` (an id returned by [`TurntableBackend::list`]).
    fn open(&self, id: &str) -> Result<Box<dyn Turntable>>;
}

/// The registered backends: `revopoint-ble`, then `sim` (always available,
/// with its own [`sim::SimRig`]; a host that wants a simulated scanner to
/// see the table builds [`sim::SimTurntableBackend::new`] with a shared rig).
pub fn backends() -> Vec<Box<dyn TurntableBackend>> {
    vec![
        Box::new(revopoint::RevopointBackend),
        Box::new(sim::SimTurntableBackend::default()),
    ]
}

/// The registered backend with id `id`.
///
/// # Errors
/// [`TurntableError::UnknownBackend`].
pub fn backend(id: &str) -> Result<Box<dyn TurntableBackend>> {
    backends()
        .into_iter()
        .find(|b| b.id() == id)
        .ok_or_else(|| TurntableError::UnknownBackend(id.to_owned()))
}

/// The axis `kind` of `axes`.
///
/// # Errors
/// [`TurntableError::NoAxis`].
pub fn axis_of(axes: &[AxisInfo], kind: AxisKind) -> Result<&AxisInfo> {
    axes.iter()
        .find(|a| a.kind == kind)
        .ok_or(TurntableError::NoAxis(kind))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn registry_and_codes() {
        let ids: Vec<&str> = backends().iter().map(|b| b.id()).collect();
        assert_eq!(ids, ["revopoint-ble", "sim"]);
        assert_eq!(
            backend("nope").err().unwrap().code(),
            "turntable-unknown-backend"
        );
        assert_eq!(
            TurntableError::NotConnected.code(),
            "turntable-not-connected"
        );
        assert_eq!(TurntableError::Busy.code(), "turntable-busy");
        assert_eq!(
            TurntableError::Protocol("x".into()).code(),
            "turntable-protocol"
        );
        assert_eq!(
            TurntableError::from(BleError::Timeout).code(),
            "ble-timeout"
        );
    }

    #[test]
    fn axis_range_is_checked() {
        let tilt = AxisInfo {
            kind: AxisKind::Tilt,
            min_deg: Some(-30.0),
            max_deg: Some(30.0),
            relative_only: false,
            feedback: Feedback::None,
        };
        tilt.check(30.0).unwrap();
        tilt.check(-30.0).unwrap();
        assert_eq!(
            tilt.check(31.0).unwrap_err().code(),
            "turntable-out-of-range"
        );
        assert!(tilt.check(f32::NAN).is_err());
        assert_eq!(
            serde_json::to_value(AxisKind::Rotate).unwrap(),
            serde_json::json!("rotate")
        );
        assert_eq!(
            axis_of(std::slice::from_ref(&tilt), AxisKind::Rotate)
                .unwrap_err()
                .code(),
            "turntable-no-axis"
        );
    }
}
