//! The fill light of the scanner, the first vendor command of the backend.
//!
//! **User decision, 2026-11-18 21:36.** Left alone, the MINI 2 blinks its
//! LEDs about twice a second, streaming or not; Revo Scan keeps them steady.
//! The user authorised exactly two command sequences, the fill light on and
//! off, and nothing else (no light level, no projector, no IR LED, no other
//! line): [`FillLight`] is that closed list, and no free-form command can be
//! built from it. Measured on the rig the same evening: on → the LEDs stay
//! steadily on; off → they go off; both take effect with or without a
//! stream.
//!
//! **Policy** (user: "whatever scans better, and off when the scan is
//! over"). The light is turned off when the device is opened (idle it
//! would blink), on while a stream runs ([`crate::ScanSettings::fill_light`],
//! default on) and off again when the stream stops, is lost or the device
//! is dropped. On the rig, static piece, 15 s each: on → 97.0% of the
//! piece's pixels with depth per frame and 19 µm median per-pixel depth
//! noise; off → 92.8% and 27 µm; 2.74 fps either way.
//!
//! The lines are shell commands that the scanner runs on its own Linux,
//! sent over the vendor extension unit with the checks of [`super::xu`].

use super::xu::ShellLine;

/// The fill light states the user authorised.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FillLight {
    /// Steadily on.
    On,
    /// Off.
    Off,
}

impl FillLight {
    /// The authorised lines of this state, in order.
    pub fn lines(self) -> &'static [ShellLine] {
        match self {
            Self::On => &[ShellLine::B03On, ShellLine::B02On],
            Self::Off => &[ShellLine::B03Off],
        }
    }
}
