//! The fast exposure profile of the MINI 2: a closed list of shell lines.
//!
//! **User decision, 2026-11-18 21:43** (supervised experiments, volatile
//! registers): besides the fill light ([`super::light`]), the shell selector
//! may receive exactly the lines of [`ExposureProfile`]. They travel as the
//! fill-light lines do, after the same checks ([`super::xu`]).
//!
//! One depth frame of the MINI 2 is 18 projected patterns; each takes the
//! "frame time" of register `0x910`. In the dark the device's own auto
//! exposure stretches it to ~20 ms, so 18 x 20.3 ms = 365 ms: 2.74 fps.
//! [`ExposureProfile::Fast`] switches the auto exposure off (`0x912 0`), sets
//! the exposure (`0x911`, µs) to 6000 and then the frame time to 8000 µs
//! (the exposure must stay at least 2000 µs below it). Below 8000 µs the
//! period does not shrink further. The auto exposure is left off: switched
//! back on (`0x912 1`) it stretched the frame time again in the dark, to
//! 4.4 fps in one run and 6.8 in another, so the result is not repeatable.
//!
//! Measured on the rig (2026-11-18, dark room, static test piece, Full
//! mode, factory intrinsics): period 148 ms (6.76 fps) against 365 ms;
//! 96-99% of the piece's pixels with depth per frame against 99.7%;
//! per-pixel depth noise 21-25 µm against 17 µm; frame-to-model ICP
//! residual 0.049-0.057 mm against 0.075 mm. A 360 degree turn at 10 °/s
//! (1.5 degree per frame) tracked all 294 frames with a median ICP residual
//! of 0.065 mm and closed the loop within 0.5 degree. Seven streams, no
//! stall and no error; the lines were also sent back to back without a
//! stall. The exposure is fixed, so a very bright object may saturate.
//!
//! The registers are volatile: they keep their values across stream
//! restarts (the device re-enumerates, it does not reboot) and return to
//! the factory behaviour only when the scanner is unplugged. The factory
//! auto-exposure mode is unknown, so turning the profile off sends nothing
//! (a scanner already switched stays fast until it is unplugged).

use super::xu::ShellLine;

/// The exposure profiles the user authorised.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ExposureProfile {
    /// Manual 6 ms exposure in an 8 ms pattern time: ~6.8 fps.
    Fast,
}

impl ExposureProfile {
    /// The authorised lines of this profile, in order: the auto exposure
    /// off, the exposure, then the pattern time.
    pub fn lines(self) -> &'static [ShellLine] {
        match self {
            Self::Fast => &[
                ShellLine::AutoExposureOff,
                ShellLine::Exposure6000Us,
                ShellLine::FrameTime8000Us,
            ],
        }
    }
}
