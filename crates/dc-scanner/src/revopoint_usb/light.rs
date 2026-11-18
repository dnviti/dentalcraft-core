//! The fill light of the scanner: the only vendor command of the backend.
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
//! The lines are shell commands that the scanner runs on its own Linux. They
//! travel over the UVC extension unit 4 of interface 0 (GUID
//! [`SHELL_UNIT_GUID`]), selector 7, as one `SET_CUR` of
//! [`SHELL_PAYLOAD_LEN`] bytes: the ASCII line padded with NUL bytes, as
//! the vendor SDK sends them (`cs::Camera::ledAndLaserCtrl`, reverse
//! engineered on the POP4 by BR1ZB3AR/All3DScan, `RE_NOTES.md`). Before the
//! first command the backend checks that the device has that unit with that
//! GUID and that the selector is [`SHELL_PAYLOAD_LEN`] bytes long; a device
//! that differs gets no command at all.
//!
//! Only Linux sends them (`UVCIOC_CTRL_QUERY` of `uvcvideo`); on Windows the
//! light is left as the device keeps it.

/// The fill light states the user authorised.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FillLight {
    /// Steadily on.
    On,
    /// Off.
    Off,
}

/// Id of the extension unit that runs the shell lines.
pub const SHELL_UNIT: u8 = 4;
/// Its selector for a shell line.
pub const SHELL_SELECTOR: u8 = 7;
/// Bytes of one shell line request (the line, NUL padded).
pub const SHELL_PAYLOAD_LEN: usize = 60;
/// GUID of the extension unit, `{c2211a6d-787e-3c4a-ac06-7e434b676f9a}`,
/// in USB descriptor byte order (the first three fields little-endian).
pub const SHELL_UNIT_GUID: [u8; 16] = [
    0x6d, 0x1a, 0x21, 0xc2, 0x7e, 0x78, 0x4a, 0x3c, 0xac, 0x06, 0x7e, 0x43, 0x4b, 0x67, 0x6f, 0x9a,
];

impl FillLight {
    /// The authorised lines of this state, in order.
    pub fn lines(self) -> &'static [&'static str] {
        match self {
            Self::On => &[
                "echo s 0xb03 1 >/dev/rk_preisp",
                "echo s 0xb02 1 >/dev/rk_preisp",
            ],
            Self::Off => &["echo s 0xb03 0 >/dev/rk_preisp"],
        }
    }

    /// The `SET_CUR` payloads of this state.
    pub fn payloads(self) -> Vec<[u8; SHELL_PAYLOAD_LEN]> {
        self.lines().iter().map(|l| payload(l)).collect()
    }
}

/// `line` as a shell request: ASCII, NUL padded to [`SHELL_PAYLOAD_LEN`]
/// bytes (always NUL terminated).
fn payload(line: &str) -> [u8; SHELL_PAYLOAD_LEN] {
    let mut p = [0u8; SHELL_PAYLOAD_LEN];
    let bytes = line.as_bytes();
    assert!(bytes.len() < SHELL_PAYLOAD_LEN && line.is_ascii());
    p[..bytes.len()].copy_from_slice(bytes);
    p
}

/// `true` when the raw USB configuration descriptors `descriptors` (as in
/// sysfs `descriptors`) hold a video-control extension unit with id
/// [`SHELL_UNIT`] and GUID [`SHELL_UNIT_GUID`].
pub fn has_shell_unit(descriptors: &[u8]) -> bool {
    // class-specific interface descriptor (0x24), VC_EXTENSION_UNIT (0x06):
    // bLength, bDescriptorType, bDescriptorSubtype, bUnitID, guid[16], ...
    let mut i = 0;
    while i + 2 <= descriptors.len() {
        let len = descriptors[i] as usize;
        if len < 2 || i + len > descriptors.len() {
            return false;
        }
        let d = &descriptors[i..i + len];
        if len >= 20
            && d[1] == 0x24
            && d[2] == 0x06
            && d[3] == SHELL_UNIT
            && d[4..20] == SHELL_UNIT_GUID
        {
            return true;
        }
        i += len;
    }
    false
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn only_the_authorised_lines_padded_to_60_bytes() {
        let on = FillLight::On.payloads();
        assert_eq!(on.len(), 2);
        assert_eq!(&on[0][..30], b"echo s 0xb03 1 >/dev/rk_preisp");
        assert_eq!(&on[1][..30], b"echo s 0xb02 1 >/dev/rk_preisp");
        assert!(on.iter().all(|p| p[30..].iter().all(|b| *b == 0)));
        let off = FillLight::Off.payloads();
        assert_eq!(off.len(), 1);
        assert_eq!(&off[0][..30], b"echo s 0xb03 0 >/dev/rk_preisp");
        assert!(off[0][30..].iter().all(|b| *b == 0));
    }

    #[test]
    fn the_shell_unit_is_found_by_id_and_guid() {
        // configuration header, then the extension unit of the MINI 2
        let mut d = vec![9, 2, 0, 0, 1, 1, 0, 0x80, 0xfa];
        let mut xu = vec![27, 0x24, 0x06, 4];
        xu.extend_from_slice(&SHELL_UNIT_GUID);
        xu.extend_from_slice(&[14, 1, 3, 3, 0xff, 0xa2, 0xa7, 0]);
        d.extend_from_slice(&xu);
        assert!(has_shell_unit(&d));
        // another unit id, another GUID, a truncated descriptor
        let mut other = d.clone();
        other[9 + 3] = 3;
        assert!(!has_shell_unit(&other));
        let mut other = d.clone();
        other[9 + 4] ^= 1;
        assert!(!has_shell_unit(&other));
        assert!(!has_shell_unit(&d[..20]));
    }
}
