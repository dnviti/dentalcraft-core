//! Where the running program file is, for the scanner errors that depend
//! on it. On Windows, opening the scanner's camera fails with
//! `ERROR_NOT_FOUND` (`0x80070490`) when the program was started from a
//! network or shared drive (seen live from a virtiofs share), while the
//! same program copied to a local disk opens it: the camera access check of
//! the system, made for the program file, fails there. The device itself is
//! fine and nothing in the program can change it, so the error says where
//! the program runs from and that a copy on a local disk works.

/// `GetDriveTypeW`: the drive can be removed (USB stick, card).
const DRIVE_REMOVABLE: u32 = 2;
/// `GetDriveTypeW`: a fixed local disk.
const DRIVE_FIXED: u32 = 3;
/// `GetDriveTypeW`: a network drive.
const DRIVE_REMOTE: u32 = 4;

/// What kind of drive the program runs from when it is not a local disk:
/// `drive_type` is the `GetDriveTypeW` value of its volume, `file_system`
/// the file system name of the volume (empty when unknown). A local disk
/// is a fixed drive formatted by Windows (NTFS or ReFS); a fixed drive with
/// another file system is a shared or virtual one (WinFsp, virtiofs).
pub(super) fn not_local(drive_type: u32, file_system: &str) -> Option<&'static str> {
    let windows_fs = ["NTFS", "ReFS"]
        .iter()
        .any(|fs| file_system.eq_ignore_ascii_case(fs));
    match drive_type {
        DRIVE_FIXED if windows_fs => None,
        DRIVE_FIXED => Some("shared or virtual drive"),
        DRIVE_REMOTE => Some("network drive"),
        DRIVE_REMOVABLE => Some("removable drive"),
        _ => Some("drive that is not a local disk"),
    }
}

/// The detail of a camera that could not be opened because of where the
/// program runs from: `dir` is its folder, `what` the kind of drive.
pub(super) fn advice(detail: &str, dir: &str, what: &str) -> String {
    format!(
        "{detail}: DentalCraft runs from {dir}, a {what}, and Windows does not open the camera \
         for programs started there: copy DentalCraft to a local disk (for example C:\\) and \
         start it from there"
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn only_a_fixed_windows_volume_is_local() {
        assert_eq!(not_local(DRIVE_FIXED, "NTFS"), None);
        assert_eq!(not_local(DRIVE_FIXED, "refs"), None);
        // the virtiofs share of a virtual machine, a WinFsp drive
        assert_eq!(
            not_local(DRIVE_FIXED, "VIRTIOFS"),
            Some("shared or virtual drive")
        );
        assert_eq!(not_local(DRIVE_FIXED, ""), Some("shared or virtual drive"));
        assert_eq!(not_local(DRIVE_REMOTE, "NTFS"), Some("network drive"));
        assert_eq!(not_local(DRIVE_REMOVABLE, "exFAT"), Some("removable drive"));
        assert_eq!(not_local(0, "NTFS"), Some("drive that is not a local disk"));
    }

    #[test]
    fn the_advice_says_where_and_what_helps() {
        let a = advice(
            "MFCreateDeviceSource: Element not found. (0x80070490)",
            r"Z:\dc-test",
            "shared or virtual drive",
        );
        assert!(a.starts_with("MFCreateDeviceSource"), "{a}");
        assert!(a.contains(r"Z:\dc-test") && a.contains("local disk"), "{a}");
    }
}
