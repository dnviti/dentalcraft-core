//! The factory calibration stored on the scanner, read once and cached.
//!
//! **User decision, 2026-11-18 21:43.** The MINI 2 keeps its factory stereo
//! calibration in files on its own Linux. They can be read over the vendor
//! extension unit with a read-only file channel, as the vendor SDK does
//! (`libuvcReadFile`, reverse engineered on the POP4 by BR1ZB3AR/All3DScan,
//! `RE_NOTES.md`): open for reading, read in chunks, close
//! ([`super::xu::read_file`]).
//!
//! The user authorised reading, and only reading, the files of the closed
//! list [`FactoryFile`]: no path can be built from outside, no other opcode
//! is sent. Measured on the rig the same evening (serial
//! `A26307475A6H01AB8`, firmware `v4.1.54.20260203|v31.7.1.0203`):
//! `camparam.yaml` (2059 bytes, OpenCV matrices at 800x600) and `Pl.bin`
//! (40 bytes) read in well under a second with the stream idle, and the
//! stream started normally afterwards.
//!
//! The depth image is the **rectified left** view: its pinhole is `pL` of
//! `camparam.yaml` (= `Pl.bin`), fx = fy = 1142.37, cx = 163.19,
//! cy = 329.85 on that unit, not the centre of the image. Replaying the
//! recorded 24 x 15 degree revolution with it gives 0.23 degree RMS on the
//! pairwise rotations and an angle gain of 1.003, against 1.68 degree and
//! 0.947 with the field-of-view estimate (fx = fy = 1190, centred).
//!
//! The backend reads the files when a device is opened, only when no user
//! calibration file is installed ([`super::calib`]) and no copy is cached,
//! with the stream idle; the copy goes to
//! `<data dir>/dentalcraft/revopoint-usb/factory/<serial>/`, so each scanner
//! is read once. The depth scale is not in these files: it stays
//! [`super::calib::DEFAULT_DEPTH_SCALE_MM`].

use std::path::{Path, PathBuf};

use super::calib::{Calibration, default_calib_dir, parse_camparam, parse_pl_bin};
use crate::{Result, ScanError};

/// The factory files the backend may read: a closed list.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FactoryFile {
    /// `/data/camparam/camparam.yaml`: OpenCV matrices of the stereo pair.
    Camparam,
    /// `/data/camparam/Pl.bin`: the rectified left projection, `f32`.
    Pl,
}

impl FactoryFile {
    /// Path of the file on the device.
    pub fn device_path(self) -> &'static str {
        match self {
            Self::Camparam => "/data/camparam/camparam.yaml",
            Self::Pl => "/data/camparam/Pl.bin",
        }
    }

    /// Name of the cached copy.
    pub fn cache_name(self) -> &'static str {
        match self {
            Self::Camparam => "camparam.yaml",
            Self::Pl => "Pl.bin",
        }
    }
}

/// The cache directory of the scanner `serial`:
/// `<calibration dir>/factory/<serial>`; `None` without a data directory or
/// for a serial that is not a plain file name.
pub fn cache_dir(serial: &str) -> Option<PathBuf> {
    cache_dir_in(default_calib_dir().as_deref()?, serial)
}

fn cache_dir_in(calib_dir: &Path, serial: &str) -> Option<PathBuf> {
    let plain = !serial.is_empty()
        && serial.len() <= 64
        && serial
            .bytes()
            .all(|b| b.is_ascii_alphanumeric() || b == b'-' || b == b'_');
    plain.then(|| calib_dir.join("factory").join(serial))
}

/// The calibration of `bytes`, the content of `file`.
fn parse(file: FactoryFile, bytes: &[u8]) -> Result<Calibration> {
    match file {
        FactoryFile::Camparam => {
            let text = std::str::from_utf8(bytes)
                .map_err(|_| ScanError::InvalidParams("camparam.yaml is not UTF-8".into()))?;
            parse_camparam(text)
        }
        FactoryFile::Pl => parse_pl_bin(bytes),
    }
}

/// The factory calibration cached in `dir`, else read with `read` (each
/// file once, `Camparam` then `Pl`) and cached there. A cached file that
/// does not parse is an error, not a reason to read the device again.
///
/// # Errors
/// The read or parse error of the last file tried.
pub fn load_or_read(
    dir: &Path,
    mut read: impl FnMut(FactoryFile) -> Result<Vec<u8>>,
) -> Result<(Calibration, PathBuf)> {
    let files = [FactoryFile::Camparam, FactoryFile::Pl];
    for file in files {
        let path = dir.join(file.cache_name());
        if let Ok(bytes) = std::fs::read(&path) {
            return parse(file, &bytes)
                .map(|c| (c, path.clone()))
                .map_err(|e| ScanError::InvalidParams(format!("{}: {e}", path.display())));
        }
    }
    let mut last = ScanError::Device("no factory calibration file".into());
    for file in files {
        let parsed = read(file).and_then(|bytes| parse(file, &bytes).map(|c| (c, bytes)));
        match parsed {
            Ok((c, bytes)) => {
                let path = dir.join(file.cache_name());
                // a cache that cannot be written only costs a read next time
                let _ = write_atomic(&path, &bytes);
                return Ok((c, path));
            }
            Err(e) => last = e,
        }
    }
    Err(last)
}

fn write_atomic(path: &Path, bytes: &[u8]) -> std::io::Result<()> {
    if let Some(dir) = path.parent() {
        std::fs::create_dir_all(dir)?;
    }
    let tmp = path.with_extension("tmp");
    std::fs::write(&tmp, bytes)?;
    std::fs::rename(&tmp, path)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cache_dir_needs_a_plain_serial() {
        let d = Path::new("/x");
        assert_eq!(
            cache_dir_in(d, "A26307475A6H01AB8"),
            Some(PathBuf::from("/x/factory/A26307475A6H01AB8"))
        );
        for bad in ["", "..", "a/b", "a b"] {
            assert_eq!(cache_dir_in(d, bad), None, "{bad}");
        }
    }

    fn pl_bin(fx: f32, cx: f32, cy: f32) -> Vec<u8> {
        [0.0, fx, 0.0, cx, 0.0, fx, cy, 0.0, 0.0, 1.0]
            .iter()
            .flat_map(|v: &f32| v.to_le_bytes())
            .collect()
    }

    #[test]
    fn read_once_then_from_the_cache() {
        let dir = tempfile::tempdir().unwrap();
        let mut asked = Vec::new();
        // camparam missing on the device: Pl.bin is used and cached
        let (c, path) = load_or_read(dir.path(), |f| {
            asked.push(f);
            match f {
                FactoryFile::Camparam => Err(ScanError::Device("empty".into())),
                FactoryFile::Pl => Ok(pl_bin(1142.0, 163.0, 330.0)),
            }
        })
        .unwrap();
        assert_eq!(asked, [FactoryFile::Camparam, FactoryFile::Pl]);
        assert_eq!(path, dir.path().join("Pl.bin"));
        assert_eq!(
            (c.intrinsics.fx, c.intrinsics.cx, c.intrinsics.cy),
            (1142.0, 163.0, 330.0)
        );
        // the cache answers, the device is not asked again
        let (again, _) = load_or_read(dir.path(), |_| panic!("read again")).unwrap();
        assert_eq!(again, c);
        // a broken cache is an error, still without reading the device
        std::fs::write(dir.path().join("Pl.bin"), b"short").unwrap();
        assert!(load_or_read(dir.path(), |_| panic!("read again")).is_err());
    }

    #[test]
    fn nothing_readable_is_the_last_error() {
        let dir = tempfile::tempdir().unwrap();
        let e = load_or_read(dir.path(), |f| {
            Err(ScanError::Device(format!("{f:?} refused")))
        })
        .unwrap_err();
        assert!(e.to_string().contains("Pl refused"), "{e}");
        assert!(std::fs::read_dir(dir.path()).unwrap().next().is_none());
    }
}
