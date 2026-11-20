//! Calibration of the depth image: pinhole intrinsics and depth scale.
//!
//! It comes from, in this order:
//!
//! 1. the file named by [`CALIB_ENV`], when set (it must exist);
//! 2. `calib.yaml`, else `calib.json`, in [`default_calib_dir`]
//!    (`<data dir>/dentalcraft/revopoint-usb/`);
//! 3. the factory calibration of the scanner, read from the device once and
//!    cached ([`super::factory`]);
//! 4. the alpha defaults ([`Calibration::default`]).
//!
//! A file overrides the defaults key by key. Keys (millimetres and pixels):
//! `fx`, `fy`, `cx`, `cy`, `depth_scale` (mm per raw depth unit), and
//! optionally `width` / `height`, the image size the intrinsics refer to
//! (default 800x600; other sizes are scaled to the 800x600 depth image).
//! Unknown keys are ignored.
//!
//! * JSON: one object, e.g. `{"fx": 1190, "fy": 1190, "cx": 399.5,
//!   "cy": 299.5, "depth_scale": 0.004}`.
//! * YAML: `key: number` lines (OpenCV `%YAML:1.0` headers, `---`, comments
//!   and nesting are accepted; a key is found at any depth).
//! * The vendor `camparam.yaml` (OpenCV matrices, [`parse_camparam`]) is
//!   recognised by its `!!opencv-matrix` tags and used as is: the pinhole
//!   is its rectified left projection `pL`.

use std::ffi::OsStr;
use std::path::{Path, PathBuf};

use serde::Deserialize;

use super::frame::{DEPTH_HEIGHT, DEPTH_WIDTH};
use crate::{CameraIntrinsics, Result, ScanError};

/// Environment variable naming a calibration file (YAML or JSON).
pub const CALIB_ENV: &str = "DC_REVOPOINT_CALIB";
/// Calibration file names searched in [`default_calib_dir`], in order.
pub const CALIB_FILES: [&str; 2] = ["calib.yaml", "calib.json"];

/// Default focal length (pixels at 800 px width), from the published field
/// of view of the MINI 2: 168 mm wide at 250 mm, so `800 * 250 / 168`.
/// Alpha estimate, not a calibration.
pub const DEFAULT_FOCAL_PX: f32 = 1190.0;
/// Default millimetres per raw depth unit: 1/200 mm. **Alpha estimate**,
/// measured on one MINI 2 (serial `A26307475A6H01AB8`, 2026-11-16), not a
/// factory value; a calibration file overrides it. Evidence:
///
/// * the raw value is linear depth along the optical axis, not disparity:
///   it grows with distance (a keyboard behind the target reads higher),
///   as in Revo Scan caches where depth is `u16 * depth_scale`;
/// * the nearest surface of a target measured with a ruler at 180 mm from
///   the front glass reads raw ~36 080 (1st percentile), i.e. 180.4 mm;
/// * with [`DEFAULT_FOCAL_PX`], the thumbstick caps of the target come out
///   ~13.7 mm wide, and the parallax of the infrared pair against `1/raw`
///   gives a stereo baseline of ~82 mm, plausible for the scanner;
/// * 1/200 mm makes the `u16` range span 0-327 mm, just above the
///   120-250 mm working range.
///
/// The other candidates fail the distance check: 1 mm, 0.1 mm, 0.125 mm and
/// 1/32 mm put that surface at 36 m, 3.6 m, 4.5 m and 1.1 m.
pub const DEFAULT_DEPTH_SCALE_MM: f32 = 1.0 / 200.0;

/// Intrinsics of the 800x600 depth image and the depth unit.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Calibration {
    /// Pinhole intrinsics of the 800x600 depth image.
    pub intrinsics: CameraIntrinsics,
    /// Millimetres per raw depth unit.
    pub depth_scale_mm: f32,
}

impl Default for Calibration {
    /// The alpha estimate: [`DEFAULT_FOCAL_PX`] for both axes, principal
    /// point at the image centre, [`DEFAULT_DEPTH_SCALE_MM`].
    fn default() -> Self {
        Self {
            intrinsics: CameraIntrinsics {
                width: DEPTH_WIDTH,
                height: DEPTH_HEIGHT,
                fx: DEFAULT_FOCAL_PX,
                fy: DEFAULT_FOCAL_PX,
                cx: (DEPTH_WIDTH as f32 - 1.0) * 0.5,
                cy: (DEPTH_HEIGHT as f32 - 1.0) * 0.5,
            },
            depth_scale_mm: DEFAULT_DEPTH_SCALE_MM,
        }
    }
}

/// Where a [`Calibration`] comes from.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum CalibrationSource {
    /// No file: the alpha defaults (a field-of-view estimate).
    Defaults,
    /// This file, over the defaults.
    File(PathBuf),
    /// The factory calibration read from the scanner when it was opened,
    /// cached in this file ([`super::factory`]).
    Factory(PathBuf),
    /// The factory calibration of the scanner cached in this file when an
    /// earlier opening read it: the scanner was not asked.
    FactoryCached(PathBuf),
}

impl CalibrationSource {
    /// The stable name of the source ([`crate::VendorStatus::calibration`]):
    /// `estimate`, `user`, `factory` or `factory-cached`.
    pub fn name(&self) -> &'static str {
        match self {
            Self::Defaults => "estimate",
            Self::File(_) => "user",
            Self::Factory(_) => "factory",
            Self::FactoryCached(_) => "factory-cached",
        }
    }
}

/// The keys of a calibration file; each one overrides its default.
#[derive(Debug, Default, Deserialize, PartialEq)]
struct CalibrationKeys {
    fx: Option<f32>,
    fy: Option<f32>,
    cx: Option<f32>,
    cy: Option<f32>,
    depth_scale: Option<f32>,
    width: Option<u32>,
    height: Option<u32>,
}

impl CalibrationKeys {
    fn from_yaml(text: &str) -> Result<Self> {
        let mut keys = Self::default();
        for (n, line) in text.lines().enumerate() {
            let line = line.split('#').next().unwrap_or_default().trim();
            if line.is_empty() || line.starts_with('%') || line.starts_with("---") {
                continue;
            }
            let Some((key, value)) = line.split_once(':') else {
                continue;
            };
            let key = key.trim().trim_matches(['"', '\'']);
            let value = value.trim();
            let float = || {
                value.parse::<f32>().map_err(|_| {
                    ScanError::InvalidParams(format!("line {}: `{key}` is not a number", n + 1))
                })
            };
            match key {
                "fx" => keys.fx = Some(float()?),
                "fy" => keys.fy = Some(float()?),
                "cx" => keys.cx = Some(float()?),
                "cy" => keys.cy = Some(float()?),
                "depth_scale" => keys.depth_scale = Some(float()?),
                "width" | "height" => {
                    let v = value.parse::<u32>().map_err(|_| {
                        ScanError::InvalidParams(format!(
                            "line {}: `{key}` is not a pixel count",
                            n + 1
                        ))
                    })?;
                    if key == "width" {
                        keys.width = Some(v);
                    } else {
                        keys.height = Some(v);
                    }
                }
                _ => {}
            }
        }
        Ok(keys)
    }

    /// The defaults with these keys applied, checked.
    fn apply(&self) -> Result<Calibration> {
        if *self == Self::default() {
            return Err(ScanError::InvalidParams(
                "no calibration key (fx, fy, cx, cy, depth_scale)".into(),
            ));
        }
        let d = Calibration::default();
        let (w, h) = (
            self.width.unwrap_or(DEPTH_WIDTH),
            self.height.unwrap_or(DEPTH_HEIGHT),
        );
        if w == 0 || h == 0 {
            return Err(ScanError::InvalidParams(
                "width and height must be > 0".into(),
            ));
        }
        // keys given at another resolution are scaled to the depth image;
        // missing ones take the default, already at the depth resolution
        let (sx, sy) = (
            DEPTH_WIDTH as f32 / w as f32,
            DEPTH_HEIGHT as f32 / h as f32,
        );
        let k = CameraIntrinsics {
            width: DEPTH_WIDTH,
            height: DEPTH_HEIGHT,
            fx: self.fx.map_or(d.intrinsics.fx, |v| v * sx),
            fy: self.fy.map_or(d.intrinsics.fy, |v| v * sy),
            cx: self.cx.map_or(d.intrinsics.cx, |v| (v + 0.5) * sx - 0.5),
            cy: self.cy.map_or(d.intrinsics.cy, |v| (v + 0.5) * sy - 0.5),
        };
        let depth_scale_mm = self.depth_scale.unwrap_or(d.depth_scale_mm);
        let positive = [k.fx, k.fy, depth_scale_mm]
            .iter()
            .all(|v| v.is_finite() && *v > 0.0);
        if !positive || !k.cx.is_finite() || !k.cy.is_finite() {
            return Err(ScanError::InvalidParams(
                "fx, fy and depth_scale must be > 0, cx and cy finite".into(),
            ));
        }
        Ok(Calibration {
            intrinsics: k,
            depth_scale_mm,
        })
    }
}

/// Parses a calibration file: JSON when the text starts with `{`, else the
/// YAML subset described in the module doc.
///
/// # Errors
/// [`ScanError::InvalidParams`] for malformed text, values out of range, or
/// a file with none of the keys.
pub fn parse_calibration(text: &str) -> Result<Calibration> {
    if text.contains(OPENCV_MATRIX_TAG) {
        return parse_camparam(text);
    }
    let keys = if text.trim_start().starts_with('{') {
        serde_json::from_str::<CalibrationKeys>(text)
            .map_err(|e| ScanError::InvalidParams(e.to_string()))?
    } else {
        CalibrationKeys::from_yaml(text)?
    };
    keys.apply()
}

/// Tag of a matrix in an OpenCV YAML file.
const OPENCV_MATRIX_TAG: &str = "!!opencv-matrix";

/// The top-level scalar `name` of an OpenCV YAML file, without quotes.
fn opencv_scalar<'a>(text: &'a str, name: &str) -> Option<&'a str> {
    text.lines().find_map(|line| {
        let rest = line.strip_prefix(name)?.trim_start().strip_prefix(':')?;
        Some(rest.trim().trim_matches(['"', '\'']))
    })
}

/// The top-level matrix `name` of an OpenCV YAML file (`name:
/// !!opencv-matrix`, then `rows`, `cols`, `dt` and `data: [..]`, the data
/// possibly over several lines): `(rows, cols, row-major data)`.
///
/// # Errors
/// [`ScanError::InvalidParams`] for a matrix without sizes, with a number
/// that does not parse or with `rows * cols` values missing.
fn opencv_matrix(text: &str, name: &str) -> Result<Option<(usize, usize, Vec<f64>)>> {
    let mut lines = text.lines();
    while let Some(line) = lines.next() {
        let Some(rest) = line
            .strip_prefix(name)
            .and_then(|r| r.trim_start().strip_prefix(':'))
        else {
            continue;
        };
        if !rest.contains(OPENCV_MATRIX_TAG) {
            continue;
        }
        let (mut rows, mut cols, mut data) = (None, None, None::<String>);
        for l in lines.by_ref() {
            let t = l.trim();
            if let Some(d) = data.as_mut() {
                d.push(' ');
                d.push_str(t);
            } else if let Some(v) = t.strip_prefix("rows:") {
                rows = v.trim().parse::<usize>().ok();
            } else if let Some(v) = t.strip_prefix("cols:") {
                cols = v.trim().parse::<usize>().ok();
            } else if let Some(v) = t.strip_prefix("data:") {
                data = Some(v.trim().to_owned());
            } else if !t.is_empty() && !l.starts_with([' ', '\t']) {
                break;
            }
            if data.as_deref().is_some_and(|d| d.contains(']')) {
                break;
            }
        }
        let bad = |what: &str| ScanError::InvalidParams(format!("matrix `{name}`: {what}"));
        let (Some(rows), Some(cols)) = (rows, cols) else {
            return Err(bad("no rows/cols"));
        };
        let data = data.ok_or_else(|| bad("no data"))?;
        let inner = data
            .trim()
            .strip_prefix('[')
            .and_then(|d| d.split_once(']'))
            .map(|(d, _)| d)
            .ok_or_else(|| bad("data is not a [list]"))?;
        let values = inner
            .split(',')
            .map(|v| v.trim().parse::<f64>().map_err(|_| bad("not a number")))
            .collect::<Result<Vec<f64>>>()?;
        if values.len() != rows * cols {
            return Err(bad("wrong number of values"));
        }
        return Ok(Some((rows, cols, values)));
    }
    Ok(None)
}

/// Parses the vendor `camparam.yaml` (OpenCV YAML, see [`super::factory`]):
/// the pinhole is `pL`, the projection of the rectified left camera, which
/// is the view of the depth image, at `width` x `height` (default
/// 800x600). The depth scale keeps its default (the file has none).
///
/// # Errors
/// [`ScanError::InvalidParams`] without a 3x4 `pL` or for values out of
/// range.
pub fn parse_camparam(text: &str) -> Result<Calibration> {
    let size = |name: &str| {
        opencv_scalar(text, name)
            .map(|v| {
                v.parse::<u32>().map_err(|_| {
                    ScanError::InvalidParams(format!("camparam `{name}` is not a pixel count"))
                })
            })
            .transpose()
    };
    let (width, height) = (size("width")?, size("height")?);
    let p = match opencv_matrix(text, "pL")? {
        Some((3, 4, p)) => p,
        Some(_) => return Err(ScanError::InvalidParams("camparam `pL` is not 3x4".into())),
        None => return Err(ScanError::InvalidParams("camparam has no `pL`".into())),
    };
    CalibrationKeys {
        fx: Some(p[0] as f32),
        fy: Some(p[5] as f32),
        cx: Some(p[2] as f32),
        cy: Some(p[6] as f32),
        depth_scale: None,
        width,
        height,
    }
    .apply()
}

/// Parses the vendor `Pl.bin`: ten little-endian `f32`, one pad then the
/// 3x3 pinhole of the rectified left camera (`fx 0 cx / 0 fy cy / 0 0 1`),
/// at the 800x600 of the depth image (as `camparam.yaml` of the MINI 2
/// states for the same values).
///
/// # Errors
/// [`ScanError::InvalidParams`] for another size or not a pinhole.
pub fn parse_pl_bin(bytes: &[u8]) -> Result<Calibration> {
    let v: Vec<f32> = bytes
        .as_chunks::<4>()
        .0
        .iter()
        .map(|b| f32::from_le_bytes(*b))
        .collect();
    let pinhole =
        v.len() == 10 && bytes.len() == 40 && [v[2], v[4], v[7], v[8]] == [0.0; 4] && v[9] == 1.0;
    if !pinhole {
        return Err(ScanError::InvalidParams(
            "Pl.bin is not 40 bytes of a 3x3 pinhole".into(),
        ));
    }
    CalibrationKeys {
        fx: Some(v[1]),
        fy: Some(v[5]),
        cx: Some(v[3]),
        cy: Some(v[6]),
        ..CalibrationKeys::default()
    }
    .apply()
}

/// The per-user calibration directory, `<data dir>/dentalcraft/revopoint-usb`
/// (the data directory as for the SDK backend). `None` when the environment
/// does not define it.
pub fn default_calib_dir() -> Option<PathBuf> {
    crate::revopoint_sdk::data_dir().map(|d| d.join("dentalcraft").join("revopoint-usb"))
}

/// The calibration file to load: `env_value` (the value of [`CALIB_ENV`])
/// when set and not empty, else the first of [`CALIB_FILES`] present in
/// `dir`; `None` without a file.
///
/// # Errors
/// [`ScanError::InvalidParams`] when [`CALIB_ENV`] names a missing file.
fn resolve_calibration(env_value: Option<&OsStr>, dir: Option<&Path>) -> Result<Option<PathBuf>> {
    if let Some(v) = env_value.filter(|v| !v.is_empty()) {
        let p = PathBuf::from(v);
        return if p.is_file() {
            Ok(Some(p))
        } else {
            Err(ScanError::InvalidParams(format!(
                "{CALIB_ENV}: {} is not a file",
                p.display()
            )))
        };
    }
    Ok(dir.and_then(|d| CALIB_FILES.iter().map(|f| d.join(f)).find(|p| p.is_file())))
}

/// Loads the calibration from `env_value` / `dir` as in the module doc.
fn load_from(
    env_value: Option<&OsStr>,
    dir: Option<&Path>,
) -> Result<(Calibration, CalibrationSource)> {
    let Some(path) = resolve_calibration(env_value, dir)? else {
        return Ok((Calibration::default(), CalibrationSource::Defaults));
    };
    let calib = std::fs::read_to_string(&path)
        .map_err(|e| ScanError::InvalidParams(e.to_string()))
        .and_then(|text| parse_calibration(&text))
        .map_err(|e| {
            ScanError::InvalidParams(format!("calibration file {}: {e}", path.display()))
        })?;
    Ok((calib, CalibrationSource::File(path)))
}

/// Loads the calibration from [`CALIB_ENV`], else [`default_calib_dir`],
/// else the defaults.
///
/// # Errors
/// [`ScanError::InvalidParams`] for a missing [`CALIB_ENV`] file or an
/// invalid file.
pub fn load_calibration() -> Result<(Calibration, CalibrationSource)> {
    load_from(
        std::env::var_os(CALIB_ENV).as_deref(),
        default_calib_dir().as_deref(),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn json_and_yaml_override_key_by_key() {
        let d = Calibration::default();
        let c =
            parse_calibration(r#"{"fx": 1000, "depth_scale": 0.03125, "other": [1, 2]}"#).unwrap();
        assert_eq!(c.intrinsics.fx, 1000.0);
        assert_eq!(c.intrinsics.fy, d.intrinsics.fy);
        assert_eq!(c.depth_scale_mm, 0.03125);

        let yaml = "%YAML:1.0\n---\n# MINI 2\ncamera:\n  fx: 1180.5\n  fy: 1181. # comment\n  \
                    cx: 401\n  cy: 302.25\ndepth_scale: 0.005\nname: mini2\n";
        let c = parse_calibration(yaml).unwrap();
        let k = c.intrinsics;
        assert_eq!((k.width, k.height), (800, 600));
        assert_eq!((k.fx, k.fy, k.cx, k.cy), (1180.5, 1181.0, 401.0, 302.25));
        assert_eq!(c.depth_scale_mm, 0.005);
    }

    #[test]
    fn intrinsics_at_another_resolution_are_scaled() {
        let c = parse_calibration(
            "width: 1600\nheight: 1200\nfx: 2400\nfy: 2400\ncx: 799.5\ncy: 599.5",
        )
        .unwrap();
        let k = c.intrinsics;
        assert_eq!((k.fx, k.fy, k.cx, k.cy), (1200.0, 1200.0, 399.5, 299.5));
        assert_eq!(c.depth_scale_mm, DEFAULT_DEPTH_SCALE_MM);
    }

    /// The layout of the vendor `camparam.yaml` (values made up).
    const CAMPARAM: &str = "%YAML:1.0\n---\ndeviceID: \"X1\"\nwidth: \"800\"\nheight: \"600\"\n\
        cameraMatrixL: !!opencv-matrix\n   rows: 3\n   cols: 3\n   dt: d\n   data: [ 1.1e+03, 0., \
        4.1e+02, 0.,\n       1.1e+03, 3.3e+02, 0., 0., 1. ]\n\
        pL: !!opencv-matrix\n   rows: 3\n   cols: 4\n   dt: d\n   data: [ 1.15e+03, 0., \
        1.6e+02, 0., 0.,\n       1.15e+03, 3.25e+02, 0., 0., 0., 1.,\n       0. ]\n\
        pR: !!opencv-matrix\n   rows: 3\n   cols: 4\n   dt: d\n   data: [ 1.15e+03, 0., \
        6.5e+02, -1.0e+05, 0., 1.15e+03, 3.25e+02, 0., 0., 0., 1., 0. ]\n";

    #[test]
    fn vendor_camparam_uses_the_rectified_left_projection() {
        for c in [
            parse_camparam(CAMPARAM).unwrap(),
            parse_calibration(CAMPARAM).unwrap(),
        ] {
            let k = c.intrinsics;
            assert_eq!((k.width, k.height), (800, 600));
            assert_eq!((k.fx, k.fy, k.cx, k.cy), (1150.0, 1150.0, 160.0, 325.0));
            assert_eq!(c.depth_scale_mm, DEFAULT_DEPTH_SCALE_MM);
        }
        // at another resolution the pinhole is scaled to the depth image
        let big = CAMPARAM
            .replace("\"800\"", "\"1600\"")
            .replace("\"600\"", "\"1200\"");
        let k = parse_camparam(&big).unwrap().intrinsics;
        assert_eq!((k.fx, k.fy, k.cx, k.cy), (575.0, 575.0, 79.75, 162.25));
        // no pL, a pL of another size, a truncated or garbled pL
        for bad in [
            CAMPARAM.replace("pL:", "pX:"),
            CAMPARAM.replacen("cols: 4", "cols: 3", 1),
            CAMPARAM.replace("       0. ]\n", "]\n"),
            CAMPARAM.replace("1.6e+02", "abc"),
            CAMPARAM.replace("width: \"800\"", "width: \"wide\""),
        ] {
            assert_eq!(
                parse_camparam(&bad).unwrap_err().code(),
                "invalid-params",
                "{bad}"
            );
        }
    }

    #[test]
    fn pl_bin_is_a_pinhole_of_ten_floats() {
        let floats = |v: [f32; 10]| -> Vec<u8> { v.iter().flat_map(|f| f.to_le_bytes()).collect() };
        let ok = floats([0.0, 1142.5, 0.0, 163.25, 0.0, 1142.5, 329.75, 0.0, 0.0, 1.0]);
        let k = parse_pl_bin(&ok).unwrap().intrinsics;
        assert_eq!((k.fx, k.fy, k.cx, k.cy), (1142.5, 1142.5, 163.25, 329.75));
        assert!(parse_pl_bin(&ok[..36]).is_err());
        assert!(parse_pl_bin(&[ok.clone(), vec![0]].concat()).is_err());
        let not_pinhole = floats([0.0, 1142.5, 0.0, 163.25, 0.0, 1142.5, 329.75, 0.0, 0.0, 2.0]);
        assert!(parse_pl_bin(&not_pinhole).is_err());
        let negative = floats([0.0, -1.0, 0.0, 163.25, 0.0, 1142.5, 329.75, 0.0, 0.0, 1.0]);
        assert!(parse_pl_bin(&negative).is_err());
    }

    #[test]
    fn bad_files_are_invalid_params() {
        for text in [
            "",
            "name: x",
            "{}",
            "{not json",
            "fx: abc",
            "fx: -1",
            "depth_scale: 0",
            "width: 0\nfx: 10",
            r#"{"fx": "1190"}"#,
        ] {
            let e = parse_calibration(text).unwrap_err();
            assert_eq!(e.code(), "invalid-params", "{text}: {e}");
        }
    }

    #[test]
    fn calibration_path_resolution() {
        let dir = tempfile::tempdir().unwrap();
        let d = Some(dir.path());
        // nothing: defaults
        assert_eq!(resolve_calibration(None, d).unwrap(), None);
        assert_eq!(resolve_calibration(None, None).unwrap(), None);
        assert_eq!(
            load_from(None, d).unwrap(),
            (Calibration::default(), CalibrationSource::Defaults)
        );
        // calib.json, then calib.yaml first
        let json = dir.path().join("calib.json");
        std::fs::write(&json, r#"{"depth_scale": 0.1}"#).unwrap();
        assert_eq!(resolve_calibration(None, d).unwrap(), Some(json.clone()));
        let yaml = dir.path().join("calib.yaml");
        std::fs::write(&yaml, "depth_scale: 0.2").unwrap();
        assert_eq!(resolve_calibration(None, d).unwrap(), Some(yaml.clone()));
        let (c, src) = load_from(None, d).unwrap();
        assert_eq!(
            (c.depth_scale_mm, src),
            (0.2, CalibrationSource::File(yaml))
        );
        // the environment wins; empty means unset; a missing file is an error
        let (c, _) = load_from(Some(json.as_os_str()), d).unwrap();
        assert_eq!(c.depth_scale_mm, 0.1);
        assert_eq!(
            resolve_calibration(Some(OsStr::new("")), d).unwrap(),
            Some(dir.path().join("calib.yaml"))
        );
        let missing = dir.path().join("nope.yaml");
        let e = load_from(Some(missing.as_os_str()), d).unwrap_err();
        assert_eq!(e.code(), "invalid-params");
        assert!(e.to_string().contains(CALIB_ENV), "{e}");
        // an invalid file names itself
        std::fs::write(&json, "{}").unwrap();
        let e = load_from(Some(json.as_os_str()), d).unwrap_err();
        assert!(e.to_string().contains("calib.json"), "{e}");
        if let Some(d) = default_calib_dir() {
            assert!(d.ends_with(Path::new("dentalcraft").join("revopoint-usb")));
            assert!(d.is_absolute());
        }
    }
}
