//! Calibration of the depth image: pinhole intrinsics and depth scale.
//!
//! The backend sends nothing to the device, so it cannot read the factory
//! calibration. It uses, in this order:
//!
//! 1. the file named by [`CALIB_ENV`], when set (it must exist);
//! 2. `calib.yaml`, else `calib.json`, in [`default_calib_dir`]
//!    (`<data dir>/dentalcraft/revopoint-usb/`);
//! 3. the alpha defaults ([`Calibration::default`]).
//!
//! A file overrides the defaults key by key. Keys (millimetres and pixels):
//! `fx`, `fy`, `cx`, `cy`, `depth_scale` (mm per raw depth unit), and
//! optionally `width` / `height`, the image size the intrinsics refer to
//! (default 800x600; other sizes are scaled to the 800x600 depth image).
//! Unknown keys are ignored, so a vendor `camparam.yaml` that uses these
//! names can be used as is.
//!
//! * JSON: one object, e.g. `{"fx": 1190, "fy": 1190, "cx": 399.5,
//!   "cy": 299.5, "depth_scale": 0.004}`.
//! * YAML: `key: number` lines (OpenCV `%YAML:1.0` headers, `---`, comments
//!   and nesting are accepted; a key is found at any depth).

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
    /// No file: the alpha defaults.
    Defaults,
    /// This file, over the defaults.
    File(PathBuf),
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
    let keys = if text.trim_start().starts_with('{') {
        serde_json::from_str::<CalibrationKeys>(text)
            .map_err(|e| ScanError::InvalidParams(e.to_string()))?
    } else {
        CalibrationKeys::from_yaml(text)?
    };
    keys.apply()
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
