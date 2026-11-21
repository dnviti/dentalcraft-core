//! Machine parameters: the printer firmware's own settings, as Elegoo
//! writes them in an `M5000` G-code file.
//!
//! Each line `M5000 I<group> [A<axis>] <letter><value> ;comment` sets one
//! firmware parameter; `M5999 I0` saves them. The axis selector `A` appears
//! in the axis groups I0 and I5 only: `A0` is the Z axis, `A1` the X axis,
//! which on this printer tilts the vat to release each layer. In every other
//! group the letter after the group is the parameter itself.
//!
//! The built-in set is "DENTAL 45" of the Saturn 4 Ultra 16K
//! ([`MachineParams::dental_45`]), prepared by the user with Elegoo's
//! engineers (`data/saturn-4-ultra-16k/100MachineParams_DENTAL_45.gcode`,
//! comments in Italian). DentalCraft uses:
//!
//! | key | meaning | DENTAL 45 |
//! |---|---|---|
//! | I0 A0 B | Z axis travel, mm | 230 |
//! | I0 A1 B | total tilt angle of the release, ° | 140 |
//! | I7 E, I7 F | resin (vat) target and preheat target temperature, °C | 45, 45 |
//! | I7 P | time limit of one vat heating cycle, min | 35 |
//! | I3 C | light source settings: 0 the machine's only, 1 the file's only, 2 both combined | 2 |
//! | I4 C | light source PWM, 0–255 | 255 |
//! | I4 T | vat maximum volume, ml | 870 |
//! | I4 Y | lowest resin level the level detection accepts, mm | 4 |
//! | I204 S, I204 P | vat width and length the level is turned into a volume with, mm | 143, 244 |
//! | I4 R | the firmware's resin estimate: 0 left out, 1 used | 0 |
//! | I200, I201 (set S1) | tilt speeds of the first N layers and after (°/s), N (I201 E) | 36, 72; 50 |
//! | I210, I211 (set S2) | the same for the other set | 36, 144; 50 |
//! | I212 A, B | tilt motion time per layer in slow mode, before and after N layers, ms | 8420, 4410 |
//! | I212 C, D | the same in fast mode | 8420, 2430 |
//! | I212 E | correction of the print time estimate, ms (may be negative) | 0 |
//! | I202 A | rest after the tilt returns, first N layers, s (0: the file's own) | 0 |
//!
//! The set S1 is the slow release mode and S2 the fast one \[INFERENCE\]: the
//! file names only the times of I212 slow and fast, and S2 turns twice as
//! fast after N layers (144 against 72 °/s) where the fast time after N
//! layers is about half the slow one (2430 against 4410 ms); the first N
//! layers run alike in both (36 °/s, 8420 ms). The mode itself is chosen on
//! the printer's touchscreen.
//!
//! With I3 C = 2 the light source combines the print file's settings with
//! the machine's: the exposure times, waits and PWM the GOO file carries per
//! layer are honoured (the machine's PWM, I4 C, is full).
//!
//! Every value is also kept in [`MachineParams::entries`], with the text
//! and the comment of its line, so nothing of the file is lost.

use serde::{Deserialize, Serialize};

/// The DENTAL 45 file, as shipped (see the module documentation).
pub const DENTAL_45_GCODE: &str =
    include_str!("../data/saturn-4-ultra-16k/100MachineParams_DENTAL_45.gcode");

/// Errors of the parser, code `print-machine-params`.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum MachineParamsError {
    /// A line is not a machine parameter line.
    #[error("print-machine-params: line {line}: {reason}")]
    Line {
        /// Line number, from 1.
        line: usize,
        /// What is wrong.
        reason: String,
    },
    /// A parameter DentalCraft needs is missing or out of range.
    #[error("print-machine-params: {key}: {reason}")]
    Value {
        /// The parameter (`I4 T`).
        key: String,
        /// What is wrong.
        reason: String,
    },
}

impl MachineParamsError {
    /// Stable identifier, the same in every language (kebab-case).
    pub fn code(&self) -> &'static str {
        "print-machine-params"
    }
}

/// One `M5000` value.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MachineParam {
    /// Parameter group (`I`).
    pub group: u32,
    /// Axis (`A`) of the axis groups I0 and I5: 0 Z, 1 X (tilt).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub axis: Option<u32>,
    /// Parameter letter.
    pub letter: char,
    /// The value.
    pub value: f64,
    /// The value as written (`0.000625000000`).
    pub text: String,
    /// The comment of the line, as written.
    pub comment: String,
}

impl MachineParam {
    /// The key as written in the file: `I0 A0 B`, `I7 E`.
    pub fn key(&self) -> String {
        key(self.group, self.axis, self.letter)
    }
}

fn key(group: u32, axis: Option<u32>, letter: char) -> String {
    match axis {
        Some(a) => format!("I{group} A{a} {letter}"),
        None => format!("I{group} {letter}"),
    }
}

/// How the light source takes its settings (I3 C).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum LightSource {
    /// 0: the machine's parameters only; the file's exposures are ignored.
    MachineOnly,
    /// 1: the print file's settings only.
    FileOnly,
    /// 2: the print file's settings combined with the machine's.
    Combined,
}

/// Tilt release mode, chosen on the printer's touchscreen.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ReleaseMode {
    /// Slow release (set S1).
    Slow,
    /// Fast release (set S2).
    Fast,
}

/// Tilt speeds of one stage of layers, °/s.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct TiltSpeeds {
    /// First stage of the return, high speed.
    pub return_high: f64,
    /// Second stage of the return, low speed.
    pub return_low: f64,
    /// First stage of the release, low speed.
    pub release_low: f64,
    /// Second stage of the release, high speed.
    pub release_high: f64,
}

/// One tilt release mode.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Release {
    /// Layers counted as the first ones, N (I201 E, I211 E).
    pub first_layers: u32,
    /// Tilt motion time of each of the first N layers, ms (I212 A, C).
    pub first_ms: f64,
    /// Tilt motion time of each later layer, ms (I212 B, D).
    pub after_ms: f64,
    /// Speeds of the first N layers (I200 / I210 A, B, C, D).
    pub first_speeds: TiltSpeeds,
    /// Speeds of the later layers (I200 / I210 S, P, R, T).
    pub after_speeds: TiltSpeeds,
}

/// A printer's machine parameters (see the module documentation).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MachineParams {
    /// Name of the set (`DENTAL 45`).
    pub name: String,
    /// File the set comes from.
    pub source: String,
    /// Z axis travel, mm (I0 A0 B). The profile's build height stays the
    /// print limit; the travel above it is room to lift the plate.
    pub z_travel_mm: f64,
    /// Total tilt angle of the release, ° (I0 A1 B).
    pub tilt_angle_deg: f64,
    /// Resin vat target temperature during the print, °C (I7 E).
    pub tank_target_c: f64,
    /// Preheat target temperature before the first layer, °C (I7 F).
    pub preheat_target_c: f64,
    /// Time limit of one vat heating cycle, min (I7 P).
    pub heat_cycle_limit_min: f64,
    /// How the light source takes its settings (I3 C).
    pub light_source: LightSource,
    /// Light source PWM, 0–255 (I4 C).
    pub light_pwm: u16,
    /// Vat maximum volume, ml (I4 T).
    pub vat_max_ml: f64,
    /// Lowest resin level the level detection accepts, mm (I4 Y).
    pub vat_min_level_mm: f64,
    /// Vat width and length, mm (I204 S, I204 P).
    pub vat_size_mm: [f64; 2],
    /// The firmware estimates the resin (I4 R = 1) or leaves it out (0).
    pub resin_estimate: bool,
    /// Slow release, set S1 (I200, I201, I212 A and B).
    pub slow: Release,
    /// Fast release, set S2 (I210, I211, I212 C and D).
    pub fast: Release,
    /// Correction of the print time estimate, ms per layer (I212 E).
    pub estimate_correction_ms: f64,
    /// Rest after the tilt returns in the first N layers, s; 0: the
    /// file's own rest (I202 A).
    pub first_layers_rest_s: f64,
    /// Every value, in file order.
    pub entries: Vec<MachineParam>,
    /// The file ends by saving the parameters (`M5999 I0`).
    pub saved: bool,
}

impl MachineParams {
    /// The built-in DENTAL 45 set of the Saturn 4 Ultra 16K.
    ///
    /// # Panics
    /// Never: the embedded file parses (a test checks it).
    pub fn dental_45() -> Self {
        match Self::parse(
            "DENTAL 45",
            "100MachineParams_DENTAL_45.gcode",
            DENTAL_45_GCODE,
        ) {
            Ok(p) => p,
            Err(e) => panic!("the built-in machine parameters do not parse: {e}"),
        }
    }

    /// Parses an `M5000` file. Blank lines are skipped; every other line
    /// must be `M5000` or `M5999`, and every key may appear once.
    ///
    /// # Errors
    /// [`MachineParamsError::Line`] for a line that does not parse or a key
    /// set twice, [`MachineParamsError::Value`] for a missing or unusable
    /// parameter that DentalCraft needs.
    pub fn parse(name: &str, source: &str, text: &str) -> Result<Self, MachineParamsError> {
        let mut entries: Vec<MachineParam> = Vec::new();
        let mut saved = false;
        for (i, raw) in text.lines().enumerate() {
            let line = i + 1;
            let bad = |reason: String| MachineParamsError::Line { line, reason };
            let (code, comment) = raw.split_once(';').unwrap_or((raw, ""));
            let mut tokens = code.split_whitespace();
            let Some(command) = tokens.next() else {
                continue;
            };
            let group = tokens
                .next()
                .and_then(|t| t.strip_prefix('I'))
                .and_then(|g| g.parse::<u32>().ok())
                .ok_or_else(|| bad(format!("{command} needs a group I<number>")))?;
            let rest: Vec<&str> = tokens.collect();
            match command {
                "M5999" => {
                    if group != 0 || !rest.is_empty() {
                        return Err(bad("M5999 takes only I0".into()));
                    }
                    saved = true;
                }
                "M5000" => {
                    let (axis, param) = match rest.as_slice() {
                        [param] => (None, *param),
                        [axis, param] => {
                            let a = axis
                                .strip_prefix('A')
                                .and_then(|a| a.parse::<u32>().ok())
                                .ok_or_else(|| bad(format!("{axis} is not an axis A<number>")))?;
                            (Some(a), *param)
                        }
                        _ => return Err(bad("M5000 needs one value after its group".into())),
                    };
                    let mut chars = param.chars();
                    let letter = chars
                        .next()
                        .filter(char::is_ascii_uppercase)
                        .ok_or_else(|| bad(format!("{param} does not start with a letter")))?;
                    let text = chars.as_str();
                    let value = text
                        .parse::<f64>()
                        .ok()
                        .filter(|v| v.is_finite())
                        .ok_or_else(|| bad(format!("{param}: {text:?} is not a number")))?;
                    if entries
                        .iter()
                        .any(|e| (e.group, e.axis, e.letter) == (group, axis, letter))
                    {
                        return Err(bad(format!("{} is set twice", key(group, axis, letter))));
                    }
                    entries.push(MachineParam {
                        group,
                        axis,
                        letter,
                        value,
                        text: text.to_owned(),
                        comment: comment.trim().to_owned(),
                    });
                }
                other => return Err(bad(format!("{other} is not a machine parameter command"))),
            }
        }
        let mut params = Typed { entries: &entries }.build(name, source, saved)?;
        params.entries = entries;
        Ok(params)
    }

    /// The value of a parameter: `get(0, Some(0), 'B')` is the Z travel.
    pub fn get(&self, group: u32, axis: Option<u32>, letter: char) -> Option<f64> {
        find(&self.entries, group, axis, letter)
    }

    /// The release mode's settings.
    pub fn release(&self, mode: ReleaseMode) -> &Release {
        match mode {
            ReleaseMode::Slow => &self.slow,
            ReleaseMode::Fast => &self.fast,
        }
    }

    /// Time the tilt release takes after layer `index` (0-based), s, with
    /// the estimate correction (I212 E).
    pub fn tilt_s(&self, mode: ReleaseMode, index: u32) -> f64 {
        let r = self.release(mode);
        let ms = if index < r.first_layers {
            r.first_ms
        } else {
            r.after_ms
        };
        (ms + self.estimate_correction_ms) / 1000.0
    }

    /// Rest before the exposure of layer `index`, s: the machine's own in
    /// the first N layers when it sets one (I202 A), the file's `file_s`
    /// otherwise.
    pub fn rest_s(&self, mode: ReleaseMode, index: u32, file_s: f64) -> f64 {
        if index < self.release(mode).first_layers && self.first_layers_rest_s > 0.0 {
            self.first_layers_rest_s
        } else {
            file_s
        }
    }

    /// Resin that must stay in the vat when the print ends, ml: the lowest
    /// level the printer accepts (I4 Y) over the vat floor (I204 S × P).
    pub fn vat_reserve_ml(&self) -> f64 {
        self.vat_min_level_mm * self.vat_size_mm[0] * self.vat_size_mm[1] / 1000.0
    }
}

fn find(entries: &[MachineParam], group: u32, axis: Option<u32>, letter: char) -> Option<f64> {
    entries
        .iter()
        .find(|e| (e.group, e.axis, e.letter) == (group, axis, letter))
        .map(|e| e.value)
}

/// Reads the typed fields from the entries.
struct Typed<'a> {
    entries: &'a [MachineParam],
}

impl Typed<'_> {
    fn need(&self, group: u32, axis: Option<u32>, letter: char) -> Result<f64, MachineParamsError> {
        find(self.entries, group, axis, letter).ok_or_else(|| MachineParamsError::Value {
            key: key(group, axis, letter),
            reason: "missing".into(),
        })
    }

    /// A value that must lie in `range`.
    fn within(
        &self,
        group: u32,
        axis: Option<u32>,
        letter: char,
        range: std::ops::RangeInclusive<f64>,
    ) -> Result<f64, MachineParamsError> {
        let v = self.need(group, axis, letter)?;
        if range.contains(&v) {
            Ok(v)
        } else {
            Err(MachineParamsError::Value {
                key: key(group, axis, letter),
                reason: format!("{v} is outside {}–{}", range.start(), range.end()),
            })
        }
    }

    fn speeds(&self, group: u32, letters: [char; 4]) -> Result<TiltSpeeds, MachineParamsError> {
        let [a, b, c, d] = letters;
        Ok(TiltSpeeds {
            return_high: self.need(group, None, a)?,
            return_low: self.need(group, None, b)?,
            release_low: self.need(group, None, c)?,
            release_high: self.need(group, None, d)?,
        })
    }

    /// A release mode: speeds in `speed_group`, N in `mode_group` E, times
    /// I212 `first` and `after`.
    fn release(
        &self,
        speed_group: u32,
        mode_group: u32,
        first: char,
        after: char,
    ) -> Result<Release, MachineParamsError> {
        let ms = 0.0..=600_000.0;
        Ok(Release {
            first_layers: self.within(mode_group, None, 'E', 0.0..=1e6)? as u32,
            first_ms: self.within(212, None, first, ms.clone())?,
            after_ms: self.within(212, None, after, ms)?,
            first_speeds: self.speeds(speed_group, ['A', 'B', 'C', 'D'])?,
            after_speeds: self.speeds(speed_group, ['S', 'P', 'R', 'T'])?,
        })
    }

    /// The typed fields; `entries` is left empty for the caller to fill.
    fn build(
        &self,
        name: &str,
        source: &str,
        saved: bool,
    ) -> Result<MachineParams, MachineParamsError> {
        let light_source = match self.need(3, None, 'C')? {
            0.0 => LightSource::MachineOnly,
            1.0 => LightSource::FileOnly,
            2.0 => LightSource::Combined,
            v => {
                return Err(MachineParamsError::Value {
                    key: "I3 C".into(),
                    reason: format!("{v} is not 0, 1 or 2"),
                });
            }
        };
        Ok(MachineParams {
            name: name.to_owned(),
            source: source.to_owned(),
            z_travel_mm: self.within(0, Some(0), 'B', 1.0..=10_000.0)?,
            tilt_angle_deg: self.need(0, Some(1), 'B')?,
            tank_target_c: self.within(7, None, 'E', 0.0..=100.0)?,
            preheat_target_c: self.within(7, None, 'F', 0.0..=100.0)?,
            heat_cycle_limit_min: self.need(7, None, 'P')?,
            light_source,
            light_pwm: self.within(4, None, 'C', 0.0..=255.0)? as u16,
            vat_max_ml: self.within(4, None, 'T', 1.0..=100_000.0)?,
            vat_min_level_mm: self.within(4, None, 'Y', 0.0..=1000.0)?,
            vat_size_mm: [
                self.within(204, None, 'S', 1.0..=10_000.0)?,
                self.within(204, None, 'P', 1.0..=10_000.0)?,
            ],
            resin_estimate: self.need(4, None, 'R')? != 0.0,
            slow: self.release(200, 201, 'A', 'B')?,
            fast: self.release(210, 211, 'C', 'D')?,
            estimate_correction_ms: self.need(212, None, 'E')?,
            first_layers_rest_s: self.within(202, None, 'A', 0.0..=3600.0)?,
            entries: Vec::new(),
            saved,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_dental_45_file_parses_line_by_line() {
        let p = MachineParams::dental_45();
        let lines: Vec<&str> = DENTAL_45_GCODE
            .lines()
            .filter(|l| !l.trim().is_empty())
            .collect();
        assert_eq!(lines.len(), 161);
        // every M5000 line is one entry, and the last line saves them
        let m5000 = lines.iter().filter(|l| l.starts_with("M5000")).count();
        assert_eq!(m5000, 160);
        assert_eq!(p.entries.len(), 160);
        assert!(p.saved);
        assert!(lines[160].starts_with("M5999 I0"));
        for (line, e) in lines.iter().zip(&p.entries) {
            assert!(line.contains(&format!("{}{}", e.letter, e.text)), "{line}");
            assert!(!e.comment.is_empty(), "{line}");
        }
        assert_eq!(p.name, "DENTAL 45");
    }

    #[test]
    fn the_dental_45_values() {
        let p = MachineParams::dental_45();
        assert_eq!(p.z_travel_mm, 230.0);
        assert_eq!(p.tilt_angle_deg, 140.0);
        assert_eq!((p.tank_target_c, p.preheat_target_c), (45.0, 45.0));
        assert_eq!(p.heat_cycle_limit_min, 35.0);
        assert_eq!(p.light_source, LightSource::Combined);
        assert_eq!(p.light_pwm, 255);
        assert_eq!(p.vat_max_ml, 870.0);
        assert_eq!(p.vat_min_level_mm, 4.0);
        assert_eq!(p.vat_size_mm, [143.0, 244.0]);
        assert!(!p.resin_estimate);
        assert_eq!(p.estimate_correction_ms, 0.0);
        assert_eq!(p.first_layers_rest_s, 0.0);
        assert_eq!(
            (p.slow.first_layers, p.slow.first_ms, p.slow.after_ms),
            (50, 8420.0, 4410.0)
        );
        assert_eq!(
            (p.fast.first_layers, p.fast.first_ms, p.fast.after_ms),
            (50, 8420.0, 2430.0)
        );
        assert_eq!(p.slow.first_speeds.release_high, 36.0);
        assert_eq!(p.slow.after_speeds.release_high, 72.0);
        assert_eq!(p.fast.after_speeds.return_high, 144.0);
        // the raw values, the comments and the odd spacing of group I6
        assert_eq!(p.get(0, Some(0), 'B'), Some(230.0));
        assert_eq!(p.get(0, Some(0), 'D'), Some(0.000625));
        assert_eq!(p.get(201, None, 'D'), Some(-10.0));
        assert_eq!(p.get(6, None, 'A'), Some(12.0));
        assert_eq!(p.get(6, None, 'S'), Some(18.0));
        assert_eq!(p.get(212, None, 'A'), Some(8420.0));
        assert_eq!(p.get(9, None, 'A'), None);
        let z = &p.entries[0];
        assert_eq!(z.key(), "I0 A0 B");
        assert_eq!(
            (z.text.as_str(), z.comment.as_str()),
            ("230.000000", "Corsa asse Z (mm)")
        );
        assert_eq!(p.entries[16].key(), "I6 A");
    }

    #[test]
    fn tilt_rest_and_reserve() {
        let p = MachineParams::dental_45();
        assert_eq!(p.tilt_s(ReleaseMode::Slow, 0), 8.42);
        assert_eq!(p.tilt_s(ReleaseMode::Slow, 49), 8.42);
        assert_eq!(p.tilt_s(ReleaseMode::Slow, 50), 4.41);
        assert_eq!(p.tilt_s(ReleaseMode::Fast, 49), 8.42);
        assert_eq!(p.tilt_s(ReleaseMode::Fast, 50), 2.43);
        assert_eq!(p.rest_s(ReleaseMode::Slow, 3, 1.5), 1.5);
        let mut q = p.clone();
        q.first_layers_rest_s = 2.0;
        assert_eq!(q.rest_s(ReleaseMode::Slow, 3, 1.5), 2.0);
        assert_eq!(q.rest_s(ReleaseMode::Slow, 50, 1.5), 1.5);
        // 4 mm over 143 × 244 mm
        assert!((p.vat_reserve_ml() - 139.568).abs() < 1e-9);
    }

    #[test]
    fn line_endings_do_not_matter() {
        let crlf = DENTAL_45_GCODE.replace('\n', "\r\n");
        let p =
            MachineParams::parse("DENTAL 45", "100MachineParams_DENTAL_45.gcode", &crlf).unwrap();
        assert_eq!(p, MachineParams::dental_45());
    }

    #[test]
    fn bad_files_are_refused_with_the_line() {
        let parse = |text: &str| MachineParams::parse("x", "x", text);
        let line = |text: &str| match parse(text) {
            Err(MachineParamsError::Line { line, .. }) => line,
            other => panic!("{other:?}"),
        };
        assert_eq!(line("\nG28\n"), 2);
        assert_eq!(line("M5000 X1 B2"), 1);
        assert_eq!(line("M5000 I0 A0 B1 C2"), 1);
        assert_eq!(line("M5000 I0 Q0 B1"), 1);
        assert_eq!(line("M5000 I7 E4five"), 1);
        assert_eq!(line("M5000 I7 Enan"), 1);
        assert_eq!(line("M5000 I7 e45"), 1);
        assert_eq!(line("M5000 I7 E45 F45"), 1);
        assert_eq!(line("M5999 I1"), 1);
        assert_eq!(line("M5000 I7 E45\nM5000 I7 E46"), 2);
        // a needed value missing
        let without_vat: String = DENTAL_45_GCODE
            .lines()
            .filter(|l| !l.starts_with("M5000 I4 T"))
            .map(|l| format!("{l}\n"))
            .collect();
        match parse(&without_vat) {
            Err(e @ MachineParamsError::Value { .. }) => {
                assert!(e.to_string().contains("I4 T"), "{e}");
                assert_eq!(e.code(), "print-machine-params");
            }
            other => panic!("{other:?}"),
        }
        let machine_only = DENTAL_45_GCODE.replace("M5000 I3 C2", "M5000 I3 C0");
        assert_eq!(
            parse(&machine_only).unwrap().light_source,
            LightSource::MachineOnly
        );
        let bad_mode = DENTAL_45_GCODE.replace("M5000 I3 C2", "M5000 I3 C7");
        assert!(parse(&bad_mode).is_err());
    }
}
