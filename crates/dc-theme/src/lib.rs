//! Design tokens and their application to egui.
//!
//! The tokens live in `tokens/tokens.json` (exported from the FigCraft
//! design system), are embedded at compile time and parsed once on first
//! use. The structure is flat:
//!
//! ```json
//! { "color": { "dark": { "bg": "#14171C", ... }, "light": { ... } },
//!   "font": { "size-body": 13.5, ... }, "space": { "md": 8, ... },
//!   "radius": { "md": 6, ... },
//!   "shadow": { "window": { "x": 0, "y": 4, "blur": 16, "spread": 0, "color": "#00000060" } } }
//! ```
//!
//! [`Theme::apply`] maps them onto egui's [`egui::Style`]. Colour names in
//! [`REQUIRED_COLORS`] must exist in both modes.

use std::collections::BTreeMap;
use std::sync::LazyLock;

use egui::{Color32, CornerRadius, FontId, Margin, Stroke, TextStyle, vec2};
use serde::Deserialize;

/// Raw JSON of the embedded tokens.
pub const TOKENS_JSON: &str = include_str!("../tokens/tokens.json");

/// Colour names every mode must define.
pub const REQUIRED_COLORS: &[&str] = &[
    "bg",
    "surface",
    "surface-raised",
    "input",
    "border",
    "hover",
    "text",
    "text-muted",
    "accent",
    "accent-text",
    "link",
    "success",
    "warning",
    "error",
];

/// Light or dark appearance.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum Mode {
    /// Light background.
    Light,
    /// Dark background (default).
    #[default]
    Dark,
}

impl Mode {
    fn egui(self) -> egui::Theme {
        match self {
            Mode::Light => egui::Theme::Light,
            Mode::Dark => egui::Theme::Dark,
        }
    }
}

/// Colour tokens per mode (`name -> "#RRGGBB[AA]"`).
#[derive(Debug, Clone, Default, Deserialize)]
pub struct ColorTokens {
    /// Dark mode colours.
    pub dark: BTreeMap<String, String>,
    /// Light mode colours.
    pub light: BTreeMap<String, String>,
}

/// A drop shadow.
#[derive(Debug, Clone, PartialEq, Deserialize)]
pub struct ShadowToken {
    /// Horizontal offset (px).
    pub x: f32,
    /// Vertical offset (px).
    pub y: f32,
    /// Blur radius (px).
    pub blur: f32,
    /// Spread (px).
    #[serde(default)]
    pub spread: f32,
    /// Colour, `#RRGGBB[AA]`.
    pub color: String,
}

/// All design tokens.
#[derive(Debug, Clone, Default, Deserialize)]
pub struct Tokens {
    /// Colours.
    pub color: ColorTokens,
    /// Typography (sizes in px as numbers, families as strings).
    #[serde(default)]
    pub font: BTreeMap<String, serde_json::Value>,
    /// Spacing scale (px).
    #[serde(default)]
    pub space: BTreeMap<String, f32>,
    /// Corner radius scale (px).
    #[serde(default)]
    pub radius: BTreeMap<String, f32>,
    /// Shadows.
    #[serde(default)]
    pub shadow: BTreeMap<String, ShadowToken>,
}

/// Parses `#RRGGBB` or `#RRGGBBAA`.
pub fn parse_hex(s: &str) -> Option<Color32> {
    let s = s.strip_prefix('#').unwrap_or(s);
    if !matches!(s.len(), 6 | 8) || !s.is_ascii() {
        return None;
    }
    let c = |i: usize| u8::from_str_radix(&s[i..i + 2], 16).ok();
    let a = if s.len() == 8 { c(6)? } else { 255 };
    Some(Color32::from_rgba_unmultiplied(c(0)?, c(2)?, c(4)?, a))
}

impl Tokens {
    /// Parses tokens from JSON.
    pub fn from_json(json: &str) -> Result<Self, serde_json::Error> {
        serde_json::from_str(json)
    }

    /// Colour `name` in `mode`, if defined and valid.
    pub fn color(&self, mode: Mode, name: &str) -> Option<Color32> {
        let map = match mode {
            Mode::Dark => &self.color.dark,
            Mode::Light => &self.color.light,
        };
        map.get(name).and_then(|h| parse_hex(h))
    }

    /// Font size `name` (px), if defined.
    pub fn font_size(&self, name: &str) -> Option<f32> {
        self.font.get(name)?.as_f64().map(|v| v as f32)
    }

    /// Spacing `name` (px), if defined.
    pub fn space(&self, name: &str) -> Option<f32> {
        self.space.get(name).copied()
    }

    /// Corner radius `name` (px), if defined.
    pub fn radius(&self, name: &str) -> Option<f32> {
        self.radius.get(name).copied()
    }

    /// Problems with the tokens: missing or invalid required colours,
    /// colour names defined in one mode only, invalid shadow colours.
    pub fn problems(&self) -> Vec<String> {
        let mut out = Vec::new();
        for mode in [Mode::Dark, Mode::Light] {
            for name in REQUIRED_COLORS {
                if self.color(mode, name).is_none() {
                    out.push(format!("{mode:?}: missing or invalid colour {name}"));
                }
            }
        }
        for k in self
            .color
            .dark
            .keys()
            .filter(|k| !self.color.light.contains_key(*k))
        {
            out.push(format!("colour {k} only in dark mode"));
        }
        for k in self
            .color
            .light
            .keys()
            .filter(|k| !self.color.dark.contains_key(*k))
        {
            out.push(format!("colour {k} only in light mode"));
        }
        for (k, s) in &self.shadow {
            if parse_hex(&s.color).is_none() {
                out.push(format!("shadow {k}: invalid colour {}", s.color));
            }
        }
        out
    }
}

static TOKENS: LazyLock<Tokens> = LazyLock::new(|| {
    Tokens::from_json(TOKENS_JSON).expect("tokens/tokens.json is valid (checked by tests)")
});

fn radius(px: Option<f32>) -> Option<CornerRadius> {
    px.map(|r| CornerRadius::same(r.round().clamp(0.0, 255.0) as u8))
}

fn shadow(t: Option<&ShadowToken>) -> Option<egui::Shadow> {
    let t = t?;
    let i8c = |v: f32| v.round().clamp(-128.0, 127.0) as i8;
    let u8c = |v: f32| v.round().clamp(0.0, 255.0) as u8;
    Some(egui::Shadow {
        offset: [i8c(t.x), i8c(t.y)],
        blur: u8c(t.blur),
        spread: u8c(t.spread),
        color: parse_hex(&t.color)?,
    })
}

/// The DentalCraft theme.
pub struct Theme;

impl Theme {
    /// The embedded design tokens.
    pub fn tokens() -> &'static Tokens {
        &TOKENS
    }

    /// egui visuals for `mode` built from the tokens (missing tokens keep
    /// egui's defaults).
    pub fn visuals(mode: Mode) -> egui::Visuals {
        let t = Self::tokens();
        let mut v = match mode {
            Mode::Dark => egui::Visuals::dark(),
            Mode::Light => egui::Visuals::light(),
        };
        let c = |name: &str| t.color(mode, name);
        if let Some(x) = c("bg") {
            v.panel_fill = x;
        }
        if let Some(x) = c("surface") {
            v.window_fill = x;
        }
        if let Some(x) = c("surface-raised") {
            v.faint_bg_color = x;
            v.code_bg_color = x;
            v.widgets.inactive.bg_fill = x;
            v.widgets.inactive.weak_bg_fill = x;
        }
        if let Some(x) = c("input") {
            v.extreme_bg_color = x;
        }
        if let Some(x) = c("border") {
            v.window_stroke = Stroke::new(1.0, x);
            v.widgets.noninteractive.bg_stroke = Stroke::new(1.0, x);
        }
        if let Some(x) = c("hover") {
            v.widgets.hovered.bg_fill = x;
            v.widgets.hovered.weak_bg_fill = x;
        }
        if let Some(x) = c("text") {
            v.override_text_color = Some(x);
            v.widgets.noninteractive.fg_stroke = Stroke::new(1.0, x);
            v.widgets.inactive.fg_stroke = Stroke::new(1.0, x);
        }
        if let Some(x) = c("text-muted") {
            v.weak_text_color = Some(x);
        }
        if let Some(x) = c("accent") {
            v.selection.bg_fill = x;
            v.widgets.active.bg_fill = x;
            v.widgets.active.weak_bg_fill = x;
            v.widgets.hovered.bg_stroke = Stroke::new(1.0, x);
        }
        if let Some(x) = c("accent-text") {
            v.selection.stroke = Stroke::new(1.0, x);
            v.widgets.active.fg_stroke = Stroke::new(1.0, x);
        }
        if let Some(x) = c("link") {
            v.hyperlink_color = x;
        }
        if let Some(x) = c("warning") {
            v.warn_fg_color = x;
        }
        if let Some(x) = c("error") {
            v.error_fg_color = x;
        }
        if let Some(r) = radius(t.radius("sm")) {
            for w in [
                &mut v.widgets.noninteractive,
                &mut v.widgets.inactive,
                &mut v.widgets.hovered,
                &mut v.widgets.active,
                &mut v.widgets.open,
            ] {
                w.corner_radius = r;
            }
        }
        if let Some(r) = radius(t.radius("md")) {
            v.menu_corner_radius = r;
        }
        if let Some(r) = radius(t.radius("lg")) {
            v.window_corner_radius = r;
        }
        if let Some(s) = shadow(t.shadow.get("window")) {
            v.window_shadow = s;
        }
        if let Some(s) = shadow(t.shadow.get("popup")) {
            v.popup_shadow = s;
        }
        v
    }

    /// Applies the tokens to `style` (visuals, spacing, text sizes).
    pub fn style(mode: Mode, style: &mut egui::Style) {
        let t = Self::tokens();
        style.visuals = Self::visuals(mode);
        let (sm, md, lg) = (t.space("sm"), t.space("md"), t.space("lg"));
        if let (Some(sm), Some(md)) = (sm, md) {
            style.spacing.item_spacing = vec2(md, sm);
            style.spacing.button_padding = vec2(md, sm);
        }
        if let Some(lg) = lg {
            let m = Margin::same(lg.round().clamp(0.0, 127.0) as i8);
            style.spacing.window_margin = m;
            style.spacing.indent = lg;
        }
        if let Some(md) = md {
            style.spacing.menu_margin = Margin::same(md.round().clamp(0.0, 127.0) as i8);
        }
        for (ts, key, mono) in [
            (TextStyle::Small, "size-small", false),
            (TextStyle::Body, "size-body", false),
            (TextStyle::Button, "size-button", false),
            (TextStyle::Heading, "size-heading", false),
            (TextStyle::Monospace, "size-mono", true),
        ] {
            if let Some(size) = t.font_size(key) {
                let id = if mono {
                    FontId::monospace(size)
                } else {
                    FontId::proportional(size)
                };
                style.text_styles.insert(ts, id);
            }
        }
    }

    /// Installs the theme for `mode` in `ctx` and makes `mode` the active one.
    pub fn apply(ctx: &egui::Context, mode: Mode) {
        ctx.style_mut_of(mode.egui(), |s| Self::style(mode, s));
        ctx.set_theme(mode.egui());
    }

    /// Installs both modes and lets egui follow the system preference.
    pub fn install_all(ctx: &egui::Context) {
        for mode in [Mode::Dark, Mode::Light] {
            ctx.style_mut_of(mode.egui(), |s| Self::style(mode, s));
        }
        ctx.set_theme(egui::ThemePreference::System);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn embedded_tokens_are_complete() {
        let problems = Theme::tokens().problems();
        assert!(problems.is_empty(), "{problems:#?}");
        assert!(Theme::tokens().font_size("size-body").is_some());
    }

    #[test]
    fn hex_parsing() {
        assert_eq!(parse_hex("#ff0000"), Some(Color32::from_rgb(255, 0, 0)));
        assert_eq!(
            parse_hex("00000080"),
            Some(Color32::from_rgba_unmultiplied(0, 0, 0, 128))
        );
        assert!(parse_hex("#12345").is_none());
        assert!(parse_hex("#zz0000").is_none());
    }

    #[test]
    fn problems_are_reported() {
        let t = Tokens::from_json(r##"{"color": {"dark": {"bg": "#000000", "only": "#000000"}, "light": {"bg": "nope"}}}"##)
            .unwrap();
        let p = t.problems();
        assert!(
            p.iter()
                .any(|s| s.contains("Light: missing or invalid colour bg"))
        );
        assert!(p.iter().any(|s| s.contains("only in dark")));
    }

    #[test]
    fn apply_sets_visuals_on_context() {
        let ctx = egui::Context::default();
        Theme::apply(&ctx, Mode::Light);
        let style = ctx.style_of(egui::Theme::Light);
        assert!(!style.visuals.dark_mode);
        assert_eq!(
            Some(style.visuals.panel_fill),
            Theme::tokens().color(Mode::Light, "bg")
        );
        assert_eq!(style.text_styles[&TextStyle::Body].size, 13.5);
        Theme::apply(&ctx, Mode::Dark);
        let style = ctx.style_of(egui::Theme::Dark);
        assert!(style.visuals.dark_mode);
        assert_eq!(
            Some(style.visuals.panel_fill),
            Theme::tokens().color(Mode::Dark, "bg")
        );
        Theme::install_all(&ctx);
    }
}
