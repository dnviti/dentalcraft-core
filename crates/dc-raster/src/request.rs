//! Render parameters as JSON (MCP `render`, `view.set`, the CLI and the app
//! bridge share them).

use std::collections::BTreeMap;

use dc_case::{BlobRef, CaseState, ScanRole};
use dc_scene::{Color, MeshColoring};
use serde_json::{Map, Value};

use crate::{Item, RenderOptions, View, case_items, render};

/// Default image width of a request.
pub const DEFAULT_WIDTH: u32 = 1024;
/// Default image height of a request.
pub const DEFAULT_HEIGHT: u32 = 768;
/// Default supersampling of a request.
pub const DEFAULT_SUPERSAMPLE: u32 = 2;

/// A render of a case, parsed from JSON.
///
/// Keys (all optional, others ignored): `width`, `height` (pixels) or
/// `size` (`"1024x768"`); `view` or `preset` (a [`View`] name, default
/// `occlusal`: looks at the crowns, from the side of the labelled teeth
/// when labels exist, else at the upper arch, or at the lower one when only
/// the lower arch is drawn); `visible` (scan roles: `upper`,
/// `lower`, `bite`); `wireframe` (bool); `supersample` (1..=4, default 2);
/// `background` (`#RRGGBB` or `#RRGGBBAA`); `margin` (0..0.45).
#[derive(Debug, Clone, PartialEq)]
pub struct RenderRequest {
    /// Render settings.
    pub options: RenderOptions,
    /// Roles drawn; all when `None`.
    pub visible: Option<Vec<ScanRole>>,
    /// The view was the generic `occlusal`: switch to the lower arch when
    /// only the lower arch is drawn.
    pub auto_occlusal: bool,
}

impl Default for RenderRequest {
    fn default() -> Self {
        let mut options = RenderOptions::new(DEFAULT_WIDTH, DEFAULT_HEIGHT, View::OcclusalUpper);
        options.supersample = DEFAULT_SUPERSAMPLE;
        Self {
            options,
            visible: None,
            auto_occlusal: true,
        }
    }
}

fn role(s: &str) -> Option<ScanRole> {
    match s.trim().to_ascii_lowercase().as_str() {
        "upper" => Some(ScanRole::Upper),
        "lower" => Some(ScanRole::Lower),
        "bite" => Some(ScanRole::Bite),
        _ => None,
    }
}

/// `"1024x768"` as (width, height).
pub fn parse_size(s: &str) -> Option<(u32, u32)> {
    let (w, h) = s.trim().split_once(['x', 'X', '*'])?;
    Some((w.trim().parse().ok()?, h.trim().parse().ok()?))
}

fn uint(v: &Value, key: &str) -> Result<u32, String> {
    v.as_u64()
        .and_then(|n| u32::try_from(n).ok())
        .ok_or_else(|| format!("`{key}` must be a positive integer"))
}

impl RenderRequest {
    /// Request from the keys of `layers`, later maps overriding earlier
    /// ones (e.g. the `view.set` defaults, then the call's parameters).
    pub fn from_layers(layers: &[&Map<String, Value>]) -> Result<Self, String> {
        let mut merged = Map::new();
        for l in layers {
            for (k, v) in l.iter() {
                merged.insert(k.clone(), v.clone());
            }
        }
        Self::from_map(&merged)
    }

    /// Request from a JSON object (`null` = defaults).
    pub fn from_json(params: &Value) -> Result<Self, String> {
        match params {
            Value::Null => Ok(Self::default()),
            Value::Object(m) => Self::from_map(m),
            _ => Err("render parameters must be an object".into()),
        }
    }

    fn from_map(m: &Map<String, Value>) -> Result<Self, String> {
        let mut r = Self::default();
        let o = &mut r.options;
        if let Some(s) = m.get("size").filter(|v| !v.is_null()) {
            let s = s.as_str().unwrap_or_default();
            (o.width, o.height) = parse_size(s)
                .ok_or_else(|| format!("`size` must look like 1024x768, not {s:?}"))?;
        }
        if let Some(v) = m.get("width").filter(|v| !v.is_null()) {
            o.width = uint(v, "width")?;
        }
        if let Some(v) = m.get("height").filter(|v| !v.is_null()) {
            o.height = uint(v, "height")?;
        }
        let view = m
            .get("view")
            .filter(|v| !v.is_null())
            .or_else(|| m.get("preset").filter(|v| !v.is_null()));
        if let Some(v) = view {
            let name = v.as_str().unwrap_or_default();
            o.view = View::parse(name).ok_or_else(|| {
                let names: Vec<&str> = View::ALL.iter().map(|v| v.name()).collect();
                format!("unknown view {name:?}; use one of {}", names.join(", "))
            })?;
            r.auto_occlusal = name.trim().eq_ignore_ascii_case("occlusal");
        }
        if let Some(v) = m.get("wireframe").filter(|v| !v.is_null()) {
            let on = v.as_bool().ok_or("`wireframe` must be true or false")?;
            o.wireframe = on.then_some(RenderOptions::WIRE);
        }
        if let Some(v) = m.get("supersample").filter(|v| !v.is_null()) {
            o.supersample = uint(v, "supersample")?.clamp(1, 4);
        }
        if let Some(v) = m.get("margin").filter(|v| !v.is_null()) {
            let x = v.as_f64().ok_or("`margin` must be a number")?;
            o.margin = (x as f32).clamp(0.0, 0.45);
        }
        if let Some(v) = m.get("background").filter(|v| !v.is_null()) {
            let s = v.as_str().unwrap_or_default();
            o.background = Color::from_hex(s)
                .ok_or_else(|| format!("`background` must be #RRGGBB or #RRGGBBAA, not {s:?}"))?;
        }
        if let Some(v) = m.get("visible").filter(|v| !v.is_null()) {
            let items = v.as_array().ok_or("`visible` must be an array of roles")?;
            let roles = items
                .iter()
                .map(|i| {
                    i.as_str().and_then(role).ok_or_else(|| {
                        format!("unknown role {i} in `visible` (upper, lower, bite)")
                    })
                })
                .collect::<Result<Vec<_>, _>>()?;
            r.visible = Some(roles);
        }
        Ok(r)
    }

    /// Renders the scans of `state` (see [`case_items`]) and encodes PNG.
    pub fn render_case_png(
        &self,
        state: &CaseState,
        labels: &BTreeMap<ScanRole, BlobRef>,
    ) -> Result<Vec<u8>, String> {
        let items = case_items(state, labels, self.visible.as_deref());
        let mut opts = self.options.clone();
        if self.auto_occlusal && opts.view == View::OcclusalUpper {
            let lower_only = || {
                let drawn = |r: ScanRole| {
                    state.case.scans.iter().any(|s| {
                        s.role == r && self.visible.as_ref().is_none_or(|v| v.contains(&r))
                    })
                };
                drawn(ScanRole::Lower) && !drawn(ScanRole::Upper)
            };
            // look at the crowns: from the side of the labelled teeth when
            // labels exist, else by arch as the app viewport
            if crowns_face_plus_z(&items).unwrap_or_else(lower_only) {
                opts.view = View::OcclusalLower;
            }
        }
        render(&items, &opts)
            .and_then(|img| img.to_png())
            .map_err(|e| e.to_string())
    }
}

/// `Some(true)` when the tooth-labelled vertices of `items` lie above (+Z)
/// the mean of all their vertices, `Some(false)` when below, `None` without
/// labels.
fn crowns_face_plus_z(items: &[Item<'_>]) -> Option<bool> {
    let (mut all, mut n_all, mut teeth, mut n_teeth) = (0.0f64, 0usize, 0.0f64, 0usize);
    for it in items {
        let labels: &[u16] = match &it.coloring {
            MeshColoring::Labels { labels, .. } => labels,
            MeshColoring::VertexLabels(_) => it.mesh.vertex_labels.as_deref().unwrap_or_default(),
            _ => continue,
        };
        if labels.len() != it.mesh.vertex_count() {
            continue;
        }
        for (p, &l) in it.mesh.positions.iter().zip(labels) {
            let z = f64::from(it.xform.transform_point3(*p).z);
            all += z;
            n_all += 1;
            if l != 0 {
                teeth += z;
                n_teeth += 1;
            }
        }
    }
    (n_teeth > 0 && n_teeth < n_all).then(|| teeth / n_teeth as f64 > all / n_all as f64)
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    #[test]
    fn defaults_and_overrides() {
        let r = RenderRequest::from_json(&Value::Null).unwrap();
        assert_eq!((r.options.width, r.options.height), (1024, 768));
        assert_eq!(r.options.view, View::OcclusalUpper);
        let r = RenderRequest::from_json(&json!({
            "size": "320x200", "preset": "front", "wireframe": true,
            "visible": ["upper"], "supersample": 9, "background": "#00000000",
            "unrelated": 1
        }))
        .unwrap();
        assert_eq!((r.options.width, r.options.height), (320, 200));
        assert_eq!(r.options.view, View::Frontal);
        assert!(!r.auto_occlusal);
        assert!(r.options.wireframe.is_some());
        assert_eq!(r.options.supersample, 4);
        assert_eq!(r.options.background.a, 0.0);
        assert_eq!(r.visible, Some(vec![ScanRole::Upper]));
    }

    #[test]
    fn later_layers_win() {
        let defaults = json!({"view": "iso", "width": 10, "height": 10});
        let call = json!({"width": 20});
        let r =
            RenderRequest::from_layers(&[defaults.as_object().unwrap(), call.as_object().unwrap()])
                .unwrap();
        assert_eq!((r.options.width, r.options.height), (20, 10));
        assert_eq!(r.options.view, View::Iso);
    }

    #[test]
    fn crowns_side_from_labels() {
        use dc_mesh::TriMesh;
        use glam::Vec3;
        let mut m = TriMesh::new(
            vec![Vec3::ZERO, Vec3::X, Vec3::new(0.0, 0.0, 5.0)],
            vec![[0, 1, 2]],
        );
        m.vertex_labels = Some(vec![0, 0, 11]);
        let it = Item::new(&m).with_coloring(MeshColoring::VertexLabels(crate::label_palette()));
        assert_eq!(crowns_face_plus_z(std::slice::from_ref(&it)), Some(true));
        let flipped = it
            .clone()
            .with_xform(glam::Mat4::from_scale(Vec3::new(1.0, 1.0, -1.0)));
        assert_eq!(crowns_face_plus_z(&[flipped]), Some(false));
        assert_eq!(crowns_face_plus_z(&[Item::new(&m)]), None);
    }

    #[test]
    fn bad_values_are_explained() {
        for bad in [
            json!({"view": "sideways"}),
            json!({"size": "big"}),
            json!({"width": -1}),
            json!({"visible": ["jaw"]}),
            json!({"wireframe": "yes"}),
            json!(3),
        ] {
            assert!(RenderRequest::from_json(&bad).is_err(), "{bad}");
        }
        assert_eq!(parse_size("640 x 480"), Some((640, 480)));
    }
}
