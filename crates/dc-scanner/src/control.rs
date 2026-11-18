//! Command surface of the scanner: the `scan.*` commands.
//!
//! [`ScanController`] owns the [`ScanSession`] (device and acquisition) and,
//! once the acquisition stopped, the [`ScanEditor`] (cloud, mesh, undo). Every
//! user action of the scan popup is one command, so the CLI, the MCP server
//! and the UI drive the same code: [`ScanController::execute`] takes the id,
//! a JSON parameters object and a cancellation probe, and returns a JSON
//! result. [`command_infos`] lists the commands with their i18n keys and the
//! JSON Schema of their parameters.
//!
//! `scan.import` needs the case, so the controller does not run it: the host
//! calls [`ScanController::export_mesh_for_import`] and hands the file to its
//! import command (see [`SCAN_IMPORT`] and [`import_role`]).
//!
//! | id | parameters | result |
//! |---|---|---|
//! | `scan.devices` | none | `{devices, backends}` |
//! | `scan.connect` | `backend?`, `device?`, `options?` (sim only) | `{device}` |
//! | `scan.disconnect`, `scan.start`, `scan.pause`, `scan.resume` | none | `{}` / `{state}` |
//! | `scan.stop` | none | `{points}` (the cloud becomes the editable scan) |
//! | `scan.reset` | none | `{}` (discards the scan and its edits) |
//! | `scan.status` | none | status + `editor` |
//! | `scan.settings` | `live_voxel_mm?`, `min_depth_mm?`, `max_depth_mm?`, `exposure_us?`, `roi_radius_mm?`, `roi_height_mm?`, `use_roi?`, `fill_light?`, `fast_exposure?` | current settings |
//! | `scan.cloud` | `op`: a [`CloudOp`] | the op's summary + `editor` |
//! | `scan.mesh-build` | `resolution_mm` | vertices / faces + `editor` |
//! | `scan.mesh` | `op`: a [`MeshOp`] | the op's summary + `editor` |
//! | `scan.undo`, `scan.redo` | none | `{done, editor}` |
//! | `scan.export` | `path`, `what` (`cloud` / `mesh`) | `{path, what}` |
//! | `scan.import` | `role?` | run by the host |
//! | `scan.auto` | `tilts_deg?`, `rotation_deg?`, `deg_per_s?`, `settle_s?` | `{started, steps}`; runs in the background |
//! | `turntable.devices` | `timeout_s?` | `{devices, backends}` |
//! | `turntable.connect` | `backend?`, `device?`, `options?` (sim only) | `{turntable, axes, calibrated}` |
//! | `turntable.disconnect` | none | `{}` |
//! | `turntable.move` | `axis`, `by_deg` or `to_deg` | turntable status (returns once commanded) |
//! | `turntable.speed` | `axis`, `deg_per_s` | turntable status |
//! | `turntable.stop`, `turntable.home`, `turntable.status` | none | turntable status |
//! | `turntable.calibrate` | `steps?`, `step_deg?` | `{started, steps}`; runs in the background |
//! | `turntable.calibration` | none | the stored [`crate::RigCalibration`] or `null` |
//!
//! The turntable commands and `scan.auto` are implemented in
//! [`crate::rig`]; they never keep the controller busy while the table
//! moves (see `rig::control`).

use std::path::{Path, PathBuf};
use std::sync::Arc;

use dc_mesh::io::MeshFormat;
use schemars::JsonSchema;
use serde::Deserialize;
use serde::de::DeserializeOwned;
use serde_json::{Map, Value, json};

use crate::editor::{CloudOp, MeshOp};
use crate::rig::control::RigState;
use crate::session::{ScanSession, ScanSettings, ScanState};
use crate::sim::{self, SimBackend, SimOptions};
use crate::{Result, ScanEditor, ScanError, ScannerBackend, backend, backends};

/// `scan.devices`.
pub const SCAN_DEVICES: &str = "scan.devices";
/// `scan.connect`.
pub const SCAN_CONNECT: &str = "scan.connect";
/// `scan.disconnect`.
pub const SCAN_DISCONNECT: &str = "scan.disconnect";
/// `scan.start`.
pub const SCAN_START: &str = "scan.start";
/// `scan.pause`.
pub const SCAN_PAUSE: &str = "scan.pause";
/// `scan.resume`.
pub const SCAN_RESUME: &str = "scan.resume";
/// `scan.stop`.
pub const SCAN_STOP: &str = "scan.stop";
/// `scan.reset`.
pub const SCAN_RESET: &str = "scan.reset";
/// `scan.status`.
pub const SCAN_STATUS: &str = "scan.status";
/// `scan.settings`.
pub const SCAN_SETTINGS: &str = "scan.settings";
/// `scan.cloud`.
pub const SCAN_CLOUD: &str = "scan.cloud";
/// `scan.mesh-build`.
pub const SCAN_MESH_BUILD: &str = "scan.mesh-build";
/// `scan.mesh`.
pub const SCAN_MESH: &str = "scan.mesh";
/// `scan.undo`.
pub const SCAN_UNDO: &str = "scan.undo";
/// `scan.redo`.
pub const SCAN_REDO: &str = "scan.redo";
/// `scan.export`.
pub const SCAN_EXPORT: &str = "scan.export";
/// `scan.import`: executed by the host (it needs the case), not by the
/// controller.
pub const SCAN_IMPORT: &str = "scan.import";
/// `scan.auto`: automatic turntable scan, in the background.
pub const SCAN_AUTO: &str = "scan.auto";
/// `turntable.devices`.
pub const TURNTABLE_DEVICES: &str = "turntable.devices";
/// `turntable.connect`.
pub const TURNTABLE_CONNECT: &str = "turntable.connect";
/// `turntable.disconnect`.
pub const TURNTABLE_DISCONNECT: &str = "turntable.disconnect";
/// `turntable.move`.
pub const TURNTABLE_MOVE: &str = "turntable.move";
/// `turntable.speed`.
pub const TURNTABLE_SPEED: &str = "turntable.speed";
/// `turntable.stop`.
pub const TURNTABLE_STOP: &str = "turntable.stop";
/// `turntable.home`.
pub const TURNTABLE_HOME: &str = "turntable.home";
/// `turntable.status`.
pub const TURNTABLE_STATUS: &str = "turntable.status";
/// `turntable.calibrate`: axis calibration, in the background.
pub const TURNTABLE_CALIBRATE: &str = "turntable.calibrate";
/// `turntable.calibration`.
pub const TURNTABLE_CALIBRATION: &str = "turntable.calibration";

/// A scan or turntable command as listed to agents and the UI.
#[derive(Debug, Clone, Copy)]
pub struct ScanCommandInfo {
    /// Stable id (`scan.*`, `turntable.*`).
    pub id: &'static str,
    /// i18n key of the label (`scan-cmd-<name>`, `turntable-cmd-<name>`).
    pub label_key: &'static str,
    /// i18n key of the documentation (`<label key>-doc`).
    pub doc_key: &'static str,
    /// JSON Schema of the parameters object.
    pub params_schema: fn() -> Value,
}

fn schema_of<T: JsonSchema>() -> Value {
    schemars::schema_for!(T).to_value()
}

macro_rules! info {
    ($id:expr, $name:literal, $params:ty) => {
        info!($id, "scan-cmd-", $name, $params)
    };
    ($id:expr, $prefix:literal, $name:literal, $params:ty) => {
        ScanCommandInfo {
            id: $id,
            label_key: concat!($prefix, $name),
            doc_key: concat!($prefix, $name, "-doc"),
            params_schema: schema_of::<$params>,
        }
    };
}

/// Every scan and turntable command, in listing order.
pub fn command_infos() -> Vec<ScanCommandInfo> {
    use crate::rig::control as rig;
    vec![
        info!(SCAN_DEVICES, "devices", NoParams),
        info!(SCAN_CONNECT, "connect", ConnectParams),
        info!(SCAN_DISCONNECT, "disconnect", NoParams),
        info!(SCAN_START, "start", NoParams),
        info!(SCAN_PAUSE, "pause", NoParams),
        info!(SCAN_RESUME, "resume", NoParams),
        info!(SCAN_STOP, "stop", NoParams),
        info!(SCAN_RESET, "reset", NoParams),
        info!(SCAN_STATUS, "status", NoParams),
        info!(SCAN_SETTINGS, "settings", SettingsParams),
        info!(SCAN_CLOUD, "cloud", CloudParams),
        info!(SCAN_MESH_BUILD, "mesh-build", MeshBuildParams),
        info!(SCAN_MESH, "mesh", MeshParams),
        info!(SCAN_UNDO, "undo", NoParams),
        info!(SCAN_REDO, "redo", NoParams),
        info!(SCAN_EXPORT, "export", ExportParams),
        info!(SCAN_IMPORT, "import", ImportParams),
        info!(SCAN_AUTO, "auto", rig::AutoParams),
        info!(
            TURNTABLE_DEVICES,
            "turntable-cmd-",
            "devices",
            rig::DevicesParams
        ),
        info!(
            TURNTABLE_CONNECT,
            "turntable-cmd-",
            "connect",
            rig::ConnectParams
        ),
        info!(
            TURNTABLE_DISCONNECT,
            "turntable-cmd-", "disconnect", NoParams
        ),
        info!(TURNTABLE_MOVE, "turntable-cmd-", "move", rig::MoveParams),
        info!(TURNTABLE_SPEED, "turntable-cmd-", "speed", rig::SpeedParams),
        info!(TURNTABLE_STOP, "turntable-cmd-", "stop", NoParams),
        info!(TURNTABLE_HOME, "turntable-cmd-", "home", NoParams),
        info!(TURNTABLE_STATUS, "turntable-cmd-", "status", NoParams),
        info!(
            TURNTABLE_CALIBRATE,
            "turntable-cmd-",
            "calibrate",
            rig::CalibrateParams
        ),
        info!(
            TURNTABLE_CALIBRATION,
            "turntable-cmd-", "calibration", NoParams
        ),
    ]
}

/// `true` for the ids of the scan and turntable commands (`scan.` and
/// `turntable.` prefixes): the commands of the [`ScanController`].
pub fn is_scan_command(id: &str) -> bool {
    id.starts_with("scan.") || is_turntable_command(id)
}

/// `true` for the ids of the turntable commands (`turntable.` prefix).
pub fn is_turntable_command(id: &str) -> bool {
    id.starts_with("turntable.")
}

/// Parameters of the commands without parameters.
#[derive(Debug, Default, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub(crate) struct NoParams {}

/// Behaviour of the simulated device (`scan.connect` with `backend: "sim"`),
/// for tests and demos. Unset fields keep the defaults of the simulator.
#[derive(Debug, Default, Deserialize, JsonSchema)]
#[serde(default, deny_unknown_fields)]
struct SimOptionsParams {
    /// Pace the frames in real time (default `true`); `false` delivers them
    /// as fast as they are computed.
    realtime: Option<bool>,
    /// Stop producing frames after this many (default: endless).
    frames: Option<u64>,
    /// Depth image width in pixels (default 640).
    width: Option<u32>,
    /// Depth image height in pixels (default 400).
    height: Option<u32>,
    /// Frame rate in frames per second (default 10).
    fps: Option<f32>,
    /// Standard deviation of the depth noise in millimetres (default 0.03).
    noise_mm: Option<f32>,
    /// Fraction of the pixels dropped at random (default 0.02).
    dropout: Option<f32>,
    /// Frames of one full orbit of the camera (default 240).
    period_frames: Option<u32>,
    /// Seed of the noise.
    seed: Option<u64>,
}

impl SimOptionsParams {
    fn to_options(&self) -> Result<SimOptions> {
        let mut o = SimOptions::default();
        if let Some(v) = self.realtime {
            o.realtime = v;
        }
        if let Some(v) = self.frames {
            o.max_frames = Some(v);
        }
        if let Some(v) = self.width {
            o.width = v;
        }
        if let Some(v) = self.height {
            o.height = v;
        }
        if let Some(v) = self.fps {
            o.fps = v;
        }
        if let Some(v) = self.noise_mm {
            o.noise_mm = v;
        }
        if let Some(v) = self.dropout {
            o.dropout = v;
        }
        if let Some(v) = self.period_frames {
            o.period_frames = v;
        }
        if let Some(v) = self.seed {
            o.seed = v;
        }
        let ok = o.width >= 16
            && o.height >= 16
            && o.width <= 4096
            && o.height <= 4096
            && o.fps.is_finite()
            && o.fps > 0.0
            && o.noise_mm.is_finite()
            && o.noise_mm >= 0.0
            && (0.0..1.0).contains(&o.dropout)
            && o.period_frames >= 1;
        if ok {
            Ok(o)
        } else {
            Err(ScanError::InvalidParams(
                "sim options out of range (size 16..4096, fps > 0, noise >= 0, dropout in [0, 1), period >= 1)"
                    .into(),
            ))
        }
    }
}

/// Parameters of `scan.connect`.
#[derive(Debug, Default, Deserialize, JsonSchema)]
#[serde(default, deny_unknown_fields)]
struct ConnectParams {
    /// Backend id (`revopoint-usb`, `revopoint-sdk`, `sim`). Default: the backend of the
    /// first hardware device found, else `sim`.
    backend: Option<String>,
    /// Device id as listed by `scan.devices`. Default: the first device of
    /// the backend (the simulated upper typodont for `sim`).
    device: Option<String>,
    /// Options of the simulated device; only with the `sim` backend.
    options: Option<SimOptionsParams>,
}

/// Parameters of `scan.settings`.
#[derive(Debug, Default, Deserialize, JsonSchema)]
#[serde(default, deny_unknown_fields)]
struct SettingsParams {
    /// Voxel size of the live fusion volume in millimetres, 0.05 to 5.
    live_voxel_mm: Option<f32>,
    /// Depths closer than this are ignored (mm).
    min_depth_mm: Option<f32>,
    /// Depths farther than this are ignored (mm).
    max_depth_mm: Option<f32>,
    /// Exposure time in microseconds; 0 returns to the automatic exposure.
    exposure_us: Option<u32>,
    /// Radius of the turntable region of interest around the axis (mm), 5
    /// to 200.
    roi_radius_mm: Option<f32>,
    /// Height of the turntable region of interest above the platter (mm),
    /// 5 to 300.
    roi_height_mm: Option<f32>,
    /// Crop the frames to the turntable region of interest when a rig
    /// calibration is in use.
    use_roi: Option<bool>,
    /// Keep the scanner's fill light steadily on while scanning (default
    /// true; false turns it off).
    fill_light: Option<bool>,
    /// Run the scanner at its fast frame rate (default true; on the MINI 2
    /// about 6.8 instead of 2.7 frames per second).
    fast_exposure: Option<bool>,
}

/// Parameters of `scan.cloud`.
#[derive(Debug, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
struct CloudParams {
    /// The point-cloud edit (`{"op": "remove-statistical", "k": 16, ...}`).
    op: CloudOp,
}

/// Parameters of `scan.mesh-build`.
#[derive(Debug, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
struct MeshBuildParams {
    /// Mesh resolution (grid cell edge) in millimetres, 0.05 to 5.
    resolution_mm: f32,
}

/// Parameters of `scan.mesh`.
#[derive(Debug, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
struct MeshParams {
    /// The mesh repair (`{"op": "fill-holes", "max_edges": 60}`).
    op: MeshOp,
}

/// What `scan.export` writes.
#[derive(Debug, Clone, Copy, Deserialize, JsonSchema)]
#[serde(rename_all = "kebab-case")]
enum ExportKind {
    /// The point cloud, as PLY.
    Cloud,
    /// The mesh, as STL, PLY or OBJ by the extension of the path (PLY for
    /// any other extension).
    Mesh,
}

/// Parameters of `scan.export`.
#[derive(Debug, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
struct ExportParams {
    /// Output file.
    path: String,
    /// `cloud` or `mesh`.
    what: ExportKind,
}

/// Arch or bite of an imported scan.
#[derive(Debug, Clone, Copy, Deserialize, JsonSchema)]
#[serde(rename_all = "lowercase")]
enum ImportRole {
    Upper,
    Lower,
    Bite,
}

impl ImportRole {
    fn as_str(self) -> &'static str {
        match self {
            Self::Upper => "upper",
            Self::Lower => "lower",
            Self::Bite => "bite",
        }
    }
}

/// Parameters of `scan.import`.
#[derive(Debug, Default, Deserialize, JsonSchema)]
#[serde(default, deny_unknown_fields)]
struct ImportParams {
    /// Role of the scan in the case: `upper`, `lower` or `bite`. Default: the
    /// import decides from the geometry.
    role: Option<ImportRole>,
}

/// The role (`upper`, `lower`, `bite`) of the `scan.import` parameters.
///
/// # Errors
/// [`ScanError::InvalidParams`] for unknown fields or an unknown role.
pub fn import_role(params: &Value) -> Result<Option<&'static str>> {
    let p: ImportParams = parse(params)?;
    Ok(p.role.map(ImportRole::as_str))
}

/// Parses command parameters: `null` is an empty object.
pub(crate) fn parse<T: DeserializeOwned>(params: &Value) -> Result<T> {
    let value = if params.is_null() {
        Value::Object(Map::new())
    } else {
        params.clone()
    };
    serde_json::from_value(value).map_err(|e| ScanError::InvalidParams(e.to_string()))
}

/// The scan and turntable commands over one device session, one editable
/// scan and one turntable.
///
/// Not thread safe on its own: the host shares it as
/// `Arc<Mutex<ScanController>>`. Commands are synchronous; the slow ones
/// (`scan.mesh-build`, `scan.stop`) hold `&mut self` until done, so a UI
/// thread that polls the controller should use `try_lock`. The turntable
/// motions and the rig jobs run on their own threads.
#[derive(Debug)]
pub struct ScanController {
    pub(crate) session: ScanSession,
    editor: Option<ScanEditor>,
    pub(crate) rig: RigState,
}

impl Default for ScanController {
    fn default() -> Self {
        Self::new()
    }
}

impl ScanController {
    /// Controller without a device, a scan or a turntable.
    pub fn new() -> Self {
        Self {
            session: ScanSession::new(ScanSettings::default()),
            editor: None,
            rig: RigState::default(),
        }
    }

    /// The acquisition session.
    pub fn session(&self) -> &ScanSession {
        &self.session
    }

    /// The acquisition session, mutable.
    pub fn session_mut(&mut self) -> &mut ScanSession {
        &mut self.session
    }

    /// The editable scan, from `scan.stop` on.
    pub fn editor(&self) -> Option<&ScanEditor> {
        self.editor.as_ref()
    }

    /// The editable scan, mutable.
    pub fn editor_mut(&mut self) -> Option<&mut ScanEditor> {
        self.editor.as_mut()
    }

    /// Runs the scan command `id`. `cancel` is polled by the long kernels.
    ///
    /// # Errors
    /// The [`ScanError`] of the command; [`ScanError::InvalidParams`] for
    /// unknown ids (and for [`SCAN_IMPORT`], which the host runs).
    pub fn execute(
        &mut self,
        id: &str,
        params: &Value,
        cancel: &dyn Fn() -> bool,
    ) -> Result<Value> {
        self.reap_rig(false);
        if is_turntable_command(id) || id == SCAN_AUTO {
            return self.execute_rig(id, params);
        }
        match id {
            SCAN_DEVICES => {
                let NoParams {} = parse(params)?;
                Ok(devices())
            }
            SCAN_CONNECT => {
                let p: ConnectParams = parse(params)?;
                let info = self.connect(&p)?;
                self.apply_rig();
                Ok(json!({ "device": info }))
            }
            SCAN_DISCONNECT => {
                let NoParams {} = parse(params)?;
                self.reap_rig(true);
                self.session.disconnect();
                self.apply_rig();
                Ok(json!({}))
            }
            SCAN_START => {
                let NoParams {} = parse(params)?;
                self.session.start()?;
                Ok(self.state_json())
            }
            SCAN_PAUSE => {
                let NoParams {} = parse(params)?;
                self.session.pause()?;
                Ok(self.state_json())
            }
            SCAN_RESUME => {
                let NoParams {} = parse(params)?;
                self.session.resume()?;
                Ok(self.state_json())
            }
            SCAN_STOP => {
                let NoParams {} = parse(params)?;
                // ends an automatic scan (the table stops) and keeps its cloud
                self.reap_rig(true);
                let live = matches!(
                    self.session.status().state,
                    ScanState::Scanning | ScanState::Paused
                );
                let cloud = self.session.stop()?;
                // stopping a stopped scan again must not drop the edits
                if live || self.editor.is_none() {
                    self.editor = Some(ScanEditor::new(cloud));
                }
                let points = self.editor.as_ref().map_or(0, |e| e.cloud().len());
                Ok(json!({ "points": points }))
            }
            SCAN_RESET => {
                let NoParams {} = parse(params)?;
                self.reap_rig(true);
                self.session.reset();
                self.editor = None;
                Ok(json!({}))
            }
            SCAN_STATUS => {
                let NoParams {} = parse(params)?;
                let mut out = serde_json::to_value(self.session.status())
                    .map_err(|e| ScanError::Device(e.to_string()))?;
                out["editor"] = self.editor_json();
                out["auto"] = serde_json::to_value(self.auto_status())
                    .map_err(|e| ScanError::Device(e.to_string()))?;
                Ok(out)
            }
            SCAN_SETTINGS => {
                let p: SettingsParams = parse(params)?;
                self.settings(&p)
            }
            SCAN_CLOUD => {
                let p: CloudParams = parse(params)?;
                let mut out = self.editor_required()?.apply_cloud(&p.op)?;
                self.add_editor(&mut out);
                Ok(out)
            }
            SCAN_MESH_BUILD => {
                let p: MeshBuildParams = parse(params)?;
                let mut out = self
                    .editor_required()?
                    .build_mesh(p.resolution_mm, cancel)?;
                self.add_editor(&mut out);
                Ok(out)
            }
            SCAN_MESH => {
                let p: MeshParams = parse(params)?;
                let mut out = self.editor_required()?.apply_mesh(&p.op)?;
                self.add_editor(&mut out);
                Ok(out)
            }
            SCAN_UNDO | SCAN_REDO => {
                let NoParams {} = parse(params)?;
                let editor = self.editor_required()?;
                let done = if id == SCAN_UNDO {
                    editor.undo()
                } else {
                    editor.redo()
                };
                let mut out = json!({ "done": done });
                self.add_editor(&mut out);
                Ok(out)
            }
            SCAN_EXPORT => {
                let p: ExportParams = parse(params)?;
                self.export(&p)
            }
            SCAN_IMPORT => Err(ScanError::InvalidParams(format!(
                "{SCAN_IMPORT} is run by the host (it needs the case)"
            ))),
            _ => Err(ScanError::InvalidParams(format!(
                "unknown scan command {id}"
            ))),
        }
    }

    /// Writes the current mesh (binary PLY) into `dir` as `scan-<role>.ply`
    /// (`scan.ply` without a role) and returns the path.
    ///
    /// # Errors
    /// [`ScanError::NoMesh`] (code `no-mesh`) before a mesh was built;
    /// [`ScanError::Mesh`] when the file cannot be written.
    pub fn export_mesh_for_import(&self, dir: &Path, role: Option<&str>) -> Result<PathBuf> {
        let mesh = self
            .editor
            .as_ref()
            .and_then(ScanEditor::mesh)
            .ok_or(ScanError::NoMesh)?;
        let name = role.map_or_else(|| "scan.ply".to_owned(), |r| format!("scan-{r}.ply"));
        let path = dir.join(name);
        dc_mesh::io::write(&path, mesh, MeshFormat::Ply)?;
        Ok(path)
    }

    fn state_json(&self) -> Value {
        json!({ "state": self.session.status().state })
    }

    fn editor_required(&mut self) -> Result<&mut ScanEditor> {
        self.editor
            .as_mut()
            .ok_or(ScanError::Empty("no scan: stop the acquisition first"))
    }

    /// Summary of the editable scan, `null` without one.
    fn editor_json(&self) -> Value {
        let Some(e) = &self.editor else {
            return Value::Null;
        };
        let mut o = Map::new();
        o.insert("points".into(), json!(e.cloud().len()));
        if let Some(m) = e.mesh() {
            o.insert("mesh_vertices".into(), json!(m.vertex_count()));
            o.insert("mesh_faces".into(), json!(m.face_count()));
        }
        o.insert("can_undo".into(), json!(e.can_undo()));
        o.insert("can_redo".into(), json!(e.can_redo()));
        Value::Object(o)
    }

    fn add_editor(&self, out: &mut Value) {
        if let Value::Object(o) = out {
            o.insert("editor".into(), self.editor_json());
        }
    }

    fn connect(&mut self, p: &ConnectParams) -> Result<crate::DeviceInfo> {
        let sim_options = p.options.as_ref().map(SimOptionsParams::to_options);
        let sim_options = sim_options.transpose()?;
        let backend_id = match (&p.backend, &sim_options) {
            (Some(b), _) => Some(b.as_str()),
            (None, Some(_)) => Some(sim::BACKEND_ID),
            (None, None) => None,
        };
        if sim_options.is_some() && backend_id != Some(sim::BACKEND_ID) {
            return Err(ScanError::InvalidParams(
                "options only apply to the sim backend".into(),
            ));
        }
        // the simulated scanner shares the rig of the simulated turntable
        let rig = Arc::clone(self.rig.sim());
        let sim_backend = |o: Option<SimOptions>| {
            SimBackend::new(o.unwrap_or_default()).with_rig(Arc::clone(&rig))
        };
        let open = |id: &str| -> Result<Box<dyn ScannerBackend>> {
            if id == sim::BACKEND_ID {
                Ok(Box::new(sim_backend(sim_options.clone())))
            } else {
                backend(id)
            }
        };
        match (backend_id, p.device.as_deref()) {
            (Some(b), Some(d)) => self.session.connect_with(open(b)?.as_ref(), d),
            (Some(b), None) => {
                let b = open(b)?;
                let first = first_device(b.as_ref())?;
                self.session.connect_with(b.as_ref(), &first)
            }
            (None, Some(d)) => {
                let b = backends()
                    .into_iter()
                    .find(|b| b.list().is_ok_and(|l| l.iter().any(|i| i.id == d)))
                    .ok_or_else(|| ScanError::DeviceNotFound(d.to_owned()))?;
                self.session.connect_with(open(b.id())?.as_ref(), d)
            }
            (None, None) => {
                for b in backends().into_iter().filter(|b| b.id() != sim::BACKEND_ID) {
                    if b.availability().is_err() {
                        continue;
                    }
                    if let Ok(list) = b.list()
                        && let Some(d) = list.first()
                    {
                        return self.session.connect_with(b.as_ref(), &d.id);
                    }
                }
                self.session.connect_with(&sim_backend(None), sim::UPPER_ID)
            }
        }
    }

    fn settings(&mut self, p: &SettingsParams) -> Result<Value> {
        let mut s: ScanSettings = self.session.settings().clone();
        let given = p.live_voxel_mm.is_some()
            || p.min_depth_mm.is_some()
            || p.max_depth_mm.is_some()
            || p.exposure_us.is_some()
            || p.roi_radius_mm.is_some()
            || p.roi_height_mm.is_some()
            || p.use_roi.is_some()
            || p.fill_light.is_some()
            || p.fast_exposure.is_some();
        if given {
            if let Some(v) = p.live_voxel_mm {
                s.live_voxel_mm = v;
            }
            if let Some(v) = p.min_depth_mm {
                s.min_depth_mm = v;
            }
            if let Some(v) = p.max_depth_mm {
                s.max_depth_mm = v;
            }
            if let Some(us) = p.exposure_us {
                s.exposure_us = (us != 0).then_some(us);
            }
            if let Some(v) = p.roi_radius_mm {
                s.roi_radius_mm = v;
            }
            if let Some(v) = p.roi_height_mm {
                s.roi_height_mm = v;
            }
            if let Some(v) = p.use_roi {
                s.use_roi = v;
            }
            if let Some(v) = p.fill_light {
                s.fill_light = v;
            }
            if let Some(v) = p.fast_exposure {
                s.fast_exposure = v;
            }
            self.session.set_settings(s)?;
        }
        let s = self.session.settings();
        Ok(json!({
            "live_voxel_mm": s.live_voxel_mm,
            "min_depth_mm": s.min_depth_mm,
            "max_depth_mm": s.max_depth_mm,
            "exposure_us": s.exposure_us,
            "roi_radius_mm": s.roi_radius_mm,
            "roi_height_mm": s.roi_height_mm,
            "use_roi": s.use_roi,
            "fill_light": s.fill_light,
            "fast_exposure": s.fast_exposure,
        }))
    }

    fn export(&self, p: &ExportParams) -> Result<Value> {
        let editor = self
            .editor
            .as_ref()
            .ok_or(ScanError::Empty("no scan: stop the acquisition first"))?;
        let path = Path::new(&p.path);
        match p.what {
            ExportKind::Cloud => dc_mesh::io::write_ply(editor.cloud(), path)?,
            ExportKind::Mesh => {
                let mesh = editor.mesh().ok_or(ScanError::NoMesh)?;
                let format = MeshFormat::from_path(path).unwrap_or(MeshFormat::Ply);
                dc_mesh::io::write(path, mesh, format)?;
            }
        }
        let what = match p.what {
            ExportKind::Cloud => "cloud",
            ExportKind::Mesh => "mesh",
        };
        Ok(json!({ "path": p.path, "what": what }))
    }
}

fn first_device(b: &dyn ScannerBackend) -> Result<String> {
    b.availability()?;
    b.list()?
        .into_iter()
        .next()
        .map(|d| d.id)
        .ok_or_else(|| ScanError::DeviceNotFound(format!("{} (no device)", b.id())))
}

/// `scan.devices`: the devices of every available backend and the state of
/// each backend.
fn devices() -> Value {
    let mut devices = Vec::new();
    let mut states = Vec::new();
    for b in backends() {
        let listed = b.availability().and_then(|()| b.list());
        match listed {
            Ok(list) => {
                devices.extend(list);
                states.push(json!({ "id": b.id(), "available": true }));
            }
            Err(e) => states.push(json!({
                "id": b.id(),
                "available": false,
                "error_code": e.code(),
            })),
        }
    }
    json!({ "devices": devices, "backends": states })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn run(c: &mut ScanController, id: &str, params: Value) -> Result<Value> {
        c.execute(id, &params, &|| false)
    }

    #[test]
    fn infos_are_unique_with_object_schemas_and_matching_keys() {
        let infos = command_infos();
        let mut ids: Vec<&str> = infos.iter().map(|i| i.id).collect();
        ids.sort_unstable();
        ids.dedup();
        assert_eq!(ids.len(), infos.len());
        for i in &infos {
            assert!(is_scan_command(i.id));
            let (prefix, name) = match i.id.strip_prefix("scan.") {
                Some(name) => ("scan-cmd-", name),
                None => ("turntable-cmd-", i.id.strip_prefix("turntable.").unwrap()),
            };
            assert_eq!(i.label_key, format!("{prefix}{name}"));
            assert_eq!(i.doc_key, format!("{prefix}{name}-doc"));
            assert_eq!((i.params_schema)()["type"], "object", "{}", i.id);
        }
        assert_eq!(infos.len(), 28);
        assert!(is_scan_command(TURNTABLE_MOVE) && is_turntable_command(TURNTABLE_MOVE));
        assert!(!is_turntable_command(SCAN_AUTO));
        assert!(!is_scan_command("case.import"));
    }

    #[test]
    fn devices_lists_the_sim_and_defaults_work() {
        let mut c = ScanController::new();
        let out = run(&mut c, SCAN_DEVICES, Value::Null).unwrap();
        let ids: Vec<&str> = out["devices"]
            .as_array()
            .unwrap()
            .iter()
            .filter_map(|d| d["id"].as_str())
            .collect();
        assert!(ids.contains(&sim::UPPER_ID) && ids.contains(&sim::LOWER_ID));
        assert!(
            out["backends"]
                .as_array()
                .unwrap()
                .iter()
                .any(|b| b["id"] == "sim" && b["available"] == true)
        );
    }

    #[test]
    fn bad_parameters_and_states_are_refused() {
        let mut c = ScanController::new();
        let code = |r: Result<Value>| r.unwrap_err().code();
        assert_eq!(code(run(&mut c, SCAN_START, json!({}))), "not-connected");
        assert_eq!(
            code(run(&mut c, SCAN_START, json!({"x": 1}))),
            "invalid-params"
        );
        assert_eq!(code(run(&mut c, SCAN_CLOUD, json!({}))), "invalid-params");
        assert_eq!(
            code(run(
                &mut c,
                SCAN_CLOUD,
                json!({"op": {"op": "downsample", "voxel_mm": 1.0}})
            )),
            "empty"
        );
        assert_eq!(code(run(&mut c, SCAN_IMPORT, json!({}))), "invalid-params");
        assert_eq!(code(run(&mut c, "scan.nope", json!({}))), "invalid-params");
        assert_eq!(
            code(run(&mut c, SCAN_CONNECT, json!({"backend": "nope"}))),
            "unknown-backend"
        );
        assert_eq!(
            code(run(
                &mut c,
                SCAN_CONNECT,
                json!({"backend": "sim", "device": "nope"})
            )),
            "device-not-found"
        );
        assert_eq!(
            code(run(
                &mut c,
                SCAN_CONNECT,
                json!({"backend": "other", "options": {}})
            )),
            "invalid-params"
        );
        assert_eq!(
            code(
                c.export_mesh_for_import(Path::new("."), None)
                    .map(|_| Value::Null)
            ),
            "no-mesh"
        );
        assert_eq!(import_role(&json!({"role": "bite"})).unwrap(), Some("bite"));
        assert_eq!(import_role(&Value::Null).unwrap(), None);
        assert!(import_role(&json!({"role": "x"})).is_err());
    }

    #[test]
    fn settings_apply_and_reject() {
        let mut c = ScanController::new();
        let out = run(
            &mut c,
            SCAN_SETTINGS,
            json!({"live_voxel_mm": 0.5, "exposure_us": 900}),
        )
        .unwrap();
        assert_eq!(out["live_voxel_mm"], 0.5);
        assert_eq!(out["exposure_us"], 900);
        let out = run(&mut c, SCAN_SETTINGS, json!({"exposure_us": 0})).unwrap();
        assert_eq!(out["exposure_us"], Value::Null);
        assert_eq!(
            run(&mut c, SCAN_SETTINGS, json!({"live_voxel_mm": 99.0}))
                .unwrap_err()
                .code(),
            "invalid-params"
        );
        assert_eq!(c.session().settings().live_voxel_mm, 0.5);
    }
}
