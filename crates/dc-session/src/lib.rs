//! Headless engine session (port plan B1.1).
//!
//! A [`Session`] holds the open project (path, manifest, thumbnail), the
//! current [`CaseState`] inside an [`UndoStack`], the [`StepHistory`], the
//! registered modules and the registry of every agent-drivable command: the
//! built-in ones of the session ([`builtins`]: `project.*`, `case.import`,
//! `case.set-role`, `case.remove-scan`, `edit.undo` / `edit.redo`, `steps.*`,
//! `view.set`, and the `scan.*`, `turntable.*` and `print.*` commands) plus
//! the [`CommandSpec`]s of the modules. The CLI and the MCP server drive it
//! through [`Session::execute`], [`Session::batch`], [`Session::describe`]
//! and [`Session::inspect`].
//!
//! Commands run synchronously on the caller's thread with a [`JobCtx`]
//! (detached by default, see [`Session::execute_with`] to report progress
//! or cancel from another thread). Core does not depend on any module:
//! callers inject them with [`Session::register_module`].
//!
//! ```ignore
//! let mut s = dc_session::Session::new();
//! s.register_module(Box::new(dca_module::AlignersModule::new()))?;
//! s.execute("case.import", json!({"path": "upper.stl"}))?;
//! s.execute("project.save", json!({"path": "case.dcprj"}))?;
//! println!("{}", s.inspect(Some("scans"))?);
//! ```

pub mod builtins;
mod inspect;

use std::path::{Path, PathBuf};
use std::sync::{Arc, LazyLock, Mutex, PoisonError};
use std::time::{SystemTime, UNIX_EPOCH};

use dc_case::{Case, CaseState};
use dc_command::{CommandError, JobCtx, StepError, StepGraph, StepHistory, UndoStack};
use dc_module_api::{CommandSpec, Module, ModuleCodecs, ModuleError, ModuleRegistry};
use dc_project::{AppInfo, DcprjError, LoadOutcome, Manifest, Project};
use dc_scanner::ScanController;
use serde::Serialize;
use serde_json::{Map, Value, json};

use builtins::{NewParams, NoParams, OpenParams, RewindParams, SaveParams, parse_params};

dc_i18n::bundle!(pub fn loader);

/// Source name of the built-in commands in [`CommandInfo::source`].
pub const SESSION_SOURCE: &str = "session";

/// Errors of the session.
#[derive(Debug, thiserror::Error)]
pub enum SessionError {
    /// No command with this id is registered (or its module is disabled).
    #[error("unknown command {0}")]
    UnknownCommand(String),
    /// The command refused its parameters or failed.
    #[error("{id}: {source}")]
    Command {
        /// Command id.
        id: String,
        /// What went wrong.
        #[source]
        source: CommandError,
    },
    /// Reading or writing the project file failed.
    #[error(transparent)]
    Project(#[from] DcprjError),
    /// The open file was loaded read-only and must not be overwritten.
    #[error("{} was opened read-only: save it to another path", .0.display())]
    ReadOnly(PathBuf),
    /// Saving needs a path: the project was never saved.
    #[error("the project has no file yet: give a path")]
    NoPath,
    /// `steps.rewind` needs the step pipeline ([`Session::set_step_graph`]).
    #[error("no step pipeline is known")]
    NoStepGraph,
    /// Error from the step history.
    #[error(transparent)]
    Step(#[from] StepError),
    /// Error from the module registry.
    #[error(transparent)]
    Module(#[from] ModuleError),
    /// A module declares a command id that is already registered.
    #[error("command {0} is already registered")]
    DuplicateCommand(String),
    /// [`Session::inspect`] path that does not exist.
    #[error("nothing at {0}")]
    PathNotFound(String),
    /// A command of a [`Session::batch`] failed; the earlier ones stay
    /// applied.
    #[error("command {index} ({id}) failed: {source}")]
    Batch {
        /// Position in the batch (0-based).
        index: usize,
        /// Command id.
        id: String,
        /// The error.
        #[source]
        source: Box<SessionError>,
    },
    /// JSON serialization failed.
    #[error(transparent)]
    Json(#[from] serde_json::Error),
}

/// Result of the session operations.
pub type Result<T, E = SessionError> = std::result::Result<T, E>;

/// A command as listed by [`Session::list_commands`].
#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub struct CommandInfo {
    /// Stable id.
    pub id: String,
    /// [`SESSION_SOURCE`] or the id of the module that provides it.
    pub source: String,
    /// Label (English when translated, the i18n key otherwise).
    pub label: String,
}

/// Full description of a command, from [`Session::describe`].
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct CommandDescription {
    /// Stable id.
    pub id: String,
    /// [`SESSION_SOURCE`] or the id of the module that provides it.
    pub source: String,
    /// i18n key of the label.
    pub label_key: String,
    /// Label (English when translated, the key otherwise).
    pub label: String,
    /// i18n key of the documentation.
    pub doc_key: String,
    /// Documentation (English when translated, the key otherwise).
    pub doc: String,
    /// JSON Schema of the parameters object.
    pub params_schema: Value,
}

/// Headless session: one open project and its command registry.
pub struct Session {
    modules: ModuleRegistry,
    /// Every command with its source (`session` or a module id).
    specs: Vec<(&'static str, CommandSpec)>,
    undo: UndoStack,
    saved: Arc<CaseState>,
    history: StepHistory,
    graph: Option<StepGraph>,
    manifest: Option<Manifest>,
    thumbnail: Option<Vec<u8>>,
    path: Option<PathBuf>,
    read_only: bool,
    load_issues: Vec<String>,
    view: Map<String, Value>,
    app: AppInfo,
    scan: LazyLock<Arc<Mutex<ScanController>>>,
}

impl Default for Session {
    fn default() -> Self {
        Self::new()
    }
}

impl Session {
    /// Session on a new empty case, with the built-in commands and no module.
    pub fn new() -> Self {
        loader();
        let state = Arc::new(CaseState::new(Case::new("")));
        Self {
            modules: ModuleRegistry::new(),
            specs: builtins::specs()
                .into_iter()
                .chain(builtins::scan_specs())
                .chain(builtins::print_specs())
                .map(|s| (SESSION_SOURCE, s))
                .collect(),
            undo: UndoStack::new(state.clone(), UndoStack::DEFAULT_BUDGET),
            saved: state,
            history: StepHistory::new(),
            graph: None,
            manifest: None,
            thumbnail: None,
            path: None,
            read_only: false,
            load_issues: Vec::new(),
            view: Map::new(),
            app: AppInfo {
                version: env!("CARGO_PKG_VERSION").to_owned(),
                commit: option_env!("DC_GIT_COMMIT").unwrap_or("dev").to_owned(),
            },
            scan: LazyLock::new(|| Arc::new(Mutex::new(ScanController::new()))),
        }
    }

    // ------------------------------------------------------------ setup

    /// Registers a module and its [`Module::command_specs`]. Fails when the
    /// module id or one of its command ids is already taken.
    pub fn register_module(&mut self, module: Box<dyn Module>) -> Result<()> {
        let id = module.id();
        if self.modules.get(id).is_some() {
            return Err(ModuleError::Duplicate(id.to_owned()).into());
        }
        let specs = module.command_specs();
        for (i, s) in specs.iter().enumerate() {
            if self.specs.iter().any(|(_, o)| o.id == s.id)
                || specs[..i].iter().any(|o| o.id == s.id)
            {
                return Err(SessionError::DuplicateCommand(s.id.to_owned()));
            }
        }
        self.modules.register(module)?;
        self.specs.extend(specs.into_iter().map(|s| (id, s)));
        Ok(())
    }

    /// The registered modules.
    pub fn modules(&self) -> &ModuleRegistry {
        &self.modules
    }

    /// Enables or disables a module (its commands are hidden when disabled).
    pub fn set_module_enabled(&mut self, id: &str, enabled: bool) -> Result<()> {
        Ok(self.modules.set_enabled(id, enabled)?)
    }

    /// Sets the step pipeline used by `steps.list` (status) and
    /// `steps.rewind`.
    pub fn set_step_graph(&mut self, graph: StepGraph) {
        self.graph = Some(graph);
    }

    /// Sets the writer recorded in the manifest of new projects.
    pub fn set_app_info(&mut self, app: AppInfo) {
        self.app = app;
    }

    // ------------------------------------------------------------ accessors

    /// The current case state.
    pub fn state(&self) -> &Arc<CaseState> {
        self.undo.current()
    }

    /// The undo stack.
    pub fn undo_stack(&self) -> &UndoStack {
        &self.undo
    }

    /// The step history.
    pub fn history(&self) -> &StepHistory {
        &self.history
    }

    /// The project file, if the project was opened or saved.
    pub fn path(&self) -> Option<&Path> {
        self.path.as_deref()
    }

    /// The manifest of the open project (`None` before the first save of a
    /// new project).
    pub fn manifest(&self) -> Option<&Manifest> {
        self.manifest.as_ref()
    }

    /// `true` when the open file must not be overwritten.
    pub fn is_read_only(&self) -> bool {
        self.read_only
    }

    /// `true` when the case changed since it was opened or saved.
    pub fn is_dirty(&self) -> bool {
        !Arc::ptr_eq(self.undo.current(), &self.saved)
    }

    /// Problems found when the project was loaded.
    pub fn load_issues(&self) -> &[String] {
        &self.load_issues
    }

    /// View parameters stored by `view.set`, for the renderer.
    pub fn view(&self) -> &Map<String, Value> {
        &self.view
    }

    // ------------------------------------------------------------ registry

    fn find(&self, id: &str) -> Option<(&'static str, CommandSpec)> {
        self.specs
            .iter()
            .find(|(src, s)| s.id == id && self.source_enabled(src))
            .copied()
    }

    fn source_enabled(&self, src: &str) -> bool {
        src == SESSION_SOURCE || self.modules.is_enabled(src)
    }

    /// Every available command: the built-ins first, then the commands of
    /// the enabled modules in registration order.
    pub fn list_commands(&self) -> Vec<CommandInfo> {
        self.specs
            .iter()
            .filter(|(src, _)| self.source_enabled(src))
            .map(|(src, s)| CommandInfo {
                id: s.id.to_owned(),
                source: (*src).to_owned(),
                label: dc_i18n::tr(s.label_key),
            })
            .collect()
    }

    /// Schema and documentation of command `id`.
    pub fn describe(&self, id: &str) -> Result<CommandDescription> {
        let (src, s) = self
            .find(id)
            .ok_or_else(|| SessionError::UnknownCommand(id.to_owned()))?;
        Ok(CommandDescription {
            id: s.id.to_owned(),
            source: src.to_owned(),
            label_key: s.label_key.to_owned(),
            label: dc_i18n::tr(s.label_key),
            doc_key: s.doc_key.to_owned(),
            doc: dc_i18n::tr(s.doc_key),
            params_schema: s.schema(),
        })
    }

    // ------------------------------------------------------------ execution

    /// Runs command `id` with `params` (a JSON object, or `null` for none)
    /// and returns its JSON output. State commands push the new state on
    /// the undo stack.
    pub fn execute(&mut self, id: &str, params: Value) -> Result<Value> {
        self.execute_with(id, params, &JobCtx::detached())
    }

    /// [`Session::execute`] with a job context, to report progress or cancel
    /// from another thread.
    pub fn execute_with(&mut self, id: &str, params: Value, ctx: &JobCtx) -> Result<Value> {
        let (_, spec) = self
            .find(id)
            .ok_or_else(|| SessionError::UnknownCommand(id.to_owned()))?;
        if builtins::is_session_command(id) {
            return self.run_session_command(id, &params, ctx);
        }
        let (next, out) = spec
            .execute(self.undo.current(), &params, ctx)
            .map_err(|source| SessionError::Command {
                id: id.to_owned(),
                source,
            })?;
        self.undo.push(Arc::new(next), spec.label_key);
        Ok(out)
    }

    /// Runs the commands in order and returns their outputs. Stops at the
    /// first failure ([`SessionError::Batch`]); the commands before it stay
    /// applied.
    pub fn batch(&mut self, commands: Vec<(String, Value)>) -> Result<Vec<Value>> {
        let mut out = Vec::with_capacity(commands.len());
        for (index, (id, params)) in commands.into_iter().enumerate() {
            match self.execute(&id, params) {
                Ok(v) => out.push(v),
                Err(e) => {
                    return Err(SessionError::Batch {
                        index,
                        id,
                        source: Box::new(e),
                    });
                }
            }
        }
        Ok(out)
    }

    /// Goes back one change; `false` when there is nothing to undo.
    pub fn undo(&mut self) -> bool {
        self.undo.undo().is_some()
    }

    /// Goes forward one undone change; `false` when there is nothing to redo.
    pub fn redo(&mut self) -> bool {
        self.undo.redo().is_some()
    }

    fn run_session_command(&mut self, id: &str, params: &Value, ctx: &JobCtx) -> Result<Value> {
        let invalid = |source| SessionError::Command {
            id: id.to_owned(),
            source,
        };
        if dc_scanner::is_scan_command(id) {
            return self.run_scan_command(id, params, ctx);
        }
        if dc_print::commands::is_print_command(id) {
            return dc_print::commands::execute(id, params, &|| ctx.is_cancelled(), &mut |f| {
                ctx.set_fraction(f)
            })
            .map_err(|e| SessionError::Command {
                id: id.to_owned(),
                source: builtins::print_error(e),
            });
        }
        match id {
            builtins::PROJECT_NEW => {
                let p: NewParams = parse_params(params).map_err(invalid)?;
                let discarded = self.is_dirty();
                self.new_project(&p.patient_code);
                Ok(json!({
                    "patient_code": p.patient_code,
                    "discarded_unsaved_changes": discarded,
                }))
            }
            builtins::PROJECT_OPEN => {
                let p: OpenParams = parse_params(params).map_err(invalid)?;
                self.open(Path::new(&p.path))
            }
            builtins::PROJECT_SAVE => {
                let p: SaveParams = parse_params(params).map_err(invalid)?;
                self.save(p.path.as_deref().map(Path::new))
            }
            builtins::EDIT_UNDO | builtins::EDIT_REDO => {
                let _: NoParams = parse_params(params).map_err(invalid)?;
                let done = if id == builtins::EDIT_UNDO {
                    self.undo()
                } else {
                    self.redo()
                };
                Ok(json!({
                    "done": done,
                    "undo_depth": self.undo.undo_depth(),
                    "redo_depth": self.undo.redo_depth(),
                }))
            }
            builtins::STEPS_LIST => {
                let _: NoParams = parse_params(params).map_err(invalid)?;
                Ok(self.steps_json())
            }
            builtins::STEPS_REWIND => {
                let p: RewindParams = parse_params(params).map_err(invalid)?;
                let g = self.graph.as_ref().ok_or(SessionError::NoStepGraph)?;
                let r = self
                    .history
                    .rewind(g, &p.step, self.undo.current().clone())?;
                Ok(json!({
                    "step": r.step,
                    "removed": r.removed,
                    "undo_available": r.undo_available,
                }))
            }
            builtins::VIEW_SET => {
                let obj = match params {
                    Value::Null => Map::new(),
                    Value::Object(m) => m.clone(),
                    _ => {
                        return Err(invalid(CommandError::Invalid(
                            "parameters: expected an object".into(),
                        )));
                    }
                };
                for (k, v) in obj {
                    if v.is_null() {
                        self.view.remove(&k);
                    } else {
                        self.view.insert(k, v);
                    }
                }
                Ok(Value::Object(self.view.clone()))
            }
            _ => Err(SessionError::UnknownCommand(id.to_owned())),
        }
    }

    // ------------------------------------------------------------ scanner

    /// The scan controller behind the `scan.*` commands, created on first
    /// use. It is shared (`Arc<Mutex<_>>`) so that the app can keep it while
    /// the scan popup is open (live preview, buttons) and run the `scan.*`
    /// commands of the CLI / MCP bridge against the same device and scan.
    /// Lock it briefly: the slow commands (`scan.mesh-build`, `scan.stop`)
    /// hold the lock until they finish, so a UI thread should `try_lock`.
    pub fn scan_controller(&self) -> Arc<Mutex<ScanController>> {
        Arc::clone(&self.scan)
    }

    /// Runs a `scan.*` or `turntable.*` command: `scan.import` here (it
    /// needs the case), `turntable.devices` without the controller (its
    /// Bluetooth discovery blocks), the others on the scan controller.
    fn run_scan_command(&mut self, id: &str, params: &Value, ctx: &JobCtx) -> Result<Value> {
        let fail = |e: dc_scanner::ScanError| SessionError::Command {
            id: id.to_owned(),
            source: builtins::scan_error(e),
        };
        if id == dc_scanner::control::TURNTABLE_DEVICES {
            return dc_scanner::rig::turntable_devices(params).map_err(fail);
        }
        if id != dc_scanner::SCAN_IMPORT {
            let controller = self.scan_controller();
            let mut c = controller.lock().unwrap_or_else(PoisonError::into_inner);
            return c.execute(id, params, &|| ctx.is_cancelled()).map_err(fail);
        }
        let role = dc_scanner::import_role(params).map_err(fail)?;
        // the module's own import (it recognises the arch from the geometry)
        // when it has one, the core's otherwise; like the app and the MCP
        let import_id = self
            .specs
            .iter()
            .filter(|(src, _)| self.source_enabled(src))
            .map(|(_, s)| s.id)
            .find(|id| dc_module_api::is_import_scans_command(id))
            .unwrap_or(builtins::CASE_IMPORT);
        let io_error = |e: std::io::Error| SessionError::Command {
            id: id.to_owned(),
            source: CommandError::Failed(e.to_string()),
        };
        let dir = std::env::temp_dir().join(format!("dentalcraft-scan-{}", uuid::Uuid::new_v4()));
        std::fs::create_dir_all(&dir).map_err(io_error)?;
        let result = (|| {
            let path = {
                let controller = self.scan_controller();
                let c = controller.lock().unwrap_or_else(PoisonError::into_inner);
                c.export_mesh_for_import(&dir, role).map_err(fail)?
            };
            let mut p = json!({ "path": path.display().to_string() });
            if let Some(role) = role {
                p["role"] = json!(role);
            }
            self.execute_with(import_id, p, ctx)
        })();
        // best effort: the import already copied the mesh into the case
        let _ = std::fs::remove_dir_all(&dir);
        result
    }

    // ------------------------------------------------------------ project

    fn reset(&mut self, state: CaseState, history: StepHistory) {
        let state = Arc::new(state);
        self.undo.reset(state.clone());
        self.saved = state;
        self.history = history;
        self.load_issues.clear();
    }

    /// Starts a new empty, unsaved project (unsaved changes are dropped).
    pub fn new_project(&mut self, patient_code: &str) {
        self.reset(CaseState::new(Case::new(patient_code)), StepHistory::new());
        self.manifest = None;
        self.thumbnail = None;
        self.path = None;
        self.read_only = false;
    }

    /// `.dcprj` codecs of the registered modules.
    pub fn codecs(&self) -> ModuleCodecs {
        let mut codecs = ModuleCodecs::new();
        for m in self.modules.list() {
            codecs.register(m.id(), m.data_codec());
        }
        codecs
    }

    /// Opens the project at `path` (unsaved changes are dropped). Damaged
    /// or newer files open read-only; the summary lists the issues.
    pub fn open(&mut self, path: &Path) -> Result<Value> {
        let LoadOutcome {
            project,
            read_only,
            issues,
        } = dc_project::load(path, &self.codecs())?;
        let discarded = self.is_dirty();
        let history = serde_json::from_value::<StepHistory>(project.history).unwrap_or_default();
        self.reset(project.case_state, history);
        self.manifest = Some(project.manifest);
        self.thumbnail = project.thumbnail;
        self.path = Some(path.to_owned());
        self.read_only = read_only;
        self.load_issues = issues.iter().map(ToString::to_string).collect();
        let state = self.undo.current();
        Ok(json!({
            "path": path.display().to_string(),
            "read_only": read_only,
            "issues": self.load_issues,
            "patient_code": state.case.patient_code,
            "scans": state.case.scans.len(),
            "modules": state.modules.keys().collect::<Vec<_>>(),
            "discarded_unsaved_changes": discarded,
        }))
    }

    /// Saves the project to `path` (`.dcprj` appended when missing), or to
    /// the open file when `None`. A file opened read-only is not
    /// overwritten.
    pub fn save(&mut self, path: Option<&Path>) -> Result<Value> {
        let target = match path {
            Some(p) => with_extension(p.to_owned()),
            None => self.path.clone().ok_or(SessionError::NoPath)?,
        };
        if self.read_only && self.path.as_deref() == Some(target.as_path()) {
            return Err(SessionError::ReadOnly(target));
        }
        let now = now_rfc3339();
        let manifest = match &self.manifest {
            Some(m) => Manifest {
                modified: now,
                ..m.clone()
            },
            None => Manifest::new(self.app.clone(), now),
        };
        let current = self.undo.current().clone();
        let project = Project {
            manifest,
            case_state: CaseState::clone(&current),
            history: serde_json::to_value(&self.history)?,
            thumbnail: self.thumbnail.clone(),
        };
        dc_project::save(&target, &project)?;
        let bytes = std::fs::metadata(&target).map(|m| m.len()).unwrap_or(0);
        let project_id = project.manifest.project_id;
        self.manifest = Some(project.manifest);
        self.path = Some(target.clone());
        self.read_only = false;
        self.load_issues.clear();
        self.saved = current;
        Ok(json!({
            "path": target.display().to_string(),
            "bytes": bytes,
            "project_id": project_id,
        }))
    }

    // ------------------------------------------------------------ steps

    fn steps_json(&self) -> Value {
        let status = self.graph.as_ref().map(|g| {
            self.history
                .status_from_records(g)
                .into_iter()
                .map(|s| {
                    json!({
                        "id": s.id,
                        "state": s.state,
                        "failure": s.failure,
                        "stale_because": s.stale_because,
                        "auto": s.auto,
                    })
                })
                .collect::<Vec<_>>()
        });
        json!({
            "rev": self.history.rev(),
            "records": self.history.records(),
            "status": status,
            "can_undo_rewind": self.history.can_undo_rewind(),
        })
    }

    // ------------------------------------------------------------ inspect

    /// JSON summary of the session: project, case, scans, meshes with
    /// counts, module states, steps, undo depth and view. With `path`
    /// (`a.b.0` or JSON pointer `/a/b/0`) only that part is returned, in
    /// full; without it, long arrays inside module states are abbreviated.
    pub fn inspect(&self, path: Option<&str>) -> Result<Value> {
        inspect::inspect(self, path)
    }
}

/// `path` with the `.dcprj` extension added when it has another one or none.
pub fn with_extension(path: PathBuf) -> PathBuf {
    let ext_ok = path
        .extension()
        .is_some_and(|e| e.eq_ignore_ascii_case(dc_project::FILE_EXTENSION));
    if ext_ok {
        path
    } else {
        let mut s = path.into_os_string();
        s.push(".");
        s.push(dc_project::FILE_EXTENSION);
        PathBuf::from(s)
    }
}

/// Current UTC time as RFC 3339 (`2026-10-31T11:20:05Z`).
pub fn now_rfc3339() -> String {
    let secs = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_or(0, |d| d.as_secs());
    rfc3339(secs)
}

/// Formats Unix seconds as RFC 3339 UTC.
pub fn rfc3339(unix_secs: u64) -> String {
    let days = (unix_secs / 86_400) as i64;
    let rem = unix_secs % 86_400;
    // civil-from-days (Howard Hinnant), proleptic Gregorian calendar
    let z = days + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z - era * 146_097;
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = doy - (153 * mp + 2) / 5 + 1;
    let m = if mp < 10 { mp + 3 } else { mp - 9 };
    let y = yoe + era * 400 + i64::from(m <= 2);
    format!(
        "{y:04}-{m:02}-{d:02}T{:02}:{:02}:{:02}Z",
        rem / 3600,
        rem % 3600 / 60,
        rem % 60
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rfc3339_formatting() {
        assert_eq!(rfc3339(0), "1970-01-01T00:00:00Z");
        assert_eq!(rfc3339(951_782_400), "2000-02-29T00:00:00Z");
        assert_eq!(rfc3339(1_793_445_605), "2026-10-31T11:20:05Z");
        assert!(now_rfc3339().ends_with('Z'));
    }

    #[test]
    fn extension_is_added() {
        assert_eq!(
            with_extension(PathBuf::from("/tmp/a")),
            PathBuf::from("/tmp/a.dcprj")
        );
        assert_eq!(
            with_extension(PathBuf::from("/tmp/a.DCPRJ")),
            PathBuf::from("/tmp/a.DCPRJ")
        );
    }

    #[test]
    fn translations_are_complete() {
        let dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("i18n");
        dc_i18n::parity::check_dir(&dir).unwrap();
    }

    #[test]
    fn builtin_labels_and_docs_resolve() {
        let s = Session::new();
        for c in s.list_commands() {
            let d = s.describe(&c.id).unwrap();
            assert_ne!(d.label, d.label_key, "{}", c.id);
            assert_ne!(d.doc, d.doc_key, "{}", c.id);
            assert_eq!(d.source, SESSION_SOURCE);
        }
    }
}
