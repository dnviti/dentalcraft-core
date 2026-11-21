//! Contract between the DentalCraft shell and its dental modules.
//!
//! A module (e.g. `aligners`) implements [`Module`]: it declares its menu
//! ([`MenuTree`]), its commands ([`CommandDesc`]) and the agent-drivable
//! ones with their parameters ([`CommandSpec`]), draws its inspector with
//! egui, describes what the viewport shows ([`dc_scene::SceneItem`]) and
//! provides a [`ModuleCodec`] (the one of `dc-project`) for its data in
//! `.dcprj` files. Modules are
//! compiled in and collected in a [`ModuleRegistry`]; which ones are usable
//! depends on the user's entitlements.

mod menu;
mod spec;

use std::sync::Arc;

pub use dc_project::{ModuleCodec, ModuleCodecs};
pub use menu::{LeafInput, MenuItem, MenuKind, MenuStatus, MenuTree, ScannerInput};
pub use spec::{CommandOutput, CommandRunFn, CommandSpec, SpecCommand};

use dc_case::CaseState;
use dc_command::Command;
use dc_jobs::JobSystem;
use dc_scene::{SceneEntry, SceneItem};

/// Errors of the module layer.
#[derive(Debug, thiserror::Error)]
pub enum ModuleError {
    /// A module with this id is already registered.
    #[error("module {0} is already registered")]
    Duplicate(String),
    /// No module with this id.
    #[error("unknown module {0}")]
    UnknownModule(String),
    /// The module has no command with this id.
    #[error("unknown command {0}")]
    UnknownCommand(String),
}

/// Describes a command a module offers.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CommandDesc {
    /// Stable id, e.g. `aligners.segment`.
    pub id: &'static str,
    /// i18n key of the label.
    pub label_key: &'static str,
    /// i18n key of the tooltip, if any.
    pub tooltip_key: Option<&'static str>,
    /// Keyboard shortcut, egui notation (e.g. `Ctrl+Shift+S`).
    pub shortcut: Option<&'static str>,
    /// Implemented or placeholder.
    pub status: MenuStatus,
}

impl CommandDesc {
    /// Ready command without tooltip or shortcut.
    pub const fn new(id: &'static str, label_key: &'static str) -> Self {
        Self {
            id,
            label_key,
            tooltip_key: None,
            shortcut: None,
            status: MenuStatus::Ready,
        }
    }

    /// Same command marked as a placeholder.
    pub const fn stub(mut self) -> Self {
        self.status = MenuStatus::Stub;
        self
    }
}

/// Severity of a [`Toast`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ToastLevel {
    /// Information.
    Info,
    /// Operation completed.
    Success,
    /// Something to look at.
    Warning,
    /// Operation failed.
    Error,
}

/// Short notification shown by the shell.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Toast {
    /// Severity.
    pub level: ToastLevel,
    /// i18n key of the message.
    pub message_key: String,
    /// Untranslated detail (e.g. an error message), if any.
    pub detail: Option<String>,
}

impl Toast {
    /// Toast of the given level.
    pub fn new(level: ToastLevel, message_key: impl Into<String>) -> Self {
        Self {
            level,
            message_key: message_key.into(),
            detail: None,
        }
    }

    /// Adds an untranslated detail.
    pub fn with_detail(mut self, detail: impl Into<String>) -> Self {
        self.detail = Some(detail.into());
        self
    }
}

/// Last segment of the command ids that act as "Import scans" (e.g.
/// `aligners.case.import-scans`), whatever the module: a surface that offers
/// an import (the app, the MCP `import_scan` tool) prefers such a command to
/// the module-agnostic `case.import`.
pub const IMPORT_SCANS_SUFFIX: &str = "import-scans";

/// `true` when `command_id` is an "Import scans" entry: its last
/// dot-separated segment is [`IMPORT_SCANS_SUFFIX`].
pub fn is_import_scans_command(command_id: &str) -> bool {
    command_id
        .rsplit('.')
        .next()
        .is_some_and(|last| last == IMPORT_SCANS_SUFFIX)
}

/// i18n key of the toast shown for placeholder commands.
pub const NOT_AVAILABLE_KEY: &str = "module-command-not-available";

/// Key of the "advanced options and parameters" preference in egui memory.
fn advanced_ui_id() -> egui::Id {
    egui::Id::unique("dc-module-api::advanced-ui")
}

/// Whether the user turned on "Advanced options and parameters" in the
/// application settings. Off by default: inspectors then show only the
/// automatic runs and the most useful manual corrections, and hide tuning
/// parameters and technical readouts. The shell sets it every frame with
/// [`set_advanced_ui`]; modules and widgets read it here.
pub fn advanced_ui(ctx: &egui::Context) -> bool {
    ctx.data(|d| d.get_temp::<bool>(advanced_ui_id()))
        .unwrap_or(false)
}

/// Sets the preference read by [`advanced_ui`]. Called by the shell.
pub fn set_advanced_ui(ctx: &egui::Context, on: bool) {
    ctx.data_mut(|d| d.insert_temp(advanced_ui_id(), on));
}

/// What a module can reach while handling UI or a command invocation.
pub struct ModuleCtx<'a> {
    state: &'a Arc<CaseState>,
    jobs: &'a JobSystem,
    commands: Vec<Box<dyn Command>>,
    toasts: Vec<Toast>,
}

impl<'a> ModuleCtx<'a> {
    /// Context over the current state and the shell's job system.
    pub fn new(state: &'a Arc<CaseState>, jobs: &'a JobSystem) -> Self {
        Self {
            state,
            jobs,
            commands: Vec::new(),
            toasts: Vec::new(),
        }
    }

    /// The current case state (read only: changes go through commands).
    pub fn state(&self) -> &Arc<CaseState> {
        self.state
    }

    /// The job system, for background work that does not change the case
    /// (previews, analyses); changes go through [`ModuleCtx::queue`].
    pub fn jobs(&self) -> &JobSystem {
        self.jobs
    }

    /// Queues a command; the shell runs it (in the background when long)
    /// and pushes the result on the undo stack.
    pub fn queue(&mut self, cmd: Box<dyn Command>) {
        self.commands.push(cmd);
    }

    /// Shows a notification.
    pub fn toast(&mut self, toast: Toast) {
        self.toasts.push(toast);
    }

    /// Takes the queued commands and toasts (called by the shell).
    pub fn finish(self) -> (Vec<Box<dyn Command>>, Vec<Toast>) {
        (self.commands, self.toasts)
    }
}

/// A dental module.
pub trait Module: Send {
    /// Stable internal id (e.g. `aligners`), also the key of its data in
    /// the case state and in `.dcprj` files.
    fn id(&self) -> &'static str;

    /// i18n key of the display name.
    fn name_key(&self) -> &'static str;

    /// Module version (semver).
    fn version(&self) -> &'static str;

    /// Menu shown in the workspace's left panel.
    fn menu(&self) -> MenuTree;

    /// Commands the module offers.
    fn commands(&self) -> Vec<CommandDesc>;

    /// Registered commands with a parameter schema and a run function, as
    /// listed and executed by the CLI and the MCP server. The default has
    /// none.
    fn command_specs(&self) -> Vec<CommandSpec> {
        Vec::new()
    }

    /// Runs command `command_id` (menu click or shortcut). The default
    /// shows the "not available yet" toast for placeholder commands.
    fn invoke(&mut self, command_id: &str, ctx: &mut ModuleCtx<'_>) -> Result<(), ModuleError> {
        match self.commands().iter().find(|c| c.id == command_id) {
            Some(c) if c.status == MenuStatus::Stub => {
                ctx.toast(Toast::new(ToastLevel::Info, NOT_AVAILABLE_KEY));
                Ok(())
            }
            _ => Err(ModuleError::UnknownCommand(command_id.to_owned())),
        }
    }

    /// `true` when the case already holds what menu leaf `leaf_id` produces,
    /// read from the data of `state`. The default knows of nothing; so does
    /// a module for a leaf that leaves no trace in the case.
    fn leaf_done(&self, _state: &CaseState, _leaf_id: &str) -> bool {
        false
    }

    /// The user selected menu row `id` (a group or a command item). The
    /// shell calls this on every click, before [`Module::invoke`] for
    /// commands. The default ignores it.
    fn select(&mut self, _id: &str) {}

    /// Draws the inspector panel.
    fn inspector(&mut self, ui: &mut egui::Ui, ctx: &mut ModuleCtx<'_>);

    /// Items to draw for `state`.
    fn scene(&self, state: &CaseState) -> Vec<SceneItem>;

    /// Tagged items to draw for `state` at treatment stage `stage` (`None`
    /// or `Some(0)`: the initial situation; `Some(n)`: the aligner with
    /// index `n`). Geometry is in case coordinates. The shell draws these
    /// in the viewport and in the headless render, filters them by
    /// [`SceneEntry::arch`], and skips the scan of an arch while a shown
    /// entry has [`SceneEntry::hides_scan`]. The default wraps
    /// [`Module::scene`] in untagged entries and ignores the stage.
    fn scene_at(&self, state: &CaseState, _stage: Option<u32>) -> Vec<SceneEntry> {
        self.scene(state).into_iter().map(SceneEntry::new).collect()
    }

    /// [`Module::scene_at`] for a UI thread: never waits for slow work. What
    /// is not ready yet is left out or drawn at rest and the flag is `false`
    /// (the work goes on in the background; ask again later). The default
    /// is [`Module::scene_at`], always complete.
    fn scene_at_nowait(&self, state: &CaseState, stage: Option<u32>) -> (Vec<SceneEntry>, bool) {
        (self.scene_at(state, stage), true)
    }

    /// Codec of the module data: its current schema version, the migrations
    /// from older versions (one step each, chained) and the decoder. Register
    /// it under [`Module::id`] in a [`ModuleCodecs`] to load projects.
    fn data_codec(&self) -> ModuleCodec;
}

/// The compiled-in modules and which of them are enabled.
#[derive(Default)]
pub struct ModuleRegistry {
    modules: Vec<(Box<dyn Module>, bool)>,
}

impl ModuleRegistry {
    /// Empty registry.
    pub fn new() -> Self {
        Self::default()
    }

    /// Adds a module (enabled). Ids must be unique.
    pub fn register(&mut self, module: Box<dyn Module>) -> Result<(), ModuleError> {
        let id = module.id();
        if self.get(id).is_some() {
            return Err(ModuleError::Duplicate(id.to_owned()));
        }
        self.modules.push((module, true));
        Ok(())
    }

    /// All modules in registration order.
    pub fn list(&self) -> impl Iterator<Item = &dyn Module> {
        self.modules.iter().map(|(m, _)| m.as_ref())
    }

    /// Enabled modules in registration order.
    pub fn enabled(&self) -> impl Iterator<Item = &dyn Module> {
        self.modules
            .iter()
            .filter(|(_, e)| *e)
            .map(|(m, _)| m.as_ref())
    }

    /// The tagged scene entries of the enabled modules at `stage` (see
    /// [`Module::scene_at`]), in registration order.
    pub fn scene_at(&self, state: &CaseState, stage: Option<u32>) -> Vec<SceneEntry> {
        self.enabled()
            .flat_map(|m| m.scene_at(state, stage))
            .collect()
    }

    /// Ids of all modules.
    pub fn ids(&self) -> Vec<&'static str> {
        self.list().map(|m| m.id()).collect()
    }

    /// Module by id.
    pub fn get(&self, id: &str) -> Option<&dyn Module> {
        self.list().find(|m| m.id() == id)
    }

    /// Mutable module by id.
    pub fn get_mut(&mut self, id: &str) -> Option<&mut (dyn Module + 'static)> {
        self.modules
            .iter_mut()
            .find(|(m, _)| m.id() == id)
            .map(|(m, _)| m.as_mut())
    }

    /// Enables or disables a module (entitlements).
    pub fn set_enabled(&mut self, id: &str, enabled: bool) -> Result<(), ModuleError> {
        let entry = self
            .modules
            .iter_mut()
            .find(|(m, _)| m.id() == id)
            .ok_or_else(|| ModuleError::UnknownModule(id.to_owned()))?;
        entry.1 = enabled;
        Ok(())
    }

    /// `true` when the module exists and is enabled.
    pub fn is_enabled(&self, id: &str) -> bool {
        self.modules.iter().any(|(m, e)| *e && m.id() == id)
    }

    /// Number of registered modules.
    pub fn len(&self) -> usize {
        self.modules.len()
    }

    /// `true` when no module is registered.
    pub fn is_empty(&self) -> bool {
        self.modules.is_empty()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn import_scans_commands_end_with_the_suffix() {
        assert!(is_import_scans_command("aligners.case.import-scans"));
        assert!(is_import_scans_command("import-scans"));
        assert!(!is_import_scans_command("case.import"));
        assert!(!is_import_scans_command("aligners.case.import-scans-x"));
        assert!(!is_import_scans_command("aligners.import-scans.run"));
    }

    #[test]
    fn advanced_ui_is_off_until_the_shell_turns_it_on() {
        let ctx = egui::Context::default();
        assert!(!advanced_ui(&ctx));
        set_advanced_ui(&ctx, true);
        assert!(advanced_ui(&ctx));
        set_advanced_ui(&ctx, false);
        assert!(!advanced_ui(&ctx));
    }

    #[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
    struct DemoData {
        n: u32,
    }
    dc_case::impl_module_data!(DemoData, 2);

    struct Demo(&'static str);

    impl Module for Demo {
        fn id(&self) -> &'static str {
            self.0
        }
        fn name_key(&self) -> &'static str {
            "demo-name"
        }
        fn version(&self) -> &'static str {
            "0.1.0"
        }
        fn menu(&self) -> MenuTree {
            MenuTree::new(vec![MenuItem::command("demo.run", "demo-run", "demo.run")])
        }
        fn commands(&self) -> Vec<CommandDesc> {
            vec![CommandDesc::new("demo.run", "demo-run").stub()]
        }
        fn inspector(&mut self, _ui: &mut egui::Ui, _ctx: &mut ModuleCtx<'_>) {}
        fn scene(&self, _state: &CaseState) -> Vec<SceneItem> {
            Vec::new()
        }
        fn data_codec(&self) -> ModuleCodec {
            ModuleCodec::typed::<DemoData>(2).with_migration(1, |v| {
                *v = serde_json::json!({ "n": v["count"] });
                Ok(())
            })
        }
    }

    #[test]
    fn default_scene_at_wraps_scene_and_skips_disabled_modules() {
        let mut reg = ModuleRegistry::new();
        reg.register(Box::new(Demo("aligners"))).unwrap();
        let state = CaseState::new(dc_case::Case::new("T"));
        assert!(reg.scene_at(&state, Some(2)).is_empty());
        reg.set_enabled("aligners", false).unwrap();
        assert!(reg.scene_at(&state, None).is_empty());
    }

    #[test]
    fn registry_register_get_enable() {
        let mut reg = ModuleRegistry::new();
        reg.register(Box::new(Demo("aligners"))).unwrap();
        reg.register(Box::new(Demo("implants"))).unwrap();
        assert!(matches!(
            reg.register(Box::new(Demo("aligners"))),
            Err(ModuleError::Duplicate(_))
        ));
        assert_eq!(reg.ids(), vec!["aligners", "implants"]);
        assert_eq!(reg.get("implants").unwrap().name_key(), "demo-name");
        assert!(reg.get("nope").is_none());
        reg.set_enabled("implants", false).unwrap();
        assert!(!reg.is_enabled("implants"));
        assert_eq!(reg.enabled().count(), 1);
        assert!(reg.set_enabled("nope", true).is_err());
        assert!(reg.get_mut("aligners").is_some());
    }

    #[test]
    fn stub_command_shows_toast() {
        let jobs = JobSystem::new(1).unwrap();
        let state = Arc::new(CaseState::default());
        let mut ctx = ModuleCtx::new(&state, &jobs);
        let mut m = Demo("aligners");
        m.invoke("demo.run", &mut ctx).unwrap();
        assert!(m.invoke("demo.other", &mut ctx).is_err());
        let (cmds, toasts) = ctx.finish();
        assert!(cmds.is_empty());
        assert_eq!(
            toasts,
            vec![Toast::new(ToastLevel::Info, NOT_AVAILABLE_KEY)]
        );
    }

    #[test]
    fn codec_migrates_and_refuses_newer() {
        let c = Demo("x").data_codec();
        assert_eq!(c.current_version(), 2);
        let d = c.load(serde_json::json!({"count": 5}), 1).unwrap();
        assert_eq!(
            d.as_any().downcast_ref::<DemoData>(),
            Some(&DemoData { n: 5 })
        );
        assert!(c.load(serde_json::json!({}), 3).is_err());
        // no step registered from version 0
        assert!(c.load(serde_json::json!({}), 0).is_err());
    }

    #[test]
    fn command_specs_default_to_none() {
        assert!(Demo("x").command_specs().is_empty());
    }

    #[test]
    fn leaf_done_defaults_to_false() {
        assert!(!Demo("x").leaf_done(&CaseState::default(), "demo.run"));
    }

    #[test]
    fn select_defaults_to_a_no_op() {
        let mut m = Demo("x");
        m.select("anything");
    }
}
