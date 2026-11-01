//! Built-in commands of the session, as [`CommandSpec`]s.
//!
//! `case.import` is an ordinary state command (it reads the mesh file and
//! returns the new case state), so it runs through the generic path and is
//! undoable. The others act on the session itself (project file, undo
//! stack, step history, view): their spec carries the id, the i18n keys and
//! the parameter schema, and [`crate::Session::execute`] handles them; their
//! `run` function only reports that.

use std::path::Path;
use std::sync::Arc;

use dc_case::{CaseState, Pose, RoleSource, Scan, ScanRole};
use dc_command::{CommandError, JobCtx};
use dc_module_api::{CommandOutput, CommandSpec};
use schemars::JsonSchema;
use serde::Deserialize;
use serde::de::DeserializeOwned;
use serde_json::{Value, json};

/// `project.new`.
pub const PROJECT_NEW: &str = "project.new";
/// `project.open`.
pub const PROJECT_OPEN: &str = "project.open";
/// `project.save`.
pub const PROJECT_SAVE: &str = "project.save";
/// `case.import`.
pub const CASE_IMPORT: &str = "case.import";
/// `edit.undo`.
pub const EDIT_UNDO: &str = "edit.undo";
/// `edit.redo`.
pub const EDIT_REDO: &str = "edit.redo";
/// `steps.list`.
pub const STEPS_LIST: &str = "steps.list";
/// `steps.rewind`.
pub const STEPS_REWIND: &str = "steps.rewind";
/// `view.set`.
pub const VIEW_SET: &str = "view.set";

/// Parses command parameters: `null` is an empty object.
pub(crate) fn parse_params<T: DeserializeOwned>(params: &Value) -> Result<T, CommandError> {
    let v = if params.is_null() {
        json!({})
    } else {
        params.clone()
    };
    serde_json::from_value(v).map_err(|e| CommandError::Invalid(format!("parameters: {e}")))
}

/// Parameters of `project.new`.
#[derive(Debug, Default, Deserialize, JsonSchema)]
#[serde(default, deny_unknown_fields)]
pub(crate) struct NewParams {
    /// Pseudonymised patient code of the new case.
    pub patient_code: String,
}

/// Parameters of `project.open`.
#[derive(Debug, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub(crate) struct OpenParams {
    /// Path of the `.dcprj` file.
    pub path: String,
}

/// Parameters of `project.save`.
#[derive(Debug, Default, Deserialize, JsonSchema)]
#[serde(default, deny_unknown_fields)]
pub(crate) struct SaveParams {
    /// Target path; the open file when missing. `.dcprj` is appended when
    /// the path has another extension or none.
    pub path: Option<String>,
}

/// Role of an imported scan.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub(crate) enum RoleParam {
    /// Upper (maxillary) arch.
    Upper,
    /// Lower (mandibular) arch.
    Lower,
    /// Bite registration.
    Bite,
}

impl From<RoleParam> for ScanRole {
    fn from(r: RoleParam) -> Self {
        match r {
            RoleParam::Upper => ScanRole::Upper,
            RoleParam::Lower => ScanRole::Lower,
            RoleParam::Bite => ScanRole::Bite,
        }
    }
}

/// Parameters of `case.import`.
#[derive(Debug, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub(crate) struct ImportParams {
    /// Path of the STL, OBJ or PLY file.
    pub path: String,
    /// Arch or bite; guessed from the file name (upper, maxil, superior,
    /// lower, mandib, inferior, bite, morso) or the first free role when
    /// missing.
    #[serde(default)]
    pub role: Option<RoleParam>,
}

/// Parameters of `steps.rewind`.
#[derive(Debug, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub(crate) struct RewindParams {
    /// Id of the step to go back to.
    pub step: String,
}

/// Parameters of the commands without parameters.
#[derive(Debug, Default, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub(crate) struct NoParams {}

fn view_schema() -> Value {
    json!({
        "type": "object",
        "description": "View parameters stored for the renderer; keys set to null are removed. Other keys are kept as given.",
        "properties": {
            "preset": {
                "type": "string",
                "enum": ["front", "back", "left", "right", "top", "bottom", "iso"],
                "description": "Camera preset."
            },
            "width": { "type": "integer", "minimum": 1, "description": "Image width in pixels." },
            "height": { "type": "integer", "minimum": 1, "description": "Image height in pixels." },
            "visible": {
                "type": "array",
                "items": { "type": "string" },
                "description": "Items to show (e.g. scan roles); all when missing."
            }
        },
        "additionalProperties": true
    })
}

fn handled_by_session(_: &CaseState, _: &Value, _: &JobCtx) -> Result<CommandOutput, CommandError> {
    Err(CommandError::Invalid(
        "session command: run it through Session::execute".into(),
    ))
}

macro_rules! session_spec {
    ($id:expr, $key:literal, $params:ty) => {
        CommandSpec {
            id: $id,
            label_key: concat!("session-cmd-", $key),
            doc_key: concat!("session-cmd-", $key, "-doc"),
            params_schema: || schemars::schema_for!($params).to_value(),
            run: handled_by_session,
        }
    };
}

/// Every built-in command, in listing order.
pub fn specs() -> Vec<CommandSpec> {
    vec![
        session_spec!(PROJECT_NEW, "project-new", NewParams),
        session_spec!(PROJECT_OPEN, "project-open", OpenParams),
        session_spec!(PROJECT_SAVE, "project-save", SaveParams),
        CommandSpec {
            id: CASE_IMPORT,
            label_key: "session-cmd-case-import",
            doc_key: "session-cmd-case-import-doc",
            params_schema: || schemars::schema_for!(ImportParams).to_value(),
            run: run_import,
        },
        session_spec!(EDIT_UNDO, "edit-undo", NoParams),
        session_spec!(EDIT_REDO, "edit-redo", NoParams),
        session_spec!(STEPS_LIST, "steps-list", NoParams),
        session_spec!(STEPS_REWIND, "steps-rewind", RewindParams),
        CommandSpec {
            id: VIEW_SET,
            label_key: "session-cmd-view-set",
            doc_key: "session-cmd-view-set-doc",
            params_schema: view_schema,
            run: handled_by_session,
        },
    ]
}

/// `true` for the built-ins that act on the session rather than on the case
/// state.
pub(crate) fn is_session_command(id: &str) -> bool {
    matches!(
        id,
        PROJECT_NEW
            | PROJECT_OPEN
            | PROJECT_SAVE
            | EDIT_UNDO
            | EDIT_REDO
            | STEPS_LIST
            | STEPS_REWIND
            | VIEW_SET
    )
}

/// Role suggested by a file name, if any.
pub fn role_from_name(name: &str) -> Option<ScanRole> {
    let n = name.to_lowercase();
    let has = |words: &[&str]| words.iter().any(|w| n.contains(w));
    if has(&["bite", "morso"]) {
        Some(ScanRole::Bite)
    } else if has(&["upper", "maxil", "superior"]) {
        Some(ScanRole::Upper)
    } else if has(&["lower", "mandib", "inferior"]) {
        Some(ScanRole::Lower)
    } else {
        None
    }
}

/// Role for a new scan: the one its name suggests, otherwise the first of
/// upper, lower, bite not taken by `existing`; upper when all are taken.
fn pick_role(name: &str, existing: &[ScanRole]) -> ScanRole {
    role_from_name(name).unwrap_or_else(|| {
        [ScanRole::Upper, ScanRole::Lower, ScanRole::Bite]
            .into_iter()
            .find(|r| !existing.contains(r))
            .unwrap_or(ScanRole::Upper)
    })
}

pub(crate) fn role_str(role: ScanRole) -> &'static str {
    match role {
        ScanRole::Upper => "upper",
        ScanRole::Lower => "lower",
        ScanRole::Bite => "bite",
    }
}

fn run_import(
    state: &CaseState,
    params: &Value,
    ctx: &JobCtx,
) -> Result<CommandOutput, CommandError> {
    let p: ImportParams = parse_params(params)?;
    let path = Path::new(&p.path);
    let name = path
        .file_name()
        .map(|n| n.to_string_lossy().into_owned())
        .unwrap_or_else(|| p.path.clone());
    ctx.check_cancelled()?;
    ctx.set_progress(0.0, Some("session-cmd-case-import"));
    let mut mesh =
        dc_mesh::io::read(path).map_err(|e| CommandError::Failed(format!("{name}: {e}")))?;
    let units = dc_mesh::io::normalize_units(&mut mesh)
        .map_err(|e| CommandError::Failed(format!("{name}: {e}")))?;
    ctx.check_cancelled()?;
    let existing: Vec<ScanRole> = state.case.scans.iter().map(|s| s.role).collect();
    let role = p
        .role
        .map(ScanRole::from)
        .unwrap_or_else(|| pick_role(&name, &existing));
    let (vertices, faces) = (mesh.vertex_count(), mesh.face_count());
    let mut next = state.clone();
    let blob = next.blobs.insert_arc(Arc::new(mesh));
    let id = uuid::Uuid::new_v4();
    let replaced = next.case.scans.iter().any(|s| s.role == role);
    next.case.scans.retain(|s| s.role != role);
    next.case.scans.push(Scan {
        id,
        role,
        role_source: RoleSource::Explicit,
        source_name: name.clone(),
        source_units: units,
        import_xform: Pose::default(),
        mesh: blob,
    });
    next.case.scans.sort_by_key(|s| s.role);
    ctx.set_progress(1.0, None);
    Ok((
        next,
        json!({
            "scan_id": id,
            "role": role_str(role),
            "source_name": name,
            "source_units": units.symbol(),
            "mesh": blob.to_hex(),
            "vertices": vertices,
            "faces": faces,
            "replaced": replaced,
        }),
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn roles_from_names() {
        assert_eq!(
            role_from_name("Patient_UpperJaw.stl"),
            Some(ScanRole::Upper)
        );
        assert_eq!(role_from_name("mandibular.ply"), Some(ScanRole::Lower));
        assert_eq!(role_from_name("morso.obj"), Some(ScanRole::Bite));
        assert_eq!(role_from_name("scan.stl"), None);
        assert_eq!(pick_role("scan.stl", &[ScanRole::Upper]), ScanRole::Lower);
        assert_eq!(
            pick_role("x.stl", &[ScanRole::Upper, ScanRole::Lower, ScanRole::Bite]),
            ScanRole::Upper
        );
    }

    #[test]
    fn specs_have_unique_ids_and_object_schemas() {
        let specs = specs();
        let mut ids: Vec<&str> = specs.iter().map(|s| s.id).collect();
        ids.sort_unstable();
        ids.dedup();
        assert_eq!(ids.len(), specs.len());
        for s in &specs {
            assert_eq!(s.schema()["type"], "object", "{}", s.id);
            assert_eq!(is_session_command(s.id), s.id != CASE_IMPORT);
        }
    }

    #[test]
    fn params_reject_unknown_fields() {
        assert!(parse_params::<OpenParams>(&json!({"path": "a", "x": 1})).is_err());
        assert!(parse_params::<OpenParams>(&Value::Null).is_err());
        assert!(parse_params::<NoParams>(&Value::Null).is_ok());
        let p: ImportParams = parse_params(&json!({"path": "a.stl", "role": "bite"})).unwrap();
        assert_eq!(p.role, Some(RoleParam::Bite));
    }
}
