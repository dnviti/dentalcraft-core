//! Built-in commands of the session, as [`CommandSpec`]s.
//!
//! `case.import` and `case.set-role` are ordinary state commands (they return
//! the new case state), so they run through the generic path and are
//! undoable. `case.import` is a thin wrapper of [`dc_case::import::import_files`]:
//! the core stays module-agnostic, so a file without a hint stays
//! provisional. The others act on the session itself (project file, undo
//! stack, step history, view): their spec carries the id, the i18n keys and
//! the parameter schema, and [`crate::Session::execute`] handles them; their
//! `run` function only reports that.

use std::collections::BTreeSet;

use dc_case::import::{ImportFile, ImportReport, ImportRequest, import_files, roles_changed};
use dc_case::{CaseState, MAX_BITE_SCANS, RoleSource, Scan, ScanRole};
use dc_command::{CommandError, JobCtx};
use dc_module_api::{CommandOutput, CommandSpec};
use schemars::JsonSchema;
use serde::Deserialize;
use serde::de::DeserializeOwned;
use serde_json::{Map, Value, json};
use uuid::Uuid;

/// `project.new`.
pub const PROJECT_NEW: &str = "project.new";
/// `project.open`.
pub const PROJECT_OPEN: &str = "project.open";
/// `project.save`.
pub const PROJECT_SAVE: &str = "project.save";
/// `case.import`.
pub const CASE_IMPORT: &str = "case.import";
/// `case.set-role`.
pub const CASE_SET_ROLE: &str = "case.set-role";
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

/// Arch or bite.
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

/// Role of an imported scan, or `auto` to let the import decide.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub(crate) enum RoleChoice {
    /// The import decides: file name, then a free arch (provisional).
    Auto,
    /// Upper (maxillary) arch.
    Upper,
    /// Lower (mandibular) arch.
    Lower,
    /// Bite registration.
    Bite,
}

impl RoleChoice {
    /// The role chosen, `None` for `auto`.
    fn role(self) -> Option<ScanRole> {
        match self {
            Self::Auto => None,
            Self::Upper => Some(ScanRole::Upper),
            Self::Lower => Some(ScanRole::Lower),
            Self::Bite => Some(ScanRole::Bite),
        }
    }
}

/// One file of a `case.import` batch.
#[derive(Debug, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub(crate) struct ImportFileParam {
    /// Path of the STL, OBJ or PLY file.
    pub path: String,
    /// Arch or bite of this file; `auto` or missing lets the import decide.
    #[serde(default)]
    pub role: Option<RoleChoice>,
}

/// Parameters of `case.import`: exactly one of `path` and `files`.
#[derive(Debug, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
#[schemars(extend("oneOf" = [{ "required": ["path"] }, { "required": ["files"] }]))]
pub(crate) struct ImportParams {
    /// Path of one STL, OBJ or PLY file. Give this or `files`.
    #[serde(default)]
    pub path: Option<String>,
    /// Several files, each with its own optional role. Give this or `path`.
    #[serde(default)]
    pub files: Option<Vec<ImportFileParam>>,
    /// Arch or bite of `path`; the default for the entries of `files` that
    /// have no role of their own. `auto` or missing lets the import decide
    /// (file name, then a free arch, marked provisional).
    #[serde(default)]
    pub role: Option<RoleChoice>,
}

/// Parameters of `case.set-role`.
#[derive(Debug, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub(crate) struct SetRoleParams {
    /// Role the scan has now.
    pub from: RoleParam,
    /// Role the scan gets. A scan already there takes the `from` role.
    pub to: RoleParam,
    /// Id of the scan, needed when `from` is `bite` and the case has two
    /// bite scans.
    #[serde(default)]
    pub scan_id: Option<String>,
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
        CommandSpec {
            id: CASE_SET_ROLE,
            label_key: "session-cmd-case-set-role",
            doc_key: "session-cmd-case-set-role-doc",
            params_schema: || schemars::schema_for!(SetRoleParams).to_value(),
            run: run_set_role,
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

pub(crate) fn role_str(role: ScanRole) -> &'static str {
    match role {
        ScanRole::Upper => "upper",
        ScanRole::Lower => "lower",
        ScanRole::Bite => "bite",
    }
}

pub(crate) fn role_source_str(source: RoleSource) -> &'static str {
    match source {
        RoleSource::Explicit => "explicit",
        RoleSource::FileName => "file_name",
        RoleSource::Detected => "detected",
        RoleSource::Provisional => "provisional",
    }
}

fn invalid(key: &str, args: &[(&str, &str)]) -> CommandError {
    CommandError::Invalid(dc_i18n::tr_args(key, args))
}

/// The files of an import, from exactly one of `path` and `files`.
fn import_request(p: &ImportParams) -> Result<ImportRequest, CommandError> {
    let default = p.role.and_then(RoleChoice::role);
    let files = match (&p.path, &p.files) {
        (Some(path), None) => vec![ImportFile {
            path: path.into(),
            role: default,
        }],
        (None, Some(files)) if !files.is_empty() => files
            .iter()
            .map(|f| ImportFile {
                path: (&f.path).into(),
                role: f.role.map_or(default, RoleChoice::role),
            })
            .collect(),
        (None, Some(_)) => return Err(invalid("session-import-error-no-files", &[])),
        _ => return Err(invalid("session-import-error-path-or-files", &[])),
    };
    Ok(ImportRequest { files })
}

/// The reasons of the failed files, one sentence each.
fn failure_text(report: &ImportReport) -> String {
    let reasons: Vec<String> = report
        .failed
        .iter()
        .map(|f| match &f.detail {
            Some(d) => format!("{} ({d})", f.reason),
            None => f.reason.clone(),
        })
        .collect();
    dc_i18n::tr_args("session-import-nothing", &[("reasons", &reasons.join(" "))])
}

fn to_value(v: impl serde::Serialize) -> Result<Value, CommandError> {
    serde_json::to_value(v).map_err(|e| CommandError::Failed(e.to_string()))
}

fn run_import(
    state: &CaseState,
    params: &Value,
    ctx: &JobCtx,
) -> Result<CommandOutput, CommandError> {
    let p: ImportParams = parse_params(params)?;
    let request = import_request(&p)?;
    ctx.check_cancelled()?;
    let mut next = state.clone();
    let report = import_files(&mut next, &request, None, ctx);
    if report.cancelled {
        return Err(CommandError::Cancelled);
    }
    if report.imported.is_empty() {
        return Err(CommandError::Failed(failure_text(&report)));
    }
    let replaced = state
        .case
        .scans
        .iter()
        .any(|s| next.case.scan_by_id(s.id).is_none());
    let imported = to_value(&report.imported)?;
    // the single-file fields describe the first imported scan
    let mut out = Map::new();
    if let Some(Value::Object(first)) = imported.get(0) {
        for key in [
            "scan_id",
            "role",
            "role_source",
            "source_name",
            "source_units",
            "mesh",
            "vertices",
            "faces",
        ] {
            if let Some(v) = first.get(key) {
                out.insert(key.to_owned(), v.clone());
            }
        }
    }
    out.insert("replaced".into(), json!(replaced));
    out.insert("imported".into(), imported);
    out.insert("warnings".into(), to_value(&report.warnings)?);
    out.insert("failed".into(), to_value(&report.failed)?);
    out.insert("moved".into(), to_value(&report.moved)?);
    out.insert("cleanup".into(), to_value(report.cleanup)?);
    out.insert("module_cleared".into(), json!(report.module_cleared));
    Ok((next, Value::Object(out)))
}

fn run_set_role(
    state: &CaseState,
    params: &Value,
    _: &JobCtx,
) -> Result<CommandOutput, CommandError> {
    let p: SetRoleParams = parse_params(params)?;
    let (from, to) = (ScanRole::from(p.from), ScanRole::from(p.to));
    let role_arg = [("role", role_str(from))];
    let case = &state.case;
    let candidates: Vec<&Scan> = case.scans_of(from).collect();
    let scan = match p.scan_id.as_deref() {
        Some(text) => {
            let id = Uuid::parse_str(text.trim())
                .map_err(|_| invalid("session-set-role-error-bad-id", &[("id", text)]))?;
            let found = candidates.iter().find(|s| s.id == id);
            *found.ok_or_else(|| {
                invalid(
                    "session-set-role-error-wrong-scan",
                    &[("id", text), ("role", role_str(from))],
                )
            })?
        }
        None => match candidates.as_slice() {
            [] => return Err(invalid("session-set-role-error-no-scan", &role_arg)),
            [one] => one,
            _ => return Err(invalid("session-set-role-error-needs-scan-id", &role_arg)),
        },
    };
    let occupant = if from == to {
        None
    } else if to == ScanRole::Bite {
        if case.scans_of(ScanRole::Bite).count() >= MAX_BITE_SCANS {
            let max = MAX_BITE_SCANS.to_string();
            return Err(invalid(
                "session-set-role-error-bite-full",
                &[("max", &max)],
            ));
        }
        None
    } else {
        case.scan(to).map(|s| (s.id, s.source_name.clone()))
    };

    let mut next = state.clone();
    let before = next.referenced_blobs();
    for s in &mut next.case.scans {
        if s.id == scan.id {
            s.role = to;
            s.role_source = RoleSource::Explicit;
        } else if occupant.as_ref().is_some_and(|(id, _)| *id == s.id) {
            s.role = from;
            s.role_source = RoleSource::Explicit;
        }
    }
    next.case.scans.sort_by_key(|s| s.role);
    let touched: BTreeSet<ScanRole> = if from == to {
        BTreeSet::new()
    } else {
        [from, to].into()
    };
    let world_frame_reset = touched.iter().any(|r| *r != ScanRole::Bite);
    let module_cleared = if touched.is_empty() {
        Vec::new()
    } else {
        roles_changed(&mut next, &touched, &before)
    };
    let swapped = occupant.map(|(id, name)| {
        json!({
            "scan_id": id,
            "source_name": name,
            "from": role_str(to),
            "to": role_str(from),
        })
    });
    Ok((
        next,
        json!({
            "scan_id": scan.id,
            "source_name": scan.source_name,
            "from": role_str(from),
            "to": role_str(to),
            "role_source": role_source_str(RoleSource::Explicit),
            "swapped": swapped,
            "world_frame_reset": world_frame_reset,
            "module_cleared": module_cleared,
        }),
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn specs_have_unique_ids_and_object_schemas() {
        let specs = specs();
        let mut ids: Vec<&str> = specs.iter().map(|s| s.id).collect();
        ids.sort_unstable();
        ids.dedup();
        assert_eq!(ids.len(), specs.len());
        for s in &specs {
            assert_eq!(s.schema()["type"], "object", "{}", s.id);
            assert_eq!(
                is_session_command(s.id),
                s.id != CASE_IMPORT && s.id != CASE_SET_ROLE
            );
        }
    }

    #[test]
    fn params_reject_unknown_fields() {
        assert!(parse_params::<OpenParams>(&json!({"path": "a", "x": 1})).is_err());
        assert!(parse_params::<OpenParams>(&Value::Null).is_err());
        assert!(parse_params::<NoParams>(&Value::Null).is_ok());
        let p: ImportParams = parse_params(&json!({"path": "a.stl", "role": "bite"})).unwrap();
        assert_eq!(p.role, Some(RoleChoice::Bite));
        let p: ImportParams = parse_params(&json!({"path": "a.stl", "role": "auto"})).unwrap();
        assert_eq!(p.role.and_then(RoleChoice::role), None);
        assert!(parse_params::<ImportParams>(&json!({"path": "a.stl", "role": "x"})).is_err());
        assert!(parse_params::<ImportParams>(&json!({"files": [{"path": "a", "x": 1}]})).is_err());
        assert!(parse_params::<SetRoleParams>(&json!({"from": "upper"})).is_err());
    }

    #[test]
    fn import_request_needs_exactly_one_of_path_and_files() {
        let req = |v: Value| import_request(&parse_params::<ImportParams>(&v).unwrap());
        assert!(req(json!({})).is_err());
        assert!(req(json!({"path": "a.stl", "files": [{"path": "b.stl"}]})).is_err());
        assert!(req(json!({"files": []})).is_err());
        let r = req(json!({"files": [{"path": "a.stl", "role": "lower"}, {"path": "b.stl"}], "role": "bite"}))
            .unwrap();
        assert_eq!(r.files[0].role, Some(ScanRole::Lower));
        assert_eq!(r.files[1].role, Some(ScanRole::Bite));
        let r = req(json!({"path": "a.stl", "role": "auto"})).unwrap();
        assert_eq!(r.files[0].role, None);
    }
}
