//! The texts of the built-in commands without a [`dc_session::Session`]: the
//! desktop app runs them from their specs and never builds one.
//!
//! One test in its own binary, so that nothing else registers the bundle.

use dc_case::{Case, CaseState};
use dc_command::{CommandError, JobCtx};
use dc_module_api::CommandSpec;
use serde_json::json;

#[test]
fn session_keys_resolve_without_a_session() {
    assert!(!dc_i18n::has("session-import-nothing"));
    let specs = dc_session::builtins::specs();
    for s in &specs {
        assert_ne!(dc_i18n::tr(s.label_key), s.label_key, "{}", s.id);
        assert_ne!(dc_i18n::tr(s.doc_key), s.doc_key, "{}", s.id);
    }
    let spec = |id: &str| -> &CommandSpec { specs.iter().find(|s| s.id == id).unwrap() };
    let state = CaseState::new(Case::new("T"));
    let ctx = JobCtx::detached();

    let import = spec("case.import");
    let err = import
        .execute(&state, &json!({"files": []}), &ctx)
        .unwrap_err();
    assert!(
        matches!(&err, CommandError::Invalid(m) if m == "The list of files is empty: give at least one file."),
        "{err}"
    );
    let err = import
        .execute(&state, &json!({"path": "missing.stl"}), &ctx)
        .unwrap_err();
    assert!(
        matches!(&err, CommandError::Failed(m) if m.starts_with("No scan was imported. missing.stl:")),
        "{err}"
    );
    let err = spec("case.set-role")
        .execute(&state, &json!({"from": "upper", "to": "lower"}), &ctx)
        .unwrap_err();
    assert!(
        matches!(&err, CommandError::Invalid(m) if m == "The case has no scan with the role upper arch."),
        "{err}"
    );
}
