//! Container (`format_version`) migrations, chained on JSON values.

use serde_json::Value;

use crate::{DcprjError, FORMAT_VERSION};

/// The JSON documents a container migration may rewrite.
#[derive(Debug, Clone)]
pub struct ContainerDoc {
    /// `manifest.json`.
    pub manifest: Value,
    /// `case.json`.
    pub case: Value,
    /// `history/steps.json`, when present.
    pub history: Option<Value>,
}

type Step = fn(&mut ContainerDoc) -> Result<(), String>;

/// `(from_version, migration to from_version + 1)`.
const MIGRATIONS: &[(u32, Step)] = &[(0, v0_to_v1)];

/// Pre-release layout: `case.patient` instead of `case.patient_code`, and no
/// `units` in the manifest.
fn v0_to_v1(doc: &mut ContainerDoc) -> Result<(), String> {
    let case = doc.case.as_object_mut().ok_or("case is not an object")?;
    if let Some(p) = case.remove("patient") {
        case.entry("patient_code").or_insert(p);
    }
    let manifest = doc
        .manifest
        .as_object_mut()
        .ok_or("manifest is not an object")?;
    manifest
        .entry("units")
        .or_insert_with(|| Value::String("mm".to_owned()));
    Ok(())
}

/// Migrates `doc` from container version `from` up to [`FORMAT_VERSION`].
pub(crate) fn migrate_container(doc: &mut ContainerDoc, from: u32) -> Result<(), DcprjError> {
    let mut v = from;
    while v < FORMAT_VERSION {
        let fail = |reason: String| DcprjError::Migration {
            container: "project".to_owned(),
            from: v,
            reason,
        };
        let Some((_, step)) = MIGRATIONS.iter().find(|(f, _)| *f == v) else {
            return Err(fail(format!("no migration from version {v}")));
        };
        step(doc).map_err(fail)?;
        v += 1;
        if let Some(m) = doc.manifest.as_object_mut() {
            m.insert("format_version".to_owned(), Value::from(v));
        }
    }
    Ok(())
}
