//! Checks that every language defines the same Fluent keys.
//!
//! Use it in a test of each crate with a bundle:
//!
//! ```ignore
//! #[test]
//! fn translations_are_complete() {
//!     let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("i18n");
//!     dc_i18n::parity::check_dir(&dir).unwrap();
//! }
//! ```

use std::collections::{BTreeMap, BTreeSet};
use std::path::Path;

use fluent_syntax::ast::Entry;

use crate::Lang;

/// Message ids (`key`), attribute ids (`key.attr`) and term ids (`-term`)
/// defined in a Fluent source. Errors when the source does not parse.
pub fn ids(ftl: &str) -> Result<BTreeSet<String>, String> {
    let res = fluent_syntax::parser::parse(ftl).map_err(|(_, errs)| format!("{errs:?}"))?;
    let mut out = BTreeSet::new();
    for entry in res.body {
        let (id, attrs) = match entry {
            Entry::Message(m) => (m.id.name.to_owned(), m.attributes),
            Entry::Term(t) => (format!("-{}", t.id.name), t.attributes),
            _ => continue,
        };
        for a in attrs {
            out.insert(format!("{id}.{}", a.id.name));
        }
        out.insert(id);
    }
    Ok(out)
}

/// Compares the `.ftl` files of every [`Lang`] under `i18n_dir`
/// (`<i18n_dir>/<lang>/*.ftl`): same file names and same ids in each file.
/// The error lists every difference.
pub fn check_dir(i18n_dir: &Path) -> Result<(), String> {
    let mut per_lang: BTreeMap<&str, BTreeMap<String, BTreeSet<String>>> = BTreeMap::new();
    for lang in Lang::ALL {
        let dir = i18n_dir.join(lang.code());
        let mut files = BTreeMap::new();
        let entries = std::fs::read_dir(&dir).map_err(|e| format!("{}: {e}", dir.display()))?;
        for entry in entries {
            let path = entry.map_err(|e| e.to_string())?.path();
            if path.extension().is_some_and(|e| e == "ftl") {
                let text = std::fs::read_to_string(&path)
                    .map_err(|e| format!("{}: {e}", path.display()))?;
                let name = path
                    .file_name()
                    .unwrap_or_default()
                    .to_string_lossy()
                    .into_owned();
                files.insert(
                    name,
                    ids(&text).map_err(|e| format!("{}: {e}", path.display()))?,
                );
            }
        }
        per_lang.insert(lang.code(), files);
    }
    let base = &per_lang[Lang::En.code()];
    let mut problems = Vec::new();
    for (lang, files) in &per_lang {
        for name in base.keys().filter(|n| !files.contains_key(*n)) {
            problems.push(format!("{lang}: missing file {name}"));
        }
        for (name, keys) in files {
            let Some(base_keys) = base.get(name) else {
                problems.push(format!("{lang}: extra file {name}"));
                continue;
            };
            for k in base_keys.difference(keys) {
                problems.push(format!("{lang}/{name}: missing {k}"));
            }
            for k in keys.difference(base_keys) {
                problems.push(format!("{lang}/{name}: extra {k}"));
            }
        }
    }
    if problems.is_empty() {
        Ok(())
    } else {
        Err(problems.join("\n"))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn collects_messages_attributes_and_terms() {
        let got = ids("-brand = DentalCraft\nhello = Hi\n    .title = Tip\n# comment\n").unwrap();
        let want: BTreeSet<String> = ["-brand", "hello", "hello.title"].map(String::from).into();
        assert_eq!(got, want);
        assert!(ids("= broken").is_err());
    }
}
