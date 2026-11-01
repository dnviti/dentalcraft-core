//! Role detection from a scan file name.
//!
//! The single implementation of the name hint: the Blender plugin
//! (`ortho_aligner/autoseed.py`, `arch_from_name`) is the oracle for the arch
//! words, [`role_from_name`] adds the bite words on top.
//!
//! Callers pass the bare file name (`UpperJaw.stl`), never a full path: a
//! directory such as `upper_case/` must not decide the role.

use crate::ScanRole;

/// Whole-token words naming the upper arch.
const UPPER_WORDS: [&str; 11] = [
    "upper",
    "maxilla",
    "maxillary",
    "maxillare",
    "mascellare",
    "superiore",
    "superior",
    "sup",
    "oberkiefer",
    "upperjaw",
    "arcatasuperiore",
];

/// Whole-token words naming the lower arch.
const LOWER_WORDS: [&str; 10] = [
    "lower",
    "mandible",
    "mandibular",
    "mandibolare",
    "inferiore",
    "inferior",
    "inf",
    "unterkiefer",
    "lowerjaw",
    "arcatainferiore",
];

/// Whole-token words naming a bite registration.
const BITE_WORDS: [&str; 2] = ["bite", "morso"];

/// Letter-only tokens of a name, and its letters joined.
///
/// camelCase boundaries (`[a-z][A-Z]`) are separators, the name is lowercased,
/// then split on every character that is not an ASCII letter (digits and
/// punctuation, the extension dot included, are separators).
fn tokens(name: &str) -> (Vec<String>, String) {
    let mut spaced = String::with_capacity(name.len() + 8);
    let mut prev: Option<char> = None;
    for ch in name.chars() {
        if prev.is_some_and(|p| p.is_ascii_lowercase()) && ch.is_ascii_uppercase() {
            spaced.push(' ');
        }
        spaced.push(ch);
        prev = Some(ch);
    }
    let lower = spaced.to_lowercase();
    let toks = lower
        .split(|c: char| !c.is_ascii_lowercase())
        .filter(|t| !t.is_empty())
        .map(str::to_owned)
        .collect();
    let joined = lower.chars().filter(char::is_ascii_lowercase).collect();
    (toks, joined)
}

fn has_word(toks: &[String], words: &[&str]) -> bool {
    toks.iter().any(|t| words.contains(&t.as_str()))
}

/// Arch written in a file name (`Patient_UpperJaw.stl`, `mandibular`,
/// `arcata_superiore`...), as exported by the intraoral scanners.
///
/// Same semantics as the plugin: whole-letter tokens after a camelCase split,
/// plus the substrings `upperjaw`, `maxill`, `lowerjaw`, `mandib` in the
/// concatenated letters. A name hinting both arches, or neither, gives `None`.
/// The extension is part of the input, as in the plugin. Returns only
/// [`ScanRole::Upper`] or [`ScanRole::Lower`].
///
/// `name` is a bare file name, not a path.
pub fn arch_hint(name: &str) -> Option<ScanRole> {
    let (toks, joined) = tokens(name);
    let up =
        has_word(&toks, &UPPER_WORDS) || joined.contains("upperjaw") || joined.contains("maxill");
    let lo =
        has_word(&toks, &LOWER_WORDS) || joined.contains("lowerjaw") || joined.contains("mandib");
    match (up, lo) {
        (true, false) => Some(ScanRole::Upper),
        (false, true) => Some(ScanRole::Lower),
        _ => None,
    }
}

/// Role of a scan from its file name: the whole tokens `bite` and `morso`
/// give [`ScanRole::Bite`], otherwise [`arch_hint`].
///
/// Pass only the file name, never a full path: the directory names must not
/// count (`/x/upper_case/scan.stl` has no hint). For safety the final path
/// component (after `/` or `\`) is taken anyway.
pub fn role_from_name(name: &str) -> Option<ScanRole> {
    let file = name.rsplit(['/', '\\']).next().unwrap_or(name);
    let (toks, _) = tokens(file);
    if has_word(&toks, &BITE_WORDS) {
        return Some(ScanRole::Bite);
    }
    arch_hint(file)
}

#[cfg(test)]
mod tests {
    use super::*;
    use ScanRole::{Bite, Lower, Upper};

    #[test]
    fn arch_hint_resolves_and_rejects() {
        let cases: &[(&str, Option<ScanRole>)] = &[
            ("upper.stl", Some(Upper)),
            ("lower.stl", Some(Lower)),
            ("maxilla.stl", Some(Upper)),
            ("mandible.stl", Some(Lower)),
            ("mascellare.stl", Some(Upper)),
            ("mandibolare.stl", Some(Lower)),
            ("superiore.stl", Some(Upper)),
            ("inferiore.stl", Some(Lower)),
            ("sup.stl", Some(Upper)),
            ("inf.stl", Some(Lower)),
            ("Paziente_sup.stl", Some(Upper)),
            ("Oberkiefer.stl", Some(Upper)),
            ("Unterkiefer.stl", Some(Lower)),
            ("UpperJaw.stl", Some(Upper)),
            ("lowerJaw.stl", Some(Lower)),
            ("UPPER.STL", Some(Upper)),
            ("flower.stl", None),
            ("superman.stl", None),
            ("lowercase.stl", None),
            ("uppercase.stl", None),
            ("scan1.stl", None),
            ("scan_upper_lower.stl", None),
            ("upper-lower.stl", None),
            ("Mandible_Upper.stl", None),
            ("superioreinferiore.stl", None),
            ("", None),
        ];
        for (name, want) in cases {
            assert_eq!(arch_hint(name), *want, "{name}");
        }
    }

    /// Every row of the measured plugin table (C2, name hint only).
    #[test]
    fn arch_hint_matches_the_plugin_table() {
        let rows: &[(&str, Option<ScanRole>)] = &[
            ("upper.stl", Some(Upper)),
            ("lower.stl", Some(Lower)),
            ("maxilla.stl", Some(Upper)),
            ("mandible.stl", Some(Lower)),
            ("mascellare.stl", Some(Upper)),
            ("mandibolare.stl", Some(Lower)),
            ("superiore.stl", Some(Upper)),
            ("inferiore.stl", Some(Lower)),
            ("sup.stl", Some(Upper)),
            ("inf.stl", Some(Lower)),
            ("Paziente_sup.stl", Some(Upper)),
            ("Oberkiefer.stl", Some(Upper)),
            ("Unterkiefer.stl", Some(Lower)),
            ("UpperJaw.stl", Some(Upper)),
            ("lowerJaw.stl", Some(Lower)),
            ("scan_upper_lower.stl", None),
            ("flower.stl", None),
            ("superman.stl", None),
            ("bite.stl", None),
            ("morso.stl", None),
            ("scan1.stl", None),
            ("UPPER.STL", Some(Upper)),
            ("maxillary.stl", Some(Upper)),
            ("mandibular.stl", Some(Lower)),
            ("Arcata_Superiore.stl", Some(Upper)),
            ("arcata-inferiore.stl", Some(Lower)),
            ("UpperJaw_bite.stl", Some(Upper)),
            ("inferior.stl", Some(Lower)),
            ("superior.stl", Some(Upper)),
            ("maxill.stl", Some(Upper)),
            ("mandib.stl", Some(Lower)),
            ("Lower.ply", Some(Lower)),
            ("case_UPPER_final.obj", Some(Upper)),
            ("upperlower.stl", None),
            ("Max.stl", None),
            ("MAN.stl", None),
            ("Mandible_Upper.stl", None),
            ("p123_Maxillary_Scan.stl", Some(Upper)),
            ("bitescan.stl", None),
            ("occlusion.stl", None),
            ("12_upper.stl", Some(Upper)),
            ("UpperJawScan.stl", Some(Upper)),
            ("LowerJawScan.stl", Some(Lower)),
        ];
        for (name, want) in rows {
            assert_eq!(arch_hint(name), *want, "{name}");
        }
    }

    #[test]
    fn camel_case_and_digits_split_tokens() {
        assert_eq!(arch_hint("scanUpper.stl"), Some(Upper));
        assert_eq!(arch_hint("12_Lower3.stl"), Some(Lower));
        assert_eq!(arch_hint("Patient_UpperJaw.stl"), Some(Upper));
        assert_eq!(arch_hint("arcata_superiore"), Some(Upper));
    }

    #[test]
    fn role_from_name_adds_bite() {
        assert_eq!(role_from_name("bite.stl"), Some(Bite));
        assert_eq!(role_from_name("morso.stl"), Some(Bite));
        assert_eq!(role_from_name("Bite_01.stl"), Some(Bite));
        assert_eq!(role_from_name("UpperJaw_bite.stl"), Some(Bite));
        assert_eq!(arch_hint("UpperJaw_bite.stl"), Some(Upper));
        assert_eq!(role_from_name("bitescan.stl"), None);
        assert_eq!(role_from_name("upper.stl"), Some(Upper));
        assert_eq!(role_from_name("lower.stl"), Some(Lower));
        assert_eq!(role_from_name("scan1.stl"), None);
    }

    #[test]
    fn directories_never_count() {
        assert_eq!(role_from_name("/x/upper_case/scan.stl"), None);
        assert_eq!(role_from_name("/x/lower_dir/scan.stl"), None);
        assert_eq!(role_from_name("C:\\bite\\upper\\scan.stl"), None);
        assert_eq!(role_from_name("/x/y/upper.stl"), Some(Upper));
        assert_eq!(role_from_name("x\\bite.stl"), Some(Bite));
    }
}
