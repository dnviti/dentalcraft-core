//! Localization for DentalCraft: English (base) and Italian, switchable at
//! runtime.
//!
//! Every crate with user-facing text embeds its own Fluent bundle
//! (`i18n/{en,it}/<crate-name>.ftl` plus an `i18n.toml`) with [`bundle!`].
//! All bundles are registered in one process-wide registry, so
//! [`set_language`] switches them together and [`tr`] resolves runtime keys
//! (menu labels, toasts) across all of them.
//!
//! ```ignore
//! // in crate `dca-module` (depends on dc-i18n; has i18n.toml and
//! // i18n/en/dca-module.ftl, i18n/it/dca-module.ftl)
//! dc_i18n::bundle!(pub fn loader);
//!
//! let title = dc_i18n::fl!(loader(), "aligners-title"); // key checked at compile time
//! let label = dc_i18n::tr("aligners-title");             // runtime key
//! ```
//!
//! The common keys of this crate (app name, OK, Cancel, Save, ...) are
//! available through [`loader`] and [`tr`].

extern crate self as dc_i18n;

pub mod parity;

use std::collections::HashMap;
use std::fmt;
use std::sync::{LazyLock, PoisonError, RwLock};

pub use i18n_embed::I18nAssets;
use i18n_embed::LanguageLoader;
pub use i18n_embed::fluent::FluentLanguageLoader;
pub use i18n_embed_fl::fl;
pub use unic_langid::LanguageIdentifier;

#[doc(hidden)]
pub use rust_embed as __rust_embed;

/// Errors of the localization layer.
#[derive(Debug, thiserror::Error)]
pub enum I18nError {
    /// A bundle could not be loaded for the requested language.
    #[error("cannot load translations for {domain}: {source}")]
    Load {
        /// Bundle domain.
        domain: String,
        /// Cause.
        source: i18n_embed::I18nEmbedError,
    },
}

/// Supported user interface languages.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum Lang {
    /// English (base language, fallback for missing keys).
    #[default]
    En,
    /// Italian.
    It,
}

impl Lang {
    /// All languages, base language first.
    pub const ALL: [Lang; 2] = [Lang::En, Lang::It];

    /// Language code, also the folder name under `i18n/`.
    pub fn code(self) -> &'static str {
        match self {
            Lang::En => "en",
            Lang::It => "it",
        }
    }

    /// Language from its code (`en`, `it`, also `it-IT` and similar).
    pub fn from_code(code: &str) -> Option<Lang> {
        let primary = code.split(['-', '_']).next()?.to_ascii_lowercase();
        Lang::ALL.into_iter().find(|l| l.code() == primary)
    }

    /// Unicode language identifier.
    pub fn langid(self) -> LanguageIdentifier {
        self.code().parse().expect("valid language code")
    }

    /// i18n key of the language's name (`lang-en`, `lang-it`).
    pub fn name_key(self) -> &'static str {
        match self {
            Lang::En => "lang-en",
            Lang::It => "lang-it",
        }
    }

    /// First supported language among the operating system's preferences,
    /// English otherwise.
    pub fn from_system() -> Lang {
        i18n_embed::DesktopLanguageRequester::requested_languages()
            .iter()
            .find_map(|l| Lang::from_code(l.language.as_str()))
            .unwrap_or_default()
    }
}

impl fmt::Display for Lang {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.code())
    }
}

struct Bundle {
    loader: &'static FluentLanguageLoader,
    assets: &'static (dyn I18nAssets + Send + Sync),
}

struct Registry {
    lang: Lang,
    bundles: Vec<Bundle>,
}

static REGISTRY: LazyLock<RwLock<Registry>> = LazyLock::new(|| {
    RwLock::new(Registry {
        lang: Lang::En,
        bundles: Vec::new(),
    })
});

fn select(b: &Bundle, lang: Lang) -> Result<(), I18nError> {
    i18n_embed::select(b.loader, b.assets, &[lang.langid()]).map_err(|source| I18nError::Load {
        domain: b.loader.domain().to_owned(),
        source,
    })?;
    // no Unicode isolation marks around placeables: egui would show them
    b.loader.set_use_isolating(false);
    Ok(())
}

/// Empty loader for `domain` with English as fallback; register it with
/// [`register`] (or use [`bundle!`], which does both).
pub fn new_loader(domain: &str) -> FluentLanguageLoader {
    FluentLanguageLoader::new(domain, Lang::En.langid())
}

/// Adds a bundle to the registry and loads it in the current language.
/// Registering the same domain twice is a no-op.
pub fn register(
    loader: &'static FluentLanguageLoader,
    assets: &'static (dyn I18nAssets + Send + Sync),
) -> Result<(), I18nError> {
    let mut reg = REGISTRY.write().unwrap_or_else(PoisonError::into_inner);
    if reg
        .bundles
        .iter()
        .any(|b| b.loader.domain() == loader.domain())
    {
        return Ok(());
    }
    let b = Bundle { loader, assets };
    select(&b, reg.lang)?;
    reg.bundles.push(b);
    Ok(())
}

/// Switches every registered bundle to `lang`.
pub fn set_language(lang: Lang) -> Result<(), I18nError> {
    loader();
    let mut reg = REGISTRY.write().unwrap_or_else(PoisonError::into_inner);
    reg.lang = lang;
    reg.bundles.iter().try_for_each(|b| select(b, lang))
}

/// The current language.
pub fn current() -> Lang {
    REGISTRY.read().unwrap_or_else(PoisonError::into_inner).lang
}

/// Domains of the registered bundles.
pub fn domains() -> Vec<String> {
    loader();
    let reg = REGISTRY.read().unwrap_or_else(PoisonError::into_inner);
    reg.bundles
        .iter()
        .map(|b| b.loader.domain().to_owned())
        .collect()
}

/// Translates a runtime key, searching every registered bundle; returns the
/// key itself when no bundle has it.
pub fn tr(key: &str) -> String {
    tr_args(key, &[])
}

/// Translates a runtime key with Fluent arguments (`{ $name }`).
pub fn tr_args(key: &str, args: &[(&str, &str)]) -> String {
    loader();
    let reg = REGISTRY.read().unwrap_or_else(PoisonError::into_inner);
    match reg.bundles.iter().find(|b| b.loader.has(key)) {
        Some(b) if args.is_empty() => b.loader.get(key),
        Some(b) => {
            let map: HashMap<&str, &str> = args.iter().copied().collect();
            b.loader.get_args(key, map)
        }
        None => key.to_owned(),
    }
}

/// `true` when some registered bundle has `key`.
pub fn has(key: &str) -> bool {
    loader();
    let reg = REGISTRY.read().unwrap_or_else(PoisonError::into_inner);
    reg.bundles.iter().any(|b| b.loader.has(key))
}

/// Declares a function returning the crate's Fluent loader, embedding
/// `i18n/<lang>/<domain>.ftl` from the calling crate and registering it on
/// first use. The domain defaults to the package name (`CARGO_PKG_NAME`),
/// which is also what [`fl!`] expects.
///
/// The calling crate needs only `dc-i18n` as dependency (plus an
/// `i18n.toml` when it uses [`fl!`]).
#[macro_export]
macro_rules! bundle {
    ($vis:vis fn $name:ident) => {
        $crate::bundle!($vis fn $name, env!("CARGO_PKG_NAME"));
    };
    ($vis:vis fn $name:ident, $domain:expr) => {
        /// Fluent loader of this crate, registered with `dc_i18n` on first use.
        $vis fn $name() -> &'static $crate::FluentLanguageLoader {
            #[derive($crate::__rust_embed::RustEmbed)]
            #[folder = "i18n/"]
            #[crate_path = "dc_i18n::__rust_embed"]
            struct Assets;

            static LOADER: ::std::sync::LazyLock<$crate::FluentLanguageLoader> =
                ::std::sync::LazyLock::new(|| $crate::new_loader($domain));
            static REGISTER: ::std::sync::Once = ::std::sync::Once::new();
            REGISTER.call_once(|| {
                if let Err(e) = $crate::register(&LOADER, &Assets) {
                    panic!("{e}");
                }
            });
            &LOADER
        }
    };
}

bundle!(pub fn loader);

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn lang_codes() {
        assert_eq!(Lang::from_code("it-IT"), Some(Lang::It));
        assert_eq!(Lang::from_code("en_US"), Some(Lang::En));
        assert_eq!(Lang::from_code("de"), None);
        assert_eq!(Lang::It.langid().to_string(), "it");
        let _ = Lang::from_system();
    }

    // one test: the language is process-wide state
    #[test]
    fn switch_language_and_lookup() {
        set_language(Lang::En).unwrap();
        assert_eq!(current(), Lang::En);
        assert_eq!(fl!(loader(), "cancel"), "Cancel");
        assert_eq!(tr("save"), "Save");
        assert_eq!(tr_args("undo-action", &[("action", "Move")]), "Undo Move");
        assert_eq!(tr("no-such-key"), "no-such-key");
        assert!(has("app-name") && !has("no-such-key"));
        assert!(domains().contains(&"dc-i18n".to_owned()));

        set_language(Lang::It).unwrap();
        assert_eq!(current(), Lang::It);
        assert_eq!(fl!(loader(), "cancel"), "Annulla");
        assert_eq!(tr(Lang::It.name_key()), "Italiano");
        assert_eq!(
            fl!(loader(), "redo-action", action = "Sposta"),
            "Ripeti Sposta"
        );

        set_language(Lang::En).unwrap();
        assert_eq!(tr("open"), "Open");
    }

    #[test]
    fn en_and_it_have_the_same_keys() {
        let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("i18n");
        parity::check_dir(&dir).unwrap();
    }
}
