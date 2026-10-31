//! Module codecs: turn the JSON state of a module into typed data, running
//! the module's schema migrations first.

use std::collections::BTreeMap;
use std::sync::Arc;

use dc_case::ModuleData;
use serde::de::DeserializeOwned;
use serde_json::Value;

type MigrateFn = Arc<dyn Fn(&mut Value) -> Result<(), String> + Send + Sync>;
type DecodeFn = Arc<dyn Fn(Value) -> Result<Box<dyn ModuleData>, String> + Send + Sync>;

/// Decoder and migrations of one module's `state.json`.
#[derive(Clone)]
pub struct ModuleCodec {
    current: u32,
    migrations: BTreeMap<u32, MigrateFn>,
    decode: DecodeFn,
}

impl ModuleCodec {
    /// Codec for schema version `current` that decodes with `decode`.
    pub fn new(
        current: u32,
        decode: impl Fn(Value) -> Result<Box<dyn ModuleData>, String> + Send + Sync + 'static,
    ) -> Self {
        Self {
            current,
            migrations: BTreeMap::new(),
            decode: Arc::new(decode),
        }
    }

    /// Codec that decodes the JSON into `T` with serde.
    pub fn typed<T: ModuleData + DeserializeOwned>(current: u32) -> Self {
        Self::new(current, |v| {
            serde_json::from_value::<T>(v)
                .map(|t| Box::new(t) as Box<dyn ModuleData>)
                .map_err(|e| e.to_string())
        })
    }

    /// Registers the migration from schema version `from` to `from + 1`.
    #[must_use]
    pub fn with_migration(
        mut self,
        from: u32,
        migrate: impl Fn(&mut Value) -> Result<(), String> + Send + Sync + 'static,
    ) -> Self {
        self.migrations.insert(from, Arc::new(migrate));
        self
    }

    /// Schema version this codec decodes.
    pub fn current_version(&self) -> u32 {
        self.current
    }

    /// Migrates `value` from schema `from` to the current one and decodes it.
    /// The error is the schema version at which it failed and the reason.
    pub(crate) fn decode_from(
        &self,
        mut value: Value,
        from: u32,
    ) -> Result<Box<dyn ModuleData>, (u32, String)> {
        let mut v = from;
        while v < self.current {
            let Some(migrate) = self.migrations.get(&v) else {
                return Err((v, format!("no migration from schema version {v}")));
            };
            migrate(&mut value).map_err(|e| (v, e))?;
            v += 1;
        }
        (self.decode)(value).map_err(|e| (v, e))
    }
}

impl std::fmt::Debug for ModuleCodec {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ModuleCodec")
            .field("current", &self.current)
            .field("migrations", &self.migrations.keys().collect::<Vec<_>>())
            .finish_non_exhaustive()
    }
}

/// Registry of the module codecs known to the application, by module id.
/// Modules without a codec are kept opaque.
#[derive(Debug, Clone, Default)]
pub struct ModuleCodecs {
    codecs: BTreeMap<String, ModuleCodec>,
}

impl ModuleCodecs {
    /// Empty registry: every module is kept opaque.
    pub fn new() -> Self {
        Self::default()
    }

    /// Registers (or replaces) the codec of module `id`.
    pub fn register(&mut self, id: impl Into<String>, codec: ModuleCodec) {
        self.codecs.insert(id.into(), codec);
    }

    /// The codec of module `id`.
    pub fn get(&self, id: &str) -> Option<&ModuleCodec> {
        self.codecs.get(id)
    }
}
