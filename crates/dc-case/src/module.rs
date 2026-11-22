//! Opaque per-module data stored in the case state.

use std::any::Any;
use std::fmt;

use crate::{BlobRef, Scan, ScanRole};

/// Data owned by one module inside a [`crate::CaseState`].
///
/// Implementors are usually plain serde structs; the trait gives the core
/// type-erased cloning, downcasting and JSON serialization. Decoding (and
/// migrating old schema versions) is the job of the module's codec.
pub trait ModuleData: Any + Send + Sync + fmt::Debug {
    /// Version of the JSON schema produced by [`ModuleData::to_json`].
    fn schema_version(&self) -> u32;

    /// Upcast for downcasting to the concrete type.
    fn as_any(&self) -> &dyn Any;

    /// Mutable upcast for downcasting to the concrete type.
    fn as_any_mut(&mut self) -> &mut dyn Any;

    /// Type-erased clone.
    fn clone_box(&self) -> Box<dyn ModuleData>;

    /// JSON form written to `modules/<id>/state.json`.
    fn to_json(&self) -> Result<serde_json::Value, serde_json::Error>;

    /// Path of the first NaN or infinite number in the data, `None` when all
    /// are finite. JSON cannot store them (they would be written as `null`),
    /// so saving refuses data that has one. [`impl_module_data!`] checks
    /// every serialized field; manual implementations should return
    /// `dc_case::find_non_finite(self)`.
    fn find_non_finite(&self) -> Option<String> {
        None
    }

    /// Approximate memory footprint (for undo budgets). Override when the
    /// data holds large buffers.
    fn approx_bytes(&self) -> usize {
        std::mem::size_of_val(self)
    }

    /// Blobs the data refers to. Saving a project drops the blobs that
    /// neither the case scans nor any module data reference, so modules
    /// that store meshes in the blob store must report them here.
    fn referenced_blobs(&self) -> Vec<BlobRef> {
        Vec::new()
    }

    /// The scan of `role` was replaced or moved to another role: drop what
    /// was computed from it and return ids of what was cleared (shown in the
    /// import report). The blobs no longer referenced afterwards are dropped
    /// by the import. Runs on the state alone, with no module registry.
    fn scan_replaced(&mut self, _role: ScanRole) -> Vec<&'static str> {
        Vec::new()
    }

    /// The scan was removed from the case (`case.remove-scan`): drop what
    /// was computed from it or entered about it and return ids of what was
    /// cleared. The blobs no longer referenced afterwards are dropped by the
    /// removal. Runs on the state alone, with no module registry. By default
    /// it is a replacement of the scan's role
    /// ([`ModuleData::scan_replaced`]).
    fn scan_removed(&mut self, scan: &Scan) -> Vec<&'static str> {
        self.scan_replaced(scan.role)
    }
}

/// Implements [`ModuleData`] for a `Clone + Debug + Serialize + Send + Sync`
/// type with a fixed schema version.
///
/// ```
/// #[derive(Debug, Clone, serde::Serialize)]
/// struct MyState { n: u32 }
/// dc_case::impl_module_data!(MyState, 1);
/// ```
#[macro_export]
macro_rules! impl_module_data {
    ($ty:ty, $schema:expr) => {
        impl $crate::ModuleData for $ty {
            fn schema_version(&self) -> u32 {
                $schema
            }
            fn as_any(&self) -> &dyn ::std::any::Any {
                self
            }
            fn as_any_mut(&mut self) -> &mut dyn ::std::any::Any {
                self
            }
            fn clone_box(&self) -> ::std::boxed::Box<dyn $crate::ModuleData> {
                ::std::boxed::Box::new(::std::clone::Clone::clone(self))
            }
            fn to_json(
                &self,
            ) -> ::std::result::Result<$crate::__serde_json::Value, $crate::__serde_json::Error>
            {
                $crate::__serde_json::to_value(self)
            }
            fn find_non_finite(&self) -> ::std::option::Option<::std::string::String> {
                $crate::find_non_finite(self)
            }
        }
    };
}

/// Data of a module that is not installed (or of a newer schema): kept as
/// JSON and written back unchanged.
#[derive(Debug, Clone, PartialEq)]
pub struct OpaqueModuleData {
    /// Schema version found in the file.
    pub schema_version: u32,
    /// Raw JSON state.
    pub value: serde_json::Value,
}

impl ModuleData for OpaqueModuleData {
    fn schema_version(&self) -> u32 {
        self.schema_version
    }

    fn as_any(&self) -> &dyn Any {
        self
    }

    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    }

    fn clone_box(&self) -> Box<dyn ModuleData> {
        Box::new(self.clone())
    }

    fn to_json(&self) -> Result<serde_json::Value, serde_json::Error> {
        Ok(self.value.clone())
    }

    /// A `serde_json::Value` cannot hold a non-finite number.
    fn find_non_finite(&self) -> Option<String> {
        None
    }

    fn approx_bytes(&self) -> usize {
        self.value.to_string().len()
    }

    /// Every string that is a blob hash: the structure is unknown, so be
    /// conservative and keep everything that could be a reference.
    fn referenced_blobs(&self) -> Vec<BlobRef> {
        fn walk(v: &serde_json::Value, out: &mut Vec<BlobRef>) {
            match v {
                serde_json::Value::String(s) if s.len() == 64 => {
                    if let Ok(r) = BlobRef::from_hex(s) {
                        out.push(r);
                    }
                }
                serde_json::Value::Array(a) => a.iter().for_each(|x| walk(x, out)),
                serde_json::Value::Object(o) => o.values().for_each(|x| walk(x, out)),
                _ => {}
            }
        }
        let mut out = Vec::new();
        walk(&self.value, &mut out);
        out
    }
}

/// Type-erased module data with value semantics (`Clone` deep-copies).
pub struct ModuleState(Box<dyn ModuleData>);

impl ModuleState {
    /// Wraps concrete module data.
    pub fn new<T: ModuleData>(data: T) -> Self {
        Self(Box::new(data))
    }

    /// Wraps already boxed module data.
    pub fn from_box(data: Box<dyn ModuleData>) -> Self {
        Self(data)
    }

    /// The type-erased data.
    pub fn data(&self) -> &dyn ModuleData {
        self.0.as_ref()
    }

    /// The type-erased data, mutable.
    pub fn data_mut(&mut self) -> &mut dyn ModuleData {
        self.0.as_mut()
    }

    /// Concrete data, when of type `T`.
    pub fn downcast_ref<T: ModuleData>(&self) -> Option<&T> {
        self.0.as_any().downcast_ref()
    }

    /// Mutable concrete data, when of type `T`.
    pub fn downcast_mut<T: ModuleData>(&mut self) -> Option<&mut T> {
        self.0.as_any_mut().downcast_mut()
    }

    /// `true` when the data is kept opaque (module unknown or too new).
    pub fn is_opaque(&self) -> bool {
        self.downcast_ref::<OpaqueModuleData>().is_some()
    }

    /// JSON form of the data.
    pub fn to_json(&self) -> Result<serde_json::Value, serde_json::Error> {
        self.0.to_json()
    }
}

impl Clone for ModuleState {
    fn clone(&self) -> Self {
        Self(self.0.clone_box())
    }
}

impl fmt::Debug for ModuleState {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_tuple("ModuleState").field(&self.0).finish()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::CaseState;

    #[derive(Debug, Clone, PartialEq, serde::Serialize)]
    struct Demo {
        stages: u32,
    }
    crate::impl_module_data!(Demo, 3);

    #[test]
    fn typed_access_clone_and_json() {
        let mut st = CaseState::default();
        st.set_module("demo", ModuleState::new(Demo { stages: 4 }));
        assert_eq!(st.module_data::<Demo>("demo"), Some(&Demo { stages: 4 }));
        assert!(st.module_data::<OpaqueModuleData>("demo").is_none());
        let mut copy = st.clone();
        copy.modules
            .get_mut("demo")
            .unwrap()
            .downcast_mut::<Demo>()
            .unwrap()
            .stages = 9;
        assert_eq!(st.module_data::<Demo>("demo").unwrap().stages, 4);
        let m = st.module("demo").unwrap();
        assert_eq!(m.data().schema_version(), 3);
        assert_eq!(m.to_json().unwrap(), serde_json::json!({"stages": 4}));
        assert!(!m.is_opaque());
    }

    #[test]
    fn opaque_data_round_trips() {
        let v = serde_json::json!({"future": [1, 2]});
        let m = ModuleState::new(OpaqueModuleData {
            schema_version: 7,
            value: v.clone(),
        });
        assert!(m.is_opaque());
        assert_eq!(m.clone().to_json().unwrap(), v);
        assert_eq!(m.data().schema_version(), 7);
    }
}
