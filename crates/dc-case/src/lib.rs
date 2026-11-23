//! Case model of DentalCraft.
//!
//! A [`Case`] holds the patient scans ([`Scan`]) and the dental world frame
//! ([`WorldFrame`]). The full, immutable state of an open case is a
//! [`CaseState`]: the case itself, the meshes it references
//! ([`BlobStore`], content addressed by [`BlobRef`]) and the opaque data of
//! every module ([`ModuleState`]). States are shared as `Arc<CaseState>` and
//! replaced, never mutated in place, so undo is a matter of keeping old
//! versions around.
//!
//! Teeth are identified by their FDI number ([`Fdi`]) and positioned with
//! rigid [`Pose`]s (millimetres, double precision).
//!
//! Scan files enter a case through one pipeline, [`import`].

mod blob;
mod case;
mod fdi;
mod finite;
pub mod import;
mod module;
mod names;
mod pose;

pub use blob::{BlobRef, BlobStore};
pub use case::{Case, CaseState, Jaws, MAX_BITE_SCANS, RoleSource, Scan, ScanRole, WorldFrame};
pub use fdi::Fdi;
pub use finite::find_non_finite;
pub use module::{ModuleData, ModuleState, OpaqueModuleData};
pub use names::{arch_hint, role_from_name};
pub use pose::Pose;

dc_i18n::bundle!(pub fn loader);

/// Errors of the case model.
#[derive(Debug, thiserror::Error)]
pub enum CaseError {
    /// Not a valid FDI tooth number.
    #[error("invalid FDI tooth number: {0}")]
    InvalidFdi(u8),
    /// Not a valid 64-digit hexadecimal BLAKE3 hash.
    #[error("invalid blob reference: {0:?}")]
    InvalidBlobRef(String),
    /// A referenced blob is not in the store.
    #[error("missing blob {0}")]
    MissingBlob(BlobRef),
    /// A rotation quaternion of zero length or with non-finite components.
    #[error("invalid rotation {0:?}: not a non-zero finite quaternion")]
    InvalidRotation([f64; 4]),
    /// Module data could not be (de)serialized.
    #[error(transparent)]
    Json(#[from] serde_json::Error),
}

/// Result alias for the case model.
pub type Result<T, E = CaseError> = std::result::Result<T, E>;

#[doc(hidden)]
pub use serde_json as __serde_json;

#[cfg(test)]
mod tests {
    #[test]
    fn translations_are_complete() {
        let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("i18n");
        dc_i18n::parity::check_dir(&dir).unwrap();
    }
}
