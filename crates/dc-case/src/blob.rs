//! Content-addressed mesh blobs.

use std::collections::{HashMap, HashSet};
use std::fmt;
use std::str::FromStr;
use std::sync::Arc;

use serde::{Deserialize, Deserializer, Serialize, Serializer};

use dc_mesh::{TriMesh, dcm};

use crate::CaseError;

/// BLAKE3 hash of a blob's canonical bytes (for meshes: the `DCM1`
/// encoding). Serialized as a 64-digit lowercase hex string.
#[derive(Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct BlobRef(pub [u8; 32]);

impl BlobRef {
    /// Hash of arbitrary bytes.
    pub fn of_bytes(bytes: &[u8]) -> Self {
        Self(*blake3::hash(bytes).as_bytes())
    }

    /// Hash of a mesh (its canonical `DCM1` encoding).
    pub fn of_mesh(mesh: &TriMesh) -> Self {
        Self::of_bytes(&dcm::encode(mesh))
    }

    /// 64-digit lowercase hex form.
    pub fn to_hex(&self) -> String {
        blake3::Hash::from_bytes(self.0).to_hex().to_string()
    }

    /// Parses the 64-digit hex form (either case).
    pub fn from_hex(s: &str) -> Result<Self, CaseError> {
        blake3::Hash::from_hex(s)
            .map(|h| Self(*h.as_bytes()))
            .map_err(|_| CaseError::InvalidBlobRef(s.to_owned()))
    }
}

impl fmt::Display for BlobRef {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.to_hex())
    }
}

impl fmt::Debug for BlobRef {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "BlobRef({})", &self.to_hex()[..12])
    }
}

impl FromStr for BlobRef {
    type Err = CaseError;

    fn from_str(s: &str) -> Result<Self, CaseError> {
        Self::from_hex(s)
    }
}

impl Serialize for BlobRef {
    fn serialize<S: Serializer>(&self, s: S) -> Result<S::Ok, S::Error> {
        s.serialize_str(&self.to_hex())
    }
}

impl<'de> Deserialize<'de> for BlobRef {
    fn deserialize<D: Deserializer<'de>>(d: D) -> Result<Self, D::Error> {
        let s = String::deserialize(d)?;
        Self::from_hex(&s).map_err(serde::de::Error::custom)
    }
}

/// In-memory store of immutable meshes keyed by their [`BlobRef`].
///
/// Cloning is cheap (the meshes are shared through `Arc`).
#[derive(Clone, Default)]
pub struct BlobStore {
    meshes: HashMap<BlobRef, Arc<TriMesh>>,
}

impl BlobStore {
    /// Empty store.
    pub fn new() -> Self {
        Self::default()
    }

    /// Adds a mesh and returns its reference (no-op when already present).
    pub fn insert(&mut self, mesh: TriMesh) -> BlobRef {
        self.insert_arc(Arc::new(mesh))
    }

    /// Adds a shared mesh and returns its reference.
    pub fn insert_arc(&mut self, mesh: Arc<TriMesh>) -> BlobRef {
        let r = BlobRef::of_mesh(&mesh);
        self.meshes.entry(r).or_insert(mesh);
        r
    }

    /// Adds a mesh whose reference is already known (e.g. read from a
    /// project file after verifying its hash).
    pub fn insert_verified(&mut self, r: BlobRef, mesh: Arc<TriMesh>) {
        self.meshes.insert(r, mesh);
    }

    /// The mesh with reference `r`.
    pub fn get(&self, r: &BlobRef) -> Option<&Arc<TriMesh>> {
        self.meshes.get(r)
    }

    /// The mesh with reference `r`, or [`CaseError::MissingBlob`].
    pub fn require(&self, r: &BlobRef) -> Result<&Arc<TriMesh>, CaseError> {
        self.get(r).ok_or(CaseError::MissingBlob(*r))
    }

    /// `true` when `r` is in the store.
    pub fn contains(&self, r: &BlobRef) -> bool {
        self.meshes.contains_key(r)
    }

    /// Number of blobs.
    pub fn len(&self) -> usize {
        self.meshes.len()
    }

    /// `true` when the store is empty.
    pub fn is_empty(&self) -> bool {
        self.meshes.is_empty()
    }

    /// All blobs, in no particular order.
    pub fn iter(&self) -> impl Iterator<Item = (&BlobRef, &Arc<TriMesh>)> {
        self.meshes.iter()
    }

    /// Drops every blob not in `keep` (garbage collection before saving).
    pub fn retain_only(&mut self, keep: &HashSet<BlobRef>) {
        self.meshes.retain(|r, _| keep.contains(r));
    }

    /// Approximate memory used by the meshes.
    pub fn approx_bytes(&self) -> usize {
        self.meshes.values().map(|m| m.approx_bytes()).sum()
    }
}

impl fmt::Debug for BlobStore {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut keys: Vec<_> = self.meshes.keys().collect();
        keys.sort();
        f.debug_struct("BlobStore").field("blobs", &keys).finish()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::Vec3;

    fn tri() -> TriMesh {
        TriMesh::new(vec![Vec3::ZERO, Vec3::X, Vec3::Y], vec![[0, 1, 2]])
    }

    #[test]
    fn hex_round_trip_and_serde() {
        let r = BlobRef::of_bytes(b"hello");
        let hex = r.to_hex();
        assert_eq!(hex.len(), 64);
        assert_eq!(BlobRef::from_hex(&hex).unwrap(), r);
        assert_eq!(hex.parse::<BlobRef>().unwrap(), r);
        assert!(BlobRef::from_hex("zz").is_err());
        let json = serde_json::to_string(&r).unwrap();
        assert_eq!(json, format!("\"{hex}\""));
        assert_eq!(serde_json::from_str::<BlobRef>(&json).unwrap(), r);
    }

    #[test]
    fn store_dedups_by_content() {
        let mut s = BlobStore::new();
        let a = s.insert(tri());
        let b = s.insert(tri());
        assert_eq!(a, b);
        assert_eq!(s.len(), 1);
        let mut other = tri();
        other.positions[0].z = 1.0;
        let c = s.insert(other);
        assert_ne!(a, c);
        assert_eq!(s.len(), 2);
        s.retain_only(&HashSet::from([c]));
        assert!(!s.contains(&a) && s.contains(&c));
        assert!(s.require(&a).is_err());
    }
}
