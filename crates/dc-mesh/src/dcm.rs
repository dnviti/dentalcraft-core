//! `DCM1`: canonical binary encoding of a [`TriMesh`] (all little endian).
//!
//! ```text
//! magic        b"DCM1"
//! version      u32 = 1
//! nv, nf       u32, u32
//! attr_count   u32
//! attr table   attr_count × u32 kind (strictly increasing, see [`Attr`])
//! positions    nv × f32×3
//! indices      nf × u32×3
//! attributes   in table order: normals nv × f32×3, vertex labels nv × u16,
//!              face labels nf × u16
//! ```
//!
//! The encoding is deterministic, so the BLAKE3 hash of these bytes is the
//! content address of a mesh blob (`blobs/<blake3>.dcm` in `.dcprj`).

use glam::Vec3;

use crate::{MeshError, Result, TriMesh};

/// File magic.
pub const MAGIC: &[u8; 4] = b"DCM1";
/// Encoding version.
pub const VERSION: u32 = 1;

/// Optional attribute kinds of the attribute table.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
#[repr(u32)]
pub enum Attr {
    /// Per-vertex normals, f32×3.
    Normals = 1,
    /// Per-vertex labels, u16.
    VertexLabels = 2,
    /// Per-face labels, u16.
    FaceLabels = 3,
}

/// Encodes `mesh` in the canonical `DCM1` layout.
pub fn encode(mesh: &TriMesh) -> Vec<u8> {
    let mut attrs = Vec::new();
    if mesh.normals.is_some() {
        attrs.push(Attr::Normals);
    }
    if mesh.vertex_labels.is_some() {
        attrs.push(Attr::VertexLabels);
    }
    if mesh.face_labels.is_some() {
        attrs.push(Attr::FaceLabels);
    }
    let mut out = Vec::with_capacity(20 + 4 * attrs.len() + mesh.approx_bytes());
    out.extend_from_slice(MAGIC);
    for v in [
        VERSION,
        mesh.positions.len() as u32,
        mesh.tris.len() as u32,
        attrs.len() as u32,
    ] {
        out.extend_from_slice(&v.to_le_bytes());
    }
    for a in &attrs {
        out.extend_from_slice(&(*a as u32).to_le_bytes());
    }
    let put_vec3 = |out: &mut Vec<u8>, v: &[Vec3]| {
        for p in v {
            for c in p.to_array() {
                out.extend_from_slice(&c.to_le_bytes());
            }
        }
    };
    put_vec3(&mut out, &mesh.positions);
    for t in &mesh.tris {
        for i in t {
            out.extend_from_slice(&i.to_le_bytes());
        }
    }
    if let Some(n) = &mesh.normals {
        put_vec3(&mut out, n);
    }
    for labels in [&mesh.vertex_labels, &mesh.face_labels]
        .into_iter()
        .flatten()
    {
        for l in labels {
            out.extend_from_slice(&l.to_le_bytes());
        }
    }
    out
}

struct Reader<'a> {
    buf: &'a [u8],
}

impl Reader<'_> {
    fn take(&mut self, n: usize) -> Result<&[u8]> {
        if self.buf.len() < n {
            return Err(bad("truncated data"));
        }
        let (head, tail) = self.buf.split_at(n);
        self.buf = tail;
        Ok(head)
    }

    fn u32(&mut self) -> Result<u32> {
        let b = self.take(4)?;
        Ok(u32::from_le_bytes([b[0], b[1], b[2], b[3]]))
    }

    fn vec3s(&mut self, n: usize) -> Result<Vec<Vec3>> {
        let raw = self.take(n.checked_mul(12).ok_or_else(|| bad("size overflow"))?)?;
        let floats: Vec<f32> = raw
            .as_chunks::<4>()
            .0
            .iter()
            .map(|w| f32::from_le_bytes(*w))
            .collect();
        Ok(floats
            .as_chunks::<3>()
            .0
            .iter()
            .map(|c| Vec3::from_array(*c))
            .collect())
    }

    fn u16s(&mut self, n: usize) -> Result<Vec<u16>> {
        let raw = self.take(n.checked_mul(2).ok_or_else(|| bad("size overflow"))?)?;
        Ok(raw
            .as_chunks::<2>()
            .0
            .iter()
            .map(|c| u16::from_le_bytes(*c))
            .collect())
    }
}

fn bad(reason: &str) -> MeshError {
    MeshError::Format {
        format: "DCM1",
        reason: reason.to_owned(),
    }
}

/// Decodes a `DCM1` buffer and checks index ranges.
pub fn decode(bytes: &[u8]) -> Result<TriMesh> {
    let mut r = Reader { buf: bytes };
    if r.take(4)? != MAGIC {
        return Err(bad("bad magic"));
    }
    let version = r.u32()?;
    if version != VERSION {
        return Err(bad(&format!("unsupported version {version}")));
    }
    let nv = r.u32()? as usize;
    let nf = r.u32()? as usize;
    let na = r.u32()? as usize;
    if na > 3 {
        return Err(bad("too many attributes"));
    }
    let mut kinds = Vec::with_capacity(na);
    for _ in 0..na {
        kinds.push(r.u32()?);
    }
    if kinds.windows(2).any(|w| w[0] >= w[1]) {
        return Err(bad("attribute table not strictly increasing"));
    }
    let positions = r.vec3s(nv)?;
    let raw = r.take(nf.checked_mul(12).ok_or_else(|| bad("size overflow"))?)?;
    let words: Vec<u32> = raw
        .as_chunks::<4>()
        .0
        .iter()
        .map(|w| u32::from_le_bytes(*w))
        .collect();
    let tris = words.as_chunks::<3>().0.to_vec();
    let mut mesh = TriMesh::new(positions, tris);
    for k in kinds {
        match k {
            1 => mesh.normals = Some(r.vec3s(nv)?),
            2 => mesh.vertex_labels = Some(r.u16s(nv)?),
            3 => mesh.face_labels = Some(r.u16s(nf)?),
            _ => return Err(bad(&format!("unknown attribute kind {k}"))),
        }
    }
    if !r.buf.is_empty() {
        return Err(bad("trailing bytes"));
    }
    mesh.check()?;
    Ok(mesh)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_meshes::*;

    #[test]
    fn round_trip_with_attributes() {
        let mut m = tetra();
        m.update_normals();
        m.vertex_labels = Some(vec![11, 12, 0, 48]);
        m.face_labels = Some(vec![1, 2, 3, 4]);
        let bytes = encode(&m);
        assert_eq!(&bytes[..4], MAGIC);
        assert_eq!(decode(&bytes).unwrap(), m);
        assert_eq!(encode(&decode(&bytes).unwrap()), bytes);
    }

    #[test]
    fn rejects_corrupt_data() {
        let bytes = encode(&quad());
        assert!(decode(&bytes[..bytes.len() - 1]).is_err());
        let mut b = bytes.clone();
        b[0] = b'X';
        assert!(decode(&b).is_err());
        let mut b = bytes;
        b.push(0);
        assert!(decode(&b).is_err());
    }
}
