//! STL reader (binary and ASCII) and writers.

use std::io::Write;

use glam::Vec3;

use super::{ReadOptions, bad, weld};
use crate::{Result, TriMesh};

const FMT: &str = "STL";
const HEADER: usize = 80;
const FACET: usize = 50;

/// `true` when `bytes` is a complete binary STL image.
fn is_binary(bytes: &[u8]) -> bool {
    if bytes.len() < HEADER + 4 {
        return false;
    }
    let n = u64::from(u32::from_le_bytes(
        bytes[HEADER..HEADER + 4].try_into().unwrap_or([0; 4]),
    ));
    HEADER as u64 + 4 + n * FACET as u64 == bytes.len() as u64
}

fn starts_with_solid(bytes: &[u8]) -> bool {
    let head = &bytes[..bytes.len().min(512)];
    let trimmed = head.trim_ascii_start();
    trimmed.len() >= 5 && trimmed[..5].eq_ignore_ascii_case(b"solid")
}

pub(super) fn read(bytes: &[u8], options: &ReadOptions) -> Result<TriMesh> {
    // Content decides: an exact binary size match wins even when the header
    // starts with "solid" (common with CAD exporters).
    let soup = if is_binary(bytes) {
        read_binary(bytes)?
    } else if starts_with_solid(bytes) {
        read_ascii(bytes)?
    } else if bytes.len() >= HEADER + 4 {
        // binary with trailing bytes is tolerated, truncation is not
        read_binary(bytes)?
    } else {
        return Err(bad(FMT, "file too short to be an STL"));
    };
    Ok(weld::weld_soup(&soup, options.weld_tolerance))
}

fn read_binary(bytes: &[u8]) -> Result<Vec<Vec3>> {
    let n = u64::from(u32::from_le_bytes(
        bytes[HEADER..HEADER + 4].try_into().unwrap_or([0; 4]),
    ));
    let body = &bytes[HEADER + 4..];
    if n * FACET as u64 > body.len() as u64 {
        return Err(bad(
            FMT,
            format!(
                "truncated binary STL: header announces {n} triangles, data holds {}",
                body.len() / FACET
            ),
        ));
    }
    let f = |c: &[u8]| f32::from_le_bytes([c[0], c[1], c[2], c[3]]);
    let mut soup = Vec::with_capacity(n as usize * 3);
    let (records, _) = body.as_chunks::<FACET>();
    for rec in records.iter().take(n as usize) {
        for v in 0..3 {
            let o = 12 + v * 12;
            let p = Vec3::new(f(&rec[o..]), f(&rec[o + 4..]), f(&rec[o + 8..]));
            if !p.is_finite() {
                return Err(bad(FMT, "non-finite vertex coordinate"));
            }
            soup.push(p);
        }
    }
    Ok(soup)
}

fn read_ascii(bytes: &[u8]) -> Result<Vec<Vec3>> {
    let text = String::from_utf8_lossy(bytes);
    let mut soup = Vec::new();
    let mut closed = false;
    for (ln, line) in text.lines().enumerate() {
        let mut tok = line.split_ascii_whitespace();
        let Some(kw) = tok.next() else { continue };
        match kw.to_ascii_lowercase().as_str() {
            "vertex" => {
                let mut c = [0f32; 3];
                for slot in &mut c {
                    *slot = tok
                        .next()
                        .and_then(|t| t.parse::<f32>().ok())
                        .filter(|v| v.is_finite())
                        .ok_or_else(|| bad(FMT, format!("line {}: bad vertex", ln + 1)))?;
                }
                soup.push(Vec3::from(c));
            }
            "endsolid" => closed = true,
            "solid" | "facet" | "outer" | "endloop" | "endfacet" => {}
            other => {
                return Err(bad(
                    FMT,
                    format!("line {}: unexpected keyword `{other}`", ln + 1),
                ));
            }
        }
    }
    if soup.len() % 3 != 0 {
        return Err(bad(FMT, "vertex count is not a multiple of 3"));
    }
    if soup.is_empty() && !closed {
        return Err(bad(FMT, "no facets found"));
    }
    Ok(soup)
}

pub(super) fn write_binary(w: &mut impl Write, mesh: &TriMesh) -> Result<()> {
    let mut header = [b' '; HEADER];
    let tag = b"DentalCraft dc-mesh binary STL";
    header[..tag.len()].copy_from_slice(tag);
    w.write_all(&header)?;
    w.write_all(&(mesh.tris.len() as u32).to_le_bytes())?;
    let mut rec = [0u8; FACET];
    for f in 0..mesh.tris.len() {
        let n = mesh.face_normal(f);
        let [a, b, c] = mesh.triangle(f);
        for (i, v) in [n, a, b, c].iter().enumerate() {
            for (j, x) in v.to_array().iter().enumerate() {
                let o = (i * 3 + j) * 4;
                rec[o..o + 4].copy_from_slice(&x.to_le_bytes());
            }
        }
        w.write_all(&rec)?;
    }
    Ok(())
}

pub(super) fn write_ascii(w: &mut impl Write, mesh: &TriMesh) -> Result<()> {
    writeln!(w, "solid dc-mesh")?;
    for f in 0..mesh.tris.len() {
        let n = mesh.face_normal(f);
        writeln!(w, "  facet normal {:e} {:e} {:e}", n.x, n.y, n.z)?;
        writeln!(w, "    outer loop")?;
        for v in mesh.triangle(f) {
            writeln!(w, "      vertex {:e} {:e} {:e}", v.x, v.y, v.z)?;
        }
        writeln!(w, "    endloop")?;
        writeln!(w, "  endfacet")?;
    }
    writeln!(w, "endsolid dc-mesh")?;
    Ok(())
}
