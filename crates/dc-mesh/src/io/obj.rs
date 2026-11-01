//! Wavefront OBJ reader and writer (geometry and vertex colours only).

use std::io::Write;

use glam::Vec3;

use super::bad;
use crate::{Result, TriMesh};

const FMT: &str = "OBJ";

fn num(tok: Option<&str>, ln: usize) -> Result<f32> {
    tok.and_then(|t| t.parse::<f32>().ok())
        .filter(|v| v.is_finite())
        .ok_or_else(|| bad(FMT, format!("line {ln}: bad number")))
}

fn to_u8(v: f32) -> u8 {
    (v.clamp(0.0, 1.0) * 255.0).round() as u8
}

/// Resolves a 1-based (or negative, relative) OBJ index to a 0-based one.
fn index(tok: &str, nv: usize, ln: usize) -> Result<u32> {
    let first = tok.split('/').next().unwrap_or("");
    let i: i64 = first
        .parse()
        .map_err(|_| bad(FMT, format!("line {ln}: bad face index `{tok}`")))?;
    let abs = if i > 0 { i - 1 } else { nv as i64 + i };
    if i == 0 || abs < 0 || abs >= nv as i64 || abs > i64::from(u32::MAX) {
        return Err(bad(
            FMT,
            format!("line {ln}: face index {i} out of range (vertices: {nv})"),
        ));
    }
    Ok(abs as u32)
}

pub(super) fn read(bytes: &[u8]) -> Result<TriMesh> {
    read_counted(bytes).map(|(mesh, _)| mesh)
}

/// Reads an OBJ and also returns the number of objects that own faces.
///
/// An object starts at each `o` line; faces before the first `o` belong to an
/// implicit object. Only objects with at least one `f` line are counted, so an
/// OBJ with faces and no `o` line gives 1 and an OBJ without faces gives 0.
/// `g` groups are not objects.
pub(super) fn read_counted(bytes: &[u8]) -> Result<(TriMesh, usize)> {
    let text = String::from_utf8_lossy(bytes);
    let mut positions: Vec<Vec3> = Vec::new();
    let mut colors: Vec<Option<[u8; 4]>> = Vec::new();
    let mut tris: Vec<[u32; 3]> = Vec::new();
    let mut poly: Vec<u32> = Vec::new();
    let mut objects_with_faces = 0usize;
    let mut current_has_faces = false;
    for (i, raw) in text.lines().enumerate() {
        let ln = i + 1;
        let line = raw.split('#').next().unwrap_or("");
        let mut tok = line.split_ascii_whitespace();
        match tok.next() {
            Some("v") => {
                let p = Vec3::new(
                    num(tok.next(), ln)?,
                    num(tok.next(), ln)?,
                    num(tok.next(), ln)?,
                );
                positions.push(p);
                // `v x y z w` has a weight, `v x y z r g b` colours
                let rest: Vec<&str> = tok.collect();
                colors.push(if rest.len() >= 3 {
                    Some([
                        to_u8(num(Some(rest[0]), ln)?),
                        to_u8(num(Some(rest[1]), ln)?),
                        to_u8(num(Some(rest[2]), ln)?),
                        255,
                    ])
                } else {
                    None
                });
            }
            Some("o") => current_has_faces = false,
            Some("f") => {
                if !current_has_faces {
                    current_has_faces = true;
                    objects_with_faces += 1;
                }
                poly.clear();
                for t in tok {
                    poly.push(index(t, positions.len(), ln)?);
                }
                if poly.len() < 3 {
                    return Err(bad(
                        FMT,
                        format!("line {ln}: face with fewer than 3 vertices"),
                    ));
                }
                for k in 1..poly.len() - 1 {
                    tris.push([poly[0], poly[k], poly[k + 1]]);
                }
            }
            _ => {}
        }
    }
    let mut mesh = TriMesh::new(positions, tris);
    if colors.iter().any(Option::is_some) {
        mesh.vertex_colors = Some(colors.into_iter().map(|c| c.unwrap_or([255; 4])).collect());
    }
    Ok((mesh, objects_with_faces))
}

pub(super) fn write(w: &mut impl Write, mesh: &TriMesh) -> Result<()> {
    writeln!(w, "# dc-mesh OBJ")?;
    for (i, p) in mesh.positions.iter().enumerate() {
        match &mesh.vertex_colors {
            Some(c) => {
                let [r, g, b, _] = c[i];
                let f = |v: u8| f32::from(v) / 255.0;
                writeln!(w, "v {} {} {} {} {} {}", p.x, p.y, p.z, f(r), f(g), f(b))?;
            }
            None => writeln!(w, "v {} {} {}", p.x, p.y, p.z)?,
        }
    }
    for t in &mesh.tris {
        writeln!(w, "f {} {} {}", t[0] + 1, t[1] + 1, t[2] + 1)?;
    }
    Ok(())
}
