//! Stanford PLY reader (ASCII, binary little/big endian) and binary LE writer.

use std::io::Write;

use glam::Vec3;

use super::bad;
use crate::{Result, TriMesh};

const FMT: &str = "PLY";

#[derive(Clone, Copy, PartialEq, Eq)]
enum Ty {
    I8,
    U8,
    I16,
    U16,
    I32,
    U32,
    F32,
    F64,
}

impl Ty {
    fn parse(s: &str) -> Option<Self> {
        Some(match s {
            "char" | "int8" => Self::I8,
            "uchar" | "uint8" => Self::U8,
            "short" | "int16" => Self::I16,
            "ushort" | "uint16" => Self::U16,
            "int" | "int32" => Self::I32,
            "uint" | "uint32" => Self::U32,
            "float" | "float32" => Self::F32,
            "double" | "float64" => Self::F64,
            _ => return None,
        })
    }

    fn size(self) -> usize {
        match self {
            Self::I8 | Self::U8 => 1,
            Self::I16 | Self::U16 => 2,
            Self::I32 | Self::U32 | Self::F32 => 4,
            Self::F64 => 8,
        }
    }

    fn is_float(self) -> bool {
        matches!(self, Self::F32 | Self::F64)
    }
}

#[derive(Clone, Copy)]
enum Kind {
    Scalar(Ty),
    List { count: Ty, item: Ty },
}

struct Prop {
    name: String,
    kind: Kind,
}

struct Element {
    name: String,
    count: usize,
    props: Vec<Prop>,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Encoding {
    Ascii,
    Le,
    Be,
}

/// Value source: whitespace separated tokens or a binary cursor.
enum Src<'a> {
    Ascii(std::str::SplitAsciiWhitespace<'a>),
    Bin {
        data: &'a [u8],
        pos: usize,
        be: bool,
    },
}

impl Src<'_> {
    fn scalar(&mut self, ty: Ty) -> Result<f64> {
        match self {
            Src::Ascii(it) => {
                let t = it
                    .next()
                    .ok_or_else(|| bad(FMT, "unexpected end of data"))?;
                t.parse::<f64>()
                    .map_err(|_| bad(FMT, format!("bad number `{t}`")))
            }
            Src::Bin { data, pos, be } => {
                let n = ty.size();
                let end = pos.checked_add(n).filter(|&e| e <= data.len());
                let end = end.ok_or_else(|| bad(FMT, "unexpected end of data"))?;
                let mut b = [0u8; 8];
                b[..n].copy_from_slice(&data[*pos..end]);
                *pos = end;
                let s = &mut b[..n];
                if *be {
                    s.reverse();
                }
                Ok(match ty {
                    Ty::I8 => f64::from(s[0] as i8),
                    Ty::U8 => f64::from(s[0]),
                    Ty::I16 => f64::from(i16::from_le_bytes([s[0], s[1]])),
                    Ty::U16 => f64::from(u16::from_le_bytes([s[0], s[1]])),
                    Ty::I32 => f64::from(i32::from_le_bytes([s[0], s[1], s[2], s[3]])),
                    Ty::U32 => f64::from(u32::from_le_bytes([s[0], s[1], s[2], s[3]])),
                    Ty::F32 => f64::from(f32::from_le_bytes([s[0], s[1], s[2], s[3]])),
                    Ty::F64 => f64::from_le_bytes(b),
                })
            }
        }
    }
}

struct Header {
    encoding: Encoding,
    elements: Vec<Element>,
    body_start: usize,
}

fn parse_header(bytes: &[u8]) -> Result<Header> {
    let mut pos = 0usize;
    let mut next_line = || -> Option<&str> {
        if pos >= bytes.len() {
            return None;
        }
        let end = bytes[pos..]
            .iter()
            .position(|&b| b == b'\n')
            .map_or(bytes.len(), |e| pos + e);
        let line = &bytes[pos..end];
        pos = end + 1;
        std::str::from_utf8(line).ok().map(str::trim)
    };
    if next_line() != Some("ply") {
        return Err(bad(FMT, "missing `ply` magic"));
    }
    let mut encoding = None;
    let mut elements: Vec<Element> = Vec::new();
    loop {
        let line = next_line().ok_or_else(|| bad(FMT, "header not terminated by end_header"))?;
        let mut tok = line.split_ascii_whitespace();
        match tok.next() {
            Some("format") => {
                encoding = Some(match tok.next() {
                    Some("ascii") => Encoding::Ascii,
                    Some("binary_little_endian") => Encoding::Le,
                    Some("binary_big_endian") => Encoding::Be,
                    other => return Err(bad(FMT, format!("unknown format {other:?}"))),
                });
            }
            Some("element") => {
                let name = tok.next().ok_or_else(|| bad(FMT, "element without name"))?;
                let count = tok
                    .next()
                    .and_then(|t| t.parse::<usize>().ok())
                    .ok_or_else(|| bad(FMT, "element without valid count"))?;
                elements.push(Element {
                    name: name.to_string(),
                    count,
                    props: Vec::new(),
                });
            }
            Some("property") => {
                let el = elements
                    .last_mut()
                    .ok_or_else(|| bad(FMT, "property before element"))?;
                let t1 = tok.next().ok_or_else(|| bad(FMT, "incomplete property"))?;
                let ty = |s: &str| {
                    Ty::parse(s).ok_or_else(|| bad(FMT, format!("unknown property type `{s}`")))
                };
                let kind = if t1 == "list" {
                    let count = ty(tok.next().unwrap_or(""))?;
                    let item = ty(tok.next().unwrap_or(""))?;
                    Kind::List { count, item }
                } else {
                    Kind::Scalar(ty(t1)?)
                };
                let name = tok
                    .next()
                    .ok_or_else(|| bad(FMT, "property without name"))?;
                el.props.push(Prop {
                    name: name.to_string(),
                    kind,
                });
            }
            Some("end_header") => break,
            Some(_) | None => {} // comment, obj_info, blank
        }
    }
    Ok(Header {
        encoding: encoding.ok_or_else(|| bad(FMT, "missing format line"))?,
        elements,
        body_start: pos.min(bytes.len()),
    })
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Slot {
    None,
    X,
    Y,
    Z,
    Nx,
    Ny,
    Nz,
    R,
    G,
    B,
    A,
}

fn slot_of(name: &str) -> Slot {
    match name {
        "x" => Slot::X,
        "y" => Slot::Y,
        "z" => Slot::Z,
        "nx" => Slot::Nx,
        "ny" => Slot::Ny,
        "nz" => Slot::Nz,
        "red" | "r" | "diffuse_red" => Slot::R,
        "green" | "g" | "diffuse_green" => Slot::G,
        "blue" | "b" | "diffuse_blue" => Slot::B,
        "alpha" | "a" | "diffuse_alpha" => Slot::A,
        _ => Slot::None,
    }
}

fn skip_prop(src: &mut Src, kind: Kind) -> Result<()> {
    match kind {
        Kind::Scalar(t) => {
            src.scalar(t)?;
        }
        Kind::List { count, item } => {
            let n = src.scalar(count)?;
            if n < 0.0 || n.fract() != 0.0 {
                return Err(bad(FMT, "invalid list length"));
            }
            for _ in 0..n as u64 {
                src.scalar(item)?;
            }
        }
    }
    Ok(())
}

fn color_u8(v: f64, float: bool) -> u8 {
    let v = if float { v * 255.0 } else { v };
    v.round().clamp(0.0, 255.0) as u8
}

pub(super) fn read(bytes: &[u8]) -> Result<TriMesh> {
    let header = parse_header(bytes)?;
    let body = &bytes[header.body_start..];
    let text;
    let mut src = match header.encoding {
        Encoding::Ascii => {
            text = std::str::from_utf8(body).map_err(|_| bad(FMT, "ASCII body is not UTF-8"))?;
            Src::Ascii(text.split_ascii_whitespace())
        }
        enc => Src::Bin {
            data: body,
            pos: 0,
            be: enc == Encoding::Be,
        },
    };
    // Upper bound for pre-allocation so a hostile element count cannot OOM.
    let cap = |n: usize| n.min(body.len());

    let mut mesh = TriMesh::default();
    let mut saw_vertex = false;
    for el in &header.elements {
        match el.name.as_str() {
            "vertex" => {
                saw_vertex = true;
                read_vertices(&mut src, el, &mut mesh, cap(el.count))?;
            }
            "face" => read_faces(&mut src, el, &mut mesh, cap(el.count))?,
            _ => {
                for _ in 0..el.count {
                    for p in &el.props {
                        skip_prop(&mut src, p.kind)?;
                    }
                }
            }
        }
    }
    if !saw_vertex {
        return Err(bad(FMT, "no vertex element"));
    }
    Ok(mesh)
}

fn read_vertices(src: &mut Src, el: &Element, mesh: &mut TriMesh, cap: usize) -> Result<()> {
    let slots: Vec<Slot> = el.props.iter().map(|p| slot_of(&p.name)).collect();
    let has = |s: Slot| slots.contains(&s);
    if !(has(Slot::X) && has(Slot::Y) && has(Slot::Z)) {
        return Err(bad(FMT, "vertex element lacks x/y/z"));
    }
    let want_n = has(Slot::Nx) && has(Slot::Ny) && has(Slot::Nz);
    let want_c = has(Slot::R) && has(Slot::G) && has(Slot::B);
    let mut positions = Vec::with_capacity(cap);
    let mut normals = Vec::with_capacity(if want_n { cap } else { 0 });
    let mut colors = Vec::with_capacity(if want_c { cap } else { 0 });
    for _ in 0..el.count {
        let mut p = Vec3::ZERO;
        let mut n = Vec3::ZERO;
        let mut c = [0u8, 0, 0, 255];
        for (prop, slot) in el.props.iter().zip(&slots) {
            let Kind::Scalar(ty) = prop.kind else {
                skip_prop(src, prop.kind)?;
                continue;
            };
            let v = src.scalar(ty)?;
            match slot {
                Slot::X => p.x = v as f32,
                Slot::Y => p.y = v as f32,
                Slot::Z => p.z = v as f32,
                Slot::Nx => n.x = v as f32,
                Slot::Ny => n.y = v as f32,
                Slot::Nz => n.z = v as f32,
                Slot::R => c[0] = color_u8(v, ty.is_float()),
                Slot::G => c[1] = color_u8(v, ty.is_float()),
                Slot::B => c[2] = color_u8(v, ty.is_float()),
                Slot::A => c[3] = color_u8(v, ty.is_float()),
                Slot::None => {}
            }
        }
        if !p.is_finite() {
            return Err(bad(FMT, "non-finite vertex coordinate"));
        }
        positions.push(p);
        if want_n {
            normals.push(n);
        }
        if want_c {
            colors.push(c);
        }
    }
    mesh.positions = positions;
    mesh.normals = want_n.then_some(normals);
    mesh.vertex_colors = want_c.then_some(colors);
    Ok(())
}

fn read_faces(src: &mut Src, el: &Element, mesh: &mut TriMesh, cap: usize) -> Result<()> {
    let idx_prop = el
        .props
        .iter()
        .position(|p| p.name == "vertex_indices" || p.name == "vertex_index");
    let Some(idx_prop) = idx_prop.filter(|&i| matches!(el.props[i].kind, Kind::List { .. })) else {
        if el.count == 0 {
            return Ok(());
        }
        return Err(bad(FMT, "face element lacks a vertex_indices list"));
    };
    let mut tris = Vec::with_capacity(cap);
    let mut poly: Vec<u32> = Vec::new();
    for _ in 0..el.count {
        for (i, prop) in el.props.iter().enumerate() {
            if i != idx_prop {
                skip_prop(src, prop.kind)?;
                continue;
            }
            let Kind::List { count, item } = prop.kind else {
                continue;
            };
            let n = src.scalar(count)?;
            if n < 3.0 || n.fract() != 0.0 {
                return Err(bad(FMT, "face with fewer than 3 vertices"));
            }
            poly.clear();
            for _ in 0..n as u64 {
                let v = src.scalar(item)?;
                if v < 0.0 || v.fract() != 0.0 || v > f64::from(u32::MAX) {
                    return Err(bad(FMT, "invalid vertex index"));
                }
                poly.push(v as u32);
            }
            for k in 1..poly.len() - 1 {
                tris.push([poly[0], poly[k], poly[k + 1]]);
            }
        }
    }
    mesh.tris = tris;
    Ok(())
}

pub(super) fn write(w: &mut impl Write, mesh: &TriMesh) -> Result<()> {
    let mut h = String::from("ply\nformat binary_little_endian 1.0\ncomment dc-mesh\n");
    h += &format!("element vertex {}\n", mesh.positions.len());
    h += "property float x\nproperty float y\nproperty float z\n";
    if mesh.normals.is_some() {
        h += "property float nx\nproperty float ny\nproperty float nz\n";
    }
    if mesh.vertex_colors.is_some() {
        h +=
            "property uchar red\nproperty uchar green\nproperty uchar blue\nproperty uchar alpha\n";
    }
    h += &format!("element face {}\n", mesh.tris.len());
    h += "property list uchar int vertex_indices\nend_header\n";
    w.write_all(h.as_bytes())?;
    for (i, p) in mesh.positions.iter().enumerate() {
        for x in p.to_array() {
            w.write_all(&x.to_le_bytes())?;
        }
        if let Some(n) = &mesh.normals {
            for x in n[i].to_array() {
                w.write_all(&x.to_le_bytes())?;
            }
        }
        if let Some(c) = &mesh.vertex_colors {
            w.write_all(&c[i])?;
        }
    }
    for t in &mesh.tris {
        let mut rec = [0u8; 13];
        rec[0] = 3;
        for (k, &i) in t.iter().enumerate() {
            rec[1 + k * 4..5 + k * 4].copy_from_slice(&(i as i32).to_le_bytes());
        }
        w.write_all(&rec)?;
    }
    Ok(())
}
