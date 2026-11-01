//! Mesh file I/O (STL ASCII/binary, OBJ, PLY) and unit handling.
//!
//! All parsers are hand-written, pure Rust and never panic on malformed
//! input: truncated or corrupt data yields [`MeshError::Format`]. Files are
//! read fully into memory (`std::fs::read`; the workspace forbids `unsafe`, so
//! no memory mapping), which keeps a 1M-triangle binary STL well below 1 s in
//! release builds.
//!
//! * STL: ASCII and binary are told apart by content, not by the header
//!   (binary files may start with `solid`). Triangle soups are welded with a
//!   tolerance based hash, see [`ReadOptions::weld_tolerance`].
//! * OBJ: `v` (optionally `v x y z r g b`) and `f`; polygons are fan
//!   triangulated, `vt`/`vn` and everything else are ignored.
//! * PLY: ASCII, binary little and big endian. Writers emit binary LE.

mod obj;
mod ply;
mod stl;
mod weld;

use std::io::Write;
use std::path::Path;

use serde::{Deserialize, Serialize};

use crate::{MeshError, Result, TriMesh};

/// Length unit of a source file. DentalCraft works in millimetres internally.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default, Serialize, Deserialize)]
pub enum Units {
    /// Metres.
    #[serde(rename = "m")]
    Meters,
    /// Centimetres.
    #[serde(rename = "cm")]
    Centimeters,
    /// Millimetres (internal unit).
    #[default]
    #[serde(rename = "mm")]
    Millimeters,
    /// Micrometres.
    #[serde(rename = "um")]
    Micrometers,
}

impl Units {
    /// All units, largest first.
    pub const ALL: [Units; 4] = [
        Units::Meters,
        Units::Centimeters,
        Units::Millimeters,
        Units::Micrometers,
    ];

    /// Factor that converts a length in this unit to millimetres.
    pub fn to_mm(self) -> f32 {
        match self {
            Units::Meters => 1000.0,
            Units::Centimeters => 10.0,
            Units::Millimeters => 1.0,
            Units::Micrometers => 0.001,
        }
    }

    /// Short symbol (`m`, `cm`, `mm`, `um`), identical to the serialized form.
    pub fn symbol(self) -> &'static str {
        match self {
            Units::Meters => "m",
            Units::Centimeters => "cm",
            Units::Millimeters => "mm",
            Units::Micrometers => "um",
        }
    }
}

/// Supported mesh file formats.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum MeshFormat {
    /// STL, ASCII or binary (detected on read; binary on write).
    Stl,
    /// Wavefront OBJ (triangles; polygons are fan-triangulated).
    Obj,
    /// Stanford PLY, ASCII or binary little endian.
    Ply,
}

impl MeshFormat {
    /// Format from the file extension (case-insensitive).
    pub fn from_path(path: &Path) -> Result<Self> {
        let ext = path
            .extension()
            .and_then(|e| e.to_str())
            .map(str::to_ascii_lowercase)
            .unwrap_or_default();
        match ext.as_str() {
            "stl" => Ok(Self::Stl),
            "obj" => Ok(Self::Obj),
            "ply" => Ok(Self::Ply),
            _ => Err(MeshError::UnsupportedFormat(path.display().to_string())),
        }
    }

    /// Canonical lowercase file extension.
    pub fn extension(self) -> &'static str {
        match self {
            Self::Stl => "stl",
            Self::Obj => "obj",
            Self::Ply => "ply",
        }
    }
}

/// Options for the mesh readers.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ReadOptions {
    /// Vertex welding tolerance for formats that store triangle soups (STL),
    /// relative to the bounding box diagonal. Vertices closer than this (and
    /// in the same quantisation cell) are merged. `0` merges only bit-identical
    /// positions. Default `1e-6`.
    pub weld_tolerance: f32,
}

impl Default for ReadOptions {
    fn default() -> Self {
        Self {
            weld_tolerance: 1e-6,
        }
    }
}

/// Builds a [`MeshError::Format`].
pub(crate) fn bad(format: &'static str, reason: impl Into<String>) -> MeshError {
    MeshError::Format {
        format,
        reason: reason.into(),
    }
}

/// Reads a mesh file, choosing the format from the extension. Positions are
/// returned as stored in the file (no unit conversion).
pub fn read(path: impl AsRef<Path>) -> Result<TriMesh> {
    let format = MeshFormat::from_path(path.as_ref())?;
    read_as(path, format)
}

/// Reads a mesh file in the given format with default [`ReadOptions`].
pub fn read_as(path: impl AsRef<Path>, format: MeshFormat) -> Result<TriMesh> {
    read_with(path, format, &ReadOptions::default())
}

/// Reads a mesh file in the given format.
pub fn read_with(
    path: impl AsRef<Path>,
    format: MeshFormat,
    options: &ReadOptions,
) -> Result<TriMesh> {
    let bytes = std::fs::read(path)?;
    read_bytes(&bytes, format, options)
}

/// Parses a mesh from an in-memory file image.
pub fn read_bytes(bytes: &[u8], format: MeshFormat, options: &ReadOptions) -> Result<TriMesh> {
    let mesh = match format {
        MeshFormat::Stl => stl::read(bytes, options)?,
        MeshFormat::Obj => obj::read(bytes)?,
        MeshFormat::Ply => ply::read(bytes)?,
    };
    mesh.check()?;
    Ok(mesh)
}

/// Result of [`read_scan`]: the raw mesh plus what the reader learnt about the file.
#[derive(Debug, Clone, PartialEq)]
pub struct ScanFile {
    /// Mesh as stored in the file (no unit conversion, no rotation, no cleanup).
    pub mesh: TriMesh,
    /// Format detected from the file extension.
    pub format: MeshFormat,
    /// For OBJ: number of `o` objects that own at least one face. Faces before
    /// the first `o` count as one implicit object, so a plain OBJ with faces
    /// gives `Some(1)` and one without faces `Some(0)`. `None` for STL and PLY.
    pub obj_object_count: Option<usize>,
}

/// Reads a scan file like [`read_with`] (format from the extension) and also
/// reports the detected format and, for OBJ, the object count. The mesh is
/// raw: callers apply cleanup, unit and axis conversion themselves.
pub fn read_scan(path: impl AsRef<Path>, options: &ReadOptions) -> Result<ScanFile> {
    let format = MeshFormat::from_path(path.as_ref())?;
    let bytes = std::fs::read(path)?;
    read_scan_bytes(&bytes, format, options)
}

/// In-memory variant of [`read_scan`].
pub fn read_scan_bytes(
    bytes: &[u8],
    format: MeshFormat,
    options: &ReadOptions,
) -> Result<ScanFile> {
    let (mesh, obj_object_count) = match format {
        MeshFormat::Obj => {
            let (mesh, count) = obj::read_counted(bytes)?;
            (mesh, Some(count))
        }
        _ => (read_bytes(bytes, format, options)?, None),
    };
    mesh.check()?;
    Ok(ScanFile {
        mesh,
        format,
        obj_object_count,
    })
}

/// Writes `mesh` to `path` in the given format (STL: binary).
pub fn write(path: impl AsRef<Path>, mesh: &TriMesh, format: MeshFormat) -> Result<()> {
    let mut w = std::io::BufWriter::with_capacity(1 << 20, std::fs::File::create(path)?);
    write_to(&mut w, mesh, format)?;
    w.flush()?;
    Ok(())
}

/// Writes `mesh` to a stream in the given format (STL: binary).
pub fn write_to(w: &mut impl Write, mesh: &TriMesh, format: MeshFormat) -> Result<()> {
    mesh.check()?;
    match format {
        MeshFormat::Stl => stl::write_binary(w, mesh),
        MeshFormat::Obj => obj::write(w, mesh),
        MeshFormat::Ply => ply::write(w, mesh),
    }
}

/// Writes `mesh` as an ASCII STL file.
pub fn write_stl_ascii(path: impl AsRef<Path>, mesh: &TriMesh) -> Result<()> {
    let mut w = std::io::BufWriter::with_capacity(1 << 20, std::fs::File::create(path)?);
    write_stl_ascii_to(&mut w, mesh)?;
    w.flush()?;
    Ok(())
}

/// Writes `mesh` as an ASCII STL stream.
pub fn write_stl_ascii_to(w: &mut impl Write, mesh: &TriMesh) -> Result<()> {
    mesh.check()?;
    stl::write_ascii(w, mesh)
}

/// Guesses the unit of a dental scan from its largest bounding box extent
/// (heuristic ported from the Blender plugin's `fix_units`). A dental arch is
/// 40-100 mm wide, so an extent of 0.02-0.2 means metres, 2-20 centimetres,
/// 20 000-200 000 micrometres; anything else is taken as millimetres.
/// Empty meshes are reported as millimetres; non-finite positions are an error.
pub fn detect_units(mesh: &TriMesh) -> Result<Units> {
    if mesh.positions.iter().any(|p| !p.is_finite()) {
        return Err(bad("mesh", "non-finite vertex positions"));
    }
    let Some(b) = mesh.aabb() else {
        return Ok(Units::Millimeters);
    };
    let size = b.size().max_element();
    Ok(match size {
        s if (0.02..=0.2).contains(&s) => Units::Meters,
        s if (2.0..=20.0).contains(&s) => Units::Centimeters,
        s if (20_000.0..=200_000.0).contains(&s) => Units::Micrometers,
        _ => Units::Millimeters,
    })
}

/// Detects the unit of `mesh` with [`detect_units`] and converts it to
/// millimetres in place. Returns the detected unit.
pub fn normalize_units(mesh: &mut TriMesh) -> Result<Units> {
    let units = detect_units(mesh)?;
    mesh.scale_to_mm(units);
    Ok(units)
}

#[cfg(test)]
mod format_tests;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn format_from_extension() {
        assert_eq!(
            MeshFormat::from_path(Path::new("a/B.STL")).unwrap(),
            MeshFormat::Stl
        );
        assert_eq!(
            MeshFormat::from_path(Path::new("x.ply")).unwrap(),
            MeshFormat::Ply
        );
        assert!(MeshFormat::from_path(Path::new("x.dcm")).is_err());
    }

    #[test]
    fn units_convert_and_serialize() {
        assert_eq!(Units::Meters.to_mm(), 1000.0);
        assert_eq!(
            serde_json::to_string(&Units::Micrometers).unwrap(),
            "\"um\""
        );
        for u in Units::ALL {
            assert_eq!(
                serde_json::to_string(&u).unwrap(),
                format!("\"{}\"", u.symbol())
            );
        }
    }
}
