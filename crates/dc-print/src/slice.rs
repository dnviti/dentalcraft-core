//! Mesh slicing into layer runs (print plan §5). Owner: Slicer.
//!
//! **Plate frame.** Plate space is in mm, with the origin at the corner of
//! plate pixel (0, 0), x along the columns, y along the rows and the plate at
//! z = 0, as [`PrinterProfile::map_pixel`] defines it: plate pixel
//! (col, row) has its centre at ((col + ½)·pitch_x, (row + ½)·pitch_y).
//! Pixels need not be square (14 × 19 µm on the Saturn 4 Ultra 16K).
//! Geometry outside [0, res_x·pitch_x] × [0, res_y·pitch_y] is clipped.
//!
//! **Sampling.** Black and white, centre sampled, half-open everywhere:
//! - layer i is the section at z = (i + ½)·h (`z`), and a vertex on that
//!   plane counts as below it, so a solid spanning [z0, z1) lights the
//!   layers whose centre lies in [z0, z1);
//! - pixel (k, j) is lit when its centre ((k + ½)·pitch_x,
//!   (j + ½)·pitch_y) is inside, with the same half-open rule along rows and
//!   columns (`pixel`, `fill`), so pixel counts are exact and touching
//!   parts never light a pixel twice;
//! - inside means a nonzero winding number over the sections of all meshes
//!   together, so supports overlapping a part, or overlapping parts, form one
//!   solid without a mesh boolean and cure once.
//!
//! **Pipeline.**
//! - `index`: triangles are sorted by the first layer they cross, and a
//!   sweep keeps the active ones, so each layer only touches the triangles
//!   that cross it.
//! - `section`: each crossing triangle gives one directed segment. Crossing
//!   points are interpolated from an edge's lower to its upper vertex, so
//!   neighbouring triangles get bit-identical points and contours close
//!   exactly.
//! - `fill`: an edge table over the rows gives each row's sorted crossings
//!   and its lit pixel spans.
//! - [`goo::runs_from_row_spans`]: the spans become runs in file pixel
//!   order, through `map_pixel`, the only place where mirroring happens.
//!
//! **Threads and memory.** Layers are computed in parallel, in batches of a
//! few layers per thread. While one batch goes to the sink, on the calling
//! thread and strictly in layer order, the next batch is computed. At most
//! two batches exist at any time; nothing is kept for all layers.
//!
//! **Determinism.** Each layer is computed with f64 arithmetic in a fixed
//! order, and nothing is reduced across threads. The volume is the integer
//! count of lit pixels × pixel area × h. The same input gives the same runs
//! and volume on every platform and with any number of threads.
//!
//! **Anti-aliasing.** None in v1 (binary pixels). The hook is the row
//! quantiser in `fill`.

mod fill;
mod index;
mod pixel;
mod section;
#[cfg(test)]
mod tests;
mod z;

use std::ops::Range;

use dc_mesh::TriMesh;
use rayon::prelude::*;

use crate::PrinterProfile;
use crate::goo::{self, GooError, Run};

use fill::{Filler, RowSpans};
use index::{Sweep, TriId, ZIndex};
use pixel::Grid;

/// One sliced layer.
#[derive(Debug, Clone, PartialEq)]
pub struct SlicedLayer {
    /// Runs in file pixel order (already through `PrinterProfile::map_pixel`).
    pub runs: Vec<Run>,
    /// Lit area of the layer, mm².
    pub area_mm2: f64,
}

/// Number of layers [`slice_plate`] emits for `meshes` at `layer_um`: the
/// layers whose sampling plane z = (i + ½)·h lies strictly below the highest
/// vertex of any triangle. This is ⌈H / h⌉ whenever the top H lies in the
/// upper half of its layer (any whole multiple of h included); a top in the
/// lower half would only add a blank layer, so none is emitted.
///
/// 0 for no triangles or `layer_um` 0; saturates at `u32::MAX`. Non-finite
/// coordinates and out-of-range indices are ignored here (`slice_plate`
/// refuses them).
pub fn layer_count(meshes: &[&TriMesh], layer_um: u32) -> u32 {
    match top_mm(meshes) {
        Some(top) if layer_um > 0 => z::first_layer_at_or_above(top, layer_um, u32::MAX),
        _ => 0,
    }
}

/// Slices closed meshes (plate space, mm, plate at z = 0) into layers of
/// `layer_um`, calling `sink` in layer order. Returns the total cured
/// volume in mm³.
///
/// The meshes should be closed and outward-oriented (an inward-oriented
/// mesh alone slices the same, but where it overlaps an outward one the
/// windings cancel and leave a hole: that is how a cavity is cut). Exactly
/// [`layer_count`] layers are emitted, as `sink(0, ..)`, `sink(1, ..)`, … on
/// the calling thread; the first error from `sink` stops slicing and is
/// returned. The volume is Σ lit pixels × pixel area × h, supports included.
///
/// Errors (`GooError::Field`): `layer_um` 0, a pixel pitch that is not
/// positive, a triangle with an index out of range or a non-finite vertex,
/// and layers that would rise above the profile's build height.
pub fn slice_plate(
    meshes: &[&TriMesh],
    profile: &PrinterProfile,
    layer_um: u32,
    sink: &mut dyn FnMut(u32, SlicedLayer) -> Result<(), GooError>,
) -> Result<f64, GooError> {
    if layer_um == 0 {
        return Err(GooError::Field("layer height"));
    }
    let grid = Grid::new(profile)?;
    check_meshes(meshes)?;
    let layers = layer_count(meshes, layer_um);
    let build_um = (f64::from(profile.build_mm[2]) * 1000.0).round();
    let fits = f64::from(layers) * f64::from(layer_um) <= build_um;
    if !fits {
        return Err(GooError::Field("plate taller than the build volume"));
    }

    let index = ZIndex::build(meshes, layer_um, layers);
    let slicer = LayerSlicer {
        meshes,
        profile,
        grid,
        layer_um,
    };
    let mut sweep = index.sweep();
    let batch = batch_len();
    let mut lit_total = 0u64;
    // Computed layers not yet given to the sink: `next - ready.len()..next`.
    let mut ready: Vec<Layer> = Vec::new();
    let mut next = 0u32;
    loop {
        let todo = next..next.saturating_add(batch).min(layers);
        let first_ready = next - ready.len() as u32;
        let mut computed = Vec::new();
        let (sweep_ref, computed_ref, slicer_ref) = (&mut sweep, &mut computed, &slicer);
        rayon::in_place_scope(|s| {
            if !todo.is_empty() {
                let todo = todo.clone();
                s.spawn(move |_| *computed_ref = slicer_ref.batch(sweep_ref, todo));
            }
            for (k, layer) in ready.drain(..).enumerate() {
                lit_total += layer.lit;
                sink(first_ready + k as u32, layer.sliced)?;
            }
            Ok::<(), GooError>(())
        })?;
        if todo.is_empty() {
            break;
        }
        next = todo.end;
        ready = computed;
    }
    Ok(lit_total as f64 * grid.pixel_area_mm2() * f64::from(layer_um) / 1000.0)
}

/// Layers per batch: a few per thread, for load balance; the result does not
/// depend on it.
fn batch_len() -> u32 {
    (rayon::current_num_threads() * 4).clamp(8, 96) as u32
}

/// The highest finite z of any vertex referenced by a triangle.
fn top_mm(meshes: &[&TriMesh]) -> Option<f64> {
    let mut top: Option<f32> = None;
    for mesh in meshes {
        for tri in &mesh.tris {
            for &i in tri {
                if let Some(p) = mesh.positions.get(i as usize)
                    && p.z.is_finite()
                {
                    top = Some(top.map_or(p.z, |t| t.max(p.z)));
                }
            }
        }
    }
    top.map(f64::from)
}

/// Refuses triangles that cannot be sliced: an index out of range or a
/// vertex that is not finite. The first offender in mesh order decides the
/// error.
fn check_meshes(meshes: &[&TriMesh]) -> Result<(), GooError> {
    for mesh in meshes {
        for tri in &mesh.tris {
            for &i in tri {
                match mesh.positions.get(i as usize) {
                    None => return Err(GooError::Field("triangle index out of range")),
                    Some(p) if !p.is_finite() => {
                        return Err(GooError::Field("non-finite vertex"));
                    }
                    Some(_) => {}
                }
            }
        }
    }
    Ok(())
}

/// One computed layer and its lit pixel count.
struct Layer {
    sliced: SlicedLayer,
    lit: u64,
}

/// Buffers of one rayon job, reused from layer to layer.
#[derive(Default)]
struct Scratch {
    filler: Filler,
    spans: RowSpans,
}

/// What every layer task needs.
struct LayerSlicer<'a> {
    meshes: &'a [&'a TriMesh],
    profile: &'a PrinterProfile,
    grid: Grid,
    layer_um: u32,
}

impl LayerSlicer<'_> {
    /// Computes the layers of `layers` in parallel, in order.
    fn batch(&self, sweep: &mut Sweep<'_>, layers: Range<u32>) -> Vec<Layer> {
        sweep.advance(layers.clone());
        let sweep = &*sweep;
        layers
            .into_par_iter()
            .map_init(Scratch::default, |scratch, i| {
                self.layer(i, sweep.layer(i), scratch)
            })
            .collect()
    }

    /// Slices layer `i` from the triangles crossing it.
    fn layer(&self, i: u32, tris: &[TriId], scratch: &mut Scratch) -> Layer {
        let zc = z::centre_mm(i, self.layer_um);
        scratch.filler.clear();
        for t in tris {
            let tri = self.meshes[t.mesh as usize].triangle(t.tri as usize);
            if let Some(segment) = section::cut(tri, zc) {
                scratch.filler.add(segment, &self.grid);
            }
        }
        scratch.filler.fill(&self.grid, &mut scratch.spans);
        let spans = &scratch.spans;
        let runs = goo::runs_from_row_spans(self.profile, |row| spans.row(row));
        let lit = spans.lit();
        Layer {
            sliced: SlicedLayer {
                runs,
                area_mm2: lit as f64 * self.grid.pixel_area_mm2(),
            },
            lit,
        }
    }
}
