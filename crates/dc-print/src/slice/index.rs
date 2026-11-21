//! The z-sorted triangle index and its sweep (print plan §5).
//!
//! Every triangle that crosses at least one sampling plane is listed once,
//! with the layers it crosses ([`super::z`]), sorted by its first layer. The
//! sweep walks the layers in consecutive batches. It keeps the *active*
//! triangles (those whose layers meet the batch) and buckets them per layer,
//! so a layer only touches the triangles that cross it.
//!
//! Memory: 16 bytes per indexed triangle, plus per batch one entry per
//! (triangle, layer) crossing inside the batch; nothing scales with the
//! number of layers of the plate.

use std::ops::Range;

use dc_mesh::TriMesh;
use rayon::prelude::*;

use super::z;

/// A triangle of the plate: its mesh and its index in that mesh.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub(crate) struct TriId {
    /// Index into the plate's meshes.
    pub(crate) mesh: u32,
    /// Triangle index in that mesh.
    pub(crate) tri: u32,
}

/// A triangle and the layers it crosses.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Entry {
    /// First layer it crosses.
    first: u32,
    /// One past the last layer it crosses.
    end: u32,
    /// The triangle.
    id: TriId,
}

/// The triangles of a plate, sorted by the first layer they cross.
#[derive(Debug)]
pub(crate) struct ZIndex {
    entries: Vec<Entry>,
}

impl ZIndex {
    /// Indexes the triangles of `meshes` for `layers` layers of `layer_um`.
    /// A triangle crosses layer i when z_min ≤ (i + ½)·h < z_max; flat
    /// triangles and triangles between two planes cross none and are left
    /// out. Indices must be valid (the caller checks them).
    pub(crate) fn build(meshes: &[&TriMesh], layer_um: u32, layers: u32) -> Self {
        let mut entries = Vec::new();
        for (m, mesh) in meshes.iter().enumerate() {
            entries.par_extend(mesh.tris.par_iter().enumerate().filter_map(|(t, tri)| {
                let z = tri.map(|i| f64::from(mesh.positions[i as usize].z));
                let first = z::first_layer_at_or_above(z[0].min(z[1]).min(z[2]), layer_um, layers);
                let end = z::first_layer_at_or_above(z[0].max(z[1]).max(z[2]), layer_um, layers);
                (first < end).then_some(Entry {
                    first,
                    end,
                    id: TriId {
                        mesh: m as u32,
                        tri: t as u32,
                    },
                })
            }));
        }
        // Unique keys: the order is the same on every run and thread count.
        entries.par_sort_unstable_by_key(|e| (e.first, e.id));
        Self { entries }
    }

    /// Number of indexed triangles.
    #[cfg(test)]
    pub(crate) fn len(&self) -> usize {
        self.entries.len()
    }

    /// A sweep from layer 0.
    pub(crate) fn sweep(&self) -> Sweep<'_> {
        Sweep {
            entries: &self.entries,
            next: 0,
            active: Vec::new(),
            starts: Vec::new(),
            cursor: Vec::new(),
            tris: Vec::new(),
            layers: 0..0,
        }
    }
}

/// Walks the layers in consecutive batches (see the module docs).
#[derive(Debug)]
pub(crate) struct Sweep<'a> {
    entries: &'a [Entry],
    /// First entry not yet active.
    next: usize,
    /// Entries whose layers may still meet the current or a later batch.
    active: Vec<Entry>,
    /// Layer `layers.start + k` has its triangles at `tris[starts[k]..starts[k + 1]]`.
    starts: Vec<usize>,
    /// Scratch for the bucketing.
    cursor: Vec<usize>,
    /// Triangles of the current batch, grouped per layer.
    tris: Vec<TriId>,
    /// The current batch.
    layers: Range<u32>,
}

impl Sweep<'_> {
    /// Moves to the batch `layers`, which starts where the previous batch
    /// ended, and buckets its triangles per layer.
    pub(crate) fn advance(&mut self, layers: Range<u32>) {
        debug_assert_eq!(layers.start, self.layers.end);
        let (lo, hi) = (layers.start, layers.end);
        self.active.retain(|e| e.end > lo);
        while let Some(e) = self.entries.get(self.next)
            && e.first < hi
        {
            self.active.push(*e);
            self.next += 1;
        }
        let n = (hi - lo) as usize;
        self.starts.clear();
        self.starts.resize(n + 1, 0);
        for e in &self.active {
            for i in e.first.max(lo)..e.end.min(hi) {
                self.starts[(i - lo) as usize + 1] += 1;
            }
        }
        for k in 0..n {
            self.starts[k + 1] += self.starts[k];
        }
        self.cursor.clear();
        self.cursor.extend_from_slice(&self.starts[..n]);
        self.tris.clear();
        self.tris.resize(self.starts[n], TriId { mesh: 0, tri: 0 });
        for e in &self.active {
            for i in e.first.max(lo)..e.end.min(hi) {
                let k = (i - lo) as usize;
                self.tris[self.cursor[k]] = e.id;
                self.cursor[k] += 1;
            }
        }
        self.layers = layers;
    }

    /// The triangles crossing `layer`, which must be in the current batch.
    pub(crate) fn layer(&self, layer: u32) -> &[TriId] {
        let k = (layer - self.layers.start) as usize;
        &self.tris[self.starts[k]..self.starts[k + 1]]
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::Vec3;

    /// One triangle per z range, as (z_min, z_max) pairs.
    fn slivers(ranges: &[(f32, f32)]) -> TriMesh {
        let mut positions = Vec::new();
        let mut tris = Vec::new();
        for (k, &(a, b)) in ranges.iter().enumerate() {
            let n = positions.len() as u32;
            let x = k as f32;
            positions.extend([
                Vec3::new(x, 0.0, a),
                Vec3::new(x + 1.0, 0.0, a),
                Vec3::new(x, 1.0, b),
            ]);
            tris.push([n, n + 1, n + 2]);
        }
        TriMesh::new(positions, tris)
    }

    #[test]
    fn layers_hold_exactly_the_crossing_triangles() {
        // h = 0.1 mm: planes at 0.05, 0.15, 0.25 (exact), ..., 0.75 (exact).
        let ranges = [
            (0.0, 1.0),
            (0.25, 0.75),
            (0.2, 0.24),
            (0.3, 0.3),
            (0.55, 2.0),
        ];
        let mesh = slivers(&ranges);
        let layers = 12;
        let index = ZIndex::build(&[&mesh], 100, layers);
        // (0.2, 0.24) crosses no plane and (0.3, 0.3) is flat.
        assert_eq!(index.len(), 3);
        let mut sweep = index.sweep();
        for start in (0..layers).step_by(5) {
            let batch = start..(start + 5).min(layers);
            sweep.advance(batch.clone());
            for i in batch {
                let mut got: Vec<u32> = sweep.layer(i).iter().map(|t| t.tri).collect();
                got.sort_unstable();
                let zc = z::centre_mm(i, 100);
                let want: Vec<u32> = (0..ranges.len() as u32)
                    .filter(|&t| {
                        let (a, b) = ranges[t as usize];
                        f64::from(a) <= zc && zc < f64::from(b)
                    })
                    .collect();
                assert_eq!(got, want, "layer {i}");
            }
        }
        // On-plane ends: [0.25, 0.75) lights layers 2..7, not 7.
        assert_eq!(z::first_layer_at_or_above(0.25, 100, layers), 2);
        assert_eq!(z::first_layer_at_or_above(0.75, 100, layers), 7);
    }
}
