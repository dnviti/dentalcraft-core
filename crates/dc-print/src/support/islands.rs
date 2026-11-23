//! Coarse raster island check (print plan §4: every island supported).
//!
//! Every layer of the part and its supports is sliced at its mid-plane,
//! z = (k + ½)·h. Overlapping solids merge by the nonzero rule. The slice
//! is rasterised conservatively on the [`PITCH_MM`] grid: a pixel is lit
//! when its centre is inside, or when the slice contour passes through it.
//! Centre sampling alone breaks a sliver thinner than a pixel into isolated
//! pixels, which would be false islands. Each row becomes a list of lit
//! spans.
//!
//! A connected region of a layer (8-neighbour) is held when one of its
//! spans overlaps a lit span of the layer below in the same row. Layer 0
//! rests on the plate. A region held by nothing is an island. The check
//! is lenient by at most one pixel, well under a contact tip.
//!
//! Layers are sliced and checked in parallel and collected in layer order;
//! the first island is the lowest one, then the first in row order. The
//! result does not depend on the thread count.
//!
//! Where an island starts on the part ([`island_bottom`]): the solid's
//! lowest point under the island is a vertex lower than all its
//! neighbours, whose faces face down ([`material_minima`]), between the
//! plane of the layer below and the island's own plane.

use dc_mesh::TriMesh;
use glam::{DVec2, DVec3, Vec2, Vec3};
use rayon::prelude::*;

/// How far around an island's pixels its starting vertex is looked for, mm.
/// A tilted bump starts up to h / tan(slope) beside its first slice.
const BOTTOM_PAD_MM: f32 = 1.0;

/// Vertices where the solid starts: lower than every neighbour (ties by
/// index), their faces facing down on average. Increasing index order.
pub(crate) fn material_minima(mesh: &TriMesh) -> Vec<u32> {
    let pos = &mesh.positions;
    let below = |u: u32, v: u32| {
        let (zu, zv) = (pos[u as usize].z, pos[v as usize].z);
        zu < zv || (zu == zv && u < v)
    };
    let mut lowest = vec![true; pos.len()];
    let mut used = vec![false; pos.len()];
    let mut normal = vec![Vec3::ZERO; pos.len()];
    for (f, t) in mesh.tris.iter().enumerate() {
        let n = mesh.face_normal_raw(f);
        for &v in t {
            used[v as usize] = true;
            normal[v as usize] += n;
            if t.iter().any(|&u| u != v && below(u, v)) {
                lowest[v as usize] = false;
            }
        }
    }
    (0..pos.len() as u32)
        .filter(|&v| used[v as usize] && lowest[v as usize] && normal[v as usize].z < 0.0)
        .collect()
}

/// The vertex of `minima` (see [`material_minima`]) where `island` starts:
/// between the plane of the layer below and the island's plane, within
/// [`BOTTOM_PAD_MM`] of its pixels, the nearest to its centre.
pub(crate) fn island_bottom(
    mesh: &TriMesh,
    minima: &[u32],
    island: &Island,
    layer_um: u32,
) -> Option<u32> {
    let h = f64::from(layer_um.max(1)) / 1000.0;
    let k = f64::from(island.layer);
    let (zlo, zhi) = (((k - 0.5) * h) as f32 - 1e-4, ((k + 0.5) * h) as f32 + 1e-4);
    let (lo, hi) = (
        island.lo - Vec2::splat(BOTTOM_PAD_MM),
        island.hi + Vec2::splat(BOTTOM_PAD_MM),
    );
    minima
        .iter()
        .copied()
        .filter(|&v| {
            let p = mesh.positions[v as usize];
            (zlo..=zhi).contains(&p.z) && p.x >= lo.x && p.x <= hi.x && p.y >= lo.y && p.y <= hi.y
        })
        .min_by(|&a, &b| {
            let d = |v: u32| {
                mesh.positions[v as usize]
                    .truncate()
                    .distance_squared(island.centre)
            };
            d(a).total_cmp(&d(b)).then(a.cmp(&b))
        })
}

/// Per vertex: `true` when one of its faces is flagged.
pub(crate) fn vertex_touches(mesh: &TriMesh, flags: &[bool]) -> Vec<bool> {
    let mut touches = vec![false; mesh.vertex_count()];
    for (t, &flagged) in mesh.tris.iter().zip(flags) {
        if flagged {
            for &v in t {
                touches[v as usize] = true;
            }
        }
    }
    touches
}

/// Raster pitch, mm.
pub(crate) const PITCH_MM: f64 = 0.1;

/// A region of a layer that rests on nothing.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct Island {
    /// Layer index.
    pub layer: u32,
    /// Centre of the region, plate space, mm.
    pub centre: Vec2,
    /// Area of the region, mm².
    pub area_mm2: f32,
    /// Lower corner of the region's pixels, plate space, mm.
    pub lo: Vec2,
    /// Upper corner of the region's pixels, plate space, mm.
    pub hi: Vec2,
}

/// Lit pixels `start..end` of one row.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Span {
    row: u32,
    start: u32,
    end: u32,
}

/// The pixel grid: pixel (i, j) has its centre at
/// (x0 + (i + ½)·pitch, y0 + (j + ½)·pitch). The origin is a multiple of
/// the pitch, so a mesh lights the same pixels whatever else is sliced
/// with it.
#[derive(Debug, Clone, Copy)]
struct Grid {
    x0: f64,
    y0: f64,
    nx: u32,
    ny: u32,
}

/// A region of lit pixels: centre and corners in pixel coordinates.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Region {
    centre: DVec2,
    pixels: u64,
    lo: [u32; 2],
    hi: [u32; 2],
}

/// The lowest island of `meshes` (closed, outward, plate space) sliced at
/// `layer_um`, if any.
#[cfg(test)]
pub(crate) fn first_island(meshes: &[&TriMesh], layer_um: u32) -> Option<Island> {
    islands(meshes, layer_um).into_iter().next()
}

/// Every island of every layer, lowest layer first, then in row order.
pub(crate) fn islands(meshes: &[&TriMesh], layer_um: u32) -> Vec<Island> {
    islands_in(meshes, layer_um, None)
}

/// The islands of the layers `wanted` only (sorted, without repeats): the
/// same as those of [`islands`] in these layers, at the cost of slicing
/// them and the layer below each.
pub(crate) fn islands_at(meshes: &[&TriMesh], layer_um: u32, wanted: &[u32]) -> Vec<Island> {
    if wanted.is_empty() {
        return Vec::new();
    }
    islands_in(meshes, layer_um, Some(wanted))
}

/// The layers whose island may start at one of `vertices` of `mesh` (see
/// [`island_bottom`]), sorted, without repeats.
pub(crate) fn layers_of(mesh: &TriMesh, vertices: &[u32], layer_um: u32) -> Vec<u32> {
    let h = f64::from(layer_um.max(1)) / 1000.0;
    let mut out: Vec<u32> = Vec::with_capacity(2 * vertices.len());
    for &v in vertices {
        // the windows of island_bottom, ((k − ½)h − 1e-4, (k + ½)h + 1e-4),
        // widened against rounding: an extra layer only costs its slice
        let z = f64::from(mesh.positions[v as usize].z);
        let lo = ((z - 1e-3) / h - 0.5).ceil().max(1.0) as u32;
        let hi = ((z + 1e-3) / h + 0.5).floor().max(0.0) as u32;
        out.extend(lo..=hi);
    }
    out.sort_unstable();
    out.dedup();
    out
}

fn islands_in(meshes: &[&TriMesh], layer_um: u32, wanted: Option<&[u32]>) -> Vec<Island> {
    let h = f64::from(layer_um.max(1)) / 1000.0;
    let (mut lo, mut hi) = (DVec3::splat(f64::MAX), DVec3::splat(f64::MIN));
    for p in meshes.iter().flat_map(|m| &m.positions) {
        lo = lo.min(p.as_dvec3());
        hi = hi.max(p.as_dvec3());
    }
    if lo.x > hi.x || hi.z <= 0.0 {
        return Vec::new();
    }
    let x0 = ((lo.x / PITCH_MM).floor() - 1.0) * PITCH_MM;
    let y0 = ((lo.y / PITCH_MM).floor() - 1.0) * PITCH_MM;
    let grid = Grid {
        x0,
        y0,
        nx: ((hi.x - x0) / PITCH_MM).ceil() as u32 + 2,
        ny: ((hi.y - y0) / PITCH_MM).ceil() as u32 + 2,
    };
    let layers = (hi.z / h).ceil() as usize;
    let plane = |k: usize| (k as f64 + 0.5) * h;
    // The layers checked, and the slot of each layer sliced (checked ones
    // and the layer below each), `usize::MAX` for the others.
    let checked: Vec<usize> = match wanted {
        None => (1..layers).collect(),
        Some(w) => w
            .iter()
            .map(|&k| k as usize)
            .filter(|&k| k >= 1 && k < layers)
            .collect(),
    };
    let mut slot = vec![usize::MAX; layers];
    let mut sliced: Vec<usize> = Vec::new();
    for &k in &checked {
        for l in [k - 1, k] {
            if slot[l] == usize::MAX {
                slot[l] = sliced.len();
                sliced.push(l);
            }
        }
    }

    // Faces per sliced layer: the planes each face's z range spans.
    let mut per_layer: Vec<Vec<(u32, u32)>> = vec![Vec::new(); sliced.len()];
    for (m, mesh) in meshes.iter().enumerate() {
        for (f, t) in mesh.tris.iter().enumerate() {
            let z = t.map(|i| f64::from(mesh.positions[i as usize].z));
            let (zmin, zmax) = (z[0].min(z[1]).min(z[2]), z[0].max(z[1]).max(z[2]));
            let first = (zmin / h - 0.5).ceil().max(0.0) as usize;
            let last = (zmax / h - 0.5).floor();
            if last < 0.0 {
                continue;
            }
            for &s in slot.iter().take(last as usize + 1).skip(first) {
                if s != usize::MAX {
                    per_layer[s].push((m as u32, f as u32));
                }
            }
        }
    }

    let spans: Vec<Vec<Span>> = per_layer
        .par_iter()
        .zip(&sliced)
        .map(|(faces, &k)| slice(meshes, faces, plane(k), &grid))
        .collect();
    let mm = |x: f64, y: f64| {
        Vec2::new(
            (grid.x0 + x * PITCH_MM) as f32,
            (grid.y0 + y * PITCH_MM) as f32,
        )
    };
    checked
        .into_par_iter()
        .map(|k| {
            unheld(&spans[slot[k]], &spans[slot[k - 1]])
                .into_iter()
                .map(|r| Island {
                    layer: k as u32,
                    centre: mm(r.centre.x + 0.5, r.centre.y + 0.5),
                    area_mm2: (r.pixels as f64 * PITCH_MM * PITCH_MM) as f32,
                    lo: mm(f64::from(r.lo[0]), f64::from(r.lo[1])),
                    hi: mm(f64::from(r.hi[0]), f64::from(r.hi[1])),
                })
                .collect::<Vec<_>>()
        })
        .collect::<Vec<_>>()
        .into_iter()
        .flatten()
        .collect()
}

/// Lit spans of one layer at height `z` (conservative raster), sorted by
/// row then start, merged.
fn slice(meshes: &[&TriMesh], faces: &[(u32, u32)], z: f64, grid: &Grid) -> Vec<Span> {
    let (rows, cols) = (i64::from(grid.ny), i64::from(grid.nx));
    // (row, x, winding) of every contour crossing of the row centre lines.
    let mut crossings: Vec<(u32, f64, i32)> = Vec::new();
    // Pixels the contour passes through.
    let mut spans: Vec<Span> = Vec::new();
    let column = |x: f64| (((x - grid.x0) / PITCH_MM).floor() as i64).clamp(0, cols - 1) as u32;
    for &(m, f) in faces {
        let mesh = meshes[m as usize];
        let [a, b, c] = mesh.tris[f as usize].map(|i| mesh.positions[i as usize].as_dvec3());
        let mut ends = [DVec2::ZERO; 2];
        let mut n = 0;
        for (p, q) in [(a, b), (b, c), (c, a)] {
            if (p.z < z) != (q.z < z) && n < 2 {
                let s = (z - p.z) / (q.z - p.z);
                ends[n] = p.truncate() + (q.truncate() - p.truncate()) * s;
                n += 1;
            }
        }
        if n < 2 {
            continue;
        }
        // Orient the segment with the material on its left (along Z × n).
        let normal = (b - a).cross(c - a);
        let (mut p, mut q) = (ends[0], ends[1]);
        if (q - p).dot(DVec2::new(-normal.y, normal.x)) < 0.0 {
            std::mem::swap(&mut p, &mut q);
        }
        let (ylo, yhi) = (p.y.min(q.y), p.y.max(q.y));
        let x_at = |y: f64| {
            if q.y == p.y {
                p.x
            } else {
                p.x + (y - p.y) * (q.x - p.x) / (q.y - p.y)
            }
        };
        // The pixels of each row band the segment crosses.
        let band_lo = (((ylo - grid.y0) / PITCH_MM).floor() as i64).clamp(0, rows - 1);
        let band_hi = (((yhi - grid.y0) / PITCH_MM).floor() as i64).clamp(0, rows - 1);
        for row in band_lo..=band_hi {
            let y0 = (grid.y0 + row as f64 * PITCH_MM).max(ylo);
            let y1 = (grid.y0 + (row + 1) as f64 * PITCH_MM).min(yhi);
            let (xa, xb) = if q.y == p.y {
                (p.x, q.x)
            } else {
                (x_at(y0), x_at(y1))
            };
            let (i0, i1) = (column(xa.min(xb)), column(xa.max(xb)));
            spans.push(Span {
                row: row as u32,
                start: i0,
                end: i1 + 1,
            });
        }
        if p.y == q.y {
            continue;
        }
        let winding = if q.y > p.y { 1 } else { -1 };
        // Rows whose centre line y satisfies ylo <= y < yhi.
        let first = ((ylo - grid.y0) / PITCH_MM - 0.5).ceil().max(0.0) as i64;
        let last = (((yhi - grid.y0) / PITCH_MM - 0.5).ceil() as i64 - 1).min(rows - 1);
        for row in first..=last {
            let y = grid.y0 + (row as f64 + 0.5) * PITCH_MM;
            crossings.push((row as u32, x_at(y), winding));
        }
    }
    crossings.sort_by(|a, b| a.0.cmp(&b.0).then(a.1.total_cmp(&b.1)).then(a.2.cmp(&b.2)));
    let pixel = |x: f64| (((x - grid.x0) / PITCH_MM - 0.5).ceil().max(0.0) as u32).min(grid.nx);
    for row in crossings.chunk_by(|a, b| a.0 == b.0) {
        let mut winding = 0;
        let mut from = 0.0;
        for &(r, x, w) in row {
            let before = winding;
            winding += w;
            if before == 0 && winding != 0 {
                from = x;
            } else if before != 0 && winding == 0 {
                let (start, end) = (pixel(from), pixel(x));
                if end > start {
                    spans.push(Span { row: r, start, end });
                }
            }
        }
    }
    spans.sort_unstable_by_key(|s| (s.row, s.start, s.end));
    let mut merged: Vec<Span> = Vec::with_capacity(spans.len());
    for s in spans {
        match merged.last_mut() {
            Some(last) if last.row == s.row && s.start <= last.end => {
                last.end = last.end.max(s.end)
            }
            _ => merged.push(s),
        }
    }
    merged
}

/// The regions of `cur` that overlap nothing in `prev`, in the order of
/// their first span.
fn unheld(cur: &[Span], prev: &[Span]) -> Vec<Region> {
    if cur.is_empty() {
        return Vec::new();
    }
    let mut parent: Vec<u32> = (0..cur.len() as u32).collect();
    fn find(parent: &mut [u32], mut x: u32) -> u32 {
        while parent[x as usize] != x {
            parent[x as usize] = parent[parent[x as usize] as usize];
            x = parent[x as usize];
        }
        x
    }
    // Spans of adjacent rows that touch, corners included, belong to one
    // region. Spans of one row are at least a pixel apart (merged), so the
    // two-pointer sweep sees every touching pair.
    let rows: Vec<&[Span]> = cur.chunk_by(|a, b| a.row == b.row).collect();
    let mut base = 0usize;
    for pair in rows.windows(2) {
        let (upper_base, (a, b)) = (base + pair[0].len(), (pair[0], pair[1]));
        if b[0].row == a[0].row + 1 {
            let (mut i, mut j) = (0, 0);
            while i < a.len() && j < b.len() {
                if a[i].start <= b[j].end && b[j].start <= a[i].end {
                    let (x, y) = (
                        find(&mut parent, (base + i) as u32),
                        find(&mut parent, (upper_base + j) as u32),
                    );
                    parent[x.max(y) as usize] = x.min(y);
                }
                if a[i].end <= b[j].end {
                    i += 1;
                } else {
                    j += 1;
                }
            }
        }
        base = upper_base;
    }
    // A region is held when one of its spans overlaps a span below.
    let mut held = vec![false; cur.len()];
    for (i, s) in cur.iter().enumerate() {
        let k = prev.partition_point(|p| (p.row, p.end) <= (s.row, s.start));
        if k < prev.len() && prev[k].row == s.row && prev[k].start < s.end {
            let root = find(&mut parent, i as u32);
            held[root as usize] = true;
        }
    }
    // Unheld regions, numbered in the order of their first span.
    let mut slot = vec![u32::MAX; cur.len()];
    let mut regions: Vec<Region> = Vec::new();
    for (i, s) in cur.iter().enumerate() {
        let root = find(&mut parent, i as u32) as usize;
        if held[root] {
            continue;
        }
        if slot[root] == u32::MAX {
            slot[root] = regions.len() as u32;
            regions.push(Region {
                centre: DVec2::ZERO,
                pixels: 0,
                lo: [s.start, s.row],
                hi: [s.end, s.row + 1],
            });
        }
        let r = &mut regions[slot[root] as usize];
        let n = u64::from(s.end - s.start);
        let mid = 0.5 * f64::from(s.start + s.end - 1);
        r.centre += DVec2::new(mid, f64::from(s.row)) * n as f64;
        r.pixels += n;
        r.lo = [r.lo[0].min(s.start), r.lo[1].min(s.row)];
        r.hi = [r.hi[0].max(s.end), r.hi[1].max(s.row + 1)];
    }
    for r in &mut regions {
        r.centre /= r.pixels as f64;
    }
    regions
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::part::Strut;
    use crate::support::solid::strut_mesh;
    use glam::Vec3;

    /// Closed box between `lo` and `hi`, outward.
    fn cuboid(lo: Vec3, hi: Vec3) -> TriMesh {
        let p = |x: bool, y: bool, z: bool| {
            Vec3::new(
                if x { hi.x } else { lo.x },
                if y { hi.y } else { lo.y },
                if z { hi.z } else { lo.z },
            )
        };
        let positions = vec![
            p(false, false, false),
            p(true, false, false),
            p(true, true, false),
            p(false, true, false),
            p(false, false, true),
            p(true, false, true),
            p(true, true, true),
            p(false, true, true),
        ];
        let tris = vec![
            [0, 2, 1],
            [0, 3, 2],
            [4, 5, 6],
            [4, 6, 7],
            [0, 1, 5],
            [0, 5, 4],
            [1, 2, 6],
            [1, 6, 5],
            [2, 3, 7],
            [2, 7, 6],
            [3, 0, 4],
            [3, 4, 7],
        ];
        TriMesh::new(positions, tris)
    }

    #[test]
    fn a_floating_box_is_an_island_until_a_strut_holds_it() {
        let block = cuboid(Vec3::new(-2.0, -1.0, 3.0), Vec3::new(2.0, 1.0, 4.0));
        let island = first_island(&[&block], 50).expect("floating box");
        assert_eq!(island.layer, 60, "first layer above z = 3");
        // Conservative raster: up to one pixel more all round.
        assert!(
            island.centre.length() < 0.1 && (7.9..9.5).contains(&island.area_mm2),
            "{island:?}"
        );
        let strut = strut_mesh(&Strut {
            tip: Vec3::new(0.5, 0.0, 3.0),
            foot: Vec3::new(0.5, 0.0, 0.0),
            tip_d_mm: 0.35,
            shaft_d_mm: 1.0,
        });
        assert_eq!(first_island(&[&block, &strut], 50), None);
    }

    #[test]
    fn a_box_on_the_plate_and_an_overhang_on_it_are_held() {
        // An L: a column on the plate and an arm reaching out from it.
        let column = cuboid(Vec3::new(0.0, 0.0, 0.0), Vec3::new(2.0, 2.0, 6.0));
        let arm = cuboid(Vec3::new(1.0, 0.0, 4.0), Vec3::new(8.0, 2.0, 5.0));
        assert_eq!(first_island(&[&column, &arm], 50), None);
        // A second arm floating beside the column is caught.
        let loose = cuboid(Vec3::new(4.0, 4.0, 2.0), Vec3::new(5.0, 5.0, 3.0));
        let island = first_island(&[&column, &arm, &loose], 100).expect("loose arm");
        assert_eq!(island.layer, 20);
        assert!(
            (island.centre - Vec2::new(4.5, 4.5)).length() < 0.1,
            "{island:?}"
        );
    }

    /// The islands of some layers are those the full check finds there.
    #[test]
    fn the_islands_of_some_layers_are_those_of_the_full_check() {
        let column = cuboid(Vec3::new(0.0, 0.0, 0.0), Vec3::new(2.0, 2.0, 6.0));
        let loose = cuboid(Vec3::new(4.0, 4.0, 2.0), Vec3::new(5.0, 5.0, 3.0));
        let high = cuboid(Vec3::new(-4.0, 4.0, 4.0), Vec3::new(-3.0, 5.0, 5.0));
        let meshes = [&column, &loose, &high];
        let all = islands(&meshes, 100);
        assert_eq!(all.iter().map(|i| i.layer).collect::<Vec<_>>(), [20, 40]);
        assert_eq!(islands_at(&meshes, 100, &[20, 40]), all);
        assert_eq!(islands_at(&meshes, 100, &[40]), all[1..]);
        assert_eq!(islands_at(&meshes, 100, &[3, 41]), []);
        assert_eq!(islands_at(&meshes, 100, &[]), []);
        // the layers of the vertices where the loose box starts
        let low: Vec<u32> = (0..loose.vertex_count() as u32)
            .filter(|&v| loose.positions[v as usize].z == 2.0)
            .collect();
        assert!(layers_of(&loose, &low, 100).contains(&20));
    }
}
