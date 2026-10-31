//! Level set operations: re-distancing, offset, CSG, interior fill.

use glam::{IVec3, Vec3};
use rayon::prelude::*;

use dc_mesh::TriMesh;

use super::build::{candidate_blocks, scan_block};
use super::flood::{self, Domain, Mask};
use super::store::{self, SIDE, Store, VOXELS, local_coord};
use super::{CsgOp, SdfGrid, mesher};
use crate::bvh::Bvh;
use crate::{GeomError, Result};

/// Rebuilds the narrow band of half width `band` around the current zero
/// level: exact distances to the extracted surface, signs from the current
/// values.
pub(crate) fn redistance(grid: &mut SdfGrid, band: f32) -> Result<()> {
    let (vs, origin) = (grid.voxel_size, grid.origin);
    let (positions, tris) = mesher::extract(&grid.store, grid.band, origin, vs, 0.0);
    if tris.is_empty() {
        // no surface: the solid is empty (the outside is unbounded)
        grid.store = Store::default();
        grid.band = band;
        grid.exact = true;
        return Ok(());
    }
    let mesh = TriMesh::new(positions, tris);
    let bvh = Bvh::build(&mesh)?;
    drop(mesh);
    let candidates = candidate_blocks(&bvh, origin, vs, band)?;
    let old = &grid.store;
    let old_band = grid.band;
    let blocks: Vec<(IVec3, Box<[f32; VOXELS]>)> = candidates
        .par_iter()
        .filter_map(|&(key, dc)| {
            let mut vals = Box::new([band; VOXELS]);
            let mut any = false;
            let src = old.find(key).map(|i| old.block(i));
            let bg = old.background(key, old_band);
            scan_block(&bvh, key, dc, origin, vs, band, |i, _, d, _| {
                any |= d < band;
                let inside = src.map_or(bg, |v| v[i]) < 0.0;
                vals[i] = store::signed(d, inside);
            });
            any.then_some((key, vals))
        })
        .collect();
    let mut new_index = store::KeySet::default();
    new_index.extend(blocks.iter().map(|(k, _)| store::pack(*k)));
    // blocks left out of the new band keep their (uniform) sign
    let mut tiles = old.sorted_tiles();
    tiles.extend(
        old.keys
            .iter()
            .enumerate()
            .filter(|&(i, k)| !new_index.contains(&store::pack(*k)) && old.block(i)[0] < 0.0)
            .map(|(_, k)| *k),
    );
    let keys: Vec<IVec3> = blocks.iter().map(|(k, _)| *k).collect();
    let mut data = Vec::with_capacity(keys.len() * VOXELS);
    for (_, v) in &blocks {
        data.extend_from_slice(&v[..]);
    }
    drop(blocks);
    grid.store = Store::new(keys, data, tiles);
    grid.band = band;
    grid.exact = true;
    Ok(())
}

/// Moves the surface by `distance` (positive outwards).
pub(crate) fn offset(grid: &mut SdfGrid, distance: f32) -> Result<()> {
    if !distance.is_finite() {
        return Err(GeomError::InvalidInput(format!(
            "offset must be finite, got {distance}"
        )));
    }
    let vs = grid.voxel_size;
    if distance.abs() < 1e-6 * vs {
        return Ok(());
    }
    // the new surface must lie where the values are exact, a voxel and a
    // half inside the band
    let need = distance.abs() + super::MIN_BAND_VOXELS * vs;
    if !grid.exact || need > grid.band {
        redistance(grid, grid.band.max(need))?;
    }
    grid.store.data.par_iter_mut().for_each(|v| *v -= distance);
    redistance(grid, grid.band)
}

/// Rebuilds the store from per-block results: blocks that hold a value
/// inside `band` are kept, uniform ones become interior tiles or are
/// dropped (outside). `blocks` must be in storage order.
fn store_from(blocks: Vec<(IVec3, Combined)>, mut tiles: Vec<IVec3>) -> Store {
    let mut kept = Vec::new();
    let mut data = Vec::new();
    for (key, res) in blocks {
        match res {
            Combined::Block(v) => {
                kept.push(key);
                data.extend_from_slice(&v[..]);
            }
            Combined::Inside => tiles.push(key),
            Combined::Outside => {}
        }
    }
    Store::new(kept, data, tiles)
}

/// Classifies one block of values clamped to `band`.
fn classify(out: Box<[f32; VOXELS]>, band: f32) -> Combined {
    if out.iter().any(|v| v.abs() < band) {
        Combined::Block(out)
    } else if out[0] < 0.0 {
        Combined::Inside
    } else {
        Combined::Outside
    }
}

/// Moves the surface by `distance` (positive outwards) without
/// re-distancing: every value minus `distance`, clamped to the new half
/// width `band` (at most the old one).
pub(crate) fn shift_to(grid: &mut SdfGrid, distance: f32, band: f32) {
    let band = band.min(grid.band);
    let store = &grid.store;
    let blocks: Vec<(IVec3, Combined)> = store
        .keys
        .par_iter()
        .enumerate()
        .map(|(bi, &key)| {
            let src = store.block(bi);
            let mut out = Box::new([0f32; VOXELS]);
            for (o, &v) in out.iter_mut().zip(src) {
                *o = (v - distance).clamp(-band, band);
            }
            (key, classify(out, band))
        })
        .collect();
    grid.store = store_from(blocks, store.sorted_tiles());
    grid.band = band;
    grid.exact = false;
}

/// The solid between the level sets `inner` and `outer` (`inner < outer`):
/// `max(v − outer, inner − v)`.
pub(crate) fn shell(grid: &SdfGrid, inner: f32, outer: f32) -> Result<SdfGrid> {
    let band = grid.band;
    let reach = outer.max(-inner) + super::MIN_BAND_VOXELS * grid.voxel_size;
    if !(inner.is_finite() && outer.is_finite() && inner < outer) || reach > band {
        return Err(GeomError::InvalidInput(format!(
            "shell levels {inner}..{outer} must be ordered and inside the band {band}"
        )));
    }
    let store = &grid.store;
    // outside the stored blocks the result is outside (band > outer, band > -inner)
    let blocks: Vec<(IVec3, Combined)> = store
        .keys
        .par_iter()
        .enumerate()
        .map(|(bi, &key)| {
            let src = store.block(bi);
            let mut out = Box::new([0f32; VOXELS]);
            for (o, &v) in out.iter_mut().zip(src) {
                *o = (v - outer).max(inner - v).clamp(-band, band);
            }
            (key, classify(out, band))
        })
        .collect();
    let mut g = SdfGrid::from_store(
        grid.voxel_size,
        grid.origin,
        band,
        store_from(blocks, Vec::new()),
    );
    g.exact = false;
    Ok(g)
}

/// The union of the solid with its translates along `−Z` (the sweep of the
/// solid downwards), cut by the plane `z = floor` (world): every voxel takes
/// the smallest value at or above it in its column, then at least
/// `floor − z`.
pub(crate) fn sweep_down(grid: &mut SdfGrid, floor: f32) -> Result<()> {
    if !floor.is_finite() {
        return Err(GeomError::InvalidInput(format!(
            "sweep floor must be finite, got {floor}"
        )));
    }
    let (band, vs, origin) = (grid.band, grid.voxel_size, grid.origin);
    let store = &grid.store;
    let mut items: Vec<IVec3> = store
        .keys
        .iter()
        .copied()
        .chain(store.sorted_tiles())
        .collect();
    if items.is_empty() {
        return Ok(());
    }
    // columns of blocks, top down
    items.sort_unstable_by_key(|k| (k.x, k.y, std::cmp::Reverse(k.z)));
    let mut columns: Vec<&[IVec3]> = Vec::new();
    let mut s = 0;
    while s < items.len() {
        let mut e = s + 1;
        while e < items.len() && items[e].x == items[s].x && items[e].y == items[s].y {
            e += 1;
        }
        columns.push(&items[s..e]);
        s = e;
    }
    // voxels farther than the band below the floor are outside: the lowest block
    let low = (((floor - band - origin.z) / vs).floor() as i32).max(-store::MAX_BLOCK * SIDE) >> 3;
    let blocks: Vec<(IVec3, Combined)> = columns
        .par_iter()
        .flat_map_iter(|col| {
            let (x, y) = (col[0].x, col[0].y);
            let mut run = [f32::INFINITY; 64];
            let mut out_blocks = Vec::new();
            let lowest = col[col.len() - 1].z;
            let mut z = col[0].z;
            while z >= low {
                let key = IVec3::new(x, y, z);
                // nothing inside above and no stored block below: the rest is outside
                if z < lowest && run.iter().all(|&r| r >= band) {
                    break;
                }
                let src = store.find(key).map(|i| store.block(i));
                let bg = store.background(key, band);
                let mut out = Box::new([0f32; VOXELS]);
                for lz in (0..SIDE as usize).rev() {
                    let zw = origin.z + ((z * SIDE) as f32 + lz as f32) * vs;
                    let cut = floor - zw;
                    for (xy, r) in run.iter_mut().enumerate() {
                        let i = lz * 64 + xy;
                        *r = r.min(src.map_or(bg, |b| b[i]));
                        out[i] = r.max(cut).clamp(-band, band);
                    }
                }
                out_blocks.push((key, classify(out, band)));
                z -= 1;
            }
            out_blocks
        })
        .collect();
    let mut blocks = blocks;
    blocks.sort_unstable_by_key(|(k, _)| store::pack(*k));
    grid.store = store_from(blocks, Vec::new());
    grid.exact = false;
    Ok(())
}

#[inline]
fn smin(a: f32, b: f32, k: f32) -> f32 {
    if k <= 0.0 {
        return a.min(b);
    }
    let h = (k - (a - b).abs()).max(0.0) / k;
    a.min(b) - h * h * k * 0.25
}

impl CsgOp {
    #[inline]
    fn apply(self, a: f32, b: f32) -> f32 {
        match self {
            Self::Union => a.min(b),
            Self::Intersection => a.max(b),
            Self::Difference => a.max(-b),
            Self::SmoothUnion(k) => smin(a, b, k),
            Self::SmoothIntersection(k) => -smin(-a, -b, k),
            Self::SmoothDifference(k) => -smin(-a, b, k),
        }
    }

    fn blend(self) -> f32 {
        match self {
            Self::SmoothUnion(k) | Self::SmoothIntersection(k) | Self::SmoothDifference(k) => k,
            _ => 0.0,
        }
    }
}

/// One block of a CSG result.
enum Combined {
    Block(Box<[f32; VOXELS]>),
    Inside,
    Outside,
}

/// `grid = op(grid, other)` voxel by voxel.
pub(crate) fn csg(grid: &mut SdfGrid, other: &SdfGrid, op: CsgOp) -> Result<()> {
    if grid.voxel_size != other.voxel_size || grid.origin != other.origin {
        return Err(GeomError::InvalidInput(
            "CSG operands must share voxel size and origin".into(),
        ));
    }
    let k = op.blend();
    if !(k.is_finite() && k >= 0.0) {
        return Err(GeomError::InvalidInput(format!(
            "blend radius must be finite and not negative, got {k}"
        )));
    }
    let band = grid.band.min(other.band);
    let (a, b) = (&grid.store, &other.store);
    let (ba, bb) = (grid.band, other.band);
    let mut keys: Vec<IVec3> = a.keys.iter().chain(&b.keys).copied().collect();
    store::sort_keys(&mut keys);
    let blocks: Vec<(IVec3, Combined)> = keys
        .par_iter()
        .map(|&key| {
            let va = a.find(key).map(|i| a.block(i));
            let vb = b.find(key).map(|i| b.block(i));
            let (ga, gb) = (a.background(key, ba), b.background(key, bb));
            let mut out = Box::new([0f32; VOXELS]);
            let mut in_band = false;
            for i in 0..VOXELS {
                let x = va.map_or(ga, |v| v[i]);
                let y = vb.map_or(gb, |v| v[i]);
                let r = op.apply(x, y).clamp(-band, band);
                in_band |= r.abs() < band;
                out[i] = r;
            }
            let res = if in_band {
                Combined::Block(out)
            } else if out[0] < 0.0 {
                Combined::Inside
            } else {
                Combined::Outside
            };
            (key, res)
        })
        .collect();
    let mut tiles: Vec<IVec3> = a.sorted_tiles();
    tiles.extend(b.sorted_tiles());
    store::sort_keys(&mut tiles);
    let tiles: Vec<IVec3> = tiles
        .into_iter()
        .filter(|&t| a.find(t).is_none() && b.find(t).is_none())
        .filter(|&t| op.apply(a.background(t, ba), b.background(t, bb)) < 0.0)
        .collect();
    grid.store = store_from(blocks, tiles);
    grid.band = band;
    grid.exact = false;
    Ok(())
}

/// Fills every region of the outside that cannot be reached from far away
/// (enclosed cavities).
pub(crate) fn fill_interior(grid: &mut SdfGrid) -> Result<()> {
    let store = &grid.store;
    let tiles = store.sorted_tiles();
    let domain = Domain::new(&store.keys, &tiles)?;
    let free: Vec<Mask> = (0..store.len())
        .into_par_iter()
        .map(|bi| {
            let mut m = [0u64; 8];
            for (i, &v) in store.block(bi).iter().enumerate() {
                if v >= 0.0 {
                    flood::set(&mut m, i);
                }
            }
            m
        })
        .collect();
    let ext = flood::flood(&domain, &free, |b| !store.is_tile(b));
    let mut changed = false;
    let mut new_tiles = tiles;
    for c in 0..domain.cells.len() {
        if domain.cells[c] == Domain::NONE && !ext.out[c] {
            let b = domain.block(c);
            if !store.is_tile(b) {
                new_tiles.push(b);
                changed = true;
            }
        }
    }
    let flips: Vec<usize> = (0..store.len())
        .into_par_iter()
        .flat_map_iter(|bi| {
            let (f, e) = (free[bi], ext.ext[bi]);
            (0..VOXELS)
                .filter(move |&i| flood::get(&f, i) && !flood::get(&e, i))
                .map(move |i| bi * VOXELS + i)
        })
        .collect();
    changed |= !flips.is_empty();
    if !changed {
        return Ok(());
    }
    let mut data = std::mem::take(&mut grid.store.data);
    for i in flips {
        data[i] = -data[i].max(f32::MIN_POSITIVE);
    }
    let keys = std::mem::take(&mut grid.store.keys);
    grid.store = Store::new(keys, data, new_tiles);
    redistance(grid, grid.band)
}

/// Trilinear interpolation of the voxel values at world point `p`.
pub(crate) fn sample(grid: &SdfGrid, p: Vec3) -> f32 {
    let u = (p - grid.origin) / grid.voxel_size;
    if !u.is_finite() {
        return f32::NAN;
    }
    let f = u.floor();
    let t = u - f;
    let g = f.as_ivec3();
    let v = |dx: i32, dy: i32, dz: i32| grid.store.value(g + IVec3::new(dx, dy, dz), grid.band);
    let x00 = v(0, 0, 0) + (v(1, 0, 0) - v(0, 0, 0)) * t.x;
    let x10 = v(0, 1, 0) + (v(1, 1, 0) - v(0, 1, 0)) * t.x;
    let x01 = v(0, 0, 1) + (v(1, 0, 1) - v(0, 0, 1)) * t.x;
    let x11 = v(0, 1, 1) + (v(1, 1, 1) - v(0, 1, 1)) * t.x;
    let y0 = x00 + (x10 - x00) * t.y;
    let y1 = x01 + (x11 - x01) * t.y;
    y0 + (y1 - y0) * t.z
}

/// Number of stored voxels with a value inside the band.
pub(crate) fn band_voxels(grid: &SdfGrid) -> usize {
    let band = grid.band;
    grid.store
        .data
        .par_iter()
        .filter(|v| v.abs() < band)
        .count()
}

/// Voxel coordinates of the stored blocks (for iteration in tests and tools).
pub(crate) fn for_each_voxel(grid: &SdfGrid, mut f: impl FnMut(IVec3, f32)) {
    for (bi, &k) in grid.store.keys.iter().enumerate() {
        let base = k * SIDE;
        for (i, &v) in grid.store.block(bi).iter().enumerate() {
            f(base + local_coord(i), v);
        }
    }
}
