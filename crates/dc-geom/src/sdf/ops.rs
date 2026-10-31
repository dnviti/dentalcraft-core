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
                vals[i] = if inside { -d } else { d };
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
    let mut tiles: Vec<IVec3> = tiles
        .into_iter()
        .filter(|&t| a.find(t).is_none() && b.find(t).is_none())
        .filter(|&t| op.apply(a.background(t, ba), b.background(t, bb)) < 0.0)
        .collect();
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
    grid.store = Store::new(kept, data, tiles);
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
