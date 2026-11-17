//! Two-level flood fill from outside: whole blocks between the stored ones,
//! voxels (bit-parallel, 512-bit masks) inside the stored ones.
//!
//! This is the exterior trace of OpenVDB's `meshToLevelSet`
//! (`traceExteriorBoundaries`, reproduced by `gpu_sdf.py`): the outside is
//! what can be reached from the border of the domain without crossing a
//! barrier voxel, so holes in a scan narrower than the barrier stay closed.

use std::collections::VecDeque;

use glam::IVec3;

use crate::{GeomError, Result};

/// One bit per voxel of a block: word `z`, bit `x + 8 y`.
pub(crate) type Mask = [u64; 8];

const X0: u64 = 0x0101_0101_0101_0101;
const X7: u64 = X0 << 7;
const Y0: u64 = 0xff;
const Y7: u64 = 0xff << 56;
/// Largest number of blocks in a flood domain.
const MAX_CELLS: i64 = 1 << 31;

/// The six face directions.
pub(crate) const DIRS: [IVec3; 6] = [
    IVec3::new(-1, 0, 0),
    IVec3::new(1, 0, 0),
    IVec3::new(0, -1, 0),
    IVec3::new(0, 1, 0),
    IVec3::new(0, 0, -1),
    IVec3::new(0, 0, 1),
];

#[inline]
pub(crate) fn get(m: &Mask, i: usize) -> bool {
    (m[i >> 6] >> (i & 63)) & 1 != 0
}

#[inline]
pub(crate) fn set(m: &mut Mask, i: usize) {
    m[i >> 6] |= 1 << (i & 63);
}

#[inline]
fn is_zero(m: &Mask) -> bool {
    m.iter().all(|&w| w == 0)
}

#[inline]
fn and_not(a: &Mask, b: &Mask) -> Mask {
    std::array::from_fn(|i| a[i] & !b[i])
}

#[inline]
fn and(a: &Mask, b: &Mask) -> Mask {
    std::array::from_fn(|i| a[i] & b[i])
}

/// Voxels of the face of a block in direction `d` (index into [`DIRS`]).
fn face(d: usize) -> Mask {
    match d {
        0 => [X0; 8],
        1 => [X7; 8],
        2 => [Y0; 8],
        3 => [Y7; 8],
        4 => [u64::MAX, 0, 0, 0, 0, 0, 0, 0],
        _ => [0, 0, 0, 0, 0, 0, 0, u64::MAX],
    }
}

/// Face voxels of a block in direction `d` moved onto the opposite face of
/// the neighbouring block in that direction.
fn transfer(m: &Mask, d: usize) -> Mask {
    match d {
        0 => m.map(|w| (w & X0) << 7),
        1 => m.map(|w| (w & X7) >> 7),
        2 => m.map(|w| (w & Y0) << 56),
        3 => m.map(|w| (w & Y7) >> 56),
        4 => [0, 0, 0, 0, 0, 0, 0, m[0]],
        _ => [m[7], 0, 0, 0, 0, 0, 0, 0],
    }
}

/// Six-connected dilation of `m` inside one block.
fn dilate(m: &Mask) -> Mask {
    std::array::from_fn(|z| {
        let w = m[z];
        let mut r = w | ((w << 1) & !X0) | ((w >> 1) & !X7) | (w << 8) | (w >> 8);
        if z > 0 {
            r |= m[z - 1];
        }
        if z < 7 {
            r |= m[z + 1];
        }
        r
    })
}

/// Dense box of blocks `[lo, lo + dims)`; each cell holds the index of a
/// stored block or [`Domain::NONE`].
pub(crate) struct Domain {
    pub lo: IVec3,
    pub dims: IVec3,
    pub cells: Vec<u32>,
}

impl Domain {
    pub const NONE: u32 = u32::MAX;

    /// Domain covering `keys` (stored blocks, indexed in order) and `extra`
    /// blocks, with one empty block of margin on every side.
    pub fn new(keys: &[IVec3], extra: &[IVec3]) -> Result<Self> {
        let mut lo = IVec3::splat(i32::MAX);
        let mut hi = IVec3::splat(i32::MIN);
        for &k in keys.iter().chain(extra) {
            lo = lo.min(k);
            hi = hi.max(k);
        }
        if keys.is_empty() && extra.is_empty() {
            lo = IVec3::ZERO;
            hi = IVec3::ZERO;
        }
        lo -= IVec3::ONE;
        hi += IVec3::ONE;
        let dims = hi - lo + IVec3::ONE;
        let n = i64::from(dims.x) * i64::from(dims.y) * i64::from(dims.z);
        if n > MAX_CELLS {
            return Err(GeomError::InvalidInput(format!(
                "level set domain too large ({n} blocks)"
            )));
        }
        let mut d = Self {
            lo,
            dims,
            cells: vec![Self::NONE; n as usize],
        };
        for (i, &k) in keys.iter().enumerate() {
            let c = d.index(k).expect("key inside its domain");
            d.cells[c] = i as u32;
        }
        Ok(d)
    }

    /// Cell index of block `b`, `None` outside the domain.
    #[inline]
    pub fn index(&self, b: IVec3) -> Option<usize> {
        let r = b - self.lo;
        if r.cmplt(IVec3::ZERO).any() || r.cmpge(self.dims).any() {
            return None;
        }
        Some(
            r.x as usize
                + self.dims.x as usize * (r.y as usize + self.dims.y as usize * r.z as usize),
        )
    }

    /// Block of cell index `c`.
    #[inline]
    pub fn block(&self, c: usize) -> IVec3 {
        let nx = self.dims.x as usize;
        let ny = self.dims.y as usize;
        self.lo
            + IVec3::new(
                (c % nx) as i32,
                ((c / nx) % ny) as i32,
                (c / (nx * ny)) as i32,
            )
    }
}

/// Result of [`flood`].
pub(crate) struct Exterior {
    /// Reached voxels of every stored block.
    pub ext: Vec<Mask>,
    /// Reached cells (meaningful for cells without a stored block).
    pub out: Vec<bool>,
}

impl Exterior {
    /// `true` when voxel `g` was reached (outside the domain: always).
    #[inline]
    pub fn reached(&self, domain: &Domain, g: IVec3) -> bool {
        let b = super::store::block_of(g);
        match domain.index(b) {
            None => true,
            Some(c) => match domain.cells[c] {
                Domain::NONE => self.out[c],
                i => get(&self.ext[i as usize], super::store::local_of(g)),
            },
        }
    }
}

/// Floods from the border of `domain` through the `free` voxels of the
/// stored blocks and the cells without a stored block for which `passable`
/// holds.
pub(crate) fn flood(domain: &Domain, free: &[Mask], passable: impl Fn(IVec3) -> bool) -> Exterior {
    let n = domain.cells.len();
    let mut out = vec![false; n];
    let mut ext = vec![[0u64; 8]; free.len()];
    let mut pending = vec![[0u64; 8]; free.len()];
    let mut queued = vec![false; n];
    let mut queue = VecDeque::new();
    let dims = domain.dims;
    for c in 0..n {
        let r = domain.block(c) - domain.lo;
        let border = r.cmpeq(IVec3::ZERO).any() || r.cmpeq(dims - IVec3::ONE).any();
        if !border {
            continue;
        }
        match domain.cells[c] {
            Domain::NONE => {
                if passable(domain.block(c)) {
                    out[c] = true;
                    queued[c] = true;
                    queue.push_back(c);
                }
            }
            i => {
                // a stored block on the border: its outer faces are seeds
                let i = i as usize;
                for (d, dir) in DIRS.iter().enumerate() {
                    if domain.index(domain.block(c) + *dir).is_none() {
                        let s = and(&face(d), &free[i]);
                        for w in 0..8 {
                            pending[i][w] |= s[w];
                        }
                    }
                }
                queued[c] = true;
                queue.push_back(c);
            }
        }
    }
    while let Some(c) = queue.pop_front() {
        queued[c] = false;
        let b = domain.block(c);
        match domain.cells[c] {
            Domain::NONE => {
                for (d, dir) in DIRS.iter().enumerate() {
                    let Some(nc) = domain.index(b + *dir) else {
                        continue;
                    };
                    match domain.cells[nc] {
                        Domain::NONE => {
                            if !out[nc] && passable(b + *dir) {
                                out[nc] = true;
                                queued[nc] = true;
                                queue.push_back(nc);
                            }
                        }
                        ni => {
                            let ni = ni as usize;
                            // the face of the neighbour that touches this cell
                            let s = and_not(&and(&face(d ^ 1), &free[ni]), &ext[ni]);
                            if !is_zero(&s) {
                                for w in 0..8 {
                                    pending[ni][w] |= s[w];
                                }
                                if !queued[nc] {
                                    queued[nc] = true;
                                    queue.push_back(nc);
                                }
                            }
                        }
                    }
                }
            }
            i => {
                let i = i as usize;
                let seeds = and_not(&and(&pending[i], &free[i]), &ext[i]);
                pending[i] = [0; 8];
                if is_zero(&seeds) {
                    continue;
                }
                let before = ext[i];
                let mut cur = seeds;
                let mut e = before;
                for w in 0..8 {
                    e[w] |= cur[w];
                }
                loop {
                    let grow = and_not(&and(&dilate(&cur), &free[i]), &e);
                    if is_zero(&grow) {
                        break;
                    }
                    for w in 0..8 {
                        e[w] |= grow[w];
                    }
                    cur = grow;
                }
                ext[i] = e;
                let new = and_not(&e, &before);
                for (d, dir) in DIRS.iter().enumerate() {
                    let f = and(&new, &face(d));
                    if is_zero(&f) {
                        continue;
                    }
                    let Some(nc) = domain.index(b + *dir) else {
                        continue;
                    };
                    match domain.cells[nc] {
                        Domain::NONE => {
                            if !out[nc] && passable(b + *dir) {
                                out[nc] = true;
                                queued[nc] = true;
                                queue.push_back(nc);
                            }
                        }
                        ni => {
                            let ni = ni as usize;
                            let s = and_not(&and(&transfer(&f, d), &free[ni]), &ext[ni]);
                            if !is_zero(&s) {
                                for w in 0..8 {
                                    pending[ni][w] |= s[w];
                                }
                                if !queued[nc] {
                                    queued[nc] = true;
                                    queue.push_back(nc);
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    Exterior { ext, out }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sdf::store::{local_coord, local_of};

    #[test]
    fn dilate_and_transfer_stay_in_block() {
        let mut m = [0u64; 8];
        set(&mut m, local_of(IVec3::new(7, 0, 0)));
        let d = dilate(&m);
        assert!(get(&d, local_of(IVec3::new(6, 0, 0))));
        assert!(!get(&d, local_of(IVec3::new(0, 1, 0))));
        let t = transfer(&and(&m, &face(1)), 1);
        assert_eq!(local_coord(t[0].trailing_zeros() as usize), IVec3::ZERO);
    }
}

#[cfg(kani)]
mod kani_proofs {
    use super::*;
    use crate::sdf::store::MAX_BLOCK;

    /// Largest domain edge in blocks (keeps the division checks small).
    const MAX_DIM: i32 = 8;

    /// Domain whose low corner is that of stored blocks (within
    /// ±(MAX_BLOCK + 1), margin included) and whose edges hold up to
    /// `MAX_DIM` blocks. `index` and `block` never read `cells`.
    fn any_domain() -> Domain {
        let lo = IVec3::new(kani::any(), kani::any(), kani::any());
        let dims = IVec3::new(kani::any(), kani::any(), kani::any());
        let r = IVec3::splat(MAX_BLOCK + 1);
        kani::assume(lo.cmpge(-r).all() && lo.cmple(r).all());
        kani::assume(dims.cmpge(IVec3::ONE).all() && dims.cmple(IVec3::splat(MAX_DIM)).all());
        Domain {
            lo,
            dims,
            cells: Vec::new(),
        }
    }

    fn cell_count(d: &Domain) -> usize {
        (d.dims.x * d.dims.y * d.dims.z) as usize
    }

    // The block of any voxel (any `i32` coordinates) is inside the domain
    // exactly when `index` finds it, at a cell index below the cell count
    // that maps back to the same block.
    #[kani::proof]
    fn domain_index_then_block_is_identity() {
        let d = any_domain();
        let g = IVec3::new(kani::any(), kani::any(), kani::any());
        let b = crate::sdf::store::block_of(g);
        let inside = b.cmpge(d.lo).all() && b.cmplt(d.lo + d.dims).all();
        match d.index(b) {
            Some(c) => {
                assert!(inside);
                assert!(c < cell_count(&d));
                assert_eq!(d.block(c), b);
            }
            None => assert!(!inside),
        }
    }

    // Every cell index of the domain is the index of its own block.
    #[kani::proof]
    fn domain_block_then_index_is_identity() {
        let d = any_domain();
        let c: usize = kani::any();
        kani::assume(c < cell_count(&d));
        assert_eq!(d.index(d.block(c)), Some(c));
    }
}
