//! Sparse block storage of the level set: 8³ voxel blocks addressed by block
//! coordinate through a hash map, plus the set of blocks that lie entirely
//! inside the surface ("interior tiles").

use std::collections::{HashMap, HashSet};
use std::hash::{BuildHasherDefault, Hasher};

use glam::IVec3;

/// Voxels along a block edge.
pub(crate) const SIDE: i32 = 8;
/// Voxels in a block.
pub(crate) const VOXELS: usize = 512;
/// Largest block coordinate (in absolute value) accepted by the packing.
pub(crate) const MAX_BLOCK: i32 = (1 << 20) - 2;

/// Finaliser of splitmix64: a cheap hash with good low bits for packed keys.
#[derive(Default)]
pub(crate) struct KeyHasher(u64);

impl Hasher for KeyHasher {
    fn finish(&self) -> u64 {
        let mut x = self.0;
        x ^= x >> 30;
        x = x.wrapping_mul(0xbf58_476d_1ce4_e5b9);
        x ^= x >> 27;
        x = x.wrapping_mul(0x94d0_49bb_1331_11eb);
        x ^ (x >> 31)
    }

    fn write(&mut self, bytes: &[u8]) {
        for &b in bytes {
            self.0 = self.0.rotate_left(8) ^ u64::from(b);
        }
    }

    fn write_u64(&mut self, n: u64) {
        self.0 ^= n;
    }
}

/// Hash map keyed by a packed block coordinate.
pub(crate) type KeyMap<V> = HashMap<u64, V, BuildHasherDefault<KeyHasher>>;
/// Hash set of packed block coordinates.
pub(crate) type KeySet = HashSet<u64, BuildHasherDefault<KeyHasher>>;

/// Packs a block coordinate into a `u64`; the order of the packed keys is
/// z, then y, then x (the storage order of the blocks).
#[inline]
pub(crate) fn pack(b: IVec3) -> u64 {
    const OFF: i64 = 1 << 20;
    let c = |v: i32| ((i64::from(v) + OFF) as u64) & 0x1f_ffff;
    c(b.x) | (c(b.y) << 21) | (c(b.z) << 42)
}

/// Block holding voxel `g`.
#[inline]
pub(crate) fn block_of(g: IVec3) -> IVec3 {
    IVec3::new(g.x >> 3, g.y >> 3, g.z >> 3)
}

/// Index of voxel `g` inside its block (`x + 8 y + 64 z`).
#[inline]
pub(crate) fn local_of(g: IVec3) -> usize {
    ((g.x & 7) | ((g.y & 7) << 3) | ((g.z & 7) << 6)) as usize
}

/// Local voxel coordinate of block index `i`.
#[inline]
pub(crate) fn local_coord(i: usize) -> IVec3 {
    let i = i as i32;
    IVec3::new(i & 7, (i >> 3) & 7, i >> 6)
}

/// Distance `d` with the sign of its side (negative inside). An inside
/// voxel at distance zero (on a wall left inside the solid, or exactly on
/// the surface) keeps a negative value: `-0.0` would read as outside.
#[inline]
pub(crate) fn signed(d: f32, inside: bool) -> f32 {
    if inside { -d.max(f32::MIN_POSITIVE) } else { d }
}

/// Sorts block keys in storage order and removes duplicates.
pub(crate) fn sort_keys(keys: &mut Vec<IVec3>) {
    keys.sort_unstable_by_key(|&k| pack(k));
    keys.dedup();
}

/// Sparse narrow-band storage. Voxels outside the stored blocks read
/// `-band` inside an interior tile and `+band` everywhere else.
#[derive(Clone, Default)]
pub(crate) struct Store {
    /// Block coordinates, sorted by [`pack`].
    pub keys: Vec<IVec3>,
    /// `keys.len() * VOXELS` values, block after block.
    pub data: Vec<f32>,
    /// Block coordinate → position in `keys`.
    index: KeyMap<u32>,
    /// Blocks entirely inside the surface (never also in `keys`).
    tiles: KeySet,
}

impl Store {
    /// Storage from blocks already sorted by [`pack`] and interior tiles.
    pub fn new(keys: Vec<IVec3>, data: Vec<f32>, tiles: impl IntoIterator<Item = IVec3>) -> Self {
        debug_assert_eq!(keys.len() * VOXELS, data.len());
        debug_assert!(keys.windows(2).all(|w| pack(w[0]) < pack(w[1])));
        let mut index = KeyMap::default();
        index.reserve(keys.len());
        for (i, &k) in keys.iter().enumerate() {
            index.insert(pack(k), i as u32);
        }
        let tiles = tiles
            .into_iter()
            .map(pack)
            .filter(|p| !index.contains_key(p))
            .collect();
        Self {
            keys,
            data,
            index,
            tiles,
        }
    }

    /// Number of stored blocks.
    pub fn len(&self) -> usize {
        self.keys.len()
    }

    /// Number of interior tiles.
    pub fn tile_count(&self) -> usize {
        self.tiles.len()
    }

    /// Interior tiles in storage order.
    pub fn sorted_tiles(&self) -> Vec<IVec3> {
        let mut t: Vec<u64> = self.tiles.iter().copied().collect();
        t.sort_unstable();
        t.into_iter().map(unpack).collect()
    }

    /// Position of block `b` in `keys`.
    #[inline]
    pub fn find(&self, b: IVec3) -> Option<usize> {
        self.index.get(&pack(b)).map(|&i| i as usize)
    }

    /// Values of stored block number `i`.
    #[inline]
    pub fn block(&self, i: usize) -> &[f32] {
        &self.data[i * VOXELS..(i + 1) * VOXELS]
    }

    /// `true` when block `b` is an interior tile.
    #[inline]
    pub fn is_tile(&self, b: IVec3) -> bool {
        self.tiles.contains(&pack(b))
    }

    /// Value of the voxels of a block that is not stored.
    #[inline]
    pub fn background(&self, b: IVec3, band: f32) -> f32 {
        if self.is_tile(b) { -band } else { band }
    }

    /// Value of voxel `g`.
    #[inline]
    pub fn value(&self, g: IVec3, band: f32) -> f32 {
        let b = block_of(g);
        match self.find(b) {
            Some(i) => self.data[i * VOXELS + local_of(g)],
            None => self.background(b, band),
        }
    }

    /// Approximate heap size in bytes.
    pub fn approx_bytes(&self) -> usize {
        self.keys.capacity() * std::mem::size_of::<IVec3>()
            + self.data.capacity() * 4
            + self.index.capacity() * 16
            + self.tiles.capacity() * 12
    }
}

/// Inverse of [`pack`].
fn unpack(p: u64) -> IVec3 {
    const OFF: i64 = 1 << 20;
    let c = |s: u32| ((((p >> s) & 0x1f_ffff) as i64) - OFF) as i32;
    IVec3::new(c(0), c(21), c(42))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pack_round_trip_and_order() {
        for b in [
            IVec3::new(0, 0, 0),
            IVec3::new(-5, 7, -9),
            IVec3::new(MAX_BLOCK, -MAX_BLOCK, 3),
        ] {
            assert_eq!(unpack(pack(b)), b);
        }
        assert!(pack(IVec3::new(5, 0, 0)) < pack(IVec3::new(0, 1, 0)));
        assert!(pack(IVec3::new(5, 5, 0)) < pack(IVec3::new(0, 0, 1)));
        assert_eq!(
            local_coord(local_of(IVec3::new(-1, 9, 23))),
            IVec3::new(7, 1, 7)
        );
        assert_eq!(block_of(IVec3::new(-1, 9, 23)), IVec3::new(-1, 1, 2));
    }

    #[test]
    fn inside_zero_stays_negative() {
        assert!(signed(0.0, true) < 0.0);
        assert!(signed(0.0, false) >= 0.0 && !signed(0.0, false).is_sign_negative());
        assert_eq!(signed(0.25, true), -0.25);
        assert_eq!(signed(0.25, false), 0.25);
    }
}

#[cfg(kani)]
mod kani_proofs {
    use super::*;

    /// Range of a block coordinate the 21-bit packing represents.
    const LO: i32 = -(1 << 20);
    const HI: i32 = (1 << 20) - 1;

    fn any_block() -> IVec3 {
        let b = IVec3::new(kani::any(), kani::any(), kani::any());
        kani::assume(b.cmpge(IVec3::splat(LO)).all() && b.cmple(IVec3::splat(HI)).all());
        b
    }

    // Every block coordinate within ±2^20 (a superset of ±MAX_BLOCK) packs
    // and unpacks to itself.
    #[kani::proof]
    fn pack_round_trips_in_range() {
        let b = any_block();
        assert_eq!(unpack(pack(b)), b);
    }

    // Packed keys sort like the coordinates compared z first, then y, then
    // x: the storage order that `Store::new` and `sort_keys` rely on (and
    // distinct blocks never share a key).
    #[kani::proof]
    fn pack_orders_by_z_then_y_then_x() {
        let (a, b) = (any_block(), any_block());
        assert_eq!(pack(a).cmp(&pack(b)), (a.z, a.y, a.x).cmp(&(b.z, b.y, b.x)));
    }

    // Any voxel is its block's origin plus its local coordinate, and its
    // local index addresses one of the block's voxels.
    #[kani::proof]
    fn voxel_splits_into_block_and_local_index() {
        let g = IVec3::new(kani::any(), kani::any(), kani::any());
        let i = local_of(g);
        assert!(i < VOXELS);
        assert_eq!(local_coord(i), g & IVec3::splat(SIDE - 1));
        assert_eq!(block_of(g) * SIDE + local_coord(i), g);
    }

    // Every local index of a block maps to a local coordinate and back.
    #[kani::proof]
    fn local_index_round_trips() {
        let i: usize = kani::any();
        kani::assume(i < VOXELS);
        let l = local_coord(i);
        assert!(l.cmpge(IVec3::ZERO).all() && l.cmplt(IVec3::splat(SIDE)).all());
        assert_eq!(local_of(l), i);
    }

    // A voxel on the inside always reads as inside (strictly negative), for
    // any distance, zero and NaN included.
    #[kani::proof]
    fn inside_is_always_negative() {
        let d: f32 = kani::any();
        assert!(signed(d, true) < 0.0);
    }
}
