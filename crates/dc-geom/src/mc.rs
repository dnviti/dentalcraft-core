//! Marching cubes over sparse 8³ voxel blocks with unknown voxels, shared by
//! the TSDF fusion ([`crate::tsdf`]) and the point cloud mesher
//! ([`crate::meshing`]).
//!
//! The field is a set of blocks of 8³ samples addressed by block coordinate;
//! voxel `g` (global integer coordinates) sits at `origin + g * voxel`. A
//! sample is unknown when it is NaN or when its block is missing: a cell
//! (eight neighbouring samples) is meshed only when its eight corners are
//! known, so the output is an open surface that stops where the field was not
//! observed.
//!
//! The cell topology is the face-consistent construction of the closed SDF
//! mesher (`sdf::mesher`): one vertex per grid edge whose ends have opposite
//! signs (inside: value below zero), linear interpolation along the edge,
//! crossing points joined face by face into oriented loops (ambiguous faces
//! resolved by the asymptotic decider, which depends only on the face values,
//! so both cells sharing a face agree), loops fanned into triangles that
//! avoid diagonals lying on a cell face. Vertices are keyed by their grid edge
//! and welded across blocks, so the surface is crack-free; every edge is used
//! by at most two triangles with opposite directions, and faces are
//! counter-clockwise seen from the positive side.
//!
//! Blocks are processed in parallel and merged in sorted key order: the
//! output does not depend on the thread count.

use std::collections::HashMap;
use std::hash::{BuildHasherDefault, Hasher};

use glam::{IVec3, Vec3};
use rayon::prelude::*;

/// Voxels along a block edge.
pub(crate) const SIDE: i32 = 8;
/// Voxels in a block.
pub(crate) const VOXELS: usize = 512;
/// Largest absolute global voxel coordinate supported by the edge keys.
pub(crate) const MAX_VOXEL: i32 = (1 << 19) - 2;

/// splitmix64 finaliser over the written integers: a cheap hash for block keys.
#[derive(Default, Clone, Copy)]
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

    fn write_i32(&mut self, n: i32) {
        self.0 = self.0.rotate_left(21) ^ u64::from(n as u32);
    }

    fn write_u64(&mut self, n: u64) {
        self.0 ^= n;
    }
}

/// Hash map keyed by block coordinate.
pub(crate) type BlockMap<T> = HashMap<IVec3, T, BuildHasherDefault<KeyHasher>>;

/// Block containing global voxel `g`.
#[inline]
pub(crate) fn block_of(g: IVec3) -> IVec3 {
    IVec3::new(
        g.x.div_euclid(SIDE),
        g.y.div_euclid(SIDE),
        g.z.div_euclid(SIDE),
    )
}

/// Index inside its block of global voxel `g`.
#[inline]
pub(crate) fn local_index(g: IVec3) -> usize {
    let l = IVec3::new(
        g.x.rem_euclid(SIDE),
        g.y.rem_euclid(SIDE),
        g.z.rem_euclid(SIDE),
    );
    (l.x + SIDE * (l.y + SIDE * l.z)) as usize
}

/// Local voxel coordinates of index `i` inside a block.
#[inline]
pub(crate) fn local_coords(i: usize) -> IVec3 {
    let i = i as i32;
    IVec3::new(i % SIDE, (i / SIDE) % SIDE, i / (SIDE * SIDE))
}

/// Sorts block keys in the canonical (z, y, x) order.
pub(crate) fn sort_keys(keys: &mut [IVec3]) {
    keys.sort_unstable_by_key(|k| (k.z, k.y, k.x));
}

/// Packs a block key into a `u64` that sorts like the key (21 bits per axis).
#[inline]
pub(crate) fn pack(k: IVec3) -> u64 {
    const OFF: i64 = 1 << 20;
    let c = |v: i32| ((i64::from(v) + OFF) as u64) & 0x1F_FFFF;
    (c(k.x) << 42) | (c(k.y) << 21) | c(k.z)
}

/// Inverse of [`pack`].
#[inline]
pub(crate) fn unpack(p: u64) -> IVec3 {
    const OFF: i64 = 1 << 20;
    let c = |s: u32| (((p >> s) & 0x1F_FFFF) as i64 - OFF) as i32;
    IVec3::new(c(42), c(21), c(0))
}

/// Sparse scalar field: sorted block keys, their index map and values
/// (NaN = unknown).
pub(crate) struct Field<'a> {
    /// Block keys in canonical order (see [`sort_keys`]).
    pub keys: &'a [IVec3],
    /// Block key to index into `values`.
    pub map: &'a BlockMap<usize>,
    /// Samples of each block, x fastest.
    pub values: &'a [[f32; VOXELS]],
    /// World position of global voxel `(0, 0, 0)`.
    pub origin: Vec3,
    /// Voxel edge length.
    pub voxel: f32,
    /// A cell with a sign change larger than this across one of its edges is
    /// skipped (rejects the jump between two truncated TSDF sides);
    /// `f32::INFINITY` disables the check.
    pub max_step: f32,
}

/// Samples of a block and its +1 neighbours.
const CUBE: usize = 9;
/// Smallest interpolation parameter along an edge (no zero-length edges).
const T_MIN: f32 = 1e-3;
/// Flag of the keys of added loop centre vertices.
const EXTRA: u64 = 1 << 63;

/// Corners of each cell face, counter-clockwise seen from outside the cell.
/// Corner `c` sits at offset `(c & 1, (c >> 1) & 1, c >> 2)`.
const FACES: [[u8; 4]; 6] = [
    [0, 4, 6, 2],
    [1, 3, 7, 5],
    [0, 1, 5, 4],
    [2, 6, 7, 3],
    [0, 2, 3, 1],
    [4, 5, 7, 6],
];

/// Cell edge (0..12) joining two corners: `axis * 4 + other two bits`.
const fn edge_id(a: u8, b: u8) -> u8 {
    let axis = (a ^ b).trailing_zeros() as u8;
    let lo = a & b;
    let rest = match axis {
        0 => (lo >> 1) & 3,
        1 => (lo & 1) | (((lo >> 2) & 1) << 1),
        _ => lo & 3,
    };
    axis * 4 + rest
}

/// Lower corner and axis of each cell edge.
const EDGES: [(u8, u8); 12] = {
    let mut e = [(0u8, 0u8); 12];
    let mut c = 0u8;
    while c < 8 {
        let mut axis = 0u8;
        while axis < 3 {
            if c & (1 << axis) == 0 {
                e[edge_id(c, c | (1 << axis)) as usize] = (c, axis);
            }
            axis += 1;
        }
        c += 1;
    }
    e
};

/// Cell edge of each side of each face (side `k` joins corners `k`, `k + 1`).
const FACE_EDGES: [[u8; 4]; 6] = {
    let mut t = [[0u8; 4]; 6];
    let mut f = 0;
    while f < 6 {
        let mut k = 0;
        while k < 4 {
            t[f][k] = edge_id(FACES[f][k], FACES[f][(k + 1) % 4]);
            k += 1;
        }
        f += 1;
    }
    t
};

/// Faces of the cell (bits, in [`FACES`] order) holding each cell edge.
const EDGE_FACES: [u8; 12] = {
    let mut t = [0u8; 12];
    let mut f = 0;
    while f < 6 {
        let mut k = 0;
        while k < 4 {
            t[FACE_EDGES[f][k] as usize] |= 1 << f;
            k += 1;
        }
        f += 1;
    }
    t
};

#[inline]
fn corner(c: u8) -> IVec3 {
    IVec3::new(i32::from(c & 1), i32::from((c >> 1) & 1), i32::from(c >> 2))
}

#[inline]
fn axis_vec(axis: u8) -> IVec3 {
    IVec3::new(
        i32::from(axis == 0),
        i32::from(axis == 1),
        i32::from(axis == 2),
    )
}

#[inline]
fn cube_index(c: IVec3) -> usize {
    c.x as usize + CUBE * (c.y as usize + CUBE * c.z as usize)
}

/// Interpolation parameter of the crossing between values `a` and `b`.
#[inline]
fn crossing(a: f32, b: f32) -> f32 {
    (a / (a - b)).clamp(T_MIN, 1.0 - T_MIN)
}

/// Key of the grid edge from global voxel `g` along `axis`.
#[inline]
fn edge_key(g: IVec3, axis: u8) -> u64 {
    const OFF: i64 = 1 << 19;
    let c = |v: i32| ((i64::from(v) + OFF) as u64) & 0xF_FFFF;
    (c(g.x) << 42) | (c(g.y) << 22) | (c(g.z) << 2) | u64::from(axis)
}

/// Samples of the 9³ voxels from block `key` (NaN where unknown); `None`
/// when the known samples do not change sign.
fn gather(field: &Field, key: IVec3) -> Option<Box<[f32; CUBE * CUBE * CUBE]>> {
    let mut cube = Box::new([f32::NAN; CUBE * CUBE * CUBE]);
    for oz in 0..2 {
        for oy in 0..2 {
            for ox in 0..2 {
                let o = IVec3::new(ox, oy, oz);
                let Some(&bi) = field.map.get(&(key + o)) else {
                    continue;
                };
                let src = &field.values[bi];
                let lo = o * SIDE;
                let hi = (lo + IVec3::splat(SIDE)).min(IVec3::splat(CUBE as i32));
                for z in lo.z..hi.z {
                    for y in lo.y..hi.y {
                        for x in lo.x..hi.x {
                            let c = IVec3::new(x, y, z);
                            let l = c - lo;
                            cube[cube_index(c)] = src[(l.x + SIDE * (l.y + SIDE * l.z)) as usize];
                        }
                    }
                }
            }
        }
    }
    let mut neg = false;
    let mut pos = false;
    for &v in cube.iter() {
        if v < 0.0 {
            neg = true;
        } else if v >= 0.0 {
            pos = true;
        }
    }
    (neg && pos).then_some(cube)
}

/// Triangles (edge keys or [`EXTRA`] centre indices) and vertex positions
/// produced by one block.
#[derive(Default)]
struct BlockOut {
    tris: Vec<[u64; 3]>,
    verts: Vec<(u64, Vec3)>,
    extra: Vec<Vec3>,
}

fn block_triangles(field: &Field, cube: &[f32], key: IVec3) -> BlockOut {
    let mut out = BlockOut::default();
    let base = key * SIDE;
    let to_world = |g: Vec3| field.origin + g * field.voxel;
    for z in 0..SIDE {
        for y in 0..SIDE {
            for x in 0..SIDE {
                let cell = IVec3::new(x, y, z);
                let mut val = [0f32; 8];
                let mut mask = 0u8;
                let mut known = true;
                for c in 0..8u8 {
                    let v = cube[cube_index(cell + corner(c))];
                    known &= v.is_finite();
                    val[c as usize] = v;
                    if v < 0.0 {
                        mask |= 1 << c;
                    }
                }
                if !known || mask == 0 || mask == 0xff {
                    continue;
                }
                let inside = |c: u8| mask & (1 << c) != 0;
                let jump = EDGES.iter().any(|&(c, axis)| {
                    let (a, b) = (val[c as usize], val[(c | (1 << axis)) as usize]);
                    (a < 0.0) != (b < 0.0) && (a - b).abs() > field.max_step
                });
                if jump {
                    continue;
                }
                // next[e]: the crossing edge that follows e on its loop
                let mut next = [u8::MAX; 12];
                for f in 0..6 {
                    let q = FACES[f];
                    let e = FACE_EDGES[f];
                    let exits: [bool; 4] =
                        std::array::from_fn(|k| inside(q[k]) && !inside(q[(k + 1) % 4]));
                    let entries: [bool; 4] =
                        std::array::from_fn(|k| !inside(q[k]) && inside(q[(k + 1) % 4]));
                    let n = exits.iter().filter(|&&b| b).count();
                    if n == 1 {
                        let a = exits.iter().position(|&b| b).unwrap_or(0);
                        let b = entries.iter().position(|&b| b).unwrap_or(0);
                        next[e[a] as usize] = e[b];
                    } else if n == 2 {
                        // asymptotic decider: the inside corners are joined
                        // through the face when their product is the larger
                        let (i0, i1, o0, o1) = if inside(q[0]) {
                            (q[0], q[2], q[1], q[3])
                        } else {
                            (q[1], q[3], q[0], q[2])
                        };
                        let joined = val[i0 as usize] * val[i1 as usize]
                            > val[o0 as usize] * val[o1 as usize];
                        for k in 0..4 {
                            if exits[k] {
                                let to = if joined { (k + 1) % 4 } else { (k + 3) % 4 };
                                next[e[k] as usize] = e[to];
                            }
                        }
                    }
                }
                let mut seen = 0u16;
                for start in 0..12u8 {
                    if next[start as usize] == u8::MAX || seen & (1 << start) != 0 {
                        continue;
                    }
                    let mut ids = [0u64; 12];
                    let mut edges = [0u8; 12];
                    let mut pos = [Vec3::ZERO; 12];
                    let mut n = 0;
                    let mut e = start;
                    loop {
                        seen |= 1 << e;
                        edges[n] = e;
                        let (c, axis) = EDGES[e as usize];
                        let a = val[c as usize];
                        let b = val[(c | (1 << axis)) as usize];
                        pos[n] = corner(c).as_vec3() + axis_vec(axis).as_vec3() * crossing(a, b);
                        let g = base + cell + corner(c);
                        ids[n] = edge_key(g, axis);
                        out.verts
                            .push((ids[n], to_world((base + cell).as_vec3() + pos[n])));
                        n += 1;
                        e = next[e as usize];
                        if e == start || e == u8::MAX || n == 12 {
                            break;
                        }
                    }
                    if n < 3 {
                        continue;
                    }
                    let extra = &mut out.extra;
                    triangulate(
                        &ids[..n],
                        &edges[..n],
                        &pos[..n],
                        &mut out.tris,
                        &mut || {
                            let c = pos[..n].iter().copied().sum::<Vec3>() / n as f32;
                            extra.push(to_world((base + cell).as_vec3() + c));
                            EXTRA | (extra.len() as u64 - 1)
                        },
                    );
                }
            }
        }
    }
    out
}

/// Triangulates one loop (inside on its left seen from outside) with
/// outward faces. A fan diagonal joining two vertices of the same cell face
/// could also be used by the neighbouring cell, so the apex is chosen to
/// avoid such diagonals; when no vertex qualifies a centre vertex
/// (`centre()`) is added.
fn triangulate(
    ids: &[u64],
    edges: &[u8],
    pos: &[Vec3],
    out: &mut Vec<[u64; 3]>,
    centre: &mut impl FnMut() -> u64,
) {
    let n = ids.len();
    let safe =
        |a: usize, b: usize| EDGE_FACES[edges[a] as usize] & EDGE_FACES[edges[b] as usize] == 0;
    let apex_ok = |a: usize| (2..n - 1).all(|k| safe(a, (a + k) % n));
    let apex = if n == 4 {
        // the shorter safe diagonal
        let d0 = pos[0].distance_squared(pos[2]);
        let d1 = pos[1].distance_squared(pos[3]);
        match (apex_ok(0), apex_ok(1)) {
            (true, true) => Some(if d0 > d1 { 1 } else { 0 }),
            (true, false) => Some(0),
            (false, true) => Some(1),
            (false, false) => None,
        }
    } else {
        (0..n).find(|&a| apex_ok(a))
    };
    match apex {
        Some(a) => {
            for k in 1..n - 1 {
                out.push([ids[a], ids[(a + k + 1) % n], ids[(a + k) % n]]);
            }
        }
        None => {
            let c = centre();
            for k in 0..n {
                out.push([c, ids[(k + 1) % n], ids[k]]);
            }
        }
    }
}

/// Zero level set of `field`: welded positions and triangles
/// (counter-clockwise seen from the positive side).
pub(crate) fn extract(field: &Field) -> (Vec<Vec3>, Vec<[u32; 3]>) {
    // cells with their lower corner in a stored block (cells reaching into
    // a stored block from a missing one have unknown corners)
    let outs: Vec<BlockOut> = field
        .keys
        .par_iter()
        .map(|&k| match gather(field, k) {
            Some(cube) => block_triangles(field, &cube[..], k),
            None => BlockOut::default(),
        })
        .collect();
    let mut index: HashMap<u64, u32, BuildHasherDefault<KeyHasher>> = HashMap::default();
    let mut positions = Vec::new();
    let mut tris = Vec::new();
    for o in outs {
        for (key, p) in o.verts {
            index.entry(key).or_insert_with(|| {
                positions.push(p);
                positions.len() as u32 - 1
            });
        }
        let extra_base = positions.len() as u32;
        positions.extend(o.extra);
        tris.extend(o.tris.into_iter().map(|t| {
            t.map(|k| {
                if k & EXTRA != 0 {
                    extra_base + (k & !EXTRA) as u32
                } else {
                    index[&k]
                }
            })
        }));
    }
    (positions, tris)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Field of `f` sampled on the blocks covering `[lo, hi]` (voxel coords),
    /// NaN where `known` is false.
    fn field_of(
        lo: IVec3,
        hi: IVec3,
        f: impl Fn(Vec3) -> f32,
        known: impl Fn(Vec3) -> bool,
    ) -> (Vec<IVec3>, BlockMap<usize>, Vec<[f32; VOXELS]>) {
        let (bl, bh) = (block_of(lo), block_of(hi));
        let mut keys = Vec::new();
        for z in bl.z..=bh.z {
            for y in bl.y..=bh.y {
                for x in bl.x..=bh.x {
                    keys.push(IVec3::new(x, y, z));
                }
            }
        }
        sort_keys(&mut keys);
        let mut map = BlockMap::default();
        let mut values = Vec::new();
        for (i, &k) in keys.iter().enumerate() {
            map.insert(k, i);
            let mut v = [0f32; VOXELS];
            for (j, s) in v.iter_mut().enumerate() {
                let p = (k * SIDE + local_coords(j)).as_vec3();
                *s = if known(p) { f(p) } else { f32::NAN };
            }
            values.push(v);
        }
        (keys, map, values)
    }

    #[test]
    fn sphere_is_closed_manifold_and_outward() {
        let (keys, map, values) = field_of(
            IVec3::splat(-10),
            IVec3::splat(10),
            |p| p.length() - 6.3,
            |_| true,
        );
        let field = Field {
            keys: &keys,
            map: &map,
            values: &values,
            origin: Vec3::ZERO,
            voxel: 0.5,
            max_step: f32::INFINITY,
        };
        let (pos, tris) = extract(&field);
        let mesh = dc_mesh::TriMesh::new(pos, tris);
        let adj = mesh.edge_adjacency();
        assert_eq!(adj.boundary_edge_count(), 0);
        assert_eq!(adj.non_manifold_edge_count(), 0);
        for p in &mesh.positions {
            assert!((p.length() - 3.15).abs() < 0.05, "{p}");
        }
        // outward: positive signed volume
        let vol: f32 = (0..mesh.face_count())
            .map(|f| {
                let [a, b, c] = mesh.triangle(f);
                a.dot(b.cross(c)) / 6.0
            })
            .sum();
        assert!(vol > 0.0);
    }

    #[test]
    fn unknown_samples_cut_an_open_surface() {
        let (keys, map, values) = field_of(
            IVec3::splat(-10),
            IVec3::splat(10),
            |p| p.z - 0.4,
            |p| p.x.abs() <= 6.0 && p.y.abs() <= 6.0,
        );
        let field = Field {
            keys: &keys,
            map: &map,
            values: &values,
            origin: Vec3::ZERO,
            voxel: 1.0,
            max_step: f32::INFINITY,
        };
        let (pos, tris) = extract(&field);
        let mesh = dc_mesh::TriMesh::new(pos, tris);
        let adj = mesh.edge_adjacency();
        assert_eq!(adj.non_manifold_edge_count(), 0);
        assert_eq!(mesh.boundary_loops().len(), 1);
        assert!((mesh.area() - 144.0).abs() < 1e-3);
        for f in 0..mesh.face_count() {
            assert!(mesh.face_normal(f).z > 0.99);
        }
    }
}
