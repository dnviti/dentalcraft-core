//! Iso-surface extraction over the sparse blocks (marching cubes with
//! face-consistent topology).
//!
//! One vertex per grid edge whose end voxels have opposite signs (inside:
//! value below the iso level), placed by linear interpolation. Inside each
//! cell the surface is built from its faces: on every face the crossing
//! points are joined into oriented segments (an ambiguous face, two diagonal
//! corners inside, is resolved by the asymptotic decider, which depends only
//! on the four values of the face, so both cells sharing it agree), the
//! segments are chained into closed loops and each loop is triangulated as a
//! fan. Every mesh edge is then shared by exactly two triangles in opposite
//! directions: the result is closed, manifold and consistently oriented
//! (counter-clockwise seen from outside).
//!
//! Blocks are meshed in parallel; vertices are numbered by owning block (the
//! block of the lower voxel of their edge) in storage order and by edge
//! inside it, triangles follow the blocks in storage order: the output does
//! not depend on the thread count.

use glam::{IVec3, Vec3};
use rayon::prelude::*;

use super::store::{self, KeyMap, SIDE, Store, VOXELS, local_of};

/// Values of a block and its +1 neighbours: 9³ voxels.
const CUBE: usize = 9;
/// Smallest interpolation parameter along an edge (no zero-length edges).
const T_MIN: f32 = 1e-3;

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

/// Values (minus `iso`) of the 9³ voxels from block `key`; `None` when they
/// all have the same sign.
fn gather(
    store: &Store,
    band: f32,
    iso: f32,
    key: IVec3,
) -> Option<Box<[f32; CUBE * CUBE * CUBE]>> {
    let mut cube = Box::new([0f32; CUBE * CUBE * CUBE]);
    for oz in 0..2 {
        for oy in 0..2 {
            for ox in 0..2 {
                let o = IVec3::new(ox, oy, oz);
                let b = key + o;
                let src = store.find(b).map(|i| store.block(i));
                let bg = store.background(b, band) - iso;
                let lo = o * SIDE;
                let hi = (o * SIDE + IVec3::splat(SIDE)).min(IVec3::splat(CUBE as i32));
                for z in lo.z..hi.z {
                    for y in lo.y..hi.y {
                        for x in lo.x..hi.x {
                            let c = IVec3::new(x, y, z);
                            cube[cube_index(c)] = match src {
                                Some(v) => v[local_of(c)] - iso,
                                None => bg,
                            };
                        }
                    }
                }
            }
        }
    }
    let neg = cube[0] < 0.0;
    cube.iter().any(|&v| (v < 0.0) != neg).then_some(cube)
}

/// Crossing edges owned by one block: sorted local edge ids
/// (`local voxel * 3 + axis`) and their vertex positions.
#[derive(Default)]
struct Owned {
    edges: Vec<u16>,
    positions: Vec<Vec3>,
}

fn owned_edges(cube: &[f32], key: IVec3, origin: Vec3, vs: f32) -> Owned {
    let mut o = Owned::default();
    let base = key * SIDE;
    for z in 0..SIDE {
        for y in 0..SIDE {
            for x in 0..SIDE {
                let v = IVec3::new(x, y, z);
                let a = cube[cube_index(v)];
                for axis in 0..3u8 {
                    let w = v + axis_vec(axis);
                    let b = cube[cube_index(w)];
                    if (a < 0.0) != (b < 0.0) {
                        let t = crossing(a, b);
                        let p = (base + v).as_vec3() + axis_vec(axis).as_vec3() * t;
                        o.edges.push((local_of(v) * 3 + axis as usize) as u16);
                        o.positions.push(origin + p * vs);
                    }
                }
            }
        }
    }
    o
}

/// Flag of the vertex ids that index the block's added centre vertices.
const EXTRA: u32 = 1 << 31;

/// Where block outputs land in world space.
struct Frame {
    origin: Vec3,
    vs: f32,
}

/// Triangles of the cells of block `key` (global vertex ids, or [`EXTRA`]
/// with an index into `extra` for added centre vertices).
#[allow(clippy::too_many_arguments)]
fn block_triangles(
    cube: &[f32],
    key: IVec3,
    frame: &Frame,
    cells: &KeyMap<u32>,
    owned: &[Owned],
    bases: &[u32],
    out: &mut Vec<[u32; 3]>,
    extra: &mut Vec<Vec3>,
) {
    let (origin, vs) = (frame.origin, frame.vs);
    let mut cached: Option<(IVec3, usize)> = None;
    let mut vertex = |c: IVec3, axis: u8| -> Option<u32> {
        let off = IVec3::new(
            i32::from(c.x >= SIDE),
            i32::from(c.y >= SIDE),
            i32::from(c.z >= SIDE),
        );
        let b = key + off;
        let oi = match cached {
            Some((k, i)) if k == b => i,
            _ => {
                let i = *cells.get(&store::pack(b))? as usize;
                cached = Some((b, i));
                i
            }
        };
        let id = (local_of(c) * 3 + axis as usize) as u16;
        let r = owned[oi].edges.binary_search(&id).ok()?;
        Some(bases[oi] + r as u32)
    };
    for z in 0..SIDE {
        for y in 0..SIDE {
            for x in 0..SIDE {
                let cell = IVec3::new(x, y, z);
                let mut val = [0f32; 8];
                let mut mask = 0u8;
                for c in 0..8u8 {
                    val[c as usize] = cube[cube_index(cell + corner(c))];
                    if val[c as usize] < 0.0 {
                        mask |= 1 << c;
                    }
                }
                if mask == 0 || mask == 0xff {
                    continue;
                }
                let inside = |c: u8| mask & (1 << c) != 0;
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
                    let mut ids = [0u32; 12];
                    let mut edges = [0u8; 12];
                    let mut pos = [Vec3::ZERO; 12];
                    let mut n = 0;
                    let mut ok = true;
                    let mut e = start;
                    loop {
                        seen |= 1 << e;
                        edges[n] = e;
                        let (c, axis) = EDGES[e as usize];
                        let a = val[c as usize];
                        let b = val[(c | (1 << axis)) as usize];
                        pos[n] = corner(c).as_vec3() + axis_vec(axis).as_vec3() * crossing(a, b);
                        match vertex(cell + corner(c), axis) {
                            Some(id) => ids[n] = id,
                            None => ok = false,
                        }
                        n += 1;
                        e = next[e as usize];
                        if e == start || e == u8::MAX || n == 12 {
                            break;
                        }
                    }
                    if !ok || n < 3 {
                        continue;
                    }
                    triangulate(&ids[..n], &edges[..n], &pos[..n], out, &mut || {
                        let c = pos[..n].iter().copied().sum::<Vec3>() / n as f32;
                        extra.push(origin + ((key * SIDE + cell).as_vec3() + c) * vs);
                        EXTRA | (extra.len() as u32 - 1)
                    });
                }
            }
        }
    }
}

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

/// Triangulates one loop (inside on its left seen from outside) with
/// outward faces. A fan diagonal joining two vertices of the same cell face
/// could also be used by the neighbouring cell (an ambiguous face crossed
/// twice by one loop), so the apex is chosen to avoid such diagonals; when
/// no vertex qualifies a centre vertex (`centre()`) is added.
fn triangulate(
    ids: &[u32],
    edges: &[u8],
    pos: &[Vec3],
    out: &mut Vec<[u32; 3]>,
    centre: &mut impl FnMut() -> u32,
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

/// Surface where the field crosses `iso`: welded positions and triangles.
pub(crate) fn extract(
    store: &Store,
    band: f32,
    origin: Vec3,
    vs: f32,
    iso: f32,
) -> (Vec<Vec3>, Vec<[u32; 3]>) {
    // cells with their lower corner in a stored block or one block below
    // one (a cell reaching into a stored block from an empty neighbour)
    let mut keys: Vec<IVec3> = store
        .keys
        .iter()
        .flat_map(|&k| (0..8).map(move |c| k - IVec3::new(c & 1, (c >> 1) & 1, (c >> 2) & 1)))
        .collect();
    store::sort_keys(&mut keys);
    let mut cells = KeyMap::default();
    cells.reserve(keys.len());
    for (i, &k) in keys.iter().enumerate() {
        cells.insert(store::pack(k), i as u32);
    }
    let owned: Vec<Owned> = keys
        .par_iter()
        .map(|&k| match gather(store, band, iso, k) {
            Some(cube) => owned_edges(&cube[..], k, origin, vs),
            None => Owned::default(),
        })
        .collect();
    let mut bases = Vec::with_capacity(owned.len());
    let mut total = 0u32;
    for o in &owned {
        bases.push(total);
        total += o.edges.len() as u32;
    }
    let frame = Frame { origin, vs };
    let per_block: Vec<(Vec<[u32; 3]>, Vec<Vec3>)> = keys
        .par_iter()
        .enumerate()
        .map(|(i, &k)| {
            let mut out = Vec::new();
            let mut extra = Vec::new();
            if owned[i].edges.is_empty() && !touches_crossing(&owned, &cells, k) {
                return (out, extra);
            }
            if let Some(cube) = gather(store, band, iso, k) {
                block_triangles(
                    &cube[..],
                    k,
                    &frame,
                    &cells,
                    &owned,
                    &bases,
                    &mut out,
                    &mut extra,
                );
            }
            (out, extra)
        })
        .collect();
    let mut positions = Vec::with_capacity(total as usize);
    for o in owned {
        positions.extend(o.positions);
    }
    // added centre vertices follow the edge vertices, block after block
    let mut tris = Vec::with_capacity(per_block.iter().map(|(t, _)| t.len()).sum());
    for (t, extra) in per_block {
        let base = positions.len() as u32;
        tris.extend(t.into_iter().map(|tri| {
            tri.map(|v| {
                if v & EXTRA != 0 {
                    base + (v & !EXTRA)
                } else {
                    v
                }
            })
        }));
        positions.extend(extra);
    }
    debug_assert!(VOXELS * 3 <= u16::MAX as usize);
    (positions, tris)
}

/// `true` when a +1 neighbour of `k` owns crossing edges (the cells of `k`
/// may then use them).
fn touches_crossing(owned: &[Owned], cells: &KeyMap<u32>, k: IVec3) -> bool {
    (1..8).any(|c| {
        let b = k + IVec3::new(c & 1, (c >> 1) & 1, (c >> 2) & 1);
        cells
            .get(&store::pack(b))
            .is_some_and(|&i| !owned[i as usize].edges.is_empty())
    })
}
