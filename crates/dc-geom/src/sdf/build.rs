//! Mesh → narrow-band signed distance (OpenVDB `meshToLevelSet` as
//! reproduced by `gpu_sdf.mesh_to_level_set`).
//!
//! 1. Candidate blocks: an octree descent over the box of the mesh keeps the
//!    blocks whose voxels can lie within the band (one BVH query per node).
//! 2. Exact unsigned distance of every voxel of those blocks through the BVH
//!    (bounded by the band and by the 1-Lipschitz bound of the previous voxel).
//! 3. Sign: the exterior is flooded from the border of the domain through the
//!    voxels farther than [`BARRIER`] voxels from the surface (holes narrower
//!    than that stay closed, enclosed cavities are filled); the voxels near the
//!    surface take the sign of the angle-weighted pseudo-normal of their
//!    closest feature, and a positive one with no exterior voxel among its 26
//!    neighbours is moved inside (the self-intersecting surface of a union of
//!    overlapping closed meshes, OpenVDB `RemoveSelfIntersectingSurface`).
//! 4. Distances measured to a surface left inside the solid (a union) are
//!    corrected by one re-distancing pass.

use glam::{DVec3, IVec3, Vec3};
use rayon::prelude::*;

use dc_mesh::TriMesh;

use super::SdfGrid;
use super::flood::{self, Domain, Mask};
use super::store::{self, MAX_BLOCK, SIDE, Store, VOXELS, local_coord};
use crate::bvh::{Bvh, ClosestHit};
use crate::{GeomError, Result};

/// Voxels closer than this many voxel sizes to the surface stop the exterior
/// flood (more than half a voxel, so that no six-connected path crosses a
/// closed surface; OpenVDB's intersecting voxels are about as wide).
pub(crate) const BARRIER: f32 = 0.75;
/// Inside voxels closer than this many voxel sizes to the surface with no
/// outside neighbour reveal a surface left inside the solid.
const BURIED: f32 = 0.25;
/// Relative slack on the 1-Lipschitz bound between neighbouring voxels.
const LIPSCHITZ_SLACK: f32 = 1.0 + 1e-4;
/// Largest voxel coordinate (in absolute value).
const MAX_VOXEL: f32 = (MAX_BLOCK * SIDE) as f32;

/// Angle-weighted pseudo-normals (Bærentzen and Aanæs) of a mesh whose
/// vertices are welded by exact position.
struct PseudoNormals {
    face: Vec<Vec3>,
    /// Edge normal opposite each corner of each face.
    edge: Vec<[Vec3; 3]>,
    /// Normal of each welded vertex (indexed by original vertex).
    vertex: Vec<Vec3>,
}

impl PseudoNormals {
    fn new(mesh: &TriMesh) -> Self {
        let nv = mesh.positions.len();
        // weld by exact position
        let mut order: Vec<u32> = (0..nv as u32).collect();
        let key = |i: u32| {
            let p = mesh.positions[i as usize];
            (p.x.to_bits(), p.y.to_bits(), p.z.to_bits())
        };
        order.par_sort_unstable_by_key(|&i| (key(i), i));
        let mut canon: Vec<u32> = (0..nv as u32).collect();
        for w in 1..order.len() {
            if key(order[w]) == key(order[w - 1]) {
                canon[order[w] as usize] = canon[order[w - 1] as usize];
            }
        }
        let tris: Vec<[u32; 3]> = mesh
            .tris
            .iter()
            .map(|t| t.map(|v| canon[v as usize]))
            .collect();
        let pos = |v: u32| mesh.positions[v as usize].as_dvec3();
        let face: Vec<Vec3> = tris
            .par_iter()
            .map(|&[a, b, c]| {
                let n = (pos(b) - pos(a)).cross(pos(c) - pos(a));
                let l = n.length();
                if l > 0.0 && l.is_finite() {
                    (n / l).as_vec3()
                } else {
                    Vec3::ZERO
                }
            })
            .collect();
        let mut vsum = vec![DVec3::ZERO; nv];
        for (f, t) in tris.iter().enumerate() {
            let n = face[f].as_dvec3();
            if n == DVec3::ZERO {
                continue;
            }
            for k in 0..3 {
                let p = pos(t[k]);
                let e1 = pos(t[(k + 1) % 3]) - p;
                let e2 = pos(t[(k + 2) % 3]) - p;
                let angle = e1.angle_between(e2);
                if angle.is_finite() {
                    vsum[t[k] as usize] += angle * n;
                }
            }
        }
        let mut edges: Vec<(u32, u32, u32, u8)> = Vec::with_capacity(tris.len() * 3);
        for (f, t) in tris.iter().enumerate() {
            for k in 0..3 {
                let (a, b) = (t[(k + 1) % 3], t[(k + 2) % 3]);
                edges.push((a.min(b), a.max(b), f as u32, k as u8));
            }
        }
        edges.par_sort_unstable();
        let mut edge = vec![[Vec3::ZERO; 3]; tris.len()];
        let mut s = 0;
        while s < edges.len() {
            let mut e = s + 1;
            while e < edges.len() && edges[e].0 == edges[s].0 && edges[e].1 == edges[s].1 {
                e += 1;
            }
            let n: Vec3 = edges[s..e].iter().map(|x| face[x.2 as usize]).sum();
            for x in &edges[s..e] {
                edge[x.2 as usize][x.3 as usize] = n;
            }
            s = e;
        }
        let vertex = (0..nv).map(|v| vsum[canon[v] as usize].as_vec3()).collect();
        Self { face, edge, vertex }
    }

    /// `true` when `p` lies on the outer side of the closest feature `hit`.
    fn outside(&self, mesh: &TriMesh, p: Vec3, hit: &ClosestHit) -> bool {
        let f = hit.face as usize;
        let w = hit.barycentric;
        let w = [w.x, w.y, w.z];
        let nonzero = w.iter().filter(|&&x| x > 1e-7).count();
        let n = match nonzero {
            0 | 1 => {
                let k = (0..3)
                    .max_by(|&a, &b| w[a].total_cmp(&w[b]).then(b.cmp(&a)))
                    .unwrap_or(0);
                self.vertex[mesh.tris[f][k] as usize]
            }
            2 => {
                let k = (0..3).find(|&k| w[k] <= 1e-7).unwrap_or(0);
                self.edge[f][k]
            }
            _ => self.face[f],
        };
        (p - hit.point).dot(n) >= 0.0
    }
}

/// Voxel coordinate box (inclusive) of the band around `bvh`.
fn voxel_box(bvh: &Bvh, origin: Vec3, vs: f32, band: f32) -> Result<(IVec3, IVec3)> {
    let b = bvh
        .bounds()
        .ok_or_else(|| GeomError::InvalidInput("mesh without finite triangles".into()))?;
    let lo = ((b.min - Vec3::splat(band) - origin) / vs).floor();
    let hi = ((b.max + Vec3::splat(band) - origin) / vs).ceil();
    if !lo.is_finite()
        || !hi.is_finite()
        || lo.min_element() < -MAX_VOXEL
        || hi.max_element() > MAX_VOXEL
    {
        return Err(GeomError::InvalidInput(
            "mesh too large for the voxel size".into(),
        ));
    }
    Ok((lo.as_ivec3(), hi.as_ivec3()))
}

/// Blocks whose voxels may lie within `band` of the surface of `bvh`, in
/// storage order, with the distance of their centre (an upper bound for the
/// voxel queries).
pub(crate) fn candidate_blocks(
    bvh: &Bvh,
    origin: Vec3,
    vs: f32,
    band: f32,
) -> Result<Vec<(IVec3, f32)>> {
    let (lo, hi) = voxel_box(bvh, origin, vs, band)?;
    let (blo, bhi) = (store::block_of(lo), store::block_of(hi));
    let ext = bhi - blo + IVec3::ONE;
    let size = (ext.max_element() as u32).next_power_of_two() as i32;
    let mut out = descend(bvh, origin, vs, band, blo, size, bhi);
    out.sort_unstable_by_key(|(k, _)| store::pack(*k));
    Ok(out)
}

fn descend(
    bvh: &Bvh,
    origin: Vec3,
    vs: f32,
    band: f32,
    lo: IVec3,
    size: i32,
    bhi: IVec3,
) -> Vec<(IVec3, f32)> {
    // voxel centres of the node span [lo * 8, (lo + size) * 8 - 1]
    let span = (size * SIDE - 1) as f32 * 0.5;
    let centre = origin + ((lo * SIDE).as_vec3() + Vec3::splat(span)) * vs;
    let half_diag = span * vs * 3f32.sqrt();
    let Some(hit) = bvh.closest_point(centre, band + half_diag) else {
        return Vec::new();
    };
    if size == 1 {
        return vec![(lo, hit.distance)];
    }
    let h = size / 2;
    let children: Vec<IVec3> = (0..8)
        .map(|c| lo + IVec3::new(c & 1, (c >> 1) & 1, (c >> 2) & 1) * h)
        .filter(|c| c.cmple(bhi).all())
        .collect();
    if size >= 8 {
        children
            .par_iter()
            .flat_map_iter(|&c| descend(bvh, origin, vs, band, c, h, bhi))
            .collect()
    } else {
        children
            .iter()
            .flat_map(|&c| descend(bvh, origin, vs, band, c, h, bhi))
            .collect()
    }
}

/// Calls `f(local index, voxel position, unsigned distance, hit)` for every
/// voxel of block `key`; the distance is clamped to `band` (no hit beyond).
/// Each query is bounded by the band, the distance of the block centre and
/// the previous voxel of the row (the distance is 1-Lipschitz).
pub(crate) fn scan_block(
    bvh: &Bvh,
    key: IVec3,
    centre_distance: f32,
    origin: Vec3,
    vs: f32,
    band: f32,
    mut f: impl FnMut(usize, Vec3, f32, Option<&ClosestHit>),
) {
    let half_diag = 3.5 * vs * 3f32.sqrt();
    let first = band.min((centre_distance + half_diag) * LIPSCHITZ_SLACK + 1e-6);
    let base = key * SIDE;
    let step = vs * LIPSCHITZ_SLACK + 1e-6;
    for zy in 0..64 {
        let mut prev = f32::INFINITY;
        for x in 0..8 {
            let i = zy * 8 + x;
            let p = origin + (base + local_coord(i)).as_vec3() * vs;
            let bound = first.min(prev + step);
            match bvh.closest_point(p, bound) {
                Some(h) => {
                    prev = h.distance;
                    f(i, p, h.distance.min(band), Some(&h));
                }
                None => {
                    prev = f32::INFINITY;
                    f(i, p, band, None);
                }
            }
        }
    }
}

/// Raw result of one block before the sign.
struct Raw {
    key: IVec3,
    dist: Box<[f32; VOXELS]>,
    barrier: Mask,
    positive: Mask,
}

/// Validates the sampling parameters; returns the band actually used.
pub(crate) fn check_params(vs: f32, band: f32) -> Result<f32> {
    if !(vs.is_finite() && vs > 0.0) {
        return Err(GeomError::InvalidInput(format!(
            "voxel size must be positive, got {vs}"
        )));
    }
    if !(band.is_finite() && band >= 0.0) {
        return Err(GeomError::InvalidInput(format!(
            "band must be finite and not negative, got {band}"
        )));
    }
    Ok(band.max(super::MIN_BAND_VOXELS * vs))
}

pub(crate) fn from_mesh(mesh: &TriMesh, vs: f32, band: f32) -> Result<SdfGrid> {
    let band = check_params(vs, band)?;
    if mesh.tris.is_empty() {
        return Err(GeomError::InvalidInput("mesh has no triangles".into()));
    }
    mesh.check()?;
    let origin = Vec3::ZERO;
    let bvh = Bvh::build(mesh)?;
    if bvh.is_empty() {
        return Err(GeomError::InvalidInput(
            "mesh without finite triangles".into(),
        ));
    }
    let normals = PseudoNormals::new(mesh);
    let candidates = candidate_blocks(&bvh, origin, vs, band)?;
    let barrier_d = BARRIER * vs;
    let raws: Vec<Raw> = candidates
        .par_iter()
        .filter_map(|&(key, dc)| {
            let mut dist = Box::new([band; VOXELS]);
            let mut barrier = [0u64; 8];
            let mut positive = [0u64; 8];
            let mut any = false;
            scan_block(&bvh, key, dc, origin, vs, band, |i, p, d, hit| {
                dist[i] = d;
                if let Some(h) = hit {
                    any |= d < band;
                    if d <= barrier_d {
                        flood::set(&mut barrier, i);
                    }
                    if normals.outside(mesh, p, h) {
                        flood::set(&mut positive, i);
                    }
                }
            });
            any.then_some(Raw {
                key,
                dist,
                barrier,
                positive,
            })
        })
        .collect();
    let keys: Vec<IVec3> = raws.iter().map(|r| r.key).collect();
    let domain = Domain::new(&keys, &[])?;
    let free: Vec<Mask> = raws.iter().map(|r| r.barrier.map(|w| !w)).collect();
    let ext = flood::flood(&domain, &free, |_| true);
    // signs and values
    let mut data = vec![0f32; keys.len() * VOXELS];
    data.par_chunks_mut(VOXELS)
        .zip(raws.par_iter().enumerate())
        .for_each(|(out, (bi, r))| {
            let base = r.key * SIDE;
            for (i, (o, &d)) in out.iter_mut().zip(r.dist.iter()).enumerate() {
                let inside = if !flood::get(&r.barrier, i) {
                    !flood::get(&ext.ext[bi], i)
                } else if flood::get(&r.positive, i) {
                    let g = base + local_coord(i);
                    !NEIGHBOURS_26.iter().any(|o| ext.reached(&domain, g + *o))
                } else {
                    true
                };
                *o = if inside { -d } else { d };
            }
        });
    let tiles: Vec<IVec3> = (0..domain.cells.len())
        .filter(|&c| domain.cells[c] == Domain::NONE && !ext.out[c])
        .map(|c| domain.block(c))
        .collect();
    drop(raws);
    let mut grid = SdfGrid::from_store(vs, origin, band, Store::new(keys, data, tiles));
    clean_isolated(&mut grid);
    if has_buried_surface(&grid) {
        grid.redistance_to(band)?;
    }
    Ok(grid)
}

/// The 26 neighbour offsets.
pub(crate) const NEIGHBOURS_26: [IVec3; 26] = {
    let mut o = [IVec3::ZERO; 26];
    let mut n = 0;
    let mut z = -1;
    while z <= 1 {
        let mut y = -1;
        while y <= 1 {
            let mut x = -1;
            while x <= 1 {
                if x != 0 || y != 0 || z != 0 {
                    o[n] = IVec3::new(x, y, z);
                    n += 1;
                }
                x += 1;
            }
            y += 1;
        }
        z += 1;
    }
    o
};

/// Flips the voxels whose sign no face neighbour shares (single-voxel
/// bubbles left by an unreliable pseudo-normal at a hole or a sharp feature).
fn clean_isolated(grid: &mut SdfGrid) {
    let band = grid.band;
    let store = &grid.store;
    let flips: Vec<(usize, f32)> = (0..store.len())
        .into_par_iter()
        .flat_map_iter(|bi| {
            let key = store.keys[bi];
            let vals = store.block(bi);
            let base = key * SIDE;
            (0..VOXELS).filter_map(move |i| {
                let v = vals[i];
                let neg = v < 0.0;
                let g = base + local_coord(i);
                let alone = flood::DIRS
                    .iter()
                    .all(|d| (store.value(g + *d, band) < 0.0) != neg);
                alone.then_some((
                    bi * VOXELS + i,
                    if neg {
                        v.abs()
                    } else {
                        -v.abs().max(f32::MIN_POSITIVE)
                    },
                ))
            })
        })
        .collect();
    for (i, v) in flips {
        grid.store.data[i] = v;
    }
}

/// `true` when an inside voxel very close to the surface has no outside
/// face neighbour: its distance was measured to a surface buried inside the
/// solid.
fn has_buried_surface(grid: &SdfGrid) -> bool {
    let band = grid.band;
    let lim = -BURIED * grid.voxel_size;
    let store = &grid.store;
    (0..store.len()).into_par_iter().any(|bi| {
        let base = store.keys[bi] * SIDE;
        let vals = store.block(bi);
        (0..VOXELS).any(|i| {
            let v = vals[i];
            v < 0.0
                && v > lim
                && flood::DIRS
                    .iter()
                    .all(|d| store.value(base + local_coord(i) + *d, band) < 0.0)
        })
    })
}
