//! Truncated signed distance fusion of depth frames (KinectFusion style) on
//! sparse voxel blocks.
//!
//! The volume stores, per voxel, the truncated signed distance to the
//! observed surface normalised by the truncation distance (`+1` in front of
//! the surface towards the camera, negative behind it) and an integration
//! weight. Only 8³ blocks near observed surfaces are allocated (voxel
//! hashing). Voxel `g` (integer coordinates) sits at world position
//! `g * voxel_mm`.
//!
//! * [`TsdfVolume::integrate`] fuses one depth image (millimetres, pinhole
//!   camera looking along +Z) taken from a known pose.
//! * [`TsdfVolume::raycast`] renders the fused surface from a pose (points
//!   and normals per pixel), the model side of frame-to-model ICP tracking.
//! * [`TsdfVolume::extract_cloud`] and [`TsdfVolume::extract_mesh`] export
//!   the zero crossing as an oriented point cloud or an open triangle mesh
//!   (marching cubes over observed voxels only, shared with the cloud mesher).

use glam::{Affine3A, IVec3, Vec3};
use rayon::prelude::*;
use serde::{Deserialize, Serialize};

use dc_mesh::{PointCloud, TriMesh};

use crate::mc::{self, BlockMap, MAX_VOXEL, SIDE, VOXELS};

/// Pinhole camera intrinsics of a depth image (pixels; `+Z` forward,
/// `+X` right, `+Y` down in the image).
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct CameraIntrinsics {
    /// Image width in pixels.
    pub width: u32,
    /// Image height in pixels.
    pub height: u32,
    /// Focal length along x, pixels.
    pub fx: f32,
    /// Focal length along y, pixels.
    pub fy: f32,
    /// Principal point x, pixels.
    pub cx: f32,
    /// Principal point y, pixels.
    pub cy: f32,
}

impl CameraIntrinsics {
    /// Number of pixels.
    pub fn pixel_count(&self) -> usize {
        self.width as usize * self.height as usize
    }

    /// Camera-space direction (z = 1) through pixel `(u, v)`.
    #[inline]
    pub fn ray(&self, u: f32, v: f32) -> Vec3 {
        Vec3::new((u - self.cx) / self.fx, (v - self.cy) / self.fy, 1.0)
    }

    /// Pixel coordinates of camera-space point `p`; `None` behind the camera.
    #[inline]
    pub fn project(&self, p: Vec3) -> Option<(f32, f32)> {
        (p.z > 0.0).then(|| (self.fx * p.x / p.z + self.cx, self.fy * p.y / p.z + self.cy))
    }

    /// Same camera with the image scaled by `1 / factor` (e.g. `2` halves
    /// the resolution), for coarse raycasts.
    pub fn downscaled(&self, factor: u32) -> Self {
        let f = factor.max(1) as f32;
        Self {
            width: (self.width / factor.max(1)).max(1),
            height: (self.height / factor.max(1)).max(1),
            fx: self.fx / f,
            fy: self.fy / f,
            cx: (self.cx + 0.5) / f - 0.5,
            cy: (self.cy + 0.5) / f - 0.5,
        }
    }
}

/// Fusion parameters.
#[derive(Clone, Copy, Debug)]
pub struct TsdfParams {
    /// Voxel edge length, millimetres.
    pub voxel_mm: f32,
    /// Truncation distance, millimetres (typically 3-5 voxels).
    pub trunc_mm: f32,
    /// Integration weight cap: older observations fade once reached.
    pub max_weight: f32,
}

impl Default for TsdfParams {
    fn default() -> Self {
        Self {
            voxel_mm: 0.3,
            trunc_mm: 1.2,
            max_weight: 64.0,
        }
    }
}

/// One 8³ block: normalised truncated distances and weights.
#[derive(Clone)]
struct Block {
    tsdf: [f32; VOXELS],
    weight: [f32; VOXELS],
}

impl Block {
    fn empty() -> Box<Self> {
        Box::new(Self {
            tsdf: [1.0; VOXELS],
            weight: [0.0; VOXELS],
        })
    }
}

/// Sparse TSDF volume (see the module docs).
#[derive(Clone)]
pub struct TsdfVolume {
    params: TsdfParams,
    map: BlockMap<usize>,
    keys: Vec<IVec3>,
    blocks: Vec<Box<Block>>,
    /// Allocated block range (inclusive), `None` when empty.
    bounds: Option<(IVec3, IVec3)>,
}

impl std::fmt::Debug for TsdfVolume {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("TsdfVolume")
            .field("params", &self.params)
            .field("blocks", &self.blocks.len())
            .finish()
    }
}

/// Valid depth sample (finite and positive).
#[inline]
fn valid(d: f32) -> bool {
    d.is_finite() && d > 0.0
}

impl TsdfVolume {
    /// Empty volume. Non-positive or non-finite parameters are replaced by
    /// the defaults; the truncation is at least one voxel.
    pub fn new(p: TsdfParams) -> Self {
        let d = TsdfParams::default();
        let ok = |v: f32| v.is_finite() && v > 0.0;
        let voxel_mm = if ok(p.voxel_mm) {
            p.voxel_mm
        } else {
            d.voxel_mm
        };
        let trunc_mm = if ok(p.trunc_mm) {
            p.trunc_mm.max(voxel_mm)
        } else {
            4.0 * voxel_mm
        };
        let max_weight = if ok(p.max_weight) {
            p.max_weight.max(1.0)
        } else {
            d.max_weight
        };
        Self {
            params: TsdfParams {
                voxel_mm,
                trunc_mm,
                max_weight,
            },
            map: BlockMap::default(),
            keys: Vec::new(),
            blocks: Vec::new(),
            bounds: None,
        }
    }

    /// Parameters in use.
    pub fn params(&self) -> TsdfParams {
        self.params
    }

    /// Number of allocated blocks.
    pub fn block_count(&self) -> usize {
        self.blocks.len()
    }

    /// Removes every block.
    pub fn clear(&mut self) {
        self.map.clear();
        self.keys.clear();
        self.blocks.clear();
        self.bounds = None;
    }

    /// Rough memory footprint in bytes.
    pub fn approx_bytes(&self) -> usize {
        self.blocks.len() * (std::mem::size_of::<Block>() + 32)
    }

    /// Fuses one depth image (`depth_mm` row-major, `width * height`
    /// entries, `0` or NaN = no measurement) seen from `cam_to_world`.
    /// A frame whose size does not match `intr` is ignored.
    pub fn integrate(
        &mut self,
        depth_mm: &[f32],
        intr: &CameraIntrinsics,
        cam_to_world: &Affine3A,
    ) {
        if depth_mm.len() != intr.pixel_count() || depth_mm.is_empty() {
            return;
        }
        let touched = self.allocate(depth_mm, intr, cam_to_world);
        if touched.is_empty() {
            return;
        }
        let mut mark = vec![false; self.blocks.len()];
        for &i in &touched {
            mark[i] = true;
        }
        let world_to_cam = cam_to_world.inverse();
        let p = self.params;
        let (w, h) = (intr.width as i64, intr.height as i64);
        let inv_trunc = 1.0 / p.trunc_mm;
        self.blocks
            .par_iter_mut()
            .zip(self.keys.par_iter())
            .zip(mark.par_iter())
            .filter(|(_, m)| **m)
            .for_each(|((block, key), _)| {
                let base = *key * SIDE;
                let step_x = Vec3::from(world_to_cam.matrix3.x_axis) * p.voxel_mm;
                for z in 0..SIDE {
                    for y in 0..SIDE {
                        let g0 = base + IVec3::new(0, y, z);
                        let mut pc = world_to_cam.transform_point3(g0.as_vec3() * p.voxel_mm);
                        let row = ((z * SIDE + y) * SIDE) as usize;
                        for x in 0..SIDE as usize {
                            let cur = pc;
                            pc += step_x;
                            if cur.z <= 0.0 {
                                continue;
                            }
                            let iz = 1.0 / cur.z;
                            let u = (intr.fx * cur.x * iz + intr.cx).round() as i64;
                            let v = (intr.fy * cur.y * iz + intr.cy).round() as i64;
                            if u < 0 || v < 0 || u >= w || v >= h {
                                continue;
                            }
                            let d = depth_mm[(v * w + u) as usize];
                            if !valid(d) {
                                continue;
                            }
                            let sdf = d - cur.z;
                            if sdf < -p.trunc_mm {
                                continue;
                            }
                            let t = (sdf * inv_trunc).min(1.0);
                            let i = row + x;
                            let wo = block.weight[i];
                            block.tsdf[i] = (block.tsdf[i] * wo + t) / (wo + 1.0);
                            block.weight[i] = (wo + 1.0).min(p.max_weight);
                        }
                    }
                }
            });
    }

    /// Allocates the blocks around the measured surface of a frame; returns
    /// the indices of the blocks the frame touches.
    fn allocate(
        &mut self,
        depth_mm: &[f32],
        intr: &CameraIntrinsics,
        cam_to_world: &Affine3A,
    ) -> Vec<usize> {
        let p = self.params;
        let block_mm = p.voxel_mm * SIDE as f32;
        let zmin = depth_mm
            .par_iter()
            .copied()
            .filter(|&d| valid(d))
            .reduce(|| f32::INFINITY, f32::min);
        if !zmin.is_finite() {
            return Vec::new();
        }
        // sample pixels so that neighbouring samples land well within a block
        let footprint = zmin / intr.fx.max(intr.fy);
        let stride = ((0.5 * block_mm / footprint).floor() as usize).clamp(1, 8);
        let samples = ((2.0 * p.trunc_mm / (0.5 * block_mm)).ceil() as usize).max(2);
        let w = intr.width as usize;
        let h = intr.height as usize;
        let inv_vs = 1.0 / p.voxel_mm;
        let limit = (MAX_VOXEL - SIDE) as f32;
        let mut keys: Vec<u64> = (0..h)
            .into_par_iter()
            .step_by(stride)
            .flat_map_iter(|v| {
                let mut out = Vec::new();
                for u in (0..w).step_by(stride) {
                    let d = depth_mm[v * w + u];
                    if !valid(d) {
                        continue;
                    }
                    let ray = intr.ray(u as f32, v as f32);
                    for s in 0..=samples {
                        let z = d - p.trunc_mm + 2.0 * p.trunc_mm * s as f32 / samples as f32;
                        if z <= 0.0 {
                            continue;
                        }
                        let g = cam_to_world.transform_point3(ray * z) * inv_vs;
                        if g.abs().max_element() > limit {
                            continue;
                        }
                        out.push(mc::pack(mc::block_of(g.round().as_ivec3())));
                    }
                }
                out.sort_unstable();
                out.dedup();
                out.into_iter()
            })
            .collect();
        keys.par_sort_unstable();
        keys.dedup();
        let mut touched = Vec::with_capacity(keys.len());
        for k in keys {
            let key = mc::unpack(k);
            let next = self.blocks.len();
            let i = *self.map.entry(key).or_insert(next);
            if i == next {
                self.keys.push(key);
                self.blocks.push(Block::empty());
                self.bounds = Some(match self.bounds {
                    Some((lo, hi)) => (lo.min(key), hi.max(key)),
                    None => (key, key),
                });
            }
            touched.push(i);
        }
        touched
    }

    /// Distance and weight of global voxel `g` (`None` if unallocated).
    #[inline]
    fn voxel(&self, g: IVec3) -> Option<(f32, f32)> {
        let b = &self.blocks[*self.map.get(&mc::block_of(g))?];
        let i = mc::local_index(g);
        Some((b.tsdf[i], b.weight[i]))
    }

    /// Distances at the eight corners of the cell with lower corner `g`
    /// (corner `k` at offset `(k & 1, (k >> 1) & 1, k >> 2)`); `None` when a
    /// corner is unallocated or unobserved.
    fn corners(&self, g: IVec3) -> Option<[f32; 8]> {
        let l = g - mc::block_of(g) * SIDE;
        let mut c = [0f32; 8];
        if l.x < SIDE - 1 && l.y < SIDE - 1 && l.z < SIDE - 1 {
            let b = &self.blocks[*self.map.get(&mc::block_of(g))?];
            let i0 = mc::local_index(g);
            for (k, ck) in c.iter_mut().enumerate() {
                let o =
                    (k & 1) + ((k >> 1) & 1) * SIDE as usize + (k >> 2) * (SIDE * SIDE) as usize;
                if b.weight[i0 + o] <= 0.0 {
                    return None;
                }
                *ck = b.tsdf[i0 + o];
            }
        } else {
            for (k, ck) in c.iter_mut().enumerate() {
                let o = IVec3::new((k & 1) as i32, ((k >> 1) & 1) as i32, (k >> 2) as i32);
                let (d, w) = self.voxel(g + o)?;
                if w <= 0.0 {
                    return None;
                }
                *ck = d;
            }
        }
        Some(c)
    }

    /// Trilinear distance at voxel-space position `q`; `None` when a corner
    /// is unallocated or unobserved.
    fn sample(&self, q: Vec3) -> Option<f32> {
        let f = q.floor();
        let c = self.corners(f.as_ivec3())?;
        let t = q - f;
        let x0 = c[0] + (c[1] - c[0]) * t.x;
        let x1 = c[2] + (c[3] - c[2]) * t.x;
        let x2 = c[4] + (c[5] - c[4]) * t.x;
        let x3 = c[6] + (c[7] - c[6]) * t.x;
        let y0 = x0 + (x1 - x0) * t.y;
        let y1 = x2 + (x3 - x2) * t.y;
        Some(y0 + (y1 - y0) * t.z)
    }

    /// Unit gradient of the field at voxel-space position `q`: central
    /// differences of trilinear samples, or (at the border of the observed
    /// region) the derivative of the trilinear interpolant of the cell
    /// holding `q`. `None` when neither is available.
    fn gradient(&self, q: Vec3) -> Option<Vec3> {
        let h = 0.5;
        let d = |e: Vec3| Some(self.sample(q + e * h)? - self.sample(q - e * h)?);
        let g = match (d(Vec3::X), d(Vec3::Y), d(Vec3::Z)) {
            (Some(x), Some(y), Some(z)) => Vec3::new(x, y, z),
            _ => {
                let f = q.floor();
                let c = self.corners(f.as_ivec3())?;
                let t = q - f;
                let lerp = |a: f32, b: f32, s: f32| a + (b - a) * s;
                // corner k: x = k & 1, y = (k >> 1) & 1, z = k >> 2
                let dx = lerp(
                    lerp(c[1] - c[0], c[3] - c[2], t.y),
                    lerp(c[5] - c[4], c[7] - c[6], t.y),
                    t.z,
                );
                let dy = lerp(
                    lerp(c[2] - c[0], c[3] - c[1], t.x),
                    lerp(c[6] - c[4], c[7] - c[5], t.x),
                    t.z,
                );
                let dz = lerp(
                    lerp(c[4] - c[0], c[5] - c[1], t.x),
                    lerp(c[6] - c[2], c[7] - c[3], t.x),
                    t.y,
                );
                Vec3::new(dx, dy, dz)
            }
        };
        let n = g.normalize_or_zero();
        (n != Vec3::ZERO).then_some(n)
    }

    /// Renders the fused surface from `cam_to_world`: world points and
    /// normals per pixel (row-major), NaN where the ray hits nothing.
    pub fn raycast(
        &self,
        intr: &CameraIntrinsics,
        cam_to_world: &Affine3A,
    ) -> (Vec<Vec3>, Vec<Vec3>) {
        let n = intr.pixel_count();
        let mut points = vec![Vec3::NAN; n];
        let mut normals = vec![Vec3::NAN; n];
        let Some((blo, bhi)) = self.bounds else {
            return (points, normals);
        };
        let vs = self.params.voxel_mm;
        let lo = (blo * SIDE).as_vec3() * vs;
        let hi = ((bhi + IVec3::ONE) * SIDE).as_vec3() * vs;
        let origin = cam_to_world.transform_point3(Vec3::ZERO);
        let w = intr.width as usize;
        points
            .par_chunks_mut(w)
            .zip(normals.par_chunks_mut(w))
            .enumerate()
            .for_each(|(v, (prow, nrow))| {
                for u in 0..w {
                    let dir = cam_to_world
                        .transform_vector3(intr.ray(u as f32, v as f32))
                        .normalize();
                    if let Some((p, nrm)) = self.cast(origin, dir, lo, hi) {
                        prow[u] = p;
                        nrow[u] = nrm;
                    }
                }
            });
        (points, normals)
    }

    /// First front-facing zero crossing along a ray inside box `[lo, hi]`.
    fn cast(&self, origin: Vec3, dir: Vec3, lo: Vec3, hi: Vec3) -> Option<(Vec3, Vec3)> {
        let inv = dir.recip();
        let ta = (lo - origin) * inv;
        let tb = (hi - origin) * inv;
        let t0 = ta.min(tb).max_element().max(0.0);
        let t1 = ta.max(tb).min_element();
        if t0.is_nan() || t1.is_nan() || t0 >= t1 {
            return None;
        }
        let p = self.params;
        let vs = p.voxel_mm;
        let inv_vs = 1.0 / vs;
        let block_mm = vs * SIDE as f32;
        let mut t = t0;
        let mut prev: Option<(f32, f32)> = None;
        while t < t1 {
            let x = origin + dir * t;
            let q = x * inv_vs;
            let key = mc::block_of(q.floor().as_ivec3());
            if !self.map.contains_key(&key) {
                // jump to where the ray leaves this block
                let bmin = key.as_vec3() * block_mm;
                let bmax = bmin + Vec3::splat(block_mm);
                let exit = ((bmin - x) * inv).max((bmax - x) * inv).min_element();
                t += if exit.is_finite() { exit.max(0.0) } else { 0.0 } + 1e-3 * vs;
                prev = None;
                continue;
            }
            match self.sample(q) {
                None => {
                    prev = None;
                    t += vs;
                }
                Some(val) => {
                    if let Some((tp, vp)) = prev {
                        if vp > 0.0 && val <= 0.0 {
                            let th = tp + (t - tp) * vp / (vp - val);
                            let hit = origin + dir * th;
                            let n = self.gradient(hit * inv_vs)?;
                            return Some((hit, n));
                        }
                    } else if val < 0.0 {
                        // entered behind a surface: it does not face the camera
                        return None;
                    }
                    prev = Some((t, val));
                    t += (val * p.trunc_mm * 0.8).max(0.5 * vs);
                }
            }
        }
        None
    }

    /// Zero-crossing points of the observed field (one per voxel edge with a
    /// sign change between two observed voxels), with unit gradient normals
    /// pointing out of the surface (towards the observing cameras).
    pub fn extract_cloud(&self) -> PointCloud {
        let vs = self.params.voxel_mm;
        let max_step = self.max_step();
        let keys = self.sorted_keys();
        let parts: Vec<(Vec<Vec3>, Vec<Vec3>)> = keys
            .par_iter()
            .map(|&key| {
                let b = &self.blocks[self.map[&key]];
                let mut pts = Vec::new();
                let mut nrm = Vec::new();
                for i in 0..VOXELS {
                    if b.weight[i] <= 0.0 {
                        continue;
                    }
                    let a = b.tsdf[i];
                    let g = key * SIDE + mc::local_coords(i);
                    for axis in [IVec3::X, IVec3::Y, IVec3::Z] {
                        let Some((c, wc)) = self.voxel(g + axis) else {
                            continue;
                        };
                        if wc <= 0.0 || (a < 0.0) == (c < 0.0) || (a - c).abs() > max_step {
                            continue;
                        }
                        let t = a / (a - c);
                        let q = g.as_vec3() + axis.as_vec3() * t;
                        let Some(n) = self.gradient(q) else {
                            continue;
                        };
                        pts.push(q * vs);
                        nrm.push(n);
                    }
                }
                (pts, nrm)
            })
            .collect();
        let mut cloud = PointCloud::default();
        let mut normals = Vec::new();
        for (p, n) in parts {
            cloud.points.extend(p);
            normals.extend(n);
        }
        cloud.normals = Some(normals);
        cloud
    }

    /// Open triangle mesh of the zero crossing (marching cubes over the
    /// observed voxels only), with vertex normals; faces are
    /// counter-clockwise seen from outside (from the cameras).
    pub fn extract_mesh(&self) -> TriMesh {
        let keys = self.sorted_keys();
        let mut map = BlockMap::default();
        map.reserve(keys.len());
        let values: Vec<[f32; VOXELS]> = keys
            .iter()
            .enumerate()
            .map(|(i, k)| {
                map.insert(*k, i);
                let b = &self.blocks[self.map[k]];
                std::array::from_fn(|j| {
                    if b.weight[j] > 0.0 {
                        b.tsdf[j]
                    } else {
                        f32::NAN
                    }
                })
            })
            .collect();
        let field = mc::Field {
            keys: &keys,
            map: &map,
            values: &values,
            origin: Vec3::ZERO,
            voxel: self.params.voxel_mm,
            max_step: self.max_step(),
        };
        let (positions, tris) = mc::extract(&field);
        let mut mesh = TriMesh::new(positions, tris);
        mesh.update_normals();
        mesh
    }

    /// Largest normalised distance change across a voxel edge that still
    /// counts as a surface crossing (larger jumps join the truncated front
    /// and back sides of two different surfaces).
    fn max_step(&self) -> f32 {
        (3.0 * self.params.voxel_mm / self.params.trunc_mm).clamp(0.5, 1.9)
    }

    fn sorted_keys(&self) -> Vec<IVec3> {
        let mut keys = self.keys.clone();
        mc::sort_keys(&mut keys);
        keys
    }
}

/// Camera-space points of the valid pixels of a depth image, sampling every
/// `stride`-th pixel along both axes (`0` counts as `1`), with the row-major
/// index of their pixel.
pub fn backproject(
    depth_mm: &[f32],
    intr: &CameraIntrinsics,
    stride: u32,
) -> (Vec<Vec3>, Vec<usize>) {
    let stride = stride.max(1) as usize;
    let w = intr.width as usize;
    let h = (intr.height as usize).min(depth_mm.len().checked_div(w).unwrap_or(0));
    let mut points = Vec::new();
    let mut pixels = Vec::new();
    for v in (0..h).step_by(stride) {
        for u in (0..w).step_by(stride) {
            let i = v * w + u;
            let d = depth_mm[i];
            if valid(d) {
                points.push(intr.ray(u as f32, v as f32) * d);
                pixels.push(i);
            }
        }
    }
    (points, pixels)
}

#[cfg(test)]
mod tests;
