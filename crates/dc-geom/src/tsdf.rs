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
//!   camera looking along +Z) taken from a known pose;
//!   [`TsdfVolume::integrate_weighted`] does the same with a confidence per
//!   pixel ([`depth_weights`]: depth edges eroded, grazing views and far
//!   samples weighted down), and [`TsdfVolume::deintegrate_weighted`] takes
//!   such a frame back out (leave-one-out registration of multi-view scans).
//! * [`TsdfVolume::raycast`] renders the fused surface from a pose (points
//!   and normals per pixel), the model side of frame-to-model ICP tracking.
//! * [`TsdfVolume::extract_cloud`] and [`TsdfVolume::extract_mesh`] export
//!   the zero crossing as an oriented point cloud or an open triangle mesh
//!   (marching cubes over observed voxels only, shared with the cloud mesher;
//!   [`TsdfVolume::extract_mesh_with`] ignores voxels observed too little).

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
    /// entries, `0` or NaN = no measurement) seen from `cam_to_world`, every
    /// pixel with weight 1. A frame whose size does not match `intr` is
    /// ignored.
    pub fn integrate(
        &mut self,
        depth_mm: &[f32],
        intr: &CameraIntrinsics,
        cam_to_world: &Affine3A,
    ) {
        self.update(depth_mm, None, intr, cam_to_world, false);
    }

    /// [`TsdfVolume::integrate`] with a confidence per pixel (`weights`,
    /// one per pixel, e.g. from [`depth_weights`]): a pixel adds its weight
    /// to the voxels it observes, a pixel of weight 0 is ignored. Weights of
    /// the wrong length leave the volume unchanged.
    pub fn integrate_weighted(
        &mut self,
        depth_mm: &[f32],
        weights: &[f32],
        intr: &CameraIntrinsics,
        cam_to_world: &Affine3A,
    ) {
        if weights.len() == depth_mm.len() {
            self.update(depth_mm, Some(weights), intr, cam_to_world, false);
        }
    }

    /// Takes back a frame fused by [`TsdfVolume::integrate_weighted`] with
    /// the same arguments. The volume returns to its state without that
    /// frame (up to rounding) as long as no voxel the frame observes reached
    /// [`TsdfParams::max_weight`]: volumes that frames are taken out of need
    /// a cap above the total weight. Voxels left without weight become
    /// unobserved; their blocks stay allocated.
    pub fn deintegrate_weighted(
        &mut self,
        depth_mm: &[f32],
        weights: &[f32],
        intr: &CameraIntrinsics,
        cam_to_world: &Affine3A,
    ) {
        if weights.len() == depth_mm.len() {
            self.update(depth_mm, Some(weights), intr, cam_to_world, true);
        }
    }

    /// Adds (or with `remove`, takes back) one frame: see
    /// [`TsdfVolume::integrate_weighted`].
    fn update(
        &mut self,
        depth_mm: &[f32],
        weights: Option<&[f32]>,
        intr: &CameraIntrinsics,
        cam_to_world: &Affine3A,
        remove: bool,
    ) {
        if depth_mm.len() != intr.pixel_count() || depth_mm.is_empty() {
            return;
        }
        let touched = self.touch(depth_mm, weights, intr, cam_to_world, !remove);
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
                            let pix = (v * w + u) as usize;
                            let d = depth_mm[pix];
                            if !valid(d) {
                                continue;
                            }
                            let wp = weights.map_or(1.0, |ws| ws[pix]);
                            // weight 0 or NaN: not used
                            if wp.is_nan() || wp <= 0.0 {
                                continue;
                            }
                            let sdf = d - cur.z;
                            if sdf < -p.trunc_mm {
                                continue;
                            }
                            let t = (sdf * inv_trunc).min(1.0);
                            let i = row + x;
                            let wo = block.weight[i];
                            if remove {
                                let wn = wo - wp;
                                if wn <= 1e-4 * wp {
                                    block.tsdf[i] = 1.0;
                                    block.weight[i] = 0.0;
                                } else {
                                    block.tsdf[i] =
                                        ((block.tsdf[i] * wo - t * wp) / wn).clamp(-1.0, 1.0);
                                    block.weight[i] = wn;
                                }
                            } else {
                                let wn = wo + wp;
                                block.tsdf[i] = (block.tsdf[i] * wo + t * wp) / wn;
                                block.weight[i] = wn.min(p.max_weight);
                            }
                        }
                    }
                }
            });
    }

    /// The blocks around the measured surface of a frame (pixels of weight
    /// above 0 when `weights` are given), allocated when `create`; returns
    /// the indices of the (existing) blocks the frame touches.
    fn touch(
        &mut self,
        depth_mm: &[f32],
        weights: Option<&[f32]>,
        intr: &CameraIntrinsics,
        cam_to_world: &Affine3A,
        create: bool,
    ) -> Vec<usize> {
        let p = self.params;
        let block_mm = p.voxel_mm * SIDE as f32;
        let used = |i: usize| valid(depth_mm[i]) && weights.is_none_or(|ws| ws[i] > 0.0);
        let zmin = (0..depth_mm.len())
            .into_par_iter()
            .with_min_len(4096)
            .filter(|&i| used(i))
            .map(|i| depth_mm[i])
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
                    if !used(v * w + u) {
                        continue;
                    }
                    let d = depth_mm[v * w + u];
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
            if !create {
                touched.extend(self.map.get(&key).copied());
                continue;
            }
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
        self.extract_mesh_with(0.0)
    }

    /// [`TsdfVolume::extract_mesh`] where only the voxels with an integration
    /// weight of at least `min_weight` (and above 0) count as observed: the
    /// surface stops where it was seen too little.
    pub fn extract_mesh_with(&self, min_weight: f32) -> TriMesh {
        let keys = self.sorted_keys();
        let mut map = BlockMap::default();
        map.reserve(keys.len());
        for (i, k) in keys.iter().enumerate() {
            map.insert(*k, i);
        }
        let values: Vec<[f32; VOXELS]> = keys
            .par_iter()
            .map(|k| {
                let b = &self.blocks[self.map[k]];
                std::array::from_fn(|j| {
                    let w = b.weight[j];
                    if w > 0.0 && w >= min_weight {
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

/// How [`depth_weights`] rates the pixels of a depth image.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DepthWeightParams {
    /// Pixels within this many pixels (city-block) of a depth edge get
    /// weight 0: the mixed and flying pixels along silhouettes and holes.
    pub erode_px: u32,
    /// Past the eroded band the weight grows linearly to full over this
    /// many pixels.
    pub ramp_px: u32,
    /// Largest depth step to a 4-neighbour, in pixel footprints
    /// (`depth / fx`), that is not a depth edge.
    pub max_slope: f32,
    /// Pixels whose surface turns more than this from the viewing ray
    /// (degrees) get weight 0; the others are weighted by the cosine.
    pub max_incidence_deg: f32,
    /// Depth of weight 1 (mm): the weight falls with the square of the
    /// depth, like the precision of the depth.
    pub reference_depth_mm: f32,
}

impl Default for DepthWeightParams {
    fn default() -> Self {
        Self {
            erode_px: 2,
            ramp_px: 3,
            max_slope: 4.0,
            max_incidence_deg: 75.0,
            reference_depth_mm: 200.0,
        }
    }
}

/// Confidence of every pixel of a depth image (row-major, `0` or NaN
/// invalid) for [`TsdfVolume::integrate_weighted`]: `0` within
/// [`DepthWeightParams::erode_px`] of a depth edge (an invalid pixel, the
/// image border, or a jump to a neighbour), then a ramp to full weight,
/// times the cosine of the angle between the surface (normal from the
/// pixels two away) and the viewing ray, times `(reference / depth)²`. A
/// size mismatch gives all zeros.
pub fn depth_weights(
    depth_mm: &[f32],
    intr: &CameraIntrinsics,
    params: &DepthWeightParams,
) -> Vec<f32> {
    let (w, h) = (intr.width as usize, intr.height as usize);
    let n = w * h;
    if depth_mm.len() != n || w < 3 || h < 3 {
        return vec![0.0; depth_mm.len()];
    }
    let edge: Vec<bool> = (0..n)
        .into_par_iter()
        .with_min_len(4096)
        .map(|i| {
            let d = depth_mm[i];
            let (u, v) = (i % w, i / w);
            if !valid(d) || u == 0 || v == 0 || u + 1 == w || v + 1 == h {
                return true;
            }
            let tol = params.max_slope * d / intr.fx;
            [i - 1, i + 1, i - w, i + w].iter().any(|&j| {
                let dn = depth_mm[j];
                !valid(dn) || (dn - d).abs() > tol
            })
        })
        .collect();
    // city-block distance to the nearest edge pixel (two passes)
    let mut dist: Vec<u32> = edge
        .iter()
        .map(|&e| if e { 0 } else { u32::MAX / 2 })
        .collect();
    for v in 0..h {
        for u in 0..w {
            let i = v * w + u;
            let mut d = dist[i];
            if u > 0 {
                d = d.min(dist[i - 1] + 1);
            }
            if v > 0 {
                d = d.min(dist[i - w] + 1);
            }
            dist[i] = d;
        }
    }
    for v in (0..h).rev() {
        for u in (0..w).rev() {
            let i = v * w + u;
            let mut d = dist[i];
            if u + 1 < w {
                d = d.min(dist[i + 1] + 1);
            }
            if v + 1 < h {
                d = d.min(dist[i + w] + 1);
            }
            dist[i] = d;
        }
    }
    let cos_min = params.max_incidence_deg.to_radians().cos();
    let r = 2usize;
    (0..n)
        .into_par_iter()
        .with_min_len(4096)
        .map(|i| {
            let e = dist[i];
            if e <= params.erode_px {
                return 0.0;
            }
            let ramp = if params.ramp_px == 0 {
                1.0
            } else {
                ((e - params.erode_px) as f32 / params.ramp_px as f32).min(1.0)
            };
            let (u, v) = (i % w, i / w);
            if u < r || v < r || u + r >= w || v + r >= h {
                return 0.0;
            }
            let at = |uu: usize, vv: usize| {
                let dn = depth_mm[vv * w + uu];
                valid(dn).then(|| intr.ray(uu as f32, vv as f32) * dn)
            };
            let (Some(left), Some(right), Some(up), Some(down)) =
                (at(u - r, v), at(u + r, v), at(u, v - r), at(u, v + r))
            else {
                return 0.0;
            };
            let normal = (down - up).cross(right - left);
            let ray = intr.ray(u as f32, v as f32);
            let len = normal.length() * ray.length();
            if len.is_nan() || len <= 0.0 {
                return 0.0;
            }
            let cos = normal.dot(ray).abs() / len;
            if cos < cos_min {
                return 0.0;
            }
            let depth = (params.reference_depth_mm / depth_mm[i]).powi(2);
            ramp * cos * depth
        })
        .collect()
}

#[cfg(test)]
mod tests;
