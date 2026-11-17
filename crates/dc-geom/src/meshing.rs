//! Surface reconstruction of an oriented point cloud (Hoppe et al. 1992).
//!
//! The signed distance at a grid voxel `x` is the weighted mean of
//! `dot(x - p, n)` over its `k` nearest points `p` with normals `n`
//! (Gaussian weights on the distance, neighbours facing away from the
//! nearest point's normal ignored, so thin walls do not mix). The field is
//! sampled on sparse 8³ blocks only within `max_dist_mm` of the points and
//! meshed by marching cubes over the sampled voxels (the implementation the
//! TSDF extraction uses): the
//! result is an open, crack-free surface following the cloud, with faces
//! counter-clockwise seen from the side the normals point to.

use glam::{IVec3, Vec3};
use rayon::prelude::*;

use dc_mesh::{PointCloud, TriMesh};

use crate::cloud::estimate_normals;
use crate::knn::KnnIndex;
use crate::mc::{self, BlockMap, MAX_VOXEL, SIDE, VOXELS};
use crate::{GeomError, Result};

/// Parameters of [`mesh_from_cloud`].
#[derive(Clone, Copy, Debug)]
pub struct CloudMeshParams {
    /// Grid voxel size, millimetres (the mesh resolution).
    pub voxel_mm: f32,
    /// Voxels farther than this from every point are left unknown
    /// (millimetres); `0` means `3 * voxel_mm`. At least `2 * voxel_mm` is used.
    pub max_dist_mm: f32,
    /// Neighbours averaged per voxel (at least 1).
    pub k: usize,
}

impl Default for CloudMeshParams {
    fn default() -> Self {
        Self {
            voxel_mm: 0.3,
            max_dist_mm: 0.0,
            k: 8,
        }
    }
}

/// Blocks evaluated between two cancellation checks.
const CHUNK: usize = 256;

/// Meshes `cloud` (see the module docs). Missing normals are estimated
/// with [`estimate_normals`] (k = 12, consistent orientation). `cancel` is
/// polled between chunks of work.
///
/// # Errors
/// [`GeomError::InvalidInput`] for a non-positive voxel size, fewer than
/// three usable points or a cloud too large for the grid;
/// [`GeomError::Cancelled`] when `cancel` returns `true`.
pub fn mesh_from_cloud(
    cloud: &PointCloud,
    params: &CloudMeshParams,
    cancel: &dyn Fn() -> bool,
) -> Result<TriMesh> {
    let vs = params.voxel_mm;
    if !vs.is_finite() || vs <= 0.0 {
        return Err(GeomError::InvalidInput(format!("voxel size {vs}")));
    }
    let max_dist = if params.max_dist_mm > 0.0 && params.max_dist_mm.is_finite() {
        params.max_dist_mm.max(2.0 * vs)
    } else {
        3.0 * vs
    };
    let k = params.k.max(1);

    // oriented points
    let mut owned;
    let src = match &cloud.normals {
        Some(n) if n.len() == cloud.len() => cloud,
        _ => {
            owned = cloud.clone();
            estimate_normals(&mut owned, 12, None);
            &owned
        }
    };
    if cancel() {
        return Err(GeomError::Cancelled);
    }
    let normals = src.normals.as_deref().unwrap_or_default();
    let (points, normals): (Vec<Vec3>, Vec<Vec3>) = src
        .points
        .iter()
        .zip(normals)
        .filter(|(p, n)| p.is_finite() && n.is_finite() && **n != Vec3::ZERO)
        .map(|(p, n)| (*p, n.normalize()))
        .unzip();
    if points.len() < 3 {
        return Err(GeomError::InvalidInput(
            "fewer than three oriented points".into(),
        ));
    }
    let limit = (MAX_VOXEL - 2 * SIDE) as f32 * vs;
    if points
        .iter()
        .any(|p| p.abs().max_element() + max_dist > limit)
    {
        return Err(GeomError::InvalidInput(
            "point cloud too large for the voxel size".into(),
        ));
    }

    // blocks within max_dist of a point
    let inv = 1.0 / vs;
    let mut packed: Vec<u64> = points
        .par_iter()
        .with_min_len(1024)
        .flat_map_iter(|&p| {
            let lo = mc::block_of(((p - max_dist) * inv).floor().as_ivec3());
            let hi = mc::block_of(((p + max_dist) * inv).ceil().as_ivec3());
            let mut out = Vec::with_capacity(8);
            for z in lo.z..=hi.z {
                for y in lo.y..=hi.y {
                    for x in lo.x..=hi.x {
                        out.push(mc::pack(IVec3::new(x, y, z)));
                    }
                }
            }
            out.into_iter()
        })
        .collect();
    packed.par_sort_unstable();
    packed.dedup();
    let mut keys: Vec<IVec3> = packed.into_iter().map(mc::unpack).collect();
    mc::sort_keys(&mut keys);
    if cancel() {
        return Err(GeomError::Cancelled);
    }

    // signed distances
    let index = KnnIndex::build(&points)?;
    let sigma2 = (vs * 1.5).powi(2);
    let eval = |x: Vec3| -> f32 {
        let nb = index.nearest_k(x, k);
        let Some(first) = nb.first() else {
            return f32::NAN;
        };
        if first.distance > max_dist {
            return f32::NAN;
        }
        let n0 = normals[first.index as usize];
        let (mut sum, mut wsum) = (0f32, 0f32);
        for n in &nb {
            let i = n.index as usize;
            if normals[i].dot(n0) < 0.5 {
                continue;
            }
            let w = (-(n.distance * n.distance) / sigma2).exp().max(1e-12);
            sum += w * (x - points[i]).dot(normals[i]);
            wsum += w;
        }
        sum / wsum
    };
    let mut values: Vec<[f32; VOXELS]> = Vec::with_capacity(keys.len());
    for chunk in keys.chunks(CHUNK) {
        if cancel() {
            return Err(GeomError::Cancelled);
        }
        let part: Vec<[f32; VOXELS]> = chunk
            .par_iter()
            .map(|&key| {
                let base = key * SIDE;
                std::array::from_fn(|i| eval((base + mc::local_coords(i)).as_vec3() * vs))
            })
            .collect();
        values.extend(part);
    }
    if cancel() {
        return Err(GeomError::Cancelled);
    }

    let mut map = BlockMap::default();
    map.reserve(keys.len());
    for (i, &k) in keys.iter().enumerate() {
        map.insert(k, i);
    }
    let field = mc::Field {
        keys: &keys,
        map: &map,
        values: &values,
        origin: Vec3::ZERO,
        voxel: vs,
        max_step: 2.0 * vs,
    };
    let (positions, tris) = mc::extract(&field);
    let mut mesh = TriMesh::new(positions, tris);
    mesh.update_normals();
    Ok(mesh)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Fibonacci samples of the cap of the sphere of radius `r` around the
    /// origin within `max_angle` of +Z, with outward normals.
    fn cap(n: usize, r: f32, max_angle: f32) -> PointCloud {
        let ga = std::f32::consts::PI * (3.0 - 5f32.sqrt());
        let mut cloud = PointCloud::default();
        let mut normals = Vec::new();
        for i in 0..n {
            let z = 1.0 - 2.0 * (i as f32 + 0.5) / n as f32;
            if z < max_angle.cos() {
                break;
            }
            let rr = (1.0 - z * z).sqrt();
            let t = ga * i as f32;
            let d = Vec3::new(rr * t.cos(), rr * t.sin(), z);
            cloud.points.push(d * r);
            normals.push(d);
        }
        cloud.normals = Some(normals);
        cloud
    }

    #[test]
    fn sphere_cap_gives_manifold_open_mesh_close_to_truth() {
        let r = 10.0;
        let cloud = cap(40_000, r, 60f32.to_radians());
        let params = CloudMeshParams {
            voxel_mm: 0.4,
            ..CloudMeshParams::default()
        };
        let mesh = mesh_from_cloud(&cloud, &params, &|| false).unwrap();
        assert!(mesh.face_count() > 1000);
        mesh.check().unwrap();
        let adj = mesh.edge_adjacency();
        assert_eq!(adj.non_manifold_edge_count(), 0);
        assert!(adj.boundary_edge_count() > 0, "open surface");
        assert_eq!(mesh.connected_components().len(), 1);
        for p in &mesh.positions {
            let e = (p.length() - r).abs();
            assert!(e < 0.5 * params.voxel_mm, "{p}: {e}");
        }
        for f in 0..mesh.face_count() {
            let [a, b, c] = mesh.triangle(f);
            let centre = (a + b + c) / 3.0;
            assert!(mesh.face_normal(f).dot(centre.normalize()) > 0.5);
        }
        let cap_area = 2.0 * std::f32::consts::PI * r * r * (1.0 - 60f32.to_radians().cos());
        let area = mesh.area();
        assert!(
            area > 0.95 * cap_area && area < 1.35 * cap_area,
            "{area} vs {cap_area}"
        );
    }

    #[test]
    fn normals_are_estimated_when_missing() {
        let mut cloud = cap(8_000, 8.0, 50f32.to_radians());
        cloud.normals = None;
        let params = CloudMeshParams {
            voxel_mm: 0.5,
            max_dist_mm: 0.0,
            k: 6,
        };
        let mesh = mesh_from_cloud(&cloud, &params, &|| false).unwrap();
        assert!(mesh.face_count() > 100);
        assert_eq!(mesh.edge_adjacency().non_manifold_edge_count(), 0);
        for p in &mesh.positions {
            assert!((p.length() - 8.0).abs() < 0.25, "{p}");
        }
    }

    #[test]
    fn invalid_input_and_cancel() {
        let cloud = cap(8_000, 8.0, 50f32.to_radians());
        let bad = CloudMeshParams {
            voxel_mm: 0.0,
            ..CloudMeshParams::default()
        };
        assert!(matches!(
            mesh_from_cloud(&cloud, &bad, &|| false),
            Err(GeomError::InvalidInput(_))
        ));
        assert!(matches!(
            mesh_from_cloud(&PointCloud::default(), &CloudMeshParams::default(), &|| {
                false
            }),
            Err(GeomError::InvalidInput(_))
        ));
        assert!(matches!(
            mesh_from_cloud(&cloud, &CloudMeshParams::default(), &|| true),
            Err(GeomError::Cancelled)
        ));
    }
}
