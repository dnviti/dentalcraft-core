//! Point cloud processing: downsampling, outlier and isolation removal,
//! normal estimation, smoothing and geometric selections.
//!
//! Functions returning `Vec<u32>` give point indices in increasing order, to
//! be applied with [`PointCloud::remove_indices`] (removal filters) or
//! [`PointCloud::retain_indices`].

use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap};

use glam::{IVec3, Vec3};
use rayon::prelude::*;

use dc_mesh::PointCloud;

use crate::knn::{KnnIndex, NO_NEIGHBOR};

/// Builds the k-d tree of the finite points; non-finite points get no
/// neighbours and are never returned as neighbours.
fn index_of(cloud: &PointCloud) -> KnnIndex {
    let pts: Vec<Vec3> = cloud
        .points
        .iter()
        .map(|p| {
            if p.is_finite() {
                *p
            } else {
                Vec3::splat(f32::MAX / 4.0)
            }
        })
        .collect();
    KnnIndex::build(&pts).unwrap_or_else(|_| KnnIndex::build(&[]).expect("empty index"))
}

/// One point per occupied voxel of size `voxel_mm` (grid anchored at the
/// origin): the centroid of its points, with averaged (renormalised)
/// normals and averaged colours. Voxels come in the order of their first
/// point. A non-positive voxel size returns a copy.
pub fn voxel_downsample(cloud: &PointCloud, voxel_mm: f32) -> PointCloud {
    if voxel_mm.is_nan() || voxel_mm <= 0.0 || voxel_mm.is_infinite() {
        return cloud.clone();
    }
    let inv = 1.0 / voxel_mm;
    let mut slot: HashMap<IVec3, usize> = HashMap::with_capacity(cloud.len() / 4);
    let mut sum_p: Vec<Vec3> = Vec::new();
    let mut sum_n: Vec<Vec3> = Vec::new();
    let mut sum_c: Vec<[u32; 3]> = Vec::new();
    let mut count: Vec<u32> = Vec::new();
    for (i, p) in cloud.points.iter().enumerate() {
        if !p.is_finite() {
            continue;
        }
        let key = (*p * inv).floor().as_ivec3();
        let s = *slot.entry(key).or_insert_with(|| {
            sum_p.push(Vec3::ZERO);
            sum_n.push(Vec3::ZERO);
            sum_c.push([0; 3]);
            count.push(0);
            count.len() - 1
        });
        sum_p[s] += *p;
        count[s] += 1;
        if let Some(n) = &cloud.normals {
            sum_n[s] += n[i];
        }
        if let Some(c) = &cloud.colors {
            for k in 0..3 {
                sum_c[s][k] += u32::from(c[i][k]);
            }
        }
    }
    PointCloud {
        points: sum_p
            .iter()
            .zip(&count)
            .map(|(p, &c)| *p / c as f32)
            .collect(),
        normals: cloud
            .normals
            .as_ref()
            .map(|_| sum_n.iter().map(|n| n.normalize_or_zero()).collect()),
        colors: cloud.colors.as_ref().map(|_| {
            sum_c
                .iter()
                .zip(&count)
                .map(|(c, &n)| c.map(|v| ((v + n / 2) / n) as u8))
                .collect()
        }),
    }
}

/// Statistical outlier removal: a point whose mean distance to its `k`
/// nearest neighbours exceeds the global mean of that measure by more than
/// `std_ratio` standard deviations is an outlier. Returns the outliers.
pub fn statistical_outliers(cloud: &PointCloud, k: usize, std_ratio: f32) -> Vec<u32> {
    if cloud.len() <= k.max(1) || k == 0 {
        return Vec::new();
    }
    let index = index_of(cloud);
    let mean_d: Vec<f64> = cloud
        .points
        .par_iter()
        .with_min_len(256)
        .map(|&p| {
            let nb = index.nearest_k(p, k + 1);
            let d: Vec<f32> = nb.iter().skip(1).map(|n| n.distance).collect();
            if d.is_empty() || !p.is_finite() {
                f64::INFINITY
            } else {
                d.iter().map(|&x| f64::from(x)).sum::<f64>() / d.len() as f64
            }
        })
        .collect();
    let finite: Vec<f64> = mean_d.iter().copied().filter(|d| d.is_finite()).collect();
    if finite.is_empty() {
        return Vec::new();
    }
    let mu = finite.iter().sum::<f64>() / finite.len() as f64;
    let var = finite.iter().map(|d| (d - mu) * (d - mu)).sum::<f64>() / finite.len() as f64;
    let limit = mu + f64::from(std_ratio) * var.sqrt();
    (0..cloud.len() as u32)
        .filter(|&i| mean_d[i as usize] > limit)
        .collect()
}

/// Radius outlier removal: points with fewer than `min_neighbors` other
/// points within `radius_mm`. Returns the outliers.
pub fn radius_outliers(cloud: &PointCloud, radius_mm: f32, min_neighbors: usize) -> Vec<u32> {
    if cloud.is_empty() || min_neighbors == 0 {
        return Vec::new();
    }
    let index = index_of(cloud);
    let keep: Vec<bool> = cloud
        .points
        .par_iter()
        .with_min_len(256)
        .map(|&p| {
            p.is_finite()
                && index
                    .nearest_k(p, min_neighbors + 1)
                    .get(min_neighbors)
                    .is_some_and(|n| n.distance <= radius_mm)
        })
        .collect();
    (0..cloud.len() as u32)
        .filter(|&i| !keep[i as usize])
        .collect()
}

/// Mean and unit normal (eigenvector of the smallest eigenvalue of the
/// covariance) of `pts`; `None` for fewer than three points.
fn pca_plane(pts: &[Vec3]) -> Option<(Vec3, Vec3)> {
    if pts.len() < 3 {
        return None;
    }
    let mean = pts.iter().copied().sum::<Vec3>() / pts.len() as f32;
    let mut c = [[0f64; 3]; 3];
    for p in pts {
        let d = (*p - mean).to_array().map(f64::from);
        for (i, row) in c.iter_mut().enumerate() {
            for (j, v) in row.iter_mut().enumerate() {
                *v += d[i] * d[j];
            }
        }
    }
    let m = nalgebra::Matrix3::new(
        c[0][0], c[0][1], c[0][2], c[1][0], c[1][1], c[1][2], c[2][0], c[2][1], c[2][2],
    );
    let eig = m.symmetric_eigen();
    let (imin, _) = eig
        .eigenvalues
        .iter()
        .enumerate()
        .min_by(|a, b| a.1.total_cmp(b.1))?;
    let v = eig.eigenvectors.column(imin);
    let n = Vec3::new(v[0] as f32, v[1] as f32, v[2] as f32).normalize_or_zero();
    (n != Vec3::ZERO).then_some((mean, n))
}

/// Edge of the orientation propagation graph (max-heap on similarity).
#[derive(PartialEq)]
struct Edge(f32, u32);

impl Eq for Edge {}

impl PartialOrd for Edge {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for Edge {
    fn cmp(&self, other: &Self) -> Ordering {
        self.0.total_cmp(&other.0).then(other.1.cmp(&self.1))
    }
}

/// Estimates unit normals by PCA over the `k` nearest neighbours (`k` is at
/// least 3) and orients them:
///
/// * toward `viewpoint` when given;
/// * otherwise like the normals the cloud already had (when present);
/// * otherwise consistently by propagation over the k-nearest graph
///   (Hoppe 1992, maximum-similarity spanning tree), starting in each
///   connected part from the point farthest from the centroid, oriented
///   away from the centroid ("outward").
///
/// Points without a usable neighbourhood get a zero normal.
pub fn estimate_normals(cloud: &mut PointCloud, k: usize, viewpoint: Option<Vec3>) {
    let n = cloud.len();
    if n == 0 {
        cloud.normals = Some(Vec::new());
        return;
    }
    let k = k.max(3);
    let index = index_of(cloud);
    let knn = index.knn_batch(&cloud.points, k + 1);
    let pts = &cloud.points;
    let mut normals: Vec<Vec3> = (0..n)
        .into_par_iter()
        .with_min_len(256)
        .map(|i| {
            let nb: Vec<Vec3> = knn
                .indices_of(i)
                .iter()
                .filter(|&&j| j != NO_NEIGHBOR)
                .map(|&j| pts[j as usize])
                .collect();
            pca_plane(&nb).map_or(Vec3::ZERO, |(_, n)| n)
        })
        .collect();
    if let Some(vp) = viewpoint {
        for (nrm, p) in normals.iter_mut().zip(pts) {
            if nrm.dot(vp - *p) < 0.0 {
                *nrm = -*nrm;
            }
        }
    } else if let Some(old) = cloud.normals.as_ref().filter(|o| o.len() == n) {
        for (nrm, o) in normals.iter_mut().zip(old) {
            if nrm.dot(*o) < 0.0 {
                *nrm = -*nrm;
            }
        }
    } else {
        orient_by_propagation(pts, &mut normals, &knn);
    }
    cloud.normals = Some(normals);
}

fn orient_by_propagation(pts: &[Vec3], normals: &mut [Vec3], knn: &crate::knn::KnnResult) {
    let n = pts.len();
    // symmetric neighbour lists
    let mut adj: Vec<Vec<u32>> = vec![Vec::new(); n];
    for i in 0..n {
        for &j in knn.indices_of(i) {
            if j != NO_NEIGHBOR && j as usize != i {
                adj[i].push(j);
                adj[j as usize].push(i as u32);
            }
        }
    }
    let centroid = pts.iter().copied().filter(|p| p.is_finite()).sum::<Vec3>() / n as f32;
    let mut done = vec![false; n];
    let mut order: Vec<u32> = (0..n as u32).collect();
    order.sort_by(|&a, &b| {
        let da = pts[a as usize].distance_squared(centroid);
        let db = pts[b as usize].distance_squared(centroid);
        db.total_cmp(&da).then(a.cmp(&b))
    });
    let mut heap = BinaryHeap::new();
    for &seed in &order {
        if done[seed as usize] {
            continue;
        }
        let s = seed as usize;
        if normals[s].dot(pts[s] - centroid) < 0.0 {
            normals[s] = -normals[s];
        }
        done[s] = true;
        heap.clear();
        heap.extend(
            adj[s]
                .iter()
                .map(|&j| Edge(normals[s].dot(normals[j as usize]).abs(), j)),
        );
        // the heap carries (similarity, target); the parent is the most
        // similar already oriented neighbour when the target is popped
        while let Some(Edge(_, j)) = heap.pop() {
            let ju = j as usize;
            if done[ju] {
                continue;
            }
            // orient against the most similar already oriented neighbour
            let parent = adj[ju]
                .iter()
                .filter(|&&p| done[p as usize])
                .max_by(|&&a, &&b| {
                    let sa = normals[a as usize].dot(normals[ju]).abs();
                    let sb = normals[b as usize].dot(normals[ju]).abs();
                    sa.total_cmp(&sb).then(b.cmp(&a))
                })
                .copied();
            if let Some(p) = parent
                && normals[p as usize].dot(normals[ju]) < 0.0
            {
                normals[ju] = -normals[ju];
            }
            done[ju] = true;
            for &q in &adj[ju] {
                if !done[q as usize] {
                    heap.push(Edge(normals[ju].dot(normals[q as usize]).abs(), q));
                }
            }
        }
    }
}

/// Moves every point toward the least-squares plane of its `k` nearest
/// neighbours by `strength` (0 = no change, 1 = onto the plane), `iterations`
/// times. Normals, when present, are replaced by the plane normals oriented
/// like the old ones; colours are kept.
pub fn smooth_cloud(cloud: &PointCloud, k: usize, iterations: u32, strength: f32) -> PointCloud {
    let mut out = cloud.clone();
    let k = k.max(3);
    let s = strength.clamp(0.0, 1.0);
    for _ in 0..iterations {
        if out.len() <= 3 {
            break;
        }
        let index = index_of(&out);
        let pts = &out.points;
        let res: Vec<(Vec3, Option<Vec3>)> = pts
            .par_iter()
            .with_min_len(256)
            .map(|&p| {
                let nb: Vec<Vec3> = index
                    .nearest_k(p, k + 1)
                    .iter()
                    .map(|n| pts[n.index as usize])
                    .collect();
                match pca_plane(&nb) {
                    Some((c, n)) => (p - n * n.dot(p - c) * s, Some(n)),
                    None => (p, None),
                }
            })
            .collect();
        let old_normals = out.normals.take();
        out.normals = old_normals.map(|old| {
            old.iter()
                .zip(&res)
                .map(|(o, (_, n))| match n {
                    Some(n) if n.dot(*o) < 0.0 => -*n,
                    Some(n) => *n,
                    None => *o,
                })
                .collect()
        });
        out.points = res.into_iter().map(|(p, _)| p).collect();
    }
    out
}

/// Indices of the points inside the box `[min, max]` (inclusive).
pub fn select_in_aabb(cloud: &PointCloud, min: Vec3, max: Vec3) -> Vec<u32> {
    let (lo, hi) = (min.min(max), min.max(max));
    select(cloud, |p| p.cmpge(lo).all() && p.cmple(hi).all())
}

/// Indices of the points within `radius` of `center` (inclusive).
pub fn select_in_sphere(cloud: &PointCloud, center: Vec3, radius: f32) -> Vec<u32> {
    let r2 = radius * radius;
    select(cloud, |p| p.distance_squared(center) <= r2)
}

/// Indices of the points strictly on the side of the plane through `point`
/// that `normal` points to.
pub fn select_by_plane(cloud: &PointCloud, point: Vec3, normal: Vec3) -> Vec<u32> {
    let n = normal.normalize_or_zero();
    if n == Vec3::ZERO {
        return Vec::new();
    }
    select(cloud, |p| n.dot(p - point) > 0.0)
}

fn select(cloud: &PointCloud, f: impl Fn(Vec3) -> bool) -> Vec<u32> {
    (0..cloud.len() as u32)
        .filter(|&i| f(cloud.points[i as usize]))
        .collect()
}

fn find(parent: &mut [u32], mut x: u32) -> u32 {
    while parent[x as usize] != x {
        let p = parent[parent[x as usize] as usize];
        parent[x as usize] = p;
        x = p;
    }
    x
}

/// Isolation removal: groups the points into clusters connected by steps of
/// at most `radius_mm` and returns the indices of the points that are NOT in
/// the largest cluster (ties: the cluster holding the smallest index).
pub fn largest_cluster(cloud: &PointCloud, radius_mm: f32) -> Vec<u32> {
    let n = cloud.len();
    if n == 0 || radius_mm.is_nan() || radius_mm < 0.0 {
        return Vec::new();
    }
    let index = index_of(cloud);
    let mut parent: Vec<u32> = (0..n as u32).collect();
    const CHUNK: usize = 1 << 15;
    for start in (0..n).step_by(CHUNK) {
        let end = (start + CHUNK).min(n);
        let nbs: Vec<Vec<u32>> = cloud.points[start..end]
            .par_iter()
            .with_min_len(256)
            .enumerate()
            .map(|(o, &p)| {
                let i = (start + o) as u32;
                index
                    .within_radius(p, radius_mm)
                    .into_iter()
                    .map(|nb| nb.index)
                    .filter(|&j| j > i)
                    .collect()
            })
            .collect();
        for (o, list) in nbs.into_iter().enumerate() {
            let i = (start + o) as u32;
            for j in list {
                let (a, b) = (find(&mut parent, i), find(&mut parent, j));
                if a != b {
                    parent[a.max(b) as usize] = a.min(b);
                }
            }
        }
    }
    let mut size = vec![0u32; n];
    let roots: Vec<u32> = (0..n as u32).map(|i| find(&mut parent, i)).collect();
    for &r in &roots {
        size[r as usize] += 1;
    }
    let best = (0..n as u32)
        .max_by(|&a, &b| size[a as usize].cmp(&size[b as usize]).then(b.cmp(&a)))
        .unwrap_or(0);
    (0..n as u32)
        .filter(|&i| roots[i as usize] != best)
        .collect()
}

#[cfg(test)]
mod tests;
