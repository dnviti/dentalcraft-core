//! k-nearest-neighbour and radius queries on point sets (WP13, port plan
//! B1.2), on top of a k-d tree (`kiddo`).
//!
//! The tree stores the points as `f64` (exact conversion of the `f32`
//! coordinates) and measures squared distances in `f64`, like the oracle
//! `knn_cpu` of the Blender add-on (`gpu_query.py`):
//!
//! * neighbours come nearest first, ties to the smallest point index
//!   (also across the k-th boundary: of several points at the same distance,
//!   the smallest indices are kept);
//! * a query with fewer than `k` points available is padded in the flat
//!   batch form with [`NO_NEIGHBOR`] and an infinite distance;
//! * a non-finite query has no neighbours (all padding).

use glam::Vec3;
use kiddo::{ImmutableKdTree, SquaredEuclidean};
use rayon::prelude::*;
use std::num::NonZero;

use crate::{GeomError, Result};

/// Index stored in the padding entries of [`KnnResult`].
pub const NO_NEIGHBOR: u32 = u32::MAX;

/// Minimum batch chunk handled by one rayon task.
const PAR_CHUNK: usize = 128;

/// One neighbour of a query point.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Neighbor {
    /// Index of the neighbour in the point set.
    pub index: u32,
    /// Euclidean distance to the query point.
    pub distance: f32,
}

/// Batched k-nearest-neighbour result: `m` queries times `k` neighbours,
/// row-major, nearest first.
#[derive(Debug, Clone, PartialEq)]
pub struct KnnResult {
    /// Neighbours per query.
    pub k: usize,
    /// Point indices, `queries * k` entries ([`NO_NEIGHBOR`] for padding).
    pub indices: Vec<u32>,
    /// Distances, `queries * k` entries (`f32::INFINITY` for padding).
    pub distances: Vec<f32>,
}

impl KnnResult {
    /// Number of queries.
    pub fn queries(&self) -> usize {
        self.indices.len().checked_div(self.k).unwrap_or(0)
    }

    /// Indices of the neighbours of query `i`.
    pub fn indices_of(&self, i: usize) -> &[u32] {
        &self.indices[i * self.k..(i + 1) * self.k]
    }

    /// Distances of the neighbours of query `i`.
    pub fn distances_of(&self, i: usize) -> &[f32] {
        &self.distances[i * self.k..(i + 1) * self.k]
    }
}

/// k-d tree over a point set.
pub struct KnnIndex {
    tree: Option<ImmutableKdTree<f64, 3>>,
    len: usize,
}

impl std::fmt::Debug for KnnIndex {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("KnnIndex").field("len", &self.len).finish()
    }
}

impl KnnIndex {
    /// Builds the tree over `points`; the index of a point in the slice is
    /// the index reported by the queries. An empty slice gives an empty
    /// index whose queries return nothing.
    ///
    /// # Errors
    /// [`GeomError::InvalidInput`] when a point is not finite or when there
    /// are more than `u32::MAX - 1` points.
    pub fn build(points: &[Vec3]) -> Result<Self> {
        if points.len() >= NO_NEIGHBOR as usize {
            return Err(GeomError::InvalidInput("too many points".into()));
        }
        if let Some(i) = points.iter().position(|p| !p.is_finite()) {
            return Err(GeomError::InvalidInput(format!("point {i} is not finite")));
        }
        if points.is_empty() {
            return Ok(Self { tree: None, len: 0 });
        }
        let pts: Vec<[f64; 3]> = points
            .iter()
            .map(|p| [p.x as f64, p.y as f64, p.z as f64])
            .collect();
        let tree = ImmutableKdTree::new_from_slice(&pts)
            .map_err(|e| GeomError::InvalidInput(format!("k-d tree: {e:?}")))?;
        Ok(Self {
            tree: Some(tree),
            len: points.len(),
        })
    }

    /// Number of points.
    pub fn len(&self) -> usize {
        self.len
    }

    /// `true` for an empty point set.
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// The nearest point to `q`, if any.
    pub fn nearest(&self, q: Vec3) -> Option<Neighbor> {
        self.nearest_k(q, 1).into_iter().next()
    }

    /// The `k` nearest points to `q`, nearest first, ties to the smallest
    /// index. Fewer than `k` entries when the set is smaller; empty for
    /// `k == 0`, an empty set or a non-finite `q`.
    pub fn nearest_k(&self, q: Vec3, k: usize) -> Vec<Neighbor> {
        let (Some(tree), Some(kz)) = (&self.tree, NonZero::new(k.min(self.len))) else {
            return Vec::new();
        };
        if !q.is_finite() {
            return Vec::new();
        }
        let qa = [q.x as f64, q.y as f64, q.z as f64];
        let k = kz.get();
        // one extra result tells whether the k-th distance is tied
        let probe = NonZero::new((k + 1).min(self.len)).unwrap_or(kz);
        let mut found: Vec<(f64, u32)> = tree
            .query(&qa)
            .nearest_n::<SquaredEuclidean<f64>>(probe)
            .execute()
            .into_iter()
            .map(|r| (r.distance, r.item))
            .collect();
        found.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));
        if found.len() > k && found[k].0 == found[k - 1].0 {
            // tie across the boundary: take the whole tie group, keep the
            // smallest indices
            let dk = found[k - 1].0;
            found = tree
                .query(&qa)
                .within::<SquaredEuclidean<f64>>(dk)
                .unsorted()
                .execute()
                .into_iter()
                .map(|r| (r.distance, r.item))
                .collect();
            found.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));
        }
        found.truncate(k);
        found
            .into_iter()
            .map(|(d2, index)| Neighbor {
                index,
                distance: d2.sqrt() as f32,
            })
            .collect()
    }

    /// Every point within `radius` of `q` (closed ball), nearest first, ties
    /// to the smallest index.
    pub fn within_radius(&self, q: Vec3, radius: f32) -> Vec<Neighbor> {
        let Some(tree) = &self.tree else {
            return Vec::new();
        };
        if !q.is_finite() || radius.is_nan() || radius < 0.0 {
            return Vec::new();
        }
        let qa = [q.x as f64, q.y as f64, q.z as f64];
        let r = radius as f64;
        let mut found: Vec<(f64, u32)> = tree
            .query(&qa)
            .within::<SquaredEuclidean<f64>>(r * r)
            .unsorted()
            .execute()
            .into_iter()
            .map(|r| (r.distance, r.item))
            .collect();
        found.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));
        found
            .into_iter()
            .map(|(d2, index)| Neighbor {
                index,
                distance: d2.sqrt() as f32,
            })
            .collect()
    }

    /// [`KnnIndex::nearest_k`] for many queries, in parallel, as one flat
    /// row-major result padded to `k` columns.
    pub fn knn_batch(&self, queries: &[Vec3], k: usize) -> KnnResult {
        let mut indices = vec![NO_NEIGHBOR; queries.len() * k];
        let mut distances = vec![f32::INFINITY; queries.len() * k];
        if k > 0 {
            indices
                .par_chunks_mut(k)
                .zip(distances.par_chunks_mut(k))
                .zip(queries.par_iter())
                .with_min_len(PAR_CHUNK)
                .for_each(|((idx, dist), &q)| {
                    for (j, n) in self.nearest_k(q, k).into_iter().enumerate() {
                        idx[j] = n.index;
                        dist[j] = n.distance;
                    }
                });
        }
        KnnResult {
            k,
            indices,
            distances,
        }
    }

    /// [`KnnIndex::nearest`] for many queries, in parallel. One entry per
    /// query, in input order.
    pub fn nearest_batch(&self, queries: &[Vec3]) -> Vec<Option<Neighbor>> {
        queries
            .par_iter()
            .with_min_len(PAR_CHUNK)
            .map(|&q| self.nearest(q))
            .collect()
    }

    /// [`KnnIndex::within_radius`] for many queries, in parallel.
    pub fn within_radius_batch(&self, queries: &[Vec3], radius: f32) -> Vec<Vec<Neighbor>> {
        queries
            .par_iter()
            .with_min_len(PAR_CHUNK)
            .map(|&q| self.within_radius(q, radius))
            .collect()
    }
}

/// One-shot k-nearest neighbours of every row of `queries` in `points`
/// (builds a [`KnnIndex`] and runs [`KnnIndex::knn_batch`]).
///
/// # Errors
/// As [`KnnIndex::build`].
pub fn knn(points: &[Vec3], queries: &[Vec3], k: usize) -> Result<KnnResult> {
    Ok(KnnIndex::build(points)?.knn_batch(queries, k))
}

#[cfg(test)]
mod tests;
