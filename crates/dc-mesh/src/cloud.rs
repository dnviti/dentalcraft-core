//! Point cloud container (scanner output before meshing).
//!
//! A [`PointCloud`] holds positions in millimetres with optional per-point
//! normals and RGB colours. Every editing helper returns a new cloud and keeps
//! the optional attributes aligned with the points.

use glam::{Affine3A, Vec3};

/// Unordered set of points with optional normals and colours.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct PointCloud {
    /// Point positions (millimetres).
    pub points: Vec<Vec3>,
    /// Optional unit normals (same length as `points`).
    pub normals: Option<Vec<Vec3>>,
    /// Optional RGB colours (same length as `points`).
    pub colors: Option<Vec<[u8; 3]>>,
}

impl PointCloud {
    /// Cloud with positions only.
    pub fn from_points(points: Vec<Vec3>) -> Self {
        Self {
            points,
            ..Self::default()
        }
    }

    /// Number of points.
    pub fn len(&self) -> usize {
        self.points.len()
    }

    /// `true` when the cloud has no points.
    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }

    /// Component-wise minimum and maximum corner; `None` when empty.
    pub fn aabb(&self) -> Option<(Vec3, Vec3)> {
        let first = *self.points.first()?;
        Some(
            self.points
                .iter()
                .fold((first, first), |(lo, hi), p| (lo.min(*p), hi.max(*p))),
        )
    }

    /// Cloud made of the points at `keep`, in that order. Out-of-range
    /// indices are skipped.
    pub fn retain_indices(&self, keep: &[u32]) -> PointCloud {
        let n = self.points.len();
        let idx: Vec<usize> = keep
            .iter()
            .map(|&i| i as usize)
            .filter(|&i| i < n)
            .collect();
        PointCloud {
            points: idx.iter().map(|&i| self.points[i]).collect(),
            normals: self
                .normals
                .as_ref()
                .map(|v| idx.iter().map(|&i| v[i]).collect()),
            colors: self
                .colors
                .as_ref()
                .map(|v| idx.iter().map(|&i| v[i]).collect()),
        }
    }

    /// Cloud without the points at `remove` (any order, duplicates and
    /// out-of-range indices are ignored); the other points keep their order.
    pub fn remove_indices(&self, remove: &[u32]) -> PointCloud {
        let mut drop = vec![false; self.points.len()];
        for &i in remove {
            if let Some(d) = drop.get_mut(i as usize) {
                *d = true;
            }
        }
        let keep: Vec<u32> = (0..self.points.len() as u32)
            .filter(|&i| !drop[i as usize])
            .collect();
        self.retain_indices(&keep)
    }

    /// Cloud transformed by `m`; normals use the inverse transpose of the
    /// linear part and are renormalised.
    pub fn transformed(&self, m: &Affine3A) -> PointCloud {
        let normal_matrix = if m.matrix3.determinant().abs() > f32::EPSILON {
            m.matrix3.inverse().transpose()
        } else {
            m.matrix3
        };
        PointCloud {
            points: self.points.iter().map(|&p| m.transform_point3(p)).collect(),
            normals: self.normals.as_ref().map(|v| {
                v.iter()
                    .map(|&n| (normal_matrix * n).normalize_or_zero())
                    .collect()
            }),
            colors: self.colors.clone(),
        }
    }

    /// Appends the points of `other`. An attribute survives only when both
    /// clouds have it (or `self` is empty, in which case `other`'s attributes
    /// are taken).
    pub fn extend(&mut self, other: &PointCloud) {
        if self.points.is_empty() {
            *self = other.clone();
            return;
        }
        if other.points.is_empty() {
            return;
        }
        self.points.extend_from_slice(&other.points);
        self.normals = match (self.normals.take(), &other.normals) {
            (Some(mut a), Some(b)) => {
                a.extend_from_slice(b);
                Some(a)
            }
            _ => None,
        };
        self.colors = match (self.colors.take(), &other.colors) {
            (Some(mut a), Some(b)) => {
                a.extend_from_slice(b);
                Some(a)
            }
            _ => None,
        };
    }

    /// Rough memory footprint in bytes.
    pub fn approx_bytes(&self) -> usize {
        use std::mem::size_of;
        self.points.len() * size_of::<Vec3>()
            + self
                .normals
                .as_ref()
                .map_or(0, |n| n.len() * size_of::<Vec3>())
            + self.colors.as_ref().map_or(0, |c| c.len() * 3)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample() -> PointCloud {
        PointCloud {
            points: vec![Vec3::ZERO, Vec3::X, Vec3::Y, Vec3::new(2.0, -1.0, 3.0)],
            normals: Some(vec![Vec3::Z; 4]),
            colors: Some(vec![[1, 2, 3], [4, 5, 6], [7, 8, 9], [10, 11, 12]]),
        }
    }

    #[test]
    fn aabb_and_len() {
        let c = sample();
        assert_eq!(c.len(), 4);
        assert!(!c.is_empty());
        assert_eq!(
            c.aabb(),
            Some((Vec3::new(0.0, -1.0, 0.0), Vec3::new(2.0, 1.0, 3.0)))
        );
        assert!(PointCloud::default().aabb().is_none());
    }

    #[test]
    fn retain_and_remove_keep_attributes_aligned() {
        let c = sample();
        let r = c.retain_indices(&[3, 1, 99]);
        assert_eq!(r.points, vec![Vec3::new(2.0, -1.0, 3.0), Vec3::X]);
        assert_eq!(r.colors.unwrap(), vec![[10, 11, 12], [4, 5, 6]]);
        let d = c.remove_indices(&[0, 2, 2, 50]);
        assert_eq!(d.points, vec![Vec3::X, Vec3::new(2.0, -1.0, 3.0)]);
        assert_eq!(d.normals.unwrap().len(), 2);
    }

    #[test]
    fn transform_moves_points_and_rotates_normals() {
        let c = sample();
        let m = Affine3A::from_rotation_x(std::f32::consts::FRAC_PI_2)
            * Affine3A::from_translation(Vec3::X);
        let t = c.transformed(&m);
        assert!((t.points[0] - Vec3::X).length() < 1e-6);
        for n in t.normals.unwrap() {
            assert!((n - Vec3::NEG_Y).length() < 1e-6);
        }
    }

    #[test]
    fn extend_drops_attributes_missing_on_one_side() {
        let mut a = sample();
        a.extend(&PointCloud::from_points(vec![Vec3::ONE]));
        assert_eq!(a.len(), 5);
        assert!(a.normals.is_none() && a.colors.is_none());
        let mut e = PointCloud::default();
        e.extend(&sample());
        assert_eq!(e, sample());
    }
}
