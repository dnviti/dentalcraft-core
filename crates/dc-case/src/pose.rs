//! Rigid transforms in double precision.

use std::ops::Mul;

use glam::{DMat4, DQuat, DVec3};
use serde::{Deserialize, Serialize};

/// Rigid transform `p ↦ q·p + t` (millimetres).
///
/// Serialized as `{"t": [x, y, z], "q": [x, y, z, w]}`.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Pose {
    /// Translation.
    pub t: DVec3,
    /// Rotation (unit quaternion).
    pub q: DQuat,
}

impl Default for Pose {
    fn default() -> Self {
        Self::IDENTITY
    }
}

impl Pose {
    /// The identity transform.
    pub const IDENTITY: Pose = Pose {
        t: DVec3::ZERO,
        q: DQuat::IDENTITY,
    };

    /// Pose from translation and rotation (the rotation is normalised).
    pub fn new(t: DVec3, q: DQuat) -> Self {
        Self {
            t,
            q: q.normalize(),
        }
    }

    /// Pure translation.
    pub fn from_translation(t: DVec3) -> Self {
        Self {
            t,
            q: DQuat::IDENTITY,
        }
    }

    /// Pure rotation.
    pub fn from_rotation(q: DQuat) -> Self {
        Self::new(DVec3::ZERO, q)
    }

    /// `self ∘ other`: applies `other` first, then `self`.
    pub fn compose(&self, other: &Pose) -> Pose {
        Pose {
            t: self.t + self.q * other.t,
            q: (self.q * other.q).normalize(),
        }
    }

    /// Inverse transform.
    pub fn inverse(&self) -> Pose {
        let qi = self.q.inverse();
        Pose {
            t: -(qi * self.t),
            q: qi,
        }
    }

    /// Transforms a point.
    pub fn apply_point(&self, p: DVec3) -> DVec3 {
        self.q * p + self.t
    }

    /// Transforms a direction (rotation only).
    pub fn apply_vector(&self, v: DVec3) -> DVec3 {
        self.q * v
    }

    /// Homogeneous 4×4 matrix.
    pub fn to_mat4(&self) -> DMat4 {
        DMat4::from_rotation_translation(self.q, self.t)
    }

    /// Rigid part of a 4×4 matrix (scale is discarded).
    pub fn from_mat4(m: &DMat4) -> Self {
        let (_, q, t) = m.to_scale_rotation_translation();
        Self::new(t, q)
    }

    /// `true` when translations differ by at most `eps_t` (mm) and rotations
    /// by at most `eps_angle` (radians).
    pub fn approx_eq(&self, other: &Pose, eps_t: f64, eps_angle: f64) -> bool {
        self.t.distance(other.t) <= eps_t && self.q.angle_between(other.q) <= eps_angle
    }
}

impl Mul for Pose {
    type Output = Pose;

    /// Same as [`Pose::compose`].
    fn mul(self, rhs: Pose) -> Pose {
        self.compose(&rhs)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample() -> Pose {
        Pose::new(
            DVec3::new(1.0, -2.0, 3.0),
            DQuat::from_rotation_z(0.7) * DQuat::from_rotation_x(0.3),
        )
    }

    #[test]
    fn inverse_and_compose() {
        let a = sample();
        let id = a.compose(&a.inverse());
        assert!(id.approx_eq(&Pose::IDENTITY, 1e-12, 1e-9));
        let b = Pose::new(DVec3::X, DQuat::from_rotation_y(1.1));
        let p = DVec3::new(0.3, 0.2, -5.0);
        let ab = a * b;
        assert!((ab.apply_point(p) - a.apply_point(b.apply_point(p))).length() < 1e-12);
        assert!((ab.to_mat4().transform_point3(p) - ab.apply_point(p)).length() < 1e-12);
        assert!(Pose::from_mat4(&a.to_mat4()).approx_eq(&a, 1e-12, 1e-9));
        assert!((a.apply_vector(DVec3::Z).length() - 1.0).abs() < 1e-12);
    }

    #[test]
    fn serde_layout() {
        let p = Pose::from_translation(DVec3::new(1.0, 2.0, 3.0));
        let v = serde_json::to_value(p).unwrap();
        assert_eq!(
            v,
            serde_json::json!({"t": [1.0, 2.0, 3.0], "q": [0.0, 0.0, 0.0, 1.0]})
        );
        assert_eq!(serde_json::from_value::<Pose>(v).unwrap(), p);
    }
}
