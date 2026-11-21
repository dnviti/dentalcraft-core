//! The turn that lays a footprint's long side along the plate's X axis (the
//! long side of the Saturn plate). The hull is the crate's single one,
//! [`crate::layout::convex_hull`].

use glam::{DVec2, Quat, Vec2, Vec3};

use crate::layout::convex_hull;

/// Turn about Z that puts the smallest enclosing rectangle of `points`
/// axis-aligned, with its long side along X. It is built from the
/// rectangle's edge direction, without trigonometry. Ties go to the first
/// hull edge.
pub(crate) fn long_side_turn(points: &[Vec2]) -> Quat {
    let hull = convex_hull(points.iter().map(|p| p.as_dvec2()).collect());
    let mut best: Option<(f64, DVec2, f64, f64)> = None;
    for i in 0..hull.len() {
        let u = (hull[(i + 1) % hull.len()] - hull[i]).normalize_or_zero();
        if u == DVec2::ZERO {
            continue;
        }
        let v = u.perp();
        let (mut umin, mut umax, mut vmin, mut vmax) = (f64::MAX, f64::MIN, f64::MAX, f64::MIN);
        for p in &hull {
            let (a, b) = (p.dot(u), p.dot(v));
            umin = umin.min(a);
            umax = umax.max(a);
            vmin = vmin.min(b);
            vmax = vmax.max(b);
        }
        let area = (umax - umin) * (vmax - vmin);
        if best.is_none_or(|b| area < b.0 * (1.0 - 1e-9)) {
            best = Some((area, u, umax - umin, vmax - vmin));
        }
    }
    let Some((_, u, along, across)) = best else {
        return Quat::IDENTITY;
    };
    let mut long = if along >= across { u } else { u.perp() };
    // Of the two directions of the long side, turn the one within
    // (−90°, 90°] onto +X: the smaller turn.
    if long.x < 0.0 || (long.x == 0.0 && long.y < 0.0) {
        long = -long;
    }
    Quat::from_rotation_arc(long.as_vec2().normalize().extend(0.0), Vec3::X)
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::Mat2;

    #[test]
    fn turn_lays_a_rotated_rectangle_along_x() {
        let rect = [
            Vec2::new(-30.0, -10.0),
            Vec2::new(30.0, -10.0),
            Vec2::new(30.0, 10.0),
            Vec2::new(-30.0, 10.0),
        ];
        for deg in [0.0f32, 17.0, 63.0, 90.0, 131.0, -44.0] {
            let rot = Mat2::from_angle(deg.to_radians());
            let pts: Vec<Vec2> = rect.iter().map(|p| rot * *p).collect();
            let turn = long_side_turn(&pts);
            let back: Vec<Vec2> = pts
                .iter()
                .map(|p| (turn * p.extend(0.0)).truncate())
                .collect();
            let w = back.iter().map(|p| p.x).fold(f32::MIN, f32::max)
                - back.iter().map(|p| p.x).fold(f32::MAX, f32::min);
            let h = back.iter().map(|p| p.y).fold(f32::MIN, f32::max)
                - back.iter().map(|p| p.y).fold(f32::MAX, f32::min);
            assert!(
                (w - 60.0).abs() < 1e-3 && (h - 20.0).abs() < 1e-3,
                "{deg}°: {w} × {h}"
            );
        }
        assert_eq!(long_side_turn(&[Vec2::ONE]), Quat::IDENTITY);
    }
}
