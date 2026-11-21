//! Layer sampling heights (print plan §5).
//!
//! Layer `i` spans z ∈ [i·h, (i + 1)·h] and is sampled on the plane through
//! its centre, z = (i + ½)·h. Every sampling height comes from
//! [`centre_mm`], computed from integers ((2i + 1)·h_µm / 2000) and rounded
//! once, so it is the same on every platform and never accumulates error.
//!
//! A triangle crosses layer `i` when z_min ≤ (i + ½)·h < z_max: a vertex on a
//! sampling plane counts as below it. A solid spanning [z0, z1) therefore
//! lights exactly the layers whose centre lies in [z0, z1).

/// Height of layer `index`'s sampling plane, mm: (index + ½)·h.
pub(crate) fn centre_mm(index: u32, layer_um: u32) -> f64 {
    ((2 * u128::from(index) + 1) * u128::from(layer_um)) as f64 / 2000.0
}

/// The first layer whose sampling plane is at or above `z_mm`, capped at
/// `limit`; 0 when `z_mm` is below the first plane.
///
/// Exact with respect to [`centre_mm`]: the result `i` satisfies
/// centre(i − 1) < z ≤ centre(i) for every finite input, whatever the
/// rounding of the first guess. So the number of sampling planes strictly
/// below a height `top` is `first_layer_at_or_above(top, ..)`.
pub(crate) fn first_layer_at_or_above(z_mm: f64, layer_um: u32, limit: u32) -> u32 {
    debug_assert!(layer_um > 0);
    let guess = (z_mm * 1000.0 / f64::from(layer_um) - 0.5).ceil();
    // `as` saturates (NaN gives 0); the loops below make the result exact.
    let mut i = if guess >= f64::from(limit) {
        limit
    } else {
        guess.max(0.0) as u32
    };
    while i > 0 && centre_mm(i - 1, layer_um) >= z_mm {
        i -= 1;
    }
    while i < limit && centre_mm(i, layer_um) < z_mm {
        i += 1;
    }
    i
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn centres_are_exact_decimals() {
        assert_eq!(centre_mm(0, 50), 0.025);
        assert_eq!(centre_mm(199, 50), 9.975);
        assert_eq!(centre_mm(3, 30), 0.105);
    }

    #[test]
    fn first_layer_is_exact_at_and_between_planes() {
        // On a plane: that plane's layer.
        assert_eq!(first_layer_at_or_above(0.025, 50, 1000), 0);
        assert_eq!(first_layer_at_or_above(9.975, 50, 1000), 199);
        // Between planes: the next one up.
        assert_eq!(first_layer_at_or_above(0.0, 50, 1000), 0);
        assert_eq!(first_layer_at_or_above(10.0, 50, 1000), 200);
        assert_eq!(first_layer_at_or_above(-3.0, 50, 1000), 0);
        // Capped.
        assert_eq!(first_layer_at_or_above(1e30, 50, 1000), 1000);
        // f32 noise on a whole multiple does not open a layer.
        assert_eq!(first_layer_at_or_above(f64::from(0.1f32), 50, 1000), 2);
        for i in 0..5000 {
            let z = centre_mm(i, 30);
            assert_eq!(first_layer_at_or_above(z, 30, u32::MAX), i);
            assert_eq!(first_layer_at_or_above(z + 1e-9, 30, u32::MAX), i + 1);
        }
    }
}
