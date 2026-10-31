//! Vertex welding of triangle soups with a tolerance based spatial hash.

use std::collections::HashMap;
use std::hash::{BuildHasherDefault, Hasher};

use glam::Vec3;

use crate::TriMesh;

/// Fast non-cryptographic hasher (FxHash style) for grid keys.
#[derive(Default)]
struct Fx(u64);

impl Fx {
    #[inline]
    fn add(&mut self, w: u64) {
        self.0 = (self.0.rotate_left(5) ^ w).wrapping_mul(0x517c_c1b7_2722_0a95);
    }
}

impl Hasher for Fx {
    #[inline]
    fn finish(&self) -> u64 {
        self.0 ^ (self.0 >> 32)
    }

    fn write(&mut self, bytes: &[u8]) {
        for &b in bytes {
            self.add(u64::from(b));
        }
    }

    #[inline]
    fn write_i64(&mut self, i: i64) {
        self.add(i as u64);
    }
}

type Key = (i64, i64, i64);

/// Welds a triangle soup (3 consecutive positions per triangle) into an
/// indexed mesh. Positions are snapped to a grid of `rel_tol * bbox diagonal`
/// and equal cells are merged; the first position seen is kept. Bit-identical
/// positions (the usual STL case) always merge. With `rel_tol <= 0` only
/// bit-identical positions merge.
pub(super) fn weld_soup(soup: &[Vec3], rel_tol: f32) -> TriMesh {
    debug_assert_eq!(soup.len() % 3, 0);
    let cell = crate::Aabb::from_points(soup)
        .map(|b| f64::from(rel_tol) * f64::from(b.diagonal()))
        .filter(|c| c.is_finite() && *c > 0.0);

    let key = |p: Vec3| -> Key {
        match cell {
            // `+ 0.0` maps -0.0 to 0.0
            None => (
                i64::from((p.x + 0.0).to_bits()),
                i64::from((p.y + 0.0).to_bits()),
                i64::from((p.z + 0.0).to_bits()),
            ),
            Some(c) => (
                (f64::from(p.x) / c).round() as i64,
                (f64::from(p.y) / c).round() as i64,
                (f64::from(p.z) / c).round() as i64,
            ),
        }
    };

    let mut map: HashMap<Key, u32, BuildHasherDefault<Fx>> =
        HashMap::with_capacity_and_hasher(soup.len() / 2 + 16, BuildHasherDefault::default());
    let mut positions: Vec<Vec3> = Vec::with_capacity(soup.len() / 2 + 16);
    let mut tris: Vec<[u32; 3]> = Vec::with_capacity(soup.len() / 3);
    for tri in soup.as_chunks::<3>().0 {
        let mut t = [0u32; 3];
        for (slot, &p) in t.iter_mut().zip(tri) {
            *slot = *map.entry(key(p)).or_insert_with(|| {
                positions.push(p);
                (positions.len() - 1) as u32
            });
        }
        tris.push(t);
    }
    positions.shrink_to_fit();
    TriMesh::new(positions, tris)
}
