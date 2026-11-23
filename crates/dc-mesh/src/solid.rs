//! Topology of a mesh as a solid: what a slicer or a print check sees.
//!
//! An STL file has no connectivity: two corners are the same vertex when
//! their positions are bit-identical, and nothing else. [`solid_topology`]
//! identifies vertices exactly that way (`-0.0` = `0.0`), whatever the
//! indices of the mesh say: vertices duplicated at one position are one
//! vertex, two distinct positions are never merged however close they are.
//! A tolerance weld would collapse the short edges of a fine mesh and
//! report the folded triangles as non-manifold edges that the file does not
//! have.
//!
//! A triangle with two corners at one position has no area and encloses
//! nothing: it is left out ([`SolidTopology::left_out`]). The rest is
//! counted the way the aligners' export print check counts it:
//! - an edge used by one triangle is a boundary (open) edge, by more than
//!   two a non-manifold edge;
//! - a directed edge run by two triangles in the same direction is a
//!   flipped edge (two neighbours oriented in opposite ways);
//! - pieces are joined by shared vertices;
//! - the genus of a closed surface is `pieces − χ/2`, with
//!   `χ = V − E + F`: the holes through the solid (a tunnel through a wall);
//! - the signed volume, positive for outward triangles.
//!
//! Sort based, no hashing: about a second on a 1M-triangle shell.

use glam::DVec3;

use crate::TriMesh;

/// The topology of a mesh as a solid, see the module documentation.
#[derive(Debug, Clone, PartialEq)]
pub struct SolidTopology {
    /// Distinct positions used by the triangles counted.
    pub vertices: usize,
    /// Distinct undirected edges.
    pub edges: usize,
    /// Triangles counted.
    pub triangles: usize,
    /// Triangles left out: two corners at one position, or an index out of
    /// range.
    pub left_out: usize,
    /// Edges used by one triangle only.
    pub boundary_edges: usize,
    /// Edges used by more than two triangles.
    pub nonmanifold_edges: usize,
    /// Directed edges run by more than one triangle (neighbours with
    /// opposite orientations).
    pub flipped_edges: usize,
    /// Connected pieces (triangles sharing a vertex are one piece).
    pub pieces: usize,
    /// Holes through the solid, when it is closed (no boundary and no
    /// non-manifold edge); `None` otherwise.
    pub genus: Option<i64>,
    /// Signed volume, mm³: positive when the triangles face outward.
    pub volume_mm3: f64,
}

impl SolidTopology {
    /// Every edge shared by exactly two triangles.
    pub fn is_closed(&self) -> bool {
        self.boundary_edges == 0 && self.nonmanifold_edges == 0
    }
}

/// Packs a directed edge.
fn key(a: u32, b: u32) -> u64 {
    (u64::from(a) << 32) | u64::from(b)
}

fn find(parent: &mut [u32], mut x: u32) -> u32 {
    while parent[x as usize] != x {
        parent[x as usize] = parent[parent[x as usize] as usize];
        x = parent[x as usize];
    }
    x
}

/// The topology of `mesh` as a solid, vertices identified by their exact
/// position (see the module documentation).
pub fn solid_topology(mesh: &TriMesh) -> SolidTopology {
    // exact weld: sort the positions by their bits, equal runs are one vertex
    let bits = |i: usize| {
        let p = mesh.positions[i];
        [
            (p.x + 0.0).to_bits(),
            (p.y + 0.0).to_bits(),
            (p.z + 0.0).to_bits(),
        ]
    };
    let mut order: Vec<u32> = (0..mesh.positions.len() as u32).collect();
    order.sort_unstable_by_key(|&i| bits(i as usize));
    let mut welded = vec![0u32; mesh.positions.len()];
    let mut distinct = 0u32;
    for (k, &i) in order.iter().enumerate() {
        if k > 0 && bits(i as usize) != bits(order[k - 1] as usize) {
            distinct += 1;
        }
        welded[i as usize] = distinct;
    }
    drop(order);

    let mut tris: Vec<[u32; 3]> = Vec::with_capacity(mesh.tris.len());
    let mut left_out = 0;
    for t in &mesh.tris {
        let w = t.map(|i| welded.get(i as usize).copied().unwrap_or(u32::MAX));
        if w.contains(&u32::MAX) || w[0] == w[1] || w[1] == w[2] || w[0] == w[2] {
            left_out += 1;
            continue;
        }
        tris.push(w);
    }

    // edges: the directed half-edges sorted, each undirected edge counted
    // once, from its smaller-first direction or from the only one present
    let mut half: Vec<u64> = Vec::with_capacity(tris.len() * 3);
    for t in &tris {
        half.extend([key(t[0], t[1]), key(t[1], t[2]), key(t[2], t[0])]);
    }
    half.sort_unstable();
    let runs = |k: u64| half.partition_point(|&h| h <= k) - half.partition_point(|&h| h < k);
    let (mut edges, mut boundary, mut nonmanifold, mut flipped) = (0, 0, 0, 0);
    let mut start = 0;
    while start < half.len() {
        let k = half[start];
        let mut end = start + 1;
        while end < half.len() && half[end] == k {
            end += 1;
        }
        let count = end - start;
        let (a, b) = ((k >> 32) as u32, k as u32);
        let reverse = runs(key(b, a));
        if a < b || reverse == 0 {
            edges += 1;
            match count + reverse {
                1 => boundary += 1,
                2 => {}
                _ => nonmanifold += 1,
            }
        }
        if count > 1 {
            flipped += 1;
        }
        start = end;
    }
    drop(half);

    // pieces, through shared vertices
    let mut parent: Vec<u32> = (0..=distinct).collect();
    let mut used = vec![false; parent.len()];
    for t in &tris {
        for &i in t {
            used[i as usize] = true;
        }
        let a = find(&mut parent, t[0]);
        for &o in &t[1..] {
            let r = find(&mut parent, o);
            if r != a {
                parent[r as usize] = a;
            }
        }
    }
    let vertices = used.iter().filter(|u| **u).count();
    let pieces = (0..parent.len() as u32)
        .filter(|&i| used[i as usize] && find(&mut parent, i) == i)
        .count();

    let genus = (boundary == 0 && nonmanifold == 0 && !tris.is_empty()).then(|| {
        let chi = vertices as i64 - edges as i64 + tris.len() as i64;
        // chi is even on a closed orientable surface; a pinch (pieces
        // touching at a vertex) makes it larger, never a handle
        pieces as i64 - chi.div_euclid(2)
    });

    // volume, around the first vertex to keep the sums small
    let origin = mesh
        .tris
        .first()
        .and_then(|t| mesh.positions.get(t[0] as usize))
        .map_or(DVec3::ZERO, |p| p.as_dvec3());
    let six_volume: f64 = mesh
        .tris
        .iter()
        .filter(|t| t.iter().all(|&i| (i as usize) < mesh.positions.len()))
        .map(|t| {
            let [a, b, c] = t.map(|i| mesh.positions[i as usize].as_dvec3() - origin);
            a.dot(b.cross(c))
        })
        .sum();
    let volume_mm3 = six_volume / 6.0;

    SolidTopology {
        vertices,
        edges,
        triangles: tris.len(),
        left_out,
        boundary_edges: boundary,
        nonmanifold_edges: nonmanifold,
        flipped_edges: flipped,
        pieces,
        genus,
        volume_mm3,
    }
}

#[cfg(test)]
mod tests;
