//! Mesh repair helpers for scanned surfaces: small component removal and
//! Taubin (λ|μ) smoothing. Decimation is [`crate::qem::decimate`].

use dc_mesh::TriMesh;
use glam::Vec3;

/// Mesh made of the faces flagged in `keep`, with unused vertices dropped
/// and every attribute carried over.
fn keep_faces(mesh: &TriMesh, keep: &[bool]) -> TriMesh {
    let mut remap = vec![u32::MAX; mesh.positions.len()];
    let mut order = Vec::new();
    let mut tris = Vec::new();
    let mut face_labels = mesh.face_labels.as_ref().map(|_| Vec::new());
    for (f, t) in mesh.tris.iter().enumerate() {
        if !keep[f] {
            continue;
        }
        tris.push(t.map(|v| {
            let r = &mut remap[v as usize];
            if *r == u32::MAX {
                *r = order.len() as u32;
                order.push(v as usize);
            }
            *r
        }));
        if let (Some(out), Some(src)) = (&mut face_labels, &mesh.face_labels) {
            out.push(src[f]);
        }
    }
    // keep the original vertex order
    let mut sorted = order.clone();
    sorted.sort_unstable();
    let mut new_index = vec![u32::MAX; mesh.positions.len()];
    for (i, &v) in sorted.iter().enumerate() {
        new_index[v] = i as u32;
    }
    let tris = tris
        .into_iter()
        .map(|t| t.map(|r| new_index[order[r as usize]]))
        .collect();
    let pick = |v: &Vec<_>| sorted.iter().map(|&i| v[i]).collect();
    TriMesh {
        positions: pick(&mesh.positions),
        tris,
        normals: mesh.normals.as_ref().map(pick),
        vertex_labels: mesh
            .vertex_labels
            .as_ref()
            .map(|l| sorted.iter().map(|&i| l[i]).collect()),
        face_labels,
        vertex_colors: mesh
            .vertex_colors
            .as_ref()
            .map(|c| sorted.iter().map(|&i| c[i]).collect()),
    }
}

/// Removes the connected components (faces sharing vertices) with fewer
/// faces than `min_fraction` times the largest component. Returns the mesh
/// and the number of components removed. Unreferenced vertices are dropped.
pub fn remove_small_components(mesh: &TriMesh, min_fraction: f32) -> (TriMesh, usize) {
    let comps = mesh.connected_components();
    let Some(largest) = comps.first().map(Vec::len) else {
        return (keep_faces(mesh, &[]), 0);
    };
    let limit = min_fraction.max(0.0) * largest as f32;
    let mut keep = vec![false; mesh.tris.len()];
    let mut removed = 0;
    for c in &comps {
        if (c.len() as f32) < limit {
            removed += 1;
        } else {
            for &f in c {
                keep[f as usize] = true;
            }
        }
    }
    (keep_faces(mesh, &keep), removed)
}

/// Taubin λ|μ smoothing (uniform Laplacian): every iteration moves the
/// vertices by `lambda` times their Laplacian, then by `mu` (negative,
/// `|mu| > lambda`, e.g. `0.5` / `-0.53`) to undo the shrinkage. Boundary
/// vertices move along the boundary only (their Laplacian uses their
/// boundary neighbours), so open scan borders are smoothed without
/// retracting. Normals, when present, are recomputed.
pub fn taubin_smooth(mesh: &TriMesh, iterations: u32, lambda: f32, mu: f32) -> TriMesh {
    let n = mesh.positions.len();
    let mut out = mesh.clone();
    if iterations == 0 || n == 0 {
        return out;
    }
    let adj = mesh.edge_adjacency();
    let mut nbrs: Vec<Vec<u32>> = vec![Vec::new(); n];
    let mut boundary_nbrs: Vec<Vec<u32>> = vec![Vec::new(); n];
    for (e, faces) in adj.iter() {
        let (a, b) = (e.0 as usize, e.1 as usize);
        nbrs[a].push(e.1);
        nbrs[b].push(e.0);
        if faces.len() == 1 {
            boundary_nbrs[a].push(e.1);
            boundary_nbrs[b].push(e.0);
        }
    }
    for (all, border) in nbrs.iter_mut().zip(&boundary_nbrs) {
        if !border.is_empty() {
            all.clone_from(border);
        }
        all.sort_unstable();
    }
    let mut next = vec![Vec3::ZERO; n];
    let pass = |pos: &mut Vec<Vec3>, next: &mut Vec<Vec3>, f: f32| {
        for (i, list) in nbrs.iter().enumerate() {
            next[i] = if list.is_empty() {
                pos[i]
            } else {
                let avg = list.iter().map(|&j| pos[j as usize]).sum::<Vec3>() / list.len() as f32;
                pos[i] + (avg - pos[i]) * f
            };
        }
        std::mem::swap(pos, next);
    };
    for _ in 0..iterations {
        pass(&mut out.positions, &mut next, lambda);
        pass(&mut out.positions, &mut next, mu);
    }
    if out.normals.is_some() {
        out.update_normals();
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn grid(n: usize, offset: Vec3) -> TriMesh {
        let mut positions = Vec::new();
        for y in 0..n {
            for x in 0..n {
                positions.push(Vec3::new(x as f32, y as f32, 0.0) + offset);
            }
        }
        let id = |x: usize, y: usize| (y * n + x) as u32;
        let mut tris = Vec::new();
        for y in 0..n - 1 {
            for x in 0..n - 1 {
                tris.push([id(x, y), id(x + 1, y), id(x + 1, y + 1)]);
                tris.push([id(x, y), id(x + 1, y + 1), id(x, y + 1)]);
            }
        }
        TriMesh::new(positions, tris)
    }

    fn merge(a: &TriMesh, b: &TriMesh) -> TriMesh {
        let mut m = a.clone();
        let off = m.positions.len() as u32;
        m.positions.extend_from_slice(&b.positions);
        m.tris.extend(b.tris.iter().map(|t| t.map(|v| v + off)));
        m
    }

    #[test]
    fn small_islands_are_removed_with_their_vertices() {
        let big = grid(10, Vec3::ZERO);
        let small = grid(2, Vec3::new(50.0, 0.0, 0.0));
        let mut m = merge(&merge(&small, &big), &grid(3, Vec3::new(0.0, 50.0, 0.0)));
        m.vertex_labels = Some((0..m.positions.len() as u16).collect());
        m.face_labels = Some(vec![7; m.tris.len()]);
        let (out, removed) = remove_small_components(&m, 0.1);
        assert_eq!(removed, 2);
        assert_eq!(out.face_count(), big.face_count());
        assert_eq!(out.vertex_count(), big.vertex_count());
        out.check().unwrap();
        assert!((out.area() - big.area()).abs() < 1e-4);
        // vertex order and attributes survive
        assert_eq!(out.positions, big.positions);
        assert_eq!(out.vertex_labels.unwrap()[0], 4);
        let (all, none) = remove_small_components(&m, 0.0);
        assert_eq!((all.face_count(), none), (m.face_count(), 0));
    }

    #[test]
    fn taubin_reduces_noise_without_shrinking() {
        let mut m = grid(15, Vec3::ZERO);
        let mut s = 1u32;
        for p in &mut m.positions {
            s = s.wrapping_mul(1_103_515_245).wrapping_add(12_345);
            p.z = ((s >> 16) % 1000) as f32 / 1000.0 * 0.2 - 0.1;
        }
        m.update_normals();
        let rms = |m: &TriMesh| {
            (m.positions.iter().map(|p| p.z * p.z).sum::<f32>() / m.vertex_count() as f32).sqrt()
        };
        let out = taubin_smooth(&m, 10, 0.5, -0.53);
        assert!(rms(&out) < 0.5 * rms(&m), "{} {}", rms(&out), rms(&m));
        let (b0, b1) = (m.aabb().unwrap(), out.aabb().unwrap());
        assert!((b0.size().x - b1.size().x).abs() < 0.3);
        assert!(out.normals.is_some());
        assert_eq!(taubin_smooth(&m, 0, 0.5, -0.53), m);
    }
}
