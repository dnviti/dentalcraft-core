//! Edge adjacency, boundary loops and connected components.

use std::collections::{BTreeMap, HashMap};

use crate::TriMesh;

/// Undirected edge, stored with the smaller vertex index first.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct EdgeKey(pub u32, pub u32);

impl EdgeKey {
    /// Edge between `a` and `b` in canonical (sorted) order.
    pub fn new(a: u32, b: u32) -> Self {
        if a <= b { Self(a, b) } else { Self(b, a) }
    }
}

/// Map from every undirected edge to the faces using it.
#[derive(Debug, Clone, Default)]
pub struct EdgeAdjacency {
    faces: HashMap<EdgeKey, Vec<u32>>,
    /// Directed half-edges `(from, to)` of boundary edges, following the face winding.
    boundary: Vec<(u32, u32)>,
}

impl EdgeAdjacency {
    /// Builds the adjacency of `mesh`. Degenerate edges (`a == b`) are ignored.
    pub fn build(mesh: &TriMesh) -> Self {
        let mut faces: HashMap<EdgeKey, Vec<u32>> = HashMap::with_capacity(mesh.tris.len() * 3 / 2);
        for (f, t) in mesh.tris.iter().enumerate() {
            for (a, b) in half_edges(t) {
                if a != b {
                    faces.entry(EdgeKey::new(a, b)).or_default().push(f as u32);
                }
            }
        }
        let mut boundary: Vec<(u32, u32)> = mesh
            .tris
            .iter()
            .flat_map(half_edges)
            .filter(|&(a, b)| {
                a != b && faces.get(&EdgeKey::new(a, b)).is_some_and(|v| v.len() == 1)
            })
            .collect();
        boundary.sort_unstable();
        Self { faces, boundary }
    }

    /// Number of distinct edges.
    pub fn edge_count(&self) -> usize {
        self.faces.len()
    }

    /// Faces using the edge `(a, b)` (empty when it does not exist).
    pub fn faces_of(&self, a: u32, b: u32) -> &[u32] {
        self.faces
            .get(&EdgeKey::new(a, b))
            .map_or(&[], Vec::as_slice)
    }

    /// Iterator over `(edge, faces)` pairs, in no particular order.
    pub fn iter(&self) -> impl Iterator<Item = (&EdgeKey, &[u32])> {
        self.faces.iter().map(|(k, v)| (k, v.as_slice()))
    }

    /// Number of edges used by exactly one face.
    pub fn boundary_edge_count(&self) -> usize {
        self.boundary.len()
    }

    /// Number of edges used by more than two faces.
    pub fn non_manifold_edge_count(&self) -> usize {
        self.faces.values().filter(|v| v.len() > 2).count()
    }

    /// Boundary loops as vertex lists following the face winding. Open
    /// chains (possible around non-manifold vertices) are returned as well.
    pub fn boundary_loops(&self) -> Vec<Vec<u32>> {
        let mut next: BTreeMap<u32, Vec<u32>> = BTreeMap::new();
        for &(a, b) in &self.boundary {
            next.entry(a).or_default().push(b);
        }
        let mut loops = Vec::new();
        for &(start, _) in &self.boundary {
            if next.get(&start).is_none_or(Vec::is_empty) {
                continue;
            }
            let mut lp = vec![start];
            let mut v = start;
            while let Some(n) = next.get_mut(&v).and_then(Vec::pop) {
                if n == start {
                    break;
                }
                lp.push(n);
                v = n;
            }
            loops.push(lp);
        }
        loops
    }
}

fn half_edges(t: &[u32; 3]) -> [(u32, u32); 3] {
    [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])]
}

fn find(parent: &mut [u32], mut x: u32) -> u32 {
    while parent[x as usize] != x {
        let p = parent[parent[x as usize] as usize];
        parent[x as usize] = p;
        x = p;
    }
    x
}

pub(crate) fn connected_components(mesh: &TriMesh) -> Vec<Vec<u32>> {
    let mut parent: Vec<u32> = (0..mesh.positions.len() as u32).collect();
    for t in &mesh.tris {
        let r0 = find(&mut parent, t[0]);
        for &v in &t[1..] {
            let r = find(&mut parent, v);
            if r != r0 {
                parent[r as usize] = r0;
            }
        }
    }
    let mut groups: BTreeMap<u32, Vec<u32>> = BTreeMap::new();
    for (f, t) in mesh.tris.iter().enumerate() {
        groups
            .entry(find(&mut parent, t[0]))
            .or_default()
            .push(f as u32);
    }
    let mut out: Vec<Vec<u32>> = groups.into_values().collect();
    out.sort_by(|a, b| b.len().cmp(&a.len()).then(a[0].cmp(&b[0])));
    out
}

#[cfg(test)]
mod tests {
    use crate::test_meshes::*;

    #[test]
    fn closed_mesh_has_no_boundary() {
        let adj = tetra().edge_adjacency();
        assert_eq!(adj.edge_count(), 6);
        assert_eq!(adj.boundary_edge_count(), 0);
        assert!(adj.boundary_loops().is_empty());
        assert_eq!(adj.faces_of(0, 1).len(), 2);
    }

    #[test]
    fn quad_has_one_boundary_loop_in_winding_order() {
        let loops = quad().boundary_loops();
        assert_eq!(loops, vec![vec![0, 1, 2, 3]]);
    }

    #[test]
    fn components_split_disjoint_parts() {
        let mut m = tetra();
        let q = quad();
        let off = m.positions.len() as u32;
        m.positions
            .extend(q.positions.iter().map(|p| *p + glam::Vec3::X * 5.0));
        m.tris.extend(q.tris.iter().map(|t| t.map(|i| i + off)));
        let comps = m.connected_components();
        assert_eq!(comps.len(), 2);
        assert_eq!(comps[0], vec![0, 1, 2, 3]);
        assert_eq!(comps[1], vec![4, 5]);
    }
}
