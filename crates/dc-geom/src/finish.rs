//! Finishing of a scanned surface in one pass: clean, close the holes,
//! optimise.
//!
//! 1. **Clean**: faces with a repeated vertex, duplicates and zero-area
//!    faces are dropped; so are the faces on edges shared by more than two
//!    faces (non-manifold edges) and the fins (faces with two or three
//!    border edges, left by a frayed scan border). Vertices where separate
//!    fans of faces meet (non-manifold vertices, e.g. two sheets touching at
//!    a corner of the scanned region) are split, one vertex per fan, so
//!    every border is a simple loop. Components smaller than
//!    [`FinishParams::min_island_fraction`] of the largest are removed (by
//!    default all but the main one), and spikes (a vertex far off the
//!    surface of its neighbours) are put back on it.
//! 2. **Close holes** ([`crate::holefill`]): every border loop except the
//!    longest, the open base of the scan, is triangulated, refined to the
//!    surrounding edge length and faired to the curvature of the
//!    surrounding surface.
//! 3. **Optimise**: the surface is first denoised ([`crate::denoise`],
//!    bilateral normal filtering; every vertex moves at most
//!    [`FinishParams::denoise_mm`]) so the ~0.1 mm grain of the scan is not
//!    baked into the result, then decimated ([`crate::qem`]) against the
//!    surface before denoising, within [`FinishParams::max_error_mm`]:
//!    every vertex of that surface stays within that distance of the
//!    output, and every vertex, edge midpoint and face centroid of the
//!    output within it of that surface. No face turns more than
//!    [`FinishParams::max_normal_deg`] from the surface and no edge grows
//!    beyond [`FinishParams::max_edge_mm`], so curved crowns keep enough
//!    faces to look smooth when shaded flat. The border stays in place;
//!    the quadrics keep sharp features.
//!
//! [`FinishReport`] counts what every stage did.

use std::time::Instant;

use dc_mesh::TriMesh;
use glam::Vec3;
use rayon::prelude::*;
use serde::{Deserialize, Serialize};

use crate::denoise::{DenoiseParams, denoise};
use crate::holefill::{Edges, HoleFillParams, fill_holes};
use crate::qem::{DecimateParams, decimate_against};
use crate::{GeomError, Result};

/// Default largest distance the optimisation may move the surface (mm).
pub const DEFAULT_MAX_ERROR_MM: f32 = 0.015;
/// Default largest distance the denoising may move a vertex (mm).
pub const DEFAULT_DENOISE_MM: f32 = 0.01;
/// Default largest angle between an optimised face and the surface (°).
pub const DEFAULT_MAX_NORMAL_DEG: f32 = 8.0;
/// Default longest edge of the optimised mesh (mm).
pub const DEFAULT_MAX_EDGE_MM: f32 = 0.3;
/// A spike: a vertex farther off the centroid of its neighbours, along
/// the normal of its faces, than this many times the size of their ring
/// (a vertex on a smooth surface or a rounded cusp at the scan resolution
/// stays well below 0.5).
const SPIKE_RATIO: f32 = 1.5;
/// Faces of at most this area (mm²) are degenerate.
const ZERO_AREA: f32 = 1e-12;
/// Rounds of fin removal (a frayed border loses a fin per round).
const FIN_ROUNDS: usize = 2;

/// Parameters of [`finish_mesh`].
#[derive(Debug, Clone, PartialEq)]
pub struct FinishParams {
    /// Components with fewer faces than this fraction of the largest are
    /// removed; `1` keeps the main component only.
    pub min_island_fraction: f32,
    /// Close every hole except the open base border.
    pub fill_holes: bool,
    /// Holes with more border edges than this stay open (0: no limit).
    pub max_hole_edges: usize,
    /// Decimate within [`FinishParams::max_error_mm`].
    pub optimise: bool,
    /// Largest distance the optimisation may move the surface (mm).
    pub max_error_mm: f32,
    /// Largest distance the denoising before the optimisation may move a
    /// vertex (mm; 0: no denoising). At most two thirds of
    /// [`FinishParams::max_error_mm`] is used: the result stays within
    /// that error of the surface before denoising.
    pub denoise_mm: f32,
    /// Largest angle between an optimised face and the surface (degrees,
    /// 0: no limit).
    pub max_normal_deg: f32,
    /// Longest edge of the optimised mesh (mm, 0: no limit).
    pub max_edge_mm: f32,
}

impl Default for FinishParams {
    fn default() -> Self {
        Self {
            min_island_fraction: 1.0,
            fill_holes: true,
            max_hole_edges: 0,
            optimise: true,
            max_error_mm: DEFAULT_MAX_ERROR_MM,
            denoise_mm: DEFAULT_DENOISE_MM,
            max_normal_deg: DEFAULT_MAX_NORMAL_DEG,
            max_edge_mm: DEFAULT_MAX_EDGE_MM,
        }
    }
}

/// What [`finish_mesh`] did.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct FinishReport {
    /// Components removed (islands).
    pub islands_removed: usize,
    /// Faces dropped on non-manifold edges plus vertices split at
    /// non-manifold vertices.
    pub nonmanifold_fixed: usize,
    /// Faces dropped as degenerate (repeated vertex, duplicate, zero area).
    pub degenerate_removed: usize,
    /// Fins dropped from the borders.
    pub fins_removed: usize,
    /// Spike vertices put back on the surface.
    pub spikes_fixed: usize,
    /// Holes closed.
    pub holes_filled: usize,
    /// Border loops left open besides the base (holes over the size
    /// limit, or loops that could not be closed).
    pub holes_left: usize,
    /// Border loops of the result (the open base included).
    pub boundary_loops: usize,
    /// Faces of the input mesh.
    pub faces_before: usize,
    /// Faces of the result.
    pub faces_after: usize,
    /// Vertices of the result.
    pub vertices: usize,
    /// Largest distance the optimisation was allowed to move the surface
    /// (mm); 0 without optimisation.
    pub max_error_mm: f32,
    /// Edges of the result shared by more than two faces (0 expected).
    pub non_manifold_edges: usize,
    /// Connected components of the result.
    pub components: usize,
    /// Time of the clean stage (s).
    pub clean_s: f32,
    /// Time of the hole filling (s).
    pub holes_s: f32,
    /// Time of the optimisation, denoising included (s).
    pub optimise_s: f32,
    /// Largest distance the denoising moved a vertex (mm).
    #[serde(default)]
    pub denoise_max_mm: f32,
    /// Time of the denoising (s).
    #[serde(default)]
    pub denoise_s: f32,
}

/// A stage of [`finish_mesh`], for progress reports.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FinishStage {
    /// Cleaning.
    Clean,
    /// Closing the holes.
    Holes,
    /// Optimising.
    Optimise,
}

/// Finishes `mesh` (see the module doc). `stage` is called when a stage
/// starts; `cancel` is polled between the stages.
///
/// # Errors
/// [`GeomError::InvalidInput`] for parameters out of range or a mesh that
/// cleaning leaves empty; [`GeomError::Cancelled`]; kernel errors.
pub fn finish_mesh(
    mesh: &TriMesh,
    params: &FinishParams,
    stage: &dyn Fn(FinishStage),
    cancel: &dyn Fn() -> bool,
) -> Result<(TriMesh, FinishReport)> {
    if !(0.0..=1.0).contains(&params.min_island_fraction) {
        return Err(GeomError::InvalidInput(format!(
            "min_island_fraction {} not in [0, 1]",
            params.min_island_fraction
        )));
    }
    if params.optimise && !(params.max_error_mm > 0.0 && params.max_error_mm <= 1.0) {
        return Err(GeomError::InvalidInput(format!(
            "max_error_mm {} not in (0, 1]",
            params.max_error_mm
        )));
    }
    if !(params.denoise_mm >= 0.0
        && params.denoise_mm.is_finite()
        && (0.0..90.0).contains(&params.max_normal_deg)
        && params.max_edge_mm >= 0.0
        && params.max_edge_mm.is_finite())
    {
        return Err(GeomError::InvalidInput(format!(
            "denoise_mm {} (>= 0), max_normal_deg {} ([0, 90)) or max_edge_mm {} (>= 0) out of range",
            params.denoise_mm, params.max_normal_deg, params.max_edge_mm
        )));
    }
    mesh.check()?;
    let check = || {
        if cancel() {
            Err(GeomError::Cancelled)
        } else {
            Ok(())
        }
    };
    let mut report = FinishReport {
        faces_before: mesh.face_count(),
        ..FinishReport::default()
    };

    stage(FinishStage::Clean);
    check()?;
    let t = Instant::now();
    let mut out = clean(mesh, params.min_island_fraction, &mut report);
    report.clean_s = t.elapsed().as_secs_f32();
    if out.is_empty() {
        return Err(GeomError::InvalidInput(
            "nothing left after cleaning".into(),
        ));
    }

    stage(FinishStage::Holes);
    check()?;
    let t = Instant::now();
    if params.fill_holes {
        let fill = HoleFillParams {
            max_boundary_edges: if params.max_hole_edges == 0 {
                usize::MAX
            } else {
                params.max_hole_edges
            },
            ..HoleFillParams::default()
        };
        let (filled, n) = fill_holes(&out, &fill)?;
        out = filled;
        report.holes_filled = n;
    }
    report.holes_s = t.elapsed().as_secs_f32();

    stage(FinishStage::Optimise);
    check()?;
    let t = Instant::now();
    if params.optimise {
        // the surface before denoising is the reference of the bound
        let reference = out.clone();
        let t = Instant::now();
        let bound = params.denoise_mm.min(params.max_error_mm * 2.0 / 3.0);
        if bound > 0.0 {
            report.denoise_max_mm = denoise(
                &mut out,
                &DenoiseParams {
                    max_displacement: bound,
                    ..DenoiseParams::default()
                },
            )?;
        }
        report.denoise_s = t.elapsed().as_secs_f32();
        check()?;
        let decimation = decimate_against(
            &out,
            &reference,
            &DecimateParams {
                max_valence: 12,
                preserve_labels: false,
                surface_tolerance: true,
                max_normal_angle_deg: f64::from(params.max_normal_deg),
                max_edge: f64::from(params.max_edge_mm),
                ..DecimateParams::to_tolerance(f64::from(params.max_error_mm))
            },
        )?;
        out = decimation.mesh;
        report.max_error_mm = params.max_error_mm;
    }
    report.optimise_s = t.elapsed().as_secs_f32();
    check()?;

    out.update_normals();
    let edges = Edges::new(&out.tris, out.positions.len());
    report.boundary_loops = edges.boundary_loops().len();
    report.holes_left = report.boundary_loops.saturating_sub(1);
    report.non_manifold_edges = edges.non_manifold();
    report.components = out.connected_components().len();
    report.faces_after = out.face_count();
    report.vertices = out.vertex_count();
    Ok((out, report))
}

/// Key of the undirected edge `a b`.
#[inline]
fn edge_key(a: u32, b: u32) -> u64 {
    let (lo, hi) = if a < b { (a, b) } else { (b, a) };
    (u64::from(lo) << 32) | u64::from(hi)
}

/// Every edge of `tris` with the number of faces using it, and per face
/// its edge indices (into the sorted edge list).
fn edge_counts(tris: &[[u32; 3]]) -> (Vec<(u64, u32)>, Vec<[u32; 3]>) {
    let mut half: Vec<(u64, u32)> = tris
        .par_iter()
        .enumerate()
        .flat_map_iter(|(f, t)| {
            (0..3).map(move |k| (edge_key(t[k], t[(k + 1) % 3]), (f * 3 + k) as u32))
        })
        .collect();
    half.par_sort_unstable();
    let mut edges: Vec<(u64, u32)> = Vec::with_capacity(half.len() / 2 + 1);
    let mut of_face = vec![[0u32; 3]; tris.len()];
    for &(key, slot) in &half {
        match edges.last_mut() {
            Some((k, n)) if *k == key => *n += 1,
            _ => edges.push((key, 1)),
        }
        let e = (edges.len() - 1) as u32;
        of_face[slot as usize / 3][slot as usize % 3] = e;
    }
    (edges, of_face)
}

/// Mesh made of the faces of `mesh` flagged in `keep` (attributes kept).
fn retain_faces(mesh: &mut TriMesh, keep: &[bool]) {
    let mut it = keep.iter();
    mesh.tris.retain(|_| *it.next().unwrap_or(&true));
    if let Some(labels) = &mut mesh.face_labels
        && labels.len() == keep.len()
    {
        let mut it = keep.iter();
        labels.retain(|_| *it.next().unwrap_or(&true));
    }
}

/// Drops the faces with a repeated vertex and the repeats of a face (the
/// same three vertices, the first occurrence kept whatever its winding), as
/// [`dc_mesh::cleanup::remove_bad_faces`] but with a parallel sort instead
/// of a hash set; returns how many were dropped.
fn remove_repeated_faces(mesh: &mut TriMesh) -> usize {
    let mut keys: Vec<([u32; 3], u32)> = mesh
        .tris
        .par_iter()
        .enumerate()
        .filter(|(_, t)| t[0] != t[1] && t[1] != t[2] && t[0] != t[2])
        .map(|(f, t)| {
            let mut k = *t;
            k.sort_unstable();
            (k, f as u32)
        })
        .collect();
    keys.par_sort_unstable();
    let mut keep = vec![false; mesh.tris.len()];
    for (i, &(k, f)) in keys.iter().enumerate() {
        if i == 0 || keys[i - 1].0 != k {
            keep[f as usize] = true;
        }
    }
    let removed = keep.iter().filter(|k| !**k).count();
    if removed > 0 {
        retain_faces(mesh, &keep);
    }
    removed
}

/// Drops the vertices no face uses (attributes kept).
fn compact(mesh: &mut TriMesh) {
    let n = mesh.positions.len();
    let mut used = vec![false; n];
    for t in &mesh.tris {
        for &v in t {
            used[v as usize] = true;
        }
    }
    if used.iter().all(|u| *u) {
        return;
    }
    let mut remap = vec![u32::MAX; n];
    let mut next = 0u32;
    for (v, u) in used.iter().enumerate() {
        if *u {
            remap[v] = next;
            next += 1;
        }
    }
    for t in &mut mesh.tris {
        *t = t.map(|v| remap[v as usize]);
    }
    fn pick<T: Copy>(v: &mut Vec<T>, used: &[bool]) {
        let mut it = used.iter();
        v.retain(|_| *it.next().unwrap_or(&false));
    }
    pick(&mut mesh.positions, &used);
    if let Some(a) = &mut mesh.normals {
        pick(a, &used);
    }
    if let Some(a) = &mut mesh.vertex_labels {
        pick(a, &used);
    }
    if let Some(a) = &mut mesh.vertex_colors {
        pick(a, &used);
    }
}

/// Union-find root of `x`.
fn find(parent: &mut [u32], mut x: u32) -> u32 {
    while parent[x as usize] != x {
        let p = parent[parent[x as usize] as usize];
        parent[x as usize] = p;
        x = p;
    }
    x
}

/// Faces of every vertex (CSR: offsets, faces).
pub(crate) fn vertex_faces(mesh: &TriMesh) -> (Vec<u32>, Vec<u32>) {
    let n = mesh.positions.len();
    let mut offsets = vec![0u32; n + 1];
    for t in &mesh.tris {
        for &v in t {
            offsets[v as usize + 1] += 1;
        }
    }
    for i in 0..n {
        offsets[i + 1] += offsets[i];
    }
    let mut fill = offsets.clone();
    let mut faces = vec![0u32; offsets[n] as usize];
    for (f, t) in mesh.tris.iter().enumerate() {
        for &v in t {
            faces[fill[v as usize] as usize] = f as u32;
            fill[v as usize] += 1;
        }
    }
    (offsets, faces)
}

/// Splits every vertex whose faces form several fans (connected through
/// edges at the vertex) into one vertex per fan; returns the vertices
/// added.
fn split_nonmanifold_vertices(mesh: &mut TriMesh) -> usize {
    let (offsets, faces) = vertex_faces(mesh);
    let n = mesh.positions.len();
    let tris = &mesh.tris;
    // per non-manifold vertex: the faces of each fan after the first
    let extra: Vec<(u32, Vec<Vec<u32>>)> = (0..n as u32)
        .into_par_iter()
        .with_min_len(1024)
        .filter_map(|v| {
            let fs = &faces[offsets[v as usize] as usize..offsets[v as usize + 1] as usize];
            if fs.len() < 2 {
                return None;
            }
            let mut parent: Vec<u32> = (0..fs.len() as u32).collect();
            // faces sharing an edge (v, x) belong to the same fan
            let mut seen: Vec<(u32, u32)> = Vec::with_capacity(2 * fs.len());
            for (i, &f) in fs.iter().enumerate() {
                for &x in tris[f as usize].iter().filter(|&&x| x != v) {
                    match seen.iter().find(|(y, _)| *y == x) {
                        Some(&(_, j)) => {
                            let (a, b) = (find(&mut parent, i as u32), find(&mut parent, j));
                            if a != b {
                                parent[a as usize] = b;
                            }
                        }
                        None => seen.push((x, i as u32)),
                    }
                }
            }
            let mut fans: Vec<(u32, Vec<u32>)> = Vec::new();
            for (i, &f) in fs.iter().enumerate() {
                let r = find(&mut parent, i as u32);
                match fans.iter_mut().find(|(root, _)| *root == r) {
                    Some((_, list)) => list.push(f),
                    None => fans.push((r, vec![f])),
                }
            }
            (fans.len() > 1).then(|| (v, fans.into_iter().skip(1).map(|(_, l)| l).collect()))
        })
        .collect();
    let mut added = 0;
    for (v, fans) in extra {
        for fan in fans {
            let nv = mesh.positions.len() as u32;
            mesh.positions.push(mesh.positions[v as usize]);
            if let Some(a) = &mut mesh.normals {
                a.push(a[v as usize]);
            }
            if let Some(a) = &mut mesh.vertex_labels {
                a.push(a[v as usize]);
            }
            if let Some(a) = &mut mesh.vertex_colors {
                a.push(a[v as usize]);
            }
            for f in fan {
                for x in &mut mesh.tris[f as usize] {
                    if *x == v {
                        *x = nv;
                    }
                }
            }
            added += 1;
        }
    }
    added
}

/// Removes the components with fewer faces than `fraction` times the
/// largest; returns how many were removed.
fn remove_islands(mesh: &mut TriMesh, fraction: f32) -> usize {
    let comps = mesh.connected_components();
    let Some(largest) = comps.first().map(Vec::len) else {
        return 0;
    };
    let limit = fraction * largest as f32;
    let mut keep = vec![false; mesh.tris.len()];
    let mut removed = 0;
    for (i, c) in comps.iter().enumerate() {
        if i == 0 || c.len() as f32 >= limit {
            for &f in c {
                keep[f as usize] = true;
            }
        } else {
            removed += 1;
        }
    }
    if removed > 0 {
        retain_faces(mesh, &keep);
    }
    removed
}

/// Vertices on a border edge (an edge of one face).
pub(crate) fn border_vertices(mesh: &TriMesh) -> Vec<bool> {
    let (edges, of_face) = edge_counts(&mesh.tris);
    let mut border = vec![false; mesh.positions.len()];
    for (f, t) in mesh.tris.iter().enumerate() {
        for k in 0..3 {
            if edges[of_face[f][k] as usize].1 == 1 {
                border[t[k] as usize] = true;
                border[t[(k + 1) % 3] as usize] = true;
            }
        }
    }
    border
}

/// Puts the spike vertices (see [`SPIKE_RATIO`]) on
/// the centroid of their neighbours; returns how many moved. Border
/// vertices stay.
fn fix_spikes(mesh: &mut TriMesh) -> usize {
    let (offsets, faces) = vertex_faces(mesh);
    let border = border_vertices(mesh);
    let pos = &mesh.positions;
    let tris = &mesh.tris;
    let moves: Vec<(usize, Vec3)> = (0..pos.len())
        .into_par_iter()
        .with_min_len(1024)
        .filter_map(|v| {
            let fs = &faces[offsets[v] as usize..offsets[v + 1] as usize];
            if border[v] || fs.len() < 3 {
                return None;
            }
            let mut normal = Vec3::ZERO;
            let mut ring = Vec3::ZERO;
            let mut count = 0.0f32;
            for &f in fs {
                let [a, b, c] = tris[f as usize].map(|i| pos[i as usize]);
                normal += (b - a).cross(c - a);
                for &x in tris[f as usize].iter().filter(|&&x| x as usize != v) {
                    ring += pos[x as usize];
                    count += 1.0;
                }
            }
            let normal = normal.try_normalize()?;
            // every ring vertex is counted twice (two faces each)
            let centroid = ring / count;
            // size of the ring: mean distance of its vertices from their
            // centroid (independent of where the vertex itself went)
            let spread = fs
                .iter()
                .flat_map(|&f| tris[f as usize])
                .filter(|&x| x as usize != v)
                .map(|x| pos[x as usize].distance(centroid))
                .sum::<f32>()
                / count;
            let off = (pos[v] - centroid).dot(normal).abs();
            (off > SPIKE_RATIO * spread).then_some((v, centroid))
        })
        .collect();
    for &(v, c) in &moves {
        mesh.positions[v] = c;
    }
    moves.len()
}

/// The clean stage (see the module doc).
fn clean(mesh: &TriMesh, island_fraction: f32, report: &mut FinishReport) -> TriMesh {
    let mut out = mesh.clone();
    report.degenerate_removed += remove_repeated_faces(&mut out);
    let keep: Vec<bool> = (0..out.tris.len())
        .into_par_iter()
        .with_min_len(4096)
        .map(|f| out.face_area(f) > ZERO_AREA)
        .collect();
    let zero = keep.iter().filter(|k| !**k).count();
    if zero > 0 {
        retain_faces(&mut out, &keep);
        report.degenerate_removed += zero;
    }
    // faces on non-manifold edges, then the fins (faces with two or three
    // border edges), round by round; the edge counts follow the removals
    let (mut edges, of_face) = edge_counts(&out.tris);
    let mut keep = vec![true; out.tris.len()];
    let remove = |keep: &mut [bool], edges: &mut [(u64, u32)], gone: &[usize]| {
        for &f in gone {
            keep[f] = false;
            for &e in &of_face[f] {
                edges[e as usize].1 -= 1;
            }
        }
    };
    let gone: Vec<usize> = (0..of_face.len())
        .filter(|&f| of_face[f].iter().any(|&e| edges[e as usize].1 > 2))
        .collect();
    report.nonmanifold_fixed += gone.len();
    remove(&mut keep, &mut edges, &gone);
    for _ in 0..FIN_ROUNDS {
        let gone: Vec<usize> = (0..of_face.len())
            .filter(|&f| {
                keep[f]
                    && of_face[f]
                        .iter()
                        .filter(|&&e| edges[e as usize].1 == 1)
                        .count()
                        >= 2
            })
            .collect();
        if gone.is_empty() {
            break;
        }
        report.fins_removed += gone.len();
        remove(&mut keep, &mut edges, &gone);
    }
    if keep.iter().any(|k| !k) {
        retain_faces(&mut out, &keep);
    }
    report.nonmanifold_fixed += split_nonmanifold_vertices(&mut out);
    report.islands_removed = remove_islands(&mut out, island_fraction);
    report.spikes_fixed = fix_spikes(&mut out);
    compact(&mut out);
    out
}

#[cfg(test)]
mod tests;
