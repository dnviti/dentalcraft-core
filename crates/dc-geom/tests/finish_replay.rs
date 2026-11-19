//! Timing and accuracy of the mesh finish on a recorded raw scan mesh.
//!
//! Ignored by default: it needs `DC_FINISH_IN`, a raw fine reconstruction
//! (e.g. the `raw.ply` written by `dc-scanner`'s `recorded_stops_reconstruct`
//! with `DC_SCAN_OUT`). Run with
//!
//! `DC_FINISH_IN=<raw.ply> cargo test --release -p dc-geom --test finish_replay -- --ignored --nocapture`
//!
//! It prints the time of every stage, the faces in and out, the topology of
//! the result and the deviation of the optimisation: the largest distance
//! from every vertex of the optimisation input (the raw mesh cleaned and with
//! its holes closed) to the result, and from every vertex, edge midpoint and
//! face centroid of the result to that input. `DC_FINISH_OUT` writes the
//! result.

use std::time::Instant;

use dc_geom::bvh::Bvh;
use dc_geom::finish::{FinishParams, finish_mesh};
use dc_mesh::TriMesh;
use glam::Vec3;

/// Distance from every point of `points` to the surface of `mesh`.
fn distances(mesh: &TriMesh, points: &[Vec3]) -> Vec<f32> {
    let bvh = Bvh::build(mesh).unwrap();
    bvh.closest_points(points, f32::INFINITY)
        .into_iter()
        .map(|h| h.map_or(f32::INFINITY, |h| h.distance))
        .collect()
}

fn max(d: &[f32]) -> f32 {
    d.iter().copied().fold(0.0, f32::max)
}

/// Edge midpoints and face centroids of `mesh`, with their face.
fn samples(mesh: &TriMesh) -> (Vec<Vec3>, Vec<usize>) {
    let p = &mesh.positions;
    let mut out = Vec::with_capacity(4 * mesh.tris.len());
    let mut face = Vec::with_capacity(4 * mesh.tris.len());
    for (f, t) in mesh.tris.iter().enumerate() {
        let [a, b, c] = t.map(|v| p[v as usize]);
        out.extend([
            (a + b) * 0.5,
            (b + c) * 0.5,
            (c + a) * 0.5,
            (a + b + c) / 3.0,
        ]);
        face.extend([f; 4]);
    }
    (out, face)
}

#[test]
#[ignore = "needs DC_FINISH_IN with a raw scan mesh"]
fn finish_replay() {
    let path = std::env::var_os("DC_FINISH_IN").expect("set DC_FINISH_IN to a raw mesh");
    let raw = dc_mesh::io::read(&path).unwrap();
    // DC_FINISH_ERR, _DENOISE, _NORMAL, _EDGE override the defaults
    let env = |key: &str, default: f32| {
        std::env::var(key)
            .ok()
            .and_then(|v| v.parse().ok())
            .unwrap_or(default)
    };
    let d = FinishParams::default();
    let params = FinishParams {
        max_error_mm: env("DC_FINISH_ERR", d.max_error_mm),
        denoise_mm: env("DC_FINISH_DENOISE", d.denoise_mm),
        max_normal_deg: env("DC_FINISH_NORMAL", d.max_normal_deg),
        max_edge_mm: env("DC_FINISH_EDGE", d.max_edge_mm),
        ..d
    };
    let t = Instant::now();
    let (finished, report) = finish_mesh(&raw, &params, &|_| {}, &|| false).unwrap();
    let total = t.elapsed().as_secs_f32();
    println!("FINISH {total:.2} s {params:?} {report:?}");
    let adj = finished.edge_adjacency();
    let mut lengths: Vec<f32> = adj
        .iter()
        .map(|(e, _)| finished.positions[e.0 as usize].distance(finished.positions[e.1 as usize]))
        .collect();
    lengths.sort_unstable_by(f32::total_cmp);
    println!(
        "result: {} faces, {} border loops, {} non-manifold edges, {} components; edge median {:.3} mm, 95 % {:.3} mm, max {:.3} mm",
        finished.face_count(),
        adj.boundary_loops().len(),
        adj.non_manifold_edge_count(),
        finished.connected_components().len(),
        lengths[lengths.len() / 2],
        lengths[lengths.len() * 95 / 100],
        lengths[lengths.len() - 1]
    );
    // the optimisation input: the same finish without the optimisation
    let (input, _) = finish_mesh(
        &raw,
        &FinishParams {
            optimise: false,
            ..FinishParams::default()
        },
        &|_| {},
        &|| false,
    )
    .unwrap();
    let to_result = max(&distances(&finished, &input.positions));
    let vertices_to_input = max(&distances(&input, &finished.positions));
    let (points, face) = samples(&finished);
    let d = distances(&input, &points);
    // samples on faces touching the open border apart: a face along the
    // border may cut across a notch of the scan border
    let border: std::collections::HashSet<u32> =
        adj.boundary_loops().into_iter().flatten().collect();
    let on_border = |f: usize| finished.tris[f].iter().any(|v| border.contains(v));
    let inner = d
        .iter()
        .zip(&face)
        .filter(|(_, f)| !on_border(**f))
        .map(|(d, _)| *d)
        .fold(0.0, f32::max);
    let mut sorted = d.clone();
    sorted.sort_unstable_by(f32::total_cmp);
    let p999 = sorted[(sorted.len() - 1) * 999 / 1000];
    println!(
        "deviation (mm, bound {}): input vertices to result {to_result:.5}, result vertices to input {vertices_to_input:.5}; \
         result edge midpoints and centroids to input: max {:.5}, off the border {inner:.5}, 99.9 % {p999:.5}",
        report.max_error_mm,
        max(&d)
    );
    if let Some(out) = std::env::var_os("DC_FINISH_OUT") {
        dc_mesh::io::write(out, &finished, dc_mesh::io::MeshFormat::Ply).unwrap();
    }
    assert_eq!(report.non_manifold_edges, 0);
    assert_eq!(report.components, 1);
    assert!(to_result <= report.max_error_mm * 1.001);
    assert!(vertices_to_input <= report.max_error_mm * 1.001);
}
