//! Post-acquisition editing of a scan: point-cloud cleanup, meshing and
//! mesh repair, with undo / redo.
//!
//! [`ScanEditor`] holds the cloud and, once built, the mesh. Operations are
//! plain data ([`CloudOp`], [`MeshOp`]; serde tag `op`, kebab-case) so that
//! the CLI / MCP commands and the UI share them. Every change pushes a
//! snapshot of both (shared buffers, cheap) on the undo stack, capped at
//! [`UNDO_LIMIT`]. A cloud edit drops the mesh, which no longer matches it;
//! undo brings it back.

use std::sync::Arc;

use dc_geom::cloud;
use dc_geom::holefill::{HoleFillParams, fill_holes};
use dc_geom::meshing::{CloudMeshParams, mesh_from_cloud};
use dc_geom::qem::{DecimateParams, decimate};
use dc_geom::repair::{remove_small_components, taubin_smooth};
use dc_mesh::{PointCloud, TriMesh};
use glam::Vec3;
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use serde_json::{Value, json};

use crate::{Result, ScanError};

/// Snapshots kept for undo.
pub const UNDO_LIMIT: usize = 20;
/// Neighbours used by the cloud-to-mesh implicit surface.
const MESH_K: usize = 10;

/// A point-cloud edit.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema, PartialEq)]
#[serde(tag = "op", rename_all = "kebab-case")]
pub enum CloudOp {
    /// Keeps one point (the centroid) per voxel.
    Downsample {
        /// Voxel edge (mm).
        voxel_mm: f32,
    },
    /// Removes points whose mean distance to their `k` neighbours exceeds
    /// the global mean by `std_ratio` standard deviations.
    RemoveStatistical {
        /// Neighbours per point.
        k: usize,
        /// Threshold in standard deviations.
        std_ratio: f32,
    },
    /// Removes points with fewer than `min_neighbors` within `radius_mm`.
    RemoveRadius {
        /// Search radius (mm).
        radius_mm: f32,
        /// Fewest neighbours to keep a point.
        min_neighbors: usize,
    },
    /// Keeps only the largest cluster of points connected within `radius_mm`.
    Isolate {
        /// Connection radius (mm).
        radius_mm: f32,
    },
    /// Moves points towards their local plane.
    Smooth {
        /// Neighbours per point.
        k: usize,
        /// Passes.
        iterations: u32,
        /// Blend towards the plane, `0..=1`.
        strength: f32,
    },
    /// Deletes the points inside a box.
    DeleteAabb {
        /// Minimum corner (mm).
        min: [f32; 3],
        /// Maximum corner (mm).
        max: [f32; 3],
    },
    /// Deletes the points inside a sphere.
    DeleteSphere {
        /// Centre (mm).
        center: [f32; 3],
        /// Radius (mm).
        radius_mm: f32,
    },
    /// Deletes the points on the side of the plane the normal points to.
    DeletePlane {
        /// A point of the plane (mm).
        point: [f32; 3],
        /// Plane normal.
        normal: [f32; 3],
    },
    /// Deletes the listed points (e.g. a lasso selection).
    DeleteIndices {
        /// Point indices.
        indices: Vec<u32>,
    },
    /// Recomputes the normals from `k` neighbours.
    EstimateNormals {
        /// Neighbours per point.
        k: usize,
    },
}

/// A mesh edit.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema, PartialEq)]
#[serde(tag = "op", rename_all = "kebab-case")]
pub enum MeshOp {
    /// Closes the holes bounded by at most `max_edges` edges (never the
    /// open border of the scan, the largest boundary).
    FillHoles {
        /// Largest hole closed, in boundary edges.
        max_edges: usize,
    },
    /// Taubin smoothing (no shrinking).
    Smooth {
        /// Passes.
        iterations: u32,
        /// Positive (smoothing) factor.
        lambda: f32,
        /// Negative (inflating) factor.
        mu: f32,
    },
    /// Removes the components smaller than `min_fraction` of the largest.
    RemoveIslands {
        /// Size threshold relative to the largest component (faces).
        min_fraction: f32,
    },
    /// Simplifies to `ratio` times the triangles.
    Decimate {
        /// Target fraction of the triangles, `(0, 1]`.
        ratio: f32,
    },
}

/// Effect of a cloud edit.
enum Change {
    /// Delete these point indices.
    Remove(Vec<u32>),
    /// Replace the cloud.
    Replace(PointCloud),
}

#[derive(Clone, Debug)]
struct Snapshot {
    cloud: Arc<PointCloud>,
    mesh: Option<Arc<TriMesh>>,
}

/// Editable scan: the cloud, the mesh built from it, undo / redo.
#[derive(Clone, Debug)]
pub struct ScanEditor {
    cloud: Arc<PointCloud>,
    mesh: Option<Arc<TriMesh>>,
    undo: Vec<Snapshot>,
    redo: Vec<Snapshot>,
}

fn positive(name: &str, v: f32) -> Result<f32> {
    if v.is_finite() && v > 0.0 {
        Ok(v)
    } else {
        Err(ScanError::InvalidParams(format!(
            "{name} must be > 0 (got {v})"
        )))
    }
}

fn at_least_one(name: &str, v: usize) -> Result<usize> {
    if v >= 1 {
        Ok(v)
    } else {
        Err(ScanError::InvalidParams(format!("{name} must be >= 1")))
    }
}

fn finite3(name: &str, v: [f32; 3]) -> Result<Vec3> {
    let v = Vec3::from(v);
    if v.is_finite() {
        Ok(v)
    } else {
        Err(ScanError::InvalidParams(format!("{name} must be finite")))
    }
}

fn mesh_summary(mesh: &TriMesh) -> Value {
    json!({ "vertices": mesh.vertex_count(), "faces": mesh.face_count() })
}

impl ScanEditor {
    /// Editor of `cloud`, without a mesh.
    pub fn new(cloud: PointCloud) -> Self {
        Self {
            cloud: Arc::new(cloud),
            mesh: None,
            undo: Vec::new(),
            redo: Vec::new(),
        }
    }

    /// The current cloud.
    pub fn cloud(&self) -> &Arc<PointCloud> {
        &self.cloud
    }

    /// The current mesh, once built.
    pub fn mesh(&self) -> Option<&Arc<TriMesh>> {
        self.mesh.as_ref()
    }

    /// `true` when [`ScanEditor::undo`] has something to undo.
    pub fn can_undo(&self) -> bool {
        !self.undo.is_empty()
    }

    /// `true` when [`ScanEditor::redo`] has something to redo.
    pub fn can_redo(&self) -> bool {
        !self.redo.is_empty()
    }

    fn snapshot(&self) -> Snapshot {
        Snapshot {
            cloud: Arc::clone(&self.cloud),
            mesh: self.mesh.clone(),
        }
    }

    fn push_undo(&mut self) {
        if self.undo.len() == UNDO_LIMIT {
            self.undo.remove(0);
        }
        self.undo.push(self.snapshot());
        self.redo.clear();
    }

    fn set_cloud(&mut self, cloud: PointCloud) {
        self.push_undo();
        self.cloud = Arc::new(cloud);
        self.mesh = None;
    }

    fn set_mesh(&mut self, mut mesh: TriMesh) {
        self.push_undo();
        mesh.update_normals();
        self.mesh = Some(Arc::new(mesh));
    }

    /// Applies a cloud edit. Returns `{"removed", "points"}`. A deletion
    /// that removes nothing leaves the history untouched.
    ///
    /// # Errors
    /// [`ScanError::Empty`] for an empty cloud, [`ScanError::InvalidParams`]
    /// for parameters out of range, kernel errors.
    pub fn apply_cloud(&mut self, op: &CloudOp) -> Result<Value> {
        let c = &*self.cloud;
        if c.is_empty() {
            return Err(ScanError::Empty("cloud"));
        }
        let before = c.len();
        let change = match *op {
            CloudOp::Downsample { voxel_mm } => {
                Change::Replace(cloud::voxel_downsample(c, positive("voxel_mm", voxel_mm)?))
            }
            CloudOp::RemoveStatistical { k, std_ratio } => {
                Change::Remove(cloud::statistical_outliers(
                    c,
                    at_least_one("k", k)?,
                    positive("std_ratio", std_ratio)?,
                ))
            }
            CloudOp::RemoveRadius {
                radius_mm,
                min_neighbors,
            } => Change::Remove(cloud::radius_outliers(
                c,
                positive("radius_mm", radius_mm)?,
                min_neighbors,
            )),
            CloudOp::Isolate { radius_mm } => {
                Change::Remove(cloud::largest_cluster(c, positive("radius_mm", radius_mm)?))
            }
            CloudOp::Smooth {
                k,
                iterations,
                strength,
            } => {
                if !(0.0..=1.0).contains(&strength) {
                    return Err(ScanError::InvalidParams(format!(
                        "strength {strength} not in [0, 1]"
                    )));
                }
                Change::Replace(cloud::smooth_cloud(
                    c,
                    at_least_one("k", k)?,
                    iterations,
                    strength,
                ))
            }
            CloudOp::DeleteAabb { min, max } => {
                let (min, max) = (finite3("min", min)?, finite3("max", max)?);
                Change::Remove(cloud::select_in_aabb(c, min.min(max), min.max(max)))
            }
            CloudOp::DeleteSphere { center, radius_mm } => Change::Remove(cloud::select_in_sphere(
                c,
                finite3("center", center)?,
                positive("radius_mm", radius_mm)?,
            )),
            CloudOp::DeletePlane { point, normal } => {
                let normal = finite3("normal", normal)?;
                if normal.length_squared() == 0.0 {
                    return Err(ScanError::InvalidParams("normal must not be zero".into()));
                }
                Change::Remove(cloud::select_by_plane(
                    c,
                    finite3("point", point)?,
                    normal.normalize(),
                ))
            }
            CloudOp::DeleteIndices { ref indices } => {
                if let Some(&i) = indices.iter().find(|&&i| i as usize >= before) {
                    return Err(ScanError::InvalidParams(format!(
                        "index {i} out of range ({before} points)"
                    )));
                }
                Change::Remove(indices.clone())
            }
            CloudOp::EstimateNormals { k } => {
                let k = at_least_one("k", k)?;
                let mut out = c.clone();
                cloud::estimate_normals(&mut out, k, None);
                Change::Replace(out)
            }
        };
        let new = match change {
            Change::Remove(mut remove) => {
                remove.sort_unstable();
                remove.dedup();
                if remove.is_empty() {
                    return Ok(json!({ "removed": 0, "points": before }));
                }
                c.remove_indices(&remove)
            }
            Change::Replace(new) => new,
        };
        let points = new.len();
        self.set_cloud(new);
        Ok(json!({ "removed": before.saturating_sub(points), "points": points }))
    }

    /// Builds the mesh from the cloud with a grid of `resolution_mm`.
    /// Returns `{"vertices", "faces"}`.
    ///
    /// # Errors
    /// [`ScanError::Empty`] for an empty cloud or an empty result,
    /// [`ScanError::InvalidParams`] for a resolution outside `[0.05, 5]` mm,
    /// [`ScanError::Cancelled`] when `cancel` returned `true`.
    pub fn build_mesh(&mut self, resolution_mm: f32, cancel: &dyn Fn() -> bool) -> Result<Value> {
        if !(0.05..=5.0).contains(&resolution_mm) {
            return Err(ScanError::InvalidParams(format!(
                "resolution_mm {resolution_mm} not in [0.05, 5]"
            )));
        }
        if self.cloud.is_empty() {
            return Err(ScanError::Empty("cloud"));
        }
        let params = CloudMeshParams {
            voxel_mm: resolution_mm,
            max_dist_mm: 0.0,
            k: MESH_K,
        };
        let mesh = mesh_from_cloud(&self.cloud, &params, cancel)?;
        if mesh.is_empty() {
            return Err(ScanError::Empty("mesh"));
        }
        let summary = mesh_summary(&mesh);
        self.set_mesh(mesh);
        Ok(summary)
    }

    /// Applies a mesh edit. Returns `{"vertices", "faces"}` plus
    /// `holes_filled` / `removed` for the operations that count something.
    ///
    /// # Errors
    /// [`ScanError::Empty`] before [`ScanEditor::build_mesh`],
    /// [`ScanError::InvalidParams`] for parameters out of range, kernel errors.
    pub fn apply_mesh(&mut self, op: &MeshOp) -> Result<Value> {
        let m = self.mesh.as_deref().ok_or(ScanError::Empty("mesh"))?;
        let (mesh, extra) = match *op {
            MeshOp::FillHoles { max_edges } => {
                let params = HoleFillParams {
                    max_boundary_edges: at_least_one("max_edges", max_edges)?,
                    ..HoleFillParams::default()
                };
                let (mesh, filled) = fill_holes(m, &params)?;
                (mesh, Some(("holes_filled", filled)))
            }
            MeshOp::Smooth {
                iterations,
                lambda,
                mu,
            } => {
                let ok = lambda > 0.0 && lambda <= 1.0 && (-1.0..0.0).contains(&mu);
                if !ok {
                    return Err(ScanError::InvalidParams(format!(
                        "need 0 < lambda <= 1 and -1 <= mu < 0 (got {lambda}, {mu})"
                    )));
                }
                (taubin_smooth(m, iterations, lambda, mu), None)
            }
            MeshOp::RemoveIslands { min_fraction } => {
                if !(0.0..=1.0).contains(&min_fraction) {
                    return Err(ScanError::InvalidParams(format!(
                        "min_fraction {min_fraction} not in [0, 1]"
                    )));
                }
                let (mesh, removed) = remove_small_components(m, min_fraction);
                (mesh, Some(("removed", removed)))
            }
            MeshOp::Decimate { ratio } => {
                if !(ratio > 0.0 && ratio <= 1.0) {
                    return Err(ScanError::InvalidParams(format!(
                        "ratio {ratio} not in (0, 1]"
                    )));
                }
                let params = DecimateParams::to_ratio(ratio as f64);
                (decimate(m, &params)?.mesh, None)
            }
        };
        if mesh.is_empty() {
            return Err(ScanError::Empty("mesh"));
        }
        let mut summary = mesh_summary(&mesh);
        if let Some((key, n)) = extra {
            summary[key] = json!(n);
        }
        self.set_mesh(mesh);
        Ok(summary)
    }

    /// Restores the state before the last edit; `false` when there is none.
    pub fn undo(&mut self) -> bool {
        let Some(prev) = self.undo.pop() else {
            return false;
        };
        self.redo.push(self.snapshot());
        self.cloud = prev.cloud;
        self.mesh = prev.mesh;
        true
    }

    /// Re-applies the last undone edit; `false` when there is none.
    pub fn redo(&mut self) -> bool {
        let Some(next) = self.redo.pop() else {
            return false;
        };
        self.undo.push(self.snapshot());
        self.cloud = next.cloud;
        self.mesh = next.mesh;
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Noise-free `n x n` grid on the plane z = 0 (0.25 mm spacing), normals +Z.
    fn plane(n: usize) -> PointCloud {
        let points: Vec<Vec3> = (0..n * n)
            .map(|i| Vec3::new((i % n) as f32, (i / n) as f32, 0.0) * 0.25)
            .collect();
        PointCloud {
            normals: Some(vec![Vec3::Z; points.len()]),
            points,
            colors: None,
        }
    }

    #[test]
    fn ops_serialize_with_kebab_case_tags() {
        let op: CloudOp =
            serde_json::from_value(json!({"op": "remove-statistical", "k": 8, "std_ratio": 2.0}))
                .unwrap();
        assert_eq!(
            op,
            CloudOp::RemoveStatistical {
                k: 8,
                std_ratio: 2.0
            }
        );
        let v = serde_json::to_value(MeshOp::FillHoles { max_edges: 50 }).unwrap();
        assert_eq!(v, json!({"op": "fill-holes", "max_edges": 50}));
    }

    #[test]
    fn statistical_removal_drops_injected_outliers() {
        let mut cloud = plane(40);
        let outliers = [
            Vec3::new(5.0, 5.0, 4.0),
            Vec3::new(2.0, 8.0, -5.0),
            Vec3::new(20.0, 20.0, 3.0),
        ];
        cloud.points.extend(outliers);
        cloud.normals.as_mut().unwrap().extend([Vec3::Z; 3]);
        let mut ed = ScanEditor::new(cloud);
        let r = ed
            .apply_cloud(&CloudOp::RemoveStatistical {
                k: 8,
                std_ratio: 2.0,
            })
            .unwrap();
        assert_eq!(r["removed"], 3, "{r}");
        assert!(ed.cloud().points.iter().all(|p| p.z == 0.0));
    }

    #[test]
    fn delete_aabb_and_undo_redo() {
        let mut ed = ScanEditor::new(plane(40));
        assert!(!ed.undo());
        let r = ed
            .apply_cloud(&CloudOp::DeleteAabb {
                min: [-1.0, -1.0, -1.0],
                max: [2.4, 2.4, 1.0],
            })
            .unwrap();
        // 10 x 10 grid points with x, y <= 2.4 (0.25 spacing: 0..=2.25)
        assert_eq!(r, json!({"removed": 100, "points": 1500}));
        // nothing inside: no history entry
        let r = ed
            .apply_cloud(&CloudOp::DeleteSphere {
                center: [0.0, 0.0, 0.0],
                radius_mm: 1.0,
            })
            .unwrap();
        assert_eq!(r["removed"], 0);
        assert!(ed.undo());
        assert_eq!(ed.cloud().len(), 1600);
        assert!(!ed.undo());
        assert!(ed.redo());
        assert_eq!(ed.cloud().len(), 1500);
        assert!(!ed.redo());
        assert_eq!(
            ed.apply_cloud(&CloudOp::DeleteIndices {
                indices: vec![5000]
            })
            .unwrap_err()
            .code(),
            "invalid-params"
        );
    }

    #[test]
    fn undo_history_is_capped() {
        let mut ed = ScanEditor::new(plane(30));
        for i in 0..UNDO_LIMIT + 5 {
            ed.apply_cloud(&CloudOp::DeleteIndices {
                indices: vec![i as u32],
            })
            .unwrap();
        }
        let mut undone = 0;
        while ed.undo() {
            undone += 1;
        }
        assert_eq!(undone, UNDO_LIMIT);
        assert_eq!(ed.cloud().len(), 900 - 5);
    }

    #[test]
    fn mesh_ops_on_a_meshed_plane() {
        let mut ed = ScanEditor::new(plane(40));
        let r = ed.build_mesh(0.5, &|| false).unwrap();
        let faces = ed.mesh().unwrap().face_count();
        assert_eq!(r["faces"], faces);
        assert!(faces > 100, "{r}");
        assert!(ed.mesh().unwrap().normals.is_some());

        let r = ed
            .apply_mesh(&MeshOp::RemoveIslands { min_fraction: 0.1 })
            .unwrap();
        assert_eq!(r["removed"], 0, "one component");
        let r = ed
            .apply_mesh(&MeshOp::Smooth {
                iterations: 3,
                lambda: 0.5,
                mu: -0.53,
            })
            .unwrap();
        assert_eq!(r["faces"], faces);
        let flat = ed
            .mesh()
            .unwrap()
            .positions
            .iter()
            .all(|p| p.z.abs() < 0.05);
        assert!(flat, "a plane stays flat");
        let e = ed
            .apply_mesh(&MeshOp::Smooth {
                iterations: 1,
                lambda: 0.5,
                mu: 0.5,
            })
            .unwrap_err();
        assert_eq!(e.code(), "invalid-params");
        let r = ed.apply_mesh(&MeshOp::Decimate { ratio: 0.5 }).unwrap();
        assert!(r["faces"].as_u64().unwrap() < faces as u64, "{r}");
        assert!(ed.undo());
        assert_eq!(ed.mesh().unwrap().face_count(), faces);
    }

    #[test]
    fn mesh_ops_need_a_mesh_and_valid_params() {
        let mut ed = ScanEditor::new(plane(10));
        let e = ed.apply_mesh(&MeshOp::Decimate { ratio: 0.5 }).unwrap_err();
        assert_eq!(e.code(), "empty");
        let e = ed.build_mesh(0.0, &|| false).unwrap_err();
        assert_eq!(e.code(), "invalid-params");
        let e = ed.build_mesh(0.5, &|| true).unwrap_err();
        assert_eq!(e.code(), "cancelled");
        assert!(ed.mesh().is_none());
        assert!(!ed.can_undo());
    }
}
