//! Fine reconstruction of a stop-and-capture scan from its kept stops.
//!
//! `scan.auto` (stop-and-capture) keeps every fused stop ([`KeptStop`]):
//! its median depth image (range filtered, cropped to the region of
//! interest, static clutter removed), its turntable angles and the pose it
//! was fused at. The live model is a coarse TSDF
//! ([`crate::ScanSettings::live_voxel_mm`]) made for tracking and the
//! preview; [`reconstruct`] rebuilds the surface from the stops at the
//! resolution of the scanner:
//!
//! 1. **Refinement and consistency gate** (optional, on by default): the
//!    stops are fused into a registration volume ([`REG_VOXEL_MM`]) with
//!    their pixel weights, tilt pass by tilt pass, starting from the
//!    reference pass (the tilt nearest 0) and going away from it:
//!    * each pass is registered as one rigid body onto the passes already
//!      in (multi-view point-to-plane ICP from a few millimetres away,
//!      [`PASS_GATES_MM`], at most [`PASS_MAX_DEG`] / [`PASS_MAX_MM`]):
//!      the object slipped once on the platter while it tilted;
//!    * then every stop of the pass is checked against those passes (the
//!      model ray cast from the stop's camera, the distance of its samples
//!      to the model surface: [`AGREE_TOL_MM`], [`AGREE_MAX_MEDIAN_MM`]).
//!      A stop that disagrees is registered once more on its own
//!      ([`RESCUE_MAX_DEG`] / [`RESCUE_MAX_MM`]: a small slip within the
//!      pass) and left out when it still disagrees; a pass of which most
//!      stops disagree ([`PASS_MIN_AGREE`]) is left out whole. Views of an
//!      object that moved during the scan are never fused, so they can
//!      never make double walls; the report counts them and warns
//!      ([`WARN_VIEWS_REJECTED`]);
//!    * for [`REFINE_ROUNDS`] rounds, each kept stop is taken out and
//!      registered onto the model of all the others from its pose; the
//!      result must stay within the limits of the live stop registration
//!      around its turntable pose (corrected by its pass) and must fit the
//!      others better than the current pose (a robust residual,
//!      [`crate::tracking`]): otherwise the current pose stays, so a stop
//!      never ends worse than at its turntable pose. In the last round
//!      every stop is checked once more against all the others and left
//!      out when it disagrees.
//! 2. **Fusion**: every stop is fused into a fine TSDF
//!    ([`ReconstructParams::resolution_mm`], 0.1 mm by default) in the scan
//!    world frame, each pixel weighted ([`depth_weights`]: depth edges
//!    eroded, grazing views and far samples weighted down) and kept only
//!    inside the region of interest (a cylinder around the turntable axis).
//! 3. **Extraction**: marching cubes straight from the volume over the
//!    voxels observed enough ([`ReconstructParams::min_weight`]): no point
//!    cloud round trip, no smoothing.
//!
//! The stops can be written to a directory and read back
//! ([`write_stops`], [`read_stops`]: `scan.export` with `what: "stops"`),
//! to tune the reconstruction offline on real scans.

use std::path::Path;
use std::sync::Arc;
use std::time::Instant;

use dc_geom::icp::{point_to_plane_solve, rotation_angle};
use dc_geom::tsdf::{CameraIntrinsics, DepthWeightParams, TsdfParams, TsdfVolume, depth_weights};
use dc_mesh::TriMesh;
use glam::{Affine3A, DAffine3, DVec3};
use rayon::prelude::*;
use serde::{Deserialize, Serialize};

use crate::rig::fit::angle_about;
use crate::rig::{ROI_BELOW_PLATTER_MM, RigCalibration, RigGeometry};
use crate::session::{STOP_MAX_DELTA_DEG, STOP_MAX_OFF_AXIS_DEG, STOP_MAX_SHIFT_MM};
use crate::tracking::{
    FrameSample, ModelView, TrackParams, sample_frame, to_d, to_f, track_in_view, tukey,
};
use crate::{Result, ScanError};

/// Default voxel of the fine reconstruction (mm).
pub const DEFAULT_RESOLUTION_MM: f32 = 0.1;
/// Range of [`ReconstructParams::resolution_mm`] (mm).
pub const RESOLUTION_RANGE_MM: (f32, f32) = (0.05, 1.0);
/// Voxel of the registration volume of the refinement (mm).
pub const REG_VOXEL_MM: f32 = 0.25;
/// Truncation of the registration volume (mm).
const REG_TRUNC_MM: f32 = 1.0;
/// Rounds of the refinement.
pub const REFINE_ROUNDS: usize = 2;
/// Association gates of the stages of the alignment of a tilt pass onto
/// the passes nearer to tilt 0 (mm): it starts from a few millimetres away.
pub const PASS_GATES_MM: [f64; 5] = [3.0, 1.5, 0.8, 0.4, 0.25];
/// Iterations per stage of the pass alignment, at most.
const PASS_ITERATIONS: usize = 6;
/// Samples per stop of the pass alignment, about.
const PASS_SAMPLES: usize = 3000;
/// Largest rotation of the correction of a tilt pass (degrees).
pub const PASS_MAX_DEG: f64 = 5.0;
/// Largest shift of the samples of a tilt pass by its correction (mm).
pub const PASS_MAX_MM: f64 = 5.0;
/// Association gates of the two calls of the registration that brings back
/// a stop that moved a little within its pass (mm).
const RESCUE_GATES_MM: [[f32; 3]; 2] = [[2.0, 1.2, 0.8], [0.5, 0.35, 0.25]];
/// Largest rotation of that registration (degrees).
pub const RESCUE_MAX_DEG: f64 = 3.0;
/// Largest shift of the samples by that registration (mm).
pub const RESCUE_MAX_MM: f64 = 3.0;
/// Agreement of a stop with the model of the others: a sample is close to
/// the model surface within this (mm).
pub const AGREE_TOL_MM: f64 = 0.25;
/// A sample farther than this behind the model surface sees through it, a
/// conflict (mm); one this far in front sees what the model lacks (the
/// stop's own coverage) and is not judged.
pub const AGREE_FAR_MM: f64 = 0.5;
/// A stop agrees when at least this fraction of its judged samples is
/// close...
pub const AGREE_MIN_CLOSE: f64 = 0.5;
/// ... at most this fraction sees through the model surface...
pub const AGREE_MAX_BEHIND: f64 = 0.1;
/// ... and their median distance to the model surface is at most this
/// (mm). Registered views of the MINI 2 show 0.03-0.1 mm, views of an
/// object that moved 0.3 mm and more.
pub const AGREE_MAX_MEDIAN_MM: f64 = 0.2;
/// A sample and the model surface at its pixel face alike within this
/// (degrees): otherwise it is another surface, not compared.
const AGREE_NORMAL_DEG: f32 = 45.0;
/// The model of an agreement check is ray cast at most this many pixels
/// wide.
const AGREE_VIEW_WIDTH: u32 = 200;
/// Fewest overlapping samples to judge a stop...
const AGREE_MIN_OVERLAP: usize = 200;
/// ... and as a fraction of its samples.
const AGREE_MIN_OVERLAP_FRACTION: f64 = 0.1;
/// Samples per stop of the agreement checks, about.
const AGREE_SAMPLES: usize = 4000;
/// A pass of which fewer than this fraction of the stops that can be
/// judged agree with the passes nearer to tilt 0 is left out whole.
pub const PASS_MIN_AGREE: f64 = 0.5;
/// Without a turntable pose to refer to, the largest correction of a stop
/// from its pose (degrees, mm).
const FREE_MAX_STEP: (f64, f64) = (1.0, 1.0);
/// A new pose must lower the robust residual by this fraction.
const MIN_GAIN: f64 = 1e-3;
/// Tilts within this of each other (degrees) belong to one pass.
const PASS_TILT_TOL_DEG: f32 = 0.05;
/// Margin of the crop of a stop image around its valid pixels (pixels).
const CROP_MARGIN: u32 = 4;
/// Truncation of the fine volume: at least this (mm), the depth noise and
/// the residual misregistration of the views.
const MIN_TRUNC_MM: f32 = 0.3;
/// Truncation of the fine volume in voxels, when larger.
const TRUNC_VOXELS: f32 = 4.0;
/// Weight cap of the volumes: none in practice (a true mean; frames are
/// taken out of the registration volume).
const NO_CAP: f32 = 1e9;

/// A fused stop kept by `scan.auto` (stop-and-capture).
#[derive(Clone, Debug, PartialEq)]
pub struct KeptStop {
    /// Valid pixels of the stop image: row-major index and depth (mm).
    pub pixels: Arc<Vec<(u32, f32)>>,
    /// Intrinsics of the stop image.
    pub intrinsics: CameraIntrinsics,
    /// Turntable angles `(rotate, tilt)` (degrees, as read).
    pub angles: (f32, f32),
    /// Camera-to-fusion pose the stop was fused at (the fusion frame is the
    /// camera frame of the first fused stop).
    pub pose: Affine3A,
    /// Its live registration was refused: fused at the turntable pose.
    pub flagged: bool,
}

impl KeptStop {
    /// The full depth image (`NaN` where no measurement).
    pub fn depth(&self) -> Vec<f32> {
        let mut d = vec![f32::NAN; self.intrinsics.pixel_count()];
        for &(i, z) in self.pixels.iter() {
            if let Some(x) = d.get_mut(i as usize) {
                *x = z;
            }
        }
        d
    }

    /// The image cropped to its valid pixels (plus a margin): depth (`NaN`
    /// invalid), intrinsics of the crop and its corner `(x0, y0)` in the
    /// full image; `None` without valid pixels.
    pub fn crop(&self) -> Option<(Vec<f32>, CameraIntrinsics, (u32, u32))> {
        let k = &self.intrinsics;
        let w = k.width.max(1);
        let (mut x0, mut y0, mut x1, mut y1) = (u32::MAX, u32::MAX, 0, 0);
        for &(i, _) in self.pixels.iter() {
            let (x, y) = (i % w, i / w);
            x0 = x0.min(x);
            y0 = y0.min(y);
            x1 = x1.max(x);
            y1 = y1.max(y);
        }
        if x0 > x1 || y1 >= k.height {
            return None;
        }
        let x0 = x0.saturating_sub(CROP_MARGIN);
        let y0 = y0.saturating_sub(CROP_MARGIN);
        let x1 = (x1 + CROP_MARGIN).min(k.width - 1);
        let y1 = (y1 + CROP_MARGIN).min(k.height - 1);
        let (cw, ch) = (x1 - x0 + 1, y1 - y0 + 1);
        let mut d = vec![f32::NAN; (cw * ch) as usize];
        for &(i, z) in self.pixels.iter() {
            let (x, y) = (i % w, i / w);
            d[((y - y0) * cw + (x - x0)) as usize] = z;
        }
        let intr = CameraIntrinsics {
            width: cw,
            height: ch,
            fx: k.fx,
            fy: k.fy,
            cx: k.cx - x0 as f32,
            cy: k.cy - y0 as f32,
        };
        Some((d, intr, (x0, y0)))
    }
}

/// The kept stops of a scan and the frames they live in.
#[derive(Clone, Debug, PartialEq)]
pub struct KeptStops {
    /// The stops, in capture order.
    pub stops: Vec<KeptStop>,
    /// The scan world frame from the fusion frame (see
    /// [`crate::ScanStatus::world_from_camera`]).
    pub world: Affine3A,
    /// Turntable angles `(rotate, tilt)` of the world frame (first fused
    /// stop).
    pub world_angles: Option<(f32, f32)>,
    /// The rig calibration the scan was taken with.
    pub calibration: Option<RigCalibration>,
    /// The region of interest in the scan world (`radius`, `height`, mm): a
    /// cylinder around +Z from just under the platter, when in use.
    pub roi: Option<(f32, f32)>,
}

/// Parameters of [`reconstruct`].
#[derive(Clone, Debug, PartialEq)]
pub struct ReconstructParams {
    /// Voxel of the fine volume, the mesh resolution (mm), in
    /// [`RESOLUTION_RANGE_MM`].
    pub resolution_mm: f32,
    /// Refine the poses of the stops first.
    pub refine: bool,
    /// Voxels with less integration weight are not observed (a full-weight
    /// view adds about 1).
    pub min_weight: f32,
}

impl Default for ReconstructParams {
    fn default() -> Self {
        Self {
            resolution_mm: DEFAULT_RESOLUTION_MM,
            refine: true,
            min_weight: 1.0,
        }
    }
}

/// What [`reconstruct`] did.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct ReconstructionReport {
    /// Voxel of the fine volume (mm).
    pub resolution_mm: f32,
    /// Stops fused (the stops with measurements but the rejected ones).
    pub stops: usize,
    /// Stops left out of the fusion: they disagree with the consensus of
    /// the others (the object moved on the platter, or a pose is wrong),
    /// so they can never make double walls.
    #[serde(default)]
    pub rejected_stops: usize,
    /// Whole tilt passes left out: most of their stops disagree with the
    /// passes nearer to tilt 0.
    #[serde(default)]
    pub rejected_passes: usize,
    /// Stops that disagreed after their pass was aligned and were
    /// registered again on their own (a small slip within the pass).
    #[serde(default)]
    pub rescued_stops: usize,
    /// Stops whose pose the refinement changed.
    pub refined: usize,
    /// Mean RMS residual of the stops against the model of all the others
    /// after the refinement (mm); `None` without refinement.
    pub mean_rms_mm: Option<f32>,
    /// The same at the poses the refinement started from (mm).
    pub mean_rms_before_mm: Option<f32>,
    /// Tilt passes.
    pub passes: usize,
    /// Tilt passes whose rigid correction was applied.
    pub passes_corrected: usize,
    /// Per tilt pass (refinement only), by tilt.
    #[serde(default)]
    pub pass_details: Vec<PassReport>,
    /// [`WARN_VIEWS_REJECTED`] when views were left out: the mesh lacks
    /// what only they saw (fix the object on the platter and scan again).
    #[serde(default)]
    pub warning: Option<String>,
    /// Vertices of the mesh.
    pub vertices: usize,
    /// Faces of the mesh.
    pub faces: usize,
    /// Allocated blocks of the fine volume.
    pub blocks: usize,
    /// Memory of the fine volume (MB).
    pub memory_mb: f32,
    /// Total time (s).
    pub seconds: f32,
    /// Time of the refinement (s).
    pub refine_s: f32,
    /// Time of the fusion (s).
    pub fuse_s: f32,
    /// Time of the extraction (s).
    pub extract_s: f32,
}

/// A stage of [`reconstruct`], for progress reports.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReconstructStage {
    /// Refining the poses.
    Refine,
    /// Fusing into the fine volume.
    Fuse,
    /// Extracting the mesh.
    Extract,
}

/// Result of [`reconstruct`].
#[derive(Clone, Debug)]
pub struct Reconstruction {
    /// The mesh, in the scan world frame, faces counter-clockwise seen from
    /// outside.
    pub mesh: TriMesh,
    /// The (refined) camera-to-fusion pose of every stop.
    pub poses: Vec<Affine3A>,
    /// Per stop, its RMS residual against the model of the others after
    /// the refinement (mm; `NaN` when not measured).
    pub stop_rms_mm: Vec<f32>,
    /// Per stop: left out of the fusion (it disagrees with the others).
    pub rejected: Vec<bool>,
    /// What was done.
    pub report: ReconstructionReport,
}

/// A stop ready to fuse: cropped image, weights, intrinsics.
struct Prepared {
    depth: Vec<f32>,
    weights: Vec<f32>,
    intr: CameraIntrinsics,
    angles: (f32, f32),
}

/// Rebuilds the surface of `stops` (see the module doc). `stage` is called
/// when a stage starts; `cancel` is polled between the stops.
///
/// # Errors
/// [`ScanError::NoStops`] without stops, [`ScanError::InvalidParams`] for
/// a resolution out of range, [`ScanError::Empty`] when nothing is
/// observed enough, [`ScanError::Cancelled`].
pub fn reconstruct(
    stops: &KeptStops,
    params: &ReconstructParams,
    stage: &dyn Fn(ReconstructStage),
    cancel: &dyn Fn() -> bool,
) -> Result<Reconstruction> {
    let (lo, hi) = RESOLUTION_RANGE_MM;
    if !(lo..=hi).contains(&params.resolution_mm) {
        return Err(ScanError::InvalidParams(format!(
            "resolution_mm {} not in [{lo}, {hi}]",
            params.resolution_mm
        )));
    }
    if !(params.min_weight.is_finite() && params.min_weight >= 0.0) {
        return Err(ScanError::InvalidParams(format!(
            "min_weight {} must be >= 0",
            params.min_weight
        )));
    }
    if stops.stops.is_empty() {
        return Err(ScanError::NoStops);
    }
    let check = || {
        if cancel() {
            Err(ScanError::Cancelled)
        } else {
            Ok(())
        }
    };
    let started = Instant::now();
    let geometry = stops.calibration.as_ref().map(RigGeometry::new);
    let prepared: Vec<Option<Prepared>> = stops
        .stops
        .par_iter()
        .map(|s| prepare(s, stops.world, stops.roi))
        .collect();
    let mut report = ReconstructionReport {
        resolution_mm: params.resolution_mm,
        stops: prepared.iter().flatten().count(),
        ..ReconstructionReport::default()
    };
    if report.stops == 0 {
        return Err(ScanError::Empty("stops without measurements"));
    }
    let mut poses: Vec<Affine3A> = stops.stops.iter().map(|s| s.pose).collect();
    let mut stop_rms = vec![f32::NAN; poses.len()];
    let mut rejected = vec![false; poses.len()];

    stage(ReconstructStage::Refine);
    check()?;
    let t = Instant::now();
    if params.refine {
        let out = refine(
            &prepared,
            &mut poses,
            geometry.as_ref(),
            stops.world_angles,
            cancel,
        )?;
        report.refined = out.refined;
        report.passes = out.passes.len();
        report.passes_corrected = out.passes_corrected;
        report.rejected_passes = out.passes.iter().filter(|p| p.rejected).count();
        report.rescued_stops = out.passes.iter().map(|p| p.rescued_stops).sum();
        report.pass_details = out.passes;
        let kept = |v: &[f32]| -> Vec<f32> {
            v.iter()
                .zip(&out.rejected)
                .map(|(r, rej)| if *rej { f32::NAN } else { *r })
                .collect()
        };
        report.mean_rms_before_mm = mean(&kept(&out.rms_first));
        stop_rms = kept(&out.rms_last);
        report.mean_rms_mm = mean(&stop_rms);
        rejected = out.rejected;
    }
    report.rejected_stops = prepared
        .iter()
        .zip(&rejected)
        .filter(|(p, r)| p.is_some() && **r)
        .count();
    report.stops -= report.rejected_stops;
    if report.rejected_stops > 0 {
        report.warning = Some(WARN_VIEWS_REJECTED.to_owned());
    }
    if report.stops == 0 {
        return Err(ScanError::Empty("stops that agree with each other"));
    }
    report.refine_s = t.elapsed().as_secs_f32();

    stage(ReconstructStage::Fuse);
    let t = Instant::now();
    let voxel = params.resolution_mm;
    let mut fine = TsdfVolume::new(TsdfParams {
        voxel_mm: voxel,
        trunc_mm: (TRUNC_VOXELS * voxel).max(MIN_TRUNC_MM),
        max_weight: NO_CAP,
    });
    for ((p, pose), rej) in prepared.iter().zip(&poses).zip(&rejected) {
        check()?;
        if let (Some(p), false) = (p, *rej) {
            fine.integrate_weighted(&p.depth, &p.weights, &p.intr, &(stops.world * *pose));
        }
    }
    report.fuse_s = t.elapsed().as_secs_f32();
    report.blocks = fine.block_count();
    report.memory_mb = fine.approx_bytes() as f32 / (1024.0 * 1024.0);

    stage(ReconstructStage::Extract);
    check()?;
    let t = Instant::now();
    let mesh = fine.extract_mesh_with(params.min_weight);
    drop(fine);
    report.extract_s = t.elapsed().as_secs_f32();
    if mesh.is_empty() {
        return Err(ScanError::Empty("mesh"));
    }
    report.vertices = mesh.vertex_count();
    report.faces = mesh.face_count();
    report.seconds = started.elapsed().as_secs_f32();
    Ok(Reconstruction {
        mesh,
        poses,
        stop_rms_mm: stop_rms,
        rejected,
        report,
    })
}

/// Mean of the finite values, `None` without any.
fn mean(v: &[f32]) -> Option<f32> {
    let (s, n) = v
        .iter()
        .filter(|x| x.is_finite())
        .fold((0.0f32, 0usize), |(s, n), x| (s + x, n + 1));
    (n > 0).then(|| s / n as f32)
}

/// Crops a stop, keeps its pixels inside the region of interest `roi` of
/// the world (placed by its pose) and weighs them.
fn prepare(s: &KeptStop, world: Affine3A, roi: Option<(f32, f32)>) -> Option<Prepared> {
    let (mut depth, intr, _) = s.crop()?;
    if let Some((radius, height)) = roi {
        let to_world = world * s.pose;
        let w = intr.width as usize;
        for (i, d) in depth.iter_mut().enumerate() {
            if !d.is_finite() {
                continue;
            }
            let p = to_world.transform_point3(intr.ray((i % w) as f32, (i / w) as f32) * *d);
            let inside = p.truncate().length_squared() <= radius * radius
                && (-ROI_BELOW_PLATTER_MM..=height).contains(&p.z);
            if !inside {
                *d = f32::NAN;
            }
        }
    }
    let weights = depth_weights(&depth, &intr, &DepthWeightParams::default());
    weights.iter().any(|w| *w > 0.0).then_some(Prepared {
        depth,
        weights,
        intr,
        angles: s.angles,
    })
}

/// Tracking parameters of the refinement: a near pose, a fine model view
/// and tight gates.
fn refine_params(target_points: usize) -> TrackParams {
    TrackParams {
        gates_mm: [0.8, 0.4, 0.25],
        iterations_per_stage: 8,
        min_inliers: 200,
        min_inlier_fraction: 0.2,
        max_rms_mm: 0.35,
        max_step_mm: 2.0,
        max_step_deg: 2.0,
        max_model_width: 400,
        target_points,
        ..TrackParams::default()
    }
}

/// Gate of the robust residual that compares two poses (mm).
const FIT_GATE_MM: f64 = 0.25;

/// One tilt pass of a [`ReconstructionReport`].
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct PassReport {
    /// Tilt of the pass (degrees).
    pub tilt_deg: f32,
    /// Its stops.
    pub stops: usize,
    /// Rotation of the rigid correction applied to the whole pass
    /// (degrees; 0 when none was applied).
    pub corrected_deg: f32,
    /// Shift of the scanned surface by that correction (mm).
    pub corrected_mm: f32,
    /// Stops that disagreed after the pass correction and were registered
    /// again on their own (a small slip within the pass).
    pub rescued_stops: usize,
    /// Stops left out of the fusion.
    pub rejected_stops: usize,
    /// The whole pass was left out: most of its stops disagree with the
    /// passes nearer to tilt 0.
    pub rejected: bool,
}

/// Warning of a reconstruction that left views out
/// ([`ReconstructionReport::warning`]).
pub const WARN_VIEWS_REJECTED: &str = "scan-views-rejected";

/// What [`refine`] did.
struct RefineOutcome {
    refined: usize,
    passes_corrected: usize,
    rms_first: Vec<f32>,
    rms_last: Vec<f32>,
    /// Per stop: left out of the fusion.
    rejected: Vec<bool>,
    /// Per pass, in tilt order.
    passes: Vec<PassReport>,
}

/// Camera pose of the turntable angles `angles` in the fusion frame whose
/// first stop was taken at `world` (the stop-and-capture prior).
fn table_pose(g: &RigGeometry, world: (f32, f32), angles: (f32, f32)) -> Affine3A {
    g.platter(world.0, world.1) * g.platter(angles.0, angles.1).inverse()
}

/// The stops grouped by tilt (passes), in capture order.
fn passes(prepared: &[Option<Prepared>]) -> Vec<Vec<usize>> {
    let mut out: Vec<(f32, Vec<usize>)> = Vec::new();
    for (i, p) in prepared.iter().enumerate() {
        let Some(p) = p else { continue };
        match out
            .iter_mut()
            .find(|(t, _)| (t - p.angles.1).abs() <= PASS_TILT_TOL_DEG)
        {
            Some((_, list)) => list.push(i),
            None => out.push((p.angles.1, vec![i])),
        }
    }
    out.into_iter().map(|(_, l)| l).collect()
}

/// The refinement and the consistency gate (see the module doc).
fn refine(
    prepared: &[Option<Prepared>],
    poses: &mut [Affine3A],
    geometry: Option<&RigGeometry>,
    world_angles: Option<(f32, f32)>,
    cancel: &dyn Fn() -> bool,
) -> Result<RefineOutcome> {
    let n = poses.len();
    let mut volume = TsdfVolume::new(TsdfParams {
        voxel_mm: REG_VOXEL_MM,
        trunc_mm: REG_TRUNC_MM,
        max_weight: NO_CAP,
    });
    let add = |volume: &mut TsdfVolume, i: usize, pose: &Affine3A| {
        if let Some(p) = &prepared[i] {
            volume.integrate_weighted(&p.depth, &p.weights, &p.intr, pose);
        }
    };
    let take = |volume: &mut TsdfVolume, i: usize, pose: &Affine3A| {
        if let Some(p) = &prepared[i] {
            volume.deintegrate_weighted(&p.depth, &p.weights, &p.intr, pose);
        }
    };
    let check = || {
        if cancel() {
            Err(ScanError::Cancelled)
        } else {
            Ok(())
        }
    };
    // turntable poses: the references of the limits (corrected per pass)
    let mut reference: Vec<Option<Affine3A>> = (0..n)
        .map(|i| match (geometry, world_angles, &prepared[i]) {
            (Some(g), Some(w), Some(p)) => Some(table_pose(g, w, p.angles)),
            _ => None,
        })
        .collect();
    // samples of the agreement checks
    let checks: Vec<Option<FrameSample>> = prepared
        .par_iter()
        .map(|p| p.as_ref().map(|p| samples(p, AGREE_SAMPLES)))
        .collect();
    let initial: Vec<Affine3A> = poses.to_vec();
    let mut groups = passes(prepared);
    let tilt = |g: &[usize]| prepared[g[0]].as_ref().map_or(0.0, |p| p.angles.1);
    // the reference pass (the tilt nearest 0, the larger on a tie) first,
    // then the others away from it
    groups.sort_by(|a, b| {
        tilt(a)
            .abs()
            .total_cmp(&tilt(b).abs())
            .then(b.len().cmp(&a.len()))
    });
    let mut out = RefineOutcome {
        refined: 0,
        passes_corrected: 0,
        rms_first: vec![f32::NAN; n],
        rms_last: vec![f32::NAN; n],
        rejected: vec![false; n],
        passes: Vec::with_capacity(groups.len()),
    };
    let mut rescued = vec![false; n];
    for (rank, group) in groups.iter().enumerate() {
        check()?;
        let mut pass = PassReport {
            tilt_deg: tilt(group),
            stops: group.len(),
            ..PassReport::default()
        };
        if rank > 0 {
            // the pass as one body onto the passes accepted so far, from a
            // few millimetres away (the object slipped while tilting)
            if let Some((c, deg, mm)) = align_pass(&volume, prepared, poses, group) {
                for &i in group {
                    poses[i] = to_f(&(c * to_d(&poses[i])));
                    reference[i] = reference[i].map(|r| to_f(&(c * to_d(&r))));
                }
                (pass.corrected_deg, pass.corrected_mm) = (deg as f32, mm as f32);
                out.passes_corrected += 1;
            }
            let verdicts: Vec<Verdict> = group
                .iter()
                .map(|&i| match (&prepared[i], &checks[i]) {
                    (Some(p), Some(f)) => agreement(&volume, p, f, &poses[i]).verdict(),
                    _ => Verdict::Unknown,
                })
                .collect();
            let agree = verdicts.iter().filter(|v| **v == Verdict::Agrees).count();
            let known = verdicts.iter().filter(|v| **v != Verdict::Unknown).count();
            if known > 0 && (agree as f64) < PASS_MIN_AGREE * known as f64 {
                pass.rejected = true;
                for &i in group {
                    out.rejected[i] = true;
                }
            } else {
                for (v, &i) in verdicts.iter().zip(group) {
                    check()?;
                    let rescue = match (v, &prepared[i], &checks[i]) {
                        // it moved within the pass: once more on its own,
                        // from a little farther away
                        (Verdict::Disagrees, Some(p), Some(f)) => {
                            rescue_stop(&volume, p, f, &poses[i])
                        }
                        _ => None,
                    };
                    if let Some(pose) = rescue {
                        poses[i] = pose;
                        // the turntable no longer tells where this view was
                        reference[i] = None;
                        rescued[i] = true;
                        pass.rescued_stops += 1;
                    } else if *v == Verdict::Disagrees {
                        out.rejected[i] = true;
                        continue;
                    }
                    add(&mut volume, i, &poses[i]);
                }
            }
        } else {
            for &i in group {
                add(&mut volume, i, &poses[i]);
            }
        }
        out.passes.push(pass);
    }
    for round in 0..REFINE_ROUNDS {
        let last = round + 1 == REFINE_ROUNDS;
        for i in 0..n {
            let Some(p) = &prepared[i] else { continue };
            if out.rejected[i] {
                continue;
            }
            check()?;
            take(&mut volume, i, &poses[i]);
            let r = register_stop(&volume, p, &poses[i], reference[i], geometry);
            if round == 0 {
                out.rms_first[i] = r.rms_before;
            }
            out.rms_last[i] = r.rms_after;
            if let Some(pose) = r.pose {
                poses[i] = pose;
            }
            // the gate: every view against the consensus of all the others
            if last
                && let Some(f) = &checks[i]
                && agreement(&volume, p, f, &poses[i]).verdict() == Verdict::Disagrees
            {
                out.rejected[i] = true;
                continue;
            }
            add(&mut volume, i, &poses[i]);
        }
    }
    for (pass, group) in out.passes.iter_mut().zip(&groups) {
        pass.rejected_stops = group.iter().filter(|&&i| out.rejected[i]).count();
    }
    out.passes.sort_by(|a, b| a.tilt_deg.total_cmp(&b.tilt_deg));
    out.refined = poses
        .iter()
        .zip(&initial)
        .zip(&out.rejected)
        .filter(|((a, b), r)| !**r && a != b)
        .count();
    if let (Some(g), Some(w)) = (geometry, world_angles) {
        let gauge: Vec<Option<Affine3A>> = reference
            .iter()
            .zip(&out.rejected)
            .zip(&rescued)
            .map(|((r, rej), res)| r.filter(|_| !*rej && !*res))
            .collect();
        regauge(poses, &gauge, g.axis_at(w.1).0);
    }
    Ok(out)
}

/// Moves every pose by the one rigid motion (about `centre`) that brings
/// them back onto their references on average: the registrations measure
/// only how the stops sit relative to each other, so the whole model may
/// slide while it is refined; the turntable keeps it where it belongs.
/// Stops without a reference do not count.
fn regauge(poses: &mut [Affine3A], reference: &[Option<Affine3A>], centre: glam::Vec3) {
    let c0 = centre.as_dvec3();
    let (mut rot, mut shift, mut k) = (DVec3::ZERO, DVec3::ZERO, 0usize);
    for (p, r) in poses.iter().zip(reference) {
        let Some(r) = r else { continue };
        let c = to_d(r) * to_d(p).inverse();
        rot += glam::DQuat::from_mat3(&c.matrix3).to_scaled_axis();
        shift += c.transform_point3(c0) - c0;
        k += 1;
    }
    if k == 0 {
        return;
    }
    let g = DAffine3::from_translation(c0 + shift / k as f64)
        * DAffine3::from_quat(glam::DQuat::from_scaled_axis(rot / k as f64))
        * DAffine3::from_translation(-c0);
    for p in poses.iter_mut() {
        *p = to_f(&(g * to_d(p)));
    }
}

/// Outcome of [`register_stop`].
struct StopFit {
    /// The better pose, when found.
    pose: Option<Affine3A>,
    /// RMS residual at the incoming pose (mm).
    rms_before: f32,
    /// RMS residual at the pose kept (mm).
    rms_after: f32,
}

/// Samples of a prepared stop for registration.
fn samples(p: &Prepared, target_points: usize) -> FrameSample {
    let params = refine_params(target_points);
    // drop the eroded pixels: the samples then come from the trusted ones
    let depth: Vec<f32> = p
        .depth
        .iter()
        .zip(&p.weights)
        .map(|(d, w)| if *w > 0.0 { *d } else { f32::NAN })
        .collect();
    sample_frame(&depth, &p.intr, &params)
}

/// Registers stop `p` onto `volume` (the model of the others) from `pose`;
/// see the module doc for the acceptance.
fn register_stop(
    volume: &TsdfVolume,
    p: &Prepared,
    pose: &Affine3A,
    reference: Option<Affine3A>,
    geometry: Option<&RigGeometry>,
) -> StopFit {
    let params = refine_params(10_000);
    let frame = samples(p, params.target_points);
    let view = ModelView::new(volume, &p.intr, pose, &params);
    let before = view.fit(&frame, &to_d(pose), FIT_GATE_MM);
    let mut out = StopFit {
        pose: None,
        rms_before: before.rms_mm as f32,
        rms_after: before.rms_mm as f32,
    };
    let r = track_in_view(&view, &frame, pose, &params);
    if !r.ok {
        return out;
    }
    let within = match (reference, geometry) {
        (Some(prior), Some(g)) => within_stop_limits(g, &prior, &r.pose, p.angles.1),
        _ => {
            let c = to_d(pose).inverse() * to_d(&r.pose);
            rotation_angle(&c).to_degrees() <= FREE_MAX_STEP.0
                && c.translation.length() <= FREE_MAX_STEP.1
        }
    };
    if !within {
        return out;
    }
    let after = view.fit(&frame, &to_d(&r.pose), FIT_GATE_MM);
    if after.cost < before.cost * (1.0 - MIN_GAIN) {
        out.pose = Some(r.pose);
        out.rms_after = after.rms_mm as f32;
    }
    out
}

/// `true` when the camera pose `pose` stays within the limits of the live
/// stop registration around the turntable pose `prior` at tilt `tilt`:
/// [`STOP_MAX_DELTA_DEG`] about the turntable axis,
/// [`STOP_MAX_OFF_AXIS_DEG`] off it, [`STOP_MAX_SHIFT_MM`] of the axis
/// point.
pub(crate) fn within_stop_limits(
    g: &RigGeometry,
    prior: &Affine3A,
    pose: &Affine3A,
    tilt: f32,
) -> bool {
    let (delta, off, shift) = stop_correction(g, prior, pose, tilt);
    delta.abs() <= STOP_MAX_DELTA_DEG && off <= STOP_MAX_OFF_AXIS_DEG && shift <= STOP_MAX_SHIFT_MM
}

/// The correction `prior^-1 * pose` of a stop: its turn about the
/// turntable axis (degrees, signed), its turn off the axis (degrees) and
/// the shift of the axis point (mm).
pub(crate) fn stop_correction(
    g: &RigGeometry,
    prior: &Affine3A,
    pose: &Affine3A,
    tilt: f32,
) -> (f64, f64, f64) {
    let c = to_d(prior).inverse() * to_d(pose);
    let (point, dir) = g.axis_at(tilt);
    let delta = angle_about(&c.matrix3, dir.as_dvec3()).to_degrees();
    let total = rotation_angle(&c).to_degrees();
    let off = (total * total - delta * delta).max(0.0).sqrt();
    let shift = c
        .transform_point3(point.as_dvec3())
        .distance(point.as_dvec3());
    (delta, off, shift)
}

/// Centroid of the samples of `frames` placed by their poses.
fn samples_centroid<'a>(frames: impl Iterator<Item = (&'a FrameSample, DAffine3)>) -> DVec3 {
    let (sum, count) = frames.fold((DVec3::ZERO, 0usize), |(s, k), (f, pose)| {
        (
            s + f
                .points
                .iter()
                .map(|x| pose.transform_point3(x.as_dvec3()))
                .sum::<DVec3>(),
            k + f.points.len(),
        )
    });
    sum / count.max(1) as f64
}

/// Registers the stops `group` (not in `volume`) as one rigid body onto
/// `volume`, from a few millimetres away: stages of shrinking association
/// gates ([`PASS_GATES_MM`]), the model ray cast again at the start of
/// every stage. The world-side correction (and its rotation in degrees and
/// the shift of the samples in millimetres) when it stays within
/// [`PASS_MAX_DEG`] / [`PASS_MAX_MM`] and fits better.
fn align_pass(
    volume: &TsdfVolume,
    prepared: &[Option<Prepared>],
    poses: &[Affine3A],
    group: &[usize],
) -> Option<(DAffine3, f64, f64)> {
    let params = TrackParams {
        max_model_width: AGREE_VIEW_WIDTH,
        ..refine_params(PASS_SAMPLES)
    };
    let frames: Vec<(&Prepared, FrameSample, DAffine3)> = group
        .iter()
        .filter_map(|&i| {
            let p = prepared[i].as_ref()?;
            Some((p, samples(p, params.target_points), to_d(&poses[i])))
        })
        .collect();
    if frames.is_empty() {
        return None;
    }
    let mut c = DAffine3::IDENTITY;
    let mut views: Vec<ModelView> = Vec::new();
    for &gate in &PASS_GATES_MM {
        views = frames
            .iter()
            .map(|(p, _, pose)| ModelView::new(volume, &p.intr, &to_f(&(c * *pose)), &params))
            .collect();
        for _ in 0..PASS_ITERATIONS {
            let (mut p, mut q, mut nrm, mut w) = (Vec::new(), Vec::new(), Vec::new(), Vec::new());
            for ((_, frame, pose), view) in frames.iter().zip(&views) {
                for (a, b, n, _) in view.pairs(frame, &(c * *pose), gate) {
                    w.push(tukey((b - a).dot(n), gate));
                    p.push(a);
                    q.push(b);
                    nrm.push(n);
                }
            }
            if p.len() < params.min_inliers {
                return None;
            }
            let solve =
                point_to_plane_solve(&p, &q, &nrm, Some(&w), params.min_relative_eigen).ok()?;
            c = solve.delta * c;
            if rotation_angle(&solve.delta) < 1e-6 && solve.delta.translation.length() < 1e-5 {
                break;
            }
        }
    }
    // the motion of the scanned surface, not of the far camera origins
    let centroid = samples_centroid(frames.iter().map(|(_, f, pose)| (f, *pose)));
    let shift = c.transform_point3(centroid).distance(centroid);
    let deg = rotation_angle(&c).to_degrees();
    if deg > PASS_MAX_DEG || shift > PASS_MAX_MM {
        return None;
    }
    let cost = |c: &DAffine3| {
        frames
            .iter()
            .zip(&views)
            .map(|((_, f, pose), v)| v.fit(f, &(*c * *pose), FIT_GATE_MM).cost)
            .sum::<f64>()
    };
    let (before, after) = (cost(&DAffine3::IDENTITY), cost(&c));
    (after < before * (1.0 - MIN_GAIN)).then_some((c, deg, shift))
}

/// Registers a stop that disagrees with `volume` once more on its own,
/// from a little farther away than the refinement ([`RESCUE_GATES_MM`]):
/// the object slipped a little within the pass. The new pose when it stays
/// within [`RESCUE_MAX_DEG`] / [`RESCUE_MAX_MM`] of `pose` and then agrees
/// with `volume` (`check`: the samples of the agreement).
fn rescue_stop(
    volume: &TsdfVolume,
    p: &Prepared,
    check: &FrameSample,
    pose: &Affine3A,
) -> Option<Affine3A> {
    let base = TrackParams {
        max_step_deg: RESCUE_MAX_DEG as f32,
        max_step_mm: RESCUE_MAX_MM as f32,
        ..refine_params(10_000)
    };
    let frame = samples(p, base.target_points);
    let mut current = *pose;
    for (k, gates) in RESCUE_GATES_MM.iter().enumerate() {
        let params = TrackParams {
            gates_mm: *gates,
            ..base
        };
        let view = ModelView::new(volume, &p.intr, &current, &params);
        let r = track_in_view(&view, &frame, &current, &params);
        // the coarse stage only brings the pose near: its residual is
        // measured at a wide gate
        let usable = r.pose.is_finite() && (r.ok || k + 1 < RESCUE_GATES_MM.len());
        if !usable {
            return None;
        }
        current = r.pose;
    }
    let c = to_d(pose).inverse() * to_d(&current);
    let centroid = samples_centroid(std::iter::once((&frame, to_d(pose))));
    let moved = to_d(&current)
        .transform_point3(to_d(pose).inverse().transform_point3(centroid))
        .distance(centroid);
    if rotation_angle(&c).to_degrees() > RESCUE_MAX_DEG || moved > RESCUE_MAX_MM {
        return None;
    }
    (agreement(volume, p, check, &current).verdict() == Verdict::Agrees).then_some(current)
}

/// How a stop agrees with a model ([`agreement`]).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct Agreement {
    /// Samples considered.
    samples: usize,
    /// Samples judged: the model has a surface facing like the stop's at
    /// their pixel, and they are not [`AGREE_FAR_MM`] in front of it.
    judged: usize,
    /// Of those, the samples within [`AGREE_TOL_MM`] of that surface...
    close: usize,
    /// ... and the samples more than [`AGREE_FAR_MM`] behind it.
    behind: usize,
    /// Median distance of the judged samples to the surface (mm).
    median_mm: f64,
}

/// What an [`Agreement`] says about a stop.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Verdict {
    /// It lies on the model.
    Agrees,
    /// It sees the same surfaces elsewhere: the object moved, or its pose
    /// is wrong.
    Disagrees,
    /// Too little in common with the model to tell.
    Unknown,
}

impl Agreement {
    fn verdict(&self) -> Verdict {
        let enough = self.judged >= AGREE_MIN_OVERLAP
            && self.judged as f64 >= AGREE_MIN_OVERLAP_FRACTION * self.samples as f64;
        if !enough {
            return Verdict::Unknown;
        }
        let n = self.judged as f64;
        let agrees = self.close as f64 >= AGREE_MIN_CLOSE * n
            && self.behind as f64 <= AGREE_MAX_BEHIND * n
            && self.median_mm <= AGREE_MAX_MEDIAN_MM;
        if agrees {
            Verdict::Agrees
        } else {
            Verdict::Disagrees
        }
    }
}

/// How the stop `p` placed by `pose` agrees with `volume`: the model is ray
/// cast from the stop's own camera (at most [`AGREE_VIEW_WIDTH`] pixels
/// wide) and every sample of `frame` is compared with the model surface at
/// its pixel, where that surface faces like the sample (normals within
/// [`AGREE_NORMAL_DEG`]): its signed distance to that plane, positive
/// toward the camera. Samples in front see what the model lacks; samples
/// behind see through a surface the model has. A view of an object that
/// moved shows both, whatever part of it still fits (a part symmetric about
/// the turntable axis).
fn agreement(volume: &TsdfVolume, p: &Prepared, frame: &FrameSample, pose: &Affine3A) -> Agreement {
    let k = p.intr.downscaled(p.intr.width.div_ceil(AGREE_VIEW_WIDTH));
    let (points, normals) = volume.raycast(&k, pose);
    let cos = AGREE_NORMAL_DEG.to_radians().cos();
    let mut dist: Vec<f64> = frame
        .points
        .par_iter()
        .zip(frame.normals.par_iter())
        .with_min_len(512)
        .filter_map(|(x, nx)| {
            let (u, v) = k.project(*x)?;
            let (u, v) = (u.round(), v.round());
            if u < 0.0 || v < 0.0 || u >= k.width as f32 || v >= k.height as f32 {
                return None;
            }
            let i = v as usize * k.width as usize + u as usize;
            let (q, n) = (points[i], normals[i]);
            if !q.is_finite() || !n.is_finite() {
                return None;
            }
            let (pw, nw) = (pose.transform_point3(*x), pose.transform_vector3(*nx));
            let d = f64::from((pw - q).dot(n));
            (nw.dot(n) >= cos && d <= AGREE_FAR_MM).then_some(d)
        })
        .collect();
    let close = dist.iter().filter(|d| d.abs() <= AGREE_TOL_MM).count();
    let behind = dist.iter().filter(|d| **d < -AGREE_FAR_MM).count();
    for d in &mut dist {
        *d = d.abs();
    }
    let median_mm = if dist.is_empty() {
        f64::NAN
    } else {
        let m = dist.len() / 2;
        *dist.select_nth_unstable_by(m, f64::total_cmp).1
    };
    Agreement {
        samples: frame.points.len(),
        judged: dist.len(),
        close,
        behind,
        median_mm,
    }
}

/// Name of the index file of a stops directory.
pub const STOPS_FILE: &str = "stops.json";
/// Format id of [`STOPS_FILE`].
pub const STOPS_FORMAT: &str = "dentalcraft-scan-stops";

/// One stop in [`STOPS_FILE`].
#[derive(Serialize, Deserialize)]
struct StopEntry {
    /// The image file: `crop[2] * crop[3]` little-endian `f32` depths (mm,
    /// `NaN` none), row-major.
    file: String,
    /// The cropped image in the full one: `[x0, y0, width, height]`.
    crop: [u32; 4],
    /// Intrinsics of the full image.
    intrinsics: CameraIntrinsics,
    rotate_deg: f32,
    tilt_deg: f32,
    /// Camera-to-fusion pose, column-major 3×4 (`Affine3A::to_cols_array`).
    pose: [f32; 12],
    flagged: bool,
    valid_px: usize,
}

/// [`STOPS_FILE`].
#[derive(Serialize, Deserialize)]
struct StopsIndex {
    format: String,
    version: u32,
    /// The scan world frame from the fusion frame, column-major 3×4.
    world_from_fusion: [f32; 12],
    world_angles_deg: Option<[f32; 2]>,
    calibration: Option<RigCalibration>,
    /// `[radius, height]` of the region of interest (mm), when in use.
    roi_mm: Option<[f32; 2]>,
    stops: Vec<StopEntry>,
}

/// Writes `stops` into the directory `dir` (created when missing):
/// [`STOPS_FILE`] (format [`STOPS_FORMAT`], version 1: the world frame,
/// its turntable angles, the rig calibration, the region of interest and
/// per stop its file, crop, intrinsics, angles, pose and flag) and one
/// `stopNNN.f32` per stop with the depths of its crop. Returns the number
/// of stops written.
///
/// # Errors
/// [`ScanError::NoStops`] without stops; [`ScanError::Device`] when a
/// file cannot be written.
pub fn write_stops(dir: &Path, stops: &KeptStops) -> Result<usize> {
    if stops.stops.is_empty() {
        return Err(ScanError::NoStops);
    }
    let io = |e: std::io::Error| ScanError::Device(format!("{}: {e}", dir.display()));
    std::fs::create_dir_all(dir).map_err(io)?;
    let mut entries = Vec::with_capacity(stops.stops.len());
    for (k, s) in stops.stops.iter().enumerate() {
        let file = format!("stop{k:03}.f32");
        let (depth, crop_intr, (x0, y0)) = s.crop().unwrap_or_else(|| {
            (
                Vec::new(),
                CameraIntrinsics {
                    width: 0,
                    height: 0,
                    ..s.intrinsics
                },
                (0, 0),
            )
        });
        let bytes: Vec<u8> = depth.iter().flat_map(|d| d.to_le_bytes()).collect();
        std::fs::write(dir.join(&file), bytes).map_err(io)?;
        entries.push(StopEntry {
            file,
            crop: [x0, y0, crop_intr.width, crop_intr.height],
            intrinsics: s.intrinsics,
            rotate_deg: s.angles.0,
            tilt_deg: s.angles.1,
            pose: s.pose.to_cols_array(),
            flagged: s.flagged,
            valid_px: s.pixels.len(),
        });
    }
    let index = StopsIndex {
        format: STOPS_FORMAT.into(),
        version: 1,
        world_from_fusion: stops.world.to_cols_array(),
        world_angles_deg: stops.world_angles.map(|(r, t)| [r, t]),
        calibration: stops.calibration.clone(),
        roi_mm: stops.roi.map(|(r, h)| [r, h]),
        stops: entries,
    };
    let json = serde_json::to_vec_pretty(&index)
        .map_err(|e| ScanError::Device(format!("stops index: {e}")))?;
    std::fs::write(dir.join(STOPS_FILE), json).map_err(io)?;
    Ok(stops.stops.len())
}

/// Reads a directory written by [`write_stops`].
///
/// # Errors
/// [`ScanError::InvalidParams`] for a missing or malformed directory.
pub fn read_stops(dir: &Path) -> Result<KeptStops> {
    let bad = |what: String| ScanError::InvalidParams(format!("{}: {what}", dir.display()));
    let raw = std::fs::read(dir.join(STOPS_FILE)).map_err(|e| bad(e.to_string()))?;
    let index: StopsIndex = serde_json::from_slice(&raw).map_err(|e| bad(e.to_string()))?;
    if index.format != STOPS_FORMAT || index.version != 1 {
        return Err(bad(format!(
            "format {} version {} not supported",
            index.format, index.version
        )));
    }
    let mut stops = Vec::with_capacity(index.stops.len());
    for e in index.stops {
        let bytes = std::fs::read(dir.join(&e.file)).map_err(|err| bad(err.to_string()))?;
        let [x0, y0, cw, ch] = e.crop;
        let k = e.intrinsics;
        if bytes.len() != (cw * ch) as usize * 4 || x0 + cw > k.width || y0 + ch > k.height {
            return Err(bad(format!("{} does not match its crop", e.file)));
        }
        let pixels: Vec<(u32, f32)> = bytes
            .as_chunks::<4>()
            .0
            .iter()
            .enumerate()
            .filter_map(|(j, b)| {
                let d = f32::from_le_bytes(*b);
                let (x, y) = (j as u32 % cw.max(1), j as u32 / cw.max(1));
                (d.is_finite() && d > 0.0).then_some(((y0 + y) * k.width + x0 + x, d))
            })
            .collect();
        stops.push(KeptStop {
            pixels: Arc::new(pixels),
            intrinsics: k,
            angles: (e.rotate_deg, e.tilt_deg),
            pose: Affine3A::from_cols_array(&e.pose),
            flagged: e.flagged,
        });
    }
    Ok(KeptStops {
        stops,
        world: Affine3A::from_cols_array(&index.world_from_fusion),
        world_angles: index.world_angles_deg.map(|[r, t]| (r, t)),
        calibration: index.calibration,
        roi: index.roi_mm.map(|[r, h]| (r, h)),
    })
}

#[cfg(test)]
mod tests;
