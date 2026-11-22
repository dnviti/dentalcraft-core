//! The print pipeline: parts in, plates sliced to GOO files out (print plan
//! §4–§5). Owner: PlateJob.
//!
//! [`prepare`] takes each part through, in parallel:
//! 1. its fitting flags: the part's own, else
//!    [`crate::orient::fitting_faces`];
//! 2. orientation ([`crate::orient::orient_with`]), then
//!    [`crate::orient::oriented_mesh`]: rotated, its lowest point at the
//!    lift, its bounding box centred on the origin in XY;
//! 3. supports ([`crate::support::supports`]), on the outer side only;
//! 4. the two hard checks of [`crate::support`]: no contact on the fitting
//!    side ([`crate::support::check_no_fitting_contact`]) and no
//!    unsupported island ([`crate::support::check_islands`]);
//! 5. its footprint (the convex hull of part and supports seen from above)
//!    and height, which must fit under the build height.
//!
//! Each failure is an error naming the part, the first in input order.
//!
//! Then [`crate::layout::nest`] places the footprints on plates, and each
//! part is turned and moved to its place. When the printer's machine
//! parameters are known ([`crate::machine`]), each plate's resin (the
//! volume of its meshes) plus what must stay in the vat has to fit the vat
//! ([`JobError::VatOverflow`]).
//!
//! [`write_plate`] slices one plate straight into the GOO writer (no layer
//! bitmap ever exists). The header comes first in the file but carries the
//! lit-pixel volume and the weight, known only once the last layer is
//! sliced: the encoded file is therefore assembled in memory (its own size,
//! some tens of MB) and written out at the end with the final header. The
//! vat check runs again on the sliced volume before anything is written.
//! Its time estimate takes the tilt release times, the first layers' rest
//! and the preheat target from the machine parameters
//! ([`estimate_time_s`], [`ASSUMED_RELEASE`]).
//! [`write_plate_stl`] writes the same plate (parts and supports) as one
//! STL, for the comparison with Elegoo SatelLite (plan §9.3).

use std::io::Write;
use std::iter;
use std::time::{SystemTime, UNIX_EPOCH};

use dc_mesh::TriMesh;
use dc_mesh::io::MeshFormat;
use glam::{DVec2, Vec3};
use rayon::prelude::*;
use serde::{Deserialize, Serialize};

use crate::PrinterProfile;
use crate::goo::{self, GooError, GooWriter, Header, JobSettings};
use crate::layout::{self, LayoutError, LayoutParams, Placement};
use crate::machine::{LightSource, MachineParams, ReleaseMode};
use crate::orient::{self, OrientParams};
use crate::part::{Footprint, Orientation, PlacedPart, PrintPart, translate_mesh};
use crate::slice;
use crate::support::{self, SupportError, SupportParams};

/// The tilt release mode the time estimates assume: slow (set S1 of the
/// machine parameters). The first N layers run alike in both modes (I212 A
/// = C), slow peels the thin aligner walls more gently, and it bounds the
/// time from above; fast (set S2, chosen on the touchscreen) saves I212 B −
/// D per layer after the first N (1.98 s with DENTAL 45).
pub const ASSUMED_RELEASE: ReleaseMode = ReleaseMode::Slow;

/// Vat temperature before the preheat, °C \[INFERENCE\]: a room.
pub const AMBIENT_C: f64 = 20.0;

/// Vat heating rate, °C/s \[INFERENCE\]: the printer took ≈ 3 min to warm
/// its vat from cold to its old 30 °C target \[S14\], about 10 °C in 180 s.
/// To be learnt per printer from finished jobs.
pub const HEATING_C_PER_S: f64 = 10.0 / 180.0;

/// Everything a print needs besides the parts.
#[derive(Debug, Clone, PartialEq)]
pub struct PrintJob {
    /// The printer.
    pub profile: PrinterProfile,
    /// The resin values entered by the user: layer height, exposures, bottom
    /// layers, rest before exposure, PWM, resin name. Volume, weight and
    /// time are filled in per plate.
    pub job: JobSettings,
    /// Resin density, g/ml (weight = volume × density).
    pub density_g_ml: f32,
    /// Orientation parameters. Their `layer_um` is not used: the
    /// orientation always takes the job's layer height (`job.layer_um`).
    pub orient: OrientParams,
    /// Support parameters. Their `layer_um` is not used: the supports
    /// always take the job's layer height (`job.layer_um`).
    pub support: SupportParams,
    /// Nesting parameters.
    pub layout: LayoutParams,
}

impl PrintJob {
    /// Checks the values a print cannot do without, before any work.
    ///
    /// # Errors
    /// [`JobError::InvalidSettings`] naming the first bad value;
    /// [`JobError::Layout`] when the gap or the margin is unusable.
    pub fn validate(&self) -> Result<(), JobError> {
        let p = &self.profile;
        let j = &self.job;
        let bad = |m: String| Err(JobError::InvalidSettings(m));
        if !p.supports_layer_um(j.layer_um) {
            return bad(format!(
                "layer_um {} is outside {}–{} µm for {}",
                j.layer_um, p.layer_min_um, p.layer_max_um, p.id
            ));
        }
        for (name, v) in [
            ("exposure_s", j.exposure_s),
            ("bottom_exposure_s", j.bottom_exposure_s),
            ("density_g_ml", self.density_g_ml),
        ] {
            if !(v.is_finite() && v > 0.0) {
                return bad(format!("{name} must be a number above 0, not {v}"));
            }
        }
        if !(j.rest_before_exposure_s.is_finite() && j.rest_before_exposure_s >= 0.0) {
            return bad(format!(
                "rest_s must be a number of 0 or more, not {}",
                j.rest_before_exposure_s
            ));
        }
        if !(1..=255).contains(&j.pwm) {
            return bad(format!("pwm must be 1–255, not {}", j.pwm));
        }
        if j.resin_name.is_empty() || j.resin_name.len() >= 32 || !j.resin_name.is_ascii() {
            return bad(format!(
                "resin_name must be 1–31 ASCII characters, not {:?}",
                j.resin_name
            ));
        }
        layout::check_params(p, &self.layout)?;
        if let Some(m) = &p.machine {
            if m.light_source == LightSource::MachineOnly {
                return bad(format!(
                    "the light source of {} follows its machine parameters only (I3 C = 0): the \
                     exposures of the file would be ignored",
                    m.name
                ));
            }
            if f64::from(p.build_mm[2]) > m.z_travel_mm {
                return bad(format!(
                    "the build height {} mm of {} exceeds its Z travel {} mm",
                    p.build_mm[2], p.id, m.z_travel_mm
                ));
            }
        }
        Ok(())
    }
}

/// Errors of the pipeline, with stable codes ([`JobError::code`]).
#[derive(Debug, thiserror::Error)]
pub enum JobError {
    /// No parts were given.
    #[error("print-no-parts: nothing to print")]
    NoParts,
    /// A part cannot be printed as given (empty, broken or non-finite mesh,
    /// fitting flags of the wrong length).
    #[error("print-invalid-part: {part}: {reason}")]
    InvalidPart {
        /// Part id.
        part: String,
        /// What is wrong.
        reason: String,
    },
    /// A print setting is missing or out of range.
    #[error("print-invalid-settings: {0}")]
    InvalidSettings(String),
    /// A hard check of the supports failed for a part: a contact on the
    /// fitting side or an unsupported island (the code is the check's).
    #[error("{source} (part {part})")]
    Support {
        /// Part id.
        part: String,
        /// The failed check.
        source: SupportError,
    },
    /// Nesting failed.
    #[error(transparent)]
    Layout(#[from] LayoutError),
    /// Slicing or writing the GOO file failed.
    #[error(transparent)]
    Goo(#[from] GooError),
    /// Writing the output failed.
    #[error("print-io: {0}")]
    Io(#[from] std::io::Error),
    /// Writing the STL failed.
    #[error("print-mesh: {0}")]
    Mesh(#[from] dc_mesh::MeshError),
    /// Cancelled by the caller.
    #[error("print-cancelled")]
    Cancelled,
    /// A plate needs more resin than the vat holds: its volume plus the
    /// resin that must stay in the vat (see
    /// [`MachineParams::vat_reserve_ml`]).
    #[error(
        "print-vat-overflow: plate {plate} needs {volume_ml:.1} ml plus {reserve_ml:.1} ml left in the vat, the vat holds {vat_ml:.0} ml"
    )]
    VatOverflow {
        /// Plate index (0-based).
        plate: usize,
        /// Resin the plate cures, ml.
        volume_ml: f64,
        /// Resin that must stay in the vat, ml.
        reserve_ml: f64,
        /// Vat maximum volume, ml.
        vat_ml: f64,
    },
}

impl JobError {
    /// Stable identifier, the same in every language (kebab-case).
    pub fn code(&self) -> &'static str {
        match self {
            Self::NoParts => "print-no-parts",
            Self::InvalidPart { .. } => "print-invalid-part",
            Self::InvalidSettings(_) => "print-invalid-settings",
            Self::Support { source, .. } => source.code(),
            Self::Layout(e) => e.code(),
            Self::Goo(_) => "print-file",
            Self::Io(_) => "print-io",
            Self::Mesh(_) => "print-mesh",
            Self::Cancelled => "print-cancelled",
            Self::VatOverflow { .. } => "print-vat-overflow",
        }
    }
}

/// How one part was prepared.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PartPlan {
    /// Part id.
    pub id: String,
    /// Plate index (0-based).
    pub plate: usize,
    /// The orientation chosen for it.
    pub orientation: Orientation,
    /// Its supports' contacts on the part.
    pub contacts: usize,
    /// Nesting turn about +Z, quarter turns counter-clockwise.
    pub quarter_turns: u8,
    /// Nesting XY translation, mm.
    pub offset_mm: [f32; 2],
    /// Height with its supports, mm.
    pub height_mm: f32,
}

/// One plate ready to slice.
#[derive(Debug, Clone)]
pub struct PreparedPlate {
    /// Its index among the plates (0-based).
    pub index: usize,
    /// Its parts, plate space, in input order.
    pub parts: Vec<PlacedPart>,
    /// Height of the tallest part with its supports, mm.
    pub height_mm: f32,
    /// Resin the plate needs before slicing, ml: the volumes of its parts
    /// and struts, and of its rafts' union (a little more than the slicing
    /// gives, where struts sink into the raft and the part).
    pub resin_estimate_ml: f64,
}

/// The result of [`prepare`].
#[derive(Debug, Clone)]
pub struct PreparedPlates {
    /// The plates, as few as the nesting needs.
    pub plates: Vec<PreparedPlate>,
    /// One entry per input part, in input order.
    pub parts: Vec<PartPlan>,
}

/// What one written plate holds and needs.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PlateSummary {
    /// Ids of its parts.
    pub parts: Vec<String>,
    /// Layers in the file.
    pub layers: u32,
    /// Cured resin, supports included, ml (lit pixels × pixel area × layer
    /// height).
    pub volume_ml: f64,
    /// Resin weight, g.
    pub weight_g: f64,
    /// Estimated print time, s: the preheat, then every layer's exposure,
    /// rest and tilt release ([`estimate_time_s`]).
    pub est_time_s: u32,
    /// Height of the tallest part with its supports, mm.
    pub height_mm: f32,
    /// Vat temperature the printer holds while printing, °C (machine
    /// parameter I7 E), when known.
    pub tank_target_c: Option<f64>,
    /// Estimated preheat before the first layer, s ([`preheat_s`]).
    pub preheat_s: u32,
    /// The tilt release mode the estimate assumes.
    pub release: ReleaseMode,
    /// Resin to pour in the vat at least, ml: the plate's volume plus what
    /// must stay in the vat.
    pub resin_needed_ml: f64,
}

/// Orients, supports, checks and nests `parts`; see the module
/// documentation.
///
/// # Errors
/// The first failing part in input order ([`JobError::InvalidPart`],
/// [`JobError::Support`]), then nesting errors ([`JobError::Layout`]);
/// [`JobError::InvalidSettings`] and [`JobError::NoParts`] before any work.
pub fn prepare(parts: &[PrintPart], job: &PrintJob) -> Result<PreparedPlates, JobError> {
    prepare_cancellable(parts, job, &|| false)
}

/// [`prepare`] that stops with [`JobError::Cancelled`] once `cancel`
/// returns `true`. It is polled before each part starts: parts already
/// being oriented and supported finish first.
///
/// # Errors
/// As [`prepare`], and [`JobError::Cancelled`].
pub fn prepare_cancellable(
    parts: &[PrintPart],
    job: &PrintJob,
    cancel: &(dyn Fn() -> bool + Sync),
) -> Result<PreparedPlates, JobError> {
    job.validate()?;
    if parts.is_empty() {
        return Err(JobError::NoParts);
    }
    // in input order, so the error reported is always the first part's
    let supported: Vec<Result<Supported, JobError>> = parts
        .par_iter()
        .map(|p| support_part(p, job, cancel))
        .collect();
    let supported = supported.into_iter().collect::<Result<Vec<_>, _>>()?;
    let footprints: Vec<Footprint> = supported.iter().map(|s| s.footprint.clone()).collect();
    let plates = layout::nest(&footprints, &job.profile, &job.layout)?;

    let mut plans: Vec<Option<PartPlan>> = vec![None; supported.len()];
    let mut supported: Vec<Option<Supported>> = supported.into_iter().map(Some).collect();
    let mut prepared = Vec::with_capacity(plates.len());
    for (plate, layout) in plates.iter().enumerate() {
        let mut height_mm: f32 = 0.0;
        let mut volume_mm3 = 0.0;
        let mut placed = Vec::with_capacity(layout.parts.len());
        for pl in &layout.parts {
            // each part is on exactly one plate: moved there, never copied
            let s = supported[pl.index]
                .take()
                .expect("nest places every part once");
            let id = &parts[pl.index].id;
            height_mm = height_mm.max(s.footprint.height_mm);
            volume_mm3 += s.volume_mm3;
            plans[pl.index] = Some(PartPlan {
                id: id.clone(),
                plate,
                orientation: s.orientation,
                contacts: s.contacts,
                quarter_turns: pl.quarter_turns,
                offset_mm: pl.offset.to_array(),
                height_mm: s.footprint.height_mm,
            });
            placed.push(s.place(id, pl));
        }
        let resin_estimate_ml = volume_mm3 / 1000.0;
        check_vat(&job.profile, plate, resin_estimate_ml)?;
        prepared.push(PreparedPlate {
            index: plate,
            parts: placed,
            height_mm,
            resin_estimate_ml,
        });
    }
    Ok(PreparedPlates {
        plates: prepared,
        parts: plans
            .into_iter()
            .map(|p| p.expect("nest places every part once"))
            .collect(),
    })
}

/// A part oriented, supported and checked, before nesting.
struct Supported {
    mesh: TriMesh,
    supports: Vec<TriMesh>,
    orientation: Orientation,
    contacts: usize,
    footprint: Footprint,
    /// Volume of the part and its supports, mm³.
    volume_mm3: f64,
}

impl Supported {
    /// The part turned and moved to its place on the plate.
    fn place(self, id: &str, pl: &Placement) -> PlacedPart {
        let mut mesh = self.mesh;
        let mut supports = self.supports;
        for m in iter::once(&mut mesh).chain(supports.iter_mut()) {
            turn_mesh(m, pl.quarter_turns);
            translate_mesh(m, pl.offset.extend(0.0));
        }
        PlacedPart {
            id: id.to_owned(),
            mesh,
            supports,
            offset: pl.offset,
        }
    }
}

/// Why `mesh` is not a closed, consistently oriented solid with a positive
/// volume (the slicer's winding rule needs one), `None` when it is.
fn solid_defect(mesh: &TriMesh) -> Option<String> {
    let adj = mesh.edge_adjacency();
    let boundary = adj.boundary_edge_count();
    if boundary > 0 {
        return Some(format!(
            "not a closed solid: {boundary} open (boundary) edges"
        ));
    }
    let non_manifold = adj.non_manifold_edge_count();
    if non_manifold > 0 {
        return Some(format!(
            "not a closed solid: {non_manifold} edges shared by more than two triangles"
        ));
    }
    // Two triangles on one edge are oriented alike when they run it in
    // opposite directions.
    let flipped = adj
        .iter()
        .filter(|(e, faces)| {
            let runs_ab = |f: u32| {
                let t = mesh.tris[f as usize];
                (0..3).any(|k| t[k] == e.0 && t[(k + 1) % 3] == e.1)
            };
            faces.len() == 2 && runs_ab(faces[0]) == runs_ab(faces[1])
        })
        .count();
    if flipped > 0 {
        return Some(format!(
            "triangles not consistently oriented: {flipped} edges between flipped triangles"
        ));
    }
    let volume: f64 = (0..mesh.face_count())
        .map(|f| {
            let [a, b, c] = mesh.triangle(f).map(|p| p.as_dvec3());
            a.dot(b.cross(c)) / 6.0
        })
        .sum();
    if volume.is_nan() || volume <= 0.0 {
        return Some(format!(
            "volume {volume:.3} mm³ is not positive (triangles facing inward)"
        ));
    }
    None
}

/// Orients, supports and checks one part; `cancel` is polled before the
/// work starts.
fn support_part(
    part: &PrintPart,
    job: &PrintJob,
    cancel: &(dyn Fn() -> bool + Sync),
) -> Result<Supported, JobError> {
    if cancel() {
        return Err(JobError::Cancelled);
    }
    let invalid = |reason: String| JobError::InvalidPart {
        part: part.id.clone(),
        reason,
    };
    if part.mesh.is_empty() {
        return Err(invalid("the mesh has no triangles".into()));
    }
    part.mesh.check().map_err(|e| invalid(e.to_string()))?;
    if part.mesh.positions.iter().any(|p| !p.is_finite()) {
        return Err(invalid("the mesh has a non-finite vertex".into()));
    }
    if let Some(defect) = solid_defect(&part.mesh) {
        return Err(invalid(defect));
    }
    let fitting = match &part.fitting {
        Some(f) => f.clone(),
        None => orient::fitting_faces(&part.mesh),
    };
    if fitting.len() != part.mesh.face_count() {
        return Err(invalid(format!(
            "{} fitting flags for {} faces",
            fitting.len(),
            part.mesh.face_count()
        )));
    }
    // the job's layer height decides, whatever the parameters carry
    let layer_um = job.job.layer_um;
    let orient_params = OrientParams {
        layer_um,
        ..job.orient.clone()
    };
    let support_params = SupportParams {
        layer_um,
        ..job.support.clone()
    };
    let orientation = orient::orient_with(&part.mesh, &fitting, &orient_params);
    let mesh = orient::oriented_mesh(&part.mesh, &orientation);
    let set = support::supports(&mesh, &fitting, &support_params);
    let failed = |source| JobError::Support {
        part: part.id.clone(),
        source,
    };
    support::check_no_fitting_contact(&mesh, &fitting, &set).map_err(failed)?;
    support::check_islands(&mesh, &set, layer_um).map_err(failed)?;
    let contacts = set.struts.len();
    // the raft's pads and bands overlap: count their union once
    let volume_mm3 = mesh_volume_mm3(&mesh)
        + set.solids.iter().map(mesh_volume_mm3).sum::<f64>()
        + union_volume_mm3(&set.raft)?;
    let mut supports = set.solids;
    if !set.raft.is_empty() {
        supports.push(set.raft);
    }
    let footprint = footprint(&part.id, iter::once(&mesh).chain(&supports));
    Ok(Supported {
        mesh,
        supports,
        orientation,
        contacts,
        footprint,
        volume_mm3,
    })
}

/// Pixel pitch of [`union_volume_mm3`], mm.
const UNION_PITCH_MM: f64 = 0.1;
/// Layer height of [`union_volume_mm3`], µm.
const UNION_LAYER_UM: u32 = 50;

/// Volume of the union of the closed pieces of `mesh`, mm³, as the printer
/// cures it: sliced by the nonzero rule on a [`UNION_PITCH_MM`] grid in
/// layers of [`UNION_LAYER_UM`]. Summing the volumes of overlapping pieces
/// would count their overlaps twice.
fn union_volume_mm3(mesh: &TriMesh) -> Result<f64, JobError> {
    let Some(b) = mesh.aabb() else {
        return Ok(0.0);
    };
    // one empty pixel on every side
    let size = b.size().as_dvec3();
    let cells = |mm: f64| (mm / UNION_PITCH_MM).ceil() as u32 + 2;
    let (nx, ny) = (cells(size.x), cells(size.y));
    let pitch = UNION_PITCH_MM as f32;
    let mut shifted = mesh.clone();
    translate_mesh(
        &mut shifted,
        Vec3::new(pitch - b.min.x, pitch - b.min.y, 0.0),
    );
    let grid = PrinterProfile {
        id: "union".into(),
        machine_name: "union".into(),
        resolution_x: nx,
        resolution_y: ny,
        pixel_pitch_x_um: UNION_PITCH_MM * 1000.0,
        pixel_pitch_y_um: UNION_PITCH_MM * 1000.0,
        build_mm: [
            (f64::from(nx) * UNION_PITCH_MM) as f32,
            (f64::from(ny) * UNION_PITCH_MM) as f32,
            b.max.z.max(0.0) + 1.0,
        ],
        layer_min_um: UNION_LAYER_UM,
        layer_max_um: UNION_LAYER_UM,
        mirror_x: false,
        mirror_y: false,
        machine: None,
        validated: true,
    };
    Ok(slice::slice_plate(
        &[&shifted],
        &grid,
        UNION_LAYER_UM,
        &mut |_, _| Ok(()),
    )?)
}

/// Volume enclosed by a closed, outward-oriented mesh, mm³ (the divergence
/// theorem over its triangles).
fn mesh_volume_mm3(mesh: &TriMesh) -> f64 {
    mesh.tris
        .iter()
        .map(|t| {
            let [a, b, c] = t.map(|i| mesh.positions[i as usize].as_dvec3());
            a.dot(b.cross(c)) / 6.0
        })
        .sum()
}

/// Refuses a plate that needs more resin than the vat holds, when the
/// printer's machine parameters are known: its `volume_ml` plus what must
/// stay in the vat ([`MachineParams::vat_reserve_ml`]) against the vat's
/// maximum (I4 T).
fn check_vat(profile: &PrinterProfile, plate: usize, volume_ml: f64) -> Result<(), JobError> {
    let Some(m) = &profile.machine else {
        return Ok(());
    };
    let reserve_ml = m.vat_reserve_ml();
    if volume_ml + reserve_ml > m.vat_max_ml {
        return Err(JobError::VatOverflow {
            plate,
            volume_ml,
            reserve_ml,
            vat_ml: m.vat_max_ml,
        });
    }
    Ok(())
}

/// Turns a mesh `turns` × 90° counter-clockwise about +Z, exactly.
fn turn_mesh(mesh: &mut TriMesh, turns: u8) {
    let turn = |v: Vec3| layout::quarter_turn(v.truncate(), turns).extend(v.z);
    for p in &mut mesh.positions {
        *p = turn(*p);
    }
    if let Some(n) = &mut mesh.normals {
        for v in n {
            *v = turn(*v);
        }
    }
}

/// The convex hull of `meshes` seen from above, and their height.
fn footprint<'a>(id: &str, meshes: impl Iterator<Item = &'a TriMesh> + Clone) -> Footprint {
    let points = meshes
        .clone()
        .flat_map(|m| m.positions.iter().map(|p| p.truncate().as_dvec2()))
        .collect();
    let height_mm = meshes
        .flat_map(|m| m.positions.iter().map(|p| p.z))
        .fold(0.0, f32::max);
    Footprint {
        id: id.to_owned(),
        hull: layout::convex_hull(points)
            .into_iter()
            .map(DVec2::as_vec2)
            .collect(),
        height_mm,
    }
}

// ------------------------------------------------------------ plate output

/// Slices `plate` and writes it as a GOO V3 file to `out`, with its volume,
/// weight, estimated time and top-view previews. `progress(done, total)`
/// is called after every layer.
///
/// The file carries the resin's exposure settings per layer (exposure,
/// rest before exposure, PWM). The printer honours them when its light
/// source combines the file's settings with its own (machine parameter I3
/// C = 2, as in DENTAL 45); [`PrintJob::validate`] refuses a printer that
/// would ignore them (I3 C = 0).
///
/// # Errors
/// [`JobError::InvalidSettings`] for unusable settings, [`JobError::Goo`]
/// when slicing or encoding fails, [`JobError::VatOverflow`] when the
/// sliced plate needs more resin than the vat holds (nothing is written
/// then), [`JobError::Io`] when `out` fails.
pub fn write_plate(
    plate: &PreparedPlate,
    job: &PrintJob,
    out: impl Write,
    progress: &mut dyn FnMut(u32, u32),
) -> Result<PlateSummary, JobError> {
    write_plate_cancellable(plate, job, out, progress, &|| false)
}

/// [`write_plate`] that stops with [`JobError::Cancelled`] once `cancel`
/// returns `true` (polled before every layer); nothing is written to `out`
/// then.
///
/// # Errors
/// As [`write_plate`], and [`JobError::Cancelled`].
pub fn write_plate_cancellable(
    plate: &PreparedPlate,
    job: &PrintJob,
    out: impl Write,
    progress: &mut dyn FnMut(u32, u32),
    cancel: &dyn Fn() -> bool,
) -> Result<PlateSummary, JobError> {
    write_plate_with(plate, job, out, progress, cancel, &file_time_now())
}

/// [`write_plate_cancellable`] with the header's file time.
pub(crate) fn write_plate_with(
    plate: &PreparedPlate,
    job: &PrintJob,
    mut out: impl Write,
    progress: &mut dyn FnMut(u32, u32),
    cancel: &dyn Fn() -> bool,
    file_time: &str,
) -> Result<PlateSummary, JobError> {
    job.validate()?;
    if plate.parts.is_empty() {
        return Err(JobError::NoParts);
    }
    let meshes: Vec<&TriMesh> = plate
        .parts
        .iter()
        .flat_map(|p| iter::once(&p.mesh).chain(&p.supports))
        .collect();
    let layers = slice::layer_count(&meshes, job.job.layer_um);
    let machine = job.profile.machine.as_ref();
    let mut settings = job.job.clone();
    settings.print_time_s = estimate_time_s(&settings, machine, ASSUMED_RELEASE, layers);
    let mut header = Header::new(
        &job.profile,
        &settings,
        layers,
        env!("CARGO_PKG_VERSION"),
        file_time,
    )?;
    header.small_preview = preview(plate, &job.profile, goo::SMALL_PREVIEW);
    header.big_preview = preview(plate, &job.profile, goo::BIG_PREVIEW);

    let mut writer = GooWriter::new(Vec::new(), header.clone(), settings)?;
    let mut cancelled = false;
    let volume_mm3 = slice::slice_plate(
        &meshes,
        &job.profile,
        job.job.layer_um,
        &mut |layer, sliced| {
            if cancel() {
                cancelled = true;
                return Err(GooError::Field("cancelled"));
            }
            writer.write_layer(sliced.runs)?;
            progress(layer + 1, layers);
            Ok(())
        },
    );
    if cancelled {
        return Err(JobError::Cancelled);
    }
    let volume_mm3 = volume_mm3?;
    let volume_ml = volume_mm3 / 1000.0;
    check_vat(&job.profile, plate.index, volume_ml)?;
    let mut file = writer.finish()?;

    let weight_g = volume_ml * f64::from(job.density_g_ml);
    header.volume_mm3 = volume_mm3 as f32;
    header.weight_g = weight_g as f32;
    file[..goo::HEADER_SIZE].copy_from_slice(&header.to_bytes()?);
    out.write_all(&file)?;
    out.flush()?;
    Ok(PlateSummary {
        parts: plate.parts.iter().map(|p| p.id.clone()).collect(),
        layers,
        volume_ml,
        weight_g,
        est_time_s: header.print_time_s,
        height_mm: plate.height_mm,
        tank_target_c: machine.map(|m| m.tank_target_c),
        preheat_s: preheat_s(machine),
        release: ASSUMED_RELEASE,
        resin_needed_ml: volume_ml + machine.map_or(0.0, MachineParams::vat_reserve_ml),
    })
}

/// Writes `plate` (parts and supports, plate space) as one binary STL.
///
/// # Errors
/// [`JobError::Mesh`] when writing fails.
pub fn write_plate_stl(plate: &PreparedPlate, mut out: impl Write) -> Result<(), JobError> {
    let mut all = TriMesh::default();
    for m in plate
        .parts
        .iter()
        .flat_map(|p| iter::once(&p.mesh).chain(&p.supports))
    {
        let base = all.positions.len() as u32;
        all.positions.extend_from_slice(&m.positions);
        all.tris.extend(m.tris.iter().map(|t| t.map(|i| i + base)));
    }
    dc_mesh::io::write_to(&mut out, &all, MeshFormat::Stl)?;
    out.flush()?;
    Ok(())
}

/// Estimated print time of `layers` layers, s: the preheat ([`preheat_s`]),
/// then for every layer its exposure (bottom or normal), its rest before
/// exposure and the tilt release of `mode` from the machine parameters
/// (I212: 8.42 s per layer for the first 50 layers, then 4.41 s slow or
/// 2.43 s fast with DENTAL 45; the machine's own rest in the first layers
/// when I202 A sets one). Without machine parameters: exposures and rests
/// only.
pub fn estimate_time_s(
    settings: &JobSettings,
    machine: Option<&MachineParams>,
    mode: ReleaseMode,
    layers: u32,
) -> u32 {
    let file_rest = f64::from(settings.rest_before_exposure_s);
    let mut total = f64::from(preheat_s(machine));
    for i in 0..layers {
        let exposure = if i < settings.bottom_layers {
            settings.bottom_exposure_s
        } else {
            settings.exposure_s
        };
        let (rest, tilt) = match machine {
            Some(m) => (m.rest_s(mode, i, file_rest), m.tilt_s(mode, i)),
            None => (file_rest, 0.0),
        };
        total += f64::from(exposure) + rest + tilt;
    }
    total.round().clamp(0.0, f64::from(u32::MAX)) as u32
}

/// Estimated preheat before the first layer, s: from [`AMBIENT_C`] to the
/// preheat target (I7 F) at [`HEATING_C_PER_S`]; 0 without machine
/// parameters.
pub fn preheat_s(machine: Option<&MachineParams>) -> u32 {
    machine.map_or(0, |m| {
        ((m.preheat_target_c - AMBIENT_C).max(0.0) / HEATING_C_PER_S).round() as u32
    })
}

/// RGB565 of an 8-bit colour.
const fn rgb565(r: u8, g: u8, b: u8) -> u16 {
    ((r as u16 >> 3) << 11) | ((g as u16 >> 2) << 5) | (b as u16 >> 3)
}

const PREVIEW_PLATE: u16 = rgb565(48, 48, 52);
const PREVIEW_SUPPORT: u16 = rgb565(130, 130, 136);
const PREVIEW_PART: u16 = rgb565(120, 200, 255);

/// A `side` × `side` RGB565 top view of the plate: the plate letterboxed on
/// black, supports in grey, parts in blue (their projected triangles).
fn preview(plate: &PreparedPlate, profile: &PrinterProfile, side: usize) -> Vec<u16> {
    let mut img = vec![0u16; side * side];
    let (w, h) = (
        f64::from(profile.build_mm[0]),
        f64::from(profile.build_mm[1]),
    );
    if !(w > 0.0 && h > 0.0) {
        return img;
    }
    let s = side as f64;
    let scale = s / w.max(h);
    let (ox, oy) = ((s - w * scale) / 2.0, (s - h * scale) / 2.0);
    // Image row 0 is the file's row 0 and column 0 its column 0: the plate
    // goes through the profile's mirror flags as the layer images do
    // (`map_point_mm`, `map_pixel` extended to points).
    let to_px = |p: Vec3| {
        let (fx, fy) = profile.map_point_mm(f64::from(p.x), f64::from(p.y));
        DVec2::new(ox + fx * scale, oy + fy * scale)
    };
    let (x0, x1) = (
        ox.round() as usize,
        ((ox + w * scale).round() as usize).min(side),
    );
    let (y0, y1) = (
        oy.round() as usize,
        ((oy + h * scale).round() as usize).min(side),
    );
    for row in img[y0 * side..y1 * side].chunks_mut(side) {
        row[x0..x1].fill(PREVIEW_PLATE);
    }
    let mut draw = |mesh: &TriMesh, colour: u16| {
        for t in &mesh.tris {
            let [a, b, c] = t.map(|i| to_px(mesh.positions[i as usize]));
            fill_triangle(&mut img, side, [a, b, c], colour);
        }
    };
    for p in &plate.parts {
        for m in &p.supports {
            draw(m, PREVIEW_SUPPORT);
        }
    }
    for p in &plate.parts {
        draw(&p.mesh, PREVIEW_PART);
    }
    img
}

/// Sets every pixel whose centre lies in triangle `t` (either winding).
fn fill_triangle(img: &mut [u16], side: usize, t: [DVec2; 3], colour: u16) {
    let [a, b, c] = t;
    let area = (b - a).perp_dot(c - a);
    if area == 0.0 || !area.is_finite() {
        return;
    }
    let lo = a.min(b).min(c).max(DVec2::ZERO);
    let hi = a.max(b).max(c).min(DVec2::splat(side as f64));
    if !(lo.x < hi.x && lo.y < hi.y) {
        return;
    }
    let edge = |p: DVec2, q: DVec2, r: DVec2| (q - p).perp_dot(r - p) * area.signum();
    for y in (lo.y - 0.5).ceil().max(0.0) as usize..((hi.y - 0.5).floor() as usize + 1).min(side) {
        for x in
            (lo.x - 0.5).ceil().max(0.0) as usize..((hi.x - 0.5).floor() as usize + 1).min(side)
        {
            let p = DVec2::new(x as f64 + 0.5, y as f64 + 0.5);
            if edge(a, b, p) >= 0.0 && edge(b, c, p) >= 0.0 && edge(c, a, p) >= 0.0 {
                img[y * side + x] = colour;
            }
        }
    }
}

/// The header's file time, now, UTC.
pub(crate) fn file_time_now() -> String {
    let secs = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_or(0, |d| d.as_secs());
    format_utc(secs)
}

/// `YYYY-MM-DD HH:MM:SS` (UTC) of Unix seconds (proleptic Gregorian
/// calendar, days counted from 0000-03-01).
fn format_utc(secs: u64) -> String {
    let days = i64::try_from(secs / 86_400).unwrap_or(0);
    let rem = secs % 86_400;
    let z = days + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z.rem_euclid(146_097);
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let day = doy - (153 * mp + 2) / 5 + 1;
    let month = if mp < 10 { mp + 3 } else { mp - 9 };
    let year = yoe + era * 400 + i64::from(month <= 2);
    format!(
        "{year:04}-{month:02}-{day:02} {:02}:{:02}:{:02}",
        rem / 3600,
        rem % 3600 / 60,
        rem % 60
    )
}

#[cfg(test)]
pub(crate) mod tests;
