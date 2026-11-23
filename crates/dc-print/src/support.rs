//! Automatic supports, outer side only (print plan §4). Owner: OrientSupport.
//!
//! Placement, on the oriented part (plate space):
//! 1. **Seeds.** Every point where the solid starts: a vertex lower than
//!    all its neighbours (ties by vertex index) whose faces face down on
//!    average. Each one starts an island in its layer, so each gets a
//!    strut.
//! 2. **Overhangs.** Points on the faces that are not fitting and face down
//!    beyond the self-supporting angle (normal.z < −cos angle): face
//!    centres, with a finer grid on large faces. They are taken lowest
//!    first and kept only when no kept point lies within
//!    [`SupportParams::spacing_mm`], which gives a deterministic
//!    Poisson-disc set.
//! 3. **Contact.** For each point, a vertical ray from the plate finds the
//!    first surface above it. A contact is accepted only when:
//!    - that surface is the point itself (no part below);
//!    - it is a face that is not fitting, facing down;
//!    - it is at least [`PLACEMENT_CLEARANCE_MM`] from every fitting face;
//!    - the strut's body, checked with rings of rays around its axis,
//!      misses the part.
//!
//!    Otherwise XY offsets of 0.5 and 1 mm in 8 directions are tried.
//! 4. **Island rounds.** The island check runs at the job's layer height
//!    ([`SupportParams::layer_um`]) on the part with its struts and raft.
//!    An island that starts on the outer side gets a strut where it starts,
//!    or under its first slice; up to four rounds. An island that starts
//!    on the fitting side gets none: the orientation refuses such poses, and
//!    [`check_islands`] reports what is left.
//! 5. **Solids** (module `solid`). Each strut is a closed 8-sided tapered
//!    prism with a cone tip 0.2 mm into the part. The raft follows the
//!    feet instead of filling their hull: a chamfered pad around each foot
//!    ([`RAFT_PAD_MM`] beyond the flared foot), joined to its neighbours
//!    by chamfered bands [`RAFT_BAND_MM`] wide along the edges of the
//!    feet's minimum spanning tree, so on an arch it is a U.
//!
//! Hard checks, run before any file is written: [`check_no_fitting_contact`]
//! and [`check_islands`] (module `islands`).
//!
//! Contacts are resolved in parallel and collected in order, so the same
//! part gives the same supports on every platform.

pub(crate) mod islands;
mod solid;

use std::collections::HashMap;

use dc_mesh::TriMesh;
use glam::{Vec2, Vec3};
use rayon::prelude::*;

use crate::orient::raycast::Bvh;
use crate::orient::sin_cos;
use crate::part::{Strut, SupportSet};

/// Hard limit: no strut tip this close to a fitting face, mm.
pub const FITTING_CLEARANCE_MM: f32 = 0.3;
/// Placement keeps tips a little further from fitting faces than the
/// check requires, mm.
pub const PLACEMENT_CLEARANCE_MM: f32 = 0.35;
/// XY offsets tried when the straight contact is not free, mm.
const OFFSETS_MM: [f32; 2] = [0.5, 1.0];
/// How far in z an offset contact may land from the intended point, mm.
const OFFSET_DZ_MM: f32 = 1.5;
/// Tolerance for the straight contact landing on the intended point, mm.
const DIRECT_DZ_MM: f32 = 0.05;
/// Shortest strut: contact height above the raft top, mm.
const MIN_STRUT_MM: f32 = 0.5;
/// Raft pad around each strut foot, beyond the flared foot, on the top
/// face, mm.
pub const RAFT_PAD_MM: f32 = 1.5;
/// Width of the raft bands joining the pads, on the top face, mm; the
/// bottom is two raft thicknesses wider (45° chamfer).
pub const RAFT_BAND_MM: f32 = 4.0;
/// Rounds of struts added where the island check still finds an island.
const ISLAND_ROUNDS: usize = 4;
/// Points of an island's first slice tried as contacts, at most.
const ISLAND_PROBES: usize = 64;
/// A strut added for an island may stand this close to another one, mm.
const ISLAND_TIP_GAP_MM: f32 = 0.05;
/// Unit directions every 45°, for the offsets.
const DIRS: [Vec2; 8] = [
    Vec2::new(1.0, 0.0),
    Vec2::new(
        std::f32::consts::FRAC_1_SQRT_2,
        std::f32::consts::FRAC_1_SQRT_2,
    ),
    Vec2::new(0.0, 1.0),
    Vec2::new(
        -std::f32::consts::FRAC_1_SQRT_2,
        std::f32::consts::FRAC_1_SQRT_2,
    ),
    Vec2::new(-1.0, 0.0),
    Vec2::new(
        -std::f32::consts::FRAC_1_SQRT_2,
        -std::f32::consts::FRAC_1_SQRT_2,
    ),
    Vec2::new(0.0, -1.0),
    Vec2::new(
        std::f32::consts::FRAC_1_SQRT_2,
        -std::f32::consts::FRAC_1_SQRT_2,
    ),
];

/// Support parameters.
#[derive(Debug, Clone, PartialEq)]
pub struct SupportParams {
    /// Contact tip diameter, mm.
    pub tip_d_mm: f32,
    /// Shaft diameter, mm.
    pub shaft_d_mm: f32,
    /// Raft thickness, mm (0 = no raft).
    pub raft_mm: f32,
    /// Spacing of the overhang contacts, mm.
    pub spacing_mm: f32,
    /// Self-supporting angle, degrees. Outer faces whose normal points down
    /// within this angle of straight down get supports.
    pub overhang_deg: f32,
    /// Layer height of the print, µm: struts are added until no island is
    /// left on the outer side at this layer height (the job's).
    pub layer_um: u32,
}

impl Default for SupportParams {
    fn default() -> Self {
        Self {
            tip_d_mm: 0.35,
            shaft_d_mm: 1.0,
            raft_mm: 1.0,
            spacing_mm: 2.5,
            overhang_deg: 45.0,
            layer_um: 50,
        }
    }
}

/// A hard check failed. Stable codes for the `print.*` commands.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum SupportError {
    /// A strut tip lies on, or too near, the fitting side.
    #[error("print-fitting-contact: strut {strut} tip is {distance_mm:.3} mm from a fitting face")]
    FittingContact {
        /// Index of the strut in [`SupportSet::struts`].
        strut: usize,
        /// Distance from the tip to the nearest fitting face, mm.
        distance_mm: f32,
    },
    /// Regions of layers start in mid-air and no strut touches them; the
    /// lowest one is named.
    #[error(
        "print-island-unsupported: {count} regions start in mid-air without a strut, the lowest at layer {layer} at ({x_mm:.2}, {y_mm:.2}) mm, {area_mm2:.3} mm²"
    )]
    IslandUnsupported {
        /// Unsupported islands.
        count: usize,
        /// Layer index (0-based).
        layer: u32,
        /// X of the island's centre, plate space, mm.
        x_mm: f32,
        /// Y of the island's centre, plate space, mm.
        y_mm: f32,
        /// Island area, mm².
        area_mm2: f32,
    },
}

impl SupportError {
    /// Stable kebab-case error code.
    pub fn code(&self) -> &'static str {
        match self {
            Self::FittingContact { .. } => "print-fitting-contact",
            Self::IslandUnsupported { .. } => "print-island-unsupported",
        }
    }
}

/// Supports for an oriented part: plate space, lowest point at the lift,
/// as made by [`crate::orient::oriented_mesh`]. `fitting` flags the faces
/// (as in [`crate::part::PrintPart::fitting`]) that must never be touched.
/// The raft lies on the plate (z = 0 to [`SupportParams::raft_mm`]) and
/// the strut feet stand on its top.
///
/// After the seeds and the overhangs, the island check runs at
/// [`SupportParams::layer_um`]: each island that starts on the outer side
/// gets a strut where it starts, for up to [`ISLAND_ROUNDS`] rounds. An
/// island on the fitting side gets none: the orientation must avoid it.
pub fn supports(oriented: &TriMesh, fitting: &[bool], params: &SupportParams) -> SupportSet {
    if oriented.is_empty() {
        return SupportSet::default();
    }
    let is_fitting = |f: usize| fitting.get(f).copied().unwrap_or(false);
    let bvh = Bvh::new(oriented);
    let fit = Bvh::with_faces(
        oriented,
        (0..oriented.face_count())
            .filter(|&f| is_fitting(f))
            .map(|f| f as u32),
    );
    let spacing = params.spacing_mm.max(0.5);
    let down_z = -sin_cos(f64::from(params.overhang_deg).to_radians()).1 as f32;

    let minima = islands::material_minima(oriented);
    let mut seeds: Vec<Vec3> = minima
        .iter()
        .map(|&v| oriented.positions[v as usize])
        .collect();
    seeds.sort_by(|p, q| {
        p.z.total_cmp(&q.z)
            .then(p.x.total_cmp(&q.x))
            .then(p.y.total_cmp(&q.y))
    });
    let samples = overhang_samples(oriented, fitting, down_z, spacing);
    let points = poisson(&seeds, &samples, spacing);

    let placer = Placer {
        mesh: oriented,
        fitting,
        bvh: &bvh,
        fit: &fit,
        raft_top: params.raft_mm.max(0.0),
        tip_r: 0.5 * params.tip_d_mm,
        shaft_r: 0.5 * params.shaft_d_mm,
    };
    let contacts: Vec<Option<Vec3>> = points.par_iter().map(|p| placer.contact(*p)).collect();
    // Offsets of nearby points may land on one spot: keep the first.
    let mut tips: Vec<Vec3> = Vec::with_capacity(contacts.len());
    for c in contacts.into_iter().flatten() {
        add_tip(&mut tips, c, params.tip_d_mm);
    }
    hold_islands(&placer, &minima, params, &mut tips)
}

/// Adds the tip `c` unless a tip already lies within `gap`; `true` when
/// added.
fn add_tip(tips: &mut Vec<Vec3>, c: Vec3, gap: f32) -> bool {
    let fresh = tips.iter().all(|t| t.distance_squared(c) >= gap * gap);
    if fresh {
        tips.push(c);
    }
    fresh
}

/// The supports on `tips`, with a strut added where each island that the
/// island check still finds starts on the outer side, for up to
/// [`ISLAND_ROUNDS`] rounds. `minima` are the part's
/// [`islands::material_minima`]. An island that starts on a fitting face
/// gets no strut.
fn hold_islands(
    placer: &Placer<'_>,
    minima: &[u32],
    params: &SupportParams,
    tips: &mut Vec<Vec3>,
) -> SupportSet {
    let part = placer.mesh;
    let on_fitting = islands::vertex_touches(part, placer.fitting);
    let mut set = support_set(tips, params, placer);
    for _ in 0..ISLAND_ROUNDS {
        let mut meshes: Vec<&TriMesh> = vec![part, &set.raft];
        meshes.extend(set.solids.iter());
        let found = islands::islands(&meshes, params.layer_um);
        let new: Vec<Option<Vec3>> = found
            .par_iter()
            .map(|island| {
                let v = islands::island_bottom(part, minima, island, params.layer_um);
                if v.is_some_and(|v| on_fitting[v as usize]) {
                    return None;
                }
                placer.island_contact(island, v, params.layer_um)
            })
            .collect();
        // An island may need a strut right beside another one's.
        let mut grew = false;
        for c in new.into_iter().flatten() {
            grew |= add_tip(tips, c, ISLAND_TIP_GAP_MM);
        }
        if !grew {
            break;
        }
        set = support_set(tips, params, placer);
    }
    set
}

/// The struts on `tips`, their solids and the raft under them.
fn support_set(tips: &[Vec3], params: &SupportParams, placer: &Placer<'_>) -> SupportSet {
    let struts: Vec<Strut> = tips
        .iter()
        .map(|t| Strut {
            tip: *t,
            foot: Vec3::new(t.x, t.y, placer.raft_top),
            tip_d_mm: params.tip_d_mm,
            shaft_d_mm: params.shaft_d_mm,
        })
        .collect();
    let solids = struts.iter().map(solid::strut_mesh).collect();
    let feet: Vec<Vec2> = struts.iter().map(|s| s.foot.truncate()).collect();
    let pad = placer.shaft_r * solid::FOOT_FLARE + RAFT_PAD_MM;
    let raft = solid::raft_mesh(&feet, params.raft_mm, pad, 0.5 * RAFT_BAND_MM);
    SupportSet {
        struts,
        raft,
        solids,
    }
}

/// Hard check: no strut tip within [`FITTING_CLEARANCE_MM`] of a fitting
/// face.
pub fn check_no_fitting_contact(
    oriented: &TriMesh,
    fitting: &[bool],
    set: &SupportSet,
) -> Result<(), SupportError> {
    let faces = (0..oriented.face_count().min(fitting.len())).filter(|&f| fitting[f]);
    let fit = Bvh::with_faces(oriented, faces.map(|f| f as u32));
    if fit.is_empty() {
        return Ok(());
    }
    for (strut, s) in set.struts.iter().enumerate() {
        if let Some((distance_mm, _)) = fit.nearest(s.tip, FITTING_CLEARANCE_MM) {
            return Err(SupportError::FittingContact { strut, distance_mm });
        }
    }
    Ok(())
}

/// Hard check: every region of a layer that does not rest on the layer
/// below (part, struts and raft merged) is touched by a strut, so nothing
/// starts in mid-air. Coarse raster at 0.1 mm, layers of `layer_um`.
pub fn check_islands(
    oriented: &TriMesh,
    set: &SupportSet,
    layer_um: u32,
) -> Result<(), SupportError> {
    let mut meshes: Vec<&TriMesh> = Vec::with_capacity(set.solids.len() + 2);
    meshes.push(oriented);
    if !set.raft.is_empty() {
        meshes.push(&set.raft);
    }
    meshes.extend(set.solids.iter());
    let found = islands::islands(&meshes, layer_um);
    match found.first() {
        Some(i) => Err(SupportError::IslandUnsupported {
            count: found.len(),
            layer: i.layer,
            x_mm: i.centre.x,
            y_mm: i.centre.y,
            area_mm2: i.area_mm2,
        }),
        None => Ok(()),
    }
}

/// Candidate contact points on the faces that are not fitting and face down
/// beyond the self-supporting angle (unit normal z < `down_z`): face
/// centres, or a grid of about half the spacing on large faces. Sorted
/// lowest first.
fn overhang_samples(mesh: &TriMesh, fitting: &[bool], down_z: f32, spacing: f32) -> Vec<Vec3> {
    let half = 0.5 * spacing;
    let mut pts = Vec::new();
    for (f, _) in mesh.tris.iter().enumerate() {
        if fitting.get(f).copied().unwrap_or(false) {
            continue;
        }
        let [a, b, c] = mesh.triangle(f);
        let raw = (b - a).cross(c - a);
        let len = raw.length();
        if len == 0.0 || raw.z >= down_z * len {
            continue;
        }
        let longest = (b - a).length().max((c - b).length()).max((a - c).length());
        let m = ((longest / half).ceil() as usize).max(1);
        if m == 1 {
            pts.push((a + b + c) / 3.0);
            continue;
        }
        for i in 0..m {
            for j in 0..m - i {
                let (u, v) = (
                    (i as f32 + 1.0 / 3.0) / m as f32,
                    (j as f32 + 1.0 / 3.0) / m as f32,
                );
                pts.push(a + (b - a) * u + (c - a) * v);
            }
        }
    }
    pts.sort_by(|p, q| {
        p.z.total_cmp(&q.z)
            .then(p.x.total_cmp(&q.x))
            .then(p.y.total_cmp(&q.y))
    });
    pts
}

/// All seeds, then the samples (in order) farther than `spacing` from
/// every point kept so far.
fn poisson(seeds: &[Vec3], samples: &[Vec3], spacing: f32) -> Vec<Vec3> {
    let cell = |p: Vec3| {
        (
            (p.x / spacing).floor() as i32,
            (p.y / spacing).floor() as i32,
            (p.z / spacing).floor() as i32,
        )
    };
    let mut grid: HashMap<(i32, i32, i32), Vec<Vec3>> = HashMap::new();
    let mut out: Vec<Vec3> = seeds.to_vec();
    for &s in seeds {
        grid.entry(cell(s)).or_default().push(s);
    }
    let r2 = spacing * spacing;
    'samples: for &p in samples {
        let (cx, cy, cz) = cell(p);
        for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    if let Some(list) = grid.get(&(cx + dx, cy + dy, cz + dz))
                        && list.iter().any(|q| q.distance_squared(p) < r2)
                    {
                        continue 'samples;
                    }
                }
            }
        }
        grid.entry(cell(p)).or_default().push(p);
        out.push(p);
    }
    out
}

/// Finds free contacts for struts.
struct Placer<'a> {
    mesh: &'a TriMesh,
    fitting: &'a [bool],
    bvh: &'a Bvh,
    fit: &'a Bvh,
    raft_top: f32,
    tip_r: f32,
    shaft_r: f32,
}

impl Placer<'_> {
    /// The contact for the point `p`: straight below it, else at the first
    /// free XY offset (0.5 mm, then 1 mm; 8 directions each).
    fn contact(&self, p: Vec3) -> Option<Vec3> {
        let xy = p.truncate();
        self.contact_at(xy, p.z, DIRECT_DZ_MM).or_else(|| {
            OFFSETS_MM
                .iter()
                .flat_map(|r| DIRS.iter().map(move |d| *d * *r))
                .find_map(|off| self.contact_at(xy + off, p.z, OFFSET_DZ_MM))
        })
    }

    /// A contact that holds `island` from its first slice: its starting
    /// vertex `bottom` straight from below, else the bottom of its first
    /// slice under its pixels (nearest the centre first), else an offset
    /// around the starting vertex.
    fn island_contact(
        &self,
        island: &islands::Island,
        bottom: Option<u32>,
        layer_um: u32,
    ) -> Option<Vec3> {
        let h = layer_um.max(1) as f32 / 1000.0;
        let (below, plane) = (
            (island.layer as f32 - 0.5) * h,
            (island.layer as f32 + 0.5) * h,
        );
        let start = bottom.map(|v| self.mesh.positions[v as usize]);
        if let Some(c) = start.and_then(|p| self.contact_at(p.truncate(), p.z, DIRECT_DZ_MM)) {
            return Some(c);
        }
        // Pixel centres of the island's box, nearest the centre first.
        let pitch = islands::PITCH_MM as f32;
        let (nx, ny) = (
            ((island.hi.x - island.lo.x) / pitch).round().max(1.0) as u32,
            ((island.hi.y - island.lo.y) / pitch).round().max(1.0) as u32,
        );
        let mut probes: Vec<Vec2> = (0..nx * ny)
            .map(|k| island.lo + Vec2::new((k % nx) as f32 + 0.5, (k / nx) as f32 + 0.5) * pitch)
            .collect();
        probes.sort_by(|a, b| {
            let d = |p: &Vec2| p.distance_squared(island.centre);
            d(a).total_cmp(&d(b))
                .then(a.x.total_cmp(&b.x))
                .then(a.y.total_cmp(&b.y))
        });
        let first_slice = probes.iter().take(ISLAND_PROBES).find_map(|xy| {
            let hit = self
                .bvh
                .cast(xy.extend(below), Vec3::Z, plane - below + h)?;
            let z = below + hit.t;
            self.contact_at(*xy, z, DIRECT_DZ_MM)
        });
        first_slice.or_else(|| start.and_then(|p| self.contact(p)))
    }

    /// The first surface above the plate at `xy`, if it lies within `dz`
    /// of `z` and makes a free contact.
    fn contact_at(&self, xy: Vec2, z: f32, dz: f32) -> Option<Vec3> {
        let origin = xy.extend(-0.01);
        let hit = self.bvh.cast(origin, Vec3::Z, z + dz + 0.01)?;
        let c = origin + Vec3::Z * hit.t;
        let face = hit.face as usize;
        let free = (c.z - z).abs() <= dz
            && c.z >= self.raft_top + MIN_STRUT_MM
            && !self.fitting.get(face).copied().unwrap_or(false)
            && self.mesh.face_normal_raw(face).z < 0.0
            && self.fit.nearest(c, PLACEMENT_CLEARANCE_MM).is_none()
            && self.body_clear(c);
        free.then_some(c)
    }

    /// Whether the strut below the contact `c` misses the part: vertical
    /// rays down to the raft from rings at the cone's middle and at its
    /// base, with the foot's flare.
    fn body_clear(&self, c: Vec3) -> bool {
        let cone = solid::cone_len(c.z - self.raft_top);
        let rings = [
            (0.5 * (self.tip_r + self.shaft_r), c.z - 0.5 * cone),
            (self.shaft_r * solid::FOOT_FLARE, c.z - cone),
        ];
        rings.iter().all(|&(r, top)| {
            let top = top - 0.05;
            top <= self.raft_top
                || solid::RING.iter().all(|d| {
                    let o = (c.truncate() + *d * r).extend(top);
                    !self.bvh.occluded(o, Vec3::NEG_Z, top - self.raft_top)
                })
        })
    }
}

#[cfg(test)]
mod tests;
