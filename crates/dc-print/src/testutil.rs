//! Synthetic aligner shell for the print pipeline tests (test builds only).
//!
//! [`aligner_shell`] builds what the export writes for a directly printed
//! aligner: a closed, outward-oriented hollow shell with a 1 mm wall,
//! shaped as a U-shaped horseshoe of tooth caps and open at the gingival
//! side. It measures about 62 × 50 × 12 mm. The generator knows which side
//! every face is on ([`Side`]), so tests can score the fitting-side
//! detection and the support placement against the truth.
//!
//! Model space is in millimetres. The gingival rim lies in the plane
//! z = 0, the cavity opens toward −Z ([`SyntheticShell::opening`]) and the
//! occlusal side is up. The arch spine is a half ellipse (25 × 40 mm)
//! carrying 14 teeth, molars to central incisors on each side. Every tooth
//! is a bump of the trough, with interproximal dips between teeth. Premolars
//! and molars have a central fossa between a buccal and a lingual cusp.
//!
//! Construction:
//! - the inner (fitting) surface is a grid swept along the spine. Its
//!   cross-section is an inverted U: vertical walls, rounded corners and a
//!   fossa on top;
//! - both distal ends are closed by revolving the half profile;
//! - the outer surface is the inner one moved along its normals into the
//!   material by the wall thickness;
//! - a rim strip joins the two boundary loops.
//!
//! The curvature radii of the inner surface stay above the wall thickness,
//! so the offset never folds.

use std::collections::BTreeSet;
use std::f32::consts::{FRAC_PI_2, PI};

use dc_mesh::TriMesh;
use glam::{Vec2, Vec3};

use crate::part::PrintPart;

/// Side of the shell a face belongs to (the generator's truth).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Side {
    /// Intaglio: the inner surface that touches the teeth.
    Fitting,
    /// Outer surface (occlusal, buccal and lingual outside).
    Outer,
    /// The band joining both surfaces along the gingival trim line.
    Rim,
}

/// Generator settings.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct ShellSpec {
    /// Target triangle edge length, mm.
    pub step_mm: f32,
    /// Wall thickness, mm.
    pub wall_mm: f32,
}

impl Default for ShellSpec {
    fn default() -> Self {
        Self {
            step_mm: 0.8,
            wall_mm: 1.0,
        }
    }
}

/// A synthetic shell and its truth.
#[derive(Debug, Clone)]
pub(crate) struct SyntheticShell {
    /// Closed, outward-oriented mesh, model space (mm).
    pub mesh: TriMesh,
    /// Side of every face.
    pub side: Vec<Side>,
    /// Unit direction the cavity opens toward, model space.
    pub opening: Vec3,
}

impl SyntheticShell {
    /// `true` for the faces of the fitting side.
    pub fn fitting_truth(&self) -> Vec<bool> {
        self.side.iter().map(|s| *s == Side::Fitting).collect()
    }

    /// The shell as a print part, with the true fitting flags or without
    /// flags (to be computed by the pipeline).
    pub fn part(&self, id: &str, with_fitting: bool) -> PrintPart {
        PrintPart {
            id: id.to_owned(),
            mesh: self.mesh.clone(),
            fitting: with_fitting.then(|| self.fitting_truth()),
        }
    }

    /// The stage model the shell was built on (no clearance): the inner
    /// surface, facing out of the teeth, and a flat gingiva 0.05 mm under
    /// the rim reaching 2 mm out, so the model comes close to the outer
    /// surface along the trim line as a real gingiva does. Surfaces only,
    /// not a closed solid.
    pub fn model(&self) -> TriMesh {
        let pos = &self.mesh.positions;
        let n_in = pos.len() / 2;
        let mut model = TriMesh::new(pos[..n_in].to_vec(), Vec::new());
        let mut inner: BTreeSet<(u32, u32)> = BTreeSet::new();
        for (t, side) in self.mesh.tris.iter().zip(&self.side) {
            if *side == Side::Fitting {
                model.tris.push([t[0], t[2], t[1]]);
                inner.extend([(t[0], t[1]), (t[1], t[2]), (t[2], t[0])]);
            }
        }
        // Away from the teeth at a rim vertex: towards its outer twin.
        let out = |v: u32| {
            let d = pos[v as usize + n_in] - pos[v as usize];
            Vec3::new(d.x, d.y, 0.0).normalize_or_zero()
        };
        for &(a, b) in &inner {
            if inner.contains(&(b, a)) {
                continue;
            }
            let base = model.positions.len() as u32;
            for v in [a, b] {
                let p = pos[v as usize] - Vec3::Z * 0.05;
                model.positions.extend([p, p + out(v) * 2.0]);
            }
            let [a0, a1, b0, b1] = [base, base + 1, base + 2, base + 3];
            let up = (model.positions[a1 as usize] - model.positions[a0 as usize])
                .cross(model.positions[b0 as usize] - model.positions[a0 as usize])
                .z
                > 0.0;
            if up {
                model.tris.extend([[a0, a1, b0], [b0, a1, b1]]);
            } else {
                model.tris.extend([[a0, b0, a1], [b0, b1, a1]]);
            }
        }
        model
    }
}

/// The default test shell (about 20 k triangles).
pub(crate) fn default_shell() -> SyntheticShell {
    aligner_shell(&ShellSpec::default())
}

/// A shell with about `target` triangles (within a few per cent) and a
/// 1 mm wall.
pub(crate) fn aligner_shell_with_faces(target: usize) -> SyntheticShell {
    let target = target.max(1) as f32;
    let mut step = 1.0;
    let mut shell = aligner_shell(&ShellSpec {
        step_mm: step,
        ..ShellSpec::default()
    });
    // Faces grow as 1 / step² (surfaces) plus 1 / step (rim): two
    // corrections land within a few per cent.
    for _ in 0..2 {
        step *= (shell.mesh.face_count() as f32 / target).sqrt();
        shell = aligner_shell(&ShellSpec {
            step_mm: step,
            ..ShellSpec::default()
        });
    }
    shell
}

/// Teeth of one side, from the midline: mesio-distal width, inner cavity
/// height, inner half width (bucco-lingual) and fossa depth, mm.
const TEETH: [[f32; 4]; 7] = [
    [8.5, 10.5, 3.3, 0.0],  // central incisor
    [6.5, 10.0, 3.0, 0.0],  // lateral incisor
    [7.5, 11.5, 3.8, 0.0],  // canine
    [7.0, 9.0, 4.0, 0.5],   // first premolar
    [6.5, 8.5, 4.2, 0.55],  // second premolar
    [10.0, 8.0, 5.0, 0.75], // first molar
    [9.0, 7.5, 4.8, 0.7],   // second molar
];

/// Half axes of the spine ellipse (x, y) and its shift along −y, mm.
const SPINE_A: f32 = 25.0;
const SPINE_B: f32 = 40.0;
const SPINE_Y0: f32 = 20.0;

/// Interproximal dip: depth as a fraction of the cavity height, and width.
const DIP: f32 = 0.22;
const DIP_SIGMA_MM: f32 = 1.8;

/// Builds the shell described in the module docs.
pub(crate) fn aligner_shell(spec: &ShellSpec) -> SyntheticShell {
    let step = spec.step_mm.max(0.05);
    let spine = Spine::new();
    let len = spine.len();
    let teeth = Teeth::along(len);

    let ns = (len / step).ceil() as usize + 1;
    let sections: Vec<Section> = (0..ns)
        .map(|i| teeth.section(len * i as f32 / (ns - 1) as f32))
        .collect();
    let max_half = sections.iter().map(Section::half_len).fold(0.0, f32::max);
    let jc = ((max_half / step).ceil() as usize).max(4);
    let nu = 2 * jc + 1;
    let halves: Vec<Vec<Vec2>> = sections.iter().map(|s| s.sample(jc + 1)).collect();
    let frames: Vec<Frame> = (0..ns)
        .map(|i| spine.frame(len * i as f32 / (ns - 1) as f32))
        .collect();

    // Inner surface: the channel grid, vertex (i, j) at i * nu + j.
    let g = |i: usize, j: usize| (i * nu + j) as u32;
    let mut inner: Vec<Vec3> = Vec::with_capacity(ns * nu);
    for (frame, half) in frames.iter().zip(&halves) {
        for j in 0..nu {
            let p = if j <= jc {
                half[j]
            } else {
                let q = half[nu - 1 - j];
                Vec2::new(-q.x, q.y)
            };
            inner.push(frame.at(p));
        }
    }
    let mut normals: Vec<Vec3> = Vec::with_capacity(inner.len());
    for i in 0..ns {
        for j in 0..nu {
            let di = inner[g((i + 1).min(ns - 1), j) as usize]
                - inner[g(i.saturating_sub(1), j) as usize];
            let dj = inner[g(i, (j + 1).min(nu - 1)) as usize]
                - inner[g(i, j.saturating_sub(1)) as usize];
            // `di × dj` points into the cavity; the material is the other way.
            let mut n = -di.cross(dj).normalize();
            if j == 0 || j == nu - 1 {
                n.z = 0.0;
                n = n.normalize();
            }
            normals.push(n);
        }
    }
    let mut tris: Vec<[u32; 3]> = Vec::new();
    for i in 0..ns - 1 {
        for j in 0..nu - 1 {
            let (a, b, c, d) = (g(i, j), g(i + 1, j), g(i + 1, j + 1), g(i, j + 1));
            tris.push([a, b, c]);
            tris.push([a, c, d]);
        }
    }

    // Distal caps: the half profile revolved about the vertical axis at
    // each end of the spine, from the buccal side (k = 0) round the distal
    // end to the lingual side (k = ng).
    for start in [true, false] {
        let ie = if start { 0 } else { ns - 1 };
        let frame = &frames[ie];
        let distal = if start { -frame.tangent } else { frame.tangent };
        let half = &halves[ie];
        let ng = ((PI * sections[ie].w / step).ceil() as usize).max(4);
        // ring[k * jc + j], j < jc; the pole (j = jc) is g(ie, jc).
        let mut ring = vec![0u32; (ng + 1) * jc];
        for k in 0..=ng {
            for j in 0..jc {
                ring[k * jc + j] = if k == 0 {
                    g(ie, j)
                } else if k == ng {
                    g(ie, nu - 1 - j)
                } else {
                    let gamma = PI * k as f32 / ng as f32;
                    let radial = frame.normal * gamma.cos() + distal * gamma.sin();
                    let p = half[j];
                    let tan = half[j + 1] - half[j.saturating_sub(1)];
                    let n2 = Vec2::new(tan.y, -tan.x).normalize();
                    inner.push(frame.origin + radial * p.x + Vec3::Z * p.y);
                    normals.push(radial * n2.x + Vec3::Z * n2.y);
                    (inner.len() - 1) as u32
                };
            }
        }
        let pole = g(ie, jc);
        for k in 0..ng {
            for j in 0..jc {
                let a = ring[k * jc + j];
                let b = ring[(k + 1) * jc + j];
                if j + 1 == jc {
                    tris.push(if start { [a, pole, b] } else { [a, b, pole] });
                } else {
                    let c = ring[(k + 1) * jc + j + 1];
                    let d = ring[k * jc + j + 1];
                    if start {
                        tris.push([a, d, c]);
                        tris.push([a, c, b]);
                    } else {
                        tris.push([a, b, c]);
                        tris.push([a, c, d]);
                    }
                }
            }
        }
    }

    // Outer surface, rim, truth.
    let n_in = inner.len() as u32;
    let inner_tris = tris.len();
    let mut positions = inner.clone();
    positions.extend(
        inner
            .iter()
            .zip(&normals)
            .map(|(p, n)| *p + *n * spec.wall_mm),
    );
    let mut side = vec![Side::Fitting; inner_tris];
    for k in 0..inner_tris {
        let [a, b, c] = tris[k];
        tris.push([a + n_in, c + n_in, b + n_in]);
    }
    side.resize(tris.len(), Side::Outer);
    let directed: BTreeSet<(u32, u32)> = tris[..inner_tris]
        .iter()
        .flat_map(|t| [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])])
        .collect();
    for &(a, b) in &directed {
        if !directed.contains(&(b, a)) {
            tris.push([b, a, a + n_in]);
            tris.push([b, a + n_in, b + n_in]);
        }
    }
    side.resize(tris.len(), Side::Rim);

    SyntheticShell {
        mesh: TriMesh::new(positions, tris),
        side,
        opening: Vec3::NEG_Z,
    }
}

/// Point and unit tangent of the spine ellipse at parameter `theta`.
fn spine_point(theta: f32) -> Vec3 {
    Vec3::new(SPINE_A * theta.sin(), SPINE_B * theta.cos() - SPINE_Y0, 0.0)
}

/// The arch spine, parametrised by arc length from the left distal end.
struct Spine {
    theta: Vec<f32>,
    s: Vec<f32>,
}

/// Spine frame: point, unit tangent (toward the right distal end) and unit
/// buccal normal (out of the U).
struct Frame {
    origin: Vec3,
    tangent: Vec3,
    normal: Vec3,
}

impl Frame {
    /// The point at profile coordinates `p` = (buccal offset, height).
    fn at(&self, p: Vec2) -> Vec3 {
        self.origin + self.normal * p.x + Vec3::Z * p.y
    }
}

impl Spine {
    fn new() -> Self {
        const N: usize = 4096;
        let theta: Vec<f32> = (0..=N)
            .map(|k| -FRAC_PI_2 + PI * k as f32 / N as f32)
            .collect();
        let mut s = Vec::with_capacity(theta.len());
        let mut acc = 0.0f64;
        let mut prev = spine_point(theta[0]);
        for &t in &theta {
            let p = spine_point(t);
            acc += f64::from((p - prev).length());
            s.push(acc as f32);
            prev = p;
        }
        Self { theta, s }
    }

    fn len(&self) -> f32 {
        self.s[self.s.len() - 1]
    }

    fn frame(&self, s: f32) -> Frame {
        let k = self
            .s
            .partition_point(|&x| x <= s)
            .clamp(1, self.s.len() - 1);
        let (s0, s1) = (self.s[k - 1], self.s[k]);
        let f = if s1 > s0 {
            ((s - s0) / (s1 - s0)).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let theta = self.theta[k - 1] + (self.theta[k] - self.theta[k - 1]) * f;
        let tangent = Vec3::new(SPINE_A * theta.cos(), -SPINE_B * theta.sin(), 0.0).normalize();
        Frame {
            origin: spine_point(theta),
            tangent,
            normal: Vec3::Z.cross(tangent),
        }
    }
}

/// Tooth parameters laid along the spine, left distal to right distal.
struct Teeth {
    centres: Vec<f32>,
    height: Vec<f32>,
    half_w: Vec<f32>,
    fossa: Vec<f32>,
    gaps: Vec<f32>,
}

impl Teeth {
    fn along(len: f32) -> Self {
        let order: Vec<&[f32; 4]> = TEETH.iter().rev().chain(TEETH.iter()).collect();
        let scale = len / order.iter().map(|t| t[0]).sum::<f32>();
        let mut t = Self {
            centres: Vec::new(),
            height: Vec::new(),
            half_w: Vec::new(),
            fossa: Vec::new(),
            gaps: Vec::new(),
        };
        let mut x = 0.0;
        for (n, tooth) in order.iter().enumerate() {
            let w = tooth[0] * scale;
            t.centres.push(x + 0.5 * w);
            t.height.push(tooth[1]);
            t.half_w.push(tooth[2]);
            t.fossa.push(tooth[3]);
            x += w;
            if n + 1 < order.len() {
                t.gaps.push(x);
            }
        }
        t
    }

    fn section(&self, s: f32) -> Section {
        let mut h = blend(s, &self.centres, &self.height);
        for &gap in &self.gaps {
            let d = s - gap;
            h -= DIP
                * blend(gap, &self.centres, &self.height)
                * (-d * d / (2.0 * DIP_SIGMA_MM * DIP_SIGMA_MM)).exp();
        }
        Section {
            h,
            w: blend(s, &self.centres, &self.half_w),
            fossa: blend(s, &self.centres, &self.fossa),
        }
    }
}

/// Smooth (C1) interpolation of tooth values `ys` placed at `xs`.
fn blend(s: f32, xs: &[f32], ys: &[f32]) -> f32 {
    let last = xs.len() - 1;
    if s <= xs[0] {
        return ys[0];
    }
    if s >= xs[last] {
        return ys[last];
    }
    let k = xs.partition_point(|&x| x <= s) - 1;
    let f = (s - xs[k]) / (xs[k + 1] - xs[k]);
    ys[k] + (ys[k + 1] - ys[k]) * f * f * (3.0 - 2.0 * f)
}

/// Inner cross-section at one point of the spine.
struct Section {
    /// Cavity height, mm.
    h: f32,
    /// Cavity half width, mm.
    w: f32,
    /// Fossa depth, mm.
    fossa: f32,
}

impl Section {
    /// Fine polyline of the half profile, from the rim (w, 0) up the wall,
    /// round the corner and over the top to the middle (0, h − fossa).
    fn polyline(&self) -> Vec<Vec2> {
        const K: usize = 48;
        let r = 0.4 * self.w;
        let top = self.w - r;
        let mut pts = Vec::with_capacity(3 * K + 1);
        for k in 0..K {
            pts.push(Vec2::new(self.w, (self.h - r) * k as f32 / K as f32));
        }
        for k in 0..K {
            let a = FRAC_PI_2 * k as f32 / K as f32;
            pts.push(Vec2::new(top + r * a.cos(), self.h - r + r * a.sin()));
        }
        for k in 0..=K {
            let v = top * (1.0 - k as f32 / K as f32);
            let z = self.h - self.fossa * 0.5 * (1.0 + (PI * v / top).cos());
            pts.push(Vec2::new(v, z));
        }
        pts
    }

    fn half_len(&self) -> f32 {
        self.polyline()
            .windows(2)
            .map(|w| (w[1] - w[0]).length())
            .sum()
    }

    /// `m` points evenly spaced along the half profile: index 0 is the rim,
    /// `m - 1` the middle (exactly on the symmetry plane).
    fn sample(&self, m: usize) -> Vec<Vec2> {
        let pts = self.polyline();
        let mut cum = vec![0.0f32; pts.len()];
        for k in 1..pts.len() {
            cum[k] = cum[k - 1] + (pts[k] - pts[k - 1]).length();
        }
        let total = cum[cum.len() - 1];
        (0..m)
            .map(|j| {
                if j + 1 == m {
                    return pts[pts.len() - 1];
                }
                let target = total * j as f32 / (m - 1) as f32;
                let k = cum
                    .partition_point(|&c| c <= target)
                    .clamp(1, pts.len() - 1);
                let span = cum[k] - cum[k - 1];
                let f = if span > 0.0 {
                    (target - cum[k - 1]) / span
                } else {
                    0.0
                };
                pts[k - 1] + (pts[k] - pts[k - 1]) * f
            })
            .collect()
    }
}

/// A part with a pocket of the fitting side that hangs down: a slab
/// 12 × 12 × 1.5 mm (top face at z = 1.5, fitting) with a square pyramid
/// 4 × 4 mm hanging 8 mm under its bottom (fitting). Returns the closed,
/// outward mesh and its fitting flags. With the opening (up) tilted by up
/// to 45°, the tip of the pyramid is where the solid starts, in mid-air on
/// the fitting side; tilted far enough to the side (about 80°), the
/// pyramid hangs from the slab's lower edge and the solid starts on the
/// outer side.
pub(crate) fn spiked_slab() -> (TriMesh, Vec<bool>) {
    let mut m = TriMesh::default();
    let mut fitting = Vec::new();
    // a convex polygon as a fan, wound to face `outward`
    let mut poly = |pts: &[Vec3], outward: Vec3, fit: bool| {
        let ids: Vec<u32> = pts
            .iter()
            .map(|&p| match m.positions.iter().position(|&v| v == p) {
                Some(i) => i as u32,
                None => {
                    m.positions.push(p);
                    (m.positions.len() - 1) as u32
                }
            })
            .collect();
        let flip = (pts[1] - pts[0]).cross(pts[2] - pts[0]).dot(outward) < 0.0;
        for k in 1..ids.len() - 1 {
            let t = [ids[0], ids[k], ids[k + 1]];
            m.tris.push(if flip { [t[0], t[2], t[1]] } else { t });
            fitting.push(fit);
        }
    };
    let (o, i, top, tip) = (6.0f32, 2.0f32, 1.5f32, -8.0f32);
    let ring =
        |r: f32, z: f32| [(-r, -r), (r, -r), (r, r), (-r, r)].map(|(x, y)| Vec3::new(x, y, z));
    let (bottom_out, bottom_in, upper) = (ring(o, 0.0), ring(i, 0.0), ring(o, top));
    poly(&upper, Vec3::Z, true);
    for k in 0..4 {
        let n = (k + 1) % 4;
        let side = (bottom_out[k] + bottom_out[n]).with_z(0.0);
        poly(
            &[bottom_out[k], bottom_out[n], upper[n], upper[k]],
            side,
            false,
        );
        poly(
            &[bottom_out[k], bottom_out[n], bottom_in[n], bottom_in[k]],
            -Vec3::Z,
            false,
        );
        let face = (bottom_in[k] + bottom_in[n]).with_z(-1.0);
        poly(
            &[bottom_in[k], bottom_in[n], Vec3::new(0.0, 0.0, tip)],
            face,
            true,
        );
    }
    (m, fitting)
}

#[cfg(test)]
mod tests {
    use std::collections::BTreeMap;

    use super::*;

    /// Every directed edge used once and its reverse once: closed, manifold
    /// and consistently wound.
    fn assert_closed(mesh: &TriMesh) {
        let mut count: BTreeMap<(u32, u32), u32> = BTreeMap::new();
        for t in &mesh.tris {
            for e in [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
                *count.entry(e).or_default() += 1;
            }
        }
        for (&(a, b), &n) in &count {
            assert_eq!(n, 1, "directed edge {a}->{b} used {n} times");
            assert_eq!(count.get(&(b, a)), Some(&1), "edge {a}->{b} has no twin");
        }
    }

    fn signed_volume(mesh: &TriMesh) -> f64 {
        (0..mesh.face_count())
            .map(|f| {
                let [a, b, c] = mesh.triangle(f);
                f64::from(a.dot(b.cross(c))) / 6.0
            })
            .sum()
    }

    #[test]
    fn shell_is_closed_outward_and_has_three_sides() {
        let shell = default_shell();
        let mesh = &shell.mesh;
        mesh.check().unwrap();
        assert_closed(mesh);
        let min_area = (0..mesh.face_count())
            .map(|f| mesh.face_area(f))
            .fold(f32::MAX, f32::min);
        assert!(min_area > 1e-4, "degenerate face, area {min_area}");
        // Volume ≈ wall × (inner area): positive, a few thousand mm³.
        let volume = signed_volume(mesh);
        let fitting_area: f32 = (0..mesh.face_count())
            .filter(|&f| shell.side[f] == Side::Fitting)
            .map(|f| mesh.face_area(f))
            .sum();
        assert!(volume > 0.0, "inward-oriented, volume {volume}");
        let ratio = volume / f64::from(fitting_area);
        assert!((0.9..1.25).contains(&ratio), "volume / inner area {ratio}");
        assert_eq!(shell.side.len(), mesh.face_count());
        for s in [Side::Fitting, Side::Outer, Side::Rim] {
            assert!(shell.side.contains(&s), "{s:?} missing");
        }
        let fitting = shell.fitting_truth().iter().filter(|f| **f).count();
        let share = fitting as f32 / mesh.face_count() as f32;
        assert!((0.4..0.5).contains(&share), "fitting share {share}");
        let part = shell.part("upper-01", true);
        assert_eq!(part.fitting.as_ref().map(Vec::len), Some(mesh.face_count()));
        assert!(shell.part("upper-01", false).fitting.is_none());
    }

    #[test]
    fn shell_has_aligner_size_and_opens_down() {
        let shell = default_shell();
        let bb = shell.mesh.aabb().unwrap();
        let size = bb.size();
        assert!((58.0..66.0).contains(&size.x), "x {}", size.x);
        assert!((46.0..56.0).contains(&size.y), "y {}", size.y);
        assert!((11.3..12.5).contains(&size.z), "z {}", size.z);
        assert!(bb.min.z.abs() < 1e-4, "rim not at z = 0");
        // Fitting normals summed over area point out of the opening.
        let sum: Vec3 = (0..shell.mesh.face_count())
            .filter(|&f| shell.side[f] == Side::Fitting)
            .map(|f| shell.mesh.face_normal_raw(f))
            .sum();
        assert!(sum.normalize().dot(shell.opening) > 0.95);
    }

    #[test]
    fn wall_is_about_one_mm_everywhere() {
        let shell = aligner_shell(&ShellSpec {
            step_mm: 1.0,
            ..ShellSpec::default()
        });
        let mesh = &shell.mesh;
        let n_in = mesh.vertex_count() / 2;
        let (inner, outer) = mesh.positions.split_at(n_in);
        let mut min = f32::MAX;
        for p in outer.iter().step_by(7) {
            let d = inner
                .iter()
                .map(|q| p.distance_squared(*q))
                .fold(f32::MAX, f32::min);
            min = min.min(d.sqrt());
        }
        assert!(min > 0.85, "the offset folds: {min} mm");
    }

    #[test]
    fn face_target_is_met_and_generation_is_deterministic() {
        let shell = aligner_shell_with_faces(30_000);
        let n = shell.mesh.face_count();
        assert!((28_500..31_500).contains(&n), "{n} faces");
        assert_eq!(default_shell().mesh, default_shell().mesh);
    }
}
