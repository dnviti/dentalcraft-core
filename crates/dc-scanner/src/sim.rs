//! Simulated scanner backend `sim`.
//!
//! The device renders depth frames of a dental model by ray casting it
//! ([`Bvh`]) from a camera that orbits smoothly over the arch, like a hand
//! moving a MINI 2-class structured-light scanner: 640x400 depth, `fx = fy =
//! 952` (the MINI 2 estimate, 1190 px at 800 px wide), working distance
//! ~130 mm, so the model fills most of the frame and
//! every depth (about 113-170 mm) is inside the default acquisition range
//! ([`crate::ScanSettings`], 100-300 mm). Depth gets
//! gaussian noise (0.03 mm), random dropouts and grazing-angle dropouts.
//! Frames are paced at 10 fps by their timestamps; the real-time sleep
//! happens only inside [`ScannerDevice::next_frame`].
//!
//! The model is a procedural typodont ([`typodont_arch`]: horseshoe gingiva
//! with 14 rounded teeth on a base plate, occlusal side towards +Z), or the
//! mesh file named by the environment variable [`MESH_ENV`] (STL / OBJ / PLY,
//! units detected, occlusal side expected towards +Z).
//!
//! [`SimOptions`] makes the device deterministic and fast for tests
//! (`realtime: false`, a frame budget, a smaller image).
//!
//! **Turntable rig.** A backend built [`SimBackend::with_rig`] shares a
//! [`SimRig`] with the simulated turntable ([`dc_turntable::sim`]). While a
//! table is open on the rig, the camera stays fixed ([`rig_camera_pose`])
//! and the model turns and tilts with the table ([`rig_model_to_scene`]),
//! inside a static enclosure ([`rig_enclosure`]: a floor under the platter
//! and a back wall) that a turntable scan must ignore. Each frame advances
//! the rig clock by one frame period, so the motion is paced by the frames.

use std::f32::consts::{FRAC_PI_2, TAU};
use std::sync::Arc;
use std::time::{Duration, Instant};

use dc_geom::bvh::Bvh;
use dc_geom::tsdf::CameraIntrinsics;
use dc_mesh::{Aabb, TriMesh};
use dc_turntable::sim::{RigPose, SimRig};
use glam::{Affine3A, Mat3, Quat, Vec2, Vec3};
use rayon::prelude::*;

use crate::{DepthFrame, DeviceInfo, Result, ScanError, ScannerBackend, ScannerDevice};

/// Id of the simulated backend.
pub const BACKEND_ID: &str = "sim";
/// Device id of the simulated upper typodont.
pub const UPPER_ID: &str = "sim-typodont-upper";
/// Device id of the simulated lower typodont.
pub const LOWER_ID: &str = "sim-typodont-lower";
/// Environment variable naming a mesh file to scan instead of the typodont.
pub const MESH_ENV: &str = "DC_SCAN_SIM_MESH";

/// Behaviour of the simulated device.
#[derive(Debug, Clone, PartialEq)]
pub struct SimOptions {
    /// Pace frames in real time at `fps`; `false` delivers them at once.
    pub realtime: bool,
    /// Stop producing frames after this many (`None`: endless).
    pub max_frames: Option<u64>,
    /// The device is lost (as if unplugged) after this many frames: its
    /// stream fails with `device-lost` from then on (`None`: never).
    pub lost_after: Option<u64>,
    /// Depth image width in pixels.
    pub width: u32,
    /// Depth image height in pixels.
    pub height: u32,
    /// Frame rate (frames per second).
    pub fps: f32,
    /// Standard deviation of the depth noise (mm).
    pub noise_mm: f32,
    /// Fraction of pixels dropped at random.
    pub dropout: f32,
    /// Frames of one full orbit of the camera around the model.
    pub period_frames: u32,
    /// Distance from the camera to the orbit target (mm).
    pub working_distance_mm: f32,
    /// Seed of the noise.
    pub seed: u64,
}

impl Default for SimOptions {
    fn default() -> Self {
        Self {
            realtime: true,
            max_frames: None,
            lost_after: None,
            width: 640,
            height: 400,
            fps: 10.0,
            noise_mm: 0.03,
            dropout: 0.02,
            period_frames: 240,
            working_distance_mm: 130.0,
            seed: 0x5ca7_d00d,
        }
    }
}

/// Pinhole intrinsics of the simulated camera for an image of the given size
/// (`fx = fy = 952` at 640 pixels wide, the MINI 2 field of view;
/// principal point at the centre).
pub fn intrinsics(width: u32, height: u32) -> CameraIntrinsics {
    let f = 952.0 * width as f32 / 640.0;
    CameraIntrinsics {
        width,
        height,
        fx: f,
        fy: f,
        cx: (width as f32 - 1.0) * 0.5,
        cy: (height as f32 - 1.0) * 0.5,
    }
}

/// Camera looking at `target` from azimuth `az` and elevation `el`
/// (radians) at `distance`; axes x right, y down, z forward.
fn look_at(target: Vec3, az: f32, el: f32, distance: f32) -> Affine3A {
    let eye = target + distance * Vec3::new(el.cos() * az.cos(), el.cos() * az.sin(), el.sin());
    let forward = (target - eye).normalize();
    let away = Vec3::new(-az.cos(), -az.sin(), 0.0);
    let right = forward.cross(away).normalize();
    let down = forward.cross(right);
    Affine3A::from_mat3_translation(Mat3::from_cols(right, down, forward), eye)
}

/// Camera-to-model pose of frame `frame`: an orbit around the centre of
/// `bounds` (full turn every `period_frames`), elevation 50..74 degrees,
/// target wobbling over the arch. Camera axes: x right, y down, z forward.
pub fn camera_pose(bounds: &Aabb, frame: u64, options: &SimOptions) -> Affine3A {
    let period = options.period_frames.max(1) as u64;
    let tau = TAU * (frame % period) as f32 / period as f32;
    let size = bounds.size();
    let c = bounds.center();
    let target = Vec3::new(
        c.x + 0.12 * size.x * tau.sin(),
        c.y + 0.10 * size.y * (2.0 * tau).sin(),
        bounds.min.z + 0.5 * size.z,
    );
    let el = (62.0 + 12.0 * (3.0 * tau).sin()).to_radians();
    look_at(target, tau - FRAC_PI_2, el, options.working_distance_mm)
}

/// Height of the tilt axis of the simulated turntable under the platter
/// top (mm): the tilt axis is the scene x axis at `z = -RIG_TILT_PIVOT_MM`.
pub const RIG_TILT_PIVOT_MM: f32 = 10.0;
/// Elevation of the fixed rig camera (degrees).
pub const RIG_ELEVATION_DEG: f32 = 60.0;
/// Height of the enclosure floor under the platter top (mm).
pub const RIG_FLOOR_MM: f32 = 25.0;
/// Distance of the enclosure back wall from the turntable axis (mm).
pub const RIG_WALL_MM: f32 = 60.0;

/// Pose of the platter in the turntable scene (platter top at `z = 0`,
/// rotation axis `+Z`, tilt axis parallel to `+X` through
/// `(0, 0, -RIG_TILT_PIVOT_MM)`) at the angles `pose`.
pub fn rig_platter(pose: RigPose) -> Affine3A {
    let pivot = Vec3::new(0.0, 0.0, -RIG_TILT_PIVOT_MM);
    Affine3A::from_translation(pivot)
        * Affine3A::from_quat(Quat::from_rotation_x(pose.tilt_deg.to_radians()))
        * Affine3A::from_translation(-pivot)
        * Affine3A::from_quat(Quat::from_rotation_z(pose.rotate_deg.to_radians()))
}

/// Model-to-scene transform of a model with bounds `bounds` standing
/// centred on the platter (its lowest point on the platter top) at `pose`.
pub fn rig_model_to_scene(bounds: &Aabb, pose: RigPose) -> Affine3A {
    let c = bounds.center();
    rig_platter(pose) * Affine3A::from_translation(Vec3::new(-c.x, -c.y, -bounds.min.z))
}

/// Camera-to-scene pose of the fixed rig camera: looking at the platter
/// centre from the front (`-Y`), [`RIG_ELEVATION_DEG`] up, at
/// [`SimOptions::working_distance_mm`].
pub fn rig_camera_pose(options: &SimOptions) -> Affine3A {
    look_at(
        Vec3::new(0.0, 0.0, 6.0),
        -FRAC_PI_2,
        RIG_ELEVATION_DEG.to_radians(),
        options.working_distance_mm,
    )
}

/// The static enclosure of the rig scene: a floor [`RIG_FLOOR_MM`] under
/// the platter top and a back wall [`RIG_WALL_MM`] behind the axis.
pub fn rig_enclosure() -> TriMesh {
    let (f, w, s) = (-RIG_FLOOR_MM, RIG_WALL_MM, 150.0);
    let positions = vec![
        Vec3::new(-s, -s, f),
        Vec3::new(s, -s, f),
        Vec3::new(s, w, f),
        Vec3::new(-s, w, f),
        Vec3::new(-s, w, 100.0),
        Vec3::new(s, w, 100.0),
    ];
    let mut mesh = TriMesh::new(positions, vec![[0, 1, 2], [0, 2, 3], [3, 2, 5], [3, 5, 4]]);
    mesh.update_normals();
    mesh
}

/// The simulated backend (devices [`UPPER_ID`] and [`LOWER_ID`]).
#[derive(Debug, Clone, Default)]
pub struct SimBackend {
    options: SimOptions,
    rig: Option<Arc<SimRig>>,
}

impl SimBackend {
    /// Backend whose devices behave as `options` says.
    pub fn new(options: SimOptions) -> Self {
        Self { options, rig: None }
    }

    /// The devices render the turntable scene of `rig` while a simulated
    /// table is open on it.
    #[must_use]
    pub fn with_rig(mut self, rig: Arc<SimRig>) -> Self {
        self.rig = Some(rig);
        self
    }

    /// The device options.
    pub fn options(&self) -> &SimOptions {
        &self.options
    }

    fn info(id: &str, upper: bool) -> DeviceInfo {
        DeviceInfo {
            backend: BACKEND_ID.to_owned(),
            id: id.to_owned(),
            name: if upper {
                "Simulated typodont (upper)".to_owned()
            } else {
                "Simulated typodont (lower)".to_owned()
            },
            serial: Some(if upper { "SIM-U-0001" } else { "SIM-L-0001" }.to_owned()),
        }
    }
}

impl ScannerBackend for SimBackend {
    fn id(&self) -> &'static str {
        BACKEND_ID
    }

    fn list(&self) -> Result<Vec<DeviceInfo>> {
        Ok(vec![
            Self::info(UPPER_ID, true),
            Self::info(LOWER_ID, false),
        ])
    }

    fn open(&self, device_id: &str) -> Result<Box<dyn ScannerDevice>> {
        let upper = match device_id {
            UPPER_ID => true,
            LOWER_ID => false,
            _ => return Err(ScanError::DeviceNotFound(device_id.to_owned())),
        };
        let mesh = match std::env::var_os(MESH_ENV) {
            Some(path) if !path.is_empty() => load_mesh(&path)?,
            _ => typodont_arch(upper),
        };
        let device = SimDevice::new(Self::info(device_id, upper), &mesh, self.options.clone())?;
        Ok(Box::new(match &self.rig {
            Some(rig) => device.with_rig(Arc::clone(rig))?,
            None => device,
        }))
    }
}

fn load_mesh(path: &std::ffi::OsStr) -> Result<TriMesh> {
    let mut mesh = dc_mesh::read_scan(path, &dc_mesh::io::ReadOptions::default())?.mesh;
    let units = dc_mesh::io::detect_units(&mesh)?;
    mesh.scale_to_mm(units);
    if mesh.is_empty() {
        return Err(ScanError::Empty("simulated scene mesh"));
    }
    Ok(mesh)
}

/// The static part of the turntable scene.
#[derive(Debug)]
struct Enclosure {
    bvh: Bvh,
    face_normals: Vec<Vec3>,
}

/// A simulated device scanning a mesh.
#[derive(Debug)]
pub struct SimDevice {
    info: DeviceInfo,
    bvh: Bvh,
    face_normals: Vec<Vec3>,
    bounds: Aabb,
    options: SimOptions,
    intrinsics: CameraIntrinsics,
    frame: u64,
    started: Option<Instant>,
    rig: Option<(Arc<SimRig>, Enclosure)>,
}

impl SimDevice {
    /// Device scanning `mesh` (millimetres).
    ///
    /// # Errors
    /// [`ScanError::Empty`] for a mesh without triangles,
    /// [`ScanError::InvalidParams`] for unusable options.
    pub fn new(info: DeviceInfo, mesh: &TriMesh, options: SimOptions) -> Result<Self> {
        let bounds = mesh
            .aabb()
            .ok_or(ScanError::Empty("simulated scene mesh"))?;
        if mesh.is_empty() {
            return Err(ScanError::Empty("simulated scene mesh"));
        }
        let fps_ok = options.fps.is_finite() && options.fps > 0.0;
        if options.width < 8 || options.height < 8 || !fps_ok {
            return Err(ScanError::InvalidParams(format!(
                "simulated image {}x{} at {} fps",
                options.width, options.height, options.fps
            )));
        }
        Ok(Self {
            info,
            bvh: Bvh::build(mesh)?,
            face_normals: face_normals(mesh),
            bounds,
            intrinsics: intrinsics(options.width, options.height),
            options,
            frame: 0,
            started: None,
            rig: None,
        })
    }

    /// The device renders the turntable scene of `rig` while a simulated
    /// table is open on it (see the module doc).
    ///
    /// # Errors
    /// [`ScanError::Geom`] when the enclosure cannot be indexed.
    pub fn with_rig(mut self, rig: Arc<SimRig>) -> Result<Self> {
        let mesh = rig_enclosure();
        let enclosure = Enclosure {
            bvh: Bvh::build(&mesh)?,
            face_normals: face_normals(&mesh),
        };
        self.rig = Some((rig, enclosure));
        Ok(self)
    }

    /// Camera-to-model pose of frame `frame` (see [`camera_pose`]).
    pub fn pose(&self, frame: u64) -> Affine3A {
        camera_pose(&self.bounds, frame, &self.options)
    }

    /// Bounds of the scanned model (model coordinates).
    pub fn bounds(&self) -> Aabb {
        self.bounds
    }

    /// Index of the next frame [`ScannerDevice::next_frame`] will deliver.
    pub fn frame_index(&self) -> u64 {
        self.frame
    }

    /// Renders frame `frame` of the hand-held orbit (noise included)
    /// without advancing the stream.
    pub fn render(&self, frame: u64) -> DepthFrame {
        let pose = self.pose(frame);
        self.depth_image(frame, &pose, &|origin, dir| {
            let hit = self.bvh.ray_cast(origin, dir, 1000.0)?;
            let cos = self.face_normals[hit.face as usize].dot(dir.normalize());
            Some((hit.t, cos.abs()))
        })
    }

    /// Renders frame `frame` of the turntable scene with the table at
    /// `pose` (noise included). Without [`SimDevice::with_rig`] the
    /// enclosure is missing.
    pub fn render_rig(&self, frame: u64, pose: RigPose) -> DepthFrame {
        let camera = rig_camera_pose(&self.options);
        let scene_to_model = rig_model_to_scene(&self.bounds, pose).inverse();
        let enclosure = self.rig.as_ref().map(|(_, e)| e);
        self.depth_image(frame, &camera, &|origin, dir| {
            // the model, moved by the table (rigid: same ray parameter)
            let model = self
                .bvh
                .ray_cast(
                    scene_to_model.transform_point3(origin),
                    scene_to_model.transform_vector3(dir),
                    1000.0,
                )
                .map(|h| {
                    let d = scene_to_model.transform_vector3(dir).normalize();
                    (h.t, self.face_normals[h.face as usize].dot(d).abs())
                });
            let wall = enclosure.and_then(|e| {
                let h = e.bvh.ray_cast(origin, dir, 1000.0)?;
                Some((
                    h.t,
                    e.face_normals[h.face as usize].dot(dir.normalize()).abs(),
                ))
            });
            match (model, wall) {
                (Some(m), Some(w)) => Some(if m.0 <= w.0 { m } else { w }),
                (m, w) => m.or(w),
            }
        })
    }

    /// Depth image seen from the camera-to-scene pose `camera`, `hit`
    /// giving the ray parameter and the absolute cosine to the surface of
    /// the first hit of a scene ray (origin, direction with unit optical
    /// component).
    fn depth_image(
        &self,
        frame: u64,
        camera: &Affine3A,
        hit: &(dyn Fn(Vec3, Vec3) -> Option<(f32, f32)> + Sync),
    ) -> DepthFrame {
        let k = self.intrinsics;
        let w = k.width as usize;
        let opt = &self.options;
        let origin: Vec3 = camera.translation.into();
        let depth_mm = (0..w * k.height as usize)
            .into_par_iter()
            .with_min_len(256)
            .map(|pixel| {
                let (u, v) = ((pixel % w) as f32, (pixel / w) as f32);
                let dir_cam = Vec3::new((u - k.cx) / k.fx, (v - k.cy) / k.fy, 1.0);
                let Some((t, cos)) = hit(origin, camera.transform_vector3(dir_cam)) else {
                    return 0.0;
                };
                let mut rng = Rng::new(opt.seed, frame, pixel as u64);
                // depth along the optical axis: dir_cam.z == 1
                if cos < 0.17 || rng.uniform() < opt.dropout {
                    return 0.0;
                }
                t + opt.noise_mm * rng.gaussian()
            })
            .collect();
        DepthFrame {
            depth_mm,
            intrinsics: k,
            timestamp_us: (frame as f64 * 1e6 / opt.fps as f64) as u64,
            color: None,
            imu_orientation: None,
            imu_up: None,
        }
    }
}

fn face_normals(mesh: &TriMesh) -> Vec<Vec3> {
    (0..mesh.face_count())
        .map(|f| mesh.face_normal(f))
        .collect()
}

impl ScannerDevice for SimDevice {
    fn info(&self) -> DeviceInfo {
        self.info.clone()
    }

    fn start(&mut self) -> Result<()> {
        // the stream restarts its clock but the camera keeps its trajectory
        let elapsed = Duration::from_secs_f64(self.frame as f64 / self.options.fps as f64);
        self.started = Some(
            Instant::now()
                .checked_sub(elapsed)
                .unwrap_or_else(Instant::now),
        );
        // the frames pace the turntable from now on
        self.set_streaming(true);
        Ok(())
    }

    fn stop(&mut self) {
        self.started = None;
        self.set_streaming(false);
    }

    fn next_frame(&mut self, timeout: Duration) -> Result<Option<DepthFrame>> {
        let Some(started) = self.started else {
            return Err(ScanError::Device("stream not started".into()));
        };
        if self.options.lost_after.is_some_and(|m| self.frame >= m) {
            self.set_streaming(false);
            return Err(ScanError::DeviceLost(
                "the simulated scanner was unplugged".into(),
            ));
        }
        if self.options.max_frames.is_some_and(|m| self.frame >= m) {
            // no frame will pace the turntable any more
            self.set_streaming(false);
            std::thread::sleep(timeout.min(Duration::from_millis(5)));
            return Ok(None);
        }
        if self.options.realtime {
            let due =
                started + Duration::from_secs_f64(self.frame as f64 / self.options.fps as f64);
            let now = Instant::now();
            if due > now {
                if due - now > timeout {
                    std::thread::sleep(timeout);
                    return Ok(None);
                }
                std::thread::sleep(due - now);
            }
        }
        // with a table open on the rig, each frame moves the table by one
        // frame period and the scanner looks at the turntable scene
        let rig_pose = self
            .rig
            .as_ref()
            .filter(|(rig, _)| rig.attached())
            .map(|(rig, _)| rig.advance(1.0 / f64::from(self.options.fps)));
        let frame = match rig_pose {
            Some(pose) => self.render_rig(self.frame, pose),
            None => self.render(self.frame),
        };
        self.frame += 1;
        Ok(Some(frame))
    }
}

impl SimDevice {
    fn set_streaming(&self, on: bool) {
        if let Some((rig, _)) = &self.rig {
            rig.set_streaming(on);
        }
    }
}

impl Drop for SimDevice {
    fn drop(&mut self) {
        self.set_streaming(false);
    }
}

/// Counter-based random numbers (SplitMix64), so pixels render in parallel
/// and deterministically.
struct Rng(u64);

impl Rng {
    fn new(seed: u64, frame: u64, pixel: u64) -> Self {
        Self(
            seed ^ frame.wrapping_mul(0x9e37_79b9_7f4a_7c15)
                ^ pixel.wrapping_mul(0xd1b5_4a32_d192_ed03),
        )
    }

    fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9e37_79b9_7f4a_7c15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
        z ^ (z >> 31)
    }

    /// Uniform in `(0, 1]`.
    fn uniform(&mut self) -> f32 {
        ((self.next() >> 40) as f32 + 1.0) / (1u64 << 24) as f32
    }

    /// Standard normal (Box-Muller).
    fn gaussian(&mut self) -> f32 {
        let (u1, u2) = (self.uniform(), self.uniform());
        (-2.0 * u1.ln()).sqrt() * (TAU * u2).cos()
    }
}

// ----- procedural typodont ------------------------------------------------

/// Kind of a simulated tooth (shape of its crown).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Kind {
    Incisor,
    Canine,
    Premolar,
    Molar,
}

/// A tooth placed on the arch: centre, mesio-distal tangent, buccal normal.
struct Tooth {
    kind: Kind,
    center: Vec2,
    tangent: Vec2,
    normal: Vec2,
    half_width: f32,
    half_depth: f32,
    crown: f32,
}

/// Grid spacing of the procedural model (mm).
const GRID_MM: f32 = 0.25;
/// Height of the gingival ridge above the base plate (mm).
const GINGIVA_MM: f32 = 9.0;
/// Half width of the gingival ridge (mm).
const RIDGE_MM: f32 = 6.5;

/// Procedural dental typodont, millimetres, occlusal side towards +Z: a base
/// plate at `z = 0`, a horseshoe gingival ridge and 14 teeth (incisors,
/// canines, premolars with two cusps, molars with four), about 60 x 55 x 17
/// mm. The surface is a height field (open, one component), front of the
/// arch towards +Y. The lower arch is slightly smaller with narrower
/// incisors.
pub fn typodont_arch(upper: bool) -> TriMesh {
    let scale = if upper { 1.0 } else { 0.92 };
    let (a, b, t_max) = (22.0 * scale, 30.0 * scale, 1.9f32);
    // arch curve x = a sin t, y = b cos t, densely sampled with arc length
    let n = 800;
    let curve: Vec<Vec2> = (0..=n)
        .map(|i| {
            let t = -t_max + 2.0 * t_max * i as f32 / n as f32;
            Vec2::new(a * t.sin(), b * t.cos())
        })
        .collect();
    let mut arc = vec![0.0f32; curve.len()];
    for i in 1..curve.len() {
        arc[i] = arc[i - 1] + curve[i].distance(curve[i - 1]);
    }
    let mid = arc[n / 2];
    // (kind, mesio-distal width, bucco-lingual depth, crown height) from the midline
    let kinds = [
        Kind::Incisor,
        Kind::Incisor,
        Kind::Canine,
        Kind::Premolar,
        Kind::Premolar,
        Kind::Molar,
        Kind::Molar,
    ];
    let (widths, depths) = if upper {
        (
            [8.6, 6.6, 7.6, 7.0, 6.6, 10.2, 9.2],
            [7.0, 6.2, 8.0, 9.0, 9.2, 11.0, 10.5],
        )
    } else {
        (
            [5.4, 5.9, 6.9, 7.0, 7.1, 11.0, 10.5],
            [6.0, 6.2, 7.5, 7.7, 8.2, 10.5, 10.0],
        )
    };
    let crowns = [8.5, 8.0, 9.5, 7.5, 7.5, 6.5, 6.2];
    let fit = (0.97 * (arc[n] - mid) / widths.iter().sum::<f32>()).min(1.0);
    let at_arc = |s: f32| -> (Vec2, Vec2) {
        let i = arc.partition_point(|&x| x < s).clamp(1, n);
        let f = (s - arc[i - 1]) / (arc[i] - arc[i - 1]).max(1e-6);
        let p = curve[i - 1].lerp(curve[i], f);
        (p, (curve[i] - curve[i - 1]).normalize())
    };
    let mut teeth = Vec::with_capacity(14);
    let mut s = 0.0;
    for k in 0..7 {
        let w = widths[k] * fit;
        for side in [-1.0f32, 1.0] {
            let (center, tangent) = at_arc(mid + side * (s + 0.5 * w));
            // buccal: away from the arch centre (the curve bends around the origin)
            let mut normal = Vec2::new(tangent.y, -tangent.x);
            if normal.dot(center) < 0.0 {
                normal = -normal;
            }
            teeth.push(Tooth {
                kind: kinds[k],
                center,
                tangent,
                normal,
                half_width: 0.5 * w,
                half_depth: 0.5 * depths[k] * scale,
                crown: crowns[k],
            });
        }
        s += w;
    }

    let margin = RIDGE_MM + 3.0;
    let lo = curve
        .iter()
        .fold(Vec2::splat(f32::INFINITY), |m, p| m.min(*p))
        - margin;
    let hi = curve
        .iter()
        .fold(Vec2::splat(f32::NEG_INFINITY), |m, p| m.max(*p))
        + margin;
    let nx = ((hi.x - lo.x) / GRID_MM).ceil() as usize + 1;
    let ny = ((hi.y - lo.y) / GRID_MM).ceil() as usize + 1;
    let positions: Vec<Vec3> = (0..nx * ny)
        .into_par_iter()
        .map(|i| {
            let p = lo + GRID_MM * Vec2::new((i % nx) as f32, (i / nx) as f32);
            let d = curve
                .iter()
                .map(|c| c.distance_squared(p))
                .fold(f32::INFINITY, f32::min)
                .sqrt();
            let mut h = gingiva(d);
            for t in &teeth {
                h = h.max(tooth_height(t, p));
            }
            p.extend(h)
        })
        .collect();
    let mut tris = Vec::with_capacity(2 * (nx - 1) * (ny - 1));
    for j in 0..ny - 1 {
        for i in 0..nx - 1 {
            let v00 = (j * nx + i) as u32;
            let (v10, v01) = (v00 + 1, v00 + nx as u32);
            let v11 = v01 + 1;
            tris.push([v00, v10, v11]);
            tris.push([v00, v11, v01]);
        }
    }
    let mut mesh = TriMesh::new(positions, tris);
    mesh.update_normals();
    mesh
}

/// Height of the gingival ridge at distance `d` from the arch curve.
fn gingiva(d: f32) -> f32 {
    let r = d / RIDGE_MM;
    if r >= 1.0 {
        0.0
    } else {
        GINGIVA_MM * (1.0 - r * r).powf(0.7)
    }
}

/// Height of the crown of `t` at `p` (0 outside the crown).
fn tooth_height(t: &Tooth, p: Vec2) -> f32 {
    let q = p - t.center;
    let (s, n) = (
        q.dot(t.tangent) / t.half_width,
        q.dot(t.normal) / t.half_depth,
    );
    let r2 = s * s + n * n;
    if r2 >= 1.0 {
        return 0.0;
    }
    let dome = (1.0 - r2).powf(0.35);
    let crown = match t.kind {
        Kind::Incisor => {
            // thin blade: lower towards the buccal and lingual faces
            t.crown * dome * (1.0 - 0.45 * n * n)
        }
        Kind::Canine => t.crown * (dome - 0.25 * r2.sqrt()),
        Kind::Premolar => {
            // buccal and lingual cusps around a mesio-distal fissure
            t.crown * dome - 1.4 * (1.0 - r2) * (-(n / 0.25).powi(2)).exp()
        }
        Kind::Molar => {
            // four cusps: two crossing fissures
            t.crown * dome
                - 1.2 * (1.0 - r2) * (-(n / 0.22).powi(2)).exp()
                - 0.9 * (1.0 - r2) * (-(s / 0.22).powi(2)).exp()
        }
    };
    GINGIVA_MM - 1.5 + crown.max(0.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fast() -> SimOptions {
        SimOptions {
            realtime: false,
            max_frames: Some(3),
            width: 160,
            height: 100,
            ..SimOptions::default()
        }
    }

    #[test]
    fn typodont_is_a_dental_sized_height_field() {
        let m = typodont_arch(true);
        assert!(m.check().is_ok());
        let b = m.aabb().unwrap();
        let s = b.size();
        assert!((50.0..70.0).contains(&s.x), "width {}", s.x);
        assert!((45.0..65.0).contains(&s.y), "depth {}", s.y);
        assert!((14.0..20.0).contains(&s.z), "height {}", s.z);
        assert_eq!(b.min.z, 0.0);
        assert_eq!(m.connected_components().len(), 1);
        let lower = typodont_arch(false).aabb().unwrap().size();
        assert!(lower.x < s.x && lower.y < s.y);
    }

    #[test]
    fn camera_looks_at_the_model_from_above() {
        let m = typodont_arch(true);
        let b = m.aabb().unwrap();
        for f in [0, 60, 120, 180] {
            let pose = camera_pose(&b, f, &SimOptions::default());
            let forward = Vec3::from(pose.matrix3.z_axis);
            assert!(forward.z < -0.6, "frame {f} looks along {forward}");
            let to_center = (b.center() - Vec3::from(pose.translation)).normalize();
            assert!(forward.dot(to_center) > 0.95);
            // right-handed rotation
            assert!((pose.matrix3.determinant() - 1.0).abs() < 1e-4);
        }
    }

    #[test]
    fn frames_are_noisy_depth_of_the_model() {
        let backend = SimBackend::new(fast());
        assert!(matches!(
            backend.open("nope"),
            Err(ScanError::DeviceNotFound(_))
        ));
        let mut dev = backend.open(UPPER_ID).unwrap();
        assert!(dev.next_frame(Duration::ZERO).is_err(), "not started");
        dev.start().unwrap();
        let f = dev.next_frame(Duration::from_secs(1)).unwrap().unwrap();
        assert_eq!(f.depth_mm.len(), 160 * 100);
        let valid: Vec<f32> = f.depth_mm.iter().copied().filter(|d| *d > 0.0).collect();
        assert!(valid.len() > f.depth_mm.len() / 2, "{} valid", valid.len());
        assert!(valid.iter().all(|d| (100.0..180.0).contains(d)));
        dev.next_frame(Duration::ZERO).unwrap().unwrap();
        dev.next_frame(Duration::ZERO).unwrap().unwrap();
        // frame budget exhausted
        assert!(dev.next_frame(Duration::ZERO).unwrap().is_none());
    }

    #[test]
    fn rendering_is_deterministic() {
        let mesh = typodont_arch(false);
        let info = SimBackend::info(LOWER_ID, false);
        let a = SimDevice::new(info.clone(), &mesh, fast()).unwrap();
        let b = SimDevice::new(info, &mesh, fast()).unwrap();
        assert_eq!(a.render(7), b.render(7));
        assert_ne!(a.render(7).depth_mm, a.render(8).depth_mm);
    }
}
