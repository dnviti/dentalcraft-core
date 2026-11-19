//! ICP tests. `tests/fixtures/icp_oracle.txt` was written once by a one-off
//! script running the add-on's `visit.kabsch`, `visit.point_to_plane`,
//! `visit.icp` (nearest vertex, vertex normals) and `bite._ficp_keep` with
//! numpy on a small tooth-like mesh.

use super::*;

/// Icosphere with `subdiv` subdivisions on a bumpy ellipsoid (a crown-sized,
/// asymmetric blob), radii about 4 x 5 x 3.5 mm.
fn tooth(subdiv: usize) -> TriMesh {
    let t = (1.0 + 5f64.sqrt()) / 2.0;
    let mut v: Vec<DVec3> = [
        (-1.0, t, 0.0),
        (1.0, t, 0.0),
        (-1.0, -t, 0.0),
        (1.0, -t, 0.0),
        (0.0, -1.0, t),
        (0.0, 1.0, t),
        (0.0, -1.0, -t),
        (0.0, 1.0, -t),
        (t, 0.0, -1.0),
        (t, 0.0, 1.0),
        (-t, 0.0, -1.0),
        (-t, 0.0, 1.0),
    ]
    .iter()
    .map(|&(x, y, z)| DVec3::new(x, y, z).normalize())
    .collect();
    let mut f: Vec<[u32; 3]> = vec![
        [0, 11, 5],
        [0, 5, 1],
        [0, 1, 7],
        [0, 7, 10],
        [0, 10, 11],
        [1, 5, 9],
        [5, 11, 4],
        [11, 10, 2],
        [10, 7, 6],
        [7, 1, 8],
        [3, 9, 4],
        [3, 4, 2],
        [3, 2, 6],
        [3, 6, 8],
        [3, 8, 9],
        [4, 9, 5],
        [2, 4, 11],
        [6, 2, 10],
        [8, 6, 7],
        [9, 8, 1],
    ];
    for _ in 0..subdiv {
        let mut cache = std::collections::HashMap::new();
        let mut nf = Vec::with_capacity(f.len() * 4);
        let mut mid = |a: u32, b: u32, v: &mut Vec<DVec3>| -> u32 {
            *cache.entry((a.min(b), a.max(b))).or_insert_with(|| {
                v.push((v[a as usize] + v[b as usize]).normalize());
                (v.len() - 1) as u32
            })
        };
        for [a, b, c] in f {
            let ab = mid(a, b, &mut v);
            let bc = mid(b, c, &mut v);
            let ca = mid(c, a, &mut v);
            nf.extend([[a, ab, ca], [b, bc, ab], [c, ca, bc], [ab, bc, ca]]);
        }
        f = nf;
    }
    let pos = v
        .iter()
        .map(|u| {
            let th = u.y.atan2(u.x);
            let ph = u.z.clamp(-1.0, 1.0).acos();
            let r = 1.0
                + 0.12 * (3.0 * th + 0.4).sin() * (2.0 * ph).cos()
                + 0.06 * (5.0 * ph + 1.0).cos();
            (*u * r * DVec3::new(4.0, 5.0, 3.5)).as_vec3()
        })
        .collect();
    TriMesh::new(pos, f)
}

fn true_transform() -> DAffine3 {
    let axis = DVec3::new(0.3, -0.5, 0.8).normalize();
    DAffine3::from_rotation_translation(
        glam::DQuat::from_axis_angle(axis, 8f64.to_radians()),
        DVec3::new(0.6, -0.4, 0.9),
    )
}

/// Largest displacement between two transforms over `pts` (mm) and the
/// angle of the relative rotation (degrees).
fn transform_error(a: &DAffine3, b: &DAffine3, pts: &[DVec3]) -> (f64, f64) {
    let d = pts
        .iter()
        .map(|p| a.transform_point3(*p).distance(b.transform_point3(*p)))
        .fold(0.0, f64::max);
    let rel = DAffine3::from_mat3(a.matrix3 * b.matrix3.transpose());
    (d, rotation_angle(&rel).to_degrees())
}

/// Source points and normals: the target vertices moved by `T^-1`.
fn source_of(mesh: &TriMesh, t: &DAffine3) -> (Vec<DVec3>, Vec<DVec3>) {
    let inv = t.inverse();
    let n = mesh.compute_vertex_normals();
    let p = mesh
        .positions
        .iter()
        .map(|p| inv.transform_point3(p.as_dvec3()))
        .collect();
    let n = n
        .iter()
        .map(|n| inv.transform_vector3(n.as_dvec3()).normalize())
        .collect();
    (p, n)
}

/// Deterministic uniform numbers in [0, 1).
struct Lcg(u64);
impl Lcg {
    fn next(&mut self) -> f64 {
        self.0 = self
            .0
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        (self.0 >> 11) as f64 / (1u64 << 53) as f64
    }
}

#[test]
fn kabsch_recovers_rigid_motion_and_scale() {
    let t = true_transform();
    let mut rng = Lcg(1);
    let a: Vec<DVec3> = (0..50)
        .map(|_| DVec3::new(rng.next(), rng.next(), rng.next()) * 10.0)
        .collect();
    let b: Vec<DVec3> = a.iter().map(|p| t.transform_point3(*p)).collect();
    let k = kabsch(&a, &b, None, false).unwrap();
    let (d, ang) = transform_error(&k, &t, &a);
    assert!(d < 1e-10 && ang < 1e-8, "{d} {ang}");

    // Umeyama: similarity with scale 1.7
    let s = DAffine3::from_scale(DVec3::splat(1.7));
    let b2: Vec<DVec3> = a.iter().map(|p| (t * s).transform_point3(*p)).collect();
    let k2 = kabsch(&a, &b2, None, true).unwrap();
    assert!((k2.matrix3.x_axis.length() - 1.7).abs() < 1e-10);
    let d2 = a
        .iter()
        .zip(&b2)
        .map(|(p, q)| k2.transform_point3(*p).distance(*q))
        .fold(0.0, f64::max);
    assert!(d2 < 1e-9);

    // weights: zero-weighted outliers are ignored
    let mut b3 = b.clone();
    let mut w = vec![1.0; a.len()];
    for i in (0..a.len()).step_by(5) {
        b3[i] += DVec3::new(3.0, -2.0, 5.0);
        w[i] = 0.0;
    }
    let k3 = kabsch(&a, &b3, Some(&w), false).unwrap();
    assert!(transform_error(&k3, &t, &a).0 < 1e-10);
}

#[test]
fn kabsch_never_reflects() {
    // coplanar points mirrored through their plane: the best rotation, not a reflection
    let a: Vec<DVec3> = (0..20)
        .map(|i| DVec3::new((i % 5) as f64, (i / 5) as f64, 0.0))
        .collect();
    let b: Vec<DVec3> = a.iter().map(|p| DVec3::new(p.x, -p.y, 0.0)).collect();
    let k = kabsch(&a, &b, None, false).unwrap();
    assert!((k.matrix3.determinant() - 1.0).abs() < 1e-12);
}

#[test]
fn kabsch_rejects_bad_input() {
    let a = [DVec3::ZERO; 3];
    assert!(kabsch(&a, &a[..2], None, false).is_err());
    assert!(kabsch(&[], &[], None, false).is_err());
    assert!(kabsch(&a, &a, Some(&[1.0]), false).is_err());
    assert!(kabsch(&a, &a, Some(&[0.0; 3]), false).is_err());
}

#[test]
fn point_to_plane_recovers_transform_on_tooth() {
    let mesh = tooth(4);
    let t = true_transform();
    let (src, n) = source_of(&mesh, &t);
    let target = IcpTarget::new(&mesh).unwrap();
    let r = icp(&src, Some(&n), &target, &IcpParams::default()).unwrap();
    let (d, ang) = transform_error(&r.transform, &t, &src);
    assert!(r.converged && !r.lost, "{r:?}");
    assert!(
        d < 1e-4 && ang < 1e-4,
        "displacement {d} mm, angle {ang} deg"
    );
    assert_eq!(r.rms_history.len(), r.iterations);
    assert!(r.rms < 1e-5);
    assert!((r.overlap - 0.8).abs() < 0.02, "{}", r.overlap);
}

#[test]
fn point_to_point_recovers_transform_on_tooth() {
    let mesh = tooth(4);
    let t = true_transform();
    let (src, n) = source_of(&mesh, &t);
    let target = IcpTarget::new(&mesh).unwrap();
    let params = IcpParams {
        metric: IcpMetric::PointToPoint,
        trim: Trim::None,
        max_iterations: 500,
        tolerance: 1e-12,
        ..IcpParams::default()
    };
    let r = icp(&src, Some(&n), &target, &params).unwrap();
    let (d, ang) = transform_error(&r.transform, &t, &src);
    assert!(
        d < 1e-4 && ang < 1e-4,
        "displacement {d} mm, angle {ang} deg, {r:?}"
    );
}

#[test]
fn trimmed_icp_ignores_20_percent_outliers() {
    let mesh = tooth(4);
    let t = true_transform();
    let (mut src, n) = source_of(&mesh, &t);
    let inliers: Vec<DVec3> = src.clone();
    // every fifth point pushed 1.5 to 3 mm off the surface
    let mut rng = Lcg(42);
    for i in (0..src.len()).step_by(5) {
        let off = n[i] * (1.5 + 1.5 * rng.next())
            + DVec3::new(rng.next() - 0.5, rng.next() - 0.5, rng.next() - 0.5);
        src[i] += off;
    }
    let target = IcpTarget::new(&mesh).unwrap();
    for trim in [Trim::Fraction(0.7), Trim::Auto { xi_min: 0.2 }] {
        let params = IcpParams {
            trim,
            ..IcpParams::default()
        };
        let r = icp(&src, Some(&n), &target, &params).unwrap();
        let (d, ang) = transform_error(&r.transform, &t, &inliers);
        assert!(d < 1e-4 && ang < 1e-4, "{trim:?}: {d} mm, {ang} deg");
        assert!(r.overlap <= 0.8 + 1e-9, "{trim:?}: overlap {}", r.overlap);
    }
}

#[test]
fn bite_variant_and_gates() {
    let mesh = tooth(4);
    let small = DAffine3::from_rotation_translation(
        glam::DQuat::from_axis_angle(DVec3::Z, 2f64.to_radians()),
        DVec3::new(0.1, 0.05, -0.08),
    );
    let (src, n) = source_of(&mesh, &small);
    let target = IcpTarget::new(&mesh).unwrap();
    let params = IcpParams::bite(40, 1.0, 45.0, IcpMetric::PointToPlane, 0.1);
    let r = icp(&src, Some(&n), &target, &params).unwrap();
    assert!(r.converged && !r.lost);
    assert!(transform_error(&r.transform, &small, &src).0 < 1e-3);

    // a gate smaller than every distance loses the target
    let far = DAffine3::from_translation(DVec3::new(0.0, 0.0, 30.0));
    let params = IcpParams {
        max_pair_distance: 0.5,
        initial: far,
        ..IcpParams::default()
    };
    let r = icp(&src, Some(&n), &target, &params).unwrap();
    assert!(r.lost && r.iterations == 0 && r.transform == far);

    // flipped source normals fail the normal gate
    let flipped: Vec<DVec3> = n.iter().map(|v| -*v).collect();
    let r = icp(&src, Some(&flipped), &target, &IcpParams::default()).unwrap();
    assert!(r.lost);
}

#[test]
fn mesh_wrappers_and_errors() {
    let mesh = tooth(3);
    let t = DAffine3::from_rotation_translation(
        glam::DQuat::from_axis_angle(DVec3::X, 3f64.to_radians()),
        DVec3::new(0.2, 0.0, 0.1),
    );
    let mut src = mesh.clone();
    let inv = t.inverse();
    for p in &mut src.positions {
        *p = inv.transform_point3(p.as_dvec3()).as_vec3();
    }
    let r = point_to_plane(&src, &mesh, &IcpParams::default()).unwrap();
    let pts: Vec<DVec3> = src.positions.iter().map(|p| p.as_dvec3()).collect();
    assert!(transform_error(&r.transform, &t, &pts).0 < 1e-4);

    // a point cloud (no triangles, no normals) registers without the normal gate
    let cloud = TriMesh::new(src.positions.clone(), Vec::new());
    let r = register(&cloud, &mesh, &IcpParams::default()).unwrap();
    assert!(!r.lost && transform_error(&r.transform, &t, &pts).0 < 1e-4);

    let empty = TriMesh::default();
    assert!(matches!(
        point_to_plane(&empty, &empty, &IcpParams::default()),
        Err(GeomError::InvalidInput(_))
    ));
    assert!(matches!(
        register(&empty, &mesh, &IcpParams::default()),
        Err(GeomError::InvalidInput(_))
    ));
    let target = IcpTarget::new(&mesh).unwrap();
    let bad = IcpParams {
        trim: Trim::Fraction(0.0),
        ..IcpParams::default()
    };
    assert!(icp(&pts, None, &target, &bad).is_err());
    assert!(icp(&pts, Some(&pts[..3]), &target, &IcpParams::default()).is_err());
}

#[test]
fn deterministic() {
    let mesh = tooth(3);
    let (src, n) = source_of(&mesh, &true_transform());
    let target = IcpTarget::new(&mesh).unwrap();
    let p = IcpParams {
        trim: Trim::Auto { xi_min: 0.2 },
        ..IcpParams::default()
    };
    let a = icp(&src, Some(&n), &target, &p).unwrap();
    let b = icp(&src, Some(&n), &target, &p).unwrap();
    assert_eq!(a, b);
}

// ---------------------------------------------------------------------------
// oracle fixture
// ---------------------------------------------------------------------------

struct Oracle(std::collections::HashMap<String, String>);

impl Oracle {
    fn load() -> Self {
        let text = include_str!("../../tests/fixtures/icp_oracle.txt");
        Self(
            text.lines()
                .filter(|l| !l.starts_with('#'))
                .filter_map(|l| l.split_once(": "))
                .map(|(k, v)| (k.to_string(), v.to_string()))
                .collect(),
        )
    }
    fn f64s(&self, k: &str) -> Vec<f64> {
        self.0[k]
            .split_whitespace()
            .map(|h| f64::from_bits(u64::from_str_radix(h, 16).unwrap()))
            .collect()
    }
    fn vecs(&self, k: &str) -> Vec<DVec3> {
        self.f64s(k)
            .chunks(3)
            .map(|c| DVec3::new(c[0], c[1], c[2]))
            .collect()
    }
    /// Row-major 3x4 matrix as an affine transform.
    fn affine(&self, k: &str) -> DAffine3 {
        let m = self.f64s(k);
        DAffine3::from_mat3_translation(
            DMat3::from_cols(
                DVec3::new(m[0], m[4], m[8]),
                DVec3::new(m[1], m[5], m[9]),
                DVec3::new(m[2], m[6], m[10]),
            ),
            DVec3::new(m[3], m[7], m[11]),
        )
    }
    fn mesh(&self) -> TriMesh {
        let pos = self.vecs("positions").iter().map(|p| p.as_vec3()).collect();
        let idx: Vec<u32> = self.0["tris"]
            .split_whitespace()
            .map(|s| s.parse().unwrap())
            .collect();
        TriMesh::new(pos, idx.chunks(3).map(|c| [c[0], c[1], c[2]]).collect())
    }
}

fn max_abs_diff(a: &DAffine3, b: &DAffine3) -> f64 {
    let (x, y) = (a.to_cols_array(), b.to_cols_array());
    x.iter()
        .zip(y)
        .map(|(p, q)| (p - q).abs())
        .fold(0.0, f64::max)
}

#[test]
fn oracle_kabsch_and_plane_step() {
    let o = Oracle::load();
    let k = kabsch(&o.vecs("kabsch_a"), &o.vecs("kabsch_b"), None, false).unwrap();
    let r = o.f64s("kabsch_r");
    let t = o.f64s("kabsch_t");
    let want = DAffine3::from_mat3_translation(
        DMat3::from_cols(
            DVec3::new(r[0], r[3], r[6]),
            DVec3::new(r[1], r[4], r[7]),
            DVec3::new(r[2], r[5], r[8]),
        ),
        DVec3::new(t[0], t[1], t[2]),
    );
    assert!(max_abs_diff(&k, &want) < 1e-12, "{k:?} vs {want:?}");

    let d =
        point_to_plane_step(&o.vecs("plane_p"), &o.vecs("plane_q"), &o.vecs("plane_n")).unwrap();
    let want = o.affine("plane_d");
    assert!(max_abs_diff(&d, &want) < 1e-12, "{d:?} vs {want:?}");
}

#[test]
fn oracle_icp_matches_plugin() {
    let o = Oracle::load();
    let mesh = o.mesh();
    let target = IcpTarget::new(&mesh).unwrap();
    let src = o.vecs("icp_src");
    let src_n = o.vecs("icp_src_n");
    let r = icp(&src, Some(&src_n), &target, &IcpParams::default()).unwrap();
    let plugin = o.affine("icp_plugin");
    let truth = o.affine("icp_true");
    let (d_plugin, a_plugin) = transform_error(&r.transform, &plugin, &src);
    let (d_true, _) = transform_error(&r.transform, &truth, &src);
    assert!(
        d_plugin < 1e-5 && a_plugin < 1e-4,
        "{d_plugin} mm {a_plugin} deg"
    );
    assert!(d_true < 1e-5);
    assert!(o.f64s("icp_plugin_err")[0] < 1e-9 && r.rms < 1e-5);
}

#[test]
fn oracle_ficp_overlap() {
    let o = Oracle::load();
    let dist = o.f64s("ficp_dist");
    let mut pairs: Vec<(u32, f64, DVec3, DVec3)> = dist
        .iter()
        .enumerate()
        .map(|(i, &d)| (i as u32, d, DVec3::ZERO, DVec3::ZERO))
        .collect();
    trim_pairs(&mut pairs, 80, Trim::Auto { xi_min: 0.2 });
    let keep: usize = o.0["ficp_keep"].trim().parse().unwrap();
    assert_eq!(pairs.len(), keep);
    assert_eq!(pairs.len() as f64 / 80.0, o.f64s("ficp_xi")[0]);
}

#[test]
fn conditioned_solve_keeps_the_start_along_weak_directions() {
    // points on two perpendicular planes, moved by a known small motion
    let truth =
        DAffine3::from_rotation_z(0.01) * DAffine3::from_translation(DVec3::new(0.2, -0.1, 0.3));
    let mut p = Vec::new();
    let mut n = Vec::new();
    for i in 0..20 {
        for j in 0..20 {
            let (a, b) = (i as f64 - 10.0, j as f64 - 10.0);
            p.push(DVec3::new(a, b, 0.0));
            n.push(DVec3::Z);
            p.push(DVec3::new(a, 0.0, b + 12.0));
            n.push(DVec3::Y);
            p.push(DVec3::new(0.0, a, b + 12.0));
            n.push(DVec3::X);
        }
    }
    let q: Vec<DVec3> = p.iter().map(|x| truth.transform_point3(*x)).collect();
    let nq: Vec<DVec3> = n.iter().map(|x| truth.transform_vector3(*x)).collect();
    let s = point_to_plane_solve(&p, &q, &nq, None, 1e-3).unwrap();
    assert_eq!(s.weak, 0);
    assert!(s.conditioning > 1e-3, "{s:?}");
    let moved = s.delta.transform_point3(DVec3::new(5.0, 5.0, 5.0));
    assert!(moved.distance(truth.transform_point3(DVec3::new(5.0, 5.0, 5.0))) < 0.01);

    // one plane only: sliding and turning in it are left alone
    let (p1, q1, n1): (Vec<DVec3>, Vec<DVec3>, Vec<DVec3>) = (0..400)
        .map(|k| {
            let x = DVec3::new((k % 20) as f64, (k / 20) as f64, 0.0);
            (x, x + DVec3::new(3.0, 2.0, 0.5), DVec3::Z)
        })
        .fold((vec![], vec![], vec![]), |mut acc, (a, b, c)| {
            acc.0.push(a);
            acc.1.push(b);
            acc.2.push(c);
            acc
        });
    let w = vec![1.0; p1.len()];
    let s = point_to_plane_solve(&p1, &q1, &n1, Some(&w), 1e-3).unwrap();
    assert_eq!(s.weak, 3);
    let t = s.delta.translation;
    assert!(
        (t.z - 0.5).abs() < 1e-9 && t.x.abs() < 1e-9 && t.y.abs() < 1e-9,
        "{t}"
    );
    assert!(point_to_plane_solve(&p1, &q1[..3], &n1, None, 1e-3).is_err());
}
