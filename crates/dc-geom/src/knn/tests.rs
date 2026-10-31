use super::*;

/// Tiny deterministic generator (no extra dependency).
struct Lcg(u64);

impl Lcg {
    fn next_f32(&mut self) -> f32 {
        self.0 = self
            .0
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        ((self.0 >> 40) as f32) / ((1u64 << 24) as f32)
    }

    fn vec(&mut self, lo: f32, hi: f32) -> Vec3 {
        let mut c = || lo + (hi - lo) * self.next_f32();
        Vec3::new(c(), c(), c())
    }
}

/// Reference: all points sorted by (squared f64 distance, index).
fn brute(points: &[Vec3], q: Vec3, k: usize) -> Vec<(u32, f64)> {
    let mut all: Vec<(u32, f64)> = points
        .iter()
        .enumerate()
        .map(|(i, p)| (i as u32, (p.as_dvec3() - q.as_dvec3()).length_squared()))
        .collect();
    all.sort_by(|a, b| a.1.total_cmp(&b.1).then(a.0.cmp(&b.0)));
    all.truncate(k);
    all
}

fn check_against_brute(points: &[Vec3], queries: &[Vec3], k: usize) {
    let index = KnnIndex::build(points).unwrap();
    let batch = index.knn_batch(queries, k);
    assert_eq!(batch.queries(), queries.len());
    for (i, &q) in queries.iter().enumerate() {
        let want = brute(points, q, k);
        let got = index.nearest_k(q, k);
        assert_eq!(got.len(), want.len());
        for (j, ((bi, bd2), g)) in want.iter().zip(&got).enumerate() {
            assert_eq!(g.index, *bi, "query {i}, rank {j}");
            assert!((g.distance as f64 - bd2.sqrt()).abs() <= 1e-6 * (1.0 + bd2.sqrt()));
            assert_eq!(batch.indices_of(i)[j], g.index);
            assert_eq!(batch.distances_of(i)[j], g.distance);
        }
        for j in got.len()..k {
            assert_eq!(batch.indices_of(i)[j], NO_NEIGHBOR);
            assert!(batch.distances_of(i)[j].is_infinite());
        }
    }
}

#[test]
fn matches_brute_force_on_random_points() {
    let mut rng = Lcg(1);
    let points: Vec<Vec3> = (0..1500).map(|_| rng.vec(-10.0, 10.0)).collect();
    let queries: Vec<Vec3> = (0..10_000).map(|_| rng.vec(-12.0, 12.0)).collect();
    check_against_brute(&points, &queries, 8);
    check_against_brute(&points, &queries[..500], 1);
    check_against_brute(&points, &queries[..500], 40);
}

#[test]
fn ties_go_to_the_smallest_index() {
    // integer lattice: huge tie groups at every distance
    let mut points = Vec::new();
    for z in 0..6 {
        for y in 0..6 {
            for x in 0..6 {
                points.push(Vec3::new(x as f32, y as f32, z as f32));
            }
        }
    }
    let queries: Vec<Vec3> = (0..200)
        .map(|i| {
            Vec3::new(
                (i % 7) as f32 * 0.5,
                (i % 5) as f32 * 0.5,
                (i % 3) as f32 * 0.5,
            )
        })
        .collect();
    for k in [1, 2, 3, 6, 7, 12] {
        check_against_brute(&points, &queries, k);
    }
    // coincident points: the smallest indices win
    let same = vec![Vec3::ONE; 50];
    let index = KnnIndex::build(&same).unwrap();
    let got: Vec<u32> = index
        .nearest_k(Vec3::ZERO, 4)
        .iter()
        .map(|n| n.index)
        .collect();
    assert_eq!(got, [0, 1, 2, 3]);
}

#[test]
fn fewer_points_than_k_pads() {
    let points = vec![Vec3::ZERO, Vec3::X, Vec3::Y];
    let index = KnnIndex::build(&points).unwrap();
    let r = index.knn_batch(
        &[Vec3::new(0.1, 0.0, 0.0), Vec3::new(f32::NAN, 0.0, 0.0)],
        5,
    );
    assert_eq!(r.indices_of(0), [0, 1, 2, NO_NEIGHBOR, NO_NEIGHBOR]);
    assert!(r.distances_of(0)[3].is_infinite());
    assert!((r.distances_of(0)[0] - 0.1).abs() < 1e-6);
    // non-finite query: all padding
    assert!(r.indices_of(1).iter().all(|&i| i == NO_NEIGHBOR));
    assert!(r.distances_of(1).iter().all(|d| d.is_infinite()));
}

#[test]
fn empty_and_invalid_input() {
    let empty = KnnIndex::build(&[]).unwrap();
    assert!(empty.is_empty());
    assert_eq!(empty.len(), 0);
    assert!(empty.nearest(Vec3::ZERO).is_none());
    assert!(empty.nearest_k(Vec3::ZERO, 3).is_empty());
    assert!(empty.within_radius(Vec3::ZERO, 1.0).is_empty());
    let r = empty.knn_batch(&[Vec3::ZERO], 2);
    assert_eq!(r.indices, [NO_NEIGHBOR; 2]);
    // k = 0 and no queries
    let one = KnnIndex::build(&[Vec3::ZERO]).unwrap();
    assert!(one.nearest_k(Vec3::ZERO, 0).is_empty());
    let r = one.knn_batch(&[Vec3::ZERO], 0);
    assert!(r.indices.is_empty() && r.queries() == 0);
    assert_eq!(one.knn_batch(&[], 3).queries(), 0);
    // non-finite points are rejected
    assert!(matches!(
        KnnIndex::build(&[Vec3::ZERO, Vec3::new(f32::INFINITY, 0.0, 0.0)]),
        Err(GeomError::InvalidInput(_))
    ));
}

#[test]
fn radius_queries_match_brute_force() {
    let mut rng = Lcg(3);
    let points: Vec<Vec3> = (0..2000).map(|_| rng.vec(0.0, 10.0)).collect();
    let index = KnnIndex::build(&points).unwrap();
    let queries: Vec<Vec3> = (0..200).map(|_| rng.vec(0.0, 10.0)).collect();
    let batch = index.within_radius_batch(&queries, 1.2);
    for (i, &q) in queries.iter().enumerate() {
        let want: Vec<u32> = brute(&points, q, points.len())
            .into_iter()
            .take_while(|&(_, d2)| d2 <= (1.2f32 as f64) * (1.2f32 as f64))
            .map(|(i, _)| i)
            .collect();
        let got: Vec<u32> = index
            .within_radius(q, 1.2)
            .iter()
            .map(|n| n.index)
            .collect();
        assert_eq!(got, want, "query {i}");
        assert_eq!(batch[i].iter().map(|n| n.index).collect::<Vec<_>>(), want);
    }
    assert!(index.within_radius(Vec3::ZERO, -1.0).is_empty());
    assert!(index.within_radius(Vec3::ZERO, f32::NAN).is_empty());
    assert_eq!(index.within_radius(points[7], 0.0)[0].index, 7);
}

#[test]
fn nearest_batch_and_one_shot() {
    let mut rng = Lcg(4);
    let points: Vec<Vec3> = (0..500).map(|_| rng.vec(-1.0, 1.0)).collect();
    let queries: Vec<Vec3> = (0..300).map(|_| rng.vec(-1.0, 1.0)).collect();
    let index = KnnIndex::build(&points).unwrap();
    let nb = index.nearest_batch(&queries);
    let flat = knn(&points, &queries, 3).unwrap();
    for (i, &q) in queries.iter().enumerate() {
        let want = brute(&points, q, 3);
        assert_eq!(nb[i].unwrap().index, want[0].0);
        assert_eq!(flat.indices_of(i), [want[0].0, want[1].0, want[2].0]);
    }
}

/// Release-only timing: `cargo test -p dc-geom --release -- --ignored knn_throughput`.
#[test]
#[ignore = "timing test, run with --release"]
fn knn_throughput() {
    let mut rng = Lcg(5);
    let points: Vec<Vec3> = (0..300_000).map(|_| rng.vec(-50.0, 50.0)).collect();
    let queries: Vec<Vec3> = (0..300_000).map(|_| rng.vec(-50.0, 50.0)).collect();
    let t0 = std::time::Instant::now();
    let index = KnnIndex::build(&points).unwrap();
    eprintln!(
        "k-d tree build of {} points: {:?}",
        points.len(),
        t0.elapsed()
    );
    let t0 = std::time::Instant::now();
    let r = index.knn_batch(&queries, 8);
    eprintln!("{} queries, k = 8: {:?}", queries.len(), t0.elapsed());
    assert_eq!(r.queries(), queries.len());
}
