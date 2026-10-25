//! Point cloud, laser and occupancy kernels. Every algorithm here has a bit-for-bit (or
//! tolerance-level, for transcendental functions) twin in ROS2.Net.Native's managed fallback.

use crate::kdtree::KdTree;
use std::collections::{BTreeMap, VecDeque};

#[inline]
fn finite(p: &[f32; 3]) -> bool {
    p[0].is_finite() && p[1].is_finite() && p[2].is_finite()
}

#[inline]
fn pt(xyz: &[f32], i: usize) -> [f32; 3] {
    [xyz[3 * i], xyz[3 * i + 1], xyz[3 * i + 2]]
}

/// Voxel-grid centroids, ordered by voxel key (x, y, z).
pub fn voxel_downsample(xyz: &[f32], leaf: f32) -> Vec<f32> {
    let n = xyz.len() / 3;
    let inv = 1.0f64 / leaf as f64;
    let mut map: BTreeMap<(i64, i64, i64), ([f64; 3], u32)> = BTreeMap::new();
    for i in 0..n {
        let p = pt(xyz, i);
        if !finite(&p) {
            continue;
        }
        let key = (
            (p[0] as f64 * inv).floor() as i64,
            (p[1] as f64 * inv).floor() as i64,
            (p[2] as f64 * inv).floor() as i64,
        );
        let e = map.entry(key).or_insert(([0.0; 3], 0));
        e.0[0] += p[0] as f64;
        e.0[1] += p[1] as f64;
        e.0[2] += p[2] as f64;
        e.1 += 1;
    }
    let mut out = Vec::with_capacity(map.len() * 3);
    for (_, (s, c)) in map {
        let c = c as f64;
        out.push((s[0] / c) as f32);
        out.push((s[1] / c) as f32);
        out.push((s[2] / c) as f32);
    }
    out
}

/// Statistical outlier removal: keep[i] = 1 when the mean distance to the k nearest neighbours is
/// within mean + std_mul * stddev over the cloud.
pub fn statistical_outlier(xyz: &[f32], k: usize, std_mul: f64, keep: &mut [u8]) {
    let n = xyz.len() / 3;
    let tree = KdTree::new(xyz);
    let mut mean = vec![f64::NAN; n];
    let mut buf = Vec::with_capacity(k + 1);
    let mut valid = 0usize;
    let (mut sum, mut sq) = (0.0f64, 0.0f64);
    for i in 0..n {
        let p = pt(xyz, i);
        if !finite(&p) {
            continue;
        }
        tree.knn(&p, k + 1, &mut buf);
        let mut acc = 0.0f64;
        let mut cnt = 0usize;
        let mut skipped = false;
        for &(d, j) in buf.iter() {
            if !skipped && j as usize == i {
                skipped = true;
                continue;
            }
            if cnt == k {
                break;
            }
            acc += (d as f64).sqrt();
            cnt += 1;
        }
        if cnt == 0 {
            continue;
        }
        let m = acc / cnt as f64;
        mean[i] = m;
        sum += m;
        sq += m * m;
        valid += 1;
    }
    if valid == 0 {
        keep.iter_mut().for_each(|k| *k = 0);
        return;
    }
    let mu = sum / valid as f64;
    let var = if valid > 1 { ((sq - sum * mu) / (valid as f64 - 1.0)).max(0.0) } else { 0.0 };
    let thr = mu + std_mul * var.sqrt();
    for i in 0..n {
        keep[i] = if mean[i].is_finite() && mean[i] <= thr { 1 } else { 0 };
    }
}

/// Radius outlier removal: keep points with at least `min_neighbors` other points within `radius`.
pub fn radius_outlier(xyz: &[f32], radius: f32, min_neighbors: usize, keep: &mut [u8]) {
    let n = xyz.len() / 3;
    let tree = KdTree::new(xyz);
    let mut buf = Vec::new();
    for i in 0..n {
        let p = pt(xyz, i);
        if !finite(&p) {
            keep[i] = 0;
            continue;
        }
        tree.radius(&p, radius, &mut buf);
        let others = buf.len().saturating_sub(1);
        keep[i] = if others >= min_neighbors { 1 } else { 0 };
    }
}

/// Euclidean clustering. Writes a label per point (-1 = noise / rejected) and returns the cluster count.
/// Clusters are numbered in order of their lowest point index.
pub fn euclidean_cluster(xyz: &[f32], tolerance: f32, min_size: usize, max_size: usize, labels: &mut [i32]) -> usize {
    let n = xyz.len() / 3;
    let tree = KdTree::new(xyz);
    let mut raw = vec![-1i32; n];
    let mut sizes: Vec<usize> = Vec::new();
    let mut queue = VecDeque::new();
    let mut buf = Vec::new();
    for seed in 0..n {
        if raw[seed] >= 0 || !finite(&pt(xyz, seed)) {
            continue;
        }
        let id = sizes.len() as i32;
        raw[seed] = id;
        let mut size = 1usize;
        queue.push_back(seed);
        while let Some(i) = queue.pop_front() {
            tree.radius(&pt(xyz, i), tolerance, &mut buf);
            for &(_, j) in buf.iter() {
                let j = j as usize;
                if raw[j] < 0 {
                    raw[j] = id;
                    size += 1;
                    queue.push_back(j);
                }
            }
        }
        sizes.push(size);
    }
    let mut remap = vec![-1i32; sizes.len()];
    let mut next = 0i32;
    for (c, &s) in sizes.iter().enumerate() {
        if s >= min_size && s <= max_size {
            remap[c] = next;
            next += 1;
        }
    }
    for i in 0..n {
        labels[i] = if raw[i] >= 0 { remap[raw[i] as usize] } else { -1 };
    }
    next as usize
}

/// xorshift64* — shared with the managed implementation so RANSAC is reproducible across both.
pub struct Rng(u64);

impl Rng {
    pub fn new(seed: u64) -> Rng {
        Rng(if seed == 0 { 0x9E37_79B9_7F4A_7C15 } else { seed })
    }

    pub fn next(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.0 = x;
        x.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }
}

/// RANSAC plane fit. Returns (inlier count, [a, b, c, d]) with a unit normal; fills `inliers`.
pub fn ransac_plane(xyz: &[f32], threshold: f64, iterations: usize, seed: u64, inliers: &mut [u8]) -> (usize, [f64; 4]) {
    let n = xyz.len() / 3;
    inliers.iter_mut().for_each(|v| *v = 0);
    if n < 3 {
        return (0, [0.0; 4]);
    }
    let mut rng = Rng::new(seed);
    let mut best = (0usize, [0.0f64; 4]);
    for _ in 0..iterations {
        let a = (rng.next() % n as u64) as usize;
        let b = (rng.next() % n as u64) as usize;
        let c = (rng.next() % n as u64) as usize;
        if a == b || b == c || a == c {
            continue;
        }
        let (p, q, r) = (pt(xyz, a), pt(xyz, b), pt(xyz, c));
        if !finite(&p) || !finite(&q) || !finite(&r) {
            continue;
        }
        let u = [(q[0] - p[0]) as f64, (q[1] - p[1]) as f64, (q[2] - p[2]) as f64];
        let v = [(r[0] - p[0]) as f64, (r[1] - p[1]) as f64, (r[2] - p[2]) as f64];
        let nx = u[1] * v[2] - u[2] * v[1];
        let ny = u[2] * v[0] - u[0] * v[2];
        let nz = u[0] * v[1] - u[1] * v[0];
        let len = (nx * nx + ny * ny + nz * nz).sqrt();
        if len < 1e-12 {
            continue;
        }
        let (nx, ny, nz) = (nx / len, ny / len, nz / len);
        let d = -(nx * p[0] as f64 + ny * p[1] as f64 + nz * p[2] as f64);
        let mut count = 0usize;
        for i in 0..n {
            let s = pt(xyz, i);
            if (nx * s[0] as f64 + ny * s[1] as f64 + nz * s[2] as f64 + d).abs() <= threshold {
                count += 1;
            }
        }
        if count > best.0 {
            best = (count, [nx, ny, nz, d]);
        }
    }
    let [nx, ny, nz, d] = best.1;
    if best.0 > 0 {
        for i in 0..n {
            let s = pt(xyz, i);
            if (nx * s[0] as f64 + ny * s[1] as f64 + nz * s[2] as f64 + d).abs() <= threshold {
                inliers[i] = 1;
            }
        }
    }
    best
}

/// Cyclic Jacobi eigen-decomposition of a symmetric n×n matrix (row-major). Returns eigenvalues and
/// column eigenvectors (row-major), unsorted.
pub fn jacobi_eigen(a: &mut [f64], n: usize) -> (Vec<f64>, Vec<f64>) {
    let mut v = vec![0.0; n * n];
    for i in 0..n {
        v[i * n + i] = 1.0;
    }
    for _sweep in 0..64 {
        let mut off = 0.0;
        for p in 0..n {
            for q in (p + 1)..n {
                off += a[p * n + q] * a[p * n + q];
            }
        }
        if off < 1e-30 {
            break;
        }
        for p in 0..n {
            for q in (p + 1)..n {
                let apq = a[p * n + q];
                if apq.abs() < 1e-300 {
                    continue;
                }
                let theta = (a[q * n + q] - a[p * n + p]) / (2.0 * apq);
                let t = theta.signum() / (theta.abs() + (theta * theta + 1.0).sqrt());
                let t = if theta == 0.0 { 1.0 } else { t };
                let c = 1.0 / (t * t + 1.0).sqrt();
                let s = t * c;
                for k in 0..n {
                    let akp = a[k * n + p];
                    let akq = a[k * n + q];
                    a[k * n + p] = c * akp - s * akq;
                    a[k * n + q] = s * akp + c * akq;
                }
                for k in 0..n {
                    let apk = a[p * n + k];
                    let aqk = a[q * n + k];
                    a[p * n + k] = c * apk - s * aqk;
                    a[q * n + k] = s * apk + c * aqk;
                }
                for k in 0..n {
                    let vkp = v[k * n + p];
                    let vkq = v[k * n + q];
                    v[k * n + p] = c * vkp - s * vkq;
                    v[k * n + q] = s * vkp + c * vkq;
                }
            }
        }
    }
    ((0..n).map(|i| a[i * n + i]).collect(), v)
}

/// Optimal rigid transform (rotation row-major 3×3, translation) aligning `src` onto `dst` pairs,
/// by Horn's closed-form quaternion method (the same least-squares optimum as SVD/Kabsch).
pub fn best_fit(src: &[[f64; 3]], dst: &[[f64; 3]]) -> ([f64; 9], [f64; 3]) {
    let m = src.len() as f64;
    let mut cs = [0.0; 3];
    let mut cd = [0.0; 3];
    for i in 0..src.len() {
        for a in 0..3 {
            cs[a] += src[i][a];
            cd[a] += dst[i][a];
        }
    }
    for a in 0..3 {
        cs[a] /= m;
        cd[a] /= m;
    }
    let mut s = [0.0f64; 9];
    for i in 0..src.len() {
        for r in 0..3 {
            for c in 0..3 {
                s[r * 3 + c] += (src[i][r] - cs[r]) * (dst[i][c] - cd[c]);
            }
        }
    }
    let (sxx, sxy, sxz, syx, syy, syz, szx, szy, szz) = (s[0], s[1], s[2], s[3], s[4], s[5], s[6], s[7], s[8]);
    let mut nm = [
        sxx + syy + szz, syz - szy, szx - sxz, sxy - syx,
        syz - szy, sxx - syy - szz, sxy + syx, szx + sxz,
        szx - sxz, sxy + syx, -sxx + syy - szz, syz + szy,
        sxy - syx, szx + sxz, syz + szy, -sxx - syy + szz,
    ];
    let (vals, vecs) = jacobi_eigen(&mut nm, 4);
    let mut best = 0;
    for i in 1..4 {
        if vals[i] > vals[best] {
            best = i;
        }
    }
    let (w, x, y, z) = (vecs[best], vecs[4 + best], vecs[8 + best], vecs[12 + best]);
    let r = [
        w * w + x * x - y * y - z * z, 2.0 * (x * y - w * z), 2.0 * (x * z + w * y),
        2.0 * (x * y + w * z), w * w - x * x + y * y - z * z, 2.0 * (y * z - w * x),
        2.0 * (x * z - w * y), 2.0 * (y * z + w * x), w * w - x * x - y * y + z * z,
    ];
    let t = [
        cd[0] - (r[0] * cs[0] + r[1] * cs[1] + r[2] * cs[2]),
        cd[1] - (r[3] * cs[0] + r[4] * cs[1] + r[5] * cs[2]),
        cd[2] - (r[6] * cs[0] + r[7] * cs[1] + r[8] * cs[2]),
    ];
    (r, t)
}

pub struct IcpResult {
    pub transform: [f64; 16],
    pub rmse: f64,
    pub iterations: u32,
    pub converged: bool,
    pub correspondences: usize,
}

/// Point-to-point ICP aligning `src` onto `dst`. The transform is row-major 4×4 (dst ← src).
pub fn icp(src: &[f32], dst: &[f32], max_iter: u32, max_corr: f32, tolerance: f64) -> IcpResult {
    let tree = KdTree::new(dst);
    let ns = src.len() / 3;
    let mut t = [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0f64];
    let mut res = IcpResult { transform: t, rmse: f64::INFINITY, iterations: 0, converged: false, correspondences: 0 };
    let max2 = max_corr * max_corr;
    let mut buf = Vec::with_capacity(1);
    let mut ps = Vec::with_capacity(ns);
    let mut pd = Vec::with_capacity(ns);
    for it in 0..max_iter {
        ps.clear();
        pd.clear();
        let mut err = 0.0f64;
        for i in 0..ns {
            let p = pt(src, i);
            if !finite(&p) {
                continue;
            }
            let (x, y, z) = (p[0] as f64, p[1] as f64, p[2] as f64);
            let q = [
                t[0] * x + t[1] * y + t[2] * z + t[3],
                t[4] * x + t[5] * y + t[6] * z + t[7],
                t[8] * x + t[9] * y + t[10] * z + t[11],
            ];
            let qf = [q[0] as f32, q[1] as f32, q[2] as f32];
            tree.knn(&qf, 1, &mut buf);
            if let Some(&(d, j)) = buf.first() {
                if d <= max2 {
                    let dp = tree.point(j as usize);
                    ps.push(q);
                    pd.push([dp[0] as f64, dp[1] as f64, dp[2] as f64]);
                    err += d as f64;
                }
            }
        }
        res.iterations = it + 1;
        res.correspondences = ps.len();
        if ps.len() < 3 {
            break;
        }
        res.rmse = (err / ps.len() as f64).sqrt();
        let (r, tr) = best_fit(&ps, &pd);
        // t = delta * t
        let mut next = [0.0f64; 16];
        for row in 0..3 {
            for col in 0..4 {
                next[row * 4 + col] = r[row * 3] * t[col] + r[row * 3 + 1] * t[4 + col] + r[row * 3 + 2] * t[8 + col]
                    + if col == 3 { tr[row] } else { 0.0 };
            }
        }
        next[15] = 1.0;
        t = next;
        res.transform = t;
        let angle = ((r[0] + r[4] + r[8] - 1.0) * 0.5).clamp(-1.0, 1.0).acos();
        let shift = (tr[0] * tr[0] + tr[1] * tr[1] + tr[2] * tr[2]).sqrt();
        if angle < tolerance && shift < tolerance {
            res.converged = true;
            break;
        }
    }
    res
}

/// Projects a planar scan into xyz (z = 0). Invalid ranges are skipped. Returns the point count.
pub fn project_scan(ranges: &[f32], angle_min: f32, angle_inc: f32, range_min: f32, range_max: f32, out: &mut [f32]) -> usize {
    let mut n = 0;
    for (i, &r) in ranges.iter().enumerate() {
        if !r.is_finite() || r < range_min || r > range_max {
            continue;
        }
        let a = angle_min as f64 + i as f64 * angle_inc as f64;
        out[3 * n] = (r as f64 * a.cos()) as f32;
        out[3 * n + 1] = (r as f64 * a.sin()) as f32;
        out[3 * n + 2] = 0.0;
        n += 1;
    }
    n
}

pub struct GridSpec {
    pub width: i64,
    pub height: i64,
    pub resolution: f64,
    pub origin_x: f64,
    pub origin_y: f64,
}

pub struct LogOdds {
    pub free: f32,
    pub occupied: f32,
    pub min: f32,
    pub max: f32,
}

/// Ray-traces each endpoint from the sensor cell (Bresenham), adding `free` to traversed cells and
/// `occupied` to the endpoint cell, clamped to [min, max]. Cells outside the grid are skipped.
pub fn occupancy_update(grid: &mut [f32], g: &GridSpec, sensor: (f64, f64), points_xy: &[f32], lo: &LogOdds) {
    let cell = |x: f64, y: f64| -> (i64, i64) {
        (((x - g.origin_x) / g.resolution).floor() as i64, ((y - g.origin_y) / g.resolution).floor() as i64)
    };
    let inside = |cx: i64, cy: i64| cx >= 0 && cy >= 0 && cx < g.width && cy < g.height;
    let mut add = |cx: i64, cy: i64, v: f32| {
        if inside(cx, cy) {
            let k = (cy * g.width + cx) as usize;
            grid[k] = (grid[k] + v).clamp(lo.min, lo.max);
        }
    };
    let (sx, sy) = cell(sensor.0, sensor.1);
    for i in 0..points_xy.len() / 2 {
        let (px, py) = (points_xy[2 * i] as f64, points_xy[2 * i + 1] as f64);
        if !px.is_finite() || !py.is_finite() {
            continue;
        }
        let (ex, ey) = cell(px, py);
        let (mut x, mut y) = (sx, sy);
        let dx = (ex - sx).abs();
        let dy = -(ey - sy).abs();
        let stx = if sx < ex { 1 } else { -1 };
        let sty = if sy < ey { 1 } else { -1 };
        let mut err = dx + dy;
        while !(x == ex && y == ey) {
            add(x, y, lo.free);
            let e2 = 2 * err;
            if e2 >= dy {
                err += dy;
                x += stx;
            }
            if e2 <= dx {
                err += dx;
                y += sty;
            }
        }
        add(ex, ey, lo.occupied);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn voxel_merges_points_in_a_cell() {
        let xyz = [0.1, 0.1, 0.1, 0.2, 0.2, 0.2, 1.5, 0.0, 0.0];
        let out = voxel_downsample(&xyz, 1.0);
        assert_eq!(out.len(), 6);
        assert!((out[0] - 0.15).abs() < 1e-6);
        assert!((out[3] - 1.5).abs() < 1e-6);
    }

    #[test]
    fn sor_removes_far_outlier() {
        let mut xyz = Vec::new();
        for i in 0..10 {
            for j in 0..10 {
                xyz.extend_from_slice(&[i as f32 * 0.1, j as f32 * 0.1, 0.0]);
            }
        }
        xyz.extend_from_slice(&[50.0, 50.0, 50.0]);
        let mut keep = vec![0u8; xyz.len() / 3];
        statistical_outlier(&xyz, 8, 1.0, &mut keep);
        assert_eq!(keep[100], 0);
        assert!(keep[..100].iter().filter(|&&k| k == 1).count() > 90);
    }

    #[test]
    fn clustering_separates_blobs() {
        let mut xyz = Vec::new();
        for i in 0..20 {
            xyz.extend_from_slice(&[i as f32 * 0.05, 0.0, 0.0]);
        }
        for i in 0..20 {
            xyz.extend_from_slice(&[10.0 + i as f32 * 0.05, 0.0, 0.0]);
        }
        xyz.extend_from_slice(&[100.0, 0.0, 0.0]);
        let mut labels = vec![0; xyz.len() / 3];
        let c = euclidean_cluster(&xyz, 0.1, 5, 1000, &mut labels);
        assert_eq!(c, 2);
        assert_eq!(labels[0], 0);
        assert_eq!(labels[25], 1);
        assert_eq!(labels[40], -1);
    }

    #[test]
    fn ransac_finds_ground_plane() {
        let mut xyz = Vec::new();
        for i in 0..30 {
            for j in 0..30 {
                xyz.extend_from_slice(&[i as f32 * 0.1, j as f32 * 0.1, 0.5]);
            }
        }
        for i in 0..50 {
            xyz.extend_from_slice(&[i as f32 * 0.03, 1.0, 2.0 + i as f32 * 0.1]);
        }
        let mut inl = vec![0u8; xyz.len() / 3];
        let (count, c) = ransac_plane(&xyz, 0.01, 200, 42, &mut inl);
        assert_eq!(count, 900);
        assert!(c[2].abs() > 0.999);
        assert!((c[3] / c[2] + 0.5).abs() < 1e-6);
    }

    #[test]
    fn icp_recovers_small_motion() {
        let mut dst = Vec::new();
        for i in 0..15 {
            for j in 0..15 {
                let (x, y) = (i as f32 * 0.2, j as f32 * 0.2);
                dst.extend_from_slice(&[x, y, (x * 1.3).sin() * 0.3 + (y * 0.7).cos() * 0.2]);
            }
        }
        let (a, tx, ty) = (0.05f64, 0.1f64, -0.05f64);
        let mut src = Vec::new();
        for i in 0..dst.len() / 3 {
            let (x, y, z) = (dst[3 * i] as f64, dst[3 * i + 1] as f64, dst[3 * i + 2] as f64);
            // src = inverse motion of dst
            let (xs, ys) = (x - tx, y - ty);
            src.extend_from_slice(&[(a.cos() * xs + a.sin() * ys) as f32, (-a.sin() * xs + a.cos() * ys) as f32, z as f32]);
        }
        let r = icp(&src, &dst, 50, 1.0, 1e-7);
        assert!(r.rmse < 1e-3, "rmse {}", r.rmse);
        assert!((r.transform[3] - tx).abs() < 1e-3);
        assert!((r.transform[7] - ty).abs() < 1e-3);
    }

    #[test]
    fn scan_projection_and_occupancy() {
        let ranges = [1.0f32, f32::INFINITY, 2.0];
        let mut out = [0f32; 9];
        let n = project_scan(&ranges, 0.0, std::f32::consts::FRAC_PI_2, 0.1, 10.0, &mut out);
        assert_eq!(n, 2);
        assert!((out[0] - 1.0).abs() < 1e-6);
        assert!((out[3] + 2.0).abs() < 1e-5);

        let g = GridSpec { width: 10, height: 10, resolution: 1.0, origin_x: 0.0, origin_y: 0.0 };
        let mut grid = vec![0f32; 100];
        occupancy_update(&mut grid, &g, (0.5, 0.5), &[5.5, 0.5], &LogOdds { free: -0.4, occupied: 0.85, min: -2.0, max: 3.5 });
        assert_eq!(grid[0], -0.4);
        assert_eq!(grid[4], -0.4);
        assert_eq!(grid[5], 0.85);
        assert_eq!(grid[6], 0.0);
    }
}
