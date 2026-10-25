//! C ABI of ros2net_native. All functions are panic-safe: a panic is caught and reported as
//! `STATUS_PANIC`. Buffers are flat little-endian f32 xyz triples unless stated otherwise.

pub mod cloud;
pub mod kdtree;

use std::panic::{catch_unwind, AssertUnwindSafe};
use std::slice;

pub const ABI_VERSION: u32 = 1;

pub const STATUS_OK: i32 = 0;
pub const STATUS_NULL: i32 = -1;
pub const STATUS_INVALID: i32 = -2;
pub const STATUS_BUFFER_TOO_SMALL: i32 = -3;
pub const STATUS_PANIC: i32 = -99;

fn guard<F: FnOnce() -> i32>(f: F) -> i32 {
    catch_unwind(AssertUnwindSafe(f)).unwrap_or(STATUS_PANIC)
}

unsafe fn sl<'a, T>(p: *const T, n: usize) -> Option<&'a [T]> {
    if n == 0 {
        Some(&[])
    } else if p.is_null() {
        None
    } else {
        Some(slice::from_raw_parts(p, n))
    }
}

unsafe fn sl_mut<'a, T>(p: *mut T, n: usize) -> Option<&'a mut [T]> {
    if n == 0 {
        Some(&mut [])
    } else if p.is_null() {
        None
    } else {
        Some(slice::from_raw_parts_mut(p, n))
    }
}

/// ABI version; bumped on any breaking change of these signatures.
#[no_mangle]
pub extern "C" fn ros2net_abi_version() -> u32 {
    ABI_VERSION
}

/// Library version as a NUL-terminated UTF-8 string with static lifetime.
#[no_mangle]
pub extern "C" fn ros2net_version() -> *const u8 {
    concat!(env!("CARGO_PKG_VERSION"), "\0").as_ptr()
}

/// Voxel-grid downsampling. Writes centroids to `out` (capacity `out_cap` points); `out_n` receives
/// the centroid count even when the buffer is too small.
#[no_mangle]
pub unsafe extern "C" fn ros2net_voxel_downsample(xyz: *const f32, n: usize, leaf: f32, out: *mut f32, out_cap: usize, out_n: *mut usize) -> i32 {
    guard(|| {
        let (Some(xyz), Some(out)) = (sl(xyz, n * 3), sl_mut(out, out_cap * 3)) else { return STATUS_NULL };
        if out_n.is_null() {
            return STATUS_NULL;
        }
        if !(leaf > 0.0) || !leaf.is_finite() {
            return STATUS_INVALID;
        }
        let r = cloud::voxel_downsample(xyz, leaf);
        *out_n = r.len() / 3;
        if r.len() > out.len() {
            return STATUS_BUFFER_TOO_SMALL;
        }
        out[..r.len()].copy_from_slice(&r);
        STATUS_OK
    })
}

/// Statistical outlier removal; `keep[i]` = 1 for inliers.
#[no_mangle]
pub unsafe extern "C" fn ros2net_statistical_outlier(xyz: *const f32, n: usize, k: usize, std_mul: f64, keep: *mut u8) -> i32 {
    guard(|| {
        let (Some(xyz), Some(keep)) = (sl(xyz, n * 3), sl_mut(keep, n)) else { return STATUS_NULL };
        if k == 0 {
            return STATUS_INVALID;
        }
        cloud::statistical_outlier(xyz, k, std_mul, keep);
        STATUS_OK
    })
}

/// Radius outlier removal; `keep[i]` = 1 when at least `min_neighbors` other points lie within `radius`.
#[no_mangle]
pub unsafe extern "C" fn ros2net_radius_outlier(xyz: *const f32, n: usize, radius: f32, min_neighbors: usize, keep: *mut u8) -> i32 {
    guard(|| {
        let (Some(xyz), Some(keep)) = (sl(xyz, n * 3), sl_mut(keep, n)) else { return STATUS_NULL };
        if !(radius > 0.0) {
            return STATUS_INVALID;
        }
        cloud::radius_outlier(xyz, radius, min_neighbors, keep);
        STATUS_OK
    })
}

/// Builds a k-d tree (copies the points). Returns null on failure. Free with `ros2net_kdtree_free`.
#[no_mangle]
pub unsafe extern "C" fn ros2net_kdtree_build(xyz: *const f32, n: usize) -> *mut kdtree::KdTree {
    catch_unwind(AssertUnwindSafe(|| match sl(xyz, n * 3) {
        Some(p) => Box::into_raw(Box::new(kdtree::KdTree::new(p))),
        None => std::ptr::null_mut(),
    }))
    .unwrap_or(std::ptr::null_mut())
}

#[no_mangle]
pub unsafe extern "C" fn ros2net_kdtree_free(tree: *mut kdtree::KdTree) {
    if !tree.is_null() {
        drop(Box::from_raw(tree));
    }
}

/// k nearest neighbours of `query` (3 floats). Writes up to `k` indices and squared distances.
#[no_mangle]
pub unsafe extern "C" fn ros2net_kdtree_knn(tree: *const kdtree::KdTree, query: *const f32, k: usize, out_idx: *mut u32, out_d2: *mut f32, out_found: *mut usize) -> i32 {
    guard(|| {
        if tree.is_null() || out_found.is_null() {
            return STATUS_NULL;
        }
        let (Some(q), Some(oi), Some(od)) = (sl(query, 3), sl_mut(out_idx, k), sl_mut(out_d2, k)) else { return STATUS_NULL };
        let mut buf = Vec::with_capacity(k);
        (*tree).knn(&[q[0], q[1], q[2]], k, &mut buf);
        for (j, &(d, i)) in buf.iter().enumerate() {
            oi[j] = i;
            od[j] = d;
        }
        *out_found = buf.len();
        STATUS_OK
    })
}

/// Indices (and squared distances) within `radius` of `query`. `out_n` receives the full count.
#[no_mangle]
pub unsafe extern "C" fn ros2net_kdtree_radius(tree: *const kdtree::KdTree, query: *const f32, radius: f32, out_idx: *mut u32, out_d2: *mut f32, cap: usize, out_n: *mut usize) -> i32 {
    guard(|| {
        if tree.is_null() || out_n.is_null() {
            return STATUS_NULL;
        }
        let (Some(q), Some(oi), Some(od)) = (sl(query, 3), sl_mut(out_idx, cap), sl_mut(out_d2, cap)) else { return STATUS_NULL };
        let mut buf = Vec::new();
        (*tree).radius(&[q[0], q[1], q[2]], radius, &mut buf);
        *out_n = buf.len();
        if buf.len() > cap {
            return STATUS_BUFFER_TOO_SMALL;
        }
        for (j, &(d, i)) in buf.iter().enumerate() {
            oi[j] = i;
            od[j] = d;
        }
        STATUS_OK
    })
}

/// Euclidean clustering; labels per point (-1 = not in an accepted cluster). `out_clusters` receives the count.
#[no_mangle]
pub unsafe extern "C" fn ros2net_euclidean_cluster(xyz: *const f32, n: usize, tolerance: f32, min_size: usize, max_size: usize, labels: *mut i32, out_clusters: *mut usize) -> i32 {
    guard(|| {
        let (Some(xyz), Some(labels)) = (sl(xyz, n * 3), sl_mut(labels, n)) else { return STATUS_NULL };
        if out_clusters.is_null() {
            return STATUS_NULL;
        }
        if !(tolerance > 0.0) {
            return STATUS_INVALID;
        }
        *out_clusters = cloud::euclidean_cluster(xyz, tolerance, min_size, max_size, labels);
        STATUS_OK
    })
}

/// RANSAC plane fit. `coeffs` receives [a, b, c, d] (unit normal), `inliers` a 0/1 mask, `out_inliers` the count.
#[no_mangle]
pub unsafe extern "C" fn ros2net_ransac_plane(xyz: *const f32, n: usize, threshold: f64, iterations: usize, seed: u64, coeffs: *mut f64, inliers: *mut u8, out_inliers: *mut usize) -> i32 {
    guard(|| {
        let (Some(xyz), Some(c), Some(inl)) = (sl(xyz, n * 3), sl_mut(coeffs, 4), sl_mut(inliers, n)) else { return STATUS_NULL };
        if out_inliers.is_null() {
            return STATUS_NULL;
        }
        let (count, best) = cloud::ransac_plane(xyz, threshold, iterations, seed, inl);
        c.copy_from_slice(&best);
        *out_inliers = count;
        STATUS_OK
    })
}

/// Point-to-point ICP aligning `src` onto `dst`. `transform` receives a row-major 4×4 (f64).
#[no_mangle]
pub unsafe extern "C" fn ros2net_icp(src: *const f32, ns: usize, dst: *const f32, nd: usize, max_iter: u32, max_corr: f32, tolerance: f64, transform: *mut f64, out_rmse: *mut f64, out_iterations: *mut u32, out_converged: *mut u8) -> i32 {
    guard(|| {
        let (Some(s), Some(d), Some(t)) = (sl(src, ns * 3), sl(dst, nd * 3), sl_mut(transform, 16)) else { return STATUS_NULL };
        if out_rmse.is_null() || out_iterations.is_null() || out_converged.is_null() {
            return STATUS_NULL;
        }
        if nd == 0 {
            return STATUS_INVALID;
        }
        let r = cloud::icp(s, d, max_iter, max_corr, tolerance);
        t.copy_from_slice(&r.transform);
        *out_rmse = r.rmse;
        *out_iterations = r.iterations;
        *out_converged = r.converged as u8;
        STATUS_OK
    })
}

/// Projects a planar laser scan into xyz. `out` must hold `n` points; `out_n` receives the valid count.
#[no_mangle]
pub unsafe extern "C" fn ros2net_scan_project(ranges: *const f32, n: usize, angle_min: f32, angle_inc: f32, range_min: f32, range_max: f32, out: *mut f32, out_n: *mut usize) -> i32 {
    guard(|| {
        let (Some(r), Some(o)) = (sl(ranges, n), sl_mut(out, n * 3)) else { return STATUS_NULL };
        if out_n.is_null() {
            return STATUS_NULL;
        }
        *out_n = cloud::project_scan(r, angle_min, angle_inc, range_min, range_max, o);
        STATUS_OK
    })
}

/// Log-odds occupancy update by Bresenham ray tracing from the sensor to each xy endpoint.
#[no_mangle]
pub unsafe extern "C" fn ros2net_occupancy_update(
    grid: *mut f32, width: i32, height: i32, resolution: f64, origin_x: f64, origin_y: f64,
    sensor_x: f64, sensor_y: f64, points_xy: *const f32, n: usize,
    l_free: f32, l_occ: f32, l_min: f32, l_max: f32,
) -> i32 {
    guard(|| {
        if width <= 0 || height <= 0 || !(resolution > 0.0) {
            return STATUS_INVALID;
        }
        let (Some(g), Some(p)) = (sl_mut(grid, (width as usize) * (height as usize)), sl(points_xy, n * 2)) else { return STATUS_NULL };
        let spec = cloud::GridSpec { width: width as i64, height: height as i64, resolution, origin_x, origin_y };
        cloud::occupancy_update(g, &spec, (sensor_x, sensor_y), p, &cloud::LogOdds { free: l_free, occupied: l_occ, min: l_min, max: l_max });
        STATUS_OK
    })
}
