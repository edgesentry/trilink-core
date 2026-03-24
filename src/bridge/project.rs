use crate::{CameraIntrinsics, DepthMap, HeightMap, PointCloud, Transform4x4};

/// Project a 3D world-space point cloud onto a 2D depth map.
///
/// # Arguments
/// - `cloud`  — LiDAR/ToF sweep in world space
/// - `pose`   — camera pose `T_cam_to_world` (row-major 4×4, same convention as `unproject`)
/// - `k`      — pinhole camera intrinsics
/// - `width`  — output image width in pixels
/// - `height` — output image height in pixels
///
/// # Math
/// 1. **World → camera** (rigid-body inverse, no matrix inversion):
///    `P_camera = Rᵀ · (P_world − t)` where `R` and `t` come from the top-left 3×3
///    and last column of `pose`.
/// 2. **Behind-camera cull**: skip points with `Zc ≤ 0`.
/// 3. **Pinhole projection**: `u = fx·(Xc/Zc) + cx`, `v = fy·(Yc/Zc) + cy`.
/// 4. **Bounds cull**: skip pixels outside `[0, width) × [0, height)`.
/// 5. **Z-buffer**: keep the nearest point (smallest `Zc`) per pixel.
///
/// Pixels with no projected point are initialised to `f32::INFINITY`.
pub fn project_to_depth_map(
    cloud: &PointCloud,
    pose: &Transform4x4,
    k: &CameraIntrinsics,
    width: u32,
    height: u32,
) -> DepthMap {
    let mut data = vec![f32::INFINITY; (width * height) as usize];
    let m = &pose.matrix;

    // Translation column from the pose matrix.
    let tx = m[3] as f64;
    let ty = m[7] as f64;
    let tz = m[11] as f64;

    for p in &cloud.points {
        let xw = p.x as f64 - tx;
        let yw = p.y as f64 - ty;
        let zw = p.z as f64 - tz;

        // Rᵀ · (P_world − t): rows of Rᵀ are columns of R.
        let xc = m[0] as f64 * xw + m[4] as f64 * yw + m[8] as f64 * zw;
        let yc = m[1] as f64 * xw + m[5] as f64 * yw + m[9] as f64 * zw;
        let zc = m[2] as f64 * xw + m[6] as f64 * yw + m[10] as f64 * zw;

        // Cull behind-camera points.
        if zc <= 0.0 {
            continue;
        }

        let u = k.fx * (xc / zc) + k.cx;
        let v = k.fy * (yc / zc) + k.cy;

        // Cull out-of-bounds pixels.
        if u < 0.0 || v < 0.0 || u >= width as f64 || v >= height as f64 {
            continue;
        }

        let idx = (v as u32 * width + u as u32) as usize;

        // Z-buffer: keep nearest.
        let zc32 = zc as f32;
        if zc32 < data[idx] {
            data[idx] = zc32;
        }
    }

    DepthMap { width, height, data }
}

/// Project a 3D world-space point cloud onto a top-down height map.
///
/// Operates entirely in world space — no camera pose is involved.
///
/// # Arguments
/// - `cloud`        — LiDAR/ToF sweep in world space
/// - `origin_x`     — world X of the grid's top-left corner
/// - `origin_y`     — world Y of the grid's top-left corner
/// - `resolution_m` — cell size in metres
/// - `cols`         — grid width in cells
/// - `rows`         — grid height in cells
///
/// # Math
/// For each point:
/// ```text
/// col = floor((P_world.x − origin_x) / resolution_m)
/// row = floor((P_world.y − origin_y) / resolution_m)
/// ```
/// Points outside `[0, cols) × [0, rows)` are silently skipped.
/// Per cell, the **maximum Z** is retained (detects protrusions above the design surface).
/// Cells with no point are initialised to `f32::NAN`.
pub fn project_to_height_map(
    cloud: &PointCloud,
    origin_x: f32,
    origin_y: f32,
    resolution_m: f32,
    cols: u32,
    rows: u32,
) -> HeightMap {
    let mut data = vec![f32::NAN; (cols * rows) as usize];

    for p in &cloud.points {
        let col_f = (p.x - origin_x) / resolution_m;
        let row_f = (p.y - origin_y) / resolution_m;

        if col_f < 0.0 || row_f < 0.0 {
            continue;
        }

        let col = col_f.floor() as u32;
        let row = row_f.floor() as u32;

        if col >= cols || row >= rows {
            continue;
        }

        let idx = (row * cols + col) as usize;
        if data[idx].is_nan() || p.z > data[idx] {
            data[idx] = p.z;
        }
    }

    HeightMap { origin_x, origin_y, resolution_m, cols, rows, data }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{CameraIntrinsics, Point3D, PointCloud, Transform4x4};

    fn identity() -> Transform4x4 {
        Transform4x4::identity()
    }

    fn k() -> CameraIntrinsics {
        // 1080p-style: fx=fy=800, cx=960, cy=540
        CameraIntrinsics { fx: 800.0, fy: 800.0, cx: 960.0, cy: 540.0 }
    }

    fn cloud(points: Vec<Point3D>) -> PointCloud {
        PointCloud { capture_ts_us: 0, points, intensities: None }
    }

    // Single point on the principal axis → pixel at (cx, cy), depth = Zc.
    #[test]
    fn principal_axis_point_lands_at_principal_pixel() {
        // Identity pose: camera frame == world frame.
        // P_world = (0, 0, 3) → Xc=0, Yc=0, Zc=3
        // u = 800*(0/3)+960 = 960, v = 800*(0/3)+540 = 540
        let dm = project_to_depth_map(
            &cloud(vec![Point3D { x: 0.0, y: 0.0, z: 3.0 }]),
            &identity(),
            &k(),
            1920,
            1080,
        );
        assert!(dm.has_depth(960, 540));
        assert!((dm.get(960, 540) - 3.0).abs() < 1e-4);
    }

    // Point at +45° horizontal (Xc = Zc) → pixel at u = cx + fx.
    #[test]
    fn point_at_45_degrees_horizontal() {
        // P_world = (2, 0, 2) → u = 800*(2/2)+960 = 1760, v = 540
        let dm = project_to_depth_map(
            &cloud(vec![Point3D { x: 2.0, y: 0.0, z: 2.0 }]),
            &identity(),
            &k(),
            1920,
            1080,
        );
        assert!(dm.has_depth(1760, 540));
        assert!((dm.get(1760, 540) - 2.0).abs() < 1e-4);
    }

    // Point behind the camera (Zc ≤ 0) → not present in depth map.
    #[test]
    fn behind_camera_point_is_culled() {
        // P_world = (0, 0, -1) → Zc = -1 ≤ 0, must be discarded.
        let dm = project_to_depth_map(
            &cloud(vec![Point3D { x: 0.0, y: 0.0, z: -1.0 }]),
            &identity(),
            &k(),
            1920,
            1080,
        );
        assert!(!dm.has_depth(960, 540));
    }

    // Two points projecting to the same pixel: nearest wins (Z-buffer).
    #[test]
    fn z_buffer_keeps_nearest_point() {
        let dm = project_to_depth_map(
            &cloud(vec![
                Point3D { x: 0.0, y: 0.0, z: 5.0 },
                Point3D { x: 0.0, y: 0.0, z: 2.0 },
            ]),
            &identity(),
            &k(),
            1920,
            1080,
        );
        assert!((dm.get(960, 540) - 2.0).abs() < 1e-4, "nearest point should win");
    }

    // Point outside image bounds → not present in depth map.
    #[test]
    fn out_of_bounds_point_is_culled() {
        // P_world = (1000, 0, 1) → u = 800*1000+960 >> 1920, out of bounds.
        let dm = project_to_depth_map(
            &cloud(vec![Point3D { x: 1000.0, y: 0.0, z: 1.0 }]),
            &identity(),
            &k(),
            1920,
            1080,
        );
        // No pixel should have finite depth.
        assert!(dm.data.iter().all(|v| v.is_infinite()));
    }

    // Identity pose + known world point → correct (u, v, Zc) end-to-end.
    #[test]
    fn identity_pose_known_world_point_end_to_end() {
        // P_world = (1.6, 1.08, 2) with k(fx=800,cx=960,fy=800,cy=540):
        // u = 800*(1.6/2)+960 = 1600, v = 800*(1.08/2)+540 = 972  → but 972 > 1080? no, 972 < 1080 ✓
        // Wait: v = 800*(1.08/2)+540 = 800*0.54+540 = 432+540 = 972
        let dm = project_to_depth_map(
            &cloud(vec![Point3D { x: 1.6, y: 1.08, z: 2.0 }]),
            &identity(),
            &k(),
            1920,
            1080,
        );
        assert!(dm.has_depth(1600, 972));
        assert!((dm.get(1600, 972) - 2.0).abs() < 1e-4);
    }

    // Empty point cloud → all pixels INFINITY.
    #[test]
    fn empty_cloud_produces_all_infinity() {
        let dm = project_to_depth_map(&cloud(vec![]), &identity(), &k(), 4, 4);
        assert!(dm.data.iter().all(|v| v.is_infinite()));
    }

    // --- project_to_height_map ---

    fn hm_cloud(points: Vec<Point3D>) -> PointCloud {
        PointCloud { capture_ts_us: 0, points, intensities: None }
    }

    // Single point → correct cell index.
    #[test]
    fn single_point_correct_cell_index() {
        // origin=(0,0), resolution=0.1: point (0.25, 0.35, 1.0)
        // col = floor(0.25/0.1) = 2, row = floor(0.35/0.1) = 3
        let hm = project_to_height_map(
            &hm_cloud(vec![Point3D { x: 0.25, y: 0.35, z: 1.0 }]),
            0.0, 0.0, 0.1, 10, 10,
        );
        assert!(!hm.data[3 * 10 + 2].is_nan(), "cell (col=2, row=3) should have data");
        assert!((hm.data[3 * 10 + 2] - 1.0).abs() < 1e-5);
    }

    // Single point → correct height value.
    #[test]
    fn single_point_correct_height() {
        let hm = project_to_height_map(
            &hm_cloud(vec![Point3D { x: 0.0, y: 0.0, z: 2.5 }]),
            0.0, 0.0, 1.0, 5, 5,
        );
        assert!((hm.data[0] - 2.5).abs() < 1e-5);
    }

    // Two points in same cell, different Z → max Z wins.
    #[test]
    fn two_points_same_cell_max_z_wins() {
        let hm = project_to_height_map(
            &hm_cloud(vec![
                Point3D { x: 0.0, y: 0.0, z: 1.0 },
                Point3D { x: 0.0, y: 0.0, z: 3.0 },
            ]),
            0.0, 0.0, 1.0, 5, 5,
        );
        assert!((hm.data[0] - 3.0).abs() < 1e-5, "max Z should win");
    }

    // Point outside grid bounds → no panic, no data written.
    #[test]
    fn out_of_bounds_point_no_panic() {
        let hm = project_to_height_map(
            &hm_cloud(vec![Point3D { x: 999.0, y: 999.0, z: 1.0 }]),
            0.0, 0.0, 1.0, 5, 5,
        );
        assert!(hm.data.iter().all(|v| v.is_nan()), "out-of-bounds point must not touch grid");
    }

    // Empty cloud → all cells NaN.
    #[test]
    fn empty_cloud_all_nan() {
        let hm = project_to_height_map(&hm_cloud(vec![]), 0.0, 0.0, 0.1, 4, 4);
        assert!(hm.data.iter().all(|v| v.is_nan()));
    }

    // resolution_m boundary: point exactly on a cell edge uses floor (maps to lower cell).
    #[test]
    fn cell_edge_uses_floor() {
        // Point at x=1.0 exactly, resolution=1.0, origin=0.0
        // col = floor(1.0 / 1.0) = 1
        let hm = project_to_height_map(
            &hm_cloud(vec![Point3D { x: 1.0, y: 0.0, z: 5.0 }]),
            0.0, 0.0, 1.0, 5, 5,
        );
        assert!(!hm.data[0 * 5 + 1].is_nan(), "point at exact edge should go to col=1");
        assert!((hm.data[0 * 5 + 1] - 5.0).abs() < 1e-5);
    }
}
