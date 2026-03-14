use crate::{BBox2D, CameraIntrinsics, Point3D, Transform4x4};

/// Unproject a bounding box centre to a 3D world-space point.
///
/// # Arguments
/// - `bbox`            — pixel-space bounding box; its centre is used
/// - `depth_m`         — depth from ToF sensor, if available
/// - `fallback_depth_m`— used when `depth_m` is `None`
/// - `k`               — pinhole camera intrinsics (`fx`, `fy`, `cx`, `cy`)
/// - `pose`            — robot pose at capture time (row-major 4×4, world frame)
///
/// # Math
/// 1. `(u, v) = bbox.center()`
/// 2. Normalise: `x_norm = (u - cx) / fx`, `y_norm = (v - cy) / fy`
/// 3. Scale: `P_camera = (x_norm·d, y_norm·d, d)`
/// 4. Transform: `P_world = pose · [P_camera, 1]ᵀ`
pub fn unproject(
    bbox: &BBox2D,
    depth_m: Option<f32>,
    fallback_depth_m: f32,
    k: &CameraIntrinsics,
    pose: &Transform4x4,
) -> Point3D {
    let (u, v) = bbox.center();
    let d = depth_m.unwrap_or(fallback_depth_m) as f64;

    // Step 2 — normalise pixel to camera-space ray direction.
    let xc = (u - k.cx) / k.fx * d;
    let yc = (v - k.cy) / k.fy * d;
    let zc = d;

    // Step 4 — apply the 4×4 pose transform (row-major).
    // P_world = pose · [xc, yc, zc, 1]ᵀ
    let m = &pose.matrix;
    let xw = m[0] as f64 * xc + m[1] as f64 * yc + m[2]  as f64 * zc + m[3]  as f64;
    let yw = m[4] as f64 * xc + m[5] as f64 * yc + m[6]  as f64 * zc + m[7]  as f64;
    let zw = m[8] as f64 * xc + m[9] as f64 * yc + m[10] as f64 * zc + m[11] as f64;

    Point3D { x: xw as f32, y: yw as f32, z: zw as f32 }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn identity() -> Transform4x4 {
        Transform4x4::identity()
    }

    fn k_simple() -> CameraIntrinsics {
        // fx=fy=1, cx=cy=0 → normalised coords equal pixel coords
        CameraIntrinsics { fx: 1.0, fy: 1.0, cx: 0.0, cy: 0.0 }
    }

    fn k_standard() -> CameraIntrinsics {
        CameraIntrinsics { fx: 800.0, fy: 800.0, cx: 960.0, cy: 540.0 }
    }

    fn bbox(u0: f32, v0: f32, u1: f32, v1: f32) -> BBox2D {
        BBox2D { u0, v0, u1, v1 }
    }

    // Identity pose + known intrinsics → deterministic world coordinate.
    #[test]
    fn identity_pose_known_intrinsics() {
        // With k_simple (fx=fy=1, cx=cy=0) and identity pose:
        //   bbox centre = (2.0, 3.0)
        //   P_camera = (2.0·d, 3.0·d, d)
        //   P_world  = P_camera  (identity transform)
        let p = unproject(&bbox(0.0, 0.0, 4.0, 6.0), None, 1.0, &k_simple(), &identity());
        assert!((p.x - 2.0).abs() < 1e-5);
        assert!((p.y - 3.0).abs() < 1e-5);
        assert!((p.z - 1.0).abs() < 1e-5);
    }

    // Depth scales world coordinate proportionally.
    #[test]
    fn depth_scales_world_coordinate() {
        let p1 = unproject(&bbox(0.0, 0.0, 4.0, 6.0), Some(1.0), 1.0, &k_simple(), &identity());
        let p2 = unproject(&bbox(0.0, 0.0, 4.0, 6.0), Some(2.0), 1.0, &k_simple(), &identity());
        assert!((p2.x - 2.0 * p1.x).abs() < 1e-4);
        assert!((p2.y - 2.0 * p1.y).abs() < 1e-4);
        assert!((p2.z - 2.0 * p1.z).abs() < 1e-4);
    }

    // Edge pixel (u=0, v=0) → correct corner coordinate.
    #[test]
    fn edge_pixel_top_left_corner() {
        // With k_standard (fx=fy=800, cx=960, cy=540) and a zero-size bbox at (0,0):
        //   u=0, v=0 → x_norm=(0-960)/800=-1.2, y_norm=(0-540)/800=-0.675
        //   at d=1: P_camera=(-1.2, -0.675, 1.0)
        let p = unproject(&bbox(0.0, 0.0, 0.0, 0.0), Some(1.0), 1.0, &k_standard(), &identity());
        assert!((p.x - (-1.2_f32)).abs() < 1e-4);
        assert!((p.y - (-0.675_f32)).abs() < 1e-4);
        assert!((p.z - 1.0_f32).abs() < 1e-4);
    }

    // Fallback depth produces non-None result.
    #[test]
    fn fallback_depth_produces_result() {
        // depth_m = None → must use fallback_depth_m = 3.0
        let p = unproject(&bbox(0.0, 0.0, 4.0, 6.0), None, 3.0, &k_simple(), &identity());
        assert!((p.z - 3.0).abs() < 1e-4);
    }

    // --- Doc-grounded tests (math.md + assumptions.md + scenarios.md) ---

    // math.md §Unprojection step 1: pixel at principal point (cx, cy) normalises to (0, 0),
    // so P_camera = (0, 0, d) — robot looks straight ahead.
    // Intrinsics are the 1080p values given in math.md numerical ranges table.
    #[test]
    fn principal_point_pixel_gives_straight_ahead_ray() {
        // cx=960, cy=540 (1080p), depth=2.0 m (assumptions.md default fallback)
        // u=cx, v=cy → x_norm=0, y_norm=0 → P_camera=(0, 0, 2)
        // identity pose → P_world=(0, 0, 2)
        let p = unproject(&bbox(960.0, 540.0, 960.0, 540.0), Some(2.0), 2.0, &k_standard(), &identity());
        assert!(p.x.abs() < 1e-4, "x should be 0 at principal point");
        assert!(p.y.abs() < 1e-4, "y should be 0 at principal point");
        assert!((p.z - 2.0).abs() < 1e-4);
    }

    // assumptions.md §Damage Localization Accuracy: default fallback_depth_m = 2.0 m.
    // Verify that None depth uses 2.0 and not some other value.
    #[test]
    fn default_fallback_depth_is_2m() {
        // principal point, identity → z must equal the fallback
        let p = unproject(&bbox(960.0, 540.0, 960.0, 540.0), None, 2.0, &k_standard(), &identity());
        assert!((p.z - 2.0).abs() < 1e-4, "z should equal fallback depth 2.0 m");
    }

    // math.md §Numerical Ranges: depth range is 0.5–5.0 m.
    // Verify both extremes produce correct z output.
    #[test]
    fn depth_range_minimum_0_5m() {
        let p = unproject(&bbox(960.0, 540.0, 960.0, 540.0), Some(0.5), 2.0, &k_standard(), &identity());
        assert!((p.z - 0.5).abs() < 1e-4);
    }

    #[test]
    fn depth_range_maximum_5m() {
        let p = unproject(&bbox(960.0, 540.0, 960.0, 540.0), Some(5.0), 2.0, &k_standard(), &identity());
        assert!((p.z - 5.0).abs() < 1e-4);
    }

    // math.md §Pose Transform: P_world = T · [P_camera, 1]ᵀ.
    // Robot translated +3 m along Z and +1 m along X in world frame.
    // At principal point with d=2.0: P_camera=(0,0,2) → P_world=(1+0, 0, 3+2)=(1,0,5).
    #[test]
    fn translated_pose_offsets_world_coords() {
        // Translation matrix: identity rotation, translation (+1, 0, +3)
        #[rustfmt::skip]
        let pose = Transform4x4 { matrix: [
            1.0, 0.0, 0.0, 1.0,   // row 0: tx = 1.0
            0.0, 1.0, 0.0, 0.0,   // row 1: ty = 0.0
            0.0, 0.0, 1.0, 3.0,   // row 2: tz = 3.0
            0.0, 0.0, 0.0, 1.0,
        ]};
        let p = unproject(&bbox(960.0, 540.0, 960.0, 540.0), Some(2.0), 2.0, &k_standard(), &pose);
        assert!((p.x - 1.0).abs() < 1e-4, "world x should include robot translation");
        assert!(p.y.abs() < 1e-4);
        assert!((p.z - 5.0).abs() < 1e-4, "world z = robot_tz + camera_zc");
    }

    // math.md §Unprojection step 2: x_norm = (u - cx) / fx.
    // Right-side pixel at u = cx + fx → x_norm = 1.0 → Xc = depth.
    // scenarios.md: vehicle side-scan at ~2 m, fx=800 from typical range.
    #[test]
    fn one_focal_length_right_gives_45_degree_ray() {
        // u = cx + fx = 960 + 800 = 1760, v = cy = 540
        // x_norm = (1760 - 960) / 800 = 1.0, y_norm = 0
        // at d=2.0: P_camera=(2.0, 0, 2.0) → P_world same with identity
        let p = unproject(&bbox(1760.0, 540.0, 1760.0, 540.0), Some(2.0), 2.0, &k_standard(), &identity());
        assert!((p.x - 2.0).abs() < 1e-3, "Xc should equal Zc at ±45° (x_norm=1)");
        assert!(p.y.abs() < 1e-3);
        assert!((p.z - 2.0).abs() < 1e-3);
    }

    // assumptions.md §Damage Localization Accuracy: ~1 cm accuracy at 2 m with ToF.
    // Verify float arithmetic error is well within 1 mm (0.001 m) — much tighter than
    // the 1 cm system-level target, ruling out implementation precision as a contributor.
    #[test]
    fn arithmetic_error_is_sub_millimetre() {
        // Typical vehicle side-scan: fx=800, d=2.0, detection at image centre.
        let p = unproject(&bbox(960.0, 540.0, 960.0, 540.0), Some(2.0), 2.0, &k_standard(), &identity());
        assert!(p.x.abs() < 0.001, "float error in x must be < 1 mm");
        assert!(p.y.abs() < 0.001, "float error in y must be < 1 mm");
        assert!((p.z - 2.0).abs() < 0.001, "float error in z must be < 1 mm");
    }
}
