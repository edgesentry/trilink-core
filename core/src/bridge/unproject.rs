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

    fn bbox(u0: u32, v0: u32, u1: u32, v1: u32) -> BBox2D {
        BBox2D { u0, v0, u1, v1 }
    }

    // Identity pose + known intrinsics → deterministic world coordinate.
    #[test]
    fn identity_pose_known_intrinsics() {
        // With k_simple (fx=fy=1, cx=cy=0) and identity pose:
        //   bbox centre = (2.0, 3.0)
        //   P_camera = (2.0·d, 3.0·d, d)
        //   P_world  = P_camera  (identity transform)
        let p = unproject(&bbox(0, 0, 4, 6), None, 1.0, &k_simple(), &identity());
        assert!((p.x - 2.0).abs() < 1e-5);
        assert!((p.y - 3.0).abs() < 1e-5);
        assert!((p.z - 1.0).abs() < 1e-5);
    }

    // Depth scales world coordinate proportionally.
    #[test]
    fn depth_scales_world_coordinate() {
        let p1 = unproject(&bbox(0, 0, 4, 6), Some(1.0), 1.0, &k_simple(), &identity());
        let p2 = unproject(&bbox(0, 0, 4, 6), Some(2.0), 1.0, &k_simple(), &identity());
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
        let p = unproject(&bbox(0, 0, 0, 0), Some(1.0), 1.0, &k_standard(), &identity());
        assert!((p.x - (-1.2_f32)).abs() < 1e-4);
        assert!((p.y - (-0.675_f32)).abs() < 1e-4);
        assert!((p.z - 1.0_f32).abs() < 1e-4);
    }

    // Fallback depth produces non-None result.
    #[test]
    fn fallback_depth_produces_result() {
        // depth_m = None → must use fallback_depth_m = 3.0
        let p = unproject(&bbox(0, 0, 4, 6), None, 3.0, &k_simple(), &identity());
        assert!((p.z - 3.0).abs() < 1e-4);
    }
}
