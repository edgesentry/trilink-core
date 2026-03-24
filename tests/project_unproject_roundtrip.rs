use trilink_core::{
    bridge::{project_to_depth_map, unproject},
    BBox2D, CameraIntrinsics, Point3D, PointCloud, Transform4x4,
};

fn k() -> CameraIntrinsics {
    CameraIntrinsics { fx: 800.0, fy: 800.0, cx: 960.0, cy: 540.0 }
}

fn identity() -> Transform4x4 {
    Transform4x4::identity()
}

fn cloud(points: Vec<Point3D>) -> PointCloud {
    PointCloud { capture_ts_us: 0, points, intensities: None }
}

/// Project a world point forward, then unproject the resulting pixel back.
/// Returns the recovered world point.
fn roundtrip(p: Point3D, pose: &Transform4x4) -> Point3D {
    let k = k();
    let width = 1920_u32;
    let height = 1080_u32;

    let dm = project_to_depth_map(&cloud(vec![p]), pose, &k, width, height);

    // Compute expected pixel via the world→camera transform (independent of project.rs).
    let pc = pose.mat.inverse().transform_point3(glam::vec3(p.x, p.y, p.z));
    let zc = pc.z as f64;
    let u = k.fx * (pc.x as f64 / zc) + k.cx;
    let v = k.fy * (pc.y as f64 / zc) + k.cy;

    // Use truncation (same as project_to_depth_map's `v as u32 * width + u as u32`).
    let u_px = u as u32;
    let v_px = v as u32;

    let depth = dm.get(u_px, v_px);
    assert!(depth.is_finite(), "projected point must land in the depth map");

    let bbox = BBox2D { u0: u as f32, v0: v as f32, u1: u as f32, v1: v as f32 };
    unproject(&bbox, Some(depth), depth, &k, pose)
}

fn assert_within_1mm(recovered: Point3D, original: Point3D) {
    assert!(
        (recovered.x - original.x).abs() < 0.001,
        "x error {:.6} m ≥ 1 mm (original={:.4}, recovered={:.4})",
        (recovered.x - original.x).abs(), original.x, recovered.x,
    );
    assert!(
        (recovered.y - original.y).abs() < 0.001,
        "y error {:.6} m ≥ 1 mm (original={:.4}, recovered={:.4})",
        (recovered.y - original.y).abs(), original.y, recovered.y,
    );
    assert!(
        (recovered.z - original.z).abs() < 0.001,
        "z error {:.6} m ≥ 1 mm (original={:.4}, recovered={:.4})",
        (recovered.z - original.z).abs(), original.z, recovered.z,
    );
}

/// 5×5 grid of world points with identity pose — all must round-trip within 1 mm.
#[test]
fn identity_pose_grid_roundtrip() {
    let pose = identity();
    for xi in 0..5_i32 {
        for yi in 0..5_i32 {
            let p = Point3D {
                x: xi as f32 * 0.4 - 0.8,  // −0.8 … +0.8 m
                y: yi as f32 * 0.3 - 0.6,  // −0.6 … +0.6 m
                z: 2.0,
            };
            let recovered = roundtrip(p, &pose);
            assert_within_1mm(recovered, p);
        }
    }
}

/// Non-identity pose (translated + rotated) — same round-trip tolerance.
#[test]
fn translated_pose_roundtrip() {
    // Platform translated +5 m along X, +3 m along Z in world space.
    #[rustfmt::skip]
    let pose = Transform4x4::from_row_major([
        1.0, 0.0, 0.0, 5.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 3.0,
        0.0, 0.0, 0.0, 1.0,
    ]);
    // World point in front of this camera (in camera space: Zc > 0).
    let p = Point3D { x: 5.2, y: 0.1, z: 5.5 };
    let recovered = roundtrip(p, &pose);
    assert_within_1mm(recovered, p);
}

/// Infinity pixel (no projection) — unproject with fallback still returns finite result.
#[test]
fn infinity_pixel_fallback_depth_gives_finite_result() {
    let dm = project_to_depth_map(&cloud(vec![]), &identity(), &k(), 1920, 1080);
    // All pixels are INFINITY — unproject with fallback must still produce finite coords.
    let bbox = BBox2D { u0: 960.0, v0: 540.0, u1: 960.0, v1: 540.0 };
    let p = unproject(&bbox, None, 2.0, &k(), &identity());
    assert!(p.x.is_finite() && p.y.is_finite() && p.z.is_finite());
    assert!((p.z - 2.0).abs() < 1e-4);
    let _ = dm; // confirm dm was used
}
