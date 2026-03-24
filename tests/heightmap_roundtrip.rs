use trilink_core::{bridge::project_to_height_map, Point3D, PointCloud};

fn cloud(points: Vec<Point3D>) -> PointCloud {
    PointCloud { capture_ts_us: 0, points, intensities: None }
}

/// 10 known world points → height map → cell values match original Z.
#[test]
fn known_points_height_matches_original_z() {
    let resolution = 0.01_f32;
    let origin_x = 0.0_f32;
    let origin_y = 0.0_f32;
    let cols = 100_u32;
    let rows = 100_u32;

    let points: Vec<Point3D> = (0..10)
        .map(|i| Point3D {
            x: i as f32 * 0.05 + 0.005, // centres within cells: 0.005, 0.055, …
            y: i as f32 * 0.07 + 0.003,
            z: i as f32 * 0.1 + 0.5,
        })
        .collect();

    let hm = project_to_height_map(&cloud(points.clone()), origin_x, origin_y, resolution, cols, rows);

    for p in &points {
        let col = ((p.x - origin_x) / resolution).floor() as u32;
        let row = ((p.y - origin_y) / resolution).floor() as u32;
        let stored = hm.data[(row * cols + col) as usize];
        assert!(
            !stored.is_nan(),
            "cell ({col},{row}) must have data for point ({:.3},{:.3},{:.3})",
            p.x, p.y, p.z
        );
        assert!(
            (stored - p.z).abs() < 1e-5,
            "cell ({col},{row}) height {stored:.6} ≠ point z {:.6}",
            p.z
        );
    }
}

/// Two points in the same cell — max Z is stored.
#[test]
fn two_points_same_cell_max_z_stored() {
    let hm = project_to_height_map(
        &cloud(vec![
            Point3D { x: 0.5, y: 0.5, z: 1.0 },
            Point3D { x: 0.5, y: 0.5, z: 4.0 },
        ]),
        0.0, 0.0, 1.0, 10, 10,
    );
    // Both map to col=0, row=0.
    assert!((hm.data[0] - 4.0).abs() < 1e-5, "max Z must be stored, got {}", hm.data[0]);
}
