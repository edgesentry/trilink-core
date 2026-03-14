pub mod buffer;
pub mod bridge;
pub mod error;
pub mod ingress;

pub use error::TriError;

/// One complete detection event: image capture with pose, intrinsics, and inference results.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct FusionPacket {
    /// Image capture time (microseconds since UNIX epoch).
    pub capture_ts_us: u64,
    /// Platform pose at `capture_ts_us`, looked up from the pose buffer.
    pub pose: Transform4x4,
    /// Static camera calibration parameters.
    pub camera_k: CameraIntrinsics,
    /// Raw JPEG frame that was sent to the inference service.
    pub image_jpeg: Vec<u8>,
    /// Inference results with world coordinates resolved.
    pub detections: Vec<Detection>,
}

/// A single object detection with optional world-space location.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct Detection {
    /// Class label returned by the inference service (e.g. `"person"`, `"crack"`, `"vehicle"`).
    pub class: String,
    pub confidence: f32,
    /// Pixel-space bounding box from the inference service.
    pub bbox: BBox2D,
    /// 3D world position after unprojection. `None` until resolved.
    pub world_pos: Option<Point3D>,
    /// Depth from ToF sensor if available.
    pub depth_m: Option<f32>,
}

/// Pinhole camera calibration parameters.
#[derive(Debug, Clone, Copy, serde::Serialize, serde::Deserialize)]
pub struct CameraIntrinsics {
    pub fx: f64,
    pub fy: f64,
    pub cx: f64,
    pub cy: f64,
}

/// Row-major homogeneous 4×4 transform (platform pose in world frame).
#[derive(Debug, Clone, Copy, serde::Serialize, serde::Deserialize)]
pub struct Transform4x4 {
    pub matrix: [f32; 16],
}

impl Transform4x4 {
    /// Identity transform — platform at origin, no rotation.
    pub fn identity() -> Self {
        #[rustfmt::skip]
        let m = [
            1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0,
        ];
        Self { matrix: m }
    }

    /// Apply this transform to a camera-space point, returning a world-space [`Point3D`].
    ///
    /// Computes `P_world = self · [xc, yc, zc, 1]ᵀ` using the top three rows of the
    /// row-major 4×4 matrix (the homogeneous bottom row is not needed for affine transforms).
    pub fn transform_point(&self, xc: f64, yc: f64, zc: f64) -> Point3D {
        let m = &self.matrix;
        Point3D {
            x: (m[0] as f64 * xc + m[1] as f64 * yc + m[2] as f64 * zc + m[3] as f64) as f32,
            y: (m[4] as f64 * xc + m[5] as f64 * yc + m[6] as f64 * zc + m[7] as f64) as f32,
            z: (m[8] as f64 * xc + m[9] as f64 * yc + m[10] as f64 * zc + m[11] as f64) as f32,
        }
    }
}

/// 3D world-space point.
#[derive(Debug, Clone, Copy, serde::Serialize, serde::Deserialize)]
pub struct Point3D {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

/// Pixel-space bounding box (inclusive min, exclusive max convention).
///
/// Fields are `f32` to accept both integer pixel coords and sub-pixel float
/// coords returned by most inference APIs (e.g. YOLO-style services).
#[derive(Debug, Clone, Copy, serde::Serialize, serde::Deserialize)]
pub struct BBox2D {
    pub u0: f32,
    pub v0: f32,
    pub u1: f32,
    pub v1: f32,
}

impl BBox2D {
    /// Returns the center pixel `(u, v)`.
    pub fn center(&self) -> (f64, f64) {
        (
            (self.u0 + self.u1) as f64 / 2.0,
            (self.v0 + self.v1) as f64 / 2.0,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // --- BBox2D::center ---

    #[test]
    fn bbox_center_symmetric() {
        let bbox = BBox2D { u0: 10.0, v0: 20.0, u1: 30.0, v1: 40.0 };
        assert_eq!(bbox.center(), (20.0, 30.0));
    }

    #[test]
    fn bbox_center_fractional() {
        // Odd-width box: centre falls on a half-pixel
        let bbox = BBox2D { u0: 0.0, v0: 0.0, u1: 1.0, v1: 3.0 };
        assert_eq!(bbox.center(), (0.5, 1.5));
    }

    #[test]
    fn bbox_center_degenerate_zero_size() {
        let bbox = BBox2D { u0: 5.0, v0: 7.0, u1: 5.0, v1: 7.0 };
        assert_eq!(bbox.center(), (5.0, 7.0));
    }

    // --- Transform4x4::transform_point ---

    #[test]
    fn transform_point_identity_returns_input() {
        let p = Transform4x4::identity().transform_point(1.0, 2.0, 3.0);
        assert!((p.x - 1.0).abs() < 1e-6);
        assert!((p.y - 2.0).abs() < 1e-6);
        assert!((p.z - 3.0).abs() < 1e-6);
    }

    #[test]
    fn transform_point_translation_offsets_correctly() {
        // Translation-only matrix: identity rotation, tx=10, ty=20, tz=30
        #[rustfmt::skip]
        let t = Transform4x4 { matrix: [
            1.0, 0.0, 0.0, 10.0,
            0.0, 1.0, 0.0, 20.0,
            0.0, 0.0, 1.0, 30.0,
            0.0, 0.0, 0.0,  1.0,
        ]};
        let p = t.transform_point(1.0, 2.0, 3.0);
        assert!((p.x - 11.0).abs() < 1e-5);
        assert!((p.y - 22.0).abs() < 1e-5);
        assert!((p.z - 33.0).abs() < 1e-5);
    }

    // --- Transform4x4::identity ---

    #[test]
    fn identity_diagonal_is_one() {
        let t = Transform4x4::identity();
        assert_eq!(t.matrix[0],  1.0); // [0,0]
        assert_eq!(t.matrix[5],  1.0); // [1,1]
        assert_eq!(t.matrix[10], 1.0); // [2,2]
        assert_eq!(t.matrix[15], 1.0); // [3,3]
    }

    #[test]
    fn identity_off_diagonal_is_zero() {
        let t = Transform4x4::identity();
        for (i, &v) in t.matrix.iter().enumerate() {
            let row = i / 4;
            let col = i % 4;
            if row != col {
                assert_eq!(v, 0.0, "matrix[{i}] (row={row}, col={col}) expected 0");
            }
        }
    }

    // --- Serde roundtrips ---

    #[test]
    fn fusion_packet_roundtrip() {
        let packet = FusionPacket {
            capture_ts_us: 42_000_000,
            pose: Transform4x4::identity(),
            camera_k: CameraIntrinsics { fx: 800.0, fy: 800.0, cx: 960.0, cy: 540.0 },
            image_jpeg: vec![0xff, 0xd8, 0xff, 0xd9],
            detections: vec![Detection {
                class: "scratch".to_string(),
                confidence: 0.92,
                bbox: BBox2D { u0: 100.0, v0: 200.0, u1: 150.0, v1: 250.0 },
                world_pos: Some(Point3D { x: 1.0, y: 2.0, z: 3.0 }),
                depth_m: Some(2.5),
            }],
        };
        let json = serde_json::to_string(&packet).unwrap();
        let decoded: FusionPacket = serde_json::from_str(&json).unwrap();
        assert_eq!(decoded.capture_ts_us, 42_000_000);
        assert_eq!(decoded.detections.len(), 1);
        assert_eq!(decoded.detections[0].class, "scratch");
        assert!((decoded.detections[0].confidence - 0.92).abs() < 1e-5);
        let wp = decoded.detections[0].world_pos.unwrap();
        assert_eq!(wp.x, 1.0);
        assert_eq!(wp.y, 2.0);
        assert_eq!(wp.z, 3.0);
    }

    #[test]
    fn detection_with_no_world_pos_roundtrip() {
        let d = Detection {
            class: "dent".to_string(),
            confidence: 0.75,
            bbox: BBox2D { u0: 0.0, v0: 0.0, u1: 10.0, v1: 10.0 },
            world_pos: None,
            depth_m: None,
        };
        let json = serde_json::to_string(&d).unwrap();
        let decoded: Detection = serde_json::from_str(&json).unwrap();
        assert!(decoded.world_pos.is_none());
        assert!(decoded.depth_m.is_none());
        assert_eq!(decoded.class, "dent");
    }
}
