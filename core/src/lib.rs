pub mod buffer;
pub mod bridge;
pub mod error;
pub mod ingress;

pub use error::TriError;

/// Unified output: one damage event with full spatial context.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct InspectionPacket {
    /// Image capture time (microseconds since UNIX epoch).
    pub capture_ts_us: u64,
    /// Robot pose at `capture_ts_us`, looked up from the pose buffer.
    pub pose: Transform4x4,
    /// Static camera calibration parameters.
    pub camera_k: CameraIntrinsics,
    /// Raw JPEG frame that was sent to the inference service.
    pub image_jpeg: Vec<u8>,
    /// Inference results with world coordinates resolved.
    pub detections: Vec<Detection>,
}

/// A single damage detection with optional world-space location.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct Detection {
    /// Damage class label (e.g. `"scratch"`, `"dent"`, `"crack"`).
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

/// Row-major homogeneous 4×4 transform (robot pose in world frame).
#[derive(Debug, Clone, Copy, serde::Serialize, serde::Deserialize)]
pub struct Transform4x4 {
    pub matrix: [f32; 16],
}

impl Transform4x4 {
    /// Identity transform — robot at origin, no rotation.
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
}

/// 3D world-space point.
#[derive(Debug, Clone, Copy, serde::Serialize, serde::Deserialize)]
pub struct Point3D {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

/// Pixel-space bounding box (inclusive min, exclusive max convention).
#[derive(Debug, Clone, Copy, serde::Serialize, serde::Deserialize)]
pub struct BBox2D {
    pub u0: u32,
    pub v0: u32,
    pub u1: u32,
    pub v1: u32,
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
