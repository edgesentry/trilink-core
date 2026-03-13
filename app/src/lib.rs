pub mod bridge;
pub mod egress;
pub mod infer;

// Re-export core types so callers can use a single import path.
pub use trilink_core::{
    BBox2D, CameraIntrinsics, Detection, InspectionPacket, Point3D, Transform4x4, TriError,
};
