pub mod mock;

use crate::{Transform4x4, error::TriError};

/// A frame emitted by the robot platform: raw JPEG + pose at shutter time.
#[derive(Debug, Clone)]
pub struct RobotFrame {
    /// Microseconds since UNIX epoch when the shutter opened.
    pub capture_ts_us: u64,
    /// Robot pose at shutter time from the SLAM subsystem.
    pub pose: Transform4x4,
    /// JPEG-encoded image bytes.
    pub jpeg: Vec<u8>,
    /// ToF depth at the frame center, if available.
    pub depth_m: Option<f32>,
}

/// Trait implemented by any source of robot frames (real hardware or mock).
pub trait RobotSource: Send {
    /// Returns the next available frame, blocking until one is ready.
    fn next_frame(&mut self) -> Result<RobotFrame, TriError>;
}
