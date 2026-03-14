pub mod mock;

use crate::{Transform4x4, error::TriError};

/// A frame emitted by a sensor platform: raw JPEG + pose at shutter time.
#[derive(Debug, Clone)]
pub struct SensorFrame {
    /// Microseconds since UNIX epoch when the shutter opened.
    pub capture_ts_us: u64,
    /// Platform pose at shutter time from the localisation subsystem.
    pub pose: Transform4x4,
    /// JPEG-encoded image bytes.
    pub jpeg: Vec<u8>,
    /// ToF depth at the frame center, if available.
    pub depth_m: Option<f32>,
}

/// Trait implemented by any source of sensor frames (real hardware or mock).
///
/// `next_frame` is `async` so implementations can await hardware SDKs or
/// network streams without blocking a tokio worker thread.
///
/// Note: `async fn` in traits is not yet object-safe (`dyn FrameSource` is
/// not supported). Use a concrete type or a newtype wrapper if dynamic
/// dispatch is needed.
pub trait FrameSource: Send {
    /// Returns the next available frame, yielding until one is ready.
    async fn next_frame(&mut self) -> Result<SensorFrame, TriError>;
}
