use super::{RobotFrame, RobotSource};
use crate::{Transform4x4, error::TriError};

/// Deterministic robot source for testing.
///
/// Produces frames at a fixed interval with:
/// - Monotonically increasing timestamps starting at `base_ts_us`
/// - A sine-wave Z-rotation pose to simulate robot turning
/// - A 1×1 pixel white JPEG (minimal valid JPEG)
/// - A fixed ToF depth
pub struct MockSource {
    base_ts_us: u64,
    step_us: u64,
    depth_m: f32,
    frame_index: u64,
    limit: Option<u64>,
}

impl MockSource {
    pub fn new(base_ts_us: u64, fps: u32, depth_m: f32) -> Self {
        Self {
            base_ts_us,
            step_us: 1_000_000 / fps as u64,
            depth_m,
            frame_index: 0,
            limit: None,
        }
    }

    /// Stop after `n` frames (useful in tests).
    pub fn with_limit(mut self, n: u64) -> Self {
        self.limit = Some(n);
        self
    }

    fn make_pose(frame_index: u64) -> Transform4x4 {
        // Rotate around Z axis: angle = frame_index * 0.1 rad
        let angle = (frame_index as f64 * 0.1) as f32;
        let (s, c) = (angle.sin(), angle.cos());
        #[rustfmt::skip]
        let matrix = [
            c, -s, 0.0, 0.0,
            s,  c, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0,
        ];
        Transform4x4 { matrix }
    }

    /// Minimal valid 1×1 white JPEG (24 bytes).
    fn blank_jpeg() -> Vec<u8> {
        // A 1x1 white pixel JPEG generated offline and embedded as bytes.
        vec![
            0xff, 0xd8, 0xff, 0xe0, 0x00, 0x10, 0x4a, 0x46, 0x49, 0x46, 0x00, 0x01, 0x01, 0x00,
            0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0xff, 0xdb, 0x00, 0x43, 0x00, 0x08, 0x06, 0x06,
            0x07, 0x06, 0x05, 0x08, 0x07, 0x07, 0x07, 0x09, 0x09, 0x08, 0x0a, 0x0c, 0x14, 0x0d,
            0x0c, 0x0b, 0x0b, 0x0c, 0x19, 0x12, 0x13, 0x0f, 0x14, 0x1d, 0x1a, 0x1f, 0x1e, 0x1d,
            0x1a, 0x1c, 0x1c, 0x20, 0x24, 0x2e, 0x27, 0x20, 0x22, 0x2c, 0x23, 0x1c, 0x1c, 0x28,
            0x37, 0x29, 0x2c, 0x30, 0x31, 0x34, 0x34, 0x34, 0x1f, 0x27, 0x39, 0x3d, 0x38, 0x32,
            0x3c, 0x2e, 0x33, 0x34, 0x32, 0xff, 0xc0, 0x00, 0x0b, 0x08, 0x00, 0x01, 0x00, 0x01,
            0x01, 0x01, 0x11, 0x00, 0xff, 0xc4, 0x00, 0x1f, 0x00, 0x00, 0x01, 0x05, 0x01, 0x01,
            0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02,
            0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0xff, 0xc4, 0x00, 0xb5, 0x10,
            0x00, 0x02, 0x01, 0x03, 0x03, 0x02, 0x04, 0x03, 0x05, 0x05, 0x04, 0x04, 0x00, 0x00,
            0x01, 0x7d, 0x01, 0x02, 0x03, 0x00, 0x04, 0x11, 0x05, 0x12, 0x21, 0x31, 0x41, 0x06,
            0x13, 0x51, 0x61, 0x07, 0x22, 0x71, 0x14, 0x32, 0x81, 0x91, 0xa1, 0x08, 0x23, 0x42,
            0xb1, 0xc1, 0x15, 0x52, 0xd1, 0xf0, 0x24, 0x33, 0x62, 0x72, 0x82, 0x09, 0x0a, 0x16,
            0x17, 0x18, 0x19, 0x1a, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2a, 0x34, 0x35, 0x36, 0x37,
            0x38, 0x39, 0x3a, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4a, 0x53, 0x54, 0x55,
            0x56, 0x57, 0x58, 0x59, 0x5a, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x73,
            0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7a, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89,
            0x8a, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9a, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6,
            0xa7, 0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xc2,
            0xc3, 0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xd2, 0xd3, 0xd4, 0xd5, 0xd6, 0xd7,
            0xd8, 0xd9, 0xda, 0xe1, 0xe2, 0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea, 0xf1,
            0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8, 0xf9, 0xfa, 0xff, 0xda, 0x00, 0x08, 0x01,
            0x01, 0x00, 0x00, 0x3f, 0x00, 0xfb, 0xd2, 0x8a, 0x28, 0x03, 0xff, 0xd9,
        ]
    }
}

impl RobotSource for MockSource {
    fn next_frame(&mut self) -> Result<RobotFrame, TriError> {
        if let Some(limit) = self.limit {
            if self.frame_index >= limit {
                return Err(TriError::Io(std::io::Error::new(
                    std::io::ErrorKind::UnexpectedEof,
                    "MockSource limit reached",
                )));
            }
        }

        let capture_ts_us = self.base_ts_us + self.frame_index * self.step_us;
        let pose = Self::make_pose(self.frame_index);
        self.frame_index += 1;

        Ok(RobotFrame {
            capture_ts_us,
            pose,
            jpeg: Self::blank_jpeg(),
            depth_m: Some(self.depth_m),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn timestamps_are_monotonically_increasing() {
        let mut src = MockSource::new(1_000_000, 30, 2.0).with_limit(5);
        let mut prev_ts = 0u64;
        for _ in 0..5 {
            let frame = src.next_frame().unwrap();
            assert!(frame.capture_ts_us > prev_ts);
            prev_ts = frame.capture_ts_us;
        }
    }

    #[test]
    fn jpeg_bytes_are_non_empty() {
        let mut src = MockSource::new(0, 10, 1.5).with_limit(1);
        let frame = src.next_frame().unwrap();
        assert!(!frame.jpeg.is_empty());
        // JPEG magic bytes
        assert_eq!(&frame.jpeg[..2], &[0xff, 0xd8]);
    }

    #[test]
    fn limit_returns_error_after_n_frames() {
        let mut src = MockSource::new(0, 10, 1.0).with_limit(2);
        src.next_frame().unwrap();
        src.next_frame().unwrap();
        assert!(src.next_frame().is_err());
    }

    #[test]
    fn first_frame_timestamp_equals_base() {
        let base = 5_000_000u64;
        let mut src = MockSource::new(base, 30, 2.0).with_limit(1);
        let frame = src.next_frame().unwrap();
        assert_eq!(frame.capture_ts_us, base);
    }

    #[test]
    fn frame_interval_matches_fps() {
        let fps = 10u32;
        let mut src = MockSource::new(0, fps, 1.0).with_limit(3);
        let f0 = src.next_frame().unwrap();
        let f1 = src.next_frame().unwrap();
        let expected_step_us = 1_000_000 / fps as u64;
        assert_eq!(f1.capture_ts_us - f0.capture_ts_us, expected_step_us);
    }

    #[test]
    fn depth_matches_configured_value() {
        let mut src = MockSource::new(0, 30, 3.5).with_limit(1);
        let frame = src.next_frame().unwrap();
        assert_eq!(frame.depth_m, Some(3.5));
    }

    #[test]
    fn frame_zero_pose_is_identity_rotation() {
        // angle = 0 * 0.1 = 0 → cos=1, sin=0 → identity rotation
        let mut src = MockSource::new(0, 30, 1.0).with_limit(1);
        let frame = src.next_frame().unwrap();
        let m = frame.pose.matrix;
        assert!((m[0] - 1.0).abs() < 1e-6, "m[0,0] should be cos(0)=1");
        assert!((m[1] - 0.0).abs() < 1e-6, "m[0,1] should be -sin(0)=0");
        assert!((m[4] - 0.0).abs() < 1e-6, "m[1,0] should be sin(0)=0");
        assert!((m[5] - 1.0).abs() < 1e-6, "m[1,1] should be cos(0)=1");
    }
}
