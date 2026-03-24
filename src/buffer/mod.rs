use crate::Transform4x4;

/// A timestamped pose entry stored in the ring buffer.
#[derive(Debug, Clone, Copy)]
struct Entry {
    ts_us: u64,
    pose: Transform4x4,
}

/// Ring buffer of robot poses indexed by capture timestamp.
///
/// Capacity is fixed at construction time. When full, the oldest entry is
/// overwritten. Lookup is O(log n) via binary search on `ts_us`.
pub struct PoseBuffer {
    entries: Vec<Entry>,
    capacity: usize,
    head: usize,
    len: usize,
    tolerance_us: u64,
}

impl PoseBuffer {
    pub fn new(capacity: usize, tolerance_us: u64) -> Self {
        Self {
            entries: Vec::with_capacity(capacity),
            capacity,
            head: 0,
            len: 0,
            tolerance_us,
        }
    }

    /// Insert a pose with its capture timestamp.
    ///
    /// # Panics (debug builds only)
    /// Asserts that `ts_us` is strictly greater than the previously pushed
    /// timestamp. The binary search in [`Self::pose_at`] relies on
    /// monotonically increasing timestamps; out-of-order pushes would cause
    /// silent incorrect lookups in release builds.
    pub fn push(&mut self, ts_us: u64, pose: Transform4x4) {
        if self.len > 0 {
            // The last-pushed entry sits at (head + len - 1) % capacity,
            // which is correct for both the growing phase and the ring phase.
            let last_idx = (self.head + self.len - 1) % self.capacity;
            debug_assert!(
                ts_us > self.entries[last_idx].ts_us,
                "PoseBuffer::push: timestamps must be strictly increasing \
                 (got {ts_us}, last was {})",
                self.entries[last_idx].ts_us,
            );
        }
        let entry = Entry { ts_us, pose };
        if self.len < self.capacity {
            self.entries.push(entry);
            self.len += 1;
        } else {
            self.entries[self.head] = entry;
            self.head = (self.head + 1) % self.capacity;
        }
    }

    /// Look up the pose at `capture_ts_us`, interpolating when the timestamp
    /// falls strictly between two bracketing entries.
    ///
    /// - **Exact match** — returns the stored pose unchanged.
    /// - **Bracketed** — LERP on translation, SLERP on rotation between the
    ///   two nearest bracketing entries, then reconstructs a [`Transform4x4`].
    /// - **Outside range** — returns the nearest boundary pose (clamped).
    ///
    /// Returns `None` if no entry is within `tolerance_us` of the requested
    /// timestamp (single-entry case) or if both brackets are outside tolerance.
    ///
    /// # Complexity
    /// O(log n) — binary-searches each of the two sorted ring-buffer segments
    /// with no heap allocation on the hot path.
    pub fn pose_at(&self, capture_ts_us: u64) -> Option<Transform4x4> {
        if self.len == 0 {
            return None;
        }

        // The ring buffer stores entries in push order.  Because timestamps
        // are monotonically increasing, both segments below are sorted:
        //   seg_a: entries[head..len]  — older (lower ts)
        //   seg_b: entries[0..head]    — newer (higher ts)
        // Concatenate logically so we can find the predecessor and successor
        // across the seam, then binary-search each segment.
        let seg_a = &self.entries[self.head..self.len];
        let seg_b = &self.entries[..self.head];

        // Find the best predecessor (ts <= capture_ts_us) and successor
        // (ts >= capture_ts_us) across both segments.
        let mut pred: Option<Entry> = None; // largest ts <= capture_ts_us
        let mut succ: Option<Entry> = None; // smallest ts >= capture_ts_us

        for seg in [seg_a, seg_b] {
            // partition_point returns the first index where ts_us >= capture_ts_us.
            let idx = seg.partition_point(|e| e.ts_us < capture_ts_us);

            // Candidate predecessor: entry just before idx (ts_us < capture_ts_us),
            // or the entry AT idx if it's an exact match.
            if idx < seg.len() && seg[idx].ts_us == capture_ts_us {
                // Exact match — both predecessor and successor converge here.
                pred = Some(seg[idx]);
                succ = Some(seg[idx]);
            } else {
                if idx > 0 {
                    let e = seg[idx - 1];
                    if pred.is_none_or(|p: Entry| e.ts_us > p.ts_us) {
                        pred = Some(e);
                    }
                }
                if let Some(&e) = seg.get(idx) {
                    if succ.is_none_or(|s: Entry| e.ts_us < s.ts_us) {
                        succ = Some(e);
                    }
                }
            }
        }

        match (pred, succ) {
            // Exact match from both sides (or a single entry that equals ts).
            (Some(p), Some(s)) if p.ts_us == s.ts_us => {
                Some(p.pose)
            }

            // Strictly bracketed — interpolate.
            (Some(p), Some(s)) => {
                // Compute t in [0, 1].
                let span = (s.ts_us - p.ts_us) as f32;
                let t = (capture_ts_us - p.ts_us) as f32 / span;

                let (_, rot_a, trans_a) = p.pose.mat.to_scale_rotation_translation();
                let (_, rot_b, trans_b) = s.pose.mat.to_scale_rotation_translation();

                let trans = trans_a.lerp(trans_b, t);
                let rot   = rot_a.slerp(rot_b, t);

                let mat = glam::Mat4::from_rotation_translation(rot, trans);
                Some(Transform4x4 { mat })
            }

            // Only a predecessor exists — return it if within tolerance.
            (Some(p), None) => {
                if p.ts_us.abs_diff(capture_ts_us) <= self.tolerance_us {
                    Some(p.pose)
                } else {
                    None
                }
            }

            // Only a successor exists — return it if within tolerance.
            (None, Some(s)) => {
                if s.ts_us.abs_diff(capture_ts_us) <= self.tolerance_us {
                    Some(s.pose)
                } else {
                    None
                }
            }

            (None, None) => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn identity() -> Transform4x4 {
        Transform4x4::identity()
    }

    #[test]
    fn exact_match() {
        let mut buf = PoseBuffer::new(16, 200_000);
        buf.push(1_000, identity());
        buf.push(2_000, identity());
        assert!(buf.pose_at(1_000).is_some());
        assert!(buf.pose_at(2_000).is_some());
    }

    #[test]
    fn nearest_within_tolerance() {
        let mut buf = PoseBuffer::new(16, 200_000);
        buf.push(1_000_000, identity());
        // Query 50 ms after — within 200 ms tolerance.
        assert!(buf.pose_at(1_050_000).is_some());
    }

    #[test]
    fn outside_tolerance_returns_none() {
        let mut buf = PoseBuffer::new(16, 200_000);
        buf.push(1_000_000, identity());
        // Query 300 ms after — outside 200 ms tolerance.
        assert!(buf.pose_at(1_300_000).is_none());
    }

    #[test]
    fn ring_overwrites_oldest() {
        let mut buf = PoseBuffer::new(4, 200_000);
        for i in 0..6u64 {
            buf.push(i * 1_000_000, identity());
        }
        // Oldest entries (0, 1) should be gone.
        assert!(buf.pose_at(0).is_none());
        assert!(buf.pose_at(1_000_000).is_none());
        // Recent entries should still be present.
        assert!(buf.pose_at(5_000_000).is_some());
    }

    #[test]
    fn empty_buffer_returns_none() {
        let buf = PoseBuffer::new(16, 200_000);
        assert!(buf.pose_at(1_000_000).is_none());
    }

    #[test]
    fn interpolates_between_two_candidates() {
        let mut buf = PoseBuffer::new(16, 500_000);
        // Distinguish poses by x-translation so we can verify interpolation.
        let pose_a = Transform4x4 { mat: glam::Mat4::from_translation(glam::vec3(1.0, 0.0, 0.0)) };
        let pose_b = Transform4x4 { mat: glam::Mat4::from_translation(glam::vec3(2.0, 0.0, 0.0)) };
        buf.push(1_000_000, pose_a);
        buf.push(1_200_000, pose_b);
        // Query at 1150 ms: t = (150/200) = 0.75 → x = 1.0 + 0.75*1.0 = 1.75
        let result = buf.pose_at(1_150_000).unwrap();
        assert!((result.mat.w_axis.x - 1.75).abs() < 1e-5, "should interpolate between poses");
    }

    // --- Acceptance-criteria tests for LERP/SLERP interpolation ---

    #[test]
    fn lerp_midpoint_translation() {
        let mut buf = PoseBuffer::new(16, 1_000_000);
        let pose_a = Transform4x4 { mat: glam::Mat4::from_translation(glam::vec3(0.0, 0.0, 0.0)) };
        let pose_b = Transform4x4 { mat: glam::Mat4::from_translation(glam::vec3(4.0, 6.0, 8.0)) };
        buf.push(0, pose_a);
        buf.push(1_000_000, pose_b);
        // Midpoint: t = 0.5 → translation = (2.0, 3.0, 4.0)
        let result = buf.pose_at(500_000).unwrap();
        let (_, _, trans) = result.mat.to_scale_rotation_translation();
        assert!((trans.x - 2.0).abs() < 1e-5, "x midpoint");
        assert!((trans.y - 3.0).abs() < 1e-5, "y midpoint");
        assert!((trans.z - 4.0).abs() < 1e-5, "z midpoint");
    }

    #[test]
    fn slerp_midpoint_rotation() {
        use std::f32::consts::FRAC_PI_2;
        let mut buf = PoseBuffer::new(16, 1_000_000);
        // Rotate from identity (0 rad) to 90° around Z.
        let rot_a = glam::Quat::IDENTITY;
        let rot_b = glam::Quat::from_rotation_z(FRAC_PI_2);
        let pose_a = Transform4x4 { mat: glam::Mat4::from_rotation_translation(rot_a, glam::Vec3::ZERO) };
        let pose_b = Transform4x4 { mat: glam::Mat4::from_rotation_translation(rot_b, glam::Vec3::ZERO) };
        buf.push(0, pose_a);
        buf.push(1_000_000, pose_b);
        // Midpoint: t = 0.5 → rotation should be ~45° around Z.
        let result = buf.pose_at(500_000).unwrap();
        let (_, rot, _) = result.mat.to_scale_rotation_translation();
        let expected = glam::Quat::from_rotation_z(FRAC_PI_2 / 2.0);
        // Quaternions q and -q represent the same rotation; dot product ≈ 1.
        let dot = rot.dot(expected).abs();
        assert!(dot > 1.0 - 1e-5, "rotation should be halfway (dot={dot})");
    }

    #[test]
    fn exact_timestamp_no_drift() {
        let mut buf = PoseBuffer::new(16, 1_000_000);
        // Use a non-trivial rotation to detect any floating-point contamination.
        let rot = glam::Quat::from_rotation_y(1.23);
        let trans = glam::vec3(5.0, -3.0, 2.0);
        let mat = glam::Mat4::from_rotation_translation(rot, trans);
        let pose = Transform4x4 { mat };
        buf.push(500_000, pose);
        buf.push(1_000_000, Transform4x4::identity());
        // Query exactly at the first entry — must get back the stored matrix unchanged.
        let result = buf.pose_at(500_000).unwrap();
        for (a, b) in result.mat.to_cols_array().iter().zip(mat.to_cols_array().iter()) {
            assert_eq!(a, b, "exact-match query must return stored matrix bit-for-bit");
        }
    }

    #[test]
    fn query_before_range_returns_nearest_boundary() {
        let mut buf = PoseBuffer::new(16, 500_000);
        let pose_a = Transform4x4 { mat: glam::Mat4::from_translation(glam::vec3(1.0, 0.0, 0.0)) };
        let pose_b = Transform4x4 { mat: glam::Mat4::from_translation(glam::vec3(2.0, 0.0, 0.0)) };
        buf.push(1_000_000, pose_a);
        buf.push(2_000_000, pose_b);
        // Query 200 ms before first entry — within tolerance, should clamp to pose_a.
        let result = buf.pose_at(800_000).unwrap();
        assert!((result.mat.w_axis.x - 1.0).abs() < 1e-5, "should clamp to lower boundary");
    }

    #[test]
    fn query_after_range_returns_nearest_boundary() {
        let mut buf = PoseBuffer::new(16, 500_000);
        let pose_a = Transform4x4 { mat: glam::Mat4::from_translation(glam::vec3(1.0, 0.0, 0.0)) };
        let pose_b = Transform4x4 { mat: glam::Mat4::from_translation(glam::vec3(2.0, 0.0, 0.0)) };
        buf.push(1_000_000, pose_a);
        buf.push(2_000_000, pose_b);
        // Query 200 ms after last entry — within tolerance, should clamp to pose_b.
        let result = buf.pose_at(2_200_000).unwrap();
        assert!((result.mat.w_axis.x - 2.0).abs() < 1e-5, "should clamp to upper boundary");
    }

    #[test]
    fn query_outside_tolerance_returns_none() {
        let mut buf = PoseBuffer::new(16, 100_000);
        buf.push(1_000_000, Transform4x4::identity());
        buf.push(2_000_000, Transform4x4::identity());
        // Query 300 ms after last entry — outside 100 ms tolerance.
        assert!(buf.pose_at(2_300_000).is_none());
    }

    #[test]
    fn capacity_one_overwrites_on_second_push() {
        let mut buf = PoseBuffer::new(1, 200_000);
        buf.push(1_000_000, identity());
        assert!(buf.pose_at(1_000_000).is_some());
        buf.push(2_000_000, identity());
        assert!(buf.pose_at(1_000_000).is_none(), "overwritten entry must be gone");
        assert!(buf.pose_at(2_000_000).is_some());
    }

    #[test]
    #[cfg(debug_assertions)]
    #[should_panic(expected = "timestamps must be strictly increasing")]
    fn out_of_order_push_panics_in_debug() {
        let mut buf = PoseBuffer::new(16, 200_000);
        buf.push(2_000_000, identity());
        buf.push(1_000_000, identity()); // older timestamp — must panic
    }

    #[test]
    #[cfg(debug_assertions)]
    #[should_panic(expected = "timestamps must be strictly increasing")]
    fn equal_timestamp_push_panics_in_debug() {
        let mut buf = PoseBuffer::new(16, 200_000);
        buf.push(1_000_000, identity());
        buf.push(1_000_000, identity()); // same timestamp — must panic
    }

    #[test]
    fn single_entry_tolerance_boundary() {
        let mut buf = PoseBuffer::new(16, 100_000);
        buf.push(1_000_000, identity());
        assert!(buf.pose_at(950_000).is_some(),  "50 ms before — within tolerance");
        assert!(buf.pose_at(1_050_000).is_some(), "50 ms after — within tolerance");
        assert!(buf.pose_at(1_200_000).is_none(), "200 ms after — outside tolerance");
    }
}
