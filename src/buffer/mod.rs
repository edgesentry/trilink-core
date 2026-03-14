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

    /// Look up the pose closest to `capture_ts_us`.
    ///
    /// Returns `None` if no entry is within `tolerance_us` of the requested
    /// timestamp.
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
        // Binary-search each segment independently and take the closest match.
        let seg_a = &self.entries[self.head..self.len];
        let seg_b = &self.entries[..self.head];

        let mut best: Option<(u64, Transform4x4)> = None;
        for seg in [seg_a, seg_b] {
            // lower_bound: idx is the first entry with ts_us >= capture_ts_us.
            // Exact match lands at idx; nearest-before lands at idx-1.
            let idx = seg.partition_point(|e| e.ts_us < capture_ts_us);
            for &i in &[idx.wrapping_sub(1), idx] {
                if let Some(e) = seg.get(i) {
                    let delta = e.ts_us.abs_diff(capture_ts_us);
                    if delta <= self.tolerance_us && best.is_none_or(|(d, _)| delta < d) {
                        best = Some((delta, e.pose));
                    }
                }
            }
        }
        best.map(|(_, pose)| pose)
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
    fn picks_closer_of_two_candidates() {
        let mut buf = PoseBuffer::new(16, 500_000);
        // Distinguish poses by x-translation so we can tell which was returned.
        let mut pose_a = Transform4x4::identity();
        pose_a.matrix[3] = 1.0; // x = 1 — stored at 1000 ms
        let mut pose_b = Transform4x4::identity();
        pose_b.matrix[3] = 2.0; // x = 2 — stored at 1200 ms
        buf.push(1_000_000, pose_a);
        buf.push(1_200_000, pose_b);
        // Query at 1150 ms: delta_a=150 ms, delta_b=50 ms → pose_b is closer
        let result = buf.pose_at(1_150_000).unwrap();
        assert_eq!(result.matrix[3], 2.0, "should select the closer pose");
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
