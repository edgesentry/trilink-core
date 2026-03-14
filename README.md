# trilink-core

Open-source Rust library for robot-vision spatial fusion.

`trilink-core` provides the building blocks for mapping AI image detections to 3D world coordinates using a patrol robot's pose stream. It is domain-agnostic — vehicle damage detection is one application, but the same pipeline applies to any problem where you need to answer: *"Where exactly in a real-world space is this thing I detected in an image?"*

## What's in the crate

| Module | What it does |
|---|---|
| `buffer::PoseBuffer` | Ring buffer of robot poses indexed by timestamp. O(log n) lookup by `capture_ts_us`. |
| `bridge::unproject` | Pinhole unprojection: pixel `(u, v)` + depth → camera space → world `(X, Y, Z)`. |
| `ingress::RobotSource` | Trait for streaming robot frames (implement for your hardware). |
| `ingress::MockSource` | Deterministic frame source for testing, no hardware required. |
| Core types | `InspectionPacket`, `Detection`, `BBox2D`, `Transform4x4`, `CameraIntrinsics`, `Point3D` |
| `TriError` | Unified error type (`thiserror`). |

## Usage

Add to your `Cargo.toml`:

```toml
[dependencies]
trilink-core = { git = "https://github.com/edgesentry/tri-link-core.git", branch = "main" }
```

### Implement `RobotSource` for your hardware

```rust
use trilink_core::ingress::{RobotSource, RobotFrame};
use trilink_core::TriError;

struct MyRobot { /* ... */ }

impl RobotSource for MyRobot {
    fn next_frame(&mut self) -> Result<RobotFrame, TriError> {
        // pull pose + image from your robot SDK
        todo!()
    }
}
```

### Buffer poses and look up by timestamp

```rust
use trilink_core::{buffer::PoseBuffer, Transform4x4};

let mut buf = PoseBuffer::new(1_000, 200_000); // 1000 entries, 200 ms tolerance
buf.push(capture_ts_us, pose);

// later, after inference returns input_ts:
if let Some(pose) = buf.pose_at(input_ts) {
    // pose is the robot's position at shutter time, not inference completion time
}
```

### Unproject a detection to world coordinates

```rust
use trilink_core::{bridge::unproject, BBox2D, CameraIntrinsics, Transform4x4};

let world_pos = unproject(
    &bbox,
    depth_m,           // Option<f32> from ToF sensor
    fallback_depth_m,  // used when depth_m is None
    &intrinsics,
    &pose,
);
// world_pos: Point3D { x, y, z } in the robot's world frame
```

## Features

| Feature | Default | Description |
|---|---|---|
| `sqlite` | off | Enables `rusqlite` dependency for SQLite output |

## Documentation

- [docs/math.md](docs/math.md) — coordinate systems, pinhole camera model, unprojection derivation
- [docs/architecture.md](docs/architecture.md) — data flow and how this library fits into the full pipeline
- [docs/contributing.md](docs/contributing.md) — build and test commands

## License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE))
- MIT license ([LICENSE-MIT](LICENSE-MIT))

at your option.
