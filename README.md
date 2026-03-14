# trilink-core

Open-source Rust library for sensor-vision spatial fusion.

`trilink-core` provides the building blocks for mapping AI image detections to 3D world coordinates using a moving sensor platform's pose stream. It is domain-agnostic — the same pipeline applies to any problem where you need to answer: *"Where exactly in a real-world space is this thing I detected in an image?"*

## Name

"Tri-Link" refers to the three coordinate spaces the library links together:

```
Image space  →  Camera space  →  World space
 (pixels)        (metres,          (metres,
                  camera-centred)   map-centred)
```

It also reflects the three systems it connects — sensor platform, inference service, and application — and the two gaps it bridges between them: the **time gap** (inference latency vs. platform motion) and the **space gap** (pixel coordinates vs. 3D world coordinates).

## What's in the crate

| Module | What it does |
|---|---|
| `buffer::PoseBuffer` | Ring buffer of platform poses indexed by timestamp. O(log n) lookup by `capture_ts_us`. |
| `bridge::unproject` | Pinhole unprojection: pixel `(u, v)` + depth → camera space → world `(X, Y, Z)`. |
| `ingress::FrameSource` | Trait for streaming sensor frames (implement for your hardware). |
| `ingress::MockSource` | Deterministic frame source for testing, no hardware required. |
| Core types | `FusionPacket`, `Detection`, `BBox2D`, `Transform4x4`, `CameraIntrinsics`, `Point3D` |
| `TriError` | Unified error type (`thiserror`). |

## Usage

Add to your `Cargo.toml`:

```toml
[dependencies]
trilink-core = { git = "https://github.com/edgesentry/tri-link-core.git", branch = "main" }
```

### Implement `FrameSource` for your hardware

```rust
use trilink_core::ingress::{FrameSource, SensorFrame};
use trilink_core::TriError;

struct MyPlatform { /* ... */ }

impl FrameSource for MyPlatform {
    fn next_frame(&mut self) -> Result<SensorFrame, TriError> {
        // pull pose + image from your platform SDK
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
    // pose is the platform's position at shutter time, not inference completion time
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
// world_pos: Point3D { x, y, z } in the platform's world frame
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
