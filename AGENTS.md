# AGENTS.md — trilink-core Codebase Guide

This file is for AI agents working in this repository. It describes the codebase structure, build system, key types, and conventions.

---

## Purpose

`trilink-core` is a standalone open-source Rust library crate (MIT OR Apache-2.0).

It provides the generic infrastructure for mapping AI image detections to 3D world coordinates using a moving sensor platform's pose stream: temporal synchronisation, pinhole unprojection math, shared data types, and a streaming interface for sensor hardware.

---

## Module Map

```
src/
  lib.rs              Core data types: FusionPacket, Detection, BBox2D,
                      Transform4x4, CameraIntrinsics, Point3D
  error.rs            TriError — unified error enum (thiserror)
  ingress/
    mod.rs            FrameSource trait + SensorFrame struct
    mock.rs           MockSource: deterministic frames for testing
  buffer/
    mod.rs            PoseBuffer: ring buffer keyed by capture_ts_us
  bridge/
    mod.rs            re-exports unproject
    unproject.rs      (u,v,depth) → world (X,Y,Z)
```

---

## Key Types

| Type | File | Purpose |
|---|---|---|
| `FusionPacket` | `src/lib.rs` | One complete event: pose + image + detections |
| `Detection` | `src/lib.rs` | Single bounding box with optional world position |
| `BBox2D` | `src/lib.rs` | Pixel-space bounding box; `.center()` returns `(f64, f64)` |
| `Transform4x4` | `src/lib.rs` | Row-major 4×4 pose matrix; `.identity()` for tests |
| `CameraIntrinsics` | `src/lib.rs` | `fx, fy, cx, cy` pinhole parameters |
| `PoseBuffer` | `src/buffer/mod.rs` | Ring buffer; `.push(ts, pose)` / `.pose_at(ts)` |
| `FrameSource` | `src/ingress/mod.rs` | Trait for frame sources; implement for real hardware |
| `SensorFrame` | `src/ingress/mod.rs` | Frame emitted by a sensor platform: JPEG + pose + depth |
| `MockSource` | `src/ingress/mock.rs` | Deterministic test source; use `.with_limit(n)` in tests |
| `TriError` | `src/error.rs` | All error variants; feature-gated `Sqlite` variant |

---

## Build and Test

See [docs/contributing.md](docs/contributing.md) for all build, test, and contribution commands.

---

## Conventions

- **Error handling**: use `TriError`; propagate with `?`; no `.unwrap()` in library code
- **Timestamps**: always `u64` microseconds since UNIX epoch (`capture_ts_us`)
- **Pose matrix**: row-major `[f32; 16]`, world frame
- **No company or product names** in any source file or document — use generic terms (`sensor platform`, `inference service`)
- **Feature gate SQLite**: anything touching `rusqlite` must be behind `#[cfg(feature = "sqlite")]`

---

## Reference Documents

| Document | Contents |
|---|---|
| [README.md](README.md) | Library overview and usage examples |
| [docs/architecture.md](docs/architecture.md) | Data flow, module map, data structures |
| [docs/math.md](docs/math.md) | Coordinate systems, camera model, unprojection math (prerequisite reading) |
| [docs/contributing.md](docs/contributing.md) | Build, test, and contribution commands |
