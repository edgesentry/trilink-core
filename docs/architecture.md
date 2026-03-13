# Architecture

## The Problem

Two independent systems produce complementary data about the physical world, but neither can answer the question that matters: *where exactly in 3D space is this damage?*

- The **robot platform** moves through a space and continuously streams its pose (position + orientation) along with camera images and depth measurements. It knows *where* the robot is but does not classify damage.
- The **AI inference service** receives a JPEG and returns bounding boxes with damage labels. It knows *what* the damage looks like but has no spatial context.

There are two fundamental gaps to bridge:

1. **Time gap** — The inference service takes ~100 ms to respond. The robot is moving. By the time the result arrives, the robot has moved ~10 cm at normal patrol speed. If Tri-Link naïvely uses the robot's *current* pose when the result arrives, the damage location will be wrong.

2. **Space gap** — The inference service returns pixel coordinates `(u, v)`. Converting those to world coordinates `(X, Y, Z)` requires the camera's optical parameters and the robot's pose at the exact moment the shutter opened.

Tri-Link resolves both gaps and writes the result to a queryable damage map.

---

## Data Flow

```
Robot Platform                  Tri-Link                    Inference Service
──────────────                  ────────                    ─────────────────

pose stream ──────────────────► PoseBuffer
                                (ring buffer, 1000 entries)

image + capture_ts_us ────────► InspectionPacket builder ──► POST JPEG + capture_ts_us
                                                              ◄── detections + input_ts

                                bridge/sync.rs
                                  look up pose_at(input_ts) from PoseBuffer
                                  ↓
                                bridge/unproject.rs
                                  (u, v, depth) → world (X, Y, Z)
                                  ↓
                                egress/semantic_map.rs
                                  ↓
                         damage_map.jsonl + damage.db
```

---

## Module Responsibilities

| Module | Responsibility |
|---|---|
| `ingress/` | Receive frames from the robot platform (`RobotSource` trait + `MockSource` for testing) |
| `buffer/` | `PoseBuffer` — ring buffer of poses indexed by timestamp; O(log n) lookup |
| `bridge/sync.rs` | Join inference result to its original pose using the echoed `capture_ts_us` |
| `bridge/unproject.rs` | Pinhole unprojection: pixel + depth → camera space → world space |
| `infer/` | `InferenceClient` — HTTP client that POSTs JPEG and parses detection response |
| `egress/` | Write `InspectionPacket` to JSONL + SQLite |
| `error.rs` | `TriError` — unified error type |
| `main.rs` | CLI entry point (`trilink run --config`) |

---

## Temporal Synchronization Detail

Every robot frame carries a `capture_ts_us` — the UNIX timestamp in microseconds when the camera shutter opened. This value is embedded in the inference request as an HTTP header `X-Capture-Ts`. The inference service echoes it back as `input_ts` in the response JSON.

When the response arrives:

```
inference response.input_ts  ──►  PoseBuffer.pose_at(input_ts)
                                  → pose at shutter time, not inference completion time
```

The `PoseBuffer` holds the last 1 000 poses (≈33 seconds at 30 Hz). Any inference result that arrives within the buffer window can be correctly located regardless of network or processing delay.

---

## Unprojection Math

> For a full explanation of the coordinate systems and camera model behind this formula, see [math.md](math.md).

Given a detection bounding box `(u0, v0, u1, v1)`:

```
center_u = (u0 + u1) / 2
center_v = (v0 + v1) / 2

P_camera = K⁻¹ · [center_u, center_v, 1]ᵀ · depth_m
         = [(center_u - cx) / fx · depth_m,
            (center_v - cy) / fy · depth_m,
            depth_m]

P_world  = pose_matrix · [P_camera, 1]ᵀ
```

Where `K` is the 3×3 pinhole intrinsics matrix built from `fx, fy, cx, cy`.

If ToF depth is unavailable, a configurable fallback distance is used (default 2.0 m, appropriate for side-on vehicle scanning).

---

## Core Data Structures

```rust
pub struct InspectionPacket {
    pub capture_ts_us: u64,       // shutter timestamp (µs since UNIX epoch)
    pub pose: Transform4x4,       // robot pose at capture_ts_us
    pub camera_k: CameraIntrinsics,
    pub image_jpeg: Vec<u8>,
    pub detections: Vec<Detection>,
}

pub struct Detection {
    pub class: String,            // "scratch" | "dent" | "crack"
    pub confidence: f32,
    pub bbox: BBox2D,
    pub world_pos: Option<Point3D>, // None until unprojection resolves it
    pub depth_m: Option<f32>,
}

pub struct CameraIntrinsics { pub fx: f64, pub fy: f64, pub cx: f64, pub cy: f64 }
pub struct Transform4x4    { pub matrix: [f32; 16] }  // row-major homogeneous
pub struct Point3D          { pub x: f32, pub y: f32, pub z: f32 }
pub struct BBox2D           { pub u0: u32, pub v0: u32, pub u1: u32, pub v1: u32 }
```

---

## Crate Dependencies

| Crate | Purpose |
|---|---|
| `tokio` (full) | Async runtime |
| `reqwest` | HTTP client for inference service |
| `serde` + `serde_json` | JSON serialization |
| `rusqlite` (optional, feature `sqlite`) | SQLite damage map |
| `thiserror` | `TriError` derivation |
| `tracing` + `tracing-subscriber` | Structured logging |
| `clap` (derive) | `trilink run --config` CLI |
| `toml` | Config file parsing |

---

## Deployment

Tri-Link runs **on the robot** or on an edge node co-located with the robot. It must be on the same network segment as the robot platform to receive pose and image streams with low latency.

The inference service location is a configuration choice — no code change required. See [assumptions.md](assumptions.md) for the full deployment decision guide and all interface contracts.
