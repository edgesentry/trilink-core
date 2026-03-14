# Architecture

## The Problem

Two independent systems produce complementary data about the physical world, but neither can answer the question that matters: *where exactly in 3D space is this thing I detected?*

- The **sensor platform** moves through a space and continuously streams its pose (position + orientation) along with camera images and optional depth measurements. It knows *where* the platform is but does not classify what it sees.
- The **AI inference service** receives a JPEG and returns bounding boxes with labels. It knows *what* was detected but has no spatial context.

There are two fundamental gaps to bridge:

1. **Time gap** — The inference service takes ~100 ms to respond. The platform is moving. By the time the result arrives, it may have moved ~10 cm. Using the platform's *current* pose when the result arrives produces a wrong location.

2. **Space gap** — The inference service returns pixel coordinates `(u, v)`. Converting those to world coordinates `(X, Y, Z)` requires the camera's optical parameters and the platform's pose at the exact moment the shutter opened.

`trilink-core` resolves both gaps.

---

## Data Flow

```
Sensor Platform                 Application layer               Inference Service
───────────────                 ─────────────────               ─────────────────

pose stream ──────────────────► PoseBuffer
                                (ring buffer, 1000 entries)

image + capture_ts_us ────────► send to inference ────────────► POST JPEG + capture_ts_us
                                                                ◄── detections + input_ts

                                pose_at(input_ts)  ◄── look up pose at shutter time
                                      ↓
                                unproject(bbox, depth, pose)
                                      ↓
                                Point3D in world frame
```

The application layer (not part of this crate) is responsible for the HTTP client, output writing, and CLI. This crate provides `PoseBuffer`, `unproject`, and the shared types.

---

## Module Responsibilities

| Module | Responsibility |
|---|---|
| `ingress/` | `FrameSource` trait for streaming frames; `MockSource` for testing |
| `buffer/` | `PoseBuffer` — ring buffer of poses indexed by timestamp; O(log n) lookup |
| `bridge/unproject.rs` | Pinhole unprojection: pixel + depth → camera space → world space |
| `error.rs` | `TriError` — unified error type |

---

## Temporal Synchronisation

Every sensor frame carries a `capture_ts_us` — the UNIX timestamp in microseconds when the camera shutter opened. This value is forwarded to the inference service. The inference service echoes it back as `input_ts` in the response.

When the response arrives:

```
inference response.input_ts  ──►  PoseBuffer.pose_at(input_ts)
                                  → pose at shutter time, not inference completion time
```

The `PoseBuffer` holds the last 1,000 poses (~33 seconds at 30 Hz). Any inference result that arrives within that window can be correctly located regardless of network or processing delay.

---

## Unprojection Math

> For the full derivation see [math.md](math.md).

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

If depth is unavailable, a configurable fallback distance is used (default 2.0 m).

---

## Core Data Structures

```rust
pub struct FusionPacket {
    pub capture_ts_us: u64,         // shutter timestamp (µs since UNIX epoch)
    pub pose: Transform4x4,         // platform pose at capture_ts_us
    pub camera_k: CameraIntrinsics,
    pub image_jpeg: Vec<u8>,
    pub detections: Vec<Detection>,
}

pub struct Detection {
    pub class: String,              // inference label, e.g. "person", "crack", "vehicle"
    pub confidence: f32,
    pub bbox: BBox2D,
    pub world_pos: Option<Point3D>, // None until unprojection resolves it
    pub depth_m: Option<f32>,
}

pub struct CameraIntrinsics { pub fx: f64, pub fy: f64, pub cx: f64, pub cy: f64 }
pub struct Transform4x4    { pub matrix: [f32; 16] }  // row-major homogeneous
pub struct Point3D         { pub x: f32, pub y: f32, pub z: f32 }
pub struct BBox2D          { pub u0: u32, pub v0: u32, pub u1: u32, pub v1: u32 }
```

---

## Crate Dependencies

| Crate | Purpose |
|---|---|
| `serde` + `serde_json` (dev) | JSON serialization of core types |
| `rusqlite` (optional, feature `sqlite`) | SQLite output support |
| `thiserror` | `TriError` derivation |
