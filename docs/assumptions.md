# Assumptions

This document records every assumption Tri-Link makes about the systems it integrates with, the runtime environment, and the data it processes. Any violation of these assumptions will require a corresponding code change.

---

## Role Boundaries

Each party does their own job. Tri-Link is the glue — its sole purpose is to bridge the robot platform and the AI inference service so that customers can see exactly where damage is located in the 3D model.

```
Robot Platform           Tri-Link              AI Inference Service
(+ cloud)                ────────              (vision AI platform)
─────────────            (our job)             ───────────────────
Captures images    ───►  Receives frames  ───► Runs damage detection
Produces 3D model        Binds pose +          Returns bounding boxes
Streams robot pose       timestamp             + confidence scores
Manages site map  ◄───   Resolves damage  ◄───
                         to 3D coordinates
                         Writes damage map
                              │
                              ▼
                         Customer sees damage
                         locations in 3D model
```

**Robot platform and its cloud** — Responsible for robot operation, autonomous patrol, 3D environment modeling (SLAM, LiDAR, point cloud), and the site map that customers use. This is entirely their domain. Tri-Link does not touch it.

**Vision AI platform** — Responsible for training and evaluating the damage detection model. They produce the model artifact. Tri-Link treats the inference endpoint as a black box: send JPEG, receive bounding boxes.

**Tri-Link (our job)** — The bridge. Receives images and poses from the robot platform, sends images to the inference service, and writes back damage coordinates anchored in the robot's 3D world frame. The customer outcome: open the 3D model and see precisely which vehicle and which surface has a scratch, dent, or crack.

---

## Robot Platform Interface

### 3D Model and Pose

| Assumption | Rationale |
|---|---|
| The robot platform (via its cloud service) is responsible for building and maintaining the 3D environment model | Tri-Link consumes poses from this model's coordinate frame but does not construct or update the model itself |
| Pose is published at ≥ 10 Hz (typical: 30 Hz) | Buffer holds 1 000 entries; at 10 Hz this covers 100 s — enough to outlast any realistic inference delay |
| Each pose carries a `capture_ts_us` field: UNIX microseconds synchronized to the camera shutter | The entire temporal sync mechanism depends on this. If the clock source differs from the camera clock, position error re-enters |
| Clock skew between the robot's pose clock and camera shutter clock is ≤ 10 ms | At 1 m/s robot speed, 10 ms skew = 1 cm position error, within the 1 cm tolerance target |
| Pose format is a row-major homogeneous 4×4 matrix (`[f32; 16]`) in the world frame established by the robot's 3D model | `Transform4x4` uses this layout; damage coordinates will be in the same world frame |
| The robot platform provides camera intrinsics (`fx, fy, cx, cy`) as a static configuration value | Intrinsics are read once at startup from `config.toml`; no online calibration |

### Camera / Image Stream

| Assumption | Rationale |
|---|---|
| Images are delivered as JPEG bytes | `InferenceClient` sends `multipart/form-data` with raw JPEG |
| Each frame is accompanied by the `capture_ts_us` of the shutter event | This timestamp is forwarded to the inference service in `X-Capture-Ts` |
| Frame rate is ≤ 30 Hz | Higher rates require larger `PoseBuffer` capacity or faster inference to prevent backpressure |

### Depth (ToF)

| Assumption | Rationale |
|---|---|
| ToF depth is optional; Tri-Link works without it using `fallback_depth_m` | Not all patrol scenarios include depth hardware |
| When present, depth is a single scalar `f32` in meters representing the distance at the frame center | Tri-Link does not process depth images or point clouds directly |
| ToF and camera are hardware-synchronized (same trigger) | Otherwise depth and image correspond to different moments in time |

---

## AI Inference Service Interface

The vision AI platform produces the damage detection model artifact. **We decide where to run it.** The choice affects whether quantization is required but does not change any Tri-Link code — only `[inference].base_url` in `config.toml` changes.

| Assumption | Rationale |
|---|---|
| The service accepts `multipart/form-data` POST with JPEG bytes | This is the `InferenceClient` request format |
| The service accepts an `X-Capture-Ts` header carrying `capture_ts_us` as a decimal string | Required for temporal sync; must be echoed back |
| The response is a JSON array: `[{class, confidence, bbox, input_ts}, ...]` | `infer/types.rs` parses this schema |
| `input_ts` in the response equals the value of `X-Capture-Ts` in the request | Tri-Link uses this to look up the correct historical pose |
| Inference latency is ≤ 33 s under normal operating conditions | The `PoseBuffer` covers 33 s at 30 Hz. Longer delays mean the pose is no longer available and the detection is dropped |
| The service is reachable at `[inference].base_url` in `config.toml` | Cloud URL or `localhost` depending on deployment — no code change required |

### Inference Deployment Decision

| Option | Quantization required? | `base_url` | Constraints |
|---|---|---|---|
| **Cloud** | No — full-precision model runs on cloud hardware | `https://api.example.com/v1/infer` | Requires stable network connectivity from the patrol site |
| **On-robot** | Yes — model must be quantized to fit the robot's SoC | `http://localhost:PORT/v1/infer` | Works offline; quantization must be validated for accuracy loss |

The deployment decision is operational, not a code change. Quantization (if on-robot) is the responsibility of the team operating the model — Tri-Link receives bounding boxes either way.

**Factors that favor cloud:** model accuracy is critical; connectivity is reliable; model updates need to be fast.

**Factors that favor on-robot:** site has no reliable network (underground car parks, tunnels); end-to-end latency is a concern; data must not leave the site.

---

## Runtime Environment

| Assumption | Rationale |
|---|---|
| Tri-Link runs on the robot or an edge node on the same LAN | Required for low-latency access to the robot platform's pose and image streams |
| The host OS provides a monotonic clock and NTP-synchronized wall clock | `capture_ts_us` is a wall-clock value; clock drift corrupts temporal sync |
| Sufficient disk space exists for JSONL output and SQLite database | Tri-Link does not enforce disk quotas; operator must provision appropriately |
| The `reports/` and `frames/` directories are writable at startup | Created by the operator before running `trilink run` |

---

## Damage Localization Accuracy

| Parameter | Default | Configurable? |
|---|---|---|
| Fallback depth when ToF unavailable | 2.0 m | Yes (`camera.fallback_depth_m`) |
| Pose buffer temporal tolerance | 200 ms | Default; configurable via `PoseBuffer::new` parameter |
| Expected accuracy with ToF | ~1 cm at 2 m depth | Depends on ToF precision and calibration quality |
| Expected accuracy with fallback depth | ±10 cm (depth error dominates) | Depends on actual target distance vs. assumed 2.0 m |

---

## Out of Scope

These are explicitly not handled by Tri-Link. A separate integration is needed if they become requirements.

- **3D model construction**: Building or updating the 3D environment model is the robot platform's responsibility (its cloud service).
- **Model training**: Training and versioning the damage detection model is the vision AI platform's responsibility.
- **Loop closure / map correction**: If the robot's SLAM system retroactively corrects poses, previously recorded damage positions are not updated.
- **Multi-camera rigs**: One camera stream with one intrinsics set per process instance.
- **Model hot-swap**: Switching the inference model at runtime without restarting is not supported.
- **Detection deduplication**: The same physical damage appearing in multiple frames produces multiple records; merging is not implemented.
- **Robot platform authentication**: Tri-Link assumes the robot's HTTP endpoints are accessible on a trusted internal network without credentials.
