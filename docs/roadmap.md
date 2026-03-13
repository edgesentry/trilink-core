# Roadmap

Implementation is divided into six milestones. Each builds on the previous and ends with a passing `cargo test --workspace`.

---

## M1 — Repository Bootstrap ✅

- [x] `Cargo.toml`, `deny.toml`, `.gitignore`
- [x] Core types compile: `InspectionPacket`, `Detection`, `CameraIntrinsics`, `Transform4x4`, `Point3D`, `BBox2D`
- [x] `MockSource` produces deterministic pose sequence + valid JPEG frames
- [x] `PoseBuffer` ring buffer with exact-match and nearest-neighbor lookup
- [x] `cargo build` and `cargo test` pass with no warnings

---

## M2 — Unprojection Math ✅

Resolve a detection's pixel center to a 3D world coordinate.

- [x] Implement `core/src/bridge/unproject.rs`:
  - Input: `BBox2D`, `depth_m: Option<f32>`, `fallback_depth_m: f32`, `CameraIntrinsics`, `Transform4x4`
  - Output: `Point3D` in world frame
  - Formula: `P_camera = K⁻¹ · [u, v, 1]ᵀ · depth` → `P_world = pose · [P_camera, 1]ᵀ`
  - When `depth_m` is `None`, use `fallback_depth_m` (caller passes `config.camera.fallback_depth_m`)
- [x] Unit tests:
  - Identity pose + known intrinsics → deterministic world coordinate
  - Depth scales world coordinate proportionally
  - Edge pixel `(u=0, v=0)` → correct corner coordinate
  - Fallback depth produces non-`None` result

---

## M3 — Inference Client

Call the inference HTTP endpoint and parse detections.

- [ ] Implement `app/src/infer/types.rs`: request and response structs with `serde` derives
- [ ] Implement `InferenceClient` in `app/src/infer/mod.rs`:
  - `POST multipart/form-data` with JPEG bytes
  - `X-Capture-Ts: {capture_ts_us}` request header
  - Parse JSON response: `[{class, confidence, bbox, input_ts}]`
  - Return `Vec<Detection>` with `world_pos = None` (unprojection happens in bridge)
- [ ] Unit tests with a mock HTTP server:
  - Happy path: valid response → correct `Detection` fields
  - `input_ts` is correctly echoed back and preserved

---

## M4 — Bridge: Sync + Assemble

Wire pose lookup and unprojection together into a complete `InspectionPacket`.

- [ ] Implement `app/src/bridge/sync.rs`:
  - Accept an inference result carrying `input_ts`
  - Look up `PoseBuffer.pose_at(input_ts)` → `TriError::PoseNotFound` if outside tolerance
- [ ] Assemble `InspectionPacket` in `app/src/bridge/mod.rs`:
  - Call `unproject` for each detection
  - Fill `world_pos` from the resolved pose
- [ ] Unit tests:
  - Inference result with known `input_ts` → correct pose selected from buffer
  - `input_ts` outside tolerance → `TriError::PoseNotFound`

---

## M5 — Semantic Map Output

Write located damage records to disk.

- [ ] Implement `app/src/egress/semantic_map.rs`:
  - Append one JSON line per detection to `reports/damage_map.jsonl`
  - Schema: `{ts, class, confidence, world_pos: {x,y,z}, image_ref}`
  - Feature-gated SQLite writer: insert into `damage.db` (columns: `ts`, `class`, `confidence`, `x`, `y`, `z`, `image_ref`)
- [ ] Save the original JPEG to `frames/{capture_ts_us}.jpg`
- [ ] Unit tests:
  - Write one packet → verify JSONL line matches expected JSON
  - Write multiple packets → verify all lines present, file is append-only

---

## M6 — Integration Test + CLI

End-to-end pipeline validation.

- [ ] `tests/integration.rs`:
  - `MockSource` → `PoseBuffer` → `MockInferenceClient` → `semantic_map`
  - Assert detection count matches expected
  - Assert `world_pos` is within 1 cm of expected coordinate
- [ ] `trilink run --config config.toml` wires all components and runs until interrupted
- [ ] `cargo test --workspace` passes with no warnings or failures
- [ ] `cargo deny check licenses` passes

---

## Configuration

All configuration fields are documented in [`config.example.toml`](../config.example.toml). Copy it to `config.toml` before running.
