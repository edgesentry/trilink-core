# trilink-core — Roadmap

Each item is derived directly from a documented limitation in [limitations.md](limitations.md).
Items are grouped by priority tier; within a tier, order is implementation dependency.

---

## Priority tiers

| Tier | Label | Criterion |
|---|---|---|
| P0 | **Correctness** | Silent wrong output today; fix before any production deployment |
| P1 | **Accuracy** | Measurable precision gap against published benchmarks; ~10 pp F1 or ~5× error |
| P2 | **Observability** | Currently impossible to detect or diagnose the problem at runtime |
| P3 | **Completeness** | Real capability gap that limits downstream milestones |

---

## P0 — Correctness

### C1 · PoseBuffer: distinguish failure modes

**Limitation:** L7 — `pose_at` returns `None` for two completely different conditions:
(a) tolerance exceeded (inference is too slow — fixable operationally), and
(b) entry evicted from ring buffer (platform paused >33 s — requires architectural response).
A caller cannot tell these apart. Both failures are silent.

**Current code:** `buffer/mod.rs` — `pose_at` returns `Option<Transform4x4>`.

**Change:** Replace the return type with a `Result`-style enum:

```rust
pub enum PoseResult {
    Found(Transform4x4),
    ToleranceExceeded { delta_us: u64 },
    Evicted,
}
```

`ToleranceExceeded` → caller may retry, increase buffer capacity, or flag the detection as low-confidence.
`Evicted` → caller must drop the detection; the data is structurally unrecoverable.

**Acceptance test:** extend `ring_overwrites_oldest` to assert `Evicted` (not `None`); extend `outside_tolerance_returns_none` to assert `ToleranceExceeded` with correct `delta_us`.

---

### C2 · Point3D: runtime guard for geodetic coordinates

**Limitation:** L4 — feeding UTM or WGS-84 coordinates into `Point3D` silently degrades
precision to ~12 mm, making the 5–10 mm deviation threshold meaningless. There is no guard.

**Current code:** `lib.rs` — `Point3D { x: f32, y: f32, z: f32 }` with no validation.

**Change:** Add a `#[cfg(debug_assertions)]` check in `Transform4x4::transform_point`
(the only place that writes a `Point3D` result) that fires when any coordinate exceeds 10 000 m:

```rust
debug_assert!(
    p.x.abs() < 10_000.0 && p.y.abs() < 10_000.0 && p.z.abs() < 10_000.0,
    "Point3D coordinate out of local-frame range: ({}, {}, {}). \
     Did you pass absolute geodetic (UTM/WGS-84) coordinates? \
     Subtract the site datum origin first.",
    p.x, p.y, p.z
);
```

**Acceptance test:** assert the panic fires in debug mode when a world-frame translation of 200 000 m is applied.

---

## P1 — Accuracy

### A1 · HeightMap: add min-Z and Z-range projections

**Limitation:** L2 — `project_to_height_map` stores only maximum Z per cell (protrusion detection).
Depressions — spalling craters, section loss, subsidence — are structurally invisible because
points below the current maximum overwrite nothing.

**Current code:** `bridge/project.rs` — `project_to_height_map` uses `max(data[idx], p.z)`.

**Change:** Add two new functions to `bridge/project.rs`:

- `project_to_height_map_min` — stores minimum Z per cell; detects depressions. Initialise with `f32::INFINITY`.
- `project_to_height_map_range` — stores `(min_z, max_z)` per cell; returns a `HeightMapRange` type. Range = `max - min` is the vertical spread, useful for detecting both protrusions and voids and for estimating surface roughness.

`project_to_height_map` (max-Z) is unchanged. All three functions share the same cell-indexing formula.

**Acceptance test:** two points in the same cell at z=1.0 and z=3.0 → min function stores 1.0; range function stores (1.0, 3.0) with spread 2.0.

---

### A2 · DepthMap: coverage statistics

**Limitation:** L1 — single-viewpoint occlusion leaves depth map cells at `f32::INFINITY`
with no way to know what fraction of the surface was actually measured.

**Current code:** `lib.rs` — `DepthMap` has no coverage methods beyond `has_depth(u, v)`.

**Change:** Add to `impl DepthMap`:

```rust
/// Fraction of pixels that have a finite depth value (0.0–1.0).
pub fn coverage_fraction(&self) -> f32

/// Returns the bounding rectangle (u_min, v_min, u_max, v_max) of all finite-depth pixels.
/// Returns None if no pixel has depth.
pub fn covered_region(&self) -> Option<(u32, u32, u32, u32)>
```

These feed directly into the P3 multi-view fusion decision: if `coverage_fraction() < 0.7`, request an additional capture pose.

**Acceptance test:** a 4×4 depth map with 8 finite pixels → `coverage_fraction() == 0.5`.

---

### A3 · Multi-view depth map fusion

**Limitation:** L1 — surfaces not visible from a single capture pose are absent from the
depth map. For bridge inspection, this may exclude undersides and edge faces entirely.

**Current code:** There is no fusion primitive. Each `project_to_depth_map` call is independent.

**Change:** Add `bridge/fuse.rs`:

```rust
/// Merge N depth maps captured from different poses into one.
///
/// Per pixel: keep the finite value with the smallest depth (nearest surface seen
/// across all poses). If a pixel is finite in multiple maps, the minimum wins
/// (consistent with the Z-buffer convention in project_to_depth_map).
pub fn fuse_depth_maps(maps: &[&DepthMap]) -> DepthMap
```

All input maps must share the same `(width, height)`. Mismatched dimensions are a programming error: `debug_assert!`.

This function is purely additive — existing single-frame callers are unaffected.

**Acceptance test:** two maps of size 2×1: map A has depth at pixel (0,0)=1.0, INFINITY at (1,0);
map B has INFINITY at (0,0), 2.0 at (1,0). Fused result: (0,0)=1.0, (1,0)=2.0.

---

### A4 · Unproject: surface-normal-aware back-projection

**Limitation:** L3 — `unproject` assumes a flat projection plane. On cylindrical or curved
surfaces, recovered world positions carry a systematic bias: ~11.7% relative error on typical
bridge columns vs ~2.5% on flat walls.

**Current code:** `bridge/unproject.rs` — `unproject` takes `bbox`, `depth_m`, `fallback_depth_m`, `k`, `pose`. It locates the detection at the bbox centre at the scalar depth. No surface orientation is considered.

**Change:** Add an extended variant to `bridge/unproject.rs`:

```rust
/// Unproject with surface-normal correction.
///
/// `surface_normal` is the outward unit normal at the detection point,
/// in world space. When the projection ray is not perpendicular to the surface,
/// the true distance from the camera to the surface along the ray differs from
/// the sensor's reported range by a factor of 1/cos(θ), where θ is the
/// angle between the ray and the normal.
///
/// This corrects the depth before unprojection, reducing back-projection
/// error on curved surfaces from ~11.7% to <3%.
pub fn unproject_with_normal(
    bbox: &BBox2D,
    depth_m: Option<f32>,
    fallback_depth_m: f32,
    k: &CameraIntrinsics,
    pose: &Transform4x4,
    surface_normal: &Point3D,   // unit normal in world space
) -> Point3D
```

The original `unproject` is unchanged. Surface normals are extracted from the IFC reference cloud (nearest-neighbour normal of the k-d tree match).

**Acceptance test:** flat surface (normal parallel to projection ray) → result identical to `unproject`;
45° angled surface → result differs by `cos(45°)` factor on the depth axis.

---

## P2 — Observability

### O1 · Detection: tag fallback-depth results

**Limitation:** L6 — when `depth_m = None`, `unproject` silently uses `fallback_depth_m`.
The resulting `Detection.world_pos` is positionally unreliable but looks identical to a
sensor-depth-backed position. Downstream reporting cannot filter them.

**Current code:** `lib.rs` — `Detection { depth_m: Option<f32>, world_pos: Option<Point3D> }`.
There is no flag distinguishing "depth from sensor" vs "depth from fallback".

**Change:** Add a field to `Detection`:

```rust
pub depth_source: DepthSource,

#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum DepthSource {
    Sensor,
    Fallback,
}
```

`unproject` is responsible for writing this field when called.
The JSON report and heatmap can then mark fallback-depth detections with a distinct colour or confidence band.

**Acceptance test:** `unproject` called with `depth_m = None` → returned detection has `depth_source = Fallback`.

---

### O2 · Precision benchmark suite (integration tests)

**Limitation:** P1–P5 from [limitations.md](limitations.md#precision-and-performance-verification) — there are no tests that assert millimetre-level accuracy against known ground truth. Existing round-trip tests verify structural correctness but not quantitative error bounds.

**Change:** Add `tests/precision.rs` with the following parametric tests:

| Test | Input | Pass criterion |
|---|---|---|
| `flat_surface_roundtrip_error` | Synthetic flat cloud, known defect at known pixel, identity pose | World-space position error ≤ 1 mm at range ≤ 10 m |
| `curved_surface_bias` | Cylindrical cloud, radii 0.1/0.5/1.0/∞ m | Document error vs radius; assert error ≤ 15% at R=0.5 m |
| `controlled_deformation_3mm` | Flat cloud with 3 mm raised patch | `max_deviation_mm ∈ [1.0, 5.0]` |
| `controlled_deformation_10mm` | Flat cloud with 10 mm raised patch | `max_deviation_mm ∈ [8.0, 12.0]` |
| `fallback_depth_sensitivity` | True depth 4.0 m, fallback 2.0 m | Position error ≥ 0.3 m — confirm and document the bound |

These tests do not call inference. They are pure geometry benchmarks.

---

### O3 · Performance benchmark suite

**Limitation:** Perf1–Perf2 from [limitations.md](limitations.md#precision-and-performance-verification).

**Change:** Add `benches/projection.rs` using `criterion`:

| Benchmark | Input sizes | Target |
|---|---|---|
| `project_to_depth_map` | 50K, 200K, 500K points | ≤ 200 ms at 500K on a single thread |
| `project_to_height_map` | 50K, 200K, 500K points | ≤ 200 ms at 500K on a single thread |
| `fuse_depth_maps` (A3) | 2, 4, 8 maps × 1920×1080 | ≤ 100 ms for 4 maps |
| `pose_at` binary search | Buffer of 1000 entries | ≤ 1 µs per lookup |

---

## P3 — Completeness

### CP1 · RGB-D input type

**Limitation:** L5 — the inference path receives a depth map but not the RGB image.
Published F1 on concrete damage detection: depth-only 76.0% vs RGB-D fused 86.7%.
`FusionPacket.image_jpeg` already carries the image but nothing passes it to the bridge.

**Change:** Add `RgbdFrame` to `lib.rs`:

```rust
pub struct RgbdFrame {
    pub depth: DepthMap,
    /// Optional aligned RGB image, same (width, height) as depth.
    /// Each pixel is [R, G, B] u8. None if no camera image available.
    pub rgb: Option<Vec<u8>>,
    pub width: u32,
    pub height: u32,
}
```

Alignment (ensuring each depth pixel corresponds to the same-indexed RGB pixel) is the
caller's responsibility. `RgbdFrame` carries no pose or intrinsics — those stay on `FusionPacket`.

The inference module in `edgesentry-inspect` consumes `RgbdFrame` to build a 4-channel tensor (R, G, B, D).

**Acceptance test:** construct `RgbdFrame` from a depth map and a matching 3-channel image; assert serialisation roundtrip; assert `rgb` channel has `width * height * 3` bytes.

---

### CP2 · IFC write-back data structures

**Limitation:** L8 — AI detection results cannot be written back into an IFC 4.3 model as
`IfcPropertySet` attributes. `trilink-core` can provide the typed data structures; the actual
IFC serialisation lives in `edgesentry-inspect`.

**Change:** Add `ifc_annotation.rs` to `trilink-core`:

```rust
pub struct IfcAnnotation {
    /// IFC GlobalId of the structural member this detection belongs to.
    pub global_id: String,
    /// Detection class (e.g. "surface_void", "crack", "misalignment").
    pub defect_class: String,
    /// Crack width or defect size in millimetres, if measurable.
    pub measured_width_mm: Option<f32>,
    /// AI model confidence score (0.0–1.0).
    pub confidence: f32,
    /// ISO 8601 timestamp of the inspection.
    pub inspection_ts: String,
    /// World-space centroid of the detection.
    pub location: Point3D,
    /// Health classification (I=sound, II=watch, III=repair, IV=urgent).
    pub health_class: HealthClass,
}

pub enum HealthClass { Sound, Watch, Repair, Urgent }
```

This type is serialisable (serde) and maps 1:1 to IFC 4.3 `Pset_DamageInspection` attributes.
The actual `IfcPropertySet` XML/STEP generation is out of scope for `trilink-core`.

**Acceptance test:** serialise and deserialise a round-trip; assert all fields preserved.

---

## Dependency graph

```
C1 (PoseBuffer enum)          — standalone
C2 (Point3D guard)            — standalone

A1 (HeightMap min/range)      — standalone
A2 (DepthMap coverage stats)  — A2 unblocks A3 (coverage fraction drives fusion trigger)
A3 (multi-view fusion)        — requires A2
A4 (normal-aware unproject)   — requires IFC normal extraction (edgesentry-inspect M2)

O1 (DepthSource tag)          — requires C1 (PoseResult drives confidence tagging)
O2 (precision benchmarks)     — requires A3, A4
O3 (perf benchmarks)          — requires A1, A3

CP1 (RgbdFrame)               — standalone; consumed by edgesentry-inspect M6 RGB-D extension
CP2 (IFC annotation types)    — standalone; consumed by edgesentry-inspect M7
```

---

## What is explicitly out of scope for trilink-core

The following are real problems but belong to a higher layer:

| Problem | Correct location |
|---|---|
| IFC 4.3 `IfcPropertySet` XML/STEP serialisation | `edgesentry-inspect` M7 |
| ONNX model training and weight files | `edgesentry-inspect` M6 |
| MLIT / CONQUAS equivalence documentation | commercial compliance layer |
| Tamper-evidence / electronic signature | `edgesentry-audit` |
| Multi-pose capture scheduling | sensor platform firmware |
