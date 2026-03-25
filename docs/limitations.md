# Known Limitations and Verification Guide

This document describes known limitations of the current projection and inference pipeline, and the benchmark suite required to verify precision and performance claims before production deployment.

The limitations below are inherent to the current design choices — not bugs. Each entry states what the constraint is, why it exists, and how to detect when you are operating outside safe boundaries.

---

## Geometric Limitations

### 1. Single-viewpoint occlusion

`project_to_depth_map` uses a Z-buffer: for any pixel, only the point closest to the camera is retained. Surfaces not visible from the capture pose — bridge undersides, the rear faces of structural members, pier bases — are absent from the depth map with no warning.

**Impact:** Defects on occluded faces are undetectable from a single frame. The deviation engine (`M2`) will report `compliant` for regions with no point coverage.

**Mitigation:** Collect frames from multiple poses and fuse depth maps before running inference. Multi-view fusion is not currently implemented. Until it is, inspection plans must ensure full surface coverage across capture poses.

**Detection signal:** After projection, count `NaN` cells in the depth map. A high `NaN` fraction (>30%) for a region of interest indicates poor coverage.

---

### 2. Height map detects protrusions only

`project_to_height_map` stores the **maximum Z** per cell. This design detects features that rise above the design surface (raised joints, unexpected objects, weld beads). It is structurally blind to depressions: spalling craters, potholes, section loss, and corroded voids will not appear as anomalies — they present as below-maximum-Z and are overwritten.

**Impact:** For any inspection task where material loss is the primary failure mode, the height map alone is insufficient.

**Mitigation:** Consider storing minimum Z (detects depressions) or Z-range / standard deviation (detects both) per cell. This is a design change to `project_to_height_map` that requires an API decision.

---

### 3. Back-projection error on curved surfaces

The unprojection step (`unproject`) assumes a flat projection plane. For cylindrical columns, arch ribs, and curved pier faces, the recovered world-space position carries a systematic bias that grows with surface curvature.

Published benchmarks on concrete infrastructure report:

| Surface type | Relative error (crack width) |
|---|---|
| Flat (columns, walls) | ~2.5% |
| Curved (cylinders, arches) | ~11.7% |

These figures are from controlled laboratory conditions. Field accuracy may differ.

**Impact:** Crack width and defect size measurements derived from `unproject` are less reliable on curved geometry. The deviation engine is unaffected (it operates in world space against the IFC reference cloud).

**Mitigation (current):** Flag detections whose back-projected region has high surface curvature (estimated from the IFC reference cloud surface normals) and apply an expanded tolerance in the report.

**Mitigation (future):** Implement surface-normal-aware back-projection that accounts for the angle between the projection ray and the local surface.

---

### 4. f32 coordinate precision outside local frame

`Point3D` stores `x, y, z` as `f32` (~7 significant decimal digits, machine epsilon ≈ 1.2 × 10⁻⁷).

This is sufficient for any site-scale local frame (≤1 km from the frame origin → sub-millimetre representable step). It is **not** sufficient for absolute geodetic coordinates:

| Coordinate magnitude | f32 representable step | Adequate for 5 mm target? |
|---|---|---|
| 1 km (local frame) | ~0.12 mm | Yes |
| 10 km | ~1.2 mm | Yes |
| 100 km (UTM scale) | ~12 mm | **No** |

**Impact:** Feeding UTM or WGS-84 coordinates directly into `Point3D` silently degrades precision to ~12 mm at continental scale, making the 5–10 mm deviation threshold meaningless.

**Requirement:** All `Point3D` values must be in a local tangent-plane frame (e.g. ENU centred on the site datum). If your pipeline ingests absolute coordinates, subtract the site origin before constructing `Point3D`. See [math.md](math.md) for the full precision table.

**Future guard:** A debug assertion firing when `|x|`, `|y|`, or `|z| > 10_000.0` would catch misuse at development time.

---

## Inference Limitations

### 5. Depth-only input — missing texture channel

The current built-in inference path (`M6`) feeds a depth map (single-channel) to the ONNX model. RGB image data (`FusionPacket.image_jpeg`) is not used during inference.

Published F1 scores on concrete damage detection:

| Input modality | F1 score |
|---|---|
| 2D RGB image only | 67.6% |
| 3D depth map only | 76.0% |
| 3D depth + 2D RGB (fused) | **86.7%** |

Running on depth alone leaves ~10 percentage points of achievable F1 on the table. Defect classes whose primary signal is colour or texture — rust, efflorescence, water staining, surface discolouration — are not detectable from a depth map alone.

**Mitigation (planned):** Extend the `InferenceBackend` input to accept an optional RGB channel alongside the depth map, forming an RGB-D input tensor. The `FusionPacket` already carries `image_jpeg`; the main change is in the inference module and model retraining.

---

### 6. Fallback depth degrades world-space localisation

When no depth measurement is available for a detection, `fallback_depth_m` (default: 2.0 m) is substituted. The world-space localisation error for a detection on a surface at true depth `d_true` is:

```
position_error ≈ |d_true − fallback_depth_m| × angular_size_of_bbox
```

For a 10° field-of-view defect at true depth 4 m with fallback 2 m, the world-space centre position is off by ~0.35 m.

**Impact:** Any detection without a valid depth reading has an unreliable `world_pos`. The deviation comparison against the IFC reference cloud will assign it to a potentially wrong structural member.

**Mitigation:** Always deploy with a range sensor (ToF, LiDAR) co-registered with the camera. For inspections where depth is temporarily unavailable (sensor dropout, water splash), mark affected detections with `depth_m = None` and exclude them from measurement outputs — treat them as positional annotations only.

---

## Temporal Limitations

### 7. Pose lookup dead zone at buffer boundary

`PoseBuffer` holds the last 1,000 poses. At 30 Hz this covers ~33 seconds. Any inference result whose `input_ts` falls outside this window returns `None` from `pose_at` and the detection is dropped silently.

At 30 Hz with a 200 ms tolerance window, inference results that take >200 ms are also dropped.

**Impact:** Under high inference latency (slow model, network congestion) or if the platform pauses for >33 seconds, detections are lost without any error surfaced to the operator.

**Mitigation:** Monitor the fraction of detections where `world_pos = None` after the unproject step. A rising `None` rate signals pose lookup failures. Consider logging a `WARN` when `pose_at` returns `None` due to tolerance exceeded vs. buffer exhausted — these are different failure modes.

---

## Regulatory Limitations

### 8. AI output is not yet "near-visual-inspection equivalent"

Japan's Road Act enforcement regulations (道路法施行規則) and MLIT periodic inspection guidelines require that any alternative inspection method demonstrate equivalence to close visual inspection (近接目視). Singapore's CONQUAS standard similarly requires certified assessor sign-off.

The current OSS pipeline produces a deviation report and defect detections, but:

- There is no documented equivalence test comparing AI detections against human inspector findings on the same structure.
- The JSON report has no electronic signature or tamper-evidence chain (the commercial audit connector in M5/M7 addresses this).
- IFC 4.3 metadata write-back (mapping AI detections to `IfcPropertySet` attributes on specific structural members) is not yet implemented in the OSS layer.

**Impact:** Reports produced by the current pipeline alone cannot be submitted as official inspection records to MLIT or BCA without additional certification steps.

---

## Precision and Performance Verification

The following test suite is required before claiming production-grade precision on any deployment.

### P1 — Flat-surface round-trip error

**What:** Project a synthetic point cloud of a flat surface, inject a known defect at a known pixel, run unproject, compare recovered world position against ground truth.

**Pass criterion:** World-space position error ≤ 1 mm at range ≤ 10 m.

**Existing coverage:** `#34` round-trip tests exist; extend with explicit millimetre-level error assertion.

---

### P2 — Curved-surface bias characterisation

**What:** Same as P1 but on a cylindrical surface. Vary curvature radius (0.1 m, 0.5 m, 1.0 m, ∞). Plot back-projection error vs. radius.

**Pass criterion:** Document the error curve and set an explicit curvature threshold above which the report flags measurements as "reduced precision".

---

### P3 — Controlled deformation benchmark (quantitative)

**What:** Extend the Demo Pipeline. Introduce deformations of 3 mm, 5 mm, 10 mm, and 20 mm. For each, check that `max_deviation_mm` in the report is within ±2 mm of the true value.

**Pass criterion:** ±2 mm for deformations ≥ 5 mm; ±1 mm for deformations ≥ 10 mm.

---

### P4 — RGB-D fusion ablation

**What:** On the same labelled dataset, run inference with (a) depth-only and (b) RGB-D input. Record precision, recall, and F1 per defect class.

**Pass criterion:** RGB-D F1 ≥ depth-only F1 + 5 pp; overall F1 ≥ 80%.

**Prerequisite:** M6 RGB-D extension implemented.

---

### P5 — Fallback-depth sensitivity sweep

**What:** Run the full pipeline with `fallback_depth_m` ∈ {1.0, 1.5, 2.0, 3.0, 5.0} m against a scene where true depth is known. Record world-space localisation error per setting.

**Pass criterion:** Document the operating range. Recommend that deployments always use sensor depth; document fallback as an emergency mode only.

---

### Perf1 — Point cloud scale benchmark

**What:** Measure wall time of `project_to_depth_map` and `compute_deviation` at 50K, 200K, and 500K points on a representative field PC (x86-64, no GPU).

**Pass criterion:**

| Points | `project_to_depth_map` | `compute_deviation` |
|---|---|---|
| 50K | ≤ 20 ms | ≤ 50 ms |
| 200K | ≤ 80 ms | ≤ 200 ms |
| 500K | ≤ 200 ms | ≤ 500 ms |

---

### Perf2 — PoseBuffer overflow behaviour

**What:** Feed 1,001 poses into `PoseBuffer`, then query the oldest timestamp. Verify it returns `None` (evicted), not a wrong value. Verify subsequent queries within the window still succeed.

**Pass criterion:** No panic, correct `None` for evicted entries, no stale-data return.
