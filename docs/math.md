# Math Concepts

This document explains the geometric and linear algebra concepts needed before implementing `bridge/unproject.rs`. No prior robotics or computer vision background is assumed.

---

## Coordinate Systems

Three coordinate systems are in play. Understanding which frame a point lives in is the most important concept to get right.

```
Image space          Camera space         World space
(pixels)             (meters,             (meters,
                      camera-centered)     map-centered)

(u, v)  ──── K⁻¹ ──► (Xc, Yc, Zc) ── T ──► (Xw, Yw, Zw)
             ↑                           ↑
         intrinsics                    pose
         (static)                  (per frame)
```

**Image space** — what the camera sensor produces. A point is `(u, v)` in pixels. The top-left corner is `(0, 0)`. `u` increases to the right, `v` increases downward.

**Camera space** — a 3D coordinate system centred on the camera lens. The Z axis points straight ahead (into the scene). X points right, Y points down. Units are metres. A point at `(Xc, Yc, Zc)` is `Zc` metres in front of the camera.

**World space** — the global 3D coordinate system established by the platform's SLAM map. A `Detection`'s `world_pos` is in this frame.

---

## Homogeneous Coordinates

To represent both rotation and translation in a single matrix multiplication, we add one extra dimension. A 3D point `(x, y, z)` becomes `(x, y, z, 1)` in homogeneous form.

```
Regular:       (x, y, z)
Homogeneous:   (x, y, z, 1)
```

A 4×4 matrix can then apply rotation, translation, and scaling in one step:

```
[ R  t ] [ x ]   [ x' ]
[ 0  1 ] [ y ] = [ y' ]
         [ z ]   [ z' ]
         [ 1 ]   [  1 ]
```

Where `R` is a 3×3 rotation matrix and `t` is a 3×1 translation vector.

`Transform4x4` in the codebase stores this 4×4 matrix as a row-major `[f32; 16]` array.

---

## The Camera Intrinsics Matrix (K)

The intrinsics matrix `K` encodes the camera's optical properties. It is a 3×3 matrix:

```
K = [ fx   0   cx ]
    [  0  fy   cy ]
    [  0   0    1 ]
```

| Parameter | Meaning |
|---|---|
| `fx` | Focal length in pixels, horizontal axis |
| `fy` | Focal length in pixels, vertical axis |
| `cx` | Horizontal pixel coordinate of the image centre (principal point) |
| `cy` | Vertical pixel coordinate of the image centre (principal point) |

**Intuition:** `fx` and `fy` control how strongly perspective compresses depth into pixels. Larger values = more telephoto (distant objects appear larger). `cx`, `cy` describe where the lens optical axis hits the sensor — ideally the image centre, but rarely exactly so after manufacturing.

These values come from the sensor platform's calibration. They do not change during a session.

---

## Pinhole Camera Model

The pinhole model describes how a 3D point in camera space projects onto the image plane.

**Forward projection (3D → pixel):**

```
u = fx · (Xc / Zc) + cx
v = fy · (Yc / Zc) + cy
```

**Intuition:** Dividing by `Zc` is perspective division — objects further away (larger `Zc`) map to pixels closer to the centre. Multiplying by `fx`/`fy` scales by focal length. Adding `cx`/`cy` shifts to image coordinates.

---

## Unprojection (Pixel → 3D)

Tri-Link needs the reverse: given a pixel `(u, v)` and a depth `d`, find the 3D camera-space point.

**Step 1 — Normalise pixel to image-plane ray:**

```
x_norm = (u - cx) / fx
y_norm = (v - cy) / fy
```

This removes the camera's focal length scale and principal point offset, giving a direction vector in normalised image coordinates (camera space ray direction).

**Step 2 — Scale by depth:**

```
Xc = x_norm · d
Yc = y_norm · d
Zc = d
```

The result `(Xc, Yc, Zc)` is the 3D point in camera space, `d` metres from the lens along the viewing ray.

Written as a matrix operation (equivalent form):

```
P_camera = K⁻¹ · [u, v, 1]ᵀ · d
```

Where:

```
K⁻¹ = [ 1/fx    0   -cx/fx ]
      [    0  1/fy  -cy/fy ]
      [    0     0       1 ]
```

---

## Pose Transform (Camera → World)

The platform's pose `T` is a 4×4 homogeneous matrix that expresses the camera's position and orientation in world space. It is recorded at the exact moment the shutter opens.

To convert a camera-space point to world space:

```
P_world = T · [Xc, Yc, Zc, 1]ᵀ
```

The result `P_world = (Xw, Yw, Zw, 1)` is the world-space position. Drop the trailing `1` to get the `Point3D` stored in `Detection.world_pos`.

---

## Full Unprojection Pipeline

Putting it all together, from a bounding box to a world-space point:

```
1. Compute bounding box centre:
   u = (bbox.u0 + bbox.u1) / 2
   v = (bbox.v0 + bbox.v1) / 2

2. Normalise to camera ray:
   x_norm = (u - cx) / fx
   y_norm = (v - cy) / fy

3. Scale by depth d (from ToF, or fallback_depth_m):
   P_camera = (x_norm · d,  y_norm · d,  d)

4. Apply pose transform:
   P_world = T · (P_camera, 1)
```

This is exactly what `bridge/unproject.rs` implements.

---

## The Pose Buffer and Timestamp Lookup

The pose `T` used in step 4 above must be the pose at the moment the image was captured — not the current platform pose. This is what `PoseBuffer` handles.

The buffer stores a time series of `(timestamp_us, Transform4x4)` pairs. When a lookup is requested for `capture_ts_us`, the buffer finds the entry whose timestamp is closest, provided it falls within a tolerance window (default 200 ms).

Implemented as a sorted ring buffer with binary search:

```
entries sorted by ts_us:
  ... [ts=1000] [ts=1033] [ts=1067] [ts=1100] ...

query: pose_at(1050)
  → binary search finds ts=1033 and ts=1067
  → pick the closer one: ts=1033 (delta=17) vs ts=1067 (delta=17) → tie → either
  → if delta > tolerance_us → return None
```

**Why this matters:** At 1 m/s platform speed, a 100 ms pose error = 10 cm position error. Using the captured pose instead of the current pose removes this error entirely.

---

## Forward Projection (3D → Pixel)

The inverse of unprojection: given a 3D world-space point and the camera pose, compute the pixel it lands on and its depth. Used by `bridge/project.rs` to convert a LiDAR point cloud into a depth map.

### Step 1 — World → camera (rigid-body inverse)

The pose matrix `T` encodes `P_world = R · P_camera + t`, so the inverse is:

```
P_camera = Rᵀ · (P_world − t)
```

Expanding with the row-major matrix layout (`m[row*4 + col]`):

```
tx = m[3],  ty = m[7],  tz = m[11]

Xc = m[0]·(Xw−tx) + m[4]·(Yw−ty) + m[8]·(Zw−tz)
Yc = m[1]·(Xw−tx) + m[5]·(Yw−ty) + m[9]·(Zw−tz)
Zc = m[2]·(Xw−tx) + m[6]·(Yw−ty) + m[10]·(Zw−tz)
```

**Why `Rᵀ` and not `R⁻¹`?** For rotation matrices `R⁻¹ = Rᵀ` (orthonormality). This avoids any general matrix inversion and keeps the operation cheap.

### Step 2 — Behind-camera cull

Discard points with `Zc ≤ 0`. These are behind the image plane and cannot be projected to a valid pixel.

### Step 3 — Pinhole projection

```
u = fx · (Xc / Zc) + cx
v = fy · (Yc / Zc) + cy
```

The perspective divide by `Zc` compresses depth: distant objects cluster towards `(cx, cy)`.

### Step 4 — Bounds cull

Discard pixels where `u ∉ [0, W)` or `v ∉ [0, H)`. These points exist in the scene but fall outside the sensor's field of view.

### Step 5 — Z-buffer

Multiple world points may project to the same pixel (occlusion). Keep the one with the smallest `Zc` (nearest to the camera). This matches the physical reality of what a camera sees.

### Full forward projection pipeline

```
For each P_world in point cloud:
  1. P_camera = Rᵀ · (P_world − t)
  2. If Zc ≤ 0 → skip
  3. u = fx·(Xc/Zc) + cx,  v = fy·(Yc/Zc) + cy
  4. If u,v outside image → skip
  5. depth_map[v][u] = min(depth_map[v][u], Zc)
```

This is exactly what `bridge/project::project_to_depth_map` implements.

---

## Height Map Projection

A height map is a top-down orthographic view of the scene. Unlike the depth map, it operates entirely in world space — no camera pose is involved.

### Cell index formula

For a grid with top-left corner at `(origin_x, origin_y)` and cell size `resolution_m`:

```
col = floor((P_world.x − origin_x) / resolution_m)
row = floor((P_world.y − origin_y) / resolution_m)
```

Points outside `[0, cols) × [0, rows)` are silently skipped.

### Aggregation: maximum Z

Each cell stores the **maximum Z** of all points that fall into it. This detects protrusions above the design surface (e.g. unexpected objects on a floor, raised features on a structure). Cells with no point are left as `f32::NAN`.

### Full height map pipeline

```
For each P_world in point cloud:
  col = floor((Xw − origin_x) / resolution_m)
  row = floor((Yw − origin_y) / resolution_m)
  If (col, row) outside grid → skip
  height_map[row][col] = max(height_map[row][col], Zw)
```

This is exactly what `bridge/project::project_to_height_map` implements.

---

## Numerical Ranges to Expect

When implementing and debugging unprojection, these are typical values for a vehicle side-scan at ~2 m distance:

| Variable | Typical range |
|---|---|
| `u`, `v` | 0–1920, 0–1080 (1080p camera) |
| `cx`, `cy` | ~960, ~540 (near image centre) |
| `fx`, `fy` | 800–1500 pixels (typical automotive camera) |
| `d` (depth) | 0.5–5.0 m |
| `Xc`, `Yc` | ±1.5 m at 2 m depth |
| `Xw`, `Yw`, `Zw` | site-scale metres, local frame from map origin |
| Point cloud size | 50K–500K points per sweep |
| `resolution_m` (height map) | 0.005–0.02 m (5–20 mm per cell) |
| Deviation threshold | 0.005–0.01 m (5–10 mm) |

## Coordinate Precision and `f32`

`Point3D` stores coordinates as `f32`. `f32` has machine epsilon ≈ 1.2 × 10⁻⁷, giving ~7 significant decimal digits.

**Requirement:** all `Point3D` values must be expressed in a **local coordinate frame** — i.e. as offsets from a nearby origin such as the SLAM map start position, not as absolute geodetic coordinates (UTM, WGS-84).

| Max coordinate magnitude | `f32` representable step | Adequate for 1 cm target? |
|---|---|---|
| 100 m (large building) | ~0.012 mm | Yes |
| 1 km (city block) | ~0.12 mm | Yes |
| 10 km | ~1.2 mm | Yes |
| 100 km (UTM scale) | ~12 mm | **No** |

The system targets 1 cm accuracy over construction-site distances (≤ 1 km). `f32` is sufficient. `CameraIntrinsics` uses `f64` and `transform_point` promotes to `f64` during the matrix multiply, so projection arithmetic is double-precision; only the final stored `Point3D` result is rounded to `f32`.

If absolute geodetic coordinates are required in future, convert to a local tangent-plane frame (e.g. ENU centred on the site datum) before constructing `Point3D` values.
