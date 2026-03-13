# AGENTS.md — Tri-Link Codebase Guide

This file is for AI agents working in this repository. It describes the codebase structure, build system, key types, and conventions.

---

## Purpose

Tri-Link is a Cargo workspace with two crates:

- **`core/`** (`trilink-core`) — open source (MIT/Apache-2.0). Core data types, pose buffer, unprojection math, and the `RobotSource` trait. Intended for eventual extraction to a separate public repository.
- **`app/`** (`tri-link`) — closed source (enterprise). HTTP inference client, semantic map output, bridge assembly, and the CLI binary. Depends on `trilink-core`.

Entry point: `app/src/main.rs` → `trilink run --config config.toml`

---

## Module Map

```
core/src/
  lib.rs              Core data types: InspectionPacket, Detection, BBox2D,
                      Transform4x4, CameraIntrinsics, Point3D
  error.rs            TriError — unified error enum (thiserror)
  ingress/
    mod.rs            RobotSource trait + RobotFrame struct
    mock.rs           MockSource: deterministic frames for testing
  buffer/
    mod.rs            PoseBuffer: ring buffer keyed by capture_ts_us
  bridge/
    mod.rs            re-exports unproject
    unproject.rs      (M2) (u,v,depth) → world (X,Y,Z)

app/src/
  lib.rs              Re-exports core types; declares closed-source modules
  bridge/
    mod.rs            re-exports sync
    sync.rs           (M4) join inference result → pose by timestamp
  infer/
    mod.rs            InferenceClient (M3)
    types.rs          API request / response types (M3)
  egress/
    mod.rs            DamageRecord output (M5)
    semantic_map.rs   JSONL + SQLite writer (M5)
  main.rs             CLI: clap Subcommand::Run { config }
```

Modules marked `(M2)`, `(M3)`, `(M4)`, `(M5)` are stubs — not yet implemented.

---

## Key Types

| Type | Crate / File | Purpose |
|---|---|---|
| `InspectionPacket` | `core/src/lib.rs` | One complete damage event: pose + image + detections |
| `Detection` | `core/src/lib.rs` | Single bounding box with optional world position |
| `BBox2D` | `core/src/lib.rs` | Pixel-space bounding box; `.center()` returns `(f64, f64)` |
| `Transform4x4` | `core/src/lib.rs` | Row-major 4×4 pose matrix; `.identity()` for tests |
| `CameraIntrinsics` | `core/src/lib.rs` | `fx, fy, cx, cy` pinhole parameters |
| `PoseBuffer` | `core/src/buffer/mod.rs` | Ring buffer; `.push(ts, pose)` / `.pose_at(ts)` |
| `RobotSource` | `core/src/ingress/mod.rs` | Trait for frame sources; implement for real hardware |
| `MockSource` | `core/src/ingress/mock.rs` | Deterministic test source; use `.with_limit(n)` in tests |
| `TriError` | `core/src/error.rs` | All error variants; feature-gated `Sqlite` variant |
| `DamageRecord` | `app/src/egress/mod.rs` | (M5) Output record written to JSONL and SQLite per detection |

---

## Build and Test

See [docs/contributing.md](docs/contributing.md) for all build, test, and contribution commands.

---

## Conventions

- **Error handling**: use `TriError`; propagate with `?`; no `.unwrap()` in library code
- **Timestamps**: always `u64` microseconds since UNIX epoch (`capture_ts_us`)
- **Pose matrix**: row-major `[f32; 16]`, world frame
- **No company or product names** in any source file or document — use generic terms (`robot platform`, `inference service`)
- **Feature gate SQLite**: anything touching `rusqlite` must be behind `#[cfg(feature = "sqlite")]`

---

## Reference Documents

| Document | Contents |
|---|---|
| [README.md](README.md) | High-level overview |
| [docs/background.md](docs/background.md) | Project context, why it exists, future applications, open source strategy |
| [docs/scenarios.md](docs/scenarios.md) | End-to-end scenarios, deployment difficulty, case studies, implementation order |
| [docs/architecture.md](docs/architecture.md) | Data flow, module map, data structures, crate deps |
| [docs/math.md](docs/math.md) | Coordinate systems, camera model, unprojection math (prerequisite reading) |
| [docs/assumptions.md](docs/assumptions.md) | Role boundaries, interface contracts, deployment decision |
| [docs/roadmap.md](docs/roadmap.md) | Implementation milestones M1–M6 with acceptance criteria |
| [docs/contributing.md](docs/contributing.md) | Build, test, and contribution commands |
| [config.example.toml](config.example.toml) | All configuration fields with example values |
