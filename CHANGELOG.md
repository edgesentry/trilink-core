# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.2.0] - 2026-10-22

### Changed

- **Breaking:** Bump public dependency [`glam`](https://crates.io/crates/glam) from `0.32`/`0.33` to `0.34` (with the `serde` feature). Types such as `Transform4x4.mat: glam::Mat4` are part of the public API, so dependents (for example `edgesentry-inspect`) must upgrade to glam `0.34` in lockstep.
- Declare crate MSRV as Rust `1.85` (required by glam `0.34`).

### Fixed

- Pin all GitHub Actions `uses:` entries to full commit SHAs and align CI toolchain/cache steps with org allowlist policy (plain `rustup` + `actions/cache`).
- Remove obsolete `--features sqlite` steps from the release quality gate (SQLite support has moved out of this crate).

## [0.1.1] - previous

See git history and crates.io for earlier releases.
