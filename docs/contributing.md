# Contributing

## Quick Start

```bash
cp config.example.toml config.toml   # fill in endpoints and API key
cargo build --release
trilink run --config config.toml
```

## Build and Test

```bash
cargo build                          # debug build
cargo build --release                # release build
cargo test                           # all unit tests
cargo test --features sqlite         # include SQLite feature
cargo clippy --all-targets -- -D warnings
cargo deny check licenses
```

All tests must pass before any commit. No warnings are allowed (`-D warnings`).

## Mandatory: Run Tests After Every Code Change

After **every** code change, run:

```bash
cargo test
```

Do not consider a change complete until all tests pass.

## Consistency Check

After every change — whether to code, tests, or docs — verify these stay in sync:

1. **Code → Docs**: If you add, remove, or rename a module, function, CLI command, or type, update all docs that reference it.
2. **Docs → Code**: If a doc describes a feature or command, verify it exists and works as described.
3. **AGENTS.md module map**: Keep the module map and key types table current with the actual `src/` layout.
