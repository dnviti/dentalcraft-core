# dentalcraft-core

Headless engine of [DentalCraft](https://dentalcraft.app), a desktop application for digital dentistry, written in Rust.

This workspace has no window and needs no GPU. It holds everything that does not draw pixels: meshes and geometry, the case model, `.dcprj` project files, undoable commands, background jobs, the module API, 3D scanner and turntable drivers, and an MCP server so that AI agents can drive the engine.

> **Status: alpha.** The APIs, the file format and the commands can still change between releases.
>
> **Not a medical device.** DentalCraft is not certified. It must not be used on patients or for clinical decisions. See [Disclaimer](#disclaimer).

## Contents

| Crate | What it does |
|---|---|
| `dc-mesh` | Triangle mesh container, topology queries, validation, mesh file I/O (STL, OBJ, PLY, and the internal `DCM1` blob). |
| `dc-geom` | Geometry kernels behind a `Backend` trait: BVH, nearest neighbours, ICP, QEM decimation, hole filling, SDF, TSDF, sparse solvers. |
| `dc-case` | Case model: scans, FDI tooth numbering, poses, world frame, content-addressed blobs, per-module state. |
| `dc-command` | Undoable commands, undo stack and the processing step history. |
| `dc-jobs` | Background jobs with progress reporting and cancellation. |
| `dc-scene` | GPU-agnostic description of what a viewport draws: meshes, lines, points, labels, overlays. |
| `dc-project` | The `.dcprj` project format (ZIP container, BLAKE3 checks, atomic save). |
| `dc-module-api` | The `Module` trait, menu tree, command descriptors, registry and codecs. Dental modules plug in here. |
| `dc-session` | Headless engine session: project, case state, undo, step history, jobs and the registry of agent-drivable commands. |
| `dc-mcp` | MCP server (JSON-RPC over stdio) on top of a session or a running app. |
| `dc-raster` | CPU rasteriser for headless renders and screenshots (PNG). |
| `dc-scanner` | 3D scanner acquisition (alpha): device backends, live tracking and TSDF fusion, point-cloud and mesh editing, turntable rig scans. |
| `dc-turntable` | Motorised turntables for scanning rigs (alpha): generic device trait, simulated table, Revopoint Dual-axis Turntable over Bluetooth LE. |
| `dc-ble` | Blocking Bluetooth LE serial links: BlueZ on Linux, WinRT on Windows, CoreBluetooth on macOS. |
| `dc-print` | Resin printing (alpha): printer profiles (Elegoo Saturn 4 Ultra 16K with the DENTAL 45 machine parameters), automatic orientation, outer-side supports, plate nesting, slicing and the Elegoo GOO V3 file format. |
| `dc-i18n` | Fluent localisation (English, Italian) with a runtime language switch. |
| `dc-theme` | Design tokens and their application to egui. |

Hardware support is extensible: a new scanner or turntable is one more backend in `dc-scanner` or `dc-turntable`.

## Build and test

You need the Rust toolchain pinned in [`rust-toolchain.toml`](rust-toolchain.toml) (`rustup` installs it on first use). No C++ toolchain is needed.

```sh
cargo build --workspace
cargo fmt --check
cargo clippy --all-targets -- -D warnings
cargo test
```

Tests that need real scans or hardware are skipped unless you set their environment variables. Never commit patient scans or datasets.

## Architecture

- [`dentalcraft`](https://github.com/dnviti/dentalcraft) (superproject): pins all repositories and holds the architecture decision records in `docs/adr`.
- [`dentalcraft-docs`](https://github.com/dnviti/dentalcraft-docs): documentation.
- Layering: this repository depends on no other DentalCraft repository. [`dentalcraft-aligners`](https://github.com/dnviti/dentalcraft-aligners) depends on core. [`dentalcraft-app`](https://github.com/dnviti/dentalcraft-app) depends on core and aligners.

## Licence

Source-available under the [PolyForm Noncommercial License 1.0.0](LICENSE.md). You may read, use, change and share the code for any noncommercial purpose. Commercial use is not allowed.

This is not an open source licence in the OSI sense. Third-party components keep their own licences: see [`NOTICE.md`](NOTICE.md).

**Commercial licensing.** If you want to use DentalCraft commercially, contact the author at hello@dentalcraft.app.

## Disclaimer

DentalCraft is not a certified medical device and has no regulatory clearance. It is provided as is, without warranty. Do not use it on patients or for clinical decisions without independent professional validation. See [`SECURITY.md`](SECURITY.md).

## Contributing

Read [`CONTRIBUTING.md`](CONTRIBUTING.md) (DCO sign-off, inbound licence grant, checks) and the [Code of Conduct](CODE_OF_CONDUCT.md). Report security problems privately, as described in [`SECURITY.md`](SECURITY.md).

## Links

| | |
|---|---|
| This repository | https://github.com/dnviti/dentalcraft-core |
| Superproject | https://github.com/dnviti/dentalcraft |
| Aligners module | https://github.com/dnviti/dentalcraft-aligners |
| Application | https://github.com/dnviti/dentalcraft-app |
| Website | https://github.com/dnviti/dentalcraft-web |
| Documentation | https://github.com/dnviti/dentalcraft-docs |
| Product site | https://dentalcraft.app |

Versions are cut by [release-please](https://github.com/googleapis/release-please) from Conventional Commits on `main` (tags `vX.Y.Z`).
