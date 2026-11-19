# Changelog

## [0.3.0-alpha.6](https://github.com/dnviti-personal-org/dentalcraft-core/compare/v0.3.0-alpha.5...v0.3.0-alpha.6) (2026-11-19)


### Features

* **geom:** faithful scan finish with surface bound, normal and edge caps and denoising ([75e513f](https://github.com/dnviti-personal-org/dentalcraft-core/commit/75e513f06612e7a0bc39578fc95178b37c9aa144))
* **geom:** weighted TSDF fusion with de-integration and a one-pass mesh finish ([75e513f](https://github.com/dnviti-personal-org/dentalcraft-core/commit/75e513f06612e7a0bc39578fc95178b37c9aa144))
* **scanner:** drive the MINI 2 fill light while streaming ([75e513f](https://github.com/dnviti-personal-org/dentalcraft-core/commit/75e513f06612e7a0bc39578fc95178b37c9aa144))
* **scanner:** fine 0.1 mm reconstruction and finish of automatic scans from the kept stops (scan.reconstruct, scan.mesh finish, scan.export stops) ([75e513f](https://github.com/dnviti-personal-org/dentalcraft-core/commit/75e513f06612e7a0bc39578fc95178b37c9aa144))
* **scanner:** scan.close ends the scan session ([75e513f](https://github.com/dnviti-personal-org/dentalcraft-core/commit/75e513f06612e7a0bc39578fc95178b37c9aa144))
* **scanner:** stand an uncalibrated scan on the true up from the IMU ([75e513f](https://github.com/dnviti-personal-org/dentalcraft-core/commit/75e513f06612e7a0bc39578fc95178b37c9aa144))
* **scanner:** use the MINI 2 factory calibration and a fast exposure profile ([75e513f](https://github.com/dnviti-personal-org/dentalcraft-core/commit/75e513f06612e7a0bc39578fc95178b37c9aa144))
* **turntable:** detect a dropped Bluetooth link at once and reconnect the table with a continuous angle ([75e513f](https://github.com/dnviti-personal-org/dentalcraft-core/commit/75e513f06612e7a0bc39578fc95178b37c9aa144))


### Bug Fixes

* **scanner:** end the acquisition before the finish of an automatic scan ([75e513f](https://github.com/dnviti-personal-org/dentalcraft-core/commit/75e513f06612e7a0bc39578fc95178b37c9aa144))
* **scanner:** leave out the views of an object that moved before fusing a scan, with safe scan.auto defaults ([75e513f](https://github.com/dnviti-personal-org/dentalcraft-core/commit/75e513f06612e7a0bc39578fc95178b37c9aa144))
* **scanner:** let a host finish a rig job that ended without a command ([75e513f](https://github.com/dnviti-personal-org/dentalcraft-core/commit/75e513f06612e7a0bc39578fc95178b37c9aa144))
* **scanner:** read the MINI 2 frame header of the depth-only stream ([75e513f](https://github.com/dnviti-personal-org/dentalcraft-core/commit/75e513f06612e7a0bc39578fc95178b37c9aa144))
* **scanner:** read the turntable angle of a frame with a latency that follows the frame rate ([75e513f](https://github.com/dnviti-personal-org/dentalcraft-core/commit/75e513f06612e7a0bc39578fc95178b37c9aa144))
* **scanner:** refuse a calibration whose tilt fails with its own code ([75e513f](https://github.com/dnviti-personal-org/dentalcraft-core/commit/75e513f06612e7a0bc39578fc95178b37c9aa144))
* **scanner:** reliable turntable tracking, stop-and-capture scanning and a validated turntable calibration ([75e513f](https://github.com/dnviti-personal-org/dentalcraft-core/commit/75e513f06612e7a0bc39578fc95178b37c9aa144))
* **scanner:** remove the static clutter of a revolution from stop-and-capture scans ([75e513f](https://github.com/dnviti-personal-org/dentalcraft-core/commit/75e513f06612e7a0bc39578fc95178b37c9aa144))
* **scanner:** report the residual at the turntable pose of a flagged stop ([75e513f](https://github.com/dnviti-personal-org/dentalcraft-core/commit/75e513f06612e7a0bc39578fc95178b37c9aa144))
* **scanner:** resume a rig job after a turntable link drop and keep the stops of a failed scan ([75e513f](https://github.com/dnviti-personal-org/dentalcraft-core/commit/75e513f06612e7a0bc39578fc95178b37c9aa144))
* **scanner:** seed the tilt axis registration down to 70 mm under the platter ([75e513f](https://github.com/dnviti-personal-org/dentalcraft-core/commit/75e513f06612e7a0bc39578fc95178b37c9aa144))
* **scanner:** start a rig job once the turntable stands still ([75e513f](https://github.com/dnviti-personal-org/dentalcraft-core/commit/75e513f06612e7a0bc39578fc95178b37c9aa144))
* **scanner:** the scan world frame is the turntable frame, with the platter on XY ([75e513f](https://github.com/dnviti-personal-org/dentalcraft-core/commit/75e513f06612e7a0bc39578fc95178b37c9aa144))
* **scanner:** tilt the table back to 0 when a scan ends ([75e513f](https://github.com/dnviti-personal-org/dentalcraft-core/commit/75e513f06612e7a0bc39578fc95178b37c9aa144))
* **scanner:** warn on a weak automatic scan from its residual and loop error ([75e513f](https://github.com/dnviti-personal-org/dentalcraft-core/commit/75e513f06612e7a0bc39578fc95178b37c9aa144))


### Performance Improvements

* **geom:** finish a dense scan in parallel within the same 0.015 mm bound ([75e513f](https://github.com/dnviti-personal-org/dentalcraft-core/commit/75e513f06612e7a0bc39578fc95178b37c9aa144))

## [0.3.0-alpha.5](https://github.com/dnviti-personal-org/dentalcraft-core/compare/v0.3.0-alpha.4...v0.3.0-alpha.5) (2026-11-18)


### Bug Fixes

* **scanner:** capture the MINI 2 on Windows through Media Foundation ([aa7fbb0](https://github.com/dnviti-personal-org/dentalcraft-core/commit/aa7fbb08299d2de7669f86ffec50ddf391b0c9da))
* **scanner:** stop rejecting depth frame headers on a non-constant word ([aa7fbb0](https://github.com/dnviti-personal-org/dentalcraft-core/commit/aa7fbb08299d2de7669f86ffec50ddf391b0c9da))

## [0.3.0-alpha.4](https://github.com/dnviti-personal-org/dentalcraft-core/compare/v0.3.0-alpha.3...v0.3.0-alpha.4) (2026-11-18)


### Features

* **module-api:** let modules declare scanner input and roles ([#15](https://github.com/dnviti-personal-org/dentalcraft-core/issues/15)) ([4fd0ce8](https://github.com/dnviti-personal-org/dentalcraft-core/commit/4fd0ce8c3684cd8209e9b25fae769ac569243325))

## [0.3.0-alpha.3](https://github.com/dnviti-personal-org/dentalcraft-core/compare/v0.3.0-alpha.2...v0.3.0-alpha.3) (2026-11-17)


### Features

* add the alpha 3D scanner connector, turntable and BLE link ([#12](https://github.com/dnviti-personal-org/dentalcraft-core/issues/12)) ([143a88b](https://github.com/dnviti-personal-org/dentalcraft-core/commit/143a88bea9477bdf067748414c7395d1781ea3ff))

## [0.3.0-alpha.2](https://github.com/dnviti-personal-org/dentalcraft-core/compare/v0.3.0-alpha.1...v0.3.0-alpha.2) (2026-11-08)


### Features

* **core:** module scene entries with arch, tooth and stage ([#9](https://github.com/dnviti-personal-org/dentalcraft-core/issues/9)) ([abe9775](https://github.com/dnviti-personal-org/dentalcraft-core/commit/abe977558bfcd0cd075c4b96a8492d698060e0d8))

## [0.3.0-alpha.1](https://github.com/dnviti-personal-org/dentalcraft-core/compare/v0.2.1...v0.3.0-alpha.1) (2026-11-01)


### Features

* command specs, port scaffolding and golden metrics ([54cfeba](https://github.com/dnviti-personal-org/dentalcraft-core/commit/54cfebac2aa873dbc87a09d427221e3389f895b3))
* **dc-case:** one shared scan import pipeline ([38aae84](https://github.com/dnviti-personal-org/dentalcraft-core/commit/38aae84566e774242fdb97aa56b23d55cee51f2a))
* **dc-case:** RoleSource::DetectedUnsure for detections the plugin marks to be checked ([1de622b](https://github.com/dnviti-personal-org/dentalcraft-core/commit/1de622b45d6f501ed7f2f9f734e3b54bff5f7b18))
* **dc-case:** single file-name role detection (arch_hint, role_from_name) ([738771b](https://github.com/dnviti-personal-org/dentalcraft-core/commit/738771be4afc3cfe2ba0a166a44ecb9c29936fd0))
* **dc-geom:** reproducible Jacobi PCG, Dirichlet solves and cotan Laplacian (WP17) ([75447e2](https://github.com/dnviti-personal-org/dentalcraft-core/commit/75447e2b51d275817a988aaf162f2a688ad24b86))
* **dc-geom:** sparse narrow-band SDF with offset, closing, CSG and meshing (WP18) ([3f48ab2](https://github.com/dnviti-personal-org/dentalcraft-core/commit/3f48ab27778fc9e84cbae61fdf7fc3d894dd8375))
* **dc-mesh:** import cleanup primitives and read_scan ([7bf9ef6](https://github.com/dnviti-personal-org/dentalcraft-core/commit/7bf9ef6dd5c07a9d657ab08af0b75e31d1da61ae))
* **dc-session:** case.import on the shared pipeline, case.set-role, import_scan batches ([61aa06f](https://github.com/dnviti-personal-org/dentalcraft-core/commit/61aa06f1f3319cd002f3b231a79850929b92f784))
* **geom:** binned-SAH BVH with closest point, ray and overlap queries ([4eaef54](https://github.com/dnviti-personal-org/dentalcraft-core/commit/4eaef5404c898e07c57065a985d20fc8e94c85a7))
* **geom:** k-nearest-neighbour index on kiddo with batched queries ([5b92ad5](https://github.com/dnviti-personal-org/dentalcraft-core/commit/5b92ad5869afd1bf8cab1c529c1ba060b1116d79))
* **geom:** manifold-safe QEM decimation (WP15) ([194acb6](https://github.com/dnviti-personal-org/dentalcraft-core/commit/194acb62182fa9c07d9fa4db5679f25819e109e6))
* **geom:** weighted Kabsch/Umeyama and trimmed ICP (WP14) ([bbc71d1](https://github.com/dnviti-personal-org/dentalcraft-core/commit/bbc71d17724591c093718673f7ff83fd6edcf9b1))
* **mcp:** JSON-RPC MCP server over stdio behind a Host trait ([cc89a75](https://github.com/dnviti-personal-org/dentalcraft-core/commit/cc89a75036f40aaf5c64e774a06c01286b7b9f15))
* **module-api:** leaf input, workflow step flag and leaf_done ([5f32c42](https://github.com/dnviti-personal-org/dentalcraft-core/commit/5f32c4219b4d76062c76172cd576f9314cc335a6))
* **raster:** deterministic CPU rasteriser with dental views and PNG output ([b7a3617](https://github.com/dnviti-personal-org/dentalcraft-core/commit/b7a3617f52637b08bb8bbe3f7706398bff39d846))
* **session:** headless session with command registry, built-ins and inspect ([4eb64ab](https://github.com/dnviti-personal-org/dentalcraft-core/commit/4eb64abbd1601afc777e1cd54a1004b907ec9bbf))


### Bug Fixes

* **case:** lower arch lies towards -Z in the world frame ([fdc7fc8](https://github.com/dnviti-personal-org/dentalcraft-core/commit/fdc7fc89cb8045ed13212f5ee0d41cfe0e6dc0a9))
* **case:** say where to set the role of a scan the import refused ([5599a2d](https://github.com/dnviti-personal-org/dentalcraft-core/commit/5599a2d8134ef16fdc0102acd3e7c35cbd8dced3))
* **dc-case:** scan import review findings ([03c7aa1](https://github.com/dnviti-personal-org/dentalcraft-core/commit/03c7aa1083c3756959889beb73dd16ac5a378b6a))
* **dc-geom:** close the SDF like OpenVDB and keep inside zeros inside ([b5359dc](https://github.com/dnviti-personal-org/dentalcraft-core/commit/b5359dc98c213dad373c1099e1d6a2edc4f58b29))


### Performance Improvements

* **dc-geom:** bound the QEM valence and prune the tolerance check ([5fbb4bb](https://github.com/dnviti-personal-org/dentalcraft-core/commit/5fbb4bb31a0f8afba5d1c5b96b2d722696565fae))
* **dc-geom:** re-distance only the blocks around buried walls ([291b177](https://github.com/dnviti-personal-org/dentalcraft-core/commit/291b177096ef38be257c373530317786b75aca71))


### Miscellaneous Chores

* **release:** release alphas through release-please ([fc81a5e](https://github.com/dnviti-personal-org/dentalcraft-core/commit/fc81a5e79bb0c87d70e253197904fdb9545053e0))

## [0.2.1](https://github.com/dnviti-personal-org/dentalcraft-core/compare/v0.2.0...v0.2.1) (2026-10-31)


### Bug Fixes

* **workspace:** drop version requirements on internal path dependencies ([#5](https://github.com/dnviti-personal-org/dentalcraft-core/issues/5)) ([73ebf70](https://github.com/dnviti-personal-org/dentalcraft-core/commit/73ebf70638cbb7875cf9fbe7b731883057eddc83))

## [0.2.0](https://github.com/dnviti-personal-org/dentalcraft-core/compare/v0.1.0...v0.2.0) (2026-10-31)


### ⚠ BREAKING CHANGES

* **module-api:** per-module persistence traits replaced by ModuleCodec.

### Features

* **module-api:** select hook, unified ModuleCodec, blob gc ([#3](https://github.com/dnviti-personal-org/dentalcraft-core/issues/3)) ([80020d0](https://github.com/dnviti-personal-org/dentalcraft-core/commit/80020d079bf24f41c762cc7575a26fe2bef493c9))

## 0.1.0 (2026-10-31)


### Features

* initial core implementation (engine crates, mesh io, dcprj container) ([#1](https://github.com/dnviti-personal-org/dentalcraft-core/issues/1)) ([41164e3](https://github.com/dnviti-personal-org/dentalcraft-core/commit/41164e3b75863fad5a8589c2454c4c7fc9e0909d))
