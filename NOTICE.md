# Notice

Required Notice: Copyright (c) Daniele Viti

## Licence

The source code of dentalcraft-core is source-available under the PolyForm Noncommercial License 1.0.0. The full text is in `LICENSE.md`.

You may use, change and share it for any noncommercial purpose. Commercial use is not allowed. This is not an open source licence in the OSI sense.

## Commercial licensing

For commercial use, contact the author, Daniele Viti: hello@dentalcraft.app.

## Not a medical device

DentalCraft is not a certified medical device. It has no regulatory clearance. Do not use it on patients or for clinical decisions without independent professional validation. It is provided as is, without warranty.

## Third-party components

This repository does not copy third-party source code. Its Rust dependencies are fetched from crates.io and keep their own licences. The dependency tree of this repository is under MIT, Apache-2.0, or both, plus a few other permissive licences (Unicode-3.0, BSD-3-Clause, ISC, Zlib, Unlicense, CC0-1.0).

- A few crates offer a choice of licences, and DentalCraft uses them under the permissive option. For example `self_cell` is `Apache-2.0 OR GPL-2.0-only` and is used under Apache-2.0.
- `epaint_default_fonts` (pulled in by egui) is `(MIT OR Apache-2.0) AND OFL-1.1 AND Ubuntu-font-1.0`: the code is MIT or Apache-2.0, and the bundled fonts are under the SIL Open Font License 1.1 and the Ubuntu Font Licence 1.0.
- No dependency is under a GPL, AGPL or LGPL licence without a permissive alternative.

The complete list, with licence texts, is in `THIRD-PARTY-NOTICES.md` of `dentalcraft-app`, which is generated for each application release. The exact versions are in `Cargo.lock`.

### Vendor SDK and community protocols

- The Revopoint `3DCamera` SDK is proprietary software of Revopoint. DentalCraft does not ship it and does not link it. The backend loads it at run time from a path the user sets, and declares its C API by hand. The user must have the right to use the SDK under Revopoint's terms.
- The Revopoint Dual-axis Turntable protocol (`dc-turntable`) follows community reverse engineering: opensourcemanufacturing/Revopoint-Dual-Turntable, SphaeroX/Revopoint-Dual-Axis-Turntable and aderusha/ha-revopoint-turntable. The Revopoint MINI 2 USB extension-unit notes (`dc-scanner`) follow BR1ZB3AR/All3DScan (`RE_NOTES.md`). Only protocol facts were taken; no code was copied.
- Revopoint is a trademark of its owner. DentalCraft is not affiliated with or endorsed by Revopoint.
