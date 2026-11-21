//! Resin printing for DentalCraft (print plan, `docs/plan/PRINT-PLAN.md`).
//!
//! Headless and GPU-free:
//! - [`profile`]: printer profiles, with the built-in Elegoo Saturn 4 Ultra
//!   16K, and the single plate-pixel ↔ file-pixel mapping
//!   ([`PrinterProfile::map_pixel`]);
//! - [`machine`]: the printer firmware's machine parameters (`M5000`
//!   files), with the built-in DENTAL 45 set of the Saturn;
//! - [`goo`]: the Elegoo GOO V3 slice format (writer, reader, RLE);
//! - [`part`]: shared pipeline types;
//! - [`orient`], [`support`], [`layout`], [`slice`]: automatic orientation,
//!   outer-side supports, plate nesting and slicing;
//! - [`job`]: the whole pipeline, parts in, GOO files out;
//! - [`commands`]: the `print.*` commands for the CLI and MCP.
//!
//! Layer images never exist as full bitmaps: a layer is a list of
//! [`goo::Run`]s in file pixel order.
//!
//! Implemented from Elegoo's published "Goo Format Spec V1.2" and the
//! project's research notes only; no third-party format code is used.

pub mod commands;
pub mod goo;
pub mod job;
pub mod layout;
pub mod machine;
pub mod orient;
pub mod part;
pub mod profile;
pub mod slice;
pub mod support;
#[cfg(test)]
mod testutil;

pub use profile::PrinterProfile;
