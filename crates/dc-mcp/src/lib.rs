//! MCP server for DentalCraft (port plan B1.1).
//!
//! Hand-rolled JSON-RPC 2.0 over stdio (MCP protocol 2025-06-18, no tokio),
//! behind a `Host` trait implemented by the headless `dc-session` or by a
//! remote host talking to a running app (`dc-app --control <port>`).
//!
//! Skeleton only for now.
