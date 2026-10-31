//! Headless engine session (port plan B1.1).
//!
//! Will hold the open project, the current [`dc_case::CaseState`], the undo
//! stack, the step history, the job system and the registry of every
//! command (built-in `project.*`, `case.import`, `edit.undo` / `edit.redo`,
//! `steps.*`, `view.set`, plus the modules' `CommandSpec`s), and expose
//! `execute`, `batch` and `inspect` for the CLI and the MCP server.
//!
//! Skeleton only for now.
