//! Undoable commands, the undo stack and the processing step history.
//!
//! Every change to a case is a [`Command`]: a pure function from the current
//! [`CaseState`] to a new one. The [`UndoStack`] keeps the resulting
//! `Arc<CaseState>` versions (sharing unchanged meshes) within a memory
//! budget. [`StepHistory`] tracks the processing pipeline of a module
//! (done / to do / failed / stale / by hand) and supports going back to a
//! step and undoing that rewind, as the old `steps.py` did.

mod steps;
mod undo;

use dc_case::{CaseError, CaseState};
pub use dc_jobs::{CancelToken, Cancelled, JobCtx, Progress};
pub use steps::{
    RewindOutcome, StepDef, StepError, StepGraph, StepHistory, StepOutcome, StepRecord, StepState,
    StepStatus, UndoInfo,
};
pub use undo::{UndoStack, Version};

/// Errors returned by commands.
#[derive(Debug, thiserror::Error)]
pub enum CommandError {
    /// The command was cancelled through its [`JobCtx`].
    #[error("cancelled")]
    Cancelled,
    /// The command cannot run on this state (precondition not met).
    #[error("invalid: {0}")]
    Invalid(String),
    /// The command ran and failed.
    #[error("failed: {0}")]
    Failed(String),
    /// The command is declared but not implemented yet (menu stub).
    #[error("not available yet: {0}")]
    NotAvailable(String),
    /// Error from the case model.
    #[error(transparent)]
    Case(#[from] CaseError),
    /// Error from the step history.
    #[error(transparent)]
    Step(#[from] StepError),
}

impl From<Cancelled> for CommandError {
    fn from(_: Cancelled) -> Self {
        Self::Cancelled
    }
}

/// An undoable operation on the case.
pub trait Command: Send + Sync {
    /// Stable identifier, e.g. `aligners.segment`.
    fn id(&self) -> &'static str;

    /// i18n key shown in the undo/redo menu entries (defaults to the id).
    fn label_key(&self) -> &str {
        self.id()
    }

    /// Computes the new state from `state`. Must not have side effects
    /// outside the returned state; long commands report progress and check
    /// cancellation through `ctx`.
    fn apply(&self, state: &CaseState, ctx: &JobCtx) -> Result<CaseState, CommandError>;
}
