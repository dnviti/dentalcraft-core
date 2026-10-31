//! Linear undo/redo of immutable case states.

use std::collections::HashSet;
use std::sync::Arc;

use dc_case::CaseState;
use dc_jobs::JobCtx;

use crate::{Command, CommandError};

/// One entry of the undo stack.
#[derive(Debug, Clone)]
pub struct Version {
    /// State after the change.
    pub state: Arc<CaseState>,
    /// i18n key of the change that produced it (empty for the initial state).
    pub label_key: String,
}

/// Linear history of case states with a memory budget.
///
/// Meshes shared between versions are counted once. When the budget is
/// exceeded the oldest undo versions are dropped first, then the farthest
/// redo versions; the current version is never dropped.
#[derive(Debug, Clone)]
pub struct UndoStack {
    versions: Vec<Version>,
    cursor: usize,
    budget_bytes: usize,
}

impl UndoStack {
    /// Default memory budget: 1 GiB.
    pub const DEFAULT_BUDGET: usize = 1 << 30;

    /// Stack holding only `initial`.
    pub fn new(initial: Arc<CaseState>, budget_bytes: usize) -> Self {
        Self {
            versions: vec![Version {
                state: initial,
                label_key: String::new(),
            }],
            cursor: 0,
            budget_bytes,
        }
    }

    /// The current state.
    pub fn current(&self) -> &Arc<CaseState> {
        &self.versions[self.cursor].state
    }

    /// Records `state` as the new current version (the redo branch is discarded).
    pub fn push(&mut self, state: Arc<CaseState>, label_key: impl Into<String>) {
        self.versions.truncate(self.cursor + 1);
        self.versions.push(Version {
            state,
            label_key: label_key.into(),
        });
        self.cursor += 1;
        self.enforce_budget();
    }

    /// Runs `cmd` on the current state and pushes the result.
    pub fn apply(
        &mut self,
        cmd: &dyn Command,
        ctx: &JobCtx,
    ) -> Result<&Arc<CaseState>, CommandError> {
        let next = cmd.apply(self.current(), ctx)?;
        self.push(Arc::new(next), cmd.label_key());
        Ok(self.current())
    }

    /// `true` when there is a version to go back to.
    pub fn can_undo(&self) -> bool {
        self.cursor > 0
    }

    /// `true` when there is an undone version to go forward to.
    pub fn can_redo(&self) -> bool {
        self.cursor + 1 < self.versions.len()
    }

    /// Label of the change [`UndoStack::undo`] would revert.
    pub fn undo_label(&self) -> Option<&str> {
        self.can_undo()
            .then(|| self.versions[self.cursor].label_key.as_str())
    }

    /// Label of the change [`UndoStack::redo`] would re-apply.
    pub fn redo_label(&self) -> Option<&str> {
        self.can_redo()
            .then(|| self.versions[self.cursor + 1].label_key.as_str())
    }

    /// Goes back one version; returns the new current state.
    pub fn undo(&mut self) -> Option<Arc<CaseState>> {
        self.can_undo().then(|| {
            self.cursor -= 1;
            self.current().clone()
        })
    }

    /// Goes forward one version; returns the new current state.
    pub fn redo(&mut self) -> Option<Arc<CaseState>> {
        self.can_redo().then(|| {
            self.cursor += 1;
            self.current().clone()
        })
    }

    /// Drops all versions and starts again from `state`.
    pub fn reset(&mut self, state: Arc<CaseState>) {
        *self = Self::new(state, self.budget_bytes);
    }

    /// Number of stored versions (including the current one).
    pub fn len(&self) -> usize {
        self.versions.len()
    }

    /// Always `false`: the stack holds at least the current version.
    pub fn is_empty(&self) -> bool {
        false
    }

    /// Number of versions [`UndoStack::undo`] can go back.
    pub fn undo_depth(&self) -> usize {
        self.cursor
    }

    /// Number of versions [`UndoStack::redo`] can go forward.
    pub fn redo_depth(&self) -> usize {
        self.versions.len() - self.cursor - 1
    }

    /// Memory budget in bytes.
    pub fn budget_bytes(&self) -> usize {
        self.budget_bytes
    }

    /// Changes the budget and drops versions if needed.
    pub fn set_budget_bytes(&mut self, budget: usize) {
        self.budget_bytes = budget;
        self.enforce_budget();
    }

    /// Approximate memory held by all versions (shared meshes counted once).
    pub fn memory_bytes(&self) -> usize {
        let mut seen = HashSet::new();
        let mut total = 0;
        for v in &self.versions {
            total += v.state.approx_bytes_without_blobs();
            for (r, mesh) in v.state.blobs.iter() {
                if seen.insert(*r) {
                    total += mesh.approx_bytes();
                }
            }
        }
        total
    }

    fn enforce_budget(&mut self) {
        while self.versions.len() > 1 && self.memory_bytes() > self.budget_bytes {
            if self.cursor > 0 {
                self.versions.remove(0);
                self.cursor -= 1;
            } else {
                self.versions.pop();
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use dc_case::Case;
    use dc_mesh::TriMesh;
    use glam::Vec3;

    struct Rename(&'static str);

    impl Command for Rename {
        fn id(&self) -> &'static str {
            "test.rename"
        }
        fn apply(&self, state: &CaseState, ctx: &JobCtx) -> Result<CaseState, CommandError> {
            ctx.check_cancelled()?;
            let mut next = state.clone();
            next.case.patient_code = self.0.to_owned();
            Ok(next)
        }
    }

    fn code(s: &UndoStack) -> &str {
        &s.current().case.patient_code
    }

    #[test]
    fn undo_redo_and_branch_truncation() {
        let mut s = UndoStack::new(
            Arc::new(CaseState::new(Case::new("a"))),
            UndoStack::DEFAULT_BUDGET,
        );
        assert!(!s.can_undo() && !s.can_redo());
        let ctx = JobCtx::detached();
        s.apply(&Rename("b"), &ctx).unwrap();
        s.apply(&Rename("c"), &ctx).unwrap();
        assert_eq!((code(&s), s.len(), s.undo_depth()), ("c", 3, 2));
        assert_eq!(s.undo_label(), Some("test.rename"));
        assert_eq!(s.undo().unwrap().case.patient_code, "b");
        assert_eq!(s.redo_label(), Some("test.rename"));
        s.undo();
        assert_eq!(code(&s), "a");
        assert!(s.undo().is_none());
        s.redo();
        assert_eq!(code(&s), "b");
        s.apply(&Rename("d"), &ctx).unwrap();
        assert!(!s.can_redo());
        assert_eq!(s.len(), 3);
        assert_eq!(code(&s), "d");
    }

    #[test]
    fn cancelled_command_leaves_stack_untouched() {
        let mut s = UndoStack::new(Arc::new(CaseState::default()), UndoStack::DEFAULT_BUDGET);
        let ctx = JobCtx::detached();
        ctx.cancel_token().cancel();
        assert!(matches!(
            s.apply(&Rename("x"), &ctx),
            Err(CommandError::Cancelled)
        ));
        assert_eq!(s.len(), 1);
    }

    #[test]
    fn budget_drops_oldest_and_counts_shared_meshes_once() {
        let big = TriMesh::new(vec![Vec3::ZERO; 10_000], vec![[0, 0, 0]; 10_000]);
        let mut base = CaseState::default();
        base.blobs.insert(big.clone());
        let mesh_bytes = big.approx_bytes();
        let mut s = UndoStack::new(Arc::new(base.clone()), usize::MAX);
        for i in 0..3 {
            let mut next = (**s.current()).clone();
            next.case.patient_code = i.to_string();
            s.push(Arc::new(next), "edit");
        }
        // four versions share one mesh
        assert!(s.memory_bytes() < 2 * mesh_bytes);
        // a new mesh per version: the budget keeps only what fits
        let mut s = UndoStack::new(Arc::new(base), 3 * mesh_bytes);
        for i in 1..=5u32 {
            let mut next = (**s.current()).clone();
            let mut m = big.clone();
            m.positions[0].x = i as f32;
            next.blobs = Default::default();
            next.blobs.insert(m);
            s.push(Arc::new(next), format!("v{i}"));
        }
        assert!(s.memory_bytes() <= s.budget_bytes());
        assert_eq!(s.len(), 2);
        assert_eq!(s.undo_label(), Some("v5"));
        s.set_budget_bytes(0);
        assert_eq!(s.len(), 1);
        assert_eq!(s.current().blobs.len(), 1);
    }
}
