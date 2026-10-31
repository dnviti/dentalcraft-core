//! Step history of a processing pipeline: go back to a step and retry it.
//!
//! Port of the semantics of the old `ortho_aligner/steps.py`:
//!
//! * every step records a revision when it completes ([`StepHistory::checkpoint`],
//!   automatic, or [`StepHistory::mark_manual`], by hand); the revision changes
//!   only when the result changes, so a step recomputed with the same result
//!   does not make the following steps stale;
//! * a step is *stale* when a step it depends on (directly or not) has a newer
//!   revision;
//! * [`StepHistory::fail`] drops the records of the failed step and of the
//!   following ones and records the failure;
//! * [`StepHistory::rewind`] removes the records from a step on and sets them
//!   aside together with the case state, so that
//!   [`StepHistory::undo_rewind`] can bring both back exactly (undoing again
//!   returns to the rewound state). Records of earlier steps changed after
//!   the rewind are restored too.
//!
//! The module owning the pipeline removes or restores its own data in the
//! case state; this type only tracks the records.

use std::collections::{BTreeMap, HashMap};
use std::sync::Arc;
use std::time::{SystemTime, UNIX_EPOCH};

use serde::{Deserialize, Serialize};
use serde_json::Value;

use dc_case::{BlobRef, CaseState};

/// Errors of the step history.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum StepError {
    /// The step id is not part of the graph.
    #[error("unknown step: {0}")]
    Unknown(String),
    /// Two steps share an id.
    #[error("duplicate step: {0}")]
    Duplicate(&'static str),
    /// A step depends on a step that is unknown or does not come before it.
    #[error("step {step} depends on {dependency}, which does not precede it")]
    BadDependency {
        /// The step.
        step: &'static str,
        /// Its offending dependency.
        dependency: &'static str,
    },
    /// There is no rewind to undo.
    #[error("no rewind to undo")]
    NothingToUndo,
}

/// Definition of one pipeline step.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StepDef {
    /// Stable identifier (also the i18n key prefix of its label).
    pub id: &'static str,
    /// Steps whose results this step is computed from (must precede it).
    pub depends: &'static [&'static str],
}

/// Ordered pipeline steps with their dependencies.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct StepGraph {
    steps: Vec<StepDef>,
    index: HashMap<&'static str, usize>,
}

impl StepGraph {
    /// Validates ids (unique) and dependencies (known and earlier).
    pub fn new(steps: Vec<StepDef>) -> Result<Self, StepError> {
        let mut index = HashMap::new();
        for (i, s) in steps.iter().enumerate() {
            for &d in s.depends {
                if !index.contains_key(d) {
                    return Err(StepError::BadDependency {
                        step: s.id,
                        dependency: d,
                    });
                }
            }
            if index.insert(s.id, i).is_some() {
                return Err(StepError::Duplicate(s.id));
            }
        }
        Ok(Self { steps, index })
    }

    /// Step ids in pipeline order.
    pub fn ids(&self) -> impl Iterator<Item = &'static str> + '_ {
        self.steps.iter().map(|s| s.id)
    }

    /// Number of steps.
    pub fn len(&self) -> usize {
        self.steps.len()
    }

    /// `true` when the graph has no steps.
    pub fn is_empty(&self) -> bool {
        self.steps.is_empty()
    }

    /// Position of step `id` in the pipeline.
    pub fn index_of(&self, id: &str) -> Result<usize, StepError> {
        self.index
            .get(id)
            .copied()
            .ok_or_else(|| StepError::Unknown(id.to_owned()))
    }

    /// Definition of step `id`.
    pub fn get(&self, id: &str) -> Result<&StepDef, StepError> {
        Ok(&self.steps[self.index_of(id)?])
    }

    /// Ids from `start` to `stop` included (`None` = first / last step).
    pub fn span(
        &self,
        start: Option<&str>,
        stop: Option<&str>,
    ) -> Result<Vec<&'static str>, StepError> {
        let i = start.map_or(Ok(0), |s| self.index_of(s))?;
        let j = stop.map_or(Ok(self.steps.len().saturating_sub(1)), |s| self.index_of(s))?;
        Ok(self
            .steps
            .get(i..=j)
            .unwrap_or_default()
            .iter()
            .map(|s| s.id)
            .collect())
    }

    fn from(&self, k: usize) -> impl Iterator<Item = &'static str> + '_ {
        self.steps[k..].iter().map(|s| s.id)
    }
}

fn now() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_or(0, |d| d.as_secs())
}

/// Record of a completed (or failed) step. Persisted in `history/steps.json`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct StepRecord {
    /// Step id.
    pub step: String,
    /// Revision: increases whenever the result of the step changes.
    pub rev: u64,
    /// Done or corrected by hand.
    pub by_hand: bool,
    /// Hash of the inputs the step was computed from, if tracked.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub inputs_hash: Option<BlobRef>,
    /// Hash of the results: same hash = same result.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub outputs: Option<BlobRef>,
    /// Failure message (the step failed and has no results).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub failed: Option<String>,
    /// Partial summary of the automatic analysis after this step.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub summary: Option<Value>,
    /// Unix time (seconds) of the record.
    pub at: u64,
    /// Module-specific fields kept across re-records (e.g. import source).
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub extra: BTreeMap<String, Value>,
}

/// What a completed step produced, passed to [`StepHistory::checkpoint`]
/// and [`StepHistory::mark_manual`].
#[derive(Debug, Clone, Default, PartialEq)]
pub struct StepOutcome {
    /// Hash of the results (`None` = unknown: always counts as changed).
    pub outputs: Option<BlobRef>,
    /// Hash of the inputs, if tracked.
    pub inputs_hash: Option<BlobRef>,
    /// Partial summary (`None` keeps the previous one).
    pub summary: Option<Value>,
    /// Extra fields merged into the record's previous extras.
    pub extra: BTreeMap<String, Value>,
}

impl StepOutcome {
    /// Outcome with only the result hash.
    pub fn with_outputs(outputs: BlobRef) -> Self {
        Self {
            outputs: Some(outputs),
            ..Self::default()
        }
    }
}

/// State of a step for the user interface.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum StepState {
    /// Done automatically and up to date.
    Done,
    /// Not done yet.
    Todo,
    /// The last automatic run failed.
    Failed,
    /// An earlier step it depends on changed after it was computed.
    Stale,
    /// Done or corrected by hand and up to date.
    Manual,
}

/// Status of one step, as returned by [`StepHistory::status`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct StepStatus {
    /// Step id.
    pub id: &'static str,
    /// State.
    pub state: StepState,
    /// Failure message when `state` is [`StepState::Failed`].
    pub failure: Option<String>,
    /// The changed step that made this one stale.
    pub stale_because: Option<&'static str>,
    /// The current result comes from the automatic processing.
    pub auto: bool,
}

/// Information about the rewind that can be undone.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct UndoInfo {
    /// Step the case was rewound to (state right before it).
    pub step: String,
    /// Unix time of the rewind.
    pub at: u64,
}

/// Result of [`StepHistory::rewind`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RewindOutcome {
    /// Step rewound to.
    pub step: String,
    /// Steps whose records were removed (set aside).
    pub removed: Vec<String>,
    /// A rewind can be undone afterwards.
    pub undo_available: bool,
}

/// What a rewind set aside. Not persisted: lives with the open case only.
#[derive(Debug, Clone)]
struct RewindUndo {
    step: String,
    at: u64,
    /// Records to put back; `None` = the step had no record.
    steps: BTreeMap<String, Option<StepRecord>>,
    rev: u64,
    state: Arc<CaseState>,
}

/// History of the steps of a pipeline (see the module documentation).
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct StepHistory {
    rev: u64,
    steps: BTreeMap<String, StepRecord>,
    #[serde(skip)]
    undo: Option<RewindUndo>,
}

impl StepHistory {
    /// Empty history.
    pub fn new() -> Self {
        Self::default()
    }

    /// Latest revision number handed out.
    pub fn rev(&self) -> u64 {
        self.rev
    }

    /// Record of step `id`, if any.
    pub fn record(&self, id: &str) -> Option<&StepRecord> {
        self.steps.get(id)
    }

    /// All records keyed by step id.
    pub fn records(&self) -> &BTreeMap<String, StepRecord> {
        &self.steps
    }

    /// Records step `id` as completed automatically. When its result
    /// changed, the following steps become stale.
    pub fn checkpoint(
        &mut self,
        g: &StepGraph,
        id: &str,
        outcome: StepOutcome,
    ) -> Result<u64, StepError> {
        self.put(g, id, false, outcome)
    }

    /// Records step `id` as done or corrected by hand. Call it for the
    /// earliest changed step first.
    pub fn mark_manual(
        &mut self,
        g: &StepGraph,
        id: &str,
        outcome: StepOutcome,
    ) -> Result<u64, StepError> {
        self.put(g, id, true, outcome)
    }

    fn put(
        &mut self,
        g: &StepGraph,
        id: &str,
        by_hand: bool,
        o: StepOutcome,
    ) -> Result<u64, StepError> {
        g.index_of(id)?;
        self.keep_for_undo(g, [id]);
        let old = self.steps.get(id);
        let same = old
            .is_some_and(|r| r.failed.is_none() && o.outputs.is_some() && r.outputs == o.outputs);
        let rev = match old {
            Some(r) if same => r.rev.max(self.upstream_rev(g, id)),
            _ => {
                self.rev += 1;
                self.rev
            }
        };
        let (summary, mut extra) = old.map_or((None, BTreeMap::new()), |r| {
            (r.summary.clone(), r.extra.clone())
        });
        extra.extend(o.extra);
        let rec = StepRecord {
            step: id.to_owned(),
            rev,
            by_hand,
            inputs_hash: o.inputs_hash,
            outputs: o.outputs,
            failed: None,
            summary: o.summary.or(summary),
            at: now(),
            extra,
        };
        self.steps.insert(id.to_owned(), rec);
        Ok(rev)
    }

    /// Records that the automatic run of step `id` failed: its record and
    /// those of the following steps are removed (the module removes their
    /// partial results), the earlier ones stay.
    pub fn fail(
        &mut self,
        g: &StepGraph,
        id: &str,
        message: impl Into<String>,
    ) -> Result<(), StepError> {
        let k = g.index_of(id)?;
        self.keep_for_undo(g, g.from(k));
        for sid in g.from(k) {
            self.steps.remove(sid);
        }
        self.rev += 1;
        self.steps.insert(
            id.to_owned(),
            StepRecord {
                step: id.to_owned(),
                rev: self.rev,
                by_hand: false,
                inputs_hash: None,
                outputs: None,
                failed: Some(message.into()),
                summary: None,
                at: now(),
                extra: BTreeMap::new(),
            },
        );
        Ok(())
    }

    /// Merges `fields` into every stored summary object.
    pub fn amend_summaries(&mut self, g: &StepGraph, fields: &serde_json::Map<String, Value>) {
        let ids: Vec<String> = self
            .steps
            .iter()
            .filter(|(_, r)| matches!(r.summary, Some(Value::Object(_))))
            .map(|(k, _)| k.clone())
            .collect();
        self.keep_for_undo(g, ids.iter().map(String::as_str));
        for id in ids {
            if let Some(Value::Object(m)) = self.steps.get_mut(&id).and_then(|r| r.summary.as_mut())
            {
                m.extend(fields.clone());
            }
        }
    }

    /// Latest summary stored by a step before `id`.
    pub fn summary_before(&self, g: &StepGraph, id: &str) -> Result<Option<&Value>, StepError> {
        let k = g.index_of(id)?;
        Ok(g.steps[..k]
            .iter()
            .rev()
            .filter_map(|s| self.steps.get(s.id)?.summary.as_ref())
            .find(|v| v.is_object()))
    }

    /// Goes back to the state right before step `id`: the records of that
    /// step and of the following ones are removed and set aside with
    /// `current`, the state of the case before the rewind, so that
    /// [`StepHistory::undo_rewind`] can bring them back. A previous rewind
    /// can no longer be undone. Nothing happens when no step from `id` on
    /// has a record.
    pub fn rewind(
        &mut self,
        g: &StepGraph,
        id: &str,
        current: Arc<CaseState>,
    ) -> Result<RewindOutcome, StepError> {
        let k = g.index_of(id)?;
        let removed: Vec<String> = g
            .from(k)
            .filter(|s| self.steps.contains_key(*s))
            .map(str::to_owned)
            .collect();
        if removed.is_empty() {
            return Ok(RewindOutcome {
                step: id.to_owned(),
                removed,
                undo_available: self.undo.is_some(),
            });
        }
        self.undo = Some(self.set_aside(g, k, current));
        for sid in g.from(k) {
            self.steps.remove(sid);
        }
        Ok(RewindOutcome {
            step: id.to_owned(),
            removed,
            undo_available: true,
        })
    }

    fn set_aside(&self, g: &StepGraph, k: usize, state: Arc<CaseState>) -> RewindUndo {
        RewindUndo {
            step: g.steps[k].id.to_owned(),
            at: now(),
            steps: g
                .from(k)
                .filter_map(|s| Some((s.to_owned(), Some(self.steps.get(s)?.clone()))))
                .collect(),
            rev: self.rev,
            state,
        }
    }

    /// The rewind that can be undone, if any.
    pub fn undo_info(&self) -> Option<UndoInfo> {
        self.undo.as_ref().map(|u| UndoInfo {
            step: u.step.clone(),
            at: u.at,
        })
    }

    /// `true` when a rewind can be undone.
    pub fn can_undo_rewind(&self) -> bool {
        self.undo.is_some()
    }

    /// Undoes the last rewind: the records it set aside come back and the
    /// returned case state is the one passed to [`StepHistory::rewind`].
    /// `current` (the state now) is set aside in its place, so undoing
    /// again returns to it.
    pub fn undo_rewind(
        &mut self,
        g: &StepGraph,
        current: Arc<CaseState>,
    ) -> Result<Arc<CaseState>, StepError> {
        let u = self.undo.take().ok_or(StepError::NothingToUndo)?;
        let k = g.index_of(&u.step)?;
        let mut new = self.set_aside(g, k, current);
        let mut cur: BTreeMap<String, StepRecord> = self
            .steps
            .iter()
            .filter(|(sid, _)| g.index_of(sid).is_ok_and(|i| i < k))
            .map(|(s, r)| (s.clone(), r.clone()))
            .collect();
        for (sid, rec) in u.steps {
            if g.index_of(&sid).is_ok_and(|i| i < k) {
                new.steps.insert(sid.clone(), cur.get(&sid).cloned());
            }
            match rec {
                Some(r) => cur.insert(sid, r),
                None => cur.remove(&sid),
            };
        }
        self.steps = cur;
        self.rev = self.rev.max(u.rev);
        self.undo = Some(new);
        Ok(u.state)
    }

    /// Before a record of a step earlier than the pending rewind changes, its
    /// current version is kept in the undo record so undo stays exact.
    fn keep_for_undo<'a>(&mut self, g: &StepGraph, ids: impl IntoIterator<Item = &'a str>) {
        let Some(u) = self.undo.as_mut() else { return };
        let Ok(k) = g.index_of(&u.step) else { return };
        for sid in ids {
            if g.index_of(sid).is_ok_and(|i| i < k) && !u.steps.contains_key(sid) {
                u.steps.insert(sid.to_owned(), self.steps.get(sid).cloned());
            }
        }
    }

    /// Newest revision among the steps `id` is computed from, transitively.
    fn upstream_rev(&self, g: &StepGraph, id: &str) -> u64 {
        g.get(id).map_or(0, |def| {
            def.depends
                .iter()
                .map(|d| {
                    let own = self.steps.get(*d).map_or(0, |r| r.rev);
                    own.max(self.upstream_rev(g, d))
                })
                .max()
                .unwrap_or(0)
        })
    }

    /// State of every step, in pipeline order. `has(id)` tells whether the
    /// case currently holds the results of step `id`.
    pub fn status(&self, g: &StepGraph, has: impl Fn(&str) -> bool) -> Vec<StepStatus> {
        let mut latest: HashMap<&'static str, (u64, &'static str)> = HashMap::new();
        let mut out = Vec::with_capacity(g.len());
        for def in &g.steps {
            let id = def.id;
            let rec = self.steps.get(id);
            let rev = rec.map_or(0, |r| r.rev);
            let present = has(id);
            let upstream = def
                .depends
                .iter()
                .filter_map(|d| latest.get(d).copied())
                .max()
                .unwrap_or((0, ""));
            let mut status = StepStatus {
                id,
                state: StepState::Done,
                failure: None,
                stale_because: None,
                auto: present && rec.is_some_and(|r| !r.by_hand && r.failed.is_none()),
            };
            if let Some(msg) = rec.and_then(|r| r.failed.clone()) {
                status.state = StepState::Failed;
                status.failure = Some(msg);
            } else if !present {
                status.state = StepState::Todo;
            } else if upstream.0 > rev {
                status.state = StepState::Stale;
                status.stale_because = Some(upstream.1);
            } else if rec.is_some_and(|r| r.by_hand) {
                status.state = StepState::Manual;
            }
            latest.insert(
                id,
                if present {
                    upstream.max((rev, id))
                } else {
                    upstream
                },
            );
            out.push(status);
        }
        out
    }

    /// [`StepHistory::status`] where a step has results when it has a
    /// successful record.
    pub fn status_from_records(&self, g: &StepGraph) -> Vec<StepStatus> {
        self.status(g, |id| {
            self.steps.get(id).is_some_and(|r| r.failed.is_none())
        })
    }

    /// First step before `id` without results: it must be completed before
    /// `id` can be retried.
    pub fn missing_prerequisite(
        &self,
        g: &StepGraph,
        id: &str,
        has: impl Fn(&str) -> bool,
    ) -> Result<Option<&'static str>, StepError> {
        let k = g.index_of(id)?;
        Ok(g.steps[..k].iter().map(|s| s.id).find(|s| !has(s)))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const STEPS: &[StepDef] = &[
        StepDef {
            id: "arch",
            depends: &[],
        },
        StepDef {
            id: "seeds",
            depends: &["arch"],
        },
        StepDef {
            id: "segment",
            depends: &["seeds"],
        },
        StepDef {
            id: "teeth",
            depends: &["segment"],
        },
        StepDef {
            id: "diagnosis",
            depends: &["teeth"],
        },
        StepDef {
            id: "setup",
            depends: &["teeth"],
        },
        StepDef {
            id: "staging",
            depends: &["setup"],
        },
        StepDef {
            id: "review",
            depends: &["diagnosis", "staging"],
        },
    ];

    fn graph() -> StepGraph {
        StepGraph::new(STEPS.to_vec()).unwrap()
    }

    fn h(n: u8) -> StepOutcome {
        StepOutcome::with_outputs(BlobRef::of_bytes(&[n]))
    }

    fn states(hist: &StepHistory, g: &StepGraph) -> Vec<StepState> {
        hist.status_from_records(g)
            .into_iter()
            .map(|s| s.state)
            .collect()
    }

    fn run_all(hist: &mut StepHistory, g: &StepGraph) {
        for (i, id) in g.ids().enumerate() {
            hist.checkpoint(g, id, h(i as u8)).unwrap();
        }
    }

    fn state_with(code: &str) -> Arc<CaseState> {
        Arc::new(CaseState::new(dc_case::Case::new(code)))
    }

    use StepState::*;

    #[test]
    fn graph_validation() {
        let g = graph();
        assert_eq!(g.len(), 8);
        assert_eq!(
            g.span(Some("teeth"), Some("setup")).unwrap(),
            vec!["teeth", "diagnosis", "setup"]
        );
        assert_eq!(g.span(None, None).unwrap().len(), 8);
        assert!(g.index_of("nope").is_err());
        let bad = StepGraph::new(vec![
            StepDef {
                id: "a",
                depends: &["b"],
            },
            StepDef {
                id: "b",
                depends: &[],
            },
        ]);
        assert!(matches!(bad, Err(StepError::BadDependency { .. })));
        let dup = StepGraph::new(vec![
            StepDef {
                id: "a",
                depends: &[],
            },
            StepDef {
                id: "a",
                depends: &[],
            },
        ]);
        assert_eq!(dup, Err(StepError::Duplicate("a")));
    }

    #[test]
    fn checkpoints_and_staleness() {
        let g = graph();
        let mut hist = StepHistory::new();
        assert!(states(&hist, &g).iter().all(|s| *s == Todo));
        run_all(&mut hist, &g);
        assert!(states(&hist, &g).iter().all(|s| *s == Done));
        // a manual change of the seeds makes everything after them stale
        hist.mark_manual(&g, "seeds", h(100)).unwrap();
        let st = hist.status_from_records(&g);
        assert_eq!(st[1].state, Manual);
        assert!(!st[1].auto);
        assert!(st[2..].iter().all(|s| s.state == Stale));
        assert_eq!(st[2].stale_because, Some("seeds"));
        assert_eq!(st[7].stale_because, Some("seeds"));
    }

    #[test]
    fn same_result_after_upstream_change_is_up_to_date() {
        let g = graph();
        let mut hist = StepHistory::new();
        run_all(&mut hist, &g);
        hist.checkpoint(&g, "setup", h(50)).unwrap();
        assert_eq!(hist.status_from_records(&g)[6].state, Stale);
        // staging recomputed with the same result as before: up to date again
        hist.checkpoint(&g, "staging", h(6)).unwrap();
        assert_eq!(hist.status_from_records(&g)[6].state, Done);
        assert_eq!(
            hist.record("staging").unwrap().rev,
            hist.record("setup").unwrap().rev
        );
        // diagnosis does not depend on setup
        assert_eq!(hist.status_from_records(&g)[4].state, Done);
    }

    #[test]
    fn failure_drops_following_steps() {
        let g = graph();
        let mut hist = StepHistory::new();
        run_all(&mut hist, &g);
        hist.fail(&g, "teeth", "no teeth found").unwrap();
        let st = hist.status_from_records(&g);
        assert_eq!(st[3].state, Failed);
        assert_eq!(st[3].failure.as_deref(), Some("no teeth found"));
        assert!(st[..3].iter().all(|s| s.state == Done));
        assert!(st[4..].iter().all(|s| s.state == Todo));
        assert_eq!(
            hist.missing_prerequisite(&g, "diagnosis", |id| hist
                .status_from_records(&g)
                .iter()
                .any(|s| s.id == id && s.state == Done))
                .unwrap(),
            Some("teeth")
        );
    }

    #[test]
    fn rewind_and_undo_rewind_toggle() {
        let g = graph();
        let mut hist = StepHistory::new();
        run_all(&mut hist, &g);
        let before = hist.records().clone();
        let full = state_with("full");
        let out = hist.rewind(&g, "teeth", full.clone()).unwrap();
        assert_eq!(
            out.removed,
            vec!["teeth", "diagnosis", "setup", "staging", "review"]
        );
        assert!(hist.can_undo_rewind());
        assert_eq!(hist.undo_info().unwrap().step, "teeth");
        assert_eq!(states(&hist, &g)[3..], [Todo; 5]);
        // redo the step differently, then undo the rewind
        hist.checkpoint(&g, "teeth", h(200)).unwrap();
        let redone = state_with("redone");
        let restored = hist.undo_rewind(&g, redone.clone()).unwrap();
        assert!(Arc::ptr_eq(&restored, &full));
        assert_eq!(hist.records(), &before);
        // undo again: back to the redone state
        let again = hist.undo_rewind(&g, restored).unwrap();
        assert!(Arc::ptr_eq(&again, &redone));
        assert_eq!(hist.record("teeth").unwrap().outputs, h(200).outputs);
        assert!(hist.record("diagnosis").is_none());
    }

    #[test]
    fn undo_rewind_restores_earlier_records_changed_after_rewind() {
        let g = graph();
        let mut hist = StepHistory::new();
        run_all(&mut hist, &g);
        let before = hist.records().clone();
        hist.rewind(&g, "setup", state_with("x")).unwrap();
        hist.mark_manual(&g, "seeds", h(77)).unwrap();
        hist.undo_rewind(&g, state_with("y")).unwrap();
        assert_eq!(hist.records(), &before);
        hist.undo_rewind(&g, state_with("z")).unwrap();
        assert_eq!(hist.record("seeds").unwrap().outputs, h(77).outputs);
        assert!(hist.record("seeds").unwrap().by_hand);
        assert!(hist.record("setup").is_none());
    }

    #[test]
    fn rewind_with_nothing_to_remove_is_a_no_op() {
        let g = graph();
        let mut hist = StepHistory::new();
        hist.checkpoint(&g, "arch", h(0)).unwrap();
        let out = hist.rewind(&g, "seeds", state_with("a")).unwrap();
        assert!(out.removed.is_empty() && !out.undo_available);
        assert!(matches!(
            hist.undo_rewind(&g, state_with("b")),
            Err(StepError::NothingToUndo)
        ));
    }

    #[test]
    fn summaries_and_serde() {
        let g = graph();
        let mut hist = StepHistory::new();
        let mut o = h(1);
        o.summary = Some(serde_json::json!({"arch": "upper"}));
        o.extra.insert("source".into(), "scanner".into());
        hist.checkpoint(&g, "arch", o).unwrap();
        hist.checkpoint(&g, "seeds", h(2)).unwrap();
        let fields = serde_json::json!({"arch": "lower"});
        hist.amend_summaries(&g, fields.as_object().unwrap());
        assert_eq!(
            hist.summary_before(&g, "segment").unwrap().unwrap()["arch"],
            "lower"
        );
        // re-recording keeps summary and extras
        hist.checkpoint(&g, "arch", h(1)).unwrap();
        let rec = hist.record("arch").unwrap();
        assert_eq!(rec.extra["source"], "scanner");
        assert!(rec.summary.is_some());
        let json = serde_json::to_string(&hist).unwrap();
        let back: StepHistory = serde_json::from_str(&json).unwrap();
        assert_eq!(back.records(), hist.records());
        assert_eq!(back.rev(), hist.rev());
        assert!(!back.can_undo_rewind());
    }
}
