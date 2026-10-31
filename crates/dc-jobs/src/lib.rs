//! Background jobs with progress and cancellation.
//!
//! [`JobSystem::spawn`] runs a closure on a dedicated rayon pool and returns
//! a [`JobHandle`] the UI polls every frame: progress fraction, a message
//! key (an i18n key, translated by the UI), cancellation and the result.
//! The closure receives a [`JobCtx`] to report progress and to check for
//! cancellation; heavy kernels inside it may use rayon's parallel iterators.

use std::panic::{AssertUnwindSafe, catch_unwind};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Condvar, Mutex, MutexGuard, PoisonError};

/// Errors of the job system.
#[derive(Debug, thiserror::Error)]
pub enum JobError {
    /// The thread pool could not be created.
    #[error("cannot start the job thread pool: {0}")]
    Pool(String),
    /// The job panicked; the payload message is kept.
    #[error("job panicked: {0}")]
    Panicked(String),
}

/// Returned by [`JobCtx::check_cancelled`] when the job was cancelled.
#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
#[error("cancelled")]
pub struct Cancelled;

/// Shared cancellation flag.
#[derive(Debug, Clone, Default)]
pub struct CancelToken(Arc<AtomicBool>);

impl CancelToken {
    /// New, not cancelled token.
    pub fn new() -> Self {
        Self::default()
    }

    /// Requests cancellation (idempotent).
    pub fn cancel(&self) {
        self.0.store(true, Ordering::Relaxed);
    }

    /// `true` once cancellation was requested.
    pub fn is_cancelled(&self) -> bool {
        self.0.load(Ordering::Relaxed)
    }
}

/// Progress snapshot of a job.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Progress {
    /// Completion in `[0, 1]`.
    pub fraction: f32,
    /// i18n key of the current sub-step, if any.
    pub message_key: Option<String>,
}

fn lock<T>(m: &Mutex<T>) -> MutexGuard<'_, T> {
    m.lock().unwrap_or_else(PoisonError::into_inner)
}

/// Context handed to a running job (and to commands): progress reporting
/// and cancellation.
#[derive(Debug, Clone, Default)]
pub struct JobCtx {
    progress: Arc<Mutex<Progress>>,
    cancel: CancelToken,
}

impl JobCtx {
    /// Context not attached to any job (synchronous use, tests).
    pub fn detached() -> Self {
        Self::default()
    }

    /// Reports progress; `fraction` is clamped to `[0, 1]`.
    pub fn set_progress(&self, fraction: f32, message_key: Option<&str>) {
        let mut p = lock(&self.progress);
        p.fraction = fraction.clamp(0.0, 1.0);
        p.message_key = message_key.map(str::to_owned);
    }

    /// Reports progress, keeping the current message.
    pub fn set_fraction(&self, fraction: f32) {
        lock(&self.progress).fraction = fraction.clamp(0.0, 1.0);
    }

    /// Current progress.
    pub fn progress(&self) -> Progress {
        lock(&self.progress).clone()
    }

    /// `true` once cancellation was requested.
    pub fn is_cancelled(&self) -> bool {
        self.cancel.is_cancelled()
    }

    /// `Err(Cancelled)` once cancellation was requested (use with `?`).
    pub fn check_cancelled(&self) -> Result<(), Cancelled> {
        if self.is_cancelled() {
            Err(Cancelled)
        } else {
            Ok(())
        }
    }

    /// The cancellation token of this context.
    pub fn cancel_token(&self) -> &CancelToken {
        &self.cancel
    }
}

struct Slot<T> {
    result: Mutex<Option<Result<T, JobError>>>,
    done: Condvar,
    finished: AtomicBool,
}

/// Handle to a spawned job. Dropping it does not cancel the job.
pub struct JobHandle<T> {
    name_key: String,
    ctx: JobCtx,
    slot: Arc<Slot<T>>,
}

impl<T> JobHandle<T> {
    /// i18n key naming the job.
    pub fn name_key(&self) -> &str {
        &self.name_key
    }

    /// Current progress.
    pub fn progress(&self) -> Progress {
        self.ctx.progress()
    }

    /// Requests cancellation; the job stops at its next check.
    pub fn cancel(&self) {
        self.ctx.cancel.cancel();
    }

    /// `true` once cancellation was requested.
    pub fn is_cancelled(&self) -> bool {
        self.ctx.is_cancelled()
    }

    /// A clone of the job's cancellation token.
    pub fn cancel_token(&self) -> CancelToken {
        self.ctx.cancel.clone()
    }

    /// `true` once the job returned (or panicked).
    pub fn is_finished(&self) -> bool {
        self.slot.finished.load(Ordering::Acquire)
    }

    /// Takes the result if the job finished; `None` while running or once taken.
    pub fn try_take(&self) -> Option<Result<T, JobError>> {
        lock(&self.slot.result).take()
    }

    /// Blocks until the job finishes and returns its result.
    ///
    /// # Panics
    /// When the result was already taken with [`JobHandle::try_take`].
    pub fn wait(self) -> Result<T, JobError> {
        let mut guard = lock(&self.slot.result);
        while !self.slot.finished.load(Ordering::Acquire) {
            guard = self
                .slot
                .done
                .wait(guard)
                .unwrap_or_else(PoisonError::into_inner);
        }
        guard.take().expect("job result already taken")
    }
}

/// Pool running background jobs.
pub struct JobSystem {
    pool: rayon::ThreadPool,
}

impl JobSystem {
    /// Pool with `threads` workers (`0` = one per logical CPU).
    pub fn new(threads: usize) -> Result<Self, JobError> {
        let pool = rayon::ThreadPoolBuilder::new()
            .num_threads(threads)
            .thread_name(|i| format!("dc-job-{i}"))
            .build()
            .map_err(|e| JobError::Pool(e.to_string()))?;
        Ok(Self { pool })
    }

    /// Number of worker threads.
    pub fn threads(&self) -> usize {
        self.pool.current_num_threads()
    }

    /// Runs `f` in the background. Panics inside `f` are caught and reported
    /// as [`JobError::Panicked`].
    pub fn spawn<T, F>(&self, name_key: impl Into<String>, f: F) -> JobHandle<T>
    where
        T: Send + 'static,
        F: FnOnce(&JobCtx) -> T + Send + 'static,
    {
        let ctx = JobCtx::default();
        let slot = Arc::new(Slot {
            result: Mutex::new(None),
            done: Condvar::new(),
            finished: AtomicBool::new(false),
        });
        let (job_ctx, job_slot) = (ctx.clone(), slot.clone());
        self.pool.spawn(move || {
            let out = catch_unwind(AssertUnwindSafe(|| f(&job_ctx))).map_err(|p| {
                let msg = p
                    .downcast_ref::<&str>()
                    .map(|s| (*s).to_owned())
                    .or_else(|| p.downcast_ref::<String>().cloned())
                    .unwrap_or_else(|| "unknown panic".to_owned());
                JobError::Panicked(msg)
            });
            if out.is_ok() {
                job_ctx.set_fraction(1.0);
            }
            {
                // result and flag change under the lock, so `wait` cannot miss the wake-up
                let mut guard = lock(&job_slot.result);
                *guard = Some(out);
                job_slot.finished.store(true, Ordering::Release);
            }
            job_slot.done.notify_all();
        });
        JobHandle {
            name_key: name_key.into(),
            ctx,
            slot,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::mpsc;
    use std::time::Duration;

    fn system() -> JobSystem {
        JobSystem::new(2).unwrap()
    }

    #[test]
    fn returns_result_and_reports_progress() {
        let js = system();
        assert_eq!(js.threads(), 2);
        let (go_tx, go_rx) = mpsc::channel::<()>();
        let (seen_tx, seen_rx) = mpsc::channel::<()>();
        let h = js.spawn("job-test", move |ctx| {
            ctx.set_progress(0.5, Some("job-half"));
            seen_tx.send(()).unwrap();
            go_rx.recv().unwrap();
            21 * 2
        });
        seen_rx.recv().unwrap();
        assert_eq!(h.name_key(), "job-test");
        assert_eq!(
            h.progress(),
            Progress {
                fraction: 0.5,
                message_key: Some("job-half".into())
            }
        );
        assert!(!h.is_finished());
        assert!(h.try_take().is_none());
        go_tx.send(()).unwrap();
        assert_eq!(h.wait().unwrap(), 42);
    }

    #[test]
    fn poll_until_finished() {
        let h = system().spawn("job-test", |_| "done");
        while !h.is_finished() {
            std::thread::sleep(Duration::from_millis(1));
        }
        assert_eq!(h.progress().fraction, 1.0);
        assert_eq!(h.try_take().unwrap().unwrap(), "done");
        assert!(h.try_take().is_none());
    }

    #[test]
    fn cancellation_is_seen_by_the_job() {
        let js = system();
        let (started_tx, started_rx) = mpsc::channel::<()>();
        let h = js.spawn("job-test", move |ctx| {
            started_tx.send(()).unwrap();
            loop {
                if ctx.check_cancelled().is_err() {
                    return Err::<(), _>(Cancelled);
                }
                std::thread::sleep(Duration::from_millis(1));
            }
        });
        started_rx.recv().unwrap();
        h.cancel();
        assert!(h.is_cancelled());
        assert_eq!(h.wait().unwrap(), Err(Cancelled));
    }

    #[test]
    fn panics_are_reported() {
        let h = system().spawn("job-test", |_| -> u32 { panic!("boom") });
        match h.wait() {
            Err(JobError::Panicked(msg)) => assert_eq!(msg, "boom"),
            other => panic!("unexpected {other:?}"),
        }
    }

    #[test]
    fn detached_ctx_clamps() {
        let ctx = JobCtx::detached();
        ctx.set_progress(3.0, None);
        assert_eq!(ctx.progress().fraction, 1.0);
        ctx.cancel_token().cancel();
        assert_eq!(ctx.check_cancelled(), Err(Cancelled));
    }
}
