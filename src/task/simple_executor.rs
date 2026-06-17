//! A first, deliberately naive executor.
//!
//! `SimpleExecutor` keeps tasks in a FIFO queue and polls them round-robin: a
//! task that returns `Poll::Pending` goes straight to the back of the queue to be
//! tried again. That is correct but wasteful — with nothing to do it spins at
//! 100% CPU instead of sleeping, and it ignores wakers entirely (it hands every
//! poll a no-op "dummy" waker). It exists to show the bare mechanism; a later
//! step replaces it with a waker-driven executor that halts the CPU via `hlt`
//! when no task is ready to make progress.

use super::Task;

use alloc::collections::VecDeque;
use core::task::{Context, Poll, RawWaker, RawWakerVTable, Waker};

/// A round-robin executor that polls queued tasks until all of them finish.
pub struct SimpleExecutor {
    task_queue: VecDeque<Task>,
}

impl SimpleExecutor {
    /// Create an executor with an empty task queue.
    pub fn new() -> SimpleExecutor {
        SimpleExecutor {
            task_queue: VecDeque::new(),
        }
    }

    /// Queue a task to be run by [`run`](Self::run).
    pub fn spawn(&mut self, task: Task) {
        self.task_queue.push_back(task)
    }

    /// Poll queued tasks round-robin until the queue empties, then return.
    ///
    /// A `Ready` task is dropped; a `Pending` task goes to the back and is
    /// retried. Because we never actually wait on a waker, a task that stays
    /// `Pending` forever would spin here forever — acceptable only because our
    /// `example_task` runs to completion.
    pub fn run(&mut self) {
        while let Some(mut task) = self.task_queue.pop_front() {
            let waker = dummy_waker();
            let mut context = Context::from_waker(&waker);
            match task.poll(&mut context) {
                Poll::Ready(()) => {}
                Poll::Pending => self.task_queue.push_back(task),
            }
        }
    }
}

/// Build a `RawWaker` whose every operation is a no-op.
///
/// A `Waker` is ultimately a data pointer plus a vtable of four function
/// pointers (`clone`, `wake`, `wake_by_ref`, `drop`). Ours carries a null data
/// pointer and functions that do nothing — except `clone`, which must hand back
/// another valid `RawWaker`, so it simply rebuilds the same dummy.
fn dummy_raw_waker() -> RawWaker {
    fn no_op(_: *const ()) {}
    fn clone(_: *const ()) -> RawWaker {
        dummy_raw_waker()
    }

    let vtable = &RawWakerVTable::new(clone, no_op, no_op, no_op);
    RawWaker::new(core::ptr::null::<()>(), vtable)
}

/// Wrap [`dummy_raw_waker`] into a real `Waker`.
fn dummy_waker() -> Waker {
    // SAFETY: `Waker::from_raw` requires the `RawWaker`'s vtable to uphold the
    // waker contract. Our vtable functions never dereference the data pointer (it
    // is null and unused) and have no side effects, and `clone` returns a fresh,
    // equally-valid dummy `RawWaker`. There is nothing to misuse, so the resulting
    // `Waker` is sound.
    unsafe { Waker::from_raw(dummy_raw_waker()) }
}
