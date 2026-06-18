//! Stage 5: cooperative multitasking built on Rust's `async`/`await`.
//!
//! A task is just a `Future` the kernel agrees to drive forward. The compiler
//! turns every `async fn` into a state machine that implements `Future`: each
//! `.await` becomes a point where the machine can pause (returning
//! `Poll::Pending`) and later resume exactly where it left off. The kernel never
//! has to allocate a separate stack or save/restore CPU registers for a task —
//! it only has to call `Future::poll` repeatedly. Tasks cooperate by yielding at
//! `.await`; nothing here preempts them (that arrives in Stage 6).
//!
//! This module defines the [`Task`] wrapper and its [`TaskId`]. Two executors
//! drive tasks: [`simple_executor`] (a first, busy-polling version, kept for
//! reference) and [`executor`] (waker-driven — it polls a task only when it has
//! been woken, and halts the CPU when none is ready).

pub mod executor;
pub mod keyboard;
pub mod simple_executor;

use core::future::Future;
use core::pin::Pin;
use core::sync::atomic::{AtomicU64, Ordering};
use core::task::{Context, Poll};

use alloc::boxed::Box;

/// A process-wide unique identifier for a spawned task.
///
/// The waker-driven [`executor`] keeps tasks in a map keyed by this id, and a
/// task's waker re-queues it *by id* when woken — so every task needs a stable,
/// unique name. `TaskId(u64)` is a "newtype": a tuple struct wrapping a single
/// `u64`. That gives a distinct type you can't accidentally mix up with a plain
/// integer, at zero runtime cost. The `derive`s generate the trait
/// implementations a map key needs: `Ord`/`Eq` to compare ids, `Copy` so passing
/// one around doesn't move it.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct TaskId(u64);

impl TaskId {
    /// Hand out the next never-before-used id.
    ///
    /// `NEXT_ID` is one counter shared by all tasks: `static` means a single
    /// instance for the whole program, and `Atomic` means incrementing it is safe
    /// even if two CPUs (or an interrupt) touch it at once. `fetch_add` atomically
    /// reads the old value *and* adds 1, returning the old value — so every caller
    /// gets a distinct number. `Ordering::Relaxed` is sufficient here: we only
    /// need the counter to be atomic, not to order any other memory around it.
    fn new() -> TaskId {
        static NEXT_ID: AtomicU64 = AtomicU64::new(0);
        TaskId(NEXT_ID.fetch_add(1, Ordering::Relaxed))
    }
}

/// A unit of cooperative work: a heap-allocated, pinned future resolving to `()`.
///
/// We store the future as `Pin<Box<dyn Future>>` for two reasons. `Box<dyn ...>`
/// erases the concrete (compiler-generated, unnameable) future type, so tasks of
/// different shapes share one type and live on the heap — exactly what Stage 4's
/// allocator made possible. `Pin` then promises the future will never move again:
/// a self-referential state machine (one whose saved state holds a reference into
/// itself) would be corrupted if relocated after the first `poll`, so `poll` is
/// only offered on a *pinned* future.
pub struct Task {
    /// This task's unique id, assigned at creation (see [`TaskId`]).
    id: TaskId,
    future: Pin<Box<dyn Future<Output = ()>>>,
}

impl Task {
    /// Wrap any future that returns `()` and lives for `'static` into a task.
    ///
    /// `'static` is required because the executor may keep the task around
    /// indefinitely; it must not borrow anything shorter-lived. `Box::pin` moves
    /// the future onto the heap and pins it there in a single step. Each task also
    /// gets a fresh unique `id`.
    pub fn new(future: impl Future<Output = ()> + 'static) -> Task {
        Task {
            id: TaskId::new(),
            future: Box::pin(future),
        }
    }

    /// Advance the task by polling its future once.
    ///
    /// Returns `Poll::Ready(())` when the task has finished, or `Poll::Pending`
    /// if it yielded at an `.await` and should be polled again later. Kept private
    /// to this module tree: executors call it, while outside code uses `spawn`.
    fn poll(&mut self, context: &mut Context) -> Poll<()> {
        self.future.as_mut().poll(context)
    }
}
