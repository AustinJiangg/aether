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
//! This module defines the [`Task`] wrapper. The executor that actually polls
//! tasks lives in [`simple_executor`] — a first, busy-polling version. A later
//! step adds a waker-driven executor that can sleep the CPU between events.

pub mod keyboard;
pub mod simple_executor;

use core::future::Future;
use core::pin::Pin;
use core::task::{Context, Poll};

use alloc::boxed::Box;

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
    future: Pin<Box<dyn Future<Output = ()>>>,
}

impl Task {
    /// Wrap any future that returns `()` and lives for `'static` into a task.
    ///
    /// `'static` is required because the executor may keep the task around
    /// indefinitely; it must not borrow anything shorter-lived. `Box::pin` moves
    /// the future onto the heap and pins it there in a single step.
    pub fn new(future: impl Future<Output = ()> + 'static) -> Task {
        Task {
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
