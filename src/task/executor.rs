//! A waker-driven executor (Stage 5).
//!
//! `SimpleExecutor` re-polled *every* task in a loop, even ones with nothing to
//! do — burning the CPU. This executor polls a task **only when it has been
//! woken**. It keeps all tasks in a map (`tasks`) and a separate queue of "ready"
//! task ids (`ready_queue`); it polls just the ready ones. A task that returns
//! `Poll::Pending` is left untouched in the map until *its waker* pushes its id
//! back onto `ready_queue`.
//!
//! How a wake reaches us: when we poll the keyboard task we pass it a [`Waker`]
//! built from a [`TaskWaker`] carrying that task's id. The keyboard stream stashes
//! that waker; later the interrupt handler calls `wake()` on it, which pushes the
//! task's id onto `ready_queue` so our next sweep polls it again.
//!
//! When `ready_queue` is empty, `sleep_if_idle` halts the CPU with `hlt` until an
//! interrupt arrives, instead of spinning — so an idle kernel uses no CPU.

use alloc::collections::BTreeMap;
use alloc::sync::Arc;
use alloc::task::Wake;
use core::task::{Context, Poll, Waker};

use crossbeam_queue::ArrayQueue;

use super::{Task, TaskId};

/// Capacity of the ready queue. We only ever have a handful of tasks, so this is
/// generous; a wake that can't fit would panic rather than be silently lost.
const READY_QUEUE_CAPACITY: usize = 100;

/// Owns every task and decides which to poll next.
pub struct Executor {
    /// Every spawned task, owned here and looked up by id.
    tasks: BTreeMap<TaskId, Task>,
    /// Ids of tasks ready to be polled. Wrapped in `Arc` (an atomically
    /// reference-counted shared pointer) because every task's waker holds a clone
    /// of it and pushes onto it when woken. `ArrayQueue` is lock-free, so a waker
    /// fired from inside an interrupt can push without risking a deadlock.
    ready_queue: Arc<ArrayQueue<TaskId>>,
    /// One cached `Waker` per task, so we build a task's waker once and reuse it
    /// on every poll instead of rebuilding it each time.
    waker_cache: BTreeMap<TaskId, Waker>,
}

impl Executor {
    /// Create an empty executor.
    pub fn new() -> Executor {
        Executor {
            tasks: BTreeMap::new(),
            ready_queue: Arc::new(ArrayQueue::new(READY_QUEUE_CAPACITY)),
            waker_cache: BTreeMap::new(),
        }
    }

    /// Register a task and mark it ready for its first poll.
    pub fn spawn(&mut self, task: Task) {
        let task_id = task.id;
        // `insert` returns the previous value for this key, if any. A task id is
        // unique, so a previous value would mean a bug — fail loudly.
        if self.tasks.insert(task_id, task).is_some() {
            panic!("a task with this id is already spawned");
        }
        self.ready_queue
            .push(task_id)
            .expect("ready_queue is full");
    }

    /// Poll every currently-ready task once, draining the ready queue.
    fn run_ready_tasks(&mut self) {
        // Destructure `self` so the borrow checker lets us borrow its fields
        // independently below. Writing `self.tasks` and `self.waker_cache` inside
        // the loop would each borrow *all* of `self`, which would conflict.
        let Self {
            tasks,
            ready_queue,
            waker_cache,
        } = self;

        while let Some(task_id) = ready_queue.pop() {
            // The id might name a task that already finished and was removed; if
            // so, skip it.
            let task = match tasks.get_mut(&task_id) {
                Some(task) => task,
                None => continue,
            };
            // Reuse this task's cached waker, or build (and store) it the first
            // time. `entry(...).or_insert_with(closure)` runs the closure only if
            // the key is absent.
            let waker = waker_cache
                .entry(task_id)
                .or_insert_with(|| TaskWaker::waker(task_id, ready_queue.clone()));
            let mut context = Context::from_waker(waker);
            match task.poll(&mut context) {
                // Finished: drop the task and its cached waker.
                Poll::Ready(()) => {
                    tasks.remove(&task_id);
                    waker_cache.remove(&task_id);
                }
                // Not done; leave it in the map. It will be re-queued by its waker.
                Poll::Pending => {}
            }
        }
    }

    /// Run forever, polling tasks as they become ready and halting the CPU when
    /// there is nothing to do.
    ///
    /// The return type `!` ("never") says this function does not return — there is
    /// no caller in the kernel to return to. Each iteration drains the ready
    /// tasks, then `sleep_if_idle` puts the CPU to sleep until the next interrupt
    /// if no task is waiting.
    pub fn run(&mut self) -> ! {
        loop {
            self.run_ready_tasks();
            self.sleep_if_idle();
        }
    }

    /// Halt the CPU until the next interrupt, but only if no task is ready.
    ///
    /// The subtlety is a race against interrupts. Between checking "is the ready
    /// queue empty?" and executing `hlt`, a keyboard interrupt could fire, enqueue
    /// a scancode, and wake its task — pushing onto `ready_queue`. If we then
    /// halted anyway, that wakeup would be wasted and the task would not run until
    /// the *next*, unrelated interrupt (or never). So we disable interrupts while
    /// we decide:
    ///   - if the queue is empty, `enable_and_hlt` runs `sti; hlt`. On x86 `sti`
    ///     only takes effect *after the following instruction*, so interrupts stay
    ///     masked until the `hlt` is already executing — a pending interrupt
    ///     cannot slip in between the check and the halt; it fires the instant we
    ///     halt and wakes us straight back up;
    ///   - if work appeared just before we disabled interrupts, we re-enable them
    ///     and loop around to poll it, without sleeping.
    fn sleep_if_idle(&self) {
        use x86_64::instructions::interrupts::{self, enable_and_hlt};

        interrupts::disable();
        if self.ready_queue.is_empty() {
            enable_and_hlt();
        } else {
            interrupts::enable();
        }
    }
}

/// The waker handed to a task: when woken, it pushes the task's id back onto the
/// executor's ready queue, so the next sweep polls that task again.
struct TaskWaker {
    task_id: TaskId,
    ready_queue: Arc<ArrayQueue<TaskId>>,
}

impl TaskWaker {
    /// Build a `Waker` for `task_id`. We wrap our `TaskWaker` in an `Arc` and let
    /// the standard library turn it into a `Waker` through the `Wake` trait below.
    fn waker(task_id: TaskId, ready_queue: Arc<ArrayQueue<TaskId>>) -> Waker {
        Waker::from(Arc::new(TaskWaker {
            task_id,
            ready_queue,
        }))
    }

    fn wake_task(&self) {
        self.ready_queue
            .push(self.task_id)
            .expect("ready_queue is full");
    }
}

/// Implementing `Wake` is what makes `Waker::from(Arc<TaskWaker>)` work: the
/// standard library builds the low-level `RawWaker` vtable for us from these two
/// methods. Both just re-queue the task — `wake` consumes the `Arc`, while
/// `wake_by_ref` only borrows it (used when the caller wants to keep its waker).
impl Wake for TaskWaker {
    fn wake(self: Arc<Self>) {
        self.wake_task();
    }

    fn wake_by_ref(self: &Arc<Self>) {
        self.wake_task();
    }
}
