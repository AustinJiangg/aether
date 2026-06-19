//! Stage 6a: cooperative kernel threads.
//!
//! Stage 5's async tasks share a single kernel stack and cooperate by returning
//! `Poll::Pending` at an `.await`; the compiler turns each task into a state
//! machine, so no CPU registers are ever saved. A *kernel thread* is the heavier,
//! more general model: each thread owns its **own stack** and a full **CPU
//! register context**, and we move between threads by hand-rolled
//! [`switch::context_switch`].
//!
//! For this stage the switches are still **cooperative** — a thread keeps the CPU
//! until it calls [`yield_now`]. Stage 6b will trigger the very same machinery
//! from the timer interrupt, turning it into *preemptive* scheduling where a
//! thread can be switched out at any instruction, even mid-loop.
//!
//! The scheduling policy is the simplest possible: **round-robin**. A
//! [`Scheduler`] keeps every thread in a map and a queue of the ones that are
//! ready to run; [`yield_now`] rotates to the front of the queue.
//!
//! Stacks are allocated from the Stage 4c heap. They have **no guard page**, so a
//! thread that overflows its stack silently corrupts adjacent heap data rather
//! than faulting — an acceptable simplification for now (mapped, guard-paged
//! stacks are a possible later refinement).

mod switch;

use alloc::collections::{BTreeMap, VecDeque};
use alloc::vec::Vec;
use core::sync::atomic::{AtomicU64, Ordering};

use spin::Mutex;

use crate::{hlt_loop, println, serial_println};

/// Size of each kernel thread's stack, in bytes (16 KiB). Generous for the demo
/// threads, and large enough to absorb an interrupt frame (the timer can fire
/// while a thread is running, pushing its frame onto that thread's stack).
const STACK_SIZE: usize = 16 * 1024;

/// A unique identifier for a kernel thread, handed out by an atomic counter —
/// the same newtype pattern as `TaskId` in the async module.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct ThreadId(u64);

impl ThreadId {
    fn new() -> ThreadId {
        static NEXT_ID: AtomicU64 = AtomicU64::new(0);
        ThreadId(NEXT_ID.fetch_add(1, Ordering::Relaxed))
    }
}

/// Where a thread is in its lifecycle.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum ThreadState {
    /// Waiting in the ready queue for its turn on the CPU.
    Ready,
    /// Currently executing (exactly one thread is `Running` at a time).
    Running,
    /// Returned from its entry function; awaiting cleanup by `reap_finished`.
    Finished,
}

/// One kernel thread: its state, the memory backing its stack, the saved stack
/// pointer to resume it from, and the function it runs.
struct Thread {
    state: ThreadState,
    /// Backing memory for the thread's stack. Never read directly; it exists only
    /// to *own* the allocation so the stack stays alive for the thread's
    /// lifetime, and to free it (on `Drop`) when the thread is removed. Empty for
    /// the bootstrap thread, which runs on the original boot stack.
    #[allow(dead_code)]
    stack: alloc::boxed::Box<[u8]>,
    /// The thread's stack pointer while it is *not* running. Meaningless while the
    /// thread is `Running`. `context_switch` writes it on the way out and reads it
    /// on the way back in.
    stack_pointer: u64,
    /// The function the thread runs. `None` for the bootstrap thread (thread 0),
    /// which is already executing real code and never enters via `thread_entry`.
    entry: Option<fn()>,
}

/// The round-robin scheduler: every thread, the queue of ready ones, and which
/// thread is currently running.
struct Scheduler {
    /// Every live thread, owned here and looked up by id.
    threads: BTreeMap<ThreadId, Thread>,
    /// Ids of threads ready to run, in turn order. Invariant: this holds exactly
    /// the `Ready` threads; the `Running` thread is never in it.
    ready_queue: VecDeque<ThreadId>,
    /// The thread currently on the CPU.
    current: Option<ThreadId>,
}

impl Scheduler {
    const fn new() -> Scheduler {
        Scheduler {
            threads: BTreeMap::new(),
            ready_queue: VecDeque::new(),
            current: None,
        }
    }
}

/// The kernel's single scheduler.
///
/// In Stage 6a nothing in interrupt context touches this lock (the timer handler
/// only counts ticks), so a plain spinlock is safe. Stage 6b, which reschedules
/// *from* the timer interrupt, will have to disable interrupts around accesses to
/// avoid an interrupt deadlocking against a thread that already holds the lock.
static SCHEDULER: Mutex<Scheduler> = Mutex::new(Scheduler::new());

/// Register the current (boot) execution context as thread 0.
///
/// The bootstrap thread already has a stack — the one the bootloader handed the
/// kernel — so it owns no heap stack and needs no fabricated frame; its
/// `stack_pointer` is filled in automatically the first time it is switched away
/// from. Giving it a thread of its own means the scheduler always has somewhere
/// to return to once every spawned thread has finished. Call once before any
/// `spawn` or `run`.
pub fn init() {
    let id = ThreadId::new();
    let mut scheduler = SCHEDULER.lock();
    scheduler.threads.insert(
        id,
        Thread {
            state: ThreadState::Running,
            stack: Vec::new().into_boxed_slice(), // empty: uses the boot stack
            stack_pointer: 0,                     // set on the first switch away
            entry: None,
        },
    );
    scheduler.current = Some(id);
}

/// Spawn a new kernel thread that will run `entry`, and return its id.
///
/// We allocate a stack, fabricate an initial frame on it (see `prepare_stack`) so
/// the first switch into the thread lands in `thread_entry`, then register it as
/// `Ready`.
pub fn spawn(entry: fn()) -> ThreadId {
    let id = ThreadId::new();

    // Allocate the thread's stack on the heap. `into_boxed_slice` gives us an
    // owning `Box<[u8]>` of exactly STACK_SIZE bytes.
    let mut stack = alloc::vec![0u8; STACK_SIZE].into_boxed_slice();

    // SAFETY: `stack` is a freshly allocated, writable buffer comfortably larger
    // than the small fabricated frame, which is all `prepare_stack` requires.
    let stack_pointer = unsafe { prepare_stack(&mut stack) };

    let mut scheduler = SCHEDULER.lock();
    scheduler.threads.insert(
        id,
        Thread {
            state: ThreadState::Ready,
            stack,
            stack_pointer,
            entry: Some(entry),
        },
    );
    scheduler.ready_queue.push_back(id);
    id
}

/// Fabricate the initial stack image for a brand-new thread.
///
/// The image is laid out to look *exactly* as if the thread had previously called
/// `context_switch` and been suspended — so the restore half of `context_switch`
/// can resume it with no special-casing. From the top of the stack downward we
/// write: a return address (pointing at `thread_entry`) and the six callee-saved
/// register slots `context_switch` will pop. We return the address of the lowest
/// fabricated word, which becomes the thread's saved stack pointer.
///
/// Alignment matters: the System V ABI requires `rsp ≡ 8 (mod 16)` at a
/// function's first instruction. After `context_switch` pops the six registers
/// and `ret`s into `thread_entry`, `rsp` equals `new_rsp + 56`, so we choose
/// `new_rsp` to be 16-byte aligned (one padding word above the return address
/// makes that work out).
///
/// # Safety
///
/// `stack` must be a writable buffer large enough to hold the fabricated frame
/// (a handful of words); any real thread stack is far larger.
unsafe fn prepare_stack(stack: &mut [u8]) -> u64 {
    let bottom = stack.as_mut_ptr() as u64;
    let top = bottom + stack.len() as u64;

    // Start at the 16-byte-aligned top of the buffer and write words downward.
    let mut sp = top & !0xF;

    // One padding word, so that after the six pops + `ret` below, `thread_entry`
    // starts with rsp ≡ 8 (mod 16) as the ABI demands.
    sp -= 8;

    // The return address `ret` will jump to on the very first switch-in.
    sp -= 8;
    let entry_trampoline = thread_entry as extern "C" fn() -> !;
    // SAFETY: `sp` is inside the buffer and 8-byte aligned; we write one u64.
    (sp as *mut u64).write(entry_trampoline as usize as u64);

    // Six callee-saved register slots (the values are irrelevant for a thread
    // that has never run, so zero them).
    for _ in 0..6 {
        sp -= 8;
        // SAFETY: still inside the buffer, 8-byte aligned, writing one u64.
        (sp as *mut u64).write(0);
    }

    // `sp` now points at the lowest fabricated word — the value `context_switch`
    // will load as the new stack pointer.
    debug_assert_eq!(sp % 16, 0, "fabricated stack pointer must be 16-byte aligned");
    sp
}

/// The trampoline every new thread runs first, reached via the return address
/// fabricated by `prepare_stack`. It looks up its own entry function, runs it,
/// and then exits. It can never return: there is no valid return address above it
/// on the fabricated stack.
extern "C" fn thread_entry() -> ! {
    // We are now the running thread; fetch the function we were spawned with. The
    // lock is released before we call into user code so that code can `yield_now`
    // (which locks the scheduler again) without deadlocking.
    let entry = {
        let scheduler = SCHEDULER.lock();
        let id = scheduler
            .current
            .expect("thread_entry reached with no current thread");
        scheduler.threads[&id]
            .entry
            .expect("thread 0 must never reach thread_entry")
    };

    entry();

    thread_exit();
}

/// End the current thread. Mark it `Finished` and switch away, never to return.
///
/// We must *not* free our own stack here — we are still running on it. The thread
/// stays in the map (now `Finished`) until `reap_finished`, running on a different
/// stack, drops it.
fn thread_exit() -> ! {
    let (old_rsp, new_rsp) = {
        let mut scheduler = SCHEDULER.lock();
        let current_id = scheduler
            .current
            .expect("thread_exit reached with no current thread");
        scheduler.threads.get_mut(&current_id).unwrap().state = ThreadState::Finished;

        // Hand the CPU to the next ready thread. The bootstrap thread is normally
        // queued, so this almost always succeeds; if somehow nothing is ready,
        // there is no thread left to run, so just halt on this (now-dead) stack.
        let next_id = match scheduler.ready_queue.pop_front() {
            Some(id) => id,
            None => {
                drop(scheduler);
                serial_println!("[scheduler] last thread exited; halting");
                hlt_loop();
            }
        };
        scheduler.threads.get_mut(&next_id).unwrap().state = ThreadState::Running;
        scheduler.current = Some(next_id);

        let new_rsp = scheduler.threads[&next_id].stack_pointer;
        // We are finished, so saving our own stack pointer is pointless; hand
        // `context_switch` our own (soon-to-be-freed) slot as a throwaway target.
        let old_rsp: *mut u64 = &mut scheduler.threads.get_mut(&current_id).unwrap().stack_pointer;
        (old_rsp, new_rsp)
    };

    // SAFETY: `old_rsp` is a valid writable slot and `new_rsp` was produced by
    // `context_switch`/`prepare_stack`; the lock is dropped before switching.
    unsafe {
        switch::context_switch(old_rsp, new_rsp);
    }
    unreachable!("a finished thread was scheduled again");
}

/// Voluntarily give up the CPU to the next ready thread (round-robin).
///
/// Returns `true` if it actually switched, `false` if this thread was the only
/// one ready (in which case it simply keeps running).
pub fn yield_now() -> bool {
    let (old_rsp, new_rsp) = {
        let mut scheduler = SCHEDULER.lock();
        let current_id = scheduler
            .current
            .expect("yield_now reached with no current thread");

        // If no other thread is ready, there is nothing to switch to.
        let next_id = match scheduler.ready_queue.pop_front() {
            Some(id) => id,
            None => return false,
        };

        // The current thread goes to the back of the ready queue...
        scheduler.threads.get_mut(&current_id).unwrap().state = ThreadState::Ready;
        scheduler.ready_queue.push_back(current_id);
        // ...and the one we popped takes the CPU.
        scheduler.threads.get_mut(&next_id).unwrap().state = ThreadState::Running;
        scheduler.current = Some(next_id);

        let new_rsp = scheduler.threads[&next_id].stack_pointer;
        let old_rsp: *mut u64 = &mut scheduler.threads.get_mut(&current_id).unwrap().stack_pointer;
        (old_rsp, new_rsp)
    };

    // SAFETY: both come from live `Thread` entries — `old_rsp` is the current
    // thread's saved-sp slot, `new_rsp` was produced by `context_switch` or
    // `prepare_stack`. Crucially, the scheduler lock is released above *before* we
    // switch, so the thread we resume can lock the scheduler itself (e.g. to
    // `yield_now` in turn) without deadlocking against us.
    unsafe {
        switch::context_switch(old_rsp, new_rsp);
    }
    true
}

/// Free the stacks of any threads that have finished.
///
/// Safe to call only from a thread that is not itself being reaped — we run this
/// from the bootstrap thread in [`run`], so we are never freeing the stack we are
/// standing on. Removing a thread from the map drops its `Box<[u8]>` stack,
/// returning that memory to the heap.
fn reap_finished() {
    let mut scheduler = SCHEDULER.lock();
    let finished: Vec<ThreadId> = scheduler
        .threads
        .iter()
        .filter(|(_, thread)| thread.state == ThreadState::Finished)
        .map(|(id, _)| *id)
        .collect();
    for id in finished {
        scheduler.threads.remove(&id);
    }
}

/// Hand the bootstrap thread over to the scheduler and never return.
///
/// We round-robin through the spawned threads with `yield_now`, reaping finished
/// ones each time around. Because nothing blocks in Stage 6a, the first time
/// `yield_now` reports it could not switch means every other thread has finished
/// — so we clean up and halt the CPU.
pub fn run() -> ! {
    loop {
        reap_finished();
        if !yield_now() {
            reap_finished();
            serial_println!("[scheduler] all kernel threads finished; idling");
            println!("All kernel threads finished; kernel is now idle.");
            hlt_loop();
        }
    }
}
