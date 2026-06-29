//! Stage 16d-3: a per-CPU cooperative run queue.
//!
//! Stage 6 built a *single, global* round-robin scheduler (`thread`): one ready
//! queue, one "current thread" — fundamentally single-core, since both answers are
//! global. With the APs awake (Stage 16c), each running its own timer (16d-1), and
//! a context switch proven to work on a non-boot core (16d-2), the next step is a
//! scheduler whose state is *private to each core*: one run queue per CPU, where a
//! core only ever schedules among **its own** threads. This module is that run
//! queue.
//!
//! It deliberately reuses the proven pieces rather than reinventing the risky ones:
//! - the architecture context switch is [`crate::thread::context_switch`] — the
//!   exact routine Stage 6 hand-wrote and Stage 16d-2 validated on an AP;
//! - the fabricated initial stack frame mirrors `thread`'s `prepare_stack`.
//!
//! What is genuinely new is *where the scheduler state lives*: not one global, but
//! one [`RunQueue`] per core, found by the running core's dense `cpu_index` (from
//! [`crate::percpu`]). The storage is published exactly like the per-CPU array — a
//! heap `Vec` leaked to a `'static` slice behind an [`AtomicPtr`] + length — the
//! storage classes an AP is already proven to reach (the 0.9 bootloader may leave
//! large `.bss` unmapped; see `percpu`/`smp`).
//!
//! Stage 16d-3 drove this cooperatively (a thread ran until it called [`yield_now`]);
//! **Stage 16d-4** adds **preemption**: this core's timer interrupt calls [`preempt`],
//! which performs the very same `switch_to_next` from interrupt context, so a thread is
//! switched out at any instruction without its cooperation. The trick is the same one
//! the process scheduler uses (Stage 12c) — the timer's naked stub has already saved the
//! interrupted thread's full register set in a `TrapFrame` on its stack, so `context_switch`
//! need only swap stacks; when the thread is resumed, the stub's epilogue restores that
//! `TrapFrame` and `iretq`s back to the exact instruction the tick interrupted. (Stage 16d-5
//! folds the async executor in.)

use alloc::boxed::Box;
use alloc::collections::{BTreeMap, VecDeque};
use alloc::vec::Vec;
use core::sync::atomic::{AtomicPtr, AtomicU64, AtomicUsize, Ordering};

use spin::{Mutex, MutexGuard};
use x86_64::instructions::interrupts;

use crate::percpu;
use crate::thread::context_switch;

/// Size of each scheduled thread's stack, in bytes (4 KiB).
///
/// Deliberately small: these threads do trivial work and a 4 KiB stack absorbs a
/// timer-interrupt frame comfortably (a kernel thread runs with interrupts on).
/// The budget matters because every woken AP may run its run queue *concurrently*
/// — `smp::boot_aps` lets an AP start as soon as it reports online, before the next
/// is woken — so the worst case is every core's threads alive at once. With 4 CPUs,
/// `AP_THREADS` threads each, on the small (100 KiB) kernel heap, 4 KiB keeps the
/// peak well under budget. Stacks are freed when [`run_to_completion`] returns.
const THREAD_STACK_SIZE: usize = 4 * 1024;

/// A globally-unique thread id, handed out by an atomic counter. Unlike Stage 6's
/// scheduler the queues are per-core, but a single id space keeps every `BTreeMap`
/// key distinct and makes a thread identifiable across cores in a log.
fn next_id() -> u64 {
    static NEXT: AtomicU64 = AtomicU64::new(0);
    NEXT.fetch_add(1, Ordering::Relaxed)
}

/// Where a thread is in its lifecycle (mirrors Stage 6's `ThreadState`).
#[derive(Clone, Copy, PartialEq, Eq)]
enum State {
    /// Waiting in the ready queue for its turn on the CPU.
    Ready,
    /// Currently executing (exactly one thread per core is `Running`).
    Running,
    /// Returned from its entry function; awaiting cleanup by [`reap_finished`].
    Finished,
}

/// One kernel thread on a core's run queue: its state, the memory backing its
/// stack, the saved stack pointer to resume it from, and the function it runs.
struct KThread {
    state: State,
    /// Backing memory for the stack. Held only to *own* the allocation for the
    /// thread's lifetime (and free it on `Drop` when reaped). Empty for the
    /// bootstrap thread, which runs on the core's existing stack.
    #[allow(dead_code)]
    stack: Box<[u8]>,
    /// The thread's stack pointer while it is *not* running. `context_switch`
    /// writes it on the way out and reads it on the way back in.
    stack_pointer: u64,
    /// The function the thread runs, or `None` for the bootstrap thread (which is
    /// already executing real code and never enters via [`thread_trampoline`]).
    entry: Option<fn()>,
    /// True for the per-core bootstrap context registered by [`run_to_completion`];
    /// it cycles through the round-robin as a yield-only thread but never finishes.
    is_bootstrap: bool,
}

/// One core's private round-robin run queue (the per-CPU analog of Stage 6's
/// single global `Scheduler`).
struct RunQueue {
    /// Every live thread on this core, owned here and looked up by id.
    threads: BTreeMap<u64, KThread>,
    /// Ids of this core's ready threads, in turn order. Invariant: holds exactly
    /// the `Ready` threads; the `Running` thread is never in it.
    ready: VecDeque<u64>,
    /// The thread currently on this core.
    current: Option<u64>,
}

impl RunQueue {
    const fn new() -> RunQueue {
        RunQueue {
            threads: BTreeMap::new(),
            ready: VecDeque::new(),
            current: None,
        }
    }
}

// The per-CPU run queues live on the heap, one per core, published through an
// `AtomicPtr` + length — the same scheme `percpu` uses, and the storage classes an
// AP is proven to reach. `QUEUES_PTR` is non-null only after [`init`].
static QUEUES_PTR: AtomicPtr<Mutex<RunQueue>> = AtomicPtr::new(core::ptr::null_mut());
static QUEUES_LEN: AtomicUsize = AtomicUsize::new(0);

/// Build one empty run queue per core. Call **once** on the BSP, after
/// [`percpu::init`] and **before** waking any AP — an AP reaches for its own queue
/// (`spawn`) the moment it enters the scheduler in `ap_entry`. `n_cpus` is the core
/// count ([`percpu::count`]); a core's queue is indexed by its dense `cpu_index`.
pub fn init(n_cpus: usize) {
    let mut queues: Vec<Mutex<RunQueue>> = Vec::with_capacity(n_cpus);
    for _ in 0..n_cpus {
        queues.push(Mutex::new(RunQueue::new()));
    }
    // Leak to a 'static slice (it lives for the rest of the kernel's life) and
    // publish: length first, then the base pointer last, so a reader that sees a
    // non-null pointer also sees the correct length.
    let leaked: &'static mut [Mutex<RunQueue>] = Vec::leak(queues);
    QUEUES_LEN.store(leaked.len(), Ordering::SeqCst);
    QUEUES_PTR.store(leaked.as_mut_ptr(), Ordering::SeqCst);
}

/// All per-CPU run queues, or an empty slice before [`init`].
fn queues() -> &'static [Mutex<RunQueue>] {
    let ptr = QUEUES_PTR.load(Ordering::SeqCst);
    if ptr.is_null() {
        return &[];
    }
    let len = QUEUES_LEN.load(Ordering::SeqCst);
    // SAFETY: after `init`, `(ptr, len)` describe the leaked 'static heap slice; it
    // is never freed or moved, and each `Mutex<RunQueue>` guards its own interior
    // mutation, so handing out `&'static [Mutex<RunQueue>]` is sound.
    unsafe { core::slice::from_raw_parts(ptr, len) }
}

/// This core's own run queue, found by its dense `cpu_index`. Works on any core
/// because [`percpu::this_cpu`] resolves the running core by its Local APIC id.
/// Panics if called before [`init`] (or on a core with no per-CPU block).
fn this_queue() -> &'static Mutex<RunQueue> {
    let index = percpu::this_cpu().cpu_index;
    queues()
        .get(index)
        .expect("sched::this_queue: no run queue for the running core (init not called?)")
}

/// This core's run queue, or `None` if the queues are not built yet (before [`init`])
/// or the running core has no per-CPU block. The non-panicking form, for the timer's
/// [`preempt`] path, which must never fault no matter how early a tick lands.
fn this_queue_opt() -> Option<&'static Mutex<RunQueue>> {
    let index = percpu::this_cpu_opt()?.cpu_index;
    queues().get(index)
}

/// Spawn a kernel thread that will run `entry` on the **current** core's run queue.
///
/// Allocates a stack, fabricates an initial frame on it (see [`prepare_stack`]) so
/// the first switch into the thread lands in [`thread_trampoline`], then registers
/// it as `Ready`. Call on the core that should run it (e.g. from `ap_entry`).
pub fn spawn(entry: fn()) {
    let id = next_id();
    let mut stack = alloc::vec![0u8; THREAD_STACK_SIZE].into_boxed_slice();
    // SAFETY: `stack` is freshly allocated, writable, and far larger than the small
    // fabricated frame `prepare_stack` writes.
    let stack_pointer = unsafe { prepare_stack(&mut stack) };

    let mut q = this_queue().lock();
    q.threads.insert(
        id,
        KThread {
            state: State::Ready,
            stack,
            stack_pointer,
            entry: Some(entry),
            is_bootstrap: false,
        },
    );
    q.ready.push_back(id);
}

/// Run every thread spawned on this core to completion under **timer preemption**,
/// then return (Stage 16d-4).
///
/// Registers the calling (bootstrap) context as a thread so the scheduler always has
/// somewhere to come back to, then **enables interrupts** and idles on `hlt`. From
/// there this core's timer drives the rotation: each tick [`preempt`]s whatever is
/// running (this bootstrap, or a worker busy-spinning) to the next ready thread — no
/// `yield` required. When every worker has `Finished`, a tick finds only the
/// bootstrap ready (a no-op switch), so control keeps returning to the idle loop,
/// which then reaps the workers' stacks and returns to `ap_entry`.
///
/// Call once per core, with interrupts disabled (we enable them ourselves once the
/// bootstrap is safely registered).
pub fn run_to_completion() {
    // Register the bootstrap context (no entry, empty stack — it runs on the core's
    // existing stack; its stack_pointer is filled in by the first switch away).
    let bootstrap_id = next_id();
    {
        let mut q = this_queue().lock();
        q.threads.insert(
            bootstrap_id,
            KThread {
                state: State::Running,
                stack: Vec::new().into_boxed_slice(),
                stack_pointer: 0,
                entry: None,
                is_bootstrap: true,
            },
        );
        q.current = Some(bootstrap_id);
        // Reserve ready-queue capacity for every thread now, with interrupts still
        // off, so the `push_back` in the timer-driven `switch_to_next` never has to
        // grow the deque (which would allocate — spinning on the heap lock from inside
        // an interrupt). After this the preemptive switch path touches no allocator.
        let n = q.threads.len();
        q.ready.reserve(n);
    }

    // Arm preemption: enable this core's timer interrupt to switch threads for us.
    interrupts::enable();
    loop {
        reap_finished();
        let work_remains = {
            let q = this_queue().lock();
            q.threads
                .values()
                .any(|t| !t.is_bootstrap && t.state != State::Finished)
        };
        if !work_remains {
            break;
        }
        // Sleep until the next timer interrupt, which preempts us into a ready worker.
        x86_64::instructions::hlt();
    }

    // Restore the interrupts-disabled state `ap_entry` expects, then free the finished
    // workers' stacks (we are the bootstrap, running on a stack nobody reaps).
    interrupts::disable();
    reap_finished();
}

/// Voluntarily give up the current core's CPU to the next ready thread on **this
/// core** (round-robin). Returns `true` if it switched, `false` if this thread was
/// the only one ready. Disables interrupts around the whole switch so the timer
/// cannot preempt between picking the next thread and the stack swap.
///
/// The cooperative API. Since Stage 16d-4 the AP demo relies on timer preemption
/// instead, so nothing calls this at runtime — but it remains the voluntary
/// switch point a thread would use, and shares its switch path with [`preempt`].
#[allow(dead_code)]
pub fn yield_now() -> bool {
    interrupts::without_interrupts(|| switch_to_next(this_queue().lock()))
}

/// Preempt the kernel thread running on this core, rotating to the next ready thread
/// on this core's run queue (Stage 16d-4). Called from the timer interrupt
/// ([`crate::interrupts::timer_dispatch`]) on an application processor.
///
/// We are already in interrupt context with interrupts disabled — exactly what
/// [`switch_to_next`] requires. We `try_lock` the queue so that if this core was
/// interrupted while mid-update (already holding the lock), we skip this tick rather
/// than spin forever on a lock only we could release. A no-op when nothing else is
/// ready (e.g. the core is parked with just its bootstrap thread), so it costs only
/// a `try_lock` once the workers are done.
pub fn preempt() {
    let q = match this_queue_opt() {
        Some(q) => q,
        None => return, // run queues not built yet
    };
    if let Some(guard) = q.try_lock() {
        if guard.ready.is_empty() {
            return; // nothing to switch to; the guard drops here
        }
        percpu::this_cpu().count_preemption();
        switch_to_next(guard);
    }
}

/// Rotate to the next ready thread on this core, given an already-acquired lock.
/// Moves the current thread to the back of the ready queue, makes the front thread
/// current, **drops the lock**, then switches. Returns `false` (lock dropped) if
/// nothing else is ready. Shared by the cooperative [`yield_now`] and the preemptive
/// [`preempt`].
///
/// Callers MUST have interrupts disabled: `context_switch` must never run with
/// interrupts on (a tick striking mid-switch, while the stack is half-swapped, is
/// fatal), and the switch must be atomic w.r.t. the timer. The interrupt path is
/// allocation-free — `run_to_completion` pre-reserved the ready queue, so the
/// `push_back` below never grows the deque.
fn switch_to_next(mut guard: MutexGuard<'static, RunQueue>) -> bool {
    let current_id = guard
        .current
        .expect("sched::switch_to_next: no current thread");
    let next_id = match guard.ready.pop_front() {
        Some(id) => id,
        None => return false, // nothing else ready; the lock drops on return
    };

    guard.threads.get_mut(&current_id).unwrap().state = State::Ready;
    guard.ready.push_back(current_id);
    guard.threads.get_mut(&next_id).unwrap().state = State::Running;
    guard.current = Some(next_id);

    let new_rsp = guard.threads[&next_id].stack_pointer;
    let old_rsp: *mut u64 = &mut guard.threads.get_mut(&current_id).unwrap().stack_pointer;
    drop(guard);

    // SAFETY: `old_rsp` is the current thread's live saved-sp slot inside its
    // heap-owned `KThread`; `new_rsp` was produced by `context_switch` or
    // `prepare_stack`. The lock is dropped so the resumed thread can lock the queue
    // itself; the caller guarantees interrupts are disabled, so the swap is atomic.
    unsafe {
        context_switch(old_rsp, new_rsp);
    }
    true
}

/// The trampoline every spawned thread runs first, reached via the return address
/// [`prepare_stack`] fabricated. It looks up its own entry function on this core's
/// run queue, runs it, then exits. It can never return: there is no valid return
/// address above it on the fabricated stack.
extern "C" fn thread_trampoline() -> ! {
    // Fetch the entry function for the thread now current on this core. The lock is
    // released before calling into it so that code can `yield_now` (which locks this
    // core's queue again) without deadlocking.
    let entry = {
        let q = this_queue().lock();
        let id = q.current.expect("thread_trampoline: no current thread");
        q.threads[&id]
            .entry
            .expect("the bootstrap thread must never reach thread_trampoline")
    };

    // A switch *into* a thread happens with interrupts disabled; a kernel thread
    // must run with them enabled (so its core's timer can still tick — counted
    // harmlessly per-CPU on an AP). Turn them on before running the body.
    interrupts::enable();

    entry();

    thread_exit();
}

/// End the current thread on this core: mark it `Finished`, tally the completion in
/// this core's per-CPU block, and switch away, never to return.
///
/// We must *not* free our own stack here — we are still running on it. The thread
/// stays `Finished` in the map until [`reap_finished`], running on another stack,
/// drops it.
fn thread_exit() -> ! {
    // Disable interrupts so the timer cannot preempt us partway through the switch.
    // We never return, so we never re-enable them; the thread we hand the CPU to
    // restores its own interrupt state as it resumes.
    interrupts::disable();

    let (old_rsp, new_rsp) = {
        let q = this_queue();
        let mut guard = q.lock();
        let current_id = guard.current.expect("thread_exit: no current thread");
        guard.threads.get_mut(&current_id).unwrap().state = State::Finished;

        // Hand the CPU to the next ready thread. The bootstrap thread cycles through
        // the ready queue, so there is always one to switch to.
        let next_id = guard
            .ready
            .pop_front()
            .expect("thread_exit: no ready thread (the bootstrap should always cycle)");
        guard.threads.get_mut(&next_id).unwrap().state = State::Running;
        guard.current = Some(next_id);

        let new_rsp = guard.threads[&next_id].stack_pointer;
        // We are finished, so saving our own stack pointer is pointless; hand
        // `context_switch` our own (soon-to-be-reaped) slot as a throwaway target.
        let old_rsp: *mut u64 = &mut guard.threads.get_mut(&current_id).unwrap().stack_pointer;
        (old_rsp, new_rsp)
    };

    // Record the completion on this core (after the queue update, before the switch
    // — we never come back). A non-zero count proves a thread ran to completion here.
    percpu::this_cpu().complete_thread();

    // SAFETY: `old_rsp` is a valid writable slot and `new_rsp` was produced by
    // `context_switch`/`prepare_stack`; the lock is dropped and interrupts are
    // disabled, so the switch cannot be interrupted.
    unsafe {
        context_switch(old_rsp, new_rsp);
    }
    unreachable!("a finished thread was scheduled again");
}

/// Free the stacks of any `Finished` threads on this core's run queue. Safe only
/// from a thread not itself being reaped — [`run_to_completion`] calls it from the
/// bootstrap context, which is never `Finished`. Removing a thread drops its
/// `Box<[u8]>` stack, returning that memory to the heap.
fn reap_finished() {
    let mut q = this_queue().lock();
    let finished: Vec<u64> = q
        .threads
        .iter()
        .filter(|(_, t)| t.state == State::Finished)
        .map(|(id, _)| *id)
        .collect();
    for id in finished {
        q.threads.remove(&id);
    }
}

/// Fabricate the initial stack image for a brand-new thread, laid out to look
/// *exactly* as if it had previously called [`context_switch`] and been suspended —
/// so the restore half can resume it with no special-casing. From the top down we
/// write a return address (pointing at [`thread_trampoline`]) and the six
/// callee-saved register slots `context_switch` pops. Returns the lowest fabricated
/// word, which becomes the thread's saved stack pointer.
///
/// This mirrors Stage 6's `thread::prepare_stack`; the save/restore register order
/// must stay in lockstep with [`context_switch`].
///
/// # Safety
///
/// `stack` must be a writable buffer large enough for the fabricated frame (a
/// handful of words); any real thread stack is far larger.
unsafe fn prepare_stack(stack: &mut [u8]) -> u64 {
    let bottom = stack.as_mut_ptr() as u64;
    let top = bottom + stack.len() as u64;

    // Start 16-byte aligned at the top of the buffer and write words downward.
    let mut sp = top & !0xF;

    // One padding word, so that after the six pops + `ret`, `thread_trampoline`
    // begins with rsp ≡ 8 (mod 16), as the System V ABI requires.
    sp -= 8;

    // The return address `ret` jumps to on the first switch-in.
    sp -= 8;
    let entry_trampoline = thread_trampoline as extern "C" fn() -> !;
    // SAFETY: `sp` is inside the buffer and 8-byte aligned; write one u64.
    (sp as *mut u64).write(entry_trampoline as usize as u64);

    // Six callee-saved register slots (values irrelevant for a never-run thread).
    for _ in 0..6 {
        sp -= 8;
        // SAFETY: still inside the buffer, 8-byte aligned, writing one u64.
        (sp as *mut u64).write(0);
    }

    debug_assert_eq!(sp % 16, 0, "fabricated stack pointer must be 16-byte aligned");
    sp
}
