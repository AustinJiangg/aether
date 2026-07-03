//! Stage 16d-5: unifying the async executor with the per-CPU scheduler.
//!
//! Until now the kernel ran two multitasking models that never coexisted:
//! - the **async executor** (`task/`), which on the BSP `run()`s forever, owning the
//!   CPU and driving the shell — tasks cooperate at `.await`;
//! - the **per-CPU thread scheduler** (`sched`, Stage 16d-3/4), preemptive, which so
//!   far ran only on the application processors.
//!
//! This stage makes them one. The key move: the async executor is no longer a
//! top-level owner of the CPU — it runs **as a kernel thread** on the BSP's own
//! per-CPU run queue, peer to ordinary kernel threads, and the BSP timer preempts
//! between them (`interrupts::timer_dispatch`'s ring-0 path now calls
//! `sched::preempt`). One scheduler dispatches both worlds.
//!
//! Two pieces here:
//! 1. [`demo`] — a small, **testable** proof that runs in *both* build profiles: it
//!    spawns an async-executor thread and a plain kernel thread on the BSP's run
//!    queue and lets the BSP timer preempt them to completion. (The interactive shell
//!    lives only in the non-test build, so `cargo test` could not otherwise cover the
//!    unification.)
//! 2. [`run_shell_threaded`] (non-test) — the real application: the interactive shell
//!    runs as a scheduled kernel thread, alongside a coexisting kernel thread, forever.

use core::sync::atomic::{AtomicU64, Ordering};

use x86_64::instructions::interrupts as int;

use crate::task::executor::Executor;
use crate::task::Task;
use crate::{interrupts, sched, serial_println};

/// How many BSP timer ticks each demo thread spans before finishing. Spanning ≥1 tick
/// guarantees the BSP timer fires — and thus preempts — while the thread runs, so the
/// demo exercises real preemption rather than finishing inside a single time slice.
const DEMO_TICKS: u64 = 2;

/// Work the async task did, running on its executor *thread* (Stage 16d-5). Non-zero
/// proves the async executor ran under the per-CPU scheduler, not as a separate owner
/// of the CPU.
static ASYNC_WORK: AtomicU64 = AtomicU64::new(0);
/// Work the plain kernel thread did, coexisting with the executor thread on the BSP.
static KERNEL_WORK: AtomicU64 = AtomicU64::new(0);

/// Work the async demo task accumulated on its executor thread (for the test).
pub fn async_work() -> u64 {
    ASYNC_WORK.load(Ordering::SeqCst)
}

/// Work the plain demo kernel thread accumulated (for the test).
pub fn kernel_work() -> u64 {
    KERNEL_WORK.load(Ordering::SeqCst)
}

/// Spin (bumping `counter`) until this core has taken `DEMO_TICKS` timer interrupts
/// since the call — long enough that the timer preempts us at least once while we run.
/// `interrupts::timer_ticks` is the BSP's global tick count (bumped on the BSP timer
/// path), so this is a wall-clock-ish window independent of CPU speed.
fn spin_for_demo_ticks(counter: &AtomicU64) {
    let start = interrupts::timer_ticks();
    while interrupts::timer_ticks().wrapping_sub(start) < DEMO_TICKS {
        counter.fetch_add(1, Ordering::SeqCst);
        core::hint::spin_loop();
    }
}

/// A bounded async task: it busy-spins for [`DEMO_TICKS`] of the BSP's timer ticks,
/// bumping [`ASYNC_WORK`], then completes. It does not `.await`, so the executor polls
/// it once; the point is that the *executor thread* running it is preemptible — the BSP
/// timer switches away from it mid-poll to run the kernel thread, and back.
async fn async_demo_task() {
    spin_for_demo_ticks(&ASYNC_WORK);
}

/// Entry of the async-executor demo thread: build an executor, spawn the bounded async
/// task, and run it to completion ([`Executor::run_until_empty`], which returns once the
/// task finishes — so this thread exits and the scheduler reaps it).
fn async_demo_thread() {
    let mut executor = Executor::new();
    executor.spawn(Task::new(async_demo_task()));
    executor.run_until_empty();
}

/// Entry of the plain kernel demo thread: busy-spin for [`DEMO_TICKS`] ticks bumping
/// [`KERNEL_WORK`], then return (the scheduler reaps it). Coexists with the executor
/// thread above, interleaved by timer preemption on the BSP.
fn kernel_demo_thread() {
    spin_for_demo_ticks(&KERNEL_WORK);
}

/// Run the testable unification demo on the BSP, then return. Spawns an async-executor
/// thread and a plain kernel thread on the BSP's own per-CPU run queue and preemptively
/// runs them to completion — proving the async executor and kernel threads coexist under
/// one scheduler. Runs in **both** build profiles, so `cargo test` covers it.
///
/// Call on the BSP after the per-CPU scheduler (`sched::init`) is up. Interrupts may be
/// on or off on entry; we leave them on.
pub fn demo() {
    // Spawn with interrupts off so the spawn + bootstrap registration is atomic w.r.t.
    // the timer; `run_to_completion` enables interrupts itself and idles while the BSP
    // timer preempts the two threads, then returns with interrupts disabled.
    int::disable();
    sched::spawn(async_demo_thread);
    sched::spawn(kernel_demo_thread);
    sched::run_to_completion();
    int::enable();

    serial_println!(
        "[unify] BSP ran an async-executor thread + a kernel thread under one scheduler: \
         async work {}, kernel work {}, BSP preemptions {}",
        async_work(),
        kernel_work(),
        crate::percpu::this_cpu().preemptions(),
    );
}

/// Usable stack for the shell's async-executor thread. The shell polls a deep chain
/// (the async executor, line editing, and filesystem/FAT commands), so it needs far
/// more room than the tiny AP/demo threads' default 4 KiB — it overflows that. (Before
/// guard pages existed, that overflow silently corrupted the heap and then crashed at a
/// garbage address; now [`crate::memory::GuardedStack`] would fault cleanly on the guard
/// page, which is how this was found.) 32 KiB gives ample margin for a singleton thread.
#[cfg(not(test))]
const SHELL_STACK_SIZE: usize = 32 * 1024;

/// Run the interactive shell as a scheduled kernel thread, forever (non-test).
///
/// This is the real unification: instead of the executor `run()`ing as a separate
/// top-level owner of the CPU, the shell's executor is just one thread on the BSP's run
/// queue, coexisting (under timer preemption) with a plain kernel thread. Never returns.
#[cfg(not(test))]
pub fn run_shell_threaded() -> ! {
    // Spawn with interrupts off (atomic w.r.t. the timer); `run_to_completion` enables
    // them. `shell_thread` never finishes, so `run_to_completion` never returns. The
    // shell executor gets a large stack (see `SHELL_STACK_SIZE`); the heartbeat thread
    // does trivial work, so the default stack is plenty.
    int::disable();
    sched::spawn_with_stack(shell_thread, SHELL_STACK_SIZE);
    sched::spawn(heartbeat_thread);
    sched::run_to_completion();
    crate::hlt_loop(); // unreachable: shell_thread runs forever
}

/// The shell as a kernel thread: build the async executor, spawn the shell task, and
/// run it forever ([`Executor::run`]). Reached via the scheduler like any other thread.
#[cfg(not(test))]
fn shell_thread() {
    let mut executor = Executor::new();
    executor.spawn(Task::new(crate::shell::run()));
    executor.run(); // -> !, never returns
}

/// How many heartbeats the coexisting kernel thread prints before exiting (non-test).
#[cfg(not(test))]
const HEARTBEATS: u64 = 5;

/// A plain kernel thread that coexists with the shell thread on the BSP, printing a few
/// heartbeats (one per timer tick) to show — on the serial log / screen — a kernel thread
/// running concurrently with the interactive shell under one preemptive scheduler, then
/// exits. (Printing is safe under preemption: the VGA/serial writers take their lock
/// inside `without_interrupts`, so a print is atomic w.r.t. the timer.)
#[cfg(not(test))]
fn heartbeat_thread() {
    for beat in 1..=HEARTBEATS {
        // Mirror to the screen (VGA) *and* the serial log, like the shell's `sh_println!`,
        // so the coexistence is visible both on screen and in a headless serial capture.
        crate::println!(
            "[kthread] heartbeat {}/{} - a kernel thread sharing the BSP with the shell",
            beat,
            HEARTBEATS,
        );
        serial_println!(
            "[kthread] heartbeat {}/{} - a kernel thread sharing the BSP with the shell",
            beat,
            HEARTBEATS,
        );
        // Spread the beats out by ~one timer tick so they visibly interleave with the
        // shell rather than all printing at once.
        let start = interrupts::timer_ticks();
        while interrupts::timer_ticks().wrapping_sub(start) < 1 {
            core::hint::spin_loop();
        }
    }
    crate::println!("[kthread] heartbeat thread done; the shell keeps running on the scheduler");
    serial_println!("[kthread] heartbeat thread done; the shell keeps running on the scheduler");
}
