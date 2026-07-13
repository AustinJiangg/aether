//! Stages 9b/10b/12a: dropping to user mode (ring 3), running a program there,
//! and returning to the kernel.
//!
//! Everything so far has run in ring 0 (full privilege). A real OS runs user
//! programs in ring 3, where they cannot touch kernel memory or execute
//! privileged instructions. Stage 9a installed the two prerequisites in the GDT
//! and TSS — ring 3 code/data segments, and `rsp0` (the kernel stack the CPU
//! switches to when an interrupt arrives while in ring 3). This stage uses them.
//!
//! There is no "jump to a lower privilege level" instruction. The trick is to
//! *forge an interrupt-return frame*: we build the exact stack image the CPU
//! would have pushed had it interrupted a ring 3 program, then execute `iretq`.
//! The CPU believes it is returning from an interrupt and lands in ring 3.
//! ([`InterruptStackFrameValue::iretq`] writes that frame and runs `iretq` for
//! us, so we describe the target context as a struct instead of hand-writing
//! assembly.)
//!
//! Proving it worked: in Stage 9b the ring 3 program was two bytes — `EB FE`, an
//! infinite `jmp .` loop — and the proof was the timer interrupt entering the
//! kernel through `rsp0` and finding the saved code selector at `CPL == 3` (which
//! proved both that we ran in ring 3 and that `rsp0` averted a triple fault).
//! Stage 10b replaced the loop with a real program that calls `write` then `exit`;
//! Stage 11b/12a goes further — the program is now a real ELF that `process.rs`
//! maps into its *own* address space, and [`enter`] runs it in ring 3 on that
//! space's CR3 (every space maps the kernel, so the `int 0x80` syscalls still
//! reach the kernel handler).
//!
//! Returning to the kernel: an interrupt handler normally `iretq`s back to where
//! it came from — the ring 3 code. To resume the *kernel* instead, the handler
//! *rewrites its own return frame* to a ring 0 context ([`resume_kernel`]). The
//! scheduler triggers this when the *last* user process exits (Stage 12b); it then
//! lets boot continue (into the shell or the tests) after the ring 3 excursion.
//! Stage 24d-2 adds a second flavor of the same machinery ([`enter_returning`]): the
//! excursion becomes an ordinary call that **returns** to its caller — a
//! setjmp/longjmp built over the identical resume point — so the *running shell* can
//! launch a user program (its `nc` command) and simply continue when it exits.

use core::arch::naked_asm;
use core::sync::atomic::{AtomicBool, AtomicU64, Ordering};

use x86_64::instructions::interrupts;
use x86_64::structures::idt::InterruptStackFrameValue;
use x86_64::VirtAddr;

use crate::{gdt, serial_println};

// --- state shared with the timer interrupt handler -------------------------

/// Set while a ring 3 excursion is in flight; [`resume_kernel`] swaps it back to
/// `false` so it performs the one-time return-to-kernel rewrite exactly once.
static EXPECT_USER_TICK: AtomicBool = AtomicBool::new(false);

/// Set once the timer has observed the CPU running in ring 3 (`CPL == 3`). Read by
/// the Stage 9b test and logged during boot.
static REACHED_RING3: AtomicBool = AtomicBool::new(false);

/// Where [`resume_kernel`] resumes the kernel after the ring 3 excursion: the
/// continuation's instruction pointer, the kernel stack pointer to run it on, and
/// the ring 0 code selector. Filled in by [`enter`] before the descent.
static RESUME_RIP: AtomicU64 = AtomicU64::new(0);
static RESUME_RSP: AtomicU64 = AtomicU64::new(0);
static RESUME_CS: AtomicU64 = AtomicU64::new(0);

/// Whether the kernel has observed ring 3 execution. Used by the Stage 9b test and
/// logged by the boot continuation.
pub fn reached_ring3() -> bool {
    REACHED_RING3.load(Ordering::SeqCst)
}

/// Drop to ring 3 at `user_entry`; never returns to the caller.
///
/// We record where to resume the kernel (for [`resume_kernel`]), then forge a ring
/// 3 interrupt-return frame and `iretq` into it. `resume` is where the kernel
/// continues *after* the timer has pulled us back out of ring 3; it runs in ring 0
/// on the current (boot) kernel stack and must never return — it takes over the
/// rest of boot.
pub fn enter(user_entry: VirtAddr, user_stack_top: VirtAddr, resume: fn() -> !) -> ! {
    // Capture the current kernel stack pointer; `resume` will run on it. This is
    // the boot stack, already known to be large enough to run the shell (it does
    // today). `enter` never returns, so reusing the stack below this point is safe.
    let kernel_rsp: u64;
    // SAFETY: reading RSP into a general register has no side effects.
    unsafe { core::arch::asm!("mov {}, rsp", out(reg) kernel_rsp, options(nomem, nostack)) };

    RESUME_RIP.store(resume as usize as u64, Ordering::SeqCst);
    // 16-byte align, then bias down by 8, so `resume` starts with the stack the
    // System V ABI expects at a function's first instruction (rsp ≡ 8 mod 16). The
    // few bytes below the captured RSP are unused stack, free to grow into.
    RESUME_RSP.store((kernel_rsp & !0xF) - 8, Ordering::SeqCst);
    RESUME_CS.store(u64::from(gdt::kernel_code_selector().0), Ordering::SeqCst);

    descend(user_entry, user_stack_top)
}

/// Stage 24d-2: enter ring 3 like [`enter`], but make the excursion **callable** — when
/// [`resume_kernel`] fires (the last process exited), control comes back to
/// `enter_returning`'s caller as an ordinary function return, instead of jumping into a
/// fresh `fn() -> !` continuation that owns the rest of boot.
///
/// This is what lets the *shell* run a user program and carry on afterwards: the boot
/// model resets the stack and forgets every in-flight stack frame (fine at boot, where
/// nothing above matters), but a shell command sits deep in live frames — the async
/// executor, the dispatch loop, the command handler — that must all still be there when
/// the program finishes. The mechanics are a setjmp/longjmp pair built from pieces the
/// kernel already has:
///
/// - **setjmp** ([`setjmp_and_start`]): save the six callee-saved registers on the
///   current stack (exactly like `thread::context_switch`'s outgoing half — everything
///   else is caller-saved, already spilled by the compiler around this call), publish
///   the resulting RSP and a landing label as the resume point, then start the phase.
/// - **longjmp** ([`resume_kernel`], unchanged): the `exit` syscall's handler rewrites
///   its own interrupt frame so the `iretq` lands at the published RIP/RSP — which is
///   now the landing label rather than a boot continuation. The landing code pops the
///   callee-saved registers back and returns.
///
/// `start` performs the actual descent (switch CR3, `iretq` — it must not return); it
/// runs on the current stack, below the saved resume point. On return, the caller is
/// still on whatever CR3 the last process used, with interrupts off — it must switch
/// back to the kernel space and re-enable interrupts (see `process::run_and_return`).
pub fn enter_returning(start: extern "C" fn() -> !) {
    RESUME_CS.store(u64::from(gdt::kernel_code_selector().0), Ordering::SeqCst);
    // SAFETY: `start` diverges into ring 3; the only way back is `resume_kernel`
    // targeting the resume point the stub records, which restores this thread's
    // callee-saved registers from its own stack and returns normally.
    unsafe { setjmp_and_start(start) };
}

/// The naked setjmp half of [`enter_returning`]. Saves the callee-saved registers and
/// the resume point, then jumps to `start` (in `rdi`, which never returns); the landing
/// label `2:` below is where [`resume_kernel`]'s rewritten `iretq` re-enters this
/// kernel context, unwinding the save.
///
/// Register discipline mirrors [`crate::thread::context_switch`]: a plain `call` into
/// this function means the compiler has already spilled every caller-saved register it
/// needs, so only `rbp`/`rbx`/`r12`-`r15` (plus RSP itself) carry state across the
/// excursion. The general-purpose registers at the landing label hold whatever the
/// exiting user process left in its final `TrapFrame` — garbage to us, and dead: the
/// pops below rebuild the callee-saved set, and the caller expects nothing else.
#[unsafe(naked)]
unsafe extern "C" fn setjmp_and_start(start: extern "C" fn() -> !) {
    naked_asm!(
        // --- setjmp: save this kernel context ---
        "push rbp",
        "push rbx",
        "push r12",
        "push r13",
        "push r14",
        "push r15",
        // Publish the resume point: RIP = the landing label, RSP = right here (atop the
        // six saved registers). Plain 64-bit stores are atomic on x86_64, and the reader
        // (`resume_kernel`, in a later syscall on this same CPU) cannot run before the
        // descent below, so no further ordering is needed.
        "lea rax, [rip + 2f]",
        "mov [rip + {resume_rip}], rax",
        "mov [rip + {resume_rsp}], rsp",
        // --- start the excursion (never returns) ---
        // `jmp`, not `call`: `start` gets this RSP as its entry stack (rsp ≡ 8 mod 16,
        // as after a call), and no dead return address is left behind.
        "jmp rdi",
        // --- longjmp target: resume_kernel's iretq lands here, RSP already restored ---
        "2:",
        "pop r15",
        "pop r14",
        "pop r13",
        "pop r12",
        "pop rbx",
        "pop rbp",
        "ret",
        resume_rip = sym RESUME_RIP,
        resume_rsp = sym RESUME_RSP,
    );
}

/// The descent itself: arm the return hook and `iretq` into ring 3 at `user_entry`.
/// Shared tail of [`enter`] (boot, one-shot continuation) and the Stage 24d-2 returning
/// path (`process::run_and_return` via [`enter_returning`]) — the caller has already
/// published the resume point in `RESUME_RIP`/`RESUME_RSP`/`RESUME_CS`, in whichever
/// flavor it wants, and switched CR3 to the space mapping `user_entry`.
pub(crate) fn descend(user_entry: VirtAddr, user_stack_top: VirtAddr) -> ! {
    // Disable interrupts so none fires on the kernel stack between arming the hook and
    // the descent. The `iretq` below loads the user frame's RFLAGS, which has IF set
    // (Stage 12c-3), so interrupts turn back on *atomically* as the CPU enters ring 3
    // — the first tick then preempts the process, not this half-finished descent.
    interrupts::disable();
    EXPECT_USER_TICK.store(true, Ordering::SeqCst);

    let user_frame = initial_user_frame(user_entry, user_stack_top);

    serial_println!(
        "[usermode] entering ring 3 at {:?} (cs={:#x}, ss={:#x})",
        user_entry,
        user_frame.code_segment,
        user_frame.stack_segment
    );

    // SAFETY: the frame describes a valid ring 3 context — `user_entry` is a
    // mapped, user-accessible, executable page; `user_stack_top` is the top of a
    // mapped, user-accessible, writable stack; CS/SS are the GDT's RPL 3 selectors;
    // RFLAGS has IF set (Stage 12c-3 preemption). `rsp0` is installed (Stage 9a), so a
    // timer interrupt or syscall taken from ring 3 switches to a valid kernel stack.
    // `iretq` thus transitions cleanly to user mode and does not return here. The
    // caller has already switched CR3 to the address space that maps `user_entry` and
    // `user_stack_top`.
    unsafe { user_frame.iretq() }
}

/// Build the ring 3 interrupt-return frame for a freshly-started user program:
/// entry point, user stack, and the GDT's RPL 3 code/data selectors.
///
/// RFLAGS sets IF (Stage 12c-3): a user process runs with interrupts *enabled*, so a
/// timer tick can preempt it mid-execution and the scheduler can switch to another
/// process without its cooperation (`yield`/`exit` remain voluntary switch points).
/// Resuming a preempted process correctly relies on the full-register `TrapFrame` that
/// every switch now saves (Stage 12c-2) — a tick can strike between any two
/// instructions, with live state in any register.
pub(crate) fn initial_user_frame(
    entry: VirtAddr,
    stack_top: VirtAddr,
) -> InterruptStackFrameValue {
    InterruptStackFrameValue {
        instruction_pointer: entry,
        code_segment: u64::from(gdt::user_code_selector().0),
        cpu_flags: 0x202, // reserved bit 1 + IF (bit 9): interrupts enabled in ring 3
        stack_pointer: stack_top,
        stack_segment: u64::from(gdt::user_data_selector().0),
    }
}

/// Leave ring 3: rewrite `iframe` so the in-flight interrupt's `iretq` resumes the
/// kernel continuation (the `resume` passed to [`enter`]) in ring 0, instead of
/// returning to the user program. Called by the scheduler from the `exit` syscall
/// when the *last* user process terminates; `iframe` is the interrupt frame inside
/// that syscall's [`crate::interrupts::TrapFrame`].
///
/// Idempotent via an atomic test-and-clear (armed in [`enter`]): a stray second call
/// is a harmless no-op.
pub fn resume_kernel(iframe: &mut InterruptStackFrameValue) {
    if !EXPECT_USER_TICK.swap(false, Ordering::SeqCst) {
        return; // already left ring 3 once
    }
    REACHED_RING3.store(true, Ordering::SeqCst);

    // A ring 0 context: the kernel continuation's RIP, the kernel code selector, the
    // kernel stack we saved in `enter`, SS = 0 (the kernel runs with a null stack
    // selector in long mode), and IF cleared so the continuation re-enables
    // interrupts deliberately.
    //
    // A plain assignment (not a volatile write) is correct: `iframe` points into the
    // TrapFrame on the kernel stack, which the syscall stub reads back via its
    // explicit `pop`/`iretq` after `syscall_dispatch` returns — the compiler cannot
    // elide a write another (assembly) reader observes.
    *iframe = InterruptStackFrameValue {
        instruction_pointer: VirtAddr::new(RESUME_RIP.load(Ordering::SeqCst)),
        code_segment: RESUME_CS.load(Ordering::SeqCst),
        cpu_flags: 0x002,
        stack_pointer: VirtAddr::new(RESUME_RSP.load(Ordering::SeqCst)),
        stack_segment: 0,
    };
}

