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
//! *rewrites its own return frame* to a ring 0 context ([`resume_kernel`]). That
//! is triggered either by the timer catching a spinning program ([`on_timer_tick`])
//! or by a ring 3 `exit` syscall. It is exactly the mechanism a scheduler will
//! later use to switch the CPU between a user process and the kernel; here it just
//! lets boot continue (into the shell or the tests) after a brief ring 3 excursion.

use core::sync::atomic::{AtomicBool, AtomicU64, Ordering};

use x86_64::instructions::interrupts;
use x86_64::structures::idt::{InterruptStackFrame, InterruptStackFrameValue};
use x86_64::VirtAddr;

use crate::{gdt, serial_println};

// --- state shared with the timer interrupt handler -------------------------

/// Set while a descent to ring 3 is in flight: it tells [`on_timer_tick`] to watch
/// for the tick that interrupts ring 3 and to perform the one-time return rewrite.
static EXPECT_USER_TICK: AtomicBool = AtomicBool::new(false);

/// Set once the timer has observed the CPU running in ring 3 (`CPL == 3`). Read by
/// the Stage 9b test and logged during boot.
static REACHED_RING3: AtomicBool = AtomicBool::new(false);

/// Where [`on_timer_tick`] resumes the kernel after pulling us out of ring 3: the
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
/// We record where to resume the kernel (for [`on_timer_tick`]), then forge a ring
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

    // Disable interrupts so nothing fires between arming the hook and the descent.
    // Stage 12b runs user processes with IF *cleared* (see `initial_user_frame`), so they
    // stay uninterrupted until their next syscall; interrupts come back on only when
    // the kernel finally resumes (in `boot_continue`).
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
    // RFLAGS has IF clear (cooperative). `rsp0` is installed (Stage 9a), so a syscall
    // taken from ring 3 has a valid kernel stack. `iretq` thus transitions cleanly to
    // user mode and does not return here. The caller has already switched CR3 to the
    // address space that maps `user_entry` and `user_stack_top`.
    unsafe { user_frame.iretq() }
}

/// Build the ring 3 interrupt-return frame for a freshly-started user program:
/// entry point, user stack, and the GDT's RPL 3 code/data selectors.
///
/// RFLAGS clears IF: Stage 12b runs user processes *cooperatively* — no timer
/// preemption yet — so a process runs uninterrupted until it `yield`s or `exit`s,
/// and the scheduler switches processes only at those points. (Stage 12c will set IF
/// and add preemption, which needs saving a preempted process's full register state.)
pub(crate) fn initial_user_frame(
    entry: VirtAddr,
    stack_top: VirtAddr,
) -> InterruptStackFrameValue {
    InterruptStackFrameValue {
        instruction_pointer: entry,
        code_segment: u64::from(gdt::user_code_selector().0),
        cpu_flags: 0x002, // reserved bit 1 only; IF clear (no interrupts in ring 3)
        stack_pointer: stack_top,
        stack_segment: u64::from(gdt::user_data_selector().0),
    }
}

/// Read the execution point an in-flight interrupt will return to — a running user
/// process's context (instruction pointer, stack pointer, flags, selectors). The
/// scheduler captures this when a process `yield`s, to resume it later.
pub fn save_frame(frame: &InterruptStackFrame) -> InterruptStackFrameValue {
    **frame
}

/// Rewrite an in-flight interrupt's return frame so its `iretq` resumes `target` — a
/// user process's context. The scheduler uses this from inside the syscall handler
/// to switch to another process; the caller must already have switched CR3 to that
/// process's address space.
pub fn load_frame(frame: &mut InterruptStackFrame, target: InterruptStackFrameValue) {
    // SAFETY: `target` is a valid ring 3 context (RPL 3 selectors, a mapped entry and
    // stack in the now-active address space) — either a fresh entry frame from
    // `initial_user_frame` or one previously captured by `save_frame`. Overwriting
    // the return frame makes the handler's `iretq` enter it.
    unsafe { frame.as_mut().write(target) };
}

/// Leave ring 3: rewrite `frame` so the in-flight interrupt's `iretq` resumes the
/// kernel continuation (the `resume` passed to [`enter`]) in ring 0, instead of
/// returning to the user program. Shared by the two triggers below.
///
/// Idempotent via an atomic test-and-clear: whichever fires first — the timer
/// catching a spinning program, or a ring 3 `exit` syscall — wins, and any later
/// call is a no-op.
pub fn resume_kernel(frame: &mut InterruptStackFrame) {
    if !EXPECT_USER_TICK.swap(false, Ordering::SeqCst) {
        return; // already left ring 3 once
    }
    REACHED_RING3.store(true, Ordering::SeqCst);

    // A ring 0 context: the kernel continuation's RIP, the kernel code selector,
    // the kernel stack we saved in `enter`, SS = 0 (the kernel runs with a null
    // stack selector in long mode), and IF cleared so the continuation re-enables
    // interrupts deliberately.
    let resumed = InterruptStackFrameValue {
        instruction_pointer: VirtAddr::new(RESUME_RIP.load(Ordering::SeqCst)),
        code_segment: RESUME_CS.load(Ordering::SeqCst),
        cpu_flags: 0x002,
        stack_pointer: VirtAddr::new(RESUME_RSP.load(Ordering::SeqCst)),
        stack_segment: 0,
    };

    // SAFETY: we overwrite the interrupt-return frame with a valid ring 0 context
    // (kernel CS, a kernel stack pointer captured in `enter`, RIP at the kernel
    // continuation). The handler's `iretq` then resumes kernel execution there.
    unsafe { frame.as_mut().write(resumed) };
}

/// Called from the timer interrupt handler on every tick. A no-op unless a ring 3
/// descent is in flight and this tick caught the CPU in ring 3 (`CPL == 3`).
///
/// This is the *fallback* return path, for a user program that spins (Stage 9b's
/// `EB FE`). Stage 10b's program exits via a syscall before a tick lands, so in
/// practice [`resume_kernel`] is usually reached through `exit` instead.
pub fn on_timer_tick(frame: &mut InterruptStackFrame) {
    // The low two bits of the saved code selector are the privilege level the
    // interrupt came from; 3 means the timer struck while the CPU was in ring 3.
    if !EXPECT_USER_TICK.load(Ordering::SeqCst) || frame.code_segment & 0b11 != 3 {
        return;
    }
    serial_println!("[usermode] timer interrupted ring 3 code (CPL=3); returning to the kernel");
    resume_kernel(frame);
}
