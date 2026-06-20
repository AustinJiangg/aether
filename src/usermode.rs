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

/// Hand-assemble the ring 3 program `write(msg_ptr, msg_len); exit(0)`.
///
/// It speaks the kernel's stack-based syscall ABI (see [`crate::syscall`]): push
/// the arguments, push the number, `int 0x80`. In machine code:
///
/// ```text
///   push msg_len         6A <len>        ; write: arg2 = len (imm8, so len < 128)
///   mov  rax, msg_ptr    48 B8 <ptr64>   ;        arg1 = ptr (a 64-bit address
///   push rax             50              ;        needs mov-then-push)
///   push 1               6A 01           ;        number = SYS_WRITE
///   int  0x80            CD 80
///   push 0               6A 00           ; exit:  arg1 = exit code 0
///   push 0               6A 00           ;        number = SYS_EXIT
///   int  0x80            CD 80           ;        never returns to ring 3
/// ```
pub(crate) fn build_user_program(msg_ptr: u64, msg_len: usize) -> [u8; 23] {
    let mut program: [u8; 23] = [
        0x6A, msg_len as u8, // push msg_len
        0x48, 0xB8, 0, 0, 0, 0, 0, 0, 0, 0, // mov rax, imm64 (msg_ptr, filled below)
        0x50, // push rax
        0x6A, crate::syscall::SYS_WRITE as u8, // push SYS_WRITE
        0xCD, 0x80, // int 0x80
        0x6A, 0x00, // push exit code 0
        0x6A, crate::syscall::SYS_EXIT as u8, // push SYS_EXIT
        0xCD, 0x80, // int 0x80
    ];
    // Patch the 8-byte little-endian immediate for `mov rax, msg_ptr`.
    program[4..12].copy_from_slice(&msg_ptr.to_le_bytes());
    program
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

    // Disable interrupts so no tick fires between arming the hook and the descent.
    // The user frame's RFLAGS has IF set, so interrupts come back on the instant we
    // enter ring 3 — and the very next tick then interrupts the ring 3 code.
    interrupts::disable();
    EXPECT_USER_TICK.store(true, Ordering::SeqCst);

    let user_frame = InterruptStackFrameValue {
        instruction_pointer: user_entry,
        code_segment: u64::from(gdt::user_code_selector().0),
        cpu_flags: 0x202, // reserved bit 1, plus IF (bit 9): ring 3 with interrupts on
        stack_pointer: user_stack_top, // top of the program's user stack
        stack_segment: u64::from(gdt::user_data_selector().0),
    };

    serial_println!(
        "[usermode] entering ring 3 at {:?} (cs={:#x}, ss={:#x})",
        user_entry,
        user_frame.code_segment,
        user_frame.stack_segment
    );

    // SAFETY: the frame describes a valid ring 3 context — `user_entry` is a
    // mapped, user-accessible, executable page; `user_stack_top` is the top of a
    // mapped, user-accessible, writable stack; CS/SS are the GDT's RPL 3 selectors;
    // RFLAGS is sane with IF set. `rsp0` is installed (Stage 9a), so the first
    // interrupt taken from ring 3 has a valid kernel stack. `iretq` thus transitions
    // cleanly to user mode and does not return here. The caller has already switched
    // CR3 to the address space that maps `user_entry` and `user_stack_top`.
    unsafe { user_frame.iretq() }
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
