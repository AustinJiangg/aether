//! Stage 10: system calls via the `int 0x80` software interrupt.
//!
//! A system call is how ring 3 asks the kernel to do something it is not allowed
//! to do itself (write to the screen, exit, ...). The user executes `int 0x80`;
//! the CPU switches to ring 0 through the IDT (the gate's DPL is set to 3 in
//! `interrupts.rs` so ring 3 is permitted to invoke it) and runs [`syscall_handler`].
//! This is the single, controlled doorway between user and kernel — the thing the
//! shell's `dispatch` only *pretended* to be while everything still ran in ring 0.
//!
//! ## Calling convention (and why it is unusual)
//!
//! Real kernels pass the syscall number and arguments in *registers* (Linux uses
//! `rax` for the number, `rdi`/`rsi`/`rdx`/... for arguments). We can't read those
//! easily: an `extern "x86-interrupt"` handler only receives the
//! [`InterruptStackFrame`], and the compiler-generated prologue has already used
//! the general registers by the time our code runs — capturing them would need a
//! hand-written `#[naked]` assembly stub. To keep this stage about the *mechanism*
//! rather than fiddly assembly, we use a **stack-based** convention instead:
//!
//! - the caller pushes the arguments, then the syscall number (number on top),
//!   then executes `int 0x80`;
//! - the handler finds them through `frame.stack_pointer` (the caller's stack
//!   pointer, which the CPU saved in the interrupt frame), runs the call, and
//!   writes the return value back over the number's slot;
//! - the caller pops the return value.
//!
//! The exact same byte sequence works whether the caller is in ring 0 (the tests
//! and the boot demo, via [`invoke`]) or ring 3 (the user program in Stage 10b).

use core::sync::atomic::{AtomicU64, Ordering};

use x86_64::structures::idt::InterruptStackFrame;

/// Syscall numbers. A real kernel would have dozens; we start with three.
pub const SYS_EXIT: u64 = 0;
pub const SYS_WRITE: u64 = 1;
pub const SYS_GETPID: u64 = 2;

/// Count of syscalls that arrived from ring 3 — proof (for the Stage 10b test)
/// that the user program really crossed into the kernel through `int 0x80`.
static RING3_SYSCALLS: AtomicU64 = AtomicU64::new(0);

/// Number of syscalls made from ring 3 since boot.
pub fn ring3_syscall_count() -> u64 {
    RING3_SYSCALLS.load(Ordering::SeqCst)
}

/// The `int 0x80` handler. Registered in `interrupts.rs` with the gate's DPL set
/// to 3 so ring 3 may invoke it.
///
/// It reads the syscall number and two arguments from the caller's stack (our ABI,
/// above), dispatches, and writes the return value back to the number's slot. The
/// gate is an *interrupt* gate, so interrupts are disabled for the duration — the
/// syscall runs to completion without being preempted.
pub extern "x86-interrupt" fn syscall_handler(mut frame: InterruptStackFrame) {
    // `frame.stack_pointer` is the caller's RSP at the `int 0x80`; by our ABI it
    // points at [number, arg1, arg2].
    let args = frame.stack_pointer.as_u64() as *mut u64;

    // SAFETY: the caller pushed the number and two arguments at its stack top
    // immediately before trapping, so these three slots exist and are writable.
    // (A hardened kernel would verify the range lies in the caller's own mapped
    // stack; our single demo program and the ring 0 tests always satisfy that.)
    let (number, arg1, arg2) = unsafe { (args.read(), args.add(1).read(), args.add(2).read()) };

    // The low two bits of the saved code selector are the caller's privilege level.
    let from_ring3 = frame.code_segment & 0b11 == 3;
    if from_ring3 {
        RING3_SYSCALLS.fetch_add(1, Ordering::SeqCst);
    }

    // `exit` from ring 3 is special: the program is done, so there is no value to
    // hand back to it. Hand off to the scheduler, which rewrites this interrupt's
    // return frame to switch to the next ready process (Stage 12b) or, if none
    // remain, to resume the kernel (Stage 9b's mechanism).
    if number == SYS_EXIT && from_ring3 {
        crate::process::on_user_exit(&mut frame, arg1);
        return;
    }

    let result = dispatch(number, arg1, arg2);

    // SAFETY: same slot, just shown to be writable; this is where the caller will
    // pop the return value from.
    unsafe { args.write(result) };
}

/// Route a syscall number to its implementation. Unknown numbers return `u64::MAX`
/// (our stand-in for "-1 / error").
fn dispatch(number: u64, arg1: u64, arg2: u64) -> u64 {
    match number {
        SYS_WRITE => sys_write(arg1, arg2),
        SYS_GETPID => sys_getpid(),
        SYS_EXIT => sys_exit(arg1),
        _ => {
            crate::serial_println!("[syscall] unknown syscall number {}", number);
            u64::MAX
        }
    }
}

/// `write(ptr, len)` — print `len` bytes of UTF-8 text at `ptr` to the console
/// (screen and serial). Returns the number of bytes written.
fn sys_write(ptr: u64, len: u64) -> u64 {
    // SAFETY: we trust the caller-supplied (ptr, len) — a real kernel would
    // validate that the whole range lies in the caller's address space before
    // dereferencing it. For our demo program and tests the range is always a valid
    // mapped buffer (a string in the user page, or a kernel byte slice).
    let bytes = unsafe { core::slice::from_raw_parts(ptr as *const u8, len as usize) };
    if let Ok(text) = core::str::from_utf8(bytes) {
        crate::print!("{}", text);
        crate::serial_print!("{}", text);
    }
    len
}

/// `getpid()` — return the caller's process id. There are no real processes yet
/// (Stage 12), so this is a fixed placeholder that simply proves a value can be
/// returned from kernel to caller.
fn sys_getpid() -> u64 {
    1
}

/// `exit(code)` — terminate the caller. There is no process to tear down yet, so
/// for now we just log the request and return; Stage 10b will make a ring 3
/// `exit` actually hand control back to the kernel. Returns 0.
fn sys_exit(code: u64) -> u64 {
    crate::serial_println!("[syscall] exit({})", code);
    0
}

/// Invoke a syscall via `int 0x80` from Rust, following the stack-based ABI above.
///
/// Used to exercise the syscall path from ring 0 — the boot demo and the tests.
/// The ring 3 program in Stage 10b performs the identical push/`int`/pop sequence
/// in its own machine code.
///
/// # Safety
///
/// Runs an arbitrary syscall: `SYS_WRITE` dereferences `(arg1, arg2)` as a
/// `(ptr, len)` byte range, so those must describe a valid readable buffer. Other
/// syscalls ignore the arguments.
pub unsafe fn invoke(number: u64, arg1: u64, arg2: u64) -> u64 {
    let ret: u64;
    // Push arg2, arg1, number (so the number ends up on top), trap into the
    // kernel, then pop the return value the handler wrote over the number, and
    // drop the two argument slots. The kernel target disables the red zone, so
    // pushing here cannot corrupt anything below RSP.
    core::arch::asm!(
        "push {a2}",
        "push {a1}",
        "push {nr}",
        "int 0x80",
        "pop {ret}",
        "add rsp, 16",
        nr = in(reg) number,
        a1 = in(reg) arg1,
        a2 = in(reg) arg2,
        ret = out(reg) ret,
    );
    ret
}
