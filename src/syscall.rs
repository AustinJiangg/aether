//! Stage 10: system calls via the `int 0x80` software interrupt.
//!
//! A system call is how ring 3 asks the kernel to do something it is not allowed
//! to do itself (write to the screen, exit, ...). The user executes `int 0x80`;
//! the CPU switches to ring 0 through the IDT (the gate's DPL is set to 3 in
//! `interrupts.rs` so ring 3 is permitted to invoke it) and runs [`syscall_entry`].
//! This is the single, controlled doorway between user and kernel — the thing the
//! shell's `dispatch` only *pretended* to be while everything still ran in ring 0.
//!
//! ## Calling convention (and why it is unusual)
//!
//! Real kernels pass the syscall number and arguments in *registers* (Linux uses
//! `rax` for the number, `rdi`/`rsi`/`rdx`/... for arguments). We use a simpler
//! **stack-based** convention instead, so the demo's hand-assembled programs and the
//! ring 0 [`invoke`] helper can share one tiny push/`int`/pop sequence:
//!
//! - the caller pushes the arguments, then the syscall number (number on top),
//!   then executes `int 0x80`;
//! - the handler finds them through the saved `iframe.stack_pointer` (the caller's
//!   stack pointer, which the CPU saved in the interrupt frame), runs the call, and
//!   writes the return value back over the number's slot;
//! - the caller pops the return value.
//!
//! The exact same byte sequence works whether the caller is in ring 0 (the tests
//! and the boot demo, via [`invoke`]) or ring 3 (the user programs).
//!
//! ## Entry stub (Stage 12c-2)
//!
//! Originally the handler was an `extern "x86-interrupt"` function, which only
//! exposes the `InterruptStackFrame` — not the general-purpose registers. That was
//! fine while switches were cooperative and the demo kept no live register state
//! across a `yield`. Preemption raises the stakes: a process must be resumable with
//! its *exact* registers, so [`syscall_entry`] is now a hand-written *naked* stub
//! (mirroring the timer's) that captures the full [`TrapFrame`]. Both kernel entries
//! then save and restore identical state, and the scheduler can move a process
//! between them without corrupting a register.

use core::arch::naked_asm;
use core::sync::atomic::{AtomicU64, Ordering};

use crate::interrupts::TrapFrame;

/// Syscall numbers. A real kernel would have dozens; we start with a handful.
pub const SYS_EXIT: u64 = 0;
pub const SYS_WRITE: u64 = 1;
pub const SYS_GETPID: u64 = 2;
/// `yield()` — voluntarily give up the CPU so the scheduler runs another process
/// (Stage 12b cooperative multitasking). Meaningful only from ring 3.
pub const SYS_YIELD: u64 = 3;
/// `wait()` — block until a child process exits, returning its exit code (Stage 12).
/// Meaningful only from ring 3; unlike the others it returns its result in `rax` (see
/// [`crate::process::on_user_wait`]).
pub const SYS_WAIT: u64 = 4;
/// `spawn(prog_id)` — create a new child process from a kernel-known program and return
/// its pid (Stage 12d). Meaningful only from ring 3; returns the pid via the stack ABI
/// (like `getpid`), and does not switch processes — the caller keeps running.
pub const SYS_SPAWN: u64 = 5;
/// `socket()` — allocate a TCP socket in the caller's per-process handle table and return
/// its **file descriptor**, a small integer indexing that table (Stage 24a). Meaningful
/// only from ring 3; returns the fd via the stack ABI (like `getpid`), and does not switch
/// processes. The socket is created *unbound*; a later `connect` binds it to a connection.
pub const SYS_SOCKET: u64 = 6;
/// `connect(fd, dst)` — actively open a TCP connection on socket `fd` to `dst` (an IPv4
/// address packed in the high 32 bits, port in the low 16), performing the three-way
/// handshake (Stage 24a). Meaningful only from ring 3. Unlike the calls above this one
/// **blocks**: the process is descheduled until the handshake reaches ESTABLISHED, and —
/// like `wait` — it returns its result in `rax` (the connected fd, or `u64::MAX` on
/// failure), since a blocking syscall resumes from its saved [`TrapFrame`].
pub const SYS_CONNECT: u64 = 7;
/// `send(fd, ptr, len)` — send `len` bytes at `ptr` on the connected socket `fd` (Stage 24b),
/// returning the number of bytes accepted (`u64::MAX` on error). Meaningful only from ring 3;
/// non-blocking (the bytes are queued and flushed), so it returns via the stack ABI like `write`.
/// This is the stack's first **three-argument** syscall, so the caller pushes a third slot the
/// handler reads at `[rsp+24]`.
pub const SYS_SEND: u64 = 8;
/// `recv(fd, ptr, len)` — receive up to `len` bytes into `ptr` on the connected socket `fd`
/// (Stage 24b), returning the number of bytes read (0 on end-of-stream). Meaningful only from
/// ring 3. Like `connect` it **blocks** — until data arrives — so it returns its count in `rax`.
pub const SYS_RECV: u64 = 9;
/// `listen(fd, port)` — turn socket `fd` into a passive **listener** bound to `port`, ready to accept
/// incoming connections (Stage 24c). Meaningful only from ring 3; non-blocking (it just registers the
/// listener), so it returns via the stack ABI like `write` (0 on success, `u64::MAX` on error).
pub const SYS_LISTEN: u64 = 10;
/// `accept(fd)` — take the next established connection from listening socket `fd`'s accept queue,
/// returning a **new** file descriptor bound to it (Stage 24c). Meaningful only from ring 3. Like
/// `connect`/`recv` it **blocks** — until a connection is ready — so it returns the new fd in `rax`
/// (`u64::MAX` on error/timeout). The listening `fd` stays open to accept more.
pub const SYS_ACCEPT: u64 = 11;

/// Count of syscalls that arrived from ring 3 — proof (for the Stage 10b test)
/// that the user program really crossed into the kernel through `int 0x80`.
static RING3_SYSCALLS: AtomicU64 = AtomicU64::new(0);

/// Number of syscalls made from ring 3 since boot.
pub fn ring3_syscall_count() -> u64 {
    RING3_SYSCALLS.load(Ordering::SeqCst)
}

/// Naked entry for `int 0x80` (Stage 12c-2). Registered in `interrupts.rs` with the
/// gate's DPL set to 3 so ring 3 may invoke it.
///
/// Mirrors [`crate::interrupts::timer_interrupt_entry`]: it pushes every
/// general-purpose register to build a [`TrapFrame`] on the kernel stack, hands a
/// pointer to it to [`syscall_dispatch`], then restores the registers and `iretq`s.
/// Capturing the *full* register set — rather than the partial state an
/// `extern "x86-interrupt"` handler exposes — is what lets a `yield`/`exit` save the
/// caller's complete context and restore the next process's, the same fidelity the
/// timer needs for preemption (Stage 12c-3). When `syscall_dispatch` switches
/// processes it overwrites this `TrapFrame` (and CR3), so the `pop`s below restore a
/// *different* context than was saved — that *is* the switch.
#[unsafe(naked)]
pub unsafe extern "C" fn syscall_entry() {
    naked_asm!(
        // Save all GP registers in TrapFrame order (see `timer_interrupt_entry`):
        // pushed highest-numbered first so that, read upward from the final rsp, they
        // are rax, rbx, ... r15. The CPU already pushed the interrupt frame above.
        "push r15",
        "push r14",
        "push r13",
        "push r12",
        "push r11",
        "push r10",
        "push r9",
        "push r8",
        "push rbp",
        "push rdi",
        "push rsi",
        "push rdx",
        "push rcx",
        "push rbx",
        "push rax",
        "mov rdi, rsp", // arg 1: pointer to the TrapFrame we just built
        "call {dispatch}",
        // Restore (possibly a different process's context after a yield/exit switch).
        "pop rax",
        "pop rbx",
        "pop rcx",
        "pop rdx",
        "pop rsi",
        "pop rdi",
        "pop rbp",
        "pop r8",
        "pop r9",
        "pop r10",
        "pop r11",
        "pop r12",
        "pop r13",
        "pop r14",
        "pop r15",
        "iretq",
        dispatch = sym syscall_dispatch,
    );
}

/// Rust side of `int 0x80`. Receives a pointer to the caller's [`TrapFrame`] on the
/// kernel stack, built by [`syscall_entry`].
///
/// It reads the syscall number and two arguments from the caller's stack (our ABI,
/// above), dispatches, and writes the return value back to the number's slot. The
/// gate is an *interrupt* gate, so interrupts are disabled for the duration — the
/// syscall runs to completion without being preempted. A ring 3 `yield`/`exit`
/// instead hands off to the scheduler, which rewrites this `TrapFrame` (and CR3) to
/// resume another process (or, for `exit` with none left, the kernel).
extern "C" fn syscall_dispatch(tf_ptr: *mut TrapFrame) {
    // SAFETY: `tf_ptr` is the TrapFrame `syscall_entry` just built at the kernel
    // stack top; it is valid and uniquely referenced for the duration of this call.
    let tf = unsafe { &mut *tf_ptr };

    // `iframe.stack_pointer` is the caller's RSP at the `int 0x80`; by our ABI [rsp] holds
    // the syscall number and any arguments follow at [rsp+8] (arg1), [rsp+16] (arg2).
    let args = tf.iframe.stack_pointer.as_u64() as *mut u64;

    // Read only the *number* eagerly, and each argument slot lazily in the branch that
    // uses it. A syscall with fewer arguments pushes fewer slots, so [rsp+8]/[rsp+16] then
    // lie above the caller's pushed data — and when the call is the first thing on a fresh
    // user stack (`socket` in the Stage 24a demo), above the mapped stack entirely, so
    // eagerly reading all three would page-fault. (This was latent while every 0-argument
    // syscall — `yield`/`wait` — only ran after the stack had already grown downward.)
    //
    // SAFETY: the caller pushed the number at its stack top immediately before trapping, so
    // this slot exists and is readable. (A hardened kernel would also bounds-check the
    // argument reads below against the caller's own mapped stack.)
    let number = unsafe { args.read() };

    // The low two bits of the saved code selector are the caller's privilege level.
    let from_ring3 = tf.iframe.code_segment & 0b11 == 3;
    if from_ring3 {
        RING3_SYSCALLS.fetch_add(1, Ordering::SeqCst);
    }

    // `yield` and `exit` from ring 3 are control transfers, not value-returning
    // calls: both hand off to the scheduler, which rewrites this `TrapFrame` (and
    // CR3) to resume a different process (or, for `exit` with none left, the kernel).
    // `yield` re-queues the caller; `exit` drops it.
    if number == SYS_YIELD && from_ring3 {
        crate::process::on_user_yield(tf); // no arguments
        return;
    }
    if number == SYS_EXIT && from_ring3 {
        // SAFETY: `exit` pushed its one argument (the code) at [rsp+8].
        let code = unsafe { args.add(1).read() };
        crate::process::on_user_exit(tf, code);
        return;
    }
    if number == SYS_WAIT && from_ring3 {
        // `wait` takes no arguments and returns its result in rax (set by on_user_wait).
        crate::process::on_user_wait(tf);
        return;
    }
    if number == SYS_SPAWN && from_ring3 {
        // `spawn` creates a child and returns its pid via the stack ABI (like the
        // value-returning calls below), but it must reach the scheduler for the caller's
        // id, so it is handled here rather than in `dispatch`. It does *not* switch
        // processes — the caller resumes right after, with the new pid.
        // SAFETY: `spawn` pushed its one argument (the program id) at [rsp+8].
        let prog_id = unsafe { args.add(1).read() };
        let pid = crate::process::on_user_spawn(prog_id);
        // SAFETY: the number slot at [rsp] is writable — the caller pops the return value.
        unsafe { args.write(pid) };
        return;
    }
    if number == SYS_SOCKET && from_ring3 {
        // `socket` allocates a handle in the *caller's* per-process table, so — like
        // `spawn` — it must reach the scheduler for the current process. It takes no
        // arguments (so no slot above [rsp] is read — the fix for the first-syscall fault
        // above), returns the fd via the stack ABI, and does not switch processes.
        let fd = crate::process::on_user_socket();
        // SAFETY: the number slot is writable, as above.
        unsafe { args.write(fd) };
        return;
    }
    if number == SYS_CONNECT && from_ring3 {
        // `connect` is a *blocking* control transfer: it deschedules the caller until the
        // handshake settles, then rewrites this `TrapFrame` (rax = result) to resume it —
        // exactly like `wait`. It returns in rax, not the stack.
        // SAFETY: `connect` pushed both arguments (fd, dst) at [rsp+8], [rsp+16].
        let fd = unsafe { args.add(1).read() };
        let dst = unsafe { args.add(2).read() };
        crate::process::on_user_connect(tf, fd, dst);
        return;
    }
    if number == SYS_SEND && from_ring3 {
        // `send` is non-blocking: queue the bytes and flush. It returns the count via the stack
        // ABI (like `write`), and does not switch processes.
        // SAFETY: `send` pushed all three arguments (fd, ptr, len) at [rsp+8], [rsp+16], [rsp+24].
        let fd = unsafe { args.add(1).read() };
        let ptr = unsafe { args.add(2).read() };
        let len = unsafe { args.add(3).read() };
        let n = crate::process::on_user_send(fd, ptr, len);
        // SAFETY: the number slot is writable, as above.
        unsafe { args.write(n) };
        return;
    }
    if number == SYS_RECV && from_ring3 {
        // `recv` *blocks* until data arrives, then rewrites this `TrapFrame` (rax = count) to
        // resume the caller — like `connect`/`wait`. It returns in rax, not the stack.
        // SAFETY: `recv` pushed all three arguments (fd, ptr, len) at [rsp+8], [rsp+16], [rsp+24].
        let fd = unsafe { args.add(1).read() };
        let ptr = unsafe { args.add(2).read() };
        let len = unsafe { args.add(3).read() };
        crate::process::on_user_recv(tf, fd, ptr, len);
        return;
    }
    if number == SYS_LISTEN && from_ring3 {
        // `listen` binds the socket to a port and registers a passive listener. Non-blocking, returns via
        // the stack ABI (like `send`), and does not switch processes.
        // SAFETY: `listen` pushed both arguments (fd, port) at [rsp+8], [rsp+16].
        let fd = unsafe { args.add(1).read() };
        let port = unsafe { args.add(2).read() };
        let result = crate::process::on_user_listen(fd, port);
        // SAFETY: the number slot is writable, as above.
        unsafe { args.write(result) };
        return;
    }
    if number == SYS_ACCEPT && from_ring3 {
        // `accept` *blocks* until a connection is ready, then rewrites this `TrapFrame` (rax = new fd) to
        // resume the caller — like `connect`/`recv`. It returns in rax, not the stack.
        // SAFETY: `accept` pushed its one argument (fd) at [rsp+8].
        let fd = unsafe { args.add(1).read() };
        crate::process::on_user_accept(tf, fd);
        return;
    }

    // The remaining calls (`write`/`getpid`/`exit` via `dispatch`) take up to two arguments
    // and return via the stack. They are reached only from ring 0 (`invoke`, which always
    // pushes both argument slots) or from ring 3 `write` (which also pushes both), so both
    // slots are present here.
    // SAFETY: those callers pushed both argument slots at [rsp+8], [rsp+16].
    let arg1 = unsafe { args.add(1).read() };
    let arg2 = unsafe { args.add(2).read() };
    let result = dispatch(number, arg1, arg2);

    // SAFETY: the number slot is writable; the caller pops the return value from it.
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
