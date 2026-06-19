//! The architecture-specific heart of Stage 6: the context switch.
//!
//! A context switch saves the registers of the thread that is giving up the CPU
//! and restores the registers of the thread that is taking over, so that the
//! incoming thread resumes exactly where it last stopped. On x86_64 this must be
//! written in assembly: it manipulates `rsp` and the callee-saved registers
//! directly, which Rust gives us no way to express.
//!
//! The key insight that keeps this tiny: [`context_switch`] is reached by an
//! ordinary `call`. By the System V x86_64 calling convention, the *caller* has
//! already preserved any caller-saved ("scratch") register it still needs, and
//! the `call` already pushed the return address. So a voluntary switch only has
//! to save the six **callee-saved** registers (`rbp`, `rbx`, `r12`-`r15`) plus
//! the stack pointer. (Stage 6b, switching from inside the timer interrupt, will
//! need to save the *full* register set, because an interrupt can strike between
//! any two instructions and the interrupted thread never agreed to give up its
//! scratch registers.)

use core::arch::naked_asm;

/// Switch from the current thread to another one.
///
/// Saves the current thread's callee-saved registers onto its own stack, records
/// the resulting stack pointer in `*old_rsp`, then loads `new_rsp` as the stack
/// pointer and restores the next thread's callee-saved registers — ending with a
/// `ret` that resumes it wherever it last called this function (or, for a freshly
/// spawned thread, the entry trampoline fabricated by `prepare_stack`).
///
/// Arguments follow the System V ABI: `old_rsp` arrives in `rdi`, `new_rsp` in
/// `rsi`.
///
/// # Safety
///
/// - `old_rsp` must point to writable storage for one `u64` (the outgoing
///   thread's saved-stack-pointer slot).
/// - `new_rsp` must be a stack pointer previously produced either by this
///   function (a thread that was switched out earlier) or by `prepare_stack` (a
///   brand-new thread). Any other value makes the `ret` jump to garbage and
///   triple-faults the machine.
/// - The save/restore register order here must stay in lockstep with the frame
///   `prepare_stack` fabricates; changing one without the other corrupts every
///   new thread.
#[unsafe(naked)]
pub unsafe extern "C" fn context_switch(old_rsp: *mut u64, new_rsp: u64) {
    naked_asm!(
        // --- save the outgoing thread ---
        // Push the six callee-saved registers onto the current (outgoing) stack.
        "push rbp",
        "push rbx",
        "push r12",
        "push r13",
        "push r14",
        "push r15",
        // Record the outgoing stack pointer in *old_rsp (rdi holds old_rsp).
        "mov [rdi], rsp",
        // --- switch stacks ---
        // Load the incoming thread's stack pointer (rsi holds new_rsp).
        "mov rsp, rsi",
        // --- restore the incoming thread ---
        // Pop its six callee-saved registers back (reverse of the push order).
        "pop r15",
        "pop r14",
        "pop r13",
        "pop r12",
        "pop rbx",
        "pop rbp",
        // Resume it: `ret` pops the saved return address and jumps there.
        "ret",
    );
}
