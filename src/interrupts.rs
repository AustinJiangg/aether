//! CPU exception handling via the Interrupt Descriptor Table (IDT).
//!
//! When the CPU hits an exception (a divide error, a page fault, a breakpoint,
//! ...) it looks up a 256-entry table called the IDT: entry N holds the address
//! of the handler for vector N. The `lidt` instruction tells the CPU where that
//! table lives. We build one table, register a handler for the breakpoint
//! exception, and load it.
//!
//! We handle two exceptions so far:
//! - The breakpoint exception (#BP, vector 3): a "trap". After the handler
//!   returns, the CPU continues at the instruction right after the one that
//!   triggered it, so the kernel keeps running.
//! - The double fault (#DF, vector 8): the safety net raised when dispatching
//!   another exception fails (e.g. on stack overflow). It cannot return, so its
//!   handler diverges. We register it to run on a dedicated stack via the IST
//!   slot set up in `gdt.rs`, so it works even when the kernel stack is gone.
//!
//! Later stages add more handlers (e.g. page fault) and hardware interrupts.

use spin::Lazy;
use x86_64::structures::idt::{InterruptDescriptorTable, InterruptStackFrame};

use crate::{gdt, hlt_loop, println, serial_println};

/// The kernel's interrupt descriptor table.
///
/// It must live for `'static`: `load()` hands the CPU a pointer to this table,
/// and the CPU keeps using it long after `init_idt` returns, so the table can
/// never be dropped. `Lazy` builds it on first access because
/// `InterruptDescriptorTable::new()` is not a `const fn` (so a plain `static`
/// initializer won't work).
static IDT: Lazy<InterruptDescriptorTable> = Lazy::new(|| {
    let mut idt = InterruptDescriptorTable::new();
    idt.breakpoint.set_handler_fn(breakpoint_handler);
    // SAFETY: `DOUBLE_FAULT_IST_INDEX` is a valid IST slot (0..=6), and `gdt.rs`
    // initializes that slot with a real, dedicated stack before we ever take an
    // exception. Pointing the double fault at it is what lets the handler run on
    // a known-good stack even when the kernel stack has overflowed.
    unsafe {
        idt.double_fault
            .set_handler_fn(double_fault_handler)
            .set_stack_index(gdt::DOUBLE_FAULT_IST_INDEX);
    }
    idt
});

/// Load the IDT into the CPU with `lidt`. Call once, early in boot.
pub fn init_idt() {
    IDT.load();
}

/// Handler for the breakpoint exception (#BP), raised by the `int3` instruction.
///
/// `extern "x86-interrupt"` makes the compiler emit the special prologue/epilogue
/// an exception handler needs: it preserves every register the handler touches
/// and returns with `iretq` instead of a normal `ret`. The CPU pushes an
/// `InterruptStackFrame` (the saved instruction pointer, code segment, flags,
/// stack pointer and stack segment) which we receive as the argument.
extern "x86-interrupt" fn breakpoint_handler(stack_frame: InterruptStackFrame) {
    // Full detail goes to the serial log; a short note goes to the screen.
    serial_println!("[EXCEPTION] breakpoint\n{:#?}", stack_frame);
    println!("[EXCEPTION] breakpoint handled; kernel continues");
}

/// Handler for the double fault (#DF, vector 8).
///
/// A double fault occurs when the CPU fails to invoke an exception handler — for
/// example a page fault whose own handler can't be dispatched, or, the case we
/// guard against here, a stack overflow that leaves no room to push the
/// exception frame. Unlike #BP this is *not* recoverable: the architecture
/// forbids returning from it (note the `-> !`), and its error code is always 0.
/// Reaching this handler at all means we successfully switched to the dedicated
/// IST stack — without that, this fault would escalate to a triple fault and
/// reset the machine. We log as much as we can and halt instead of rebooting.
extern "x86-interrupt" fn double_fault_handler(
    stack_frame: InterruptStackFrame,
    _error_code: u64,
) -> ! {
    serial_println!("[EXCEPTION] DOUBLE FAULT\n{:#?}", stack_frame);
    println!("[EXCEPTION] DOUBLE FAULT - halting");
    hlt_loop();
}
