//! CPU exception handling via the Interrupt Descriptor Table (IDT).
//!
//! When the CPU hits an exception (a divide error, a page fault, a breakpoint,
//! ...) it looks up a 256-entry table called the IDT: entry N holds the address
//! of the handler for vector N. The `lidt` instruction tells the CPU where that
//! table lives. We build one table, register a handler for the breakpoint
//! exception, and load it.
//!
//! For now we only handle the breakpoint exception (#BP, vector 3). It is a
//! "trap": after the handler returns, the CPU continues at the instruction right
//! after the one that triggered it, so the kernel keeps running. Later stages
//! add more handlers (e.g. double fault, page fault) and hardware interrupts.

use spin::Lazy;
use x86_64::structures::idt::{InterruptDescriptorTable, InterruptStackFrame};

use crate::{println, serial_println};

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
