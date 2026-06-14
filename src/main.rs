//! Aether — a from-scratch, iteratively-built educational x86_64 OS kernel.
//!
//! "Aether" comes from ancient Greek, once imagined as the fundamental medium
//! filling the universe and carrying all things — much like a kernel underlies
//! everything that runs on top of it.
//!
//! Current stage (Stage 3): on top of the Stage 0 serial output, the Stage 1
//! VGA text buffer, and the Stage 2 breakpoint exception, the kernel now loads
//! a GDT and TSS (so the double fault handler runs on a dedicated stack — the
//! safety net against a stack overflow triple faulting the machine) and brings
//! up the 8259 PIC to handle hardware interrupts: the periodic timer (IRQ0)
//! and the keyboard (IRQ1), whose keystrokes it echoes to the screen.
//! This is already a true "bare metal" program — it runs on no underlying
//! operating system and takes over the CPU.
//!
//! See ROADMAP.md for what comes next.

// Don't link the standard library: on bare metal there is no OS to provide the
// syscalls that `std` depends on. We can only use `core` (the language core
// library, which needs no OS support).
#![no_std]
// Don't use Rust's default entry point (it relies on the C runtime crt0).
// We define our own entry point instead.
#![no_main]
// Exception/interrupt handlers use the special "x86-interrupt" calling
// convention, which is still unstable, so we opt in to it here.
#![feature(abi_x86_interrupt)]

mod serial;
mod vga_buffer;
mod gdt;
mod interrupts;

use core::panic::PanicInfo;

/// Kernel entry point.
///
/// After the `bootloader` crate finishes the real-mode -> long-mode switch, it
/// jumps to a function named `_start`. Therefore:
/// - `#[no_mangle]`: disable name mangling so the symbol is exactly `_start`.
/// - `extern "C"`: use the C calling convention.
/// - returns `!`: the kernel entry never returns (there is no caller to return to).
#[no_mangle]
pub extern "C" fn _start() -> ! {
    serial::init();
    serial_println!("[ OK ] serial port initialized");

    // Stage 1: the VGA text buffer. Unlike the serial lines above, these go to
    // the *screen* (the QEMU window). We start from a clean screen, then print a
    // banner and a formatted expression to prove both the driver and the `{}`
    // formatting machinery work.
    vga_buffer::WRITER.lock().clear_screen();
    println!("========================================");
    println!("        Hello from Aether kernel!");
    println!("     VGA text buffer is now working.");
    println!("========================================");
    println!();
    println!("Formatting works too: {} + {} = {}", 19, 23, 19 + 23);

    serial_println!("[ OK ] VGA text buffer initialized");

    // Stage 3 (step 1): load the GDT and TSS first. This installs a dedicated,
    // known-good stack (via the IST) for the double fault handler. It must run
    // before `init_idt`, because the IDT's double fault entry references the IST
    // slot defined here.
    gdt::init();
    serial_println!("[ OK ] GDT and TSS loaded");

    // Stage 2: load the IDT, then deliberately raise a breakpoint exception with
    // `int3`. The CPU dispatches to our handler, which prints and returns; since
    // #BP is a trap, execution resumes right after `int3` — so reaching the line
    // below proves the kernel took an exception and kept running.
    interrupts::init_idt();
    serial_println!("[ OK ] IDT loaded");
    x86_64::instructions::interrupts::int3();
    serial_println!("[ OK ] survived breakpoint, kernel continues");

    // To SEE the double fault safety net in action, uncomment the next line.
    // `stack_overflow` recurses forever and overflows the kernel stack. Without
    // the IST stack installed by `gdt::init`, the page fault that follows would
    // itself fault (no stack left to dispatch it), escalate to a double fault,
    // fail again, and *triple* fault — QEMU would reboot endlessly. With the
    // IST, the double fault handler runs on its own stack and prints
    // "DOUBLE FAULT" instead. Re-comment it before committing so boot continues.
    // stack_overflow();

    // Stage 3 (step 2): bring up the 8259 PICs and enable hardware interrupts.
    // From here the timer (IRQ0) fires on its own several times a second; its
    // handler logs a tick to the serial port. This is the first time the CPU
    // runs our code because an external device asked it to, not because we did.
    interrupts::init_pics();
    serial_println!("[ OK ] PIC initialized");
    x86_64::instructions::interrupts::enable();
    serial_println!("[ OK ] hardware interrupts enabled; timer is now ticking");

    println!();
    println!("Keyboard is live - type and your keystrokes will echo below:");

    serial_println!("Kernel entering idle loop. Press Ctrl-A then X to exit QEMU.");

    hlt_loop();
}

/// Handler invoked when the kernel panics. On bare metal we must define this
/// ourselves, otherwise the code won't compile.
#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    serial_println!();
    serial_println!("[PANIC] kernel panicked: {}", info);
    hlt_loop();
}

/// Repeatedly execute the `hlt` instruction to put the CPU into a low-power wait
/// until the next interrupt arrives. Far more efficient than a busy `loop {}`
/// (and it won't peg the host CPU under QEMU).
pub fn hlt_loop() -> ! {
    loop {
        x86_64::instructions::hlt();
    }
}

/// Deliberately overflow the kernel stack by recursing without end, to prove the
/// double fault handler (running on its dedicated IST stack) catches what would
/// otherwise be a triple fault. Not called during normal boot — uncomment the
/// call in `_start` to try it.
#[allow(unconditional_recursion, dead_code)]
fn stack_overflow() {
    stack_overflow();
    // Touch the stack *after* the recursive call so the compiler can't turn this
    // into a tail call (which would loop in place without growing the stack).
    // `black_box` is an optimization barrier that forces the frame to persist.
    core::hint::black_box(());
}
