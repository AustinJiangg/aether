//! Aether — a from-scratch, iteratively-built educational x86_64 OS kernel.
//!
//! "Aether" comes from ancient Greek, once imagined as the fundamental medium
//! filling the universe and carrying all things — much like a kernel underlies
//! everything that runs on top of it.
//!
//! Current stage (Stage 1): on top of the Stage 0 serial output, the kernel now
//! drives the VGA text buffer, printing characters directly to the screen via
//! memory-mapped I/O. This is already a true "bare metal" program — it runs on
//! no underlying operating system and takes over the CPU itself.
//!
//! See ROADMAP.md for what comes next.

// Don't link the standard library: on bare metal there is no OS to provide the
// syscalls that `std` depends on. We can only use `core` (the language core
// library, which needs no OS support).
#![no_std]
// Don't use Rust's default entry point (it relies on the C runtime crt0).
// We define our own entry point instead.
#![no_main]

mod serial;
mod vga_buffer;

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
fn hlt_loop() -> ! {
    loop {
        x86_64::instructions::hlt();
    }
}
