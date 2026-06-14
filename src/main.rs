//! Aether — a from-scratch, iteratively-built educational x86_64 OS kernel.
//!
//! "Aether" comes from ancient Greek, once imagined as the fundamental medium
//! filling the universe and carrying all things — much like a kernel underlies
//! everything that runs on top of it.
//!
//! Current stage (Stage 0): the kernel boots and prints output to the terminal
//! over the serial port. This is already a true "bare metal" program — it runs
//! on no underlying operating system and takes over the CPU itself.
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

    serial_println!("========================================");
    serial_println!("       Hello from Aether kernel!        ");
    serial_println!("     Running on bare metal x86_64       ");
    serial_println!("========================================");
    serial_println!();
    serial_println!("[ OK ] serial port initialized");
    serial_println!("[ OK ] kernel booted successfully");
    serial_println!();
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
