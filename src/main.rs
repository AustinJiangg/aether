//! Aether — a from-scratch, iteratively-built educational x86_64 OS kernel.
//!
//! "Aether" comes from ancient Greek, once imagined as the fundamental medium
//! filling the universe and carrying all things — much like a kernel underlies
//! everything that runs on top of it.
//!
//! Current stage (Stage 4a): on top of Stages 0–3 (serial output, the VGA text
//! buffer, the IDT with the breakpoint and double-fault handlers, and hardware
//! interrupts via the 8259 PIC — timer and keyboard), the kernel now reaches the
//! page tables. The bootloader maps all of physical memory for us, so we can read
//! CR3, build an `OffsetPageTable` over the active tables, and translate virtual
//! addresses to the physical frames they map to. This is the foundation for the
//! heap allocator that makes `Box`/`Vec` usable in later sub-stages.
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
mod memory;

use core::panic::PanicInfo;

use bootloader::{entry_point, BootInfo};
use x86_64::structures::paging::Translate;
use x86_64::VirtAddr;

// Register `kernel_main` as the kernel entry point.
//
// The `bootloader` crate finishes the real-mode -> long-mode switch and then
// jumps to a symbol named `_start`. Rather than hand-write that symbol (as we
// did before with `#[no_mangle] pub extern "C" fn _start`), the `entry_point!`
// macro generates it for us with the correct ABI *and* type-checks that our
// function has the exact signature the bootloader calls: it passes a
// `&'static BootInfo`. Defining `_start` by hand gave us no such check, and no
// access to that argument. (Plain `//` comments here: `///` docs can't attach to
// a macro invocation.)
entry_point!(kernel_main);

/// Kernel entry point. Never returns (`!`): there is no caller to return to.
///
/// `boot_info` is assembled by the bootloader and describes the machine we woke
/// up on. We use two of its fields: `memory_map` (which physical regions are
/// usable RAM — needed once we start allocating frames) and
/// `physical_memory_offset`, the virtual address at which the bootloader mapped
/// *all* of physical memory for us (because we enabled the `map_physical_memory`
/// feature). That mapping is what makes the page tables — which hold physical
/// addresses — reachable.
fn kernel_main(boot_info: &'static BootInfo) -> ! {
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

    // Stage 4a: paging. With paging on, every address the kernel uses is a
    // *virtual* address that the CPU translates to a physical one by walking a
    // 4-level page table in hardware. Here we learn to walk that same table in
    // software: `memory::init` builds an `OffsetPageTable` over the active table,
    // and `Translate::translate_addr` resolves a virtual address to the physical
    // frame it maps to (or `None` if nothing is mapped there). This is purely
    // read-only — installing new mappings needs a frame allocator (Stage 4b).
    let phys_mem_offset = VirtAddr::new(boot_info.physical_memory_offset);
    // SAFETY: the bootloader mapped all physical memory at `phys_mem_offset` (the
    // `map_physical_memory` feature is enabled in Cargo.toml), and we call `init`
    // exactly once here — the two invariants `memory::init` documents.
    let mapper = unsafe { memory::init(phys_mem_offset) };

    // Translate four real, known addresses to prove the page-table walk works:
    // the VGA frame, a spot on the current kernel stack, the boot info struct,
    // and the base of the physical-memory mapping (which must resolve to physical
    // address 0, since that virtual base is exactly where physical 0 was mapped).
    let stack_probe = 0u64;
    let addresses = [
        0xb8000,                              // VGA text buffer (memory-mapped I/O)
        &stack_probe as *const u64 as u64,    // somewhere on the current kernel stack
        boot_info as *const BootInfo as u64,  // where the boot info struct lives
        boot_info.physical_memory_offset,     // base of the physical-memory mapping
    ];
    serial_println!("[paging] virtual -> physical translations:");
    for &address in &addresses {
        let virt = VirtAddr::new(address);
        let phys = mapper.translate_addr(virt);
        serial_println!("    {:?} -> {:?}", virt, phys);
    }
    serial_println!("[ OK ] paging initialized; page-table walk works");
    println!("Paging is live (virtual->physical translations on the serial log).");

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
