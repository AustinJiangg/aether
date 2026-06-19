//! Aether — a from-scratch, iteratively-built educational x86_64 OS kernel.
//!
//! "Aether" comes from ancient Greek, once imagined as the fundamental medium
//! filling the universe and carrying all things — much like a kernel underlies
//! everything that runs on top of it.
//!
//! Current stage (Stage 4c): on top of Stages 0–3 (serial output, the VGA text
//! buffer, the IDT with the breakpoint and double-fault handlers, and hardware
//! interrupts via the 8259 PIC — timer and keyboard), the kernel now manages
//! virtual memory and has a working heap. The bootloader maps all of physical
//! memory for us, so we read CR3 and build an `OffsetPageTable` to translate
//! addresses (4a) and create new mappings via a frame allocator (4b). On top of
//! that we map a heap region and register a `#[global_allocator]` (a hand-written
//! fixed-size block allocator over a linked-list fallback), so the `alloc` crate's
//! `Box`/`Vec`/`Rc` now work (4c), which completes Stage 4.
//! This is already a true "bare metal" program — it runs on no underlying
//! operating system and takes over the CPU.
//!
//! Stage 5 added cooperative multitasking with `async`/`await`. A `Task` wraps a
//! pinned, heap-allocated future. A waker-driven `Executor` polls a task only when
//! it has been woken (each task carries a unique `TaskId`). The async keyboard's
//! interrupt handler only enqueues raw scancodes and wakes the task that decodes
//! and echoes them. When no task is ready, the executor halts the CPU until the
//! next interrupt, so an idle kernel uses no CPU.
//!
//! Stage 6 added independent kernel threads with a hand-written context switch,
//! driven cooperatively (6a) and then preemptively from the timer (6b).
//!
//! Stage 7 (this stage) adds a tiny interactive shell. It revives the Stage 5
//! async executor: the shell is an async task that `.await`s decoded keystrokes
//! from the keyboard `ScancodeStream`, buffers a line, and on Enter dispatches it
//! to a built-in command (help, echo, clear, ticks, uptime, mem). A boot-time
//! self-test runs a few canned commands so the shell is verifiable even without a
//! keyboard. There is no user mode yet, so the shell runs in kernel space and its
//! commands are direct kernel calls — a precursor to real system calls. (The
//! Stage 6 thread scheduler is dormant while this stage runs the executor.)
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

extern crate alloc;

mod serial;
mod vga_buffer;
mod gdt;
mod interrupts;
mod memory;
mod allocator;
mod task;
// The Stage 6 thread scheduler is dormant during Stage 7: the kernel runs the
// async executor (the shell) instead, so the scheduler's spawn/run/yield sit
// unused. (Its `schedule` is still called from the timer handler, but no-ops
// because preemption is never armed.) Silence the dead-code warnings for the
// subtree until a later stage folds async tasks and threads together.
#[allow(dead_code)]
mod thread;
mod shell;

use core::panic::PanicInfo;

use alloc::boxed::Box;
use alloc::rc::Rc;
use alloc::vec::Vec;
use bootloader::{entry_point, BootInfo};
use x86_64::structures::paging::{FrameAllocator, Page, Translate};
use x86_64::VirtAddr;

use task::executor::Executor;
use task::Task;

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
    let mut mapper = unsafe { memory::init(phys_mem_offset) };

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

    // Stage 4b: a frame allocator and the first hand-made mapping. Translation
    // (above) only reads the tables; to *create* a mapping we need free physical
    // frames for any missing intermediate tables. `BootInfoFrameAllocator` draws
    // those from the regions the bootloader marked usable in the memory map.
    // SAFETY: the bootloader's memory map is valid and the regions it marks
    // `Usable` are genuinely free, so frames handed out are not aliased.
    let mut frame_allocator =
        unsafe { memory::BootInfoFrameAllocator::init(&boot_info.memory_map) };

    // Prove the allocator yields a usable frame before we map anything.
    if let Some(frame) = frame_allocator.allocate_frame() {
        serial_println!("[paging] frame allocator handed out {:?}", frame);
    }

    // Map a brand-new page at 64 TiB (nothing is mapped near there) onto the VGA
    // frame. Because that region has no page tables yet, `map_to` must build the
    // L3/L2/L1 tables on the way down, drawing those frames from our allocator —
    // so a successful mapping exercises the allocator end to end.
    let page = Page::containing_address(VirtAddr::new(0x_4000_0000_0000));
    memory::create_example_mapping(page, &mut mapper, &mut frame_allocator);
    serial_println!(
        "[paging] mapped {:?} -> {:?}",
        page.start_address(),
        mapper.translate_addr(page.start_address())
    );

    // Write through the NEW page. It aliases the VGA frame, so this lands on the
    // screen: "New!" at row 20 (byte offset 3200 = 400 u64 units into the buffer).
    let page_ptr: *mut u64 = page.start_address().as_mut_ptr();
    // SAFETY: `page` was just mapped writable to the VGA frame, so this offset
    // stays inside that 4 KiB page and writes to VGA device memory; the volatile
    // write keeps the compiler from optimizing the memory-mapped store away.
    unsafe {
        page_ptr.offset(400).write_volatile(0x_f021_f077_f065_f04e);
    }
    serial_println!("[ OK ] frame allocator works; wrote \"New!\" via the new mapping");
    println!("Frame allocator live; mapped a fresh page onto VGA (look for \"New!\").");

    // Stage 4c: stand up a kernel heap. `init_heap` maps a fixed virtual range to
    // freshly-allocated frames (the same `map_to` + frame allocator as 4b), then
    // arms the `#[global_allocator]` over that range. Once it returns, the `alloc`
    // crate's types work.
    allocator::init_heap(&mut mapper, &mut frame_allocator).expect("heap initialization failed");
    serial_println!(
        "[ OK ] heap mapped at {:#x}, size {} KiB",
        allocator::HEAP_START,
        allocator::HEAP_SIZE / 1024
    );

    // A long-lived allocation alongside thousands of short-lived ones. The
    // fixed-size block allocator serves each small box from a per-size free list
    // in O(1) and recycles freed blocks back onto it, so this runs fast and in
    // bounded memory. (The original bump allocator could not reclaim at all — its
    // cursor would march off the end of the 100 KiB heap here.)
    let long_lived = Box::new(1);
    for i in 0..10_000 {
        let x = Box::new(i);
        assert_eq!(*x, i);
    }
    assert_eq!(*long_lived, 1);
    serial_println!("[heap] 10000 boxes + 1 long-lived OK (block allocator recycles freed blocks)");

    // The basic alloc types still work as before.
    let heap_value = Box::new(41);
    serial_println!("[heap] Box holds {} at {:p}", *heap_value, heap_value);

    let mut vec = Vec::new();
    for i in 0..500 {
        vec.push(i);
    }
    serial_println!("[heap] Vec has {} elements, last is {}", vec.len(), vec[vec.len() - 1]);

    let rc = Rc::new(alloc::vec![1, 2, 3]);
    let rc_clone = Rc::clone(&rc);
    serial_println!("[heap] Rc strong_count after clone = {}", Rc::strong_count(&rc));
    core::mem::drop(rc);
    serial_println!("[heap] Rc strong_count after drop  = {}", Rc::strong_count(&rc_clone));
    serial_println!("[ OK ] heap works; Box / Vec / Rc are usable");
    println!("Heap is live; Box / Vec / Rc all work (details on the serial log).");

    // Stage 7: an interactive shell on the revived async executor. First a boot
    // self-test runs a few canned commands through the shell's dispatcher, so the
    // command logic is verifiable even without a keyboard (e.g. headless QEMU).
    // Then we hand the CPU to the executor running the shell task, which reads
    // keystrokes, buffers a line, and dispatches it on Enter. `Executor::run`
    // never returns (`-> !`), so it is the kernel's final call.
    println!();
    serial_println!("Kernel starting the interactive shell on the async executor.");
    shell::selftest();

    let mut executor = Executor::new();
    executor.spawn(Task::new(shell::run()));
    executor.run();
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

// Stage 7's interactive shell lives in `src/shell.rs`; its demo threads from
// Stage 6 are gone now that the kernel runs the async executor instead.
