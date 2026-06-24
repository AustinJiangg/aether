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
//! Stage 7 added a tiny interactive shell on the revived Stage 5 async executor:
//! an async task that `.await`s decoded keystrokes from the keyboard
//! `ScancodeStream`, buffers a line, and on Enter dispatches it to a built-in
//! command. Stage 8 (this stage) adds an in-memory file system (`fs`): a tree of
//! files and directories on the heap, with shell commands `ls`, `cat`, `write`,
//! `mkdir`, `rm`, `cd`, and `pwd`. A boot self-test exercises the commands and the
//! file system, so both are verifiable without a keyboard. There is no user mode
//! yet, so the shell runs in kernel space and its commands are direct kernel calls
//! — a precursor to real system calls. (The Stage 6 thread scheduler is dormant
//! while this stage runs the executor.)
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
// `cargo test` for a `#![no_std]` binary can't use the standard library's test
// harness (it needs an OS). The `custom_test_frameworks` feature lets us supply
// our own: the compiler gathers every `#[test_case]` and passes them to the
// runner named below. We reexport the generated harness entry point as
// `test_main` so `kernel_main` can invoke it after boot. See `src/testing.rs`.
#![feature(custom_test_frameworks)]
#![test_runner(crate::testing::test_runner)]
#![reexport_test_harness_main = "test_main"]
// In a `cargo test` build the interactive layer (shell + async executor) is
// excluded via `#[cfg(not(test))]` in `kernel_main`, so every function reachable
// only from it reads as "never used". Allow dead code in test builds to keep
// `cargo test` output clean; the normal `cargo build` keeps full dead-code
// detection, so genuinely-unused code still surfaces there.
#![cfg_attr(test, allow(dead_code))]

extern crate alloc;

mod serial;
mod vga_buffer;
mod gdt;
mod interrupts;
mod memory;
mod allocator;
mod ata;
mod task;
// The Stage 6 thread scheduler is dormant during Stage 7: the kernel runs the
// async executor (the shell) instead, so the scheduler's spawn/run/yield sit
// unused. (Its `schedule` is still called from the timer handler, but no-ops
// because preemption is never armed.) Silence the dead-code warnings for the
// subtree until a later stage folds async tasks and threads together.
#[allow(dead_code)]
mod thread;
mod fs;
mod shell;
mod syscall;
mod usermode;
mod elf;
mod process;
// Test-only: the in-QEMU unit-test harness (runner, QEMU exit, `#[test_case]`s).
// Compiled solely for `cargo test`, never into the real kernel image.
#[cfg(test)]
mod testing;

use core::panic::PanicInfo;

use alloc::boxed::Box;
use alloc::rc::Rc;
use alloc::vec::Vec;
use bootloader::{entry_point, BootInfo};
use x86_64::structures::paging::{FrameAllocator, Page, Translate};
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

    // Stage 13a: read a raw sector from disk via ATA PIO (polling, no DMA/IRQ). The
    // bootimage is attached as the primary IDE master, so sector 0 is the boot sector —
    // its last two bytes are the MBR signature 0x55 0xAA, a stable thing to verify without
    // assuming any file-system layout. This is the first taste of real persistence.
    {
        // Heap buffer, not a stack array — see read_sector's note on the small boot stack.
        let mut sector = alloc::vec![0u8; ata::SECTOR_SIZE];
        match ata::read_sector(0, &mut sector) {
            Ok(()) => {
                let sig_ok = sector[510] == 0x55 && sector[511] == 0xAA;
                serial_println!(
                    "[ata] read sector 0 ({} bytes); MBR signature {:#04x} {:#04x} (valid = {})",
                    ata::SECTOR_SIZE,
                    sector[510],
                    sector[511],
                    sig_ok,
                );
                println!(
                    "Disk read works (ATA PIO): sector 0 MBR signature valid = {}",
                    sig_ok
                );
            }
            Err(e) => {
                serial_println!("[ata] read of sector 0 failed: {:?}", e);
                println!("Disk read (ATA PIO) FAILED: {:?}", e);
            }
        }
    }

    // Stage 11a: process address spaces. To the hardware, a process *is* its own
    // top-level page table — its own value in CR3. Before giving each user program
    // a private space, we prove the core move in isolation: build a second address
    // space that clones the kernel's mappings, switch the CPU onto it, run real
    // kernel work there, and switch back. Surviving the round-trip shows we can hand
    // the CPU a fresh CR3 without the kernel vanishing underneath it — the
    // foundation the ELF loader (11b) and per-process scheduling (12) build on.
    memory::demo_clone_kernel_space(&mut frame_allocator, phys_mem_offset);
    serial_println!(
        "[ OK ] address-space clone + CR3 round-trip verified = {}",
        memory::address_space_clone_ok()
    );
    println!("Address spaces live; cloned the kernel space and switched CR3 (serial log).");

    // Stage 11b: load a real ELF64 program into its own address space. The loader
    // parses the ELF, then maps each PT_LOAD segment into a fresh space (cloned
    // from the kernel in 11a) and copies the bytes in through the physical-memory
    // window — because the new space is not active yet, the user addresses are only
    // reachable that way. We verify by translating the entry point in the new space
    // and reading the code back; switching to the space and running it in ring 3 is
    // the next step.
    // Stage 11b: load two user programs, each into its own private address space.
    // They are byte-identical except for the message string — yet each reads its own
    // message from the *same* virtual address, because the address spaces are
    // separate (the whole point of per-process paging).
    let img1 = process::demo_load_elf(
        b"hello from user process #1\n",
        &mut frame_allocator,
        phys_mem_offset,
    );
    let img2 = process::demo_load_elf(
        b"hello from user process #2\n",
        &mut frame_allocator,
        phys_mem_offset,
    );
    serial_println!(
        "[ OK ] two ELF programs loaded into private address spaces, verified = {}",
        process::elf_load_ok()
    );
    println!("ELF loader live; loaded two programs into separate address spaces (serial log).");

    // Stage 12c: spawn the two interleaving workers and start the scheduler. Each runs
    // several rounds of `write` + busy-spin + `yield`; the timer preempts them mid-spin
    // and they also `yield`, so their output interleaves (#1, #2, #1, #2, ...).
    let p1 = process::spawn(img1, None);
    let p2 = process::spawn(img2, None);
    // Stage 12/12d: also spawn a parent that blocks in `wait()`. Unlike before, the kernel
    // spawns *only* the parent; the parent itself creates its child at runtime via the
    // `spawn` syscall, then collects its exit code on wakeup. All run together.
    let parent = process::spawn_wait_demo(&mut frame_allocator, phys_mem_offset);
    serial_println!(
        "[sched] spawned workers {} and {}, wait-demo parent {} (spawns its own child)",
        p1, p2, parent
    );
    // Stage 12d: hand the frame allocator + physical-memory offset to the kernel globals
    // so the `spawn` syscall can load an ELF at runtime from inside the trap handler
    // (which cannot borrow these locals). This *moves* `frame_allocator`; nothing below
    // uses it again. Must happen before any user process runs.
    memory::install_kernel_allocator(frame_allocator, phys_mem_offset);
    // When the last process exits, the kernel resumes at `boot_continue` (which switches
    // CR3 back to the kernel space). `run` never returns here.
    process::run(boot_continue);
}

/// Continue (and finish) boot after the ring 3 excursion (Stage 12a runs a loaded
/// ELF program there).
///
/// Reached via [`usermode::resume_kernel`] — triggered by the scheduler when the
/// last user process `exit`s — rewriting the interrupt's return frame to land here in
/// ring 0, on the boot stack that `usermode::enter` saved. We arrive on the *last
/// user program's* CR3 and switch back to
/// the kernel space first thing. From the kernel's point of view this is simply "the
/// rest of `kernel_main`": it runs the tests in a `cargo test` build, or launches
/// the interactive shell otherwise. Never returns.
fn boot_continue() -> ! {
    // Stage 12a: we resumed on the *user program's* CR3 (every address space maps
    // the kernel, so the handful of instructions to get here were fine). Switch back
    // to the kernel address space before doing anything else.
    process::return_to_kernel_space();

    // We were resumed with interrupts disabled (the rewritten frame cleared IF);
    // re-enable them now that we are safely back on the kernel stack.
    x86_64::instructions::interrupts::enable();
    serial_println!(
        "[usermode] resumed in the kernel (ring 0); reached ring 3 = {}, ring 3 syscalls = {}",
        usermode::reached_ring3(),
        syscall::ring3_syscall_count()
    );
    serial_println!(
        "[sched] {} processes ran with {} yields and {} preemptions (last on L4 {:#x}, kernel L4 {:#x}); back on the kernel space",
        process::processes_exited(),
        process::processes_yielded(),
        process::processes_preempted(),
        process::last_user_run_l4(),
        process::kernel_l4(),
    );
    serial_println!(
        "[sched] wait: {} parent(s) collected a child, last child exit code = {}",
        process::processes_waited(),
        process::last_waited_code(),
    );
    serial_println!(
        "[sched] spawn: {} child process(es) created at runtime via the spawn syscall",
        process::processes_spawned(),
    );
    println!("Back from running user processes; continuing boot.");

    // Stage 10a: exercise the `int 0x80` syscall path from ring 0 — the very path
    // the ring 3 program will use in 10b. `sys_write` makes the kernel print on the
    // caller's behalf; `sys_getpid` returns a value back across the boundary.
    {
        let msg = b"hello, kernel, from a syscall\n";
        // SAFETY: `msg` is a valid readable byte slice for SYS_WRITE; SYS_GETPID
        // ignores its arguments.
        let pid = unsafe {
            syscall::invoke(syscall::SYS_WRITE, msg.as_ptr() as u64, msg.len() as u64);
            syscall::invoke(syscall::SYS_GETPID, 0, 0)
        };
        serial_println!("[syscall] getpid() returned {}", pid);
    }

    // From here the two builds diverge (`#[cfg(test)]` compiles exactly one block;
    // see the test-harness note in `src/testing.rs`). The interactive shell's
    // executor never returns, so in a test build it would keep the tests from ever
    // running — instead we hand control to the generated test harness, which runs
    // every `#[test_case]` and then exits QEMU with a pass/fail status.
    #[cfg(test)]
    {
        test_main(); // runs the tests, then exits QEMU; never returns in practice
        hlt_loop(); // only to satisfy the `-> !` return type
    }

    // Stage 7: an interactive shell on the revived async executor. A boot self-test
    // first runs canned commands through the dispatcher (so the command logic is
    // verifiable without a keyboard), then we hand the CPU to the executor running
    // the shell task, which reads keystrokes, buffers a line, and dispatches it on
    // Enter. `Executor::run` never returns (`-> !`), so it is the kernel's final call.
    #[cfg(not(test))]
    {
        // Imported here, inside the non-test block, so they don't read as unused
        // when building the test harness (which excludes this whole block).
        use task::executor::Executor;
        use task::Task;

        println!();
        serial_println!("Kernel starting the interactive shell on the async executor.");
        shell::selftest();

        let mut executor = Executor::new();
        executor.spawn(Task::new(shell::run()));
        executor.run();
    }
}

/// Handler invoked when the kernel panics. On bare metal we must define this
/// ourselves, otherwise the code won't compile. In a normal build we log the
/// panic and halt forever.
#[cfg(not(test))]
#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    serial_println!();
    serial_println!("[PANIC] kernel panicked: {}", info);
    hlt_loop();
}

/// Panic handler for `cargo test` builds.
///
/// A panicking test means an assertion failed (or the kernel faulted) inside a
/// `#[test_case]`. We print `[failed]` and the message, then exit QEMU with the
/// `Failed` status so `bootimage` reports the run as a failure — rather than
/// halting forever and tripping the test timeout.
#[cfg(test)]
#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    serial_println!("[failed]");
    serial_println!("Error: {}", info);
    testing::exit_qemu(testing::QemuExitCode::Failed);
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
