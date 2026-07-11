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
mod apic;
mod acpi;
mod smp;
mod percpu;
mod memory;
mod allocator;
mod ata;
mod fat;
mod pci;
mod e1000;
mod net;
mod task;
// The Stage 6 thread scheduler is dormant during Stage 7: the kernel runs the
// async executor (the shell) instead, so the scheduler's spawn/run/yield sit
// unused. (Its `schedule` is still called from the timer handler, but no-ops
// because preemption is never armed.) Silence the dead-code warnings for the
// subtree until a later stage folds async tasks and threads together.
#[allow(dead_code)]
mod thread;
// Stage 16d-3/16d-4: a per-CPU preemptive run queue, built on `thread`'s context
// switch. Each core schedules kernel threads on its own queue; the per-core timer
// preempts them (`sched::preempt`).
mod sched;
// Stage 16d-5: unify the async executor with the per-CPU scheduler — the executor runs
// as a kernel thread under `sched` on the BSP, peer to ordinary kernel threads.
mod unify;
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

    // Stage 3 / Stage 15: the legacy 8259 PIC is being retired in favor of the APIC.
    // Remap it now (clear of the CPU exception vectors, so any spurious PIC IRQ is
    // harmless), but leave interrupts *disabled* for the moment: the APIC and its
    // timer are brought up below, after paging maps the APIC's MMIO page, and only
    // then do we `sti`.
    interrupts::init_pics();
    serial_println!("[ OK ] PIC remapped (to be masked once the APIC is up)");

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

    // Stage 15: switch from the legacy 8259 PIC to the APIC. This must run after
    // paging + the frame allocator, because it maps the APIC's memory-mapped registers
    // into virtual memory (uncacheable) — which is why it lives here rather than beside
    // the old PIC setup above. `apic::init` masks the 8259 PIC, software-enables the
    // Local APIC and runs its timer (calibrated against the PIT) on vector 32 — the
    // very gate the PIT timer used — and brings up the IO-APIC to route the keyboard
    // IRQ to vector 33. From here the Local APIC timer (not the PIT) drives ticks and
    // preemption, and device IRQs arrive through the IO-APIC; `interrupts.rs`'s gates
    // and handlers are unchanged — only the interrupt source and the EOI moved.
    apic::init(&mut mapper, &mut frame_allocator);
    serial_println!("[ OK ] APIC up; timer + keyboard now delivered through the APIC");
    x86_64::instructions::interrupts::enable();
    serial_println!("[ OK ] hardware interrupts enabled; APIC timer is ticking");

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

    // Refinement: guard-paged kernel stacks. Each scheduled kernel thread's stack
    // now carries an unmapped page just below it, so an overflow faults on the guard
    // page (a clean #PF that names the address) instead of silently corrupting the
    // heap. Prove the mechanism here — allocate a guarded stack, confirm the guard
    // page is unmapped while the usable region is mapped, then free it and confirm
    // the guard is restored — before the scheduler hands these stacks to real threads.
    memory::demo_guard_page();
    serial_println!(
        "[ OK ] guard-paged kernel stacks verified = {}",
        memory::guard_page_ok()
    );
    println!("Kernel thread stacks now have guard pages (overflow -> page fault).");

    // Stage 16a: discover the machine's CPUs via ACPI. So far only the BSP (this
    // core) is running; the others (APs) are halted, waiting for the INIT-SIPI-SIPI
    // wake-up Stage 16b will send. To wake them we need their Local APIC ids, which
    // the firmware lists in the ACPI MADT table — so parse it now and report what we
    // found. Pure memory reads (no hardware is touched beyond reading our own LAPIC
    // id); placed here because it allocates while parsing, so the heap must be up.
    acpi::discover(phys_mem_offset);
    let aps = acpi::application_processors();
    serial_println!(
        "[ OK ] ACPI/SMP: {} CPU core(s); BSP apic id {}, {} AP(s) to wake: {:?}",
        acpi::cpu_count(),
        acpi::bsp_apic_id(),
        aps.len(),
        aps.iter().map(|c| c.apic_id).collect::<Vec<_>>(),
    );
    println!(
        "SMP: found {} CPU core(s) via ACPI ({} AP(s) still asleep; only the BSP runs).",
        acpi::cpu_count(),
        aps.len(),
    );

    // Stage 16b-1: prove the Local APIC can deliver an IPI by sending one to *this*
    // CPU. This exercises the exact ICR send + delivery-status poll that Stage 16b-2
    // will use to wake the APs with INIT-SIPI-SIPI — but on one core, with no assembly,
    // so any failure is isolated to the IPI mechanism. Interrupts were enabled above.
    let ipi_ok = interrupts::self_ipi_works();
    serial_println!(
        "[ OK ] self-IPI delivered (Local APIC send/receive path works) = {}",
        ipi_ok
    );
    println!("SMP: self-IPI test (Local APIC can send and receive an IPI) = {}", ipi_ok);

    // Stage 16c: build a private per-CPU data block for every discovered core, then wake
    // *all* the application processors. `percpu::init` must run before any AP is woken —
    // an AP marks its own block online the instant it enters Rust. `boot_aps` then wakes
    // the APs one at a time (each gets its own heap stack), climbing each from 16-bit real
    // mode through protected mode to 64-bit long mode (sharing the kernel's CR3) and into
    // `ap_entry`, which records the core online in its per-CPU block and parks. (16d puts
    // the now-idle cores to work.)
    percpu::init(&acpi::cpus());
    // Stage 16d-3: build one cooperative run queue per core, before any AP is woken — an
    // AP reaches for its own queue (to spawn its threads) the moment it enters the
    // scheduler in `ap_entry`. Indexed by the dense `cpu_index` `percpu` just assigned.
    sched::init(percpu::count());
    smp::boot_aps(&mut mapper, &mut frame_allocator, phys_mem_offset);
    serial_println!(
        "[ OK ] SMP bring-up: {}/{} application processor(s) online (every AP reached stage {}/3)",
        smp::aps_online(),
        acpi::application_processors().len(),
        smp::ap_stage(),
    );
    // The per-CPU table: each core's private block — its dense index, APIC id, role,
    // whether it is online, and (for an AP) the stack it is running on.
    serial_println!(
        "[percpu] {} per-CPU block(s), {} online:",
        percpu::count(),
        percpu::online_count(),
    );
    for cpu in percpu::all() {
        serial_println!(
            "[percpu]   cpu{} apic id {} {} {} (stack {:#x})",
            cpu.cpu_index,
            cpu.apic_id,
            if cpu.is_bsp { "BSP" } else { "AP " },
            if cpu.is_online() { "online " } else { "offline" },
            cpu.stack(),
        );
    }
    println!(
        "SMP: woke {} AP(s); {} of {} core(s) online, each with its own per-CPU data.",
        smp::aps_online(),
        percpu::online_count(),
        percpu::count(),
    );

    // Stage 16d-1: each AP enabled its *own* Local APIC timer in `ap_entry`. Give them a
    // few timer periods, then read each core's per-CPU tick count — a climbing count is
    // proof the APs are doing autonomous work (servicing their own timer interrupts on
    // their own LAPIC), not just parked. (`pit_sleep_us` polls the PIT, independent of the
    // BSP's own running timer.)
    apic::pit_sleep_us(50_000); // ~5 ticks per AP at 100 Hz
    serial_println!("[percpu] AP Local APIC timer ticks after ~50 ms:");
    for cpu in percpu::all().iter().filter(|c| !c.is_bsp) {
        serial_println!(
            "[percpu]   cpu{} apic id {}: {} timer tick(s)",
            cpu.cpu_index,
            cpu.apic_id,
            cpu.timer_ticks(),
        );
    }
    println!("SMP: each AP runs its own LAPIC timer now (per-CPU tick counts on the serial log).");

    // Stage 16d-4: each AP ran several kernel threads on its *own* per-CPU run queue under
    // timer preemption (in `ap_entry`) before parking — real per-core preemptive scheduling.
    // Report, per AP, how many threads completed, how many times the timer preempted one, the
    // work they did, and that the run queue drained back to its bootstrap context.
    serial_println!(
        "[percpu] AP per-CPU run queue ({} threads, preemptive, each spans {} tick(s)):",
        smp::AP_THREADS,
        smp::AP_THREAD_TICKS,
    );
    for cpu in percpu::all().iter().filter(|c| !c.is_bsp) {
        serial_println!(
            "[percpu]   cpu{} apic id {}: {} thread(s) completed, {} preemption(s), work {}, scheduler done = {}",
            cpu.cpu_index,
            cpu.apic_id,
            cpu.threads_completed(),
            cpu.preemptions(),
            cpu.work(),
            cpu.scheduler_done(),
        );
    }
    println!("SMP: each AP preemptively scheduled several kernel threads on its own run queue (serial log).");

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

    // Stage 13b: write a sector to disk, then read it back and confirm it matches. The
    // write goes to a *scratch* disk (the primary IDE slave) so the boot image is never
    // touched; QEMU gets it via `-drive ...,if=ide,index=1` and build.rs creates the file.
    // A successful round-trip proves the WRITE SECTORS + CACHE FLUSH path works end to end.
    {
        const SCRATCH_LBA: u32 = 0;
        // Build a recognizable sector: an ASCII tag up front (visible in a hexdump of
        // scratch.img after the run) followed by a 0,1,2,...,255,0,... ramp, so a wrong
        // byte anywhere in the sector — not just the first few — fails the comparison.
        let mut out = alloc::vec![0u8; ata::SECTOR_SIZE];
        for (i, b) in out.iter_mut().enumerate() {
            *b = (i & 0xFF) as u8;
        }
        out[..8].copy_from_slice(b"AETHER13");

        match ata::write_sector(ata::Drive::PrimarySlave, SCRATCH_LBA, &out) {
            Ok(()) => {
                let mut back = alloc::vec![0u8; ata::SECTOR_SIZE];
                match ata::read_sector_from(ata::Drive::PrimarySlave, SCRATCH_LBA, &mut back) {
                    Ok(()) => {
                        let matches = back == out;
                        serial_println!(
                            "[ata] wrote sector {} to the scratch disk and read it back; \
                             round-trip match = {}",
                            SCRATCH_LBA,
                            matches,
                        );
                        println!(
                            "Disk write works (ATA PIO): scratch-disk round-trip match = {}",
                            matches
                        );
                    }
                    Err(e) => {
                        serial_println!("[ata] read-back of scratch sector failed: {:?}", e);
                        println!("Disk write (ATA PIO) read-back FAILED: {:?}", e);
                    }
                }
            }
            Err(e) => {
                serial_println!("[ata] write to scratch sector failed: {:?}", e);
                println!("Disk write (ATA PIO) FAILED: {:?}", e);
            }
        }
    }

    // Stage 14b-2: mount a real FAT16 filesystem and read a file off it. The FAT disk is the
    // secondary IDE master (`fat.img`, formatted by the host's mkfs.fat with a known
    // HELLO.TXT). `Fat::mount` parses the boot sector's BPB (Stage 14b-1); `read_file` then
    // scans the root directory for the 8.3 name, follows its FAT cluster chain, and returns
    // the bytes — the kernel reading a file off a genuine on-disk filesystem.
    {
        match fat::Fat::mount(ata::Drive::SecondaryMaster) {
            Ok(volume) => {
                let bpb = volume.bpb();
                serial_println!(
                    "[fat] mounted: {} clusters; FAT@{}, root-dir@{}, data@{} (LBA)",
                    bpb.count_of_clusters(),
                    bpb.fat_start_sector(),
                    bpb.root_dir_start_sector(),
                    bpb.data_start_sector(),
                );
                match volume.read_file("HELLO.TXT") {
                    Ok(bytes) => {
                        serial_println!("[fat] read HELLO.TXT ({} bytes):", bytes.len());
                        // The file is known ASCII text; echo it to the serial log.
                        if let Ok(text) = core::str::from_utf8(&bytes) {
                            serial_print!("{}", text);
                        }
                        println!(
                            "FAT16 file read: HELLO.TXT ({} bytes) off a real disk",
                            bytes.len()
                        );
                    }
                    Err(e) => {
                        serial_println!("[fat] reading HELLO.TXT failed: {:?}", e);
                        println!("FAT16 file read FAILED: {:?}", e);
                    }
                }

                // Stage 14d-1: mkdir on the FAT disk — create a real subdirectory (a cluster
                // holding `.`/`..` plus an ATTR_DIRECTORY entry in the root), tolerating one left
                // by a previous boot (removing a directory comes in a later step), so the listing
                // below shows it alongside HELLO.TXT.
                match volume.make_root_dir("BOOTDIR") {
                    Ok(()) => {
                        serial_println!("[fat] mkdir /mnt/BOOTDIR: created a subdirectory on disk")
                    }
                    Err(fat::FatError::Exists) => {
                        serial_println!("[fat] mkdir /mnt/BOOTDIR: already exists (previous boot)")
                    }
                    Err(e) => serial_println!("[fat] mkdir /mnt/BOOTDIR failed: {:?}", e),
                }
                println!("FAT mkdir: created /mnt/BOOTDIR on disk (try 'ls /mnt').");

                // Stage 14b-2b: the FAT volume also implements the VFS `FileSystem` trait, so
                // it can be driven through a trait object — the very interface `RamFs` uses.
                // List the root directory through `&dyn FileSystem` to show the dispatch.
                use fs::FileSystem;
                let vfs: &dyn FileSystem = &volume;
                match vfs.list("/") {
                    Ok(entries) => {
                        serial_println!("[fat] root directory via the VFS trait:");
                        for (name, is_dir) in &entries {
                            serial_println!("        {}{}", name, if *is_dir { "/" } else { "" });
                        }
                        println!(
                            "FAT16 volume mounted behind the VFS trait ({} root entr{})",
                            entries.len(),
                            if entries.len() == 1 { "y" } else { "ies" },
                        );
                    }
                    Err(e) => serial_println!("[fat] VFS list of root failed: {:?}", e),
                }

                // Stage 14d-2: the read path now traverses subdirectories. `build.rs` seeds the
                // image with SUB/NESTED.TXT, so resolving that two-component path — scanning the
                // root for SUB, then walking SUB's own cluster chain — reads the nested file, the
                // same read a subdirectory `ls`/`cat` performs.
                match vfs.read("/SUB/NESTED.TXT") {
                    Ok(bytes) => {
                        serial_println!(
                            "[fat] traversed /SUB, read NESTED.TXT ({} bytes):",
                            bytes.len()
                        );
                        if let Ok(text) = core::str::from_utf8(&bytes) {
                            serial_print!("{}", text);
                        }
                        println!("FAT16 subdirectory traversal: read /mnt/SUB/NESTED.TXT off disk");
                    }
                    Err(e) => {
                        serial_println!("[fat] traversing /SUB/NESTED.TXT failed: {:?}", e);
                        println!("FAT16 subdirectory traversal FAILED: {:?}", e);
                    }
                }

                // Stage 14b-3: mount the FAT volume into the VFS at /mnt, so the interactive
                // shell's `ls`/`cat` reach disk files through the same `fs::*` API as the
                // in-memory tree. From here `/mnt/HELLO.TXT` reads this disk live.
                fs::mount(Box::new(volume));
                serial_println!("[fat] mounted the FAT volume at /mnt");
                println!("FAT volume mounted at /mnt (try: ls /mnt, cat /mnt/HELLO.TXT)");

                // Stage 14d-4: mkdir traverses too — create a directory *inside* the mounted
                // /mnt/SUB (its `..` pointing back at SUB), tolerating one from a previous boot
                // since rmdir is a later step.
                match fs::mkdir("/mnt/SUB/CHILD") {
                    Ok(()) => {
                        serial_println!("[fat] mkdir /mnt/SUB/CHILD: created a nested directory on disk")
                    }
                    Err(fs::FsError::Exists) => {
                        serial_println!("[fat] mkdir /mnt/SUB/CHILD: already exists (previous boot)")
                    }
                    Err(e) => serial_println!("[fat] mkdir /mnt/SUB/CHILD failed: {:?}", e),
                }
                println!("FAT mkdir: created /mnt/SUB/CHILD, a nested directory (try 'ls /mnt/SUB').");
            }
            Err(e) => {
                serial_println!("[fat] mount failed: {:?}", e);
                println!("FAT16 mount FAILED: {:?}", e);
            }
        }
    }

    // Stage 17a (networking): enumerate the PCI bus and locate the e1000 NIC. Before the kernel
    // can touch the card's registers (Stage 17b), it has to *find* the card: every PCI device
    // advertises its identity and the physical address of its register block (a BAR) in
    // configuration space, reached through the 0xCF8/0xCFC ports. QEMU puts `-device e1000` on the
    // bus. This is pure discovery — read-only config-space reads, like the ACPI table walk.
    let pci_devices = pci::enumerate();
    serial_println!("[pci] {} function(s) on the PCI bus:", pci_devices.len());
    for d in &pci_devices {
        let tag = if d.class == pci::CLASS_NETWORK && d.subclass == pci::SUBCLASS_ETHERNET {
            "  <- Ethernet controller"
        } else {
            ""
        };
        serial_println!(
            "[pci]   {:02x}:{:02x}.{}  {:04x}:{:04x}  class {:02x}.{:02x}.{:02x}{}",
            d.address.bus,
            d.address.device,
            d.address.function,
            d.vendor_id,
            d.device_id,
            d.class,
            d.subclass,
            d.prog_if,
            tag,
        );
    }
    match pci::find_e1000() {
        Some(nic) => {
            let bar0 = nic.mmio_bar(0).unwrap_or(0);
            serial_println!(
                "[ OK ] e1000 NIC at {:02x}:{:02x}.{}: MMIO BAR0 {:#x}, IRQ {}",
                nic.address.bus,
                nic.address.device,
                nic.address.function,
                bar0,
                nic.interrupt_line(),
            );

            // Stage 17b: start driving the card. 17b-1 maps its MMIO register block (uncacheable,
            // like the APIC's) into virtual memory; 17b-2 resets the card and applies general
            // configuration (Set-Link-Up, clear the multicast filter), then reads its identity — the
            // MAC address and Device Status register. `init` stashes the card globally; we read it
            // back through the global accessors for the summary line, confirming that path too.
            if e1000::init(&nic, &mut mapper, &mut frame_allocator).is_some() {
                if let Some(dev) = e1000::device() {
                    let mac = dev.mac();
                    println!(
                        "Network: e1000 reset {}, MAC {:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}, link {}, {}-duplex.",
                        if dev.reset_succeeded() { "ok" } else { "TIMED OUT" },
                        mac[0],
                        mac[1],
                        mac[2],
                        mac[3],
                        mac[4],
                        mac[5],
                        if dev.link_up() { "up" } else { "down" },
                        if dev.full_duplex() { "full" } else { "half" },
                    );
                    serial_println!(
                        "[ OK ] e1000 driver up: present={}, reset_ok={}, link-up requested (SLU)={}",
                        e1000::present(),
                        dev.reset_succeeded(),
                        dev.link_requested(),
                    );
                    // Stage 17b-3: the receive descriptor ring is armed and the receiver enabled.
                    serial_println!(
                        "[ OK ] e1000 RX ring installed = {}, receiver enabled = {} ({} descriptors)",
                        dev.rx_ring_installed(),
                        dev.receiver_enabled(),
                        dev.rx_count(),
                    );
                    println!(
                        "Network: e1000 RX ring armed with {} buffers; receiver enabled = {}.",
                        dev.rx_count(),
                        dev.receiver_enabled(),
                    );
                    // Stage 17b-4: the transmit ring is armed. Send a raw Ethernet frame and let the
                    // card confirm it (the transmit descriptor's Done bit) — proof the TX DMA path
                    // works, with no incoming traffic needed.
                    serial_println!(
                        "[ OK ] e1000 TX ring installed = {}, transmitter enabled = {} ({} descriptors)",
                        dev.tx_ring_installed(),
                        dev.transmitter_enabled(),
                        dev.tx_count(),
                    );
                    // Stage 17b-5: prove the receive path. Enable PHY loopback, send a frame to our
                    // own MAC, and receive it back off the RX ring — no external traffic needed. Do
                    // this *before* the outgoing SLIRP transmit below: a normal-mode frame handed to
                    // QEMU's host network stack leaves its receiver briefly busy, which drops the
                    // very next looped-back frame.
                    let looped = e1000::loopback_selftest();
                    serial_println!("[ OK ] e1000 loopback receive round-trip = {}", looped);
                    println!(
                        "Network: e1000 received a frame via loopback (round-trip = {}).",
                        looped,
                    );
                    // Stage 17b-4: the transmit ring is armed. Send a raw Ethernet frame out to the
                    // network and let the card confirm it (the transmit descriptor's Done bit) — proof
                    // the TX DMA path works.
                    let sent = e1000::transmit_test_frame();
                    serial_println!(
                        "[ OK ] e1000 transmitted a raw frame, card confirmed (DD) = {}",
                        sent,
                    );
                    println!(
                        "Network: e1000 transmitted a raw Ethernet frame (card confirmed = {}).",
                        sent,
                    );
                    // Stage 17b-6: switch the receive path from polling to interrupts. Route the
                    // card's IRQ line through the IO-APIC to a handled vector and arm the card's RX
                    // interrupt, then prove it: enable loopback, send a frame to ourselves, and let
                    // the interrupt handler — not a poll loop — drain it from the ring.
                    e1000::enable_interrupts(nic.interrupt_line());
                    let irq_rx = e1000::interrupt_selftest();
                    serial_println!(
                        "[ OK ] e1000 interrupt-driven receive = {} ({} IRQ(s), {} frame(s) of {} B polled off the ring)",
                        irq_rx,
                        e1000::rx_irq_count(),
                        e1000::rx_frames(),
                        e1000::last_rx_len(),
                    );
                    println!(
                        "Network: e1000 received a frame via interrupt (handler-driven = {}).",
                        irq_rx,
                    );
                }
            } else {
                println!("Network: e1000 found but BAR0 was not a memory BAR (unexpected).");
            }
        }
        None => {
            serial_println!("[pci] no e1000 NIC found (is QEMU started with -device e1000?)");
            println!("Network: no e1000 NIC found on the PCI bus.");
        }
    }

    // Stage 18a (networking): bring up the network stack over the e1000. The NIC moves raw Ethernet
    // frames; the `net` module turns them into a protocol stack, starting with the outermost layer —
    // Ethernet framing — and the receive plumbing (the RX interrupt now flags frames, `net::poll`
    // drains and dispatches them). Prove it with the card's own loopback: send a frame to ourselves
    // and confirm the stack receives and classifies it. ARP (18b) and IPv4/ICMP (18c) build on this.
    if e1000::present() {
        if let Some(dev) = e1000::device() {
            net::init(dev.mac());

            // Stage 20b (networking): DHCP. Before doing anything else, lease our IPv4 configuration
            // from SLIRP's built-in DHCP server via the four-step DORA exchange, instead of hardcoding
            // 10.0.2.15. On success the whole stack (ARP/ping/UDP below) runs on the *leased* address;
            // on failure we fall back to the static address so boot still proceeds.
            if net::dhcp_configure() {
                let ip = net::our_ip();
                let gw = net::leased_gateway();
                let dns = net::leased_dns();
                serial_println!(
                    "[ OK ] net 20b: DHCP lease {}.{}.{}.{} (gw {}.{}.{}.{}, dns {}.{}.{}.{}, {} s)",
                    ip[0], ip[1], ip[2], ip[3],
                    gw[0], gw[1], gw[2], gw[3],
                    dns[0], dns[1], dns[2], dns[3],
                    net::lease_secs(),
                );
                println!(
                    "Network: DHCP leased {}.{}.{}.{} (gateway {}.{}.{}.{}).",
                    ip[0], ip[1], ip[2], ip[3], gw[0], gw[1], gw[2], gw[3],
                );
            } else {
                net::use_static_fallback();
                let ip = net::our_ip();
                serial_println!(
                    "[net] net 20b: DHCP got no lease; using static fallback {}.{}.{}.{}",
                    ip[0], ip[1], ip[2], ip[3],
                );
                println!("Network: DHCP failed; using static address.");
            }

            let framing_ok = net::loopback_selftest();
            serial_println!(
                "[ OK ] net 18a: Ethernet framing over loopback = {} ({} frame(s) parsed)",
                framing_ok,
                net::frames_received(),
            );
            println!(
                "Network stack: received and parsed an Ethernet frame via loopback = {}.",
                framing_ok,
            );

            // Stage 18b (networking): ARP. Ask SLIRP's gateway for its MAC — the stack's first live
            // exchange with a real peer (SLIRP always answers ARP for 10.0.2.2). Getting a reply back
            // proves send + receive + parse all work end to end over the (emulated) wire.
            let gw = net::GATEWAY_IP;
            match net::arp_resolve(gw) {
                Some(mac) => {
                    serial_println!(
                        "[ OK ] ARP: {}.{}.{}.{} is at {:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x} ({} cache entry/entries)",
                        gw[0], gw[1], gw[2], gw[3],
                        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
                        net::arp::cache_len(),
                    );
                    println!(
                        "Network: ARP resolved gateway {}.{}.{}.{} -> {:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}.",
                        gw[0], gw[1], gw[2], gw[3],
                        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
                    );
                }
                None => {
                    serial_println!("[net] ARP: gateway did not answer (SLIRP link not ready?)");
                    println!("Network: ARP got no reply from the gateway.");
                }
            }

            // Stage 18c (networking): IPv4 + ICMP echo — ping. First a deterministic self-test of the
            // whole ICMP path via loopback (send an echo request to ourselves, answer it, receive the
            // reply). Then the headline: ping SLIRP's gateway over the real (emulated) wire and time
            // the round-trip by sequence number.
            let icmp_ok = net::ping_loopback_selftest();
            serial_println!(
                "[ OK ] net 18c: ICMP echo over loopback = {} (answered {}, received {})",
                icmp_ok,
                net::icmp_requests_handled(),
                net::icmp_replies_received(),
            );

            // Stage 19a-2 (networking): UDP — the first transport layer. A deterministic self-test of
            // the whole UDP path via loopback: send a datagram to our own echo server (port 7), which
            // bounces it back, and receive that echo — proving build/parse, the pseudo-header checksum,
            // dispatch by protocol, and the echo we generate, all with no external peer.
            let udp_ok = net::udp_echo_loopback_selftest();
            serial_println!(
                "[ OK ] net 19a: UDP echo over loopback = {} (echoes sent {}, delivered {})",
                udp_ok,
                net::udp_echoes_sent(),
                net::udp_delivered(),
            );

            // Stage 19b (networking): DNS over UDP — the first thing UDP does for real. Resolve a
            // hostname through SLIRP's DNS server (10.0.2.3), which forwards to the host's resolver.
            // Non-fatal: it needs working upstream DNS, so a failure only logs (like a ping timeout).
            let host = "example.com";
            match net::dns_resolve(host) {
                Some(ip) => serial_println!(
                    "[ OK ] net 19b: DNS {} -> {}.{}.{}.{}",
                    host, ip[0], ip[1], ip[2], ip[3],
                ),
                None => serial_println!(
                    "[net] net 19b: DNS {} did not resolve (no upstream DNS?)",
                    host,
                ),
            }

            // Stage 21b (networking): TCP three-way handshake. TCP is the first *reliable* transport —
            // connection-oriented, with sequence numbers and acknowledgements. Prove the handshake
            // deterministically via PHY loopback: listen on a port and connect to ourselves, so a
            // client TCB and a server TCB complete SYN / SYN-ACK / ACK and both reach ESTABLISHED.
            let tcp_ok = net::tcp_handshake_loopback_selftest();
            serial_println!(
                "[ OK ] net 21b: TCP handshake over loopback = {} ({} segment(s) parsed)",
                tcp_ok,
                net::tcp_segments_received(),
            );

            // Stage 21c (networking): TCP data transfer. Once established, the connection is a reliable,
            // ordered byte stream. Prove it via loopback: send a payload to ourselves and confirm the
            // receiving end buffered exactly those bytes in order and the sending end saw them ACKed.
            let tcp_data_ok = net::tcp_data_loopback_selftest();
            serial_println!(
                "[ OK ] net 21c: TCP data transfer over loopback = {}",
                tcp_data_ok,
            );

            // Stage 21d (networking): TCP teardown. Closing is a four-way FIN handshake — each direction
            // of the full-duplex stream closes independently. Prove it via loopback: actively close one
            // end and passively close the other, walking through FIN_WAIT/CLOSE_WAIT/LAST_ACK/TIME_WAIT
            // until the active closer is in TIME_WAIT and the passive closer is CLOSED.
            let tcp_teardown_ok = net::tcp_teardown_loopback_selftest();
            serial_println!(
                "[ OK ] net 21d: TCP teardown over loopback = {}",
                tcp_teardown_ok,
            );

            // Stage 21e (networking): TCP retransmission — what finally makes the transport *reliable*.
            // Every sent segment is kept until acknowledged; a timer resends it if the ACK is late, and
            // the active closer's TIME_WAIT expires under the same timer. Prove it via loopback by
            // dropping a data segment on purpose and confirming the timer recovers it (then closes).
            let tcp_rexmit_ok = net::tcp_retransmit_loopback_selftest();
            serial_println!(
                "[ OK ] net 21e: TCP retransmission over loopback = {} ({} resend(s) total)",
                tcp_rexmit_ok,
                net::tcp_retransmits(),
            );

            // Stage 22a (networking): TCP out-of-order reassembly. A segment arriving ahead of the next
            // expected byte is buffered, not dropped, and spliced into the stream once the gap fills.
            // Prove it via loopback by sending a payload as two segments delivered in *reversed* order and
            // confirming the receiver reassembles the bytes in order and acknowledges both.
            let tcp_reasm_ok = net::tcp_reassembly_loopback_selftest();
            serial_println!(
                "[ OK ] net 22a: TCP reassembly over loopback = {} ({} out-of-order buffered total)",
                tcp_reasm_ok,
                net::tcp_out_of_order_buffered(),
            );

            // Stage 22b (networking): TCP flow control — the receiver's sliding window. The advertised
            // window is the free receive-buffer space, so it shrinks as unread data piles up (to zero when
            // full) and reopens when the application reads. Prove it via loopback: fill the window to zero,
            // confirm a further segment is refused, then read to reopen it and watch the refused data land.
            let tcp_flow_ok = net::tcp_flow_control_loopback_selftest();
            serial_println!("[ OK ] net 22b: TCP flow control over loopback = {}", tcp_flow_ok);

            // Stage 22c (networking): the sender's sliding window. The sender now paces to the peer's
            // advertised window — segmenting a large send, buffering what the window has no room for, and
            // probing a zero window — so a slow receiver is never overrun. Prove it via loopback: hand the
            // sender more than the window, confirm it caps in-flight data and buffers the rest, then read
            // to reopen the window and watch every byte arrive in order.
            let tcp_snd_ok = net::tcp_sender_window_loopback_selftest();
            serial_println!("[ OK ] net 22c: TCP sender window over loopback = {}", tcp_snd_ok);

            // Stage 22d (networking): TCP congestion control — slow start. Beyond the peer's advertised
            // window (flow control), the sender now also obeys a *congestion window* (cwnd) that paces it to
            // the network. cwnd starts at one MSS and grows one MSS per ACK in slow start (doubling each
            // round trip). Prove it via loopback: stream several segments, draining the receiver so ACKs
            // flow, and watch cwnd climb well above its initial one-MSS value while the bytes arrive in order.
            let tcp_cc_ok = net::tcp_congestion_control_loopback_selftest();
            serial_println!("[ OK ] net 22d-1: TCP congestion control over loopback = {}", tcp_cc_ok);

            // Stage 22d-2 (networking): congestion backoff on loss — the multiplicative-decrease half of
            // AIMD. A retransmission timeout (a segment lost outright) collapses cwnd back to one MSS and
            // halves ssthresh, so a lossy path retreats and re-probes instead of hammering the network.
            // Prove it via loopback: grow cwnd by streaming a batch, then drop a segment so the RTO fires,
            // and watch cwnd fall back to ~one MSS and ssthresh drop while the bytes still recover in order.
            let tcp_bk_ok = net::tcp_congestion_backoff_loopback_selftest();
            serial_println!("[ OK ] net 22d-2: TCP congestion backoff over loopback = {}", tcp_bk_ok);

            // Stage 22d-3 (networking): fast retransmit + fast recovery. Three duplicate ACKs (a later
            // segment arrived but an earlier one is missing) let the sender recover a loss without waiting
            // for the full RTO, and only halve cwnd instead of collapsing it (the dup ACKs prove data still
            // flows). Prove it via loopback: burst four segments with the first dropped, so the three that
            // arrive trigger three dup ACKs and a fast retransmit — recovered before the timer, cwnd intact.
            let tcp_fr_ok = net::tcp_fast_retransmit_loopback_selftest();
            serial_println!(
                "[ OK ] net 22d-3: TCP fast retransmit over loopback = {} ({} fast retransmit(s) total)",
                tcp_fr_ok,
                net::tcp_fast_retransmits(),
            );

            // Stage 23a (networking): adaptive RTO — the sender now measures round-trip time and computes
            // its retransmission timeout per RFC 6298 (SRTT + 4*RTTVAR, clamped) with Karn's algorithm,
            // instead of a fixed constant. Prove the estimator formula on known samples and confirm a live
            // loopback transfer samples an RTT and lands on a sane RTO.
            let tcp_rtt_ok = net::tcp_rtt_estimation_loopback_selftest();
            serial_println!("[ OK ] net 23a: TCP adaptive RTO over loopback = {}", tcp_rtt_ok);

            // Stage 23b (networking): delayed ACKs — the receiver acknowledges at most every second in-order
            // segment (or after a short timer) instead of every one, halving ACK traffic; out-of-order
            // segments are still ACKed immediately so fast retransmit is unaffected. Prove it via loopback:
            // a batch of in-order segments draws fewer ACKs than there were segments, still in order.
            let tcp_dack_ok = net::tcp_delayed_ack_loopback_selftest();
            serial_println!("[ OK ] net 23b: TCP delayed ACK over loopback = {}", tcp_dack_ok);

            // Stage 23c (networking): Nagle's algorithm — the sender coalesces a burst of small writes,
            // holding a sub-MSS segment while earlier data is unacknowledged, so many tiny writes leave as a
            // few packets instead of one each (TCP_NODELAY disables it). Prove it via loopback: 16 one-byte
            // writes go out as far fewer segments, still in order.
            let tcp_nagle_ok = net::tcp_nagle_loopback_selftest();
            serial_println!("[ OK ] net 23c: TCP Nagle over loopback = {}", tcp_nagle_ok);

            // Stage 23d-1 (networking): TCP options infrastructure + SACK-permitted negotiation — the SYN and
            // SYN-ACK now carry the RFC 2018 SACK-permitted option, the stack's first use of TCP options.
            // Prove it via loopback: after the handshake both ends have SACK enabled (the option round-tripped
            // past the enlarged data offset and the negotiation recorded it on both sides).
            let tcp_sack_ok = net::tcp_sack_negotiation_loopback_selftest();
            serial_println!("[ OK ] net 23d-1: TCP SACK-permitted negotiated = {}", tcp_sack_ok);

            // Stage 23d-2a (networking): the receiver reports out-of-order data in SACK option blocks. When a
            // segment arrives ahead of the missing one, the dup ACK now carries a SACK block naming the range
            // it holds, so the sender (23d-2b) will learn exactly what to retransmit. Prove it via loopback:
            // reorder two segments and confirm the receiver emitted a SACK-carrying ACK and still reassembled.
            let tcp_sack_blk_ok = net::tcp_sack_blocks_loopback_selftest();
            serial_println!("[ OK ] net 23d-2a: TCP SACK blocks advertised = {}", tcp_sack_blk_ok);

            // Stage 23d-2b (networking): the sender consumes SACK blocks to recover several losses in one
            // round trip. When an ACK reports out-of-order data, the sender marks those segments received and,
            // on a fast retransmit, resends only the gaps between them. Prove it via loopback: burst five
            // segments with two non-adjacent ones dropped and confirm both holes recover in one event.
            let tcp_sack_rec_ok = net::tcp_sack_recovery_loopback_selftest();
            serial_println!("[ OK ] net 23d-2b: TCP SACK-guided recovery = {}", tcp_sack_rec_ok);

            match net::ping(gw) {
                Some(seq) => {
                    serial_println!(
                        "[ OK ] ping {}.{}.{}.{}: reply seq={}",
                        gw[0], gw[1], gw[2], gw[3], seq,
                    );
                    println!(
                        "Network: ping {}.{}.{}.{} -> reply received (seq {}).",
                        gw[0], gw[1], gw[2], gw[3], seq,
                    );
                }
                None => {
                    serial_println!("[net] ping {}.{}.{}.{}: no reply", gw[0], gw[1], gw[2], gw[3]);
                    println!("Network: ping to the gateway got no reply.");
                }
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

    // Stage 16d-5: unify the async executor with the per-CPU scheduler. Run a brief,
    // *testable* coexistence demo on the BSP — an async-executor thread and a plain
    // kernel thread, both scheduled on the BSP's own per-CPU run queue and time-sliced
    // by the BSP timer (its ring-0 tick now calls `sched::preempt`). This runs in BOTH
    // build profiles, so `cargo test` covers the unification (the interactive shell,
    // which only the non-test build runs, otherwise would not be reachable by a test).
    unify::demo();

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

    // Stage 7 + 16d-5: an interactive shell on the async executor — but the executor
    // now runs as a *kernel thread under the per-CPU scheduler* (`unify::run_shell_threaded`),
    // not as a separate top-level owner of the CPU. A boot self-test first runs canned
    // commands through the dispatcher (verifiable without a keyboard); then the shell
    // thread (and a coexisting kernel thread) run on the BSP scheduler forever.
    #[cfg(not(test))]
    {
        println!();
        serial_println!("Kernel starting the interactive shell as a scheduled kernel thread.");
        shell::selftest();
        unify::run_shell_threaded();
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
