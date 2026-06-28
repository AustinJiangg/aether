//! Stage 16b: SMP bring-up — waking the application processors.
//!
//! The APs that [`crate::acpi`] discovered are halted, waiting for the wake-up
//! sequence. This module sends it and climbs a freshly-woken core from a standing
//! start up to 64-bit long mode:
//!
//! ```text
//!   (BSP) INIT-SIPI-SIPI ──► AP begins in 16-bit *real mode* at vector << 12
//!                              └─► 32-bit protected mode   (marker 2)
//!                                    └─► 64-bit long mode   (marker 3)
//!                                          └─► Rust `ap_entry`  (Stage 16b-3)
//! ```
//!
//! This is the most triple-fault-prone code in the kernel. Stage 16b-2a proved the
//! *wake-up* with a minimal real-mode stub; **Stage 16b-2b (here)** extends the
//! trampoline through the full real → protected → long mode climb, writing a progress
//! marker at each rung (1 = real, 2 = protected, 3 = long). If the AP triple-faults
//! partway, the last marker the BSP polls pinpoints which transition failed — the only
//! practical way to debug a second core that has no console.
//!
//! Key mechanics:
//! - The AP starts at physical `vector << 12`, so the trampoline lives in a free,
//!   page-aligned, sub-1-MiB page (0x8000); we copy the assembled blob there at boot.
//! - Every absolute address is computed as `0x8000 + (label - start)` — a compile-time
//!   constant equal to the runtime address, so the copied blob needs no relocation.
//! - The two mode-switch far jumps are emitted as raw bytes (`0xEA ptr`), sidestepping
//!   assembler ambiguity over the far-jump mnemonics.
//! - The AP loads the *kernel's* CR3 (passed in a parameter slot), so once paging is on
//!   it shares the kernel's address space; the trampoline page is identity-mapped so the
//!   instruction right after CR0.PG still fetches.

use core::sync::atomic::{AtomicU32, Ordering};

use x86_64::registers::control::Cr3;
use x86_64::structures::paging::{FrameAllocator, OffsetPageTable, Size4KiB};
use x86_64::VirtAddr;

use crate::{acpi, apic, memory, serial_println};

/// Physical page where an AP begins executing. The SIPI vector is this page's number,
/// so the AP starts 16-bit real-mode code here. It must be a free, page-aligned,
/// sub-1-MiB conventional-RAM page; 0x8000 is the standard, free choice under QEMU.
const AP_TRAMPOLINE_PHYS: u64 = 0x8000;
/// The SIPI vector for [`AP_TRAMPOLINE_PHYS`]: its page number (0x8000 >> 12 = 0x08).
const AP_TRAMPOLINE_VECTOR: u8 = (AP_TRAMPOLINE_PHYS >> 12) as u8;
/// Offset within the page of the parameter slot the BSP fills before the SIPI: the
/// kernel's CR3 (an `u64`), which the trampoline loads to share the kernel's mappings.
const AP_CR3_OFFSET: u64 = 0xF00;
/// Offset of the per-AP stack-top pointer the BSP fills (an `u64`); the trampoline
/// loads it into RSP before entering Rust (Stage 16b-3).
const AP_STACK_OFFSET: u64 = 0xF08;
/// Offset of the Rust entry-point address the BSP fills (an `u64`); the trampoline
/// jumps to it once in long mode on a valid stack (Stage 16b-3).
const AP_ENTRY_OFFSET: u64 = 0xF10;
/// Offset within the page of the 4-byte progress marker the AP writes and the BSP polls.
const AP_MARKER_OFFSET: u64 = 0xFF0;

/// Progress-marker values: how far up the mode ladder the AP climbed.
const AP_MARKER_REAL: u32 = 1; // reached 16-bit real mode (executing our code)
const AP_MARKER_PROT: u32 = 2; // reached 32-bit protected mode
const AP_MARKER_LONG: u32 = 3; // reached 64-bit long mode

/// The highest stage an AP reached at boot (0 = never ran). Read by the tests.
static AP_STAGE: AtomicU32 = AtomicU32::new(0);

/// Number of application processors that have entered Rust (`ap_entry`) and reported
/// in. Each woken AP bumps this; the BSP polls it. Read by the Stage 16b-3 test.
static AP_ONLINE: AtomicU32 = AtomicU32::new(0);

/// Size of the stack handed to a woken AP. It is allocated from the kernel heap, which
/// is explicitly mapped (Stage 4c) and shared via the kernel CR3 the AP loads — a large
/// `static` array would not work: the 0.9 bootloader does not map the `.bss` pages past
/// the kernel file image, so the AP faults (not-present) on its first stack push.
const AP_STACK_SIZE: usize = 8 * 1024;

// The AP trampoline blob, assembled in place and copied to the low page at runtime.
//
// It climbs real -> protected -> long mode. The only runtime-provided value is the
// kernel CR3 (read from the parameter slot); everything else — the GDT, the absolute
// jump targets — is baked in relative to the known load address 0x8000.
core::arch::global_asm!(
    ".section .text.ap_trampoline, \"ax\"",
    ".code16",
    ".global ap_trampoline_start",
    "ap_trampoline_start:",
    "    cli",
    "    xor ax, ax",
    "    mov ds, ax",                       // DS = 0 so [0x8xxx] reaches the low page
    "    mov word ptr [{marker}], {mark_real}", // marker = 1: real mode reached
    "    lgdt [ap_gdtr_value]",
    "    mov eax, cr0",
    "    or eax, 1",                        // CR0.PE: enter protected mode
    "    mov cr0, eax",
    // Far jump to the 32-bit code segment (raw EA ptr16:16, real mode).
    "    .byte 0xEA",
    "    .word {tramp} + (ap_prot32 - ap_trampoline_start)",
    "    .word 0x08",

    ".code32",
    "ap_prot32:",
    "    mov ax, 0x10",                     // 32-bit data selector
    "    mov ds, ax",                       // reload segments (real-mode DS=0 is null now)
    "    mov es, ax",
    "    mov ss, ax",
    "    mov dword ptr [{marker}], {mark_prot}", // marker = 2: protected mode reached
    "    mov eax, cr4",
    "    or eax, 0x20",                     // CR4.PAE: required for long mode
    "    mov cr4, eax",
    "    mov eax, [{cr3p}]",                // load the kernel's CR3 (PML4 phys, < 4 GiB)
    "    mov cr3, eax",
    "    mov ecx, 0xC0000080",              // IA32_EFER
    "    rdmsr",
    "    or eax, 0x900",                    // EFER.LME (0x100, long mode) + EFER.NXE
    "    wrmsr",                            // (0x800): the kernel's page tables set the NX
                                            // bit, which is a *reserved* bit unless NXE is
                                            // on — the BSP runs with NXE, so the AP must too
                                            // or any walk of an NX page reserved-bit-faults
    "    mov eax, cr0",
    "    or eax, 0x80000000",               // CR0.PG: enable paging -> long mode active
    "    mov cr0, eax",
    // Far jump to the 64-bit code segment (raw EA ptr16:32, protected mode).
    "    .byte 0xEA",
    "    .long {tramp} + (ap_long64 - ap_trampoline_start)",
    "    .word 0x18",

    ".code64",
    "ap_long64:",
    "    mov ax, 0x20",                     // 64-bit data selector
    "    mov ds, ax",
    "    mov es, ax",
    "    mov ss, ax",
    "    mov rdi, {marker}",
    "    mov dword ptr [rdi], {mark_long}", // marker = 3: long mode reached!
    // Enter Rust: load this AP's stack, then jump to ap_entry. Its absolute address was
    // published by the BSP into a parameter slot, so no relocation is needed here (a
    // relative/RIP jump would mis-compute, since this code runs at 0x8000, not its link
    // address).
    "    mov rax, {stackp}",
    "    mov rsp, [rax]",                   // RSP = this AP's stack top
    "    mov rax, {entryp}",
    "    mov rax, [rax]",                   // RAX = &ap_entry
    "    jmp rax",                          // enter Rust (never returns)

    // Data: the GDT the trampoline installs, and its pseudo-descriptor for lgdt.
    ".balign 8",
    "ap_gdt:",
    "    .quad 0",                          // 0x00: null
    "    .quad 0x00CF9A000000FFFF",         // 0x08: 32-bit code (G, D, present, exec/read)
    "    .quad 0x00CF92000000FFFF",         // 0x10: 32-bit data (G, B, present, read/write)
    "    .quad 0x00AF9A000000FFFF",         // 0x18: 64-bit code (G, L=1, present, exec/read)
    "    .quad 0x00CF92000000FFFF",         // 0x20: 64-bit data (present, read/write)
    "ap_gdt_end:",
    "ap_gdt_ptr:",
    "    .word ap_gdt_end - ap_gdt - 1",    // limit
    "    .long {tramp} + (ap_gdt - ap_trampoline_start)", // base (absolute linear addr)

    // The absolute address of the GDT pointer, as a single constant symbol — an
    // instruction's memory operand may not contain a label difference directly, so we
    // fold `0x8000 + (ap_gdt_ptr - start)` into one `.set` symbol for the `lgdt` above.
    "    .set ap_gdtr_value, {tramp} + (ap_gdt_ptr - ap_trampoline_start)",

    ".global ap_trampoline_end",
    "ap_trampoline_end:",
    ".code64",                              // restore the assembler's default mode
    ".previous",                            // and the previously-active section
    tramp = const AP_TRAMPOLINE_PHYS,
    marker = const AP_TRAMPOLINE_PHYS + AP_MARKER_OFFSET,
    cr3p = const AP_TRAMPOLINE_PHYS + AP_CR3_OFFSET,
    mark_real = const AP_MARKER_REAL,
    mark_prot = const AP_MARKER_PROT,
    mark_long = const AP_MARKER_LONG,
    stackp = const AP_TRAMPOLINE_PHYS + AP_STACK_OFFSET,
    entryp = const AP_TRAMPOLINE_PHYS + AP_ENTRY_OFFSET,
);

extern "C" {
    /// First byte of the assembled trampoline blob (a label, not a function).
    static ap_trampoline_start: u8;
    /// One past the last byte of the trampoline blob.
    static ap_trampoline_end: u8;
}

/// Wake one application processor and climb it to 64-bit long mode.
///
/// Copies the trampoline to the low page, fills the kernel CR3 into the parameter slot,
/// identity-maps the page (so the AP can fetch after it enables paging), clears the
/// marker, then sends the target AP INIT-SIPI-SIPI and polls (bounded) for the marker
/// to reach the long-mode stage. `mapper`/`frame_allocator` install the identity
/// mapping; `phys_offset` reaches the low page through the physical-memory window.
pub fn boot_one_ap(
    mapper: &mut OffsetPageTable,
    frame_allocator: &mut impl FrameAllocator<Size4KiB>,
    phys_offset: VirtAddr,
) {
    let target = match acpi::application_processors().first() {
        Some(ap) => ap.apic_id,
        None => {
            serial_println!("[smp] no application processors to wake");
            return;
        }
    };

    // The AP runs the trampoline at its physical address with paging off, then loads
    // the kernel CR3 and enables paging — so the page must map to itself afterwards.
    memory::ensure_identity_mapped(AP_TRAMPOLINE_PHYS, mapper, frame_allocator);

    let kernel_cr3 = Cr3::read().0.start_address().as_u64();
    // Allocate this AP's stack on the heap (mapped and shared via the kernel CR3) and
    // leak it — the AP owns it for its lifetime. See AP_STACK_SIZE for why not a static.
    let stack = alloc::vec![0u8; AP_STACK_SIZE];
    let stack_base = stack.as_ptr() as usize as u64;
    core::mem::forget(stack);
    // Top of the stack, 16-aligned then biased by 8 so `ap_entry` sees the SysV ABI's
    // "rsp+8 is 16-aligned at entry" — we jump into it rather than `call`, so no return
    // address is pushed.
    let stack_top = ((stack_base + AP_STACK_SIZE as u64) & !0xF) - 8;
    let entry = ap_entry as *const () as usize as u64;
    let win = phys_offset.as_u64() + AP_TRAMPOLINE_PHYS;
    let marker_ptr = (win + AP_MARKER_OFFSET) as *mut u32;
    // SAFETY: `ap_trampoline_start/end` bound the assembled blob in readable .text.
    // `AP_TRAMPOLINE_PHYS` is a free low conventional-RAM page, so the window address
    // for the whole page is valid to write. We copy the blob and publish the kernel CR3,
    // this AP's stack top, and the `ap_entry` address into their parameter slots, then
    // zero the marker — all before any SIPI.
    unsafe {
        let start = core::ptr::addr_of!(ap_trampoline_start);
        let end = core::ptr::addr_of!(ap_trampoline_end);
        let len = end as usize - start as usize;
        core::ptr::copy_nonoverlapping(start, win as *mut u8, len);
        ((win + AP_CR3_OFFSET) as *mut u64).write_volatile(kernel_cr3);
        ((win + AP_STACK_OFFSET) as *mut u64).write_volatile(stack_top);
        ((win + AP_ENTRY_OFFSET) as *mut u64).write_volatile(entry);
        marker_ptr.write_volatile(0);
    }

    serial_println!(
        "[smp] waking AP apic id {} via INIT-SIPI-SIPI (trampoline at {:#x}, kernel cr3 {:#x})",
        target,
        AP_TRAMPOLINE_PHYS,
        kernel_cr3,
    );

    // The Intel universal startup sequence: INIT, wait 10 ms, SIPI, wait 200 us, SIPI.
    apic::send_init_ipi(target);
    apic::pit_sleep_us(10_000);
    apic::send_startup_ipi(target, AP_TRAMPOLINE_VECTOR);
    apic::pit_sleep_us(200);
    apic::send_startup_ipi(target, AP_TRAMPOLINE_VECTOR);

    // Poll (bounded, ~100 ms) for the AP to enter Rust and report online. The marker
    // (read afterwards) still records how far up the mode ladder it climbed, so a stall
    // is pinned to the rung it died on.
    let mut online = false;
    for _ in 0..100 {
        if AP_ONLINE.load(Ordering::SeqCst) >= 1 {
            online = true;
            break;
        }
        apic::pit_sleep_us(1_000);
    }
    // SAFETY: `marker_ptr` is the low-page marker we cleared; the volatile read observes
    // the AP's cross-core write.
    let stage = unsafe { marker_ptr.read_volatile() };
    AP_STAGE.store(stage, Ordering::SeqCst);

    if online {
        serial_println!(
            "[smp] AP apic id {} is online (running ap_entry on its own stack); stage {}/3",
            target,
            stage
        );
    } else {
        serial_println!(
            "[smp] AP apic id {} did not come online; stalled at stage {}/3 (1=real 2=prot 3=long)",
            target,
            stage
        );
    }
}

/// The highest mode-ladder stage an AP reached at boot: 0 = never ran, 1 = real mode,
/// 2 = protected mode, 3 = long mode. Recorded by [`boot_one_ap`]; read by the tests.
pub fn ap_stage() -> u32 {
    AP_STAGE.load(Ordering::SeqCst)
}

/// Number of application processors that have entered Rust (`ap_entry`) and reported
/// in. Recorded by the APs themselves; read by the Stage 16b-3 test.
pub fn aps_online() -> u32 {
    AP_ONLINE.load(Ordering::SeqCst)
}

/// Entry point for a freshly-woken AP: 64-bit long mode, on the kernel's address space
/// (it loaded the kernel CR3) and its own stack. Reached by the trampoline's final
/// `jmp`. Stage 16b-3 just records that this core is online, then parks it; later stages
/// give the AP its own IDT, Local APIC, and a share of the scheduler.
extern "C" fn ap_entry() -> ! {
    AP_ONLINE.fetch_add(1, Ordering::SeqCst);
    // Interrupts are still disabled on this core (it has no IDT yet), so `hlt` parks it
    // cleanly — waiting for an NMI rather than busy-spinning a core.
    crate::hlt_loop();
}
