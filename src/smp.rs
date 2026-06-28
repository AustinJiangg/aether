//! Stage 16b: SMP bring-up — waking the application processors.
//!
//! The APs that [`crate::acpi`] discovered are halted, waiting for the wake-up
//! sequence. This module sends it and brings a core to life. Waking an AP is, in
//! full, a four-rung climb the AP must make from a standing start:
//!
//! ```text
//!   (BSP) INIT-SIPI-SIPI ──► AP begins in 16-bit *real mode* at vector << 12
//!                              └─► 32-bit protected mode  (16b-2b)
//!                                    └─► 64-bit long mode  (16b-2b)
//!                                          └─► Rust `ap_entry`  (16b-3)
//! ```
//!
//! This is the most triple-fault-prone code in the kernel, so it is split. **Stage
//! 16b-2a (here)** does only the first rung: it proves the BSP can *wake* an AP and
//! that the AP *executes our code*, by running a tiny real-mode stub that writes a
//! marker the BSP polls — then halts. Because it never enables paging, it needs no
//! page tables and no mode-switch assembly, isolating the wake-up mechanism itself.
//! Stage 16b-2b then extends the trampoline through protected and long mode (writing
//! a marker at each rung), and 16b-3 hands off to Rust.
//!
//! The AP starts in real mode at physical `vector << 12`, so the trampoline must live
//! in a free, page-aligned, sub-1-MiB page. We copy the assembled stub there at
//! runtime through the physical-memory window.

use core::sync::atomic::{AtomicBool, Ordering};

use x86_64::VirtAddr;

use crate::{acpi, apic, serial_println};

/// Physical page where an AP begins executing. The SIPI vector is this page's number,
/// so the AP starts 16-bit real-mode code here. It must be a free, page-aligned,
/// sub-1-MiB conventional-RAM page; 0x8000 is the standard, free choice under QEMU
/// (the bootloader loads the kernel at 1 MiB+, and the BIOS data areas sit below 0x500
/// and up near 0x9FC00).
const AP_TRAMPOLINE_PHYS: u64 = 0x8000;
/// The SIPI vector for [`AP_TRAMPOLINE_PHYS`]: its page number (0x8000 >> 12 = 0x08).
const AP_TRAMPOLINE_VECTOR: u8 = (AP_TRAMPOLINE_PHYS >> 12) as u8;
/// Offset within the trampoline page of the 4-byte progress marker the AP writes and
/// the BSP polls — kept well clear of the small code blob at the page start.
const AP_MARKER_OFFSET: u64 = 0xFF0;
/// The value the trampoline writes once it is executing (16b-2a: reached real mode).
const AP_MARKER_ALIVE: u32 = 0xA11E;

/// Set once an AP has reported alive. Read by the Stage 16b-2a test.
static AP_WOKE: AtomicBool = AtomicBool::new(false);

// The AP trampoline blob, assembled in place and copied to the low page at runtime.
//
// Stage 16b-2a keeps it minimal: the AP wakes in 16-bit real mode, sets DS = 0 so it
// can address the low page with a 16-bit displacement, writes the "alive" marker, and
// halts. With no mode switching yet it runs entirely with paging off and needs no page
// tables. Its only memory reference is the fixed physical marker address and its only
// jump is PC-relative, so the code is position-independent and safe to run at 0x8000.
core::arch::global_asm!(
    ".section .text.ap_trampoline, \"ax\"",
    ".code16",
    ".global ap_trampoline_start",
    "ap_trampoline_start:",
    "    cli",                               // the AP has no IDT; take no interrupts
    "    xor ax, ax",
    "    mov ds, ax",                        // DS = 0 so [phys] reaches the low page
    "    mov word ptr [{marker}], {alive}",  // tell the BSP we are executing
    "2:  hlt",                               // park (16b-3 will instead enter Rust)
    "    jmp 2b",
    ".global ap_trampoline_end",
    "ap_trampoline_end:",
    ".code64",                               // restore the assembler's default mode
    ".previous",                             // and the previously-active section
    marker = const AP_TRAMPOLINE_PHYS + AP_MARKER_OFFSET,
    alive = const AP_MARKER_ALIVE,
);

extern "C" {
    /// First byte of the assembled trampoline blob (a label, not a function).
    static ap_trampoline_start: u8;
    /// One past the last byte of the trampoline blob.
    static ap_trampoline_end: u8;
}

/// Wake one application processor and confirm it executes our trampoline.
///
/// Copies the real-mode trampoline to the low page, clears the marker, sends the
/// target AP the INIT-SIPI-SIPI sequence, then polls (bounded) for the AP to write the
/// "alive" marker. Returns whether an AP reported in. `phys_offset` is the
/// bootloader's physical-memory-window base, used to reach the low page.
pub fn boot_one_ap(phys_offset: VirtAddr) {
    let target = match acpi::application_processors().first() {
        Some(ap) => ap.apic_id,
        None => {
            serial_println!("[smp] no application processors to wake");
            return;
        }
    };

    let marker_ptr = (phys_offset.as_u64() + AP_TRAMPOLINE_PHYS + AP_MARKER_OFFSET) as *mut u32;
    // SAFETY: `ap_trampoline_start/end` bound the assembled blob in the kernel image
    // (readable .text). `AP_TRAMPOLINE_PHYS` is a free low conventional-RAM page, so
    // `phys_offset + AP_TRAMPOLINE_PHYS` is a valid writable window address for the
    // whole page. We copy the blob there and zero the marker before issuing any SIPI.
    unsafe {
        let start = core::ptr::addr_of!(ap_trampoline_start);
        let end = core::ptr::addr_of!(ap_trampoline_end);
        let len = end as usize - start as usize;
        let dst = (phys_offset.as_u64() + AP_TRAMPOLINE_PHYS) as *mut u8;
        core::ptr::copy_nonoverlapping(start, dst, len);
        marker_ptr.write_volatile(0);
    }

    serial_println!(
        "[smp] waking AP apic id {} via INIT-SIPI-SIPI (trampoline at {:#x}, vector {:#x})",
        target,
        AP_TRAMPOLINE_PHYS,
        AP_TRAMPOLINE_VECTOR,
    );

    // The Intel universal startup sequence: INIT, wait 10 ms, SIPI, wait 200 us, SIPI.
    apic::send_init_ipi(target);
    apic::pit_sleep_us(10_000);
    apic::send_startup_ipi(target, AP_TRAMPOLINE_VECTOR);
    apic::pit_sleep_us(200);
    apic::send_startup_ipi(target, AP_TRAMPOLINE_VECTOR);

    // Wait (bounded, ~100 ms) for the AP to write the alive marker.
    for _ in 0..100 {
        // SAFETY: `marker_ptr` is the low-page marker we just cleared; the volatile
        // read forces a fresh fetch so we observe the AP's cross-core write.
        if unsafe { marker_ptr.read_volatile() } == AP_MARKER_ALIVE {
            AP_WOKE.store(true, Ordering::SeqCst);
            serial_println!("[smp] AP apic id {} is alive (executed the trampoline)", target);
            return;
        }
        apic::pit_sleep_us(1_000);
    }
    serial_println!("[smp] AP apic id {} did not report in", target);
}

/// Whether an application processor reported alive at boot. Recorded by
/// [`boot_one_ap`]; read by the Stage 16b-2a test.
pub fn ap_woke() -> bool {
    AP_WOKE.load(Ordering::SeqCst)
}
