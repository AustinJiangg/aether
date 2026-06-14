//! Global Descriptor Table (GDT), Task State Segment (TSS), and a dedicated
//! stack for the double fault handler (via the Interrupt Stack Table, IST).
//!
//! Why this comes before hardware interrupts:
//!
//! When the CPU hits an exception it calls the matching handler. But if
//! dispatching that handler *itself* fails — most importantly when the kernel
//! stack has overflowed, so the CPU can't even push the exception frame — the
//! CPU raises a "double fault". If dispatching the double fault *also* fails, it
//! gives up and "triple faults", which on real hardware and in QEMU means an
//! instant reset (an endless reboot loop). The double fault handler is our last
//! line of defense, so it must be able to run even when the normal kernel stack
//! is unusable.
//!
//! The x86_64 architecture lets a handler switch to a known-good stack via the
//! Interrupt Stack Table (IST): an array of 7 stack pointers stored in the Task
//! State Segment (TSS). We point the double fault's IST entry at a fresh stack,
//! so even a stack overflow can be handled cleanly instead of triple faulting.
//!
//! Loading a TSS requires it to be referenced by a segment descriptor in the
//! GDT — a leftover from the segmentation era that long mode still needs for
//! exactly this purpose. So we build a minimal GDT with a kernel code segment
//! and a TSS segment, load it, reload CS, then load the TSS selector with `ltr`.

use spin::Lazy;
use x86_64::structures::gdt::{Descriptor, GlobalDescriptorTable, SegmentSelector};
use x86_64::structures::tss::TaskStateSegment;
use x86_64::VirtAddr;

/// Which IST slot (0..=6) holds the double fault handler's stack. The same
/// index is referenced from `interrupts.rs` when registering the handler, so
/// the CPU knows which stack to switch to.
pub const DOUBLE_FAULT_IST_INDEX: u16 = 0;

/// Size of the dedicated double fault stack: 5 pages (20 KiB). Small, but plenty
/// for a handler that only prints and halts.
const DOUBLE_FAULT_STACK_SIZE: usize = 4096 * 5;

/// The Task State Segment. In long mode the TSS no longer holds a task's saved
/// registers (hardware task switching is gone); it survives mainly to hold the
/// IST and the privilege-level stack table. We fill IST entry 0 with a
/// dedicated stack for double faults.
static TSS: Lazy<TaskStateSegment> = Lazy::new(|| {
    let mut tss = TaskStateSegment::new();
    tss.interrupt_stack_table[DOUBLE_FAULT_IST_INDEX as usize] = {
        // Backing memory for the stack. We have no heap yet (that arrives in
        // Stage 4), so the stack lives as a zero-initialized static in .bss.
        static mut STACK: [u8; DOUBLE_FAULT_STACK_SIZE] = [0; DOUBLE_FAULT_STACK_SIZE];

        // Take the static's address with `addr_of!` rather than a reference, so
        // we never form a `&`/`&mut` to a mutable static (which would be unsound
        // if aliased). Nothing else touches `STACK`. x86 stacks grow downward,
        // so the CPU wants the *top* (highest) address as the stack pointer.
        let stack_start = VirtAddr::from_ptr(core::ptr::addr_of!(STACK));
        stack_start + DOUBLE_FAULT_STACK_SIZE as u64
    };
    tss
});

/// Segment selectors produced while building the GDT. We need them after
/// loading the table: the code selector is reloaded into CS, and the TSS
/// selector is loaded with `ltr`.
struct Selectors {
    code_selector: SegmentSelector,
    tss_selector: SegmentSelector,
}

/// The GDT together with the selectors that index into it.
static GDT: Lazy<(GlobalDescriptorTable, Selectors)> = Lazy::new(|| {
    let mut gdt = GlobalDescriptorTable::new();
    let code_selector = gdt.add_entry(Descriptor::kernel_code_segment());
    let tss_selector = gdt.add_entry(Descriptor::tss_segment(&TSS));
    (
        gdt,
        Selectors {
            code_selector,
            tss_selector,
        },
    )
});

/// Load the GDT, reload the code segment register (CS), and load the TSS.
/// Call once during early boot, before loading the IDT (the IDT's double fault
/// entry references the IST slot the TSS defines here).
pub fn init() {
    use x86_64::instructions::segmentation::{Segment, CS};
    use x86_64::instructions::tables::load_tss;

    GDT.0.load();

    // SAFETY: both selectors index valid descriptors in the GDT we just loaded
    // — `code_selector` the kernel code segment, `tss_selector` the TSS. After
    // `lgdt`, reloading CS with the matching code selector and loading the TSS
    // with `ltr` is exactly the required setup sequence; the values come
    // straight from `add_entry`, so they are correct by construction.
    unsafe {
        CS::set_reg(GDT.1.code_selector);
        load_tss(GDT.1.tss_selector);
    }
}
