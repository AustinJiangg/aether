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

/// Size of the ring 0 stack the CPU switches to on a privilege change — an
/// interrupt or exception taken while running in ring 3 (Stage 9). 5 pages.
const PRIVILEGE_STACK_SIZE: usize = 4096 * 5;

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
    // privilege_stack_table[0] is "rsp0": the stack the CPU switches to when an
    // interrupt or exception transfers control from ring 3 up to ring 0. User
    // code runs on its own (untrusted) stack, so the CPU must not handle the
    // interrupt there — it loads this dedicated kernel stack, pushes the interrupt
    // frame, and runs the handler on it. Without rsp0 set, the first interrupt
    // taken in user mode would have no kernel stack and escalate to a triple
    // fault. Unused until Stage 9b enters ring 3; harmless to set up now.
    tss.privilege_stack_table[0] = {
        // Same pattern as the IST stack above: a zero-initialized static in .bss
        // (no heap this early), addressed with `addr_of!` so we never form a
        // reference to a mutable static. x86 stacks grow down, so the CPU wants
        // the top (highest address) as the initial stack pointer.
        static mut STACK: [u8; PRIVILEGE_STACK_SIZE] = [0; PRIVILEGE_STACK_SIZE];
        let stack_start = VirtAddr::from_ptr(core::ptr::addr_of!(STACK));
        stack_start + PRIVILEGE_STACK_SIZE as u64
    };
    tss
});

/// Segment selectors produced while building the GDT. We need them after
/// loading the table: the code selector is reloaded into CS, and the TSS
/// selector is loaded with `ltr`.
struct Selectors {
    code_selector: SegmentSelector,
    tss_selector: SegmentSelector,
    user_code_selector: SegmentSelector,
    user_data_selector: SegmentSelector,
}

/// The GDT together with the selectors that index into it.
static GDT: Lazy<(GlobalDescriptorTable, Selectors)> = Lazy::new(|| {
    let mut gdt = GlobalDescriptorTable::new();
    let code_selector = gdt.add_entry(Descriptor::kernel_code_segment());
    let tss_selector = gdt.add_entry(Descriptor::tss_segment(&TSS));
    // Ring 3 code and data segments for user mode (Stage 9). `add_entry` folds
    // each descriptor's DPL into the returned selector's RPL, so both selectors
    // already carry RPL 3 — exactly what gets pushed as CS/SS when we drop to
    // ring 3 in Stage 9b.
    let user_code_selector = gdt.add_entry(Descriptor::user_code_segment());
    let user_data_selector = gdt.add_entry(Descriptor::user_data_segment());
    (
        gdt,
        Selectors {
            code_selector,
            tss_selector,
            user_code_selector,
            user_data_selector,
        },
    )
});

/// The ring 3 user code selector (RPL 3), pushed as `CS` when entering user mode.
pub fn user_code_selector() -> SegmentSelector {
    GDT.1.user_code_selector
}

/// The ring 3 user data selector (RPL 3), pushed as `SS` when entering user mode.
pub fn user_data_selector() -> SegmentSelector {
    GDT.1.user_data_selector
}

/// The ring 0 kernel code selector (RPL 0). Used to rebuild a ring 0 context when
/// returning to the kernel from user mode (Stage 9b).
pub fn kernel_code_selector() -> SegmentSelector {
    GDT.1.code_selector
}

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

    // Stage 9a checkpoint: the GDT now carries ring 3 segments and the TSS holds a
    // kernel stack (rsp0) for ring-3 -> ring-0 transitions. Log the user selectors
    // so a `cargo run` shows them; the descent into ring 3 itself arrives in 9b.
    let uc = user_code_selector();
    let ud = user_data_selector();
    crate::serial_println!(
        "[gdt] ring 3 segments ready: code={:#x}, data={:#x} (rpl {:?})",
        uc.0,
        ud.0,
        uc.rpl(),
    );
}

/// Load the shared kernel GDT on an application processor (Stage 16d) and reload its
/// segment registers — *without* loading a TSS.
///
/// A woken AP runs on the trampoline's temporary GDT, where the kernel selectors do not
/// exist. Before it can take an interrupt — whose IDT gate names the kernel code selector
/// — it must run on the kernel GDT. So we `lgdt` it and reload CS to the kernel code
/// segment. We also reload SS/DS/ES to the **null** selector (valid for ring 0 in long
/// mode, where data segments are ignored): the trampoline left them at its data selector,
/// which in the kernel GDT is the *DPL 3* user-data descriptor — leaving SS that way would
/// #GP on the first `iretq` (returning to ring 0 with a ring-3 stack segment).
///
/// We deliberately do **not** load a TSS. The kernel's single TSS is already `ltr`-loaded
/// by the BSP, and its descriptor's busy bit makes a second `ltr` on it #GP; an AP needs
/// no TSS yet, since it runs only ring-0 handlers that use the current stack (the timer
/// gate has no IST, and there is no ring-3 -> ring-0 transition to need rsp0). A per-CPU
/// TSS arrives when an AP must run ring 3 or survive a fault on a dedicated stack.
pub fn init_ap() {
    use x86_64::instructions::segmentation::{Segment, CS, DS, ES, SS};
    use x86_64::PrivilegeLevel;

    GDT.0.load();

    // SAFETY: `code_selector` indexes the 64-bit kernel code descriptor in the GDT we
    // just loaded, so reloading CS with it is the required sequence after `lgdt`. The
    // null selector (index 0) is a valid ring-0 stack/data segment in long mode; loading
    // it into SS/DS/ES replaces the trampoline's now-DPL-3 data selector.
    unsafe {
        CS::set_reg(GDT.1.code_selector);
        let null = SegmentSelector::new(0, PrivilegeLevel::Ring0);
        SS::set_reg(null);
        DS::set_reg(null);
        ES::set_reg(null);
    }
}
