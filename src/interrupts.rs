//! CPU exception handling via the Interrupt Descriptor Table (IDT).
//!
//! When the CPU hits an exception (a divide error, a page fault, a breakpoint,
//! ...) it looks up a 256-entry table called the IDT: entry N holds the address
//! of the handler for vector N. The `lidt` instruction tells the CPU where that
//! table lives. We build one table, register a handler for the breakpoint
//! exception, and load it.
//!
//! We handle two exceptions so far:
//! - The breakpoint exception (#BP, vector 3): a "trap". After the handler
//!   returns, the CPU continues at the instruction right after the one that
//!   triggered it, so the kernel keeps running.
//! - The double fault (#DF, vector 8): the safety net raised when dispatching
//!   another exception fails (e.g. on stack overflow). It cannot return, so its
//!   handler diverges. We register it to run on a dedicated stack via the IST
//!   slot set up in `gdt.rs`, so it works even when the kernel stack is gone.
//!
//! On top of CPU exceptions we also handle *hardware* interrupts, delivered
//! through the legacy 8259 PIC (programmable interrupt controller): the timer
//! on IRQ0, the keyboard on IRQ1, and so on. We remap the PIC's vectors to
//! 32..=47 (just past the exception vectors the CPU reserves) and, for now,
//! handle the timer.
//!
//! Later stages add more handlers (e.g. page fault, keyboard).

use core::sync::atomic::{AtomicU64, Ordering};

use pic8259::ChainedPics;
use spin::{Lazy, Mutex};
use x86_64::structures::idt::{InterruptDescriptorTable, InterruptStackFrame};

use crate::{gdt, hlt_loop, println, serial_println};

/// The kernel's interrupt descriptor table.
///
/// It must live for `'static`: `load()` hands the CPU a pointer to this table,
/// and the CPU keeps using it long after `init_idt` returns, so the table can
/// never be dropped. `Lazy` builds it on first access because
/// `InterruptDescriptorTable::new()` is not a `const fn` (so a plain `static`
/// initializer won't work).
static IDT: Lazy<InterruptDescriptorTable> = Lazy::new(|| {
    let mut idt = InterruptDescriptorTable::new();
    idt.breakpoint.set_handler_fn(breakpoint_handler);
    // SAFETY: `DOUBLE_FAULT_IST_INDEX` is a valid IST slot (0..=6), and `gdt.rs`
    // initializes that slot with a real, dedicated stack before we ever take an
    // exception. Pointing the double fault at it is what lets the handler run on
    // a known-good stack even when the kernel stack has overflowed.
    unsafe {
        idt.double_fault
            .set_handler_fn(double_fault_handler)
            .set_stack_index(gdt::DOUBLE_FAULT_IST_INDEX);
    }
    // Hardware interrupt: the timer (IRQ0). Indexing the IDT by vector number
    // reaches the entries past the 32 CPU-exception slots.
    idt[InterruptIndex::Timer.as_usize()].set_handler_fn(timer_interrupt_handler);
    idt
});

/// Load the IDT into the CPU with `lidt`. Call once, early in boot.
pub fn init_idt() {
    IDT.load();
}

// ---------------------------------------------------------------------------
// Hardware interrupts: the 8259 PIC and the timer (IRQ0).
// ---------------------------------------------------------------------------

/// Where we remap the PICs' interrupt vectors. The 8259 defaults to 8..=15,
/// which collides with the CPU exception vectors (0..=31, e.g. #DF at 8), so we
/// move them just above the exceptions: the primary PIC takes 32..=39, the
/// secondary 40..=47.
pub const PIC_1_OFFSET: u8 = 32;
pub const PIC_2_OFFSET: u8 = PIC_1_OFFSET + 8;

/// The two chained 8259 PICs, behind a spinlock so the init code and the
/// interrupt handlers can share them.
///
/// SAFETY: the offsets 32 and 40 are >= 32 (clear of the exception vectors), are
/// 8 apart, and fit in a `u8` — exactly the contract `ChainedPics::new`
/// requires. Wrong offsets would misroute interrupts or shadow CPU exceptions.
static PICS: Mutex<ChainedPics> =
    Mutex::new(unsafe { ChainedPics::new(PIC_1_OFFSET, PIC_2_OFFSET) });

/// Count of timer interrupts handled since boot. For now it only throttles the
/// serial log below; later stages (preemptive scheduling) will drive time slices
/// from a tick counter like this one.
static TIMER_TICKS: AtomicU64 = AtomicU64::new(0);

/// Vector numbers for the hardware interrupts we handle, laid out relative to
/// the PIC offset. IRQ0 (the timer) lands on `PIC_1_OFFSET` (= 32).
#[derive(Debug, Clone, Copy)]
#[repr(u8)]
enum InterruptIndex {
    Timer = PIC_1_OFFSET,
}

impl InterruptIndex {
    fn as_u8(self) -> u8 {
        self as u8
    }

    fn as_usize(self) -> usize {
        usize::from(self.as_u8())
    }
}

/// Initialize and remap the PICs. Call once after the IDT is loaded and before
/// enabling interrupts with `sti`.
pub fn init_pics() {
    // SAFETY: called exactly once during boot. The PICs sit at their fixed,
    // standard I/O ports, and we configured valid, non-overlapping offsets
    // above, so initializing them here cannot misconfigure another device.
    unsafe {
        PICS.lock().initialize();
    }
}

/// Handler for the breakpoint exception (#BP), raised by the `int3` instruction.
///
/// `extern "x86-interrupt"` makes the compiler emit the special prologue/epilogue
/// an exception handler needs: it preserves every register the handler touches
/// and returns with `iretq` instead of a normal `ret`. The CPU pushes an
/// `InterruptStackFrame` (the saved instruction pointer, code segment, flags,
/// stack pointer and stack segment) which we receive as the argument.
extern "x86-interrupt" fn breakpoint_handler(stack_frame: InterruptStackFrame) {
    // Full detail goes to the serial log; a short note goes to the screen.
    serial_println!("[EXCEPTION] breakpoint\n{:#?}", stack_frame);
    println!("[EXCEPTION] breakpoint handled; kernel continues");
}

/// Handler for the double fault (#DF, vector 8).
///
/// A double fault occurs when the CPU fails to invoke an exception handler — for
/// example a page fault whose own handler can't be dispatched, or, the case we
/// guard against here, a stack overflow that leaves no room to push the
/// exception frame. Unlike #BP this is *not* recoverable: the architecture
/// forbids returning from it (note the `-> !`), and its error code is always 0.
/// Reaching this handler at all means we successfully switched to the dedicated
/// IST stack — without that, this fault would escalate to a triple fault and
/// reset the machine. We log as much as we can and halt instead of rebooting.
extern "x86-interrupt" fn double_fault_handler(
    stack_frame: InterruptStackFrame,
    _error_code: u64,
) -> ! {
    serial_println!("[EXCEPTION] DOUBLE FAULT\n{:#?}", stack_frame);
    println!("[EXCEPTION] DOUBLE FAULT - halting");
    hlt_loop();
}

/// Handler for the timer interrupt (IRQ0, remapped to vector 32).
///
/// The PIC raises this periodically on its own; unlike `int3`, nothing in our
/// code asks for it. We count the tick (logging it occasionally) and then
/// acknowledge the interrupt. Acknowledgement is mandatory: until the PIC
/// receives an end-of-interrupt (EOI), it delivers no further interrupt on this
/// line, so the timer would appear to fire exactly once.
extern "x86-interrupt" fn timer_interrupt_handler(_stack_frame: InterruptStackFrame) {
    let count = TIMER_TICKS.fetch_add(1, Ordering::Relaxed) + 1;
    // Log the first tick (proof the IRQ fired and EOI works), then every 100th,
    // so the serial log shows the timer running steadily without flooding it.
    if count == 1 || count % 100 == 0 {
        serial_println!("[timer] tick {}", count);
    }
    // SAFETY: we send the EOI for exactly the vector we are currently servicing.
    // Signaling the wrong vector could acknowledge an interrupt that never fired.
    unsafe {
        PICS.lock()
            .notify_end_of_interrupt(InterruptIndex::Timer.as_u8());
    }
}
