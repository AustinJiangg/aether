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
//! 32..=47 (just past the exception vectors the CPU reserves) and handle the
//! timer and the keyboard.
//!
//! Later stages add more handlers (e.g. page fault).

use core::arch::naked_asm;
use core::sync::atomic::{AtomicU64, Ordering};

use pic8259::ChainedPics;
use spin::{Lazy, Mutex};
use x86_64::instructions::port::Port;
use x86_64::registers::control::Cr2;
use x86_64::structures::idt::{
    InterruptDescriptorTable, InterruptStackFrame, InterruptStackFrameValue, PageFaultErrorCode,
};
use x86_64::{PrivilegeLevel, VirtAddr};

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
    // Hardware interrupts: the timer (IRQ0) and the keyboard (IRQ1). Indexing
    // the IDT by vector number reaches the entries past the 32 exception slots.
    // The timer uses a hand-written *naked* entry (Stage 12c) so it can capture the
    // full register set for preemption; the keyboard keeps the typed handler.
    // SAFETY: `timer_interrupt_entry` is a real naked interrupt entry that saves all
    // registers and ends in `iretq`; registering its address as the timer gate (a
    // default interrupt gate: present, DPL 0, IF cleared on entry) is sound.
    unsafe {
        idt[InterruptIndex::Timer.as_usize()]
            .set_handler_addr(VirtAddr::from_ptr(timer_interrupt_entry as *const ()));
    }
    // Page fault (#PF, vector 14) and general protection fault (#GP, vector 13). Without
    // these, any stray memory access or protection violation has no handler to dispatch,
    // so it escalates straight to a double fault — losing the one detail that explains it
    // (the faulting address in CR2, or the #GP error code). Registering them turns a
    // mysterious reboot/halt into a precise diagnostic.
    idt.page_fault.set_handler_fn(page_fault_handler);
    idt.general_protection_fault
        .set_handler_fn(general_protection_fault_handler);
    idt[InterruptIndex::Keyboard.as_usize()].set_handler_fn(keyboard_interrupt_handler);
    // Stage 10: the syscall vector. Setting the gate's DPL to 3 is what lets ring 3
    // execute `int 0x80` (a gate's DPL is the highest-numbered ring allowed to
    // invoke it); with the default DPL 0, a ring 3 `int 0x80` would raise a #GP
    // instead of entering the handler. Since Stage 12c-2 the syscall, like the timer,
    // enters through a hand-written *naked* stub that captures a full `TrapFrame`, so
    // the scheduler can save and restore a process's complete register state when a
    // `yield`/`exit` switches processes.
    // SAFETY: `syscall_entry` is a real naked interrupt entry that saves all
    // registers and ends in `iretq`; registering its address as the `int 0x80` gate
    // (an interrupt gate: present, IF cleared on entry) with DPL 3 is sound.
    unsafe {
        idt[0x80]
            .set_handler_addr(VirtAddr::from_ptr(crate::syscall::syscall_entry as *const ()))
            .set_privilege_level(PrivilegeLevel::Ring3);
    }
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

/// Count of timer interrupts handled since boot. It throttles the serial log
/// below, paces Stage 6b's preemptive time slices, and backs the shell's `ticks`
/// and `uptime` commands (read via [`timer_ticks`]).
static TIMER_TICKS: AtomicU64 = AtomicU64::new(0);

/// The number of timer interrupts handled since boot. Exposed for the shell's
/// `ticks` / `uptime` commands.
pub fn timer_ticks() -> u64 {
    TIMER_TICKS.load(Ordering::Relaxed)
}

/// Vector numbers for the hardware interrupts we handle, laid out relative to
/// the PIC offset. IRQ0 (the timer) lands on `PIC_1_OFFSET` (= 32) and IRQ1
/// (the keyboard) on the next vector (= 33).
#[derive(Debug, Clone, Copy)]
#[repr(u8)]
enum InterruptIndex {
    Timer = PIC_1_OFFSET,
    Keyboard,
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

/// Handler for the page fault (#PF, vector 14).
///
/// The CPU pushes an error code describing the access, and stashes the faulting *linear
/// address* in the CR2 register. We log both (plus the saved instruction pointer) and
/// halt — enough to pin down a bad pointer or an unmapped access, which previously would
/// have escalated to an unhelpful double fault.
extern "x86-interrupt" fn page_fault_handler(
    stack_frame: InterruptStackFrame,
    error_code: PageFaultErrorCode,
) {
    serial_println!("[EXCEPTION] PAGE FAULT");
    // SAFETY: reading CR2 is a plain register read with no side effects; it holds the
    // address whose access raised this fault.
    serial_println!("  accessed address (CR2): {:?}", Cr2::read());
    serial_println!("  error code: {:?}", error_code);
    serial_println!("{:#?}", stack_frame);
    println!("[EXCEPTION] PAGE FAULT - halting (see serial log)");
    hlt_loop();
}

/// Handler for the general protection fault (#GP, vector 13).
///
/// Raised by a protection violation — a bad segment selector, a disallowed privileged
/// instruction, a non-canonical address, and so on. The error code is the offending
/// selector (or 0). We log it and halt rather than escalate to a double fault.
extern "x86-interrupt" fn general_protection_fault_handler(
    stack_frame: InterruptStackFrame,
    error_code: u64,
) {
    serial_println!(
        "[EXCEPTION] GENERAL PROTECTION FAULT (error code {:#x})",
        error_code
    );
    serial_println!("{:#?}", stack_frame);
    println!("[EXCEPTION] GP FAULT - halting (see serial log)");
    hlt_loop();
}

/// The full register state of an interrupted context: every general-purpose
/// register followed by the interrupt frame the CPU pushed.
///
/// Both kernel entries that can trigger a context switch build this *same* layout on
/// the kernel stack: [`timer_interrupt_entry`] (Stage 12c-1) and, as of Stage 12c-2,
/// the syscall stub [`crate::syscall::syscall_entry`]. Each pushes the GP registers
/// so that, read from the lowest address up, they are `rax, rbx, ..., r15`
/// (`#[repr(C)]` keeps the struct in that exact order), then the CPU's already-pushed
/// `iframe` sits on top — so `rsp` after the pushes is a `*mut TrapFrame`. Saving and
/// restoring the *whole* frame is what lets the scheduler switch processes without
/// corrupting registers, whether the switch is a voluntary `yield`/`exit` syscall or
/// (Stage 12c-3) an asynchronous timer preemption that can strike between any two
/// instructions — where, unlike a syscall the program chose to make, any register may
/// hold live state.
#[derive(Clone, Copy)]
#[repr(C)]
pub struct TrapFrame {
    pub rax: u64,
    pub rbx: u64,
    pub rcx: u64,
    pub rdx: u64,
    pub rsi: u64,
    pub rdi: u64,
    pub rbp: u64,
    pub r8: u64,
    pub r9: u64,
    pub r10: u64,
    pub r11: u64,
    pub r12: u64,
    pub r13: u64,
    pub r14: u64,
    pub r15: u64,
    pub iframe: InterruptStackFrameValue,
}

impl TrapFrame {
    /// A fresh context for a not-yet-run process: begin executing at `iframe` with
    /// every general-purpose register zero. The scheduler overwrites these the first
    /// time the process is switched *out* (saving its live registers); a brand-new
    /// program reads no register before first writing it, so zero is a safe start.
    pub fn new(iframe: InterruptStackFrameValue) -> TrapFrame {
        TrapFrame {
            rax: 0,
            rbx: 0,
            rcx: 0,
            rdx: 0,
            rsi: 0,
            rdi: 0,
            rbp: 0,
            r8: 0,
            r9: 0,
            r10: 0,
            r11: 0,
            r12: 0,
            r13: 0,
            r14: 0,
            r15: 0,
            iframe,
        }
    }
}

/// Naked entry point for the timer interrupt (IRQ0, vector 32) — Stage 12c.
///
/// Unlike an `extern "x86-interrupt"` handler (which only exposes the interrupt
/// frame), this captures the *full* register set so the scheduler can preempt a user
/// process running at any instruction. It pushes every general-purpose register
/// (building a [`TrapFrame`] on the kernel stack), hands a pointer to it to
/// [`timer_dispatch`], then restores the registers and `iretq`s. In Stage 12c the
/// dispatch may swap the `TrapFrame`'s contents (and CR3) to another process, so the
/// `pop`s below can restore a *different* context than was saved.
#[unsafe(naked)]
unsafe extern "C" fn timer_interrupt_entry() {
    naked_asm!(
        // Save all GP registers. Pushed highest-numbered first so that in memory,
        // read upward from the final rsp, they are rax, rbx, ... r15 (TrapFrame order).
        "push r15",
        "push r14",
        "push r13",
        "push r12",
        "push r11",
        "push r10",
        "push r9",
        "push r8",
        "push rbp",
        "push rdi",
        "push rsi",
        "push rdx",
        "push rcx",
        "push rbx",
        "push rax",
        "mov rdi, rsp", // arg 1: pointer to the TrapFrame we just built
        "call {dispatch}",
        // Restore (possibly a different process's context after a preemptive switch).
        "pop rax",
        "pop rbx",
        "pop rcx",
        "pop rdx",
        "pop rsi",
        "pop rdi",
        "pop rbp",
        "pop r8",
        "pop r9",
        "pop r10",
        "pop r11",
        "pop r12",
        "pop r13",
        "pop r14",
        "pop r15",
        "iretq",
        dispatch = sym timer_dispatch,
    );
}

/// Rust side of the timer interrupt. Receives a pointer to the interrupted context's
/// [`TrapFrame`] on the kernel stack.
///
/// Counts the tick and sends the EOI (mandatory, and *before* any reschedule: until
/// the PIC is acknowledged it delivers no further timer IRQ). Through Stage 12c-2 the
/// timer behavior is unchanged (count, EOI, dormant kernel-thread reschedule); Stage
/// 12c-3 will, when the interrupted context is a user process, save `*tf` and
/// round-robin to the next process — true preemption.
extern "C" fn timer_dispatch(tf: *mut TrapFrame) {
    let count = TIMER_TICKS.fetch_add(1, Ordering::Relaxed) + 1;
    if count == 1 {
        // Prove the naked stub captured a real context: log the interrupted
        // instruction/stack pointers, the privilege level (CPL = the low 2 bits of the
        // saved CS — 0 in the kernel, 3 in a user process), and a register.
        // SAFETY: `tf` points at this interrupt's TrapFrame on the kernel stack.
        let f = unsafe { &*tf };
        serial_println!(
            "[timer] first tick: captured rip={:?} cs={:#x} (CPL {}) rsp={:?} rax={:#x}",
            f.iframe.instruction_pointer,
            f.iframe.code_segment,
            f.iframe.code_segment & 3,
            f.iframe.stack_pointer,
            f.rax,
        );
    } else if count % 100 == 0 {
        serial_println!("[timer] tick {}", count);
    }
    // SAFETY: we send the EOI for exactly the vector we are currently servicing, and
    // *before* any reschedule — until the PIC is acknowledged it delivers no further
    // timer IRQ. Signaling the wrong vector could acknowledge an interrupt that never
    // fired.
    unsafe {
        PICS.lock()
            .notify_end_of_interrupt(InterruptIndex::Timer.as_u8());
    }
    // Stage 12c-3: if the tick interrupted a *user* process (ring 3), preempt it and
    // round-robin to the next one; a tick in ring 0 instead feeds the (dormant)
    // kernel-thread scheduler, exactly as before. (Syscalls run with IF clear, so a
    // tick never lands inside one — only in user code or plain kernel code.)
    // SAFETY: `tf` points at this interrupt's TrapFrame; `on_timer_tick` may rewrite it
    // (and switch CR3) to resume a different process, which the stub's `iretq` enters.
    let frame = unsafe { &mut *tf };
    if frame.iframe.code_segment & 3 == 3 {
        crate::process::on_timer_tick(frame);
    } else {
        crate::thread::schedule();
    }
}

/// Handler for the keyboard interrupt (IRQ1, remapped to vector 33).
///
/// The PS/2 controller latches one scancode byte at I/O port 0x60 for each key
/// press or release. We must read that byte to clear the controller, or it will
/// not raise another keyboard interrupt. As of Stage 5 the handler does nothing
/// else with it: decoding and echoing moved into the async keyboard task. We just
/// hand the raw byte off through a lock-free queue (`add_scancode`) and then
/// acknowledge the interrupt. Keeping the handler this short — no locks, no
/// allocation, no printing on the hot path — is what makes it safe to run at any
/// moment, even while other code holds a lock.
extern "x86-interrupt" fn keyboard_interrupt_handler(_stack_frame: InterruptStackFrame) {
    let mut port: Port<u8> = Port::new(0x60);
    // SAFETY: 0x60 is the fixed PS/2 data port. Reading it consumes exactly one
    // pending scancode byte and has no other side effects; we must do it before
    // sending the EOI so the controller will deliver the next byte.
    let scancode: u8 = unsafe { port.read() };
    crate::task::keyboard::add_scancode(scancode);

    // SAFETY: we send the EOI for exactly the vector we are currently servicing.
    unsafe {
        PICS.lock()
            .notify_end_of_interrupt(InterruptIndex::Keyboard.as_u8());
    }
}
