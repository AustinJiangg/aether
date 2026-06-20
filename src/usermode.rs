//! Stage 9b: dropping to user mode (ring 3) and proving we got there.
//!
//! Everything so far has run in ring 0 (full privilege). A real OS runs user
//! programs in ring 3, where they cannot touch kernel memory or execute
//! privileged instructions. Stage 9a installed the two prerequisites in the GDT
//! and TSS — ring 3 code/data segments, and `rsp0` (the kernel stack the CPU
//! switches to when an interrupt arrives while in ring 3). This stage uses them.
//!
//! There is no "jump to a lower privilege level" instruction. The trick is to
//! *forge an interrupt-return frame*: we build the exact stack image the CPU
//! would have pushed had it interrupted a ring 3 program, then execute `iretq`.
//! The CPU believes it is returning from an interrupt and lands in ring 3.
//! ([`InterruptStackFrameValue::iretq`] writes that frame and runs `iretq` for
//! us, so we describe the target context as a struct instead of hand-writing
//! assembly.)
//!
//! Proving it worked: the ring 3 program is two bytes — `EB FE`, an infinite
//! `jmp .` loop that uses no memory and no stack. It just spins until the timer
//! interrupt fires. That interrupt enters the kernel through `rsp0`; in the
//! handler we read the saved code-segment selector, whose low two bits are the
//! privilege level the CPU came from. Seeing `CPL == 3` proves two things at
//! once: we really executed in ring 3, and `rsp0` let us take the interrupt
//! without a triple fault.
//!
//! Returning to the kernel: an interrupt handler normally `iretq`s back to
//! wherever it came from — here, the spinning ring 3 code. To resume the kernel
//! instead, the handler *rewrites its own return frame* to a ring 0 context (see
//! [`on_timer_tick`]). This is exactly the mechanism a scheduler will later use
//! to switch the CPU between a user process and the kernel; here it just lets boot
//! continue (into the shell or the tests) after a brief excursion into ring 3.

use core::sync::atomic::{AtomicBool, AtomicU64, Ordering};

use x86_64::instructions::interrupts;
use x86_64::structures::idt::{InterruptStackFrame, InterruptStackFrameValue};
use x86_64::structures::paging::{
    FrameAllocator, Mapper, OffsetPageTable, Page, PageTableFlags, PhysFrame, Size4KiB,
};
use x86_64::VirtAddr;

use crate::{gdt, serial_println};

/// Virtual address of the single user-accessible page that holds the ring 3 code.
///
/// Chosen in an otherwise-empty PML4 slot (index 4, the 32 TiB region) so that no
/// existing mapping shares its parent page tables. That matters: a ring 3 access
/// faults unless *every* level on the way down is marked `USER_ACCESSIBLE`, and
/// `map_to_with_table_flags` can only stamp that bit on tables it creates fresh.
const USER_PAGE: u64 = 0x2000_0000_0000;

// --- state shared with the timer interrupt handler -------------------------

/// Set while a descent to ring 3 is in flight: it tells [`on_timer_tick`] to watch
/// for the tick that interrupts ring 3 and to perform the one-time return rewrite.
static EXPECT_USER_TICK: AtomicBool = AtomicBool::new(false);

/// Set once the timer has observed the CPU running in ring 3 (`CPL == 3`). Read by
/// the Stage 9b test and logged during boot.
static REACHED_RING3: AtomicBool = AtomicBool::new(false);

/// Where [`on_timer_tick`] resumes the kernel after pulling us out of ring 3: the
/// continuation's instruction pointer, the kernel stack pointer to run it on, and
/// the ring 0 code selector. Filled in by [`enter`] before the descent.
static RESUME_RIP: AtomicU64 = AtomicU64::new(0);
static RESUME_RSP: AtomicU64 = AtomicU64::new(0);
static RESUME_CS: AtomicU64 = AtomicU64::new(0);

/// Whether the kernel has observed ring 3 execution. Used by the Stage 9b test and
/// logged by the boot continuation.
pub fn reached_ring3() -> bool {
    REACHED_RING3.load(Ordering::SeqCst)
}

/// Map the single ring 3 page and write a "spin forever" program into it; return
/// the user entry point (the virtual address of its first instruction).
///
/// The program is the two bytes `EB FE` (`jmp .`): an infinite loop that touches
/// no memory and uses no stack — the smallest possible ring 3 payload. All it has
/// to do is keep executing until the timer interrupts it.
pub fn map_user_code(
    mapper: &mut OffsetPageTable,
    frame_allocator: &mut impl FrameAllocator<Size4KiB>,
) -> VirtAddr {
    let page: Page<Size4KiB> = Page::containing_address(VirtAddr::new(USER_PAGE));
    let frame: PhysFrame<Size4KiB> = frame_allocator
        .allocate_frame()
        .expect("no free frame for the user code page");

    // The final page *and every parent table* must be USER_ACCESSIBLE, or a ring 3
    // instruction fetch faults at whichever level lacks the bit. Because USER_PAGE
    // lives in an empty PML4 slot, `map_to_with_table_flags` builds all four levels
    // fresh and stamps each parent with `parent_flags`. The page is left writable
    // (so the kernel can write the code below) and executable (no NO_EXECUTE), the
    // latter being what lets ring 3 fetch from it.
    let page_flags =
        PageTableFlags::PRESENT | PageTableFlags::WRITABLE | PageTableFlags::USER_ACCESSIBLE;
    let parent_flags =
        PageTableFlags::PRESENT | PageTableFlags::WRITABLE | PageTableFlags::USER_ACCESSIBLE;

    // SAFETY: `frame` is a fresh, unused frame from the allocator, mapped at a
    // virtual address (`USER_PAGE`) that nothing else uses, so no aliasing is
    // created. Making it user-accessible and executable is intentional — ring 3
    // must be able to fetch its instructions from here.
    unsafe {
        mapper
            .map_to_with_table_flags(page, frame, page_flags, parent_flags, frame_allocator)
            .expect("failed to map the user code page")
            .flush();
    }

    // Write the program. SMAP is not enabled, so ring 0 may write a user page
    // directly. `EB FE` = `jmp .` (a 2-byte relative jump back to itself).
    let code: *mut u8 = USER_PAGE as *mut u8;
    // SAFETY: the page was just mapped present and writable, so these two bytes
    // land inside it; nothing else aliases this address.
    unsafe {
        code.write_volatile(0xEB);
        code.add(1).write_volatile(0xFE);
    }

    VirtAddr::new(USER_PAGE)
}

/// Drop to ring 3 at `user_entry`; never returns to the caller.
///
/// We record where to resume the kernel (for [`on_timer_tick`]), then forge a ring
/// 3 interrupt-return frame and `iretq` into it. `resume` is where the kernel
/// continues *after* the timer has pulled us back out of ring 3; it runs in ring 0
/// on the current (boot) kernel stack and must never return — it takes over the
/// rest of boot.
pub fn enter(user_entry: VirtAddr, resume: fn() -> !) -> ! {
    // Capture the current kernel stack pointer; `resume` will run on it. This is
    // the boot stack, already known to be large enough to run the shell (it does
    // today). `enter` never returns, so reusing the stack below this point is safe.
    let kernel_rsp: u64;
    // SAFETY: reading RSP into a general register has no side effects.
    unsafe { core::arch::asm!("mov {}, rsp", out(reg) kernel_rsp, options(nomem, nostack)) };

    RESUME_RIP.store(resume as usize as u64, Ordering::SeqCst);
    // 16-byte align, then bias down by 8, so `resume` starts with the stack the
    // System V ABI expects at a function's first instruction (rsp ≡ 8 mod 16). The
    // few bytes below the captured RSP are unused stack, free to grow into.
    RESUME_RSP.store((kernel_rsp & !0xF) - 8, Ordering::SeqCst);
    RESUME_CS.store(u64::from(gdt::kernel_code_selector().0), Ordering::SeqCst);

    // Disable interrupts so no tick fires between arming the hook and the descent.
    // The user frame's RFLAGS has IF set, so interrupts come back on the instant we
    // enter ring 3 — and the very next tick then interrupts the ring 3 code.
    interrupts::disable();
    EXPECT_USER_TICK.store(true, Ordering::SeqCst);

    let user_frame = InterruptStackFrameValue {
        instruction_pointer: user_entry,
        code_segment: u64::from(gdt::user_code_selector().0),
        cpu_flags: 0x202, // reserved bit 1, plus IF (bit 9): ring 3 with interrupts on
        stack_pointer: VirtAddr::new(USER_PAGE + 0x1000), // top of the user page (EB FE never uses it)
        stack_segment: u64::from(gdt::user_data_selector().0),
    };

    serial_println!(
        "[usermode] entering ring 3 at {:?} (cs={:#x}, ss={:#x})",
        user_entry,
        user_frame.code_segment,
        user_frame.stack_segment
    );

    // SAFETY: the frame describes a valid ring 3 context — `user_entry` is a
    // mapped, user-accessible, executable page; the stack pointer is inside that
    // mapped page; CS/SS are the GDT's RPL 3 selectors; RFLAGS is sane with IF set.
    // `rsp0` is installed (Stage 9a), so the first interrupt taken from ring 3 has
    // a valid kernel stack. `iretq` thus transitions cleanly to user mode and does
    // not return here.
    unsafe { user_frame.iretq() }
}

/// Called from the timer interrupt handler on every tick.
///
/// A no-op unless a ring 3 descent is in flight ([`enter`] armed it). On the first
/// tick that interrupts ring 3 code (`CPL == 3`), it records success and rewrites
/// the interrupt-return frame so the handler's `iretq` resumes the kernel
/// continuation in ring 0 instead of returning to the spinning user program.
pub fn on_timer_tick(stack_frame: &mut InterruptStackFrame) {
    if !EXPECT_USER_TICK.load(Ordering::SeqCst) {
        return;
    }
    // The low two bits of the saved code selector are the privilege level the
    // interrupt came from. 3 means the timer struck while the CPU was in ring 3.
    if stack_frame.code_segment & 0b11 != 3 {
        return;
    }

    EXPECT_USER_TICK.store(false, Ordering::SeqCst);
    REACHED_RING3.store(true, Ordering::SeqCst);
    serial_println!("[usermode] timer interrupted ring 3 code (CPL=3); returning to the kernel");

    // Rewrite the return frame to a ring 0 context: the kernel continuation's RIP,
    // the kernel code selector, the kernel stack we saved in `enter`, SS = 0 (the
    // kernel runs with a null stack selector in long mode), and IF cleared so the
    // continuation re-enables interrupts deliberately.
    let resumed = InterruptStackFrameValue {
        instruction_pointer: VirtAddr::new(RESUME_RIP.load(Ordering::SeqCst)),
        code_segment: RESUME_CS.load(Ordering::SeqCst),
        cpu_flags: 0x002,
        stack_pointer: VirtAddr::new(RESUME_RSP.load(Ordering::SeqCst)),
        stack_segment: 0,
    };

    // SAFETY: we overwrite the interrupt-return frame with a valid ring 0 context
    // (kernel CS, a kernel stack pointer captured in `enter`, RIP at the kernel
    // continuation). The handler's `iretq` then resumes kernel execution there.
    unsafe { stack_frame.as_mut().write(resumed) };
}
