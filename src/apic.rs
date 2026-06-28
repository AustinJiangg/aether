//! Stage 15: the Advanced Programmable Interrupt Controller (APIC).
//!
//! Until now hardware interrupts came through the legacy **8259 PIC** — a pair of
//! 1981-era chips with 15 interrupt lines, no notion of multiple CPUs, and no way
//! to scale to SMP. This stage retires it in favor of the modern **APIC**, which
//! is split into two parts:
//!
//! - the **Local APIC (LAPIC)**, one per CPU core, which receives interrupts,
//!   acknowledges them (the EOI), and has its own built-in timer. (It also sends
//!   the inter-processor interrupts that Stage 16's SMP bring-up will need.)
//! - the **IO-APIC**, one shared unit, which routes external device IRQs (the
//!   keyboard, ...) to a chosen core and vector. That is Stage 15b; this file's
//!   `init` brings up the Local APIC and moves the system timer onto it.
//!
//! Both are **memory-mapped**: the LAPIC's registers live at physical
//! `0xFEE00000` (reported by the IA32_APIC_BASE MSR). Three subtleties this stage
//! teaches:
//!
//! 1. **MMIO must be uncacheable.** APIC registers are device memory, not RAM. If
//!    the page were cached, a read could return a stale value and a write might
//!    never reach the device. We map the page with `NO_CACHE` (the PCD bit) so
//!    every access goes straight through.
//! 2. **The LAPIC timer must be calibrated.** The PIT's frequency is a fixed,
//!    known 1.193182 MHz; the LAPIC timer's is derived from the CPU bus clock,
//!    which is *not* architecturally fixed and is not reported anywhere. So we
//!    measure it: let the LAPIC timer free-run for a 10 ms window timed by the PIT,
//!    and count how far it moved. Real kernels do exactly this.
//! 3. **EOI moves.** A handler signals "interrupt serviced" by writing the LAPIC's
//!    EOI register ([`end_of_interrupt`]) instead of the 8259's command port.
//!
//! The LAPIC timer reuses the *same* interrupt vector the PIT timer used (32), so
//! the naked timer entry in `interrupts.rs` handles it unchanged — only the
//! interrupt *source* and the EOI change.

use x86_64::instructions::port::Port;
use x86_64::registers::model_specific::Msr;
use x86_64::structures::paging::{
    FrameAllocator, Mapper, Page, PageTableFlags, PhysFrame, Size4KiB,
};
use x86_64::{PhysAddr, VirtAddr};

use crate::serial_println;

/// IA32_APIC_BASE model-specific register: holds the LAPIC's physical base
/// address (bits 12+) and the global-enable bit (bit 11).
const IA32_APIC_BASE_MSR: u32 = 0x1B;
/// Bit 11 of IA32_APIC_BASE: the APIC global-enable flag.
const APIC_GLOBAL_ENABLE: u64 = 1 << 11;

/// Virtual address where we map the Local APIC's 4 KiB MMIO page. It sits in L4
/// slot 100, an otherwise-empty top-level slot well clear of the kernel, heap, and
/// user regions. Because it is mapped into the kernel L4 *before* any process
/// address space is cloned, every clone inherits the entry — which matters because
/// the timer's EOI is written from whatever process happens to be running.
const LAPIC_VIRT_BASE: u64 = 0x_0000_3200_0000_0000;

// Local APIC register offsets, relative to the MMIO base. Each is a 32-bit
// register at a 16-byte-aligned offset.
const REG_ID: u32 = 0x020;
const REG_VERSION: u32 = 0x030;
const REG_EOI: u32 = 0x0B0;
const REG_SVR: u32 = 0x0F0;
const REG_LVT_TIMER: u32 = 0x320;
const REG_TIMER_INIT_COUNT: u32 = 0x380;
const REG_TIMER_CUR_COUNT: u32 = 0x390;
const REG_TIMER_DIV: u32 = 0x3E0;

/// Spurious Interrupt Vector Register, bit 8: software-enable the Local APIC.
const SVR_APIC_ENABLE: u32 = 1 << 8;
/// The vector spurious interrupts are delivered on. A spurious interrupt needs no
/// EOI; `interrupts.rs` registers a no-op handler for this vector.
pub const SPURIOUS_VECTOR: u8 = 0xFF;

/// LVT entry bit 16: mask the interrupt source.
const LVT_MASKED: u32 = 1 << 16;
/// LVT timer bit 17: periodic mode (the timer reloads and fires repeatedly). With
/// this bit clear the timer is one-shot.
const LVT_TIMER_PERIODIC: u32 = 1 << 17;

/// Timer Divide Configuration value `0b0011` = divide the bus clock by 16. (The
/// encoding is non-obvious: bit 2 is skipped, so 0b0011 means 16, not 3.)
const TIMER_DIV_16: u32 = 0b0011;

/// The vector the LAPIC timer fires on — the same one the PIT timer used (IRQ0
/// remapped to 32), so the existing naked timer entry handles it unchanged.
const TIMER_VECTOR: u8 = 32;

/// Periodic tick frequency we program the LAPIC timer at. The shell's `uptime`
/// reads this (via `crate::apic::TIMER_HZ`) to convert ticks to seconds, so it is
/// the single source of truth for the kernel's tick rate.
pub const TIMER_HZ: u32 = 100;

// The PIT (8253/8254), used once to calibrate the LAPIC timer. Its input clock is
// a fixed 1.193182 MHz, which is what makes it a usable reference.
const PIT_FREQUENCY: u32 = 1_193_182;
/// PIT channel 2 data port. Channel 2's gate and output are software-controlled
/// through port 0x61, so we can time an interval by polling instead of taking
/// IRQ0 (which is masked once the PIC is disabled).
const PIT_CH2_DATA: u16 = 0x42;
/// PIT mode/command register.
const PIT_CMD: u16 = 0x43;
/// The port that gates PIT channel 2 and exposes its output: bit 0 = gate (enable
/// counting), bit 1 = speaker (we keep it off), bit 5 = channel-2 output level.
const PIT_CH2_GATE: u16 = 0x61;

/// Bring up the Local APIC and move the system timer onto it.
///
/// Call once at boot, after paging and the frame allocator are up (this maps the
/// APIC's MMIO page) and *before* enabling interrupts. It masks the 8259 PIC,
/// software-enables the Local APIC, calibrates its timer against the PIT, and
/// starts it firing periodically on the timer vector.
pub fn init(
    mapper: &mut impl Mapper<Size4KiB>,
    frame_allocator: &mut impl FrameAllocator<Size4KiB>,
) {
    disable_pic();
    map_lapic(mapper, frame_allocator);
    enable_lapic();
    init_timer();
}

/// Mask every line of the legacy 8259 PIC, so all hardware interrupts now arrive
/// through the APIC instead. `interrupts::init_pics` already remapped the PIC clear
/// of the CPU exception vectors, so even a masked-off spurious IRQ is harmless.
fn disable_pic() {
    // SAFETY: 0x21 and 0xA1 are the fixed data ports of the primary/secondary 8259
    // PICs. Writing 0xFF sets every interrupt-mask bit, which only stops the PIC
    // from delivering interrupts; it cannot misconfigure any other device. We mask
    // the secondary first so a cascade IRQ cannot slip through mid-update.
    unsafe {
        let mut pic2: Port<u8> = Port::new(0xA1);
        let mut pic1: Port<u8> = Port::new(0x21);
        pic2.write(0xFF);
        pic1.write(0xFF);
    }
    serial_println!("[apic] 8259 PIC masked (interrupts now go through the APIC)");
}

/// Map the Local APIC's MMIO page at [`LAPIC_VIRT_BASE`], uncacheable, and ensure
/// the APIC is globally enabled in the IA32_APIC_BASE MSR.
fn map_lapic(
    mapper: &mut impl Mapper<Size4KiB>,
    frame_allocator: &mut impl FrameAllocator<Size4KiB>,
) {
    // The LAPIC's physical base lives in IA32_APIC_BASE; read it (rather than assume
    // 0xFEE00000) and set the global-enable bit while we are here.
    let mut apic_base = Msr::new(IA32_APIC_BASE_MSR);
    // SAFETY: reading IA32_APIC_BASE is a side-effect-free MSR read; writing it back
    // with bit 11 set only (re)enables this CPU's own Local APIC.
    let base_val = unsafe { apic_base.read() };
    let phys_base = base_val & 0x000F_FFFF_FFFF_F000; // bits 12..52 = the base address
    unsafe { apic_base.write(base_val | APIC_GLOBAL_ENABLE) };

    let page = Page::<Size4KiB>::containing_address(VirtAddr::new(LAPIC_VIRT_BASE));
    let frame = PhysFrame::containing_address(PhysAddr::new(phys_base));
    // Present + writable, and crucially NO_CACHE: MMIO must bypass the cache so reads
    // see the device's live state and writes reach it immediately. Not
    // user-accessible — the APIC is kernel-only.
    let flags = PageTableFlags::PRESENT | PageTableFlags::WRITABLE | PageTableFlags::NO_CACHE;

    // SAFETY: `frame` is the Local APIC's MMIO page — device memory that is sound to
    // map, and deliberately the only mapping of it. `page` is an otherwise-unused
    // virtual address, so nothing is aliased; `map_to` draws intermediate-table
    // frames only from `frame_allocator`. We map into the active kernel space, so we
    // flush the TLB for the new page.
    unsafe {
        mapper
            .map_to(page, frame, flags, frame_allocator)
            .expect("failed to map the Local APIC MMIO page")
            .flush();
    }
    serial_println!(
        "[apic] Local APIC MMIO: phys {:#x} -> virt {:#x} (uncacheable)",
        phys_base,
        LAPIC_VIRT_BASE
    );
}

/// Software-enable the Local APIC through the Spurious Interrupt Vector Register.
fn enable_lapic() {
    // SAFETY: `map_lapic` mapped the MMIO page just above, so these register accesses
    // are valid. Setting SVR bit 8 software-enables the APIC; the low byte is the
    // spurious-interrupt vector, which `interrupts.rs` backs with a no-op handler.
    unsafe {
        write(REG_SVR, SVR_APIC_ENABLE | SPURIOUS_VECTOR as u32);
        let id = read(REG_ID) >> 24; // the APIC ID sits in bits 24..32
        let version = read(REG_VERSION) & 0xFF;
        serial_println!(
            "[apic] Local APIC enabled (id {}, version {:#x})",
            id,
            version
        );
    }
}

/// Calibrate the LAPIC timer against the PIT, then start it firing periodically on
/// the timer vector.
fn init_timer() {
    let counts_per_sec = calibrate();
    // Counts to load for one period at TIMER_HZ. `.max(1)` guards against a degenerate
    // measurement producing 0 (which would disable the timer).
    let initial_count = (counts_per_sec / TIMER_HZ).max(1);

    // SAFETY: the LAPIC is mapped and enabled. Program the divisor, then the LVT timer
    // (periodic, our vector, unmasked), then the initial count last — writing the
    // count is what starts the periodic countdown.
    unsafe {
        write(REG_TIMER_DIV, TIMER_DIV_16);
        write(REG_LVT_TIMER, LVT_TIMER_PERIODIC | TIMER_VECTOR as u32);
        write(REG_TIMER_INIT_COUNT, initial_count);
    }
    serial_println!(
        "[apic] LAPIC timer calibrated: {} counts/s (bus/16); periodic = {} counts every tick ({} Hz) on vector {}",
        counts_per_sec,
        initial_count,
        TIMER_HZ,
        TIMER_VECTOR
    );
}

/// Measure the LAPIC timer's rate in counts-per-second (after the /16 divisor) by
/// letting it free-run for a 10 ms window timed by the PIT.
///
/// Runs with interrupts still disabled (the caller has not `sti`'d yet) and the
/// LAPIC timer masked, so nothing fires during the measurement.
fn calibrate() -> u32 {
    const CALIB_MS: u32 = 10;
    // PIT counts for a CALIB_MS window: input clock / (ticks per second of the window).
    let pit_count = (PIT_FREQUENCY / (1000 / CALIB_MS)) as u16;

    // SAFETY: the ports below are the fixed PIT and gate ports; the LAPIC is mapped and
    // enabled. Interrupts are disabled and the LAPIC timer is masked throughout, so the
    // busy-poll measures a clean, uninterrupted window.
    unsafe {
        // LAPIC timer: divide by 16, masked, free-running from the maximum count.
        write(REG_TIMER_DIV, TIMER_DIV_16);
        write(REG_LVT_TIMER, LVT_MASKED);
        write(REG_TIMER_INIT_COUNT, u32::MAX);

        // PIT channel 2, mode 0 (interrupt-on-terminal-count), lobyte/hibyte, binary.
        let mut gate: Port<u8> = Port::new(PIT_CH2_GATE);
        let base = gate.read() & 0xFC; // clear gate (bit 0) + speaker (bit 1)
        gate.write(base); // gate low: counting paused while we load the count
        Port::<u8>::new(PIT_CMD).write(0xB0);
        let mut ch2: Port<u8> = Port::new(PIT_CH2_DATA);
        ch2.write((pit_count & 0xFF) as u8);
        ch2.write((pit_count >> 8) as u8);

        // Snapshot the LAPIC count, raise the gate to start the PIT, and read the LAPIC
        // again right at the start — so we measure exactly the PIT's 10 ms window with no
        // skew from the few instructions of setup.
        gate.write(base | 1); // gate high: the PIT begins counting down
        let start = read(REG_TIMER_CUR_COUNT);
        // The PIT output (port 0x61 bit 5) goes high when the count reaches 0.
        while gate.read() & 0x20 == 0 {}
        let end = read(REG_TIMER_CUR_COUNT);

        write(REG_LVT_TIMER, LVT_MASKED); // stop the timer

        let elapsed = start.saturating_sub(end); // LAPIC counts during the window
        let per_sec = elapsed.saturating_mul(1000 / CALIB_MS);
        if per_sec == 0 {
            serial_println!("[apic] WARNING: timer calibration measured 0; using a fallback");
            10_000_000 // a safe-ish fallback so the timer still ticks
        } else {
            per_sec
        }
    }
}

/// Signal end-of-interrupt to the Local APIC.
///
/// Every APIC-delivered interrupt handler must call this (it replaces the 8259
/// PIC's EOI), or the LAPIC will deliver no further interrupt at or below the
/// current priority. Writing any value (we use 0) to the EOI register acknowledges
/// the interrupt currently in service.
pub fn end_of_interrupt() {
    // SAFETY: the LAPIC is mapped and enabled before interrupts are ever enabled, so
    // by the time any handler calls this the MMIO write is valid.
    unsafe { write(REG_EOI, 0) };
}

/// Read a 32-bit Local APIC register at `offset`.
///
/// # Safety
/// The LAPIC MMIO page must already be mapped (i.e. after [`init`] / [`map_lapic`]),
/// and `offset` must be a valid register offset.
unsafe fn read(offset: u32) -> u32 {
    ((LAPIC_VIRT_BASE + offset as u64) as *const u32).read_volatile()
}

/// Write a 32-bit Local APIC register at `offset`.
///
/// # Safety
/// The LAPIC MMIO page must already be mapped (i.e. after [`init`] / [`map_lapic`]),
/// and `offset` must be a valid register offset.
unsafe fn write(offset: u32, value: u32) {
    ((LAPIC_VIRT_BASE + offset as u64) as *mut u32).write_volatile(value);
}
