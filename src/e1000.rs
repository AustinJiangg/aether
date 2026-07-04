//! Stage 17b: the Intel e1000 (82540EM) network card driver.
//!
//! Stage 17a *found* the card on the PCI bus. Now we start *driving* it. Like the APIC, the e1000
//! is a memory-mapped device: it exposes a block of registers at the physical address advertised in
//! its BAR0, and we talk to it by reading and writing those registers. The card's real work —
//! sending and receiving Ethernet frames over DMA descriptor rings — comes in later sub-steps; the
//! ones here lay the foundation.
//!
//! ## 17b-1: reach the card's registers, read its identity
//!
//! Two things every MMIO driver must get right, both mirrored from `apic.rs`:
//!
//! 1. **The register block must be mapped uncacheable.** Device registers are not RAM: reading one
//!    can have side effects, and its value changes underneath us (the hardware writes it). If the
//!    CPU were allowed to cache it, a read might return a stale cached copy and a write might sit in
//!    the cache instead of reaching the device. We map the pages with `NO_CACHE` (the PCD bit) so
//!    every access goes straight to the card.
//!
//! 2. **Every access must be `volatile`.** Otherwise the optimizer, seeing "ordinary memory", could
//!    reorder, coalesce, or elide the loads and stores — fatal for a device that reacts to the exact
//!    sequence of register touches.
//!
//! The e1000's BAR0 region is 128 KiB. We map the whole region (32 pages) once, so the registers
//! scattered across it are all reachable.
//!
//! ## 17b-2: reset and general configuration
//!
//! Before doing anything useful, put the card in a known state — the standard opening move of a
//! device driver:
//!
//! 1. **Mask every interrupt** at the card (write all-ones to IMC, Interrupt Mask Clear). We have no
//!    e1000 IRQ handler yet (Stage 17b-5), so no interrupt source should be armed.
//! 2. **Global reset** (set CTRL.RST, a self-clearing bit): clears the card's internal state
//!    machines, FIFOs, and registers back to their power-on defaults. Poll until the bit self-clears
//!    (bounded, so a broken card cannot hang boot), then mask interrupts again (reset re-arms some)
//!    and drain any pending cause by reading ICR.
//! 3. **General config** in CTRL: set **SLU** (Set Link Up) so the MAC drives the link, and **ASDE**
//!    (Auto-Speed Detection Enable) so it negotiates speed; clear the link-reset, PHY-reset,
//!    invert-loss-of-signal, and VLAN-mode bits.
//! 4. **Clear the Multicast Table Array** (128 entries): accept no multicast group by default, and
//!    make sure no stale filter survives the reset.
//!
//! The reset reloads the Receive Address registers from the card's EEPROM, so we (re-)read the MAC
//! out of Receive Address entry 0 afterward, and read CTRL back to confirm SLU stuck.
//!
//! ## 17b-3: the receive (RX) descriptor ring
//!
//! Now the card can actually *receive*. A NIC does not interrupt the CPU for every byte; it moves
//! whole frames into main memory by **DMA**, coordinated through a **descriptor ring** — a circular
//! array of small records in RAM, shared between driver and card:
//!
//! - Each **receive descriptor** (16 bytes) holds the physical address of a **receive buffer** and a
//!   status byte. The driver fills in the buffer address and clears the status; when a frame arrives
//!   the card DMAs it into that buffer and writes the length plus a Descriptor-Done (DD) status bit.
//! - Two registers form a producer/consumer pair over the ring: **RDH** (head, advanced by the card
//!   as it fills descriptors) and **RDT** (tail, advanced by the driver as it recycles buffers). The
//!   card owns the descriptors in `[RDH, RDT)`; the driver owns the rest. We start the tail at the
//!   last descriptor, handing the card the whole ring to fill.
//! - **RDBAL/RDBAH** give the card the ring's physical base, **RDLEN** its size in bytes, and
//!   **RCTL** enables the receiver and sets the filters (accept broadcast, strip the Ethernet CRC)
//!   and the 2048-byte buffer size.
//!
//! The ring and buffers are ordinary RAM, not device registers, so — unlike the MMIO block — they
//! are reached through the kernel's normal **cacheable** physical-memory window. x86 DMA is
//! cache-coherent (the hardware snoops the caches), so the card and CPU see each other's writes with
//! no manual cache management; we still use `volatile` descriptor accesses so the *compiler* cannot
//! cache or reorder them. The buffer addresses we hand the card are raw *physical* addresses (DMA
//! speaks physical), which is exactly what the frame allocator returns.
//!
//! Interrupts stay masked — there is no RX interrupt handler yet (a later sub-step) — so once the
//! receiver is enabled the card fills buffers silently and advances RDH on its own. This sub-step
//! builds the ring and verifies it by reading the ring registers back off the card; consuming
//! received frames comes next.

use spin::Mutex;
use x86_64::{
    structures::paging::{FrameAllocator, Mapper, Page, PageTableFlags, PhysFrame, Size4KiB},
    PhysAddr, VirtAddr,
};

use crate::apic;
use crate::pci;
use crate::serial_println;

/// Virtual base where we map the e1000's MMIO register block. This is L4 slot 101 — an
/// otherwise-empty top-level slot, one slot above the APIC's (slot 100), well clear of the kernel,
/// heap, and physical-memory window (all in the lower half under the 0.9 bootloader). Each L4 slot
/// spans 512 GiB, so slot 101 = 101 * 0x80_0000_0000.
const E1000_VIRT_BASE: u64 = 0x_0000_3280_0000_0000;

/// Size of the e1000's MMIO register block (what BAR0 advertises): 128 KiB = 32 pages.
const E1000_MMIO_SIZE: u64 = 128 * 1024;

// --- e1000 register offsets, in bytes from the MMIO base (Intel 8254x manual) ---

/// Device Control.
const REG_CTRL: u64 = 0x0000;
/// Device Status — link up, link speed, full-/half-duplex, and more.
const REG_STATUS: u64 = 0x0008;
/// Interrupt Cause Read — reading it returns and clears the pending interrupt causes.
const REG_ICR: u64 = 0x00C0;
/// Interrupt Mask Clear — writing a 1 to a bit disables (masks) that interrupt cause.
const REG_IMC: u64 = 0x00D8;
/// Multicast Table Array: 128 32-bit entries (0x5200..0x5400) that filter which multicast MAC
/// addresses the receiver accepts.
const REG_MTA: u64 = 0x5200;
/// Number of entries in the Multicast Table Array.
const MTA_ENTRIES: u64 = 128;
/// Receive Address Low, entry 0: the low 4 bytes of the card's MAC address.
const REG_RAL0: u64 = 0x5400;
/// Receive Address High, entry 0: the high 2 bytes of the MAC, plus the Address Valid bit.
const REG_RAH0: u64 = 0x5404;

// --- Receive-path registers (Stage 17b-3) ---

/// Receive Control: enable the receiver and set its filters and buffer size.
const REG_RCTL: u64 = 0x0100;
/// Receive Descriptor Base Address Low — low 32 bits of the ring's physical address.
const REG_RDBAL: u64 = 0x2800;
/// Receive Descriptor Base Address High — high 32 bits of the ring's physical address.
const REG_RDBAH: u64 = 0x2804;
/// Receive Descriptor Length — the ring's size in bytes (must be a multiple of 128).
const REG_RDLEN: u64 = 0x2808;
/// Receive Descriptor Head — index the card fills next (the card advances it).
const REG_RDH: u64 = 0x2810;
/// Receive Descriptor Tail — one past the last descriptor the driver has handed the card.
const REG_RDT: u64 = 0x2818;

// --- CTRL (Device Control) bits ---

/// Link Reset.
const CTRL_LRST: u32 = 1 << 3;
/// Auto-Speed Detection Enable.
const CTRL_ASDE: u32 = 1 << 5;
/// Set Link Up — the MAC drives the link up.
const CTRL_SLU: u32 = 1 << 6;
/// Invert Loss-of-Signal.
const CTRL_ILOS: u32 = 1 << 7;
/// Device Reset — self-clearing: the card clears it when the reset completes.
const CTRL_RST: u32 = 1 << 26;
/// VLAN Mode Enable.
const CTRL_VME: u32 = 1 << 30;
/// PHY Reset.
const CTRL_PHY_RST: u32 = 1 << 31;

// --- STATUS (Device Status) bits ---

/// STATUS bit 1: link is up.
const STATUS_LU: u32 = 1 << 1;
/// STATUS bit 0: the link is full-duplex (else half-duplex).
const STATUS_FD: u32 = 1 << 0;
/// RAH bit 31: Address Valid — the MAC in RAL/RAH is populated and usable.
const RAH_AV: u32 = 1 << 31;

// --- RCTL (Receive Control) bits ---

/// Receiver Enable.
const RCTL_EN: u32 = 1 << 1;
/// Broadcast Accept Mode — receive broadcast frames (needed for ARP later).
const RCTL_BAM: u32 = 1 << 15;
/// Strip the Ethernet CRC (the 4-byte frame check sequence) from received frames.
const RCTL_SECRC: u32 = 1 << 26;
// Receive buffer size lives in RCTL bits 16-17 with the BSEX extender in bit 25; the all-zero
// encoding (which we use) selects 2048-byte buffers, so we leave those bits clear.

// --- Receive descriptor ring geometry (Stage 17b-3) ---

/// Number of receive descriptors in the ring. 32 x 16 B = 512 B — a multiple of 128 (as RDLEN
/// requires) that fits in a single 4 KiB frame (which holds 256 descriptors), so the ring is
/// physically contiguous by construction.
const RX_DESC_COUNT: usize = 32;
/// Bytes in each receive buffer, matching RCTL's 2048-byte setting. We give each descriptor its own
/// frame and use the low 2048 bytes — simple, and a frame is contiguous, which DMA requires.
#[allow(dead_code)]
const RX_BUFFER_SIZE: usize = 2048;

/// A legacy receive descriptor: 16 bytes the card DMAs into as it fills a buffer.
///
/// The driver writes `addr` (the physical address of a receive buffer) and clears `status`; the card
/// writes `length`/`checksum`/`status`/`errors` when a frame lands in the buffer, setting `status`'s
/// Descriptor-Done (DD) bit. `#[repr(C)]` pins the exact field order and 16-byte layout the hardware
/// reads — no padding (8 + 2 + 2 + 1 + 1 + 2 = 16, all naturally aligned).
#[repr(C)]
#[derive(Clone, Copy)]
struct RxDesc {
    /// Physical address of this descriptor's receive buffer (the card's DMA target).
    addr: u64,
    /// Length of the received frame, written by the card.
    length: u16,
    /// Packet checksum, written by the card.
    checksum: u16,
    /// Status bits (bit 0 = Descriptor Done), written by the card.
    status: u8,
    /// Error bits, written by the card.
    errors: u8,
    /// VLAN / special field, written by the card.
    special: u16,
}

/// The initialized card, once [`init`] has mapped its registers, reset it, and read its identity.
/// Stored behind a global so later sub-steps (and the tests) can reach the one card without
/// threading a handle through boot. The fields are plain data, so this is safe to move into a
/// `Mutex`.
static DEVICE: Mutex<Option<E1000>> = Mutex::new(None);

/// A handle on the e1000 NIC: the virtual base of its mapped register block, its MAC address, and
/// whether the reset completed. `Copy` so a caller can take a snapshot out of the global without
/// holding the lock while it works.
#[derive(Debug, Clone, Copy)]
pub struct E1000 {
    /// Virtual address of the mapped MMIO register block (== [`E1000_VIRT_BASE`]).
    mmio_base: u64,
    /// The card's 6-byte Ethernet MAC address.
    mac: [u8; 6],
    /// Whether the Stage 17b-2 global reset self-cleared within the timeout.
    reset_ok: bool,
    /// Virtual address (through the physical-memory window) of the receive descriptor ring, so the
    /// CPU can read/write descriptors. Zero until [`setup_rx`](Self::setup_rx) runs.
    rx_ring: u64,
    /// Physical address of the receive descriptor ring — the value programmed into RDBAL/RDBAH.
    rx_ring_phys: u64,
}

impl E1000 {
    /// Read the 32-bit register at `offset` bytes into the MMIO block.
    ///
    /// # Safety
    ///
    /// The MMIO block must be mapped (true after [`init`]) and `offset` must name a real register
    /// within the 128 KiB region. The read is `volatile`, so the compiler cannot elide or reorder
    /// it, and the mapping is uncacheable, so it reaches the device.
    unsafe fn read_reg(&self, offset: u64) -> u32 {
        ((self.mmio_base + offset) as *const u32).read_volatile()
    }

    /// Write `value` to the 32-bit register at `offset` bytes into the MMIO block.
    ///
    /// # Safety
    ///
    /// Same conditions as [`read_reg`](Self::read_reg); a write to a register with side effects
    /// takes effect on the device immediately.
    unsafe fn write_reg(&self, offset: u64, value: u32) {
        ((self.mmio_base + offset) as *mut u32).write_volatile(value);
    }

    /// Stage 17b-2: issue a global device reset and wait for it to complete. Returns whether the
    /// self-clearing `CTRL.RST` bit went low within the (bounded) timeout.
    ///
    /// # Safety
    ///
    /// The MMIO block must be mapped. This resets the whole card, so it must run during bring-up,
    /// before any descriptor ring or interrupt is armed.
    unsafe fn reset(&self) -> bool {
        // Mask every interrupt source first: we have no e1000 IRQ handler yet, and reset can raise
        // causes. Writing all-ones to IMC clears (disables) every mask bit.
        self.write_reg(REG_IMC, 0xFFFF_FFFF);

        // Set CTRL.RST. The device clears this bit itself once the reset finishes.
        let ctrl = self.read_reg(REG_CTRL);
        self.write_reg(REG_CTRL, ctrl | CTRL_RST);

        // The manual requires waiting a moment before touching the card again; then poll for the
        // bit to self-clear. Bounded (up to ~10 ms) so a broken card cannot hang boot forever.
        apic::pit_sleep_us(1);
        let mut cleared = false;
        for _ in 0..1000 {
            if self.read_reg(REG_CTRL) & CTRL_RST == 0 {
                cleared = true;
                break;
            }
            apic::pit_sleep_us(10);
        }

        // Reset re-arms interrupts, so mask them again and drain any pending cause.
        self.write_reg(REG_IMC, 0xFFFF_FFFF);
        let _ = self.read_reg(REG_ICR);
        cleared
    }

    /// Stage 17b-2: general device configuration after a reset — bring the link up and clear the
    /// multicast filter.
    ///
    /// # Safety
    ///
    /// The MMIO block must be mapped and the card just reset.
    unsafe fn configure(&self) {
        // Set-Link-Up + Auto-Speed-Detection; clear the reset / loss-of-signal / VLAN bits.
        let mut ctrl = self.read_reg(REG_CTRL);
        ctrl |= CTRL_SLU | CTRL_ASDE;
        ctrl &= !(CTRL_LRST | CTRL_PHY_RST | CTRL_ILOS | CTRL_VME);
        self.write_reg(REG_CTRL, ctrl);

        // Clear the Multicast Table Array (128 entries): accept no multicast group by default, and
        // make sure no stale filter survives the reset.
        for i in 0..MTA_ENTRIES {
            self.write_reg(REG_MTA + i * 4, 0);
        }
    }

    /// Read the MAC out of Receive Address entry 0 into `self.mac`. RAL holds bytes 0..4
    /// little-endian; RAH holds bytes 4..6 in its low 16 bits. Returns the raw RAH so the caller can
    /// check the Address Valid bit. On QEMU the reset reloads these from the emulated EEPROM.
    fn load_mac(&mut self) -> u32 {
        // SAFETY: the MMIO block is mapped and RAL0/RAH0 (0x5400/0x5404) are valid, side-effect-free
        // registers within it.
        let (ral, rah) = unsafe { (self.read_reg(REG_RAL0), self.read_reg(REG_RAH0)) };
        self.mac = [
            ral as u8,
            (ral >> 8) as u8,
            (ral >> 16) as u8,
            (ral >> 24) as u8,
            rah as u8,
            (rah >> 8) as u8,
        ];
        rah
    }

    /// This card's MAC address.
    pub fn mac(&self) -> [u8; 6] {
        self.mac
    }

    /// Whether the Stage 17b-2 global reset completed.
    pub fn reset_succeeded(&self) -> bool {
        self.reset_ok
    }

    /// Raw Device Control register (a live read from the card).
    pub fn control(&self) -> u32 {
        // SAFETY: `init` mapped the MMIO block; CTRL (0x0000) is a valid register within it.
        unsafe { self.read_reg(REG_CTRL) }
    }

    /// Raw Device Status register (a live read from the card).
    pub fn status(&self) -> u32 {
        // SAFETY: `init` mapped the MMIO block; STATUS (0x0008) is a valid, side-effect-free
        // register within it.
        unsafe { self.read_reg(REG_STATUS) }
    }

    /// Whether Set-Link-Up is asserted in CTRL — i.e. our [`configure`](Self::configure) write took
    /// effect.
    pub fn link_requested(&self) -> bool {
        self.control() & CTRL_SLU != 0
    }

    /// Whether the card reports its link as up.
    pub fn link_up(&self) -> bool {
        self.status() & STATUS_LU != 0
    }

    /// Whether the link is full-duplex.
    pub fn full_duplex(&self) -> bool {
        self.status() & STATUS_FD != 0
    }

    /// Stage 17b-3: build the receive descriptor ring and enable the receiver.
    ///
    /// Allocates one frame for the ring and one per receive buffer, points each descriptor at its
    /// buffer and clears its status, programs the ring's base/length/head/tail registers, then
    /// enables reception. The ring and buffers are normal cacheable RAM reached through the
    /// physical-memory window (x86 DMA is cache-coherent, so only the *registers* need the
    /// uncacheable MMIO mapping). The frames are never freed — they belong to the NIC for the life
    /// of the kernel, like the address-space L4 frames.
    ///
    /// # Safety
    ///
    /// The MMIO block must be mapped and the card reset/configured (so the receive-address filter is
    /// live). The physical-memory offset must be installed (true after `memory::init`).
    unsafe fn setup_rx(&mut self, frame_allocator: &mut impl FrameAllocator<Size4KiB>) {
        let phys_offset = crate::memory::physical_memory_offset().as_u64();

        // One frame for the descriptor ring (512 B of descriptors, with room to spare).
        let ring_frame = frame_allocator
            .allocate_frame()
            .expect("no free frame for the e1000 RX descriptor ring");
        self.rx_ring_phys = ring_frame.start_address().as_u64();
        self.rx_ring = phys_offset + self.rx_ring_phys;

        // Give each descriptor a fresh receive buffer and clear its status, so the card sees the
        // whole ring as available (DD clear) and knows where to DMA each frame.
        for i in 0..RX_DESC_COUNT {
            let buf = frame_allocator
                .allocate_frame()
                .expect("no free frame for an e1000 RX buffer");
            let desc = RxDesc {
                addr: buf.start_address().as_u64(),
                length: 0,
                checksum: 0,
                status: 0,
                errors: 0,
                special: 0,
            };
            // SAFETY: `rx_ring + i*16` is the i-th 16-byte descriptor slot of the ring frame, which
            // lies fully inside the physical-memory window and is 8-byte aligned (a multiple of 16
            // from a page-aligned base). The write is `volatile` so the compiler cannot reorder it
            // past the RDT store below that hands the ring to the card; x86's store ordering (TSO)
            // makes it visible to the card's DMA engine without an explicit fence.
            ((self.rx_ring + (i * 16) as u64) as *mut RxDesc).write_volatile(desc);
        }

        // Point the card at the ring and tell it the ring's size in bytes.
        self.write_reg(REG_RDBAL, self.rx_ring_phys as u32);
        self.write_reg(REG_RDBAH, (self.rx_ring_phys >> 32) as u32);
        self.write_reg(REG_RDLEN, (RX_DESC_COUNT * 16) as u32);

        // Head at 0, tail at the last descriptor. The card owns `[head, tail)` — every descriptor
        // but the one the tail rests on — and fills them as frames arrive, advancing the head.
        self.write_reg(REG_RDH, 0);
        self.write_reg(REG_RDT, (RX_DESC_COUNT - 1) as u32);

        // Enable the receiver: accept broadcast (for ARP later), strip the Ethernet CRC, 2048-byte
        // buffers (the BSIZE/BSEX bits stay clear). Unicast to our own MAC is accepted via Receive
        // Address 0 (its Address-Valid bit is set after the reset). Interrupts remain masked (no RX
        // IRQ handler yet), so the card just fills buffers silently.
        self.write_reg(REG_RCTL, RCTL_EN | RCTL_BAM | RCTL_SECRC);
    }

    /// Physical address of the receive descriptor ring (what we programmed into RDBAL/RDBAH).
    pub fn rx_ring_phys(&self) -> u64 {
        self.rx_ring_phys
    }

    /// Number of descriptors in the receive ring.
    pub fn rx_count(&self) -> usize {
        RX_DESC_COUNT
    }

    /// Receive Descriptor Base Address read back off the card (RDBAH:RDBAL).
    pub fn rx_descriptor_base(&self) -> u64 {
        // SAFETY: `init` mapped the MMIO block; RDBAL/RDBAH are valid registers within it.
        unsafe { (u64::from(self.read_reg(REG_RDBAH)) << 32) | u64::from(self.read_reg(REG_RDBAL)) }
    }

    /// Receive Descriptor Length in bytes, read back off the card (RDLEN).
    pub fn rx_descriptor_len(&self) -> u32 {
        // SAFETY: `init` mapped the MMIO block; RDLEN is a valid register within it.
        unsafe { self.read_reg(REG_RDLEN) }
    }

    /// Receive Descriptor Head index, read live (the card advances it as it fills descriptors).
    pub fn rx_head(&self) -> u32 {
        // SAFETY: `init` mapped the MMIO block; RDH is a valid register within it.
        unsafe { self.read_reg(REG_RDH) }
    }

    /// Receive Descriptor Tail index, read live.
    pub fn rx_tail(&self) -> u32 {
        // SAFETY: `init` mapped the MMIO block; RDT is a valid register within it.
        unsafe { self.read_reg(REG_RDT) }
    }

    /// Whether the receiver is enabled (RCTL.EN), read live from the card.
    pub fn receiver_enabled(&self) -> bool {
        // SAFETY: `init` mapped the MMIO block; RCTL is a valid register within it.
        unsafe { self.read_reg(REG_RCTL) & RCTL_EN != 0 }
    }

    /// Whether the RX ring is correctly installed: the card's descriptor base and length read back as
    /// what we programmed, and the receiver is enabled. One read-back check for the boot log and test.
    pub fn rx_ring_installed(&self) -> bool {
        self.rx_descriptor_base() == self.rx_ring_phys
            && self.rx_descriptor_len() as usize == RX_DESC_COUNT * 16
            && self.receiver_enabled()
    }
}

/// Map the e1000's 128 KiB MMIO register block at [`E1000_VIRT_BASE`], uncacheable.
///
/// The region is 32 contiguous 4 KiB pages; each virtual page `E1000_VIRT_BASE + i*0x1000` maps to
/// the physical frame `phys_base + i*0x1000`. Mirrors `apic::map_lapic`.
fn map_mmio(
    phys_base: u64,
    mapper: &mut impl Mapper<Size4KiB>,
    frame_allocator: &mut impl FrameAllocator<Size4KiB>,
) {
    // Present + writable, and crucially NO_CACHE: MMIO must bypass the cache (see the module docs).
    // Not user-accessible — the NIC is kernel-only.
    let flags = PageTableFlags::PRESENT | PageTableFlags::WRITABLE | PageTableFlags::NO_CACHE;
    let pages = E1000_MMIO_SIZE / 4096;
    for i in 0..pages {
        let page = Page::<Size4KiB>::containing_address(VirtAddr::new(E1000_VIRT_BASE + i * 0x1000));
        let frame = PhysFrame::containing_address(PhysAddr::new(phys_base + i * 0x1000));
        // SAFETY: `frame` is one page of the e1000's MMIO block — device memory that is sound to
        // map, and the only mapping of it. `page` is in an otherwise-unused top-level slot, so
        // nothing is aliased; `map_to` draws intermediate-table frames only from `frame_allocator`,
        // which hands out exclusively free frames. We map into the active kernel space, so we flush
        // the TLB for the new page.
        unsafe {
            mapper
                .map_to(page, frame, flags, frame_allocator)
                .expect("failed to map an e1000 MMIO page")
                .flush();
        }
    }
    serial_println!(
        "[e1000] MMIO register block: phys {:#x}..{:#x} -> virt {:#x} (uncacheable)",
        phys_base,
        phys_base + E1000_MMIO_SIZE,
        E1000_VIRT_BASE,
    );
}

/// Bring up the e1000: map its register block (17b-1), reset and configure the card (17b-2), and
/// read its identity (MAC + status). Returns the handle (also stashed in the global [`DEVICE`]), or
/// `None` if the card's BAR0 is not a memory BAR (it always is on QEMU).
///
/// Must run after paging and the frame allocator are up (it maps MMIO pages), after the APIC is up
/// (the reset uses `apic::pit_sleep_us` to pace the poll), and — since it reuses the active kernel
/// page tables — while the kernel address space is current.
pub fn init(
    nic: &pci::Device,
    mapper: &mut impl Mapper<Size4KiB>,
    frame_allocator: &mut impl FrameAllocator<Size4KiB>,
) -> Option<E1000> {
    let phys_base = nic.mmio_bar(0)?;
    map_mmio(phys_base, mapper, frame_allocator);

    let mut dev =
        E1000 { mmio_base: E1000_VIRT_BASE, mac: [0; 6], reset_ok: false, rx_ring: 0, rx_ring_phys: 0 };

    // Stage 17b-2: reset the card to a known state, then apply general configuration.
    // SAFETY: `map_mmio` just mapped the register block, so these register accesses are valid, and
    // this runs during bring-up before any ring or interrupt is armed.
    dev.reset_ok = unsafe { dev.reset() };
    unsafe { dev.configure() };

    // Read the MAC out of Receive Address entry 0 (the reset reloaded it from the EEPROM).
    let rah = dev.load_mac();

    // Stage 17b-3: build the receive descriptor ring and enable the receiver.
    // SAFETY: the MMIO block is mapped and the card was just reset and configured; the ring and
    // buffers come from the kernel frame allocator and are reached through the physical-memory
    // window, and the physical-memory offset is installed by this point in boot.
    unsafe { dev.setup_rx(frame_allocator) };

    let ctrl = dev.control();
    let status = dev.status();
    serial_println!(
        "[e1000] reset {} (CTRL {:#010x}, SLU {})",
        if dev.reset_ok { "completed" } else { "TIMED OUT" },
        ctrl,
        ctrl & CTRL_SLU != 0,
    );
    serial_println!(
        "[e1000] MAC {:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x} (address valid = {})",
        dev.mac[0],
        dev.mac[1],
        dev.mac[2],
        dev.mac[3],
        dev.mac[4],
        dev.mac[5],
        rah & RAH_AV != 0,
    );
    serial_println!(
        "[e1000] STATUS {:#010x}: link {}, {}-duplex",
        status,
        if status & STATUS_LU != 0 { "up" } else { "down" },
        if status & STATUS_FD != 0 { "full" } else { "half" },
    );
    serial_println!(
        "[e1000] RX ring: {} descriptors @ phys {:#x} (RDBA {:#x}, RDLEN {}, RDH {}, RDT {}), receiver {}",
        dev.rx_count(),
        dev.rx_ring_phys(),
        dev.rx_descriptor_base(),
        dev.rx_descriptor_len(),
        dev.rx_head(),
        dev.rx_tail(),
        if dev.receiver_enabled() { "enabled" } else { "DISABLED" },
    );

    *DEVICE.lock() = Some(dev);
    Some(dev)
}

/// The initialized card, if [`init`] has run and succeeded.
pub fn device() -> Option<E1000> {
    *DEVICE.lock()
}

/// Whether the e1000 has been brought up.
pub fn present() -> bool {
    DEVICE.lock().is_some()
}
