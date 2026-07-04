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
//!
//! ## 17b-4: the transmit (TX) descriptor ring
//!
//! Transmission mirrors reception: a **descriptor ring** shared with the card, driven by a head/tail
//! register pair. The difference is who produces and who consumes — for TX the *driver* fills
//! descriptors (a frame to send) and the *card* drains them:
//!
//! - Each **transmit descriptor** (16 bytes) holds the physical address of a buffer holding a frame,
//!   its length, and a **command** byte. The driver sets **EOP** (this descriptor ends the frame),
//!   **IFCS** (have the card append the 4-byte Ethernet CRC), and **RS** (report status — so the
//!   card writes back a Descriptor-Done (DD) bit we can poll).
//! - **TDH** (head) is advanced by the card as it transmits descriptors; **TDT** (tail) is advanced
//!   by the driver to hand new descriptors over. To send: fill the descriptor, then bump TDT past
//!   it. The card transmits everything in `[TDH, TDT)`, then advances TDH to meet TDT.
//! - **TDBAL/TDBAH/TDLEN** locate and size the ring; **TCTL** enables the transmitter and sets *pad
//!   short packets* (so a sub-60-byte frame is padded to the Ethernet minimum) and the collision
//!   parameters; **TIPG** sets the inter-packet gap.
//!
//! Verifying transmission needs no incoming traffic: after we bump TDT the card processes the
//! descriptor and — because we set RS — writes back its DD bit. Polling DD confirms the frame was
//! sent, entirely locally. (Under QEMU's SLIRP the frame reaches the host network stack, which drops
//! our experimental broadcast; nothing comes back until we speak a real protocol, in Stage 18.) The
//! demo transmits one minimum-length raw Ethernet frame and checks DD; consuming *received* frames
//! is the next sub-step.
//!
//! ## 17b-6: interrupt-driven receive
//!
//! Until now the driver *polls* the RX ring (`receive` reads the descriptor's Done bit). A real
//! driver instead lets the card tell it when a frame arrives, via an interrupt — the CPU does other
//! work and is only pulled in when there is a packet. Three pieces:
//!
//! 1. **Route the card's IRQ.** The e1000 raises a PCI interrupt line; the IO-APIC must forward it
//!    to a CPU vector (`apic::route_pci_irq(irq, E1000_VECTOR)`). Unlike the keyboard's ISA IRQ, a
//!    PCI interrupt is **level-triggered and active-low**: the card holds the line asserted until
//!    the driver clears the cause, so the redirection entry must say "level" or the interrupt is
//!    mishandled.
//! 2. **Arm the card.** Writing the receive causes (RXT0 | RXDMT0) to the Interrupt Mask Set
//!    register (IMS) tells the card to actually assert its line when a frame lands.
//! 3. **Handle it.** The interrupt handler ([`on_interrupt`]) *reads ICR* first — that returns and
//!    clears the pending causes, de-asserting the (level-triggered) line, without which the
//!    interrupt would re-fire forever — then drains every ready frame from the ring. It must use a
//!    `try_lock`: a handler may never block on a lock the interrupted code holds, so if the device
//!    is momentarily busy it clears the cause and returns, leaving frames in the ring for later.
//!
//! We prove it with loopback again ([`interrupt_selftest`]), but this time we do *not* poll: we send
//! a frame to ourselves and wait for the handler to have drained it. The one trick is that QEMU
//! delivers a looped frame synchronously during the transmit's doorbell write, so the transmit runs
//! with interrupts disabled — the raised IRQ stays pending until we drop the device lock and
//! re-enable interrupts, at which point the handler takes the lock cleanly (rather than deadlocking
//! against the transmit that still holds it, on our single boot core).

use core::sync::atomic::{AtomicU64, AtomicUsize, Ordering};

use spin::Mutex;
use x86_64::{
    instructions::interrupts::without_interrupts,
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
/// Interrupt Mask Set — writing a 1 to a bit enables (unmasks) that interrupt cause (Stage 17b-6).
const REG_IMS: u64 = 0x00D0;
/// Interrupt Mask Clear — writing a 1 to a bit disables (masks) that interrupt cause.
const REG_IMC: u64 = 0x00D8;

// --- Interrupt cause / mask bits (shared by ICR, IMS, IMC), Stage 17b-6 ---

/// Receive Descriptor Minimum Threshold hit — the free-descriptor count fell below RCTL's
/// threshold (the ring is filling up). We enable it alongside RXT0 as a second "frames waiting"
/// signal.
const ICR_RXDMT0: u32 = 1 << 4;
/// Receiver Timer interrupt — a frame has been received (with the receive delay timer at 0, as
/// after reset, the card raises this immediately per frame). This is the "a packet arrived" cause.
const ICR_RXT0: u32 = 1 << 7;
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

// --- Transmit-path registers (Stage 17b-4) ---

/// Transmit Control: enable the transmitter, pad short packets, and set collision parameters.
const REG_TCTL: u64 = 0x0400;
/// Transmit Inter-Packet Gap timing.
const REG_TIPG: u64 = 0x0410;
/// Transmit Descriptor Base Address Low — low 32 bits of the ring's physical address.
const REG_TDBAL: u64 = 0x3800;
/// Transmit Descriptor Base Address High — high 32 bits of the ring's physical address.
const REG_TDBAH: u64 = 0x3804;
/// Transmit Descriptor Length — the ring's size in bytes (a multiple of 128).
const REG_TDLEN: u64 = 0x3808;
/// Transmit Descriptor Head — the descriptor the card transmits next (the card advances it).
const REG_TDH: u64 = 0x3810;
/// Transmit Descriptor Tail — one past the last descriptor the driver has handed the card.
const REG_TDT: u64 = 0x3818;

// --- PHY access via MDIC (Stage 17b-5) ---

/// MDI Control — reads/writes the internal PHY's MII registers indirectly (the PHY is not itself
/// memory-mapped): write an opcode + PHY address + register address (+ data), then poll the Ready
/// bit; a read leaves the value in the low 16 bits.
const REG_MDIC: u64 = 0x0020;

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

// --- TCTL (Transmit Control) bits, and TX descriptor command/status bits (Stage 17b-4) ---

/// Transmitter Enable.
const TCTL_EN: u32 = 1 << 1;
/// Pad Short Packets — the card pads a frame below 60 bytes up to the Ethernet minimum.
const TCTL_PSP: u32 = 1 << 3;
/// Collision Threshold occupies TCTL bits 4-11; Collision Distance bits 12-21.
const TCTL_CT_SHIFT: u32 = 4;
const TCTL_COLD_SHIFT: u32 = 12;
/// Collision Threshold: the manual's recommended value.
const TCTL_CT: u32 = 0x0F;
/// Collision Distance: the recommended full-duplex value (QEMU's model ignores it).
const TCTL_COLD: u32 = 0x40;
/// Transmit Inter-Packet Gap: the 82540EM's recommended IPGT=10, IPGR1=8, IPGR2=6
/// (10 | 8<<10 | 6<<20).
const TIPG_DEFAULT: u32 = 0x0060_200A;

/// TX command bit: End Of Packet — this descriptor is the last of the frame.
const TXD_CMD_EOP: u8 = 1 << 0;
/// TX command bit: Insert FCS — the card appends the 4-byte Ethernet CRC.
const TXD_CMD_IFCS: u8 = 1 << 1;
/// TX command bit: Report Status — the card writes back the Descriptor-Done bit when finished.
const TXD_CMD_RS: u8 = 1 << 3;
/// TX status bit: Descriptor Done — the card has transmitted this descriptor.
const TXD_STAT_DD: u8 = 1 << 0;

/// Number of transmit descriptors in the ring. 8 x 16 B = 128 B — exactly the minimum TDLEN
/// alignment, and it fits in a single 4 KiB frame.
const TX_DESC_COUNT: usize = 8;
/// Bytes in each transmit buffer — large enough for a full 1518-byte Ethernet frame; one per frame.
const TX_BUFFER_SIZE: usize = 2048;

/// A legacy transmit descriptor: 16 bytes the driver fills to hand the card a frame to send.
///
/// The driver writes `addr` (the physical address of a frame buffer), `length`, and `cmd`
/// (EOP | IFCS | RS) and clears `status`; the card writes back `status`'s Descriptor-Done (DD) bit
/// when it has transmitted the frame. `#[repr(C)]` pins the exact 16-byte layout (8 + 2 + 1 + 1 + 1
/// + 1 + 2 = 16, no padding).
#[repr(C)]
#[derive(Clone, Copy)]
struct TxDesc {
    /// Physical address of this descriptor's frame buffer (the card's DMA source).
    addr: u64,
    /// Length of the frame to send (without the CRC the card appends).
    length: u16,
    /// Checksum offset (unused here).
    cso: u8,
    /// Command bits: EOP | IFCS | RS.
    cmd: u8,
    /// Status bits (bit 0 = Descriptor Done), written by the card.
    status: u8,
    /// Checksum start (unused here).
    css: u8,
    /// VLAN / special field (unused here).
    special: u16,
}

// --- MDIC fields, PHY registers, and RX descriptor status bits (Stage 17b-5) ---

/// MDIC opcode: write the addressed PHY register.
const MDIC_OP_WRITE: u32 = 0x0400_0000;
/// MDIC opcode: read the addressed PHY register.
const MDIC_OP_READ: u32 = 0x0800_0000;
/// MDIC Ready bit — the card sets it when the PHY access completes.
const MDIC_READY: u32 = 0x1000_0000;
/// MDIC PHY-address field shift; the e1000's internal PHY answers at address 1.
const MDIC_PHY_SHIFT: u32 = 21;
const MDIC_PHY_ADDR: u32 = 1;
/// MDIC register-address field shift.
const MDIC_REG_SHIFT: u32 = 16;
/// PHY register 0: the Basic Mode Control Register (BMCR / MII control).
const PHY_BMCR: u32 = 0;
/// BMCR loopback bit — the PHY loops transmitted frames back into the receiver (QEMU preserves it).
const BMCR_LOOPBACK: u16 = 0x4000;

/// RX descriptor status bit: Descriptor Done — the card has filled this descriptor with a frame.
const RXD_STAT_DD: u8 = 1 << 0;

/// The initialized card, once [`init`] has mapped its registers, reset it, and read its identity.
/// Stored behind a global so later sub-steps (and the tests) can reach the one card without
/// threading a handle through boot. The fields are plain data, so this is safe to move into a
/// `Mutex`.
static DEVICE: Mutex<Option<E1000>> = Mutex::new(None);

/// The card's mapped MMIO base, published separately from [`DEVICE`] (Stage 17b-6). The interrupt
/// handler needs to read/clear ICR *without* taking the device `Mutex` (it may not block on a lock
/// the interrupted code holds), so `init` stores the base here where the handler can reach it lock-free.
/// Zero until the card is up.
static MMIO_BASE: AtomicU64 = AtomicU64::new(0);

/// How many times the e1000 receive interrupt handler ran and saw a receive cause (Stage 17b-6).
static RX_IRQ_COUNT: AtomicU64 = AtomicU64::new(0);
/// How many frames the interrupt handler drained from the RX ring (Stage 17b-6). Distinct from
/// [`RX_IRQ_COUNT`] because one interrupt can cover several arrived frames.
static RX_FRAMES_VIA_IRQ: AtomicU64 = AtomicU64::new(0);
/// Length of the most recent frame the interrupt handler drained (Stage 17b-6), for the self-test.
static LAST_RX_LEN: AtomicUsize = AtomicUsize::new(0);

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
    /// Virtual address (through the physical-memory window) of the transmit descriptor ring. Zero
    /// until [`setup_tx`](Self::setup_tx) runs.
    tx_ring: u64,
    /// Physical address of the transmit descriptor ring — the value programmed into TDBAL/TDBAH.
    tx_ring_phys: u64,
    /// Index of the next receive descriptor the driver expects the card to fill — the software-side
    /// cursor into the RX ring, advanced by [`receive`](Self::receive).
    rx_cur: u16,
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

    /// Stage 17b-4: build the transmit descriptor ring and enable the transmitter.
    ///
    /// Allocates one frame for the ring and one per transmit buffer, pre-fills each descriptor with
    /// its buffer's physical address (idle — command/status zero), programs the ring's
    /// base/length/head/tail, and enables the transmitter in TCTL. Like the RX ring, the ring and
    /// buffers are normal cacheable RAM reached through the physical-memory window (x86 DMA is
    /// cache-coherent), and the frames are never freed.
    ///
    /// # Safety
    ///
    /// The MMIO block must be mapped and the physical-memory offset installed (true after
    /// `memory::init`). Runs during bring-up, before any concurrent transmit.
    unsafe fn setup_tx(&mut self, frame_allocator: &mut impl FrameAllocator<Size4KiB>) {
        let phys_offset = crate::memory::physical_memory_offset().as_u64();

        // One frame for the descriptor ring (128 B of descriptors).
        let ring_frame = frame_allocator
            .allocate_frame()
            .expect("no free frame for the e1000 TX descriptor ring");
        self.tx_ring_phys = ring_frame.start_address().as_u64();
        self.tx_ring = phys_offset + self.tx_ring_phys;

        // Give each descriptor its own buffer and leave it idle (command/status zero, so head==tail
        // means the ring is empty and the card transmits nothing yet).
        for i in 0..TX_DESC_COUNT {
            let buf = frame_allocator
                .allocate_frame()
                .expect("no free frame for an e1000 TX buffer");
            let desc = TxDesc {
                addr: buf.start_address().as_u64(),
                length: 0,
                cso: 0,
                cmd: 0,
                status: 0,
                css: 0,
                special: 0,
            };
            // SAFETY: `tx_ring + i*16` is the i-th 16-byte descriptor slot of the ring frame, inside
            // the physical-memory window and 8-byte aligned; the write is volatile.
            ((self.tx_ring + (i * 16) as u64) as *mut TxDesc).write_volatile(desc);
        }

        // Point the card at the ring, size it, and start head/tail at 0 (empty ring).
        self.write_reg(REG_TDBAL, self.tx_ring_phys as u32);
        self.write_reg(REG_TDBAH, (self.tx_ring_phys >> 32) as u32);
        self.write_reg(REG_TDLEN, (TX_DESC_COUNT * 16) as u32);
        self.write_reg(REG_TDH, 0);
        self.write_reg(REG_TDT, 0);

        // Inter-packet gap, then enable the transmitter: pad short packets to the Ethernet minimum,
        // and set the recommended collision threshold/distance (QEMU's model ignores the latter).
        self.write_reg(REG_TIPG, TIPG_DEFAULT);
        let tctl = TCTL_EN | TCTL_PSP | (TCTL_CT << TCTL_CT_SHIFT) | (TCTL_COLD << TCTL_COLD_SHIFT);
        self.write_reg(REG_TCTL, tctl);
    }

    /// Transmit one Ethernet frame and wait (bounded) for the card to confirm it. Copies `frame`
    /// into the tail descriptor's buffer, sets the descriptor (EOP | IFCS | RS), advances TDT to
    /// hand it to the card, then polls the Descriptor-Done bit. Returns whether DD was set within
    /// the timeout. The card appends the CRC (IFCS) and pads short frames (TCTL.PSP).
    ///
    /// # Safety
    ///
    /// The MMIO block must be mapped and [`setup_tx`](Self::setup_tx) must have run. `frame` must fit
    /// in a transmit buffer.
    unsafe fn transmit(&mut self, frame: &[u8]) -> bool {
        assert!(frame.len() <= TX_BUFFER_SIZE, "frame too large for an e1000 TX buffer");
        let phys_offset = crate::memory::physical_memory_offset().as_u64();

        // Use the descriptor the tail currently points at.
        let tail = self.read_reg(REG_TDT) as usize;
        let desc_ptr = (self.tx_ring + (tail * 16) as u64) as *mut TxDesc;

        // The descriptor's buffer, pre-assigned in `setup_tx`. The card preserves `addr` on
        // writeback (it only writes the status byte), so reading it back is valid.
        let buf_phys = (desc_ptr as *const TxDesc).read_volatile().addr;
        let buf_virt = (phys_offset + buf_phys) as *mut u8;
        // Copy the frame into the buffer (source and destination never overlap).
        core::ptr::copy_nonoverlapping(frame.as_ptr(), buf_virt, frame.len());

        // Fill the descriptor: length, command (end-of-packet, insert CRC, report status), DD clear.
        let desc = TxDesc {
            addr: buf_phys,
            length: frame.len() as u16,
            cso: 0,
            cmd: TXD_CMD_EOP | TXD_CMD_IFCS | TXD_CMD_RS,
            status: 0,
            css: 0,
            special: 0,
        };
        desc_ptr.write_volatile(desc);

        // Advance the tail (wrapping) to hand the descriptor to the card. The volatile descriptor
        // write above is ordered before this on x86 (TSO), so the card sees a complete descriptor.
        let next = ((tail + 1) % TX_DESC_COUNT) as u32;
        self.write_reg(REG_TDT, next);

        // Poll the Descriptor-Done bit (bounded, ~10 ms) so a stuck card cannot hang the caller.
        for _ in 0..1000 {
            if (desc_ptr as *const TxDesc).read_volatile().status & TXD_STAT_DD != 0 {
                return true;
            }
            apic::pit_sleep_us(10);
        }
        false
    }

    /// Physical address of the transmit descriptor ring (what we programmed into TDBAL/TDBAH).
    pub fn tx_ring_phys(&self) -> u64 {
        self.tx_ring_phys
    }

    /// Number of descriptors in the transmit ring.
    pub fn tx_count(&self) -> usize {
        TX_DESC_COUNT
    }

    /// Transmit Descriptor Base Address read back off the card (TDBAH:TDBAL).
    pub fn tx_descriptor_base(&self) -> u64 {
        // SAFETY: `init` mapped the MMIO block; TDBAL/TDBAH are valid registers within it.
        unsafe { (u64::from(self.read_reg(REG_TDBAH)) << 32) | u64::from(self.read_reg(REG_TDBAL)) }
    }

    /// Transmit Descriptor Length in bytes, read back off the card (TDLEN).
    pub fn tx_descriptor_len(&self) -> u32 {
        // SAFETY: `init` mapped the MMIO block; TDLEN is a valid register within it.
        unsafe { self.read_reg(REG_TDLEN) }
    }

    /// Transmit Descriptor Head index, read live (the card advances it as it transmits).
    pub fn tx_head(&self) -> u32 {
        // SAFETY: `init` mapped the MMIO block; TDH is a valid register within it.
        unsafe { self.read_reg(REG_TDH) }
    }

    /// Transmit Descriptor Tail index, read live.
    pub fn tx_tail(&self) -> u32 {
        // SAFETY: `init` mapped the MMIO block; TDT is a valid register within it.
        unsafe { self.read_reg(REG_TDT) }
    }

    /// Whether the transmitter is enabled (TCTL.EN), read live from the card.
    pub fn transmitter_enabled(&self) -> bool {
        // SAFETY: `init` mapped the MMIO block; TCTL is a valid register within it.
        unsafe { self.read_reg(REG_TCTL) & TCTL_EN != 0 }
    }

    /// Whether the TX ring is correctly installed: the card's descriptor base and length read back as
    /// what we programmed, and the transmitter is enabled. One read-back check for the log and test.
    pub fn tx_ring_installed(&self) -> bool {
        self.tx_descriptor_base() == self.tx_ring_phys
            && self.tx_descriptor_len() as usize == TX_DESC_COUNT * 16
            && self.transmitter_enabled()
    }

    /// Read PHY register `reg` through the MDI Control register. The internal PHY is address 1.
    /// Bounded poll for the Ready bit; returns the 16-bit value (0 on timeout).
    ///
    /// # Safety
    ///
    /// The MMIO block must be mapped.
    unsafe fn phy_read(&self, reg: u32) -> u16 {
        self.write_reg(
            REG_MDIC,
            (reg << MDIC_REG_SHIFT) | (MDIC_PHY_ADDR << MDIC_PHY_SHIFT) | MDIC_OP_READ,
        );
        for _ in 0..1000 {
            let mdic = self.read_reg(REG_MDIC);
            if mdic & MDIC_READY != 0 {
                return mdic as u16;
            }
            apic::pit_sleep_us(1);
        }
        0
    }

    /// Write `data` to PHY register `reg` through the MDI Control register. Bounded poll for Ready.
    ///
    /// # Safety
    ///
    /// The MMIO block must be mapped.
    unsafe fn phy_write(&self, reg: u32, data: u16) {
        self.write_reg(
            REG_MDIC,
            u32::from(data)
                | (reg << MDIC_REG_SHIFT)
                | (MDIC_PHY_ADDR << MDIC_PHY_SHIFT)
                | MDIC_OP_WRITE,
        );
        for _ in 0..1000 {
            if self.read_reg(REG_MDIC) & MDIC_READY != 0 {
                return;
            }
            apic::pit_sleep_us(1);
        }
    }

    /// Enable or disable PHY loopback: with it on, transmitted frames are looped straight back into
    /// the receiver instead of going out on the wire — the way to exercise the RX path with no
    /// external traffic. Read-modify-writes the PHY BMCR so the other control bits survive.
    ///
    /// # Safety
    ///
    /// The MMIO block must be mapped.
    unsafe fn set_loopback(&self, enable: bool) {
        let mut bmcr = self.phy_read(PHY_BMCR);
        if enable {
            bmcr |= BMCR_LOOPBACK;
        } else {
            bmcr &= !BMCR_LOOPBACK;
        }
        self.phy_write(PHY_BMCR, bmcr);
    }

    /// Poll the current receive descriptor; if the card has filled it (Descriptor Done), copy the
    /// frame into `buf`, recycle the descriptor back to the card, advance the software cursor, and
    /// return the frame's length. Returns `None` if no frame is ready. Non-blocking — checks one
    /// descriptor and returns.
    ///
    /// # Safety
    ///
    /// The MMIO block must be mapped and [`setup_rx`](Self::setup_rx) must have run.
    unsafe fn receive(&mut self, buf: &mut [u8]) -> Option<usize> {
        let phys_offset = crate::memory::physical_memory_offset().as_u64();
        let idx = self.rx_cur as usize;
        let desc_ptr = (self.rx_ring + (idx * 16) as u64) as *mut RxDesc;
        let desc = (desc_ptr as *const RxDesc).read_volatile();
        if desc.status & RXD_STAT_DD == 0 {
            return None; // the card has not filled this descriptor yet
        }
        let len = desc.length as usize;
        let n = len.min(buf.len());
        // Copy the received frame out of the DMA buffer (reached through the physical-memory window).
        core::ptr::copy_nonoverlapping((phys_offset + desc.addr) as *const u8, buf.as_mut_ptr(), n);

        // Recycle: clear Descriptor Done so a re-poll of this slot does not see the old frame, then
        // hand the descriptor back to the card by moving the tail onto it, and advance our cursor.
        desc_ptr.write_volatile(RxDesc { status: 0, ..desc });
        self.write_reg(REG_RDT, idx as u32);
        self.rx_cur = ((idx + 1) % RX_DESC_COUNT) as u16;
        Some(len)
    }

    /// Stage 17b-6: arm the card's receive interrupt. Writing the receive causes (RXT0, a frame
    /// arrived; RXDMT0, the ring is filling) to the Interrupt Mask Set register tells the card to
    /// assert its IRQ line on those events. The IO-APIC must already route that line to a handled
    /// vector ([`crate::interrupts::E1000_VECTOR`]) before this is called, or the card would assert
    /// a line nothing listens to.
    ///
    /// # Safety
    ///
    /// The MMIO block must be mapped. Unmasking a cause only enables an interrupt source; the handler
    /// for the routed vector must be registered first.
    unsafe fn enable_rx_interrupt(&self) {
        self.write_reg(REG_IMS, ICR_RXT0 | ICR_RXDMT0);
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

    // Stage 17b-6: publish the MMIO base so the interrupt handler can read/clear ICR without taking
    // the device lock. Release-ordered so a handler that later sees a non-zero base also sees the
    // mapping established.
    MMIO_BASE.store(E1000_VIRT_BASE, Ordering::Release);

    // Enable PCI bus mastering before any DMA: the descriptor rings and packet buffers are all DMA,
    // and the card cannot touch host memory until this is set (the command register resets to 0 at
    // power-on). Without it the card silently reads/writes zeros — descriptors look processed but
    // nothing is transmitted and no Done status is written back.
    nic.enable_bus_mastering();

    let mut dev = E1000 {
        mmio_base: E1000_VIRT_BASE,
        mac: [0; 6],
        reset_ok: false,
        rx_ring: 0,
        rx_ring_phys: 0,
        tx_ring: 0,
        tx_ring_phys: 0,
        rx_cur: 0,
    };

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

    // Stage 17b-4: build the transmit descriptor ring and enable the transmitter.
    // SAFETY: same conditions as `setup_rx` — the MMIO block is mapped and the frame allocator and
    // physical-memory window are available.
    unsafe { dev.setup_tx(frame_allocator) };

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
    serial_println!(
        "[e1000] TX ring: {} descriptors @ phys {:#x} (TDBA {:#x}, TDLEN {}, TDH {}, TDT {}), transmitter {}",
        dev.tx_count(),
        dev.tx_ring_phys(),
        dev.tx_descriptor_base(),
        dev.tx_descriptor_len(),
        dev.tx_head(),
        dev.tx_tail(),
        if dev.transmitter_enabled() { "enabled" } else { "DISABLED" },
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

/// Stage 17b-4: transmit one raw Ethernet frame through the e1000, if it is up. Returns whether the
/// card confirmed the transmit (set the descriptor's Done bit) within the timeout.
pub fn transmit(frame: &[u8]) -> bool {
    let mut guard = DEVICE.lock();
    match guard.as_mut() {
        // SAFETY: the device is initialized, so its MMIO block and TX ring are set up; `transmit`
        // copies the frame into a TX buffer and hands the descriptor to the card. Interrupts are
        // masked at the card and a syscall/boot context holds no other e1000 lock, so this cannot
        // deadlock against a handler.
        Some(dev) => unsafe { dev.transmit(frame) },
        None => false,
    }
}

/// Stage 17b-4 boot demo / test helper: build a canonical raw Ethernet frame — broadcast
/// destination, our own MAC as source, an experimental ethertype, and an ASCII payload — and
/// transmit it. Returns whether the card confirmed the transmit.
pub fn transmit_test_frame() -> bool {
    let mac = match device() {
        Some(dev) => dev.mac,
        None => return false,
    };
    // A minimum-length (60-byte) Ethernet frame; the card appends the 4-byte CRC (IFCS) itself.
    let mut frame = [0u8; 60];
    frame[0..6].copy_from_slice(&[0xFF; 6]); // destination: broadcast
    frame[6..12].copy_from_slice(&mac); // source: our MAC
    frame[12] = 0x88; // ethertype 0x88B5 (IEEE 802 local-experimental), big-endian
    frame[13] = 0xB5;
    let msg = b"aether e1000 raw tx test";
    frame[14..14 + msg.len()].copy_from_slice(msg);
    transmit(&frame)
}

/// Stage 17b-5: prove the receive path with PHY loopback. Turn loopback on so a transmitted frame is
/// looped straight back into the receiver, send a frame addressed to our own MAC (accepted by the RX
/// filter via Receive Address 0), receive it off the RX ring, and confirm the bytes round-trip. Turns
/// loopback back off before returning. Returns whether the frame came back intact.
pub fn loopback_selftest() -> bool {
    let mut guard = DEVICE.lock();
    let dev = match guard.as_mut() {
        Some(d) => d,
        None => return false,
    };
    let mac = dev.mac;

    // A 60-byte frame addressed to ourselves (accepted by the RX filter via Receive Address 0).
    let mut frame = [0u8; 60];
    frame[0..6].copy_from_slice(&mac); // destination: ourselves
    frame[6..12].copy_from_slice(&mac); // source: ourselves
    frame[12] = 0x88; // ethertype 0x88B5 (IEEE 802 local-experimental)
    frame[13] = 0xB5;
    let msg = b"aether e1000 loopback rx test";
    frame[14..14 + msg.len()].copy_from_slice(msg);

    // SAFETY: the device is initialized — MMIO mapped, both rings set up, bus mastering enabled.
    unsafe {
        let mut rxbuf = [0u8; 2048];
        // Drain anything already sitting in the RX ring (e.g. delivered late from a prior call), so
        // we only inspect the frame we loop back now.
        while dev.receive(&mut rxbuf).is_some() {}

        dev.set_loopback(true);

        // Retry until the frame comes back. Under QEMU the e1000's receiver is not ready to accept
        // frames until the link has settled (up to ~1 s of boot time on the first run); a frame sent
        // before then is silently dropped rather than looped back. So resend, watching RDH advance as
        // the fast "did the card accept a frame" check. Bounded so a link that never comes up cannot
        // hang boot.
        let mut result = None;
        for _ in 0..1000 {
            let rdh_before = dev.read_reg(REG_RDH);
            let _ = dev.transmit(&frame);
            // The loopback delivery is synchronous under QEMU, so a received frame shows up in RDH
            // immediately; if RDH moved, consume it.
            if dev.read_reg(REG_RDH) != rdh_before {
                if let Some(len) = dev.receive(&mut rxbuf) {
                    result = Some(len);
                    break;
                }
            }
            apic::pit_sleep_us(2000);
        }
        dev.set_loopback(false);

        match result {
            Some(len) => {
                let ok = len >= frame.len() && rxbuf[..frame.len()] == frame;
                serial_println!(
                    "[e1000] loopback: sent a {} B frame to self, received {} B, match = {}",
                    frame.len(),
                    len,
                    ok,
                );
                ok
            }
            None => {
                serial_println!("[e1000] loopback: no frame looped back (link never became ready)");
                false
            }
        }
    }
}

/// Stage 17b-6: switch the receive path from polling to interrupts. Route the card's PCI IRQ line
/// through the IO-APIC to the handled e1000 vector, then arm the card's receive interrupt (IMS), so
/// a received frame raises an interrupt that [`on_interrupt`] services. `irq` is the card's
/// `interrupt_line` from PCI config space. Order matters: the IDT gate and the IO-APIC route are put
/// in place before the card is told (via IMS) to assert its line.
pub fn enable_interrupts(irq: u8) {
    // Route the card's IRQ line to our vector first (the handler is already in the IDT).
    apic::route_pci_irq(irq, crate::interrupts::E1000_VECTOR);
    let guard = DEVICE.lock();
    if let Some(dev) = guard.as_ref() {
        // SAFETY: the device is initialized (MMIO mapped); IMS is a valid register and setting the
        // receive-cause bits only unmasks those interrupt sources. The E1000_VECTOR handler is
        // registered and the IO-APIC route is now in place, so the asserted line has a destination.
        unsafe { dev.enable_rx_interrupt() };
    }
    serial_println!("[e1000] receive interrupt armed (IRQ{} -> IO-APIC, IMS set)", irq);
}

/// Stage 17b-6: the receive-interrupt bottom half, called from the IDT handler
/// (`interrupts::e1000_interrupt_handler`). Reads and clears the card's interrupt cause — mandatory
/// for a level-triggered PCI interrupt, or the line stays asserted and the interrupt re-fires
/// endlessly — and, if a receive cause is set, drains every ready frame from the RX ring.
///
/// Draining uses `try_lock`: an interrupt handler must never *block* on a lock the code it
/// interrupted might hold (a single-core deadlock). If the device is momentarily locked elsewhere
/// (e.g. a polled transmit), we still clear the cause and return; the frames stay in the ring
/// (Descriptor Done set) for the next drain. Runs in interrupt context, so it only counts and
/// measures frames — a real driver would hand each up to the network stack (Stage 18).
pub fn on_interrupt() {
    let base = MMIO_BASE.load(Ordering::Acquire);
    if base == 0 {
        return; // the card is not up yet
    }
    // Read ICR: returns the pending causes and clears them, de-asserting the card's IRQ line.
    // SAFETY: `base` is the mapped, uncacheable MMIO base published by `init`; ICR (0x00C0) is a
    // valid register whose read is exactly what clears the pending cause.
    let icr = unsafe { ((base + REG_ICR) as *const u32).read_volatile() };
    if icr & (ICR_RXT0 | ICR_RXDMT0) == 0 {
        return; // not a receive cause (e.g. a link-status change) — nothing to drain
    }
    RX_IRQ_COUNT.fetch_add(1, Ordering::Relaxed);

    if let Some(mut guard) = DEVICE.try_lock() {
        if let Some(dev) = guard.as_mut() {
            let mut buf = [0u8; 2048];
            // SAFETY: the device is initialized (MMIO mapped, RX ring set up); `receive` reads the
            // next ready descriptor and recycles it. Drain every ready frame — one interrupt can
            // cover several arrivals — recording the last length and counting each.
            while let Some(len) = unsafe { dev.receive(&mut buf) } {
                LAST_RX_LEN.store(len, Ordering::Relaxed);
                RX_FRAMES_VIA_IRQ.fetch_add(1, Ordering::Release);
            }
        }
    }
}

/// Number of times the e1000 receive interrupt fired with a receive cause (Stage 17b-6).
pub fn rx_irq_count() -> u64 {
    RX_IRQ_COUNT.load(Ordering::Relaxed)
}

/// Number of frames drained from the RX ring by the interrupt handler (Stage 17b-6).
pub fn rx_frames_via_irq() -> u64 {
    RX_FRAMES_VIA_IRQ.load(Ordering::Acquire)
}

/// Length of the most recent frame the interrupt handler drained (Stage 17b-6).
pub fn last_rx_len() -> usize {
    LAST_RX_LEN.load(Ordering::Relaxed)
}

/// Stage 17b-6: prove interrupt-driven receive. With the card's RX interrupt armed and its IRQ
/// routed (see [`enable_interrupts`]), enable PHY loopback and send a frame to our own MAC; the card
/// loops it back and raises the receive interrupt, whose handler ([`on_interrupt`]) drains it from
/// the ring — this path never polls. Returns whether the interrupt fired and delivered our frame.
///
/// The measured transmit runs under `without_interrupts` so the IRQ raised during the doorbell write
/// stays pending until the device lock is dropped; on re-enable the handler takes the lock cleanly (a
/// single-core handler would otherwise deadlock against the transmit still holding it). Bounded
/// resends cover the case where QEMU's receiver is not yet ready (the looped frame is dropped and no
/// interrupt fires), and stale frames are drained first so the counters reflect only our frame.
pub fn interrupt_selftest() -> bool {
    let mac = match device() {
        Some(dev) => dev.mac,
        None => return false,
    };
    // A 60-byte frame addressed to ourselves (accepted by the RX filter via Receive Address 0).
    let mut frame = [0u8; 60];
    frame[0..6].copy_from_slice(&mac); // destination: ourselves
    frame[6..12].copy_from_slice(&mac); // source: ourselves
    frame[12] = 0x88; // ethertype 0x88B5 (IEEE 802 local-experimental)
    frame[13] = 0xB5;
    let msg = b"aether e1000 irq rx test";
    frame[14..14 + msg.len()].copy_from_slice(msg);

    // Enable loopback and drain anything already in the ring, so the counters below reflect only the
    // frame we send.
    {
        let mut guard = DEVICE.lock();
        let dev = match guard.as_mut() {
            Some(d) => d,
            None => return false,
        };
        // SAFETY: the device is initialized — MMIO mapped, RX ring set up.
        unsafe {
            dev.set_loopback(true);
            let mut sink = [0u8; 2048];
            while dev.receive(&mut sink).is_some() {}
        }
    }
    RX_IRQ_COUNT.store(0, Ordering::Relaxed);
    RX_FRAMES_VIA_IRQ.store(0, Ordering::Relaxed);
    LAST_RX_LEN.store(0, Ordering::Relaxed);

    // Send, then wait for the handler to drain our frame. Resend (bounded) until QEMU's receiver is
    // ready to loop it back.
    let mut delivered = false;
    for _ in 0..1000 {
        // Transmit with interrupts off: the IRQ raised during the doorbell write is delivered only
        // after the lock is dropped and interrupts are re-enabled, so the handler cannot deadlock
        // against this transmit.
        without_interrupts(|| {
            let mut guard = DEVICE.lock();
            if let Some(dev) = guard.as_mut() {
                // SAFETY: device initialized; `transmit` copies the frame in and rings the doorbell.
                unsafe {
                    let _ = dev.transmit(&frame);
                }
            }
        });
        // Interrupts are back on; a pending IRQ (if the frame was accepted) is delivered before the
        // spin below. Give the handler a bounded moment to run.
        for _ in 0..100_000 {
            if RX_FRAMES_VIA_IRQ.load(Ordering::Acquire) > 0 {
                delivered = true;
                break;
            }
            core::hint::spin_loop();
        }
        if delivered {
            break;
        }
        // Not yet — the receiver is still settling; wait and resend.
        apic::pit_sleep_us(2000);
    }

    // Restore normal (non-loopback) operation.
    {
        let guard = DEVICE.lock();
        if let Some(dev) = guard.as_ref() {
            // SAFETY: device initialized.
            unsafe { dev.set_loopback(false) };
        }
    }

    let len = LAST_RX_LEN.load(Ordering::Relaxed);
    let ok = delivered && len == frame.len();
    serial_println!(
        "[e1000] interrupt receive: irqs {}, frames drained {}, last len {}, match = {}",
        RX_IRQ_COUNT.load(Ordering::Relaxed),
        RX_FRAMES_VIA_IRQ.load(Ordering::Relaxed),
        len,
        ok,
    );
    ok
}
