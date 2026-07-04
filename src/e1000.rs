//! Stage 17b: the Intel e1000 (82540EM) network card driver.
//!
//! Stage 17a *found* the card on the PCI bus. Now we start *driving* it. Like the APIC, the e1000
//! is a memory-mapped device: it exposes a block of registers at the physical address advertised in
//! its BAR0, and we talk to it by reading and writing those registers. The card's real work —
//! sending and receiving Ethernet frames over DMA descriptor rings — comes in later sub-steps; this
//! one lays the foundation.
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
//! scattered across it — the receive/transmit descriptor and address registers a few sub-steps out
//! — are all reachable. Then, to prove register access works end to end, we read two things that
//! need no setup: the **Device Status** register (link state, speed, duplex) and the card's **MAC
//! address**, which QEMU's model has already loaded from its EEPROM into the Receive Address
//! registers by the time we boot.

use spin::Mutex;
use x86_64::{
    structures::paging::{FrameAllocator, Mapper, Page, PageTableFlags, PhysFrame, Size4KiB},
    PhysAddr, VirtAddr,
};

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
#[allow(dead_code)]
const REG_CTRL: u64 = 0x0000;
/// Device Status — link up, link speed, full-/half-duplex, and more.
const REG_STATUS: u64 = 0x0008;
/// Receive Address Low, entry 0: the low 4 bytes of the card's MAC address.
const REG_RAL0: u64 = 0x5400;
/// Receive Address High, entry 0: the high 2 bytes of the MAC, plus the Address Valid bit.
const REG_RAH0: u64 = 0x5404;

// --- selected register bits ---

/// STATUS bit 1: link is up.
const STATUS_LU: u32 = 1 << 1;
/// STATUS bit 0: the link is full-duplex (else half-duplex).
const STATUS_FD: u32 = 1 << 0;
/// RAH bit 31: Address Valid — the MAC in RAL/RAH is populated and usable.
const RAH_AV: u32 = 1 << 31;

/// The initialized card, once [`init`] has mapped its registers and read its identity. Stored
/// behind a global so later sub-steps (and the tests) can reach the one card without threading a
/// handle through boot. The fields are plain data (a virtual base address and the MAC bytes), so
/// this is safe to move into a `Mutex`.
static DEVICE: Mutex<Option<E1000>> = Mutex::new(None);

/// A handle on the e1000 NIC: the virtual base of its mapped register block, plus its MAC address
/// (read once at init). `Copy` so a caller can take a snapshot out of the global without holding
/// the lock while it works.
#[derive(Debug, Clone, Copy)]
pub struct E1000 {
    /// Virtual address of the mapped MMIO register block (== [`E1000_VIRT_BASE`]).
    mmio_base: u64,
    /// The card's 6-byte Ethernet MAC address.
    mac: [u8; 6],
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
    #[allow(dead_code)]
    unsafe fn write_reg(&self, offset: u64, value: u32) {
        ((self.mmio_base + offset) as *mut u32).write_volatile(value);
    }

    /// This card's MAC address.
    pub fn mac(&self) -> [u8; 6] {
        self.mac
    }

    /// Raw Device Status register (a live read from the card).
    pub fn status(&self) -> u32 {
        // SAFETY: `init` mapped the MMIO block, and STATUS (0x0008) is a valid, side-effect-free
        // status register within it.
        unsafe { self.read_reg(REG_STATUS) }
    }

    /// Whether the card reports its link as up.
    pub fn link_up(&self) -> bool {
        self.status() & STATUS_LU != 0
    }

    /// Whether the link is full-duplex.
    pub fn full_duplex(&self) -> bool {
        self.status() & STATUS_FD != 0
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

/// Bring up the e1000: map its register block and read its identity (MAC + status). Returns the
/// handle (also stashed in the global [`DEVICE`]), or `None` if the card's BAR0 is not a memory BAR
/// (it always is on QEMU).
///
/// Must run after paging and the frame allocator are up (it maps MMIO pages), and — since it reuses
/// the active kernel page tables — while the kernel address space is current.
pub fn init(
    nic: &pci::Device,
    mapper: &mut impl Mapper<Size4KiB>,
    frame_allocator: &mut impl FrameAllocator<Size4KiB>,
) -> Option<E1000> {
    let phys_base = nic.mmio_bar(0)?;
    map_mmio(phys_base, mapper, frame_allocator);

    let mut dev = E1000 { mmio_base: E1000_VIRT_BASE, mac: [0; 6] };

    // Read the MAC out of Receive Address entry 0. QEMU's e1000 model has already loaded it there
    // (from its emulated EEPROM) by power-on. RAL holds bytes 0..4 little-endian; RAH holds bytes
    // 4..6 in its low 16 bits, and bit 31 (AV) flags the entry as valid.
    // SAFETY: `map_mmio` just mapped the block, and RAL0/RAH0 (0x5400/0x5404) are valid
    // side-effect-free registers within it.
    let (ral, rah) = unsafe { (dev.read_reg(REG_RAL0), dev.read_reg(REG_RAH0)) };
    dev.mac = [
        ral as u8,
        (ral >> 8) as u8,
        (ral >> 16) as u8,
        (ral >> 24) as u8,
        rah as u8,
        (rah >> 8) as u8,
    ];

    let status = dev.status();
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
