//! Virtual memory: reaching and reading the page tables.
//!
//! Once the CPU is in long mode with paging enabled, every address the kernel
//! touches is a *virtual* address. On each memory access the CPU translates it to
//! a physical address by walking a 4-level page table, entirely in hardware:
//!
//!     virtual addr --> L4 (PML4) --> L3 (PDPT) --> L2 (PD) --> L1 (PT) --> frame
//!
//! Every level is a 512-entry table living in its own 4 KiB physical frame. An
//! entry holds the physical address of the next-level table (or, at L1, of the
//! mapped frame) plus permission bits. The CPU finds the top-level (L4) table
//! through the CR3 register.
//!
//! Reading these tables ourselves has a chicken-and-egg problem: the entries
//! store *physical* addresses, but the kernel can only dereference *virtual*
//! ones. The bootloader breaks the cycle for us. With the `map_physical_memory`
//! feature it maps the whole physical address space into virtual memory at a
//! constant offset, reported as `BootInfo::physical_memory_offset`. So the frame
//! at physical address P is readable at virtual address `offset + P`. That single
//! trick is what lets `active_level_4_table` turn the physical frame address in
//! CR3 into a reference we can actually follow.
//!
//! For now this module is read-only: it builds an `OffsetPageTable` and uses it
//! to translate addresses. Creating *new* mappings additionally needs a supply of
//! free physical frames (a frame allocator), which is the next sub-stage (4b).

use bootloader::bootinfo::{MemoryMap, MemoryRegionType};
use x86_64::registers::control::Cr3;
use x86_64::structures::paging::{
    FrameAllocator, Mapper, OffsetPageTable, Page, PageTable, PageTableFlags, PhysFrame, Size4KiB,
};
use x86_64::{PhysAddr, VirtAddr};

/// Build an [`OffsetPageTable`] over the page table that is currently active.
///
/// `OffsetPageTable` is the `x86_64` crate's high-level handle on an address
/// space whose physical memory is fully mapped at a constant offset. With it we
/// can translate addresses (the `Translate` trait) and, later, install new
/// mappings (the `Mapper` trait) without hand-walking four levels of tables.
///
/// # Safety
///
/// The caller must guarantee that **all** of physical memory is mapped into
/// virtual memory at `physical_memory_offset` — true here because we enabled the
/// bootloader's `map_physical_memory` feature. The function must also be called
/// **once**: it produces a `&mut` to the live level-4 table, and a second call
/// would alias that `&mut`, which is undefined behavior.
pub unsafe fn init(physical_memory_offset: VirtAddr) -> OffsetPageTable<'static> {
    let level_4_table = active_level_4_table(physical_memory_offset);
    // SAFETY: `level_4_table` is the active L4 table we just read from CR3, and
    // (per this function's contract) every physical frame is mapped at
    // `physical_memory_offset`, so the mapper can follow each lower-level table.
    OffsetPageTable::new(level_4_table, physical_memory_offset)
}

/// Return a mutable reference to the active level-4 (PML4) page table.
///
/// CR3 holds the *physical* frame of the active L4 table. We read it, add the
/// physical-memory offset to obtain a virtual address we can dereference, and
/// return a `'static` reference (the table lives as long as the kernel does).
///
/// # Safety
///
/// Same contract as [`init`]: all physical memory must be mapped at
/// `physical_memory_offset`, and because the returned `&mut` aliases the live
/// page table, the caller must not create a second reference to it.
unsafe fn active_level_4_table(physical_memory_offset: VirtAddr) -> &'static mut PageTable {
    // CR3 hands back the L4 table's frame (plus flags we don't need). Reading CR3
    // is safe; turning the physical address it yields into a dereferenceable
    // reference is the delicate part.
    let (level_4_table_frame, _) = Cr3::read();

    let phys = level_4_table_frame.start_address();
    let virt = physical_memory_offset + phys.as_u64();
    let page_table_ptr: *mut PageTable = virt.as_mut_ptr();

    // SAFETY: `virt` points at the L4 table because physical memory is mapped at
    // `physical_memory_offset`; the table outlives the kernel, so `'static` is
    // sound; and per the contract above the caller does not alias this `&mut`.
    &mut *page_table_ptr
}

// ---------------------------------------------------------------------------
// Frame allocation and creating new mappings (Stage 4b).
// ---------------------------------------------------------------------------

/// A frame allocator that hands out usable physical frames taken from the
/// bootloader's memory map, one at a time and never reused.
///
/// The bootloader surveys the machine's RAM and records each region's type in
/// `boot_info.memory_map`. We walk the regions marked `Usable`, chop them into
/// 4 KiB frames, and dole them out. This is the supply of free frames that
/// `Mapper::map_to` needs whenever it must create a missing intermediate page
/// table — and, in Stage 4c, the memory the heap will be built from.
pub struct BootInfoFrameAllocator {
    memory_map: &'static MemoryMap,
    next: usize,
}

impl BootInfoFrameAllocator {
    /// Create a frame allocator over the bootloader's memory map.
    ///
    /// # Safety
    ///
    /// The caller must guarantee that the regions the map marks `Usable` really
    /// are unused. If a frame that is already in use were handed out, two owners
    /// could write the same physical memory — undefined behavior.
    pub unsafe fn init(memory_map: &'static MemoryMap) -> Self {
        BootInfoFrameAllocator {
            memory_map,
            next: 0,
        }
    }

    /// Iterator over every usable 4 KiB frame described by the memory map.
    ///
    /// Rebuilt from scratch on each call; `allocate_frame` then skips the frames
    /// it has already handed out. That is quadratic, but simple, and fine for the
    /// handful of frames we allocate at boot.
    fn usable_frames(&self) -> impl Iterator<Item = PhysFrame> {
        let regions = self.memory_map.iter();
        let usable_regions = regions.filter(|r| r.region_type == MemoryRegionType::Usable);
        // Turn each usable region into its half-open range of physical addresses.
        let addr_ranges = usable_regions.map(|r| r.range.start_addr()..r.range.end_addr());
        // Walk each range one 4 KiB frame at a time.
        let frame_addresses = addr_ranges.flat_map(|r| r.step_by(4096));
        frame_addresses.map(|addr| PhysFrame::containing_address(PhysAddr::new(addr)))
    }
}

// SAFETY: the `FrameAllocator` trait is `unsafe` to implement because its callers
// (notably `map_to`) rely on every returned frame being unused. `usable_frames`
// yields only frames from `Usable` regions, and the `next` cursor only moves
// forward, so we never hand out the same frame twice.
unsafe impl FrameAllocator<Size4KiB> for BootInfoFrameAllocator {
    fn allocate_frame(&mut self) -> Option<PhysFrame> {
        let frame = self.usable_frames().nth(self.next);
        self.next += 1;
        frame
    }
}

/// Map `page` onto the VGA text-buffer frame (physical `0xb8000`), creating any
/// missing intermediate page tables from `frame_allocator`.
///
/// A Stage 4b demonstration: afterwards, writing through `page` reaches the same
/// physical memory as the screen. It is a *safe* function because the VGA frame
/// is device memory that is always sound to map writable, and aliasing it is the
/// whole point; a general "map any frame" helper would have to be `unsafe`.
pub fn create_example_mapping(
    page: Page,
    mapper: &mut OffsetPageTable,
    frame_allocator: &mut impl FrameAllocator<Size4KiB>,
) {
    let frame = PhysFrame::containing_address(PhysAddr::new(0xb8000));
    let flags = PageTableFlags::PRESENT | PageTableFlags::WRITABLE;

    // SAFETY: `frame` is the VGA text buffer — memory-mapped device memory that is
    // always valid to map writable, and deliberately aliased here (the screen is
    // also reachable at 0xb8000). `map_to` only pulls intermediate-table frames
    // from `frame_allocator`, which yields exclusively unused frames.
    let map_result = unsafe { mapper.map_to(page, frame, flags, frame_allocator) };
    map_result.expect("map_to failed").flush();
}
