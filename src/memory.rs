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

use x86_64::registers::control::Cr3;
use x86_64::structures::paging::{OffsetPageTable, PageTable};
use x86_64::VirtAddr;

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
