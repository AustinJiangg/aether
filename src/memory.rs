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

use alloc::alloc::{alloc_zeroed, dealloc, handle_alloc_error, Layout};
use core::sync::atomic::{AtomicBool, AtomicU64, Ordering};

use bootloader::bootinfo::{MemoryMap, MemoryRegionType};
use spin::Mutex;
use x86_64::instructions::tlb;
use x86_64::registers::control::{Cr3, Cr3Flags};
use x86_64::structures::paging::{
    FrameAllocator, Mapper, OffsetPageTable, Page, PageTable, PageTableFlags, PhysFrame, Size4KiB,
    Translate,
};
use x86_64::{PhysAddr, VirtAddr};

use crate::serial_println;

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
    // Record the physical-memory-window base globally now, at the very start of
    // boot, so code far from `kernel_main` (an AP spawning guard-paged stacks, the
    // guard-page helpers below) can walk the page tables without threading the
    // offset through by hand. `install_kernel_allocator` stores the same value
    // again later; doing it here just makes it available from early boot on.
    PHYSICAL_MEMORY_OFFSET.store(physical_memory_offset.as_u64(), Ordering::SeqCst);

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

// ---------------------------------------------------------------------------
// A globally reachable kernel frame allocator (Stage 12d).
// ---------------------------------------------------------------------------
//
// Until now the frame allocator and the physical-memory offset were locals in
// `kernel_main`, threaded by reference into the boot-time setup. That is fine while
// only boot code allocates frames. The `spawn` syscall changes that: a *user* program
// asks the kernel to load another program at runtime, so the ELF loader must allocate
// frames from deep inside a trap handler — code that has no way to borrow `kernel_main`'s
// locals. So we stash the frame allocator and the offset in globals, installed once at
// the end of boot (after the boot-time allocations, before the first user process runs).
// A syscall runs with interrupts disabled, so nothing can preempt it to contend this
// lock; the lock just makes the global safe to express in Rust.

/// The kernel's frame allocator, once [`install_kernel_allocator`] has moved it here.
static KERNEL_FRAME_ALLOCATOR: Mutex<Option<BootInfoFrameAllocator>> = Mutex::new(None);

/// The physical-memory-window base (`BootInfo::physical_memory_offset`), stored so trap
/// handlers can reach it alongside the frame allocator. Zero until installed.
static PHYSICAL_MEMORY_OFFSET: AtomicU64 = AtomicU64::new(0);

/// Move the frame allocator and physical-memory offset into the globals above, so trap
/// handlers (notably the `spawn` syscall's ELF load) can allocate frames. Call once,
/// after the boot-time frame allocations and before entering the first user process.
pub fn install_kernel_allocator(
    frame_allocator: BootInfoFrameAllocator,
    physical_memory_offset: VirtAddr,
) {
    *KERNEL_FRAME_ALLOCATOR.lock() = Some(frame_allocator);
    PHYSICAL_MEMORY_OFFSET.store(physical_memory_offset.as_u64(), Ordering::SeqCst);
}

/// The physical-memory-window base recorded by [`install_kernel_allocator`].
pub fn physical_memory_offset() -> VirtAddr {
    VirtAddr::new(PHYSICAL_MEMORY_OFFSET.load(Ordering::SeqCst))
}

/// Run `f` with exclusive access to the kernel frame allocator installed by
/// [`install_kernel_allocator`]. Panics if no allocator has been installed yet.
pub fn with_kernel_frame_allocator<R>(f: impl FnOnce(&mut BootInfoFrameAllocator) -> R) -> R {
    let mut guard = KERNEL_FRAME_ALLOCATOR.lock();
    let allocator = guard.as_mut().expect("kernel frame allocator not installed");
    f(allocator)
}

/// Map `page` onto the VGA text-buffer frame (physical `0xb8000`), creating any
/// missing intermediate page tables from `frame_allocator`.
///
/// A Stage 4b demonstration: afterwards, writing through `page` reaches the same
/// physical memory as the screen. It is a *safe* function because the VGA frame
/// is device memory that is always sound to map writable, and aliasing it is the
/// whole point; a general "map any frame" helper would have to be `unsafe`.
/// Ensure the 4 KiB frame at physical `phys` is identity-mapped (virtual address ==
/// physical address), present and writable, in the active page tables.
///
/// Stage 16b SMP bring-up needs this: an AP starts executing the trampoline at its
/// physical address with paging off, then loads the kernel's CR3 and enables paging —
/// at which point the very next instruction is fetched through the page tables, so the
/// trampoline page must map to itself or the AP triple-faults. If the bootloader
/// already identity-maps this low page (it maps low memory), this is a no-op.
pub fn ensure_identity_mapped(
    phys: u64,
    mapper: &mut OffsetPageTable,
    frame_allocator: &mut impl FrameAllocator<Size4KiB>,
) {
    let addr = VirtAddr::new(phys);
    if mapper.translate_addr(addr) == Some(PhysAddr::new(phys)) {
        return; // already identity-mapped (e.g. the bootloader's low-memory mapping)
    }
    let page = Page::<Size4KiB>::containing_address(addr);
    let frame = PhysFrame::<Size4KiB>::containing_address(PhysAddr::new(phys));
    let flags = PageTableFlags::PRESENT | PageTableFlags::WRITABLE;
    // SAFETY: `phys` is a real low-RAM frame and `addr` was just confirmed unmapped, so
    // mapping it to itself aliases nothing unexpected. The trampoline page must be
    // executable, which it is: we leave NO_EXECUTE clear.
    unsafe {
        mapper
            .map_to(page, frame, flags, frame_allocator)
            .expect("identity map of the AP trampoline page failed")
            .flush();
    }
}

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

// ---------------------------------------------------------------------------
// Guard-paged kernel stacks (a refinement over the Stage 6 / 16d heap stacks).
// ---------------------------------------------------------------------------
//
// Kernel-thread stacks (`sched.rs`, and the dormant `thread`) are plain heap
// allocations. Nothing sits below them but *other heap data*, so a thread that
// overflows its stack silently scribbles over some unrelated allocation — a
// corruption that surfaces far away and long after, as an impossible-looking bug.
//
// A *guard page* turns that silent corruption into an immediate, precise fault. We
// allocate one extra page *below* the usable stack and clear its PRESENT bit, so
// the first push past the end of the stack touches an unmapped page and raises a
// page fault (#PF) — whose handler prints the faulting address (CR2), pointing
// straight at the overflow. The stack grows downward, so the guard is the lowest
// page of the allocation.
//
// We carve the guard out of the heap allocation itself, rather than mapping a
// dedicated stack area, for one concrete reason: `BootInfoFrameAllocator` cannot
// free frames, so a stack area backed by fresh frames would leak a frame on every
// thread exit. The heap *can* reclaim, so the guard page is simply a heap page we
// mark not-present while the stack is alive and restore before freeing it. And
// because we only ever toggle the PRESENT bit (never the frame), a stale TLB entry
// on another core is harmless: the mapping still points at the same frame, and no
// core but the stack's owner ever touches that virtual address.

/// x86-64 base page size (the granularity of a guard page).
const PAGE_SIZE: usize = 4096;
/// Bit 7 of a page-table entry: at L3/L2 it marks a huge (1 GiB / 2 MiB) page, so
/// the walk stops rather than mistaking it for a pointer to the next-level table.
const ENTRY_HUGE_PAGE: u64 = 1 << 7;
/// Bits 12..52 of a page-table entry hold the physical address of the next-level
/// table (or the mapped frame); the remaining bits are flags.
const ENTRY_ADDR_MASK: u64 = 0x000f_ffff_ffff_f000;

/// Serializes the read-modify-write of a leaf page-table entry across cores. Each
/// guard page is a distinct entry, so edits never actually collide; this is cheap
/// insurance that a page-table write is atomic with respect to any other. Never
/// taken from an interrupt handler, and never held across a heap-allocator call, so
/// it cannot deadlock against the timer or the heap lock.
static PT_EDIT_LOCK: Mutex<()> = Mutex::new(());

/// Walk the active page tables to the *leaf* (L1) entry mapping `virt`, and return a
/// raw pointer to that 8-byte entry — regardless of whether the leaf itself is
/// present, so the caller can toggle its PRESENT bit either way. Returns `None` if
/// an intermediate level is missing or is a huge page (neither happens for a 4 KiB
/// heap mapping).
///
/// Uses raw `u64` reads through the physical-memory window — never a `&PageTable` —
/// on purpose: `init` already handed out the one `&mut` to the active L4, so any
/// reference here would alias it (undefined behavior). This mirrors
/// [`AddressSpace::new_cloning_kernel`].
///
/// # Safety
///
/// The physical-memory offset must be installed (true after [`init`]) and all of
/// physical memory mapped at it, so each table frame is readable at `offset + phys`.
unsafe fn leaf_pte_ptr(virt: VirtAddr) -> Option<*mut u64> {
    let offset = physical_memory_offset().as_u64();
    let va = virt.as_u64();
    // The four 9-bit table indices packed into the virtual address.
    let index = [
        ((va >> 39) & 0x1FF) as usize, // L4 (PML4)
        ((va >> 30) & 0x1FF) as usize, // L3 (PDPT)
        ((va >> 21) & 0x1FF) as usize, // L2 (PD)
        ((va >> 12) & 0x1FF) as usize, // L1 (PT)
    ];

    // Start at the active L4 frame (from CR3) and descend three levels to the L1.
    let (l4_frame, _) = Cr3::read();
    let mut table_phys = l4_frame.start_address().as_u64();
    for level in 0..3 {
        let table = (offset + table_phys) as *const u64;
        // SAFETY: `table` addresses a page-table frame through the physical-memory
        // window, so all 512 entries are readable; we only read.
        let entry = table.add(index[level]).read();
        if entry & ENTRY_PRESENT == 0 || entry & ENTRY_HUGE_PAGE != 0 {
            return None;
        }
        table_phys = entry & ENTRY_ADDR_MASK;
    }
    // `table_phys` now names the L1 table; return a pointer to its leaf entry.
    let l1 = (offset + table_phys) as *mut u64;
    Some(l1.add(index[3]))
}

/// Set or clear the PRESENT bit on the 4 KiB page containing `virt`, in the active
/// address space, and flush this core's TLB for it. Used to punch a guard page out
/// of a stack allocation (clear) and to restore it before the memory is freed (set).
///
/// # Safety
///
/// `virt` must lie in a 4 KiB mapping of the active space (a heap page here), and
/// clearing PRESENT is only sound if nothing accesses the page while it is cleared
/// — which holds for a guard page (the usable stack begins one page above it).
pub unsafe fn set_page_present(virt: VirtAddr, present: bool) {
    // Disable this core's interrupts for the whole edit (so the timer cannot preempt
    // us while we hold the lock) and serialize across cores with `PT_EDIT_LOCK`.
    x86_64::instructions::interrupts::without_interrupts(|| {
        let _guard = PT_EDIT_LOCK.lock();
        // SAFETY: the caller guarantees `virt` is a 4 KiB mapping of the active space
        // and the physical-memory window is installed.
        let pte = unsafe { leaf_pte_ptr(virt) }
            .expect("set_page_present: address is not a 4 KiB mapping");
        // SAFETY: `pte` points at a live leaf page-table entry (see `leaf_pte_ptr`).
        let mut entry = unsafe { pte.read() };
        if present {
            entry |= ENTRY_PRESENT;
        } else {
            entry &= !ENTRY_PRESENT;
        }
        // SAFETY: we write back a valid entry — only the PRESENT bit changed, the
        // frame address and other flags are untouched.
        unsafe { pte.write(entry) };
        // The CPU may have cached the old translation; flush this page so the change
        // takes effect on the running core. Other cores never touch a guard page and
        // the frame is unchanged, so no cross-core TLB shootdown is needed.
        tlb::flush(virt);
    });
}

/// Whether the 4 KiB page containing `virt` is currently present in the active
/// space. Reads the leaf entry directly (not the TLB), so it reflects the true
/// mapping. For the guard-page demo and test.
pub fn page_is_present(virt: VirtAddr) -> bool {
    // SAFETY: after `init` the physical-memory window is mapped, so the walk is
    // sound; we only read the entry.
    match unsafe { leaf_pte_ptr(virt) } {
        Some(pte) => {
            let entry = unsafe { pte.read() };
            entry & ENTRY_PRESENT != 0
        }
        None => false,
    }
}

/// A kernel-thread stack with an unmapped **guard page** just below its usable
/// region, so a stack overflow faults immediately instead of corrupting the heap.
///
/// Layout (low → high address):
///
/// ```text
/// [ guard page (unmapped) ][ usable stack (mapped) ]
/// ^base                    ^base + PAGE_SIZE        ^base + PAGE_SIZE + usable_size
/// ```
///
/// The stack pointer starts near the high end and grows downward; crossing below
/// `base + PAGE_SIZE` lands in the guard page and raises #PF. The whole thing is one
/// page-aligned heap allocation; [`Drop`] restores the guard page's mapping and
/// returns the memory to the heap.
pub struct GuardedStack {
    /// Base (lowest address) of the allocation — the guard page. Page-aligned.
    base: *mut u8,
    /// The exact layout `base` was allocated with (needed to free it).
    layout: Layout,
    /// Usable bytes above the guard page.
    usable_size: usize,
}

// SAFETY: a `GuardedStack` uniquely owns its heap allocation and the page-table
// state of its guard page; the raw `base` pointer is just that owned allocation.
// Moving it between threads (it lives in a per-CPU run queue behind a lock) transfers
// that sole ownership, so sending it across threads is sound.
unsafe impl Send for GuardedStack {}

impl GuardedStack {
    /// Allocate a stack with `usable_size` usable bytes (a positive multiple of the
    /// page size) plus one guard page below it. Aborts via `handle_alloc_error` if
    /// the heap is exhausted.
    pub fn new(usable_size: usize) -> GuardedStack {
        assert!(
            usable_size > 0 && usable_size % PAGE_SIZE == 0,
            "guarded stack size must be a positive multiple of the page size"
        );
        let total = usable_size + PAGE_SIZE; // one guard page below the usable stack
        let layout =
            Layout::from_size_align(total, PAGE_SIZE).expect("guarded stack layout is valid");

        // SAFETY: `layout` has nonzero size. `alloc_zeroed` returns a page-aligned
        // block of mapped, writable heap memory (align == PAGE_SIZE forces the
        // linked-list fallback, which honors alignment) or null on OOM.
        let base = unsafe { alloc_zeroed(layout) };
        if base.is_null() {
            handle_alloc_error(layout);
        }

        // Punch out the guard page: clear PRESENT on the lowest page. An overflow
        // that runs off the bottom of the usable stack now faults here.
        // SAFETY: `base` is page-aligned and was just mapped by the allocator; we
        // toggle only its PRESENT bit and never touch the page while it is cleared
        // (the usable stack begins one page above). Restored in `Drop` before free.
        unsafe {
            set_page_present(VirtAddr::new(base as u64), false);
        }

        GuardedStack {
            base,
            layout,
            usable_size,
        }
    }

    /// The usable stack region (above the guard page) as a writable slice, for
    /// fabricating the initial thread frame into its top.
    pub fn usable_mut(&mut self) -> &mut [u8] {
        // SAFETY: `[base + PAGE_SIZE, base + PAGE_SIZE + usable_size)` is mapped,
        // owned by this `GuardedStack`, and unaliased while `&mut self` is held.
        unsafe { core::slice::from_raw_parts_mut(self.base.add(PAGE_SIZE), self.usable_size) }
    }

    /// Virtual address of the guard page (the lowest, unmapped page).
    pub fn guard_page(&self) -> VirtAddr {
        VirtAddr::new(self.base as u64)
    }

    /// Lowest *usable* stack address (one page above the guard page).
    pub fn usable_bottom(&self) -> VirtAddr {
        VirtAddr::new(self.base as u64 + PAGE_SIZE as u64)
    }
}

impl Drop for GuardedStack {
    fn drop(&mut self) {
        // Restore the guard page's mapping *before* freeing: the allocator writes its
        // free-list bookkeeping across the whole region, including this page.
        // SAFETY: we cleared PRESENT on this same page in `new`; setting it again
        // restores the original mapping (the frame was never changed).
        unsafe {
            set_page_present(self.guard_page(), true);
        }
        // SAFETY: `base`/`layout` are exactly what `alloc_zeroed` returned in `new`,
        // and the whole region is mapped again, so the allocator may reuse it.
        unsafe {
            dealloc(self.base, self.layout);
        }
    }
}

/// Set once [`demo_guard_page`] has confirmed a guarded stack is fault-armed: its
/// guard page is unmapped, its usable region is mapped, and the guard page is
/// remapped after the stack is freed. Read by the guard-page test.
static GUARD_PAGE_OK: AtomicBool = AtomicBool::new(false);

/// Whether the boot-time guard-page check passed.
pub fn guard_page_ok() -> bool {
    GUARD_PAGE_OK.load(Ordering::SeqCst)
}

/// Guard-page demonstration: allocate a guarded stack, verify the page below the
/// usable region is unmapped (so an overflow would fault) while the usable region is
/// mapped, then free it and verify the guard page is restored (so the heap can
/// safely reuse the memory). Records the outcome for [`guard_page_ok`].
///
/// Call after the heap is up and the physical-memory offset is installed.
pub fn demo_guard_page() {
    let stack = GuardedStack::new(PAGE_SIZE); // one usable page + one guard page
    let guard = stack.guard_page();
    let usable = stack.usable_bottom();

    let guard_unmapped = !page_is_present(guard);
    let usable_mapped = page_is_present(usable);

    drop(stack); // frees the allocation; `Drop` remaps the guard page first
    let guard_restored = page_is_present(guard);

    let ok = guard_unmapped && usable_mapped && guard_restored;
    GUARD_PAGE_OK.store(ok, Ordering::SeqCst);
    serial_println!(
        "[stack] guard page @ {:?} unmapped = {}, usable @ {:?} mapped = {}, restored on free = {}",
        guard,
        guard_unmapped,
        usable,
        usable_mapped,
        guard_restored,
    );
}

// ---------------------------------------------------------------------------
// Process address spaces (Stage 11a).
// ---------------------------------------------------------------------------
//
// Until now the whole kernel has run in a single address space: one set of page
// tables, one value in CR3, set up by the bootloader. A *process* needs its own
// address space, so two programs can use the same virtual addresses for different
// physical memory and neither can reach the other's. At the hardware level an
// address space *is* a top-level (L4 / PML4) page table; "switching to a process"
// is loading that table's frame into CR3.
//
// The one thing that must survive a CR3 switch is the kernel itself. The CPU is
// running kernel instructions on a kernel stack, and the instant after `mov cr3`
// it fetches the next instruction through the *new* table. If that table does not
// map the kernel, the fetch faults, no handler is reachable, and the machine
// triple-faults. So every address space must map the kernel (plus the heap, the
// stacks, and the physical-memory window) at the same virtual addresses.
//
// The textbook way to guarantee that is the "higher-half kernel": keep the kernel
// in L4 slots 256..512 and user programs in 0..256, so a new space just copies the
// higher half. Bootloader 0.9 does not relocate us there, though — it maps the
// kernel, heap, and physical-memory window in the *lower* half (watch the boot
// log: the present L4 slots are all < 256). So rather than copy a fixed half we
// copy *every present* top-level entry, wherever it sits. The clone then maps
// exactly what the kernel maps; a user program's pages later go into slots that
// are still empty here.

/// A page table holds 512 entries.
const PAGE_TABLE_ENTRIES: usize = 512;
/// Bit 0 of a page-table entry marks it present (it maps something).
const ENTRY_PRESENT: u64 = 1 << 0;

/// One process's address space: ownership of a top-level (L4) page-table frame.
///
/// Loading [`AddressSpace::l4_frame`] into CR3 makes this space active. For now an
/// `AddressSpace` only ever clones the kernel's mappings; Stage 11b will map a user
/// program into the empty lower slots of one.
///
/// The L4 frame is never freed: `BootInfoFrameAllocator` cannot reclaim frames, so
/// dropping an `AddressSpace` simply leaks its one frame. That is fine for the
/// boot-time experiments here; a real allocator comes later.
pub struct AddressSpace {
    l4_frame: PhysFrame,
}

impl AddressSpace {
    /// Build a new address space that mirrors the kernel's current one.
    ///
    /// Allocates a fresh frame for the L4 table, zeroes it, then copies every
    /// present entry from the active (kernel) L4 into it, so the result maps
    /// exactly what the kernel maps. Returns `None` if no physical frame is free.
    ///
    /// `physical_memory_offset` must be the bootloader's physical-memory-window
    /// base (the value passed to [`init`]); it is trusted to reach both L4 frames.
    pub fn new_cloning_kernel(
        frame_allocator: &mut impl FrameAllocator<Size4KiB>,
        physical_memory_offset: VirtAddr,
    ) -> Option<AddressSpace> {
        let l4_frame = frame_allocator.allocate_frame()?;
        // The L4 that CR3 points at right now: the kernel space we are cloning.
        let (active_l4_frame, _) = Cr3::read();

        // Reach both L4 tables through the physical-memory window, as raw arrays of
        // 512 eight-byte entries. We use raw `u64` pointers rather than
        // `&PageTable` on purpose: the kernel's `mapper` (built in `init`) already
        // holds a `&mut PageTable` to the active L4, so forming any reference to it
        // here would alias that `&mut` — undefined behavior. Raw reads alias
        // nothing.
        let src =
            (physical_memory_offset + active_l4_frame.start_address().as_u64()).as_ptr::<u64>();
        let dst =
            (physical_memory_offset + l4_frame.start_address().as_u64()).as_mut_ptr::<u64>();

        // SAFETY: `src` and `dst` address two distinct, page-aligned 4 KiB frames
        // lying fully inside the physical-memory window, so every one of the 512
        // eight-byte slots of each is valid to access. `dst`'s frame is freshly
        // allocated and referenced by nothing else. We zero `dst` before copying so
        // leftover bits can never be walked as a bogus entry; then we copy only
        // present entries. A copied L4 entry holds the physical address of one of
        // the kernel's L3 tables, so the clone shares — and thus maps — exactly the
        // kernel's memory.
        unsafe {
            for i in 0..PAGE_TABLE_ENTRIES {
                dst.add(i).write(0);
            }
            for i in 0..PAGE_TABLE_ENTRIES {
                let entry = src.add(i).read();
                if entry & ENTRY_PRESENT != 0 {
                    dst.add(i).write(entry);
                }
            }
        }

        Some(AddressSpace { l4_frame })
    }

    /// The physical frame holding this space's L4 table — the value CR3 takes to
    /// make the space active.
    pub fn l4_frame(&self) -> PhysFrame {
        self.l4_frame
    }

    /// Build an [`OffsetPageTable`] over this space's L4, so the caller can install
    /// mappings into it — even while the space is *inactive*. The new tables are
    /// reached and edited through the physical-memory window, not through the user
    /// virtual addresses (which only become reachable once CR3 points here).
    ///
    /// Takes `&mut self` so the borrow checker forbids two live mappers over the
    /// same space at once, which would alias its L4 table.
    pub fn mapper(&mut self, physical_memory_offset: VirtAddr) -> OffsetPageTable<'_> {
        // SAFETY: this points at the space's own L4 frame through the
        // physical-memory window. Nothing else references that frame while the
        // `&mut self` borrow is held, so the `&mut PageTable` is unaliased.
        let level_4_table: &mut PageTable = unsafe {
            &mut *(physical_memory_offset + self.l4_frame.start_address().as_u64())
                .as_mut_ptr::<PageTable>()
        };
        // SAFETY: all physical memory is mapped at `physical_memory_offset`, so the
        // mapper can follow this space's lower-level tables.
        unsafe { OffsetPageTable::new(level_4_table, physical_memory_offset) }
    }

    /// Make this address space active: load its L4 frame into CR3. Returns the
    /// previously-active `(frame, flags)`, which [`restore_address_space`] takes to
    /// switch back.
    ///
    /// # Safety
    ///
    /// This space's L4 must map the executing kernel — its code, the current stack,
    /// and the physical-memory window — at their current virtual addresses, or the
    /// instruction right after the CR3 load faults with no reachable handler and
    /// the CPU triple-faults. A space from [`AddressSpace::new_cloning_kernel`]
    /// satisfies this. (Loading CR3 also flushes the TLB.)
    pub unsafe fn activate(&self) -> (PhysFrame, Cr3Flags) {
        let previous = Cr3::read();
        // SAFETY: by this method's contract `self.l4_frame` maps the running
        // kernel; we keep the current CR3 flags unchanged.
        Cr3::write(self.l4_frame, previous.1);
        previous
    }
}

/// Restore a previously-active address space — pass the value [`AddressSpace::activate`]
/// returned.
///
/// # Safety
///
/// `previous` must be a CR3 `(frame, flags)` that was active earlier in this boot
/// and still maps the running kernel; the value `activate` returns is exactly that.
/// (Loading CR3 also flushes the TLB.)
pub unsafe fn restore_address_space(previous: (PhysFrame, Cr3Flags)) {
    // SAFETY: per the contract `previous` is a known-good CR3 that maps the running
    // kernel, so restoring it is sound.
    Cr3::write(previous.0, previous.1);
}

// --- Stage 11a boot demo + verification state ------------------------------

/// Set once the boot demo has cloned the kernel space, switched CR3 onto the
/// clone, run there, and switched back successfully. Read by the Stage 11a test.
static CLONE_ROUNDTRIP_OK: AtomicBool = AtomicBool::new(false);

/// Whether the address-space clone + CR3 round-trip succeeded at boot.
pub fn address_space_clone_ok() -> bool {
    CLONE_ROUNDTRIP_OK.load(Ordering::SeqCst)
}

/// Stage 11a demonstration: clone the kernel address space, switch the CPU onto
/// the clone, do real kernel work there (a heap allocation), then switch back.
///
/// Completing the round-trip is the proof the clone is faithful: were it missing
/// any entry the running kernel needs, the first access after the CR3 switch would
/// fault. Records the outcome for [`address_space_clone_ok`].
pub fn demo_clone_kernel_space(
    frame_allocator: &mut impl FrameAllocator<Size4KiB>,
    physical_memory_offset: VirtAddr,
) {
    let kernel_l4 = Cr3::read().0;
    serial_println!(
        "[addrspace] kernel address space: L4 at {:?}",
        kernel_l4.start_address()
    );

    let space = match AddressSpace::new_cloning_kernel(frame_allocator, physical_memory_offset) {
        Some(space) => space,
        None => {
            serial_println!("[addrspace] no free frame for a new L4; skipping demo");
            return;
        }
    };

    // List which top-level slots the kernel occupies. Printing them makes the
    // "kernel lives in the lower half" claim above concrete, and explains why the
    // clone copies every present entry instead of a fixed higher half.
    let l4_ptr =
        (physical_memory_offset + space.l4_frame().start_address().as_u64()).as_ptr::<u64>();
    let mut slots = alloc::vec::Vec::new();
    for i in 0..PAGE_TABLE_ENTRIES {
        // SAFETY: `l4_ptr` addresses the clone's own L4 frame through the
        // physical-memory window, so all 512 slots are readable, and nothing else
        // references this freshly-allocated frame. We only read.
        if unsafe { l4_ptr.add(i).read() } & ENTRY_PRESENT != 0 {
            slots.push(i);
        }
    }
    let all_lower = slots.iter().all(|&s| s < 256);
    serial_println!(
        "[addrspace] cloned the kernel into a new L4 at {:?}; present L4 slots {:?} ({})",
        space.l4_frame().start_address(),
        slots,
        if all_lower {
            "all in the lower half, so the clone copies every present entry"
        } else {
            "spanning both halves"
        },
    );

    // Switch onto the clone, prove the kernel still works there, then switch back.
    // SAFETY: `space` cloned every present entry of the active kernel space, so it
    // maps the running code, stack, heap, and physical-memory window; switching to
    // it is sound.
    let previous = unsafe { space.activate() };
    let active_on_clone = Cr3::read().0;
    // Exercise the heap while on the clone. The kernel heap is the *same* physical
    // memory in both spaces (we copied its L4 entry), so allocating here and
    // freeing it after the switch-back is valid. `black_box` stops the compiler
    // from optimizing the probe — and thus the round-trip — away.
    let probe = alloc::boxed::Box::new(0xA5A5_u64);
    let probe_ok = core::hint::black_box(*probe) == 0xA5A5;
    // SAFETY: `previous` is the kernel space active moments ago; it still maps the
    // running kernel, so restoring it is sound.
    unsafe { restore_address_space(previous) };
    let active_after_restore = Cr3::read().0;

    let ok = active_on_clone == space.l4_frame() && probe_ok && active_after_restore == kernel_l4;
    CLONE_ROUNDTRIP_OK.store(ok, Ordering::SeqCst);
    serial_println!(
        "[addrspace] switched to the clone (CR3 -> {:?}) and back to the kernel (CR3 -> {:?})",
        active_on_clone.start_address(),
        active_after_restore.start_address(),
    );
}
