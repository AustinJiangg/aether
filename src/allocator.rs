//! Kernel heap: a memory region plus a global allocator, so the `alloc` crate's
//! types (`Box`, `Vec`, `String`, `Rc`, ...) become usable.
//!
//! Two pieces are needed. First, real backing memory: we pick a fixed virtual
//! address range for the heap and map every page in it to a freshly-allocated
//! physical frame (using the Stage 4b frame allocator + `map_to`). Second, a
//! *global allocator*: a type marked `#[global_allocator]` that implements
//! `GlobalAlloc`, which `alloc` calls to carve that region into individual
//! allocations. Here that is a hand-written **bump allocator** — the simplest
//! possible design. It is instructive precisely because of its limitation: it
//! cannot reclaim an individual freed allocation, only reset once the whole heap
//! empties.

use alloc::alloc::{GlobalAlloc, Layout};
use core::ptr;

use spin::Mutex;
use x86_64::structures::paging::mapper::MapToError;
use x86_64::structures::paging::{FrameAllocator, Mapper, Page, PageTableFlags, Size4KiB};
use x86_64::VirtAddr;

/// Where the kernel heap starts in virtual memory. The exact address is
/// arbitrary; it only has to be unused and clear of everything the bootloader
/// mapped (and of the demo mapping from Stage 4b).
pub const HEAP_START: usize = 0x_4444_4444_0000;
/// How big the heap is: 100 KiB, plenty for early experiments.
pub const HEAP_SIZE: usize = 100 * 1024;

/// The global allocator instance. `#[global_allocator]` wires it into `alloc`:
/// every `Box::new`, `Vec::push`, etc. ultimately calls this value's
/// `GlobalAlloc` methods. It starts empty and is armed by `init_heap`.
#[global_allocator]
static ALLOCATOR: Locked<BumpAllocator> = Locked::new(BumpAllocator::new());

/// Map the heap's pages and hand the backing memory to the global allocator.
///
/// Call once, after paging and the frame allocator are up. We walk every page in
/// `HEAP_START..HEAP_START + HEAP_SIZE`, allocate a frame for each, and map it
/// writable; then we tell the bump allocator which range it now owns.
pub fn init_heap(
    mapper: &mut impl Mapper<Size4KiB>,
    frame_allocator: &mut impl FrameAllocator<Size4KiB>,
) -> Result<(), MapToError<Size4KiB>> {
    // The inclusive range of pages covering the heap.
    let page_range = {
        let heap_start = VirtAddr::new(HEAP_START as u64);
        let heap_end = heap_start + HEAP_SIZE as u64 - 1u64;
        let heap_start_page = Page::containing_address(heap_start);
        let heap_end_page = Page::containing_address(heap_end);
        Page::range_inclusive(heap_start_page, heap_end_page)
    };

    for page in page_range {
        let frame = frame_allocator
            .allocate_frame()
            .ok_or(MapToError::FrameAllocationFailed)?;
        let flags = PageTableFlags::PRESENT | PageTableFlags::WRITABLE;
        // SAFETY: each `frame` comes fresh from the frame allocator, so it is
        // unused and unaliased; the heap range is currently unmapped, so mapping
        // it writable cannot clobber an existing mapping. `?` aborts boot if a
        // frame runs out, rather than continuing with a half-built heap.
        unsafe {
            mapper.map_to(page, frame, flags, frame_allocator)?.flush();
        }
    }

    // SAFETY: the loop above backed the whole range with mapped, writable memory
    // that nothing else uses, which is exactly `init`'s contract; called once.
    unsafe {
        ALLOCATOR.lock().init(HEAP_START, HEAP_SIZE);
    }

    Ok(())
}

/// A spinlock newtype around an allocator.
///
/// Two reasons it exists. The orphan rule forbids implementing the foreign
/// `GlobalAlloc` trait directly on the foreign `spin::Mutex`, so we wrap it in a
/// type of our own. And `GlobalAlloc`'s methods take `&self`, yet a bump
/// allocator must mutate its cursor — the `Mutex` supplies that interior
/// mutability (and makes concurrent access safe).
pub struct Locked<A> {
    inner: Mutex<A>,
}

impl<A> Locked<A> {
    pub const fn new(inner: A) -> Self {
        Locked {
            inner: Mutex::new(inner),
        }
    }

    pub fn lock(&self) -> spin::MutexGuard<'_, A> {
        self.inner.lock()
    }
}

/// The simplest allocator there is. Keep a `next` cursor that only moves forward:
/// each allocation returns `next` (rounded up to the requested alignment) and
/// bumps it past the new block. Memory is reclaimed only when *every* live
/// allocation has been freed (tracked by `allocations`), at which point `next`
/// snaps back to the start.
pub struct BumpAllocator {
    heap_start: usize,
    heap_end: usize,
    next: usize,
    allocations: usize,
}

impl BumpAllocator {
    /// Create a new, empty bump allocator. `const` so it can initialize a
    /// `static` (the `#[global_allocator]` above) at compile time.
    pub const fn new() -> Self {
        BumpAllocator {
            heap_start: 0,
            heap_end: 0,
            next: 0,
            allocations: 0,
        }
    }

    /// Arm the allocator with the heap bounds.
    ///
    /// # Safety
    ///
    /// `heap_start..heap_start + heap_size` must be unused, mapped, writable
    /// memory, and this must be called exactly once.
    pub unsafe fn init(&mut self, heap_start: usize, heap_size: usize) {
        self.heap_start = heap_start;
        self.heap_end = heap_start + heap_size;
        self.next = heap_start;
    }
}

// SAFETY: `GlobalAlloc` is an unsafe trait — the allocator must return blocks
// that are correctly aligned, large enough, and not currently handed out. `alloc`
// below upholds all three: it aligns `next` up to `layout.align()`, reserves
// exactly `layout.size()` bytes, and only ever moves `next` forward (or resets it
// when nothing is live), so it never overlaps a live allocation.
unsafe impl GlobalAlloc for Locked<BumpAllocator> {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        let mut bump = self.lock(); // hold the spinlock for the whole operation

        let alloc_start = align_up(bump.next, layout.align());
        let alloc_end = match alloc_start.checked_add(layout.size()) {
            Some(end) => end,
            None => return ptr::null_mut(), // address overflow -> out of memory
        };

        if alloc_end > bump.heap_end {
            ptr::null_mut() // not enough room; GlobalAlloc signals failure with null
        } else {
            bump.next = alloc_end;
            bump.allocations += 1;
            alloc_start as *mut u8
        }
    }

    unsafe fn dealloc(&self, _ptr: *mut u8, _layout: Layout) {
        let mut bump = self.lock();

        // A bump allocator cannot free an individual block. We only count live
        // allocations down; when the last one is freed, the whole heap is reusable
        // again, so reset the cursor.
        bump.allocations -= 1;
        if bump.allocations == 0 {
            bump.next = bump.heap_start;
        }
    }
}

/// Round `addr` up to the nearest multiple of `align`, which must be a power of
/// two. The bit trick: adding `align - 1` pushes past the next boundary, and
/// masking off the low bits with `!(align - 1)` clears any remainder in one step.
fn align_up(addr: usize, align: usize) -> usize {
    (addr + align - 1) & !(align - 1)
}
