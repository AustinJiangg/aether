//! Kernel heap: a memory region plus a global allocator, so the `alloc` crate's
//! types (`Box`, `Vec`, `String`, `Rc`, ...) become usable.
//!
//! Two pieces are needed. First, real backing memory: we pick a fixed virtual
//! address range for the heap and map every page in it to a freshly-allocated
//! physical frame (using the Stage 4b frame allocator + `map_to`). Second, a
//! *global allocator*: a type marked `#[global_allocator]` that implements
//! `GlobalAlloc`, which `alloc` calls to carve that region into individual
//! allocations. Here that is a hand-written **linked-list allocator** (a.k.a.
//! free-list allocator): the free parts of the heap are themselves threaded into
//! a linked list of `ListNode`s, each holding its region's size and a pointer to
//! the next free region. Allocation walks the list for a region big enough (first
//! fit) and splits off the remainder; deallocation pushes the freed region back
//! onto the list. Unlike the bump allocator it replaced, it reclaims individual
//! freed blocks, so long- and short-lived allocations can coexist without
//! exhausting the heap.

use alloc::alloc::{GlobalAlloc, Layout};
use core::mem;
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
static ALLOCATOR: Locked<LinkedListAllocator> = Locked::new(LinkedListAllocator::new());

/// Map the heap's pages and hand the backing memory to the global allocator.
///
/// Call once, after paging and the frame allocator are up. We walk every page in
/// `HEAP_START..HEAP_START + HEAP_SIZE`, allocate a frame for each, and map it
/// writable; then we tell the allocator which range it now owns.
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
/// type of our own. And `GlobalAlloc`'s methods take `&self`, yet the allocator
/// must mutate its free list — the `Mutex` supplies that interior mutability (and
/// makes concurrent access safe).
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

/// A node in the free list. Each free region of the heap stores one of these *in
/// its own first bytes*, recording the region's size and a link to the next free
/// region — so the bookkeeping costs no memory beyond the free space itself.
struct ListNode {
    size: usize,
    next: Option<&'static mut ListNode>,
}

impl ListNode {
    const fn new(size: usize) -> Self {
        ListNode { size, next: None }
    }

    fn start_addr(&self) -> usize {
        self as *const Self as usize
    }

    fn end_addr(&self) -> usize {
        self.start_addr() + self.size
    }
}

/// A free-list allocator. `head` is a dummy node whose `next` points at the first
/// real free region; following the `next` links walks every free region.
pub struct LinkedListAllocator {
    head: ListNode,
}

impl LinkedListAllocator {
    /// Create an empty allocator (no free regions yet). `const` so it can
    /// initialize the `#[global_allocator]` static at compile time.
    pub const fn new() -> Self {
        Self {
            head: ListNode::new(0),
        }
    }

    /// Arm the allocator by handing it the whole heap as one big free region.
    ///
    /// # Safety
    ///
    /// `heap_start..heap_start + heap_size` must be unused, mapped, writable
    /// memory, and this must be called exactly once.
    pub unsafe fn init(&mut self, heap_start: usize, heap_size: usize) {
        self.add_free_region(heap_start, heap_size);
    }

    /// Push a free region of `size` bytes at `addr` onto the front of the list.
    ///
    /// # Safety
    ///
    /// The region must be unused and writable. `addr` must be aligned for a
    /// `ListNode` and `size` large enough to hold one (both asserted).
    unsafe fn add_free_region(&mut self, addr: usize, size: usize) {
        // The region must be able to hold a ListNode written at its start.
        assert_eq!(align_up(addr, mem::align_of::<ListNode>()), addr);
        assert!(size >= mem::size_of::<ListNode>());

        // Build the node, link it ahead of the current first free region, then
        // write it into the region's own memory and make it the new list head.
        let mut node = ListNode::new(size);
        node.next = self.head.next.take();
        let node_ptr = addr as *mut ListNode;
        node_ptr.write(node);
        self.head.next = Some(&mut *node_ptr);
    }

    /// Find the first free region that fits `size` bytes at `align`, unlink it
    /// from the list, and return it with the aligned start address.
    fn find_region(&mut self, size: usize, align: usize) -> Option<(&'static mut ListNode, usize)> {
        let mut current = &mut self.head;
        // Walk the list, keeping `current` one node behind `region` so we can
        // unlink `region` by rewiring `current.next`.
        while let Some(ref mut region) = current.next {
            if let Ok(alloc_start) = Self::alloc_from_region(&region, size, align) {
                // Suitable: unlink this region and return it.
                let next = region.next.take();
                let ret = Some((current.next.take().unwrap(), alloc_start));
                current.next = next;
                return ret;
            } else {
                // Too small: advance to the next region.
                current = current.next.as_mut().unwrap();
            }
        }
        None
    }

    /// Check whether `region` can hold `size` bytes at `align`; if so return the
    /// aligned start. Fails if it is too small, or if the leftover after the
    /// allocation would itself be too small to track as a free `ListNode`.
    fn alloc_from_region(region: &ListNode, size: usize, align: usize) -> Result<usize, ()> {
        let alloc_start = align_up(region.start_addr(), align);
        let alloc_end = alloc_start.checked_add(size).ok_or(())?;

        if alloc_end > region.end_addr() {
            return Err(()); // region too small
        }

        let excess_size = region.end_addr() - alloc_end;
        if excess_size > 0 && excess_size < mem::size_of::<ListNode>() {
            return Err(()); // remainder too small to become its own free node
        }

        Ok(alloc_start)
    }

    /// Adjust a requested `Layout` so every allocation is at least the size and
    /// alignment of a `ListNode`. That guarantees the block can be turned back
    /// into a free-list node when it is later deallocated.
    fn size_align(layout: Layout) -> (usize, usize) {
        let layout = layout
            .align_to(mem::align_of::<ListNode>())
            .expect("adjusting alignment failed")
            .pad_to_align();
        let size = layout.size().max(mem::size_of::<ListNode>());
        (size, layout.align())
    }
}

// SAFETY: `GlobalAlloc` is an unsafe trait; callers rely on returned blocks being
// correctly aligned, large enough, and not currently handed out. `find_region`
// returns only a region just unlinked from the free list (so it is not live), and
// `size_align` forces every block to fit a `ListNode` so it can rejoin the list on
// free; any leftover tail is returned to the list as its own free region.
unsafe impl GlobalAlloc for Locked<LinkedListAllocator> {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        let (size, align) = LinkedListAllocator::size_align(layout);
        let mut allocator = self.lock();

        if let Some((region, alloc_start)) = allocator.find_region(size, align) {
            let alloc_end = alloc_start.checked_add(size).expect("overflow");
            let excess_size = region.end_addr() - alloc_end;
            if excess_size > 0 {
                // Return the unused tail of the region to the free list.
                allocator.add_free_region(alloc_end, excess_size);
            }
            alloc_start as *mut u8
        } else {
            ptr::null_mut() // no region big enough -> out of memory
        }
    }

    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        // Re-derive the true block size and push the region back onto the list.
        let (size, _) = LinkedListAllocator::size_align(layout);
        self.lock().add_free_region(ptr as usize, size);
    }
}

/// Round `addr` up to the nearest multiple of `align`, which must be a power of
/// two. The bit trick: adding `align - 1` pushes past the next boundary, and
/// masking off the low bits with `!(align - 1)` clears any remainder in one step.
fn align_up(addr: usize, align: usize) -> usize {
    (addr + align - 1) & !(align - 1)
}
