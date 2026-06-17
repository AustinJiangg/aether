//! Kernel heap: a memory region plus a global allocator, so the `alloc` crate's
//! types (`Box`, `Vec`, `String`, `Rc`, ...) become usable.
//!
//! First, real backing memory: we pick a fixed virtual address range for the heap
//! and map every page in it to a freshly-allocated physical frame (using the
//! Stage 4b frame allocator + `map_to`). Second, a *global allocator*: a type
//! marked `#[global_allocator]` implementing `GlobalAlloc`, which `alloc` calls to
//! carve that region into individual allocations.
//!
//! The global allocator here is a two-layer, hand-written design:
//!
//! - A **fixed-size block allocator** on top. It keeps a separate free list for
//!   each of a few power-of-two block sizes (8, 16, ... 2048 bytes). A small
//!   allocation is rounded up to the nearest block size and served by popping a
//!   block off that list in O(1); freeing just pushes the block back. This is fast
//!   and avoids the linear search and fragmentation of a pure free list.
//! - A **linked-list (free-list) allocator** underneath, as the fallback. It hands
//!   out blocks when a size's list is empty, and serves any request too large for
//!   the biggest block size. The free regions of the heap thread a list of
//!   `ListNode`s stored in the free memory itself.
//!
//! So most allocations are O(1) from the block lists, and the linked-list layer is
//! the slower, general-purpose backstop.

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
static ALLOCATOR: Locked<FixedSizeBlockAllocator> = Locked::new(FixedSizeBlockAllocator::new());

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
/// must mutate its lists — the `Mutex` supplies that interior mutability (and
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

// ===========================================================================
// Fixed-size block allocator (the global allocator).
// ===========================================================================

/// The block sizes that get a dedicated free list. Each MUST be a power of two,
/// because we also use the size as the block's alignment. A request larger than
/// the last size falls through to the linked-list fallback. The smallest size (8)
/// must be at least `size_of::<BlockNode>()` so a freed block can hold its node.
const BLOCK_SIZES: &[usize] = &[8, 16, 32, 64, 128, 256, 512, 1024, 2048];

/// A free block on one of the fixed-size lists. Only a `next` link is needed: the
/// block's size is implied by which list it hangs on, so unlike `ListNode` it does
/// not store a size. Like `ListNode`, it lives inside the free block's own memory.
struct BlockNode {
    next: Option<&'static mut BlockNode>,
}

/// Pick the index of the smallest block size that fits `layout`, or `None` if the
/// request is larger than the largest block size (then it goes to the fallback).
/// We use `size.max(align)` so the chosen block satisfies both — valid because the
/// block sizes are powers of two and double as alignments.
fn list_index(layout: &Layout) -> Option<usize> {
    let required_block_size = layout.size().max(layout.align());
    BLOCK_SIZES.iter().position(|&s| s >= required_block_size)
}

/// One free list per block size, plus a linked-list allocator as the fallback.
pub struct FixedSizeBlockAllocator {
    list_heads: [Option<&'static mut BlockNode>; BLOCK_SIZES.len()],
    fallback: LinkedListAllocator,
}

impl FixedSizeBlockAllocator {
    /// Create an empty allocator: all lists empty, fallback empty. `const` so it
    /// can initialize the `#[global_allocator]` static at compile time.
    pub const fn new() -> Self {
        // `Option<&mut _>` is not `Copy`, so the array can't be `[None; N]`; the
        // const-item repeat below is the standard way to build it.
        const EMPTY: Option<&'static mut BlockNode> = None;
        FixedSizeBlockAllocator {
            list_heads: [EMPTY; BLOCK_SIZES.len()],
            fallback: LinkedListAllocator::new(),
        }
    }

    /// Arm the allocator by handing the whole heap to the fallback. Blocks migrate
    /// from the fallback into the per-size lists as they are allocated and freed.
    ///
    /// # Safety
    ///
    /// `heap_start..heap_start + heap_size` must be unused, mapped, writable
    /// memory, and this must be called exactly once.
    pub unsafe fn init(&mut self, heap_start: usize, heap_size: usize) {
        self.fallback.init(heap_start, heap_size);
    }
}

// SAFETY: `GlobalAlloc` is an unsafe trait; callers rely on returned blocks being
// correctly aligned, large enough, and not currently handed out. A block popped
// from `list_heads[i]` was pushed there by a matching `dealloc` (so it is free and
// is `BLOCK_SIZES[i]` bytes, hence big enough and aligned); fresh blocks and
// oversized requests come from the fallback, which upholds the same guarantees.
unsafe impl GlobalAlloc for Locked<FixedSizeBlockAllocator> {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        let mut allocator = self.lock();
        match list_index(&layout) {
            Some(index) => match allocator.list_heads[index].take() {
                // A block is waiting on this size's list: pop it. O(1).
                Some(node) => {
                    allocator.list_heads[index] = node.next.take();
                    node as *mut BlockNode as *mut u8
                }
                // The list is empty: carve a fresh block of this size from the
                // fallback. When freed it joins this list, growing the pool.
                None => {
                    let block_size = BLOCK_SIZES[index];
                    let block_align = block_size; // sizes are powers of two
                    let layout = Layout::from_size_align(block_size, block_align).unwrap();
                    allocator.fallback.allocate(layout)
                }
            },
            // Too big for any fixed size: serve it from the fallback directly.
            None => allocator.fallback.allocate(layout),
        }
    }

    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        let mut allocator = self.lock();
        match list_index(&layout) {
            // Fits a fixed size: push it back onto that list instead of truly
            // freeing it. O(1), no walking or splitting.
            Some(index) => {
                let new_node = BlockNode {
                    next: allocator.list_heads[index].take(),
                };
                // The block is BLOCK_SIZES[index] bytes (>= the node's size and
                // alignment, see BLOCK_SIZES), so it can hold the node we write.
                let new_node_ptr = ptr as *mut BlockNode;
                new_node_ptr.write(new_node);
                allocator.list_heads[index] = Some(&mut *new_node_ptr);
            }
            // Was a fallback allocation: free it through the fallback.
            None => allocator.fallback.deallocate(ptr, layout),
        }
    }
}

// ===========================================================================
// Linked-list (free-list) allocator — the fallback under the block allocator.
// ===========================================================================

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
    /// initialize the fixed-size allocator's fallback field at compile time.
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

    /// Allocate a block for `layout`, returning a pointer or null if no free region
    /// is big enough. Used by the fixed-size allocator as its fallback path.
    fn allocate(&mut self, layout: Layout) -> *mut u8 {
        let (size, align) = Self::size_align(layout);
        if let Some((region, alloc_start)) = self.find_region(size, align) {
            let alloc_end = alloc_start.checked_add(size).expect("overflow");
            let excess_size = region.end_addr() - alloc_end;
            if excess_size > 0 {
                // SAFETY: the tail [alloc_end, region.end) is the unused remainder
                // of a region we just unlinked, so it is free and writable;
                // `alloc_end` is ListNode-aligned and `excess_size` is at least one
                // ListNode (guaranteed by `alloc_from_region`'s split check).
                unsafe {
                    self.add_free_region(alloc_end, excess_size);
                }
            }
            alloc_start as *mut u8
        } else {
            ptr::null_mut()
        }
    }

    /// Return a block to the free list.
    ///
    /// # Safety
    ///
    /// `ptr` must have come from a previous `allocate` with the same `layout` and
    /// must not have been freed since.
    unsafe fn deallocate(&mut self, ptr: *mut u8, layout: Layout) {
        let (size, _) = Self::size_align(layout);
        self.add_free_region(ptr as usize, size);
    }
}

/// Round `addr` up to the nearest multiple of `align`, which must be a power of
/// two. The bit trick: adding `align - 1` pushes past the next boundary, and
/// masking off the low bits with `!(align - 1)` clears any remainder in one step.
fn align_up(addr: usize, align: usize) -> usize {
    (addr + align - 1) & !(align - 1)
}
