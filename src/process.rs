//! Stage 11b: loading an ELF program into its own address space.
//!
//! Stage 11a built an empty [`AddressSpace`] that clones the kernel. This stage
//! fills the *user* part of one: it parses a real ELF64 executable (see `elf.rs`)
//! and maps each `PT_LOAD` segment into the new space at the virtual address the
//! linker chose, copying the bytes in.
//!
//! The catch is that we map into a space that is **not active** — CR3 still points
//! at the kernel, so the CPU cannot reach the user virtual addresses yet. We solve
//! it the same way the kernel reaches any page table: through the physical-memory
//! window. For each page of a segment we allocate a frame, map it into the new
//! space (so it will appear at the user address once we switch CR3), and write the
//! segment's bytes to that frame via *its physical-memory-window address*.
//!
//! This stage only loads and verifies the program — by translating the entry point
//! in the new space and reading the code back. Switching CR3 to the space and
//! executing the entry in ring 3 is the next step (it reuses the Stage 9/10
//! ring-3 machinery, which already works on any address space because every space
//! maps the kernel).

use core::sync::atomic::{AtomicBool, AtomicU64, Ordering};

use alloc::vec::Vec;
use x86_64::registers::control::{Cr3, Cr3Flags};
use x86_64::structures::paging::{
    FrameAllocator, Mapper, OffsetPageTable, Page, PageTableFlags, PhysFrame, Size4KiB, Translate,
};
use x86_64::{PhysAddr, VirtAddr};

use crate::elf::{ElfError, ElfFile, ProgramHeader, PF_R, PF_W, PF_X, PT_LOAD};
use crate::memory::{self, AddressSpace};
use crate::serial_println;
use crate::usermode;

/// Where the demo ELF asks to be loaded: slot 64 of the L4 (`0x2000_0000_0000`),
/// which the kernel leaves empty — so the loader builds *private* lower-level
/// tables there instead of sharing the kernel's. (Stage 11a's boot log shows the
/// kernel's present L4 slots are 0, 2, 3, 4, 5, 31, 128, 136 — not 64.)
pub const USER_LOAD_BASE: u64 = 0x2000_0000_0000;

/// Offset of the entry point within the demo image: ELF header (64) + one program
/// header (56). The code sits right after the headers.
const DEMO_ENTRY_OFFSET: usize = 64 + 56;
/// Length of the hand-assembled ring 3 program (`write` then `exit`).
const DEMO_CODE_LEN: usize = 23;

/// Top of the program's user stack (the initial user `rsp`). It sits in the same
/// private L4 slot as the loaded image, well above the code page; the stack grows
/// down from here. An ELF does not describe a stack — setting one up is the OS's
/// job — so the loader maps it separately.
const USER_STACK_TOP: u64 = 0x2000_0010_0000;
/// How many 4 KiB pages back the user stack (16 KiB — ample for the demo program).
const USER_STACK_PAGES: u64 = 4;

/// A loaded user program: its private address space, entry point, and user stack.
/// Stage 12 will grow this into a schedulable process (a saved register context).
pub struct UserImage {
    space: AddressSpace,
    entry: VirtAddr,
    user_stack_top: VirtAddr,
}

impl UserImage {
    /// The program's entry point (a virtual address inside `space`).
    pub fn entry(&self) -> VirtAddr {
        self.entry
    }
}

/// Load an ELF64 executable into a fresh address space that clones the kernel.
pub fn load(
    elf_bytes: &[u8],
    frame_allocator: &mut impl FrameAllocator<Size4KiB>,
    physical_memory_offset: VirtAddr,
) -> Result<UserImage, ElfError> {
    let elf = ElfFile::parse(elf_bytes)?;
    let entry = VirtAddr::new(elf.entry());

    let mut space = AddressSpace::new_cloning_kernel(frame_allocator, physical_memory_offset)
        .ok_or(ElfError::OutOfFrames)?;

    let user_stack_top;
    {
        let mut mapper = space.mapper(physical_memory_offset);
        for ph in elf.load_segments() {
            map_segment(
                &mut mapper,
                &ph,
                elf_bytes,
                frame_allocator,
                physical_memory_offset,
            )?;
        }
        // The ELF describes only its own segments; the OS supplies the user stack.
        user_stack_top = map_user_stack(&mut mapper, frame_allocator, physical_memory_offset)?;
    } // drop the mapper, releasing the &mut borrow of `space`

    Ok(UserImage {
        space,
        entry,
        user_stack_top,
    })
}

/// Map a writable, user-accessible stack into the (inactive) space and return its
/// top — the initial user `rsp`. The ring 3 program pushes its syscall arguments
/// here, so without it the first `push` would fault.
fn map_user_stack(
    mapper: &mut OffsetPageTable,
    frame_allocator: &mut impl FrameAllocator<Size4KiB>,
    physical_memory_offset: VirtAddr,
) -> Result<VirtAddr, ElfError> {
    let flags =
        PageTableFlags::PRESENT | PageTableFlags::WRITABLE | PageTableFlags::USER_ACCESSIBLE;
    let bottom = USER_STACK_TOP - USER_STACK_PAGES * 4096;
    let first = Page::<Size4KiB>::containing_address(VirtAddr::new(bottom));
    let last = Page::<Size4KiB>::containing_address(VirtAddr::new(USER_STACK_TOP - 1));

    for page in Page::range_inclusive(first, last) {
        let frame = frame_allocator
            .allocate_frame()
            .ok_or(ElfError::OutOfFrames)?;

        // SAFETY: `frame` is fresh and unused; `page` is a user stack address in an
        // empty slot of this private space, so nothing is aliased. User-accessible +
        // writable is what a user stack needs. The space is inactive, so `ignore()`
        // the flush — switching CR3 to run the program flushes the whole TLB.
        unsafe {
            mapper
                .map_to_with_table_flags(page, frame, flags, flags, frame_allocator)
                .map_err(|_| ElfError::MapFailed)?
                .ignore();
        }

        // Zero the stack page through the physical-memory window (it is not yet
        // reachable at its user address).
        let window = (physical_memory_offset + frame.start_address().as_u64()).as_mut_ptr::<u8>();
        // SAFETY: `window` addresses this fresh 4 KiB frame through the physical
        // memory window; the writes stay within those 4 KiB.
        unsafe {
            for i in 0..4096usize {
                window.add(i).write(0);
            }
        }
    }

    Ok(VirtAddr::new(USER_STACK_TOP))
}

/// Map one `PT_LOAD` segment into `mapper`'s (inactive) address space and copy its
/// bytes in through the physical-memory window.
fn map_segment(
    mapper: &mut OffsetPageTable,
    ph: &ProgramHeader,
    elf_bytes: &[u8],
    frame_allocator: &mut impl FrameAllocator<Size4KiB>,
    physical_memory_offset: VirtAddr,
) -> Result<(), ElfError> {
    // User pages: present + user-accessible, executable (we do not set NO_EXECUTE,
    // so ring 3 can fetch), and writable if the segment is. Every parent table must
    // be user-accessible too, or a ring 3 walk faults before reaching the leaf.
    let mut page_flags = PageTableFlags::PRESENT | PageTableFlags::USER_ACCESSIBLE;
    if ph.is_writable() {
        page_flags |= PageTableFlags::WRITABLE;
    }
    let parent_flags =
        PageTableFlags::PRESENT | PageTableFlags::WRITABLE | PageTableFlags::USER_ACCESSIBLE;

    let seg_start = ph.vaddr;
    let seg_end = seg_start + ph.mem_size as u64; // exclusive
    let first_page = Page::<Size4KiB>::containing_address(VirtAddr::new(seg_start));
    let last_page = Page::<Size4KiB>::containing_address(VirtAddr::new(seg_end - 1));

    for page in Page::range_inclusive(first_page, last_page) {
        let frame = frame_allocator
            .allocate_frame()
            .ok_or(ElfError::OutOfFrames)?;

        // SAFETY: `frame` is fresh and unused; `page` is a user virtual address in
        // an empty slot of this private space, so nothing is aliased. Mapping it
        // user-accessible and executable is intentional — this is user code/data.
        // The space is inactive, so we `ignore()` the TLB flush; switching CR3 to
        // run the program flushes the whole TLB anyway.
        unsafe {
            mapper
                .map_to_with_table_flags(page, frame, page_flags, parent_flags, frame_allocator)
                .map_err(|_| ElfError::MapFailed)?
                .ignore();
        }

        // Populate the frame through the physical-memory window — the user virtual
        // address is unreachable until CR3 points at this space.
        let window = (physical_memory_offset + frame.start_address().as_u64()).as_mut_ptr::<u8>();
        let page_base = page.start_address().as_u64();
        let file_vstart = seg_start;
        let file_vend = seg_start + ph.file_size as u64; // exclusive

        // The bytes of this page that the file backs; the rest stays zero (.bss).
        let copy_start = core::cmp::max(page_base, file_vstart);
        let copy_end = core::cmp::min(page_base + 4096, file_vend);

        // SAFETY: `window` addresses this fresh 4 KiB frame through the physical
        // memory window; every write below stays within those 4 KiB, and nothing
        // else references the frame.
        unsafe {
            for i in 0..4096usize {
                window.add(i).write(0);
            }
            if copy_start < copy_end {
                let src_off = ph.file_offset + (copy_start - file_vstart) as usize;
                let len = (copy_end - copy_start) as usize;
                let dst_off = (copy_start - page_base) as usize;
                let src = elf_bytes
                    .get(src_off..src_off + len)
                    .ok_or(ElfError::BadSegment)?;
                for (i, &byte) in src.iter().enumerate() {
                    window.add(dst_off + i).write(byte);
                }
            }
        }
    }

    Ok(())
}

// --- demo ELF + boot verification ------------------------------------------

/// Build a tiny but valid ELF64 executable for the loader to chew on.
///
/// One `PT_LOAD` segment covers the whole file and asks to be loaded at
/// [`USER_LOAD_BASE`]; the entry point sits just past the headers. The code (the
/// same `write`-then-`exit` sequence the Stage 10 ring 3 program used) and its
/// message follow. Layout within the file / segment:
///
/// ```text
///   [0   .. 64 )  ELF header
///   [64  .. 120)  one program header (PT_LOAD)
///   [120 .. 143)  code  (entry point = USER_LOAD_BASE + 120)
///   [143 ..    )  message string
/// ```
pub fn demo_elf() -> Vec<u8> {
    let msg: &[u8] = b"hello from a loaded ELF, in its own address space\n";
    let msg_offset = DEMO_ENTRY_OFFSET + DEMO_CODE_LEN; // 143
    let total = msg_offset + msg.len();

    // The code references the message at its final virtual address.
    let code = crate::usermode::build_user_program(USER_LOAD_BASE + msg_offset as u64, msg.len());

    let mut v = alloc::vec![0u8; total];

    // --- ELF header (Elf64_Ehdr) ---
    v[0..4].copy_from_slice(b"\x7FELF");
    v[4] = 2; // EI_CLASS = ELFCLASS64
    v[5] = 1; // EI_DATA  = ELFDATA2LSB
    v[6] = 1; // EI_VERSION
    v[16..18].copy_from_slice(&2u16.to_le_bytes()); // e_type = ET_EXEC
    v[18..20].copy_from_slice(&0x3Eu16.to_le_bytes()); // e_machine = x86-64
    v[20..24].copy_from_slice(&1u32.to_le_bytes()); // e_version
    v[24..32].copy_from_slice(&(USER_LOAD_BASE + DEMO_ENTRY_OFFSET as u64).to_le_bytes()); // e_entry
    v[32..40].copy_from_slice(&64u64.to_le_bytes()); // e_phoff
    v[52..54].copy_from_slice(&64u16.to_le_bytes()); // e_ehsize
    v[54..56].copy_from_slice(&56u16.to_le_bytes()); // e_phentsize
    v[56..58].copy_from_slice(&1u16.to_le_bytes()); // e_phnum

    // --- program header (Elf64_Phdr) at offset 64 ---
    let p = 64;
    v[p..p + 4].copy_from_slice(&PT_LOAD.to_le_bytes()); // p_type
    v[p + 4..p + 8].copy_from_slice(&(PF_R | PF_W | PF_X).to_le_bytes()); // p_flags
    v[p + 8..p + 16].copy_from_slice(&0u64.to_le_bytes()); // p_offset (segment = whole file)
    v[p + 16..p + 24].copy_from_slice(&USER_LOAD_BASE.to_le_bytes()); // p_vaddr
    v[p + 24..p + 32].copy_from_slice(&USER_LOAD_BASE.to_le_bytes()); // p_paddr
    v[p + 32..p + 40].copy_from_slice(&(total as u64).to_le_bytes()); // p_filesz
    v[p + 40..p + 48].copy_from_slice(&(total as u64).to_le_bytes()); // p_memsz
    v[p + 48..p + 56].copy_from_slice(&0x1000u64.to_le_bytes()); // p_align

    // --- code + message ---
    v[DEMO_ENTRY_OFFSET..DEMO_ENTRY_OFFSET + DEMO_CODE_LEN].copy_from_slice(&code);
    v[msg_offset..msg_offset + msg.len()].copy_from_slice(msg);

    v
}

/// Set once the boot demo has loaded the demo ELF into a fresh space and verified
/// the entry's bytes. Read by the Stage 11b test.
static ELF_LOAD_OK: AtomicBool = AtomicBool::new(false);

/// Whether the boot demo loaded and verified the demo ELF.
pub fn elf_load_ok() -> bool {
    ELF_LOAD_OK.load(Ordering::SeqCst)
}

/// Stage 11b demo: build the demo ELF, load it into a fresh address space, and
/// verify by translating the entry point in that space and reading the code back.
pub fn demo_load_elf(
    frame_allocator: &mut impl FrameAllocator<Size4KiB>,
    physical_memory_offset: VirtAddr,
) -> UserImage {
    let bytes = demo_elf();

    // Log what the parser sees in the ELF (entry + each loadable segment's perms).
    if let Ok(elf) = ElfFile::parse(&bytes) {
        serial_println!("[elf] parsed ELF64: entry {:#x}", elf.entry());
        for (i, ph) in elf.load_segments().enumerate() {
            serial_println!(
                "[elf]   PT_LOAD[{}]: vaddr {:#x}, file {} B / mem {} B, perms {}{}{}",
                i,
                ph.vaddr,
                ph.file_size,
                ph.mem_size,
                if ph.is_readable() { "r" } else { "-" },
                if ph.is_writable() { "w" } else { "-" },
                if ph.is_executable() { "x" } else { "-" },
            );
        }
    }

    let mut image =
        load(&bytes, frame_allocator, physical_memory_offset).expect("failed to load the demo ELF");
    let entry = image.entry();
    serial_println!(
        "[elf] loaded ELF64 into a new address space (L4 at {:?}); entry {:?}",
        image.space.l4_frame().start_address(),
        entry,
    );

    // Verify the entry page landed where the ELF asked, with the right bytes.
    // Translate the entry virtual address *in the loaded (inactive) space* by
    // walking its tables through the offset mapper, then read the code back through
    // the physical-memory window and compare to what we wrote into the file.
    let ok = {
        let mapper = image.space.mapper(physical_memory_offset);
        match mapper.translate_addr(entry) {
            Some(phys) => {
                let window = (physical_memory_offset + phys.as_u64()).as_ptr::<u8>();
                let expected = &bytes[DEMO_ENTRY_OFFSET..DEMO_ENTRY_OFFSET + DEMO_CODE_LEN];
                // SAFETY: `window` points at the entry's just-mapped frame through the
                // physical-memory window; reading DEMO_CODE_LEN bytes stays in the frame.
                let matches =
                    (0..DEMO_CODE_LEN).all(|i| unsafe { window.add(i).read() } == expected[i]);
                serial_println!(
                    "[elf] entry {:?} -> {:?}; first opcode {:#04x}; code read-back matches = {}",
                    entry,
                    phys,
                    expected[0],
                    matches,
                );
                matches
            }
            None => {
                serial_println!("[elf] entry {:?} is not mapped in the loaded space!", entry);
                false
            }
        }
    }; // drop the mapper, releasing the &mut borrow of `image.space`

    ELF_LOAD_OK.store(ok, Ordering::SeqCst);
    image
}

// --- Stage 12a: running a loaded program in ring 3 -------------------------

/// The kernel's CR3 (frame address + flags), saved by [`run`] so the resume
/// continuation can switch back. The user program runs on its own CR3, but the
/// kernel is mapped there too, so the continuation reaches this code either way.
static KERNEL_L4_ADDR: AtomicU64 = AtomicU64::new(0);
static KERNEL_L4_FLAGS: AtomicU64 = AtomicU64::new(0);
/// The L4 the last [`run`] executed a user program on — for the Stage 12a test.
static RAN_USER_L4_ADDR: AtomicU64 = AtomicU64::new(0);

/// Run a loaded program in ring 3 on its own address space; never returns to the
/// caller.
///
/// Remembers the kernel's CR3 (so the continuation can switch back), switches CR3
/// to the image, and enters ring 3 at the program's entry on its user stack. When
/// the program calls `exit` (or the timer catches it spinning), the kernel resumes
/// at `resume` — which **must** call [`return_to_kernel_space`] first, before it
/// touches anything mapped only in the kernel's address space.
pub fn run(image: &UserImage, resume: fn() -> !) -> ! {
    let kernel = Cr3::read();
    KERNEL_L4_ADDR.store(kernel.0.start_address().as_u64(), Ordering::SeqCst);
    KERNEL_L4_FLAGS.store(kernel.1.bits(), Ordering::SeqCst);
    RAN_USER_L4_ADDR.store(
        image.space.l4_frame().start_address().as_u64(),
        Ordering::SeqCst,
    );

    serial_println!(
        "[process] running user program: CR3 {:?} -> {:?}, entry {:?}, stack top {:?}",
        kernel.0.start_address(),
        image.space.l4_frame().start_address(),
        image.entry,
        image.user_stack_top,
    );

    // SAFETY: `image.space` is a clone of the kernel space with the program mapped
    // into empty slots, so it maps the running kernel; switching CR3 to it is sound.
    unsafe {
        image.space.activate();
    }
    usermode::enter(image.entry, image.user_stack_top, resume);
}

/// Switch CR3 back to the kernel address space saved by [`run`]. Call once, at the
/// very start of the `resume` continuation, before using kernel-only mappings.
pub fn return_to_kernel_space() {
    let frame =
        PhysFrame::containing_address(PhysAddr::new(KERNEL_L4_ADDR.load(Ordering::SeqCst)));
    let flags = Cr3Flags::from_bits_truncate(KERNEL_L4_FLAGS.load(Ordering::SeqCst));
    // SAFETY: this is the CR3 that was active before `run` switched away; it maps
    // the running kernel, so restoring it is sound.
    unsafe {
        memory::restore_address_space((frame, flags));
    }
}

/// The L4 frame the last [`run`] executed a user program on (0 if none). For the
/// Stage 12a test: it must differ from the kernel's, proving the program ran in a
/// separate address space.
pub fn last_user_run_l4() -> u64 {
    RAN_USER_L4_ADDR.load(Ordering::SeqCst)
}

/// The kernel's L4 frame, saved by the last [`run`]. For the Stage 12a test.
pub fn kernel_l4() -> u64 {
    KERNEL_L4_ADDR.load(Ordering::SeqCst)
}
