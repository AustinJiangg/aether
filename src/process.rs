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

use core::sync::atomic::{AtomicBool, Ordering};

use alloc::vec::Vec;
use x86_64::structures::paging::{
    FrameAllocator, Mapper, OffsetPageTable, Page, PageTableFlags, Size4KiB, Translate,
};
use x86_64::VirtAddr;

use crate::elf::{ElfError, ElfFile, ProgramHeader, PF_R, PF_W, PF_X, PT_LOAD};
use crate::memory::AddressSpace;
use crate::serial_println;

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

/// A loaded user program: its private address space and entry point. Stage 12 will
/// grow this into a schedulable process (a saved register context, a user stack).
pub struct UserImage {
    space: AddressSpace,
    entry: VirtAddr,
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
    } // drop the mapper, releasing the &mut borrow of `space`

    Ok(UserImage { space, entry })
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
) {
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

    let mut image = match load(&bytes, frame_allocator, physical_memory_offset) {
        Ok(image) => image,
        Err(err) => {
            serial_println!("[elf] load failed: {:?}", err);
            return;
        }
    };
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
    let mapper = image.space.mapper(physical_memory_offset);
    let ok = match mapper.translate_addr(entry) {
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
    };

    ELF_LOAD_OK.store(ok, Ordering::SeqCst);
}
