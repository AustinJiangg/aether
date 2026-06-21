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
use spin::Mutex;
use x86_64::registers::control::{Cr3, Cr3Flags};
use x86_64::structures::paging::{
    FrameAllocator, Mapper, OffsetPageTable, Page, PageTableFlags, PhysFrame, Size4KiB, Translate,
};
use x86_64::{PhysAddr, VirtAddr};

use crate::elf::{ElfError, ElfFile, ProgramHeader, PF_R, PF_W, PF_X, PT_LOAD};
use crate::interrupts::TrapFrame;
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
/// How many `write` + busy-spin + `yield` rounds the demo program runs before `exit`.
const DEMO_ITERATIONS: usize = 3;
/// Iterations of the ring-3 busy-spin each round performs (a `dec rcx; jnz` loop, see
/// [`build_looping_program`]). Sized so the spin lasts well over one ~55 ms timer tick
/// under QEMU, so a tick reliably lands mid-spin and *preempts* the process — proving
/// timer-driven scheduling, not just cooperative `yield`. Tune if boot drags or no
/// preemption is observed.
const DEMO_SPIN: u32 = 50_000_000;
/// Bytes of machine code per round: a 17-byte `write`, a 12-byte busy-spin, and a
/// 4-byte `yield` (see [`build_looping_program`]).
const DEMO_ROUND_LEN: usize = 17 + 12 + 4;
/// Length of the hand-assembled demo program: the rounds plus a 6-byte `exit`.
const DEMO_CODE_LEN: usize = DEMO_ITERATIONS * DEMO_ROUND_LEN + 6;

/// Exit code the Stage 12 wait-demo child passes to `exit`, for its parent's `wait` to
/// collect.
const CHILD_EXIT_CODE: u8 = 42;
/// Length of the wait-demo child's code: a 17-byte `write` + a 6-byte `exit`.
const CHILD_CODE_LEN: usize = 17 + 6;
/// Length of the wait-demo parent's code: `write` + `wait` (4 B) + `write` + `exit`.
const PARENT_CODE_LEN: usize = 17 + 4 + 17 + 6;

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

// --- ring 3 machine-code emitters (shared by the demo program builders) ----
//
// All hand-assembled; see the per-line opcode comments. Each demo program is a flat
// sequence of these, speaking the stack-based syscall ABI: push the arguments, push the
// syscall number, `int 0x80`. The pushes are never popped — the user stack just drifts
// down a little, well within its 16 KiB.

/// `write(msg_ptr, msg_len)` — 17 bytes. Reloads its arguments from immediates, so it
/// needs no incoming register value to survive a context switch.
fn emit_write(code: &mut Vec<u8>, msg_ptr: u64, msg_len: u8) {
    code.extend_from_slice(&[0x6A, msg_len]); // push msg_len
    code.extend_from_slice(&[0x48, 0xB8]); // mov rax, imm64 ...
    code.extend_from_slice(&msg_ptr.to_le_bytes()); // ... = msg_ptr
    code.push(0x50); // push rax
    code.extend_from_slice(&[0x6A, crate::syscall::SYS_WRITE as u8]); // push SYS_WRITE
    code.extend_from_slice(&[0xCD, 0x80]); // int 0x80
}

/// A zero-argument syscall — `push number; int 0x80` (4 bytes). Used for `yield`, and
/// for `wait` (whose result the kernel returns in rax, so the program reads rax after).
fn emit_syscall0(code: &mut Vec<u8>, number: u8) {
    code.extend_from_slice(&[0x6A, number]); // push number
    code.extend_from_slice(&[0xCD, 0x80]); // int 0x80
}

/// `exit(exit_code)` — 6 bytes. Never returns.
fn emit_exit(code: &mut Vec<u8>, exit_code: u8) {
    code.extend_from_slice(&[0x6A, exit_code]); // push exit_code
    code.extend_from_slice(&[0x6A, crate::syscall::SYS_EXIT as u8]); // push SYS_EXIT
    code.extend_from_slice(&[0xCD, 0x80]); // int 0x80
}

/// A busy-spin of `count` iterations — `mov rcx, count; dec rcx; jnz` (12 bytes). A
/// ring-3 delay long enough that a ~55 ms timer tick lands mid-spin and *preempts* the
/// process (Stage 12c-3). `rcx` is live throughout, so a correct preemption must save
/// and restore it — were `rcx` lost the spin would mis-count and likely never
/// terminate, hanging boot, which makes the spin a built-in check of the full-register
/// `TrapFrame` switch.
fn emit_spin(code: &mut Vec<u8>, count: u32) {
    code.extend_from_slice(&[0x48, 0xC7, 0xC1]); // mov rcx, imm32 (sign-extended to 64)
    code.extend_from_slice(&count.to_le_bytes());
    code.extend_from_slice(&[0x48, 0xFF, 0xC9]); // dec rcx
    code.extend_from_slice(&[0x75, 0xFB]); // jnz -5 -> back to `dec rcx`
}

/// Hand-assemble the Stage 12c interleaving demo: `iterations` rounds of
/// `write(msg); busy_spin(); yield()`, then `exit(0)`. Two of these interleave via both
/// timer preemption (during the spin) and cooperative `yield`.
fn build_looping_program(msg_ptr: u64, msg_len: u8, iterations: usize) -> Vec<u8> {
    let mut code = Vec::new();
    for _ in 0..iterations {
        emit_write(&mut code, msg_ptr, msg_len);
        emit_spin(&mut code, DEMO_SPIN);
        emit_syscall0(&mut code, crate::syscall::SYS_YIELD as u8);
    }
    emit_exit(&mut code, 0);
    debug_assert_eq!(code.len(), iterations * DEMO_ROUND_LEN + 6);
    code
}

/// Hand-assemble the Stage 12 wait-demo *child*: `write(msg); exit(CHILD_EXIT_CODE)`.
/// Produces exactly [`CHILD_CODE_LEN`] bytes.
fn build_child(msg_ptr: u64, msg_len: u8) -> Vec<u8> {
    let mut code = Vec::new();
    emit_write(&mut code, msg_ptr, msg_len);
    emit_exit(&mut code, CHILD_EXIT_CODE);
    debug_assert_eq!(code.len(), CHILD_CODE_LEN);
    code
}

/// Hand-assemble the Stage 12 wait-demo *parent*: `write(msg); wait(); write(msg);
/// exit(0)`. Writing the same message before and after `wait()` makes the two lines
/// bracket the child's output — visibly proving the parent blocked, then resumed once
/// the child exited. Produces exactly [`PARENT_CODE_LEN`] bytes.
fn build_parent(msg_ptr: u64, msg_len: u8) -> Vec<u8> {
    let mut code = Vec::new();
    emit_write(&mut code, msg_ptr, msg_len);
    emit_syscall0(&mut code, crate::syscall::SYS_WAIT as u8);
    emit_write(&mut code, msg_ptr, msg_len);
    emit_exit(&mut code, 0);
    debug_assert_eq!(code.len(), PARENT_CODE_LEN);
    code
}

/// Assemble a tiny but valid ELF64 `ET_EXEC` from raw `code` and a `msg` string.
///
/// One `PT_LOAD` segment covers the whole file, loaded at [`USER_LOAD_BASE`]; the entry
/// sits just past the headers. The caller must have built `code` to reference `msg` at
/// its final virtual address — `USER_LOAD_BASE + DEMO_ENTRY_OFFSET + code.len()` (see
/// [`msg_vaddr`]). Layout within the file / segment:
///
/// ```text
///   [0   .. 64 )            ELF header
///   [64  .. 120)            one program header (PT_LOAD)
///   [120 .. 120+code)       code  (entry point = USER_LOAD_BASE + 120)
///   [120+code ..   )        message string
/// ```
fn build_elf(code: &[u8], msg: &[u8]) -> Vec<u8> {
    let msg_offset = DEMO_ENTRY_OFFSET + code.len();
    let total = msg_offset + msg.len();

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
    v[DEMO_ENTRY_OFFSET..DEMO_ENTRY_OFFSET + code.len()].copy_from_slice(code);
    v[msg_offset..msg_offset + msg.len()].copy_from_slice(msg);

    v
}

/// The virtual address a program's message lands at, given its code length: the message
/// follows the code, which follows the headers.
fn msg_vaddr(code_len: usize) -> u64 {
    USER_LOAD_BASE + (DEMO_ENTRY_OFFSET + code_len) as u64
}

/// Build the Stage 12c interleaving demo ELF (the `write` + busy-spin + `yield` loop).
pub fn demo_elf(msg: &[u8]) -> Vec<u8> {
    let code = build_looping_program(msg_vaddr(DEMO_CODE_LEN), msg.len() as u8, DEMO_ITERATIONS);
    build_elf(&code, msg)
}

/// Build the Stage 12 wait-demo *child* ELF (`write`; `exit(CHILD_EXIT_CODE)`).
fn child_elf() -> Vec<u8> {
    let msg = b"  child: running, then exiting\n";
    let code = build_child(msg_vaddr(CHILD_CODE_LEN), msg.len() as u8);
    build_elf(&code, msg)
}

/// Build the Stage 12 wait-demo *parent* ELF (`write`; `wait`; `write`; `exit(0)`).
fn parent_elf() -> Vec<u8> {
    let msg = b"parent: before/after wait()\n";
    let code = build_parent(msg_vaddr(PARENT_CODE_LEN), msg.len() as u8);
    build_elf(&code, msg)
}

/// Stage 12 boot demo for `wait`: load and spawn a parent that `wait`s for its child.
/// The child `write`s then `exit`s with [`CHILD_EXIT_CODE`]; the parent blocks in
/// `wait` until then and collects the code (recorded for the test). Returns the two
/// process ids. Spawned alongside the Stage 12c interleaving workers, so all run
/// together under the scheduler — the wait logic matches a parent to *its* child by id,
/// regardless of the other unrelated processes.
pub fn spawn_wait_demo(
    frame_allocator: &mut impl FrameAllocator<Size4KiB>,
    physical_memory_offset: VirtAddr,
) -> (u64, u64) {
    let parent_img = load(&parent_elf(), frame_allocator, physical_memory_offset)
        .expect("failed to load the wait-demo parent");
    let child_img = load(&child_elf(), frame_allocator, physical_memory_offset)
        .expect("failed to load the wait-demo child");
    let parent_id = spawn(parent_img, None);
    let child_id = spawn(child_img, Some(parent_id));
    serial_println!(
        "[sched] wait-demo: spawned parent {} and its child {}",
        parent_id,
        child_id
    );
    (parent_id, child_id)
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
    msg: &[u8],
    frame_allocator: &mut impl FrameAllocator<Size4KiB>,
    physical_memory_offset: VirtAddr,
) -> UserImage {
    let bytes = demo_elf(msg);

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

// --- Stage 12a/12b: running loaded programs in ring 3 ----------------------

/// The kernel's CR3 (frame address + flags), saved when the scheduler starts so the
/// resume continuation can switch back. User programs run on their own CR3, but the
/// kernel is mapped there too, so the continuation reaches this code either way.
static KERNEL_L4_ADDR: AtomicU64 = AtomicU64::new(0);
static KERNEL_L4_FLAGS: AtomicU64 = AtomicU64::new(0);
/// The L4 the most recent process ran on — for the "ran in its own space" test.
static RAN_USER_L4_ADDR: AtomicU64 = AtomicU64::new(0);
/// How many user processes have exited — for the Stage 12b test.
static PROCESSES_EXITED: AtomicU64 = AtomicU64::new(0);
/// How many times a process has `yield`ed — for the Stage 12b interleaving test.
static PROCESSES_YIELDED: AtomicU64 = AtomicU64::new(0);
/// How many times the timer has *preempted* a running user process — Stage 12c-3 test.
static PROCESSES_PREEMPTED: AtomicU64 = AtomicU64::new(0);
/// How many times a parent collected an exited child via `wait` — Stage 12 test.
static PROCESSES_WAITED: AtomicU64 = AtomicU64::new(0);
/// Exit code the most recent `wait` returned (`u64::MAX` = none yet) — Stage 12 test.
static LAST_WAITED_CODE: AtomicU64 = AtomicU64::new(u64::MAX);

/// A user process: a unique id, its loaded image (address space, entry, stack), its
/// saved execution context (`context`), and its `parent`. As of Stage 12c-2 the context
/// is a full [`TrapFrame`]: every general-purpose register plus the interrupt frame
/// (instruction/stack pointers, flags, selectors). Saving the GP registers too — not
/// just the interrupt frame — is what makes a process resumable after being switched
/// out at *any* instruction, the prerequisite for timer preemption (Stage 12c-3).
struct Process {
    id: u64,
    image: UserImage,
    context: TrapFrame,
    /// The process that may collect this one's exit code via `wait` (Stage 12); `None`
    /// for the root processes the kernel spawns directly at boot.
    parent: Option<u64>,
}

/// An exited child whose parent has not yet `wait`ed for it — a "zombie". We keep only
/// the ids and exit code (the child's image/space is dropped when it exits); a later
/// `wait` from the parent collects the code. (Stage 12.)
struct Zombie {
    parent: u64,
    child: u64,
    code: u64,
}

/// A minimal round-robin scheduler: a FIFO queue of ready processes plus the one
/// currently running. Dispatch is driven both by the voluntary `yield`/`exit` syscalls
/// (see [`on_user_yield`] / [`on_user_exit`]) and, since Stage 12c-3, by the timer
/// preempting a running process ([`on_timer_tick`]) — processes now run with interrupts
/// *on*, so a switch can happen at any instruction, not only at voluntary points.
///
/// Stage 12 adds `wait`: a parent blocking on a child's exit goes into `blocked` (out of
/// the round-robin) until a child exits and wakes it; a child that exits *before* its
/// parent waits leaves a [`Zombie`] in `zombies` for the parent to collect later.
struct Scheduler {
    ready: Vec<Process>,
    current: Option<Process>,
    /// Processes blocked in `wait`, waiting for a child to exit (Stage 12).
    blocked: Vec<Process>,
    /// Exited children whose parents have not yet collected them via `wait` (Stage 12).
    zombies: Vec<Zombie>,
    next_id: u64,
}

static SCHEDULER: Mutex<Scheduler> = Mutex::new(Scheduler {
    ready: Vec::new(),
    current: None,
    blocked: Vec::new(),
    zombies: Vec::new(),
    next_id: 1,
});

/// Add a loaded program to the scheduler's ready queue; returns its process id.
///
/// `parent` is the process that may `wait` for this one (`None` for a root process the
/// kernel spawns at boot). Its initial context starts at the program's entry on a fresh
/// user stack with every general-purpose register zero (see [`TrapFrame::new`]).
pub fn spawn(image: UserImage, parent: Option<u64>) -> u64 {
    let mut sched = SCHEDULER.lock();
    let id = sched.next_id;
    sched.next_id += 1;
    let iframe = usermode::initial_user_frame(image.entry, image.user_stack_top);
    sched.ready.push(Process {
        id,
        image,
        context: TrapFrame::new(iframe),
        parent,
    });
    id
}

/// Start the cooperative scheduler: run the spawned processes in ring 3, one after
/// another, each on its own address space. Never returns to the caller. When the
/// last process exits, the kernel resumes at `resume` — which **must** call
/// [`return_to_kernel_space`] first, before touching kernel-only mappings.
pub fn run(resume: fn() -> !) -> ! {
    // Remember the kernel's CR3 so the eventual return can switch back.
    let kernel = Cr3::read();
    KERNEL_L4_ADDR.store(kernel.0.start_address().as_u64(), Ordering::SeqCst);
    KERNEL_L4_FLAGS.store(kernel.1.bits(), Ordering::SeqCst);

    let started = {
        let mut sched = SCHEDULER.lock();
        if sched.ready.is_empty() {
            None
        } else {
            let first = sched.ready.remove(0);
            let entry = first.image.entry;
            let stack = first.image.user_stack_top;
            let l4 = first.image.space.l4_frame();
            RAN_USER_L4_ADDR.store(l4.start_address().as_u64(), Ordering::SeqCst);
            serial_println!(
                "[sched] starting process {} on L4 {:?} ({} more queued)",
                first.id,
                l4.start_address(),
                sched.ready.len(),
            );
            // SAFETY: the image clones the kernel, so its space maps the running
            // kernel; switching CR3 to it is sound.
            unsafe { first.image.space.activate() };
            sched.current = Some(first);
            Some((entry, stack))
        }
    };

    match started {
        Some((entry, stack)) => usermode::enter(entry, stack, resume),
        None => {
            serial_println!("[sched] no processes to run");
            resume()
        }
    }
}

/// Pop the next ready process, switch CR3 to its address space, make it current, and
/// return its `(id, saved context)` so the caller can resume it. `None` if the ready
/// queue is empty. The caller must hold the scheduler lock.
fn activate_next(sched: &mut Scheduler) -> Option<(u64, TrapFrame)> {
    if sched.ready.is_empty() {
        return None;
    }
    let next = sched.ready.remove(0);
    let id = next.id;
    let context = next.context;
    RAN_USER_L4_ADDR.store(
        next.image.space.l4_frame().start_address().as_u64(),
        Ordering::SeqCst,
    );
    // SAFETY: the next image clones the kernel, so its space maps the running kernel;
    // switching CR3 to it from the handler is sound — the rsp0 stack holding the
    // TrapFrame is mapped identically in every address space.
    unsafe { next.image.space.activate() };
    sched.current = Some(next);
    Some((id, context))
}

/// Called by the `yield` syscall: save the running process's full register context,
/// put it back at the end of the ready queue, and switch to the next one. With two
/// processes this alternates them, interleaving their output. `tf` is the caller's
/// [`TrapFrame`] on the kernel stack; rewriting it makes the syscall stub's `iretq`
/// resume a *different* process.
pub fn on_user_yield(tf: &mut TrapFrame) {
    PROCESSES_YIELDED.fetch_add(1, Ordering::SeqCst);

    let next = {
        let mut sched = SCHEDULER.lock();
        let yielded_id = if let Some(mut current) = sched.current.take() {
            current.context = *tf; // save the caller's full context to resume later
            let id = current.id;
            sched.ready.push(current); // back of the queue (round-robin)
            id
        } else {
            0
        };
        let next = activate_next(&mut sched);
        if let Some((id, _)) = next {
            serial_println!("[sched] process {} yielded; switching to process {}", yielded_id, id);
        }
        next
    };

    // `next` is always `Some` here (we just re-queued the yielding process), but the
    // match keeps it total: with nothing to run we simply resume the same context.
    if let Some((_, context)) = next {
        *tf = context; // restore the next process's full register context
    }
}

/// Called by the `exit` syscall when a ring 3 process terminates (see `syscall.rs`).
/// Drops the finished process; if it has a parent, delivers its exit code to that
/// parent — waking it if it is blocked in `wait` (returning the code in rax), otherwise
/// leaving a [`Zombie`] for a later `wait` to collect. Then switches to the next ready
/// process, or resumes the kernel if none remain.
pub fn on_user_exit(tf: &mut TrapFrame, code: u64) {
    PROCESSES_EXITED.fetch_add(1, Ordering::SeqCst);

    let next = {
        let mut sched = SCHEDULER.lock();
        // Take (and drop) the exiting process, remembering its id and parent.
        let (finished_id, parent) = match sched.current.take() {
            Some(p) => (p.id, p.parent),
            None => (0, None),
        };

        // Deliver the exit code to the parent, if there is one.
        if let Some(parent_id) = parent {
            if let Some(idx) = sched.blocked.iter().position(|p| p.id == parent_id) {
                // The parent is blocked in wait(): wake it, returning `code` in rax.
                let mut waiting = sched.blocked.remove(idx);
                waiting.context.rax = code;
                sched.ready.push(waiting);
                PROCESSES_WAITED.fetch_add(1, Ordering::SeqCst);
                LAST_WAITED_CODE.store(code, Ordering::SeqCst);
                serial_println!("[sched] child {} woke waiting parent {} (code {})", finished_id, parent_id, code);
            } else {
                // The parent has not waited yet: leave a zombie for it to collect.
                sched.zombies.push(Zombie { parent: parent_id, child: finished_id, code });
                serial_println!("[sched] child {} became a zombie for parent {} (code {})", finished_id, parent_id, code);
            }
        }

        let next = activate_next(&mut sched);
        match next {
            Some((id, _)) => serial_println!(
                "[sched] process {} exited (code {}); switching to process {}",
                finished_id,
                code,
                id,
            ),
            None => serial_println!(
                "[sched] process {} exited (code {}); none left, returning to the kernel",
                finished_id,
                code,
            ),
        }
        next
    };

    match next {
        // Resume the next process's full context (the dropped one is gone for good).
        Some((_, context)) => *tf = context,
        // No process left: rewrite the interrupt frame to resume the kernel instead.
        None => usermode::resume_kernel(&mut tf.iframe),
    }
}

/// Called by the `wait` syscall (ring 3): block the caller until one of its children
/// exits, then return that child's exit code. Three cases:
/// - a child already exited (a [`Zombie`] is queued): collect it and resume immediately;
/// - the caller has a live child: save its context, move it to `blocked`, and switch to
///   another process — the child's eventual `exit` ([`on_user_exit`]) wakes the parent;
/// - the caller has no children: return `u64::MAX` (-1) immediately.
///
/// Unlike the stack-based ABI of the other syscalls, `wait` returns its result in
/// **rax**. The kernel often delivers it asynchronously, from a child's `exit` running
/// in a *different* address space where the parent's user stack is not reachable — but
/// the parent's saved `TrapFrame.rax` always is. The demo's parent reads rax after the
/// `int 0x80`.
pub fn on_user_wait(tf: &mut TrapFrame) {
    let mut sched = SCHEDULER.lock();
    let parent = match sched.current.take() {
        Some(p) => p,
        None => return, // no current process — should not happen from a ring 3 syscall
    };
    let pid = parent.id;

    // (1) A child already exited? Collect the zombie and resume the parent immediately.
    if let Some(idx) = sched.zombies.iter().position(|z| z.parent == pid) {
        let Zombie { child, code, .. } = sched.zombies.remove(idx);
        sched.current = Some(parent); // the parent keeps running
        PROCESSES_WAITED.fetch_add(1, Ordering::SeqCst);
        LAST_WAITED_CODE.store(code, Ordering::SeqCst);
        serial_println!("[sched] process {} waited; reaped zombie child {} (code {})", pid, child, code);
        tf.rax = code; // wait() returns the child's exit code in rax
        return;
    }

    // (2) Any live child to block on? (a child sits in `ready` or `blocked`.)
    let has_live_child = sched.ready.iter().any(|p| p.parent == Some(pid))
        || sched.blocked.iter().any(|p| p.parent == Some(pid));
    if !has_live_child {
        sched.current = Some(parent);
        serial_println!("[sched] process {} called wait with no children; returning -1", pid);
        tf.rax = u64::MAX; // -1: nothing to wait for
        return;
    }

    // (3) Block the parent until a child exits.
    let mut parent = parent;
    parent.context = *tf; // resume point; rax is filled in when the child exits
    serial_println!("[sched] process {} blocked in wait", pid);
    sched.blocked.push(parent);
    match activate_next(&mut sched) {
        Some((_, context)) => *tf = context,
        None => {
            // Nothing else to run while the parent waits — a deadlock in general, but in
            // the demo a child is always ready here, so this is defensive.
            serial_println!("[sched] nothing to run while process {} waits; resuming kernel", pid);
            usermode::resume_kernel(&mut tf.iframe);
        }
    }
}

/// Called from the timer interrupt when it fires while a *user* process runs in ring 3
/// — Stage 12c-3 preemption. Saves the running process's full register context,
/// round-robins it to the back of the ready queue, switches to the next ready process
/// (its CR3 and full context), and rewrites `tf` so the timer stub's `iretq` resumes
/// *that* process. Unlike `yield`, the preempted process does not cooperate — it is
/// switched out wherever the tick happened to strike.
///
/// A no-op (the same process simply resumes) when a switch is impossible:
/// - `try_lock`, never `lock`: at boot `spawn`/`run` briefly hold this lock with
///   interrupts enabled, so a tick landing then must skip rather than deadlock. (Once a
///   process actually runs in ring 3 no kernel code holds the lock, so it is free.)
/// - if `ready` is empty there is only one process, so nothing to switch to.
pub fn on_timer_tick(tf: &mut TrapFrame) {
    let mut sched = match SCHEDULER.try_lock() {
        Some(sched) => sched,
        None => return,
    };
    if sched.current.is_none() || sched.ready.is_empty() {
        return;
    }

    PROCESSES_PREEMPTED.fetch_add(1, Ordering::SeqCst);

    // Save the preempted process's full context, then round-robin it to the back.
    let preempted_id = {
        let mut current = sched.current.take().expect("current is_some, checked above");
        current.context = *tf;
        let id = current.id;
        sched.ready.push(current);
        id
    };

    // `ready` was non-empty and we just pushed onto it, so this is always `Some`.
    let (next_id, context) = activate_next(&mut sched).expect("ready queue is non-empty");
    serial_println!(
        "[sched] preempted process {}; switching to process {}",
        preempted_id,
        next_id
    );
    *tf = context;
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

/// How many user processes have exited since boot. For the Stage 12b test.
pub fn processes_exited() -> u64 {
    PROCESSES_EXITED.load(Ordering::SeqCst)
}

/// How many times a process has `yield`ed since boot. For the Stage 12b test.
pub fn processes_yielded() -> u64 {
    PROCESSES_YIELDED.load(Ordering::SeqCst)
}

/// How many times the timer preempted a running user process since boot. For the Stage
/// 12c-3 test.
pub fn processes_preempted() -> u64 {
    PROCESSES_PREEMPTED.load(Ordering::SeqCst)
}

/// How many times a parent collected an exited child via `wait`. For the Stage 12 test.
pub fn processes_waited() -> u64 {
    PROCESSES_WAITED.load(Ordering::SeqCst)
}

/// The exit code the most recent `wait` returned (`u64::MAX` if none yet). For the
/// Stage 12 test.
pub fn last_waited_code() -> u64 {
    LAST_WAITED_CODE.load(Ordering::SeqCst)
}
