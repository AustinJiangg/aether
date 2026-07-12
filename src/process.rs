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
/// kernel's present L4 slots are 0, 2, 3, 4, 5, 31, 128, 136 — plus 100, the Stage 15
/// Local APIC MMIO — but not 64.)
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
/// Length of the wait-demo parent's code (Stage 12d): `write` + `spawn` (6 B) +
/// `wait` (4 B) + `write` + `exit`. The parent now creates its own child via `spawn`.
const PARENT_CODE_LEN: usize = 17 + 6 + 4 + 17 + 6;
/// Length of the Stage 24c accept-demo code — one ring 3 process playing both ends over loopback:
/// `socket` (5 B, server) + `mov r15,rax` (3 B, stash the listen fd) + `listen` (10 B) + `socket` (5 B,
/// client) + `mov rbx,rax` (3 B, stash the client fd) + `connect` (19 B) + `mov rax,r15` (3 B, restore the
/// listen fd) + `accept` (5 B) + `mov r14,rax` (3 B, stash the accepted fd) + `send` (18 B, on the client
/// fd) + `mov rbx,r14` (3 B, the accepted fd) + `recv` (18 B, on the accepted fd) + dynamic `write` (16 B) +
/// `exit` (6 B).
const ACCEPT_DEMO_CODE_LEN: usize = 5 + 3 + 10 + 5 + 3 + 19 + 3 + 5 + 3 + 18 + 3 + 18 + 16 + 6;
/// Capacity of the socket demo's receive buffer (Stage 24b/24c): plenty for the short message.
const SOCKET_DEMO_RECV_CAP: u8 = 64;

/// How many times [`on_user_recv`] pumps the network waiting for data before giving up (returning 0
/// bytes), and the pause between pumps. Over PHY loopback the echo returns in a handful of pumps; the
/// bound only guards against a silent peer hanging the process (a syscall runs with interrupts off).
const RECV_POLL_ITERS: usize = 2000;
const RECV_POLL_US: u32 = 500;

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

/// `spawn(prog_id)` — 6 bytes (Stage 12d): push the program id, push `SYS_SPAWN`, then
/// `int 0x80`. The kernel loads that program into a fresh process and writes the new
/// child's pid over the number slot; the demo parent ignores the pid (its `wait()` reaps
/// whichever child exits). Like the other emitters it never pops.
fn emit_spawn(code: &mut Vec<u8>, prog_id: u8) {
    code.extend_from_slice(&[0x6A, prog_id]); // push prog_id
    code.extend_from_slice(&[0x6A, crate::syscall::SYS_SPAWN as u8]); // push SYS_SPAWN
    code.extend_from_slice(&[0xCD, 0x80]); // int 0x80
}

/// `socket()` — 5 bytes (Stage 24a). A zero-argument call whose result comes back on the
/// stack (like `getpid`): push the number, `int 0x80`, then `pop rax` so the returned fd
/// lands in `rax`, where [`emit_connect`] expects it.
fn emit_socket(code: &mut Vec<u8>) {
    code.extend_from_slice(&[0x6A, crate::syscall::SYS_SOCKET as u8]); // push SYS_SOCKET
    code.extend_from_slice(&[0xCD, 0x80]); // int 0x80
    code.push(0x58); // pop rax   (rax = fd)
}

/// `connect(fd, dst)` — 19 bytes (Stage 24a). Consumes the fd left in `rax` by
/// [`emit_socket`] and connects it to `dst` (the packed IPv4 address + port). `connect`
/// blocks until the handshake completes and returns its result in `rax`, so nothing is
/// popped here. It stashes the fd in `rsi` first, because materializing the 64-bit `dst`
/// immediate clobbers `rax`; then pushes the args (dst, fd, number) and traps.
fn emit_connect(code: &mut Vec<u8>, dst: u64) {
    code.extend_from_slice(&[0x48, 0x89, 0xC6]); // mov rsi, rax   (stash fd)
    code.extend_from_slice(&[0x48, 0xB8]); // mov rax, imm64 ...
    code.extend_from_slice(&dst.to_le_bytes()); // ... = dst (packed ip:port)
    code.push(0x50); // push rax   (arg2 = dst)
    code.push(0x56); // push rsi   (arg1 = fd)
    code.extend_from_slice(&[0x6A, crate::syscall::SYS_CONNECT as u8]); // push SYS_CONNECT
    code.extend_from_slice(&[0xCD, 0x80]); // int 0x80
}

/// `send(fd, ptr, len)` — 18 bytes (Stage 24b). The socket demo keeps the fd in `rbx` (stashed once
/// after `socket`, and preserved across every syscall by the entry stub), so this reads it from there.
/// A three-argument call: push `len`, `ptr`, `fd` (so the handler finds them at [rsp+24], +16, +8),
/// push the number, trap. `send` returns via the stack ABI, which this program ignores.
fn emit_send(code: &mut Vec<u8>, ptr: u64, len: u8) {
    code.extend_from_slice(&[0x6A, len]); // push len          (arg3)
    code.extend_from_slice(&[0x48, 0xB8]); // mov rax, imm64 ...
    code.extend_from_slice(&ptr.to_le_bytes()); // ... = ptr
    code.push(0x50); // push rax          (arg2 = ptr)
    code.push(0x53); // push rbx          (arg1 = fd)
    code.extend_from_slice(&[0x6A, crate::syscall::SYS_SEND as u8]); // push SYS_SEND
    code.extend_from_slice(&[0xCD, 0x80]); // int 0x80
}

/// `recv(fd, ptr, len)` — 18 bytes (Stage 24b). Same three-argument shape as [`emit_send`], with the fd
/// in `rbx`. `recv` blocks until data arrives and returns the byte count in `rax`, which the demo then
/// hands to [`emit_write_dyn`] as the length to print.
fn emit_recv(code: &mut Vec<u8>, ptr: u64, len: u8) {
    code.extend_from_slice(&[0x6A, len]); // push len          (arg3 = capacity)
    code.extend_from_slice(&[0x48, 0xB8]); // mov rax, imm64 ...
    code.extend_from_slice(&ptr.to_le_bytes()); // ... = ptr
    code.push(0x50); // push rax          (arg2 = ptr)
    code.push(0x53); // push rbx          (arg1 = fd)
    code.extend_from_slice(&[0x6A, crate::syscall::SYS_RECV as u8]); // push SYS_RECV
    code.extend_from_slice(&[0xCD, 0x80]); // int 0x80
}

/// `write(ptr, rax)` — 16 bytes (Stage 24b). A `write` whose length is the value in `rax` (the count a
/// preceding `recv` returned), rather than a compile-time immediate: push `rax` (the length), then `ptr`,
/// then the number. Lets the demo print exactly the bytes it received.
fn emit_write_dyn(code: &mut Vec<u8>, ptr: u64) {
    code.push(0x50); // push rax          (arg2 = len, from recv)
    code.extend_from_slice(&[0x48, 0xB8]); // mov rax, imm64 ...
    code.extend_from_slice(&ptr.to_le_bytes()); // ... = ptr
    code.push(0x50); // push rax          (arg1 = ptr)
    code.extend_from_slice(&[0x6A, crate::syscall::SYS_WRITE as u8]); // push SYS_WRITE
    code.extend_from_slice(&[0xCD, 0x80]); // int 0x80
}

/// `listen(fd, port)` — 10 bytes (Stage 24c). Consumes the fd left in `rax` by the preceding
/// [`emit_socket`] and binds it to `port` as a passive listener. A two-argument call: push `port` (a 32-bit
/// immediate, `arg2`), push `rax` (the fd, `arg1`), push the number, trap. `listen` returns via the stack
/// ABI, which this program ignores — and, crucially, leaves `rax` untouched, so the listen fd survives for a
/// later `accept`.
fn emit_listen(code: &mut Vec<u8>, port: u16) {
    code.push(0x68); // push imm32 ...
    code.extend_from_slice(&(port as u32).to_le_bytes()); // ... = port (arg2)
    code.push(0x50); // push rax          (arg1 = fd)
    code.extend_from_slice(&[0x6A, crate::syscall::SYS_LISTEN as u8]); // push SYS_LISTEN
    code.extend_from_slice(&[0xCD, 0x80]); // int 0x80
}

/// `accept(fd)` — 5 bytes (Stage 24c). Consumes the listen fd in `rax`, blocks until a connection is ready,
/// and returns a **new** fd for it in `rax`. A one-argument call: push `fd`, push the number, trap. The demo
/// then stashes the returned fd for the following `recv`.
fn emit_accept(code: &mut Vec<u8>) {
    code.push(0x50); // push rax          (arg1 = listen fd)
    code.extend_from_slice(&[0x6A, crate::syscall::SYS_ACCEPT as u8]); // push SYS_ACCEPT
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

/// Hand-assemble the wait-demo *parent*, extended for Stage 12d: `write(msg);
/// spawn(PROG_CHILD); wait(); write(msg); exit(0)`. The parent now creates its own child
/// at runtime through the `spawn` syscall — the kernel only spawns the parent. Writing the
/// same message before the spawn and after the wait makes the two lines bracket the
/// child's output, visibly proving the parent ran, created and blocked on the child, then
/// resumed once the child exited. Produces exactly [`PARENT_CODE_LEN`] bytes.
fn build_parent(msg_ptr: u64, msg_len: u8) -> Vec<u8> {
    let mut code = Vec::new();
    emit_write(&mut code, msg_ptr, msg_len);
    emit_spawn(&mut code, PROG_CHILD as u8);
    emit_syscall0(&mut code, crate::syscall::SYS_WAIT as u8);
    emit_write(&mut code, msg_ptr, msg_len);
    emit_exit(&mut code, 0);
    debug_assert_eq!(code.len(), PARENT_CODE_LEN);
    code
}

/// Hand-assemble the Stage 24c accept demo: one ring 3 process that plays **both ends** of a loopback TCP
/// connection, exercising the whole socket lifecycle — `socket`(server) → `listen`(port) → `socket`(client)
/// → `connect`(dst) → `accept` → `send`(client) → `recv`(accepted) → `write` → `exit`. The `connect` forks a
/// server-side TCB into the listener's accept queue; `accept` claims it and returns a *new* fd; the client
/// then `send`s a message that the accepted (server) socket `recv`s directly (no echo server needed — the
/// two fds are the two ends of the same loopback connection), and the program prints what it received.
///
/// Register plan (every GP register survives a syscall — the entry stub saves the full `TrapFrame`): the
/// listen fd lives in `r15`, the client fd in `rbx`, the accepted fd in `r14`; `rax` carries each `socket`'s
/// result and is reloaded from `r15`/`r14` when a specific fd is needed. Produces exactly
/// [`ACCEPT_DEMO_CODE_LEN`] bytes. `dst` packs our own address + `port` (loopback; see [`pack_dst`]).
fn build_accept_demo(
    dst: u64,
    port: u16,
    send_ptr: u64,
    send_len: u8,
    recv_ptr: u64,
    recv_cap: u8,
) -> Vec<u8> {
    let mut code = Vec::new();
    emit_socket(&mut code); // rax = server (listen) fd
    code.extend_from_slice(&[0x49, 0x89, 0xC7]); // mov r15, rax   (stash the listen fd)
    emit_listen(&mut code, port); // listen(fd, port); leaves rax = listen fd
    emit_socket(&mut code); // rax = client fd
    code.extend_from_slice(&[0x48, 0x89, 0xC3]); // mov rbx, rax   (stash the client fd; rax still = it)
    emit_connect(&mut code, dst); // connect(client fd, dst) -> forks + queues a server-side TCB
    code.extend_from_slice(&[0x4C, 0x89, 0xF8]); // mov rax, r15   (restore the listen fd for accept)
    emit_accept(&mut code); // accept(listen fd) -> rax = accepted fd
    code.extend_from_slice(&[0x49, 0x89, 0xC6]); // mov r14, rax   (stash the accepted fd)
    emit_send(&mut code, send_ptr, send_len); // send(client fd, msg)   [reads rbx = client fd]
    code.extend_from_slice(&[0x4C, 0x89, 0xF3]); // mov rbx, r14   (rbx = accepted fd for recv)
    emit_recv(&mut code, recv_ptr, recv_cap); // recv(accepted fd, buf) [reads rbx = accepted fd]
    emit_write_dyn(&mut code, recv_ptr); // write(buf, count)
    emit_exit(&mut code, 0);
    debug_assert_eq!(code.len(), ACCEPT_DEMO_CODE_LEN);
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

/// Programs the kernel can create on a `spawn` syscall (Stage 12d), addressed by the
/// small integer a ring 3 caller passes in. Today there is just one: the wait-demo child,
/// which writes a line and exits with [`CHILD_EXIT_CODE`] — the very program the kernel
/// used to spawn directly, now created on demand by its parent.
pub const PROG_CHILD: u64 = 0;

/// Build the ELF bytes for a spawnable program id, or `None` if the id is unknown.
fn program_elf(prog_id: u64) -> Option<Vec<u8>> {
    match prog_id {
        PROG_CHILD => Some(child_elf()),
        _ => None,
    }
}

/// Boot demo for `wait` + `spawn` (Stage 12/12d): load and spawn only the *parent*. The
/// parent, running in ring 3, then creates its own child at runtime via the `spawn`
/// syscall ([`on_user_spawn`]) and `wait`s for it; the child `write`s and `exit`s with
/// [`CHILD_EXIT_CODE`], which the parent collects (recorded for the tests). Spawned
/// alongside the Stage 12c interleaving workers, so everything runs together under the
/// scheduler. Returns the parent's process id.
pub fn spawn_wait_demo(
    frame_allocator: &mut impl FrameAllocator<Size4KiB>,
    physical_memory_offset: VirtAddr,
) -> u64 {
    let parent_img = load(&parent_elf(), frame_allocator, physical_memory_offset)
        .expect("failed to load the wait-demo parent");
    let parent_id = spawn(parent_img, None);
    serial_println!(
        "[sched] wait-demo: spawned parent {} (it will spawn its own child via syscall)",
        parent_id
    );
    parent_id
}

/// The loopback TCP port the Stage 24c accept demo uses: the ring 3 program `listen`s on it and
/// `connect`s to it (playing both ends over PHY loopback). Arbitrary and otherwise unused.
pub const CONNECT_DEMO_PORT: u16 = 7900;

/// Pack an IPv4 address + port into the single 64-bit argument the `connect` syscall takes.
/// Our stack ABI passes two u64 arguments, but the destination is three values (fd is the
/// other argument), so the address goes in the high 32 bits — in big-endian octet order, so
/// the packed value reads left-to-right as the dotted-decimal address — and the port in the
/// low 16. [`on_user_connect`] unpacks it the same way.
fn pack_dst(ip: [u8; 4], port: u16) -> u64 {
    ((u32::from_be_bytes(ip) as u64) << 16) | port as u64
}

/// Build the Stage 24c accept-demo ELF (`socket`; `listen`; `socket`; `connect`; `accept`; `send`; `recv`;
/// `write`; `exit`).
///
/// The program's data area is the message to *send* followed by a zeroed *receive* buffer — both in the
/// (writable) loaded segment, so `recv` can copy the received bytes into the second region. The send message
/// sits right after the code (at `msg_vaddr`), and the receive buffer right after it. The 27-byte message is
/// kept from the Stage 24b demo so its byte count is a stable value the tests assert.
fn accept_demo_elf(dst: u64) -> Vec<u8> {
    let send_msg = b"hello from a ring 3 socket\n";
    let send_ptr = msg_vaddr(ACCEPT_DEMO_CODE_LEN);
    let recv_ptr = send_ptr + send_msg.len() as u64;
    let code = build_accept_demo(
        dst,
        CONNECT_DEMO_PORT,
        send_ptr,
        send_msg.len() as u8,
        recv_ptr,
        SOCKET_DEMO_RECV_CAP,
    );
    // Data = the send message, then a zeroed receive buffer of SOCKET_DEMO_RECV_CAP bytes.
    let mut data = send_msg.to_vec();
    data.extend_from_slice(&alloc::vec![0u8; SOCKET_DEMO_RECV_CAP as usize]);
    build_elf(&code, &data)
}

/// Boot demo for the Stage 24c server-socket syscalls (subsuming 24a/24b's client path): with the NIC's PHY
/// loopback enabled — so a locally-sent frame returns to us, mirroring the `net` loopback self-tests — load
/// and spawn a ring 3 program that plays **both ends** of a loopback TCP connection: `socket`/`listen` a
/// server socket, `socket`/`connect` a client socket to it, `accept` the connection (a new fd), then `send`
/// on the client fd and `recv` on the accepted fd. The blocking `connect`/`accept`/`recv` drive the network
/// inline (see [`on_user_connect`] / [`on_user_accept`] / [`on_user_recv`]); the caller disables loopback
/// again after the process phase. Returns the demo process's pid.
pub fn spawn_accept_demo(
    frame_allocator: &mut impl FrameAllocator<Size4KiB>,
    physical_memory_offset: VirtAddr,
) -> u64 {
    let ip = crate::net::our_ip();
    crate::net::tcp_loopback_reset();
    let dst = pack_dst(ip, CONNECT_DEMO_PORT);
    let img = load(&accept_demo_elf(dst), frame_allocator, physical_memory_offset)
        .expect("failed to load the accept-demo program");
    let pid = spawn(img, None);
    serial_println!(
        "[sched] accept-demo: spawned process {} (it listens on {}, connects to {}.{}.{}.{}:{}, accepts, and exchanges data over loopback)",
        pid, CONNECT_DEMO_PORT, ip[0], ip[1], ip[2], ip[3], CONNECT_DEMO_PORT,
    );
    pid
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
/// How many child processes ring 3 created via the `spawn` syscall — Stage 12d test.
static PROCESSES_SPAWNED: AtomicU64 = AtomicU64::new(0);
/// How many ring 3 processes established a TCP connection via the `connect` syscall — Stage
/// 24a test.
static PROCESSES_CONNECTED: AtomicU64 = AtomicU64::new(0);
/// How many `send` syscalls a ring 3 process completed — Stage 24b test.
static PROCESSES_SENT: AtomicU64 = AtomicU64::new(0);
/// How many `recv` syscalls returned data to a ring 3 process — Stage 24b test.
static PROCESSES_RECEIVED: AtomicU64 = AtomicU64::new(0);
/// Bytes the most recent `recv` delivered — Stage 24b test.
static LAST_RECV_LEN: AtomicU64 = AtomicU64::new(0);
/// How many `listen` syscalls a ring 3 process completed — Stage 24c test.
static PROCESSES_LISTENED: AtomicU64 = AtomicU64::new(0);
/// How many `accept` syscalls returned a new connection to a ring 3 process — Stage 24c test.
static PROCESSES_ACCEPTED: AtomicU64 = AtomicU64::new(0);
/// The fd the most recent `accept` returned (`u64::MAX` = none yet) — Stage 24c test.
static LAST_ACCEPTED_FD: AtomicU64 = AtomicU64::new(u64::MAX);

/// Stage 24a: a user **socket** — the per-process handle a ring 3 program obtains from the
/// `socket` syscall and connects with. It is a thin binding from a small-integer **file
/// descriptor** (an index into [`Process::sockets`]) to a TCP connection in the global
/// connection table, which `net`/`tcp` key by the `(local_port, remote_port)` pair. A fresh
/// socket is *unbound* (both ports 0); `connect` fills them in once the handshake
/// establishes, so a later `send`/`recv` (Stage 24b) can find the connection's TCB. This is
/// the kernel's first step toward Unix's "everything is a file descriptor".
#[derive(Clone, Copy)]
struct UserSocket {
    local_port: u16,
    remote_port: u16,
    /// Stage 24c: whether this socket is a passive **listener** (bound via `listen`, awaiting `accept`)
    /// rather than a connection endpoint. A listener has `local_port` set (its bound port) and
    /// `remote_port == 0`; `accept` produces a fresh, non-listening socket for each connection it claims.
    listening: bool,
}

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
    /// Stage 24a: the process's **socket handle table** — its open sockets, indexed by file
    /// descriptor (a `None` slot is a closed/free fd). Empty until the process calls
    /// `socket`. Per-process, so the same small fd means different connections in different
    /// processes — the point of a handle table.
    sockets: Vec<Option<UserSocket>>,
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
    /// Stage 24a: processes blocked in a networking syscall (so far only `connect`),
    /// waiting for a network event — the same idea as `blocked`, but woken by the network
    /// stack rather than a child's exit. A blocked process is on no run queue; the
    /// `connect` handler parks it here while it drives the handshake, then moves it back to
    /// `current`/`ready` with its result (see [`on_user_connect`]).
    net_blocked: Vec<Process>,
    /// Exited children whose parents have not yet collected them via `wait` (Stage 12).
    zombies: Vec<Zombie>,
    next_id: u64,
}

static SCHEDULER: Mutex<Scheduler> = Mutex::new(Scheduler {
    ready: Vec::new(),
    current: None,
    blocked: Vec::new(),
    net_blocked: Vec::new(),
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
        sockets: Vec::new(), // no sockets until the process calls `socket` (Stage 24a)
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

/// Called by the `spawn` syscall (ring 3, Stage 12d): create a new child process from the
/// kernel-known program `prog_id`, enqueue it as a child of the caller, and return the
/// child's process id (`u64::MAX` on error: unknown program, no current process, or a
/// load failure). Unlike `yield`/`exit`/`wait`, `spawn` does not switch processes — the
/// caller resumes right after with the new pid.
///
/// The subtle part is *which* address space the loader clones. [`load`] calls
/// [`AddressSpace::new_cloning_kernel`], which clones whatever CR3 points at — but here
/// that is the *caller's* space, whose user slot is already populated. Cloning it would
/// make the child share (then clobber) the parent's user page tables, and remapping the
/// same `USER_LOAD_BASE` would fail. So we switch to the pure kernel space (user slot
/// empty, exactly as at boot) for the load, then switch back so the syscall's `iretq`
/// returns into the caller.
pub fn on_user_spawn(prog_id: u64) -> u64 {
    // The caller (parent) is the current process. A syscall runs with interrupts off, so
    // `current` cannot change under us between here and the `spawn` below.
    let parent_id = {
        let sched = SCHEDULER.lock();
        match &sched.current {
            Some(p) => p.id,
            None => return u64::MAX,
        }
    };

    let elf = match program_elf(prog_id) {
        Some(bytes) => bytes,
        None => {
            serial_println!("[sched] spawn: unknown program id {}", prog_id);
            return u64::MAX;
        }
    };

    // Load into a fresh space against the *kernel* CR3 (see the doc comment), then restore
    // the caller's CR3.
    let parent_cr3 = Cr3::read();
    return_to_kernel_space();
    let offset = memory::physical_memory_offset();
    let loaded = memory::with_kernel_frame_allocator(|fa| load(&elf, fa, offset));
    // SAFETY: `parent_cr3` was active a moment ago and maps the running kernel (it is a
    // kernel clone), so restoring it is sound; it must be active when the syscall returns
    // to the caller in ring 3.
    unsafe { memory::restore_address_space(parent_cr3) };

    let image = match loaded {
        Ok(img) => img,
        Err(_) => {
            serial_println!("[sched] spawn: failed to load program {}", prog_id);
            return u64::MAX;
        }
    };

    let child_id = spawn(image, Some(parent_id));
    PROCESSES_SPAWNED.fetch_add(1, Ordering::SeqCst);
    serial_println!(
        "[sched] process {} spawned child {} (program {})",
        parent_id,
        child_id,
        prog_id
    );
    child_id
}

/// Called by the `socket` syscall (ring 3, Stage 24a): allocate a fresh, unbound socket in
/// the calling process's handle table and return its **file descriptor** (the table index).
/// Reuses the lowest free slot, mirroring Unix's "lowest available fd". Returns `u64::MAX`
/// if there is no current process (it should never be called from ring 0). Like `getpid`,
/// it returns via the stack ABI and does not switch processes.
pub fn on_user_socket() -> u64 {
    let mut sched = SCHEDULER.lock();
    let proc = match sched.current.as_mut() {
        Some(p) => p,
        None => return u64::MAX,
    };
    let fd = alloc_socket(proc, UserSocket { local_port: 0, remote_port: 0, listening: false });
    serial_println!("[sched] process {} opened socket fd {}", proc.id, fd);
    fd
}

/// Install `sock` in `proc`'s handle table at the lowest free file descriptor (reusing a closed slot,
/// mirroring Unix's "lowest available fd"), and return that fd. Shared by `socket` (a fresh unbound socket)
/// and `accept` (a socket bound to a just-accepted connection).
fn alloc_socket(proc: &mut Process, sock: UserSocket) -> u64 {
    let fd = match proc.sockets.iter().position(|s| s.is_none()) {
        Some(i) => {
            proc.sockets[i] = Some(sock);
            i
        }
        None => {
            proc.sockets.push(Some(sock));
            proc.sockets.len() - 1
        }
    };
    fd as u64
}

/// Called by the `listen` syscall (ring 3, Stage 24c): turn the process's unbound socket `fd` into a passive
/// **listener** bound to `port`, and register that listener with the TCP stack so incoming SYNs are accepted
/// (the listener stays open and forks a TCB per connection — see [`crate::net::tcp_listen`]). Non-blocking:
/// it returns via the stack ABI (0 on success, `u64::MAX` if `fd` is not an allocated, still-unbound socket).
/// A later `accept` on the same `fd` claims the connections this listener accumulates.
pub fn on_user_listen(fd: u64, port: u64) -> u64 {
    let port16 = port as u16;
    let pid = {
        let mut sched = SCHEDULER.lock();
        let proc = match sched.current.as_mut() {
            Some(p) => p,
            None => return u64::MAX,
        };
        // The fd must be an allocated socket that is neither connected nor already listening.
        match proc.sockets.get(fd as usize).and_then(|s| *s) {
            Some(sock) if sock.local_port == 0 && !sock.listening => {}
            _ => {
                serial_println!("[sched] listen: process {} has no unbound socket fd {}", proc.id, fd);
                return u64::MAX;
            }
        }
        proc.sockets[fd as usize] = Some(UserSocket {
            local_port: port16,
            remote_port: 0,
            listening: true,
        });
        proc.id
    };
    // Register the listener with the TCP stack outside the scheduler lock (it takes its own connection lock).
    crate::net::tcp_listen(port16);
    PROCESSES_LISTENED.fetch_add(1, Ordering::SeqCst);
    serial_println!("[sched] process {} listening on port {} (socket fd {})", pid, port16, fd);
    0
}

/// Called by the `accept` syscall (ring 3, Stage 24c): take the next connection from the listening socket
/// `fd`'s accept queue and return a **new** file descriptor bound to it (the listening `fd` stays open for
/// more). Like `connect`/`recv` this **blocks** — until a connection is ready — and drives the network
/// **inline** while parked (see [`on_user_connect`] for why): it pumps [`crate::net::poll`], which completes
/// a loopback client's handshake (the forked server-side TCB reaches ESTABLISHED), then claims that
/// connection with [`crate::net::tcp_accept`]. Returns the new fd in **rax** (`u64::MAX` on error/timeout).
///
/// Structurally identical to [`on_user_recv`]: park the caller in `net_blocked`, drive `poll` until the
/// condition is met (or a bound elapses), then wake the caller with the result. Allocating the new fd is
/// safe from Phase 3 because the parked process's own handle table is reachable (we never switched CR3).
pub fn on_user_accept(tf: &mut TrapFrame, fd: u64) {
    // Phase 1 (locked): validate that `fd` is a listening socket, capture its port, and park the caller.
    let (pid, port) = {
        let mut sched = SCHEDULER.lock();
        let proc = match sched.current.as_mut() {
            Some(p) => p,
            None => return,
        };
        let port = match proc.sockets.get(fd as usize).and_then(|s| *s) {
            Some(sock) if sock.listening => sock.local_port,
            _ => {
                serial_println!("[sched] accept: process {} has no listening socket fd {}", proc.id, fd);
                tf.rax = u64::MAX;
                return;
            }
        };
        let mut blocked = sched.current.take().expect("current is_some, checked above");
        blocked.context = *tf;
        let id = blocked.id;
        sched.net_blocked.push(blocked);
        (id, port)
    };

    // Phase 2 (unlocked): drive the network until a connection is ready to accept (or we give up). Over
    // loopback the client's handshake finishes here — its final ACK is delivered by `poll`, moving the forked
    // server-side TCB to ESTABLISHED — and `tcp_accept` then claims it. Bounded so a silent peer cannot hang
    // the process forever (a syscall runs with interrupts off).
    let mut accepted: Option<(u16, u16)> = None;
    for _ in 0..RECV_POLL_ITERS {
        if let Some(ports) = crate::net::tcp_accept(port) {
            accepted = Some(ports);
            break;
        }
        crate::net::poll();
        crate::apic::pit_sleep_us(RECV_POLL_US);
    }

    // Phase 3 (locked): allocate a new fd bound to the accepted connection and resume the caller with it.
    let mut sched = SCHEDULER.lock();
    let pos = sched
        .net_blocked
        .iter()
        .position(|p| p.id == pid)
        .expect("the process we just blocked is still in net_blocked");
    let mut proc = sched.net_blocked.remove(pos);
    let result = match accepted {
        Some((local_port, remote_port)) => {
            let newfd = alloc_socket(
                &mut proc,
                UserSocket { local_port, remote_port, listening: false },
            );
            PROCESSES_ACCEPTED.fetch_add(1, Ordering::SeqCst);
            LAST_ACCEPTED_FD.store(newfd, Ordering::SeqCst);
            serial_println!(
                "[sched] process {} accepted a connection on port {}: new socket fd {} -> {}:{}",
                proc.id, port, newfd, local_port, remote_port,
            );
            newfd
        }
        None => {
            serial_println!("[sched] process {} accept timed out on port {}", proc.id, port);
            u64::MAX
        }
    };
    proc.context.rax = result;
    let resume = proc.context;
    sched.current = Some(proc);
    *tf = resume;
}

/// Called by the `connect` syscall (ring 3, Stage 24a): actively open a TCP connection from
/// the process's socket `fd` to the destination packed in `dst` — the IPv4 address in the
/// high 32 bits (big-endian octet order, so it reads left-to-right) and the port in the low
/// 16. This is the stack's **first blocking syscall**: the process cannot proceed until the
/// three-way handshake reaches ESTABLISHED, so it is descheduled meanwhile.
///
/// The block reuses the `wait` pattern: the caller is moved out of `current` into
/// `net_blocked` (on no run queue) while the handshake runs, then moved back with its
/// result. *How* the handshake is driven is the Stage 24a design decision (see ROADMAP.md):
/// the process scheduler runs as a boot phase with no concurrent network thread yet, so this
/// handler drives it **inline** — [`crate::net::tcp_connect`] sends the SYN and pumps
/// `net::poll` until ESTABLISHED (for a loopback peer, a handful of microseconds; interrupts
/// are off during a syscall, so it is atomic w.r.t. the other processes). A later stage that
/// runs user processes alongside the background `net_thread` would instead switch to another
/// ready process here and let the net thread's poll wake this one.
///
/// Returns its result in **rax** (like `wait`, since a blocking syscall resumes from its
/// saved [`TrapFrame`]): the connected socket fd on success, or `u64::MAX` on failure/timeout.
/// On success the socket is bound to the established connection so `send`/`recv` (Stage 24b)
/// can find its TCB. Rewrites `tf` to resume the (now-unblocked) caller.
pub fn on_user_connect(tf: &mut TrapFrame, fd: u64, dst: u64) {
    let ip = ((dst >> 16) as u32).to_be_bytes();
    let remote_port = (dst & 0xFFFF) as u16;

    // Phase 1 (locked): validate the fd, then park the caller in `net_blocked` for the
    // duration of the handshake. Taking it out of `current` is what makes this a real block —
    // it is on no run queue until Phase 3 wakes it.
    let pid = {
        let mut sched = SCHEDULER.lock();
        let proc = match sched.current.as_mut() {
            Some(p) => p,
            None => return, // no current process — should not happen from a ring 3 syscall
        };
        let idx = fd as usize;
        if idx >= proc.sockets.len() || proc.sockets[idx].is_none() {
            serial_println!("[sched] connect: process {} has no socket fd {}", proc.id, fd);
            tf.rax = u64::MAX;
            return;
        }
        let mut blocked = sched.current.take().expect("current is_some, checked above");
        blocked.context = *tf; // resume point; rax is filled in once the handshake settles
        let id = blocked.id;
        serial_println!(
            "[sched] process {} blocked in connect(fd {}) to {}.{}.{}.{}:{}",
            id, fd, ip[0], ip[1], ip[2], ip[3], remote_port,
        );
        sched.net_blocked.push(blocked);
        id
    };

    // Phase 2 (unlocked): drive the handshake to completion. `tcp_connect` sends the SYN and
    // pumps `net::poll` until ESTABLISHED, returning the chosen ephemeral local port, or
    // `None` on timeout. The scheduler lock is released so the poll path can run freely.
    let result = crate::net::tcp_connect(ip, remote_port);

    // Phase 3 (locked): wake the blocked process with the outcome and resume it. On success
    // bind the socket to the established connection's port pair.
    let mut sched = SCHEDULER.lock();
    let pos = sched
        .net_blocked
        .iter()
        .position(|p| p.id == pid)
        .expect("the process we just blocked is still in net_blocked");
    let mut proc = sched.net_blocked.remove(pos);
    match result {
        Some(local_port) => {
            proc.sockets[fd as usize] = Some(UserSocket { local_port, remote_port, listening: false });
            proc.context.rax = fd; // connect() returns the connected fd
            PROCESSES_CONNECTED.fetch_add(1, Ordering::SeqCst);
            serial_println!(
                "[sched] process {} connected: socket fd {} -> {}:{} (ESTABLISHED)",
                proc.id, fd, local_port, remote_port,
            );
        }
        None => {
            proc.context.rax = u64::MAX; // connect() failed / timed out
            serial_println!("[sched] process {} connect timed out", proc.id);
        }
    }
    let resume = proc.context;
    sched.current = Some(proc);
    *tf = resume; // resume the caller (its rax now holds the connect result)
}

/// Look up the calling process's socket `fd` and return the `(local_port, remote_port)` of the
/// connection it is bound to, or `None` if the fd is invalid or the socket is not connected
/// (still port 0/0). Shared by `send` and `recv`. The caller must hold the scheduler lock.
fn socket_ports(proc: &Process, fd: u64) -> Option<(u16, u16)> {
    match proc.sockets.get(fd as usize).and_then(|s| *s) {
        // A connected socket (bound local port, not a listener). A listening socket (Stage 24c) is not a
        // send/recv endpoint — data flows on the sockets `accept` produces, not on the listener itself.
        Some(sock) if sock.local_port != 0 && !sock.listening => {
            Some((sock.local_port, sock.remote_port))
        }
        _ => None,
    }
}

/// Called by the `send` syscall (ring 3, Stage 24b): send `len` bytes from the process's buffer
/// at `ptr` on socket `fd`. Non-blocking — the bytes are queued and flushed by [`crate::net::tcp_send`]
/// (TCP's own send buffer and sliding window pace the wire), so it returns immediately with the
/// count via the stack ABI (`u64::MAX` on error: bad/unconnected fd, or no such connection).
pub fn on_user_send(fd: u64, ptr: u64, len: u64) -> u64 {
    let (local, remote) = {
        let sched = SCHEDULER.lock();
        let proc = match &sched.current {
            Some(p) => p,
            None => return u64::MAX,
        };
        match socket_ports(proc, fd) {
            Some(ports) => ports,
            None => {
                serial_println!("[sched] send: no connected socket fd {}", fd);
                return u64::MAX;
            }
        }
    };
    // SAFETY: `(ptr, len)` is a buffer in the caller's address space, which is the active one (a
    // syscall runs on the caller's CR3). We only read it. A hardened kernel would bounds-check the
    // range against the caller's own mappings, as `sys_write` notes.
    let data = unsafe { core::slice::from_raw_parts(ptr as *const u8, len as usize) };
    if crate::net::tcp_send(local, remote, data) {
        PROCESSES_SENT.fetch_add(1, Ordering::SeqCst);
        serial_println!("[sched] process sent {} byte(s) on socket fd {}", len, fd);
        len
    } else {
        u64::MAX
    }
}

/// Called by the `recv` syscall (ring 3, Stage 24b): receive up to `len` bytes into the process's
/// buffer at `ptr` on socket `fd`, returning the count in **rax** (0 at end-of-stream). Like
/// `connect` this **blocks** — if no data has arrived it parks the caller in `net_blocked` and
/// drives [`crate::net::poll`] inline (the loopback echo server, driven by that same poll, produces
/// the reply) until bytes are available, then copies them into the user buffer and wakes the caller.
/// A later stage with a concurrent net thread would instead switch away and be woken by it.
///
/// The copy into `ptr` is safe throughout because we never switch CR3 while blocked — the caller's
/// address space stays active — so its buffer is reachable the whole time.
pub fn on_user_recv(tf: &mut TrapFrame, fd: u64, ptr: u64, len: u64) {
    // Phase 1 (locked): validate the fd, resolve its connection, and park the caller.
    let (pid, local, remote) = {
        let mut sched = SCHEDULER.lock();
        let proc = match sched.current.as_mut() {
            Some(p) => p,
            None => return,
        };
        let (local, remote) = match socket_ports(proc, fd) {
            Some(ports) => ports,
            None => {
                serial_println!("[sched] recv: no connected socket fd {}", fd);
                tf.rax = u64::MAX;
                return;
            }
        };
        let mut blocked = sched.current.take().expect("current is_some, checked above");
        blocked.context = *tf;
        let id = blocked.id;
        sched.net_blocked.push(blocked);
        (id, local, remote)
    };

    // Phase 2 (unlocked): drive the network until data arrives (or we give up), then copy it into
    // the caller's buffer. `tcp_read` drains up to `len` bytes; `poll` runs the echo server that
    // generates the reply. Bounded so a silent peer cannot hang the process forever.
    let max = len as usize;
    let mut got: Vec<u8> = Vec::new();
    for _ in 0..RECV_POLL_ITERS {
        if let Some(data) = crate::net::tcp_read(local, remote, max) {
            if !data.is_empty() {
                got = data;
                break;
            }
        }
        crate::net::poll();
        crate::apic::pit_sleep_us(RECV_POLL_US);
    }
    let n = got.len().min(max);
    if n > 0 {
        // SAFETY: `(ptr, n)` is a writable buffer in the caller's address space, still the active
        // one (we never switched CR3 while blocked), and `n <= len` bounds the write to the caller's
        // requested size.
        let dst = unsafe { core::slice::from_raw_parts_mut(ptr as *mut u8, n) };
        dst.copy_from_slice(&got[..n]);
    }

    // Phase 3 (locked): wake the parked process with the byte count and resume it.
    let mut sched = SCHEDULER.lock();
    let pos = sched
        .net_blocked
        .iter()
        .position(|p| p.id == pid)
        .expect("the process we just blocked is still in net_blocked");
    let mut proc = sched.net_blocked.remove(pos);
    proc.context.rax = n as u64;
    if n > 0 {
        PROCESSES_RECEIVED.fetch_add(1, Ordering::SeqCst);
        LAST_RECV_LEN.store(n as u64, Ordering::SeqCst);
        serial_println!("[sched] process {} received {} byte(s) on socket fd {}", proc.id, n, fd);
    }
    let resume = proc.context;
    sched.current = Some(proc);
    *tf = resume;
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

/// How many child processes have been created via the `spawn` syscall. For the Stage 12d
/// test.
pub fn processes_spawned() -> u64 {
    PROCESSES_SPAWNED.load(Ordering::SeqCst)
}

/// How many ring 3 processes established a TCP connection via the `connect` syscall. For the
/// Stage 24a test.
pub fn processes_connected() -> u64 {
    PROCESSES_CONNECTED.load(Ordering::SeqCst)
}

/// How many `send` syscalls a ring 3 process completed. For the Stage 24b test.
pub fn processes_sent() -> u64 {
    PROCESSES_SENT.load(Ordering::SeqCst)
}

/// How many `recv` syscalls returned data to a ring 3 process. For the Stage 24b test.
pub fn processes_received() -> u64 {
    PROCESSES_RECEIVED.load(Ordering::SeqCst)
}

/// Bytes the most recent `recv` delivered. For the Stage 24b test.
pub fn last_recv_len() -> u64 {
    LAST_RECV_LEN.load(Ordering::SeqCst)
}

/// How many `listen` syscalls a ring 3 process completed. For the Stage 24c test.
pub fn processes_listened() -> u64 {
    PROCESSES_LISTENED.load(Ordering::SeqCst)
}

/// How many `accept` syscalls returned a new connection to a ring 3 process. For the Stage 24c test.
pub fn processes_accepted() -> u64 {
    PROCESSES_ACCEPTED.load(Ordering::SeqCst)
}

/// The fd the most recent `accept` returned (`u64::MAX` if none yet). For the Stage 24c test.
pub fn last_accepted_fd() -> u64 {
    LAST_ACCEPTED_FD.load(Ordering::SeqCst)
}
