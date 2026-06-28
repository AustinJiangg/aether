# Aether Roadmap

Each stage builds on the previous one, runs in QEMU, and is worth a git commit.
This is the essence of "runs and iterates continuously": small steps, each
verifiable.

Recommended: create a git branch per stage, and merge back to main once it runs.

| Stage | What to build | OS concepts learned | Status |
|-------|---------------|---------------------|--------|
| **0** | Boot + serial output (current) | boot flow, freestanding binary, no_std | Done |
| **1** | VGA text buffer, print to the screen | memory-mapped I/O, video memory | Done |
| **2** | CPU exception handling (IDT), trigger a breakpoint exception | interrupt / exception mechanism | Done |
| **3** | Hardware interrupts (PIC), keyboard input, timer interrupt | interrupt controller, device drivers | Done |
| **4** | Paging + heap allocator, make `Box` / `Vec` usable | virtual memory, address spaces | Done |
| **5** | Cooperative multitasking (tasks written with `async` / `await`) | processes / tasks, scheduling | Done |
| **6** | Preemptive scheduling, independent kernel threads | context switching, time slices | Done |
| **7** | Simple shell + built-in commands | system calls, user interaction | Done |
| **8** | In-memory simple file system | file abstraction, VFS | Done |

**All planned stages (0-8) are complete.** The kernel also has an in-QEMU
unit-test harness (`cargo test`, see `src/testing.rs`) — engineering scaffolding
rather than a numbered stage, added before the user-space work below so the
riskier code that follows has an automated safety net.

## Beyond the roadmap (stages 9+)

Stages 0-8 produced a single-address-space kernel that runs entirely in ring 0:
kernel threads, an async executor, a heap, and an in-memory file system — but no
user mode, no privilege separation, and no real system calls. The through-line
from here is to **run real, isolated user programs**, the step that turns "a
kernel with threads" into "an operating system." That is the main line below;
three more independent tracks (persistence, modern hardware, networking) can
follow in any order.

An architectural note that shapes the main line: the kernel currently has two
multitasking models that do not yet coexist — the async executor (`task/`, which
drives the shell today) and the preemptive thread scheduler (`thread/`, dormant).
User processes need a per-process kernel stack, a saved register context, and an
address space — which the thread side already prototypes — so the user-space work
extends `thread/`, while the async executor stays an in-kernel facility. The two
unify later.

### Main line: the road to user space

> **Status:** Stages 9-11 are complete; Stage 12 is complete (12a-12c, a `wait` syscall,
> and 12d a `spawn` syscall — a process can now create another at runtime, rather than the
> kernel spawning them all at boot). Stage 9 reaches
> ring 3 (`gdt.rs`, `usermode.rs`); Stage 10 adds `int 0x80` system calls
> (`syscall.rs`); Stage 11a adds an `AddressSpace` (`memory.rs`) that clones the
> kernel L4 and switches CR3; Stage 11b adds an ELF64 parser (`elf.rs`) and loader
> (`process.rs`) that maps a program's `PT_LOAD` segments into a fresh space. Stage
> 12a runs one loaded program in ring 3 on its own CR3 (the `int 0x80` handler still
> reaches the kernel because every space maps it). Stage 12b adds a cooperative
> scheduler (`process.rs`): `spawn` queues loaded programs, `run` enters the first in
> ring 3, and the `yield`/`exit` syscalls switch processes — rewriting the interrupt
> frame and CR3 from inside the handler. `yield` saves the caller's resume point and
> round-robins to the next; `exit` drops it. Two programs that each run several
> `write`+`yield` rounds therefore *interleave* their output (#1, #2, #1, #2, ...),
> and being byte-identical yet printing different messages from the same virtual
> address, they also prove address-space isolation. Stage 12c then made scheduling
> **preemptive** in three sub-steps: 12c-1 and 12c-2 route the timer *and* the
> `int 0x80` syscall through hand-written *naked* stubs that capture the full register
> set into a `TrapFrame` (so a switch saves and restores every register, not just the
> interrupt frame), and 12c-3 sets IF in the ring 3 frame and has the timer — on a
> tick that interrupted ring 3 — save the running process's `TrapFrame` and round-robin
> to the next. Two programs busy-spinning between writes therefore interleave under
> timer preemption with no `yield` required (the `yield`/`exit` syscalls remain as
> voluntary switch points). Note: under bootloader 0.9 the kernel, heap, and
> physical-memory window all live in the *lower* half (present L4 slots all < 256), so a
> clone copies every present top-level entry, and user programs load into an
> otherwise-empty slot (64) for private lower-level tables. Stage 12 also adds `wait`: a
> parent blocks until its child exits and collects the child's exit code (returned in
> rax — the kernel often wakes the parent from the child's exit running in a *different*
> address space, where only the saved register, not the user stack, is reachable). Stage
> 12d then adds `spawn` (`SYS_SPAWN`): a ring 3 process loads a kernel-known program into a
> fresh address space and enqueues it as its child, returning the new pid — so the wait
> demo's parent now creates its own child at runtime instead of the kernel pre-spawning it.
> This needed a globally reachable kernel frame allocator (`memory.rs`), since the loader
> runs inside the syscall trap handler, far from `kernel_main`'s locals.

| Stage | What to build | OS concepts | Smallest verifiable step |
|-------|---------------|-------------|--------------------------|
| **9**  | Drop to user mode (ring 3) | privilege rings, GDT user segments, TSS `rsp0`, the `iretq` descent | Map a user page, `iretq` into ring-3 code; a timer interrupt fires and the handler observes `CPL == 3` in the saved frame — proving both that we reached ring 3 and that `rsp0` keeps the interrupt from triple-faulting. |
| **10** | System calls | the user/kernel boundary, register-passed arguments | Start with `int 0x80` (IDT gate `DPL = 3`), simpler than the `syscall`/`sysret` MSRs. Implement `write`, `exit`, `getpid`; the Stage 9 program prints via `sys_write` and exits cleanly. |
| **11** | Process address space + ELF loader | address-space isolation, higher-half kernel, ELF64 | (11a) each process its own page table (own CR3), kernel mapped into every space's higher half; (11b) a minimal ELF64 loader maps `PT_LOAD` segments. Hardest step: copy the top-level entries covering the kernel + physical-memory map into the new L4, or the kernel is unreachable after the CR3 switch. |
| **12** | Multiple user processes | process scheduling, time-sliced userland | Tie processes into the (extended) thread scheduler, switching CR3 on each context switch; two user programs interleave their output under timer preemption. Add `spawn`/`exit`/`wait`. |

### Parallel tracks (any order, after the main line)

> **Status:** Stage 13a and 13b are done. **13a** — a polled **ATA PIO** driver (`ata.rs`)
> reads raw 512-byte sectors from the primary IDE master, verified at boot and in a test
> against the boot disk's MBR signature (`0x55 0xAA`). The drive runs with interrupts
> disabled (nIEN): the kernel polls and registers no ATA IRQ handler, so an unhandled IRQ14
> would otherwise cascade (vector 46 → not-present gate → #NP → double fault). This work
> also added page-fault and general-protection-fault handlers (`interrupts.rs`). **13b** —
> sector *writes*: WRITE SECTORS (0x30) plus a CACHE FLUSH (0xE7) so the data is durable,
> driven word-at-a-time over the data port. Writes target a *separate scratch disk* attached
> as the primary slave (a `Drive` enum names master vs. slave at every call site, so the boot
> image is never at risk; `build.rs` creates the `scratch.img` backing file QEMU needs).
> Verified by a boot demo and a test that write a sector and read it back for an exact
> round-trip.
>
> **Stage 14a is also done** — the VFS seam. The in-memory file system's operations are
> factored into a `FileSystem` trait (`fs.rs`), the virtual-filesystem layer real kernels
> put between user code and the concrete filesystem drivers; `RamFs` is the first
> implementor. Pure refactor (no behavior change): the shell still calls the same global
> `fs::*` functions, and a new test drives a `RamFs` through a `&mut dyn FileSystem` trait
> object to prove the abstraction dispatches dynamically.
>
> **Stage 14b-1 is also done** — the FAT volume's boot sector. `fat.rs` reads sector 0 of a
> real FAT16 disk and parses its **BPB** (BIOS Parameter Block) into geometry — sector/cluster
> sizes, FAT count and size, root-entry count, total sectors — and derives the region layout
> (FAT, root-directory, and data start LBAs). The disk (`fat.img`) is formatted by the host's
> `mkfs.fat` in `build.rs` (a known `HELLO.TXT` copied in via `mcopy`) and attached as the
> *secondary* IDE master, which extended the ATA driver to address the second bus
> (`SecondaryMaster`, ports 0x170/0x376). Verified by a boot demo and a test asserting the
> exact geometry.
>
> **Stage 14b-2a is also done** — reading a file off the FAT volume. `fat.rs` gains a mounted
> `Fat` handle (`Fat::mount` parses the BPB) and `read_file(name)`: it scans the fixed-size
> root directory for the 8.3 entry (case-insensitive, skipping deleted, long-name, and
> volume-label entries), then walks the file's **FAT cluster chain** — each FAT16 entry a
> little-endian `u16` pointing at the next cluster, until an end-of-chain marker — reading each
> cluster's sectors and truncating to the directory's size field. A corrupt or non-terminating
> chain is bounded and rejected (`BadChain`). Verified by a boot demo that prints the known
> `HELLO.TXT` and a test asserting its exact bytes, the case-insensitive match, and the
> `NotFound` path.
>
> **Stage 14b-2b is also done** — the FAT volume behind the VFS trait. `Fat` now implements
> the `FileSystem` trait from Stage 14a, so it is usable through a `&dyn FileSystem` object
> exactly like `RamFs`: `read`, `list`, and `is_dir` operate on the root directory and its
> entries, while the mutating operations (`mkdir`/`write`/`remove`) return `Unsupported`, since
> this driver is read-only. `FsError` gains `Unsupported` and `Io` variants, and a
> `From<FatError>` maps the driver's errors onto the shared VFS error type; `fs::components` is
> shared so FAT and `RamFs` split paths identically. Verified by a boot demo that lists the
> root through `&dyn FileSystem` and a test driving the volume through `&mut dyn FileSystem`.
>
> **Stage 14b-3 is also done** — the FAT volume is mounted into the VFS. `fs.rs` gains a
> minimal one-entry mount table: a path under `/mnt` is routed to a mounted
> `Box<dyn FileSystem>` (prefix stripped), while everything else stays in the in-memory
> `RamFs`; the six `fs::*` wrappers funnel through one `dispatch` helper, so the shell does not
> know which filesystem backs a path. Boot mounts the read-only FAT volume at `/mnt`, so the
> shell's `ls /mnt`, `cat /mnt/HELLO.TXT`, and `cd /mnt` read real disk files. Verified by the
> shell selftest and a test reading `/mnt/HELLO.TXT` through the global `fs::read`.
>
> **Stage 14c-1 is also done** — writing a file to the FAT volume. `fat.rs` gains the write
> path: `alloc_cluster` scans the FAT for a free cluster and marks it end-of-chain;
> `write_chain` reserves a chain, writes the data (zero-padding the final sector), and links the
> clusters; `write_file` finds or creates the root-directory entry, frees the old chain on an
> overwrite, and stores the name, first cluster, and size. Every FAT copy is updated so the
> mirrors stay identical, and the bytes reach the disk through the Stage 13b `ata::write_sector`
> (cache flush included). It is wired into `FileSystem::write`, so the shell's `write /mnt/foo`
> lands on disk and survives a reboot. Verified by a multi-cluster write/overwrite/read-back
> test and the shell selftest.
>
> **Stage 14c-2 is also done** — removing a file. `remove_file` frees the file's cluster chain
> and marks its directory entry deleted (`0xE5`); a shared `find_entry` helper backs both the
> write and delete lookups. Wired into `FileSystem::remove`, so the shell's `rm /mnt/foo`
> deletes a root-level file. Verified by a write/read/remove test and the shell selftest's full
> lifecycle. **This completes Stage 14**: an on-disk FAT16 filesystem with read *and* write,
> coexisting with `RamFs` behind the VFS. (`mkdir` and subdirectory traversal stay unsupported —
> optional later polish.)
>
> **Stage 15a is also done** — the Local APIC and its timer (the hardware track begins). A new
> `apic.rs` maps the LAPIC's MMIO page uncacheable (`NO_CACHE`, because device registers must
> bypass the cache), software-enables the APIC via the spurious-vector register, and masks the 8259
> PIC, so hardware interrupts now arrive through the APIC. The LAPIC timer's frequency is not
> architecturally fixed (unlike the PIT's known 1.193182 MHz), so it is *calibrated* against the
> PIT over a 10 ms polled window, then run periodically at 100 Hz on vector 32 — the same gate the
> PIT timer used, so the naked timer entry is unchanged; the EOI moves from the 8259 to the LAPIC's
> EOI register, and a no-op handler backs the spurious vector. Timer ticks and preemption now run
> on the APIC (the boot demo shows 50+ preemptions). Verified by all 26 tests (including
> `timer_preempted_a_process`). Next: **Stage 15b** — the IO-APIC, routing the keyboard's IRQ1 to a
> vector (keyboard input is off until then).

| Stage | Track | What to build | OS concepts |
|-------|-------|---------------|-------------|
| **13** | Persistence | Block device driver: ATA PIO read/write of raw sectors from a QEMU disk image | device I/O, polling |
| **14** | Persistence | On-disk file system (FAT, read then write); factor a VFS trait so `RamFs` and the disk FS coexist | real FS layout, VFS |
| **15** | Hardware | Replace the 8259 PIC with the Local APIC + IO-APIC; use the Local APIC timer instead of the PIT | modern interrupt delivery; prereq for SMP |
| **16** | Hardware | SMP: bring up the other cores via INIT-SIPI-SIPI, per-CPU data, run the scheduler on multiple cores | real concurrency, per-CPU state |
| **17** | Networking | NIC driver (virtio-net or e1000): send/receive raw Ethernet frames | DMA, ring buffers |
| **18** | Networking | Minimal network stack: ARP + IPv4 + ICMP (reply to host `ping`), or integrate `smoltcp` | protocol layering |

### Notes

- **Each stage stays one `cargo run`-verifiable commit**, the same discipline as
  stages 0-8 — and from Stage 9 on, each new mechanism should ship with a
  `#[test_case]` (e.g. "after an interrupt from ring 3, the saved `CPL` is 3").
- Large stages split into sub-steps with their own commits, as 4 (4a/4b/4c) and 6
  (6a/6b) did — e.g. 9a sets up the GDT user segments and TSS `rsp0`, 9b performs
  the ring-3 descent.
- **Version caveats**: the `x86_64` crate's TSS / MSR APIs, ATA's QEMU wiring, and
  ELF-crate choices have all shifted across versions — verify against current docs
  rather than assuming from memory.
- Optional, non-blocking refinements: upgrade `bootloader` 0.9 → 0.11 (framebuffer,
  modern boot info), give kernel thread stacks a guard page, and eventually unify
  the async executor with the thread scheduler.

## References

- **Stages 0-5** map almost section-by-section to Philipp Oppermann's
  *Writing an OS in Rust* (second edition):
  https://os.phil-opp.com/
  When a concept is unclear, that's where the most detailed explanations are.
  Be sure to read the **second edition** (which uses the `bootloader` crate);
  the first edition uses GRUB and is no longer maintained.
- **OSDev Wiki** (encyclopedic reference): https://wiki.osdev.org/
- **Rust OSDev monthly** (ecosystem news, crate updates): https://rust-osdev.com/

- **Stages 6-8** go beyond the tutorial above and are where you "build it
  yourself and understand it deeply."

## Suggested vibe-coding workflow

1. Have Claude Code focus on **one stage at a time**, e.g.:
   "Implement Stage 2: set up the IDT, register a breakpoint exception handler,
    and trigger a breakpoint in `_start` to verify it."
2. After it's done, run `cargo run` and confirm the terminal output matches
   expectations.
3. If it passes, `git commit` and update the Status column in this table.
4. Review the diff to understand each line, then move on to the next stage.

Kernel code is extremely sensitive to correctness — a single wrong pointer or
page-table entry can triple-fault and reboot the kernel. **Small steps + verifying
each one in QEMU** is far more reliable than having the model generate a large
chunk all at once.
