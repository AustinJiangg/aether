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

> **Status:** Stages 9-11 are complete; Stage 12 is complete (12a-12c plus a `wait`
> syscall). Process-creation syscalls (so a process can spawn another, rather than the
> kernel spawning them all at boot) are a later step. Stage 9 reaches
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
> address space, where only the saved register, not the user stack, is reachable). Still
> later: process-creation syscalls so a process can spawn another (today the kernel
> spawns them all at boot).

| Stage | What to build | OS concepts | Smallest verifiable step |
|-------|---------------|-------------|--------------------------|
| **9**  | Drop to user mode (ring 3) | privilege rings, GDT user segments, TSS `rsp0`, the `iretq` descent | Map a user page, `iretq` into ring-3 code; a timer interrupt fires and the handler observes `CPL == 3` in the saved frame — proving both that we reached ring 3 and that `rsp0` keeps the interrupt from triple-faulting. |
| **10** | System calls | the user/kernel boundary, register-passed arguments | Start with `int 0x80` (IDT gate `DPL = 3`), simpler than the `syscall`/`sysret` MSRs. Implement `write`, `exit`, `getpid`; the Stage 9 program prints via `sys_write` and exits cleanly. |
| **11** | Process address space + ELF loader | address-space isolation, higher-half kernel, ELF64 | (11a) each process its own page table (own CR3), kernel mapped into every space's higher half; (11b) a minimal ELF64 loader maps `PT_LOAD` segments. Hardest step: copy the top-level entries covering the kernel + physical-memory map into the new L4, or the kernel is unreachable after the CR3 switch. |
| **12** | Multiple user processes | process scheduling, time-sliced userland | Tie processes into the (extended) thread scheduler, switching CR3 on each context switch; two user programs interleave their output under timer preemption. Add `spawn`/`exit`/`wait`. |

### Parallel tracks (any order, after the main line)

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
