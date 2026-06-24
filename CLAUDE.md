# Project notes for Claude Code

**Aether** — a from-scratch educational x86_64 OS kernel in Rust, built in small,
verifiable iterations to understand OS internals deeply.

## TL;DR (the rules that matter most)

1. **Everything in English** — code, comments, docs, commits, and anything on
   GitHub. No non-English text anywhere in the repo.
2. **One stage at a time** — don't generate code across multiple roadmap stages
   unless asked. Each stage must `cargo run` cleanly and be one commit.
3. **Correctness over speed** — kernel `unsafe` / page tables / interrupts can
   triple-fault on a single mistake. Add a SAFETY comment to every `unsafe` block.
4. **Verify in QEMU** — every new feature ships with a way to trigger and see it
   in the terminal via `cargo run`.
5. **Commits follow Conventional Commits** — `<type>: <imperative summary>`,
   lowercase, no period, ≤50 chars. Use `/commit`, which also pushes to the
   personal remote.

The author is learning OS development by hand. Explain concepts as you go.
Details below.

## Project background

- Language: Rust **nightly** (required for bare-metal development).
- Architecture: x86_64.
- Boot: uses the `bootloader` crate (0.9.x) + the `bootimage` tool; `cargo run`
  boots it in QEMU.
- Environment: WSL2 (Ubuntu) on Windows 11, with QEMU emulating bare metal.
- Primary learning reference: Philipp Oppermann's *Writing an OS in Rust*,
  second edition (os.phil-opp.com).

The full staged plan is in `ROADMAP.md`. **Stages 0–4 are done**: serial output,
the VGA text buffer, the IDT with CPU exception handlers, hardware interrupts via
the 8259 PIC (timer and keyboard), and Stage 4's virtual-memory work — page-table
access and translation (4a), a frame allocator over the bootloader memory map plus
hand-made page mappings (4b), and a heap backed by a hand-written fixed-size block
allocator over a linked-list fallback, making `Box`/`Vec`/`Rc` usable (4c).
**Stage 5 is also done**: cooperative multitasking with `async`/`await` — a
`Task`/`TaskId` abstraction over heap-allocated futures, an async keyboard whose
interrupt handler only enqueues scancodes onto a lock-free queue, and a
waker-driven executor that polls a task only when woken and halts the CPU (`hlt`)
when idle. **Stage 6 is also done**: preemptive scheduling with independent
kernel threads — each thread owns a heap stack and a saved register context, a
hand-written `context_switch` swaps between them (6a, cooperative via
`yield_now`), and the timer interrupt drives a round-robin scheduler that
preempts a running thread without its cooperation (6b). **Stage 7 is also done**:
an interactive kernel shell (REPL) on the revived async executor — the shell is
an async task that awaits decoded keystrokes from the keyboard `ScancodeStream`,
buffers a line (with Backspace editing), and on Enter dispatches it to a built-in
command (`help`, `echo`, `clear`, `ticks`, `uptime`, `mem`); a boot self-test
makes it verifiable headless. There is no user mode yet, so commands are direct
kernel calls — a precursor to system calls. The thread scheduler is dormant
during this stage. **Stage 8 is also done**: an in-memory file system (`fs`) — a
heap-backed tree of file and directory nodes addressed by `/`-separated paths,
with `RamFs` operations (mkdir/write/read/list/remove) behind a global mutex, and
shell commands (`ls`, `cat`, `write`, `mkdir`, `rm`, `cd`, `pwd`) over it with a
current working directory and relative-path (`..`) resolution. This completed the
originally planned roadmap (stages 0-8). **Stage 9 is now also done**: user mode
(ring 3). Stage 9a adds ring 3 GDT segments and a TSS `rsp0` stack (`gdt.rs`);
Stage 9b (`usermode.rs`) maps a user-accessible page holding a tiny ring 3
program, forges an interrupt-return frame and `iretq`s into it, then proves it —
the timer handler sees `CPL == 3` — before rewriting that frame to resume the
kernel in ring 0. **Stage 10 is also done**: system calls via `int 0x80`
(`syscall.rs`) — the gate's DPL is 3 so ring 3 may invoke it, arguments cross on a
stack-based ABI, and the ring 3 program now calls `write` (the kernel prints on
its behalf) then `exit` (which hands control back). **Stage 11 is also done**:
per-process address spaces and an ELF loader. Stage 11a (`memory.rs`) adds an
`AddressSpace` that clones the active kernel L4 into a fresh frame and switches CR3
onto it and back — every space must map the kernel, or the switch triple-faults;
since bootloader 0.9 keeps the kernel, heap, and physical-memory window in the
lower half, the clone copies every present top-level entry. Stage 11b adds an
ELF64 parser (`elf.rs`) and a loader (`process.rs`) that maps a program's
`PT_LOAD` segments into a fresh space through the physical-memory window (the space
is not yet active) and verifies the load by translating its entry point. **Stage
12a is also done**: `process::run` maps the image a user stack, switches CR3 to it,
and `iretq`s into ring 3 — the loaded program runs on its own CR3 (the `int 0x80`
handler still reaches the kernel because every space maps it), prints via a syscall,
and `exit`s back to the kernel, which switches CR3 back; this replaced the Stage
9b/10b hand-mapped excursion. **Stage 12b is also done**: a cooperative scheduler in
`process.rs` — `spawn` queues loaded programs, `run` enters the first in ring 3, and
the ring 3 `yield`/`exit` syscalls switch processes (rewriting the interrupt frame
and CR3 from inside the handler). `yield` saves the caller's resume point and
round-robins to the next; `exit` drops it. Two programs that each run several
`write`+`yield` rounds therefore interleave their output (#1, #2, #1, #2, ...), and
being byte-identical yet printing different messages from the same virtual address,
they also prove address-space isolation. **Stage 12c is also done**: preemptive
scheduling, in three sub-steps. 12c-1 and 12c-2 route the timer *and* the `int 0x80`
syscall through hand-written *naked* stubs that capture the full register set into a
`TrapFrame`, so a context switch saves and restores every general-purpose register, not
just the interrupt frame. 12c-3 sets IF in the ring 3 frame and, on a timer tick that
interrupted ring 3, has `timer_dispatch` save the running process's `TrapFrame` and
round-robin to the next process (`process::on_timer_tick`) — true preemption: two
programs busy-spinning between writes interleave with no `yield` required (the
`yield`/`exit` syscalls remain as voluntary switch points). Stage 12 also adds `wait`:
a parent blocks until its child exits and collects the child's exit code (delivered in
rax, since the kernel often wakes the parent from the child's exit in a *different*
address space where the parent's user stack is unreachable). **Stage 12d is also done**:
process-creation syscalls — a ring 3 process calls `spawn` (`SYS_SPAWN`) to load a
kernel-known program into a fresh address space and enqueue it as its child (returning the
new pid), so the wait demo's parent now creates its own child at runtime instead of the
kernel pre-spawning it. This needed a globally reachable kernel frame allocator
(`memory.rs`), since the ELF load runs inside the syscall handler, far from `kernel_main`'s
locals. **Stage 13a (persistence track) is also done**: a polled ATA PIO disk driver
(`ata.rs`) reads raw 512-byte sectors from the primary IDE master, verified at boot and in
a test against the boot disk's MBR signature; it runs the drive with interrupts disabled
(nIEN), since the kernel polls and registers no ATA IRQ handler (an unhandled IRQ14 would
otherwise cascade into a double fault). That work also added page-fault and
general-protection-fault handlers (`interrupts.rs`). `ROADMAP.md` carries the forward plan
(stages 9-18): the user-space main line (system calls, per-process address spaces + ELF,
multiprocessing), plus persistence, APIC/SMP, and networking tracks.

## Language and writing conventions

- **Everything in English.** All code, comments, documentation, commit messages,
  and any content pushed to GitHub (README, issues, PRs, releases, discussions,
  etc.) must be written in English. Do not introduce non-English text anywhere in
  the repository.
- Keep prose clear and concise. Prefer short, direct sentences.

## Commit message conventions

- Follow **Conventional Commits**: `<type>: <short imperative summary>`.
- The summary is lowercase, imperative mood ("add", not "added"/"adds"), no
  trailing period, and **no longer than 50 characters**.
- Common types: `feat` (new feature), `fix` (bug fix), `docs` (documentation),
  `refactor` (code change that neither fixes a bug nor adds a feature),
  `chore` (build/tooling/maintenance), `test` (tests), `perf` (performance).
- Keep commits small and focused — one logical change per commit.
- Add a body only when the change needs explanation (what/why, not how); wrap the
  body at ~72 characters and separate it from the summary with a blank line.
- Examples:
  - `feat: add VGA text buffer driver`
  - `fix: correct IDT entry for breakpoint`
  - `docs: document serial port setup in readme`
  - `refactor: extract hlt_loop helper`
  - `chore: pin bootloader to 0.9.x`

## Core constraints

1. **`#![no_std]` environment**: the standard library is unavailable. Only `core`
   may be used, plus `alloc` after a heap allocator is implemented. Do not pull in
   crates that depend on `std`.

2. **One stage at a time**: unless the author explicitly asks otherwise, do not
   generate large amounts of code spanning multiple stages. Each stage should
   `cargo run` cleanly on its own and be worth a single git commit.

3. **Correctness over speed**: in kernel code, a single wrong pointer, page-table
   entry, or interrupt handler can triple-fault and reboot, or cause
   hard-to-debug crashes. When dealing with `unsafe`, memory mapping, page tables,
   or interrupts, be careful and add a SAFETY comment explaining why the `unsafe`
   block is sound.

4. **Every step must be verifiable in QEMU**: when implementing a new feature,
   also provide a way to trigger / verify it in `_start` or a test, so the author
   can immediately see the expected output in the terminal.

## Common commands

```bash
cargo run            # build and boot the kernel in QEMU
cargo build          # build only
cargo test           # run the unit tests headless in QEMU (see src/testing.rs)
cargo bootimage      # only generate the bootable disk image
```

Exit QEMU: `Ctrl-A` then `X`.

## Current files

- `src/main.rs`: kernel entry `_start`, panic handler, `hlt_loop`.
- `src/serial.rs`: serial output, providing the `serial_print!` /
  `serial_println!` macros.
- `src/vga_buffer.rs`: VGA text-buffer driver, providing the `print!` /
  `println!` macros that write to the screen.
- `src/gdt.rs`: the Global Descriptor Table and Task State Segment, providing a
  dedicated IST stack for the double fault handler (loaded before the IDT).
- `src/interrupts.rs`: the IDT, the CPU exception handlers (breakpoint, double
  fault, and — since Stage 13a — page-fault and general-protection-fault handlers
  that log CR2 / the error code and halt, instead of escalating to a double fault),
  and the hardware interrupt handlers along with the 8259 PIC setup. Since Stage 12c the timer (IRQ0) uses a hand-written *naked* entry
  (`timer_interrupt_entry`) that pushes the full register set into a `TrapFrame`
  and calls `timer_dispatch`, which counts the tick, sends the EOI, then — if the
  tick interrupted ring 3 — preempts the running user process via
  `process::on_timer_tick` (a ring 0 tick instead feeds the dormant
  `thread::schedule`). The keyboard handler (since Stage 5) just pushes the raw
  scancode onto the async keyboard's queue. Stage 10 registers the `int 0x80`
  syscall gate (DPL 3); since Stage 12c-2 it too points at a naked stub
  (`syscall::syscall_entry`) that builds the same `TrapFrame`, so a `yield`/`exit`
  saves and restores a full register context.
- `src/memory.rs`: virtual-memory helpers — reads CR3 and builds an
  `OffsetPageTable` over the active page tables (via the bootloader's complete
  physical-memory mapping) for translating virtual addresses, plus a
  `BootInfoFrameAllocator` that hands out usable physical frames from the memory
  map and a helper that creates new page mappings. Stage 11a also adds an
  `AddressSpace` (a process's L4) that clones the kernel's present top-level
  entries into a fresh frame, hands out an `OffsetPageTable` over it (to map an
  inactive space), and switches CR3 onto it and back. Stage 12d adds a globally
  reachable kernel frame allocator + physical-memory offset
  (`install_kernel_allocator`/`with_kernel_frame_allocator`), so the `spawn` syscall can
  load an ELF from inside the trap handler (which cannot borrow `kernel_main`'s locals).
- `src/allocator.rs`: the kernel heap — maps a fixed virtual range to frames and
  registers a `#[global_allocator]` (a hand-written fixed-size block allocator
  over a linked-list fallback), so the `alloc` crate's `Box`/`Vec`/`Rc`/`String`
  become usable.
- `src/task/`: Stage 5 cooperative multitasking — `mod.rs` (`Task` and the unique
  `TaskId`), `simple_executor.rs` (a naive busy-polling executor, kept for
  reference), `keyboard.rs` (the async keyboard: a lock-free scancode queue filled
  by the IRQ1 handler and drained by a `Stream`-based task that decodes and
  echoes), and `executor.rs` (the waker-driven executor that sleeps on `hlt` when
  no task is ready). Revived in Stage 7 to drive the shell.
- `src/thread/`: Stage 6 preemptive kernel threads — `mod.rs` (`Thread`/`ThreadId`,
  a round-robin `Scheduler`, `spawn`/`yield_now`/`run`, the fabricated initial
  stack frame, and `schedule` called from the timer to preempt) and `switch.rs`
  (the naked `context_switch` that saves callee-saved registers and swaps stacks).
  Dormant during Stage 7 (marked `#[allow(dead_code)]`); the timer still calls
  `schedule`, but it no-ops because preemption is never armed.
- `src/shell.rs`: Stage 7-8 interactive shell — an async task that reads decoded
  keystrokes from the keyboard `ScancodeStream`, buffers a line (with Backspace)
  against a current working directory, and on Enter routes it through a `dispatch`
  table of built-in commands (including the Stage 8 file commands). Includes a
  boot `selftest` so the shell and file system are verifiable without a keyboard.
- `src/fs.rs`: Stage 8 in-memory file system — a heap-backed tree of `File`/`Dir`
  nodes addressed by `/`-separated paths, exposed as a global `RamFs` behind a
  mutex with `mkdir`/`write`/`read`/`list`/`remove`/`is_dir`. No disk, no
  persistence, no VFS layer.
- `src/usermode.rs`: the ring 3 entry/return mechanism — `enter` forges an
  interrupt-return frame (`initial_user_frame`: entry point + user stack; since Stage
  12c-3 IF is *set* so the process is preemptible) and `iretq`s into ring 3;
  `resume_kernel` rewrites an in-flight interrupt's frame to return to the kernel (the
  scheduler uses it when the last process exits). The per-process context is now a full
  `TrapFrame` saved/restored by `process.rs`, so the old `save_frame`/`load_frame`
  helpers are gone. (Stage 9a added the ring 3 GDT segments and the TSS `rsp0` stack in
  `gdt.rs`.)
- `src/syscall.rs`: Stage 10 system calls over `int 0x80` (its IDT gate's DPL is 3 so
  ring 3 may invoke it) with a stack-based argument ABI:
  `write`/`getpid`/`exit`/`yield`/`wait`/`spawn` (Stage 12d's `spawn` creates a child
  process from a kernel-known program and returns its pid).
  Since Stage 12c-2 the entry is a hand-written *naked* stub (`syscall_entry`) that
  builds a full `TrapFrame` and calls `syscall_dispatch`, mirroring the timer — so the
  general-purpose registers survive a context switch. Ring 3 `yield`/`exit` call
  `process::on_user_yield`/`on_user_exit` (which switch to another process or resume
  the kernel); an `invoke` helper drives the value-returning calls from ring 0 (the
  boot demo and the tests).
- `src/elf.rs`: Stage 11b minimal ELF64 parser — validates the header (x86-64,
  ET_EXEC), bounds-checks the program-header table, and iterates the `PT_LOAD`
  segments. Pure (reads bytes, no page tables), so it is unit-testable on its own.
- `src/process.rs`: Stage 11b ELF loader + Stage 12 scheduler — parses an ELF
  (via `elf.rs`), clones the kernel into a fresh `AddressSpace`, maps each `PT_LOAD`
  segment plus a user stack into it (writing through the physical-memory window
  while the space is inactive), and bundles it as a `UserImage`. A round-robin
  `Scheduler` (a `Mutex`-guarded ready queue) holds `Process`es, each carrying a full
  `TrapFrame` context: `spawn` enqueues, `run` enters the first in ring 3 (saving the
  kernel CR3 for the return), the `yield`/`exit` syscalls (`on_user_yield`/
  `on_user_exit`) round-robin voluntarily, and since Stage 12c-3 the timer preempts via
  `on_timer_tick` — each switching CR3 and the saved `TrapFrame`, or `resume_kernel`
  when none remain. Stage 12's `wait` (`on_user_wait`) blocks a parent (into a `blocked`
  list) until a child exits, when `on_user_exit` wakes it with the child's code in rax
  (or leaves a `Zombie` if the parent has not waited yet). `return_to_kernel_space`
  switches CR3 back in the resume continuation. Stage 12d adds `spawn` (`on_user_spawn`):
  a ring 3 process loads a kernel-known program (`program_elf`) into a fresh space and
  enqueues it as its child, returning the new pid — loading against the kernel CR3 (not
  the caller's populated space) and restoring the caller's CR3 before returning. The boot
  demo runs two `write`+busy-spin+`yield` workers interleaved under preemption, plus a
  parent that `spawn`s its own child via `SYS_SPAWN` and `wait`s for it.
- `src/ata.rs`: Stage 13a block device driver — a minimal ATA (IDE) disk driver in
  PIO mode. `read_sector` reads one raw 512-byte sector from the primary master by
  the polled READ SECTORS (LBA28) protocol: write the LBA/count registers, issue the
  command, poll the status register for BSY-clear + DRQ-set, then read 256 16-bit
  words from the data port. It disables the drive's interrupt (nIEN) since the kernel
  polls and has no ATA IRQ handler. Read-only and single-sector for now; writes (13b,
  against a scratch disk) come later.
- `src/testing.rs`: the in-QEMU unit-test harness. Built on the
  `custom_test_frameworks` feature, it provides a custom `test_runner`,
  `exit_qemu` (which ends the VM through the `isa-debug-exit` device so the run
  reports a pass/fail status code), and the `#[test_case]`s themselves (heap and
  file-system checks). The whole module is `#[cfg(test)]`: `cargo test` builds it,
  but the real kernel image never includes it. `kernel_main` runs the generated
  `test_main()` instead of the shell when built for testing.
- `.cargo/config.toml`: the bare-metal target (`x86_64-unknown-none`), build-std,
  and the QEMU runner config.
- `.claude/settings.json`: pre-approved permissions (cargo + git, including
  `git push` — this is a personal project).
- `.claude/commands/commit.md`: the `/commit` slash command — commits in
  Conventional Commits format and pushes to the personal remote.

## Suggested way to help

- Before starting a new stage, briefly explain what it implements, which OS
  concepts it covers, and which files will be added/modified.
- After writing code, prompt the author to run `cargo run` and describe the
  expected output.
- Once it passes, suggest updating the corresponding stage status in `ROADMAP.md`
  and provide a suitable Conventional Commits message.
- If a crate version or API may have changed, remind the author to verify rather
  than assuming from memory.
- On first-time git setup, remind the author that the commit email
  (`git config user.email`) must match an email registered on their GitHub
  account, otherwise commits won't appear on their contribution graph.
