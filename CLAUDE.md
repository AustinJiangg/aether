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
during this stage. **Next is Stage 8** (an in-memory file system).

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
- `src/interrupts.rs`: the IDT, the CPU exception handlers (breakpoint and
  double fault), and the hardware interrupt handlers along with the 8259 PIC
  setup — the timer counts ticks and (since Stage 6b) calls `thread::schedule`
  after its EOI to preempt the running thread, and the keyboard handler (since
  Stage 5) just pushes the raw scancode onto the async keyboard's queue.
- `src/memory.rs`: virtual-memory helpers — reads CR3 and builds an
  `OffsetPageTable` over the active page tables (via the bootloader's complete
  physical-memory mapping) for translating virtual addresses, plus a
  `BootInfoFrameAllocator` that hands out usable physical frames from the memory
  map and a helper that creates new page mappings.
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
- `src/shell.rs`: Stage 7 interactive shell — an async task that reads decoded
  keystrokes from the keyboard `ScancodeStream`, buffers a line (with Backspace),
  and on Enter routes it through a `dispatch` table of built-in commands. Includes
  a boot `selftest` so the shell is verifiable without a keyboard.
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
