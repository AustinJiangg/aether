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
| **4** | Paging + heap allocator, make `Box` / `Vec` usable | virtual memory, address spaces | Todo |
| **5** | Cooperative multitasking (tasks written with `async` / `await`) | processes / tasks, scheduling | Todo |
| **6** | Preemptive scheduling, independent kernel threads | context switching, time slices | Todo |
| **7** | Simple shell + built-in commands | system calls, user interaction | Todo |
| **8** | In-memory simple file system | file abstraction, VFS | Todo |

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
