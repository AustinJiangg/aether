# Aether

A from-scratch, iteratively-built **educational x86_64 operating system kernel**, written in Rust.

The goal is not to clone Ubuntu, but to implement the core mechanisms of an OS by
hand — booting, interrupts, memory management, process scheduling, file systems —
in order to **deeply understand how an OS works**.

**All planned stages (0–8) are complete.** The kernel boots on bare metal and
brings up: serial and VGA output; CPU exception handling and hardware interrupts
(timer + keyboard) via the IDT and the 8259 PIC; paging with a kernel heap (so
`Box`/`Vec` work); `async`/`await` cooperative tasks; preemptive kernel threads
with a hand-written context switch; an interactive shell; and an in-memory file
system. It boots straight into the shell — type `help`. See
[`ROADMAP.md`](./ROADMAP.md) for the staged plan and [`CLAUDE.md`](./CLAUDE.md)
for design notes.

---

## 1. Environment setup (WSL2 / Ubuntu)

> Assumes WSL2 (Ubuntu) on Windows 11. The bottleneck in OS development is the
> toolchain, not performance; a modest machine is plenty, and the GPU is unused.

```bash
# 1. Install Rust (skip if already installed)
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
source "$HOME/.cargo/env"

# 2. Install the nightly toolchain and bare-metal components
#    (rust-toolchain.toml auto-switches to nightly inside this project, but
#     these two components still need to be installed once)
rustup toolchain install nightly
rustup component add rust-src llvm-tools-preview --toolchain nightly

# 3. Install bootimage (packages the kernel + bootloader into a bootable image)
cargo install bootimage

# 4. Install the QEMU virtual machine
sudo apt update
sudo apt install -y qemu-system-x86
```

Verify the installation:

```bash
rustc +nightly --version          # should contain "nightly"
qemu-system-x86_64 --version
bootimage --version
```

---

## 2. Build and run

From the project root:

```bash
cargo run
```

- The first run compiles `core` / `compiler_builtins` from source, which takes a
  few minutes — this is normal; subsequent runs are much faster.
- On success the kernel prints its boot log and then drops into an interactive
  shell prompt (`aether:/>`). Try `help`, `ls`, `mkdir /docs`,
  `write /docs/a hello`, `cat /docs/a`.
- **Exit QEMU**: press `Ctrl-A`, release, then press `X`.

To only build the bootable image without running:

```bash
cargo bootimage
# Output at target/x86_64-unknown-none/debug/bootimage-aether.bin
```

To run the unit tests (headless, inside QEMU):

```bash
cargo test
```

The tests boot the real kernel, run the `#[test_case]`s (heap, file system, ...),
and report each result over the serial log; the kernel then exits QEMU with a
status code that `cargo test` maps to pass or fail. No window is opened, so this
also works over SSH / in CI. See [`src/testing.rs`](./src/testing.rs).

---

## 3. Project structure

```
aether/
├── Cargo.toml             # Dependencies and bootimage/QEMU run args
├── rust-toolchain.toml    # Pins the nightly toolchain
├── .cargo/
│   └── config.toml        # Target (x86_64-unknown-none), build-std, runner
├── src/
│   ├── main.rs            # Kernel entry + panic handler; wires the stages together
│   ├── serial.rs          # Serial output (serial_print! / serial_println!)
│   ├── vga_buffer.rs      # VGA text buffer (print! / println!)
│   ├── gdt.rs             # GDT + TSS (IST stack for the double-fault handler)
│   ├── interrupts.rs      # IDT, CPU exceptions, and the 8259 PIC (timer + keyboard)
│   ├── memory.rs          # Paging: page-table walk + physical frame allocator
│   ├── allocator.rs       # Kernel heap (fixed-size block over a linked-list fallback)
│   ├── task/              # async/await tasks: waker-driven executor + async keyboard
│   ├── thread/            # Preemptive kernel threads + hand-written context switch
│   ├── shell.rs           # Interactive shell (REPL) and built-in commands
│   ├── fs.rs              # In-memory hierarchical file system
│   └── testing.rs         # In-QEMU unit-test harness (cargo test)
├── ROADMAP.md             # Staged iteration plan
├── CLAUDE.md              # Project context and conventions for Claude Code
└── README.md
```

---

## 4. FAQ

**Q: `cargo run` fails saying it can't find `rust-src`?**
A: Run `rustup component add rust-src --toolchain nightly`.

**Q: It says `bootimage` is not found?**
A: Make sure you ran `cargo install bootimage` and that `~/.cargo/bin` is on your `PATH`.

**Q: No QEMU window pops up?**
A: `cargo run` opens a graphical window for the VGA output and also mirrors output
   to the terminal over serial; the window needs WSLg (ships with Windows 11) or
   an X server. On a headless machine, add `"-display", "none"` back to the
   `run-args` in `Cargo.toml` — the serial log keeps working without a window.

**Q: Compilation is slow / stuck at "Compiling compiler_builtins"?**
A: The first build compiles standard-library components from source. Please be
   patient; incremental builds afterward are much faster.

---

## 5. Learning references

- *Writing an OS in Rust*, second edition: https://os.phil-opp.com/ (closely tracks Stages 0–5; Stages 6–8 go beyond it)
- OSDev Wiki: https://wiki.osdev.org/
- Rust OSDev monthly: https://rust-osdev.com/

---

## License

Released under the [MIT License](./LICENSE).
