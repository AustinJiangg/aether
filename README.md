# Aether

A from-scratch, iteratively-built **educational x86_64 operating system kernel**, written in Rust.

The goal is not to clone Ubuntu, but to implement the core mechanisms of an OS by
hand — booting, interrupts, memory management, process scheduling, file systems —
in order to **deeply understand how an OS works**.

Currently at **Stage 1**: the kernel boots, prints to the terminal over the serial
port, and drives the VGA text buffer to print directly to the screen. See
[`ROADMAP.md`](./ROADMAP.md) for the full iteration plan.

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
- On success, the terminal shows the kernel's boot output ("Hello from Aether
  kernel!" etc.).
- **Exit QEMU**: press `Ctrl-A`, release, then press `X`.

To only build the bootable image without running:

```bash
cargo bootimage
# Output at target/x86_64-unknown-none/debug/bootimage-aether.bin
```

---

## 3. Project structure

```
aether/
├── Cargo.toml             # Dependencies and bootimage/QEMU run args
├── rust-toolchain.toml    # Pins the nightly toolchain
├── .cargo/
│   └── config.toml        # Target (x86_64-unknown-none), build-std, runner
├── src/
│   ├── main.rs            # Kernel entry _start + panic handler
│   ├── serial.rs          # Serial output (serial_print! / serial_println!)
│   └── vga_buffer.rs      # VGA text buffer (print! / println!)
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
A: This is expected. We currently use `-display none`, so kernel output goes
   straight to the terminal over serial. Once you reach the VGA stage, remove the
   `"-display", "none"` entries in `Cargo.toml` to get a graphical window (WSL2
   needs WSLg support).

**Q: Compilation is slow / stuck at "Compiling compiler_builtins"?**
A: The first build compiles standard-library components from source. Please be
   patient; incremental builds afterward are much faster.

---

## 5. Learning references

- *Writing an OS in Rust*, second edition: https://os.phil-opp.com/ (main reference for Stages 0–5)
- OSDev Wiki: https://wiki.osdev.org/
- Rust OSDev monthly: https://rust-osdev.com/

---

## License

Released under the [MIT License](./LICENSE).
