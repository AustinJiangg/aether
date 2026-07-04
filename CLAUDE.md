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
general-protection-fault handlers (`interrupts.rs`). **Stage 13b is also done**: ATA PIO
sector *writes* — `write_sector` issues WRITE SECTORS (0x30), pushes 256 little-endian words
out the data port, waits for the drive to commit, then issues CACHE FLUSH (0xE7) for
durability. To keep the boot image safe, a `Drive` enum names the target (primary master =
boot disk vs. primary slave = scratch disk) at every call site, and writes go only to the
scratch disk — a separate `scratch.img` attached as the primary slave (`Cargo.toml`
`run-args`/`test-args`), which a host `build.rs` creates if missing (QEMU won't start without
the backing file). A boot demo and a test write a sector and read it back to prove an exact
round-trip. **Stage 14a is also done**: the VFS seam — `fs.rs`'s six file operations are
factored into a `FileSystem` trait (the virtual-filesystem layer that lets different
filesystems coexist behind one interface), with `RamFs` as the first implementor. A pure
refactor: the shell still calls the same global `fs::*` functions, and a new test drives a
`RamFs` through a `&mut dyn FileSystem` trait object. **Stage 14b-1 is also done**: the FAT
boot sector. `fat.rs` reads sector 0 of a real FAT16 disk and parses its BPB (BIOS Parameter
Block) into geometry, then derives the FAT/root-directory/data region LBAs. The disk
(`fat.img`) is formatted by the host's `mkfs.fat` in `build.rs` (with a known `HELLO.TXT`)
and attached as the secondary IDE master, which extended the ATA driver to a second bus
(`Drive::SecondaryMaster`, ports 0x170/0x376). **Stage 14b-2a is also done**: reading a file
off the FAT volume — a mounted `Fat` handle (`Fat::mount` parses the BPB) with
`read_file(name)` that scans the root directory for the 8.3 entry (case-insensitive) and
follows the FAT16 cluster chain, truncating to the directory's size; a boot demo and a test
read the known `HELLO.TXT`. **Stage 14b-2b is also done**: `Fat` implements the Stage 14a
`FileSystem` trait, so the FAT volume is usable through a `&dyn FileSystem` object like `RamFs`
(`read`/`list`/`is_dir` over the root; the read-only driver returns `Unsupported` for the
mutating `mkdir`/`write`/`remove`), with `FsError` gaining `Unsupported`/`Io` variants and a
`From<FatError>` mapping. **Stage 14b-3 is also done**: the FAT volume is mounted into the VFS
at `/mnt` via a minimal one-entry mount table in `fs.rs` (the six `fs::*` wrappers route a
`/mnt`-prefixed path to the mounted `Box<dyn FileSystem>`, everything else to `RamFs`), so the
shell's `ls`/`cat`/`cd` reach real disk files. **Stage 14c-1 is also done**: FAT *writes* —
`Fat::write_file` allocates a cluster chain (`alloc_cluster`/`write_chain`), writes the data
through `ata::write_sector`, and creates/overwrites the root-directory entry (updating every FAT
copy), wired into `FileSystem::write` so the shell's `write /mnt/foo` lands on disk and survives
a reboot. **Stage 14c-2 is also done**: `Fat::remove_file` frees a file's cluster chain and
marks its directory entry deleted, wired into `FileSystem::remove` (so `rm /mnt/foo` works).
**This completes Stage 14** — an on-disk FAT16 filesystem with read *and* write, coexisting with
`RamFs` behind the VFS (root-level `mkdir` since Stage 14d-1; Stage 14d-2 adds read-path
subdirectory traversal, so `cd`/`ls`/`cat` descend into subdirectories; Stage 14d-3 extends it to
the file write path, so `write`/`rm` land inside subdirectories too; Stage 14d-4 does the same for
`mkdir`, so a directory can be created inside a subdirectory with a correct `..` back-link; Stage
14d-5 grows a subdirectory past its first cluster (appending a cluster to its chain) when it fills
up; and Stage 14d-6 adds `rmdir`, so `rm` removes an empty directory too — completing the FAT
subdirectory work). **Stage 15a (hardware
track) is also done**: the Local APIC and its timer (`apic.rs`) replace the 8259 PIC's timer. It
maps the LAPIC's MMIO page uncacheable (`NO_CACHE` — device registers must bypass the cache),
software-enables the APIC via the spurious-vector register, and masks the 8259 PIC, so hardware
interrupts now arrive through the APIC. Because the LAPIC timer's frequency is not architecturally
fixed, it is *calibrated* against the PIT over a 10 ms polled window, then run periodically at 100
Hz on vector 32 — the same gate the PIT timer used, so the naked timer entry is unchanged; the EOI
moves from the PIC to the LAPIC's EOI register, and a no-op handler backs the spurious vector.
Timer ticks and preemption now run on the APIC. **Stage 15b** then brought up the IO-APIC —
accessed *indirectly* through IOREGSEL/IOWIN — and routed the keyboard's IRQ1 to vector 33 through
it (its EOI also moving to the LAPIC), so the keyboard works again. **This completes Stage 15**: the
8259 PIC is fully retired (masked), and both the timer and the keyboard arrive through the APIC,
clearing the last prerequisite for SMP (Stage 16). **Stage 16a (SMP track) is now also done**:
`acpi.rs` parses the ACPI tables (RSDP -> RSDT/XSDT -> MADT) to enumerate the machine's CPU cores,
each a `CpuCore` with its Local APIC id and a BSP flag, reading every table through the
physical-memory window (pure byte parsing, length/checksum bounds-checked, degrading to "BSP only"
on failure). `apic.rs` gains `lapic_id()` so the running core flags its own MADT entry as the BSP;
the rest are APs, halted until 16b wakes them with INIT-SIPI-SIPI. QEMU now boots with `-smp 4`, and
boot reports the four discovered cores (BSP apic id 0, APs [1, 2, 3]). **Stage 16b-1 is now also
done**: `apic.rs` gains `send_fixed_ipi` (writes the Local APIC ICR — destination, then issue — and
polls the delivery-status bit), the IPI send path Stage 16b-2's INIT-SIPI-SIPI will reuse; it is
proved end to end by a self-IPI (the BSP sends a fixed IPI to its own apic id on vector 0x40, handled
by `interrupts.rs`'s `ipi_test_handler` / `self_ipi_works`). **Stage 16b-2a is now also done**: a new
`smp.rs` wakes an application processor — `boot_one_ap` copies a tiny real-mode `global_asm!`
trampoline to physical 0x8000 and sends it INIT-SIPI-SIPI (new `apic::send_init_ipi` /
`send_startup_ipi` / `pit_sleep_us`); the AP writes an "alive" marker the BSP polls, proving a second
core executes our code. **Stage 16b-2b is now also done**: the trampoline grows into the full
`.code16`->`.code32`->`.code64` climb (temporary GDT, CR0.PE, CR4.PAE, kernel CR3, EFER.LME, CR0.PG,
raw-byte far jumps), writing a progress marker at each rung; `memory::ensure_identity_mapped` maps the
trampoline page to itself so the AP survives enabling paging, and `boot_one_ap` passes it the kernel
CR3. **Stage 16b-3 is now also done**: the trampoline loads a per-AP (heap-allocated) stack and jumps
to a Rust `ap_entry`, which bumps an `AP_ONLINE` atomic the BSP polls, then parks — a second core now
runs real kernel Rust ("AP apic id 1 is online"), and boot continues to the shell. (Two subtleties
fixed: the AP stack must be heap, not a large `static`, since the 0.9 bootloader leaves `.bss` past the
file image unmapped; and the trampoline must set `EFER.NXE` as well as `LME`, or the AP reserved-bit-
faults reading any NX page.) **Stage 17a (networking track) is now also done**: PCI bus enumeration
(`pci.rs`). Before the kernel can drive the Intel e1000 NIC it must *find* it, so `enumerate` scans
every PCI function's configuration space through the legacy `0xCF8`/`0xCFC` ports (write a
bus/device/function/offset address to CONFIG_ADDRESS, read the dword at CONFIG_DATA), reading each
device's vendor/device id, class code, header type, and BARs; `find_e1000` locates the card (vendor
`0x8086`, device `0x100E`), and `Device::mmio_bar`/`interrupt_line` decode its MMIO register base
and IRQ. QEMU now attaches `-device e1000,netdev=net0 -netdev user,id=net0` (SLIRP, no host
privileges), and boot reports the six bus-0 functions and the e1000 (`00:03.0`, BAR0 `0xfeb80000`,
IRQ 11). **Stage 17b-1 is now also done**: the kernel starts *driving* the e1000 (`e1000.rs`). Like
the APIC, the NIC is an MMIO device, so `init` maps its 128 KiB BAR0 register block into the kernel
address space **uncacheable** (`NO_CACHE`) and accesses it only through `volatile` reads/writes; to
prove register access works before the descriptor-ring work, it reads the Device Status register and
the card's MAC address out of Receive Address entry 0 (RAL0/RAH0, which QEMU's model pre-loads from
its emulated EEPROM) — boot reports MAC `52:54:00:12:34:56`, link up, full-duplex, and the handle is
stashed in a global for later sub-steps. **Stage 17b-2 is now also done**: `init` first *resets* the
card (mask all interrupts via IMC, set the self-clearing `CTRL.RST` and poll until it clears, drain
ICR) and *configures* it (set `CTRL.SLU`/`ASDE`, clear the reset/loss-of-signal/VLAN bits, zero the
128-entry multicast table) — the standard "put the device in a known state" opening move — then
re-reads the EEPROM-reloaded MAC; boot reports "reset completed". `ROADMAP.md` carries the forward
plan (stages 9-18): the user-space main line (system calls, per-process address spaces +
ELF, multiprocessing), plus persistence, APIC/SMP, and networking tracks.

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
  and the hardware interrupt handlers. The 8259 PIC is still set up (remapped) but,
  since Stage 15, *masked* — interrupts arrive through the APIC now (see `apic.rs`),
  and a no-op handler backs the LAPIC spurious vector. Since Stage 12c the timer uses
  a hand-written *naked* entry (`timer_interrupt_entry`) that pushes the full register
  set into a `TrapFrame` and calls `timer_dispatch`, which counts the tick, sends the
  EOI (since Stage 15 to the Local APIC via `apic::end_of_interrupt`), then — if the
  tick interrupted ring 3 — preempts the running user process via
  `process::on_timer_tick` (a ring 0 tick instead preempts the BSP's per-CPU
  run queue via `sched::preempt` — Stage 16d-5, unifying the async executor and
  kernel threads under one scheduler). The keyboard handler (since Stage 5) just pushes the raw
  scancode onto the async keyboard's queue. Stage 10 registers the `int 0x80`
  syscall gate (DPL 3); since Stage 12c-2 it too points at a naked stub
  (`syscall::syscall_entry`) that builds the same `TrapFrame`, so a `yield`/`exit`
  saves and restores a full register context. Stage 16b-1 adds a self-IPI test: a
  dedicated gate (`IPI_TEST_VECTOR` = 0x40) whose `ipi_test_handler` sets a flag and
  EOIs, driven by `self_ipi_works()` (sends a fixed IPI to this CPU and confirms it
  arrives) — proving the Local APIC IPI path before Stage 16b uses it to wake the APs.
  Stage 16d-1 makes `timer_dispatch` per-CPU aware (via `percpu::this_cpu_opt`): on an AP
  it bumps that core's per-CPU tick count, EOIs, and (Stage 16d-4) calls `sched::preempt`
  to round-robin this core's *own* per-CPU run queue — leaving the global tally and the
  process scheduler BSP-only; `init_idt_ap` points a woken AP's IDTR at the one shared IDT.
- `src/apic.rs`: Stage 15 APIC (Advanced Programmable Interrupt Controller) support.
  `init` maps the Local APIC's MMIO page uncacheable (`NO_CACHE`), software-enables it
  via the spurious-vector register, masks the 8259 PIC, then *calibrates* the LAPIC
  timer against the PIT (a 10 ms polled window — the LAPIC timer's frequency is not
  architecturally fixed) and runs it periodically at 100 Hz on vector 32 (reusing the
  existing timer entry). `end_of_interrupt` writes the LAPIC EOI register, replacing
  the 8259 EOI for APIC-delivered interrupts; `TIMER_HZ` is the kernel's tick rate (the
  shell's `uptime` reads it). Stage 15b adds the IO-APIC (accessed
  indirectly via IOREGSEL/IOWIN): `init` maps it uncacheable and programs the keyboard's
  redirection entry to route IRQ1 to vector 33 (`ioapic_redirection` reads an entry back
  for the test). The 8259 PIC is now masked; all hardware interrupts come via the APIC. Stage 16a adds
  `lapic_id()`, which reads this core's own Local APIC ID register (used to identify the BSP). Stage
  16b-1 adds `send_fixed_ipi(dest, vector)`, which issues an inter-processor interrupt through the ICR
  (Interrupt Command Register: write the destination apic id, then the low half to send, then poll
  the delivery-status bit) — the IPI send path Stage 16b-2's INIT-SIPI-SIPI will reuse. Stage 16b-2a
  adds `send_init_ipi` / `send_startup_ipi` (the INIT and SIPI delivery modes over that same ICR
  path) and `pit_sleep_us` (a polled PIT channel-2 delay) to pace the wake-up sequence. Stage 16d-1
  adds `init_ap`: a woken AP software-enables its *own* Local APIC and starts its periodic timer,
  reusing the BSP's calibrated count (saved in `TIMER_INITIAL_COUNT`) — the LAPIC MMIO address is
  per-core-aliased, so each write targets the running core's own LAPIC.
- `src/smp.rs`: Stage 16b SMP bring-up — waking the application processors. A `global_asm!` trampoline
  climbs a woken AP from 16-bit real mode through 32-bit protected mode to 64-bit long mode
  (`.code16`->`.code32`->`.code64`: temporary GDT + CR0.PE, then CR4.PAE + kernel CR3 + EFER.LME +
  CR0.PG, each transition a raw-byte far jump), writing a progress marker at each rung (1/2/3). All
  absolute addresses are `0x8000 + (label - start)` constants, so the copied blob needs no relocation.
  Stage 16c's `boot_aps` copies the blob to physical 0x8000 (the SIPI vector is its page number) and
  publishes the kernel CR3 + `ap_entry` address once, identity-maps the page
  (`memory::ensure_identity_mapped`, so an AP survives enabling paging), then wakes each discovered AP
  **serially** — write its own heap stack into a slot, clear the marker, send INIT-SIPI-SIPI (via the
  `apic` helpers), poll until that AP reports online (the barrier that makes reusing the one shared
  trampoline page safe), repeat. `ap_stage()` exposes the *lowest* rung any AP reached (so one straggler
  shows even when the rest succeed). Stage 16b-3: the long-mode tail loads a per-AP heap-allocated stack
  and jumps to the Rust `ap_entry` (its address published in a parameter slot); each AP marks its own
  per-CPU block online (`percpu::this_cpu().mark_online`) and bumps an `AP_ONLINE` atomic
  (`aps_online()`), then parks — real kernel Rust on a second core. The trampoline sets `EFER.NXE` (not
  just `LME`) so walking the kernel's NX page-table entries does not reserved-bit-fault. Stage 16d-1:
  before parking, each AP now brings its own interrupt path online — `gdt::init_ap` (load the kernel
  GDT, reload CS, null out SS/DS/ES, no TSS), `interrupts::init_idt_ap` (the shared IDT), `apic::init_ap`
  (its own Local APIC timer), then `sti` — so it takes its own timer interrupts (counted per-CPU) instead
  of sitting idle. Stage 16d-2 validated the context-switch primitive on an AP with a single hand-driven
  worker; **Stage 16d-3** built the real per-CPU scheduler on it, and **Stage 16d-4** makes it
  *preemptive*: before parking, each AP spawns `AP_THREADS` (3) kernel threads onto its **own per-CPU run
  queue** (`sched.rs`) and `run_to_completion`s them. The `ap_worker` body now busy-spins (per-CPU `work`)
  for `AP_THREAD_TICKS` (2) of this core's timer ticks and **never yields** — so the only thing that
  switches between the threads is this core's timer (`sched::preempt`), round-robining them with no
  cooperation. `run_to_completion` enables interrupts and idles on `hlt`; the timer drives the rotation,
  and when the workers finish it marks `scheduler_done` and returns. (The 16d-2 single-worker scaffolding
  is gone.)
- `src/percpu.rs`: Stage 16c per-CPU data — one private `PerCpu` block per core (dense cpu index, Local
  APIC id, BSP flag, an `online` flag, and the stack the core runs on), the foundation for "the current
  process / run queue is per-core" that Stage 16d needs. The blocks live in a heap array published
  through two atomics (`AtomicPtr` + `AtomicUsize` — the storage classes an AP is proven to reach, since
  the 0.9 bootloader may leave large `.bss` unmapped). `init(cores)` builds one block per discovered
  core (BSP pre-marked online) and must run before any AP is woken; `this_cpu()` returns the running
  core's block, found by its own `apic::lapic_id()` (one fixed MMIO register that reads a different id on
  each core); `all()`/`count()`/`online_count()` expose the table. An AP records itself online here in
  `smp::ap_entry`; the BSP prints the per-CPU table at boot. Stage 16d-1 adds a per-CPU `timer_ticks`
  counter (each core tallies its *own* LAPIC timer interrupts) and a non-panicking `this_cpu_opt()` for
  handlers that can fire before `init` (the BSP's timer ticks before per-CPU data is built). Stage
  16d-2/16d-3/16d-4 add a `work` counter, a `threads_completed` counter, a `preemptions` counter, and a
  `scheduler_done` flag that the per-CPU run queue (`sched.rs`) updates as its threads run, the timer
  preempts them, and the rotation unwinds back to the bootstrap context.
- `src/acpi.rs`: Stage 16a SMP discovery — parses just enough ACPI to enumerate the machine's CPU
  cores. `discover` scans low memory for the RSDP signature, follows it to the RSDT/XSDT (a table of
  table pointers), finds the MADT (signature "APIC"), and reads its Processor Local APIC entries into a
  list of `CpuCore`s (Local APIC id + a BSP flag set by matching `apic::lapic_id()`). All reads go
  through the physical-memory window; it is pure byte parsing like `elf.rs`/the FAT BPB, bounds-checked
  so a malformed table degrades to a single BSP-only entry. `cpu_count`/`bsp_apic_id`/
  `application_processors` expose the result; the APs it lists are halted until Stage 16b wakes them.
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
  load an ELF from inside the trap handler (which cannot borrow `kernel_main`'s locals). Stage 16b-2b
  adds `ensure_identity_mapped`, which maps a low frame to itself (no-op if already mapped) so an AP's
  trampoline page survives the AP enabling paging. A later refinement stores the physical-memory offset
  globally in `init` (so page-table walks work from early boot) and adds guard-paged kernel stacks:
  `set_page_present`/`page_is_present` (raw-walk the active tables to a leaf PTE and toggle/read its
  PRESENT bit, TLB-flushing locally, serialized by a lock and with interrupts off) and a `GuardedStack`
  type — a page-aligned heap allocation whose lowest page is marked not-present as a guard, restored on
  `Drop` before the memory is freed. `demo_guard_page`/`guard_page_ok` verify it at boot.
- `src/allocator.rs`: the kernel heap — maps a fixed virtual range to frames and
  registers a `#[global_allocator]` (a hand-written fixed-size block allocator
  over a linked-list fallback), so the `alloc` crate's `Box`/`Vec`/`Rc`/`String`
  become usable. `HEAP_SIZE` was grown 100 KiB → 1 MiB to fit the extra guard page
  each kernel thread stack now carries (`memory::GuardedStack`).
- `src/task/`: Stage 5 cooperative multitasking — `mod.rs` (`Task` and the unique
  `TaskId`), `simple_executor.rs` (a naive busy-polling executor, kept for
  reference), `keyboard.rs` (the async keyboard: a lock-free scancode queue filled
  by the IRQ1 handler and drained by a `Stream`-based task that decodes and
  echoes), and `executor.rs` (the waker-driven executor that sleeps on `hlt` when
  no task is ready). Revived in Stage 7 to drive the shell. Stage 16d-5 adds
  `Executor::run_until_empty` (run until tasks finish, then *return*) so an executor
  can be a finite kernel thread, and runs the shell's executor as a scheduled thread
  under `sched` (via `unify.rs`) rather than as a separate top-level owner of the CPU.
- `src/thread/`: Stage 6 preemptive kernel threads — `mod.rs` (`Thread`/`ThreadId`,
  a round-robin `Scheduler`, `spawn`/`yield_now`/`run`, the fabricated initial
  stack frame, and `schedule` called from the timer to preempt) and `switch.rs`
  (the naked `context_switch` that saves callee-saved registers and swaps stacks).
  Dormant during Stage 7 (marked `#[allow(dead_code)]`); the timer still calls
  `schedule`, but it no-ops because preemption is never armed. Since Stage 16d-2
  `context_switch` is re-exported (`pub use switch::context_switch`) and reused by
  the per-CPU run queue (`sched.rs`) to switch kernel threads on the application
  processors — it is CPU-agnostic, so the same routine serves the BSP's (dormant)
  global `Scheduler` here and each AP's per-CPU run queue.
- `src/sched.rs`: Stage 16d-3/16d-4 per-CPU run queue — the per-CPU analog of the Stage 6 global `thread`
  scheduler. One `RunQueue` (a `BTreeMap` of `KThread`s + a ready `VecDeque` + a `current`) per core,
  published like the per-CPU array (a heap `Vec` leaked to a `'static` slice behind an `AtomicPtr` +
  length) and indexed by the running core's dense `cpu_index` (`percpu`). `init(n_cpus)` builds one empty
  queue per core (on the BSP, before any AP is woken); `spawn(entry)` fabricates a stack (mirroring Stage
  6's `prepare_stack`, return address = `thread_trampoline`) and enqueues a `Ready` thread on *this*
  core's queue — on a `memory::GuardedStack` (an unmapped guard page below the usable stack, so an
  overflow faults instead of corrupting the heap); `spawn_with_stack` takes an explicit size for a
  thread with a deep call chain (the shell executor). The shared `switch_to_next` round-robins to the next ready thread (interrupts off around
  the `thread::context_switch`), reached either cooperatively via `yield_now` (the now-`dead_code` API) or
  — Stage 16d-4 — preemptively via `preempt`, which the AP's timer (`interrupts::timer_dispatch`) calls
  each tick (a `try_lock` so a tick during a queue update simply skips). `run_to_completion` registers the
  caller as a bootstrap thread, **enables interrupts**, and idles on `hlt` while the timer rotates the
  workers (it pre-reserves the ready deque so the interrupt-context switch never allocates); when they all
  `Finished` it reaps their stacks and returns, clearing its bootstrap so the queue is left empty (the BSP
  calls it twice — Stage 16d-5). Used by `smp::ap_entry` to schedule kernel threads on each AP, and (Stage
  16d-5) by the BSP, where the async executor runs as one of these threads.
- `src/unify.rs`: Stage 16d-5 unification of the async executor with the per-CPU scheduler. The async
  executor (`task/`) used to `run()` forever owning the BSP; now it runs **as a kernel thread** on the
  BSP's own `sched` run queue, peer to ordinary kernel threads, with the BSP timer preempting between them
  (`interrupts::timer_dispatch`'s ring-0 path calls `sched::preempt`). `demo()` — run in *both* build
  profiles, so `cargo test` covers it — spawns an async-executor thread (a bounded async task) and a plain
  kernel thread on the BSP run queue and lets the timer preempt them to completion, recording `async_work`/
  `kernel_work` (and BSP `preemptions` via `percpu`). `run_shell_threaded` (non-test) runs the interactive
  shell as a scheduled thread alongside a coexisting heartbeat thread, forever — on a 32 KiB stack via
  `sched::spawn_with_stack` (the deep executor+shell+FAT call chain overflows the default 4 KiB; the
  guard-page refinement exposed this as a clean fault). Concurrent printing from
  these BSP threads is safe because the VGA/serial writers lock inside `without_interrupts`.
- `src/shell.rs`: Stage 7-8 interactive shell — an async task that reads decoded
  keystrokes from the keyboard `ScancodeStream`, buffers a line (with Backspace)
  against a current working directory, and on Enter routes it through a `dispatch`
  table of built-in commands (including the Stage 8 file commands, which since Stage
  14b-3 also reach the FAT disk mounted at `/mnt`). Includes a boot `selftest` so the
  shell and file system are verifiable without a keyboard.
- `src/fs.rs`: Stage 8 in-memory file system — a heap-backed tree of `File`/`Dir`
  nodes addressed by `/`-separated paths, exposed as a global `RamFs` behind a
  mutex with `mkdir`/`write`/`read`/`list`/`remove`/`is_dir`. No disk, no
  persistence. Stage 14a factors those six operations into a `FileSystem` trait (the
  VFS seam) that `RamFs` implements; Stage 14b-2b adds the FAT driver as a second
  implementor behind the same interface (and `FsError` grows `Unsupported`/`Io` variants for
  read-only and device errors). Stage 14b-3 makes the global `fs::*` functions route through a
  minimal one-entry mount table (`mount`/`MOUNT_POINT` = `/mnt`): a path under the mount point
  goes to a mounted `Box<dyn FileSystem>` (the FAT volume), everything else to the root `RamFs`.
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
- `src/ata.rs`: Stage 13a/13b block device driver — a minimal ATA (IDE) disk driver in
  PIO mode. `read_sector` reads one raw 512-byte sector from the primary master by
  the polled READ SECTORS (LBA28) protocol: write the LBA/count registers, issue the
  command, poll the status register for BSY-clear + DRQ-set, then read 256 16-bit
  words from the data port. It disables the drive's interrupt (nIEN) since the kernel
  polls and has no ATA IRQ handler. Stage 13b adds `write_sector(drive, lba, buf)`: the
  mirror WRITE SECTORS (0x30) protocol — poll for DRQ, push 256 words out, wait for the
  commit, then CACHE FLUSH (0xE7) for durability. A shared `issue_command` prologue feeds
  both paths, and a `Drive` enum (primary master = boot disk, primary slave = scratch disk)
  names the target so a write never reaches the boot image; writes go to a git-ignored
  `scratch.img` that a host `build.rs` creates and `Cargo.toml` attaches as the primary
  slave. Single-sector. Stage 14b adds the secondary bus: `Drive::SecondaryMaster` (ports
  0x170/0x376) addresses the FAT disk, with the bus `(io_base, ctrl_base)` chosen per drive.
- `src/fat.rs`: Stage 14b/14c FAT16 driver over the ATA block driver (read, plus write since
  14c-1). `Bpb::parse`
  reads a boot sector's BIOS Parameter Block — sector/cluster size, FAT count and size,
  root-entry count, total sectors — validates the `0x55AA` signature and FAT16 cluster
  range, and derives the FAT/root-directory/data region start LBAs; `read_bpb(drive)` does
  the sector-0 read then parses. The disk is the secondary master (`fat.img`), formatted by
  the host's `mkfs.fat` in `build.rs`. Stage 14b-2 adds a mounted `Fat` handle: `Fat::mount`
  parses the BPB, and `read_file(name)` scans the root directory for the 8.3 entry
  (case-insensitive; skipping deleted, long-name, and volume-label entries) then follows the
  file's FAT16 cluster chain, reading each cluster's sectors and truncating to the directory's
  size (`BadChain` guards a corrupt or non-terminating chain). Stage 14b-2b implements the
  `FileSystem` VFS trait for `Fat` (`read`/`list`/`is_dir` over the root directory), with a
  `From<FatError>` mapping onto the shared `FsError`. Stage 14c-1 adds the write path —
  `alloc_cluster`/`write_chain`/`write_file` create or overwrite a root-level file (updating
  every FAT copy via `ata::write_sector`), wired into `FileSystem::write`. Stage 14c-2 adds
  `remove_file` (free the chain, mark the entry `0xE5`), wired into `FileSystem::remove`; a
  shared `find_entry` backs both lookups. Stage 14d-1 adds `mkdir` at the **root level**:
  `make_root_dir` allocates a cluster, `init_dir_cluster` writes the `.`/`..` entries into it
  (a shared `fill_dir_entry` builds every 32-byte directory entry), and it adds an
  `ATTR_DIRECTORY` entry to the root — wired into `FileSystem::mkdir`. Stage 14d-2 adds
  **read-path subdirectory traversal**: a `DirLocation` (the fixed root region *or* a
  subdirectory cluster chain) unifies directory scanning (`dir_sector_lbas` enumerates a
  directory's sectors — the root region, or a chain walked via the FAT — feeding a shared
  `scan_dir`), and `resolve_dir` walks a multi-component path directory by directory, descending
  from `Root` into `Sub(first_cluster)` at each subdirectory. So `read`/`list`/`is_dir` now reach
  files *inside* subdirectories (the shell's `cd`/`ls`/`cat` descend into `/mnt/SUB`, a directory
  `build.rs` seeds on the image as `SUB/NESTED.TXT`; `list` hides the `.`/`..` links). Stage 14d-3
  extends the **file write path** to subdirectories the same way: `find_entry`/`find_dir_slot` walk
  a directory via `dir_sector_lbas` (not the hardcoded root region), and `write_file_in`/
  `remove_file_in` take a `DirLocation`, so `FileSystem::write`/`remove` resolve the parent path
  (`resolve_dir`) and create/overwrite or delete a **file** anywhere in the tree (`write /mnt/SUB/x`
  lands in the subdirectory). Stage 14d-4 does the same for `mkdir`: `make_dir_in(parent, name)`
  takes a `DirLocation`, and `FileSystem::mkdir` resolves the parent path — the crucial detail is
  that the new directory's `..` back-link is set to the parent's first cluster (0 for the root, the
  subdirectory's own first cluster otherwise), so `mkdir /mnt/SUB/CHILD` then
  `write /mnt/SUB/CHILD/DEEP.TXT` works (three-level traversal). Stage 14d-5 grows a subdirectory
  past its first cluster: when `find_dir_slot` finds no free entry, `grow_dir` walks to the chain's
  last cluster, allocates and zeroes a fresh one, and links it on (the fixed-size root still reports
  `DirFull`). Stage 14d-6 adds `rmdir`: `FileSystem::remove` routes a directory to `remove_dir_in`,
  which (via `dir_is_empty`) removes an *empty* directory — freeing its cluster chain and deleting
  its parent entry — and refuses a non-empty one with `NotEmpty`/`FsError::DirNotEmpty` (no recursive
  delete, unlike `RamFs`). This completes the FAT subdirectory work.
- `src/pci.rs`: Stage 17a PCI bus enumeration — the first step of the networking track. `read_config_u32`
  reads a device's configuration space through the legacy access mechanism #1 (write a bus/device/
  function/offset address to `CONFIG_ADDRESS` = `0xCF8`, read the dword at `CONFIG_DATA` = `0xCFC`,
  serialized by a lock). `enumerate` brute-scans all 256 buses (multifunction-aware) into a `Vec<Device>`,
  each `Device` carrying its vendor/device id, class/subclass, prog-IF, and header type; `Device::bar`/
  `mmio_bar` decode a Base Address Register (32- or 64-bit memory BAR) and `interrupt_line` reads the
  assigned IRQ. `find_e1000` locates QEMU's `-device e1000` (vendor `0x8086`, device `0x100E`); boot lists
  every bus-0 function and reports the NIC's MMIO BAR0 + IRQ, the register block Stage 17b maps. Pure
  read-only config-space reads, like the ACPI table walk; needs the heap (returns a `Vec`).
- `src/e1000.rs`: Stage 17b Intel e1000 (82540EM) NIC driver — the second step of the networking track,
  where the kernel starts *driving* the card 17a found. Stage 17b-1: `init` takes the `pci::Device`,
  maps its 128 KiB BAR0 MMIO register block (32 pages) into the kernel address space **uncacheable**
  (`NO_CACHE` — device registers must bypass the cache, like the APIC's), and reads its identity through
  `volatile` register accesses (`read_reg`/`write_reg`): the Device Status register (`REG_STATUS`) and
  the MAC address out of Receive Address entry 0 (`REG_RAL0`/`REG_RAH0`, pre-loaded from the emulated
  EEPROM by QEMU at power-on). Stage 17b-2: `init` also resets and configures the card first —
  `reset()` masks all interrupts (`REG_IMC`), sets the self-clearing `CTRL.RST` bit and polls (bounded,
  via `apic::pit_sleep_us`) until the card clears it, then re-masks and drains `REG_ICR`; `configure()`
  sets `CTRL_SLU` (Set Link Up) + `CTRL_ASDE` and clears the link-reset/PHY-reset/loss-of-signal/VLAN
  bits, then zeroes the 128-entry Multicast Table Array (`REG_MTA`). The MAC is (re-)read from RAL0/RAH0
  after the reset (which reloads them from the EEPROM). The `E1000` handle (MMIO virtual base + the
  6-byte MAC + a `reset_ok` flag) is stored in a global `Mutex<Option<E1000>>` (`device()`/`present()`)
  for later sub-steps; `control()`/`status()` are live register reads, `reset_succeeded()`/
  `link_requested()` (CTRL.SLU)/`link_up()`/`full_duplex()` decode them. The MMIO block is mapped at
  `E1000_VIRT_BASE` (L4 slot 101, one slot above the APIC's). Descriptor rings, TX/RX, and the IRQ come
  in later sub-steps (17b-3+).
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
