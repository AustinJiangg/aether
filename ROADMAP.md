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
> `timer_preempted_a_process`). Keyboard input is off until 15b.
>
> **Stage 15b is also done** — the IO-APIC, which routes external device IRQs to LAPIC vectors.
> Unlike the LAPIC's flat registers, the IO-APIC is accessed *indirectly*: write a register index
> to IOREGSEL, then read/write its value through IOWIN. `apic.rs` maps the IO-APIC's MMIO page
> uncacheable (next to the LAPIC's), then programs the keyboard's redirection entry — IRQ1 to
> vector 33, fixed delivery, edge-triggered, unmasked, delivered to the BSP — so the keyboard works
> again; its EOI now goes to the LAPIC like the timer's. **This completes Stage 15**: the 8259 PIC
> is fully retired (masked), and both the timer and the keyboard arrive through the APIC — clearing
> the last prerequisite for SMP (Stage 16). Verified by 27 tests (including `ioapic_routes_keyboard`,
> which reads the redirection entry back) and an end-to-end check that injects keystrokes via the
> QEMU monitor and sees the shell echo and run a typed command.
>
> **Stage 16a is also done** — discovering the CPUs (the SMP track begins). A new `acpi.rs` parses
> just enough ACPI to enumerate the machine's cores: it scans low memory for the **RSDP**, follows
> it to the **RSDT/XSDT** (a table of pointers), finds the **MADT** (signature "APIC"), and reads
> its Processor Local APIC entries into a list of `CpuCore`s (apic id + a BSP flag) — reading every
> table through the physical-memory window, pure byte parsing like `elf.rs` and the FAT BPB, with
> length/checksum bounds checks so a malformed table degrades to "BSP only". `apic.rs` gains
> `lapic_id()` (read from the LAPIC ID register) so the running core flags its own MADT entry as the
> **BSP**; the rest are **APs**, halted until Stage 16b's INIT-SIPI-SIPI. QEMU now boots with
> `-smp 4` (Cargo.toml run/test args), so boot reports "4 CPU core(s): BSP apic id 0, 3 application
> processor(s) [1, 2, 3]". Verified by 28 tests — the new `acpi_discovers_all_cpus` asserts all four
> are found, the BSP id matches `lapic_id()`, and the other three are APs.
>
> **Stage 16b-1 is also done** — the IPI mechanism (the first of three steps toward waking an AP).
> `apic.rs` gains `send_fixed_ipi(dest, vector)`, which writes the Local APIC's **ICR** (Interrupt
> Command Register: the destination apic id in the high half, then the low half to issue the IPI) and
> polls the delivery-status bit until the send is accepted — the exact send path Stage 16b-2's
> INIT-SIPI-SIPI will use. To prove it end to end with no assembly and no second core, the BSP sends
> a fixed IPI to *itself* on a dedicated vector (0x40); `interrupts.rs`'s `ipi_test_handler` sets a
> flag and EOIs, and `self_ipi_works()` confirms the flag flipped. Verified by 29 tests (the new
> `self_ipi_is_delivered`) and the boot log ("self-IPI delivered ... = true").
>
> **Stage 16b-2a is also done** — waking an AP (first of two trampoline sub-steps). Because the
> AP-bring-up assembly is the most triple-fault-prone code in the kernel, Stage 16b-2 is split: 2a
> proves the *wake-up mechanism* with a minimal real-mode stub, 2b climbs it to long mode. A new
> `smp.rs` holds a tiny `global_asm!` trampoline (16-bit `.code16`): the AP wakes, sets DS=0, writes
> an "alive" marker to a fixed low address, and halts — no mode switches, so it runs with paging off
> and needs no page tables. `boot_one_ap` copies the blob to physical 0x8000 (a free conventional-RAM
> page; the SIPI vector is its page number, 0x08) through the physical-memory window, clears the
> marker, and sends INIT-SIPI-SIPI via new `apic` helpers (`send_init_ipi`/`send_startup_ipi`, reusing
> 16b-1's ICR path) paced by a `pit_sleep_us` PIT delay (10 ms, then 200 us between the two SIPIs).
> The BSP polls the marker: boot logs "AP apic id 1 is alive (executed the trampoline)" — a second
> core ran our code. Verified by 30 tests (the new `woke_an_application_processor`).
>
> **Stage 16b-2b is also done** — climbing the woken AP to 64-bit long mode. The `smp.rs` trampoline
> now grows from the 2a real-mode stub into the full `.code16` → `.code32` → `.code64` ladder: load a
> temporary GDT and set CR0.PE (protected mode), then set CR4.PAE, load the kernel's CR3, set
> EFER.LME, set CR0.PG (long mode), each transition a raw-byte far jump (`0xEA`) to flush CS. It
> writes a progress marker at each rung (1 = real, 2 = protected, 3 = long) so a stall pinpoints the
> failing transition. Two new pieces support it: `memory::ensure_identity_mapped` maps the trampoline
> page to itself (the instruction after CR0.PG is fetched through the page tables, so 0x8000 must map
> to 0x8000), and `boot_one_ap` publishes the kernel CR3 into a parameter slot the trampoline loads
> (so the AP shares the kernel's address space). All absolute addresses are `0x8000 + (label - start)`
> constants — no runtime relocation. Boot logs "AP apic id 1 reached 64-bit long mode (stage 3/3)".
> Verified by 31 tests (the new `ap_reaches_long_mode`).
>
> **Stage 16b-3 is also done** — the woken AP now enters Rust. The trampoline's long-mode tail loads a
> per-AP stack and jumps to `ap_entry` (a Rust `extern "C"` fn whose absolute address the BSP publishes
> into a parameter slot, so no relocation is needed), which bumps an `AP_ONLINE` atomic the BSP polls,
> then parks (`hlt`). Boot logs "AP apic id 1 is online (running ap_entry on its own stack)" and
> continues to the shell. Two bugs surfaced and were fixed using a `-no-reboot -d int` exception trace:
> (1) the AP's stack must come from the **heap**, not a large `static` — the 0.9 bootloader does not map
> the `.bss` pages past the kernel file image, so a static stack page-faulted (not-present) on the first
> push; (2) the trampoline must set **`EFER.NXE`** as well as `EFER.LME` — the kernel's page tables set
> the NX bit, which is a *reserved* bit unless NXE is enabled, so the AP reserved-bit-faulted the moment
> it read an NX page (the `.rodata` jump table inside `atomic_add`). Verified by 32 tests (the new
> `ap_comes_online`).
>
> **Stage 16c is also done** — waking *all* the APs, each with its own per-CPU data. A new `percpu.rs`
> introduces the **per-CPU area**: one private `PerCpu` block per core (cpu index, APIC id, BSP flag,
> an online flag, and the stack it runs on), held in a heap array published through two atomics (the
> storage an AP is proven to reach), and a `this_cpu()` that finds the running core's block by its own
> Local APIC id — the same fixed MMIO register that returns a different id on each core. `smp.rs`'s
> `boot_one_ap` becomes `boot_aps`, which wakes the discovered APs **serially**: the trampoline code,
> kernel CR3, and `ap_entry` address (identical for all) are written once, then each AP gets its own
> heap stack and is sent INIT-SIPI-SIPI; the BSP waits for that AP to report online (the barrier that
> makes reusing the one shared trampoline page safe) before starting the next. Each AP, in `ap_entry`,
> now finds its own `PerCpu` (by LAPIC id) and marks it online with the stack it is running on, then
> parks. `ap_stage()` reports the *lowest* rung any AP reached, so one straggler is visible even when
> the rest succeed. Boot logs a per-CPU table — "cpu0 apic id 0 BSP online", "cpu1 apic id 1 AP online
> (stack 0x…38f0)", … one stack per AP, 0x2000 apart. Verified by 33 tests (the new
> `all_application_processors_online`: all three APs online, four per-CPU blocks, three distinct nonzero
> AP stacks).
>
> **Stage 16d-1 is also done** — each woken AP now runs its *own* Local APIC timer, the first autonomous
> work a non-boot core does. In `ap_entry` an AP brings its interrupt path online: `gdt::init_ap` loads
> the shared kernel GDT and reloads CS to the kernel code selector (the AP still ran on the trampoline's
> temporary GDT, where the kernel selectors are absent) — and crucially reloads SS to the null selector,
> since the trampoline left SS at its data selector, which in the kernel GDT is the *DPL 3* user-data
> descriptor and would #GP on the first `iretq`. It loads no TSS: the kernel's one TSS is already
> `ltr`-loaded by the BSP (its busy bit makes a second `ltr` #GP), and an AP needs no rsp0/IST yet,
> running only ring-0 handlers on the current stack. `interrupts::init_idt_ap` points the AP's IDTR at
> the one shared IDT; `apic::init_ap` software-enables this core's Local APIC and starts its periodic
> timer, reusing the BSP's calibrated count (the bus clock is the same on every core, and the LAPIC MMIO
> address is per-core-aliased, so each write targets the running core's own LAPIC). The AP then `sti`s
> and parks, woken on each tick. `interrupts::timer_dispatch` now branches on `percpu::this_cpu()`: on an
> AP it just bumps that core's per-CPU `timer_ticks` and EOIs (the global tally and the process/thread
> scheduler stay BSP-only and SMP-unsafe for now); a non-panicking `this_cpu_opt` handles the BSP's timer
> firing before `percpu::init`. Boot logs each AP taking ~5 ticks over a 50 ms window; verified by 34
> tests (the new `aps_take_timer_interrupts`: every AP's per-CPU tick count is non-zero).
>
> **Stage 16d-2 is also done** — a kernel thread now runs on an AP via a context switch. Multi-core
> scheduling is the most triple-fault-prone code yet (an async context switch on a core with no
> console), so — following the 16b discipline of tiny sub-steps — 16d-2 validates just the primitive:
> can `thread::context_switch` (the CPU-agnostic save-callee-saved-+-swap-stacks routine, proven on the
> BSP in Stage 6) work *from* an application processor? In `ap_entry`, after the 16d-1 timer setup (but
> before `sti` — a context switch must be atomic w.r.t. the timer), each AP runs one **cooperative**
> worker thread: it fabricates a worker stack (`prepare_worker_stack`, mirroring Stage 6), `context_switch`es
> into `ap_worker_entry` (which bumps this core's per-CPU `work` counter, then switches back), and resumes
> — a full round-trip, exercising *both* halves of the switch off the BSP. The worker stack is freed on
> return (it would otherwise exhaust the 100 KiB heap), and the bootstrap's resume stack pointer reaches
> the worker through a per-CPU slot. Boot logs each AP doing `work 50000, bootstrap resumed = true`;
> verified by 35 tests (the new `aps_run_a_thread_via_context_switch`).
>
> **Stage 16d-3 is also done** — a real per-CPU run queue. Stage 16d-2 validated the context-switch
> primitive on an AP with a single hand-driven worker; 16d-3 builds the actual scheduler on it. A new
> `sched.rs` holds one cooperative round-robin **run queue per core** — the per-CPU analog of Stage 6's
> single global `thread` scheduler: `RunQueue`/`KThread`, `spawn`, `yield_now`, and `run_to_completion`,
> reusing `thread::context_switch` (the proven primitive) and a fabricated initial stack frame mirroring
> Stage 6's. The queues live in a heap `Vec` leaked to a `'static` slice behind an `AtomicPtr` + length
> (the publish scheme `percpu` uses), one per core, indexed by the running core's dense `cpu_index`. In
> `ap_entry` each woken AP now spawns `AP_THREADS` (3) worker threads onto its own queue and
> `run_to_completion`s them: each thread does `AP_THREAD_ROUNDS` (5) rounds of (per-CPU `work` +
> `yield_now`), so the three interleave (A → B → C → A → …) and the core tallies exactly 15 work; when
> all have finished, control returns to the AP's bootstrap context, which marks `scheduler_done` and
> parks. The per-CPU block swaps 16d-2's `bootstrap_slot`/`bootstrap_resumed` scaffolding for a
> `threads_completed` counter and a `scheduler_done` flag. Scheduling is still **cooperative** (a thread
> runs until it yields or returns). Boot logs each AP "3 thread(s) completed, work 15, scheduler done =
> true"; verified by 35 tests (the new `aps_run_threads_round_robin`, which replaces 16d-2's single-worker
> test and asserts each AP completed exactly 3 threads and 15 work and drained cleanly).
>
> **Stage 16d-4 is also done** — the per-CPU run queue is now **preemptive**. Stage 16d-3 rotated a core's
> threads only when they cooperatively `yield`ed; 16d-4 lets that core's *timer* rotate them, so a thread
> is switched out at any instruction without its cooperation. `sched.rs` gains `preempt`, which performs
> the same `switch_to_next` from interrupt context; `interrupts::timer_dispatch` calls it on the AP path
> (after the EOI) each tick. The trick is the one the process scheduler already uses (Stage 12c): the
> timer's naked stub has saved the interrupted thread's full register set in a `TrapFrame` on its stack,
> so `context_switch` need only swap stacks — when the thread is later rescheduled, the stub's epilogue
> restores that `TrapFrame` and `iretq`s back to the exact instruction the tick interrupted. `preempt`
> `try_lock`s the run queue (skipping a tick that lands mid-update, like the BSP's `thread::schedule`),
> and `run_to_completion` now enables interrupts and idles on `hlt` while the timer drives the rotation
> (pre-reserving the ready deque so the interrupt-context switch never allocates). The AP demo workers
> now **busy-spin and never yield** — each runs until its core has taken `AP_THREAD_TICKS` (2) timer
> interrupts — so the only thing that interleaves them is preemption; a per-CPU `preemptions` counter
> proves it. Boot logs each AP "3 thread(s) completed, N preemption(s)" with N > 0; verified by 35 tests
> (the new `aps_preempt_threads`, replacing 16d-3's exact-work test, asserts each AP completed 3 threads,
> took ≥1 preemption, and drained cleanly).
>
> **Stage 16d-5 is also done** — the async executor and the per-CPU scheduler are **unified**, completing
> the 16d series (and the SMP track). Until now the kernel ran two multitasking models that never
> coexisted: the async executor (`task/`), which on the BSP `run()`s forever driving the shell, and the
> preemptive per-CPU run queue (`sched`), which ran only on the APs. 16d-5 makes the executor run **as a
> kernel thread** on the BSP's own per-CPU run queue, peer to ordinary kernel threads, with the BSP timer
> preempting between them (`interrupts::timer_dispatch`'s ring-0 path now calls `sched::preempt` instead
> of the dormant `thread::schedule`). A new `unify.rs` holds: (1) a testable `demo` — run in *both* build
> profiles — that spawns an async-executor thread (running a bounded async task) and a plain kernel thread
> on the BSP run queue and lets the BSP timer preempt them to completion (so `cargo test` covers the
> unification, which otherwise lives only in the non-test shell path); and (2) `run_shell_threaded`
> (non-test), which runs the interactive shell as a scheduled kernel thread alongside a coexisting
> heartbeat thread, forever. `Executor` gains `run_until_empty` (return when its tasks are done, so an
> executor can be a finite thread), and `sched::run_to_completion` now clears its bootstrap on return (so
> the BSP can call it twice — once for the demo, once for the shell). Boot logs "async work N, kernel work
> M, BSP preemptions K", and the real kernel shows heartbeats interleaving with the live shell; verified
> by 36 tests (the new `bsp_unifies_executor_and_threads`) and a headless run. **This completes Stage 16
> (SMP): all cores discovered, woken, and running an interrupt-driven preemptive scheduler over a unified
> task/thread model.**

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
