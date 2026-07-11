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
>
> **Stage 17a is also done** — PCI bus enumeration (the networking track begins). The chosen NIC is the
> Intel **e1000** (QEMU's `-device e1000`), which uses MMIO registers and TX/RX descriptor rings — the
> "DMA, ring buffers" this stage is about. But first the kernel must *find* the card: it is a PCI device,
> so a new `pci.rs` enumerates the bus. `read_config_u32` reaches a function's 256-byte configuration
> space through the legacy access mechanism #1 (write a bus/device/function/offset to `CONFIG_ADDRESS`
> `0xCF8`, read the dword at `CONFIG_DATA` `0xCFC`); `enumerate` brute-scans all 256 buses
> (multifunction-aware) into a `Vec<Device>` of vendor/device id, class code, and header type;
> `Device::mmio_bar`/`interrupt_line` decode a BAR (32-/64-bit memory) and the assigned IRQ. `find_e1000`
> locates the card (vendor `0x8086`, device `0x100E`). QEMU now attaches `-device e1000,netdev=net0
> -netdev user,id=net0` (SLIRP — no host privileges, works in WSL2). Boot lists the six bus-0 functions
> and reports the e1000 at `00:03.0` with MMIO BAR0 `0xfeb80000`, IRQ 11 — the register block Stage 17b
> maps. Verified by 44 tests (the new `pci_finds_the_e1000_nic`).
>
> **Stage 17b-1 is also done** — reaching the e1000's registers. Like the APIC, the e1000 is an MMIO
> device: it exposes a 128 KiB register block at the physical address in its BAR0, and we drive it by
> reading/writing those registers. A new `e1000.rs` maps that whole block (32 pages) into the kernel
> address space **uncacheable** (`NO_CACHE`) — device registers must bypass the cache, or reads see a
> stale copy and writes never reach the card — and accesses it only through `volatile` reads/writes (so
> the optimizer cannot reorder or elide them). To prove register access works end to end before the
> descriptor-ring work, `init` reads two things that need no setup: the **Device Status** register (link
> up, speed, duplex) and the card's **MAC address**, which QEMU's model has already loaded from its
> emulated EEPROM into Receive Address entry 0 (RAL0/RAH0) by power-on. Boot logs the mapped block, the
> MAC `52:54:00:12:34:56` (AV bit set), and STATUS `0x80080783` (link up, full-duplex). The handle is
> stashed in a global for later sub-steps. Verified by 45 tests (the new `e1000_reads_its_identity`:
> the card is present and reports a MAC that is neither all-zeros nor all-ones).
>
> **Stage 17b-2 is also done** — resetting and configuring the card, the standard opening move of a
> device driver: put the hardware in a known state before using it. `e1000.rs`'s `init` now, after
> mapping the registers, (1) masks every interrupt at the card (writes all-ones to **IMC**, since
> there is no e1000 IRQ handler yet), (2) issues a **global reset** by setting the self-clearing
> **CTRL.RST** bit and polling (bounded, paced by `apic::pit_sleep_us`) until the card clears it,
> then re-masks interrupts and drains **ICR**, (3) applies general config in CTRL — sets **SLU**
> (Set Link Up) + **ASDE** (Auto-Speed Detection), clears the link-reset/PHY-reset/loss-of-signal/
> VLAN bits — and (4) clears the 128-entry **Multicast Table Array** (accept no multicast by
> default). The reset reloads Receive Address entry 0 from the EEPROM, so the MAC is re-read after
> it. Boot logs "reset completed (CTRL 0x00140260, SLU true)" and the surviving MAC. Verified by 46
> tests (the new `e1000_reset_and_configure`: reset completed, the MAC survived, and a live CTRL
> read-back confirms SLU stuck).
>
> **Stage 17b-3 is also done** — the receive (RX) descriptor ring, so the card can receive frames by
> DMA. A NIC moves whole frames into RAM through a **descriptor ring**: a circular array of 16-byte
> descriptors (each pointing at a receive buffer plus a status byte) shared with the card, driven by
> a head/tail register pair — **RDH** (head, advanced by the card as it fills descriptors) and **RDT**
> (tail, advanced by the driver as it recycles them); the card owns `[RDH, RDT)`. `e1000.rs`'s
> `setup_rx` allocates one frame for the ring and one per receive buffer (from the kernel frame
> allocator), points each descriptor at its buffer, programs the ring base/length
> (**RDBAL/RDBAH/RDLEN**) and head/tail (RDH = 0, RDT = last), then enables the receiver in **RCTL**
> (accept broadcast, strip the Ethernet CRC, 2048-byte buffers; unicast to our own MAC is accepted via
> Receive Address 0). The crucial distinction from the MMIO registers: the ring and buffers are
> **normal cacheable RAM** reached through the physical-memory window — x86 DMA is cache-coherent, so
> only the *registers* need the uncacheable mapping — and the buffer addresses handed to the card are
> raw *physical* addresses (DMA speaks physical), exactly what the frame allocator returns. Interrupts
> stay masked (no RX handler yet), so the card fills buffers silently and advances RDH on its own. Boot
> logs "RX ring: 32 descriptors @ phys 0x390000 (RDBA 0x390000, RDLEN 512, RDH 0, RDT 31), receiver
> enabled". Verified by 47 tests (the new `e1000_sets_up_rx_ring`, which reads RDBAL/RDBAH/RDLEN/RDT/
> RCTL back off the card and confirms they match the ring we installed).
>
> **Stage 17b-4 is also done** — the transmit (TX) descriptor ring, so the card can *send* a raw
> Ethernet frame. (The sub-step order was flipped from "consume RX" to "TX first": under QEMU's SLIRP
> nothing arrives until we send, so TX is the natural, externally-independent next step — DD is a
> purely local completion signal.) TX mirrors RX: `e1000.rs`'s `setup_tx` allocates a ring and
> per-descriptor buffers, programs **TDBAL/TDBAH/TDLEN/TDH/TDT**, sets **TIPG** and enables the
> transmitter in **TCTL** (enable + pad-short-packets + collision params). `transmit(frame)` copies
> the frame into the tail descriptor's buffer, sets the descriptor command (**EOP | IFCS | RS** — end
> of packet, let the card append the CRC, report status), advances TDT to ring the doorbell, and polls
> the descriptor's **Descriptor-Done (DD)** bit. The boot demo/test sends a 60-byte broadcast frame and
> confirms DD.
>
> **The bug that took the most digging: PCI bus mastering was never enabled.** `pci.rs` only ever
> *read* config space; a NIC's rings and buffers are all DMA, and QEMU's `pci_dma_*` silently
> reads/writes **zeros** when the device's bus-master bit (PCI command register, offset `0x04`, bit 2)
> is clear — which it is at power-on. The symptom was maddening: the card advanced TDH and set TXQE
> (so the descriptor "processed"), but read a *zeroed* descriptor (no RS/EOP), so it never transmitted
> and never wrote DD. MMIO register access worked throughout because that is Memory-Space Enable, not
> bus mastering. Fixed by adding `pci::write_config_u32` + `Device::enable_bus_mastering` and calling
> it in `e1000::init` before any ring setup. (The RX ring had the same latent defect — its test only
> checks register read-backs, not live DMA.) Verified by 48 tests (the new `e1000_transmits_a_frame`,
> which sends a frame and asserts DD is set and the ring drains).
>
> **Stage 17b-5 is also done** — consuming a *received* frame, via PHY loopback. SLIRP is passive
> (nothing arrives until we speak a protocol, which is Stage 18), so to exercise the RX path now we use
> the card's own **loopback**: a transmitted frame is looped straight back into the receiver. In QEMU
> 8.2 loopback is triggered by the **PHY BMCR loopback bit**, not the RCTL.LBM bits — and the PHY is
> not memory-mapped, so `e1000.rs` gains MDIO access through the **MDIC** register (`phy_read`/
> `phy_write`: opcode + PHY address 1 + register + data, poll the Ready bit) and `set_loopback`
> (read-modify-write the BMCR). `receive(buf)` polls the current RX descriptor's Descriptor-Done bit,
> copies the frame out of its DMA buffer, recycles the descriptor (clear status, move RDT onto it,
> advance a software cursor), and returns the length. `loopback_selftest` enables loopback, sends a
> 60-byte frame to our own MAC (accepted by the receive filter via Receive Address 0), receives it, and
> checks the bytes round-trip. Two QEMU behaviors had to be handled, each found by instrumenting the
> descriptor/registers: (1) `qemu_receive_packet` **drops** a looped frame (rather than queuing it)
> while the receiver is not ready, and QEMU's e1000 link is not ready to receive until it has settled
> **~0.9 s** into boot — so the selftest resends, watching RDH advance as the readiness check, bounded
> so a dead link cannot hang boot; (2) once ready, delivery is synchronous, so RDH moves the instant we
> transmit. Verified by 49 tests (the new `e1000_receives_via_loopback`).
>
> **Stage 17b-6 is also done** — interrupt-driven receive, so the card *tells* the kernel a frame
> arrived instead of the driver polling for it. Three pieces. (1) **Route the card's IRQ**: the e1000
> reports IRQ 11 in its PCI `interrupt_line`, and on QEMU's i440fx a PCI interrupt appears on the
> IO-APIC pin equal to that ISA IRQ, so `apic::route_pci_irq(11, E1000_VECTOR)` programs a redirection
> entry to vector 43 — **level-triggered and active-low**, the PCI convention (unlike the keyboard's
> edge-triggered, active-high ISA line), so `apic.rs`'s `set_redirection` grew a `level` flag. (2)
> **Arm the card**: `enable_rx_interrupt` sets the receive causes (RXT0 | RXDMT0) in the Interrupt Mask
> Set register (IMS), so the card asserts its line on a received frame. (3) **Handle it**: a new IDT
> gate (`interrupts::e1000_interrupt_handler` on `E1000_VECTOR`) calls `e1000::on_interrupt`, which
> **reads ICR first** (that returns and clears the pending causes, de-asserting the level-triggered
> line — without it the interrupt re-fires endlessly) and then drains every ready frame from the RX
> ring, before the LAPIC EOI. Two correctness details: the handler reads ICR through a **lock-free
> cached MMIO base** (`MMIO_BASE`, an `AtomicU64` published by `init`) and drains under a **`try_lock`**
> — an interrupt handler must never block on a lock the interrupted code holds (a single-core
> deadlock), so if the device is momentarily busy it still clears the cause and returns, leaving frames
> in the ring. `interrupt_selftest` proves it without polling: enable loopback, send a frame to our own
> MAC, and wait for the *handler* to have drained it — the transmit runs under `without_interrupts` so
> the IRQ QEMU raises synchronously during the doorbell write stays pending until the device lock is
> dropped (else the handler would deadlock against the transmit still holding it). Boot logs "interrupt
> receive: irqs 1, frames drained 1, last len 60, match = true", and the RX interrupt is left armed as
> the kernel runs on to the shell (a stray broadcast just fires the handler, which drains or drops it —
> no storm). Verified by 50 tests (the new `e1000_receives_via_interrupt`). **This completes the e1000
> driver (Stage 17b): reset/config, RX and TX descriptor rings, transmit, and now interrupt-driven
> receive — the raw-frame send/receive the networking track needed.** Next is Stage 18: a minimal
> network stack (ARP + IPv4 + ICMP, replying to the host's `ping`).
>
> **Stage 18a is also done** — Ethernet framing plus the receive plumbing, the first layer of the
> network stack (`net/`). Networking is built in layers (each wrapping the next like nested
> envelopes); 18a handles the outermost, Ethernet II, and wires the NIC into the stack. `net/ether.rs`
> parses and builds the 14-byte header (dst/src MAC + EtherType, all big-endian "network byte order"),
> and `net/mod.rs` gives the stack a static identity (IP `10.0.2.15`, the SLIRP default lease; the
> card's MAC) and a `poll` that drains frames from the NIC and dispatches each by EtherType (ARP vs
> IPv4 vs other — 18a only classifies and counts; the handlers come in 18b/18c). This sub-step also
> **refactored the e1000 receive path into the standard NAPI-style split**: the RX interrupt handler
> (`e1000::on_interrupt`) no longer drains the ring or allocates in interrupt context — it only reads
> ICR (still mandatory, to clear the level-triggered cause) and *flags* that frames are waiting;
> `e1000::poll_frame` does the actual ring drain from ordinary context, so the handler takes no lock
> and cannot deadlock (which also let the 17b-6 self-test drop its `without_interrupts` dance).
> `net::loopback_selftest` proves the path end to end with the card's PHY loopback (build a frame to
> ourselves, send, and confirm the stack receives and classifies it) — boot logs "stack up: IP
> 10.0.2.15 ..." and "loopback framing test: ... match = true". Verified by 52 tests (the new
> `ethernet_frame_parses_and_builds` — pure parse/build round-trip — and `net_receives_ethernet_frame`
> — the loopback path). Next (18b): ARP, resolving the gateway's MAC (the first live SLIRP exchange).
>
> **Stage 18b is also done** — ARP, and with it the stack's **first live exchange with a real peer**.
> ARP (the Address Resolution Protocol) is the IP-to-MAC lookup: IP addresses are 32-bit but Ethernet
> delivers to 48-bit MACs, so before sending an IP packet a host broadcasts "who has IP X? tell me",
> and X's owner replies (unicast) with its MAC. `net/arp.rs` parses/builds the fixed 28-byte ARP
> packet (IPv4-over-Ethernet), keeps a small **ARP cache** (`BTreeMap<[u8;4], MacAddr>`), and has a
> pure, unit-testable `process` that learns any sender's mapping and returns a reply payload when the
> packet is a request for our IP. Both directions are live: `net::receive` now routes ARP frames to
> `process` and transmits the reply (so other hosts can find us), and `net::arp_resolve(ip)` broadcasts
> a request and pumps `poll` until the reply lands in the cache (bounded, re-broadcasting periodically).
> The headline: at boot, `arp_resolve(10.0.2.2)` asks QEMU's SLIRP gateway for its MAC and gets a real
> answer — boot logs "ARP: 10.0.2.2 is at 52:55:0a:00:02:02" (libslirp's gateway MAC is `52:55` + the IP
> bytes) — proving send, receive, and parse all work end to end over the emulated wire, not just
> loopback. Verified by 55 tests: `arp_packet_parses_and_builds` and `arp_replies_to_request_for_us`
> (pure logic — the reply payload and cache learning) plus the live `arp_resolves_gateway`. Next (18c):
> IPv4 + ICMP echo — pinging the gateway (the headline milestone).
>
> **Stage 18c is also done — and with it Stage 18, the networking track, and the whole roadmap.** IPv4
> + ICMP echo: the kernel can **ping**. `net/ipv4.rs` parses/builds the 20-byte IPv4 header (no options,
> never fragmented) and holds the **Internet checksum** (RFC 1071 — the one's-complement sum that
> protects both the IP header and the ICMP message). `net/icmp.rs` parses/builds ICMP echo messages
> (type 8 request / 0 reply, with identifier + sequence). Both directions are live: `net::receive`
> dispatches IPv4/ICMP frames addressed to us — `handle_icmp` answers an echo *request* with a reply
> (so the kernel is pingable) and records an echo *reply* by id/seq — and `net::ping(ip)` resolves the
> MAC via ARP, sends ICMP-in-IPv4-in-Ethernet, and pumps `poll` until the matching reply returns. Two
> proofs at boot: a deterministic loopback self-test (`ping_loopback_selftest` — send an echo request
> to ourselves, answer it, receive our own reply, exercising *both* directions with no peer) logs "ICMP
> echo over loopback = true", and the headline — `ping(10.0.2.2)` over the emulated wire logs "ping
> 10.0.2.2: reply seq=1" (libslirp reflects echoes aimed at the gateway, so no host ICMP permission is
> needed). Verified by 60 tests: `internet_checksum_known_answer` (the canonical IPv4-header value),
> `ipv4_header_parses_and_builds` and `icmp_echo_parses_and_builds` (build/parse + self-checksum),
> `net_pings_over_loopback` (the bidirectional loopback path), and the live `pings_the_gateway`. **This
> completes the from-scratch stack: Ethernet -> ARP -> IPv4 -> ICMP, sending and receiving real frames
> over the e1000 with interrupt-driven receive.**
>
> **Stage 18d is also done** — the network stack is now a *live service* with a user interface. A
> background **network thread** (`unify::net_thread`) runs `net::poll` forever on the BSP's run queue,
> peer to the shell thread, `hlt`ing between polls (woken by the e1000 receive IRQ or the timer) — so
> the kernel keeps answering ARP requests and incoming pings while the shell sits idle, at no cost when
> the network is quiet. The shell gains three commands: `ifconfig` (our IP/MAC + traffic counters),
> `arp` (the ARP cache), and `ping <a.b.c.d>` (send an echo and report the reply), backed by
> `net::parse_ipv4` and `arp::cache_entries`. The boot self-test drives them headlessly: `ping 10.0.2.2`
> and `ping 10.0.2.3` both get replies over the wire. Verified by 62 tests (`parse_ipv4_accepts_and_rejects`
> and `arp_cache_snapshot_lists_the_gateway`).
>
> **Stage 19a is also done** — UDP, the stack's first *transport* layer (the follow-on that lets
> *applications* talk, not just the control-plane ICMP). `net/udp.rs` parses/builds the 8-byte header and
> computes the **pseudo-header checksum** — the Internet checksum (reusing `ipv4::checksum`) over a 12-byte
> scratch header of {src IP, dst IP, protocol 17, UDP length} prepended to the datagram, a deliberate
> layering shortcut so a receiver can confirm the datagram really was addressed to it (and it applies the
> RFC 768 rule that a computed 0 is transmitted as 0xFFFF). 19a-1 was that pure module; **19a-2** wired it
> live: `net::receive` now dispatches IPv4 by protocol number, routing UDP to `handle_udp`, a tiny **echo
> server** on port 7 (RFC 862) that bounces a datagram straight back to its sender with the ports swapped
> — the UDP analog of answering a ping — while datagrams on other ports are *delivered* (payload
> recorded). `net::udp_send` is the outbound path (resolves the MAC via ARP, builds
> UDP-in-IPv4-in-Ethernet), and `udp_echo_loopback_selftest` proves both directions with no peer (send to
> our own echo port, echo it, receive the echo). The shell's `ifconfig` gained UDP counters. Verified by
> 64 tests (`udp_datagram_parses_and_builds` — pure build/parse + pseudo-header checksum — and
> `net_udp_echoes_over_loopback` — the live loopback round-trip) and the boot self-test.
>
> **Stage 19b is also done** — DNS over UDP: the kernel can **resolve a hostname to an IP**, the first
> real application UDP carries. `net/dns.rs` builds a query and parses a response, introducing two DNS
> wire ideas: **name encoding** (a hostname is length-prefixed labels ended by a zero byte —
> `example.com` → `\x07example\x03com\x00`) and **compression pointers** (a name in a response is often a
> 2-byte pointer, top bits `11`/`0xC0`, giving an offset back into the message; `skip_name` steps over
> names — and a preceding CNAME — to reach the A record). The **transaction id** ties a response to its
> query, since UDP is unordered. 19b-1 was that pure module; **19b-2** wired the live resolver:
> `net::dns_resolve(hostname)` stamps a fresh id, sends the query to `DNS_SERVER` (`10.0.2.3`) via
> `udp_send`, and pumps `poll` until the reply is delivered (reusing the 19a-2 UDP receive path —
> `LAST_UDP_PAYLOAD` / `UDP_DELIVERED`), then parses out the address. The shell gained `nslookup <host>`,
> and the boot self-test resolves a name (non-fatal, since it needs the host's upstream DNS — SLIRP just
> forwards). Live proof: `dns_resolve("example.com")` returns a real address over the wire. Verified by 66
> tests (`dns_query_and_response_parse` — pure, with a hand-crafted response exercising a compression
> pointer and a CNAME — and the lenient live `net_resolves_a_hostname`).
>
> **Stage 20 is also done — DHCP: the kernel *leases* its IPv4 address instead of hardcoding it.** DHCP
> (the Dynamic Host Configuration Protocol) is another application protocol carried in UDP, but where DNS
> runs *after* we have an address, DHCP is how a real host *gets* one at link-up — so it must work before
> the stack has any identity beyond its MAC. That drives its two defining quirks: a client with no address
> sends from `0.0.0.0` to the broadcast `255.255.255.255` (UDP `68 -> 67`), and it sets the **broadcast
> flag** so the server *broadcasts* the reply back (there is no address yet to unicast to). The exchange is
> the four-step **DORA**: DISCOVER (client broadcast: "any server out there?"), OFFER (server: "you can
> have 10.0.2.15; I am 10.0.2.2"), REQUEST (client broadcast: "I formally request 10.0.2.15 from that
> server" — broadcast on purpose, so any *other* server that offered can reclaim its reservation), ACK
> (server: "confirmed; lease N seconds, mask/router/DNS are ..."). The packet reuses the older **BOOTP**
> layout: a fixed 236-byte header (`op`, `xid`, `yiaddr` = "your" address, `chaddr` = our MAC), a 4-byte
> **magic cookie** (`0x63825363`), then a variable list of **options** (`code, length, value` TLVs — the
> message type, requested IP, server id, lease time, subnet mask, router, DNS — terminated by option 255).
>
> **Stage 20a** is the pure message module `net/dhcp.rs` (like 19b-1): `build_discover` / `build_request`
> emit the BOOTP header + cookie + options, and `parse_reply` validates a BOOTREPLY carrying our
> transaction id and the cookie, reads `yiaddr`, and walks the TLV options (all bounds-checked). **Stage
> 20b** wires it live and — the headline — makes our address **dynamic**: `OUR_IP` becomes a runtime
> `CURRENT_IP` (unconfigured `0.0.0.0` until leased), read everywhere through `our_ip()`, so the moment the
> ACK lands the whole stack (ARP replies, ping, UDP) runs on the *leased* address. `net::dhcp_configure`
> runs DORA against SLIRP's built-in DHCP server (broadcasting via a raw `send_dhcp`, since ARP is
> impossible with no address), matching each reply by transaction id; `receive` now also accepts limited
> broadcast (`255.255.255.255`) so the reply reaches `handle_udp`, which routes the client port (68) to a
> dedicated delivery slot. Boot leases the address before the other net self-tests (falling back to the
> static `10.0.2.15` only if DHCP fails, so boot always proceeds), and `ifconfig` shows the lease
> (mask/gateway/DNS/time). Live proof: boot logs "DHCP lease 10.0.2.15 (gw 10.0.2.2, dns 10.0.2.3,
> 86400 s)". Verified by 68 tests (`dhcp_message_builds_and_parses` — pure build/parse of a DISCOVER,
> REQUEST, and a hand-crafted OFFER — and the live `dhcp_leases_an_address`, which asserts the boot-time
> DORA installed SLIRP's deterministic lease).
>
> **Stage 21 (TCP) has begun** — TCP (the Transmission Control Protocol), the stack's first *reliable*
> transport and the big final follow-on. Where UDP is connectionless "send and forget", TCP is a
> connection-oriented, reliable, ordered byte stream: every byte is numbered (a **sequence number**), the
> receiver **acknowledges** the next byte it expects, control **flags** (SYN/ACK/FIN/RST) run the
> connection, and a **window** does flow control. Being large and stateful, it is split into many
> sub-steps (like the SMP track), each `cargo test`-verifiable.
>
> **Stage 21a** is the pure segment layer `net/tcp.rs` (mirroring `net/udp.rs`): `Segment::parse` reads the
> 20-byte header — ports, seq, ack, flags, window — and honors the **data offset** (header length in
> 32-bit words) so it skips any options to return the real payload; `build` emits a no-options header; and
> `checksum` is the same pseudo-header Internet checksum UDP uses but over protocol 6, with no "computed 0
> becomes 0xFFFF" rule (TCP's checksum is mandatory). **Stage 21b** adds the **connection state machine and
> the three-way handshake**. A **TCB** (Transmission Control Block) per connection tracks its `State`
> (`Closed`/`Listen`/`SynSent`/`SynReceived`/`Established`) and the send/receive **sequence bookkeeping**
> (`snd_una`/`snd_nxt`/`iss`, `rcv_nxt`/`irs` — a SYN consumes one sequence number, so the peer
> acknowledges `iss + 1`). `tcp::open_active` starts an active open (SYN_SENT + the SYN to send),
> `open_passive` registers a listener, and `on_segment` drives both halves: a listener answers a SYN with a
> SYN-ACK (SYN_RECEIVED), an active opener answers a SYN-ACK with the final ACK (ESTABLISHED), and the
> listener reaches ESTABLISHED on that ACK. `net::receive` now dispatches IPv4 protocol 6 to `handle_tcp`,
> which transmits any response the state machine emits; `net::tcp_connect` creates the TCB, sends the SYN,
> and pumps `poll` until ESTABLISHED (using our own MAC for a loopback destination, so no ARP). Proved
> deterministically with no peer via PHY loopback: `tcp_handshake_loopback_selftest` listens on a port and
> connects to *ourselves*, so a client TCB and a server TCB complete SYN / SYN-ACK / ACK and **both** reach
> ESTABLISHED — boot logs "TCP handshake over loopback = true (3 segment(s) parsed)". Verified by 70 tests
> (`tcp_segment_builds_and_parses` — pure build/parse incl. an options-skip and bad-offset rejection — and
> the live `tcp_completes_handshake_over_loopback`).
>
> **Stage 21c** adds **data transfer** — the ESTABLISHED connection becomes a live, reliable, ordered byte
> stream. The TCB grows a **receive buffer**, and `step`'s formerly-stubbed `Established` arm becomes
> `on_established`: it advances `snd_una` over any of our sent bytes the peer now ACKs, accepts **in-order**
> payload (`seq == rcv_nxt`) into the receive buffer while advancing `rcv_nxt`, and replies with an ACK for
> every data segment (a duplicate/out-of-order segment is re-ACKed, not buffered — a gap waits for the
> peer's retransmit in 21e; out-of-order reassembly is deliberately skipped). `tcp::send_data` builds a
> `PSH|ACK` data segment from the connection's send state and advances `snd_nxt`; `net::tcp_send` frames and
> transmits it, and the peer's ACK is processed by the normal `poll`/`handle_tcp` receive path. Proved
> deterministically with no peer via PHY loopback: `tcp_data_loopback_selftest` establishes a loopback
> connection, sends a payload from the client, and confirms the server buffered exactly those bytes **in
> order** and the client saw them **acknowledged** — boot logs "TCP data transfer over loopback = true" (25
> bytes received, acknowledged = true). Verified by 71 tests (`tcp_transfers_data_over_loopback`).
>
> **Stage 21d** adds **connection teardown** — the FIN handshake. TCP is full-duplex, so each direction of
> the stream closes independently; closing is a **four-way** exchange (FIN + ACK each way). The `State`
> enum grows the RFC 793 teardown states — `FinWait1`/`FinWait2`/`TimeWait` (active closer),
> `CloseWait`/`LastAck` (passive closer), and `Closing` (simultaneous close) — with **no new TCB fields**:
> a FIN, like a SYN, consumes one sequence number, so the existing `snd_nxt`/`snd_una`/`rcv_nxt` bookkeeping
> tracks it (our FIN is acknowledged when `snd_una` catches up to `snd_nxt`). `on_established` now also
> accepts a peer's FIN (advancing `rcv_nxt`, moving to CLOSE_WAIT); `step` gains an arm per teardown state;
> and `tcp::close` sends our FIN (ESTABLISHED -> FIN_WAIT_1 active, or CLOSE_WAIT -> LAST_ACK passive).
> `net::tcp_close` frames and transmits it, and the peer's ACK/FIN are handled by the normal `poll` path.
> Proved deterministically with no peer via PHY loopback: `tcp_teardown_loopback_selftest` establishes a
> loopback connection, then actively closes one end and passively closes the other, walking both TCBs
> through the full handshake until the active closer is in **TIME_WAIT** and the passive closer is
> **CLOSED** — boot logs "TCP teardown over loopback = true (client TimeWait, server Closed)". Verified by
> 72 tests (`tcp_tears_down_over_loopback`).
>
> **Stage 21e** adds **retransmission timers** — the last piece that makes the transport truly *reliable*,
> completing the TCP track. Every outbound segment carrying sequence space is kept on a per-connection
> **retransmit queue** (`Unacked { end_seq, deadline, tries, segment }`); `send_data` and `close` enqueue
> theirs, and `process_ack` drops each once `snd_una` reaches its `end_seq`. Time is the global 100 Hz tick
> counter (`interrupts::timer_ticks`, read directly by `tcp.rs`), so no clock is threaded through the send
> paths. `tcp::on_tick` — serviced once per `net::poll` — resends the oldest unacknowledged segment whose
> deadline has passed (exponential backoff, aborting the connection after `MAX_RETRIES`) and expires a
> TIME_WAIT connection to CLOSED once its 2*MSL linger elapses. Proved deterministically via PHY loopback
> with **fault injection**: `tcp_retransmit_loopback_selftest` arms a "drop the next TCP frame" hook
> (`DROP_NEXT_TCP_TX`), sends a payload whose data segment is silently dropped, and confirms the timer
> resends it (so the transfer still completes in order and acknowledged), then tears the connection down
> and confirms the active closer's TIME_WAIT expires to CLOSED — boot logs "TCP retransmission over
> loopback = true (1 resend)". Verified by 73 tests (`tcp_recovers_from_loss_over_loopback`). **This
> completes Stage 21 (TCP) and the networking track: a from-scratch, reliable, connection-oriented
> transport — segment layer, three-way handshake, in-order data transfer with acknowledgements, the FIN
> teardown handshake, and retransmission timers — over the hand-written Ethernet/ARP/IPv4/UDP stack and the
> interrupt-driven e1000.**
>
> **Stage 22 (TCP refinements) has begun** — the follow-on that turns the deliberately-simplified Stage 21
> transport into something closer to a production TCP. It refines two mechanisms Stage 21 stubbed out:
> **out-of-order reassembly** (22a) and the **sliding window** for flow/congestion control (22b onward).
>
> **Stage 22a is done — out-of-order reassembly.** Stage 21c accepted stream data *in order only*
> (`seq == rcv_nxt`), dropping any segment that arrived ahead of the next expected byte and waiting for the
> peer to retransmit it. Stage 22a instead **buffers** an ahead-of-sequence segment in a per-connection
> **reassembly queue** (`Tcb::ooo`) and splices it into the stream once the gap fills. `on_established` now
> routes every data segment through `accept_segment_data`, which handles the three cases against `rcv_nxt`:
> entirely-old (a duplicate — re-ACK only), in-order (append the new tail, advance `rcv_nxt`, then
> `drain_ooo` splices in any now-contiguous buffered segment), and ahead-of-`rcv_nxt` (a gap — `buffer_ooo`
> holds it, bounded by `MAX_OOO_SEGMENTS`, and the segment is dup-ACKed, the classic fast-retransmit
> trigger). Overlaps are handled by appending only the bytes beyond the current `rcv_nxt`. Proved
> deterministically with no peer via PHY loopback and a **reorder** fault-injection hook
> (`REORDER_NEXT_TCP_TX`, mirroring 21e's `DROP_NEXT_TCP_TX`): `tcp_reassembly_loopback_selftest` sends a
> payload as two segments whose wire order is reversed, and confirms the receiver reassembles all bytes in
> order and acknowledges both — boot logs "TCP reassembly over loopback = true (1 out-of-order buffered)".
> Verified by 74 tests (the new `tcp_reassembles_out_of_order`).
>
> **Stage 22b is done — the receiver's sliding window (flow control).** Stage 21 advertised a *fixed*
> window (`DEFAULT_WINDOW = 64240`) on every segment while `rx` grew without bound — the window was a lie.
> Stage 22b makes it honest: the receive buffer is capped at `RCV_WINDOW_MAX`, and the window advertised on
> each segment is `recv_window` = the **free space left** (max minus the unread bytes buffered), so it
> *shrinks* as data piles up unread, hits **zero** when the buffer is full, and *reopens* when the
> application reads. The receive path now **enforces** it: `accept_segment_data` accepts an in-order segment
> only if it fits the free window, otherwise dropping it (and advertising the smaller window), which keeps
> `rx` bounded so the advertised number is truthful. Two application-facing APIs appear: `tcp::read` (drain
> consumed bytes from the buffer, reopening the window — the destructive counterpart to the inspect-only
> `received_data`) and `tcp::receive_window` (observe the current window). Proved deterministically with no
> peer via PHY loopback: `tcp_flow_control_loopback_selftest` fills the window to zero, confirms a further
> segment is **refused** (dropped, `rcv_nxt` unmoved), `read`s to reopen the window by exactly that many
> bytes, and confirms the refused data is finally **admitted** once there is room (redelivered by the Stage
> 21e retransmission timer) — boot logs "TCP flow control over loopback = true". Verified by 75 tests (the
> new `tcp_enforces_receive_window`). Simplifications kept for teaching clarity: the out-of-order
> reassembly queue is not charged against the window, and reopening relies on the sender's retransmission
> (Stage 22c's probe) rather than a proactive window-update ACK.
>
> **Stage 22c is done — the sender's sliding window (the other half of flow control).** Stage 22b made the
> *receiver* advertise an honest window; 22c makes the *sender* obey it. The `Tcb` grows `snd_wnd` (the
> peer's most recently advertised window, learned from the `window` field of every acceptable segment) and
> `snd_buf` (a **send buffer** of queued-but-unsent bytes). The send path splits in two: `queue_send`
> appends application bytes to `snd_buf`, and `flush` transmits as many as the window admits —
> `usable = snd_wnd - inflight` — in `MSS`-sized segments, leaving the rest buffered. `flush` runs right
> after a `queue_send` and once per `net::poll`, so a window that reopens (an ACK advancing the peer's
> window, or the peer reading its buffer) is promptly used. The hard part is the **zero-window deadlock**:
> if the peer advertises zero and we have nothing in flight to prompt a fresh ACK, we would stall forever
> (this minimal stack sends no proactive window update). The fix is the classic **zero-window probe** — a
> one-byte segment sent past `snd_una` that the peer drops-and-re-ACKs until its window reopens, when it
> accepts the byte and its ACK carries the new window; the probe rides the ordinary retransmit queue, so
> the **Stage 21e timer resends it — doubling as the persist timer**, no new clock. Proved deterministically
> with no peer via PHY loopback: `tcp_sender_window_loopback_selftest` hands the sender more than the peer's
> window, confirms it caps in-flight data at the window and buffers the excess (segmented into ≥2 MSS
> pieces), then drains the receiver so the buffered remainder flows out — via the probe — until all bytes
> arrive **in order** and acknowledged — boot logs "TCP sender window over loopback = true (... delivered
> 8704/8704 in order)" (the window was two MSS when 22c landed; Stage 22d-3 later enlarged it to eight).
> Verified by 76 tests (the new `tcp_sender_obeys_peer_window`). This completes the
> bidirectional sliding window (receiver advertises, sender obeys, zero-window probe). (**Congestion
> control** — slow start / congestion avoidance / fast retransmit, pacing the sender to the *network*
> rather than the peer — is the remaining refinement, Stage 22d.)
>
> **Stage 22d has begun — congestion control.** Flow control (Stage 22c) paces the sender to the *receiver*;
> congestion control paces it to the *network*, so a fast receiver behind a congested link cannot be used to
> flood that link (the 1986 "congestion collapse" this mechanism was invented to prevent). It is split into
> sub-steps, each `cargo test`-verifiable: **22d-1** slow start + the growth machinery; **22d-2** the loss
> response (RTO → multiplicative decrease, then congestion avoidance); **22d-3** fast retransmit / fast
> recovery on three duplicate ACKs.
>
> **Stage 22d-1 is done — slow start (the congestion-window machinery).** The `Tcb` grows two fields: `cwnd`
> (the **congestion window** — the sender's estimate of how much data the *network* can absorb) and
> `ssthresh` (the **slow-start threshold** dividing the two growth modes). The sender now paces to
> `min(snd_wnd, cwnd)` — `flush` takes the smaller of the peer's advertised window and `cwnd` — so whichever
> of the receiver or the network is the tighter bottleneck governs. `cwnd` is not advertised; the sender
> infers it: it starts at one **MSS** (`INIT_CWND`, deliberately small so the ramp is visible and so `cwnd`,
> not the loopback receive window, is the binding limit early) and `grow_cwnd` — called on every ACK
> that confirms new data (`process_ack`) — adds one MSS per ACK while `cwnd < ssthresh` (**slow start**:
> `cwnd` doubles every RTT, exponential, since a full window yields `cwnd/MSS` ACKs per round trip), or
> `MSS*MSS/cwnd` per ACK once `cwnd >= ssthresh` (**congestion avoidance**: ~one MSS per RTT, linear). A pure
> duplicate/window-update ACK (`acked == 0`) does not grow it. `ssthresh` starts arbitrarily high
> (`INIT_SSTHRESH = u32::MAX`, RFC 5681), so a fresh connection stays in slow start — the congestion-avoidance
> branch stays unreachable until a loss lowers `ssthresh` (Stage 22d-2). Proved deterministically with no peer
> via PHY loopback: `tcp_congestion_control_loopback_selftest` establishes a connection (initial `cwnd` = one
> MSS), streams 8 KiB while draining the receiver so ACKs flow, and confirms `cwnd` climbs well above its
> initial value (boot logs "cwnd 1024 -> 9216") with every byte still in order. Because `INIT_CWND` (one MSS)
> is smaller than the loopback receive window, the sender's *first* burst is a single segment (not a full
> window), so `cwnd` genuinely binds — the Stage 22c `tcp_sender_window_loopback_selftest` was updated to count its
> segmentation over the whole transfer (slow start spreads the pieces over several round trips) rather than
> the first instant. Verified by 77 tests (the new `tcp_grows_congestion_window`).
>
> **Stage 22d-2 is done — the congestion backoff on loss (the multiplicative-decrease half of AIMD).** Slow
> start (22d-1) only *opens* `cwnd`; 22d-2 adds the *close*. The strongest congestion signal is a
> **retransmission timeout** — a segment lost outright — so when `on_tick` (the Stage 21e retransmit timer)
> resends a segment, it now also calls `on_rto`, which per RFC 5681 §3.1 lowers `ssthresh` to
> `max(flight / 2, 2*MSS)` (**multiplicative decrease** — retreat to half of what was in flight, never below
> two segments) and collapses `cwnd` all the way back to one MSS, re-entering slow start. `cwnd` then ramps up
> exponentially again until it reaches the now-lowered `ssthresh`, where `grow_cwnd` switches to congestion
> avoidance — so the previously-unreachable congestion-avoidance branch is finally exercised, and a lossy path
> converges on the capacity it can sustain instead of hammering it. Proved deterministically with no peer via
> PHY loopback: `tcp_congestion_backoff_loopback_selftest` first streams a batch and drains it so `cwnd` grows
> well above one MSS (with `ssthresh` still at its initial near-infinity), then arms the Stage 21e
> `DROP_NEXT_TCP_TX` loss hook so the next data segment is dropped; when the RTO fires it recovers the segment
> *and* backs the sender off — boot logs "cwnd 7168 -> 1059 (min after loss), ssthresh 4294967295 -> 2048",
> and every byte still arrives in order. Verified by 78 tests (the new `tcp_backs_off_on_loss`). (Remaining:
> Stage 22d-3, fast retransmit / fast recovery on three duplicate ACKs — recover from a single loss without
> waiting for the full RTO.)
>
> **Stage 22d-3 is done — fast retransmit + fast recovery, completing Stage 22d (congestion control), Stage 22
> (TCP refinements), the networking track, and the whole roadmap.** Stage 22d-2 recovered a loss only via the
> RTO: wait a whole timeout, then collapse `cwnd` to one MSS. Fast retransmit recovers **sooner** and
> **gentler**. When a segment is lost but later ones arrive, the receiver re-ACKs the same byte (a **duplicate
> ACK**) for each; `process_ack` now counts consecutive dup ACKs (an ACK advancing nothing, no payload, data
> still outstanding), and the **third** (`DUP_ACK_THRESHOLD`) fires a **fast retransmit** — `on_established`
> resends the missing segment (the head of the retransmit queue) immediately, before the RTO. That also enters
> **fast recovery**: `ssthresh` halves (like an RTO) but `cwnd` is set to `ssthresh + 3*MSS` (the three dup
> ACKs mean three segments already left the network) rather than collapsing to one MSS — each further dup ACK
> inflates `cwnd` by an MSS, and the first new ACK deflates it back to `ssthresh` and exits recovery. Because
> three dup ACKs need one lost segment plus three later ones in flight at once — four segments — the receive
> window `RCV_WINDOW_MAX` was enlarged from two to eight MSS (2048 → 8192); every self-test uses the dynamic
> window value, so this only enlarges the numbers they exercise (the Stage 22b/22c logs now show an 8192-byte
> window). Proved deterministically with no peer via PHY loopback: `tcp_fast_retransmit_loopback_selftest`
> grows `cwnd` past four MSS, then bursts four MSS-sized segments with the first dropped (the Stage 21e
> `DROP_NEXT_TCP_TX` hook); the three that arrive trigger three dup ACKs and a fast retransmit — boot logs
> "fast-retransmits 1, rto-resends 0, cwnd-min-after-loss 2048", confirming recovery beat the RTO timer
> (`rto-resends 0`), `cwnd` only halved to two MSS (not one, so it was fast recovery), and all bytes arrived in
> order. Verified by 79 tests (the new `tcp_fast_retransmits_on_dup_acks`). **This completes the from-scratch
> TCP: reliable ordered delivery, the full close handshake, retransmission, out-of-order reassembly, the
> bidirectional sliding window, and congestion control (slow start, congestion avoidance, AIMD backoff, and
> fast retransmit / fast recovery) — over the hand-written Ethernet/ARP/IPv4/UDP stack and the
> interrupt-driven e1000.**

| Stage | Track | What to build | OS concepts |
|-------|-------|---------------|-------------|
| **13** | Persistence | Block device driver: ATA PIO read/write of raw sectors from a QEMU disk image | device I/O, polling |
| **14** | Persistence | On-disk file system (FAT, read then write); factor a VFS trait so `RamFs` and the disk FS coexist | real FS layout, VFS |
| **15** | Hardware | Replace the 8259 PIC with the Local APIC + IO-APIC; use the Local APIC timer instead of the PIT | modern interrupt delivery; prereq for SMP |
| **16** | Hardware | SMP: bring up the other cores via INIT-SIPI-SIPI, per-CPU data, run the scheduler on multiple cores | real concurrency, per-CPU state |
| **17** | Networking | NIC driver (virtio-net or e1000): send/receive raw Ethernet frames | DMA, ring buffers |
| **18** | Networking | Minimal network stack: ARP + IPv4 + ICMP (ping the gateway) | protocol layering |

## Post-roadmap tracks (Stage 23+)

> **Status: in progress — Stage 23 complete (23a-23d); Stages 24-26 remain.** With the original roadmap complete (stages 0-22d-3), four independent
> follow-on tracks extend it. They are **not** strictly ordered by dependency, but the recommended sequence
> is **23 → 24 → 25 → 26**, chosen by risk and blast radius: do the isolated TCP polish first (it rides the
> momentum of the just-finished TCP work), then the socket capstone that makes the stack usable, then the
> disruptive bootloader migration *before* the layout-sensitive multi-core work (so the SMP code is not
> migrated twice), and leave the most triple-fault-prone multi-core scheduling for last, on a modern base
> with the most test coverage. Each sub-step stays one `cargo run`/`cargo test`-verifiable commit, the same
> discipline as stages 0-22. Cross-track prerequisites: **Stage 26 hard-needs per-CPU TSS (26a)**; Stages 25
> and 26 both reuse the `wait` syscall's block-list / cross-address-space wake pattern; Stage 24's delayed
> ACK must keep *out-of-order* segments immediately ACKed or it breaks Stage 22d-3 fast retransmit.

### Stage 23 (Networking) — TCP refinements toward a production stack

Isolated to `net/tcp.rs` plus a few `net/mod.rs` self-tests; no new subsystems, lowest risk.

> **Stage 23a is done — adaptive RTO (RFC 6298).** The fixed `RTO_TICKS` is replaced by a per-connection
> estimate. Each `Unacked` carries a `sent_at` tick; when an ACK acknowledges a segment that was never
> retransmitted (**Karn's algorithm** — `tries == 0`), `process_ack` samples `now - sent_at` and folds it
> into the estimator. The estimator (`rtt_step`, kept pure so it is unit-testable) is the classic integer
> RFC 6298: `srtt` holds the smoothed RTT scaled by 8 and `rttvar` the variation scaled by 4 (gains 1/8 and
> 1/4), and `rto = clamp(SRTT + 4*RTTVAR, RTO_MIN, RTO_MAX)`. The retransmit timer and backoff now use this
> per-connection `rto` instead of a constant. Because loopback RTTs are below our 10 ms tick granularity (a
> measured RTT of 0 floors the RTO), the *formula* is verified by a deterministic known-answer unit test
> (`rtt_estimator_selftest`: first sample R=40 → RTO 120; a repeat → 100; clamping at the floor/ceiling), and
> a live loopback transfer (`tcp_rtt_estimation_loopback_selftest`) confirms the sender actually sampled an
> RTT (its estimator went valid) and landed on a sane RTO while the data arrived in order. Verified by 81
> tests (the new `tcp_rtt_estimator_known_answer` and `tcp_estimates_rtt_over_loopback`).
>
> **Stage 23b is done — delayed ACKs (RFC 1122).** The receiver no longer ACKs every data segment. In-order
> data is now classified by `accept_segment_data` (returning an `Accept` verdict): plain in-order bytes are
> **delayable**, while an out-of-order segment (its immediate duplicate ACK is the fast-retransmit trigger),
> a gap-filling segment, an old duplicate, or a window-refused segment must be **ACKed now**. `on_established`
> acknowledges delayable data only on **every second** in-order segment (the RFC 1122 rule), otherwise arming
> a per-connection delayed-ACK timer (`DELAYED_ACK_TICKS` = 5, i.e. 50 ms — well under the RTO, so the ACK
> always beats the sender's timeout); `flush_delayed_acks`, serviced once per `net::poll` like the retransmit
> timer but through the ordinary (uncounted) transmit path, sends any ACK whose timer elapsed. A companion
> change adopts **byte-counting slow start** (RFC 3465 "ABC", `grow_cwnd` grows by `min(bytes_acked, 2*MSS)`
> per ACK) so halving the ACK count does not halve the slow-start ramp — the Stage 22d cwnd numbers are
> unchanged. Proved via loopback: `tcp_delayed_ack_loopback_selftest` warms up `cwnd` so the sender bursts
> several segments (which arrive paired), then sends eight in-order segments and confirms they drew only four
> ACKs (coalesced) with the bytes still in order; the Stage 22a/22d-3 tests keep passing, confirming
> out-of-order segments are still ACKed immediately. Verified by 82 tests (the new `tcp_coalesces_acks`).
>
> **Stage 23c is done — Nagle's algorithm (RFC 896).** The sender coalesces a burst of small writes. In
> `flush`, while earlier data is unacknowledged (`inflight != 0`), a sub-MSS chunk is now **held** (left in
> `snd_buf`) instead of sent — it leaves only once a full MSS accumulates or the outstanding data is
> acknowledged; a write is still sent immediately when nothing is outstanding, or when the application set
> **`TCP_NODELAY`** (`set_nodelay`). Proved via loopback: `tcp_nagle_loopback_selftest` issues sixteen
> one-byte writes with Nagle on and confirms they left as just **two** segments (the first byte immediately,
> the rest coalesced behind it) with every byte still in order. Nagle changes small-segment timing, so the
> two prior tests that drive precisely-sized sub-MSS segments — reassembly (22a) and flow control (22b) —
> now set `TCP_NODELAY` (they exercise segment-level mechanics, which real latency-sensitive apps also
> disable Nagle for); every other test is unaffected (their sends are full MSS segments, or single writes
> with nothing outstanding). Verified by 83 tests (the new `tcp_coalesces_small_writes`). This leaves only
> Stage 23d (SACK) in the TCP-refinements track.
>
> **Stage 23d-1 is done — TCP options infrastructure + SACK-permitted negotiation.** SACK (selective
> acknowledgment) is carried in **TCP options**, the variable-length trailer after the fixed 20-byte header,
> which the stack had never used before now (the data offset was always 5). 23d-1 lays that groundwork and
> uses it for the simplest option, the RFC 2018 **SACK-permitted** capability flag; 23d-2 will carry the
> actual SACK blocks. In `net/tcp.rs`: `build_with_options` appends raw option bytes between the header and
> payload, zero-padded to a 4-byte boundary with the **data offset** set to match (`build` is now its
> no-options wrapper, so every existing call site is unchanged); `Segment` grows an `options` slice and a
> `sack_permitted()` accessor over a bounds-checked TLV walker (`find_option`, honoring the one-byte
> END/NOP options). The handshake negotiates per RFC 2018: our SYN (`open_active`) always offers
> SACK-permitted (`kind 4, length 2`), a listener echoes it in its SYN-ACK **only if the incoming SYN
> carried it** (`on_segment`), and each end records the outcome on a new `Tcb::sack_permitted` (the active
> opener from the SYN-ACK, the passive opener from the SYN) — so SACK is enabled only when *both* SYNs
> offered it. Proved deterministically with no peer via PHY loopback: `tcp_sack_negotiation_loopback_selftest`
> completes a loopback handshake and confirms both the client and server TCBs flagged SACK — boot logs
> "TCP SACK negotiation selftest: established = 2, client SACK = true, server SACK = true". Verified by 85
> tests (the pure `tcp_sack_permitted_option_round_trips` — a SYN's option round-trips through build/parse
> with the enlarged data offset and a valid checksum — and the live `tcp_negotiates_sack_over_loopback`).
> (Remaining: Stage 23d-2, SACK blocks in ACKs + the sender retransmitting only the holes.)
>
> **Stage 23d-2a is done — the receiver advertises SACK blocks.** Now that SACK is negotiated (23d-1), the
> receiver reports its **out-of-order data** to the sender. The RFC 2018 **SACK option** (`kind 5`) carries a
> list of `[left, right)` sequence ranges the receiver has buffered *above* the cumulative `ack`, so the
> sender learns exactly which segments already arrived. In `net/tcp.rs`: `sack_blocks` coalesces the
> reassembly queue (`Tcb::ooo`) into maximal contiguous ranges (sorted by distance past `rcv_nxt`, capped at
> four); `build_sack_option` encodes them with the conventional two-NOP alignment; `parse_sack_blocks` /
> `Segment::sack_blocks` decode them (for the sender in 23d-2b and the test); and `build_ack` — the workhorse
> for every ACK — now attaches a SACK option whenever SACK is negotiated and out-of-order data is buffered,
> so a **duplicate ACK for a reordered segment tells the sender precisely what to keep** (counted in
> `SACK_ACKS_SENT`). The sender does not yet act on the blocks — `process_ack` still ignores the option — so
> existing behavior is unchanged (a SACK block is purely additive information). Proved deterministically with
> no peer via PHY loopback, reusing the Stage 22a `REORDER_NEXT_TCP_TX` hook:
> `tcp_sack_blocks_loopback_selftest` sends two segments reordered so the second is buffered out of order, and
> confirms the receiver's dup ACK carried a SACK option (`sack_acks_sent` rose) while the stream still
> reassembled in order — boot logs "TCP SACK blocks loopback selftest: sack-acks 1, reassembled true". Verified
> by 87 tests (the pure `tcp_sack_option_round_trips` — two ranges round-trip through build/parse with a valid
> checksum and the enlarged data offset — and the live `tcp_advertises_sack_blocks`). (Remaining: Stage 23d-2b,
> the sender consuming the blocks to retransmit only the holes.)
>
> **Stage 23d-2b is done — the sender consumes SACK blocks, completing Stage 23d (SACK), Stage 23 (TCP
> refinements), and the TCP-refinements track.** Now the *sender* acts on the SACK blocks the receiver
> advertises (23d-2a): it learns exactly which out-of-order segments the peer already holds and, on a loss,
> retransmits **only the gaps between them** — recovering several losses in one round trip instead of one per
> RTT. In `net/tcp.rs`: each `Unacked` grows a `start_seq` and a `sacked` flag; `mark_sacked` (called from
> `process_ack` on every ACK) sets `sacked` on each queued segment a SACK block fully covers; and `sack_holes`
> selects the segments to fast-retransmit — every not-yet-SACKed segment below the highest SACKed sequence
> (RFC 6675's "data acknowledged beyond it, so it is lost"), falling back to just the oldest segment when there
> is no SACK information (the classic single-hole fast retransmit, unchanged). The fast-retransmit path
> (`on_established`) resends the first hole inline and queues the rest on a new `Tcb::sack_resend`, drained by
> `take_sack_resends` each `net::poll` through the ordinary (uncounted) transmit path — so the RTO counter is
> untouched and the existing fast-retransmit test's "rto-resends 0" still holds. Proved deterministically with
> no peer via PHY loopback and a **bitmask loss hook** (`DROP_TCP_TX_MASK`, dropping *non-adjacent* frames,
> unlike the consecutive Stage 21e `DROP_NEXT_TCP_TX`): `tcp_sack_recovery_loopback_selftest` bursts five
> MSS-sized segments with the **first and third dropped**; the three that arrive draw three SACK-carrying dup
> ACKs, and one fast retransmit recovers **both** holes at once — boot logs "fast-retransmits 1,
> extra-sack-holes 1, rto-resends 0, delivered 11264/11264 in order true". Verified by 88 tests (the new
> `tcp_recovers_multiple_holes_with_sack`). **This completes the from-scratch TCP-refinements track (Stage 23):
> adaptive RTO, delayed ACKs, Nagle, and SACK — atop the Stage 21/22 reliable, congestion-controlled TCP.**

| Sub-step | What to build | OS concepts | Smallest verifiable step |
|----------|---------------|-------------|--------------------------|
| **23a** | RTT measurement + adaptive RTO (RFC 6298) | round-trip estimation, Karn's algorithm | Timestamp each `Unacked`; on a non-retransmitted ACK, sample RTT; maintain `SRTT`/`RTTVAR`; set `RTO = clamp(SRTT + 4*RTTVAR, MIN, MAX)`, replacing the fixed 15-tick RTO. Unit-test the RFC 6298 recurrence on synthetic samples; loopback confirms a transfer completes with a sane RTO. |
| **23b** | Delayed ACKs (RFC 1122) | ACK coalescing, the delayed-ACK timer | ACK in-order data at most every second segment (or after a timeout), but **always** immediately dup-ACK an out-of-order segment. Loopback: two in-order segments draw one ACK; the Stage 22d-3 fast-retransmit test still fires three immediate dup ACKs. |
| **23c** | Nagle's algorithm (RFC 896) | small-segment coalescing, `TCP_NODELAY` | Hold a sub-MSS segment while unacked small data is outstanding; flush on a full MSS or an ACK. Loopback: many small writes coalesce into fewer segments; a `TCP_NODELAY` flag disables it. |
| **23d** | SACK — selective acknowledgment (RFC 2018) | **TCP options** (first use), selective retransmit | Emit options (data offset > 5): negotiate SACK-permitted in the SYN, carry SACK blocks describing the out-of-order queue in ACKs, and have the sender retransmit only the holes. Large — split 23d-1 (options infra + SACK-permitted) / 23d-2 (SACK blocks + sender use). Loopback with several holes. |

### Stage 24 (User space + Networking) — socket system calls

Connects the two finished lines — the network stack and ring 3 — so user programs can do I/O. Touches
`syscall.rs`, `process.rs`, `net/mod.rs`, and a new user program.

| Sub-step | What to build | OS concepts | Smallest verifiable step |
|----------|---------------|-------------|--------------------------|
| **24a** | Per-process handle table + `SYS_SOCKET`/`SYS_CONNECT` (blocking) | file descriptors, blocking syscalls | A socket is a small handle indexing a table binding a process to a TCB. `connect` builds the TCB, sends the SYN, and **blocks the process** until ESTABLISHED — reusing the `wait` block-list pattern; the `net_thread` poll drives the handshake and wakes the process. A ring-3 program connects to a loopback listener. |
| **24b** | `SYS_SEND` / `SYS_RECV` | stream I/O, cross-context wakeup | `send` -> `tcp::queue_send`; `recv` -> `tcp::read`, blocking when empty until data arrives (woken from the receive path in the net thread — a different address space, so the bytes cross in `rax`, exactly as `wait` delivers a child's exit code). A ring-3 program sends and receives over loopback. |
| **24c** | `SYS_LISTEN` / `SYS_ACCEPT` | server sockets, the accept queue | `accept` blocks until a SYN establishes a connection. Requires upgrading the minimal "the listener becomes the connection" model to hold multiple connections. A client and a server ring-3 program exchange data. |
| **24d** | `SYS_CLOSE` + a user "netcat" demo | end-to-end user-space networking | A ring-3 program that opens a socket, transfers data, and closes, wired into the shell. End-to-end proof over loopback / SLIRP. |

### Stage 25 (Hardware) — bootloader 0.9 -> 0.11

The largest blast radius: it changes the boot flow, the boot info, and replaces VGA text with a framebuffer.
Sequenced before Stage 26 so the layout-sensitive SMP code is written once, on the new base.

| Sub-step | What to build | OS concepts | Smallest verifiable step |
|----------|---------------|-------------|--------------------------|
| **25a** | Build-system migration | modern boot protocol | Replace `bootimage` + bootloader 0.9 with bootloader 0.11's build API (a build dependency that assembles the disk image); new `entry_point!`/`BootInfo` and `_start` signature; update `.cargo/config.toml` runner + `Cargo.toml`. Boots and prints "hello" over serial. Highest single-step risk. |
| **25b** | Framebuffer text console | linear framebuffer, glyph rendering | 0.11 boots into a pixel framebuffer, not VGA text mode (`0xb8000`). Write a font-glyph -> pixel renderer backing `print!`/`println!`, replacing the VGA driver. On-screen text (serial already works headless). |
| **25c** | Memory-map migration | boot-info memory regions | 0.11's memory regions and physical-memory offset differ from 0.9. Revisit `BootInfoFrameAllocator`, `physical_memory_offset`, the lower-half assumptions, the AP-stack `.bss` note, and the `AddressSpace` kernel-map clone. Regressions across every prior stage surface here — re-run the whole test suite. |

### Stage 26 (SMP) — multi-core process scheduling

The deepest, most triple-fault-prone track: user processes running across cores. The process scheduler
(`process.rs`) is BSP-only and SMP-unsafe today; APs run only their own per-CPU `sched` kernel threads.

| Sub-step | What to build | OS concepts | Smallest verifiable step |
|----------|---------------|-------------|--------------------------|
| **26a** | Per-CPU TSS + `rsp0` (prerequisite) | per-core ring 0 stacks | Today one TSS is loaded by the BSP; APs load none. Give each core its own TSS with its own `rsp0` (and IST), loaded in `gdt::init_ap` — without it an AP cannot safely take an interrupt from ring 3. An AP takes a ring-0 interrupt on its own `rsp0`. |
| **26b** | Run one user process on an AP | per-core CR3 + TSS | An AP enters ring 3 on its own CR3 and TSS, takes an `int 0x80` syscall (`write`/`exit`), and returns. "AP N ran a ring-3 program." |
| **26c** | Multi-core process run queue | SMP-safe scheduling, per-CPU current | Make the process scheduler SMP-safe: a global ready queue cores pull from (or per-core queues + work stealing), with a per-CPU "current process". Two user processes run on two different cores concurrently. |
| **26d** | Cross-core preemption + load balancing | migration, work stealing | Each core's timer preempts its own running process; idle cores pull/steal ready processes. N processes spread across N cores, all making progress, with observable migrations. |

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
- Optional, non-blocking refinements:
  - **Guard-paged kernel thread stacks — done.** Each `sched` thread's stack now
    carries an unmapped guard page just below its usable region (`memory::GuardedStack`),
    so a stack overflow raises a page fault on the guard page instead of silently
    corrupting adjacent heap data. Carved out of the heap (the frame allocator cannot
    reclaim, so a dedicated stack area would leak frames) by toggling only the PRESENT
    bit; the heap was grown 100 KiB → 1 MiB to fit the extra page per stack. This
    immediately exposed — and fixed — a latent bug: the Stage 16d-5 threaded shell was
    overflowing its 4 KiB stack (it now runs on a 32 KiB stack via `spawn_with_stack`).
  - **FAT subdirectories — done.** Stage 14d-1 creates a *root-level* subdirectory
    (`Fat::make_root_dir`: allocate a cluster, initialize it with `.`/`..`, add an
    `ATTR_DIRECTORY` entry to the root). Stage 14d-2 adds **read-path traversal**: a `DirLocation`
    (the fixed root region *or* a subdirectory cluster chain) unifies directory scanning
    (`dir_sector_lbas`/`scan_dir`), and `resolve_dir` walks a multi-component path directory by
    directory, so `read`/`list`/`is_dir` — hence the shell's `cd`/`ls`/`cat` — work *inside* a
    subdirectory (`build.rs` seeds the image with `SUB/NESTED.TXT` as a real target). Stage 14d-3
    extends the **file write path** the same way: `find_entry`/`find_dir_slot` walk any directory's
    sectors (via `dir_sector_lbas`) and `write_file_in`/`remove_file_in` take a `DirLocation`, so
    `write`/`remove` resolve the parent path and create or delete a file inside a subdirectory
    (`write /mnt/SUB/x`). Stage 14d-4 does the same for `mkdir`: `make_dir_in(parent, name)` sets
    the new directory's `..` back-link to the parent's first cluster (0 for the root), so
    `mkdir /mnt/SUB/CHILD` — and then `write /mnt/SUB/CHILD/DEEP.TXT` — works. Stage 14d-5 **grows a
    directory** past its first cluster: when `find_dir_slot` finds no free entry in a subdirectory,
    `grow_dir` appends a fresh, zeroed cluster to the directory's chain and puts the new entry there
    (the fixed-size root still reports `DirFull`), so a subdirectory no longer caps at 14 files.
    Stage 14d-6 adds **`rmdir`**: `FileSystem::remove` routes a directory to `remove_dir_in`, which
    removes an *empty* directory (`dir_is_empty` guards it) — freeing its cluster chain and deleting
    its parent entry — and refuses a non-empty one with `DirNotEmpty` (no recursive `rm -r`, unlike
    `RamFs`). This completes FAT subdirectory support (grow-then-`rmdir`).
  - Still open: upgrade `bootloader` 0.9 → 0.11 (framebuffer, modern boot info). (Unifying the
    async executor with the thread scheduler is already done — Stage 16d-5.)

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
