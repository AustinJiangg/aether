//! Stage 16c: per-CPU data — a private block of state for each core.
//!
//! Once more than one core runs (Stage 16b woke the first AP; 16c wakes them all),
//! the kernel needs state that is *private to each core*: which process this core is
//! running, this core's own scheduler run queue, its idle stack, and so on. Sharing
//! one global would be both a correctness problem (two cores racing) and a meaning
//! problem ("the current process" is a different answer on each core). The standard
//! answer is a **per-CPU area**: one block of state per core, where every core reads
//! and writes *only its own* block.
//!
//! The block here is deliberately tiny for now — just identity and a liveness flag —
//! but it establishes the mechanism Stage 16d builds on. Two questions define it:
//!
//! 1. **Where does a core find its own block?** Each core is asked for its Local APIC
//!    id (`apic::lapic_id()`), and that indexes a small registry. The trick that makes
//!    this work across cores: the LAPIC ID register lives at one fixed MMIO address,
//!    yet each core's read of it returns *that core's* id — so the same code, run on
//!    any core, lands on the right block. (A faster scheme sets the `GS` base per core
//!    so `gs:[..]` reaches the block in one instruction; we can add that later.)
//!
//! 2. **Where does the registry live?** On the **heap**. An AP reaches it through the
//!    kernel CR3 it shares, and the heap is explicitly mapped (Stage 4c) — unlike a
//!    large `static`, whose `.bss` pages the 0.9 bootloader may leave unmapped (the
//!    same reason an AP's stack is heap-allocated; see `smp.rs`). We publish the
//!    heap array's base + length through two atomics — the exact storage classes an
//!    AP is already proven to touch safely in Stage 16b-3.

use core::slice;
use core::sync::atomic::{AtomicBool, AtomicPtr, AtomicU64, AtomicUsize, Ordering};

use alloc::vec::Vec;

use crate::acpi::CpuCore;
use crate::apic;

/// One core's private state. A core reaches its own via [`this_cpu`]. Fields a core
/// updates about itself are atomics, so a core can write its own block — and the BSP
/// can read every block during bring-up — without taking a lock.
pub struct PerCpu {
    /// Dense index assigned at boot: 0 = BSP, then APs in discovery order. A compact
    /// id to index per-CPU arrays by, distinct from the (possibly sparse) APIC id.
    pub cpu_index: usize,
    /// This core's Local APIC id — how [`this_cpu`] matches a core to its block.
    pub apic_id: u8,
    /// True for the bootstrap processor (the one core already running at power-on).
    pub is_bsp: bool,
    /// Set true once the core is executing kernel Rust: the BSP from the start, an AP
    /// the moment it reaches `ap_entry` and calls [`PerCpu::mark_online`].
    online: AtomicBool,
    /// A stack pointer on the stack this core runs on (0 for the BSP, which kept the
    /// bootloader's stack). Recorded by the core itself — concrete proof it reached
    /// and wrote its *own* block, and a distinct value per core.
    stack: AtomicU64,
    /// Count of Local APIC timer interrupts this core has taken (Stage 16d). Each core
    /// counts its *own* timer here — the first autonomous work an AP does once its timer
    /// is enabled. (The BSP also tallies its ticks globally in `interrupts::TIMER_TICKS`.)
    timer_ticks: AtomicU64,
}

impl PerCpu {
    /// Whether this core has reported itself running kernel Rust.
    pub fn is_online(&self) -> bool {
        self.online.load(Ordering::SeqCst)
    }

    /// The stack pointer this core recorded for itself (0 if it never has).
    pub fn stack(&self) -> u64 {
        self.stack.load(Ordering::SeqCst)
    }

    /// How many Local APIC timer interrupts this core has taken.
    pub fn timer_ticks(&self) -> u64 {
        self.timer_ticks.load(Ordering::SeqCst)
    }

    /// Record one Local APIC timer interrupt on this core. Called from the timer handler
    /// ([`crate::interrupts`]) when the tick lands on this core.
    pub fn tick(&self) {
        self.timer_ticks.fetch_add(1, Ordering::SeqCst);
    }

    /// Called by a core, on itself, to record that it is up and on which stack. Writes
    /// the stack first, then flips `online` last, so an observer that sees `online` also
    /// sees the stack.
    pub fn mark_online(&self, stack: u64) {
        self.stack.store(stack, Ordering::SeqCst);
        self.online.store(true, Ordering::SeqCst);
    }
}

// The per-CPU array lives on the heap; we publish its base + length through two
// atomics (the storage an AP is proven to reach — see the module docs). `CPUS_PTR`
// is non-null only after [`init`] completes, so it doubles as the "ready" signal.
static CPUS_PTR: AtomicPtr<PerCpu> = AtomicPtr::new(core::ptr::null_mut());
static CPUS_LEN: AtomicUsize = AtomicUsize::new(0);

/// Build a per-CPU block for every discovered core. Call **once** on the BSP, after
/// the heap and Local APIC are up and ACPI discovery has run, and **before** waking
/// any AP — an AP reads its block the instant it enters `ap_entry`. The BSP's block
/// is marked online immediately (it is already running); each AP marks its own.
pub fn init(cores: &[CpuCore]) {
    let mut blocks: Vec<PerCpu> = Vec::with_capacity(cores.len());
    for (index, core) in cores.iter().enumerate() {
        blocks.push(PerCpu {
            cpu_index: index,
            apic_id: core.apic_id,
            is_bsp: core.is_bsp,
            online: AtomicBool::new(core.is_bsp), // the BSP is already running
            stack: AtomicU64::new(0),
            timer_ticks: AtomicU64::new(0),
        });
    }

    // Leak the array to a 'static slice on the heap (it lives for the rest of the
    // kernel's life) and publish it: length first, then the base pointer last, so a
    // reader that sees a non-null pointer also sees the correct length.
    let leaked: &'static mut [PerCpu] = Vec::leak(blocks);
    CPUS_LEN.store(leaked.len(), Ordering::SeqCst);
    CPUS_PTR.store(leaked.as_mut_ptr(), Ordering::SeqCst);
}

/// Every per-CPU block (BSP + APs), or an empty slice before [`init`] has run.
pub fn all() -> &'static [PerCpu] {
    let ptr = CPUS_PTR.load(Ordering::SeqCst);
    if ptr.is_null() {
        return &[];
    }
    let len = CPUS_LEN.load(Ordering::SeqCst);
    // SAFETY: after `init`, `(ptr, len)` describe the leaked 'static heap slice. It is
    // never freed or moved, and the only writes after `init` are to the atomic fields
    // inside each `PerCpu`, so handing out a shared `&'static [PerCpu]` is sound.
    unsafe { slice::from_raw_parts(ptr, len) }
}

/// This core's own per-CPU block, or `None` if [`init`] has not run yet (only the BSP
/// runs that early) or the running core is one ACPI never listed. The non-panicking
/// form, for interrupt handlers that can fire before `init` — notably the BSP's timer,
/// which starts ticking before per-CPU data is built.
pub fn this_cpu_opt() -> Option<&'static PerCpu> {
    let id = apic::lapic_id();
    all().iter().find(|cpu| cpu.apic_id == id)
}

/// This core's own per-CPU block, found by its Local APIC id. Works on any core — the
/// BSP or a woken AP — because each core's read of the LAPIC id register returns its
/// own id. Panics only if called before [`init`], or on a core ACPI never listed.
pub fn this_cpu() -> &'static PerCpu {
    this_cpu_opt().expect("this_cpu: no per-CPU block for the running core (init not called?)")
}

/// How many cores have a per-CPU block (BSP + APs).
pub fn count() -> usize {
    all().len()
}

/// How many cores are online (running kernel Rust): the BSP plus every AP that has
/// reached `ap_entry` and marked its block.
pub fn online_count() -> usize {
    all().iter().filter(|cpu| cpu.is_online()).count()
}
