//! In-QEMU unit-test harness.
//!
//! A normal Rust test binary links against the standard library's test harness,
//! which needs an OS underneath it. We have no OS — we *are* the OS — so that
//! harness is unavailable. Instead we use the unstable `custom_test_frameworks`
//! feature (enabled in `main.rs`): the compiler collects every `#[test_case]`
//! function into a slice and hands it to our own [`test_runner`], which the
//! kernel entry point invokes via the generated `test_main()`.
//!
//! Reporting a result is also different with no OS: a freestanding kernel has no
//! `exit(2)` to set a process status. So we let QEMU do it, via its
//! `isa-debug-exit` device (wired up in `Cargo.toml`'s `test-args`): writing a
//! value to I/O port `0xf4` makes QEMU terminate with host status
//! `(value << 1) | 1`. We pick two values, and `bootimage` maps the "success"
//! one to a passing `cargo test`.
//!
//! The whole module is `#[cfg(test)]` (see the `mod testing;` declaration in
//! `main.rs`), so none of it is compiled into the real kernel image.

use x86_64::instructions::port::Port;

use crate::{hlt_loop, serial_print, serial_println};

/// Status codes the kernel asks QEMU to exit with. The concrete numbers are
/// arbitrary — they only need to avoid colliding with codes QEMU itself
/// produces. `Cargo.toml` tells `bootimage` which one means success.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u32)]
pub enum QemuExitCode {
    Success = 0x10,
    Failed = 0x11,
}

/// Exit QEMU via the `isa-debug-exit` device.
///
/// Writing `exit_code` to port `0xf4` (the `iobase` we configure for the device)
/// makes QEMU terminate with host status `(exit_code << 1) | 1`: `Success`
/// (`0x10`) → 33 and `Failed` (`0x11`) → 35. `Cargo.toml` sets
/// `test-success-exit-code = 33`, so `bootimage` reports exit-33 as a passing
/// test run and any other code (35, or a timeout) as a failure.
pub fn exit_qemu(exit_code: QemuExitCode) -> ! {
    // SAFETY: `0xf4` is the iobase declared for the `isa-debug-exit` device in
    // the `test-args` we pass QEMU. The device's only effect is to terminate the
    // VM with the written value as its status; no other hardware uses this port.
    unsafe {
        let mut port = Port::new(0xf4);
        port.write(exit_code as u32);
    }
    // The write above terminates QEMU, so control never reaches here. The halt
    // loop exists only to satisfy the `!` return type (and to stop cleanly were
    // the debug-exit device somehow absent).
    hlt_loop();
}

/// Lets [`test_runner`] print a uniform "name ... [ok]" line around each test.
///
/// Implemented for every zero-argument function, so any `fn()` used as a
/// `#[test_case]` automatically prints its own fully-qualified name (via
/// `type_name`) before running and `[ok]` after. If the test panics instead, the
/// `#[cfg(test)]` panic handler in `main.rs` prints `[failed]` and exits QEMU.
pub trait Testable {
    fn run(&self);
}

impl<T: Fn()> Testable for T {
    fn run(&self) {
        serial_print!("{} ...\t", core::any::type_name::<T>());
        self();
        serial_println!("[ok]");
    }
}

/// The custom test runner named by `main.rs`'s `#![test_runner(...)]`.
///
/// The compiler gathers all `#[test_case]` functions into `tests` and generates
/// a `test_main()` that calls this. We run each test, then exit QEMU with
/// `Success` — reaching the end means none of them panicked.
pub fn test_runner(tests: &[&dyn Testable]) {
    serial_println!("Running {} test(s)", tests.len());
    for test in tests {
        test.run();
    }
    exit_qemu(QemuExitCode::Success);
}

// ---------------------------------------------------------------------------
// The tests themselves.
//
// These run from `kernel_main` *after* the heap and file system are up (the
// `test_main()` call sits at the end of boot), so they may allocate and touch
// `fs`. A `#[test_case]` is just a plain `fn()`; `assert!`/`assert_eq!` panic on
// failure, which the test panic handler turns into a `[failed]` + non-zero exit.
// ---------------------------------------------------------------------------

/// The simplest possible test: proves the whole harness (collection, running,
/// serial reporting, QEMU exit) is wired up correctly.
#[test_case]
fn trivial_assertion() {
    assert_eq!(1 + 1, 2);
}

/// A heap allocation round-trips its value — a smoke test for the global
/// allocator from Stage 4c.
#[test_case]
fn heap_box_alloc() {
    let value = alloc::boxed::Box::new(42);
    assert_eq!(*value, 42);
}

/// Growing a `Vec` forces several reallocations through the allocator.
#[test_case]
fn heap_grow_vec() {
    let mut v = alloc::vec::Vec::new();
    for i in 0u64..1000 {
        v.push(i);
    }
    assert_eq!(v.len(), 1000);
    assert_eq!(v[999], 999);
}

/// Write then read a file through the Stage 8 in-memory file system, and confirm
/// removal really drops it. Cleans up after itself so test order does not matter.
#[test_case]
fn fs_write_read_roundtrip() {
    use crate::fs;
    fs::mkdir("/testtmp").unwrap();
    fs::write("/testtmp/a.txt", b"hello aether").unwrap();
    assert_eq!(fs::read("/testtmp/a.txt").unwrap(), b"hello aether".to_vec());
    fs::remove("/testtmp").unwrap();
    assert!(fs::read("/testtmp/a.txt").is_err());
}

/// Stage 9a: the user-mode segments installed in the GDT carry privilege level 3,
/// so the descent to ring 3 (9b) will push the correct CS/SS.
#[test_case]
fn gdt_user_selectors_are_ring3() {
    use x86_64::PrivilegeLevel;
    assert_eq!(crate::gdt::user_code_selector().rpl(), PrivilegeLevel::Ring3);
    assert_eq!(crate::gdt::user_data_selector().rpl(), PrivilegeLevel::Ring3);
}

/// Stage 9b: the kernel actually executed code in ring 3 and came back. The tests
/// run from `boot_continue`, which is reached only after the timer observed the
/// CPU at `CPL == 3` and rewrote its return frame — so by now this must be set.
#[test_case]
fn reached_user_mode() {
    assert!(crate::usermode::reached_ring3());
}

/// Stage 10a: the `int 0x80` syscall path works — dispatch routes to the right
/// implementation and the return value crosses back to the caller. Driven from
/// ring 0 here; ring 3 uses the identical sequence in 10b.
#[test_case]
fn syscall_dispatch_works() {
    use crate::syscall;
    // getpid returns the fixed placeholder pid.
    let pid = unsafe { syscall::invoke(syscall::SYS_GETPID, 0, 0) };
    assert_eq!(pid, 1);
    // write returns the number of bytes it printed.
    let msg = b"[test] int 0x80 round-trip\n";
    let written =
        unsafe { syscall::invoke(syscall::SYS_WRITE, msg.as_ptr() as u64, msg.len() as u64) };
    assert_eq!(written, msg.len() as u64);
    // an unknown syscall number reports the error sentinel.
    let bad = unsafe { syscall::invoke(9999, 0, 0) };
    assert_eq!(bad, u64::MAX);
}

/// Stage 10b: the ring 3 program made real system calls. It runs during boot
/// (before the tests) and calls `write` then `exit` through `int 0x80`, so by the
/// time the tests run the kernel must have seen at least one syscall from ring 3.
#[test_case]
fn ring3_made_a_syscall() {
    assert!(crate::syscall::ring3_syscall_count() >= 1);
}

/// Stage 11a: the kernel can clone its own address space into a second L4, switch
/// CR3 onto the clone, run real kernel work there, and switch back. The round-trip
/// runs during boot (in `kernel_main`, before this harness) via
/// `memory::demo_clone_kernel_space`, which records its success — so by now the
/// flag must be set.
#[test_case]
fn address_space_clone_roundtrip() {
    assert!(crate::memory::address_space_clone_ok());
}

/// Stage 11b: the ELF parser reads the demo program's header correctly. Pure — it
/// needs no page tables, so it exercises `elf.rs` directly on the bytes.
#[test_case]
fn elf_parser_reads_demo_program() {
    use crate::elf::ElfFile;
    use crate::process;
    let bytes = process::demo_elf(b"test message\n");
    let elf = ElfFile::parse(&bytes).expect("demo ELF must parse");
    assert_eq!(elf.entry(), process::USER_LOAD_BASE + 120);
    let segments: alloc::vec::Vec<_> = elf.load_segments().collect();
    assert_eq!(segments.len(), 1);
    assert_eq!(segments[0].vaddr, process::USER_LOAD_BASE);
    assert!(segments[0].is_executable());
}

/// Stage 11b: the loader mapped the demo ELF into a fresh address space and the
/// entry's code reads back correctly. The load runs during boot (in `kernel_main`,
/// before this harness) via `process::demo_load_elf`, which records the outcome.
#[test_case]
fn elf_loaded_into_address_space() {
    assert!(crate::process::elf_load_ok());
}

/// Stage 12a: the loaded ELF program actually executed in ring 3 on its *own*
/// address space — a different CR3 than the kernel's. During boot `process::run`
/// switches to the image's CR3 and enters ring 3; the program's `write`/`exit`
/// syscalls set `usermode::reached_ring3`, and `run` records both L4 frames.
#[test_case]
fn elf_ran_in_its_own_address_space() {
    assert!(crate::usermode::reached_ring3());
    let user_l4 = crate::process::last_user_run_l4();
    let kernel_l4 = crate::process::kernel_l4();
    assert_ne!(user_l4, 0);
    assert_ne!(user_l4, kernel_l4);
}

/// Stage 12b: the cooperative scheduler ran more than one user process. Boot spawns
/// two demo programs before this harness; each exits via the `exit` syscall, which
/// dispatches the next, so by now at least two processes must have exited.
#[test_case]
fn scheduler_ran_multiple_processes() {
    assert!(crate::process::processes_exited() >= 2);
}

/// Stage 12b: the two demo programs interleaved through the cooperative `yield`
/// syscall — each runs several `write`+`yield` rounds before exiting, so by the time
/// this harness runs the scheduler must have handled several yields.
#[test_case]
fn processes_interleaved_via_yield() {
    assert!(crate::process::processes_yielded() >= 4);
}
