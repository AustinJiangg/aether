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

/// Stage 14a: `RamFs` implements the `FileSystem` VFS trait, so it can be driven
/// through a trait object — the seam the FAT driver will later slot into. Uses a
/// fresh local `RamFs` (not the global one), so it is independent of other tests.
#[test_case]
fn ramfs_satisfies_vfs_trait() {
    use crate::fs::{FileSystem, RamFs};
    let mut ram = RamFs::new();
    // Dynamic dispatch through the vtable, exactly as a mounted filesystem would be.
    let fs: &mut dyn FileSystem = &mut ram;
    fs.mkdir("/d").unwrap();
    fs.write("/d/f", b"vfs").unwrap();
    assert_eq!(fs.read("/d/f").unwrap(), b"vfs".to_vec());
    assert!(fs.is_dir("/d"));
    assert_eq!(fs.list("/d").unwrap().len(), 1);
    fs.remove("/d/f").unwrap();
    assert!(fs.read("/d/f").is_err());
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

/// Stage 12c-3: the timer *preempted* a running user process. The demo programs busy-
/// spin in ring 3 (no syscall) between writes, long enough that a ~55 ms timer tick
/// lands mid-spin; the scheduler then switches processes without their cooperation. So
/// by the time this harness runs, at least one preemption must have happened — proof of
/// timer-driven scheduling, not just cooperative `yield`.
#[test_case]
fn timer_preempted_a_process() {
    assert!(crate::process::processes_preempted() >= 1);
}

/// Stage 15b: the IO-APIC routes the keyboard's IRQ to its vector. `apic::init`
/// programs the redirection entry at boot; reading it back proves the IO-APIC's
/// indirect IOREGSEL/IOWIN access works and the entry is armed correctly — the right
/// vector, and unmasked (enabled). The actual keypress path is interactive (a headless
/// QEMU cannot type), so this checks that the routing is set up.
#[test_case]
fn ioapic_routes_keyboard() {
    let entry = crate::apic::ioapic_redirection(crate::apic::KEYBOARD_IRQ);
    // Low byte of the redirection entry is the delivery vector.
    assert_eq!((entry & 0xFF) as u8, crate::apic::KEYBOARD_VECTOR);
    // Bit 16 is the mask; it must be clear so the keyboard IRQ is enabled.
    assert_eq!(entry & (1 << 16), 0);
}

/// Stage 12: the `wait` syscall worked — a parent blocked until its child exited and
/// collected the child's exit code. The boot demo spawns a parent that `wait`s and a
/// child that `exit`s with code 42, so by the time this harness runs the parent must
/// have collected exactly that code (delivered in rax when the child exited).
#[test_case]
fn parent_waited_for_child() {
    assert!(crate::process::processes_waited() >= 1);
    assert_eq!(crate::process::last_waited_code(), 42); // == CHILD_EXIT_CODE
}

/// Stage 12d: a user process created another process at runtime via the `spawn` syscall.
/// The boot demo's parent runs in ring 3 and calls `spawn(PROG_CHILD)` to create its own
/// child — the kernel no longer spawns the child directly — then `wait`s for it. So by
/// the time this harness runs, at least one process must have been spawned from ring 3,
/// and the parent must still have collected the runtime-created child's exit code (42),
/// proving the spawned child is a real, waitable process.
#[test_case]
fn process_spawned_via_syscall() {
    assert!(crate::process::processes_spawned() >= 1);
    assert!(crate::process::processes_waited() >= 1);
    assert_eq!(crate::process::last_waited_code(), 42);
}

/// Stage 13a: the ATA PIO driver reads a raw sector from disk. The bootimage QEMU attaches
/// the kernel image as the primary IDE master, so sector 0 is the boot sector, whose final
/// two bytes are the MBR boot signature 0x55 0xAA — a stable value to assert without
/// depending on any particular file-system layout.
#[test_case]
fn ata_reads_boot_sector_signature() {
    let mut sector = alloc::vec![0u8; crate::ata::SECTOR_SIZE];
    crate::ata::read_sector(0, &mut sector).expect("ATA PIO read of sector 0 failed");
    assert_eq!(sector[510], 0x55);
    assert_eq!(sector[511], 0xAA);
}

/// Stage 13b: the ATA PIO driver writes a sector and reads back exactly what it wrote. The
/// write targets the *scratch* disk (the primary IDE slave attached in `test-args`), never
/// the boot image. We use a different LBA than the boot demo's so the two are independent,
/// and a full-sector pattern so a wrong byte anywhere fails the comparison.
#[test_case]
fn ata_write_then_read_roundtrips_on_scratch() {
    use crate::ata::{self, Drive};
    const LBA: u32 = 1;
    let mut out = alloc::vec![0u8; ata::SECTOR_SIZE];
    for (i, b) in out.iter_mut().enumerate() {
        // A non-trivial pattern (not just the index) so a stuck or shifted byte shows up.
        *b = ((i * 31 + 7) & 0xFF) as u8;
    }
    ata::write_sector(Drive::PrimarySlave, LBA, &out).expect("ATA PIO write failed");

    let mut back = alloc::vec![0u8; ata::SECTOR_SIZE];
    ata::read_sector_from(Drive::PrimarySlave, LBA, &mut back).expect("ATA PIO read-back failed");
    assert_eq!(out, back);
}

/// Stage 14b-1: parse the BPB of the host-formatted FAT16 disk (the secondary IDE master,
/// `fat.img`). Asserts the exact geometry mkfs.fat produced for our 5 MiB, 1-sector-cluster
/// image, which also exercises the secondary ATA bus and the region-layout arithmetic.
#[test_case]
fn fat_bpb_parses_known_geometry() {
    use crate::ata::Drive;
    use crate::fat;
    let bpb = fat::read_bpb(Drive::SecondaryMaster).expect("reading/parsing the FAT BPB failed");
    assert_eq!(bpb.bytes_per_sector, 512);
    assert_eq!(bpb.sectors_per_cluster, 1);
    assert_eq!(bpb.reserved_sectors, 1);
    assert_eq!(bpb.num_fats, 2);
    assert_eq!(bpb.root_entry_count, 512);
    assert_eq!(bpb.fat_size_sectors, 40);
    assert_eq!(bpb.total_sectors, 10240);
    // Derived layout: FAT at LBA 1, root dir after both FATs (1 + 2*40 = 81), data after the
    // 32-sector root directory (81 + 32 = 113).
    assert_eq!(bpb.fat_start_sector(), 1);
    assert_eq!(bpb.root_dir_start_sector(), 81);
    assert_eq!(bpb.data_start_sector(), 113);
}

/// Stage 14b-2: read a real file off the FAT16 disk end to end. The host's `build.rs` copies a
/// known HELLO.TXT into the image, so mounting the volume, scanning the root directory for the
/// 8.3 entry, and following its FAT cluster chain must return exactly those bytes — this
/// exercises the directory scan, the case-insensitive name match, and the chain walk together.
#[test_case]
fn fat_reads_known_file() {
    use crate::ata::Drive;
    use crate::fat::{Fat, FatError};
    // Must match FAT_FILE_CONTENT in build.rs.
    const EXPECTED: &[u8] = b"Hello from a real FAT16 disk, read by Aether.\n";

    let volume = Fat::mount(Drive::SecondaryMaster).expect("mounting the FAT volume failed");

    // The known file reads back byte-for-byte.
    let bytes = volume.read_file("HELLO.TXT").expect("reading HELLO.TXT failed");
    assert_eq!(bytes, EXPECTED);

    // 8.3 names match case-insensitively, so a lowercase request finds the same file.
    let lower = volume.read_file("hello.txt").expect("case-insensitive read failed");
    assert_eq!(lower, EXPECTED);

    // A name with no matching entry is reported as NotFound (not a panic or wrong bytes).
    assert_eq!(volume.read_file("NOPE.TXT"), Err(FatError::NotFound));
}

/// Stage 14b-2b: the FAT volume implements the `FileSystem` VFS trait, so it can be driven
/// through a trait object — the same seam `RamFs` slots into (see `ramfs_satisfies_vfs_trait`).
/// Exercises read/list/is_dir over the root and the FatError -> FsError mapping (writing files is
/// covered by `fat_writes_a_file`, and subdirectory traversal by `fat_traverses_subdirectory`).
#[test_case]
fn fat_satisfies_vfs_trait() {
    use crate::ata::Drive;
    use crate::fat::Fat;
    use crate::fs::{FileSystem, FsError};
    // Must match FAT_FILE_CONTENT in build.rs.
    const EXPECTED: &[u8] = b"Hello from a real FAT16 disk, read by Aether.\n";

    let mut volume = Fat::mount(Drive::SecondaryMaster).expect("mounting the FAT volume failed");
    // Dynamic dispatch through the vtable, exactly as a mounted filesystem would be used.
    let fs: &mut dyn FileSystem = &mut volume;

    // Read a root-level file through the trait object.
    assert_eq!(fs.read("/HELLO.TXT").unwrap(), EXPECTED);

    // The root is a directory; a regular file is not.
    assert!(fs.is_dir("/"));
    assert!(!fs.is_dir("/HELLO.TXT"));

    // The known file shows up in the root listing (other files may exist from write tests).
    let entries = fs.list("/").unwrap();
    assert!(entries.iter().any(|(name, is_dir)| name.as_str() == "HELLO.TXT" && !*is_dir));

    // Error mapping: reading the root is IsDir, a missing name is NotFound.
    assert_eq!(fs.read("/"), Err(FsError::IsDir));
    assert_eq!(fs.read("/NOPE.TXT"), Err(FsError::NotFound));

    // Root-level mkdir works (Stage 14d-1); nested mkdir now traverses (Stage 14d-4), so a parent
    // that does not exist resolves to NotFound rather than being rejected as unsupported. (Use a
    // name that is genuinely absent — `/sub` would case-insensitively match the seeded `SUB`.)
    assert_eq!(fs.mkdir("/ABSENT/child"), Err(FsError::NotFound));
}

/// Stage 14d-1: the FAT driver creates a subdirectory in the root. `mkdir` allocates a cluster,
/// writes `.`/`..` into it, and adds an `ATTR_DIRECTORY` entry to the root — so the directory then
/// shows up in the root listing, `is_dir` agrees, and reading it as a file fails. Uses a fixed
/// name and tolerates a directory left by a previous run (removing directories is a later step);
/// a nested directory (subdirectory parent) is still unsupported.
#[test_case]
fn fat_mkdir_creates_a_directory() {
    use crate::ata::Drive;
    use crate::fat::Fat;
    use crate::fs::{FileSystem, FsError};

    let mut volume = Fat::mount(Drive::SecondaryMaster).expect("mounting the FAT volume failed");

    // Create the directory, tolerating one persisted by an earlier run.
    match volume.mkdir("/MKDIRT") {
        Ok(()) | Err(FsError::Exists) => {}
        Err(e) => panic!("root-level mkdir failed: {:?}", e),
    }

    // It appears in the root listing, flagged as a directory...
    let entries = volume.list("/").unwrap();
    assert!(
        entries
            .iter()
            .any(|(name, is_dir)| name.as_str() == "MKDIRT" && *is_dir),
        "created directory not found in the root listing"
    );
    // ...`is_dir` agrees, and reading it as a file reports it is a directory.
    assert!(volume.is_dir("/MKDIRT"));
    assert_eq!(volume.read("/MKDIRT"), Err(FsError::IsDir));

    // Creating it again reports that it already exists.
    assert_eq!(volume.mkdir("/MKDIRT"), Err(FsError::Exists));

    // Nested mkdir now traverses (Stage 14d-4, covered by `fat_mkdir_in_subdirectory`); a parent
    // that does not exist resolves to NotFound.
    assert_eq!(volume.mkdir("/NOSUCHDIR/child"), Err(FsError::NotFound));
}

/// Stage 14d-2: read-path traversal into a subdirectory. `build.rs` seeds the image with
/// `SUB/NESTED.TXT`, so resolving a multi-component path — scanning the root for `SUB`, then
/// following that subdirectory's own cluster chain — lets `read`/`list`/`is_dir` reach the nested
/// file, while a file mid-path and a missing directory report the right errors.
#[test_case]
fn fat_traverses_subdirectory() {
    use crate::ata::Drive;
    use crate::fat::Fat;
    use crate::fs::{FileSystem, FsError};
    // Must match FAT_NESTED_CONTENT in build.rs.
    const NESTED: &[u8] = b"Nested file inside a FAT16 subdirectory.\n";

    let volume = Fat::mount(Drive::SecondaryMaster).expect("mounting the FAT volume failed");
    let fs: &dyn FileSystem = &volume;

    // The seeded subdirectory is a directory, and traversal reads the nested file's bytes.
    assert!(fs.is_dir("/SUB"));
    assert_eq!(fs.read("/SUB/NESTED.TXT").unwrap(), NESTED);

    // Listing the subdirectory shows the nested file and hides the `.`/`..` self/parent links.
    let entries = fs.list("/SUB").unwrap();
    assert!(entries.iter().any(|(n, is_dir)| n.as_str() == "NESTED.TXT" && !*is_dir));
    assert!(!entries.iter().any(|(n, _)| n.as_str() == "." || n.as_str() == ".."));

    // Error paths: a missing name inside the subdirectory and a missing subdirectory are both
    // NotFound; descending into a regular file (`HELLO.TXT`) is NotDir.
    assert_eq!(fs.read("/SUB/NOPE.TXT"), Err(FsError::NotFound));
    assert_eq!(fs.list("/NODIR"), Err(FsError::NotFound));
    assert_eq!(fs.read("/HELLO.TXT/x"), Err(FsError::NotDir));
}

/// Stage 14d-3: write-path traversal — create, overwrite, and remove a **file inside a
/// subdirectory**. The parent path (`/mnt/SUB`) is traversed to the subdirectory's cluster chain,
/// then the file is written/removed there, alongside the seeded `NESTED.TXT`. Self-cleaning, and
/// a write to a nonexistent parent must fail to resolve.
#[test_case]
fn fat_writes_into_subdirectory() {
    use crate::fs;
    use crate::fs::FsError;
    // Must match FAT_NESTED_CONTENT in build.rs — the seeded neighbor that must survive.
    const NESTED: &[u8] = b"Nested file inside a FAT16 subdirectory.\n";

    // Create a file inside the seeded /mnt/SUB and read it back through the traversal path.
    let data = b"written into a FAT subdirectory".to_vec();
    fs::write("/mnt/SUB/INSUB.TXT", &data).expect("writing into a subdirectory failed");
    assert_eq!(fs::read("/mnt/SUB/INSUB.TXT").unwrap(), data);

    // It shows up listing the subdirectory, next to the seeded NESTED.TXT.
    let entries = fs::list("/mnt/SUB").unwrap();
    assert!(entries.iter().any(|(n, is_dir)| n.as_str() == "INSUB.TXT" && !*is_dir));

    // Overwriting in place updates the contents (and frees/reallocates the chain).
    let data2 = b"second, different contents in the subdirectory".to_vec();
    fs::write("/mnt/SUB/INSUB.TXT", &data2).expect("overwriting in a subdirectory failed");
    assert_eq!(fs::read("/mnt/SUB/INSUB.TXT").unwrap(), data2);

    // Remove it: gone, while the seeded neighbor is untouched.
    fs::remove("/mnt/SUB/INSUB.TXT").expect("removing from a subdirectory failed");
    assert_eq!(fs::read("/mnt/SUB/INSUB.TXT"), Err(FsError::NotFound));
    assert_eq!(fs::read("/mnt/SUB/NESTED.TXT").unwrap(), NESTED);

    // Writing under a parent that does not resolve to a directory fails at traversal.
    assert_eq!(fs::write("/mnt/NODIR/x.txt", b"y"), Err(FsError::NotFound));
    assert_eq!(fs::remove("/mnt/NODIR/x.txt"), Err(FsError::NotFound));
}

/// Stage 14d-4: `mkdir` inside a subdirectory. Create a directory in the seeded `/mnt/SUB`, then
/// prove it is real and usable: it traverses as a directory, lists in its parent, and the write
/// path reaches two levels deep (`/mnt/SUB/CHILD/DEEP.TXT`) — which also exercises three-component
/// traversal (SUB -> CHILD -> file). Tolerates a `CHILD` left by a previous run (rmdir is a later
/// step) and cleans up the inner file.
#[test_case]
fn fat_mkdir_in_subdirectory() {
    use crate::fs;
    use crate::fs::FsError;

    match fs::mkdir("/mnt/SUB/CHILD") {
        Ok(()) | Err(FsError::Exists) => {}
        Err(e) => panic!("mkdir inside a subdirectory failed: {:?}", e),
    }
    // It traverses as a directory and shows up in the parent listing, flagged as one.
    assert!(fs::is_dir("/mnt/SUB/CHILD"));
    assert!(!fs::is_dir("/mnt/SUB/NOPE"));
    let entries = fs::list("/mnt/SUB").unwrap();
    assert!(entries.iter().any(|(n, is_dir)| n.as_str() == "CHILD" && *is_dir));

    // The write path reaches into the nested directory: create a file two levels down and read it
    // back, proving the new directory is a genuine, usable subdirectory.
    let data = b"two levels down".to_vec();
    fs::write("/mnt/SUB/CHILD/DEEP.TXT", &data).expect("writing into a nested directory failed");
    assert_eq!(fs::read("/mnt/SUB/CHILD/DEEP.TXT").unwrap(), data);
    let deep = fs::list("/mnt/SUB/CHILD").unwrap();
    assert!(deep.iter().any(|(n, is_dir)| n.as_str() == "DEEP.TXT" && !*is_dir));

    // Clean up the file (leaving CHILD, since rmdir is a later step); re-creating CHILD reports it
    // already exists, and mkdir under an unresolvable parent fails at traversal.
    fs::remove("/mnt/SUB/CHILD/DEEP.TXT").expect("removing the nested file failed");
    assert_eq!(fs::mkdir("/mnt/SUB/CHILD"), Err(FsError::Exists));
    assert_eq!(fs::mkdir("/mnt/NODIR/x"), Err(FsError::NotFound));
}

/// Stage 14d-5: a subdirectory grows past its first cluster. Our test image has one sector per
/// cluster (512 B = 16 directory entries), and a fresh subdirectory already spends two entries on
/// `.`/`..`, so its first cluster holds only 14 files. Creating more than that must append a second
/// cluster to the directory's chain instead of failing with `DirFull`. We create 20 files (forcing
/// the grow), read each back (proving the appended cluster is walked on the read path), and confirm
/// all 20 list. Then we delete them — self-cleaning at the file level; the directory keeps its now
/// two clusters, which real FAT never shrinks, so a later run reuses the freed slots.
#[test_case]
fn fat_grows_a_directory() {
    use crate::fs::{self, FsError};
    use alloc::format;

    const N: usize = 20; // > 14, so the first cluster overflows and the directory must grow

    // A dedicated directory, tolerating one left by a previous run (no rmdir yet).
    match fs::mkdir("/mnt/BIGDIR") {
        Ok(()) | Err(FsError::Exists) => {}
        Err(e) => panic!("mkdir /mnt/BIGDIR failed: {:?}", e),
    }

    // Create N files with distinct contents, forcing the directory past its first cluster.
    for i in 0..N {
        let path = format!("/mnt/BIGDIR/F{}.TXT", i);
        let content = format!("file number {}", i);
        fs::write(&path, content.as_bytes()).expect("writing a file in the growing dir failed");
    }

    // Every file reads back correctly — the entry in the appended cluster is found and its data
    // chain is followed.
    for i in 0..N {
        let path = format!("/mnt/BIGDIR/F{}.TXT", i);
        let expected = format!("file number {}", i);
        assert_eq!(fs::read(&path).unwrap(), expected.as_bytes());
    }

    // All N files list, so the scan crossed the cluster boundary into the appended cluster.
    let entries = fs::list("/mnt/BIGDIR").unwrap();
    for i in 0..N {
        let name = format!("F{}.TXT", i);
        assert!(
            entries.iter().any(|(n, is_dir)| n.as_str() == name && !*is_dir),
            "missing {} after growing the directory",
            name
        );
    }

    // Clean up the files (the directory itself stays, two clusters long).
    for i in 0..N {
        let path = format!("/mnt/BIGDIR/F{}.TXT", i);
        fs::remove(&path).expect("removing a file from the grown dir failed");
    }
    let after = fs::list("/mnt/BIGDIR").unwrap();
    assert!(after.is_empty(), "files remained after cleanup: {:?}", after);
}

/// Stage 14d-6: `rmdir` — the FAT driver removes an *empty* directory. Create a directory, put a
/// file in it and confirm removing it then fails with `DirNotEmpty`, empty it, then remove the
/// directory itself and confirm it is gone (reading, re-removing, and `is_dir` all agree).
/// Self-cleaning: it leaves the disk image as it found it.
#[test_case]
fn fat_removes_a_directory() {
    use crate::fs::{self, FsError};

    // A dedicated directory, tolerating one left by an earlier interrupted run.
    match fs::mkdir("/mnt/RMTEST") {
        Ok(()) | Err(FsError::Exists) => {}
        Err(e) => panic!("mkdir /mnt/RMTEST failed: {:?}", e),
    }
    assert!(fs::is_dir("/mnt/RMTEST"));

    // A non-empty directory cannot be removed: rmdir refuses it with DirNotEmpty.
    fs::write("/mnt/RMTEST/A.TXT", b"content").expect("writing into the dir failed");
    assert_eq!(fs::remove("/mnt/RMTEST"), Err(FsError::DirNotEmpty));

    // Empty it, then the directory itself is removable.
    fs::remove("/mnt/RMTEST/A.TXT").expect("removing the inner file failed");
    fs::remove("/mnt/RMTEST").expect("rmdir on the now-empty directory failed");

    // Gone: it is no longer a directory, and re-removing reports NotFound.
    assert!(!fs::is_dir("/mnt/RMTEST"));
    assert_eq!(fs::remove("/mnt/RMTEST"), Err(FsError::NotFound));
    let root = fs::list("/mnt").unwrap();
    assert!(!root.iter().any(|(n, _)| n.as_str() == "RMTEST"));
}

/// Stage 17a: the PCI bus is enumerated and the e1000 NIC is discovered. QEMU attaches
/// `-device e1000` on the bus (see Cargo.toml test-args), so enumeration must find it by its
/// identity (vendor 0x8086, device 0x100E) and recognize it as an Ethernet network controller.
/// Its BAR0 must decode to a non-zero memory-mapped base — the register block Stage 17b maps.
#[test_case]
fn pci_finds_the_e1000_nic() {
    use crate::pci;

    let nic = pci::find_e1000().expect("e1000 NIC not found on the PCI bus");
    assert_eq!(nic.vendor_id, pci::INTEL_VENDOR_ID);
    assert_eq!(nic.device_id, pci::E1000_DEVICE_ID);
    assert_eq!(nic.class, pci::CLASS_NETWORK);
    assert_eq!(nic.subclass, pci::SUBCLASS_ETHERNET);

    // BAR0 is a memory BAR (bit 0 clear), and QEMU maps it at a real, non-zero address.
    let mmio = nic.mmio_bar(0).expect("e1000 BAR0 should be a memory BAR");
    assert!(mmio != 0, "e1000 BAR0 (MMIO base) should be non-zero");
}

/// Stage 17b-1: the e1000 driver mapped the card's MMIO register block and read its identity over
/// it. `kernel_main` calls `e1000::init` at boot (before this harness), which maps BAR0 uncacheable
/// and reads the MAC out of Receive Address entry 0. So by now the card must be present and report a
/// real MAC — non-zero and not the all-ones "no device" pattern — proving uncacheable MMIO register
/// reads reach the hardware.
#[test_case]
fn e1000_reads_its_identity() {
    use crate::e1000;

    assert!(e1000::present(), "e1000 was not brought up during boot");
    let dev = e1000::device().expect("e1000 device handle missing");

    // A valid MAC is neither all-zeros nor all-ones (an unmapped register reads back 0xFF..).
    let mac = dev.mac();
    assert_ne!(mac, [0x00; 6], "e1000 MAC read back as all zeros");
    assert_ne!(mac, [0xFF; 6], "e1000 MAC read back as all ones (register not reaching the device)");

    // The Device Status register reads back a plausible value: not the all-ones of an unmapped read.
    let status = dev.status();
    assert_ne!(status, 0xFFFF_FFFF, "e1000 STATUS read back as all ones");
}

/// Stage 17b-2: the e1000 driver reset the card and applied general configuration. `e1000::init`
/// (at boot, before this harness) issues a global reset via the self-clearing CTRL.RST bit, waits
/// for it to complete, then sets Set-Link-Up in CTRL and clears the multicast table. So by now: the
/// reset must have completed, the MAC must have survived it (the reset reloads Receive Address entry
/// 0 from the EEPROM), and our CTRL.SLU write must have stuck (a live read-back proves the config
/// took effect — independent of the link's actual state).
#[test_case]
fn e1000_reset_and_configure() {
    use crate::e1000;

    let dev = e1000::device().expect("e1000 not initialized");

    // The global reset self-cleared within the timeout.
    assert!(dev.reset_succeeded(), "e1000 device reset did not complete");

    // The MAC survived the reset (reloaded into RAL0/RAH0 from the EEPROM).
    let mac = dev.mac();
    assert_ne!(mac, [0x00; 6], "e1000 MAC lost after reset");
    assert_ne!(mac, [0xFF; 6], "e1000 MAC reads back as all ones after reset");

    // Our configuration write took effect: Set-Link-Up is asserted in CTRL.
    assert!(dev.link_requested(), "CTRL.SLU not set after configure");
}

/// Stage 17b-3: the e1000 driver built the receive descriptor ring and enabled the receiver.
/// `e1000::init` (at boot) allocates the ring and its buffers, points each descriptor at a buffer,
/// programs RDBAL/RDBAH/RDLEN/RDH/RDT, and sets RCTL.EN. We read those registers back off the card
/// to confirm the ring is installed exactly as we set it — the card's own view of the ring.
#[test_case]
fn e1000_sets_up_rx_ring() {
    use crate::e1000;

    let dev = e1000::device().expect("e1000 not initialized");

    // The card's descriptor-base register must point at the ring we allocated (and it must be a real,
    // non-zero physical address), and the length must equal the ring size in bytes.
    assert_ne!(dev.rx_ring_phys(), 0, "e1000 RX ring physical address is zero");
    assert_eq!(
        dev.rx_descriptor_base(),
        dev.rx_ring_phys(),
        "RDBAH:RDBAL does not point at our RX ring"
    );
    assert_eq!(
        dev.rx_descriptor_len() as usize,
        dev.rx_count() * 16,
        "RDLEN does not match the ring size in bytes"
    );

    // The tail is a valid descriptor index (it starts at the last descriptor, but moves as frames
    // are received and recycled, so we only require it stay in range).
    assert!(
        dev.rx_tail() < dev.rx_count() as u32,
        "RDT is out of range"
    );
    assert!(dev.receiver_enabled(), "receiver (RCTL.EN) is not enabled");

    // The one-shot read-back check agrees.
    assert!(dev.rx_ring_installed(), "RX ring not installed per the read-back check");
}

/// Stage 17b-4: the e1000 driver built the transmit descriptor ring and can send a frame. `e1000::init`
/// (at boot) allocates the TX ring and buffers, programs TDBAL/TDBAH/TDLEN/TDH/TDT, and enables the
/// transmitter (TCTL). We confirm the ring reads back as installed, then transmit a raw Ethernet frame
/// and require the card to confirm it — set the descriptor's Done bit and drain the ring (head catches
/// tail). This needs no incoming traffic: DD is a purely local completion signal.
#[test_case]
fn e1000_transmits_a_frame() {
    use crate::e1000;

    let dev = e1000::device().expect("e1000 not initialized");

    // The TX ring is installed: base and length read back as programmed, transmitter enabled.
    assert_ne!(dev.tx_ring_phys(), 0, "e1000 TX ring physical address is zero");
    assert_eq!(
        dev.tx_descriptor_base(),
        dev.tx_ring_phys(),
        "TDBAH:TDBAL does not point at our TX ring"
    );
    assert_eq!(
        dev.tx_descriptor_len() as usize,
        dev.tx_count() * 16,
        "TDLEN does not match the ring size in bytes"
    );
    assert!(dev.transmitter_enabled(), "transmitter (TCTL.EN) is not enabled");
    assert!(dev.tx_ring_installed(), "TX ring not installed per the read-back check");

    // Transmit a raw frame; the card must confirm it (Descriptor Done) within the timeout.
    let sent = e1000::transmit_test_frame();
    assert!(sent, "card did not set the transmit descriptor Done bit");

    // After transmit the card has drained the ring: head has caught up to tail.
    let dev = e1000::device().expect("e1000 device handle missing");
    assert_eq!(dev.tx_head(), dev.tx_tail(), "TX ring not drained after transmit");
}

/// Stage 17b-5: the e1000 driver can receive a frame. With no external traffic under QEMU's SLIRP,
/// we prove the RX path via PHY loopback: enable loopback (transmitted frames loop straight back into
/// the receiver), send a frame addressed to our own MAC, receive it off the RX ring, and confirm the
/// bytes round-trip. This exercises the whole receive path — descriptor Done polling, the buffer DMA,
/// and descriptor recycling — end to end.
#[test_case]
fn e1000_receives_via_loopback() {
    use crate::e1000;

    assert!(e1000::present(), "e1000 not initialized");
    assert!(
        e1000::loopback_selftest(),
        "e1000 loopback receive round-trip failed (frame did not come back intact)"
    );
}

/// Stage 17b-6 / 18a: the e1000 driver receives a frame by *interrupt* rather than polling.
/// `kernel_main` has already routed the card's IRQ through the IO-APIC and armed its receive
/// interrupt (IMS). This enables loopback, sends a frame to our own MAC, confirms the interrupt fired
/// (the NAPI-style handler flags a waiting frame), and pulls the frame off the ring via `poll_frame`.
/// Exercises the full interrupt path: IO-APIC routing, the level-triggered PCI IRQ, ICR
/// cause-clearing, the pending flag, and the poll-side ring drain.
#[test_case]
fn e1000_receives_via_interrupt() {
    use crate::e1000;

    assert!(e1000::present(), "e1000 not initialized");
    assert!(
        e1000::interrupt_selftest(),
        "e1000 interrupt-driven receive failed (no interrupt flagged our looped-back frame)"
    );
    assert!(
        e1000::rx_irq_count() > 0,
        "e1000 receive interrupt never fired"
    );
    assert!(
        e1000::rx_frames() > 0,
        "no frame was polled off the RX ring"
    );
    // We polled our 60-byte frame off the ring (the card strips the CRC via RCTL.SECRC).
    assert_eq!(
        e1000::last_rx_len(),
        60,
        "polled a frame of unexpected length"
    );
}

/// Stage 18a: the Ethernet layer parses and builds frames correctly. Pure logic (no hardware):
/// build a frame, parse it back, and confirm every field round-trips, plus that a runt is rejected.
#[test_case]
fn ethernet_frame_parses_and_builds() {
    use crate::net::ether;

    let dst = [0x11, 0x22, 0x33, 0x44, 0x55, 0x66];
    let src = [0xAA; 6];
    let payload = [0xDE, 0xAD, 0xBE, 0xEF];
    let frame = ether::build(dst, src, ether::ETHERTYPE_ARP, &payload);
    assert_eq!(frame.len(), ether::HEADER_LEN + payload.len());

    let parsed = ether::Frame::parse(&frame).expect("built frame should parse");
    assert_eq!(parsed.dst, dst);
    assert_eq!(parsed.src, src);
    assert_eq!(parsed.ethertype, ether::ETHERTYPE_ARP, "EtherType must be big-endian on the wire");
    assert_eq!(parsed.payload, &payload);

    // A buffer shorter than the 14-byte header is a runt and must not parse.
    assert!(ether::Frame::parse(&[0u8; 10]).is_none());
}

/// Stage 18a: the network stack receives a frame off the NIC and dispatches it. `kernel_main` brings
/// the stack up; this loops a frame back to ourselves and confirms `net::poll` pulled it off the ring
/// and classified it (by EtherType) — the receive plumbing from the card up into the stack.
#[test_case]
fn net_receives_ethernet_frame() {
    use crate::net;

    assert!(crate::e1000::present(), "e1000 not initialized");
    assert!(
        net::loopback_selftest(),
        "net stack did not receive its own looped-back Ethernet frame"
    );
    assert!(net::frames_received() > 0, "stack parsed no frames");
    assert!(net::arp_seen() > 0, "stack did not classify the ARP-typed loopback frame");
}

/// Stage 18b: the ARP layer parses and builds packets correctly. Pure logic (no hardware): build a
/// request, parse it back, confirm the fields, and reject a runt / non-ARP buffer.
#[test_case]
fn arp_packet_parses_and_builds() {
    use crate::net::arp;

    let our_mac = [1, 2, 3, 4, 5, 6];
    let our_ip = [10, 0, 2, 15];
    let target_ip = [10, 0, 2, 2];
    let req = arp::build_request(our_mac, our_ip, target_ip);
    assert_eq!(req.len(), arp::PACKET_LEN);

    let p = arp::ArpPacket::parse(&req).expect("built request should parse");
    assert_eq!(p.oper, arp::OPER_REQUEST);
    assert_eq!(p.sha, our_mac);
    assert_eq!(p.spa, our_ip);
    assert_eq!(p.tpa, target_ip);
    assert_eq!(p.tha, [0u8; 6], "a request has no known target MAC");

    assert!(arp::ArpPacket::parse(&[0u8; 10]).is_none(), "a runt must not parse as ARP");
}

/// Stage 18b: we answer an ARP request for our own IP, and learn the sender's mapping. Pure logic:
/// `arp::process` on a crafted request for our IP returns the correct reply payload and caches the
/// sender; a request for some other IP returns no reply (but still learns the sender).
#[test_case]
fn arp_replies_to_request_for_us() {
    use crate::net::{arp, OUR_IP};

    let our_mac = [0x52, 0x54, 0x00, 0x12, 0x34, 0x56];
    let sender_mac = [0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF];
    let sender_ip = [10, 0, 2, 99];

    // "Who has OUR_IP? tell sender" — we must reply.
    let request = arp::ArpPacket {
        oper: arp::OPER_REQUEST,
        sha: sender_mac,
        spa: sender_ip,
        tha: [0; 6],
        tpa: OUR_IP,
    };
    let reply = arp::process(our_mac, OUR_IP, &request).expect("should reply to a request for us");
    let parsed = arp::ArpPacket::parse(&reply).expect("reply should parse");
    assert_eq!(parsed.oper, arp::OPER_REPLY);
    assert_eq!(parsed.sha, our_mac, "reply announces our MAC");
    assert_eq!(parsed.spa, OUR_IP);
    assert_eq!(parsed.tha, sender_mac, "reply is addressed back to the requester");
    assert_eq!(parsed.tpa, sender_ip);
    // Processing any ARP packet learns the sender's mapping.
    assert_eq!(arp::cache_lookup(sender_ip), Some(sender_mac));

    // A request for a different IP is not ours: no reply.
    let not_us = arp::ArpPacket {
        oper: arp::OPER_REQUEST,
        sha: sender_mac,
        spa: sender_ip,
        tha: [0; 6],
        tpa: [10, 0, 2, 200],
    };
    assert!(arp::process(our_mac, OUR_IP, &not_us).is_none());
}

/// Stage 18b: the first *live* network exchange. Resolve SLIRP's gateway (10.0.2.2) to a MAC via ARP
/// — broadcast a request, receive SLIRP's reply, parse it, and cache the mapping. `kernel_main`
/// already resolved it at boot, so this usually hits the cache; either way it asserts a real MAC came
/// back (non-zero and not our own).
#[test_case]
fn arp_resolves_gateway() {
    use crate::net;

    assert!(crate::e1000::present(), "e1000 not initialized");
    let mac = net::arp_resolve(net::GATEWAY_IP).expect("SLIRP gateway did not answer ARP");
    assert_ne!(mac, [0u8; 6], "gateway MAC resolved to all-zeros");
    assert_ne!(mac, net::our_mac(), "gateway MAC should differ from our own");
}

/// Stage 18c: the Internet checksum (RFC 1071). A known-answer test on the canonical IPv4 header
/// example: with the checksum field zeroed the routine yields 0xb861, and with that value in place
/// re-summing the header yields zero (how a receiver validates a header).
#[test_case]
fn internet_checksum_known_answer() {
    use crate::net::ipv4;

    let mut hdr = [
        0x45u8, 0x00, 0x00, 0x73, 0x00, 0x00, 0x40, 0x00, 0x40, 0x11, 0x00, 0x00, 0xc0, 0xa8, 0x00,
        0x01, 0xc0, 0xa8, 0x00, 0xc7,
    ];
    assert_eq!(ipv4::checksum(&hdr), 0xb861, "checksum of the zeroed header");
    hdr[10] = 0xb8;
    hdr[11] = 0x61;
    assert_eq!(ipv4::checksum(&hdr), 0, "a header with its checksum re-sums to zero");
}

/// Stage 18c: the IPv4 layer parses and builds packets, with a correct header checksum. Build a
/// packet, confirm summing its header yields zero, then parse the fields back; reject a runt.
#[test_case]
fn ipv4_header_parses_and_builds() {
    use crate::net::ipv4;

    let src = [10, 0, 2, 15];
    let dst = [10, 0, 2, 2];
    let payload = [0xAA, 0xBB, 0xCC, 0xDD];
    let pkt = ipv4::build(src, dst, ipv4::PROTO_ICMP, &payload);
    assert_eq!(pkt.len(), ipv4::HEADER_LEN + payload.len());
    assert_eq!(ipv4::checksum(&pkt[..ipv4::HEADER_LEN]), 0, "built header must carry a valid checksum");

    let parsed = ipv4::Ipv4Packet::parse(&pkt).expect("built packet should parse");
    assert_eq!(parsed.src, src);
    assert_eq!(parsed.dst, dst);
    assert_eq!(parsed.protocol, ipv4::PROTO_ICMP);
    assert_eq!(parsed.payload, &payload);
    assert!(ipv4::Ipv4Packet::parse(&[0u8; 10]).is_none(), "a runt must not parse as IPv4");
}

/// Stage 18c: the ICMP layer parses and builds echo messages, with a correct checksum. Build a
/// request and a reply, confirm each self-checksums to zero, and round-trip the fields.
#[test_case]
fn icmp_echo_parses_and_builds() {
    use crate::net::{icmp, ipv4};

    let data = b"ping-data";
    let req = icmp::build_echo_request(0x1234, 7, data);
    assert_eq!(ipv4::checksum(&req), 0, "built ICMP message must carry a valid checksum");

    let e = icmp::Echo::parse(&req).expect("request should parse");
    assert_eq!(e.typ, icmp::TYPE_ECHO_REQUEST);
    assert_eq!(e.id, 0x1234);
    assert_eq!(e.seq, 7);
    assert_eq!(e.data, data);

    let rep = icmp::build_echo_reply(0x1234, 7, data);
    let e2 = icmp::Echo::parse(&rep).expect("reply should parse");
    assert_eq!(e2.typ, icmp::TYPE_ECHO_REPLY);
    assert!(icmp::Echo::parse(&[0u8; 4]).is_none(), "a runt must not parse as ICMP echo");
}

/// Stage 19a: the UDP layer parses and builds datagrams, with a correct pseudo-header checksum. Build
/// a datagram, confirm it self-checksums to zero (recomputing over a valid datagram yields 0, since
/// the pseudo-header inputs are the same), and round-trip the ports and payload.
#[test_case]
fn udp_datagram_parses_and_builds() {
    use crate::net::udp;

    let src_ip = [10, 0, 2, 15];
    let dst_ip = [10, 0, 2, 2];
    let payload = b"aether udp payload";
    let dg = udp::build(src_ip, dst_ip, 40000, 53, payload);
    assert_eq!(dg.len(), udp::HEADER_LEN + payload.len());
    assert_eq!(
        udp::checksum(src_ip, dst_ip, &dg),
        0,
        "built datagram must carry a valid pseudo-header checksum"
    );

    let parsed = udp::Datagram::parse(&dg).expect("built datagram should parse");
    assert_eq!(parsed.src_port, 40000);
    assert_eq!(parsed.dst_port, 53);
    assert_eq!(parsed.payload, payload);

    // The checksum genuinely covers the pseudo-header: verifying with a different dst IP must fail.
    assert_ne!(
        udp::checksum(src_ip, [10, 0, 2, 3], &dg),
        0,
        "checksum must depend on the destination IP (the pseudo-header)"
    );

    assert!(udp::Datagram::parse(&[0u8; 4]).is_none(), "a runt must not parse as UDP");
}

/// Stage 19b-1: the DNS layer builds a query and parses a response. Pure logic (no network): build a
/// query and check its header + label-encoded question, then hand-craft a response that exercises the
/// two tricky parts — a compression pointer for the answer name, and a CNAME record before the A
/// record — and confirm the parser skips to the IPv4 address.
#[test_case]
fn dns_query_and_response_parse() {
    use crate::net::dns;
    use alloc::vec::Vec;

    // A query for example.com: 12-byte header, then the name as labels + QTYPE(A) + QCLASS(IN).
    let query = dns::build_query(0x1234, "example.com").expect("hostname should encode");
    assert_eq!(&query[0..2], &[0x12, 0x34], "transaction id");
    assert_eq!(&query[2..4], &[0x01, 0x00], "flags: recursion desired");
    assert_eq!(&query[4..6], &[0x00, 0x01], "one question");
    assert_eq!(
        &query[12..],
        b"\x07example\x03com\x00\x00\x01\x00\x01",
        "label-encoded name + QTYPE/QCLASS"
    );

    // Malformed hostnames are rejected: empty, and a single label longer than 63 bytes.
    assert!(dns::build_query(1, "").is_none(), "empty hostname");
    let long = core::str::from_utf8(&[b'a'; 64]).unwrap();
    assert!(dns::build_query(1, long).is_none(), "label longer than 63 bytes");

    // Hand-craft a response: id 0x1234, flags QR+RD+RA (RCODE 0), one question echoed back, and two
    // answers — a CNAME then the A record — whose names are compression pointers to offset 12.
    let mut resp: Vec<u8> = Vec::new();
    resp.extend_from_slice(&[0x12, 0x34]); // transaction id
    resp.extend_from_slice(&[0x81, 0x80]); // flags: QR=1, RD=1, RA=1, RCODE=0
    resp.extend_from_slice(&[0x00, 0x01]); // QDCOUNT = 1
    resp.extend_from_slice(&[0x00, 0x02]); // ANCOUNT = 2
    resp.extend_from_slice(&[0x00, 0x00, 0x00, 0x00]); // NSCOUNT, ARCOUNT
    resp.extend_from_slice(b"\x07example\x03com\x00"); // echoed question name (at offset 12)
    resp.extend_from_slice(&[0x00, 0x01, 0x00, 0x01]); // QTYPE = A, QCLASS = IN
    // Answer 1: a CNAME. name = pointer to offset 12; TYPE=5; CLASS=1; TTL=300; RDLENGTH=2; RDATA.
    resp.extend_from_slice(&[0xC0, 0x0C, 0x00, 0x05, 0x00, 0x01, 0x00, 0x00, 0x01, 0x2C, 0x00, 0x02]);
    resp.extend_from_slice(&[0xC0, 0x0C]); // CNAME RDATA: a (compressed) name
    // Answer 2: the A record. name = pointer; TYPE=1; CLASS=1; TTL=300; RDLENGTH=4; RDATA = address.
    resp.extend_from_slice(&[0xC0, 0x0C, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x01, 0x2C, 0x00, 0x04]);
    resp.extend_from_slice(&[93, 184, 216, 34]); // the resolved IPv4 address

    assert_eq!(
        dns::parse_response(&resp, 0x1234),
        Some([93, 184, 216, 34]),
        "parser should skip the CNAME and return the A record's address"
    );
    assert_eq!(dns::parse_response(&resp, 0x9999), None, "a response with the wrong id is rejected");
    assert_eq!(dns::parse_response(&[0u8; 4], 0x1234), None, "a runt is rejected");
}

/// Stage 20a: the DHCP layer builds DISCOVER/REQUEST messages and parses an OFFER/ACK. Pure logic (no
/// network): build a DISCOVER and check its BOOTP header, magic cookie, and message-type option, then
/// hand-craft an OFFER — exercising the TLV option walk (message type, server id, and the address
/// options subnet mask / router / DNS, plus the lease time) — and confirm the parser recovers `yiaddr`
/// and every option, rejecting a wrong transaction id and a runt.
#[test_case]
fn dhcp_message_builds_and_parses() {
    use crate::net::dhcp;
    use alloc::vec::Vec;

    let mac = [0x52, 0x54, 0x00, 0x12, 0x34, 0x56];
    let xid = 0xAABBCCDD;

    // A DISCOVER: op = 1 (client request), the transaction id at 4..8, our MAC in chaddr at 28..34,
    // the magic cookie at 236..240, and option 53 = DISCOVER right after it.
    let disc = dhcp::build_discover(xid, mac);
    assert!(disc.len() >= 240, "a DHCP message spans at least the fixed header + cookie");
    assert_eq!(disc[0], 1, "op = BOOTREQUEST");
    assert_eq!(&disc[4..8], &xid.to_be_bytes(), "transaction id");
    assert_eq!(&disc[28..34], &mac, "chaddr carries our MAC");
    assert_eq!(&disc[236..240], &[0x63, 0x82, 0x53, 0x63], "magic cookie");
    assert_eq!(&disc[240..243], &[53, 1, dhcp::DISCOVER], "option 53 = DISCOVER");

    // A REQUEST additionally carries option 50 (requested IP) and option 54 (server id).
    let req = dhcp::build_request(xid, mac, [10, 0, 2, 15], [10, 0, 2, 2]);
    assert!(
        req.windows(6).any(|w| w == [50, 4, 10, 0, 2, 15]),
        "REQUEST includes the requested IP (option 50)"
    );
    assert!(
        req.windows(6).any(|w| w == [54, 4, 10, 0, 2, 2]),
        "REQUEST names the server (option 54)"
    );

    // Hand-craft an OFFER: BOOTREPLY (op = 2), our xid, yiaddr = 10.0.2.15, the magic cookie, then
    // options — message type OFFER, server id, subnet mask, router, DNS, and a 24h lease.
    let mut offer: Vec<u8> = Vec::new();
    offer.push(2); // op = BOOTREPLY
    offer.extend_from_slice(&[1, 6, 0]); // htype, hlen, hops
    offer.extend_from_slice(&xid.to_be_bytes()); // xid at 4..8
    offer.extend_from_slice(&[0; 8]); // secs, flags, (start of addrs)
    offer.extend_from_slice(&[10, 0, 2, 15]); // yiaddr at 16..20
    offer.extend_from_slice(&[0u8; 216]); // siaddr..file, zero-filled (offset 20 up to the cookie at 236)
    offer.extend_from_slice(&[0x63, 0x82, 0x53, 0x63]); // magic cookie
    offer.extend_from_slice(&[53, 1, dhcp::OFFER]); // option 53 = OFFER
    offer.extend_from_slice(&[54, 4, 10, 0, 2, 2]); // option 54 = server id
    offer.extend_from_slice(&[1, 4, 255, 255, 255, 0]); // option 1 = subnet mask
    offer.extend_from_slice(&[3, 4, 10, 0, 2, 2]); // option 3 = router
    offer.extend_from_slice(&[6, 4, 10, 0, 2, 3]); // option 6 = DNS
    offer.extend_from_slice(&[51, 4, 0, 0x01, 0x51, 0x80]); // option 51 = lease 86400s
    offer.push(255); // END

    let r = dhcp::parse_reply(&offer, xid).expect("hand-crafted OFFER should parse");
    assert_eq!(r.msg_type, dhcp::OFFER);
    assert_eq!(r.your_ip, [10, 0, 2, 15], "yiaddr — the offered address");
    assert_eq!(r.server_id, [10, 0, 2, 2]);
    assert_eq!(r.subnet_mask, Some([255, 255, 255, 0]));
    assert_eq!(r.router, Some([10, 0, 2, 2]));
    assert_eq!(r.dns, Some([10, 0, 2, 3]));
    assert_eq!(r.lease_secs, Some(86400));

    assert!(dhcp::parse_reply(&offer, 0x99999999).is_none(), "wrong transaction id is rejected");
    assert!(dhcp::parse_reply(&[0u8; 8], xid).is_none(), "a runt is rejected");
}

/// Stage 21a: the TCP layer builds and parses segments, with a correct pseudo-header checksum. Pure
/// logic (no connection): build a SYN, check its header fields (seq, data offset, flags) and that it
/// self-checksums to zero, round-trip the fields, parse a segment that carries options (data offset > 5)
/// to confirm the payload starts after them, and reject a runt and a bogus data offset.
#[test_case]
fn tcp_segment_builds_and_parses() {
    use crate::net::tcp;

    let src_ip = [10, 0, 2, 15];
    let dst_ip = [10, 0, 2, 2];
    // A pure SYN (no payload): our initial sequence number, an advertised window, the SYN flag.
    let seq = 0x1122_3344;
    let syn = tcp::build(src_ip, dst_ip, 40000, 80, seq, 0, tcp::SYN, 64240, &[]);
    assert_eq!(syn.len(), tcp::HEADER_LEN, "a flags-only segment is just the header");
    assert_eq!(&syn[4..8], &seq.to_be_bytes(), "sequence number");
    assert_eq!(syn[12] >> 4, 5, "data offset = 5 words (20-byte header, no options)");
    assert_eq!(syn[13], tcp::SYN, "only the SYN flag is set");
    assert_eq!(
        tcp::checksum(src_ip, dst_ip, &syn),
        0,
        "a built segment must carry a valid pseudo-header checksum"
    );

    let s = tcp::Segment::parse(&syn).expect("built SYN should parse");
    assert_eq!(s.src_port, 40000);
    assert_eq!(s.dst_port, 80);
    assert_eq!(s.seq, seq);
    assert_eq!(s.flags, tcp::SYN);
    assert_eq!(s.window, 64240);
    assert!(s.payload.is_empty(), "a SYN carries no stream data");

    // A segment with a data offset of 6 words (24-byte header): 4 option bytes then the payload "hi".
    // The parser must skip the options and return just "hi".
    let mut with_opts = tcp::build(src_ip, dst_ip, 80, 40000, 0, seq + 1, tcp::ACK, 100, b"hi");
    with_opts[12] = 6 << 4; // claim a 24-byte header ...
    with_opts.splice(tcp::HEADER_LEN..tcp::HEADER_LEN, [0x02, 0x04, 0x05, 0xB4]); // ... insert an MSS option
    let s2 = tcp::Segment::parse(&with_opts).expect("segment with options should parse");
    assert_eq!(s2.flags, tcp::ACK);
    assert_eq!(s2.payload, b"hi", "payload must start after the options");

    assert!(tcp::Segment::parse(&[0u8; 10]).is_none(), "a runt must not parse as TCP");
    let mut bad = syn.clone();
    bad[12] = 0xF0; // data offset 15 words = 60 bytes, past this 20-byte buffer
    assert!(tcp::Segment::parse(&bad).is_none(), "a data offset past the buffer is rejected");
}

/// Stage 18c: the full ICMP path bidirectionally, via loopback (no external peer). Send an echo
/// request to ourselves; the stack answers it and receives its own reply — exercising build, parse,
/// checksum, dispatch, the reply we generate, and receiving that reply.
#[test_case]
fn net_pings_over_loopback() {
    use crate::net;

    assert!(crate::e1000::present(), "e1000 not initialized");
    assert!(net::ping_loopback_selftest(), "ICMP loopback round-trip failed");
    assert!(net::icmp_requests_handled() > 0, "we answered no echo request");
    assert!(net::icmp_replies_received() > 0, "we received no echo reply");
}

/// Stage 19a-2: the full UDP path bidirectionally, via loopback (no external peer). Send a datagram to
/// our own echo server; the stack echoes it and receives the echo back — exercising build, parse, the
/// pseudo-header checksum, protocol dispatch, the echo we generate, and delivering it with its bytes
/// intact.
#[test_case]
fn net_udp_echoes_over_loopback() {
    use crate::net;

    assert!(crate::e1000::present(), "e1000 not initialized");
    assert!(net::udp_echo_loopback_selftest(), "UDP echo loopback round-trip failed");
    assert!(net::udp_echoes_sent() > 0, "our echo server bounced nothing");
    assert!(net::udp_delivered() > 0, "we received no echoed datagram back");
}

/// Stage 21b: the TCP three-way handshake bidirectionally, via loopback (no external peer). Listen on a
/// port and connect to ourselves: the SYN / SYN-ACK / ACK exchange loops back so a client TCB and a
/// server TCB both reach ESTABLISHED — exercising the active and passive opens, the sequence-number
/// bookkeeping, and the segment build/parse + checksum on the wire.
#[test_case]
fn tcp_completes_handshake_over_loopback() {
    use crate::net;

    assert!(crate::e1000::present(), "e1000 not initialized");
    assert!(net::tcp_handshake_loopback_selftest(), "TCP handshake over loopback failed");
    assert_eq!(
        crate::net::tcp::established_count(),
        2,
        "both the client and server TCBs should be ESTABLISHED"
    );
}

/// Stage 21c: TCP data transfer bidirectionally, via loopback (no external peer). Establish a loopback
/// connection, send a payload from the client, and confirm the server buffered exactly those bytes in
/// order and the client saw them acknowledged — exercising the send path (sequence tracking), the
/// in-order receive path, and the ACK that closes the reliable-transfer loop.
#[test_case]
fn tcp_transfers_data_over_loopback() {
    use crate::net;

    assert!(crate::e1000::present(), "e1000 not initialized");
    assert!(net::tcp_data_loopback_selftest(), "TCP data transfer over loopback failed");
}

/// Stage 21d: TCP connection teardown via loopback (no external peer). Establish a loopback connection,
/// then actively close one end and passively close the other, walking the full four-way FIN handshake —
/// and assert the active closer ends in TIME_WAIT and the passive closer in CLOSED. Exercises the FIN
/// handshake states, each FIN consuming a sequence number, and each FIN being acknowledged.
#[test_case]
fn tcp_tears_down_over_loopback() {
    use crate::net;

    assert!(crate::e1000::present(), "e1000 not initialized");
    assert!(net::tcp_teardown_loopback_selftest(), "TCP teardown over loopback failed");
}

/// Stage 21e: TCP retransmission via loopback (no external peer). Establish a connection, drop a data
/// segment on purpose (loss injection), and confirm the retransmission timer resends it so the transfer
/// still completes in order and acknowledged — then that the active closer's TIME_WAIT expires to CLOSED
/// under the timer. Exercises the retransmit queue, the timeout/resend, and the timed close.
#[test_case]
fn tcp_recovers_from_loss_over_loopback() {
    use crate::net;

    assert!(crate::e1000::present(), "e1000 not initialized");
    assert!(net::tcp_retransmit_loopback_selftest(), "TCP retransmission over loopback failed");
    assert!(net::tcp_retransmits() > 0, "the retransmission timer never resent anything");
}

/// Stage 22a: TCP out-of-order reassembly via loopback (no external peer). Establish a connection, then
/// send a payload as two segments delivered in reversed order (reorder injection), and confirm the
/// receiver buffers the ahead-of-sequence segment and splices it into the stream once the earlier segment
/// fills the gap — so the bytes arrive in order and both are acknowledged. Exercises the reassembly queue.
#[test_case]
fn tcp_reassembles_out_of_order() {
    use crate::net;

    assert!(crate::e1000::present(), "e1000 not initialized");
    let before = net::tcp_out_of_order_buffered();
    assert!(net::tcp_reassembly_loopback_selftest(), "TCP out-of-order reassembly over loopback failed");
    assert!(
        net::tcp_out_of_order_buffered() > before,
        "the reassembly path was never exercised (no out-of-order segment buffered)",
    );
}

/// Stage 19b-2: the live DNS resolver — resolve a hostname through SLIRP's DNS server over the wire.
/// Unlike the SLIRP-internal gateway ping, this depends on the *host* having working upstream DNS
/// (SLIRP forwards to it), so the test is lenient: it always exercises the full path (build the query,
/// ARP the server, send, poll, parse) and asserts a sane address when one comes back, but only notes a
/// timeout (an offline host) instead of failing. The deterministic parsing is covered by
/// `dns_query_and_response_parse`.
#[test_case]
fn net_resolves_a_hostname() {
    use crate::net;

    assert!(crate::e1000::present(), "e1000 not initialized");
    // The DNS server must be reachable on the link — this part is SLIRP-internal, so always true.
    assert!(net::arp_resolve(net::DNS_SERVER).is_some(), "DNS server did not answer ARP");

    match net::dns_resolve("example.com") {
        Some(ip) => {
            assert_ne!(ip, [0, 0, 0, 0], "resolved address must not be all-zeros");
            assert_ne!(ip, [255, 255, 255, 255], "resolved address must not be broadcast");
        }
        None => crate::serial_println!(
            "[test] dns_resolve got no answer (host likely has no upstream DNS); skipping assert"
        ),
    }
}

/// Stage 18c: the headline — ping SLIRP's gateway over the (emulated) wire and get an echo reply.
/// `net::ping` resolves the MAC, sends ICMP-in-IPv4-in-Ethernet, and matches the returning reply by
/// identifier and sequence number.
#[test_case]
fn pings_the_gateway() {
    use crate::net;

    assert!(crate::e1000::present(), "e1000 not initialized");
    let seq = net::ping(net::GATEWAY_IP).expect("SLIRP gateway did not answer our ping");
    assert!(seq >= 1, "ping returned an invalid sequence number");
}

/// Stage 20b: the live DHCP client leased our address during boot, over the wire. `kernel_main` runs
/// the DORA exchange against SLIRP's built-in DHCP server before the test harness starts, so by now we
/// must hold a lease: our live address is configured (not 0.0.0.0), and — because SLIRP's server is
/// deterministic — it is exactly the address/gateway/DNS SLIRP always hands out, learned dynamically
/// rather than hardcoded. (A DHCP server is SLIRP-internal, so this is reliable, unlike upstream DNS.)
#[test_case]
fn dhcp_leases_an_address() {
    use crate::net;

    assert!(crate::e1000::present(), "e1000 not initialized");
    assert_ne!(net::our_ip(), [0, 0, 0, 0], "the interface has no address after boot");
    assert!(net::dhcp_configured(), "DHCP did not install a lease from SLIRP");
    // SLIRP always leases the default guest address 10.0.2.15, gateway 10.0.2.2, DNS 10.0.2.3.
    assert_eq!(net::our_ip(), net::OUR_IP, "DHCP leased an unexpected address");
    assert_eq!(net::leased_gateway(), net::GATEWAY_IP, "DHCP router option mismatch");
    assert_eq!(net::leased_dns(), net::DNS_SERVER, "DHCP DNS option mismatch");
    assert!(net::lease_secs() > 0, "DHCP lease time should be positive");
}

/// Stage 18d: the `ping` command's dotted-decimal IPv4 parser. Pure logic: accept well-formed
/// addresses and reject malformed ones (wrong part count, out-of-range octet, non-numeric).
#[test_case]
fn parse_ipv4_accepts_and_rejects() {
    use crate::net;

    assert_eq!(net::parse_ipv4("10.0.2.2"), Some([10, 0, 2, 2]));
    assert_eq!(net::parse_ipv4("255.255.255.255"), Some([255, 255, 255, 255]));
    assert_eq!(net::parse_ipv4("  1.2.3.4 "), Some([1, 2, 3, 4]), "surrounding spaces are trimmed");

    assert!(net::parse_ipv4("10.0.2").is_none(), "too few octets");
    assert!(net::parse_ipv4("10.0.2.2.2").is_none(), "too many octets");
    assert!(net::parse_ipv4("10.0.2.256").is_none(), "octet out of range");
    assert!(net::parse_ipv4("10.0.x.2").is_none(), "non-numeric octet");
    assert!(net::parse_ipv4("").is_none(), "empty string");
}

/// Stage 18d: the ARP cache snapshot the `arp` command prints reflects real learned mappings. After
/// resolving the gateway, its (IP, MAC) pair appears in `cache_entries` with a MAC matching the
/// direct lookup.
#[test_case]
fn arp_cache_snapshot_lists_the_gateway() {
    use crate::net;

    let mac = net::arp_resolve(net::GATEWAY_IP).expect("gateway did not answer ARP");
    let entries = net::arp::cache_entries();
    let found = entries.iter().find(|(ip, _)| *ip == net::GATEWAY_IP);
    assert_eq!(found.map(|(_, m)| *m), Some(mac), "gateway mapping missing from the ARP cache snapshot");
}

/// Stage 14c-1: the FAT driver creates and overwrites a root-level file. Write a payload
/// spanning several clusters through the global VFS (`/mnt/...`), read it back, and confirm the
/// bytes round-trip — exercising free-cluster allocation, the cluster chain, and the directory
/// entry, then re-reading them through the independent read path. Overwrites a fixed name, so
/// re-running `cargo test` reuses the entry (the file persists on the disk image — real
/// persistence) without the root directory growing.
#[test_case]
fn fat_writes_a_file() {
    use crate::fs;
    // A multi-cluster payload (cluster = 512 B here): a position-dependent pattern, so a
    // misplaced or dropped byte anywhere fails the comparison.
    let mut data = alloc::vec::Vec::new();
    for i in 0..1500u32 {
        data.push((i.wrapping_mul(7).wrapping_add(3)) as u8);
    }
    fs::write("/mnt/WRITTEN.DAT", &data).expect("writing a FAT file failed");
    assert_eq!(fs::read("/mnt/WRITTEN.DAT").unwrap(), data);

    // Overwriting with a shorter payload updates the size and frees the tail clusters.
    let small = b"second, shorter contents".to_vec();
    fs::write("/mnt/WRITTEN.DAT", &small).expect("overwriting a FAT file failed");
    assert_eq!(fs::read("/mnt/WRITTEN.DAT").unwrap(), small);

    // The new file shows up in the mounted directory listing as a regular file.
    let entries = fs::list("/mnt").unwrap();
    assert!(entries.iter().any(|(name, is_dir)| name.as_str() == "WRITTEN.DAT" && !*is_dir));
}

/// Stage 14c-2: the FAT driver removes a root-level file — frees its cluster chain and marks the
/// directory entry deleted. Write a file through the VFS, confirm it reads back, remove it, then
/// confirm it is gone (reading and re-removing both report `NotFound`, and it is off the
/// listing). Self-cleaning, so it leaves the disk image as it found it.
#[test_case]
fn fat_removes_a_file() {
    use crate::fs;
    use crate::fs::FsError;
    let data = b"a file that will be deleted".to_vec();
    fs::write("/mnt/DELME.TXT", &data).expect("writing the file to remove failed");
    assert_eq!(fs::read("/mnt/DELME.TXT").unwrap(), data);

    fs::remove("/mnt/DELME.TXT").expect("removing the file failed");

    // Gone: reading and re-removing both report NotFound, and it is absent from the listing.
    assert_eq!(fs::read("/mnt/DELME.TXT"), Err(FsError::NotFound));
    assert_eq!(fs::remove("/mnt/DELME.TXT"), Err(FsError::NotFound));
    let entries = fs::list("/mnt").unwrap();
    assert!(!entries.iter().any(|(name, _)| name.as_str() == "DELME.TXT"));
}

/// Stage 14b-3: the FAT volume is mounted into the global VFS at /mnt during boot, so the
/// shell's `fs::*` API reaches disk files transparently. `kernel_main` mounts it before this
/// harness runs (the very path the interactive shell uses), so reading `/mnt/HELLO.TXT` through
/// the global `fs::read` returns the disk file, while paths outside `/mnt` stay in the
/// in-memory tree.
#[test_case]
fn fat_mounts_into_vfs() {
    use crate::fs;
    // Must match FAT_FILE_CONTENT in build.rs.
    const EXPECTED: &[u8] = b"Hello from a real FAT16 disk, read by Aether.\n";

    // The mount point is a directory, and the disk file reads through the global API.
    assert!(fs::is_dir("/mnt"));
    assert_eq!(fs::read("/mnt/HELLO.TXT").unwrap(), EXPECTED);

    // The file shows up listing the mount point, and `/mnt` itself shows up listing the root.
    let mnt = fs::list("/mnt").unwrap();
    assert!(mnt.iter().any(|(name, is_dir)| name.as_str() == "HELLO.TXT" && !*is_dir));
    let root = fs::list("/").unwrap();
    assert!(root.iter().any(|(name, is_dir)| name.as_str() == "mnt" && *is_dir));

    // A path outside the mount still routes to the in-memory tree (no disk involved).
    fs::mkdir("/vfs_probe").unwrap();
    fs::write("/vfs_probe/f", b"ram").unwrap();
    assert_eq!(fs::read("/vfs_probe/f").unwrap(), b"ram".to_vec());
    fs::remove("/vfs_probe").unwrap();
}

/// Stage 16a: ACPI discovery enumerated every CPU core. QEMU is launched with
/// `-smp 4` (see Cargo.toml `test-args`), so the firmware's MADT must list four
/// Processor Local APIC entries; `kernel_main` parses it (via `acpi::discover`)
/// before this harness runs. Exactly one core is flagged the BSP, and its apic id
/// must match what this running core's Local APIC reports — proving we both found
/// the APs and correctly identified ourselves among them.
#[test_case]
fn acpi_discovers_all_cpus() {
    use crate::{acpi, apic};
    // We asked QEMU for 4 CPUs; the MADT must enumerate all of them.
    assert_eq!(acpi::cpu_count(), 4);
    // The recorded BSP is this running core.
    assert_eq!(acpi::bsp_apic_id(), apic::lapic_id());
    // The other three are application processors, none of them flagged as the BSP.
    let aps = acpi::application_processors();
    assert_eq!(aps.len(), 3);
    assert!(aps.iter().all(|c| !c.is_bsp));
}

/// Stage 16b-1: the Local APIC can send and receive an IPI. The BSP sends a fixed
/// IPI to its own APIC id on a dedicated vector; the handler sets a flag and EOIs.
/// This proves the ICR send + delivery-status poll path — the same one Stage 16b-2
/// uses for INIT-SIPI-SIPI — works, on a single core with no assembly. The harness
/// runs with interrupts enabled (boot turns them on before reaching here), so the
/// self-IPI can actually be taken.
#[test_case]
fn self_ipi_is_delivered() {
    assert!(crate::interrupts::self_ipi_works());
}

/// Stage 16b-2a: the BSP woke an application processor. Boot copies a trampoline to
/// low memory and sends the target AP INIT-SIPI-SIPI; the AP writes a progress marker
/// the BSP polls. By the time this harness runs that wake-up must have succeeded —
/// proving the INIT-SIPI-SIPI sequence works and a second core executed our code.
#[test_case]
fn woke_an_application_processor() {
    assert!(crate::smp::ap_stage() >= 1);
}

/// Stage 16b-2b: the woken AP climbed the full real -> protected -> long mode ladder.
/// The trampoline writes a higher marker at each rung; reaching stage 3 means the AP
/// loaded the kernel CR3, enabled PAE + paging + long mode, and far-jumped into 64-bit
/// code — all on a second core. Boot records the highest stage reached.
#[test_case]
fn ap_reaches_long_mode() {
    assert_eq!(crate::smp::ap_stage(), 3);
}

/// Stage 16b-3: a woken AP far-jumped into Rust on its own stack and reported in. The
/// trampoline loads a per-AP stack and jumps to `ap_entry`, which bumps an online counter
/// the BSP polls. Reaching it means a second core is executing real kernel Rust — not
/// merely sitting in the hand-written trampoline. (Stage 16c wakes *all* the APs, so the
/// count is now >= 1; `all_application_processors_online` asserts the exact total.)
#[test_case]
fn ap_comes_online() {
    assert!(crate::smp::aps_online() >= 1);
}

/// Refinement: guard-paged kernel stacks. A scheduled kernel thread's stack has an
/// unmapped guard page just below its usable region, so an overflow raises a page fault
/// instead of silently corrupting the heap. This checks the mechanism directly — a fresh
/// `GuardedStack` has its guard page unmapped and its usable region mapped, and freeing it
/// restores the guard page's mapping (so the heap can safely reuse the memory) — and also
/// confirms the boot-time `demo_guard_page` check passed.
#[test_case]
fn thread_stack_has_guard_page() {
    use crate::memory::{self, GuardedStack};

    // The boot demo (run before the tests, from `kernel_main`) confirmed the whole
    // allocate / unmap / restore cycle end to end.
    assert!(memory::guard_page_ok(), "boot-time guard-page check failed");

    // And check it directly on a fresh stack: guard page unmapped, usable mapped.
    let stack = GuardedStack::new(4096);
    let guard = stack.guard_page();
    let usable = stack.usable_bottom();
    assert!(!memory::page_is_present(guard), "guard page should be unmapped");
    assert!(memory::page_is_present(usable), "usable stack should be mapped");

    // Freeing the stack restores the guard page's mapping.
    drop(stack);
    assert!(
        memory::page_is_present(guard),
        "guard page should be remapped after the stack is freed"
    );
}

/// Stage 16c: every application processor was woken — not just one — and each has its
/// own per-CPU data block. Boot calls `percpu::init` for all four cores, then
/// `smp::boot_aps` wakes all three APs; each AP enters `ap_entry`, finds its own block by
/// its LAPIC id, and marks it online. So by the time this harness runs, all three APs have
/// reported in and all four cores (BSP + APs) are online in their per-CPU data, each AP on
/// a distinct, nonzero stack.
#[test_case]
fn all_application_processors_online() {
    use crate::{percpu, smp};
    // All three APs ran `ap_entry`; with the BSP, all four cores have a per-CPU block,
    // and every one is marked online.
    assert_eq!(smp::aps_online(), 3);
    assert_eq!(percpu::count(), 4);
    assert_eq!(percpu::online_count(), 4);

    // Each AP recorded the stack it is running on — a distinct, nonzero per-core value;
    // the BSP's stack field stays 0 (it kept the bootloader's stack).
    let ap_stacks: alloc::vec::Vec<u64> = percpu::all()
        .iter()
        .filter(|cpu| !cpu.is_bsp)
        .map(|cpu| cpu.stack())
        .collect();
    assert_eq!(ap_stacks.len(), 3);
    assert!(ap_stacks.iter().all(|&s| s != 0));
    // The three AP stacks are all different (each AP got its own heap stack).
    for i in 0..ap_stacks.len() {
        for j in (i + 1)..ap_stacks.len() {
            assert_ne!(ap_stacks[i], ap_stacks[j]);
        }
    }
}

/// Stage 16d-1: each woken AP runs its own Local APIC timer. In `ap_entry` every AP loads
/// the kernel GDT + the shared IDT, software-enables its Local APIC, starts its periodic
/// timer, and `sti`s; from then it takes timer interrupts on its own core and counts them
/// in its per-CPU block (`interrupts::timer_dispatch` routes an AP tick there). These tests
/// run late in boot (after the whole user-process demo), so by now every AP must have
/// accumulated ticks — proof the non-boot cores are doing autonomous work, not parked.
#[test_case]
fn aps_take_timer_interrupts() {
    use crate::percpu;
    for cpu in percpu::all().iter().filter(|c| !c.is_bsp) {
        assert!(
            cpu.timer_ticks() > 0,
            "AP cpu{} (apic id {}) took no timer interrupts",
            cpu.cpu_index,
            cpu.apic_id,
        );
    }
}

/// Stage 16d-4: each AP preemptively scheduled several kernel threads on its own per-CPU run
/// queue. In `ap_entry` every AP spawns `AP_THREADS` workers that busy-spin and **never
/// yield**, then idles; this core's timer preempts whatever is running on each tick,
/// round-robining them. By the time these tests run, every AP must have: completed exactly
/// `AP_THREADS` threads, recorded at least one timer preemption (proof scheduling was
/// preemptive, not cooperative — nothing yielded), done some work, and drained back to its
/// bootstrap context (`scheduler_done`).
#[test_case]
fn aps_preempt_threads() {
    use crate::{percpu, smp};
    for cpu in percpu::all().iter().filter(|c| !c.is_bsp) {
        assert_eq!(
            cpu.threads_completed(),
            smp::AP_THREADS as u64,
            "AP cpu{} completed {} threads, expected {}",
            cpu.cpu_index,
            cpu.threads_completed(),
            smp::AP_THREADS,
        );
        assert!(
            cpu.preemptions() > 0,
            "AP cpu{} recorded no timer preemptions (scheduling was not preemptive)",
            cpu.cpu_index,
        );
        assert!(cpu.work() > 0, "AP cpu{} did no work", cpu.cpu_index);
        assert!(
            cpu.scheduler_done(),
            "AP cpu{} run queue did not drain back to its bootstrap context",
            cpu.cpu_index,
        );
    }
}

/// Stage 16d-5: the async executor runs as a scheduled kernel thread, unified with the
/// per-CPU scheduler on the BSP. `boot_continue` calls `unify::demo()` (in both build
/// profiles) before the tests: it spawns an async-executor thread and a plain kernel
/// thread on the BSP's own run queue and lets the BSP timer preempt them to completion.
/// By the time this runs, both must have done work (so the executor really ran *as a
/// thread* alongside a kernel thread under one scheduler) and the BSP timer must have
/// preempted between them (so the BSP's ring-0 tick now drives `sched::preempt`).
#[test_case]
fn bsp_unifies_executor_and_threads() {
    use crate::{percpu, unify};
    assert!(
        unify::async_work() > 0,
        "the async task never ran on its executor thread"
    );
    assert!(unify::kernel_work() > 0, "the kernel demo thread never ran");
    let bsp = percpu::all()
        .iter()
        .find(|c| c.is_bsp)
        .expect("no BSP per-CPU block");
    assert!(
        bsp.preemptions() > 0,
        "the BSP timer did not preempt the unify demo threads"
    );
}
