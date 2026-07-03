//! Host build script (Stage 13b / 14b).
//!
//! Runs on the *host* at build time (it is ordinary `std` Rust, not kernel code), before
//! `cargo run`/`cargo test` launch QEMU. Its job is to make sure the disk-experiment images
//! QEMU attaches actually exist on disk — QEMU refuses to start if a `-drive` backing file
//! is missing. Both images are git-ignored local artifacts, regenerated on demand; we create
//! each only if absent, so their contents persist across boots (a real disk's whole point).
//!
//! Two disks:
//!  - `scratch.img` (Stage 13b): a small zero-filled disk for the raw ATA sector-write
//!    experiments. Attached as the primary IDE slave.
//!  - `fat.img` (Stage 14b): a FAT16-formatted disk holding a known file — plus, since Stage
//!    14d-2, a subdirectory with a nested file so the kernel's traversal has a real target — so
//!    the kernel's FAT reader has a real filesystem (produced by the host's `mkfs.fat`) to
//!    parse. Attached as the secondary IDE master.
//!
//! We intentionally emit no `rerun-if-changed` directive: deleting an image then changes the
//! package fingerprint, so Cargo re-runs this script and recreates it.

use std::path::{Path, PathBuf};
use std::process::Command;

/// Scratch disk size: 1 MiB = 2048 sectors of 512 bytes. Tiny, but ample for the
/// single-sector read/write experiments.
const SCRATCH_SIZE: usize = 1024 * 1024;

/// FAT disk size: 5 MiB. With one sector per cluster this yields ~10k clusters — inside the
/// FAT16 range (4085..65525), so `mkfs.fat -F 16` accepts it without complaint.
const FAT_SIZE: usize = 5 * 1024 * 1024;

/// The known file dropped into the FAT image, so the kernel's reader has a fixed target to
/// find and verify. The name is upper-case 8.3 because that is how it lands in a FAT
/// directory entry (`HELLO   TXT`).
const FAT_FILE_NAME: &str = "HELLO.TXT";
const FAT_FILE_CONTENT: &str = "Hello from a real FAT16 disk, read by Aether.\n";

/// A subdirectory, and a known file inside it, seeded into the FAT image so the kernel's
/// subdirectory *traversal* (Stage 14d-2) has a real nested target to find and read.
const FAT_SUBDIR_NAME: &str = "SUB";
const FAT_NESTED_NAME: &str = "NESTED.TXT";
const FAT_NESTED_CONTENT: &str = "Nested file inside a FAT16 subdirectory.\n";

fn main() {
    let manifest_dir = std::env::var("CARGO_MANIFEST_DIR")
        .expect("CARGO_MANIFEST_DIR is always set for build scripts");
    let manifest_dir = Path::new(&manifest_dir);

    // Stage 13b: the raw-write scratch disk — just a zero-filled file.
    let scratch = manifest_dir.join("scratch.img");
    if !scratch.exists() {
        std::fs::write(&scratch, vec![0u8; SCRATCH_SIZE])
            .expect("build.rs: failed to create scratch.img");
    }

    // Stage 14b: the FAT16 test disk — formatted by the host's mkfs.fat, with one known file
    // copied in via mtools' mcopy.
    let fat = manifest_dir.join("fat.img");
    if !fat.exists() {
        create_fat_image(&fat);
    }
}

/// Create `fat.img`: a zero-filled file, formatted FAT16, with [`FAT_FILE_NAME`] copied in.
fn create_fat_image(path: &Path) {
    // 1. Lay down the backing file at the target size; mkfs.fat formats it in place.
    std::fs::write(path, vec![0u8; FAT_SIZE]).expect("build.rs: failed to create fat.img");

    // 2. Format as FAT16, one sector per cluster (the simplest geometry), volume label AETHER.
    let mkfs = resolve_tool("mkfs.fat").unwrap_or_else(|| {
        panic!(
            "build.rs: `mkfs.fat` not found. Install it with `sudo apt install dosfstools` \
             (and `mtools` for mcopy), or delete fat.img and rebuild after installing."
        )
    });
    run(
        Command::new(&mkfs)
            .args(["-F", "16", "-s", "1", "-n", "AETHER"])
            .arg(path),
        "mkfs.fat",
    );

    // 3. Write the known file to OUT_DIR, then copy it into the image's root directory.
    let out_dir = std::env::var("OUT_DIR").expect("OUT_DIR is set for build scripts");
    let local = Path::new(&out_dir).join("hello.txt");
    std::fs::write(&local, FAT_FILE_CONTENT).expect("build.rs: failed to write temp hello.txt");

    let mcopy = resolve_tool("mcopy").unwrap_or_else(|| {
        panic!(
            "build.rs: `mcopy` not found. Install it with `sudo apt install mtools`, or \
             delete fat.img and rebuild after installing."
        )
    });
    // MTOOLS_SKIP_CHECK=1 silences mtools' picky disk-geometry checks on our plain image.
    run(
        Command::new(&mcopy)
            .env("MTOOLS_SKIP_CHECK", "1")
            .arg("-i")
            .arg(path)
            .arg(&local)
            .arg(format!("::{}", FAT_FILE_NAME)),
        "mcopy",
    );

    // Stage 14d-2: seed a subdirectory holding a known file, so the kernel's traversal has a real
    // nested target. `mmd` makes the directory (a cluster with `.`/`..` plus a root entry); the
    // second `mcopy` drops a file into it. The kernel reaches it as `/mnt/SUB/NESTED.TXT`.
    let mmd = resolve_tool("mmd").unwrap_or_else(|| {
        panic!(
            "build.rs: `mmd` not found. Install it with `sudo apt install mtools`, or \
             delete fat.img and rebuild after installing."
        )
    });
    run(
        Command::new(&mmd)
            .env("MTOOLS_SKIP_CHECK", "1")
            .arg("-i")
            .arg(path)
            .arg(format!("::{}", FAT_SUBDIR_NAME)),
        "mmd",
    );

    let nested = Path::new(&out_dir).join("nested.txt");
    std::fs::write(&nested, FAT_NESTED_CONTENT).expect("build.rs: failed to write temp nested.txt");
    run(
        Command::new(&mcopy)
            .env("MTOOLS_SKIP_CHECK", "1")
            .arg("-i")
            .arg(path)
            .arg(&nested)
            .arg(format!("::{}/{}", FAT_SUBDIR_NAME, FAT_NESTED_NAME)),
        "mcopy (nested)",
    );
}

/// Find a host tool by name: search `PATH`, then the usual sbin/bin dirs (a build script's
/// `PATH` does not always include `/usr/sbin`, where `mkfs.fat` lives).
fn resolve_tool(name: &str) -> Option<PathBuf> {
    if let Ok(path) = std::env::var("PATH") {
        for dir in std::env::split_paths(&path) {
            let candidate = dir.join(name);
            if candidate.is_file() {
                return Some(candidate);
            }
        }
    }
    for dir in ["/usr/sbin", "/sbin", "/usr/bin", "/bin"] {
        let candidate = Path::new(dir).join(name);
        if candidate.is_file() {
            return Some(candidate);
        }
    }
    None
}

/// Run a host command to completion, panicking with its output if it fails.
fn run(cmd: &mut Command, label: &str) {
    let output = cmd
        .output()
        .unwrap_or_else(|e| panic!("build.rs: failed to spawn {label}: {e}"));
    if !output.status.success() {
        panic!(
            "build.rs: {label} failed ({})\nstdout: {}\nstderr: {}",
            output.status,
            String::from_utf8_lossy(&output.stdout),
            String::from_utf8_lossy(&output.stderr),
        );
    }
}
