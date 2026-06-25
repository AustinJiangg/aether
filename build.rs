//! Host build script (Stage 13b).
//!
//! Runs on the *host* at build time (it is ordinary `std` Rust, not kernel code), before
//! `cargo run`/`cargo test` launch QEMU. Its only job is to make sure the ATA write
//! experiments have a disk to write to: QEMU refuses to start if a `-drive` backing file
//! is missing, and we deliberately do not write to the boot image. So we create a small,
//! zero-filled scratch disk image once, attached as the primary slave in `Cargo.toml`'s
//! `run-args`/`test-args`.
//!
//! The image is git-ignored — a local artifact, regenerated on demand. We create it only
//! if absent, so its contents persist across boots (a real disk's whole point). Deleting
//! it changes the package fingerprint, so Cargo re-runs this script and recreates it; we
//! intentionally emit no `rerun-if-changed` directive that would suppress that.

use std::path::Path;

/// Scratch disk size: 1 MiB = 2048 sectors of 512 bytes. Tiny, but ample for the
/// single-sector read/write experiments.
const SCRATCH_SIZE: usize = 1024 * 1024;

fn main() {
    let manifest_dir = std::env::var("CARGO_MANIFEST_DIR")
        .expect("CARGO_MANIFEST_DIR is always set for build scripts");
    let scratch = Path::new(&manifest_dir).join("scratch.img");

    if !scratch.exists() {
        std::fs::write(&scratch, vec![0u8; SCRATCH_SIZE])
            .expect("build.rs: failed to create scratch.img");
    }
}
