//! Stage 13a/13b: a minimal ATA (IDE) disk driver in PIO mode — reading *and now
//! writing* raw sectors.
//!
//! This is the persistence track: getting bytes on and off a real disk. We use the
//! oldest, simplest method — **ATA PIO (Programmed I/O)**: the CPU drives the transfer
//! itself by reading and writing a handful of I/O ports, and *polls* a status register
//! to know when the drive is ready. No DMA, no interrupts. Slow, but easy to get right.
//!
//! ## The register interface
//!
//! The legacy "primary" ATA bus exposes its command-block registers at I/O ports
//! `0x1F0..=0x1F7`, plus a control/alternate-status register at `0x3F6`:
//!
//! ```text
//!   0x1F0  Data            16-bit; 256 transfers move one 512-byte sector
//!   0x1F2  Sector count    how many sectors to transfer
//!   0x1F3  LBA low         logical block address, bits  0..8
//!   0x1F4  LBA mid         logical block address, bits  8..16
//!   0x1F5  LBA high        logical block address, bits 16..24
//!   0x1F6  Drive/Head      drive select + LBA mode + LBA bits 24..28
//!   0x1F7  Status (read) / Command (write)
//!   0x3F6  Alternate status (read) — same status byte, but reading it has no
//!          side effects (reading 0x1F7 acknowledges a pending IRQ; 0x3F6 does not)
//! ```
//!
//! One bus carries up to two drives — a **master** and a **slave** — distinguished
//! only by bit 4 of the drive-select byte (`0xE0` master, `0xF0` slave). They share
//! every other register, so we select one before each operation. See [`Drive`].
//!
//! ## The read protocol (28-bit LBA, READ SECTORS = 0x20)
//!
//! 1. wait until the drive is not BSY;
//! 2. write the drive-select byte (master/slave, LBA mode, top 4 LBA bits);
//! 3. write the sector count and the low 24 LBA bits;
//! 4. write the READ SECTORS command;
//! 5. poll the status register until BSY clears and DRQ (data-request) sets;
//! 6. read 256 16-bit words from the data port into the buffer.
//!
//! ## The write protocol (28-bit LBA, WRITE SECTORS = 0x30)
//!
//! Steps 1–4 are identical except the command is WRITE SECTORS, then:
//!
//! 5. poll until BSY clears and DRQ sets — now the drive wants the data *from* us;
//! 6. write 256 16-bit words to the data port (the sector's bytes);
//! 7. poll until BSY clears — the drive has committed the sector to its buffer;
//! 8. issue **CACHE FLUSH** (0xE7) and poll again — this forces the drive to push
//!    its write cache to the media, so the write is durable. Skipping the flush is
//!    the classic way to "successfully" write data that silently never lands.
//!
//! ## Disks: the boot image vs. a scratch disk
//!
//! `bootimage` hands QEMU the kernel as `-drive format=raw,file=...`, whose default
//! interface is legacy IDE — so the boot disk *is* the primary master here. Reading its
//! sector 0 returns the boot sector, whose final two bytes are the MBR boot signature
//! `0x55 0xAA` (what [`read_sector`] verifies at boot).
//!
//! Writing to the boot disk would corrupt the kernel, so writes target a **separate
//! scratch disk** attached as the primary *slave* (`Cargo.toml`'s `run-args`/`test-args`
//! add `-drive ...,if=ide,index=1`; `build.rs` creates the backing `scratch.img`). The
//! `Drive` argument names which drive an operation touches, so the boot image is never
//! at risk from a stray write.
//!
//! Scope: one sector at a time, primary bus only. A real driver would IDENTIFY each
//! drive, support multi-sector transfers, and serialize access behind a lock; here a
//! single caller drives the disk at boot, so the functions are left stateless.

use x86_64::instructions::port::Port;

/// A disk sector is 512 bytes.
pub const SECTOR_SIZE: usize = 512;

// Legacy primary ATA bus ports.
const IO_BASE: u16 = 0x1F0;
const CTRL_BASE: u16 = 0x3F6;

// Command-block register addresses (offsets from `IO_BASE`).
const REG_DATA: u16 = IO_BASE; // +0: 16-bit data port (PIO transfer)
const REG_SECTOR_COUNT: u16 = IO_BASE + 2;
const REG_LBA_LOW: u16 = IO_BASE + 3;
const REG_LBA_MID: u16 = IO_BASE + 4;
const REG_LBA_HIGH: u16 = IO_BASE + 5;
const REG_DRIVE: u16 = IO_BASE + 6;
const REG_STATUS_CMD: u16 = IO_BASE + 7; // status (read) / command (write)

// Status register bits.
const ST_ERR: u8 = 1 << 0; // an error occurred
const ST_DRQ: u8 = 1 << 3; // data request: a word can be transferred
const ST_BSY: u8 = 1 << 7; // drive busy; the other bits are meaningless while set

// Commands (PIO, 28-bit LBA).
const CMD_READ_SECTORS: u8 = 0x20;
const CMD_WRITE_SECTORS: u8 = 0x30;
// FLUSH CACHE: tell the drive to commit its write cache to the media. Issued after a
// write so the data is durable rather than sitting in a volatile cache.
const CMD_CACHE_FLUSH: u8 = 0xE7;

/// Device-control register bit nIEN ("not interrupt enable"). Setting it stops the drive
/// from asserting its IRQ (IRQ14 on the primary bus). A polled driver wants this: we have
/// no IRQ14 handler, and an unhandled ATA interrupt (vector 46) would cascade through a
/// not-present IDT gate (#NP) into a double fault.
const DEV_CTRL_NIEN: u8 = 1 << 1;

// Drive-select base bytes ("LBA mode, drive N"): bits 7 and 5 are obsolete-but-set, bit 6
// selects LBA addressing, bit 4 selects the drive (0 = master, 1 = slave). The low nibble
// carries LBA bits 24..28, OR'd in per operation.
const DRIVE_LBA_MASTER: u8 = 0xE0;
const DRIVE_LBA_SLAVE: u8 = 0xF0;

/// Which drive on the primary bus an operation targets.
///
/// The boot image is the master; the scratch disk (safe to write) is the slave. Naming
/// the drive at each call site is what keeps a write from ever reaching the boot image.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Drive {
    /// The primary master — the boot disk here. Read-only in practice.
    PrimaryMaster,
    /// The primary slave — the scratch disk used for write experiments.
    PrimarySlave,
}

impl Drive {
    /// The drive-select base byte (before the top LBA nibble is OR'd in).
    fn select_base(self) -> u8 {
        match self {
            Drive::PrimaryMaster => DRIVE_LBA_MASTER,
            Drive::PrimarySlave => DRIVE_LBA_SLAVE,
        }
    }
}

// A generous polling bound. QEMU answers within a handful of reads; a missing or faulty
// drive then times out instead of hanging the kernel forever.
const POLL_LIMIT: u32 = 1_000_000;

/// What can go wrong with a PIO transfer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AtaError {
    /// The drive never reached the expected state within [`POLL_LIMIT`] polls.
    Timeout,
    /// The drive set the ERR status bit.
    DriveError,
}

/// Read the alternate-status register four times to burn ~400 ns. After a drive select or
/// a command the status bits need a moment to settle; reading `0x3F6` samples the status
/// with no side effects, the canonical way to wait.
unsafe fn delay_400ns() {
    let mut alt_status: Port<u8> = Port::new(CTRL_BASE);
    for _ in 0..4 {
        // SAFETY: 0x3F6 is the fixed alternate-status port; a read only returns the
        // status byte and has no side effects.
        let _ = alt_status.read();
    }
}

/// Spin until the drive clears BSY, or time out. Used before issuing a command, when ERR
/// is not yet meaningful; [`wait_ready`] is the variant that also checks ERR.
unsafe fn wait_while_busy() -> Result<(), AtaError> {
    let mut status: Port<u8> = Port::new(REG_STATUS_CMD);
    for _ in 0..POLL_LIMIT {
        // SAFETY: 0x1F7 read returns the status byte with no side effects.
        if status.read() & ST_BSY == 0 {
            return Ok(());
        }
    }
    Err(AtaError::Timeout)
}

/// Spin until the drive is idle (BSY clear) after a command, surfacing a drive-reported
/// failure as `DriveError`. Used to wait out a write and a cache flush, where the drive
/// can report an error but does not raise DRQ.
unsafe fn wait_ready() -> Result<(), AtaError> {
    let mut status: Port<u8> = Port::new(REG_STATUS_CMD);
    for _ in 0..POLL_LIMIT {
        // SAFETY: 0x1F7 read returns the status byte with no side effects.
        let s = status.read();
        if s & ST_BSY == 0 {
            if s & ST_ERR != 0 {
                return Err(AtaError::DriveError);
            }
            return Ok(());
        }
    }
    Err(AtaError::Timeout)
}

/// Spin until the drive has a word of data to transfer: BSY clear and DRQ set. Returns
/// `DriveError` if the drive raises ERR meanwhile, `Timeout` if neither happens in time.
/// On a read this means "the sector is ready to pull"; on a write, "the drive is ready to
/// accept the sector".
unsafe fn wait_for_data() -> Result<(), AtaError> {
    let mut status: Port<u8> = Port::new(REG_STATUS_CMD);
    for _ in 0..POLL_LIMIT {
        // SAFETY: 0x1F7 read returns the status byte with no side effects.
        let s = status.read();
        if s & ST_ERR != 0 {
            return Err(AtaError::DriveError);
        }
        if s & ST_BSY == 0 && s & ST_DRQ != 0 {
            return Ok(());
        }
    }
    Err(AtaError::Timeout)
}

/// Select `drive`, program the LBA28 address and a single-sector count, and issue `cmd`.
/// Shared prologue of both the read and write paths: everything up to (but not including)
/// the data transfer. On return the command is in flight; the caller polls for the drive's
/// response. `cmd` must be one of the LBA28 PIO commands above.
///
/// # Safety
/// Performs raw port I/O to the fixed legacy primary-ATA registers. Sound to call from the
/// kernel because nothing else touches these ports concurrently (the kernel is the sole,
/// single-threaded driver here) and the sequence is the architectural command setup.
unsafe fn issue_command(drive: Drive, lba: u32, cmd: u8) -> Result<(), AtaError> {
    let mut sector_count: Port<u8> = Port::new(REG_SECTOR_COUNT);
    let mut lba_low: Port<u8> = Port::new(REG_LBA_LOW);
    let mut lba_mid: Port<u8> = Port::new(REG_LBA_MID);
    let mut lba_high: Port<u8> = Port::new(REG_LBA_HIGH);
    let mut drive_reg: Port<u8> = Port::new(REG_DRIVE);
    let mut command: Port<u8> = Port::new(REG_STATUS_CMD);
    let mut control: Port<u8> = Port::new(CTRL_BASE);

    wait_while_busy()?;

    // Polled driver: disable the drive's interrupt (nIEN) so completing the command does
    // not assert IRQ14. We have no IRQ14 handler, and an unhandled ATA interrupt would
    // cascade (vector 46 -> not-present gate -> #NP -> double fault).
    control.write(DEV_CTRL_NIEN);

    // Select the drive in LBA mode; LBA bits 24..28 go in the low nibble.
    drive_reg.write(drive.select_base() | (((lba >> 24) & 0x0F) as u8));
    delay_400ns(); // let the drive selection settle

    sector_count.write(1); // a single sector
    lba_low.write((lba & 0xFF) as u8);
    lba_mid.write(((lba >> 8) & 0xFF) as u8);
    lba_high.write(((lba >> 16) & 0xFF) as u8);

    command.write(cmd);
    delay_400ns(); // let BSY assert before the caller starts polling
    Ok(())
}

/// Read one 512-byte sector at logical block address `lba` from `drive` into the first
/// [`SECTOR_SIZE`] bytes of `buf`, using 28-bit LBA PIO.
///
/// Issue READ SECTORS, poll until the drive signals data-ready, then pull the sector in as
/// 256 little-endian 16-bit words. A pure read, so this is a safe function — the unsafe
/// port I/O is encapsulated, and every access is a standard, well-defined ATA register
/// access.
///
/// `buf` must be at least [`SECTOR_SIZE`] bytes. Callers should pass a *heap* buffer (e.g.
/// `vec![0u8; SECTOR_SIZE]`): a 512-byte array on the small kernel boot stack can overflow
/// it into the guard page.
pub fn read_sector_from(drive: Drive, lba: u32, buf: &mut [u8]) -> Result<(), AtaError> {
    assert!(
        buf.len() >= SECTOR_SIZE,
        "read_sector_from: buffer must hold at least one 512-byte sector"
    );

    let mut data: Port<u16> = Port::new(REG_DATA);

    // SAFETY: every port accessed is a fixed, standard legacy-ATA primary-bus register, and
    // the sequence is the architectural READ SECTORS (LBA28) protocol; the reads only
    // sample status or pull sector data. Nothing here aliases memory or disturbs another
    // device, and every poll is bounded, so a missing/faulty drive times out rather than
    // hanging. We read exactly 256 words = 512 bytes into `buf`, whose length was checked
    // to be at least one sector above.
    unsafe {
        issue_command(drive, lba, CMD_READ_SECTORS)?;

        // Wait for the drive to load the sector into its buffer and raise DRQ.
        wait_for_data()?;

        // Transfer 256 little-endian 16-bit words into the byte buffer.
        for i in 0..(SECTOR_SIZE / 2) {
            let word = data.read();
            buf[i * 2] = (word & 0xFF) as u8;
            buf[i * 2 + 1] = (word >> 8) as u8;
        }
    }

    Ok(())
}

/// Read one sector from the primary master (the boot disk). Convenience wrapper over
/// [`read_sector_from`] for the common, read-only case.
pub fn read_sector(lba: u32, buf: &mut [u8]) -> Result<(), AtaError> {
    read_sector_from(Drive::PrimaryMaster, lba, buf)
}

/// Write the first [`SECTOR_SIZE`] bytes of `buf` to logical block address `lba` on
/// `drive`, using 28-bit LBA PIO, then flush the drive's write cache so the data is
/// durable.
///
/// Issue WRITE SECTORS, poll until the drive is ready to accept data, push the sector out
/// as 256 little-endian 16-bit words (the exact inverse of the read packing, so a write
/// then read-back round-trips identically), wait for the drive to commit it, then issue
/// CACHE FLUSH and wait for that too.
///
/// The drive is named explicitly (there is no "default" target) because a write to the
/// wrong drive could corrupt the boot image. Pass [`Drive::PrimarySlave`] — the scratch
/// disk — for experiments.
///
/// `buf` must be at least [`SECTOR_SIZE`] bytes; like the read path, prefer a heap buffer.
pub fn write_sector(drive: Drive, lba: u32, buf: &[u8]) -> Result<(), AtaError> {
    assert!(
        buf.len() >= SECTOR_SIZE,
        "write_sector: buffer must hold at least one 512-byte sector"
    );

    let mut data: Port<u16> = Port::new(REG_DATA);
    let mut command: Port<u8> = Port::new(REG_STATUS_CMD);

    // SAFETY: every port accessed is a fixed, standard legacy-ATA primary-bus register, and
    // the sequence is the architectural WRITE SECTORS (LBA28) protocol followed by CACHE
    // FLUSH; the writes only program registers and push sector data, the reads only sample
    // status. Nothing here aliases memory, and every poll is bounded, so a missing/faulty
    // drive times out rather than hanging. We write exactly 256 words = 512 bytes from
    // `buf`, whose length was checked to be at least one sector above. The caller chose
    // `drive`, so the boot image is touched only if it explicitly asked for the master.
    unsafe {
        issue_command(drive, lba, CMD_WRITE_SECTORS)?;

        // Wait until the drive is ready to receive the sector (BSY clear, DRQ set).
        wait_for_data()?;

        // Transfer 256 little-endian 16-bit words out of the byte buffer. Deliberately a
        // plain word-at-a-time loop, not a `rep outsw` burst: ATA wants a brief gap between
        // words, which the per-iteration overhead naturally provides.
        for i in 0..(SECTOR_SIZE / 2) {
            let word = (buf[i * 2] as u16) | ((buf[i * 2 + 1] as u16) << 8);
            data.write(word);
        }

        // The drive now writes the sector out of its buffer; wait for it to finish.
        wait_ready()?;

        // Flush the write cache to the media so the data survives a power loss, then wait
        // for the flush to complete.
        command.write(CMD_CACHE_FLUSH);
        delay_400ns();
        wait_ready()?;
    }

    Ok(())
}
