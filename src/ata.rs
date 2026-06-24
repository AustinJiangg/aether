//! Stage 13a: a minimal ATA (IDE) disk driver in PIO mode — reading raw sectors.
//!
//! This is the first step of persistence: getting bytes off a real disk. We use the
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
//!   0x1F0  Data            16-bit; 256 reads transfer one 512-byte sector
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
//! ## The read protocol (28-bit LBA, READ SECTORS = 0x20)
//!
//! 1. wait until the drive is not BSY;
//! 2. write the drive-select byte (master, LBA mode, top 4 LBA bits);
//! 3. write the sector count and the low 24 LBA bits;
//! 4. write the READ SECTORS command;
//! 5. poll the status register until BSY clears and DRQ (data-request) sets;
//! 6. read 256 16-bit words from the data port into the buffer.
//!
//! ## Why this works without any QEMU configuration
//!
//! `bootimage` hands QEMU the kernel as `-drive format=raw,file=...`, whose default
//! interface is legacy IDE — so the boot disk *is* the primary master here. Reading its
//! sector 0 returns the boot sector, whose final two bytes are the MBR boot signature
//! `0x55 0xAA`: a stable value to verify against, with no file-system layout assumed.
//!
//! Scope: read-only, one sector at a time, primary master only. Writing (which needs a
//! scratch disk so we never corrupt the boot image) is the next step (13b). A real driver
//! would also serialize access behind a lock; here a single caller reads at boot, so the
//! functions are left stateless.

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

// READ SECTORS (PIO, 28-bit LBA).
const CMD_READ_SECTORS: u8 = 0x20;

/// Device-control register bit nIEN ("not interrupt enable"). Setting it stops the drive
/// from asserting its IRQ (IRQ14 on the primary bus). A polled driver wants this: we have
/// no IRQ14 handler, and an unhandled ATA interrupt (vector 46) would cascade through a
/// not-present IDT gate (#NP) into a double fault.
const DEV_CTRL_NIEN: u8 = 1 << 1;

// Drive-select byte for "master, LBA mode": bits 7 and 5 are obsolete-but-set, bit 6
// selects LBA addressing, bit 4 = 0 selects the master drive. The low nibble carries
// LBA bits 24..28.
const DRIVE_MASTER_LBA: u8 = 0xE0;

// A generous polling bound. QEMU answers within a handful of reads; a missing or faulty
// drive then times out instead of hanging the kernel forever.
const POLL_LIMIT: u32 = 1_000_000;

/// What can go wrong with a PIO read.
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

/// Spin until the drive clears BSY, or time out.
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

/// Spin until the drive has data ready: BSY clear and DRQ set. Returns `DriveError` if the
/// drive raises ERR meanwhile, `Timeout` if neither happens in time.
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

/// Read one 512-byte sector at logical block address `lba` from the primary master drive
/// into the first [`SECTOR_SIZE`] bytes of `buf`, using 28-bit LBA PIO.
///
/// The whole ATA PIO read dance: select the drive and feed it the LBA, issue READ
/// SECTORS, poll until the drive signals data-ready, then pull the sector in as 256
/// little-endian 16-bit words. A pure read, so this is a safe function — the unsafe port
/// I/O is encapsulated, and every access is a standard, well-defined ATA register access.
///
/// `buf` must be at least [`SECTOR_SIZE`] bytes. Callers should pass a *heap* buffer (e.g.
/// `vec![0u8; SECTOR_SIZE]`): a 512-byte array on the small kernel boot stack can overflow
/// it into the guard page, and with no page-fault handler that escalates to a double fault.
pub fn read_sector(lba: u32, buf: &mut [u8]) -> Result<(), AtaError> {
    assert!(
        buf.len() >= SECTOR_SIZE,
        "read_sector: buffer must hold at least one 512-byte sector"
    );

    let mut data: Port<u16> = Port::new(REG_DATA);
    let mut sector_count: Port<u8> = Port::new(REG_SECTOR_COUNT);
    let mut lba_low: Port<u8> = Port::new(REG_LBA_LOW);
    let mut lba_mid: Port<u8> = Port::new(REG_LBA_MID);
    let mut lba_high: Port<u8> = Port::new(REG_LBA_HIGH);
    let mut drive: Port<u8> = Port::new(REG_DRIVE);
    let mut command: Port<u8> = Port::new(REG_STATUS_CMD);
    let mut control: Port<u8> = Port::new(CTRL_BASE);

    // SAFETY: every port below is a fixed, standard legacy-ATA primary-bus register, and
    // the write sequence is the architectural READ SECTORS (LBA28) protocol; the reads
    // only sample status or pull sector data. Nothing here aliases memory or disturbs
    // another device, and every poll is bounded, so a missing/faulty drive times out
    // rather than hanging. We write exactly 256 words = 512 bytes into `buf`, whose length
    // was checked to be at least one sector above.
    unsafe {
        wait_while_busy()?;

        // Polled driver: disable the drive's interrupt (nIEN) so completing the read does
        // not assert IRQ14. We have no IRQ14 handler, and an unhandled ATA interrupt would
        // cascade (vector 46 -> not-present gate -> #NP -> double fault).
        control.write(DEV_CTRL_NIEN);

        // Select the master drive in LBA mode; LBA bits 24..28 go in the low nibble.
        drive.write(DRIVE_MASTER_LBA | (((lba >> 24) & 0x0F) as u8));
        delay_400ns(); // let the drive selection settle

        sector_count.write(1); // a single sector
        lba_low.write((lba & 0xFF) as u8);
        lba_mid.write(((lba >> 8) & 0xFF) as u8);
        lba_high.write(((lba >> 16) & 0xFF) as u8);

        command.write(CMD_READ_SECTORS);
        delay_400ns(); // let BSY assert before we start polling

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
