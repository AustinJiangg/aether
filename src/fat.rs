//! Stage 14b: a read-only FAT16 filesystem driver, built on the Stage 13 ATA block driver.
//!
//! FAT (File Allocation Table) is the filesystem of DOS and of nearly every USB stick and
//! SD card. It is simple enough to implement by hand, yet real: the `fat.img` this reads is
//! produced by the host's `mkfs.fat`, so the kernel is parsing the exact on-disk layout a
//! PC has used for forty years.
//!
//! ## On-disk layout
//!
//! A FAT volume is a sequence of 512-byte sectors in four regions, back to back:
//!
//! ```text
//!   [ reserved ][ FAT(s) ][ root directory ][ data (clusters) ]
//!      ^boot sector (BPB)            ^fixed-size on FAT12/16     ^file & subdir contents
//! ```
//!
//! - **Reserved**: starts with the *boot sector*, whose **BPB** (BIOS Parameter Block) holds
//!   the geometry — sector size, cluster size, how many FATs, how big they are, and so on.
//!   Everything else is computed from those numbers. This module (Stage 14b-1) parses the
//!   BPB; the FAT walk and directory reading come next (14b-2).
//! - **FAT**: an array of cluster entries forming linked lists — each entry says "the next
//!   cluster of this file" or "end of chain". On FAT16 each entry is a little-endian `u16`.
//! - **Root directory**: a fixed-size array of 32-byte directory entries (8.3 names).
//! - **Data**: the file/subdirectory contents, addressed in *clusters* (groups of sectors).
//!   Cluster numbering starts at 2, so cluster 2 is the first data cluster.
//!
//! ## What this module reads
//!
//! The FAT disk is the secondary master ([`ata::Drive::SecondaryMaster`]); the boot image
//! and the raw scratch disk are untouched. Read-only for now: enough to find and read a file
//! a host tool wrote.

use crate::ata::{self, AtaError, Drive};

use alloc::vec;

/// A FAT volume's boot signature lives in the last two bytes of the boot sector.
const BOOT_SIGNATURE: [u8; 2] = [0x55, 0xAA];

/// The only sector size this driver supports (and the only one real FAT volumes use).
const SECTOR_SIZE: usize = 512;

/// What can go wrong bringing up a FAT volume.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FatError {
    /// The underlying block read failed.
    Io(AtaError),
    /// The boot sector did not end with the `0x55 0xAA` signature — not a formatted volume.
    BadSignature,
    /// The BPB reports a sector size this driver does not handle (we only do 512).
    UnsupportedSectorSize(u16),
    /// The geometry works out to a cluster count outside the FAT16 range; this minimal
    /// driver only handles FAT16.
    NotFat16,
}

impl From<AtaError> for FatError {
    fn from(e: AtaError) -> Self {
        FatError::Io(e)
    }
}

/// Read a little-endian `u16` from `buf` at `off`.
fn read_u16(buf: &[u8], off: usize) -> u16 {
    u16::from_le_bytes([buf[off], buf[off + 1]])
}

/// Read a little-endian `u32` from `buf` at `off`.
fn read_u32(buf: &[u8], off: usize) -> u32 {
    u32::from_le_bytes([buf[off], buf[off + 1], buf[off + 2], buf[off + 3]])
}

/// The geometry parsed out of a FAT boot sector's BPB, plus the region offsets derived from
/// it. Everything the rest of the driver needs to locate the FAT, the root directory, and
/// the data clusters.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Bpb {
    /// Bytes per sector (always 512 here).
    pub bytes_per_sector: u16,
    /// Sectors per cluster (a power of two; 1 on our test image).
    pub sectors_per_cluster: u8,
    /// Reserved sectors before the first FAT (the boot sector lives here).
    pub reserved_sectors: u16,
    /// How many copies of the FAT (usually 2, for redundancy).
    pub num_fats: u8,
    /// Number of 32-byte entries in the fixed-size root directory.
    pub root_entry_count: u16,
    /// Total sectors in the volume.
    pub total_sectors: u32,
    /// Sectors occupied by one FAT.
    pub fat_size_sectors: u32,
}

impl Bpb {
    /// Parse a 512-byte boot sector into a [`Bpb`], validating the signature, the sector
    /// size, and that the geometry is FAT16. Pure (no I/O), so it is unit-testable on a
    /// hand-built buffer. `sector` must be at least [`SECTOR_SIZE`] bytes.
    pub fn parse(sector: &[u8]) -> Result<Bpb, FatError> {
        assert!(sector.len() >= SECTOR_SIZE, "boot sector buffer too small");

        // The two-byte boot signature must terminate the sector.
        if sector[510..512] != BOOT_SIGNATURE {
            return Err(FatError::BadSignature);
        }

        let bytes_per_sector = read_u16(sector, 0x0B);
        if bytes_per_sector as usize != SECTOR_SIZE {
            return Err(FatError::UnsupportedSectorSize(bytes_per_sector));
        }

        // Total sectors lives in the 16-bit field, or — if that is zero (volume > 65535
        // sectors) — in the 32-bit field.
        let total_sectors_16 = read_u16(sector, 0x13);
        let total_sectors_32 = read_u32(sector, 0x20);
        let total_sectors = if total_sectors_16 != 0 {
            total_sectors_16 as u32
        } else {
            total_sectors_32
        };

        let bpb = Bpb {
            bytes_per_sector,
            sectors_per_cluster: sector[0x0D],
            reserved_sectors: read_u16(sector, 0x0E),
            num_fats: sector[0x10],
            root_entry_count: read_u16(sector, 0x11),
            total_sectors,
            fat_size_sectors: read_u16(sector, 0x16) as u32,
        };

        // FAT type is defined by the cluster count, not by any stored field. We only handle
        // FAT16.
        if bpb.count_of_clusters() < 4085 || bpb.count_of_clusters() >= 65525 {
            return Err(FatError::NotFat16);
        }

        Ok(bpb)
    }

    /// Sectors spanned by the fixed-size root directory (rounded up to a whole sector).
    pub fn root_dir_sectors(&self) -> u32 {
        let bytes = self.root_entry_count as u32 * 32;
        (bytes + self.bytes_per_sector as u32 - 1) / self.bytes_per_sector as u32
    }

    /// LBA of the first FAT (the FAT region begins right after the reserved sectors).
    pub fn fat_start_sector(&self) -> u32 {
        self.reserved_sectors as u32
    }

    /// LBA of the root directory (after the reserved sectors and all FAT copies).
    pub fn root_dir_start_sector(&self) -> u32 {
        self.reserved_sectors as u32 + self.num_fats as u32 * self.fat_size_sectors
    }

    /// LBA of the first data cluster (cluster 2). Everything after the root directory.
    pub fn data_start_sector(&self) -> u32 {
        self.root_dir_start_sector() + self.root_dir_sectors()
    }

    /// Number of data clusters in the volume — the value that defines the FAT type.
    ///
    /// Uses a saturating subtraction so a bogus boot sector (one that happens to carry the
    /// `0x55 0xAA` signature but nonsense geometry) yields 0 clusters and is rejected as
    /// `NotFat16`, rather than underflowing.
    pub fn count_of_clusters(&self) -> u32 {
        let data_sectors = self.total_sectors.saturating_sub(self.data_start_sector());
        // sectors_per_cluster is validated as a power of two >= 1 for real volumes; guard
        // against a zero from a malformed BPB to avoid a divide-by-zero.
        let spc = self.sectors_per_cluster.max(1) as u32;
        data_sectors / spc
    }
}

/// Read and parse the BPB of the FAT volume on `drive`. Reads sector 0 over the ATA driver,
/// then hands the bytes to [`Bpb::parse`].
pub fn read_bpb(drive: Drive) -> Result<Bpb, FatError> {
    let mut sector = vec![0u8; SECTOR_SIZE];
    ata::read_sector_from(drive, 0, &mut sector)?;
    Bpb::parse(&sector)
}
