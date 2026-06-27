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
//!   Everything else is computed from those numbers. Stage 14b-1 parsed the BPB; Stage
//!   14b-2 adds the FAT walk and directory reading on top, so a file can be read by name.
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
use crate::fs::{FileSystem, FsError};

use alloc::string::String;
use alloc::vec;
use alloc::vec::Vec;

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
    /// No directory entry matched the requested name.
    NotFound,
    /// The name resolves to a subdirectory, not a file.
    IsDirectory,
    /// The cluster chain is malformed: a free/bad cluster appears mid-file, or the chain
    /// never reaches an end-of-chain marker (so it would loop forever).
    BadChain,
    /// The volume has no free cluster left to allocate (the disk is full).
    NoSpace,
    /// The root directory has no free entry left for a new file.
    DirFull,
    /// The requested name does not fit FAT's 8.3 short-name form (empty, an over-long base or
    /// extension, more than one `.`, or a disallowed character).
    InvalidName,
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

// ---------------------------------------------------------------------------
// Stage 14b-2: reading a file — the root-directory scan and the FAT cluster walk.
// ---------------------------------------------------------------------------

/// A FAT directory entry is exactly 32 bytes; a 512-byte sector holds 16 of them.
const DIR_ENTRY_SIZE: usize = 32;

// Byte offsets within a 32-byte directory entry.
const ENTRY_ATTR_OFFSET: usize = 0x0B; // attribute byte
const ENTRY_FIRST_CLUSTER_LO_OFFSET: usize = 0x1A; // low 16 bits of the start cluster
const ENTRY_SIZE_OFFSET: usize = 0x1C; // file size in bytes (u32)

// Sentinel values for the first name byte of a directory entry.
const NAME_END: u8 = 0x00; // entry free AND no entry after it is used: end of the directory
const NAME_DELETED: u8 = 0xE5; // entry free (a deleted file): skip it, keep scanning

// Directory-entry attribute bits we test.
const ATTR_VOLUME_ID: u8 = 0x08; // the volume-label entry, not a real file
const ATTR_DIRECTORY: u8 = 0x10; // a subdirectory rather than a file
const ATTR_LONG_NAME: u8 = 0x0F; // a long-file-name fragment (RO|HIDDEN|SYSTEM|VOLUME): skip

/// FAT16 cluster values `2..=0xFFEF` address real data clusters. `0` means free, `1` is
/// reserved, `0xFFF0..=0xFFF6` are reserved, `0xFFF7` is a bad cluster, and `0xFFF8..=0xFFFF`
/// mark the end of a chain. So a value is "another data cluster to follow" iff it is in range.
fn is_data_cluster(cluster: u16) -> bool {
    (2..=0xFFEF).contains(&cluster)
}

/// Drop the trailing space padding from one fixed-width name field (the 8-byte base or the
/// 3-byte extension of an 8.3 name).
fn trim_trailing_spaces(field: &[u8]) -> &[u8] {
    match field.iter().rposition(|&b| b != b' ') {
        Some(last) => &field[..=last],
        None => &[], // all spaces (e.g. a no-extension file): an empty field
    }
}

/// Turn the 11-byte, space-padded 8.3 name from a directory entry into a normal string, e.g.
/// `b"HELLO   TXT"` -> `"HELLO.TXT"` and `b"README     "` -> `"README"`. Only ASCII names are
/// handled (each byte mapped straight to a `char`), which is all our host tools produce.
fn short_name_to_string(raw: &[u8]) -> String {
    let base = trim_trailing_spaces(&raw[0..8]);
    let ext = trim_trailing_spaces(&raw[8..11]);

    let mut name = String::new();
    for &b in base {
        name.push(b as char);
    }
    if !ext.is_empty() {
        name.push('.');
        for &b in ext {
            name.push(b as char);
        }
    }
    name
}

/// The fields of a located directory entry that the reader cares about.
struct RootEntry {
    /// First cluster of the file's data (its head in the FAT chain).
    first_cluster: u16,
    /// File length in bytes, from the directory entry.
    size: u32,
    /// Whether this entry is a subdirectory rather than a regular file.
    is_dir: bool,
}

/// A mounted, read-only FAT16 volume: the drive it lives on plus its parsed geometry
/// ([`Bpb`]). Construct one with [`Fat::mount`], then read files with [`Fat::read_file`].
pub struct Fat {
    drive: Drive,
    bpb: Bpb,
}

impl Fat {
    /// Mount the FAT volume on `drive` by reading and parsing its boot sector (the BPB).
    pub fn mount(drive: Drive) -> Result<Fat, FatError> {
        let bpb = read_bpb(drive)?;
        Ok(Fat { drive, bpb })
    }

    /// The volume's parsed geometry (sector/cluster sizes and the region start LBAs).
    pub fn bpb(&self) -> &Bpb {
        &self.bpb
    }

    /// The LBA of the first sector of data cluster `cluster` (which must be >= 2). Cluster
    /// numbering starts at 2, so cluster 2 maps to the very start of the data region.
    fn cluster_lba(&self, cluster: u16) -> u32 {
        self.bpb.data_start_sector() + (cluster as u32 - 2) * self.bpb.sectors_per_cluster as u32
    }

    /// Look up the FAT entry for `cluster`: the next cluster in the chain, or an
    /// end-of-chain marker (>= 0xFFF8). On FAT16 each entry is a little-endian `u16`, so the
    /// entry for cluster N lives at byte offset `N * 2` into the FAT region. 512 is even and
    /// entries are 2 bytes wide, so an entry never straddles a sector boundary.
    ///
    /// This reads a fresh FAT sector on every call — simple, and fine for the tiny files
    /// here; a real driver would cache the FAT.
    fn next_cluster(&self, cluster: u16) -> Result<u16, FatError> {
        let fat_offset = cluster as u32 * 2; // 2 bytes per FAT16 entry
        let sector = self.bpb.fat_start_sector() + fat_offset / SECTOR_SIZE as u32;
        let offset = (fat_offset % SECTOR_SIZE as u32) as usize;

        let mut buf = vec![0u8; SECTOR_SIZE];
        ata::read_sector_from(self.drive, sector, &mut buf)?;
        Ok(read_u16(&buf, offset))
    }

    /// Scan the fixed-size root directory, calling `visit` with the formatted 8.3 name and
    /// the raw 32-byte entry of each in-use file/subdirectory entry — skipping free, deleted,
    /// long-file-name, and volume-label entries. If `visit` returns `Some`, scanning stops
    /// and yields that value; otherwise it runs to the end of the directory and yields `None`.
    /// The shared core of [`find_root_entry`] (search) and [`list_root`] (collect).
    fn scan_root<T>(
        &self,
        mut visit: impl FnMut(&str, &[u8]) -> Option<T>,
    ) -> Result<Option<T>, FatError> {
        let start = self.bpb.root_dir_start_sector();
        let sectors = self.bpb.root_dir_sectors();
        let mut buf = vec![0u8; SECTOR_SIZE];

        for s in 0..sectors {
            ata::read_sector_from(self.drive, start + s, &mut buf)?;

            for e in 0..(SECTOR_SIZE / DIR_ENTRY_SIZE) {
                let entry = &buf[e * DIR_ENTRY_SIZE..(e + 1) * DIR_ENTRY_SIZE];

                match entry[0] {
                    // A 0x00 first byte means this entry is free and so is every entry after
                    // it: the directory ends here.
                    NAME_END => return Ok(None),
                    // A deleted (free) entry — skip it and keep scanning.
                    NAME_DELETED => continue,
                    _ => {}
                }

                let attr = entry[ENTRY_ATTR_OFFSET];
                if attr == ATTR_LONG_NAME || attr & ATTR_VOLUME_ID != 0 {
                    continue; // skip LFN fragments and the volume label
                }

                if let Some(value) = visit(&short_name_to_string(&entry[0..11]), entry) {
                    return Ok(Some(value));
                }
            }
        }
        Ok(None)
    }

    /// Find the root-directory entry named `name` (8.3, ASCII, case-insensitive), or `None`
    /// if no entry matches. Returns its start cluster, size, and whether it is a subdirectory.
    fn find_root_entry(&self, name: &str) -> Result<Option<RootEntry>, FatError> {
        self.scan_root(|entry_name, entry| {
            if entry_name.eq_ignore_ascii_case(name) {
                Some(RootEntry {
                    first_cluster: read_u16(entry, ENTRY_FIRST_CLUSTER_LO_OFFSET),
                    size: read_u32(entry, ENTRY_SIZE_OFFSET),
                    is_dir: entry[ENTRY_ATTR_OFFSET] & ATTR_DIRECTORY != 0,
                })
            } else {
                None
            }
        })
    }

    /// List the root directory as `(name, is_dir)` pairs — every in-use entry it holds.
    fn list_root(&self) -> Result<Vec<(String, bool)>, FatError> {
        let mut entries = Vec::new();
        // `visit` always returns `None`, so the scan runs over the whole directory.
        self.scan_root(|name, entry| -> Option<()> {
            let is_dir = entry[ENTRY_ATTR_OFFSET] & ATTR_DIRECTORY != 0;
            entries.push((String::from(name), is_dir));
            None
        })?;
        Ok(entries)
    }

    /// Follow the FAT cluster chain from `first_cluster`, reading cluster contents until
    /// `size` bytes are gathered (or the chain ends), then truncate to `size` — the final
    /// cluster is usually only partly used. A `size` of 0 (an empty file, whose start cluster
    /// is 0) yields an empty vector.
    fn read_chain(&self, first_cluster: u16, size: usize) -> Result<Vec<u8>, FatError> {
        let mut data = Vec::with_capacity(size);
        let mut buf = vec![0u8; SECTOR_SIZE];

        // A file cannot span more clusters than the volume has; following more than that
        // means the chain is circular or otherwise corrupt, so we bail instead of hanging.
        let max_steps = self.bpb.count_of_clusters() as usize + 2;
        let mut cluster = first_cluster;
        let mut steps = 0usize;

        while is_data_cluster(cluster) {
            steps += 1;
            if steps > max_steps {
                return Err(FatError::BadChain);
            }

            // Read every sector of this cluster into the output buffer.
            let lba = self.cluster_lba(cluster);
            for s in 0..self.bpb.sectors_per_cluster as u32 {
                ata::read_sector_from(self.drive, lba + s, &mut buf)?;
                data.extend_from_slice(&buf);
            }

            // Once we have the whole file there is no need to read further clusters.
            if data.len() >= size {
                break;
            }
            cluster = self.next_cluster(cluster)?;
        }

        if data.len() < size {
            // The chain ended (a free/bad/end-of-chain cluster) before the file's declared
            // size was reached — the directory and FAT disagree, so the volume is corrupt.
            return Err(FatError::BadChain);
        }
        data.truncate(size);
        Ok(data)
    }

    /// Read the file named `name` (8.3, case-insensitive) from the root directory and return
    /// its bytes. Fails with [`FatError::NotFound`] if there is no such entry, or
    /// [`FatError::IsDirectory`] if the name is a subdirectory rather than a file.
    pub fn read_file(&self, name: &str) -> Result<Vec<u8>, FatError> {
        match self.find_root_entry(name)? {
            None => Err(FatError::NotFound),
            Some(entry) if entry.is_dir => Err(FatError::IsDirectory),
            Some(entry) => self.read_chain(entry.first_cluster, entry.size as usize),
        }
    }
}

// ---------------------------------------------------------------------------
// Stage 14b-2b: the FAT volume behind the VFS `FileSystem` trait.
// ---------------------------------------------------------------------------

/// Map a FAT-specific error onto the generic VFS [`FsError`], so [`Fat`] can implement the
/// shared [`FileSystem`] trait. "Not found" and "is a directory" have direct equivalents;
/// every other FAT failure is a device- or format-level fault the VFS reports as
/// [`FsError::Io`].
impl From<FatError> for FsError {
    fn from(e: FatError) -> FsError {
        match e {
            FatError::NotFound => FsError::NotFound,
            FatError::IsDirectory => FsError::IsDir,
            FatError::InvalidName => FsError::Unsupported,
            FatError::Io(_)
            | FatError::BadSignature
            | FatError::UnsupportedSectorSize(_)
            | FatError::NotFat16
            | FatError::BadChain
            | FatError::NoSpace
            | FatError::DirFull => FsError::Io,
        }
    }
}

/// [`Fat`] behind the VFS [`FileSystem`] trait, so it slots in beside [`RamFs`] and the shell
/// (or, later, system calls) can read a disk path without knowing which filesystem backs it.
///
/// This driver currently understands only the **root directory** (no subdirectory traversal),
/// which shapes the implementation:
/// - `read`/`list`/`is_dir` operate on the root and its entries;
/// - `write` creates or overwrites a root-level file (Stage 14c-1);
/// - `mkdir` and `remove` are not supported yet, and a path that descends into a subdirectory
///   yields [`FsError::Unsupported`].
impl FileSystem for Fat {
    fn mkdir(&mut self, _path: &str) -> Result<(), FsError> {
        Err(FsError::Unsupported) // creating subdirectories is not supported
    }

    fn write(&mut self, path: &str, data: &[u8]) -> Result<(), FsError> {
        let mut comps = crate::fs::components(path);
        match (comps.next(), comps.next()) {
            (None, _) => Err(FsError::IsDir),                // cannot write the root itself
            (Some(name), None) => Ok(self.write_file(name, data)?),
            (Some(_), Some(_)) => Err(FsError::Unsupported), // no subdirectory traversal yet
        }
    }

    fn remove(&mut self, _path: &str) -> Result<(), FsError> {
        Err(FsError::Unsupported) // file removal comes in Stage 14c-2
    }

    /// Read a root-level file. The root itself (`/`) is a directory, not a file; a path with
    /// two or more components would need subdirectory traversal this driver does not do yet.
    fn read(&self, path: &str) -> Result<Vec<u8>, FsError> {
        let mut comps = crate::fs::components(path);
        match (comps.next(), comps.next()) {
            (None, _) => Err(FsError::IsDir),            // "/" is the root directory
            (Some(name), None) => Ok(self.read_file(name)?), // FatError -> FsError via `?`
            (Some(_), Some(_)) => Err(FsError::Unsupported), // no subdirectory traversal yet
        }
    }

    /// List the root directory (`/`). Subdirectories are not traversed yet, so any deeper
    /// path is reported as unsupported.
    fn list(&self, path: &str) -> Result<Vec<(String, bool)>, FsError> {
        if crate::fs::components(path).next().is_none() {
            Ok(self.list_root()?)
        } else {
            Err(FsError::Unsupported)
        }
    }

    /// Whether `path` names a directory: the root always is; a single root-level component is
    /// iff its entry is flagged a directory; a deeper path cannot be resolved, so it is not.
    fn is_dir(&self, path: &str) -> bool {
        let mut comps = crate::fs::components(path);
        match (comps.next(), comps.next()) {
            (None, _) => true, // the root directory
            // A lookup error (or a missing entry) reads as "not a directory".
            (Some(name), None) => matches!(self.find_root_entry(name), Ok(Some(e)) if e.is_dir),
            (Some(_), Some(_)) => false,
        }
    }
}

// ---------------------------------------------------------------------------
// Stage 14c-1: writing a file — free-cluster allocation, the FAT chain, and the
// directory entry. Writes reach the FAT disk via the Stage 13b `ata::write_sector`.
// ---------------------------------------------------------------------------

/// FAT16 entry value for a free cluster.
const FAT_ENTRY_FREE: u16 = 0x0000;
/// FAT16 entry value we write to mark the end of a cluster chain.
const FAT_ENTRY_EOC: u16 = 0xFFFF;
/// Directory-entry attribute for a regular file (the "archive" bit).
const ATTR_ARCHIVE: u8 = 0x20;

/// Validate and upper-case one byte of a short name. Allows ASCII letters (upper-cased),
/// digits, and a few safe punctuation characters; rejects everything else (spaces, `.`, `/`,
/// control bytes), so a malformed name cannot corrupt the fixed 8.3 layout of a directory entry.
fn short_name_byte(b: u8) -> Option<u8> {
    match b {
        b'a'..=b'z' => Some(b - (b'a' - b'A')), // upper-case
        b'A'..=b'Z' | b'0'..=b'9' => Some(b),
        b'_' | b'-' | b'~' | b'!' | b'#' | b'$' | b'%' | b'&' | b'(' | b')' => Some(b),
        _ => None,
    }
}

/// Convert a filename like `"hello.txt"` into the 11-byte, space-padded, upper-case 8.3 form a
/// directory entry stores (`b"HELLO   TXT"`), or `None` if it does not fit 8.3: an empty or
/// over-long base (> 8), an over-long extension (> 3), or a disallowed byte. The split is on the
/// *last* `.`, and a `.` left in the base (e.g. `"a.b.c"`) is rejected.
fn string_to_short_name(name: &str) -> Option<[u8; 11]> {
    let (base, ext) = match name.rsplit_once('.') {
        Some((b, e)) => (b, e),
        None => (name, ""),
    };
    if base.is_empty() || base.len() > 8 || ext.len() > 3 || base.contains('.') {
        return None;
    }

    let mut short = [b' '; 11];
    for (i, b) in base.bytes().enumerate() {
        short[i] = short_name_byte(b)?;
    }
    for (i, b) in ext.bytes().enumerate() {
        short[8 + i] = short_name_byte(b)?;
    }
    Some(short)
}

/// A located root-directory slot: where a file's 32-byte entry lives, or where a new one goes.
struct DirSlot {
    /// LBA of the directory sector holding the slot.
    lba: u32,
    /// Byte offset of the 32-byte entry within that sector.
    offset: usize,
    /// For an existing entry: its current first cluster and whether it is a subdirectory.
    /// `None` means this is a *free* slot (the name was not found).
    existing: Option<(u16, bool)>,
}

impl Fat {
    /// Bytes in one cluster (`sectors_per_cluster * bytes_per_sector`).
    fn cluster_size(&self) -> usize {
        self.bpb.sectors_per_cluster as usize * self.bpb.bytes_per_sector as usize
    }

    /// Write `value` into the FAT entry for `cluster`, in *every* FAT copy, so the redundant
    /// FATs stay identical. Read-modify-write per copy: read the entry's sector, patch the two
    /// bytes in place, write it back — preserving the other entries in that sector.
    fn write_fat_entry(&self, cluster: u16, value: u16) -> Result<(), FatError> {
        let fat_offset = cluster as u32 * 2; // 2 bytes per FAT16 entry
        let sector_in_fat = fat_offset / SECTOR_SIZE as u32;
        let offset = (fat_offset % SECTOR_SIZE as u32) as usize;
        let mut buf = vec![0u8; SECTOR_SIZE];

        for copy in 0..self.bpb.num_fats as u32 {
            let sector =
                self.bpb.fat_start_sector() + copy * self.bpb.fat_size_sectors + sector_in_fat;
            ata::read_sector_from(self.drive, sector, &mut buf)?;
            buf[offset] = (value & 0xFF) as u8;
            buf[offset + 1] = (value >> 8) as u8;
            ata::write_sector(self.drive, sector, &buf)?;
        }
        Ok(())
    }

    /// Find a free cluster, claim it by marking it end-of-chain, and return its number, or
    /// `Ok(None)` if the volume is full. Scans the FAT a sector at a time for a `0x0000` (free)
    /// entry. Marking it EOC before returning means a follow-up `alloc_cluster` will not hand
    /// the same cluster out again, so a chain can be built one cluster at a time.
    fn alloc_cluster(&self) -> Result<Option<u16>, FatError> {
        let max_cluster = self.bpb.count_of_clusters() + 1; // highest valid data cluster number
        let entries_per_sector = SECTOR_SIZE / 2;
        let mut buf = vec![0u8; SECTOR_SIZE];

        for s in 0..self.bpb.fat_size_sectors {
            ata::read_sector_from(self.drive, self.bpb.fat_start_sector() + s, &mut buf)?;
            let base = s * entries_per_sector as u32; // first cluster number in this sector
            for i in 0..entries_per_sector {
                let cluster = base + i as u32;
                if cluster < 2 {
                    continue; // clusters 0 and 1 are reserved, never allocatable
                }
                if cluster > max_cluster {
                    return Ok(None); // ran past the last real cluster: the volume is full
                }
                if read_u16(&buf, i * 2) == FAT_ENTRY_FREE {
                    self.write_fat_entry(cluster as u16, FAT_ENTRY_EOC)?;
                    return Ok(Some(cluster as u16));
                }
            }
        }
        Ok(None)
    }

    /// Free every cluster in the chain starting at `first_cluster` (set each FAT entry back to
    /// free), reading each entry's successor *before* clearing it. Bounded against a corrupt
    /// chain, like [`Fat::read_chain`].
    fn free_chain(&self, first_cluster: u16) -> Result<(), FatError> {
        let max_steps = self.bpb.count_of_clusters() as usize + 2;
        let mut cluster = first_cluster;
        let mut steps = 0usize;
        while is_data_cluster(cluster) {
            steps += 1;
            if steps > max_steps {
                return Err(FatError::BadChain);
            }
            let next = self.next_cluster(cluster)?; // read the link before clearing it
            self.write_fat_entry(cluster, FAT_ENTRY_FREE)?;
            cluster = next;
        }
        Ok(())
    }

    /// Allocate a fresh cluster chain holding `data`, write the bytes into it, and return its
    /// first cluster (`0` for empty `data`, which needs no clusters). The final cluster's last
    /// sector is zero-padded. A full disk rolls back the partial allocation and returns
    /// `NoSpace`.
    fn write_chain(&self, data: &[u8]) -> Result<u16, FatError> {
        if data.is_empty() {
            return Ok(0);
        }
        let cluster_size = self.cluster_size();
        let count = (data.len() + cluster_size - 1) / cluster_size;

        // Reserve all the clusters first (each alloc marks the cluster EOC, so it is not handed
        // out twice). If the disk fills up partway, release what we took and report the failure.
        let mut clusters = Vec::with_capacity(count);
        for _ in 0..count {
            match self.alloc_cluster()? {
                Some(c) => clusters.push(c),
                None => {
                    for &c in &clusters {
                        let _ = self.write_fat_entry(c, FAT_ENTRY_FREE);
                    }
                    return Err(FatError::NoSpace);
                }
            }
        }

        // Link the chain: each cluster points to the next; the last keeps its EOC mark.
        for i in 0..count - 1 {
            self.write_fat_entry(clusters[i], clusters[i + 1])?;
        }

        // Write the data, zero-padding the final cluster's last sector.
        let mut buf = vec![0u8; SECTOR_SIZE];
        for (i, &cluster) in clusters.iter().enumerate() {
            let lba = self.cluster_lba(cluster);
            for s in 0..self.bpb.sectors_per_cluster as u32 {
                let off = i * cluster_size + s as usize * SECTOR_SIZE;
                for b in buf.iter_mut() {
                    *b = 0;
                }
                if off < data.len() {
                    let n = core::cmp::min(SECTOR_SIZE, data.len() - off);
                    buf[..n].copy_from_slice(&data[off..off + n]);
                }
                ata::write_sector(self.drive, lba + s, &buf)?;
            }
        }
        Ok(clusters[0])
    }

    /// Locate the root-directory slot for `short`: the existing entry of that name (to
    /// overwrite), or the first free slot (to create). Errors with `DirFull` if the directory
    /// is full and the name is not already present.
    fn find_dir_slot(&self, short: &[u8; 11]) -> Result<DirSlot, FatError> {
        let start = self.bpb.root_dir_start_sector();
        let sectors = self.bpb.root_dir_sectors();
        let mut buf = vec![0u8; SECTOR_SIZE];
        let mut free_slot: Option<(u32, usize)> = None;

        for s in 0..sectors {
            let lba = start + s;
            ata::read_sector_from(self.drive, lba, &mut buf)?;
            for i in 0..(SECTOR_SIZE / DIR_ENTRY_SIZE) {
                let off = i * DIR_ENTRY_SIZE;
                let first_byte = buf[off];
                let attr = buf[off + ENTRY_ATTR_OFFSET];

                let in_use = first_byte != NAME_END && first_byte != NAME_DELETED;
                let real = attr != ATTR_LONG_NAME && attr & ATTR_VOLUME_ID == 0;
                if in_use && real && &buf[off..off + 11] == &short[..] {
                    let first = read_u16(&buf, off + ENTRY_FIRST_CLUSTER_LO_OFFSET);
                    let is_dir = attr & ATTR_DIRECTORY != 0;
                    return Ok(DirSlot { lba, offset: off, existing: Some((first, is_dir)) });
                }

                if free_slot.is_none() && !in_use {
                    free_slot = Some((lba, off)); // remember the first reusable slot
                }
                if first_byte == NAME_END {
                    // Nothing in use beyond here, so create in the first free slot seen (at
                    // worst this NAME_END slot itself).
                    let (lba, off) = free_slot.expect("the NAME_END slot is itself free");
                    return Ok(DirSlot { lba, offset: off, existing: None });
                }
            }
        }
        match free_slot {
            Some((lba, off)) => Ok(DirSlot { lba, offset: off, existing: None }),
            None => Err(FatError::DirFull),
        }
    }

    /// Create or overwrite the root-level file `name` with `data`. Overwriting frees the old
    /// cluster chain first. Fails with `InvalidName` if `name` is not a valid 8.3 name, or
    /// `IsDirectory` if `name` already names a subdirectory.
    pub fn write_file(&self, name: &str, data: &[u8]) -> Result<(), FatError> {
        let short = string_to_short_name(name).ok_or(FatError::InvalidName)?;

        let slot = self.find_dir_slot(&short)?;
        if let Some((old_first, is_dir)) = slot.existing {
            if is_dir {
                return Err(FatError::IsDirectory);
            }
            if old_first >= 2 {
                self.free_chain(old_first)?; // overwrite: release the previous contents
            }
        }

        // Write the data as a fresh cluster chain (first cluster 0 for an empty file).
        let first = self.write_chain(data)?;

        // Build the 32-byte directory entry from scratch and write its sector back.
        let mut buf = vec![0u8; SECTOR_SIZE];
        ata::read_sector_from(self.drive, slot.lba, &mut buf)?;
        let e = slot.offset;
        for b in &mut buf[e..e + DIR_ENTRY_SIZE] {
            *b = 0; // clear the slot: name, dates, attributes, the cluster-high word, etc.
        }
        buf[e..e + 11].copy_from_slice(&short);
        buf[e + ENTRY_ATTR_OFFSET] = ATTR_ARCHIVE;
        buf[e + ENTRY_FIRST_CLUSTER_LO_OFFSET] = (first & 0xFF) as u8;
        buf[e + ENTRY_FIRST_CLUSTER_LO_OFFSET + 1] = (first >> 8) as u8;
        buf[e + ENTRY_SIZE_OFFSET..e + ENTRY_SIZE_OFFSET + 4]
            .copy_from_slice(&(data.len() as u32).to_le_bytes());
        ata::write_sector(self.drive, slot.lba, &buf)?;

        Ok(())
    }
}
