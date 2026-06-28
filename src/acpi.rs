//! Stage 16a: discovering the machine's CPUs via ACPI.
//!
//! On x86, power-on starts exactly one CPU — the **BSP** (bootstrap processor).
//! Every other core — an **AP** (application processor) — sits halted, waiting for
//! a wake-up sequence (INIT-SIPI-SIPI, sent in Stage 16b). To send that we need
//! each AP's **Local APIC id** (an IPI is addressed by APIC id), and to know how
//! many cores exist at all. The firmware reports both — along with much else about
//! the machine — through **ACPI** (Advanced Configuration and Power Interface)
//! tables in memory.
//!
//! This module walks just enough of ACPI to enumerate the CPUs:
//!
//! ```text
//!   RSDP   (found by scanning low memory for the signature "RSD PTR ")
//!     └─> RSDT / XSDT   (an array of physical pointers to other tables)
//!           └─> MADT    (signature "APIC": the interrupt-controller table)
//!                 └─> Processor Local APIC entries — one per core (apic id + flags)
//! ```
//!
//! Every ACPI table *after* the RSDP begins with a common 36-byte header whose
//! `length` field bounds it and whose bytes sum to zero (the checksum). We trust
//! the firmware's tables (QEMU's are well-formed), but still bounds-check the
//! lengths so a malformed table degrades to "BSP only" rather than reading past a
//! buffer or looping forever.
//!
//! Like `elf.rs` and the FAT BPB parser, this is pure byte parsing over memory the
//! bootloader already mapped — everything is read through the physical-memory
//! window (`BootInfo::physical_memory_offset`). The only hardware it touches is
//! reading this core's own Local APIC id, to tell which entry is the BSP.

use alloc::vec::Vec;

use spin::Mutex;
use x86_64::VirtAddr;

use crate::{apic, serial_println};

/// One CPU core, as listed in the ACPI MADT.
#[derive(Clone, Copy, Debug)]
pub struct CpuCore {
    /// The Local APIC id — the address used to send this core an IPI (Stage 16b).
    pub apic_id: u8,
    /// True for the one core currently executing this code (the BSP).
    pub is_bsp: bool,
}

/// Every usable CPU core discovered at boot (BSP + APs). Filled once by [`discover`].
static CORES: Mutex<Vec<CpuCore>> = Mutex::new(Vec::new());

/// The 8-byte signature that marks the RSDP. (`b"..."` is `&[u8; 8]`; storing it as
/// `&[u8]` lets us compare it against a slice without juggling array lengths.)
const RSDP_SIG: &[u8] = b"RSD PTR ";
/// The 4-byte signature of the MADT within the RSDT/XSDT.
const MADT_SIG: &[u8] = b"APIC";

/// MADT entry type for a Processor Local APIC (one per core, 8-bit apic id). A
/// machine with >255 cores uses the Processor Local x2APIC entry (type 9, 32-bit
/// apic id) instead; QEMU `-smp` up to 255 uses the type-0 entry we parse here.
const MADT_LOCAL_APIC: u8 = 0;
/// Local APIC flags, bit 0: the processor is enabled (usable now).
const MADT_APIC_ENABLED: u32 = 1 << 0;
/// Local APIC flags, bit 1: the processor is online-capable (the OS may enable it).
const MADT_APIC_ONLINE_CAPABLE: u32 = 1 << 1;

// --- little-endian field readers over a byte slice (ACPI is little-endian) ------

fn u32_le(buf: &[u8], off: usize) -> u32 {
    u32::from_le_bytes([buf[off], buf[off + 1], buf[off + 2], buf[off + 3]])
}
fn u64_le(buf: &[u8], off: usize) -> u64 {
    u64::from_le_bytes([
        buf[off], buf[off + 1], buf[off + 2], buf[off + 3],
        buf[off + 4], buf[off + 5], buf[off + 6], buf[off + 7],
    ])
}

/// A readable pointer to physical address `phys`, via the physical-memory window.
fn window_ptr(phys_offset: VirtAddr, phys: u64) -> *const u8 {
    (phys_offset.as_u64() + phys) as *const u8
}

/// Copy `len` bytes starting at physical address `phys` into a fresh `Vec`, read
/// through the physical-memory window.
///
/// # Safety
/// `phys .. phys+len` must lie within mapped physical memory — true for any address
/// the firmware's ACPI tables point at, since the bootloader maps all of RAM into
/// the window.
unsafe fn read_phys(phys_offset: VirtAddr, phys: u64, len: usize) -> Vec<u8> {
    let mut buf = alloc::vec![0u8; len];
    // SAFETY: `window_ptr` is valid for `len` bytes (see the contract above), and
    // `buf` is a fresh, non-overlapping allocation of exactly `len` bytes.
    core::ptr::copy_nonoverlapping(window_ptr(phys_offset, phys), buf.as_mut_ptr(), len);
    buf
}

/// True if `buf`'s bytes sum to 0 (mod 256) — how every ACPI structure is checksummed.
fn checksum_ok(buf: &[u8]) -> bool {
    buf.iter().fold(0u8, |acc, &b| acc.wrapping_add(b)) == 0
}

/// Read an ACPI table that begins with the standard 36-byte SDT header: read the
/// header to learn the table's `length`, then copy the whole table. Returns `None`
/// if the length is implausible (so a corrupt field cannot drive a wild allocation).
///
/// # Safety
/// `phys` must point at an ACPI table inside the physical-memory window.
unsafe fn read_table(phys_offset: VirtAddr, phys: u64) -> Option<Vec<u8>> {
    let header = read_phys(phys_offset, phys, 36);
    let length = u32_le(&header, 4) as usize; // total length, header included
    if length < 36 || length > 0x10000 {
        return None;
    }
    Some(read_phys(phys_offset, phys, length))
}

/// Scan the two legacy BIOS regions for the RSDP and return its physical address.
///
/// On a BIOS boot (which the 0.9 bootloader performs) the RSDP lives either in the
/// first KiB of the Extended BIOS Data Area, or in the ROM region
/// `0xE0000..0x100000`, always on a 16-byte boundary. We confirm the signature and
/// the 20-byte v1 checksum before trusting a hit.
///
/// # Safety
/// The physical-memory window must be mapped (the whole search range lies in the
/// first MiB, which the bootloader always maps).
unsafe fn find_rsdp(phys_offset: VirtAddr) -> Option<u64> {
    // The EBDA segment is stored as a real-mode segment (a word) at physical 0x40E;
    // shift left 4 to get its physical base.
    let ebda_seg = (window_ptr(phys_offset, 0x40E) as *const u16).read_unaligned();
    let ebda = (ebda_seg as u64) << 4;
    let ranges = [(ebda, ebda + 1024), (0xE_0000u64, 0x10_0000u64)];

    for (start, end) in ranges {
        let mut addr = start & !0xF; // round down to a 16-byte boundary
        while addr + 8 <= end {
            // SAFETY: `addr` is within the first MiB, fully inside the mapped
            // window, so these 8 bytes are valid to read as a slice.
            let sig = core::slice::from_raw_parts(window_ptr(phys_offset, addr), 8);
            if sig == RSDP_SIG && checksum_ok(&read_phys(phys_offset, addr, 20)) {
                return Some(addr);
            }
            addr += 16;
        }
    }
    None
}

/// From the RSDP, walk the RSDT (or XSDT) and return the MADT's table bytes.
///
/// # Safety
/// `rsdp_phys` must be a valid RSDP address (as returned by [`find_rsdp`]); all
/// table pointers it leads to are read through the window.
unsafe fn find_madt(phys_offset: VirtAddr, rsdp_phys: u64) -> Option<Vec<u8>> {
    let rsdp = read_phys(phys_offset, rsdp_phys, 36); // covers the v2 fields too
    let revision = rsdp[15];

    // ACPI 2.0+ (revision >= 2) provides a 64-bit XSDT; prefer it when present.
    // Otherwise fall back to the 32-bit RSDT. Both are SDTs whose entries are
    // pointers to other tables — only the pointer width differs.
    let (sdt_phys, ptr_size) = if revision >= 2 {
        match u64_le(&rsdp, 24) {
            0 => (u32_le(&rsdp, 16) as u64, 4),
            xsdt => (xsdt, 8),
        }
    } else {
        (u32_le(&rsdp, 16) as u64, 4)
    };

    let sdt = read_table(phys_offset, sdt_phys)?;
    if !checksum_ok(&sdt) {
        return None;
    }

    // Entries follow the 36-byte header, each a pointer to another ACPI table.
    let entries = (sdt.len() - 36) / ptr_size;
    for i in 0..entries {
        let off = 36 + i * ptr_size;
        let table_phys = if ptr_size == 8 {
            u64_le(&sdt, off)
        } else {
            u32_le(&sdt, off) as u64
        };
        // Peek only the 4-byte signature; copy the whole table only if it is the MADT.
        if read_phys(phys_offset, table_phys, 4) == MADT_SIG {
            return read_table(phys_offset, table_phys);
        }
    }
    None
}

/// Parse the MADT's interrupt-controller structures into the list of usable cores.
///
/// `bsp_apic_id` is this running core's Local APIC id, used to flag its entry as the
/// BSP. Pure: it reads only the bytes passed in.
fn parse_madt(madt: &[u8], bsp_apic_id: u8) -> Vec<CpuCore> {
    let mut cores = Vec::new();
    // The variable-length entries begin after the MADT-specific fixed fields that
    // follow the 36-byte SDT header: the local-APIC address (u32) and flags (u32).
    let mut off = 44;
    while off + 2 <= madt.len() {
        let entry_type = madt[off];
        let entry_len = madt[off + 1] as usize;
        // A zero length would loop forever; a length past the buffer is corrupt.
        if entry_len < 2 || off + entry_len > madt.len() {
            break;
        }
        if entry_type == MADT_LOCAL_APIC && entry_len >= 8 {
            // Entry layout: type, length, acpi_processor_id, apic_id, flags(u32).
            let apic_id = madt[off + 3];
            let flags = u32_le(madt, off + 4);
            // A core is usable if it is enabled now or can be enabled by the OS.
            if flags & (MADT_APIC_ENABLED | MADT_APIC_ONLINE_CAPABLE) != 0 {
                cores.push(CpuCore {
                    apic_id,
                    is_bsp: apic_id == bsp_apic_id,
                });
            }
        }
        off += entry_len;
    }
    cores
}

/// Discover the machine's CPUs via ACPI. Call **once** at boot, after the heap and
/// the Local APIC are up: it allocates while parsing, and reads this core's LAPIC id
/// to flag the BSP. Stores the result for [`cpu_count`] / [`bsp_apic_id`] /
/// [`application_processors`] and logs a summary.
///
/// On any parse failure it degrades to a single BSP-only entry, so boot continues
/// even on a machine whose ACPI we cannot read.
pub fn discover(phys_offset: VirtAddr) {
    let bsp = apic::lapic_id();

    // SAFETY: every physical address dereferenced below comes from the firmware's
    // own ACPI tables (or the fixed first-MiB BIOS scan regions), all of which lie
    // inside the bootloader's physical-memory window, so the reads are valid.
    let detected = unsafe {
        find_rsdp(phys_offset)
            .and_then(|rsdp| find_madt(phys_offset, rsdp))
            .map(|madt| parse_madt(&madt, bsp))
    };

    let mut cores = detected.unwrap_or_default();
    if cores.is_empty() {
        // No ACPI, no MADT, or no Local APIC entries: we still know about ourselves.
        serial_println!("[acpi] no MADT processor entries found; assuming BSP only");
        cores.push(CpuCore { apic_id: bsp, is_bsp: true });
    }

    let ap_ids: Vec<u8> = cores.iter().filter(|c| !c.is_bsp).map(|c| c.apic_id).collect();
    serial_println!(
        "[acpi] {} CPU core(s): BSP apic id {}, {} application processor(s) {:?}",
        cores.len(),
        bsp,
        ap_ids.len(),
        ap_ids,
    );

    *CORES.lock() = cores;
}

/// How many CPU cores the machine has (BSP + APs), per ACPI discovery.
pub fn cpu_count() -> usize {
    CORES.lock().len()
}

/// Every discovered CPU core (BSP + APs), in discovery order. Stage 16c builds a
/// per-CPU data block per entry, and waking the APs walks the same list.
pub fn cpus() -> Vec<CpuCore> {
    CORES.lock().clone()
}

/// The BSP's Local APIC id, as recorded by [`discover`].
pub fn bsp_apic_id() -> u8 {
    CORES
        .lock()
        .iter()
        .find(|c| c.is_bsp)
        .map(|c| c.apic_id)
        .unwrap_or(0)
}

/// Just the application processors — every core except the BSP. These are the cores
/// Stage 16b will wake with INIT-SIPI-SIPI.
pub fn application_processors() -> Vec<CpuCore> {
    CORES.lock().iter().copied().filter(|c| !c.is_bsp).collect()
}
