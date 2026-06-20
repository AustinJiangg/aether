//! Minimal ELF64 parser (Stage 11b).
//!
//! An ELF ("Executable and Linkable Format") file is how compilers and linkers
//! hand a program to the operating system. We need only the bare minimum to load
//! a statically-linked ELF64 executable: validate the file header, then walk the
//! *program headers* and find the `PT_LOAD` segments — the chunks of the file the
//! loader must copy into memory at fixed virtual addresses before jumping to the
//! entry point.
//!
//! This module only *reads* the bytes; mapping the segments into an address space
//! is the loader's job (see `process.rs`). Keeping the parser pure makes it
//! unit-testable without any page tables.
//!
//! We read every field by byte offset (with `from_le_bytes`) rather than casting
//! the buffer to a `#[repr(C)]` struct: the input may be unaligned, so explicit
//! little-endian reads are both safe and clear. The field offsets below come
//! straight from the ELF64 specification (`Elf64_Ehdr` and `Elf64_Phdr`).

/// `p_type` value marking a loadable segment.
pub const PT_LOAD: u32 = 1;
/// `p_flags` bit: the segment is executable.
pub const PF_X: u32 = 1 << 0;
/// `p_flags` bit: the segment is writable.
pub const PF_W: u32 = 1 << 1;
/// `p_flags` bit: the segment is readable.
pub const PF_R: u32 = 1 << 2;

/// Why parsing or loading an ELF failed. The first group is detected by the parser
/// here; the last three by the loader in `process.rs` (which reuses this type).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ElfError {
    /// The buffer is smaller than an ELF64 header.
    TooSmall,
    /// The first four bytes are not the ELF magic `\x7FELF`.
    BadMagic,
    /// Not a 64-bit ELF (`EI_CLASS != ELFCLASS64`).
    NotElf64,
    /// Not little-endian (`EI_DATA != ELFDATA2LSB`).
    NotLittleEndian,
    /// `e_machine` is not x86-64.
    NotX86_64,
    /// `e_type` is not `ET_EXEC` (we only load static executables).
    NotExecutable,
    /// The program-header table is malformed or runs past the buffer.
    BadProgramHeaders,
    /// A `PT_LOAD` segment's file range runs past the buffer (loader-side).
    BadSegment,
    /// Ran out of physical frames while loading (loader-side).
    OutOfFrames,
    /// `map_to` failed while mapping a segment (loader-side).
    MapFailed,
}

/// A parsed, validated ELF64 file. Borrows the backing bytes.
pub struct ElfFile<'a> {
    bytes: &'a [u8],
}

impl<'a> ElfFile<'a> {
    /// Validate `bytes` as an x86-64 ELF64 executable and bounds-check its
    /// program-header table. On success every field read below is in range.
    pub fn parse(bytes: &'a [u8]) -> Result<ElfFile<'a>, ElfError> {
        if bytes.len() < 64 {
            return Err(ElfError::TooSmall);
        }
        if &bytes[0..4] != b"\x7FELF" {
            return Err(ElfError::BadMagic);
        }
        if bytes[4] != 2 {
            return Err(ElfError::NotElf64); // EI_CLASS = ELFCLASS64
        }
        if bytes[5] != 1 {
            return Err(ElfError::NotLittleEndian); // EI_DATA = ELFDATA2LSB
        }
        if read_u16(bytes, 18) != 0x3E {
            return Err(ElfError::NotX86_64); // e_machine = EM_X86_64
        }
        if read_u16(bytes, 16) != 2 {
            return Err(ElfError::NotExecutable); // e_type = ET_EXEC
        }

        // Bounds-check the program-header table so the segment reads can't panic.
        let phoff = read_u64(bytes, 32) as usize;
        let phentsize = read_u16(bytes, 54) as usize;
        let phnum = read_u16(bytes, 56) as usize;
        if phentsize < 56 {
            return Err(ElfError::BadProgramHeaders);
        }
        let table_size = phentsize
            .checked_mul(phnum)
            .ok_or(ElfError::BadProgramHeaders)?;
        let table_end = phoff
            .checked_add(table_size)
            .ok_or(ElfError::BadProgramHeaders)?;
        if table_end > bytes.len() {
            return Err(ElfError::BadProgramHeaders);
        }

        Ok(ElfFile { bytes })
    }

    /// The virtual address of the program's first instruction (`e_entry`).
    pub fn entry(&self) -> u64 {
        read_u64(self.bytes, 24)
    }

    /// Iterate the `PT_LOAD` segments — the parts the loader must place in memory.
    pub fn load_segments(&self) -> impl Iterator<Item = ProgramHeader> + '_ {
        let phoff = read_u64(self.bytes, 32) as usize;
        let phentsize = read_u16(self.bytes, 54) as usize;
        let phnum = read_u16(self.bytes, 56) as usize;
        let bytes = self.bytes;
        (0..phnum).filter_map(move |i| {
            let base = phoff + i * phentsize;
            if read_u32(bytes, base) != PT_LOAD {
                return None;
            }
            Some(ProgramHeader {
                flags: read_u32(bytes, base + 4),
                file_offset: read_u64(bytes, base + 8) as usize,
                vaddr: read_u64(bytes, base + 16),
                file_size: read_u64(bytes, base + 32) as usize,
                mem_size: read_u64(bytes, base + 40) as usize,
            })
        })
    }
}

/// One `PT_LOAD` program header: copy `file_size` bytes from `file_offset` to
/// `vaddr`, then zero-fill up to `mem_size` (the tail is `.bss`).
#[derive(Debug, Clone, Copy)]
pub struct ProgramHeader {
    pub flags: u32,
    pub file_offset: usize,
    pub vaddr: u64,
    pub file_size: usize,
    pub mem_size: usize,
}

impl ProgramHeader {
    /// Whether ring 3 may execute from this segment.
    pub fn is_executable(&self) -> bool {
        self.flags & PF_X != 0
    }
    /// Whether ring 3 may write to this segment.
    pub fn is_writable(&self) -> bool {
        self.flags & PF_W != 0
    }
    /// Whether ring 3 may read this segment.
    pub fn is_readable(&self) -> bool {
        self.flags & PF_R != 0
    }
}

// Little-endian field readers. Every call site is bounds-checked by `parse` (the
// header is >= 64 bytes; the program-header table end is validated), so the
// slicing below cannot panic on a parsed `ElfFile`.
fn read_u16(b: &[u8], o: usize) -> u16 {
    u16::from_le_bytes([b[o], b[o + 1]])
}
fn read_u32(b: &[u8], o: usize) -> u32 {
    u32::from_le_bytes([b[o], b[o + 1], b[o + 2], b[o + 3]])
}
fn read_u64(b: &[u8], o: usize) -> u64 {
    let mut a = [0u8; 8];
    a.copy_from_slice(&b[o..o + 8]);
    u64::from_le_bytes(a)
}
