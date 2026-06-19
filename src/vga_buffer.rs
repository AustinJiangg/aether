//! VGA text-mode buffer driver.
//!
//! In VGA text mode the screen is an 80x25 grid stored in a small region of
//! memory the graphics hardware exposes at physical address `0xb8000`. Writing
//! to that memory changes what is on the screen — this is "memory-mapped I/O":
//! the same load/store instructions you use for normal memory, but the address
//! happens to be wired to a device instead of RAM.
//!
//! Each cell is two bytes:
//! - byte 0: the code-page-437 character to display (close to ASCII for 0x20..)
//! - byte 1: the color — low nibble = foreground, high nibble = background.
//!
//! The bootloader identity-maps low memory, so the physical address `0xb8000`
//! is also valid as a virtual address here; we can use it directly.
//!
//! This module mirrors `serial.rs`: it exposes `print!` / `println!` macros, but
//! the output goes to the *screen* (the QEMU window) instead of the serial port.

use core::fmt::{self, Write};
use spin::Mutex;

/// VGA text mode is 25 rows by 80 columns (BIOS "mode 3").
const BUFFER_HEIGHT: usize = 25;
const BUFFER_WIDTH: usize = 80;

/// Physical (and, thanks to identity mapping, virtual) address of the buffer.
const VGA_BUFFER_ADDR: usize = 0xb8000;

/// The 16 colors available in VGA text mode. `repr(u8)` pins each variant to its
/// hardware color number so we can cast the enum straight to a byte.
#[allow(dead_code)] // not every color is used yet; keep the full palette.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Color {
    Black = 0,
    Blue = 1,
    Green = 2,
    Cyan = 3,
    Red = 4,
    Magenta = 5,
    Brown = 6,
    LightGray = 7,
    DarkGray = 8,
    LightBlue = 9,
    LightGreen = 10,
    LightCyan = 11,
    LightRed = 12,
    Pink = 13,
    Yellow = 14,
    White = 15,
}

/// A foreground/background pair packed into the one color byte the hardware
/// wants: bits 0-3 foreground, bits 4-6 background, bit 7 blink.
/// `repr(transparent)` means a `ColorCode` is laid out exactly as its inner
/// `u8`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(transparent)]
pub struct ColorCode(u8);

impl ColorCode {
    /// `const fn` so it can be used to initialize the global `WRITER` statically.
    const fn new(foreground: Color, background: Color) -> ColorCode {
        ColorCode((background as u8) << 4 | (foreground as u8))
    }
}

/// One character cell as the hardware stores it. `repr(C)` keeps the field order
/// (ascii byte first, then color byte) matching the layout the VGA card expects.
#[derive(Debug, Clone, Copy)]
#[repr(C)]
struct ScreenChar {
    ascii_character: u8,
    color_code: ColorCode,
}

/// Tracks where the next character goes and in which color. We deliberately do
/// *not* store a pointer to the buffer: the address is a constant, so each write
/// recomputes the cell pointer from `VGA_BUFFER_ADDR`. That keeps `Writer`
/// trivially constructible in a `const` context (see `WRITER` below).
pub struct Writer {
    column_position: usize,
    color_code: ColorCode,
}

impl Writer {
    /// Raw pointer to the cell at (`row`, `col`).
    fn cell_ptr(row: usize, col: usize) -> *mut ScreenChar {
        let base = VGA_BUFFER_ADDR as *mut ScreenChar;
        // SAFETY: callers only pass row < BUFFER_HEIGHT and col < BUFFER_WIDTH,
        // so the resulting offset stays inside the 80x25 buffer the VGA hardware
        // maps at 0xb8000.
        unsafe { base.add(row * BUFFER_WIDTH + col) }
    }

    /// Write a single byte at the current position on the bottom row, advancing
    /// the column and wrapping / scrolling as needed.
    fn write_byte(&mut self, byte: u8) {
        match byte {
            b'\n' => self.new_line(),
            byte => {
                if self.column_position >= BUFFER_WIDTH {
                    self.new_line();
                }
                let row = BUFFER_HEIGHT - 1;
                let col = self.column_position;
                let cell = ScreenChar {
                    ascii_character: byte,
                    color_code: self.color_code,
                };
                // SAFETY: row/col are in range. We use write_volatile because the
                // compiler cannot see that the VGA hardware "reads" this memory to
                // draw the screen; a plain store could be optimized away.
                unsafe {
                    Self::cell_ptr(row, col).write_volatile(cell);
                }
                self.column_position += 1;
            }
        }
    }

    /// Write a whole string, substituting a `■` for any byte outside printable
    /// ASCII (so a stray byte never corrupts the display).
    fn write_string(&mut self, s: &str) {
        for byte in s.bytes() {
            match byte {
                0x20..=0x7e | b'\n' => self.write_byte(byte),
                _ => self.write_byte(0xfe),
            }
        }
    }

    /// Scroll the screen up by one line: copy every row onto the row above it,
    /// blank the bottom row, and return the cursor to its start.
    fn new_line(&mut self) {
        for row in 1..BUFFER_HEIGHT {
            for col in 0..BUFFER_WIDTH {
                // SAFETY: row/col are in range; volatile read then write of VGA
                // cells, for the same reason as in `write_byte`.
                unsafe {
                    let character = Self::cell_ptr(row, col).read_volatile();
                    Self::cell_ptr(row - 1, col).write_volatile(character);
                }
            }
        }
        self.clear_row(BUFFER_HEIGHT - 1);
        self.column_position = 0;
    }

    /// Overwrite a row with blank spaces in the current color.
    fn clear_row(&mut self, row: usize) {
        let blank = ScreenChar {
            ascii_character: b' ',
            color_code: self.color_code,
        };
        for col in 0..BUFFER_WIDTH {
            // SAFETY: col is in range; volatile write of a VGA cell.
            unsafe {
                Self::cell_ptr(row, col).write_volatile(blank);
            }
        }
    }

    /// Blank the entire screen and move the cursor home.
    pub fn clear_screen(&mut self) {
        for row in 0..BUFFER_HEIGHT {
            self.clear_row(row);
        }
        self.column_position = 0;
    }

    /// Erase the most recently written character on the bottom row: step the
    /// cursor back one column and blank that cell. Does nothing at column 0 — we
    /// do not wrap back up to the previous row, so the shell's line editing only
    /// erases within the current line. Used by `backspace` below.
    pub fn backspace(&mut self) {
        if self.column_position > 0 {
            self.column_position -= 1;
            let blank = ScreenChar {
                ascii_character: b' ',
                color_code: self.color_code,
            };
            // SAFETY: column_position < BUFFER_WIDTH and the row is the last one,
            // so the cell is in range; volatile write of one VGA cell.
            unsafe {
                Self::cell_ptr(BUFFER_HEIGHT - 1, self.column_position).write_volatile(blank);
            }
        }
    }
}

/// Implementing `core::fmt::Write` is what lets `write_fmt` (and therefore the
/// `{}` formatting machinery behind `println!`) drive our writer.
impl Write for Writer {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        self.write_string(s);
        Ok(())
    }
}

/// The one global screen writer, guarded by a spinlock. Default color is light
/// gray on black. Because `Writer` has only `const`-constructible fields, this
/// needs no lazy initialization.
pub static WRITER: Mutex<Writer> = Mutex::new(Writer {
    column_position: 0,
    color_code: ColorCode::new(Color::LightGray, Color::Black),
});

/// Backing function for the `print!` / `println!` macros. Do not call directly.
#[doc(hidden)]
pub fn _print(args: fmt::Arguments) {
    use x86_64::instructions::interrupts;

    // Disable interrupts while we hold the WRITER lock. Otherwise an interrupt
    // handler that also prints (e.g. the timer) could fire mid-write, try to
    // take the same spinlock, and deadlock: the lock can't be released until the
    // interrupted code resumes, which can't happen until the handler returns.
    interrupts::without_interrupts(|| {
        // write_str never fails for our Writer, so unwrap can't actually panic.
        WRITER.lock().write_fmt(args).unwrap();
    });
}

/// Clear the whole screen. Free-function wrapper around [`Writer::clear_screen`]
/// that takes the lock with interrupts disabled, for the same deadlock-avoidance
/// reason as `_print`. Used by the shell's `clear` command.
pub fn clear_screen() {
    x86_64::instructions::interrupts::without_interrupts(|| WRITER.lock().clear_screen());
}

/// Erase the last character on the current line. Free-function wrapper around
/// [`Writer::backspace`], locked with interrupts disabled like `_print`. Used by
/// the shell's line editing.
pub fn backspace() {
    x86_64::instructions::interrupts::without_interrupts(|| WRITER.lock().backspace());
}

/// Print to the VGA screen without a trailing newline. Same usage as `print!`.
#[macro_export]
macro_rules! print {
    ($($arg:tt)*) => {
        $crate::vga_buffer::_print(format_args!($($arg)*))
    };
}

/// Print to the VGA screen with a trailing newline. Same usage as `println!`.
#[macro_export]
macro_rules! println {
    () => { $crate::print!("\n") };
    ($($arg:tt)*) => {
        $crate::print!("{}\n", format_args!($($arg)*))
    };
}
