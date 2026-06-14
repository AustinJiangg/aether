//! Serial (UART) output module.
//!
//! On bare metal there is no `std`, and therefore no `print!` / `println!`.
//! This module uses a 16550 serial driver to implement equivalent
//! `serial_print!` / `serial_println!` macros that send kernel output to the
//! serial port owned by QEMU, which then forwards it to your terminal (see the
//! `-serial stdio` argument in Cargo.toml).
//!
//! Why serial instead of the VGA screen?
//! - Output appears directly in the terminal, easy to redirect, log, or have
//!   tools read.
//! - No graphical interface required; works with zero config under WSL2.
//! - It is the most common "debug print" mechanism in kernel development.

use core::fmt::{self, Write};
use spin::Mutex;
use uart_16550::SerialPort;

/// Global serial instance. Guarded by a `Mutex` so it can be accessed safely in
/// future multitasking / interrupt contexts; wrapped in `Option` because a
/// static needs const initialization, so the real hardware init is deferred to
/// `init()`.
static SERIAL1: Mutex<Option<SerialPort>> = Mutex::new(None);

/// Initialize the COM1 serial port. Must be called once before the first print.
pub fn init() {
    // 0x3F8 is the standard I/O port address for COM1 on the PC.
    // SAFETY: we have exclusive access to this port, and the address is the
    // fixed value defined by the x86 platform convention.
    let mut port = unsafe { SerialPort::new(0x3F8) };
    port.init();
    *SERIAL1.lock() = Some(port);
}

/// Internal print function used by the macros. Do not call directly.
#[doc(hidden)]
pub fn _print(args: fmt::Arguments) {
    let mut guard = SERIAL1.lock();
    if let Some(port) = guard.as_mut() {
        // Writing to the serial port shouldn't fail (barring hardware faults),
        // so we expect here.
        port.write_fmt(args).expect("failed to write to serial port");
    }
}

/// Print to the serial port without a trailing newline. Same usage as `print!`.
#[macro_export]
macro_rules! serial_print {
    ($($arg:tt)*) => {
        $crate::serial::_print(format_args!($($arg)*))
    };
}

/// Print to the serial port with a trailing newline. Same usage as `println!`.
#[macro_export]
macro_rules! serial_println {
    () => { $crate::serial_print!("\n") };
    ($($arg:tt)*) => {
        $crate::serial_print!("{}\n", format_args!($($arg)*))
    };
}
