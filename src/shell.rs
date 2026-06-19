//! Stage 7: a tiny interactive shell (a read-eval-print loop).
//!
//! This is the kernel's first real "user interaction": it reads a command line
//! from the keyboard, parses it, runs a built-in command, prints the result, and
//! repeats. It is built on the **revived Stage 5 async executor** — the shell is
//! an async task that `.await`s decoded keystrokes from the keyboard
//! [`ScancodeStream`]. When no key is waiting the task suspends and the executor
//! halts the CPU until the keyboard interrupt wakes it, so an idle shell costs
//! nothing.
//!
//! A note on "system calls": there is no user mode yet (no ring 3, no privilege
//! separation), so this shell runs in *kernel* space and its commands are plain
//! kernel function calls. The single `dispatch` entry point is the seed of what
//! becomes a system-call interface once user mode exists — but it is not one yet.
//!
//! Verifying a shell is awkward when there is no keyboard (headless QEMU cannot
//! type), so [`selftest`] feeds a few canned command lines through the same
//! `dispatch` path at boot. That output proves parsing and the commands work; the
//! interactive [`run`] loop then handles real keystrokes.

use alloc::string::String;

use futures_util::stream::StreamExt;
use pc_keyboard::{layouts::Us104Key, DecodedKey, HandleControl, PS2Keyboard, ScancodeSet1};

use crate::task::keyboard::ScancodeStream;
use crate::{allocator, interrupts, vga_buffer};

/// Print to BOTH the screen and the serial log, with a trailing newline.
///
/// The screen is where an interactive user looks; mirroring to the serial port
/// lets the boot self-test and a headless QEMU run capture the shell's output for
/// verification. `sh_print!` is the same without the newline (used to echo typed
/// characters). They expand to the crate's existing `print!`/`serial_print!`.
macro_rules! sh_print {
    ($($arg:tt)*) => {{
        $crate::print!($($arg)*);
        $crate::serial_print!($($arg)*);
    }};
}
macro_rules! sh_println {
    () => {{
        $crate::println!();
        $crate::serial_println!();
    }};
    ($($arg:tt)*) => {{
        $crate::println!($($arg)*);
        $crate::serial_println!($($arg)*);
    }};
}

/// What we print before each command line.
const PROMPT: &str = "aether> ";

/// The default PIT rate is ~18.2 Hz. We round to 18 for a rough `uptime`; this is
/// an estimate, not a calibrated clock.
const TIMER_HZ: u64 = 18;

/// Parse one command line and run it.
///
/// The first whitespace-separated word is the command name; everything after it
/// is the argument string. This is the shell's central dispatch point — both the
/// interactive loop and the boot self-test go through here.
fn dispatch(line: &str) {
    let line = line.trim();
    if line.is_empty() {
        return; // a blank line does nothing
    }

    // Split into the command word and the rest (its arguments).
    let mut parts = line.splitn(2, char::is_whitespace);
    let cmd = parts.next().unwrap_or("");
    let args = parts.next().unwrap_or("").trim();

    match cmd {
        "help" => help(),
        "echo" => sh_println!("{}", args),
        "clear" => vga_buffer::clear_screen(),
        "ticks" => sh_println!("timer ticks since boot: {}", interrupts::timer_ticks()),
        "uptime" => {
            let ticks = interrupts::timer_ticks();
            sh_println!("uptime: ~{} s ({} ticks @ ~{} Hz)", ticks / TIMER_HZ, ticks, TIMER_HZ);
        }
        "mem" => sh_println!(
            "kernel heap: start={:#x}, size={} KiB",
            allocator::HEAP_START,
            allocator::HEAP_SIZE / 1024
        ),
        other => sh_println!("unknown command: '{}' (try 'help')", other),
    }
}

/// The `help` command: list the built-ins.
fn help() {
    sh_println!("available commands:");
    sh_println!("  help          show this list");
    sh_println!("  echo <text>   print <text>");
    sh_println!("  clear         clear the screen");
    sh_println!("  ticks         timer ticks since boot");
    sh_println!("  uptime        rough seconds since boot");
    sh_println!("  mem           kernel heap location and size");
}

/// Boot-time self-test: run a few canned commands through [`dispatch`].
///
/// This makes the shell verifiable without a keyboard (headless QEMU), exercising
/// the exact parse-and-dispatch path the interactive loop uses. It deliberately
/// omits `clear`, which would wipe the boot log we want to inspect.
pub fn selftest() {
    sh_println!();
    sh_println!("[shell selftest] running canned commands through the dispatcher:");
    for command in ["help", "echo hello aether", "ticks", "mem", "bogus"] {
        sh_println!("{}{}", PROMPT, command); // show the line as if it were typed
        dispatch(command);
    }

    // Exercise the interactive key path (echo, Backspace, Enter) by feeding
    // decoded keys through the same `handle_key` the live loop uses — no keyboard
    // needed. We "type" `echX`, Backspace (erasing the X), then `o hi` and Enter,
    // so the buffer becomes `echo hi`; the resulting `hi` proves the editing
    // worked. (On serial the X still shows, since the port cannot un-print; on the
    // screen the Backspace really erases it.)
    sh_println!("[shell selftest] simulating typed input with a Backspace:");
    let mut line = String::new();
    sh_print!("{}", PROMPT);
    for key in ['e', 'c', 'h', 'X', '\u{8}', 'o', ' ', 'h', 'i', '\n'] {
        handle_key(&mut line, DecodedKey::Unicode(key));
    }

    sh_println!("[shell selftest] done");
}

/// The interactive shell task.
///
/// Reads decoded keystrokes from the keyboard [`ScancodeStream`], echoes them
/// with minimal line editing (printable characters and Backspace), and on Enter
/// dispatches the buffered line. `scancodes.next().await` suspends the task when
/// no input is waiting, so the executor can halt the CPU until the keyboard
/// interrupt wakes us. The decoding mirrors the Stage 5 keyboard task; the new
/// part is buffering a line and dispatching it.
pub async fn run() {
    let mut scancodes = ScancodeStream::new();
    let mut keyboard = PS2Keyboard::new(ScancodeSet1::new(), Us104Key, HandleControl::Ignore);
    let mut line = String::new();

    sh_println!();
    sh_println!("Interactive shell ready - type a command (try 'help'):");
    sh_print!("{}", PROMPT);

    while let Some(scancode) = scancodes.next().await {
        // A key event may span several scancode bytes, so `add_byte` returns
        // `Ok(None)` until it has assembled one; `process_keyevent` then maps it to
        // a `DecodedKey` (or `None` for, say, a modifier press).
        let Ok(Some(event)) = keyboard.add_byte(scancode) else {
            continue;
        };
        let Some(key) = keyboard.process_keyevent(event) else {
            continue;
        };
        handle_key(&mut line, key);
    }
}

/// Handle one decoded key against the current line buffer.
///
/// Echoes printable characters, erases on Backspace, and on Enter runs the
/// buffered line. Factored out of [`run`] so the boot [`selftest`] can drive the
/// exact same key-handling logic without a real keyboard.
fn handle_key(line: &mut String, key: DecodedKey) {
    match key {
        // Enter: finish the line, run it, then show a fresh prompt.
        DecodedKey::Unicode('\n') => {
            sh_println!();
            dispatch(line);
            line.clear();
            sh_print!("{}", PROMPT);
        }
        // Backspace (0x08) or Delete (0x7f): erase the last buffered character.
        // We only erase when the buffer is non-empty, which also keeps the cursor
        // from deleting into the prompt.
        DecodedKey::Unicode('\u{8}') | DecodedKey::Unicode('\u{7f}') => {
            if line.pop().is_some() {
                vga_buffer::backspace();
            }
        }
        // Any other printable character: buffer it and echo it.
        DecodedKey::Unicode(character) => {
            line.push(character);
            sh_print!("{}", character);
        }
        // Non-character keys (arrows, function keys, ...) are ignored for now.
        DecodedKey::RawKey(_) => {}
    }
}
