//! Stage 7-8: a tiny interactive shell (a read-eval-print loop).
//!
//! This is the kernel's first real "user interaction": it reads a command line
//! from the keyboard, parses it, runs a built-in command, prints the result, and
//! repeats. It is built on the **revived Stage 5 async executor** — the shell is
//! an async task that `.await`s decoded keystrokes from the keyboard
//! [`ScancodeStream`]. When no key is waiting the task suspends and the executor
//! halts the CPU until the keyboard interrupt wakes it, so an idle shell costs
//! nothing.
//!
//! Stage 8 adds file commands (`ls`, `cat`, `write`, `mkdir`, `rm`, `cd`, `pwd`)
//! over the in-memory file system in [`crate::fs`]. The shell keeps a current
//! working directory and resolves relative paths against it.
//!
//! A note on "system calls": there is no user mode yet (no ring 3, no privilege
//! separation), so this shell runs in *kernel* space and its commands are plain
//! kernel function calls. The single `dispatch` entry point is the seed of what
//! becomes a system-call interface once user mode exists — but it is not one yet.
//!
//! Verifying a shell is awkward when there is no keyboard (headless QEMU cannot
//! type), so [`selftest`] drives canned commands (and a few simulated keystrokes)
//! through the same code paths at boot. The interactive [`run`] loop then handles
//! real keys.

use alloc::string::String;
use alloc::vec::Vec;

use futures_util::stream::StreamExt;
use pc_keyboard::{layouts::Us104Key, DecodedKey, HandleControl, PS2Keyboard, ScancodeSet1};

use crate::task::keyboard::ScancodeStream;
use crate::{allocator, fs, interrupts, net, process, vga_buffer};

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

/// The tick rate the kernel programs the Local APIC timer at (Stage 15). Sourced
/// from `apic.rs` so `uptime`'s ticks-to-seconds conversion never drifts from the
/// real timer. Unlike the old ~18.2 Hz PIT estimate, this rate is calibrated.
const TIMER_HZ: u64 = crate::apic::TIMER_HZ as u64;

/// Print the prompt, which shows the current working directory, e.g. `aether:/docs> `.
fn print_prompt(cwd: &str) {
    sh_print!("aether:{}> ", cwd);
}

/// Resolve `arg` (absolute or relative) against `cwd` into a normalized absolute
/// path. Handles `.` (stay), `..` (up a level), and redundant slashes. The result
/// always starts with `/`.
fn resolve_path(cwd: &str, arg: &str) -> String {
    // Absolute args ignore the cwd; relative ones build on it.
    let base = if arg.starts_with('/') { "" } else { cwd };

    let mut comps: Vec<&str> = Vec::new();
    for comp in base.split('/').chain(arg.split('/')) {
        match comp {
            "" | "." => {}              // skip empty parts and "."
            ".." => {
                comps.pop(); // ".." backs up one level (a no-op at the root)
            }
            name => comps.push(name),
        }
    }

    if comps.is_empty() {
        return String::from("/");
    }
    let mut path = String::new();
    for comp in comps {
        path.push('/');
        path.push_str(comp);
    }
    path
}

/// Parse one command line and run it.
///
/// The first whitespace-separated word is the command name; the rest is its
/// argument string. This is the shell's central dispatch point — the interactive
/// loop and the boot self-test both go through here. `cwd` is `&mut` so `cd` can
/// change it.
fn dispatch(cwd: &mut String, line: &str) {
    let line = line.trim();
    if line.is_empty() {
        return; // a blank line does nothing
    }

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

        // --- file system commands (Stage 8) ---
        "pwd" => sh_println!("{}", cwd),
        "ls" => cmd_ls(cwd, args),
        "cat" => cmd_cat(cwd, args),
        "write" => cmd_write(cwd, args),
        "mkdir" => cmd_mkdir(cwd, args),
        "rm" => cmd_rm(cwd, args),
        "cd" => cmd_cd(cwd, args),

        // --- network commands (Stage 18d, 19b) ---
        "ifconfig" => cmd_ifconfig(),
        "arp" => cmd_arp(),
        "ping" => cmd_ping(args),
        "nslookup" => cmd_nslookup(args),

        // --- user-space networking (Stage 24d-2) ---
        "nc" => cmd_nc(args),

        other => sh_println!("unknown command: '{}' (try 'help')", other),
    }
}

/// The `help` command: list the built-ins.
fn help() {
    sh_println!("available commands:");
    sh_println!("  help                  show this list");
    sh_println!("  echo <text>           print <text>");
    sh_println!("  clear                 clear the screen");
    sh_println!("  ticks / uptime        timer ticks / rough seconds since boot");
    sh_println!("  mem                   kernel heap location and size");
    sh_println!("  ls [path]             list a directory");
    sh_println!("  cat <path>            print a file");
    sh_println!("  write <path> <text>   write text to a file");
    sh_println!("  mkdir <path>          create a directory");
    sh_println!("  rm <path>             remove a file or directory");
    sh_println!("  cd <path>             change directory");
    sh_println!("  pwd                   print the working directory");
    sh_println!("  ifconfig              show the network interface + stats");
    sh_println!("  arp                   show the ARP cache");
    sh_println!("  ping <a.b.c.d>        send an ICMP echo to an IPv4 address");
    sh_println!("  nslookup <hostname>   resolve a hostname to an IPv4 address via DNS");
    sh_println!("  nc <ip> <port> [text] ring 3 netcat: connect, send, print the reply");
    sh_println!("                        (aimed at our own IP, a loopback echo answers)");
}

/// `ls [path]` — list a directory (the cwd if no path is given).
fn cmd_ls(cwd: &str, args: &str) {
    let path = if args.is_empty() {
        String::from(cwd)
    } else {
        resolve_path(cwd, args)
    };
    match fs::list(&path) {
        Ok(entries) if entries.is_empty() => sh_println!("(empty)"),
        Ok(entries) => {
            for (name, is_dir) in entries {
                // A trailing slash marks directories, like a real `ls -F`.
                sh_println!("  {}{}", name, if is_dir { "/" } else { "" });
            }
        }
        Err(e) => sh_println!("ls: {}: {}", path, e.as_str()),
    }
}

/// `cat <path>` — print a file's contents (as UTF-8 text when possible).
fn cmd_cat(cwd: &str, args: &str) {
    let path = resolve_path(cwd, args);
    match fs::read(&path) {
        Ok(data) => match core::str::from_utf8(&data) {
            Ok(text) => sh_println!("{}", text),
            Err(_) => sh_println!("<{} bytes of binary data>", data.len()),
        },
        Err(e) => sh_println!("cat: {}: {}", path, e.as_str()),
    }
}

/// `write <path> <text>` — create/overwrite a file with the given text.
fn cmd_write(cwd: &str, args: &str) {
    let mut parts = args.splitn(2, char::is_whitespace);
    let path_arg = parts.next().unwrap_or("");
    let text = parts.next().unwrap_or("");
    if path_arg.is_empty() {
        sh_println!("usage: write <path> <text>");
        return;
    }
    let path = resolve_path(cwd, path_arg);
    match fs::write(&path, text.as_bytes()) {
        Ok(()) => sh_println!("wrote {} bytes to {}", text.len(), path),
        Err(e) => sh_println!("write: {}: {}", path, e.as_str()),
    }
}

/// `mkdir <path>` — create a directory.
fn cmd_mkdir(cwd: &str, args: &str) {
    let path = resolve_path(cwd, args);
    if let Err(e) = fs::mkdir(&path) {
        sh_println!("mkdir: {}: {}", path, e.as_str());
    }
}

/// `rm <path>` — remove a file or directory.
fn cmd_rm(cwd: &str, args: &str) {
    let path = resolve_path(cwd, args);
    if let Err(e) = fs::remove(&path) {
        sh_println!("rm: {}: {}", path, e.as_str());
    }
}

/// `cd <path>` — change the working directory (no arg goes to root).
fn cmd_cd(cwd: &mut String, args: &str) {
    let path = if args.is_empty() {
        String::from("/")
    } else {
        resolve_path(cwd, args)
    };
    if fs::is_dir(&path) {
        *cwd = path;
    } else {
        sh_println!("cd: {}: not a directory", path);
    }
}

/// `ifconfig` — show our network interface (IP + MAC), the DHCP lease, and traffic counters
/// (Stage 18d; the lease line is Stage 20b).
fn cmd_ifconfig() {
    let ip = net::our_ip();
    let mac = net::our_mac();
    sh_println!(
        "eth0: inet {}.{}.{}.{}  ether {:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}",
        ip[0], ip[1], ip[2], ip[3],
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
    );
    if net::dhcp_configured() {
        let mask = net::leased_mask();
        let gw = net::leased_gateway();
        let dns = net::leased_dns();
        sh_println!(
            "  DHCP lease: mask {}.{}.{}.{}  gateway {}.{}.{}.{}  dns {}.{}.{}.{}  ({} s)",
            mask[0], mask[1], mask[2], mask[3],
            gw[0], gw[1], gw[2], gw[3],
            dns[0], dns[1], dns[2], dns[3],
            net::lease_secs(),
        );
    } else {
        sh_println!("  DHCP: not configured (static address)");
    }
    sh_println!(
        "  RX frames {}  ARP replies sent {}  pings sent-back {}  answered {}",
        net::frames_received(),
        net::arp_replies_sent(),
        net::icmp_replies_received(),
        net::icmp_requests_handled(),
    );
    sh_println!(
        "  UDP received {}  echoed {}  delivered {}",
        net::udp_received(),
        net::udp_echoes_sent(),
        net::udp_delivered(),
    );
}

/// `arp` — print the ARP cache (learned IPv4 -> MAC mappings), Stage 18d.
fn cmd_arp() {
    let entries = net::arp::cache_entries();
    if entries.is_empty() {
        sh_println!("(arp cache empty)");
        return;
    }
    for (ip, mac) in entries {
        sh_println!(
            "  {}.{}.{}.{}  ->  {:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}",
            ip[0], ip[1], ip[2], ip[3],
            mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
        );
    }
}

/// `ping <a.b.c.d>` — send an ICMP echo request and report the reply (Stage 18d).
fn cmd_ping(args: &str) {
    let ip = match net::parse_ipv4(args) {
        Some(ip) => ip,
        None => {
            sh_println!("usage: ping <a.b.c.d>");
            return;
        }
    };
    sh_println!("PING {}.{}.{}.{}", ip[0], ip[1], ip[2], ip[3]);
    match net::ping(ip) {
        Some(seq) => sh_println!(
            "  reply from {}.{}.{}.{}: icmp_seq={}",
            ip[0], ip[1], ip[2], ip[3], seq
        ),
        None => sh_println!("  no reply from {}.{}.{}.{}", ip[0], ip[1], ip[2], ip[3]),
    }
}

/// `nslookup <hostname>` — resolve a hostname to an IPv4 address via DNS (Stage 19b-2).
fn cmd_nslookup(args: &str) {
    let host = args.trim();
    if host.is_empty() {
        sh_println!("usage: nslookup <hostname>");
        return;
    }
    match net::dns_resolve(host) {
        Some(ip) => sh_println!("  {} has address {}.{}.{}.{}", host, ip[0], ip[1], ip[2], ip[3]),
        None => sh_println!("  could not resolve {}", host),
    }
}

/// `nc <a.b.c.d> <port> [text]` — Stage 24d-2: a tiny ring 3 "netcat".
///
/// This is the stage's point: the other network commands run *in the kernel*, but `nc`
/// spawns a **user process** that drives the whole Stage 24 socket lifecycle itself —
/// `socket`, a blocking `connect`, `send`, a blocking `recv`, `write` (printing
/// whatever the peer sent back), `close`, `exit` — and the shell simply resumes when it
/// exits ([`process::run_netcat`], the first user program launched from the running
/// shell rather than as a boot phase). Aimed at our own IP the kernel stands up a
/// loopback echo peer, so the text comes straight back; aimed elsewhere it is a real
/// TCP client over SLIRP (e.g. `nc 10.0.2.2 8000 hi` reaches a listener on the host).
fn cmd_nc(args: &str) {
    let mut parts = args.splitn(3, char::is_whitespace);
    let ip = parts.next().and_then(net::parse_ipv4);
    let port = parts.next().and_then(|p| p.parse::<u16>().ok());
    let (Some(ip), Some(port)) = (ip, port) else {
        sh_println!("usage: nc <a.b.c.d> <port> [text]");
        return;
    };
    // The text to send, newline-terminated like a real netcat line (default if omitted).
    let mut msg = String::from(parts.next().unwrap_or("hello from ring 3"));
    msg.push('\n');
    if msg.len() > process::NETCAT_MAX_MSG {
        sh_println!("nc: text too long (max {} bytes)", process::NETCAT_MAX_MSG - 1);
        return;
    }

    sh_println!(
        "nc: connecting to {}.{}.{}.{}:{} from ring 3 ...",
        ip[0], ip[1], ip[2], ip[3], port
    );
    // The reply the user program receives is printed by the program itself (its
    // `write` syscall lands on the screen and serial); we only report the outcome.
    if process::run_netcat(ip, port, msg.as_bytes()) {
        sh_println!("nc: connection closed");
    } else {
        sh_println!("nc: could not connect to {}.{}.{}.{}:{}", ip[0], ip[1], ip[2], ip[3], port);
    }
}

/// Handle one decoded key against the current line buffer.
///
/// Echoes printable characters, erases on Backspace, and on Enter runs the
/// buffered line. Factored out of [`run`] so the boot [`selftest`] can drive the
/// exact same key-handling logic without a real keyboard.
fn handle_key(line: &mut String, cwd: &mut String, key: DecodedKey) {
    match key {
        // Enter: finish the line, run it, then show a fresh prompt.
        DecodedKey::Unicode('\n') => {
            sh_println!();
            dispatch(cwd, line);
            line.clear();
            print_prompt(cwd);
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

/// Boot-time self-test: drive canned commands and a few simulated keystrokes
/// through the real `dispatch`/`handle_key`, so the shell and file system are
/// verifiable without a keyboard (e.g. headless QEMU). It builds a small
/// directory tree, lists and reads it, then removes part of it.
pub fn selftest() {
    let mut cwd = String::from("/");

    sh_println!();
    sh_println!("[shell selftest] commands + in-memory file system:");
    let script = [
        "help",
        "mkdir /docs",
        "write /docs/hello.txt hi from aether",
        "write /readme top-level readme",
        "mkdir /docs/sub",
        "ls /",
        "ls /docs",
        "cat /docs/hello.txt",
        "cd /docs",
        "pwd",
        "ls",
        "cat ../readme",
        "rm /docs/hello.txt",
        "ls",
        "cd /",
        "bogus",
    ];
    for command in script {
        sh_println!("aether:{}> {}", cwd, command);
        dispatch(&mut cwd, command);
    }

    // Stage 14b-3 / 14c: the FAT disk is mounted at /mnt, so the same ls/cat/write/rm commands
    // reach real on-disk files through the VFS — no special "disk" command needed. Here we write
    // a file, read it back, then remove it: the full lifecycle. (WRITTEN.DAT, written by the
    // test suite, persists on the disk image across reboots — the point of persistence.)
    sh_println!("[shell selftest] read/write/remove on the mounted FAT disk at /mnt:");
    for command in [
        "ls /mnt",
        "cat /mnt/HELLO.TXT",
        "write /mnt/NOTE.TXT shell wrote this to disk",
        "cat /mnt/NOTE.TXT",
        "rm /mnt/NOTE.TXT",
        "ls /mnt",
    ] {
        sh_println!("aether:{}> {}", cwd, command);
        dispatch(&mut cwd, command);
    }

    // Stage 14d-2: the FAT read path now traverses subdirectories, so cd/ls/cat descend into a
    // nested directory (SUB/NESTED.TXT, seeded on the disk image by build.rs) exactly as they do
    // in the root — proving the driver walks a subdirectory's cluster chain, not just the fixed
    // root region.
    sh_println!("[shell selftest] subdirectory traversal on the FAT disk under /mnt/SUB:");
    for command in [
        "ls /mnt/SUB",
        "cat /mnt/SUB/NESTED.TXT",
        "cd /mnt/SUB",
        "pwd",
        "ls",
        "cat NESTED.TXT",
        "cd /",
    ] {
        sh_println!("aether:{}> {}", cwd, command);
        dispatch(&mut cwd, command);
    }

    // Stage 14d-3: the write path traverses too — create, read, and remove a file *inside* the
    // subdirectory, exactly as at the root (proving write/rm resolve the parent directory, not
    // just the root). Self-cleaning, so a reboot starts from the seeded state.
    sh_println!("[shell selftest] write + remove inside the FAT subdirectory /mnt/SUB:");
    for command in [
        "write /mnt/SUB/HELLO2.TXT hi from inside a subdir",
        "ls /mnt/SUB",
        "cat /mnt/SUB/HELLO2.TXT",
        "rm /mnt/SUB/HELLO2.TXT",
        "ls /mnt/SUB",
    ] {
        sh_println!("aether:{}> {}", cwd, command);
        dispatch(&mut cwd, command);
    }

    // Stage 14d-6: `rm` now removes an *empty* directory on the FAT disk (rmdir). Make a directory,
    // list it in the mount root, remove it, and confirm it is gone — the mkdir/rmdir round-trip.
    // Self-cleaning, so a reboot starts from the seeded state.
    sh_println!("[shell selftest] mkdir + rmdir on the FAT disk under /mnt:");
    for command in [
        "mkdir /mnt/TMPDIR",
        "ls /mnt",
        "rm /mnt/TMPDIR",
        "ls /mnt",
    ] {
        sh_println!("aether:{}> {}", cwd, command);
        dispatch(&mut cwd, command);
    }

    // Stage 18d: the network commands over the live stack. The e1000 + net stack came up earlier in
    // boot (our IP/MAC, the gateway resolved by ARP), so `ifconfig`/`arp` show real state and `ping`
    // reaches SLIRP's gateway over the (emulated) wire — the same commands a user types interactively.
    sh_println!("[shell selftest] network stack at /net (18d): ifconfig / arp / ping:");
    for command in ["ifconfig", "arp", "ping 10.0.2.2", "ping 10.0.2.3"] {
        sh_println!("aether:{}> {}", cwd, command);
        dispatch(&mut cwd, command);
    }

    // Stage 24d-2: the ring 3 netcat, wired into the shell. Aimed at our own (DHCP-leased)
    // address, the kernel stands up a loopback echo peer, so this one command drives the whole
    // user-space socket lifecycle — a freshly spawned ring 3 process socket()s, connect()s,
    // send()s the text, recv()s the echo, write()s it (the line printed below), close()s, and
    // exits — after which the shell simply carries on. The address is read at runtime, so the
    // command is built dynamically rather than scripted.
    sh_println!("[shell selftest] ring 3 netcat over the loopback echo (24d-2):");
    let ip = net::our_ip();
    let nc_cmd = alloc::format!(
        "nc {}.{}.{}.{} 7 hello from ring 3 netcat",
        ip[0], ip[1], ip[2], ip[3]
    );
    sh_println!("aether:{}> {}", cwd, nc_cmd);
    dispatch(&mut cwd, &nc_cmd);

    // Exercise the interactive key path (echo, Backspace, Enter) by feeding
    // decoded keys through the same `handle_key` the live loop uses. We "type"
    // `echX`, Backspace (erasing the X), then `o hi` and Enter, so the buffer
    // becomes `echo hi`; the resulting `hi` proves the editing worked. (On serial
    // the X still shows, since the port cannot un-print; on screen it is erased.)
    sh_println!("[shell selftest] simulating typed input with a Backspace:");
    let mut line = String::new();
    print_prompt(&cwd);
    for key in ['e', 'c', 'h', 'X', '\u{8}', 'o', ' ', 'h', 'i', '\n'] {
        handle_key(&mut line, &mut cwd, DecodedKey::Unicode(key));
    }

    sh_println!("[shell selftest] done");
}

/// The interactive shell task.
///
/// Reads decoded keystrokes from the keyboard [`ScancodeStream`], echoes them
/// with minimal line editing, and on Enter dispatches the buffered line.
/// `scancodes.next().await` suspends the task when no input is waiting, so the
/// executor can halt the CPU until the keyboard interrupt wakes us. The decoding
/// mirrors the Stage 5 keyboard task; the new part is buffering and dispatching.
pub async fn run() {
    let mut scancodes = ScancodeStream::new();
    let mut keyboard = PS2Keyboard::new(ScancodeSet1::new(), Us104Key, HandleControl::Ignore);
    let mut line = String::new();
    let mut cwd = String::from("/");

    sh_println!();
    sh_println!("Interactive shell ready - type a command (try 'help'):");
    print_prompt(&cwd);

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
        handle_key(&mut line, &mut cwd, key);
    }
}
