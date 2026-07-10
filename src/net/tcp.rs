//! TCP — the Transmission Control Protocol (Stage 21), the stack's first *reliable* transport.
//!
//! UDP (Stage 19a) is "send it and forget it": no connection, no acknowledgements, no ordering. TCP is
//! the opposite — a **connection-oriented, reliable, ordered byte stream**. That reliability is built
//! from a few ideas layered on top of the same IPv4 datagrams UDP uses:
//!
//! - **Sequence numbers.** Every byte in the stream is numbered. A segment's `seq` is the number of its
//!   first payload byte; the receiver replies with an `ack` naming the next byte it expects. This is how
//!   TCP detects loss (a gap in the numbers), reorders (numbers out of order), and de-duplicates.
//! - **Flags** mark the control segments that run the connection: **SYN** (synchronize — open, and carry
//!   the initial sequence number), **ACK** (the `ack` field is valid), **FIN** (no more data — close),
//!   **RST** (abort), plus **PSH**/**URG** (delivery hints we can ignore).
//! - **A window** (`window`) is flow control: how many more bytes the sender of this segment is willing
//!   to receive right now, so a fast sender cannot overrun a slow receiver.
//!
//! This module (Stage 21a) is only the **segment** layer — parse and build one TCP segment, with the
//! pseudo-header checksum — exactly as `udp.rs` was for UDP. The connection state machine (the
//! three-way handshake, data transfer, teardown, retransmission) is built on top of it in later
//! sub-steps. A segment is the payload of an IPv4 packet (protocol 6):
//!
//! ```text
//!   0               1               2               3
//!   +-------------------------------+-------------------------------+
//!   |          source port          |       destination port        |  0..4
//!   +-------------------------------+-------------------------------+
//!   |                        sequence number                        |  4..8
//!   +---------------------------------------------------------------+
//!   |                     acknowledgment number                     |  8..12
//!   +-------+-----------+-----------+-------------------------------+
//!   | offset|  reserved |  flags    |            window             |  12..16
//!   +-------+-----------+-----------+-------------------------------+
//!   |           checksum            |         urgent pointer        |  16..20
//!   +-------------------------------+-------------------------------+
//!   |                    options (if offset > 5) ...                |  20..
//!   +---------------------------------------------------------------+
//!   |                            data ...                           |
//!   +---------------------------------------------------------------+
//! ```
//!
//! `offset` (the "data offset", top 4 bits of byte 12) is the header length in 32-bit words — 5 means a
//! 20-byte header with no options. Everything multi-byte is big-endian. Unlike UDP, the TCP checksum is
//! **mandatory** and there is no "0 becomes 0xFFFF" rule (a zero field would just be a wrong checksum).

#![allow(dead_code)] // a few completeness items (states/flags) are unused until later sub-steps

use alloc::vec::Vec;
use core::sync::atomic::{AtomicU32, Ordering};

use spin::Mutex;

use super::ipv4;

/// IP protocol number for TCP (what an IPv4 header's `protocol` field holds for a TCP payload).
pub const PROTO_TCP: u8 = 6;

/// The minimum TCP header length (no options): five 32-bit words.
pub const HEADER_LEN: usize = 20;

// The six classic control flags (byte 13, low six bits). We ignore the ECN/CWR bits above them.
pub const FIN: u8 = 0x01;
pub const SYN: u8 = 0x02;
pub const RST: u8 = 0x04;
pub const PSH: u8 = 0x08;
pub const ACK: u8 = 0x10;
pub const URG: u8 = 0x20;
/// Mask of the six flags we recognize, used when parsing a peer's segment.
const FLAG_MASK: u8 = 0x3F;

/// A parsed, borrowed TCP segment. `payload` borrows the caller's buffer, starting after the header (and
/// any options), so it is the actual stream bytes this segment carries (empty for a pure control segment
/// like SYN or ACK).
pub struct Segment<'a> {
    pub src_port: u16,
    pub dst_port: u16,
    /// Sequence number: the stream position of the first payload byte (or, for a SYN/FIN, the control
    /// flag's own position — SYN and FIN each consume one sequence number).
    pub seq: u32,
    /// Acknowledgment number: the next sequence number the sender expects (valid only when [`ACK`] set).
    pub ack: u32,
    /// The control flags ([`SYN`], [`ACK`], [`FIN`], [`RST`], [`PSH`], [`URG`]), already masked.
    pub flags: u8,
    /// The sender's current receive window (flow control), in bytes.
    pub window: u16,
    pub payload: &'a [u8],
}

impl<'a> Segment<'a> {
    /// Parse a TCP segment. Returns `None` for a runt (shorter than the 20-byte header) or a bogus data
    /// offset (less than 5 words, or claiming more header than the buffer holds). The checksum is not
    /// re-verified here, matching the other layers (peers on the emulated link produce valid ones).
    pub fn parse(buf: &'a [u8]) -> Option<Segment<'a>> {
        if buf.len() < HEADER_LEN {
            return None;
        }
        let data_offset = (buf[12] >> 4) as usize * 4;
        if data_offset < HEADER_LEN || buf.len() < data_offset {
            return None; // a header shorter than the minimum, or longer than the bytes we have
        }
        Some(Segment {
            src_port: u16::from_be_bytes([buf[0], buf[1]]),
            dst_port: u16::from_be_bytes([buf[2], buf[3]]),
            seq: u32::from_be_bytes([buf[4], buf[5], buf[6], buf[7]]),
            ack: u32::from_be_bytes([buf[8], buf[9], buf[10], buf[11]]),
            flags: buf[13] & FLAG_MASK,
            window: u16::from_be_bytes([buf[14], buf[15]]),
            // Skip any options (data_offset past the fixed header) to reach the stream bytes.
            payload: &buf[data_offset..],
        })
    }
}

/// Build a TCP segment — a 20-byte header (no options) with a correct checksum, followed by `payload`.
/// The source/destination IPs are not stored in the segment; they are needed only for the checksum's
/// pseudo-header (see [`checksum`]), the same layering shortcut UDP makes. `flags` is an OR of the flag
/// constants (e.g. `SYN`, or `SYN | ACK`).
#[allow(clippy::too_many_arguments)]
pub fn build(
    src_ip: [u8; 4],
    dst_ip: [u8; 4],
    src_port: u16,
    dst_port: u16,
    seq: u32,
    ack: u32,
    flags: u8,
    window: u16,
    payload: &[u8],
) -> Vec<u8> {
    let mut seg = Vec::with_capacity(HEADER_LEN + payload.len());
    seg.extend_from_slice(&src_port.to_be_bytes());
    seg.extend_from_slice(&dst_port.to_be_bytes());
    seg.extend_from_slice(&seq.to_be_bytes());
    seg.extend_from_slice(&ack.to_be_bytes());
    seg.push(5 << 4); // data offset = 5 words (20 bytes, no options); reserved bits zero
    seg.push(flags);
    seg.extend_from_slice(&window.to_be_bytes());
    seg.extend_from_slice(&[0, 0]); // checksum placeholder, zero for the computation below
    seg.extend_from_slice(&[0, 0]); // urgent pointer (unused)
    seg.extend_from_slice(payload);

    let ck = checksum(src_ip, dst_ip, &seg);
    seg[16..18].copy_from_slice(&ck.to_be_bytes());
    seg
}

/// The TCP checksum: the Internet checksum (RFC 1071, via [`ipv4::checksum`]) computed over the same
/// 12-byte **pseudo-header** UDP uses — {source IP, dest IP, zero, protocol 6, TCP length} — followed by
/// the whole segment. The pseudo-header is a scratch input only; it is never transmitted. `segment` must
/// already carry its checksum field (zero when building; the received value when verifying, in which
/// case a valid segment sums to zero). Unlike UDP there is no "computed 0 becomes 0xFFFF" rule.
pub fn checksum(src_ip: [u8; 4], dst_ip: [u8; 4], segment: &[u8]) -> u16 {
    let mut buf = Vec::with_capacity(12 + segment.len());
    buf.extend_from_slice(&src_ip);
    buf.extend_from_slice(&dst_ip);
    buf.push(0); // reserved zero byte
    buf.push(PROTO_TCP);
    buf.extend_from_slice(&(segment.len() as u16).to_be_bytes()); // TCP length (header + data)
    buf.extend_from_slice(segment);
    ipv4::checksum(&buf)
}

// ---------------------------------------------------------------------------------------------------
// Stage 21b: the connection state machine (the three-way handshake).
//
// TCP is a *stateful* protocol: unlike UDP, each side keeps a small record — a **Transmission Control
// Block (TCB)** — per connection, tracking where it is in the connection's lifecycle and the sequence
// numbers that make the byte stream reliable. Opening a connection is the **three-way handshake**:
//
// ```text
//   active opener (client)                     passive opener (server)
//     CLOSED                                       LISTEN
//       |  --- SYN, seq=x ------------------------->  |
//       |                                    SYN_RECEIVED
//     SYN_SENT                                        |
//       |  <-- SYN, ACK, seq=y, ack=x+1 ------------  |
//       |  --- ACK, seq=x+1, ack=y+1 -------------->  |
//     ESTABLISHED                                 ESTABLISHED
// ```
//
// A **SYN consumes one sequence number** (so the client's next byte is x+1), which is why the peer
// acknowledges x+1. The initial sequence numbers (x, y) are each side's ISS (initial send sequence).
// ---------------------------------------------------------------------------------------------------

/// The receive window we advertise, in bytes. Fixed for now; real flow control comes in a later step.
const DEFAULT_WINDOW: u16 = 64240;

/// The connection lifecycle states this sub-step needs (a subset of RFC 793's eleven — teardown states
/// arrive with the FIN handshake later).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum State {
    Closed,
    Listen,
    SynSent,
    SynReceived,
    Established,
}

/// A Transmission Control Block: everything TCP tracks for one connection. Stage 21b uses only what the
/// handshake needs — the state, the port/address tuple identifying the connection, and the send/receive
/// **sequence bookkeeping** (RFC 793's send and receive sequence spaces). Data buffers, real windows,
/// and retransmission timers come in later sub-steps.
struct Tcb {
    state: State,
    local_port: u16,
    remote_port: u16,
    remote_ip: [u8; 4],
    /// Send space: `una` = oldest unacknowledged seq, `nxt` = next seq to send, `iss` = our initial seq.
    snd_una: u32,
    snd_nxt: u32,
    iss: u32,
    /// Receive space: `nxt` = next seq we expect from the peer, `irs` = the peer's initial seq.
    rcv_nxt: u32,
    irs: u32,
}

/// The connection table. Small and linear — one entry per connection (and one per listener). A real
/// stack keys a hash map by the 4-tuple; a `Vec` scanned by port is plenty for our handful of
/// connections. Behind a `Mutex`, since `poll` (hence `on_segment`) can run from more than one thread.
static CONNECTIONS: Mutex<Vec<Tcb>> = Mutex::new(Vec::new());

/// Generates a distinct initial sequence number per connection. A real stack derives the ISN from a
/// clock so a stale duplicate from an old connection cannot look current; a striding counter suffices
/// here (each connection's sequence space starts far from its neighbors').
static ISN_GEN: AtomicU32 = AtomicU32::new(0x0001_0000);
fn next_isn() -> u32 {
    ISN_GEN.fetch_add(0x0001_0000, Ordering::Relaxed)
}

/// Register a passive open (a **listener**) on `local_port`: a TCB in [`State::Listen`] that will accept
/// an incoming SYN. Idempotent — a second listen on the same port is ignored. (Minimal model: the
/// listener itself becomes the connection on the first SYN, rather than forking a fresh TCB and staying
/// open for more, which is all the loopback handshake test needs.)
pub fn open_passive(local_port: u16) {
    let mut table = CONNECTIONS.lock();
    if table.iter().any(|c| c.state == State::Listen && c.local_port == local_port) {
        return;
    }
    table.push(Tcb {
        state: State::Listen,
        local_port,
        remote_port: 0,
        remote_ip: [0; 4],
        snd_una: 0,
        snd_nxt: 0,
        iss: 0,
        rcv_nxt: 0,
        irs: 0,
    });
}

/// Start an active open: create a TCB in [`State::SynSent`] for the `local_port -> remote_ip:remote_port`
/// connection and return the **SYN segment** to transmit (built with the pseudo-header checksum, so the
/// caller needs `local_ip` for it). The SYN carries our ISS and consumes one sequence number.
pub fn open_active(
    local_ip: [u8; 4],
    remote_ip: [u8; 4],
    local_port: u16,
    remote_port: u16,
) -> Vec<u8> {
    let iss = next_isn();
    let mut table = CONNECTIONS.lock();
    table.push(Tcb {
        state: State::SynSent,
        local_port,
        remote_port,
        remote_ip,
        snd_una: iss,
        snd_nxt: iss.wrapping_add(1), // SYN consumes one sequence number
        iss,
        rcv_nxt: 0, // unknown until the SYN-ACK tells us the peer's ISN
        irs: 0,
    });
    build(local_ip, remote_ip, local_port, remote_port, iss, 0, SYN, DEFAULT_WINDOW, &[])
}

/// Process one received TCP segment against the connection table, advancing the relevant TCB's state
/// machine and returning an optional **response segment** to transmit (a SYN-ACK or ACK during the
/// handshake). `local_ip` is our address and `remote_ip` is the segment's source (both needed for the
/// response checksum). Returns `None` when no segment need be sent. The response is built while the lock
/// is held but the lock is released before this returns, so the caller transmits without holding it.
pub fn on_segment(local_ip: [u8; 4], remote_ip: [u8; 4], seg: &Segment) -> Option<Vec<u8>> {
    let mut table = CONNECTIONS.lock();

    // 1. An existing (non-listening) connection for this exact 4-tuple? Advance its state machine.
    if let Some(idx) = table.iter().position(|c| {
        c.state != State::Listen
            && c.local_port == seg.dst_port
            && c.remote_port == seg.src_port
            && c.remote_ip == remote_ip
    }) {
        return step(&mut table[idx], local_ip, remote_ip, seg);
    }

    // 2. Otherwise, a listener on the destination port accepting a fresh SYN (passive open).
    if let Some(idx) = table
        .iter()
        .position(|c| c.state == State::Listen && c.local_port == seg.dst_port)
    {
        // Only a bare SYN (not SYN-ACK) opens a connection; anything else to a listener is ignored.
        if seg.flags & SYN != 0 && seg.flags & ACK == 0 {
            let iss = next_isn();
            let tcb = &mut table[idx];
            tcb.remote_ip = remote_ip;
            tcb.remote_port = seg.src_port;
            tcb.irs = seg.seq;
            tcb.rcv_nxt = seg.seq.wrapping_add(1); // the peer's SYN consumes one seq
            tcb.iss = iss;
            tcb.snd_una = iss;
            tcb.snd_nxt = iss.wrapping_add(1); // our SYN consumes one seq
            tcb.state = State::SynReceived;
            return Some(build(
                local_ip,
                remote_ip,
                tcb.local_port,
                tcb.remote_port,
                iss,
                tcb.rcv_nxt,
                SYN | ACK,
                DEFAULT_WINDOW,
                &[],
            ));
        }
    }

    // 3. No connection and no listener — a real stack would reply RST; we simply drop it for now.
    None
}

/// Advance one established-or-half-open connection by one received segment. Handshake-only for Stage
/// 21b: complete the active open (SYN-ACK -> send ACK, ESTABLISHED) and the passive open (final ACK ->
/// ESTABLISHED). Data segments in [`State::Established`] are ignored until the data-transfer sub-step.
fn step(tcb: &mut Tcb, local_ip: [u8; 4], remote_ip: [u8; 4], seg: &Segment) -> Option<Vec<u8>> {
    match tcb.state {
        State::SynSent => {
            // Expect the SYN-ACK: it must acknowledge our SYN (ack == snd_nxt == iss + 1).
            if seg.flags & SYN != 0 && seg.flags & ACK != 0 && seg.ack == tcb.snd_nxt {
                tcb.irs = seg.seq;
                tcb.rcv_nxt = seg.seq.wrapping_add(1);
                tcb.snd_una = seg.ack;
                tcb.state = State::Established;
                // Complete the handshake with the final ACK.
                return Some(build(
                    local_ip,
                    remote_ip,
                    tcb.local_port,
                    tcb.remote_port,
                    tcb.snd_nxt,
                    tcb.rcv_nxt,
                    ACK,
                    DEFAULT_WINDOW,
                    &[],
                ));
            }
            None
        }
        State::SynReceived => {
            // A retransmitted SYN (the peer never got our SYN-ACK): resend the SYN-ACK, same ISN.
            if seg.flags & SYN != 0 && seg.flags & ACK == 0 && seg.seq == tcb.irs {
                return Some(build(
                    local_ip,
                    remote_ip,
                    tcb.local_port,
                    tcb.remote_port,
                    tcb.iss,
                    tcb.rcv_nxt,
                    SYN | ACK,
                    DEFAULT_WINDOW,
                    &[],
                ));
            }
            // Otherwise expect the final ACK that acknowledges our SYN-ACK.
            if seg.flags & ACK != 0 && seg.ack == tcb.snd_nxt {
                tcb.snd_una = seg.ack;
                tcb.state = State::Established;
            }
            None
        }
        _ => None, // Established (data) and teardown states are handled in later sub-steps
    }
}

/// The state of the connection identified by `(local_port, remote_port)`, or `None` if there is no such
/// TCB. Used by the connection driver in `net` to wait for [`State::Established`], and by tests.
pub fn connection_state(local_port: u16, remote_port: u16) -> Option<State> {
    CONNECTIONS
        .lock()
        .iter()
        .find(|c| c.local_port == local_port && c.remote_port == remote_port)
        .map(|c| c.state)
}

/// How many connections are currently [`State::Established`] (both ends of a loopback handshake count).
pub fn established_count() -> usize {
    CONNECTIONS
        .lock()
        .iter()
        .filter(|c| c.state == State::Established)
        .count()
}

/// Drop all connections (and listeners). Used to isolate the loopback self-test/tests from each other.
pub fn reset_connections() {
    CONNECTIONS.lock().clear();
}
