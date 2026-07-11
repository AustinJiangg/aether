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

// Stage 21e: retransmission-timer constants. Time is measured in timer ticks (`interrupts::timer_ticks`,
// 100 Hz, so 1 tick = 10 ms). A real stack estimates the retransmission timeout from measured round-trip
// times (RFC 6298); fixed, short values are enough to *demonstrate* recovery deterministically.

/// Initial retransmission timeout: resend an unacknowledged segment after this many ticks (~150 ms).
const RTO_TICKS: u64 = 15;
/// Give up (abort the connection) after resending the same segment this many times.
const MAX_RETRIES: u32 = 5;
/// How long the active closer lingers in TIME_WAIT before closing (~300 ms). A real stack waits 2*MSL
/// (minutes) to be sure a lost final ACK could be retransmitted and old duplicates have died out.
const TIME_WAIT_TICKS: u64 = 30;

/// The current time, in timer ticks since boot. TCP reads the global monotonic tick counter directly so
/// the send/close/tick paths need not thread a clock value through their signatures.
fn now_ticks() -> u64 {
    crate::interrupts::timer_ticks()
}

/// TCP sequence comparison (RFC 1982): `a <= b` within the 32-bit *wrapping* sequence space — true when
/// `b` is at or ahead of `a` in the near half. Used to decide when `snd_una` has reached a segment's end.
fn seq_leq(a: u32, b: u32) -> bool {
    b.wrapping_sub(a) < 0x8000_0000
}

/// Stage 21e: one unacknowledged outbound segment, retained so the retransmission timer can resend it if
/// its ACK does not arrive in time. Dropped from the connection's queue once `snd_una` reaches `end_seq`.
struct Unacked {
    /// The send sequence number just past this segment (its data/flags end); acknowledged once `snd_una`
    /// reaches or passes it (a FIN, like a SYN, contributes one to this beyond any data).
    end_seq: u32,
    /// Absolute tick at which to resend if still unacknowledged.
    deadline: u64,
    /// How many times it has already been resent — bounds retries and drives exponential backoff.
    tries: u32,
    /// The exact segment bytes (already checksummed) to retransmit verbatim.
    segment: Vec<u8>,
}

/// The connection lifecycle states (RFC 793). The opening subset — `Closed`/`Listen`/`SynSent`/
/// `SynReceived`/`Established` — is the three-way handshake (Stage 21b); the rest are the **teardown**
/// states of the FIN handshake (Stage 21d). TCP is full-duplex, so each direction closes independently:
/// the active closer walks `Established -> FinWait1 -> FinWait2 -> TimeWait -> Closed` (or via `Closing`
/// on a simultaneous close), and the passive closer walks `Established -> CloseWait -> LastAck -> Closed`.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum State {
    Closed,
    Listen,
    SynSent,
    SynReceived,
    Established,
    /// Active close: we sent a FIN, awaiting its ACK (and possibly the peer's FIN).
    FinWait1,
    /// Active close: our FIN is acknowledged; awaiting the peer's FIN.
    FinWait2,
    /// Passive close: the peer sent a FIN (we acknowledged it); awaiting our application's own close.
    CloseWait,
    /// Simultaneous close: both sides sent a FIN before either was acknowledged; awaiting our FIN's ACK.
    Closing,
    /// Passive close: we sent our FIN, awaiting its final ACK to complete the close.
    LastAck,
    /// Active close: the FIN handshake is done; linger 2*MSL (so a lost final ACK can be resent) before
    /// truly closing. The timed transition to `Closed` arrives with the timers in Stage 21e.
    TimeWait,
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
    /// Stage 21c: the **receive buffer** — in-order stream bytes we have accepted and acknowledged but
    /// the application has not yet consumed. Data arriving with `seq == rcv_nxt` is appended here; a real
    /// socket `read` would drain it.
    rx: Vec<u8>,
    /// Stage 21e: the **retransmission queue** — outbound segments sent but not yet acknowledged, oldest
    /// first, kept so the timer can resend them if an ACK is late. Emptied as `snd_una` advances.
    retransmit: Vec<Unacked>,
    /// Stage 21e: the tick at which a TIME_WAIT connection may finally close (set on entering TIME_WAIT).
    time_wait_deadline: u64,
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
        rx: Vec::new(),
        retransmit: Vec::new(),
        time_wait_deadline: 0,
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
        rx: Vec::new(),
        retransmit: Vec::new(),
        time_wait_deadline: 0,
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
        State::Established => on_established(tcb, local_ip, remote_ip, seg),

        // --- Stage 21d: the FIN handshake (connection teardown). Each direction closes independently, so
        // the machine tracks our FIN's acknowledgement (snd_una catching up to snd_nxt, since our FIN
        // advanced snd_nxt) and the peer's FIN (which consumes one of its sequence numbers). ---

        // We sent a FIN (active close) and await its ACK — and possibly the peer's FIN too.
        State::FinWait1 => {
            process_ack(tcb, seg);
            let our_fin_acked = tcb.snd_una == tcb.snd_nxt;
            let peer_fin = seg.flags & FIN != 0 && seg.seq == tcb.rcv_nxt;
            if peer_fin {
                tcb.rcv_nxt = tcb.rcv_nxt.wrapping_add(1); // the peer's FIN consumes one seq
            }
            match (our_fin_acked, peer_fin) {
                // Our FIN is acked and the peer has finished too: ACK its FIN and linger in TIME_WAIT.
                (true, true) => {
                    enter_time_wait(tcb);
                    Some(build_ack(tcb, local_ip, remote_ip))
                }
                // Our FIN is acked; wait for the peer's FIN in FIN_WAIT_2.
                (true, false) => {
                    tcb.state = State::FinWait2;
                    None
                }
                // Simultaneous close: the peer's FIN arrived before ours was acked. ACK it, wait (in
                // CLOSING) for the ACK of our own FIN.
                (false, true) => {
                    tcb.state = State::Closing;
                    Some(build_ack(tcb, local_ip, remote_ip))
                }
                (false, false) => None,
            }
        }

        // Our FIN is acked; wait for the peer's FIN, then acknowledge it and enter TIME_WAIT.
        State::FinWait2 => {
            process_ack(tcb, seg);
            if seg.flags & FIN != 0 && seg.seq == tcb.rcv_nxt {
                tcb.rcv_nxt = tcb.rcv_nxt.wrapping_add(1);
                enter_time_wait(tcb);
                return Some(build_ack(tcb, local_ip, remote_ip));
            }
            None
        }

        // The peer closed first (we are the passive closer) and we already acknowledged its FIN. Wait for
        // our application to close (`close`, which sends our FIN -> LAST_ACK); meanwhile re-ACK a
        // retransmitted FIN in case the peer never saw our acknowledgement.
        State::CloseWait => {
            process_ack(tcb, seg);
            if seg.flags & FIN != 0 {
                return Some(build_ack(tcb, local_ip, remote_ip));
            }
            None
        }

        // Simultaneous close: both FINs are outstanding. Once ours is acked, enter TIME_WAIT.
        State::Closing => {
            process_ack(tcb, seg);
            if tcb.snd_una == tcb.snd_nxt {
                enter_time_wait(tcb);
            }
            None
        }

        // Passive closer: we sent our FIN and await its final ACK, which completes the close.
        State::LastAck => {
            process_ack(tcb, seg);
            if tcb.snd_una == tcb.snd_nxt {
                tcb.state = State::Closed;
            }
            None
        }

        // Active closer, waiting out 2*MSL. Re-ACK a retransmitted peer FIN (our final ACK was lost); the
        // timed transition to CLOSED belongs with the retransmission timers (Stage 21e).
        State::TimeWait => {
            if seg.flags & FIN != 0 {
                return Some(build_ack(tcb, local_ip, remote_ip));
            }
            None
        }

        State::Closed | State::Listen => None, // never reached via step (Listen is handled in on_segment)
    }
}

/// Build a bare ACK for this connection's current sequence state (`seq = snd_nxt`, `ack = rcv_nxt`). The
/// workhorse reply of every acknowledging state — data receipt, a FIN, or a re-ACK of a retransmission.
fn build_ack(tcb: &Tcb, local_ip: [u8; 4], remote_ip: [u8; 4]) -> Vec<u8> {
    build(
        local_ip,
        remote_ip,
        tcb.local_port,
        tcb.remote_port,
        tcb.snd_nxt,
        tcb.rcv_nxt,
        ACK,
        DEFAULT_WINDOW,
        &[],
    )
}

/// Process an incoming acknowledgement: advance `snd_una` over the bytes the peer's `ack` newly confirms.
/// Accepts an ack in `(snd_una, snd_nxt]` using wrapping arithmetic (so it stays correct across a
/// sequence-number wrap); a duplicate ack (`ack == snd_una`) is a harmless no-op, and an ack beyond
/// `snd_nxt` is ignored. After a FIN we sent, `snd_una == snd_nxt` therefore means our FIN was acked.
fn process_ack(tcb: &mut Tcb, seg: &Segment) {
    if seg.flags & ACK == 0 {
        return;
    }
    let acked = seg.ack.wrapping_sub(tcb.snd_una);
    let outstanding = tcb.snd_nxt.wrapping_sub(tcb.snd_una);
    if acked <= outstanding {
        tcb.snd_una = seg.ack;
        // Stage 21e: drop every queued segment the peer has now cumulatively acknowledged, so the
        // retransmission timer stops resending it.
        let una = tcb.snd_una;
        tcb.retransmit.retain(|u| !seq_leq(u.end_seq, una));
    }
}

/// Move a connection into TIME_WAIT and stamp its 2*MSL linger deadline (Stage 21e), after which
/// [`on_tick`] closes it. The active closer lingers here so a lost final ACK can still be retransmitted.
fn enter_time_wait(tcb: &mut Tcb) {
    tcb.state = State::TimeWait;
    tcb.time_wait_deadline = now_ticks() + TIME_WAIT_TICKS;
}

/// Handle a segment on an ESTABLISHED connection — the steady state (Stage 21c) plus the *start* of a
/// passive close (Stage 21d). In order per segment: process any acknowledgement of our sent data; accept
/// in-order stream data into the receive buffer; and, if the segment carries a **FIN** (the peer is done
/// sending), consume its sequence number and move to CLOSE_WAIT. Anything that consumed sequence space
/// (data or a FIN) is acknowledged; a bare ACK of our own data needs no reply (acknowledging an
/// acknowledgement would loop forever).
fn on_established(
    tcb: &mut Tcb,
    local_ip: [u8; 4],
    remote_ip: [u8; 4],
    seg: &Segment,
) -> Option<Vec<u8>> {
    process_ack(tcb, seg);

    // Accept stream data, in order only: a real stack would queue out-of-order data for reassembly, but
    // requiring `seq == rcv_nxt` keeps the byte stream simple and still correct — a gap is left
    // unacknowledged until the peer retransmits it (Stage 21e).
    let in_order = seg.seq == tcb.rcv_nxt;
    if in_order && !seg.payload.is_empty() {
        tcb.rx.extend_from_slice(seg.payload);
        tcb.rcv_nxt = tcb.rcv_nxt.wrapping_add(seg.payload.len() as u32);
    }

    // A FIN occupies the sequence number just past the segment's data; honor it only in order. It moves
    // us to CLOSE_WAIT — the peer will send no more data, though our side may still send until the
    // application closes too (`close`, which then sends our own FIN).
    let fin = seg.flags & FIN != 0 && seg.seq.wrapping_add(seg.payload.len() as u32) == tcb.rcv_nxt;
    if fin {
        tcb.rcv_nxt = tcb.rcv_nxt.wrapping_add(1); // the FIN consumes one sequence number
        tcb.state = State::CloseWait;
    }

    // Acknowledge anything that consumed sequence space (data or a FIN); also re-ACK a duplicate segment
    // to prompt the peer. A pure ACK of our own data (no payload, no FIN) needs no reply.
    if fin || !seg.payload.is_empty() {
        Some(build_ack(tcb, local_ip, remote_ip))
    } else {
        None
    }
}

/// Stage 21c: queue application data to send on the ESTABLISHED connection `(local_port -> remote_port)`,
/// and return the **data segment** to transmit together with the peer's IP (which the caller needs to
/// frame it). The segment carries `seq = snd_nxt` (the next unsent stream position), acknowledges the
/// peer up to `rcv_nxt`, and sets `PSH | ACK` ("deliver this data now"); `snd_nxt` then advances over the
/// bytes. `local_ip` is needed for the pseudo-header checksum. Returns `None` if there is no such
/// established connection. (No retransmission buffer yet — Stage 21e keeps the unacked bytes for resend.)
pub fn send_data(
    local_ip: [u8; 4],
    local_port: u16,
    remote_port: u16,
    data: &[u8],
) -> Option<(Vec<u8>, [u8; 4])> {
    let mut table = CONNECTIONS.lock();
    let tcb = table.iter_mut().find(|c| {
        c.state == State::Established && c.local_port == local_port && c.remote_port == remote_port
    })?;
    let seg = build(
        local_ip,
        tcb.remote_ip,
        tcb.local_port,
        tcb.remote_port,
        tcb.snd_nxt,
        tcb.rcv_nxt,
        PSH | ACK,
        DEFAULT_WINDOW,
        data,
    );
    tcb.snd_nxt = tcb.snd_nxt.wrapping_add(data.len() as u32);
    // Stage 21e: queue this segment for retransmission until the peer acknowledges it.
    if !data.is_empty() {
        tcb.retransmit.push(Unacked {
            end_seq: tcb.snd_nxt,
            deadline: now_ticks() + RTO_TICKS,
            tries: 0,
            segment: seg.clone(),
        });
    }
    Some((seg, tcb.remote_ip))
}

/// Stage 21d: close our end of the connection `(local_port -> remote_port)` — the application is done
/// sending. Returns the **FIN segment** to transmit (with the peer's IP for framing), or `None` if the
/// connection is not in a closable state. A FIN consumes one sequence number, like a SYN. Two cases:
///
/// - From ESTABLISHED this is an **active close**: send our FIN and enter FIN_WAIT_1.
/// - From CLOSE_WAIT (the peer already closed and we acknowledged its FIN) this is the **passive close**:
///   send our FIN and enter LAST_ACK.
pub fn close(local_ip: [u8; 4], local_port: u16, remote_port: u16) -> Option<(Vec<u8>, [u8; 4])> {
    let mut table = CONNECTIONS.lock();
    let tcb = table
        .iter_mut()
        .find(|c| c.local_port == local_port && c.remote_port == remote_port)?;
    let next_state = match tcb.state {
        State::Established => State::FinWait1, // active close
        State::CloseWait => State::LastAck,    // passive close (the peer already sent its FIN)
        _ => return None,                      // not in a closable state (not established / already closing)
    };
    let seg = build(
        local_ip,
        tcb.remote_ip,
        tcb.local_port,
        tcb.remote_port,
        tcb.snd_nxt,
        tcb.rcv_nxt,
        FIN | ACK,
        DEFAULT_WINDOW,
        &[],
    );
    tcb.snd_nxt = tcb.snd_nxt.wrapping_add(1); // the FIN consumes one sequence number
    // Stage 21e: queue the FIN for retransmission too — a lost FIN would otherwise stall teardown.
    tcb.retransmit.push(Unacked {
        end_seq: tcb.snd_nxt,
        deadline: now_ticks() + RTO_TICKS,
        tries: 0,
        segment: seg.clone(),
    });
    tcb.state = next_state;
    Some((seg, tcb.remote_ip))
}

/// Stage 21c: a clone of the receive buffer for `(local_port, remote_port)` — the in-order stream bytes
/// accepted and acknowledged but not yet consumed. `None` if no such connection. A real socket `read`
/// would drain these bytes; a clone is enough for the self-test and tests to inspect what arrived.
pub fn received_data(local_port: u16, remote_port: u16) -> Option<Vec<u8>> {
    CONNECTIONS
        .lock()
        .iter()
        .find(|c| c.local_port == local_port && c.remote_port == remote_port)
        .map(|c| c.rx.clone())
}

/// Stage 21c: whether every byte we have sent on `(local_port, remote_port)` has been acknowledged by the
/// peer (`snd_una == snd_nxt`, i.e. nothing outstanding). `None` if no such connection.
pub fn all_data_acked(local_port: u16, remote_port: u16) -> Option<bool> {
    CONNECTIONS
        .lock()
        .iter()
        .find(|c| c.local_port == local_port && c.remote_port == remote_port)
        .map(|c| c.snd_una == c.snd_nxt)
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

/// Stage 21e: service the connection timers — call once per [`super::poll`]. For every connection whose
/// oldest unacknowledged segment's deadline has passed, resend it (with exponential backoff, aborting the
/// connection after [`MAX_RETRIES`]); and expire any TIME_WAIT connection whose linger has elapsed,
/// moving it to CLOSED. Returns the segments to retransmit, each paired with the peer IP for framing —
/// built under the lock, transmitted by the caller after it is released (as [`on_segment`] does).
pub fn on_tick() -> Vec<(Vec<u8>, [u8; 4])> {
    let now = now_ticks();
    let mut resends = Vec::new();
    let mut table = CONNECTIONS.lock();
    for tcb in table.iter_mut() {
        // Expire a TIME_WAIT connection once its 2*MSL linger has elapsed.
        if tcb.state == State::TimeWait && now >= tcb.time_wait_deadline {
            tcb.state = State::Closed;
            tcb.retransmit.clear();
            continue;
        }
        // Retransmit the oldest unacknowledged segment if its deadline has passed.
        if let Some(u) = tcb.retransmit.first_mut() {
            if now >= u.deadline {
                if u.tries >= MAX_RETRIES {
                    // Too many attempts: give up and abort the connection (a real stack sends RST).
                    tcb.state = State::Closed;
                    tcb.retransmit.clear();
                    continue;
                }
                u.tries += 1;
                // Exponential backoff, capped: each successive resend waits twice as long.
                u.deadline = now + (RTO_TICKS << core::cmp::min(u.tries, 6));
                resends.push((u.segment.clone(), tcb.remote_ip));
            }
        }
    }
    resends
}

/// Drop all connections (and listeners). Used to isolate the loopback self-test/tests from each other.
pub fn reset_connections() {
    CONNECTIONS.lock().clear();
}
