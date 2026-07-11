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
use core::sync::atomic::{AtomicU32, AtomicU64, Ordering};

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

/// Stage 22b: the maximum receive buffer, in bytes — the largest window we ever advertise. Kept modest
/// (a real stack uses tens of KiB, auto-tuned) so a self-test can fill it and drive the window to zero
/// cheaply, but large enough to hold **several `MSS`-sized segments** — Stage 22d-3's fast-retransmit test
/// needs four segments in flight at once (one lost, three later ones each triggering a duplicate ACK), so a
/// two-segment window would be too small. The window we actually advertise on each segment is this minus the
/// unread bytes already buffered (`recv_window`), so it shrinks as data piles up unread and reopens on a read.
const RCV_WINDOW_MAX: usize = 8192;

/// Stage 22b: the receive window to advertise given `rx_used` unread bytes already buffered — the free
/// space left in the receive buffer, clamped to the 16-bit window field. Zero means "stop sending".
fn window_for(rx_used: usize) -> u16 {
    RCV_WINDOW_MAX.saturating_sub(rx_used).min(u16::MAX as usize) as u16
}

/// Stage 22b: the window this connection currently advertises — free space in its receive buffer.
fn recv_window(tcb: &Tcb) -> u16 {
    window_for(tcb.rx.len())
}

/// Stage 22c: the **maximum segment size** — the most stream data we put in one segment. Capped well under
/// the link's frame budget (an e1000 receive buffer is 2048 bytes, minus the 14+20+20-byte Ethernet/IP/TCP
/// headers), so a segment always fits one frame; it also forces a large send to be *segmented*, exercising
/// the sender's windowing. A real stack learns the peer's MSS from a SYN option; we fix it.
const MSS: usize = 1024;

// Stage 22d: congestion-control constants. Where flow control (Stage 22c) paces the sender to the
// *receiver's* buffer via `snd_wnd`, congestion control paces it to the *network's* capacity via a second
// window, `cwnd`. The sender never puts more than `min(snd_wnd, cwnd)` bytes in flight, so whichever is
// smaller — a slow receiver or a congested network — governs. `cwnd` is not told to us; the sender infers
// it, growing it on every successful ACK and (Stage 22d-2) shrinking it on loss.

/// The **initial congestion window** — where slow start begins. RFC 5681/3390 permit a larger initial
/// window (~4 MSS), but we start at one MSS so the exponential ramp is clearly visible over a small
/// transfer and so `cwnd` (not the tiny loopback receive window) is the binding limit early on.
const INIT_CWND: u32 = MSS as u32;

/// The **initial slow-start threshold**: the `cwnd` boundary between slow start (below it, exponential
/// growth) and congestion avoidance (at/above it, linear growth). RFC 5681 says it SHOULD start
/// arbitrarily high, so a fresh connection begins in slow start and stays there until the first loss
/// lowers it (Stage 22d-2). Until then the congestion-avoidance branch of [`grow_cwnd`] is unreachable.
const INIT_SSTHRESH: u32 = u32::MAX;

/// Stage 22d-3: how many **duplicate ACKs** (ACKs that acknowledge no new data, so the peer is still stuck
/// waiting for the same byte) trigger a **fast retransmit** — resend the missing segment at once, without
/// waiting for the RTO. Three is the classic RFC 5681 value: one or two dup ACKs may just be reordering
/// (a later segment overtaking an earlier one), but a third strongly implies the earlier segment was lost.
const DUP_ACK_THRESHOLD: u32 = 3;

// Retransmission-timer constants. Time is measured in timer ticks (`interrupts::timer_ticks`, 100 Hz, so
// 1 tick = 10 ms). Stage 21e used a single fixed RTO; Stage 23a replaces it with an RTO *estimated* per
// connection from measured round-trip times (RFC 6298), bounded by these floor/ceiling constants.

/// The retransmission timeout before any RTT has been measured (~150 ms). RFC 6298 §2.1 suggests 1 s; a
/// shorter default keeps the loopback self-tests fast while a real transfer quickly measures its own RTO.
const RTO_INITIAL_TICKS: u32 = 15;
/// The RTO floor. Our clock granularity is a whole tick (10 ms) and loopback RTTs are sub-millisecond, so a
/// computed RTO would round toward zero — this floor keeps it sane (and the retransmit tests fast). RFC
/// 6298 uses a 1 s minimum; we use a shorter one for the same reason as [`RTO_INITIAL_TICKS`].
const RTO_MIN_TICKS: u32 = 15;
/// The RTO ceiling (~60 s), so a wildly varying RTT estimate cannot push the timer arbitrarily far out.
const RTO_MAX_TICKS: u32 = 6000;
/// Give up (abort the connection) after resending the same segment this many times.
const MAX_RETRIES: u32 = 5;

/// Stage 23b: how long a **delayed ACK** may wait before it must be sent. RFC 1122 caps this at 500 ms; we
/// use a shorter 50 ms so it stays well under the RTO (a delayed ACK must arrive before the sender times
/// out) and keeps the loopback tests fast. The "ACK every second segment" rule usually fires first, so the
/// timer only matters for a lone segment with no follow-up.
const DELAYED_ACK_TICKS: u64 = 5;
/// How long the active closer lingers in TIME_WAIT before closing (~300 ms). A real stack waits 2*MSL
/// (minutes) to be sure a lost final ACK could be retransmitted and old duplicates have died out.
const TIME_WAIT_TICKS: u64 = 30;

/// Stage 22a: cap on how many out-of-order segments a connection buffers while waiting for a gap to fill.
/// A real stack bounds its reassembly queue by the advertised receive window; a small fixed cap is enough
/// here, and a full queue simply drops further out-of-order segments (the peer retransmits them later).
const MAX_OOO_SEGMENTS: usize = 16;

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

/// Strict version of [`seq_leq`]: `a < b` in the wrapping sequence space (Stage 22a reassembly).
fn seq_lt(a: u32, b: u32) -> bool {
    a != b && seq_leq(a, b)
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
    /// Stage 23a: the tick at which this segment was *first* sent. When the segment is acknowledged and was
    /// never retransmitted (`tries == 0`, Karn's algorithm), `now - sent_at` is a clean RTT sample.
    sent_at: u64,
    /// The exact segment bytes (already checksummed) to retransmit verbatim.
    segment: Vec<u8>,
}

/// Stage 22a: one out-of-order segment's payload, held in the receive **reassembly queue** because a gap
/// precedes it (`seq > rcv_nxt`). It is spliced into the stream once the missing bytes arrive and
/// `rcv_nxt` reaches it. `seq` is the stream position of `data[0]`.
struct OutOfOrder {
    seq: u32,
    data: Vec<u8>,
}

/// Stage 22a: how many out-of-order segments have been buffered for later reassembly, over the whole run.
/// A test reads this to confirm the reorder path was actually exercised (not merely in-order delivery).
static OOO_BUFFERED: AtomicU64 = AtomicU64::new(0);

/// Total out-of-order segments buffered since boot (Stage 22a; see [`OOO_BUFFERED`]).
pub fn out_of_order_buffered() -> u64 {
    OOO_BUFFERED.load(Ordering::Relaxed)
}

/// Stage 22c: how many data-carrying segments [`flush`] has emitted, over the whole run — a test reads it
/// to confirm a large send was actually *segmented* into multiple MSS-sized pieces (not sent whole).
static DATA_SEGMENTS_SENT: AtomicU64 = AtomicU64::new(0);

/// Total data segments the sender has emitted since boot (Stage 22c; see [`DATA_SEGMENTS_SENT`]).
pub fn data_segments_sent() -> u64 {
    DATA_SEGMENTS_SENT.load(Ordering::Relaxed)
}

/// Stage 22d-3: how many **fast retransmits** have fired since boot — a segment resent on the third
/// duplicate ACK rather than on a retransmission timeout. A test reads it to confirm the fast path ran.
static FAST_RETRANSMITS: AtomicU64 = AtomicU64::new(0);

/// Total fast retransmits since boot (Stage 22d-3; see [`FAST_RETRANSMITS`]).
pub fn fast_retransmits() -> u64 {
    FAST_RETRANSMITS.load(Ordering::Relaxed)
}

/// Stage 23b: how many ACK segments the stack has built since boot ([`build_ack`]). The delayed-ACK test
/// reads it to confirm the receiver sent *fewer* ACKs than it received data segments (i.e. it coalesced).
static ACKS_SENT: AtomicU64 = AtomicU64::new(0);

/// Total ACK segments built since boot (Stage 23b; see [`ACKS_SENT`]).
pub fn acks_sent() -> u64 {
    ACKS_SENT.load(Ordering::Relaxed)
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
    /// Stage 22c: the peer's most recently advertised **receive window** — how many bytes past `snd_una`
    /// the peer is currently willing to accept. The sender never puts more than this many bytes in flight
    /// (`snd_nxt - snd_una <= snd_wnd`). Learned from the `window` field of every acceptable segment.
    snd_wnd: u32,
    /// Stage 22c: the **send buffer** — application bytes queued to send but not yet transmitted (because
    /// the window had no room). [`flush`] drains it into segments as the window allows.
    snd_buf: Vec<u8>,
    /// Stage 22d: the **congestion window** — the sender's estimate of how much data the *network* (not the
    /// peer) can absorb without loss. [`flush`] limits in-flight data to `min(snd_wnd, cwnd)`, so `cwnd`
    /// paces to congestion while `snd_wnd` paces to the receiver. Grows via slow start / congestion
    /// avoidance ([`grow_cwnd`]) as ACKs arrive, and collapses on loss (Stage 22d-2).
    cwnd: u32,
    /// Stage 22d: the **slow-start threshold** — the `cwnd` value at which the sender switches from slow
    /// start's exponential growth to congestion avoidance's linear growth. Starts high (see
    /// [`INIT_SSTHRESH`]) and is lowered to about half the flight on a loss.
    ssthresh: u32,
    /// Stage 22d-3: count of consecutive **duplicate ACKs** received (each acknowledging no new data while
    /// data is still outstanding). Reset to zero by any ACK that advances `snd_una`. Reaching
    /// [`DUP_ACK_THRESHOLD`] fires a fast retransmit and enters fast recovery.
    dup_acks: u32,
    /// Stage 22d-3: whether the connection is in **fast recovery** — the window between a fast retransmit and
    /// the ACK that acknowledges the recovered data. While set, each further dup ACK inflates `cwnd` by one
    /// MSS; the first new ACK deflates `cwnd` back to `ssthresh` and clears this.
    in_fast_recovery: bool,
    /// Stage 23a: the RFC 6298 RTT estimator. `srtt` is the smoothed RTT scaled by 8 (three fractional
    /// bits) and `rttvar` is the RTT variation scaled by 4 (two fractional bits) — the classic integer
    /// representation that avoids floating point. `rto` is the current retransmission timeout (in ticks),
    /// recomputed from them on each RTT sample; `rtt_valid` is false until the first sample initializes them.
    srtt: u32,
    rttvar: u32,
    rto: u32,
    rtt_valid: bool,
    /// Stage 23b: in-order data segments received but not yet acknowledged — the "ACK every second segment"
    /// counter (RFC 1122). Reset to zero whenever an ACK is sent (immediate or delayed).
    unacked_segs: u32,
    /// Stage 23b: the tick by which a pending **delayed ACK** must be sent, or 0 if none is pending.
    delayed_ack_deadline: u64,
    /// Stage 23c: whether **Nagle's algorithm is disabled** (`TCP_NODELAY`). With Nagle on (the default,
    /// `false`), a sub-MSS segment is held while earlier data is unacknowledged, coalescing small writes;
    /// with it disabled, every write is sent at once (for latency-sensitive traffic).
    nodelay: bool,
    /// Receive space: `nxt` = next seq we expect from the peer, `irs` = the peer's initial seq.
    rcv_nxt: u32,
    irs: u32,
    /// Stage 21c: the **receive buffer** — in-order stream bytes we have accepted and acknowledged but
    /// the application has not yet consumed. Data arriving with `seq == rcv_nxt` is appended here; a real
    /// socket `read` would drain it.
    rx: Vec<u8>,
    /// Stage 22a: the **out-of-order reassembly queue** — segments accepted with `seq > rcv_nxt` (a gap
    /// precedes them), held until the missing bytes arrive so they can be spliced into `rx` in order.
    ooo: Vec<OutOfOrder>,
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
        snd_wnd: 0,
        snd_buf: Vec::new(),
        cwnd: INIT_CWND,
        ssthresh: INIT_SSTHRESH,
        dup_acks: 0,
        in_fast_recovery: false,
        srtt: 0,
        rttvar: 0,
        rto: RTO_INITIAL_TICKS,
        rtt_valid: false,
        unacked_segs: 0,
        delayed_ack_deadline: 0,
        nodelay: false,
        rcv_nxt: 0,
        irs: 0,
        rx: Vec::new(),
        ooo: Vec::new(),
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
        snd_wnd: 0, // unknown until the SYN-ACK advertises the peer's window
        snd_buf: Vec::new(),
        cwnd: INIT_CWND,
        ssthresh: INIT_SSTHRESH,
        dup_acks: 0,
        in_fast_recovery: false,
        srtt: 0,
        rttvar: 0,
        rto: RTO_INITIAL_TICKS,
        rtt_valid: false,
        unacked_segs: 0,
        delayed_ack_deadline: 0,
        nodelay: false,
        rcv_nxt: 0, // unknown until the SYN-ACK tells us the peer's ISN
        irs: 0,
        rx: Vec::new(),
        ooo: Vec::new(),
        retransmit: Vec::new(),
        time_wait_deadline: 0,
    });
    build(local_ip, remote_ip, local_port, remote_port, iss, 0, SYN, window_for(0), &[])
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
            tcb.snd_wnd = seg.window as u32; // learn the peer's receive window from its SYN (Stage 22c)
            tcb.state = State::SynReceived;
            return Some(build(
                local_ip,
                remote_ip,
                tcb.local_port,
                tcb.remote_port,
                iss,
                tcb.rcv_nxt,
                SYN | ACK,
                recv_window(tcb),
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
                tcb.snd_wnd = seg.window as u32; // learn the peer's window from its SYN-ACK (Stage 22c)
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
                    recv_window(tcb),
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
                    recv_window(tcb),
                    &[],
                ));
            }
            // Otherwise expect the final ACK that acknowledges our SYN-ACK.
            if seg.flags & ACK != 0 && seg.ack == tcb.snd_nxt {
                tcb.snd_una = seg.ack;
                tcb.snd_wnd = seg.window as u32; // learn the peer's window from the final ACK (Stage 22c)
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
    ACKS_SENT.fetch_add(1, Ordering::Relaxed); // Stage 23b: count ACKs so a test can see delayed ACK coalesce them
    build(
        local_ip,
        remote_ip,
        tcb.local_port,
        tcb.remote_port,
        tcb.snd_nxt,
        tcb.rcv_nxt,
        ACK,
        recv_window(tcb),
        &[],
    )
}

/// Process an incoming acknowledgement: advance `snd_una` over the bytes the peer's `ack` newly confirms.
/// Accepts an ack in `(snd_una, snd_nxt]` using wrapping arithmetic (so it stays correct across a
/// sequence-number wrap); a duplicate ack (`ack == snd_una`) is a harmless no-op, and an ack beyond
/// `snd_nxt` is ignored. After a FIN we sent, `snd_una == snd_nxt` therefore means our FIN was acked.
/// Process an incoming acknowledgement (see the field-level comments). Returns `true` when the third
/// duplicate ACK just fired a **fast retransmit** (Stage 22d-3), so the caller should resend the missing
/// segment now; `false` otherwise.
fn process_ack(tcb: &mut Tcb, seg: &Segment) -> bool {
    if seg.flags & ACK == 0 {
        return false;
    }
    // Stage 22c: track the peer's advertised window from every acceptable ACK (including a pure
    // window-update / duplicate ACK), so the sender's [`flush`] paces to it.
    tcb.snd_wnd = seg.window as u32;
    let acked = seg.ack.wrapping_sub(tcb.snd_una);
    let outstanding = tcb.snd_nxt.wrapping_sub(tcb.snd_una);
    let mss = MSS as u32;

    // Stage 22d-3: a *duplicate* ACK — acknowledges no new data (`acked == 0`) while data is still
    // outstanding and carries no payload/SYN/FIN — hints that a later segment reached the peer but an
    // earlier one is missing. Count consecutive ones; the third fires a fast retransmit + fast recovery.
    if acked == 0 && outstanding > 0 && seg.payload.is_empty() && seg.flags & (SYN | FIN) == 0 {
        tcb.dup_acks += 1;
        if tcb.dup_acks == DUP_ACK_THRESHOLD {
            // Multiplicative decrease, but gentler on `cwnd` than an RTO: halve `ssthresh` to the flight
            // size, then set `cwnd` to `ssthresh` plus the three segments that have already left the network
            // (the three dup ACKs). Tell the caller to retransmit the missing segment now, not at the RTO.
            tcb.ssthresh = core::cmp::max(outstanding / 2, 2 * mss);
            tcb.cwnd = tcb.ssthresh + DUP_ACK_THRESHOLD * mss;
            tcb.in_fast_recovery = true;
            FAST_RETRANSMITS.fetch_add(1, Ordering::Relaxed);
            return true;
        }
        if tcb.in_fast_recovery {
            // Each further dup ACK during recovery means another segment left the network — inflate `cwnd`
            // by one MSS so new data can flow out to keep the pipe full.
            tcb.cwnd = tcb.cwnd.saturating_add(mss);
        }
        return false;
    }

    // A new ACK that advances the window (acknowledges new data).
    if acked != 0 && acked <= outstanding {
        if tcb.in_fast_recovery {
            // Recovery is over: deflate `cwnd` back to `ssthresh` (RFC 5681) and leave fast recovery.
            tcb.cwnd = tcb.ssthresh;
            tcb.in_fast_recovery = false;
        } else {
            // Stage 22d: an ACK confirming new data without loss means the network is coping — grow `cwnd`.
            grow_cwnd(tcb, acked);
        }
        tcb.dup_acks = 0;
        tcb.snd_una = seg.ack;
        let una = tcb.snd_una;
        // Stage 23a: sample the RTT from the most recently sent segment this ACK now acknowledges that was
        // never retransmitted (Karn's algorithm — a retransmitted segment's ACK is ambiguous), and fold it
        // into the RFC 6298 estimator, which recomputes `rto`. Done before the retain below removes them.
        if let Some(sent) = tcb
            .retransmit
            .iter()
            .filter(|u| u.tries == 0 && seq_leq(u.end_seq, una))
            .map(|u| u.sent_at)
            .max()
        {
            update_rtt(tcb, now_ticks().saturating_sub(sent) as u32);
        }
        // Stage 21e: drop every queued segment the peer has now cumulatively acknowledged, so the
        // retransmission timer stops resending it.
        tcb.retransmit.retain(|u| !seq_leq(u.end_seq, una));
    }
    false
}

/// Stage 22d: grow the congestion window after an ACK confirmed `acked` new bytes, per RFC 5681. Two modes,
/// split at [`Tcb::ssthresh`]:
///
/// - **Slow start** (`cwnd < ssthresh`): add one MSS per ACK (byte-counted: `min(acked, MSS)`). Since a
///   full window of data yields `cwnd / MSS` ACKs in a round trip, `cwnd` **doubles every RTT** —
///   exponential growth, to find the network's capacity quickly from a cold start.
/// - **Congestion avoidance** (`cwnd >= ssthresh`): add `MSS * MSS / cwnd` per ACK, which sums to about one
///   MSS per RTT — **linear growth**, easing up gently once near the estimated limit. (Unreachable until a
///   loss lowers `ssthresh` below `cwnd`, which arrives in Stage 22d-2.)
fn grow_cwnd(tcb: &mut Tcb, acked: u32) {
    let mss = MSS as u32;
    if tcb.cwnd < tcb.ssthresh {
        // Slow start, byte-counted (RFC 3465 "ABC", limit L = 2*MSS). Growing by the bytes acknowledged
        // rather than per-ACK keeps the ramp the same when Stage 23b delayed ACK halves the ACK count —
        // one ACK for two segments still opens cwnd by two MSS. Capped at 2*MSS to avoid bursts.
        tcb.cwnd = tcb.cwnd.saturating_add(acked.min(2 * mss));
    } else {
        tcb.cwnd = tcb.cwnd.saturating_add((mss * mss / tcb.cwnd).max(1));
    }
}

/// Stage 22d-2: the congestion response to a **retransmission timeout** (RFC 5681 §3.1). An RTO means a
/// segment was dropped outright — the strongest evidence of congestion — so the sender retreats hard: lower
/// `ssthresh` to half the data that was in flight (never below two segments, RFC 5681 eq. 4), then collapse
/// `cwnd` all the way back to one MSS, re-entering slow start. `cwnd` then ramps up exponentially again
/// until it reaches the lowered `ssthresh`, where [`grow_cwnd`] switches to the gentler congestion
/// avoidance — so a lossy path converges on the capacity it can actually sustain instead of hammering it.
/// This is the "multiplicative decrease" half of TCP's AIMD (additive-increase / multiplicative-decrease).
fn on_rto(tcb: &mut Tcb) {
    let mss = MSS as u32;
    let flight = tcb.snd_nxt.wrapping_sub(tcb.snd_una);
    tcb.ssthresh = core::cmp::max(flight / 2, 2 * mss);
    tcb.cwnd = mss;
}

/// Stage 23a: one RFC 6298 RTT-estimator step, kept **pure** so it is unit-testable without a connection.
/// `srtt` is the smoothed RTT scaled by 8 and `rttvar` the RTT variation scaled by 4 (the standard integer
/// representation, gains alpha = 1/8 and beta = 1/4); `valid` says whether they hold a prior sample; `m` is
/// the new RTT measurement in ticks. Returns the updated `(srtt, rttvar, rto)`, `rto` in ticks clamped to
/// `[RTO_MIN_TICKS, RTO_MAX_TICKS]`.
///
/// - First sample: `SRTT = m`, `RTTVAR = m / 2` (so `srtt = m*8`, `rttvar = m*2`).
/// - Later samples: `RTTVAR += (|m - SRTT| - RTTVAR) / 4`, then `SRTT += (m - SRTT) / 8` — in the scaled
///   integers, `rttvar += |err| - rttvar/4` and `srtt += err` where `err = m - SRTT`.
/// - `RTO = SRTT + max(G, 4*RTTVAR)`, with clock granularity `G` one tick; here `4*RTTVAR == rttvar`.
fn rtt_step(srtt: u32, rttvar: u32, valid: bool, m: u32) -> (u32, u32, u32) {
    let (srtt, rttvar) = if !valid {
        (m << 3, m << 1)
    } else {
        let err = m as i64 - (srtt >> 3) as i64; // m - SRTT (may be negative)
        let srtt = (srtt as i64 + err).max(0) as u32;
        let rttvar = (rttvar as i64 + (err.abs() - (rttvar >> 2) as i64)).max(0) as u32;
        (srtt, rttvar)
    };
    // RTO = SRTT + max(G, 4*RTTVAR). srtt>>3 is SRTT; rttvar already equals 4*RTTVAR in the scaled form.
    let rto = ((srtt >> 3) + core::cmp::max(1, rttvar)).clamp(RTO_MIN_TICKS, RTO_MAX_TICKS);
    (srtt, rttvar, rto)
}

/// Stage 23a: fold a new RTT measurement (in ticks) into the connection's estimator and update its `rto`.
fn update_rtt(tcb: &mut Tcb, measured: u32) {
    let (srtt, rttvar, rto) = rtt_step(tcb.srtt, tcb.rttvar, tcb.rtt_valid, measured);
    tcb.srtt = srtt;
    tcb.rttvar = rttvar;
    tcb.rto = rto;
    tcb.rtt_valid = true;
}

/// Stage 23a: a deterministic unit self-test of the RFC 6298 estimator ([`rtt_step`]), checkable without a
/// live connection (loopback RTTs are below our tick granularity, so they only exercise the floor). Feeds
/// known samples and asserts the known-answer outputs and the clamping. Returns whether all held.
pub fn rtt_estimator_selftest() -> bool {
    // First sample R = 40 ticks: SRTT = 40 (srtt = 320), RTTVAR = 20 (rttvar = 80), RTO = 40 + 4*20 = 120.
    let (srtt, rttvar, rto) = rtt_step(0, 0, false, 40);
    let first_ok = srtt == 320 && rttvar == 80 && rto == 120;
    // A second identical sample shrinks the variation, so RTO falls back toward the RTT (120 -> 100).
    let (_, _, rto2) = rtt_step(srtt, rttvar, true, 40);
    let converge_ok = rto2 == 100 && rto2 < rto;
    // Clamping: a zero RTT floors RTO at the minimum; a huge one caps it at the maximum.
    let (_, _, rto_lo) = rtt_step(0, 0, false, 0);
    let (_, _, rto_hi) = rtt_step(0, 0, false, 100_000);
    let clamp_ok = rto_lo == RTO_MIN_TICKS && rto_hi == RTO_MAX_TICKS;
    first_ok && converge_ok && clamp_ok
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
    // Stage 22d-3: a third duplicate ACK fires a fast retransmit — resend the missing segment (the head of
    // the retransmit queue) immediately instead of waiting for the RTO. A dup ACK carries no data or FIN, so
    // there is nothing else to process for this segment; return the resent segment as the response.
    if process_ack(tcb, seg) {
        let rto = tcb.rto as u64;
        if let Some(u) = tcb.retransmit.first_mut() {
            u.deadline = now_ticks() + rto; // push the RTO out so on_tick does not also resend it
            return Some(u.segment.clone());
        }
    }

    // Accept stream data with **reassembly** (Stage 22a): in-order bytes extend the stream and let any
    // buffered out-of-order bytes be spliced in behind them, while a segment ahead of `rcv_nxt` (a gap
    // precedes it) is *held* in the reassembly queue instead of dropped. Either way the ACK below carries
    // the resulting `rcv_nxt` — unchanged for an out-of-order segment, i.e. a duplicate ACK that prompts
    // the peer to retransmit the missing segment (the classic fast-retransmit trigger).
    let disp = if seg.payload.is_empty() {
        None
    } else {
        Some(accept_segment_data(tcb, seg.seq, seg.payload))
    };

    // A FIN occupies the sequence number just past the segment's data; honor it only in order. It moves
    // us to CLOSE_WAIT — the peer will send no more data, though our side may still send until the
    // application closes too (`close`, which then sends our own FIN).
    let fin = seg.flags & FIN != 0 && seg.seq.wrapping_add(seg.payload.len() as u32) == tcb.rcv_nxt;
    if fin {
        tcb.rcv_nxt = tcb.rcv_nxt.wrapping_add(1); // the FIN consumes one sequence number
        tcb.state = State::CloseWait;
    }

    // Stage 23b: decide the ACK's *timing* (delayed ACK, RFC 1122). A FIN, or a segment that must be
    // acknowledged at once (out-of-order — the dup ACK is the fast-retransmit trigger — or gap-filling / old
    // / window-refused), draws an immediate ACK. Plain in-order data may be delayed: acknowledge every
    // second such segment, otherwise arm the delayed-ACK timer and stay silent (`flush_delayed_acks`,
    // serviced once per poll, sends it before the sender's RTO). A pure ACK of our own data needs no reply.
    if fin || matches!(disp, Some(Accept::AckNow)) {
        clear_delayed_ack(tcb);
        Some(build_ack(tcb, local_ip, remote_ip))
    } else if matches!(disp, Some(Accept::InOrderDelayable)) {
        tcb.unacked_segs += 1;
        if tcb.unacked_segs >= 2 {
            clear_delayed_ack(tcb);
            Some(build_ack(tcb, local_ip, remote_ip))
        } else {
            if tcb.delayed_ack_deadline == 0 {
                tcb.delayed_ack_deadline = now_ticks() + DELAYED_ACK_TICKS;
            }
            None
        }
    } else {
        None
    }
}

/// Stage 23b: clear a connection's pending delayed-ACK state — called whenever an ACK is actually sent (an
/// ACK carries the cumulative `rcv_nxt`, so it covers every in-order segment received so far).
fn clear_delayed_ack(tcb: &mut Tcb) {
    tcb.unacked_segs = 0;
    tcb.delayed_ack_deadline = 0;
}

/// Stage 22a: accept one data segment's payload into the receive stream, handling **reordering**. The
/// segment covers the sequence range `[seq, seq + len)`; three cases relative to `rcv_nxt` (call it R):
///
/// - **Entirely old** (`end <= R`): a duplicate of bytes already accepted — store nothing (the caller
///   still re-ACKs, so the peer stops resending it).
/// - **In order, possibly with an old prefix** (`seq <= R < end`): append the new tail `[R, end)`, advance
///   `rcv_nxt`, then splice in any buffered out-of-order segment that is now contiguous ([`drain_ooo`]).
/// - **Ahead of R** (`R < seq`): a gap precedes it — buffer it for later reassembly ([`buffer_ooo`])
///   rather than dropping it, so the peer need not retransmit it once the gap fills.
/// Stage 23b: what accepting a data segment did, which decides its ACK *timing* (delayed ACK, RFC 1122).
enum Accept {
    /// Plain in-order data — its ACK may be delayed (bundled with the next segment or a short timer).
    InOrderDelayable,
    /// Must be acknowledged immediately: an out-of-order segment (an immediate duplicate ACK is the
    /// fast-retransmit trigger), a segment that filled a reassembly gap (RFC 5681 SHOULD ack at once), an
    /// old duplicate, or one refused for a closed window.
    AckNow,
}

fn accept_segment_data(tcb: &mut Tcb, seq: u32, payload: &[u8]) -> Accept {
    let end = seq.wrapping_add(payload.len() as u32);

    if seq_leq(end, tcb.rcv_nxt) {
        return Accept::AckNow; // entirely old — re-ACK immediately so the peer stops resending
    }
    if seq_leq(seq, tcb.rcv_nxt) {
        // In order (dropping any prefix we already hold): the new bytes are the part beyond `rcv_nxt`.
        let skip = tcb.rcv_nxt.wrapping_sub(seq) as usize;
        let new = &payload[skip..];
        // Stage 22b: **flow control** — accept the segment only if it fits the free receive window. If it
        // does not, drop it (leaving `rcv_nxt` where it is): the ACK then advertises the smaller window, and
        // the peer waits / retransmits until the application reads and reopens it. This keeps `rx` bounded by
        // `RCV_WINDOW_MAX`, so the window we advertise is honest.
        let free = RCV_WINDOW_MAX.saturating_sub(tcb.rx.len());
        if new.len() > free {
            return Accept::AckNow; // window-refused — ACK now to advertise the closed window
        }
        tcb.rx.extend_from_slice(new);
        tcb.rcv_nxt = end;
        // Stage 23b: plain in-order data may have its ACK delayed; but if this segment just filled a
        // reassembly gap (spliced buffered out-of-order data back into the stream), acknowledge at once.
        if drain_ooo(tcb) {
            Accept::AckNow
        } else {
            Accept::InOrderDelayable
        }
    } else {
        // Ahead of `rcv_nxt`: a gap precedes this segment. Hold it in the reassembly queue and dup-ACK now.
        buffer_ooo(tcb, seq, payload);
        Accept::AckNow
    }
}

/// Stage 22a: hold an out-of-order segment in the reassembly queue — unless it is already fully covered by
/// a queued segment (a retransmit of one we hold) or the queue is full (then drop it; the peer will
/// resend). A genuinely new buffered segment is counted in [`OOO_BUFFERED`].
fn buffer_ooo(tcb: &mut Tcb, seq: u32, payload: &[u8]) {
    let end = seq.wrapping_add(payload.len() as u32);
    let covered = tcb.ooo.iter().any(|o| {
        let o_end = o.seq.wrapping_add(o.data.len() as u32);
        seq_leq(o.seq, seq) && seq_leq(end, o_end)
    });
    if covered || tcb.ooo.len() >= MAX_OOO_SEGMENTS {
        return;
    }
    tcb.ooo.push(OutOfOrder { seq, data: payload.to_vec() });
    OOO_BUFFERED.fetch_add(1, Ordering::Relaxed);
}

/// Stage 22a: after `rcv_nxt` advances, splice in every buffered out-of-order segment now contiguous with
/// it (and discard any that has become entirely old), repeating until the queue no longer touches
/// `rcv_nxt`. Overlaps are handled by appending only each segment's bytes *beyond* the current `rcv_nxt`.
/// Returns whether it spliced any buffered data back into the stream (Stage 23b uses this to acknowledge a
/// gap-filling segment immediately).
fn drain_ooo(tcb: &mut Tcb) -> bool {
    let mut spliced = false;
    loop {
        let r = tcb.rcv_nxt;
        // A queued segment that reaches `rcv_nxt` (`seq <= R < end`) fills the gap (or part of it).
        if let Some(i) = tcb.ooo.iter().position(|o| {
            let end = o.seq.wrapping_add(o.data.len() as u32);
            seq_leq(o.seq, r) && seq_lt(r, end)
        }) {
            let o = tcb.ooo.swap_remove(i);
            let skip = r.wrapping_sub(o.seq) as usize;
            tcb.rx.extend_from_slice(&o.data[skip..]);
            tcb.rcv_nxt = o.seq.wrapping_add(o.data.len() as u32);
            spliced = true;
            continue; // the advance may make a further queued segment contiguous
        }
        // Nothing contiguous: prune any segment now entirely behind `rcv_nxt`, then stop.
        let before = tcb.ooo.len();
        tcb.ooo.retain(|o| {
            let end = o.seq.wrapping_add(o.data.len() as u32);
            !seq_leq(end, r)
        });
        if tcb.ooo.len() == before {
            break;
        }
    }
    spliced
}

/// Stage 22c: queue application data to send on the ESTABLISHED connection `(local_port -> remote_port)`.
/// The bytes go into the connection's **send buffer**; [`flush`] then transmits as many as the peer's
/// advertised window admits. Returns `false` if there is no such established connection. Splitting "queue"
/// from "transmit" is what lets the sender obey flow control — the buffer holds bytes the window has no
/// room for yet, and they leave later as ACKs open the window.
pub fn queue_send(local_port: u16, remote_port: u16, data: &[u8]) -> bool {
    let mut table = CONNECTIONS.lock();
    match table.iter_mut().find(|c| {
        c.state == State::Established && c.local_port == local_port && c.remote_port == remote_port
    }) {
        Some(tcb) => {
            tcb.snd_buf.extend_from_slice(data);
            true
        }
        None => false,
    }
}

/// Stage 23c: set (or clear) `TCP_NODELAY` on the connection `(local_port, remote_port)` — `true` disables
/// Nagle's algorithm, so every write is sent at once instead of small ones being coalesced. Returns `false`
/// if there is no such connection. Latency-sensitive traffic disables Nagle; so do the self-tests that need
/// to control exactly when each (small) segment goes on the wire.
pub fn set_nodelay(local_port: u16, remote_port: u16, enabled: bool) -> bool {
    let mut table = CONNECTIONS.lock();
    match table.iter_mut().find(|c| c.local_port == local_port && c.remote_port == remote_port) {
        Some(tcb) => {
            tcb.nodelay = enabled;
            true
        }
        None => false,
    }
}

/// Stage 22c: build and record one data segment carrying the next `n` bytes of the send buffer, and return
/// it for transmission. Drains `n` bytes from `snd_buf`, stamps `seq = snd_nxt` (advancing it), sets
/// `PSH | ACK`, and queues the segment on the retransmit list so a lost data segment is recovered (Stage
/// 21e). `n` must be `> 0` and `<= snd_buf.len()`.
fn emit_segment(tcb: &mut Tcb, local_ip: [u8; 4], n: usize) -> Vec<u8> {
    let chunk: Vec<u8> = tcb.snd_buf.drain(..n).collect();
    let win = recv_window(tcb);
    let seg = build(
        local_ip,
        tcb.remote_ip,
        tcb.local_port,
        tcb.remote_port,
        tcb.snd_nxt,
        tcb.rcv_nxt,
        PSH | ACK,
        win,
        &chunk,
    );
    tcb.snd_nxt = tcb.snd_nxt.wrapping_add(n as u32);
    tcb.retransmit.push(Unacked {
        end_seq: tcb.snd_nxt,
        deadline: now_ticks() + tcb.rto as u64,
        tries: 0,
        sent_at: now_ticks(),
        segment: seg.clone(),
    });
    DATA_SEGMENTS_SENT.fetch_add(1, Ordering::Relaxed);
    seg
}

/// Stage 22c: the sender's half of the **sliding window** — for every established connection, transmit as
/// much queued data as the peer's advertised window ([`Tcb::snd_wnd`]) allows, in [`MSS`]-sized segments,
/// leaving the rest in the send buffer for a later call. Returns the segments to transmit (with each
/// peer's IP), built under the lock and sent by the caller after it is released. Call once per [`super::poll`]
/// (so a window that reopens is promptly used) and right after [`queue_send`].
///
/// **Zero-window probe.** If the peer advertised a window of zero and we have nothing in flight to prompt a
/// fresh ACK, we would otherwise stall forever (the peer, in this minimal stack, sends no window update
/// when its buffer drains). So we send a **one-byte probe** past `snd_una`: the peer drops it and re-ACKs
/// (still zero) until its window reopens, when it accepts the byte and its ACK carries the new window. The
/// probe rides the ordinary retransmit queue, so the Stage 21e timer resends it — serving as our persist
/// timer, no separate clock needed.
pub fn flush(local_ip: [u8; 4]) -> Vec<(Vec<u8>, [u8; 4])> {
    let mut out = Vec::new();
    let mut table = CONNECTIONS.lock();
    for tcb in table.iter_mut() {
        if tcb.state != State::Established {
            continue;
        }
        while !tcb.snd_buf.is_empty() {
            let inflight = tcb.snd_nxt.wrapping_sub(tcb.snd_una);
            // Stage 22d: obey *both* limits at once — the peer's advertised window (flow control: don't
            // overrun the receiver) and the congestion window (congestion control: don't overrun the
            // network). The effective send window is the smaller of the two.
            let window = core::cmp::min(tcb.snd_wnd, tcb.cwnd);
            let usable = window.saturating_sub(inflight);
            if usable == 0 {
                // Window-blocked. On a true zero window with nothing in flight, send a one-byte probe to
                // keep the connection moving; otherwise wait for the in-flight data's ACKs to open it.
                if tcb.snd_wnd == 0 && inflight == 0 {
                    let seg = emit_segment(tcb, local_ip, 1);
                    out.push((seg, tcb.remote_ip));
                }
                break;
            }
            let available = core::cmp::min(usable as usize, tcb.snd_buf.len());
            // Stage 23c: Nagle's algorithm — while earlier data is still unacknowledged, only send a
            // *full-sized* segment; a partial one is held (left in `snd_buf`) until an MSS accumulates or the
            // outstanding data is acknowledged. This coalesces a burst of small writes into fewer packets. It
            // does not apply when nothing is outstanding (send the small write immediately) or when the
            // application set `TCP_NODELAY`.
            if available < MSS && inflight != 0 && !tcb.nodelay {
                break;
            }
            let n = available.min(MSS);
            let seg = emit_segment(tcb, local_ip, n);
            out.push((seg, tcb.remote_ip));
        }
    }
    out
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
        recv_window(tcb),
        &[],
    );
    tcb.snd_nxt = tcb.snd_nxt.wrapping_add(1); // the FIN consumes one sequence number
    // Stage 21e: queue the FIN for retransmission too — a lost FIN would otherwise stall teardown.
    tcb.retransmit.push(Unacked {
        end_seq: tcb.snd_nxt,
        deadline: now_ticks() + tcb.rto as u64,
        tries: 0,
        sent_at: now_ticks(),
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

/// Stage 22b: the application **consuming** received data — drain up to `max` bytes from the front of the
/// connection's receive buffer and return them. This is what reopens the flow-control window: the buffer
/// shrinks, so the next segment we send advertises a larger [`recv_window`]. `None` if no such connection.
pub fn read(local_port: u16, remote_port: u16, max: usize) -> Option<Vec<u8>> {
    let mut table = CONNECTIONS.lock();
    let tcb = table
        .iter_mut()
        .find(|c| c.local_port == local_port && c.remote_port == remote_port)?;
    let n = max.min(tcb.rx.len());
    Some(tcb.rx.drain(..n).collect())
}

/// Stage 22b: the receive window this connection currently advertises — the free space in its receive
/// buffer (`RCV_WINDOW_MAX` minus the unread bytes). Zero means the buffer is full ("stop sending"). Used
/// by the flow-control self-test to observe the window shrink to zero and reopen after a [`read`]. `None`
/// if no such connection.
pub fn receive_window(local_port: u16, remote_port: u16) -> Option<u16> {
    CONNECTIONS
        .lock()
        .iter()
        .find(|c| c.local_port == local_port && c.remote_port == remote_port)
        .map(recv_window)
}

/// Stage 21c: whether every byte the application has handed us on `(local_port, remote_port)` has been
/// sent *and* acknowledged — nothing outstanding (`snd_una == snd_nxt`) and nothing still queued to send
/// (`snd_buf` empty, Stage 22c). `None` if no such connection.
pub fn all_data_acked(local_port: u16, remote_port: u16) -> Option<bool> {
    CONNECTIONS
        .lock()
        .iter()
        .find(|c| c.local_port == local_port && c.remote_port == remote_port)
        .map(|c| c.snd_una == c.snd_nxt && c.snd_buf.is_empty())
}

/// Stage 22c: bytes currently **in flight** on `(local_port, remote_port)` — sent but not yet acknowledged
/// (`snd_nxt - snd_una`). Never exceeds the peer's advertised window; the flow-control self-test checks it.
pub fn bytes_in_flight(local_port: u16, remote_port: u16) -> Option<u32> {
    CONNECTIONS
        .lock()
        .iter()
        .find(|c| c.local_port == local_port && c.remote_port == remote_port)
        .map(|c| c.snd_nxt.wrapping_sub(c.snd_una))
}

/// Stage 22c: bytes still queued in the **send buffer** of `(local_port, remote_port)` — handed to us by
/// the application but not yet transmitted because the window had no room. `None` if no such connection.
pub fn send_buffered(local_port: u16, remote_port: u16) -> Option<usize> {
    CONNECTIONS
        .lock()
        .iter()
        .find(|c| c.local_port == local_port && c.remote_port == remote_port)
        .map(|c| c.snd_buf.len())
}

/// Stage 22d: the current **congestion window** of `(local_port, remote_port)`, in bytes — the sender's
/// estimate of how much data the network can absorb, which (together with the peer's advertised window)
/// bounds how much it puts in flight. Grows via slow start / congestion avoidance as ACKs arrive. `None` if
/// no such connection. The congestion-control self-test watches this climb from [`INIT_CWND`].
pub fn congestion_window(local_port: u16, remote_port: u16) -> Option<u32> {
    CONNECTIONS
        .lock()
        .iter()
        .find(|c| c.local_port == local_port && c.remote_port == remote_port)
        .map(|c| c.cwnd)
}

/// Stage 22d-2: the current **slow-start threshold** of `(local_port, remote_port)`, in bytes — the `cwnd`
/// boundary between slow start (below) and congestion avoidance (at/above). Starts arbitrarily high
/// ([`INIT_SSTHRESH`]) and is lowered to about half the flight on a retransmission timeout ([`on_rto`]), so
/// the backoff self-test watches it fall from near-infinity to a small value. `None` if no such connection.
pub fn slow_start_threshold(local_port: u16, remote_port: u16) -> Option<u32> {
    CONNECTIONS
        .lock()
        .iter()
        .find(|c| c.local_port == local_port && c.remote_port == remote_port)
        .map(|c| c.ssthresh)
}

/// Stage 23a: the connection's current **retransmission timeout**, in ticks — the RFC 6298 estimate
/// (`SRTT + 4*RTTVAR`, clamped), or the initial default before any RTT has been measured. `None` if no such
/// connection. The RTT self-test reads it to confirm a live transfer produced a sane RTO.
pub fn current_rto(local_port: u16, remote_port: u16) -> Option<u32> {
    CONNECTIONS
        .lock()
        .iter()
        .find(|c| c.local_port == local_port && c.remote_port == remote_port)
        .map(|c| c.rto)
}

/// Stage 23a: whether the connection has folded at least one RTT measurement into its estimator (so `rto`
/// is measured, not the initial default). `None` if no such connection.
pub fn rtt_sampled(local_port: u16, remote_port: u16) -> Option<bool> {
    CONNECTIONS
        .lock()
        .iter()
        .find(|c| c.local_port == local_port && c.remote_port == remote_port)
        .map(|c| c.rtt_valid)
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
        let rto = tcb.rto as u64; // read before borrowing tcb.retransmit below (Stage 23a estimated RTO)
        let mut timed_out = false;
        if let Some(u) = tcb.retransmit.first_mut() {
            if now >= u.deadline {
                if u.tries >= MAX_RETRIES {
                    // Too many attempts: give up and abort the connection (a real stack sends RST).
                    tcb.state = State::Closed;
                    tcb.retransmit.clear();
                    continue;
                }
                u.tries += 1;
                // Exponential backoff (Karn), capped: each successive resend waits twice as long, based on
                // the connection's current estimated RTO rather than a fixed constant.
                u.deadline = now + (rto << core::cmp::min(u.tries, 6));
                resends.push((u.segment.clone(), tcb.remote_ip));
                timed_out = true;
            }
        }
        // Stage 22d-2: a retransmission timeout is the strongest congestion signal — a segment was lost
        // outright — so back the sender off (multiplicative decrease + re-enter slow start). Done after the
        // borrow of `tcb.retransmit` above is released, since it mutates other `tcb` fields (cwnd/ssthresh).
        if timed_out {
            on_rto(tcb);
        }
    }
    resends
}

/// Stage 23b: send any **delayed ACK** whose deadline has elapsed — call once per [`super::poll`] (like the
/// retransmit timer). Scans connections and, for each with an overdue pending delayed ACK, emits a bare ACK
/// carrying the current `rcv_nxt` and clears the pending state; returns the ACK segments (with each peer IP)
/// for the caller to transmit. Kept **separate from [`on_tick`]** so these ACKs are transmitted through the
/// ordinary path and are *not* counted as retransmissions.
pub fn flush_delayed_acks(local_ip: [u8; 4]) -> Vec<(Vec<u8>, [u8; 4])> {
    let now = now_ticks();
    let mut acks = Vec::new();
    let mut table = CONNECTIONS.lock();
    for tcb in table.iter_mut() {
        if tcb.delayed_ack_deadline != 0 && now >= tcb.delayed_ack_deadline {
            clear_delayed_ack(tcb);
            acks.push((build_ack(tcb, local_ip, tcb.remote_ip), tcb.remote_ip));
        }
    }
    acks
}

/// Drop all connections (and listeners). Used to isolate the loopback self-test/tests from each other.
pub fn reset_connections() {
    CONNECTIONS.lock().clear();
}
