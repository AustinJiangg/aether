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

#![allow(dead_code)] // the connection state machine that drives this is wired up in Stage 21b+

use alloc::vec::Vec;

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
