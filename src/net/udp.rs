//! UDP — the User Datagram Protocol (Stage 19a), the stack's first *transport* layer.
//!
//! IPv4 carries a payload between two machines; UDP is the simplest thing that rides on top and lets
//! *applications* talk. It adds just two ideas over the ICMP we already speak:
//!
//! - **Ports.** A machine has one IP but many simultaneous conversations (DNS, DHCP, a game, ...).
//!   The 16-bit source/destination *ports* pick *which conversation* on each end — IP routes to the
//!   host, the port routes to the socket. That is all the multiplexing UDP does: it is connectionless
//!   and unreliable (no handshake, no acknowledgements, no ordering — "send it and forget it").
//!
//! - **A pseudo-header checksum.** ICMP's checksum covered only the ICMP message. UDP's checksum
//!   *also* covers a 12-byte **pseudo-header** built from the IP layer's fields — source IP, dest IP,
//!   the protocol number (17), and the UDP length. This is a deliberate break in the layering: UDP
//!   reaches *down* into IP so a receiver can confirm the datagram really was addressed to it (and to
//!   this protocol) and not misdelivered. The pseudo-header is only a scratch input to the checksum;
//!   it is never actually transmitted.
//!
//! The 8-byte header:
//!
//! ```text
//!   0               1               2               3
//!   +-------------------------------+-------------------------------+
//!   |          source port          |       destination port        |  0..4
//!   +-------------------------------+-------------------------------+
//!   |            length             |            checksum            |  4..8
//!   +-------------------------------+-------------------------------+
//!   |                            data ...                           |  8..
//!   +--------------------------------------------------------------+
//! ```
//!
//! `length` counts the header + data (so the minimum is 8). Everything multi-byte is big-endian.
#![allow(dead_code)] // the send/receive paths that use this are wired into `net` in Stage 19a-2

use alloc::vec::Vec;

use super::ipv4;

/// IP protocol number for UDP (what an IPv4 header's `protocol` field holds for a UDP payload).
pub const PROTO_UDP: u8 = 17;

/// The fixed UDP header length: source port + destination port + length + checksum.
pub const HEADER_LEN: usize = 8;

/// A parsed, borrowed UDP datagram. `payload` borrows the caller's buffer, bounded by the header's
/// `length` field (so any IP/Ethernet padding past the real datagram is excluded).
pub struct Datagram<'a> {
    pub src_port: u16,
    pub dst_port: u16,
    pub payload: &'a [u8],
}

impl<'a> Datagram<'a> {
    /// Parse a UDP datagram (needs at least the 8-byte header). The checksum is not re-verified here;
    /// like the IPv4/ICMP layers, we trust peers on the emulated link to produce valid ones.
    pub fn parse(buf: &'a [u8]) -> Option<Datagram<'a>> {
        if buf.len() < HEADER_LEN {
            return None;
        }
        let src_port = u16::from_be_bytes([buf[0], buf[1]]);
        let dst_port = u16::from_be_bytes([buf[2], buf[3]]);
        let length = u16::from_be_bytes([buf[4], buf[5]]) as usize;
        // `length` covers the header + data; clamp the payload to it, but never past the bytes we
        // actually have (a lying length must not let us read out of bounds).
        let end = length.min(buf.len()).max(HEADER_LEN);
        Some(Datagram {
            src_port,
            dst_port,
            payload: &buf[HEADER_LEN..end],
        })
    }
}

/// Build a UDP datagram: the 8-byte header (with a correct checksum) followed by `payload`. The
/// source/destination IPs are not stored in the datagram — they are needed only for the checksum's
/// pseudo-header (see [`checksum`]), the layering shortcut UDP makes.
pub fn build(
    src_ip: [u8; 4],
    dst_ip: [u8; 4],
    src_port: u16,
    dst_port: u16,
    payload: &[u8],
) -> Vec<u8> {
    let length = HEADER_LEN + payload.len();
    let mut dg = Vec::with_capacity(length);
    dg.extend_from_slice(&src_port.to_be_bytes());
    dg.extend_from_slice(&dst_port.to_be_bytes());
    dg.extend_from_slice(&(length as u16).to_be_bytes());
    dg.extend_from_slice(&[0, 0]); // checksum placeholder, zero for the computation below
    dg.extend_from_slice(payload);

    let ck = checksum(src_ip, dst_ip, &dg);
    // RFC 768: a computed checksum of zero is transmitted as 0xFFFF, because a zero *field* is the
    // special value meaning "no checksum computed". (One's complement makes 0x0000 and 0xFFFF both
    // represent zero, so this never changes the arithmetic for a receiver.)
    let ck = if ck == 0 { 0xFFFF } else { ck };
    dg[6..8].copy_from_slice(&ck.to_be_bytes());
    dg
}

/// The UDP checksum: the Internet checksum (RFC 1071, shared with [`ipv4::checksum`]) computed over a
/// 12-byte **pseudo-header** followed by the whole UDP datagram. The pseudo-header is:
///
/// ```text
///   +--------------------------------------------------------------+
///   |                        source address                        |  4 bytes
///   +--------------------------------------------------------------+
///   |                      destination address                     |  4 bytes
///   +---------------+---------------+-------------------------------+
///   |     zero      |   protocol    |          UDP length           |  1 + 1 + 2 bytes
///   +---------------+---------------+-------------------------------+
/// ```
///
/// It is assembled into a scratch buffer only for the sum; it is never put on the wire. `datagram`
/// must already carry its checksum field (zero when building, the received value when verifying — in
/// which case a valid datagram sums to zero).
pub fn checksum(src_ip: [u8; 4], dst_ip: [u8; 4], datagram: &[u8]) -> u16 {
    let mut buf = Vec::with_capacity(12 + datagram.len());
    buf.extend_from_slice(&src_ip);
    buf.extend_from_slice(&dst_ip);
    buf.push(0); // reserved zero byte
    buf.push(PROTO_UDP);
    buf.extend_from_slice(&(datagram.len() as u16).to_be_bytes()); // UDP length (header + data)
    buf.extend_from_slice(datagram);
    ipv4::checksum(&buf)
}
