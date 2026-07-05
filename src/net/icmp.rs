//! ICMP — the Internet Control Message Protocol (Stage 18c), just the echo request/reply used by
//! `ping`.
//!
//! An ICMP echo message rides inside an IPv4 packet:
//!
//! ```text
//!   0               1               2               3
//!   +---------------+---------------+-------------------------------+
//!   |     type      |     code      |            checksum           |  0..4
//!   +---------------+---------------+-------------------------------+
//!   |          identifier           |        sequence number        |  4..8
//!   +-------------------------------+-------------------------------+
//!   |                            data ...                           |  8..
//!   +--------------------------------------------------------------+
//! ```
//!
//! - `type` = 8 for an echo *request*, 0 for an echo *reply*; `code` = 0.
//! - `checksum` is the Internet checksum (see [`super::ipv4::checksum`]) over the whole ICMP message.
//! - `identifier` / `sequence` let a sender match a reply to its request; the responder echoes them
//!   and the `data` back unchanged. Pinging is exactly this: send an echo request, get the same
//!   identifier/sequence/data back in an echo reply.

use alloc::vec::Vec;

use super::ipv4;

/// ICMP type: echo request (what `ping` sends).
pub const TYPE_ECHO_REQUEST: u8 = 8;
/// ICMP type: echo reply (what a ping target sends back).
pub const TYPE_ECHO_REPLY: u8 = 0;

/// The fixed ICMP echo header length (before the data): type + code + checksum + id + seq.
pub const HEADER_LEN: usize = 8;

/// A parsed ICMP echo message. `data` borrows the caller's buffer.
pub struct Echo<'a> {
    pub typ: u8,
    pub id: u16,
    pub seq: u16,
    pub data: &'a [u8],
}

impl<'a> Echo<'a> {
    /// Parse an ICMP echo message (needs at least the 8-byte header). The checksum is not
    /// re-verified here.
    pub fn parse(buf: &'a [u8]) -> Option<Echo<'a>> {
        if buf.len() < HEADER_LEN {
            return None;
        }
        Some(Echo {
            typ: buf[0],
            id: u16::from_be_bytes([buf[4], buf[5]]),
            seq: u16::from_be_bytes([buf[6], buf[7]]),
            data: &buf[HEADER_LEN..],
        })
    }
}

/// Build an ICMP echo message of `typ` (request or reply) with the given id/seq/data and a correct
/// checksum.
fn build(typ: u8, id: u16, seq: u16, data: &[u8]) -> Vec<u8> {
    let mut m = Vec::with_capacity(HEADER_LEN + data.len());
    m.push(typ);
    m.push(0); // code
    m.extend_from_slice(&[0, 0]); // checksum placeholder
    m.extend_from_slice(&id.to_be_bytes());
    m.extend_from_slice(&seq.to_be_bytes());
    m.extend_from_slice(data);
    let ck = ipv4::checksum(&m);
    m[2..4].copy_from_slice(&ck.to_be_bytes());
    m
}

/// Build an ICMP echo *request* (`ping` out).
pub fn build_echo_request(id: u16, seq: u16, data: &[u8]) -> Vec<u8> {
    build(TYPE_ECHO_REQUEST, id, seq, data)
}

/// Build an ICMP echo *reply* (answering someone else's ping).
pub fn build_echo_reply(id: u16, seq: u16, data: &[u8]) -> Vec<u8> {
    build(TYPE_ECHO_REPLY, id, seq, data)
}
