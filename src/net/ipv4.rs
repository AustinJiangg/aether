//! IPv4 — the Internet Protocol, version 4 (Stage 18c).
//!
//! IP is the layer that carries a payload (here, ICMP) between two 32-bit addresses across a network.
//! We only need the simplest case: a 20-byte header (no options), never fragmented.
//!
//! ```text
//!   0               1               2               3
//!   +-------+-------+---------------+-------------------------------+
//!   |version|  IHL  |    DSCP/ECN   |          total length         |  0..4
//!   +-------+-------+---------------+-----+-------------------------+
//!   |        identification         |flags|     fragment offset     |  4..8
//!   +---------------+---------------+-----+-------------------------+
//!   |      TTL      |    protocol   |         header checksum       |  8..12
//!   +---------------+---------------+-------------------------------+
//!   |                        source address                        |  12..16
//!   +--------------------------------------------------------------+
//!   |                      destination address                     |  16..20
//!   +--------------------------------------------------------------+
//! ```
//!
//! Everything multi-byte is big-endian. The **header checksum** is the "Internet checksum" (RFC 1071):
//! the one's-complement sum of the header's 16-bit words. It is computed with the checksum field set
//! to zero, and verifying a received header (checksum field included) yields zero. The same routine
//! covers the ICMP message, so [`checksum`] lives here and `icmp` reuses it.

use alloc::vec::Vec;

/// IP protocol number for ICMP.
pub const PROTO_ICMP: u8 = 1;

/// The fixed IPv4 header length we emit (no options): 5 * 4 = 20 bytes.
pub const HEADER_LEN: usize = 20;

/// The Internet checksum (RFC 1071): the one's-complement of the one's-complement sum of `data`'s
/// 16-bit big-endian words (a trailing odd byte is padded with zero). Computing it over a header
/// whose checksum field is zero yields the value to store; computing it over a header that already
/// contains its checksum yields zero, which is how a receiver validates one.
pub fn checksum(data: &[u8]) -> u16 {
    let mut sum: u32 = 0;
    let mut i = 0;
    while i + 1 < data.len() {
        sum += u16::from_be_bytes([data[i], data[i + 1]]) as u32;
        i += 2;
    }
    if i < data.len() {
        // Odd trailing byte: it is the high byte of a final word padded with a zero low byte.
        sum += (data[i] as u32) << 8;
    }
    // Fold the carries out of the high 16 bits until none remain.
    while (sum >> 16) != 0 {
        sum = (sum & 0xFFFF) + (sum >> 16);
    }
    !(sum as u16)
}

/// A parsed, borrowed IPv4 packet. `payload` is bounded by the header's total-length field, so any
/// Ethernet padding past the real packet is excluded.
pub struct Ipv4Packet<'a> {
    pub protocol: u8,
    pub src: [u8; 4],
    pub dst: [u8; 4],
    pub payload: &'a [u8],
}

impl<'a> Ipv4Packet<'a> {
    /// Parse an IPv4 packet, rejecting anything that is not version 4 or is too short for its own
    /// header. The checksum is not re-verified here (peers on the emulated link produce valid ones).
    pub fn parse(buf: &'a [u8]) -> Option<Ipv4Packet<'a>> {
        if buf.len() < HEADER_LEN {
            return None;
        }
        let version = buf[0] >> 4;
        let ihl = (buf[0] & 0x0F) as usize;
        if version != 4 || ihl < 5 {
            return None;
        }
        let header_len = ihl * 4;
        if buf.len() < header_len {
            return None;
        }
        let total_len = u16::from_be_bytes([buf[2], buf[3]]) as usize;
        let protocol = buf[9];
        let mut src = [0u8; 4];
        let mut dst = [0u8; 4];
        src.copy_from_slice(&buf[12..16]);
        dst.copy_from_slice(&buf[16..20]);
        // Clamp the payload to [header_len, total_len], but never past the buffer we actually have.
        let end = total_len.min(buf.len()).max(header_len);
        Some(Ipv4Packet {
            protocol,
            src,
            dst,
            payload: &buf[header_len..end],
        })
    }
}

/// Build an IPv4 packet: a 20-byte header (with a correct checksum) followed by `payload`. TTL 64,
/// Don't-Fragment set, no fragmentation.
pub fn build(src: [u8; 4], dst: [u8; 4], protocol: u8, payload: &[u8]) -> Vec<u8> {
    let total_len = HEADER_LEN + payload.len();
    let mut hdr = [0u8; HEADER_LEN];
    hdr[0] = 0x45; // version 4, IHL 5 (20-byte header)
    hdr[1] = 0; // DSCP / ECN
    hdr[2..4].copy_from_slice(&(total_len as u16).to_be_bytes());
    hdr[4..6].copy_from_slice(&0u16.to_be_bytes()); // identification
    hdr[6..8].copy_from_slice(&0x4000u16.to_be_bytes()); // flags: Don't Fragment; offset 0
    hdr[8] = 64; // TTL
    hdr[9] = protocol;
    // hdr[10..12] = checksum, left zero for the computation below
    hdr[12..16].copy_from_slice(&src);
    hdr[16..20].copy_from_slice(&dst);
    let ck = checksum(&hdr);
    hdr[10..12].copy_from_slice(&ck.to_be_bytes());

    let mut pkt = Vec::with_capacity(total_len);
    pkt.extend_from_slice(&hdr);
    pkt.extend_from_slice(payload);
    pkt
}
