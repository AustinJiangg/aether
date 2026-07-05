//! Ethernet II framing (Stage 18a).
//!
//! Every frame on the wire begins with the same 14-byte Ethernet II header:
//!
//! ```text
//!   0        6        12   14                        end
//!   +--------+--------+----+------------- ... -------+
//!   | dst(6) | src(6) | et |        payload          |
//!   +--------+--------+----+------------- ... -------+
//! ```
//!
//! - `dst` / `src`: 48-bit MAC addresses (destination first).
//! - `et` (EtherType): a 16-bit tag naming what the payload is — `0x0800` = IPv4, `0x0806` = ARP.
//!   It is stored **big-endian** ("network byte order"), like every multi-byte field on the wire,
//!   so we convert with `to_be_bytes` / `from_be_bytes` (x86 is little-endian).
//!
//! The 4-byte Ethernet CRC (frame check sequence) that follows the payload is handled by the NIC —
//! the card strips it on receive (RCTL.SECRC) and appends it on transmit (TXD.IFCS) — so it never
//! appears in the buffers we parse or build here.

use alloc::vec::Vec;

/// A 48-bit Ethernet MAC address.
pub type MacAddr = [u8; 6];

/// The all-ones destination that every station accepts — used for ARP requests (Stage 18b).
#[allow(dead_code)] // sent by the ARP request path in 18b
pub const BROADCAST: MacAddr = [0xFF; 6];

/// EtherType for IPv4 payloads.
pub const ETHERTYPE_IPV4: u16 = 0x0800;
/// EtherType for ARP payloads.
pub const ETHERTYPE_ARP: u16 = 0x0806;

/// The fixed Ethernet II header length: dst(6) + src(6) + ethertype(2).
pub const HEADER_LEN: usize = 14;

/// A parsed, borrowed view over a received Ethernet II frame. Holds no copy of the payload — it
/// points into the caller's receive buffer, so parsing is allocation-free.
#[allow(dead_code)] // `dst`/`payload` are read by the ARP (18b) and IPv4 (18c) handlers
pub struct Frame<'a> {
    pub dst: MacAddr,
    pub src: MacAddr,
    pub ethertype: u16,
    pub payload: &'a [u8],
}

impl<'a> Frame<'a> {
    /// Parse an Ethernet II frame. Returns `None` if the buffer is too short to hold even the
    /// 14-byte header (a runt), in which case there is nothing to dispatch.
    pub fn parse(buf: &'a [u8]) -> Option<Frame<'a>> {
        if buf.len() < HEADER_LEN {
            return None;
        }
        let mut dst = [0u8; 6];
        let mut src = [0u8; 6];
        dst.copy_from_slice(&buf[0..6]);
        src.copy_from_slice(&buf[6..12]);
        let ethertype = u16::from_be_bytes([buf[12], buf[13]]);
        Some(Frame {
            dst,
            src,
            ethertype,
            payload: &buf[HEADER_LEN..],
        })
    }
}

/// Build an Ethernet II frame — header followed by `payload` — as a heap buffer ready to hand to
/// `e1000::transmit`. The NIC pads a sub-60-byte frame to the Ethernet minimum and appends the CRC.
pub fn build(dst: MacAddr, src: MacAddr, ethertype: u16, payload: &[u8]) -> Vec<u8> {
    let mut frame = Vec::with_capacity(HEADER_LEN + payload.len());
    frame.extend_from_slice(&dst);
    frame.extend_from_slice(&src);
    frame.extend_from_slice(&ethertype.to_be_bytes());
    frame.extend_from_slice(payload);
    frame
}
