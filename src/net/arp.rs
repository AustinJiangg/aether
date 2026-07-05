//! ARP — the Address Resolution Protocol (Stage 18b).
//!
//! IP addresses are 32-bit, but Ethernet delivers frames to 48-bit MAC addresses. ARP is the lookup
//! that bridges the two: to send an IP packet to a neighbour, a host first needs that neighbour's
//! MAC. It broadcasts a request — "who has IP X? tell me" — and the owner of X answers (unicast)
//! with its MAC. We remember the answer in a small **ARP cache** so we ask only once.
//!
//! An ARP packet for IPv4-over-Ethernet is a fixed 28 bytes:
//!
//! ```text
//!   0     2     4   5   6     8        14      18       24      28
//!   +-----+-----+---+---+-----+--------+-------+--------+-------+
//!   |htype|ptype|hln|pln| oper|  sha   |  spa  |  tha   |  tpa  |
//!   +-----+-----+---+---+-----+--------+-------+--------+-------+
//! ```
//!
//! - `htype`/`ptype` = hardware/protocol type (Ethernet = 1, IPv4 = 0x0800);
//! - `hln`/`pln` = their address lengths (6 and 4);
//! - `oper` = 1 request, 2 reply;
//! - `sha`/`spa` = sender's MAC / IP; `tha`/`tpa` = target's MAC / IP.
//!
//! All multi-byte fields are big-endian (network byte order).

use alloc::collections::BTreeMap;
use alloc::vec::Vec;

use spin::Mutex;

use super::ether::MacAddr;

/// Hardware type: Ethernet.
const HTYPE_ETHERNET: u16 = 1;
/// Protocol type: IPv4 (the same value as the IPv4 EtherType).
const PTYPE_IPV4: u16 = 0x0800;
/// Hardware address length: a MAC is 6 bytes.
const HLEN_ETHERNET: u8 = 6;
/// Protocol address length: an IPv4 address is 4 bytes.
const PLEN_IPV4: u8 = 4;

/// ARP operation: request ("who has `tpa`?").
pub const OPER_REQUEST: u16 = 1;
/// ARP operation: reply ("`tpa` is at `sha`").
pub const OPER_REPLY: u16 = 2;

/// The fixed ARP packet length for IPv4-over-Ethernet.
pub const PACKET_LEN: usize = 28;

/// A parsed ARP packet (IPv4 over Ethernet).
pub struct ArpPacket {
    /// Operation: [`OPER_REQUEST`] or [`OPER_REPLY`].
    pub oper: u16,
    /// Sender hardware address (MAC).
    pub sha: MacAddr,
    /// Sender protocol address (IPv4).
    pub spa: [u8; 4],
    /// Target hardware address (MAC); zero in a request. Parsed for completeness; the reply path
    /// keys off `sha`/`spa`, so nothing reads it yet.
    #[allow(dead_code)]
    pub tha: MacAddr,
    /// Target protocol address (IPv4).
    pub tpa: [u8; 4],
}

impl ArpPacket {
    /// Parse an ARP packet, rejecting anything shorter than 28 bytes or not IPv4-over-Ethernet.
    pub fn parse(buf: &[u8]) -> Option<ArpPacket> {
        if buf.len() < PACKET_LEN {
            return None;
        }
        let htype = u16::from_be_bytes([buf[0], buf[1]]);
        let ptype = u16::from_be_bytes([buf[2], buf[3]]);
        let hlen = buf[4];
        let plen = buf[5];
        if htype != HTYPE_ETHERNET
            || ptype != PTYPE_IPV4
            || hlen != HLEN_ETHERNET
            || plen != PLEN_IPV4
        {
            return None;
        }
        let oper = u16::from_be_bytes([buf[6], buf[7]]);
        let mut sha = [0u8; 6];
        let mut spa = [0u8; 4];
        let mut tha = [0u8; 6];
        let mut tpa = [0u8; 4];
        sha.copy_from_slice(&buf[8..14]);
        spa.copy_from_slice(&buf[14..18]);
        tha.copy_from_slice(&buf[18..24]);
        tpa.copy_from_slice(&buf[24..28]);
        Some(ArpPacket { oper, sha, spa, tha, tpa })
    }
}

/// Build an ARP packet payload (the 28 bytes that follow the Ethernet header).
fn build(oper: u16, sha: MacAddr, spa: [u8; 4], tha: MacAddr, tpa: [u8; 4]) -> Vec<u8> {
    let mut p = Vec::with_capacity(PACKET_LEN);
    p.extend_from_slice(&HTYPE_ETHERNET.to_be_bytes());
    p.extend_from_slice(&PTYPE_IPV4.to_be_bytes());
    p.push(HLEN_ETHERNET);
    p.push(PLEN_IPV4);
    p.extend_from_slice(&oper.to_be_bytes());
    p.extend_from_slice(&sha);
    p.extend_from_slice(&spa);
    p.extend_from_slice(&tha);
    p.extend_from_slice(&tpa);
    p
}

/// Build an ARP request: "who has `target_ip`? tell `our_ip` / `our_mac`." The target hardware
/// address is unknown (zeros); the caller sends it to the Ethernet broadcast address.
pub fn build_request(our_mac: MacAddr, our_ip: [u8; 4], target_ip: [u8; 4]) -> Vec<u8> {
    build(OPER_REQUEST, our_mac, our_ip, [0; 6], target_ip)
}

/// Build an ARP reply to `target_mac` / `target_ip`: "`our_ip` is at `our_mac`."
pub fn build_reply(
    our_mac: MacAddr,
    our_ip: [u8; 4],
    target_mac: MacAddr,
    target_ip: [u8; 4],
) -> Vec<u8> {
    build(OPER_REPLY, our_mac, our_ip, target_mac, target_ip)
}

/// The ARP cache: learned IPv4 -> MAC mappings. `BTreeMap::new()` is `const`, so this needs no lazy
/// initialization.
static CACHE: Mutex<BTreeMap<[u8; 4], MacAddr>> = Mutex::new(BTreeMap::new());

/// Record an IPv4 -> MAC mapping learned from an ARP packet.
pub fn cache_insert(ip: [u8; 4], mac: MacAddr) {
    CACHE.lock().insert(ip, mac);
}

/// Look up a cached MAC for `ip`.
pub fn cache_lookup(ip: [u8; 4]) -> Option<MacAddr> {
    CACHE.lock().get(&ip).copied()
}

/// Number of entries in the ARP cache (for the boot log / tests).
pub fn cache_len() -> usize {
    CACHE.lock().len()
}

/// Snapshot the ARP cache as a list of (IP, MAC) pairs — for the shell's `arp` command (Stage 18d).
pub fn cache_entries() -> alloc::vec::Vec<([u8; 4], MacAddr)> {
    CACHE.lock().iter().map(|(&ip, &mac)| (ip, mac)).collect()
}

/// Process an incoming ARP packet. It always learns the sender's IP -> MAC mapping (every ARP packet
/// carries one), and if the packet is a *request for `our_ip`* it returns the ARP **reply payload**
/// to send back to the requester; otherwise `None`. Pure apart from the cache update — no I/O — so it
/// is unit-testable, and the caller (`net::receive`) does the actual transmit.
pub fn process(our_mac: MacAddr, our_ip: [u8; 4], pkt: &ArpPacket) -> Option<Vec<u8>> {
    cache_insert(pkt.spa, pkt.sha);
    if pkt.oper == OPER_REQUEST && pkt.tpa == our_ip {
        Some(build_reply(our_mac, our_ip, pkt.sha, pkt.spa))
    } else {
        None
    }
}
