//! DNS — the Domain Name System (Stage 19b), just enough to resolve a hostname to an IPv4 address.
//!
//! DNS is our first *application-layer* protocol: a DNS message is simply the payload of a UDP
//! datagram (to a server on port 53). It maps a human name (`example.com`) to an address
//! (`93.184.216.34`) by asking a resolver, which does the legwork and answers.
//!
//! A DNS message — query and response share the same shape — is a 12-byte header followed by a
//! variable number of *questions* and *resource records*:
//!
//! ```text
//!   0               1               2               3
//!   +-------------------------------+-------------------------------+
//!   |          transaction id       |             flags             |  0..4
//!   +-------------------------------+-------------------------------+
//!   |            QDCOUNT             |            ANCOUNT            |  4..8   (# questions / answers)
//!   +-------------------------------+-------------------------------+
//!   |            NSCOUNT            |            ARCOUNT             |  8..12
//!   +-------------------------------+-------------------------------+
//!   |  question(s): QNAME, QTYPE, QCLASS                            |  12..
//!   |  answer(s):   NAME, TYPE, CLASS, TTL, RDLENGTH, RDATA         |
//! ```
//!
//! Two ideas are new here:
//!
//! - **Name encoding.** A name is a sequence of length-prefixed *labels*, terminated by a zero byte:
//!   `example.com` becomes `\x07example\x03com\x00`. A label is at most 63 bytes (the top two bits of
//!   the length are reserved — see the next point).
//!
//! - **Compression pointers.** To save space, a name in a *response* is often replaced by a 2-byte
//!   pointer whose top two bits are `11` (`0xC0`): the low 14 bits are an *offset* back into the
//!   message where the real name lives. We never need to *follow* one — we only skip past names to
//!   reach the fields we want — but the parser must recognize a pointer (2 bytes, name ends) versus a
//!   literal label.
//!
//! The **transaction id** ties a response to its query: UDP is unreliable and unordered, so the
//! resolver stamps a 16-bit id on the query and only accepts a response echoing it.
#![allow(dead_code)] // the resolver that drives this (`net::dns_resolve`) is wired up in Stage 19b-2

use alloc::vec::Vec;

/// The well-known UDP port a DNS server listens on.
pub const DNS_PORT: u16 = 53;

/// Resource-record / question TYPE for an IPv4 host address (an "A record").
pub const TYPE_A: u16 = 1;
/// Resource-record / question CLASS for the Internet.
pub const CLASS_IN: u16 = 1;

/// The fixed DNS header length (id + flags + the four section counts).
const HEADER_LEN: usize = 12;
/// Query flags: standard query (opcode 0) with **Recursion Desired** set, so the server resolves the
/// name fully on our behalf instead of just referring us to another server.
const FLAG_RECURSION_DESIRED: u16 = 0x0100;
/// Response flag bit: **QR** (query/response) — set on a response.
const FLAG_RESPONSE: u16 = 0x8000;
/// Mask for the **RCODE** (response code) in the flags: nonzero means the server reported an error.
const RCODE_MASK: u16 = 0x000F;
/// The two-bit tag (in a name's length byte) marking a compression pointer rather than a label.
const PTR_MASK: u8 = 0xC0;

/// Encode a hostname into DNS wire format (length-prefixed labels, zero-terminated) appended to `out`.
/// Returns `false` if a label is empty or longer than 63 bytes (unrepresentable). A trailing dot is
/// tolerated (its empty final label is simply the root).
fn encode_name(name: &str, out: &mut Vec<u8>) -> bool {
    for label in name.split('.') {
        if label.is_empty() {
            continue; // skip a trailing/leading dot; the terminating zero is appended below
        }
        if label.len() > 63 {
            return false; // a label's length must fit in 6 bits (the top two are the pointer tag)
        }
        out.push(label.len() as u8);
        out.extend_from_slice(label.as_bytes());
    }
    out.push(0); // root label terminates the name
    true
}

/// Build a DNS query for the A record of `hostname`, stamped with transaction id `id`. Returns the
/// raw message bytes (the UDP payload), or `None` if `hostname` is empty or has an invalid label.
pub fn build_query(id: u16, hostname: &str) -> Option<Vec<u8>> {
    if hostname.is_empty() {
        return None;
    }
    let mut msg = Vec::with_capacity(HEADER_LEN + hostname.len() + 6);
    msg.extend_from_slice(&id.to_be_bytes());
    msg.extend_from_slice(&FLAG_RECURSION_DESIRED.to_be_bytes());
    msg.extend_from_slice(&1u16.to_be_bytes()); // QDCOUNT: one question
    msg.extend_from_slice(&0u16.to_be_bytes()); // ANCOUNT
    msg.extend_from_slice(&0u16.to_be_bytes()); // NSCOUNT
    msg.extend_from_slice(&0u16.to_be_bytes()); // ARCOUNT
    if !encode_name(hostname, &mut msg) {
        return None;
    }
    msg.extend_from_slice(&TYPE_A.to_be_bytes()); // QTYPE: A (IPv4 address)
    msg.extend_from_slice(&CLASS_IN.to_be_bytes()); // QCLASS: IN
    Some(msg)
}

/// Skip past a DNS name starting at `pos`, returning the offset of the byte *after* it. A name is a
/// run of labels ending in either a zero byte or a compression pointer (2 bytes). We only need to step
/// over names to reach the fields after them, so a pointer is treated as an end marker — we never
/// follow it. Returns `None` if the name runs off the end of the buffer or uses a reserved tag.
fn skip_name(buf: &[u8], mut pos: usize) -> Option<usize> {
    loop {
        let len = *buf.get(pos)?;
        if len == 0 {
            return Some(pos + 1); // root label: name ends
        } else if len & PTR_MASK == PTR_MASK {
            buf.get(pos + 1)?; // a pointer is two bytes; ensure the second is present
            return Some(pos + 2); // the name ends at a pointer
        } else if len & PTR_MASK == 0 {
            pos += 1 + len as usize; // a literal label: its length byte plus that many bytes
            if pos > buf.len() {
                return None;
            }
        } else {
            return None; // 0x40 / 0x80 are reserved tags
        }
    }
}

/// Parse a DNS response and return the first IPv4 address in its answer section, or `None` if the
/// message is not a valid response to our query (id mismatch, not a response, server error) or carries
/// no A record. Questions and non-A answers (e.g. a CNAME preceding the address) are skipped; answer
/// names — usually compression pointers — are stepped over via [`skip_name`].
pub fn parse_response(buf: &[u8], expected_id: u16) -> Option<[u8; 4]> {
    if buf.len() < HEADER_LEN {
        return None;
    }
    let id = u16::from_be_bytes([buf[0], buf[1]]);
    let flags = u16::from_be_bytes([buf[2], buf[3]]);
    if id != expected_id || flags & FLAG_RESPONSE == 0 || flags & RCODE_MASK != 0 {
        return None; // not our response, or the server reported an error
    }
    let qdcount = u16::from_be_bytes([buf[4], buf[5]]);
    let ancount = u16::from_be_bytes([buf[6], buf[7]]);

    let mut pos = HEADER_LEN;
    // Step over the echoed question(s): each is a name followed by QTYPE + QCLASS (4 bytes).
    for _ in 0..qdcount {
        pos = skip_name(buf, pos)?;
        pos = pos.checked_add(4)?;
        if pos > buf.len() {
            return None;
        }
    }
    // Walk the answer records, returning the first A record's address.
    for _ in 0..ancount {
        pos = skip_name(buf, pos)?;
        // Each record: TYPE(2) CLASS(2) TTL(4) RDLENGTH(2) then RDLENGTH bytes of RDATA.
        if pos + 10 > buf.len() {
            return None;
        }
        let typ = u16::from_be_bytes([buf[pos], buf[pos + 1]]);
        let rdlength = u16::from_be_bytes([buf[pos + 8], buf[pos + 9]]) as usize;
        pos += 10;
        if pos + rdlength > buf.len() {
            return None;
        }
        if typ == TYPE_A && rdlength == 4 {
            return Some([buf[pos], buf[pos + 1], buf[pos + 2], buf[pos + 3]]);
        }
        pos += rdlength; // skip a non-A record (e.g. a CNAME) and keep looking
    }
    None
}
