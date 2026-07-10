//! DHCP — the Dynamic Host Configuration Protocol (Stage 20a), just enough to *lease* our IPv4
//! address instead of hardcoding it. Like DNS, DHCP is an application protocol carried inside UDP
//! datagrams — but where DNS runs *after* we have an address, DHCP is how we get one in the first
//! place, so it must work before the stack has any identity beyond its MAC.
//!
//! ## The chicken-and-egg problem
//!
//! To send an IP packet you normally need a source address — but the whole point of DHCP is that we do
//! not have one yet. So a DHCP client sends from `0.0.0.0` to the broadcast address `255.255.255.255`
//! (UDP port 68 -> 67), and — because the reply likewise cannot be unicast to an address we do not yet
//! own — it sets the **broadcast flag** so the server *broadcasts* the reply back. Everyone on the link
//! sees it; we recognize ours by the transaction id we stamped.
//!
//! ## DORA: the four-step exchange
//!
//! ```text
//!   client                          server
//!     |------ DISCOVER (broadcast) --->|   "any server out there? I need an address"
//!     |<----- OFFER    (broadcast) ----|   "you can have 10.0.2.15; I am 10.0.2.2"
//!     |------ REQUEST  (broadcast) --->|   "I formally request 10.0.2.15 from server 10.0.2.2"
//!     |<----- ACK      (broadcast) ----|   "confirmed; lease N seconds, mask/router/DNS are ..."
//! ```
//!
//! REQUEST is broadcast (not unicast to the offering server) on purpose: any *other* server that also
//! made an offer sees that we declined it and can reclaim the address it had reserved.
//!
//! ## The packet: BOOTP + options
//!
//! DHCP reuses the older BOOTP layout — a fixed 236-byte header, a 4-byte **magic cookie**
//! (`0x63825363`) that marks the start of DHCP data, then a variable list of **options**:
//!
//! ```text
//!   0        1        2        3
//!   +--------+--------+--------+--------+
//!   |   op   | htype  |  hlen  |  hops  |  0..4    op: 1 = request (client), 2 = reply (server)
//!   +--------+--------+--------+--------+
//!   |          transaction id (xid)     |  4..8
//!   +-----------------+-----------------+
//!   |      secs       |      flags      |  8..12   flags bit 15 = broadcast
//!   +-----------------+-----------------+
//!   |            ciaddr (client)        |  12..16  0 until we own an address
//!   |            yiaddr ("your")        |  16..20  the address the server assigns us
//!   |            siaddr (server)        |  20..24
//!   |            giaddr (relay)         |  24..28
//!   |            chaddr (16 bytes)      |  28..44  our MAC + padding
//!   |            sname  (64 bytes)      |  44..108
//!   |            file   (128 bytes)     |  108..236
//!   +-----------------------------------+
//!   |         magic cookie (4)          |  236..240  0x63 0x82 0x53 0x63
//!   +-----------------------------------+
//!   |   options: code, len, value ...   |  240..     TLV, terminated by option 255 (END)
//!   +-----------------------------------+
//! ```
//!
//! The real information — the message type, the lease time, the subnet mask, the router, the DNS
//! server — all lives in the **options** as `code, length, value` triples (a couple have no length:
//! PAD = 0 and END = 255). Everything multi-byte is big-endian, as always on the wire.

#![allow(dead_code)] // the client that drives this (`net::dhcp_configure`) is wired up in Stage 20b

use alloc::vec::Vec;

use super::ether::MacAddr;

/// The UDP port a DHCP *server* listens on.
pub const SERVER_PORT: u16 = 67;
/// The UDP port a DHCP *client* listens on (where offers/acks come back to).
pub const CLIENT_PORT: u16 = 68;

/// BOOTP op code for a message from client to server (all our outgoing messages).
const BOOTREQUEST: u8 = 1;
/// BOOTP op code for a message from server to client (every reply we parse).
const BOOTREPLY: u8 = 2;
/// Hardware type 1 = 10 Mb Ethernet (the `htype` field); `hlen` is then the 6-byte MAC length.
const HTYPE_ETHERNET: u8 = 1;
const HLEN_ETHERNET: u8 = 6;

/// `flags` bit 15: ask the server to *broadcast* its reply, since we have no address to unicast to yet.
const FLAG_BROADCAST: u16 = 0x8000;

/// The four magic-cookie bytes that separate the BOOTP header from the DHCP options (RFC 2131).
const MAGIC_COOKIE: [u8; 4] = [0x63, 0x82, 0x53, 0x63];

/// Length of the fixed BOOTP header, up to (not including) the magic cookie.
const FIXED_LEN: usize = 236;

// DHCP message types (the value of option 53).
pub const DISCOVER: u8 = 1;
pub const OFFER: u8 = 2;
pub const REQUEST: u8 = 3;
pub const ACK: u8 = 5;
pub const NAK: u8 = 6;

// DHCP option codes we build or parse.
const OPT_PAD: u8 = 0;
const OPT_SUBNET_MASK: u8 = 1;
const OPT_ROUTER: u8 = 3;
const OPT_DNS: u8 = 6;
const OPT_REQUESTED_IP: u8 = 50;
const OPT_LEASE_TIME: u8 = 51;
const OPT_MSG_TYPE: u8 = 53;
const OPT_SERVER_ID: u8 = 54;
const OPT_PARAM_REQUEST: u8 = 55;
const OPT_END: u8 = 255;

/// The configuration a server hands us in an OFFER or ACK — the useful contents of a parsed reply.
/// `yiaddr` is the offered/assigned address; the rest come from options and may be absent.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Reply {
    /// The DHCP message type (option 53): [`OFFER`], [`ACK`], [`NAK`], ...
    pub msg_type: u8,
    /// The address the server is giving us (the BOOTP `yiaddr` field).
    pub your_ip: [u8; 4],
    /// Which server sent this (option 54) — we echo it in our REQUEST so the others stand down.
    pub server_id: [u8; 4],
    /// The subnet mask (option 1), if present.
    pub subnet_mask: Option<[u8; 4]>,
    /// The default gateway / router (option 3, first address), if present.
    pub router: Option<[u8; 4]>,
    /// The DNS server (option 6, first address), if present.
    pub dns: Option<[u8; 4]>,
    /// The lease duration in seconds (option 51), if present.
    pub lease_secs: Option<u32>,
}

/// Build the fixed BOOTP header for an outgoing client message of `msg_type`, up to and including the
/// magic cookie and option 53 (message type). Callers append their own options and the END marker.
/// `ciaddr` is our current address (zero until we own one).
fn build_base(msg_type: u8, xid: u32, mac: MacAddr, ciaddr: [u8; 4]) -> Vec<u8> {
    let mut m = Vec::with_capacity(300);
    m.push(BOOTREQUEST); // op
    m.push(HTYPE_ETHERNET); // htype
    m.push(HLEN_ETHERNET); // hlen
    m.push(0); // hops
    m.extend_from_slice(&xid.to_be_bytes()); // transaction id
    m.extend_from_slice(&0u16.to_be_bytes()); // secs
    m.extend_from_slice(&FLAG_BROADCAST.to_be_bytes()); // flags: broadcast the reply back to us
    m.extend_from_slice(&ciaddr); // ciaddr
    m.extend_from_slice(&[0; 4]); // yiaddr (server fills this in)
    m.extend_from_slice(&[0; 4]); // siaddr
    m.extend_from_slice(&[0; 4]); // giaddr
    m.extend_from_slice(&mac); // chaddr: our 6-byte MAC ...
    m.extend_from_slice(&[0; 10]); //          ... padded to 16 bytes
    m.extend_from_slice(&[0u8; 64]); // sname (unused)
    m.extend_from_slice(&[0u8; 128]); // file (unused)
    m.extend_from_slice(&MAGIC_COOKIE);
    // Option 53: DHCP message type. Every DHCP message carries this first.
    m.extend_from_slice(&[OPT_MSG_TYPE, 1, msg_type]);
    m
}

/// The Parameter Request List (option 55): the config items we ask the server to include in its reply
/// — subnet mask, router, and DNS server. A server may send more than we ask for, or omit some.
fn push_param_request(m: &mut Vec<u8>) {
    m.extend_from_slice(&[OPT_PARAM_REQUEST, 3, OPT_SUBNET_MASK, OPT_ROUTER, OPT_DNS]);
}

/// Build a DHCPDISCOVER — the broadcast that starts the exchange ("any server out there?"). We have no
/// address yet, so `ciaddr` is zero. Returns the raw DHCP message (the UDP payload).
pub fn build_discover(xid: u32, mac: MacAddr) -> Vec<u8> {
    let mut m = build_base(DISCOVER, xid, mac, [0; 4]);
    push_param_request(&mut m);
    m.push(OPT_END);
    m
}

/// Build a DHCPREQUEST — formally requesting the `requested_ip` a server offered, naming that server in
/// option 54 (server identifier) so any *other* server that offered can reclaim its reservation. Still
/// broadcast with `ciaddr` zero (we do not own the address until the ACK arrives).
pub fn build_request(xid: u32, mac: MacAddr, requested_ip: [u8; 4], server_id: [u8; 4]) -> Vec<u8> {
    let mut m = build_base(REQUEST, xid, mac, [0; 4]);
    // Option 50: the address we are requesting (from the offer's yiaddr).
    m.extend_from_slice(&[OPT_REQUESTED_IP, 4]);
    m.extend_from_slice(&requested_ip);
    // Option 54: the server we accepted the offer from.
    m.extend_from_slice(&[OPT_SERVER_ID, 4]);
    m.extend_from_slice(&server_id);
    push_param_request(&mut m);
    m.push(OPT_END);
    m
}

/// Copy the first four bytes of `val` into a fresh `[u8; 4]`, or `None` if `val` is too short. Used for
/// address-valued options (subnet mask, router, DNS) — the router/DNS options may list several
/// addresses, and we keep only the first.
fn first_ipv4(val: &[u8]) -> Option<[u8; 4]> {
    if val.len() < 4 {
        return None;
    }
    Some([val[0], val[1], val[2], val[3]])
}

/// Parse a DHCP reply (an OFFER or ACK), returning its [`Reply`] contents, or `None` if the message is
/// not a valid server reply to *our* exchange: too short, not a BOOTREPLY, the wrong transaction id, a
/// missing/short magic cookie, or no message-type option. The BOOTP `yiaddr` gives the address; the
/// options give the message type and the network parameters.
pub fn parse_reply(buf: &[u8], expected_xid: u32) -> Option<Reply> {
    // Need the fixed header plus the 4-byte magic cookie before any options can appear.
    if buf.len() < FIXED_LEN + 4 {
        return None;
    }
    if buf[0] != BOOTREPLY {
        return None; // not a server-to-client message
    }
    let xid = u32::from_be_bytes([buf[4], buf[5], buf[6], buf[7]]);
    if xid != expected_xid {
        return None; // a reply to some other client's exchange
    }
    if buf[FIXED_LEN..FIXED_LEN + 4] != MAGIC_COOKIE {
        return None; // BOOTP without DHCP options
    }

    let mut your_ip = [0u8; 4];
    your_ip.copy_from_slice(&buf[16..20]); // yiaddr

    let mut msg_type = 0u8;
    let mut server_id = [0u8; 4];
    let mut subnet_mask = None;
    let mut router = None;
    let mut dns = None;
    let mut lease_secs = None;

    // Walk the TLV options. PAD (0) and END (255) have no length byte; every other option is
    // code, length, then `length` value bytes.
    let mut pos = FIXED_LEN + 4;
    while pos < buf.len() {
        let code = buf[pos];
        if code == OPT_END {
            break;
        }
        if code == OPT_PAD {
            pos += 1;
            continue;
        }
        let len = *buf.get(pos + 1)? as usize;
        let val_start = pos + 2;
        let val_end = val_start.checked_add(len)?;
        if val_end > buf.len() {
            return None; // an option claims to run past the buffer
        }
        let val = &buf[val_start..val_end];
        match code {
            OPT_MSG_TYPE if len == 1 => msg_type = val[0],
            OPT_SERVER_ID if len == 4 => server_id.copy_from_slice(val),
            OPT_SUBNET_MASK => subnet_mask = first_ipv4(val),
            OPT_ROUTER => router = first_ipv4(val),
            OPT_DNS => dns = first_ipv4(val),
            OPT_LEASE_TIME if len == 4 => {
                lease_secs = Some(u32::from_be_bytes([val[0], val[1], val[2], val[3]]));
            }
            _ => {} // an option we do not care about
        }
        pos = val_end;
    }

    if msg_type == 0 {
        return None; // no option 53 — not a DHCP message
    }
    Some(Reply {
        msg_type,
        your_ip,
        server_id,
        subnet_mask,
        router,
        dns,
        lease_secs,
    })
}
