//! Stage 18: a minimal network stack (Ethernet, then ARP + IPv4 + ICMP).
//!
//! The e1000 driver moves raw Ethernet frames on and off the wire; this module turns those bytes
//! into a *protocol stack*. Networking is built in **layers**, each wrapping the next like nested
//! envelopes: an Ethernet frame carries an ARP or IPv4 payload; an IPv4 packet carries an ICMP (or
//! TCP/UDP) payload. Each layer parses only its own header and hands the rest up.
//!
//! ## 18a: Ethernet framing + the receive plumbing
//!
//! This sub-step wires the NIC into the stack and handles the outermost layer:
//!
//! - [`ether`] parses and builds the 14-byte Ethernet II header.
//! - The e1000 receive interrupt (Stage 17b-6) now only *flags* that frames are waiting; [`poll`]
//!   pulls them off the ring (via `e1000::poll_frame`) from ordinary context and dispatches each by
//!   its EtherType. 18a just classifies and counts (ARP vs IPv4 vs other) — the ARP and IPv4/ICMP
//!   handlers arrive in 18b and 18c.
//! - [`loopback_selftest`] proves the whole path end to end with the card's own PHY loopback (no
//!   external traffic needed): build a frame addressed to ourselves, send it, and confirm the stack
//!   receives and classifies it.
//!
//! Our network identity is **static** for now: we simply claim `10.0.2.15`, the address QEMU's SLIRP
//! user-mode network hands out by default (DHCP would negotiate it — a possible later step). The MAC
//! is whatever the card reports.
//!
//! ## 18b: ARP
//!
//! [`arp`] adds the Address Resolution Protocol — the IP-to-MAC lookup. Two directions, both live:
//! `receive` now feeds ARP frames to [`arp::process`], which learns the sender's mapping and, if the
//! packet is a request for our IP, sends a reply (so other hosts can find us); and [`arp_resolve`]
//! sends a request for a target IP and pumps [`poll`] until the reply populates the cache. This is
//! the stack's first *live* exchange: asking SLIRP's gateway (`10.0.2.2`) for its MAC and getting an
//! answer back proves send, receive, and parse all work against a real peer.
//!
//! ## 18c: IPv4 + ICMP echo (ping)
//!
//! [`ipv4`] and [`icmp`] add the next two layers and the headline capability: **ping**. An echo
//! request is ICMP-in-IPv4-in-Ethernet; the target echoes an ICMP reply. Both directions are live:
//! `receive` dispatches IPv4/ICMP frames addressed to us — answering echo *requests* (so we are
//! pingable) and recording echo *replies* — and [`ping`] sends an echo request and pumps [`poll`]
//! until the matching reply comes back. [`ping`]ing SLIRP's gateway succeeds because libslirp reflects
//! echoes aimed at `10.0.2.2` directly. The [`ipv4::checksum`] (the "Internet checksum") protects both
//! the IP header and the ICMP message.

pub mod arp;
pub mod ether;
pub mod icmp;
pub mod ipv4;

use core::sync::atomic::{AtomicU32, AtomicU64, Ordering};

use spin::Mutex;

use crate::{e1000, serial_println};
use ether::MacAddr;

/// ICMP identifier we stamp on our outgoing pings (arbitrary; lets us recognize our own replies).
const PING_ID: u16 = 0xAE71;
/// The payload our pings carry (echoed back by the target unchanged).
const PING_PAYLOAD: &[u8] = b"aether ping 0123456789abcdef";

/// Our static IPv4 address on QEMU's SLIRP network (the default guest lease).
pub const OUR_IP: [u8; 4] = [10, 0, 2, 15];
/// SLIRP's virtual gateway (it answers our ARP and — in 18c — our pings).
pub const GATEWAY_IP: [u8; 4] = [10, 0, 2, 2];

/// Our MAC address, copied from the e1000 in [`init`]. Behind a `Mutex` so it can be a non-const
/// `static`; it is written once at boot and only read afterward.
static OUR_MAC: Mutex<MacAddr> = Mutex::new([0; 6]);

/// Total Ethernet frames the stack has parsed (Stage 18a).
static FRAMES_RECEIVED: AtomicU64 = AtomicU64::new(0);
/// Frames classified as ARP (Stage 18a; ARP is actually handled in 18b).
static ARP_SEEN: AtomicU64 = AtomicU64::new(0);
/// Frames classified as IPv4 (Stage 18a; IPv4 is actually handled in 18c).
static IPV4_SEEN: AtomicU64 = AtomicU64::new(0);
/// ARP replies we have sent in response to requests for our IP (Stage 18b).
static ARP_REPLIES_SENT: AtomicU64 = AtomicU64::new(0);
/// ICMP echo *requests* we have answered — i.e. how many times we were successfully pinged (18c).
static ICMP_REQUESTS_HANDLED: AtomicU64 = AtomicU64::new(0);
/// ICMP echo *replies* we have received — i.e. how many of our pings came back (18c).
static ICMP_REPLIES_RECEIVED: AtomicU64 = AtomicU64::new(0);
/// Identifier / sequence of the most recently received echo reply, so [`ping`] can match its own.
static LAST_REPLY_ID: AtomicU32 = AtomicU32::new(0);
static LAST_REPLY_SEQ: AtomicU32 = AtomicU32::new(0);
/// Monotonic sequence number stamped on successive outgoing pings.
static PING_SEQ: AtomicU32 = AtomicU32::new(0);
/// Source MAC of the most recently received frame, for the loopback self-test.
static LAST_SRC: Mutex<MacAddr> = Mutex::new([0; 6]);

/// Bring the stack up: record our MAC and log our identity. Call once, after the e1000 is up.
pub fn init(mac: MacAddr) {
    *OUR_MAC.lock() = mac;
    serial_println!(
        "[net] stack up: IP {}.{}.{}.{}, MAC {:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}",
        OUR_IP[0], OUR_IP[1], OUR_IP[2], OUR_IP[3],
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
    );
}

/// Our MAC address (the card's), as recorded by [`init`].
pub fn our_mac() -> MacAddr {
    *OUR_MAC.lock()
}

/// Our static IPv4 address.
#[allow(dead_code)] // used by the ARP/IP layers in 18b/18c
pub fn our_ip() -> [u8; 4] {
    OUR_IP
}

/// Drain and dispatch every frame the NIC currently has waiting. Returns how many it processed.
/// Called in a bounded loop from boot (18a-c) and, later (18d), from a background task woken by the
/// receive interrupt. A single stack buffer is reused for each frame — no per-frame allocation.
pub fn poll() -> usize {
    let mut buf = [0u8; 2048];
    let mut n = 0;
    while let Some(len) = e1000::poll_frame(&mut buf) {
        receive(&buf[..len]);
        n += 1;
    }
    n
}

/// Process one received Ethernet frame: parse the header and dispatch by EtherType. In 18a this only
/// classifies and counts; 18b routes ARP to the ARP handler and 18c routes IPv4 to the IP layer.
fn receive(bytes: &[u8]) {
    let frame = match ether::Frame::parse(bytes) {
        Some(f) => f,
        None => return, // a runt too short for even an Ethernet header
    };
    FRAMES_RECEIVED.fetch_add(1, Ordering::Relaxed);
    *LAST_SRC.lock() = frame.src;
    match frame.ethertype {
        ether::ETHERTYPE_ARP => {
            ARP_SEEN.fetch_add(1, Ordering::Relaxed);
            // Stage 18b: learn the sender's IP->MAC, and reply if it is asking for us.
            if let Some(pkt) = arp::ArpPacket::parse(frame.payload) {
                if let Some(reply) = arp::process(our_mac(), OUR_IP, &pkt) {
                    // Unicast the reply back to the requester (Ethernet src = us).
                    let eth = ether::build(pkt.sha, our_mac(), ether::ETHERTYPE_ARP, &reply);
                    e1000::transmit(&eth);
                    ARP_REPLIES_SENT.fetch_add(1, Ordering::Relaxed);
                }
            }
        }
        ether::ETHERTYPE_IPV4 => {
            IPV4_SEEN.fetch_add(1, Ordering::Relaxed);
            // Stage 18c: handle IPv4 packets addressed to us. Only ICMP so far.
            if let Some(pkt) = ipv4::Ipv4Packet::parse(frame.payload) {
                if pkt.dst == OUR_IP && pkt.protocol == ipv4::PROTO_ICMP {
                    handle_icmp(&pkt, frame.src);
                }
            }
        }
        _ => {} // some other EtherType — ignored for now
    }
}

/// Stage 18c: handle an ICMP message addressed to us. An echo *request* is answered with a reply
/// (wrapped back through IP and Ethernet to the sender's MAC), so we are pingable; an echo *reply* is
/// recorded (its id/seq) so [`ping`] can confirm its own ping came back. `src_mac` is the Ethernet
/// source of the frame — where a reply goes (no ARP lookup needed to answer whoever addressed us).
fn handle_icmp(pkt: &ipv4::Ipv4Packet, src_mac: MacAddr) {
    let echo = match icmp::Echo::parse(pkt.payload) {
        Some(e) => e,
        None => return,
    };
    match echo.typ {
        icmp::TYPE_ECHO_REQUEST => {
            // Answer: swap the addresses and send an echo reply back to the requester.
            let reply = icmp::build_echo_reply(echo.id, echo.seq, echo.data);
            let ip = ipv4::build(OUR_IP, pkt.src, ipv4::PROTO_ICMP, &reply);
            let frame = ether::build(src_mac, our_mac(), ether::ETHERTYPE_IPV4, &ip);
            e1000::transmit(&frame);
            ICMP_REQUESTS_HANDLED.fetch_add(1, Ordering::Relaxed);
        }
        icmp::TYPE_ECHO_REPLY => {
            LAST_REPLY_ID.store(u32::from(echo.id), Ordering::Relaxed);
            LAST_REPLY_SEQ.store(u32::from(echo.seq), Ordering::Relaxed);
            ICMP_REPLIES_RECEIVED.fetch_add(1, Ordering::Release);
        }
        _ => {}
    }
}

/// Total frames the stack has parsed (Stage 18a).
pub fn frames_received() -> u64 {
    FRAMES_RECEIVED.load(Ordering::Relaxed)
}

/// Frames classified as ARP (Stage 18a).
pub fn arp_seen() -> u64 {
    ARP_SEEN.load(Ordering::Relaxed)
}

/// Frames classified as IPv4 (Stage 18a).
#[allow(dead_code)] // asserted by the IPv4/ICMP tests in 18c
pub fn ipv4_seen() -> u64 {
    IPV4_SEEN.load(Ordering::Relaxed)
}

/// ARP replies we have sent in response to requests for our IP (Stage 18b).
#[allow(dead_code)] // surfaced by the `arp` shell command in 18d
pub fn arp_replies_sent() -> u64 {
    ARP_REPLIES_SENT.load(Ordering::Relaxed)
}

/// Stage 18b: resolve `ip` to a MAC via ARP. Returns a cached answer immediately if we have one;
/// otherwise broadcasts an ARP request and pumps [`poll`] until the reply lands in the cache,
/// bounded so a silent network cannot hang boot (returns `None` on timeout). Re-broadcasts
/// periodically in case an early frame is lost while the link settles.
pub fn arp_resolve(ip: [u8; 4]) -> Option<MacAddr> {
    if let Some(mac) = arp::cache_lookup(ip) {
        return Some(mac);
    }
    let request = arp::build_request(our_mac(), OUR_IP, ip);
    let frame = ether::build(ether::BROADCAST, our_mac(), ether::ETHERTYPE_ARP, &request);
    // Up to ~2 s total, re-broadcasting every ~200 ms; returns the instant the reply is cached.
    for i in 0..2000 {
        if i % 200 == 0 {
            e1000::transmit(&frame);
        }
        poll(); // drain and dispatch anything the NIC has received (the reply arrives here)
        if let Some(mac) = arp::cache_lookup(ip) {
            return Some(mac);
        }
        crate::apic::pit_sleep_us(1000);
    }
    None
}

/// Stage 18c: ping `ip` — send an ICMP echo request and wait (bounded) for the matching reply.
/// Returns the round-trip sequence number on success, or `None` on timeout. Resolves the target's
/// MAC via ARP first, then builds ICMP-in-IPv4-in-Ethernet, transmits, and pumps [`poll`] until an
/// echo reply carrying our identifier and this request's sequence number comes back. Re-sends
/// periodically in case a frame is lost.
pub fn ping(ip: [u8; 4]) -> Option<u16> {
    let mac = arp_resolve(ip)?;
    let seq = (PING_SEQ.fetch_add(1, Ordering::Relaxed) + 1) as u16;
    let icmp = icmp::build_echo_request(PING_ID, seq, PING_PAYLOAD);
    let ip_pkt = ipv4::build(OUR_IP, ip, ipv4::PROTO_ICMP, &icmp);
    let frame = ether::build(mac, our_mac(), ether::ETHERTYPE_IPV4, &ip_pkt);

    let replies_before = ICMP_REPLIES_RECEIVED.load(Ordering::Acquire);
    for i in 0..2000 {
        if i % 200 == 0 {
            e1000::transmit(&frame);
        }
        poll();
        if ICMP_REPLIES_RECEIVED.load(Ordering::Acquire) > replies_before
            && LAST_REPLY_ID.load(Ordering::Relaxed) == u32::from(PING_ID)
            && LAST_REPLY_SEQ.load(Ordering::Relaxed) == u32::from(seq)
        {
            return Some(seq);
        }
        crate::apic::pit_sleep_us(1000);
    }
    None
}

/// Stage 18c self-test of the full ICMP path with no external peer: with PHY loopback on, send an
/// echo *request* addressed to ourselves. The stack receives it, answers with an echo *reply*, which
/// (still in loopback) comes back and is recorded — so success exercises both directions: building
/// and parsing the request, the reply we generate, and receiving that reply. Returns whether both a
/// request was handled and a reply received.
pub fn ping_loopback_selftest() -> bool {
    let mac = our_mac();
    let icmp = icmp::build_echo_request(PING_ID, 0xFFFF, b"aether icmp loopback selftest");
    // Addressed to ourselves at every layer, so our generated reply also returns to us.
    let ip_pkt = ipv4::build(OUR_IP, OUR_IP, ipv4::PROTO_ICMP, &icmp);
    let frame = ether::build(mac, mac, ether::ETHERTYPE_IPV4, &ip_pkt);

    e1000::set_loopback(true);
    // Drain stale frames so the counters reflect only this exchange.
    let mut sink = [0u8; 2048];
    while e1000::poll_frame(&mut sink).is_some() {}
    let handled_before = ICMP_REQUESTS_HANDLED.load(Ordering::Relaxed);
    let replies_before = ICMP_REPLIES_RECEIVED.load(Ordering::Acquire);

    let mut ok = false;
    for _ in 0..1000 {
        e1000::transmit(&frame);
        // First poll receives the request (and transmits our reply, which loops back); a second
        // poll receives that reply.
        poll();
        poll();
        if ICMP_REQUESTS_HANDLED.load(Ordering::Relaxed) > handled_before
            && ICMP_REPLIES_RECEIVED.load(Ordering::Acquire) > replies_before
        {
            ok = true;
            break;
        }
        crate::apic::pit_sleep_us(2000);
    }
    e1000::set_loopback(false);

    serial_println!(
        "[net] ICMP loopback selftest: requests answered {}, replies received {}, ok = {}",
        icmp_requests_handled(),
        icmp_replies_received(),
        ok,
    );
    ok
}

/// ICMP echo requests we have answered (times we were pinged), Stage 18c.
pub fn icmp_requests_handled() -> u64 {
    ICMP_REQUESTS_HANDLED.load(Ordering::Relaxed)
}

/// ICMP echo replies we have received (pings that came back), Stage 18c.
pub fn icmp_replies_received() -> u64 {
    ICMP_REPLIES_RECEIVED.load(Ordering::Acquire)
}

/// Stage 18d: parse a dotted-decimal IPv4 address (`"10.0.2.2"`) into four octets, or `None` if it is
/// malformed (wrong number of parts, non-numeric, or an octet out of 0..=255). Used by the shell's
/// `ping` command. `u8::from_str` (via `parse`) rejects out-of-range and non-digit octets for us.
pub fn parse_ipv4(s: &str) -> Option<[u8; 4]> {
    let mut octets = [0u8; 4];
    let mut parts = s.trim().split('.');
    for octet in octets.iter_mut() {
        *octet = parts.next()?.parse::<u8>().ok()?;
    }
    if parts.next().is_some() {
        return None; // more than four parts
    }
    Some(octets)
}

/// Stage 18a self-test: send an Ethernet frame to ourselves through the card's PHY loopback and
/// confirm the stack receives and classifies it. Returns whether a frame with our own source MAC and
/// the sent EtherType came back. Reuses the e1000 loopback (no external traffic), the same technique
/// the driver's own receive self-tests use.
pub fn loopback_selftest() -> bool {
    let mac = our_mac();
    // Address the frame to ourselves (accepted via Receive Address 0), tagged ARP so the dispatch
    // in `receive` classifies it — a real ARP packet is not needed to exercise the framing path.
    let payload = b"aether net 18a ethernet framing test";
    let frame = ether::build(mac, mac, ether::ETHERTYPE_ARP, payload);

    e1000::set_loopback(true);
    // Drain anything already in the ring so the counters reflect only our frame.
    let mut sink = [0u8; 2048];
    while e1000::poll_frame(&mut sink).is_some() {}
    let arp_before = ARP_SEEN.load(Ordering::Relaxed);

    let mut ok = false;
    for _ in 0..1000 {
        e1000::transmit(&frame);
        // Pull the looped-back frame (if any) off the ring and dispatch it through the stack.
        poll();
        if ARP_SEEN.load(Ordering::Relaxed) > arp_before && *LAST_SRC.lock() == mac {
            ok = true;
            break;
        }
        // QEMU's receiver may still be settling early in boot — wait and resend, bounded.
        crate::apic::pit_sleep_us(2000);
    }
    e1000::set_loopback(false);

    serial_println!(
        "[net] loopback framing test: received {} frame(s) total, {} classified ARP, match = {}",
        frames_received(),
        arp_seen(),
        ok,
    );
    ok
}
