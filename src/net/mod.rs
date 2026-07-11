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
pub mod dhcp;
pub mod dns;
pub mod ether;
pub mod icmp;
pub mod ipv4;
pub mod tcp;
pub mod udp;

use core::sync::atomic::{AtomicBool, AtomicU32, AtomicU64, Ordering};

use alloc::vec::Vec;
use spin::Mutex;

use crate::{e1000, serial_println};
use ether::MacAddr;

/// ICMP identifier we stamp on our outgoing pings (arbitrary; lets us recognize our own replies).
const PING_ID: u16 = 0xAE71;
/// The payload our pings carry (echoed back by the target unchanged).
const PING_PAYLOAD: &[u8] = b"aether ping 0123456789abcdef";

/// The address SLIRP leases us by default — used as a **static fallback** if DHCP (Stage 20b) fails,
/// and still the value the ARP/ping/DNS tests expect. Since 20b our *live* address is dynamic (leased
/// via DHCP into [`CURRENT_IP`]); read it through [`our_ip`], not this constant.
pub const OUR_IP: [u8; 4] = [10, 0, 2, 15];
/// SLIRP's virtual gateway (it answers our ARP and — in 18c — our pings).
pub const GATEWAY_IP: [u8; 4] = [10, 0, 2, 2];
/// The limited broadcast address (255.255.255.255). A DHCP client has no address of its own yet, so it
/// sends to this and — with the broadcast flag set — the server broadcasts its reply back here; [`receive`]
/// accepts IPv4 packets addressed to it (Stage 20b), the way any host accepts limited-broadcast traffic.
pub const LIMITED_BROADCAST: [u8; 4] = [255, 255, 255, 255];

/// SLIRP's virtual DNS server (Stage 19b): it answers DNS queries by forwarding them to the host's
/// resolver, so `dns_resolve` can turn a hostname into an address without us implementing a resolver.
pub const DNS_SERVER: [u8; 4] = [10, 0, 2, 3];
/// The (fixed, ephemeral) UDP source port our DNS queries go out from. A response comes back to it, so
/// `handle_udp` delivers it (it is not the echo port); `dns_resolve` matches by transaction id anyway.
const DNS_CLIENT_PORT: u16 = 50000;

/// The UDP port on which we run a tiny echo server (Stage 19a-2): a datagram that arrives here is
/// sent straight back to its sender, with the ports swapped. Port 7 is the well-known Echo Protocol
/// (RFC 862). This makes the kernel a *live* UDP service — a peer, or our own loopback self-test, can
/// bounce a datagram off it to prove the send/receive path end to end.
pub const UDP_ECHO_PORT: u16 = 7;

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
/// Counter feeding the transaction id stamped on successive DNS queries (Stage 19b).
static DNS_XID: AtomicU32 = AtomicU32::new(0);
/// Source MAC of the most recently received frame, for the loopback self-test.
static LAST_SRC: Mutex<MacAddr> = Mutex::new([0; 6]);

/// UDP datagrams addressed to us that we have parsed (Stage 19a-2), whatever their port.
static UDP_RECEIVED: AtomicU64 = AtomicU64::new(0);
/// UDP echo replies we have sent (datagrams that arrived on [`UDP_ECHO_PORT`]), Stage 19a-2.
static UDP_ECHOES_SENT: AtomicU64 = AtomicU64::new(0);
/// UDP datagrams delivered to us on a non-echo port — i.e. data/replies meant for us to keep
/// (as opposed to bounce), Stage 19a-2. `Release`/`Acquire`-ordered so a reader that sees the count
/// bump also sees the `LAST_UDP_PAYLOAD` write that preceded it.
static UDP_DELIVERED: AtomicU64 = AtomicU64::new(0);
/// The payload of the most recently *delivered* datagram (non-echo port), so a self-test — and, later,
/// a socket layer — can inspect what arrived. A `Vec` in a `Mutex` (`Vec::new` is a const fn).
static LAST_UDP_PAYLOAD: Mutex<Vec<u8>> = Mutex::new(Vec::new());

/// Our live IPv4 address (Stage 20b). Unconfigured (`0.0.0.0`) until DHCP leases one — the whole point
/// of DHCP is that this is *negotiated*, not hardcoded. [`our_ip`] reads it; [`dhcp_configure`] (or the
/// [`use_static_fallback`] path) writes it. Behind a `Mutex` so it can be a mutable `static`.
static CURRENT_IP: Mutex<[u8; 4]> = Mutex::new([0, 0, 0, 0]);
/// The gateway/router, DNS server, and subnet mask the DHCP server handed us (Stage 20b), for display
/// by `ifconfig`. The gateway/DNS happen to match the [`GATEWAY_IP`]/[`DNS_SERVER`] constants the older
/// stages still use directly; these record what the *lease* actually said.
static LEASED_GATEWAY: Mutex<[u8; 4]> = Mutex::new([0, 0, 0, 0]);
static LEASED_DNS: Mutex<[u8; 4]> = Mutex::new([0, 0, 0, 0]);
static LEASED_MASK: Mutex<[u8; 4]> = Mutex::new([0, 0, 0, 0]);
/// The DHCP server that granted our lease (option 54), and the lease duration in seconds (option 51).
static DHCP_SERVER: Mutex<[u8; 4]> = Mutex::new([0, 0, 0, 0]);
static LEASE_SECS: AtomicU32 = AtomicU32::new(0);
/// Set once DHCP has installed a lease (so `ifconfig` can say whether the address is DHCP or fallback).
static DHCP_CONFIGURED: AtomicBool = AtomicBool::new(false);
/// A DHCP reply (OFFER/ACK) delivered to us on the client port — bumped by [`handle_udp`], watched by
/// [`dhcp_configure`]. `Release`/`Acquire`-ordered so seeing the bump implies seeing the payload write.
static DHCP_DELIVERED: AtomicU64 = AtomicU64::new(0);
/// The payload (raw DHCP message) of the most recent reply delivered on the client port.
static LAST_DHCP_PAYLOAD: Mutex<Vec<u8>> = Mutex::new(Vec::new());
/// Counter feeding the transaction id stamped on a DHCP exchange (like [`DNS_XID`]).
static DHCP_XID: AtomicU32 = AtomicU32::new(0);

/// TCP segments addressed to us that we have parsed (Stage 21b).
static TCP_SEGMENTS_RECEIVED: AtomicU64 = AtomicU64::new(0);
/// The next ephemeral local port an outgoing TCP connection will use (Stage 21b). Bumped per connect,
/// wrapping within the ephemeral range so successive connections do not collide.
static TCP_EPHEMERAL: AtomicU32 = AtomicU32::new(49152);

/// Bring the stack up: record our MAC and log our identity. Call once, after the e1000 is up. Our IP
/// is still unconfigured here (`0.0.0.0`) — DHCP (Stage 20b) leases one immediately after.
pub fn init(mac: MacAddr) {
    *OUR_MAC.lock() = mac;
    serial_println!(
        "[net] stack up: MAC {:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}, IP unconfigured (pending DHCP)",
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
    );
}

/// Our MAC address (the card's), as recorded by [`init`].
pub fn our_mac() -> MacAddr {
    *OUR_MAC.lock()
}

/// Our live IPv4 address — the DHCP lease (Stage 20b), or `0.0.0.0` before one is installed. Every
/// layer reads its source/own address through this, so the moment DHCP writes [`CURRENT_IP`] the whole
/// stack (ARP replies, ping, UDP, ...) uses the leased address.
pub fn our_ip() -> [u8; 4] {
    *CURRENT_IP.lock()
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
                if let Some(reply) = arp::process(our_mac(), our_ip(), &pkt) {
                    // Unicast the reply back to the requester (Ethernet src = us).
                    let eth = ether::build(pkt.sha, our_mac(), ether::ETHERTYPE_ARP, &reply);
                    e1000::transmit(&eth);
                    ARP_REPLIES_SENT.fetch_add(1, Ordering::Relaxed);
                }
            }
        }
        ether::ETHERTYPE_IPV4 => {
            IPV4_SEEN.fetch_add(1, Ordering::Relaxed);
            // Stage 18c/19a-2: handle IPv4 packets addressed to us, dispatched by protocol. Since
            // Stage 20b we also accept limited broadcast (255.255.255.255) so a DHCP reply — sent
            // before we own an address — reaches `handle_udp` on the client port.
            if let Some(pkt) = ipv4::Ipv4Packet::parse(frame.payload) {
                if pkt.dst == our_ip() || pkt.dst == LIMITED_BROADCAST {
                    match pkt.protocol {
                        ipv4::PROTO_ICMP => handle_icmp(&pkt, frame.src),
                        udp::PROTO_UDP => handle_udp(&pkt, frame.src),
                        tcp::PROTO_TCP => handle_tcp(&pkt, frame.src),
                        _ => {} // some other IP protocol — ignored for now
                    }
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
            let ip = ipv4::build(our_ip(), pkt.src, ipv4::PROTO_ICMP, &reply);
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

/// Stage 19a-2: handle a UDP datagram addressed to us. Mirrors [`handle_icmp`]. A datagram aimed at
/// [`UDP_ECHO_PORT`] is bounced straight back to its sender (ports swapped) — our tiny echo server, the
/// UDP analog of answering a ping; anything else is *delivered* (its payload recorded) as data meant
/// for us. `src_mac` is the Ethernet source of the frame, where an echo goes (no ARP lookup needed to
/// answer whoever addressed us — exactly as ICMP replies do).
fn handle_udp(pkt: &ipv4::Ipv4Packet, src_mac: MacAddr) {
    let dg = match udp::Datagram::parse(pkt.payload) {
        Some(d) => d,
        None => return,
    };
    UDP_RECEIVED.fetch_add(1, Ordering::Relaxed);
    if dg.dst_port == dhcp::CLIENT_PORT {
        // Stage 20b: a DHCP reply (OFFER/ACK) from a server on port 67, arriving as broadcast (accepted
        // by `receive` above). Record it — `dhcp_configure` is pumping `poll` and matches it by the
        // transaction id it stamped. Keep it out of the generic delivery slot so a lease exchange and a
        // concurrent DNS/UDP conversation cannot clobber each other.
        *LAST_DHCP_PAYLOAD.lock() = dg.payload.to_vec();
        DHCP_DELIVERED.fetch_add(1, Ordering::Release);
    } else if dg.dst_port == UDP_ECHO_PORT {
        // Echo server: reply with the same payload, swapping the ports (our echo port -> their port)
        // and the addresses (us -> them). The reply is a fresh UDP-in-IPv4-in-Ethernet frame.
        let reply = udp::build(our_ip(), pkt.src, UDP_ECHO_PORT, dg.src_port, dg.payload);
        let ip = ipv4::build(our_ip(), pkt.src, udp::PROTO_UDP, &reply);
        let frame = ether::build(src_mac, our_mac(), ether::ETHERTYPE_IPV4, &ip);
        e1000::transmit(&frame);
        UDP_ECHOES_SENT.fetch_add(1, Ordering::Relaxed);
    } else {
        // Delivered to us (a reply to something we sent, or data for some port we "listen" on):
        // record its payload so a self-test — or a future socket layer — can consume it.
        *LAST_UDP_PAYLOAD.lock() = dg.payload.to_vec();
        UDP_DELIVERED.fetch_add(1, Ordering::Release);
    }
}

/// Stage 21b: handle a TCP segment addressed to us. Parse it, run it through the connection state
/// machine ([`tcp::on_segment`]), and if that produces a response (a SYN-ACK or an ACK during the
/// handshake) wrap it back through IP and Ethernet to the sender and transmit it. `src_mac` is the
/// frame's source — where the response goes, exactly as ICMP/UDP replies do (no ARP needed to answer
/// whoever addressed us).
fn handle_tcp(pkt: &ipv4::Ipv4Packet, src_mac: MacAddr) {
    let seg = match tcp::Segment::parse(pkt.payload) {
        Some(s) => s,
        None => return,
    };
    TCP_SEGMENTS_RECEIVED.fetch_add(1, Ordering::Relaxed);
    // The state machine may hand us a segment to send back (checksummed for our_ip -> pkt.src).
    if let Some(resp) = tcp::on_segment(our_ip(), pkt.src, &seg) {
        let ip = ipv4::build(our_ip(), pkt.src, tcp::PROTO_TCP, &resp);
        let frame = ether::build(src_mac, our_mac(), ether::ETHERTYPE_IPV4, &ip);
        e1000::transmit(&frame);
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
    let request = arp::build_request(our_mac(), our_ip(), ip);
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
    let ip_pkt = ipv4::build(our_ip(), ip, ipv4::PROTO_ICMP, &icmp);
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
    let ip_pkt = ipv4::build(our_ip(), our_ip(), ipv4::PROTO_ICMP, &icmp);
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

/// Stage 19a-2 self-test of the full UDP path with no external peer, mirroring [`ping_loopback_selftest`].
/// With PHY loopback on, send a datagram addressed to ourselves on [`UDP_ECHO_PORT`]. The stack
/// receives it, the echo server bounces it back (still in loopback), and — because that echo is now
/// aimed at our source port, not the echo port — it is *delivered* to us. So success exercises both
/// directions: building and parsing the datagram, the echo we generate, and receiving that echo with
/// its payload intact. Returns whether an echo was both sent and delivered back with the same bytes.
pub fn udp_echo_loopback_selftest() -> bool {
    let mac = our_mac();
    let payload = b"aether udp echo loopback selftest";
    let src_port: u16 = 40000;
    // Addressed to ourselves at every layer, on the echo port, so our generated echo returns to us.
    let dg = udp::build(our_ip(), our_ip(), src_port, UDP_ECHO_PORT, payload);
    let ip_pkt = ipv4::build(our_ip(), our_ip(), udp::PROTO_UDP, &dg);
    let frame = ether::build(mac, mac, ether::ETHERTYPE_IPV4, &ip_pkt);

    e1000::set_loopback(true);
    // Drain stale frames so the counters reflect only this exchange.
    let mut sink = [0u8; 2048];
    while e1000::poll_frame(&mut sink).is_some() {}
    let echoed_before = UDP_ECHOES_SENT.load(Ordering::Relaxed);
    let delivered_before = UDP_DELIVERED.load(Ordering::Acquire);

    let mut ok = false;
    for _ in 0..1000 {
        e1000::transmit(&frame);
        // First poll receives the echo request (and transmits our echo, which loops back); a second
        // poll receives that echo and delivers it.
        poll();
        poll();
        if UDP_ECHOES_SENT.load(Ordering::Relaxed) > echoed_before
            && UDP_DELIVERED.load(Ordering::Acquire) > delivered_before
            && LAST_UDP_PAYLOAD.lock().as_slice() == &payload[..]
        {
            ok = true;
            break;
        }
        crate::apic::pit_sleep_us(2000);
    }
    e1000::set_loopback(false);

    serial_println!(
        "[net] UDP echo loopback selftest: echoes sent {}, delivered {}, ok = {}",
        udp_echoes_sent(),
        udp_delivered(),
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

/// UDP datagrams addressed to us that we have parsed (Stage 19a-2).
pub fn udp_received() -> u64 {
    UDP_RECEIVED.load(Ordering::Relaxed)
}

/// UDP echo replies we have sent (times our echo server bounced a datagram back), Stage 19a-2.
pub fn udp_echoes_sent() -> u64 {
    UDP_ECHOES_SENT.load(Ordering::Relaxed)
}

/// UDP datagrams delivered to us on a non-echo port (data/replies we kept), Stage 19a-2.
pub fn udp_delivered() -> u64 {
    UDP_DELIVERED.load(Ordering::Acquire)
}

/// Stage 19a-2: send a UDP datagram to `dst_ip:dst_port` from `src_port`, carrying `payload`. Resolves
/// the destination MAC via ARP (so it reaches a real peer over the wire, or SLIRP's gateway/servers),
/// builds UDP-in-IPv4-in-Ethernet, and transmits. Returns `false` if the MAC could not be resolved.
/// Fire-and-forget: UDP is unreliable, so there is no delivery confirmation.
pub fn udp_send(dst_ip: [u8; 4], dst_port: u16, src_port: u16, payload: &[u8]) -> bool {
    let mac = match arp_resolve(dst_ip) {
        Some(m) => m,
        None => return false,
    };
    let dg = udp::build(our_ip(), dst_ip, src_port, dst_port, payload);
    let ip = ipv4::build(our_ip(), dst_ip, udp::PROTO_UDP, &dg);
    let frame = ether::build(mac, our_mac(), ether::ETHERTYPE_IPV4, &ip);
    e1000::transmit(&frame);
    true
}

/// Stage 19b-2: resolve `hostname` to an IPv4 address via DNS — the first thing UDP does for real.
/// Builds a DNS query stamped with a fresh transaction id, sends it to [`DNS_SERVER`] with [`udp_send`],
/// and pumps [`poll`] until the reply is *delivered* (the same UDP receive path the echo server uses:
/// the response lands in `LAST_UDP_PAYLOAD` and bumps `UDP_DELIVERED`), then parses out the address.
/// Returns `None` on a bad hostname, if the server's MAC cannot be resolved, or on timeout. Bounded and
/// re-sending periodically, since UDP may drop a datagram and there is no retransmit underneath us.
pub fn dns_resolve(hostname: &str) -> Option<[u8; 4]> {
    let id = 0xA000u16.wrapping_add(DNS_XID.fetch_add(1, Ordering::Relaxed) as u16);
    let query = dns::build_query(id, hostname)?;

    // Match a response by the *count* of delivered datagrams changing, then check its transaction id.
    let mut seen = UDP_DELIVERED.load(Ordering::Acquire);
    // Up to ~3 s total, re-sending every ~300 ms; returns the instant a matching reply is parsed.
    for i in 0..3000 {
        if i % 300 == 0 && !udp_send(DNS_SERVER, dns::DNS_PORT, DNS_CLIENT_PORT, &query) {
            return None; // could not resolve the DNS server's MAC (no route to it)
        }
        poll(); // drain and dispatch; a delivered DNS reply updates LAST_UDP_PAYLOAD
        let delivered = UDP_DELIVERED.load(Ordering::Acquire);
        if delivered != seen {
            seen = delivered;
            // Clone out from under the lock, then parse — matching on our transaction id.
            let payload = LAST_UDP_PAYLOAD.lock().clone();
            if let Some(ip) = dns::parse_response(&payload, id) {
                return Some(ip);
            }
        }
        crate::apic::pit_sleep_us(1000);
    }
    None
}

/// Stage 20b: obtain our IPv4 configuration from a DHCP server by running the four-step **DORA**
/// exchange — DISCOVER, OFFER, REQUEST, ACK — and install the lease (address, gateway, DNS, mask) so the
/// rest of the stack uses the *leased* address instead of a hardcoded one. Returns `true` if a lease was
/// installed. Bounded and re-sending, like [`arp_resolve`]/[`dns_resolve`], since UDP may drop a message.
///
/// Everything goes out as broadcast from `0.0.0.0` (we have no address yet), and the reply comes back as
/// broadcast because we set the broadcast flag; `receive` accepts it and `handle_udp` records it. We
/// match a reply to our request by the transaction id we stamp on the exchange.
pub fn dhcp_configure() -> bool {
    let mac = our_mac();
    let xid = 0xAE7C_0000u32.wrapping_add(DHCP_XID.fetch_add(1, Ordering::Relaxed));

    // DISCOVER -> OFFER: find a server and the address it will give us.
    let discover = dhcp::build_discover(xid, mac);
    let offer = match dhcp_exchange(&discover, xid, dhcp::OFFER) {
        Some(o) => o,
        None => return false, // no server answered
    };

    // REQUEST -> ACK: formally request that address from that server, and get the confirmed lease.
    let request = dhcp::build_request(xid, mac, offer.your_ip, offer.server_id);
    let ack = match dhcp_exchange(&request, xid, dhcp::ACK) {
        Some(a) => a,
        None => return false, // the server never confirmed (or sent a NAK)
    };

    install_lease(&ack);
    true
}

/// One request/response leg of the DORA exchange: broadcast `msg`, then pump [`poll`] until a DHCP reply
/// carrying our transaction id `xid` and message type `want` is delivered, or time out. Re-broadcasts
/// periodically. Returns the parsed [`dhcp::Reply`], or `None` on timeout.
fn dhcp_exchange(msg: &[u8], xid: u32, want: u8) -> Option<dhcp::Reply> {
    let mut seen = DHCP_DELIVERED.load(Ordering::Acquire);
    // Up to ~3 s, re-sending every ~300 ms; returns the instant a matching reply is parsed.
    for i in 0..3000 {
        if i % 300 == 0 {
            send_dhcp(msg);
        }
        poll(); // drain and dispatch; a delivered DHCP reply updates LAST_DHCP_PAYLOAD
        let delivered = DHCP_DELIVERED.load(Ordering::Acquire);
        if delivered != seen {
            seen = delivered;
            let payload = LAST_DHCP_PAYLOAD.lock().clone();
            if let Some(reply) = dhcp::parse_reply(&payload, xid) {
                if reply.msg_type == want {
                    return Some(reply);
                }
            }
        }
        crate::apic::pit_sleep_us(1000);
    }
    None
}

/// Broadcast a raw DHCP message: UDP `68 -> 67` from `0.0.0.0` to `255.255.255.255`, in an Ethernet
/// frame to the broadcast MAC. No ARP is possible (we have no address and the destination is broadcast),
/// so this bypasses [`udp_send`] and frames the datagram directly.
fn send_dhcp(payload: &[u8]) {
    let dg = udp::build(
        [0, 0, 0, 0],
        LIMITED_BROADCAST,
        dhcp::CLIENT_PORT,
        dhcp::SERVER_PORT,
        payload,
    );
    let ip = ipv4::build([0, 0, 0, 0], LIMITED_BROADCAST, udp::PROTO_UDP, &dg);
    let frame = ether::build(ether::BROADCAST, our_mac(), ether::ETHERTYPE_IPV4, &ip);
    e1000::transmit(&frame);
}

/// Install a lease from an ACK: set our live address and record the gateway/DNS/mask/server/lease time.
fn install_lease(ack: &dhcp::Reply) {
    *CURRENT_IP.lock() = ack.your_ip;
    *DHCP_SERVER.lock() = ack.server_id;
    if let Some(gw) = ack.router {
        *LEASED_GATEWAY.lock() = gw;
    }
    if let Some(dns) = ack.dns {
        *LEASED_DNS.lock() = dns;
    }
    if let Some(mask) = ack.subnet_mask {
        *LEASED_MASK.lock() = mask;
    }
    LEASE_SECS.store(ack.lease_secs.unwrap_or(0), Ordering::Relaxed);
    DHCP_CONFIGURED.store(true, Ordering::Release);
}

/// Fall back to the static [`OUR_IP`] if DHCP did not answer, so the rest of the stack still has a usable
/// address (the boot self-tests, the shell). A real host might instead pick a link-local address.
pub fn use_static_fallback() {
    *CURRENT_IP.lock() = OUR_IP;
}

/// Whether a DHCP lease has been installed (vs. the static fallback), for `ifconfig`.
pub fn dhcp_configured() -> bool {
    DHCP_CONFIGURED.load(Ordering::Acquire)
}

/// The gateway/router the DHCP lease named (Stage 20b), or `0.0.0.0` if not configured.
pub fn leased_gateway() -> [u8; 4] {
    *LEASED_GATEWAY.lock()
}

/// The DNS server the DHCP lease named (Stage 20b), or `0.0.0.0` if not configured.
pub fn leased_dns() -> [u8; 4] {
    *LEASED_DNS.lock()
}

/// The subnet mask the DHCP lease named (Stage 20b), or `0.0.0.0` if not configured.
pub fn leased_mask() -> [u8; 4] {
    *LEASED_MASK.lock()
}

/// The lease duration in seconds the DHCP server granted (Stage 20b), or 0 if not configured.
pub fn lease_secs() -> u32 {
    LEASE_SECS.load(Ordering::Relaxed)
}

/// TCP segments addressed to us that we have parsed (Stage 21b).
pub fn tcp_segments_received() -> u64 {
    TCP_SEGMENTS_RECEIVED.load(Ordering::Relaxed)
}

/// The next-hop MAC for reaching `dst_ip`: our own MAC when it *is* our own address (a loopback
/// connection — the frame must be addressed to our MAC to pass the receive filter), otherwise resolved
/// via ARP. SLIRP puts every guest on one `/24`, so an on-link peer's own MAC works; a real router would
/// resolve the gateway for an off-subnet destination. Returns `None` if ARP cannot resolve it.
fn tcp_next_hop(dst_ip: [u8; 4]) -> Option<MacAddr> {
    if dst_ip == our_ip() {
        return Some(our_mac());
    }
    arp_resolve(dst_ip)
}

/// Pick the next ephemeral local port for an outgoing connection (49152..=65535, wrapping).
fn next_ephemeral_port() -> u16 {
    let n = TCP_EPHEMERAL.fetch_add(1, Ordering::Relaxed);
    49152 + (n % (65536 - 49152)) as u16
}

/// Stage 21b: actively open a TCP connection to `remote_ip:remote_port` by performing the three-way
/// handshake. Returns the local port on success (the connection is then ESTABLISHED), or `None` on
/// timeout. Resolves the next-hop MAC, creates a SYN_SENT TCB and its SYN (`tcp::open_active`),
/// transmits it, and pumps [`poll`] — where the SYN-ACK is received and the state machine emits the final
/// ACK — until the connection reaches ESTABLISHED. Bounded and re-sending the SYN periodically, since a
/// segment may be lost and (this sub-step) there is no retransmission timer yet.
pub fn tcp_connect(remote_ip: [u8; 4], remote_port: u16) -> Option<u16> {
    let mac = tcp_next_hop(remote_ip)?;
    let local_port = next_ephemeral_port();
    let syn = tcp::open_active(our_ip(), remote_ip, local_port, remote_port);
    let frame = ether::build(
        mac,
        our_mac(),
        ether::ETHERTYPE_IPV4,
        &ipv4::build(our_ip(), remote_ip, tcp::PROTO_TCP, &syn),
    );
    for i in 0..2000 {
        if i % 200 == 0 {
            e1000::transmit(&frame);
        }
        poll(); // the SYN-ACK arrives here; the state machine transmits the final ACK
        if tcp::connection_state(local_port, remote_port) == Some(tcp::State::Established) {
            return Some(local_port);
        }
        crate::apic::pit_sleep_us(1000);
    }
    None
}

/// Stage 21c: send application `data` on the established TCP connection `(local_port -> remote_port)`.
/// Builds a data segment ([`tcp::send_data`], which advances the connection's send sequence), frames it
/// as TCP-in-IPv4-in-Ethernet to the peer, and transmits. Returns `false` if there is no such established
/// connection, or the peer's MAC cannot be resolved. The peer's ACK is processed by [`poll`]/[`handle_tcp`]
/// when it arrives; there is no retransmission if this segment is lost (Stage 21e adds the timer).
pub fn tcp_send(local_port: u16, remote_port: u16, data: &[u8]) -> bool {
    let (seg, remote_ip) = match tcp::send_data(our_ip(), local_port, remote_port, data) {
        Some(x) => x,
        None => return false, // no established connection with this port pair
    };
    let mac = match tcp_next_hop(remote_ip) {
        Some(m) => m,
        None => return false, // cannot resolve the peer's MAC (no route)
    };
    let frame = ether::build(
        mac,
        our_mac(),
        ether::ETHERTYPE_IPV4,
        &ipv4::build(our_ip(), remote_ip, tcp::PROTO_TCP, &seg),
    );
    e1000::transmit(&frame);
    true
}

/// Stage 21c self-test of TCP **data transfer** with no external peer, via PHY loopback — the follow-on to
/// [`tcp_handshake_loopback_selftest`]. Establish a loopback connection (listen, then connect to
/// ourselves), then send a payload from the client to the server: the data segment loops back, the server
/// accepts it in order and ACKs, and that ACK loops back so the client marks the bytes acknowledged.
/// Success proves the reliable byte stream in both directions — the send path (sequence tracking) on one
/// end, the in-order receive path plus acknowledgement on the other. Returns whether the exact bytes were
/// received in order *and* the sender saw them all acknowledged.
pub fn tcp_data_loopback_selftest() -> bool {
    tcp::reset_connections();
    let port: u16 = 7778;
    tcp::open_passive(port); // a listener to accept our own SYN

    e1000::set_loopback(true);
    // Drain stale frames so the exchange sees a clean ring.
    let mut sink = [0u8; 2048];
    while e1000::poll_frame(&mut sink).is_some() {}

    // Handshake first (as in the 21b self-test), then let the server also reach ESTABLISHED.
    let client_port = match tcp_connect(our_ip(), port) {
        Some(p) => p,
        None => {
            e1000::set_loopback(false);
            serial_println!("[net] TCP data loopback selftest: handshake failed");
            return false;
        }
    };
    for _ in 0..200 {
        if tcp::established_count() >= 2 {
            break;
        }
        poll();
        crate::apic::pit_sleep_us(500);
    }

    // Send a payload from the client and pump: the data segment loops back (server buffers it and ACKs),
    // then the ACK loops back (client marks the bytes acknowledged).
    let payload = b"hello from aether tcp 21c";
    tcp_send(client_port, port, payload);
    let mut ok = false;
    for _ in 0..400 {
        poll();
        let received = tcp::received_data(port, client_port);
        let acked = tcp::all_data_acked(client_port, port) == Some(true);
        if received.as_deref() == Some(&payload[..]) && acked {
            ok = true;
            break;
        }
        crate::apic::pit_sleep_us(500);
    }
    e1000::set_loopback(false);

    let got = tcp::received_data(port, client_port).map(|d| d.len()).unwrap_or(0);
    serial_println!(
        "[net] TCP data loopback selftest: {} byte(s) received in order, acknowledged = {}, ok = {}",
        got,
        tcp::all_data_acked(client_port, port) == Some(true),
        ok,
    );
    ok
}

/// Stage 21b self-test of the three-way handshake with no external peer, via PHY loopback. Listen on a
/// port, then actively connect to *ourselves* on it: our SYN loops back, the listener answers SYN-ACK,
/// that loops back and we answer ACK, which loops back to the listener — so both a client TCB and a
/// server TCB reach ESTABLISHED. Success exercises both halves of the handshake plus segment build/parse
/// and the checksum. Returns whether the connect succeeded and both ends established.
pub fn tcp_handshake_loopback_selftest() -> bool {
    tcp::reset_connections();
    let port: u16 = 7777;
    tcp::open_passive(port); // a listener to accept our own SYN

    e1000::set_loopback(true);
    // Drain stale frames so the handshake sees a clean ring.
    let mut sink = [0u8; 2048];
    while e1000::poll_frame(&mut sink).is_some() {}

    let connected = tcp_connect(our_ip(), port).is_some();
    // The client reaches ESTABLISHED when it sends the final ACK; pump a little more so the listener
    // receives that ACK (looped back) and reaches ESTABLISHED too.
    for _ in 0..200 {
        if tcp::established_count() >= 2 {
            break;
        }
        poll();
        crate::apic::pit_sleep_us(500);
    }
    e1000::set_loopback(false);

    let established = tcp::established_count();
    serial_println!(
        "[net] TCP handshake loopback selftest: connect = {}, established connections = {}",
        connected, established,
    );
    connected && established >= 2
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
