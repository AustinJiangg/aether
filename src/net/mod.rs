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
/// TCP segments the retransmission timer has resent (Stage 21e).
static TCP_RETRANSMITS: AtomicU64 = AtomicU64::new(0);
/// Stage 21e fault-injection hook: when > 0, the next TCP frame handed to [`tx_tcp`] is silently dropped
/// instead of transmitted, simulating packet loss so the retransmission timer must recover it. Test-only
/// (armed by `tcp_retransmit_loopback_selftest`); zero in normal operation.
static DROP_NEXT_TCP_TX: AtomicU32 = AtomicU32::new(0);
/// Stage 23d-2b fault-injection hook: a **bitmask** dropping specific frames of an upcoming burst — bit `k`
/// (from the LSB) drops the `k`-th frame handed to [`tx_tcp`] after arming, consuming one bit per frame and
/// disarming at zero. Unlike the consecutive [`DROP_NEXT_TCP_TX`], this can lose *non-adjacent* segments, so
/// the SACK test can open two holes in one burst. Test-only (armed by `tcp_sack_recovery_loopback_selftest`).
static DROP_TCP_TX_MASK: AtomicU32 = AtomicU32::new(0);
/// Stage 22a fault-injection hook: when armed, [`tx_tcp`] holds the next TCP frame back and sends it only
/// *after* the following one, so two consecutive sends leave in reversed order — exercising the receiver's
/// out-of-order reassembly with no real network reordering. Test-only (armed by
/// `tcp_reassembly_loopback_selftest`); the held frame lives in [`HELD_TCP_TX`].
static REORDER_NEXT_TCP_TX: AtomicBool = AtomicBool::new(false);
/// The frame [`REORDER_NEXT_TCP_TX`] is holding back, released after the next `tx_tcp` (Stage 22a).
static HELD_TCP_TX: Mutex<Option<Vec<u8>>> = Mutex::new(None);

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
    // Stage 21e: first service the TCP timers — retransmit any segment whose ACK is overdue and expire
    // any elapsed TIME_WAIT. The resends are framed and transmitted here, outside tcp's connection lock.
    for (segment, remote_ip) in tcp::on_tick() {
        if let Some(mac) = tcp_resend_next_hop(remote_ip) {
            let frame = ether::build(
                mac,
                our_mac(),
                ether::ETHERTYPE_IPV4,
                &ipv4::build(our_ip(), remote_ip, tcp::PROTO_TCP, &segment),
            );
            tx_tcp(&frame);
            TCP_RETRANSMITS.fetch_add(1, Ordering::Relaxed);
        }
    }

    // Stage 22c: send any queued data the peer's window now admits — a window that reopened (via an ACK
    // processed on a previous poll, or a read that drained our own receive buffer) is used here.
    transmit_tcp_flush(tcp::flush(our_ip()));

    // Stage 23b: send any delayed ACK whose timer has elapsed. Through the ordinary path, so — unlike a
    // retransmission — it is not counted as one.
    transmit_tcp_flush(tcp::flush_delayed_acks(our_ip()));

    let mut buf = [0u8; 2048];
    let mut n = 0;
    while let Some(len) = e1000::poll_frame(&mut buf) {
        receive(&buf[..len]);
        n += 1;
    }

    // Stage 23d-2b: send any extra holes a SACK-guided fast retransmit queued while dispatching the ACKs
    // above (the head hole already went out inline as a segment's response). Through the ordinary path, so —
    // like the inline fast retransmit — these are not counted as RTO retransmissions.
    transmit_tcp_flush(tcp::take_sack_resends());
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

/// Like [`tcp_next_hop`] but **cache-only** — it never pumps [`poll`] (which would reenter it). Used on
/// the Stage 21e retransmit path *inside* `poll`, where an already-established connection's peer MAC is
/// already in the ARP cache (or is our own MAC, for a loopback connection).
fn tcp_resend_next_hop(dst_ip: [u8; 4]) -> Option<MacAddr> {
    if dst_ip == our_ip() {
        return Some(our_mac());
    }
    arp::cache_lookup(dst_ip)
}

/// Transmit a fully-framed TCP frame, honoring the Stage 21e loss-injection hook: when [`DROP_NEXT_TCP_TX`]
/// is armed the frame is silently dropped (simulating packet loss) instead of sent, so the retransmission
/// timer must recover it. The outbound data path and the resend path both go through here.
fn tx_tcp(frame: &[u8]) {
    if DROP_NEXT_TCP_TX.load(Ordering::Acquire) > 0 {
        DROP_NEXT_TCP_TX.fetch_sub(1, Ordering::AcqRel);
        return; // "lost" on the wire
    }
    // Stage 23d-2b: the bitmask drop hook — consume one bit per frame; drop this one if its bit is set.
    let mask = DROP_TCP_TX_MASK.load(Ordering::Acquire);
    if mask != 0 {
        DROP_TCP_TX_MASK.store(mask >> 1, Ordering::Release);
        if mask & 1 != 0 {
            return; // this frame of the burst is "lost"
        }
    }
    // Stage 22a: if the reorder hook is armed, hold this frame back and let the *next* one go first, then
    // release the held one behind it — so two consecutive sends arrive reversed for the reassembly test.
    if REORDER_NEXT_TCP_TX.swap(false, Ordering::AcqRel) {
        *HELD_TCP_TX.lock() = Some(frame.to_vec());
        return;
    }
    e1000::transmit(frame);
    if let Some(held) = HELD_TCP_TX.lock().take() {
        e1000::transmit(&held); // the deferred frame, now out of order (after this one)
    }
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

/// Stage 22c: transmit the segments a [`tcp::flush`] produced (queued data the window now admits, or a
/// zero-window probe), each framed as TCP-in-IPv4-in-Ethernet to its peer. Uses the cache-only next-hop
/// (an established connection's peer MAC is already resolved, or is our own for loopback) and goes through
/// [`tx_tcp`] so the loss/reorder fault hooks still apply.
fn transmit_tcp_flush(segments: Vec<(Vec<u8>, [u8; 4])>) {
    for (seg, remote_ip) in segments {
        if let Some(mac) = tcp_resend_next_hop(remote_ip) {
            let frame = ether::build(
                mac,
                our_mac(),
                ether::ETHERTYPE_IPV4,
                &ipv4::build(our_ip(), remote_ip, tcp::PROTO_TCP, &seg),
            );
            tx_tcp(&frame);
        }
    }
}

/// Stage 21c/22c: send application `data` on the established TCP connection `(local_port -> remote_port)`.
/// The bytes are **queued** ([`tcp::queue_send`]); [`tcp::flush`] then transmits as many as the peer's
/// advertised window admits, leaving any excess buffered to leave later (as ACKs open the window, driven
/// by [`poll`]). Returns `false` if there is no such established connection. A lost segment is recovered by
/// the Stage 21e retransmission timer.
pub fn tcp_send(local_port: u16, remote_port: u16, data: &[u8]) -> bool {
    if !tcp::queue_send(local_port, remote_port, data) {
        return false; // no established connection with this port pair
    }
    transmit_tcp_flush(tcp::flush(our_ip()));
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

/// Stage 21d: close our end of the TCP connection `(local_port -> remote_port)` — the application is done
/// sending. Builds the FIN segment ([`tcp::close`], which advances the send sequence and moves the state
/// machine into the FIN handshake), frames it, and transmits. Returns `false` if there is no such
/// connection in a closable state, or the peer's MAC cannot be resolved. The peer's ACK / FIN are handled
/// by the normal [`poll`]/[`handle_tcp`] path, driving the connection through teardown to CLOSED.
pub fn tcp_close(local_port: u16, remote_port: u16) -> bool {
    let (seg, remote_ip) = match tcp::close(our_ip(), local_port, remote_port) {
        Some(x) => x,
        None => return false, // no connection in a closable state with this port pair
    };
    let mac = match tcp_next_hop(remote_ip) {
        Some(m) => m,
        None => return false,
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

/// Stage 21d self-test of connection **teardown** with no external peer, via PHY loopback — the follow-on
/// to [`tcp_data_loopback_selftest`]. Establish a loopback connection, then walk both ends through the
/// four-way FIN handshake:
///
/// 1. The client actively closes (`tcp_close`): its FIN loops back, the server acknowledges it (reaching
///    CLOSE_WAIT) and the client, seeing that ACK, reaches FIN_WAIT_2.
/// 2. The server then closes its own end: its FIN loops back, the client acknowledges it (reaching
///    TIME_WAIT) and the server, seeing that final ACK, reaches CLOSED.
///
/// Success is the client in TIME_WAIT and the server in CLOSED — proving the full teardown state machine,
/// each FIN consuming a sequence number and each being acknowledged. Returns whether both ends arrived.
pub fn tcp_teardown_loopback_selftest() -> bool {
    tcp::reset_connections();
    let port: u16 = 7779;
    tcp::open_passive(port);

    e1000::set_loopback(true);
    let mut sink = [0u8; 2048];
    while e1000::poll_frame(&mut sink).is_some() {}

    let client_port = match tcp_connect(our_ip(), port) {
        Some(p) => p,
        None => {
            e1000::set_loopback(false);
            serial_println!("[net] TCP teardown loopback selftest: handshake failed");
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

    // Step 1 — the client actively closes. Pump until the server ACKs the FIN (CLOSE_WAIT) and the client
    // reaches FIN_WAIT_2. (The server must reach CLOSE_WAIT before it can passively close in step 2.)
    tcp_close(client_port, port);
    for _ in 0..400 {
        poll();
        if tcp::connection_state(port, client_port) == Some(tcp::State::CloseWait)
            && tcp::connection_state(client_port, port) == Some(tcp::State::FinWait2)
        {
            break;
        }
        crate::apic::pit_sleep_us(500);
    }

    // Step 2 — the server closes its own end. Pump until the client ACKs the server's FIN (TIME_WAIT) and
    // the server, seeing that final ACK, reaches CLOSED.
    tcp_close(port, client_port);
    for _ in 0..400 {
        poll();
        if tcp::connection_state(client_port, port) == Some(tcp::State::TimeWait)
            && tcp::connection_state(port, client_port) == Some(tcp::State::Closed)
        {
            break;
        }
        crate::apic::pit_sleep_us(500);
    }
    e1000::set_loopback(false);

    let client_state = tcp::connection_state(client_port, port);
    let server_state = tcp::connection_state(port, client_port);
    let ok = client_state == Some(tcp::State::TimeWait) && server_state == Some(tcp::State::Closed);
    serial_println!(
        "[net] TCP teardown loopback selftest: client {:?}, server {:?}, ok = {}",
        client_state, server_state, ok,
    );
    ok
}

/// Stage 21e: how many TCP segments the retransmission timer has resent (over the whole run).
pub fn tcp_retransmits() -> u64 {
    TCP_RETRANSMITS.load(Ordering::Relaxed)
}

/// Stage 22d-3: how many TCP fast retransmits have fired (a segment resent on the third duplicate ACK
/// rather than on a retransmission timeout), over the whole run. The fast-retransmit test reads it.
pub fn tcp_fast_retransmits() -> u64 {
    tcp::fast_retransmits()
}

/// Stage 22a: how many out-of-order TCP segments the receiver has buffered for reassembly (whole run).
pub fn tcp_out_of_order_buffered() -> u64 {
    tcp::out_of_order_buffered()
}

/// Stage 22a self-test of **out-of-order reassembly** with no external peer, via PHY loopback — the
/// follow-on to [`tcp_retransmit_loopback_selftest`]. Establish a loopback connection, then send a payload
/// as *two* data segments with the reorder hook armed, so the second segment reaches the receiver *before*
/// the first. The receiver must buffer the ahead-of-sequence segment, then splice it in once the earlier
/// segment fills the gap — so the stream is reassembled **in order** despite arriving reversed, and both
/// segments end up acknowledged. Returns whether the bytes arrived in order and the reorder path was hit.
pub fn tcp_reassembly_loopback_selftest() -> bool {
    tcp::reset_connections();
    let port: u16 = 7781;
    tcp::open_passive(port);

    // Start from a clean hook state, then drain any stale looped frames.
    REORDER_NEXT_TCP_TX.store(false, Ordering::Release);
    *HELD_TCP_TX.lock() = None;
    e1000::set_loopback(true);
    let mut sink = [0u8; 2048];
    while e1000::poll_frame(&mut sink).is_some() {}

    let client_port = match tcp_connect(our_ip(), port) {
        Some(p) => p,
        None => {
            e1000::set_loopback(false);
            serial_println!("[net] TCP reassembly loopback selftest: handshake failed");
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

    // Disable Nagle so the two small segments go out at once (Stage 23c would otherwise hold the second
    // sub-MSS segment while the first is unacknowledged); this test controls segment timing directly.
    tcp::set_nodelay(client_port, port, true);

    // Split a payload into two segments and send them with the reorder hook armed: segment #1 is held back
    // and segment #2 goes on the wire first, so the receiver sees them out of order.
    let payload = b"aether tcp reassembly 22a: out-of-order splice";
    let split = 20;
    let ooo_before = tcp::out_of_order_buffered();
    REORDER_NEXT_TCP_TX.store(true, Ordering::Release);
    tcp_send(client_port, port, &payload[..split]); // #1: held back by the hook
    tcp_send(client_port, port, &payload[split..]); // #2: sent first, then #1 released behind it

    // Pump until the receiver has reassembled the whole payload in order and the sender sees it acked.
    let mut reassembled = false;
    for _ in 0..2000 {
        poll();
        if tcp::received_data(port, client_port).as_deref() == Some(&payload[..])
            && tcp::all_data_acked(client_port, port) == Some(true)
        {
            reassembled = true;
            break;
        }
        crate::apic::pit_sleep_us(500);
    }
    let buffered = tcp::out_of_order_buffered() > ooo_before;

    // Leave the hook clean for anything that transmits TCP afterwards.
    REORDER_NEXT_TCP_TX.store(false, Ordering::Release);
    *HELD_TCP_TX.lock() = None;
    e1000::set_loopback(false);

    let got = tcp::received_data(port, client_port).map(|d| d.len()).unwrap_or(0);
    let ok = reassembled && buffered;
    serial_println!(
        "[net] TCP reassembly loopback selftest: {} of {} bytes in order, buffered out-of-order {}, ok = {}",
        got,
        payload.len(),
        buffered,
        ok,
    );
    ok
}

/// Stage 23d-2a self-test of the receiver's **SACK block** emission, via PHY loopback (no external peer).
/// Mirrors [`tcp_reassembly_loopback_selftest`]: send a payload as two segments with the reorder hook armed
/// so the second reaches the receiver first, leaving a gap. With SACK negotiated (every SYN now offers it),
/// the dup ACK the receiver sends for that out-of-order segment must carry a **SACK option** naming the
/// buffered range — so `sack_acks_sent` rises — and the stream still reassembles in order once the gap
/// fills. Returns whether a SACK-carrying ACK was emitted and all bytes arrived in order.
pub fn tcp_sack_blocks_loopback_selftest() -> bool {
    tcp::reset_connections();
    let port: u16 = 7783;
    tcp::open_passive(port);

    REORDER_NEXT_TCP_TX.store(false, Ordering::Release);
    *HELD_TCP_TX.lock() = None;
    e1000::set_loopback(true);
    let mut sink = [0u8; 2048];
    while e1000::poll_frame(&mut sink).is_some() {}

    let client_port = match tcp_connect(our_ip(), port) {
        Some(p) => p,
        None => {
            e1000::set_loopback(false);
            serial_println!("[net] TCP SACK blocks loopback selftest: handshake failed");
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

    // Send the two segments at once (no Nagle hold), reordered so #2 lands before #1 and is buffered.
    tcp::set_nodelay(client_port, port, true);
    let payload = b"aether tcp sack 23d-2a: selective ack blocks";
    let split = 20;
    let sack_before = tcp::sack_acks_sent();
    REORDER_NEXT_TCP_TX.store(true, Ordering::Release);
    tcp_send(client_port, port, &payload[..split]); // #1: held back by the hook
    tcp_send(client_port, port, &payload[split..]); // #2: sent first -> buffered out of order -> SACK dup-ACK

    let mut sack_seen = false;
    let mut reassembled = false;
    for _ in 0..2000 {
        poll();
        if tcp::sack_acks_sent() > sack_before {
            sack_seen = true; // the receiver reported its out-of-order range in a SACK option
        }
        if tcp::received_data(port, client_port).as_deref() == Some(&payload[..])
            && tcp::all_data_acked(client_port, port) == Some(true)
        {
            reassembled = true;
            break;
        }
        crate::apic::pit_sleep_us(500);
    }

    REORDER_NEXT_TCP_TX.store(false, Ordering::Release);
    *HELD_TCP_TX.lock() = None;
    e1000::set_loopback(false);

    let ok = sack_seen && reassembled;
    serial_println!(
        "[net] TCP SACK blocks loopback selftest: sack-acks {}, reassembled {}, ok = {}",
        tcp::sack_acks_sent() - sack_before,
        reassembled,
        ok,
    );
    ok
}

/// Stage 23d-2b self-test of the sender **consuming SACK blocks** to recover several losses in one round
/// trip, via PHY loopback. Grow `cwnd` so five MSS-sized segments fit in flight, then send five with the
/// **first and third dropped** (the bitmask hook — two non-adjacent holes). The three that arrive
/// (segments 2, 4, 5) are out of order, so the receiver dup-ACKs each *with SACK blocks* naming them; on the
/// third dup ACK the sender fast-retransmits, and — guided by SACK — resends **both** holes at once (skipping
/// the three it now knows arrived) rather than one per round trip. Returns whether the extra hole was
/// SACK-recovered (`sack_retransmits` rose), the fast path ran, the RTO timer never fired, and all bytes
/// arrived in order.
pub fn tcp_sack_recovery_loopback_selftest() -> bool {
    tcp::reset_connections();
    let port: u16 = 7784;
    tcp::open_passive(port);

    DROP_TCP_TX_MASK.store(0, Ordering::Release);
    e1000::set_loopback(true);
    let mut sink = [0u8; 2048];
    while e1000::poll_frame(&mut sink).is_some() {}

    let client_port = match tcp_connect(our_ip(), port) {
        Some(p) => p,
        None => {
            e1000::set_loopback(false);
            serial_println!("[net] TCP SACK recovery loopback selftest: handshake failed");
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

    // Phase 1: grow cwnd above five MSS so a five-segment burst goes out at once. Stream a batch and drain it.
    let warmup = 6144usize;
    let d1: Vec<u8> = (0..warmup).map(|i| (i % 251) as u8).collect();
    tcp_send(client_port, port, &d1);
    let mut got: Vec<u8> = Vec::new();
    for _ in 0..8000 {
        poll();
        if let Some(chunk) = tcp_read(port, client_port, 8192) {
            got.extend_from_slice(&chunk);
        }
        if got.len() >= warmup && tcp::all_data_acked(client_port, port) == Some(true) {
            break;
        }
        crate::apic::pit_sleep_us(300);
    }

    // Phase 2: five MSS-sized segments in one burst with the first and third dropped (mask 0b00101 = 5).
    let fast_before = tcp::fast_retransmits();
    let sack_before = tcp::sack_retransmits();
    let rto_before = tcp_retransmits();
    DROP_TCP_TX_MASK.store(0b0_0101, Ordering::Release);
    let payload_len = 5 * 1024usize; // 5 * MSS
    let d2: Vec<u8> = (0..payload_len).map(|i| ((i + 7) % 251) as u8).collect();
    tcp_send(client_port, port, &d2);

    let total = warmup + payload_len;
    for _ in 0..6000 {
        poll();
        if let Some(chunk) = tcp_read(port, client_port, 8192) {
            got.extend_from_slice(&chunk);
        }
        if got.len() >= total && tcp::all_data_acked(client_port, port) == Some(true) {
            break;
        }
        crate::apic::pit_sleep_us(300);
    }
    DROP_TCP_TX_MASK.store(0, Ordering::Release);
    e1000::set_loopback(false);

    let mut expected = d1.clone();
    expected.extend_from_slice(&d2);
    let fast_fired = tcp::fast_retransmits() > fast_before; // a fast retransmit fired
    let extra_hole = tcp::sack_retransmits() > sack_before; // SACK recovered a second hole in the same event
    let no_rto = tcp_retransmits() == rto_before; // recovery beat the RTO timer
    let in_order = got == expected && got.len() == total;
    let ok = fast_fired && extra_hole && no_rto && in_order;
    serial_println!(
        "[net] TCP SACK recovery loopback selftest: fast-retransmits {}, extra-sack-holes {}, rto-resends {}, delivered {}/{} in order {}, ok = {}",
        tcp::fast_retransmits() - fast_before,
        tcp::sack_retransmits() - sack_before,
        tcp_retransmits() - rto_before,
        got.len(),
        total,
        got == expected,
        ok,
    );
    ok
}

/// Stage 22b: the application consuming received data — drain up to `max` bytes from the connection's
/// receive buffer, reopening the flow-control window. Thin pass-through to [`tcp::read`].
pub fn tcp_read(local_port: u16, remote_port: u16, max: usize) -> Option<Vec<u8>> {
    tcp::read(local_port, remote_port, max)
}

/// Stage 22b: the receive window a connection currently advertises (free receive-buffer space). Thin
/// pass-through to [`tcp::receive_window`].
pub fn tcp_receive_window(local_port: u16, remote_port: u16) -> Option<u16> {
    tcp::receive_window(local_port, remote_port)
}

/// Stage 22b self-test of **flow control** (the receiver's sliding window) with no external peer, via PHY
/// loopback. Establish a loopback connection, then:
///
/// 1. **Fill** the receiver's window exactly (the sender still ignores the window in this sub-step, so we
///    simply hand it `RCV_WINDOW_MAX` bytes): the receiver accepts them and its advertised window shrinks
///    to **zero** as the unread bytes pile up.
/// 2. **Overrun** it: send one more segment while the window is zero — the receiver must *refuse* it (drop,
///    not buffer), so its receive buffer and `rcv_nxt` do not move. Flow control in action.
/// 3. **Read**: the application drains some bytes, so the window **reopens** to that many bytes.
/// 4. **Re-admit**: with room again, the sender's retransmission timer (Stage 21e) resends the refused
///    segment and it is now accepted — proving the reopened window actually admits data.
///
/// Returns whether all four held.
pub fn tcp_flow_control_loopback_selftest() -> bool {
    tcp::reset_connections();
    let port: u16 = 7782;
    tcp::open_passive(port);

    e1000::set_loopback(true);
    let mut sink = [0u8; 2048];
    while e1000::poll_frame(&mut sink).is_some() {}

    let client_port = match tcp_connect(our_ip(), port) {
        Some(p) => p,
        None => {
            e1000::set_loopback(false);
            serial_println!("[net] TCP flow-control loopback selftest: handshake failed");
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

    // Disable Nagle so each sub-MSS chunk goes out immediately (Stage 23c would otherwise coalesce them
    // while data is unacknowledged); this test drives the window with precisely-sized chunks.
    tcp::set_nodelay(client_port, port, true);

    // 1. Fill the receiver's window exactly, in chunks small enough to fit a loopback frame. The advertised
    //    window (free receive-buffer space) shrinks to zero as the unread bytes accumulate.
    let window_max = tcp_receive_window(port, client_port).unwrap_or(0) as usize;
    let chunk = [b'A'; 512];
    let mut sent = 0;
    while sent < window_max {
        let n = core::cmp::min(chunk.len(), window_max - sent);
        tcp_send(client_port, port, &chunk[..n]);
        sent += n;
        for _ in 0..50 {
            poll();
            if tcp::all_data_acked(client_port, port) == Some(true) {
                break;
            }
            crate::apic::pit_sleep_us(200);
        }
    }
    let filled_window = tcp_receive_window(port, client_port).unwrap_or(0xffff);
    let filled_rx = tcp::received_data(port, client_port).map(|d| d.len()).unwrap_or(0);

    // 2. Window is closed. One more segment must be refused (dropped, not buffered): the receive buffer and
    //    the next-expected sequence number stay put. (The receiver still sends a zero-window ACK, which the
    //    sender treats as a duplicate — so the segment stays on the sender's retransmit queue.)
    let extra = b"beyond-the-window!!!"; // 20 bytes
    tcp_send(client_port, port, extra);
    for _ in 0..100 {
        poll();
        crate::apic::pit_sleep_us(200);
    }
    let after_extra_rx = tcp::received_data(port, client_port).map(|d| d.len()).unwrap_or(0);
    let refused = filled_window == 0 && filled_rx == window_max && after_extra_rx == filled_rx;

    // 3. The application reads some bytes, reopening the window by exactly that many.
    let read_n = 512;
    let drained = tcp_read(port, client_port, read_n).map(|d| d.len()).unwrap_or(0);
    let reopened_window = tcp_receive_window(port, client_port).unwrap_or(0) as usize;

    // 4. With room again, the sender's retransmission timer resends the refused segment; the receiver now
    //    accepts it. Pump long enough for the ~150 ms RTO.
    let want_rx = filled_rx - drained + extra.len();
    let mut admitted = false;
    for _ in 0..4000 {
        poll();
        if tcp::received_data(port, client_port).map(|d| d.len()) == Some(want_rx)
            && tcp::all_data_acked(client_port, port) == Some(true)
        {
            admitted = true;
            break;
        }
        crate::apic::pit_sleep_us(500);
    }

    e1000::set_loopback(false);

    let ok = refused && drained == read_n && reopened_window == read_n && admitted;
    serial_println!(
        "[net] TCP flow-control loopback selftest: filled rx {} window {}, refused-beyond {}, read {} -> window {}, re-admitted {}, ok = {}",
        filled_rx, filled_window, refused, drained, reopened_window, admitted, ok,
    );
    ok
}

/// Stage 22c self-test of the **sender's sliding window** with no external peer, via PHY loopback — the
/// send-side counterpart to [`tcp_flow_control_loopback_selftest`]. Establish a loopback connection, then
/// hand the sender **more data than the peer's window** in one call and check three things:
///
/// 1. **The sender respects the window.** Right after the send, no more than the peer's advertised window
///    is in flight (the rest is buffered), and the send was **segmented** into MSS-sized pieces (not sent
///    whole) — so a slow receiver cannot be overrun.
/// 2. **The excess is buffered, not lost.** The bytes the window had no room for wait in the send buffer.
/// 3. **It all arrives, in order.** As the application reads (reopening the window), the buffered remainder
///    flows out — via the zero-window probe once the window had closed — until every byte has arrived in
///    the original order and been acknowledged.
///
/// Returns whether all three held.
pub fn tcp_sender_window_loopback_selftest() -> bool {
    tcp::reset_connections();
    let port: u16 = 7783;
    tcp::open_passive(port);

    e1000::set_loopback(true);
    let mut sink = [0u8; 2048];
    while e1000::poll_frame(&mut sink).is_some() {}

    let client_port = match tcp_connect(our_ip(), port) {
        Some(p) => p,
        None => {
            e1000::set_loopback(false);
            serial_println!("[net] TCP sender-window loopback selftest: handshake failed");
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

    // Hand the sender more than the peer's window in one call. A distinct byte pattern lets us verify the
    // bytes arrive in the original order after being segmented, buffered, and probed out.
    let window = tcp::receive_window(port, client_port).unwrap_or(0) as usize;
    let total = window + 512;
    let data: Vec<u8> = (0..total).map(|i| (i % 251) as u8).collect();

    let segs_before = tcp::data_segments_sent();
    tcp_send(client_port, port, &data);

    // 1: immediately (before any ACK is processed) the sender has capped in-flight data at the current send
    // window and buffered the rest. Since Stage 22d the effective window is min(advertised, cwnd), and slow
    // start opens cwnd at one MSS — so the first burst is only one MSS, well under the receiver's two-MSS
    // window — but either way in-flight never exceeds the advertised window and in-flight + buffered is the
    // whole send. (That the send is *segmented* into >= 2 MSS pieces is checked after the transfer, below:
    // under slow start those pieces leave over several round trips as cwnd opens, not all at once.)
    let in_flight = tcp::bytes_in_flight(client_port, port).unwrap_or(0) as usize;
    let buffered = tcp::send_buffered(client_port, port).unwrap_or(0);
    let respected = in_flight <= window && buffered > 0 && in_flight + buffered == total;

    // 2 & 3: drain the receiver as data arrives, reopening the window so the buffered remainder flows out (the
    // zero-window probe redelivers it once the window had closed), until every byte has arrived in order.
    let mut got: Vec<u8> = Vec::new();
    for _ in 0..8000 {
        poll();
        if let Some(chunk) = tcp_read(port, client_port, 4096) {
            got.extend_from_slice(&chunk);
        }
        if got.len() >= total && tcp::all_data_acked(client_port, port) == Some(true) {
            break;
        }
        crate::apic::pit_sleep_us(300);
    }
    e1000::set_loopback(false);

    // The large send was split into at least two MSS-sized segments — counted over the whole transfer,
    // since under Stage 22d's slow start they leave over several round trips (as cwnd opens), not at once.
    let segmented = tcp::data_segments_sent() - segs_before >= 2;
    let in_order = got == data;
    let ok = respected && segmented && in_order && got.len() == total;
    serial_println!(
        "[net] TCP sender-window loopback selftest: window {}, first-burst-in-flight {}, buffered {}, segmented {}, delivered {}/{} in order {}, ok = {}",
        window, in_flight, buffered, segmented, got.len(), total, in_order, ok,
    );
    ok
}

/// Stage 22d self-test of **congestion control** (slow start) with no external peer, via PHY loopback — the
/// follow-on to [`tcp_sender_window_loopback_selftest`]. Where Stage 22c paced the sender to the *peer's*
/// advertised window (flow control), Stage 22d adds a second limit — the **congestion window** (`cwnd`) —
/// pacing it to the *network*. The sender may put only `min(snd_wnd, cwnd)` bytes in flight.
///
/// This establishes a loopback connection and notes its initial `cwnd` — one MSS, because slow start starts
/// small — then streams several segments' worth of data while draining the receiver so ACKs keep flowing
/// back. Each ACK that confirms new data grows `cwnd` by one MSS (slow start), so by the end `cwnd` has
/// climbed well above its initial value, and every byte still arrives in order. It returns whether `cwnd`
/// grew and the stream was intact. (Because the loopback receive window is only two MSS, `cwnd` is genuinely
/// the binding limit at the start — the very first flush sends one MSS, not two — so this really exercises
/// congestion pacing, not just flow-control pacing.)
pub fn tcp_congestion_control_loopback_selftest() -> bool {
    tcp::reset_connections();
    let port: u16 = 7784;
    tcp::open_passive(port);

    e1000::set_loopback(true);
    let mut sink = [0u8; 2048];
    while e1000::poll_frame(&mut sink).is_some() {}

    let client_port = match tcp_connect(our_ip(), port) {
        Some(p) => p,
        None => {
            e1000::set_loopback(false);
            serial_println!("[net] TCP congestion-control loopback selftest: handshake failed");
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

    // The sender's congestion window right after the handshake — slow start begins at one MSS.
    let cwnd_initial = tcp::congestion_window(client_port, port).unwrap_or(0);

    // Stream several segments' worth of data, draining the receiver as it arrives so ACKs keep flowing back
    // (each new-data ACK grows cwnd). A distinct byte pattern lets us confirm the bytes arrive in order.
    let total = 8192usize;
    let data: Vec<u8> = (0..total).map(|i| (i % 251) as u8).collect();
    tcp_send(client_port, port, &data);

    let mut got: Vec<u8> = Vec::new();
    let mut cwnd_max = cwnd_initial;
    for _ in 0..8000 {
        poll();
        if let Some(c) = tcp::congestion_window(client_port, port) {
            cwnd_max = cwnd_max.max(c);
        }
        if let Some(chunk) = tcp_read(port, client_port, 4096) {
            got.extend_from_slice(&chunk);
        }
        if got.len() >= total && tcp::all_data_acked(client_port, port) == Some(true) {
            break;
        }
        crate::apic::pit_sleep_us(300);
    }
    e1000::set_loopback(false);

    let grew = cwnd_max > cwnd_initial;
    let in_order = got == data;
    let ok = grew && in_order && got.len() == total;
    serial_println!(
        "[net] TCP congestion-control loopback selftest: cwnd {} -> {}, delivered {}/{} in order {}, ok = {}",
        cwnd_initial, cwnd_max, got.len(), total, in_order, ok,
    );
    ok
}

/// Stage 22d-2 self-test of the **congestion backoff on loss** with no external peer, via PHY loopback — the
/// follow-on to [`tcp_congestion_control_loopback_selftest`]. Slow start (22d-1) only opens `cwnd`; this
/// exercises the *close* — the multiplicative-decrease half of AIMD — on a retransmission timeout.
///
/// Two phases. **Phase 1** streams a batch and drains it so `cwnd` grows well above one MSS (with `ssthresh`
/// still at its initial near-infinity). **Phase 2** arms the loss-injection hook so the next data segment is
/// silently dropped: no ACK comes, so only the RTO timer recovers it — and when it fires it also applies
/// [`tcp::on_rto`], collapsing `cwnd` back to one MSS and halving `ssthresh` to the flight size. The test
/// confirms `cwnd` fell back from its grown value to about one MSS, `ssthresh` dropped from near-infinity to
/// a small value, and every byte (both phases) still arrived in order once the segment was recovered.
pub fn tcp_congestion_backoff_loopback_selftest() -> bool {
    tcp::reset_connections();
    let port: u16 = 7785;
    tcp::open_passive(port);

    e1000::set_loopback(true);
    let mut sink = [0u8; 2048];
    while e1000::poll_frame(&mut sink).is_some() {}

    let client_port = match tcp_connect(our_ip(), port) {
        Some(p) => p,
        None => {
            e1000::set_loopback(false);
            serial_println!("[net] TCP congestion-backoff loopback selftest: handshake failed");
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

    // Phase 1: grow cwnd above one MSS by streaming a batch and draining it (all delivered and acknowledged).
    let warmup = 6144usize;
    let d1: Vec<u8> = (0..warmup).map(|i| (i % 251) as u8).collect();
    tcp_send(client_port, port, &d1);
    let mut got: Vec<u8> = Vec::new();
    for _ in 0..8000 {
        poll();
        if let Some(chunk) = tcp_read(port, client_port, 4096) {
            got.extend_from_slice(&chunk);
        }
        if got.len() >= warmup && tcp::all_data_acked(client_port, port) == Some(true) {
            break;
        }
        crate::apic::pit_sleep_us(300);
    }
    let cwnd_grown = tcp::congestion_window(client_port, port).unwrap_or(0);
    let ssthresh_before = tcp::slow_start_threshold(client_port, port).unwrap_or(0);

    // Phase 2: drop the next data segment so no ACK returns — only the RTO can recover it. When the timer
    // fires it resends *and* backs the sender off: cwnd -> one MSS, ssthresh -> half the flight.
    DROP_NEXT_TCP_TX.store(1, Ordering::Release);
    let d2 = b"aether tcp congestion backoff 22d-2";
    tcp_send(client_port, port, d2);

    let total = warmup + d2.len();
    let mut cwnd_min = cwnd_grown;
    for _ in 0..6000 {
        poll();
        if let Some(c) = tcp::congestion_window(client_port, port) {
            cwnd_min = cwnd_min.min(c);
        }
        if let Some(chunk) = tcp_read(port, client_port, 4096) {
            got.extend_from_slice(&chunk);
        }
        if got.len() >= total && tcp::all_data_acked(client_port, port) == Some(true) {
            break;
        }
        crate::apic::pit_sleep_us(300);
    }
    let ssthresh_after = tcp::slow_start_threshold(client_port, port).unwrap_or(0);
    e1000::set_loopback(false);

    let mut expected = d1.clone();
    expected.extend_from_slice(d2);
    let grew = cwnd_grown > 1024; // phase 1 opened cwnd past one MSS, so there was something to collapse
    let collapsed = cwnd_min < cwnd_grown && cwnd_min <= 2048; // the RTO backed cwnd off to about one MSS
    let ssthresh_dropped = ssthresh_after < ssthresh_before; // multiplicative decrease lowered the threshold
    let in_order = got == expected;
    let ok = grew && collapsed && ssthresh_dropped && in_order && got.len() == total;
    serial_println!(
        "[net] TCP congestion-backoff loopback selftest: cwnd {} -> {} (min after loss), ssthresh {} -> {}, delivered {}/{} in order {}, ok = {}",
        cwnd_grown, cwnd_min, ssthresh_before, ssthresh_after, got.len(), total, in_order, ok,
    );
    ok
}

/// Stage 22d-3 self-test of **fast retransmit + fast recovery** with no external peer, via PHY loopback — the
/// final congestion-control sub-step. Stage 22d-2 recovered a loss only via the RTO (wait a whole timeout,
/// then collapse `cwnd` to one MSS). Fast retransmit recovers *sooner* and *gentler*: the receiver's
/// duplicate ACKs (a later segment arrived but an earlier one is missing) tell the sender about the loss
/// before the timer would, and after resending it the sender only halves `cwnd` (fast recovery) rather than
/// collapsing it — because the dup ACKs prove data is still flowing, so the path is not fully congested.
///
/// The test grows `cwnd` past four MSS (so four segments can be in flight), then sends four MSS-sized
/// segments in one burst with the **first dropped**. The three that arrive are out of order, so the receiver
/// sends three duplicate ACKs; the third fires the fast retransmit of the missing segment. It confirms the
/// fast-retransmit path ran, the RTO timer did *not* fire (recovery beat it), `cwnd` never collapsed to one
/// MSS (fast recovery, not an RTO backoff), and every byte still arrived in order.
pub fn tcp_fast_retransmit_loopback_selftest() -> bool {
    tcp::reset_connections();
    let port: u16 = 7786;
    tcp::open_passive(port);

    e1000::set_loopback(true);
    let mut sink = [0u8; 2048];
    while e1000::poll_frame(&mut sink).is_some() {}

    let client_port = match tcp_connect(our_ip(), port) {
        Some(p) => p,
        None => {
            e1000::set_loopback(false);
            serial_println!("[net] TCP fast-retransmit loopback selftest: handshake failed");
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

    // Phase 1: grow cwnd above four MSS so four segments can be in flight at once — the setup needed to put
    // three later segments behind one lost segment (three duplicate ACKs). Stream a batch and drain it.
    let warmup = 6144usize;
    let d1: Vec<u8> = (0..warmup).map(|i| (i % 251) as u8).collect();
    tcp_send(client_port, port, &d1);
    let mut got: Vec<u8> = Vec::new();
    for _ in 0..8000 {
        poll();
        if let Some(chunk) = tcp_read(port, client_port, 8192) {
            got.extend_from_slice(&chunk);
        }
        if got.len() >= warmup && tcp::all_data_acked(client_port, port) == Some(true) {
            break;
        }
        crate::apic::pit_sleep_us(300);
    }

    // Phase 2: send four MSS-sized segments in one burst with the first dropped on the wire. The three that
    // arrive are ahead of the receiver's next-expected byte (a gap precedes them), so it dup-ACKs each; the
    // third dup ACK fires the fast retransmit — before the ~150 ms RTO — and enters fast recovery.
    let fast_before = tcp::fast_retransmits();
    let rto_before = tcp_retransmits();
    DROP_NEXT_TCP_TX.store(1, Ordering::Release);
    let payload_len = 4096usize; // 4 * MSS
    let d2: Vec<u8> = (0..payload_len).map(|i| ((i + 7) % 251) as u8).collect();
    tcp_send(client_port, port, &d2);

    let total = warmup + payload_len;
    let mut cwnd_min = u32::MAX;
    for _ in 0..6000 {
        poll();
        if let Some(c) = tcp::congestion_window(client_port, port) {
            cwnd_min = cwnd_min.min(c);
        }
        if let Some(chunk) = tcp_read(port, client_port, 8192) {
            got.extend_from_slice(&chunk);
        }
        if got.len() >= total && tcp::all_data_acked(client_port, port) == Some(true) {
            break;
        }
        crate::apic::pit_sleep_us(300);
    }
    e1000::set_loopback(false);

    let mut expected = d1.clone();
    expected.extend_from_slice(&d2);
    let fast_fired = tcp::fast_retransmits() > fast_before; // the third dup ACK triggered a fast retransmit
    let no_rto = tcp_retransmits() == rto_before; // recovery beat the RTO timer — it never resent anything
    let no_collapse = cwnd_min > 1024; // fast recovery halves cwnd; an RTO would have collapsed it to 1 MSS
    let in_order = got == expected;
    let ok = fast_fired && no_rto && no_collapse && in_order && got.len() == total;
    serial_println!(
        "[net] TCP fast-retransmit loopback selftest: fast-retransmits {}, rto-resends {}, cwnd-min-after-loss {}, delivered {}/{} in order {}, ok = {}",
        tcp::fast_retransmits() - fast_before,
        tcp_retransmits() - rto_before,
        cwnd_min,
        got.len(),
        total,
        in_order,
        ok,
    );
    ok
}

/// Stage 23a self-test of **adaptive RTO** (RFC 6298 RTT estimation) — the first Stage 23 refinement.
/// Two parts. (1) The pure estimator formula on known synthetic samples (`tcp::rtt_estimator_selftest`),
/// since loopback RTTs are below our 10 ms tick granularity and so only exercise the RTO floor. (2) A live
/// loopback transfer, confirming the sender actually *sampled* an RTT (its estimator went valid) and that
/// the resulting RTO is within the sane `[floor, ceiling]` band while the data still arrives in order.
pub fn tcp_rtt_estimation_loopback_selftest() -> bool {
    // Part 1: the pure RFC 6298 recurrence and its clamping, on hand-chosen samples.
    let formula_ok = tcp::rtt_estimator_selftest();

    tcp::reset_connections();
    let port: u16 = 7787;
    tcp::open_passive(port);

    e1000::set_loopback(true);
    let mut sink = [0u8; 2048];
    while e1000::poll_frame(&mut sink).is_some() {}

    let client_port = match tcp_connect(our_ip(), port) {
        Some(p) => p,
        None => {
            e1000::set_loopback(false);
            serial_println!("[net] TCP RTT-estimation loopback selftest: handshake failed");
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

    // Part 2: transfer data so ACKs return and the sender folds RTT samples into its estimator.
    let total = 4096usize;
    let data: Vec<u8> = (0..total).map(|i| (i % 251) as u8).collect();
    tcp_send(client_port, port, &data);
    let mut got: Vec<u8> = Vec::new();
    for _ in 0..8000 {
        poll();
        if let Some(chunk) = tcp_read(port, client_port, 8192) {
            got.extend_from_slice(&chunk);
        }
        if got.len() >= total && tcp::all_data_acked(client_port, port) == Some(true) {
            break;
        }
        crate::apic::pit_sleep_us(300);
    }
    e1000::set_loopback(false);

    let sampled = tcp::rtt_sampled(client_port, port) == Some(true);
    let rto = tcp::current_rto(client_port, port).unwrap_or(0);
    let rto_sane = (15..=6000).contains(&rto); // >= floor, <= ceiling (loopback RTT floors it at the minimum)
    let in_order = got == data;
    let ok = formula_ok && sampled && rto_sane && in_order && got.len() == total;
    serial_println!(
        "[net] TCP RTT-estimation loopback selftest: formula {}, sampled {}, rto {} ticks, delivered {}/{} in order {}, ok = {}",
        formula_ok, sampled, rto, got.len(), total, in_order, ok,
    );
    ok
}

/// Stage 23b self-test of **delayed ACKs** (RFC 1122) via PHY loopback. The receiver no longer ACKs every
/// data segment: it ACKs at most every *second* in-order segment (or after a short timer), so a stream of N
/// segments draws **fewer than N** ACKs. This warms up `cwnd` so the sender bursts several segments at once
/// (they then arrive paired at the receiver and the "every second segment" rule coalesces them), then sends
/// a batch of in-order data and confirms the receiver sent fewer ACKs than it received data segments, with
/// the bytes still delivered in order. (Out-of-order segments still draw an *immediate* dup ACK — verified
/// by the Stage 22a/22d-3 tests, which keep passing — so fast retransmit is unaffected.)
pub fn tcp_delayed_ack_loopback_selftest() -> bool {
    tcp::reset_connections();
    let port: u16 = 7788;
    tcp::open_passive(port);

    e1000::set_loopback(true);
    let mut sink = [0u8; 2048];
    while e1000::poll_frame(&mut sink).is_some() {}

    let client_port = match tcp_connect(our_ip(), port) {
        Some(p) => p,
        None => {
            e1000::set_loopback(false);
            serial_println!("[net] TCP delayed-ACK loopback selftest: handshake failed");
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

    // Warm up: grow cwnd (and drain fully, so no delayed ACK is left pending) before measuring, so the
    // sender can burst several segments at once in the measured phase.
    let warmup: Vec<u8> = (0..4096).map(|i| (i % 251) as u8).collect();
    tcp_send(client_port, port, &warmup);
    for _ in 0..8000 {
        poll();
        let _ = tcp_read(port, client_port, 8192);
        if tcp::all_data_acked(client_port, port) == Some(true) {
            break;
        }
        crate::apic::pit_sleep_us(300);
    }

    // Measure: send a batch of in-order segments and count the ACKs the receiver produces against them.
    let total = 8192usize;
    let data: Vec<u8> = (0..total).map(|i| ((i + 3) % 251) as u8).collect();
    let acks_before = tcp::acks_sent();
    let segs_before = tcp::data_segments_sent();
    tcp_send(client_port, port, &data);

    let mut got: Vec<u8> = Vec::new();
    for _ in 0..8000 {
        poll();
        if let Some(chunk) = tcp_read(port, client_port, 8192) {
            got.extend_from_slice(&chunk);
        }
        if got.len() >= total && tcp::all_data_acked(client_port, port) == Some(true) {
            break;
        }
        crate::apic::pit_sleep_us(300);
    }
    e1000::set_loopback(false);

    let acks = tcp::acks_sent() - acks_before;
    let segs = tcp::data_segments_sent() - segs_before;
    let coalesced = acks < segs && acks > 0; // fewer ACKs than data segments -> delayed ACK worked
    let in_order = got == data;
    let ok = coalesced && in_order && got.len() == total;
    serial_println!(
        "[net] TCP delayed-ACK loopback selftest: {} data segments drew {} ACK(s), coalesced {}, delivered {}/{} in order {}, ok = {}",
        segs, acks, coalesced, got.len(), total, in_order, ok,
    );
    ok
}

/// Stage 23c self-test of **Nagle's algorithm** (RFC 896) via PHY loopback. Nagle coalesces a burst of
/// small writes: while earlier data is unacknowledged, a sub-MSS write is held (buffered) rather than sent,
/// until the outstanding data is acknowledged or a full segment accumulates — so many tiny writes leave as
/// a few packets instead of one packet each. This sends a run of one-byte writes with Nagle *on* (the
/// default) and confirms they were coalesced into **far fewer segments than writes**, with every byte still
/// delivered in order. (The complementary `TCP_NODELAY` path — send each write at once — is exercised by the
/// reassembly and flow-control self-tests, which enable it.)
pub fn tcp_nagle_loopback_selftest() -> bool {
    tcp::reset_connections();
    let port: u16 = 7789;
    tcp::open_passive(port);

    e1000::set_loopback(true);
    let mut sink = [0u8; 2048];
    while e1000::poll_frame(&mut sink).is_some() {}

    let client_port = match tcp_connect(our_ip(), port) {
        Some(p) => p,
        None => {
            e1000::set_loopback(false);
            serial_println!("[net] TCP Nagle loopback selftest: handshake failed");
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

    // Write a run of single bytes with Nagle on. The first goes out immediately (nothing outstanding); the
    // rest are held and coalesced behind it, so they leave as one further segment once the first is acked.
    let writes = 16usize;
    let expected: Vec<u8> = (0..writes).map(|i| (i + 1) as u8).collect();
    let segs_before = tcp::data_segments_sent();
    for b in &expected {
        tcp_send(client_port, port, &[*b]);
    }

    let mut got: Vec<u8> = Vec::new();
    for _ in 0..8000 {
        poll();
        if let Some(chunk) = tcp_read(port, client_port, 8192) {
            got.extend_from_slice(&chunk);
        }
        if got.len() >= writes && tcp::all_data_acked(client_port, port) == Some(true) {
            break;
        }
        crate::apic::pit_sleep_us(300);
    }
    e1000::set_loopback(false);

    let segs = tcp::data_segments_sent() - segs_before;
    let coalesced = segs < writes as u64; // far fewer segments than the 16 writes -> Nagle coalesced them
    let in_order = got == expected;
    let ok = coalesced && in_order && got.len() == writes;
    serial_println!(
        "[net] TCP Nagle loopback selftest: {} write(s) sent as {} segment(s), coalesced {}, delivered {}/{} in order {}, ok = {}",
        writes, segs, coalesced, got.len(), writes, in_order, ok,
    );
    ok
}

/// Stage 21e self-test of **retransmission** with no external peer, via PHY loopback — the follow-on to
/// [`tcp_teardown_loopback_selftest`] and the last piece that makes the transport truly *reliable*.
/// Establish a loopback connection, then send a payload with the loss-injection hook armed so that data
/// segment is silently dropped: no ACK comes back, so after the retransmission timeout the timer resends
/// it, this time delivered — the server buffers it in order and ACKs, and the sender's queue clears. Then
/// tear the connection down and confirm the active closer's TIME_WAIT expires to CLOSED under the timer.
/// Success proves both timers: loss recovery and the timed close. Returns whether all three held.
pub fn tcp_retransmit_loopback_selftest() -> bool {
    tcp::reset_connections();
    let port: u16 = 7780;
    tcp::open_passive(port);

    e1000::set_loopback(true);
    let mut sink = [0u8; 2048];
    while e1000::poll_frame(&mut sink).is_some() {}

    let client_port = match tcp_connect(our_ip(), port) {
        Some(p) => p,
        None => {
            e1000::set_loopback(false);
            serial_println!("[net] TCP retransmit loopback selftest: handshake failed");
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

    // Inject loss: the next TCP frame (our data segment) is dropped, so the server never sees it and never
    // ACKs — only the retransmission timer can recover it.
    let retransmits_before = tcp_retransmits();
    DROP_NEXT_TCP_TX.store(1, Ordering::Release);
    let payload = b"aether tcp retransmit 21e";
    tcp_send(client_port, port, payload); // this frame is dropped on the wire

    // Pump long enough for the ~150 ms RTO: the timer (serviced inside `poll`) resends the segment; this
    // time it is delivered, the server buffers it and ACKs, and the client's retransmit queue clears.
    let mut recovered = false;
    for _ in 0..4000 {
        poll();
        if tcp::received_data(port, client_port).as_deref() == Some(&payload[..])
            && tcp::all_data_acked(client_port, port) == Some(true)
        {
            recovered = true;
            break;
        }
        crate::apic::pit_sleep_us(1000);
    }
    let resent = tcp_retransmits() > retransmits_before;

    // Tear the connection down, then confirm the client's TIME_WAIT expires to CLOSED under the timer.
    tcp_close(client_port, port);
    for _ in 0..400 {
        poll();
        if tcp::connection_state(port, client_port) == Some(tcp::State::CloseWait) {
            break;
        }
        crate::apic::pit_sleep_us(500);
    }
    tcp_close(port, client_port);
    let mut closed = false;
    for _ in 0..4000 {
        poll();
        if tcp::connection_state(client_port, port) == Some(tcp::State::Closed) {
            closed = true;
            break;
        }
        crate::apic::pit_sleep_us(1000);
    }
    e1000::set_loopback(false);

    let ok = recovered && resent && closed;
    serial_println!(
        "[net] TCP retransmit loopback selftest: resends {}, recovered {}, time_wait->closed {}, ok = {}",
        tcp_retransmits() - retransmits_before,
        recovered,
        closed,
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

/// Stage 23d-1 self-test: complete a loopback handshake and confirm **both** ends negotiated SACK-permitted
/// (RFC 2018), the stack's first use of TCP options. Our SYN offers the option and the SYN-ACK echoes it
/// back, so after the handshake each TCB has the flag set — proving the option round-trips on the wire
/// (parsed past the enlarged data offset) and the negotiation records it on both sides. Returns whether
/// both connections are established and both flagged SACK-permitted.
pub fn tcp_sack_negotiation_loopback_selftest() -> bool {
    tcp::reset_connections();
    let port: u16 = 7779;
    tcp::open_passive(port); // a listener to accept our own SYN

    e1000::set_loopback(true);
    // Drain stale frames so the handshake sees a clean ring.
    let mut sink = [0u8; 2048];
    while e1000::poll_frame(&mut sink).is_some() {}

    let client_port = match tcp_connect(our_ip(), port) {
        Some(p) => p,
        None => {
            e1000::set_loopback(false);
            serial_println!("[net] TCP SACK negotiation selftest: connect failed");
            return false;
        }
    };
    // Pump until the listener also reaches ESTABLISHED (it records SACK from the looped-back SYN).
    for _ in 0..200 {
        if tcp::established_count() >= 2 {
            break;
        }
        poll();
        crate::apic::pit_sleep_us(500);
    }
    e1000::set_loopback(false);

    let established = tcp::established_count();
    // The listener's TCB is keyed by (our port, the client's ephemeral port) — the mirror of the client's.
    let client_sack = tcp::sack_permitted(client_port, port).unwrap_or(false);
    let server_sack = tcp::sack_permitted(port, client_port).unwrap_or(false);
    serial_println!(
        "[net] TCP SACK negotiation selftest: established = {}, client SACK = {}, server SACK = {}",
        established, client_sack, server_sack,
    );
    established >= 2 && client_sack && server_sack
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
