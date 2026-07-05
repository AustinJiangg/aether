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

pub mod arp;
pub mod ether;

use core::sync::atomic::{AtomicU64, Ordering};

use spin::Mutex;

use crate::{e1000, serial_println};
use ether::MacAddr;

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
        }
        _ => {} // some other EtherType — ignored for now
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
