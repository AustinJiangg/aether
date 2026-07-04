//! Stage 17a: PCI bus enumeration — finding the network card.
//!
//! Before the kernel can drive the Intel e1000 NIC (Stage 17b onward), it has to *find* it. The
//! NIC is a PCI device, so we enumerate the PCI bus and read each device's identity out of its
//! configuration space.
//!
//! ## PCI configuration space
//!
//! Every PCI function has 256 bytes of *configuration space* — a standardized header the firmware
//! and OS read to identify the device and learn where its registers live. The first dwords are
//! fixed by the spec: the vendor/device id, the class code (what kind of device it is), and the six
//! **Base Address Registers** (BARs), each reporting where one of the device's register regions is
//! mapped (in physical memory, for an MMIO BAR, or in I/O-port space).
//!
//! On the PC, configuration space is reached through two 32-bit I/O ports — the legacy
//! "Configuration Access Mechanism #1": write a `(bus, device, function, offset)` address to
//! `CONFIG_ADDRESS` (`0xCF8`), then read or write the dword at `CONFIG_DATA` (`0xCFC`). A slot with
//! no device reads back all-ones (vendor id `0xFFFF`), which is how enumeration knows to skip it.
//!
//! Stage 17a only *reads*: enumerate the bus, list what is present, and locate the e1000 (vendor
//! `0x8086`, device `0x100E` — QEMU's `-device e1000`) so later stages can map its registers.

use alloc::vec::Vec;

use spin::Mutex;
use x86_64::instructions::port::Port;

/// `CONFIG_ADDRESS`: the port you write an encoded config-space address to.
const CONFIG_ADDRESS: u16 = 0xCF8;
/// `CONFIG_DATA`: the port you then read/write the addressed 32-bit dword through.
const CONFIG_DATA: u16 = 0xCFC;

/// Intel's PCI vendor id.
pub const INTEL_VENDOR_ID: u16 = 0x8086;
/// Device id of the Intel 82540EM — the card behind QEMU's `-device e1000`.
pub const E1000_DEVICE_ID: u16 = 0x100E;

/// PCI class code for a network controller, and the Ethernet subclass under it.
pub const CLASS_NETWORK: u8 = 0x02;
pub const SUBCLASS_ETHERNET: u8 = 0x00;

/// An empty slot reads back `0xFFFF` in the vendor-id field.
const INVALID_VENDOR: u16 = 0xFFFF;

/// The address/data protocol is two steps (write the address, then touch the data port), so two
/// CPUs racing on these ports would interleave and read each other's dword. Serialize every access
/// with this lock.
static CONFIG_LOCK: Mutex<()> = Mutex::new(());

/// The bus/device/function triple ("BDF") that names one PCI function.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Address {
    pub bus: u8,
    pub device: u8,   // 0..32
    pub function: u8, // 0..8
}

impl Address {
    /// Encode this BDF plus a dword-aligned `offset` into the value `CONFIG_ADDRESS` expects:
    /// bit 31 = enable, then the bus/device/function fields, then the register offset (its low two
    /// bits forced to zero, because configuration space is addressed one 32-bit dword at a time).
    fn encode(self, offset: u8) -> u32 {
        (1 << 31)
            | ((self.bus as u32) << 16)
            | ((self.device as u32) << 11)
            | ((self.function as u32) << 8)
            | ((offset as u32) & 0xFC)
    }
}

/// Read one 32-bit dword from `addr`'s configuration space at `offset` (dword-aligned).
pub fn read_config_u32(addr: Address, offset: u8) -> u32 {
    let _guard = CONFIG_LOCK.lock();
    let mut address_port = Port::<u32>::new(CONFIG_ADDRESS);
    let mut data_port = Port::<u32>::new(CONFIG_DATA);
    // SAFETY: 0xCF8/0xCFC are the architected PCI configuration ports. Writing the encoded address
    // then reading the data port is the standard access mechanism #1; it touches only those two
    // ports and no memory, and an absent device harmlessly reads back all-ones.
    unsafe {
        address_port.write(addr.encode(offset));
        data_port.read()
    }
}

/// A discovered PCI function: where it lives, plus the identity fields Stage 17 needs.
#[derive(Debug, Clone, Copy)]
pub struct Device {
    pub address: Address,
    pub vendor_id: u16,
    pub device_id: u16,
    pub class: u8,
    pub subclass: u8,
    pub prog_if: u8,
    /// Header-type byte; bit 7 flags a multifunction device.
    pub header_type: u8,
}

impl Device {
    /// Probe `addr`: return the function that lives there, or `None` if the slot is empty (its
    /// vendor id reads back all-ones). Reads the identity dwords: `0x00` holds vendor+device,
    /// `0x08` holds the class code (revision/prog-IF/subclass/class, low to high), and the
    /// header-type byte sits in the dword at `0x0C`.
    fn probe(addr: Address) -> Option<Device> {
        let id = read_config_u32(addr, 0x00);
        let vendor_id = id as u16;
        if vendor_id == INVALID_VENDOR {
            return None;
        }
        let class_dword = read_config_u32(addr, 0x08);
        let header_type = (read_config_u32(addr, 0x0C) >> 16) as u8;
        Some(Device {
            address: addr,
            vendor_id,
            device_id: (id >> 16) as u16,
            prog_if: (class_dword >> 8) as u8,
            subclass: (class_dword >> 16) as u8,
            class: (class_dword >> 24) as u8,
            header_type,
        })
    }

    /// Whether this is a multifunction device (header-type bit 7): only then is it worth probing
    /// functions 1..8 of the same device slot.
    fn is_multifunction(&self) -> bool {
        self.header_type & 0x80 != 0
    }

    /// Read Base Address Register `index` (0..6) — the raw dword, undecoded.
    pub fn bar(&self, index: u8) -> u32 {
        read_config_u32(self.address, 0x10 + index * 4)
    }

    /// Decode BAR `index` as a memory-mapped base address, or `None` if it is an I/O-space BAR
    /// (bit 0 set). Handles both 32-bit and 64-bit memory BARs — a 64-bit BAR (type bits `0b10`)
    /// keeps its high 32 bits in the *next* BAR slot. The e1000's BAR0 is a 32-bit memory BAR.
    pub fn mmio_bar(&self, index: u8) -> Option<u64> {
        let low = self.bar(index);
        if low & 1 != 0 {
            return None; // bit 0 set => an I/O-space BAR, not memory-mapped
        }
        let base = (low & 0xFFFF_FFF0) as u64;
        match (low >> 1) & 0b11 {
            0b10 => Some(base | ((self.bar(index + 1) as u64) << 32)), // 64-bit: high half next BAR
            _ => Some(base),                                           // 32-bit memory BAR
        }
    }

    /// The interrupt line (legacy IRQ number) the firmware assigned, from config offset `0x3C`.
    pub fn interrupt_line(&self) -> u8 {
        read_config_u32(self.address, 0x3C) as u8
    }
}

/// Enumerate every present PCI function, function-by-function. Brute-forces all 256 buses: QEMU's
/// default machine populates only bus 0, but probing the rest is harmless (an absent slot reads
/// back all-ones). Multifunction devices (header-type bit 7) have their functions 1..8 probed too.
pub fn enumerate() -> Vec<Device> {
    let mut devices = Vec::new();
    for bus in 0u16..256 {
        for device in 0u8..32 {
            let base = Address { bus: bus as u8, device, function: 0 };
            let function0 = match Device::probe(base) {
                Some(d) => d,
                None => continue, // empty slot
            };
            devices.push(function0);

            if function0.is_multifunction() {
                for function in 1u8..8 {
                    let addr = Address { bus: bus as u8, device, function };
                    if let Some(d) = Device::probe(addr) {
                        devices.push(d);
                    }
                }
            }
        }
    }
    devices
}

/// Find the first function matching `vendor_id`/`device_id`, or `None` if none is present.
pub fn find_device(vendor_id: u16, device_id: u16) -> Option<Device> {
    enumerate()
        .into_iter()
        .find(|d| d.vendor_id == vendor_id && d.device_id == device_id)
}

/// Find the Intel e1000 NIC QEMU attaches with `-device e1000`, if present.
pub fn find_e1000() -> Option<Device> {
    find_device(INTEL_VENDOR_ID, E1000_DEVICE_ID)
}
