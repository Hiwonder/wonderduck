//! What the advertisement carries besides the name: the robot's IPv4 address.
//!
//! Platform-independent, and here rather than in [`crate::bluez`] for [`crate::gatt`]'s reason —
//! it is wire contract. The robot encodes with [`address_data`] and `duckctl` decodes with
//! [`address_in`], so the two halves cannot disagree about the layout. A decoder written
//! separately in the client would agree only with itself.
//!
//! ## Why the advertisement rather than a call
//!
//! `net.status` already reports the address, and reading it costs a connection, a bond and the
//! PIN — per robot. `duckctl scan` deliberately connects to nothing, which is what makes it the
//! command to reach for when a robot is unreachable, so a listing can only report what an
//! advertisement carries. Broadcasting the address is therefore the only way `scan` can answer
//! "where do I ssh?", and that is the question a listing is most often read to answer.
//!
//! ## Why four bytes and no more
//!
//! A legacy advertisement holds **31 bytes**, and `btd` already spends 21: flags (3) and a
//! 128-bit service UUID (2 + 16). One manufacturer-data field costs 2 for its header and 2 for the
//! company id, so the payload has 6 bytes to live in and this one uses 4. The name is not in that
//! budget — BlueZ puts a Local Name in the scan response, which has 31 bytes of its own.
//!
//! That is the whole reason the SSID is not here too: an SSID is up to 32 bytes on its own, so no
//! version of it fits. It stays a `wifi status` question.
//!
//! ## Why the address is always present
//!
//! A robot with no wifi advertises [`Ipv4Addr::UNSPECIFIED`] rather than dropping the field. The
//! field is then evidence in itself: absent means a robot on a release that predates this, present
//! and zero means a robot that has no address, and those two want different next moves. Dropping
//! the field would collapse them into one blank column.
//!
//! ## The extended twin carries one byte more
//!
//! `btd` puts the same payload on the air twice: once as a legacy advertisement and once as an
//! extended one, because the robot's controller cannot take a connection from a central that offers
//! channel selection algorithm #2 over a legacy advertisement. The two
//! arrive from different random addresses, so to a scanner they are two peripherals with the same
//! name — which `duckctl` refuses as a collision unless it can tell they are one robot.
//!
//! [`EXTENDED_MARK`] after the address is how it tells. The extended advertisement has no 31-byte
//! budget, so the byte costs nothing there, and the legacy payload is unchanged.

use std::collections::HashMap;
use std::net::Ipv4Addr;

/// The company id the payload is filed under.
///
/// `0xFFFF` is the id the Bluetooth SIG reserves for internal and interoperability testing, and it
/// is the correct choice for a project that has not been assigned one. Anyone else may use it too,
/// so **this is not an identity check**: [`address_in`] is only ever asked about a device that
/// already advertised [`crate::gatt::SERVICE_UUID`], which is the discriminator.
pub const COMPANY_ID: u16 = 0xFFFF;

/// The byte after the address that says "this is the extended twin of a legacy advertisement".
pub const EXTENDED_MARK: u8 = 0x01;

/// The robot's address as it goes into the advertisement.
///
/// `None` — no wifi, or `configd` would not say — becomes [`Ipv4Addr::UNSPECIFIED`] rather than an
/// absent field, for the reason in this module's docs.
pub fn address_data(address: Option<Ipv4Addr>) -> Vec<u8> {
    address.unwrap_or(Ipv4Addr::UNSPECIFIED).octets().to_vec()
}

/// [`address_data`] followed by [`EXTENDED_MARK`], for the extended advertisement.
pub fn extended_address_data(address: Option<Ipv4Addr>) -> Vec<u8> {
    let mut data = address_data(address);
    data.push(EXTENDED_MARK);
    data
}

/// The four address bytes of a field in either layout, or `None` for anything else.
fn address_bytes(manufacturer_data: &HashMap<u16, Vec<u8>>) -> Option<[u8; 4]> {
    match manufacturer_data.get(&COMPANY_ID)?.as_slice() {
        [a, b, c, d] | [a, b, c, d, EXTENDED_MARK] => Some([*a, *b, *c, *d]),
        _ => None,
    }
}

/// Whether this is the extended twin — see this module's docs.
pub fn is_extended(manufacturer_data: &HashMap<u16, Vec<u8>>) -> bool {
    manufacturer_data
        .get(&COMPANY_ID)
        .is_some_and(|data| data.len() == 5 && data[4] == EXTENDED_MARK)
}

/// The address a scan reported, if it reported one.
///
/// `None` covers three cases that a listing renders differently and this function does not
/// distinguish, because it cannot: no field at all, a field of the wrong length, and a field
/// saying `0.0.0.0`. The caller has the advertisement and can tell the first from the third; see
/// `duckctl`'s listing, which does.
pub fn address_in(manufacturer_data: &HashMap<u16, Vec<u8>>) -> Option<Ipv4Addr> {
    Some(Ipv4Addr::from(address_bytes(manufacturer_data)?))
        .filter(|address| !address.is_unspecified())
}

/// Whether this device broadcast an address field at all, however it reads.
///
/// Separate from [`address_in`] because "an older release, which broadcasts nothing" and "a robot
/// that is not on wifi" are the two things a blank address could mean, and a listing that cannot
/// tell them apart sends the reader to check the wrong thing.
pub fn has_address_field(manufacturer_data: &HashMap<u16, Vec<u8>>) -> bool {
    address_bytes(manufacturer_data).is_some()
}
