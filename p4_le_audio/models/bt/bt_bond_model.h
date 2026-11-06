#pragma once
// BLE bond (pairing remembered by the host), pure part: recognising a paired device even after its private
// address (RPA) changes, and storing the bond in the key/value store. No HAL - host tests
// (test/host/test_bt_bond.cpp).
//
// LE Audio earbuds (e.g. Galaxy Buds3 Pro) connect from a resolvable private address that changes every few
// minutes; without their IRK the host took them for a new device and paired again, and the earbuds, which kept the
// bond, rejected that (observed 2026-10-31). Sources: Core 5.4 Vol 6 Part B 1.3.2.2 (resolvable private address),
// Vol 3 Part H 2.2.2 (ah) and 3.6.4 (Identity Information / Identity Address Information).

#include <cstddef>
#include <cstdint>
#include <string>

namespace app::btbond {

// AES-128 of one block, key and data MSB first - e() from Core Vol 3 Part H 2.2.1.
using Aes128Fn = bool (*)(const uint8_t keyBe[16], const uint8_t inBe[16], uint8_t outBe[16]);

struct Bond {
    bool valid       = false;
    uint8_t addr[6]  = {};  // identity address (Identity Address Information), else the pairing address; LSB first
    uint8_t ltk[16]  = {};  // LSB first, as in HCI_LE_Enable_Encryption
    bool hasIrk      = false;
    uint8_t irk[16]  = {};  // LSB first, as in SMP Identity Information
};

// Random address (HCI type 1, also 3 = random identity) with the two most significant bits 01. addr LSB first.
bool isResolvablePrivate(uint8_t addrType, const uint8_t addr[6]);

// ah(IRK, prand) == hash: prand = upper 24 bits of the address, hash = lower 24. false also on an AES error.
bool rpaMatches(const uint8_t irk[16], const uint8_t addr[6], Aes128Fn aes);

// Connection from a bonded device: the same address, or a private address resolved with its IRK.
bool bondMatches(const Bond& b, uint8_t addrType, const uint8_t addr[6], Aes128Fn aes);

// kv format (hex): address 6 B + LTK 16 B [+ IRK 16 B]. The older format (address + LTK only) parses without an IRK.
std::string formatBond(const Bond& b);
// false for a malformed string (length, characters) - b unchanged.
bool parseBond(const std::string& s, Bond& b);

// Several bonds: each Galaxy Buds3 Pro earbud is a separate device (own identity address, IRK and LTK). With a single
// stored bond the second earbud paired again from scratch, and the first, which kept its bond, did not respond
constexpr size_t kMaxBonds = 4;
struct BondList {
    Bond items[kMaxBonds];  // most recently paired first
};

size_t bondCount(const BondList& l);
// Index of the bond of the connecting device (bondMatches), -1 = unknown.
int findBond(const BondList& l, uint8_t addrType, const uint8_t addr[6], Aes128Fn aes);
// Puts a bond from a new pairing at the front. An entry of the same device (same address or IRK) is removed, and when
// the list is full the oldest paired one is dropped.
void putBond(BondList& l, const Bond& b);
// Removes an entry (the earbuds rejected its key - they forgot the pairing); an index outside the list changes nothing.
void eraseBond(BondList& l, int idx);
// List format: formatBond of the bonds separated by ','. Parsing also accepts the old single-bond format; false for
// a malformed string - l unchanged.
std::string formatBonds(const BondList& l);
bool parseBonds(const std::string& s, BondList& l);

}  // namespace app::btbond
