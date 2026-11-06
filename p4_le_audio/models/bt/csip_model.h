#pragma once
// CSIP (earbud set), pure part: the set key (SIRK) from the CSIS characteristic and recognising the second member
// of the set by the RSI in its advertisement. No HAL - host tests (test/host/test_csip.cpp).
//
// Sources: CSIS 1.0.1 ch. 4 (functions sih, sef/sdf, s1, k1; SIRK value: type 0 = encrypted with link key, 1 = plain),
// CSIP 1.0.1 4.7 (RSI: hash 24 bit || prand 24 bit, AD type 0x2E). CSIS A.1/A.2 sample data as in BlueZ
// unit/test-crypto.c (test_sef, test_sih) and Zephyr subsys/bluetooth/audio/csip_crypto.c. Arrays are LSB first (as in
// HCI and on air), AES via Aes128Fn with key and block MSB first (e() from Core Vol 3 Part H 2.2.1).

#include <bt/bt_bond_model.h>

#include <cstddef>
#include <cstdint>

namespace app::csip {

using app::btbond::Aes128Fn;

// AES-CMAC (RFC 4493), key, message and result MSB first - as in the Bluetooth specifications.
bool aesCmac(const uint8_t keyBe[16], const uint8_t* msg, size_t len, uint8_t outBe[16], Aes128Fn aes);

// sih(SIRK, prand) = e(SIRK, padding || prand) mod 2^24 (CSIS 4.7). false on an AES error.
bool sih(const uint8_t sirkLe[16], const uint8_t prandLe[3], uint8_t hashLe[3], Aes128Fn aes);

// sdf(K, EncSIRK) = k1(K, s1("SIRKenc"), "csis") XOR EncSIRK (CSIS 4.6), K = the LTK of the link with the earbud.
// Symmetric: sef (encryption) gives the same.
bool sdf(const uint8_t ltkLe[16], const uint8_t encSirkLe[16], uint8_t sirkLe[16], Aes128Fn aes);

// Set Identity Resolving Key value (0x2B84): type (1 B) + 16 B. Type 0 is encrypted with the link key, 1 is plain.
// false: bad length, unknown type or an AES error.
bool sirkFromValue(const uint8_t* v, size_t n, const uint8_t ltkLe[16], uint8_t sirkLe[16], Aes128Fn aes);

// RSI (AD type 0x2E, 6 B) from advertising data (length-type-data structures). false = missing or malformed.
bool findRsi(const uint8_t* ad, size_t len, uint8_t rsiLe[6]);

// The RSI belongs to the set with this SIRK: prand with the top two bits 01 (CSIP 4.7) and hash == sih(SIRK, prand).
bool rsiMatches(const uint8_t sirkLe[16], const uint8_t rsiLe[6], Aes128Fn aes);

}  // namespace app::csip
