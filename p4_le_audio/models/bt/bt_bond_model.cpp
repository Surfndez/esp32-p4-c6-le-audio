// BLE bond, pure part (bt_bond_model.h).
#include "bt_bond_model.h"

#include <algorithm>
#include <cstdio>
#include <cstring>

namespace app::btbond {

namespace {

constexpr size_t kAddrLtkHex = 2 * (6 + 16);
constexpr size_t kWithIrkHex = 2 * (6 + 16 + 16);

void putHex(std::string& s, const uint8_t* p, size_t n)
{
    char b[3];
    for (size_t i = 0; i < n; i++) {
        std::snprintf(b, sizeof(b), "%02x", p[i]);
        s += b;
    }
}

bool getHex(const std::string& s, size_t at, uint8_t* out, size_t n)
{
    for (size_t i = 0; i < n; i++) {
        unsigned v = 0;
        for (int k = 0; k < 2; k++) {
            const char c = s[at + 2 * i + k];
            const int d = c >= '0' && c <= '9' ? c - '0' : c >= 'a' && c <= 'f' ? c - 'a' + 10
                        : c >= 'A' && c <= 'F' ? c - 'A' + 10 : -1;
            if (d < 0) {
                return false;
            }
            v = v * 16 + (unsigned)d;
        }
        out[i] = (uint8_t)v;
    }
    return true;
}

}  // namespace

bool isResolvablePrivate(uint8_t addrType, const uint8_t addr[6])
{
    return (addrType & 0x01) != 0 && (addr[5] & 0xC0) == 0x40;
}

bool rpaMatches(const uint8_t irk[16], const uint8_t addr[6], Aes128Fn aes)
{
    // ah(k, r) = e(k, r') mod 2^24, r' = 104 zero bits || prand (MSB first); the IRK is LSB first in SMP.
    uint8_t keyBe[16];
    for (int i = 0; i < 16; i++) {
        keyBe[i] = irk[15 - i];
    }
    uint8_t in[16] = {};
    in[13] = addr[5];
    in[14] = addr[4];
    in[15] = addr[3];
    uint8_t out[16] = {};
    if (!aes(keyBe, in, out)) {
        return false;
    }
    return out[15] == addr[0] && out[14] == addr[1] && out[13] == addr[2];
}

bool bondMatches(const Bond& b, uint8_t addrType, const uint8_t addr[6], Aes128Fn aes)
{
    if (!b.valid) {
        return false;
    }
    if (std::memcmp(b.addr, addr, 6) == 0) {
        return true;
    }
    return b.hasIrk && isResolvablePrivate(addrType, addr) && rpaMatches(b.irk, addr, aes);
}

std::string formatBond(const Bond& b)
{
    std::string s;
    s.reserve(kWithIrkHex);
    putHex(s, b.addr, 6);
    putHex(s, b.ltk, 16);
    if (b.hasIrk) {
        putHex(s, b.irk, 16);
    }
    return s;
}

bool parseBond(const std::string& s, Bond& b)
{
    if (s.size() != kAddrLtkHex && s.size() != kWithIrkHex) {
        return false;
    }
    Bond r;
    if (!getHex(s, 0, r.addr, 6) || !getHex(s, 12, r.ltk, 16)) {
        return false;
    }
    if (s.size() == kWithIrkHex) {
        if (!getHex(s, kAddrLtkHex, r.irk, 16)) {
            return false;
        }
        r.hasIrk = true;
    }
    r.valid = true;
    b       = r;
    return true;
}

size_t bondCount(const BondList& l)
{
    size_t n = 0;
    while (n < kMaxBonds && l.items[n].valid) {
        n++;
    }
    return n;
}

int findBond(const BondList& l, uint8_t addrType, const uint8_t addr[6], Aes128Fn aes)
{
    for (size_t i = 0; i < kMaxBonds; i++) {
        if (bondMatches(l.items[i], addrType, addr, aes)) {
            return (int)i;
        }
    }
    return -1;
}

void putBond(BondList& l, const Bond& b)
{
    // Same device: same address or same IRK (without an Identity Address the bond keeps the pairing address, which
    // may be a different RPA at the next pairing)
    auto same = [&b](const Bond& o) {
        return std::memcmp(o.addr, b.addr, 6) == 0 || (o.hasIrk && b.hasIrk && std::memcmp(o.irk, b.irk, 16) == 0);
    };
    BondList out;
    out.items[0] = b;
    size_t n     = 1;
    for (size_t i = 0; i < kMaxBonds && n < kMaxBonds; i++) {
        if (l.items[i].valid && !same(l.items[i])) {
            out.items[n++] = l.items[i];
        }
    }
    l = out;
}

void eraseBond(BondList& l, int idx)
{
    if (idx < 0 || idx >= (int)kMaxBonds) {
        return;
    }
    for (size_t i = (size_t)idx; i + 1 < kMaxBonds; i++) {
        l.items[i] = l.items[i + 1];
    }
    l.items[kMaxBonds - 1] = Bond{};
}

std::string formatBonds(const BondList& l)
{
    std::string s;
    for (size_t i = 0; i < bondCount(l); i++) {
        if (i > 0) {
            s += ',';
        }
        s += formatBond(l.items[i]);
    }
    return s;
}

bool parseBonds(const std::string& s, BondList& l)
{
    BondList r;
    size_t n = 0, at = 0;
    while (at <= s.size()) {
        const size_t end = std::min(s.find(',', at), s.size());
        if (n == kMaxBonds || !parseBond(s.substr(at, end - at), r.items[n])) {
            return false;
        }
        n++;
        at = end + 1;
    }
    l = r;
    return true;
}

}  // namespace app::btbond
