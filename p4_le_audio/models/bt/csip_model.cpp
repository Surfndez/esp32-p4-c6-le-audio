// CSIP: SIRK and RSI (description in csip_model.h).

#include "csip_model.h"

#include <cstring>

namespace app::csip {

namespace {

void reverse16(const uint8_t in[16], uint8_t out[16])
{
    for (int i = 0; i < 16; i++) {
        out[i] = in[15 - i];
    }
}

// CMAC subkey: shift left by one bit, XOR Rb = 0x87 when a one is shifted out (RFC 4493 2.3)
void subkey(const uint8_t in[16], uint8_t out[16])
{
    for (int i = 0; i < 16; i++) {
        out[i] = (uint8_t)((in[i] << 1) | (i < 15 ? in[i + 1] >> 7 : 0));
    }
    if (in[0] & 0x80) {
        out[15] ^= 0x87;
    }
}

}  // namespace

bool aesCmac(const uint8_t keyBe[16], const uint8_t* msg, size_t len, uint8_t outBe[16], Aes128Fn aes)
{
    uint8_t zero[16] = {}, l[16], k1[16], k2[16];
    if (!aes(keyBe, zero, l)) {
        return false;
    }
    subkey(l, k1);
    subkey(k1, k2);
    const size_t blocks   = len == 0 ? 1 : (len + 15) / 16;
    const bool complete   = len > 0 && len % 16 == 0;
    uint8_t x[16]         = {};
    uint8_t y[16];
    for (size_t b = 0; b + 1 < blocks; b++) {
        for (int i = 0; i < 16; i++) {
            y[i] = (uint8_t)(x[i] ^ msg[b * 16 + i]);
        }
        if (!aes(keyBe, y, x)) {
            return false;
        }
    }
    uint8_t last[16]  = {};
    const size_t rest = len - (blocks - 1) * 16;
    if (rest > 0) {
        std::memcpy(last, msg + (blocks - 1) * 16, rest);
    }
    if (!complete) {
        last[rest] = 0x80;
    }
    for (int i = 0; i < 16; i++) {
        y[i] = (uint8_t)(x[i] ^ last[i] ^ (complete ? k1[i] : k2[i]));
    }
    return aes(keyBe, y, outBe);
}

bool sih(const uint8_t sirkLe[16], const uint8_t prandLe[3], uint8_t hashLe[3], Aes128Fn aes)
{
    uint8_t k[16], r[16] = {}, e[16];
    reverse16(sirkLe, k);
    // r' = padding (104 zero bits) || prand, MSB first
    r[13] = prandLe[2];
    r[14] = prandLe[1];
    r[15] = prandLe[0];
    if (!aes(k, r, e)) {
        return false;
    }
    // Lowest 24 bits of the result
    hashLe[0] = e[15];
    hashLe[1] = e[14];
    hashLe[2] = e[13];
    return true;
}

bool sdf(const uint8_t ltkLe[16], const uint8_t encSirkLe[16], uint8_t sirkLe[16], Aes128Fn aes)
{
    static const uint8_t kM[] = {'S', 'I', 'R', 'K', 'e', 'n', 'c'};
    static const uint8_t kP[] = {'c', 's', 'i', 's'};
    uint8_t zero[16] = {}, salt[16], k[16], t[16], out[16];
    reverse16(ltkLe, k);
    // s1(M) = AES-CMAC_zero(M); k1(N, SALT, P) = AES-CMAC_T(P), T = AES-CMAC_SALT(N)
    if (!aesCmac(zero, kM, sizeof(kM), salt, aes) || !aesCmac(salt, k, sizeof(k), t, aes) ||
        !aesCmac(t, kP, sizeof(kP), out, aes)) {
        return false;
    }
    for (int i = 0; i < 16; i++) {
        sirkLe[i] = (uint8_t)(out[15 - i] ^ encSirkLe[i]);
    }
    return true;
}

bool sirkFromValue(const uint8_t* v, size_t n, const uint8_t ltkLe[16], uint8_t sirkLe[16], Aes128Fn aes)
{
    if (v == nullptr || n < 17) {
        return false;
    }
    if (v[0] == 0x01) {
        std::memcpy(sirkLe, v + 1, 16);
        return true;
    }
    return v[0] == 0x00 && ltkLe != nullptr && sdf(ltkLe, v + 1, sirkLe, aes);
}

bool findRsi(const uint8_t* ad, size_t len, uint8_t rsiLe[6])
{
    size_t off = 0;
    while (ad != nullptr && off < len) {
        const size_t l = ad[off];
        if (l == 0) {
            return false;  // the rest of the data is padding
        }
        if (off + 1 + l > len) {
            return false;
        }
        if (ad[off + 1] == 0x2E && l == 7) {
            std::memcpy(rsiLe, ad + off + 2, 6);
            return true;
        }
        off += 1 + l;
    }
    return false;
}

bool rsiMatches(const uint8_t sirkLe[16], const uint8_t rsiLe[6], Aes128Fn aes)
{
    const uint8_t* prand = rsiLe + 3;
    if ((prand[2] & 0xC0) != 0x40) {
        return false;
    }
    uint8_t h[3];
    return sih(sirkLe, prand, h, aes) && std::memcmp(h, rsiLe, 3) == 0;
}

}  // namespace app::csip
