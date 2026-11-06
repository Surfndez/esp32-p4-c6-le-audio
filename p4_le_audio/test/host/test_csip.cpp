// Host tests: CSIP (bt/csip_model.cpp) - SIRK from the CSIS characteristic and RSI of the set's other earbud.
// CSIS A.1/A.2 sample data (as in BlueZ unit/test-crypto.c), CMAC from RFC 4493. AES reference from CIS engine tests
// (le_audio/cis_ll/test/aes_ref.c). Separate program.

#include <bt/csip_model.h>

#include <cstdio>
#include <cstring>

extern "C" void aes_ref_encrypt(const uint8_t key[16], const uint8_t in[16], uint8_t out[16]);

namespace {

int g_ok = 0, g_fail = 0;

void expect(bool cond, const char* what)
{
    if (cond) {
        g_ok++;
    } else {
        g_fail++;
        std::printf("  FAIL %s\n", what);
    }
}

bool aes(const uint8_t keyBe[16], const uint8_t inBe[16], uint8_t outBe[16])
{
    aes_ref_encrypt(keyBe, inBe, outBe);
    return true;
}

bool aesFails(const uint8_t[16], const uint8_t[16], uint8_t[16])
{
    return false;
}

// CSIS A.1/A.2: SIRK 457d7d09 21a1fd22 cecd8c86 dd72cccd, K d9cee53c 22c61e06 6f6948d4 9b1b6e67 (MSB first:
// 676e1b9b ...), prand 69f563 -> sih 1948da, sef -> 170a3835 e13524a0 7e2562d5 f25fd346. Here LSB first.
const uint8_t kSirk[16] = {0xcd, 0xcc, 0x72, 0xdd, 0x86, 0x8c, 0xcd, 0xce,
                           0x22, 0xfd, 0xa1, 0x21, 0x09, 0x7d, 0x7d, 0x45};
const uint8_t kK[16]    = {0xd9, 0xce, 0xe5, 0x3c, 0x22, 0xc6, 0x1e, 0x06,
                           0x6f, 0x69, 0x48, 0xd4, 0x9b, 0x1b, 0x6e, 0x67};
const uint8_t kEnc[16]  = {0x46, 0xd3, 0x5f, 0xf2, 0xd5, 0x62, 0x25, 0x7e,
                           0xa0, 0x24, 0x35, 0xe1, 0x35, 0x38, 0x0a, 0x17};
const uint8_t kRsi[6]   = {0xda, 0x48, 0x19, 0x63, 0xf5, 0x69};  // hash || prand

void testCmac()
{
    using app::csip::aesCmac;
    // RFC 4493 4: K = 2b7e1516 28aed2a6 abf71588 09cf4f3c
    const uint8_t key[16]  = {0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6,
                              0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf, 0x4f, 0x3c};
    const uint8_t exp0[16] = {0xbb, 0x1d, 0x69, 0x29, 0xe9, 0x59, 0x37, 0x28,
                              0x7f, 0xa3, 0x7d, 0x12, 0x9b, 0x75, 0x67, 0x46};
    const uint8_t m16[16]  = {0x6b, 0xc1, 0xbe, 0xe2, 0x2e, 0x40, 0x9f, 0x96,
                              0xe9, 0x3d, 0x7e, 0x11, 0x73, 0x93, 0x17, 0x2a};
    const uint8_t exp16[16] = {0x07, 0x0a, 0x16, 0xb4, 0x6b, 0x4d, 0x41, 0x44,
                               0xf7, 0x9b, 0xdd, 0x9d, 0xd0, 0x4a, 0x28, 0x7c};
    uint8_t out[16];
    expect(aesCmac(key, nullptr, 0, out, aes) && std::memcmp(out, exp0, 16) == 0, "CMAC RFC 4493 example 1 (empty)");
    expect(aesCmac(key, m16, 16, out, aes) && std::memcmp(out, exp16, 16) == 0, "CMAC RFC 4493 example 2 (16 B)");
    expect(!aesCmac(key, m16, 16, out, aesFails), "AES failure = CMAC failure");
}

void testSirk()
{
    using namespace app::csip;
    uint8_t out[16];
    expect(sdf(kK, kSirk, out, aes) && std::memcmp(out, kEnc, 16) == 0, "sef from CSIS sample data");
    expect(sdf(kK, kEnc, out, aes) && std::memcmp(out, kSirk, 16) == 0, "sdf reverses sef (same key)");

    uint8_t v[17];
    v[0] = 0x00;
    std::memcpy(v + 1, kEnc, 16);
    std::memset(out, 0, sizeof(out));
    expect(sirkFromValue(v, sizeof(v), kK, out, aes) && std::memcmp(out, kSirk, 16) == 0,
           "SIRK type 0 decrypted with the link key");
    v[0] = 0x01;
    std::memcpy(v + 1, kSirk, 16);
    expect(sirkFromValue(v, sizeof(v), nullptr, out, aes) && std::memcmp(out, kSirk, 16) == 0, "SIRK type 1 plain");
    v[0] = 0x00;
    expect(!sirkFromValue(v, sizeof(v), nullptr, out, aes), "type 0 without link key - error");
    v[0] = 0x02;
    expect(!sirkFromValue(v, sizeof(v), kK, out, aes), "unknown type - error");
    expect(!sirkFromValue(v, 16, kK, out, aes), "value too short - error");
    v[0] = 0x00;
    expect(!sirkFromValue(v, sizeof(v), kK, out, aesFails), "AES failure - error");
}

void testRsi()
{
    using namespace app::csip;
    uint8_t h[3];
    const uint8_t prand[3] = {0x63, 0xf5, 0x69};
    expect(sih(kSirk, prand, h, aes) && h[0] == 0xda && h[1] == 0x48 && h[2] == 0x19, "sih from CSIS sample data");
    expect(rsiMatches(kSirk, kRsi, aes), "sample RSI belongs to the set");
    uint8_t other[6];
    std::memcpy(other, kRsi, 6);
    other[0] ^= 0x01;
    expect(!rsiMatches(kSirk, other, aes), "modified hash - foreign set");
    uint8_t sirk2[16];
    std::memcpy(sirk2, kSirk, 16);
    sirk2[5] ^= 0x10;
    expect(!rsiMatches(sirk2, kRsi, aes), "different SIRK - foreign set");
    std::memcpy(other, kRsi, 6);
    other[5] = (uint8_t)(other[5] | 0x80);  // prand 11xxxxxx - not an RSI (CSIP 4.7)
    expect(!rsiMatches(kSirk, other, aes), "prand without bits 01 - rejected");
    expect(!rsiMatches(kSirk, kRsi, aesFails), "AES failure - no match");

    // Advertisement: Flags, RSI, name
    const uint8_t ad[] = {0x02, 0x01, 0x06, 0x07, 0x2E, 0xda, 0x48, 0x19, 0x63, 0xf5, 0x69, 0x03, 0x09, 'B', 'u'};
    uint8_t rsi[6]     = {};
    expect(findRsi(ad, sizeof(ad), rsi) && std::memcmp(rsi, kRsi, 6) == 0, "RSI from advertisement data");
    expect(!findRsi(ad, 6, rsi), "truncated RSI structure - none");
    const uint8_t noRsi[] = {0x02, 0x01, 0x06, 0x00, 0x07, 0x2E, 1, 2, 3, 4, 5, 6};
    expect(!findRsi(noRsi, sizeof(noRsi), rsi), "only padding after a zero length");
    const uint8_t badLen[] = {0x05, 0x2E, 1, 2, 3, 4};
    expect(!findRsi(badLen, sizeof(badLen), rsi), "RSI with wrong length - skipped");
    expect(!findRsi(nullptr, 0, rsi), "no data");
}

}  // namespace

int main()
{
    std::printf("CSIP: SIRK and RSI\n");
    testCmac();
    testSirk();
    testRsi();
    std::printf("CSIP: %d OK, %d FAIL\n", g_ok, g_fail);
    return g_fail == 0 ? 0 : 1;
}
