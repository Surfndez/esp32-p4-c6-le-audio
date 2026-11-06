// Host tests: BLE bond (bt/bt_bond_model.cpp) - recognising paired earbuds after the private address changes
// and bond serialisation. Reference AES from the CIS engine tests (le_audio/cis_ll/test/aes_ref.c). Separate program.

#include <bt/bt_bond_model.h>

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

// Core Vol 3 Part H, ah sample data: IRK ec0234a3 57c8ad05 341010a6 0a397d9b, prand 708194 -> ah 0dfbaa.
// In SMP and HCI the values go LSB first.
const uint8_t kIrk[16] = {0x9b, 0x7d, 0x39, 0x0a, 0xa6, 0x10, 0x10, 0x34,
                          0x05, 0xad, 0xc8, 0x57, 0xa3, 0x34, 0x02, 0xec};
const uint8_t kRpa[6]  = {0xaa, 0xfb, 0x0d, 0x94, 0x81, 0x70};  // 70:81:94:0D:FB:AA = prand || hash

}  // namespace

int main()
{
    using namespace app::btbond;
    std::printf("bond BLE: private address and storage\n");

    expect(isResolvablePrivate(1, kRpa), "RPA (random, bits 01) recognised");
    const uint8_t staticRnd[6] = {0x11, 0x22, 0x33, 0x44, 0x55, 0xC6};
    expect(!isResolvablePrivate(1, staticRnd), "static random (bits 11) is not an RPA");
    expect(!isResolvablePrivate(0, kRpa), "public address is not an RPA, even with bits 01");

    expect(rpaMatches(kIrk, kRpa, aes), "ah from the Core sample data");
    uint8_t other[6];
    std::memcpy(other, kRpa, 6);
    other[0] ^= 0x01;
    expect(!rpaMatches(kIrk, other, aes), "modified hash does not match");
    uint8_t irk2[16];
    std::memcpy(irk2, kIrk, 16);
    irk2[0] ^= 0x80;
    expect(!rpaMatches(irk2, kRpa, aes), "different IRK does not match");
    expect(!rpaMatches(kIrk, kRpa, aesFails), "AES failure = no match");

    // Bond with IRK: recognised by identity address and by RPA; without IRK only by the address from pairing
    Bond b;
    b.valid = true;
    const uint8_t idAddr[6] = {0x87, 0xDC, 0x67, 0x8A, 0x31, 0x3C};
    std::memcpy(b.addr, idAddr, 6);
    for (int i = 0; i < 16; i++) {
        b.ltk[i] = (uint8_t)(0xA0 + i);
    }
    b.hasIrk = true;
    std::memcpy(b.irk, kIrk, 16);
    expect(bondMatches(b, 0, idAddr, aes), "bond under the identity address");
    expect(bondMatches(b, 1, kRpa, aes), "bond under a new RPA (resolved via IRK)");
    expect(!bondMatches(b, 1, other, aes), "foreign RPA - not our bond");
    Bond noIrk = b;
    noIrk.hasIrk = false;
    expect(!bondMatches(noIrk, 1, kRpa, aes), "without IRK an RPA cannot be recognised");
    Bond empty;
    expect(!bondMatches(empty, 0, idAddr, aes), "empty bond matches nothing");

    // Serialise and parse
    const std::string s = formatBond(b);
    expect(s.size() == 2 * (6 + 16 + 16), "serialised length with IRK");
    Bond r;
    expect(parseBond(s, r) && r.valid && r.hasIrk && std::memcmp(r.addr, b.addr, 6) == 0 &&
               std::memcmp(r.ltk, b.ltk, 16) == 0 && std::memcmp(r.irk, b.irk, 16) == 0,
           "parsing a serialised bond with IRK");
    const std::string legacy = s.size() >= 2 * (6 + 16) ? s.substr(0, 2 * (6 + 16)) : std::string(2 * (6 + 16), '0');
    Bond l;
    expect(parseBond(legacy, l) && l.valid && !l.hasIrk && std::memcmp(l.ltk, b.ltk, 16) == 0,
           "older format (address + LTK) without IRK");
    Bond keep = b;
    expect(!parseBond("zz" + legacy.substr(2), keep) && keep.valid && keep.hasIrk, "bad characters - bond unchanged");
    expect(!parseBond(legacy.substr(1), keep), "wrong length rejected");
    expect(formatBond(noIrk).size() == 2 * (6 + 16), "without IRK serialised as before");

    // Multiple bonds: each earbud is a separate device (own identity address, IRK and LTK)
    BondList list;
    expect(bondCount(list) == 0 && findBond(list, 1, kRpa, aes) == -1, "empty list recognises nothing");
    Bond right = b;
    right.addr[0] ^= 0x55;
    right.irk[0] ^= 0x80;
    right.ltk[0] = 0x11;
    putBond(list, b);
    putBond(list, right);
    expect(bondCount(list) == 2, "two earbuds - two bonds");
    expect(findBond(list, 1, kRpa, aes) == 1, "left recognised by RPA despite the newer right one");
    expect(findBond(list, 0, right.addr, aes) == 0, "right recognised by identity address");
    expect(findBond(list, 1, other, aes) == -1, "foreign RPA - unknown");

    Bond again = b;
    again.ltk[0] = 0x77;
    putBond(list, again);
    expect(bondCount(list) == 2 && findBond(list, 1, kRpa, aes) == 0 && list.items[0].ltk[0] == 0x77,
           "re-pairing the same earbud replaces its entry and moves it to the front");
    Bond sameIrk = again;
    sameIrk.addr[1] ^= 0x01;  // different pairing address (no Identity Address), same IRK
    putBond(list, sameIrk);
    expect(bondCount(list) == 2, "same IRK = same device");

    for (int i = 0; i < 4; i++) {
        Bond n = right;
        n.addr[2] = (uint8_t)(0xE0 + i);
        n.irk[2]  = (uint8_t)(0xE0 + i);
        putBond(list, n);
    }
    expect(bondCount(list) == kMaxBonds, "full list does not grow");
    expect(findBond(list, 1, kRpa, aes) == -1, "the earliest paired one drops out");

    // Removing a bond the earbuds no longer know (key rejected)
    BondList drop;
    putBond(drop, b);
    putBond(drop, right);
    eraseBond(drop, 0);
    expect(bondCount(drop) == 1 && findBond(drop, 1, kRpa, aes) == 0 && findBond(drop, 0, right.addr, aes) == -1,
           "removed entry disappears, the rest shifts to the front");
    eraseBond(drop, 5);
    eraseBond(drop, -1);
    expect(bondCount(drop) == 1, "an index outside the list removes nothing");

    // Serialise and parse a list (including the old single-bond format)
    BondList two;
    putBond(two, b);
    putBond(two, right);
    BondList back;
    expect(parseBonds(formatBonds(two), back) && bondCount(back) == 2 &&
               std::memcmp(back.items[0].ltk, right.ltk, 16) == 0 && std::memcmp(back.items[1].irk, b.irk, 16) == 0,
           "serialise and parse two bonds");
    BondList old;
    expect(parseBonds(s, old) && bondCount(old) == 1 && findBond(old, 1, kRpa, aes) == 0,
           "old single-bond format parsed as a list");
    BondList keepList = two;
    expect(!parseBonds(formatBonds(two) + ",zz", keepList) && bondCount(keepList) == 2,
           "bad entry - list unchanged");
    expect(!parseBonds("", keepList), "empty input rejected");

    std::printf("bond BLE: %d OK, %d FAIL\n", g_ok, g_fail);
    return g_fail == 0 ? 0 : 1;
}
