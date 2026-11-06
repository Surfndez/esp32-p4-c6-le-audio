/*
 * cis_crypto tests: AES (FIPS-197), SK/IV/nonce and AES-CCM on the Core Vol 6 Part C 1 sample data
 * (LE encryption), a long vector from an independent implementation (Python cryptography AESCCM, 4 B tag),
 * IV CIS = IV ACL XOR AA (Zephyr ull_conn_iso.c).
 */
#include "../cis_crypto.h"
#include "aes_ref.h"
#include "test_util.h"

static void hex(const char *s, uint8_t *out, unsigned n)
{
	for (unsigned i = 0; i < n; i++) {
		unsigned v;
		sscanf(s + 2 * i, "%2x", &v);
		out[i] = (uint8_t)v;
	}
}

static void rev(const uint8_t *in, uint8_t *out, unsigned n)
{
	for (unsigned i = 0; i < n; i++)
		out[i] = in[n - 1 - i];
}

static void test_aes_fips197(void)
{
	uint8_t key[16], pt[16], want[16], got[16];
	hex("000102030405060708090a0b0c0d0e0f", key, 16);
	hex("00112233445566778899aabbccddeeff", pt, 16);
	hex("69c4e0d86a7b0430d8cdb78070b4c55a", want, 16);
	aes_ref_encrypt(key, pt, got);
	CHECK_MEM(got, want, 16);
}

/* Core Vol 6 Part C 1: LTK, SKDm, SKDs, IVm, IVs written MSO..LSO. */
static void core_sample_keys(cis_ccm_t *k)
{
	uint8_t ltk_msb[16], skdm_msb[8], skds_msb[8], ivm_msb[4], ivs_msb[4];
	uint8_t ltk[16], skdm[8], skds[8], ivm[4], ivs[4];
	hex("4C68384139F574D836BCF34E9DFB01BF", ltk_msb, 16);
	hex("ACBDCEDFE0F10213", skdm_msb, 8);
	hex("0213243546576879", skds_msb, 8);
	hex("BADCAB24", ivm_msb, 4);
	hex("DEAFBABE", ivs_msb, 4);
	/* From HCI and from the air they arrive LSO first. */
	rev(ltk_msb, ltk, 16);
	rev(skdm_msb, skdm, 8);
	rev(skds_msb, skds, 8);
	rev(ivm_msb, ivm, 4);
	rev(ivs_msb, ivs, 4);
	k->aes = aes_ref_platform;
	k->aes_ctx = NULL;
	cis_crypto_session_key(aes_ref_platform, NULL, ltk, skdm, skds, k->sk);
	cis_crypto_iv(ivm, ivs, k->iv);
}

static void test_core_sample_sk_iv_nonce(void)
{
	cis_ccm_t k;
	uint8_t want_sk[16], want_iv[8], want_nonce[13], nonce[13];
	core_sample_keys(&k);
	hex("99AD1B5226A37E3E058E3B8E27C2C666", want_sk, 16);
	CHECK_MEM(k.sk, want_sk, 16);
	/* IV = 0xDEAFBABEBADCAB24, LSO first in the nonce. */
	hex("24ABDCBABEBAAFDE", want_iv, 8);
	CHECK_MEM(k.iv, want_iv, 8);
	/* B0 from the sample data: 49 | 00 00 00 00 80 24 AB DC BA BE BA AF DE | 00 01 */
	hex("000000008024ABDCBABEBAAFDE", want_nonce, 13);
	cis_crypto_nonce(0, CIS_DIR_C_TO_P, k.iv, nonce);
	CHECK_MEM(nonce, want_nonce, 13);
	/* the counter is 39 bits: bit 39 and above must not leak into the direction bit */
	hex("FFFFFFFF7F24ABDCBABEBAAFDE", want_nonce, 13);
	cis_crypto_nonce(0xFFFFFFFFFFull, CIS_DIR_P_TO_C, k.iv, nonce);
	CHECK_MEM(nonce, want_nonce, 13);
}

static void test_core_sample_ccm(void)
{
	cis_ccm_t k;
	uint8_t out[8], want[5], plain[4];
	const uint8_t start_enc_rsp = 0x06;
	core_sample_keys(&k);

	/* Packet 0 (C->P): header 0x0F, LL_START_ENC_RSP -> 0F 05 9F CD A7 F4 48; AAD ACL = hdr & 0xE3. */
	cis_ccm_encrypt(&k, 0, CIS_DIR_C_TO_P, 0x0F & 0xE3, &start_enc_rsp, 1, out);
	hex("9FCDA7F448", want, 5);
	CHECK_MEM(out, want, 5);
	CHECK(cis_ccm_decrypt(&k, 0, CIS_DIR_C_TO_P, 0x0F & 0xE3, out, 5, plain));
	CHECK_EQ(plain[0], 0x06);

	/* Packet 1 (P->C): header 0x07 -> 07 05 A3 4C 13 A4 15. */
	cis_ccm_encrypt(&k, 0, CIS_DIR_P_TO_C, 0x07 & 0xE3, &start_enc_rsp, 1, out);
	hex("A34C13A415", want, 5);
	CHECK_MEM(out, want, 5);

	/* Bad MIC, bad direction, bad counter - all must be rejected. */
	out[4] ^= 1;
	CHECK(!cis_ccm_decrypt(&k, 0, CIS_DIR_P_TO_C, 0x03, out, 5, plain));
	CHECK_EQ(plain[0], 0);
	out[4] ^= 1;
	CHECK(cis_ccm_decrypt(&k, 0, CIS_DIR_P_TO_C, 0x03, out, 5, plain));
	CHECK(!cis_ccm_decrypt(&k, 0, CIS_DIR_C_TO_P, 0x03, out, 5, plain));
	CHECK(!cis_ccm_decrypt(&k, 1, CIS_DIR_P_TO_C, 0x03, out, 5, plain));
	CHECK(!cis_ccm_decrypt(&k, 0, CIS_DIR_P_TO_C, 0x01, out, 5, plain));
}

static void test_ccm_long_vector(void)
{
	/* Python: AESCCM(SK, tag_length=4).encrypt(78 56 34 12 AA || IV, bytes(range(40)), b"\x01") */
	cis_ccm_t k;
	uint8_t in[40], out[44], want[44], back[40];
	core_sample_keys(&k);
	for (int i = 0; i < 40; i++)
		in[i] = (uint8_t)i;
	hex("8179c31ff1679700df471d0a92bc9358b531c2900560d6586548905786017ca60eb74ec9db65775ea598511e", want, 44);
	cis_ccm_encrypt(&k, 0x2A12345678ull, CIS_DIR_C_TO_P, 0x01, in, 40, out);
	CHECK_MEM(out, want, 44);
	CHECK(cis_ccm_decrypt(&k, 0x2A12345678ull, CIS_DIR_C_TO_P, 0x01, out, 44, back));
	CHECK_MEM(back, in, 40);
	/* empty payload: MIC only */
	cis_ccm_encrypt(&k, 7, CIS_DIR_P_TO_C, 0x00, in, 0, out);
	CHECK(cis_ccm_decrypt(&k, 7, CIS_DIR_P_TO_C, 0x00, out, 4, back));
	CHECK(!cis_ccm_decrypt(&k, 7, CIS_DIR_P_TO_C, 0x00, out, 3, back));
	/* maximum CIS payload (247 + MIC = 251) */
	{
		uint8_t big[247], bigc[251], bigp[247];
		for (int i = 0; i < 247; i++)
			big[i] = (uint8_t)(i * 7 + 3);
		cis_ccm_encrypt(&k, 0x7FFFFFFFFFull, CIS_DIR_C_TO_P, 0x00, big, 247, bigc);
		CHECK(cis_ccm_decrypt(&k, 0x7FFFFFFFFFull, CIS_DIR_C_TO_P, 0x00, bigc, 251, bigp));
		CHECK_MEM(bigp, big, 247);
	}
}

static void test_cis_iv(void)
{
	uint8_t acl_iv[8], cis_iv[8], want[8];
	hex("24ABDCBABEBAAFDE", acl_iv, 8);
	/* AA 0x12345678 little-endian: 78 56 34 12 XOR the lower octets; IVs unchanged. */
	cis_crypto_cis_iv(acl_iv, 0x12345678u, cis_iv);
	want[0] = 0x24 ^ 0x78; want[1] = 0xAB ^ 0x56; want[2] = 0xDC ^ 0x34; want[3] = 0xBA ^ 0x12;
	want[4] = 0xBE; want[5] = 0xBA; want[6] = 0xAF; want[7] = 0xDE;
	CHECK_MEM(cis_iv, want, 8);
}

int main(void)
{
	test_aes_fips197();
	test_core_sample_sk_iv_nonce();
	test_core_sample_ccm();
	test_ccm_long_vector();
	test_cis_iv();
	TEST_DONE("test_crypto");
}
