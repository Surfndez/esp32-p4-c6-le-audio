/*
 * Unit tests of the simulator's own primitives on vectors from the standards:
 * FIPS-197 C.1 (AES-128), Core Vol 6 Part C 1 (LE encryption sample data: SK, LL_START_ENC_RSP),
 * Core Vol 6 Part C 3 (CSA#2, including subevents), AA rules, air times.
 */
#include "sim.h"

#include <string.h>

static int s_fail;

static void hex(const uint8_t *b, unsigned n, char *out)
{
	for (unsigned i = 0; i < n; i++)
		sprintf(out + 2 * i, "%02x", b[i]);
}

static void check_bytes(const char *name, const uint8_t *got, const uint8_t *exp, unsigned n)
{
	if (memcmp(got, exp, n) == 0) {
		printf("  ok   %s\n", name);
		return;
	}
	char g[2 * 64 + 1], e[2 * 64 + 1];
	hex(got, n, g);
	hex(exp, n, e);
	printf("  FAIL %s\n       got %s\n       exp %s\n", name, g, e);
	s_fail++;
}

static void check_int(const char *name, long got, long exp)
{
	if (got == exp) {
		printf("  ok   %s = %ld\n", name, got);
		return;
	}
	printf("  FAIL %s: got %ld exp %ld\n", name, got, exp);
	s_fail++;
}

static void reverse(uint8_t *d, const uint8_t *s, unsigned n)
{
	for (unsigned i = 0; i < n; i++)
		d[i] = s[n - 1 - i];
}

int selftest_units(void)
{
	s_fail = 0;
	printf("unit: AES-128 / CCM / SK (FIPS-197, Core Vol 6 Part C 1)\n");
	{
		const uint8_t key[16] = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b,
			0x0c, 0x0d, 0x0e, 0x0f };
		const uint8_t pt[16] = { 0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb,
			0xcc, 0xdd, 0xee, 0xff };
		const uint8_t ct[16] = { 0x69, 0xc4, 0xe0, 0xd8, 0x6a, 0x7b, 0x04, 0x30, 0xd8, 0xcd, 0xb7, 0x80,
			0x70, 0xb4, 0xc5, 0x5a };
		uint8_t out[16];
		aes128_encrypt(key, pt, out);
		check_bytes("FIPS-197 C.1 AES-128", out, ct, 16);
	}
	/* Core: LTK, SKDm/SKDs, IVm/IVs (MSO first in the specification), SK. */
	const uint8_t ltk_be[16] = { 0x4C, 0x68, 0x38, 0x41, 0x39, 0xF5, 0x74, 0xD8, 0x36, 0xBC, 0xF3, 0x4E,
		0x9D, 0xFB, 0x01, 0xBF };
	const uint8_t skdm_be[8] = { 0xAC, 0xBD, 0xCE, 0xDF, 0xE0, 0xF1, 0x02, 0x13 };
	const uint8_t skds_be[8] = { 0x02, 0x13, 0x24, 0x35, 0x46, 0x57, 0x68, 0x79 };
	const uint8_t ivm_be[4] = { 0xBA, 0xDC, 0xAB, 0x24 };
	const uint8_t ivs_be[4] = { 0xDE, 0xAF, 0xBA, 0xBE };
	const uint8_t sk_exp[16] = { 0x99, 0xAD, 0x1B, 0x52, 0x26, 0xA3, 0x7E, 0x3E, 0x05, 0x8E, 0x3B, 0x8E,
		0x27, 0xC2, 0xC6, 0x66 };
	uint8_t ltk_le[16], skdc_le[8], skdp_le[8], iv_le[8], sk[16];
	reverse(ltk_le, ltk_be, 16);
	reverse(skdc_le, skdm_be, 8);
	reverse(skdp_le, skds_be, 8);
	reverse(iv_le, ivm_be, 4);
	reverse(iv_le + 4, ivs_be, 4);
	ble_session_key(ltk_le, skdc_le, skdp_le, sk);
	check_bytes("Core SK = e(LTK, SKDs||SKDm)", sk, sk_exp, 16);
	{
		/* LL_START_ENC_RSP1 (C->P, counter 0): header 0x0F, payload 0x06 -> 0F 05 9F CD A7 F4 48 */
		uint8_t nonce[13], out[5], dec[1];
		const uint8_t pl = 0x06;
		const uint8_t exp1[5] = { 0x9F, 0xCD, 0xA7, 0xF4, 0x48 };
		ble_nonce(nonce, 0, true, iv_le);
		ble_ccm_encrypt(sk, nonce, 0x0F & CCM_AAD_MASK, &pl, 1, out);
		check_bytes("Core LL_START_ENC_RSP1 C->P ciphertext+MIC", out, exp1, 5);
		bool ok = ble_ccm_decrypt(sk, nonce, 0x0F & CCM_AAD_MASK, out, 5, dec);
		check_int("CCM decrypt round trip", ok && dec[0] == 0x06, 1);
		/* LL_START_ENC_RSP2 (P->C, counter 0): header 0x07 -> 07 05 A3 4C 13 A4 15 */
		const uint8_t exp2[5] = { 0xA3, 0x4C, 0x13, 0xA4, 0x15 };
		ble_nonce(nonce, 0, false, iv_le);
		ble_ccm_encrypt(sk, nonce, 0x07 & CCM_AAD_MASK, &pl, 1, out);
		check_bytes("Core LL_START_ENC_RSP2 P->C ciphertext+MIC", out, exp2, 5);
		out[0] ^= 1;
		check_int("CCM detects corrupted payload", ble_ccm_decrypt(sk, nonce, 0x07, out, 5, dec), 0);
	}

	printf("unit: CSA#2 (Core Vol 6 Part C 3)\n");
	{
		const uint8_t all[5] = { 0xFF, 0xFF, 0xFF, 0xFF, 0x1F };
		const uint8_t nine[5] = { 0x00, 0x06, 0xE0, 0x00, 0x1E };
		const uint32_t aa = 0x8E89BED6;
		check_int("chanId", (long)((aa >> 16) ^ (aa & 0xFFFF)), 0x305F);
		check_int("37 ch, counter 1", csa2_chan(aa, all, 1, 0), 20);
		check_int("37 ch, counter 2", csa2_chan(aa, all, 2, 0), 6);
		check_int("37 ch, counter 3", csa2_chan(aa, all, 3, 0), 21);
		check_int("9 ch, counter 6", csa2_chan(aa, nine, 6, 0), 23);
		check_int("9 ch, counter 7", csa2_chan(aa, nine, 7, 0), 9);
		check_int("9 ch, counter 8", csa2_chan(aa, nine, 8, 0), 34);
		/* Subevents (Core 5.2+, ISO sample data): event 0, 37 channels: 25, 1, 16, 36. */
		check_int("37 ch, counter 0, subevent 0", csa2_chan(aa, all, 0, 0), 25);
		check_int("37 ch, counter 0, subevent 1", csa2_chan(aa, all, 0, 1), 1);
		check_int("37 ch, counter 0, subevent 2", csa2_chan(aa, all, 0, 2), 16);
		check_int("37 ch, counter 0, subevent 3", csa2_chan(aa, all, 0, 3), 36);
		/* Property: subevent channels are always from the map and differ from the previous one. */
		int bad = 0;
		for (uint16_t ctr = 0; ctr < 2000; ctr++) {
			csa2_t c;
			csa2_init(&c, 0x12345678 ^ ctr * 0x01000193u, nine);
			uint8_t prev = csa2_event(&c, ctr);
			for (int se = 1; se < 8; se++) {
				uint8_t ch = csa2_subevent(&c);
				if (ch == prev || !(nine[ch / 8] & (1u << (ch % 8))))
					bad++;
				prev = ch;
			}
		}
		check_int("subevent channel in map and != previous (14000 cases)", bad, 0);
	}

	printf("unit: access address rules, air time\n");
	check_int("AA adv rejected", aa_check(0x8E89BED6) != NULL, 1);
	check_int("AA one-bit-off rejected", aa_check(0x8E89BED7) != NULL, 1);
	check_int("AA 7 zeros rejected", aa_check(0x9A3B0005u) != NULL, 1);
	check_int("AA equal octets rejected", aa_check(0x5A5A5A5Au) != NULL, 1);
	check_int("AA 0x9A3B2C1D accepted", aa_check(0x9A3B2C1Du) == NULL, 1);
	check_int("air 1M 27B", air_time_us(CIS_PHY_1M, 27), 296);
	check_int("air 2M 124B", air_time_us(CIS_PHY_2M, 124), 540);
	return s_fail;
}
