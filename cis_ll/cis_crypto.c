/*
 * Cyberdeck LE Audio: AES-CCM for LE (RFC 3610, M = 4, L = 2) and CIS keys - see cis_crypto.h.
 * Written from scratch per Core Vol 6 Part E; conformance is checked by tests on the Core Vol 6 Part C 1 sample data.
 */
#include "cis_crypto.h"

#include <string.h>

void cis_crypto_session_key(cis_aes128_fn aes, void *ctx, const uint8_t ltk_le[16], const uint8_t skdm_le[8],
		const uint8_t skds_le[8], uint8_t sk[16])
{
	uint8_t key[16], skd[16];
	/* e() takes MSO first: SKD = SKDs || SKDm, so the most significant octets are the reversed SKDs. */
	for (int i = 0; i < 16; i++)
		key[i] = ltk_le[15 - i];
	for (int i = 0; i < 8; i++) {
		skd[i] = skds_le[7 - i];
		skd[8 + i] = skdm_le[7 - i];
	}
	aes(ctx, key, skd, sk);
	memset(key, 0, sizeof(key));
}

void cis_crypto_iv(const uint8_t ivm_le[4], const uint8_t ivs_le[4], uint8_t iv[8])
{
	memcpy(iv, ivm_le, 4);
	memcpy(iv + 4, ivs_le, 4);
}

void cis_crypto_cis_iv(const uint8_t acl_iv[8], uint32_t cis_aa, uint8_t cis_iv[8])
{
	/* Zephyr ull_conn_iso.c: mem_xor_32(iv, acl_iv, access_addr) - AA in over-the-air order (LE). */
	for (int i = 0; i < 4; i++)
		cis_iv[i] = (uint8_t)(acl_iv[i] ^ (uint8_t)(cis_aa >> (8 * i)));
	memcpy(cis_iv + 4, acl_iv + 4, 4);
}

void cis_crypto_nonce(uint64_t counter, uint8_t direction, const uint8_t iv[8], uint8_t nonce[13])
{
	for (int i = 0; i < 4; i++)
		nonce[i] = (uint8_t)(counter >> (8 * i));
	/* the counter is 39 bits; bit 7 of octet 4 is the direction */
	nonce[4] = (uint8_t)(((counter >> 32) & 0x7F) | (direction ? 0x80 : 0));
	memcpy(nonce + 5, iv, 8);
}

static void ccm_block(const cis_ccm_t *k, uint8_t flags, const uint8_t nonce[13], uint16_t tail, uint8_t blk[16])
{
	blk[0] = flags;
	memcpy(blk + 1, nonce, 13);
	blk[14] = (uint8_t)(tail >> 8);
	blk[15] = (uint8_t)tail;
	(void)k;
}

/* CBC-MAC over B0 (length), B1 (AAD: 1 octet) and the data blocks; returns T (first 4 octets of X_n). */
static void ccm_mac(const cis_ccm_t *k, const uint8_t nonce[13], uint8_t aad, const uint8_t *data, uint8_t len,
		uint8_t t[4])
{
	uint8_t b[16], x[16];
	/* B0 flags: Adata = 1 (0x40), M' = (4 - 2) / 2 = 1 (0x08), L' = 2 - 1 = 1 */
	ccm_block(k, 0x49, nonce, len, b);
	k->aes(k->aes_ctx, k->sk, b, x);
	memset(b, 0, sizeof(b));
	b[1] = 1;      /* AAD length = 1, encoded in 2 octets */
	b[2] = aad;
	for (int i = 0; i < 16; i++)
		b[i] ^= x[i];
	k->aes(k->aes_ctx, k->sk, b, x);
	for (uint16_t off = 0; off < len; off = (uint16_t)(off + 16)) {
		uint8_t n = (uint8_t)(len - off < 16 ? len - off : 16);
		for (uint8_t i = 0; i < n; i++)
			x[i] ^= data[off + i];
		k->aes(k->aes_ctx, k->sk, x, x);
	}
	memcpy(t, x, 4);
}

/* CTR stream: A_i = 01 | nonce | i; S_0 masks the MIC, S_1.. the data. */
static void ccm_ctr(const cis_ccm_t *k, const uint8_t nonce[13], const uint8_t *in, uint8_t len, uint8_t *out,
		uint8_t s0[4])
{
	uint8_t a[16], s[16];
	ccm_block(k, 0x01, nonce, 0, a);
	k->aes(k->aes_ctx, k->sk, a, s);
	memcpy(s0, s, 4);
	uint16_t i = 1;
	for (uint16_t off = 0; off < len; off = (uint16_t)(off + 16), i++) {
		uint8_t n = (uint8_t)(len - off < 16 ? len - off : 16);
		ccm_block(k, 0x01, nonce, i, a);
		k->aes(k->aes_ctx, k->sk, a, s);
		for (uint8_t j = 0; j < n; j++)
			out[off + j] = (uint8_t)(in[off + j] ^ s[j]);
	}
}

void cis_ccm_encrypt(const cis_ccm_t *k, uint64_t counter, uint8_t direction, uint8_t aad,
		const uint8_t *in, uint8_t len, uint8_t *out)
{
	uint8_t nonce[13], t[4], s0[4];
	cis_crypto_nonce(counter, direction, k->iv, nonce);
	/* MAC before CTR: also works for in == out */
	ccm_mac(k, nonce, aad, in, len, t);
	ccm_ctr(k, nonce, in, len, out, s0);
	for (int i = 0; i < 4; i++)
		out[len + i] = (uint8_t)(t[i] ^ s0[i]);
}

bool cis_ccm_decrypt(const cis_ccm_t *k, uint64_t counter, uint8_t direction, uint8_t aad,
		const uint8_t *in, uint8_t len_with_mic, uint8_t *out)
{
	if (len_with_mic < 4)
		return false;
	uint8_t len = (uint8_t)(len_with_mic - 4);
	uint8_t nonce[13], t[4], s0[4], mic[4];
	memcpy(mic, in + len, 4);
	cis_crypto_nonce(counter, direction, k->iv, nonce);
	ccm_ctr(k, nonce, in, len, out, s0);
	ccm_mac(k, nonce, aad, out, len, t);
	uint8_t diff = 0;
	for (int i = 0; i < 4; i++)
		diff |= (uint8_t)(mic[i] ^ t[i] ^ s0[i]);
	if (diff) {
		memset(out, 0, len);
		return false;
	}
	return true;
}
