/*
 * Cyberdeck LE Audio: LE encryption for CIS - AES-CCM (4 B MIC) on an injected AES-128, session key and IV.
 *
 * Byte-order conventions (the most common source of mistakes, hence spelled out):
 * - values from the air and from HCI (LTK, SKDm/SKDs, IVm/IVs) arrive LSO first (little-endian),
 * - the Core e() function and the AES block take key and data MSO first, so SK = e(LTK, SKD) is computed on
 *   reversed arrays, and the resulting SK is directly the AES key for CCM (MSO first),
 * - SKD = SKDs || SKDm (SKDm is the lower half), IV = IVs || IVm (IVm is the lower 4 octets); in the array
 *   iv[8] we keep the IV LSO first: iv[0..3] = IVm, iv[4..7] = IVs - the same order as it goes into the nonce.
 * - nonce (13 B) = 39-bit counter LSO first, direction bit in bit 7 of octet 4, then iv[0..7]
 *   (Core Vol 6 Part E 2.1; sample data Vol 6 Part C 1: B0 = 49 | 00 00 00 00 80 | 24 AB DC BA ...).
 * CIS (Zephyr ull_conn_iso.c, Core Vol 6 Part E 2.2): key = SK of the ACL connection, CIS IV = ACL IV with the
 * lower 4 octets XORed with the CIS access address (little-endian), counter = cisPayloadCounter,
 * direction: C->P = 1, P->C = 0.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*cis_aes128_fn)(void *ctx, const uint8_t key[16], const uint8_t in[16], uint8_t out[16]);

typedef struct {
	cis_aes128_fn aes;
	void   *aes_ctx;
	uint8_t sk[16];      /* MSO first (AES key) */
	uint8_t iv[8];       /* LSO first (as in the nonce) */
} cis_ccm_t;

#define CIS_DIR_P_TO_C 0
#define CIS_DIR_C_TO_P 1

/* SK = e(LTK, SKDs || SKDm); inputs LSO first (as from HCI / the air), result MSO first. */
void cis_crypto_session_key(cis_aes128_fn aes, void *ctx, const uint8_t ltk_le[16], const uint8_t skdm_le[8],
		const uint8_t skds_le[8], uint8_t sk[16]);
void cis_crypto_iv(const uint8_t ivm_le[4], const uint8_t ivs_le[4], uint8_t iv[8]);
void cis_crypto_cis_iv(const uint8_t acl_iv[8], uint32_t cis_aa, uint8_t cis_iv[8]);
void cis_crypto_nonce(uint64_t counter, uint8_t direction, const uint8_t iv[8], uint8_t nonce[13]);

/* out = ciphertext (len) + MIC (4). aad = first header octet, already masked. len <= 251 - 4. */
void cis_ccm_encrypt(const cis_ccm_t *k, uint64_t counter, uint8_t direction, uint8_t aad,
		const uint8_t *in, uint8_t len, uint8_t *out);
/* in = ciphertext + MIC, len_with_mic >= 4; out = plaintext (len_with_mic - 4). false = bad MIC
 * (out is then zeroed so nobody uses unverified data). */
bool cis_ccm_decrypt(const cis_ccm_t *k, uint64_t counter, uint8_t direction, uint8_t aad,
		const uint8_t *in, uint8_t len_with_mic, uint8_t *out);

#ifdef __cplusplus
}
#endif
