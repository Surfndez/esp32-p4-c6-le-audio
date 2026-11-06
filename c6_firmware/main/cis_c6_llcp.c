/*
 * Cyberdeck LE Audio: C6 adapter - LLCP (receiving/sending control PDUs), ACL encryption snooping, LL features.
 *
 * Object, function and offset come from analysis of the controller library. Hooks (cmake/ble_hooks.txt):
 *   r_ble_ll_ctrl_rx_pdu       - called from ble_ll_conn.c.o r_ble_ll_conn_rx_data_pdu+0x382 (result ignored)
 *   r_ble_ll_ctrl_enc_req_make - called from ble_ll_ctrl.c.o r_ble_ll_ctrl_proc_init+0x416 (same object)
 * Rule: while the engine is dormant (no CIG) both hooks only snoop; the original always makes the decision.
 */
#include <string.h>

#include "mbedtls/aes.h"
#include "os/os_mbuf.h" /* bt/porting/include/os/os_mbuf.h - layout matches the library */

#include "cis_c6.h"

/* ---------- library symbols ---------- */
/* ble_ll.c.o: +13 u8 max control PDU length (43), +28 pointer to LL data: +0 u64 local features. */
extern uint8_t *ble_ll_env_p;
extern void *r_ble_ll_conn_sm_find_active_by_handle(uint16_t handle);
/* ble_ll_mem.c.o: (dsize, user header, flags) as in proc_init (env[13], 36, 7). */
extern struct os_mbuf *r_ble_ll_mem_mbuf_get(uint16_t dsize, uint16_t user_hdr_len, uint8_t flags);
/* ble_ll_conn.c.o: (connsm, om, llid, len); om_data[0] = opcode; != 0 = error, the caller frees the mbuf. */
extern int r_ble_ll_conn_enqueue_pkt(void *connsm, struct os_mbuf *om, uint8_t llid, uint8_t len);
/* ble_ll_conn.c.o: the fragment the transmitter will cut from the PDU (min(connsm+20, PHY time limit)). */
extern uint16_t r_ble_ll_conn_adjust_pyld_len(void *connsm, uint16_t len);
extern int r_os_mbuf_free_chain(struct os_mbuf *om);

extern int orig_r_ble_ll_ctrl_rx_pdu(void *connsm, struct os_mbuf *om);
extern void orig_r_ble_ll_ctrl_enc_req_make(void *connsm, uint8_t *dptr);

int r_ble_ll_ctrl_rx_pdu(void *connsm, struct os_mbuf *om);
void r_ble_ll_ctrl_enc_req_make(void *connsm, uint8_t *dptr);

/* ---------- connsm ---------- */
#define CONNSM_FLAGS      0   /* u32; bit 28 = remote features received (r_ble_ll_ctrl_update_features) */
#define CONNSM_HANDLE     8   /* u16 */
#define CONNSM_ROLE       11  /* u8; 1 = Central */
#define CONNSM_REM_FEAT   88  /* u8[8]; [0] = AND with ours, [1..7] raw remote */
#define CONNSM_ENC_STATE  272 /* u8; 7 = after ENC_RSP */
#define CONNSM_ENC_IV     304 /* u8[8] IVm || IVs, PDU order (NOTES_cis_llcp connsm table) */
#define CONNSM_ENC_KEY    312 /* u8[16] LTK, MSB first */
#define CONNSM_ENC_PLAIN  328 /* u8[16] SKD, MSB first */
#define CONNSM_ENC_CIPHER 344 /* u8[16] SK */

#define LLID_CTRL         3
#define PDU_HDR_CP        0x20 /* ACL PDU header: CP = CTEInfo present */
#define MBUF_USER_HDR_LEN 36   /* r_ble_ll_ctrl_proc_init+0x28 */
#define MBUF_FLAGS_CTRL   7    /* r_ble_ll_ctrl_proc_init+0x26 */
#define LL_OPCODE_MAX     0x2A /* r_ble_ll_conn_enqueue_pkt+0x11e: larger -> r_ll_assert */

#define LL_UNKNOWN_RSP       0x07
#define LL_ENC_REQ           0x03
#define LL_REJECT_EXT_IND    0x11
#define LL_CIS_REQ           0x1F
#define LL_CIS_RSP           0x20
#define LL_CIS_IND           0x21
#define LL_CIS_TERMINATE_IND 0x22

#define FEAT_CIS_CENTRAL     28u
#define FEAT_ISO_HOST        32u

static inline uint16_t u16_at(const void *base, uint32_t off)
{
	const uint8_t *p = (const uint8_t *)base + off;
	return (uint16_t)(p[0] | (p[1] << 8));
}

static uint32_t *ll_feature_words(void) { return *(uint32_t **)(ble_ll_env_p + 28); }

/* ---------- LL features ---------- */
static bool s_iso_feat;

void cis_c6_set_iso_features(bool on)
{
	/* The feature word goes into LL_FEATURE_REQ/RSP and 0x2003; only bit 27 is masked (NOTES Q4). HCI Reset
	 * does not touch the word, so we clear it ourselves when snooping 0x0C03. */
	uint32_t *w = ll_feature_words();
	const uint32_t lo = 1u << FEAT_CIS_CENTRAL;
	const uint32_t hi = 1u << (FEAT_ISO_HOST - 32u);

	if (on) {
		w[0] |= lo;
		w[1] |= hi;
	} else {
		w[0] &= ~lo;
		w[1] &= ~hi;
	}
	s_iso_feat = on;
}

bool cis_c6_iso_features_on(void)
{
	return s_iso_feat;
}

/* ---------- platform operations (LL task) ---------- */

int cis_c6_send_ctrl_pdu(void *ctx, uint16_t acl_handle, const uint8_t *ctr, uint8_t len)
{
	(void)ctx;
	/* prepare_tx_pdu is not protected by a critical section - LL task only (NOTES Q2, B7). */
	if (!cis_c6_in_ll_task()) {
		CIS_C6_CNT(tx_ctrl_fail);
		return -1;
	}
	void *connsm = r_ble_ll_conn_sm_find_active_by_handle(acl_handle);
	if (connsm == NULL || len == 0 || len > ble_ll_env_p[13] || ctr[0] > LL_OPCODE_MAX) {
		CIS_C6_CNT(tx_ctrl_fail);
		return -2;
	}
	if (r_ble_ll_conn_adjust_pyld_len(connsm, len) < len) {
		/* LL_CIS_REQ 36 B > 27 B without DLE: the transmitter would cut the control PDU into LLID 3 + LLID 1. */
		CIS_C6_CNT(tx_ctrl_too_long);
		return CIS_PLAT_ERR_TOO_LONG;
	}
	struct os_mbuf *om = r_ble_ll_mem_mbuf_get(ble_ll_env_p[13], MBUF_USER_HDR_LEN, MBUF_FLAGS_CTRL);
	if (om == NULL) {
		CIS_C6_CNT(tx_ctrl_fail);
		return -3;
	}
	memcpy(om->om_data, ctr, len);
	if (r_ble_ll_conn_enqueue_pkt(connsm, om, LLID_CTRL, len) != 0) {
		r_os_mbuf_free_chain(om); /* as in proc_init .L427 */
		CIS_C6_CNT(tx_ctrl_fail);
		return -4;
	}
	CIS_C6_CNT(tx_ctrl_ok);
	return 0;
}

bool cis_c6_acl_is_central(void *ctx, uint16_t acl_handle)
{
	(void)ctx;
	const uint8_t *connsm = r_ble_ll_conn_sm_find_active_by_handle(acl_handle);
	return connsm != NULL && connsm[CONNSM_ROLE] == 1;
}

uint64_t cis_c6_acl_remote_features(void *ctx, uint16_t acl_handle)
{
	(void)ctx;
	const uint8_t *connsm = r_ble_ll_conn_sm_find_active_by_handle(acl_handle);
	uint32_t flags;

	if (connsm == NULL) {
		return 0;
	}
	memcpy(&flags, connsm + CONNSM_FLAGS, sizeof(flags));
	if ((flags & (1u << 28)) == 0) {
		return 0; /* no feature exchange yet: the engine waits for LE Read Remote Features Complete */
	}
	uint64_t f = 0;
	for (unsigned i = 0; i < 8; i++) {
		f |= (uint64_t)connsm[CONNSM_REM_FEAT + i] << (8 * i);
	}
	/* Byte 0 holds the common features (rx_feature_req.part.6+0x3c) - bits 0..7 are not CIS-related. */
	return f;
}

static uint8_t sk_order(const uint8_t *connsm)
{
	uint8_t sk[16];
	uint8_t rev[16];
	uint8_t res;

	/* Encryption state: 2 = enabled (Central after LL_START_ENC_RSP, rx_start_enc_rsp.part.16+0x38..0x44 - this state
	 * triggers Encryption Change to the host), 7 = after LL_ENC_RSP, 8 = after LL_START_ENC_REQ; 1 = none, 6 = pause.
	 * The ECB block is checked by the AES call below anyway (a zeroed key yields 2). */
	const uint8_t st = connsm == NULL ? 0 : connsm[CONNSM_ENC_STATE];
	if (st != 2 && st != 7 && st != 8) {
		return 3;
	}
	/* Same AES as the engine (contract: key and data MSB first): a match with +344 decides the byte order of the
	 * controller's ECB block without sending keys outside the C6 (NOTES Q3, risk 3). */
	cis_c6_aes128(NULL, connsm + CONNSM_ENC_KEY, connsm + CONNSM_ENC_PLAIN, sk);
	for (int i = 0; i < 16; i++) {
		rev[i] = sk[15 - i];
	}
	if (memcmp(sk, connsm + CONNSM_ENC_CIPHER, 16) == 0) {
		res = 0;
	} else if (memcmp(rev, connsm + CONNSM_ENC_CIPHER, 16) == 0) {
		res = 1;
	} else {
		res = 2;
	}
	memset(sk, 0, sizeof(sk));
	memset(rev, 0, sizeof(rev));
	return res;
}

uint8_t cis_c6_sk_check(uint16_t acl_handle)
{
	return sk_order(r_ble_ll_conn_sm_find_active_by_handle(acl_handle));
}

int cis_c6_acl_session_keys(void *ctx, uint16_t acl_handle, uint8_t sk[16], uint8_t iv[8])
{
	(void)ctx;
	const uint8_t *connsm = r_ble_ll_conn_sm_find_active_by_handle(acl_handle);
	const uint8_t order = sk_order(connsm);

	if (order > 1) {
		return -1; /* not encrypted or ECB block inconsistent - the engine keeps relying on its own snooping */
	}
	/* SK: the controller's ECB block used to encrypt ACL (rx_enc_rsp.part.2), in the engine's order (MSB first).
	 * IV: +304 = IVm || IVs in PDU order (enc_req_make+0x50..0x76, rx_enc_rsp.part.2+0x38..0x42) - as in
	 * cis_crypto.h. */
	for (int i = 0; i < 16; i++) {
		sk[i] = connsm[CONNSM_ENC_CIPHER + (order == 0 ? i : 15 - i)];
	}
	memcpy(iv, connsm + CONNSM_ENC_IV, 8);
	return 0;
}

/* ---------- AES-128 (mbedtls) ---------- */
/* Not r_ble_hw_encrypt_block: that is the radio's CCM unit, run with interrupts disabled and busy-waiting (NOTES Q3).
 * The context is shared because all callers run in the LL task; the key changes rarely (SK per CIS). */
static mbedtls_aes_context s_aes;
static uint8_t s_aes_key[16];
static bool s_aes_ready;

void cis_c6_aes128(void *ctx, const uint8_t key[16], const uint8_t in[16], uint8_t out[16])
{
	(void)ctx;
	if (!s_aes_ready || memcmp(s_aes_key, key, 16) != 0) {
		if (!s_aes_ready) {
			mbedtls_aes_init(&s_aes);
		}
		s_aes_ready = mbedtls_aes_setkey_enc(&s_aes, key, 128) == 0;
		memcpy(s_aes_key, key, 16);
	}
	if (!s_aes_ready || mbedtls_aes_crypt_ecb(&s_aes, MBEDTLS_AES_ENCRYPT, in, out) != 0) {
		/* A bad block causes a MIC error on both sides - the engine ends the CIS; no silent "good" result. */
		CIS_C6_CNT(aes_fail);
		memset(out, 0, 16);
	}
}

/* ---------- RX of control PDUs ---------- */

/* Defence against an engine bug (lesson from v3/B3): only CIS opcodes with the exact length may be consumed. */
static bool llcp_may_consume(const uint8_t *ctr, uint8_t len)
{
	switch (ctr[0]) {
	case LL_CIS_REQ:           return len == 36;
	case LL_CIS_RSP:           return len == 9;
	case LL_CIS_IND:           return len == 16;
	case LL_CIS_TERMINATE_IND: return len == 4;
	case LL_REJECT_EXT_IND:    return len == 3 && ctr[1] >= LL_CIS_REQ && ctr[1] <= LL_CIS_TERMINATE_IND;
	case LL_UNKNOWN_RSP:       return len == 2 && ctr[1] >= LL_CIS_REQ && ctr[1] <= LL_CIS_TERMINATE_IND;
	default:                   return false;
	}
}

/* Control PDU trace of a regular connection (since 1.4.64): the left Buds3 Pro does not finish encryption start with
 * a stored key (silence, 0x08), the right one does - without a sniffer only this shows which LL_ENC_x/LL_REJECT_x pass.
 * Trace ring entry (kind outside cis_trace_kind_t): code = opcode, v0 = length, v1 = up to 4 CtrData bytes. LL task. */
static void acl_ctrl_trace(uint8_t kind, uint16_t handle, const uint8_t *ctr, uint8_t len)
{
	cis_trace_t t = { .kind = kind, .code = ctr[0], .acl = handle };
	t.v[0] = len;
	for (uint8_t i = 1; i < len && i <= 4; i++) {
		t.v[1] |= (uint32_t)ctr[i] << (8u * (i - 1u));
	}
	cis_c6_trace(NULL, &t);
}

int r_ble_ll_ctrl_rx_pdu(void *connsm, struct os_mbuf *om)
{
	cis_engine_t *e = cis_c6_engine_ll();

	if (e == NULL) {
		/* Adapter disabled or not the LL task: plan B (copy + hop) described in NOTES Q1, counter on G3. */
		if (cis_c6_on() && !cis_c6_in_ll_task()) {
			CIS_C6_CNT(rx_ctrl_wrong_ctx);
		}
		return orig_r_ble_ll_ctrl_rx_pdu(connsm, om);
	}
	/* om_data[0] = header (LLID, NESN, SN, MD, CP), [1] = length (opcode + CtrData, without MIC), opcode in [2],
	 * or in [3] with CP (CTEInfo in [2]). */
	const uint8_t *d = om->om_data;
	const uint8_t off = (d[0] & PDU_HDR_CP) ? 3 : 2;
	const uint8_t len = d[1];
	/* UNVERIFIED: om_len includes the header. If wrong, everything goes to the original (CIS stays untouched). */
	if (len == 0 || (uint32_t)off + len > om->om_len) {
		return orig_r_ble_ll_ctrl_rx_pdu(connsm, om);
	}
	acl_ctrl_trace(CIS_C6_TR_ACL_CTRL_RX, u16_at(connsm, CONNSM_HANDLE), d + off, len);
	const bool active = cis_engine_active(e);
	const bool eaten = cis_engine_ll_ctrl_rx(e, u16_at(connsm, CONNSM_HANDLE), d + off, len); /* + ENC_RSP snooping */

	if (!eaten) {
		return orig_r_ble_ll_ctrl_rx_pdu(connsm, om);
	}
	if (!active || !llcp_may_consume(d + off, len)) {
		/* A dormant engine consumes nothing (v4 behaviour: LL_UNKNOWN_RSP from the controller). */
		CIS_C6_CNT(rx_ctrl_veto);
		return orig_r_ble_ll_ctrl_rx_pdu(connsm, om);
	}
	/* Consumed: no LL_UNKNOWN_RSP (.L848/.L907). The mbuf is ours (the original frees it itself in .L919/.L955),
	 * the caller ignores the result (rx_data_pdu+0x38a). We skip the connsm+0 bit 30 flag (delayed DLE, .L952) -
	 * it runs on the next control PDU handed to the original. */
	r_os_mbuf_free_chain(om);
	CIS_C6_CNT(rx_ctrl_consumed);
	cis_c6_kick(CIS_C6_KICK_PROCESS);
	return 0;
}

/* ---------- TX: snooping LL_ENC_REQ (SKDm, IVm) ---------- */

void r_ble_ll_ctrl_enc_req_make(void *connsm, uint8_t *dptr)
{
	orig_r_ble_ll_ctrl_enc_req_make(connsm, dptr);
	cis_engine_t *e = cis_c6_engine_ll();
	if (e != NULL) {
		const uint8_t op = LL_ENC_REQ;
		acl_ctrl_trace(CIS_C6_TR_ACL_CTRL_TX, u16_at(connsm, CONNSM_HANDLE), &op, 1);
		/* dptr = CtrData without opcode: Rand[0..7] EDIV[8..9] SKDm[10..17] IVm[18..21] (+296, +274, +336, +304). */
		uint8_t pdu[23];
		pdu[0] = LL_ENC_REQ;
		memcpy(&pdu[1], dptr, 22);
		cis_engine_ll_ctrl_tx_seen(e, u16_at(connsm, CONNSM_HANDLE), pdu, sizeof(pdu));
		memset(pdu, 0, sizeof(pdu));
		CIS_C6_CNT(enc_req_seen);
	}
}
