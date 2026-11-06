/*
 * CIS Central engine tests on a fake platform and a scripted Peripheral (only as much logic as needed
 * to check bytes and flows - the full Peripheral and the simulator live in cis_ll/sim).
 * Flows: encryption snoop -> Set CIG -> Create CIS -> LL_CIS_REQ (bytes) -> LL_CIS_RSP -> LL_CIS_IND (bytes)
 * -> link_open -> CIS Established (bytes) -> SDU -> encrypted PDU (decrypted with the same SK) -> acknowledgement
 * -> NOCP; PDU from the Peripheral -> HCI ISO; underrun; flush; disconnection; rejections, timeouts,
 * ACL data length, remote features, ACL loss.
 */
#include "../cis_crypto.h"
#include "../cis_engine.h"
#include "../cis_proto.h"
#include "aes_ref.h"
#include "test_util.h"

#include <stdlib.h>

/* ---------------- fake platform ---------------- */
#define MAX_HOST 256
typedef struct {
	uint8_t pkt[MAX_HOST][600];
	uint16_t len[MAX_HOST];
	int n;
	uint8_t ctrl[32][64];
	uint8_t ctrl_len[32];
	uint16_t ctrl_acl[32];
	int n_ctrl;
	int ctrl_ret;
	int ctrl_ret_count;          /* how many consecutive calls return ctrl_ret */
	cis_acl_timing_t timing;
	cis_acl_timing_t timing2;    /* ACL 2, when timing2_on (a different phase than ACL 1) */
	bool timing2_on;
	bool central;
	uint64_t features;
	cis_link_params_t lp;
	const cis_link_cb_t *cb;
	void *cb_arg;
	const cis_link_cb_t *cbs[4];  /* in link_open order (two CISes at once) */
	void *cb_args[4];
	int n_open;
	int n_close;
	cis_radio_caps_t caps;
	uint32_t now;
	uint32_t timer_delay;
	int n_timer;
	uint64_t rng;
	char link_obj[4];
	cis_trace_t tr[64];
	int n_tr;
} fake_t;

static fake_t F;

static void f_hci(void *ctx, const uint8_t *h4, uint16_t len)
{
	(void)ctx;
	if (F.n >= MAX_HOST || len > sizeof(F.pkt[0])) {
		fprintf(stderr, "fake: host overflow\n");
		abort();
	}
	memcpy(F.pkt[F.n], h4, len);
	F.len[F.n++] = len;
}

static int f_ctrl(void *ctx, uint16_t acl, const uint8_t *ctr, uint8_t len)
{
	(void)ctx;
	if (F.ctrl_ret_count > 0) {
		F.ctrl_ret_count--;
		return F.ctrl_ret;
	}
	memcpy(F.ctrl[F.n_ctrl], ctr, len);
	F.ctrl_len[F.n_ctrl] = len;
	F.ctrl_acl[F.n_ctrl++] = acl;
	return 0;
}

static int f_timing(void *ctx, uint16_t acl, cis_acl_timing_t *out)
{
	(void)ctx;
	if (acl != 0x0001 && acl != 0x0002)
		return -1;
	*out = (acl == 0x0002 && F.timing2_on) ? F.timing2 : F.timing;
	return 0;
}

static bool f_central(void *ctx, uint16_t acl) { (void)ctx; (void)acl; return F.central; }
static uint64_t f_feat(void *ctx, uint16_t acl) { (void)ctx; (void)acl; return F.features; }

static int f_open(void *ctx, const cis_link_params_t *p, const cis_link_cb_t *cb, void *arg, cis_link_t **out)
{
	(void)ctx;
	F.lp = *p;
	F.cb = cb;
	F.cb_arg = arg;
	if (F.n_open < 4) {
		F.cbs[F.n_open] = cb;
		F.cb_args[F.n_open] = arg;
	}
	F.n_open++;
	*out = (cis_link_t *)(void *)F.link_obj;
	return 0;
}

static void f_close(void *ctx, cis_link_t *l) { (void)ctx; (void)l; F.n_close++; F.cb = NULL; }
static void f_caps(void *ctx, cis_radio_caps_t *out) { (void)ctx; *out = F.caps; }
static uint32_t f_now(void *ctx) { (void)ctx; return F.now; }
static void f_rand(void *ctx, uint8_t *buf, uint16_t len)
{
	(void)ctx;
	for (uint16_t i = 0; i < len; i++) {
		F.rng ^= F.rng << 13;
		F.rng ^= F.rng >> 7;
		F.rng ^= F.rng << 17;
		buf[i] = (uint8_t)(F.rng >> 32);
	}
}
static void f_timer(void *ctx, uint32_t d) { (void)ctx; F.timer_delay = d; F.n_timer++; }
static void f_log(void *ctx, int level, const char *msg) { (void)ctx; (void)level; (void)msg; }
/* Keys with which the controller encrypts the ACL (acl_session_keys); outside test_controller_keys the platform does
 * not supply them. */
static struct {
	bool on;
	uint8_t sk[16];
	uint8_t iv[8];
} g_ctl;

static int f_keys(void *ctx, uint16_t acl, uint8_t sk[16], uint8_t iv[8])
{
	(void)ctx;
	if (!g_ctl.on || acl != 0x0001)
		return -1;
	memcpy(sk, g_ctl.sk, 16);
	memcpy(iv, g_ctl.iv, 8);
	return 0;
}

static void f_trace(void *ctx, const cis_trace_t *t)
{
	(void)ctx;
	if (F.n_tr < (int)(sizeof(F.tr) / sizeof(F.tr[0])))
		F.tr[F.n_tr++] = *t;
}

static const cis_platform_ops_t OPS = {
	.ctx = NULL, .hci_to_host = f_hci, .send_ctrl_pdu = f_ctrl, .acl_timing = f_timing,
	.acl_is_central = f_central, .acl_remote_features = f_feat, .link_open = f_open, .link_close = f_close,
	.radio_caps = f_caps, .now_us = f_now, .aes128 = aes_ref_platform, .random = f_rand,
	.timer_start = f_timer, .log = f_log, .acl_session_keys = f_keys, .trace = f_trace,
};

static uint64_t g_mem[(64 * 1024) / 8];

/* Lower bound of the jitter buffer for fresh() (test_jitter_floor_from_radio overrides it temporarily). */
static uint8_t g_jitter_floor = 0;
/* TX queue slots from the radio for fresh() (test_tx_slots_from_radio). */
static uint8_t g_tx_slots = 0;

static cis_engine_t *fresh(void)
{
	memset(&F, 0, sizeof(F));
	F.central = true;
	F.features = (1ull << CIS_FEAT_BIT_CIS_PERIPHERAL) | 1ull;
	F.timing = (cis_acl_timing_t){ .event_counter = 100, .anchor_us = 1000000, .interval_us = 50000,
		.crc_init = 0x123456, .chan_map = { 0xFF, 0xFF, 0xFF, 0xFF, 0x1F }, .phy_tx = CIS_PHY_2M,
		.phy_rx = CIS_PHY_2M, .peripheral_latency = 0 };
	F.caps = (cis_radio_caps_t){ .max_nse = 1, .radio_owns_seq = false, .tx_prepare_lead_us = 500,
				     .jitter_min_sdu = g_jitter_floor, .tx_slots = g_tx_slots };
	F.now = 900000;
	F.rng = 0x1234567890ABCDEFull;
	if (cis_engine_mem_size() > sizeof(g_mem)) {
		fprintf(stderr, "mem_size %u > test buffer\n", (unsigned)cis_engine_mem_size());
		abort();
	}
	return cis_engine_init(g_mem, (uint32_t)sizeof(g_mem), &OPS);
}

/* ---------------- HCI helpers ---------------- */
static bool cmd(cis_engine_t *e, uint16_t op, const uint8_t *p, uint8_t plen)
{
	uint8_t b[260];
	b[0] = 0x01;
	cis_put_le16(&b[1], op);
	b[3] = plen;
	if (plen)
		memcpy(&b[4], p, plen);
	return cis_engine_hci_cmd(e, b, (uint16_t)(4 + plen));
}

static bool evt(cis_engine_t *e, const uint8_t *p, uint8_t plen, uint8_t code)
{
	uint8_t b[260];
	b[0] = 0x04;
	b[1] = code;
	b[2] = plen;
	memcpy(&b[3], p, plen);
	return cis_engine_hci_evt_from_controller(e, b, (uint16_t)(3 + plen));
}

/* the last packet to the host of a given kind: an event code (0x3E = subevent sub) or ISO (code 0) */
static const uint8_t *last_evt(uint8_t code, uint8_t sub, uint16_t *len)
{
	for (int i = F.n - 1; i >= 0; i--) {
		const uint8_t *p = F.pkt[i];
		if (code == 0 && p[0] == 0x05) {
			if (len)
				*len = F.len[i];
			return p;
		}
		if (p[0] == 0x04 && p[1] == code && (code != 0x3E || p[3] == sub)) {
			if (len)
				*len = F.len[i];
			return p;
		}
	}
	return NULL;
}

static int count_evt(uint8_t code)
{
	int n = 0;
	for (int i = 0; i < F.n; i++)
		n += F.pkt[i][0] == 0x04 && F.pkt[i][1] == code;
	return n;
}

/* ---------------- encryption data (Core Vol 6 Part C 1) ---------------- */
static const uint8_t LTK_MSB[16] = { 0x4C, 0x68, 0x38, 0x41, 0x39, 0xF5, 0x74, 0xD8, 0x36, 0xBC, 0xF3, 0x4E,
	0x9D, 0xFB, 0x01, 0xBF };
static const uint8_t SKDM_MSB[8] = { 0xAC, 0xBD, 0xCE, 0xDF, 0xE0, 0xF1, 0x02, 0x13 };
static const uint8_t SKDS_MSB[8] = { 0x02, 0x13, 0x24, 0x35, 0x46, 0x57, 0x68, 0x79 };
static const uint8_t IVM_MSB[4] = { 0xBA, 0xDC, 0xAB, 0x24 };
static const uint8_t IVS_MSB[4] = { 0xDE, 0xAF, 0xBA, 0xBE };

/* What the engine saw of the ACL key exchange (test_controller_keys); the other tests - everything. */
typedef enum { SNOOP_FULL, SNOOP_NONE, SNOOP_BAD_LTK } snoop_t;
static snoop_t g_snoop = SNOOP_FULL;

static void encrypt_acl(cis_engine_t *e, uint16_t acl)
{
	uint8_t p[28], req[23], rsp[13];
	uint8_t ec[4] = { 0x00, (uint8_t)acl, (uint8_t)(acl >> 8), 0x01 };
	if (g_snoop == SNOOP_NONE) {
		CHECK(!evt(e, ec, 4, CIS_EVT_ENC_CHANGE));
		return;
	}
	cis_put_le16(p, acl);
	memset(p + 2, 0x11, 10);
	for (int i = 0; i < 16; i++)
		p[12 + i] = LTK_MSB[15 - i];
	if (g_snoop == SNOOP_BAD_LTK)
		p[12] ^= 0x01;
	CHECK(!cmd(e, CIS_OP_LE_ENABLE_ENC, p, 28));    /* snoop only */
	req[0] = CIS_LL_ENC_REQ;
	memset(req + 1, 0x22, 10);
	for (int i = 0; i < 8; i++)
		req[11 + i] = SKDM_MSB[7 - i];
	for (int i = 0; i < 4; i++)
		req[19 + i] = IVM_MSB[3 - i];
	cis_engine_ll_ctrl_tx_seen(e, acl, req, 23);
	rsp[0] = CIS_LL_ENC_RSP;
	for (int i = 0; i < 8; i++)
		rsp[1 + i] = SKDS_MSB[7 - i];
	for (int i = 0; i < 4; i++)
		rsp[9 + i] = IVS_MSB[3 - i];
	CHECK(!cis_engine_ll_ctrl_rx(e, acl, rsp, 13));
	CHECK(!evt(e, ec, 4, CIS_EVT_ENC_CHANGE));
}

/* CIS keys computed independently by the test: SK from the Core data, ACL IV XOR AA */
static void cis_keys(uint32_t aa, cis_ccm_t *k)
{
	static const uint8_t sk[16] = { 0x99, 0xAD, 0x1B, 0x52, 0x26, 0xA3, 0x7E, 0x3E, 0x05, 0x8E, 0x3B, 0x8E, 0x27,
		0xC2, 0xC6, 0x66 };
	static const uint8_t acl_iv[8] = { 0x24, 0xAB, 0xDC, 0xBA, 0xBE, 0xBA, 0xAF, 0xDE };
	k->aes = aes_ref_platform;
	k->aes_ctx = NULL;
	memcpy(k->sk, sk, 16);
	cis_crypto_cis_iv(acl_iv, aa, k->iv);
}

/* Set CIG: 1 CIS (id 0), SDU 10 ms, 120/40 B, 2M, RTN 0, latency 10 ms */
static const uint8_t CIG1[24] = {
	0x01, 0x10, 0x27, 0x00, 0x10, 0x27, 0x00, 0x00, 0x00, 0x00, 0x0A, 0x00, 0x0A, 0x00, 0x01,
	0x00, 0x78, 0x00, 0x28, 0x00, 0x02, 0x02, 0x00, 0x00,
};

static void host_feature(cis_engine_t *e)
{
	static const uint8_t hf[2] = { 32, 1 };
	CHECK(cmd(e, CIS_OP_LE_SET_HOST_FEATURE, hf, 2));
}

/* Set CIG parameters for set_cig/establish; test_jitter_from_latency overrides them temporarily. */
static const uint8_t *g_cig = CIG1;

static void set_cig(cis_engine_t *e)
{
	CHECK(cmd(e, CIS_OP_LE_SET_CIG_PARAMS, g_cig, sizeof(CIG1)));
	static const uint8_t want[11] = { 0x04, 0x0E, 0x08, 0x01, 0x62, 0x20, 0x00, 0x01, 0x01, 0x00, 0x0E };
	uint16_t l = 0;
	const uint8_t *p = last_evt(0x0E, 0, &l);
	CHECK(p != NULL);
	if (p) {
		CHECK_EQ(l, 11);
		CHECK_MEM(p, want, 11);
	}
	CHECK(cis_engine_active(e));
}

static bool create_cis(cis_engine_t *e, uint16_t cis, uint16_t acl)
{
	uint8_t p[5] = { 1, (uint8_t)cis, (uint8_t)(cis >> 8), (uint8_t)acl, (uint8_t)(acl >> 8) };
	return cmd(e, CIS_OP_LE_CREATE_CIS, p, 5);
}

static uint8_t last_cmd_status(void)
{
	const uint8_t *p = last_evt(0x0F, 0, NULL);
	return p ? p[3] : 0xFF;
}

static void rsp_from_peer(cis_engine_t *e, uint32_t mn, uint32_t mx, uint16_t cnt)
{
	cis_ll_cis_rsp_t r = { .cis_offset_min_us = mn, .cis_offset_max_us = mx, .conn_event_count = cnt };
	uint8_t b[CIS_LL_CIS_RSP_LEN];
	cis_ll_cis_rsp_pack(&r, b);
	CHECK(cis_engine_ll_ctrl_rx(e, 0x0001, b, sizeof(b)));
}

/* ---------------- radio and scripted Peripheral (NSE = BN = FT = 1) ---------------- */
static cis_pdu_t g_tx;

static bool r_prep(uint32_t ev)
{
	if (!F.cb)
		return false;
	return F.cb->prepare_tx(F.cb_arg, ev, 0, &g_tx);
}

static void r_rx(uint32_t ev, const cis_pdu_t *p, bool crc)
{
	if (F.cb)
		F.cb->rx_done(F.cb_arg, ev, 0, p, crc, F.now);
}

static void r_close(uint32_t ev)
{
	if (F.cb)
		F.cb->event_closed(F.cb_arg, ev);
}

/* Peripheral at FT = 1: SN and NESN toggle once per payload (event), so in event ev both = ev & 1,
 * and the response NESN is additionally toggled when it accepted our PDU. */
static cis_pdu_t peer_null(uint32_t ev, bool accepted)
{
	cis_pdu_t p;
	memset(&p, 0, sizeof(p));
	bool nesn = ((ev & 1) != 0) ^ accepted;
	p.hdr0 = (uint8_t)(CIS_LLID_START_CONT | CIS_HDR_NPI | (nesn ? CIS_HDR_NESN : 0));
	return p;
}

static cis_pdu_t peer_data(const cis_ccm_t *k, uint32_t ev, const uint8_t *d, uint8_t len)
{
	cis_pdu_t p;
	memset(&p, 0, sizeof(p));
	p.hdr0 = (uint8_t)(CIS_LLID_COMPLETE_END | ((ev & 1) ? (CIS_HDR_SN | CIS_HDR_NESN) : 0));
	cis_ccm_encrypt(k, ev, CIS_DIR_P_TO_C, (uint8_t)(p.hdr0 & CIS_CCM_AAD_MASK), d, len, p.payload);
	p.len = (uint8_t)(len + 4);
	return p;
}

/* an event without data from either side */
static void idle_event(cis_engine_t *e, uint32_t ev)
{
	CHECK(r_prep(ev));
	cis_pdu_t n = peer_null(ev, false);
	r_rx(ev, &n, true);
	r_close(ev);
	cis_engine_process(e);
}

/* ---------------- tests ---------------- */
static void test_init_and_snoop(void)
{
	CHECK(cis_engine_mem_size() > 0);
	CHECK(cis_engine_init(g_mem, 16, &OPS) == NULL);
	cis_engine_t *e = fresh();
	CHECK(e != NULL);
	if (!e)
		return;
	CHECK(!cis_engine_active(e));

	/* 0x2003: bit 28 (CIS Central) added, the event goes to the host from the engine */
	static const uint8_t feat[12] = { 0x01, 0x03, 0x20, 0x00, 0xFF, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
	CHECK(evt(e, feat, 12, CIS_EVT_CMD_COMPLETE));
	const uint8_t *p = last_evt(0x0E, 0, NULL);
	CHECK(p != NULL);
	if (p) {
		CHECK_EQ(p[7], 0xFF);
		CHECK_EQ(p[10], 0x10);
		CHECK_EQ(p[8], 0x01);
	}

	/* 0x2002 snooped, then 0x2060 builds the ACL fields from it */
	static const uint8_t rbs[7] = { 0x01, 0x02, 0x20, 0x00, 0xFB, 0x00, 0x0C };
	CHECK(!evt(e, rbs, 7, CIS_EVT_CMD_COMPLETE));
	CHECK(cmd(e, CIS_OP_LE_READ_BUF_SIZE_V2, NULL, 0));
	p = last_evt(0x0E, 0, NULL);
	CHECK(p != NULL);
	if (p) {
		CHECK_EQ(cis_get_le16(&p[4]), 0x2060);
		CHECK_EQ(p[6], 0);
		CHECK_EQ(cis_get_le16(&p[7]), 251);
		CHECK_EQ(p[9], 12);
		CHECK(cis_get_le16(&p[10]) >= 8 + 120);
		/* host ISO buffers: a credit per payload queue slot (2 CIS x CIS_TX_SLOTS 12) - with 20 the host dropped
		 * to 0 free and dropped SDUs (Tab5, 2026-11-01, drop_no_credit) */
		CHECK_EQ(p[12], 24);
	}

	/* Set Host Feature: bit 32 is ours, the others go to the controller */
	static const uint8_t hf_other[2] = { 38, 1 };
	CHECK(!cmd(e, CIS_OP_LE_SET_HOST_FEATURE, hf_other, 2));
	host_feature(e);
	p = last_evt(0x0E, 0, NULL);
	CHECK(p && cis_get_le16(&p[4]) == 0x2074 && p[6] == 0);

	/* foreign commands pass through */
	static const uint8_t z[1] = { 0 };
	CHECK(!cmd(e, 0x2005, z, 1));
	/* Create CIS without a CIG: unknown handle */
	create_cis(e, 0x0E00, 0x0001);
	CHECK_EQ(last_cmd_status(), CIS_ERR_UNKNOWN_CONN_ID);
}

typedef struct {
	cis_engine_t *e;
	uint32_t aa;
	uint32_t anchor0;
	cis_ccm_t k;
} est_t;

/* Full setup with encryption; returns the state for further tests. */
static bool establish(est_t *s)
{
	cis_engine_t *e = fresh();
	s->e = e;
	if (!e)
		return false;
	encrypt_acl(e, 0x0001);
	host_feature(e);
	set_cig(e);
	CHECK(create_cis(e, 0x0E00, 0x0001));
	CHECK_EQ(last_cmd_status(), 0);

	/* LL_CIS_REQ: offset min = 1000 (500 + ACL guard), max = ISO - CIG_Sync_Delay (1060) - ACL guard (1000: the C6
	 * starts the schedule element of the next ACL event before its anchor) = 7940 - ACL 50 ms is 5 x ISO, so
	 * the CIG must fit in one ISO interval after the ACL so that no CIS event hits the ACL; instant +6 */
	CHECK_EQ(F.n_ctrl, 1);
	static const uint8_t want_req[36] = {
		0x1F, 0x01, 0x00, 0x02, 0x02, 0x78, 0x00, 0x28, 0x00, 0x10, 0x27, 0x00, 0x10, 0x27, 0x00,
		0x78, 0x00, 0x28, 0x00, 0x01, 0x00, 0x00, 0x00, 0x11, 0x01, 0x01, 0x08, 0x00,
		0xE8, 0x03, 0x00, 0x04, 0x1F, 0x00, 0x6A, 0x00,
	};
	CHECK_EQ(F.ctrl_len[0], 36);
	CHECK_MEM(F.ctrl[0], want_req, 36);
	CHECK_EQ(F.ctrl_acl[0], 0x0001);

	rsp_from_peer(e, 1000, 48940, 106);
	CHECK_EQ(F.n_ctrl, 2);
	cis_ll_cis_ind_t ind;
	memset(&ind, 0, sizeof(ind));
	CHECK(cis_ll_cis_ind_parse(F.ctrl[1], F.ctrl_len[1], &ind));
	CHECK(cis_aa_valid(ind.access_address));
	CHECK_EQ(ind.cis_offset_us, 1000);
	CHECK_EQ(ind.cig_sync_delay_us, 1060);
	CHECK_EQ(ind.cis_sync_delay_us, 1060);
	CHECK_EQ(ind.conn_event_count, 106);
	{
		uint8_t want_ind[16];
		cis_ll_cis_ind_pack(&ind, want_ind);
		CHECK_EQ(F.ctrl[1][0], 0x21);
		CHECK_MEM(F.ctrl[1], want_ind, 16);
	}
	s->aa = ind.access_address;

	CHECK_EQ(F.n_open, 1);
	CHECK_EQ(F.lp.access_address, ind.access_address);
	CHECK_EQ(F.lp.crc_init, 0x123456);
	CHECK_EQ(F.lp.phy_c_to_p, CIS_PHY_2M);
	CHECK_EQ(F.lp.nse, 1);
	CHECK_EQ(F.lp.iso_interval_us, 10000);
	CHECK_EQ(F.lp.first_anchor_us, 1000000u + 6u * 50000u + 1000u);
	CHECK_EQ(F.lp.max_pdu_c_to_p, 124);   /* with MIC */
	CHECK_EQ(F.lp.max_pdu_p_to_c, 44);
	CHECK_EQ(F.lp.chan_map[4], 0x1F);
	s->anchor0 = F.lp.first_anchor_us;
	cis_keys(s->aa, &s->k);

	/* event 0: null PDU, Peripheral response -> CIS Established */
	CHECK(r_prep(0));
	CHECK_EQ(g_tx.len, 0);
	CHECK_EQ(g_tx.hdr0, CIS_LLID_START_CONT | CIS_HDR_NPI);
	CHECK(last_evt(0x3E, 0x19, NULL) == NULL);
	cis_pdu_t n = peer_null(0, false);
	r_rx(0, &n, true);
	r_close(0);
	cis_engine_process(e);
	static const uint8_t want_est[32] = { 0x04, 0x3E, 0x1D, 0x19, 0x00, 0x00, 0x0E, 0x24, 0x04, 0x00, 0x24, 0x04,
		0x00, 0x24, 0x04, 0x00, 0x24, 0x04, 0x00, 0x02, 0x02, 0x01, 0x01, 0x01, 0x01, 0x01, 0x78, 0x00, 0x28,
		0x00, 0x08, 0x00 };
	uint16_t l = 0;
	const uint8_t *p = last_evt(0x3E, 0x19, &l);
	CHECK(p != NULL);
	if (!p)
		return false;
	CHECK_EQ(l, 32);
	CHECK_MEM(p, want_est, 32);
	return true;
}

static void setup_paths(cis_engine_t *e)
{
	uint8_t p[13] = { 0x00, 0x0E, 0x00, 0x00, 0x03, 0, 0, 0, 0, 0, 0, 0, 0 };
	CHECK(cmd(e, CIS_OP_LE_SETUP_ISO_PATH, p, 13));
	static const uint8_t want[9] = { 0x04, 0x0E, 0x06, 0x01, 0x6E, 0x20, 0x00, 0x00, 0x0E };
	const uint8_t *q = last_evt(0x0E, 0, NULL);
	CHECK(q && memcmp(q, want, 9) == 0);
	p[2] = 1;
	CHECK(cmd(e, CIS_OP_LE_SETUP_ISO_PATH, p, 13));
	q = last_evt(0x0E, 0, NULL);
	CHECK(q && q[6] == 0);
	/* the same path a second time: Command Disallowed */
	CHECK(cmd(e, CIS_OP_LE_SETUP_ISO_PATH, p, 13));
	q = last_evt(0x0E, 0, NULL);
	CHECK(q && q[6] == CIS_ERR_CMD_DISALLOWED);
}

static void send_sdu_to(cis_engine_t *e, uint16_t handle, uint16_t psn, const uint8_t *d, uint16_t len)
{
	uint8_t b[600];
	b[0] = 0x05;
	cis_put_le16(&b[1], (uint16_t)(handle | (CIS_ISO_PB_COMPLETE << 12)));
	cis_put_le16(&b[3], (uint16_t)(4 + len));
	cis_put_le16(&b[5], psn);
	cis_put_le16(&b[7], len);
	memcpy(&b[9], d, len);
	cis_engine_hci_iso(e, b, (uint16_t)(9 + len));
}

static void send_sdu(cis_engine_t *e, uint16_t psn, const uint8_t *d, uint16_t len)
{
	send_sdu_to(e, 0x0E00, psn, d, len);
}

static void test_main_flow(void)
{
	est_t s;
	if (!establish(&s))
		return;
	cis_engine_t *e = s.e;
	setup_paths(e);

	/* SDU from the host in event 0 -> buffer 2 -> PDU in event 3 */
	uint8_t sdu[120];
	for (int i = 0; i < 120; i++)
		sdu[i] = (uint8_t)(i * 3 + 1);
	send_sdu(e, 0, sdu, 120);
	idle_event(e, 1);
	idle_event(e, 2);
	CHECK(r_prep(3));
	CHECK_EQ(g_tx.len, 124);
	/* SN = NESN = 3 & 1 (after three flushed payloads), LLID 0b00, no NPI */
	CHECK_EQ(g_tx.hdr0, CIS_LLID_COMPLETE_END | CIS_HDR_SN | CIS_HDR_NESN);
	{
		uint8_t plain[120];
		bool ok = cis_ccm_decrypt(&s.k, 3, CIS_DIR_C_TO_P, (uint8_t)(g_tx.hdr0 & CIS_CCM_AAD_MASK), g_tx.payload,
				g_tx.len, plain);
		CHECK(ok);
		CHECK_MEM(plain, sdu, 120);
	}
	cis_pdu_t n = peer_null(3, true);
	r_rx(3, &n, true);
	r_close(3);
	cis_engine_process(e);
	static const uint8_t want_nocp[8] = { 0x04, 0x13, 0x05, 0x01, 0x00, 0x0E, 0x01, 0x00 };
	const uint8_t *q = last_evt(0x13, 0, NULL);
	CHECK(q && memcmp(q, want_nocp, 8) == 0);

	/* event 4: the Peripheral sends 40 B -> an HCI ISO packet with PSN 4 and a timestamp */
	uint8_t up[40];
	for (int i = 0; i < 40; i++)
		up[i] = (uint8_t)(0xC0 + i);
	CHECK(r_prep(4));
	CHECK_EQ(g_tx.hdr0, CIS_LLID_START_CONT | CIS_HDR_NPI);    /* SN = NESN = 0 */
	cis_pdu_t d = peer_data(&s.k, 4, up, 40);
	r_rx(4, &d, true);
	r_close(4);
	cis_engine_process(e);
	uint16_t l = 0;
	q = last_evt(0, 0, &l);
	CHECK(q != NULL);
	if (q) {
		cis_hci_iso_t iso;
		CHECK(cis_hci_iso_parse(q, l, &iso));
		CHECK_EQ(iso.handle, 0x0E00);
		CHECK_EQ(iso.pb, CIS_ISO_PB_COMPLETE);
		CHECK(iso.ts_flag);
		CHECK_EQ(iso.timestamp, s.anchor0 + 4u * 10000u + 1060u);
		CHECK_EQ(iso.psn, 4);
		CHECK_EQ(iso.status, CIS_ISO_STATUS_VALID);
		CHECK_EQ(iso.sdu_len, 40);
		CHECK(iso.data_len == 40 && memcmp(iso.data, up, 40) == 0);
	}
	/* NESN acknowledges the Peripheral data in the next PDU (5 & 1 = 1) */
	CHECK(r_prep(5));
	CHECK_EQ(g_tx.hdr0, CIS_LLID_START_CONT | CIS_HDR_NPI | CIS_HDR_NESN);
	n = peer_null(5, false);
	r_rx(5, &n, true);
	r_close(5);
	cis_engine_process(e);
	/* lost SDUs 1..3 (output path open after event 0) reported as LOST */
	{
		int lost = 0;
		for (int i = 0; i < F.n; i++) {
			cis_hci_iso_t iso;
			if (F.pkt[i][0] == 0x05 && cis_hci_iso_parse(F.pkt[i], F.len[i], &iso) &&
			    iso.status == CIS_ISO_STATUS_LOST && iso.sdu_len == 0)
				lost++;
		}
		CHECK(lost >= 3);
	}

	/* underrun: events 6, 7 without an SDU */
	idle_event(e, 6);
	idle_event(e, 7);
	cis_engine_stats_t st;
	CHECK(cis_engine_get_stats(e, 0x0E00, &st));
	CHECK_EQ(st.tx_acked, 1);
	CHECK_EQ(st.tx_underrun, 4);    /* payloads 4, 5, 6, 7 */
	CHECK_EQ(st.rx_ok, 1);
	CHECK_EQ(st.sdu_in, 1);
	CHECK(!cis_engine_get_stats(e, 0x0E05, &st));

	/* flush: SDU with PSN 7 (CIG mapping: PSN 0 -> event 3, so PSN 7 -> 10), the Peripheral does not respond
	 * in event 10 */
	int nocp_before = count_evt(0x13);
	send_sdu(e, 7, sdu, 120);
	idle_event(e, 8);
	idle_event(e, 9);
	CHECK(r_prep(10));
	CHECK_EQ(g_tx.len, 124);
	r_rx(10, NULL, false);
	r_close(10);
	cis_engine_process(e);
	CHECK(cis_engine_get_stats(e, 0x0E00, &st));
	CHECK_EQ(st.tx_flushed, 1);
	CHECK_EQ(st.sdu_late, 0);
	CHECK_EQ(count_evt(0x13), nocp_before + 1);   /* a flushed one also returns the buffer to the host */

	/* CRC error counted */
	CHECK(r_prep(11));
	r_rx(11, &n, false);
	r_close(11);
	cis_engine_process(e);
	CHECK(cis_engine_get_stats(e, 0x0E00, &st));
	CHECK_EQ(st.rx_crc, 1);

	/* Read ISO TX Sync: the last transmitted SDU */
	{
		uint8_t h[2] = { 0x00, 0x0E };
		CHECK(cmd(e, CIS_OP_LE_READ_ISO_TX_SYNC, h, 2));
		q = last_evt(0x0E, 0, &l);
		CHECK(q && l == 6 + 12 && q[6] == 0 && cis_get_le16(&q[9]) == 7);
	}

	/* Remove CIG with an active CIS: not allowed */
	{
		uint8_t g = 1;
		CHECK(cmd(e, CIS_OP_LE_REMOVE_CIG, &g, 1));
		q = last_evt(0x0E, 0, NULL);
		CHECK(q && q[6] == CIS_ERR_CMD_DISALLOWED);
	}

	/* disconnection by the host: Command Status, LL_CIS_TERMINATE_IND, link_close, Disconnection Complete 0x16 */
	int nctrl = F.n_ctrl;
	uint8_t dp[3] = { 0x00, 0x0E, 0x13 };
	CHECK(cmd(e, CIS_OP_DISCONNECT, dp, 3));
	CHECK_EQ(last_cmd_status(), 0);
	CHECK_EQ(F.n_ctrl, nctrl + 1);
	static const uint8_t want_term[4] = { 0x22, 0x01, 0x00, 0x13 };
	CHECK_MEM(F.ctrl[nctrl], want_term, 4);
	CHECK_EQ(F.n_close, 1);
	static const uint8_t want_dc[7] = { 0x04, 0x05, 0x04, 0x00, 0x00, 0x0E, 0x16 };
	q = last_evt(0x05, 0, NULL);
	CHECK(q && memcmp(q, want_dc, 7) == 0);
	/* a foreign handle goes to the controller */
	uint8_t acl_dp[3] = { 0x01, 0x00, 0x13 };
	CHECK(!cmd(e, CIS_OP_DISCONNECT, acl_dp, 3));
	/* now Remove CIG */
	{
		uint8_t g = 1;
		CHECK(cmd(e, CIS_OP_LE_REMOVE_CIG, &g, 1));
		q = last_evt(0x0E, 0, NULL);
		CHECK(q && q[6] == 0 && q[7] == 1);
		CHECK(!cis_engine_active(e));
	}
}

static void test_reject_paths(void)
{
	cis_engine_t *e = fresh();
	if (!e)
		return;
	/* without host bit 32: Command Disallowed */
	set_cig(e);
	CHECK(create_cis(e, 0x0E00, 0x0001));
	CHECK_EQ(last_cmd_status(), CIS_ERR_CMD_DISALLOWED);
	host_feature(e);

	/* LL_REJECT_EXT_IND for LL_CIS_REQ -> CIS Established with the code from the PDU */
	CHECK(create_cis(e, 0x0E00, 0x0001));
	CHECK_EQ(last_cmd_status(), 0);
	/* a second Create CIS in progress: Command Disallowed */
	CHECK(create_cis(e, 0x0E00, 0x0001));
	CHECK_EQ(last_cmd_status(), CIS_ERR_CMD_DISALLOWED);
	static const uint8_t rej[3] = { 0x11, 0x1F, 0x20 };
	CHECK(cis_engine_ll_ctrl_rx(e, 0x0001, rej, 3));
	const uint8_t *p = last_evt(0x3E, 0x19, NULL);
	CHECK(p && p[4] == 0x20 && cis_get_le16(&p[5]) == 0x0E00);
	CHECK_EQ(F.n_open, 0);
	/* error code 0 from the Peripheral must not give the host a "success" */
	CHECK(create_cis(e, 0x0E00, 0x0001));
	static const uint8_t rej0[3] = { 0x11, 0x1F, 0x00 };
	CHECK(cis_engine_ll_ctrl_rx(e, 0x0001, rej0, 3));
	p = last_evt(0x3E, 0x19, NULL);
	CHECK(p && p[4] == CIS_ERR_UNSPECIFIED);
	/* the rejection of another procedure is not ours */
	static const uint8_t rej_other[3] = { 0x11, 0x14, 0x1A };
	CHECK(!cis_engine_ll_ctrl_rx(e, 0x0001, rej_other, 3));

	/* LL_UNKNOWN_RSP -> 0x1A */
	CHECK(create_cis(e, 0x0E00, 0x0001));
	static const uint8_t unk[2] = { 0x07, 0x1F };
	CHECK(cis_engine_ll_ctrl_rx(e, 0x0001, unk, 2));
	p = last_evt(0x3E, 0x19, NULL);
	CHECK(p && p[4] == CIS_ERR_UNSUPP_REMOTE_FEAT);

	/* no LL_CIS_RSP for 40 s -> LL Response Timeout */
	CHECK(create_cis(e, 0x0E00, 0x0001));
	CHECK(F.n_timer > 0);
	int nest = 0;
	for (int i = 0; i < F.n; i++)
		nest += F.pkt[i][1] == 0x3E && F.pkt[i][3] == 0x19;
	F.now += 39000000u;
	cis_engine_on_timer(e);
	int nest2 = 0;
	for (int i = 0; i < F.n; i++)
		nest2 += F.pkt[i][1] == 0x3E && F.pkt[i][3] == 0x19;
	CHECK_EQ(nest2, nest);
	F.now += 1000001u;
	cis_engine_on_timer(e);
	p = last_evt(0x3E, 0x19, NULL);
	CHECK(p && p[4] == CIS_ERR_LL_RESP_TIMEOUT);
	/* a late LL_CIS_RSP after the timeout is not ours */
	cis_ll_cis_rsp_t r = { 1000, 2000, 106 };
	uint8_t b[9];
	cis_ll_cis_rsp_pack(&r, b);
	CHECK(!cis_engine_ll_ctrl_rx(e, 0x0001, b, 9));

	/* a non-Central on the ACL */
	F.central = false;
	CHECK(create_cis(e, 0x0E00, 0x0001));
	CHECK_EQ(last_cmd_status(), CIS_ERR_CMD_DISALLOWED);
	F.central = true;
	/* Peripheral without the CIS Peripheral feature (bit 29) */
	F.features = 1;
	CHECK(create_cis(e, 0x0E00, 0x0001));
	CHECK_EQ(last_cmd_status(), CIS_ERR_UNSUPP_REMOTE_FEAT);
	F.features = 1ull << CIS_FEAT_BIT_CIS_PERIPHERAL;
	/* unknown ACL */
	CHECK(create_cis(e, 0x0E00, 0x0009));
	CHECK_EQ(last_cmd_status(), CIS_ERR_UNKNOWN_CONN_ID);
}

static void test_establish_timeout(void)
{
	cis_engine_t *e = fresh();
	if (!e)
		return;
	host_feature(e);
	set_cig(e);
	CHECK(create_cis(e, 0x0E00, 0x0001));
	rsp_from_peer(e, 1000, 48940, 106);
	CHECK_EQ(F.n_open, 1);
	for (uint32_t ev = 0; ev < 5; ev++) {
		CHECK(r_prep(ev));
		r_rx(ev, NULL, false);
		r_close(ev);
		cis_engine_process(e);
	}
	CHECK(last_evt(0x3E, 0x19, NULL) == NULL);
	CHECK(r_prep(5));
	r_rx(5, NULL, false);
	r_close(5);
	cis_engine_process(e);
	const uint8_t *p = last_evt(0x3E, 0x19, NULL);
	CHECK(p && p[4] == CIS_ERR_CONN_FAIL_ESTAB);
	CHECK_EQ(F.n_close, 1);
	/* without encryption: link lengths without MIC */
	CHECK_EQ(F.lp.max_pdu_c_to_p, 120);
}

static void test_data_length_gate(void)
{
	cis_engine_t *e = fresh();
	if (!e)
		return;
	host_feature(e);
	set_cig(e);
	/* C6: LL_CIS_REQ (36 B) does not fit in 27 B -> -4; the procedure waits for LE Data Length Change */
	F.ctrl_ret = CIS_PLAT_ERR_TOO_LONG;
	F.ctrl_ret_count = 1;
	CHECK(create_cis(e, 0x0E00, 0x0001));
	CHECK_EQ(last_cmd_status(), 0);
	CHECK_EQ(F.n_ctrl, 0);
	CHECK(last_evt(0x3E, 0x19, NULL) == NULL);
	cis_engine_process(e);
	CHECK_EQ(F.n_ctrl, 0);   /* without a length change we do not retry in a loop */
	/* Data Length Change with MaxTxOctets 27: we keep waiting */
	uint8_t dl[11] = { 0x07, 0x01, 0x00, 27, 0, 0x48, 0x01, 27, 0, 0x48, 0x01 };
	CHECK(!evt(e, dl, 11, CIS_EVT_LE_META));
	CHECK_EQ(F.n_ctrl, 0);
	dl[3] = 251;
	CHECK(!evt(e, dl, 11, CIS_EVT_LE_META));
	CHECK_EQ(F.n_ctrl, 1);
	CHECK_EQ(F.ctrl[0][0], CIS_LL_CIS_REQ);

	/* second case: the length never grows -> after the procedure timeout a Create CIS error */
	cis_engine_t *e2 = fresh();
	host_feature(e2);
	set_cig(e2);
	F.ctrl_ret = CIS_PLAT_ERR_TOO_LONG;
	F.ctrl_ret_count = 100;
	CHECK(create_cis(e2, 0x0E00, 0x0001));
	F.now += 40000001u;
	cis_engine_on_timer(e2);
	const uint8_t *p = last_evt(0x3E, 0x19, NULL);
	CHECK(p && p[4] == CIS_ERR_LIMITED_RESOURCES);
}

/* cis_acl_timing_t.event_len_us: a CIS does not start before the end of the ACL event (C6: the sequencer finishes it at
 * connsm+124). Longer than the fixed 1000 us margin -> CIS_Offset_Min = event_len_us; shorter -> the margin. */
static void test_acl_event_len_guard(void)
{
	static const uint32_t len_us[2] = { 2500, 300 };
	static const uint32_t want_min[2] = { 2500, 1000 };

	for (int i = 0; i < 2; i++) {
		cis_engine_t *e = fresh();
		if (!e)
			return;
		F.timing.event_len_us = len_us[i];
		host_feature(e);
		set_cig(e);
		CHECK(create_cis(e, 0x0E00, 0x0001));
		CHECK_EQ(F.n_ctrl, 1);
		cis_ll_cis_req_t r;
		memset(&r, 0, sizeof(r));
		CHECK(cis_ll_cis_req_parse(F.ctrl[0], F.ctrl_len[0], &r));
		CHECK_EQ(r.cis_offset_min_us, want_min[i]);
		/* the window end does not depend on the event length; ACL 50 ms = 5 x ISO, so the CIG fits in one interval
		 * of ISO after the ACL event: ISO - CIG_Sync_Delay - margin (otherwise every few events a CIS hits the ACL) */
		CHECK_EQ(r.cis_offset_max_us, 10000u - 1060u - 1000u);
	}
}

static void test_peer_terminate_and_acl_loss(void)
{
	est_t s;
	if (!establish(&s))
		return;
	cis_engine_t *e = s.e;
	/* the Peripheral ends the CIS */
	static const uint8_t term[4] = { 0x22, 0x01, 0x00, 0x13 };
	CHECK(cis_engine_ll_ctrl_rx(e, 0x0001, term, 4));
	const uint8_t *p = last_evt(0x05, 0, NULL);
	CHECK(p && cis_get_le16(&p[4]) == 0x0E00 && p[6] == 0x13);
	CHECK_EQ(F.n_close, 1);

	/* ACL loss: the CIS disconnected with the ACL reason, the ACL event goes on to the host */
	est_t s2;
	if (!establish(&s2))
		return;
	e = s2.e;
	int n_before = count_evt(0x05);
	uint8_t dc[4] = { 0x00, 0x01, 0x00, 0x08 };
	CHECK(!evt(e, dc, 4, CIS_EVT_DISCONN_COMPLETE));
	CHECK_EQ(count_evt(0x05), n_before + 1);
	p = last_evt(0x05, 0, NULL);
	CHECK(p && cis_get_le16(&p[4]) == 0x0E00 && p[6] == 0x08);
	CHECK_EQ(F.n_close, 1);
	/* after closing the radio transmits nothing */
	CHECK(!r_prep(7));
}

/* Trace of the setup steps (ops.trace): on the Tab5 the only view of where the setup with the earbuds stalls. */
static const cis_trace_t *trace_find(uint8_t kind, int from)
{
	for (int i = from; i < F.n_tr; i++)
		if (F.tr[i].kind == kind)
			return &F.tr[i];
	return NULL;
}

static void test_trace_steps(void)
{
	est_t s;
	if (!establish(&s))
		return;
	static const uint8_t want_kinds[] = { CIS_TR_CREATE, CIS_TR_REQ, CIS_TR_RSP, CIS_TR_IND, CIS_TR_OPEN,
		CIS_TR_ESTABLISHED };
	CHECK_EQ(F.n_tr, (int)sizeof(want_kinds));
	for (int i = 0; i < F.n_tr && i < (int)sizeof(want_kinds); i++) {
		CHECK_EQ(F.tr[i].kind, want_kinds[i]);
		CHECK_EQ(F.tr[i].cis, 0x0E00);
		CHECK_EQ(F.tr[i].acl, 0x0001);
	}
	const cis_trace_t *t = trace_find(CIS_TR_CREATE, 0);
	CHECK(t && t->v[0] == s.aa);
	t = trace_find(CIS_TR_REQ, 0);
	CHECK(t && t->code == 0 && t->v[0] == (100u | (106u << 16)) && t->v[1] == 1000000u && t->v[2] == 1000u &&
	      t->v[3] == 7940u);
	t = trace_find(CIS_TR_RSP, 0);
	CHECK(t && t->v[0] == 106u && t->v[1] == 1000u && t->v[2] == 48940u);
	t = trace_find(CIS_TR_IND, 0);
	CHECK(t && t->v[0] == (106u | (100u << 16)) && t->v[1] == 1000000u && t->v[2] == 1000u && t->v[3] == 50000u);
	/* CIS anchor from the IND entry = the first link anchor */
	if (t)
		CHECK_EQ(t->v[1] + (uint32_t)((t->v[0] & 0xFFFFu) - (t->v[0] >> 16)) * t->v[3] + t->v[2], s.anchor0);
	t = trace_find(CIS_TR_OPEN, 0);
	CHECK(t && t->code == 0 && t->v[0] == s.anchor0);
	t = trace_find(CIS_TR_ESTABLISHED, 0);
	CHECK(t && t->code == CIS_ERR_SUCCESS && t->v[0] == 1u);

	/* end from the Peripheral: its code, then an end with that reason */
	static const uint8_t term[4] = { 0x22, 0x01, 0x00, 0x13 };
	int n0 = F.n_tr;
	CHECK(cis_engine_ll_ctrl_rx(s.e, 0x0001, term, 4));
	t = trace_find(CIS_TR_PEER_TERM, n0);
	CHECK(t && t->code == 0x13 && t->cis == 0x0E00);
	t = trace_find(CIS_TR_END, n0);
	CHECK(t && t->code == 0x13 && t->v[0] == 1u);

	/* Peripheral refusals of LL_CIS_REQ: the code from the PDU and the opcode, then CIS Established with an error */
	cis_engine_t *e = fresh();
	if (!e)
		return;
	host_feature(e);
	set_cig(e);
	CHECK(create_cis(e, 0x0E00, 0x0001));
	static const uint8_t rej[3] = { 0x11, 0x1F, 0x20 };
	CHECK(cis_engine_ll_ctrl_rx(e, 0x0001, rej, 3));
	t = trace_find(CIS_TR_REFUSED, 0);
	CHECK(t && t->code == 0x20 && t->v[0] == 0x11u);
	t = trace_find(CIS_TR_ESTABLISHED, 0);
	CHECK(t && t->code == 0x20);
	n0 = F.n_tr;
	CHECK(create_cis(e, 0x0E00, 0x0001));
	static const uint8_t unk[2] = { 0x07, 0x1F };
	CHECK(cis_engine_ll_ctrl_rx(e, 0x0001, unk, 2));
	t = trace_find(CIS_TR_REFUSED, n0);
	CHECK(t && t->code == CIS_ERR_UNSUPP_REMOTE_FEAT && t->v[0] == 0x07u);

	/* no response: CIS Established with LL Response Timeout */
	n0 = F.n_tr;
	CHECK(create_cis(e, 0x0E00, 0x0001));
	F.now += 40000001u;
	cis_engine_on_timer(e);
	t = trace_find(CIS_TR_ESTABLISHED, n0);
	CHECK(t && t->code == CIS_ERR_LL_RESP_TIMEOUT);
	CHECK(trace_find(CIS_TR_RSP, n0) == NULL);

	/* LL_CIS_REQ held until LE Data Length: REQ with code 1, not transmitted */
	e = fresh();
	if (!e)
		return;
	host_feature(e);
	set_cig(e);
	F.ctrl_ret = CIS_PLAT_ERR_TOO_LONG;
	F.ctrl_ret_count = 1;
	CHECK(create_cis(e, 0x0E00, 0x0001));
	t = trace_find(CIS_TR_REQ, 0);
	CHECK(t && t->code == 1);
	CHECK_EQ(F.n_ctrl, 0);

	/* a platform without trace (ops.trace = NULL) - the engine works as before */
	cis_platform_ops_t ops = OPS;
	ops.trace = NULL;
	(void)fresh();
	e = cis_engine_init(g_mem, (uint32_t)sizeof(g_mem), &ops);
	CHECK(e != NULL);
	if (!e)
		return;
	host_feature(e);
	set_cig(e);
	CHECK(create_cis(e, 0x0E00, 0x0001));
	CHECK_EQ(last_cmd_status(), 0);
	CHECK_EQ(F.n_ctrl, 1);
	CHECK_EQ(F.n_tr, 0);
}

static void test_mic_failure(void)
{
	est_t s;
	if (!establish(&s))
		return;
	cis_engine_t *e = s.e;
	setup_paths(e);
	uint8_t up[10] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };
	CHECK(r_prep(1));
	cis_pdu_t d = peer_data(&s.k, 1, up, 10);
	d.payload[3] ^= 0x40;
	r_rx(1, &d, true);
	r_close(1);
	cis_engine_process(e);
	cis_engine_stats_t st;
	CHECK(cis_engine_get_stats(e, 0x0E00, &st));
	CHECK_EQ(st.rx_mic_fail, 1);
	const uint8_t *p = last_evt(0x05, 0, NULL);
	CHECK(p && p[6] == CIS_ERR_MIC_FAILURE);
	CHECK_EQ(F.n_close, 1);
	/* the Peripheral gets LL_CIS_TERMINATE_IND with reason 0x3D */
	CHECK(F.n_ctrl >= 1 && F.ctrl[F.n_ctrl - 1][0] == CIS_LL_CIS_TERMINATE_IND && F.ctrl[F.n_ctrl - 1][3] == 0x3D);
}

/* The C->P PDU with data in event 3 is decrypted with a key computed by the test from the Core data. */
static bool c_to_p_decrypts(est_t *s)
{
	cis_engine_t *e = s->e;
	setup_paths(e);
	uint8_t sdu[120];
	for (int i = 0; i < 120; i++)
		sdu[i] = (uint8_t)(i ^ 0x5A);
	send_sdu(e, 0, sdu, 120);
	idle_event(e, 1);
	idle_event(e, 2);
	if (!r_prep(3) || g_tx.len != 124)
		return false;
	uint8_t plain[120];
	return cis_ccm_decrypt(&s->k, 3, CIS_DIR_C_TO_P, (uint8_t)(g_tx.hdr0 & CIS_CCM_AAD_MASK), g_tx.payload,
			g_tx.len, plain) && memcmp(plain, sdu, 120) == 0;
}

static uint32_t key_src(const est_t *s)
{
	cis_engine_stats_t st;
	memset(&st, 0, sizeof(st));
	CHECK(cis_engine_get_stats(s->e, 0x0E00, &st));
	return st.key_src;
}

/* CIS key: the controller (acl_session_keys) before the LTK/SKD/IV snoop - it already encrypts the ACL, which the
 * earbuds accepted; the snoop remains a cross-check. On an encrypted ACL with no key at all Create CIS refuses
 * (0x06) instead of setting up an unencrypted CIS, which the earbuds would not accept. */
static void test_controller_keys(void)
{
	static const uint8_t acl_iv[8] = { 0x24, 0xAB, 0xDC, 0xBA, 0xBE, 0xBA, 0xAF, 0xDE };
	static const struct {
		snoop_t snoop;
		bool ctl;
		uint32_t want;
	} cases[] = {
		{ SNOOP_FULL, false, CIS_KEY_SNOOP },
		{ SNOOP_FULL, true, CIS_KEY_MATCH },
		{ SNOOP_NONE, true, CIS_KEY_CTRL },
		{ SNOOP_BAD_LTK, true, CIS_KEY_MISMATCH },
	};
	cis_ccm_t core;
	cis_keys(0, &core);
	for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
		g_snoop = cases[i].snoop;
		g_ctl.on = cases[i].ctl;
		memcpy(g_ctl.sk, core.sk, 16);
		memcpy(g_ctl.iv, acl_iv, 8);
		est_t s;
		if (establish(&s)) {
			CHECK_EQ(key_src(&s), cases[i].want);
			CHECK(c_to_p_decrypts(&s));
		}
	}

	/* neither a snoop nor controller keys */
	g_snoop = SNOOP_NONE;
	g_ctl.on = false;
	cis_engine_t *e = fresh();
	if (e) {
		encrypt_acl(e, 0x0001);
		host_feature(e);
		set_cig(e);
		CHECK(create_cis(e, 0x0E00, 0x0001));
		CHECK_EQ(last_cmd_status(), CIS_ERR_PIN_KEY_MISSING);
		CHECK_EQ(F.n_ctrl, 0);
	}
	g_snoop = SNOOP_FULL;
}

static void test_radio_owns_seq(void)
{
	cis_engine_t *e = fresh();
	if (!e)
		return;
	F.caps.radio_owns_seq = true;
	host_feature(e);
	set_cig(e);
	CHECK(create_cis(e, 0x0E00, 0x0001));
	rsp_from_peer(e, 1000, 48940, 106);
	CHECK(r_prep(0));
	cis_pdu_t n = peer_null(0, false);
	r_rx(0, &n, true);
	r_close(0);
	cis_engine_process(e);
	/* event 1: logically NESN = 1, but the radio sets the SN/NESN bits - the engine does not write them */
	CHECK(r_prep(1));
	CHECK_EQ(g_tx.hdr0 & (CIS_HDR_SN | CIS_HDR_NESN), 0);
}

static void test_hci_validation(void)
{
	cis_engine_t *e = fresh();
	if (!e)
		return;
	/* Set CIG with a truncated length */
	CHECK(cmd(e, CIS_OP_LE_SET_CIG_PARAMS, CIG1, 20));
	const uint8_t *p = last_evt(0x0E, 0, NULL);
	CHECK(p && p[6] == CIS_ERR_INVALID_PARAMS);
	CHECK(!cis_engine_active(e));
	/* framing -> Unsupported */
	uint8_t c[24];
	memcpy(c, CIG1, 24);
	c[9] = 1;
	CHECK(cmd(e, CIS_OP_LE_SET_CIG_PARAMS, c, 24));
	p = last_evt(0x0E, 0, NULL);
	CHECK(p && p[6] == CIS_ERR_UNSUPP_FEATURE);
	set_cig(e);
	/* a second CIG: out of memory */
	c[0] = 2;
	c[9] = 0;
	CHECK(cmd(e, CIS_OP_LE_SET_CIG_PARAMS, c, 24));
	p = last_evt(0x0E, 0, NULL);
	CHECK(p && p[6] == CIS_ERR_MEM_CAPACITY);
	/* Set CIG again for the same CIG in the configurable state: allowed, the same handle */
	set_cig(e);
	/* Remove CIG of an unknown one */
	uint8_t g = 7;
	CHECK(cmd(e, CIS_OP_LE_REMOVE_CIG, &g, 1));
	p = last_evt(0x0E, 0, NULL);
	CHECK(p && p[6] == CIS_ERR_UNKNOWN_ADV_ID);
	/* data path for a CIS that is not established */
	uint8_t sp[13] = { 0x00, 0x0E, 0x00, 0x00, 0x03, 0, 0, 0, 0, 0, 0, 0, 0 };
	CHECK(cmd(e, CIS_OP_LE_SETUP_ISO_PATH, sp, 13));
	p = last_evt(0x0E, 0, NULL);
	CHECK(p && p[6] == CIS_ERR_CMD_DISALLOWED);
	/* wrong codec configuration length */
	sp[12] = 5;
	CHECK(cmd(e, CIS_OP_LE_SETUP_ISO_PATH, sp, 13));
	p = last_evt(0x0E, 0, NULL);
	CHECK(p && p[6] == CIS_ERR_INVALID_PARAMS);
	/* Disconnect with a disallowed reason */
	uint8_t dp[3] = { 0x00, 0x0E, 0x01 };
	CHECK(cmd(e, CIS_OP_DISCONNECT, dp, 3));
	CHECK_EQ(last_cmd_status(), CIS_ERR_INVALID_PARAMS);
	/* Disconnect of a CIS that is not established */
	dp[2] = 0x13;
	CHECK(cmd(e, CIS_OP_DISCONNECT, dp, 3));
	CHECK_EQ(last_cmd_status(), CIS_ERR_UNKNOWN_CONN_ID);
	/* ISO garbage: inconsistent length, foreign handle - consumed without effect */
	static const uint8_t junk[6] = { 0x05, 0x00, 0x2E, 0x40, 0x00, 0x00 };
	int n = F.n;
	cis_engine_hci_iso(e, junk, 6);
	CHECK_EQ(F.n, n);
	/* HCI Reset clears the CIG and goes to the controller */
	CHECK(!cmd(e, CIS_OP_RESET, NULL, 0));
	CHECK(!cis_engine_active(e));
}

static void test_cancel_pending(void)
{
	cis_engine_t *e = fresh();
	if (!e)
		return;
	host_feature(e);
	set_cig(e);
	CHECK(create_cis(e, 0x0E00, 0x0001));
	uint8_t dp[3] = { 0x00, 0x0E, 0x13 };
	CHECK(cmd(e, CIS_OP_DISCONNECT, dp, 3));
	CHECK_EQ(last_cmd_status(), 0);
	/* the response arrives after cancellation: we reject it and report 0x44 */
	int nctrl = F.n_ctrl;
	rsp_from_peer(e, 1000, 48940, 106);
	CHECK_EQ(F.n_ctrl, nctrl + 1);
	CHECK_EQ(F.ctrl[nctrl][0], CIS_LL_REJECT_EXT_IND);
	CHECK_EQ(F.ctrl[nctrl][1], CIS_LL_CIS_RSP);
	const uint8_t *p = last_evt(0x3E, 0x19, NULL);
	CHECK(p && p[4] == CIS_ERR_OP_CANCELLED_BY_HOST);
	CHECK_EQ(F.n_open, 0);
}


/* BN = 2, NSE = 2 (a radio with two subevents): SDU 300 B -> START 247 B in subevent 0, END 53 B in 1;
 * each payload with its own CCM counter; the flush point of an even payload = subevent 0. */
static void test_bn2_segmentation(void)
{
	cis_engine_t *e = fresh();
	if (!e)
		return;
	F.caps.max_nse = 2;
	encrypt_acl(e, 0x0001);
	host_feature(e);
	uint8_t c[24];
	memcpy(c, CIG1, 24);
	cis_put_le16(&c[16], 300);
	CHECK(cmd(e, CIS_OP_LE_SET_CIG_PARAMS, c, 24));
	CHECK(create_cis(e, 0x0E00, 0x0001));
	CHECK_EQ(F.ctrl[0][19], 2);          /* NSE */
	CHECK_EQ(F.ctrl[0][23], 0x12);       /* BN_C 2, BN_P 1 */
	CHECK_EQ(cis_get_le16(&F.ctrl[0][15]), CIS_MAX_PDU);
	CHECK_EQ(cis_get_le24(&F.ctrl[0][20]), 1568);   /* Sub_Interval */
	rsp_from_peer(e, 1000, 46000, 106);
	cis_ll_cis_ind_t ind;
	CHECK(cis_ll_cis_ind_parse(F.ctrl[1], F.ctrl_len[1], &ind));
	CHECK_EQ(ind.cig_sync_delay_us, 3136);
	CHECK_EQ(F.lp.nse, 2);
	CHECK_EQ(F.lp.sub_interval_us, 1568);
	CHECK_EQ(F.lp.max_pdu_c_to_p, 251);
	cis_ccm_t k;
	cis_keys(ind.access_address, &k);
	for (uint32_t ev = 0; ev < 1; ev++) {
		for (uint8_t se = 0; se < 2; se++) {
			CHECK(F.cb->prepare_tx(F.cb_arg, ev, se, &g_tx));
			cis_pdu_t n = peer_null(ev, false);
			F.cb->rx_done(F.cb_arg, ev, se, &n, true, F.now);
		}
		r_close(ev);
		cis_engine_process(e);
	}
	CHECK(last_evt(0x3E, 0x19, NULL) != NULL);
	setup_paths(e);
	uint8_t sdu[300];
	for (int i = 0; i < 300; i++)
		sdu[i] = (uint8_t)(i ^ 0x5A);
	send_sdu(e, 0, sdu, 300);
	for (uint32_t ev = 1; ev < 3; ev++) {
		for (uint8_t se = 0; se < 2; se++) {
			CHECK(F.cb->prepare_tx(F.cb_arg, ev, se, &g_tx));
			CHECK_EQ(g_tx.len, 0);
			cis_pdu_t n = peer_null(ev, false);
			F.cb->rx_done(F.cb_arg, ev, se, &n, true, F.now);
		}
		r_close(ev);
		cis_engine_process(e);
	}
	/* event 3: payloads 6 and 7 (SN after six payloads = 0, then 1) */
	uint8_t got[300];
	uint16_t got_len = 0;
	for (uint8_t se = 0; se < 2; se++) {
		CHECK(F.cb->prepare_tx(F.cb_arg, 3, se, &g_tx));
		CHECK_EQ(g_tx.hdr0 & 0x03, se == 0 ? CIS_LLID_START_CONT : CIS_LLID_COMPLETE_END);
		CHECK_EQ((g_tx.hdr0 & CIS_HDR_SN) != 0, se == 1);
		uint8_t plain[CIS_PDU_MAX_PAYLOAD];
		bool ok = g_tx.len >= 4 && cis_ccm_decrypt(&k, 6u + se, CIS_DIR_C_TO_P,
				(uint8_t)(g_tx.hdr0 & CIS_CCM_AAD_MASK), g_tx.payload, g_tx.len, plain);
		CHECK(ok);
		if (ok && got_len + g_tx.len - 4u <= sizeof(got)) {
			memcpy(got + got_len, plain, g_tx.len - 4u);
			got_len = (uint16_t)(got_len + g_tx.len - 4u);
		}
		/* the Peripheral accepted: NESN = SN + 1 */
		cis_pdu_t n;
		memset(&n, 0, sizeof(n));
		n.hdr0 = (uint8_t)(CIS_LLID_START_CONT | CIS_HDR_NPI | (se == 0 ? CIS_HDR_NESN : 0));
		F.cb->rx_done(F.cb_arg, 3, se, &n, true, F.now);
	}
	r_close(3);
	cis_engine_process(e);
	CHECK_EQ(got_len, 300);
	CHECK_MEM(got, sdu, 300);
	cis_engine_stats_t st;
	CHECK(cis_engine_get_stats(e, 0x0E00, &st));
	CHECK_EQ(st.tx_acked, 2);
	CHECK_EQ(st.tx_flushed, 0);
	const uint8_t *q = last_evt(0x13, 0, NULL);
	CHECK(q && cis_get_le16(&q[6]) == 1);

	/* An even payload without an acknowledgement in subevent 0: its flush point has passed, so subevent 1 already
	 * carries the odd payload (END), not a START retransmission. SDU with PSN 3 (PSN 0 -> event 3) -> event 6. */
	send_sdu(e, 3, sdu, 300);
	for (uint32_t ev = 4; ev < 6; ev++) {
		for (uint8_t se = 0; se < 2; se++) {
			CHECK(F.cb->prepare_tx(F.cb_arg, ev, se, &g_tx));
			cis_pdu_t n = peer_null(ev, false);
			F.cb->rx_done(F.cb_arg, ev, se, &n, true, F.now);
		}
		r_close(ev);
		cis_engine_process(e);
	}
	CHECK(F.cb->prepare_tx(F.cb_arg, 6, 0, &g_tx));
	CHECK_EQ(g_tx.hdr0 & 0x03, CIS_LLID_START_CONT);
	F.cb->rx_done(F.cb_arg, 6, 0, NULL, false, F.now);
	CHECK(F.cb->prepare_tx(F.cb_arg, 6, 1, &g_tx));
	CHECK_EQ(g_tx.hdr0 & 0x03, CIS_LLID_COMPLETE_END);
	{
		uint8_t plain[CIS_PDU_MAX_PAYLOAD];
		CHECK(g_tx.len == 57 && cis_ccm_decrypt(&k, 13, CIS_DIR_C_TO_P, (uint8_t)(g_tx.hdr0 & CIS_CCM_AAD_MASK),
				g_tx.payload, g_tx.len, plain));
	}
	r_close(6);
	cis_engine_process(e);
	CHECK(cis_engine_get_stats(e, 0x0E00, &st));
	CHECK_EQ(st.tx_flushed, 2);    /* 12 in subevent 1, 13 at the end of the event */
	CHECK_EQ(st.sdu_late, 0);
}

/* Two CISes of one CIG on two ACLs: the second CIS gets a CIS_Offset in the CIG phase (anchor1 - anchor0 = 1060 mod
 * ISO). The LL_CIS_REQ of the second one already knows the phase: the window is a single value (min = max), as in
 * Zephyr (ull_central_iso.c, mfy_cis_offset_get). The Galaxy Buds3 Pro narrows every window to a single value (1312
 * us after its ACL event), so with a full window the second earbud answered with an offset outside the CIG phase and
 * choose_instant rejected it (0x20). */
static void test_two_cis_phase(void)
{
	cis_engine_t *e = fresh();
	if (!e)
		return;
	host_feature(e);
	static const uint8_t cig2[33] = {
		0x01, 0x10, 0x27, 0x00, 0x10, 0x27, 0x00, 0x00, 0x00, 0x00, 0x0A, 0x00, 0x0A, 0x00, 0x02,
		0x00, 0x78, 0x00, 0x28, 0x00, 0x02, 0x02, 0x00, 0x00,
		0x01, 0x78, 0x00, 0x28, 0x00, 0x02, 0x02, 0x00, 0x00,
	};
	CHECK(cmd(e, CIS_OP_LE_SET_CIG_PARAMS, cig2, 33));
	const uint8_t *q = last_evt(0x0E, 0, NULL);
	CHECK(q && q[6] == 0 && q[8] == 2 && cis_get_le16(&q[9]) == 0x0E00 && cis_get_le16(&q[11]) == 0x0E01);
	/* ACL 2 has a different phase: the anchor is shifted by 3 ms (the same interval) */
	F.timing2 = F.timing;
	F.timing2.anchor_us += 3000;
	F.timing2_on = true;
	uint8_t p[9] = { 2, 0x00, 0x0E, 0x01, 0x00, 0x01, 0x0E, 0x02, 0x00 };
	CHECK(cmd(e, CIS_OP_LE_CREATE_CIS, p, 9));
	CHECK_EQ(last_cmd_status(), 0);
	CHECK_EQ(F.n_ctrl, 1);   /* one LL_CIS_REQ at a time in a CIG, also on different ACLs (the first fixes the phase) */
	rsp_from_peer(e, 1312, 1312, 106);
	cis_ll_cis_ind_t i0, i1;
	CHECK(cis_ll_cis_ind_parse(F.ctrl[1], F.ctrl_len[1], &i0));
	CHECK_EQ(i0.cis_offset_us, 1312);
	CHECK_EQ(i0.cig_sync_delay_us, 2120);
	CHECK_EQ(i0.cis_sync_delay_us, 2120);
	uint32_t a0 = F.lp.first_anchor_us;
	/* the second LL_CIS_REQ only after the response to the first */
	CHECK_EQ(F.n_ctrl, 3);
	CHECK_EQ(F.ctrl_acl[2], 0x0002);
	CHECK_EQ(F.ctrl[2][2], 1);   /* CIS_ID */
	cis_ll_cis_req_t r1;
	memset(&r1, 0, sizeof(r1));
	CHECK(cis_ll_cis_req_parse(F.ctrl[2], F.ctrl_len[2], &r1));
	CHECK_EQ(r1.cis_offset_min_us, r1.cis_offset_max_us);
	CHECK(r1.cis_offset_min_us >= 1000 && r1.cis_offset_min_us <= 50000u - 2120u - 1000u);
	const uint32_t acl2 = F.timing2.anchor_us +
		(uint32_t)(uint16_t)(r1.conn_event_count - F.timing2.event_counter) * F.timing2.interval_us;
	CHECK_EQ((acl2 + r1.cis_offset_min_us - a0) % 10000u, 1060);
	/* the second earbud accepts exactly that offset */
	cis_ll_cis_rsp_t r = { r1.cis_offset_min_us, r1.cis_offset_max_us, r1.conn_event_count };
	uint8_t b[9];
	cis_ll_cis_rsp_pack(&r, b);
	CHECK(cis_engine_ll_ctrl_rx(e, 0x0002, b, 9));
	CHECK(F.n_ctrl == 4 && cis_ll_cis_ind_parse(F.ctrl[3], F.ctrl_len[3], &i1));
	CHECK_EQ(i1.cis_offset_us, r1.cis_offset_min_us);
	CHECK_EQ(i1.cis_sync_delay_us, 1060);
	CHECK(i1.access_address != i0.access_address);
	uint32_t a1 = F.lp.first_anchor_us;
	CHECK_EQ((a1 - a0) % 10000u, 1060);
	CHECK_EQ(F.n_open, 2);
}


/* Radio gap between CISes (cis_radio_caps_t.cis_gap_us): CIS 1 at 1060 + 2500 us after CIS 0 in the CIG, CIG_Sync_Delay
 * and CIS_Sync_Delay in LL_CIS_IND include the gap, and the anchor of the second one is in this phase. */
static void test_two_cis_gap(void)
{
	cis_engine_t *e = fresh();
	if (!e)
		return;
	F.caps.cis_gap_us = 2500;
	host_feature(e);
	static const uint8_t cig2[33] = {
		0x01, 0x10, 0x27, 0x00, 0x10, 0x27, 0x00, 0x00, 0x00, 0x00, 0x0A, 0x00, 0x0A, 0x00, 0x02,
		0x00, 0x78, 0x00, 0x28, 0x00, 0x02, 0x02, 0x00, 0x00,
		0x01, 0x78, 0x00, 0x28, 0x00, 0x02, 0x02, 0x00, 0x00,
	};
	CHECK(cmd(e, CIS_OP_LE_SET_CIG_PARAMS, cig2, 33));
	F.timing2 = F.timing;
	F.timing2.anchor_us += 3000;
	F.timing2_on = true;
	uint8_t p[9] = { 2, 0x00, 0x0E, 0x01, 0x00, 0x01, 0x0E, 0x02, 0x00 };
	CHECK(cmd(e, CIS_OP_LE_CREATE_CIS, p, 9));
	cis_ll_cis_req_t r0;
	memset(&r0, 0, sizeof(r0));
	CHECK(F.n_ctrl == 1 && cis_ll_cis_req_parse(F.ctrl[0], F.ctrl_len[0], &r0));
	/* the first one fixes the phase: an exact offset from place_cig (two ACLs) in the CIG window within one ISO; the
	 * Peripheral may still pick another one from that window (Buds3 Pro: 1312) - we accept it */
	CHECK_EQ(r0.cis_offset_min_us, r0.cis_offset_max_us);
	CHECK(r0.cis_offset_min_us >= 1000u && r0.cis_offset_min_us <= 10000u - 4620u - 1000u);
	rsp_from_peer(e, 1312, 1312, 106);
	cis_ll_cis_ind_t i0, i1;
	CHECK(cis_ll_cis_ind_parse(F.ctrl[1], F.ctrl_len[1], &i0));
	CHECK_EQ(i0.cig_sync_delay_us, 4620);
	CHECK_EQ(i0.cis_sync_delay_us, 4620);
	const uint32_t a0 = F.lp.first_anchor_us;
	cis_ll_cis_req_t r1;
	memset(&r1, 0, sizeof(r1));
	CHECK(F.n_ctrl == 3 && cis_ll_cis_req_parse(F.ctrl[2], F.ctrl_len[2], &r1));
	CHECK_EQ(r1.cis_offset_min_us, r1.cis_offset_max_us);
	cis_ll_cis_rsp_t r = { r1.cis_offset_min_us, r1.cis_offset_max_us, r1.conn_event_count };
	uint8_t b[9];
	cis_ll_cis_rsp_pack(&r, b);
	CHECK(cis_engine_ll_ctrl_rx(e, 0x0002, b, 9));
	CHECK(F.n_ctrl == 4 && cis_ll_cis_ind_parse(F.ctrl[3], F.ctrl_len[3], &i1));
	CHECK_EQ(i1.cig_sync_delay_us, 4620);
	CHECK_EQ(i1.cis_sync_delay_us, 1060);
	CHECK_EQ((F.lp.first_anchor_us - a0) % 10000u, 3560);
}

/* Event ev of link i (link_open order): the PDU to send in g_tx, then an empty response and the end of the event. */
static void link_event(cis_engine_t *e, int i, uint32_t ev)
{
	CHECK(F.cbs[i]->prepare_tx(F.cb_args[i], ev, 0, &g_tx));
	cis_pdu_t n = peer_null(ev, g_tx.len != 0);
	F.cbs[i]->rx_done(F.cb_args[i], ev, 0, &n, true, F.now);
	F.cbs[i]->event_closed(F.cb_args[i], ev);
	cis_engine_process(e);
}

/* Two CISes on two ACLs with a gap (CIS 1 at 3560 us after CIS 0): both established, input paths open. Returns
 * the shift of the CIS 1 event numbering relative to CIS 0 (CIG event = CIS 0 event = CIS 1 event + off1). */
static bool two_cis_streaming(cis_engine_t **out, uint32_t *off1)
{
	cis_engine_t *e = fresh();
	if (!e)
		return false;
	F.caps.cis_gap_us = 2500;
	host_feature(e);
	static const uint8_t cig2[33] = {
		0x01, 0x10, 0x27, 0x00, 0x10, 0x27, 0x00, 0x00, 0x00, 0x00, 0x0A, 0x00, 0x0A, 0x00, 0x02,
		0x00, 0x78, 0x00, 0x28, 0x00, 0x02, 0x02, 0x00, 0x00,
		0x01, 0x78, 0x00, 0x28, 0x00, 0x02, 0x02, 0x00, 0x00,
	};
	CHECK(cmd(e, CIS_OP_LE_SET_CIG_PARAMS, cig2, 33));
	F.timing2 = F.timing;
	F.timing2.anchor_us += 3000;
	F.timing2_on = true;
	uint8_t p[9] = { 2, 0x00, 0x0E, 0x01, 0x00, 0x01, 0x0E, 0x02, 0x00 };
	CHECK(cmd(e, CIS_OP_LE_CREATE_CIS, p, 9));
	rsp_from_peer(e, 1312, 1312, 106);
	const uint32_t a0 = F.lp.first_anchor_us;
	cis_ll_cis_req_t r1;
	memset(&r1, 0, sizeof(r1));
	if (!(F.n_ctrl == 3 && cis_ll_cis_req_parse(F.ctrl[2], F.ctrl_len[2], &r1)))
		return false;
	cis_ll_cis_rsp_t r = { r1.cis_offset_min_us, r1.cis_offset_max_us, r1.conn_event_count };
	uint8_t b[9];
	cis_ll_cis_rsp_pack(&r, b);
	CHECK(cis_engine_ll_ctrl_rx(e, 0x0002, b, 9));
	if (F.n_open != 2)
		return false;
	const uint32_t a1 = F.lp.first_anchor_us;
	*off1 = (a1 - 3560u - a0) / 10000u;
	CHECK_EQ((a1 - 3560u - a0) % 10000u, 0);
	/* setup: the first event of each CIS with a Peripheral response */
	link_event(e, 0, 0);
	link_event(e, 1, 0);
	CHECK_EQ(count_evt(0x3E), 2);
	for (uint8_t h = 0; h < 2; h++) {
		const uint8_t sp[13] = { h, 0x0E, 0x00, 0x00, 0x03, 0, 0, 0, 0, 0, 0, 0, 0 };
		CHECK(cmd(e, CIS_OP_LE_SETUP_ISO_PATH, sp, 13));
		const uint8_t *q = last_evt(0x0E, 0, NULL);
		CHECK(q && q[6] == 0);
	}
	*out = e;
	return true;
}

/* First event of link i from ev with data (len > 0), at most 8 events; -1 = none. */
static int64_t first_data_event(cis_engine_t *e, int i, uint32_t ev)
{
	for (uint32_t k = ev; k < ev + 8u; k++) {
		link_event(e, i, k);
		if (g_tx.len != 0)
			return k;
	}
	return -1;
}

/* A host SDU with the same PSN goes to both CISes in the same CIG event, also when it arrives between the CIS 0 event
 * and the CIS 1 event of the same interval (CIS 0 is already an interval ahead). Previously each CIS anchored its own
 * queue by itself - the L/R earbuds then played shifted by 10 ms (Tab5, 2026-11-01, observed after a screen change). */
static void test_two_cis_sdu_aligned(void)
{
	cis_engine_t *e = NULL;
	uint32_t off1 = 0;
	if (!two_cis_streaming(&e, &off1))
		return;
	/* CIS 0 after event 20 (CIG event 20), CIS 1 after CIG event 19: its event 20 is still ahead of it */
	link_event(e, 0, 20);
	link_event(e, 1, 19 - off1);
	uint8_t sdu7[120], sdu8[120];
	memset(sdu7, 0x57, sizeof(sdu7));
	memset(sdu8, 0x58, sizeof(sdu8));
	send_sdu_to(e, 0x0E00, 7, sdu7, 120);
	send_sdu_to(e, 0x0E01, 7, sdu7, 120);
	send_sdu_to(e, 0x0E00, 8, sdu8, 120);
	send_sdu_to(e, 0x0E01, 8, sdu8, 120);
	const int64_t e0 = first_data_event(e, 0, 21);
	CHECK_EQ(g_tx.payload[0], 0x57);
	const int64_t e1 = first_data_event(e, 1, 20 - off1);
	CHECK_EQ(g_tx.payload[0], 0x57);
	CHECK(e0 >= 0 && e1 >= 0);
	CHECK_EQ(e0, e1 + off1);
	/* the next SDU in the next CIG event on both */
	link_event(e, 0, (uint32_t)e0 + 1u);
	CHECK(g_tx.len != 0 && g_tx.payload[0] == 0x58);
	link_event(e, 1, (uint32_t)e1 + 1u);
	CHECK(g_tx.len != 0 && g_tx.payload[0] == 0x58);

	/* A host stall of 10 intervals and a permanently shifted numbering (further from PSN 9, as if nothing had
	 * passed): the first late SDUs are dropped, after CIS_REMAP_LATE a new mapping for the whole CIG - and again the
	 * same SDU in the same event of the CIG on both (the SDU arrives between the CIS 0 and CIS 1 events). */
	uint32_t k0 = (uint32_t)e0 + 2u, k1 = (uint32_t)e1 + 2u;
	for (int i = 0; i < 10; i++) {
		link_event(e, 0, k0++);
		link_event(e, 1, k1++);
	}
	int both = 0, differ = 0;
	for (uint16_t i = 0; i < 24; i++) {
		uint8_t sdux[120];
		memset(sdux, (int)(0x60 + i), sizeof(sdux));
		send_sdu_to(e, 0x0E00, (uint16_t)(9 + i), sdux, 120);
		send_sdu_to(e, 0x0E01, (uint16_t)(9 + i), sdux, 120);
		link_event(e, 0, k0++);
		const int b0 = g_tx.len ? g_tx.payload[0] : -1;
		link_event(e, 1, k1++);
		const int b1 = g_tx.len ? g_tx.payload[0] : -1;
		both += b0 >= 0 && b0 == b1;
		differ += b0 != b1;
	}
	CHECK(both >= 8);
	CHECK_EQ(differ, 0);
}

/* A short host stall and a burst of backlogged SDUs at once: the first one late (new mapping), the rest of the burst
 * too far ahead (buffer full). An SDU rejected as "too early" must not move the mapping - otherwise every following
 * SDU would also fall outside the buffer and the sound would vanish permanently (seen on the Tab5: after leaving the
 * Bluetooth screen 11 193 SDUs were dropped). */
static void test_two_cis_burst_recovers(void)
{
	cis_engine_t *e = NULL;
	uint32_t off1 = 0;
	if (!two_cis_streaming(&e, &off1))
		return;
	uint8_t sdu[120];
	memset(sdu, 0x33, sizeof(sdu));
	uint16_t psn = 0;
	uint32_t ev0 = 1, ev1 = 1;
	/* start-up: an SDU every interval */
	for (int i = 0; i < 6; i++, psn++) {
		send_sdu_to(e, 0x0E00, psn, sdu, 120);
		send_sdu_to(e, 0x0E01, psn, sdu, 120);
		link_event(e, 0, ev0++);
		link_event(e, 1, ev1++);
	}
	/* host stall of 4 intervals, then 8 backlogged SDUs at once */
	for (int i = 0; i < 4; i++) {
		link_event(e, 0, ev0++);
		link_event(e, 1, ev1++);
	}
	for (int i = 0; i < 8; i++, psn++) {
		send_sdu_to(e, 0x0E00, psn, sdu, 120);
		send_sdu_to(e, 0x0E01, psn, sdu, 120);
	}
	/* then evenly every interval: after a short transition every event has data again */
	int with0 = 0, with1 = 0;
	for (int i = 0; i < 30; i++, psn++) {
		send_sdu_to(e, 0x0E00, psn, sdu, 120);
		send_sdu_to(e, 0x0E01, psn, sdu, 120);
		link_event(e, 0, ev0++);
		with0 += i >= 20 && g_tx.len != 0;
		link_event(e, 1, ev1++);
		with1 += i >= 20 && g_tx.len != 0;
	}
	CHECK_EQ(with0, 10);
	CHECK_EQ(with1, 10);
}

/* Event of link i from ev in which a payload with first byte b goes out (at most 12 events); -1 = it did not. */
static int64_t event_with(cis_engine_t *e, int i, uint32_t ev, uint8_t b, uint32_t *next_ev)
{
	for (uint32_t k = ev; k < ev + 12u; k++) {
		link_event(e, i, k);
		if (g_tx.len != 0 && g_tx.payload[0] == b) {
			*next_ev = k + 1u;
			return k;
		}
	}
	*next_ev = ev + 12u;
	return -1;
}

/* A single late SDU (a momentary host stall) drops out by itself - without a new mapping. Previously every late SDU
 * moved the CIG to cur + 1 + buffer, and the following (on-time) SDUs lined up behind it: the queue then stayed at
 * its maximum and held the host credits. */
static void test_single_late_keeps_mapping(void)
{
	cis_engine_t *e = NULL;
	uint32_t off1 = 0;
	if (!two_cis_streaming(&e, &off1))
		return;
	uint8_t sdu[120];
	uint32_t ev = 1;
	uint16_t psn = 0;
	/* start-up: an SDU every interval, both CISes */
	for (int i = 0; i < 6; i++, psn++) {
		memset(sdu, (int)(0x10 + psn), sizeof(sdu));
		send_sdu_to(e, 0x0E00, psn, sdu, 120);
		send_sdu_to(e, 0x0E01, psn, sdu, 120);
		link_event(e, 0, ev);
		link_event(e, 1, ev - off1);
		ev++;
	}
	/* the next SDU (psn 6) arrives too late: 6 events without an SDU, then psn 6 and right after psn 7..9 on time */
	for (int i = 0; i < 6; i++) {
		link_event(e, 0, ev);
		link_event(e, 1, ev - off1);
		ev++;
	}
	for (uint16_t p = 6; p <= 12; p++) {
		memset(sdu, (int)(0x10 + p), sizeof(sdu));
		send_sdu_to(e, 0x0E00, p, sdu, 120);
		send_sdu_to(e, 0x0E01, p, sdu, 120);
	}
	/* psn 6..11 already late (dropped), is psn 12 the second late one in a row? No - psn 12 still has room:
	 * the start-up mapping puts it on an event that has not come yet. We check that psn 12 went out in its
	 * original place (the same distance from psn 5 as at start-up), not shifted by a buffer. */
	uint32_t nx = 0;
	const int64_t at12 = event_with(e, 0, ev, 0x10 + 12, &nx);
	CHECK(at12 >= 0);
	/* psn 0 went out in event 3 (1 + 2 buffer), so psn 12 in event 15 */
	CHECK_EQ(at12, 15);
}

/* Host clock drift relative to the radio (Tab5, 2026-11-01: after 30 min 10 % of SDUs late): SDUs arrive later and
 * later and hover at the boundary - alternately on time and one event too late. The mapping changed only after
 * CIS_REMAP_LATE late SDUs in a row, and on-time SDUs reset the counter - every other SDU was dropped endlessly. Now
 * CIS_DRIFT_LATE late SDUs within a window of CIS_DRIFT_WINDOW SDUs shift the mapping by one event (one gap) and
 * then everything goes on time. */
static void test_drift_shifts_mapping(void)
{
	cis_engine_t *e = NULL;
	uint32_t off1 = 0;
	if (!two_cis_streaming(&e, &off1))
		return;
	cis_engine_stats_t st0;
	CHECK(cis_engine_get_stats(e, 0x0E00, &st0));
	uint8_t sdu[120];
	memset(sdu, 0x33, sizeof(sdu));
	uint16_t psn = 0;
	for (uint32_t ev = 1; ev <= 400; ev++) {
		/* host: from event 50 delayed by an interval (the buffer boundary), from 100 additionally every other SDU
		 * one more interval later */
		for (;;) {
			const uint32_t lag    = ev < 50 ? 0u : 1u;
			const uint32_t jitter = ev >= 100 && (psn & 1u) ? 1u : 0u;
			if (psn + 1u + lag + jitter > ev)
				break;
			send_sdu_to(e, 0x0E00, psn, sdu, 120);
			send_sdu_to(e, 0x0E01, psn, sdu, 120);
			psn++;
		}
		link_event(e, 0, ev);
		link_event(e, 1, ev - off1);
		F.n = 0;   /* host events (NOCP) are not checked here - avoids overflowing the fake in the long loop */
	}
	cis_engine_stats_t st;
	CHECK(cis_engine_get_stats(e, 0x0E00, &st));
	CHECK(st.sdu_late - st0.sdu_late <= 3u);   /* without the shift: about 150 (every other SDU from event 100) */
}

/* A queue above the target (host stall and an SDU burst) returns to the buffer: the CIG drops one SDU at a time
 * (both CISes together), instead of permanently holding 2 x the buffer of SDUs and host credits. */
static void test_queue_recenters(void)
{
	cis_engine_t *e = NULL;
	uint32_t off1 = 0;
	if (!two_cis_streaming(&e, &off1))
		return;
	uint8_t sdu[120];
	uint32_t ev = 1;
	uint16_t psn = 0;
	/* start-up, then 3 SDUs ahead at once (the host faster after a stall): the queue 3 above the target */
	for (int i = 0; i < 4; i++, psn++) {
		memset(sdu, (int)psn, sizeof(sdu));
		send_sdu_to(e, 0x0E00, psn, sdu, 120);
		send_sdu_to(e, 0x0E01, psn, sdu, 120);
		link_event(e, 0, ev);
		link_event(e, 1, ev - off1);
		ev++;
	}
	for (int i = 0; i < 3; i++, psn++) {
		memset(sdu, (int)psn, sizeof(sdu));
		send_sdu_to(e, 0x0E00, psn, sdu, 120);
		send_sdu_to(e, 0x0E01, psn, sdu, 120);
	}
	/* 6 s at a steady rate: SDU psn sent right before event ev - its latency (transmission event - ev) at the end
	 * returns to 1 + buffer (3), the same on both CISes */
	int64_t lag0 = -1, lag1 = -1;
	for (int i = 0; i < 600; i++, psn++) {
		F.n = 0;   /* events to the host (NOCP) are irrelevant - the fake buffer has a limited capacity */
		memset(sdu, (int)(psn & 0xFF), sizeof(sdu));
		send_sdu_to(e, 0x0E00, psn, sdu, 120);
		send_sdu_to(e, 0x0E01, psn, sdu, 120);
		link_event(e, 0, ev);
		const uint8_t b0 = g_tx.len ? g_tx.payload[0] : 0xFF;
		link_event(e, 1, ev - off1);
		const uint8_t b1 = g_tx.len ? g_tx.payload[0] : 0xFF;
		if (i == 599) {
			lag0 = (int64_t)(uint8_t)(psn - b0);
			lag1 = (int64_t)(uint8_t)(psn - b1);
		}
		ev++;
	}
	CHECK(lag0 <= 3);
	CHECK_EQ(lag0, lag1);
}

/* Overlap of the interval [s, s + len) with [t, t + tlen) on a circle of length p (us). */
static uint32_t circ_overlap(uint32_t s, uint32_t len, uint32_t t, uint32_t tlen, uint32_t p)
{
	uint32_t sum = 0;
	s %= p;
	t %= p;
	for (int k = -1; k <= 1; k++) {
		const int64_t a0 = (int64_t)s, a1 = a0 + len;
		const int64_t b0 = (int64_t)t + (int64_t)k * p, b1 = b0 + tlen;
		const int64_t lo = a0 > b0 ? a0 : b0, hi = a1 < b1 ? a1 : b1;
		if (hi > lo)
			sum += (uint32_t)(hi - lo);
	}
	return sum;
}

/* The first CIS fixes the CIG phase, and the ACL events of both earbuds lie where the controller put them. On the
 * Tab5 the ACL of the left one landed 1.4 ms after the CIS anchor of the right one - inside the reservation of its
 * event in the C6 schedule: every 30 ms one of them was lost, the right one lost more PDUs, and the left one dropped
 * its ACL after tens of seconds (0x08). Now the CIS that fixes the phase gets an exact CIS_Offset at which the
 * events of both ACLs fall outside the CIS events (including the gap to the next CIS). */
static void test_cig_placement_avoids_acls(void)
{
	cis_engine_t *e = fresh();
	if (!e)
		return;
	F.caps.cis_gap_us = 2500;
	host_feature(e);
	static const uint8_t cig2[33] = {
		0x01, 0x10, 0x27, 0x00, 0x10, 0x27, 0x00, 0x00, 0x00, 0x00, 0x0A, 0x00, 0x0A, 0x00, 0x02,
		0x00, 0x78, 0x00, 0x28, 0x00, 0x02, 0x02, 0x00, 0x00,
		0x01, 0x78, 0x00, 0x28, 0x00, 0x02, 0x02, 0x00, 0x00,
	};
	CHECK(cmd(e, CIS_OP_LE_SET_CIG_PARAMS, cig2, 33));
	/* ACL 2 600 us after ACL 1: with CIS_Offset 1000 (the smallest in the window) CIS 0 [1000, 4560) covers ACL 2 */
	F.timing2 = F.timing;
	F.timing2.anchor_us += 600;
	F.timing2_on = true;
	uint8_t p[9] = { 2, 0x00, 0x0E, 0x01, 0x00, 0x01, 0x0E, 0x02, 0x00 };
	CHECK(cmd(e, CIS_OP_LE_CREATE_CIS, p, 9));
	cis_ll_cis_req_t r0;
	memset(&r0, 0, sizeof(r0));
	CHECK(F.n_ctrl == 1 && cis_ll_cis_req_parse(F.ctrl[0], F.ctrl_len[0], &r0));
	CHECK_EQ(r0.cis_offset_min_us, r0.cis_offset_max_us);
	/* the Peripheral accepts exactly what it was given */
	rsp_from_peer(e, r0.cis_offset_min_us, r0.cis_offset_max_us, r0.conn_event_count);
	const uint32_t a0 = F.lp.first_anchor_us;
	cis_ll_cis_req_t r1;
	memset(&r1, 0, sizeof(r1));
	CHECK(F.n_ctrl == 3 && cis_ll_cis_req_parse(F.ctrl[2], F.ctrl_len[2], &r1));
	cis_ll_cis_rsp_t r = { r1.cis_offset_min_us, r1.cis_offset_max_us, r1.conn_event_count };
	uint8_t b[9];
	cis_ll_cis_rsp_pack(&r, b);
	CHECK(cis_engine_ll_ctrl_rx(e, 0x0002, b, 9));
	CHECK_EQ(F.n_open, 2);
	/* CIS 0 in [a0, a0 + 3560), CIS 1 in [a0 + 3560, a0 + 7120); ACL 1 and ACL 2 (with 300 us of preparation before
	 * the anchor and 1000 us of event) must not fall into them */
	const uint32_t acl1 = F.timing.anchor_us % 10000u, acl2 = F.timing2.anchor_us % 10000u;
	CHECK_EQ(circ_overlap(a0, 7120, acl1 + 10000u - 300u, 1300, 10000), 0);
	CHECK_EQ(circ_overlap(a0, 7120, acl2 + 10000u - 300u, 1300, 10000), 0);
}

/* Margin after the end of the ACL event (cis_radio_caps_t.acl_margin_us): an ACL event with data (earbud notifications)
 * lasts longer than the planned end - a CIS right behind it (1000 us on the Tab5) lost more than with the offset
 * of the earbud 1312 us. The first CIS sits at least an ACL event + margin after its ACL and outside the second ACL. */
static void test_cig_placement_acl_margin(void)
{
	cis_engine_t *e = fresh();
	if (!e)
		return;
	F.caps.cis_gap_us = 2500;
	F.caps.acl_margin_us = 500;
	host_feature(e);
	static const uint8_t cig2[33] = {
		0x01, 0x10, 0x27, 0x00, 0x10, 0x27, 0x00, 0x00, 0x00, 0x00, 0x0A, 0x00, 0x0A, 0x00, 0x02,
		0x00, 0x78, 0x00, 0x28, 0x00, 0x02, 0x02, 0x00, 0x00,
		0x01, 0x78, 0x00, 0x28, 0x00, 0x02, 0x02, 0x00, 0x00,
	};
	CHECK(cmd(e, CIS_OP_LE_SET_CIG_PARAMS, cig2, 33));
	F.timing2 = F.timing;
	F.timing2.anchor_us += 1000;   /* ACL 2 1 ms after ACL 1: both events with margin are [-300, 2500) */
	F.timing2_on = true;
	uint8_t p[9] = { 2, 0x00, 0x0E, 0x01, 0x00, 0x01, 0x0E, 0x02, 0x00 };
	CHECK(cmd(e, CIS_OP_LE_CREATE_CIS, p, 9));
	cis_ll_cis_req_t r0;
	memset(&r0, 0, sizeof(r0));
	CHECK(F.n_ctrl == 1 && cis_ll_cis_req_parse(F.ctrl[0], F.ctrl_len[0], &r0));
	CHECK_EQ(r0.cis_offset_min_us, r0.cis_offset_max_us);
	CHECK(r0.cis_offset_min_us >= 1000u + 500u);   /* ACL event (margin 1000) + 500 */
}

/* A radio with NSE 2 only for a CIG without P->C data (cis_radio_caps_t.max_nse_p_to_c - C6 until a microphone trial):
 * RTN 1 gives music alone NSE 2, while a CIG with a microphone (Max_SDU P->C > 0) stays at NSE 1. */
static void test_nse_limit_with_p_to_c(void)
{
	for (int mic = 0; mic < 2; mic++) {
		cis_engine_t *e = fresh();
		if (!e)
			return;
		F.caps.max_nse = 2;
		F.caps.max_nse_p_to_c = 1;
		encrypt_acl(e, 0x0001);
		host_feature(e);
		uint8_t c[24];
		memcpy(c, CIG1, 24);
		c[22] = 1;                       /* RTN C->P 1: two attempts per SDU */
		if (!mic)
			cis_put_le16(&c[18], 0);     /* without Max_SDU P->C */
		CHECK(cmd(e, CIS_OP_LE_SET_CIG_PARAMS, c, 24));
		CHECK(create_cis(e, 0x0E00, 0x0001));
		CHECK_EQ(F.ctrl[0][19], (mic ? 1 : 2));   /* NSE in LL_CIS_REQ */
	}
}

/* Buds3 Pro answers an exact CIS_Offset beyond the ISO interval with the same instant written modulo ISO (10694 ->
 * 694). This is the same anchor in the CIG phase - we accept it (previously choose_instant rejected it after the
 * window clipping: 0x20). */
static void test_peer_offset_mod_iso(void)
{
	cis_engine_t *e = NULL;
	uint32_t off1 = 0;
	F.caps.cis_gap_us = 0;
	if (!two_cis_streaming(&e, &off1))
		return;
	CHECK(e != NULL);
	/* a new CIG with two CISes: the second REQ gets an exact offset X, the earbud answers X -+ 10000 */
	cis_engine_t *e2 = fresh();
	if (!e2)
		return;
	F.caps.cis_gap_us = 2500;
	host_feature(e2);
	static const uint8_t cig2[33] = {
		0x01, 0x10, 0x27, 0x00, 0x10, 0x27, 0x00, 0x00, 0x00, 0x00, 0x0A, 0x00, 0x0A, 0x00, 0x02,
		0x00, 0x78, 0x00, 0x28, 0x00, 0x02, 0x02, 0x00, 0x00,
		0x01, 0x78, 0x00, 0x28, 0x00, 0x02, 0x02, 0x00, 0x00,
	};
	CHECK(cmd(e2, CIS_OP_LE_SET_CIG_PARAMS, cig2, 33));
	F.timing2 = F.timing;
	F.timing2.anchor_us += 3000;
	F.timing2_on = true;
	uint8_t p[9] = { 2, 0x00, 0x0E, 0x01, 0x00, 0x01, 0x0E, 0x02, 0x00 };
	CHECK(cmd(e2, CIS_OP_LE_CREATE_CIS, p, 9));
	cis_ll_cis_req_t r0;
	memset(&r0, 0, sizeof(r0));
	CHECK(F.n_ctrl == 1 && cis_ll_cis_req_parse(F.ctrl[0], F.ctrl_len[0], &r0));
	rsp_from_peer(e2, r0.cis_offset_min_us, r0.cis_offset_max_us, r0.conn_event_count);
	const uint32_t a0 = F.lp.first_anchor_us;
	cis_ll_cis_req_t r1;
	memset(&r1, 0, sizeof(r1));
	CHECK(F.n_ctrl == 3 && cis_ll_cis_req_parse(F.ctrl[2], F.ctrl_len[2], &r1));
	const uint32_t x = r1.cis_offset_min_us;
	const uint32_t y = x >= 10500u ? x - 10000u : x + 10000u;
	cis_ll_cis_rsp_t r = { y, y, r1.conn_event_count };
	uint8_t b[9];
	cis_ll_cis_rsp_pack(&r, b);
	CHECK(cis_engine_ll_ctrl_rx(e2, 0x0002, b, 9));
	cis_ll_cis_ind_t i1;
	CHECK(F.n_ctrl == 4 && cis_ll_cis_ind_parse(F.ctrl[3], F.ctrl_len[3], &i1));
	CHECK_EQ(i1.cis_offset_us, y);
	CHECK_EQ((F.lp.first_anchor_us - a0) % 10000u, 3560);
}

/* CIS_Offset_Max within one ISO interval only for the CIS that fixes the CIG phase. When another CIS is already
 * established, the new one gets an offset in the CIG phase from the whole connInterval window (previously only the
 * lower index counted - a repeated Create CIS for CIS 0 with CIS 1 established limited the window and choose_instant
 * rejected it). */
static void test_offset_cap_phase_setter(void)
{
	cis_engine_t *e = fresh();
	if (!e)
		return;
	host_feature(e);
	static const uint8_t cig2[33] = {
		0x01, 0x10, 0x27, 0x00, 0x10, 0x27, 0x00, 0x00, 0x00, 0x00, 0x0A, 0x00, 0x0A, 0x00, 0x02,
		0x00, 0x78, 0x00, 0x28, 0x00, 0x02, 0x02, 0x00, 0x00,
		0x01, 0x78, 0x00, 0x28, 0x00, 0x02, 0x02, 0x00, 0x00,
	};
	CHECK(cmd(e, CIS_OP_LE_SET_CIG_PARAMS, cig2, 33));
	/* CIS 1 alone on ACL 2: fixes the phase - a limited window (CIG_Sync_Delay 2 x 1060) */
	const uint8_t p1[5] = { 1, 0x01, 0x0E, 0x02, 0x00 };
	CHECK(cmd(e, CIS_OP_LE_CREATE_CIS, p1, 5));
	CHECK_EQ(last_cmd_status(), 0);
	cis_ll_cis_req_t r;
	memset(&r, 0, sizeof(r));
	CHECK(F.n_ctrl == 1 && cis_ll_cis_req_parse(F.ctrl[0], F.ctrl_len[0], &r));
	CHECK_EQ(r.cis_offset_max_us, 10000u - 2120u - 1000u);
	cis_ll_cis_rsp_t rsp = { 1000, 6880, 106 };
	uint8_t b[9];
	cis_ll_cis_rsp_pack(&rsp, b);
	CHECK(cis_engine_ll_ctrl_rx(e, 0x0002, b, 9));
	CHECK(r_prep(0));
	cis_pdu_t n = peer_null(0, false);
	r_rx(0, &n, true);
	r_close(0);
	cis_engine_process(e);
	const uint8_t *est = last_evt(0x3E, 0x19, NULL);
	CHECK(est && est[4] == 0x00);
	const uint32_t a1 = F.lp.first_anchor_us;
	/* CIS 0 on ACL 1 with CIS 1 established: a single value in the CIG phase (CIS 0 1060 us before CIS 1), also from
	 * outside the first ISO interval (window up to connInterval - CIG_Sync_Delay - margin) */
	const uint8_t p0[5] = { 1, 0x00, 0x0E, 0x01, 0x00 };
	CHECK(cmd(e, CIS_OP_LE_CREATE_CIS, p0, 5));
	CHECK_EQ(last_cmd_status(), 0);
	memset(&r, 0, sizeof(r));
	CHECK(F.n_ctrl == 3 && cis_ll_cis_req_parse(F.ctrl[2], F.ctrl_len[2], &r));
	CHECK_EQ(r.cis_offset_min_us, r.cis_offset_max_us);
	CHECK(r.cis_offset_max_us >= 1000 && r.cis_offset_max_us <= 50000u - 2120u - 1000u);
	const uint32_t acl1 = F.timing.anchor_us +
		(uint32_t)(uint16_t)(r.conn_event_count - F.timing.event_counter) * F.timing.interval_us;
	CHECK_EQ((acl1 + r.cis_offset_min_us + 1060u - a1) % 10000u, 0);
}

/* Loss of the ACL of the first CIS while it waits for LL_CIS_RSP: the second one (on another ACL) gets its
 * LL_CIS_REQ right away, not after the timeout (one LL_CIS_REQ at a time in a CIG - acl_lost must release the queue
 * too). */
static void test_acl_loss_releases_next_req(void)
{
	cis_engine_t *e = fresh();
	if (!e)
		return;
	host_feature(e);
	static const uint8_t cig2[33] = {
		0x01, 0x10, 0x27, 0x00, 0x10, 0x27, 0x00, 0x00, 0x00, 0x00, 0x0A, 0x00, 0x0A, 0x00, 0x02,
		0x00, 0x78, 0x00, 0x28, 0x00, 0x02, 0x02, 0x00, 0x00,
		0x01, 0x78, 0x00, 0x28, 0x00, 0x02, 0x02, 0x00, 0x00,
	};
	CHECK(cmd(e, CIS_OP_LE_SET_CIG_PARAMS, cig2, 33));
	uint8_t p[9] = { 2, 0x00, 0x0E, 0x01, 0x00, 0x01, 0x0E, 0x02, 0x00 };
	CHECK(cmd(e, CIS_OP_LE_CREATE_CIS, p, 9));
	CHECK_EQ(F.n_ctrl, 1);
	const uint8_t dc[4] = { 0x00, 0x01, 0x00, 0x08 };   /* Disconnection Complete ACL 1, supervision */
	CHECK(!evt(e, dc, 4, 0x05));
	CHECK_EQ(F.n_ctrl, 2);
	CHECK_EQ(F.ctrl_acl[1], 0x0002);
	CHECK_EQ(F.ctrl[1][0], CIS_LL_CIS_REQ);
}

/* FT 2, NSE 2: an empty payload at the head of the queue goes out as a padding PDU (LLID 0b01, length 0, NPI 0), gets
 * acknowledged and does not block the next one - data goes out in its first event, without a constant delay. */
/* Jitter buffer from Max_Transport_Latency (cis_cig_derive): at 10 ms (CIG1) 2 SDUs before the first transmission, at
 * 40 ms - 3. Without radio events the host can insert SDUs for events cur+1+depth .. cur+1+2*depth, that is
 * depth + 1 SDUs; the rest is "host too fast". */
static void test_jitter_from_latency(void)
{
	static const uint8_t lat_ms[2] = { 10, 40 };
	static const uint32_t want_in[2] = { 3, 4 };
	uint8_t sdu[120];
	memset(sdu, 0x5A, sizeof(sdu));

	for (int k = 0; k < 2; k++) {
		uint8_t cig[sizeof(CIG1)];
		memcpy(cig, CIG1, sizeof(cig));
		cig[10] = lat_ms[k];   /* Max_Transport_Latency_C_To_P, u16 LE */
		cig[11] = 0;
		g_cig = cig;
		est_t s;
		const bool ok = establish(&s);
		g_cig = CIG1;
		if (!ok)
			return;
		setup_paths(s.e);
		for (uint16_t i = 0; i < 6; i++)
			send_sdu(s.e, i, sdu, sizeof(sdu));
		cis_engine_stats_t st;
		CHECK(cis_engine_get_stats(s.e, 0x0E00, &st));
		CHECK_EQ(st.sdu_in, want_in[k]);
		CHECK_EQ(st.sdu_dropped, 6u - want_in[k]);
	}
}

/* Lower bound of the jitter buffer from the radio (cis_radio_caps_t.jitter_min_sdu): Tab5, 2026-11-01 - at NSE 2 FT
 * 4 consumed the Max_Transport_Latency budget (40 ms) and left 2 SDUs (20 ms), and SDIO P4 -> C6 stalls of up to 14
 * ms gave 0.6-0.8 % late SDUs (sdu_late, clipping). Bound 4: at 10 ms (derived 2) the host inserts 4 + 1 SDUs. It
 * does not grow beyond CIS_JITTER_MAX. */
static void test_jitter_floor_from_radio(void)
{
	static const uint8_t floor_sdu[3] = { 0, 4, 9 };
	static const uint32_t want_in[3] = { 3, 5, 5 };
	uint8_t sdu[120];
	memset(sdu, 0x5A, sizeof(sdu));
	for (int k = 0; k < 3; k++) {
		g_jitter_floor = floor_sdu[k];
		est_t s;
		const bool ok = establish(&s);   /* CIG1: Max_Transport_Latency 10 ms - derived 2 SDUs */
		g_jitter_floor = 0;
		if (!ok)
			return;
		setup_paths(s.e);
		for (uint16_t i = 0; i < 8; i++)
			send_sdu(s.e, i, sdu, sizeof(sdu));
		cis_engine_stats_t st;
		CHECK(cis_engine_get_stats(s.e, 0x0E00, &st));
		CHECK_EQ(st.sdu_in, want_in[k]);
	}
}

/* Host credits from cis_radio_caps_t.tx_slots (the C6 tx_slots switch): 0 = 12 per CIS as before, 16 = 32 credits,
 * out of range clamped to 12..16. A larger pool does not change the jitter buffer (the same SDU acceptance). */
static void test_tx_slots_from_radio(void)
{
	static const uint8_t slots[5] = { 0, 12, 16, 40, 5 };
	static const uint8_t credits[5] = { 24, 24, 32, 32, 24 };
	uint8_t sdu[120];
	memset(sdu, 0x6B, sizeof(sdu));
	for (int k = 0; k < 5; k++) {
		g_tx_slots = slots[k];
		est_t s;
		const bool ok = establish(&s);
		g_tx_slots = 0;
		if (!ok)
			return;
		CHECK(cmd(s.e, CIS_OP_LE_READ_BUF_SIZE_V2, NULL, 0));
		const uint8_t *p = last_evt(0x0E, 0, NULL);
		CHECK(p != NULL);
		if (p)
			CHECK_EQ(p[12], credits[k]);
		/* host without radio events: the queue reaches 1 + 2 x jitter buffer (CIG1: 2 SDUs) regardless of the pool */
		setup_paths(s.e);
		for (uint16_t i = 0; i < 8; i++)
			send_sdu(s.e, i, sdu, sizeof(sdu));
		cis_engine_stats_t st;
		CHECK(cis_engine_get_stats(s.e, 0x0E00, &st));
		CHECK_EQ(st.sdu_in, 3);
	}
}

static void test_ft2_padding(void)
{
	cis_engine_t *e = fresh();
	if (!e)
		return;
	F.caps.max_nse = 2;
	host_feature(e);
	uint8_t c[24];
	memcpy(c, CIG1, 24);
	c[10] = 20;      /* Max_Transport_Latency_C_To_P 20 ms */
	c[12] = 20;
	c[22] = 2;       /* RTN_C_To_P */
	CHECK(cmd(e, CIS_OP_LE_SET_CIG_PARAMS, c, 24));
	CHECK(create_cis(e, 0x0E00, 0x0001));
	CHECK_EQ(F.ctrl[0][19], 2);    /* NSE */
	CHECK_EQ(F.ctrl[0][24], 2);    /* FT_C_To_P */
	rsp_from_peer(e, 1000, 46000, 106);
	/* Peripheral: the expected SN, acceptance toggles NESN (flushes will not occur - it accepts everything) */
	bool pn_nesn = false;
	for (uint8_t se = 0; se < 2; se++) {
		CHECK(F.cb->prepare_tx(F.cb_arg, 0, se, &g_tx));
		CHECK(g_tx.hdr0 & CIS_HDR_NPI);   /* before the stream starts: a plain null PDU */
		cis_pdu_t n;
		memset(&n, 0, sizeof(n));
		n.hdr0 = CIS_LLID_START_CONT | CIS_HDR_NPI;
		F.cb->rx_done(F.cb_arg, 0, se, &n, true, F.now);
	}
	r_close(0);
	cis_engine_process(e);
	setup_paths(e);
	uint8_t sdu[120];
	memset(sdu, 0x77, sizeof(sdu));
	send_sdu(e, 0, sdu, 120);
	int pads = 0, nulls = 0;
	for (uint32_t ev = 1; ev <= 3; ev++) {
		for (uint8_t se = 0; se < 2; se++) {
			CHECK(F.cb->prepare_tx(F.cb_arg, ev, se, &g_tx));
			bool npi = g_tx.hdr0 & CIS_HDR_NPI;
			if (ev == 3 && se == 0) {
				CHECK_EQ(g_tx.len, 120);
				CHECK(!npi);
			} else if (!npi) {
				CHECK_EQ(g_tx.len, 0);
				CHECK_EQ(g_tx.hdr0 & 0x03, CIS_LLID_START_CONT);
				pads++;
			} else {
				nulls++;
			}
			if (!npi && ((g_tx.hdr0 & CIS_HDR_SN) != 0) == pn_nesn)
				pn_nesn = !pn_nesn;
			cis_pdu_t n;
			memset(&n, 0, sizeof(n));
			n.hdr0 = (uint8_t)(CIS_LLID_START_CONT | CIS_HDR_NPI | (pn_nesn ? CIS_HDR_NESN : 0));
			F.cb->rx_done(F.cb_arg, ev, se, &n, true, F.now);
		}
		r_close(ev);
		cis_engine_process(e);
	}
	CHECK_EQ(pads, 3);    /* payloads 0, 1, 2 */
	cis_engine_stats_t st;
	CHECK(cis_engine_get_stats(e, 0x0E00, &st));
	CHECK_EQ(st.tx_acked, 1);
	CHECK_EQ(st.tx_flushed, 0);
	CHECK_EQ(st.tx_underrun, 0);   /* payloads before the first SDU are not an underrun */
}

int main(void)
{
	test_init_and_snoop();
	test_main_flow();
	test_reject_paths();
	test_establish_timeout();
	test_data_length_gate();
	test_acl_event_len_guard();
	test_peer_terminate_and_acl_loss();
	test_trace_steps();
	test_mic_failure();
	test_controller_keys();
	test_radio_owns_seq();
	test_hci_validation();
	test_cancel_pending();
	test_bn2_segmentation();
	test_two_cis_phase();
	test_two_cis_gap();
	test_two_cis_sdu_aligned();
	test_two_cis_burst_recovers();
	test_single_late_keeps_mapping();
	test_queue_recenters();
	test_drift_shifts_mapping();
	test_cig_placement_avoids_acls();
	test_cig_placement_acl_margin();
	test_nse_limit_with_p_to_c();
	test_peer_offset_mod_iso();
	test_offset_cap_phase_setter();
	test_acl_loss_releases_next_req();
	test_ft2_padding();
	test_jitter_from_latency();
	test_jitter_floor_from_radio();
	test_tx_slots_from_radio();
	TEST_DONE("test_engine");
}
