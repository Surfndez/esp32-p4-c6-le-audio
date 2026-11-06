/*
 * Model of the ACL connections Central (C6) <-> Peripheral and of the "rest of the controller" of the C6 (what the
 * CIS engine does not handle): LE Enable Encryption and the encryption procedure, Disconnect ACL, LE Read Local
 * Supported Features, connection supervision. One PDU exchange per ACL event, SN/NESN as in Core (retransmission in
 * the next event), the LLCP response in the next event at the earliest.
 */
#include "sim.h"

#include <string.h>

#include "cis_engine.h"

#define CQ_MAX 16
#define CLAIMS 64

typedef struct {
	uint8_t len;
	uint8_t b[64];
	bool    from_engine;
	bool    seen;
} cpdu_t;

typedef enum { CENC_OFF, CENC_WAIT_RSP, CENC_WAIT_START_REQ, CENC_WAIT_START_RSP, CENC_ON } cenc_t;

typedef struct {
	uint16_t  handle;
	periph_t *p;
	bool      connected;
	int64_t   anchor0_us; /* local C6 clock, us, without wraparound */
	uint32_t  interval_us;
	uint32_t  crc_init, aa;
	uint8_t   chan_map[5];
	cpdu_t    q[CQ_MAX];
	int       qh, qn;
	bool      c_sn, c_nesn, c_pend, c_data; /* c_pend: the last PDU without ACK, c_data: non-empty */
	bool      p_sn, p_nesn, p_pend, p_data;
	sim_ns_t  last_rx;
	cenc_t    enc;
	bool      lost_applied;
	bool      dle_done;
} acl_t;

typedef struct {
	int      owner;
	sim_ns_t s, e;
	bool     is_acl;
} claim_t;

static acl_t   s_acl[MAX_BUDS];
static int     s_n;
static claim_t s_claims[CLAIMS];
static unsigned s_nclaims;
static rng_t   s_rng;

static const uint8_t k_chan_map[5] = { 0xFF, 0xFF, 0xFF, 0xFF, 0x1F };

static uint32_t reserve_us(void)
{
	return 2 * air_time_us(g_cfg.acl_phy, g_cfg.acl_reserve_octets + 4u) + T_IFS_US;
}

static void acl_event(void *ctx, uint64_t n);

static void emit_conn_complete(void *ctx, uint64_t arg)
{
	(void)arg;
	acl_t *a = ctx;
	uint8_t ev[3 + 19] = { 0x04, 0x3E, 19, 0x01, 0x00 };
	put16(ev + 5, a->handle);
	ev[7] = 0x00; /* role: Central */
	ev[8] = 0x00;
	for (int i = 0; i < 6; i++)
		ev[9 + i] = (uint8_t)(0x10 * (a - s_acl) + i + 1);
	put16(ev + 15, a->interval_us / 1250);
	put16(ev + 17, 0);
	put16(ev + 19, g_cfg.sup_timeout_ms / 10);
	ev[21] = 0;
	plat_ctrl_evt(ev, sizeof(ev));
}

/* The controller completes the Data Length Update and reports LE Data Length Change (the engine snoops it). */
static void emit_dle(void *ctx, uint64_t arg)
{
	(void)arg;
	acl_t *a = ctx;
	if (!a->connected)
		return;
	a->dle_done = true;
	uint16_t t = (uint16_t)((g_cfg.acl_max_octets + 14) * 8);
	uint8_t ev[3 + 11] = { 0x04, 0x3E, 11, 0x07 };
	put16(ev + 4, a->handle);
	put16(ev + 6, g_cfg.acl_max_octets);
	put16(ev + 8, t);
	put16(ev + 10, g_cfg.acl_max_octets);
	put16(ev + 12, t);
	slog("C6", "ACL 0x%04x LE Data Length Change: MaxTxOctets %u", a->handle, g_cfg.acl_max_octets);
	plat_ctrl_evt(ev, sizeof(ev));
}

void acl_init(void)
{
	memset(s_acl, 0, sizeof(s_acl));
	s_nclaims = 0;
	rng_seed(&s_rng, g_cfg.seed * 13 + 5);
	s_n = g_cfg.n_buds;
	for (int i = 0; i < s_n; i++) {
		acl_t *a = &s_acl[i];
		a->handle = (uint16_t)(0x0001 + i);
		a->p = NULL;
		a->connected = true;
		a->interval_us = g_cfg.acl_interval_us;
		a->crc_init = (uint32_t)rng_u64(&s_rng) & 0xFFFFFF;
		do {
			a->aa = (uint32_t)rng_u64(&s_rng);
		} while (aa_check(a->aa));
		memcpy(a->chan_map, k_chan_map, 5);
		sim_ns_t t0 = MS(5) + US((int64_t)i * g_cfg.acl2_offset_us);
		a->anchor0_us = plat_c_local_us64(t0);
		a->last_rx = 0;
		uint8_t ltk[16];
		for (int k = 0; k < 16; k++)
			ltk[k] = (uint8_t)(0x40 + k);
		sim_clock_t pc = { (int64_t)g_cfg.ppm_p[i] * 1000, (int64_t)rng_range(&s_rng, 0, SEC(1000)) };
		a->p = periph_new(i, &pc, g_cfg.seed * 17 + (uint64_t)i, ltk);
		periph_acl_set(a->p, a->interval_us, a->chan_map, a->aa);
		sim_at(MS(1) + US(200) * i, emit_conn_complete, a, 0);
		if (g_cfg.acl_dle_at > 0)
			sim_at(g_cfg.acl_dle_at + MS(30) * i, emit_dle, a, 0);
		else
			a->dle_done = true;
		sim_at(plat_c_true_of_us64(a->anchor0_us), acl_event, a, 0);
	}
}

int acl_count(void)
{
	return s_n;
}

uint16_t acl_handle(int i)
{
	return s_acl[i].handle;
}

periph_t *acl_periph(int i)
{
	return s_acl[i].p;
}

int acl_find(uint16_t handle)
{
	for (int i = 0; i < s_n; i++)
		if (s_acl[i].handle == handle)
			return i;
	return -1;
}

bool acl_connected(uint16_t handle)
{
	int i = acl_find(handle);
	return i >= 0 && s_acl[i].connected;
}

static void cq_push(acl_t *a, const uint8_t *b, uint8_t len, bool from_engine)
{
	if (a->qn == CQ_MAX) {
		slog("acl", "BUG: central LLCP queue full");
		return;
	}
	cpdu_t *c = &a->q[(a->qh + a->qn++) % CQ_MAX];
	memcpy(c->b, b, len);
	c->len = len;
	c->from_engine = from_engine;
	c->seen = false;
}

int acl_send_from_engine(uint16_t handle, const uint8_t *ctr, uint8_t len)
{
	int i = acl_find(handle);
	if (i < 0 || !s_acl[i].connected) {
		viol(V_CONTRACT, "C6", "send_ctrl_pdu on unknown/disconnected ACL 0x%04x", handle);
		return -1;
	}
	if (len < 1 || len > 64) {
		viol(V_CONTRACT, "C6", "send_ctrl_pdu length %u", len);
		return -1;
	}
	uint8_t max_tx = s_acl[i].dle_done ? g_cfg.acl_max_octets : 27;
	if (len > max_tx) {
		g_m.eng_too_long++;
		slog("C6", "send_ctrl_pdu 0x%02x len %u > MaxTxOctets %u -> CIS_PLAT_ERR_TOO_LONG", ctr[0], len, max_tx);
		return CIS_PLAT_ERR_TOO_LONG;
	}
	slog("C6", "engine LLCP tx 0x%02x len %u on 0x%04x", ctr[0], len, handle);
	cq_push(&s_acl[i], ctr, len, true);
	return 0;
}

static int64_t next_event(const acl_t *a, int64_t now_us, uint16_t *evt)
{
	int64_t d = now_us - a->anchor0_us;
	int64_t n = d < 0 ? 0 : d / a->interval_us + 1;
	*evt = (uint16_t)n;
	return a->anchor0_us + n * a->interval_us;
}

int acl_timing(uint16_t handle, cis_acl_timing_t *out)
{
	int i = acl_find(handle);
	if (i < 0 || !s_acl[i].connected)
		return -1;
	acl_t *a = &s_acl[i];
	int64_t at = next_event(a, plat_c_local_us64(sim_now()), &out->event_counter);
	out->anchor_us = (uint32_t)at;
	out->interval_us = a->interval_us;
	out->crc_init = a->crc_init;
	memcpy(out->chan_map, a->chan_map, 5);
	out->phy_tx = g_cfg.acl_phy;
	out->phy_rx = g_cfg.acl_phy;
	out->peripheral_latency = 0;
	out->event_len_us = reserve_us(); /* the same reservation that acl_window_overlap checks */
	return 0;
}

uint64_t acl_remote_features(uint16_t handle)
{
	if (acl_find(handle) < 0)
		return 0;
	/* LE Encryption (0), CIS Peripheral (29), CIS Host Support (32). */
	return (1ull << 0) | (1ull << 29) | (1ull << 32);
}

/* ---------------- C6 radio reservations ---------------- */
bool radio_claim(int owner, sim_ns_t s, sim_ns_t e, bool is_acl)
{
	bool ok = true;
	unsigned n = s_nclaims < CLAIMS ? s_nclaims : CLAIMS;
	for (unsigned i = 0; i < n; i++) {
		const claim_t *c = &s_claims[i];
		if (c->owner == owner)
			continue;
		if (c->s < e && s < c->e) {
			ok = false;
			if (is_acl || c->is_acl)
				g_m.overlap_acl++;
			else
				g_m.overlap_cis++;
			slog("radio", "overlap: owner %d [%lld..%lld] vs owner %d [%lld..%lld]", owner, (long long)s,
					(long long)e, c->owner, (long long)c->s, (long long)c->e);
			break;
		}
		sim_ns_t gap = s >= c->e ? s - c->e : c->s - e;
		if (gap >= 0 && gap < US(T_MSS_US))
			g_m.tight_gap++;
	}
	if (ok || is_acl) {
		claim_t *c = &s_claims[s_nclaims++ % CLAIMS];
		c->owner = owner;
		c->s = s;
		c->e = e;
		c->is_acl = is_acl;
	}
	return ok;
}

bool acl_window_overlap(sim_ns_t s, sim_ns_t e)
{
	int64_t ls = plat_c_local_us64(s);
	for (int i = 0; i < s_n; i++) {
		const acl_t *a = &s_acl[i];
		if (!a->connected)
			continue;
		int64_t n = (ls - a->anchor0_us) / a->interval_us;
		for (int64_t k = n - 1; k <= n + 1; k++) {
			if (k < 0)
				continue;
			sim_ns_t as = plat_c_true_of_us64(a->anchor0_us + k * a->interval_us);
			sim_ns_t ae = as + US(reserve_us());
			if (as < e && s < ae) {
				g_m.overlap_acl++;
				slog("radio", "CIS subevent [%lld..%lld] hits ACL 0x%04x event %lld reservation", (long long)s,
						(long long)e, a->handle, (long long)k);
				return true;
			}
		}
	}
	return false;
}

/* ---------------- the rest of the C6 controller ---------------- */
static void cmd_complete(uint16_t op, const uint8_t *ret, uint8_t rlen)
{
	uint8_t ev[64] = { 0x04, 0x0E, (uint8_t)(3 + rlen), 1 };
	put16(ev + 4, op);
	memcpy(ev + 6, ret, rlen);
	plat_ctrl_evt(ev, (uint16_t)(6 + rlen));
}

static void cmd_status(uint16_t op, uint8_t status)
{
	uint8_t ev[7] = { 0x04, 0x0F, 4, status, 1 };
	put16(ev + 5, op);
	plat_ctrl_evt(ev, 7);
}

static void disc_complete(uint16_t handle, uint8_t reason)
{
	uint8_t ev[7] = { 0x04, 0x05, 4, 0x00 };
	put16(ev + 4, handle);
	ev[6] = reason;
	plat_ctrl_evt(ev, 7);
}

static void acl_drop(acl_t *a, uint8_t reason)
{
	if (!a->connected)
		return;
	a->connected = false;
	slog("C6", "ACL 0x%04x disconnected, reason 0x%02x", a->handle, reason);
	periph_acl_lost(a->p);
	disc_complete(a->handle, reason);
}

void ctrl_hci_cmd(const uint8_t *h4, uint16_t len)
{
	if (len < 4)
		return;
	uint16_t op = get16(h4 + 1);
	const uint8_t *pr = h4 + 4;
	slog("C6", "controller handles HCI 0x%04x", op);
	switch (op) {
	case 0x0C03: { /* Reset */
		uint8_t st = 0;
		cmd_complete(op, &st, 1);
		break;
	}
	case 0x2003: { /* LE Read Local Supported Features: the C6 without ISO bits */
		uint8_t r[9] = { 0 };
		put64(r + 1, 0x00000000000F79FFull & ~((1ull << 28) | (1ull << 29)));
		cmd_complete(op, r, 9);
		break;
	}
	case 0x2019: { /* LE Enable Encryption */
		int i = acl_find(get16(pr));
		if (i < 0 || !s_acl[i].connected) {
			cmd_status(op, 0x02);
			break;
		}
		acl_t *a = &s_acl[i];
		cmd_status(op, 0x00);
		uint8_t b[23] = { LL_ENC_REQ };
		memcpy(b + 1, pr + 2, 8);   /* Rand */
		memcpy(b + 9, pr + 10, 2);  /* EDIV */
		rng_bytes(&s_rng, b + 11, 8); /* SKDc */
		rng_bytes(&s_rng, b + 19, 4); /* IVc */
		cq_push(a, b, 23, false);
		a->enc = CENC_WAIT_RSP;
		break;
	}
	case 0x0406: { /* Disconnect */
		int i = acl_find(get16(pr));
		if (i < 0 || !s_acl[i].connected) {
			cmd_status(op, 0x02);
			break;
		}
		cmd_status(op, 0x00);
		uint8_t b[2] = { LL_TERMINATE_IND, pr[2] };
		cq_push(&s_acl[i], b, 2, false);
		break;
	}
	default: {
		uint8_t st = 0x01; /* Unknown HCI Command */
		cmd_complete(op, &st, 1);
		break;
	}
	}
}

/* A PDU from the Peripheral to the Central: the engine first, then the rest of the controller. */
static void central_rx_pdu(acl_t *a, const uint8_t *ctr, uint8_t len)
{
	plat_ll_ctrl_rx(a->handle, ctr, len);
}

void ctrl_ll_rx(uint16_t handle, const uint8_t *ctr, uint8_t len)
{
	int i = acl_find(handle);
	if (i < 0)
		return;
	acl_t *a = &s_acl[i];
	switch (ctr[0]) {
	case LL_ENC_RSP:
		if (a->enc == CENC_WAIT_RSP)
			a->enc = CENC_WAIT_START_REQ;
		break;
	case LL_START_ENC_REQ:
		if (a->enc == CENC_WAIT_START_REQ) {
			uint8_t b = LL_START_ENC_RSP;
			cq_push(a, &b, 1, false);
			a->enc = CENC_WAIT_START_RSP;
		}
		break;
	case LL_START_ENC_RSP:
		if (a->enc == CENC_WAIT_START_RSP) {
			a->enc = CENC_ON;
			uint8_t ev[7] = { 0x04, 0x08, 4, 0x00 };
			put16(ev + 4, a->handle);
			ev[6] = 0x01;
			plat_ctrl_evt(ev, 7);
		}
		break;
	case LL_TERMINATE_IND:
		acl_drop(a, ctr[1]);
		break;
	case LL_UNKNOWN_RSP:
	case LL_FEATURE_RSP:
		break;
	case LL_CIS_REQ:
	case LL_CIS_RSP:
	case LL_CIS_IND:
	case LL_CIS_TERMINATE_IND:
	case LL_REJECT_EXT_IND:
		/* A C6 controller without ISO answers CIS opcodes it does not know. */
		slog("C6", "engine did not consume LLCP 0x%02x (len %u) - controller sends LL_UNKNOWN_RSP", ctr[0], len);
		if (ctr[0] != LL_REJECT_EXT_IND) {
			uint8_t b[2] = { LL_UNKNOWN_RSP, ctr[0] };
			cq_push(a, b, 2, false);
		}
		break;
	default: {
		uint8_t b[2] = { LL_UNKNOWN_RSP, ctr[0] };
		cq_push(a, b, 2, false);
		break;
	}
	}
}

typedef struct {
	acl_t  *a;
	uint8_t len;
	uint8_t b[64];
} deliver_t;

#define DQ 32
static deliver_t s_dq[DQ];
static unsigned s_dq_w;

static void deliver_cb(void *ctx, uint64_t arg)
{
	(void)arg;
	deliver_t *d = ctx;
	if (d->a->connected)
		central_rx_pdu(d->a, d->b, d->len);
}

static void acl_event(void *ctx, uint64_t n)
{
	acl_t *a = ctx;
	if (!a->connected)
		return;
	sim_ns_t t = sim_now();
	int idx = (int)(a - s_acl);
	if (g_cfg.acl_loss_at && idx == 0 && t >= g_cfg.acl_loss_at && !a->lost_applied) {
		a->lost_applied = true;
		slog("acl", "earbud 0 vanishes (scenario)");
		periph_acl_lost(a->p);
	}
	if (a->last_rx == 0)
		a->last_rx = t;
	if (t - a->last_rx > MS(g_cfg.sup_timeout_ms)) {
		acl_drop(a, 0x08);
		return;
	}
	sim_at(plat_c_true_of_us64(a->anchor0_us + (int64_t)(n + 1) * a->interval_us), acl_event, a, n + 1);

	/* Core: without an acknowledgement we send the same PDU (also an empty one) with the same SN; new content only
	 * after an ACK. */
	if (!a->c_pend) {
		a->c_data = a->qn > 0;
		a->c_pend = true;
	}
	cpdu_t *head = a->c_data ? &a->q[a->qh] : NULL;
	uint8_t c_len = head ? head->len : 0;
	unsigned mic = (a->enc == CENC_ON && c_len) ? 4 : 0;
	sim_ns_t c_end = t + US(air_time_us(g_cfg.acl_phy, c_len + mic));
	if (air_coex_blocked(t, c_end + US(T_IFS_US + 300))) {
		g_m.coex_blocked++;
		return;
	}
	if (head && !head->seen) {
		head->seen = true;
		if (!head->from_engine)
			plat_ll_ctrl_tx_seen(a->handle, head->b, head->len);
	}
	bool c_sn = a->c_sn, c_nesn = a->c_nesn;

	periph_t *p = a->p;
	air_res_t r1 = air_packet();
	if (a->lost_applied)
		r1 = AIR_MISS;
	if (r1 == AIR_MISS) {
		radio_claim(100 + idx, t, c_end + US(T_IFS_US + 50), true);
		return;
	}
	if (r1 == AIR_OK) {
		periph_acl_sync(p, (uint16_t)n, t);
		if (c_nesn != a->p_sn) {
			if (a->p_pend && a->p_data)
				periph_acl_tx_pop(p);
			a->p_pend = false;
			a->p_sn = !a->p_sn;
		}
	}
	const uint8_t *pb = NULL;
	uint8_t p_len = 0;
	uint8_t p_copy[64];
	if (!a->p_pend) {
		a->p_data = periph_acl_tx_peek(p, &pb, &p_len);
		a->p_pend = true;
	}
	bool p_has = a->p_data && periph_acl_tx_peek(p, &pb, &p_len);
	if (p_has)
		memcpy(p_copy, pb, p_len);
	if (r1 == AIR_OK && c_sn == a->p_nesn) {
		a->p_nesn = !a->p_nesn;
		if (head)
			periph_acl_rx_ctrl(p, head->b, head->len, c_end, (uint16_t)n);
	}
	bool p_sn = a->p_sn, p_nesn = a->p_nesn;
	sim_ns_t r_start = c_end + US(T_IFS_US);
	unsigned pmic = (a->enc == CENC_ON && p_len) ? 4 : 0;
	sim_ns_t r_end = r_start + US(air_time_us(g_cfg.acl_phy, (p_has ? p_len : 0) + pmic));
	radio_claim(100 + idx, t, r_end, true);
	if (air_packet() != AIR_OK)
		return;
	a->last_rx = r_end;
	if (p_nesn != a->c_sn) {
		if (a->c_pend && a->c_data && a->qn) {
			a->qh = (a->qh + 1) % CQ_MAX;
			a->qn--;
		}
		a->c_pend = false;
		a->c_sn = !a->c_sn;
	}
	if (p_sn == a->c_nesn) {
		a->c_nesn = !a->c_nesn;
		if (p_has) {
			deliver_t *d = &s_dq[s_dq_w++ % DQ];
			d->a = a;
			d->len = p_len;
			memcpy(d->b, p_copy, p_len);
			sim_at(r_end + US(50), deliver_cb, d, 0);
		}
	}
}

void acl_finish(void)
{
	for (int i = 0; i < s_n; i++) {
		if (!s_acl[i].p)
			continue;
		periph_finish(s_acl[i].p);
		periph_free(s_acl[i].p);
		s_acl[i].p = NULL;
	}
	s_n = 0;
}
