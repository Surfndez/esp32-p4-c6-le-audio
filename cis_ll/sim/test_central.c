/*
 * ONLY FOR SELF-TESTING THE SIMULATOR: a small, scripted Central with the cis_engine.h API, to check the host,
 * the platform, radio, air and Peripheral without a real engine. Fixed choices: BN = 1, FT = 1,
 * NSE = min(max_nse, 3), Max_PDU = Max_SDU, ISO_Interval = SDU_Interval, a 3-event buffer for SDUs,
 * SN/NESN per the CIS rules (we assume radio_owns_seq = false). It is not a template for the engine and shares
 * no code with it; it uses the simulator's own primitives (AES/CCM, AA).
 */
#include <string.h>

#include "cis_engine.h"
#include "sim.h"

#define TC_MAX_CIS 2
#define SDUQ       8
#define OUTQ       64

typedef struct {
	uint32_t psn; /* extended to 32 bits */
	uint16_t len;
	uint8_t  d[CIS_PDU_MAX_PAYLOAD];
} sdu_t;

typedef enum { TC_IDLE, TC_REQ, TC_WAIT_EST, TC_UP, TC_DOWN } tcst_t;

typedef struct cis_engine cis_engine_t;

typedef struct {
	cis_engine_t *eng;
	uint16_t      handle, acl;
	uint8_t       cis_id;
	tcst_t        st;
	cis_link_t   *link;
	uint32_t      aa, first_anchor, cis_len;
	uint8_t       nse;
	uint32_t      sub_us, cig_sync, cis_sync;
	uint16_t      max_pdu_c, max_pdu_p;
	bool          enc, dp_in, dp_out;
	uint8_t       iv[8];
	sdu_t         q[SDUQ];
	int           qh, qn;
	bool          base_set;
	uint32_t      psn0, psn_ext;
	uint16_t      psn_last;
	uint32_t      evt0, last_prep;
	bool          have_cur, tx_have, tx_acked, rx_got;
	uint32_t      cur_evt;
	cis_pdu_t     txp;
	uint64_t      rx_next;
	uint16_t      up_psn;
	uint32_t      nocp;
} tc_cis_t;

typedef struct {
	bool    enc_on;
	uint8_t ltk[16], skdc[8], ivc[4], sk[16], iv[8];
} tc_acl_t;

struct cis_engine {
	const cis_platform_ops_t *ops;
	cis_radio_caps_t caps;
	bool     wait_dle;
	bool     cig_set;
	uint8_t  cig_id, phy;
	uint32_t sdu_int;
	int      n_cis, req_idx, create_n;
	uint16_t create_order[TC_MAX_CIS];
	tc_cis_t cis[TC_MAX_CIS];
	tc_acl_t acl[4];
	uint8_t  outq[OUTQ][260];
	uint16_t outlen[OUTQ];
	int      oh, on;
};

const char *sim_engine_name(void);
const char *sim_engine_name(void)
{
	return "TEST CENTRAL (simulator self-test fixture, not the cis_ll engine)";
}

static void out(cis_engine_t *e, const uint8_t *b, uint16_t len)
{
	if (e->on == OUTQ || len > 260)
		return;
	int i = (e->oh + e->on++) % OUTQ;
	memcpy(e->outq[i], b, len);
	e->outlen[i] = len;
}

static void cc(cis_engine_t *e, uint16_t op, const uint8_t *ret, uint8_t rlen)
{
	uint8_t b[64] = { 0x04, 0x0E, (uint8_t)(3 + rlen), 1 };
	put16(b + 4, op);
	memcpy(b + 6, ret, rlen);
	out(e, b, (uint16_t)(6 + rlen));
}

static void cs(cis_engine_t *e, uint16_t op, uint8_t status)
{
	uint8_t b[7] = { 0x04, 0x0F, 4, status, 1 };
	put16(b + 5, op);
	out(e, b, 7);
}

static void disc(cis_engine_t *e, uint16_t h, uint8_t reason)
{
	uint8_t b[7] = { 0x04, 0x05, 4, 0 };
	put16(b + 4, h);
	b[6] = reason;
	out(e, b, 7);
}

static void est(cis_engine_t *e, tc_cis_t *c, uint8_t status)
{
	uint8_t b[3 + 29] = { 0x04, 0x3E, 29, 0x19, status };
	put16(b + 5, c->handle);
	if (status == 0) {
		uint8_t *p = b + 7;
		put24(p, c->cig_sync);
		put24(p + 3, c->cis_sync);
		put24(p + 6, c->cig_sync); /* unframed: CIG_Sync_Delay + FT*ISO - SDU_Interval */
		put24(p + 9, c->cig_sync);
		p[12] = e->phy;
		p[13] = e->phy;
		p[14] = c->nse;
		p[15] = 1;
		p[16] = c->max_pdu_p ? 1 : 0;
		p[17] = 1;
		p[18] = 1;
		put16(p + 19, c->max_pdu_c);
		put16(p + 21, c->max_pdu_p);
		put16(p + 23, (uint16_t)(e->sdu_int / 1250));
	}
	out(e, b, sizeof(b));
}

uint32_t cis_engine_mem_size(void)
{
	return sizeof(cis_engine_t);
}

cis_engine_t *cis_engine_init(void *mem, uint32_t mem_size, const cis_platform_ops_t *ops)
{
	if (!mem || mem_size < sizeof(cis_engine_t))
		return NULL;
	cis_engine_t *e = mem;
	memset(e, 0, sizeof(*e));
	e->ops = ops;
	e->req_idx = -1;
	ops->radio_caps(ops->ctx, &e->caps);
	return e;
}

static tc_cis_t *by_handle(cis_engine_t *e, uint16_t h)
{
	for (int i = 0; i < e->n_cis; i++)
		if (e->cis[i].handle == h)
			return &e->cis[i];
	return NULL;
}

static uint32_t se_len(const cis_engine_t *e, const tc_cis_t *c)
{
	unsigned mic = c->enc ? 4 : 0;
	return air_time_us(e->phy, c->max_pdu_c + mic) + T_IFS_US +
			air_time_us(e->phy, c->max_pdu_p + (c->max_pdu_p ? mic : 0));
}

static void start_req(cis_engine_t *e)
{
	e->req_idx = -1;
	for (int k = 0; k < e->create_n; k++) {
		tc_cis_t *c = by_handle(e, e->create_order[k]);
		if (!c || c->st != TC_IDLE)
			continue;
		cis_acl_timing_t t;
		if (e->ops->acl_timing(e->ops->ctx, c->acl, &t)) {
			c->st = TC_DOWN;
			est(e, c, 0x02);
			continue;
		}
		c->enc = e->acl[c->acl & 3].enc_on;
		c->nse = e->caps.max_nse > 3 ? 3 : (e->caps.max_nse ? e->caps.max_nse : 1);
		c->sub_us = c->nse > 1 ? se_len(e, c) + T_MSS_US : 0;
		uint8_t b[36] = { LL_CIS_REQ, e->cig_id, c->cis_id, e->phy, e->phy };
		put16(b + 5, c->max_pdu_c);
		put16(b + 7, c->max_pdu_p);
		put24(b + 9, e->sdu_int);
		put24(b + 12, e->sdu_int);
		put16(b + 15, c->max_pdu_c);
		put16(b + 17, c->max_pdu_p);
		b[19] = c->nse;
		put24(b + 20, c->sub_us);
		b[23] = (uint8_t)(1 | ((c->max_pdu_p ? 1 : 0) << 4));
		b[24] = 1;
		b[25] = c->max_pdu_p ? 1 : 0;
		put16(b + 26, (uint16_t)(e->sdu_int / 1250));
		put24(b + 28, 500);
		put24(b + 31, e->sdu_int - 500);
		put16(b + 34, (uint16_t)(t.event_counter + 3));
		if (e->ops->send_ctrl_pdu(e->ops->ctx, c->acl, b, 36) == CIS_PLAT_ERR_TOO_LONG) {
			e->wait_dle = true; /* we retry after LE Data Length Change */
			return;
		}
		c->st = TC_REQ;
		e->req_idx = (int)(c - e->cis);
		return;
	}
}

/* Whether a CIS starting at time s (us relative to the ACL anchor ref) collides with an ACL or another CIS. */
static bool placement_ok(cis_engine_t *e, const tc_cis_t *c, uint32_t ref_anchor, int64_t s)
{
	int64_t iso = e->sdu_int, len = (int64_t)(c->nse - 1) * c->sub_us + se_len(e, c) + 300;
	int64_t acl_len = 2 * air_time_us(CIS_PHY_1M, 31) + T_IFS_US + 300;
	for (int a = 1; a <= 2; a++) {
		cis_acl_timing_t t;
		if (e->ops->acl_timing(e->ops->ctx, (uint16_t)a, &t))
			continue;
		int64_t rel = (int32_t)(t.anchor_us - ref_anchor);
		for (int k = 0; k < 6; k++) {
			int64_t cs0 = s + k * iso;
			int64_t m = (cs0 - rel) / (int64_t)t.interval_us;
			for (int64_t j = m - 1; j <= m + 1; j++) {
				int64_t as = rel + j * (int64_t)t.interval_us;
				if (as < cs0 + len && cs0 < as + acl_len)
					return false;
			}
		}
	}
	for (int i = 0; i < e->n_cis; i++) {
		const tc_cis_t *o = &e->cis[i];
		if (o == c || (o->st != TC_UP && o->st != TC_WAIT_EST))
			continue;
		int64_t rel = (int32_t)(o->first_anchor - ref_anchor);
		int64_t d = ((s - rel) % iso + iso) % iso;
		if (d < (int64_t)o->cis_len + 300 || iso - d < len)
			return false;
	}
	return true;
}

static void on_cis_rsp(cis_engine_t *e, const uint8_t *ctr)
{
	tc_cis_t *c = &e->cis[e->req_idx];
	uint32_t omin = get24(ctr + 1), omax = get24(ctr + 4);
	cis_acl_timing_t t;
	if (e->ops->acl_timing(e->ops->ctx, c->acl, &t)) {
		c->st = TC_DOWN;
		est(e, c, 0x02);
		start_req(e);
		return;
	}
	uint16_t cec = (uint16_t)(t.event_counter + 4);
	int64_t base = (int64_t)(uint16_t)(cec - t.event_counter) * t.interval_us;
	uint32_t off = omin;
	c->cis_len = (uint32_t)(c->nse - 1) * c->sub_us + se_len(e, c);
	for (uint32_t o = omin; o <= omax; o += 250) {
		if (placement_ok(e, c, t.anchor_us, base + o)) {
			off = o;
			break;
		}
	}
	do {
		e->ops->random(e->ops->ctx, (uint8_t *)&c->aa, 4);
	} while (aa_check(c->aa) || (c != &e->cis[0] && c->aa == e->cis[0].aa));
	c->cis_sync = c->cis_len;
	c->cig_sync = c->cis_len;
	uint8_t b[16] = { LL_CIS_IND };
	put32(b + 1, c->aa);
	put24(b + 5, off);
	put24(b + 8, c->cig_sync);
	put24(b + 11, c->cis_sync);
	put16(b + 14, cec);
	e->ops->send_ctrl_pdu(e->ops->ctx, c->acl, b, 16);
	c->first_anchor = t.anchor_us + (uint32_t)base + off;
	tc_acl_t *a = &e->acl[c->acl & 3];
	memcpy(c->iv, a->iv, 8);
	for (int i = 0; i < 4; i++)
		c->iv[i] ^= (uint8_t)(c->aa >> (8 * i));
	cis_link_params_t lp = {
		.access_address = c->aa,
		.crc_init = t.crc_init,
		.phy_c_to_p = e->phy,
		.phy_p_to_c = e->phy,
		.nse = c->nse,
		.sub_interval_us = c->sub_us,
		.iso_interval_us = e->sdu_int,
		.first_anchor_us = c->first_anchor,
		.max_pdu_c_to_p = (uint8_t)(c->max_pdu_c + (c->enc ? 4 : 0)),
		.max_pdu_p_to_c = (uint8_t)(c->max_pdu_p + (c->enc && c->max_pdu_p ? 4 : 0)),
	};
	memcpy(lp.chan_map, t.chan_map, 5);
	extern const cis_link_cb_t k_tc_cb;
	if (e->ops->link_open(e->ops->ctx, &lp, &k_tc_cb, c, &c->link)) {
		c->st = TC_DOWN;
		est(e, c, 0x0D);
	} else {
		c->st = TC_WAIT_EST;
	}
	start_req(e);
}

/* ---------------- radio ---------------- */
static void begin_event(tc_cis_t *c, uint32_t evt)
{
	c->have_cur = true;
	c->cur_evt = evt;
	c->tx_have = c->tx_acked = c->rx_got = false;
	c->last_prep = evt;
	while (c->qn) {
		sdu_t *s = &c->q[c->qh];
		uint32_t target = c->evt0 + (s->psn - c->psn0);
		if ((int32_t)(target - evt) > 0)
			break;
		c->qh = (c->qh + 1) % SDUQ;
		c->qn--;
		c->nocp++;
		if (target != evt)
			continue; /* late */
		uint8_t hdr = (uint8_t)((evt & 1) ? HDR_SN : 0);
		c->txp.hdr0 = hdr;
		if (c->enc) {
			uint8_t nonce[13];
			ble_nonce(nonce, evt, true, c->iv);
			ble_ccm_encrypt(c->eng->acl[c->acl & 3].sk, nonce, hdr & CCM_AAD_MASK, s->d, (uint8_t)s->len,
					c->txp.payload);
			c->txp.len = (uint8_t)(s->len + 4);
		} else {
			memcpy(c->txp.payload, s->d, s->len);
			c->txp.len = (uint8_t)s->len;
		}
		c->tx_have = true;
		break;
	}
}

static bool tc_prepare_tx(void *arg, uint32_t evt, uint8_t se, cis_pdu_t *o)
{
	(void)se;
	tc_cis_t *c = arg;
	if (!c->have_cur || evt != c->cur_evt)
		begin_event(c, evt);
	uint8_t nesn = (c->rx_next & 1) ? HDR_NESN : 0;
	if (c->tx_have && !c->tx_acked) {
		*o = c->txp;
		o->hdr0 = (uint8_t)((c->txp.hdr0 & ~HDR_NESN) | nesn);
	} else {
		memset(o, 0, sizeof(*o));
		o->hdr0 = (uint8_t)(HDR_NPI | nesn | ((evt & 1) ? HDR_SN : 0));
		if (c->rx_got || !c->max_pdu_p)
			o->hdr0 |= HDR_CIE;
	}
	return true;
}

static void tc_rx_done(void *arg, uint32_t evt, uint8_t se, const cis_pdu_t *pdu, bool crc_ok, uint32_t ts)
{
	(void)se;
	(void)ts;
	tc_cis_t *c = arg;
	if (!crc_ok || !pdu)
		return;
	cis_engine_t *e = c->eng;
	if (c->st == TC_WAIT_EST) {
		c->st = TC_UP;
		est(e, c, 0);
	}
	uint8_t h = pdu->hdr0;
	if (((h & HDR_NESN) != 0) != (bool)(evt & 1) && c->tx_have && !c->tx_acked)
		c->tx_acked = true;
	if ((h & HDR_NPI) || !c->max_pdu_p || ((h & HDR_SN) != 0) != (bool)(c->rx_next & 1) || c->rx_next != evt)
		return;
	uint8_t plain[CIS_PDU_MAX_PAYLOAD];
	uint8_t len = pdu->len;
	if (c->enc && len) {
		uint8_t nonce[13];
		ble_nonce(nonce, c->rx_next, false, c->iv);
		if (len < 4 || !ble_ccm_decrypt(e->acl[c->acl & 3].sk, nonce, h & CCM_AAD_MASK, pdu->payload, len, plain))
			return;
		len = (uint8_t)(len - 4);
	} else {
		memcpy(plain, pdu->payload, len);
	}
	c->rx_next++;
	c->rx_got = true;
	if (c->dp_out) {
		uint8_t b[9 + CIS_PDU_MAX_PAYLOAD] = { 0x05 };
		put16(b + 1, c->handle | (2u << 12));
		put16(b + 3, 4u + len);
		put16(b + 5, c->up_psn++);
		put16(b + 7, len);
		memcpy(b + 9, plain, len);
		out(e, b, (uint16_t)(9 + len));
	}
}

static void close_evt(tc_cis_t *c, uint32_t evt)
{
	cis_engine_t *e = c->eng;
	if (c->rx_next < (uint64_t)evt + 1)
		c->rx_next = (uint64_t)evt + 1;
	if (c->st == TC_UP && c->dp_out && !c->rx_got && c->max_pdu_p) {
		uint8_t b[9] = { 0x05 };
		put16(b + 1, c->handle | (2u << 12));
		put16(b + 3, 4);
		put16(b + 5, c->up_psn++);
		put16(b + 7, 0x8000); /* Packet_Status_Flag = 10: lost */
		out(e, b, 9);
	}
	c->have_cur = false;
	if (c->st == TC_WAIT_EST && evt >= 5) {
		e->ops->link_close(e->ops->ctx, c->link);
		c->st = TC_DOWN;
		est(e, c, 0x3E);
	}
}

static void tc_event_closed(void *arg, uint32_t evt)
{
	close_evt(arg, evt);
}

static void tc_event_skipped(void *arg, uint32_t evt)
{
	tc_cis_t *c = arg;
	c->last_prep = evt;
	c->rx_got = false;
	close_evt(c, evt);
}

const cis_link_cb_t k_tc_cb = { tc_prepare_tx, tc_rx_done, tc_event_closed, tc_event_skipped };

/* ---------------- HCI ---------------- */
bool cis_engine_hci_cmd(cis_engine_t *e, const uint8_t *h4, uint16_t len)
{
	if (len < 4 || h4[0] != 0x01)
		return false;
	uint16_t op = get16(h4 + 1);
	const uint8_t *p = h4 + 4;
	uint8_t r[16] = { 0 };
	switch (op) {
	case 0x2019: {
		uint16_t a = get16(p);
		if (a >= 1 && a <= 3)
			memcpy(e->acl[a].ltk, p + 12, 16);
		return false;
	}
	case 0x2074:
		cc(e, op, r, 1);
		return true;
	case 0x2060:
		put16(r + 1, 251);
		r[3] = 4;
		put16(r + 4, 251);
		r[6] = 6;
		cc(e, op, r, 7);
		return true;
	case 0x2062: {
		e->cig_id = p[0];
		e->sdu_int = get24(p + 1);
		e->n_cis = p[14] > TC_MAX_CIS ? TC_MAX_CIS : p[14];
		r[1] = e->cig_id;
		r[2] = (uint8_t)e->n_cis;
		for (int i = 0; i < e->n_cis; i++) {
			const uint8_t *q = p + 15 + 9 * i;
			tc_cis_t *c = &e->cis[i];
			memset(c, 0, sizeof(*c));
			c->eng = e;
			c->cis_id = q[0];
			c->max_pdu_c = get16(q + 1);
			c->max_pdu_p = get16(q + 3);
			e->phy = q[5];
			c->handle = (uint16_t)(0x0100 + i);
			put16(r + 3 + 2 * i, c->handle);
		}
		e->cig_set = true;
		cc(e, op, r, (uint8_t)(3 + 2 * e->n_cis));
		return true;
	}
	case 0x2064:
		e->create_n = 0;
		for (int i = 0; i < p[0] && i < TC_MAX_CIS; i++) {
			tc_cis_t *c = by_handle(e, get16(p + 1 + 4 * i));
			if (!c)
				continue;
			c->acl = get16(p + 3 + 4 * i);
			e->create_order[e->create_n++] = c->handle;
		}
		cs(e, op, 0);
		start_req(e);
		return true;
	case 0x206E:
	case 0x206F: {
		tc_cis_t *c = by_handle(e, get16(p));
		if (op == 0x206E && c) {
			if (p[2] == 0)
				c->dp_in = true;
			else
				c->dp_out = true;
		}
		r[0] = c ? 0 : 0x02;
		put16(r + 1, get16(p));
		cc(e, op, r, 3);
		return true;
	}
	case 0x0406: {
		tc_cis_t *c = by_handle(e, get16(p));
		if (!c)
			return false;
		cs(e, op, 0);
		if (c->st == TC_UP || c->st == TC_WAIT_EST) {
			uint8_t b[4] = { LL_CIS_TERMINATE_IND, e->cig_id, c->cis_id, p[2] };
			e->ops->send_ctrl_pdu(e->ops->ctx, c->acl, b, 4);
			e->ops->link_close(e->ops->ctx, c->link);
			c->st = TC_DOWN;
			disc(e, c->handle, 0x16);
		}
		return true;
	}
	case 0x2065:
		e->cig_set = false;
		r[1] = p[0];
		cc(e, op, r, 2);
		return true;
	default:
		return false;
	}
}

void cis_engine_hci_iso(cis_engine_t *e, const uint8_t *h4, uint16_t len)
{
	if (len < 9)
		return;
	tc_cis_t *c = by_handle(e, get16(h4 + 1) & 0x0FFF);
	if (!c || !c->dp_in || c->st != TC_UP)
		return;
	uint16_t psn = get16(h4 + 5), slen = get16(h4 + 7) & 0x0FFF;
	if (slen > CIS_PDU_MAX_PAYLOAD - 4 || len < 9u + slen)
		return;
	if (!c->base_set) {
		c->base_set = true;
		c->psn_ext = c->psn0 = psn;
		c->psn_last = psn;
		c->evt0 = c->last_prep + 3;
	}
	c->psn_ext += (uint16_t)(psn - c->psn_last);
	c->psn_last = psn;
	if (c->qn == SDUQ) {
		c->qh = (c->qh + 1) % SDUQ;
		c->qn--;
		c->nocp++;
	}
	sdu_t *s = &c->q[(c->qh + c->qn++) % SDUQ];
	s->psn = c->psn_ext;
	s->len = slen;
	memcpy(s->d, h4 + 9, slen);
}

bool cis_engine_hci_evt_from_controller(cis_engine_t *e, const uint8_t *h4, uint16_t len)
{
	if (len < 7 || h4[0] != 0x04)
		return false;
	if (h4[1] == 0x3E && h4[3] == 0x07 && e->wait_dle) {
		e->wait_dle = false;
		start_req(e);
		return false;
	}
	uint16_t h = get16(h4 + 4);
	if (h4[1] == 0x08 && h4[3] == 0 && h < 4) {
		e->acl[h].enc_on = h4[6] != 0;
	} else if (h4[1] == 0x05) {
		for (int i = 0; i < e->n_cis; i++) {
			tc_cis_t *c = &e->cis[i];
			if (c->acl == h && (c->st == TC_UP || c->st == TC_WAIT_EST)) {
				e->ops->link_close(e->ops->ctx, c->link);
				c->st = TC_DOWN;
				disc(e, c->handle, h4[6]);
			}
		}
	}
	return false;
}

bool cis_engine_ll_ctrl_rx(cis_engine_t *e, uint16_t acl_handle, const uint8_t *ctr, uint8_t len)
{
	tc_acl_t *a = &e->acl[acl_handle & 3];
	switch (ctr[0]) {
	case LL_ENC_RSP:
		if (len == 13) {
			ble_session_key(a->ltk, a->skdc, ctr + 1, a->sk);
			memcpy(a->iv, a->ivc, 4);
			memcpy(a->iv + 4, ctr + 9, 4);
		}
		return false;
	case LL_CIS_RSP:
		if (len != 9 || e->req_idx < 0)
			return false;
		on_cis_rsp(e, ctr);
		return true;
	case LL_REJECT_EXT_IND:
	case LL_UNKNOWN_RSP:
		if (e->req_idx < 0 || ctr[1] != LL_CIS_REQ)
			return false;
		e->cis[e->req_idx].st = TC_DOWN;
		est(e, &e->cis[e->req_idx], ctr[0] == LL_REJECT_EXT_IND && len == 3 ? ctr[2] : 0x1A);
		start_req(e);
		return true;
	case LL_CIS_TERMINATE_IND:
		for (int i = 0; i < e->n_cis; i++) {
			tc_cis_t *c = &e->cis[i];
			if (len == 4 && c->acl == acl_handle && c->cis_id == ctr[2] && c->st == TC_UP) {
				e->ops->link_close(e->ops->ctx, c->link);
				c->st = TC_DOWN;
				disc(e, c->handle, ctr[3]);
			}
		}
		return true;
	default:
		return false;
	}
}

void cis_engine_ll_ctrl_tx_seen(cis_engine_t *e, uint16_t acl_handle, const uint8_t *ctr, uint8_t len)
{
	if (ctr[0] == LL_ENC_REQ && len == 23) {
		tc_acl_t *a = &e->acl[acl_handle & 3];
		memcpy(a->skdc, ctr + 11, 8);
		memcpy(a->ivc, ctr + 19, 4);
	}
}

void cis_engine_on_timer(cis_engine_t *e)
{
	(void)e;
}

void cis_engine_process(cis_engine_t *e)
{
	for (int i = 0; i < e->n_cis; i++) {
		tc_cis_t *c = &e->cis[i];
		if (c->nocp) {
			uint8_t b[8] = { 0x04, 0x13, 5, 1 };
			put16(b + 4, c->handle);
			put16(b + 6, c->nocp);
			c->nocp = 0;
			out(e, b, 8);
		}
	}
	while (e->on) {
		e->ops->hci_to_host(e->ops->ctx, e->outq[e->oh], e->outlen[e->oh]);
		e->oh = (e->oh + 1) % OUTQ;
		e->on--;
	}
}

bool cis_engine_active(const cis_engine_t *e)
{
	return e->cig_set;
}

bool cis_engine_get_stats(const cis_engine_t *e, uint16_t cis_handle, cis_engine_stats_t *out)
{
	(void)e;
	(void)cis_handle;
	(void)out;
	return false; /* the test Central keeps no counters - the simulator does the checking */
}
