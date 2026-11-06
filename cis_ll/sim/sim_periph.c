/*
 * Reference LL Peripheral (an earbud) - written only from the Core 5.3+ specification, without access to the engine.
 *
 * CIS acknowledgement model (Vol 6 Part B 4.5.13): every payload has a number p (cisPayloadCounter); the payload SN
 * = p & 1, NESN = (the smallest payload not yet received) & 1. Payload p is scheduled in events
 * floor(p/BN) .. floor(p/BN)+FT-1; after the flush point (the end of the last of them) the sender and the receiver
 * shift SN/NESN as if the payload had gone through - so both sides agree without any exchange.
 * Null PDUs (NPI = 1) have no number: the receiver ignores their SN and does not acknowledge them.
 * CIS encryption: SK from the ACL, CIS IV = ACL IV with the 32 low bits XORed with the CIS AA, nonce with counter p,
 * direction 1 = C->P. AAD = header & 0xE3.
 *
 * Every Central deviation from the specification goes to viol() with a category; less certain interpretations - warn().
 */
#include "sim.h"

#include <stdlib.h>
#include <string.h>

#define PQ_MAX   8
#define TXR      64  /* uplink payload ring */
#define MAX_BN   15
#define W_JITTER US(2)
#define BEGIN_MARGIN US(100)

typedef struct {
	uint8_t len;
	uint8_t b[40];
} ctrl_pdu_t;

typedef enum { CIS_IDLE, CIS_RSP_SENT, CIS_PENDING, CIS_ACTIVE, CIS_DONE } cis_state_t;
typedef enum { ENC_OFF, ENC_STARTING, ENC_ON } enc_state_t;

typedef struct {
	uint8_t  cig_id, cis_id, phy_c, phy_p;
	uint16_t max_sdu_c, max_sdu_p;
	bool     framed;
	uint32_t sdu_int_c, sdu_int_p;
	uint16_t max_pdu_c, max_pdu_p;
	uint8_t  nse;
	uint32_t sub_us;
	uint8_t  bn_c, bn_p, ft_c, ft_p;
	uint16_t iso_int;
	uint32_t off_min, off_max;
	uint16_t cec_req;
	uint32_t rsp_off_min, rsp_off_max;
	uint16_t rsp_cec;
	uint32_t aa, cis_off, cig_sync, cis_sync;
	uint16_t cec;
} cis_par_t;

typedef struct {
	bool    valid;
	uint8_t llid;
	uint8_t len;
	uint8_t data[CIS_PDU_MAX_PAYLOAD];
} txp_t;

struct periph {
	int         idx;
	char        who[8];
	sim_clock_t clk;
	rng_t       rng;
	uint8_t     ltk_le[16];
	bool        gone;
	/* ACL */
	uint32_t    acl_interval_us;
	uint8_t     chan_map[5];
	uint32_t    acl_aa;
	bool        acl_synced;
	int64_t     acl_ref_local;
	uint16_t    acl_ref_evt;
	ctrl_pdu_t  txq[PQ_MAX];
	int         txq_head, txq_n;
	/* ACL encryption */
	enc_state_t enc;
	uint8_t     sk[16];
	uint8_t     iv_le[8];
	/* CIS */
	cis_state_t st;
	cis_par_t   par;
	sim_ns_t    rsp_time;
	bool        cis_enc;
	uint8_t     cis_iv[8];
	int64_t     iso_ns, sub_ns;
	int64_t     ref_local, wbase_local;
	int64_t     ref_evt;
	uint32_t    gen;
	/* current event */
	bool        evt_open, evt_closed, evt_got_any;
	int64_t     cur_evt, cur_anchor_local;
	uint8_t     next_se;
	csa2_t      csa;
	uint8_t     chans[32];
	/* payload state */
	int64_t     rx_next, tx_next, last_rx_payload;
	bool        tx_sent_cur;
	txp_t       txr[TXR];
	/* SDU reassembly */
	uint8_t     sdu[4096];
	uint16_t    sdu_len;
	int64_t     sdu_group;
	bool        sdu_bad;
	bool        have_seq;
	uint32_t    last_seq;
	uint32_t    up_seq;
	sim_ns_t    last_crc_ok;
	bool        established;
	bool        have_lat;
	int64_t     lat_first;
};

static const char *who(const periph_t *p)
{
	return p->who;
}

periph_t *periph_new(int idx, const sim_clock_t *clk, uint64_t seed, const uint8_t ltk_le[16])
{
	periph_t *p = calloc(1, sizeof(*p));
	if (!p)
		abort();
	p->idx = idx;
	snprintf(p->who, sizeof(p->who), "P%d", idx);
	p->clk = *clk;
	rng_seed(&p->rng, seed);
	memcpy(p->ltk_le, ltk_le, 16);
	p->last_rx_payload = -1;
	return p;
}

void periph_free(periph_t *p)
{
	free(p);
}

const sim_clock_t *periph_clock(const periph_t *p)
{
	return &p->clk;
}

bool periph_acl_encrypted(const periph_t *p)
{
	return p->enc == ENC_ON;
}

uint32_t periph_cis_aa(const periph_t *p)
{
	return (p->st == CIS_PENDING || p->st == CIS_ACTIVE || p->st == CIS_DONE) ? p->par.aa : 0;
}

bool periph_has_aa(const periph_t *p, uint32_t aa)
{
	return !p->gone && (p->st == CIS_PENDING || p->st == CIS_ACTIVE) && p->par.aa == aa;
}

/* ---------------- LLCP queue to the Central ---------------- */
static void txq_push(periph_t *p, const uint8_t *b, uint8_t len)
{
	if (p->txq_n == PQ_MAX) {
		slog(who(p), "BUG: LLCP tx queue full");
		return;
	}
	ctrl_pdu_t *c = &p->txq[(p->txq_head + p->txq_n++) % PQ_MAX];
	c->len = len;
	memcpy(c->b, b, len);
}

bool periph_acl_tx_peek(periph_t *p, const uint8_t **ctr, uint8_t *len)
{
	if (p->gone || !p->txq_n)
		return false;
	*ctr = p->txq[p->txq_head].b;
	*len = p->txq[p->txq_head].len;
	return true;
}

void periph_acl_tx_pop(periph_t *p)
{
	if (p->txq_n) {
		p->txq_head = (p->txq_head + 1) % PQ_MAX;
		p->txq_n--;
	}
}

void periph_acl_set(periph_t *p, uint32_t interval_us, const uint8_t chan_map[5], uint32_t acl_aa)
{
	p->acl_interval_us = interval_us;
	memcpy(p->chan_map, chan_map, 5);
	p->acl_aa = acl_aa;
}

void periph_acl_sync(periph_t *p, uint16_t evt, sim_ns_t t_true)
{
	p->acl_synced = true;
	p->acl_ref_local = clk_local(&p->clk, t_true);
	p->acl_ref_evt = evt;
}

void periph_acl_lost(periph_t *p)
{
	p->gone = true;
	if (p->st == CIS_ACTIVE || p->st == CIS_PENDING) {
		p->st = CIS_DONE;
		p->gen++;
	}
}

/* ---------------- LLCP ---------------- */
static void send_unknown(periph_t *p, uint8_t op)
{
	uint8_t b[2] = { LL_UNKNOWN_RSP, op };
	txq_push(p, b, 2);
}

static uint32_t subevent_len_us(const cis_par_t *c, bool enc)
{
	unsigned mic = enc ? 4 : 0;
	return air_time_us(c->phy_c, c->max_pdu_c + (c->max_pdu_c ? mic : 0)) + T_IFS_US +
			air_time_us(c->phy_p, c->max_pdu_p + (c->max_pdu_p ? mic : 0));
}

static bool one_phy_bit(uint8_t v)
{
	return v == 0x01 || v == 0x02 || v == 0x04;
}

static void rx_cis_req(periph_t *p, const uint8_t *d)
{
	cis_par_t *c = &p->par;
	if (p->st == CIS_RSP_SENT || p->st == CIS_PENDING)
		viol(V_LLCP, who(p), "LL_CIS_REQ while a CIS procedure is already in progress");
	if (p->st == CIS_ACTIVE)
		warn(who(p), "LL_CIS_REQ while a CIS is active on this ACL");
	if (p->enc == ENC_STARTING)
		viol(V_LLCP, who(p), "LL_CIS_REQ during the encryption start procedure");
	memset(c, 0, sizeof(*c));
	c->cig_id = d[0];
	c->cis_id = d[1];
	c->phy_c = d[2];
	c->phy_p = d[3];
	uint16_t w = get16(d + 4);
	c->max_sdu_c = w & 0x0FFF;
	c->framed = (w >> 15) & 1;
	if (w & 0x7000)
		viol(V_FORMAT, who(p), "CIS_REQ Max_SDU_C_To_P RFU bits set (0x%04x)", w);
	w = get16(d + 6);
	c->max_sdu_p = w & 0x0FFF;
	if (w & 0xF000)
		viol(V_FORMAT, who(p), "CIS_REQ Max_SDU_P_To_C RFU bits set (0x%04x)", w);
	uint32_t v = get24(d + 8);
	c->sdu_int_c = v & 0xFFFFF;
	if (v & 0xF00000)
		viol(V_FORMAT, who(p), "CIS_REQ SDU_Interval_C_To_P RFU bits set");
	v = get24(d + 11);
	c->sdu_int_p = v & 0xFFFFF;
	if (v & 0xF00000)
		viol(V_FORMAT, who(p), "CIS_REQ SDU_Interval_P_To_C RFU bits set");
	c->max_pdu_c = get16(d + 14);
	c->max_pdu_p = get16(d + 16);
	c->nse = d[18];
	c->sub_us = get24(d + 19);
	c->bn_c = d[22] & 0x0F;
	c->bn_p = d[22] >> 4;
	c->ft_c = d[23];
	c->ft_p = d[24];
	c->iso_int = get16(d + 25);
	c->off_min = get24(d + 27);
	c->off_max = get24(d + 30);
	c->cec_req = get16(d + 33);
	slog(who(p), "rx LL_CIS_REQ cig %u cis %u phy %u/%u sdu %u/%u int %u/%u pdu %u/%u nse %u sub %u bn %u/%u "
			"ft %u/%u iso %u off %u..%u cec %u", c->cig_id, c->cis_id, c->phy_c, c->phy_p, c->max_sdu_c,
			c->max_sdu_p, c->sdu_int_c, c->sdu_int_p, c->max_pdu_c, c->max_pdu_p, c->nse, c->sub_us,
			c->bn_c, c->bn_p, c->ft_c, c->ft_p, c->iso_int, c->off_min, c->off_max, c->cec_req);

	/* Specification rules (2.4.2.25 and the field descriptions). */
	if (!one_phy_bit(c->phy_c) || !one_phy_bit(c->phy_p))
		viol(V_PARAM, who(p), "PHY field must have exactly one bit set (%u/%u)", c->phy_c, c->phy_p);
	if (c->phy_c == 0x04 || c->phy_p == 0x04)
		warn(who(p), "Coded PHY requested - not modelled");
	if (c->sdu_int_c < 0xFF || c->sdu_int_p < 0xFF)
		viol(V_PARAM, who(p), "SDU_Interval below 255 us");
	if (c->max_pdu_c > 251 || c->max_pdu_p > 251)
		viol(V_PARAM, who(p), "Max_PDU above 251");
	if (c->nse < 1 || c->nse > 0x1F)
		viol(V_PARAM, who(p), "NSE %u out of 1..31", c->nse);
	if (c->bn_c > c->nse || c->bn_p > c->nse)
		viol(V_PARAM, who(p), "BN (%u/%u) greater than NSE %u", c->bn_c, c->bn_p, c->nse);
	if ((c->max_sdu_c && !c->bn_c) || (c->max_sdu_p && !c->bn_p))
		viol(V_PARAM, who(p), "BN = 0 in a direction with Max_SDU > 0");
	if ((c->bn_c && !c->ft_c) || (c->bn_p && !c->ft_p))
		viol(V_PARAM, who(p), "FT = 0");
	if (c->iso_int < 4 || c->iso_int > 3200)
		viol(V_PARAM, who(p), "ISO_Interval %u out of 4..3200", c->iso_int);
	uint32_t iso_us = c->iso_int * 1250u;
	bool enc_planned = p->enc == ENC_ON;
	if (c->nse > 1) {
		if (c->sub_us < 400 || c->sub_us >= iso_us)
			viol(V_PARAM, who(p), "Sub_Interval %u us out of 400..ISO_Interval", c->sub_us);
		if (c->sub_us < subevent_len_us(c, enc_planned) + T_MSS_US)
			viol(V_PARAM, who(p), "Sub_Interval %u us shorter than subevent %u us + T_MSS", c->sub_us,
					subevent_len_us(c, enc_planned));
		if ((uint64_t)(c->nse - 1) * c->sub_us + subevent_len_us(c, enc_planned) > iso_us)
			viol(V_PARAM, who(p), "NSE * Sub_Interval does not fit the ISO interval");
	} else if (c->sub_us != 0) {
		warn(who(p), "NSE = 1 but Sub_Interval = %u (expected 0)", c->sub_us);
	}
	if (!c->framed) {
		if (c->sdu_int_c && iso_us % c->sdu_int_c)
			viol(V_PARAM, who(p), "unframed: ISO_Interval %u us not a multiple of SDU_Interval_C_To_P %u",
					iso_us, c->sdu_int_c);
		if (c->sdu_int_p && iso_us % c->sdu_int_p)
			viol(V_PARAM, who(p), "unframed: ISO_Interval not a multiple of SDU_Interval_P_To_C");
		if (c->sdu_int_c && c->max_pdu_c &&
				(uint32_t)c->bn_c * c->max_pdu_c < (uint32_t)c->max_sdu_c * (iso_us / c->sdu_int_c))
			viol(V_PARAM, who(p), "unframed C->P: BN*Max_PDU cannot carry Max_SDU per ISO interval");
		if (c->sdu_int_p && c->max_pdu_p &&
				(uint32_t)c->bn_p * c->max_pdu_p < (uint32_t)c->max_sdu_p * (iso_us / c->sdu_int_p))
			viol(V_PARAM, who(p), "unframed P->C: BN*Max_PDU cannot carry Max_SDU per ISO interval");
	}
	if (c->off_min > c->off_max)
		viol(V_PARAM, who(p), "CIS_Offset_Min %u > CIS_Offset_Max %u", c->off_min, c->off_max);
	if (c->off_min < 500)
		warn(who(p), "CIS_Offset_Min %u below 500 us", c->off_min);
	/* Conformance with the host request (LE Set CIG Parameters). */
	if (c->cig_id != 1 || c->cis_id != p->idx)
		viol(V_PARAM, who(p), "CIG_ID/CIS_ID %u/%u differ from host request 1/%d", c->cig_id, c->cis_id, p->idx);
	if (c->framed)
		viol(V_PARAM, who(p), "framed requested, host asked for unframed");
	if (c->max_sdu_c != g_cfg.max_sdu_c2p || c->max_sdu_p != (g_cfg.uplink ? g_cfg.max_sdu_p2c : c->max_sdu_p))
		viol(V_PARAM, who(p), "Max_SDU %u/%u differ from host request %u/%u", c->max_sdu_c, c->max_sdu_p,
				g_cfg.max_sdu_c2p, g_cfg.max_sdu_p2c);
	if (c->sdu_int_c != g_cfg.sdu_interval_us || c->sdu_int_p != g_cfg.sdu_interval_us)
		viol(V_PARAM, who(p), "SDU_Interval %u/%u differ from host request %u", c->sdu_int_c, c->sdu_int_p,
				g_cfg.sdu_interval_us);
	if (c->phy_c != g_cfg.phy || c->phy_p != g_cfg.phy)
		viol(V_PARAM, who(p), "PHY %u/%u differ from host request %u", c->phy_c, c->phy_p, g_cfg.phy);
	if (c->max_pdu_c < (c->max_sdu_c + c->bn_c - 1) / (c->bn_c ? c->bn_c : 1))
		viol(V_PARAM, who(p), "Max_PDU_C_To_P %u too small", c->max_pdu_c);

	if (g_cfg.reject) {
		uint8_t b[3] = { LL_REJECT_EXT_IND, LL_CIS_REQ, g_cfg.reject_reason };
		txq_push(p, b, 3);
		p->st = CIS_IDLE;
		slog(who(p), "tx LL_REJECT_EXT_IND reason 0x%02x", g_cfg.reject_reason);
		return;
	}
	c->rsp_off_min = c->off_min + g_cfg.rsp_offset_min_add_us;
	if (c->rsp_off_min > c->off_max)
		c->rsp_off_min = c->off_max;
	c->rsp_off_max = c->off_max;
	c->rsp_cec = c->cec_req;
	uint8_t b[9] = { LL_CIS_RSP };
	put24(b + 1, c->rsp_off_min);
	put24(b + 4, c->rsp_off_max);
	put16(b + 7, c->rsp_cec);
	txq_push(p, b, 9);
	p->st = CIS_RSP_SENT;
	p->rsp_time = sim_now();
	slog(who(p), "tx LL_CIS_RSP off %u..%u cec %u", c->rsp_off_min, c->rsp_off_max, c->rsp_cec);
}

static void evt_begin(void *ctx, uint64_t arg);

static int64_t widening(const periph_t *p, int64_t at_local)
{
	int64_t span = at_local - p->wbase_local;
	if (span < 0)
		span = -span;
	return span * (int64_t)(g_cfg.sca_c_ppm + g_cfg.sca_p_ppm) / 1000000 + W_JITTER;
}

static void schedule_begin(periph_t *p, int64_t evt)
{
	int64_t anchor = p->ref_local + (evt - p->ref_evt) * p->iso_ns;
	int64_t at = anchor - widening(p, anchor) - BEGIN_MARGIN;
	sim_ns_t t = clk_true(&p->clk, at);
	if (t < sim_now())
		t = sim_now();
	sim_at(t, evt_begin, p, ((uint64_t)p->gen << 32) | (uint32_t)evt);
}

static void rx_cis_ind(periph_t *p, const uint8_t *d, sim_ns_t t_true, uint16_t evt)
{
	cis_par_t *c = &p->par;
	if (p->st != CIS_RSP_SENT) {
		viol(V_LLCP, who(p), "LL_CIS_IND without a preceding LL_CIS_RSP (state %d)", p->st);
		return;
	}
	c->aa = get32(d);
	c->cis_off = get24(d + 4);
	c->cig_sync = get24(d + 7);
	c->cis_sync = get24(d + 10);
	c->cec = get16(d + 13);
	slog(who(p), "rx LL_CIS_IND aa %08x off %u cig_sync %u cis_sync %u cec %u (acl evt %u)", c->aa, c->cis_off,
			c->cig_sync, c->cis_sync, c->cec, evt);
	const char *aa_err = aa_check(c->aa);
	if (aa_err)
		viol(V_PARAM, who(p), "CIS access address %08x invalid: %s", c->aa, aa_err);
	if (c->aa == p->acl_aa)
		viol(V_PARAM, who(p), "CIS access address equals the ACL access address");
	if (c->cis_off < c->rsp_off_min || c->cis_off > c->rsp_off_max)
		viol(V_PARAM, who(p), "CIS_Offset %u outside LL_CIS_RSP range %u..%u", c->cis_off, c->rsp_off_min,
				c->rsp_off_max);
	bool enc = p->enc == ENC_ON;
	uint32_t iso_us = c->iso_int * 1250u;
	uint32_t min_sync = (uint32_t)(c->nse - 1) * c->sub_us + subevent_len_us(c, enc);
	if (c->cis_sync < min_sync)
		viol(V_PARAM, who(p), "CIS_Sync_Delay %u shorter than the CIS event (%u)", c->cis_sync, min_sync);
	if (c->cig_sync < c->cis_sync)
		viol(V_PARAM, who(p), "CIG_Sync_Delay %u < CIS_Sync_Delay %u", c->cig_sync, c->cis_sync);
	if (c->cig_sync > iso_us)
		warn(who(p), "CIG_Sync_Delay %u longer than ISO interval", c->cig_sync);
	if ((int16_t)(c->cec - evt) <= 0)
		warn(who(p), "connEventCount %u not after the ACL event %u that carried LL_CIS_IND", c->cec, evt);
	if (!p->acl_synced) {
		viol(V_TIMING, who(p), "LL_CIS_IND before any ACL anchor");
		return;
	}
	int64_t acl_anchor = p->acl_ref_local + (int64_t)(int16_t)(c->cec - p->acl_ref_evt) * p->acl_interval_us * 1000;
	int64_t first = acl_anchor + (int64_t)c->cis_off * 1000;
	p->iso_ns = (int64_t)iso_us * 1000;
	p->sub_ns = (int64_t)c->sub_us * 1000;
	p->ref_local = first;
	p->ref_evt = 0;
	p->wbase_local = p->acl_ref_local;
	if (clk_true(&p->clk, first - widening(p, first)) <= t_true) {
		viol(V_TIMING, who(p), "first CIS anchor (ACL evt %u + %u us) already passed when LL_CIS_IND arrived",
				c->cec, c->cis_off);
		p->st = CIS_DONE;
		return;
	}
	p->cis_enc = enc;
	memcpy(p->cis_iv, p->iv_le, 8);
	for (int i = 0; i < 4; i++)
		p->cis_iv[i] ^= (uint8_t)(c->aa >> (8 * i));
	p->rx_next = p->tx_next = 0;
	p->last_rx_payload = -1;
	p->tx_sent_cur = false;
	memset(p->txr, 0, sizeof(p->txr));
	p->sdu_group = -1;
	p->evt_open = false;
	p->established = false;
	p->last_crc_ok = t_true;
	p->st = CIS_PENDING;
	p->gen++;
	schedule_begin(p, 0);
}

static void rx_cis_terminate(periph_t *p, const uint8_t *d)
{
	slog(who(p), "rx LL_CIS_TERMINATE_IND cig %u cis %u reason 0x%02x", d[0], d[1], d[2]);
	if (p->st != CIS_ACTIVE && p->st != CIS_PENDING) {
		viol(V_LLCP, who(p), "LL_CIS_TERMINATE_IND for a CIS that is not active");
		return;
	}
	if (d[0] != p->par.cig_id || d[1] != p->par.cis_id)
		viol(V_FORMAT, who(p), "LL_CIS_TERMINATE_IND ids %u/%u differ from CIS %u/%u", d[0], d[1],
				p->par.cig_id, p->par.cis_id);
	if (d[2] == 0)
		viol(V_FORMAT, who(p), "LL_CIS_TERMINATE_IND with ErrorCode 0");
	p->st = CIS_DONE;
	p->gen++;
}

static const struct {
	uint8_t op, len;
} k_len[] = {
	{ LL_TERMINATE_IND, 2 }, { LL_ENC_REQ, 23 }, { LL_START_ENC_RSP, 1 }, { LL_UNKNOWN_RSP, 2 },
	{ LL_FEATURE_REQ, 9 }, { LL_REJECT_EXT_IND, 3 }, { LL_CIS_REQ, 36 }, { LL_CIS_RSP, 9 },
	{ LL_CIS_IND, 16 }, { LL_CIS_TERMINATE_IND, 4 },
};

void periph_acl_rx_ctrl(periph_t *p, const uint8_t *ctr, uint8_t len, sim_ns_t t_true, uint16_t evt)
{
	if (p->gone || len < 1)
		return;
	uint8_t op = ctr[0];
	for (unsigned i = 0; i < sizeof(k_len) / sizeof(k_len[0]); i++) {
		if (k_len[i].op == op && k_len[i].len != len) {
			viol(V_FORMAT, who(p), "LLCP opcode 0x%02x with length %u, expected %u", op, len, k_len[i].len);
			send_unknown(p, op);
			return;
		}
	}
	if (p->enc == ENC_STARTING && op != LL_START_ENC_RSP && op != LL_TERMINATE_IND)
		viol(V_LLCP, who(p), "LLCP opcode 0x%02x during encryption start", op);
	switch (op) {
	case LL_ENC_REQ: {
		if (p->enc != ENC_OFF)
			viol(V_LLCP, who(p), "LL_ENC_REQ while encryption already started");
		uint8_t skdp[8], ivp[4];
		rng_bytes(&p->rng, skdp, 8);
		rng_bytes(&p->rng, ivp, 4);
		ble_session_key(p->ltk_le, ctr + 11, skdp, p->sk);
		memcpy(p->iv_le, ctr + 19, 4);
		memcpy(p->iv_le + 4, ivp, 4);
		uint8_t b[13] = { LL_ENC_RSP };
		memcpy(b + 1, skdp, 8);
		memcpy(b + 9, ivp, 4);
		txq_push(p, b, 13);
		uint8_t s = LL_START_ENC_REQ;
		txq_push(p, &s, 1);
		p->enc = ENC_STARTING;
		slog(who(p), "rx LL_ENC_REQ -> LL_ENC_RSP, LL_START_ENC_REQ");
		break;
	}
	case LL_START_ENC_RSP:
		if (p->enc != ENC_STARTING) {
			viol(V_LLCP, who(p), "unexpected LL_START_ENC_RSP");
			break;
		}
		{
			uint8_t s = LL_START_ENC_RSP;
			txq_push(p, &s, 1);
		}
		p->enc = ENC_ON;
		slog(who(p), "ACL encryption on");
		break;
	case LL_FEATURE_REQ: {
		uint8_t b[9] = { LL_FEATURE_RSP };
		put64(b + 1, (1ull << 0) | (1ull << 29) | (1ull << 32));
		txq_push(p, b, 9);
		break;
	}
	case LL_CIS_REQ:
		rx_cis_req(p, ctr + 1);
		break;
	case LL_CIS_IND:
		rx_cis_ind(p, ctr + 1, t_true, evt);
		break;
	case LL_CIS_TERMINATE_IND:
		rx_cis_terminate(p, ctr + 1);
		break;
	case LL_TERMINATE_IND:
		slog(who(p), "rx LL_TERMINATE_IND 0x%02x", ctr[1]);
		periph_acl_lost(p);
		break;
	case LL_UNKNOWN_RSP:
		slog(who(p), "rx LL_UNKNOWN_RSP for 0x%02x", ctr[1]);
		break;
	case LL_CIS_RSP:
		viol(V_LLCP, who(p), "Central sent LL_CIS_RSP (Peripheral-only PDU)");
		break;
	default:
		slog(who(p), "rx LLCP 0x%02x - not supported, LL_UNKNOWN_RSP", op);
		send_unknown(p, op);
		break;
	}
}

/* ---------------- CIS events ---------------- */
static void deliver_sdu(periph_t *p, int64_t group)
{
	if (p->sdu_bad || p->sdu_len == 0) {
		if (p->sdu_bad)
			g_m.sdu_corrupt++;
		return;
	}
	uint32_t seq;
	sim_ns_t gen;
	if (p->sdu_len != g_cfg.max_sdu_c2p || !sdu_check(p->sdu, p->sdu_len, 0x5A, &seq, &gen)) {
		g_m.sdu_corrupt++;
		viol(V_DESYNC, who(p), "SDU content corrupt (len %u) - payload/counter mix-up", p->sdu_len);
		return;
	}
	g_m.sdu_rx++;
	if (p->have_seq) {
		if (seq == p->last_seq || (int32_t)(seq - p->last_seq) < 0) {
			g_m.sdu_dup++;
			viol(V_DESYNC, who(p), "SDU seq %u delivered after %u (duplicate/reorder)", seq, p->last_seq);
		} else if (seq != p->last_seq + 1) {
			g_m.sdu_gap += seq - p->last_seq - 1;
		}
	}
	p->have_seq = true;
	p->last_seq = seq;
	/* SDU synchronization reference (unframed): the anchor of the group event + CIS_Sync_Delay + (FT-1) * ISO. */
	int64_t anchor = p->ref_local + (group - p->ref_evt) * p->iso_ns;
	sim_ns_t ref = clk_true(&p->clk, anchor) + US(p->par.cis_sync) + (int64_t)(p->par.ft_c - 1) * p->iso_ns;
	sim_ns_t lat = ref - gen;
	if (!p->have_lat) {
		p->have_lat = true;
		p->lat_first = lat;
	}
	stat_add(&g_m.lat_ns, lat);
	if (llabs(lat - p->lat_first) > p->iso_ns / 2)
		g_m.sdu_late++;
}

static void flush_upto(periph_t *p, int64_t e)
{
	cis_par_t *c = &p->par;
	if (c->bn_c) {
		int64_t min_rx = (e - c->ft_c + 2) * c->bn_c;
		while (p->rx_next < min_rx) {
			if (p->last_rx_payload != p->rx_next)
				g_m.c2p_flushed++;
			p->rx_next++;
		}
	}
	if (c->bn_p) {
		int64_t min_tx = (e - c->ft_p + 2) * c->bn_p;
		while (p->tx_next < min_tx) {
			txp_t *t = &p->txr[p->tx_next % TXR];
			if (t->valid)
				g_m.p2c_flushed++;
			t->valid = false;
			p->tx_next++;
			p->tx_sent_cur = false;
		}
	}
}

static void evt_finalize(periph_t *p)
{
	if (!p->evt_open)
		return;
	p->evt_open = false;
	g_m.cis_events++;
	if (!p->evt_got_any)
		g_m.events_missed++;
	flush_upto(p, p->cur_evt);
}

static void make_uplink(periph_t *p, int64_t e)
{
	cis_par_t *c = &p->par;
	if (!g_cfg.uplink || !c->bn_p || !c->max_sdu_p || c->sdu_int_p != c->iso_int * 1250u)
		return;
	uint8_t sdu[4096];
	uint16_t len = c->max_sdu_p;
	int64_t anchor = p->ref_local + (e - p->ref_evt) * p->iso_ns;
	sdu_fill(sdu, len, p->up_seq++, clk_true(&p->clk, anchor) - MS(1), (uint8_t)(0xA5 + p->idx));
	unsigned segs = (len + c->max_pdu_p - 1) / c->max_pdu_p;
	if (segs > c->bn_p) {
		warn(who(p), "uplink SDU needs %u segments > BN %u", segs, c->bn_p);
		return;
	}
	for (unsigned i = 0; i < c->bn_p; i++) {
		txp_t *t = &p->txr[(e * c->bn_p + i) % TXR];
		memset(t, 0, sizeof(*t));
		if (i >= segs)
			continue;
		unsigned off = i * c->max_pdu_p;
		unsigned n = len - off < c->max_pdu_p ? len - off : c->max_pdu_p;
		t->valid = true;
		t->llid = (i == segs - 1) ? 0 : 1;
		t->len = (uint8_t)n;
		memcpy(t->data, sdu + off, n);
	}
}

static void cis_stop(periph_t *p, const char *why)
{
	slog(who(p), "CIS stopped: %s", why);
	evt_finalize(p);
	p->st = CIS_DONE;
	p->gen++;
}

static void evt_begin(void *ctx, uint64_t arg)
{
	periph_t *p = ctx;
	if ((uint32_t)(arg >> 32) != p->gen || p->gone || (p->st != CIS_PENDING && p->st != CIS_ACTIVE))
		return;
	int64_t e = (uint32_t)arg;
	evt_finalize(p);
	if (p->st == CIS_PENDING && e >= 6) {
		viol(V_TIMING, who(p), "CIS not established: no packet from Central in the first 6 CIS events");
		cis_stop(p, "establishment failed");
		return;
	}
	if (sim_now() - p->last_crc_ok > MS(g_cfg.sup_timeout_ms)) {
		g_m.p_cis_lost[p->idx] = true;
		cis_stop(p, "supervision timeout");
		return;
	}
	p->cur_evt = e;
	p->evt_open = true;
	p->evt_closed = false;
	p->evt_got_any = false;
	p->next_se = 0;
	p->cur_anchor_local = p->ref_local + (e - p->ref_evt) * p->iso_ns;
	csa2_init(&p->csa, p->par.aa, p->chan_map);
	p->chans[0] = csa2_event(&p->csa, (uint16_t)e);
	for (int se = 1; se < p->par.nse && se < 32; se++)
		p->chans[se] = csa2_subevent(&p->csa);
	make_uplink(p, e);
	int64_t last_end = (int64_t)(p->par.nse - 1) * p->sub_ns + US(subevent_len_us(&p->par, p->cis_enc));
	int64_t next_anchor = p->cur_anchor_local + p->iso_ns;
	if (last_end + widening(p, next_anchor) * 2 + BEGIN_MARGIN > p->iso_ns)
		warn(who(p), "CIS event %lld leaves no gap before the next one", (long long)e);
	schedule_begin(p, e + 1);
}

static void seg_rx(periph_t *p, int64_t payload, uint8_t llid, const uint8_t *data, uint8_t len)
{
	int64_t group = payload / p->par.bn_c;
	if (group != p->sdu_group) {
		p->sdu_group = group;
		p->sdu_len = 0;
		p->sdu_bad = false;
	}
	if (len == 0 && llid == 0 && p->sdu_len == 0) {
		g_m.c2p_empty++;
		return;
	}
	if (p->sdu_len + len > sizeof(p->sdu)) {
		p->sdu_bad = true;
		return;
	}
	memcpy(p->sdu + p->sdu_len, data, len);
	p->sdu_len = (uint16_t)(p->sdu_len + len);
	if (llid == 0) {
		deliver_sdu(p, group);
		p->sdu_len = 0;
		p->sdu_group = -1;
	}
}

bool periph_cis_rx(periph_t *p, uint32_t aa, uint8_t chan, uint8_t phy, sim_ns_t t_start, sim_ns_t t_end,
		const cis_pdu_t *pdu, air_res_t res, cis_pdu_t *resp, sim_ns_t *t_resp, uint8_t *resp_phy)
{
	cis_par_t *c = &p->par;
	if (!periph_has_aa(p, aa) || res == AIR_MISS)
		return false;
	int64_t L = clk_local(&p->clk, t_start);
	if (!p->evt_open || p->evt_closed) {
		viol(V_TIMING, who(p), "packet at %lld outside any open CIS event (evt %lld, closed %d)",
				(long long)L, (long long)p->cur_evt, p->evt_closed);
		return false;
	}
	int64_t rel = L - p->cur_anchor_local;
	int se = 0;
	if (c->nse > 1 && p->sub_ns > 0) {
		int64_t r = rel + p->sub_ns / 2;
		se = r < 0 ? 0 : (int)(r / p->sub_ns);
	}
	int64_t exp = p->cur_anchor_local + se * p->sub_ns;
	int64_t w = widening(p, exp);
	if (se >= c->nse || se < p->next_se || llabs(L - exp) > w) {
		viol(V_TIMING, who(p), "evt %lld: packet %+lld ns from subevent %d window centre (widening %lld ns, "
				"next_se %u)", (long long)p->cur_evt, (long long)(L - exp), se, (long long)w, p->next_se);
		return false;
	}
	if (chan != p->chans[se]) {
		viol(V_CHANNEL, who(p), "evt %lld se %d: Central on channel %u, CSA#2 gives %u", (long long)p->cur_evt,
				se, chan, p->chans[se]);
		return false;
	}
	if (phy != c->phy_c) {
		viol(V_PARAM, who(p), "Central transmits on PHY %u, CIS uses %u", phy, c->phy_c);
		return false;
	}
	p->next_se = (uint8_t)(se + 1);
	p->evt_got_any = true;
	bool crc_ok = res == AIR_OK;
	bool central_cie = false;
	int64_t e = p->cur_evt;

	if (crc_ok) {
		p->last_crc_ok = sim_now();
		if (p->st == CIS_PENDING) {
			p->st = CIS_ACTIVE;
			p->established = true;
			g_m.p_established[p->idx] = true;
			slog(who(p), "CIS established at evt %lld", (long long)e);
		}
		/* Re-synchronizing the anchor from the received packet. */
		p->ref_local = L - se * p->sub_ns;
		p->ref_evt = e;
		p->wbase_local = p->ref_local;
		p->cur_anchor_local = p->ref_local;

		uint8_t h = pdu->hdr0;
		uint8_t llid = h & HDR_LLID_MASK;
		bool npi = h & HDR_NPI;
		central_cie = h & HDR_CIE;
		bool rx_nesn = h & HDR_NESN, rx_sn = h & HDR_SN;
		bool fmt_ok = true;
		if (h & HDR_RFU_MASK) {
			viol(V_FORMAT, who(p), "evt %lld: header RFU bits set (0x%02x)", (long long)e, h);
			fmt_ok = false;
		}
		if (npi && pdu->len) {
			viol(V_FORMAT, who(p), "evt %lld: NPI PDU with length %u", (long long)e, pdu->len);
			fmt_ok = false;
		}
		if (!npi && llid > 1) {
			viol(V_FORMAT, who(p), "evt %lld: LLID %u on an unframed CIS", (long long)e, llid);
			fmt_ok = false;
		}
		unsigned max_len = c->max_pdu_c + (p->cis_enc && pdu->len ? 4u : 0u);
		if (pdu->len > max_len) {
			viol(V_FORMAT, who(p), "evt %lld: length %u > Max_PDU %u", (long long)e, pdu->len, max_len);
			fmt_ok = false;
		}
		/* Acknowledgement of our payload. */
		if (rx_nesn != (bool)(p->tx_next & 1)) {
			if (p->tx_sent_cur) {
				p->txr[p->tx_next % TXR].valid = false;
				p->tx_next++;
				p->tx_sent_cur = false;
				g_m.p2c_acked++;
			} else {
				viol(V_DESYNC, who(p), "evt %lld se %d: NESN %d acknowledges payload %lld that was never sent",
						(long long)e, se, rx_nesn, (long long)p->tx_next);
			}
		}
		if (npi) {
			g_m.c2p_null++;
		} else if (fmt_ok) {
			if (rx_sn == (bool)(p->rx_next & 1)) {
				if (c->bn_c == 0 || p->rx_next >= (e + 1) * c->bn_c) {
					viol(V_DESYNC, who(p), "evt %lld: new payload %lld not scheduled in this event", (long long)e,
							(long long)p->rx_next);
				} else {
					uint8_t plain[CIS_PDU_MAX_PAYLOAD];
					uint8_t plen = pdu->len;
					bool ok = true;
					if (p->cis_enc && pdu->len > 0) {
						if (pdu->len < 4) {
							viol(V_FORMAT, who(p), "evt %lld: encrypted PDU shorter than MIC", (long long)e);
							ok = false;
						} else {
							uint8_t nonce[13];
							ble_nonce(nonce, (uint64_t)p->rx_next, true, p->cis_iv);
							ok = ble_ccm_decrypt(p->sk, nonce, h & CCM_AAD_MASK, pdu->payload, pdu->len, plain);
							plen = (uint8_t)(pdu->len - 4);
							if (!ok)
								viol(V_MIC, who(p), "evt %lld se %d: MIC failure on payload %lld", (long long)e, se,
										(long long)p->rx_next);
						}
					} else {
						memcpy(plain, pdu->payload, pdu->len);
					}
					if (ok) {
						g_m.c2p_new++;
						p->last_rx_payload = p->rx_next;
						seg_rx(p, p->rx_next, llid, plain, plen);
						p->rx_next++;
					}
				}
			} else if (p->last_rx_payload == p->rx_next - 1) {
				g_m.c2p_dup++;
			} else {
				viol(V_DESYNC, who(p), "evt %lld se %d: SN %d but expected %d and payload %lld was never received "
						"(SN not advanced at flush point?)", (long long)e, se, rx_sn, (int)(p->rx_next & 1),
						(long long)(p->rx_next - 1));
			}
		}
	}

	/* Response after T_IFS (also after a CRC error - AA detected). */
	memset(resp, 0, sizeof(*resp));
	bool can_tx = c->bn_p && p->tx_next < (e + 1) * c->bn_p && p->txr[p->tx_next % TXR].valid;
	uint8_t hdr = (uint8_t)((p->rx_next & 1) ? HDR_NESN : 0) | (uint8_t)((p->tx_next & 1) ? HDR_SN : 0);
	if (can_tx) {
		txp_t *t = &p->txr[p->tx_next % TXR];
		hdr |= t->llid;
		resp->hdr0 = hdr;
		if (p->cis_enc && t->len) {
			uint8_t nonce[13];
			ble_nonce(nonce, (uint64_t)p->tx_next, false, p->cis_iv);
			ble_ccm_encrypt(p->sk, nonce, hdr & CCM_AAD_MASK, t->data, t->len, resp->payload);
			resp->len = (uint8_t)(t->len + 4);
		} else {
			memcpy(resp->payload, t->data, t->len);
			resp->len = t->len;
		}
		if (!p->tx_sent_cur)
			g_m.p2c_sent_payloads++;
		p->tx_sent_cur = true;
	} else {
		resp->hdr0 = hdr | HDR_NPI;
		resp->len = 0;
	}
	int64_t resp_local = clk_local(&p->clk, t_end) + US(T_IFS_US);
	*t_resp = clk_true(&p->clk, resp_local);
	*resp_phy = c->phy_p;
	if ((central_cie && crc_ok) || se == c->nse - 1)
		p->evt_closed = true;
	return true;
}

void periph_finish(periph_t *p)
{
	if (p->st == CIS_ACTIVE)
		evt_finalize(p);
}
