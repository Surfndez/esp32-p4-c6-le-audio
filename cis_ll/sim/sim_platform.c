/*
 * CIS engine platform in the simulator: an implementation of cis_platform_ops_t + an ideal C6 radio.
 *
 * The radio calls prepare_tx/rx_done/event_closed/event_skipped at simulated times according to the link parameters
 * (anchor, ISO interval, Sub_Interval, NSE), exchanges PDUs with the Peripheral through the air model, and computes
 * the CSA#2 channel from the AA and channel map of the link. Collisions with the ACL/another CIS or Wi-Fi
 * coexistence -> the subevent is not served; none served -> event_skipped. The engine work (cis_engine_process) is
 * run by the "LL task" with a task_lat_us delay.
 */
#include "sim.h"

#include <stdlib.h>
#include <string.h>

#include "cis_engine.h"

#define MAX_LINKS 4

struct cis_link {
	int              id;
	bool             open;
	uint32_t         gen;
	cis_link_params_t par;
	cis_link_cb_t    cb;
	void            *arg;
	int64_t          anchor0_us;  /* local C6 clock, without wraparound */
	uint32_t         evt;
	bool             sn, nesn;    /* hardware state with radio_owns_seq */
	uint8_t          last_hdr;    /* SN/NESN bits of the last engine PDU (for the radio null PDU) */
	cis_pdu_t        tx, rx;
	bool             rx_valid, rx_crc;
	sim_ns_t         rx_start;
	bool             any_served, first_checked, streaming;
	uint8_t          chans[32];
	/* prep_ahead: the PDUs of all subevents of event ahead_evt (prepared before the first, as on the C6) */
	cis_pdu_t        ahead[32];
	bool             ahead_ok[32];
	uint32_t         ahead_evt;
	bool             ahead_valid;
};

static sim_clock_t        s_cclk;
static cis_engine_t      *s_eng;
static void              *s_mem;
static rng_t              s_rng;
static bool               s_proc_pending;
static uint32_t           s_timer_gen;
static struct cis_link    s_links[MAX_LINKS];
static int                s_nlinks;
static sim_ns_t           s_up_last;
static cis_platform_ops_t s_ops;

__attribute__((weak)) const char *sim_engine_name(void);

const char *plat_engine_name(void)
{
	return sim_engine_name ? sim_engine_name() : "cis_ll engine";
}

const sim_clock_t *plat_c_clock(void)
{
	return &s_cclk;
}

int64_t plat_c_local_us64(sim_ns_t t)
{
	int64_t l = clk_local(&s_cclk, t);
	return l >= 0 ? l / 1000 : -((-l + 999) / 1000);
}

sim_ns_t plat_c_true_of_us64(int64_t us)
{
	return clk_true(&s_cclk, us * 1000);
}

static int64_t unwrap_us(uint32_t us)
{
	int64_t now = plat_c_local_us64(sim_now());
	return now + (int32_t)(us - (uint32_t)now);
}

/* ---------------- LL task ---------------- */
static void proc_cb(void *ctx, uint64_t arg)
{
	(void)ctx;
	(void)arg;
	s_proc_pending = false;
	cis_engine_process(s_eng);
}

void plat_kick(void)
{
	if (s_proc_pending)
		return;
	s_proc_pending = true;
	sim_at(sim_now() + US(g_cfg.task_lat_us), proc_cb, NULL, 0);
}

/* ---------------- C6 -> P4 transport ---------------- */
typedef struct {
	uint16_t len;
	uint8_t  b[];
} blob_t;

static void up_cb(void *ctx, uint64_t arg)
{
	(void)arg;
	blob_t *b = ctx;
	host_rx_h4(b->b, b->len);
	free(b);
}

static void op_hci_to_host(void *ctx, const uint8_t *h4, uint16_t len)
{
	(void)ctx;
	bool ok = false;
	if (len >= 3 && h4[0] == 0x04)
		ok = len == 3u + h4[2];
	else if (len >= 5 && h4[0] == 0x05)
		ok = len == 5u + (get16(h4 + 3) & 0x3FFF);
	if (!ok) {
		viol(V_CONTRACT, "C6", "hci_to_host: malformed H4 packet (type 0x%02x len %u)", len ? h4[0] : 0, len);
		return;
	}
	blob_t *b = malloc(sizeof(*b) + len);
	if (!b)
		abort();
	b->len = len;
	memcpy(b->b, h4, len);
	sim_ns_t t = sim_now() + rng_range(&s_rng, g_cfg.up_lat_min, g_cfg.up_lat_max);
	if (t <= s_up_last)
		t = s_up_last + US(5);
	s_up_last = t;
	sim_at(t, up_cb, b, 0);
}

void plat_ctrl_evt(const uint8_t *h4, uint16_t len)
{
	if (!cis_engine_hci_evt_from_controller(s_eng, h4, len))
		op_hci_to_host(NULL, h4, len);
	plat_kick();
}

void plat_c6_rx_h4(const uint8_t *h4, uint16_t len)
{
	if (h4[0] == 0x01) {
		uint16_t op = get16(h4 + 1);
		bool eaten = cis_engine_hci_cmd(s_eng, h4, len);
		if (eaten && op == 0x2019)
			viol(V_CONTRACT, "C6", "engine consumed LE Enable Encryption (contract: peek only)");
		if (!eaten)
			ctrl_hci_cmd(h4, len);
	} else if (h4[0] == 0x05) {
		cis_engine_hci_iso(s_eng, h4, len);
	}
	plat_kick();
}

void plat_ll_ctrl_rx(uint16_t handle, const uint8_t *ctr, uint8_t len)
{
	if (!cis_engine_ll_ctrl_rx(s_eng, handle, ctr, len))
		ctrl_ll_rx(handle, ctr, len);
	plat_kick();
}

void plat_ll_ctrl_tx_seen(uint16_t handle, const uint8_t *ctr, uint8_t len)
{
	cis_engine_ll_ctrl_tx_seen(s_eng, handle, ctr, len);
	plat_kick();
}

/* ---------------- ideal CIS radio ---------------- */
enum { EV_PREP, EV_TX, EV_RX };

static void radio_ev(void *ctx, uint64_t arg);

static uint64_t ev_arg(const struct cis_link *l, int kind, uint8_t se)
{
	return ((uint64_t)l->gen << 32) | ((uint64_t)kind << 8) | se;
}

static int64_t anchor_us(const struct cis_link *l, uint32_t evt)
{
	return l->anchor0_us + (int64_t)evt * l->par.iso_interval_us;
}

static sim_ns_t se_start(const struct cis_link *l, uint32_t evt, uint8_t se)
{
	return plat_c_true_of_us64(anchor_us(l, evt) + (int64_t)se * l->par.sub_interval_us) + g_cfg.tx_skew_ns;
}

static void sched_prep(struct cis_link *l, uint8_t se)
{
	sim_ns_t t = se_start(l, l->evt, se) - US(g_cfg.tx_lead_us);
	if (t < sim_now())
		t = sim_now();
	sim_at(t, radio_ev, l, ev_arg(l, EV_PREP, se));
}

static void next_event(struct cis_link *l)
{
	l->evt++;
	sched_prep(l, 0);
}

static void end_event(struct cis_link *l)
{
	uint32_t e = l->evt;
	if (l->any_served)
		l->cb.event_closed(l->arg, e);
	else {
		g_m.radio_skipped++;
		l->cb.event_skipped(l->arg, e);
	}
	plat_kick();
	if (l->open)
		next_event(l);
}

static uint32_t reserved_us(const struct cis_link *l)
{
	return air_time_us(l->par.phy_c_to_p, l->par.max_pdu_c_to_p) + T_IFS_US +
			air_time_us(l->par.phy_p_to_c, l->par.max_pdu_p_to_c);
}

static periph_t *peer_of(uint32_t aa)
{
	for (int i = 0; i < acl_count(); i++)
		if (periph_has_aa(acl_periph(i), aa))
			return acl_periph(i);
	return NULL;
}

static void do_prep(struct cis_link *l, uint8_t se)
{
	if (se == 0) {
		g_m.radio_events++;
		l->any_served = false;
		l->first_checked = false;
		if (l->par.nse > 32)
			return;
		csa2_t c;
		csa2_init(&c, l->par.access_address, l->par.chan_map);
		l->chans[0] = csa2_event(&c, (uint16_t)l->evt);
		for (int i = 1; i < l->par.nse; i++)
			l->chans[i] = g_cfg.subevent_hop ? csa2_subevent(&c) : l->chans[0];
	}
	sim_ns_t s = se_start(l, l->evt, se);
	sim_ns_t e = s + US(reserved_us(l));
	bool blocked = air_coex_blocked(s, e);
	if (blocked)
		g_m.coex_blocked++;
	else if (acl_window_overlap(s, e) || !radio_claim(l->id, s, e, false))
		blocked = true;
	if (blocked) {
		if (se + 1 < l->par.nse)
			sched_prep(l, (uint8_t)(se + 1));
		else
			end_event(l);
		return;
	}
	bool have;
	if (g_cfg.prep_ahead && l->par.nse > 1) {
		/* C6 at NSE > 1: a separate pseudo-connection per subevent, PDUs loaded between events - all
		 * before the first subevent, without knowing its outcome (cis_radio_caps_t.subevents_prepared_ahead). */
		if (!l->ahead_valid || l->ahead_evt != l->evt) {
			for (int k = 0; k < l->par.nse && k < 32; k++) {
				memset(&l->ahead[k], 0, sizeof(l->ahead[k]));
				l->ahead_ok[k] = l->cb.prepare_tx(l->arg, l->evt, (uint8_t)k, &l->ahead[k]);
			}
			l->ahead_evt = l->evt;
			l->ahead_valid = true;
		}
		l->tx = l->ahead[se < 32 ? se : 0];
		have = l->ahead_ok[se < 32 ? se : 0];
	} else {
		memset(&l->tx, 0, sizeof(l->tx));
		have = l->cb.prepare_tx(l->arg, l->evt, se, &l->tx);
	}
	plat_kick();
	if (!l->open)
		return;
	if (!have) {
		/* Contract: on a live link the engine builds the null PDU itself (NESN, CIE) and returns true. The radio
		 * sends the null PDU anyway: NPI, SN/NESN bits as in the last engine PDU (the sequencer remembers them). */
		viol(V_CONTRACT, "radio", "prepare_tx returned false on an open link (evt %u se %u)", l->evt, se);
		l->tx.hdr0 = (uint8_t)(HDR_NPI | (l->last_hdr & (HDR_SN | HDR_NESN)));
		l->tx.len = 0;
	} else {
		l->last_hdr = l->tx.hdr0;
		if (l->tx.len > l->par.max_pdu_c_to_p || l->tx.len > CIS_PDU_MAX_PAYLOAD)
			viol(V_CONTRACT, "radio", "prepare_tx: PDU length %u > max_pdu_c_to_p %u", l->tx.len,
					l->par.max_pdu_c_to_p);
	}
	sim_at(s, radio_ev, l, ev_arg(l, EV_TX, se));
}

static void do_tx(struct cis_link *l, uint8_t se)
{
	cis_pdu_t pdu = l->tx;
	if (g_cfg.owns_seq)
		pdu.hdr0 = (uint8_t)((pdu.hdr0 & ~(HDR_SN | HDR_NESN)) | (l->sn ? HDR_SN : 0) | (l->nesn ? HDR_NESN : 0));
	if (!l->first_checked) {
		l->first_checked = true;
		bool data = !(pdu.hdr0 & HDR_NPI) && pdu.len > 0;
		if (data) {
			l->streaming = true;
			g_m.data_pdus++;
		} else {
			g_m.null_pdus++;
			sim_ns_t stop = host_stream_stop_time();
			if (l->streaming && (stop == 0 || sim_now() < stop))
				g_m.underruns++;
		}
	}
	if (pdu.len > 0 && air_mic_corrupt())
		pdu.payload[0] ^= 0x01;
	l->any_served = true;
	g_m.radio_subevents++;
	sim_ns_t t0 = sim_now();
	sim_ns_t t_end = t0 + US(air_time_us(l->par.phy_c_to_p, pdu.len));
	periph_t *p = peer_of(l->par.access_address);
	if (!p && l->evt < 8)
		slog("radio", "link %d evt %u: no Peripheral listens on AA %08x", l->id, l->evt, l->par.access_address);
	air_res_t r = air_packet();
	cis_pdu_t resp;
	sim_ns_t t_resp = 0;
	uint8_t rphy = 0;
	bool got = p && periph_cis_rx(p, l->par.access_address, l->chans[se < 32 ? se : 0], l->par.phy_c_to_p, t0,
			t_end, &pdu, r, &resp, &t_resp, &rphy);
	l->rx_valid = false;
	l->rx_crc = false;
	sim_ns_t done = t_end + US(T_IFS_US + 40);
	if (got) {
		sim_ns_t r_end = t_resp + US(air_time_us(rphy, resp.len));
		air_res_t r2 = rphy == l->par.phy_p_to_c ? air_packet() : AIR_MISS;
		if (r2 != AIR_MISS) {
			l->rx = resp;
			l->rx_valid = true;
			l->rx_crc = r2 == AIR_OK;
			l->rx_start = t_resp;
			done = r_end;
		}
	}
	sim_at(done, radio_ev, l, ev_arg(l, EV_RX, se));
}

static void do_rx(struct cis_link *l, uint8_t se)
{
	if (g_cfg.owns_seq && l->rx_valid && l->rx_crc) {
		/* ACL sequencer: a new packet when SN == NESN, an acknowledgement when NESN != SN (also for null PDUs). */
		if (((l->rx.hdr0 & HDR_SN) != 0) == l->nesn)
			l->nesn = !l->nesn;
		if (((l->rx.hdr0 & HDR_NESN) != 0) != l->sn)
			l->sn = !l->sn;
	}
	uint32_t ts = (uint32_t)plat_c_local_us64(l->rx_start);
	l->cb.rx_done(l->arg, l->evt, se, l->rx_valid ? &l->rx : NULL, l->rx_valid && l->rx_crc, ts);
	plat_kick();
	if (!l->open)
		return;
	bool close = (l->tx.hdr0 & HDR_CIE) || (l->rx_valid && l->rx_crc && (l->rx.hdr0 & HDR_CIE)) ||
			se + 1 >= l->par.nse;
	if (close)
		end_event(l);
	else
		sched_prep(l, (uint8_t)(se + 1));
}

static void radio_ev(void *ctx, uint64_t arg)
{
	struct cis_link *l = ctx;
	if ((uint32_t)(arg >> 32) != l->gen || !l->open)
		return;
	int kind = (int)((arg >> 8) & 0xFF);
	uint8_t se = (uint8_t)arg;
	if (kind == EV_PREP)
		do_prep(l, se);
	else if (kind == EV_TX)
		do_tx(l, se);
	else
		do_rx(l, se);
}

/* ---------------- ops ---------------- */
static int op_send_ctrl_pdu(void *ctx, uint16_t acl_handle, const uint8_t *ctr, uint8_t len)
{
	(void)ctx;
	return acl_send_from_engine(acl_handle, ctr, len);
}

static int op_acl_timing(void *ctx, uint16_t acl_handle, cis_acl_timing_t *out)
{
	(void)ctx;
	return acl_timing(acl_handle, out);
}

static bool op_acl_is_central(void *ctx, uint16_t acl_handle)
{
	(void)ctx;
	return acl_find(acl_handle) >= 0;
}

static uint64_t op_acl_remote_features(void *ctx, uint16_t acl_handle)
{
	(void)ctx;
	return acl_remote_features(acl_handle);
}

static int op_link_open(void *ctx, const cis_link_params_t *p, const cis_link_cb_t *cb, void *cb_arg,
		cis_link_t **out)
{
	(void)ctx;
	if (s_nlinks == MAX_LINKS) {
		viol(V_CONTRACT, "radio", "link_open: no free link");
		return -1;
	}
	const char *bad = NULL;
	if (!cb || !cb->prepare_tx || !cb->rx_done || !cb->event_closed || !cb->event_skipped)
		bad = "callback missing";
	else if (p->nse < 1 || p->nse > g_cfg.max_nse)
		bad = "nse outside 1..max_nse";
	else if (p->iso_interval_us < 5000 || p->iso_interval_us % 1250)
		bad = "iso_interval_us not a multiple of 1250 >= 5000";
	else if (p->nse > 1 && (p->sub_interval_us == 0 || (uint64_t)(p->nse - 1) * p->sub_interval_us >=
			p->iso_interval_us))
		bad = "sub_interval_us inconsistent with nse/iso_interval";
	else if ((p->phy_c_to_p != CIS_PHY_1M && p->phy_c_to_p != CIS_PHY_2M) ||
			(p->phy_p_to_c != CIS_PHY_1M && p->phy_p_to_c != CIS_PHY_2M))
		bad = "phy";
	else if (p->max_pdu_c_to_p > CIS_PDU_MAX_PAYLOAD || p->max_pdu_p_to_c > CIS_PDU_MAX_PAYLOAD)
		bad = "max_pdu > 251";
	else if (p->crc_init > 0xFFFFFF)
		bad = "crc_init > 24 bits";
	else if (aa_check(p->access_address))
		bad = "invalid access address";
	if (bad) {
		viol(V_CONTRACT, "radio", "link_open rejected: %s", bad);
		return -1;
	}
	struct cis_link *l = &s_links[s_nlinks];
	memset(l, 0, sizeof(*l));
	l->id = s_nlinks++;
	l->open = true;
	l->gen = 1;
	l->par = *p;
	l->cb = *cb;
	l->arg = cb_arg;
	l->anchor0_us = unwrap_us(p->first_anchor_us);
	int64_t now = plat_c_local_us64(sim_now());
	slog("radio", "link_open %d: aa %08x nse %u sub %u iso %u first anchor in %lld us, pdu %u/%u phy %u/%u",
			l->id, p->access_address, p->nse, p->sub_interval_us, p->iso_interval_us,
			(long long)(l->anchor0_us - now), p->max_pdu_c_to_p, p->max_pdu_p_to_c, p->phy_c_to_p, p->phy_p_to_c);
	if (l->anchor0_us - (int64_t)g_cfg.tx_lead_us < now)
		viol(V_CONTRACT, "radio", "link_open: first anchor %lld us in the past/too close (lead %u)",
				(long long)(l->anchor0_us - now), g_cfg.tx_lead_us);
	g_m.link_opened = true;
	l->evt = 0;
	sched_prep(l, 0);
	*out = (cis_link_t *)l;
	return 0;
}

static void op_link_close(void *ctx, cis_link_t *link)
{
	(void)ctx;
	struct cis_link *l = (struct cis_link *)link;
	if (!l || l < s_links || l >= s_links + s_nlinks) {
		viol(V_CONTRACT, "radio", "link_close with an unknown link");
		return;
	}
	if (!l->open)
		viol(V_CONTRACT, "radio", "link_close twice on link %d", l->id);
	slog("radio", "link_close %d at evt %u", l->id, l->evt);
	l->open = false;
	l->gen++;
}

static void op_radio_caps(void *ctx, cis_radio_caps_t *out)
{
	(void)ctx;
	out->max_nse = g_cfg.max_nse;
	out->radio_owns_seq = g_cfg.owns_seq;
	out->tx_prepare_lead_us = g_cfg.tx_lead_us;
	out->subevents_prepared_ahead = g_cfg.prep_ahead;
}

static uint32_t op_now_us(void *ctx)
{
	(void)ctx;
	return (uint32_t)plat_c_local_us64(sim_now());
}

static void op_aes128(void *ctx, const uint8_t key[16], const uint8_t in[16], uint8_t out[16])
{
	(void)ctx;
	aes128_encrypt(key, in, out);
}

static void op_random(void *ctx, uint8_t *buf, uint16_t len)
{
	(void)ctx;
	rng_bytes(&s_rng, buf, len);
}

static void timer_cb(void *ctx, uint64_t arg)
{
	(void)ctx;
	if ((uint32_t)arg != s_timer_gen)
		return;
	cis_engine_on_timer(s_eng);
	plat_kick();
}

static void op_timer_start(void *ctx, uint32_t delay_us)
{
	(void)ctx;
	s_timer_gen++;
	int64_t local = clk_local(&s_cclk, sim_now()) + (int64_t)delay_us * 1000;
	sim_at(clk_true(&s_cclk, local), timer_cb, NULL, s_timer_gen);
}

static void op_log(void *ctx, int level, const char *msg)
{
	(void)ctx;
	slog("eng", "[%d] %s", level, msg);
}

void plat_engine_stats(uint16_t cis_handle)
{
	cis_engine_stats_t s;
	if (!s_eng || !cis_engine_get_stats(s_eng, cis_handle, &s))
		return;
	g_m.eng_stats_ok = true;
	g_m.eng_events += s.events;
	g_m.eng_skipped += s.events_skipped;
	g_m.eng_tx_underrun += s.tx_underrun;
	g_m.eng_tx_flushed += s.tx_flushed;
	g_m.eng_sdu_dropped += s.sdu_dropped;
	g_m.eng_sdu_late += s.sdu_late;
	g_m.eng_rx_ok += s.rx_ok;
	g_m.eng_rx_mic_fail += s.rx_mic_fail;
	g_m.eng_rx_lost += s.rx_lost;
}

bool plat_init(void)
{
	s_cclk.ppb = (int64_t)g_cfg.ppm_c * 1000;
	s_cclk.offset_ns = (int64_t)g_cfg.c_clock_start_us * 1000;
	rng_seed(&s_rng, g_cfg.seed * 19 + 7);
	s_proc_pending = false;
	s_timer_gen = 0;
	s_nlinks = 0;
	s_up_last = 0;
	memset(s_links, 0, sizeof(s_links));
	s_ops = (cis_platform_ops_t){
		.ctx = NULL,
		.hci_to_host = op_hci_to_host,
		.send_ctrl_pdu = op_send_ctrl_pdu,
		.acl_timing = op_acl_timing,
		.acl_is_central = op_acl_is_central,
		.acl_remote_features = op_acl_remote_features,
		.link_open = op_link_open,
		.link_close = op_link_close,
		.radio_caps = op_radio_caps,
		.now_us = op_now_us,
		.aes128 = op_aes128,
		.random = op_random,
		.timer_start = op_timer_start,
		.log = op_log,
	};
	uint32_t sz = cis_engine_mem_size();
	size_t asz = ((size_t)sz + 15) & ~(size_t)15;
	s_mem = aligned_alloc(16, asz ? asz : 16);
	if (!s_mem)
		abort();
	memset(s_mem, 0xA5, asz ? asz : 16);
	s_eng = cis_engine_init(s_mem, sz, &s_ops);
	if (!s_eng) {
		slog("C6", "cis_engine_init(mem, %u) returned NULL", sz);
		g_m.engine_init_failed = true;
		free(s_mem);
		s_mem = NULL;
		return false;
	}
	return true;
}

void plat_finish(void)
{
	bool all_closed = true;
	for (int i = 0; i < s_nlinks; i++)
		if (s_links[i].open)
			all_closed = false;
	g_m.link_closed_all = all_closed;
	if (!s_eng)
		return;
	if (cis_engine_active(s_eng) && g_m.host_done)
		viol(V_CONTRACT, "C6", "engine still active after LE Remove CIG");
	free(s_mem);
	s_mem = NULL;
	s_eng = NULL;
}
