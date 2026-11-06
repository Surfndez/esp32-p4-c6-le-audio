/*
 * P4 host model: an HCI sequence as in a BAP unicast client (features, Set Host Feature bit 32, ISO buffers,
 * ACL encryption, Set CIG Parameters, Create CIS, Setup ISO Data Path), then an SDU every SDU_Interval
 * with SDIO jitter (FIFO, rare stalls), flow control through Number Of Completed Packets,
 * reception of the HCI ISO uplink; finally Disconnect CIS and Remove CIG.
 */
#include "sim.h"

#include <stdlib.h>
#include <string.h>

typedef enum {
	H_WAIT_CONN, H_FEAT, H_HOSTFEAT, H_BUFSZ, H_ENC, H_WAIT_ENC, H_CIG, H_CREATE, H_WAIT_EST, H_DATAPATH,
	H_STREAM, H_DISC, H_REMOVE, H_DONE,
} hst_t;

#define PEND_MAX 3
#define CIG_ID   1

typedef struct {
	uint8_t  cis;
	uint32_t seq;
	sim_ns_t gen;
} pend_t;

static struct {
	hst_t       st;
	uint16_t    pending_op;
	uint32_t    step_gen;
	int         conn_seen, enc_idx, dp_idx, disc_idx, est_n, disc_n;
	uint16_t    cis_h[MAX_BUDS];
	bool        cis_up[MAX_BUDS];
	int         credits, credits_total;
	uint32_t    k;
	uint16_t    psn[MAX_BUDS];
	sim_ns_t    tx_last;
	rng_t       rng;
	sim_clock_t hclk;
	int64_t     gen0_local;
	sim_ns_t    stream_start, stream_stop;
	pend_t      pend[PEND_MAX];
	int         pend_n;
	bool        up_have[MAX_BUDS];
	uint32_t    up_seq[MAX_BUDS];
	bool        up_psn_have[MAX_BUDS];
	uint16_t    up_psn[MAX_BUDS];
	bool        feat_cis_central;
	bool        stats_taken;
} H;

static void advance(void);

static void stop_cb(void *ctx, uint64_t arg)
{
	(void)ctx;
	(void)arg;
	sim_stop();
}

static void advance_cb(void *ctx, uint64_t arg)
{
	(void)ctx;
	(void)arg;
	advance();
}

static void fail(const char *msg)
{
	if (g_m.host_failed)
		return;
	g_m.host_failed = true;
	snprintf(g_m.host_fail_msg, sizeof(g_m.host_fail_msg), "%s", msg);
	slog("host", "FAIL: %s", msg);
	H.st = H_DONE;
	sim_at(sim_now() + MS(200), stop_cb, NULL, 0);
}

/* ---------------- P4 -> C6 transport (SDIO, FIFO) ---------------- */
typedef struct {
	uint16_t len;
	uint8_t  b[];
} blob_t;

static void down_cb(void *ctx, uint64_t arg)
{
	(void)arg;
	blob_t *b = ctx;
	plat_c6_rx_h4(b->b, b->len);
	free(b);
}

static void host_send(const uint8_t *h4, uint16_t len, bool iso)
{
	blob_t *b = malloc(sizeof(*b) + len);
	if (!b)
		abort();
	b->len = len;
	memcpy(b->b, h4, len);
	sim_ns_t d = sim_now();
	if (iso) {
		d += rng_range(&H.rng, 0, g_cfg.host_jitter_max);
		if (rng_chance(&H.rng, g_cfg.stall_prob))
			d += g_cfg.stall_len;
	} else {
		d += rng_range(&H.rng, US(20), US(200));
	}
	if (d <= H.tx_last)
		d = H.tx_last + US(20);
	H.tx_last = d;
	sim_at(d, down_cb, b, 0);
}

static void timeout_cb(void *ctx, uint64_t arg)
{
	(void)ctx;
	if ((uint32_t)arg == H.step_gen && H.st != H_DONE && H.st != H_STREAM) {
		char m[96];
		snprintf(m, sizeof(m), "timeout in host step %d (pending opcode 0x%04x)", H.st, H.pending_op);
		fail(m);
	}
}

static void arm_timeout(sim_ns_t d)
{
	H.step_gen++;
	sim_at(sim_now() + d, timeout_cb, NULL, H.step_gen);
}

static void cmd(uint16_t op, const uint8_t *p, uint8_t plen)
{
	uint8_t b[260] = { 0x01 };
	put16(b + 1, op);
	b[3] = plen;
	if (plen)
		memcpy(b + 4, p, plen);
	H.pending_op = op;
	slog("host", "cmd 0x%04x (%u B)", op, plen);
	host_send(b, (uint16_t)(4 + plen), false);
	arm_timeout(SEC(5));
}

/* ---------------- stream ---------------- */
static void send_sdu(uint8_t cis, uint32_t seq, sim_ns_t gen)
{
	uint16_t n = g_cfg.max_sdu_c2p;
	uint8_t b[5 + 4 + 4096];
	put16(b + 1, H.cis_h[cis] | (0x2u << 12)); /* PB = 10: complete SDU, TS = 0 */
	put16(b + 3, 4u + n);
	b[0] = 0x05;
	put16(b + 5, H.psn[cis]++);
	put16(b + 7, n);
	sdu_fill(b + 9, n, seq, gen, 0x5A);
	host_send(b, (uint16_t)(9 + n), true);
	H.credits--;
	g_m.host_sdu_sent++;
}

static void try_send_pending(void)
{
	while (H.pend_n && H.credits > 0) {
		pend_t p = H.pend[0];
		memmove(H.pend, H.pend + 1, sizeof(pend_t) * (size_t)(--H.pend_n));
		if (H.cis_up[p.cis])
			send_sdu(p.cis, p.seq, p.gen);
	}
}

static sim_ns_t gen_time(uint32_t k)
{
	int64_t l = H.gen0_local + (int64_t)k * g_cfg.sdu_interval_us * 1000;
	return g_cfg.host_follows_c ? clk_true(plat_c_clock(), l) : clk_true(&H.hclk, l);
}

static void stream_tick(void *ctx, uint64_t arg)
{
	(void)ctx;
	(void)arg;
	if (H.st != H_STREAM)
		return;
	sim_ns_t now = sim_now();
	if (now >= H.stream_stop) {
		H.st = H_DISC;
		H.disc_idx = 0;
		/* Let the in-flight SDUs arrive, then disconnect. */
		sim_at(now + MS(150), advance_cb, NULL, 0);
		return;
	}
	for (uint8_t c = 0; c < g_cfg.n_buds; c++) {
		if (!H.cis_up[c])
			continue;
		g_m.host_sdu_gen++;
		if (H.credits > 0 && !H.pend_n) {
			send_sdu(c, H.k, now);
		} else if (H.pend_n < PEND_MAX) {
			H.pend[H.pend_n++] = (pend_t){ c, H.k, now };
		} else {
			g_m.host_drop_credit++;
		}
	}
	H.k++;
	sim_at(gen_time(H.k), stream_tick, NULL, 0);
}

static void start_stream(void)
{
	H.st = H_STREAM;
	H.k = 0;
	sim_ns_t t0 = sim_now() + MS(3);
	H.gen0_local = g_cfg.host_follows_c ? clk_local(plat_c_clock(), t0) : clk_local(&H.hclk, t0);
	H.stream_start = gen_time(0);
	H.stream_stop = H.stream_start + g_cfg.stream_time;
	slog("host", "streaming %lld s, credits %d", (long long)(g_cfg.stream_time / SEC(1)), H.credits);
	sim_at(H.stream_start, stream_tick, NULL, 0);
}

sim_ns_t host_stream_stop_time(void)
{
	return H.stream_stop;
}

/* ---------------- step machine ---------------- */
static void advance(void)
{
	uint8_t p[64];
	switch (H.st) {
	case H_WAIT_CONN:
		if (H.conn_seen < g_cfg.n_buds)
			return;
		H.st = H_FEAT;
		cmd(0x2003, NULL, 0);
		break;
	case H_FEAT:
		H.st = H_HOSTFEAT;
		p[0] = 32;
		p[1] = 1;
		cmd(0x2074, p, 2);
		break;
	case H_HOSTFEAT:
		H.st = H_BUFSZ;
		cmd(0x2060, NULL, 0);
		break;
	case H_BUFSZ:
	case H_WAIT_ENC:
		if (H.st == H_BUFSZ)
			H.enc_idx = 0;
		if (g_cfg.encrypt && H.enc_idx < g_cfg.n_buds) {
			H.st = H_ENC;
			put16(p, acl_handle(H.enc_idx));
			for (int i = 0; i < 8; i++)
				p[2 + i] = (uint8_t)(0x90 + i);
			put16(p + 10, 0x2474);
			for (int i = 0; i < 16; i++)
				p[12 + i] = (uint8_t)(0x40 + i);
			cmd(0x2019, p, 28);
			break;
		}
		/* fallthrough */
	case H_CIG: {
		H.st = H_CIG;
		uint8_t *q = p;
		*q++ = CIG_ID;
		put24(q, g_cfg.sdu_interval_us);
		put24(q + 3, g_cfg.sdu_interval_us);
		q += 6;
		*q++ = 5; /* Worst_Case_SCA: 31-50 ppm */
		*q++ = 0; /* sequential */
		*q++ = 0; /* unframed */
		put16(q, g_cfg.max_lat_c2p_ms);
		put16(q + 2, g_cfg.max_lat_p2c_ms);
		q += 4;
		*q++ = g_cfg.n_buds;
		for (int i = 0; i < g_cfg.n_buds; i++) {
			*q++ = (uint8_t)i;
			put16(q, g_cfg.max_sdu_c2p);
			put16(q + 2, g_cfg.uplink ? g_cfg.max_sdu_p2c : 0);
			q += 4;
			*q++ = g_cfg.phy;
			*q++ = g_cfg.phy;
			*q++ = g_cfg.rtn_c2p;
			*q++ = g_cfg.rtn_p2c;
		}
		cmd(0x2062, p, (uint8_t)(q - p));
		break;
	}
	case H_CREATE: {
		p[0] = g_cfg.n_buds;
		for (int i = 0; i < g_cfg.n_buds; i++) {
			put16(p + 1 + 4 * i, H.cis_h[i]);
			put16(p + 3 + 4 * i, acl_handle(i));
		}
		cmd(0x2064, p, (uint8_t)(1 + 4 * g_cfg.n_buds));
		break;
	}
	case H_DATAPATH: {
		int dirs = g_cfg.uplink ? 2 : 1;
		if (H.dp_idx >= g_cfg.n_buds * dirs) {
			start_stream();
			break;
		}
		int cis = H.dp_idx / dirs, dir = H.dp_idx % dirs;
		memset(p, 0, 13);
		put16(p, H.cis_h[cis]);
		p[2] = (uint8_t)dir;
		p[3] = 0x00; /* HCI */
		p[4] = 0x03; /* transparent */
		cmd(0x206E, p, 13);
		break;
	}
	case H_DISC: {
		bool any_up = false;
		for (int k = 0; k < g_cfg.n_buds; k++)
			any_up |= H.cis_up[k];
		if (!H.stats_taken) {
			/* Engine counters before disconnecting (the CIS handles may disappear afterwards). */
			H.stats_taken = true;
			for (int k = 0; k < g_cfg.n_buds; k++)
				plat_engine_stats(H.cis_h[k]);
		}
		if (!any_up) {
			H.st = H_REMOVE;
			advance();
			break;
		}
		while (H.disc_idx < g_cfg.n_buds && !H.cis_up[H.disc_idx])
			H.disc_idx++;
		if (H.disc_idx >= g_cfg.n_buds || H.pending_op)
			break; /* wait for Disconnection Complete / Command Status */
		put16(p, H.cis_h[H.disc_idx]);
		p[2] = 0x13;
		cmd(0x0406, p, 3);
		break;
	}
	case H_REMOVE:
		p[0] = CIG_ID;
		cmd(0x2065, p, 1);
		break;
	default:
		break;
	}
}

static void on_cmd_done(uint16_t op, uint8_t status, const uint8_t *ret, uint8_t rlen)
{
	if (op != H.pending_op) {
		viol(V_HOST, "host", "completion for 0x%04x while waiting for 0x%04x", op, H.pending_op);
		return;
	}
	H.pending_op = 0;
	H.step_gen++;
	char m[96];
	switch (H.st) {
	case H_FEAT:
		if (rlen >= 9)
			H.feat_cis_central = (get64(ret + 1) >> 28) & 1;
		slog("host", "local features: CIS Central bit %s", H.feat_cis_central ? "set" : "NOT set");
		advance();
		break;
	case H_HOSTFEAT:
	case H_BUFSZ:
	case H_CIG:
	case H_DATAPATH:
	case H_REMOVE:
		if (status) {
			snprintf(m, sizeof(m), "HCI 0x%04x failed with status 0x%02x", op, status);
			fail(m);
			return;
		}
		if (H.st == H_BUFSZ) {
			if (rlen < 7) {
				fail("LE Read Buffer Size v2: short return");
				return;
			}
			H.credits = H.credits_total = ret[6];
			if (!H.credits || get16(ret + 4) < g_cfg.max_sdu_c2p + 4)
				viol(V_HOST, "host", "ISO buffers: %u x %u B too small", ret[6], get16(ret + 4));
			advance();
		} else if (H.st == H_CIG) {
			if (rlen < 3 + 2 * g_cfg.n_buds || ret[1] != CIG_ID || ret[2] != g_cfg.n_buds) {
				fail("LE Set CIG Parameters: bad return parameters");
				return;
			}
			for (int i = 0; i < g_cfg.n_buds; i++)
				H.cis_h[i] = get16(ret + 3 + 2 * i);
			H.st = H_CREATE;
			advance();
		} else if (H.st == H_DATAPATH) {
			H.dp_idx++;
			advance();
		} else if (H.st == H_REMOVE) {
			H.st = H_DONE;
			g_m.host_done = true;
			slog("host", "done");
			sim_at(sim_now() + MS(100), stop_cb, NULL, 0);
		} else {
			advance();
		}
		break;
	case H_ENC:
	case H_CREATE:
	case H_DISC:
		if (status) {
			snprintf(m, sizeof(m), "HCI 0x%04x Command Status 0x%02x", op, status);
			fail(m);
			return;
		}
		if (H.st == H_ENC)
			H.st = H_WAIT_ENC;
		else if (H.st == H_CREATE) {
			H.st = H_WAIT_EST;
			arm_timeout(SEC(10));
		} else {
			H.disc_idx++;
			arm_timeout(SEC(5));
		}
		break;
	default:
		break;
	}
}

static int cis_idx(uint16_t h)
{
	for (int i = 0; i < g_cfg.n_buds; i++)
		if (H.cis_h[i] == h && h)
			return i;
	return -1;
}

static void on_cis_established(const uint8_t *e, uint8_t len)
{
	if (len < 29) {
		viol(V_HOST, "host", "LE CIS Established too short (%u)", len);
		return;
	}
	uint8_t status = e[1];
	int i = cis_idx(get16(e + 2));
	if (i < 0) {
		viol(V_HOST, "host", "LE CIS Established for unknown handle 0x%04x", get16(e + 2));
		return;
	}
	if (g_m.cis_est_seen[i])
		viol(V_HOST, "host", "duplicate LE CIS Established for CIS %d", i);
	g_m.cis_est_seen[i] = true;
	g_m.cis_est_status[i] = status;
	H.est_n++;
	slog("host", "CIS %d established: status 0x%02x cig_sync %u cis_sync %u tl %u/%u nse %u bn %u/%u ft %u/%u "
			"pdu %u/%u iso %u", i, status, get24(e + 4), get24(e + 7), get24(e + 10), get24(e + 13), e[18],
			e[19], e[20], e[21], e[22], get16(e + 23), get16(e + 25), get16(e + 27));
	if (status == 0) {
		H.cis_up[i] = true;
		g_m.est_nse = e[18];
		g_m.est_bn_c = e[19];
		g_m.est_ft_c = e[21];
		if (get24(e + 10) > g_cfg.max_lat_c2p_ms * 1000u)
			warn("host", "Transport_Latency_C_To_P %u us above requested %u ms", get24(e + 10), g_cfg.max_lat_c2p_ms);
	}
	if (H.est_n == g_cfg.n_buds && H.st == H_WAIT_EST) {
		H.step_gen++;
		bool any = false;
		for (int k = 0; k < g_cfg.n_buds; k++)
			any |= H.cis_up[k];
		if (any) {
			H.st = H_DATAPATH;
			H.dp_idx = 0;
		} else {
			H.st = H_REMOVE;
		}
		advance();
	}
}

static void on_uplink(const uint8_t *h4, uint16_t len)
{
	uint16_t hf = get16(h4 + 1);
	uint16_t dlen = get16(h4 + 3) & 0x3FFF;
	if (len != 5u + dlen) {
		viol(V_HOST, "host", "ISO packet length field %u vs %u", dlen, len);
		return;
	}
	int i = cis_idx(hf & 0x0FFF);
	unsigned pb = (hf >> 12) & 3, ts = (hf >> 14) & 1;
	if (i < 0) {
		viol(V_HOST, "host", "uplink ISO for unknown handle 0x%03x", hf & 0x0FFF);
		return;
	}
	if (pb != 2) {
		viol(V_HOST, "host", "uplink PB flag %u (fragmented SDU not expected for %u B)", pb, g_cfg.max_sdu_p2c);
		return;
	}
	const uint8_t *d = h4 + 5 + (ts ? 4 : 0);
	if (dlen < 4u + (ts ? 4 : 0)) {
		viol(V_HOST, "host", "uplink ISO too short");
		return;
	}
	uint16_t psn = get16(d);
	uint16_t slen = get16(d + 2) & 0x0FFF;
	unsigned psf = d[3] >> 6;
	if (H.up_psn_have[i] && psn != (uint16_t)(H.up_psn[i] + 1))
		warn("host", "uplink CIS %d: Packet_Sequence_Number %u after %u", i, psn, H.up_psn[i]);
	H.up_psn_have[i] = true;
	H.up_psn[i] = psn;
	if (psf == 2) {
		g_m.up_lost_flag++;
		return;
	}
	if (slen != dlen - 4u - (ts ? 4 : 0)) {
		viol(V_HOST, "host", "uplink ISO_SDU_Length %u vs payload %u", slen, dlen - 4u);
		return;
	}
	uint32_t seq;
	sim_ns_t gen;
	if (slen != g_cfg.max_sdu_p2c || !sdu_check(d + 4, slen, (uint8_t)(0xA5 + i), &seq, &gen)) {
		g_m.up_corrupt++;
		viol(V_DESYNC, "host", "uplink SDU on CIS %d corrupt (len %u, psf %u)", i, slen, psf);
		return;
	}
	g_m.up_rx++;
	if (H.up_have[i]) {
		if ((int32_t)(seq - H.up_seq[i]) <= 0) {
			g_m.up_dup++;
			viol(V_DESYNC, "host", "uplink CIS %d seq %u after %u", i, seq, H.up_seq[i]);
		} else {
			g_m.up_gap += seq - H.up_seq[i] - 1;
		}
	}
	H.up_have[i] = true;
	H.up_seq[i] = seq;
	stat_add(&g_m.up_lat_ns, sim_now() - gen);
}

void host_rx_h4(const uint8_t *h4, uint16_t len)
{
	if (h4[0] == 0x05) {
		on_uplink(h4, len);
		return;
	}
	if (h4[0] != 0x04 || len < 3)
		return;
	const uint8_t *pr = h4 + 3;
	uint8_t plen = h4[2];
	switch (h4[1]) {
	case 0x0E:
		if (plen >= 4)
			on_cmd_done(get16(pr + 1), pr[3], pr + 3, (uint8_t)(plen - 3));
		break;
	case 0x0F:
		if (plen >= 4)
			on_cmd_done(get16(pr + 2), pr[0], pr, 1);
		break;
	case 0x13: {
		uint8_t n = pr[0];
		if (plen != 1 + 4 * n) {
			viol(V_HOST, "host", "Number Of Completed Packets length %u for %u handles", plen, n);
			break;
		}
		for (int k = 0; k < n; k++) {
			uint16_t h = get16(pr + 1 + 4 * k), c = get16(pr + 3 + 4 * k);
			if (cis_idx(h) < 0) {
				viol(V_HOST, "host", "NOCP for non-CIS handle 0x%04x", h);
				continue;
			}
			H.credits += c;
			g_m.host_nocp += c;
		}
		if (H.credits > H.credits_total)
			viol(V_HOST, "host", "NOCP returned more credits (%d) than buffers (%d)", H.credits, H.credits_total);
		try_send_pending();
		break;
	}
	case 0x05: {
		uint16_t h = get16(pr + 1);
		int i = cis_idx(h);
		slog("host", "Disconnection Complete handle 0x%04x status 0x%02x reason 0x%02x", h, pr[0], pr[3]);
		if (i >= 0) {
			if (!H.cis_up[i])
				viol(V_HOST, "host", "Disconnection Complete for CIS %d that is not up", i);
			H.cis_up[i] = false;
			g_m.cis_disc_at[i] = sim_now();
			H.disc_n++;
			if (H.st == H_DISC) {
				advance();
			} else if (H.st == H_STREAM) {
				bool any = false;
				for (int k = 0; k < g_cfg.n_buds; k++)
					any |= H.cis_up[k];
				if (!any) {
					H.st = H_REMOVE;
					advance();
				}
			}
		}
		break;
	}
	case 0x08:
		if (pr[0] || !pr[3]) {
			fail("Encryption Change with error");
			break;
		}
		if (H.st == H_WAIT_ENC && get16(pr + 1) == acl_handle(H.enc_idx)) {
			H.enc_idx++;
			advance();
		}
		break;
	case 0x3E:
		if (pr[0] == 0x01) {
			H.conn_seen++;
			if (H.st == H_WAIT_CONN)
				advance();
		} else if (pr[0] == 0x19) {
			on_cis_established(pr, plen);
		}
		break;
	default:
		break;
	}
}

void host_init(void)
{
	memset(&H, 0, sizeof(H));
	rng_seed(&H.rng, g_cfg.seed * 23 + 9);
	H.hclk.ppb = (int64_t)g_cfg.ppm_host * 1000;
	H.st = H_WAIT_CONN;
	arm_timeout(SEC(5));
}

void host_finish(void)
{
	if (!g_m.host_done && !g_m.host_failed) {
		char m[96];
		snprintf(m, sizeof(m), "host did not finish (step %d, pending 0x%04x)", H.st, H.pending_op);
		g_m.host_failed = true;
		snprintf(g_m.host_fail_msg, sizeof(g_m.host_fail_msg), "%s", m);
	}
}
