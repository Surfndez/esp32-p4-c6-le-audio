/*
 * Simulator scenarios (command-line presets), the run and the verdict per gate G2
 * (docs/c6-link-layer.md). The log of each scenario: <logdir>/<name>.log, only the
 * summary on stdout.
 */
#include "sim.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

static void defaults(sim_cfg_t *c)
{
	memset(c, 0, sizeof(*c));
	c->seed = 1;
	c->stream_time = SEC(110); /* >= 10 000 CIS events at 10 ms */
	c->crc_frac = 0.6;
	c->coex_min = MS(1);
	c->coex_max = MS(6);
	c->host_follows_c = true;
	c->sca_c_ppm = 50;
	c->sca_p_ppm = 50;
	c->ppm_c = 20;
	c->ppm_p[0] = -15;
	c->ppm_p[1] = 10;
	c->c_clock_start_us = 0xFFFFFFFFu - 40000000u; /* the C6 clock wraps after 40 s */
	c->max_nse = 1;
	c->owns_seq = false;
	c->tx_lead_us = 300;
	c->subevent_hop = true;
	c->task_lat_us = 100;
	c->n_buds = 1;
	c->acl_interval_us = 30000;
	c->acl2_offset_us = 15000;
	c->sup_timeout_ms = 2000;
	c->acl_phy = CIS_PHY_1M;
	c->acl_max_octets = 251;
	c->acl_dle_at = MS(400);
	c->acl_reserve_octets = 27;
	c->encrypt = true;
	c->sdu_interval_us = 10000;
	c->max_sdu_c2p = 120;
	c->max_sdu_p2c = 40;
	c->phy = CIS_PHY_2M;
	c->rtn_c2p = 2;
	c->rtn_p2c = 2;
	c->max_lat_c2p_ms = 20;
	c->max_lat_p2c_ms = 20;
	c->up_lat_min = US(80);
	c->up_lat_max = US(400);
	c->reject_reason = 0x0D;
	c->uplink = true;
	c->gate_max_loss_pct = 0.0;
	c->gate_min_events = 10000;
	c->gate_latency = true;
	c->gate_underrun = true;
}

typedef struct {
	const char *name;
	const char *desc;
	void (*apply)(sim_cfg_t *c);
} preset_t;

static void p_ideal(sim_cfg_t *c) { (void)c; }
static void p_loss1(sim_cfg_t *c) { c->loss = 0.01; c->gate_max_loss_pct = 1.0; }
static void p_loss5(sim_cfg_t *c) { c->loss = 0.05; c->gate_max_loss_pct = -1; }
static void p_drift(sim_cfg_t *c)
{
	c->ppm_c = 50;
	c->ppm_p[0] = -50;
	c->stream_time = SEC(1800);
	c->gate_min_events = 180000;
}
static void p_coex(sim_cfg_t *c) { c->coex_duty = 0.10; c->gate_max_loss_pct = -1; c->gate_underrun = false; }
static void p_jitter(sim_cfg_t *c) { c->host_jitter_max = MS(3); c->stall_prob = 0.005; c->stall_len = MS(8); }
static void p_twobuds(sim_cfg_t *c) { c->n_buds = 2; c->loss = 0.01; c->gate_max_loss_pct = 1.0; }
static void p_nse1(sim_cfg_t *c) { c->loss = 0.05; c->max_nse = 1; c->gate_max_loss_pct = -1; }
static void p_nse3(sim_cfg_t *c) { c->loss = 0.05; c->max_nse = 3; c->gate_max_loss_pct = 1.0; }
/* C6 model (the C6 controller adapter): PDUs loaded about one ISO interval before the anchor (tx_lead 10 ms); at NSE 2
 * the second subevent is a separate pseudo-connection with its PDU prepared together with the first. */
static void p_c6nse1(sim_cfg_t *c) { c->loss = 0.05; c->max_nse = 1; c->tx_lead_us = 10000; c->gate_max_loss_pct = -1; }
static void p_c6nse2(sim_cfg_t *c)
{
	c->loss = 0.05;
	c->max_nse = 2;
	c->prep_ahead = true;
	c->tx_lead_us = 10000;
	c->gate_max_loss_pct = 1.0;
}
static void p_reject(sim_cfg_t *c) { c->reject = true; c->expect_reject = true; c->stream_time = SEC(2); }
static void p_aclloss(sim_cfg_t *c)
{
	c->acl_loss_at = SEC(20);
	c->expect_acl_loss = true;
	c->stream_time = SEC(40);
	c->gate_min_events = 0;
	c->gate_max_loss_pct = -1;
	c->gate_underrun = false;
}
static void p_hwseq(sim_cfg_t *c) { c->owns_seq = true; c->loss = 0.01; c->gate_max_loss_pct = 1.0; }
static void p_noenc(sim_cfg_t *c) { c->encrypt = false; c->stream_time = SEC(20); c->gate_min_events = 1500; }
/* Negative tests: the simulator must detect an injected error. */
static void n_mic(sim_cfg_t *c) { c->mic_corrupt = 0.01; c->stream_time = SEC(10); c->expect_violations = true; }
static void n_skew(sim_cfg_t *c) { c->tx_skew_ns = 30000; c->stream_time = SEC(5); c->expect_violations = true; }
static void n_nohop(sim_cfg_t *c)
{
	c->max_nse = 3;
	c->loss = 0.1;
	c->subevent_hop = false;
	c->stream_time = SEC(5);
	c->expect_violations = true;
}
static void n_hwseq(sim_cfg_t *c) { c->owns_seq = true; c->loss = 0.05; c->stream_time = SEC(10); c->expect_violations = true; }

static const preset_t k_presets[] = {
	{ "ideal", "no loss, small drift, clock wrap", p_ideal },
	{ "loss1", "1% air loss (G2: SDU loss <= 1%)", p_loss1 },
	{ "loss5", "5% air loss (report)", p_loss5 },
	{ "drift", "+50/-50 ppm, 30 min", p_drift },
	{ "coex10", "Wi-Fi coexistence 10% duty, 1-6 ms windows", p_coex },
	{ "hostjitter", "SDIO jitter 0-3 ms + 8 ms stalls", p_jitter },
	{ "twobuds", "2 ACL + 2 CIS in one CIG, 1% loss", p_twobuds },
	{ "nse1", "radio max_nse 1, 5% loss (report)", p_nse1 },
	{ "nse3", "radio max_nse 3, 5% loss (loss <= 1%)", p_nse3 },
	{ "c6nse1", "C6 model: NSE 1, PDU 10 ms ahead, 5% loss (report)", p_c6nse1 },
	{ "c6nse2", "C6 model: NSE 2, all subevents prepared ahead, 5% loss (loss <= 1%)", p_c6nse2 },
	{ "reject", "Peripheral answers LL_REJECT_EXT_IND", p_reject },
	{ "aclloss", "earbud vanishes at 20 s (ACL supervision)", p_aclloss },
	{ "hwseq", "radio_owns_seq (ACL sequencer semantics), 1% loss", p_hwseq },
	{ "noenc", "unencrypted ACL and CIS", p_noenc },
	{ "neg_mic", "NEGATIVE: injected payload corruption -> MIC failures detected", n_mic },
	{ "neg_skew", "NEGATIVE: radio transmits 30 us late -> timing violations detected", n_skew },
	{ "neg_nohop", "NEGATIVE: no subevent hopping with NSE 3 -> channel violations detected", n_nohop },
	{ "neg_hwseq", "NEGATIVE (expected for ACL sequencer): SN/NESN desync detected", n_hwseq },
};
#define N_PRESETS (sizeof(k_presets) / sizeof(k_presets[0]))

static double ms(double ns)
{
	return ns / 1e6;
}

static int verdict(const sim_cfg_t *c, FILE *out)
{
	const sim_metrics_t *m = &g_m;
	char why[512] = "";
	size_t wl = 0;
#define FAILIF(cond, ...) \
	do { \
		if (cond) \
			wl += (size_t)snprintf(why + wl, wl < sizeof(why) ? sizeof(why) - wl : 0, __VA_ARGS__); \
	} while (0)

	uint32_t vtot = 0;
	for (int k = 0; k < V_COUNT; k++)
		vtot += m->viol[k];
	double loss_pct = m->host_sdu_gen ? 100.0 * (double)(m->host_sdu_gen - (m->sdu_rx < m->host_sdu_gen ?
			m->sdu_rx : m->host_sdu_gen)) / (double)m->host_sdu_gen : 0;
	uint64_t ev_per_bud = m->cis_events / (c->n_buds ? c->n_buds : 1);
	bool pass;
	if (m->engine_init_failed) {
		FAILIF(true, " cis_engine_init-returned-NULL");
		pass = false;
	} else if (c->expect_violations) {
		FAILIF(vtot == 0, " no-violation-detected");
		pass = vtot > 0;
	} else if (c->expect_reject) {
		FAILIF(!m->cis_est_seen[0], " no-CIS-Established");
		FAILIF(m->cis_est_seen[0] && m->cis_est_status[0] == 0, " CIS-Established-status-0");
		FAILIF(m->cis_est_seen[0] && m->cis_est_status[0] != 0 && m->cis_est_status[0] != c->reject_reason,
				" status-0x%02x!=reject-reason", m->cis_est_status[0]);
		FAILIF(m->link_opened, " link-opened");
		FAILIF(!m->host_done, " host:%s", m->host_failed ? m->host_fail_msg : "not-done");
		FAILIF(vtot, " violations=%u", vtot);
		pass = wl == 0;
	} else {
		for (int i = 0; i < c->n_buds; i++)
			FAILIF(!m->cis_est_seen[i] || m->cis_est_status[i], " handshake[%d]", i);
		FAILIF(!m->host_done, " host:%s", m->host_failed ? m->host_fail_msg : "not-done");
		FAILIF(ev_per_bud < c->gate_min_events, " events=%llu<%u", (unsigned long long)ev_per_bud,
				c->gate_min_events);
		FAILIF(c->gate_max_loss_pct >= 0 && loss_pct > c->gate_max_loss_pct + 1e-9, " loss=%.3f%%>%.1f%%",
				loss_pct, c->gate_max_loss_pct);
		for (int k = 0; k < V_COUNT; k++)
			FAILIF(m->viol[k], " %s=%u", g_vname[k], m->viol[k]);
		FAILIF(m->overlap_acl, " cis-acl-overlap=%llu", (unsigned long long)m->overlap_acl);
		FAILIF(m->overlap_cis, " cis-cis-overlap=%llu", (unsigned long long)m->overlap_cis);
		FAILIF(c->gate_latency && m->sdu_late, " late=%llu", (unsigned long long)m->sdu_late);
		FAILIF(c->gate_underrun && m->underruns, " underruns=%llu", (unsigned long long)m->underruns);
		FAILIF(m->sdu_dup, " dup-sdu=%llu", (unsigned long long)m->sdu_dup);
		FAILIF(!m->link_closed_all, " link-not-closed");
		FAILIF(m->eng_stats_ok && m->eng_rx_mic_fail, " engine-mic-fail=%u", m->eng_rx_mic_fail);
		if (c->expect_acl_loss) {
			sim_ns_t lim = c->acl_loss_at + MS(c->sup_timeout_ms) + SEC(1);
			FAILIF(!m->cis_disc_at[0] || m->cis_disc_at[0] > lim, " no-CIS-disconnect-after-ACL-loss");
		}
		pass = wl == 0;
	}
	double lat_avg = m->lat_ns.n ? (double)m->lat_ns.sum / (double)m->lat_ns.n : 0;
	double lat_sd = m->lat_ns.n ? sqrt(fmax(0, m->lat_ns.sumsq / (double)m->lat_ns.n - lat_avg * lat_avg)) : 0;
	fprintf(out, "[%s] %-10s ev %llu miss %llu skip %llu | SDU gen %llu rx %llu loss %.3f%% late %llu corrupt %llu | "
			"lat %.3f/%.3f/%.3f ms sd %.3f | underrun %llu flushC %llu | up rx %llu gap %llu lostflag %llu | "
			"nse %u ft %u | ovl %llu/%llu warn %u\n",
			pass ? "PASS" : "FAIL", c->name, (unsigned long long)ev_per_bud, (unsigned long long)m->events_missed,
			(unsigned long long)m->radio_skipped, (unsigned long long)m->host_sdu_gen,
			(unsigned long long)m->sdu_rx, loss_pct, (unsigned long long)m->sdu_late,
			(unsigned long long)m->sdu_corrupt, ms((double)m->lat_ns.min), ms(lat_avg), ms((double)m->lat_ns.max),
			ms(lat_sd), (unsigned long long)m->underruns, (unsigned long long)m->c2p_flushed,
			(unsigned long long)m->up_rx, (unsigned long long)m->up_gap, (unsigned long long)m->up_lost_flag,
			m->est_nse, m->est_ft_c, (unsigned long long)m->overlap_acl, (unsigned long long)m->overlap_cis,
			m->warn);
	fprintf(out, "       violations:");
	for (int k = 0; k < V_COUNT; k++)
		fprintf(out, " %s=%u", g_vname[k], m->viol[k]);
	fprintf(out, "\n");
	if (m->eng_stats_ok)
		fprintf(out, "       engine stats: events %u skipped %u underrun %u flushed %u sdu_dropped %u sdu_late %u "
				"rx_ok %u mic_fail %u rx_lost %u | too_long %u\n", m->eng_events, m->eng_skipped,
				m->eng_tx_underrun, m->eng_tx_flushed, m->eng_sdu_dropped, m->eng_sdu_late, m->eng_rx_ok,
				m->eng_rx_mic_fail, m->eng_rx_lost, m->eng_too_long);
	if (!pass)
		fprintf(out, "       FAIL:%s\n", why);
	return pass ? 0 : 1;
#undef FAILIF
}

static int run_one(const preset_t *pr, const char *logdir, const sim_cfg_t *ovr, uint32_t ovr_mask)
{
	defaults(&g_cfg);
	g_cfg.name = pr->name;
	pr->apply(&g_cfg);
	if (ovr_mask & 1)
		g_cfg.seed = ovr->seed;
	if (ovr_mask & 2) {
		g_cfg.stream_time = ovr->stream_time;
		g_cfg.gate_min_events = 0;
	}
	if (ovr_mask & 4)
		g_cfg.loss = ovr->loss;
	if (ovr_mask & 8)
		g_cfg.max_nse = ovr->max_nse;
	if (ovr_mask & 16)
		g_cfg.owns_seq = ovr->owns_seq;
	if (ovr_mask & 32)
		g_cfg.coex_duty = ovr->coex_duty;
	memset(&g_m, 0, sizeof(g_m));
	char path[512];
	snprintf(path, sizeof(path), "%s/%s.log", logdir, pr->name);
	g_log = fopen(path, "w");
	slog("sim", "scenario %s (%s), engine: %s, seed %llu", pr->name, pr->desc, plat_engine_name(),
			(unsigned long long)g_cfg.seed);
	sim_core_init();
	air_init();
	if (plat_init()) {
		acl_init();
		host_init();
		sim_run_until(g_cfg.stream_time + SEC(30));
	}
	host_finish();
	acl_finish();
	plat_finish();
	slog("sim", "end at %.3f s, %llu sim events", (double)sim_now() / 1e9,
			(unsigned long long)sim_events_processed());
	int rc = verdict(&g_cfg, stdout);
	if (g_log) {
		verdict(&g_cfg, g_log);
		fclose(g_log);
		g_log = NULL;
	}
	sim_core_free();
	return rc;
}

static void usage(void)
{
	printf("usage: sim [--selftest] [--all | --scenario NAME ...] [--logdir DIR] [--seed N] [--time S]\n"
			"           [--loss P] [--max-nse N] [--owns-seq 0|1] [--coex DUTY]\n"
			"scenarios:\n");
	for (unsigned i = 0; i < N_PRESETS; i++)
		printf("  %-10s %s\n", k_presets[i].name, k_presets[i].desc);
}

int main(int argc, char **argv)
{
	const char *logdir = ".";
	const char *names[64];
	int n_names = 0, rc = 0;
	bool all = false, selftest = false;
	sim_cfg_t ovr;
	uint32_t mask = 0;
	memset(&ovr, 0, sizeof(ovr));
	for (int i = 1; i < argc; i++) {
		const char *a = argv[i];
		const char *v = i + 1 < argc ? argv[i + 1] : NULL;
		if (!strcmp(a, "--selftest"))
			selftest = true;
		else if (!strcmp(a, "--all"))
			all = true;
		else if (!strcmp(a, "--scenario") && v && n_names < 64)
			names[n_names++] = argv[++i];
		else if (!strcmp(a, "--logdir") && v)
			logdir = argv[++i];
		else if (!strcmp(a, "--seed") && v)
			ovr.seed = strtoull(argv[++i], NULL, 0), mask |= 1;
		else if (!strcmp(a, "--time") && v)
			ovr.stream_time = (sim_ns_t)(atof(argv[++i]) * 1e9), mask |= 2;
		else if (!strcmp(a, "--loss") && v)
			ovr.loss = atof(argv[++i]), mask |= 4;
		else if (!strcmp(a, "--max-nse") && v)
			ovr.max_nse = (uint8_t)atoi(argv[++i]), mask |= 8;
		else if (!strcmp(a, "--owns-seq") && v)
			ovr.owns_seq = atoi(argv[++i]) != 0, mask |= 16;
		else if (!strcmp(a, "--coex") && v)
			ovr.coex_duty = atof(argv[++i]), mask |= 32;
		else {
			usage();
			return 2;
		}
	}
	if (!selftest && !all && !n_names) {
		usage();
		return 2;
	}
	if (selftest) {
		int f = selftest_units();
		printf("unit tests: %s (%d failed)\n", f ? "FAIL" : "PASS", f);
		rc |= f != 0;
	}
	printf("engine: %s\n", plat_engine_name());
	for (unsigned i = 0; i < N_PRESETS; i++) {
		/* --all = the G2 scenarios; the negative harness tests only explicitly (--scenario neg_*). */
		bool want = all && strncmp(k_presets[i].name, "neg_", 4) != 0;
		for (int k = 0; k < n_names; k++)
			want |= !strcmp(names[k], k_presets[i].name);
		if (want)
			rc |= run_one(&k_presets[i], logdir, &ovr, mask);
	}
	for (int k = 0; k < n_names; k++) {
		bool found = false;
		for (unsigned i = 0; i < N_PRESETS; i++)
			found |= !strcmp(names[k], k_presets[i].name);
		if (!found) {
			printf("unknown scenario %s\n", names[k]);
			rc = 2;
		}
	}
	return rc;
}
