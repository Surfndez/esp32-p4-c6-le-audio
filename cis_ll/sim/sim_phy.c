/*
 * Simulator physical layer: CSA#2 (Vol 6 Part B 4.5.8.3, with subevents), access address rules
 * (2.1.2), packet air time for 1M/2M, the air model (losses, CRC, Wi-Fi coexistence windows).
 */
#include "sim.h"

#include <math.h>
#include <string.h>

/* ---------------- CSA#2 ---------------- */
static uint8_t rev8(uint8_t b)
{
	b = (uint8_t)((b & 0xF0) >> 4 | (b & 0x0F) << 4);
	b = (uint8_t)((b & 0xCC) >> 2 | (b & 0x33) << 2);
	b = (uint8_t)((b & 0xAA) >> 1 | (b & 0x55) << 1);
	return b;
}

static uint16_t perm(uint16_t v)
{
	return (uint16_t)(rev8((uint8_t)(v >> 8)) << 8 | rev8((uint8_t)v));
}

static uint16_t mam(uint16_t a, uint16_t b)
{
	return (uint16_t)(17u * a + b);
}

void csa2_init(csa2_t *c, uint32_t aa, const uint8_t chan_map[5])
{
	memset(c, 0, sizeof(*c));
	c->chan_id = (uint16_t)((aa >> 16) ^ (aa & 0xFFFF));
	for (int ch = 0; ch < 37; ch++)
		if (chan_map[ch / 8] & (1u << (ch % 8)))
			c->used[c->n_used++] = (uint8_t)ch;
}

uint8_t csa2_event(csa2_t *c, uint16_t counter)
{
	uint16_t u = counter ^ c->chan_id;
	for (int i = 0; i < 3; i++)
		u = mam(perm(u), c->chan_id);
	c->prn_lu = u; /* prn_s */
	uint16_t prn_e = u ^ c->chan_id;
	uint8_t unmapped = (uint8_t)(prn_e % 37);
	for (uint8_t i = 0; i < c->n_used; i++) {
		if (c->used[i] == unmapped) {
			c->remap_idx = i;
			return unmapped;
		}
	}
	c->remap_idx = (uint8_t)(((uint32_t)c->n_used * prn_e) >> 16);
	return c->used[c->remap_idx];
}

uint8_t csa2_subevent(csa2_t *c)
{
	c->prn_lu = mam(perm(c->prn_lu), c->chan_id);
	uint16_t prn_se = c->prn_lu ^ c->chan_id;
	int n = c->n_used;
	int a = n - 5 < 3 ? n - 5 : 3;
	int b = (n - 10) / 2 < 11 ? (n - 10) / 2 : 11;
	int d = a > b ? a : b;
	if (d < 1)
		d = 1;
	uint32_t x = ((uint32_t)(n - 2 * d + 1) * prn_se) >> 16;
	c->remap_idx = (uint8_t)((c->remap_idx + d + x) % (uint32_t)n);
	return c->used[c->remap_idx];
}

uint8_t csa2_chan(uint32_t aa, const uint8_t chan_map[5], uint16_t counter, uint8_t subevent)
{
	csa2_t c;
	csa2_init(&c, aa, chan_map);
	uint8_t ch = csa2_event(&c, counter);
	for (uint8_t i = 0; i < subevent; i++)
		ch = csa2_subevent(&c);
	return ch;
}

/* ---------------- access address ---------------- */
const char *aa_check(uint32_t aa)
{
	const uint32_t adv = 0x8E89BED6u;
	if (aa == adv)
		return "equals advertising AA";
	if (__builtin_popcount(aa ^ adv) == 1)
		return "differs from advertising AA by one bit";
	uint8_t o0 = (uint8_t)aa, o1 = (uint8_t)(aa >> 8), o2 = (uint8_t)(aa >> 16), o3 = (uint8_t)(aa >> 24);
	if (o0 == o1 && o1 == o2 && o2 == o3)
		return "all four octets equal";
	int run = 1, transitions = 0;
	for (int i = 1; i < 32; i++) {
		bool same = ((aa >> i) & 1) == ((aa >> (i - 1)) & 1);
		if (same) {
			if (++run > 6)
				return "more than six consecutive equal bits";
		} else {
			run = 1;
			transitions++;
		}
	}
	if (transitions > 24)
		return "more than 24 transitions";
	int msb_tr = 0;
	for (int i = 27; i < 32; i++)
		if (((aa >> i) & 1) != ((aa >> (i - 1)) & 1))
			msb_tr++;
	if (msb_tr < 2)
		return "fewer than two transitions in the six most significant bits";
	return NULL;
}

/* Preamble + AA + header + payload (with MIC) + CRC; 1M: 8 us/octet, 2M: 4 us/octet and a 2-octet preamble. */
uint32_t air_time_us(uint8_t phy, unsigned pdu_payload_len)
{
	if (phy == CIS_PHY_2M)
		return (2 + 4 + 2 + pdu_payload_len + 3) * 4;
	return (1 + 4 + 2 + pdu_payload_len + 3) * 8;
}

/* ---------------- air ---------------- */
#define COEX_HIST 64
static rng_t    s_rng_loss, s_rng_coex;
static sim_ns_t s_coex_s[COEX_HIST], s_coex_e[COEX_HIST];
static unsigned s_coex_n;     /* how many windows were generated */
static sim_ns_t s_coex_until; /* end of the generated timeline */

void air_init(void)
{
	rng_seed(&s_rng_loss, g_cfg.seed * 7 + 1);
	rng_seed(&s_rng_coex, g_cfg.seed * 11 + 3);
	s_coex_n = 0;
	s_coex_until = 0;
}

air_res_t air_packet(void)
{
	if (!rng_chance(&s_rng_loss, g_cfg.loss))
		return AIR_OK;
	return rng_chance(&s_rng_loss, g_cfg.crc_frac) ? AIR_CRC : AIR_MISS;
}

bool air_mic_corrupt(void)
{
	return rng_chance(&s_rng_loss, g_cfg.mic_corrupt);
}

static void coex_extend(sim_ns_t upto)
{
	double duty = g_cfg.coex_duty;
	while (s_coex_until <= upto) {
		sim_ns_t busy = rng_range(&s_rng_coex, g_cfg.coex_min, g_cfg.coex_max);
		/* The mean gap such that the busy share = duty (exponential gaps). */
		double mean_busy = (double)(g_cfg.coex_min + g_cfg.coex_max) / 2.0;
		double mean_idle = mean_busy * (1.0 - duty) / duty;
		double u = rng_unit(&s_rng_coex);
		sim_ns_t idle = (sim_ns_t)(-mean_idle * log(1.0 - u));
		sim_ns_t s = s_coex_until + idle;
		unsigned k = s_coex_n++ % COEX_HIST;
		s_coex_s[k] = s;
		s_coex_e[k] = s + busy;
		s_coex_until = s + busy;
	}
}

bool air_coex_blocked(sim_ns_t s, sim_ns_t e)
{
	if (g_cfg.coex_duty <= 0)
		return false;
	coex_extend(e);
	unsigned n = s_coex_n < COEX_HIST ? s_coex_n : COEX_HIST;
	for (unsigned i = 0; i < n; i++) {
		unsigned k = (s_coex_n - 1 - i) % COEX_HIST;
		if (s_coex_s[k] < e && s < s_coex_e[k])
			return true;
		if (s_coex_e[k] < s - MS(100))
			break;
	}
	return false;
}
