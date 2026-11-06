/*
 * Simulator core: the event heap (ns time, FIFO for equal times), clocks with drift, RNG,
 * log, violation counters.
 */
#include "sim.h"

#include <stdarg.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
	sim_ns_t t;
	uint64_t seq;
	sim_fn_t fn;
	void    *ctx;
	uint64_t arg;
} ev_t;

static ev_t    *s_heap;
static size_t   s_n, s_cap;
static uint64_t s_seq;
static sim_ns_t s_now;
static bool     s_stop;
static uint64_t s_processed;

FILE *g_log;
sim_cfg_t g_cfg;
sim_metrics_t g_m;

const char *const g_vname[V_COUNT] = {
	"format", "timing", "channel", "mic", "desync", "llcp", "param", "contract", "host",
};

static bool ev_less(const ev_t *a, const ev_t *b)
{
	return a->t < b->t || (a->t == b->t && a->seq < b->seq);
}

void sim_core_init(void)
{
	free(s_heap);
	s_heap = NULL;
	s_n = s_cap = 0;
	s_seq = 0;
	s_now = 0;
	s_stop = false;
	s_processed = 0;
}

void sim_core_free(void)
{
	free(s_heap);
	s_heap = NULL;
	s_n = s_cap = 0;
}

void sim_at(sim_ns_t t, sim_fn_t fn, void *ctx, uint64_t arg)
{
	if (t < s_now) {
		/* The past does not exist: the event runs now (and that is a caller bug). */
		slog("core", "BUG: event scheduled in the past (%lld < %lld)", (long long)t, (long long)s_now);
		t = s_now;
	}
	if (s_n == s_cap) {
		s_cap = s_cap ? s_cap * 2 : 1024;
		s_heap = realloc(s_heap, s_cap * sizeof(*s_heap));
		if (!s_heap) {
			fprintf(stderr, "out of memory\n");
			exit(2);
		}
	}
	size_t i = s_n++;
	ev_t e = { t, s_seq++, fn, ctx, arg };
	while (i > 0) {
		size_t p = (i - 1) / 2;
		if (!ev_less(&e, &s_heap[p]))
			break;
		s_heap[i] = s_heap[p];
		i = p;
	}
	s_heap[i] = e;
}

static ev_t pop(void)
{
	ev_t top = s_heap[0];
	ev_t last = s_heap[--s_n];
	size_t i = 0;
	for (;;) {
		size_t l = 2 * i + 1, r = l + 1, m = i;
		const ev_t *mv = &last;
		if (l < s_n && ev_less(&s_heap[l], mv)) {
			m = l;
			mv = &s_heap[l];
		}
		if (r < s_n && ev_less(&s_heap[r], mv))
			m = r;
		if (m == i)
			break;
		s_heap[i] = s_heap[m];
		i = m;
	}
	if (s_n)
		s_heap[i] = last;
	return top;
}

sim_ns_t sim_now(void)
{
	return s_now;
}

void sim_stop(void)
{
	s_stop = true;
}

uint64_t sim_events_processed(void)
{
	return s_processed;
}

void sim_run_until(sim_ns_t end)
{
	while (!s_stop && s_n && s_heap[0].t <= end) {
		ev_t e = pop();
		s_now = e.t;
		s_processed++;
		e.fn(e.ctx, e.arg);
	}
	if (!s_stop && s_now < end)
		s_now = end;
}

/* ---- RNG: xoshiro256** seeded with splitmix64 ---- */
static uint64_t splitmix(uint64_t *x)
{
	uint64_t z = (*x += 0x9E3779B97F4A7C15ULL);
	z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
	z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
	return z ^ (z >> 31);
}

void rng_seed(rng_t *r, uint64_t seed)
{
	for (int i = 0; i < 4; i++)
		r->s[i] = splitmix(&seed);
}

static uint64_t rotl(uint64_t x, int k)
{
	return (x << k) | (x >> (64 - k));
}

uint64_t rng_u64(rng_t *r)
{
	uint64_t *s = r->s;
	uint64_t res = rotl(s[1] * 5, 7) * 9;
	uint64_t t = s[1] << 17;
	s[2] ^= s[0];
	s[3] ^= s[1];
	s[1] ^= s[2];
	s[0] ^= s[3];
	s[2] ^= t;
	s[3] = rotl(s[3], 45);
	return res;
}

double rng_unit(rng_t *r)
{
	return (double)(rng_u64(r) >> 11) * (1.0 / 9007199254740992.0);
}

int64_t rng_range(rng_t *r, int64_t lo, int64_t hi)
{
	if (hi <= lo)
		return lo;
	return lo + (int64_t)(rng_u64(r) % (uint64_t)(hi - lo + 1));
}

bool rng_chance(rng_t *r, double p)
{
	return p > 0 && rng_unit(r) < p;
}

void rng_bytes(rng_t *r, uint8_t *buf, unsigned len)
{
	for (unsigned i = 0; i < len; i++)
		buf[i] = (uint8_t)rng_u64(r);
}

/* ---- clocks: local = offset + t * (1 + ppb/1e9) ---- */
int64_t clk_local(const sim_clock_t *c, sim_ns_t t)
{
	return c->offset_ns + t + (t * c->ppb) / 1000000000LL;
}

sim_ns_t clk_true(const sim_clock_t *c, int64_t local)
{
	int64_t d = local - c->offset_ns;
	sim_ns_t t = d - (d * c->ppb) / 1000000000LL;
	/* Two Newton corrections suffice (second-order error < 1 ns). */
	for (int i = 0; i < 3; i++) {
		int64_t err = clk_local(c, t) - local;
		if (err == 0)
			break;
		t -= err;
	}
	/* Rounding up: the event is no earlier than the local time. */
	while (clk_local(c, t) < local)
		t++;
	return t;
}

/* ---- log and violations ---- */
void slog(const char *who, const char *fmt, ...)
{
	if (!g_log)
		return;
	va_list ap;
	va_start(ap, fmt);
	fprintf(g_log, "%12.6f %-6s ", (double)s_now / 1e9, who);
	vfprintf(g_log, fmt, ap);
	fputc('\n', g_log);
	va_end(ap);
}

void viol(int kind, const char *who, const char *fmt, ...)
{
	g_m.viol[kind]++;
	if (!g_log || g_m.viol[kind] > 30)
		return;
	va_list ap;
	va_start(ap, fmt);
	fprintf(g_log, "%12.6f %-6s VIOLATION[%s] ", (double)s_now / 1e9, who, g_vname[kind]);
	vfprintf(g_log, fmt, ap);
	fputc('\n', g_log);
	va_end(ap);
}

void warn(const char *who, const char *fmt, ...)
{
	g_m.warn++;
	if (!g_log || g_m.warn > 40)
		return;
	va_list ap;
	va_start(ap, fmt);
	fprintf(g_log, "%12.6f %-6s WARN ", (double)s_now / 1e9, who);
	vfprintf(g_log, fmt, ap);
	fputc('\n', g_log);
	va_end(ap);
}

void stat_add(stat_t *s, int64_t v)
{
	if (s->n == 0 || v < s->min)
		s->min = v;
	if (s->n == 0 || v > s->max)
		s->max = v;
	s->n++;
	s->sum += v;
	s->sumsq += (double)v * (double)v;
}

/* ---- SDU content: [seq u32][gen ns u64][pattern dependent on seq and salt] ---- */
static uint8_t pat(uint32_t seq, unsigned i, uint8_t salt)
{
	return (uint8_t)(seq * 31u + i * 7u + salt);
}

void sdu_fill(uint8_t *buf, uint16_t len, uint32_t seq, sim_ns_t gen, uint8_t salt)
{
	uint8_t hdr[12];
	put32(hdr, seq);
	put64(hdr + 4, (uint64_t)gen);
	for (unsigned i = 0; i < len; i++)
		buf[i] = i < 12 ? hdr[i] : pat(seq, i, salt);
}

bool sdu_check(const uint8_t *buf, uint16_t len, uint8_t salt, uint32_t *seq, sim_ns_t *gen)
{
	if (len < 12)
		return false;
	*seq = get32(buf);
	*gen = (sim_ns_t)get64(buf + 4);
	for (unsigned i = 12; i < len; i++)
		if (buf[i] != pat(*seq, i, salt))
			return false;
	return true;
}
