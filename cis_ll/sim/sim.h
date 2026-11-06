/*
 * Cyberdeck LE Audio: discrete-event simulator for the CIS engine (Central) - shared types.
 *
 * World: Central (C6: CIS engine + the rest of the controller + an ideal radio), 1-2 Peripherals (earbuds,
 * own reference LL implementation), the air, a P4 host with SDIO jitter. Global time: int64 ns
 * ("real time"); each device has its own clock with drift. The Peripheral is written only from the
 * Core specification (Vol 6 Part B 4.5.13-14, 2.4.2.25-29, Part E), without looking into the engine.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "cis_platform.h"

typedef int64_t sim_ns_t;

#define US(x)  ((sim_ns_t)(x) * 1000LL)
#define MS(x)  ((sim_ns_t)(x) * 1000000LL)
#define SEC(x) ((sim_ns_t)(x) * 1000000000LL)

#define T_IFS_US 150
#define T_MSS_US 150
#define MAX_BUDS 2

/* ---------------- core: event queue ---------------- */
typedef void (*sim_fn_t)(void *ctx, uint64_t arg);
void sim_core_init(void);
void sim_core_free(void);
void sim_at(sim_ns_t t, sim_fn_t fn, void *ctx, uint64_t arg);
sim_ns_t sim_now(void);
void sim_run_until(sim_ns_t end);
void sim_stop(void);
uint64_t sim_events_processed(void);

/* ---------------- randomness (separate streams) ---------------- */
typedef struct {
	uint64_t s[4];
} rng_t;
void rng_seed(rng_t *r, uint64_t seed);
uint64_t rng_u64(rng_t *r);
double rng_unit(rng_t *r);
int64_t rng_range(rng_t *r, int64_t lo, int64_t hi);
bool rng_chance(rng_t *r, double p);
void rng_bytes(rng_t *r, uint8_t *buf, unsigned len);

/* ---------------- clocks with drift ---------------- */
typedef struct {
	int64_t ppb;       /* drift in parts per billion (+50 ppm = 50000) */
	int64_t offset_ns; /* local time at real time 0 */
} sim_clock_t;
int64_t clk_local(const sim_clock_t *c, sim_ns_t t);
sim_ns_t clk_true(const sim_clock_t *c, int64_t local);

/* ---------------- log ---------------- */
extern FILE *g_log;
void slog(const char *who, const char *fmt, ...) __attribute__((format(printf, 2, 3)));

/* ---------------- crypto (own, not from the engine) ---------------- */
void aes128_encrypt(const uint8_t key[16], const uint8_t in[16], uint8_t out[16]);
void ble_nonce(uint8_t nonce[13], uint64_t counter39, bool dir_c_to_p, const uint8_t iv_le[8]);
void ble_ccm_encrypt(const uint8_t key_be[16], const uint8_t nonce[13], uint8_t aad, const uint8_t *in,
		uint8_t len, uint8_t *out);
bool ble_ccm_decrypt(const uint8_t key_be[16], const uint8_t nonce[13], uint8_t aad, const uint8_t *in,
		uint8_t len_with_mic, uint8_t *out);
/* SK = e(LTK, SKDp||SKDc); ltk_le as in HCI, skd*_le as in the PDU; the result is MSO first (the key for e()). */
void ble_session_key(const uint8_t ltk_le[16], const uint8_t skdc_le[8], const uint8_t skdp_le[8],
		uint8_t sk_be[16]);

/* ---------------- CSA#2, access address, air time ---------------- */
typedef struct {
	uint16_t chan_id;
	uint8_t  n_used;
	uint8_t  used[37];
	uint16_t prn_lu;    /* subevent generator state */
	uint8_t  remap_idx; /* index of the last channel in the used-channels table */
} csa2_t;
void csa2_init(csa2_t *c, uint32_t aa, const uint8_t chan_map[5]);
uint8_t csa2_event(csa2_t *c, uint16_t counter);
uint8_t csa2_subevent(csa2_t *c);
/* Channel (event, subevent) from scratch, stateless. */
uint8_t csa2_chan(uint32_t aa, const uint8_t chan_map[5], uint16_t counter, uint8_t subevent);
/* NULL = valid; otherwise a description of the broken rule (Vol 6 Part B 2.1.2). */
const char *aa_check(uint32_t aa);
uint32_t air_time_us(uint8_t phy, unsigned pdu_payload_len);

/* ---------------- LE helpers ---------------- */
static inline uint16_t get16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static inline uint32_t get24(const uint8_t *p) { return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16); }
static inline uint32_t get32(const uint8_t *p) { return get24(p) | ((uint32_t)p[3] << 24); }
static inline uint64_t get64(const uint8_t *p) { return (uint64_t)get32(p) | ((uint64_t)get32(p + 4) << 32); }
static inline void put16(uint8_t *p, uint32_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static inline void put24(uint8_t *p, uint32_t v) { put16(p, v); p[2] = (uint8_t)(v >> 16); }
static inline void put32(uint8_t *p, uint32_t v) { put24(p, v); p[3] = (uint8_t)(v >> 24); }
static inline void put64(uint8_t *p, uint64_t v) { put32(p, (uint32_t)v); put32(p + 4, (uint32_t)(v >> 32)); }

/* LLCP opcodes */
enum {
	LL_TERMINATE_IND = 0x02,
	LL_ENC_REQ = 0x03,
	LL_ENC_RSP = 0x04,
	LL_START_ENC_REQ = 0x05,
	LL_START_ENC_RSP = 0x06,
	LL_UNKNOWN_RSP = 0x07,
	LL_FEATURE_REQ = 0x08,
	LL_FEATURE_RSP = 0x09,
	LL_REJECT_EXT_IND = 0x11,
	LL_CIS_REQ = 0x1F,
	LL_CIS_RSP = 0x20,
	LL_CIS_IND = 0x21,
	LL_CIS_TERMINATE_IND = 0x22,
};

/* CIS PDU header bits (byte 0). */
#define HDR_LLID_MASK 0x03
#define HDR_NESN      0x04
#define HDR_SN        0x08
#define HDR_CIE       0x10
#define HDR_NPI       0x40
#define HDR_RFU_MASK  0xA0
#define CCM_AAD_MASK  0xE3 /* NESN, SN, CIE masked in the AAD */

/* ---------------- scenario configuration ---------------- */
typedef struct {
	const char *name;
	uint64_t seed;
	sim_ns_t stream_time;     /* stream time after the CIS is established */
	/* air */
	double   loss;            /* probability of corrupting a packet */
	double   crc_frac;        /* fraction of corrupted packets received with a CRC error (the rest: no AA) */
	double   coex_duty;       /* share of time when the C6 radio is busy with Wi-Fi */
	sim_ns_t coex_min, coex_max;
	double   mic_corrupt;     /* injected payload corruption with a good CRC (detection test) */
	/* clocks */
	int32_t  ppm_c, ppm_p[MAX_BUDS], ppm_host;
	bool     host_follows_c;  /* the host paces SDUs with the controller clock (ISO TX Sync) */
	uint32_t sca_c_ppm, sca_p_ppm;
	uint32_t c_clock_start_us; /* C6 clock start (close to the uint32 wraparound) */
	/* C6 radio */
	uint8_t  max_nse;
	bool     owns_seq;
	uint32_t tx_lead_us;
	bool     subevent_hop;    /* false = a radio like the ACL: the same channel in subevents (detection test) */
	bool     prep_ahead;      /* true = like the C6 at NSE > 1: prepare_tx of all subevents before the first */
	int32_t  tx_skew_ns;      /* injected transmit offset (detection test) */
	uint32_t task_lat_us;     /* LL task latency (cis_engine_process) */
	/* ACL */
	uint8_t  n_buds;
	uint32_t acl_interval_us;
	uint32_t acl2_offset_us;
	uint32_t sup_timeout_ms;
	uint8_t  acl_phy;
	uint8_t  acl_max_octets;  /* MaxTxOctets after LE Data Length Change (27 before it) */
	sim_ns_t acl_dle_at;      /* time of LE Data Length Change; 0 = from the start */
	uint8_t  acl_reserve_octets;
	bool     encrypt;
	/* P4 host */
	uint32_t sdu_interval_us;
	uint16_t max_sdu_c2p, max_sdu_p2c;
	uint8_t  phy;
	uint8_t  rtn_c2p, rtn_p2c;
	uint16_t max_lat_c2p_ms, max_lat_p2c_ms;
	sim_ns_t host_jitter_max;
	double   stall_prob;
	sim_ns_t stall_len;
	sim_ns_t up_lat_min, up_lat_max; /* C6 -> P4 */
	/* Peripheral */
	bool     reject;
	uint8_t  reject_reason;
	bool     uplink;
	uint32_t rsp_offset_min_add_us; /* the Peripheral narrows CIS_Offset_Min in LL_CIS_RSP */
	/* events */
	sim_ns_t acl_loss_at;     /* 0 = none; from this time earbud 0 disappears (switched off) */
	/* gates */
	double   gate_max_loss_pct; /* < 0: report only */
	uint32_t gate_min_events;
	bool     gate_latency;
	bool     gate_underrun;
	bool     expect_reject;
	bool     expect_acl_loss;
	bool     expect_violations; /* negative tests: we expect detection */
} sim_cfg_t;

/* ---------------- metrics ---------------- */
enum {
	V_FORMAT,  /* bad PDU / LLCP format */
	V_TIMING,  /* outside the receive window, bad anchor/offset */
	V_CHANNEL, /* wrong channel */
	V_MIC,     /* MIC error */
	V_DESYNC,  /* SN/NESN or the payload counter */
	V_LLCP,    /* non-conforming LLCP procedure */
	V_PARAM,   /* CIS parameters contradicting the spec or the host request */
	V_CONTRACT,/* the engine violated the cis_platform.h contract */
	V_HOST,    /* HCI mismatch seen by the host */
	V_COUNT
};
extern const char *const g_vname[V_COUNT];

typedef struct {
	int64_t n, sum, min, max;
	double  sumsq;
} stat_t;
void stat_add(stat_t *s, int64_t v);

typedef struct {
	uint32_t viol[V_COUNT];
	uint32_t warn;
	/* Peripheral (downlink) */
	uint64_t cis_events, events_missed, c2p_new, c2p_dup, c2p_null, c2p_empty, c2p_flushed;
	uint64_t sdu_rx, sdu_dup, sdu_gap, sdu_corrupt, sdu_late;
	stat_t   lat_ns;
	int64_t  lat_first_ns;
	/* Peripheral (uplink) */
	uint64_t p2c_sent_payloads, p2c_acked, p2c_flushed;
	/* C6 radio (ground truth from the air) */
	uint64_t radio_events, radio_skipped, radio_subevents, overlap_acl, overlap_cis, tight_gap;
	uint64_t underruns, data_pdus, null_pdus, coex_blocked;
	/* host */
	uint64_t host_sdu_gen, host_sdu_sent, host_drop_credit, host_nocp;
	uint64_t up_rx, up_gap, up_lost_flag, up_corrupt, up_dup;
	stat_t   up_lat_ns;
	int      cis_est_status[MAX_BUDS];
	bool     cis_est_seen[MAX_BUDS];
	uint8_t  est_nse, est_bn_c, est_ft_c;
	bool     host_done, host_failed;
	char     host_fail_msg[128];
	sim_ns_t cis_disc_at[MAX_BUDS];
	bool     link_opened, link_closed_all, engine_init_failed;
	bool     eng_stats_ok;
	uint32_t eng_events, eng_skipped, eng_tx_underrun, eng_tx_flushed, eng_sdu_dropped, eng_sdu_late;
	uint32_t eng_rx_ok, eng_rx_mic_fail, eng_rx_lost;
	uint32_t eng_too_long;
	bool     p_cis_lost[MAX_BUDS];
	bool     p_established[MAX_BUDS];
} sim_metrics_t;

extern sim_cfg_t g_cfg;
extern sim_metrics_t g_m;

/* Violation: a counter + the first entries in the log. */
void viol(int kind, const char *who, const char *fmt, ...) __attribute__((format(printf, 3, 4)));
void warn(const char *who, const char *fmt, ...) __attribute__((format(printf, 2, 3)));

/* ---------------- air ---------------- */
typedef enum { AIR_OK, AIR_CRC, AIR_MISS } air_res_t;
void air_init(void);
air_res_t air_packet(void);
bool air_coex_blocked(sim_ns_t s, sim_ns_t e);
bool air_mic_corrupt(void);

/* ---------------- Reference Peripheral ---------------- */
typedef struct periph periph_t;
periph_t *periph_new(int idx, const sim_clock_t *clk, uint64_t seed, const uint8_t ltk_le[16]);
void periph_free(periph_t *p);
const sim_clock_t *periph_clock(const periph_t *p);
/* ACL: called by the ACL model. */
void periph_acl_set(periph_t *p, uint32_t interval_us, const uint8_t chan_map[5], uint32_t acl_aa);
void periph_acl_sync(periph_t *p, uint16_t evt, sim_ns_t t_true);
void periph_acl_rx_ctrl(periph_t *p, const uint8_t *ctr, uint8_t len, sim_ns_t t_true, uint16_t evt);
bool periph_acl_tx_peek(periph_t *p, const uint8_t **ctr, uint8_t *len);
void periph_acl_tx_pop(periph_t *p);
bool periph_acl_encrypted(const periph_t *p);
void periph_acl_lost(periph_t *p);
/* CIS: a Central packet on the air; true = the Peripheral responds (resp, t_resp). */
bool periph_cis_rx(periph_t *p, uint32_t aa, uint8_t chan, uint8_t phy, sim_ns_t t_start, sim_ns_t t_end,
		const cis_pdu_t *pdu, air_res_t res, cis_pdu_t *resp, sim_ns_t *t_resp, uint8_t *resp_phy);
bool periph_has_aa(const periph_t *p, uint32_t aa);
uint32_t periph_cis_aa(const periph_t *p);
void periph_finish(periph_t *p);

/* ---------------- ACL + the rest of the C6 controller ---------------- */
void acl_init(void);
int acl_count(void);
uint16_t acl_handle(int i);
int acl_find(uint16_t handle);
periph_t *acl_periph(int i);
int acl_send_from_engine(uint16_t handle, const uint8_t *ctr, uint8_t len);
int acl_timing(uint16_t handle, cis_acl_timing_t *out);
uint64_t acl_remote_features(uint16_t handle);
bool acl_connected(uint16_t handle);
/* An HCI command not consumed by the engine. */
void ctrl_hci_cmd(const uint8_t *h4, uint16_t len);
/* An LLCP PDU from the Peripheral that the engine did not consume. */
void ctrl_ll_rx(uint16_t handle, const uint8_t *ctr, uint8_t len);
/* C6 radio reservations (collision detection). Returns false when it collides with another owner. */
bool radio_claim(int owner, sim_ns_t s, sim_ns_t e, bool is_acl);
/* Whether the interval collides with a future ACL event (reservation). */
bool acl_window_overlap(sim_ns_t s, sim_ns_t e);
void acl_finish(void);

/* ---------------- platform (engine ops) + ideal radio ---------------- */
bool plat_init(void);
/* Engine counters (cis_engine_get_stats) for a CIS handle -> g_m.eng_* (sum). */
void plat_engine_stats(uint16_t cis_handle);
void plat_finish(void);
void plat_kick(void);         /* schedule cis_engine_process */
void plat_c6_rx_h4(const uint8_t *h4, uint16_t len); /* a packet from the host reached the C6 */
void plat_ctrl_evt(const uint8_t *h4, uint16_t len); /* controller event -> engine snoop -> host */
void plat_ll_ctrl_rx(uint16_t handle, const uint8_t *ctr, uint8_t len);
void plat_ll_ctrl_tx_seen(uint16_t handle, const uint8_t *ctr, uint8_t len);
const sim_clock_t *plat_c_clock(void);
int64_t plat_c_local_us64(sim_ns_t t);
sim_ns_t plat_c_true_of_us64(int64_t us);
const char *plat_engine_name(void);

/* ---------------- P4 host ---------------- */
void host_init(void);
void host_rx_h4(const uint8_t *h4, uint16_t len); /* from the C6, after the transport delay */
void host_finish(void);
sim_ns_t host_stream_stop_time(void);

/* Test SDU content: seq, generation time, pattern. */
void sdu_fill(uint8_t *buf, uint16_t len, uint32_t seq, sim_ns_t gen, uint8_t salt);
bool sdu_check(const uint8_t *buf, uint16_t len, uint8_t salt, uint32_t *seq, sim_ns_t *gen);

/* Unit tests (Core vectors, FIPS-197, CSA#2, AA). */
int selftest_units(void);
