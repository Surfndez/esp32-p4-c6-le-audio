/*
 * Cyberdeck LE Audio: ESP32-C6 radio adapter for the CIS engine (Central, NSE 1 or 2, BN = FT = 1).
 *
 * A CIS subevent = pseudo-connection: an extra Central connsm of the closed controller with the CIS AA, CRC init from
 * the ACL, ISO interval, CSA#2 and our first anchor (NSE 2: second connsm with anchor + Sub_Interval, subevent channel
 * 1, docs/c6-link-layer.md). The hardware sequencer handles timing, T_IFS and the RX window; the hooks below supply one
 * ready (encrypted) LLID 00 PDU per connsm event, collect the raw Peripheral PDU before HCI, write the engine's SN/NESN
 * before every event and keep the link at the highest scheduler priority.
 *
 * Derived from analysis of the controller library: "N:Qx" / "N:1.x" = section of our analysis notes (not
 * included), "obj: fn+0xOFF" = location in libble_app.a. Everything here runs in the LL task (N:Q4). Points marked
 * UNVERIFIED were inferred, not yet confirmed on hardware.
 */
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "esp_bt.h"
#include "esp_random.h"

#include "cis_c6.h"
#include "cis_proto.h"

/* ---------- controller symbols (libble_app.a), prototypes from analysis ---------- */

/* ble_ll_conn.c: sm_get -> sm_fetch_new(int *err): *err = 0, 7 when no connsm is free (fetch_new+0x16, +0x6c);
 * returns the connsm or NULL. The RE sketch had (void) - it would write zero to a random address. */
extern void *r_ble_ll_conn_sm_get(int *err);
/* ble_ll_conn.c: defaults of a fresh connsm (sm_new+0x0..0x18: a0 = connsm); at the end it inserts it into the
 * active list with the lowest free handle (tail jump sm_new+0x26a -> sm_active_insert). */
extern void r_ble_ll_conn_sm_new(void *sm);
/* ble_ll_conn.c: active list by u16 handle (NOTES_cis_llcp Q5). */
extern void *r_ble_ll_conn_sm_find_active_by_handle(uint16_t handle);
/* ble_ll_conn.c: CSA#2 - bit21, channel identifier +64 from the AA, +67 = channel (set_csa+0x30..0x60). */
extern void r_ble_ll_conn_set_csa(void *sm, int use_csa2);
/* ble_ll_conn.c: +164 = +140 * 1250 ticks (calc_itvl_ticks+0x4..0x52). */
extern uint32_t r_ble_ll_conn_calc_itvl_ticks(void *sm);
/* ble_ll.c: PDU time in us (payload length, PHY mode 1/2/3; other mode = r_ll_assert, pdu_tx_time_get+0x6..0x32). */
extern uint32_t r_ble_ll_pdu_tx_time_get(uint16_t len, int mode);
/* ble_lll_conn.c: master_new(sm, rxhdr, pdu): link state, buffers, reset (AA, CRC), first anchor via
 * calc_first_anchor_point, sched_insert (master_new+0x12..0x16, +0x58, +0x114). */
extern int r_ble_lll_conn_master_new(void *sm, void *rxhdr, void *pdu);
/* ble_lll_conn.c: end(sm, 0) without Disconnection Complete (conn_end+0x176). Hooked (below): the controller ends the
 * connsm itself (32 s supervision, failed next_event in sched_next_anchor+0x10c, master_new+0x1a4). */
void *r_ble_lll_conn_end(void *sm, uint8_t reason);
extern void *orig_r_ble_lll_conn_end(void *sm, uint8_t reason);
/* ble_lll_mmgmt.c: 16 B descriptor + len data + 18 B from the pool (alloc_tx_buffer_and_hdr+0xe..0x4c: flag bits 3-10 =
 * len, +8 = data; alloc_tx_buffer+0x4: len + 18). Pool = pointer ls+112 (append_tx_buffer+0x2e). */
extern void *r_ble_lll_mmgmt_alloc_tx_buffer_and_hdr(void *pool, uint16_t len);
/* ble_lll.c: collecting RX buffers like update_buffer_hdr+0xc8 / +0xd4. */
extern void *r_ble_lll_get_rxed_buffer(void *ls, void *upto);
extern void r_ble_lll_conn_align_pld_cnt_with_rxbuffer(void *sm, int one, void *desc);
extern void r_ble_lll_append_rx_buffer(void *ls, void *desc);
extern void r_ble_lll_mmgmt_free_tx_buffer_hdr(void *desc);
/* ble_lll.c: current_tx_info_get(ls, &desc, &seq) under hardware block 0x600A1B00 (+0x0..0x4a). */
extern void r_ble_lll_current_tx_info_get(void *ls, void **desc, uint8_t *seq);
/* ble_lll.c: the same for RX: desc from ls+8, flag = !bit27 ls+20 (current_rx_info_get+0x1e..0x4a). */
extern void r_ble_lll_current_rx_info_get(void *ls, void **desc, uint8_t *done);
/* ble_lll_timer.c: 1 us LL clock (register 0x600A1924), N:Q7. */
extern uint32_t r_ble_lll_timer_current_tick_get(void);
extern uint8_t r_ble_phy_rate_to_mode(uint8_t rate);
/* ble_phy.c: rx_process+0x19a..0x1ae - hardware timestamp -> packet start for the PHY mode. */
extern uint32_t r_ble_phy_get_actual_tx_time(uint32_t hw_ts, uint8_t mode);

/* ble_ll_conn.c: connsm from the active list by number (recycle_buffer+0x5a). */
extern void *r_ble_ll_conn_find_active_conn(uint16_t num);
/* ble_ll.c: controller options; +66 u16 = base of connsm numbers (recycle_buffer+0x18..0x2e). */
extern const uint8_t *r_sdkconfig_get_opts(void);

/* Originals from the objcopy step (cmake/ble_hooks.txt). */
extern uint32_t orig_r_ble_lll_conn_calc_first_anchor_point(void *sm, void *rxhdr, void *pdu);
extern int orig_r_ble_lll_conn_update_buffer_hdr(void *sm, void *ls);
extern int orig_r_ble_ll_conn_next_event(void *sm);
extern void orig_r_ble_lll_conn_coex_dpc_process(void *sm, int role_idx);
extern void orig_r_ble_lll_conn_recycle_buffer(void *sch, uint8_t sm_num, void *a2, void *a3);

/* Our (strong) definitions replacing the weakened originals. */
uint32_t r_ble_lll_conn_calc_first_anchor_point(void *sm, void *rxhdr, void *pdu);
int r_ble_lll_conn_update_buffer_hdr(void *sm, void *ls);
int r_ble_ll_conn_next_event(void *sm);
void r_ble_lll_conn_coex_dpc_process(void *sm, int role_idx);
void r_ble_lll_conn_recycle_buffer(void *sch, uint8_t sm_num, void *a2, void *a3);

/* ---------- offsets (N:1.1 connsm, N:1.2 link state, N:1.4/1.5 buffers) ---------- */
#define SM_FLAGS          0   /* u32: bit21 CSA#2, bits 28/29 pending RRF event (must be 0) */
#define SM_HANDLE         8   /* u16: HCI handle (sm_active_insert+0x44) */
#define SM_STATE          10  /* u8: 1 created */
#define SM_ROLE           11  /* u8: 1 Central */
#define SM_EFF_OCTETS     20  /* u8 TX, u8 RX (+21): effective DLE, sm_new 27/27 */
#define SM_EFF_TIME       30  /* u16 TX, u16 RX (+32): effective DLE in us, sm_new 328/328 */
#define SM_PHY            40  /* u32: bits 0-1 / 2-3 PHY modes */
#define SM_CHMAP          51  /* u8[5] */
#define SM_CHAN           67  /* u8: data channel of the scheduled event (set_csa+0x3e, next_event+0x104) */
#define SM_CHANS_USED     69
#define SM_LAST_CTR       100 /* u16 */
#define SM_EVENT_CTR      102 /* u16: scheduled event counter */
#define SM_AA             112
#define SM_CRC_INIT       116
#define SM_EVENT_END      124 /* u32: end of the scheduled event (tick = us) */
#define SM_CREATE_TICK    132
#define SM_ITVL           140 /* u16, 1.25 ms */
#define SM_LATENCY        142
#define SM_SPVN_TMO       144
#define SM_TX_WIN_OFF     150
#define SM_ANCHOR         156 /* u32: anchor of the scheduled event */
#define SM_PEER_ADDR_TYPE 186
#define SM_PEER_ADDR      187
#define SM_LINK_STATE     260
#define SM_ENC_TX         273 /* u8 bit0: hardware CCM on TX */
#define SM_MISSED         532 /* u16 counter, u16 +534 limit */
#define SM_LLL_FLAGS      566 /* u16 bit6: hardware CCM on RX, bit7: event time limit with DLE */
#define SM_LLL_FLAGS2     568

#define LS_TX_CUR         0   /* u32: bits 0-19 current TX buffer (descriptor + 4), bits 20-27 length limit */
#define LS_SEQ            20  /* u32: bit26 SN, bit27 NESN (reset_link_state+0x2a..0x6a) */
#define LS_TX_HEAD        96  /* ptr: head of the TX chain (alloc_memory+0x140, reset_link_state+0x32) */
#define LS_TX_TAIL        104 /* ptr: tail of the TX chain, append inserts after it (alloc_memory+0x142) */
#define LS_TX_POOL        112 /* ptr: TX buffer pool (append_tx_buffer+0x2e) */
#define LS_PRIO           84  /* u8: scheduler priority */

#define HW_PTR(v) ((uint8_t *)(uintptr_t)(((v) & 0xfffffu) | 0x40800000u)) /* 20-bit SRAM offsets, N:0 */
#define DESC_DATA         8
#define TXD_HDR           16
#define TXD_LEN           17
#define TXD_PAYLOAD       18
#define RXD_STATUS        12  /* u32 bits 0-23; 0xFFFFFF = not written; & 0x17 = bad packet */
#define RXD_TS            16
#define RXD_RATE          24  /* bits 24-25 */
#define RXD_HDR           28
#define RXD_LEN           29
#define RXD_PAYLOAD       30

#define RX_STATUS_BAD_MASK 0x17u /* rx_process+0x7e: andi s4,s4,23. Meaning of the bits UNVERIFIED. */
#define RX_STATUS_EMPTY    0xFFFFFFu
#define CIS_PRIO_MAX       15     /* priority_increase+0x24: 15 = urgent */
#define CIS_C6_MAX_LINKS   2      /* L + P */
#define CIS_C6_MAX_NSE     2      /* link subevents = connsm from the pool (CONFIG_BT_LE_MAX_CONNECTIONS) */
/* R5: the connsm pool holds both earbud ACLs and all subevents of both CIS (sdkconfig.defaults.esp32c6). */
_Static_assert(CONFIG_BT_LE_MAX_CONNECTIONS >= 2 + CIS_C6_MAX_LINKS * CIS_C6_MAX_NSE,
	       "CONFIG_BT_LE_MAX_CONNECTIONS: 2 ACL + CIS_C6_MAX_LINKS x CIS_C6_MAX_NSE connsm");
/* prepare_tx runs in the next_event hook, right after the previous event (NSE 2: after its last subevent, i.e.
 * one Sub_Interval later): at most one ISO interval before the anchor. */
#define CIS_C6_TX_LEAD_US  10000u

/* CIS PDU header (Core Vol 6 Part B 2.6.2, contract cis_platform.h): byte 0 = LLID[1:0] NESN SN CIE RFU NPI RFU. */
#define HDR_LLID   0x03u
#define HDR_NESN   0x04u
#define HDR_SN     0x08u
#define HDR_NPI    0x40u /* bit 6 = RFU in ACL, so the hardware does not interpret it (R2 resolved in the log) */
/* CIE (bit 4) is left out: in ACL it is MD, which the hardware sets; a connsm event ends after one exchange anyway,
 * and MD = 1 from us could extend the event in the sequencer (R3). With NSE 2, CIE = 0 is even required: with CIE = 1
 * the earbud could close the event and not listen to subevent 1 (CIE, Core Vol 6 Part B 2.6.2). */
#define HDR_TX_MASK (HDR_LLID | HDR_NESN | HDR_SN | HDR_NPI)

#define LS_COEX           40  /* u32: bits 0-3 event PTI (coex_dpc_process), 4-7 second PTI (reset_link_state) */

/* ---------- experiment switches (cis_c6.h) ---------- */
/* Defaults = earlier adapter constants, except the coexistence PTI: 15 = highest (ESP-IDF only in esp_reset_modem;
 * controller: 4 normal connection, 9 urgent; Wi-Fi coex_pti_tab mostly 4..10) - design choice: audio before Wi-Fi,
 * Wi-Fi packets may be late. Cost to Wi-Fi about 1.2 ms per 10 ms per CIS link. */
uint32_t g_cis_c6_knob[CIS_C6_KNOB_COUNT] = {
	[CIS_C6_KNOB_SEQ_POLARITY] = 0,  /* link reset zeroes the bits = SN/NESN 0 as in Core (UNVERIFIED R1) */
	[CIS_C6_KNOB_SEQ_WRITE] = 5, /* Tab5 + Buds3 Pro 2026-11-01: 99.2 % acked, smooth audio (mode 1: 34 %) */
	[CIS_C6_KNOB_HDR_SEQ] = 1,
	[CIS_C6_KNOB_MIN_LEAD_US] = 3000, /* UNVERIFIED: allocation + scheduler preparation (sched_env[0]) */
	[CIS_C6_KNOB_SCHED_PRIO] = CIS_PRIO_MAX,
	[CIS_C6_KNOB_COEX_PTI] = 15,
	[CIS_C6_KNOB_COEX_PTI_HI] = 0,
	[CIS_C6_KNOB_EVT_LIMIT] = 1,
	[CIS_C6_KNOB_NESN_TRACK] = 0,
	[CIS_C6_KNOB_SN_TRACK] = 0,
	/* Extra gap between CISes on top of subevent_gap_us, which since 1.4.75 the engine adds after every subevent, also
	 * with NSE 1 (cis_slot_us). Consecutive connsm events 1.56 ms apart work (train 2026-11-01), 1.06 ms does not (1.3 %
	 * acked); with NSE 2 every additional 2500 us would push the CIG past the ISO interval (2 x 5.62 ms). */
	[CIS_C6_KNOB_CIS_GAP_US] = 0,
	[CIS_C6_KNOB_TX_DBM] = 0,
	/* UNVERIFIED: an ACL event with data runs past its scheduled end. 1.4.72 put the CIS 1000 us after the ACL (no
	 * margin) and the right one lost 10.5 % vs 4.9 % for the left; the earbud itself chose 1312 us. Tab5 + Buds3 Pro
	 * at home 2026-11-02: at 500 us the second earbud lost its ACL (0x08) 6 s and 3.5 min after start, SDU 0.7 / 0.35 %;
	 * at 2000 us 3 min without a drop, 0.006 / 0.06 % - the earbud does not serve its ACL right next to CIS subevents. */
	[CIS_C6_KNOB_ACL_MARGIN_US] = 2000,
	/* Tab5 with Buds3 Pro, train 2026-11-01 (1.4.74, 3 x 20 s alternating): NSE 1 2.5 / 3.0 % SDU loss, NSE 2 0.30 /
	 * 0.34 %, se_open_fail/se_skipped/se_late 0. With a microphone the engine takes NSE 1 anyway (max_nse_p_to_c). */
	[CIS_C6_KNOB_MAX_NSE] = 2,
	/* Measured (train 2026-11-01) for two CIS, each in its own connsm - second anchor 1.06 ms after the first: 1.3 %
	 * acked, 1.56 ms (se_length about 1060 us + 500): losses as at 3.56 ms; subevents of the same CIS with the same
	 * AA 1.56 ms apart also work (1.4.74, above). */
	[CIS_C6_KNOB_SUBEVENT_GAP_US] = 500,
	/* Tab5 2026-11-01 (1.4.75, NSE 2, FT 4): with Max_Transport_Latency 40 ms only 2 SDUs of margin remained, P4 SDIO
	 * stalls of up to 14 ms (SD card read) gave 0.6-0.8 % late SDUs (sdu_late) - audible micro-dropouts at 0.2-0.4 % radio
	 * loss. 4 SDUs = 40 ms of margin, +20 ms audio latency. */
	[CIS_C6_KNOB_JITTER_MIN] = 4,
	/* 16 slots = 32 host credits (until 1.4.77: 12 = 24): margin for credit return (NOCP over the SDIO shared with the
	 * SD card) 70 ms instead of 30 ms. Tab5 2026-11-02: min free credits 6-13 instead of 0-4, return stalls up to 63 ms
	 * with a faulty card (docs/c6-link-layer.md). 12 is kept for comparisons. */
	[CIS_C6_KNOB_TX_SLOTS] = 16,
};

static const struct { uint32_t lo, hi; } k_knob_range[CIS_C6_KNOB_COUNT] = {
	[CIS_C6_KNOB_SEQ_POLARITY] = { 0, 1 },
	[CIS_C6_KNOB_SEQ_WRITE] = { 0, 5 },
	[CIS_C6_KNOB_HDR_SEQ] = { 0, 1 },
	[CIS_C6_KNOB_MIN_LEAD_US] = { 500, 100000 },
	[CIS_C6_KNOB_SCHED_PRIO] = { 0, 15 },
	[CIS_C6_KNOB_COEX_PTI] = { 0, 15 },
	[CIS_C6_KNOB_COEX_PTI_HI] = { 0, 15 },
	[CIS_C6_KNOB_EVT_LIMIT] = { 0, 1 },
	[CIS_C6_KNOB_NESN_TRACK] = { 0, 0 }, /* 1.4.60 on Tab5: bit 27 = 1 before the event = crash in CIS - locked */
	[CIS_C6_KNOB_SN_TRACK] = { 0, 2 },
	[CIS_C6_KNOB_CIS_GAP_US] = { 0, 6000 },
	[CIS_C6_KNOB_TX_DBM] = { 0, 20 },
	[CIS_C6_KNOB_ACL_MARGIN_US] = { 0, 3000 },
	[CIS_C6_KNOB_MAX_NSE] = { 1, CIS_C6_MAX_NSE },
	[CIS_C6_KNOB_SUBEVENT_GAP_US] = { 0, 3000 },
	[CIS_C6_KNOB_JITTER_MIN] = { 0, 4 },
	[CIS_C6_KNOB_TX_SLOTS] = { 12, 16 },
};

/* Power in dBm -> controller level (esp_bt.h C6: P3 = 9 ... P18 = 14, P20 = 15); -1 = no such level. */
static int tx_level(uint32_t dbm)
{
	switch (dbm) {
	case 3: return ESP_PWR_LVL_P3;
	case 6: return ESP_PWR_LVL_P6;
	case 9: return ESP_PWR_LVL_P9;
	case 12: return ESP_PWR_LVL_P12;
	case 15: return ESP_PWR_LVL_P15;
	case 18: return ESP_PWR_LVL_P18;
	case 20: return ESP_PWR_LVL_P20;
	default: return -1;
	}
}

/* CIS radio losses grow with interference (hotspot, train: 2-5.5 % unacked at +9 dBm from sdkconfig).
 * Default power applies to new connections (CIS pseudo-connections at the next stream start), CONN - to existing ones
 * (handles 0..CONFIG_BT_LE_MAX_CONNECTIONS-1, free ones return an error, which we ignore). */
static void apply_tx_power(uint32_t dbm)
{
	const int lvl = tx_level(dbm);
	if (lvl < 0) {
		return;
	}
	(void)esp_ble_tx_power_set_enhanced(ESP_BLE_ENHANCED_PWR_TYPE_DEFAULT, 0, (esp_power_level_t)lvl);
	for (uint16_t h = 0; h < CONFIG_BT_LE_MAX_CONNECTIONS; h++) {
		(void)esp_ble_tx_power_set_enhanced(ESP_BLE_ENHANCED_PWR_TYPE_CONN, h, (esp_power_level_t)lvl);
	}
}

bool cis_c6_knob_set(uint8_t id, uint32_t value)
{
	if (id >= CIS_C6_KNOB_COUNT || value < k_knob_range[id].lo || value > k_knob_range[id].hi) {
		return false;
	}
	if (id == CIS_C6_KNOB_TX_DBM) {
		if (value != 0 && tx_level(value) < 0) {
			return false;
		}
		apply_tx_power(value);
	}
	g_cis_c6_knob[id] = value;
	return true;
}

static inline uint32_t knob(cis_c6_knob_t k)
{
	return g_cis_c6_knob[k];
}

/* ---------- black box (cis_c6.h) ---------- */
_Static_assert(sizeof(cis_c6_bb_t) == 24, "entry layout = format 0xFC30/0x04");

static cis_c6_bb_t s_bb_first[CIS_C6_BB_FIRST];
static uint32_t s_bb_first_n;
static cis_c6_bb_t s_bb_last[CIS_C6_BB_LAST];
static uint32_t s_bb_last_n;

static void bb_commit(const cis_c6_bb_t *e)
{
	if (s_bb_first_n < CIS_C6_BB_FIRST) {
		s_bb_first[s_bb_first_n++] = *e;
	}
	s_bb_last[s_bb_last_n % CIS_C6_BB_LAST] = *e;
	s_bb_last_n++;
}

uint8_t cis_c6_bb_read(uint8_t which, uint16_t index, uint8_t max, cis_c6_bb_t *out, uint32_t *total)
{
	uint8_t n = 0;

	if (which == 0) {
		*total = s_bb_first_n;
		while (n < max && (uint32_t)index + n < s_bb_first_n) {
			out[n] = s_bb_first[index + n];
			n++;
		}
		return n;
	}
	*total = s_bb_last_n;
	const uint32_t kept = s_bb_last_n < CIS_C6_BB_LAST ? s_bb_last_n : CIS_C6_BB_LAST;
	const uint32_t oldest = s_bb_last_n - kept;
	while (n < max && (uint32_t)index + n < kept) {
		out[n] = s_bb_last[(oldest + index + n) % CIS_C6_BB_LAST];
		n++;
	}
	return n;
}

/* ---------- CIS setup trace (cis_c6.h, engine ops.trace) ---------- */
_Static_assert(sizeof(cis_c6_tr_t) == 28, "entry layout = format 0xFC30/0x0A");

static cis_c6_tr_t s_tr[CIS_C6_TR_KEPT];
static uint32_t s_tr_n;
/* At LL_CIS_IND: the real anchor of the ACL event with this counter (next_event of the foreign connsm), compared with
 * the anchor the engine used to compute the CIS start (IND entry): does the CIS start where we told the earbuds. */
static struct {
	bool armed;
	uint16_t cis;
	uint16_t acl;
	uint16_t ev;
} s_watch;

static void tr_put(const cis_trace_t *t)
{
	cis_c6_tr_t *e = &s_tr[s_tr_n % CIS_C6_TR_KEPT];
	e->t_us = cis_c6_now_us(NULL);
	e->tr = *t;
	s_tr_n++;
}

void cis_c6_trace(void *ctx, const cis_trace_t *t)
{
	(void)ctx;
	tr_put(t);
	if (t->kind == CIS_TR_IND) {
		s_watch.cis = t->cis;
		s_watch.acl = t->acl;
		s_watch.ev = (uint16_t)t->v[0];
		s_watch.armed = true;
	} else if (t->kind == CIS_TR_CREATE || t->kind == CIS_TR_END ||
		   (t->kind == CIS_TR_ESTABLISHED && t->code != 0)) {
			/* Setup just finished or a new one began: the ACL handle gets reused with another event counter, and the stale
			 * watch would hit a foreign connection. */
		s_watch.armed = false;
	}
}

uint8_t cis_c6_trace_read(uint32_t *index, uint8_t max, cis_c6_tr_t *out, uint32_t *total)
{
	const uint32_t oldest = s_tr_n > CIS_C6_TR_KEPT ? s_tr_n - CIS_C6_TR_KEPT : 0u;
	uint8_t n = 0;

	*total = s_tr_n;
	if (*index < oldest) {
		*index = oldest;
	}
	while (n < max && *index + n < s_tr_n) {
		out[n] = s_tr[(*index + n) % CIS_C6_TR_KEPT];
		n++;
	}
	return n;
}

static inline uint8_t *f8(void *base, size_t off) { return (uint8_t *)base + off; }
static inline uint16_t rd16(void *base, size_t off) { uint16_t v; memcpy(&v, f8(base, off), 2); return v; }
static inline uint32_t rd32(void *base, size_t off) { uint32_t v; memcpy(&v, f8(base, off), 4); return v; }
static inline void wr16(void *base, size_t off, uint16_t v) { memcpy(f8(base, off), &v, 2); }
static inline void wr32(void *base, size_t off, uint32_t v) { memcpy(f8(base, off), &v, 4); }
static inline void *link_state(void *sm) { return *(void **)f8(sm, SM_LINK_STATE); }
/* Link-state block words (ls+0, +20, +40) that the hardware also reads and writes: one lw/sw as in the controller, not
 * memcpy (which compiles to four byte accesses). The block is 4-aligned (mmgmt pool). */
static inline uint32_t ls_rd(void *ls, size_t off) { return *(volatile uint32_t *)f8(ls, off); }
static inline void ls_wr(void *ls, size_t off, uint32_t v) { *(volatile uint32_t *)f8(ls, off) = v; }

/* ---------- link table ---------- */
/* PDU received during an event (recycle_buffer hook): a copy, because the buffer returns to the hardware at once; the
 * engine gets it in update_buffer_hdr of that event. A connsm event is one exchange: at most one reply, the second slot
 * is spare. */
#define CIS_C6_RX_STASH 2
struct rx_raw {
	uint32_t status;             /* hardware status (24 bits) */
	uint32_t ts;                 /* packet start, anchor clock */
	uint8_t hdr;
	uint8_t len;
	uint8_t payload[CIS_PDU_MAX_PAYLOAD];
};

/* A CIS subevent = controller connsm with its own link-state block (ls), TX buffer and RX set. Everything about ls or
 * the events of one connsm is here; settling of CIS events for the engine - in struct cis_link. */
struct cis_sub {
	void *sm;                    /* controller connsm, NULL = none (slot free or connsm ended) */
	uint32_t next;               /* lowest CIS event this connsm has not settled yet (not sent over the
	                             * air, or is scheduled); only this connsm's hooks advance it */
	cis_c6_bb_t bb;              /* black-box entry of the loaded event (tx_load -> update_buffer_hdr) */
	void *txb;                   /* own TX buffer (descriptor): the connsm's permanent current buffer (install_txb) */
	bool txb_fresh;              /* txb still has no PDU (the first tx_load marks a CIS_C6_BB_TX_NEW entry) */
	bool seq_forced;             /* the adapter wrote SN/NESN to ls+20 since the last buffer collection */
	uint32_t seq_pre;            /* ls+20 before the write - controller state */
	uint32_t seq_set;            /* ls+20 after the write */
	uint32_t seq_m;              /* bits written last (seq_mask) - only these are restored by seq_restore / seq_resync */
	uint32_t ls0_hi;             /* ls+0 bits 20-31 from install_txb (length limit) */
	uint8_t tx_sn;               /* SN written to ls+20 bit24 in mode 5 - ack for the engine after the event */
	uint8_t rx_n;                /* PDUs in rx_stash */
	struct rx_raw rx_stash[CIS_C6_RX_STASH];
};

struct cis_link {
	struct cis_sub se[CIS_C6_MAX_NSE]; /* se[i] = subevent i; all sm == NULL and !dead = free slot */
	uint8_t nse;                 /* subevents in the contract (cis_link_params_t.nse) */
	uint8_t n_sm;                /* link connsm count: nse, or 1 if the second did not open (subevent 1 always skipped) */
	uint8_t air;                 /* bits of subevents of event next_ev that went over the air (rx_done already given) */
	const cis_link_cb_t *cb;
	void *cb_arg;
	uint32_t first_anchor_us;
	uint32_t sub_interval_us;
	uint8_t tx_cap;              /* TX buffer capacity in the hardware chain (Max_PDU C->P with MIC) */
	uint8_t chmap[5];            /* CIS channel map and CSA#2 channel identifier from the AA - channel of subevent 1 */
	uint16_t chan_id;
	uint32_t next_ev;            /* 32-bit counter of the next event not yet settled */
	uint32_t loaded_ev;          /* event for which the engine already supplied a PDU (prepare_tx) - one load */
	bool broken;                 /* connsm counter went backwards: no engine to close it (CIS supervision in the engine) */
	bool opening;                /* during master_new (link_open) - a controller conn_end here means the open failed */
	bool dead;                   /* controller ended the connsm: slot waits for engine's link_close (cis_c6_radio_tick) */
	bool rx_seen;                /* last received earbud header (NESN_TRACK / SN_TRACK) */
	uint32_t rx_last_ev;
	uint8_t rx_last_hdr;
	bool rx_last_acc;            /* in event rx_last_ev the earbuds accepted our PDU */
};

static struct cis_link s_links[CIS_C6_MAX_LINKS];
/* The LL task writes, diagnostics read (atomically, no lock) - cis_c6_chan_stats. */
static uint32_t s_ch_tries[CIS_C6_CHANNELS];
static uint32_t s_ch_bad[CIS_C6_CHANNELS];
/* ACL connections (foreign connsm) - only the LL task writes; diagnostics read fields separately (pairs may tear). */
static cis_c6_acl_st_t s_acl[CIS_C6_ACL_KEPT] = {
	[0 ... CIS_C6_ACL_KEPT - 1] = { .handle = 0xFFFF },
};

static cis_c6_acl_st_t *acl_slot(void *sm)
{
	const uint16_t h = rd16(sm, SM_HANDLE);
	cis_c6_acl_st_t *free_slot = NULL;
	for (int i = 0; i < CIS_C6_ACL_KEPT; i++) {
		if (s_acl[i].handle == h) {
			return &s_acl[i];
		}
		if (free_slot == NULL && s_acl[i].handle == 0xFFFF) {
			free_slot = &s_acl[i];
		}
	}
	if (free_slot == NULL) {
		free_slot = &s_acl[h % CIS_C6_ACL_KEPT];  /* more connections than slots - overwrite, counters from zero */
	}
	*free_slot = (cis_c6_acl_st_t){ .handle = h, .itvl = rd16(sm, SM_ITVL), .spvn_tmo = rd16(sm, SM_SPVN_TMO),
					.latency = rd16(sm, SM_LATENCY) };
	return free_slot;
}

static void acl_planned(void *sm)
{
	cis_c6_acl_st_t *a = acl_slot(sm);
	const uint16_t itvl = rd16(sm, SM_ITVL), spvn = rd16(sm, SM_SPVN_TMO), lat = rd16(sm, SM_LATENCY);
	if (itvl != a->itvl || spvn != a->spvn_tmo || lat != a->latency) {
		a->itvl = itvl;
		a->spvn_tmo = spvn;
		a->latency = lat;
		__atomic_fetch_add(&a->changes, 1u, __ATOMIC_RELAXED);
	}
	__atomic_fetch_add(&a->planned, 1u, __ATOMIC_RELAXED);
}

void cis_c6_acl_stats(cis_c6_acl_st_t out[CIS_C6_ACL_KEPT])
{
	for (int i = 0; i < CIS_C6_ACL_KEPT; i++) {
		out[i] = s_acl[i];
		out[i].planned = __atomic_load_n(&s_acl[i].planned, __ATOMIC_RELAXED);
		out[i].aired = __atomic_load_n(&s_acl[i].aired, __ATOMIC_RELAXED);
	}
}

/* Called on every event of every connection: comparisons only, no other calls. *se = subevent index. */
static inline struct cis_link *link_by_sm(void *sm, uint8_t *se)
{
	if (sm == NULL) {
		return NULL;
	}
	for (int i = 0; i < CIS_C6_MAX_LINKS; i++) {
		for (uint8_t s = 0; s < CIS_C6_MAX_NSE; s++) {
			if (s_links[i].se[s].sm == sm) {
				*se = s;
				return &s_links[i];
			}
		}
	}
	return NULL;
}

static bool link_has_sm(const struct cis_link *l)
{
	for (uint8_t s = 0; s < CIS_C6_MAX_NSE; s++) {
		if (l->se[s].sm != NULL) {
			return true;
		}
	}
	return false;
}

static inline uint8_t bb_se_flag(uint8_t s)
{
	return s != 0 ? CIS_C6_BB_SE1 : 0u;
}

/* controller u16 -> contract u32: counters only move forward by a few events per call. A step back (the controller
 * rescheduled the event, e.g. enqueue_pkt -> event_delete_and_reschedule rolls back the counter and anchor together)
 * is not a wrap-around: the over-the-air counter and our event number (CCM packet counter, channel) would diverge for
 * good. The link then goes into "broken" mode: zero PDUs without the engine, no engine calls - its supervision (5 s
 * without reception) closes the CIS the normal way. Reference = this connsm's settled count (NSE 1: always next_ev). */
static inline uint32_t ext_ctr(struct cis_link *l, const struct cis_sub *u, uint16_t c16)
{
	const int16_t d = (int16_t)(uint16_t)(c16 - (uint16_t)u->next);
	if (d < 0) {
		if (!l->broken) {
			CIS_C6_CNT(ctr_backwards);
		}
		l->broken = true;
		return u->next;
	}
	return u->next + (uint32_t)d;
}

/* This connsm is past the CIS events before 'upto': they did not go over the air (schedule conflict, coexistence). */
static inline void sub_skip_to(struct cis_sub *u, uint32_t upto)
{
	if ((int32_t)(upto - u->next) > 0) {
		u->next = upto;
	}
}

/* Closes event next_ev, which all connsm of the link have settled. No subevent on the air = event_skipped (as with
 * NSE 1); otherwise rx_done without PDU for the skipped subevents (the engine waits for an acknowledgement from any
 * subevent) and one event_closed. A subevent without a connsm (n_sm < nse) is an always-skipped subevent - the contract
 * is the same, so the link behaves like NSE 1 on the air with no separate path in the engine. */
static void close_event(struct cis_link *l)
{
	const uint32_t ev = l->next_ev;

	if (l->air == 0) {
		const cis_c6_bb_t e = { .ev = (uint16_t)ev, .link = (uint8_t)(l - s_links),
					.flags = CIS_C6_BB_SKIPPED, .rx_status = 0xFFFFFFFFu };
		CIS_C6_CNT(ev_skipped);
		bb_commit(&e);
		l->cb->event_skipped(l->cb_arg, ev);
	} else {
		for (uint8_t s = 0; s < l->nse; s++) {
			if (l->air & (1u << s)) {
				continue;
			}
			if (s < l->n_sm) {
				const cis_c6_bb_t e = { .ev = (uint16_t)ev, .link = (uint8_t)(l - s_links),
							.flags = (uint8_t)(CIS_C6_BB_SKIPPED | bb_se_flag(s)),
							.rx_status = 0xFFFFFFFFu };
				CIS_C6_CNT(se_skipped);
				bb_commit(&e);
			}
			l->cb->rx_done(l->cb_arg, ev, s, NULL, false, 0);
		}
		l->cb->event_closed(l->cb_arg, ev);
		CIS_C6_CNT(ev_done);
	}
	l->air = 0;
	l->next_ev = ev + 1u;
}

/* Closes in order the events settled by all connsm of the link. NSE 1: each at once (like the former report_skipped
 * and event_closed in update_buffer_hdr). NSE 2: the event is closed by the connsm that settled it last - normally
 * update_buffer_hdr of subevent 1, and when one is skipped, also the next_event hook. */
static void close_ready(struct cis_link *l)
{
	if (l->n_sm == 0) {
		return; /* slot zeroed (cis_c6_radio_reset) - nothing to settle */
	}
	for (;;) {
		for (uint8_t s = 0; s < l->n_sm; s++) {
			if ((int32_t)(l->se[s].next - l->next_ev) <= 0) {
				return;
			}
		}
		close_event(l);
	}
}

/* ---------- SN/NESN (N:Q3, simulator: hwseq drops 56 % of the audio without this) ---------- */
/* The hardware keeps SN/NESN in ls+20 bits 26/27 and toggles them by ACL rules. The engine tracks them by CIS rules
 * (radio_owns_seq = false), so before every event we write its values. UNVERIFIED (R1): that the hardware
 * does not overwrite the block after the recycle callback (a write between events persists). */
/* Sync with the hardware's link-state write, exactly as r_ble_lll_current_tx/rx_info_get (+0x0..0x1c): write 0xB1 to
 * 0x600A1B00, then wait for bit 6 of 0x600A1B04 to clear (two equal reads in a row). The controller does this before
 * reading ls+0/ls+20; it writes bits 26/27 itself between events without it (r_ble_lll_mmgmt_rxbuffer_*), so here this
 * guards the read-modify-write against a late hardware write. */
#define BB_LS_SYNC_REQ   ((volatile uint32_t *)0x600A1B00u)
#define BB_LS_SYNC_STAT  ((volatile uint32_t *)0x600A1B04u)
#define BB_LS_SYNC_CMD   0xB1u
#define BB_LS_SYNC_BUSY  0x40u

static void ls_sync(void)
{
	uint32_t a;
	uint32_t b;

	do {
		*BB_LS_SYNC_REQ = BB_LS_SYNC_CMD;
		do {
			a = *BB_LS_SYNC_STAT;
			b = *BB_LS_SYNC_STAT;
		} while (a != b);
	} while (a & BB_LS_SYNC_BUSY);
	/* The ls+20 read (memcpy, not volatile) must not move before the loop - the controller has a plain lw after it. */
	__atomic_signal_fence(__ATOMIC_SEQ_CST);
}

/* ls+20 bits written by the adapter: SEQ_WRITE 1 - SN (26) and NESN (27), 2 - SN only (experiment). Bit 27 is also
 * the state of the controller's current RX buffer (current_rx_info_get: ready = !bit27). Tab5 with Galaxy Buds3 Pro
 * 2026-10-31: with 1 CIS it sets up, but reception happens only every other event and with error 0x1 (19-21 % acked,
 * audio breaks up); with 2 (and 0) bit 27 stays 1 after the first reception, the RX buffer never goes back to hardware
 * and the CIS does not set up (0x3E). To do: RX buffer collection during the event as in stock (recycle_buffer).
 * Bit 26 is also the state of the current TX buffer (current_tx_info_get: ready/acked = !bit26, get_txed_buffer then
 * takes it as handed back). Black box 1.4.58 (Buds3 Pro): with SN 0 (bit 26 = 0) the earbuds do not respond, with
 * SN 1 they do - the hardware most likely does not send a buffer it considers acknowledged. SEQ_WRITE 3: bit 26 always
 * 1 (buffer waiting), SN only in the header byte (HDR_SEQ 1), bit 27 as in 1 - tells if hardware takes SN from it. */
static inline uint32_t seq_mask(void)
{
	const uint32_t mode = knob(CIS_C6_KNOB_SEQ_WRITE);
	return mode == 2 ? (1u << 26) : mode == 4 ? (7u << 25) : (3u << 26);
}

/* Returns ls+20 after the write (for the black box); with SEQ_WRITE = 0 read only - the hardware drives the bits.
 * The pre-write value stays in u->seq_pre: update_buffer_hdr restores controller state from it before collection. */
static uint32_t write_seq(struct cis_sub *u, void *ls, uint8_t hdr0)
{
	const uint32_t pol = knob(CIS_C6_KNOB_SEQ_POLARITY);
	uint32_t sn = knob(CIS_C6_KNOB_SEQ_WRITE) == 3 ? 1u : ((hdr0 & HDR_SN) ? 1u : 0u) ^ pol;
	uint32_t nesn = ((hdr0 & HDR_NESN) ? 1u : 0u) ^ pol;
	ls_sync();
	uint32_t w = ls_rd(ls, LS_SEQ);

	if (!knob(CIS_C6_KNOB_SEQ_WRITE)) {
		return w;
	}
	const uint32_t m = seq_mask();
	if (!u->seq_forced) {
		u->seq_pre = w;
	} else {
			/* Second write before collection (event did not go out, or failed sched_insert and repeated next_event): bits that
			 * the controller changed since our write (rxbuffer_allocate clears bit 27) are its state - into seq_pre. */
		const uint32_t diff = (w ^ u->seq_set) & u->seq_m;
		u->seq_pre = (u->seq_pre & ~diff) | (w & diff);
	}
	if (knob(CIS_C6_KNOB_SEQ_WRITE) == 4) {
		/* Per ble_hw_rst_soft_lc_wb: bit 25 = SN, 26 = current TX buffer waiting, 27 = current RX buffer (NESN in bit
		 * 24 is driven by the hardware). Black box 1.4.60: the hardware does not advance SN (it does so at the next
		 * descriptor, and our buffer is permanent), so the engine's SN per event + "frame waiting"; bit 27 = 0 as in
		 * mode 1 (engine NESN with BN P->C = 0) - 1 crashes the controller. */
		const uint32_t esn = (hdr0 & HDR_SN) ? 1u : 0u;
		w = (w & ~m) | (esn << 25) | (1u << 26);
	} else {
		w = (w & ~m) | (((sn << 26) | (nesn << 27)) & m);
	}
	ls_wr(ls, LS_SEQ, w);
	u->seq_set = w;
	u->seq_m = m;
	u->seq_forced = true;
	return w;
}

/* TX header byte 0; without HDR_SEQ only LLID and NPI (does the hardware take SN/NESN from the byte or ls+20 - D4). */
static inline uint8_t tx_hdr_byte(uint8_t hdr0)
{
	return (uint8_t)(hdr0 & (knob(CIS_C6_KNOB_HDR_SEQ) ? HDR_TX_MASK : (HDR_LLID | HDR_NPI)));
}

/* ---------- TX: one permanent buffer, rewritten each event (one PDU per event, hardware MD = 0, R3) ---------- */
/* After master_new the TX chain is a single 16 B dummy descriptor without a data area (+8 points at itself = empty
 * PDU): current (ls+0), head (ls+96) and tail (ls+104) - ble_lll_conn.c alloc_memory+0x118..0x142,
 * reset_link_state+0x32..0x50. The hardware sends the next buffer only after the current one is acknowledged, and CIS
 * (FT = 1) needs new content in every event - so the permanent current buffer is our own, of Max_PDU C->P capacity,
 * overwritten before every event. "Current = head = tail = one buffer with data" is the state plain ACL reaches after
 * the first acknowledged PDU; reset_link_state computes current from head, so it will not break it. The dummy returns
 * to the pool (free_tx_buffer_hdr: data == descriptor -> header only); lll_conn_end frees our buffer with the chain.
 * We do not call get_txed_buffer for CIS: it would turn the acknowledged buffer back into a dummy (get_txed_buffer
 * .L129) and hand its data to the pool. Every connsm (subevent) has its own chain and its own buffer. */
static int install_txb(struct cis_sub *u, uint8_t tx_cap)
{
	void *ls = link_state(u->sm);
	void *dummy = *(void **)f8(ls, LS_TX_HEAD);

	if (dummy == NULL || *(void **)f8(ls, LS_TX_TAIL) != dummy ||
	    (void *)HW_PTR(rd32(dummy, DESC_DATA)) != dummy) {
		return -1; /* chain not as after master_new in stock - we do not touch it */
	}
	void *b = r_ble_lll_mmgmt_alloc_tx_buffer_and_hdr(*(void **)f8(ls, LS_TX_POOL), tx_cap);
	if (b == NULL) {
		return -2;
	}
	uint8_t *d = HW_PTR(rd32(b, DESC_DATA));
	d[TXD_HDR] = HDR_NPI;
	d[TXD_LEN] = 0;
	ls_sync();
	*(void **)f8(ls, LS_TX_HEAD) = b;
	*(void **)f8(ls, LS_TX_TAIL) = b;
	u->ls0_hi = ls_rd(ls, LS_TX_CUR) & 0xFFF00000u;
	ls_wr(ls, LS_TX_CUR, u->ls0_hi | (((uint32_t)(uintptr_t)b + 4u) & 0xFFFFFu));
	r_ble_lll_mmgmt_free_tx_buffer_hdr(dummy);
	u->txb = b;
	u->txb_fresh = true;
	CIS_C6_CNT(tx_new_buf);
	return 0;
}

/* Experiment 1.4.60 (black box 1.4.58/1.4.59, Buds3 Pro): the earbud reply arrives only every other event, even when
 * our SN stays put - hypothesis: their null PDUs change SN every event (flush), and the hardware, as in ACL, accepts
 * only a PDU with SN equal to our NESN (forced to 0), the rest as a retransmission. Also, the earbuds accept our PDUs
 * in runs and reject them in runs (change after events that did not go out), i.e. the engine's SN drifts from what
 * they expect. NESN_TRACK / SN_TRACK set both bits from what the earbuds last sent. */
static uint8_t track_seq(const struct cis_link *l, uint32_t ev, uint8_t hdr0)
{
	const uint32_t nm = knob(CIS_C6_KNOB_NESN_TRACK), sm = knob(CIS_C6_KNOB_SN_TRACK);
	const bool fresh = l->rx_seen && ev > l->rx_last_ev;
	const uint32_t gap = fresh ? ev - l->rx_last_ev : 0u;

	if (nm == 1 || nm == 2) {
		const uint32_t nesn = (ev & 1u) ^ (nm == 2 ? 1u : 0u);
		hdr0 = (uint8_t)((hdr0 & ~HDR_NESN) | (nesn ? HDR_NESN : 0u));
	} else if (nm == 3 && fresh) {
		/* earbud SN in event ev = last received, toggled by the number of events since it */
		const uint32_t nesn = ((l->rx_last_hdr & HDR_SN) ? 1u : 0u) ^ (gap & 1u);
		hdr0 = (uint8_t)((hdr0 & ~HDR_NESN) | (nesn ? HDR_NESN : 0u));
	}
	if (sm == 1 && fresh) {
		/* ACL rule: we send the SN the earbuds expect (their NESN), without toggling our own */
		hdr0 = (uint8_t)((hdr0 & ~HDR_SN) | (l->rx_last_hdr & HDR_NESN ? HDR_SN : 0u));
	} else if (sm == 2 && fresh) {
		/* CIS rule with a flush every event: after acceptance they expect their NESN, after rejection NESN + 1 (flush at
		 * the end of the event); every further event without reception is another flush */
		const uint32_t exp1 = ((l->rx_last_hdr & HDR_NESN) ? 1u : 0u) ^ (l->rx_last_acc ? 0u : 1u);
		const uint32_t sn = exp1 ^ ((gap - 1u) & 1u);
		hdr0 = (uint8_t)((hdr0 & ~HDR_SN) | (sn ? HDR_SN : 0u));
	}
	return hdr0;
}

/* SEQ_WRITE 5 - model from the link-controller copy in libble_app.a (ble_single_conn.c):
 * ls+20 bit24 = SN, bit25 = NESN (write_back+0x9e..0xc8), bit26 = current TX buffer unacknowledged, bit27 = current RX
 * buffer free; ls+0 bit31 = chain mode. After the last buffer is acknowledged the hardware enters empty-PDU mode
 * (bit31 = 0), which ignores bit26 and ends only with the acknowledgement of an empty PDU - hence in earlier modes a
 * full frame only every other event. Before every event: our buffer as current in chain mode, "waiting", engine SN/NESN
 * (CIS flush: an unacknowledged payload is replaced by a new one with the next SN). Bit 27 and ls+8 are left alone and
 * nothing is written during the event (the hardware works on registers and writes them back to RAM at the end). The
 * header byte supplies only LLID (tx_pdu+0xa0..0xd4), so NPI cannot be sent: no SDU = LLID 01 length 0 (ISOAL padding).
 * NSE 2: every connsm has its own ls, so both subevents get the same PDU with the same SN/NESN (contract
 * subevents_prepared_ahead) - the acknowledgement from subevent 0 clears "waiting" only in the ls of connsm 0; connsm 1
 * sends the same PDU again, and the earbud takes it as a retransmission (old SN) and answers with the same NESN. */
static void tx_load_hw(struct cis_sub *u, void *ls, void *desc, const cis_pdu_t *pdu)
{
	const bool null_pdu = (pdu->hdr0 & HDR_NPI) != 0;
	uint8_t *d = HW_PTR(rd32(desc, DESC_DATA));

	d[TXD_HDR] = null_pdu ? 0x01u : (uint8_t)(pdu->hdr0 & HDR_LLID);
	d[TXD_LEN] = null_pdu ? 0u : pdu->len;
	if (!null_pdu) {
		memcpy(&d[TXD_PAYLOAD], pdu->payload, pdu->len);
	}
	const uint32_t sn = (pdu->hdr0 & HDR_SN) ? 1u : 0u;
	const uint32_t nesn = (pdu->hdr0 & HDR_NESN) ? 1u : 0u;
	ls_sync();
	ls_wr(ls, LS_TX_CUR, (ls_rd(ls, LS_TX_CUR) & 0x7FF00000u) | (((uint32_t)(uintptr_t)u->txb + 4u) & 0xFFFFFu) |
				     0x80000000u);
	const uint32_t w = (ls_rd(ls, LS_SEQ) & ~(7u << 24)) | (sn << 24) | (nesn << 25) | (1u << 26);
	ls_wr(ls, LS_SEQ, w);
	u->seq_forced = false; /* nothing to restore - hardware-format state, not a replacement of its bits */
	u->tx_sn = (uint8_t)sn;
	u->bb.seq_set = w;
	u->bb.flags |= CIS_C6_BB_SEQ_SET;
}

/* Engine PDU for subevent s of event 'ev' into its connsm's buffer + SN/NESN into ls+20. Without a connsm (n_sm < nse)
 * only prepare_tx - the contract requires it for every subevent, and the content would not go out anyway. */
static int tx_load(struct cis_link *l, uint8_t s, uint32_t ev)
{
	struct cis_sub *u = &l->se[s];
	cis_pdu_t pdu;
	void *desc = NULL;
	uint8_t seq;

	if (!l->cb->prepare_tx(l->cb_arg, ev, s, &pdu)) {
		/* Contract: false only on a closed link - we send nothing from new content, a null PDU with NPI. */
		pdu.hdr0 = HDR_NPI;
		pdu.len = 0;
	}
	if (s >= l->n_sm || u->sm == NULL) {
		return 0;
	}
	void *ls = link_state(u->sm);
	/* New black-box entry: update_buffer_hdr completes it after this event. */
	u->bb = (cis_c6_bb_t){ .ev = (uint16_t)ev, .link = (uint8_t)(l - s_links), .flags = bb_se_flag(s),
			       .ch = *f8(u->sm, SM_CHAN), .tx_hdr = pdu.hdr0, .rx_status = 0xFFFFFFFFu };
	if (pdu.len > l->tx_cap) {
		u->bb.flags |= CIS_C6_BB_TX_FAIL;
		return -1; /* engine error: longer than Max_PDU C->P with MIC */
	}
	r_ble_lll_current_tx_info_get(ls, &desc, &seq);
	if (desc == NULL && u->txb != NULL) {
		/* UNVERIFIED (D7): in case the hardware zeroes current after the last buffer is acknowledged - back to ours. */
		ls_sync();
		ls_wr(ls, LS_TX_CUR, u->ls0_hi | (((uint32_t)(uintptr_t)u->txb + 4u) & 0xFFFFFu));
		CIS_C6_CNT(tx_cur_restored);
		desc = u->txb;
	}
	if (u->txb == NULL || desc != u->txb) {
		/* Current buffer is not ours (controller put in its own PDU, or no txb yet) - we do not write to a foreign one. */
		u->bb.flags |= CIS_C6_BB_TX_FAIL;
		CIS_C6_CNT(tx_cur_foreign);
		return -1;
	}
	if (knob(CIS_C6_KNOB_SEQ_WRITE) == 5) {
		tx_load_hw(u, ls, desc, &pdu);
		if (u->txb_fresh) {
			u->bb.flags |= CIS_C6_BB_TX_NEW;
			u->txb_fresh = false;
		}
		CIS_C6_CNT(tx_rewrite);
		return 0;
	}
	pdu.hdr0 = track_seq(l, ev, pdu.hdr0);
	u->bb.tx_hdr = pdu.hdr0;
	u->bb.seq_set = write_seq(u, ls, pdu.hdr0);
	if (knob(CIS_C6_KNOB_SEQ_WRITE)) {
		u->bb.flags |= CIS_C6_BB_SEQ_SET;
	}
	if (u->txb_fresh) {
		u->bb.flags |= CIS_C6_BB_TX_NEW;
		u->txb_fresh = false;
	}
	/* Safe: the LL task is between events of this connsm (N:1.4). SN/NESN also in byte 0 (HDR_SEQ switch): if the
	 * hardware takes the byte raw, the engine's values are on air; if it inserts them from ls+20, we wrote the same there.
	 * UNVERIFIED which case applies (D4). */
	uint8_t *d = HW_PTR(rd32(desc, DESC_DATA));
	d[TXD_HDR] = tx_hdr_byte(pdu.hdr0);
	d[TXD_LEN] = pdu.len;
	memcpy(&d[TXD_PAYLOAD], pdu.payload, pdu.len);
	CIS_C6_CNT(tx_rewrite);
	return 0;
}

/* PDUs of event next_ev into the buffers of all subevents: with subevents_prepared_ahead the engine wants
 * prepare_tx(ev, 0) ... (ev, nse-1) before the first rx_done. Once per event (loaded_ev). Called after previous event
 * closes, in the next_event hook (NSE 2: usually the connsm of subevent 1, after the whole event) - connsm 0 is
 * then between events and its schedule element already inserted: reschedule_event changes in ls only ls+20 bits 8-19
 * (+0x1f6..0x1fc), not the ones we write, and the hardware reads ls only at event start. 0 = all loaded. */
static int load_event(struct cis_link *l)
{
	int rc = 0;
	l->loaded_ev = l->next_ev;
	for (uint8_t s = 0; s < l->nse; s++) {
		if (tx_load(l, s, l->next_ev) != 0) {
			CIS_C6_CNT(tx_load_fail);
			rc = -1;
		}
	}
	return rc;
}

/* Channel of subevent s of the CIS event with counter ctr (Core Vol 6 Part B 4.5.8.3: ISO event mapping, then s steps
 * of the subevent mapping). Subevent 0 = the event's CSA#2 channel, same formula as for a connection (prn_e from the
 * counter and channel identifier, remap on an unused channel) - the controller computes it in calc_dci from +102 and
 * +64, and the connsm counter = CIS event counter from first_anchor (NSE 1 on Tab5: 99.2 % acked on this channel).
 * The controller would compute subevent >= 1 channels like 0, so we overwrite them. */
static uint8_t sub_chan(const struct cis_link *l, uint8_t s, uint16_t ctr)
{
	cis_csa2_iso_t st;
	uint8_t ch = cis_csa2_iso_event(ctr, l->chan_id, l->chmap, &st);
	for (uint8_t i = 0; i < s; i++) {
		ch = cis_csa2_iso_subevent(l->chan_id, l->chmap, &st);
	}
	return ch;
}

/* ---------- hooks (objcopy: weakened original, alias orig_*); foreign connsm: original unchanged ---------- */

/* Called from r_ble_lll_conn_master_new+0x58 (same object, R_RISCV_CALL, redirection proven in Q8). */
uint32_t r_ble_lll_conn_calc_first_anchor_point(void *sm, void *rxhdr, void *pdu)
{
	uint8_t s = 0;
	struct cis_link *l = link_by_sm(sm, &s);

	if (l == NULL) {
		return orig_r_ble_lll_conn_calc_first_anchor_point(sm, rxhdr, pdu);
	}
	/* master_new stores this as the anchor (+156) and programs start = anchor - sched_env[0] (+0x60..0x8a). Subevent
	 * s starts s x Sub_Interval after the CIS event anchor (cis_link_params_t.sub_interval_us). */
	return l->first_anchor_us + (uint32_t)s * l->sub_interval_us;
}

/* get_rxed_buffer compares bit 27 with the RX descriptor markers; with our NESN instead of the controller state the
 * inconsistency ends in r_ll_assert (get_rxed_buffer .L184 -> .L189, status 0xFFFFFF). Before collection the controller
 * state returns: its pre-write value plus the toggles the hardware made in the event (after = ls+20 after ls_sync). */
static void seq_restore(struct cis_sub *u, void *ls, uint32_t after)
{
	if (!u->seq_forced) {
		return;
	}
	const uint32_t m = u->seq_m;
	const uint32_t toggled = (u->seq_set ^ after) & m;
	const uint32_t w = ls_rd(ls, LS_SEQ);
	ls_wr(ls, LS_SEQ, (w & ~m) | ((u->seq_pre ^ toggled) & m));
	u->seq_forced = false;
}

/* Last defence against r_ll_assert(699): get_rxed_buffer (.L217 -> .L213 -> .L189 -> .L194) takes the current RX buffer
 * as finished when bit 27 = 0 (current_rx_info_get flag), and then requires a written status (+12 != 0xFFFFFF,
 * +24 != 0xFFFF). Bit 27 can be changed after our write by writes no hook sees (rxbuffer_allocate of other links ->
 * update_rxbuf_insert_ptr -> cnt_change). An unwritten current buffer = nothing received: bit 27 = 1. */
static void rx_seq_guard(void *ls)
{
	void *desc = NULL;
	uint8_t done = 0;
	if (knob(CIS_C6_KNOB_SEQ_WRITE) != 1) {
		return; /* without our bit 27 write (modes 0 and 2) the controller drives it itself - consistent as in stock */
	}
	r_ble_lll_current_rx_info_get(ls, &desc, &done);
	if (desc == NULL || !done) {
		return;
	}
	const uint8_t *d = HW_PTR(rd32(desc, DESC_DATA));
	if ((const void *)d == desc) {
		return;
	}
	uint16_t w24;
	memcpy(&w24, d + 24, 2);
	if ((rd32((void *)d, RXD_STATUS) & 0xFFFFFFu) == RX_STATUS_EMPTY || w24 == 0xFFFFu) {
		ls_wr(ls, LS_SEQ, ls_rd(ls, LS_SEQ) | (1u << 27));
		CIS_C6_CNT(rx_seq_guard);
	}
}

/* "Broken" link (ext_ctr): RX buffers go back to the hardware without the engine. */
static void rx_recycle_only(void *sm, void *ls)
{
	void *desc;
	while ((desc = r_ble_lll_get_rxed_buffer(ls, NULL)) != NULL) {
		r_ble_lll_conn_align_pld_cnt_with_rxbuffer(sm, 1, desc);
		r_ble_lll_append_rx_buffer(ls, desc);
	}
}

/* After a connsm event without the engine (outside the LL task, link broken/dead, late result): controller state in
 * ls+20 and RX buffers back to the hardware, the stash from recycle_buffer discarded. */
static void rx_drop(struct cis_sub *u, void *sm, void *ls)
{
	u->rx_n = 0;
	ls_sync();
	seq_restore(u, ls, ls_rd(ls, LS_SEQ));
	rx_seq_guard(ls);
	rx_recycle_only(sm, ls);
}

/* Null PDU with NPI in our buffer (link "broken" or "dead" - without the engine). */
static void tx_null(struct cis_sub *u)
{
	void *desc = NULL;
	uint8_t seq;
	r_ble_lll_current_tx_info_get(link_state(u->sm), &desc, &seq);
	if (u->txb != NULL && desc == u->txb) {
		uint8_t *d = HW_PTR(rd32(desc, DESC_DATA));
		d[TXD_HDR] = HDR_NPI;
		d[TXD_LEN] = 0;
	}
}

/* Received PDU of subevent s to the engine and the black box. Raw PDU from the hardware buffer, before mbuf/HCI
 * (N:1.5, rx_process+0x28..0x1ae). */
static void rx_deliver(struct cis_link *l, uint8_t s, uint32_t status, uint32_t ts, uint8_t hdr, uint8_t len,
		       const uint8_t *payload, uint32_t ev, uint32_t anchor, cis_c6_bb_t *bb)
{
	cis_pdu_t pdu;
	const bool ok = (status & RX_STATUS_BAD_MASK) == 0 && len <= CIS_PDU_MAX_PAYLOAD;

	CIS_C6_CNT(rx_pdu);
	if (!ok) {
		CIS_C6_CNT(rx_bad);
	}
	pdu.hdr0 = hdr;
	pdu.len = ok ? len : 0;
	if (ok) {
		memcpy(pdu.payload, payload, len);
	}
	bb->rx_hdr = hdr;
	bb->rx_len = len;
	bb->rx_status = status;
	bb->flags = (uint8_t)((bb->flags & ~CIS_C6_BB_RX_OK) | (ok ? CIS_C6_BB_RX_OK : 0u));
	const int32_t off = (int32_t)(ts - anchor);
	bb->rx_off_us = (int16_t)(off > INT16_MAX ? INT16_MAX : off < INT16_MIN ? INT16_MIN : off);
	if (bb->rxed < UINT8_MAX) {
		bb->rxed++;
	}
	/* The header is valid also with status 0x1 (R2: NPI = CP in ACL breaks the CRC, not the header - black box). */
	l->rx_seen = true;
	l->rx_last_ev = ev;
	l->rx_last_hdr = hdr;
	l->rx_last_acc = ((hdr & HDR_NESN) != 0) != ((bb->tx_hdr & HDR_SN) != 0); /* NESN != our SN = accepted */
	l->cb->rx_done(l->cb_arg, ev, s, &pdu, ok, ts);
}

/* Packet start time from the RX buffer in the anchor clock. */
static uint32_t rx_ts(const uint8_t *d)
{
	const uint8_t mode = r_ble_phy_rate_to_mode((uint8_t)((rd32((void *)d, RXD_RATE) >> 24) & 3u));
	return r_ble_phy_get_actual_tx_time(rd32((void *)d, RXD_TS), mode);
}

/* Called from r_ble_lll_conn_recycle_sch_item+0x27c in the LL task after every event (N:Q4). */
int r_ble_lll_conn_update_buffer_hdr(void *sm, void *ls)
{
	uint8_t s = 0;
	struct cis_link *l = link_by_sm(sm, &s);
	void *desc;

	if (l == NULL) {
		if (*f8(sm, SM_ROLE) == 1) {
			__atomic_fetch_add(&acl_slot(sm)->aired, 1u, __ATOMIC_RELAXED);
		}
		return orig_r_ble_lll_conn_update_buffer_hdr(sm, ls);
	}
	struct cis_sub *u = &l->se[s];
	if (!cis_c6_in_ll_task()) {
		/* The original with our permanent TX buffer = r_ll_assert(492) (get_txed_buffer) - RX buffers only. */
		CIS_C6_CNT(hook_wrong_ctx);
		rx_drop(u, sm, ls);
		return 0;
	}
	/* Counter not advanced yet: next_event comes later (recycle+0x36c). */
	uint32_t ev = ext_ctr(l, u, rd16(sm, SM_EVENT_CTR));
	if (l->broken || l->dead) {
		rx_drop(u, sm, ls);
		if (l->dead) {
			return 0; /* the controller ended the link's other connsm - events go to the engine via cis_c6_radio_tick */
		}
		/* The engine keeps counting (skipped) events: its supervision (5 s without reception) closes the CIS via link_close,
		 * TX slots drain, so NOCP and host credits return. Supervision counts only at the end of an event (r_end_of_event).
		 * NSE 2: every connsm does this, so the engine counter runs twice as fast and supervision closes the CIS after
		 * about 2.5 s - the link is lost anyway and does not depend on whether both connsm still have events. */
		l->cb->event_skipped(l->cb_arg, l->next_ev++);
		cis_c6_kick(CIS_C6_KICK_PROCESS);
		return 0;
	}

	sub_skip_to(u, ev);
	close_ready(l);
	if (ev != l->next_ev) {
		/* NSE 2: the second connsm has not settled an earlier event yet (LL task late by a whole event; normally subevent 1
		 * ends the event about 8 ms before subevent 0 of the next). The engine gets events in order, and this event's PDU
		 * was not loaded yet (it is loaded after the previous one closes), so the result is dropped - on close the subevent
		 * counts as skipped. */
		CIS_C6_CNT(se_late);
		rx_drop(u, sm, ls);
		u->next = ev + 1u;
		cis_c6_kick(CIS_C6_KICK_PROCESS);
		return 0;
	}

	/* SN/NESN state after the event, before buffer collection changes anything: directly what the hardware did (R1). */
	cis_c6_bb_t *bb = &u->bb;
	if (bb->ev != (uint16_t)ev) {
		/* tx_load did not see this event (e.g. event 0 loaded before the hook) - entry from scratch */
		*bb = (cis_c6_bb_t){ .ev = (uint16_t)ev, .link = (uint8_t)(l - s_links), .flags = bb_se_flag(s),
				     .rx_status = 0xFFFFFFFFu };
	}
	if (l->loaded_ev != ev) {
		/* The event went out with the previous content (next_event outside the LL task or LL task late): prepare_tx for it
		 * after rx_done would violate the contract, and the content will not go out anyway. */
		l->loaded_ev = ev;
		bb->flags |= CIS_C6_BB_TX_FAIL;
	}
	ls_sync();
	bb->seq_after = ls_rd(ls, LS_SEQ);
	const uint32_t anchor = rd32(sm, SM_ANCHOR);

	seq_restore(u, ls, bb->seq_after);
	rx_seq_guard(ls);
	/* TX: txb is permanent (install_txb) - no get_txed_buffer. The engine reads acknowledgements from the Peripheral NESN,
	 * and builds NOCP for the host itself. */
	void *cur = NULL;
	uint8_t cseq;
	r_ble_lll_current_tx_info_get(ls, &cur, &cseq);
	bb->txed = cur == u->txb ? 1u : 0u; /* 0 = hardware or controller changed the current buffer (D7) */

	/* RX: first what was collected during the event (recycle_buffer), then the rest from the chain. */
	bool got = false;
	for (uint8_t i = 0; i < u->rx_n; i++) {
		const struct rx_raw *r = &u->rx_stash[i];
		rx_deliver(l, s, r->status, r->ts, r->hdr, r->len, r->payload, ev, anchor, bb);
		got = true;
	}
	u->rx_n = 0;
	const bool hw = knob(CIS_C6_KNOB_SEQ_WRITE) == 5;
	while ((desc = r_ble_lll_get_rxed_buffer(ls, NULL)) != NULL) {
		const uint8_t *d = HW_PTR(rd32(desc, DESC_DATA));
		const uint32_t status = rd32((void *)d, RXD_STATUS) & 0xFFFFFFu;

		/* Mode 5: a bad status (0x1 = no reception, with a header from an earlier packet, NOTES section 9) is not a
		 * reply - the engine would take its NESN as an acknowledgement or a refusal. */
		if (status != RX_STATUS_EMPTY && (!hw || (status & RX_STATUS_BAD_MASK) == 0)) {
			rx_deliver(l, s, status, rx_ts(d), d[RXD_HDR], d[RXD_LEN], &d[RXD_PAYLOAD], ev, anchor, bb);
			got = true;
		}
		/* The buffer returns to the hardware RX chain exactly as in stock (update_buffer_hdr+0xc8..0xd8). */
		r_ble_lll_conn_align_pld_cnt_with_rxbuffer(sm, 1, desc);
		r_ble_lll_append_rx_buffer(ls, desc);
	}
	if (hw && !got && ((bb->seq_after >> 26) & 1u) == 0) {
		/* The hardware cleared "TX waiting" = got a valid packet from the earbud with NESN != our SN. Their null PDUs (length
		 * 0) do not occupy an RX buffer, so the engine gets this acknowledgement as a null PDU (time = anchor). */
		static const uint8_t none[1];
		const uint8_t hdr = (uint8_t)(0x01u | HDR_NPI | (u->tx_sn ? 0u : HDR_NESN));
		rx_deliver(l, s, 0, anchor, hdr, 0, none, ev, anchor, bb);
		got = true;
	}
	if (!got) {
		l->cb->rx_done(l->cb_arg, ev, s, NULL, false, 0);
	}
	/* This subevent's channel is still in the connsm (next_event sets the next one after settling) */
	const uint8_t ch = *f8(sm, SM_CHAN);
	if (ch < CIS_C6_CHANNELS) {
		__atomic_fetch_add(&s_ch_tries[ch], 1u, __ATOMIC_RELAXED);
		if (!got || (bb->flags & CIS_C6_BB_RX_OK) == 0) {
			__atomic_fetch_add(&s_ch_bad[ch], 1u, __ATOMIC_RELAXED);
		}
	}
	l->air |= (uint8_t)(1u << s);
	bb_commit(bb);
	/* priority_reset would restore 8 (N:Q6). */
	if (knob(CIS_C6_KNOB_SCHED_PRIO) != 0) {
		*f8(ls, LS_PRIO) = (uint8_t)knob(CIS_C6_KNOB_SCHED_PRIO);
	}
	/* NSE 1: event_closed here. NSE 2: after subevent 1 (or when subevent 1 is already known to be skipped). */
	u->next = ev + 1u;
	close_ready(l);
	/* Decryption and HCI events after leaving the recycle, not in it (contract: radio callbacks are short). */
	cis_c6_kick(CIS_C6_KICK_PROCESS);
	/* 0 = the original's normal path (update_buffer_hdr .L887: li a0,0): recycle goes on to check the MIC and schedule the
	 * anchor. Non-zero ends recycle at once (recycle_sch_item+0x286 bnez -> .L926), with no next event. */
	return 0;
}

/* After a foreign connsm's next_event the counter and anchor describe the planned event: the LL_CIS_IND trace point. */
static void watch_acl(void *sm)
{
	if (rd16(sm, SM_HANDLE) != s_watch.acl || !cis_c6_in_ll_task()) {
		return;
	}
	const uint16_t ctr = rd16(sm, SM_EVENT_CTR);
	const int16_t d = (int16_t)(uint16_t)(ctr - s_watch.ev);
	if (d < 0) {
		return;
	}
	const cis_trace_t t = { .kind = CIS_TR_ACL_AT_INSTANT, .code = d > 0 ? 1u : 0u, .cis = s_watch.cis,
				.acl = s_watch.acl, .v = { ctr, rd32(sm, SM_ANCHOR), 0, 0 } };
	s_watch.armed = false;
	tr_put(&t);
}

/* Called from r_ble_lll_conn_sched_next_anchor (LL task): counter +1, anchor += interval, channel from CSA#2. After it
 * reschedule_event takes the channel from +67 into the schedule element (+0x136 -> phy_chan_to_freq). */
int r_ble_ll_conn_next_event(void *sm)
{
	uint8_t s = 0;
	struct cis_link *l = link_by_sm(sm, &s);

	if (l == NULL) {
		if (*f8(sm, SM_ROLE) == 1) {
			acl_planned(sm);
		}
		const int rc = orig_r_ble_ll_conn_next_event(sm);
		if (s_watch.armed && rc == 0) {
			watch_acl(sm);
		}
		return rc;
	}
	int rc = orig_r_ble_ll_conn_next_event(sm);
	if (rc != 0) {
		return rc; /* -1: the event cannot be set (next_event+0x514); lll reschedules or ends */
	}
	if (s != 0) {
		/* Without the engine and in any context: the original wrote the event's CSA#2 channel (= subevent 0). Only set_csa,
		 * next_event and init_rx_pkt_in (Peripheral) write +67 - so here and in link_open is enough. */
		*f8(sm, SM_CHAN) = sub_chan(l, s, rd16(sm, SM_EVENT_CTR));
	}
	if (!cis_c6_in_ll_task()) {
		/* No engine calls outside the LL task: the buffer keeps the old content (the engine empties it after FT). */
		CIS_C6_CNT(hook_wrong_ctx);
		return 0;
	}
	struct cis_sub *u = &l->se[s];
	uint32_t ev = ext_ctr(l, u, rd16(sm, SM_EVENT_CTR));
	if (l->broken || l->dead) {
		tx_null(u);
		return 0;
	}

	/* A call without the previous one finishing = the previous event of this connsm did not enter the schedule. */
	sub_skip_to(u, ev);
	close_ready(l);
	if (knob(CIS_C6_KNOB_SCHED_PRIO) != 0) {
		*f8(link_state(sm), LS_PRIO) = (uint8_t)knob(CIS_C6_KNOB_SCHED_PRIO);
	}
	/* NSE 1: always (the previous event has just closed). NSE 2: in the hook of the connsm that closed the event - usually
	 * subevent 1; next_event of connsm 0 after its subevent loads nothing (the event is still open). In link_open
	 * link_open loads itself (TX buffers not set up yet). */
	if (!l->opening && l->loaded_ev != l->next_ev) {
		(void)load_event(l); /* errors in tx_load_fail; old content goes out, the engine empties it after FT */
	}
	return 0;
}

/* After the SN/NESN write (next_event), scheduling the event may itself still write bits 26/27: with
 * CONFIG_BT_LE_RXBUF_OPT_ENABLED sched_insert (reschedule_event+0x228) -> insert_forcibly -> rxbuffer_allocate clears
 * bit 27 (rxbuffer_free_all, rxbuffer_cnt_change). Without it the XOR in seq_restore would take the controller's write
 * for a hardware toggle. Bits changed by the controller go to seq_pre (its state), and our write returns to ls+20.
 * Called from the coex_dpc_process hook - reschedule_event+0x278, after sched_insert (bt_rf_coex_cfg_p always sets it
 * in r_ble_ll_init+0x202). */
static void seq_resync(struct cis_sub *u, void *ls)
{
	if (!u->seq_forced) {
		return;
	}
	const uint32_t m = u->seq_m;
	ls_sync();
	const uint32_t cur = ls_rd(ls, LS_SEQ);
	const uint32_t diff = (cur ^ u->seq_set) & m;
	if (diff == 0) {
		return;
	}
	u->seq_pre = (u->seq_pre & ~diff) | (cur & diff);
	ls_wr(ls, LS_SEQ, (cur & ~m) | (u->seq_set & m));
	CIS_C6_CNT(seq_resync);
}

/* Called when planning every connection event (reschedule_event+0x278, after sched_insert, so a write
 * in next_event would be overwritten): ls+40[3:0] = conn_dynamic_pti_param_rom[role * 2 + connsm level+485]
 * (4 normal, 9 urgent). For CIS the PTI comes from the knobs; 0 = controller value. */
void r_ble_lll_conn_coex_dpc_process(void *sm, int role_idx)
{
	uint8_t s = 0;
	struct cis_link *l = link_by_sm(sm, &s);
	const uint32_t pti = knob(CIS_C6_KNOB_COEX_PTI);
	const uint32_t hi = knob(CIS_C6_KNOB_COEX_PTI_HI);

	if (l == NULL || pti == 0) {
		orig_r_ble_lll_conn_coex_dpc_process(sm, role_idx);
		if (l == NULL || hi == 0) {
			if (l != NULL) {
				seq_resync(&l->se[s], link_state(sm));
			}
			return;
		}
	}
	void *ls = link_state(sm);
	uint32_t w = ls_rd(ls, LS_COEX);
	if (pti != 0) {
		w = (w & ~0xFu) | pti;
	}
	if (hi != 0) {
		w = (w & ~0xF0u) | (hi << 4);
	}
	ls_wr(ls, LS_COEX, w);
	seq_resync(&l->se[s], ls);
}

/* Buffer collection during the event: radio interrupt (bits 27/28 of 0x600A130C, phy_isr) -> LL task ->
 * recycle_buffer_in_task+0x30..0x40 -> callback of the schedule element sch+68 (for connections this symbol, set by
 * address in alloc_memory+0xc6) with (sch, connsm number sch+32, current hardware RX buffer a2, current TX a3); the
 * result is ignored. For CIS without the TX part: get_txed_buffer (+0xe6) with our permanent buffer and an empty slot
 * sm+256 ends in r_ll_assert(492) (get_txed_buffer .L129 -> .L148). The RX part as in stock (+0x11e..0x16a):
 * get_rxed_buffer up to the current hardware buffer (readiness from descriptor bit 0, not bit 27 of ls+20), the buffer
 * returns to the hardware at once - without it the hardware kept the RX buffer busy and received only every other event
 * (Galaxy Buds3 Pro 2026-10-31, 19-21 % acked). Instead of rx_process, a PDU copy for the engine, handed over in
 * update_buffer_hdr of that event. (The 32 s connsm supervision is armed anyway by recycle_sch_item .L1029 on every
 * reception - the engine closes the CIS earlier, after 5 s; failing that, the hook below catches conn_end.) */
void r_ble_lll_conn_recycle_buffer(void *sch, uint8_t sm_num, void *a2, void *a3)
{
	uint16_t base;
	memcpy(&base, r_sdkconfig_get_opts() + 66, 2);
	void *sm = r_ble_ll_conn_find_active_conn((uint16_t)(sm_num - 1u - base));
	uint8_t s = 0;
	struct cis_link *l = link_by_sm(sm, &s);
	if (l == NULL) {
		orig_r_ble_lll_conn_recycle_buffer(sch, sm_num, a2, a3);
		return;
	}
	CIS_C6_CNT(recycle_skipped);
	if (a2 == NULL || !cis_c6_in_ll_task()) {
		return; /* stock would stop at r_ll_assert(0xc20) - without a reference buffer we collect nothing */
	}
	struct cis_sub *u = &l->se[s];
	void *ls = link_state(sm);
	void *desc;
	while ((desc = r_ble_lll_get_rxed_buffer(ls, a2)) != NULL) {
		const uint8_t *d = HW_PTR(rd32(desc, DESC_DATA));
		const uint32_t status = rd32((void *)d, RXD_STATUS) & 0xFFFFFFu;
		if (status != RX_STATUS_EMPTY && !l->broken && !l->dead) {
			if (u->rx_n < CIS_C6_RX_STASH) {
				struct rx_raw *r = &u->rx_stash[u->rx_n++];
				r->status = status;
				r->ts = rx_ts(d);
				r->hdr = d[RXD_HDR];
				r->len = d[RXD_LEN] <= CIS_PDU_MAX_PAYLOAD ? d[RXD_LEN] : CIS_PDU_MAX_PAYLOAD;
				memcpy(r->payload, &d[RXD_PAYLOAD], r->len);
				CIS_C6_CNT(rx_in_event);
			} else {
				CIS_C6_CNT(rx_stash_full);
			}
		}
		r_ble_lll_conn_align_pld_cnt_with_rxbuffer(sm, 1, desc);
		r_ble_lll_append_rx_buffer(ls, desc);
	}
}

/* The controller itself ends a CIS connsm: connsm supervision (superversion_timer_cb+0x54, reason 8), failed
 * next_event (sched_next_anchor+0x10c, reason 22), master_new+0x1a4. The connsm returns to the pool and the slot would
 * hold a dangling pointer - a later link_close would end it a second time (the active-list loop in conn_end+0xf2..0x110
 * runs past the end). Our own link_close zeroes se[].sm before the call, so it does not see this. NSE 2: the other
 * connsm runs on to link_close without the engine (null PDUs, RX buffers back) - engine closes the CIS as in NSE 1. */
void *r_ble_lll_conn_end(void *sm, uint8_t reason)
{
	uint8_t s = 0;
	struct cis_link *l = link_by_sm(sm, &s);

	if (l != NULL) {
		CIS_C6_CNT(ctrl_conn_end);
		l->se[s].sm = NULL;
			l->se[s].txb = NULL; /* this conn_end frees it together with the chain */
		if (!l->opening) {
			l->dead = true; /* the engine still thinks the link is alive - cis_c6_radio_tick brings it to a close */
		}
		/* Reason 0 as in link_close: no Disconnection Complete for the pseudo-connection handle (conn_end+0x176..0x19a),
		 * which the host does not know - the engine reports the end of the CIS. */
		reason = 0;
	}
	return orig_r_ble_lll_conn_end(sm, reason);
}

/* From the npl event handling (LL task, cis_c6_core.c): a link ended by the controller has no more events, and the
 * engine's supervision counts only at the end of an event - so we feed it skipped events (up to 64 per call, 500 = 5 s)
 * until it closes the CIS via link_close. */
static void radio_test_finish(void);

void cis_c6_radio_tick(void)
{
	radio_test_finish();
	bool more = false;
	for (int i = 0; i < CIS_C6_MAX_LINKS; i++) {
		struct cis_link *l = &s_links[i];
		for (int k = 0; l->dead && k < 64; k++) {
			l->cb->event_skipped(l->cb_arg, l->next_ev++);
		}
		more = more || l->dead;
	}
	if (more) {
		/* Next batch without waiting for other events (the host may send nothing more); the engine closes the CIS while
		 * handling the same npl event, so this is at most a few turns (500 events = 8 batches). */
		cis_c6_kick(CIS_C6_KICK_PROCESS);
	}
}

/* ---------- radio test without a Peripheral (0xFC30 subcommands 7/8) ---------- */
/* A CIS pseudo-connection as at setup, but without the engine and earbuds: a PDU of the given length every ISO, nobody
 * answers. Measures what is possible without the other side: regularity of scheduled events (anchor against anchor0 +
 * k x ISO), events skipped by the scheduler (Wi-Fi coexistence, ACL), stability of the TX path and buffer collection,
 * and whether the hardware moves the written SN/NESN bits with no acknowledgement (black box: seq_set / seq_after). It
 * cannot tell if earbuds would receive (window timing, channel, AA, R1 with ACK) - only a session or a sniffer does. */
#define RT_ISO_US 10000u

_Static_assert(sizeof(cis_c6_rt_t) == 36, "layout = format 0xFC30/0x08");
static cis_c6_rt_t s_rt;
static cis_link_t *s_rt_link;

static void rt_rnd(void *ctx, uint8_t *buf, uint16_t len)
{
	(void)ctx;
	esp_fill_random(buf, len);
}

static bool rt_prepare(void *arg, uint32_t ev, uint8_t sub, cis_pdu_t *out)
{
	(void)arg;
	(void)sub;
		/* As CIS with NSE = BN = FT = 1: SN and NESN toggle every event; LLID 0b00 (full SDU), no NPI. */
	out->hdr0 = (uint8_t)((ev & 1u) ? (HDR_SN | HDR_NESN) : 0u);
	out->len = s_rt.pdu_len;
	for (uint8_t i = 0; i < out->len; i++) {
		out->payload[i] = (uint8_t)(ev + i);
	}
	return true;
}

static void rt_rx(void *arg, uint32_t ev, uint8_t sub, const cis_pdu_t *pdu, bool crc_ok, uint32_t ts)
{
	(void)arg;
	(void)ev;
	(void)sub;
	(void)crc_ok;
	(void)ts;
	if (pdu != NULL) {
		s_rt.rx_any++;
	}
}

static void rt_count(void)
{
	if (s_rt.state == CIS_C6_RT_RUNNING && s_rt.closed + s_rt.skipped >= s_rt.target) {
		cis_c6_kick(CIS_C6_KICK_PROCESS); /* close in the npl event handling, not in the hook */
	}
}

static void rt_closed(void *arg, uint32_t ev)
{
	(void)arg;
	const struct cis_link *l = (const struct cis_link *)s_rt_link;
	const struct cis_sub *u = l != NULL ? &l->se[0] : NULL; /* radio test: always NSE 1 */
	if (u != NULL && u->sm != NULL) {
		/* update_buffer_hdr: counter and anchor still from the event that has just ended (recycle+0x36c) */
		const int32_t dev = (int32_t)(rd32(u->sm, SM_ANCHOR) - (s_rt.anchor0 + ev * RT_ISO_US));
		s_rt.dev_min = dev < s_rt.dev_min ? dev : s_rt.dev_min;
		s_rt.dev_max = dev > s_rt.dev_max ? dev : s_rt.dev_max;
		if ((u->bb.flags & CIS_C6_BB_SEQ_SET) && ((u->bb.seq_after ^ u->bb.seq_set) & u->seq_m) != 0) {
			s_rt.seq_moved++;
		}
	}
	s_rt.closed++;
	rt_count();
}

static void rt_skipped(void *arg, uint32_t ev)
{
	(void)arg;
	(void)ev;
	s_rt.skipped++;
	rt_count();
}

static const cis_link_cb_t k_rt_cb = { rt_prepare, rt_rx, rt_closed, rt_skipped };

int cis_c6_radio_test_start(uint16_t events, uint8_t pdu_len, uint8_t phy)
{
	if (s_rt.state == CIS_C6_RT_RUNNING) {
		return -1;
	}
	for (int i = 0; i < CIS_C6_MAX_LINKS; i++) {
		if (link_has_sm(&s_links[i]) || s_links[i].dead) {
			return -2; /* the engine's CIS or the previous test is still open */
		}
	}
	if (events < 10 || events > 6000 || pdu_len > CIS_PDU_MAX_PAYLOAD || (phy != CIS_PHY_1M && phy != CIS_PHY_2M)) {
		return -3;
	}
	memset(&s_rt, 0, sizeof(s_rt));
	s_rt.target = events;
	s_rt.pdu_len = pdu_len;
	s_rt.dev_min = INT32_MAX;
	s_rt.dev_max = INT32_MIN;

	cis_link_params_t p;
	memset(&p, 0, sizeof(p));
	p.access_address = cis_aa_generate(rt_rnd, NULL, NULL, 0);
	uint8_t c[3];
	esp_fill_random(c, sizeof(c));
	p.crc_init = (uint32_t)c[0] | ((uint32_t)c[1] << 8) | ((uint32_t)c[2] << 16);
	p.phy_c_to_p = phy;
	p.phy_p_to_c = phy;
	memset(p.chan_map, 0xFF, 4);
	p.chan_map[4] = 0x1F;
	p.nse = 1;
	p.iso_interval_us = RT_ISO_US;
	p.first_anchor_us = cis_c6_now_us(NULL) + 20000u;
	p.max_pdu_c_to_p = pdu_len;
	p.max_pdu_p_to_c = pdu_len;
	s_rt.anchor0 = p.first_anchor_us;
	s_rt.state = CIS_C6_RT_RUNNING;
	const int rc = cis_c6_link_open(NULL, &p, &k_rt_cb, NULL, &s_rt_link);
	/* In the setup trace like the engine's link_open (without handles): without earbuds this is the only way to check the
	 * trace on the Tab5 from the C6 side via le_audio_lab. */
	const cis_trace_t t = { .kind = CIS_TR_OPEN, .code = rc != 0 ? 1u : 0u, .v = { p.first_anchor_us, 0, 0, 0 } };
	cis_c6_trace(NULL, &t);
	if (rc != 0) {
		s_rt.state = CIS_C6_RT_OPEN_FAIL;
		s_rt.open_rc = (int8_t)rc;
		s_rt_link = NULL;
	}
	return rc;
}

static void radio_test_finish(void)
{
	if (s_rt.state != CIS_C6_RT_RUNNING || s_rt.closed + s_rt.skipped < s_rt.target || s_rt_link == NULL) {
		return;
	}
	cis_c6_link_close(NULL, s_rt_link);
	s_rt_link = NULL;
	s_rt.state = CIS_C6_RT_DONE;
}

void cis_c6_radio_test_get(cis_c6_rt_t *out)
{
	*out = s_rt;
}

/* ---------- contract operations ---------- */

/* connsm of subevent s: a Central pseudo-connection with the CIS parameters, anchor first_anchor + s x Sub_Interval.
 * Each connsm has its own link-state block; AA, CRC init, interval, channel map, PHY and DLE are the same (one CIS).
 * 0 or the link_open error code (-4 no connsm in the pool, -5 master_new). */
static int sub_open(struct cis_link *l, uint8_t s, const cis_link_params_t *p)
{
	int err = 0;
	void *sm = r_ble_ll_conn_sm_get(&err);
	if (sm == NULL) {
		/* R5: the connsm pool = CONFIG_BT_LE_MAX_CONNECTIONS must hold the ACLs + the CIS subevents. */
		return -4;
	}
	/* Also puts the connsm on the active list (recycle_sch_item+0x34 looks it up there). A second sm_active_insert of the
	 * same connsm makes a loop of it (sm+244 = sm): num_comp_pkts_event_send spun forever after the first event - the C6
	 * hang in 1.4.52 in the radio test, watchdog trace 0xFC30/0x09 in 1.4.53. */
	r_ble_ll_conn_sm_new(sm);

	/* What master_common_init and conn_created write (N:Q1), with CIS values. UNVERIFIED (R6): that nothing more
	 * is needed by the event path. */
	*f8(sm, SM_ROLE) = 1;
	wr32(sm, SM_AA, p->access_address);
	wr32(sm, SM_CRC_INIT, p->crc_init & 0xFFFFFFu);
	wr16(sm, SM_ITVL, (uint16_t)(p->iso_interval_us / 1250u));
	wr16(sm, SM_LATENCY, 0);
	wr16(sm, SM_SPVN_TMO, 3200); /* 32 s: high limit +534; the supervision callout is not armed anyway (N:Q5) */
	wr16(sm, SM_TX_WIN_OFF, 0);
	memcpy(f8(sm, SM_CHMAP), p->chan_map, 5);
	uint8_t used = 0;
	for (int i = 0; i < 37; i++) {
		used = (uint8_t)(used + ((p->chan_map[i >> 3] >> (i & 7)) & 1u));
	}
	*f8(sm, SM_CHANS_USED) = used;
	memset(f8(sm, SM_PEER_ADDR), 0, 6); /* set_connected_dev(...,0) at the end leaves the white list alone (N:Q5) */
	*f8(sm, SM_PEER_ADDR_TYPE) = 0;
	wr16(sm, SM_EVENT_CTR, 0);
	wr16(sm, SM_LAST_CTR, 0);
	/* PHY mode = PHY number for 1M/2M: r_ble_ll_phy_to_phy_mode returns phy unchanged except coded (ble_phy.c +0x0). */
	uint32_t mode = p->phy_c_to_p == CIS_PHY_2M ? 2u : 1u;
	wr32(sm, SM_PHY, (rd32(sm, SM_PHY) & ~0xFu) | mode | (mode << 2));
	*f8(sm, SM_ENC_TX) &= (uint8_t)~1u; /* no hardware CCM: we send ciphertext + our MIC */
	/* R3/R6: the longest event in hardware (ls+44) is computed by r_ble_lll_conn_reschedule_event (bit test +0xea..0xee,
	 * write +0x11c or +0x284..0x2ac): without bit 7 of +566 it is interval - 2000 us (8 ms at ISO 10 ms - with CIE = 1
	 * from the earbuds the hardware would keep exchanging as with MD), with bit 7 - one exchange per the effective DLE
	 * (eff_max_tx_rx_time_get: PDU time from +20/+21 (+4 B) capped by +30/+32) + 2150 us. The times are the exact CIS PDU
	 * time with MIC, so the cap eats those +4 B. Bit 7 is also set by chk_wthr_adj_max_evt_time, and cleared only by
	 * adj_max_evt_time_flag_clr together with bit 8 or 9 (we set neither).
	 * Event 0 still has the limit from master_new (interval - 2000 us).
	 * The EVT_LIMIT knob = 0 keeps the controller limit (comparison on Tab5). */
	*f8(sm, SM_EFF_OCTETS) = p->max_pdu_c_to_p;
	*f8(sm, SM_EFF_OCTETS + 1) = p->max_pdu_p_to_c;
	wr16(sm, SM_EFF_TIME, (uint16_t)r_ble_ll_pdu_tx_time_get(p->max_pdu_c_to_p, (int)mode));
	wr16(sm, SM_EFF_TIME + 2, (uint16_t)r_ble_ll_pdu_tx_time_get(p->max_pdu_p_to_c, (int)mode));
	uint16_t lll = (uint16_t)(rd16(sm, SM_LLL_FLAGS) & ~((1u << 6) | (1u << 7)));
	if (knob(CIS_C6_KNOB_EVT_LIMIT)) {
		lll |= 1u << 7;
	}
	wr16(sm, SM_LLL_FLAGS, lll);
	wr32(sm, SM_FLAGS, rd32(sm, SM_FLAGS) & ~(3u << 28)); /* no Read Remote Features Complete at the end */
	*f8(sm, SM_STATE) = 1;
	wr32(sm, SM_CREATE_TICK, r_ble_lll_timer_current_tick_get());
	wr32(sm, SM_MISSED, 0x60000u);
	*f8(sm, SM_LLL_FLAGS2) = 0;
	r_ble_ll_conn_calc_itvl_ticks(sm);
	r_ble_ll_conn_set_csa(sm, 1);
	if (s != 0) {
		/* set_csa computed +67 for counter 0 as for subevent 0, and master_new+0x8c uses it for the first schedule element -
		 * the subevent channel must be written before master_new (later the next_event hook does it). */
		*f8(sm, SM_CHAN) = sub_chan(l, s, 0);
	}
	l->se[s].sm = sm; /* from now on the hooks treat this connsm as CIS */

	/* rxhdr/pdu (s4/s5) go only to calc_first_anchor_point (master_new+0x4e..0x58, our hook ignores them);
	 * right after, master_new overwrites both registers (+0x64, +0x68). */
	const int mn = r_ble_lll_conn_master_new(sm, NULL, NULL);
	if (mn != 0 || l->se[s].sm == NULL) {
		/* master_new may end the connsm itself (+0x1a4 -> conn_end, the hook zeroes se[s].sm) - not a second time. */
		const bool ended = l->se[s].sm == NULL;
		l->se[s].sm = NULL;
		if (!ended) {
			(void)r_ble_lll_conn_end(sm, 0);
		}
		return -5;
	}
	if (knob(CIS_C6_KNOB_SCHED_PRIO) != 0) {
		*f8(link_state(sm), LS_PRIO) = (uint8_t)knob(CIS_C6_KNOB_SCHED_PRIO);
	}
	return 0;
}

/* Ends all connsm of the link without Disconnection Complete. se[].sm are zeroed before the call: the conn_end hook
 * does not see them (does not mark the link "dead"). */
static void subs_end(struct cis_link *l)
{
	for (uint8_t s = 0; s < CIS_C6_MAX_NSE; s++) {
		void *sm = l->se[s].sm;
		if (sm == NULL) {
			continue;
		}
		l->se[s].sm = NULL;
			l->se[s].txb = NULL; /* lll_conn_end frees it together with the TX chain (head ls+96) */
		/* Reason 0: removes schedule elements, frees link state and buffers, returns the connsm to the pool, without
		 * Disconnection Complete (lll_conn_end+0x56..0x248). It unlinks from the active list itself (+0xf2..0x110: head
		 * ble_ll_conn_env_p+32, field +244), before sm_retrieve uses +244 as a free-list pointer. The connsm must be on the
		 * active list (the +0x108 loop does not know the end of the list) - sm_new inserts it there in sub_open. */
		(void)r_ble_lll_conn_end(sm, 0);
	}
}

int cis_c6_link_open(void *ctx, const cis_link_params_t *p, const cis_link_cb_t *cb, void *cb_arg,
		     cis_link_t **out)
{
	struct cis_link *l = NULL;

	if (!cis_c6_in_ll_task()) {
		CIS_C6_CNT(link_open_fail);
		return -1;
	}
	/* NSE 1 or 2 (subevent = connsm, because the sequencer does one exchange per event), interval
	 * in units of 1.25 ms (+140), one PHY both ways, subevents before the next CIS event. */
	if (p->nse < 1 || p->nse > CIS_C6_MAX_NSE || p->iso_interval_us % 1250u != 0 || p->phy_c_to_p != p->phy_p_to_c ||
	    p->max_pdu_c_to_p > CIS_PDU_MAX_PAYLOAD || p->max_pdu_p_to_c > CIS_PDU_MAX_PAYLOAD ||
	    (p->nse > 1 && (p->sub_interval_us == 0 || (uint64_t)p->sub_interval_us * p->nse > p->iso_interval_us))) {
		CIS_C6_CNT(link_open_fail);
		return -1;
	}
	if ((int32_t)(p->first_anchor_us - r_ble_lll_timer_current_tick_get()) < (int32_t)knob(CIS_C6_KNOB_MIN_LEAD_US)) {
		CIS_C6_CNT(link_open_fail);
		return -2;
	}
	for (int i = 0; i < CIS_C6_MAX_LINKS; i++) {
		if (!link_has_sm(&s_links[i]) && !s_links[i].dead) {
			l = &s_links[i];
			break;
		}
	}
	if (l == NULL) {
		CIS_C6_CNT(link_open_fail);
		return -3;
	}

	/* Link state before the first master_new: hooks called in it (next_event, when the schedule slot is taken) already
	 * settle events. */
	memset(l->se, 0, sizeof(l->se));
	for (uint8_t s = 0; s < CIS_C6_MAX_NSE; s++) {
		l->se[s].bb.ev = 0xFFFFu;
	}
	l->nse = p->nse;
	l->n_sm = p->nse;
	l->air = 0;
	l->cb = cb;
	l->cb_arg = cb_arg;
	l->first_anchor_us = p->first_anchor_us;
	l->sub_interval_us = p->sub_interval_us;
	l->tx_cap = p->max_pdu_c_to_p;
	memcpy(l->chmap, p->chan_map, 5);
	l->chan_id = cis_csa2_chan_id(p->access_address);
	l->next_ev = 0;
	l->loaded_ev = UINT32_MAX;
	l->rx_seen = false;
	l->broken = false;
	l->dead = false;
	s_bb_first_n = 0; /* "first events" always from the most recently opened link */

	l->opening = true;
	const int rc = sub_open(l, 0, p);
	if (rc == 0 && p->nse > 1 && sub_open(l, 1, p) != 0) {
		/* The second connsm did not open (connsm pool, allocation in master_new). The link stays with subevent 1 always
		 * skipped: the engine gets prepare_tx and rx_done without a PDU for it as for any skip by the scheduler (contract
		 * subevents_prepared_ahead), on the air it is NSE 1 as in 1.4.73. Better than refusing the CIS (LIMITED_RESOURCES =
		 * earbud without sound), and the max_nse knob does not know the pool state at Set CIG time. */
		CIS_C6_CNT(se_open_fail);
		l->n_sm = 1;
	}
	l->opening = false;
	if (rc != 0) {
		CIS_C6_CNT(link_open_fail);
		return rc;
	}
	/* Events settled by the connsm in master_new close only now, when n_sm is certain. */
	close_ready(l);
	/* Event 0 was inserted by master_new (or by next_event when the slot was taken - then the hook did not load, as there
	 * was no buffer). Now our own TX buffers instead of dummies, and the PDU of the event that is scheduled. */
	int txb = 0;
	for (uint8_t s = 0; s < l->n_sm && txb == 0; s++) {
		txb = install_txb(&l->se[s], l->tx_cap);
	}
	if (txb != 0 || load_event(l) != 0) {
		cis_c6_link_close(ctx, (cis_link_t *)l);
		CIS_C6_CNT(link_open_fail);
		return txb != 0 ? -6 : -7;
	}
	CIS_C6_CNT(link_open_ok);
	cis_c6_live_mark(true);
	*out = (cis_link_t *)l;
	return 0;
}

void cis_c6_link_close(void *ctx, cis_link_t *link)
{
	struct cis_link *l = (struct cis_link *)link;
	(void)ctx;

	if (l == NULL) {
		return;
	}
	if (!link_has_sm(l)) {
		l->dead = false; /* the controller already ended the connsm (conn_end hook) - slot free */
		for (int i = 0; i < CIS_C6_MAX_LINKS; i++) {
			if (link_has_sm(&s_links[i]) || s_links[i].dead) {
				return;
			}
		}
		cis_c6_live_mark(false);
		return;
	}
	l->dead = false; /* NSE 2: the controller ended one connsm, we end the other here */
	CIS_C6_CNT(link_close);
	subs_end(l);
	for (int i = 0; i < CIS_C6_MAX_LINKS; i++) {
		if (link_has_sm(&s_links[i])) {
			return;
		}
	}
	cis_c6_live_mark(false); /* a crash is no longer "during a CIS" from now on */
}

void cis_c6_radio_reset(void)
{
	for (int i = 0; i < CIS_C6_ACL_KEPT; i++) {
		s_acl[i] = (cis_c6_acl_st_t){ .handle = 0xFFFF };
	}
	memset(s_links, 0, sizeof(s_links));
	s_watch.armed = false;
	cis_c6_live_mark(false);
}

void cis_c6_chan_stats(uint32_t tries[CIS_C6_CHANNELS], uint32_t bad[CIS_C6_CHANNELS])
{
	for (int i = 0; i < CIS_C6_CHANNELS; i++) {
		tries[i] = __atomic_load_n(&s_ch_tries[i], __ATOMIC_RELAXED);
		bad[i] = __atomic_load_n(&s_ch_bad[i], __ATOMIC_RELAXED);
	}
}

void cis_c6_radio_caps(void *ctx, cis_radio_caps_t *out)
{
	(void)ctx;
	const uint32_t nse = knob(CIS_C6_KNOB_MAX_NSE);

	/* The sequencer does one TX/RX exchange per ACL event, so subevent 1 is a second connsm. The knob
	 * (default 2 since 1.4.75) is read by the engine at Set CIG and at each CIS open - change only with no stream. */
	out->max_nse = (uint8_t)nse;
	/* P->C reception (microphone) in subevent 1 not yet checked on earbuds - a CIG with a microphone gets NSE 1. */
	out->max_nse_p_to_c = 1;
	out->radio_owns_seq = false;    /* SN/NESN are written by the adapter from the engine header (write_seq) */
	out->tx_prepare_lead_us = CIS_C6_TX_LEAD_US;
	/* NSE 2: the PDUs of all subevents are loaded by load_event after the previous event closes, i.e. before the first
	 * rx_done (model: the c6nse2 simulator). NSE 1 as before (false). */
	out->subevents_prepared_ahead = nse > 1;
	out->cis_gap_us = knob(CIS_C6_KNOB_CIS_GAP_US);
	out->acl_margin_us = knob(CIS_C6_KNOB_ACL_MARGIN_US);
	out->subevent_gap_us = knob(CIS_C6_KNOB_SUBEVENT_GAP_US);
	out->jitter_min_sdu  = (uint8_t)knob(CIS_C6_KNOB_JITTER_MIN);
	out->tx_slots        = (uint8_t)knob(CIS_C6_KNOB_TX_SLOTS);
}

int cis_c6_acl_timing(void *ctx, uint16_t acl_handle, cis_acl_timing_t *out)
{
	(void)ctx;
	void *sm = r_ble_ll_conn_sm_find_active_by_handle(acl_handle);

	uint8_t s = 0;
	if (sm == NULL || link_by_sm(sm, &s) != NULL) {
		return -1;
	}
	uint32_t phy = rd32(sm, SM_PHY);

	/* Everything describes the next scheduled event (next_event+0x6e/+0x8a/+0x150); read in the LL task,
	 * so consistent (only next_event changes it). */
	out->event_counter = rd16(sm, SM_EVENT_CTR);
	out->anchor_us = rd32(sm, SM_ANCHOR);
	out->interval_us = (uint32_t)rd16(sm, SM_ITVL) * 1250u;
	out->crc_init = rd32(sm, SM_CRC_INIT) & 0xFFFFFFu;
	memcpy(out->chan_map, f8(sm, SM_CHMAP), 5);
	out->phy_tx = (phy & 3u) == 2u ? CIS_PHY_2M : CIS_PHY_1M;        /* UNVERIFIED which field is TX */
	out->phy_rx = ((phy >> 2) & 3u) == 2u ? CIS_PHY_2M : CIS_PHY_1M;
	out->peripheral_latency = rd16(sm, SM_LATENCY);
	/* Upper bound of the ACL event time = scheduled end - anchor. UNVERIFIED: that the sequencer does not extend the
	 * event beyond +124 (event extension with data). Without a reliable value the engine uses its own margin. */
	uint32_t len = rd32(sm, SM_EVENT_END) - out->anchor_us;
	if (len == 0 || len >= out->interval_us) {
		CIS_C6_CNT(acl_evt_len_unknown);
		len = 0;
	}
	out->event_len_us = len;
	return 0;
}

uint32_t cis_c6_now_us(void *ctx)
{
	(void)ctx;
	return r_ble_lll_timer_current_tick_get(); /* the same 1 us clock as every anchor (N:Q7) */
}
