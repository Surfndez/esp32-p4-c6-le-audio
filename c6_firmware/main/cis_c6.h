/*
 * Cyberdeck LE Audio: ESP32-C6 adapter (libble_app.a, ESP-IDF 5.5.5) for the CIS engine from cis_ll/.
 *
 * Specification: docs/c6-link-layer.md. Offsets and calls come from analysis of the controller library (radio,
 * LLCP, encryption, features, HCI; our analysis notes are not included).
 *
 * Contexts: the whole engine lives in the controller's LL task (hooks are called there; send_ctrl_pdu/acl_timing/
 * link_open must run there - design decision 6). Other tasks (SDIO, esp_timer) only queue work and wake the LL task
 * with a single npl event (cis_c6_kick). Until the host sends LE Set CIG Parameters, the hooks pass everything through.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "cis_engine.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- API for slave_bt.c ---- */

/* After esp_bt_controller_enable() and VHCI registration. 0 = OK; error = adapter disabled, BLE works as in v4. */
int cis_c6_init(void);
/* Before esp_bt_controller_disable(): from this moment everything is passed through, queues are dropped. */
void cis_c6_deinit(void);
/* H4 packet from the host (h4[0] = type). true = taken by the adapter (do not send to VHCI). */
bool cis_c6_h2c(const uint8_t *h4, uint16_t len);
/* H4 packet from the controller to the host. May modify the buffer in place. true = swallow (not sent to the host). */
bool cis_c6_c2h(uint8_t *h4, uint16_t len);
/* In slave_bt.c: sends an H4 packet to the host without interception (engine -> host path). */
int slave_bt_send_to_host(const uint8_t *h4, uint16_t len);

/* ---- adapter internals ---- */

/* Adapter counters (diagnostic 0xFC30). Order = order in the reply; append only at the end. */
typedef struct {
	uint32_t ll_kicks;          /* runs of npl event handling in the LL task */
	uint32_t h2c_cmd;           /* commands taken by the adapter */
	uint32_t h2c_iso;           /* HCI ISO packets from the host */
	uint32_t h2c_snoop;         /* copies of commands for inspection only (0x2019, 0x0C03) */
	uint32_t h2c_full_cmd;      /* ring full: the command went to the controller without the engine */
	uint32_t h2c_full_iso;      /* ring full: ISO packet dropped (the host loses a credit) */
	uint32_t h2c_full_snoop;    /* ring full: the inspection copy lost */
	uint32_t h2c_fwd_from_ll;   /* command taken but the engine did not want it - sent from the LL task */
	uint32_t c2h_direct;        /* events handed to the engine in the LL task */
	uint32_t c2h_async;         /* events from outside the LL task - inspection through the ring (plan B) */
	uint32_t c2h_swallowed;     /* events consumed by the engine */
	uint32_t c2h_full;          /* event inspection ring full */
	uint32_t rx_ctrl_consumed;  /* control PDUs consumed by the engine */
	uint32_t rx_ctrl_wrong_ctx; /* LLCP RX hook outside the LL task (only passed through) */
	uint32_t rx_ctrl_veto;      /* the engine wanted to consume a PDU outside the CIS set or with the CIG asleep */
	uint32_t tx_ctrl_ok;
	uint32_t tx_ctrl_fail;
	uint32_t tx_ctrl_too_long;  /* PDU longer than the ACL MaxTxOctets (the engine waits for DLE) */
	uint32_t enc_req_seen;      /* LL_ENC_REQ inspection */
	uint32_t hook_wrong_ctx;    /* radio hook on a CIS link outside the LL task */
	uint32_t link_open_ok;
	uint32_t link_open_fail;
	uint32_t link_close;
	uint32_t ev_done;           /* finished CIS events (event_closed) */
	uint32_t ev_skipped;        /* events that did not go on air */
	uint32_t rx_pdu;            /* received buffers with a header (good and bad) */
	uint32_t rx_bad;            /* hardware status with error bits or wrong length */
	uint32_t tx_new_buf;        /* link's own TX buffer set up instead of the controller's dummy (install_txb) */
	uint32_t tx_rewrite;        /* PDU written into the own TX buffer */
	uint32_t tx_load_fail;
	uint32_t timer_fires;
	uint32_t aes_fail;
	uint32_t acl_evt_len_unknown; /* acl_timing without a reliable ACL event length */
	uint32_t host_tx_fail;      /* engine packet to the host did not enter the SDIO queue */
	uint32_t h2c_iso_off;       /* ISO packet from the host while the adapter is disabled (dropped) */
	uint32_t crashes_in_cis;    /* C6 crashes during CIS for this image (from NVS, cis_c6_crash.h) */
	uint32_t tx_cur_foreign;    /* the link's current TX buffer is not ours - PDU not written */
	uint32_t ctr_backwards;     /* connsm event counter moved back by the controller */
	uint32_t host_q_full;       /* engine -> host queue full: packet dropped (NOCP deferred, not lost) */
	uint32_t recycle_skipped;   /* buffer set during an event skipped for CIS (recycle_buffer) */
	uint32_t seq_resync;        /* the controller changed the SN/NESN bits after the adapter's write - write repeated */
	uint32_t tx_cur_restored;   /* current TX buffer was empty (zeroed by hardware?) - ours restored (D7) */
	uint32_t rx_seq_guard;      /* bit 27 corrected before the RX set (unwritten current buffer, assert 699) */
	uint32_t ctrl_conn_end;     /* the controller itself ended the CIS connsm (conn_end hook) */
	uint32_t rx_in_event;       /* PDUs collected during an event (recycle_buffer, since 1.4.58) */
	uint32_t rx_stash_full;     /* collected PDU with no room in the link's stash - lost for the engine */
	/* NSE 2 (docs/c6-link-layer.md): */
	uint32_t se_skipped;        /* subevent did not go on air although another subevent of the same event did */
	uint32_t se_late;           /* connsm result of a subevent before the other settled the previous event - skipped */
	uint32_t se_open_fail;      /* the second CIS connsm failed to open - the link transmits only subevent 0 */
} cis_c6_counters_t;

extern cis_c6_counters_t g_cis_c6_cnt;
#define CIS_C6_CNT(f) ((void)__atomic_fetch_add(&g_cis_c6_cnt.f, 1u, __ATOMIC_RELAXED))

/* Experiment switches (0xFC30 subcommands 5/6): checking hypotheses R1-R3 and coexistence on the target device without
 * re-flashing the C6. RAM only - after boot always the default values. Read and changed only in the LL task
 * (diagnostics go through the H2C ring). Number = index in 0xFC30; append only at the end. */
typedef enum {
	CIS_C6_KNOB_SEQ_POLARITY = 0, /* 0/1: bit ls+20 = SN/NESN (0) or its negation (1) */
	CIS_C6_KNOB_SEQ_WRITE,        /* 1: the adapter writes the engine's SN/NESN to ls+20; 2: SN only (bit 27 = controller
	                               * RX buffers); 3: bit 26 = 1 (current TX buffer waits for transmission), SN only in the
	                               * header byte, bit 27 as in 1; 4: bit 25 = engine SN, bit 26 = 1, bit 27 = 0 (bit 24
	                               * NESN left to the hardware - wrong: 24 = SN, 25 = NESN); 5: model from ble_single_conn
	                               * (ls+0 bit31, bit24 SN, bit25 NESN, bit26 = 1, bit27 unchanged, acknowledgement from
	                               * bit 26 after the event, as the library does); 0: left to the hardware */
	CIS_C6_KNOB_HDR_SEQ,          /* 1: SN/NESN also in byte 0 of the TX buffer; 0: only LLID and NPI there (D4) */
	CIS_C6_KNOB_MIN_LEAD_US,      /* 500..100000: smallest lead before the first anchor in link_open */
	CIS_C6_KNOB_SCHED_PRIO,       /* 0..15: scheduler priority ls+84 after each event (0 = as the controller) */
	CIS_C6_KNOB_COEX_PTI,         /* 0..15: coexistence PTI ls+40[3:0] of CIS events (0 = as the controller: 4/9) */
	CIS_C6_KNOB_COEX_PTI_HI,      /* 0..15: ls+40[7:4] (0 = unchanged, link reset gives 4) */
	CIS_C6_KNOB_EVT_LIMIT,        /* 1: event limit with DLE (bit 7 +566) in link_open; 0: interval - 2000 us */
	CIS_C6_KNOB_NESN_TRACK,       /* NESN to ls+20/header instead of the engine's: 1/2 = event parity (^0/^1),
	                               * 3 = the earbuds' SN predicted from the last reception. LOCKED (range 0..0):
	                               * bit 27 = 1 before an event crashes the controller (1.4.60, crashes_in_cis) */
	CIS_C6_KNOB_SN_TRACK,         /* SN instead of the engine's: 1 = ACL rule (the earbuds' NESN from the last reception),
	                               * 2 = CIS rule with flush every event (after a drop NESN + 1, then every event) */
	CIS_C6_KNOB_CIS_GAP_US,       /* 0..6000: gap between CISes in the CIG (cis_radio_caps_t.cis_gap_us), read at
	                               * Set CIG - a change takes effect from the next LE Audio start (1.4.66) */
	CIS_C6_KNOB_TX_DBM,           /* BLE TX power in dBm: 0 = unchanged (sdkconfig, +9); 3/6/9/12/15/18/20 - at once
	                               * for existing connections and the default for new ones (1.4.70) */
	CIS_C6_KNOB_ACL_MARGIN_US,    /* 0..3000: margin after the ACL event when laying out the CIG
	                               * (cis_radio_caps_t.acl_margin_us), from the next LE Audio start (1.4.73) */
	CIS_C6_KNOB_MAX_NSE,          /* 1..2: cis_radio_caps_t.max_nse - 2 = a second SDU attempt in a CIS event (second
	                               * connsm per CIS, docs/c6-link-layer.md), from the next LE Audio start */
	CIS_C6_KNOB_SUBEVENT_GAP_US,  /* 0..3000: subevent gap beyond se_length (cis_radio_caps_t.subevent_gap_us), from
	                               * the next LE Audio start */
	CIS_C6_KNOB_JITTER_MIN,       /* 0..4: smallest C->P jitter buffer in SDUs (cis_radio_caps_t.jitter_min_sdu), from
	                               * the next LE Audio start (1.4.76) */
	CIS_C6_KNOB_TX_SLOTS,         /* 12..16 (default 16): TX queue slots per CIS = host credits / CIS
	                               * (cis_radio_caps_t.tx_slots); the P4 reads the credits at every LE Audio start - from
	                               * the next start (1.4.79) */
	CIS_C6_KNOB_COUNT
} cis_c6_knob_t;

extern uint32_t g_cis_c6_knob[CIS_C6_KNOB_COUNT];
/* false = wrong number or value out of range (state unchanged). */
bool cis_c6_knob_set(uint8_t id, uint32_t value);

/* CIS event black box (0xFC30 subcommand 4): the first CIS_C6_BB_FIRST events after the last link_open
 * and the circulating last CIS_C6_BB_LAST. Byte layout = the diagnostic reply (little endian); a change = a new
 * version of the diagnostic format. */
#define CIS_C6_BB_FIRST 48
#define CIS_C6_BB_LAST  48

#define CIS_C6_BB_RX_OK    0x01u /* PDU received without error according to the hardware status */
#define CIS_C6_BB_TX_NEW   0x02u /* first PDU in the own TX buffer (right after install_txb) */
#define CIS_C6_BB_SKIPPED  0x04u /* the event did not go on air (event_skipped) */
#define CIS_C6_BB_TX_FAIL  0x08u /* tx_load did not load the PDU */
#define CIS_C6_BB_SEQ_SET  0x10u /* the adapter wrote SN/NESN to ls+20 */
#define CIS_C6_BB_SE1      0x20u /* subevent 1 entry (NSE 2, second connsm); no flag: subevent 0 or the whole
                                  * event (CIS_C6_BB_SKIPPED) */

typedef struct {
	uint16_t ev;        /* low 16 bits of the CIS event counter */
	uint8_t  link;      /* adapter link index */
	uint8_t  flags;     /* CIS_C6_BB_* */
	uint8_t  ch;        /* data channel of the event (connsm+67) */
	uint8_t  tx_hdr;    /* byte 0 of the header from the engine */
	uint8_t  rx_hdr;    /* raw byte 0 of the last received PDU */
	uint8_t  rx_len;
	uint32_t rx_status; /* hardware status of the last PDU (24 bits); 0xFFFFFFFF = nothing received */
	uint32_t seq_set;   /* ls+20 after the adapter's write, before the event */
	uint32_t seq_after; /* ls+20 after the event, before the buffer set - what the hardware did */
	int16_t  rx_off_us; /* start of the received PDU relative to the event anchor */
	uint8_t  txed;      /* 1 = after the event the current TX buffer is still ours (0 = hardware/controller moved it) */
	uint8_t  rxed;      /* RX buffers with a header */
} cis_c6_bb_t;

/* which 0 = first events, 1 = last events (index 0 = oldest kept). Returns the number of copied entries,
 * *total = how many were recorded (for 1: since boot, entries older than CIS_C6_BB_LAST are lost). LL task. */
uint8_t cis_c6_bb_read(uint8_t which, uint16_t index, uint8_t max, cis_c6_bb_t *out, uint32_t *total);

/* Work bits for the LL task (cis_c6_kick). */
#define CIS_C6_KICK_H2C     (1u << 0)
#define CIS_C6_KICK_C2H     (1u << 1)
#define CIS_C6_KICK_TIMER   (1u << 2)
#define CIS_C6_KICK_PROCESS (1u << 3)

/* Wake the LL task (safe from any task; not from an ISR). */
void cis_c6_kick(uint32_t bits);
/* RTC "CIS live" marker (cis_c6_crash.h): true when a link opens, false when none is left. */
void cis_c6_live_mark(bool live);
bool cis_c6_in_ll_task(void);
/* Adapter on (engine initialised, controller enabled). */
bool cis_c6_on(void);
/* The engine when the adapter is on and in the LL task; otherwise NULL. */
cis_engine_t *cis_c6_engine_ll(void);

/* cis_c6_radio.c */
int cis_c6_link_open(void *ctx, const cis_link_params_t *p, const cis_link_cb_t *cb, void *cb_arg,
		     cis_link_t **out);
void cis_c6_link_close(void *ctx, cis_link_t *link);
void cis_c6_radio_caps(void *ctx, cis_radio_caps_t *out);
int cis_c6_acl_timing(void *ctx, uint16_t acl_handle, cis_acl_timing_t *out);
uint32_t cis_c6_now_us(void *ctx);
/* After controller (re)initialisation: the controller released everything, we forget the links. */
void cis_c6_radio_reset(void);
/* LL task (npl event handling): links ended by the controller lead to a close by the engine. */
void cis_c6_radio_tick(void);
/* CIS channel quality (0xFC30 subcommand 11): subevents on the channel and those without a valid earbud reply,
 * since C6 boot, all links together - the basis for excluding noisy channels (channel map, AFH). */
#define CIS_C6_CHANNELS 40
void cis_c6_chan_stats(uint32_t tries[CIS_C6_CHANNELS], uint32_t bad[CIS_C6_CHANNELS]);
/* ACL connection events (0xFC30 subcommand 12): planned (next_event) vs executed (update_buffer_hdr) - the gap is
 * events given to others (CIS, scan), e.g. before 0x08 of the 2nd earbud; plus parameters, changes. Layout = reply. */
#define CIS_C6_ACL_KEPT 4
typedef struct {
	uint16_t handle;    /* 0xFFFF = free */
	uint16_t itvl;      /* 1,25 ms */
	uint16_t spvn_tmo;  /* 10 ms */
	uint16_t latency;
	uint32_t planned;
	uint32_t aired;
	uint32_t changes;   /* changes of itvl / latency / spvn_tmo since the first event */
} cis_c6_acl_st_t;
void cis_c6_acl_stats(cis_c6_acl_st_t out[CIS_C6_ACL_KEPT]);

/* Radio test without a Peripheral (0xFC30 subcommands 7/8, cis_c6_radio.c): layout = diagnostic reply, little endian */
#define CIS_C6_RT_IDLE      0
#define CIS_C6_RT_RUNNING   1
#define CIS_C6_RT_DONE      2
#define CIS_C6_RT_OPEN_FAIL 3
typedef struct {
	uint8_t  state;     /* CIS_C6_RT_* */
	uint8_t  pdu_len;
	int8_t   open_rc;   /* cis_c6_link_open code at CIS_C6_RT_OPEN_FAIL */
	uint8_t  rsv;
	uint32_t target;    /* events to settle (finished + skipped) */
	uint32_t closed;    /* finished events (update_buffer_hdr) */
	uint32_t skipped;   /* events skipped by the scheduler */
	uint32_t rx_any;    /* anything received (without the other side it should be 0) */
	uint32_t seq_moved; /* events after which the hardware changed the written SN/NESN bits (without any ACK) */
	int32_t  dev_min;   /* event anchor - (anchor0 + k x ISO), us */
	int32_t  dev_max;
	uint32_t anchor0;
} cis_c6_rt_t;
/* LL task. 0 = test started; -1 running, -2 links busy, -3 bad parameters, other = cis_c6_link_open code. */
int cis_c6_radio_test_start(uint16_t events, uint8_t pdu_len, uint8_t phy);
void cis_c6_radio_test_get(cis_c6_rt_t *out);

/* CIS setup trace (0xFC30 subcommand 10): engine steps (ops.trace, cis_platform.h) and the ACL anchor at the moment
 * of LL_CIS_IND; the last CIS_C6_TR_KEPT entries circulate. Layout = diagnostic reply, little endian. */
#define CIS_C6_TR_KEPT 32
/* Trace entries outside cis_trace_kind_t: control PDUs of a regular connection (cis_c6_llcp.c acl_ctrl_trace). */
#define CIS_C6_TR_ACL_CTRL_RX 0x40 /* received: code = opcode, v0 = length, v1 = CtrData[0..3] */
#define CIS_C6_TR_ACL_CTRL_TX 0x41 /* sent by the controller (only LL_ENC_REQ - enc_req_make hook) */
typedef struct {
	uint32_t    t_us; /* cis_c6_now_us at the time of the write */
	cis_trace_t tr;
} cis_c6_tr_t;
/* ops.trace of the engine; LL task. */
void cis_c6_trace(void *ctx, const cis_trace_t *t);
/* From *index (absolute; older than kept - moves to the oldest) at most max entries; *total = how many were recorded
 * since boot. LL task. */
uint8_t cis_c6_trace_read(uint32_t *index, uint8_t max, cis_c6_tr_t *out, uint32_t *total);

/* Task watchdog hang trace (cis_c6_hang.c, 0xFC30 subcommand 9): layout = diagnostic reply. */
#define CIS_C6_HANG_CODE 12
typedef struct {
	uint32_t magic;       /* CIS_C6_HANG_MAGIC = record valid; 0 = the C6 has not hung since the NVS was cleared */
	uint8_t  img[4];      /* beginning of the SHA-256 of the ELF of the image that hung */
	uint32_t uptime_ms;   /* from boot to the first watchdog hit */
	uint16_t hits;        /* watchdog hits in this hang (abort after CIS_C6_HANG_ABORT_HITS) */
	uint16_t rsv;
	char     task[12];    /* task interrupted by the watchdog ISR */
	uint32_t mepc;        /* where it stood (the task's interrupt frame) */
	uint32_t ra;
	uint32_t mcause;
	uint32_t code[CIS_C6_HANG_CODE]; /* code addresses from the task stack, from the frame upwards (0 = none) */
	uint32_t kicks_first; /* ll_kicks at the first hit - the increase up to kicks_last = npl event loop */
	uint32_t kicks_last;
	uint32_t ev_done;
	uint32_t ev_skipped;
	uint32_t link_open_ok;
	uint32_t link_close;
	uint32_t ctrl_conn_end;
	uint32_t recycle_skipped;
	uint32_t hook_wrong_ctx;
	cis_c6_rt_t rt;       /* radio test state at the last hit */
} cis_c6_hang_t;
#define CIS_C6_HANG_MAGIC 0x474E4148u /* 'HANG' */
/* Last trace (from NVS, written at the first boot after a hang); recognise the image by img. */
void cis_c6_hang_get(cis_c6_hang_t *out);

/* cis_c6_llcp.c */
int cis_c6_send_ctrl_pdu(void *ctx, uint16_t acl_handle, const uint8_t *ctr, uint8_t len);
bool cis_c6_acl_is_central(void *ctx, uint16_t acl_handle);
uint64_t cis_c6_acl_remote_features(void *ctx, uint16_t acl_handle);
/* Bits 28 (CIS Central), 32 (ISO Host Support) in the controller feature word (on-air LL_FEATURE_REQ/RSP). LL task. */
void cis_c6_set_iso_features(bool on);
bool cis_c6_iso_features_on(void);
/* Cross-check of SK: 0 = AES(LTK, SKD) from connsm == the controller's SK, 1 = equal after byte reversal,
 * 2 = not equal, 3 = no connection / encryption (connsm+272 state other than 2/7/8). LL task. Keys do not
 * leave the C6. */
uint8_t cis_c6_sk_check(uint16_t acl_handle);
/* acl_session_keys contract: SK and IV straight from connsm (SK order as in cis_c6_sk_check), only for result 0/1. */
int cis_c6_acl_session_keys(void *ctx, uint16_t acl_handle, uint8_t sk[16], uint8_t iv[8]);
/* AES-128 ECB (mbedtls, the general-purpose hardware AES accelerator). */
void cis_c6_aes128(void *ctx, const uint8_t key[16], const uint8_t in[16], uint8_t out[16]);

#ifdef __cplusplus
}
#endif
