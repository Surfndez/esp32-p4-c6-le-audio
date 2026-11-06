/*
 * Cyberdeck LE Audio: contract between the CIS engine (Central) and the platform.
 *
 * The platform is either the ESP32-C6 controller adapter or the simulator (cis_ll/sim).
 * The engine knows nothing about ESP-IDF; all controller- or time-dependent work goes through cis_platform_ops_t.
 * Specification: docs/c6-link-layer.md. A change to this contract must be reflected there.
 *
 * Time: controller clock microseconds, uint32_t with wraparound (compare only via signed differences).
 * Contexts: cis_link_cb_t calls come from the radio context (short, non-blocking, no allocation);
 * the rest of the engine API from a single task (the LL task on the C6, the event loop in the simulator).
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CIS_PHY_1M 1
#define CIS_PHY_2M 2

/* Largest CIS PDU: 2 B header + 251 B payload (including MIC). */
#define CIS_PDU_MAX_PAYLOAD 251

/* One PDU on the air: header exactly as in Core (byte 0: LLID[1:0] NESN SN CIE RFU NPI RFU, byte 1: length). */
typedef struct {
	uint8_t hdr0;
	uint8_t len;
	uint8_t payload[CIS_PDU_MAX_PAYLOAD];
} cis_pdu_t;

/* Parameters of the periodic link run by the radio (on the C6: an ACL pseudo-connection). */
typedef struct {
	uint32_t access_address;
	uint32_t crc_init;          /* 24 bits, from the ACL connection */
	uint8_t  phy_c_to_p;        /* CIS_PHY_* */
	uint8_t  phy_p_to_c;
	uint8_t  chan_map[5];       /* ACL channel map (37 bits) */
	uint8_t  nse;               /* number of subevents; the radio reports max_nse in capabilities */
	uint32_t sub_interval_us;   /* subevent spacing (nse > 1) */
	uint32_t iso_interval_us;   /* CIS event spacing */
	uint32_t first_anchor_us;   /* controller clock time of the start of the first event (event 0) */
	uint8_t  max_pdu_c_to_p;    /* payload including MIC */
	uint8_t  max_pdu_p_to_c;
} cis_link_params_t;

typedef struct cis_link cis_link_t;

/* send_ctrl_pdu: the PDU does not fit the current ACL data length (the C6 splits every PDU to MaxTxOctets). */
#define CIS_PLAT_ERR_TOO_LONG (-4)

/* Calls from the radio context. */
typedef struct {
	/* Before a subevent: the engine fills in the PDU to transmit (ready, encrypted). Returns false = do not send
	 * anything in this subevent (the radio then sends a null PDU with NPI). event_counter counts from 0 (the event at
	 * first_anchor_us), subevent from 0 to nse-1. The engine builds the null PDU itself (NPI, NESN, CIE), so on a
	 * live link it returns true; false only for a closed link. */
	bool (*prepare_tx)(void *arg, uint32_t event_counter, uint8_t subevent, cis_pdu_t *out);
	/* After a subevent: the PDU from the Peripheral (crc_ok = false or pdu = NULL when nothing was received). */
	void (*rx_done)(void *arg, uint32_t event_counter, uint8_t subevent, const cis_pdu_t *pdu, bool crc_ok,
			uint32_t rx_timestamp_us);
	/* End of the CIS event (after the last subevent or a close via CIE). */
	void (*event_closed)(void *arg, uint32_t event_counter);
	/* The radio could not serve the event (schedule collision, Wi-Fi coexistence). */
	void (*event_skipped)(void *arg, uint32_t event_counter);
} cis_link_cb_t;

typedef struct {
	uint8_t max_nse;             /* how many subevents the radio supports (C6 version 1: 1) */
	bool    radio_owns_seq;      /* true: hardware sets SN/NESN (engine reads them from rx, ignores them in tx) */
	uint32_t tx_prepare_lead_us; /* how long before the anchor prepare_tx is called */
	/* true: prepare_tx of all subevents of an event comes before rx_done of the first (C6 at NSE > 1: separate
	 * pseudo-connection per subevent, PDUs loaded between events). Subevents >= 1 then carry the same payload with
	 * the same SN/NESN, and the acknowledgement may arrive in any of them. */
	bool    subevents_prepared_ahead;
	/* Gap in the CIG between the end of one CIS event and the anchor of the next (on top of T_MSS in se_length). A
	 * radio that serves each CIS with a separate schedule event (C6) cannot switch the link within T_MSS: the second
	 * CIS went out too late and the earbud did not hear it (Tab5, 2026-11-01: 1.3 % acknowledged). 0 = packing as in
	 * Core. */
	uint32_t cis_gap_us;
	/* Margin after the end of the ACL event when laying out the CIG (place_cig): an event with data lasts longer than
	 * planned end (event_len_us), and a CIS right behind it lost PDUs. 0 = no margin. */
	uint32_t acl_margin_us;
	/* Radio gap after each subevent beyond its length (se_length): Sub_Interval = se_length + subevent_gap_us at NSE
	 * > 1, and at NSE 1 a CIS takes se_length + subevent_gap_us in the CIG. The C6 serves each subevent (and each
	 * CIS) with a separate schedule event and needs about 1.5 ms between them (docs/c6-link-layer.md). 0 = as in
	 * Core. */
	uint32_t subevent_gap_us;
	/* Largest NSE for a CIG with P->C data (microphone), 0 = max_nse. C6: 1, until NSE 2 with a microphone is
	 * verified on earbuds. */
	uint8_t max_nse_p_to_c;
	/* Smallest C->P jitter buffer in SDUs (cis_cis_derived_t.jitter_c_to_p, at most CIS_JITTER_MAX), 0 = from
	 * Max_Transport_Latency alone. A radio to which the host delivers SDUs with delays (C6: SDIO shared with the SD
	 * card) wants margin regardless of how much budget FT consumed (NSE 2: FT 4 left 2 SDUs). */
	uint8_t jitter_min_sdu;
	/* Used slots of the C->P payload queue per CIS = host credits / CIS (LE Read Buffer Size v2), 0 = 12. The engine
	 * clamps to 12..16 (cis_engine.c CIS_TX_SLOTS / CIS_TX_SLOTS_MAX). More slots = a longer stall in credit return
	 * (NOCP over SDIO) without the host dropping SDUs. The host reads the credits on every LE Audio start. */
	uint8_t tx_slots;
} cis_radio_caps_t;

/* Timing state of the ACL connection (to compute CIS_Offset and the connEventCount instant). */
typedef struct {
	uint16_t event_counter;      /* ACL event counter of the nearest future event */
	uint32_t anchor_us;          /* its anchor */
	uint32_t interval_us;
	uint32_t crc_init;
	uint8_t  chan_map[5];
	uint8_t  phy_tx;             /* CIS_PHY_* currently in use on the ACL */
	uint8_t  phy_rx;
	uint16_t peripheral_latency;
	/* Upper bound of the ACL event time from the anchor (us); 0 = unknown (the engine uses CIS_ACL_GUARD_US).
	 * A CIS must not start before the sequencer finishes the ACL event (C6: end taken from connsm). */
	uint32_t event_len_us;
} cis_acl_timing_t;

/* CIS setup and teardown steps for on-device diagnostics (ops.trace): on the Tab5 without a sniffer this is the only
 * view of which step and with which values the setup with the earbuds stalls. v[] depends on the kind; layout = C6
 * record. */
typedef enum {
	CIS_TR_CREATE = 1,   /* Create CIS accepted: v0 AA */
	CIS_TR_REQ,          /* LL_CIS_REQ sent (code 1 = waiting for LE DLE): v0 ACL counter | connEventCount << 16,
	                      * v1 ACL anchor, v2 CIS_Offset_Min, v3 CIS_Offset_Max */
	CIS_TR_RSP,          /* LL_CIS_RSP: v0 connEventCount, v1 CIS_Offset_Min, v2 CIS_Offset_Max */
	CIS_TR_REFUSED,      /* Peripheral refused LL_CIS_REQ: code = error code, v0 = PDU opcode (0x11 / 0x07) */
	CIS_TR_IND,          /* LL_CIS_IND sent: v0 instant connEventCount | ACL counter << 16, v1 ACL anchor (for this
	                      * counter), v2 CIS_Offset, v3 ACL interval -
	                      * CIS anchor = v1 + (instant - counter) x v3 + v2 */
	CIS_TR_OPEN,         /* link_open: code 0 = opened, v0 first CIS anchor */
	CIS_TR_ESTABLISHED,  /* LE CIS Established to the host: code = status, v0 events, v1 rx_ok, v2 rx_crc */
	CIS_TR_END,          /* end of an established CIS (Disconnection Complete): code = reason, v0 events, v1 rx_ok,
	                      * v2 rx_crc, v3 rx_mic_fail */
	CIS_TR_PEER_TERM,    /* LL_CIS_TERMINATE_IND from the Peripheral: code = error code */
	CIS_TR_ACL_AT_INSTANT, /* platform only: ACL event at the LL_CIS_IND instant - code 1 = the instant passed without
	                        * an event with that counter, v0 counter, v1 actual anchor */
} cis_trace_kind_t;

typedef struct {
	uint8_t  kind;       /* cis_trace_kind_t */
	uint8_t  code;
	uint16_t cis;        /* CIS handle */
	uint16_t acl;        /* ACL handle */
	uint16_t rsv;
	uint32_t v[4];
} cis_trace_t;

typedef struct {
	void *ctx;
	/* HCI to the host (P4): full H4 packet with the type byte (0x04 event, 0x05 ISO). */
	void (*hci_to_host)(void *ctx, const uint8_t *h4, uint16_t len);
	/* Send an LL control PDU on the ACL: ctr = opcode + CtrData (without the PDU header). 0 = accepted for sending,
	 * CIS_PLAT_ERR_TOO_LONG = PDU longer than the current ACL MaxTxOctets (the engine waits for LE Data Length Change),
	 * any other negative value = error. */
	int (*send_ctrl_pdu)(void *ctx, uint16_t acl_handle, const uint8_t *ctr, uint8_t len);
	int (*acl_timing)(void *ctx, uint16_t acl_handle, cis_acl_timing_t *out);
	bool (*acl_is_central)(void *ctx, uint16_t acl_handle);
	/* LL feature bits of the remote device (from LL_FEATURE_RSP), 0 if unknown. */
	uint64_t (*acl_remote_features)(void *ctx, uint16_t acl_handle);
	int (*link_open)(void *ctx, const cis_link_params_t *p, const cis_link_cb_t *cb, void *cb_arg,
			cis_link_t **out);
	void (*link_close)(void *ctx, cis_link_t *link);
	void (*radio_caps)(void *ctx, cis_radio_caps_t *out);
	uint32_t (*now_us)(void *ctx);
	/* AES-128 ECB of one block (key and data in Core order - MSB first). */
	void (*aes128)(void *ctx, const uint8_t key[16], const uint8_t in[16], uint8_t out[16]);
	void (*random)(void *ctx, uint8_t *buf, uint16_t len);
	/* Schedule a cis_engine_on_timer() call after delay_us (one active engine timer). */
	void (*timer_start)(void *ctx, uint32_t delay_us);
	void (*log)(void *ctx, int level, const char *msg);
	/* Optional (NULL = absent): the session key and IV with which the controller encrypts the ACL - SK MSO first,
	 * iv[0..3] = IVm, iv[4..7] = IVs (LSO first, as in cis_crypto.h). 0 = provided, only when the controller has them
	 * complete and consistent (C6: SK = e(LTK, SKD) from the same connsm). Called when ACL encryption is enabled; the
	 * engine prefers them over the snooped LTK/SKD/IV - the earbuds already accepted them on the ACL. */
	int (*acl_session_keys)(void *ctx, uint16_t acl_handle, uint8_t sk[16], uint8_t iv[8]);
	/* Optional (NULL = absent): records a CIS setup step (cis_trace_t). From the engine task;
	 * record only - no calls back into the engine. */
	void (*trace)(void *ctx, const cis_trace_t *t);
} cis_platform_ops_t;

#ifdef __cplusplus
}
#endif
