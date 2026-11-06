/*
 * Cyberdeck LE Audio: CIS engine on the Central side - see cis_engine.h and docs/c6-link-layer.md.
 *
 * Split into contexts (and why):
 * - the radio context (cis_link_cb_t) only copies a ready PDU, computes SN/NESN, acknowledgements and flush points
 *   and copies the raw PDU from the Peripheral into a queue - no encryption and no HCI, because on the C6 this is
 *   the sequencer ISR and every microsecond in it shifts T_IFS;
 * - the task (the rest of the API) encrypts SDUs ahead of time (the payload number is known up front: event x BN),
 *   decrypts received ones, reassembles SDUs, sends HCI and runs LLCP. Exchange between them: TX slots with an
 *   atomic state (task: FREE -> READY, radio: READY -> ACKED/FLUSHED, task: -> FREE), an RX SPSC queue, a few atomic
 *   counters (last event, receive progress). The platform guarantees that no radio call for this link arrives after
 *   link_close().
 *
 * Payload number and SN/NESN per Core Vol 6 Part B 4.5.13 and Zephyr lll_central_iso.c: every payload (acknowledged
 * or flushed) toggles SN once; on the receive side a flush toggles NESN. This keeps both sides in agreement without
 * exchanging anything beyond the headers.
 *
 * Memory: one block from the platform, no malloc and no recursion.
 */
#include "cis_engine.h"

#include <stdatomic.h>
#include <stddef.h>
#include <string.h>

#include "cis_crypto.h"
#include "cis_proto.h"

/* ---- configuration (can be overridden with -D) ---- */
/* The C->P jitter buffer (SDUs before the first transmission) is computed by cis_cig_derive from
 * Max_Transport_Latency (CIS_JITTER_MIN..MAX), the engine clamps it to the queue (jitter_depth). */
#ifndef CIS_SUPERVISION_US
#define CIS_SUPERVISION_US 5000000u /* no PDU from the Peripheral for this long = CIS lost (0x08) */
#endif
#ifndef CIS_ACL_GUARD_US
#define CIS_ACL_GUARD_US 1000u      /* margin for an ACL event before and after the CIS (short control PDUs) */
#endif
#define CIS_MAX_ACL         4
#define CIS_TX_SLOTS        12      /* >= BN x (JITTER x 2 + 2): payload queue of one CIS */
/* Queue slots from cis_radio_caps_t.tx_slots (12..16; the arrays are always sized for 16). A payload holds a slot,
 * and an SDU a credit of the host, from acceptance until NOCP: the queue up to the target (1 + buffer + 1) plus the
 * retransmission window (FT - 1 events after its event) plus the credit return over SDIO R. BN 1, buffer 4, FT 4: 6
 * + 3 = 9, so 12 slots leave R = 3 intervals (30 ms) of return stall, 16 - 9 = 7 (70 ms). The jitter buffer
 * (jitter_depth) is still computed from CIS_TX_SLOTS - a larger pool only changes the credit margin, not the
 * latency. */
#define CIS_TX_SLOTS_MAX    16
#define CIS_RX_SLOTS        8
#define CIS_HANDLE_BASE     0x0E00u /* outside the range of the C6 controller ACL handles (small numbers) */
#define CIS_PROC_TIMEOUT_US 40000000u /* LL response timeout (Core); the controller does not watch our PDUs */
#define CIS_EST_EVENTS      6       /* Core: no reception in the first 6 events = 0x3E */
#define CIS_INSTANT_MARGIN  6       /* ACL events to deliver LL_CIS_IND before the start instant */
#define CIS_INSTANT_SEARCH  32      /* how many consecutive ACL events we try to match to the CIG phase */
#define CIS_LOST_REPORT_MAX 4       /* after a longer gap we report only this many lost SDUs */
#define CIS_HCI_ISO_LOAD    (8u + CIS_SDU_MAX)
/* 0x2060 comes before Set CIG (the depth is not known yet): one credit per payload queue slot of all CISes. A credit
 * returns only after transmission and Number Of Completed Packets, and on a shared SDIO bus (SD card with music,
 * Wi-Fi) the return can be late by tens of ms - with 20 credits for two CISes the host dropped to 0 free and dropped
 * about 2 SDU/s (Tab5, 2026-11-01, drop_no_credit). A full C6 queue drops SDUs by itself anyway (and returns the
 * credit). */
#define CIS_HOST_CREDITS(slots) (CIS_MAX_CIS * (slots))
#define CIS_NO_DEADLINE     0
/* Number of consecutive late SDUs (different PSNs) before the CIG gets a new mapping: a series of backlogged SDUs
 * after a host stall (< 50 ms; a longer P4 stall shifts the SDU number) drops out by itself, and on-time SDUs stay
 * in place. Only a lasting shift of the host numbering (clock drift) moves the CIG - at the cost of this many
 * intervals of silence. */
#define CIS_REMAP_LATE      8
/* Number of consecutive SDUs (of all CISes) accepted above the queue target before the CIG drops one SDU and the
 * queue steps back by an interval: about 1 s with two CISes. Without it the queue stayed at 2 x buffer after a burst
 * and held the host credits. */
#define CIS_RECENTER_SDU    200
/* Host clock drift (Tab5, 2026-11-01: after 30 min 10 % of SDUs late): SDUs hover at the lateness boundary,
 * alternately on time and late, so CIS_REMAP_LATE in a row never occurs. CIS_DRIFT_LATE late episodes (a late SDU
 * after an on-time one) within a window of CIS_DRIFT_WINDOW SDUs shift the mapping by one event: one gap instead of
 * every few SDUs without end. A series of late SDUs from a single stall is one episode - the mapping stays. */
#define CIS_DRIFT_WINDOW    64
#define CIS_DRIFT_LATE      2
#define CIS_ACL_PREP_US     300u    /* radio preparation before the ACL event anchor (place_cig) */
#define CIS_PLACE_STEP_US   50u     /* CIS_Offset search step in place_cig */

#if CIS_TX_SLOTS < 2 * (2 * CIS_JITTER_MIN + 2)
#error "CIS_TX_SLOTS too small for CIS_JITTER_MIN at BN = 2"
#endif
#if CIS_TX_SLOTS < 2 * CIS_JITTER_MAX + 2
#error "CIS_TX_SLOTS too small for CIS_JITTER_MAX at BN = 1"
#endif
#if CIS_TX_SLOTS_MAX < CIS_TX_SLOTS || CIS_TX_SLOTS_MAX > 127
#error "CIS_TX_SLOTS_MAX outside CIS_TX_SLOTS..127 (int8_t indices)"
#endif
#if CIS_HOST_CREDITS(CIS_TX_SLOTS_MAX) > 255
#error "host credits do not fit in Total_Num_ISO_Data_Packets (u8)"
#endif

enum { LOG_ERR = 1, LOG_WARN = 2, LOG_INFO = 3 };

/* CIS state on the task side. */
typedef enum {
	CS_FREE = 0,       /* unused slot */
	CS_CONFIGURED,     /* in a CIG, not connected */
	CS_WAIT_SEND,      /* Create CIS accepted, LL_CIS_REQ not sent yet (ACL busy or the PDU is too long) */
	CS_WAIT_RSP,       /* LL_CIS_REQ sent */
	CS_ESTABLISHING,   /* LL_CIS_IND sent, link open, waiting for the first PDU */
	CS_ESTABLISHED,
} cis_state_t;

enum { SLOT_FREE = 0, SLOT_FILLING, SLOT_READY, SLOT_ACKED, SLOT_FLUSHED };

/* r_tx_slot: -1 = null PDU (NPI), -2 = padding PDU for an empty payload (requires an acknowledgement) */
#define TX_NULL    (-1)
#define TX_PADDING (-2)

/* radio -> task flags */
#define F_RX_SEEN     0x01u
#define F_EST_TIMEOUT 0x02u
#define F_SUPERVISION 0x04u

typedef struct {
	_Atomic uint8_t state;
	_Atomic uint8_t sent;      /* the radio transmitted at least once (for LE Read ISO TX Sync) */
	uint8_t  hci_pkts;         /* HCI packets closed on release (the last payload of an SDU) */
	bool     sdu_first;        /* first payload of an SDU */
	uint16_t psn;
	uint64_t pn;
	cis_pdu_t pdu;             /* payload ready for the air; hdr0 = LLID only */
} tx_slot_t;

typedef struct {
	uint64_t pn;
	cis_pdu_t pdu;             /* raw, encrypted */
} rx_slot_t;

typedef struct {
	_Atomic uint32_t events, events_skipped, tx_pdus, tx_acked, tx_flushed, tx_underrun, sdu_in, sdu_dropped,
		sdu_late, rx_ok, rx_crc, rx_mic_fail, rx_lost, rx_overflow, key_src;
} stats_atomic_t;

struct cis_engine;

typedef struct {
	struct cis_engine *e;
	cis_state_t st;
	uint16_t handle;
	cis_hci_cis_cfg_t cfg;
	cis_cis_derived_t d;
	uint16_t acl;
	uint32_t aa;
	uint32_t deadline;         /* end time of the LLCP procedure (when deadline_on) */
	uint32_t req_omin;         /* CIS_Offset window sent in LL_CIS_REQ */
	uint32_t req_omax;
	bool     deadline_on;
	bool     cancel;           /* the host cancelled Create CIS before the response */
	bool     dp_in;
	bool     dp_out;
	cis_link_t *link;
	uint32_t anchor0;          /* anchor of event 0 */
	bool     enc;
	cis_ccm_t ccm;

	/* --- radio context only (after link_open) --- */
	uint64_t r_ev;
	uint64_t r_tx_pn;
	uint64_t r_rx_pn;
	uint8_t  r_sn;
	uint8_t  r_nesn;
	int8_t   r_tx_slot;        /* slot sent in the current subevent, -1 = null PDU */
	bool     r_rx_seen;
	uint64_t r_last_rx_ev;
	bool     r_owns_seq;
	uint8_t  r_rx_max_len;     /* largest valid PDU len from the Peripheral (with MIC) */

	/* --- shared --- */
	_Atomic bool     a_open;
	_Atomic uint32_t a_ev;         /* low 32 bits of the last event seen by the radio */
	_Atomic uint32_t a_rx_pn;      /* low 32 bits of r_rx_pn (after reception or a flush) */
	_Atomic uint8_t  a_flags;
	_Atomic bool     a_tx_started;
	uint64_t tx_first_pn;          /* written before a_tx_started = true */
	tx_slot_t tx[CIS_TX_SLOTS_MAX];
	rx_slot_t rx[CIS_RX_SLOTS];
	_Atomic uint32_t rx_wr;
	_Atomic uint32_t rx_rd;
	stats_atomic_t s;

	/* --- task only --- */
	uint64_t t_ev;             /* a_ev extended to 64 bits */
	bool     t_started;
	uint64_t t_next_ev;        /* event for the next SDU */
	uint64_t cig_off;          /* CIG event (shared by the CISes) = cig_off + this CIS event */
	uint64_t t_rx_next;        /* next payload to reassemble */
	uint16_t nocp;             /* HCI packets to return to the host */
	bool     tx_sync_valid;
	uint16_t tx_sync_psn;
	uint32_t tx_sync_ts;
	/* reassembling the SDU from HCI fragments */
	uint8_t  h_buf[CIS_SDU_MAX];
	uint16_t h_len;
	uint16_t h_expect;
	uint16_t h_psn;
	uint8_t  h_pkts;
	bool     h_active;
	bool     h_bad;
	/* reassembling the SDU from PDUs */
	uint8_t  u_buf[CIS_SDU_MAX];
	cis_unframed_rx_t u;
} cis_t;

typedef struct {
	bool     used;
	uint16_t handle;
	bool     ltk_ok, skdm_ok, skds_ok;
	uint8_t  ltk[16];
	uint8_t  skdm[8], ivm[4], skds[8], ivs[4];
	bool     enc;
	bool     enc_no_keys;      /* ACL encrypted, but neither the snooping nor the controller knows the key */
	uint8_t  key_src;          /* CIS_KEY_* */
	uint8_t  sk[16];
	uint8_t  iv[8];
	uint16_t max_tx_octets;    /* 0 = unknown */
	bool     dl_blocked;       /* last LL_CIS_REQ attempt: PDU too long */
} acl_t;

struct cis_engine {
	cis_platform_ops_t ops;
	cis_radio_caps_t caps;
	bool     host_cis;         /* LE Set Host Feature bit 32 */
	bool     acl_buf_known;
	uint16_t acl_buf_len;
	uint8_t  acl_buf_num;
	bool     cig_used;
	cis_hci_cig_params_t cig;  /* CISes in slot order (cig.cis[i] <-> cis[i]) */
	cis_cig_derived_t der;
	acl_t    acl[CIS_MAX_ACL];
	cis_t    cis[CIS_MAX_CIS];
	uint8_t  iso_out[5 + CIS_HCI_ISO_LOAD];
	/* CIG event numbering: event 0 = the CIG anchor of the first opened CIS (until all are closed). */
	bool     cig_ref_valid;
	uint32_t cig_ref_us;
	/* Host SDU -> CIG event, one for all CISes (PSN map_psn goes in CIG event map_gev): the same SDU of
	 * L/R in the same ISO interval on every CIS. Task only. */
	bool     map_valid;
	uint16_t map_psn;
	uint64_t map_gev;
	uint16_t late_run;         /* consecutive late SDUs (different PSNs) - a new mapping only from CIS_REMAP_LATE */
	uint16_t late_psn;         /* PSN of the last late one (a second CIS with the same PSN does not count twice) */
	uint16_t high_run;         /* consecutive SDUs above the queue target (1 + buffer + 1) - CIS_RECENTER_SDU */
	uint16_t win_psn;          /* last PSN in the drift window (both CISes with the same PSN count once) */
	uint8_t win_sdus;          /* SDUs in the current CIS_DRIFT_WINDOW window */
	uint8_t win_late;          /* late SDUs in this window - from CIS_DRIFT_LATE a shift by one event */
};

static const cis_link_cb_t k_link_cb;
static void refresh_caps(cis_engine_t *e);

/* ======================================================================== helpers */

static void elog(cis_engine_t *e, int level, const char *msg)
{
	if (e->ops.log)
		e->ops.log(e->ops.ctx, level, msg);
}

static void to_host(cis_engine_t *e, const uint8_t *p, uint16_t len)
{
	e->ops.hci_to_host(e->ops.ctx, p, len);
}

/* Setup step for the platform trace (cis_trace_kind_t; ops.trace is optional). */
static void tr(cis_engine_t *e, const cis_t *c, uint8_t kind, uint8_t code, uint32_t v0, uint32_t v1, uint32_t v2,
	       uint32_t v3)
{
	if (!e->ops.trace)
		return;
	cis_trace_t t = { .kind = kind, .code = code, .cis = c->handle, .acl = c->acl, .v = { v0, v1, v2, v3 } };
	e->ops.trace(e->ops.ctx, &t);
}

static uint32_t st_ld(const _Atomic uint32_t *v)
{
	return atomic_load_explicit(v, memory_order_relaxed);
}

static void cmd_complete(cis_engine_t *e, uint16_t op, const uint8_t *ret, uint8_t n)
{
	uint8_t b[6 + 32];
	to_host(e, b, cis_hci_evt_cmd_complete(b, op, ret, n));
}

static void cmd_complete_status(cis_engine_t *e, uint16_t op, uint8_t status)
{
	cmd_complete(e, op, &status, 1);
}

static void cmd_status(cis_engine_t *e, uint16_t op, uint8_t status)
{
	uint8_t b[8];
	to_host(e, b, cis_hci_evt_cmd_status(b, status, op));
}

#define STAT_INC(c, f) atomic_fetch_add_explicit(&(c)->s.f, 1u, memory_order_relaxed)
#define STAT_ADD(c, f, n) atomic_fetch_add_explicit(&(c)->s.f, (uint32_t)(n), memory_order_relaxed)

static cis_t *cis_by_handle(cis_engine_t *e, uint16_t h)
{
	if (h < CIS_HANDLE_BASE || h >= CIS_HANDLE_BASE + CIS_MAX_CIS)
		return NULL;
	cis_t *c = &e->cis[h - CIS_HANDLE_BASE];
	return c->st == CS_FREE ? NULL : c;
}

static acl_t *acl_get(cis_engine_t *e, uint16_t h, bool create)
{
	acl_t *free_slot = NULL;
	for (int i = 0; i < CIS_MAX_ACL; i++) {
		if (e->acl[i].used && e->acl[i].handle == h)
			return &e->acl[i];
		if (!e->acl[i].used && !free_slot)
			free_slot = &e->acl[i];
	}
	if (!create || !free_slot)
		return NULL;
	memset(free_slot, 0, sizeof(*free_slot));
	free_slot->used = true;
	free_slot->handle = h;
	return free_slot;
}

static uint64_t extend64(uint64_t base, uint32_t lo)
{
	return base + (uint32_t)(lo - (uint32_t)base);
}

/* ======================================================================== radio context */

static int8_t r_find_slot(cis_t *c, uint64_t pn)
{
	for (int8_t i = 0; i < CIS_TX_SLOTS_MAX; i++) {
		if (atomic_load_explicit(&c->tx[i].state, memory_order_acquire) == SLOT_READY && c->tx[i].pn == pn)
			return i;
	}
	return -1;
}

/* Payload pn leaves the queue without an acknowledgement (the flush point has passed). */
static void r_retire_tx_flushed(cis_t *c, uint64_t pn)
{
	int8_t s = r_find_slot(c, pn);
	if (s >= 0) {
		atomic_store_explicit(&c->tx[s].state, SLOT_FLUSHED, memory_order_release);
		STAT_INC(c, tx_flushed);
	} else if (atomic_load_explicit(&c->a_tx_started, memory_order_acquire) && pn >= c->tx_first_pn) {
		STAT_INC(c, tx_underrun);
	}
}

/* Flush all payloads whose last chance (event, subevent) is before (ev, se).
 * se = 0xFF: the end of event ev. */
static void r_flush(cis_t *c, uint64_t ev, uint8_t se)
{
	const cis_cis_derived_t *d = &c->d;
	if (d->bn_c_to_p) {
		uint8_t bn = d->bn_c_to_p, ft = d->ft_c_to_p;
		/* jump after skipped events: all payloads of events < ev - FT + 1 are already past the point */
		if (ev >= ft) {
			uint64_t np = (ev - (ft - 1u)) * bn;
			if (np > c->r_tx_pn + bn) {
				np -= bn;
				bool started = atomic_load_explicit(&c->a_tx_started, memory_order_acquire);
				uint64_t from = c->r_tx_pn;
				if (started && c->tx_first_pn > from)
					from = c->tx_first_pn;
				uint64_t with_data = 0;
				for (int8_t i = 0; i < CIS_TX_SLOTS_MAX; i++) {
					if (atomic_load_explicit(&c->tx[i].state, memory_order_acquire) == SLOT_READY &&
					    c->tx[i].pn < np) {
						atomic_store_explicit(&c->tx[i].state, SLOT_FLUSHED, memory_order_release);
						STAT_INC(c, tx_flushed);
						with_data += c->tx[i].pn >= from;
					}
				}
				/* payloads without data in the skipped range = underrun (only after the stream has started) */
				if (started && np > from + with_data)
					STAT_ADD(c, tx_underrun, np - from - with_data);
				c->r_sn ^= (uint8_t)((np - c->r_tx_pn) & 1u);
				c->r_tx_pn = np;
			}
		}
		for (;;) {
			uint64_t fe;
			uint8_t fs;
			cis_flush_point(c->r_tx_pn, bn, ft, d->nse, &fe, &fs);
			if (fe > ev || (fe == ev && fs >= se))
				break;
			r_retire_tx_flushed(c, c->r_tx_pn);
			c->r_tx_pn++;
			c->r_sn ^= 1u;
		}
	}
	if (d->bn_p_to_c) {
		uint8_t bn = d->bn_p_to_c, ft = d->ft_p_to_c;
		uint64_t before = c->r_rx_pn;
		if (ev >= ft) {
			uint64_t np = (ev - (ft - 1u)) * bn;
			if (np > c->r_rx_pn + bn) {
				np -= bn;
				c->r_nesn ^= (uint8_t)((np - c->r_rx_pn) & 1u);
				c->r_rx_pn = np;
			}
		}
		for (;;) {
			uint64_t fe;
			uint8_t fs;
			cis_flush_point(c->r_rx_pn, bn, ft, d->nse, &fe, &fs);
			if (fe > ev || (fe == ev && fs >= se))
				break;
			c->r_rx_pn++;
			c->r_nesn ^= 1u;
		}
		if (c->r_rx_pn != before) {
			STAT_ADD(c, rx_lost, c->r_rx_pn - before);
			atomic_store_explicit(&c->a_rx_pn, (uint32_t)c->r_rx_pn, memory_order_release);
		}
	}
}

static uint64_t r_event(cis_t *c, uint32_t ec)
{
	c->r_ev = extend64(c->r_ev, ec);
	atomic_store_explicit(&c->a_ev, ec, memory_order_release);
	return c->r_ev;
}

static bool r_prepare_tx(void *arg, uint32_t event_counter, uint8_t subevent, cis_pdu_t *out)
{
	cis_t *c = arg;
	if (!atomic_load_explicit(&c->a_open, memory_order_acquire))
		return false;
	uint64_t ev = r_event(c, event_counter);
	r_flush(c, ev, subevent);
	const cis_cis_derived_t *d = &c->d;
	uint8_t seq = 0;
	c->r_tx_slot = TX_NULL;
	if (d->bn_c_to_p && c->r_tx_pn < (ev + 1u) * d->bn_c_to_p) {
		int8_t s = r_find_slot(c, c->r_tx_pn);
		if (s >= 0) {
			const tx_slot_t *t = &c->tx[s];
			if (!c->r_owns_seq)
				seq = (uint8_t)((c->r_sn ? CIS_HDR_SN : 0) | (c->r_nesn ? CIS_HDR_NESN : 0));
			out->hdr0 = (uint8_t)((t->pdu.hdr0 & 0x03u) | seq);
			out->len = t->pdu.len;
			memcpy(out->payload, t->pdu.payload, t->pdu.len);
			atomic_store_explicit(&c->tx[s].sent, 1, memory_order_relaxed);
			c->r_tx_slot = s;
			STAT_INC(c, tx_pdus);
			return true;
		}
		/* With FT > 1 an empty payload would wait at the head of the queue until its flush point and push every
		 * following SDU to its last chance. The padding PDU (ISOAL: length 0, LLID 0b01) gets acknowledged
		 * and frees the queue; the Peripheral reports a lost SDU from it - which is true. */
		if (d->ft_c_to_p > 1 && atomic_load_explicit(&c->a_tx_started, memory_order_acquire)) {
			if (!c->r_owns_seq)
				seq = (uint8_t)((c->r_sn ? CIS_HDR_SN : 0) | (c->r_nesn ? CIS_HDR_NESN : 0));
			out->hdr0 = (uint8_t)(CIS_LLID_START_CONT | seq);
			out->len = 0;
			c->r_tx_slot = TX_PADDING;
			return true;
		}
	}
	/* Null PDU: SN reserved (0), NESN acknowledges the Peripheral; CIE when both sides have finished the event */
	bool tx_done = d->bn_c_to_p == 0 || c->r_tx_pn >= (ev + 1u) * d->bn_c_to_p;
	bool rx_done = d->bn_p_to_c == 0 || c->r_rx_pn >= (ev + 1u) * d->bn_p_to_c;
	if (!c->r_owns_seq && c->r_nesn)
		seq = CIS_HDR_NESN;
	out->hdr0 = (uint8_t)(CIS_LLID_START_CONT | CIS_HDR_NPI | seq | (tx_done && rx_done ? CIS_HDR_CIE : 0));
	out->len = 0;
	return true;
}

static void r_rx_done(void *arg, uint32_t event_counter, uint8_t subevent, const cis_pdu_t *pdu, bool crc_ok,
		uint32_t rx_timestamp_us)
{
	cis_t *c = arg;
	(void)subevent;
	(void)rx_timestamp_us;
	if (!atomic_load_explicit(&c->a_open, memory_order_acquire))
		return;
	uint64_t ev = r_event(c, event_counter);
	int8_t sent = c->r_tx_slot;
	/* Subevents prepared ahead: the same payload goes out in every subevent, so the slot waits for
	 * an acknowledgement from any of them (it is released by an acknowledgement or the end of the event). */
	if (!c->e->caps.subevents_prepared_ahead)
		c->r_tx_slot = TX_NULL;
	if (!pdu)
		return;
	if (!crc_ok) {
		STAT_INC(c, rx_crc);
		return;
	}
	if (!c->r_rx_seen) {
		c->r_rx_seen = true;
		atomic_fetch_or_explicit(&c->a_flags, F_RX_SEEN, memory_order_release);
	}
	c->r_last_rx_ev = ev;
	cis_pdu_hdr_t h;
	cis_pdu_hdr_unpack(pdu->hdr0, &h);
	const cis_cis_derived_t *d = &c->d;

	/* acknowledgement of our PDU: the Peripheral toggled NESN */
	if (sent != TX_NULL && h.nesn != (c->r_sn != 0)) {
		if (sent >= 0) {
			atomic_store_explicit(&c->tx[sent].state, SLOT_ACKED, memory_order_release);
			STAT_INC(c, tx_acked);
		} else if (c->r_tx_pn >= c->tx_first_pn) {
			STAT_INC(c, tx_underrun);   /* padding after the stream started = no data from the host */
		}
		c->r_tx_pn++;
		c->r_sn ^= 1u;
		c->r_tx_slot = TX_NULL;   /* a second ack of the same payload in the next subevent changes nothing */
	}

	/* new data: the expected SN, a payload of this or an earlier event */
	if (h.npi || !d->bn_p_to_c || c->r_rx_pn >= (ev + 1u) * d->bn_p_to_c || h.sn != (c->r_nesn != 0))
		return;
	if (pdu->len > c->r_rx_max_len || h.llid > CIS_LLID_START_CONT) {
		/* invalid - no acknowledgement; the Peripheral will retransmit or the payload will be flushed */
		STAT_INC(c, rx_crc);
		return;
	}
	uint32_t wr = atomic_load_explicit(&c->rx_wr, memory_order_relaxed);
	uint32_t rd = atomic_load_explicit(&c->rx_rd, memory_order_acquire);
	if (wr - rd >= CIS_RX_SLOTS) {
		STAT_INC(c, rx_overflow);
		return;
	}
	rx_slot_t *s = &c->rx[wr % CIS_RX_SLOTS];
	s->pn = c->r_rx_pn;
	s->pdu.hdr0 = pdu->hdr0;
	s->pdu.len = pdu->len;
	memcpy(s->pdu.payload, pdu->payload, pdu->len);
	atomic_store_explicit(&c->rx_wr, wr + 1u, memory_order_release);
	c->r_rx_pn++;
	c->r_nesn ^= 1u;
	atomic_store_explicit(&c->a_rx_pn, (uint32_t)c->r_rx_pn, memory_order_release);
}

static void r_end_of_event(cis_t *c, uint32_t event_counter)
{
	uint64_t ev = r_event(c, event_counter);
	r_flush(c, ev, 0xFF);
	/* A slot published too late (the task stalled for longer than the margin of 2 events) has pn < r_tx_pn and
	 * nobody would release it - a leak of the slot and of the host HCI credits. */
	for (int8_t i = 0; i < CIS_TX_SLOTS_MAX; i++) {
		if (atomic_load_explicit(&c->tx[i].state, memory_order_acquire) == SLOT_READY && c->tx[i].pn < c->r_tx_pn) {
			atomic_store_explicit(&c->tx[i].state, SLOT_FLUSHED, memory_order_release);
			STAT_INC(c, tx_flushed);
		}
	}
	STAT_INC(c, events);
	if (!c->r_rx_seen) {
		if (ev + 1u >= CIS_EST_EVENTS)
			atomic_fetch_or_explicit(&c->a_flags, F_EST_TIMEOUT, memory_order_release);
	} else if ((ev - c->r_last_rx_ev) * (uint64_t)c->e->der.iso_interval_us >= CIS_SUPERVISION_US) {
		atomic_fetch_or_explicit(&c->a_flags, F_SUPERVISION, memory_order_release);
	}
}

static void r_event_closed(void *arg, uint32_t event_counter)
{
	cis_t *c = arg;
	if (!atomic_load_explicit(&c->a_open, memory_order_acquire))
		return;
	c->r_tx_slot = TX_NULL;   /* subevents prepared ahead: the slot lived until the end of the event */
	r_end_of_event(c, event_counter);
}

static void r_event_skipped(void *arg, uint32_t event_counter)
{
	cis_t *c = arg;
	if (!atomic_load_explicit(&c->a_open, memory_order_acquire))
		return;
	STAT_INC(c, events_skipped);
	c->r_tx_slot = TX_NULL;
	r_end_of_event(c, event_counter);
}

static const cis_link_cb_t k_link_cb = {
	.prepare_tx = r_prepare_tx,
	.rx_done = r_rx_done,
	.event_closed = r_event_closed,
	.event_skipped = r_event_skipped,
};

/* ======================================================================== task: CIS life cycle */

static void arm_timer(cis_engine_t *e)
{
	uint32_t now = e->ops.now_us(e->ops.ctx);
	bool any = false;
	uint32_t best = 0;
	for (int i = 0; i < CIS_MAX_CIS; i++) {
		cis_t *c = &e->cis[i];
		if (!c->deadline_on)
			continue;
		int32_t left = (int32_t)(c->deadline - now);
		uint32_t d = left > 0 ? (uint32_t)left : 0;
		if (!any || d < best)
			best = d;
		any = true;
	}
	if (any)
		e->ops.timer_start(e->ops.ctx, best);
}

static void clear_data(cis_t *c)
{
	for (int i = 0; i < CIS_TX_SLOTS_MAX; i++) {
		atomic_store_explicit(&c->tx[i].state, SLOT_FREE, memory_order_relaxed);
		atomic_store_explicit(&c->tx[i].sent, 0, memory_order_relaxed);
	}
	atomic_store_explicit(&c->rx_wr, 0, memory_order_relaxed);
	atomic_store_explicit(&c->rx_rd, 0, memory_order_relaxed);
	atomic_store_explicit(&c->a_flags, 0, memory_order_relaxed);
	atomic_store_explicit(&c->a_tx_started, false, memory_order_relaxed);
	atomic_store_explicit(&c->a_ev, 0, memory_order_relaxed);
	atomic_store_explicit(&c->a_rx_pn, 0, memory_order_relaxed);
	c->dp_in = c->dp_out = false;
	c->t_ev = 0;
	c->t_started = false;
	c->t_next_ev = 0;
	c->t_rx_next = 0;
	c->nocp = 0;
	c->tx_sync_valid = false;
	c->h_active = false;
	c->enc = false;
}

static void close_link(cis_engine_t *e, cis_t *c)
{
	if (c->link) {
		atomic_store_explicit(&c->a_open, false, memory_order_release);
		e->ops.link_close(e->ops.ctx, c->link);
		c->link = NULL;
		bool any = false;
		for (int i = 0; i < CIS_MAX_CIS; i++)
			any = any || e->cis[i].link != NULL;
		if (!any) {
			e->cig_ref_valid = false;
			e->map_valid = false;
			e->late_run = 0;
			e->high_run = 0;
		}
	}
	memset(&c->ccm, 0, sizeof(c->ccm));
	clear_data(c);
	c->deadline_on = false;
	c->cancel = false;
	c->st = CS_CONFIGURED;
}

static void send_established(cis_engine_t *e, cis_t *c, uint8_t status)
{
	cis_hci_cis_established_t ev;
	memset(&ev, 0, sizeof(ev));
	ev.status = status;
	ev.handle = c->handle;
	if (status == CIS_ERR_SUCCESS) {
		ev.cig_sync_delay_us = e->der.cig_sync_delay_us;
		ev.cis_sync_delay_us = c->d.cis_sync_delay_us;
		ev.tl_c_to_p_us = c->d.tl_c_to_p_us;
		ev.tl_p_to_c_us = c->d.tl_p_to_c_us;
		ev.phy_c_to_p = c->d.phy_c_to_p;
		ev.phy_p_to_c = c->d.phy_p_to_c;
		ev.nse = c->d.nse;
		ev.bn_c_to_p = c->d.bn_c_to_p;
		ev.bn_p_to_c = c->d.bn_p_to_c;
		ev.ft_c_to_p = c->d.ft_c_to_p;
		ev.ft_p_to_c = c->d.ft_p_to_c;
		ev.max_pdu_c_to_p = c->d.max_pdu_c_to_p;
		ev.max_pdu_p_to_c = c->d.max_pdu_p_to_c;
		ev.iso_interval = e->der.iso_interval;
	}
	uint8_t b[CIS_HCI_CIS_ESTABLISHED_LEN];
	to_host(e, b, cis_hci_evt_cis_established(b, &ev));
	tr(e, c, CIS_TR_ESTABLISHED, status, st_ld(&c->s.events), st_ld(&c->s.rx_ok), st_ld(&c->s.rx_crc), 0);
}

/* Setup failed: cleanup and CIS Established with an error. */
static void fail_create(cis_engine_t *e, cis_t *c, uint8_t status)
{
	/* code 0 from a Peripheral PDU must not turn a failure into a success for the host */
	if (status == CIS_ERR_SUCCESS)
		status = CIS_ERR_UNSPECIFIED;
	close_link(e, c);
	send_established(e, c, status);
	arm_timer(e);
}

/* Loss of an established CIS: cleanup and Disconnection Complete. */
static void drop_established(cis_engine_t *e, cis_t *c, uint8_t reason)
{
	if (reason == CIS_ERR_SUCCESS)
		reason = CIS_ERR_UNSPECIFIED;
	close_link(e, c);
	uint8_t b[8];
	to_host(e, b, cis_hci_evt_disconn_complete(b, CIS_ERR_SUCCESS, c->handle, reason));
	tr(e, c, CIS_TR_END, reason, st_ld(&c->s.events), st_ld(&c->s.rx_ok), st_ld(&c->s.rx_crc),
	   st_ld(&c->s.rx_mic_fail));
}

/* Local teardown (MIC, supervision, no setup): the Peripheral gets LL_CIS_TERMINATE_IND so that it does not keep
 * transmitting until its own timeout. */
static void send_term(cis_engine_t *e, cis_t *c, uint8_t reason)
{
	cis_ll_cis_term_t term = { e->cig.cig_id, c->cfg.cis_id, reason };
	uint8_t b[CIS_LL_CIS_TERM_LEN];
	cis_ll_cis_term_pack(&term, b);
	(void)e->ops.send_ctrl_pdu(e->ops.ctx, c->acl, b, sizeof(b));
}

/* Ends a CIS in any state except CONFIGURED - picks the right event for the host. */
static void end_cis(cis_engine_t *e, cis_t *c, uint8_t reason)
{
	if (c->st == CS_ESTABLISHED)
		drop_established(e, c, reason);
	else if (c->st == CS_WAIT_SEND || c->st == CS_WAIT_RSP || c->st == CS_ESTABLISHING)
		fail_create(e, c, reason);
}

static bool acl_busy(const cis_engine_t *e, uint16_t acl)
{
	for (int i = 0; i < CIS_MAX_CIS; i++)
		if (e->cis[i].st == CS_WAIT_RSP && e->cis[i].acl == acl)
			return true;
	return false;
}

static void send_reject(cis_engine_t *e, uint16_t acl, uint8_t err)
{
	uint8_t b[CIS_LL_REJECT_EXT_LEN];
	cis_ll_reject_ext_pack(CIS_LL_CIS_RSP, err, b);
	(void)e->ops.send_ctrl_pdu(e->ops.ctx, acl, b, sizeof(b));
}

/* Margin after the ACL anchor: the event length from the platform (event_len_us), no less than CIS_ACL_GUARD_US. */
static uint32_t acl_guard_us(const cis_acl_timing_t *t)
{
	return t->event_len_us > CIS_ACL_GUARD_US ? t->event_len_us : CIS_ACL_GUARD_US;
}

static bool cig_ref_now(cis_engine_t *e, const cis_t *except, uint32_t *ref);
static bool choose_instant(cis_engine_t *e, cis_t *c, const cis_acl_timing_t *t, const cis_ll_cis_rsp_t *rsp,
		uint16_t *ev_out, uint32_t *off_out, uint32_t *anchor_out);

/* Overlap of [s, s + len) with [t, t + tlen) on a circle of length p (us). */
static uint32_t circ_overlap(uint32_t s, uint32_t len, uint32_t t, uint32_t tlen, uint32_t p)
{
	uint32_t sum = 0;
	s %= p;
	t %= p;
	for (int k = -1; k <= 1; k++) {
		const int64_t a0 = (int64_t)s, a1 = a0 + len;
		const int64_t b0 = (int64_t)t + (int64_t)k * p, b1 = b0 + tlen;
		const int64_t lo = a0 > b0 ? a0 : b0, hi = a1 < b1 ? a1 : b1;
		if (hi > lo)
			sum += (uint32_t)(hi - lo);
	}
	return sum;
}

/* Exact CIS_Offset for the CIS that establishes the CIG phase, at which the ACL events of all CISes of this CIG
 * (from the radio preparation before the anchor to the end of the event) do not fall into CIS events (from the
 * anchor to the next CIS: the C6 reserves a separate schedule event for each). The ACL phase is chosen by the
 * controller at connection, so with two earbuds the ACL of the second one landed anywhere - on the Tab5 1.4 ms after
 * the CIS anchor of the first: every 30 ms one of them was lost, and the left one dropped its ACL (0x08). The
 * smallest offset without a collision (earbuds prefer short ones), otherwise the smallest collision. false = a
 * single ACL or the ACL interval is not a multiple of ISO - the existing window stays. */
static bool place_cig(cis_engine_t *e, const cis_t *c, const cis_acl_timing_t *t, uint16_t ev_req, uint32_t omin,
		uint32_t omax, uint32_t *best)
{
	const uint32_t iso = e->der.iso_interval_us;
	if (iso == 0 || t->interval_us % iso != 0)
		return false;
	const uint32_t a_this = t->anchor_us +
		(uint32_t)((int32_t)(int16_t)(uint16_t)(ev_req - t->event_counter) * (int32_t)t->interval_us);
	uint32_t acl_start[CIS_MAX_CIS], acl_len[CIS_MAX_CIS];
	uint16_t acl_h[CIS_MAX_CIS];
	int nacl = 0;
	for (int j = 0; j < CIS_MAX_CIS; j++) {
		const cis_t *o = &e->cis[j];
		if (o->st == CS_FREE || (o->st == CS_CONFIGURED && o != c))
			continue;
		bool dup = false;
		for (int k = 0; k < nacl; k++)
			dup = dup || acl_h[k] == o->acl;
		cis_acl_timing_t tj;
		if (dup || e->ops.acl_timing(e->ops.ctx, o->acl, &tj) != 0 || tj.interval_us % iso != 0)
			continue;
		int32_t r = (int32_t)(tj.anchor_us - a_this) % (int32_t)iso;
		if (r < 0)
			r += (int32_t)iso;
		acl_h[nacl] = o->acl;
		acl_start[nacl] = ((uint32_t)r + iso - CIS_ACL_PREP_US) % iso;
		acl_len[nacl] = acl_guard_us(&tj) + CIS_ACL_PREP_US + e->caps.acl_margin_us;
		nacl++;
	}
	if (nacl < 2)
		return false;
	const uint32_t ph_this = e->der.cig_sync_delay_us - c->d.cis_sync_delay_us;
	uint32_t best_cost = UINT32_MAX, best_off = omin;
	for (uint32_t off = omin; off <= omax && off < omin + iso; off += CIS_PLACE_STEP_US) {
		uint32_t cost = 0;
		for (int k = 0; k < CIS_MAX_CIS; k++) {
			const cis_t *o = &e->cis[k];
			if (o->st == CS_FREE)
				continue;
			const uint32_t ph_k = e->der.cig_sync_delay_us - o->d.cis_sync_delay_us;
			const uint32_t start = (off + iso + ph_k - ph_this) % iso;
			const uint32_t len = cis_slot_us(o->d.nse, o->d.se_length_us, e->caps.subevent_gap_us) +
					     e->caps.cis_gap_us;
			for (int a = 0; a < nacl; a++)
				cost += circ_overlap(start, len, acl_start[a], acl_len[a], iso);
		}
		if (cost < best_cost) {
			best_cost = cost;
			best_off = off;
			if (cost == 0)
				break;
		}
	}
	*best = best_off;
	return true;
}

/* Sends the pending LL_CIS_REQ when the ACL is free and the data length allows. One at a time in a CIG (also on
 * different ACLs): the first CIS being set up fixes the CIG phase, and the next gets a window already in that phase
 * - with two at once the response of the second could arrive first and the narrow window of the first
 * (CIS_Offset_Max within one ISO) missed the phase (0x20). The next one goes out from on_cis_rsp / a rejection / a
 * timeout (try_send_pending) or from cis_engine_process. */
static void try_send_pending(cis_engine_t *e)
{
	for (int j = 0; j < CIS_MAX_CIS; j++)
		if (e->cis[j].st == CS_WAIT_RSP)
			return;
	for (int i = 0; i < CIS_MAX_CIS; i++) {
		cis_t *c = &e->cis[i];
		if (c->st != CS_WAIT_SEND || acl_busy(e, c->acl))
			continue;
		acl_t *a = acl_get(e, c->acl, true);
		if (a && (a->dl_blocked || (a->max_tx_octets && a->max_tx_octets < CIS_LL_CIS_REQ_LEN)))
			continue;
		cis_acl_timing_t t;
		if (e->ops.acl_timing(e->ops.ctx, c->acl, &t) != 0) {
			fail_create(e, c, CIS_ERR_UNKNOWN_CONN_ID);
			continue;
		}
		uint32_t guard = acl_guard_us(&t);
		uint32_t omin = CIS_OFFSET_MIN_US > guard ? CIS_OFFSET_MIN_US : guard;
		/* CIS_Offset_Max < connInterval - CIG_Sync_Delay - margin: the whole CIG before the next ACL event, with
		 * room for the radio preparation before its anchor (the C6 programs the start of a schedule element before
		 * the anchor). */
		uint32_t tail = e->der.cig_sync_delay_us + CIS_ACL_GUARD_US;
		uint32_t omax;
		if (t.interval_us > omin + tail)
			omax = t.interval_us - tail;
		else
			omax = t.interval_us > omin ? t.interval_us - 1u : omin;
		/* ACL interval a multiple of ISO: the ACL anchor always falls in the same phase of the ISO interval, so
		 * collisions of all events (not only the first) are decided by CIS_Offset mod ISO - the CIG must fit in one
		 * ISO interval after the ACL event. Otherwise, with an offset from the end of the window, every few events a
		 * CIS hits the ACL. Only the CIS that establishes the CIG phase: no other one is waiting for LL_CIS_RSP or
		 * being set up / established (those define the CIG phase or are about to - choose_instant), and their ACLs
		 * have their own phase. */
		uint32_t iso = e->der.iso_interval_us;
		bool sets_phase = true;
		for (int j = 0; j < CIS_MAX_CIS; j++) {
			cis_state_t s = e->cis[j].st;
			if (j != i && (s == CS_WAIT_RSP || s == CS_ESTABLISHING || s == CS_ESTABLISHED))
				sets_phase = false;
		}
		if (sets_phase && iso != 0 && t.interval_us % iso == 0 && iso > omin + tail && omax > iso - tail)
			omax = iso - tail;
		uint16_t ev_req = (uint16_t)(t.event_counter + CIS_INSTANT_MARGIN + t.peripheral_latency);
		/* The window we accept in LL_CIS_RSP: for the phase-establishing CIS the whole one (when the Peripheral does
		 * not accept our exact offset, it plays as before), for a joining one - a single value in the CIG phase. */
		uint32_t acc_min = omin, acc_max = omax;
		uint32_t placed;
		if (sets_phase && place_cig(e, c, &t, ev_req, omin, omax, &placed))
			omin = omax = placed;
		/* A CIS joining a running CIG: the CIG already knows the phase, so the window is a single value in that
		 * phase (min = max), like Zephyr (ull_central_iso.c, mfy_cis_offset_get). The Peripheral may narrow the full
		 * window to any value (Galaxy Buds3 Pro: always 1312 us after its ACL event), and that one usually falls
		 * outside the CIG phase (0x20). */
		uint32_t ref;
		if (!sets_phase && cig_ref_now(e, c, &ref)) {
			cis_ll_cis_rsp_t want = { .cis_offset_min_us = omin, .cis_offset_max_us = omax,
				.conn_event_count = ev_req };
			uint16_t ev;
			uint32_t off, anchor;
			if (choose_instant(e, c, &t, &want, &ev, &off, &anchor)) {
				omin = omax = off;
				ev_req = ev;
			}
			acc_min = omin;
			acc_max = omax;
		}
		cis_ll_cis_req_t r = {
			.cig_id = e->cig.cig_id, .cis_id = c->cfg.cis_id,
			.phy_c_to_p = c->d.phy_c_to_p, .phy_p_to_c = c->d.phy_p_to_c,
			.max_sdu_c_to_p = c->d.max_sdu_c_to_p, .max_sdu_p_to_c = c->d.max_sdu_p_to_c, .framed = false,
			.sdu_interval_c_to_p = e->cig.sdu_interval_c_to_p, .sdu_interval_p_to_c = e->cig.sdu_interval_p_to_c,
			.max_pdu_c_to_p = c->d.max_pdu_c_to_p, .max_pdu_p_to_c = c->d.max_pdu_p_to_c,
			.nse = c->d.nse, .sub_interval_us = c->d.sub_interval_us,
			.bn_c_to_p = c->d.bn_c_to_p, .bn_p_to_c = c->d.bn_p_to_c,
			.ft_c_to_p = c->d.ft_c_to_p, .ft_p_to_c = c->d.ft_p_to_c,
			.iso_interval = e->der.iso_interval,
			.cis_offset_min_us = omin, .cis_offset_max_us = omax, .conn_event_count = ev_req,
		};
		uint8_t b[CIS_LL_CIS_REQ_LEN];
		cis_ll_cis_req_pack(&r, b);
		int rc = e->ops.send_ctrl_pdu(e->ops.ctx, c->acl, b, sizeof(b));
		const uint32_t tr_ev = t.event_counter | ((uint32_t)r.conn_event_count << 16);
		if (rc == CIS_PLAT_ERR_TOO_LONG) {
			/* The C6 splits PDUs to the ACL MaxTxOctets: we wait for LE Data Length Change >= 36 (or the procedure
			 * timeout) */
			if (a)
				a->dl_blocked = true;
			elog(e, LOG_WARN, "cis: LL_CIS_REQ waiting for LE Data Length >= 36");
			tr(e, c, CIS_TR_REQ, 1, tr_ev, t.anchor_us, omin, omax);
			continue;
		}
		if (rc != 0) {
			fail_create(e, c, CIS_ERR_UNSPECIFIED);
			continue;
		}
		tr(e, c, CIS_TR_REQ, 0, tr_ev, t.anchor_us, omin, omax);
		c->st = CS_WAIT_RSP;
		c->req_omin = acc_min;
		c->req_omax = acc_max;
		/* the response timeout counts from sending LL_CIS_REQ, not from Create CIS (waiting for the data length) */
		c->deadline = e->ops.now_us(e->ops.ctx) + CIS_PROC_TIMEOUT_US;
		c->deadline_on = true;
		return;   /* one LL_CIS_REQ at a time */
	}
}

/* CIG reference point (the anchor of a CIG event from which the CIS phases are counted) from any open CIS. */
static bool cig_ref_now(cis_engine_t *e, const cis_t *except, uint32_t *ref)
{
	for (int i = 0; i < CIS_MAX_CIS; i++) {
		cis_t *o = &e->cis[i];
		if (o == except || !o->link)
			continue;
		o->t_ev = extend64(o->t_ev, atomic_load_explicit(&o->a_ev, memory_order_acquire));
		uint32_t phase = e->der.cig_sync_delay_us - o->d.cis_sync_delay_us;
		*ref = o->anchor0 + (uint32_t)(o->t_ev * e->der.iso_interval_us) - phase;
		return true;
	}
	return false;
}

/* Choice of the instant (connEventCount) and CIS_Offset: within the LL_CIS_RSP window (counted from the chosen ACL
 * event), outside the ACL event, and for a subsequent CIS in the CIG - in the CIG phase (anchor = ref + n x ISO +
 * CIS phase), so that the CISes of the CIG do not overlap. */
static bool choose_instant(cis_engine_t *e, cis_t *c, const cis_acl_timing_t *t, const cis_ll_cis_rsp_t *rsp,
		uint16_t *ev_out, uint32_t *off_out, uint32_t *anchor_out)
{
	uint16_t min_ev = (uint16_t)(t->event_counter + CIS_INSTANT_MARGIN + t->peripheral_latency);
	uint16_t ev = rsp->conn_event_count;
	if ((int16_t)(ev - min_ev) < 0)
		ev = min_ev;
	uint32_t lo = rsp->cis_offset_min_us, hi = rsp->cis_offset_max_us;
	uint32_t guard = acl_guard_us(t);
	if (lo < guard && hi >= guard)
		lo = guard;
	uint32_t ref = 0;
	bool have_ref = cig_ref_now(e, c, &ref);
	uint32_t iso = e->der.iso_interval_us;
	uint32_t phase = e->der.cig_sync_delay_us - c->d.cis_sync_delay_us;
	for (int k = 0; k < CIS_INSTANT_SEARCH; k++, ev++) {
		int32_t dev = (int16_t)(ev - t->event_counter);
		uint32_t acl_anchor = t->anchor_us + (uint32_t)(dev * (int32_t)t->interval_us);
		uint32_t off = lo;
		if (have_ref) {
			int32_t dlt = (int32_t)(ref + phase - (acl_anchor + lo));
			int32_t r = dlt % (int32_t)iso;
			if (r < 0)
				r += (int32_t)iso;
			off = lo + (uint32_t)r;
			if (off > hi)
				continue;
		}
		*ev_out = ev;
		*off_out = off;
		*anchor_out = acl_anchor + off;
		return true;
	}
	return false;
}

static void on_cis_rsp(cis_engine_t *e, cis_t *c, const cis_ll_cis_rsp_t *rsp)
{
	c->deadline_on = false;
	if (c->cancel) {
		send_reject(e, c->acl, CIS_ERR_OP_CANCELLED_BY_HOST);
		fail_create(e, c, CIS_ERR_OP_CANCELLED_BY_HOST);
		return;
	}
	if (rsp->cis_offset_min_us > rsp->cis_offset_max_us || rsp->cis_offset_min_us < CIS_OFFSET_MIN_US) {
		send_reject(e, c->acl, 0x1E);   /* Invalid LL Parameters */
		fail_create(e, c, 0x1E);
		return;
	}
	cis_acl_timing_t t;
	if (e->ops.acl_timing(e->ops.ctx, c->acl, &t) != 0) {
		fail_create(e, c, CIS_ERR_UNKNOWN_CONN_ID);
		return;
	}
	/* The Peripheral window clipped to ours from LL_CIS_REQ: outside it the CIG overlaps ACL events. */
	cis_ll_cis_rsp_t win = *rsp;
	if (win.cis_offset_min_us < c->req_omin)
		win.cis_offset_min_us = c->req_omin;
	if (win.cis_offset_max_us > c->req_omax)
		win.cis_offset_max_us = c->req_omax;
	/* Buds3 Pro writes the exact offset beyond the ISO interval modulo ISO (10694 -> 694) - the same instant in the
	 * CIG phase */
	const uint32_t iso = e->der.iso_interval_us;
	if (win.cis_offset_min_us > win.cis_offset_max_us && iso != 0 && c->req_omin == c->req_omax &&
	    rsp->cis_offset_min_us == rsp->cis_offset_max_us && rsp->cis_offset_min_us >= CIS_OFFSET_MIN_US) {
		const uint32_t d = c->req_omin > rsp->cis_offset_min_us ? c->req_omin - rsp->cis_offset_min_us
								 : rsp->cis_offset_min_us - c->req_omin;
		if (d % iso == 0)
			win = *rsp;
	}
	uint16_t ev;
	uint32_t off, anchor;
	if (win.cis_offset_min_us > win.cis_offset_max_us || !choose_instant(e, c, &t, &win, &ev, &off, &anchor)) {
		send_reject(e, c->acl, CIS_ERR_UNSUPP_LL_PARAM);
		fail_create(e, c, CIS_ERR_UNSUPP_LL_PARAM);
		return;
	}
	cis_ll_cis_ind_t ind = { .access_address = c->aa, .cis_offset_us = off,
		.cig_sync_delay_us = e->der.cig_sync_delay_us, .cis_sync_delay_us = c->d.cis_sync_delay_us,
		.conn_event_count = ev };
	uint8_t b[CIS_LL_CIS_IND_LEN];
	cis_ll_cis_ind_pack(&ind, b);
	if (e->ops.send_ctrl_pdu(e->ops.ctx, c->acl, b, sizeof(b)) != 0) {
		fail_create(e, c, CIS_ERR_UNSPECIFIED);
		return;
	}
	tr(e, c, CIS_TR_IND, 0, ev | ((uint32_t)t.event_counter << 16), t.anchor_us, off, t.interval_us);

	/* CIS key = ACL SK, IV = ACL IV XOR AA (cis_crypto.h). CIS encryption follows the ACL state at the moment of
	 * setup - the same as in Zephyr ull_conn_iso.c. */
	acl_t *a = acl_get(e, c->acl, false);
	c->enc = a && a->enc;
	atomic_store_explicit(&c->s.key_src, c->enc ? a->key_src : (uint32_t)CIS_KEY_NONE, memory_order_relaxed);
	if (c->enc) {
		c->ccm.aes = (cis_aes128_fn)e->ops.aes128;
		c->ccm.aes_ctx = e->ops.ctx;
		memcpy(c->ccm.sk, a->sk, 16);
		cis_crypto_cis_iv(a->iv, c->aa, c->ccm.iv);
	}
	uint8_t mic = c->enc ? CIS_MIC_LEN : 0;

	/* radio state before opening: the first call may arrive right after link_open */
	c->r_ev = 0;
	c->r_tx_pn = 0;
	c->r_rx_pn = 0;
	c->r_sn = 0;
	c->r_nesn = 0;
	c->r_tx_slot = TX_NULL;
	c->r_rx_seen = false;
	c->r_last_rx_ev = 0;
	refresh_caps(e);
	c->r_owns_seq = e->caps.radio_owns_seq;
	c->r_rx_max_len = (uint8_t)(c->d.max_pdu_p_to_c + mic);
	clear_data(c);
	c->enc = a && a->enc;
	c->anchor0 = anchor;
	/* CIG anchor of this CIS (its event 0) - choose_instant keeps it in the CIG phase, so the difference is whole
	 * intervals */
	const uint32_t cig_anchor = anchor - (e->der.cig_sync_delay_us - c->d.cis_sync_delay_us);
	if (!e->cig_ref_valid) {
		e->cig_ref_valid = true;
		e->cig_ref_us = cig_anchor;
	}
	c->cig_off = (cig_anchor - e->cig_ref_us + e->der.iso_interval_us / 2u) / e->der.iso_interval_us;
	atomic_store_explicit(&c->a_open, true, memory_order_release);

	cis_link_params_t lp;
	memset(&lp, 0, sizeof(lp));
	lp.access_address = c->aa;
	lp.crc_init = t.crc_init & 0xFFFFFFu;
	lp.phy_c_to_p = c->d.phy_c_to_p;
	lp.phy_p_to_c = c->d.phy_p_to_c;
	memcpy(lp.chan_map, t.chan_map, 5);
	lp.nse = c->d.nse;
	lp.sub_interval_us = c->d.sub_interval_us;
	lp.iso_interval_us = e->der.iso_interval_us;
	lp.first_anchor_us = anchor;
	lp.max_pdu_c_to_p = (uint8_t)(c->d.max_pdu_c_to_p + (c->d.bn_c_to_p ? mic : 0));
	lp.max_pdu_p_to_c = (uint8_t)(c->d.max_pdu_p_to_c + (c->d.bn_p_to_c ? mic : 0));
	cis_link_t *link = NULL;
	if (e->ops.link_open(e->ops.ctx, &lp, &k_link_cb, c, &link) != 0 || !link) {
		atomic_store_explicit(&c->a_open, false, memory_order_release);
		tr(e, c, CIS_TR_OPEN, 1, anchor, 0, 0, 0);
		send_term(e, c, CIS_ERR_LIMITED_RESOURCES);
		fail_create(e, c, CIS_ERR_LIMITED_RESOURCES);
		return;
	}
	tr(e, c, CIS_TR_OPEN, 0, anchor, 0, 0, 0);
	c->link = link;
	c->st = CS_ESTABLISHING;
	try_send_pending(e);
}

/* ======================================================================== task: data */

/* The jitter buffer from cis_cig_derive clamped to the queue: occupancy up to BN x (2 x depth + 2) slots. */
static uint8_t jitter_depth(const cis_cis_derived_t *d)
{
	const uint8_t bn = d->bn_c_to_p ? d->bn_c_to_p : 1u;
	const uint8_t cap = (uint8_t)((CIS_TX_SLOTS / bn - 2u) / 2u);
	const uint8_t want = d->jitter_c_to_p ? d->jitter_c_to_p : CIS_JITTER_MIN;
	return want < cap ? want : cap;
}

/* Used payload queue slots (cis_radio_caps_t.tx_slots, 0 = CIS_TX_SLOTS). When it changes during a stream,
 * releasing and flushing scan the whole array anyway - only the number of free slots a new SDU takes changes. */
static uint8_t tx_slots(const cis_engine_t *e)
{
	const uint8_t s = e->caps.tx_slots ? e->caps.tx_slots : (uint8_t)CIS_TX_SLOTS;
	return s < CIS_TX_SLOTS ? (uint8_t)CIS_TX_SLOTS : s > CIS_TX_SLOTS_MAX ? (uint8_t)CIS_TX_SLOTS_MAX : s;
}

/* SDU from the host -> BN payloads encrypted ahead of time for a future event. */
static void enqueue_sdu(cis_t *c, const uint8_t *sdu, uint16_t len, uint16_t psn, uint8_t pkts)
{
	const cis_cis_derived_t *d = &c->d;
	const uint8_t depth = jitter_depth(d);
	c->nocp = (uint16_t)(c->nocp + pkts);   /* returned immediately by default; on success the slot returns it */
	if (!c->dp_in || d->bn_c_to_p == 0 || len > d->max_sdu_c_to_p) {
		STAT_INC(c, sdu_dropped);
		return;
	}
	c->t_ev = extend64(c->t_ev, atomic_load_explicit(&c->a_ev, memory_order_acquire));
	uint64_t cur = c->t_ev;
	cis_engine_t *e = c->e;
	/* Event from the CIG mapping by PSN (the same L/R SDU in the same interval on every CIS). Event cur+1 may already
	 * be in preparation (tx_prepare_lead_us) - cur+2 at the earliest; a late SDU (host stall, clock drift) gives
	 * a new mapping with a jitter buffer - for the whole CIG, so all CISes shift together. */
	uint64_t want = cur + 1u + depth;
	if (e->map_valid) {
		const int64_t g = (int64_t)e->map_gev + (int16_t)(psn - e->map_psn) - (int64_t)c->cig_off;
		if (psn != e->win_psn) {
			e->win_psn = psn;
			if (++e->win_sdus >= CIS_DRIFT_WINDOW) {
				e->win_sdus = 0;
				e->win_late = 0;
			}
		}
		if (g >= (int64_t)(cur + 2u)) {
			want = (uint64_t)g;
			e->late_run = 0;
		} else {
			STAT_INC(c, sdu_late);
			if (e->late_run == 0 || psn != e->late_psn) {
				/* The window counts episodes (the first late one after an on-time one): drift is lateness interleaved
				 * with on-time SDUs, and a series from one host stall stays on the old mapping (test_single_late_*) */
				if (++e->late_run == 1)
					e->win_late++;
			}
			e->late_psn = psn;
			if (e->late_run < CIS_REMAP_LATE) {
				/* the late SDU is dropped (the earbud conceals it), the mapping stays - on-time SDUs stay in place */
				e->map_psn = psn;
				e->map_gev = (uint64_t)((int64_t)c->cig_off + g);
				if (e->win_late >= CIS_DRIFT_LATE) {
					/* drift: the following SDUs one event later (once per window - the queue grows by one, not by a
					 * buffer) */
					e->map_gev++;
					e->win_late = 0;
					e->win_sdus = 0;
				}
				return;
			}
			e->late_run = 0;   /* lasting shift of the host numbering: a new mapping (want = cur + 1 + buffer) */
		}
	}
	e->map_valid = true;
	e->map_psn = psn;
	e->map_gev = c->cig_off + want;
	if (want > cur + 1u + 2u * depth) {
		/* Buffer full (a series of backlogged SDUs after a host stall, a faster host clock): the SDU is dropped and
		 * does not take an event - the next one goes into the same place. Without stepping the mapping back every
		 * following SDU would also be too far ahead and the sound would vanish permanently. */
		e->map_gev--;
		STAT_INC(c, sdu_dropped);
		return;
	}
	if (c->t_started && want < c->t_next_ev) {
		STAT_INC(c, sdu_dropped);   /* event already taken (another CIS moved the mapping back) */
		return;
	}
	/* Queue target: 1 + jitter buffer (+1 for the host phase relative to the events). Persistently above it - one
	 * SDU is dropped for the whole CIG (the mapping steps back by an interval, the second CIS hits the occupied
	 * event), until the queue returns to the target. */
	if (want > cur + 2u + depth) {
		if (++e->high_run >= CIS_RECENTER_SDU) {
			e->high_run = 0;
			e->map_gev--;
			STAT_INC(c, sdu_dropped);
			return;
		}
	} else {
		e->high_run = 0;
	}
	int8_t slots[CIS_TX_SLOTS_MAX];
	uint8_t n = 0;
	const int8_t usable = (int8_t)tx_slots(e);
	for (int8_t i = 0; i < usable && n < d->bn_c_to_p; i++)
		if (atomic_load_explicit(&c->tx[i].state, memory_order_acquire) == SLOT_FREE)
			slots[n++] = i;
	if (n < d->bn_c_to_p) {
		STAT_INC(c, sdu_dropped);
		return;
	}
	uint64_t pn0 = want * d->bn_c_to_p;
	for (uint8_t i = 0; i < d->bn_c_to_p; i++) {
		tx_slot_t *s = &c->tx[slots[i]];
		uint16_t off;
		uint8_t flen, llid;
		if (!cis_unframed_fragment(len, d->max_pdu_c_to_p, d->bn_c_to_p, i, &off, &flen, &llid)) {
			STAT_INC(c, sdu_dropped);
			return;   /* cannot happen: len <= Max_SDU <= BN x Max_PDU; the slots are still FREE */
		}
		s->pdu.hdr0 = llid;
		/* A PDU of length 0 goes without MIC (Core; Zephyr: encrypts only pdu->len != 0) */
		if (c->enc && flen) {
			cis_ccm_encrypt(&c->ccm, pn0 + i, CIS_DIR_C_TO_P, (uint8_t)(llid & CIS_CCM_AAD_MASK), sdu + off, flen,
					s->pdu.payload);
			s->pdu.len = (uint8_t)(flen + CIS_MIC_LEN);
		} else {
			if (flen)
				memcpy(s->pdu.payload, sdu + off, flen);
			s->pdu.len = flen;
		}
		s->pn = pn0 + i;
		s->psn = psn;
		s->sdu_first = i == 0;
		s->hci_pkts = (uint8_t)(i == d->bn_c_to_p - 1u ? pkts : 0);
		atomic_store_explicit(&s->sent, 0, memory_order_relaxed);
	}
	c->nocp = (uint16_t)(c->nocp - pkts);
	if (!c->t_started) {
		c->tx_first_pn = pn0;
		atomic_store_explicit(&c->a_tx_started, true, memory_order_release);
		c->t_started = true;
	}
	/* publish only after all payloads of the SDU are written */
	for (uint8_t i = 0; i < d->bn_c_to_p; i++)
		atomic_store_explicit(&c->tx[slots[i]].state, SLOT_READY, memory_order_release);
	c->t_next_ev = want + 1u;
	STAT_INC(c, sdu_in);
}

static void reclaim_tx(cis_engine_t *e, cis_t *c)
{
	for (int i = 0; i < CIS_TX_SLOTS_MAX; i++) {
		tx_slot_t *s = &c->tx[i];
		uint8_t st = atomic_load_explicit(&s->state, memory_order_acquire);
		if (st != SLOT_ACKED && st != SLOT_FLUSHED)
			continue;
		c->nocp = (uint16_t)(c->nocp + s->hci_pkts);
		if (s->sdu_first && atomic_load_explicit(&s->sent, memory_order_relaxed)) {
			uint64_t ev = s->pn / c->d.bn_c_to_p;
			c->tx_sync_valid = true;
			c->tx_sync_psn = s->psn;
			/* CIG reference point of the event in which the SDU went out */
			c->tx_sync_ts = c->anchor0 + (uint32_t)(ev * e->der.iso_interval_us) -
					(e->der.cig_sync_delay_us - c->d.cis_sync_delay_us);
		}
		atomic_store_explicit(&s->state, SLOT_FREE, memory_order_release);
	}
}

/* One payload (received or lost) for SDU reassembly; after the last payload of an SDU - an HCI ISO packet. */
static void rx_feed(cis_engine_t *e, cis_t *c, uint64_t pn, uint8_t llid, const uint8_t *data, uint8_t len, bool lost)
{
	uint8_t bn = c->d.bn_p_to_c;
	uint8_t idx = (uint8_t)(pn % bn);
	if (idx == 0)
		cis_unframed_rx_begin(&c->u, c->u_buf, sizeof(c->u_buf));
	cis_unframed_rx_pdu(&c->u, llid, data, len, lost);
	if (idx != bn - 1u)
		return;
	uint8_t status = cis_unframed_rx_end(&c->u);
	if (!c->dp_out)
		return;
	uint64_t k = pn / bn;   /* SDU k = event k (ISO = SDU interval) */
	cis_hci_iso_t iso = {
		.handle = c->handle, .pb = CIS_ISO_PB_COMPLETE, .ts_flag = true,
		/* SDU synchronization point: CIS anchor + CIS_Sync_Delay = end of the CIG (Core Vol 6 Part G 3.2) */
		.timestamp = c->anchor0 + (uint32_t)(k * e->der.iso_interval_us) + c->d.cis_sync_delay_us,
		.psn = (uint16_t)k, .sdu_len = status == CIS_ISO_STATUS_LOST ? 0 : c->u.len, .status = status,
		.data = c->u_buf, .data_len = status == CIS_ISO_STATUS_LOST ? 0 : c->u.len,
	};
	uint16_t n = cis_hci_iso_build(&iso, e->iso_out, sizeof(e->iso_out));
	if (n)
		to_host(e, e->iso_out, n);
}

/* Payloads [c->t_rx_next, upto) with no entry in the queue = lost; a long gap is shortened to a few SDUs. */
static void rx_gap(cis_engine_t *e, cis_t *c, uint64_t upto)
{
	uint8_t bn = c->d.bn_p_to_c;
	while (c->t_rx_next < upto && c->t_rx_next % bn) {
		rx_feed(e, c, c->t_rx_next, 0, NULL, 0, true);
		c->t_rx_next++;
	}
	uint64_t keep = (uint64_t)CIS_LOST_REPORT_MAX * bn;
	if (upto - c->t_rx_next > keep + bn) {
		uint64_t skip_to = ((upto / bn) - CIS_LOST_REPORT_MAX) * bn;
		if (skip_to > c->t_rx_next)
			c->t_rx_next = skip_to;
	}
	while (c->t_rx_next < upto) {
		rx_feed(e, c, c->t_rx_next, 0, NULL, 0, true);
		c->t_rx_next++;
	}
}

/* false = CIS closed (MIC error). */
static bool drain_rx(cis_engine_t *e, cis_t *c)
{
	if (!c->d.bn_p_to_c)
		return true;
	uint64_t progress = extend64(c->t_rx_next, atomic_load_explicit(&c->a_rx_pn, memory_order_acquire));
	uint32_t rd = atomic_load_explicit(&c->rx_rd, memory_order_relaxed);
	uint32_t wr = atomic_load_explicit(&c->rx_wr, memory_order_acquire);
	while (rd != wr) {
		rx_slot_t *s = &c->rx[rd % CIS_RX_SLOTS];
		if (s->pn >= c->t_rx_next) {
			rx_gap(e, c, s->pn);
			uint8_t plain[CIS_PDU_MAX_PAYLOAD];
			uint8_t len = s->pdu.len;
			const uint8_t *data = s->pdu.payload;
			if (c->enc && len) {
				if (len < CIS_MIC_LEN || !cis_ccm_decrypt(&c->ccm, s->pn, CIS_DIR_P_TO_C,
						(uint8_t)(s->pdu.hdr0 & CIS_CCM_AAD_MASK), s->pdu.payload, len, plain)) {
					STAT_INC(c, rx_mic_fail);
					elog(e, LOG_ERR, "cis: MIC error - CIS closed");
					send_term(e, c, CIS_ERR_MIC_FAILURE);
					drop_established(e, c, CIS_ERR_MIC_FAILURE);
					return false;
				}
				len = (uint8_t)(len - CIS_MIC_LEN);
				data = plain;
			}
			STAT_INC(c, rx_ok);
			rx_feed(e, c, s->pn, s->pdu.hdr0 & 0x03u, data, len, false);
			c->t_rx_next = s->pn + 1u;
		}
		rd++;
		atomic_store_explicit(&c->rx_rd, rd, memory_order_release);
	}
	if (progress > c->t_rx_next)
		rx_gap(e, c, progress);
	return true;
}

static void send_nocp(cis_engine_t *e)
{
	uint16_t h[CIS_MAX_CIS], n[CIS_MAX_CIS];
	uint8_t k = 0;
	for (int i = 0; i < CIS_MAX_CIS; i++) {
		cis_t *c = &e->cis[i];
		if (c->nocp) {
			h[k] = c->handle;
			n[k++] = c->nocp;
			c->nocp = 0;
		}
	}
	if (k) {
		uint8_t b[4 + 4 * CIS_MAX_CIS];
		to_host(e, b, cis_hci_evt_nocp(b, h, n, k));
	}
}

/* ======================================================================== API: init */

uint32_t cis_engine_mem_size(void)
{
	return (uint32_t)(sizeof(struct cis_engine) + _Alignof(struct cis_engine));
}

static void reset_all(cis_engine_t *e)
{
	for (int i = 0; i < CIS_MAX_CIS; i++) {
		cis_t *c = &e->cis[i];
		if (c->link) {
			atomic_store_explicit(&c->a_open, false, memory_order_release);
			e->ops.link_close(e->ops.ctx, c->link);
			c->link = NULL;
		}
	}
	cis_platform_ops_t ops = e->ops;
	cis_radio_caps_t caps = e->caps;
	memset(e, 0, sizeof(*e));
	e->ops = ops;
	e->caps = caps;
	for (int i = 0; i < CIS_MAX_CIS; i++) {
		e->cis[i].e = e;
		e->cis[i].handle = (uint16_t)(CIS_HANDLE_BASE + i);
	}
}

cis_engine_t *cis_engine_init(void *mem, uint32_t mem_size, const cis_platform_ops_t *ops)
{
	if (!mem || !ops || mem_size < cis_engine_mem_size())
		return NULL;
	if (!ops->hci_to_host || !ops->send_ctrl_pdu || !ops->acl_timing || !ops->acl_is_central ||
	    !ops->acl_remote_features || !ops->link_open || !ops->link_close || !ops->radio_caps || !ops->now_us ||
	    !ops->aes128 || !ops->random || !ops->timer_start)
		return NULL;
	uintptr_t p = (uintptr_t)mem;
	uintptr_t al = _Alignof(struct cis_engine);
	p = (p + al - 1u) & ~(al - 1u);
	cis_engine_t *e = (cis_engine_t *)p;
	memset(e, 0, sizeof(*e));
	e->ops = *ops;
	refresh_caps(e);
	reset_all(e);
	return e;
}

bool cis_engine_active(const cis_engine_t *e)
{
	return e->cig_used;
}

bool cis_engine_get_stats(const cis_engine_t *e, uint16_t cis_handle, cis_engine_stats_t *out)
{
	const cis_t *c = cis_by_handle((cis_engine_t *)e, cis_handle);
	if (!c)
		return false;
#define LD(f) out->f = atomic_load_explicit(&c->s.f, memory_order_relaxed)
	LD(events);
	LD(events_skipped);
	LD(tx_pdus);
	LD(tx_acked);
	LD(tx_flushed);
	LD(tx_underrun);
	LD(sdu_in);
	LD(sdu_dropped);
	LD(sdu_late);
	LD(rx_ok);
	LD(rx_crc);
	LD(rx_mic_fail);
	LD(rx_lost);
	LD(rx_overflow);
	LD(key_src);
#undef LD
	return true;
}

/* ======================================================================== API: HCI from the host */

/* The radio capabilities are read on every use: the C6 adapter may bring the radio up later than the engine. */
static void refresh_caps(cis_engine_t *e)
{
	e->ops.radio_caps(e->ops.ctx, &e->caps);
	if (e->caps.max_nse == 0)
		e->caps.max_nse = 1;
}

/* Radio NSE for a CIG: with P->C data (microphone) at most caps.max_nse_p_to_c - the radio may have NSE > 1 verified
 * for C->P only (C6, docs/c6-link-layer.md). */
static uint8_t cig_max_nse(const cis_engine_t *e, const cis_hci_cig_params_t *cig)
{
	const uint8_t lim = e->caps.max_nse_p_to_c;
	if (lim == 0 || lim >= e->caps.max_nse)
		return e->caps.max_nse;
	for (uint8_t i = 0; i < cig->cis_count; i++)
		if (cig->cis[i].max_sdu_p_to_c)
			return lim;
	return e->caps.max_nse;
}

/* Lower bound of the jitter buffer from the radio (cis_radio_caps_t.jitter_min_sdu) above the one derived from
 * Max_Transport_Latency. */
static void apply_jitter_floor(const cis_engine_t *e, cis_cig_derived_t *d, uint8_t n)
{
	const uint8_t f = e->caps.jitter_min_sdu < CIS_JITTER_MAX ? e->caps.jitter_min_sdu : CIS_JITTER_MAX;
	for (uint8_t i = 0; i < n; i++)
		if (d->cis[i].jitter_c_to_p < f)
			d->cis[i].jitter_c_to_p = f;
}

static void hci_set_cig(cis_engine_t *e, const uint8_t *p, uint8_t plen)
{
	const uint16_t op = CIS_OP_LE_SET_CIG_PARAMS;
	refresh_caps(e);
	uint8_t ret[3 + 2 * CIS_MAX_CIS];
	cis_hci_cig_params_t in;
	uint8_t st = cis_hci_cig_params_parse(p, plen, &in);
	ret[1] = plen ? p[0] : 0;
	ret[2] = 0;
	if (st == CIS_ERR_SUCCESS && e->cig_used && in.cig_id != e->cig.cig_id)
		st = CIS_ERR_MEM_CAPACITY;    /* one CIG: that is all the memory we have */
	/* Merge with the existing configuration: CISes not in the command stay, those in the command overwrite by
	 * CIS_ID. */
	cis_hci_cig_params_t merged;
	uint8_t slot_of[CIS_MAX_CIS];
	if (st == CIS_ERR_SUCCESS) {
		merged = in;
		merged.cis_count = 0;
		for (int i = 0; i < CIS_MAX_CIS; i++) {
			if (e->cig_used && e->cis[i].st != CS_FREE) {
				if (e->cis[i].st != CS_CONFIGURED)
					st = CIS_ERR_CMD_DISALLOWED;
				merged.cis[merged.cis_count] = e->cis[i].cfg;
				slot_of[merged.cis_count++] = (uint8_t)i;
			}
		}
		for (uint8_t j = 0; j < in.cis_count && st == CIS_ERR_SUCCESS; j++) {
			int found = -1;
			for (uint8_t k = 0; k < merged.cis_count; k++)
				if (merged.cis[k].cis_id == in.cis[j].cis_id)
					found = k;
			for (uint8_t k = 0; k < j; k++)
				if (in.cis[k].cis_id == in.cis[j].cis_id)
					st = CIS_ERR_INVALID_PARAMS;
			if (found >= 0) {
				merged.cis[found] = in.cis[j];
				continue;
			}
			if (merged.cis_count >= CIS_MAX_CIS) {
				st = CIS_ERR_MEM_CAPACITY;
				break;
			}
			/* free slot */
			uint8_t free_slot = 0xFF;
			for (uint8_t s = 0; s < CIS_MAX_CIS && free_slot == 0xFF; s++) {
				bool taken = false;
				for (uint8_t k = 0; k < merged.cis_count; k++)
					taken = taken || slot_of[k] == s;
				if (!taken)
					free_slot = s;
			}
			merged.cis[merged.cis_count] = in.cis[j];
			slot_of[merged.cis_count++] = free_slot;
		}
	}
	cis_cig_derived_t der;
	const uint8_t max_nse = st == CIS_ERR_SUCCESS ? cig_max_nse(e, &merged) : 1;
	if (st == CIS_ERR_SUCCESS)
		st = cis_cig_derive(&merged, max_nse, e->caps.cis_gap_us, e->caps.subevent_gap_us, &der);
	if (st != CIS_ERR_SUCCESS) {
		ret[0] = st;
		cmd_complete(e, op, ret, 3);
		return;
	}
	apply_jitter_floor(e, &der, merged.cis_count);
	/* commit: CISes in slot order (CIG phase = slot order) */
	e->cig_used = true;
	e->cig = merged;
	e->der = der;
	for (int i = 0; i < CIS_MAX_CIS; i++)
		if (e->cis[i].st == CS_CONFIGURED)
			e->cis[i].st = CS_FREE;
	cis_hci_cig_params_t ordered = merged;
	cis_cig_derived_t dord = der;
	uint8_t n = 0;
	for (uint8_t s = 0; s < CIS_MAX_CIS; s++) {
		for (uint8_t k = 0; k < merged.cis_count; k++) {
			if (slot_of[k] != s)
				continue;
			ordered.cis[n] = merged.cis[k];
			n++;
		}
	}
	/* The slot order may differ from the order in merged - recompute in slot order so that
	 * CIS_Sync_Delay matches the phase that choose_instant uses later. */
	if (cis_cig_derive(&ordered, max_nse, e->caps.cis_gap_us, e->caps.subevent_gap_us, &dord) != CIS_ERR_SUCCESS)
		dord = der;
	apply_jitter_floor(e, &dord, ordered.cis_count);
	e->cig = ordered;
	e->der = dord;
	n = 0;
	for (uint8_t s = 0; s < CIS_MAX_CIS; s++) {
		bool used = false;
		for (uint8_t k = 0; k < merged.cis_count; k++)
			used = used || slot_of[k] == s;
		if (!used)
			continue;
		cis_t *c = &e->cis[s];
		c->st = CS_CONFIGURED;
		c->cfg = ordered.cis[n];
		c->d = dord.cis[n];
		n++;
	}
	ret[0] = CIS_ERR_SUCCESS;
	ret[1] = in.cig_id;
	ret[2] = in.cis_count;
	for (uint8_t j = 0; j < in.cis_count; j++) {
		uint16_t h = 0;
		for (int s = 0; s < CIS_MAX_CIS; s++)
			if (e->cis[s].st == CS_CONFIGURED && e->cis[s].cfg.cis_id == in.cis[j].cis_id)
				h = e->cis[s].handle;
		cis_put_le16(&ret[3 + 2 * j], h);
	}
	cmd_complete(e, op, ret, (uint8_t)(3 + 2 * in.cis_count));
}

static void hci_create_cis(cis_engine_t *e, const uint8_t *p, uint8_t plen)
{
	const uint16_t op = CIS_OP_LE_CREATE_CIS;
	if (plen < 1 || p[0] == 0 || plen != 1u + 4u * p[0]) {
		cmd_status(e, op, CIS_ERR_INVALID_PARAMS);
		return;
	}
	uint8_t n = p[0];
	if (!e->host_cis) {
		cmd_status(e, op, CIS_ERR_CMD_DISALLOWED);
		return;
	}
	for (int i = 0; i < CIS_MAX_CIS; i++) {
		cis_state_t s = e->cis[i].st;
		if (s == CS_WAIT_SEND || s == CS_WAIT_RSP || s == CS_ESTABLISHING) {
			/* Core: the previous Create CIS has not yet delivered all CIS Established */
			cmd_status(e, op, CIS_ERR_CMD_DISALLOWED);
			return;
		}
	}
	if (n > CIS_MAX_CIS) {
		cmd_status(e, op, CIS_ERR_UNKNOWN_CONN_ID);
		return;
	}
	cis_t *list[CIS_MAX_CIS];
	uint16_t acls[CIS_MAX_CIS];
	for (uint8_t i = 0; i < n; i++) {
		uint16_t ch = cis_get_le16(&p[1 + 4 * i]) & 0x0FFF;
		uint16_t ah = cis_get_le16(&p[3 + 4 * i]) & 0x0FFF;
		cis_t *c = cis_by_handle(e, ch);
		cis_acl_timing_t t;
		if (!c || e->ops.acl_timing(e->ops.ctx, ah, &t) != 0) {
			cmd_status(e, op, CIS_ERR_UNKNOWN_CONN_ID);
			return;
		}
		if (c->st == CS_ESTABLISHED) {
			cmd_status(e, op, 0x0B);   /* Connection Already Exists */
			return;
		}
		for (uint8_t j = 0; j < i; j++) {
			if (list[j] == c) {
				cmd_status(e, op, CIS_ERR_INVALID_PARAMS);
				return;
			}
		}
		if (!e->ops.acl_is_central(e->ops.ctx, ah)) {
			cmd_status(e, op, CIS_ERR_CMD_DISALLOWED);
			return;
		}
		/* Features known (after LL_FEATURE_RSP) without CIS Peripheral: 0x1A right away. Unknown (0): we try - the
		 * refusal will arrive as LL_UNKNOWN_RSP and also end with 0x1A. */
		uint64_t f = e->ops.acl_remote_features(e->ops.ctx, ah);
		if (f && !(f & (1ull << CIS_FEAT_BIT_CIS_PERIPHERAL))) {
			cmd_status(e, op, CIS_ERR_UNSUPP_REMOTE_FEAT);
			return;
		}
		const acl_t *a = acl_get(e, ah, false);
		if (a && a->enc_no_keys) {
			cmd_status(e, op, CIS_ERR_PIN_KEY_MISSING);
			return;
		}
		list[i] = c;
		acls[i] = ah;
	}
	cmd_status(e, op, CIS_ERR_SUCCESS);
	uint32_t now = e->ops.now_us(e->ops.ctx);
	for (uint8_t i = 0; i < n; i++) {
		cis_t *c = list[i];
		uint32_t avoid[CIS_MAX_CIS];
		uint8_t na = 0;
		for (int j = 0; j < CIS_MAX_CIS; j++)
			if (&e->cis[j] != c && e->cis[j].aa)
				avoid[na++] = e->cis[j].aa;
		c->aa = cis_aa_generate(e->ops.random, e->ops.ctx, avoid, na);
		c->acl = acls[i];
		c->cancel = false;
		c->deadline = now + CIS_PROC_TIMEOUT_US;
		c->deadline_on = true;
		memset((void *)&c->s, 0, sizeof(c->s));
		if (!c->aa) {
			fail_create(e, c, CIS_ERR_UNSPECIFIED);
			continue;
		}
		c->st = CS_WAIT_SEND;
		tr(e, c, CIS_TR_CREATE, 0, c->aa, 0, 0, 0);
	}
	try_send_pending(e);
	arm_timer(e);
}

static void hci_remove_cig(cis_engine_t *e, const uint8_t *p, uint8_t plen)
{
	uint8_t ret[2] = { CIS_ERR_SUCCESS, plen ? p[0] : 0 };
	if (plen != 1)
		ret[0] = CIS_ERR_INVALID_PARAMS;
	else if (!e->cig_used || e->cig.cig_id != p[0])
		ret[0] = CIS_ERR_UNKNOWN_ADV_ID;
	else {
		for (int i = 0; i < CIS_MAX_CIS; i++)
			if (e->cis[i].st != CS_FREE && e->cis[i].st != CS_CONFIGURED)
				ret[0] = CIS_ERR_CMD_DISALLOWED;
	}
	if (ret[0] == CIS_ERR_SUCCESS) {
		e->cig_used = false;
		for (int i = 0; i < CIS_MAX_CIS; i++) {
			e->cis[i].st = CS_FREE;
			e->cis[i].aa = 0;
		}
	}
	cmd_complete(e, CIS_OP_LE_REMOVE_CIG, ret, 2);
}

static void hci_setup_path(cis_engine_t *e, const uint8_t *p, uint8_t plen)
{
	uint8_t ret[3] = { CIS_ERR_SUCCESS, plen >= 2 ? p[0] : 0, plen >= 2 ? p[1] : 0 };
	if (plen < 13 || plen != 13u + p[12]) {
		ret[0] = CIS_ERR_INVALID_PARAMS;
	} else {
		cis_t *c = cis_by_handle(e, cis_get_le16(p) & 0x0FFF);
		uint8_t dir = p[2], id = p[3];
		if (!c)
			ret[0] = CIS_ERR_UNKNOWN_CONN_ID;
		else if (dir > 1)
			ret[0] = CIS_ERR_INVALID_PARAMS;
		else if (c->st != CS_ESTABLISHED || (dir == 0 ? c->dp_in : c->dp_out))
			ret[0] = CIS_ERR_CMD_DISALLOWED;
		else if (id != 0)
			ret[0] = CIS_ERR_UNSUPP_FEATURE;   /* HCI path only */
		else if (dir == 0)
			c->dp_in = true;
		else
			c->dp_out = true;
	}
	cmd_complete(e, CIS_OP_LE_SETUP_ISO_PATH, ret, 3);
}

static void hci_remove_path(cis_engine_t *e, const uint8_t *p, uint8_t plen)
{
	uint8_t ret[3] = { CIS_ERR_SUCCESS, plen >= 2 ? p[0] : 0, plen >= 2 ? p[1] : 0 };
	if (plen != 3 || (p[2] & ~0x03u) || !p[2]) {
		ret[0] = CIS_ERR_INVALID_PARAMS;
	} else {
		cis_t *c = cis_by_handle(e, cis_get_le16(p) & 0x0FFF);
		if (!c)
			ret[0] = CIS_ERR_UNKNOWN_CONN_ID;
		else if (((p[2] & 1) && !c->dp_in) || ((p[2] & 2) && !c->dp_out))
			ret[0] = CIS_ERR_CMD_DISALLOWED;
		else {
			if (p[2] & 1)
				c->dp_in = false;
			if (p[2] & 2)
				c->dp_out = false;
		}
	}
	cmd_complete(e, CIS_OP_LE_REMOVE_ISO_PATH, ret, 3);
}

static void hci_read_tx_sync(cis_engine_t *e, const uint8_t *p, uint8_t plen)
{
	uint8_t ret[12];
	memset(ret, 0, sizeof(ret));
	if (plen != 2) {
		ret[0] = CIS_ERR_INVALID_PARAMS;
		cmd_complete(e, CIS_OP_LE_READ_ISO_TX_SYNC, ret, 1);
		return;
	}
	ret[1] = p[0];
	ret[2] = p[1];
	cis_t *c = cis_by_handle(e, cis_get_le16(p) & 0x0FFF);
	if (c)
		reclaim_tx(e, c);
	if (!c || c->st != CS_ESTABLISHED)
		ret[0] = CIS_ERR_UNKNOWN_CONN_ID;
	else if (!c->tx_sync_valid)
		ret[0] = CIS_ERR_CMD_DISALLOWED;
	else {
		cis_put_le16(&ret[3], c->tx_sync_psn);
		cis_put_le32(&ret[5], c->tx_sync_ts);
		cis_put_le24(&ret[9], 0);   /* Time_Offset: 0 for unframed */
	}
	cmd_complete(e, CIS_OP_LE_READ_ISO_TX_SYNC, ret, 12);
}

static void hci_read_buf_v2(cis_engine_t *e)
{
	uint8_t ret[7];
	ret[0] = CIS_ERR_SUCCESS;
	/* ACL fields from snooping 0x2002; without it 0 = "use HCI_Read_Buffer_Size" (Core). The platform may send
	 * 0x2002 itself at start and pass the response to cis_engine_hci_evt_from_controller. */
	cis_put_le16(&ret[1], e->acl_buf_known ? e->acl_buf_len : 0);
	ret[3] = e->acl_buf_known ? e->acl_buf_num : 0;
	cis_put_le16(&ret[4], (uint16_t)CIS_HCI_ISO_LOAD);
	refresh_caps(e);   /* tx_slots from the radio: the host reads the credits on every LE Audio start */
	ret[6] = (uint8_t)CIS_HOST_CREDITS(tx_slots(e));
	cmd_complete(e, CIS_OP_LE_READ_BUF_SIZE_V2, ret, 7);
}

static bool hci_disconnect(cis_engine_t *e, const uint8_t *p, uint8_t plen)
{
	if (plen != 3)
		return false;
	uint16_t h = cis_get_le16(p) & 0x0FFF;
	if (h < CIS_HANDLE_BASE || h >= CIS_HANDLE_BASE + CIS_MAX_CIS)
		return false;   /* ACL - to the controller */
	uint8_t reason = p[2];
	cis_t *c = cis_by_handle(e, h);
	/* Core Vol 4 Part E 7.1.6: allowed reasons */
	if (reason != 0x05 && reason != 0x13 && reason != 0x14 && reason != 0x15 && reason != 0x1A &&
	    reason != 0x29 && reason != 0x3B) {
		cmd_status(e, CIS_OP_DISCONNECT, CIS_ERR_INVALID_PARAMS);
		return true;
	}
	if (!c || c->st == CS_CONFIGURED || c->st == CS_FREE) {
		cmd_status(e, CIS_OP_DISCONNECT, CIS_ERR_UNKNOWN_CONN_ID);
		return true;
	}
	cmd_status(e, CIS_OP_DISCONNECT, CIS_ERR_SUCCESS);
	switch (c->st) {
	case CS_WAIT_SEND:
		fail_create(e, c, CIS_ERR_OP_CANCELLED_BY_HOST);
		break;
	case CS_WAIT_RSP:
		/* LL_CIS_REQ already on the air: we will reject the response (LL_REJECT_EXT_IND) */
		c->cancel = true;
		break;
	case CS_ESTABLISHING:
	case CS_ESTABLISHED: {
		send_term(e, c, reason);
		/* Without an LL acknowledgement from the platform we close right away: the Peripheral gets
		 * LL_CIS_TERMINATE_IND over the ACL, and transmitting on the CIS after that point makes no sense anyway. */
		if (c->st == CS_ESTABLISHED)
			drop_established(e, c, CIS_ERR_LOCAL_HOST_TERM);
		else
			fail_create(e, c, CIS_ERR_OP_CANCELLED_BY_HOST);
		break;
	}
	default:
		break;
	}
	return true;
}

bool cis_engine_hci_cmd(cis_engine_t *e, const uint8_t *h4, uint16_t len)
{
	uint16_t op;
	const uint8_t *p;
	uint8_t plen;
	if (!cis_hci_cmd_header(h4, len, &op, &p, &plen))
		return false;
	switch (op) {
	case CIS_OP_RESET:
		reset_all(e);
		return false;
	case CIS_OP_LE_ENABLE_ENC:
		if (plen == 28) {
			acl_t *a = acl_get(e, cis_get_le16(p) & 0x0FFF, true);
			if (a) {
				memcpy(a->ltk, &p[12], 16);
				a->ltk_ok = true;
			}
		}
		return false;
	case CIS_OP_LE_READ_BUF_SIZE_V2:
		hci_read_buf_v2(e);
		return true;
	case CIS_OP_LE_SET_HOST_FEATURE:
		if (plen != 2 || p[0] != CIS_FEAT_BIT_CIS_HOST)
			return false;
		/* The engine handles bit 32; the C6 controller does not know it and could answer with an error - we do not
		 * forward it. */
		if (p[1] > 1) {
			cmd_complete_status(e, op, CIS_ERR_INVALID_PARAMS);
			return true;
		}
		e->host_cis = p[1] == 1;
		cmd_complete_status(e, op, CIS_ERR_SUCCESS);
		return true;
	case CIS_OP_LE_SET_CIG_PARAMS:
		hci_set_cig(e, p, plen);
		return true;
	case CIS_OP_LE_CREATE_CIS:
		hci_create_cis(e, p, plen);
		return true;
	case CIS_OP_LE_REMOVE_CIG:
		hci_remove_cig(e, p, plen);
		return true;
	case CIS_OP_LE_SETUP_ISO_PATH:
		hci_setup_path(e, p, plen);
		return true;
	case CIS_OP_LE_REMOVE_ISO_PATH:
		hci_remove_path(e, p, plen);
		return true;
	case CIS_OP_LE_READ_ISO_TX_SYNC:
		hci_read_tx_sync(e, p, plen);
		return true;
	case CIS_OP_DISCONNECT:
		return hci_disconnect(e, p, plen);
	default:
		return false;
	}
}

void cis_engine_hci_iso(cis_engine_t *e, const uint8_t *h4, uint16_t len)
{
	cis_hci_iso_t iso;
	if (!cis_hci_iso_parse(h4, len, &iso))
		return;
	cis_t *c = cis_by_handle(e, iso.handle);
	if (!c)
		return;
	if (c->st != CS_ESTABLISHED) {
		c->nocp++;   /* the host buffer is always returned, otherwise the host loses credits */
		return;
	}
	switch (iso.pb) {
	case CIS_ISO_PB_COMPLETE:
		if (c->h_active) {
			STAT_INC(c, sdu_dropped);   /* the previous one is unfinished - its packets are returned too */
			c->nocp = (uint16_t)(c->nocp + c->h_pkts);
		}
		c->h_active = false;
		if (iso.data_len != iso.sdu_len || iso.data_len > CIS_SDU_MAX) {
			STAT_INC(c, sdu_dropped);
			c->nocp++;
			return;
		}
		enqueue_sdu(c, iso.data, iso.data_len, iso.psn, 1);
		return;
	case CIS_ISO_PB_FIRST:
		if (c->h_active) {
			STAT_INC(c, sdu_dropped);
			c->nocp = (uint16_t)(c->nocp + c->h_pkts);
		}
		c->h_active = true;
		c->h_bad = iso.sdu_len > CIS_SDU_MAX;
		c->h_expect = iso.sdu_len;
		c->h_psn = iso.psn;
		c->h_len = 0;
		c->h_pkts = 0;
		break;
	default:
		if (!c->h_active) {
			STAT_INC(c, sdu_dropped);
			c->nocp++;
			return;
		}
		break;
	}
	c->h_pkts++;
	if (!c->h_bad && (uint32_t)c->h_len + iso.data_len <= c->h_expect) {
		memcpy(c->h_buf + c->h_len, iso.data, iso.data_len);
		c->h_len = (uint16_t)(c->h_len + iso.data_len);
	} else {
		c->h_bad = true;
	}
	if (iso.pb == CIS_ISO_PB_LAST) {
		c->h_active = false;
		if (c->h_bad || c->h_len != c->h_expect) {
			STAT_INC(c, sdu_dropped);
			c->nocp = (uint16_t)(c->nocp + c->h_pkts);
			return;
		}
		enqueue_sdu(c, c->h_buf, c->h_len, c->h_psn, c->h_pkts);
	}
}

/* ======================================================================== API: controller events */

static void acl_lost(cis_engine_t *e, uint16_t acl, uint8_t reason)
{
	for (int i = 0; i < CIS_MAX_CIS; i++) {
		cis_t *c = &e->cis[i];
		if (c->st > CS_CONFIGURED && c->acl == acl)
			end_cis(e, c, reason);
	}
	acl_t *a = acl_get(e, acl, false);
	if (a)
		memset(a, 0, sizeof(*a));
	try_send_pending(e);   /* the CIS waiting for LL_CIS_RSP is gone - the next LL_CIS_REQ may go out */
	arm_timer(e);
}

/* The CIS key follows the ACL key. The controller keys (acl_session_keys) take precedence: it already encrypts the
 * ACL with them, which the earbuds accepted. Our own snooping of LTK (0x2019) and SKD/IV (LL_ENC_REQ/RSP) remains a
 * cross-check and a fallback when the platform does not supply keys. */
static void acl_encryption_on(cis_engine_t *e, uint16_t h, bool on)
{
	acl_t *a = acl_get(e, h, on);
	if (!a)
		return;
	a->enc = false;
	a->enc_no_keys = false;
	a->key_src = CIS_KEY_NONE;
	if (!on)
		return;
	const bool own = a->ltk_ok && a->skdm_ok && a->skds_ok;
	if (own) {
		cis_crypto_session_key((cis_aes128_fn)e->ops.aes128, e->ops.ctx, a->ltk, a->skdm, a->skds, a->sk);
		cis_crypto_iv(a->ivm, a->ivs, a->iv);
	}
	uint8_t sk[16], iv[8];
	if (e->ops.acl_session_keys && e->ops.acl_session_keys(e->ops.ctx, h, sk, iv) == 0) {
		if (own && memcmp(sk, a->sk, 16) == 0 && memcmp(iv, a->iv, 8) == 0) {
			a->key_src = CIS_KEY_MATCH;
		} else {
			if (own)
				elog(e, LOG_WARN, "cis: snooped ACL keys != controller - CIS uses the controller key");
			a->key_src = own ? CIS_KEY_MISMATCH : CIS_KEY_CTRL;
			memcpy(a->sk, sk, 16);
			memcpy(a->iv, iv, 8);
		}
		memset(sk, 0, sizeof(sk));
	} else if (own) {
		a->key_src = CIS_KEY_SNOOP;
	} else {
		/* Earbuds reject a CIS without encryption on an encrypted ACL (MIC) - a clear Create CIS refusal is better. */
		elog(e, LOG_WARN, "cis: ACL encryption without a key (snooping incomplete, controller did not supply one)");
		a->enc_no_keys = true;
		return;
	}
	a->enc = true;
}

bool cis_engine_hci_evt_from_controller(cis_engine_t *e, const uint8_t *h4, uint16_t len)
{
	if (len < 3 || h4[0] != CIS_H4_EVT || (uint32_t)len != 3u + h4[2])
		return false;
	uint8_t code = h4[1];
	const uint8_t *p = &h4[3];
	uint8_t plen = h4[2];
	switch (code) {
	case CIS_EVT_CMD_COMPLETE: {
		if (plen < 4)
			return false;
		uint16_t op = cis_get_le16(&p[1]);
		if (op == CIS_OP_LE_READ_BUF_SIZE && plen == 7 && p[3] == 0) {
			e->acl_buf_len = cis_get_le16(&p[4]);
			e->acl_buf_num = p[6];
			e->acl_buf_known = true;
			return false;
		}
		if (op == CIS_OP_LE_READ_LOCAL_FEAT && plen == 12 && p[3] == 0) {
			/* The host must see CIS Central (bit 28) - a copy with the bit added, the original is consumed. */
			uint8_t b[3 + 12];
			memcpy(b, h4, len);
			b[3 + 4 + CIS_FEAT_BIT_CIS_CENTRAL / 8] |= (uint8_t)(1u << (CIS_FEAT_BIT_CIS_CENTRAL % 8));
			to_host(e, b, len);
			return true;
		}
		return false;
	}
	case CIS_EVT_ENC_CHANGE:
		if (plen == 4 && p[0] == 0)
			acl_encryption_on(e, cis_get_le16(&p[1]) & 0x0FFF, p[3] != 0);
		return false;
	case CIS_EVT_ENC_KEY_REFRESH:
		if (plen == 3 && p[0] == 0)
			acl_encryption_on(e, cis_get_le16(&p[1]) & 0x0FFF, true);
		return false;
	case CIS_EVT_DISCONN_COMPLETE:
		if (plen == 4 && p[0] == 0)
			acl_lost(e, cis_get_le16(&p[1]) & 0x0FFF, p[3]);
		return false;
	case CIS_EVT_LE_META:
		/* LE Data Length Change: Connection_Handle, MaxTxOctets, MaxTxTime, MaxRxOctets, MaxRxTime */
		if (plen == 11 && p[0] == 0x07) {
			acl_t *a = acl_get(e, cis_get_le16(&p[1]) & 0x0FFF, true);
			if (a) {
				a->max_tx_octets = cis_get_le16(&p[3]);
				a->dl_blocked = a->max_tx_octets < CIS_LL_CIS_REQ_LEN;
				try_send_pending(e);
				arm_timer(e);
			}
		}
		return false;
	default:
		return false;
	}
}

/* ======================================================================== API: LLCP */

static cis_t *cis_waiting_rsp(cis_engine_t *e, uint16_t acl)
{
	for (int i = 0; i < CIS_MAX_CIS; i++)
		if (e->cis[i].st == CS_WAIT_RSP && e->cis[i].acl == acl)
			return &e->cis[i];
	return NULL;
}

bool cis_engine_ll_ctrl_rx(cis_engine_t *e, uint16_t acl_handle, const uint8_t *ctr, uint8_t len)
{
	if (!ctr || len < 1)
		return false;
	acl_handle &= 0x0FFF;
	switch (ctr[0]) {
	case CIS_LL_ENC_RSP: {
		acl_t *a = acl_get(e, acl_handle, true);
		if (a && cis_ll_enc_rsp_parse(ctr, len, a->skds, a->ivs))
			a->skds_ok = true;
		return false;
	}
	case CIS_LL_CIS_RSP: {
		cis_ll_cis_rsp_t r;
		cis_t *c = cis_waiting_rsp(e, acl_handle);
		if (!c || !cis_ll_cis_rsp_parse(ctr, len, &r))
			return false;
		tr(e, c, CIS_TR_RSP, 0, r.conn_event_count, r.cis_offset_min_us, r.cis_offset_max_us, 0);
		on_cis_rsp(e, c, &r);
		arm_timer(e);
		return true;
	}
	case CIS_LL_REJECT_EXT_IND: {
		uint8_t op, err;
		cis_t *c = cis_waiting_rsp(e, acl_handle);
		if (!c || !cis_ll_reject_ext_parse(ctr, len, &op, &err) || op != CIS_LL_CIS_REQ)
			return false;
		tr(e, c, CIS_TR_REFUSED, err, CIS_LL_REJECT_EXT_IND, 0, 0, 0);
		fail_create(e, c, c->cancel ? CIS_ERR_OP_CANCELLED_BY_HOST : err);
		try_send_pending(e);
		return true;
	}
	case CIS_LL_UNKNOWN_RSP: {
		uint8_t t;
		cis_t *c = cis_waiting_rsp(e, acl_handle);
		if (!c || !cis_ll_unknown_rsp_parse(ctr, len, &t) || t != CIS_LL_CIS_REQ)
			return false;
		tr(e, c, CIS_TR_REFUSED, CIS_ERR_UNSUPP_REMOTE_FEAT, CIS_LL_UNKNOWN_RSP, 0, 0, 0);
		fail_create(e, c, c->cancel ? CIS_ERR_OP_CANCELLED_BY_HOST : CIS_ERR_UNSUPP_REMOTE_FEAT);
		try_send_pending(e);
		return true;
	}
	case CIS_LL_CIS_TERMINATE_IND: {
		cis_ll_cis_term_t t;
		if (!cis_ll_cis_term_parse(ctr, len, &t))
			return false;
		for (int i = 0; i < CIS_MAX_CIS; i++) {
			cis_t *c = &e->cis[i];
			if (c->st > CS_CONFIGURED && c->acl == acl_handle && e->cig_used && t.cig_id == e->cig.cig_id &&
			    t.cis_id == c->cfg.cis_id) {
				tr(e, c, CIS_TR_PEER_TERM, t.error_code, 0, 0, 0, 0);
				end_cis(e, c, t.error_code);
				try_send_pending(e);
			}
		}
		/* Always consumed: the controller does not know CIS and would answer LL_UNKNOWN_RSP to a valid PDU. */
		return true;
	}
	default:
		/* LL_CIS_REQ/LL_CIS_IND from the Peripheral is a protocol error of the peer - let the controller answer
		 * LL_UNKNOWN_RSP. */
		return false;
	}
}

void cis_engine_ll_ctrl_tx_seen(cis_engine_t *e, uint16_t acl_handle, const uint8_t *ctr, uint8_t len)
{
	if (!ctr || len < 1 || ctr[0] != CIS_LL_ENC_REQ)
		return;
	acl_t *a = acl_get(e, acl_handle & 0x0FFF, true);
	if (a && cis_ll_enc_req_parse(ctr, len, a->skdm, a->ivm)) {
		a->skdm_ok = true;
		a->skds_ok = false;   /* new encryption procedure: SKDs will arrive in LL_ENC_RSP */
	}
}

/* ======================================================================== API: timer and deferred work */

void cis_engine_on_timer(cis_engine_t *e)
{
	uint32_t now = e->ops.now_us(e->ops.ctx);
	for (int i = 0; i < CIS_MAX_CIS; i++) {
		cis_t *c = &e->cis[i];
		if (!c->deadline_on || (int32_t)(now - c->deadline) < 0)
			continue;
		c->deadline_on = false;
		if (c->st == CS_WAIT_SEND) {
			/* LL_CIS_REQ never went out: the ACL did not get LE Data Length >= 36 */
			fail_create(e, c, c->cancel ? CIS_ERR_OP_CANCELLED_BY_HOST : CIS_ERR_LIMITED_RESOURCES);
		} else if (c->st == CS_WAIT_RSP) {
			fail_create(e, c, c->cancel ? CIS_ERR_OP_CANCELLED_BY_HOST : CIS_ERR_LL_RESP_TIMEOUT);
		}
	}
	try_send_pending(e);
	arm_timer(e);
}

void cis_engine_process(cis_engine_t *e)
{
	for (int i = 0; i < CIS_MAX_CIS; i++) {
		cis_t *c = &e->cis[i];
		if (c->st != CS_ESTABLISHING && c->st != CS_ESTABLISHED)
			continue;
		uint8_t fl = atomic_load_explicit(&c->a_flags, memory_order_acquire);
		if (c->st == CS_ESTABLISHING) {
			if (fl & F_RX_SEEN) {
				c->st = CS_ESTABLISHED;
				send_established(e, c, CIS_ERR_SUCCESS);
			} else if (fl & F_EST_TIMEOUT) {
				send_term(e, c, CIS_ERR_CONN_FAIL_ESTAB);
				fail_create(e, c, CIS_ERR_CONN_FAIL_ESTAB);
				continue;
			} else {
				continue;
			}
		}
		if (fl & F_SUPERVISION) {
			send_term(e, c, CIS_ERR_CONN_TIMEOUT);
			drop_established(e, c, CIS_ERR_CONN_TIMEOUT);
			continue;
		}
		reclaim_tx(e, c);
		if (!drain_rx(e, c))
			continue;
	}
	send_nocp(e);
	try_send_pending(e);
}
