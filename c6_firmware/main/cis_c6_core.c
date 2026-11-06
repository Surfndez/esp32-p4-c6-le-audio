/*
 * Cyberdeck LE Audio: C6 adapter - CIS engine instance, hop into the LL task, timer, HCI interception
 * and the 0xFC30 diagnostic command.
 *
 * Contract: cis_platform.h (all engine calls come from one task = the controller's LL task).
 * Other tasks only put work into the rings and wake the LL task with a single npl event.
 * The HCI paths come from analysis of the controller library (section Q6 of our notes, not included).
 */
#include <stdlib.h>
#include <string.h>

#include "esp_app_desc.h"
#include "esp_attr.h"
#include "esp_bt.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nimble/nimble_npl.h"
#include "nimble/npl_freertos.h"
#include "nvs.h"

#include "cis_c6.h"
#include "cis_c6_crash.h"

static const char *TAG = "cis_c6";

/* ble_ll.c.o: eventq_put onto the LL task queue (*(ble_ll_env_p+28)+16), r_ble_ll_event_send+0x10..0x1c. */
extern void r_ble_ll_event_send(struct ble_npl_event *ev);

cis_c6_counters_t g_cis_c6_cnt;

/* ---------- engine memory ---------- */
/* Exactly cis_engine_mem_size() (14,400 B in the v5 image up to 1.4.77, from 1.4.78 about 2.2 kB more: TX queues of
 * 2 CIS with 16 slots - CIS_TX_SLOTS_MAX), once on the first cis_c6_init and never
 * freed - later inits after deinit reuse the same block. The size comes from the engine because a constant in the
 * adapter has already drifted from the engine once (4096 B reserved = LE Audio always off). Out of memory = adapter
 * disabled with an error in the log, BLE works as in v4. */
static void *s_engine_mem;
static cis_engine_t *s_engine;
static bool s_on;              /* read from many tasks: __atomic */
static TaskHandle_t s_ll_task; /* set on the first npl event handled in the LL task */

bool cis_c6_on(void)
{
	return __atomic_load_n(&s_on, __ATOMIC_ACQUIRE);
}

bool cis_c6_in_ll_task(void)
{
	TaskHandle_t ll = __atomic_load_n(&s_ll_task, __ATOMIC_ACQUIRE);
	return ll != NULL && !xPortInIsrContext() && xTaskGetCurrentTaskHandle() == ll;
}

cis_engine_t *cis_c6_engine_ll(void)
{
	return cis_c6_on() && cis_c6_in_ll_task() ? s_engine : NULL;
}

/* ---------- hop into the LL task ---------- */
/* One npl event with its own static FreeRTOS structure: ble_npl_event_init would take it from the controller pool,
 * which in controller-only mode has exactly as many slots as controller events (npl_freertos_mempool_init:
 * BLE_HOST_EV_COUNT = 0) - an extra init ends in BLE_LL_ASSERT. eventq_put/get/run use only the fields
 * queued/fn/arg (npl_os_freertos.c:230, :215, :405). */
static void kick_handler(struct ble_npl_event *ev);
static struct ble_npl_event_freertos s_kick_impl;
static struct ble_npl_event s_kick = { .event = &s_kick_impl };
static uint32_t s_kick_bits;
#define CIS_C6_KICK_SHUTDOWN (1u << 31)
static SemaphoreHandle_t s_shutdown_done;

void cis_c6_kick(uint32_t bits)
{
	if (!cis_c6_on()) {
		return;
	}
	__atomic_fetch_or(&s_kick_bits, bits, __ATOMIC_SEQ_CST);
	/* The queued field is non-atomic: two tasks may put the event twice - the handler empties the mask anyway.
	 * Inside the LL task put does not block as long as the queue (length = number of controller events) has room. */
	r_ble_ll_event_send(&s_kick);
}

/* ---------- platform operations ---------- */

/* Engine -> host path without blocking the LL task. send_to_host_queue waits for room in the SDIO queue without limit
 * (portMAX_DELAY), and the engine sends from the LL task (radio hooks, about 100-200 packets/s while streaming): a
 * read stall on the P4 side (SD card on the shared SDMMC controller) would stop CIS and ACL events, and with
 * datapath = 0 forever. Copies go to a private queue without waiting, and the cis_host task sends them. ISO data does
 * not take the last HOST_Q_RESERVE slots (it is dropped then = audio loss), so that events (Command Complete,
 * CIS Established, Disconnection Complete) always have room; NOCPs on a full queue go to s_nocp and return as one
 * aggregated event (a lost NOCP = the host is one credit short forever). The order relative to the controller's own
 * events (host_rcv_pkt path) may change - the P4 host does not depend on it. */
#define HOST_Q_DEPTH   48
#define HOST_Q_RESERVE 8 /* slots reserved for HCI events only - ISO data never takes them */
#define NOCP_PENDING   4

/* Engine CIS handles: 0x0E00 + i (cis_engine.c CIS_HANDLE_BASE); the C6 controller's ACL handles are small numbers. */
#define CIS_C6_CIS_HANDLE_MIN 0x0E00
/* Engine CIS handles 0x0E00 + i, i < CIS_MAX_CIS (2) - with headroom for more. */
#define H2C_LOST_HANDLES 4
/* ISO packets dropped when the H2C ring is full (cis_c6_h2c): the SDIO task adds, the LL task returns them to the host
 * as NOCP (h2c_lost_return), Disconnection Complete clears them (nocp_drop). */
static uint32_t s_h2c_lost[H2C_LOST_HANDLES];

typedef struct {
	uint8_t *buf;
	uint16_t len;
} host_pkt_t;

static QueueHandle_t s_host_q;
static struct {
	uint16_t handle;
	uint16_t count;
} s_nocp[NOCP_PENDING]; /* LL task only (every caller of hci_to_host) */

static void host_tx_task(void *arg)
{
	(void)arg;
	host_pkt_t p;
	for (;;) {
		if (xQueueReceive(s_host_q, &p, portMAX_DELAY) == pdTRUE) {
			if (slave_bt_send_to_host(p.buf, p.len) != 0) {
				CIS_C6_CNT(host_tx_fail);
			}
			free(p.buf);
		}
	}
}

static bool host_enqueue(const uint8_t *h4, uint16_t len)
{
	host_pkt_t p = { .buf = malloc(len), .len = len };
	if (p.buf == NULL) {
		return false;
	}
	memcpy(p.buf, h4, len);
	if (xQueueSend(s_host_q, &p, 0) != pdTRUE) {
		free(p.buf);
		return false;
	}
	return true;
}

/* NOCP (04 13 len n {handle, count}...) to defer; false = not everything fitted. */
static bool nocp_defer(const uint8_t *h4, uint16_t len)
{
	if (len < 4 || h4[0] != 0x04 || h4[1] != 0x13 || len != 3u + h4[2] || h4[2] != 1u + 4u * h4[3]) {
		return false;
	}
	bool all = true;
	for (uint8_t i = 0; i < h4[3]; i++) {
		const uint16_t h = (uint16_t)(h4[4 + 4 * i] | (h4[5 + 4 * i] << 8));
		const uint16_t c = (uint16_t)(h4[6 + 4 * i] | (h4[7 + 4 * i] << 8));
		int k = 0;
		while (k < NOCP_PENDING && s_nocp[k].count != 0 && s_nocp[k].handle != h) {
			k++;
		}
		if (k == NOCP_PENDING) {
			all = false;
			continue;
		}
		s_nocp[k].handle = h;
		s_nocp[k].count = (uint16_t)(s_nocp[k].count + c);
	}
	return all;
}

/* Disconnection Complete for a handle: the deferred NOCPs of that handle no longer mean anything (the host returns its
 * credits itself), and afterwards they would hit the reused handle 0x0E00+i. */
static void nocp_drop(uint16_t h)
{
	const uint32_t i = (uint32_t)h - CIS_C6_CIS_HANDLE_MIN;
	if (i < H2C_LOST_HANDLES) {
		__atomic_store_n(&s_h2c_lost[i], 0u, __ATOMIC_RELAXED); /* like s_nocp: after disconnect the host returns credits */
	}
	for (int k = 0; k < NOCP_PENDING; k++) {
		if (s_nocp[k].count != 0 && s_nocp[k].handle == h) {
			s_nocp[k].count = 0;
		}
	}
}

/* Deferred NOCPs as one event (handle, count pairs as in Core 7.7.19); only above the reserve for events. */
static void nocp_flush(void)
{
	uint8_t ev[4 + 4 * NOCP_PENDING];
	uint8_t n = 0;
	for (int k = 0; k < NOCP_PENDING; k++) {
		if (s_nocp[k].count != 0) {
			ev[4 + 4 * n] = (uint8_t)s_nocp[k].handle;
			ev[5 + 4 * n] = (uint8_t)(s_nocp[k].handle >> 8);
			ev[6 + 4 * n] = (uint8_t)s_nocp[k].count;
			ev[7 + 4 * n] = (uint8_t)(s_nocp[k].count >> 8);
			n++;
		}
	}
	if (n == 0 || uxQueueSpacesAvailable(s_host_q) <= HOST_Q_RESERVE) {
		return;
	}
	ev[0] = 0x04;
	ev[1] = 0x13;
	ev[2] = (uint8_t)(1 + 4 * n);
	ev[3] = n;
	if (host_enqueue(ev, (uint16_t)(4 + 4 * n))) {
		memset(s_nocp, 0, sizeof(s_nocp));
	}
}

static void op_hci_to_host(void *ctx, const uint8_t *h4, uint16_t len)
{
	(void)ctx;
	const bool iso = len >= 1 && h4[0] == 0x05;
	const bool nocp = len >= 2 && h4[0] == 0x04 && h4[1] == 0x13;
	if (len >= 6 && h4[0] == 0x04 && h4[1] == 0x05) {
		nocp_drop((uint16_t)((h4[4] | (h4[5] << 8)) & 0x0FFF));
	}
	nocp_flush();
	/* ISO data and NOCP do not take the reserve: events (Command Complete/Status, CIS Established, Disconnection
	 * Complete) must always get in. NOCP then waits in s_nocp. */
	if ((iso || nocp) && uxQueueSpacesAvailable(s_host_q) <= HOST_Q_RESERVE) {
		CIS_C6_CNT(host_q_full);
		if (nocp && !nocp_defer(h4, len)) {
			CIS_C6_CNT(host_tx_fail); /* more handles than NOCP_PENDING - credits lost */
		}
		return;
	}
	if (host_enqueue(h4, len)) {
		return;
	}
	CIS_C6_CNT(host_q_full);
	if (nocp && !nocp_defer(h4, len)) {
		CIS_C6_CNT(host_tx_fail);
	}
}

static void op_random(void *ctx, uint8_t *buf, uint16_t len)
{
	(void)ctx;
	esp_fill_random(buf, len);
}

static esp_timer_handle_t s_timer;

static void timer_cb(void *arg)
{
	(void)arg;
	cis_c6_kick(CIS_C6_KICK_TIMER);
}

static void op_timer_start(void *ctx, uint32_t delay_us)
{
	(void)ctx;
	esp_err_t err = esp_timer_stop(s_timer);
	if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
		ESP_LOGE(TAG, "esp_timer_stop: %s", esp_err_to_name(err));
	}
	/* 0 = as soon as possible; esp_timer runs through its own task anyway, so 1 us does not change the ordering. */
	err = esp_timer_start_once(s_timer, delay_us ? delay_us : 1u);
	if (err != ESP_OK) {
		/* Without the timer the engine will not close a procedure at its limit - loud, a configuration error. */
		ESP_LOGE(TAG, "esp_timer_start_once(%lu us): %s", (unsigned long)delay_us, esp_err_to_name(err));
	}
}

static void op_log(void *ctx, int level, const char *msg)
{
	(void)ctx;
	if (level <= 1) {
		ESP_LOGE(TAG, "%s", msg);
	} else if (level == 2) {
		ESP_LOGW(TAG, "%s", msg);
	} else {
		ESP_LOGI(TAG, "%s", msg);
	}
}

static const cis_platform_ops_t k_ops = {
	.ctx = NULL,
	.hci_to_host = op_hci_to_host,
	.send_ctrl_pdu = cis_c6_send_ctrl_pdu,
	.acl_timing = cis_c6_acl_timing,
	.acl_is_central = cis_c6_acl_is_central,
	.acl_remote_features = cis_c6_acl_remote_features,
	.link_open = cis_c6_link_open,
	.link_close = cis_c6_link_close,
	.radio_caps = cis_c6_radio_caps,
	.now_us = cis_c6_now_us,
	.aes128 = cis_c6_aes128,
	.random = op_random,
	.timer_start = op_timer_start,
	.log = op_log,
	.acl_session_keys = cis_c6_acl_session_keys,
	.trace = cis_c6_trace,
};

/* ---------- HCI host -> controller (SDIO task -> LL task) ---------- */

#define H4_CMD 0x01
#define H4_EVT 0x04
#define H4_ISO 0x05

#define OP_DISCONNECT   0x0406
#define OP_RESET        0x0C03
#define OP_LE_READ_FEAT 0x2003
#define OP_LE_ENABLE_ENC 0x2019
#define OP_LE_SET_HOST_FEATURE 0x2074
#define FEAT_BIT_ISO_HOST 32


/* Diagnostics: OGF 0x3F (vendor), OCF 0x030. Without the magic the command goes to the controller as in v4. */
#define CIS_C6_DIAG_OPCODE 0xFC30
static const uint8_t k_diag_magic[4] = { 'C', 'I', 'S', '5' };

/* The ring holds the host's default ISO credits (engine: 24 at tx_slots 12) and commands. At tx_slots 16 (32 credits)
 * it fills up only when the LL task does not receive for about 160 ms (2 CIS x 16 SDU) - the ISO packet is dropped
 * then (h2c_full_iso), but the credit returns as NOCP (s_h2c_lost) instead of being lost at the host for good. 64
 * slots would cost 16.5 kB of C6 internal RAM for a case in which the stream is lost anyway (1.4.78). A power of two
 * - the index wr % H2C_SLOTS stays continuous on wraparound. Slot = 1 + 4 + 502 (ISO_Data_Packet_Length, 0x2060). */
#define H2C_SLOTS 32
#define H2C_DATA  512

enum { H2C_NONE, H2C_CMD, H2C_ISO, H2C_SNOOP };

typedef struct {
	uint16_t len;
	uint8_t kind;
	uint8_t data[H2C_DATA];
} h2c_slot_t;

static h2c_slot_t s_h2c[H2C_SLOTS];
static uint32_t s_h2c_wr; /* producer only: SDIO task (process_hci_rx_pkt) */
static uint32_t s_h2c_rd; /* consumer only: LL task */

static uint16_t le16(const uint8_t *p)
{
	return (uint16_t)(p[0] | (p[1] << 8));
}

static int h2c_class(const uint8_t *h4, uint16_t len)
{
	if (h4[0] == H4_ISO) {
		return H2C_ISO;
	}
	if (h4[0] != H4_CMD || len < 4 || len != 4u + h4[3]) {
		return H2C_NONE; /* malformed command - let the controller answer, as in v4 */
	}
	const uint8_t plen = h4[3];
	switch (le16(&h4[1])) {
	case 0x2060: /* LE Read Buffer Size v2 */
	case 0x2061: /* LE Read ISO TX Sync */
	case 0x2062: /* LE Set CIG Parameters */
	case 0x2064: /* LE Create CIS */
	case 0x2065: /* LE Remove CIG */
	case 0x206E: /* LE Setup ISO Data Path */
	case 0x206F: /* LE Remove ISO Data Path */
		return H2C_CMD;
	case OP_LE_SET_HOST_FEATURE:
		/* Only bit 32; the other bits are known to the controller (r_ble_ll_set_host_feat). */
		return plen == 2 && h4[4] == FEAT_BIT_ISO_HOST ? H2C_CMD : H2C_NONE;
	case OP_DISCONNECT:
		return plen == 3 && (le16(&h4[4]) & 0x0FFF) >= CIS_C6_CIS_HANDLE_MIN ? H2C_CMD : H2C_NONE;
	case CIS_C6_DIAG_OPCODE:
		return plen >= 5 && memcmp(&h4[4], k_diag_magic, 4) == 0 ? H2C_CMD : H2C_NONE;
	case OP_LE_ENABLE_ENC: /* LTK for SK = AES(LTK, SKD) */
	case OP_RESET:         /* the engine closes links before the controller resets connections (LL queue FIFO) */
		return H2C_SNOOP;
	default:
		return H2C_NONE;
	}
}

bool cis_c6_h2c(const uint8_t *h4, uint16_t len)
{
	if (len < 1) {
		return false;
	}
	if (!cis_c6_on()) {
		/* hci_driver_vhci_host_tx knows only ACL and CMD - it would pass ISO on as a raw buffer. After cis_c6_deinit
		 * the host may still have ISO packets in flight: we drop them. */
		if (h4[0] == H4_ISO) {
			CIS_C6_CNT(h2c_iso_off);
			return true;
		}
		return false;
	}
	const int kind = h2c_class(h4, len);
	if (kind == H2C_NONE) {
		return false;
	}
	const uint32_t wr = s_h2c_wr;
	if (len > H2C_DATA || wr - __atomic_load_n(&s_h2c_rd, __ATOMIC_ACQUIRE) >= H2C_SLOTS) {
		if (kind == H2C_ISO) {
			CIS_C6_CNT(h2c_full_iso);
				/* the VHCI driver has no host->controller ISO path - packet dropped, the LL task returns the host credit */
			const uint32_t i = len >= 3 ? (uint32_t)(le16(&h4[1]) & 0x0FFF) - CIS_C6_CIS_HANDLE_MIN : H2C_LOST_HANDLES;
			if (i < H2C_LOST_HANDLES) {
				__atomic_fetch_add(&s_h2c_lost[i], 1u, __ATOMIC_RELAXED);
				cis_c6_kick(CIS_C6_KICK_H2C);
			}
			return true;
		}
		if (kind == H2C_SNOOP) {
			CIS_C6_CNT(h2c_full_snoop);
		} else {
			CIS_C6_CNT(h2c_full_cmd); /* the controller answers with an error instead of silence until the host timeout */
		}
		return false;
	}
	h2c_slot_t *slot = &s_h2c[wr % H2C_SLOTS];
	memcpy(slot->data, h4, len);
	slot->len = len;
	slot->kind = (uint8_t)kind;
	__atomic_store_n(&s_h2c_wr, wr + 1, __ATOMIC_RELEASE);
	/* Kick before forwarding the snooped command to VHCI: the event reaches the LL queue before the command
	 * (PLAUSIBLE: r_ble_ll_hci_cmd_rx puts the event into the same queue), so the engine sees it first. */
	cis_c6_kick(CIS_C6_KICK_H2C);
	if (kind == H2C_SNOOP) {
		CIS_C6_CNT(h2c_snoop);
		return false;
	}
	if (kind == H2C_ISO) {
		CIS_C6_CNT(h2c_iso);
	} else {
		CIS_C6_CNT(h2c_cmd);
	}
	return true;
}

static void send_cmd_complete(uint16_t op, const uint8_t *ret, uint8_t n)
{
	uint8_t ev[3 + 3 + 255];

	if (n > 252) {
		return;
	}
	ev[0] = H4_EVT;
	ev[1] = 0x0E;
	ev[2] = (uint8_t)(3 + n);
	ev[3] = 1; /* Num_HCI_Command_Packets */
	ev[4] = (uint8_t)op;
	ev[5] = (uint8_t)(op >> 8);
	memcpy(&ev[6], ret, n);
	op_hci_to_host(NULL, ev, (uint16_t)(6 + n));
}

static void put32(uint8_t *p, uint32_t v)
{
	p[0] = (uint8_t)v;
	p[1] = (uint8_t)(v >> 8);
	p[2] = (uint8_t)(v >> 16);
	p[3] = (uint8_t)(v >> 24);
}

/* 0xFC30: magic 'CIS5' + subcommand. Reply: status, format version, subcommand, data (README v5). */
#define DIAG_VERSION 1
static void diag_handle(const uint8_t *p, uint8_t plen)
{
	uint8_t ret[252];
	uint8_t n = 3;
	const uint8_t sub = plen >= 5 ? p[4] : 0xFF;

	ret[0] = 0x00;
	ret[1] = DIAG_VERSION;
	ret[2] = sub;
	if (sub == 0x00 && plen == 5) {
		const uint32_t *c = (const uint32_t *)&g_cis_c6_cnt;
		const uint8_t cnt = (uint8_t)(sizeof(g_cis_c6_cnt) / sizeof(uint32_t));
		ret[n++] = (uint8_t)((cis_engine_active(s_engine) ? 1u : 0u) | 2u /* LL task known */ |
				     (cis_c6_iso_features_on() ? 4u : 0u));
		ret[n++] = cnt;
		for (uint8_t i = 0; i < cnt && n + 4 <= sizeof(ret); i++) {
			put32(&ret[n], __atomic_load_n(&c[i], __ATOMIC_RELAXED));
			n += 4;
		}
	} else if (sub == 0x01 && plen == 7) {
		cis_engine_stats_t st;
		if (!cis_engine_get_stats(s_engine, le16(&p[5]), &st)) {
			ret[0] = 0x02; /* Unknown Connection Identifier */
		} else {
			const uint32_t *s = (const uint32_t *)&st;
			for (size_t i = 0; i < sizeof(st) / sizeof(uint32_t); i++) {
				put32(&ret[n], s[i]);
				n += 4;
			}
		}
	} else if (sub == 0x02 && plen == 7) {
		ret[n++] = cis_c6_sk_check(le16(&p[5]));
	} else if (sub == 0x04 && plen == 8) {
		/* Black box: [which: 0 first / 1 last][index u16] -> total u32, n u8, n x 24 B. */
		cis_c6_bb_t e[8];
		uint32_t total = 0;
		const uint8_t got = cis_c6_bb_read(p[5], le16(&p[6]), 8, e, &total);
		put32(&ret[n], total);
		n += 4;
		ret[n++] = got;
		memcpy(&ret[n], e, (size_t)got * sizeof(e[0]));
		n = (uint8_t)(n + got * sizeof(e[0]));
	} else if (sub == 0x05 && plen == 10) {
		/* Knob: [id][value u32] -> [id][effective value u32]; a bad pair = 0x12, no change. */
		const uint8_t id = p[5];
		const uint32_t v = (uint32_t)p[6] | ((uint32_t)p[7] << 8) | ((uint32_t)p[8] << 16) | ((uint32_t)p[9] << 24);
		if (!cis_c6_knob_set(id, v)) {
			ret[0] = 0x12;
		} else {
				ESP_LOGW(TAG, "knob %u = %lu", id, (unsigned long)v);
			ret[n++] = id;
			put32(&ret[n], g_cis_c6_knob[id]);
			n += 4;
		}
	} else if (sub == 0x06 && plen == 5) {
		ret[n++] = CIS_C6_KNOB_COUNT;
		for (uint8_t i = 0; i < CIS_C6_KNOB_COUNT; i++) {
			put32(&ret[n], g_cis_c6_knob[i]);
			n += 4;
		}
	} else if (sub == 0x07 && plen == 9) {
		/* Radio test without Peripheral: [events u16][PDU len][PHY 1/2] -> [code i8]. Not while the engine CIG is active. */
		if (cis_engine_active(s_engine)) {
			ret[0] = 0x0C; /* Command Disallowed */
		} else {
			const int rc = cis_c6_radio_test_start(le16(&p[5]), p[7], p[8]);
				ESP_LOGW(TAG, "radio test: %u events, PDU %u B, PHY %u -> %d", le16(&p[5]), p[7], p[8], rc);
			ret[n++] = (uint8_t)(int8_t)rc;
		}
	} else if (sub == 0x08 && plen == 5) {
		cis_c6_rt_t rt;
		cis_c6_radio_test_get(&rt);
		memcpy(&ret[n], &rt, sizeof(rt));
		n = (uint8_t)(n + sizeof(rt));
	} else if (sub == 0x0A && plen == 9) {
		/* Setup trace: [from index u32] -> [total u32][first returned u32][n u8][n x 28 B cis_c6_tr_t]. */
		cis_c6_tr_t t[8];
		uint32_t index = (uint32_t)p[5] | ((uint32_t)p[6] << 8) | ((uint32_t)p[7] << 16) | ((uint32_t)p[8] << 24);
		uint32_t total = 0;
		const uint8_t got = cis_c6_trace_read(&index, 8, t, &total);
		put32(&ret[n], total);
		n += 4;
		put32(&ret[n], index);
		n += 4;
		ret[n++] = got;
		memcpy(&ret[n], t, (size_t)got * sizeof(t[0]));
		n = (uint8_t)(n + got * sizeof(t[0]));
	} else if (sub == 0x09 && plen == 5) {
		/* Last hang trace (cis_c6_hang.c): [reset reason of this boot][cis_c6_hang_t]. */
		_Static_assert(4 + sizeof(cis_c6_hang_t) <= sizeof(ret), "hang trace in one reply");
		cis_c6_hang_t h;
		cis_c6_hang_get(&h);
		ret[n++] = (uint8_t)esp_reset_reason();
		memcpy(&ret[n], &h, sizeof(h));
		n = (uint8_t)(n + sizeof(h));
	} else if (sub == 0x0B && plen == 6) {
		/* Channel quality: [from channel] -> [from channel][n u8][n x (tries u32, no reply u32)], at most 20 at a time. */
		uint32_t tries[CIS_C6_CHANNELS], bad[CIS_C6_CHANNELS];
		cis_c6_chan_stats(tries, bad);
		const uint8_t from = p[5] < CIS_C6_CHANNELS ? p[5] : CIS_C6_CHANNELS;
		const uint8_t cnt = (uint8_t)(CIS_C6_CHANNELS - from < 20 ? CIS_C6_CHANNELS - from : 20);
		ret[n++] = from;
		ret[n++] = cnt;
		for (uint8_t i = 0; i < cnt; i++) {
			put32(&ret[n], tries[from + i]);
			put32(&ret[n + 4], bad[from + i]);
			n += 8;
		}
	} else if (sub == 0x0C && plen == 5) {
		/* ACL connections: [n u8][n x cis_c6_acl_st_t (20 B)] - only occupied entries. */
		cis_c6_acl_st_t a[CIS_C6_ACL_KEPT];
		cis_c6_acl_stats(a);
		const uint8_t at = n++;
		uint8_t cnt = 0;
		for (int i = 0; i < CIS_C6_ACL_KEPT; i++) {
			if (a[i].handle != 0xFFFF) {
				memcpy(&ret[n], &a[i], sizeof(a[i]));
				n = (uint8_t)(n + sizeof(a[i]));
				cnt++;
			}
		}
		ret[at] = cnt;
	} else if (sub == 0x03 && plen == 5) {
		/* Smallest free stack of the LL task since start, in bytes (StackType_t in IDF is a byte). The engine and hooks
		 * run in this task - this measures the headroom of CONFIG_BT_LE_CONTROLLER_TASK_STACK_SIZE on the DevKit. */
		TaskHandle_t ll = __atomic_load_n(&s_ll_task, __ATOMIC_ACQUIRE);
		put32(&ret[n], ll != NULL ? (uint32_t)uxTaskGetStackHighWaterMark(ll) : 0xFFFFFFFFu);
		n += 4;
	} else {
		ret[0] = 0x12; /* Invalid HCI Command Parameters */
	}
	send_cmd_complete(CIS_C6_DIAG_OPCODE, ret, n);
}

static void h2c_drain(void)
{
	for (;;) {
		const uint32_t rd = s_h2c_rd;
		if (rd == __atomic_load_n(&s_h2c_wr, __ATOMIC_ACQUIRE)) {
			return;
		}
		h2c_slot_t *slot = &s_h2c[rd % H2C_SLOTS];
		uint8_t *h4 = slot->data;

		if (slot->kind == H2C_ISO) {
			cis_engine_hci_iso(s_engine, h4, slot->len);
		} else if (slot->kind == H2C_SNOOP) {
			(void)cis_engine_hci_cmd(s_engine, h4, slot->len); /* snoop: the command already went to the controller */
			if (le16(&h4[1]) == OP_RESET) {
				cis_c6_set_iso_features(false); /* HCI Reset restores the default host features */
			}
		} else if (le16(&h4[1]) == CIS_C6_DIAG_OPCODE) {
			diag_handle(&h4[4], h4[3]);
		} else if (cis_engine_hci_cmd(s_engine, h4, slot->len)) {
			if (le16(&h4[1]) == OP_LE_SET_HOST_FEATURE && h4[5] <= 1) {
				/* The engine answered the host; the CIS Central (28) and ISO Host Support (32) bits also go into the
				 * controller feature word, since the Peripheral sees them in LL_FEATURE_RSP. A sleeping engine omits them. */
				cis_c6_set_iso_features(h4[5] == 1);
			}
		} else {
			/* The engine did not handle the intercepted command (e.g. Disconnect of an unknown CIS handle): the controller
			 * answers as in v4. UNVERIFIED: esp_vhci_host_send_packet from the LL task (NOTES Q6: no lock). */
			CIS_C6_CNT(h2c_fwd_from_ll);
			esp_vhci_host_send_packet(h4, slot->len);
		}
		__atomic_store_n(&s_h2c_rd, rd + 1, __ATOMIC_RELEASE);
	}
}

/* NOCPs for ISO packets dropped on a full ring (cis_c6_h2c): the host took a credit for them and the engine never
 * saw them. LL task only (op_hci_to_host). */
static void h2c_lost_return(void)
{
	uint8_t ev[4 + 4 * H2C_LOST_HANDLES];
	uint8_t n = 0;
	for (uint32_t i = 0; i < H2C_LOST_HANDLES; i++) {
		const uint32_t c = __atomic_exchange_n(&s_h2c_lost[i], 0u, __ATOMIC_RELAXED);
		if (c != 0) {
			const uint16_t h = (uint16_t)(CIS_C6_CIS_HANDLE_MIN + i);
			ev[4 + 4 * n] = (uint8_t)h;
			ev[5 + 4 * n] = (uint8_t)(h >> 8);
			ev[6 + 4 * n] = (uint8_t)c;
			ev[7 + 4 * n] = (uint8_t)(c >> 8);
			n++;
		}
	}
	if (n == 0) {
		return;
	}
	ev[0] = 0x04;
	ev[1] = 0x13;
	ev[2] = (uint8_t)(1 + 4 * n);
	ev[3] = n;
	op_hci_to_host(NULL, ev, (uint16_t)(4 + 4 * n));
}

/* ---------- HCI controller -> host ---------- */

/* Plan B (events outside the LL task): a copy of the events the engine only snoops. 0x2003 is patched by the adapter in
 * place (same as the engine), because otherwise the engine would send the host a second copy. */
#define C2H_SLOTS 8
#define C2H_DATA  32

typedef struct {
	uint8_t len;
	uint8_t data[C2H_DATA];
} c2h_slot_t;

static c2h_slot_t s_c2h[C2H_SLOTS];
static uint32_t s_c2h_wr;
static uint32_t s_c2h_rd;
static portMUX_TYPE s_c2h_mux = portMUX_INITIALIZER_UNLOCKED; /* there may be several producers */

static bool c2h_snooped(const uint8_t *h4, uint16_t len)
{
	switch (h4[1]) {
	case 0x05: /* Disconnection Complete */
	case 0x08: /* Encryption Change */
	case 0x30: /* Encryption Key Refresh Complete */
		return true;
	case 0x0E: /* Command Complete LE Read Buffer Size (ACL fields for 0x2060) */
		return len >= 6 && le16(&h4[4]) == 0x2002;
	case 0x3E: /* LE Data Length Change */
		return len >= 4 && h4[3] == 0x07;
	default:
		return false;
	}
}

static void c2h_patch_features(uint8_t *h4, uint16_t len)
{
	/* Command Complete 0x2003: [3]=num [4..5]=op [6]=status [7..14]=features; bit 28 = byte 3 bit 4. */
	if (h4[1] == 0x0E && len == 15 && h4[2] == 12 && le16(&h4[4]) == OP_LE_READ_FEAT && h4[6] == 0) {
		h4[7 + 3] |= 1u << 4;
	}
}

bool cis_c6_c2h(uint8_t *h4, uint16_t len)
{
	if (!cis_c6_on() || len < 3 || h4[0] != H4_EVT) {
		return false;
	}
	if (cis_c6_in_ll_task()) {
		CIS_C6_CNT(c2h_direct);
		if (cis_engine_hci_evt_from_controller(s_engine, h4, len)) {
			CIS_C6_CNT(c2h_swallowed);
			return true;
		}
		return false;
	}
	/* UNVERIFIED (NOTES Q6): we expect the LL task. Outside it the engine must not be called - snoop through the ring. */
	CIS_C6_CNT(c2h_async);
	c2h_patch_features(h4, len);
	if (len <= C2H_DATA && c2h_snooped(h4, len)) {
		bool ok;
		taskENTER_CRITICAL(&s_c2h_mux);
		ok = s_c2h_wr - s_c2h_rd < C2H_SLOTS;
		if (ok) {
			c2h_slot_t *slot = &s_c2h[s_c2h_wr % C2H_SLOTS];
			memcpy(slot->data, h4, len);
			slot->len = (uint8_t)len;
			s_c2h_wr++;
		}
		taskEXIT_CRITICAL(&s_c2h_mux);
		if (ok) {
			cis_c6_kick(CIS_C6_KICK_C2H);
		} else {
			CIS_C6_CNT(c2h_full);
		}
	}
	return false;
}

static void c2h_drain(void)
{
	c2h_slot_t ev;

	for (;;) {
		bool have;
		taskENTER_CRITICAL(&s_c2h_mux);
		have = s_c2h_rd != s_c2h_wr;
		if (have) {
			ev = s_c2h[s_c2h_rd % C2H_SLOTS];
			s_c2h_rd++;
		}
		taskEXIT_CRITICAL(&s_c2h_mux);
		if (!have) {
			return;
		}
		/* The event already went to the host; the engine only updates its state (it does not swallow snooped types). */
		(void)cis_engine_hci_evt_from_controller(s_engine, ev.data, ev.len);
	}
}

/* ---------- handling in the LL task ---------- */

static void kick_handler(struct ble_npl_event *ev)
{
	(void)ev;
	if (__atomic_load_n(&s_ll_task, __ATOMIC_ACQUIRE) == NULL) {
		__atomic_store_n(&s_ll_task, xTaskGetCurrentTaskHandle(), __ATOMIC_RELEASE);
		ESP_LOGI(TAG, "LL task %p", (void *)xTaskGetCurrentTaskHandle());
	}
	const uint32_t bits = __atomic_exchange_n(&s_kick_bits, 0u, __ATOMIC_SEQ_CST);
	if (!cis_c6_on()) {
		return;
	}
	CIS_C6_CNT(ll_kicks);
	if (bits & CIS_C6_KICK_SHUTDOWN) {
		/* The engine closes all links as on HCI Reset (cis_engine_hci_cmd 0x0C03 -> reset_all). */
		static const uint8_t k_reset[4] = { H4_CMD, 0x03, 0x0C, 0x00 };
		(void)cis_engine_hci_cmd(s_engine, k_reset, sizeof(k_reset));
		cis_c6_set_iso_features(false);
		__atomic_store_n(&s_on, false, __ATOMIC_RELEASE);
		xSemaphoreGive(s_shutdown_done);
		return;
	}
	if (bits & CIS_C6_KICK_H2C) {
		h2c_drain();
		h2c_lost_return();
	}
	if (bits & CIS_C6_KICK_C2H) {
		c2h_drain();
	}
	if (bits & CIS_C6_KICK_TIMER) {
		CIS_C6_CNT(timer_fires);
		cis_engine_on_timer(s_engine);
	}
	cis_c6_radio_tick();
	/* With an idle engine this is a few comparisons; with CIS - decryption, NOCP, HCI events. */
	cis_engine_process(s_engine);
	/* Deferred NOCPs also without new engine packets (a kick arrives on every CIS event). */
	nocp_flush();
}

/* ---------- self-disable after crashes during CIS (cis_c6_crash.h) ---------- */

#define CIS_C6_LIVE_MAGIC 0x43495335u /* 'CIS5' */
#define CIS_C6_NVS_NS     "cis_c6"
#define CIS_C6_NVS_CRASH  "crash"

/* Uninitialized RTC memory: survives panic and watchdogs, does not survive power loss (then garbage != magic). */
static RTC_NOINIT_ATTR uint32_t s_rtc_live;

void cis_c6_live_mark(bool live)
{
	s_rtc_live = live ? CIS_C6_LIVE_MAGIC : 0u;
}

static bool reset_is_crash(esp_reset_reason_t r)
{
	switch (r) {
	case ESP_RST_PANIC:
	case ESP_RST_INT_WDT:
	case ESP_RST_TASK_WDT:
	case ESP_RST_CPU_LOCKUP:
	case ESP_RST_WDT:
		return true;
	default:
		return false; /* power, reset from P4, plain restart, brownout - not CIS's fault */
	}
}

/* Once per C6 boot (later inits after deinit take the remembered result). Main task, NVS already initialized. */
static bool crash_guard_allows(void)
{
	static int8_t s_allowed = -1;
	if (s_allowed >= 0) {
		return s_allowed != 0;
	}
	cis_crash_rec_t stored;
	size_t len = sizeof(stored);
	bool have = false;
	nvs_handle_t h;
	if (nvs_open(CIS_C6_NVS_NS, NVS_READONLY, &h) == ESP_OK) {
		have = nvs_get_blob(h, CIS_C6_NVS_CRASH, &stored, &len) == ESP_OK && len == sizeof(stored);
		nvs_close(h);
	}
	uint8_t img[CIS_CRASH_IMG_LEN];
	memcpy(img, esp_app_get_description()->app_elf_sha256, sizeof(img));
	const bool live = s_rtc_live == CIS_C6_LIVE_MAGIC;
	const bool crash = reset_is_crash(esp_reset_reason());
	s_rtc_live = 0u;

	const cis_crash_boot_t b = cis_crash_boot(have ? &stored : NULL, img, live, crash);
	if (live && crash) {
		ESP_LOGW(TAG, "C6 crash during CIS (%u of %u)", b.rec.crashes, CIS_CRASH_LIMIT);
	}
	if (b.write) {
		esp_err_t err = nvs_open(CIS_C6_NVS_NS, NVS_READWRITE, &h);
		if (err == ESP_OK) {
			err = nvs_set_blob(h, CIS_C6_NVS_CRASH, &b.rec, sizeof(b.rec));
			if (err == ESP_OK) {
				err = nvs_commit(h);
			}
			nvs_close(h);
		}
		if (err != ESP_OK) {
			/* Without a write the crash will not be counted at the next boot; for this boot b decides anyway. */
			ESP_LOGE(TAG, "writing CIS crash counter: %s", esp_err_to_name(err));
		}
	}
	g_cis_c6_cnt.crashes_in_cis = b.rec.crashes;
	if (b.disabled) {
		ESP_LOGE(TAG, "LE Audio disabled after %u crashes during CIS - flash a different image", b.rec.crashes);
	}
	s_allowed = b.disabled ? 0 : 1;
	return !b.disabled;
}

/* ---------- start / stop ---------- */

int cis_c6_init(void)
{
	if (cis_c6_on()) {
		return 0;
	}
	if (!crash_guard_allows()) {
		return -1;
	}
	if (s_engine_mem == NULL) {
		/* Internal RAM: the engine runs in the LL task and in radio callbacks; 8 B alignment for the engine's u64 fields. */
		s_engine_mem = heap_caps_aligned_alloc(8, cis_engine_mem_size(), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
		if (s_engine_mem == NULL) {
			ESP_LOGE(TAG, "no %lu B for the CIS engine - LE Audio disabled", (unsigned long)cis_engine_mem_size());
			return -1;
		}
	}
	if (s_timer == NULL) {
		const esp_timer_create_args_t args = { .callback = timer_cb, .name = "cis_c6" };
		if (esp_timer_create(&args, &s_timer) != ESP_OK) {
			ESP_LOGE(TAG, "esp_timer_create - LE Audio disabled");
			return -1;
		}
	}
	if (s_host_q == NULL) {
		/* The task lives until the C6 restarts (like recv_task/send_task of esp_hosted), same priority as send_task. */
		s_host_q = xQueueCreate(HOST_Q_DEPTH, sizeof(host_pkt_t));
		if (s_host_q != NULL &&
		    xTaskCreate(host_tx_task, "cis_host", 3072, NULL, CONFIG_ESP_DEFAULT_TASK_PRIO, NULL) != pdPASS) {
			vQueueDelete(s_host_q);
			s_host_q = NULL;
		}
		if (s_host_q == NULL) {
			ESP_LOGE(TAG, "no memory for the host queue - LE Audio disabled");
			return -1;
		}
	}
	if (s_shutdown_done == NULL) {
		s_shutdown_done = xSemaphoreCreateBinary();
		if (s_shutdown_done == NULL) {
			ESP_LOGE(TAG, "no memory for the semaphore - LE Audio disabled");
			return -1;
		}
	}
	s_engine = cis_engine_init(s_engine_mem, cis_engine_mem_size(), &k_ops);
	if (s_engine == NULL) {
		ESP_LOGE(TAG, "cis_engine_init - LE Audio disabled");
		return -1;
	}
	cis_c6_radio_reset();
	s_h2c_wr = s_h2c_rd = 0;
	memset(s_h2c_lost, 0, sizeof(s_h2c_lost));
	s_c2h_wr = s_c2h_rd = 0;
	s_kick_bits = 0;
	memset(s_nocp, 0, sizeof(s_nocp));
	/* After the controller deinit our event may have stayed "queued" (queued = true) - put would then skip it. */
	s_kick_impl.queued = false;
	s_kick_impl.fn = kick_handler;
	s_kick_impl.arg = NULL;
	__atomic_store_n(&s_ll_task, NULL, __ATOMIC_RELEASE);
	__atomic_store_n(&s_on, true, __ATOMIC_RELEASE);
	/* The first handled event establishes the LL task; until then hooks and HCI only pass through. */
	cis_c6_kick(0);
	ESP_LOGI(TAG, "CIS engine idle (%lu B), waiting for LE Set CIG Parameters", (unsigned long)cis_engine_mem_size());
	return 0;
}

void cis_c6_deinit(void)
{
	if (!cis_c6_on()) {
		return;
	}
	(void)esp_timer_stop(s_timer);
	xSemaphoreTake(s_shutdown_done, 0);
	cis_c6_kick(CIS_C6_KICK_SHUTDOWN);
	if (xSemaphoreTake(s_shutdown_done, pdMS_TO_TICKS(200)) != pdTRUE) {
		/* The LL task does not respond: the controller frees all connsm on deinit anyway. */
		ESP_LOGW(TAG, "LL task did not close the CIS links within 200 ms");
		__atomic_store_n(&s_on, false, __ATOMIC_RELEASE);
	}
	cis_c6_radio_reset();
}
