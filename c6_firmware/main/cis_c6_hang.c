/*
 * Cyberdeck LE Audio: trace of a C6 hang from the task watchdog (0xFC30 subcommand 9).
 *
 * A high-priority task spinning in a loop does not reset the C6: CONFIG_ESP_TASK_WDT_PANIC is disabled, the watchdog
 * only writes to a UART that nobody reads in the deployed device, and the P4 restarts everything only after minutes
 * without a network (the first radio test of 1.4.52 ended this way, with no trace). The watchdog ISR hook records in
 * RTC memory where the interrupted task was and the adapter state; after CIS_C6_HANG_ABORT_HITS consecutive hits
 * (about 10 s without the IDLE task) abort(): the panic restarts the C6, RTC memory survives, the "CIS live" marker
 * counts crashes during CIS (cis_c6_crash.h). After a C6 crash the P4 restarts and resets the C6 via EN, which RTC
 * memory may not survive - so the first boot after a hang moves the trace to NVS (cis_c6_hang_boot, after NVS init).
 */
#include <string.h>

#include "esp_app_desc.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "riscv/rvruntime-frames.h"
#include "sdkconfig.h"
#include "soc/soc.h"

#include "cis_c6.h"
#include "cis_c6_hang.h"

static const char *TAG = "cis_c6";

#define CIS_C6_HANG_ABORT_HITS 2
/* The next hit comes every CONFIG_ESP_TASK_WDT_TIMEOUT_S while IDLE still has not run; a longer gap = a new hang or
 * IDLE managed to feed the watchdog. */
#define HANG_NEXT_HIT_US ((int64_t)CONFIG_ESP_TASK_WDT_TIMEOUT_S * 1500000)
#define HANG_STACK_WORDS 128
#define HANG_NVS_NS      "cis_c6"
#define HANG_NVS_KEY     "hang"

/* Code ranges in the ELF (linker): IRAM text and flash text; the C6 ROM starts at 0x40000000. The rest of SRAM is data:
 * addresses from there are not return addresses. */
extern const char _iram_text_start[];
extern const char _iram_text_end[];
extern const char _text_start[];
extern const char _text_end[];
#define ROM_TEXT_LOW  0x40000000u
#define ROM_TEXT_HIGH 0x40050000u

_Static_assert(sizeof(cis_c6_hang_t) == 160, "layout = 0xFC30/0x09 format");
static RTC_NOINIT_ATTR cis_c6_hang_t s_rtc;
static cis_c6_hang_t s_last; /* the last trace (RTC from the previous boot, or NVS) */
static int64_t s_last_hit_us;

static bool is_code(uint32_t v)
{
	if ((v & 1u) != 0) {
		return false;
	}
	return (v >= (uint32_t)(uintptr_t)_iram_text_start && v < (uint32_t)(uintptr_t)_iram_text_end) ||
	       (v >= (uint32_t)(uintptr_t)_text_start && v < (uint32_t)(uintptr_t)_text_end) ||
	       (v >= ROM_TEXT_LOW && v < ROM_TEXT_HIGH);
}

void cis_c6_hang_boot(void)
{
	nvs_handle_t h;

	if (s_rtc.magic == CIS_C6_HANG_MAGIC) {
		s_last = s_rtc;
		esp_err_t err = nvs_open(HANG_NVS_NS, NVS_READWRITE, &h);
		if (err == ESP_OK) {
			err = nvs_set_blob(h, HANG_NVS_KEY, &s_last, sizeof(s_last));
			if (err == ESP_OK) {
				err = nvs_commit(h);
			}
			nvs_close(h);
		}
		ESP_LOGE(TAG, "C6 hung in the previous boot: task %.12s, MEPC 0x%08lx, RA 0x%08lx (0xFC30/0x09)",
			 s_last.task, (unsigned long)s_last.mepc, (unsigned long)s_last.ra);
		if (err != ESP_OK) {
			/* RTC stays - the next boot will try again */
			ESP_LOGE(TAG, "writing the hang trace to NVS: %s", esp_err_to_name(err));
			return;
		}
	} else if (nvs_open(HANG_NVS_NS, NVS_READONLY, &h) == ESP_OK) {
		size_t len = sizeof(s_last);
		if (nvs_get_blob(h, HANG_NVS_KEY, &s_last, &len) != ESP_OK || len != sizeof(s_last) ||
		    s_last.magic != CIS_C6_HANG_MAGIC) {
			memset(&s_last, 0, sizeof(s_last));
		}
		nvs_close(h);
	}
	memset(&s_rtc, 0, sizeof(s_rtc));
}

void cis_c6_hang_get(cis_c6_hang_t *out)
{
	*out = s_last;
}

/* From the task watchdog ISR (task_wdt_isr, before it handles the timeout), when the IDLE task has not fed it for
 * CONFIG_ESP_TASK_WDT_TIMEOUT_S. The interrupted task's frame: rtos_int_enter stores sp (= RvExcFrame) in pxTopOfStack,
 * the first TCB field - task_wdt_timeout_abort reads it the same way (vTaskGetSnapshot). */
void esp_task_wdt_isr_user_handler(void)
{
	const int64_t now = esp_timer_get_time();
	cis_c6_hang_t *h = &s_rtc;

	if (h->magic != CIS_C6_HANG_MAGIC || now - s_last_hit_us > HANG_NEXT_HIT_US) {
		memset(h, 0, sizeof(*h));
		h->magic = CIS_C6_HANG_MAGIC;
		memcpy(h->img, esp_app_get_description()->app_elf_sha256, sizeof(h->img));
		h->uptime_ms = (uint32_t)(now / 1000);
		h->kicks_first = g_cis_c6_cnt.ll_kicks;
	}
	s_last_hit_us = now;
	h->hits++;

	TaskHandle_t t = xTaskGetCurrentTaskHandle();
	strlcpy(h->task, pcTaskGetName(t), sizeof(h->task));
	const RvExcFrame *f = *(RvExcFrame *const *)t;
	h->mepc = f->mepc;
	h->ra = f->ra;
	h->mcause = f->mcause;
	const uint32_t *w = (const uint32_t *)(f + 1);
	unsigned n = 0;
	for (unsigned i = 0; i < HANG_STACK_WORDS && n < CIS_C6_HANG_CODE; i++) {
		if ((uintptr_t)&w[i] >= SOC_DRAM_HIGH) {
			break;
		}
		if (is_code(w[i])) {
			h->code[n++] = w[i];
		}
	}
	for (; n < CIS_C6_HANG_CODE; n++) {
		h->code[n] = 0;
	}

	h->kicks_last = g_cis_c6_cnt.ll_kicks;
	h->ev_done = g_cis_c6_cnt.ev_done;
	h->ev_skipped = g_cis_c6_cnt.ev_skipped;
	h->link_open_ok = g_cis_c6_cnt.link_open_ok;
	h->link_close = g_cis_c6_cnt.link_close;
	h->ctrl_conn_end = g_cis_c6_cnt.ctrl_conn_end;
	h->recycle_skipped = g_cis_c6_cnt.recycle_skipped;
	h->hook_wrong_ctx = g_cis_c6_cnt.hook_wrong_ctx;
	cis_c6_radio_test_get(&h->rt);

	if (h->hits >= CIS_C6_HANG_ABORT_HITS) {
		esp_system_abort("C6 hung: task watchdog (trace in 0xFC30/0x09)");
	}
}
