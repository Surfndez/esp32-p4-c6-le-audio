/*
 * Cyberdeck: C6 image guard after OTA - see fw_guard.h.
 *
 * Why a custom mechanism rather than bootloader rollback: the C6 bootloader cannot be replaced over OTA, and it is
 * unknown whether the factory one has CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE.
 * Rollback: esp_ota_mark_app_invalid_rollback() marks INVALID only the otadata entry of this image;
 * the previous entry stays untouched (e.g. VALID), and every bootloader skips INVALID entries.
 * When there is no entry for the previous image (first install over a cable), we create a new one through
 * esp_ota_set_boot_partition() - it also verifies the whole image (SHA-256) before switching.
 * The "good" key in NVS (digest of the last good image) makes the image we rolled back to
 * start without a trial even in the UNDEFINED state - without it two images with the guard could
 * roll each other back forever.
 */
#include "fw_guard.h"
#include "fw_guard_policy.h"

#include <string.h>

#include "esp_app_desc.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_private/esp_clk.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/wdt_hal.h"
#include "nvs.h"
#include "stats.h"
#if CONFIG_BT_ENABLED
#include "esp_bt.h"
#endif

#if !ESP_PKT_STATS
#error "fw_guard counts Wi-Fi frames from pkt_stats: requires CONFIG_ESP_PKT_STATS=y"
#endif

/* app_main.c */
extern volatile uint8_t datapath;
extern volatile uint8_t station_connected;

#define NVS_NAMESPACE   "fw_guard"
#define NVS_KEY_TRIAL   "trial"
#define NVS_KEY_GOOD    "good"
#define STORED_VER      2
#define IMG_ID_LEN      8
/* Rollback (full image verification, otadata and NVS writes) may run in this task. */
#define GUARD_TASK_STACK 6144

typedef struct {
	uint8_t        ver;
	uint8_t        img[IMG_ID_LEN];  /* beginning of the ELF digest of the image the record belongs to */
	fw_guard_rec_t rec;
} stored_t;

static const char *TAG = "fw_guard";

static volatile bool s_trial;
static fw_guard_rec_t s_rec;
static uint32_t s_hci_from_host, s_hci_to_host;  /* __atomic: counted from several tasks */
static bool s_wifi_requested;

void fw_guard_note_hci_from_host(void)
{
	__atomic_fetch_add(&s_hci_from_host, 1, __ATOMIC_RELAXED);
}

void fw_guard_note_hci_to_host(void)
{
	__atomic_fetch_add(&s_hci_to_host, 1, __ATOMIC_RELAXED);
}

void fw_guard_note_wifi_started(void)
{
	__atomic_store_n(&s_wifi_requested, true, __ATOMIC_RELAXED);
}

static void image_id(uint8_t out[IMG_ID_LEN])
{
	memcpy(out, esp_app_get_description()->app_elf_sha256, IMG_ID_LEN);
}

static bool nvs_write_blob(const char *key, const void *data, size_t len)
{
	nvs_handle_t h;
	esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
	if (err == ESP_OK) {
		err = nvs_set_blob(h, key, data, len);
		if (err == ESP_OK) {
			err = nvs_commit(h);
		}
		nvs_close(h);
	}
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "writing %s to NVS failed: %s", key, esp_err_to_name(err));
	}
	return err == ESP_OK;
}

static bool nvs_read_blob(const char *key, void *data, size_t len)
{
	nvs_handle_t h;
	if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) {
		return false;  /* no namespace = no record */
	}
	size_t got = len;
	const esp_err_t err = nvs_get_blob(h, key, data, &got);
	nvs_close(h);
	return err == ESP_OK && got == len;
}

static bool trial_load(fw_guard_rec_t *out)
{
	stored_t s;
	uint8_t img[IMG_ID_LEN];
	if (!nvs_read_blob(NVS_KEY_TRIAL, &s, sizeof(s)) || s.ver != STORED_VER) {
		return false;
	}
	image_id(img);
	if (memcmp(s.img, img, IMG_ID_LEN) != 0) {
		return false;  /* a record of another image */
	}
	*out = s.rec;
	return true;
}

static bool trial_save(const fw_guard_rec_t *rec)
{
	stored_t s;
	memset(&s, 0, sizeof(s));
	s.ver = STORED_VER;
	image_id(s.img);
	s.rec = *rec;
	return nvs_write_blob(NVS_KEY_TRIAL, &s, sizeof(s));
}

static void trial_erase(void)
{
	nvs_handle_t h;
	if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) == ESP_OK) {
		nvs_erase_key(h, NVS_KEY_TRIAL);
		nvs_commit(h);
		nvs_close(h);
	}
}

static bool is_last_good_image(void)
{
	uint8_t good[IMG_ID_LEN];
	uint8_t img[IMG_ID_LEN];
	if (!nvs_read_blob(NVS_KEY_GOOD, good, sizeof(good))) {
		return false;
	}
	image_id(img);
	return memcmp(good, img, IMG_ID_LEN) == 0;
}

/* RTC hardware watchdog: resets the chip even when a hung system will no longer run any
 * task. Configured the same way as esp_restart_noos() in ESP-IDF; the time is computed
 * from the calibrated RC_SLOW period (the spread of the nominal 136 kHz reaches over ten percent). */
static void hw_dog_arm(uint32_t secs)
{
	const uint32_t cal = esp_clk_slowclk_cal_get();  /* us per tick, Q13.19 */
	const uint64_t ticks_per_sec = cal ? (1000000ULL << 19) / cal : 136000;
	uint64_t ticks = (uint64_t)secs * ticks_per_sec;
	if (ticks > UINT32_MAX) {
		ticks = UINT32_MAX;
	}
	wdt_hal_context_t ctx;
	wdt_hal_init(&ctx, WDT_RWDT, 0, false);
	wdt_hal_write_protect_disable(&ctx);
	wdt_hal_config_stage(&ctx, WDT_STAGE0, (uint32_t)ticks, WDT_STAGE_ACTION_RESET_SYSTEM);
	wdt_hal_config_stage(&ctx, WDT_STAGE1, (uint32_t)ticks_per_sec, WDT_STAGE_ACTION_RESET_RTC);
	wdt_hal_enable(&ctx);
	wdt_hal_write_protect_enable(&ctx);
}

static void hw_dog_off(void)
{
	wdt_hal_context_t ctx = RWDT_HAL_CONTEXT_DEFAULT();
	wdt_hal_write_protect_disable(&ctx);
	wdt_hal_disable(&ctx);
	wdt_hal_write_protect_enable(&ctx);
}

/* Returns only when the rollback failed: then we stay on this image without the guard
 * and remember it (without a rollback target further attempts would only block OTA). */
/* Rollback target: a valid image whose otadata entry is not rejected (INVALID/ABORTED = an image we already rolled
 * back from). A missing entry is acceptable (the factory partition has none at all). */
static bool previous_image_usable(const esp_partition_t *prev, esp_app_desc_t *desc)
{
	esp_ota_img_states_t st;
	if (!prev || esp_ota_get_partition_description(prev, desc) != ESP_OK) {
		return false;
	}
	if (esp_ota_get_state_partition(prev, &st) == ESP_OK &&
			(st == ESP_OTA_IMG_INVALID || st == ESP_OTA_IMG_ABORTED)) {
		return false;
	}
	return true;
}

/* The second OTA partition, and when it is empty - the factory partition. The M5Stack 1.4.1 factory image (M5Burner,
 * full image) has ota_0/ota_1 without factory, like our partitions.esp32c6.csv; factory is a reserve for a unit
 * flashed differently (OTA from the P4 does not change the table): then the new image lands in ota_0, ota_1 is empty
 * and without this the guard would find nothing to roll back to. Rolling back to factory works on both paths of
 * roll_back: esp_ota_check_rollback_is_possible() counts it (IDF 5.5.5), set_boot_partition(factory) clears otadata. */
static const esp_partition_t *previous_partition(esp_app_desc_t *desc)
{
	const esp_partition_t *p = esp_ota_get_next_update_partition(NULL);
	if (previous_image_usable(p, desc)) {
		return p;
	}
	p = esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_FACTORY, NULL);
	if (p != NULL && p != esp_ota_get_running_partition() && previous_image_usable(p, desc)) {
		return p;
	}
	return NULL;
}

static void roll_back(const char *why)
{
	esp_app_desc_t prev_desc;
	const esp_partition_t *prev = previous_partition(&prev_desc);
	esp_err_t err = ESP_ERR_NOT_FOUND;

	ESP_LOGE(TAG, "ROLLBACK to the previous image (%s): %s", prev ? prev->label : "none", why);
	if (prev) {
		/* The previous image ran before the OTA (OTA is blocked during the trial): after the rollback it
		 * starts as good, without its own trial. */
		nvs_write_blob(NVS_KEY_GOOD, prev_desc.app_elf_sha256, IMG_ID_LEN);
		if (esp_ota_check_rollback_is_possible()) {
			err = esp_ota_mark_app_invalid_rollback();
		} else {
			err = esp_ota_set_boot_partition(prev);
		}
	}
	if (err == ESP_OK) {
		trial_erase();
		esp_restart();
	}
	ESP_LOGE(TAG, "rollback failed (%s) - staying on this image, guard disabled",
			esp_err_to_name(err));
	s_rec.gave_up = 1;
	trial_save(&s_rec);
	hw_dog_off();
	s_trial = false;
}

#if !CONFIG_CYBERDECK_FW_GUARD_TEST /* the test image never confirms itself */
static void confirm(void)
{
	uint8_t img[IMG_ID_LEN];
	const esp_err_t err = esp_ota_mark_app_valid_cancel_rollback();
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "writing VALID to otadata failed (%s) - confirmation only in NVS",
				esp_err_to_name(err));
	}
	image_id(img);
	nvs_write_blob(NVS_KEY_GOOD, img, sizeof(img));
	trial_erase();
	hw_dog_off();
	s_trial = false;
	ESP_LOGW(TAG, "image CONFIRMED: SDIO and BLE work, %s",
			__atomic_load_n(&s_wifi_requested, __ATOMIC_RELAXED) ? "Wi-Fi too" : "BLE-only host (HCI)");
}
#endif

static fw_guard_health_t read_health(void)
{
	const fw_guard_health_t h = {
		.datapath = datapath != 0,
		.sta_connected = station_connected != 0,
#if CONFIG_BT_ENABLED
		.bt_enabled = esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_ENABLED,
#else
		.bt_enabled = true,
#endif
		.frames_to_air = pkt_stats.hs_bus_sta_out,
		.frames_to_host = pkt_stats.sta_sh_out,
		.wifi_requested = __atomic_load_n(&s_wifi_requested, __ATOMIC_RELAXED),
		.hci_from_host = __atomic_load_n(&s_hci_from_host, __ATOMIC_RELAXED),
		.hci_to_host = __atomic_load_n(&s_hci_to_host, __ATOMIC_RELAXED),
	};
	return h;
}

static void guard_task(void *arg)
{
	(void)arg;
	const int64_t t0 = esp_timer_get_time();
	uint32_t healthy_secs = 0;
	uint32_t persisted_at = 0;
	fw_guard_health_t window_start = { 0 };

	for (;;) {
		vTaskDelay(pdMS_TO_TICKS(1000));
		const uint32_t now = (uint32_t)((esp_timer_get_time() - t0) / 1000000);
		const fw_guard_health_t health = read_health();
		if (fw_guard_health_ok(&health)) {
			if (healthy_secs++ == 0) {
				window_start = health;
			}
		} else {
			healthy_secs = 0;
		}
#if CONFIG_CYBERDECK_FW_GUARD_TEST
		/* Test image: healthy but never confirmed - after the trial time a rollback must follow (stage 2). */
		if (healthy_secs == FW_GUARD_HEALTHY_SECS) {
			ESP_LOGW(TAG, "TEST IMAGE: healthy, deliberately not confirming - waiting for the rollback after the trial time");
		}
		(void)window_start;
#else
		if (fw_guard_should_confirm(healthy_secs, &window_start, &health)) {
			confirm();
			break;
		}
#endif
		const uint32_t left = fw_guard_trial_left(s_rec.secs, now);
		if (left == 0) {
			roll_back("total trial time elapsed without confirmation");
			break;
		}
		if (now - persisted_at >= FW_GUARD_PERSIST_SECS) {
			fw_guard_rec_t r = s_rec;
			r.secs = (uint16_t)(FW_GUARD_TRIAL_SECS - left);
			trial_save(&r);
			persisted_at = now;
			ESP_LOGW(TAG, "trial: %lu s left (SDIO=%d, Wi-Fi=%d/%d, BLE=%d, frames %lu/%lu, HCI %lu/%lu)",
					(unsigned long)left, health.datapath, health.wifi_requested, health.sta_connected,
					health.bt_enabled, (unsigned long)health.frames_to_air, (unsigned long)health.frames_to_host,
					(unsigned long)health.hci_from_host, (unsigned long)health.hci_to_host);
		}
	}
	vTaskDelete(NULL);
}

static fw_guard_rst_t classify_reset(esp_reset_reason_t r)
{
	switch (r) {
	case ESP_RST_PANIC:
	case ESP_RST_INT_WDT:
	case ESP_RST_TASK_WDT:
	case ESP_RST_CPU_LOCKUP:
	case ESP_RST_BROWNOUT:
		return FW_GUARD_RST_CRASH;
	case ESP_RST_WDT:
		return FW_GUARD_RST_HW_DOG;
	default:
		return FW_GUARD_RST_NORMAL;
	}
}

/* Crash counter before working with the host (fw_guard_policy.h): RTC memory survives a reset after a panic or a
 * watchdog and does not depend on NVS. An EN pin reset from the P4 and power-on clear it - then it is a normal boot. */
static RTC_NOINIT_ATTR fw_guard_early_t s_early;

void fw_guard_early_check(void)
{
	const esp_reset_reason_t r = esp_reset_reason();
	const bool crash = r == ESP_RST_PANIC || r == ESP_RST_INT_WDT || r == ESP_RST_TASK_WDT ||
		r == ESP_RST_CPU_LOCKUP || r == ESP_RST_WDT;
	if (!fw_guard_early_step(&s_early, crash)) {
		return;
	}
	esp_app_desc_t prev_desc;
	const esp_partition_t *prev = previous_partition(&prev_desc);
	ESP_LOGE(TAG, "%u consecutive crashes before working with the host (reset %d) - ROLLBACK to %s %s",
			(unsigned)FW_GUARD_EARLY_CRASHES, (int)r, prev ? prev->label : "(no image)",
			prev ? prev_desc.version : "");
	if (prev == NULL) {
		return;
	}
	/* Without NVS (it may be the cause): the target image decides about its own trial. */
	const esp_err_t err = esp_ota_check_rollback_is_possible() ? esp_ota_mark_app_invalid_rollback()
		: esp_ota_set_boot_partition(prev);
	if (err == ESP_OK) {
		esp_restart();
	}
	ESP_LOGE(TAG, "rollback failed (%s)", esp_err_to_name(err));
}

void fw_guard_reached_host(void)
{
	fw_guard_early_passed(&s_early);
}

void fw_guard_boot(void)
{
	const esp_partition_t *running = esp_ota_get_running_partition();
	esp_ota_img_states_t state = ESP_OTA_IMG_UNDEFINED;
	const bool confirmed = (running &&
		esp_ota_get_state_partition(running, &state) == ESP_OK && state == ESP_OTA_IMG_VALID) ||
		is_last_good_image();

	esp_app_desc_t prev_desc;
	const esp_partition_t *prev = previous_partition(&prev_desc);
	const bool have_previous = prev != NULL;

	fw_guard_rec_t stored;
	const bool has_stored = trial_load(&stored);
	const esp_reset_reason_t reason = esp_reset_reason();
	const fw_guard_decision_t d = fw_guard_decide_boot(confirmed, have_previous,
			has_stored ? &stored : NULL, classify_reset(reason));

	ESP_LOGW(TAG, "%s: %s (reset %d, boots %u, crashes %u, time %u s, previous %s %s)",
			running ? running->label : "?", d.why, (int)reason, d.rec.boots, d.rec.crashes,
			d.rec.secs, have_previous ? prev->label : "-", have_previous ? prev_desc.version : "");

	s_rec = d.rec;
	switch (d.action) {
	case FW_GUARD_FREE:
		return;
	case FW_GUARD_ROLLBACK:
		roll_back(d.why);
		return;
	case FW_GUARD_TRIAL:
		break;
	}

	/* Without storing the counters further crashes would not be counted (a loop without rollback). */
	if (!trial_save(&s_rec)) {
		roll_back("cannot store the trial state in NVS");
		return;
	}
	s_trial = true;
	hw_dog_arm(fw_guard_trial_left(s_rec.secs, 0) + FW_GUARD_HW_DOG_GRACE_SECS);
	/* High priority so that the deadline does not depend on the Wi-Fi/BT tasks; if it still does not run,
	 * the RTC watchdog resets and the next boot rolls the image back. */
	if (xTaskCreate(guard_task, "fw_guard", GUARD_TASK_STACK, NULL, configMAX_PRIORITIES - 2,
				NULL) != pdPASS) {
		ESP_LOGE(TAG, "no guard task - the deadline is watched only by the RTC watchdog");
	}
}

bool fw_guard_in_trial(void)
{
	return s_trial;
}
