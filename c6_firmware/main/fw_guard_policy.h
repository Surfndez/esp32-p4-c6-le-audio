/*
 * Cyberdeck: C6 image guard decisions (trial period after OTA and rollback to the previous image).
 *
 * Pure logic without ESP-IDF, tested on the host (test/test_fw_guard_policy.c). The IDF code (NVS, otadata, RTC
 * watchdog) is in fw_guard.c.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

/* Total trial time (summed across restarts) after which an unconfirmed image is rolled back. */
#define FW_GUARD_TRIAL_SECS          600u
/* Crashes (panic, interrupt/task watchdog, brownout) during the trial that trigger an immediate rollback. */
#define FW_GUARD_MAX_CRASHES         3u
/* Boots during the trial (e.g. the P4 restarts the C6 on each of its own starts). */
#define FW_GUARD_MAX_BOOTS           10u
/* This many seconds in a row of healthy operation (see fw_guard_health_ok) confirm the image. */
#define FW_GUARD_HEALTHY_SECS        60u
/* Minimum Wi-Fi traffic in both directions since boot: proof that the SDIO <-> radio data path works. */
#define FW_GUARD_MIN_FRAMES          8u
/* BLE-only host: minimum HCI traffic in both directions since boot (controller initialisation by the host). */
#define FW_GUARD_MIN_HCI             4u
/* How often (seconds) the trial time is written to NVS (at most ~20 writes per whole trial). */
#define FW_GUARD_PERSIST_SECS        30u
/* The hardware RTC watchdog fires this long after the software deadline: it catches a hang in which the
 * guard task no longer runs (the margin also covers the RC_SLOW clock spread). */
#define FW_GUARD_HW_DOG_GRACE_SECS   120u

typedef enum {
	FW_GUARD_RST_NORMAL,  /* power-on, EN pin reset from the P4, esp_restart, wake-up */
	FW_GUARD_RST_CRASH,   /* panic, interrupt/task watchdog, CPU lockup, brownout */
	FW_GUARD_RST_HW_DOG,  /* other (RTC) watchdog: the software trial deadline was not met */
} fw_guard_rst_t;

/* Trial record in NVS, valid only for an image with the same ELF digest. */
typedef struct {
	uint8_t  boots;
	uint8_t  crashes;
	uint8_t  gave_up;   /* rollback failed (no target): continue without a trial */
	uint8_t  reserved;
	uint16_t secs;      /* total trial time from previous boots */
} fw_guard_rec_t;

typedef enum {
	FW_GUARD_FREE,      /* image confirmed or nothing to roll back to: no trial */
	FW_GUARD_TRIAL,     /* image unconfirmed: a trial with a deadline */
	FW_GUARD_ROLLBACK,  /* roll back to the previous image now */
} fw_guard_action_t;

typedef struct {
	fw_guard_action_t action;
	fw_guard_rec_t    rec;  /* to be written before the action is executed */
	const char       *why;
} fw_guard_decision_t;

typedef struct {
	bool     datapath;        /* the P4 host opened the SDIO path */
	bool     sta_connected;   /* the Wi-Fi station is connected to the AP */
	bool     bt_enabled;      /* the BLE controller is enabled */
	uint32_t frames_to_air;   /* frames from the host sent to the radio (since boot) */
	uint32_t frames_to_host;  /* frames from the radio handed to the host over SDIO (since boot) */
	bool     wifi_requested;  /* the host enabled Wi-Fi (RPC WifiStart) - then the image must have working Wi-Fi */
	uint32_t hci_from_host;   /* HCI packets from the host (since boot) */
	uint32_t hci_to_host;     /* HCI packets taken by the host - buffer returned after SDIO transmission (since boot) */
} fw_guard_health_t;

/* confirmed: VALID state in otadata or an image digest equal to the stored "last good" one.
 * stored == NULL: no trial record for this image (fresh install). */
fw_guard_decision_t fw_guard_decide_boot(bool confirmed, bool have_previous,
		const fw_guard_rec_t *stored, fw_guard_rst_t rst);

bool fw_guard_health_ok(const fw_guard_health_t *h);

/* healthy_secs: this many seconds in a row of fw_guard_health_ok; start: state at the beginning of that window.
 * Frames must arrive in both directions within the window - a counter since boot alone is not enough. */
bool fw_guard_should_confirm(uint32_t healthy_secs, const fw_guard_health_t *start,
		const fw_guard_health_t *now);

/* Trial seconds that remain; 0 = the deadline has passed. */
uint32_t fw_guard_trial_left(uint16_t secs_before_boot, uint32_t secs_this_boot);

/* Crashes before normal operation (IDF start, nvs_flash_init, fw_guard_boot, BT, the SDIO path with the host): counter
 * in RTC memory, because NVS may be what fails - fw_guard_boot counts in NVS and does not see such loops. This many
 * crashes in a row = rollback to the previous image, also a confirmed one: a C6 boot loop without Wi-Fi is worse. */
#define FW_GUARD_EARLY_CRASHES  3u
#define FW_GUARD_EARLY_MAGIC    0x45415231u  /* "EAR1" */

typedef struct {
	uint32_t magic;
	uint32_t crashes;
} fw_guard_early_t;

/* First call in app_main. crash: reset after panic or watchdog (not brownout - weak power is not the image's fault).
 * true = roll back now (the counter is already zeroed). */
bool fw_guard_early_step(fw_guard_early_t *st, bool crash);
/* Boot reached the SDIO path with the host: the counter restarts from zero. */
void fw_guard_early_passed(fw_guard_early_t *st);
