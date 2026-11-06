/*
 * Cyberdeck LE Audio: self-disable of the CIS adapter after crashes during a stream (pure logic, host test).
 *
 * A C6 crash during CIS ends in a C6 restart, and on its next attempt the P4 enables CIS again. The fw_guard guard
 * does not catch this, because the image is already confirmed. The adapter keeps a "CIS live" marker in RTC memory;
 * a crash reset with the marker set = a crash during CIS, counted in NVS per image. After CIS_CRASH_LIMIT crashes
 * the adapter does not enable itself (BLE as in v4) until another image is flashed.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CIS_CRASH_IMG_LEN 8
#define CIS_CRASH_LIMIT   2

/* NVS record (blob): the image the counter belongs to and the number of crashes during CIS. */
typedef struct {
	uint8_t ver;
	uint8_t img[CIS_CRASH_IMG_LEN];
	uint8_t crashes;
} cis_crash_rec_t;

#define CIS_CRASH_REC_VER 1

typedef struct {
	cis_crash_rec_t rec; /* state after boot */
	bool write;          /* rec must be written to NVS */
	bool disabled;       /* do not enable the adapter */
} cis_crash_boot_t;

/* stored = read from NVS or NULL (missing, wrong size, other version); img = digest of the current image;
 * live_at_reset = the RTC "CIS live" marker before the reset; crash_reset = a crash reset (panic, watchdog, lockup). */
cis_crash_boot_t cis_crash_boot(const cis_crash_rec_t *stored, const uint8_t img[CIS_CRASH_IMG_LEN],
				bool live_at_reset, bool crash_reset);

#ifdef __cplusplus
}
#endif
