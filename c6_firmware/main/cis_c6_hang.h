/*
 * Cyberdeck LE Audio: C6 hang trace from the task watchdog (cis_c6_hang.c). A separate header so that app_main does
 * not pull in the CIS engine headers.
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* app_main, right after nvs_flash_init: the trace from the previous boot (RTC) goes to NVS and to 0xFC30/0x09 reads. */
void cis_c6_hang_boot(void);

#ifdef __cplusplus
}
#endif
