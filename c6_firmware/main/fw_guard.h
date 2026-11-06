/*
 * Cyberdeck: C6 image guard after OTA.
 *
 * An unconfirmed image (after OTA) runs on trial. It confirms itself when, for FW_GUARD_HEALTHY_SECS in a row, the
 * SDIO path works, the Wi-Fi station is connected, the BLE controller is enabled and Wi-Fi frames flow in both
 * directions. Otherwise it rolls back to the previous image: after 3 crashes, after 10 boots, after 10 minutes of
 * total trial time or after the hardware RTC watchdog (a hang). It does not need rollback support in the C6 factory
 * bootloader.
 */
#pragma once

#include <stdbool.h>

/* First call in app_main (before nvs_flash_init): the third crash in a row (panic, watchdog) before
 * fw_guard_reached_host() rolls back to the previous image - also when NVS itself fails or fw_guard_boot, which counts
 * in NVS. The counter is in RTC memory. May not return (rollback + restart). */
void fw_guard_early_check(void);

/* The host opened the SDIO path: the start succeeded, the fw_guard_early_check crash counter restarts from zero. */
void fw_guard_reached_host(void);

/* Call in app_main right after nvs_flash_init(), before BT and the transport.
 * May not return (rollback to the previous image + restart). */
void fw_guard_boot(void);

/* Image health evidence (fw_guard_policy.h): an HCI packet from the host, an HCI packet received by the host (after
 * the SDIO transmission, not after queueing), the host started enabling Wi-Fi (RPC WifiInit / WifiStart). A host that
 * enabled Wi-Fi confirms the image only with working Wi-Fi; a BLE-only host - with HCI traffic both ways. From any
 * task (atomic). */
void fw_guard_note_hci_from_host(void);
void fw_guard_note_hci_to_host(void);
void fw_guard_note_wifi_started(void);

/* true = the image is on trial: OTA is rejected so as not to overwrite the last good image. */
bool fw_guard_in_trial(void);

/* Guard test image (CONFIG_CYBERDECK_FW_GUARD_TEST): this patch number replaces PROJECT_VERSION_PATCH_1 in the
 * coprocessor version, so the P4 (/api/status bt.coprocessor_fw) can see the test image running, then the rollback. */
#define FW_GUARD_TEST_PATCH 49
