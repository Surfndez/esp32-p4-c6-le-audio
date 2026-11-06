/*
 * Cyberdeck - diagnostics for the local esp_hosted 1.4.0 patches (see PATCHES.md in the component root).
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Late or foreign RPC responses dropped by the waiting request (rpc_core.c). */
uint32_t esp_hosted_cyberdeck_stale_responses(void);
/* Copies from/to NULL turned into zero fill instead of an assert (os_wrapper.c). */
uint32_t esp_hosted_cyberdeck_null_copies(void);

/* Station packet counters since boot (stats.c; zeros without CONFIG_ESP_HOSTED_PKT_STATS, except
 * throttle_starts). tx_drop: packets dropped by the host because the C6 reported its transmit queue
 * filled above the threshold (TCP has to retransmit them - stalls of ~0.5 s). */
typedef struct {
	uint32_t rx_in;            /* received from the C6 */
	uint32_t tx_pass;          /* accepted for sending */
	uint32_t tx_drop;          /* dropped while throttling */
	uint32_t tx_out;           /* sent to the C6 */
	uint32_t throttle_starts;  /* how many times the C6 started throttling */
	uint8_t throttling;        /* throttling is active now */
} esp_hosted_cyberdeck_pkt_stats_t;
void esp_hosted_cyberdeck_pkt_stats(esp_hosted_cyberdeck_pkt_stats_t *out);

/* Shared SDMMC controller: slot 1 is the co-processor (SDIO), slot 0 is the application's SD card.
 * Card initialisation changes the controller clock (shared divider) with a "clock update" command
 * that IDF sends WITHOUT the transaction lock - entering a transfer on the other slot can hang
 * the controller. The (recursive) lock therefore covers every SDIO transaction of the co-processor,
 * start/release of its slot and card, and SD card mount/unmount. Plain file reads and writes on the
 * card do not need it - IDF serialises those transactions itself (os_wrapper.c). */
void esp_hosted_cyberdeck_sdmmc_lock(void);
void esp_hosted_cyberdeck_sdmmc_unlock(void);

/* Raw HCI H4 channel over SDIO (ESP_HCI_IF) to the C6 co-processor with the host BT stack disabled
 * (hci_stub_drv.c): the first byte is the H4 packet type (0x01 CMD, 0x02 ACL, 0x04 EVT, 0x05 ISO). */
typedef void (*esp_hosted_cyberdeck_hci_rx_cb_t)(const uint8_t *data, uint32_t len);
void esp_hosted_cyberdeck_set_hci_rx_cb(esp_hosted_cyberdeck_hci_rx_cb_t cb);
int esp_hosted_cyberdeck_hci_tx(const uint8_t *data, uint16_t len);

/* le-audio patch 8: bring the SDIO transport to the C6 up without Wi-Fi - reset the slave (EN) and wait up to
 * timeout_ms for its start-up event (re-reset every 10 s). ESP_OK = up (also when it already was),
 * ESP_ERR_TIMEOUT = the C6 did not answer. Nothing may be sent to the C6 (RPC, HCI) before this succeeds. */
esp_err_t esp_hosted_cyberdeck_connect(uint32_t timeout_ms);
bool esp_hosted_cyberdeck_transport_up(void);

#ifdef __cplusplus
}
#endif
