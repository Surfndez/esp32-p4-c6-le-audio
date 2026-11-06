// Copyright 2015-2024 Espressif Systems (Shanghai) PTE LTD
/* SPDX-License-Identifier: GPL-2.0 OR Apache-2.0 */

#include <string.h>
#include "esp_hosted_transport.h"
#include "os_wrapper.h"
#include "transport_drv.h"
#include "hci_drv.h"
#include "esp_hosted_cyberdeck.h"

#include "esp_hosted_log.h"
static const char TAG[] = "hci_stub_drv";

/* Cyberdeck: raw HCI H4 channel for the application's own BLE host with the ESP-IDF BT stack disabled.
 * All H4 types in both directions without filtering: CMD 0x01, ACL 0x02, EVT 0x04 and ISO 0x05 (LE Audio CIS) - the
 * SDIO transport carries the first byte in hci_pkt_type, and receive hands the packet over with the type byte. The
 * counterpart in vhci_drv.c (NimBLE/Bluedroid stack) drops ISO, but that file is not compiled in this configuration. */
static esp_hosted_cyberdeck_hci_rx_cb_t s_hci_rx_cb = NULL;

void esp_hosted_cyberdeck_set_hci_rx_cb(esp_hosted_cyberdeck_hci_rx_cb_t cb)
{
	s_hci_rx_cb = cb;
}

int esp_hosted_cyberdeck_hci_tx(const uint8_t *data, uint16_t len)
{
	if (!data || !len) {
		return ESP_FAIL;
	}
	/* Cyberdeck: the copy that lives while queued goes to PSRAM - the SDIO write task copies it into the transport
	 * buffer anyway (H_BUFF_NO_ZEROCOPY), and an LE Audio stream is ~100-200 packets/s that should not churn
	 * internal DMA memory. */
	uint8_t *ptr = MEM_ALLOC_PREFER_PSRAM(len);
	if (!ptr) {
		return ESP_FAIL;
	}
	memcpy(ptr, data, len);
	return esp_hosted_tx(ESP_HCI_IF, 0, ptr, len, H_BUFF_NO_ZEROCOPY, H_DEFLT_FREE_FUNC);
}

#if H_BT_HOST_ESP_NIMBLE
#include "host/ble_hs_mbuf.h"
#include "nimble/transport.h"
#endif

#if H_BT_HOST_ESP_BLUEDROID
#include "esp_hosted_bt.h"
#endif

#define WEAK __attribute__((weak))

/* Cyberdeck: every received HCI packet (including ISO 0x05) goes to the application callback,
   in the SDIO receive thread. */
int hci_rx_handler(interface_buffer_handle_t *buf_handle)
{
	esp_hosted_cyberdeck_hci_rx_cb_t cb = s_hci_rx_cb;
	if (cb && buf_handle && buf_handle->payload && buf_handle->payload_len) {
		cb(buf_handle->payload, buf_handle->payload_len);
	}
	return ESP_OK;
}

void hci_drv_init(void)
{
}

void hci_drv_show_configuration(void)
{
	ESP_LOGI(TAG, "Host BT Support: Disabled");
}

#if H_BT_HOST_ESP_NIMBLE
/**
 * ESP NimBLE expects these interfaces for Tx
 *
 * There are marked as weak references:
 *
 * - to allow ESP NimBLE BT Host code to override the functions if
 *   NimBLE BT Host is configured to act as the HCI transport
 *
 * - to allow the User to use their own ESP NimBLE HCI transport code
 *   without causing linker errors from Hosted
 *
 * - to allow Hosted code to build without linker errors if ESP NimBLE
 *   BT Host is enabled, but Hosted is not configured as HCI transport
 *   and there is no other ESP NimBLE HCI transport code being
 *   used. In this case, the stub functions are used and drops the
 *   incoming data.
 */

WEAK int ble_transport_to_ll_acl_impl(struct os_mbuf *om)
{
	os_mbuf_free_chain(om);

	return ESP_FAIL;
}

WEAK int ble_transport_to_ll_cmd_impl(void *buf)
{
	ble_transport_free(buf);

	return ESP_FAIL;
}
#endif // H_BT_HOST_ESP_NIMBLE

#if H_BT_HOST_ESP_BLUEDROID
WEAK void hosted_hci_bluedroid_open(void)
{
}

WEAK void hosted_hci_bluedroid_close(void)
{
}

WEAK void hosted_hci_bluedroid_send(uint8_t *data, uint16_t len)
{
}

WEAK bool hosted_hci_bluedroid_check_send_available(void)
{
	return false;
}

WEAK esp_err_t hosted_hci_bluedroid_register_host_callback(const esp_bluedroid_hci_driver_callbacks_t *callback)
{
	return ESP_FAIL;
}
#endif // H_BT_HOST_ESP_BLUEDROID
