# esp_hosted 1.4.0 (host side) - patched copy

Copy of the managed component `espressif/esp_hosted` 1.4.0 (esp-hosted-mcu commit `6040085e`) without `slave/`,
`examples/` and `docs/`. A component in a project's component directories takes precedence over the managed one of
the same name, so `examples/tab5_player` (and any project that lists this directory) uses this copy. License: Apache-2.0 (Espressif), see `LICENSE`.

Changes (search for "Cyberdeck" in the code - the firmware they were made for):

1. `host/drivers/rpc/core/rpc_rsp.c` - a response parse error (missing field, wrong type) ended with SUCCESS and NULL
   pointers; now FAILURE (the co-processor's code is kept).
2. `host/drivers/rpc/core/rpc_core.c` - synchronous responses share one queue and a waiter took the first one: two
   requests at once, or a late response after a timeout, delivered someone else's response. Now one synchronous
   request at a time (`rpc_sync_lock`) and responses with a different `uid` are dropped.
3. `host/port/src/os_wrapper.c` - `hosted_memcpy` with NULL: zero fill and a counter instead of an assert.
4. `host/port/src/sdio_wrapper.c` + `os_wrapper.c` - the SDMMC controller is shared with the SD card (slot 0). Card
   initialisation changes the shared controller clock with a command sent outside the IDF transaction lock and can
   hang the controller in the middle of a transfer on the other slot. A shared recursive lock,
   `esp_hosted_cyberdeck_sdmmc_lock()`, now covers every SDIO transaction, controller/slot start and the
   co-processor card init; the application takes it around SD card mount and unmount. On error / shutdown
   `sdmmc_host_deinit_slot` is used instead of `sdmmc_host_deinit` (which also turned the SD card off).
5. `host/utils/stats.c`, `stats.h`, `host/drivers/transport/sdio/sdio_drv.c` - station packet counters for the
   application: `esp_hosted_cyberdeck_pkt_stats()` (needs `CONFIG_ESP_HOSTED_PKT_STATS=y`; the periodic log report
   is off).
6. `host/drivers/transport/sdio/sdio_drv.c` - the copy of a received packet for lwIP (`copy_payload`) is allocated
   in PSRAM first, internal RAM only as a fallback (it becomes an lwIP pbuf and can live long).
7. `host/drivers/bt/hci_stub_drv.c` + `host/api/include/esp_hosted_cyberdeck.h` - raw HCI H4 channel for an
   application's own BLE host (ESP-IDF Bluetooth stack disabled, `CONFIG_BT_ENABLED` not set, so this file is
   compiled instead of `vhci_drv.c`): `esp_hosted_cyberdeck_hci_tx()` (packet with the H4 type byte) and
   `esp_hosted_cyberdeck_set_hci_rx_cb()` (receive in the SDIO thread). All packet types in both directions,
   including **HCI ISO (0x05)** used by LE Audio CIS (`p4_le_audio`); `vhci_drv.c` (NimBLE / Bluedroid) drops ISO.
   The copy of an outgoing packet is in PSRAM (`MEM_ALLOC_PREFER_PSRAM`).

8. `host/drivers/transport/transport_drv.c` + `host/api/src/esp_hosted_api.c` (le-audio patch 8) - without Wi-Fi
   nobody calls `esp_wifi_init()` -> `esp_hosted_reconfigure()`, which is what resets the C6 and waits for the SDIO
   transport. `esp_hosted_cyberdeck_connect(timeout_ms)` does the same with a time limit (re-reset every 10 s) and
   returns `ESP_ERR_TIMEOUT` instead of aborting; `esp_hosted_cyberdeck_transport_up()` reports the state. An
   HCI-only application must call it before the first RPC or HCI packet.
9. `host/drivers/virtual_serial_if/serial_if.c`, `host/drivers/serial/serial_drv.c`, `serial_ll_if.c`
   (le-audio patch 9) - TX buffer ownership in the RPC write path. `esp_hosted_tx()` takes the buffer in every
   case and frees it itself when it fails (e.g. transport not up yet), but `transport_pserial_send_data()` freed it
   again: `assert failed: tlsf_free ... block already marked as free` and a reboot on the first RPC sent before the
   transport was up. Now the buffer belongs to `serial_drv_write()` and every failing layer below frees it exactly
   once; the caller only reports the error.
10. `host/drivers/rpc/wrap/rpc_wrap.c` (le-audio patch 10) - `rpc_ota_end()` waits up to 30 s instead of the
   default 5 s: the C6 verifies the whole image before it answers. With 5 s a slow verification looked like a
   failure on the P4 while the C6 went on and booted the new image.

When moving to a newer esp_hosted: check whether these fixes are upstream and port the rest.
