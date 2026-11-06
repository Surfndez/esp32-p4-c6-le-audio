#pragma once
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Version of the ESP32-C6 firmware embedded in this app (c6_firmware/build/network_adapter.bin), from its
 * esp_app_desc_t. "" = the embedded image is not a valid c6_firmware image. */
const char *c6_update_embedded_version(void);

/* Writes the embedded image to the C6 over esp_hosted's standard OTA RPC (progress 0..100 via cb, may be NULL).
 * ESP_OK: the C6 accepted and verified the whole image and restarts into it in ~5 s; restart the P4 too (the SDIO
 * link does not survive the C6 restart). The new image is on trial on the C6 (fw_guard) and goes back to the
 * previous one by itself if it does not work. ESP_ERR_INVALID_STATE: the C6 refused to start (its current image
 * is still on trial - try again in a minute). Any other error: the C6 keeps its current image. Blocks ~30 s. */
esp_err_t c6_update_install(void (*progress)(int pct));

#ifdef __cplusplus
}
#endif
