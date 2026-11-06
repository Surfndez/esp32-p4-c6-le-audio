#pragma once
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* I2C + IO expanders (powers the ESP32-C6). Call first in app_main. */
esp_err_t board_tab5_init(void);
/* SD card on SDMMC slot 0 (shared controller with the C6 SDIO link), FAT mounted at mount_point. */
esp_err_t board_tab5_sd_mount(const char *mount_point);
/* "mounted at /sd, N GB" after a successful mount. */
const char *board_tab5_sd_info(void);

#ifdef __cplusplus
}
#endif
