/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * M5Stack Tab5 board bring-up for the LE Audio example: I2C and the IO expanders through the BSP
 * (components/m5stack_tab5, they power the ESP32-C6), and the SD card. The SD mount is extracted from the BSP's
 * bsp_sdcard_init() with the settings of the firmware this project comes from: aligned 8 kB bounce buffer and
 * 16-sector transfers, card init under the SDMMC lock shared with the C6 SDIO slot (patched esp_hosted,
 * components/espressif__esp_hosted/PATCHES.md); the patched fatfs (components/fatfs) adds exFAT and 16 kB pieces.
 */
#include "board_tab5.h"

#include <stdio.h>

#include "bsp/m5stack_tab5.h"
#include "driver/i2c_master.h"
#include "driver/sdmmc_host.h"
#include "esp_check.h"
#include "esp_dma_utils.h"
#include "esp_heap_caps.h"
#include "esp_hosted_cyberdeck.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_ldo_regulator.h"
#include "sdmmc_cmd.h"

static const char *TAG = "board";

static sdmmc_card_t *s_card;
static char s_sd_info[48] = "";

esp_err_t board_tab5_init(void)
{
    // BSP (components/m5stack_tab5): I2C bus, then both PI4IOE5V6408 expanders - WLAN_PWR_EN (ESP32-C6 power) and
    // USB 5 V on, speaker amplifier off. Then charging like the original firmware: quick charge allowed, charger on.
    ESP_RETURN_ON_ERROR(bsp_i2c_init(), TAG, "I2C bus");
    bsp_io_expander_pi4ioe_init(bsp_i2c_get_handle());
    bsp_set_charge_qc_en(true);
    vTaskDelay(pdMS_TO_TICKS(50));
    bsp_set_charge_en(true);
    ESP_LOGI(TAG, "IO expanders ready (C6 powered)");
    return ESP_OK;
}

const char *board_tab5_sd_info(void)
{
    return s_sd_info;
}

esp_err_t board_tab5_sd_mount(const char *mount_point)
{
    if (s_card != NULL) {
        return ESP_OK;
    }
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.slot         = SDMMC_HOST_SLOT_0;  // slot 1 of the same controller is the C6 SDIO link
    host.max_freq_khz = SDMMC_FREQ_HIGHSPEED;
    // Unaligned buffers (e.g. from malloc) were read one sector at a time with a fresh allocation each; a fixed
    // 128-byte aligned 8 kB bounce buffer (PSRAM, internal as fallback) and 16 sectors per transfer instead.
    static void *s_bounce = NULL;
    if (s_bounce == NULL) {
        esp_dma_mem_info_t info = {.extra_heap_caps = MALLOC_CAP_SPIRAM, .dma_alignment_bytes = 128};
        size_t sz               = 0;
        if (esp_dma_capable_malloc(8 * 1024, &info, &s_bounce, &sz) != ESP_OK) {
            s_bounce = NULL;
        }
    }
    if (s_bounce == NULL) {
        s_bounce = heap_caps_aligned_alloc(128, 8 * 1024, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    }
    if (s_bounce != NULL) {
        host.dma_aligned_buffer = s_bounce;
    }
    host.unaligned_multi_block_rw_max_chunk_size = 16;

    // SD card IO supply: on-chip LDO_VO4 at a fixed 3.3 V (the same channel and voltage the BSP's sd_pwr_ctrl ends
    // up with). sd_pwr_ctrl_new_on_chip_ldo() acquires the channel with 0 mV first ("voltage value 0 is out of the
    // recommended range") and only then sets 3.3 V; a fixed channel avoids that, and the card runs at 3.3 V high
    // speed only (no 1.8 V UHS switching, so no power control handle is needed).
    static esp_ldo_channel_handle_t s_ldo = NULL;
    if (s_ldo == NULL) {
        esp_ldo_channel_config_t ldo = {.chan_id = 4, .voltage_mv = 3300};
        ESP_RETURN_ON_ERROR(esp_ldo_acquire_channel(&ldo, &s_ldo), TAG, "SD card LDO_VO4 3.3 V");
    }

    sdmmc_slot_config_t slot = SDMMC_SLOT_CONFIG_DEFAULT();
    slot.width               = 4;
    slot.clk                 = GPIO_NUM_43;
    slot.cmd                 = GPIO_NUM_44;
    slot.d0                  = GPIO_NUM_39;
    slot.d1                  = GPIO_NUM_40;
    slot.d2                  = GPIO_NUM_41;
    slot.d3                  = GPIO_NUM_42;

    esp_vfs_fat_sdmmc_mount_config_t mount = {
        .format_if_mount_failed = false, .max_files = 8, .allocation_unit_size = 16 * 1024};
    // Card init changes the shared controller clock: under the lock that every C6 SDIO transaction takes.
    esp_hosted_cyberdeck_sdmmc_lock();
    const esp_err_t err = esp_vfs_fat_sdmmc_mount(mount_point, &host, &slot, &mount, &s_card);
    esp_hosted_cyberdeck_sdmmc_unlock();
    if (err != ESP_OK) {
        s_card = NULL;
        ESP_LOGE(TAG, "SD card not mounted (%s) - FAT32 and exFAT are supported", esp_err_to_name(err));
        return err;
    }
    sdmmc_card_print_info(stdout, s_card);
    const unsigned long long mb = (unsigned long long)s_card->csd.capacity * s_card->csd.sector_size / (1024 * 1024);
    snprintf(s_sd_info, sizeof(s_sd_info), "mounted at %s, %llu GB", mount_point, mb / 1024);
    ESP_LOGI(TAG, "SD card mounted at %s (%llu MB)", mount_point, mb);
    return ESP_OK;
}
