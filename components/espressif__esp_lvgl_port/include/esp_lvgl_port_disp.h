/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief ESP LVGL port display
 */

#pragma once

#include "esp_err.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "lvgl.h"

#if LVGL_VERSION_MAJOR == 8
#include "esp_lvgl_port_compatibility.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Rotation configuration
 */
typedef struct {
    bool swap_xy;  /*!< LCD Screen swapped X and Y (in esp_lcd driver) */
    bool mirror_x; /*!< LCD Screen mirrored X (in esp_lcd driver) */
    bool mirror_y; /*!< LCD Screen mirrored Y (in esp_lcd driver) */
} lvgl_port_rotation_cfg_t;

/**
 * @brief Configuration display structure
 */
typedef struct {
    esp_lcd_panel_io_handle_t io_handle;   /*!< LCD panel IO handle */
    esp_lcd_panel_handle_t panel_handle;   /*!< LCD panel handle */
    esp_lcd_panel_handle_t control_handle; /*!< LCD panel control handle */

    uint32_t buffer_size; /*!< Size of the buffer for the screen in pixels */
    bool double_buffer;   /*!< True, if should be allocated two buffers */
    uint32_t trans_size;  /*!< Allocated buffer will be in SRAM to move framebuf (optional) */

    uint32_t hres; /*!< LCD display horizontal resolution */
    uint32_t vres; /*!< LCD display vertical resolution */

    bool monochrome; /*!< True, if display is monochrome and using 1bit for 1px */

    lvgl_port_rotation_cfg_t
        rotation; /*!< Default values of the screen rotation (Only HW state. Not supported for default SW rotation!) */
#if LVGL_VERSION_MAJOR >= 9
    lv_color_format_t color_format; /*!< The color format of the display */
#endif
    struct {
        unsigned int buff_dma : 1;    /*!< Allocated LVGL buffer will be DMA capable */
        unsigned int buff_spiram : 1; /*!< Allocated LVGL buffer will be in PSRAM */
        unsigned int sw_rotate : 1;   /*!< Use software rotation (slower) or PPA if available */
#if LVGL_VERSION_MAJOR >= 9
        unsigned int swap_bytes : 1; /*!< Swap bytes in RGB656 (16-bit) color format before send to LCD driver */
#endif
        unsigned int full_refresh : 1; /*!< 1: Always make the whole screen redrawn */
        unsigned int direct_mode : 1;  /*!< 1: Use screen-sized buffers and draw to absolute coordinates */
    } flags;
} lvgl_port_display_cfg_t;

/**
 * @brief Configuration RGB display structure
 */
typedef struct {
    struct {
        unsigned int bb_mode : 1; /*!< 1: Use bounce buffer mode */
        unsigned int
            avoid_tearing : 1; /*!< 1: Use internal RGB buffers as a LVGL draw buffers to avoid tearing effect, enabling
                                  this option requires over two LCD buffers and may reduce the frame rate */
    } flags;
} lvgl_port_display_rgb_cfg_t;

/**
 * @brief Configuration MIPI-DSI display structure
 */
typedef struct {
    struct {
        unsigned int
            avoid_tearing : 1; /*!< 1: Use internal MIPI-DSI buffers as a LVGL draw buffers to avoid tearing effect,
                                  enabling this option requires over two LCD buffers and may reduce the frame rate */
    } flags;
} lvgl_port_display_dsi_cfg_t;

/**
 * @brief Add I2C/SPI/I8080 display handling to LVGL
 *
 * @note Allocated memory in this function is not free in deinit. You must call lvgl_port_remove_disp for free all
 * memory!
 *
 * @param disp_cfg Display configuration structure
 * @return Pointer to LVGL display or NULL when error occurred
 */
lv_display_t *lvgl_port_add_disp(const lvgl_port_display_cfg_t *disp_cfg);

/**
 * @brief Add MIPI-DSI display handling to LVGL
 *
 * @note Allocated memory in this function is not free in deinit. You must call lvgl_port_remove_disp for free all
 * memory!
 *
 * @param disp_cfg Display configuration structure
 * @param dsi_cfg MIPI-DSI display specific configuration structure
 * @return Pointer to LVGL display or NULL when error occurred
 */
lv_display_t *lvgl_port_add_disp_dsi(const lvgl_port_display_cfg_t *disp_cfg,
                                     const lvgl_port_display_dsi_cfg_t *dsi_cfg);

/**
 * @brief Cyberdeck: number of frames the DPI DMA has sent to the panel since boot (MIPI-DSI, without
 *        avoid_tearing). A counter that stops growing means the video stream died (panel lit, black).
 */
uint32_t lvgl_port_dpi_frame_count(void);

/**
 * @brief Cyberdeck: LVGL flushes handed to the MIPI-DSI panel (submitted) and flushes whose pixels are already
 *        in the DPI frame buffer (done: DMA2D copy or direct PPA rotation finished). done == submitted plus one
 *        more complete DPI frame (lvgl_port_dpi_frame_count) = the rendered frame is on the panel.
 */
void lvgl_port_flush_counts(uint32_t *submitted, uint32_t *done);

/**
 * @brief Cyberdeck: rotate with the PPA straight into the DPI frame buffer (LV_DISPLAY_ROTATION_90, sw_rotate)
 *        instead of into a third full-screen buffer followed by a DMA2D copy: two PSRAM passes per flush fewer
 *        and one full-screen buffer (1.84 MB at 720x1280 RGB565) less. Must be called before
 *        lvgl_port_add_disp_dsi(). Visual changes (tearing) must be confirmed on the device.
 */
void lvgl_port_set_direct_rotation(bool on);

/**
 * @brief Cyberdeck: direct rotation active (requested, frame buffer found, no fallback after a PPA error)
 */
bool lvgl_port_direct_rotation(void);

/**
 * @brief Cyberdeck: height of the LVGL render strip in rows of the rotated (landscape) picture, rounded down to a
 *        multiple of 4; 0 or more than the draw buffers hold = whole buffers. Returns the height in use (0 = not
 *        supported: frame buffers used as LVGL buffers). Call under the LVGL lock while no flush is in progress.
 */
int32_t lvgl_port_set_strip_lines(lv_display_t *disp, int32_t lines);

/**
 * @brief Cyberdeck: total time spent in the flush callback since boot, us (PPA rotation, DMA2D copy start).
 */
uint64_t lvgl_port_flush_time_us(void);

/**
 * @brief Cyberdeck: dim the whole picture in the flush (black over everything with this opacity, 0 = none), with the
 *        same result as an LVGL black rectangle on top. LVGL task only. false = not available (not RGB565): draw
 *        the dimming layer in LVGL instead.
 */
bool lvgl_port_set_dim(uint8_t opa);

/**
 * @brief Add RGB display handling to LVGL
 *
 * @note Allocated memory in this function is not free in deinit. You must call lvgl_port_remove_disp for free all
 * memory!
 *
 * @param disp_cfg Display configuration structure
 * @param rgb_cfg RGB display specific configuration structure
 * @return Pointer to LVGL display or NULL when error occurred
 */
lv_display_t *lvgl_port_add_disp_rgb(const lvgl_port_display_cfg_t *disp_cfg,
                                     const lvgl_port_display_rgb_cfg_t *rgb_cfg);

/**
 * @brief Remove display handling from LVGL
 *
 * @note Free all memory used for this display.
 *
 * @return
 *      - ESP_OK                    on success
 */
esp_err_t lvgl_port_remove_disp(lv_display_t *disp);

#ifdef __cplusplus
}
#endif
