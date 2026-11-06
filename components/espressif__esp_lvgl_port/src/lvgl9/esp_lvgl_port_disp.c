/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_heap_caps.h"
#include "esp_idf_version.h"
#include "esp_timer.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lvgl_port.h"
#include "esp_lvgl_port_priv.h"
#include "esp_lvgl_port_rotate.h"
#include "driver/ppa.h"
#include "esp_heap_caps.h"
#include "esp_private/esp_cache_private.h"
#include "esp_cache.h"
#include "src/display/lv_display_private.h"  // Cyberdeck: last_part (last strip of the area)

#define ALIGN_UP_BY(num, align) (((num) + ((align)-1)) & ~((align)-1))
#define BLOCK_SIZE_SMALL        (32)
#define BLOCK_SIZE_LARGE        (256)
static ppa_client_handle_t ppa_srm_handle = NULL;
static size_t data_cache_line_size        = 0;
// Cyberdeck: progress of flushes to the DSI panel (lvgl_port_flush_counts) and PPA rotation straight into the
// frame buffer.
static volatile uint32_t s_flush_submitted = 0;  // LVGL task
static volatile uint32_t s_flush_done      = 0;  // PPA / DMA2D interrupt or the LVGL task
static bool s_direct_requested             = false;
static volatile uint64_t s_flush_cb_us     = 0;  // Cyberdeck: time spent in flush_cb since boot (measurement)
// Cyberdeck: after a PPA error or timeout (hung SRM, esp-idf #19096) rotate on the CPU for
// LVGL_PORT_PPA_PAUSE_MS - the UI slows down but does not stop; then PPA is tried again.
#define LVGL_PORT_PPA_PAUSE_MS 3000
static TickType_t s_ppa_pause_from         = 0;
static bool s_ppa_paused                   = false;
static uint32_t s_ppa_failures             = 0;
// Cyberdeck: background flushes (LVGL strips). The end of a flush (PPA or DMA2D interrupt) releases s_flush_sem -
// the LVGL task waits on it (lvgl_port_flush_wait) instead of spinning in a loop on core 0.
static SemaphoreHandle_t s_flush_sem = NULL;
static volatile bool s_flush_busy    = false;  // flush in progress (PPA or DMA2D in the background)
static bool s_area_open              = false;  // strips assembled in the rotation buffer, not yet in the frame buffer
static lv_area_t s_area;                       // that area (LVGL coordinates, whole - lvgl_port_find_area)

#if CONFIG_IDF_TARGET_ESP32S3 && ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
#include "esp_lcd_panel_rgb.h"
#endif

#if (CONFIG_IDF_TARGET_ESP32P4 && ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 3, 0))
#include "esp_lcd_mipi_dsi.h"
#endif

#if (ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(4, 4, 4)) || (ESP_IDF_VERSION == ESP_IDF_VERSION_VAL(5, 0, 0))
#define LVGL_PORT_HANDLE_FLUSH_READY 0
#else
#define LVGL_PORT_HANDLE_FLUSH_READY 1
#endif

#ifndef CONFIG_LV_DRAW_BUF_ALIGN
#define CONFIG_LV_DRAW_BUF_ALIGN 1
#endif

static const char* TAG = "LVGL";

/*******************************************************************************
 * Types definitions
 *******************************************************************************/

typedef struct {
    lvgl_port_disp_type_t disp_type;       /* Display type */
    esp_lcd_panel_io_handle_t io_handle;   /* LCD panel IO handle */
    esp_lcd_panel_handle_t panel_handle;   /* LCD panel handle */
    esp_lcd_panel_handle_t control_handle; /* LCD panel control handle */
    lvgl_port_rotation_cfg_t rotation;     /* Default values of the screen rotation */
    lv_color_t* draw_buffs[3];             /* Display draw buffers */
    uint8_t* oled_buffer;
    lv_display_t* disp_drv; /* LVGL display driver */
    lv_display_rotation_t current_rotation;
    SemaphoreHandle_t trans_sem; /* Idle transfer mutex */
    void* fb;                    /* Cyberdeck: DPI frame buffer (direct rotation), NULL = off */
    void* panel_fb;              /* Cyberdeck: DPI frame buffer (rotated strips land here), NULL = not found */
    size_t fb_size;
    size_t rot_bytes;            /* Cyberdeck: rotation buffer size = whole screen (an area is assembled there from strips) */
    size_t buffer_bytes;         /* size of one draw buffer (rotation buffer allocated lazily when needed) */
    struct {
        unsigned int monochrome : 1;   /* True, if display is monochrome and using 1bit for 1px */
        unsigned int swap_bytes : 1;   /* Swap bytes in RGB656 (16-bit) before send to LCD driver */
        unsigned int full_refresh : 1; /* Always make the whole screen redrawn */
        unsigned int direct_mode : 1;  /* Use screen-sized buffers and draw to absolute coordinates */
        unsigned int sw_rotate : 1;    /* Use software rotation (slower) or PPA if available */
    } flags;
} lvgl_port_display_ctx_t;

static lvgl_port_display_ctx_t* s_direct_ctx = NULL;  // Cyberdeck: the display with direct rotation (only one)

/*******************************************************************************
 * Function definitions
 *******************************************************************************/
static lv_display_t* lvgl_port_add_disp_priv(const lvgl_port_display_cfg_t* disp_cfg,
                                             const lvgl_port_disp_priv_cfg_t* priv_cfg);
#if LVGL_PORT_HANDLE_FLUSH_READY
static bool lvgl_port_flush_io_ready_callback(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t* edata,
                                              void* user_ctx);
#if CONFIG_IDF_TARGET_ESP32S3 && ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
static bool lvgl_port_flush_rgb_vsync_ready_callback(esp_lcd_panel_handle_t panel_io,
                                                     const esp_lcd_rgb_panel_event_data_t* edata, void* user_ctx);
#endif
#if (CONFIG_IDF_TARGET_ESP32P4 && ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 3, 0))
static bool lvgl_port_flush_dpi_panel_ready_callback(esp_lcd_panel_handle_t panel_io,
                                                     esp_lcd_dpi_panel_event_data_t* edata, void* user_ctx);
static bool lvgl_port_flush_dpi_vsync_ready_callback(esp_lcd_panel_handle_t panel_io,
                                                     esp_lcd_dpi_panel_event_data_t* edata, void* user_ctx);
static bool lvgl_port_dpi_frame_callback(esp_lcd_panel_handle_t panel, esp_lcd_dpi_panel_event_data_t* edata,
                                         void* user_ctx);
#endif
#endif
static void lvgl_port_flush_callback(lv_display_t* drv, const lv_area_t* area, uint8_t* color_map);
static void lvgl_port_flush_wait(lv_display_t* drv);
static bool lvgl_port_ppa_done(ppa_client_handle_t client, ppa_event_data_t* event, void* user_data);
static void lvgl_port_disp_size_update_callback(lv_event_t* e);
static void lvgl_port_disp_rotation_update(lvgl_port_display_ctx_t* disp_ctx);
static void lvgl_port_display_invalidate_callback(lv_event_t* e);

/*******************************************************************************
 * Public API functions
 *******************************************************************************/

lv_display_t* lvgl_port_add_disp(const lvgl_port_display_cfg_t* disp_cfg)
{
    lvgl_port_lock(0);
    lv_disp_t* disp = lvgl_port_add_disp_priv(disp_cfg, NULL);

    if (disp != NULL) {
        lvgl_port_display_ctx_t* disp_ctx = (lvgl_port_display_ctx_t*)lv_display_get_driver_data(disp);
        /* Set display type */
        disp_ctx->disp_type = LVGL_PORT_DISP_TYPE_OTHER;

        assert(disp_cfg->io_handle != NULL);

#if LVGL_PORT_HANDLE_FLUSH_READY
        const esp_lcd_panel_io_callbacks_t cbs = {
            .on_color_trans_done = lvgl_port_flush_io_ready_callback,
        };
        /* Register done callback */
        esp_lcd_panel_io_register_event_callbacks(disp_ctx->io_handle, &cbs, disp);
#endif

        /* Apply rotation from initial display configuration */
        lvgl_port_disp_rotation_update(disp_ctx);
    }
    lvgl_port_unlock();

    return disp;
}

lv_display_t* lvgl_port_add_disp_dsi(const lvgl_port_display_cfg_t* disp_cfg,
                                     const lvgl_port_display_dsi_cfg_t* dsi_cfg)
{
    // Initialize the PPA
    // Cyberdeck: strip rotations run in the background - several transactions at once in the client queue.
    ppa_client_config_t ppa_srm_config = {
        .oper_type             = PPA_OPERATION_SRM,
        .max_pending_trans_num = 4,
    };
    ESP_ERROR_CHECK(ppa_register_client(&ppa_srm_config, &ppa_srm_handle));
    const ppa_event_callbacks_t ppa_cbs = {.on_trans_done = lvgl_port_ppa_done};
    ESP_ERROR_CHECK(ppa_client_register_event_callbacks(ppa_srm_handle, &ppa_cbs));
    ESP_ERROR_CHECK(esp_cache_get_alignment(MALLOC_CAP_SPIRAM, &data_cache_line_size));

    assert(dsi_cfg != NULL);
    // Cyberdeck: rotate straight into the frame buffer - only without avoid_tearing (frame buffers as LVGL buffers).
    const bool direct = s_direct_requested && !dsi_cfg->flags.avoid_tearing && disp_cfg->flags.sw_rotate;
    const lvgl_port_disp_priv_cfg_t priv_cfg = {
        .avoid_tearing = dsi_cfg->flags.avoid_tearing,
        .direct_fb     = direct,
    };
    lvgl_port_lock(0);
    lv_disp_t* disp = lvgl_port_add_disp_priv(disp_cfg, &priv_cfg);

    if (disp != NULL) {
        lvgl_port_display_ctx_t* disp_ctx = (lvgl_port_display_ctx_t*)lv_display_get_driver_data(disp);
        /* Set display type */
        disp_ctx->disp_type = LVGL_PORT_DISP_TYPE_DSI;
#if (CONFIG_IDF_TARGET_ESP32P4 && ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 3, 0))
        // Cyberdeck: strips rotated in the background into the rotation buffer and the area copied by DMA2D to the
        // frame buffer, or (disp.ppa_direct) straight into the frame buffer. Without a frame buffer - the old path
        // (busy-wait loop).
        if (!dsi_cfg->flags.avoid_tearing && disp_cfg->flags.sw_rotate) {
            void* fb            = NULL;
            const size_t fbSize = (size_t)disp_cfg->hres * disp_cfg->vres * lv_color_format_get_size(
                                      lv_display_get_color_format(disp));
            if (esp_lcd_dpi_panel_get_frame_buffer(disp_ctx->panel_handle, 1, &fb) == ESP_OK && fb != NULL &&
                ((uintptr_t)fb % data_cache_line_size) == 0 && (fbSize % data_cache_line_size) == 0 &&
                fbSize == disp_ctx->rot_bytes && (s_flush_sem = xSemaphoreCreateBinary()) != NULL) {
                disp_ctx->panel_fb = fb;
                disp_ctx->fb_size  = fbSize;
                lv_display_set_flush_wait_cb(disp, lvgl_port_flush_wait);
                if (direct) {
                    disp_ctx->fb = fb;
                    s_direct_ctx = disp_ctx;
                    ESP_LOGI(TAG, "Cyberdeck: PPA rotation straight into the frame buffer (no rotation buffer)");
                }
            } else {
                ESP_LOGE(TAG, "Cyberdeck: frame buffer unusable for strip rotation - plain rotation");
            }
        }
#endif

#if (CONFIG_IDF_TARGET_ESP32P4 && ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 3, 0))
        esp_lcd_dpi_panel_event_callbacks_t cbs = {0};
        if (dsi_cfg->flags.avoid_tearing) {
            cbs.on_refresh_done = lvgl_port_flush_dpi_vsync_ready_callback;
        } else {
            cbs.on_color_trans_done = lvgl_port_flush_dpi_panel_ready_callback;
            // Cyberdeck: counter of frames sent to the panel (the display watchdog in the HAL)
            cbs.on_frame_buf_complete = lvgl_port_dpi_frame_callback;
        }
        /* Register done callback */
        esp_lcd_dpi_panel_register_event_callbacks(disp_ctx->panel_handle, &cbs, disp);

        /* Apply rotation from initial display configuration */
        lvgl_port_disp_rotation_update(disp_ctx);
#else
        ESP_RETURN_ON_FALSE(false, NULL, TAG, "MIPI-DSI is supported only on ESP32P4 and from IDF 5.3!");
#endif
    }
    lvgl_port_unlock();

    return disp;
}

lv_display_t* lvgl_port_add_disp_rgb(const lvgl_port_display_cfg_t* disp_cfg,
                                     const lvgl_port_display_rgb_cfg_t* rgb_cfg)
{
    lvgl_port_lock(0);
    assert(rgb_cfg != NULL);
    const lvgl_port_disp_priv_cfg_t priv_cfg = {
        .avoid_tearing = rgb_cfg->flags.avoid_tearing,
    };
    lv_disp_t* disp = lvgl_port_add_disp_priv(disp_cfg, &priv_cfg);

    if (disp != NULL) {
        lvgl_port_display_ctx_t* disp_ctx = (lvgl_port_display_ctx_t*)lv_display_get_driver_data(disp);
        /* Set display type */
        disp_ctx->disp_type = LVGL_PORT_DISP_TYPE_RGB;

#if (CONFIG_IDF_TARGET_ESP32S3 && ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0))
        /* Register done callback */
        const esp_lcd_rgb_panel_event_callbacks_t vsync_cbs = {
            .on_vsync = lvgl_port_flush_rgb_vsync_ready_callback,
        };

        const esp_lcd_rgb_panel_event_callbacks_t bb_cbs = {
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 1, 2)
            .on_bounce_frame_finish = lvgl_port_flush_rgb_vsync_ready_callback,
#endif
        };

        if (rgb_cfg->flags.bb_mode && (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 1, 2))) {
            ESP_ERROR_CHECK(
                esp_lcd_rgb_panel_register_event_callbacks(disp_ctx->panel_handle, &bb_cbs, disp_ctx->disp_drv));
        } else {
            ESP_ERROR_CHECK(
                esp_lcd_rgb_panel_register_event_callbacks(disp_ctx->panel_handle, &vsync_cbs, disp_ctx->disp_drv));
        }
#else
        ESP_RETURN_ON_FALSE(false, NULL, TAG, "RGB is supported only on ESP32S3 and from IDF 5.0!");
#endif

        /* Apply rotation from initial display configuration */
        lvgl_port_disp_rotation_update(disp_ctx);
    }
    lvgl_port_unlock();

    return disp;
}

esp_err_t lvgl_port_remove_disp(lv_display_t* disp)
{
    assert(disp);
    lvgl_port_display_ctx_t* disp_ctx = (lvgl_port_display_ctx_t*)lv_display_get_driver_data(disp);

    lvgl_port_lock(0);
    lv_disp_remove(disp);
    lvgl_port_unlock();
    if (s_direct_ctx == disp_ctx) {
        s_direct_ctx = NULL;
    }

    if (disp_ctx->draw_buffs[0]) {
        free(disp_ctx->draw_buffs[0]);
    }

    if (disp_ctx->draw_buffs[1]) {
        free(disp_ctx->draw_buffs[1]);
    }

    if (disp_ctx->draw_buffs[2]) {
        free(disp_ctx->draw_buffs[2]);
    }

    if (disp_ctx->oled_buffer) {
        free(disp_ctx->oled_buffer);
    }

    if (disp_ctx->trans_sem) {
        vSemaphoreDelete(disp_ctx->trans_sem);
    }

    free(disp_ctx);

    return ESP_OK;
}

void lvgl_port_flush_ready(lv_display_t* disp)
{
    assert(disp);
    lv_disp_flush_ready(disp);
}

/*******************************************************************************
 * Private functions
 *******************************************************************************/

static lv_display_t* lvgl_port_add_disp_priv(const lvgl_port_display_cfg_t* disp_cfg,
                                             const lvgl_port_disp_priv_cfg_t* priv_cfg)
{
    esp_err_t ret               = ESP_OK;
    lv_display_t* disp          = NULL;
    lv_color_t* buf1            = NULL;
    lv_color_t* buf2            = NULL;
    uint32_t buffer_size        = 0;
    SemaphoreHandle_t trans_sem = NULL;
    assert(disp_cfg != NULL);
    assert(disp_cfg->panel_handle != NULL);
    assert(disp_cfg->buffer_size > 0);
    assert(disp_cfg->hres > 0);
    assert(disp_cfg->vres > 0);

    buffer_size = disp_cfg->buffer_size;

    /* Check supported display color formats */
    ESP_RETURN_ON_FALSE(
        disp_cfg->color_format == 0 || disp_cfg->color_format == LV_COLOR_FORMAT_RGB565 ||
            disp_cfg->color_format == LV_COLOR_FORMAT_RGB888 || disp_cfg->color_format == LV_COLOR_FORMAT_XRGB8888 ||
            disp_cfg->color_format == LV_COLOR_FORMAT_ARGB8888 || disp_cfg->color_format == LV_COLOR_FORMAT_I1,
        NULL, TAG, "Not supported display color format!");

    lv_color_format_t display_color_format =
        (disp_cfg->color_format != 0 ? disp_cfg->color_format : LV_COLOR_FORMAT_RGB565);
    uint8_t color_bytes = lv_color_format_get_size(display_color_format);
    if (disp_cfg->flags.swap_bytes) {
        /* Swap bytes can be used only in RGB565 color format */
        ESP_RETURN_ON_FALSE(display_color_format == LV_COLOR_FORMAT_RGB565, NULL, TAG,
                            "Swap bytes can be used only in display color format RGB565!");
    }

    if (disp_cfg->flags.buff_dma) {
        /* DMA buffer can be used only in RGB565 color format */
        ESP_RETURN_ON_FALSE(display_color_format == LV_COLOR_FORMAT_RGB565, NULL, TAG,
                            "DMA buffer can be used only in display color format RGB565 (not aligned copy)!");
    }

    /* Display context */
    lvgl_port_display_ctx_t* disp_ctx = malloc(sizeof(lvgl_port_display_ctx_t));
    ESP_GOTO_ON_FALSE(disp_ctx, ESP_ERR_NO_MEM, err, TAG, "Not enough memory for display context allocation!");
    memset(disp_ctx, 0, sizeof(lvgl_port_display_ctx_t));
    disp_ctx->io_handle         = disp_cfg->io_handle;
    disp_ctx->panel_handle      = disp_cfg->panel_handle;
    disp_ctx->control_handle    = disp_cfg->control_handle;
    disp_ctx->rotation.swap_xy  = disp_cfg->rotation.swap_xy;
    disp_ctx->rotation.mirror_x = disp_cfg->rotation.mirror_x;
    disp_ctx->rotation.mirror_y = disp_cfg->rotation.mirror_y;
    disp_ctx->flags.swap_bytes  = disp_cfg->flags.swap_bytes;
    disp_ctx->flags.sw_rotate   = disp_cfg->flags.sw_rotate;
    disp_ctx->current_rotation  = LV_DISPLAY_ROTATION_0;

    uint32_t buff_caps = 0;
#if SOC_PSRAM_DMA_CAPABLE == 0
    if (disp_cfg->flags.buff_dma && disp_cfg->flags.buff_spiram) {
        ESP_GOTO_ON_FALSE(false, ESP_ERR_NOT_SUPPORTED, err, TAG,
                          "Alloc DMA capable buffer in SPIRAM is not supported!");
    }
#endif
    if (disp_cfg->flags.buff_dma) {
        buff_caps |= MALLOC_CAP_DMA;
    }
    if (disp_cfg->flags.buff_spiram) {
        buff_caps |= MALLOC_CAP_SPIRAM;
    }
    if (buff_caps == 0) {
        buff_caps |= MALLOC_CAP_DEFAULT;
    }

    /* Use RGB internal buffers for avoid tearing effect */
    if (priv_cfg && priv_cfg->avoid_tearing) {
#if CONFIG_IDF_TARGET_ESP32S3 && ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
        buffer_size = disp_cfg->hres * disp_cfg->vres;
        ESP_GOTO_ON_ERROR(esp_lcd_rgb_panel_get_frame_buffer(disp_cfg->panel_handle, 2, (void*)&buf1, (void*)&buf2),
                          err, TAG, "Get RGB buffers failed");
#elif CONFIG_IDF_TARGET_ESP32P4 && ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 3, 0)
        buffer_size = disp_cfg->hres * disp_cfg->vres;
        ESP_GOTO_ON_ERROR(esp_lcd_dpi_panel_get_frame_buffer(disp_cfg->panel_handle, 2, (void*)&buf1, (void*)&buf2),
                          err, TAG, "Get RGB buffers failed");
#endif

        trans_sem = xSemaphoreCreateCounting(1, 0);
        ESP_GOTO_ON_FALSE(trans_sem, ESP_ERR_NO_MEM, err, TAG, "Failed to create transport counting Semaphore");
        disp_ctx->trans_sem = trans_sem;
    } else {
        /* alloc draw buffers used by LVGL */
        /* it's recommended to choose the size of the draw buffer(s) to be at least 1/10 screen sized */
        buf1 = heap_caps_aligned_alloc(CONFIG_LV_DRAW_BUF_ALIGN, buffer_size * color_bytes, buff_caps);
        ESP_GOTO_ON_FALSE(buf1, ESP_ERR_NO_MEM, err, TAG, "Not enough memory for LVGL buffer (buf1) allocation!");
        if (disp_cfg->double_buffer) {
            buf2 = heap_caps_aligned_alloc(CONFIG_LV_DRAW_BUF_ALIGN, buffer_size * color_bytes, buff_caps);
            ESP_GOTO_ON_FALSE(buf2, ESP_ERR_NO_MEM, err, TAG, "Not enough memory for LVGL buffer (buf2) allocation!");
        }

        disp_ctx->draw_buffs[0] = buf1;
        disp_ctx->draw_buffs[1] = buf2;
    }

    disp = lv_display_create(disp_cfg->hres, disp_cfg->vres);

    /* Set display color format */
    lv_display_set_color_format(disp, display_color_format);

    /* Monochrome display settings */
    if (disp_cfg->monochrome) {
#if CONFIG_LV_COLOR_DEPTH_1
#error please disable LV_COLOR_DEPTH_1 for using monochromatic screen
#endif

        /* Monochrome can be used only in RGB565 color format */
        ESP_RETURN_ON_FALSE(
            display_color_format == LV_COLOR_FORMAT_RGB565 || display_color_format == LV_COLOR_FORMAT_I1, NULL, TAG,
            "Monochrome can be used only in display color format RGB565 or I1!");

        /* When using monochromatic display, there must be used full bufer! */
        ESP_GOTO_ON_FALSE((disp_cfg->hres * disp_cfg->vres == buffer_size), ESP_ERR_INVALID_ARG, err, TAG,
                          "Monochromatic display must using full buffer!");

        disp_ctx->flags.monochrome = 1;
        lv_display_set_buffers(disp, buf1, buf2, buffer_size * color_bytes, LV_DISPLAY_RENDER_MODE_FULL);

        if (display_color_format == LV_COLOR_FORMAT_I1) {
            /* OLED monochrome buffer */
            // To use LV_COLOR_FORMAT_I1, we need an extra buffer to hold the converted data
            disp_ctx->oled_buffer = heap_caps_malloc(buffer_size, buff_caps);
            ESP_GOTO_ON_FALSE(disp_ctx->oled_buffer, ESP_ERR_NO_MEM, err, TAG,
                              "Not enough memory for LVGL buffer (OLED buffer) allocation!");
        }

    } else if (disp_cfg->flags.direct_mode) {
        /* When using direct_mode, there must be used full bufer! */
        ESP_GOTO_ON_FALSE((disp_cfg->hres * disp_cfg->vres == buffer_size), ESP_ERR_INVALID_ARG, err, TAG,
                          "Direct mode must using full buffer!");

        disp_ctx->flags.direct_mode = 1;
        lv_display_set_buffers(disp, buf1, buf2, buffer_size * color_bytes, LV_DISPLAY_RENDER_MODE_DIRECT);
    } else if (disp_cfg->flags.full_refresh) {
        /* When using full_refresh, there must be used full bufer! */
        ESP_GOTO_ON_FALSE((disp_cfg->hres * disp_cfg->vres == buffer_size), ESP_ERR_INVALID_ARG, err, TAG,
                          "Full refresh must using full buffer!");

        disp_ctx->flags.full_refresh = 1;
        lv_display_set_buffers(disp, buf1, buf2, buffer_size * color_bytes, LV_DISPLAY_RENDER_MODE_FULL);
    } else {
        lv_display_set_buffers(disp, buf1, buf2, buffer_size * color_bytes, LV_DISPLAY_RENDER_MODE_PARTIAL);
        // lv_display_set_buffers(disp, buf1, buf2, buffer_size * color_bytes, LV_DISPLAY_RENDER_MODE_FULL);
    }

    lv_display_set_flush_cb(disp, lvgl_port_flush_callback);
    lv_display_add_event_cb(disp, lvgl_port_disp_size_update_callback, LV_EVENT_RESOLUTION_CHANGED, disp_ctx);
    lv_display_add_event_cb(disp, lvgl_port_display_invalidate_callback, LV_EVENT_INVALIDATE_AREA, disp_ctx);
    lv_display_add_event_cb(disp, lvgl_port_display_invalidate_callback, LV_EVENT_REFR_REQUEST, disp_ctx);

    lv_display_set_driver_data(disp, disp_ctx);
    disp_ctx->disp_drv = disp;

    /* Use SW rotation */
    disp_ctx->buffer_bytes = buffer_size * color_bytes;
    // Cyberdeck: the rotation buffer is full-screen even with small draw buffers - an area (up to the whole screen)
    // is assembled in it from strips. PPA output: address and size aligned to the cache line.
    disp_ctx->rot_bytes = (size_t)disp_cfg->hres * disp_cfg->vres * color_bytes;
    // Cyberdeck: with rotation straight into the frame buffer the rotation buffer is created only when needed
    // (a rotation other than 90 degrees, or a fallback after a PPA error) - usually never (saves 1.84 MB of PSRAM).
    if (disp_cfg->flags.sw_rotate && !(priv_cfg && priv_cfg->direct_fb)) {
        disp_ctx->draw_buffs[2] = heap_caps_aligned_alloc(128, disp_ctx->rot_bytes, buff_caps);
        ESP_GOTO_ON_FALSE(disp_ctx->draw_buffs[2], ESP_ERR_NO_MEM, err, TAG,
                          "Not enough memory for LVGL buffer (rotation buffer) allocation!");
    }

err:
    if (ret != ESP_OK) {
        if (disp_ctx->draw_buffs[0]) {
            free(disp_ctx->draw_buffs[0]);
        }
        if (disp_ctx->draw_buffs[1]) {
            free(disp_ctx->draw_buffs[1]);
        }
        if (disp_ctx->draw_buffs[2]) {
            free(disp_ctx->draw_buffs[2]);
        }
        if (disp_ctx->oled_buffer) {
            free(disp_ctx->oled_buffer);
        }
        if (disp_ctx) {
            free(disp_ctx);
        }
        if (trans_sem) {
            vSemaphoreDelete(trans_sem);
        }
    }

    return disp;
}

#if LVGL_PORT_HANDLE_FLUSH_READY
static bool lvgl_port_flush_io_ready_callback(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t* edata,
                                              void* user_ctx)
{
    lv_display_t* disp_drv = (lv_display_t*)user_ctx;
    assert(disp_drv != NULL);
    lv_disp_flush_ready(disp_drv);
    return false;
}

#if (CONFIG_IDF_TARGET_ESP32P4 && ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 3, 0))
// Cyberdeck: end of a background flush - from an interrupt (PPA, DMA2D) or from a task (recovering a hung SRM).
static bool lvgl_port_flush_finished(lv_display_t* disp)
{
    s_flush_busy = false;
    s_flush_done++;
    lv_display_flush_ready(disp);
    BaseType_t yield = pdFALSE;
    if (s_flush_sem != NULL) {
        xSemaphoreGiveFromISR(s_flush_sem, &yield);
    }
    return yield == pdTRUE;
}

// Cyberdeck: PPA transaction finished. user_data = the display when it ends the flush; NULL - a blocking rotation or
// the last strip of an area, after which the area copy to the frame buffer still follows.
static bool lvgl_port_ppa_done(ppa_client_handle_t client, ppa_event_data_t* event, void* user_data)
{
    return user_data != NULL ? lvgl_port_flush_finished((lv_display_t*)user_data) : false;
}

// Cyberdeck: every frame the DMA sent to the panel (end of the frame buffer). A counter that stays constant = the
// video stream stalled (e.g. after a DSI bridge underrun) - the screen is lit but shows no picture. Read by the
// watchdog in the HAL.
static volatile uint32_t s_dpi_frames = 0;

static bool IRAM_ATTR lvgl_port_dpi_frame_callback(esp_lcd_panel_handle_t panel, esp_lcd_dpi_panel_event_data_t* edata,
                                                   void* user_ctx)
{
    s_dpi_frames++;
    return false;
}

uint32_t lvgl_port_dpi_frame_count(void)
{
    return s_dpi_frames;
}

static bool lvgl_port_flush_dpi_panel_ready_callback(esp_lcd_panel_handle_t panel_io,
                                                     esp_lcd_dpi_panel_event_data_t* edata, void* user_ctx)
{
    lv_display_t* disp_drv = (lv_display_t*)user_ctx;
    assert(disp_drv != NULL);
    return lvgl_port_flush_finished(disp_drv);  // Cyberdeck: pixels are in the frame buffer (DMA2D copy done)
}

static bool lvgl_port_flush_dpi_vsync_ready_callback(esp_lcd_panel_handle_t panel_io,
                                                     esp_lcd_dpi_panel_event_data_t* edata, void* user_ctx)
{
    BaseType_t need_yield = pdFALSE;

    lv_display_t* disp_drv = (lv_display_t*)user_ctx;
    assert(disp_drv != NULL);
    lvgl_port_display_ctx_t* disp_ctx = (lvgl_port_display_ctx_t*)lv_display_get_driver_data(disp_drv);
    assert(disp_ctx != NULL);

    if (disp_ctx->trans_sem) {
        xSemaphoreGiveFromISR(disp_ctx->trans_sem, &need_yield);
    }

    return (need_yield == pdTRUE);
}
#endif

#if CONFIG_IDF_TARGET_ESP32S3 && ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
static bool lvgl_port_flush_rgb_vsync_ready_callback(esp_lcd_panel_handle_t panel_io,
                                                     const esp_lcd_rgb_panel_event_data_t* edata, void* user_ctx)
{
    BaseType_t need_yield = pdFALSE;

    lv_display_t* disp_drv = (lv_display_t*)user_ctx;
    assert(disp_drv != NULL);
    lvgl_port_display_ctx_t* disp_ctx = (lvgl_port_display_ctx_t*)lv_display_get_driver_data(disp_drv);
    assert(disp_ctx != NULL);

    if (disp_ctx->trans_sem) {
        xSemaphoreGiveFromISR(disp_ctx->trans_sem, &need_yield);
    }

    return (need_yield == pdTRUE);
}
#endif
#endif

static void _lvgl_port_transform_monochrome(lv_display_t* display, const lv_area_t* area, uint8_t** color_map)
{
    assert(color_map);
    assert(*color_map);
    uint8_t* src                      = *color_map;
    lv_color16_t* color               = (lv_color16_t*)*color_map;
    lvgl_port_display_ctx_t* disp_ctx = (lvgl_port_display_ctx_t*)lv_display_get_driver_data(display);
    uint16_t hor_res                  = lv_display_get_physical_horizontal_resolution(display);
    uint16_t ver_res                  = lv_display_get_physical_vertical_resolution(display);
    uint16_t res                      = hor_res;
    bool swap_xy                      = (lv_display_get_rotation(display) == LV_DISPLAY_ROTATION_90 ||
                    lv_display_get_rotation(display) == LV_DISPLAY_ROTATION_270);

    int x1 = area->x1;
    int x2 = area->x2;
    int y1 = area->y1;
    int y2 = area->y2;

    lv_color_format_t color_format = lv_display_get_color_format(display);
    if (color_format == LV_COLOR_FORMAT_I1) {
        // This is necessary because LVGL reserves 2 x 4 bytes in the buffer, as these are assumed to be used as a
        // palette. Skip the palette here More information about the monochrome, please refer to
        // https://docs.lvgl.io/9.2/porting/display.html#monochrome-displays
        src += 8;
        /*Use oled_buffer as output */
        *color_map = disp_ctx->oled_buffer;
    }

    int out_x, out_y;
    for (int y = y1; y <= y2; y++) {
        for (int x = x1; x <= x2; x++) {
            bool chroma_color = 0;
            if (color_format == LV_COLOR_FORMAT_I1) {
                chroma_color = (src[(hor_res >> 3) * y + (x >> 3)] & 1 << (7 - x % 8));
            } else {
                chroma_color = (color[hor_res * y + x].blue > 16);
            }

            if (swap_xy) {
                out_x = y;
                out_y = x;
                res   = ver_res;
            } else {
                out_x = x;
                out_y = y;
                res   = hor_res;
            }

            /* Write to the buffer as required for the display.
             * It writes only 1-bit for monochrome displays mapped vertically.*/
            uint8_t* outbuf = NULL;
            outbuf          = *color_map + res * (out_y >> 3) + (out_x);
            if (chroma_color) {
                (*outbuf) &= ~(1 << (out_y % 8));
            } else {
                (*outbuf) |= (1 << (out_y % 8));
            }
        }
    }
}

void lvgl_port_rotate_area(lv_display_t* disp, lv_area_t* area)
{
    lv_display_rotation_t rotation = lv_display_get_rotation(disp);

    int32_t w = lv_area_get_width(area);
    int32_t h = lv_area_get_height(area);

    int32_t hres = lv_display_get_horizontal_resolution(disp);
    int32_t vres = lv_display_get_vertical_resolution(disp);
    if (rotation == LV_DISPLAY_ROTATION_90 || rotation == LV_DISPLAY_ROTATION_270) {
        vres = lv_display_get_horizontal_resolution(disp);
        hres = lv_display_get_vertical_resolution(disp);
    }

    switch (rotation) {
        case LV_DISPLAY_ROTATION_0:
            return;
        case LV_DISPLAY_ROTATION_90:
            area->y2 = vres - area->x1 - 1;
            area->x1 = area->y1;
            area->x2 = area->x1 + h - 1;
            area->y1 = area->y2 - w + 1;
            break;
        case LV_DISPLAY_ROTATION_180:
            area->y2 = vres - area->y1 - 1;
            area->y1 = area->y2 - h + 1;
            area->x2 = hres - area->x1 - 1;
            area->x1 = area->x2 - w + 1;
            break;
        case LV_DISPLAY_ROTATION_270:
            area->x1 = hres - area->y2 - 1;
            area->y2 = area->x2;
            area->x2 = area->x1 + h - 1;
            area->y1 = area->y2 - w + 1;
            break;
    }
}

// Cyberdeck: returns an error instead of ESP_ERROR_CHECK (abort) - the caller then rotates on the CPU.
IRAM_ATTR static esp_err_t rotate_copy_pixel(const uint16_t* from, uint16_t* to, uint16_t x_start, uint16_t y_start,
                                             uint16_t x_end, uint16_t y_end, uint16_t w, uint16_t h, uint16_t rotation)
{
    ppa_srm_rotation_angle_t ppa_rotation;
    int x_offset = 0, y_offset = 0;

    // Determine rotation settings once and reuse
    switch (rotation) {
        case 90:
            ppa_rotation = PPA_SRM_ROTATION_ANGLE_270;
            x_offset     = h - y_end - 1;
            y_offset     = x_start;
            break;
        case 180:
            ppa_rotation = PPA_SRM_ROTATION_ANGLE_180;
            x_offset     = w - x_end - 1;
            y_offset     = h - y_end - 1;
            break;
        case 270:
            ppa_rotation = PPA_SRM_ROTATION_ANGLE_90;
            x_offset     = y_start;
            y_offset     = w - x_end - 1;
            break;
        default:
            ppa_rotation = PPA_SRM_ROTATION_ANGLE_0;
            break;
    }

    // Fill operation config for PPA rotation, without recalculating each time
    ppa_srm_oper_config_t oper_config = {
        .in.buffer         = from,
        .in.pic_w          = w,
        .in.pic_h          = h,
        .in.block_w        = x_end - x_start + 1,
        .in.block_h        = y_end - y_start + 1,
        .in.block_offset_x = x_start,
        .in.block_offset_y = y_start,
        .in.srm_cm         = (LV_COLOR_DEPTH == 24) ? PPA_SRM_COLOR_MODE_RGB888 : PPA_SRM_COLOR_MODE_RGB565,

        .out.buffer      = to,
        .out.buffer_size = ALIGN_UP_BY(sizeof(lv_color_t) * w * h, data_cache_line_size),
        .out.pic_w = (ppa_rotation == PPA_SRM_ROTATION_ANGLE_90 || ppa_rotation == PPA_SRM_ROTATION_ANGLE_270) ? h : w,
        .out.pic_h = (ppa_rotation == PPA_SRM_ROTATION_ANGLE_90 || ppa_rotation == PPA_SRM_ROTATION_ANGLE_270) ? w : h,
        .out.block_offset_x = x_offset,
        .out.block_offset_y = y_offset,
        .out.srm_cm         = (LV_COLOR_DEPTH == 24) ? PPA_SRM_COLOR_MODE_RGB888 : PPA_SRM_COLOR_MODE_RGB565,

        .rotation_angle = ppa_rotation,
        .scale_x        = 1.0,
        .scale_y        = 1.0,
        .rgb_swap       = 0,
        .byte_swap      = 0,
        .mode           = PPA_TRANS_MODE_BLOCKING,
    };

    return ppa_do_scale_rotate_mirror(ppa_srm_handle, &oper_config);
}

// Cyberdeck: whether to rotate with the PPA (not during the pause after an error), and recording of a PPA error.
static bool lvgl_port_ppa_allowed(void)
{
    if (s_ppa_paused && (xTaskGetTickCount() - s_ppa_pause_from) >= pdMS_TO_TICKS(LVGL_PORT_PPA_PAUSE_MS)) {
        s_ppa_paused = false;
        ESP_LOGW(TAG, "Cyberdeck: rotating with the PPA again");
    }
    return !s_ppa_paused;
}

static void lvgl_port_ppa_failed(esp_err_t err)
{
    s_ppa_failures++;
    s_ppa_paused     = true;
    s_ppa_pause_from = xTaskGetTickCount();
    ESP_LOGE(TAG, "Cyberdeck: PPA rotation: %s (failure #%u) - rotating on the CPU for %d ms", esp_err_to_name(err),
             (unsigned)s_ppa_failures, LVGL_PORT_PPA_PAUSE_MS);
}

// Cyberdeck: the rotation buffer is created only when needed (direct mode has none).
static bool lvgl_port_rotation_buffer(lvgl_port_display_ctx_t* disp_ctx)
{
    if (disp_ctx->draw_buffs[2] == NULL) {
        disp_ctx->draw_buffs[2] = heap_caps_aligned_alloc(128, disp_ctx->rot_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA);
        if (disp_ctx->draw_buffs[2] == NULL) {
            ESP_LOGE(TAG, "Cyberdeck: out of memory for the rotation buffer (%u B)", (unsigned)disp_ctx->rot_bytes);
            return false;
        }
    }
    return true;
}

/* ---------------- Cyberdeck: LVGL strips rotated in the background ----------------
 * LVGL renders an area in strips (small draw buffers instead of two full-screen ones); the PPA rotates strip N in
 * the background while LVGL renders strip N+1 - the rotation (formerly ~44 ms for a whole screen, after rendering)
 * runs in parallel with rendering. Normally the strips of an area are assembled in the rotation buffer (the rotated
 * area, without gaps - as the whole area used to be at once), and after the last strip DMA2D copies the area to the
 * frame buffer: the area appears on the panel at once, as when rendered whole. disp.ppa_direct: strips straight into
 * the frame buffer (no copy, ~13 ms faster for a whole screen; the panel may show half an area - tearing). The panel
 * reads the frame buffer as 720 x 1280, a UI row = a panel column.
 * The PPA pause after an error (lvgl_port_ppa_allowed): the same on the CPU. */

// 90 degree rotation of a strip by the PPA into block b of a cols x rows buffer (rotation buffer or frame buffer).
// done = the display, when the end of the transaction ends the flush (lvgl_port_ppa_done); blocking - waits for the
// end (last strip of an area).
static esp_err_t lvgl_port_ppa_rot90(const uint8_t* px, int32_t w, int32_t h, void* out, size_t out_size,
                                     int32_t cols, int32_t rows, lvgl_port_fb_block_t b, void* done, bool blocking)
{
    ppa_srm_oper_config_t c = {
        .in.buffer          = px,
        .in.pic_w           = w,  // LVGL buffer without padding between rows (LV_DRAW_BUF_STRIDE_ALIGN = 1)
        .in.pic_h           = h,
        .in.block_w         = w,
        .in.block_h         = h,
        .in.srm_cm          = (LV_COLOR_DEPTH == 24) ? PPA_SRM_COLOR_MODE_RGB888 : PPA_SRM_COLOR_MODE_RGB565,
        .out.buffer         = out,
        .out.buffer_size    = out_size,
        .out.pic_w          = cols,
        .out.pic_h          = rows,
        .out.block_offset_x = b.col,
        .out.block_offset_y = b.row,
        .out.srm_cm         = (LV_COLOR_DEPTH == 24) ? PPA_SRM_COLOR_MODE_RGB888 : PPA_SRM_COLOR_MODE_RGB565,
        .rotation_angle     = PPA_SRM_ROTATION_ANGLE_90,  // like rotate_copy_pixel(..., 270) in the plain path
        .scale_x            = 1.0,
        .scale_y            = 1.0,
        .mode               = blocking ? PPA_TRANS_MODE_BLOCKING : PPA_TRANS_MODE_NON_BLOCKING,
        .user_data          = done,
    };
    return ppa_do_scale_rotate_mirror(ppa_srm_handle, &c);
}

// The full LVGL area whose first strip is `strip` (LVGL splits an area from the top into strips with the same x).
static bool lvgl_port_find_area(lv_display_t* drv, const lv_area_t* strip, lv_area_t* out)
{
    const int32_t ver = lv_display_get_vertical_resolution(drv);
    for (int32_t i = 0; i < (int32_t)drv->inv_p; i++) {
        const lv_area_t* a = &drv->inv_areas[i];
        if (!drv->inv_area_joined[i] && a->x1 == strip->x1 && a->x2 == strip->x2 && a->y1 == strip->y1 &&
            a->y2 >= strip->y2) {
            *out = *a;
            if (out->y2 >= ver) {
                out->y2 = ver - 1;  // like refr_area: the area clipped to the screen
            }
            return true;
        }
    }
    return false;
}

// Write the cache back to PSRAM after the CPU wrote into the frame buffer (the panel reads PSRAM).
static void lvgl_port_rows_writeback(uint8_t* buf, int32_t cols, lvgl_port_fb_block_t b, uint32_t bpp)
{
    esp_cache_msync(buf + (size_t)b.row * cols * bpp, (size_t)b.h * cols * bpp,
                    ESP_CACHE_MSYNC_FLAG_DIR_C2M | ESP_CACHE_MSYNC_FLAG_UNALIGNED);
}

// An LVGL strip (90 degree rotation) into the rotation buffer or straight into the frame buffer. false = the old path
// (block outside the screen, no rotation buffer).
static bool lvgl_port_flush_strip(lvgl_port_display_ctx_t* ctx, lv_display_t* drv, const lv_area_t* area,
                                  uint8_t* px)
{
    // 90 degree rotation: UI row = panel column (720), UI column = panel row (1280). Not
    // lv_display_get_physical_*: under rotation they return the pre-rotation dimensions only without physical_*_res.
    const int32_t cols           = lv_display_get_vertical_resolution(drv);
    const int32_t rows           = lv_display_get_horizontal_resolution(drv);
    const lvgl_port_fb_block_t b = lvgl_port_rot90_block(area->x1, area->y1, area->x2, area->y2, rows);
    if (!lvgl_port_fb_block_fits(b, cols, rows)) {
        ESP_LOGE(TAG, "Cyberdeck: area (%d,%d)-(%d,%d) outside the frame buffer", (int)area->x1, (int)area->y1,
                 (int)area->x2, (int)area->y2);
        return false;
    }
    const lv_color_format_t cf = lv_display_get_color_format(drv);
    const uint32_t bpp         = lv_color_format_get_size(cf);
    const int32_t w            = lv_area_get_width(area);
    const int32_t h            = lv_area_get_height(area);
    const bool ppa             = lvgl_port_ppa_allowed();

    if (ctx->fb != NULL) {  // disp.ppa_direct: strip straight into the frame buffer
        if (ppa) {
            s_flush_busy        = true;
            const esp_err_t err = lvgl_port_ppa_rot90(px, w, h, ctx->fb, ctx->fb_size, cols, rows, b, drv, false);
            if (err == ESP_OK) {
                return true;
            }
            s_flush_busy = false;
            lvgl_port_ppa_failed(err);
            ctx->fb = NULL;  // after an error, the plain path for good (from the next area)
            ESP_LOGE(TAG, "Cyberdeck: PPA rotation into the frame buffer: %s - falling back to the rotation buffer",
                     esp_err_to_name(err));
        }
        uint8_t* fb = ctx->panel_fb;
        lv_draw_sw_rotate(px, fb + ((size_t)b.row * cols + b.col) * bpp, w, h, lv_draw_buf_width_to_stride(w, cf),
                          cols * bpp, LV_DISPLAY_ROTATION_90, cf);
        lvgl_port_rows_writeback(fb, cols, b, bpp);
        s_flush_done++;
        lv_display_flush_ready(drv);
        return true;
    }

    if (!lvgl_port_rotation_buffer(ctx)) {
        return false;
    }
    // An area assembled in the rotation buffer: the first strip defines the area, later ones must belong to it.
    if (!s_area_open) {
        if (!lvgl_port_find_area(drv, area, &s_area)) {
            s_area = *area;  // unknown area: the strip as a separate area (appears at once)
        }
        s_area_open = true;
    }
    lvgl_port_fb_block_t sb = {0};
    if (!lvgl_port_strip_block(s_area.x1, s_area.y1, s_area.x2, s_area.y2, area->x1, area->y1, area->x2, area->y2,
                               &sb)) {
        ESP_LOGW(TAG, "Cyberdeck: strip (%d,%d)-(%d,%d) outside the assembled area - restarting", (int)area->x1,
                 (int)area->y1, (int)area->x2, (int)area->y2);
        s_area = *area;
        lvgl_port_strip_block(s_area.x1, s_area.y1, s_area.x2, s_area.y2, area->x1, area->y1, area->x2, area->y2,
                              &sb);
    }
    const bool last     = drv->last_part != 0 || area->y2 >= s_area.y2;  // last strip of the area
    const int32_t acols = lv_area_get_height(&s_area);                    // rotated area: columns
    const int32_t arows = lv_area_get_width(&s_area);                     //                    rows
    uint8_t* rot        = (uint8_t*)ctx->draw_buffs[2];

    if (!last && ppa) {
        s_flush_busy        = true;
        const esp_err_t err = lvgl_port_ppa_rot90(px, w, h, rot, ctx->rot_bytes, acols, arows, sb, drv, false);
        if (err == ESP_OK) {
            return true;
        }
        s_flush_busy = false;
        lvgl_port_ppa_failed(err);
    }
    // Last strip: a blocking rotation (one strip, a few ms), then the area copy by DMA2D - its end ends the flush.
    esp_err_t err = ESP_ERR_INVALID_STATE;
    if (last && ppa) {
        err = lvgl_port_ppa_rot90(px, w, h, rot, ctx->rot_bytes, acols, arows, sb, NULL, true);
        if (err != ESP_OK) {
            lvgl_port_ppa_failed(err);
        }
    }
    if (err != ESP_OK) {  // CPU (PPA pause after an error); the DMA2D copy writes back the source cache itself
        lv_draw_sw_rotate(px, rot + (size_t)sb.col * bpp, w, h, lv_draw_buf_width_to_stride(w, cf), acols * bpp,
                          LV_DISPLAY_ROTATION_90, cf);
        // Before every operation the PPA invalidates (without writeback) the output window - dirty CPU lines would be
        // lost.
        esp_cache_msync(rot, (size_t)acols * arows * bpp, ESP_CACHE_MSYNC_FLAG_DIR_C2M | ESP_CACHE_MSYNC_FLAG_UNALIGNED);
    }
    if (!last) {
        s_flush_done++;
        lv_display_flush_ready(drv);
        return true;
    }
    s_area_open                  = false;
    const lvgl_port_fb_block_t a = lvgl_port_rot90_block(s_area.x1, s_area.y1, s_area.x2, s_area.y2, rows);
    s_flush_busy                 = true;  // end of the DMA2D copy: lvgl_port_flush_dpi_panel_ready_callback
    if (esp_lcd_panel_draw_bitmap(ctx->panel_handle, a.col, a.row, a.col + a.w, a.row + a.h, rot) != ESP_OK) {
        ESP_LOGE(TAG, "Cyberdeck: area copy to the frame buffer rejected");
        s_flush_busy = false;
        s_flush_done++;
        lv_display_flush_ready(drv);
    }
    return true;
}

// Cyberdeck: LVGL waits for the previous flush to end (lv_display_set_flush_wait_cb): on a semaphore, not in a loop.
// Hung SRM (esp-idf #19096): after 300 ms recovery (ends the transaction and the flush), after 1.5 s a forced end.
static void lvgl_port_flush_wait(lv_display_t* drv)
{
    const TickType_t t0 = xTaskGetTickCount();
    bool recovered      = false;
    while (s_flush_busy) {
        if (xSemaphoreTake(s_flush_sem, pdMS_TO_TICKS(50)) == pdTRUE) {
            continue;
        }
        const uint32_t ms = (uint32_t)((xTaskGetTickCount() - t0) * portTICK_PERIOD_MS);
        if (!recovered && ms >= 300) {
            recovered = true;
            ppa_cyberdeck_srm_recover(250);
        } else if (ms >= 1500) {
            ESP_LOGE(TAG, "Cyberdeck: flush did not finish in %u ms - forcing the end", (unsigned)ms);
            s_flush_busy = false;
            s_flush_done = s_flush_submitted;
            lvgl_port_ppa_failed(ESP_ERR_TIMEOUT);
        }
    }
}

void lvgl_port_set_direct_rotation(bool on)
{
    s_direct_requested = on;
}

bool lvgl_port_direct_rotation(void)
{
    return s_direct_ctx != NULL && s_direct_ctx->fb != NULL;
}

void lvgl_port_flush_counts(uint32_t* submitted, uint32_t* done)
{
    if (submitted != NULL) {
        *submitted = s_flush_submitted;
    }
    if (done != NULL) {
        *done = s_flush_done;
    }
}

int32_t lvgl_port_set_strip_lines(lv_display_t* disp, int32_t lines)
{
    lvgl_port_display_ctx_t* disp_ctx = (lvgl_port_display_ctx_t*)lv_display_get_driver_data(disp);
    if (disp_ctx == NULL || disp_ctx->draw_buffs[0] == NULL || disp_ctx->trans_sem != NULL) {
        return 0;  // frame buffers used as LVGL buffers (avoid_tearing) - no strips
    }
    const lv_color_format_t cf = lv_display_get_color_format(disp);
    const uint32_t stride      = lv_draw_buf_width_to_stride(lv_display_get_horizontal_resolution(disp), cf);
    const int32_t max_lines    = (int32_t)(disp_ctx->buffer_bytes / stride);
    if (lines <= 0 || lines > max_lines) {
        lines = max_lines;
    }
    lines = lines >= 4 ? (lines & ~3) : 4;  // multiple of 4 rows (area rounding, esp-idf #19096)
    lv_display_set_buffers(disp, disp_ctx->draw_buffs[0], disp_ctx->draw_buffs[1], (uint32_t)lines * stride,
                           LV_DISPLAY_RENDER_MODE_PARTIAL);
    return lines;
}

uint64_t lvgl_port_flush_time_us(void)
{
    return s_flush_cb_us;
}

static void lvgl_port_flush_callback_impl(lv_display_t* drv, const lv_area_t* area, uint8_t* color_map);

/* Cyberdeck: dimming the picture in the flush instead of an LVGL black layer over the whole screen (a rectangle
 * with opacity: lv_color_16_16_mix called separately for every pixel, ~34 ms for a full redraw).
 * The result is bit-exact with LVGL: for black, lv_color_16_16_mix(0, c, opa) is floor(v * k / 32) in every channel,
 * k = 32 - ((opa + 4) >> 3) (checked on the device for all colours and opacities), thresholds
 * LV_OPA_MIN / LV_OPA_MAX as when drawing a rectangle. Per-pixel on the CPU it is
 * no faster than LVGL (64 K colour table: 32 ms, arithmetic: 47 ms) - hence PIE: 8 pixels at a time, each channel
 * cut out by a mask ((c & mask) * k) >> 5 in a single esp.vmul.u16 (32-bit product, then SAR = 5). */
static uint8_t s_dim_opa = 0;

bool lvgl_port_set_dim(uint8_t opa)
{
#if LV_COLOR_DEPTH == 16
    s_dim_opa = opa;
    return true;
#else
    (void)opa;
    s_dim_opa = 0;
    return false;
#endif
}

static inline uint16_t lvgl_port_dim_px(uint32_t c, uint32_t k)
{
    return (uint16_t)(((((c & 0xF800u) * k) >> 5) & 0xF800u) | ((((c & 0x07E0u) * k) >> 5) & 0x07E0u) |
                      (((c & 0x001Fu) * k) >> 5));
}

#if CONFIG_IDF_TARGET_ESP32P4
// PIE instruction operands work only in some of the general registers: bind to a0..a3 right before each asm
// (the same pattern as the original firmware's other PIE code).
#define DIM_PIE_R(n, v) register uintptr_t r##n asm("a" #n) = (uintptr_t)(v)
static const uint16_t s_dim_mask_r = 0xF800u, s_dim_mask_g = 0x07E0u, s_dim_mask_b = 0x001Fu;

// Blocks of 8 pixels from an address aligned to 16 B; n8 = number of blocks.
static void lvgl_port_dim_pie(uint16_t* p, uint32_t n8, uint16_t k)
{
    {
        DIM_PIE_R(0, &s_dim_mask_r);
        DIM_PIE_R(1, &s_dim_mask_g);
        DIM_PIE_R(2, &s_dim_mask_b);
        DIM_PIE_R(3, &k);
        asm volatile("esp.vldbc.16.ip q5, a0, 0\n\t"
                     "esp.vldbc.16.ip q6, a1, 0\n\t"
                     "esp.vldbc.16.ip q7, a2, 0\n\t"
                     "esp.vldbc.16.ip q4, a3, 0\n\t"
                     "li a0, 5\n\t"
                     "esp.movx.w.sar a0\n\t"
                     : "+r"(r0)
                     : "r"(r1), "r"(r2), "r"(r3)
                     : "memory");
    }
    for (uint32_t i = 0; i < n8; i++) {
        DIM_PIE_R(0, p);
        asm volatile("esp.vld.128.ip q0, a0, 0\n\t"
                     "esp.andq q1, q0, q5\n\t"
                     "esp.andq q2, q0, q6\n\t"
                     "esp.andq q3, q0, q7\n\t"
                     "esp.vmul.u16 q1, q1, q4\n\t"
                     "esp.vmul.u16 q2, q2, q4\n\t"
                     "esp.vmul.u16 q3, q3, q4\n\t"
                     "esp.andq q1, q1, q5\n\t"
                     "esp.andq q2, q2, q6\n\t"
                     "esp.orq q1, q1, q2\n\t"
                     "esp.orq q1, q1, q3\n\t"
                     "esp.vst.128.ip q1, a0, 16\n\t"
                     : "+r"(r0)::"memory");
        p = (uint16_t*)r0;
    }
}
#endif

// Area pixels one after another: an LVGL buffer without padding between rows.
_Static_assert(LV_DRAW_BUF_STRIDE_ALIGN == 1, "dimming in the flush assumes rows without padding");

static void lvgl_port_apply_dim(uint8_t* px, uint32_t n)
{
    const uint8_t opa = s_dim_opa;
    if (opa <= LV_OPA_MIN) {
        return;
    }
    if (opa >= LV_OPA_MAX) {
        memset(px, 0, (size_t)n * sizeof(uint16_t));
        return;
    }
    const uint32_t k = 32u - (((uint32_t)opa + 4u) >> 3);
    uint16_t* p      = (uint16_t*)px;
    uint32_t i       = 0;
#if CONFIG_IDF_TARGET_ESP32P4
    for (; i < n && ((uintptr_t)(p + i) & 15u) != 0; i++) {  // up to a 16 B boundary (LVGL buffers usually already are)
        p[i] = lvgl_port_dim_px(p[i], k);
    }
    const uint32_t n8 = (n - i) / 8;
    lvgl_port_dim_pie(p + i, n8, (uint16_t)k);
    i += n8 * 8;
#endif
    for (; i < n; i++) {
        p[i] = lvgl_port_dim_px(p[i], k);
    }
}

// Cyberdeck: time spent in the flush (blocking PPA rotation, queuing the DMA2D copy, dimming) - LVGL task.
static void lvgl_port_flush_callback(lv_display_t* drv, const lv_area_t* area, uint8_t* color_map)
{
    const int64_t t0 = esp_timer_get_time();
    lvgl_port_apply_dim(color_map, (uint32_t)lv_area_get_size(area));
    lvgl_port_flush_callback_impl(drv, area, color_map);
    s_flush_cb_us += (uint64_t)(esp_timer_get_time() - t0);
}

static void lvgl_port_flush_callback_impl(lv_display_t* drv, const lv_area_t* area, uint8_t* color_map)
{
    assert(drv != NULL);
    assert(area != NULL);
    assert(color_map != NULL);
    lvgl_port_display_ctx_t* disp_ctx = (lvgl_port_display_ctx_t*)lv_display_get_driver_data(drv);
    assert(disp_ctx != NULL);

    if (disp_ctx->disp_type == LVGL_PORT_DISP_TYPE_DSI) {
        s_flush_submitted++;  // Cyberdeck: flush progress (lvgl_port_flush_counts)
    }

    // Cyberdeck: strips rotated in the background (rotation buffer + area copy, or straight into the frame buffer).
    if (disp_ctx->panel_fb != NULL && disp_ctx->flags.sw_rotate &&
        disp_ctx->current_rotation == LV_DISPLAY_ROTATION_90 && !disp_ctx->flags.swap_bytes &&
        !disp_ctx->flags.monochrome && !disp_ctx->flags.direct_mode && !disp_ctx->flags.full_refresh) {
        if (lvgl_port_flush_strip(disp_ctx, drv, area, color_map)) {
            return;
        }
    }

    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;

    // printf("%d %d %d %d\n", offsetx1, offsetx2, offsety1, offsety2);

    /* SW rotation enabled */
    if (disp_ctx->flags.sw_rotate && (disp_ctx->current_rotation > LV_DISPLAY_ROTATION_0)) {
        /* SW rotation */
        if (lvgl_port_rotation_buffer(disp_ctx)) {
            int32_t ww           = lv_area_get_width(area);
            int32_t hh           = lv_area_get_height(area);
            lv_color_format_t cf = lv_display_get_color_format(drv);
            uint32_t w_stride    = lv_draw_buf_width_to_stride(ww, cf);
            uint32_t h_stride    = lv_draw_buf_width_to_stride(hh, cf);
            if (disp_ctx->current_rotation == LV_DISPLAY_ROTATION_180) {
                lv_draw_sw_rotate(color_map, disp_ctx->draw_buffs[2], hh, ww, h_stride, h_stride,
                                  LV_DISPLAY_ROTATION_180, cf);
            } else if (disp_ctx->current_rotation == LV_DISPLAY_ROTATION_90) {
                // printf("%ld %ld\n", w_stride, h_stride);
                // lv_draw_sw_rotate(color_map, disp_ctx->draw_buffs[2], ww, hh, w_stride, h_stride,
                //                   LV_DISPLAY_ROTATION_90, cf);
                // rotate_copy_pixel((uint16_t*)color_map, (uint16_t*)disp_ctx->draw_buffs[2], offsetx1, offsety1,
                //                   offsetx2, offsety2, LV_HOR_RES, LV_VER_RES, 270);
                // Cyberdeck: the PPA wait has a time limit (esp_driver_ppa in components/); an error = the same
                // rotation on the CPU (lv_draw_sw_rotate 90: point (x, y) -> row ww-1-x, column y - like
                // PPA_SRM_ROTATION_ANGLE_90).
                esp_err_t perr = ESP_ERR_INVALID_STATE;
                if (lvgl_port_ppa_allowed()) {
                    perr = rotate_copy_pixel((uint16_t*)color_map, (uint16_t*)disp_ctx->draw_buffs[2], 0, 0,
                                             offsetx2 - offsetx1, offsety2 - offsety1, offsetx2 - offsetx1 + 1,
                                             offsety2 - offsety1 + 1, 270);
                    if (perr != ESP_OK) {
                        lvgl_port_ppa_failed(perr);
                    }
                }
                if (perr != ESP_OK) {
                    lv_draw_sw_rotate(color_map, disp_ctx->draw_buffs[2], ww, hh, w_stride, h_stride,
                                      LV_DISPLAY_ROTATION_90, cf);
                }
            } else if (disp_ctx->current_rotation == LV_DISPLAY_ROTATION_270) {
                lv_draw_sw_rotate(color_map, disp_ctx->draw_buffs[2], ww, hh, w_stride, h_stride,
                                  LV_DISPLAY_ROTATION_270, cf);
            }
            color_map = (uint8_t*)disp_ctx->draw_buffs[2];
            lvgl_port_rotate_area(drv, (lv_area_t*)area);
            offsetx1 = area->x1;
            offsetx2 = area->x2;
            offsety1 = area->y1;
            offsety2 = area->y2;
        }
    }

    if (disp_ctx->flags.swap_bytes) {
        size_t len = lv_area_get_size(area);
        lv_draw_sw_rgb565_swap(color_map, len);
    }

    /* Transfer data in buffer for monochromatic screen */
    if (disp_ctx->flags.monochrome) {
        _lvgl_port_transform_monochrome(drv, area, &color_map);
    }

    if ((disp_ctx->disp_type == LVGL_PORT_DISP_TYPE_RGB || disp_ctx->disp_type == LVGL_PORT_DISP_TYPE_DSI) &&
        (disp_ctx->flags.direct_mode || disp_ctx->flags.full_refresh)) {
        if (lv_disp_flush_is_last(drv)) {
            /* If the interface is I80 or SPI, this step cannot be used for drawing. */
            esp_lcd_panel_draw_bitmap(disp_ctx->panel_handle, 0, 0, lv_disp_get_hor_res(drv), lv_disp_get_ver_res(drv),
                                      color_map);
            /* Waiting for the last frame buffer to complete transmission */
            // xSemaphoreTake(disp_ctx->trans_sem, 0);
            // xSemaphoreTake(disp_ctx->trans_sem, portMAX_DELAY);
        }
    } else {
        if (disp_ctx->disp_type == LVGL_PORT_DISP_TYPE_DSI && s_flush_sem != NULL) {
            s_flush_busy = true;  // Cyberdeck: an interrupt signals the DMA2D copy end (lvgl_port_flush_wait)
        }
        esp_lcd_panel_draw_bitmap(disp_ctx->panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
    }

    if (disp_ctx->disp_type == LVGL_PORT_DISP_TYPE_RGB ||
        (disp_ctx->disp_type == LVGL_PORT_DISP_TYPE_DSI &&
         (disp_ctx->flags.direct_mode || disp_ctx->flags.full_refresh))) {
        lv_disp_flush_ready(drv);
    }
}

// static void lvgl_port_flush_callback(lv_display_t *drv, const lv_area_t *area, uint8_t *color_map)
// {
//     assert(drv != NULL);
//     assert(area != NULL);
//     assert(color_map != NULL);
//     lvgl_port_display_ctx_t *disp_ctx = (lvgl_port_display_ctx_t *)lv_display_get_driver_data(drv);
//     assert(disp_ctx != NULL);

//     int offsetx1 = area->x1;
//     int offsetx2 = area->x2;
//     int offsety1 = area->y1;
//     int offsety2 = area->y2;

//     /* SW rotation enabled */
//     if (disp_ctx->flags.sw_rotate && (disp_ctx->current_rotation > LV_DISPLAY_ROTATION_0)) {
//         /* SW rotation */
//         if (disp_ctx->draw_buffs[2]) {
//             int32_t ww = lv_area_get_width(area);
//             int32_t hh = lv_area_get_height(area);
//             lv_color_format_t cf = lv_display_get_color_format(drv);
//             uint32_t w_stride = lv_draw_buf_width_to_stride(ww, cf);
//             uint32_t h_stride = lv_draw_buf_width_to_stride(hh, cf);
//             if (disp_ctx->current_rotation == LV_DISPLAY_ROTATION_180) {
//                 lv_draw_sw_rotate(color_map, disp_ctx->draw_buffs[2], hh, ww, h_stride, h_stride,
//                 LV_DISPLAY_ROTATION_180, cf);
//             } else if (disp_ctx->current_rotation == LV_DISPLAY_ROTATION_90) {
//                 lv_draw_sw_rotate(color_map, disp_ctx->draw_buffs[2], ww, hh, w_stride, h_stride,
//                 LV_DISPLAY_ROTATION_90, cf);
//             } else if (disp_ctx->current_rotation == LV_DISPLAY_ROTATION_270) {
//                 lv_draw_sw_rotate(color_map, disp_ctx->draw_buffs[2], ww, hh, w_stride, h_stride,
//                 LV_DISPLAY_ROTATION_270, cf);
//             }
//             color_map = (uint8_t *)disp_ctx->draw_buffs[2];
//             lvgl_port_rotate_area(drv, (lv_area_t *)area);
//             offsetx1 = area->x1;
//             offsetx2 = area->x2;
//             offsety1 = area->y1;
//             offsety2 = area->y2;
//         }
//     }

//     if (disp_ctx->flags.swap_bytes) {
//         size_t len = lv_area_get_size(area);
//         lv_draw_sw_rgb565_swap(color_map, len);
//     }

//     /* Transfer data in buffer for monochromatic screen */
//     if (disp_ctx->flags.monochrome) {
//         _lvgl_port_transform_monochrome(drv, area, &color_map);
//     }

//     if ((disp_ctx->disp_type == LVGL_PORT_DISP_TYPE_RGB || disp_ctx->disp_type == LVGL_PORT_DISP_TYPE_DSI) &&
//     (disp_ctx->flags.direct_mode || disp_ctx->flags.full_refresh)) {
//         if (lv_disp_flush_is_last(drv)) {
//             /* If the interface is I80 or SPI, this step cannot be used for drawing. */
//             esp_lcd_panel_draw_bitmap(disp_ctx->panel_handle, 0, 0, lv_disp_get_hor_res(drv),
//             lv_disp_get_ver_res(drv), color_map);
//             /* Waiting for the last frame buffer to complete transmission */
//             xSemaphoreTake(disp_ctx->trans_sem, 0);
//             xSemaphoreTake(disp_ctx->trans_sem, portMAX_DELAY);
//         }
//     } else {
//         esp_lcd_panel_draw_bitmap(disp_ctx->panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
//     }

//     if (disp_ctx->disp_type == LVGL_PORT_DISP_TYPE_RGB || (disp_ctx->disp_type == LVGL_PORT_DISP_TYPE_DSI &&
//     (disp_ctx->flags.direct_mode || disp_ctx->flags.full_refresh))) {
//         lv_disp_flush_ready(drv);
//     }
// }

static void lvgl_port_disp_rotation_update(lvgl_port_display_ctx_t* disp_ctx)
{
    assert(disp_ctx != NULL);

    disp_ctx->current_rotation = lv_display_get_rotation(disp_ctx->disp_drv);
    if (disp_ctx->flags.sw_rotate) {
        return;
    }

    esp_lcd_panel_handle_t control_handle =
        (disp_ctx->control_handle ? disp_ctx->control_handle : disp_ctx->panel_handle);
    /* Solve rotation screen and touch */
    switch (lv_display_get_rotation(disp_ctx->disp_drv)) {
        case LV_DISPLAY_ROTATION_0:
            /* Rotate LCD display */
            esp_lcd_panel_swap_xy(control_handle, disp_ctx->rotation.swap_xy);
            esp_lcd_panel_mirror(control_handle, disp_ctx->rotation.mirror_x, disp_ctx->rotation.mirror_y);
            break;
        case LV_DISPLAY_ROTATION_90:
            /* Rotate LCD display */
            esp_lcd_panel_swap_xy(control_handle, !disp_ctx->rotation.swap_xy);
            if (disp_ctx->rotation.swap_xy) {
                esp_lcd_panel_mirror(control_handle, !disp_ctx->rotation.mirror_x, disp_ctx->rotation.mirror_y);
            } else {
                esp_lcd_panel_mirror(control_handle, disp_ctx->rotation.mirror_x, !disp_ctx->rotation.mirror_y);
            }
            break;
        case LV_DISPLAY_ROTATION_180:
            /* Rotate LCD display */
            esp_lcd_panel_swap_xy(control_handle, disp_ctx->rotation.swap_xy);
            esp_lcd_panel_mirror(control_handle, !disp_ctx->rotation.mirror_x, !disp_ctx->rotation.mirror_y);
            break;
        case LV_DISPLAY_ROTATION_270:
            /* Rotate LCD display */
            esp_lcd_panel_swap_xy(control_handle, !disp_ctx->rotation.swap_xy);
            if (disp_ctx->rotation.swap_xy) {
                esp_lcd_panel_mirror(control_handle, disp_ctx->rotation.mirror_x, !disp_ctx->rotation.mirror_y);
            } else {
                esp_lcd_panel_mirror(control_handle, !disp_ctx->rotation.mirror_x, disp_ctx->rotation.mirror_y);
            }
            break;
    }

    /* Wake LVGL task, if needed */
    lvgl_port_task_wake(LVGL_PORT_EVENT_DISPLAY, disp_ctx->disp_drv);
}

static void lvgl_port_disp_size_update_callback(lv_event_t* e)
{
    assert(e);
    lvgl_port_display_ctx_t* disp_ctx = (lvgl_port_display_ctx_t*)lv_event_get_user_data(e);
    lvgl_port_disp_rotation_update(disp_ctx);
}

static void lvgl_port_display_invalidate_callback(lv_event_t* e)
{
    /* Wake LVGL task, if needed */
    lvgl_port_task_wake(LVGL_PORT_EVENT_DISPLAY, NULL);
}
