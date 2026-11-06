/*
 * SPDX-FileCopyrightText: 2023-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "esp_lcd_types.h"

typedef struct esp_lcd_dsi_bus_t *esp_lcd_dsi_bus_handle_t;   /*!< Type of MIPI DSI bus handle */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief MIPI DSI bus configuration structure
 */
typedef struct {
    int bus_id;                              /*!< Select which DSI controller, index from 0 */
    uint8_t num_data_lanes;                  /*!< Number of data lanes, if set to 0, the driver will fallback to use maximum number of lanes */
    mipi_dsi_phy_pllref_clock_source_t phy_clk_src; /*!< The clock source for the PHY PLL */
    float lane_bit_rate_mbps;                /*!< Lane bit rate in Mbps */
    /// Extra configuration flags for MIPI DSI bus
    struct extra_dsi_bus_flags {
        uint32_t clock_lane_force_hs: 1;     /*!< Force the clock lane to high speed mode */
    } flags;                                 /*!< Extra configuration flags */
} esp_lcd_dsi_bus_config_t;

/**
 * @brief Create MIPI DSI bus handle
 *
 * @param[in] bus_config Bus configuration
 * @param[out] ret_bus Returned bus handle
 * @return
 *      - ESP_OK: Create MIPI DSI bus successfully
 *      - ESP_ERR_INVALID_ARG: Create MIPI DSI bus failed because of invalid argument
 *      - ESP_ERR_NO_MEM: Create MIPI DSI bus failed because of out of memory
 *      - ESP_ERR_NOT_FOUND: Create MIPI DSI bus failed because no more free DSI hardware instance
 *      - ESP_FAIL: Create MIPI DSI bus failed because of other error
 */
esp_err_t esp_lcd_new_dsi_bus(const esp_lcd_dsi_bus_config_t *bus_config, esp_lcd_dsi_bus_handle_t *ret_bus);

/**
 * @brief Destroy MIPI DSI bus handle
 *
 * @param[in] bus MIPI DSI bus handle, returned from `esp_lcd_new_dsi_bus`
 * @return
 *      - ESP_OK: Destroy MIPI DSI bus successfully
 *      - ESP_ERR_INVALID_ARG: Destroy MIPI DSI bus failed because of invalid argument
 *      - ESP_FAIL: Destroy MIPI DSI bus failed because of other error
 */
esp_err_t esp_lcd_del_dsi_bus(esp_lcd_dsi_bus_handle_t bus);

/**
 * @brief Panel IO configuration structure, for MIPI DSI command interface
 */
typedef struct {
    uint8_t virtual_channel;  /*!< Virtual channel ID, index from 0 */
    int lcd_cmd_bits;         /*!< Bit-width of LCD command */
    int lcd_param_bits;       /*!< Bit-width of LCD parameter */
} esp_lcd_dbi_io_config_t;

/**
 * @brief Create LCD panel IO, for MIPI DSI DBI interface
 *
 * @note Although we call it "DBI", internally the driver is using a co-called "generic" interface for transmitting/receiving LCD commands and parameters.
 *
 * @param[in] bus MIPI DSI bus handle, returned from `esp_lcd_new_dsi_bus`
 * @param[in] io_config IO configuration
 * @param[out] ret_io Returned panel IO handle
 * @return
 *      - ESP_OK: Create MIPI DSI command IO successfully
 *      - ESP_ERR_INVALID_ARG: Create MIPI DSI command IO failed because of invalid argument
 *      - ESP_ERR_NO_MEM: Create MIPI DSI command IO failed because of out of memory
 *      - ESP_FAIL: Create MIPI DSI command IO failed because of other error
 */
esp_err_t esp_lcd_new_panel_io_dbi(esp_lcd_dsi_bus_handle_t bus, const esp_lcd_dbi_io_config_t *io_config, esp_lcd_panel_io_handle_t *ret_io);

/**
 * @brief MIPI DSI DPI panel configuration structure
 */
typedef struct {
    uint8_t virtual_channel;                   /*!< Virtual channel ID, index from 0 */
    mipi_dsi_dpi_clock_source_t dpi_clk_src;   /*!< MIPI DSI DPI clock source */
    float dpi_clock_freq_mhz;                  /*!< Pixel clock frequency in MHz */
    lcd_color_rgb_pixel_format_t pixel_format; /*!< Pixel format that used by the MIPI LCD device */
    lcd_color_format_t in_color_format;        /*!< Format of the input data (color space and pixel format),
                                                    which is the format stored in the frame buffer */
    lcd_color_format_t out_color_format;       /*!< Format of the output data (color space and pixel format),
                                                    which is the format that the panel can accept */
    uint8_t num_fbs;                           /*!< Number of screen-sized frame buffers that allocated by the driver
                                                    By default (set to either 0 or 1) only one frame buffer will be created */
    esp_lcd_video_timing_t video_timing;       /*!< Video timing */
    /// Extra configuration flags for MIPI DSI DPI panel
    struct extra_dpi_panel_flags {
        uint32_t use_dma2d: 1; /*!< Use DMA2D to copy user buffer to the frame buffer when necessary */
        uint32_t disable_lp: 1;/*!< Disable low-power for DPI */
    } flags;                   /*!< Extra configuration flags */
} esp_lcd_dpi_panel_config_t;

/**
 * @brief Create LCD panel for MIPI DSI DPI interface
 *
 * @param[in] bus MIPI DSI bus handle, returned from `esp_lcd_new_dsi_bus`
 * @param[in] panel_config DSI data panel configuration
 * @param[out] ret_panel Returned LCD panel handle
 * @return
 *      - ESP_OK: Create MIPI DSI data panel successfully
 *      - ESP_ERR_INVALID_ARG: Create MIPI DSI data panel failed because of invalid argument
 *      - ESP_ERR_NO_MEM: Create MIPI DSI data panel failed because of out of memory
 *      - ESP_ERR_NOT_SUPPORTED: Create MIPI DSI data panel failed because of unsupported feature
 *      - ESP_FAIL: Create MIPI DSI data panel failed because of other error
 */
esp_err_t esp_lcd_new_panel_dpi(esp_lcd_dsi_bus_handle_t bus, const esp_lcd_dpi_panel_config_t *panel_config, esp_lcd_panel_handle_t *ret_panel);

/**
 * @brief Get the address of the frame buffer(s) that allocated by the driver
 *
 * @param[in] dpi_panel MIPI DPI panel handle, returned from esp_lcd_new_panel_dpi()
 * @param[in] fb_num Number of frame buffer(s) to get. This value must be the same as the number of the followed parameters.
 * @param[out] fb0 Address of the frame buffer 0 (first frame buffer)
 * @param[out] ... List of other frame buffers if any
 * @return
 *      - ESP_ERR_INVALID_ARG: Get frame buffer address failed because of invalid argument
 *      - ESP_OK: Get frame buffer address successfully
 */
esp_err_t esp_lcd_dpi_panel_get_frame_buffer(esp_lcd_panel_handle_t dpi_panel, uint32_t fb_num, void **fb0, ...);

/**
 * @brief Set pre-defined pattern to the screen for testing or debugging purpose
 *
 * @param[in] dpi_panel MIPI DPI panel handle, returned from esp_lcd_new_panel_dpi()
 * @param[in] pattern Pattern type
 * @return
 *      - ESP_OK: Set pattern successfully
 *      - ESP_ERR_INVALID_ARG: Set pattern failed because of invalid argument
 *      - ESP_FAIL: Set pattern failed because of other error
 */
esp_err_t esp_lcd_dpi_panel_set_pattern(esp_lcd_panel_handle_t dpi_panel, mipi_dsi_pattern_type_t pattern);

/**
 * @brief Set color conversion configuration for DPI panel
 *
 * @param[in] dpi_panel MIPI DPI panel handle, returned from esp_lcd_new_panel_dpi()
 * @param[in] config Color conversion configuration
 * @return
 *      - ESP_OK: Set color conversion configuration successfully
 *      - ESP_ERR_INVALID_ARG: Set color conversion configuration failed because of invalid argument
 *      - ESP_FAIL: Set color conversion configuration failed because of other error
 */
esp_err_t esp_lcd_dpi_panel_set_color_conversion(esp_lcd_panel_handle_t dpi_panel, const esp_lcd_color_conv_config_t *config);

/**
 * @brief Type of LCD DPI panel event data
 */
typedef struct {
} esp_lcd_dpi_panel_event_data_t;

/**
 * @brief A general function callback prototype for DPI panel driver
 *
 * @param[in] panel LCD panel handle, which is created by factory API like esp_lcd_new_panel_dpi()
 * @param[in] edata DPI panel event data, fed by driver
 * @param[in] user_ctx User data
 * @return Whether a high priority task has been waken up by this function
 */
typedef bool (*esp_lcd_dpi_panel_general_cb_t)(esp_lcd_panel_handle_t panel, esp_lcd_dpi_panel_event_data_t *edata, void *user_ctx);

/**
 * @brief Declare the prototype of the function that will be invoked
 *        when driver finishes coping user's color buffer to frame buffer
 */
typedef esp_lcd_dpi_panel_general_cb_t esp_lcd_dpi_panel_color_trans_done_cb_t;

/**
 * @brief Declare the prototype of the function that will be invoked
 *        when the frame buffer can be reused safely
 *
 * @deprecated Use esp_lcd_dpi_panel_frame_buf_complete_cb_t instead.
 */
typedef esp_lcd_dpi_panel_general_cb_t esp_lcd_dpi_panel_refresh_done_cb_t;

/**
 * @brief Declare the prototype of the function that will be invoked when the LCD controller sends the VSYNC signal.
 */
typedef esp_lcd_dpi_panel_general_cb_t esp_lcd_dpi_panel_vsync_cb_t;

/**
 * @brief Declare the prototype of the function that will be invoked
 *        when the frame buffer can be reused safely
 */
typedef esp_lcd_dpi_panel_general_cb_t esp_lcd_dpi_panel_frame_buf_complete_cb_t;

/**
 * @brief Type of LCD DPI panel callbacks
 */
typedef struct {
    esp_lcd_dpi_panel_color_trans_done_cb_t on_color_trans_done; /*!< Invoked when user's draw buffer copied to the frame buffer.
                                                                      This is an indicator that the draw buffer can be recycled safely.
                                                                      But doesn't mean the draw buffer finishes the refreshing to the screen. */
    union {
        esp_lcd_dpi_panel_refresh_done_cb_t on_refresh_done __attribute__((deprecated("Deprecated, use on_frame_buf_complete instead"))); /*!< Deprecated, use on_frame_buf_complete instead */
        esp_lcd_dpi_panel_frame_buf_complete_cb_t on_frame_buf_complete; /*!< Invoked when the frame buffer can be reused safely
                                                                              when the frame buffer is the draw buffer. */
    };
    esp_lcd_dpi_panel_vsync_cb_t on_vsync;                       /*!< VSYNC event callback */
} esp_lcd_dpi_panel_event_callbacks_t;

/**
 * @brief Register LCD DPI panel callbacks
 *
 * @param[in] dpi_panel LCD DPI panel handle, which is returned from esp_lcd_new_panel_dpi()
 * @param[in] cbs structure with all LCD panel callbacks
 * @param[in] user_ctx User private data, passed directly to callback's user_ctx
 * @return
 *      - ESP_ERR_INVALID_ARG: Register callbacks failed because of invalid argument
 *      - ESP_OK: Register callbacks successfully
 */
esp_err_t esp_lcd_dpi_panel_register_event_callbacks(esp_lcd_panel_handle_t dpi_panel, const esp_lcd_dpi_panel_event_callbacks_t *cbs, void *user_ctx);

/**
 * @brief Cyberdeck: stop the DPI stream cleanly - the DMA finishes the current frame, the bridge DPI
 *        output stops at the frame boundary, the DSI host core is reset into command mode.
 *        Needed before explicit light sleep (PSRAM goes offline) and to resync after an underrun.
 * @return ESP_OK, or ESP_ERR_TIMEOUT when the DMA did not finish a frame and was stopped by force
 */
esp_err_t esp_lcd_dpi_panel_stream_stop(esp_lcd_panel_handle_t dpi_panel);

/**
 * @brief Cyberdeck: start the DPI stream again from the first pixel of the current frame buffer
 */
esp_err_t esp_lcd_dpi_panel_stream_start(esp_lcd_panel_handle_t dpi_panel);

/**
 * @brief Cyberdeck: release (hold = false) or take back (hold = true) the power management locks of the picture
 *        path - the DPI panel's ESP_PM_CPU_FREQ_MAX and the DSI bus' ESP_PM_NO_LIGHT_SLEEP. Only while the stream
 *        is stopped (esp_lcd_dpi_panel_stream_stop): then the CPU may scale down and the chip may enter automatic
 *        light sleep. Take them back BEFORE raising the clock and starting the stream. Idempotent.
 * @return ESP_ERR_INVALID_STATE when releasing while the stream runs
 */
esp_err_t esp_lcd_dpi_panel_pm_hold(esp_lcd_panel_handle_t dpi_panel, bool hold);

/**
 * @brief Cyberdeck: DPI stream diagnostics (reading clears the host interrupt status registers)
 */
typedef struct {
    uint32_t underruns;      /*!< bridge underruns outside of a deliberate stop */
    bool stopped;            /*!< stream stopped by esp_lcd_dpi_panel_stream_stop */
    bool video_mode;         /*!< DSI host in video mode */
    bool host_powered;       /*!< DSI host core out of reset */
    bool dpi_output;         /*!< bridge DPI output enabled */
    uint32_t phy_status;     /*!< DSI host PHY_STATUS (bit 0 PLL locked, 2/4/7 lanes in stop state) */
    uint32_t host_int_st0;   /*!< DSI host INT_ST0 bits not yet taken by the guard (acknowledge / PHY errors) */
    uint32_t host_int_st1;   /*!< DSI host INT_ST1 bits not yet taken by the guard (timeouts, ECC/CRC, DPI payload) */
    uint32_t vid_pkt_status; /*!< DSI host VID_PKT_STATUS */
    uint32_t brg_int_raw;    /*!< bridge raw interrupt status */
    uint32_t brg_fifo_depth; /*!< bridge FIFO depth */
} esp_lcd_dpi_stream_state_t;

esp_err_t esp_lcd_dpi_panel_stream_state(esp_lcd_panel_handle_t dpi_panel, esp_lcd_dpi_stream_state_t *state);

/**
 * @brief Cyberdeck: number of bridge underruns outside of a deliberate stop (ISR counter)
 */
uint32_t esp_lcd_dpi_panel_underruns(esp_lcd_panel_handle_t dpi_panel);

/**
 * @brief Cyberdeck: false when the stream should run but the DSI host is out of video mode, in reset,
 *        or the bridge DPI output is off (black screen while frames still flow)
 */
bool esp_lcd_dpi_panel_stream_healthy(esp_lcd_panel_handle_t dpi_panel);

/**
 * @brief Cyberdeck test hook: pause the frame DMA mid-frame (max 50 ms) to provoke a bridge underrun
 */
esp_err_t esp_lcd_dpi_panel_test_underrun(esp_lcd_panel_handle_t dpi_panel, uint32_t pause_us);

/**
 * @brief Cyberdeck test hook: like esp_lcd_dpi_panel_test_underrun, but the pause starts delay_us after
 *        the end of a DMA frame (both max 20 ms) - places the underrun at a chosen scan line, e.g. to learn
 *        whether underruns in some lines are not reported (bridge fifo_underrun_discard_vcnt).
 */
esp_err_t esp_lcd_dpi_panel_test_underrun_at(esp_lcd_panel_handle_t dpi_panel, uint32_t delay_us, uint32_t pause_us);

/**
 * @brief Cyberdeck: DSI host error status (INT_ST0 / INT_ST1 are clear-on-read on the DW MIPI DSI host).
 *        Every read goes into one accumulator; take = true hands the bits over and clears the accumulator
 *        (one consumer: the display guard), take = false only peeks.
 */
esp_err_t esp_lcd_dpi_panel_host_errors(esp_lcd_panel_handle_t dpi_panel, bool take, uint32_t *st0, uint32_t *st1);

/**
 * @brief Cyberdeck: result of esp_lcd_dpi_panel_host_errors_selftest
 */
typedef struct {
    bool latched_first; /*!< the forced event showed up in INT_ST1 with the masks as they were */
    bool unmask_needed; /*!< ... only after all INT_MSK0/INT_MSK1 sources were enabled (this run) */
    bool latched;       /*!< final: the error counters can see events */
    bool clear_on_read; /*!< a second read no longer had the forced bit (INT_ST clears on read) */
    bool sticky;        /*!< INT_ST does not clear on read - only newly set bits are counted from now on */
    bool unmasked;      /*!< the masks were set by a self-test (kept, re-applied at stream start) */
    uint32_t msk0;      /*!< INT_MSK0 after the test */
    uint32_t msk1;      /*!< INT_MSK1 after the test */
} esp_lcd_dpi_err_selftest_t;

/**
 * @brief Cyberdeck: does a DSI host error reach INT_ST0/INT_ST1 at all? Forces one event (INT_FORCE1 to_lp_rx,
 *        which cannot occur in video mode), reads it back and removes it from the accumulator. IDF never writes
 *        INT_MSK0/INT_MSK1 (reset value 0); if the event does not latch and allow_unmask is set, all sources are
 *        enabled (the DSI host interrupt is not routed to any CPU interrupt) and the test repeats.
 *        Also tells whether reading INT_ST clears it (assumed so far, as in Linux dw_mipi_dsi_clear_err).
 */
esp_err_t esp_lcd_dpi_panel_host_errors_selftest(esp_lcd_panel_handle_t dpi_panel, bool allow_unmask,
                                                 esp_lcd_dpi_err_selftest_t *out);

/**
 * @brief Cyberdeck diagnostic aid: bridge fifo_underrun_discard_vcnt ("underrun interrupt mask when the line count
 *        is less than this field"; IDF sets h_size). Applied at the next frame boundary, not persistent.
 */
esp_err_t esp_lcd_dpi_panel_set_underrun_discard(esp_lcd_panel_handle_t dpi_panel, uint32_t lines);

/**
 * @brief Cyberdeck: counters and state for display diagnostics (cheap, no register side effects)
 */
typedef struct {
    uint32_t underruns;             /*!< bridge underruns while streaming */
    uint32_t underruns_stop;        /*!< bridge underruns during a deliberate stop (expected, still counted) */
    uint32_t frames;                /*!< DMA frames sent to the bridge */
    int64_t frame_us;               /*!< esp_timer time of the last DMA frame end */
    int64_t underrun_us;            /*!< esp_timer time of the last underrun (0 = none) */
    uint32_t underrun_phase_us;     /*!< last underrun: time since the preceding DMA frame end */
    bool stopped;                   /*!< stream stopped by esp_lcd_dpi_panel_stream_stop */
    uint32_t brg_fifo_depth;        /*!< bridge FIFO depth now (64-bit words) */
    uint32_t fill;                  /*!< pixel sent to the DSI host when the bridge FIFO is empty (dpi_rsv_dpi_data) */
    uint32_t underrun_discard_vcnt; /*!< bridge fifo_underrun_discard_vcnt (underrun interrupt mask by line) */
    uint32_t int_msk0;              /*!< DSI host INT_MSK0 now */
    uint32_t int_msk1;              /*!< DSI host INT_MSK1 now */
} esp_lcd_dpi_diag_t;

esp_err_t esp_lcd_dpi_panel_diag(esp_lcd_panel_handle_t dpi_panel, esp_lcd_dpi_diag_t *out);

/**
 * @brief Cyberdeck diagnostic aid: set the pixel the bridge sends when its FIFO is empty (30 bits, IDF default
 *        0x3FFF = blue, the same colour the panel shows without video). Not persistent - resets at boot.
 */
esp_err_t esp_lcd_dpi_panel_set_fill(esp_lcd_panel_handle_t dpi_panel, uint32_t value);

#ifdef __cplusplus
}
#endif
