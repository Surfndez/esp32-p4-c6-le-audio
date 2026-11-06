/*
 * Cyberdeck: geometry of a 90 degree rotation straight into the panel frame buffer (pure arithmetic, no ESP-IDF
 * or LVGL - covered by a host unit test).
 *
 * The LVGL UI is landscape (LV_DISPLAY_ROTATION_90), the panel is portrait (720 columns x 1280 rows).
 * lvgl_port_rotate_area() for ROTATION_90 and a PPA_SRM_ROTATION_ANGLE_90 rotation (90 degrees counter-clockwise)
 * together give: UI point (X, Y) -> panel column Y, row panel_rows - 1 - X. The area x1..x2, y1..y2
 * therefore becomes a block starting at column y1 and row panel_rows - 1 - x2, (y2 - y1 + 1) wide
 * and (x2 - x1 + 1) high. Area rounding in the application's display code: x1 even, x2 odd (the PPA
 * skews areas with odd dimensions), y1 divisible by 4, and so is y2 + 1 (the block column in the panel is
 * divisible by 4 - esp-idf #19096) - an even start and even block dimensions (panel_rows is even).
 */
#pragma once

#include <stdint.h>

typedef struct {
    int32_t col; /*!< first column of the block in the frame buffer */
    int32_t row; /*!< first row */
    int32_t w;   /*!< block width (columns) */
    int32_t h;   /*!< block height (rows) */
} lvgl_port_fb_block_t;

static inline lvgl_port_fb_block_t lvgl_port_rot90_block(int32_t x1, int32_t y1, int32_t x2, int32_t y2,
                                                         int32_t panel_rows)
{
    lvgl_port_fb_block_t b;
    b.col = y1;
    b.row = panel_rows - 1 - x2;
    b.w   = y2 - y1 + 1;
    b.h   = x2 - x1 + 1;
    return b;
}

// The block fits in the panel_cols x panel_rows frame buffer.
static inline int lvgl_port_fb_block_fits(lvgl_port_fb_block_t b, int32_t panel_cols, int32_t panel_rows)
{
    return b.col >= 0 && b.row >= 0 && b.w > 0 && b.h > 0 && b.col + b.w <= panel_cols && b.row + b.h <= panel_rows;
}

// LVGL strips: LVGL splits the area ax1..ax2, ay1..ay2 from the top into strips with the same x. After rotation the
// area lies in the rotation buffer without gaps (columns = area height, rows = width, as the whole area used to);
// the strip sy1..sy2 is the block in it starting at column sy1 - ay1, row 0. Returns 0 if the strip is not part of
// the area.
static inline int lvgl_port_strip_block(int32_t ax1, int32_t ay1, int32_t ax2, int32_t ay2, int32_t sx1, int32_t sy1,
                                        int32_t sx2, int32_t sy2, lvgl_port_fb_block_t* out)
{
    if (sx1 != ax1 || sx2 != ax2 || sy1 < ay1 || sy2 > ay2 || sy1 > sy2) {
        return 0;
    }
    out->col = sy1 - ay1;
    out->row = 0;
    out->w   = sy2 - sy1 + 1;
    out->h   = ax2 - ax1 + 1;
    return 1;
}
