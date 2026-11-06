# Display components (used by examples/tab5_player)

Copied unchanged from the firmware this project comes from, where they run on the M5Stack Tab5 every day. Search for
"Cyberdeck" in the code for every local change (comments there are in Polish).

| Component | Origin | License | Local changes |
|---|---|---|---|
| `m5stack_tab5/` | M5Stack Tab5 BSP | Apache-2.0 | panel/touch detection kept (ILI9881C + GT911, ST7123, ST7121); speaker amplifier off at start; SD bounce buffer; panel handle / DCS access, standby; I2S underrun counter |
| `esp_lcd/` | ESP-IDF v5.5.5 `components/esp_lcd` | Apache-2.0 | MIPI-DSI DPI panel: stop / resume of the frame stream, underrun counting, spinlock init, interrupt status handling |
| `esp_driver_ppa/` | ESP-IDF v5.5.5 `components/esp_driver_ppa` | Apache-2.0 | recovery of a stuck SRM (rotate) transaction instead of blocking LVGL forever (esp-idf issue #19096), bounded blocking mode |
| `espressif__esp_lvgl_port/` | esp-bsp `esp_lvgl_port` 2.5.0 | Apache-2.0 | strip rendering with PPA rotation into the frame buffer, rotation buffer handling, SRM recovery hook |
| `esp_lcd_st7121/` | M5Stack ST7121 panel driver | Apache-2.0 | none |

The example uses the BSP's `bsp_display_start_with_config()` with the original firmware's settings: RGB565,
48-line double strip buffers in PSRAM, software (PPA) rotation to landscape, refreshed areas rounded to even width and
heights of 4 rows.
