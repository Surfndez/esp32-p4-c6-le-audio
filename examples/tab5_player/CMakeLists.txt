# tab5_player - LE Audio MP3 player example for the M5Stack Tab5 (ESP32-P4 + ESP32-C6 running c6_firmware).
cmake_minimum_required(VERSION 3.16)

# The LE Audio host component and the patched esp_hosted host (raw HCI channel incl. ISO) from this repository.
set(EXTRA_COMPONENT_DIRS
    "${CMAKE_CURRENT_LIST_DIR}/../../p4_le_audio"
    "${CMAKE_CURRENT_LIST_DIR}/../../components/espressif__esp_hosted"
    # Patched fatfs: exFAT and SD transfers of at most 16 kB (components/fatfs/PATCHES.md)
    "${CMAKE_CURRENT_LIST_DIR}/../../components/fatfs"
    # Display / touch: M5Stack Tab5 BSP, patched esp_lcd and esp_lvgl_port, ST7121 panel (from the original firmware)
    "${CMAKE_CURRENT_LIST_DIR}/../../components/m5stack_tab5"
    "${CMAKE_CURRENT_LIST_DIR}/../../components/esp_lcd"
    "${CMAKE_CURRENT_LIST_DIR}/../../components/esp_driver_ppa"
    "${CMAKE_CURRENT_LIST_DIR}/../../components/esp_lcd_st7121"
    "${CMAKE_CURRENT_LIST_DIR}/../../components/espressif__esp_lvgl_port"
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(tab5_player)
