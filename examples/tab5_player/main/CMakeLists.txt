get_filename_component(DR_LIBS_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../third_party/dr_libs" ABSOLUTE)
if(NOT EXISTS "${DR_LIBS_DIR}/dr_mp3.h")
    message(FATAL_ERROR "dr_libs not found in ${DR_LIBS_DIR} - run tools/fetch_deps.sh")
endif()

# The ESP32-C6 firmware of this project travels inside the player (c6_update.cpp installs it over esp_hosted OTA),
# so a Tab5 needs no USB-UART adapter for the C6: build it first with sh c6_firmware/build.sh.
get_filename_component(C6_IMAGE "${CMAKE_CURRENT_LIST_DIR}/../../../c6_firmware/build/network_adapter.bin" ABSOLUTE)
if(NOT EXISTS "${C6_IMAGE}")
    message(FATAL_ERROR "C6 firmware image not found: ${C6_IMAGE} - build it first: sh c6_firmware/build.sh")
endif()

idf_component_register(
    SRCS "main.cpp" "ui.cpp" "log_sink.cpp" "player.cpp" "resample.cpp" "bond_store.cpp" "c6_update.cpp" "board_tab5.c"
         "dr_impl.c"
    INCLUDE_DIRS "." "${DR_LIBS_DIR}"
    PRIV_REQUIRES p4_le_audio espressif__esp_hosted m5stack_tab5 lvgl__lvgl espressif__esp_lvgl_port esp_driver_i2c esp_driver_sdmmc sdmmc fatfs vfs nvs_flash
                  esp_driver_usb_serial_jtag app_update esp_partition esp_timer esp_mm esp_hw_support
)
target_add_binary_data(${COMPONENT_TARGET} "${C6_IMAGE}" BINARY)
