# p4_le_audio - LE Audio host for ESP32-P4 (BAP unicast client, CSIP, LC3, ISO over HCI / esp_hosted SDIO).
#
# Needs: the patched esp_hosted host component of this repository (components/espressif__esp_hosted: raw HCI
# channel with ISO packets) and liblc3 (Google, Apache-2.0) - fetched by tools/fetch_deps.sh into third_party/.
# Another location: set LIBLC3_DIR (CMake cache or environment).

if(NOT LIBLC3_DIR)
    if(DEFINED ENV{LIBLC3_DIR})
        set(LIBLC3_DIR "$ENV{LIBLC3_DIR}")
    else()
        get_filename_component(LIBLC3_DIR "${CMAKE_CURRENT_LIST_DIR}/../third_party/liblc3" ABSOLUTE)
    endif()
endif()
if(NOT EXISTS "${LIBLC3_DIR}/include/lc3.h")
    message(FATAL_ERROR "liblc3 not found in ${LIBLC3_DIR} - run tools/fetch_deps.sh (or set LIBLC3_DIR)")
endif()
file(GLOB LC3_SRCS "${LIBLC3_DIR}/src/*.c")

idf_component_register(
    SRCS "src/le_host.cpp" "src/le_stream.cpp" "src/le_audio_api.cpp" "src/le_port.cpp" "src/le_c6_ota.cpp"
         "models/bt/bt_audio_model.cpp" "models/bt/bt_bond_model.cpp" "models/bt/bt_reconnect_model.cpp"
         "models/bt/csip_model.cpp" "models/bt/le_audio_model.cpp" "models/bt/c6_image_model.cpp"
         ${LC3_SRCS}
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "src" "models" "${LIBLC3_DIR}/include"
    REQUIRES esp_common
    PRIV_REQUIRES espressif__esp_hosted esp_wifi esp_driver_sdmmc esp_timer esp_pm mbedtls pthread freertos log heap esp_hw_support
)

# liblc3: third-party code, no warnings; without LC3plus / High-Resolution mode (BAP uses plain LC3) - smaller
# tables in flash. Same flags as in the firmware this component was taken from.
set_source_files_properties(${LC3_SRCS} PROPERTIES COMPILE_OPTIONS "-w;-O2;-DLC3_PLUS=0;-DLC3_PLUS_HR=0")
