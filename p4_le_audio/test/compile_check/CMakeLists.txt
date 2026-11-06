# Compile check of p4_le_audio for the ESP32-P4 (CI and local): the smallest firmware that links the component
# with the patched esp_hosted host. Not a demo - it has no UI and plays nothing.
cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS
    "${CMAKE_CURRENT_LIST_DIR}/../.."
    "${CMAKE_CURRENT_LIST_DIR}/../../../components/espressif__esp_hosted"
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(le_audio_compile_check)
