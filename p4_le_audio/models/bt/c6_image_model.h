#pragma once
// Header check of an ESP32-C6 firmware image of this project before it is sent to the C6 (pure part, host test
// test/host/test_c6_image.cpp). The C6 verifies the checksum and SHA-256 of the whole image itself; this check only
// makes sure nothing that is not a c6_firmware image starts an OTA at all.

#include <cstddef>
#include <cstdint>
#include <string>

namespace app::c6image {

constexpr size_t kHeaderBytes = 112;       // image header + first segment header + esp_app_desc_t up to project_name
constexpr size_t kMaxImage    = 0x180000;  // OTA slot of c6_firmware/partitions.esp32c6.csv
constexpr uint16_t kChipEsp32c6 = 0x000D;  // esp_image_header_t.chip_id
constexpr uint32_t kAppMagic  = 0xABCD5432u;
constexpr const char* kProject = "network_adapter";

// true: ESP image magic, chip ESP32-C6, appended SHA-256, esp_app_desc_t of the c6_firmware project, size within
// the OTA slot; version = esp_app_desc_t.version. false: err says why (static string).
bool parse(const uint8_t* img, size_t len, std::string& version, const char*& err);

}  // namespace app::c6image
