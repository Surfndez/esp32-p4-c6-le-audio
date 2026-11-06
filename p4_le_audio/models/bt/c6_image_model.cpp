#include "bt/c6_image_model.h"

#include <cstring>

namespace app::c6image {

namespace {

uint32_t le32(const uint8_t* p)
{
    return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}

}  // namespace

bool parse(const uint8_t* img, size_t len, std::string& version, const char*& err)
{
    if (img == nullptr || len < kHeaderBytes) {
        err = "too short for an ESP image header";
        return false;
    }
    if (len > kMaxImage) {
        err = "larger than the C6 OTA slot (1.5 MB)";
        return false;
    }
    if (img[0] != 0xE9) {
        err = "not an ESP image (magic)";
        return false;
    }
    if ((uint16_t)(img[12] | img[13] << 8) != kChipEsp32c6) {
        err = "image for another chip than the ESP32-C6";
        return false;
    }
    if (img[23] != 1) {
        err = "no appended SHA-256 - the C6 could not verify the whole image";
        return false;
    }
    if (le32(img + 32) != kAppMagic) {
        err = "no application descriptor (esp_app_desc_t)";
        return false;
    }
    if (strncmp(reinterpret_cast<const char*>(img + 80), kProject, 32) != 0) {
        err = "not the c6_firmware project (network_adapter)";
        return false;
    }
    version.assign(reinterpret_cast<const char*>(img + 48), strnlen(reinterpret_cast<const char*>(img + 48), 32));
    return true;
}

}  // namespace app::c6image
