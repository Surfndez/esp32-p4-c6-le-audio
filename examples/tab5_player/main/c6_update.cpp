// Embedded ESP32-C6 firmware and its installation through p4_le_audio's le_audio_c6_install() (header check, then
// esp_hosted's OTA RPC begin / write / end). The C6 checks the checksum and SHA-256 of the whole image before
// switching its boot partition; a failed or interrupted transfer leaves its current image in place.
#include "c6_update.h"

#include <cstdint>

#include "le_audio.h"

extern const uint8_t c6_image_start[] asm("_binary_network_adapter_bin_start");
extern const uint8_t c6_image_end[] asm("_binary_network_adapter_bin_end");

namespace {

size_t imageSize()
{
    return (size_t)(c6_image_end - c6_image_start);
}

void forwardProgress(int pct, void* ctx)
{
    reinterpret_cast<void (*)(int)>(ctx)(pct);
}

}  // namespace

const char* c6_update_embedded_version(void)
{
    static char version[33];
    if (version[0] == '\0' && le_audio_c6_image_version(c6_image_start, imageSize(), version, sizeof(version)) != ESP_OK) {
        version[0] = '\0';
    }
    return version;
}

esp_err_t c6_update_install(void (*progress)(int pct))
{
    return le_audio_c6_install(c6_image_start, imageSize(), progress != nullptr ? forwardProgress : nullptr,
                               reinterpret_cast<void*>(progress));
}
