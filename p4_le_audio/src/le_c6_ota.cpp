// ESP32-C6 firmware update from the P4: header check (models/bt/c6_image_model.cpp), then esp_hosted's OTA RPC
// begin / write / end - the same steps the firmware this component comes from uses for its C6 updates. The C6
// verifies the checksum and SHA-256 of the whole image before it switches its boot partition; a failed or
// interrupted transfer leaves its current image in place.
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <string>

#include "bt/c6_image_model.h"
#include "esp_log.h"
#include "le_audio.h"

extern "C" {
// esp_hosted host, drivers/rpc/wrap/rpc_wrap.c (synchronous; ota_write takes one chunk of up to 1400 B)
int rpc_ota_begin(void);
int rpc_ota_write(uint8_t* ota_data, uint32_t ota_data_len);
int rpc_ota_end(void);
}

namespace {

constexpr const char* kTag = "le_audio_c6";
constexpr size_t kChunk    = 1400;  // CHUNK_SIZE of rpc_wrap.c (one ota_write RPC)

}  // namespace

esp_err_t le_audio_c6_image_version(const uint8_t* img, size_t len, char* out, size_t cap)
{
    std::string version;
    const char* err = nullptr;
    if (!app::c6image::parse(img, len, version, err)) {
        ESP_LOGW(kTag, "not a c6_firmware image: %s", err);
        return ESP_ERR_INVALID_ARG;
    }
    if (out != nullptr && cap > 0) {
        std::snprintf(out, cap, "%s", version.c_str());
    }
    return ESP_OK;
}

esp_err_t le_audio_c6_install(const uint8_t* img, size_t len, void (*progress)(int pct, void* ctx), void* ctx)
{
    std::string version;
    const char* err = nullptr;
    if (!app::c6image::parse(img, len, version, err)) {
        ESP_LOGE(kTag, "not sent: %s", err);
        return ESP_ERR_INVALID_ARG;
    }
    ESP_LOGW(kTag, "installing C6 firmware %s (%u B)", version.c_str(), (unsigned)len);
    // Erasing the C6 slot takes a few seconds; the C6 refuses while its current image is on trial
    if (rpc_ota_begin() != 0) {
        ESP_LOGE(kTag, "C6 refused OTA (current image still on trial, or slot erase failed) - nothing changed");
        return ESP_ERR_INVALID_STATE;
    }
    static uint8_t chunk[kChunk];  // rpc_ota_write takes a writable buffer; the image may be in flash
    int lastPct = -1;
    for (size_t off = 0; off < len; off += kChunk) {
        const size_t n = std::min(kChunk, len - off);
        std::memcpy(chunk, img + off, n);
        if (rpc_ota_write(chunk, (uint32_t)n) != 0) {
            // Without rpc_ota_end the C6 keeps booting its current image
            ESP_LOGE(kTag, "write failed at %u B - C6 keeps its current image", (unsigned)off);
            return ESP_FAIL;
        }
        const int pct = (int)((off + n) * 100 / len);
        if (progress != nullptr && pct / 5 != lastPct / 5) {
            progress(pct, ctx);
            lastPct = pct;
        }
    }
    if (rpc_ota_end() != 0) {
        ESP_LOGE(kTag, "C6 rejected the image at verification - it keeps its current image");
        return ESP_FAIL;
    }
    ESP_LOGW(kTag, "C6 firmware %s written and verified - the C6 restarts into it", version.c_str());
    return ESP_OK;
}
