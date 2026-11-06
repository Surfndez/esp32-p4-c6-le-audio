// Compile check of p4_le_audio: the bring-up order of docs/integration.md in one place. It links every public call
// family so that a broken component fails the build. On hardware it would connect saved earbuds and stream silence.
#include <stdio.h>
#include <string.h>

#include "esp_hosted_cyberdeck.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "le_audio.h"

static const char *TAG = "compile_check";

// Bond store kept in RAM only (a real firmware loads "bt.bond" / "bt.irk" from its storage before le_audio_init and
// writes them back from set, outside the host's PSRAM-stack threads).
static char s_bond[520], s_irk[40];

static bool kv_get(const char *key, char *out, size_t cap, void *ctx)
{
    (void)ctx;
    const char *v = strcmp(key, "bt.bond") == 0 ? s_bond : strcmp(key, "bt.irk") == 0 ? s_irk : NULL;
    if (v == NULL || v[0] == '\0') {
        return false;
    }
    snprintf(out, cap, "%s", v);
    return true;
}

static bool kv_set(const char *key, const char *value, void *ctx)
{
    (void)ctx;
    char *dst = strcmp(key, "bt.bond") == 0 ? s_bond : strcmp(key, "bt.irk") == 0 ? s_irk : NULL;
    if (dst == NULL || strlen(value) >= (strcmp(key, "bt.bond") == 0 ? sizeof(s_bond) : sizeof(s_irk))) {
        return false;
    }
    strcpy(dst, value);
    return true;
}

void app_main(void)
{
    if (esp_hosted_cyberdeck_connect(30000) != ESP_OK) {  // a BLE-only firmware brings the SDIO transport up itself
        ESP_LOGE(TAG, "no SDIO transport to the ESP32-C6");
        return;
    }
    le_audio_config_t cfg = LE_AUDIO_CONFIG_DEFAULT();
    cfg.kv            = (le_audio_kv_t){.get = kv_get, .set = kv_set, .ctx = NULL};
    cfg.allow_pairing = true;
    cfg.device_name   = "LE Audio compile check";
    const esp_err_t err = le_audio_init(&cfg);
    if (err != ESP_OK) {
        le_audio_status_t st;
        le_audio_get_status(&st);
        ESP_LOGE(TAG, "le_audio_init: %s, C6 firmware %s", esp_err_to_name(err), st.coprocessor_fw);
        // ESP_ERR_NOT_SUPPORTED: the C6 runs a firmware without CIS. A real firmware links c6_firmware's image and,
        // after the user agrees, calls le_audio_c6_install(image, size, progress, ctx). Referenced here so the OTA
        // path (esp_hosted rpc_ota_*) is linked by this check.
        ESP_LOGI(TAG, "C6 installer: %p", (void *)le_audio_c6_install);
        return;
    }
    if (le_audio_connect_bonded(5) != ESP_OK) {
        le_audio_device_t found[8];
        const int n = le_audio_scan(found, 8, 6);
        for (int i = 0; i < n; i++) {
            ESP_LOGI(TAG, "%s %s %d dBm%s%s", found[i].addr, found[i].name, found[i].rssi,
                     found[i].le_audio ? " LE Audio" : "", found[i].bonded ? " saved" : "");
        }
        if (n > 0 && found[0].le_audio) {
            le_audio_connect_device(&found[0]);  // a real firmware lets the user pick
        }
    }
    static int16_t silence[480 * 2];  // 10 ms of 48 kHz stereo
    for (;;) {
        if (le_audio_write_pcm(silence, 480, 200) == 0) {
            vTaskDelay(pdMS_TO_TICKS(100));  // stream not up yet
        }
        le_audio_media_cmd_t cmd;
        while (le_audio_take_media_command(&cmd)) {
            ESP_LOGI(TAG, "earbud gesture %d", (int)cmd);
        }
    }
}
