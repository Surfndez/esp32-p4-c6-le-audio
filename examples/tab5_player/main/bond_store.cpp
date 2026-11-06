// Bond store of the example: the same NVS layout as the Cyberdeck firmware (partition "nvs", namespace "sec",
// keys "bt.bond" / "bt.irk"; identity in namespace "cfg", key "bt.ident"), so earbuds already paired by that
// firmware connect without pairing.
//
// With CONFIG_TAB5_PLAYER_SAVE_BONDS off (default on): READ-ONLY. NVS is opened through a copy of the partition descriptor
// with readonly = true, so the NVS library skips its start-up clean-up writes and esp_partition_write/erase refuse
// every write (ESP_ERR_NOT_ALLOWED). Values are copied to RAM once, at boot, from the main task (internal stack):
// the LE Audio host calls the store from threads with PSRAM stacks, which must not touch flash.
#include "bond_store.h"

#include <cstdio>
#include <cstring>
#include <string>

#include "esp_log.h"
#include "esp_partition.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

namespace {

constexpr const char* kTag = "bonds";

std::string g_bond, g_irk;  // RAM copies (secrets)
bool g_hasBond = false, g_hasIrk = false;
SemaphoreHandle_t g_lock = nullptr;

bool readStr(nvs_handle_t h, const char* key, std::string& out)
{
    size_t len = 0;
    if (nvs_get_str(h, key, nullptr, &len) != ESP_OK || len == 0 || len > 1024) {
        return false;
    }
    out.assign(len, '\0');
    if (nvs_get_str(h, key, out.data(), &len) != ESP_OK) {
        out.clear();
        return false;
    }
    out.resize(len > 0 ? len - 1 : 0);  // without the terminating NUL
    return true;
}

bool kvGet(const char* key, char* out, size_t cap, void*)
{
    xSemaphoreTake(g_lock, portMAX_DELAY);
    const std::string* v = nullptr;
    if (std::strcmp(key, "bt.bond") == 0 && g_hasBond) {
        v = &g_bond;
    } else if (std::strcmp(key, "bt.irk") == 0 && g_hasIrk) {
        v = &g_irk;
    }
    const bool ok = v != nullptr && v->size() < cap;
    if (ok) {
        std::memcpy(out, v->c_str(), v->size() + 1);
    }
    xSemaphoreGive(g_lock);
    return ok;
}

#if CONFIG_TAB5_PLAYER_SAVE_BONDS
// Writes go to an internal-stack task (flash access from a PSRAM stack asserts while the cache is disabled).
struct WriteReq {
    char key[16];
    char value[520];
};
QueueHandle_t g_writes = nullptr;

void writerTask(void*)
{
    WriteReq r;
    for (;;) {
        if (xQueueReceive(g_writes, &r, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        nvs_handle_t h;
        esp_err_t err = nvs_open("sec", NVS_READWRITE, &h);
        if (err == ESP_OK) {
            err = nvs_set_str(h, r.key, r.value);
            if (err == ESP_OK) {
                err = nvs_commit(h);
            }
            nvs_close(h);
        }
        ESP_LOGI(kTag, "saved %s: %s", r.key, esp_err_to_name(err));
        std::memset(&r, 0, sizeof(r));
    }
}

bool kvSet(const char* key, const char* value, void*)
{
    WriteReq r = {};
    if (std::strlen(key) >= sizeof(r.key) || std::strlen(value) >= sizeof(r.value)) {
        return false;
    }
    std::strcpy(r.key, key);
    std::strcpy(r.value, value);
    xSemaphoreTake(g_lock, portMAX_DELAY);
    if (std::strcmp(key, "bt.bond") == 0) {
        g_bond    = value;
        g_hasBond = true;
    } else if (std::strcmp(key, "bt.irk") == 0) {
        g_irk    = value;
        g_hasIrk = true;
    }
    xSemaphoreGive(g_lock);
    const bool ok = xQueueSend(g_writes, &r, pdMS_TO_TICKS(100)) == pdTRUE;
    std::memset(&r, 0, sizeof(r));
    return ok;
}
#endif

}  // namespace

int bond_store_count(void)
{
    // Entries of the saved list (formatBonds: ',' separated); the host parses and validates them
    if (g_lock == nullptr) {
        return 0;
    }
    xSemaphoreTake(g_lock, portMAX_DELAY);
    int n = g_hasBond && !g_bond.empty() ? 1 : 0;
    for (char c : g_bond) {
        n += c == ',' ? 1 : 0;
    }
    xSemaphoreGive(g_lock);
    return n;
}

bool bond_store_load(le_audio_kv_t* kv, int* identity)
{
    g_lock    = xSemaphoreCreateMutex();
    *identity = 1;
    if (g_lock == nullptr) {
        ESP_LOGE(kTag, "out of memory - no saved earbuds");
        kv->get = nullptr;
        kv->set = nullptr;
        return false;
    }  // Cyberdeck default: public address, no IRK
    kv->get   = kvGet;
    kv->set   = nullptr;
    kv->ctx   = nullptr;
#if CONFIG_TAB5_PLAYER_START_FRESH
    ESP_LOGW(kTag, "TEST START_FRESH: saved earbuds ignored, NVS not opened - new pairings stay in RAM");
    return false;
#endif

    const esp_partition_t* p =
        esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_NVS, "nvs");
    if (p == nullptr) {
        ESP_LOGW(kTag, "no \"nvs\" partition - no saved earbuds");
        return false;
    }
#if CONFIG_TAB5_PLAYER_SAVE_BONDS
    esp_err_t err = nvs_flash_init_partition_ptr(p);  // writable; never erased on errors
    ESP_LOGW(kTag, "bond store WRITABLE (CONFIG_TAB5_PLAYER_SAVE_BONDS): new pairings are saved to NVS");
#else
    static esp_partition_t ro;  // NVS keeps the pointer for the partition's lifetime
    ro          = *p;
    ro.readonly = true;
    esp_err_t err = nvs_flash_init_partition_ptr(&ro);
#endif
    if (err != ESP_OK) {
        ESP_LOGW(kTag, "NVS not readable (%s) - no saved earbuds; nothing was changed", esp_err_to_name(err));
        return false;
    }
    nvs_handle_t h;
    if (nvs_open("sec", NVS_READONLY, &h) == ESP_OK) {
        g_hasBond = readStr(h, "bt.bond", g_bond);
        g_hasIrk  = readStr(h, "bt.irk", g_irk);
        nvs_close(h);
    }
    if (nvs_open("cfg", NVS_READONLY, &h) == ESP_OK) {
        int32_t v = 1;
        if (nvs_get_i32(h, "bt.ident", &v) == ESP_OK && v >= 0 && v <= 2) {
            *identity = (int)v;
        }
        nvs_close(h);
    }
#if CONFIG_TAB5_PLAYER_SAVE_BONDS
    g_writes = xQueueCreate(2, sizeof(WriteReq));
    if (g_writes != nullptr && xTaskCreate(writerTask, "bond-wr", 4096, nullptr, 3, nullptr) == pdPASS) {
        kv->set = kvSet;
    } else {
        ESP_LOGE(kTag, "bond writer not started - new pairings stay in RAM");
    }
#else
    // Read-only: done with NVS. Deinit leaves the flash untouched.
    nvs_flash_deinit_partition("nvs");
#endif
    ESP_LOGI(kTag, "saved bond list %s, own IRK %s, identity mode %d (%s)", g_hasBond ? "found" : "none",
             g_hasIrk ? "found" : "none", *identity, kv->set != nullptr ? "writable" : "read-only");
    return g_hasBond;
}
