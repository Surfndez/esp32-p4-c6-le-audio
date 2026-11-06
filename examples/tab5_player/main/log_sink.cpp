// Console robustness for field tests without a PC watching:
// 1. USB Serial/JTAG through its driver: a write without a host waits at most 50 ms once and then drops data
//    without blocking, so a missing / late host does not stall the tasks that log.
// 2. Optional copy of every log line to the SD card (/sd/tab5_player.log, previous run in tab5_player.old):
//    the log hook only appends to a RAM ring (never blocks, drops when busy or full), a low-priority task writes it
//    out every 2 s and fsyncs, so the log survives the reset that ends a test.
#include "log_sink.h"

#include <cstdarg>
#include <cstdio>
#include <cstring>

#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include <unistd.h>

namespace {

constexpr const char* kTag    = "log_sink";
constexpr size_t kRing        = 32 * 1024;
constexpr long kMaxFileBytes  = 4L * 1024 * 1024;

vprintf_like_t s_prev       = nullptr;
SemaphoreHandle_t s_lock    = nullptr;
char* s_ring                = nullptr;  // PSRAM; s_fill bytes pending
size_t s_fill               = 0;
uint32_t s_dropped          = 0;
char s_path[64]             = {};

int hook(const char* fmt, va_list ap)
{
    va_list copy;
    va_copy(copy, ap);
    const int n = s_prev != nullptr ? s_prev(fmt, ap) : vprintf(fmt, ap);
    char line[256];
    const int len = vsnprintf(line, sizeof(line), fmt, copy);
    va_end(copy);
    if (len > 0 && xSemaphoreTake(s_lock, 0) == pdTRUE) {
        const size_t l = (size_t)len < sizeof(line) ? (size_t)len : sizeof(line) - 1;
        if (s_fill + l <= kRing) {
            std::memcpy(s_ring + s_fill, line, l);
            s_fill += l;
        } else {
            s_dropped++;
        }
        xSemaphoreGive(s_lock);
    }
    return n;
}

void writerTask(void*)
{
    char* out = static_cast<char*>(heap_caps_malloc(kRing, MALLOC_CAP_SPIRAM));
    FILE* f   = std::fopen(s_path, "w");
    if (out == nullptr || f == nullptr) {
        ESP_LOGE(kTag, "SD log not started (%s)", out == nullptr ? "no memory" : "cannot create file");
        vTaskDelete(nullptr);
        return;
    }
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(2000));
        size_t n         = 0;
        uint32_t dropped = 0;
        xSemaphoreTake(s_lock, portMAX_DELAY);
        n = s_fill;
        std::memcpy(out, s_ring, n);
        s_fill    = 0;
        dropped   = s_dropped;
        s_dropped = 0;
        xSemaphoreGive(s_lock);
        if (n == 0) {
            continue;
        }
        if (dropped > 0) {
            std::fprintf(f, "[log_sink: %lu line(s) dropped]\n", (unsigned long)dropped);
        }
        std::fwrite(out, 1, n, f);
        std::fflush(f);
        fsync(fileno(f));  // the test usually ends with a reset
        if (std::ftell(f) > kMaxFileBytes) {
            std::fclose(f);
            f = std::fopen(s_path, "w");
            if (f == nullptr) {
                vTaskDelete(nullptr);
                return;
            }
        }
    }
}

}  // namespace

void log_sink_usb_init(void)
{
    usb_serial_jtag_driver_config_t cfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    cfg.tx_buffer_size                  = 4096;
    cfg.rx_buffer_size                  = 256;
    if (usb_serial_jtag_driver_install(&cfg) == ESP_OK) {
        usb_serial_jtag_vfs_use_driver();
    }
}

void log_sink_sd_start(const char* dir)
{
#if CONFIG_TAB5_PLAYER_LOG_TO_SD
    s_lock = xSemaphoreCreateMutex();
    s_ring = static_cast<char*>(heap_caps_malloc(kRing, MALLOC_CAP_SPIRAM));
    if (s_lock == nullptr || s_ring == nullptr) {
        ESP_LOGE(kTag, "SD log: no memory");
        return;
    }
    char old[64];
    std::snprintf(s_path, sizeof(s_path), "%s/tab5_player.log", dir);
    std::snprintf(old, sizeof(old), "%s/tab5_player.old", dir);
    std::remove(old);
    std::rename(s_path, old);  // previous run kept once
    s_prev = esp_log_set_vprintf(hook);
    if (xTaskCreate(writerTask, "log-sd", 6144, nullptr, 2, nullptr) != pdPASS) {
        esp_log_set_vprintf(s_prev);
        ESP_LOGE(kTag, "SD log: task not started");
        return;
    }
    ESP_LOGI(kTag, "log copied to %s every 2 s (previous run: %s)", s_path, old);
#else
    (void)dir;
#endif
}
