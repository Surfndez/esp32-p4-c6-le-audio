// Platform glue (le_port.h): threads with PSRAM stacks, CPU boost through esp_pm locks, key/value store callbacks.
#include "le_port.h"

#include "le_audio.h"

#include <algorithm>
#include <atomic>
#include <mutex>
#include <thread>
#include <vector>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_pthread.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace leport {

namespace {

constexpr const char* kTag = "le_port";

std::mutex g_cfgLock;
Config g_cfg;
le_audio_kv_t g_kv = {nullptr, nullptr, nullptr};

// One pair of PM locks shared by all Boost objects (reference counted), created on first use.
std::mutex g_boostLock;
int g_boostCount              = 0;
esp_pm_lock_handle_t g_cpuMax = nullptr;
esp_pm_lock_handle_t g_noSleep = nullptr;
bool g_pmTried                = false;

}  // namespace

void spawn(const char* name, size_t stack, std::function<void()> fn, int prio, int core)
{
    esp_pthread_cfg_t cfg = esp_pthread_get_default_config();
    cfg.stack_size        = stack;
    cfg.thread_name       = name;
    cfg.stack_alloc_caps  = MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT;
    cfg.prio              = prio;
    cfg.pin_to_core       = core < 0 ? tskNO_AFFINITY : core;
    const esp_err_t err = esp_pthread_set_cfg(&cfg);
    if (err != ESP_OK) {
        // The thread would get the default stack - for the stream thread (12 kB, LC3) that is not enough
        ESP_LOGE(kTag, "thread %s: config rejected (%s), not started", name, esp_err_to_name(err));
        return;
    }
    std::thread(std::move(fn)).detach();
    esp_pthread_cfg_t def = esp_pthread_get_default_config();
    esp_pthread_set_cfg(&def);
}

Boost::Boost()
{
    std::lock_guard<std::mutex> l(g_boostLock);
    if (!g_pmTried) {
        g_pmTried = true;
        // ESP_ERR_NOT_SUPPORTED without CONFIG_PM_ENABLE: the CPU never scales down, nothing to hold
        const esp_err_t a = esp_pm_lock_create(ESP_PM_CPU_FREQ_MAX, 0, "le_audio_cpu", &g_cpuMax);
        const esp_err_t b = esp_pm_lock_create(ESP_PM_NO_LIGHT_SLEEP, 0, "le_audio_nosleep", &g_noSleep);
        if (a != ESP_OK || b != ESP_OK) {
            ESP_LOGI(kTag, "power management locks unavailable (%s / %s) - CPU stays at its configured speed",
                     esp_err_to_name(a), esp_err_to_name(b));
        }
    }
    if (g_boostCount++ == 0) {
        if (g_cpuMax != nullptr) {
            esp_pm_lock_acquire(g_cpuMax);
        }
        if (g_noSleep != nullptr) {
            esp_pm_lock_acquire(g_noSleep);
        }
    }
}

Boost::~Boost()
{
    std::lock_guard<std::mutex> l(g_boostLock);
    if (--g_boostCount == 0) {
        if (g_cpuMax != nullptr) {
            esp_pm_lock_release(g_cpuMax);
        }
        if (g_noSleep != nullptr) {
            esp_pm_lock_release(g_noSleep);
        }
    }
}

const Config& config()
{
    // Written by le_audio_init before the first host thread starts; a retried init writes the same values.
    return g_cfg;
}

void setConfig(const Config& c)
{
    std::lock_guard<std::mutex> l(g_cfgLock);
    g_cfg = c;
}

void setKv(const le_audio_kv_t& kv)
{
    std::lock_guard<std::mutex> l(g_cfgLock);
    g_kv = kv;
}

bool kvGet(const char* key, std::string& out)
{
    le_audio_kv_t kv;
    {
        std::lock_guard<std::mutex> l(g_cfgLock);
        kv = g_kv;
    }
    if (kv.get == nullptr) {
        return false;
    }
    std::vector<char> buf(512, '\0');
    if (!kv.get(key, buf.data(), buf.size(), kv.ctx)) {
        std::fill(buf.begin(), buf.end(), '\0');
        return false;
    }
    buf.back() = '\0';
    out.assign(buf.data());
    std::fill(buf.begin(), buf.end(), '\0');  // secrets: no copy left on the heap
    return true;
}

bool kvSet(const char* key, const std::string& value)
{
    le_audio_kv_t kv;
    {
        std::lock_guard<std::mutex> l(g_cfgLock);
        kv = g_kv;
    }
    return kv.set != nullptr && kv.set(key, value.c_str(), kv.ctx);
}

bool kvWritable()
{
    std::lock_guard<std::mutex> l(g_cfgLock);
    return g_kv.set != nullptr;
}

}  // namespace leport
