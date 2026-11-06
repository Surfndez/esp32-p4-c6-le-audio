#pragma once
// Platform glue of the LE Audio host: threads, CPU boost, key/value store and configuration. Replaces the Cyberdeck
// application services (app::task, app::energy, app::settings, GetHAL()->kv*) that hal_bt.cpp / hal_le_audio.cpp used.

#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>

#include "esp_pm.h"
#include "le_audio.h"

namespace leport {

// std::thread with the stack in PSRAM (like Cyberdeck app::task::spawn). Code running there must not touch flash
// (NVS, SPI flash) - the key/value callbacks are called from such threads, see le_audio_kv_t.
void spawn(const char* name, size_t stack, std::function<void()> fn, int prio = 4, int core = -1);

// Full CPU speed and no light sleep for the lifetime of the object (the stream thread holds one for the whole
// stream). Without it, light sleep in screen standby shifted esp_timer and SDUs went out ~3 % fast. With
// CONFIG_PM_ENABLE off this is a no-op (the CPU already runs at full speed).
class Boost {
public:
    Boost();
    ~Boost();
    Boost(const Boost&)            = delete;
    Boost& operator=(const Boost&) = delete;
};

struct Config {
    int identity         = 1;  // 0 = static random + IRK, 1 = public without IRK (default), 2 = public + IRK
    bool allowPairing    = false;
    std::string deviceName = "ESP32-P4 LE Audio";
};
const Config& config();
void setConfig(const Config& c);

// Key/value store of the application (keys "bt.bond", "bt.irk"). false = missing / not stored.
void setKv(const le_audio_kv_t& kv);
bool kvGet(const char* key, std::string& out);
bool kvSet(const char* key, const std::string& value);
bool kvWritable();

}  // namespace leport
