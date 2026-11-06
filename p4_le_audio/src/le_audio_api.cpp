// Public C API (include/le_audio.h) over the host (le_host.cpp) and the stream (le_stream.cpp), plus the background
// manager: reconnect saved earbuds, join the second earbud of the set, start the stream. The manager is the
// service of the firmware this project was extracted from, with the same pure models (models/bt/bt_reconnect_model,
// bt_audio_model).
#include "le_audio.h"

#include "le_bt_types.h"
#include "le_host_internal.h"
#include "le_port.h"

#include <bt/bt_audio_model.h>
#include <bt/bt_reconnect_model.h>

#include <algorithm>
#include <atomic>
#include <cstdio>
#include <cstring>
#include <mutex>

#include "esp_hosted_cyberdeck.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {

constexpr const char* kTag = "le_audio";

std::atomic<bool> g_init{false};
std::atomic<int> g_volume{60};
std::atomic<uint32_t> g_lost{0};
// User actions for the manager's models (AutoStart::userStopped / Reconnect::userDisconnected, called from the UI)
std::atomic<bool> g_userStopped{false}, g_userDisconnected{false};
le_audio_config_t g_cfg = LE_AUDIO_CONFIG_DEFAULT();

uint32_t nowMs()
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

void copyStr(char* dst, size_t cap, const std::string& s)
{
    std::snprintf(dst, cap, "%s", s.c_str());
}

// Joining the second earbud of the set works right after the main connection, BEFORE the CIG exists; with a
// streaming CIG the second connection fails (0x3E / encryption on a dead link - known limitation). So: join first,
// start the stream after it; a second earbud that comes back later gets the stream stopped, joined, restarted.
// One join procedure at a time (manager thread, UI command task, API callers).
std::mutex g_joinLock;
std::atomic<bool> g_joinBusy{false};  // the manager's auto start waits (AutoStart input busy)
constexpr int kJoinWaitMs   = 15000;  // background join started by the connection (scan 4 s + connect + keys)
constexpr int kJoinRetryMs  = 6000;   // one more attempt within this budget when the first did not find it
constexpr int kProbeMs      = 4000;   // scan for the second earbud while the stream keeps playing

bool setJoining()
{
    return halBtLeAudioLink().setJoining;
}

// After the main earbud connected (or before a stream start): the set's second earbud joins before the CIG.
void joinBeforeStream(const char* why)
{
    std::lock_guard<std::mutex> jl(g_joinLock);
    g_joinBusy = true;
    const int64_t t0 = esp_timer_get_time();
    if (setJoining()) {
        ESP_LOGI(kTag, "JOIN 1/3 (%s): second earbud joining in the background - waiting (max %d s)", why,
                 kJoinWaitMs / 1000);
        while (setJoining() && esp_timer_get_time() - t0 < (int64_t)kJoinWaitMs * 1000) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
    auto k = lehost::btLink();
    if (k.state == lehost::BtState::Connected && k.setMissing) {
        ESP_LOGI(kTag, "JOIN 2/3 (%s): second earbud missing - one more scan + join (budget %d s)", why,
                 kJoinRetryMs / 1000);
        const int64_t t1 = esp_timer_get_time();
        lehost::btJoinSet();  // RSI scan up to 4 s, then connection with its saved key
        ESP_LOGI(kTag, "JOIN 2/3: attempt took %d ms", (int)((esp_timer_get_time() - t1) / 1000));
        k = lehost::btLink();
    }
    const bool second = halBtLeAudioLink().acl2 != 0xFFFF;
    ESP_LOGI(kTag, "JOIN 3/3 (%s): %s after %d ms - stream %s", why,
             second ? "second earbud joined" : k.setMissing ? "second earbud NOT joined" : "no second earbud in the set",
             (int)((esp_timer_get_time() - t0) / 1000), second ? "starts in stereo" : "starts with one earbud");
    g_joinBusy = false;
}

// Like connectKnown() of the original: scan, the first saved LE Audio earbud, its saved key only. Then the second
// earbud of the set, before anything starts a stream.
lehost::BtReconnect connectKnown(int scanS)
{
    for (const auto& d : lehost::btScan(scanS)) {
        if (!d.bonded || !(d.leAudio || d.audioCandidate)) {
            continue;
        }
        ESP_LOGI(kTag, "saved earbud %s (%s, %d dBm) - connecting with its key", d.addr.c_str(), d.name.c_str(),
                 d.rssi);
        const auto r = lehost::btReconnect(d.addr, d.addrType, d.name);
        if (r == lehost::BtReconnect::Connected) {
            joinBeforeStream("after connect");
        }
        return r;
    }
    return lehost::BtReconnect::Failed;
}

// Stream start for the API and the manager: a missing / joining second earbud first.
bool startStream()
{
    const auto k = lehost::btLink();
    if (k.state == lehost::BtState::Connected && (k.setMissing || setJoining())) {
        joinBeforeStream("before stream start");
    }
    return lehost::btLeAudioStart(false);
}

// Second earbud of the set while connected. Not streaming: join. Streaming with one earbud: scan first (the stream
// keeps playing), and only if the second earbud is there: stop the stream (CIG removed), join, start again in
// stereo - a short gap instead of a join that fails against the running CIG.
bool joinSet()
{
    std::unique_lock<std::mutex> jl(g_joinLock);
    g_joinBusy = true;
    const auto st = lehost::btLeAudioStatus();
    bool ok       = false;
    if (st.phase == lehost::LeAudioPhase::Streaming && st.members < 2) {
        ESP_LOGI(kTag, "JOIN late 1/4: streaming with one earbud - scanning for the second (%d s)", kProbeMs / 1000);
        if (!lehost::btFindSetMember(kProbeMs)) {
            ESP_LOGI(kTag, "JOIN late 1/4: second earbud not advertising - stream untouched");
        } else {
            ESP_LOGI(kTag, "JOIN late 2/4: second earbud found - stopping the stream (CIG off) to join it");
            lehost::btLeAudioStop();
            ok = lehost::btJoinSet();
            ESP_LOGI(kTag, "JOIN late 3/4: second earbud %s", ok ? "joined" : "NOT joined");
            const bool started = lehost::btLeAudioStart(false);
            const auto now     = lehost::btLeAudioStatus();
            ESP_LOGI(kTag, "JOIN late 4/4: stream %s, earbuds %d%s", started ? "restarted" : "NOT restarted",
                     now.members, now.stereo ? " (stereo)" : "");
            if (!started) {
                g_lost++;
            }
        }
    } else {
        ok = lehost::btJoinSet();
        ESP_LOGI(kTag, "JOIN: second earbud %s (no stream running)", ok ? "joined" : "NOT joined");
    }
    g_joinBusy = false;
    return ok;
}

void managerThread()
{
    using app::btreconnect::Action;
    app::btreconnect::Reconnect reconnect;
    app::btaudio::AutoStart autoStart;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        if (g_userStopped.exchange(false)) {
            autoStart.userStopped();  // no automatic start until the next connection
        }
        if (g_userDisconnected.exchange(false)) {
            reconnect.userDisconnected();  // no automatic reconnect until the next (manual) connection
        }
        const auto k = lehost::btLink();
        if (g_cfg.auto_connect) {
            app::btreconnect::Inputs in;
            in.nowMs      = nowMs();
            in.enabled    = true;
            in.radioOn    = k.state != lehost::BtState::Off;
            in.hasBonds   = lehost::btBondCount() != 0;
            in.connected  = k.state == lehost::BtState::Connected || k.streamActive;
            in.setMissing = k.setMissing;
            in.busy       = k.state == lehost::BtState::Scanning || k.state == lehost::BtState::Connecting ||
                      g_joinBusy.load();
            const Action a = reconnect.tick(in);
            if (a != Action::None) {
                bool ok = false, rejected = false;
                if (a == Action::RadioOn) {
                    lehost::btSetRadio(true);
                    ok = lehost::btLink().state != lehost::BtState::Off;
                } else if (a == Action::Connect) {
                    const auto r = connectKnown(3);
                    ok           = r == lehost::BtReconnect::Connected;
                    rejected     = r == lehost::BtReconnect::KeyRejected;
                } else if (a == Action::JoinSet) {
                    ok = joinSet();
                }
                reconnect.done(a, ok, nowMs());
                if (rejected) {
                    // The earbuds forgot the pairing: no pairing in the background (security) - wait for the user
                    reconnect.userDisconnected();
                    ESP_LOGW(kTag, "earbuds rejected the saved key - pair them again (auto-connect paused)");
                }
                continue;  // link state changed - fresh snapshot for the auto start
            }
        }
        if (g_cfg.auto_start) {
            app::btaudio::Inputs in;
            in.nowMs      = nowMs();
            in.enabled    = true;
            in.connected  = k.state == lehost::BtState::Connected;
            in.ready      = k.paired && k.leAudio;
            in.connHandle = k.connHandle;
            in.streaming  = k.streamActive;
            in.busy       = g_joinBusy.load();  // start runs in this thread; a join elsewhere goes first
            if (autoStart.tick(in)) {
                if (!startStream()) {
                    ESP_LOGW(kTag, "auto start failed: %s", lehost::btLeAudioStatus().lastError.c_str());
                }
            }
        }
    }
}

}  // namespace

// Stream lost during playback (le_stream.cpp btLeAudioRestart failure): counted, the application decides.
void halAudioBtLost()
{
    g_lost++;
}

extern "C" {

esp_err_t le_audio_init(const le_audio_config_t* cfg)
{
    if (cfg == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    // Nothing goes to the C6 (RPC or HCI) before the SDIO transport is up: esp_hosted_cyberdeck_connect()
    if (!esp_hosted_cyberdeck_transport_up()) {
        ESP_LOGE(kTag, "esp_hosted transport to the C6 is not up - call esp_hosted_cyberdeck_connect() first");
        return ESP_ERR_INVALID_STATE;
    }
    if (g_init.exchange(true)) {
        return ESP_ERR_INVALID_STATE;
    }
    g_cfg = *cfg;
    leport::Config c;
    c.identity     = (int)cfg->identity;
    c.allowPairing = cfg->allow_pairing;
    if (cfg->device_name != nullptr && cfg->device_name[0] != '\0') {
        c.deviceName = cfg->device_name;
    }
    leport::setConfig(c);
    leport::setKv(cfg->kv);

    lehost::btSetRadio(true);
    const auto st = lehost::btStatus();
    if (st.state == lehost::BtState::Off || st.error == lehost::BtError::RadioDown) {
        ESP_LOGE(kTag, "controller did not start: %s", st.lastError.c_str());
        g_init = false;
        return ESP_ERR_INVALID_STATE;
    }
    const HalBtLeAudioLink link = halBtLeAudioLink();
    ESP_LOGI(kTag, "controller %s, C6 firmware %s, CIS Central %s, saved bonds %d", st.bdAddr.c_str(),
             st.coprocessorFw.c_str(), link.cisCentral ? "yes" : "NO", lehost::btBondCount());
    if (cfg->auto_connect || cfg->auto_start) {
        leport::spawn("le-audio-mgr", 16 * 1024, managerThread, 5);
    }
    if (!link.cisCentral) {
        ESP_LOGE(kTag, "C6 firmware without CIS Central (LE feature bit 28) - flash c6_firmware from this project");
        return ESP_ERR_NOT_SUPPORTED;
    }
    return ESP_OK;
}

esp_err_t le_audio_connect_bonded(int scan_s)
{
    if (!g_init) {
        return ESP_ERR_INVALID_STATE;
    }
    if (lehost::btLink().state == lehost::BtState::Connected) {
        return ESP_OK;
    }
    switch (connectKnown(std::clamp(scan_s, 1, 15))) {
        case lehost::BtReconnect::Connected: return ESP_OK;
        case lehost::BtReconnect::KeyRejected: return ESP_ERR_INVALID_RESPONSE;
        default: {
            // The background manager may have connected in the meantime
            const auto k = lehost::btLink();
            return k.state == lehost::BtState::Connected ? (k.paired ? ESP_OK : ESP_FAIL) : ESP_ERR_NOT_FOUND;
        }
    }
}

namespace {

// Connection with pairing allowed (checked by the caller): a saved device tries its key first and pairs again only if
// the key is rejected; the second earbud of the set pairs in the same session.
esp_err_t pairDevice(const std::string& addr, uint8_t addrType, const std::string& name, int rssi)
{
    ESP_LOGI(kTag, "pairing %s (%s, %d dBm)", addr.c_str(), name.c_str(), rssi);
    if (lehost::btConnect(addr, addrType, name) && lehost::btLink().paired) {
        joinBeforeStream("after pairing");
        return ESP_OK;
    }
    if (lehost::btLink().state == lehost::BtState::Connected) {
        lehost::btDisconnect();  // connected but not encrypted - LE Audio would not start on it
    }
    return ESP_FAIL;
}

}  // namespace

esp_err_t le_audio_pair_new(int scan_s)
{
    if (!g_init) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!leport::config().allowPairing) {
        return ESP_ERR_NOT_ALLOWED;
    }
    for (const auto& d : lehost::btScan(std::clamp(scan_s, 1, 15))) {
        if (d.leAudio && !d.bonded) {
            return pairDevice(d.addr, d.addrType, d.name, d.rssi);
        }
    }
    return ESP_ERR_NOT_FOUND;
}

esp_err_t le_audio_connect_device(const le_audio_device_t* dev)
{
    if (!g_init) {
        return ESP_ERR_INVALID_STATE;
    }
    if (dev == nullptr || dev->addr[0] == '\0') {
        return ESP_ERR_INVALID_ARG;
    }
    if (!dev->bonded && !leport::config().allowPairing) {
        return ESP_ERR_NOT_ALLOWED;
    }
    if (lehost::btLink().state == lehost::BtState::Connected) {
        // Not le_audio_disconnect(): its "user disconnected" flag would keep the background reconnect off after the
        // switch (the manager rarely sees the short gap before the new connection that clears it)
        ESP_LOGI(kTag, "closing the current connection for %s", dev->addr);
        lehost::btLeAudioStop();
        lehost::btDisconnect();
    }
    if (leport::config().allowPairing) {
        return pairDevice(dev->addr, dev->addr_type, dev->name, dev->rssi);
    }
    // Pairing off: the saved key only - a device that rejects it keeps the saved key
    ESP_LOGI(kTag, "saved device %s (%s, %d dBm) - connecting with its key", dev->addr, dev->name, dev->rssi);
    switch (lehost::btReconnect(dev->addr, dev->addr_type, dev->name)) {
        case lehost::BtReconnect::Connected: joinBeforeStream("after connect"); return ESP_OK;
        case lehost::BtReconnect::KeyRejected: return ESP_ERR_INVALID_RESPONSE;
        default: return ESP_FAIL;
    }
}

int le_audio_scan(le_audio_device_t* out, int max, int scan_s)
{
    if (!g_init || out == nullptr || max <= 0) {
        return 0;
    }
    int n = 0;
    for (const auto& d : lehost::btScan(std::clamp(scan_s, 1, 15))) {
        if (n >= max) {
            break;
        }
        le_audio_device_t& o = out[n++];
        o                    = le_audio_device_t{};
        copyStr(o.addr, sizeof(o.addr), d.addr);
        copyStr(o.name, sizeof(o.name), d.name);
        o.addr_type = d.addrType;
        o.rssi      = d.rssi;
        o.le_audio  = d.leAudio;
        o.bonded    = d.bonded;
    }
    return n;
}

esp_err_t le_audio_join_set(void)
{
    if (!g_init) {
        return ESP_ERR_INVALID_STATE;
    }
    return joinSet() ? ESP_OK : ESP_ERR_NOT_FOUND;
}

esp_err_t le_audio_start(void)
{
    if (!g_init) {
        return ESP_ERR_INVALID_STATE;
    }
    return startStream() ? ESP_OK : ESP_FAIL;
}

esp_err_t le_audio_restart(void)
{
    if (!g_init) {
        return ESP_ERR_INVALID_STATE;
    }
    return lehost::btLeAudioRestart(false) ? ESP_OK : ESP_FAIL;
}

void le_audio_stop(void)
{
    if (g_init) {
        g_userStopped = true;
        lehost::btLeAudioStop();
    }
}

void le_audio_disconnect(void)
{
    if (g_init) {
        g_userStopped      = true;
        g_userDisconnected = true;
        lehost::btLeAudioStop();
        lehost::btDisconnect();
    }
}

bool le_audio_ready(void)
{
    return halLeAudioSinkReady();
}

size_t le_audio_write_pcm(const int16_t* stereo, size_t frames, int timeout_ms)
{
    return halLeAudioWritePcm(stereo, frames, g_volume.load(), std::max(timeout_ms, 0));
}

void le_audio_set_volume(int pct)
{
    g_volume = std::clamp(pct, 0, 100);
}

void le_audio_get_status(le_audio_status_t* out)
{
    if (out == nullptr) {
        return;
    }
    *out              = le_audio_status_t{};
    const auto k      = lehost::btLink();
    const auto bs     = lehost::btStatus();
    const auto st     = lehost::btLeAudioStatus();
    const auto link   = halBtLeAudioLink();
    out->link         = (le_audio_link_state_t)k.state;
    out->paired       = k.paired;
    out->second_joined = link.acl2 != 0xFFFF;
    out->set_missing  = k.setMissing;
    out->battery_pct  = k.batteryPct;
    copyStr(out->coprocessor_fw, sizeof(out->coprocessor_fw), bs.coprocessorFw);
    copyStr(out->peer_addr, sizeof(out->peer_addr), bs.addr);
    copyStr(out->peer_name, sizeof(out->peer_name), bs.name);
    out->phase                 = (le_audio_stream_phase_t)st.phase;
    out->members               = st.members;
    out->stereo                = st.stereo;
    out->sink_rate_hz          = st.sinkRate;
    out->sink_octets           = st.sinkOctets;
    out->presentation_delay_us = st.presentationDelayUs;
    out->tx_sdu                = st.txSdu;
    out->tx_dropped            = st.txDropped;
    out->drop_no_credit        = st.dropNoCredit;
    out->drop_send_fail        = st.dropSendFail;
    out->drop_deadline         = st.dropDeadline;
    out->txq_sent              = st.txqSent;
    out->txq_max               = st.txqMax;
    out->credits_min           = st.creditsMin;
    out->credits_total         = st.creditsTotal;
    out->credits_recovered     = st.creditsRecovered;
    out->late_wakes            = st.lateWakes;
    out->slow_sends            = st.slowSends;
    out->pcm_short             = st.pcmShort;
    out->resyncs               = st.resyncs;
    out->skipped_frames        = st.skippedFrames;
    out->max_late_us           = st.maxLateUs;
    out->max_enc_us            = st.maxEncUs;
    out->max_send_us           = st.maxSendUs;
    out->max_stall_us          = st.maxStallUs;
    out->stream_lost           = g_lost.load();
    out->rssi                  = bs.rssi;
    for (int i = 0; i < 2; i++) {
        out->member_location[i]  = st.location[i];
        out->member_streaming[i] = st.cisUp[i];
    }
    copyStr(out->last_error, sizeof(out->last_error), !st.lastError.empty() ? st.lastError : bs.lastError);
}

esp_err_t le_audio_get_cis_stats(int member, le_audio_cis_stats_t* out)
{
    if (!g_init || out == nullptr || member < 0 || member > 1) {
        return ESP_ERR_INVALID_ARG;
    }
    const uint16_t h = lehost::btLeAudioStatus().cisHandle[member];
    if (h == 0xFFFF) {
        return ESP_ERR_NOT_FOUND;
    }
    // c6_firmware/main/cis_c6_core.c diag_handle: magic "CIS5", sub-command, CIS handle -> status, format version,
    // sub-command, then cis_engine_stats_t as u32 little endian (cis_ll/cis_engine.h field order)
    const uint8_t p[7] = {'C', 'I', 'S', '5', 0x01, (uint8_t)(h & 0xFF), (uint8_t)(h >> 8)};
    uint8_t ret[80];
    uint8_t rl = 0, status = 0xFF;
    if (!halBtHciCmd(0xFC30, p, sizeof(p), 400, &status, ret, sizeof(ret), &rl) || status != 0 || rl < 2 + 8 * 4 ||
        ret[1] != 0x01) {
        return ESP_ERR_NOT_FOUND;
    }
    auto u32 = [&](int i) {
        const uint8_t* b = &ret[2 + 4 * i];
        return (uint32_t)b[0] | ((uint32_t)b[1] << 8) | ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24);
    };
    *out             = le_audio_cis_stats_t{};
    out->cis_handle  = h;
    out->events      = u32(0);
    out->tx_acked    = u32(3);
    out->tx_flushed  = u32(4);
    out->tx_underrun = u32(5);
    out->sdu_in      = u32(6);
    out->sdu_dropped = u32(7);
    out->sdu_late    = u32(8);
    const uint32_t all = out->tx_acked + out->tx_flushed;
    out->air_loss_pct  = all > 0 ? 100.0f * (float)out->tx_flushed / (float)all : 0.0f;
    return ESP_OK;
}

int le_audio_get_volume(void)
{
    return g_volume.load();
}

void le_audio_set_media_state(bool active, bool playing, const char* title, uint32_t duration_ms)
{
    if (g_init) {
        lehost::btSetMediaState(active, playing, title != nullptr ? title : "", duration_ms);
    }
}

bool le_audio_take_media_command(le_audio_media_cmd_t* out)
{
    lehost::BtMediaCmd c;
    if (out == nullptr || !g_init || !lehost::btTakeMediaCommand(c)) {
        return false;
    }
    *out = (le_audio_media_cmd_t)c;
    return true;
}

}  // extern "C"
