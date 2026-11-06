// tab5_player - minimal LE Audio music player for the M5Stack Tab5 (ESP32-P4 + ESP32-C6 with c6_firmware).
//
// Boot: board (IO expanders power the C6), SD card, saved bonds from NVS, LE Audio host, connection to the saved
// earbuds (the second earbud of the set joins by RSI/SIRK) or, with none saved, the device list to pick and pair
// them; then MP3 files from the SD card stream in stereo. Touch UI on the display, the same stages and counters in
// the serial log (USB Serial/JTAG, every 5 s) and in /sd/tab5_player.log.
//
// SAFETY when testing on a device that runs other firmware: this app NEVER calls
// esp_ota_mark_app_valid_cancel_rollback(). Installed by OTA into the inactive slot it stays "pending verify", and
// with a bootloader built with CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE the next reset (button, crash, watchdog, power
// cycle) boots the previous firmware again. See README.md ("Test without a cable").
#include <cinttypes>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <string>

#include "board_tab5.h"
#include "bond_store.h"
#include "c6_update.h"
#include "esp_app_desc.h"
#include "esp_hosted_cyberdeck.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "le_audio.h"
#include "log_sink.h"
#include "player.h"
#include "ui.h"
#include "sdkconfig.h"

namespace {

constexpr const char* kTag    = "tab5_player";
constexpr const char* kSdPath = "/sd";
#if CONFIG_TAB5_PLAYER_ALLOW_PAIRING
constexpr bool kAllowPairing = true;
#else
constexpr bool kAllowPairing = false;
#endif

const char* otaStateName(esp_ota_img_states_t s)
{
    switch (s) {
        case ESP_OTA_IMG_NEW: return "new";
        case ESP_OTA_IMG_PENDING_VERIFY: return "PENDING_VERIFY (next reset returns to the previous firmware)";
        case ESP_OTA_IMG_VALID: return "valid";
        case ESP_OTA_IMG_INVALID: return "invalid";
        case ESP_OTA_IMG_ABORTED: return "aborted";
        default: return "undefined (flashed over USB)";
    }
}

void logBoot()
{
    const esp_app_desc_t* d     = esp_app_get_description();
    const esp_partition_t* run  = esp_ota_get_running_partition();
    esp_ota_img_states_t state  = ESP_OTA_IMG_UNDEFINED;
    if (run != nullptr) {
        esp_ota_get_state_partition(run, &state);
    }
    ESP_LOGI(kTag, "%s %s (%s %s), running from %s, image state: %s", d->project_name, d->version, d->date, d->time,
             run != nullptr ? run->label : "?", otaStateName(state));
    ESP_LOGI(kTag, "this firmware never confirms itself (no esp_ota_mark_app_valid_cancel_rollback)");
}

const char* linkName(le_audio_link_state_t s)
{
    static const char* const k[] = {"off", "idle", "scanning", "connecting", "connected", "failed"};
    return (unsigned)s < 6 ? k[s] : "?";
}

const char* phaseName(le_audio_stream_phase_t p)
{
    static const char* const k[] = {"off", "configuring", "streaming", "failed"};
    return (unsigned)p < 4 ? k[p] : "?";
}

void logStatus()
{
    le_audio_status_t s;
    le_audio_get_status(&s);
    char title[64];
    player_title(title, sizeof(title));
    // One line for the link and earbuds, one for the stream counters (cumulative since the stream started)
    ESP_LOGI(kTag, "link %s, peer %s (%s), paired %d, L/R: main %s, second %s%s, battery %d %%, C6 %s",
             linkName(s.link), s.peer_addr, s.peer_name, s.paired, s.link == LE_AUDIO_LINK_CONNECTED ? "up" : "down",
             s.second_joined ? "up" : "down", s.set_missing ? " (missing)" : "", s.battery_pct, s.coprocessor_fw);
    ESP_LOGI(kTag,
             "stream %s, members %d%s, LC3 %d Hz/%d B, PD %" PRIu32 " us | sdu tx %" PRIu32 " drop %" PRIu32
             " (no credit %" PRIu32 ", send %" PRIu32 ", deadline %" PRIu32 ") | credits min %d/%d recovered %" PRIu32
             " | txq sent %" PRIu32 " max %d | late %" PRIu32 " slow %" PRIu32 " pcm_short %" PRIu32 " resync %" PRIu32
             " | max late %" PRIu32 " enc %" PRIu32 " send %" PRIu32 " us | %s%s%s",
             phaseName(s.phase), s.members, s.stereo ? " stereo" : "", s.sink_rate_hz, s.sink_octets,
             s.presentation_delay_us, s.tx_sdu, s.tx_dropped, s.drop_no_credit, s.drop_send_fail, s.drop_deadline,
             s.credits_min, s.credits_total, s.credits_recovered, s.txq_sent, s.txq_max, s.late_wakes, s.slow_sends,
             s.pcm_short, s.resyncs, s.max_late_us, s.max_enc_us, s.max_send_us, player_paused() ? "paused: " : "",
             title, s.last_error[0] != '\0' ? (std::string(" | last error: ") + s.last_error).c_str() : "");
}

// Boot stages in the log ("STAGE n/7 <name>: <result>" - grep for STAGE to read a capture) and on the display.
void stage(int n, const char* name, ui_stage_state_t state, const char* fmt, ...) __attribute__((format(printf, 4, 5)));
void stage(int n, const char* name, ui_stage_state_t state, const char* fmt, ...)
{
    char msg[160];
    va_list ap;
    va_start(ap, fmt);
    std::vsnprintf(msg, sizeof(msg), fmt, ap);
    va_end(ap);
    ESP_LOGI(kTag, "STAGE %d/7 %s: %s", n, name, msg);
    ui_stage(n, state, msg);
}

[[noreturn]] void idleForever(const char* why)
{
    ESP_LOGE(kTag, "stopped: %s (status every 10 s; reset to retry)", why);
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(10000));
        ESP_LOGW(kTag, "stopped: %s", why);
    }
}

}  // namespace

extern "C" void app_main(void)
{
    log_sink_usb_init();
    logBoot();
    ESP_ERROR_CHECK(board_tab5_init());
    stage(1, "board", UI_STAGE_OK, "IO expanders set, ESP32-C6 powered");
    ui_start();

    // The esp_hosted host driver is initialised at boot, but without Wi-Fi nothing resets the C6 and waits for its
    // SDIO transport (esp_wifi_init does that in a Wi-Fi app). Nothing may go to the C6 before it is up.
    const int64_t t0    = esp_timer_get_time();
    const esp_err_t tr  = esp_hosted_cyberdeck_connect(30000);
    const int waitedMs  = (int)((esp_timer_get_time() - t0) / 1000);
    if (tr != ESP_OK) {
        stage(2, "transport", UI_STAGE_FAIL, "C6 SDIO transport NOT up after %d ms (%s)", waitedMs, esp_err_to_name(tr));
        idleForever("no SDIO transport to the ESP32-C6");
    }
    stage(2, "transport", UI_STAGE_OK, "C6 SDIO transport up after %d ms", waitedMs);

    // SD card after the C6 slot: card init runs under the SDMMC lock shared with the C6 SDIO slot.
    const bool sdOk = board_tab5_sd_mount(kSdPath) == ESP_OK;
    if (sdOk) {
        log_sink_sd_start(kSdPath);
    }
    stage(3, "sd", sdOk ? UI_STAGE_OK : UI_STAGE_FAIL, "%s", sdOk ? board_tab5_sd_info() : "NOT mounted - nothing to play");

    le_audio_config_t cfg = LE_AUDIO_CONFIG_DEFAULT();
    int identity          = 1;
    const bool haveBonds  = bond_store_load(&cfg.kv, &identity);
    cfg.identity          = (le_audio_identity_t)identity;
    cfg.allow_pairing     = kAllowPairing;
    cfg.device_name       = "Tab5 LE Audio player";
    stage(4, "bonds", haveBonds || kAllowPairing ? UI_STAGE_OK : UI_STAGE_FAIL, "%d saved (identity mode %d, pairing %s)", haveBonds ? bond_store_count() : 0,
          identity, kAllowPairing ? "allowed" : "off");

    esp_err_t err = ESP_FAIL;
    for (int i = 0; i < 10; i++) {
        err = le_audio_init(&cfg);
        if (err == ESP_OK || err == ESP_ERR_NOT_SUPPORTED) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    le_audio_status_t st;
    le_audio_get_status(&st);
    const char* carried = c6_update_embedded_version();
    if (err != ESP_OK) {
        if (err == ESP_ERR_NOT_SUPPORTED) {
            // e.g. the factory C6 firmware: the UI stays up, "C6 firmware" installs the one this player carries
            stage(5, "host", UI_STAGE_FAIL, "C6 firmware %s has no LE Audio - tap \"C6 firmware\" to install %s",
                  st.coprocessor_fw[0] != '\0' ? st.coprocessor_fw : "?", carried);
            idleForever("C6 firmware without CIS - install it with the \"C6 firmware\" button");
        }
        stage(5, "host", UI_STAGE_FAIL, "LE Audio host NOT ready (%s), C6 firmware %s", esp_err_to_name(err),
              st.coprocessor_fw[0] != '\0' ? st.coprocessor_fw : "?");
        idleForever("LE Audio host did not start");
    }
    if (std::strcmp(st.coprocessor_fw, carried) != 0) {
        stage(5, "host", UI_STAGE_OK, "LE Audio host ready, C6 firmware %s (this player carries %s: \"C6 firmware\")",
              st.coprocessor_fw, carried);
    } else {
        stage(5, "host", UI_STAGE_OK, "LE Audio host ready, C6 firmware %s", st.coprocessor_fw);
    }
    le_audio_set_volume(CONFIG_TAB5_PLAYER_VOLUME);

    if (!haveBonds) {
#if CONFIG_TAB5_PLAYER_ALLOW_PAIRING
        // Never pair the first device heard (it may be someone else's): the user picks from the list
        stage(6, "connect", UI_STAGE_PENDING, "no saved earbuds - pick yours from the list (Find earbuds)");
        ui_find_earbuds();
#else
        stage(6, "connect", UI_STAGE_FAIL, "no saved earbuds and pairing off (CONFIG_TAB5_PLAYER_ALLOW_PAIRING) - nothing to connect");
#endif
    } else {
        // First attempt right away; afterwards the background manager keeps reconnecting and joins the second earbud.
        const esp_err_t c = le_audio_connect_bonded(5);
        stage(6, "connect", c == ESP_OK ? UI_STAGE_OK : UI_STAGE_PENDING, "saved earbuds: %s%s", esp_err_to_name(c),
              c == ESP_OK ? "" : " (background manager keeps trying)");
    }

    if (sdOk) {
        player_start(CONFIG_TAB5_PLAYER_MUSIC_DIR);
    }
    le_audio_stream_phase_t lastPhase = LE_AUDIO_STREAM_OFF;
    int lastMembers                   = 0;
    le_audio_link_state_t lastLink    = LE_AUDIO_LINK_OFF;
    for (int tick = 0;; tick++) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        le_audio_get_status(&st);
        if (st.link != lastLink) {
            // The background manager (or the Connect button) connects later: keep stage 6 current
            if (st.link == LE_AUDIO_LINK_CONNECTED) {
                stage(6, "connect", UI_STAGE_OK, "connected %s (%s)", st.peer_addr, st.peer_name);
            } else if (lastLink == LE_AUDIO_LINK_CONNECTED) {
                stage(6, "connect", UI_STAGE_PENDING, "earbuds disconnected - reconnecting");
            }
            lastLink = st.link;
        }
        if (st.phase != lastPhase || st.members != lastMembers) {
            stage(7, "stream",
                  st.phase == LE_AUDIO_STREAM_STREAMING ? UI_STAGE_OK
                  : st.phase == LE_AUDIO_STREAM_FAILED  ? UI_STAGE_FAIL
                                                        : UI_STAGE_PENDING,
                  "%s, earbuds %d%s%s%s", phaseName(st.phase), st.members, st.stereo ? " (stereo L/R)" : "",
                  st.last_error[0] != '\0' && st.phase == LE_AUDIO_STREAM_FAILED ? " - " : "",
                  st.phase == LE_AUDIO_STREAM_FAILED ? st.last_error : "");
            lastPhase   = st.phase;
            lastMembers = st.members;
        }
        if (tick % 5 == 4) {
            logStatus();
        }
    }
}
