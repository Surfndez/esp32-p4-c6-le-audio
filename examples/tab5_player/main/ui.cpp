// Minimal touch UI of the example on the Tab5 display (1280 x 720 landscape): boot stages, both earbuds, player
// and stream counters, a row of buttons. Display and touch come from the Tab5 BSP (components/m5stack_tab5) with the
// LVGL port set up like the original firmware (strip buffers in PSRAM, PPA rotation to landscape).
//
// Threads: LVGL objects are touched only in the LVGL task (an lv_timer applies a snapshot; button callbacks run
// there too). The snapshot is filled by the "ui-poll" task (status every second, C6 CIS counters every 2 s) and by
// ui_stage() from app_main. Blocking actions (connect, scan, pair) go to the "ui-cmd" task. "Find earbuds" scans and
// shows the devices found in an overlay list; tapping one connects it (saved: its key; new: pairing, if allowed).
#include "ui.h"

#include <cstdio>
#include <cstring>
#include <mutex>
#include <string>

#include "bsp/display.h"
#include "bsp/m5stack_tab5.h"
#include "esp_log.h"
#include "esp_lvgl_port.h"
#include "esp_ota_ops.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "c6_update.h"
#include "esp_system.h"
#include "le_audio.h"
#include "lvgl.h"
#include "player.h"
#include "sdkconfig.h"

namespace {

constexpr const char* kTag = "ui";
constexpr int kStages      = 7;
const char* const kStageNames[kStages] = {"Board", "C6 transport", "SD card", "Bonds", "LE Audio host", "Connect",
                                          "Stream"};

enum class StageState : uint8_t { Pending, Ok, Fail };

struct Snapshot {
    StageState stage[kStages] = {};
    char stageText[kStages][64] = {};
    le_audio_status_t st      = {};
    bool cisValid[2]          = {};
    le_audio_cis_stats_t cis[2] = {};
    char title[64]            = {};
    uint32_t posMs            = 0;
    bool paused               = false;
    int volume                = 0;
    char action[128]          = {};  // last button action result
};

std::mutex g_lock;
Snapshot g_snap;  // under g_lock

enum class Cmd : uint8_t { Connect, Scan, ConnectFound, InstallC6 };
struct CmdMsg {
    Cmd cmd;
    int index;     // ConnectFound: entry of g_found
    uint32_t gen;  // ConnectFound: g_found.gen of the list the user tapped (a newer scan invalidates the index)
};
QueueHandle_t g_cmds = nullptr;

#if CONFIG_TAB5_PLAYER_ALLOW_PAIRING
constexpr bool kPairing = true;
#else
constexpr bool kPairing = false;
#endif
#if CONFIG_TAB5_PLAYER_SAVE_BONDS && !CONFIG_TAB5_PLAYER_START_FRESH
constexpr bool kSaveBonds = true;
#else
constexpr bool kSaveBonds = false;
#endif

// Devices of the last scan, under g_lock. gen changes with every scan start and result (the LVGL task rebuilds the
// list then).
constexpr int kMaxFound = 12;
constexpr int kScanS    = 6;
struct Found {
    le_audio_device_t dev[kMaxFound] = {};
    int n         = 0;
    uint32_t gen  = 0;
    bool scanning = false;
    bool visible  = false;
} g_found;

struct Widgets {
    lv_obj_t* stage[kStages];
    lv_obj_t* earbud[2];
    lv_obj_t* player;
    lv_obj_t* counters;
    lv_obj_t* action;
    lv_obj_t* overlay;  // device list over the main screen
    lv_obj_t* listTitle;
    lv_obj_t* list;
    uint32_t listGen = UINT32_MAX;
    bool listVisible = false;
} w;

bool post(Cmd c, int index = 0, uint32_t gen = 0)
{
    const CmdMsg m = {c, index, gen};
    return g_cmds != nullptr && xQueueSend(g_cmds, &m, 0) == pdTRUE;
}

void setActionLocked(const char* text)  // g_lock held
{
    std::snprintf(g_snap.action, sizeof(g_snap.action), "%s", text);
}

void setAction(const char* text)
{
    std::lock_guard<std::mutex> l(g_lock);
    setActionLocked(text);
}

const char* sideName(uint32_t loc, int member)
{
    if (loc & 0x1) {
        return "Left";
    }
    if (loc & 0x2) {
        return "Right";
    }
    return member == 0 ? "Earbud 1 (main)" : "Earbud 2 (set)";
}

const char* earbudState(const le_audio_status_t& s, int i)
{
    if (s.member_streaming[i]) {
        return "streaming";
    }
    if (i == 0) {
        switch (s.link) {
            case LE_AUDIO_LINK_CONNECTED: return "connected";
            case LE_AUDIO_LINK_CONNECTING: return "connecting";
            case LE_AUDIO_LINK_SCANNING: return "scanning";
            default: return "down";
        }
    }
    if (s.second_joined) {
        return "connected";
    }
    return s.set_missing ? "down (joining)" : "down";
}

void onFound(lv_event_t* e)
{
    const int i = (int)(intptr_t)lv_event_get_user_data(e);
    {
        std::lock_guard<std::mutex> l(g_lock);
        g_found.visible = false;
    }
    if (!post(Cmd::ConnectFound, i, w.listGen)) {
        setAction("Busy - try again");
    }
}

// Shows / hides the device list and rebuilds it after a scan start or result; LVGL task only.
void applyFound()
{
    Found f;
    {
        std::lock_guard<std::mutex> l(g_lock);
        f = g_found;
    }
    if (f.visible != w.listVisible) {
        if (f.visible) {
            lv_obj_remove_flag(w.overlay, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_add_flag(w.overlay, LV_OBJ_FLAG_HIDDEN);
        }
        w.listVisible = f.visible;
    }
    if (f.gen == w.listGen) {
        return;
    }
    w.listGen = f.gen;
    lv_obj_clean(w.list);
    if (f.scanning) {
        lv_label_set_text_fmt(w.listTitle, "Scanning for %d s... (open the earbuds case; new earbuds: pairing mode)",
                              kScanS);
        return;
    }
    lv_label_set_text_fmt(w.listTitle, "Found %d device(s) - tap yours%s", f.n,
                          kPairing ? "" : " (pairing off: saved earbuds only)");
    char b[96];
    for (int i = 0; i < f.n; i++) {
        const le_audio_device_t& d = f.dev[i];
        std::snprintf(b, sizeof(b), "%s   %d dBm%s%s", d.name[0] != '\0' ? d.name : d.addr, d.rssi,
                      d.le_audio ? "   LE Audio" : "   (no LE Audio seen)", d.bonded ? "   saved" : "");
        lv_obj_t* btn = lv_list_add_button(w.list, d.bonded ? LV_SYMBOL_OK : LV_SYMBOL_AUDIO, b);
        lv_obj_set_height(btn, 64);
        lv_obj_set_style_text_font(btn, &lv_font_montserrat_20, 0);
        if (!d.le_audio) {
            lv_obj_set_style_text_color(btn, lv_palette_main(LV_PALETTE_GREY), 0);
        }
        // New devices pair only with pairing on and only if they advertise LE Audio: the bond list holds 4 entries,
        // a stray pairing could push the earbuds out
        if (!d.bonded && (!kPairing || !d.le_audio)) {
            lv_obj_add_state(btn, LV_STATE_DISABLED);
        } else {
            lv_obj_add_event_cb(btn, onFound, LV_EVENT_CLICKED, (void*)(intptr_t)i);
        }
    }
    if (f.n == 0) {
        lv_list_add_text(w.list, "Nothing found - open the earbuds case (new earbuds: pairing mode) and Rescan");
    }
}

void applySnapshot(lv_timer_t*)
{
    Snapshot s;
    {
        std::lock_guard<std::mutex> l(g_lock);
        s = g_snap;
    }
    char b[256];
    for (int i = 0; i < kStages; i++) {
        const char* mark = s.stage[i] == StageState::Ok ? LV_SYMBOL_OK : s.stage[i] == StageState::Fail ? LV_SYMBOL_CLOSE : "...";
        std::snprintf(b, sizeof(b), "%s  %d %s: %s", mark, i + 1, kStageNames[i], s.stageText[i]);
        lv_label_set_text(w.stage[i], b);
        lv_obj_set_style_text_color(w.stage[i],
                                    s.stage[i] == StageState::Ok     ? lv_palette_main(LV_PALETTE_GREEN)
                                    : s.stage[i] == StageState::Fail ? lv_palette_main(LV_PALETTE_RED)
                                                                     : lv_palette_main(LV_PALETTE_GREY),
                                    0);
    }
    for (int i = 0; i < 2; i++) {
        char extra[64] = "";
        if (i == 0) {
            std::snprintf(extra, sizeof(extra), "  battery %s%d%%  RSSI %d dBm", s.st.battery_pct < 0 ? "? " : "",
                          s.st.battery_pct < 0 ? 0 : s.st.battery_pct, s.st.rssi);
        }
        char loss[48] = "";
        if (s.cisValid[i]) {
            std::snprintf(loss, sizeof(loss), "  air loss %.2f %%", (double)s.cis[i].air_loss_pct);
        }
        std::snprintf(b, sizeof(b), "%s: %s%s%s", sideName(s.st.member_location[i], i), earbudState(s.st, i), extra,
                      loss);
        lv_label_set_text(w.earbud[i], b);
    }
    const uint32_t sec = s.posMs / 1000;
    std::snprintf(b, sizeof(b), "%s %s   %02u:%02u   volume %d", s.paused ? LV_SYMBOL_PAUSE : LV_SYMBOL_PLAY,
                  s.title[0] != '\0' ? s.title : "(no track)", (unsigned)(sec / 60), (unsigned)(sec % 60), s.volume);
    lv_label_set_text(w.player, b);
    std::snprintf(b, sizeof(b),
                  "Stream: %s, %d earbud(s)%s, LC3 %d Hz / %d B\nSDU sent %lu, host drops %lu (no credit %lu, "
                  "deadline %lu), credits min %d/%d%s%s",
                  s.st.phase == LE_AUDIO_STREAM_STREAMING     ? "streaming"
                  : s.st.phase == LE_AUDIO_STREAM_CONFIGURING ? "configuring"
                  : s.st.phase == LE_AUDIO_STREAM_FAILED      ? "failed"
                                                              : "off",
                  s.st.members, s.st.stereo ? " stereo" : "", s.st.sink_rate_hz, s.st.sink_octets,
                  (unsigned long)s.st.tx_sdu, (unsigned long)s.st.tx_dropped, (unsigned long)s.st.drop_no_credit,
                  (unsigned long)s.st.drop_deadline, s.st.credits_min, s.st.credits_total,
                  s.st.last_error[0] != '\0' ? "\nLast error: " : "", s.st.last_error);
    lv_label_set_text(w.counters, b);
    lv_label_set_text(w.action, s.action);
    applyFound();
}

void pollTask(void*)
{
    for (int tick = 0;; tick++) {
        le_audio_status_t st;
        le_audio_get_status(&st);
        Snapshot add;
        const bool cisTick = tick % 2 == 0 && st.phase == LE_AUDIO_STREAM_STREAMING;
        for (int i = 0; cisTick && i < 2; i++) {
            add.cisValid[i] = le_audio_get_cis_stats(i, &add.cis[i]) == ESP_OK;
        }
        char title[64];
        player_title(title, sizeof(title));
        {
            std::lock_guard<std::mutex> l(g_lock);
            g_snap.st = st;
            if (cisTick) {
                for (int i = 0; i < 2; i++) {
                    g_snap.cisValid[i] = add.cisValid[i];
                    g_snap.cis[i]      = add.cis[i];
                }
            } else if (st.phase != LE_AUDIO_STREAM_STREAMING) {
                g_snap.cisValid[0] = g_snap.cisValid[1] = false;
            }
            std::snprintf(g_snap.title, sizeof(g_snap.title), "%s", title);
            g_snap.posMs  = player_position_ms();
            g_snap.paused = player_paused();
            g_snap.volume = le_audio_get_volume();
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void scanToList()
{
    {
        std::lock_guard<std::mutex> l(g_lock);
        g_found.scanning = true;
        g_found.visible  = true;
        g_found.n        = 0;
        g_found.gen++;
    }
    le_audio_status_t st;
    le_audio_get_status(&st);
    setAction(st.phase == LE_AUDIO_STREAM_STREAMING ? "Scanning (audio keeps playing, may stutter briefly)..."
                                                    : "Scanning...");
    le_audio_device_t found[kMaxFound];
    const int n = le_audio_scan(found, kMaxFound, kScanS);
    {
        std::lock_guard<std::mutex> l(g_lock);
        for (int i = 0; i < n; i++) {
            g_found.dev[i] = found[i];
        }
        g_found.n        = n;
        g_found.scanning = false;
        g_found.gen++;
    }
    char b[64];
    std::snprintf(b, sizeof(b), "Scan: %d device(s) found", n);
    ESP_LOGI(kTag, "%s", b);
    setAction(b);
}

void connectFound(int index, uint32_t gen)
{
    le_audio_device_t d;
    {
        std::lock_guard<std::mutex> l(g_lock);
        if (gen != g_found.gen || index < 0 || index >= g_found.n) {
            g_found.visible = true;  // show the current list again
            setActionLocked("The list changed (new scan) - tap your earbuds again");
            return;
        }
        d = g_found.dev[index];
    }
    const char* name = d.name[0] != '\0' ? d.name : d.addr;
    char b[112];
    std::snprintf(b, sizeof(b), "%s %s...", d.bonded ? "Connecting" : "Pairing (confirm on the earbuds if asked)", name);
    setAction(b);
    const esp_err_t e = le_audio_connect_device(&d);
    std::snprintf(b, sizeof(b), "%s %s: %s%s", d.bonded ? "Connect" : "Pair", name, esp_err_to_name(e),
                  e == ESP_OK && !d.bonded && !kSaveBonds ? " (key kept in RAM until reset)" : "");
    ESP_LOGI(kTag, "%s", b);
    setAction(b);
}

void installProgress(int pct)
{
    char b[96];
    std::snprintf(b, sizeof(b), "Installing C6 firmware %s: %d %% (do not power off)", c6_update_embedded_version(), pct);
    setAction(b);
}

// The C6 restarts into the new image by itself; the SDIO link does not survive that, so the P4 restarts as well
// (like the original firmware after a C6 update). The C6's fw_guard keeps the new image on trial and goes back to
// the previous one by itself if it does not work.
void installC6()
{
    le_audio_stop();  // no stream while the C6 slot is erased and written
    installProgress(0);
    const esp_err_t e = c6_update_install(installProgress);
    char b[112];
    if (e == ESP_OK) {
        std::snprintf(b, sizeof(b), "C6 firmware %s installed - the Tab5 restarts in 8 s", c6_update_embedded_version());
        setAction(b);
        ESP_LOGW(kTag, "%s", b);
        vTaskDelay(pdMS_TO_TICKS(8000));  // the C6 restarts ~5 s after the verification
        esp_restart();
    }
    std::snprintf(b, sizeof(b), "C6 update not done (%s)%s - the C6 keeps its firmware", esp_err_to_name(e),
                  e == ESP_ERR_INVALID_STATE ? ": its new firmware is still being verified, try in a minute" : "");
    setAction(b);
    ESP_LOGW(kTag, "%s", b);
}

void cmdTask(void*)
{
    CmdMsg m;
    char b[64];
    for (;;) {
        if (xQueueReceive(g_cmds, &m, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        if (m.cmd == Cmd::Scan) {
            scanToList();
            continue;
        }
        if (m.cmd == Cmd::ConnectFound) {
            connectFound(m.index, m.gen);
            continue;
        }
        if (m.cmd == Cmd::InstallC6) {
            installC6();
            continue;
        }
        le_audio_status_t st;
        le_audio_get_status(&st);
        if (st.link == LE_AUDIO_LINK_CONNECTED && st.second_joined) {
            setAction("Both earbuds are connected");
            continue;
        }
        if (st.link == LE_AUDIO_LINK_CONNECTED) {
            setAction(st.phase == LE_AUDIO_STREAM_STREAMING
                          ? "Looking for the second earbud (stream pauses briefly if found)..."
                          : "Joining the second earbud...");
            const esp_err_t e = le_audio_join_set();
            std::snprintf(b, sizeof(b), "Join second earbud: %s", esp_err_to_name(e));
        } else {
            setAction("Scanning for saved earbuds...");
            const esp_err_t e = le_audio_connect_bonded(5);
            std::snprintf(b, sizeof(b), "Connect saved earbuds: %s", esp_err_to_name(e));
        }
        ESP_LOGI(kTag, "%s", b);
        setAction(b);
    }
}

void showC6Box();  // below, with its dialog callbacks

void onButton(lv_event_t* e)
{
    switch ((intptr_t)lv_event_get_user_data(e)) {
        case 0:
            if (!post(Cmd::Connect)) {
                setAction("Busy - try again");
            }
            break;
        case 1: player_toggle_pause(); break;
        case 2: player_next(); break;
        case 3: le_audio_set_volume(le_audio_get_volume() - 10); break;
        case 4: le_audio_set_volume(le_audio_get_volume() + 10); break;
        case 5:
            if (!post(Cmd::Scan)) {
                setAction("Busy - try again");
            }
            break;
        case 6: showC6Box(); break;
        default: break;
    }
}

void onC6Install(lv_event_t* e)
{
    lv_msgbox_close(static_cast<lv_obj_t*>(lv_event_get_user_data(e)));
    if (!post(Cmd::InstallC6)) {
        setAction("Busy - try again");
    }
}

void onC6Close(lv_event_t* e)
{
    lv_msgbox_close(static_cast<lv_obj_t*>(lv_event_get_user_data(e)));
}

// "C6 firmware": what the C6 runs now and what this player carries; installing is always the user's tap (someone
// may run another C6 firmware on purpose). LVGL task.
void showC6Box()
{
    le_audio_status_t st;
    le_audio_get_status(&st);
    const char* embedded = c6_update_embedded_version();
    char b[256];
    std::snprintf(b, sizeof(b),
                  "ESP32-C6 now: %s\nIn this player: %s\n\nInstalling takes about 30 s, then the Tab5 restarts. "
                  "If the new firmware does not work, the C6 goes back to its previous firmware by itself.",
                  st.coprocessor_fw[0] != '\0' ? st.coprocessor_fw : "unknown (no LE Audio firmware?)",
                  embedded[0] != '\0' ? embedded : "none (build problem)");
    lv_obj_t* box = lv_msgbox_create(nullptr);
    lv_msgbox_add_title(box, "C6 firmware");
    lv_obj_t* text = lv_msgbox_add_text(box, b);
    lv_obj_set_style_text_font(text, &lv_font_montserrat_20, 0);
    lv_obj_set_width(box, LV_PCT(60));
    if (embedded[0] != '\0') {
        lv_obj_t* ok = lv_msgbox_add_footer_button(box, "Install");
        lv_obj_add_event_cb(ok, onC6Install, LV_EVENT_CLICKED, box);
    }
    lv_obj_t* close = lv_msgbox_add_footer_button(box, "Close");
    lv_obj_add_event_cb(close, onC6Close, LV_EVENT_CLICKED, box);
}

// Buttons of the device list: 0 = Rescan, 1 = Close
void onListButton(lv_event_t* e)
{
    if ((intptr_t)lv_event_get_user_data(e) == 0) {
        if (!post(Cmd::Scan)) {
            setAction("Busy - try again");
        }
        return;
    }
    std::lock_guard<std::mutex> l(g_lock);
    g_found.visible = false;
}

lv_obj_t* label(lv_obj_t* parent, const lv_font_t* font)
{
    lv_obj_t* l = lv_label_create(parent);
    lv_obj_set_style_text_font(l, font, 0);
    lv_obj_set_width(l, LV_PCT(100));
    lv_label_set_long_mode(l, LV_LABEL_LONG_WRAP);
    lv_label_set_text(l, "");
    return l;
}

lv_obj_t* panel(lv_obj_t* parent, int widthPct)
{
    lv_obj_t* p = lv_obj_create(parent);
    lv_obj_set_size(p, LV_PCT(widthPct), LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(p, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(p, 6, 0);
    return p;
}

// Device list of "Find earbuds" on the top layer, hidden until a scan starts
void buildOverlay()
{
    w.overlay = lv_obj_create(lv_layer_top());
    lv_obj_set_size(w.overlay, LV_PCT(80), LV_PCT(86));
    lv_obj_center(w.overlay);
    lv_obj_set_style_bg_color(w.overlay, lv_color_hex(0x1C232B), 0);
    lv_obj_set_flex_flow(w.overlay, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(w.overlay, 10, 0);
    w.listTitle = label(w.overlay, &lv_font_montserrat_20);
    w.list      = lv_list_create(w.overlay);
    lv_obj_set_width(w.list, LV_PCT(100));
    lv_obj_set_flex_grow(w.list, 1);
    lv_obj_t* row = lv_obj_create(w.overlay);
    lv_obj_set_size(row, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_column(row, 12, 0);
    lv_obj_set_style_bg_opa(row, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(row, 0, 0);
    const char* names[] = {"Rescan", "Close"};
    for (int i = 0; i < 2; i++) {
        lv_obj_t* btn = lv_button_create(row);
        lv_obj_set_height(btn, 64);
        lv_obj_set_style_pad_hor(btn, 32, 0);
        lv_obj_add_event_cb(btn, onListButton, LV_EVENT_CLICKED, (void*)(intptr_t)i);
        lv_obj_t* t = lv_label_create(btn);
        lv_obj_set_style_text_font(t, &lv_font_montserrat_20, 0);
        lv_label_set_text(t, names[i]);
        lv_obj_center(t);
    }
    lv_obj_add_flag(w.overlay, LV_OBJ_FLAG_HIDDEN);
}

void build(lv_display_t* disp)
{
    lv_obj_t* scr = lv_screen_active();
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x101418), 0);
    lv_obj_set_style_text_color(scr, lv_color_hex(0xE8E8E8), 0);
    lv_obj_set_flex_flow(scr, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_all(scr, 12, 0);
    lv_obj_set_style_pad_row(scr, 8, 0);

    lv_obj_t* h = label(scr, &lv_font_montserrat_28);
    lv_label_set_text(h, "tab5_player - LE Audio demo");
    const esp_partition_t* run = esp_ota_get_running_partition();
    esp_ota_img_states_t state = ESP_OTA_IMG_UNDEFINED;
    if (run != nullptr) {
        esp_ota_get_state_partition(run, &state);
    }
    lv_obj_t* fw = label(scr, &lv_font_montserrat_20);
    const bool test = state == ESP_OTA_IMG_PENDING_VERIFY;
    lv_label_set_text(fw, test ? "Firmware: TEST (not confirmed - reset returns to previous firmware)"
                               : "Firmware: flashed over USB (no previous firmware to return to)");
    lv_obj_set_style_text_color(fw, test ? lv_palette_main(LV_PALETTE_ORANGE) : lv_palette_main(LV_PALETTE_GREY), 0);

    lv_obj_t* row = lv_obj_create(scr);
    lv_obj_set_size(row, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_column(row, 10, 0);
    lv_obj_set_style_bg_opa(row, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(row, 0, 0);
    lv_obj_set_style_pad_all(row, 0, 0);

    lv_obj_t* left = panel(row, 44);
    for (int i = 0; i < kStages; i++) {
        w.stage[i] = label(left, &lv_font_montserrat_20);
    }
    lv_obj_t* right = panel(row, 54);
    w.earbud[0] = label(right, &lv_font_montserrat_20);
    w.earbud[1] = label(right, &lv_font_montserrat_20);
    w.player    = label(right, &lv_font_montserrat_20);
    w.counters  = label(right, &lv_font_montserrat_14);

    lv_obj_t* buttons = lv_obj_create(scr);
    lv_obj_set_size(buttons, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(buttons, LV_FLEX_FLOW_ROW_WRAP);
    lv_obj_set_style_pad_column(buttons, 12, 0);
    lv_obj_set_style_bg_opa(buttons, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(buttons, 0, 0);
    const char* names[] = {"Connect / Rescan", "Play / Pause", "Next", "Vol -", "Vol +", "Find earbuds", "C6 firmware"};
    for (int i = 0; i < 7; i++) {
        lv_obj_t* btn = lv_button_create(buttons);
        lv_obj_set_height(btn, 72);
        lv_obj_set_style_pad_hor(btn, 24, 0);
        lv_obj_add_event_cb(btn, onButton, LV_EVENT_CLICKED, (void*)(intptr_t)i);
        lv_obj_t* t = lv_label_create(btn);
        lv_obj_set_style_text_font(t, &lv_font_montserrat_20, 0);
        lv_label_set_text(t, names[i]);
        lv_obj_center(t);
    }
    w.action = label(scr, &lv_font_montserrat_20);
    buildOverlay();
    lv_timer_create(applySnapshot, 500, nullptr);
    (void)disp;
}

}  // namespace

void ui_start(void)
{
    // Like the original firmware: touch controller reset, then the BSP display + LVGL port with 48-line strip
    // buffers in PSRAM and software (PPA) rotation; the panel is portrait, the UI landscape.
    bsp_reset_tp();
    constexpr int32_t kStripLines = 48;
    bsp_display_cfg_t cfg         = {.lvgl_port_cfg = ESP_LVGL_PORT_INIT_CONFIG(),
                                     .buffer_size   = BSP_LCD_V_RES * kStripLines,
                                     .double_buffer = true,
                                     .flags         = {
                                         .buff_dma    = true,
                                         .buff_spiram = true,
                                         .sw_rotate   = true,
                                     }};
    cfg.lvgl_port_cfg.task_stack    = 12 * 1024;  // internal RAM; UI callbacks format text on it
    cfg.lvgl_port_cfg.task_affinity = 0;  // core 1 runs the LE Audio stream thread
    lv_display_t* disp              = bsp_display_start_with_config(&cfg);
    if (disp == nullptr) {
        ESP_LOGE(kTag, "display / touch did not start - serial log only");
        return;
    }
    bsp_display_lock(0);
    lv_display_set_rotation(disp, LV_DISPLAY_ROTATION_90);
    // PPA rotation shifts rows of areas with odd width / height, and the DPI frame buffer needs blocks that start on a
    // column divisible by 4 - every refreshed area gets even width and a height in multiples of 4 (original firmware)
    lv_display_add_event_cb(
        disp,
        [](lv_event_t* e) {
            auto* a = static_cast<lv_area_t*>(lv_event_get_param(e));
            a->x1 &= ~1;
            a->y1 &= ~3;
            a->x2 |= 1;
            a->y2 |= 3;
        },
        LV_EVENT_INVALIDATE_AREA, nullptr);
    g_cmds = xQueueCreate(2, sizeof(CmdMsg));
    build(disp);
    bsp_display_unlock();
    bsp_display_brightness_set(80);  // backlight on for the whole run - the demo never turns the screen off
    if (g_cmds == nullptr || xTaskCreate(cmdTask, "ui-cmd", 8192, nullptr, 4, nullptr) != pdPASS ||
        xTaskCreate(pollTask, "ui-poll", 6144, nullptr, 3, nullptr) != pdPASS) {
        ESP_LOGE(kTag, "UI tasks not started - buttons and live values do not work");
    }
    ESP_LOGI(kTag, "display and touch ready");
}

void ui_stage(int n, ui_stage_state_t state, const char* detail)
{
    if (n < 1 || n > kStages) {
        return;
    }
    std::lock_guard<std::mutex> l(g_lock);
    g_snap.stage[n - 1] = state == UI_STAGE_OK     ? StageState::Ok
                          : state == UI_STAGE_FAIL ? StageState::Fail
                                                   : StageState::Pending;
    std::snprintf(g_snap.stageText[n - 1], sizeof(g_snap.stageText[0]), "%s", detail != nullptr ? detail : "");
}

void ui_find_earbuds(void)
{
    if (!post(Cmd::Scan)) {
        ESP_LOGW(kTag, "find earbuds: UI command task busy or not started");
    }
}
