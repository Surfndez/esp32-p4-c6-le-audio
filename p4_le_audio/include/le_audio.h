#pragma once
/*
 * LE Audio host for ESP32-P4 + ESP32-C6 (CIS link layer from this project on the C6, HCI over SDIO / esp_hosted).
 *
 * Role: BAP Unicast Client + CSIP Set Coordinator. One CIG with up to two CIS (left / right earbud), LC3 48 kHz /
 * 10 ms (the earbuds' PACS decide the exact configuration), ISO data over HCI with credit flow control
 * (IsoCredits: lost Number-Of-Completed-Packets recovered after 500 ms) and a deadline queue for SDUs waiting for a
 * credit (IsoTxQueue, 25 ms).
 *
 * Threading: every call except le_audio_write_pcm(), le_audio_set_volume(), le_audio_get_status() and
 * le_audio_take_media_command() may block for seconds (HCI, ATT, SMP); call them from a task, not from an ISR or a
 * UI thread. Internal threads keep their stacks in PSRAM.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Key/value store for bonds. Keys: "bt.bond" (list of bonds, hex: address 6 B + LTK 16 B [+ IRK 16 B], ','
 * separated) and "bt.irk" (our IRK, 32 hex chars). Both are secrets. The callbacks are called from threads whose
 * stacks are in PSRAM: they must NOT access flash (NVS) directly - load the values into RAM before
 * le_audio_init() and serve them from there. set == NULL makes the store read-only: new bonds (pairing) stay in RAM
 * until reset and no stored bond is ever erased. */
typedef struct {
    bool (*get)(const char *key, char *out, size_t cap, void *ctx); /* NUL-terminated value; false = missing */
    bool (*set)(const char *key, const char *value, void *ctx);     /* NULL = read-only store */
    void *ctx;
} le_audio_kv_t;

/* Address we connect and pair from. Must be the same as when the earbuds were paired, or they won't recognise us. */
typedef enum {
    LE_AUDIO_IDENTITY_STATIC_RANDOM_IRK = 0, /* static random address + our IRK (old variant) */
    LE_AUDIO_IDENTITY_PUBLIC            = 1, /* controller public address, no IRK (default) */
    LE_AUDIO_IDENTITY_PUBLIC_IRK        = 2, /* public address + our IRK */
} le_audio_identity_t;

typedef struct {
    le_audio_kv_t kv;
    le_audio_identity_t identity;
    bool allow_pairing; /* false: only saved bonds are used; unknown earbuds are never paired, keys never erased */
    bool auto_connect;  /* background task: reconnect saved earbuds, join the second earbud of the set */
    bool auto_start;    /* background task: start the stream when paired earbuds with ASCS are connected */
    const char *device_name; /* GAP name of our local GATT server, NULL = "ESP32-P4 LE Audio" */
} le_audio_config_t;

#define LE_AUDIO_CONFIG_DEFAULT()                                                                                      \
    {                                                                                                                  \
        .kv = {NULL, NULL, NULL}, .identity = LE_AUDIO_IDENTITY_PUBLIC, .allow_pairing = false,                       \
        .auto_connect = true, .auto_start = true, .device_name = NULL,                                                 \
    }

typedef enum {
    LE_AUDIO_LINK_OFF = 0,
    LE_AUDIO_LINK_IDLE,
    LE_AUDIO_LINK_SCANNING,
    LE_AUDIO_LINK_CONNECTING,
    LE_AUDIO_LINK_CONNECTED,
    LE_AUDIO_LINK_FAILED,
} le_audio_link_state_t;

typedef enum {
    LE_AUDIO_STREAM_OFF = 0,
    LE_AUDIO_STREAM_CONFIGURING,
    LE_AUDIO_STREAM_STREAMING,
    LE_AUDIO_STREAM_FAILED,
} le_audio_stream_phase_t;

/* Snapshot of the link and stream. Counters are cumulative since the stream started. */
typedef struct {
    le_audio_link_state_t link;
    bool paired;         /* main link encrypted */
    bool second_joined;  /* second earbud of the set connected and encrypted */
    bool set_missing;    /* set of >= 2 earbuds, second one missing */
    int battery_pct;     /* -1 = unknown */
    char coprocessor_fw[16];
    char peer_addr[20];
    char peer_name[40];

    le_audio_stream_phase_t phase;
    int members; /* earbuds in the stream: 1 = mono, 2 = set */
    bool stereo; /* left / right split */
    int sink_rate_hz, sink_octets;
    uint32_t presentation_delay_us;
    uint32_t tx_sdu, tx_dropped;
    uint32_t drop_no_credit, drop_send_fail, drop_deadline;
    uint32_t txq_sent;
    int txq_max;
    int credits_min, credits_total;
    uint32_t credits_recovered;
    uint32_t late_wakes, slow_sends, pcm_short, resyncs, skipped_frames;
    uint32_t max_late_us, max_enc_us, max_send_us, max_stall_us;
    uint32_t stream_lost; /* restarts that did not bring the stream back (since le_audio_init) */
    int rssi;             /* main earbud, dBm at connection time (0 = unknown) */
    /* Stream members (index 0 = main earbud, 1 = second earbud of the set): Audio Location of the Sink ASE
     * (0x1 = left, 0x2 = right, 0 = unknown until the stream is configured) and whether its CIS carries audio. */
    uint32_t member_location[2];
    bool member_streaming[2];
    char last_error[96];
} le_audio_status_t;

typedef struct {
    char addr[20];
    uint8_t addr_type; /* 0 public, 1 random */
    char name[40];
    int rssi;
    bool le_audio; /* advertises LE Audio services */
    bool bonded;   /* saved bond (identity address or RPA resolved with its IRK) */
} le_audio_device_t;

typedef enum {
    LE_AUDIO_MEDIA_PLAY = 0,
    LE_AUDIO_MEDIA_PAUSE,
    LE_AUDIO_MEDIA_STOP,
    LE_AUDIO_MEDIA_NEXT,
    LE_AUDIO_MEDIA_PREVIOUS,
} le_audio_media_cmd_t;

/* Starts the host: HCI receive hook on esp_hosted, HCI Reset, controller features, bonds from the store. The
 * esp_hosted SDIO transport to the C6 must be up first: esp_hosted_cyberdeck_connect(timeout_ms) from
 * esp_hosted_cyberdeck.h (patched esp_hosted; without Wi-Fi nothing else brings it up). ESP_ERR_INVALID_STATE: transport
 * down (nothing was sent) or already initialised; ESP_ERR_NOT_SUPPORTED: the C6
 * firmware does not report CIS Central (LE feature bit 28) - flash the C6 firmware from this project. */
esp_err_t le_audio_init(const le_audio_config_t *cfg);

/* Short scan (scan_s seconds) and connection to the first saved LE Audio earbud, with its saved key only (no
 * pairing, a rejected key is kept). The second earbud of the set joins in the background (CSIP, RSI/SIRK).
 * ESP_ERR_NOT_FOUND: no saved earbud seen; ESP_ERR_INVALID_RESPONSE: the earbud rejected the key (pair it again). */
esp_err_t le_audio_connect_bonded(int scan_s);

/* Pairing (only with allow_pairing): scan for scan_s seconds and pair the first LE Audio device that is not saved
 * (earbuds in pairing mode). The second earbud of the set is paired in the same session. WARNING: the first device
 * heard may be someone else's - for a UI prefer le_audio_scan() + le_audio_connect_device() (the user picks). Does
 * not close an existing connection. */
esp_err_t le_audio_pair_new(int scan_s);

/* Scan for scan_s seconds (1..15). Returns the number of devices written to out (LE Audio first, then by RSSI). */
int le_audio_scan(le_audio_device_t *out, int max, int scan_s);

/* Connect the device the user picked from le_audio_scan(). With allow_pairing: a saved device (bonded) tries its key
 * first and is paired again only if the key is rejected, a new one is paired; the second earbud of its set joins in
 * the same session. Without allow_pairing: saved devices only, with their key (a rejected key is kept). An existing
 * connection is closed first. ESP_ERR_NOT_ALLOWED: new device and pairing off; ESP_ERR_INVALID_RESPONSE: the saved key
 * was rejected (pairing off); ESP_FAIL: connection or pairing failed. Blocks up to ~30 s. */
esp_err_t le_audio_connect_device(const le_audio_device_t *dev);

/* Join the missing second earbud of the set now (RSI scan up to 4 s). While streaming with one earbud: scan first
 * (audio keeps playing) and only if the second earbud is advertising, stop the stream (a CIG blocks the join),
 * join it and start again in stereo. le_audio_connect_bonded() / le_audio_pair_new() already join it before
 * returning (up to ~15 s for the background join plus one 6 s retry). */
esp_err_t le_audio_join_set(void);

/* BAP unicast start: PACS, ASCS (Config Codec, Config QoS, Enable), CIG + CIS, ISO data path, stream thread.
 * A second earbud of the set that is still joining or missing gets its join attempt first (see above).
 * Blocks up to several seconds. */
esp_err_t le_audio_start(void);
/* Restart with the current set (e.g. the second earbud came back). */
esp_err_t le_audio_restart(void);
/* Manual stop / disconnect: the background manager does not start / reconnect again until the next connection. */
void le_audio_stop(void);
void le_audio_disconnect(void);

/* true = the stream accepts PCM. */
bool le_audio_ready(void);
/* Interleaved stereo int16 at 48 kHz. Blocks until the encoder queue has room (about 40 ms queued) or timeout_ms
 * elapses; the stream thread is the clock, so writing in a loop paces the producer. Returns frames accepted
 * (0 = stream not ready or timeout). */
size_t le_audio_write_pcm(const int16_t *stereo, size_t frames, int timeout_ms);
/* Digital volume 0..100 applied before LC3 (default 60). */
void le_audio_set_volume(int pct);

void le_audio_get_status(le_audio_status_t *out);

/* Link-layer counters of one stream member's CIS, read from the C6 (vendor HCI 0xFC30, sub-command 0x01).
 * air_loss_pct = payloads flushed after the flush timeout without an acknowledgement / all payloads sent.
 * Blocks up to ~0.5 s (one HCI command). ESP_ERR_NOT_FOUND: no CIS for this member. */
typedef struct {
    uint16_t cis_handle;
    uint32_t events, tx_acked, tx_flushed, tx_underrun, sdu_in, sdu_dropped, sdu_late;
    float air_loss_pct;
} le_audio_cis_stats_t;
esp_err_t le_audio_get_cis_stats(int member, le_audio_cis_stats_t *out);

/* Current digital volume (0..100). */
int le_audio_get_volume(void);

/* Media Control (GMCS) seen by the earbuds: player state, and gestures (Play / Pause / Next / Previous) back. */
void le_audio_set_media_state(bool active, bool playing, const char *title, uint32_t duration_ms);
bool le_audio_take_media_command(le_audio_media_cmd_t *out);

/* ESP32-C6 firmware update over SDIO (esp_hosted's OTA RPC), e.g. with c6_firmware/build/network_adapter.bin linked
 * into the app (target_add_binary_data). Needs only the esp_hosted transport, not le_audio_init(), so it also works
 * when the C6 runs a firmware without LE Audio (le_audio_init() -> ESP_ERR_NOT_SUPPORTED).
 *
 * le_audio_c6_image_version: version of a c6_firmware image (its esp_app_desc_t, "1.4.85"); compare it with
 * le_audio_status_t.coprocessor_fw. ESP_ERR_INVALID_ARG: not a c6_firmware image for the ESP32-C6.
 *
 * le_audio_c6_install: sends the image (progress 0..100 in steps of 5, may be NULL). Ask the user first and stop
 * the stream (le_audio_stop). ESP_OK: the C6 accepted and verified the whole image and restarts into it in ~5 s;
 * restart the P4 a few seconds later (the SDIO link does not survive the C6 restart). The new image runs on trial
 * and goes back to the previous one by itself if it does not work (fw_guard). ESP_ERR_INVALID_STATE: the C6
 * refused to start (its current image is still on trial - try again in a minute). ESP_ERR_INVALID_ARG: not a
 * c6_firmware image, nothing sent. ESP_FAIL: write or verification failed, the C6 keeps its current image.
 * Blocks ~30 s. */
esp_err_t le_audio_c6_image_version(const uint8_t *img, size_t len, char *out, size_t cap);
esp_err_t le_audio_c6_install(const uint8_t *img, size_t len, void (*progress)(int pct, void *ctx), void *ctx);

#ifdef __cplusplus
}
#endif
