# Measurements (2026-11-02, M5Stack Tab5 + Galaxy Buds3 Pro)

All windows were recorded **on the device** with the recorder of the original firmware (vendor diag `le_audio_rec`,
not part of this repository). It stores
samples in PSRAM, writes JSONL to the SD card and downloads after the window, so network dropouts do not corrupt
measurements. Music played during every window.

| Condition | Duration | Earbuds | SDU lost on air (L / R) | Host-side drops |
|---|---|---|---|---|
| Home, `acl_margin_us` 2000 | 15 min | 2 the whole time | 0.016 % / 0.07 % | 0 |
| Home, release build | 5 min | 2 the whole time | 0.003 % / 0.02 % | 0 |
| Home, `acl_margin_us` 500 (old) | 3.5 min | second dropped twice (0x08) | 0.7 % / 0.35 % | 0 |
| Crowded place (64 BLE devices in 8 s), `acl_margin_us` 500 (old) | 2 min | 2 | 1.9 % / 3.3 % | 0 |

Findings that shaped the defaults:
- **Screen standby:** P4 light sleep shifted `esp_timer`, so SDUs went out about 3 % fast and 6–18 % arrived
  late. Fix: full CPU speed for the whole stream.
- **ACL margin:** earbuds stop serving their ACL when CIS subevents start right after it. Fix: 2 ms margin.
- **Crowded places:** measured only with the old 500 µs margin, which also gave losses of this order at home; the
  2 ms margin fixed them there. With the fix, a listening test on a crowded train (many people on Bluetooth
  earbuds) played without interruptions; no recorder numbers from a crowd yet. With the old margin the losses
  were spread evenly across all 37 channels (a channel map would not help), and a higher TX power (9 / 18 / 20 dBm)
  or the Wi-Fi coexistence priority made no difference.
- **SD card on the shared SDMMC controller:** a failing card stalled SDIO for 60 ms and more. A 16-slot ISO pool
  rides out about 70 ms.
