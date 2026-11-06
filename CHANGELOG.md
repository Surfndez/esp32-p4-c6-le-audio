# Changelog

## [Unreleased]

First public version.

### Added
- ESP32-C6 CIS link layer on the stock BLE controller (2 CIS in 1 CIG, NSE 2, FT 4, CCM), HCI ISO data path.
- CIS engine (`cis_ll/`) with host unit tests and a discrete-event simulator (drift, host jitter, NSE 2 gates).
- ESP32-C6 firmware 1.4.85 (`c6_firmware/`): esp_hosted slave + CIS adapter, build-time hooks checked in the linked
  ELF, `fw_guard` (trial after OTA, self-confirm, automatic rollback), crash fuse, hang trace, vendor diagnostics.
  `fw_guard` confirms an image for a Wi-Fi host only with working Wi-Fi, and for a BLE-only host with HCI traffic
  delivered in both directions within the 60 s window.
- ESP32-P4 LE Audio host as an ESP-IDF component (`p4_le_audio/`): HCI/ATT/SMP over esp_hosted SDIO, reconnect from
  a saved bond with encryption, PACS/ASCS/BAP unicast client, CSIP set member join (RSI/SIRK), dual LC3 (liblc3),
  ISO credits with lost-NOCP recovery (500 ms) and an SDU deadline queue (25 ms), stream thread holding full CPU
  speed, background manager (reconnect, join before the stream, auto start), scan and connect of the device the
  user picks, CIS statistics from the C6, Media Control (gestures), and the C6 firmware update over SDIO
  (`le_audio_c6_install`). Host tests of the pure models, including the P4 <-> C6 HCI contract through the real CIS
  engine and the C6 image header check.
- Patched esp_hosted 1.4.0 host component: raw HCI channel that carries ISO packets, transport bring-up without
  Wi-Fi, TX buffer ownership fix in the RPC path, longer OTA end timeout.
- `p4_le_audio/test/compile_check`: the smallest firmware that uses the API, built in CI.
- M5Stack Tab5 demo (`examples/tab5_player/`): MP3 files from the SD card to both earbuds, touch UI, device list
  for pairing, bonds in NVS, a copy of the log on the SD card, and the C6 firmware embedded in the app and installed
  with `le_audio_c6_install` after a tap. For it, `components/` also carries the patched fatfs (exFAT, 16 kB SD
  transfers) and the Tab5 display stack. Built in CI.
- Documentation: integration guide, architecture, C6 link layer, flashing and recovery, measurements; notes for AI
  coding agents (`AGENTS.md`).
- Tools: `c6_ota.py` (C6 update through a P4 network endpoint), `privacy_scan.py`, `fetch_deps.sh`. CI workflow.

### Verified on hardware (M5Stack Tab5, Galaxy Buds3 Pro)
- Stereo playback with both earbuds, reconnect from saved keys, the second earbud joining before the stream,
  pairing never-paired earbuds picked from a device list, earbud gestures.
- 15 minutes of stereo at home with 0.016 % / 0.07 % SDUs lost on air; a crowded train without audible dropouts.
- C6 firmware 1.4.85 installed from the P4 with `le_audio_c6_install` in the Tab5 demo (over 1.4.84, and over
  itself); `fw_guard` confirmed it with a Wi-Fi host and with the BLE-only demo (13 minutes of stereo past the 600 s
  trial limit, no rollback).

### Known issues
- The second earbud's CIS sometimes drops with 0x08 a few seconds after the stream starts (in a series right after
  connecting); the host re-joins it. Under investigation.
- Installing over the M5Stack factory C6 firmware is not tested yet.
