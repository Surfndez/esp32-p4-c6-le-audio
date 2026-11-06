# Notes for AI coding agents

If you are an AI agent asked to add LE Audio from this repository to another project, read this first. Nothing
here works by copying blindly: check every step against the target project and adapt it.

## Before you change anything

1. Read [docs/integration.md](docs/integration.md) and the C API in
   [p4_le_audio/include/le_audio.h](p4_le_audio/include/le_audio.h).
   [p4_le_audio/test/compile_check](p4_le_audio/test/compile_check) is the smallest firmware that uses the API, and
   the Tab5 demo ([examples/tab5_player](examples/tab5_player)) is a complete one (UI, bonds in NVS, device list,
   C6 installer). Both are built in CI.
2. Find out from the target project, not from assumptions: the board and its SDIO wiring to the ESP32-C6 (pins,
   bus width, clock, reset GPIO, how the C6 is powered), the ESP-IDF version (this repository is built and tested
   with v5.5.5), whether the firmware uses Wi-Fi, PSRAM, power management and an SD card on the same SDMMC
   controller, and where it stores settings (NVS namespaces) so bonds do not collide with existing keys.
3. Run this repository's checks once so you know the baseline (Docker, `espressif/idf:v5.5.5`):
   `sh c6_firmware/build.sh` (must end with `RESULT: PASS`), `sh p4_le_audio/test/compile_check/build.sh`,
   `sh p4_le_audio/test/host/run.sh`, `sh cis_ll/test/run.sh`.

## Rules

- **The ESP32-C6 is the risky part.** On many boards it can be reflashed only over SDIO from the P4. Never flash a
  C6 image whose build did not end with `RESULT: PASS`, never change `fw_guard` without its host tests
  (`c6_firmware/test/run_host_tests.sh`), and ask the user before installing a C6 image on their device.
- Install the C6 image (`le_audio_c6_install`) only after an explicit user action, never automatically at boot:
  the user may run another C6 firmware on purpose.
- Never pair the first device a scan returns without the user choosing it: it may be someone else's.
- Keep the patched esp_hosted host component; the stock one has no ISO-capable HCI channel.
- Bond store callbacks run on PSRAM stacks: serve values from RAM, never read or write flash inside them.
- Do not commit bonding keys, device addresses, IP addresses or tokens; `python tools/privacy_scan.py` checks the
  tree.
- Report what you verified on hardware and what you did not. The [README](README.md#known-limitations) lists what
  is known not to work yet.
