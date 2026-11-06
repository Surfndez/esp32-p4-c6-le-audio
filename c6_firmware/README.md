# ESP32-C6 firmware (esp_hosted slave + LE Audio CIS link layer)

The radio co-processor firmware for the M5Stack Tab5. It reports itself as esp_hosted **1.4.85**
(`main/coprocessor_fw_version.txt`, also the version in its app descriptor). Its CIS link layer is the one measured
in [docs/measurements.md](../docs/measurements.md) (1.4.82); 1.4.85 adds the `fw_guard` rule for BLE-only hosts.

## What is inside
- **Base:** the esp-hosted-mcu slave that M5Stack ships on the Tab5 (esp_hosted 1.4.1, commit `96bea3a48e74`), built
  with ESP-IDF v5.5.5 instead of M5Stack's ESP-IDF snapshot, plus a fix for ESP-IDF >= 5.4.
- **CIS link layer:** the stock controller (`libble_app.a`) has no isochronous channels. `cis_ll/` (one source,
  compiled here without a copy) runs CIS Central on top of the controller's connection state machines;
  `main/cis_c6_*.c` is the C6 adapter:
  - `cis_c6_core.c` - engine instance, HCI interception (LE ISO commands, HCI ISO packets), hop to the LL task,
    timers, vendor diagnostics (HCI 0xFC30).
  - `cis_c6_radio.c` - one connsm per CIS subevent (access address, CRC init, CSA#2 subevent channel, anchor),
    SN/NESN written from the engine, NSE 2 with a second connsm.
  - `cis_c6_llcp.c` - LL control PDUs (LL_CIS_REQ/RSP/IND/TERMINATE), ACL encryption snooping, feature bits.
  - `cis_c6_crash.c`, `cis_c6_hang.c` - crash fuse and hang trace.
- **Build-time hooks:** `cmake/ble_hooks.cmake` links a copy of `libble_app.a` with the symbols in
  `cmake/ble_hooks.txt` weakened (`objcopy --weaken-symbol`); `cmake/verify_hooks.py` proves in the linked ELF that
  every hook is ours, every original is reachable as `orig_*`, and every caller inside the library hits the hook.
  A changed ESP-IDF therefore fails the build instead of producing a broken image.
- **Self-guard (`fw_guard`):** trial boot after OTA; self-confirmation after 60 s with SDIO and BLE up and the
  host talking to it (working Wi-Fi if the host turned Wi-Fi on, HCI delivered both ways otherwise); automatic
  rollback (3 crashes, 10 boots, 600 s unconfirmed, RTC watchdog). See [docs/flashing-and-recovery.md](../docs/flashing-and-recovery.md).
- **Defaults that matter:** `acl_margin_us` 2000 (CIS subevents at least 2 ms away from both earbuds' ACL events),
  jitter buffer at least 4 SDUs, NSE 2, `CONFIG_BT_LE_MAX_CONNECTIONS=6` (2 ACL + 2 CIS x 2 subevents).

Without an LE Set CIG Parameters from the host every hook passes everything to the original code, so the firmware
behaves like the stock BLE controller. LE Read Local Supported Features reports bit 28 (CIS Central).

## Build
```
sh c6_firmware/build.sh               # host tests, CIS engine tests, idf.py build, hook check -> RESULT: PASS
sh c6_firmware/build.sh --guard-test  # fw_guard test image (version 1.4.49, never confirms itself)
```
Output: `c6_firmware/build/network_adapter.bin`. The build uses the pinned `espressif/idf:v5.5.5` image (by
digest). The existing `sdkconfig` takes precedence over `sdkconfig.defaults*`: change options in both.

## Flashing
See [docs/flashing-and-recovery.md](../docs/flashing-and-recovery.md): over SDIO from the P4 (`le_audio_c6_install()` in your firmware,
see [docs/integration.md](../docs/integration.md); the Tab5 demo carries this image and installs it after a tap), or over the C6's UART pads (J1 "C6_ISP" on the Tab5) with a USB-UART
adapter and esptool.

The original development notes (Polish) with the vendor diagnostics command table and the experiment switches are
in [NOTES.pl.md](NOTES.pl.md).
