# Flashing and recovery

The ESP32-C6 on the Tab5 sits behind the ESP32-P4: it is reachable only over SDIO (esp_hosted). The P4 can reset
it (EN line) and power-cycle it, but it cannot put it into the ROM download mode. A C6 image that breaks SDIO can
therefore only be fixed with a cable. The firmware is built so that this should never be necessary.

## Normal update path (OTA over SDIO)
The P4 sends the C6 image with esp_hosted's standard OTA RPC (`rpc_ota_begin` / `rpc_ota_write` / `rpc_ota_end`).
Your P4 firmware does this with `le_audio_c6_install()` from `p4_le_audio`, for example with the image linked into
the app (see [integration.md](integration.md#4-installing-the-c6-firmware-from-your-firmware-ota)); a P4 firmware
with a network endpoint can take the image from `tools/c6_ota.py` instead (`POST /api/c6/ota`). The Tab5 demo
carries the image and installs it when you tap **C6 firmware → Install**
([`c6_update.cpp`](../examples/tab5_player/main/c6_update.cpp)).

1. The P4 checks the image header (chip ESP32-C6, project `network_adapter`, appended SHA-256).
2. The C6 writes it to the inactive OTA slot and **verifies the whole image** before switching to it.
3. The P4 restarts and resets the C6; the new image starts as a **trial**.

## Safety net on the C6 (`fw_guard`)
| Protection | What it does |
|---|---|
| Trial boot | An image installed by OTA counts as unconfirmed until it has worked for 60 s: SDIO and BLE up, and the P4 talking to it. A P4 that turned Wi-Fi on (RPC WifiStart) needs a connected Wi-Fi station with traffic both ways (Wi-Fi may be its only way to update the C6 again); a BLE-only P4 firmware needs HCI traffic delivered in both directions within the window. |
| Automatic rollback | Back to the previous confirmed image after 3 crashes, 10 boots, 600 s of unconfirmed run time, or an RTC watchdog reset. |
| CIS crash fuse | Two C6 crashes while a CIS is active disable the CIS layer until the next update; the C6 then behaves like the stock BLE controller. |
| Hang trace | The task watchdog ISR records the hung task, MEPC/RA and stack code addresses to RTC memory; the next boot moves them to NVS for diagnosis (vendor HCI 0x09). |
| P4 control | The P4 can always reset (EN) or power-cycle the C6. |

The rollback path is tested before every release with a dedicated guard-test image that never confirms itself:
`sh c6_firmware/build.sh --guard-test`, then `python tools/c6_ota.py --bin <guardtest image> --expect-rollback`
(about 10 minutes until the automatic return to the previous image).

## Last resort: J1 "C6_ISP" pads
Only needed if an image breaks SDIO before the guard can act. The Tab5 has 6 pads labelled **J1 C6_ISP**
(3V3, RXD, TXD, RST, IO9, GND). With a 3.3 V USB-UART adapter, hold IO9 low during reset to enter the ROM
bootloader and flash the factory M5Stack image (or any working image) with `esptool.py --chip esp32c6`.
