# tab5_player - LE Audio MP3 player for the M5Stack Tab5

Minimal ESP-IDF app for the ESP32-P4 of the M5Stack Tab5. It plays every `*.mp3` under `/sd/music` (recursive,
sorted, in a loop forever) to LE Audio earbuds: the first earbud is connected with its saved key, the second earbud
of the set joins by its RSI/SIRK (CSIP), and each earbud gets its own channel (left / right).

Requirements:
- ESP32-C6 running the firmware from [`c6_firmware/`](../../c6_firmware) (CIS link layer). The player carries it:
  with the stock M5Stack C6 image (or another version) boot stage 5 says so, and **C6 firmware → Install** writes
  it to the C6 over esp_hosted's standard OTA (about 30 s, then the Tab5 restarts). No adapter needed.
- Earbuds that support LE Audio unicast (tested: Samsung Galaxy Buds3 Pro).
- An SD card (FAT32 or exFAT) with a folder `music` at its root. Only `*.mp3` files are played (any bitrate and
  sample rate, mono or stereo, resampled to 48 kHz stereo); other files are skipped. Subfolders are searched up to
  8 levels deep, tracks play in alphabetical order of their paths, in a loop. Another folder: `idf.py menuconfig`
  -> `tab5_player` -> music directory.

**On the display** (touch, landscape, backlight always on): firmware state (`TEST (not confirmed - reset returns
to previous firmware)` when OTA-installed), boot stages 1-7 with OK / FAIL and a short detail, both earbuds (left /
right once the stream knows their Audio Locations, otherwise main / second; state, battery and RSSI of the main
earbud, air loss per CIS read from the C6), the track with position and play / pause, stream counters, and buttons
*Connect / Rescan* (joins the second earbud when the first is connected), *Play / Pause*, *Next*, *Vol -*, *Vol +*
and *Find earbuds*: a 6 s scan, then a list of the devices found (name, RSSI, `LE Audio`, `saved`); tap yours to
connect or pair it, *Rescan* / *Close* below the list; *C6 firmware* shows the C6 version next to the one the
player carries and installs it after a tap.

**In the serial log** (USB Serial/JTAG, `idf.py monitor`) the same boot stages are tagged `STAGE n/7` (board,
transport to the C6, SD card, bonds, LE Audio host + C6 version, connect, stream); every scan logs what it heard
(advertising reports, devices, resolvable private addresses, which ones resolved to a saved bond and whether the
connect filter takes them); `JOIN ...` lines show the second earbud joining (before the stream starts, or later
with a short stop of the stream); the link state of both earbuds and the stream counters follow every 5 s. The same
log is copied to `/sd/tab5_player.log` (previous run: `tab5_player.old`) unless `TAB5_PLAYER_LOG_TO_SD` is off:
```
I (35012) tab5_player: link connected, peer XX:XX:XX:XX:XX:XX (...), paired 1, L/R: main up, second up, battery 80 %, C6 1.4.85
I (35012) tab5_player: stream streaming, members 2 stereo, LC3 48000 Hz/120 B, PD 40000 us | sdu tx 6012 drop 0 ...
```
Earbud gestures arrive as Media Control commands: play / pause, next and previous track.

## Build
```
sh c6_firmware/build.sh                     # first: the C6 image the player embeds (c6_firmware/build/network_adapter.bin)
sh examples/tab5_player/build.sh            # fetches liblc3 + dr_libs (tools/fetch_deps.sh), builds in espressif/idf:v5.5.5
```
Output: `examples/tab5_player/build/tab5_player.bin` (plus bootloader and partition table). Options in
`idf.py menuconfig` -> `tab5_player`: music directory, digital volume, pairing (below).

## Flash over USB (normal use)
The example has its own partition table (`partitions.csv`, 16 MB: two 7 MB app slots, NVS at 0x11000, table at
0x10000), bootloader and OTA data:
```
cd examples/tab5_player/build
python -m esptool --chip esp32p4 -p <PORT> -b 460800 --before default_reset --after hard_reset write_flash "@flash_args"
```
(or, with a local ESP-IDF v5.5.5: `cd examples/tab5_player && idf.py -p <PORT> flash monitor`).
This replaces the firmware on the ESP32-P4. The ESP32-C6 is not touched.

### Pairing
With no saved earbuds the player opens the device list by itself. Put the earbuds into pairing mode (Galaxy Buds:
both in the case, lid open, hold the touch pads until the LED pulses), tap *Rescan* if they are not listed yet, then
tap them. The second earbud of the set is paired in the same session. Nothing is paired without a tap - the player
never takes "the first device it hears", which may be someone else's.

- `TAB5_PLAYER_ALLOW_PAIRING` (default on): new devices in the list can be paired; a saved device connects with its
  key and is paired again only if it rejects that key. Off: saved earbuds only, new ones are shown greyed out.
- `TAB5_PLAYER_SAVE_BONDS` (default on): new bonds are written to NVS (`sec`/`bt.bond`, `sec`/`bt.irk`), so the
  earbuds reconnect by themselves after a reset. Off: new bonds stay in RAM until reset.

## Test without a cable (OTA with automatic return)
If the Tab5 already runs another firmware that can install an OTA image into its inactive app slot, and its
bootloader was built with `CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=y`, you can try the player without a USB cable and
get the previous firmware back with a reset:

1. The other firmware must use the same partition table (app slots of at least 0x700000 bytes, table at 0x10000,
   NVS at 0x11000) - the player's image is about 2.5 MB (1.2 MB of it is the embedded C6 firmware).
2. Install `build/tab5_player.bin` with that firmware's OTA (it writes the inactive slot and selects it for the
   next boot).
3. The player boots as **pending verify** and **never confirms itself** (no call to
   `esp_ota_mark_app_valid_cancel_rollback()`). Its boot log says so:
   `image state: PENDING_VERIFY (next reset returns to the previous firmware)`.
4. Any reset (reset button, power cycle, crash, watchdog) makes the bootloader mark the player's image as aborted
   and boot the previous firmware again.

For this test turn `TAB5_PLAYER_SAVE_BONDS` off (`idf.py menuconfig`). The player then opens NVS **read-only**
(a copy of the partition descriptor with `readonly = true`: the NVS library skips its clean-up writes and every
flash write or erase is refused) and only reads the saved bonds (`sec`/`bt.bond`), our IRK (`sec`/`bt.irk`) and the
address mode (`cfg`/`bt.ident`). Earbuds paired by the other firmware therefore connect without pairing, and the
other firmware finds its NVS unchanged after the return. With `TAB5_PLAYER_SAVE_BONDS` on (default) a new pairing
writes NVS.

Status on hardware: see the [main README](../../README.md#status).
