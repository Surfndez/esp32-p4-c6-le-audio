# The CIS link layer on the ESP32-C6

The stock ESP32-C6 BLE controller (ESP-IDF v5.5.5, `libble_app.a`) implements BLE 5.x connections, but its
feature table marks **LE Connected Isochronous Streams as unsupported**: there is no ISO code in the library.
This project adds CIS without replacing the controller.

## How
- **Build-time hooks.** Calls inside `libble_app.a` keep `R_RISCV_CALL` relocations against global symbols. The
  build makes a copy of the library with selected symbols weakened (`objcopy --weaken-symbol`) and provides our
  own implementations, which call the originals through `orig_*` aliases. `verify_hooks.py` checks the linked
  ELF, so a changed ESP-IDF fails the build instead of producing a broken image.
- **CIS events on connection state machines.** Each CIS subevent uses one controller connection state machine
  (connsm) with the CIS access address, CRC init and CSA#2 subevent channel. NSE 2 uses a second connsm for
  subevent 1. The controller's scheduler, radio sequencer and CCM hardware do the air work. Our engine provides
  the PDUs, the SN/NESN state and the flush timeout.
- **Portable engine (`cis_ll/`).** All protocol logic is in plain C with a platform interface (`cis_platform.h`)
  and is unit-tested on the host, including clock drift, host jitter and NSE 2 scenarios in a simulator. The
  C6-specific part is only the adapter (`c6_firmware/main/cis_c6_*.c`).
- **HCI.** Standard HCI ISO commands (Set CIG Parameters, Create CIS, Setup ISO Data Path, ...) and ISO data
  packets from the host are handled by the engine. Everything else goes to the stock controller unchanged.

## What it is not
- It is not a Bluetooth-qualified controller. Interoperability is tested with real earbuds (see the README).
- No Broadcast (BIS / Auracast) yet, only unicast CIS.
