# Architecture

## Split of responsibilities
| Layer | Chip | Component | Notes |
|---|---|---|---|
| Player, mixing, volume | ESP32-P4 | application | PCM 48 kHz stereo from the media pipeline |
| LC3 encoding | ESP32-P4 | liblc3 | one encoder per earbud (L, R), 10 ms frames, 120 octets |
| LE Audio host | ESP32-P4 | `p4_le_audio` | PACS/ASCS/BAP unicast client, CSIP set coordinator, VCP volume, ISO data path |
| ISO flow control | ESP32-P4 | `IsoCredits`, `IsoTxQueue` | host credits from LE Read Buffer Size v2; SDUs wait up to 25 ms for a credit instead of being dropped; credits not acknowledged within 500 ms are recovered |
| Transport | P4 <-> C6 | esp_hosted over SDIO | standard HCI commands, events, ACL and ISO data |
| CIS link layer | ESP32-C6 | `cis_ll` engine + `c6_firmware` adapter | CIG/CIS setup (LL_CIS_REQ/RSP/IND), event scheduling, SDU mapping and jitter buffer, CCM, flush timeout, NOCP |
| BLE controller | ESP32-C6 | Espressif `libble_app.a` (stock) | advertising, scanning, ACL connections, LLCP, encryption; hooked for CIS events |

## Stream timing
- CIG: ISO interval 10 ms, 2 CIS (left, right), NSE 2 (a second transmission chance inside each event), BN 1, FT 4.
- The CIG is placed at least 2 ms away from both earbuds' ACL events (`acl_margin_us`). Closer placement made the
  earbuds drop their own ACL link (0x08 supervision timeout).
- Jitter buffer in the C6: at least 4 SDUs (40 ms), presentation delay 40 ms.
- The P4 stream thread is paced by `esp_timer` and holds the CPU at full speed for the whole stream. Otherwise
  light sleep in screen standby made SDU pacing about 3 % fast relative to the radio.

## Earbud set handling
- The first earbud is reconnected from its saved key. The second is found by its Resolvable Set Identifier (RSI)
  matched against the set's SIRK, read from CSIS over the encrypted link.
- In crowded places, earbud advertisements replace the weakest non-audio entries in a full scan table.
- When the second earbud is missing, joining is retried 4 times quickly, then every 3 minutes for as long as
  the first one stays connected.
