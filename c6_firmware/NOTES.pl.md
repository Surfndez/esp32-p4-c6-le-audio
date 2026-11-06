# Notatki projektowe wsadu ESP32-C6 (oryginal, po polsku)

Notatki z rozwoju (Cyberdeck, wsad v5), zachowane bez tlumaczenia: baza zrodel M5Stack, zachowanie adaptera CIS,
diagnostyka HCI vendor 0xFC30, przelaczniki eksperymentow i samowylaczenie po awariach. Liczby wersji i pomiary sa
z chwili pisania; aktualny opis po angielsku: README.md. Sciezki `firmware/le_audio/cis_ll` = `cis_ll/` w tym repo.

## Baza = zrodlo fabrycznego wsadu M5Stack
Fabryczny wsad C6 (M5Burner "Tab5 ESP32-C6 Wi-Fi SDIO" v1.4.1) zbudowano z esp-hosted-mcu commit
`96bea3a48e74` (opis aplikacji: wersja "96bea3a", bez "-dirty"). Sprawdzone 2026-10-30 skrotami git blob: kazdy plik
`slave/`, `common/` i submodul protobuf-c tego commita jest w v4 i v5 bajt w bajt taki sam, poza naszymi zmianami
(rejestr 1.4.1 = `ee11e08`, a `96bea3a` doklada tylko 2 commity w `host/`). Konfiguracja: `sdkconfig.defaults*` i
`partitions.esp32c6.csv` jak w tym commicie plus opisane dopiski (tablica partycji i naglowki obrazu jak w wsadzie
M5Stack). Zmienione pliki zrodla M5Stack: `CMakeLists.txt`, `main/CMakeLists.txt`, `main/Kconfig.projbuild`,
`main/app_main.c` (straznik), `main/slave_bt.c` (przechwycenie HCI dla silnika CIS), `main/slave_control.c` (wersja,
OTA w czasie proby, poprawka pod IDF >= 5.4), `sdkconfig.defaults.esp32c6`; reszta to nowe pliki (`fw_guard*`,
`cis_c6_*`, `cmake/`, testy). Jedyna roznica poza tym: ESP-IDF 5.5.5 (wydanie) zamiast migawki M5Stack
`v5.5-dev-407-gf420609c332-dirty` - inne biblioteki radia (Wi-Fi, BLE, PHY, koegzystencja). Zostaje 5.5.5: hooki CIS
i cala analiza RE sa na `libble_app.a` z 5.5.5, migawka ma niejawne zmiany lokalne ("dirty") i nie ma poprawek
z wydania. Zgodnosc w praktyce sprawdza etap 2 (obraz testowy z powrotem do wsadu M5Stack).

## Co to jest
- Wszystko z v4 (`wsad v4 (nie w tym repozytorium)`): esp_hosted slave 1.4.1, poprawka pod ESP-IDF >= 5.4, straznik
  `fw_guard` (proba nowego obrazu, powrot do poprzedniego, licznik awarii startu w RTC).
- Adapter LE Audio: kontroler BLE C6 nie ma kanalow izochronicznych, wiec lacze CIS (Central, NSE 1 albo 2, BN = FT = 1)
  robi silnik `cis_ll` (jedno zrodlo, kompilowane tutaj bez kopii) na "pseudo-polaczeniu" ACL
  zamknietego kontrolera. P4 rozmawia z C6 standardowym HCI (komendy LE ISO, pakiety HCI ISO 0x05).
  - `main/cis_c6_core.c` - instancja silnika, przechwyt HCI, przeskok do zadania LL, timer, diagnostyka 0xFC30.
  - `main/cis_c6_radio.c` - connsm na podzdarzenie z AA/CRC/interwalem/anchor CIS, jeden PDU na zdarzenie connsm,
    SN/NESN wpisywane z silnika, surowy odbior przed HCI, priorytet harmonogramu 15. NSE 2 (przelacznik `max_nse`,
    domyslnie 2 od 1.4.75, z mikrofonem NSE 1): drugi connsm o Sub_Interval pozniej z kanalem podzdarzenia 1
    (`docs/c6-link-layer.md`).
  - `main/cis_c6_llcp.c` - nadawanie i odbior PDU sterujacych CIS na ACL, podglad szyfrowania (LTK, SKD, IV),
    bity cech LL 28 (CIS Central) i 32 (ISO Host Support), AES-128 przez mbedtls.
  - `main/slave_bt.c` - trzy wpiecia: `process_hci_rx_pkt` -> `cis_c6_h2c`, `host_rcv_pkt` -> `cis_c6_c2h`,
    `slave_bt_send_to_host` (tor silnik -> host bez ponownego przechwytu).
- Hooki do `libble_app.a` bez zmian w ESP-IDF: `cmake/patch_ble_lib.cmake` robi kopie biblioteki w
  `build/ble_hooked/`, oslabia 5 symboli (`cmake/ble_hooks.txt`) i dodaje aliasy `orig_*`; `cmake/ble_hooks.cmake`
  (wczytywany z `main/project_include.cmake`) podstawia te kopie w miejsce oryginalu. Wszystkie hookowane funkcje
  leza w `.text.*` (flash), nie w IRAM.

| hook | wolajacy w bibliotece | po co |
|---|---|---|
| `r_ble_lll_conn_calc_first_anchor_point` | `r_ble_lll_conn_master_new` | pierwszy anchor CIS |
| `r_ble_lll_conn_update_buffer_hdr` | `r_ble_lll_conn_recycle_sch_item` | RX/TX zdarzenia CIS bez HCI |
| `r_ble_ll_conn_next_event` | `r_ble_lll_conn_sched_next_anchor` | PDU na nastepne zdarzenie, pominiete zdarzenia |
| `r_ble_ll_ctrl_rx_pdu` | `r_ble_ll_conn_rx_data_pdu` | LL_CIS_RSP/IND/TERMINATE_IND, podglad ENC_RSP |
| `r_ble_ll_ctrl_enc_req_make` | `r_ble_ll_ctrl_proc_init` | podglad SKDm/IVm |
| `r_ble_lll_conn_coex_dpc_process` | `master_new`, `reschedule_event`, `slave_new`, `coex_dpc_update` | PTI koegzystencji z Wi-Fi dla zdarzen CIS |

## Zachowanie wzgledem v4
- Bez CIG (do pierwszej LE Set CIG Parameters) kazdy hook przepuszcza wszystko do oryginalu; silnik tylko
  podglada szyfrowanie ACL (zwykle rusza przed CIG).
- Roznice widoczne zawsze: LE Read Local Supported Features (0x2003) zglasza bit 28 (CIS Central); po LE Set Host
  Feature bit 32 (P4 wysyla ja przy starcie BT) bity 28 i 32 sa tez w slowie cech kontrolera, wiec widzi je druga
  strona w LL_FEATURE_RSP. HCI Reset je czysci.
- Komendy przejmowane: 0x2060, 0x2061, 0x2062, 0x2064, 0x2065, 0x206E, 0x206F, 0x2074 (tylko bit 32), 0x0406 dla
  uchwytow >= 0x0E00, 0xFC30 z magia. Podgladane i przekazywane dalej: 0x2019, 0x0C03. Pakiety ISO od hosta
  przejmuje zawsze adapter (przy wylaczonym adapterze sa odrzucane - VHCI nie ma toru ISO).
- Blad `cis_c6_init()` (log "LE Audio (CIS) wylaczone, BLE bez zmian") = praca jak v4 bez bitu 28.
- Konfiguracja: `CONFIG_BT_LE_MAX_CONNECTIONS=6` (2 ACL + 2 CIS x 2 podzdarzenia, od 1.4.74),
  `CONFIG_BT_LE_CONTROLLER_TASK_STACK_SIZE=6144`
  (silnik dziala w zadaniu LL). Pamiec statyczna +6,8 kB wzgledem v4 (glownie pierscien H2C 6 kB), wolne D/IRAM
  235 kB; silnik bierze `cis_engine_mem_size()` (14 400 B) z wewnetrznego heapu raz, przy pierwszym starcie BT.
  Obraz 1,18 MB, 25 % wolnego w partycji OTA (pomiar 2026-10-30).

## Diagnostyka 0xFC30 (HCI vendor, OGF 0x3F OCF 0x030)
Parametry: `'C' 'I' 'S' '5'`, podkomenda, argumenty. Bez magii komenda idzie do kontrolera jak w v4.
Odpowiedz w Command Complete: status, wersja formatu (1), podkomenda, dane (liczby u32/u16 little endian).

| podkomenda | argumenty | dane |
|---|---|---|
| 0x00 | - | flagi (bit0 CIG aktywny, bit1 zadanie LL znane, bit2 bity cech ISO wlaczone), liczba N, N x u32 licznikow `cis_c6_counters_t` (kolejnosc jak w `main/cis_c6.h`, nowe tylko na koncu) |
| 0x01 | u16 uchwyt CIS | 14 x u32 `cis_engine_stats_t`; status 0x02 = nieznany uchwyt |
| 0x02 | u16 uchwyt ACL | u8: 0 = SK silnika == SK kontrolera, 1 = zgodne po odwroceniu bajtow, 2 = niezgodne, 3 = brak szyfrowania (klucze nie opuszczaja C6) |
| 0x03 | - | u32 najmniejszy wolny stos zadania LL od startu (bajty; 0xFFFFFFFF = zadanie LL jeszcze nieznane) |
| 0x04 | u8 ktora (0 pierwsze zdarzenia po ostatnim otwarciu lacza, 1 ostatnie), u16 indeks | u32 ile zapisano, u8 n (<= 8), n x 24 B `cis_c6_bb_t` (czarna skrzynka, uklad w `main/cis_c6.h`) |
| 0x05 | u8 numer przelacznika, u32 wartosc | u8 numer, u32 wartosc obowiazujaca; zly numer albo zakres = 0x12 bez zmian |
| 0x06 | - | u8 N, N x u32 wartosci przelacznikow |
| 0x07 | u16 zdarzenia (10..6000), u8 dlugosc PDU, u8 PHY (1/2) | i8 kod startu testu radia bez Peripheral (0 = ruszyl); status 0x0C przy aktywnym CIG silnika |
| 0x08 | - | 36 B `cis_c6_rt_t` (wynik testu radia) |
| 0x09 | - | u8 powod resetu tego startu (`esp_reset_reason_t`), 160 B `cis_c6_hang_t` - ostatni slad zawieszenia (magia 0 = brak): zadanie, MEPC/RA, adresy kodu ze stosu, liczniki, obraz (img); zapisuje go ISR watchdoga zadan do RTC, po dwoch kolejnych zadzialaniach (ok. 10 s bez IDLE) abort, nastepny start przenosi go do NVS (`cis_c6`/`hang`) |
| 0x0A | u32 od indeksu | u32 lacznie, u32 pierwszy oddany indeks, u8 n (<= 8), n x 28 B `cis_c6_tr_t` - slad zestawiania CIS (kroki silnika `cis_trace_t`, anchor ACL w chwili z LL_CIS_IND), krazace ostatnie 32 |

Inna podkomenda albo dlugosc: status 0x12.

### Przelaczniki eksperymentow (0x05/0x06)
Tylko w RAM, po kazdym starcie C6 wartosci domyslne. Sluza do sprawdzania hipotez na Tab5 bez ponownego wgrywania.
Strukturalne (zapas, limit zdarzenia) dzialaja od nastepnego otwarcia lacza CIS, reszta od nastepnego zdarzenia.

| nr | nazwa | domyslnie | zakres | co robi |
|---|---|---|---|---|
| 0 | SEQ_POLARITY | 0 | 0-1 | bit ls+20 = SN/NESN (0) albo negacja (1) - R1 |
| 1 | SEQ_WRITE | 1 | 0-1 | 0 = adapter nie wpisuje SN/NESN, prowadzi je sprzet (porownanie R1) |
| 2 | HDR_SEQ | 1 | 0-1 | 0 = w bajcie 0 bufora TX tylko LLID i NPI (D4) |
| 3 | MIN_LEAD_US | 3000 | 500-100000 | najmniejszy zapas przed pierwszym anchor |
| 4 | SCHED_PRIO | 15 | 0-15 | priorytet harmonogramu po zdarzeniu (0 = jak kontroler, 8) |
| 5 | COEX_PTI | 9 | 0-15 | PTI wobec Wi-Fi zdarzen CIS (0 = jak kontroler, 4 zwykle) |
| 6 | COEX_PTI_HI | 0 | 0-15 | drugi PTI ls+40[7:4] (0 = bez zmian) |
| 7 | EVT_LIMIT | 1 | 0-1 | limit zdarzenia na jedna wymiane (bit 7 +566 i DLE) |

### Samowylaczenie po awariach
Przy otwartym laczu CIS adapter trzyma w pamieci RTC znacznik. Reset awaryjny (panic, watchdog, lockup) przy
zapalonym znaczniku = awaria w trakcie CIS, liczona w NVS (`cis_c6/crash`) dla danego obrazu. Od 2 awarii adapter
sie nie wlacza (log `LE Audio wylaczone po N awariach`, BLE jak w 1.4.1) az do wgrania innego obrazu. Licznik widac
w 0xFC30/0x00 (`crashes_in_cis`). Logika: `main/cis_c6_crash.c`, test `test/test_cis_c6_crash.c`.

