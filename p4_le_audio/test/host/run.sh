#!/bin/sh
# Host tests of the pure LE Audio models (models/bt): HCI ISO framing, ISO credits and the deadline queue, PACS/ASCS
# parsing, HCI CIG/CIS parameters, LC3 round trip (liblc3), the P4 <-> C6 HCI contract through the real CIS engine
# (cis_ll), bonds (RPA resolution), CSIP (SIRK/RSI), auto start and reconnect state machines.
# Runs in the pinned ESP-IDF image (gcc/g++) or any Linux with gcc. Needs tools/fetch_deps.sh (liblc3).
# Sanitizers: SANITIZE=1. Exit code 0 = all green.
set -e
D="$(cd "$(dirname "$0")/../.." && pwd)"
ROOT="$(cd "$D/.." && pwd)"
LC3="$ROOT/third_party/liblc3"
CIS="$ROOT/cis_ll"
[ -f "$LC3/include/lc3.h" ] || { echo "liblc3 missing - run tools/fetch_deps.sh"; exit 2; }
B="${TMPDIR:-/tmp}/p4_le_audio_test"
rm -rf "$B"
mkdir -p "$B/lc3" "$B/cis"
SAN=""
if [ "${SANITIZE:-0}" = 1 ]; then
    SAN="-g -fsanitize=address,undefined -fno-sanitize-recover=undefined -fno-omit-frame-pointer"
fi
# Third-party / other-module objects: liblc3 without instrumentation, the CIS engine (our C11 code) with it.
(cd "$B/lc3" && gcc -O2 -w -I"$LC3/include" -c "$LC3"/src/*.c)
(cd "$B/cis" && gcc -std=c11 -O1 -Wall -Wextra -Werror $SAN -I"$CIS" -c "$CIS/cis_proto.c" "$CIS/cis_crypto.c" \
    "$CIS/cis_engine.c" "$CIS/test/aes_ref.c")

M="$D/models/bt"
failed=""
t() {
    name=$1
    shift
    if ! g++ -std=c++17 -O1 -Wall -Wextra -Werror $SAN -I"$D/models" "$D/test/host/test_$name.cpp" "$@" \
        -lpthread -lm -o "$B/${name}_tests"; then
        failed="$failed $name(build)"
        return 0
    fi
    "$B/${name}_tests" || failed="$failed $name"
}

t le_audio -I"$LC3/include" "$M/le_audio_model.cpp" "$B"/lc3/*.o
t le_audio_contract -I"$CIS" "$M/le_audio_model.cpp" "$B"/cis/cis_proto.o "$B"/cis/cis_crypto.o "$B"/cis/cis_engine.o \
    "$B"/cis/aes_ref.o
t bt_bond "$M/bt_bond_model.cpp" "$B"/cis/aes_ref.o
t csip "$M/csip_model.cpp" "$B"/cis/aes_ref.o
t bt_audio "$M/bt_audio_model.cpp"
t bt_reconnect "$M/bt_reconnect_model.cpp"
t c6_image "$M/c6_image_model.cpp"
# SD card transfer split of the patched fatfs used by the Tab5 demo (16 kB per SDMMC transaction)
g++ -std=c++17 -O1 -Wall -Wextra -Werror $SAN -I"$ROOT/components/fatfs/diskio" "$ROOT/components/fatfs/test_host/test_sd_split.cpp" -o "$B/sd_split_tests" && "$B/sd_split_tests" || failed="$failed sd_split"

if [ -n "$failed" ]; then
    echo "FAILED:$failed"
    exit 1
fi
echo "ALL PASS"
