#!/bin/sh
# Builds the ESP32-C6 firmware (esp_hosted slave + LE Audio CIS link layer) in the pinned ESP-IDF container.
# Nothing is flashed.
# Usage: sh c6_firmware/build.sh               -> c6_firmware/build/network_adapter.bin
#        sh c6_firmware/build.sh --guard-test  -> c6_firmware/build_guardtest/network_adapter.bin (fw_guard test
#           image: never confirms itself, version 1.4.49, separate sdkconfig)
# Mounts the whole repository, because main/CMakeLists.txt compiles the CIS engine from its single source, cis_ll/.
# Order: host tests (fw_guard, CIS engine), idf.py build, hook verification in the linked ELF.
# The ESP-IDF image is pinned by digest (a tag can be re-pushed, a digest cannot).
set -e
IDF_IMAGE="espressif/idf:v5.5.5@sha256:a9231d0697ab8f7517cc072e93b7c83e04907bfbfba80b6440d7dbbf90665cf2"
VARIANT="${1:-}"
case "$VARIANT" in ""|--guard-test) ;; *) echo "usage: sh $0 [--guard-test]"; exit 2 ;; esac
FW="$(cd "$(dirname "$0")/.." && pwd)"
# Git Bash on Windows: host path as C:/..., container paths without MSYS conversion.
case "$(uname -s)" in MINGW*|MSYS*) FW="$(cd "$FW" && pwd -W)"; export MSYS_NO_PATHCONV=1 ;; esac
docker run --rm -e VARIANT="$VARIANT" -v "$FW:/fw" -w /fw/c6_firmware "$IDF_IMAGE" bash -c '
	set -e
	. /opt/esp/idf/export.sh >/dev/null 2>&1
	sh test/run_host_tests.sh
	sh ../cis_ll/test/run.sh
	if [ "$VARIANT" = "--guard-test" ]; then
		B=build_guardtest
		idf.py -B "$B" -D IDF_TARGET=esp32c6 -D SDKCONFIG="$B/sdkconfig" \
			-D "SDKCONFIG_DEFAULTS=sdkconfig.defaults;sdkconfig.guardtest" build
		grep -q "^CONFIG_CYBERDECK_FW_GUARD_TEST=y" "$B/sdkconfig" || { echo "missing CYBERDECK_FW_GUARD_TEST"; exit 1; }
	else
		B=build
		[ -f sdkconfig ] || idf.py set-target esp32c6 >/dev/null
		grep -q "^CONFIG_CYBERDECK_FW_GUARD_TEST=y" sdkconfig && { echo "sdkconfig contains the test image"; exit 1; }
		idf.py build
	fi
	python3 cmake/verify_hooks.py --elf "$B/network_adapter.elf" --map "$B/network_adapter.map" \
		--hooks cmake/ble_hooks.txt \
		--orig-lib "$IDF_PATH/components/bt/controller/lib_esp32c6/esp32c6-bt-lib/esp32c6/libble_app.a"
'
