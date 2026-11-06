#!/usr/bin/env python3
"""Upload an ESP32-C6 coprocessor image through the P4 (POST /api/c6/ota) and check that the C6 kept it.

    python tools/c6_ota.py HOST [--bin c6_firmware/build/network_adapter.bin] [--watch S] [--expect VERSION]

HOST (or env LE_AUDIO_HOST) is the address of a P4 firmware that exposes the /api/c6/ota HTTP endpoint.
Authentication: "Authorization: Bearer <token>", token from --token or env LE_AUDIO_OTA_TOKEN. The token is never
printed. The transport is plain HTTP, so use it only on a network you trust.

IMPORTANT: this needs a P4 firmware that exposes /api/c6/ota (le_audio_c6_install() covers firmware with no
network stack). Today the C6 can be updated either from a P4 firmware that implements this endpoint, or over the
J1 UART pads with esptool (`esptool.py --chip esp32c6 ... write_flash`, see docs/flashing-and-recovery.md).

Safety order:
  1. The image header is checked locally (ESP32-C6 chip id, project `network_adapter`, appended SHA-256, size).
  2. The P4 streams the image to the C6, the C6 verifies the whole image and only then switches partitions;
     the P4 restarts and resets the C6.
  3. The fw_guard in the new image confirms it after a healthy run or falls back to the previous image, so the
     C6 version is observed for --watch seconds (default 240).
  The device should be idle (no playback or call) because the P4 restarts after the write.

Exit codes:
  0  the C6 runs the new version for the whole observation window
  1  any other failure (bad file, rejected image, endpoint missing, no answer, P4 did not return)
  2  the C6 reports the same version as before the upload, or went back to the previous one (guard rollback);
     also argparse usage errors (missing host or token)

--expect-rollback: stage 2 of the guard test (build c6_firmware with `--guard-test`). Success (0) means the test
image started and the guard then rolled back to the previous version; 2 means no rollback was seen. The default
observation window is 780 s because the guard trial lasts 10 minutes. Pass --expect with the test image's version
to make the check strict.

The C6 version is read from /api/status (bt.coprocessor_fw). When the BT radio is asleep after the P4 restart,
a short BLE scan (POST /api/diag {"ble_probe": {"scan_s": 1}}) wakes it and returns the version.
"""
from __future__ import annotations

import argparse
import json
import os
import pathlib
import sys
import time
import urllib.error
import urllib.request

REPO = pathlib.Path(__file__).resolve().parent.parent
DEFAULT_BIN = REPO / "c6_firmware" / "build" / "network_adapter.bin"

CHIP_ID_C6 = 0x000D
APP_DESC_MAGIC = 0xABCD5432
PROJECT = "network_adapter"
MAX_IMAGE = 0x180000          # OTA slot size in c6_firmware/partitions.esp32c6.csv
HEADER_NEED = 112             # esp_image_header_t + segment header + esp_app_desc_t up to project_name
UPLOAD_TIMEOUT_S = 180        # slot erase on the C6 takes up to 15 s, plus roughly 850 RPC writes
RESTART_WAIT_S = 660          # the P4 waits up to 10 minutes for playback/calls to end before restarting
WATCH_DEFAULT_S = 240         # guard confirms after 60 s with Wi-Fi up, plus margin for P4 restart and reconnect
GUARD_TEST_WATCH_S = 780      # guard test image: 10 minute trial plus C6 and P4 restarts
POLL_S = 10


def check_image(image: bytes) -> tuple[str | None, str]:
    """Return (error, version from the app descriptor). Same conditions the P4 checks before accepting."""
    if len(image) < HEADER_NEED or image[0] != 0xE9:
        return "not an ESP-IDF application image", ""
    chip = int.from_bytes(image[12:14], "little")
    if chip != CHIP_ID_C6:
        return f"image is for another chip (chip_id 0x{chip:04X}, C6 = 0x{CHIP_ID_C6:04X})", ""
    if image[23] != 1:
        return "image has no appended SHA-256 - the C6 could not verify the whole image", ""
    if int.from_bytes(image[32:36], "little") != APP_DESC_MAGIC:
        return "no esp_app_desc_t at the start of the image", ""
    project = image[80:112].split(b"\0", 1)[0].decode("ascii", "replace")
    if project != PROJECT:
        return f"image of project '{project}', expected '{PROJECT}' (the esp_hosted C6 image)", ""
    if len(image) > MAX_IMAGE:
        return f"image {len(image)} B > OTA slot {MAX_IMAGE} B", ""
    return None, image[48:80].split(b"\0", 1)[0].decode("ascii", "replace")


def expected_coproc_version(bin_path: pathlib.Path) -> str | None:
    """Version the C6 will report (main/coprocessor_fw_version.txt next to build/), or None."""
    txt = bin_path.resolve().parent.parent / "main" / "coprocessor_fw_version.txt"
    if txt.is_file():
        return txt.read_text(encoding="utf-8").strip()
    return None


def coproc_version(status: dict | None) -> str:
    if not status:
        return ""
    return str((status.get("bt") or {}).get("coprocessor_fw") or "")


def verdict(before: str, expected: str | None, seen: list[str]) -> tuple[int, str]:
    """Outcome of the observation; seen = non-empty C6 versions after the restart, oldest first."""
    if not seen:
        return 1, "no coprocessor version in /api/status (Bluetooth off? P4 did not come back?)"
    last = seen[-1]
    target = expected or (last if last != before else None)
    if target is None:
        return 2, f"C6 still reports {last} - same version as before the upload (guard rollback or same image)"
    if last == target and all(v == target for v in seen):
        return 0, f"C6 ran {target} for the whole observation"
    if target in seen and last != target:
        return 2, f"C6 went back from {target} to {last} - the guard rolled the new image back (see C6 log: fw_guard)"
    return 2, f"C6 reports {last}, expected {target}"


def verdict_rollback(before: str, expected: str | None, seen: list[str]) -> tuple[int, str]:
    """Guard test (--expect-rollback): success = the test image showed up and then the C6 returned to `before`."""
    if not seen:
        return 1, "no coprocessor version in /api/status (Bluetooth off? P4 did not come back?)"
    target = expected or next((v for v in seen if v != before), None)
    if target is None or target not in seen:
        return 2, f"test image did not start (C6 reports {seen[-1]})"
    if seen[-1] == before:
        return 0, f"guard works: {target} was not confirmed, C6 went back to {before}"
    return 2, f"C6 still reports {seen[-1]} - no rollback seen (increase --watch: the trial lasts up to 10 min)"


class Device:
    """Minimal HTTP client for the P4 endpoints used here. The token is kept private and never printed."""

    def __init__(self, host: str, token: str):
        self.host = host
        self._token = token

    def _request(self, path: str, data: bytes | None, content_type: str | None) -> urllib.request.Request:
        headers = {"Authorization": f"Bearer {self._token}"}
        if content_type:
            headers["Content-Type"] = content_type
        return urllib.request.Request(f"http://{self.host}{path}", data=data,
                                      method="POST" if data is not None else "GET", headers=headers)

    def get_json(self, path: str, timeout: float = 10) -> dict | None:
        try:
            with urllib.request.urlopen(self._request(path, None, None), timeout=timeout) as r:
                return json.load(r)
        except (urllib.error.URLError, OSError, ValueError):
            return None

    def wake_bt_version(self) -> str:
        """bt.coprocessor_fw is known only while the BT radio is on; a 1 s BLE scan turns it on."""
        body = json.dumps({"ble_probe": {"scan_s": 1}}).encode()
        try:
            with urllib.request.urlopen(self._request("/api/diag", body, "application/json"), timeout=30) as r:
                return str((json.load(r).get("ble_probe") or {}).get("coprocessor_fw") or "")
        except (urllib.error.URLError, OSError, ValueError):
            return ""

    def read_coproc(self) -> str:
        return coproc_version(self.get_json("/api/status")) or self.wake_bt_version()

    def post_image(self, image: bytes) -> tuple[int, dict]:
        req = self._request("/api/c6/ota", image, "application/octet-stream")
        try:
            with urllib.request.urlopen(req, timeout=UPLOAD_TIMEOUT_S) as r:
                return r.status, json.load(r)
        except urllib.error.HTTPError as e:
            body = e.read().decode("utf-8", "replace")
            try:
                return e.code, json.loads(body)
            except ValueError:
                return e.code, {"error": body or e.reason}

    def wait_back(self, before_uptime: int) -> dict | None:
        """Wait for the P4 to restart (uptime lower than before)."""
        deadline = time.time() + RESTART_WAIT_S
        while time.time() < deadline:
            time.sleep(3)
            now = self.get_json("/api/info", timeout=3)
            if now and now.get("uptime_s", 1 << 30) < before_uptime:
                return now
        return None


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("host", nargs="?", default=os.environ.get("LE_AUDIO_HOST", ""),
                    help="P4 address, host or host:port (env LE_AUDIO_HOST)")
    ap.add_argument("--token", default=os.environ.get("LE_AUDIO_OTA_TOKEN", ""),
                    help="bearer token (env LE_AUDIO_OTA_TOKEN)")
    ap.add_argument("--bin", type=pathlib.Path, default=DEFAULT_BIN, help="C6 image (default: %(default)s)")
    ap.add_argument("--watch", type=float, default=WATCH_DEFAULT_S, metavar="S",
                    help="observe the C6 version for S seconds after the P4 restart (guard rollback)")
    ap.add_argument("--expect", default=None,
                    help="version the C6 should report (default: main/coprocessor_fw_version.txt)")
    ap.add_argument("--expect-rollback", action="store_true",
                    help="guard test image (build.sh --guard-test): success = rolled back to the previous "
                         "version; default --watch 780")
    args = ap.parse_args(argv)
    if not args.host:
        ap.error("host is required (positional argument or env LE_AUDIO_HOST)")
    if not args.token.strip():
        ap.error("token is required (--token or env LE_AUDIO_OTA_TOKEN)")
    if args.expect_rollback and args.watch == WATCH_DEFAULT_S:
        args.watch = GUARD_TEST_WATCH_S

    if not args.bin.is_file():
        print(f"missing file {args.bin} (build it with: sh c6_firmware/build.sh)")
        return 1
    image = args.bin.read_bytes()
    err, app_ver = check_image(image)
    if err:
        print(f"REFUSED: {err}")
        return 1
    expected = args.expect or (None if args.expect_rollback else expected_coproc_version(args.bin))
    print(f"image {args.bin.name}: {len(image)} B, app descriptor {app_ver}, C6 will report {expected or '?'}")

    dev = Device(args.host, args.token.strip())
    status = dev.get_json("/api/status")
    before = coproc_version(status) or dev.wake_bt_version()
    info_before = dev.get_json("/api/info", timeout=3) or {}
    print(f"before: C6 {before or '?'}, P4 up {info_before.get('uptime_s', '?')} s")

    code, reply = dev.post_image(image)
    if code == 404:
        print("REFUSED: the P4 firmware has no /api/c6/ota endpoint - flash a P4 firmware that implements it, "
              "or update the C6 over the J1 UART pads with esptool")
        return 1
    if code != 200 or not reply.get("ok"):
        print(f"REFUSED ({code}): {reply.get('error', reply)} - the C6 keeps its current image")
        return 1
    print(f"C6 accepted and verified the image ({reply.get('bytes')} B) - waiting for the P4 restart ...", flush=True)

    if dev.wait_back(int(info_before.get("uptime_s", 1 << 30))) is None:
        print("the P4 did not come back within 11 minutes - check the device")
        return 1
    seen: list[str] = []
    start = time.time()
    deadline = start + args.watch
    while time.time() < deadline:
        v = dev.read_coproc()
        if v and (not seen or seen[-1] != v):
            print(f"C6: {v} ({int(time.time() - start)} s after the P4 came back)", flush=True)
        if v:
            seen.append(v)
        time.sleep(POLL_S)
    rc, why = (verdict_rollback if args.expect_rollback else verdict)(before, expected, seen)
    print(f"REPORT: {why}")
    return rc


if __name__ == "__main__":
    sys.exit(main())
