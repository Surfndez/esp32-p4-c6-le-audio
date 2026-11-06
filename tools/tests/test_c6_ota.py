"""Tests for the pure helpers in tools/c6_ota.py (no network)."""
import contextlib
import io
import os
import pathlib
import sys
import unittest
from unittest import mock

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))
import c6_ota  # noqa: E402


def make_image(chip=c6_ota.CHIP_ID_C6, sha=1, magic=c6_ota.APP_DESC_MAGIC, project=c6_ota.PROJECT,
               version="1.4.82", size=c6_ota.HEADER_NEED) -> bytes:
    img = bytearray(max(size, c6_ota.HEADER_NEED))
    img[0] = 0xE9
    img[12:14] = chip.to_bytes(2, "little")
    img[23] = sha
    img[32:36] = magic.to_bytes(4, "little")
    img[48:48 + len(version)] = version.encode()
    img[80:80 + len(project)] = project.encode()
    return bytes(img)


class CheckImage(unittest.TestCase):
    def test_valid(self):
        self.assertEqual(c6_ota.check_image(make_image()), (None, "1.4.82"))

    def test_rejections(self):
        cases = {
            "not an ESP-IDF": b"\x00" * 200,
            "too short": b"\xe9" * 20,
            "another chip": make_image(chip=0x0012),
            "SHA-256": make_image(sha=0),
            "esp_app_desc_t": make_image(magic=0),
            "project": make_image(project="other"),
            "OTA slot": make_image(size=c6_ota.MAX_IMAGE + 1),
        }
        for needle, image in cases.items():
            err, _ = c6_ota.check_image(image)
            self.assertIsNotNone(err, needle)
            if needle != "too short":
                self.assertIn(needle, err)

    def test_exactly_max_size_is_accepted(self):
        self.assertIsNone(c6_ota.check_image(make_image(size=c6_ota.MAX_IMAGE))[0])


class Verdicts(unittest.TestCase):
    def test_new_version_ok(self):
        self.assertEqual(c6_ota.verdict("1.4.81", "1.4.82", ["1.4.82", "1.4.82"])[0], 0)

    def test_guard_rollback_detected(self):
        self.assertEqual(c6_ota.verdict("1.4.81", "1.4.82", ["1.4.82", "1.4.81"])[0], 2)

    def test_same_version_without_expectation(self):
        self.assertEqual(c6_ota.verdict("1.4.81", None, ["1.4.81"])[0], 2)
        self.assertEqual(c6_ota.verdict("1.4.81", None, ["1.4.82"])[0], 0)

    def test_no_version_seen(self):
        self.assertEqual(c6_ota.verdict("1.4.81", "1.4.82", [])[0], 1)
        self.assertEqual(c6_ota.verdict_rollback("1.4.81", None, [])[0], 1)

    def test_rollback_mode(self):
        self.assertEqual(c6_ota.verdict_rollback("1.4.81", None, ["1.4.49", "1.4.81"])[0], 0)
        self.assertEqual(c6_ota.verdict_rollback("1.4.81", "1.4.49", ["1.4.49"])[0], 2)
        self.assertEqual(c6_ota.verdict_rollback("1.4.81", "1.4.49", ["1.4.81"])[0], 2)

    def test_coproc_version(self):
        self.assertEqual(c6_ota.coproc_version({"bt": {"coprocessor_fw": "1.4.82"}}), "1.4.82")
        self.assertEqual(c6_ota.coproc_version({"bt": None}), "")
        self.assertEqual(c6_ota.coproc_version(None), "")


class CommandLine(unittest.TestCase):
    def run_main(self, argv, env=None):
        err = io.StringIO()
        with mock.patch.dict(os.environ, env or {}, clear=True), contextlib.redirect_stderr(err), \
                contextlib.redirect_stdout(io.StringIO()):
            try:
                rc = c6_ota.main(argv)
            except SystemExit as e:
                rc = e.code
        return rc, err.getvalue()

    def test_host_and_token_required(self):
        rc, err = self.run_main([])
        self.assertEqual(rc, 2)
        self.assertIn("host is required", err)
        rc, err = self.run_main(["somehost"])
        self.assertEqual(rc, 2)
        self.assertIn("token is required", err)

    def test_env_vars_are_used_and_missing_file_fails_cleanly(self):
        rc, _ = self.run_main(["--bin", "does-not-exist.bin"],
                              {"LE_AUDIO_HOST": "somehost", "LE_AUDIO_OTA_TOKEN": "t"})
        self.assertEqual(rc, 1)


if __name__ == "__main__":
    unittest.main()
