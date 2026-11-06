"""Tests for tools/privacy_scan.py. Sensitive-looking samples are assembled at runtime from pieces so this file
does not trigger the scanner itself."""
import contextlib
import io
import json
import pathlib
import sys
import tempfile
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))
import privacy_scan as ps  # noqa: E402


def rules_hit(text: str, extra: list[ps.Rule] | None = None) -> list[str]:
    return [h.rule for h in ps.scan_text(text, ps.generic_rules() + (extra or []), ps.load_allow(None))]


def dot(*parts: object) -> str:
    return ".".join(str(p) for p in parts)


class GenericRules(unittest.TestCase):
    def test_private_ipv4(self):
        for ip in (dot(10, 1, 2, 3), dot(172, 16, 0, 9), dot(172, 31, 255, 1), dot(192, 168, 4, 1)):
            self.assertIn("ipv4-private", rules_hit(f"host = {ip}"), ip)

    def test_cgnat_and_linklocal(self):
        self.assertIn("ipv4-cgnat", rules_hit(dot(100, 64, 0, 1)))
        self.assertIn("ipv4-cgnat", rules_hit(dot(100, 127, 9, 9)))
        self.assertIn("ipv4-linklocal", rules_hit(dot(169, 254, 1, 1)))

    def test_public_loopback_and_versions_are_not_ips(self):
        for text in ("bind 0.0.0.0", "127.0.0.1", "mask 255.255.255.255", "version 1.4.82", "ESP-IDF v5.5.5",
                     "8.8.8.8", dot(172, 32, 0, 1), dot(100, 63, 0, 1), dot(100, 128, 0, 1), "v" + dot(10, 0, 0, 1),
                     dot(10, 0, 0, 1, 5), dot(10, 300, 0, 1), "1.2.3"):
            self.assertEqual(rules_hit(text), [], text)

    def test_windows_absolute_path(self):
        bs = chr(92)
        self.assertIn("windows-abs-path", rules_hit("cd " + "D:" + "/" + "Work/repo"))
        self.assertIn("windows-abs-path", rules_hit("E:" + bs + "src" + bs + "x"))
        for text in ("host path as C:/..., ok", "http://a:b/c", "ratio 1:2/3"):
            self.assertNotIn("windows-abs-path", rules_hit(text), text)

    def test_tailnet_host_and_key(self):
        self.assertIn("tailnet-host", rules_hit("ssh box." + "tail1234" + ".ts.net"))
        self.assertIn("tailscale-key", rules_hit("key tskey-" + "auth-abcdef123456"))

    def test_windows_and_unix_user_paths(self):
        users = "Users"
        cases = ["C:\\" + users + "\\alice\\proj", "c:/" + users + "/alice/proj", "/c/" + users + "/alice/x",
                 "/mnt/c/" + users + "/alice", "C:\\\\" + users + "\\\\alice\\\\x", "/ho" + "me/alice/.ssh",
                 "/" + users + "/alice/Library"]
        for text in cases:
            self.assertTrue(any(r.startswith("user-path") for r in rules_hit(text)), text)

    def test_user_path_placeholders_are_ignored(self):
        users = "Users"
        for text in ("C:\\" + users + "\\<user>\\x", "C:\\" + users + "\\USERNAME\\x", "C:\\" + users + "\\you\\x",
                     "C:\\" + users + "\\Public\\x", "/ho" + "me/<user>/x", "/ho" + "me/runner/work",
                     "/" + users + "/you/x", "C:\\" + users + "\\%USERNAME%\\x", "/ho" + "me/$USER/x"):
            self.assertEqual(rules_hit(text), [], text)

    def test_windows_path_reported_once(self):
        self.assertEqual(rules_hit("C:/" + "Users/alice/x"), ["user-path-windows"])

    def test_email(self):
        self.assertIn("email", rules_hit("contact: " + "jane.doe" + "@" + "mailbox.pl"))

    def test_generic_emails_are_ignored(self):
        for text in ("a@" + "example.com", "a@" + "example.org", "noreply" + "@" + "anthropic.com",
                     "12345+bob@" + "users.noreply.github.com", "git" + "@" + "github.com:me/repo.git",
                     "user" + "@" + "host", "icon" + "@" + "2x.png", "esp-idf" + "@" + "v5.5", "@types/node"):
            self.assertEqual(rules_hit(text), [], text)

    def test_secret_formats(self):
        samples = {
            "private-key": "-----BEGIN " + "OPENSSH PRIVATE KEY-----",
            "aws-key": "AK" + "IA" + "ABCDEFGHIJKLMNOP",
            "github-token": "gh" + "p_" + "a" * 36,
            "slack-token": "xo" + "xb-" + "1234-abcd",
            "quoted-secret": 'api_' + 'key = "' + "Zq8" * 6 + '"',
            "bonding-key-hex": "ltk = " + "a1b2" * 8,
        }
        for rule, text in samples.items():
            self.assertIn(rule, rules_hit(text), rule)

    def test_bonding_key_variants(self):
        self.assertIn("bonding-key-hex", rules_hit('"sirk": "' + "0123456789abcdef" * 2 + '"'))
        self.assertIn("bonding-key-hex", rules_hit("peer_irk: 0x" + "9f3c" * 8))
        bytes16 = ", ".join("0x%02x" % (i * 7 + 3) for i in range(16))
        self.assertIn("bonding-key-bytes", rules_hit("static uint8_t ltk[16] = { " + bytes16 + " };"))

    def test_secret_placeholders_are_ignored(self):
        for text in ('token = "' + 'your-token-here' + '"', 'password: "' + "${PASSWORD_FROM_ENV}" + '"',
                     "key = " + "0" * 32, "sha256 = " + "ab" * 32, 'secret = "' + "x" * 20 + '"',
                     "key = " + "a1b2" * 4):
            self.assertEqual(rules_hit(text), [], text)


class Allowing(unittest.TestCase):
    def test_inline_marker(self):
        self.assertEqual(rules_hit(dot(10, 0, 0, 1) + "  # privacy-scan: allow"), [])

    def test_builtin_placeholders_cover_overlapping_hit(self):
        self.assertEqual(rules_hit("mailto:BMAC_USERNAME" + "@" + "mailbox.pl"), [])

    def test_allow_file(self):
        with tempfile.TemporaryDirectory() as d:
            p = pathlib.Path(d) / "allow.txt"
            p.write_text("# comment\n\n" + "10" + r"\.9\.9\.9(?!\d)" + "\n", encoding="utf-8")
            allow = ps.load_allow(p)
        text = dot(10, 9, 9, 9) + " " + dot(10, 9, 9, 8)
        found = ps.scan_text(text, ps.generic_rules(), allow)
        self.assertEqual([h.text for h in found], [dot(10, 9, 9, 8)])


class RulesFile(unittest.TestCase):
    def rules(self, data):
        return ps.parse_rules(data)

    def test_case_sensitive_literal(self):
        r = self.rules({"replace": [{"from": "Zorblax", "to": "X"}]})
        self.assertEqual(rules_hit("a Zorblax b", r), ["rules-replace[0]"])
        self.assertEqual(rules_hit("a zorblax b", r), [])

    def test_ci_literal(self):
        r = self.rules({"replace": [{"from": "Zorblax", "to": "X", "ci": True}]})
        self.assertEqual(rules_hit("a ZORBLAX b", r), ["rules-replace[0]"])

    def test_word_literal(self):
        r = self.rules({"replace": [{"from": "zorb", "to": "X", "word": True}]})
        self.assertEqual(rules_hit("a zorb b", r), ["rules-replace[0]"])
        self.assertEqual(rules_hit("zorbing", r), [])
        self.assertEqual(rules_hit("(zorb)", r), ["rules-replace[0]"])

    def test_forbid_is_case_insensitive_substring(self):
        r = self.rules({"forbid": ["Quuxville"]})
        self.assertEqual(rules_hit("in quuxvilles", r), ["rules-forbid[0]"])

    def test_literal_with_regex_chars_is_escaped(self):
        r = self.rules({"forbid": ["a.b"]})
        self.assertEqual(rules_hit("axb", r), [])
        self.assertEqual(rules_hit("a.b", r), ["rules-forbid[0]"])

    def test_bad_structure(self):
        with self.assertRaises(ValueError):
            self.rules({"replace": [{"to": "x"}]})
        with self.assertRaises(ValueError):
            self.rules({"forbid": [1]})
        with self.assertRaises(ValueError):
            self.rules([])


class Masking(unittest.TestCase):
    def test_mask_hides_everything_after_two_chars(self):
        self.assertEqual(ps.mask("supersecret"), "su***")
        self.assertEqual(ps.mask("a"), "a***")


class CommandLine(unittest.TestCase):
    def run_cli(self, *argv):
        out, err = io.StringIO(), io.StringIO()
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
            try:
                rc = ps.main(list(argv))
            except SystemExit as e:  # argparse usage errors
                rc = int(e.code)
        return rc, out.getvalue(), err.getvalue()

    def make_tree(self, root: pathlib.Path, ip: str):
        (root / "src").mkdir()
        (root / "src" / "a.txt").write_text("x\nip " + ip + "\n", encoding="utf-8")
        for skipped in (".git", "build", "build_guardtest", "managed_components", "third_party", "__pycache__"):
            (root / skipped).mkdir()
            (root / skipped / "leak.txt").write_text(ip, encoding="utf-8")
        (root / "blob.bin").write_bytes(b"\0" + ip.encode())
        (root / "big.txt").write_text((ip + "\n") * 1 + "a" * (ps.MAX_FILE_BYTES + 1), encoding="utf-8")

    def test_scan_hits_masks_and_skips(self):
        ip = dot(192, 168, 7, 7)
        with tempfile.TemporaryDirectory() as d:
            root = pathlib.Path(d)
            self.make_tree(root, ip)
            rc, out, _ = self.run_cli(str(root))
        self.assertEqual(rc, 1)
        self.assertIn("src/a.txt:2: ipv4-private: 19***", out)
        self.assertNotIn(ip, out)
        self.assertIn("privacy_scan: 1 hit(s) in 1 file(s)", out)

    def test_clean_tree_exits_zero(self):
        with tempfile.TemporaryDirectory() as d:
            (pathlib.Path(d) / "ok.txt").write_text("version 1.4.82 on 127.0.0.1\n", encoding="utf-8")
            rc, out, _ = self.run_cli(d)
        self.assertEqual(rc, 0)
        self.assertIn("privacy_scan: 0 hit(s) in 0 file(s)", out)

    def test_rules_file_and_filename_check(self):
        with tempfile.TemporaryDirectory() as d:
            root = pathlib.Path(d) / "repo"
            root.mkdir()
            (root / "notes.txt").write_text("hello Zorblax\n", encoding="utf-8")
            (root / "zorblax-file.txt").write_text("clean\n", encoding="utf-8")
            rules = pathlib.Path(d) / "rules.json"
            rules.write_text(json.dumps({"replace": [{"from": "zorblax", "to": "X", "ci": True}]}), encoding="utf-8")
            rc, out, _ = self.run_cli(str(root), "--rules", str(rules))
        self.assertEqual(rc, 1)
        self.assertIn("notes.txt:1: rules-replace[0]: Zo***", out)
        self.assertIn("zorblax-file.txt:0: rules-replace[0]: zo***", out)
        self.assertNotIn("Zorblax", out)

    def test_usage_errors_exit_two(self):
        with tempfile.TemporaryDirectory() as d:
            self.assertEqual(self.run_cli(d + "/missing")[0], 2)
            bad = pathlib.Path(d) / "bad.json"
            bad.write_text("{not json", encoding="utf-8")
            self.assertEqual(self.run_cli(d, "--rules", str(bad))[0], 2)
            self.assertEqual(self.run_cli(d, "--rules", d + "/nope.json")[0], 2)
        self.assertEqual(self.run_cli("--bogus")[0], 2)


if __name__ == "__main__":
    unittest.main()
