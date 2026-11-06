#!/usr/bin/env python3
"""Scan a repository for owner/private data that must not be published.

    python tools/privacy_scan.py [ROOT] [--rules RULES.json] [--allow ALLOW.txt]

ROOT defaults to the repository root (the parent of tools/). Stdlib only.

Built-in rules (see RULE NAMES below): private/CGNAT/link-local IPv4 addresses, *.ts.net hosts and Tailscale auth
keys, user home directories (Windows, Linux, macOS, WSL), e-mail addresses, private keys and common token formats,
quoted secrets, and hex bonding keys (LTK/IRK/SIRK or anything named key/token).

Extra rules file (--rules PATH or env PRIVACY_RULES), JSON:
    {"replace": [{"from": "...", "to": "...", "ci": true, "word": true}, ...], "forbid": ["...", ...]}
Every "from" and "forbid" string is a forbidden literal ("ci": case-insensitive, "word": whole word only; forbid
strings are matched case-insensitively). File names are checked against these literals too. Keep such a file
outside the repository.

Allowlist: --allow PATH holds one regex per line ('#' starts a comment); a hit is dropped when a match of any
regex overlaps it. A line containing the marker `privacy-scan: allow` is skipped entirely. The placeholders
BMAC_URL, BMAC_USERNAME and COPYRIGHT_HOLDER are always allowed.

Skipped: .git, build*/ directories, managed_components, third_party, __pycache__, .venv, node_modules, binary
files (containing a NUL byte) and files larger than 5 MB.

Output: one line per hit `path:line: rule: masked snippet` (first 2 characters + ***, so the scan never prints
the secret), then `privacy_scan: N hit(s) in M file(s)`. Exit code: 0 no hits, 1 hits, 2 usage error.
"""
from __future__ import annotations

import argparse
import json
import os
import pathlib
import re
import sys
from typing import Callable, Iterable, NamedTuple

MAX_FILE_BYTES = 5 * 1024 * 1024
SKIP_DIRS = {".git", "managed_components", "third_party", "__pycache__", ".venv", "node_modules"}
SKIP_DIR_PREFIX = "build"
# Vendored upstream code kept byte-identical (its copyright headers carry the upstream authors' public addresses).
# Relative to the scanned root, '/' separators.
SKIP_PATHS = {"c6_firmware/main/common/protobuf-c", "components/espressif__esp_hosted/common/protobuf-c"}
ALLOW_MARKER = "privacy-scan: allow"
BUILTIN_ALLOWED = ("BMAC_URL", "BMAC_USERNAME", "COPYRIGHT_HOLDER")


class Hit(NamedTuple):
    path: str
    line: int
    rule: str
    text: str  # the matched text; callers must print mask(text), never this


class Rule(NamedTuple):
    name: str
    regex: re.Pattern[str]
    check: Callable[[re.Match[str]], bool] | None = None  # False = not a hit


def mask(text: str) -> str:
    return text[:2] + "***"


# --- generic rules ----------------------------------------------------------------------------------------------

_IPV4 = re.compile(r"(?<![\w.])(\d{1,3})\.(\d{1,3})\.(\d{1,3})\.(\d{1,3})(?!\w|\.\d)")


def _ipv4_octets(m: re.Match[str]) -> tuple[int, ...] | None:
    octets = tuple(int(g) for g in m.groups())
    return octets if all(o <= 255 for o in octets) else None


def _ip_check(kind: str) -> Callable[[re.Match[str]], bool]:
    def check(m: re.Match[str]) -> bool:
        o = _ipv4_octets(m)
        if o is None:
            return False
        a, b = o[0], o[1]
        if kind == "private":
            return a == 10 or (a == 172 and 16 <= b <= 31) or (a == 192 and b == 168)
        if kind == "cgnat":
            return a == 100 and 64 <= b <= 127
        return a == 169 and b == 254  # link-local

    return check


_PLACEHOLDER_NAMES = {
    "username", "user", "you", "public", "default", "name", "yourname", "your-name", "your_name", "me",
    "runner", "ubuntu", "all users", "default user", "user name", "home", "someone",
}


def _is_placeholder_name(name: str) -> bool:
    n = name.strip().lower()
    return (n in _PLACEHOLDER_NAMES or n[:1] in {"$", "%", "{", "~", "<", "*", "."} or n.startswith("your")
            or not any(c.isalnum() for c in n))


_WIN_NAME = r"([^\\/\s\"'<>:*?|]+)"
_WIN_PATH = re.compile(r"(?i)[A-Z]:(?:\\{1,2}|/)Users(?:\\{1,2}|/)" + _WIN_NAME)
_WIN_POSIX_PATH = re.compile(r"(?<![\w.\-])/(?:(?:mnt|cygdrive)/)?[A-Za-z]/Users/" + _WIN_NAME)
_HOME_PATH = re.compile(r"(?<![\w.\-/:])/(?:home|Users)/" + _WIN_NAME)


def _name_check(m: re.Match[str]) -> bool:
    return not _is_placeholder_name(m.group(1))


_EMAIL = re.compile(r"[A-Za-z0-9._%+\-]+@([A-Za-z0-9\-]+(?:\.[A-Za-z0-9\-]+)*\.[A-Za-z]{2,})")
_EMAIL_OK_DOMAINS = {"example.com", "example.org", "example.net", "users.noreply.github.com", "github.com",
                     "gitlab.com", "bitbucket.org"}
_EMAIL_OK_LOCALS = {"git", "noreply", "no-reply", "user", "username", "you", "name", "yourname", "me", "email",
                    "someone"}
_EMAIL_OK_TLDS = {"invalid", "test", "localhost", "example", "png", "jpg", "jpeg", "gif", "svg", "webp", "ico",
                  "css", "js", "ts", "json", "html", "md", "txt", "py", "c", "h", "cpp"}


def _email_check(m: re.Match[str]) -> bool:
    domain = m.group(1).lower()
    local = m.group(0).split("@", 1)[0].lower()
    if domain in _EMAIL_OK_DOMAINS or domain.endswith(".example.com") or domain.endswith(".users.noreply.github.com"):
        return False
    if local in _EMAIL_OK_LOCALS or local.startswith("noreply") or local.startswith("no-reply"):
        return False
    if domain.rsplit(".", 1)[-1] in _EMAIL_OK_TLDS:  # icon@2x.png and similar
        return False
    return True


_PLACEHOLDER_VALUE_HINTS = ("your", "example", "xxxx", "changeme", "change-me", "placeholder", "<", ">", "${",
                            "$(", "{{", "...", "dummy", "sample", "redacted", "todo", "here")


def _secret_check(m: re.Match[str]) -> bool:
    value = m.group("value").lower()
    if value.startswith("$") or any(h in value for h in _PLACEHOLDER_VALUE_HINTS) or len(set(value)) <= 2:
        return False
    return True


def _hex_check(m: re.Match[str]) -> bool:
    return len(set(m.group("value").lower())) > 2  # all zeros / all f are placeholders


def generic_rules() -> list[Rule]:
    return [
        Rule("ipv4-private", _IPV4, _ip_check("private")),
        Rule("ipv4-cgnat", _IPV4, _ip_check("cgnat")),
        Rule("ipv4-linklocal", _IPV4, _ip_check("linklocal")),
        Rule("tailnet-host", re.compile(r"(?i)\b[a-z0-9][a-z0-9.\-]*\.ts\.net\b")),
        Rule("tailscale-key", re.compile(r"\btskey-[A-Za-z0-9_\-]{8,}")),
        Rule("user-path-windows", _WIN_PATH, _name_check),
        Rule("user-path-windows", _WIN_POSIX_PATH, _name_check),
        Rule("user-path-home", _HOME_PATH, _name_check),
        # Any other absolute Windows path names a directory on someone's machine (project folders, drives);
        # documentation uses "C:/..." as a placeholder.
        Rule("windows-abs-path", re.compile(r"(?<![A-Za-z0-9])[A-Za-z]:[\\/]+(?!\.\.\.|Users\b)[A-Za-z0-9_\-]{2,}")),
        Rule("email", _EMAIL, _email_check),
        Rule("private-key", re.compile(r"-----BEGIN (?:RSA |EC |OPENSSH |DSA |ENCRYPTED )?PRIVATE KEY-----")),
        Rule("aws-key", re.compile(r"\bAKIA[0-9A-Z]{16}\b")),
        Rule("github-token", re.compile(r"\bgh[pousr]_[A-Za-z0-9]{36,}")),
        Rule("slack-token", re.compile(r"\bxox[baprs]-[A-Za-z0-9\-]{4,}")),
        Rule("quoted-secret",
             re.compile(r"""(?i)(?:api[_-]?key|secret|token|password)["']?\s*[:=]\s*['"](?P<value>[^'"\s]{12,})['"]"""),
             _secret_check),
        Rule("bonding-key-hex",
             re.compile(r"""(?i)\b\w*(?:key|token|ltk|irk|sirk)\w*["']?\s*[:=]\s*["']?(?:0x)?"""
                        r"""(?P<value>[0-9a-f]{32,})(?![0-9a-z])"""),
             _hex_check),
        Rule("bonding-key-bytes",
             re.compile(r"(?i)\b\w*(?:ltk|irk|sirk)\w*\s*(?:\[[^\]]*\])?\s*=\s*\{\s*0x[0-9a-f]{2}"
                        r"(?:\s*,\s*0x[0-9a-f]{2}){15,}")),
    ]


# --- rules file and allowlist ----------------------------------------------------------------------------------

def _literal_rule(name: str, literal: str, ci: bool, word: bool) -> Rule:
    pattern = re.escape(literal)
    if word:
        pattern = rf"(?<!\w){pattern}(?!\w)"
    return Rule(name, re.compile(pattern, re.IGNORECASE if ci else 0))


def parse_rules(data: object) -> list[Rule]:
    """Literal rules from the parsed JSON rules file. Raises ValueError on a wrong structure."""
    if not isinstance(data, dict):
        raise ValueError("rules file must be a JSON object")
    rules: list[Rule] = []
    for i, entry in enumerate(data.get("replace", [])):
        if not isinstance(entry, dict) or not isinstance(entry.get("from"), str):
            raise ValueError(f"replace[{i}] needs a string 'from'")
        if entry["from"]:
            rules.append(_literal_rule(f"rules-replace[{i}]", entry["from"], bool(entry.get("ci")),
                                       bool(entry.get("word"))))
    for i, literal in enumerate(data.get("forbid", [])):
        if not isinstance(literal, str):
            raise ValueError(f"forbid[{i}] must be a string")
        if literal:
            rules.append(_literal_rule(f"rules-forbid[{i}]", literal, True, False))
    return rules


def load_rules(path: pathlib.Path) -> list[Rule]:
    return parse_rules(json.loads(path.read_text(encoding="utf-8")))


def load_allow(path: pathlib.Path | None) -> list[re.Pattern[str]]:
    patterns = [re.compile(re.escape(p)) for p in BUILTIN_ALLOWED]
    if path is not None:
        for raw in path.read_text(encoding="utf-8").splitlines():
            line = raw.strip()
            if line and not line.startswith("#"):
                patterns.append(re.compile(line))
    return patterns


# --- scanning ---------------------------------------------------------------------------------------------------

def _allowed(line: str, span: tuple[int, int], allow: Iterable[re.Pattern[str]]) -> bool:
    for pat in allow:
        for am in pat.finditer(line):
            if am.start() < span[1] and span[0] < am.end():  # overlap
                return True
    return False


def scan_text(text: str, rules: Iterable[Rule], allow: Iterable[re.Pattern[str]] = (),
              path: str = "<text>") -> list[Hit]:
    rules = list(rules)
    allow = list(allow) or load_allow(None)
    hits: list[Hit] = []
    for lineno, line in enumerate(text.split("\n"), 1):
        if ALLOW_MARKER in line:
            continue
        for rule in rules:
            for m in rule.regex.finditer(line):
                if rule.check is not None and not rule.check(m):
                    continue
                if _allowed(line, m.span(), allow):
                    continue
                hits.append(Hit(path, lineno, rule.name, m.group(0)))
    return hits


def iter_files(root: pathlib.Path) -> Iterable[pathlib.Path]:
    for dirpath, dirnames, filenames in os.walk(root):
        rel = pathlib.Path(dirpath).relative_to(root).as_posix()
        dirnames[:] = sorted(
            d for d in dirnames
            if d not in SKIP_DIRS and not d.startswith(SKIP_DIR_PREFIX)
            and (f"{rel}/{d}" if rel != "." else d) not in SKIP_PATHS
        )
        for name in sorted(filenames):
            yield pathlib.Path(dirpath) / name


def scan_file(file: pathlib.Path, rel: str, rules: list[Rule], literal_rules: list[Rule],
              allow: list[re.Pattern[str]]) -> list[Hit]:
    hits: list[Hit] = []
    for hit in scan_text(rel, literal_rules, allow, rel):  # file names against the rules-file literals
        hits.append(hit._replace(line=0))
    try:
        if file.is_symlink() or file.stat().st_size > MAX_FILE_BYTES:
            return hits
        data = file.read_bytes()
    except OSError as e:
        print(f"privacy_scan: cannot read {rel}: {e}", file=sys.stderr)
        return hits
    if b"\0" in data:
        return hits
    return hits + scan_text(data.decode("utf-8", "replace"), rules, allow, rel)


def scan_tree(root: pathlib.Path, extra_rules: list[Rule], allow: list[re.Pattern[str]]) -> list[Hit]:
    rules = generic_rules() + extra_rules
    hits: list[Hit] = []
    for file in iter_files(root):
        rel = file.relative_to(root).as_posix()
        hits.extend(scan_file(file, rel, rules, extra_rules, allow))
    return hits


def main(argv: list[str] | None = None) -> int:
    default_root = pathlib.Path(__file__).resolve().parent.parent
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("root", nargs="?", type=pathlib.Path, default=default_root,
                    help="directory to scan (default: repository root)")
    ap.add_argument("--rules", type=pathlib.Path, default=os.environ.get("PRIVACY_RULES") or None,
                    help="JSON rules file with forbidden literals (env PRIVACY_RULES)")
    ap.add_argument("--allow", type=pathlib.Path, default=None, help="file with regexes of allowed matches")
    args = ap.parse_args(argv)

    if not args.root.is_dir():
        print(f"privacy_scan: not a directory: {args.root}", file=sys.stderr)
        return 2
    try:
        extra = load_rules(pathlib.Path(args.rules)) if args.rules else []
        allow = load_allow(args.allow)
    except (OSError, ValueError, re.error) as e:  # json.JSONDecodeError is a ValueError
        print(f"privacy_scan: bad rules/allow file: {type(e).__name__}: {e}", file=sys.stderr)
        return 2

    hits = scan_tree(args.root.resolve(), extra, allow)
    for h in hits:
        print(f"{h.path}:{h.line}: {h.rule}: {mask(h.text)}")
    print(f"privacy_scan: {len(hits)} hit(s) in {len({h.path for h in hits})} file(s)")
    return 1 if hits else 0


if __name__ == "__main__":
    sys.exit(main())
