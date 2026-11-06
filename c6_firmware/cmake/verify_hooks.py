#!/usr/bin/env python3
"""Cyberdeck v5: after the build, checks that the CIS hooks are wired in (result: exit code 0 = OK).

For every symbol X in ble_hooks.txt:
  1. the ELF has exactly one X and one orig_X, at different addresses;
  2. according to the linker map X lies in a section from our object (libmain.a) and orig_X in a section from the copy
     build/ble_hooked/libble_app.a (the member from ble_hooks.txt); the ESP-IDF original is not linked;
  3. every function of the original library (any object, also the one containing X) that has a relocation against X
     (R_RISCV_CALL, HI20/LO12) and was linked points to <X> in the ELF, not <orig_X>; at least one such function
     is in the ELF (otherwise the hook would be dead); only our X refers to <orig_X>.
"""
import argparse
import os
import re
import subprocess
import sys


def run(cmd, cwd=None):
    return subprocess.run(cmd, cwd=cwd, check=True, capture_output=True, text=True).stdout


def read_hooks(path):
    hooks = []
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line and not line.startswith("#"):
                obj, sym = line.split()
                hooks.append((obj, sym))
    return hooks


def map_sections(map_path):
    """(address, size, input file) of the .text*/.iram* input sections from a GNU ld map."""
    out = []
    pending = None
    rx_one = re.compile(r"^ (\.\S+)\s+0x([0-9a-f]+)\s+0x([0-9a-f]+)\s+(\S.*)$")
    rx_name = re.compile(r"^ (\.\S+)$")
    rx_cont = re.compile(r"^\s+0x([0-9a-f]+)\s+0x([0-9a-f]+)\s+(\S.*)$")
    with open(map_path, encoding="utf-8", errors="replace") as f:
        for line in f:
            line = line.rstrip("\n")
            m = rx_one.match(line)
            if m:
                out.append((int(m.group(2), 16), int(m.group(3), 16), m.group(4), m.group(1)))
                pending = None
                continue
            m = rx_name.match(line)
            if m:
                pending = m.group(1)
                continue
            if pending:
                m = rx_cont.match(line)
                if m:
                    out.append((int(m.group(1), 16), int(m.group(2), 16), m.group(3), pending))
                pending = None
    return out


def owner(sections, addr):
    for a, size, path, name in sections:
        if a <= addr < a + size and (name.startswith(".text") or name.startswith(".iram")):
            return path, name
    return None, None


def lib_referrers(objdump, lib_path, syms):
    """{sym: {(member, function)}} - functions of the whole library with a relocation against sym (a call or an
    address load). Cross-object calls (e.g. next_event from ble_lll_conn.c.o) must hit the hook too."""
    dis = run([objdump, "-dr", lib_path])
    member = None
    func = None
    refs = {s: set() for s in syms}
    rx_member = re.compile(r"^(\S+):\s+file format ")
    rx_func = re.compile(r"^[0-9a-f]+ <([^>]+)>:$")
    rx_rel = re.compile(r"R_RISCV_(CALL|CALL_PLT|HI20|LO12_I|LO12_S|PCREL_HI20|JAL)\s+(\S+)$")
    for line in dis.splitlines():
        m = rx_member.match(line)
        if m:
            member, func = m.group(1), None
            continue
        m = rx_func.match(line)
        if m and not m.group(1).startswith(".L"):
            func = m.group(1)
            continue
        m = rx_rel.search(line)
        if m and m.group(2) in refs and func and func != m.group(2):
            refs[m.group(2)].add((member, func))
    return refs


def elf_refs(objdump, elf, func, target):
    """Whether function func in the ELF refers to <target> (exactly, without +offset)."""
    dis = run([objdump, "-d", "--disassemble=" + func, elf])
    return ("<%s>" % target) in dis


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--elf", required=True)
    ap.add_argument("--map", required=True)
    ap.add_argument("--hooks", required=True)
    ap.add_argument("--orig-lib", required=True)
    ap.add_argument("--objdump", default="riscv32-esp-elf-objdump")
    ap.add_argument("--nm", default="riscv32-esp-elf-nm")
    a = ap.parse_args()

    hooks = read_hooks(a.hooks)
    nm = run([a.nm, a.elf])
    syms = {}
    for line in nm.splitlines():
        p = line.split()
        if len(p) == 3:
            syms.setdefault(p[2], []).append((int(p[0], 16), p[1]))
    sections = map_sections(a.map)
    with open(a.map, encoding="utf-8", errors="replace") as f:
        map_text = f.read()

    ok = True
    print("hook verification: %d hooks" % len(hooks))
    if a.orig_lib in map_text:
        print("FAIL map references the original IDF library %s" % a.orig_lib)
        ok = False
    if "ble_hooked/libble_app.a" not in map_text:
        print("FAIL map does not reference build/ble_hooked/libble_app.a")
        ok = False

    all_refs = lib_referrers(a.objdump, a.orig_lib, [s for _, s in hooks])
    for obj, sym in hooks:
        x = syms.get(sym, [])
        o = syms.get("orig_" + sym, [])
        if len(x) != 1 or len(o) != 1 or x[0][0] == o[0][0]:
            print("FAIL %s: nm %s / orig %s" % (sym, x, o))
            ok = False
            continue
        xp, _ = owner(sections, x[0][0])
        op, _ = owner(sections, o[0][0])
        x_ours = xp is not None and "libmain.a" in xp
        o_lib = op is not None and ("ble_hooked/libble_app.a(%s)" % obj) in op
        # Functions removed by --gc-sections have no symbol in the ELF - nothing to check.
        linked = sorted((f, m) for m, f in all_refs[sym] if f in syms)
        bad = [f for f, _ in linked if not elf_refs(a.objdump, a.elf, f, sym)
               or elf_refs(a.objdump, a.elf, f, "orig_" + sym)]
        orig_callers_ok = elf_refs(a.objdump, a.elf, sym, "orig_" + sym)
        good = x_ours and o_lib and linked and not bad and orig_callers_ok
        ok = ok and bool(good)
        print("%s %s: X=0x%08x [%s] orig=0x%08x [%s]; linked referrers %d/%d -> <X>: %s%s%s" % (
            "OK  " if good else "FAIL", sym, x[0][0], os.path.basename(xp or "?"), o[0][0],
            os.path.basename(op or "?"), len(linked), len(all_refs[sym]),
            ", ".join("%s [%s]" % (f, m) for f, m in linked) or "none",
            "" if not bad else "; WRONG: " + ", ".join(bad),
            "" if orig_callers_ok else "; hook does not call orig"))
    # Who besides the hooks calls orig_* (only our X should).
    full = run([a.objdump, "-d", a.elf])
    func = None
    rx_func = re.compile(r"^[0-9a-f]+ <([^>]+)>:$")
    offenders = set()
    names = {"orig_" + s: s for _, s in hooks}
    for line_ in full.splitlines():
        m = rx_func.match(line_)
        if m:
            func = m.group(1)
            continue
        for orig, sym in names.items():
            if ("<%s>" % orig) in line_ and func not in (sym, orig):
                offenders.add("%s->%s" % (func, orig))
    if offenders:
        print("FAIL orig_* referenced outside hooks: " + ", ".join(sorted(offenders)))
        ok = False
    print("RESULT: %s" % ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
