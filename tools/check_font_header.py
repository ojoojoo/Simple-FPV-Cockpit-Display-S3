#!/usr/bin/env python3
"""
Validate a generated GFX font header the way a C compiler reads it:
apply line continuations and strip comments FIRST, then count entries.

A regex run over the raw lines will not catch a glyph swallowed by a
trailing backslash in a comment, which is exactly the bug this guards
against.

Usage: python3 check_font_header.py HEADER.h [HEADER.h ...]
"""
import re, sys

FIRST, LAST = 0x20, 0x7E
EXPECTED = LAST - FIRST + 1


def preprocess(src):
    src = src.replace("\\\n", "")                 # line continuations
    src = re.sub(r'/\*.*?\*/', '', src, flags=re.S)
    src = re.sub(r'//[^\n]*', '', src)            # // comments
    return src


def check(path):
    raw = open(path, encoding="utf-8").read()
    src = preprocess(raw)
    ok = True

    gl = re.search(r'Glyphs\[\]\s*=\s*\{(.*?)\};', src, re.S)
    bm = re.search(r'Bitmaps\[\]\s*=\s*\{(.*?)\};', src, re.S)
    fo = re.search(r'0x20,\s*0x7E,\s*(\d+)\s*\};', src)
    if not (gl and bm and fo):
        print("%s: FAIL - could not find the arrays" % path)
        return False

    entries = re.findall(r'\{([^{}]*)\}', gl.group(1))
    nbytes = len(re.findall(r'0x[0-9A-Fa-f]{2}', bm.group(1)))

    if len(entries) != EXPECTED:
        print("%s: FAIL - %d glyph entries, expected %d"
              % (path, len(entries), EXPECTED))
        ok = False

    # Offsets must be strictly consistent with the packed sizes.
    expect_off = 0
    for i, e in enumerate(entries):
        f = [int(x) for x in e.split(",")]
        off, w, h = f[0], f[1], f[2]
        if off != expect_off:
            print("%s: FAIL - glyph 0x%02X offset %d, expected %d"
                  % (path, FIRST + i, off, expect_off))
            ok = False
            break
        expect_off += (w * h + 7) // 8
    if ok and expect_off != nbytes:
        print("%s: FAIL - bitmap is %d bytes, glyphs account for %d"
              % (path, nbytes, expect_off))
        ok = False

    # No line may end with a backslash: that is a C line continuation.
    for n, line in enumerate(raw.splitlines(), 1):
        if line.rstrip("\n").endswith("\\"):
            print("%s:%d FAIL - line ends with a backslash" % (path, n))
            ok = False

    if ok:
        print("%s: OK - %d glyphs, %d bitmap bytes, yAdvance %s"
              % (path, len(entries), nbytes, fo.group(1)))
    return ok


if __name__ == "__main__":
    sys.exit(0 if all([check(p) for p in sys.argv[1:]]) else 1)
