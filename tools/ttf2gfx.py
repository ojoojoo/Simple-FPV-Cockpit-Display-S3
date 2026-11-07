#!/usr/bin/env python3
"""
Convert a TrueType font into an Adafruit-GFX-style C header that
LovyanGFX can use directly.

Replicates the conventions of Adafruit's fontconvert.c:
  width/height  = glyph bitmap dimensions in pixels
  xAdvance      = horizontal advance, whole pixels
  xOffset       = bitmap_left
  yOffset       = 1 - bitmap_top
  bitmaps       = 1 bit per pixel, MSB first, row-major, packed
                  continuously within a glyph and zero-padded to a
                  byte boundary at the end of each glyph
  yAdvance      = ascent + descent (line height)

Usage:
  python3 ttf2gfx.py FONT.ttf EM_PIXELS NAME > NAME.h
"""
import sys
from PIL import ImageFont

FIRST, LAST = 0x20, 0x7E


def build(ttf_path, em_px, name):
    font = ImageFont.truetype(ttf_path, em_px)
    ascent, descent = font.getmetrics()

    bitmaps = []          # flat list of bytes
    glyphs = []           # dicts
    offset = 0

    for code in range(FIRST, LAST + 1):
        ch = chr(code)
        mask = font.getmask(ch, mode="1")
        w, h = mask.size
        try:
            _, off = font.getmask2(ch, mode="1")
            left, top_from_asc = off
        except Exception:
            left, top_from_asc = 0, 0
        bitmap_top = ascent - top_from_asc
        x_adv = int(round(font.getlength(ch)))

        # Pack this glyph's bits.
        bits = []
        for y in range(h):
            for x in range(w):
                bits.append(1 if mask.getpixel((x, y)) else 0)
        while len(bits) % 8:
            bits.append(0)
        gbytes = []
        for i in range(0, len(bits), 8):
            b = 0
            for k in range(8):
                b = (b << 1) | bits[i + k]
            gbytes.append(b)

        glyphs.append(dict(off=offset, w=w, h=h, adv=x_adv,
                           xo=left, yo=1 - bitmap_top, ch=ch, code=code))
        bitmaps.extend(gbytes)
        offset += (w * h + 7) // 8
        assert offset == len(bitmaps), "bitmap offset desync at %r" % ch

    return bitmaps, glyphs, ascent + descent


def emit(name, bitmaps, glyphs, y_advance, src_note):
    o = []
    o.append("// %s - generated from %s by tools/ttf2gfx.py" % (name, src_note))
    o.append("// Do not edit by hand.")
    o.append("//")
    o.append("// B612 is licensed under the SIL Open Font License v1.1.")
    o.append("// See docs/OFL.txt. Copyright 2012 The B612 Project Authors.")
    o.append("")
    o.append("#pragma once")
    o.append("")
    o.append("const uint8_t %sBitmaps[] = {" % name)
    for i in range(0, len(bitmaps), 12):
        row = ", ".join("0x%02X" % b for b in bitmaps[i:i + 12])
        o.append("  " + row + ("," if i + 12 < len(bitmaps) else ""))
    o.append("};")
    o.append("")
    o.append("const lgfx::GFXglyph %sGlyphs[] = {" % name)
    # Characters that must never appear raw at the end of a // comment.
    # A trailing backslash is a C line continuation and would swallow the
    # next line of the table, silently deleting a glyph.
    SAFE_LABEL = {0x20: "space", 0x5C: "backslash", 0x3F: "question"}
    for i, g in enumerate(glyphs):
        label = SAFE_LABEL.get(g["code"], g["ch"])
        o.append("  {%6d, %3d, %3d, %3d, %4d, %4d }%s  // 0x%02X %s"
                 % (g["off"], g["w"], g["h"], g["adv"], g["xo"], g["yo"],
                    "," if i < len(glyphs) - 1 else " ", g["code"], label))
    o.append("};")
    o.append("")
    o.append("const lgfx::GFXfont %s = {" % name)
    o.append("  (uint8_t *)%sBitmaps," % name)
    o.append("  (lgfx::GFXglyph *)%sGlyphs," % name)
    o.append("  0x%02X, 0x%02X, %d };" % (FIRST, LAST, y_advance))
    o.append("")
    return "\n".join(o)


if __name__ == "__main__":
    ttf, em, name = sys.argv[1], int(sys.argv[2]), sys.argv[3]
    bm, gl, ya = build(ttf, em, name)
    src = "%s at %d px/em" % (ttf.split("/")[-1], em)
    sys.stdout.write(emit(name, bm, gl, ya, src))
    sys.stderr.write("%s: %d bytes of bitmap, yAdvance %d, digit box %dx%d\n"
                     % (name, len(bm), ya,
                        gl[ord("8") - FIRST]["w"], gl[ord("8") - FIRST]["h"]))
