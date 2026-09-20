#!/usr/bin/env python3

"""Vendor the task images RoboNation actually prints.

WHY NOT THE FONT
----------------
These props were drawing their emoji by striking `NotoColorEmoji.ttf`, a CBDT
BITMAP font with a SINGLE 109 px strike -- `truetype(..., 109)` is the only size
that loads, so every glyph was rendered at 109 px and resampled UP to fill a
256 px placard or a 1024 px board. That is most of why the artwork read as soft
and cartoonish next to a photograph. On top of it, Noto's droplet is BLUE, so
the code re-tinted it by collapsing the glyph to luminance and multiplying by a
magenta -- which throws away the shading that makes the image legible and
leaves a flat pink blob.

RoboNation prints **Microsoft Fluent 3D** emoji: the gradient flame and the
smooth magenta teardrop on the Task 3 and Task 4 slides are that set, not Noto's
and not Apple's. They are MIT licensed, 256x256 RGBA, and already the right
colour -- so the hue-shift hack disappears with them.

This script vendors them ONCE, into the repo, so a checkout builds the same
props with no network. Re-run it only to refresh or add a glyph.
"""

from __future__ import annotations

import os
import sys
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(os.path.dirname(HERE), "models", "robosub_textures", "emoji")

RAW = ("https://raw.githubusercontent.com/microsoft/fluentui-emoji/main/"
       "assets/{asset}/3D/{stem}_3d.png")

# our name -> (Fluent asset directory, Fluent file stem)
#
# The eight the two rulebooks between them call for: four for the RoboSub role
# pairs (Survey & Repair = fire + fire engine, Search & Rescue = droplet +
# ambulance) and four for the gate and octagon role markers.
GLYPHS = {
    "fire":        ("Fire", "fire"),
    # DROP OF BLOOD, not Droplet. The rulebook says ":drop_of_blood:" and
    # RoboNation prints the magenta teardrop; Fluent's "Droplet" is the BLUE
    # water drop. Rendered once with the wrong one and it was obvious -- a blue
    # drop would also have made the sim task EASIER than the pool task, because
    # blue-vs-orange separates far more cleanly than magenta-vs-orange.
    "droplet":     ("Drop of blood", "drop_of_blood"),
    "ambulance":   ("Ambulance", "ambulance"),
    "fire_engine": ("Fire engine", "fire_engine"),
    "compass":     ("Compass", "compass"),
    "hammer_pick": ("Hammer and pick", "hammer_and_pick"),
    "ring_buoy":   ("Ring buoy", "ring_buoy"),
    "sos":         ("Sos button", "sos_button"),
}

LICENSE = """Microsoft Fluent Emoji -- 3D style
https://github.com/microsoft/fluentui-emoji
Copyright (c) Microsoft Corporation.  Licensed under the MIT License.

Vendored by scripts/fetch_emoji.py. These are the images RoboNation prints on
the RoboSub task props; see that script for why the emoji FONT is not used.
"""


def main() -> int:
    os.makedirs(OUT, exist_ok=True)
    with open(os.path.join(OUT, "LICENSE"), "w") as fh:
        fh.write(LICENSE)

    failed = []
    for name, (asset, stem) in sorted(GLYPHS.items()):
        url = RAW.format(asset=asset.replace(" ", "%20"), stem=stem)
        dest = os.path.join(OUT, f"{name}.png")
        try:
            with urllib.request.urlopen(url, timeout=30) as r:
                data = r.read()
            if not data.startswith(b"\x89PNG"):
                raise ValueError("not a PNG")
            with open(dest, "wb") as fh:
                fh.write(data)
            print(f"wrote {dest}  ({len(data)} B)")
        except Exception as exc:                            # noqa: BLE001
            print(f"  FAILED {name}: {exc}", file=sys.stderr)
            failed.append(name)
    if failed:
        print(f"\n{len(failed)} glyph(s) missing: {', '.join(failed)}. The "
              "texture generator falls back to the Noto font for those and "
              "says so per glyph.", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
