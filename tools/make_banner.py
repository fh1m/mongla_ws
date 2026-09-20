#!/usr/bin/env python3
"""Compose the README banner from the real CAD render and the real palette.

The previous banner was an illustration of a vehicle that does not exist, with
the retired project name set into the artwork. This one uses the hull as built,
rendered by tools/pack_hull.py + the site's viewer, and every colour is the
palette sampled from the CAD's own materials.

Sized 2400x760. GitHub renders a README on a near-black page, so the canvas is
black with no border of its own — it sits on the page rather than on top of it.

    python3 tools/make_banner.py
"""
import pathlib, sys
from PIL import Image, ImageDraw, ImageFont

REPO = pathlib.Path(__file__).resolve().parent.parent
W, H = 2400, 760

VOID = (0, 0, 0)
INK = (226, 230, 236)
INK2 = (155, 163, 177)
INK3 = (120, 130, 148)
RED = (255, 0, 0)
RED_LIT = (255, 87, 71)
OCEAN = (0, 78, 255)
OCEAN_LIT = (90, 141, 255)
RULE = (28, 32, 41)

F_DISPLAY = "/usr/local/share/fonts/TTF/IosevkaNerdFont-ExtraBold.ttf"
F_MONO = "/usr/local/share/fonts/noto/NotoSansMono-Regular.ttf"
F_SERIF = "/usr/local/share/fonts/noto/NotoSerifDisplay-MediumItalic.ttf"


def font(path, size):
    for p in (path, "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"):
        try:
            return ImageFont.truetype(p, size)
        except OSError:
            continue
    return ImageFont.load_default()


def tracked(d, xy, text, f, fill, tracking=0):
    """Draw text with letter-spacing; PIL has none of its own."""
    x, y = xy
    for ch in text:
        d.text((x, y), ch, font=f, fill=fill)
        x += d.textlength(ch, font=f) + tracking
    return x - xy[0]


def tracked_width(d, text, f, tracking=0):
    return sum(d.textlength(c, font=f) + tracking for c in text) - tracking


def main():
    img = Image.new("RGB", (W, H), VOID)

    # a shallow column of water behind everything, brighter at the surface
    grad = Image.new("L", (1, H))
    for y in range(H):
        t = y / H
        grad.putpixel((0, y), int(26 * (1 - t) ** 2))
    wash = Image.merge("RGB", [grad.resize((W, H)).point(lambda v: int(v * c / 26))
                               for c in (2, 10, 34)])
    img = Image.blend(img, wash, 1.0)

    # the hull, as built, bleeding off the right edge
    render = REPO / "docs/imgs/cad/mongla-quarter-aft.webp"
    if render.exists():
        hull = Image.open(render).convert("RGBA")
        scale = (H * 0.86) / hull.height
        hull = hull.resize((int(hull.width * scale), int(hull.height * scale)), Image.LANCZOS)
        img.paste(hull, (W - hull.width + 120, int(H * 0.50 - hull.height * 0.52)), hull)
    else:
        print(f"missing {render}", file=sys.stderr)

    # a gradient scrim, not a rectangle: a hard edge at the boundary reads as a
    # seam across the whole banner, which is exactly what the first pass did
    scrim = Image.new("L", (W, 1))
    for x in range(W):
        t = min(max((x - 620) / 700.0, 0.0), 1.0)
        scrim.putpixel((x, 0), int(238 * (1 - t) ** 1.5))
    img.paste(Image.new("RGB", (W, H), VOID), (0, 0), scrim.resize((W, H)))

    d = ImageDraw.Draw(img, "RGBA")

    pad = 96
    f_word = font(F_DISPLAY, 128)
    f_tag = font(F_SERIF, 40)
    f_lbl = font(F_MONO, 21)
    f_num = font(F_MONO, 33)
    f_sub = font(F_MONO, 20)

    # corner registration ticks — the sheet knows it is a sheet
    for cx, cy, dx, dy in ((pad, 52, 1, 1), (W - pad, 52, -1, 1),
                           (pad, H - 52, 1, -1), (W - pad, H - 52, -1, -1)):
        d.line([cx, cy, cx + 20 * dx, cy], fill=RULE, width=2)
        d.line([cx, cy, cx, cy + 20 * dy], fill=RULE, width=2)

    tracked(d, (pad, 66), "AN AUTONOMY STACK FOR AUTONOMOUS UNDERWATER VEHICLES", f_sub, INK3, 3.0)

    # wordmark, with the hull's red as the live indicator
    wy = 210
    d.ellipse([pad, wy + 58, pad + 20, wy + 78], fill=RED)
    tracked(d, (pad + 46, wy), "MONGLA", f_word, INK, 13.0)

    d.text((pad + 4, wy + 176), "Machines that have to work when nobody is watching.",
           font=f_tag, fill=INK2)

    # the numbers, each one measured
    readout = [("500 Hz", "control loop"), ("18.0 ms", "photon to detection"),
               ("53.9 Hz", "through ROS"), ("46.7°", "field of view, in water"),
               ("3 311", "tests passing")]
    y0 = 520
    d.line([pad, y0 - 30, 1104, y0 - 30], fill=RULE, width=2)
    x = pad
    for i, (num, lab) in enumerate(readout):
        col = RED_LIT if i == 0 else INK
        d.text((x, y0), num, font=f_num, fill=col)
        tracked(d, (x, y0 + 52), lab.upper(), f_lbl, INK3, 1.6)
        x += max(d.textlength(num, font=f_num),
                 tracked_width(d, lab.upper(), f_lbl, 1.6)) + 56

    tracked(d, (pad, H - 92), "MUHAMMAD FAHIM FAISAL", f_sub, INK3, 3.0)
    w1 = tracked_width(d, "MUHAMMAD FAHIM FAISAL", f_sub, 3.0)
    tracked(d, (pad + w1 + 26, H - 92), "//", f_sub, RED_LIT, 3.0)
    tracked(d, (pad + w1 + 74, H - 92), "FIRMWARE, RAKIBUL ISLAM", f_sub, INK3, 3.0)

    # one ocean hairline along the foot: the medium, under everything
    d.line([0, H - 3, W, H - 3], fill=OCEAN, width=3)

    out = REPO / "docs/imgs/mongla-banner.png"
    img.save(out, "PNG", optimize=True)
    print(f"wrote {out.relative_to(REPO)}  {out.stat().st_size // 1024} KB  {W}x{H}")


if __name__ == "__main__":
    main()
