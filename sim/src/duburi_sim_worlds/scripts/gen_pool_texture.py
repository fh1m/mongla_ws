#!/usr/bin/env python3

"""Generate the pool floor and wall tile textures from spec/arena.yaml.

Produces PNGs into models/sauvc_textures/:

    pool_floor.png       the full 25 x 16 m floor, with lane lines and crosses
    pool_wall_long.png   the 25 m x depth walls
    pool_wall_short.png  the 16 m x depth walls
    gate_stripe_orange.png / gate_stripe_red.png / gate_stripe_green.png
    drum_rim.png         banded drum rim cue

Every texture is drawn at the true aspect ratio of the surface it covers. A box
face in Gazebo maps its texture exactly once, so a tileable patch stretched over
a 25 m wall would render metre-wide tiles. Sizing each image to its own surface
keeps the tile pitch identical everywhere.

Floor and walls are also drawn from one palette on purpose: Gazebo sRGB-decodes
texture images but not solid <diffuse> colours, so a solid-coloured wall beside a
textured floor never matches however carefully the numbers are picked.

Usage:
    scripts/gen_pool_texture.py [--spec ...] [--outdir ...]
"""

import argparse
import math
import os

import numpy as np
import yaml
from PIL import Image

import prop_library as pl

# Palette, linear 0-1. Competition pool: pale blue mosaic tiles, white grout,
# navy lane lines, black cross markers.
FIELD = np.array([0.58, 0.80, 0.90])
# Grout was near-white against a mid-blue field -- the single highest-contrast
# edge in the scene, repeated every few pixels. Real grout is a shade of the
# tile, not a highlight.
GROUT = np.array([0.72, 0.84, 0.90])
LANE = np.array([0.04, 0.10, 0.40])
CROSS = np.array([0.02, 0.02, 0.03])

# 0.25 m, not 0.10. At the spec's 64 px/m a 0.10 m tile is SIX pixels, and a
# six-pixel tile carrying per-tile random colour is a high-frequency field: seen
# from a few metres away it minifies into the chaotic dash pattern that made the
# pool floor read as static rather than tiles. Real competition pools use large
# floor mosaic anyway. The aliasing fix is fewer, bigger tiles plus much less
# per-tile variance -- not a bigger texture, which only moves the beat frequency.
TILE_M = 0.25  # tile pitch in metres
GROUT_PX = 2  # ~3 cm at 64 px/m with the 0.25 m tile: a believable joint

# Competition pools mark lanes every 2.5 m across the width.
LANE_SPACING_M = 2.5
LANE_WIDTH_M = 0.30
CROSS_WIDTH_M = 0.28
CROSS_LENGTH_M = 1.1
CROSS_INSET_M = 2.0

# Per-tile mosaic variation (std of additive RGB noise).
# Subtle. At 0.035 the field read as confetti once tiles were small; the point
# of per-tile noise is to break up a flat plane, not to be seen.
TILE_NOISE = 0.014

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
DEFAULT_SPEC = os.path.join(PKG, "spec", "arena.yaml")
def outdir_for(competition: str) -> str:
    """models/<competition>_textures. Per competition because the floor and wall
    PNGs are sized from that competition's pool -- one shared set stretched over
    a differently-shaped pool is wrong with no error anywhere."""
    return os.path.join(PKG, "models", f"{competition}_textures")


DEFAULT_OUTDIR = outdir_for("sauvc")


def _grout_mask(height: int, width: int, tile_px: int) -> np.ndarray:
    """True where a pixel falls on a grout line between tiles."""
    rows = (np.arange(height) % tile_px) < GROUT_PX
    cols = (np.arange(width) % tile_px) < GROUT_PX
    return rows[:, None] | cols[None, :]


def _mosaic_field(height: int, width: int, tile_px: int, rng: np.random.Generator):
    """Pale-blue mosaic with per-tile colour noise (not per-pixel grain)."""
    n_rows = (height + tile_px - 1) // tile_px
    n_cols = (width + tile_px - 1) // tile_px
    tile_colours = FIELD + rng.normal(0.0, TILE_NOISE, size=(n_rows, n_cols, 3))
    # Occasional slightly greener / darker mosaic chips.
    chip = rng.random((n_rows, n_cols)) < 0.08
    tile_colours[chip] *= np.array([0.92, 1.02, 0.98])
    tile_colours = np.clip(tile_colours, 0.0, 1.0)

    row_idx = np.arange(height) // tile_px
    col_idx = np.arange(width) // tile_px
    return tile_colours[row_idx[:, None], col_idx[None, :]]


def _tiled(width_m: float, height_m: float, px_per_m: int, rng: np.random.Generator):
    """Return a mosaic tiled field of the given physical size, plus tile pitch."""
    width = max(1, int(round(width_m * px_per_m)))
    height = max(1, int(round(height_m * px_per_m)))
    tile_px = max(4, int(round(TILE_M * px_per_m)))
    img = _mosaic_field(height, width, tile_px, rng)
    return img, tile_px


def _save(img: np.ndarray, tile_px: int, path: str) -> None:
    img = img.copy()
    img[_grout_mask(img.shape[0], img.shape[1], tile_px)] = GROUT
    Image.fromarray((np.clip(img, 0, 1) * 255).astype(np.uint8)).save(
        path, optimize=True
    )
    print(f"wrote {path}  ({img.shape[1]}x{img.shape[0]})")



def _caustics(height: int, width: int, px_per_m: float, rng, strength: float = 0.16):
    """A baked caustic light field, returned as a multiplicative [1-s, 1+s] map.

    The bright wobbling net cast on a pool floor by the rippling surface is the
    single strongest "this is underwater" cue a still frame has -- stronger than
    fog, stronger than colour. Gazebo will not compute it: real caustics need
    photon transport from an animated surface, and the wave shader other teams
    use only deforms the surface, it does not light the floor through it.

    Baking it into the albedo costs nothing at render time and is honest for a
    fixed-camera dataset: the pattern does not animate, but a detector trained
    on frames from this pool sees floor texture with the right spatial
    statistics instead of a flat plane.

    Built the way caustics actually form: the surface is a sum of a few
    travelling waves, and light focuses where that surface is CONCAVE. So take
    the Laplacian of the wave field and keep the positive part -- sharp bright
    filaments over a dim background, which is what the eye recognises. A sum of
    smooth blobs would give soft mush and read as dirt.
    """
    yy, xx = np.mgrid[0:height, 0:width].astype(np.float32)
    xm, ym = xx / px_per_m, yy / px_per_m

    # Nine directions, not five, and short wavelengths. With a handful of long
    # waves the Laplacian's ridges stay parallel and the floor gets diagonal
    # BANDS -- which is wrong twice over: it does not look like a pool, and a
    # detector trained on it learns a directional texture prior. Caustics are a
    # CELLULAR net, and a net is what you get when enough directions interfere
    # at a wavelength near the tile scale.
    surface = np.zeros((height, width), dtype=np.float32)
    for _ in range(9):
        ang = rng.uniform(0, 2 * np.pi)
        k = 2 * np.pi / rng.uniform(0.22, 0.65)         # 22-65 cm wavelengths
        # Amplitude 1/k^2, NOT 1/k. The Laplacian scales a wave by k^2, so
        # with 1/k the shortest wave survives at k times the others and its
        # single direction becomes the whole pattern -- that is what produced
        # diagonal streaks instead of a net. 1/k^2 makes every direction
        # contribute equally AFTER the Laplacian, which is what lets nine of
        # them interfere into cells.
        surface += np.sin(k * (np.cos(ang) * xm + np.sin(ang) * ym)
                          + rng.uniform(0, 2 * np.pi)) / (k * k)

    lap = (np.roll(surface, 1, 0) + np.roll(surface, -1, 0)
           + np.roll(surface, 1, 1) + np.roll(surface, -1, 1) - 4.0 * surface)
    lap = np.maximum(lap, 0.0)
    peak = np.percentile(lap, 99.5)
    if peak <= 0:
        return np.ones((height, width, 1), dtype=np.float32)
    net = np.clip(lap / peak, 0.0, 1.0) ** 0.7

    # Bright filaments pull up hard, the shadowed background dips slightly --
    # net light is conserved, so the floor does not simply get brighter.
    return (1.0 + strength * (2.2 * net - 0.45))[:, :, None].astype(np.float32)


def make_floor(
    length_m: float, width_m: float, px_per_m: int, path: str, rng: np.random.Generator
) -> None:
    """Draw the pool floor: mosaic tiles, lane lines, cross markers."""
    img, tile_px = _tiled(length_m, width_m, px_per_m, rng)
    height, width = img.shape[:2]

    lane_half = LANE_WIDTH_M * px_per_m / 2.0
    y = np.arange(height)
    lane_mask = np.zeros(height, dtype=bool)
    lane_centres = []
    for i in range(1, int(width_m / LANE_SPACING_M)):
        centre = i * LANE_SPACING_M * px_per_m
        lane_centres.append(centre)
        lane_mask |= np.abs(y - centre) <= lane_half
    img[lane_mask, :] = LANE

    cross_half_w = CROSS_WIDTH_M * px_per_m / 2.0
    cross_half_l = CROSS_LENGTH_M * px_per_m / 2.0
    x = np.arange(width)
    for centre in lane_centres:
        band = np.abs(y - centre) <= cross_half_l
        for cx in (CROSS_INSET_M * px_per_m, width - CROSS_INSET_M * px_per_m):
            img[np.ix_(band, np.abs(x - cx) <= cross_half_w)] = CROSS

    # AFTER the lane lines and crosses, so the light plays over the markings
    # too. Caustics that stop at a painted line look like a decal, not light.
    img = np.clip(img * _caustics(height, width, px_per_m, rng), 0.0, 1.0)

    _save(img, tile_px, path)


def make_wall(
    span_m: float,
    depth_m: float,
    px_per_m: int,
    path: str,
    rng: np.random.Generator,
) -> None:
    """Draw a tiled wall with a subtle depth-band tint (darker toward the floor)."""
    img, tile_px = _tiled(span_m, depth_m, px_per_m, rng)
    height = img.shape[0]
    # Image y=0 is the top of the wall (surface); y=height is the floor.
    depth_t = np.linspace(0.0, 1.0, height)[:, None, None]
    # Cooler / slightly darker near the floor so walls do not look like a flat wash.
    tint = 1.0 - 0.18 * depth_t
    cool = np.array([0.97, 0.99, 1.03])
    img = np.clip(img * tint * cool, 0.0, 1.0)
    _save(img, tile_px, path)


def make_stripe(
    path: str,
    colour,
    *,
    post_m: float = 1.00,
    band_m: float = 0.20,
    gap_m: float = 0.10,
    foot_m: float = 0.05,
    dark=(0.05, 0.05, 0.06),
    foot=(0.93, 0.93, 0.93),
    width: int = 128,
    px_per_m: int = 512,
    rng=None,
) -> None:
    """Gate-post stripe map, driven by METRES rather than a band count.

    The rulebook figure is explicit: ~20 cm of colour alternating with ~10 cm of
    BLACK up a 100 cm post, with a white foot. The previous version alternated
    the colour with WHITE at a fixed 8 bands, which is neither the right colour
    nor the right pitch -- a detector trained on it learns the wrong target.

    V runs along the post, so row 0 is the TOP of the cylinder and the foot is
    drawn last. Sized from px_per_m so a 1 m post and a 1.6 m post get the same
    physical stripe pitch instead of the same number of stripes.
    """
    rng = rng or np.random.default_rng(0)
    height = max(8, int(round(post_m * px_per_m)))
    img = np.zeros((height, width, 3), dtype=np.float64)
    img[:, :] = dark

    # Lay bands from the BOTTOM up, above the foot, so the foot is always a
    # whole band and the truncation (if any) lands at the top where the gate
    # corner is anyway.
    y = height - int(round(foot_m * px_per_m))
    img[y:, :] = foot
    band_px = int(round(band_m * px_per_m))
    gap_px = int(round(gap_m * px_per_m))
    while y > 0:
        y0 = max(0, y - band_px)
        img[y0:y, :] = colour
        y = y0 - gap_px          # the gap stays `dark`, already filled

    # PVC is not flat plastic underwater: a little vertical shading stops the
    # post reading as a paper cut-out and gives the detector some gradient.
    shade = 1.0 - 0.10 * np.linspace(0.0, 1.0, width)[None, :, None]
    img *= shade
    img += rng.normal(0.0, 0.012, img.shape)

    Image.fromarray((np.clip(img, 0, 1) * 255).astype(np.uint8)).save(
        path, optimize=True
    )
    print(f"wrote {path}  ({width}x{height})")


def make_water_surface(path: str, length_m: float, width_m: float,
                      px_per_m: int = 24, rng=None) -> None:
    """Underside of the water surface: overlapping ripples, seen from below.

    Gazebo has no water. The pool is a floor, four walls and a fog volume, so
    everything above z=0 was raw <sky> and the surface read as a hard edge
    between "pool" and "sky" -- which is exactly what the competition photos do
    NOT look like. A translucent rippled plane at z=0 is what turns the sky into
    something seen THROUGH water.

    Bright, because from below the surface is the brightest thing in the scene
    (it is where all the light comes in) and the fog then knocks it back.
    """
    rng = rng or np.random.default_rng(0)
    h = max(64, int(round(width_m * px_per_m)))
    w = max(64, int(round(length_m * px_per_m)))
    yy, xx = np.mgrid[0:h, 0:w].astype(np.float64)

    # A few sine trains at different angles and wavelengths read as wind chop
    # far better than one, and tile without an obvious repeat.
    field = np.zeros((h, w))
    for wavelength, angle, amp in ((11.0, 0.3, 1.0), (23.0, 1.9, 0.7),
                                   (7.0, 2.7, 0.45), (37.0, 0.9, 0.5)):
        k = 2.0 * np.pi / wavelength
        field += amp * np.sin(k * (xx * np.cos(angle) + yy * np.sin(angle)))
    field /= 2.65
    field += rng.normal(0.0, 0.05, field.shape)

    base = np.array([0.72, 0.86, 0.93])
    img = base[None, None, :] * (0.80 + 0.20 * field)[:, :, None]
    Image.fromarray((np.clip(img, 0, 1) * 255).astype(np.uint8)).save(
        path, optimize=True)
    print(f"wrote {path}  ({w}x{h})")


def make_roughness(path: str, base: float, width: int = 128, height: int = 512,
                   variation: float = 0.08, rng=None) -> None:
    """Greyscale roughness map. Uniform gloss is what makes a prop look CG."""
    rng = rng or np.random.default_rng(0)
    v = np.clip(base + rng.normal(0.0, variation, (height, width)), 0.0, 1.0)
    img = np.repeat((v * 255).astype(np.uint8)[:, :, None], 3, axis=2)
    Image.fromarray(img).save(path, optimize=True)
    print(f"wrote {path}  ({width}x{height})")


def make_normal_map(path: str, height, strength: float = 2.0) -> None:
    """Tangent-space normal map from a height field.

    NOTHING in this tree used a normal map before, and it is the clearest single
    tell that a prop is CG: a perfectly smooth surface with its detail painted
    into the albedo. Painted detail does not move when the light moves, and it
    does not survive a viewpoint change -- which is exactly what breaks feature
    matching, since descriptors key on local gradients.

    Standard encoding: n = normalize(-dh/dx, -dh/dy, 1/strength), mapped from
    [-1,1] to [0,255]. A flat surface is the familiar (128, 128, 255).
    """
    h = np.asarray(height, dtype=np.float32)
    dx = np.gradient(h, axis=1) * strength
    dy = np.gradient(h, axis=0) * strength
    nz = np.ones_like(h)
    inv = 1.0 / np.sqrt(dx * dx + dy * dy + nz * nz)
    rgb = np.stack([-dx * inv, -dy * inv, nz * inv], axis=2)
    img = ((rgb * 0.5 + 0.5) * 255.0).clip(0, 255).astype(np.uint8)
    Image.fromarray(img).save(path, optimize=True)
    print(f"wrote {path}  ({h.shape[1]}x{h.shape[0]} normal)")


def _pvc_height(width: int, height: int, rng) -> np.ndarray:
    """Extrusion seams and the faint die lines that run along a PVC tube."""
    yy, xx = np.mgrid[0:height, 0:width].astype(np.float32)
    h = 0.45 * np.sin(2.0 * np.pi * xx / (width / 3.0))
    h += 0.18 * np.sin(2.0 * np.pi * xx / 7.0)
    h += rng.normal(0.0, 0.06, (height, width)).astype(np.float32)
    return h


def _plastic_height(width: int, height: int, rng) -> np.ndarray:
    """Moulded-plastic pebbling, plus the odd scuff."""
    coarse = rng.normal(0.0, 1.0, (height // 8, width // 8)).astype(np.float32)
    h = np.asarray(Image.fromarray(
        ((coarse - coarse.min()) / max(1e-6, coarse.ptp()) * 255).astype(np.uint8)
    ).resize((width, height), Image.BICUBIC), dtype=np.float32) / 255.0
    h = 0.9 * h + 0.15 * rng.normal(0.0, 1.0, (height, width)).astype(np.float32)
    return h


def _fabric_height(width: int, height: int, rng) -> np.ndarray:
    """Woven flare fabric: a weave in both directions."""
    yy, xx = np.mgrid[0:height, 0:width].astype(np.float32)
    h = 0.35 * np.sin(2.0 * np.pi * xx / 5.0) + 0.35 * np.sin(2.0 * np.pi * yy / 5.0)
    return h + rng.normal(0.0, 0.07, (height, width)).astype(np.float32)


def make_flare_fabric(path: str, colour, width: int = 128, height: int = 640,
                      rng=None) -> None:
    """The orange flare is an inflated fabric tube, not a painted pipe.

    Vertical weave plus a soft cylindrical shading ramp, so the silhouette reads
    round under the flat pool lighting.
    """
    rng = rng or np.random.default_rng(0)
    img = np.zeros((height, width, 3), dtype=np.float64)
    img[:, :] = colour
    weave = 1.0 + 0.05 * np.sin(np.arange(width) * np.pi / 3.0)[None, :, None]
    img *= weave
    # cylinder shading: bright down the middle, falling to both edges
    u = np.linspace(-1.0, 1.0, width)
    img *= (1.0 - 0.35 * (u ** 2))[None, :, None]
    img += rng.normal(0.0, 0.015, img.shape)
    Image.fromarray((np.clip(img, 0, 1) * 255).astype(np.uint8)).save(
        path, optimize=True)
    print(f"wrote {path}  ({width}x{height})")


def make_drum_wall(path: str, colour, width: int = 512, height: int = 256,
                   rng=None) -> None:
    """Moulded plastic drum wall: vertical ribs, scuffs, a darker rim band."""
    rng = rng or np.random.default_rng(0)
    img = np.zeros((height, width, 3), dtype=np.float64)
    img[:, :] = colour
    ribs = 1.0 + 0.06 * np.sin(np.arange(width) * 2.0 * np.pi / 26.0)[None, :, None]
    img *= ribs
    img *= (1.0 - 0.22 * np.linspace(0.0, 1.0, height) ** 2)[:, None, None]
    scuff = rng.random((height, width)) > 0.994
    img[scuff] *= 1.25
    img += rng.normal(0.0, 0.014, img.shape)
    Image.fromarray((np.clip(img, 0, 1) * 255).astype(np.uint8)).save(
        path, optimize=True)
    print(f"wrote {path}  ({width}x{height})")


def make_drum_rim(path: str, width: int = 256, height: int = 64) -> None:
    """Horizontal banded rim: dark shell with a bright highlight stripe."""
    img = np.zeros((height, width, 3), dtype=np.float64)
    img[:] = (0.08, 0.08, 0.09)
    # Mid band highlight.
    y0, y1 = int(0.35 * height), int(0.65 * height)
    img[y0:y1, :] = (0.55, 0.55, 0.58)
    # Thin top lip.
    img[: max(2, height // 10), :] = (0.75, 0.75, 0.78)
    Image.fromarray((np.clip(img, 0, 1) * 255).astype(np.uint8)).save(
        path, optimize=True
    )
    print(f"wrote {path}  ({width}x{height})")



# ---------------------------------------------------------------------------
# RoboSub role imagery
# ---------------------------------------------------------------------------

# RoboNation prints the task images as emoji on 12x12 in vinyl. Rendering the
# ACTUAL glyphs -- rather than an approximation of them -- is the whole point:
# every RoboSub task after the gate scores on matching the role the vehicle
# chose, so the discrimination a detector has to learn is fire-vs-droplet, and
# it should be learning it from the real artwork.
NOTO_EMOJI = "/usr/local/share/fonts/noto/NotoColorEmoji.ttf"

# Only the Noto FALLBACK needs this: Noto's droplet is blue where RoboNation's
# is magenta. The vendored Fluent droplet is already the right colour, so the
# intended path never re-tints -- which matters, because re-tinting collapses
# the glyph to luminance and destroys the shading a detector reads.
MAGENTA_DROPLET = (1.45, 0.28, 0.80)

# Codepoints for the FALLBACK path only -- the vendored PNGs are the
# intended source. Keyed by the name used in spec/robosub.yaml.
ROLE_GLYPHS = {
    "fire": "\U0001F525",          # Survey & Repair    (bins, torpedo)
    "droplet": "\U0001F4A7",       # Search & Rescue    (bins, torpedo)
    "compass": "\U0001F9ED",       # Survey & Repair    (gate, octagon)
    "hammer_pick": "\U00002692",   # Survey & Repair    (gate, octagon)
    "sos": "\U0001F198",           # Search & Rescue    (gate, octagon)
    "ring_buoy": "\U0001F6DF",     # Search & Rescue    (gate, octagon)
    "ambulance": "\U0001F691",     # Search & Rescue    (torpedo)
    "fire_engine": "\U0001F692",   # Survey & Repair    (torpedo)
}



EMOJI_DIR = os.path.join(os.path.dirname(HERE), "models", "robosub_textures",
                         "emoji")


def _glyph_rgba(name, size=136, hue_shift=None):
    """One task image as RGBA at `size`.

    Loads the VENDORED Microsoft Fluent 3D PNG -- the set RoboNation actually
    prints -- and falls back, loudly, to striking the Noto emoji font.

    The font path is a fallback and not the intent, for two measured reasons.
    NotoColorEmoji is a CBDT BITMAP face with a SINGLE 109 px strike, so every
    glyph was drawn at 109 px and resampled up to fill a 256 px placard or a
    1024 px board -- soft by construction. And Noto's droplet is BLUE where
    RoboNation's is magenta, so the old code collapsed the glyph to luminance
    and multiplied by a tint, throwing away the shading that makes it legible
    and leaving a flat pink blob. The Fluent droplet is already magenta, so
    `hue_shift` is dead weight on that path and is accepted only to keep the
    fallback honest.
    """
    from PIL import ImageDraw, ImageFont

    asset = os.path.join(EMOJI_DIR, f"{name}.png")
    if os.path.isfile(asset):
        img = Image.open(asset).convert("RGBA")
        # LANCZOS both ways: these are 256 px renders, so a placard upsamples
        # slightly and a board glyph downsamples. Neither is a font strike.
        return img.resize((size, size), Image.LANCZOS)

    print(f"  WARNING: no vendored artwork for {name!r} -- falling back to the "
          f"Noto font's 109 px strike. Run scripts/fetch_emoji.py.")
    glyph = ROLE_GLYPHS.get(name, "?")
    font = ImageFont.truetype(NOTO_EMOJI, 109)   # the font's only strike
    img = Image.new("RGBA", (size, size), (255, 255, 255, 0))
    ImageDraw.Draw(img).text((size // 2, size // 2), glyph, font=font,
                             embedded_color=True, anchor="mm")
    if hue_shift is not None:
        a = np.asarray(img).astype(np.float32)
        rgb, alpha = a[..., :3], a[..., 3:]
        lum = rgb.mean(axis=2, keepdims=True)
        a[..., :3] = np.clip(lum * np.array(hue_shift, dtype=np.float32), 0, 255)
        img = Image.fromarray(
            np.concatenate([a[..., :3], alpha], axis=2).astype(np.uint8), "RGBA")
    return img


def make_role_image(path, name, px=512, border=0.055, rng=None,
                    hue_shift=None):
    """A printed vinyl role sign: one task image, centred on white, thin edge.

    512 px, not 256. The artwork is now a vendored 256 px Fluent render rather
    than a 109 px font strike, so the placard can carry it at close to its own
    resolution instead of throwing half of it away.

    Falls back to a flat grey square with a loud warning rather than aborting
    the whole asset build. A missing sign then looks obviously wrong in the
    render instead of silently looking like a legitimately blank panel.
    """
    img = Image.new("RGB", (px, px), (255, 255, 255))
    try:
        inner = int(px * (1.0 - 2.2 * border))
        g = _glyph_rgba(name, size=inner, hue_shift=hue_shift)
        img.paste(g, ((px - inner) // 2, (px - inner) // 2), g)
    except Exception as exc:                      # noqa: BLE001 - see docstring
        print(f"  WARNING: no emoji glyph for {os.path.basename(path)} ({exc}); "
              f"writing a blank panel. Install fonts-noto-color-emoji.")
        img = Image.new("RGB", (px, px), (170, 170, 170))

    # The printed signs have a dark border; it also gives a detector an edge to
    # find when the glyph itself washes out in turbid water.
    a = np.asarray(img).astype(np.float32) / 255.0
    b = max(1, int(px * border * 0.45))
    a[:b, :], a[-b:, :], a[:, :b], a[:, -b:] = 0.12, 0.12, 0.12, 0.12
    Image.fromarray((np.clip(a, 0, 1) * 255).astype(np.uint8)).save(
        path, optimize=True)
    print(f"wrote {path}  ({px}x{px})")


def make_torpedo_panel(path, role, spec, px=1024, rng=None):
    """The Task 4 board face: FOUR openings and the FOUR role images, 2026.

    THE OPENINGS ARE NOT DEFINED HERE. They come from
    `prop_library.torpedo_openings_uv`, the same list the collision plate is
    tiled around and the visual mesh is cut from. They used to be written out
    twice and had drifted badly -- this function painted FOUR circles while the
    collision cut TWO, in different places, so a shot lined up on the artwork
    struck solid board and nothing in any log said why.

    2026 layout, from the TeamTime "Task 4 - Deploy (Torpedoes)" slide: a 2x2 of
    image+opening cells, two large openings and two small, with all four
    emergency images (fire, fire engine, ambulance, blood drop) on BOTH boards.
    Only which image sits beside which opening size differs between the two
    versions, so a detector cannot separate the roles by image presence alone --
    which is the discrimination the real task demands.

    WHAT CHANGED, AND WHY IT MATTERS FOR A DATASET
    This was the one prop in the library with no surface treatment at all: a
    flat `np.ones` field, boolean-mask circles with stair-stepped edges, and the
    `rng` it accepts ignored. Every other prop already had weave, ribbing,
    scuffs or noise. Clip art on a white field is not something a detector can
    learn a real board from, and the openings above were the larger half of the
    same problem. So: supersample for real edges, corrugated-plastic ribbing
    (the backing the rulebook names), print bleed and wear.

    Resolution was never the bottleneck -- 512 px over 0.6 m was already 13x the
    pool floor -- but it is raised to 1024 because four images and four rims now
    share the face where two did.
    """
    rng = rng or np.random.default_rng(2026)
    cfg = spec["props"]["torpedo_board"]
    ss = 3                                          # supersample factor
    big = px * ss
    # GREY, from the spec. `torpedo_board.colour` had been declared since the
    # prop was written and never read by anything -- the panel was white purely
    # because this line used to be `np.ones`. The CAD's panel is grey, and a
    # white board against a pale pool is a much easier detection than the real
    # one.
    img = np.ones((big, big, 3), dtype=np.float32) * np.array(
        cfg["colour"], dtype=np.float32)
    yy, xx = np.mgrid[0:big, 0:big].astype(np.float32)

    # --- corrugated plastic backing -------------------------------------
    # The rulebook calls the board "corrugated plastic backing with vinyl
    # print". Flutes run vertically; the print sits on top of them, so the
    # ribbing shows through as a faint periodic shading rather than as colour.
    flute_px = big / 60.0
    ribs = 0.020 * np.cos(2.0 * np.pi * xx / flute_px)
    img += ribs[:, :, None]

    # Vinyl print is not perfectly white and does not stay clean in a pool.
    # Low-frequency blotching, via PIL rather than scipy: this tree has no
    # scipy dependency and adding one for a blur would be the wrong trade.
    coarse = rng.normal(0.5, 0.16, (48, 48)).astype(np.float32)
    grime = np.asarray(Image.fromarray(
        (np.clip(coarse, 0, 1) * 255).astype(np.uint8)
    ).resize((big, big), Image.BICUBIC), dtype=np.float32) / 255.0
    img *= (1.0 - 0.055 * grime)[:, :, None]
    img += rng.normal(0.0, 0.004, (big, big, 1)).astype(np.float32)

    holes = pl.torpedo_openings_uv(spec)

    # --- openings -------------------------------------------------------
    # Anti-aliased by construction: coverage is a smooth ramp across one
    # supersampled pixel, not a boolean mask. The old hard mask is what made
    # the circles read as stair-stepped clip art.
    for cx, cy, r in holes:
        d = np.sqrt((xx / big - cx) ** 2 + (yy / big - cy) ** 2)
        e = 0.6 / big
        # THE RING IS DRAWN OUTSIDE THE HOLE, NOT INSIDE IT.
        #
        # It used to span 0.70r..r -- which is exactly the material the mesh
        # CUTS AWAY, so the painted ring was applied to board that no longer
        # exists and the openings rendered as bare holes with no rim at all.
        # Widening the band only moved more of it into the void. Drawn from r
        # outward it lands on solid board and survives any overcut.
        # Band from the SPEC, not a literal: `prop_library.torpedo_layout`
        # packs the columns using this same multiple as the ring's outer
        # radius, so a hard-coded number here would let the drawn ring and
        # the reserved space drift apart -- which is the whole class of bug
        # this board keeps being rebuilt for.
        rim = (np.clip((d - r) / e, 0.0, 1.0)
               * np.clip((r * cfg["ring_band"] - d) / e, 0.0, 1.0))
        hole = np.clip((r - d) / e, 0.0, 1.0)
        img = img * (1.0 - rim[:, :, None]) + np.array(
            (0.72, 0.06, 0.09), dtype=np.float32) * rim[:, :, None]
        # The hole itself is a placeholder only: the MESH is what actually
        # opens now, so nothing is ever drawn through this. Kept dark so a
        # stale flat-plate build is obviously wrong rather than subtly wrong.
        img = img * (1.0 - hole[:, :, None]) + np.array(
            (0.05, 0.08, 0.10), dtype=np.float32) * hole[:, :, None]

    # Printed edge, so the board reads as a bounded object through fog.
    b = int(big * 0.012)
    img[:b, :] = img[-b:, :] = img[:, :b] = img[:, -b:] = (0.22, 0.22, 0.24)

    base = Image.fromarray((np.clip(img, 0, 1) * 255).astype(np.uint8))
    base = base.resize((px, px), Image.LANCZOS)

    # --- the four images ------------------------------------------------
    # Each sits in the cell DIAGONALLY OPPOSITE its own opening, so image and
    # opening pair without overlapping. Both boards carry all four; the two
    # versions differ in which image is next to the large opening.
    order = spec["roles"][role]["task_images"] + spec["roles"][
        "search_rescue" if role == "survey_repair" else "survey_repair"
    ]["task_images"]
    try:
        # PLACED BY THE PACKER, not by this file. The images alternate with the
        # openings across two rows of four -- the arrangement the CAD draws --
        # and their column centres come from `prop_library.torpedo_layout`, the
        # same call the mesh and the collision plate read. Placing them here
        # from hand-typed spec coordinates is what put an emoji inside a hole.
        for (gx, gy, frac), name in zip(pl.torpedo_images_uv(spec), order):
            side = max(1, int(px * frac))
            g = _glyph_rgba(name, size=side)
            base.paste(g, (int(gx * px - side / 2), int(gy * px - side / 2)), g)
        base.save(path, optimize=True)
    except Exception as exc:                      # noqa: BLE001
        print(f"  WARNING: torpedo panel without glyphs ({exc})")
        base.save(path, optimize=True)
    print(f"wrote {path}  ({px}x{px}, {len(holes)} openings)")


def make_surface_maps(outdir: str, rng) -> None:
    """The shared roughness + normal maps every prop material reads.

    Both competitions call this. Round 9 wrote the normal maps in the RoboSub
    branch only, so every SAUVC prop -- gates, mats, flares, drums -- kept a
    perfectly smooth surface with its detail painted on, and nothing said so:
    a missing <normal_map> is simply an absent element, not an error.
    """
    make_roughness(os.path.join(outdir, "rough_pvc.png"), 0.62, rng=rng)
    make_roughness(os.path.join(outdir, "rough_plastic.png"), 0.78, rng=rng)
    make_roughness(os.path.join(outdir, "rough_fabric.png"), 0.88, rng=rng)
    for nm, fn, w, hgt, st in (
        ("norm_pvc.png", _pvc_height, 128, 512, 6.0),
        ("norm_plastic.png", _plastic_height, 512, 256, 7.0),
        ("norm_fabric.png", _fabric_height, 128, 640, 5.0),
    ):
        make_normal_map(os.path.join(outdir, nm), fn(w, hgt, rng), strength=st)


def make_particle_sprite(path: str, size: int = 64) -> None:
    """A soft round speck for the marine-snow emitter.

    gz-sim logs "ParticleEmitter SetColorRange is currently disabled" and
    ignores <color_start>/<color_end>, so the only way to stop particles
    rendering as hard-edged white quads is to give the material a sprite whose
    ALPHA falls off to zero at the edge. The colour then comes from <diffuse>.
    """
    img = Image.new("RGBA", (size, size), (0, 0, 0, 0))
    px = img.load()
    c = (size - 1) / 2.0
    for y in range(size):
        for x in range(size):
            r = math.hypot(x - c, y - c) / c
            if r >= 1.0:
                continue
            # Smoothstep falloff, then squared so the core stays bright and the
            # halo goes soft -- a linear ramp reads as a flat disc.
            t = 1.0 - r
            a = t * t * (3.0 - 2.0 * t)
            px[x, y] = (255, 255, 255, int(255 * a * a))
    img.save(path)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--spec", default=None, help="Arena spec YAML.")
    parser.add_argument("--competition", default=None,
                        help="Only this competition (default: all).")
    parser.add_argument("--outdir", default=None, help="Output directory.")
    parser.add_argument("--seed", type=int, default=2026, help="Mosaic RNG seed.")
    args = parser.parse_args()

    comps = [args.competition] if args.competition else _competitions()
    for comp in comps:
        spec_file = args.spec or os.path.join(PKG, "spec", f"{comp}.yaml")
        with open(spec_file) as f:
            spec = yaml.safe_load(f)
        outdir = args.outdir or outdir_for(comp)
        _generate(spec, outdir, args.seed, comp)


def _competitions() -> list:
    d = os.path.join(PKG, "spec")
    return sorted(f[:-5] for f in os.listdir(d) if f.endswith(".yaml"))


def _generate(spec: dict, outdir: str, seed: int, competition: str) -> None:
    pool = spec["pool"]
    px_per_m = int(spec.get("texture_px_per_m", 64))
    os.makedirs(outdir, exist_ok=True)
    rng = np.random.default_rng(seed)
    args = type("A", (), {"outdir": outdir})()   # keeps the body below unchanged

    floor_png = os.path.join(args.outdir, "pool_floor.png")
    make_floor(pool["length"], pool["width"], px_per_m, floor_png, rng)

    # A SLOPED floor is two slabs, each only half the pool long. A box face maps
    # its texture exactly ONCE, so handing both slabs the full-length PNG
    # squeezes 25 m of floor into 12.5 m and then repeats it -- anisotropic
    # mosaic, and the two cross markers appear four times in the wrong places.
    # Slice the real floor in half so each slab shows its own half.
    if pool.get("floor_edge_depth"):
        whole = Image.open(floor_png)
        mid = whole.size[0] // 2
        whole.crop((0, 0, mid, whole.size[1])).save(
            os.path.join(args.outdir, "pool_floor_neg.png"), optimize=True)
        whole.crop((mid, 0, whole.size[0], whole.size[1])).save(
            os.path.join(args.outdir, "pool_floor_pos.png"), optimize=True)
        print(f"wrote pool_floor_{{neg,pos}}.png  ({mid}x{whole.size[1]} each)")
    make_wall(
        pool["length"],
        pool["depth"],
        px_per_m,
        os.path.join(args.outdir, "pool_wall_long.png"),
        rng,
    )
    make_wall(
        pool["width"],
        pool["depth"],
        px_per_m,
        os.path.join(args.outdir, "pool_wall_short.png"),
        rng,
    )

    make_particle_sprite(os.path.join(args.outdir, "marine_snow.png"))

    # ---- prop textures -------------------------------------------------
    # Everything above is pool geometry and applies to any competition.
    # Everything below is keyed to SAUVC prop names in the spec.
    if competition != "sauvc":
        make_surface_maps(args.outdir, rng)

        make_water_surface(
            os.path.join(args.outdir, "water_surface.png"),
            pool["length"], pool["width"], rng=rng)
        if competition == "robosub":
            for name in ROLE_GLYPHS:
                make_role_image(
                    os.path.join(args.outdir, f"role_{name}.png"), name, rng=rng)
            for role in ("survey_repair", "search_rescue"):
                make_torpedo_panel(
                    os.path.join(args.outdir, f"torpedo_panel_{role}.png"),
                    role, spec, rng=rng)
        return

    # Driven by the rulebook segment sizes in spec/<competition>.yaml, NOT by a
    # band count, so the physical stripe pitch is right on every post length.
    props = spec["props"]
    fg = props["final_gate"]
    band = float(fg.get("stripe_band_m", 0.20))
    gap = float(fg.get("stripe_gap_m", 0.10))
    foot = float(fg.get("stripe_foot_m", 0.05))

    qg = props["qualification_gate"]
    make_stripe(
        os.path.join(args.outdir, "gate_stripe_orange.png"),
        qg["marking_colour"],
        post_m=float(qg.get("marking_length", 0.60)),
        band_m=band, gap_m=gap, foot_m=0.0, rng=rng,
    )
    for name, key in (("gate_stripe_red.png", "port_colour"),
                      ("gate_stripe_green.png", "starboard_colour")):
        make_stripe(
            os.path.join(args.outdir, name), fg[key],
            post_m=float(fg["height"]), band_m=band, gap_m=gap, foot_m=foot,
            rng=rng,
        )

    make_water_surface(
        os.path.join(args.outdir, "water_surface.png"),
        pool["length"], pool["width"], rng=rng)

    make_drum_rim(os.path.join(args.outdir, "drum_rim.png"))
    for name, key in (("drum_wall_red.png", "red"), ("drum_wall_blue.png", "blue")):
        make_drum_wall(os.path.join(args.outdir, name),
                       props["drum"]["colours"][key], rng=rng)

    make_flare_fabric(os.path.join(args.outdir, "flare_orange.png"),
                      props["orange_flare"]["colour"], rng=rng)
    for colour_name, rgb in props["bump_flare"]["colours"].items():
        make_flare_fabric(
            os.path.join(args.outdir, f"flare_{colour_name}.png"), rgb,
            height=320, rng=rng)

    # Roughness maps. Uniform gloss is a large part of why untextured props read
    # as CG; PVC and moulded plastic are matte and unevenly so.
    make_surface_maps(args.outdir, rng)


if __name__ == "__main__":
    main()
