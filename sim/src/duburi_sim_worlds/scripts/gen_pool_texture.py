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
import os

import numpy as np
import yaml
from PIL import Image

# Palette, linear 0-1. Competition pool: pale blue mosaic tiles, white grout,
# navy lane lines, black cross markers.
FIELD = np.array([0.58, 0.80, 0.90])
GROUT = np.array([0.90, 0.93, 0.95])
LANE = np.array([0.04, 0.10, 0.40])
CROSS = np.array([0.02, 0.02, 0.03])

TILE_M = 0.10  # tile pitch in metres
GROUT_PX = 2  # thicker grout reads better at camera distance

# Competition pools mark lanes every 2.5 m across the width.
LANE_SPACING_M = 2.5
LANE_WIDTH_M = 0.30
CROSS_WIDTH_M = 0.28
CROSS_LENGTH_M = 1.1
CROSS_INSET_M = 2.0

# Per-tile mosaic variation (std of additive RGB noise).
TILE_NOISE = 0.035

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
DEFAULT_SPEC = os.path.join(PKG, "spec", "arena.yaml")
DEFAULT_OUTDIR = os.path.join(PKG, "models", "sauvc_textures")


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


def make_roughness(path: str, base: float, width: int = 128, height: int = 512,
                   variation: float = 0.08, rng=None) -> None:
    """Greyscale roughness map. Uniform gloss is what makes a prop look CG."""
    rng = rng or np.random.default_rng(0)
    v = np.clip(base + rng.normal(0.0, variation, (height, width)), 0.0, 1.0)
    img = np.repeat((v * 255).astype(np.uint8)[:, :, None], 3, axis=2)
    Image.fromarray(img).save(path, optimize=True)
    print(f"wrote {path}  ({width}x{height})")


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


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--spec", default=DEFAULT_SPEC, help="Arena spec YAML.")
    parser.add_argument("--outdir", default=DEFAULT_OUTDIR, help="Output directory.")
    parser.add_argument("--seed", type=int, default=2026, help="Mosaic RNG seed.")
    args = parser.parse_args()

    with open(args.spec) as f:
        spec = yaml.safe_load(f)

    pool = spec["pool"]
    px_per_m = int(spec.get("texture_px_per_m", 64))
    os.makedirs(args.outdir, exist_ok=True)
    rng = np.random.default_rng(args.seed)

    make_floor(
        pool["length"],
        pool["width"],
        px_per_m,
        os.path.join(args.outdir, "pool_floor.png"),
        rng,
    )
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

    # ---- prop textures -------------------------------------------------
    # Driven by the rulebook segment sizes in spec/arena.yaml, NOT by a band
    # count, so the physical stripe pitch is right on every post length.
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
    make_roughness(os.path.join(args.outdir, "rough_pvc.png"), 0.62, rng=rng)
    make_roughness(os.path.join(args.outdir, "rough_plastic.png"), 0.78, rng=rng)
    make_roughness(os.path.join(args.outdir, "rough_fabric.png"), 0.88, rng=rng)


if __name__ == "__main__":
    main()
