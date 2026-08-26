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
    bands: int = 8,
    width: int = 128,
    height: int = 512,
    white=(0.92, 0.92, 0.92),
) -> None:
    """Vertical-axis stripe map for a cylinder (V along height)."""
    img = np.zeros((height, width, 3), dtype=np.float64)
    band_h = height / bands
    for i in range(bands):
        y0 = int(i * band_h)
        y1 = int((i + 1) * band_h)
        c = colour if i % 2 == 0 else white
        img[y0:y1, :] = c
    Image.fromarray((np.clip(img, 0, 1) * 255).astype(np.uint8)).save(
        path, optimize=True
    )
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

    # Gate / drum textures used by prop_library.
    make_stripe(
        os.path.join(args.outdir, "gate_stripe_orange.png"),
        spec["props"]["qualification_gate"]["marking_colour"],
        bands=6,
    )
    make_stripe(
        os.path.join(args.outdir, "gate_stripe_red.png"),
        spec["props"]["final_gate"]["port_colour"],
        bands=int(spec["props"]["final_gate"].get("stripe_count", 5)) * 2,
    )
    make_stripe(
        os.path.join(args.outdir, "gate_stripe_green.png"),
        spec["props"]["final_gate"]["starboard_colour"],
        bands=int(spec["props"]["final_gate"].get("stripe_count", 5)) * 2,
    )
    make_drum_rim(os.path.join(args.outdir, "drum_rim.png"))


if __name__ == "__main__":
    main()
