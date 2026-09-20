"""Underwater image statistics, and what they predict about detection.

WHY A MODULE AND NOT A SCRIPT. Three separate things now need the same
numbers -- the water check an operator runs poolside, the dataset survey that
tells us which conditions we have data for, and the preprocessing decision
itself. Three copies of "how blurry is this frame" is how two of them come to
disagree, which is the defect the camera profile table already had.

WHAT THESE QUANTITIES ARE, AND WHY THESE ONES

Underwater imaging degrades in ways that are physically distinct, and lumping
them into "bad visibility" is what makes a fix work on one pool and fail on
the next:

  sharpness    Variance of the Laplacian. Falls with MOTION BLUR (the vehicle
               is moving) and with SCATTERING (the water is carrying
               particulate). Both flatten edges; neither is fixed by exposure.

  contrast     Standard deviation of luminance. Falls with BACKSCATTER -- the
               veiling light that adds a constant to every pixel and crushes
               the dynamic range the detector needs.

  saturation   Mean HSV S. Falls with ABSORPTION, which is wavelength
               dependent: red dies first (~5 m), then green. A desaturated
               frame is a DEEP or LONG-RANGE frame; a saturated one is a
               shallow, murky, green one. This is the quantity that separated
               the two clips where CLAHE helped and hurt -- harder than blur
               did -- and it is the one most people leave out.

  colour cast  Mean B minus mean R. Positive is the blue-green shift of
               absorption. Says WHICH water, not how bad.

  brightness   Mean luminance. Included because it is the one people reach
               for first and it explains the least: the two clips that
               behaved oppositely under CLAHE had brightness 170 and 174.

THE POINT IS TO GENERALISE, NOT TO FIT ROBOSUB. A competition venue we have
never seen is characterised by these five numbers, and a preprocessing choice
made from them travels; a choice made from "this is the RoboSub pool" does
not.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, List, Sequence

import cv2
import numpy as np


@dataclass
class WaterStats:
    """One scene's optical character. All medians over the frames given."""
    sharpness: float          # Laplacian variance -- blur AND scattering
    contrast: float           # luminance stddev -- backscatter
    saturation: float         # mean HSV S -- absorption / range
    cast: float               # mean(B) - mean(R) -- which water
    brightness: float         # mean luminance -- explains the least
    frames: int = 0

    def as_row(self) -> str:
        return (f'sharp {self.sharpness:8.1f}  contrast {self.contrast:6.1f}  '
                f'sat {self.saturation:6.1f}  cast {self.cast:+7.1f}  '
                f'bright {self.brightness:6.1f}')


def frame_stats(frame_bgr) -> tuple:
    """The five numbers for ONE frame. Cheap enough to run per frame."""
    g = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    b, _gr, r = cv2.split(frame_bgr)
    return (float(cv2.Laplacian(g, cv2.CV_64F).var()),
            float(g.std()),
            float(hsv[:, :, 1].mean()),
            float(b.mean()) - float(r.mean()),
            float(g.mean()))


def analyse_frames(frames: Iterable) -> WaterStats:
    """Medians, not means.

    A single bright frame -- a surface glint, a passing wall -- moves a mean
    and not a median, and the whole point is to characterise the water rather
    than one moment in it. The same reasoning that took the freshness
    thresholds off a maximum and onto a p99.
    """
    cols: List[List[float]] = [[], [], [], [], []]
    n = 0
    for f in frames:
        if f is None:
            continue
        for i, v in enumerate(frame_stats(f)):
            cols[i].append(v)
        n += 1
    if not n:
        return WaterStats(*(float('nan'),) * 5, frames=0)
    med = [float(np.median(c)) for c in cols]
    return WaterStats(*med, frames=n)


# --------------------------------------------------------------------------- #
#  What the numbers predict
# --------------------------------------------------------------------------- #
# From the only two regimes measured end to end so far (RoboSub 2025):
#
#                       sharp   contrast   sat     CLAHE effect
#   gate approach        315      27.7     160     +42 points of presence
#   bin / torpedo       1241      36.1      28     -64 points
#
# Deliberately a WIDE undecided band. Two samples do not make a universal
# threshold, and a rule that pretends otherwise is the overfit this module
# exists to avoid. Widening the band is the honest way to hold n=2.

# --------------------------------------------------------------------------- #
#  There is deliberately NO recommend() here any more
# --------------------------------------------------------------------------- #
# This module used to map these stats onto a CLAHE verdict, on thresholds fitted
# to two clips. A third venue (Mirpur -- sharpness 33, saturation 148, cast +92,
# the murkiest water in the archive) is exactly what those thresholds call
# `CLAHE ON`, and CLAHE measurably does not help there. Re-measured on raw
# detection rate across 17 configurations -- 4 props, 3 venues, a 39x sharpness
# range, five independent frame samples of the clip the original claim came
# from -- it was NEVER positive, and on the gate it took 30.4 % to 1.2 %.
#
# So the mapping is deleted rather than inverted. What survives is the
# CHARACTERISATION above, which reproduced across all three venues: it tells
# you which water you are in, which is a fact about the pool. What preprocessing
# to do about it is not something two clips ever knew.
#
# The mechanism, stated because it generalises past CLAHE: the models were
# trained on UNPROCESSED underwater frames. Any preprocessing that makes an
# image look better to a person moves it away from the distribution the
# detector learned. A contrast fix is a domain shift wearing a helpful face.
