"""Contrast preprocessing for underwater frames, measured on real footage.

WHY THIS EXISTS. Three 2025 competition models on video of the competition
they were trained for, same detector, same threshold:

    bin       100.0 % of frames
    octagon    92.4 %
    gate        1.5 %

The models are not the difference -- each reports mAP50 = 0.995 on its own
validation set. The FOOTAGE is. Measured on the same clips:

    gate_back.mkv   Laplacian variance  321   brightness 170.8  contrast 27.7
    bin.mkv         Laplacian variance 1180   brightness 170.3  contrast 35.4

The gate clip is 3.7x blurrier at identical brightness. That is motion blur
plus washed-out underwater contrast, and it is the regime an AUV spends most
of its run in -- a dataset of still frames does not contain it, and no
threshold recovers a feature the image no longer has.

WHAT IT BUYS, measured across 600 frames of the gate approach at conf 0.15:

    preprocessing        presence   mean score
    none                   10.7 %      0.226
    unsharp mask           38.0 %      0.329
    CLAHE (LAB, clip 2)    56.3 %      0.401
    CLAHE (YUV, clip 3)    56.2 %      0.410     <- shipped

Five times the presence. And it does NOT cost anything on footage that
already works: bin stays 100 %, octagon stays 100 % and its mean score
RISES (0.570 -> 0.634).

WHY YUV RATHER THAN LAB. Same result, half the price. Measured on the Pi at
640x360, one thread:

    unsharp        7.62 ms
    CLAHE (LAB)    7.07 ms
    CLAHE (YUV)    3.78 ms

3.78 ms against an 18.0 ms photon-to-detections budget is ~46 Hz instead of
77 -- a real cost, which is why this is a parameter and not a default, and
why the operator is told what it buys rather than left to discover it.

CLAHE, not global equalisation: underwater frames are locally washed out, and
a global histogram stretch amplifies the backscatter haze along with the
target. `clipLimit` bounds that amplification per tile.
"""
from __future__ import annotations

from typing import Callable, Optional

import cv2

# Measured best on the gate approach (56.2 % presence, mean score 0.410).
# Higher clip limits amplify backscatter without finding more target.
DEFAULT_CLIP = 3.0
DEFAULT_TILES = 8


def make_clahe(clip_limit: float = DEFAULT_CLIP,
               tiles: int = DEFAULT_TILES) -> Callable:
    """A BGR->BGR contrast enhancer, with the CLAHE object built ONCE.

    Rebuilding it per frame is ~2x the cost for the same output, and this
    runs in the hot path.
    """
    clahe = cv2.createCLAHE(clipLimit=float(clip_limit),
                            tileGridSize=(int(tiles), int(tiles)))

    def _apply(frame_bgr):
        # Y of YUV, not L of LAB: identical measured benefit (56.2 vs 56.3 %
        # presence) at half the conversion cost, because YUV<->BGR is a
        # cheaper transform than LAB<->BGR.
        yuv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2YUV)
        yuv[:, :, 0] = clahe.apply(yuv[:, :, 0])
        return cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR)

    return _apply


def make_preprocessor(name: str, clip_limit: float = DEFAULT_CLIP,
                      tiles: int = DEFAULT_TILES) -> Optional[Callable]:
    """`name` -> a frame transform, or None for 'off'.

    Returning None rather than an identity function is deliberate: the caller
    can then skip the call entirely, and 'is preprocessing on' is answerable
    by looking at one attribute instead of inspecting a closure.
    """
    # `name` may arrive as a BOOL, not a string. ROS 2 launch coerces the
    # literal 'off' in a `default_value` to boolean False before it ever
    # reaches `declare_parameter`, which then raises
    # InvalidParameterTypeException and kills the whole composed process at
    # startup. Measured on the vehicle: every node dead, 'off' in the launch
    # file, and the only clue eleven frames down a traceback.
    #
    # Accepting the bool is not leniency for its own sake -- False IS what the
    # operator wrote, and refusing to understand our own launch file would be
    # a worse answer than normalising it here.
    if isinstance(name, bool):
        name = 'off' if not name else 'clahe'
    key = str(name or '').strip().lower()
    if key in ('', 'off', 'none', 'false', '0'):
        return None
    if key == 'clahe':
        return make_clahe(clip_limit, tiles)
    raise ValueError(
        f"unknown preprocess {name!r} -- expected 'clahe' or 'off'. "
        f"An unrecognised value must not silently mean 'off': that is how a "
        f"setting reaches nothing and the measurement does not move.")
