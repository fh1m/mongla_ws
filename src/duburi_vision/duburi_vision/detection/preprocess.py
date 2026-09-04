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

WHAT IT BUYS: NOTHING, AND OFTEN LESS THAN NOTHING. THE ORIGINAL CLAIM HERE
IS RETRACTED.

This docstring used to record CLAHE taking the gate approach from 10.7 % to
56.2 % presence -- "five times the presence" -- and that number is what put
`preprocess='clahe'` into the `murky` profile. Re-measured on RAW DETECTION
RATE, one variable at a time, it does not reproduce anywhere:

    gate_back.mkv, robosub_gate2, conf 0.10, five independent frame samples
        stride 5  off 0     12.0 % -> 0.4 %
        stride 7  off 40    30.4 % -> 1.2 %
        stride 11 off 90    30.8 % -> 2.0 %

    9 still-image cases, 4 props, 3 venues, sharpness 33 .. 1284
        every delta between -2.7 and +2.7 points

    3 video clips, same measure
        gate -0.8, bin +3.8, octagon -6.5

Seventeen configurations. Never once meaningfully positive, and on the very
footage the original claim came from it destroys 95 % of the detections.

WHY, AND IT GENERALISES PAST CLAHE. The models were trained on UNPROCESSED
underwater frames -- not one of the 25 archived training configs uses blur,
rotation, perspective, or any contrast augmentation. Any preprocessing that
makes an image look better to a PERSON moves it away from the distribution the
detector actually learned. A contrast fix is a domain shift wearing a helpful
face, and the prettier the result looks the further it has moved.

WHAT THE ORIGINAL MEASUREMENT WAS PROBABLY SEEING. It was taken as tracker
PRESENCE with the tracker-clamp fix and a conf drop stacked into the same arm,
not as detector output with one variable moved. The clamp alone took presence
0.0 -> 45.1 % and conf 0.10 added 8.5 more. Attributing the remainder to the
last thing switched on is the confounded-A/B trap this round documented
elsewhere and then walked into.

WHAT SURVIVES. The blur measurement above is real and reproduced across three
venues: footage, not models, is what separates a 100 % clip from a 1.5 % one.
That still points at motion blur -- it just says the answer is exposure and
training-time augmentation, not a filter in front of the detector.

The code stays, selectable as `preprocess:=clahe`, because an operator may
have water we have never measured. No profile turns it on for them
(`test_profiles.py::test_no_profile_enables_preprocessing`).

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

⛔ IT IS NOT UNIVERSALLY GOOD, AND THIS IS WHY IT IS OFF BY DEFAULT.

Measured on a SECOND target -- the torpedo model over the bin clip, sparse
detections across a whole run -- it goes the other way:

    arm                        gate approach      torpedo on bin
    tracker clamp + conf 0.10      53.6 %             79.5 %
    ...+ CLAHE                     95.4 %  (+42)      15.3 %  (-64)

The two clips separate cleanly on frame statistics, and saturation splits
them harder than blur does:

                  blur (lapvar)   contrast   saturation
    gate                  315        27.7        159.9
    bin/torpedo          1241        36.1         27.9

**The rule: CLAHE helps BLURRY, LOW-CONTRAST, SATURATED water (green/murky)
and HURTS sharp, desaturated water.** That is physically sensible -- it
amplifies local contrast, which recovers a washed-out target and over-sharpens
one that was already crisp, pushing an already-marginal detection past the
model's decision boundary the wrong way.

So this is a per-water decision, not a per-vehicle one. Judge it from the
pool on the day: `tools/water_check.py` prints the three statistics and says
which side of the line the water is on. Do not set it once and forget it.
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
