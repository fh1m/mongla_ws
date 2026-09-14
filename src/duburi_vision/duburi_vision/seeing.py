"""Can this camera see at all? A blinded camera and an empty scene look alike.

⛔ THE FAILURE THIS NAMES. The detector returns no boxes for two very
different reasons: nothing is there, or the camera cannot show anything --
a covered or fouled port, a washed-out veil, sun glare, a frozen driver. A
mission reads both as "keep searching" and searches until its budget runs
out. CCTV solved the same problem as "camera tampering detection" (defocus,
occlusion, freeze; e.g. arXiv 1608.02385), and this borrows its cheapest
signals.

MEASURED ON THE REAL ARCHIVE, NOT SIM. 3,385 frames at 1 frame/s from all 35
clips in ~/Work/Projects/Duburi/2025/raw_videos (Mirpur murky, RoboSub 2025,
final run), metrics on a 160x120 grey thumbnail. Synthetic faults were applied
to 64 real frames drawn from those clips. Scripts + raw JSONL:
bumblebee-study/rounds/seeing/.

    rule              real frames flagged   synthetic caught
    std < 10          0 / 3385 (min 12.4)   lens cap 64/64, veil 0.8 55/64,
                                            veil 0.6 12/64
    clipped > 0.25    0 / 3385 (max 0.175)  glare +140 64/64, +90 62/64
    frozen x5         0 identical consecutive frames in 2,400 read

WHAT IT CANNOT DO, measured and stated rather than discovered:
  * DEFOCUS / MILD FOG. A sigma-4 blur leaves edge energy at 0.70 of the
    frame's own value (median), and 1 % of real frames already dip to 0.52 of
    their 30 s baseline on scene change alone. Any rule that caught it would
    fire on ordinary footage.
  * A LIGHT VEIL (0.6) is caught 12/64. Murky Mirpur water sits at std ~20,
    so the washout margin is thin (12.4 real vs < 10) and is the first number
    to re-measure at a new venue.
So `ok` means "none of these faults", not "the image is good".

HYSTERESIS: a state must hold for HOLD_FRAMES consecutive frames before it is
reported, so one flash of sun or one dark frame never flips a mission.
"""
from __future__ import annotations

import cv2
import numpy as np

OK, COVERED, WASHOUT, GLARE, FROZEN = 'ok', 'covered', 'washout', 'glare', 'frozen'

COVERED_STD = 6.0      # real min 12.4; lens cap max 3.6
WASHOUT_STD = 10.0     # real min 12.4 -- thin margin, re-measure per venue
GLARE_CLIPPED = 0.25   # real max 0.175; +140 glare min 0.306
FROZEN_FRAMES = 5      # identical thumbnails in a row; real footage: 0 identical
HOLD_FRAMES = 3        # consecutive frames before a state is reported
THUMB = (160, 120)


def classify(gray_thumb: np.ndarray) -> str:
    """One frame's verdict, before hysteresis and freeze."""
    s = gray_thumb.astype(np.float32)
    std = float(s.std())
    if float((s >= 250).mean()) > GLARE_CLIPPED:
        return GLARE
    if std < COVERED_STD:
        return COVERED
    if std < WASHOUT_STD:
        return WASHOUT
    return OK


class Seeing:
    """Feed every processed frame; read `state`."""

    def __init__(self):
        self.state = OK
        self._prev = None
        self._same = 0
        self._candidate = OK
        self._run = 0

    def observe(self, frame: np.ndarray) -> str:
        if frame is None or frame.size == 0:
            return self.state
        gray = frame if frame.ndim == 2 else cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        thumb = cv2.resize(gray, THUMB, interpolation=cv2.INTER_AREA)
        if self._prev is not None and np.array_equal(thumb, self._prev):
            self._same += 1
        else:
            self._same = 0
        self._prev = thumb
        verdict = FROZEN if self._same >= FROZEN_FRAMES - 1 else classify(thumb)
        if verdict == self._candidate:
            self._run += 1
        else:
            self._candidate, self._run = verdict, 1
        # FROZEN already needed its own run of identical frames.
        if self._run >= HOLD_FRAMES or verdict == FROZEN:
            self.state = verdict
        return self.state
