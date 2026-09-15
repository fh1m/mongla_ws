"""Receive the SAUVC flare order through the camera the vehicle already has.

SAUVC Task 4: after the gate, "the team will be informed of the order in which
the flares have to be bumped" (e.g. R-B-Y) and "is then allowed to communicate
the order with their AUVs using their communication equipment". Tethers are
banned. The order is worth 60 points on top of 20 per flare, and at SAUVC 2025
"none succeeded in underwater communication".

Every team that tried built an acoustic modem. This needs no new hardware on
the vehicle: the operator shows the order to the camera with a coloured light
(an RGB torch or a phone screen in a bag), and this decodes it.

PROTOCOL (operator side), repeated until the vehicle acts:

    colour 1 ~0.6 s, dark ~0.4 s, colour 2, dark, colour 3, then dark >= 1.5 s

A message is accepted only when the SAME three distinct colours are received
twice in a row between long gaps. One flash misread, one dropped frame, or a
red prop drifting through view cannot produce two identical framed messages.

MEASURED ON THE REAL 2025 ARCHIVE (33 clips, 96,781 frames, no light flashed):
the FRAMING RULE held -- zero orders decoded. The PER-FRAME classifier did not:
15 % of frames read as a colour (R 3267, B 9768, Y 1616), from sun caustics
(yellow), blue water and glare, and red props. So a real flash competes with
the scene: hold the vehicle looking at the operator's light, close, against a
plain background, and expect to need more repeats in a bright or coloured view.
Thresholds are still not set with a real light and camera.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np

SYMBOLS = ('R', 'B', 'Y')


@dataclass
class OpticalOrderParams:
    v_min: int = 200            # a light source, not a lit prop
    s_min: int = 120            # a colour, not white glare
    min_area_frac: float = 6e-4  # of the downscaled frame
    min_on_s: float = 0.25
    max_on_s: float = 1.5       # longer than this is a prop, not a flash
    frame_gap_s: float = 1.2    # dark this long ends a message


def classify_frame(bgr: np.ndarray, p: OpticalOrderParams = OpticalOrderParams()) -> str:
    """'R' | 'B' | 'Y' | '.' (dark) for one frame."""
    import cv2
    small = cv2.resize(bgr, (160, 120), interpolation=cv2.INTER_AREA)
    hsv = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)
    h, s, v = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    lit = (v >= p.v_min) & (s >= p.s_min)
    counts = {
        'R': int(np.count_nonzero(lit & ((h <= 8) | (h >= 172)))),
        'Y': int(np.count_nonzero(lit & (h >= 20) & (h <= 35))),
        'B': int(np.count_nonzero(lit & (h >= 100) & (h <= 130))),
    }
    best = max(counts, key=counts.get)
    return best if counts[best] >= p.min_area_frac * small.shape[0] * small.shape[1] else '.'


@dataclass
class OrderDecoder:
    """Feed (t, symbol) in time order; `order` becomes a 3-tuple once confirmed."""
    p: OpticalOrderParams = field(default_factory=OpticalOrderParams)
    order: Optional[Tuple[str, str, str]] = None
    _run_sym: str = '.'
    _run_start: Optional[float] = None
    _msg: List[str] = field(default_factory=list)
    _last_msg: Optional[Tuple[str, ...]] = None
    _framed: bool = False
    _gap_closed: bool = False

    def feed(self, t: float, sym: str) -> Optional[Tuple[str, str, str]]:
        if self._run_start is None:
            self._run_sym, self._run_start = sym, t
            return self.order
        if sym == self._run_sym:
            # A long dark run ends a message even before the next colour arrives.
            if (sym == '.' and not self._gap_closed
                    and t - self._run_start >= self.p.frame_gap_s):
                self._gap_closed = True
                self._end_message()
            return self.order
        self._close_run(t)
        self._run_sym, self._run_start = sym, t
        self._gap_closed = False
        return self.order

    def _close_run(self, t: float) -> None:
        dur = t - self._run_start
        if self._run_sym == '.':
            if dur >= self.p.frame_gap_s and not self._gap_closed:
                self._end_message()
            return
        if dur > self.p.max_on_s:
            self._msg = []          # a steady colour is not a flash: drop the message
            self._framed = False
            self._last_msg = None
        elif dur >= self.p.min_on_s and self._framed:
            self._msg.append(self._run_sym)

    def _end_message(self) -> None:
        msg = tuple(self._msg)
        if len(msg) == 3 and len(set(msg)) == 3 and set(msg) <= set(SYMBOLS):
            if msg == self._last_msg:
                self.order = msg          # the same framed message twice
            self._last_msg = msg
        elif msg:
            self._last_msg = None
        self._msg = []
        self._framed = True
