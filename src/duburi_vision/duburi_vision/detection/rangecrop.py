"""Trade field of view for range, automatically, when the target is far.

THE MEASUREMENT THAT MOTIVATES IT. Simulated approach on real labelled data
(the bin dataset, its own model, conf 0.15) -- recall against apparent
target size:

    apparent size   ~range        full frame   centre 50 % crop
    1.00            1.0x            100.0 %        69.0 %
    0.50            2.0x             99.2 %       100.0 %
    0.35            2.9x             98.4 %        99.2 %
    0.25            4.0x             65.9 %       100.0 %
    0.15            6.7x             20.2 %        67.4 %

Recall falls off a CLIFF past ~3x the training distance -- 99 % to 66 % to
20 % -- and cropping recovers almost all of it, because the target then
occupies more of the 640x640 the chip actually sees.

WHY NOT JUST RAISE imgsz. On the Hailo it is baked into the HEF (all three
of ours are `NHWC(640x640x3)`), so it is a compile-time decision, not a knob.
Cropping reaches the same place at runtime.

IT IS A REAL TRADE, AND BOTH SIDES ARE MEASURED. Half the field of view lost
36 objects outright in that run -- they fell outside the crop -- and CLOSE
targets drop to 69 % because they no longer fit inside it. So a fixed crop is
strictly worse than no crop for a prop you are next to.

Hence: crop only while the target is FAR, release the moment it is not. The
switch is on the observed target size, which the detector already reports, so
it costs nothing to know and the vehicle is never cropped when it is
manoeuvring close.

HYSTERESIS IS NOT OPTIONAL. A single threshold on a quantity that jitters
makes the field of view flap frame to frame, which is worse than either
state: the control loop sees the target appear and vanish for reasons that
have nothing to do with the water. Enter and exit are separated.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

import cv2

# Sized from the measurement above. Below ENTER the full frame is losing
# targets (65.9 % at 0.25); above EXIT the crop is losing them (69.0 % at
# 1.0). The gap between them is the hysteresis band, and it is wide because
# the quantity is a bbox area fraction, which is noisy.
ENTER_FRAC = 0.010     # target below 1 % of frame -> start cropping
EXIT_FRAC = 0.045      # target above 4.5 % of the CROP -> stop
LOST_RELEASE_S = 1.5   # no target for this long -> full frame, always

# Half linear size = a quarter of the area = 2x apparent size. Deeper crops
# were not measured and are not offered: an unmeasured setting is how a knob
# becomes a superstition.
CROP_FRAC = 0.5


@dataclass
class CropState:
    """What the policy decided, and why. Returned so the caller can log it."""
    active: bool = False
    x0: int = 0
    y0: int = 0
    w: int = 0
    h: int = 0
    reason: str = 'full frame'

    def to_full(self, xyxy: Tuple[float, float, float, float]):
        """Map a box from crop coordinates back to the full frame.

        Every consumer downstream -- the bearing, the pixel error, the HUD --
        works in full-frame pixels. Forgetting this offset does not raise; it
        steers the vehicle at a point displaced by the crop origin, which is
        the sort of error that looks like a calibration problem.
        """
        if not self.active:
            return xyxy
        x1, y1, x2, y2 = xyxy
        return (x1 + self.x0, y1 + self.y0, x2 + self.x0, y2 + self.y0)


class RangeCrop:
    """Decides, per frame, whether to feed the detector a centre crop."""

    def __init__(self, *, enter_frac: float = ENTER_FRAC,
                 exit_frac: float = EXIT_FRAC,
                 crop_frac: float = CROP_FRAC,
                 lost_release_s: float = LOST_RELEASE_S):
        if not 0.0 < crop_frac < 1.0:
            raise ValueError(f'crop_frac must be in (0,1), got {crop_frac}')
        if enter_frac >= exit_frac:
            raise ValueError(
                f'enter_frac {enter_frac} must be BELOW exit_frac {exit_frac} '
                f'-- without a gap the field of view flaps frame to frame')
        self._enter = float(enter_frac)
        self._exit = float(exit_frac)
        self._crop = float(crop_frac)
        self._release = float(lost_release_s)
        self._active = False
        self._last_seen_t: Optional[float] = None

    def observe(self, area_frac: Optional[float], now: float) -> None:
        """Feed the largest target's area fraction, or None if not seen.

        `area_frac` is measured in whatever frame the detector was given --
        so while cropped it is a fraction OF THE CROP, which is exactly the
        quantity `exit_frac` should compare against.
        """
        if area_frac is None:
            # Release on a sustained loss. A cropped view that has lost the
            # target is the worst state to search from, and searching is
            # precisely what happens next.
            if (self._active and self._last_seen_t is not None
                    and now - self._last_seen_t > self._release):
                self._active = False
            return
        self._last_seen_t = now
        if self._active:
            if area_frac > self._exit:
                self._active = False
        else:
            if area_frac < self._enter:
                self._active = True

    def apply(self, frame_bgr) -> Tuple[object, CropState]:
        """(frame_to_infer, state). The frame is a VIEW, not a copy."""
        if not self._active or frame_bgr is None:
            return frame_bgr, CropState(reason='full frame')
        h, w = frame_bgr.shape[:2]
        cw, ch = int(w * self._crop), int(h * self._crop)
        x0, y0 = (w - cw) // 2, (h - ch) // 2
        return (frame_bgr[y0:y0 + ch, x0:x0 + cw],
                CropState(active=True, x0=x0, y0=y0, w=cw, h=ch,
                          reason=f'target below {self._enter:.1%} of frame '
                                 f'-- cropping to {self._crop:.0%} for range'))

    @property
    def active(self) -> bool:
        return self._active
