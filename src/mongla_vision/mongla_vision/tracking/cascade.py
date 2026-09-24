"""Never lose the target: an association cascade, cheapest cue first.

⛔ THE PROBLEM, MEASURED. 179 identity switches in 253 s (section 54), and a
lock ladder whose lowest rung is outrun by a third of real gaps (section 51).
Both are the same failure: the tracker associates on MOTION alone, so anything
motion cannot explain becomes a new identity.

⭐ THE SOTA SHAPE. Enhanced CenterTrack (2026) reports IDF1 75.5 -> 82.5 from a
THREE-STAGE CASCADE -- Mahalanobis motion, then IoU, then centroid distance --
where each stage sees only what the previous one failed to match. BoT-SORT adds
appearance and explicit camera-motion compensation for the same purpose.
McByte++ (2026) gets +6.1 IDF1 from training-free online re-identification.

The cascade is what makes appearance AFFORDABLE. A dedicated Re-ID network at
15-25 ms/frame is unaffordable if it runs on every track; it is nearly free if
it runs only on the handful the cheap stages could not explain.

⭐⭐ AND ONE STAGE HERE IS NOT IN ANY OF THOSE PAPERS. BoT-SORT estimates
camera motion by fitting an affine transform BETWEEN IMAGES -- because a
surveillance camera has no other way to know it moved. This vehicle does: the
downward camera is a verified velocity sensor (1.09 cm over 30 cm, section 5)
and the board reports attitude at 50 Hz with the gyro bias already removed
(section 37). So ego-motion is MEASURED, not inferred from the very pixels
whose motion is in question. A target that "moved" because the VEHICLE moved
is explained before appearance is ever consulted.

⛔ WHAT THIS MODULE IS NOT. It does not fabricate a box. A stage that cannot
explain a detection passes it on; a detection no stage explains becomes a new
track, honestly. The ladder already refuses to invent a position and this does
not change that -- it reduces how often the question has to be asked.
"""
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Sequence, Tuple
import math

# Stage names, in the order they run. Cheapest first is not a style choice:
# it is what keeps the expensive stage off the hot path.
MOTION = 'motion'
EGO = 'ego'
APPEARANCE = 'appearance'
NEW = 'new'

# IoU below this is not the same object by motion alone.
IOU_GATE = 0.30

# After ego-compensation the box should land close. Looser than IOU_GATE
# because the compensation itself carries error.
EGO_IOU_GATE = 0.20

# ⛔ Appearance is the expensive stage. Never ask it about more than this many
# unmatched detections in one frame -- the cost must stay bounded even in the
# pathological case where the detector fires on a cloud of silt.
MAX_APPEARANCE_CALLS = 4


@dataclass(frozen=True)
class Assoc:
    """One detection's fate, and which cue decided it."""
    det_index: int
    track_id: Optional[int]
    stage: str
    score: float = 0.0
    reason: str = ''


def iou(a: Sequence[float], b: Sequence[float]) -> float:
    ax0, ay0, ax1, ay1 = a
    bx0, by0, bx1, by1 = b
    ix0, iy0 = max(ax0, bx0), max(ay0, by0)
    ix1, iy1 = min(ax1, bx1), min(ay1, by1)
    iw, ih = max(0.0, ix1 - ix0), max(0.0, iy1 - iy0)
    inter = iw * ih
    if inter <= 0.0:
        return 0.0
    ua = (ax1 - ax0) * (ay1 - ay0) + (bx1 - bx0) * (by1 - by0) - inter
    return inter / ua if ua > 0 else 0.0


def shift_box(box: Sequence[float], dx: float, dy: float) -> Tuple[float, ...]:
    return (box[0] + dx, box[1] + dy, box[2] + dx, box[3] + dy)


def ego_pixel_shift(vx_mps: float, vy_mps: float, yaw_rate_dps: float,
                    dt_s: float, m_per_px: float, fx_px: float) -> Tuple[float, float]:
    """How far the IMAGE moved because the VEHICLE moved, in pixels.

    ⭐ MEASURED, NOT FITTED. `vx`/`vy` come from the downward camera's verified
    velocity, `yaw_rate` from the board's IMU. BoT-SORT has to estimate this
    from image content; we read it from sensors that do not share a failure
    mode with the thing being tracked.

    ⛔ Returns (nan, nan) when scale is unknown. Pixels become metres only with
    an altitude, and substituting a constant would put a plausible number where
    a measurement is missing -- the recurring defect in this codebase.
    """
    if not (m_per_px > 0.0) or m_per_px != m_per_px:
        return float('nan'), float('nan')
    if dt_s <= 0.0 or dt_s != dt_s:
        return float('nan'), float('nan')
    # Translation: metres of vehicle motion -> pixels of scene motion, opposite
    # sign (the world slides the other way).
    tx = -(vx_mps * dt_s) / m_per_px
    ty = -(vy_mps * dt_s) / m_per_px
    # Rotation about the optical axis moves the image by f * tan(theta); for
    # the small angles one frame allows, f * theta is the same number.
    if fx_px > 0.0 and math.isfinite(yaw_rate_dps):
        tx += -math.radians(yaw_rate_dps * dt_s) * fx_px
    return tx, ty


class AssociationCascade:
    """Ordered association. Each stage sees only what the last could not match.

    Pure: no ROS, no cv2. `appearance` is injected, so the same logic runs
    against XFeat, a stub, or nothing at all.
    """

    def __init__(self, *, appearance: Optional[Callable] = None,
                 iou_gate: float = IOU_GATE,
                 ego_iou_gate: float = EGO_IOU_GATE,
                 max_appearance_calls: int = MAX_APPEARANCE_CALLS):
        self._appearance = appearance
        self.iou_gate = float(iou_gate)
        self.ego_iou_gate = float(ego_iou_gate)
        self.max_appearance_calls = int(max_appearance_calls)
        self.stage_counts: Dict[str, int] = {}

    def _note(self, stage: str) -> None:
        self.stage_counts[stage] = self.stage_counts.get(stage, 0) + 1

    def associate(self, detections: Sequence[Sequence[float]],
                  predictions: Dict[int, Sequence[float]],
                  *, ego_shift: Tuple[float, float] = (0.0, 0.0),
                  appearance_ctx: Optional[Sequence] = None) -> List[Assoc]:
        """Match detections to predicted track boxes.

        `detections`  xyxy per detection
        `predictions` track_id -> predicted xyxy for this frame
        `ego_shift`   (dx, dy) px the image moved because the VEHICLE moved;
                      (nan, nan) when unknown, which SKIPS the ego stage
                      rather than guessing it is zero.
        """
        out: List[Assoc] = []
        free_tracks = dict(predictions)
        pending = list(range(len(detections)))

        # ---- stage 1: motion ------------------------------------------
        still: List[int] = []
        for di in pending:
            best_id, best = None, 0.0
            for tid, pbox in free_tracks.items():
                v = iou(detections[di], pbox)
                if v > best:
                    best_id, best = tid, v
            if best_id is not None and best >= self.iou_gate:
                free_tracks.pop(best_id)
                self._note(MOTION)
                out.append(Assoc(di, best_id, MOTION, best, f'IoU {best:.2f}'))
            else:
                still.append(di)

        # ---- stage 2: the same test, after removing OUR OWN motion ----
        # ⛔ SKIPPED, not assumed zero, when the shift is unknown. A vehicle
        # that cannot measure its own motion must not claim it did not move.
        dx, dy = ego_shift
        if still and free_tracks and dx == dx and dy == dy and (dx or dy):
            again: List[int] = []
            for di in still:
                best_id, best = None, 0.0
                for tid, pbox in free_tracks.items():
                    v = iou(detections[di], shift_box(pbox, dx, dy))
                    if v > best:
                        best_id, best = tid, v
                if best_id is not None and best >= self.ego_iou_gate:
                    free_tracks.pop(best_id)
                    self._note(EGO)
                    out.append(Assoc(di, best_id, EGO, best,
                                     f'IoU {best:.2f} after ego shift '
                                     f'({dx:+.0f},{dy:+.0f}) px'))
                else:
                    again.append(di)
            still = again

        # ---- stage 3: appearance, on the few that are left ------------
        if still and self._appearance is not None:
            # ⛔ THE COST BOUND. This is what makes a 15-25 ms Re-ID stage
            # affordable: it is asked about a handful, not about every track.
            budget = min(self.max_appearance_calls, len(still))
            leftover: List[int] = []
            for k, di in enumerate(still):
                if k >= budget:
                    leftover.append(di)
                    continue
                ctx = appearance_ctx[di] if appearance_ctx else None
                res = self._appearance(di, ctx)
                if res is not None:
                    self._note(APPEARANCE)
                    out.append(Assoc(di, int(res), APPEARANCE, 0.0,
                                     're-identified by appearance'))
                else:
                    leftover.append(di)
            still = leftover

        for di in still:
            self._note(NEW)
            out.append(Assoc(di, None, NEW, 0.0, 'no stage explained it'))
        out.sort(key=lambda a: a.det_index)
        return out
