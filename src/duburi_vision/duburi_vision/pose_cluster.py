"""Fuse a stream of single-frame poses into one answer, with its support.

⛔ THE DEFECT THIS EXISTS TO PREVENT, and it is the one this repo keeps
finding: a plausible number standing in for an absent measurement. A planar
target's PnP solution is AMBIGUOUS -- two poses reproject almost equally well,
mirrored about the board normal, so a stream of per-frame answers is BIMODAL at
roughly `+yaw` and `-yaw`. The mean of that distribution sits near ZERO: a
confident, stable, face-on reading of a board the vehicle is 30 degrees off.
No stage errors, nothing looks wrong, and the torpedo misses.

So this never averages across the stream. It groups, picks the group with the
most support, and reports the MEDIAN of that group plus how many frames agreed
and how far apart they were. A caller that wants a point estimate gets one; a
caller that wants to know whether to believe it gets `support` and `spread`.

BumblebeeAS reached the same place from the other direction: their 2026 gate
replaced TF clustering with pose clustering time-synced to odometry, with
`min_poses: 4` over a 15 s window -- after winning the year before without it.
Ours differs in one way that matters: we cluster in the CAMERA frame and need
no odometry, which is why this runs today rather than after the flow DVL lands.

Pure math. No ROS, no cv2, no node -- so the decision rule can be tested
against constructed distributions instead of against whatever the pool
happened to show.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Iterable, Optional

# A pose whose two flip branches disagree by less than this is not really
# ambiguous; the solver already picked. Above it, the frame carries a genuine
# fork and belongs in a cluster rather than being trusted alone.
FLIP_SPREAD_DEG = 8.0

# Frames grouped together must agree to within this. Set from the flip
# geometry, NOT from a tuning session: the two branches of a planar solve sit
# roughly symmetric about zero, so the gap between them is about `2 * |yaw|`.
# A tolerance wider than the smallest interesting |yaw| would merge the fork it
# exists to separate. 12 deg keeps forks apart down to |yaw| ~ 6 deg.
CLUSTER_TOL_DEG = 12.0

# Frames older than this cannot describe where the vehicle is NOW. 15 s is
# BumblebeeAS's window; it suits a hull that is station-keeping in front of a
# board, not one in transit, so a mission that is moving should shorten it.
WINDOW_S = 15.0

# Below this many agreeing frames there is no cluster, only noise that happens
# to be adjacent. Reported as `decided=False` rather than a low-confidence
# answer, because a caller that reads the number will act on it.
MIN_POSES = 4


@dataclass(frozen=True)
class PoseSample:
    """One frame's answer. `t` is the CAPTURE instant, never arrival."""
    t: float
    yaw_deg: float
    range_m: float = 0.0
    ambiguity: float = 0.0      # best/second reprojection error; ->1 = a coin flip
    reproj_px: float = 0.0
    n_points: int = 0


@dataclass(frozen=True)
class Fused:
    decided: bool
    yaw_deg: float = float('nan')
    range_m: float = float('nan')
    support: int = 0            # frames in the winning cluster
    considered: int = 0         # frames that passed the gates
    spread_deg: float = float('nan')   # MAD of the winning cluster
    rival: int = 0              # frames in the NEXT largest cluster
    reason: str = ''


def _wrap180(deg: float) -> float:
    """Fold an angle into (-180, 180]. Clustering across the wrap is a real
    case: a board seen from behind-left reads +179 on one frame and -179 on the
    next, and those two agree."""
    d = math.fmod(float(deg) + 180.0, 360.0)
    if d <= 0.0:
        d += 360.0
    return d - 180.0


def _angdiff(a: float, b: float) -> float:
    return abs(_wrap180(a - b))


def _median(xs: list[float]) -> float:
    s = sorted(xs)
    n = len(s)
    if n == 0:
        return float('nan')
    mid = n // 2
    return s[mid] if n % 2 else 0.5 * (s[mid - 1] + s[mid])


def _circular_median(angles: list[float]) -> float:
    """Median of angles that may straddle the +/-180 wrap.

    Taken about the first sample so the wrap cannot split a tight group into
    two halves 358 degrees apart -- the same failure a plain median has on
    compass headings.
    """
    if not angles:
        return float('nan')
    ref = angles[0]
    return _wrap180(ref + _median([_wrap180(a - ref) for a in angles]))


class PoseCluster:
    """Sliding-window pose fuser. Feed samples, ask for the answer."""

    def __init__(self, *, window_s: float = WINDOW_S,
                 tol_deg: float = CLUSTER_TOL_DEG,
                 min_poses: int = MIN_POSES,
                 max_ambiguity: float = 0.9,
                 max_reproj_px: float = 10.0):
        self.window_s = float(window_s)
        self.tol_deg = float(tol_deg)
        self.min_poses = int(min_poses)
        self.max_ambiguity = float(max_ambiguity)
        self.max_reproj_px = float(max_reproj_px)
        self._samples: list[PoseSample] = []

    def add(self, s: PoseSample) -> None:
        self._samples.append(s)

    def _live(self, now: float) -> list[PoseSample]:
        cut = now - self.window_s
        self._samples = [s for s in self._samples if s.t >= cut]
        return self._samples

    def fuse(self, now: Optional[float] = None) -> Fused:
        """The current answer, or a stated reason there is not one."""
        if now is None:
            now = self._samples[-1].t if self._samples else 0.0
        live = self._live(now)
        if not live:
            return Fused(False, reason='no poses in the window')

        # Gate first, cluster second. A frame the solver could not distinguish
        # (`ambiguity` -> 1) contributes a coin flip to BOTH branches, so it
        # cannot break a tie -- it can only make one look like a majority.
        kept = [s for s in live
                if s.ambiguity <= self.max_ambiguity
                and (s.reproj_px <= self.max_reproj_px or s.reproj_px <= 0.0)]
        if not kept:
            return Fused(False, considered=0,
                         reason=f'all {len(live)} poses failed the '
                                f'ambiguity/reprojection gates')

        clusters = self._cluster([s.yaw_deg for s in kept])
        groups = sorted(clusters, key=len, reverse=True)
        best = groups[0]
        rival = len(groups[1]) if len(groups) > 1 else 0
        support = len(best)
        if support < self.min_poses:
            return Fused(False, considered=len(kept), support=support,
                         rival=rival,
                         reason=f'largest cluster has {support} poses, '
                                f'needs {self.min_poses}')

        members = [kept[i] for i in best]
        yaws = [m.yaw_deg for m in members]
        centre = _circular_median(yaws)
        spread = _median([_angdiff(y, centre) for y in yaws])
        ranges = [m.range_m for m in members if m.range_m > 0.0]
        return Fused(
            True,
            yaw_deg=centre,
            range_m=_median(ranges) if ranges else float('nan'),
            support=support,
            considered=len(kept),
            spread_deg=spread,
            rival=rival,
            reason='')

    def _cluster(self, yaws: list[float]) -> list[list[int]]:
        """Greedy grouping by angular distance to a group's running median.

        Greedy and not k-means on purpose: the number of modes is not known
        (one solid answer, or a two-way flip, or a third from a misdetection),
        and k-means with the wrong k returns a confident split of a single
        mode -- the same class of error as averaging the fork.
        """
        groups: list[list[int]] = []
        for i, y in enumerate(yaws):
            for g in groups:
                if _angdiff(y, _circular_median([yaws[j] for j in g])) <= self.tol_deg:
                    g.append(i)
                    break
            else:
                groups.append([i])
        return groups
