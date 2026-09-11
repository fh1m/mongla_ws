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
    """One frame's answer. `t` is the CAPTURE instant, never arrival.

    `vehicle_yaw_deg` is the hull's own heading at that instant. Optional: with
    it the fuser can tell the two branches apart by HOW THEY MOVE, which works
    where counting does not (see `slope_of`). Without it, counting is all there
    is and the fuser says so.
    """
    t: float
    yaw_deg: float
    range_m: float = 0.0
    ambiguity: float = 0.0      # best/second reprojection error; ->1 = a coin flip
    reproj_px: float = 0.0
    n_points: int = 0
    vehicle_yaw_deg: Optional[float] = None


# The hull must actually TURN before ego-motion can separate the branches. A
# station-keeping vehicle gives dpsi ~ 0, the regression divides by nothing and
# returns a confident slope from noise -- the exact failure this fuser exists to
# refuse. Degrees of heading excursion required inside the window.
MIN_YAW_EXCURSION_DEG = 4.0

# The true branch's slope is -1 and the false branch's is +1, so anything
# inside this band of zero is not evidence either way.
SLOPE_DEADBAND = 0.25


@dataclass(frozen=True)
class Fused:
    decided: bool
    yaw_deg: float = float('nan')
    range_m: float = float('nan')
    support: int = 0            # frames in the winning cluster
    considered: int = 0         # frames that passed the gates
    spread_deg: float = float('nan')   # MAD of the winning cluster
    rival: int = 0              # frames in the NEXT largest cluster
    rule: str = ''              # 'egomotion' | 'support' -- WHICH test decided
    slope: float = float('nan') # d(pose yaw)/d(hull yaw) of the winner
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



def slope_of(members: list) -> tuple:
    """d(pose yaw) / d(hull yaw) over a cluster, and the hull's excursion.

    ⛔ THE DISCRIMINANT, AND WHY IT BEATS COUNTING. The two planar-PnP branches
    are mirror images about the viewing ray, and each frame's solver mirrors
    afresh. So when the hull yaws by dpsi, the TRUE branch's pose yaw moves by
    -dpsi (the board is fixed in the world; turning the camera sweeps it the
    other way) while the FALSE branch, being the mirror, moves by +dpsi.

        true  branch:  d(pose yaw) / d(hull yaw) = -1
        false branch:  d(pose yaw) / d(hull yaw) = +1

    That is unit-free, needs no calibration, and does not care which branch the
    detector happens to report more often -- which is the case that defeats
    "take the largest cluster". A biased corner detector, or a few frames from a
    slightly different viewpoint, can hand the majority to the wrong branch; it
    cannot make that branch move the right way under the hull's own rotation.

    The literature resolves this ambiguity with multi-view rotation averaging
    (Jin et al., arXiv:1909.11888) or by locating the second minimum
    analytically (Schweighofer & Pinz, TPAMI 2006). Both are about the geometry
    of the target. This uses something we already have and they did not assume:
    a heading source good to under 0.01 deg/min, on a hull that is turning
    anyway.

    Returns `(slope, excursion_deg)`. `slope` is NaN when the hull did not turn
    enough for the question to mean anything -- a station-keeping vehicle gives
    dpsi ~ 0, and a regression on that returns a confident number built from
    noise.
    """
    pairs = [(m.vehicle_yaw_deg, m.yaw_deg) for m in members
             if m.vehicle_yaw_deg is not None]
    if len(pairs) < 3:
        return float('nan'), 0.0
    # Unwrap both series about their first sample so a pass through +/-180 does
    # not inject a 360 deg step into a regression that reads slope.
    psi0, th0 = pairs[0]
    xs = [_wrap180(p - psi0) for p, _ in pairs]
    ys = [_wrap180(t - th0) for _, t in pairs]
    excursion = max(xs) - min(xs)
    if excursion < MIN_YAW_EXCURSION_DEG:
        return float('nan'), excursion
    mx = sum(xs) / len(xs)
    my = sum(ys) / len(ys)
    sxx = sum((x - mx) ** 2 for x in xs)
    if sxx <= 1e-9:
        return float('nan'), excursion
    sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    return sxy / sxx, excursion


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

        # ⛔ COUNTING IS THE FALLBACK, NOT THE RULE. Ask first how each cluster
        # MOVES under the hull's own rotation: the true branch slopes -1
        # against hull yaw and the mirrored one +1, whichever the detector
        # reported more often. Counting cannot see that, so a biased corner
        # detector hands the majority -- and the answer -- to the wrong branch.
        rule = 'support'
        slope = float('nan')
        if len(groups) > 1:
            scored = []
            for g in groups:
                sl, exc = slope_of([kept[i] for i in g])
                if sl == sl and abs(sl) > SLOPE_DEADBAND:   # not NaN, not flat
                    scored.append((sl, g))
            # Only decide this way when the branches actually DISAGREE about
            # direction. Two clusters sloping the same way are not a mirror
            # pair; they are two different things, and the mirror test says
            # nothing about which to believe.
            if len(scored) >= 2 and min(sl for sl, _ in scored) < 0 < max(
                    sl for sl, _ in scored):
                sl, g = min(scored, key=lambda sg: sg[0])   # most negative
                groups = [g] + [h for h in groups if h is not g]
                rule, slope = 'egomotion', sl

        best = groups[0]
        rival = len(groups[1]) if len(groups) > 1 else 0
        support = len(best)
        if support < self.min_poses:
            return Fused(False, considered=len(kept), support=support,
                         rival=rival, rule=rule, slope=slope,
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
            rule=rule,
            slope=slope,
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
