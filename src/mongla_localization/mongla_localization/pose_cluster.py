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
    # The OTHER planar-PnP branch for the same frame, and the camera-frame
    # bearing to the target (+ = right of the optical axis). With both, and
    # the hull's heading, every frame votes for BOTH hypotheses and the
    # viewpoint test can tell them apart (see `resolve_mirror`). None = the
    # producer did not supply them, never "zero".
    alt_yaw_deg: Optional[float] = None
    bearing_deg: Optional[float] = None


# ⛔ THE RULE THAT DECIDES A MIRROR PAIR (issue #55). A planar target's two PnP
# branches are reflections about the VIEWING RAY: for yaw, the alternate sits
# at about `2*beta - theta`, beta the bearing to the target and theta its true
# normal. Expressed in the WORLD (pose yaw + hull yaw):
#
#     true branch    world yaw = theta                 -> slope 0 vs bearing
#     mirror branch  world yaw = 2*beta_world - theta  -> slope 2 vs bearing
#
# measured by projecting a fixed target through a moving camera and solving
# (`test_pose_mirror_truth.py`): 0.00 and +2.00 exactly, in EITHER translation
# direction, surviving 10 deg of pitch and 0.5 px noise (0.14 / 1.86).
#
# ⛔ ONLY A CHANGE OF VIEWPOINT SEPARATES THEM. A hull that only ROTATES keeps
# beta_world fixed, so both branches are constant in the world and both fit
# every frame. That pair is unresolvable from this target alone, and the fuser
# says so -- it does not count, and it does not guess.
RULE_VIEWPOINT = 'viewpoint'
MIRROR_SLOPE = 2.0
# Each slope must sit within this of 0 or of 2 ...
SLOPE_BAND = 0.5
# ... and BOTH slopes must be pinned this tightly (regression standard error).
# MEASURED, 200 seeds per cell, 10 frames, target at 3 m, lateral 0.1-0.6 m,
# corner noise 0.5-3.0 px: at 0.25 the rule made ZERO wrong decisions in all
# 16 000 trials, deciding 200/200 at 0.3 m / 0.5 px and 17/200 at 0.6 m / 3 px.
# A fixed bearing-excursion floor instead (3.8 deg) still erred 5 times in 200
# at 2 px: the noise, not the excursion, is what has to be beaten, and the
# standard error measures exactly that. 0.25 puts 0 and 2 eight sigma apart.
SLOPE_SE_MAX = 0.25

# Legacy path only (one branch per frame): a rival cluster this large blocks a
# counted decision. See `fuse`.
LEGACY_RIVAL_MIN = 2


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
    slope: float = float('nan') # d(world yaw)/d(world bearing) of the winner
    reason: str = ''
    # When two hypotheses both fit, BOTH, camera-relative now -- so the
    # disagreement reaches the wire instead of being resolved by a vote.
    candidates: tuple = ()


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



def _fit(xs: list, ys: list) -> tuple:
    """Least-squares slope and its standard error. (nan, inf) if undefined."""
    n = len(xs)
    if n < 3:
        return float('nan'), float('inf')
    mx, my = sum(xs) / n, sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    if sxx <= 1e-12:
        return float('nan'), float('inf')
    slope = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx
    icpt = my - slope * mx
    rss = sum((y - (icpt + slope * x)) ** 2 for x, y in zip(xs, ys))
    return slope, math.sqrt(rss / (n - 2) / sxx)


def resolve_mirror(samples: list) -> dict:
    """Which of each frame's two branches is the target, judged by geometry.

    Every frame contributes BOTH branches, so the detector's preference for one
    -- the thing a biased corner detector corrupts -- casts no vote at all. The
    branches are tracked across frames by world-yaw continuity, and each track
    is regressed against the world bearing to the target: the true one is flat
    (slope 0), the mirror swings at slope 2 (see `MIRROR_SLOPE`).

    Returns a dict with `resolved`, the two tracks' world yaws, slopes and
    standard errors, and the bearing excursion that bought them.
    """
    rows = [(s.vehicle_yaw_deg + s.bearing_deg,
             s.vehicle_yaw_deg + s.yaw_deg,
             s.vehicle_yaw_deg + s.alt_yaw_deg) for s in samples]
    b0 = rows[0][0]
    A, B = [], []
    for wb, w1, w2 in rows:
        x = _wrap180(wb - b0)
        if not A:
            A.append((x, w1)); B.append((x, w2))
            continue
        keep = _angdiff(w1, A[-1][1]) + _angdiff(w2, B[-1][1])
        swap = _angdiff(w2, A[-1][1]) + _angdiff(w1, B[-1][1])
        if keep <= swap:
            A.append((x, w1)); B.append((x, w2))
        else:
            A.append((x, w2)); B.append((x, w1))

    def unwrap(track):
        ref = track[0][1]
        return [x for x, _ in track], [_wrap180(y - ref) + ref for _, y in track]
    xa, ya = unwrap(A)
    xb, yb = unwrap(B)
    sa, ea = _fit(xa, ya)
    sb, eb = _fit(xb, yb)
    out = dict(resolved=False, track_a=ya, track_b=yb, slope_a=sa, slope_b=sb,
               se_a=ea, se_b=eb, excursion=(max(xa) - min(xa)) if xa else 0.0,
               true='')
    if not (ea <= SLOPE_SE_MAX and eb <= SLOPE_SE_MAX):
        return out
    a_true = abs(sa) < SLOPE_BAND and abs(sb - MIRROR_SLOPE) < SLOPE_BAND
    b_true = abs(sb) < SLOPE_BAND and abs(sa - MIRROR_SLOPE) < SLOPE_BAND
    if a_true != b_true:
        out.update(resolved=True, true='a' if a_true else 'b')
    return out


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

        # Reprojection gates every frame: a pose that does not explain its own
        # pixels is not evidence for either branch.
        fitted = [s for s in live
                  if s.reproj_px <= self.max_reproj_px or s.reproj_px <= 0.0]

        # ⭐ BOTH BRANCHES, WHEN THE PRODUCER SENT THEM. Then the solver's own
        # preference never votes, and the viewpoint decides (`resolve_mirror`).
        paired = [s for s in fitted
                  if None not in (s.alt_yaw_deg, s.bearing_deg, s.vehicle_yaw_deg)
                  and all(math.isfinite(v) for v in
                          (s.yaw_deg, s.alt_yaw_deg, s.bearing_deg,
                           s.vehicle_yaw_deg))]
        if len(paired) >= self.min_poses:
            # The answer is for NOW: the newest heading in the window,
            # which may belong to a frame that carried only one branch.
            psi_now = next(s.vehicle_yaw_deg for s in reversed(live)
                           if s.vehicle_yaw_deg is not None)
            return self._fuse_paired(paired, psi_now)

        # LEGACY: one branch per frame, no viewpoint data. A frame the solver
        # could not distinguish (`ambiguity` -> 1) is a coin flip and is dropped.
        kept = [s for s in fitted if s.ambiguity <= self.max_ambiguity]
        if not kept:
            return Fused(False, considered=0,
                         reason=f'all {len(live)} poses failed the '
                                f'ambiguity/reprojection gates')
        groups = sorted(self._cluster([s.yaw_deg for s in kept]),
                        key=len, reverse=True)
        best = groups[0]
        rival = len(groups[1]) if len(groups) > 1 else 0
        support = len(best)
        if support < self.min_poses:
            return Fused(False, considered=len(kept), support=support,
                         rival=rival, reason=f'largest cluster has {support} '
                                             f'poses, needs {self.min_poses}')
        # ⛔ A SECOND SUPPORTED CLUSTER IS A MIRROR PAIR, and a majority does
        # not resolve it (#55: a 7/3 wrong-branch majority was accepted and
        # anchored a heading 54 deg against a true 9). Without the second
        # branch and the bearing there is no geometric test, so: refuse.
        # Two frames agreeing ELSEWHERE are a second hypothesis; one is a stray
        # detection. #55's own case was a 7/3 split, so a rival this small
        # must already block -- `min_poses` (4) let it through.
        if rival >= LEGACY_RIVAL_MIN:
            rv = _circular_median([kept[i].yaw_deg for i in groups[1]])
            bv = _circular_median([kept[i].yaw_deg for i in best])
            return Fused(False, considered=len(kept), support=support,
                         rival=rival, candidates=(bv, rv),
                         reason=f'two branches, {bv:+.1f} deg ({support}) and '
                                f'{rv:+.1f} deg ({rival}); a majority does not '
                                f'resolve a mirror pair and this producer sent '
                                f'no second branch to test')
        members = [kept[i] for i in best]
        yaws = [m.yaw_deg for m in members]
        centre = _circular_median(yaws)
        ranges = [m.range_m for m in members if m.range_m > 0.0]
        return Fused(True, yaw_deg=centre,
                     range_m=_median(ranges) if ranges else float('nan'),
                     support=support, considered=len(kept),
                     spread_deg=_median([_angdiff(y, centre) for y in yaws]),
                     rival=rival, rule='support')

    def _fuse_paired(self, paired: list, psi_now: float) -> Fused:
        """Both branches per frame: the viewpoint decides, or nobody does."""
        r = resolve_mirror(paired)
        ca = _wrap180(_circular_median(r['track_a']) - psi_now)
        cb = _wrap180(_circular_median(r['track_b']) - psi_now)
        ranges = [m.range_m for m in paired if m.range_m > 0.0]
        rng = _median(ranges) if ranges else float('nan')
        if not r['resolved']:
            return Fused(
                False, considered=len(paired), range_m=rng,
                support=len(paired), rival=len(paired), candidates=(ca, cb),
                reason=(f'mirror pair unresolved: {ca:+.1f} deg (slope '
                        f'{r["slope_a"]:+.2f}+/-{r["se_a"]:.2f}) or {cb:+.1f} '
                        f'deg (slope {r["slope_b"]:+.2f}+/-{r["se_b"]:.2f}) '
                        f'over {r["excursion"]:.1f} deg of bearing; both fit. '
                        f'Only a change of viewpoint separates them -- '
                        f'translate sideways, a turn in place cannot'))
        track = r['track_a'] if r['true'] == 'a' else r['track_b']
        centre_world = _circular_median(track)
        return Fused(
            True, yaw_deg=_wrap180(centre_world - psi_now), range_m=rng,
            support=len(paired), considered=len(paired),
            spread_deg=_median([_angdiff(y, centre_world) for y in track]),
            rival=len(paired), rule=RULE_VIEWPOINT,
            slope=r['slope_a'] if r['true'] == 'a' else r['slope_b'])

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
