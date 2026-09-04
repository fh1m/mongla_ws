"""Detection continuity: the statistics the whole lock ladder is tuned on.

WHY THIS MODULE EXISTS
----------------------
Six constants protect a target lock -- the freshness ramp, `coast_s` (0.8),
`lost_grace_s` (1.0), `_STALE_LIMIT_S` (1.0), the Kalman `max_predict_s` (1.5)
and the tracker's `track_buffer` (5.0 s). Every one is sized from the
detection RATE and from the others' ordering. Not one is sized from the
quantity they actually defend against: HOW LONG A REAL GAP IN WATER LASTS.

Nothing in this repo has ever recorded that. `check_tracker.py` comes closest
and collapses it to a scalar -- a 5 % "predicted ratio" is one 400 ms blackout
or a hundred single-frame flickers, and it reports the same number for both.
That distinction is the entire question.

WHAT IS MEASURED, AND WHY EACH ONE
----------------------------------
`gaps`        every interval where the target was absent, as a DISTRIBUTION.
              A mean hides a stream that stopped; `srot_replay.py:13` already
              records that lesson for MAVLink. p50/p90/p99/max, plus the
              count over each ladder threshold -- which is the direct answer
              to "is coast_s=0.8 generous or tight".
`presence`    fraction of frames the target was seen at all. The headline:
              1.0 is the golden run.
`reacquire`   how long after a gap until the box is stable again. A detector
              that flickers back for one frame has not recovered.
`score`       confidence distribution over time. `hailo-vision.md:232` flags
              our only score measurement as "distribution, NOT recall -- the
              camera was pointed at a room". With labelled competition
              footage that limitation is gone.
`jitter`      per-frame centre movement, in NORMALISED units so it is
              comparable across resolutions. Distinguishes a box that is
              present-but-useless from one that is genuinely tracking.
`switches`    identity changes on what is spatially the same object.

Pure functions over a list of observations. No ROS, no camera, no model --
so the same code scores a live topic, a bag, and an offline video replay, and
a change in any of those is measured by the same ruler.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple


@dataclass
class Obs:
    """One frame's worth of what the detector said about ONE target class.

    `t` is the frame's own capture time, not the time we looked at it.
    `cx`/`cy` are NORMALISED centre coordinates ([0,1]) so jitter is
    comparable between a 640x360 and a 1920x1080 source.
    """
    t: float
    seen: bool
    score: float = 0.0
    cx: float = float('nan')
    cy: float = float('nan')
    track_id: int = -1
    area: float = 0.0          # normalised box area, for range context


@dataclass
class Gap:
    """One interval where the target was not detected."""
    start_t: float
    end_t: float
    frames: int

    @property
    def duration(self) -> float:
        return self.end_t - self.start_t


def _pct(values: Sequence[float], q: float) -> float:
    if not values:
        return float('nan')
    s = sorted(values)
    return s[min(len(s) - 1, int(round(q / 100.0 * (len(s) - 1))))]


def find_gaps(obs: Sequence[Obs]) -> List[Gap]:
    """Every absence BETWEEN two sightings.

    THE GAP IS MEASURED FROM THE LAST SIGHTING, not from the first missing
    frame, because that is what the control loop compares against: `coast_s`
    is checked as `monotonic() - _last_real[track_id]` (`vision_state.py:483`)
    and `_last_real` is stamped on the last REAL detection. Measuring from the
    first miss would under-report every gap by one frame period -- 33 ms at
    30 Hz, which against the 0.20 s freshness rung is a 17 % error, in the
    direction that flatters us.

    Leading and trailing absences are excluded deliberately: before the first
    sighting the target may simply not be in view, and after the last one the
    clip ended. Counting those would make a video that opens on a wall look
    like a detector failure -- the same class of error as
    `hailo-vision.md:232`'s room-not-prop measurement.
    """
    first = next((i for i, o in enumerate(obs) if o.seen), None)
    last = next((i for i in range(len(obs) - 1, -1, -1) if obs[i].seen), None)
    if first is None or last is None or first >= last:
        return []

    gaps: List[Gap] = []
    last_seen_t: Optional[float] = None
    run_frames = 0
    for o in obs[first:last + 1]:
        if o.seen:
            if run_frames and last_seen_t is not None:
                gaps.append(Gap(last_seen_t, o.t, run_frames))
            last_seen_t, run_frames = o.t, 0
        else:
            run_frames += 1
    return gaps


def count_switches(obs: Sequence[Obs], gate: float = 0.15) -> int:
    """Identity changes on what is spatially the SAME object.

    A bare "the id changed" count is not a switch count: a genuinely new
    object entering frame also changes the id. Only a NEW id whose centre is
    within `gate` of where the old id was counts -- that is the same physical
    thing being relabelled, which is what breaks a lock keyed on `locked_id`.
    """
    switches = 0
    prev: Optional[Obs] = None
    for o in obs:
        if not o.seen or o.track_id < 0:
            continue
        if (prev is not None and o.track_id != prev.track_id
                and math.isfinite(o.cx) and math.isfinite(prev.cx)):
            if math.hypot(o.cx - prev.cx, o.cy - prev.cy) <= gate:
                switches += 1
        prev = o
    return switches


def jitter(obs: Sequence[Obs]) -> List[float]:
    """Per-frame normalised centre movement between CONSECUTIVE sightings.

    Skips across gaps: the jump either side of a 500 ms absence is real
    target motion, not jitter, and including it would let a flickering
    detector look smooth by comparison.
    """
    out: List[float] = []
    prev: Optional[Obs] = None
    for o in obs:
        if not o.seen or not math.isfinite(o.cx):
            prev = None
            continue
        if prev is not None:
            out.append(math.hypot(o.cx - prev.cx, o.cy - prev.cy))
        prev = o
    return out


def reacquire_times(obs: Sequence[Obs], stable_frames: int = 3) -> List[float]:
    """After each gap, how long until the target is STABLY back.

    "Stably" because one lucky frame is not a recovery -- `align_stable_frames`
    exists for exactly that reason, and a detector that returns for a single
    frame and drops again has not given the control loop anything to steer on.
    """
    out: List[float] = []
    i = 0
    n = len(obs)
    while i < n:
        if obs[i].seen:
            i += 1
            continue
        # a gap: walk to its end
        while i < n and not obs[i].seen:
            i += 1
        if i >= n:
            break
        gap_end_t = obs[i].t
        run = 0
        j = i
        while j < n and run < stable_frames:
            if obs[j].seen:
                run += 1
                j += 1
            else:
                run = 0
                j += 1
        if run >= stable_frames:
            out.append(obs[j - 1].t - gap_end_t)
        i = j
    return out


# The ladder, so the report answers "is coast_s enough" directly rather than
# leaving it to be read off a percentile table. Values from
# vision_tunables.py / motion_vision.py / config/tracker.yaml.
LADDER: Tuple[Tuple[str, float], ...] = (
    ('freshness zero (0.20 s)', 0.20),
    ('coast_s (0.80 s)',        0.80),
    ('lost_grace_s (1.00 s)',   1.00),
    ('kalman predict (1.50 s)', 1.50),
    ('track_buffer (5.00 s)',   5.00),
)


@dataclass
class Report:
    label: str = ''
    frames: int = 0
    seen: int = 0
    duration_s: float = 0.0
    gaps: List[Gap] = field(default_factory=list)
    scores: List[float] = field(default_factory=list)
    jitters: List[float] = field(default_factory=list)
    reacquires: List[float] = field(default_factory=list)
    switches: int = 0

    @property
    def presence(self) -> float:
        return self.seen / self.frames if self.frames else float('nan')

    @property
    def rate_hz(self) -> float:
        return self.seen / self.duration_s if self.duration_s > 0 else float('nan')

    def over(self, threshold: float) -> int:
        """Gaps longer than a ladder rung -- i.e. losses that rung cannot cover."""
        return sum(1 for g in self.gaps if g.duration > threshold)

    def text(self) -> str:
        L: List[str] = []
        A = L.append
        A(f'\n  === {self.label} ===')
        A(f'    frames {self.frames}  over {self.duration_s:.1f}s   '
          f'seen {self.seen} ({100.0 * self.presence:.1f} %)   '
          f'{self.rate_hz:.1f} Hz')
        if not self.gaps:
            A('    NO GAPS between first and last sighting -- an unbroken lock.')
        else:
            d = [g.duration for g in self.gaps]
            A(f'    gaps  n={len(self.gaps)}   '
              f'p50 {1000 * _pct(d, 50):6.0f}  p90 {1000 * _pct(d, 90):6.0f}  '
              f'p99 {1000 * _pct(d, 99):6.0f}  max {1000 * max(d):6.0f} ms')
            A(f'    longest gap is {max(d):.2f}s  '
              f'({sum(g.frames for g in self.gaps)} frames lost in total)')
            A('    gaps NOT covered by each rung of the lock ladder:')
            for name, thr in LADDER:
                n = self.over(thr)
                mark = '  <-- LOSSES' if n else ''
                A(f'        {name:<26} {n:4d}{mark}')
        if self.scores:
            A(f'    score  p10 {_pct(self.scores, 10):.3f}  '
              f'p50 {_pct(self.scores, 50):.3f}  '
              f'p90 {_pct(self.scores, 90):.3f}  max {max(self.scores):.3f}')
        if self.jitters:
            A(f'    jitter p50 {_pct(self.jitters, 50):.4f}  '
              f'p95 {_pct(self.jitters, 95):.4f}  max {max(self.jitters):.4f} '
              f'(normalised centre movement / frame)')
        if self.reacquires:
            A(f'    reacquire (3 stable frames) p50 '
              f'{1000 * _pct(self.reacquires, 50):.0f}  '
              f'max {1000 * max(self.reacquires):.0f} ms')
        A(f'    id switches (same place, new id): {self.switches}')
        return '\n'.join(L)


def analyse(obs: Sequence[Obs], label: str = '') -> Report:
    """Everything above, in one pass."""
    if not obs:
        return Report(label=label)
    return Report(
        label=label,
        frames=len(obs),
        seen=sum(1 for o in obs if o.seen),
        duration_s=obs[-1].t - obs[0].t,
        gaps=find_gaps(obs),
        scores=[o.score for o in obs if o.seen],
        jitters=jitter(obs),
        reacquires=reacquire_times(obs),
        switches=count_switches(obs),
    )
