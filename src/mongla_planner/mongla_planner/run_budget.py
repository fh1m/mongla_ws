"""What is still worth attempting, given the clock. Effort is not points.

⛔ TIER 5 IS NOT IN THE CAPABILITY MAP. The map runs 0-4 and stops at mission
composition. This is the layer above it, named here rather than assumed: Tiers
0-4 make the vehicle CAPABLE, and Tier 5 is what turns capability into a SCORED
RUN. The first item is the clock, because it is the constraint every other
decision sits inside and the one no amount of perception removes.

⛔ THE LESSON THIS ENCODES. BumblebeeAS built an 11-file, ~3,000-line slalom
pipeline with a 2000-particle filter, and no mother tree imports any of it --
they dead-reckon past the slalom. A team that could build the hard thing still
shipped waypoints where perception was hardest, because a run is scored on
points and bounded by minutes, not on what exists in the repository.

So a task is attempted when its WORST case fits what is left, not when its best
case does. A task that overruns does not merely fail: it consumes the budget of
every task after it, which is how one greedy attempt at 300 points costs three
easy ones.

⛔ AND THE ONE THING THE CLOCK MAY NOT SKIP. Surfacing, disarming and getting
out of the water are not tasks competing for budget -- they are the reserve the
budget is measured against. `remaining()` is net of that reserve, so a plan can
never spend the seconds that get the hull back.

Pure Python. No ROS.
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional


@dataclass(frozen=True)
class Task:
    """One scoring opportunity, with what it costs and what it is worth."""
    name: str
    points: int
    worst_case_s: float
    run: Optional[Callable] = None
    # A task whose points can still be taken cheaply after giving up on the
    # precise version -- their Selector[precise, blind_fire] as a number.
    fallback_s: float = 0.0
    fallback_points: int = 0


@dataclass(frozen=True)
class Verdict:
    attempt: bool
    mode: str = ''          # 'full' | 'fallback' | 'skip'
    reason: str = ''
    remaining_s: float = 0.0


class RunBudget:
    """The mission clock, and what it will still allow.

    `reserve_s` is the surface-and-disarm allowance. It is subtracted from
    everything the planner is allowed to see, so no amount of optimism can
    spend it.
    """

    def __init__(self, total_s: float, *, reserve_s: float = 45.0,
                 now: Optional[Callable[[], float]] = None):
        self.total_s = float(total_s)
        self.reserve_s = float(reserve_s)
        self._now = now or time.monotonic
        self._t0: Optional[float] = None
        self.log: List[str] = []

    def start(self) -> None:
        """⛔ Called on ARM, not on script start. A mission that spends two
        minutes waiting for a tether to be removed has not started its run, and
        counting that time makes every later decision pessimistic by exactly
        the wrong amount."""
        self._t0 = self._now()

    @property
    def started(self) -> bool:
        return self._t0 is not None

    def elapsed_s(self) -> float:
        return 0.0 if self._t0 is None else self._now() - self._t0

    def remaining_s(self) -> float:
        """Seconds a task may spend. Never includes the reserve, never negative."""
        left = self.total_s - self.reserve_s - self.elapsed_s()
        return max(0.0, left)

    def verdict(self, task: Task) -> Verdict:
        """Attempt it fully, take the cheap points, or skip.

        The order is deliberate. A task whose full version no longer fits is
        NOT skipped while its fallback still fits, because the fallback exists
        exactly for the case where there is not enough left to do it properly.
        """
        left = self.remaining_s()
        if not self.started:
            return Verdict(True, 'full', 'clock not started; nothing to ration',
                           left)
        if task.worst_case_s <= left:
            return Verdict(True, 'full',
                           f'{task.worst_case_s:.0f}s worst case fits in '
                           f'{left:.0f}s', left)
        if task.fallback_s > 0.0 and task.fallback_s <= left:
            return Verdict(True, 'fallback',
                           f'no room for {task.worst_case_s:.0f}s, but the '
                           f'{task.fallback_points}-point fallback fits in '
                           f'{left:.0f}s', left)
        return Verdict(False, 'skip',
                       f'{task.name}: needs {task.worst_case_s:.0f}s '
                       f'(fallback {task.fallback_s:.0f}s), {left:.0f}s left',
                       left)

    def plan(self, tasks: List[Task]) -> List[tuple]:
        """Dry-run the whole list against the clock as it stands NOW.

        ⚠ A PLAN, NOT A PROMISE. It assumes every task takes its worst case,
        which is what makes it safe to act on and also means the real run will
        usually have room for more. Re-ask `verdict` between tasks rather than
        trusting this list to the end.
        """
        out, spent = [], 0.0
        left0 = self.remaining_s()
        for t in tasks:
            left = left0 - spent
            if t.worst_case_s <= left:
                out.append((t.name, 'full', t.points))
                spent += t.worst_case_s
            elif t.fallback_s > 0.0 and t.fallback_s <= left:
                out.append((t.name, 'fallback', t.fallback_points))
                spent += t.fallback_s
            else:
                out.append((t.name, 'skip', 0))
        return out

    def projected_points(self, tasks: List[Task]) -> int:
        return sum(p for _, _, p in self.plan(tasks))


def by_points_per_second(tasks: List[Task]) -> List[Task]:
    """Order tasks by value density, highest first.

    ⚠ ORDERING IS NOT FREE ON A COURSE. This ranks by points per second and
    ignores where the props are, so it is a planning INPUT, not a running
    order: swimming across the pool to take a dense task and back again can
    cost more than it earns. Use it to decide what to drop, not what order to
    visit.
    """
    return sorted(tasks, key=lambda t: (t.points / max(1e-6, t.worst_case_s)),
                  reverse=True)
