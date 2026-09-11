"""The clock decides what is worth attempting. Effort is not points.

A task that overruns does not merely fail, it spends the budget of every task
after it. The decisive tests are that a task is judged on its WORST case, that
the fallback is reached instead of a skip, and that the surface-and-disarm
reserve cannot be spent by any amount of optimism.
"""
import pytest

from duburi_planner.run_budget import (
    RunBudget, Task, Verdict, by_points_per_second,
)


class _Clock:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t

    def advance(self, s):
        self.t += s


GATE = Task('gate', points=100, worst_case_s=60.0)
TORPEDO = Task('torpedo', points=300, worst_case_s=180.0,
               fallback_s=20.0, fallback_points=100)
BIN = Task('bin', points=200, worst_case_s=90.0)


def _budget(total=600.0, reserve=45.0):
    clock = _Clock()
    b = RunBudget(total, reserve_s=reserve, now=clock)
    return b, clock


# --- the reserve ------------------------------------------------------------


def test_the_reserve_is_never_visible_to_a_planner():
    b, _ = _budget(total=600.0, reserve=45.0)
    b.start()
    assert b.remaining_s() == pytest.approx(555.0)


def test_remaining_never_goes_negative():
    b, clock = _budget(total=100.0, reserve=45.0)
    b.start()
    clock.advance(500.0)
    assert b.remaining_s() == 0.0


def test_the_clock_starts_on_arm_not_on_script_start():
    # A mission waiting for a tether has not started its run, and counting that
    # makes every later decision pessimistic by exactly the wrong amount.
    b, clock = _budget()
    clock.advance(120.0)
    assert b.elapsed_s() == 0.0
    b.start()
    assert b.elapsed_s() == 0.0


# --- the verdict ------------------------------------------------------------


def test_a_task_is_judged_on_its_WORST_case():
    b, clock = _budget(total=200.0, reserve=45.0)   # 155 s usable
    b.start()
    clock.advance(100.0)                            # 55 s left
    assert b.verdict(GATE).mode == 'skip'           # needs 60 s worst case


def test_a_task_that_fits_is_attempted_fully():
    b, _ = _budget()
    b.start()
    assert b.verdict(GATE).mode == 'full'


def test_the_fallback_is_reached_instead_of_a_skip():
    # THE POINT. No room to do it properly is exactly when the cheap points
    # matter, and skipping straight past them is how a run scores zero on a
    # task it could have half-scored.
    b, clock = _budget(total=200.0, reserve=45.0)
    b.start()
    clock.advance(120.0)                            # 35 s left
    v = b.verdict(TORPEDO)
    assert v.attempt and v.mode == 'fallback'
    assert '100-point fallback' in v.reason


def test_a_task_with_no_fallback_and_no_room_is_skipped_with_the_numbers():
    b, clock = _budget(total=200.0, reserve=45.0)
    b.start()
    clock.advance(140.0)
    v = b.verdict(BIN)
    assert not v.attempt and v.mode == 'skip'
    assert '90s' in v.reason and 'left' in v.reason


def test_an_unstarted_clock_rations_nothing():
    # Rationing before the run begins would refuse the first task of every
    # mission that forgot to call start().
    b, _ = _budget()
    assert b.verdict(TORPEDO).mode == 'full'


# --- the plan ---------------------------------------------------------------


def test_the_plan_spends_the_budget_in_order():
    b, _ = _budget(total=300.0, reserve=45.0)       # 255 s usable
    b.start()
    got = b.plan([GATE, BIN, TORPEDO])              # 60 + 90 = 150, then 180
    assert got[0] == ('gate', 'full', 100)
    assert got[1] == ('bin', 'full', 200)
    assert got[2] == ('torpedo', 'fallback', 100)   # 105 s left, 180 needed


def test_the_plan_projects_what_the_run_would_score():
    b, _ = _budget(total=300.0, reserve=45.0)
    b.start()
    assert b.projected_points([GATE, BIN, TORPEDO]) == 400


def test_a_long_first_task_starves_the_rest_which_is_the_whole_warning():
    b, _ = _budget(total=250.0, reserve=45.0)       # 205 s
    b.start()
    got = b.plan([TORPEDO, GATE, BIN])              # torpedo takes 180
    assert got[0][1] == 'full'
    assert [m for _, m, _ in got[1:]] == ['skip', 'skip']


# --- ordering ---------------------------------------------------------------


def test_value_density_ranks_the_cheap_points_first():
    order = [t.name for t in by_points_per_second([GATE, TORPEDO, BIN])]
    assert order[0] == 'bin'            # 200/90 beats 100/60 and 300/180
    assert order[-1] == 'torpedo'
