"""`mongla.run_plan()` — a declared plan, re-decided from the live clock.

These tests EXECUTE the plan loop. The point of the surface is that a mission
author declares what each task is worth and what it costs, and the DSL handles
the clock, the refusals, the deadlines and the reporting — so each of those is
asserted here rather than described.

The three behaviours that matter, and why each exists:

  * a step whose backend verb is refused is SKIPPED, not attempted.
    This is the J04 lesson: the retired FSM carried a capability flag with zero
    callers, dispatched the refused verb anyway, and ended the run one state
    after DIVE.
  * a step that overruns is ABANDONED and the plan CONTINUES.
    A task that cannot be finished must not cost the run.
  * `can()` reads the backend's own refusal set.
    A hand-copied list here would be the second place to disagree.

Run:
    pytest -q -p no:anyio src/mongla_planner/test/test_run_plan.py
"""
from unittest.mock import MagicMock

import pytest

from mongla_planner.client import MissionRefused, TaskAbandoned
from mongla_planner.mongla_dsl import MonglaMission


# ── a mission with the ROS surface stubbed, and nothing else ──────────────────

def _mission(backend: str = 'srot') -> MonglaMission:
    m = MonglaMission.__new__(MonglaMission)      # no ROS, no client
    m.log = MagicMock()
    m._scoreboard = []
    m._budget = None
    m._backend_cache = backend
    m._task_depth = 0
    return m


class _NullTask:
    """Stand-in for `mongla.task(...)`: a context manager that does nothing."""
    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False


def _no_deadlines(m):
    m.task = lambda *a, **k: _NullTask()
    return m


# ── can(): one truth, read from the backend ──────────────────────────────────

def test_can_reads_the_backends_own_refusal_set():
    """Not a copy. `lock_heading` is in srot_fc.UNSUPPORTED_VERBS."""
    m = _mission('srot')

    assert m.can('lock_heading') is False
    assert m.can('move_forward') is True


def test_everything_is_allowed_on_pixhawk():
    m = _mission('pixhawk')

    assert m.can('lock_heading') is True


def test_require_raises_on_the_deck_not_underwater():
    m = _mission('srot')

    with pytest.raises(MissionRefused) as exc:
        m.require('lock_heading', why='the plan holds a heading through the leg')

    assert 'lock_heading' in str(exc.value)
    assert 'the plan holds a heading' in str(exc.value)


def test_require_is_silent_when_the_verb_works():
    _mission('srot').require('move_forward')      # must not raise


# ── run_plan(): the loop ─────────────────────────────────────────────────────

def test_a_step_needing_a_refused_verb_is_skipped_not_attempted():
    m = _no_deadlines(_mission('srot'))
    ran = []

    out = m.run_plan([
        m.step('locked', points=100, worst_case_s=10,
               run=lambda _: ran.append('locked'), needs='lock_heading'),
        m.step('plain', points=50, worst_case_s=10,
               run=lambda _: ran.append('plain')),
    ])

    assert ran == ['plain'], 'a step needing a refused verb was attempted'
    assert ('locked', 'unsupported') in out


def test_an_overrun_step_is_abandoned_and_the_plan_continues():
    """The deadline cancels the goal; run_plan must not let that end the run."""
    m = _no_deadlines(_mission('srot'))
    ran = []

    def _overruns(_):
        raise TaskAbandoned('deadline passed mid-goal; goal cancelled')

    out = m.run_plan([
        m.step('slow', points=100, worst_case_s=10, run=_overruns),
        m.step('after', points=50, worst_case_s=10,
               run=lambda _: ran.append('after')),
    ])

    assert ('slow', 'abandoned') in out
    assert ran == ['after'], 'the plan stopped after an abandoned step'


def test_the_fallback_runs_when_the_budget_says_fallback():
    m = _no_deadlines(_mission('srot'))
    ran = []
    # 30 s of spendable clock: the full task wants 120 s, the fallback 20 s
    from mongla_planner.run_budget import RunBudget
    m._budget = RunBudget(total_s=75.0, reserve_s=45.0)
    m._budget.start()

    out = m.run_plan([
        m.step('torpedo', points=300, worst_case_s=120,
               run=lambda _: ran.append('full'),
               fallback=lambda _: ran.append('blind'),
               fallback_s=20.0, fallback_points=100),
    ])

    assert ran == ['blind'], f'expected the fallback, ran {ran}'
    assert ('torpedo', 'fallback') in out


def test_a_step_with_no_authored_fallback_is_skipped_not_crashed():
    m = _no_deadlines(_mission('srot'))
    from mongla_planner.run_budget import RunBudget
    m._budget = RunBudget(total_s=75.0, reserve_s=45.0)
    m._budget.start()

    out = m.run_plan([
        m.step('torpedo', points=300, worst_case_s=120,
               run=lambda _: None, fallback_s=20.0, fallback_points=100),
    ])

    assert ('torpedo', 'skipped') in out


def test_every_outcome_reaches_the_scoreboard():
    """A run has to explain itself afterwards."""
    m = _no_deadlines(_mission('srot'))

    m.run_plan([
        m.step('locked', points=10, worst_case_s=5,
               run=lambda _: None, needs='lock_heading'),
        m.step('plain', points=10, worst_case_s=5, run=lambda _: None),
    ])

    rows = ' '.join(r['cmd'] for r in m._scoreboard)
    assert 'note:locked' in rows, 'the skipped step left no trace'
    assert 'budget:plain' in rows, 'the attempted step left no verdict'


def test_order_by_value_sorts_by_points_per_second():
    m = _no_deadlines(_mission('srot'))
    ran = []

    m.run_plan([
        m.step('cheap_slow', points=10, worst_case_s=100,
               run=lambda _: ran.append('cheap_slow')),
        m.step('rich_fast', points=300, worst_case_s=10,
               run=lambda _: ran.append('rich_fast')),
    ], order='by_value')

    assert ran == ['rich_fast', 'cheap_slow']


def test_as_written_is_the_default_order():
    """Ordering by value ignores where the props are; it must be opt-in."""
    m = _no_deadlines(_mission('srot'))
    ran = []

    m.run_plan([
        m.step('cheap_slow', points=10, worst_case_s=100,
               run=lambda _: ran.append('cheap_slow')),
        m.step('rich_fast', points=300, worst_case_s=10,
               run=lambda _: ran.append('rich_fast')),
    ])

    assert ran == ['cheap_slow', 'rich_fast']


def test_a_step_takes_a_callable_never_a_verb_name():
    """`resilience.py` holds this rule and the plan must not weaken it:
    a mechanism that accepts a verb name can be pointed at `disarm`."""
    m = _no_deadlines(_mission('srot'))

    step = m.step('x', points=1, worst_case_s=1, run=lambda _: None)

    assert callable(step['run'])
