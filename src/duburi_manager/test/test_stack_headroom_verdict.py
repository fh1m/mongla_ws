"""The board's per-task stack high-water marks, graded at preflight.

The board has emitted six STK_* values since the telemetry was added, and no
consumer read them. A stack overflow hard-resets the control board, so this is
the one crash that is visible in advance in data we already receive.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from duburi_manager.bringup_check import (                    # noqa: E402
    FAIL, PASS, WARN, _stack_headroom_verdict,
)

# The values measured live on the board, 2026-09-11.
MEASURED = {'STK_CTL': 4980.0, 'STK_DSH': 2868.0, 'STK_LORA': 4208.0,
            'STK_MAV': 5144.0, 'STK_SEN': 6500.0, 'STK_UI': 4100.0}


def test_the_measured_board_passes_and_names_its_lowest_task():
    grade, _title, detail = _stack_headroom_verdict(MEASURED)
    assert grade == PASS
    assert 'STK_DSH=2868' in detail


def test_one_low_task_fails_the_whole_check():
    """The MINIMUM decides. An average would hide the task about to crash."""
    named = dict(MEASURED, STK_CTL=300.0)
    grade, _t, detail = _stack_headroom_verdict(named)
    assert grade == FAIL
    assert 'STK_CTL' in detail


def test_a_marginal_task_warns():
    grade, _t, _d = _stack_headroom_verdict(dict(MEASURED, STK_UI=800.0))
    assert grade == WARN


def test_no_values_is_unknown_not_healthy():
    """Absence is not headroom. A firmware that stops reporting must not read
    as a board with plenty of stack."""
    grade, _t, _d = _stack_headroom_verdict({})
    assert grade == WARN


def test_a_missing_task_is_named_in_a_pass():
    named = dict(MEASURED)
    del named['STK_LORA']
    grade, _t, detail = _stack_headroom_verdict(named)
    assert grade == PASS
    assert 'STK_LORA' in detail
