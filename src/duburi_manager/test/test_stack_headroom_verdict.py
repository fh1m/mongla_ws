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


def test_the_implausible_depth_line_uses_the_telemetry_sign():
    """⛔ Measured 2026-09-14: one preflight printed "depth telemetry +1.19 m" and
    "board reads -1.22 m" -- the same baro offset in two sign conventions. The
    board is positive-down internally; the operator's convention is negative =
    submerged. A baro that thinks it is ABOVE zero must print a POSITIVE number."""
    from duburi_manager.bringup_check import _depth_loop_verdict

    # DEPTH_CMD -0.66 at DEPTH_P 0.5 is the live reading: board depth -1.22 m
    # positive-down, i.e. 1.22 m above zero.
    grade, _t, detail = _depth_loop_verdict(-0.66, 0.5)
    assert grade == FAIL
    assert '+1.22 m' in detail, detail


# --------------------------------------------------------------------------- #
#  free heap, beside the stacks
# --------------------------------------------------------------------------- #
from duburi_manager.bringup_check import (                    # noqa: E402
    HEAP_BASELINE_BYTES, HEAP_FAIL_BYTES, _heap_verdict,
)


def test_the_measured_heap_passes():
    """156180 B is what both recorded Pi bench sessions show, flat."""
    assert _heap_verdict({'HEAP': 156180.0})[0] == PASS


def test_a_quarter_gone_warns():
    assert _heap_verdict({'HEAP': HEAP_BASELINE_BYTES * 0.70})[0] == WARN
    assert _heap_verdict({'HEAP': HEAP_BASELINE_BYTES * 0.80})[0] == PASS


def test_near_exhaustion_fails():
    assert _heap_verdict({'HEAP': HEAP_FAIL_BYTES - 1.0})[0] == FAIL


def test_no_heap_is_unknown_not_healthy():
    assert _heap_verdict({})[0] == WARN


def test_the_heap_verdict_is_actually_graded_in_the_srot_section():
    src = (Path(__file__).resolve().parents[1] / 'duburi_manager'
           / 'bringup_check.py').read_text()
    assert 'out.append(_heap_verdict(named))' in src
