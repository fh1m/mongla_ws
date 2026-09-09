"""The companion is a Pi, and it had no power check.

Section L grades `nvpmodel`, which is Jetson-only -- on the vehicle it returns
PASS "not a Jetson (nvpmodel absent) -- skipped". So the board every mission
actually runs on was graded on power by nothing at all, while the change map
lists a measured `get_throttled = 0x50000` (under-voltage since boot) as a hard
gate before water.

`vcgencmd get_throttled` is a bitmask whose LOW half is "now" and HIGH half is
"has happened since boot". The sticky half is the one a preflight needs: a
brown-out that already cost frames clears itself the instant the load drops.
"""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from duburi_manager.bringup_check import (            # noqa: E402
    FAIL, PASS, WARN, _throttled_verdict)


def test_clean_is_a_pass_that_refuses_to_certify():
    """0x0 is good news and NOT a load test -- the thrusters were not drawing.
    A preflight line that implied otherwise would be worse than no line."""
    st, det = _throttled_verdict(0x0)
    assert st == PASS
    assert 'not a load test' in det


def test_under_voltage_happening_NOW_fails():
    st, det = _throttled_verdict(0x1)
    assert st == FAIL
    assert 'under-voltage' in det


def test_the_map_s_measured_value_is_a_warning_not_a_pass():
    """0x50000 = bits 16 and 18: under-voltage and throttling HAVE occurred.
    That is the reading the change map recorded, and it must not read as OK."""
    st, det = _throttled_verdict(0x50000)
    assert st == WARN
    assert 'HAS OCCURRED' in det and 'since boot' in det


def test_a_past_event_is_not_reported_as_present():
    st, det = _throttled_verdict(0x10000)
    assert st == WARN
    assert 'ACTIVE NOW' not in det


def test_active_beats_historical_when_both_are_set():
    """Both halves set: the live fault is the headline."""
    st, det = _throttled_verdict(0x50001)
    assert st == FAIL and 'ACTIVE NOW' in det


def test_a_soft_temperature_limit_alone_is_not_a_power_fault():
    """Bit 19 is thermal, not supply. Grading it as an electrical fault would
    send someone to change a PSU over a warm afternoon."""
    st, _ = _throttled_verdict(0x80000)
    assert st == PASS


@pytest.mark.parametrize('raw', [0x0, 0x1, 0x50000, 0x80000, 0xF000F])
def test_every_verdict_is_a_known_status_and_says_something(raw):
    st, det = _throttled_verdict(raw)
    assert st in (PASS, WARN, FAIL)
    assert det.strip()


def test_the_check_is_REGISTERED_not_merely_defined():
    """A grader nothing calls is the defect class this repo keeps producing."""
    src = (Path(__file__).resolve().parents[1] / 'duburi_manager'
           / 'bringup_check.py').read_text()
    assert '_check_pi_power()' in src.split('def main', 1)[-1] or \
           'st, det = _check_pi_power()' in src, (
        '_check_pi_power is defined but never called from the preflight')
    assert 'L2. Raspberry Pi power' in src
