"""The depth-loop SIGN verdict — the check the SURFACE failsafe depends on.

`depth_control.cpp` says the loop "has NEVER run closed", and its sign WAS
inverted once ("commanded the opposite of what it wanted, on every axis"). The
fix is unvalidated. A leak, a low thruster battery and a GCS loss all route to
SURFACE, which calls `depth::update()` — so if the sign were still wrong, the
emergency ascent drives the vehicle DOWN.

`tools/depth_sign_check.py` makes that checkable disarmed with a thumb over the
Bar30 port. The decision is pure so it is testable without a board.
"""
import importlib.util
import sys
from pathlib import Path

_TOOL = Path(__file__).resolve().parents[3] / 'tools' / 'depth_sign_check.py'


def _load():
    spec = importlib.util.spec_from_file_location('depth_sign_check', _TOOL)
    mod = importlib.util.module_from_spec(spec)
    sys.modules['depth_sign_check'] = mod
    spec.loader.exec_module(mod)
    return mod


def test_tool_exists():
    assert _TOOL.is_file(), f'{_TOOL} missing'


def test_pressing_the_port_must_command_ASCEND():
    """Pressurise = board reads DEEPER = must command POSITIVE = ascend."""
    m = _load()
    v, txt = m.sign_verdict(-0.22, +0.40)
    assert v == 'PASS', txt


def test_the_INVERTED_case_is_a_FAIL_and_says_so():
    """⛔ The defect this exists to catch. Deeper -> descend is a runaway."""
    m = _load()
    v, txt = m.sign_verdict(-0.22, -0.70)
    assert v == 'FAIL'
    assert 'DO NOT DIVE' in txt
    assert 'AWAY' in txt


def test_a_stimulus_too_small_is_NOT_a_pass():
    """A delta under the floor cannot validate a sign. Reporting PASS from noise
    is the exact failure this whole file is written against."""
    m = _load()
    for pressed in (-0.22, -0.20, -0.24):
        v, _ = m.sign_verdict(-0.22, pressed)
        assert v == 'INCONCLUSIVE', f'{pressed} should be inconclusive, got {v}'


def test_absent_DEPTH_CMD_is_a_FINDING_not_a_pass():
    """The board SUPPRESSES DEPTH_CMD when the barometer is unhealthy — absence
    is not zero, and it is certainly not a pass."""
    m = _load()
    assert m.sign_verdict(None, 0.4)[0] == 'INCONCLUSIVE'
    assert m.sign_verdict(-0.22, None)[0] == 'INCONCLUSIVE'
    assert m.sign_verdict(None, None)[0] == 'INCONCLUSIVE'
    assert 'BARO_HEALTH' in m.sign_verdict(None, None)[1]


def test_the_direction_convention_is_pinned():
    """If anyone ever 'fixes' this by flipping the comparison, this fails. The
    convention comes from three independent firmware sources: THRUSTER_MAP.md's
    axis table, DEPTH_LOST_ASCENT being applied POSITIVE to surface without a
    sensor, and MANUAL passing throttle straight through (z > 500 = up)."""
    m = _load()
    assert m.sign_verdict(0.0, +1.0)[0] == 'PASS'
    assert m.sign_verdict(0.0, -1.0)[0] == 'FAIL'
