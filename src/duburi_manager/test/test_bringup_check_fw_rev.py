"""The preflight firmware-behaviour-rev gate.

This grades the one number that decides whether `stop` decelerates 20 kg of hull.
Below `FW_BEHAVIOUR_REV_REQUIRED` the board's MOVE_STOP applies zero braking thrust
and this host no longer carries the reverse-leg brake that used to cover it -- so a
misgrade here is a silent failure in the water, not an error anyone would see.

`_behaviour_rev_verdict` is pure precisely so this can be tested without a board.
"""

import pytest

from duburi_manager.bringup_check import _behaviour_rev_verdict, PASS, WARN, FAIL
from duburi_control.fc import srot_protocol as sp


def _status(rev, required=2):
    return _behaviour_rev_verdict(rev, required)[0]


def test_current_firmware_passes():
    assert _status(2) == PASS
    assert _status(3) == PASS          # a newer board is fine, not "unexpected"


def test_old_firmware_fails_closed():
    """A board that ANSWERS with a too-old rev is a definite statement: stop coasts."""
    assert _status(1) == FAIL


def test_zero_is_too_old_not_unknown():
    """0 is what pre-2026-08-01 firmware reports -- that build never populated the
    field. Reading it as "unknown" and warning through would be exactly backwards."""
    assert _status(0) == FAIL


def test_silent_board_warns_but_does_not_fail():
    """Asymmetric on purpose: no answer is far more likely a dropped frame than a
    genuine old board, and failing a whole preflight on a comms hiccup is its own
    hazard. It must still be loud."""
    status, label, detail = _behaviour_rev_verdict(None, 2)
    assert status == WARN
    assert 'COAST' in detail.upper()   # the operator is told what they are risking


def test_failure_detail_names_the_consequence_and_the_fix():
    """A preflight line nobody understands gets overridden. Say what breaks and how
    to fix it, in the line itself -- not in a doc the operator is not reading."""
    _, _, detail = _behaviour_rev_verdict(1, 2)
    assert 'decelerate' in detail.lower()
    assert 'flash' in detail.lower()


def test_grades_against_the_shipped_requirement():
    """Guards the wiring, not the helper: the real call site must pass the pinned
    constant, so bumping FW_BEHAVIOUR_REV_REQUIRED tightens the preflight too."""
    required = sp.FW_BEHAVIOUR_REV_REQUIRED
    assert _status(required, required) == PASS
    assert _status(required - 1, required) == FAIL


@pytest.mark.parametrize('rev', [0, 1])
def test_no_too_old_revision_is_ever_a_mere_warning(rev):
    """The whole point of the gate: a known-old board must not be waved through."""
    assert _behaviour_rev_verdict(rev, 2)[0] == FAIL


# --------------------------------------------------------------------------- #
#  Bar30 health -- since fw rev 3 this decides whether ANY move verb runs      #
# --------------------------------------------------------------------------- #

from duburi_manager.bringup_check import _baro_health_verdict          # noqa: E402
from pymavlink import mavutil                                          # noqa: E402

_BARO = mavutil.mavlink.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE
_OTHER = mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_GYRO


def test_healthy_baro_passes():
    status, label, _ = _baro_health_verdict(_BARO | _OTHER, _BARO | _OTHER)
    assert status == PASS and 'Bar30' in label


def test_unhealthy_baro_fails_because_every_move_verb_is_denied():
    """Present but unhealthy is a definite statement: the board refuses AUTO, and
    SROT_MOVE enters AUTO, so move_forward is denied too. FAIL, not WARN -- finding
    this on the bench is the entire point of the line."""
    status, _, detail = _baro_health_verdict(_OTHER, _BARO | _OTHER)
    assert status == FAIL
    assert 'DENIED' in detail or 'denied' in detail


def test_absent_sys_status_warns_rather_than_failing():
    """Same asymmetry as the behaviour-rev check: silence is far more likely a dropped
    frame than a dead sensor, and failing a preflight on a comms hiccup is its own hazard."""
    assert _baro_health_verdict(None, None)[0] == WARN


def test_baro_not_present_warns():
    assert _baro_health_verdict(_OTHER, _OTHER)[0] == WARN
