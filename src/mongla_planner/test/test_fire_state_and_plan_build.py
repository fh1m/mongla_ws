"""FireState outcomes, and that the plans containing one still BUILD.

WHY THE BUILD TEST EXISTS. YASMIN validates a state's transition map against its
declared outcomes at `add_state()` time -- which is mission-construction time, i.e.
on the pool deck. When `FireState` gained a `FAILED` outcome, every plan that adds
one needed a `FAILED:` transition or construction would raise. Nothing in the suite
instantiated those plans, so a full green run proved nothing about it.

WHY THE NO_ACK CASE IS SPELLED OUT. `FireResult.ok` is deliberately False for
NO_ACK -- "we do not know whether it fired" must never read as "it fired" to
anything holding only a boolean. But at the mission layer the right call is the
opposite: the command almost certainly went out and only the acknowledgement was
lost, which over the BlueOS bridge is a measured ~8-9% of frames. Failing a
competition task over link jitter loses points for nothing. So FireState branches
on the outcome CODE, not on `success`, and these tests pin that the two layers
disagree deliberately rather than by accident.
"""

import pytest

from mongla_control.fc.base import (FIRE_FIRED, FIRE_REJECTED_ARM, FIRE_DISABLED,
                                    FIRE_DENIED, FIRE_NO_ACK, FIRE_NOT_READY,
                                    FIRE_BUSY)
from mongla_planner.state_machines.core.outcomes import SUCCEED, FAILED
from mongla_planner.state_machines.states.utility import FireState


class _Result:
    """Stands in for the Move.Result a `mongla.fire()` goal returns."""
    def __init__(self, code):
        self.final_value = float(code)
        self.success = (code == FIRE_FIRED)
        self.message = ''


class _Mongla:
    camera = 'forward'

    def __init__(self, code):
        self._code = code
        self.fired = []

    def fire(self, channel):
        self.fired.append(channel)
        return _Result(self._code)

    def pause(self, *_a, **_k):
        pass


def _outcome(code, channel=9):
    d = _Mongla(code)
    state = FireState(d, profile=None, channel=channel, confirm_pause_s=0.0)
    return state._run(bb=None), d


def test_a_fired_shot_succeeds():
    out, d = _outcome(FIRE_FIRED)
    assert out == SUCCEED and d.fired == [9]


@pytest.mark.parametrize('code', [FIRE_REJECTED_ARM, FIRE_DISABLED, FIRE_DENIED,
                                  FIRE_NOT_READY, FIRE_BUSY])
def test_a_shot_that_did_not_go_out_fails(code):
    """The old code returned SUCCEED unconditionally, so a refused shot was
    indistinguishable from a hit -- including the case where the refusal was
    'that channel is the manipulator arm'."""
    out, _ = _outcome(code)
    assert out == FAILED


def test_no_ack_succeeds_because_it_is_not_evidence_of_a_miss():
    out, _ = _outcome(FIRE_NO_ACK)
    assert out == SUCCEED, (
        'NO_ACK means the ACK was lost, not that the shot was. Failing here '
        'abandons a task over link jitter.')


def test_the_channel_is_passed_through_untouched():
    """No mapping anywhere in this path: FireState(channel=11) fires 11."""
    _, d = _outcome(FIRE_FIRED, channel=11)
    assert d.fired == [11]


# --------------------------------------------------------------------------- #
#  The plans that contain a FireState must still construct                      #
# --------------------------------------------------------------------------- #

class _StubMission:
    camera = 'forward'

    def __getattr__(self, _name):
        return lambda *a, **k: None


def _profile():
    from mongla_planner.state_machines.core.vehicle_profile import VehicleProfile
    try:
        return VehicleProfile.auto('mavlink_ahrs')
    except Exception:                                        # noqa: BLE001
        return VehicleProfile()


@pytest.mark.parametrize('name,params', [
    ('torpedo_fire', {'torpedo_depth_m': -1.2, 'fire_channel': 9}),
    ('bin_drop',     {'fire_channel': 11}),
    ('full_2026',    {'bin_fire_channel': 11, 'torpedo_fire_channel': 9}),
])
def test_plans_with_a_firestate_still_build(name, params):
    from mongla_planner.state_machines.plans.torpedo_fire import build_torpedo_fire_fsm
    from mongla_planner.state_machines.plans.bin_drop import build_bin_drop_fsm
    from mongla_planner.state_machines.plans.full_competition import (
        build_full_competition_fsm)
    builder = {'torpedo_fire': build_torpedo_fire_fsm,
               'bin_drop': build_bin_drop_fsm,
               'full_2026': build_full_competition_fsm}[name]
    # Raises if any FireState outcome lacks a transition -- the failure that would
    # otherwise surface at mission start, in the water.
    assert builder(_StubMission(), _profile(), params) is not None
