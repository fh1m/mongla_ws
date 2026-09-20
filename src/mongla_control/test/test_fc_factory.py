"""FlightController factory tests -- fail-loud dispatch, both backends constructible."""

from types import SimpleNamespace

import pytest

from mongla_control.fc import make_flight_controller, FlightController


class _FakeMaster:
    def __init__(self):
        self.messages = {}
        self.mav = SimpleNamespace()


def test_unknown_backend_raises_valueerror():
    with pytest.raises(ValueError, match='unknown flight_controller'):
        make_flight_controller('warpdrive', master=_FakeMaster())


def test_srot_backend_builds_and_is_a_flight_controller():
    fc = make_flight_controller('srot', master=_FakeMaster())
    assert isinstance(fc, FlightController)
    assert fc.name == 'srot'


def test_pixhawk_backend_builds_and_is_a_flight_controller():
    fc = make_flight_controller('pixhawk', master=_FakeMaster())
    assert isinstance(fc, FlightController)
    assert fc.name == 'pixhawk'
    # PixhawkFC keeps the full Pixhawk surface (is-a Pixhawk) so existing callers
    # are unchanged -- spot-check an inherited method exists.
    assert hasattr(fc, 'send_rc_override') and hasattr(fc, 'set_target_depth')


def test_case_insensitive_name():
    assert make_flight_controller('SROT', master=_FakeMaster()).name == 'srot'
