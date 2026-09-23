"""`motor_test` -- the verb that turns ONE thruster.

⛔ THE SAFETY SHAPE IS THE POINT. The board expires the test when the
keep-alives stop, and expiry AUTO-DISARMS. There is no stop command and none is
wanted: silence IS the stop. So the things worth testing are that the verb
cannot hold a motor spinning past its deadline, that it refuses a backend that
has no such command rather than doing nothing quietly, and that it passes the
abort callable through -- not that it produces a particular number.
"""
import pytest

from mongla_control.commands import COMMANDS


class _FakeArduSub:
    """Deliberately has NO motor_test -- the ArduSub path never had a driver."""
    name = 'pixhawk'

    def is_armed(self):
        return True

    def get_attitude(self):
        return None        # `_make_result` falls back to depth 0.0


class _FakeSrot(_FakeArduSub):
    name = 'srot'

    def __init__(self, ok=True, reason='tested'):
        self.ok, self.reason, self.calls = ok, reason, []

    def motor_test(self, motor, pct, seconds, *, abort_fn=None):
        self.calls.append((motor, pct, seconds, abort_fn))
        return self.ok, self.reason


def _mongla(fc):
    """A Mongla with no ROS and no link -- only what `_command_scope` touches."""
    import logging
    import threading

    from mongla_control.mongla import Mongla
    m = Mongla.__new__(Mongla)
    m.pixhawk = fc
    m.lock = threading.RLock()
    m._abort_event = threading.Event()
    m._heartbeat = None
    m._heading_lock = None
    m._lock_deferred = False
    m._lock_holds_heartbeat = False
    m.log = logging.getLogger('test')
    return m


# --------------------------------------------------------------------------- #
#  The registry
# --------------------------------------------------------------------------- #

def test_the_verb_is_registered():
    assert 'motor_test' in COMMANDS


def test_it_takes_the_motor_index_on_the_overloaded_target_field():
    """No wire change: `target` already carries metres for set_depth and degrees
    for yaw_*. A dedicated field would need a mongla_interfaces rebuild."""
    spec = COMMANDS['motor_test']
    assert spec['fields'] == ['target', 'gain', 'duration']
    assert spec['defaults']['target'] == 1.0


def test_the_help_warns_that_it_turns_a_thruster():
    """A verb that spins a propeller must say so where an operator reads it --
    `mongla --help` is the last thing between a hand and a blade."""
    help_text = COMMANDS['motor_test']['help'].upper()
    assert 'PROPS OFF' in help_text or 'RESTRAINED' in help_text
    assert 'DISARM' in help_text        # the expiry behaviour is stated


def test_the_default_throttle_is_gentle():
    """20 %, not 80 % like the move_* verbs. The first thing anybody runs on a
    new thruster should not be most of full scale."""
    assert COMMANDS['motor_test']['defaults']['gain'] == 20.0
    assert COMMANDS['motor_test']['defaults']['gain'] < COMMANDS['move_forward']['defaults']['gain']


# --------------------------------------------------------------------------- #
#  Dispatch
# --------------------------------------------------------------------------- #

def test_it_drives_the_backend_with_the_index_throttle_and_duration():
    fc = _FakeSrot()
    res = _mongla(fc).motor_test(target=3.0, gain=25.0, duration=1.5)
    assert res.success
    motor, pct, seconds, abort_fn = fc.calls[0]
    assert (motor, pct, seconds) == (3, 25.0, 1.5)
    assert callable(abort_fn), 'the abort callable must reach the keep-alive loop'


def test_a_negative_throttle_reaches_the_board_unchanged():
    """Reverse is not a special case to be clamped away -- running a thruster
    backwards is how REVERSE_EFFICIENCY gets measured instead of quoted."""
    fc = _FakeSrot()
    _mongla(fc).motor_test(target=2.0, gain=-30.0, duration=0.5)
    assert fc.calls[0][1] == -30.0


def test_a_backend_without_the_command_is_refused_loudly():
    """⛔ NOT a silent no-op. A verb that reports success while nothing turns is
    the failure mode that ends competition runs."""
    res = _mongla(_FakeArduSub()).motor_test(target=1.0)
    assert res.success is False
    assert 'motor_test' in res.message


def test_a_refusal_from_the_board_is_reported_as_a_failure():
    fc = _FakeSrot(ok=False, reason='motor test requires ARMED')
    res = _mongla(fc).motor_test(target=1.0)
    assert res.success is False
    assert 'ARMED' in res.message


@pytest.mark.parametrize('motor', [1.0, 4.0, 8.0])
def test_the_index_is_reported_back_so_a_log_says_which_motor(motor):
    res = _mongla(_FakeSrot()).motor_test(target=motor)
    assert res.final_value == motor


def test_the_verb_is_gated_on_ARMED_by_the_command_scope():
    """⛔ NOT a courtesy check. The board refuses a motor test when disarmed, and
    `_command_scope` refuses it here first, so a disarmed caller learns without
    a round trip -- and, more importantly, `motor_test` is deliberately NOT in
    `_UNARM_SAFE`. A verb that spins a propeller must never be unarm-safe."""
    from mongla_control.mongla import _UNARM_SAFE, NotArmedError

    assert 'motor_test' not in _UNARM_SAFE

    fc = _FakeSrot()
    fc.is_armed = lambda: False
    with pytest.raises(NotArmedError):
        _mongla(fc).motor_test(target=1.0)
    assert fc.calls == [], 'nothing may reach the board from a disarmed hull'
