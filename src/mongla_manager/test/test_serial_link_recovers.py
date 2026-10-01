"""A pulled cable is recovered from, not retried forever (issue #12).

Measured on the bench board, 2026-10-01, with the manager running: USB pulled
~5 s and replugged. The reader saw the dead port, the 2 Hz heartbeat writer
reopened it in place (pymavlink `autoreconnect`), the board's reboot was
detected and its stream rates and JS_GAIN_DEFAULT re-applied. /mongla/state
yaw read NaN -- never a frozen value -- for 7.7 s (88.0-94.5 s, first good
sample 95.7 s), and the board came back DISARMED.
"""
import serial
import pytest

from mongla_manager.auv_manager_node import _is_dead_port, _validate_fc_kind


class _Master:
    def __init__(self, dead=False):
        self.portdead = dead


def test_the_first_failed_READ_is_recognised_as_a_dead_port():
    """pymavlink sets portdead only on a failed write; the read comes first."""
    exc = serial.SerialException('device reports readiness to read but '
                                 'returned no data (device disconnected ...)')
    assert _is_dead_port(exc, _Master(dead=False))


def test_portdead_alone_is_enough():
    assert _is_dead_port(RuntimeError('anything'), _Master(dead=True))


def test_a_reader_bug_is_not_called_a_dead_cable():
    assert not _is_dead_port(KeyError('x'), _Master(dead=False))


@pytest.mark.parametrize('kind', ['srot', 'pixhawk'])
def test_a_known_backend_passes(kind):
    _validate_fc_kind(kind)


def test_a_typo_is_refused_before_any_port_is_opened():
    """It used to resolve the ArduSub UDP profile and hang in wait_heartbeat()."""
    with pytest.raises(ValueError, match="'srto'"):
        _validate_fc_kind('srto')
