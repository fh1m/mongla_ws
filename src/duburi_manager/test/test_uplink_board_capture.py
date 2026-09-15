"""The uplink's capture instant on the board clock: right when the clock map
is good, and absent -- never guessed -- when it is not."""
import time
from types import SimpleNamespace

from duburi_manager.auv_manager_node import AUVManagerNode


class _Clock:
    def to_board(self, host_s):
        return host_s - 1.7e9          # board booted at host 1.7e9 s


def _node(ok=True):
    n = AUVManagerNode.__new__(AUVManagerNode)
    n._imu_clock_ok = ok
    n._imu_clock = _Clock()
    return n


def test_live_box_maps_capture_not_send_time():
    t0 = time.time()
    got = _node()._board_capture_s(SimpleNamespace(age_s=0.040, coasted=False))
    assert got is not None
    assert abs(got - (t0 - 0.040 - 1.7e9)) < 0.01


def test_coasted_or_unmapped_is_none():
    s = SimpleNamespace(age_s=0.04, coasted=False)
    assert _node(ok=False)._board_capture_s(s) is None
    assert _node()._board_capture_s(SimpleNamespace(age_s=0.04, coasted=True)) is None
    assert _node()._board_capture_s(SimpleNamespace(age_s=float('nan'), coasted=False)) is None


# --------------------------------------------------------------------------- #
#  The velocity uplink handler
# --------------------------------------------------------------------------- #
class _FC:
    def __init__(self): self.calls = []
    def send_speed_estimate(self, *a):
        self.calls.append(a)
        return True


class _Param:
    def __init__(self, v): self.value = v


def _odom(stamp_s, vx=0.2, vy=-0.1, vx_var=1e-3, vy_var=2e-3):
    cov = [0.0] * 36
    cov[0], cov[7] = vx_var, vy_var
    return SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(
            sec=int(stamp_s), nanosec=int((stamp_s % 1) * 1e9))),
        twist=SimpleNamespace(twist=SimpleNamespace(
            linear=SimpleNamespace(x=vx, y=vy)), covariance=cov))


def _unode(enabled=True, ok=True):
    n = _node(ok)
    n.fc = _FC()
    n._vel_uplink_n = 0
    n.get_parameter = lambda name: _Param(enabled)
    n.get_logger = lambda: SimpleNamespace(info=lambda m: None, warn=lambda m: None)
    return n


def test_velocity_uplink_maps_the_odom_stamp_to_board_time():
    n = _unode()
    n._on_odom_uplink(_odom(1.7e9 + 42.5))
    (vx, vy, var_x, var_y, board_s), = n.fc.calls
    assert (vx, vy, var_x, var_y) == (0.2, -0.1, 1e-3, 2e-3)
    assert abs(board_s - 42.5) < 1e-3


def test_velocity_uplink_off_or_unmapped_sends_nothing():
    for n in (_unode(enabled=False), _unode(ok=False)):
        n._on_odom_uplink(_odom(1.7e9 + 42.5))
        assert n.fc.calls == []
