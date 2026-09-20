"""The uplink's capture instant on the board clock: right when the clock map
is good, and absent -- never guessed -- when it is not."""
import time
from types import SimpleNamespace

from mongla_manager.auv_manager_node import AUVManagerNode


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
    def __init__(self):
        self.calls = []
        self.pos = []
    def send_speed_estimate(self, *a):
        self.calls.append(a)
        return True
    def send_position_estimate(self, *a):
        self.pos.append(a)
        return True


class _Param:
    def __init__(self, v): self.value = v


def _odom(stamp_s, vx=0.2, vy=-0.1, vx_var=1e-3, vy_var=2e-3,
          px=1.0, py=2.0, pz=0.8, frame='odom', yaw=0.0):
    import math
    cov = [0.0] * 36
    cov[0], cov[7] = vx_var, vy_var
    pcov = [0.0] * 36
    for i, v in enumerate((0.04, 0.05, 0.0004, 0.001, 0.001, 0.003)):
        pcov[i * 6 + i] = v
    pcov[1] = pcov[6] = 0.01
    return SimpleNamespace(
        header=SimpleNamespace(frame_id=frame, stamp=SimpleNamespace(
            sec=int(stamp_s), nanosec=int((stamp_s % 1) * 1e9))),
        pose=SimpleNamespace(pose=SimpleNamespace(
            position=SimpleNamespace(x=px, y=py, z=pz),
            orientation=SimpleNamespace(w=math.cos(yaw / 2), x=0.0, y=0.0,
                                        z=math.sin(yaw / 2))),
            covariance=pcov),
        twist=SimpleNamespace(twist=SimpleNamespace(
            linear=SimpleNamespace(x=vx, y=vy)), covariance=cov))


def _unode(enabled=True, ok=True, position=False):
    n = _node(ok)
    n.fc = _FC()
    n._vel_uplink_n = 0
    params = {'velocity_uplink': enabled, 'position_uplink': position}
    n.get_parameter = lambda name: _Param(params[name])
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


# --------------------------------------------------------------------------- #
#  The position uplink handler
# --------------------------------------------------------------------------- #
def test_position_uplink_sends_pose_yaw_and_board_time():
    import math
    n = _unode(enabled=False, position=True)
    n._on_odom_uplink(_odom(1.7e9 + 10.0, yaw=math.radians(30)))
    (x, y, z, yaw, cov6, board_s, reset), = n.fc.pos
    assert (x, y, z) == (1.0, 2.0, 0.8)
    assert abs(math.degrees(yaw) - 30.0) < 1e-6
    assert abs(board_s - 10.0) < 1e-3 and reset == 0
    assert cov6[0][0] == 0.04 and cov6[0][1] == 0.01 and cov6[5][5] == 0.003
    assert n.fc.calls == [], 'velocity is independent of the position switch'


def test_a_frame_change_or_a_jump_bumps_the_reset_counter_and_motion_does_not():
    n = _unode(enabled=False, position=True)
    n._on_odom_uplink(_odom(1.7e9 + 10.0, px=1.0))
    n._on_odom_uplink(_odom(1.7e9 + 10.1, px=1.05))           # motion
    n._on_odom_uplink(_odom(1.7e9 + 10.2, px=1.05, frame='pool'))  # anchor
    n._on_odom_uplink(_odom(1.7e9 + 10.3, px=3.0, frame='pool'))   # prop fix
    assert [c[-1] for c in n.fc.pos] == [0, 0, 1, 2]


def test_position_uplink_off_or_unmapped_sends_nothing():
    for n in (_unode(enabled=False, position=False), _unode(ok=False, position=True)):
        n._on_odom_uplink(_odom(1.7e9 + 1.0))
        assert n.fc.pos == []
