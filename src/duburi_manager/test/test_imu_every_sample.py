"""`/duburi/imu` publishes every board sample once, each on its own stamp.

A 50 Hz timer reading pymavlink's one-slot cache of a 50 Hz stream aliases.
Measured on the vehicle 2026-09-15: 384 of 2501 messages were repeats and 42.4
of the board's 44.0 Hz got out; queued, 2160 of 2160 unique at 44.0 Hz. `_drain_imu` is driven for real here.
"""
from collections import deque

from duburi_manager.auv_manager_node import AUVManagerNode


class _Msg:
    def __init__(self, **kw):
        self.__dict__.update(kw)


class _Clock:
    ready = True

    def to_host(self, board_s):
        return 1000.0 + board_s


class _Node:
    _drain_imu = AUVManagerNode._drain_imu

    def __init__(self):
        self._imu_clock = _Clock()
        self._imu_q, self._att_q = deque(maxlen=64), deque(maxlen=64)
        self._imu_last_ms = None
        self.out = []

    def _publish_imu(self, stamp_s, imu_msg=None, att_msg=None):
        self.out.append((stamp_s, imu_msg.time_boot_ms,
                         None if att_msg is None else att_msg.time_boot_ms))


def _imu(ms):
    return _Msg(time_boot_ms=ms)


def test_a_burst_of_samples_is_all_published_in_order_on_own_stamps():
    n = _Node()
    for ms in (1000, 1020, 1040):
        n._att_q.append(_imu(ms))
        n._imu_q.append(_imu(ms))
    n._drain_imu()
    assert [o[1] for o in n.out] == [1000, 1020, 1040]
    assert [round(o[0], 3) for o in n.out] == [1001.0, 1001.02, 1001.04]
    assert all(o[1] == o[2] for o in n.out), 'paired with its own ATTITUDE'


def test_a_repeat_is_never_published_twice():
    n = _Node()
    n._imu_q.extend([_imu(1000), _imu(1000), _imu(980)])
    n._drain_imu()
    n._imu_q.append(_imu(1000))
    n._drain_imu()
    assert [o[1] for o in n.out] == [1000]


def test_attitude_arriving_after_its_imu_is_paired_on_the_next_drain_or_previous():
    """Wire order is not a contract. With only an older ATTITUDE known, the
    older one is used -- never a later one."""
    n = _Node()
    n._att_q.extend([_imu(980), _imu(1040)])
    n._imu_q.append(_imu(1000))
    n._drain_imu()
    assert n.out[0][2] == 980


def test_nothing_is_stamped_before_the_board_clock_is_mapped():
    n = _Node()
    n._imu_clock.ready = False
    n._imu_q.append(_imu(1000))
    n._drain_imu()
    assert n.out == [] and not n._imu_q
