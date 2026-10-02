"""`rx_hz` is samples that arrived, not polls that returned a value (#37)."""
import types

from mongla_sensors.sensors_node import SensorsNode
from mongla_sensors.sources.base import YawSource


class _OneHz(YawSource):
    """Caches one sample and returns it on every poll, like the real ones."""
    name = 'one_hz'

    def __init__(self):
        self.n = 0

    def read_yaw(self):
        return 12.0

    def samples_received(self):
        return self.n

    def is_healthy(self):
        return True


def _node(src):
    n = SensorsNode.__new__(SensorsNode)
    n._src = src
    n._reads_total = n._reads_window = 0
    n._last_print = 0.0
    n._samples_at_print = src.samples_received()
    seen = []
    log = types.SimpleNamespace(info=seen.append, warn=seen.append)
    n.get_logger = lambda: log
    return n, seen


def test_a_source_stuck_at_one_hz_does_not_read_as_the_poll_rate(monkeypatch):
    import mongla_sensors.sensors_node as sn
    src = _OneHz()
    n, seen = _node(src)
    clock = [0.0]
    monkeypatch.setattr(sn.time, 'monotonic', lambda: clock[0])
    for k in range(100):                      # 5 s of 20 Hz polling
        clock[0] = k * 0.05
        n._sample()
        if k % 20 == 0:
            src.n += 1                        # one real sample a second
    clock[0] = 5.0
    n._tick()
    hz = float(seen[-1].split('rx_hz=')[1].split()[0])
    assert hz <= 1.5


def test_a_source_that_cannot_count_says_so():
    class _Blind(_OneHz):
        def samples_received(self):
            return None
    n, seen = _node(_Blind())
    n._tick()
    assert 'rx_hz=  nan' in seen[-1]
