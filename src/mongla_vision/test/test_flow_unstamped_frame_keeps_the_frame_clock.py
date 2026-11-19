"""An unstamped frame is timed on the clock every other frame uses.

Camera frames carry the capture instant on the WALL clock (`camera_node.
_make_header`, from `meta.stamp_wall`), and `/mongla/imu_rates` is stamped on
the same clock. `_take` gave a frame with a zero stamp `time.monotonic()`
instead -- seconds since boot against seconds since 1970 -- so its interval
and the gyro looked up for it were in a different clock from every other
frame's.
"""
import threading
import time
import types

from mongla_vision.flow.flow_setup import FlowSetupMixin


def test_a_zero_stamp_falls_back_to_the_wall_clock():
    s = types.SimpleNamespace(_slot_lock=threading.Lock(), _slot=None,
                              _dropped=0, _seq=0, _slot_evt=threading.Event())
    header = types.SimpleNamespace(stamp=types.SimpleNamespace(sec=0, nanosec=0))
    before = time.time()
    FlowSetupMixin._take(s, object(), header)
    assert before - 1.0 <= s._slot[1] <= time.time() + 1.0
