"""The chain, photons to thrust: every stage's rate and age, measured where it is seen.

One row per stage in `STAGES`; a new stage on the vehicle is one row here and
nothing else. For each stage the node counts messages over a window (the rate)
and, where the message carries a stamp, takes `seen - stamp` (the age): how
old that stage's data is by the time anyone downstream can use it.

Every stamp here is on the host wall clock, which is what makes the ages
comparable down the chain: camera frames are stamped at capture, detections,
tracks and the lock carry the stamp of the frame they describe, the demand is
stamped when it is sent, and the board's IMU is stamped on the board's clock
mapped onto the host (`ClockMap`). That last map is fitted from arrival pairs,
so the IMU's age is its latency ABOVE the fitted offset, not an absolute
one-way delay; the row says so.

A stage that sends no stamp has a rate and no age, and a stage the board does
not send at all is listed with the ask that would produce it. Absence is
`None` in the JSON and `--` on the panel, never 0.

    ros2 run mongla_batighor chain       # publishes /mongla/batighor/chain

It measures only while someone watches: the stage subscriptions exist while
`/mongla/batighor/chain` has a subscriber (`overlay.Watched`). Subscribed for
good, it cost 17 % of a Pi core with nobody looking (2026-11-21).
"""
from __future__ import annotations

import bisect
import collections
import json
import time
from dataclasses import dataclass

from mongla_batighor.seat import CAMERAS

TOPIC = '/mongla/batighor/chain'
PERIOD_S = 0.5
WINDOW_S = 5.0
_KEEP = 4096                     # samples per stage: 5 s at 500 Hz is 2500


@dataclass(frozen=True)
class Stage:
    key: str
    label: str
    topic: str
    msg: str                     # 'pkg/Type'
    qos: str                     # a name in mongla_vision.qos, or 'default'
    stamped: bool
    clock: str                   # where its stamp comes from, or why it has none
    ask: str = ''                # for a stage with no stamp: what would give it one


def stages(cameras=CAMERAS) -> list:
    """Every measured stage, in the order the chain runs."""
    v = '/mongla/vision'
    rows = []
    for c in cameras:
        rows += [
            Stage(f'frame:{c}', f'{c} frame', f'{v}/{c}/camera_info',
                  'sensor_msgs/CameraInfo', 'CAMERA_INFO', True,
                  'capture, host wall clock'),
            Stage(f'detect:{c}', f'{c} detect', f'{v}/{c}/detections',
                  'vision_msgs/Detection2DArray', 'DETECTIONS', True,
                  'the frame it describes'),
            Stage(f'track:{c}', f'{c} track', f'{v}/{c}/tracks',
                  'vision_msgs/Detection2DArray', 'DETECTIONS', True,
                  'the frame it describes'),
            Stage(f'lock:{c}', f'{c} lock', f'{v}/{c}/lock',
                  'vision_msgs/Detection2DArray', 'DETECTIONS', True,
                  'the frame the winning rung saw'),
        ]
    rows += [
        Stage('demand', 'demand', '/mongla/demand', 'geometry_msgs/Vector3Stamped',
              'default', True, 'when the manager sent it',
              ask='ours: the demand does not carry the frame it acted on, so '
                  'frame-to-command is not measured'),
        Stage('imu', 'board IMU', '/mongla/imu', 'sensor_msgs/Imu', 'default', True,
              'board clock mapped to host (latency above the fitted offset)'),
        Stage('rpm', 'ESC rpm', '/mongla/esc_rpm', 'std_msgs/Int32MultiArray',
              'default', False, 'none: no header',
              ask='upstream F: per-thruster presence and a board time with the rpm'),
    ]
    return rows


# What the board does not send at all; shown so the gap is seen, not guessed.
NOT_SENT = (
    ('board loop', 'upstream B: loop_dt_us, the 500 Hz loop\'s own jitter'),
    ('mixer', 'upstream B: the mixer\'s per-group saturation'),
    ('camera trigger', 'upstream W: the board triggers the cameras on its clock'),
)


class ChainMeter:
    """Rates and ages per stage over a sliding window. Pure: no ROS, no clock."""

    def __init__(self, window_s: float = WINDOW_S):
        self.window_s = float(window_s)
        self._seen = collections.defaultdict(lambda: collections.deque(maxlen=_KEEP))

    def note(self, key: str, stamp_s, seen_s: float) -> None:
        """One message of `key`, stamped `stamp_s` (None: no stamp), seen at `seen_s`."""
        age = None
        if stamp_s is not None and stamp_s > 0.0:
            age = (seen_s - stamp_s) * 1000.0
        self._seen[key].append((seen_s, age))

    def summary(self, now_s: float) -> dict:
        out = {}
        for key, q in self._seen.items():
            times = [t for t, _ in q]
            i = bisect.bisect_left(times, now_s - self.window_s)
            recent = list(q)[i:]
            ages = sorted(a for _, a in recent if a is not None)
            n = len(recent)
            span = (recent[-1][0] - recent[0][0]) if n > 1 else 0.0
            out[key] = {
                'n': n,
                'hz': ((n - 1) / span) if span > 0.0 else 0.0,
                'age_ms_p50': _pct(ages, 50),
                'age_ms_p95': _pct(ages, 95),
                'last_s': (now_s - times[-1]) if times else None,
            }
        return out


def _pct(xs, p):
    if not xs:
        return None
    k = min(len(xs) - 1, max(0, round(p / 100.0 * (len(xs) - 1))))
    return xs[k]


def _stamp_s(msg):
    h = getattr(msg, 'header', None)
    if h is None:
        return None
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def _msg_class(path: str):
    import importlib
    pkg, name = path.split('/')
    return getattr(importlib.import_module(f'{pkg}.msg'), name)


def _qos(name: str):
    if name == 'default':
        return 10
    from mongla_vision import qos
    return getattr(qos, name)


def ChainNode(cameras=CAMERAS, period_s: float = PERIOD_S):
    """The node: one subscription per stage, the summary on `TOPIC`."""
    from rclpy.node import Node
    from std_msgs.msg import String

    class _Chain(Node):
        def __init__(self):
            super().__init__('mongla_chain')
            from mongla_batighor.overlay import Watched
            self.meter = ChainMeter()
            self.rows = stages(cameras)
            self.pub = self.create_publisher(String, TOPIC, 10)
            self.watched = Watched(self, self.pub, [
                (_msg_class(st.msg), st.topic,
                 lambda m, k=st.key, s=st.stamped: self.meter.note(
                     k, _stamp_s(m) if s else None, time.time()),
                 _qos(st.qos))
                for st in self.rows], on_drop=self._forget)
            self.create_timer(period_s, self._publish)

        def _forget(self):
            # The next viewer must not see rates from before it looked.
            self.meter = ChainMeter(self.meter.window_s)

        def _publish(self):
            self.watched.poll()
            if not self.watched.subs:
                return
            got = self.meter.summary(time.time())
            empty = {'n': 0, 'hz': 0.0, 'age_ms_p50': None, 'age_ms_p95': None,
                     'last_s': None}
            body = {
                'window_s': self.meter.window_s,
                'order': [s.key for s in self.rows],
                'stages': {s.key: {**got.get(s.key, empty), 'label': s.label,
                                   'clock': s.clock, 'ask': s.ask}
                           for s in self.rows},
                'not_sent': [{'label': a, 'ask': b} for a, b in NOT_SENT],
            }
            self.pub.publish(String(data=json.dumps(body)))

    return _Chain()


def main(args=None):
    import rclpy
    from mongla_runtime.params_file import refuse_unknown
    from mongla_runtime.spin import spin
    rclpy.init(args=args)
    node = ChainNode()
    refuse_unknown(node)
    try:
        spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
