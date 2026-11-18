#!/usr/bin/env python3
"""What retrodiction costs the localiser, on the machine that runs it.

`retrodict:=true` applies each flow sample at the instant it describes and
replays the events after it. Its cost grows with how LATE a sample is and how
often samples come, so it is measured at the vehicle's rates rather than
argued: the board's IMU at 50 Hz (`/mongla/imu`, measured-bars "50.001 Hz"),
and flow while moving -- one interval per ~8 px of floor, which at 0.5 m/s and
1 m over the floor with f ~741 px is ~46 a second -- each arriving `--late-ms`
after its exposure.

It drives the SHIPPED node: a real `LocalizationNode`, constructed with the
parameter, fed real messages through its own callbacks, with no executor in
between. CPU is `time.process_time` over the whole run, so it counts every
replayed IMU step and attitude update, and the per-callback p99 says whether a
replay could hold the IMU callback past its 20 ms period.

    python3 tools/bench/retro_cost.py               # both arms, 60 s each
    python3 tools/bench/retro_cost.py --late-ms 120 --flow-hz 46
"""
import argparse
import time

import numpy as np


def _arm(retro: bool, seconds: float, imu_hz: float, flow_hz: float,
         late_s: float):
    import rclpy
    from rclpy.parameter import Parameter
    from geometry_msgs.msg import TwistWithCovarianceStamped
    from sensor_msgs.msg import Imu
    from mongla_localization.localization_node import LocalizationNode

    node = LocalizationNode.__new__(LocalizationNode)
    import rclpy.node
    orig = rclpy.node.Node.__init__

    def patched(self, name, **kw):
        kw['parameter_overrides'] = list(kw.get('parameter_overrides') or []) \
            + [Parameter('retrodict', value=retro)]
        orig(self, name, **kw)
    rclpy.node.Node.__init__ = patched
    try:
        node.__init__()
    finally:
        rclpy.node.Node.__init__ = orig

    def stamp(msg, t):
        msg.header.stamp.sec = int(t)
        msg.header.stamp.nanosec = int(round((t - int(t)) * 1e9))
        return msg

    t0, dt = 1000.0, 1.0 / imu_hz
    flow_dt = 1.0 / flow_hz
    pending = []                      # (arrival_t, measured_t)
    next_flow = t0 + flow_dt
    per_cb = []
    cpu0 = time.process_time()
    k = 0
    while k * dt < seconds:
        t = t0 + k * dt
        m = Imu()
        m.linear_acceleration.x = 0.2 * np.sin(0.5 * t)   # a gentle surge
        m.linear_acceleration.z = -9.80665
        m.angular_velocity.z = 0.05 * np.cos(0.3 * t)
        m.orientation.w = 1.0
        m.orientation_covariance[0] = 7.6e-5
        a = time.perf_counter()
        node._on_imu(stamp(m, t))
        per_cb.append(time.perf_counter() - a)
        while next_flow <= t:
            pending.append((next_flow + late_s, next_flow))
            next_flow += flow_dt
        while pending and pending[0][0] <= t:
            _arr, meas = pending.pop(0)
            f = TwistWithCovarianceStamped()
            f.twist.twist.linear.x = 0.4
            f.twist.covariance[0] = f.twist.covariance[7] = 1e-3
            a = time.perf_counter()
            node._on_flow(stamp(f, meas))
            per_cb.append(time.perf_counter() - a)
        k += 1
    cpu = time.process_time() - cpu0
    r = getattr(node, '_retro', None)
    out = dict(cpu_frac=cpu / seconds,
               p50_ms=float(np.percentile(per_cb, 50) * 1e3),
               p99_ms=float(np.percentile(per_cb, 99) * 1e3),
               max_ms=float(np.max(per_cb) * 1e3),
               flow_used=node._n.get('flow', 0),
               late=getattr(r, 'late', 0), replayed=getattr(r, 'replayed', 0),
               refused=getattr(r, 'refused', 0))
    node.destroy_node()
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--seconds', type=float, default=60.0)
    ap.add_argument('--imu-hz', type=float, default=50.0)
    ap.add_argument('--flow-hz', type=float, default=46.0)
    ap.add_argument('--late-ms', type=float, default=60.0)
    a = ap.parse_args()
    import rclpy
    rclpy.init()
    try:
        for retro in (False, True):
            r = _arm(retro, a.seconds, a.imu_hz, a.flow_hz, a.late_ms / 1e3)
            print(f'retrodict={str(retro).lower():5}  '
                  f'cpu {100 * r["cpu_frac"]:5.2f} % of a core   '
                  f'callback p50 {r["p50_ms"]:.3f} ms  p99 {r["p99_ms"]:.3f} ms  '
                  f'max {r["max_ms"]:.2f} ms   flow used {r["flow_used"]}  '
                  f'late {r["late"]}  replayed {r["replayed"]}  refused {r["refused"]}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
