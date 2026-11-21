#!/usr/bin/env python3
"""A stand-in for the localization filter on a bench: anchored, and moved by the demand.

Batighor's map draws the vehicle only once the filter is anchored in the
pool, and the anchor needs a prop seen in water. On a bench with the fake
board, this takes the filter's place (start bringup with `localization:=false`)
so the map, the Mission view and a real `go_to` can be watched end to end:

    ros2 launch mongla_manager bringup.launch.py localization:=false batighor:=true \\
        mav_device:=udpin:127.0.0.1:14555 course:=<course>
    python3 tools/bench/emulate_localization.py --x 0 --y 0 --yaw 0
    ros2 run mongla_planner mongla go_to --x_m 8 --y_m 2 --tolerance_m 0.5

It publishes what the filter publishes (`/mongla/localization/state`,
`/mongla/odom`, the `odom_ned -> mongla` transform), and moves by integrating
the manager's own `/mongla/demand` (surge and sway, -1 to 1) at `--speed`
metres a second at full demand, along the board's heading from `/mongla/state`.
That is kinematics, not a vehicle: no drag, no inertia, no drift. Every state
it publishes says `aided (emulated)`, so no screen can take it for the filter.

Beside a running filter it refuses, at start and whenever one appears: two
publishers on the filter's topics drew the vehicle anchored at the origin and
13 m away by turns, five seconds apart (2026-11-21).

    python3 tools/bench/emulate_localization.py --self-test
"""
from __future__ import annotations

import argparse
import math
import sys
import time

AIDING = 'aided (emulated)'
TOPICS = ('/mongla/localization/state', '/mongla/odom')


def rivals(node) -> list:
    """`node on topic` for every other node publishing the filter's topics."""
    me = node.get_name()
    return [f'{i.node_name if i.node_name != UNKNOWN else "a node not yet named"} on {t}'
            for t in TOPICS for i in node.get_publishers_info_by_topic(t)
            if i.node_name != me]


# What the graph reports for an endpoint whose node's name has not been
# discovered yet: on CI the publisher was seen before its name.
UNKNOWN = '_NODE_NAME_UNKNOWN_'


def step(x, y, yaw_deg, surge, sway, speed, dt):
    """The next (x, y): the body demand rotated into the pool frame by the heading. Pure."""
    s = surge if math.isfinite(surge) else 0.0
    w = sway if math.isfinite(sway) else 0.0
    psi = math.radians(yaw_deg if math.isfinite(yaw_deg) else 0.0)
    vx = speed * (math.cos(psi) * s - math.sin(psi) * w)
    vy = speed * (math.sin(psi) * s + math.cos(psi) * w)
    return x + vx * dt, y + vy * dt


def self_test() -> int:
    """go_to's own demand, through this step, must arrive: the loop closes with the right signs."""
    sys.path.insert(0, str(__import__('pathlib').Path(__file__).resolve().parents[2]
                           / 'src' / 'mongla_control'))
    from mongla_control.localization_verbs import body_demand
    ok = True
    for yaw in (0.0, 90.0, -135.0):
        x, y = 0.0, 0.0
        for _ in range(4000):
            fwd, stb = body_demand(8.0 - x, 2.0 - y, yaw, 30.0)
            x, y = step(x, y, yaw, fwd / 100.0, stb / 100.0, 0.5, 0.05)
        err = math.hypot(8.0 - x, 2.0 - y)
        print(f'yaw {yaw:+6.1f}: ends {err:.3f} m from (8, 2)')
        ok &= err < 0.05
    # The injected defect: a sway sign flipped must miss.
    x, y = 0.0, 0.0
    for _ in range(4000):
        fwd, stb = body_demand(8.0 - x, 2.0 - y, 90.0, 30.0)
        x, y = step(x, y, 90.0, fwd / 100.0, -stb / 100.0, 0.5, 0.05)
    flipped = math.hypot(8.0 - x, 2.0 - y)
    print(f'sway sign flipped: ends {flipped:.2f} m away (must miss)')
    ok &= flipped > 1.0
    print('PASS' if ok else 'FAIL')
    return 0 if ok else 1


def run(a) -> int:
    import rclpy
    from geometry_msgs.msg import TransformStamped, Vector3Stamped
    from mongla_interfaces.msg import LocalizationState, MonglaState
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from tf2_ros import TransformBroadcaster

    rclpy.init()
    node = Node('emulated_localization')
    end = time.monotonic() + 2.0              # let discovery see a filter already up
    while time.monotonic() < end and not rivals(node):
        rclpy.spin_once(node, timeout_sec=0.1)
    # A rival is found; give discovery a second to name it.
    end = time.monotonic() + 1.0
    while time.monotonic() < end and any('not yet named' in r for r in rivals(node)):
        rclpy.spin_once(node, timeout_sec=0.1)
    if rivals(node):
        print('ERROR: the filter is publishing (' + ', '.join(rivals(node)) + '); start '
              'bringup with localization:=false, or stop it, then run this again',
              file=sys.stderr)
        return 1
    pub_state = node.create_publisher(LocalizationState, '/mongla/localization/state', 10)
    pub_odom = node.create_publisher(Odometry, '/mongla/odom', 10)
    tf = TransformBroadcaster(node)
    st = {'x': a.x, 'y': a.y, 'yaw': a.yaw, 'surge': 0.0, 'sway': 0.0, 'depth': math.nan,
          't': time.monotonic()}

    def on_demand(m):
        st['surge'], st['sway'] = m.vector.x, m.vector.y

    def on_state(m):
        if math.isfinite(m.yaw_deg):
            st['yaw'] = m.yaw_deg
        st['depth'] = m.depth_m

    node.create_subscription(Vector3Stamped, '/mongla/demand', on_demand, 10)
    node.create_subscription(MonglaState, '/mongla/state', on_state, 10)

    def tick():
        if st.get('n', 0) % int(a.rate) == 0 and rivals(node):
            print('ERROR: a filter started publishing (' + ', '.join(rivals(node))
                  + '); stopping', file=sys.stderr, flush=True)
            raise SystemExit(1)
        st['n'] = st.get('n', 0) + 1
        now = time.monotonic()
        st['x'], st['y'] = step(st['x'], st['y'], st['yaw'], st['surge'], st['sway'],
                                a.speed, now - st['t'])
        st['t'] = now
        stamp = node.get_clock().now().to_msg()
        s = LocalizationState()
        s.header.stamp, s.header.frame_id = stamp, 'pool'
        s.frame, s.anchored, s.usable = 'pool', True, True
        s.x_m, s.y_m, s.depth_m, s.yaw_deg = st['x'], st['y'], st['depth'], st['yaw']
        s.sigma_xy_m, s.sigma_yaw_deg = a.sigma, 1.0
        s.aiding, s.motion = AIDING, 'ok'
        s.imu_age_s = s.depth_age_s = s.flow_age_s = 0.0
        pub_state.publish(s)
        q = (0.0, 0.0, math.sin(math.radians(st['yaw']) / 2), math.cos(math.radians(st['yaw']) / 2))
        o = Odometry()
        o.header.stamp, o.header.frame_id, o.child_frame_id = stamp, 'odom_ned', 'mongla'
        o.pose.pose.position.x, o.pose.pose.position.y = st['x'], st['y']
        o.pose.pose.position.z = -st['depth'] if math.isfinite(st['depth']) else 0.0
        (o.pose.pose.orientation.x, o.pose.pose.orientation.y,
         o.pose.pose.orientation.z, o.pose.pose.orientation.w) = q
        pub_odom.publish(o)
        t = TransformStamped()
        t.header.stamp, t.header.frame_id, t.child_frame_id = stamp, 'odom_ned', 'mongla'
        t.transform.translation.x, t.transform.translation.y = st['x'], st['y']
        t.transform.translation.z = o.pose.pose.position.z
        (t.transform.rotation.x, t.transform.rotation.y,
         t.transform.rotation.z, t.transform.rotation.w) = q
        tf.sendTransform(t)

    node.create_timer(1.0 / a.rate, tick)
    print(f'emulating an anchored filter at ({a.x}, {a.y}), {a.speed} m/s at full demand; '
          f'every state says "{AIDING}"', flush=True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except SystemExit as exc:
        return int(exc.code or 1)
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--self-test', action='store_true')
    ap.add_argument('--x', type=float, default=0.0, help='start, pool frame metres')
    ap.add_argument('--y', type=float, default=0.0)
    ap.add_argument('--yaw', type=float, default=0.0, help='until /mongla/state gives one')
    ap.add_argument('--speed', type=float, default=0.5, help='m/s at full demand')
    ap.add_argument('--sigma', type=float, default=0.2, help='the horizontal sigma it claims, m')
    ap.add_argument('--rate', type=float, default=20.0, help='Hz')
    a = ap.parse_args(argv)
    return self_test() if a.self_test else run(a)


if __name__ == '__main__':
    sys.exit(main())
