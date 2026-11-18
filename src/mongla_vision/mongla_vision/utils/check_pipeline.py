#!/usr/bin/env python3
"""vision_check -- grade the vision graph a mission runs on, before it runs.

With no `--camera` it grades the WHOLE graph at once, because a mission uses
every sensor at once (measured-bars 87): both cameras, both detectors and
their trackers, the lock ladders, optical flow, the board's IMU and state, and
the localiser's odometry. Each row is `ok`, `WARNING` or `FAIL`, and the
command exits 1 on any FAIL, so it can gate a run.

With `--camera NAME` it grades that one camera's frames and detections, for a
bench with one camera or a laptop.

Frames are counted from `camera_info`, never `image_raw`: the camera builds the
raw image only for a subscriber, so watching it would put back the 210 Hz of
691 kB messages the composed graph no longer pays for (measured-bars 89).

A detector that publishes nothing is asked whether it is paused. Paused is the
launch default and a mission resumes it, so that is a WARNING; silent and NOT
paused is a dead detector, a FAIL. Without that question the two look the same.

Examples
--------
ros2 run mongla_vision vision_check                          # the whole graph
ros2 run mongla_vision vision_check --camera forward --require-class gate
ros2 run mongla_vision vision_check --camera laptop --duration 5
"""

from __future__ import annotations

import argparse
import sys
import time
from collections import Counter
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

OK, WARN, FAIL = 'ok', 'WARNING', 'FAIL'

# The graph's bars. Each is the falsifier written before the run that set it.
DET_MIN_HZ = 25.0        # a detector under this fails the graph (measured-bars 87)
IMU_MIN_HZ = 45.0        # the board's IMU runs at 50.001 Hz (measured-bars, "/mongla/imu")
CAMERAS = ('forward', 'downward')


@dataclass
class Observed:
    """What one window saw. Rates in Hz over the window; None = not asked."""
    frames_hz: Dict[str, float] = field(default_factory=dict)
    det_hz: Dict[str, float] = field(default_factory=dict)
    tracks_hz: Dict[str, float] = field(default_factory=dict)
    seeing: Dict[str, Optional[str]] = field(default_factory=dict)
    paused: Dict[str, Optional[bool]] = field(default_factory=dict)
    lock_publishers: Dict[str, int] = field(default_factory=dict)
    classes: Dict[str, Counter] = field(default_factory=dict)
    flow_quality_n: int = 0
    flow_used_n: int = 0
    flow_calibration: Optional[str] = None
    imu_hz: float = 0.0
    state_seen: bool = False
    odom_hz: float = 0.0


def grade_camera(o: Observed, cam: str, *, det_min_hz: float = DET_MIN_HZ
                 ) -> List[Tuple[str, str, str]]:
    """Rows for one camera: (status, check, value)."""
    rows = []
    f = o.frames_hz.get(cam, 0.0)
    rows.append((OK if f > 0 else FAIL, f'{cam} frames',
                 f'{f:.1f} Hz' if f > 0 else 'none -- camera absent or dead'))
    s = o.seeing.get(cam)
    if s is not None:
        rows.append((OK if s == 'ok' else WARN, f'{cam} sight', s))
    d = o.det_hz.get(cam, 0.0)
    paused = o.paused.get(cam)
    if d <= 0.0:
        if paused:
            rows.append((WARN, f'{cam} detections',
                         'paused (a mission resumes it)'))
        elif paused is None:
            rows.append((FAIL, f'{cam} detections',
                         'none, and no detector answers -- not running'))
        else:
            rows.append((FAIL, f'{cam} detections', 'none while NOT paused -- dead'))
    else:
        # A detector cannot outrun its camera. Say so when that is the
        # cause: the Fantech's rate falls with the light (30, 15, 9.5 fps
        # measured), and that is a camera to change, not a detector to debug.
        why = (f'; the camera delivers {f:.1f} fps' if d < det_min_hz
               and 0 < f < det_min_hz else '')
        rows.append((OK if d >= det_min_hz else FAIL, f'{cam} detections',
                     f'{d:.1f} Hz (bar {det_min_hz:.0f}){why}'))
        t = o.tracks_hz.get(cam, 0.0)
        rows.append((OK if t > 0 else FAIL, f'{cam} tracks',
                     f'{t:.1f} Hz' if t > 0 else 'none -- tracker not running'))
    return rows


def grade_graph(o: Observed, *, det_min_hz: float = DET_MIN_HZ,
                imu_min_hz: float = IMU_MIN_HZ) -> List[Tuple[str, str, str]]:
    """Every row the mission graph is graded on."""
    rows = []
    for cam in CAMERAS:
        rows += grade_camera(o, cam, det_min_hz=det_min_hz)
        n = o.lock_publishers.get(cam, 0)
        rows.append((OK if n > 0 else FAIL, f'{cam} lock ladder',
                     'running' if n > 0 else 'not running'))
    if o.flow_quality_n > 0:
        rows.append((OK, 'flow', f'{o.flow_quality_n} intervals judged, '
                                 f'{o.flow_used_n} used'))
    else:
        rows.append((FAIL, 'flow', 'no flow_quality -- flow is not running'))
    rows.append((OK if o.flow_calibration else WARN, 'flow calibration',
                 o.flow_calibration or 'none latched'))
    rows.append((OK if o.imu_hz >= imu_min_hz else FAIL, 'board IMU',
                 f'{o.imu_hz:.1f} Hz (bar {imu_min_hz:.0f})'))
    rows.append((OK if o.state_seen else FAIL, 'board state',
                 'seen' if o.state_seen else 'no /mongla/state -- manager down'))
    rows.append((OK if o.odom_hz > 0 else FAIL, 'odometry',
                 f'{o.odom_hz:.1f} Hz' if o.odom_hz > 0 else 'none -- localiser down'))
    return rows


def _class_of(det) -> str:
    if not det.results:
        return ''
    hyp = det.results[0]
    if hasattr(hyp, 'hypothesis'):
        return str(hyp.hypothesis.class_id)
    return str(getattr(hyp, 'id', ''))


def _paused(node, cam: str, timeout_s: float = 1.0) -> Optional[bool]:
    """The detector's own `paused` parameter, or None if it does not answer."""
    import rclpy
    from rcl_interfaces.srv import GetParameters
    cli = node.create_client(GetParameters,
                             f'/mongla_detector_{cam}/get_parameters')
    try:
        if not cli.wait_for_service(timeout_sec=timeout_s):
            return None
        fut = cli.call_async(GetParameters.Request(names=['paused']))
        rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout_s)
        res = fut.result()
        if res is None or not res.values:
            return None
        return bool(res.values[0].bool_value)
    finally:
        node.destroy_client(cli)


def observe(node, cams, duration: float, graph: bool) -> Observed:
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from mongla_vision import qos
    from sensor_msgs.msg import CameraInfo, Imu
    from std_msgs.msg import String, UInt8
    from vision_msgs.msg import Detection2DArray

    o = Observed()
    n: Counter = Counter()

    def count(key):
        return lambda _m: n.update([key])

    for cam in cams:
        ns = f'/mongla/vision/{cam}'
        o.classes[cam] = Counter()
        node.create_subscription(CameraInfo, f'{ns}/camera_info',
                                 count(('frames', cam)), qos.CAMERA_INFO)

        def on_det(msg, cam=cam):
            n.update([('det', cam)])
            o.classes[cam].update(c for c in map(_class_of, msg.detections) if c)
        node.create_subscription(Detection2DArray, f'{ns}/detections', on_det,
                                 qos.DETECTIONS)
        node.create_subscription(Detection2DArray, f'{ns}/tracks',
                                 count(('tracks', cam)), qos.DETECTIONS)
        node.create_subscription(String, f'{ns}/seeing',
                                 lambda m, cam=cam: o.seeing.__setitem__(cam, m.data),
                                 qos.LATCHED)
    if graph:
        from mongla_interfaces.msg import MonglaState
        from nav_msgs.msg import Odometry
        dns = '/mongla/vision/downward'

        def on_q(m):
            n.update(['flow_q'])
            if m.data > 0:
                n.update(['flow_used'])
        node.create_subscription(UInt8, f'{dns}/flow_quality', on_q, 10)
        node.create_subscription(String, f'{dns}/flow_calibration',
                                 lambda m: setattr(o, 'flow_calibration', m.data),
                                 qos.LATCHED)
        node.create_subscription(Imu, '/mongla/imu', count('imu'),
                                 qos_profile_sensor_data)
        node.create_subscription(MonglaState, '/mongla/state',
                                 lambda _m: setattr(o, 'state_seen', True), 10)
        node.create_subscription(Odometry, '/mongla/odom', count('odom'), 10)

    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)

    for cam in cams:
        o.frames_hz[cam] = n[('frames', cam)] / duration
        o.det_hz[cam] = n[('det', cam)] / duration
        o.tracks_hz[cam] = n[('tracks', cam)] / duration
        if o.det_hz[cam] <= 0.0:
            o.paused[cam] = _paused(node, cam)
        o.lock_publishers[cam] = node.count_publishers(f'/mongla/vision/{cam}/lock')
    o.flow_quality_n, o.flow_used_n = n['flow_q'], n['flow_used']
    o.imu_hz, o.odom_hz = n['imu'] / duration, n['odom'] / duration
    return o


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--camera', default='',
                        help='grade only this camera (forward, downward, laptop); '
                             'default: the whole mission graph')
    parser.add_argument('--duration', type=float, default=5.0,
                        help='measurement window seconds (default: 5)')
    parser.add_argument('--min-det-hz', type=float, default=DET_MIN_HZ,
                        help=f'detections bar, Hz (default: {DET_MIN_HZ:.0f})')
    parser.add_argument('--require-class', default='',
                        help='fail unless this class was seen at least once')
    args = parser.parse_args(argv)

    import rclpy
    from rclpy.node import Node
    rclpy.init(args=None)
    node = Node('vision_check')
    graph = not args.camera
    cams = CAMERAS if graph else (args.camera,)
    print(f'[VCHK ] watching {"the mission graph" if graph else cams[0]} '
          f'for {args.duration:.1f}s ...')
    duration = max(args.duration, 1e-3)
    try:
        o = observe(node, cams, duration, graph)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    rows = (grade_graph(o, det_min_hz=args.min_det_hz) if graph
            else grade_camera(o, cams[0], det_min_hz=args.min_det_hz))
    if args.require_class:
        seen = any(o.classes[c].get(args.require_class, 0) for c in cams)
        rows.append((OK if seen else FAIL, f'class {args.require_class!r}',
                     'seen' if seen else 'never seen'))
    width = max(len(r[1]) for r in rows) + 2
    print()
    for status, check, value in rows:
        print(f'  {status:<8} {check:<{width}} {value}')
    for cam in cams:
        top = ', '.join(f'{c}:{k}' for c, k in o.classes[cam].most_common(5))
        print(f'  {"":<8} {cam + " classes":<{width}} {top or "(none)"}')
    failed = sum(1 for r in rows if r[0] == FAIL)
    print(f'\n[VCHK ] {"FAIL" if failed else "ok"}: {failed} FAIL, '
          f'{sum(1 for r in rows if r[0] == WARN)} WARNING')
    sys.exit(1 if failed else 0)


if __name__ == '__main__':
    main()
