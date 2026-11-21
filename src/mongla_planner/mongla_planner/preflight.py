"""Before a mission moves: does the velocity sensor have a floor, and is there a depth floor?

Two settings decide whether a run is safe and whether it can navigate, and
both used to be warnings the operator could scroll past:

  * Flow's height. Without the floor's depth (a course's `pool:`, or
    `pool_depth_m`), flow refuses every interval and localization drifts on
    the IMU alone. The run looks healthy and has no velocity. The runner now
    gives flow the course's floor first, then REFUSES to start while flow
    still has none, unless the operator says `--allow-no-height` (a bench run
    with no floor).
  * The depth floor (`shield_max_depth_m`, the course's `pool:`). Unset,
    nothing below the mission stops a dive past the pool. It stays a warning, because the deck may be
    a bench, but it is printed at every mission start until it is set.

The course is the mission's own `COURSE` if it names one, else the deck's
`course:` (in `~/mongla/deck/bringup.yaml`). The decisions are pure functions
here; `run()` does the ROS half.
"""
from __future__ import annotations

import sys
import time

# Flow states in which a run can measure velocity. 'survey, no position' scales
# on the middle of the surveyed floor with its spread in the covariance, until
# localization anchors to the pool.
HEIGHT_OK = ('tiles', 'survey', 'survey, no position', 'pool_depth')


def height_decision(source, allow_no_height: bool) -> tuple[bool, str]:
    """(may start, what to say). `source` is flow's latched height source, or
    None when no flow node answered (flow:=false, or not up)."""
    if source is None:
        return True, ('WARNING: no flow node answered: this run has no velocity '
                      'sensor, and localization drifts on the IMU alone')
    if source in HEIGHT_OK:
        return True, f'ok flow height from {source}'
    if allow_no_height:
        return True, ('WARNING: flow has no floor and --allow-no-height was given: '
                      'every velocity interval will be refused this run')
    return False, ('ERROR: flow has no floor, so it would refuse every interval and '
                   'the run would navigate on the IMU alone. Name the venue\'s course '
                   '(course: in ~/mongla/deck/bringup.yaml, or COURSE in the mission) '
                   'whose pool: gives depth_m or floor: points, or launch with '
                   'pool_depth_m:=<m>. A bench run with no floor: --allow-no-height.')


def shield_warning(floor_m) -> str:
    """'' when a depth floor is set, else the line to print at every start."""
    try:
        f = float(floor_m)
    except (TypeError, ValueError):
        f = 0.0
    if f > 0.0:
        return ''
    return ('WARNING: no depth floor. shield_max_depth_m is unset, so nothing below '
            'the mission stops a dive past the pool. Set pool: shield_max_depth_m '
            'in the venue\'s course, ~/mongla/deck/courses/<name>.yaml.')


def course_for(mission_fn) -> str:
    """The mission module's `COURSE`, else the deck's `course:`, else ''."""
    mod = sys.modules.get(getattr(mission_fn, '__module__', ''), None)
    c = getattr(mod, 'COURSE', None)
    if isinstance(c, str) and c.strip():
        return c.strip()
    return deck_course()


def deck_course() -> str:
    """`course:` from the deck file, or ''."""
    try:
        import yaml
        from mongla_runtime.launch_defaults import config_path
        path = config_path()
        if path is None:
            return ''
        raw = yaml.safe_load(path.read_text()) or {}
        c = raw.get('course')
        return str(c).strip() if c else ''
    except Exception:                                    # noqa: BLE001
        return ''


def _latched_string(node, topic: str, timeout: float, *, good=()):
    """Last value on a latched String topic within `timeout`, or None.

    Returns early on a value in `good`. Flow re-publishes its height source
    once a second on change, so right after `use_course` the latched value
    can still be the old one; listening to the end of `timeout` catches the
    new one.
    """
    import rclpy
    from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
    from std_msgs.msg import String
    got = []
    sub = node.create_subscription(
        String, topic, lambda m: got.append(m.data),
        QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                   durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and not (got and got[-1] in good):
        rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_subscription(sub)
    return got[-1] if got else None


def _manager_param(node, name: str, timeout: float = 2.0):
    import rclpy
    from rcl_interfaces.srv import GetParameters
    cli = node.create_client(GetParameters, '/mongla_manager/get_parameters')
    try:
        if not cli.wait_for_service(timeout_sec=timeout):
            return None
        fut = cli.call_async(GetParameters.Request(names=[name]))
        rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
        res = fut.result()
        if res is None or not res.values:
            return None
        return res.values[0].double_value
    finally:
        node.destroy_client(cli)


def run(mongla, node, log, mission_fn, *, allow_no_height: bool) -> None:
    """Give flow the course's floor, then refuse a run flow cannot scale."""
    warn = shield_warning(_manager_param(node, 'shield_max_depth_m'))
    if warn:
        log.warning(warn)
    course = course_for(mission_fn)
    if course:
        mongla.use_course(course)
    else:
        log.info('[PRE  ] no course named (mission COURSE or the deck\'s course:)')
    src = _latched_string(node, '/mongla/flow/height_source', 2.5, good=HEIGHT_OK)
    ok, msg = height_decision(src, allow_no_height)
    (log.info if msg.startswith('ok') else log.warning if ok else log.error)(
        f'[PRE  ] {msg}')
    if not ok:
        raise RuntimeError(msg)
