"""Every Batighor view, from one table: `layouts/*.json` are generated here.

A view is one aspect of Mongla, laid out the way a deck reads it: what the
vehicle sees on the left at the size of a picture, the numbers and the
controls on the right where a hand and an eye go next. Each aspect is its own
layout, chosen from Lichtblick's layout menu, so no one screen is crammed.

Extending it is one edit here, never a new hand-made layout:
- a camera added to `seat.CAMERAS` appears in every per-camera view;
- a new sensor is one series in the view of the aspect it measures;
- a new aspect is one function in `VIEWS`;
- a new instrument is one entry in `panels/instruments` and one `instrument()`.

`ros2 run mongla_batighor views --write` rewrites the layouts (no argument:
exit 1 naming each stale one);
`test_views.py` fails when a committed layout is not what this generates.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from mongla_batighor.seat import CAMERAS

V = '/mongla/vision'
RUNG = '.detections[0].results[0].hypothesis'
PANEL = 'mongla.batighor.'
LAYOUTS = Path(__file__).resolve().parents[1] / 'layouts'


# -- what a panel shows ---------------------------------------------------

def series(path: str, label: str, receive: bool = False) -> dict:
    """One line on a plot or a state strip. `receive` for a topic with no header."""
    s = {'value': path, 'label': label, 'enabled': True}
    if receive:
        s['timestampMethod'] = 'receiveTime'
    return s


def plot(*lines: dict) -> tuple:
    # Legend at the left: 'top' stacks one line per series above the plot and
    # squashed small plots flat (seen on Kutum); 'floating' covered the data.
    return 'Plot', {'paths': list(lines), 'showLegend': True, 'legendDisplay': 'left',
                    'isSynced': True, 'xAxisVal': 'timestamp'}


def states(*lines: dict) -> tuple:
    return 'StateTransitions', {'paths': [{k: v for k, v in s.items() if k != 'enabled'}
                                          for s in lines], 'isSynced': True}


def image(topic: str, *layers: str) -> tuple:
    mode = {'imageTopic': topic}
    if layers:
        mode['annotations'] = {a: {'visible': True} for a in layers}
    return 'Image', {'imageMode': mode}


def camera(cam: str, *extra: str, stream: str = 'stream') -> tuple:
    """A camera's own JPEG with what the vision stack drew over it."""
    return image(f'{V}/{cam}/{stream}/compressed', f'{V}/{cam}/annotations',
                 *(f'{V}/{cam}/{e}' for e in extra))


def align(x_label: str, y_label: str) -> tuple:
    """A vision verb's pixel error, from the Move action's feedback."""
    fb = '/mongla/move/_action/feedback.feedback'
    return plot(series(f'{fb}.err_x_px', x_label, True), series(f'{fb}.err_y_px', y_label, True))


def raw(topic: str) -> tuple:
    return 'RawMessages', {'topicPath': topic}


def diagnostics() -> tuple:
    return 'DiagnosticSummary', {'minLevel': 0, 'pinnedIds': [], 'hardwareIdFilter': 'mongla',
                                 'topicToRender': '/diagnostics', 'sortByLevel': True}


def scene() -> tuple:
    """Lichtblick's 3D panel, following the vehicle in the odometry frame."""
    return '3D', {'followTf': 'mongla', 'followMode': 'follow-position',
                  'topics': {'/mongla/odom': {'visible': True}},
                  'scene': {'backgroundColor': '#05070c'}}


def instrument(name: str) -> tuple:
    """One of ours (panels/instruments, panels/control, panels/calibrate)."""
    return PANEL + name, {}


# -- where it goes ----------------------------------------------------------

class View:
    """A layout under construction: panels by id, and the tree that places them."""

    def __init__(self):
        self.config: dict = {}

    def __call__(self, name: str, panel: tuple) -> str:
        kind, cfg = panel
        pid = f'{kind}!{name}'
        assert pid not in self.config, pid
        self.config[pid] = cfg
        return pid


def _split(direction: str, parts) -> dict | str:
    """Lichtblick's mosaic splits in two; n weighted parts become a nested chain."""
    parts = [p if isinstance(p, tuple) else (1, p) for p in parts]
    if len(parts) == 1:
        return parts[0][1]
    total = sum(w for w, _ in parts)
    return {'direction': direction, 'first': parts[0][1],
            'second': _split(direction, parts[1:]),
            'splitPercentage': round(100 * parts[0][0] / total, 1)}


def row(*parts):
    return _split('row', parts)


def col(*parts):
    return _split('column', parts)


# -- the views --------------------------------------------------------------

def _ladder_lines(detail: bool) -> list:
    out = []
    for c in CAMERAS:
        out.append(series(f'{V}/{c}/lock{RUNG}.class_id', f'{c} rung'))
        if detail:
            out.append(series(f'{V}/{c}/lock.detections[0].id', f'{c} holds'))
            out.append(series(f'{V}/{c}/seeing.data', f'{c} seeing', True))
    out.append(series(f'{V}/active_camera.data', 'live camera', True))
    return out


def cockpit(v: View):
    """The default: both cameras, the lock's history, the vehicle, and control."""
    extra = {'downward': ['flow_annotations']}
    cams = [(3 if i == 0 else 2, v(c, camera(c, *extra.get(c, []))))
            for i, c in enumerate(CAMERAS)]
    strip = v('ladder', states(*_ladder_lines(False),
                               series('/mongla/state.mode', 'mode'),
                               series('/mongla/mission_alive.data', 'mission', True),
                               series('/mongla/localization/aiding.data', 'aided by', True)))
    return row((62, col(*cams, (1, strip))),
               (38, col((11, v('control', instrument('ControlCenter'))),
                        (5, v('vehicle', instrument('Vehicle'))),
                        (5, v('vision', instrument('Vision'))))))


def vision(v: View):
    """Detection and tracking in depth: every rung, its authority, the pose solved."""
    cams = [(3, v(c, camera(c))) for c in CAMERAS]
    fwd, dwn = f'{V}/forward', f'{V}/downward'
    # How far each detection is, when depth_estimation runs (off by default).
    depth = row(v('range_map', image(f'{fwd}/vis_range_map')), v('range', raw(f'{fwd}/vis_range')))
    right = col(
        (3, v('vision', instrument('Vision'))),
        (3, v('ladder', states(*_ladder_lines(True)))),
        (2, v('authority', plot(*(series(f'{V}/{c}/lock{RUNG}.score', f'{c} authority')
                                  for c in CAMERAS)))),
        (2, v('pose', plot(series(f'{fwd}/target_pose_fused.range_m', 'range m'),
                           series(f'{fwd}/target_pose_fused.bearing_deg', 'bearing deg'),
                           series(f'{fwd}/target_pose_fused.yaw_deg', 'target yaw deg'),
                           series(f'{fwd}/target_pose_fused.reproj_px', 'reprojection px')))),
        (2, v('anchor', plot(series(f'{dwn}/anchor_decomposition.vector.x', 'anchor rotation'),
                             series(f'{dwn}/anchor_decomposition.vector.y', 'anchor scale'),
                             series(f'{dwn}/anchor_decomposition.vector.z', 'anchor range m')))),
        (2, v('align', align('align err x px', 'align err y px'))))
    return row((55, col(*cams, (2, depth))), (45, right))


def flow(v: View):
    """The downward camera as a speed sensor: the floor tracked, and what it measured."""
    d = f'{V}/downward'
    return row(
        (58, v('floor', image(f'{d}/stream/compressed', f'{d}/flow_annotations'))),
        (42, col(
            v('velocity', plot(series(f'{d}/velocity.twist.twist.linear.x', 'flow vx m/s', True),
                               series(f'{d}/velocity.twist.twist.linear.y', 'flow vy m/s', True),
                               series('/mongla/odom.twist.twist.linear.x', 'filter vx m/s'),
                               series('/mongla/odom.twist.twist.linear.y', 'filter vy m/s'))),
            v('distance', plot(series(f'{d}/distance_traveled.data', 'distance m', True))),
            v('calibration', states(series(f'{d}/flow_calibration.data', 'velocity scale from',
                                           True))),
            v('quality', plot(series(f'{d}/flow_quality.data', 'quality', True),
                              series(f'{d}/distance_debug.data[2]', 'tracks', True),
                              series(f'{d}/distance_debug.data[1]', 'height m', True),
                              series(f'{d}/floor_height.range', 'floor height m'))))))


def localization(v: View):
    """Navigation: where the filter puts the vehicle, in 3D and over time, and why."""
    d = f'{V}/downward'
    return row(
        (50, col((3, v('nav', scene())), (2, v('navigation', instrument('Navigation'))))),
        (50, col(
            (2, v('aiding', states(series('/mongla/localization/aiding.data', 'aided by', True),
                                   series('/mongla/localization/motion.data', 'motion', True),
                                   series('/mongla/state.mode', 'mode')))),
            (3, v('pos', plot(series('/mongla/odom.pose.pose.position.x', 'x m'),
                              series('/mongla/odom.pose.pose.position.y', 'y m'),
                              series('/mongla/odom.pose.pose.position.z', 'z m (down)'),
                              series('/mongla/localization/fix.point.x', 'prop fix x'),
                              series('/mongla/localization/fix.point.y', 'prop fix y')))),
            (3, v('vel', plot(series('/mongla/odom.twist.twist.linear.x', 'filter vx'),
                              series(f'{d}/velocity.twist.twist.linear.x', 'flow vx'),
                              series('/mongla/odom.twist.twist.linear.y', 'filter vy'),
                              series(f'{d}/velocity.twist.twist.linear.y', 'flow vy')))),
            (3, v('heading', plot(series('/mongla/state.yaw_deg', 'board heading'),
                                  series('/mongla/localization/heading.data', 'given heading',
                                         True),
                                  series(f'{d}/floor_grid_deg.heading_deg', 'floor grid'),
                                  series(f'{d}/lane_heading_deg.heading_deg', 'lane line')))),
            (3, v('floor', plot(series(f'{d}/floor_height.range', 'floor height m'),
                                series('/mongla/state.depth_m', 'depth m (negative down)')))))))


def thrust(v: View):
    """The chain's far end: what was demanded, what the ESCs turned, what it cost."""
    return row(
        (50, col((3, v('thrust', instrument('Thrust'))),
                 (2, v('demand', plot(series('/mongla/demand.vector.x', 'demand surge'),
                                      series('/mongla/demand.vector.y', 'demand sway')))))),
        (50, col(v('power', plot(series('/mongla/state.battery_voltage', 'PM1 V'))),
                 v('rates', plot(series('/mongla/imu.angular_velocity.x', 'roll rate'),
                                 series('/mongla/imu.angular_velocity.y', 'pitch rate'),
                                 series('/mongla/imu.angular_velocity.z', 'yaw rate'))),
                 v('esc', raw('/mongla/esc_rpm')))))


def board_health(v: View):
    """Every subsystem graded by the manager, the board's and Kutum's alike."""
    return row(
        (55, col(v('depthyaw', plot(series('/mongla/state.depth_m', 'depth m'),
                                    series('/mongla/state.battery_voltage', 'PM1 V'))),
                 v('gyro', plot(series('/mongla/imu.angular_velocity.x', 'gx'),
                                series('/mongla/imu.angular_velocity.y', 'gy'),
                                series('/mongla/imu.angular_velocity.z', 'gz'))),
                 v('order', raw('/mongla/flare_order')))),
        (45, col((3, v('health', instrument('Health'))),
                 (2, v('kutum', instrument('Kutum'))),
                 (3, v('graded', diagnostics())))))


def board_link(v: View):
    """The one USB-C line: its use against capacity, the board's drops, every channel."""
    return row(
        (45, col((2, v('link', instrument('Link'))),
                 (2, v('bytes', plot(series('/mongla/board_link.bytes_per_s', 'bytes/s'),
                                     series('/mongla/board_link.capacity_bytes_per_s',
                                            'the line, 115200 8N1')))),
                 (2, v('drops', plot(series('/mongla/board_link.board_drop_pct', 'board drop %'),
                                     series('/mongla/board_link.board_errors', 'board errors')))))),
        (55, row((55, v('named', raw('/mongla/board_link.named'))),
                 (45, v('rates', raw('/mongla/board_link.rates'))))))


def bins_downward(v: View):
    """Aligning over a target below: the floor with boxes, the detections, the error."""
    return row((60, v('down', camera('downward'))),
               (40, col(v('dets', raw(f'{V}/downward/detections')),
                        v('align', align('err_x_px', 'err_y_px')))))


def calibrate(v: View):
    """A camera calibration: the board found, the next pose wanted, the fit."""
    c = f'{V}/calibration'
    return row((62, col((65, v('camera', image(f'{c}/stream/compressed', f'{c}/annotations'))),
                        (35, v('undistorted', image(f'{c}/undistorted/compressed'))))),
               (38, v('calibrate', instrument('Calibrate'))))


def weak_link(v: View):
    """For a slow connection: the small previews, and the vehicle's numbers."""
    return row((70, col(*(v(c, camera(c, stream='preview')) for c in CAMERAS))),
               (30, col(v('vehicle', instrument('Vehicle')),
                        v('control', instrument('ControlCenter')))))


# What the stack publishes and no view shows, each with the reason. A topic
# not shown and not here fails test_views: an operator cannot see it.
NOT_SHOWN = {
    'image_raw': 'the raw frame; every view shows the same frame as stream/compressed',
    'image_debug': "the detector's own drawn frame; the browser draws over the camera's JPEG",
    'classes_filter': "the allowlist; the Control Center shows it (helm_state, detector settings)",
    'imu_rates': 'the body rates again at the board rate, for the filter; /mongla/imu is plotted',
    'distance_control': 'a command to the flow distance bracket, not an observation',
    'video_seek_frame': "a replay command from the display node",
    'video_seek_rel': "a replay command from the display node",
    'video_speed': "a replay command from the display node",
}

VIEWS = {f.__name__: f for f in (cockpit, vision, flow, localization, thrust, board_health,
                                 board_link, bins_downward, calibrate, weak_link)}
FILE = {'cockpit': 'mongla_layout'}            # the default keeps the name links use


def build(name: str) -> dict:
    v = View()
    tree = VIEWS[name](v)
    return {'configById': v.config, 'globalVariables': {}, 'userNodes': {},
            'playbackConfig': {'speed': 1}, 'layout': tree}


def render(name: str) -> str:
    return json.dumps(build(name), indent=2) + '\n'


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--write', action='store_true', help='rewrite layouts/ from this table')
    ap.add_argument('--dir', type=Path, default=None,
                    help='the source tree\'s layouts/ (default: found from here)')
    a = ap.parse_args(argv)
    # The source tree's layouts, never the installed copy: run installed, this
    # file sits in site-packages and a write there would vanish on the next build.
    out = a.dir or (LAYOUTS if LAYOUTS.is_dir() else Path.cwd() / 'src/mongla_batighor/layouts')
    if not out.is_dir():
        print(f'ERROR: no layouts at {out}: run from the workspace root or pass --dir',
              file=sys.stderr)
        return 2
    stale = []
    for name in VIEWS:
        path = out / f'{FILE.get(name, name)}.json'
        text = render(name)
        if a.write:
            path.write_text(text)
        elif not path.is_file() or path.read_text() != text:
            stale.append(path.name)
    for s in stale:
        print(f'stale: {s} (ros2 run mongla_batighor views --write)', file=sys.stderr)
    return 1 if stale else 0


if __name__ == '__main__':
    sys.exit(main())
