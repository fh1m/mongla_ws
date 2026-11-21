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


LEGEND_PX = 190
# A legend label beyond this is cut by the legend (desktop app, 2026-11-13);
# the panel's title carries what the plot is, the label only which line.
LABEL_CHARS = 13
# A plot's or state panel's title, where its view name is not one.
TITLES = {'authority': 'lock authority', 'pose': 'target pose', 'anchor': 'anchor (XFeat)',
          'align': 'pixel error', 'ladder': 'lock ladder', 'calibration': 'velocity scale',
          'quality': 'flow quality and heights', 'vel': 'velocity',
          'floor': 'heights (depth negative down)', 'floorcam': 'downward camera, the floor read', 'power': 'battery', 'rates': 'body rates',
          'depthyaw': 'depth', 'bytes': 'bytes per second', 'drops': 'board drops',
          'forward': 'forward camera', 'downward': 'downward camera', 'hull': 'attitude',
          'nav': 'odometry, 3D', 'down': 'downward camera',
          'camera': 'calibration camera', 'locmap': 'localization map'}
# The last minute, following the newest sample. Unset, the axis spans the whole
# connection, and a panel opened later drew its data in the right-hand sliver
# (desktop app, 2026-11-12).
PLOT_WINDOW_S = 60


def plot(*lines: dict) -> tuple:
    # Legend at the left: 'top' stacks one line per series above the plot and
    # squashed small plots flat (seen on Kutum); 'floating' covered the data.
    # Lichtblick's default legend is 240 px, half a plot in a narrow column
    # (desktop app, 2026-11-12); LEGEND_PX holds a label such as
    # "board heading" whole (160 cut it).
    return 'Plot', {'paths': list(lines), 'showLegend': True, 'legendDisplay': 'left',
                    'sidebarDimension': LEGEND_PX, 'followingViewWidth': PLOT_WINDOW_S,
                    'isSynced': True, 'xAxisVal': 'timestamp'}


def states(*lines: dict) -> tuple:
    # Not synced: a synced state panel spans the whole connection and pulled
    # every plot in its view out of the last minute (`Reset view`, desktop
    # app 2026-11-12). The plots line up by each following the last minute,
    # and so does this, by `xAxisRange`: without it an unsynced panel also
    # spans the whole connection, and two hours in, the last minute was a
    # sliver at its right edge and the panel read as empty (desktop app,
    # 2026-11-21).
    return 'StateTransitions', {'paths': [{k: v for k, v in s.items() if k != 'enabled'}
                                          for s in lines], 'isSynced': False,
                                'xAxisRange': PLOT_WINDOW_S}


def image(topic: str, *layers: str, synced: bool = False) -> tuple:
    mode = {'imageTopic': topic}
    if layers:
        mode['annotations'] = {a: {'visible': True} for a in layers}
    if synced:
        mode['synchronize'] = True
    return 'Image', {'imageMode': mode}


def camera(cam: str, *extra: str, stream: str = 'stream', synced: bool = False) -> tuple:
    """A camera's own JPEG with what the vision stack drew over it.

    `synced`: show only frames the detector answered, each with its own
    boxes. The boxes come at the detector's rate, ~2 frames after the picture
    (Kutum, 2026-11-12: 20 Hz and 68 ms against 30 Hz and 35 ms), so unsynced
    a box trails a moving target. Synced, the picture stops when the
    detector does, so the cockpit never is: there a live picture comes first.
    """
    return image(f'{V}/{cam}/{stream}/compressed', f'{V}/{cam}/annotations',
                 *(f'{V}/{cam}/{e}' for e in extra), synced=synced)


def align(x_label: str, y_label: str, *more: dict) -> tuple:
    """A vision verb's pixel error, from the Move action's feedback, and `more` in pixels."""
    fb = '/mongla/move/_action/feedback.feedback'
    return plot(series(f'{fb}.err_x_px', x_label, True), series(f'{fb}.err_y_px', y_label, True),
                *more)


def scene() -> tuple:
    """Lichtblick's 3D panel, following the vehicle in the odometry frame."""
    return '3D', {'followTf': 'mongla', 'followMode': 'follow-position',
                  # The odometry, and the demand as an arrow on the hull (overlay.py).
                  'topics': {'/mongla/odom': {'visible': True},
                             '/mongla/batighor/scene': {'visible': True}},
                  # The hull says where the body is; seven frame names drawn
                  # on one point say nothing. Preloading acts on a recording
                  # only, and the panel alerts without it once /tf exists.
                  'scene': {'backgroundColor': '#05070c', 'labelScaleFactor': 0.3,
                            'transforms': {'showLabel': False, 'enablePreloading': True}},
                  # From behind and above, the way a pilot looks at a
                  # vehicle. At 2.2 m a 45-degree view of a wide panel is
                  # 3.4 m across and the 0.7 m hull filled a fifth of it;
                  # at 1.3 m it is 2 m, room for the hull and the demand.
                  'cameraState': {'distance': 1.3, 'perspective': True, 'phi': 55,
                                  'thetaOffset': 200, 'targetOffset': [0, 0, 0],
                                  'target': [0, 0, 0], 'targetOrientation': [0, 0, 0, 1],
                                  'fovy': 45, 'near': 0.01, 'far': 5000},
                  # The hull, hung on the body frame /tf moves (model.py). A
                  # root path, so it resolves against Kutum whatever its name.
                  'layers': {'hull': {'visible': True, 'label': 'Mongla',
                                      'layerId': 'foxglove.Urdf', 'instanceId': 'hull',
                                      'sourceType': 'url', 'url': '/model/mongla.urdf',
                                      'order': 1}}}


def instrument(name: str) -> tuple:
    """One of ours (panels/instruments, panels/control, panels/calibrate)."""
    return PANEL + name, {}


# -- where it goes ----------------------------------------------------------

class View:
    """A layout under construction: panels by id, and the tree that places them.

    `prefix` keeps ids apart when several views share one layout (the console's
    tabs): Lichtblick keys every panel's settings by id, layout-wide."""

    def __init__(self, prefix: str = ''):
        self.config: dict = {}
        self.prefix = prefix

    def __call__(self, name: str, panel: tuple) -> str:
        kind, cfg = panel
        pid = f'{kind}!{self.prefix}{name}'
        assert pid not in self.config, pid
        if not kind.startswith(PANEL):
            # Untitled, every plot read "Plot" and every camera its topic path.
            # Ours title themselves. The key is Lichtblick's own: Foxglove's
            # `foxglovePanelTitle` is ignored by this build (its bundle, 2026-11-18).
            cfg = {**cfg, 'lichtblickPanelTitle': TITLES.get(name, name.replace('_', ' '))}
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

def _ladder_lines() -> list:
    # Each camera's rung over time and which camera is live. What each holds
    # and sees now is the Vision instrument's: as rows here too, the strip
    # needed seven rows of 64 px and scrolled the downward camera out of view.
    return [*(series(f'{V}/{c}/lock{RUNG}.class_id', f'{c} rung') for c in CAMERAS),
            series(f'{V}/active_camera.data', 'live camera', True)]


# Each column of cameras is as wide as its 4:3 pictures, stacked two high:
# wider, the panel was black beside the picture (test_views measures it).
CAMERA_COLUMN = 30


def _cameras(v: View, *, synced: bool = False):
    extra = {'downward': ['flow_annotations', 'floor_annotations']}
    return col(*(v(c, camera(c, *extra.get(c, []), synced=synced)) for c in CAMERAS))


def cockpit(v: View):
    """The default: both cameras, control, and what the vehicle sees and feels.

    Three columns, read left to right: the pictures; the Control Center whole
    (keys, helm, verb, mission, log: at a third of a column ARM and SEND were
    below the fold); the vehicle's numbers and attitude over what holds the
    target now and which stages run. The lock's history is in `vision`."""
    return row((CAMERA_COLUMN, _cameras(v)),
               (32, v('control', instrument('ControlCenter'))),
               (38, col((4, row(v('vehicle', instrument('Vehicle')), v('hull', scene()))),
                        (5, v('vision', instrument('Vision'))))))


def vision(v: View):
    """Detection and tracking in depth: every rung, its authority, the pose solved."""
    fwd, dwn = f'{V}/forward', f'{V}/downward'
    # Two columns. Now: both cameras' rungs and stages side by side (at a
    # sixth of a column the instrument scrolled and hid the downward camera)
    # over the ladder's history, every row in view. Then: the detectors'
    # settings, read against what the ladder does, over the numbers the lock
    # produces, one unit per plot (metres, degrees and pixels on one axis
    # flattened all but the largest).
    now = col((5, v('vision', instrument('Vision'))),
              (5, v('ladder', states(*_ladder_lines()))))
    numbers = col(
        (6, v('detectors', instrument('Detectors'))),
        (2, v('authority', plot(*(series(f'{V}/{c}/lock{RUNG}.score', c) for c in CAMERAS)))),
        (2, row(v('target_range', plot(series(f'{fwd}/target_pose_fused.range_m', 'range m'))),
                v('pose', plot(series(f'{fwd}/target_pose_fused.bearing_deg', 'bearing deg'),
                               series(f'{fwd}/target_pose_fused.yaw_deg', 'yaw deg'))))),
        (2, v('anchor', plot(series(f'{dwn}/anchor_decomposition.vector.x', 'rotation'),
                             series(f'{dwn}/anchor_decomposition.vector.y', 'scale'),
                             series(f'{dwn}/anchor_decomposition.vector.z', 'range m')))),
        (2, v('align', align('align x px', 'align y px',
                             series(f'{fwd}/target_pose_fused.reproj_px', 'reproj px')))))
    right = row(now, numbers)
    return row((CAMERA_COLUMN, col(*(v(c, camera(c, synced=True)) for c in CAMERAS))),
               (100 - CAMERA_COLUMN, right))


def flow(v: View):
    """The downward camera as a speed sensor: the floor tracked, and what it measured."""
    d = f'{V}/downward'
    return row(
        (58, v('floorcam', image(f'{d}/stream/compressed', f'{d}/flow_annotations',
                              f'{d}/floor_annotations'))),
        (42, col(
            (3, v('velocity', plot(series(f'{d}/velocity.twist.twist.linear.x', 'flow vx m/s', True),
                               series(f'{d}/velocity.twist.twist.linear.y', 'flow vy m/s', True),
                               series('/mongla/odom.twist.twist.linear.x', 'filter vx m/s'),
                               series('/mongla/odom.twist.twist.linear.y', 'filter vy m/s')))),
            (2, v('distance', plot(series(f'{d}/distance_traveled.data', 'distance m', True)))),
            (3, v('calibration', states(series(f'{d}/flow_calibration.data', 'scale from',
                                               True),
                                        # tiles | survey | survey, no position |
                                        # pool_depth | none (a mission refuses none)
                                        series('/mongla/flow/height_source.data',
                                               'height from', True)))),
            # Quality and heights read 0 to a few; the track count reads to
            # ~160, and on one axis it flattened them onto the floor.
            (3, v('quality', plot(series(f'{d}/flow_quality.data', 'quality', True),
                                  series(f'{d}/distance_debug.data[1]', 'height m', True),
                                  series(f'{d}/floor_height.range', 'floor m')))),
            (2, v('tracks', plot(series(f'{d}/distance_debug.data[2]', 'tracks', True)))))))


def localization(v: View):
    """Navigation: where the filter puts the vehicle on the pool, and why.

    The localization map on the left (each fix with the correction it made,
    the filter's and the flow's velocity); over time on the right."""
    d = f'{V}/downward'
    return row(
        (50, col((3, v('locmap', instrument('LocalizationMap'))),
                 (2, row(v('navigation', instrument('Navigation')), v('nav', scene()))))),
        (50, col(
            # Motion is on the localization map and the mode on the status
            # strip; as rows here the strip needed 350 px and showed one row.
            (6, v('aiding', states(series('/mongla/localization/state.usable', 'usable', True),
                                   series('/mongla/localization/state.reason', 'why not', True),
                                   series('/mongla/localization/aiding.data', 'aided by', True)))),
            (3, v('pos', plot(series('/mongla/odom.pose.pose.position.x', 'x m'),
                              series('/mongla/odom.pose.pose.position.y', 'y m'),
                              series('/mongla/odom.pose.pose.position.z', 'z m (down)'),
                              series('/mongla/localization/fix.point.x', 'prop fix x'),
                              series('/mongla/localization/fix.point.y', 'prop fix y')))),
            (2, v('sigma', plot(series('/mongla/localization/state.sigma_xy_m', 'sigma xy m', True)))),
            (3, v('vel', plot(series('/mongla/odom.twist.twist.linear.x', 'filter vx'),
                              series(f'{d}/velocity.twist.twist.linear.x', 'flow vx'),
                              series('/mongla/odom.twist.twist.linear.y', 'filter vy'),
                              series(f'{d}/velocity.twist.twist.linear.y', 'flow vy')))),
            (3, v('heading', plot(series('/mongla/state.yaw_deg', 'board heading'),
                                  series('/mongla/localization/heading.data', 'given heading',
                                         True),
                                  series(f'{d}/floor_grid_deg.heading_deg', 'floor grid'),
                                  series(f'{d}/lane_heading_deg.heading_deg', 'lane line')))),
            (3, v('floor', plot(series(f'{d}/floor_height.range', 'floor m'),
                                series('/mongla/state.depth_m', 'depth m')))))))


def thrust(v: View):
    """The chain's far end: what was demanded, what the ESCs turned, what it cost."""
    # The ESC rpm is the Thrust instrument's bars, one per ESC the board reports.
    return row(
        # The instrument is a few lines until the board announces its ESCs;
        # at three fifths it was mostly empty above a squeezed demand plot.
        (50, col((2, v('thrust', instrument('Thrust'))),
                 (3, v('demand', plot(series('/mongla/demand.vector.x', 'surge'),
                                      series('/mongla/demand.vector.y', 'sway')))))),
        (50, col(v('power', plot(series('/mongla/state.battery_voltage', 'PM1 V'))),
                 v('rates', plot(series('/mongla/imu.angular_velocity.x', 'roll rate'),
                                 series('/mongla/imu.angular_velocity.y', 'pitch rate'),
                                 series('/mongla/imu.angular_velocity.z', 'yaw rate'))))))


def board_health(v: View):
    """Every subsystem graded by the manager: the board's on top, Kutum's below."""
    # One reading of /diagnostics per row: Health and Kutum split it, and a
    # DiagnosticSummary beside them showed every row a third time.
    return row(
        # Depth and volts on their own axes: on one, 12 V flattened the depth.
        (55, col(v('depthyaw', plot(series('/mongla/state.depth_m', 'depth m'))),
                 v('battery', plot(series('/mongla/state.battery_voltage', 'PM1 V'))),
                 v('gyro', plot(series('/mongla/imu.angular_velocity.x', 'gx'),
                                series('/mongla/imu.angular_velocity.y', 'gy'),
                                series('/mongla/imu.angular_velocity.z', 'gz'))))),
        (45, col((3, v('health', instrument('Health'))),
                 (2, v('kutum', instrument('Kutum'))))))


def board_link(v: View):
    """The one USB-C line: its use against capacity, the board's drops, every channel."""
    # The per-message rates and named channels are tables in the Link
    # instrument; as raw message trees they read as JSON, not as a link.
    return row(
        (55, col(v('bytes', plot(series('/mongla/board_link.bytes_per_s', 'bytes/s'),
                                 series('/mongla/board_link.capacity_bytes_per_s',
                                        'capacity'))),
                 v('drops', plot(series('/mongla/board_link.board_drop_pct', 'drop %'),
                                 series('/mongla/board_link.board_errors', 'errors'))))),
        (45, v('link', instrument('Link'))))


def chain(v: View):
    """Photons to thrust: each stage's rate and age, beside what the cameras see."""
    # The ages are computed by chain.py (seen minus stamp, per stage), so they
    # are one table in the Chain instrument, not plots of raw stamps.
    return row((CAMERA_COLUMN, col(*(v(c, camera(c)) for c in CAMERAS))),
               (100 - CAMERA_COLUMN, v('chain', instrument('Chain'))))


def map_(v: View):
    """Planning a run: the pool from the course file, the props, and Mongla on it.

    The map takes most of the width: a plan is read off it. Beside it what the
    plan view cannot show: the vehicle's state and attitude, and the decision
    layer (task, verb, how each ended). The pool is drawn to its own shape,
    so a wider map only added black beside it."""
    return row((60, v('map', instrument('Map'))),
               (40, col((4, row(v('vehicle', instrument('Vehicle')), v('hull', scene()))),
                        (5, v('mission', instrument('Mission'))))))


def mission(v: View):
    """The plan beside the pool: the map, the mission's text with the line that
    sent the running verb marked, and the decision layer (task and clock, the
    verb, how each ended) over the demand it sends. Read left to right: where,
    what the plan says, what the vehicle is doing about it."""
    return row((36, v('map', instrument('Map'))),
               (34, v('text', instrument('MissionText'))),
               (30, col((3, v('mission', instrument('Mission'))),
                        (2, v('demand', plot(series('/mongla/demand.vector.x', 'surge'),
                                             series('/mongla/demand.vector.y', 'sway')))))))


def downward(v: View):
    """Everything under the hull: the downward camera at full height with its
    boxes, and beside it what holds the
    target, the ladder, the error an alignment closes, the heights it is
    flown at and the anchor's matched view. The bins, the floor markers and
    any target below are flown from here."""
    d = f'{V}/downward'
    return row(
        # Synchronised, a frame shows only when every layer answered it; the
        # flow and the floor read do not answer every frame, so they are the
        # Flow view's. Here the boxes sit on their own frame.
        (59, v('down', camera('downward', synced=True))),
        (41, col(
            (5, v('vision', instrument('Vision'))),
            (4, v('ladder', states(series(f'{d}/lock{RUNG}.class_id', 'downward rung'),
                                   series(f'{V}/active_camera.data', 'live camera', True)))),
            (3, v('align', align('align x px', 'align y px'))),
            (3, v('floor', plot(series(f'{d}/floor_height.range', 'floor m'),
                                series('/mongla/state.depth_m', 'depth m')))),
            (3, v('anchor', plot(series(f'{d}/anchor_decomposition.vector.x', 'rotation'),
                                 series(f'{d}/anchor_decomposition.vector.y', 'scale')))))))


def calibrate(v: View):
    """A camera calibration: the board found, the next pose wanted, the fit."""
    c = f'{V}/calibration'
    return row((62, col((65, v('camera', image(f'{c}/stream/compressed', f'{c}/annotations'))),
                        (35, v('undistorted', image(f'{c}/undistorted/compressed'))))),
               (38, v('calibrate', instrument('Calibrate'))))


def weak_link(v: View):
    """For a slow connection: the small previews, and the vehicle's numbers."""
    return row((CAMERA_COLUMN, col(*(v(c, camera(c, stream='preview')) for c in CAMERAS))),
               (28, v('vehicle', instrument('Vehicle'))),
               (42, v('control', instrument('ControlCenter'))))


# What the stack publishes and no view shows, each with the reason. A topic
# not shown and not here fails test_views: an operator cannot see it.
NOT_SHOWN = {
    'image_raw': 'the raw frame; every view shows the same frame as stream/compressed',
    'image_debug': "the detector's own drawn frame; the browser draws over the camera's JPEG",
    'classes_filter': "the allowlist; the Control Center shows it (helm_state, detector settings)",
    'model_classes': 'what the model can emit, for the mission DSL to check a course against',
    'imu_rates': 'the body rates again at the board rate, for the filter; /mongla/imu is plotted',
    'distance_control': 'a command to the flow distance bracket, not an observation',
    'video_seek_frame': "a replay command from the display node",
    'video_seek_rel': "a replay command from the display node",
    'video_speed': "a replay command from the display node",
}

VIEWS = {f.__name__.rstrip('_'): f for f in (cockpit, mission, map_, vision, flow, localization,
                                             thrust, board_health, board_link, chain,
                                             downward, calibrate, weak_link)}
FILE = {'cockpit': 'mongla_layout'}            # the default keeps the name links use

# The console: every view as a tab of one layout, in the order a run is read.
# Each view stays its own layout too (links and the layout menu name them);
# here they are one place, and only the open tab's panels subscribe.
CONSOLE = 'batighor'
TABS = [('cockpit', 'Cockpit'), ('map', 'Map'), ('mission', 'Mission'), ('vision', 'Vision'),
        ('flow', 'Flow'), ('localization', 'Localization'), ('thrust', 'Thrust'),
        ('board_health', 'Health'), ('board_link', 'Link'), ('chain', 'Chain'),
        ('downward', 'Downward'), ('calibrate', 'Calibrate')]


def build(name: str) -> dict:
    if name == CONSOLE:
        v, tabs = View(), []
        for view, title in TABS:
            v.prefix = f'{view}.'
            tabs.append({'title': title, 'layout': VIEWS[view](v)})
        v.config['Tab!console'] = {'activeTabIdx': 0, 'tabs': tabs}
        # The status strip on every tab: a split is a percentage of the window,
        # and 5 % is one line of cells on a laptop's 1000 px of height.
        v.prefix = ''
        tree = col((5, v('status', instrument('Status'))), (95, 'Tab!console'))
    else:
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
    for name in [CONSOLE, *VIEWS]:
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
