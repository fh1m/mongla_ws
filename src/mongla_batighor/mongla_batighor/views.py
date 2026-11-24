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
TITLES = {'fwd': 'forward camera', 'dets': "this frame's detections",
          'depth_gauge': 'depth m', 'pack_gauge': 'thruster pack V', 'rail_gauge': 'electronics V',
          'link_pie': 'link share', 'diag_summary': 'diagnostics',
          'diag_detail': 'diagnostic detail', 'track': 'track id ($track)',
          'track_xy': 'the chosen track, centre px', 'per_frame': 'detections per frame',
          'raw_helm': 'helm request, typed whole', 'scripts': 'user scripts', 'pad': 'pad: one short step per press, seated pilot only',
          'baro': 'baro', 'target_range': 'target range',
          'authority': 'lock authority', 'pose': 'target pose', 'anchor': 'anchor (XFeat)',
          'align': 'pixel error', 'ladder': 'lock ladder', 'calibration': 'velocity scale',
          'quality': 'flow quality and heights', 'vel': 'velocity',
          'floor': 'heights (depth negative down)', 'floorcam': 'downward camera, the floor read', 'power': 'battery', 'rates': 'body rates',
          'depthyaw': 'depth', 'bytes': 'bytes per second', 'drops': 'board drops',
          'forward': 'forward camera', 'downward': 'downward camera', 'hull': 'attitude',
          'nav': 'odometry, 3D', 'down': 'downward camera', 'graph': 'nodes and topics: the diamond at the right, "No topics", draws nodes only',
          'topics': 'every topic, its type and rate', 'raw': 'one message, whole',
          'rosout': 'every node\'s log',
          # Empty until a calibration runs, and the panel itself says only that
          # the topic does not exist: the title says when there is a picture.
          # Lichtblick keeps the graph's topic filter in the panel, not its
          # config, so every topic shows at first and the graph is a line of
          # a hundred boxes; the title says where the filter is (bead tci).
          'camera': 'calibration camera, while a calibration runs',
          'undistorted': 'undistorted, once a fit exists',
          'locmap': 'localization map',
          'f2c': 'frame to command: each vision command, how old its frame was when it left'}
# The last minute, following the newest sample. Unset, the axis spans the whole
# connection, and a panel opened later drew its data in the right-hand sliver
# (desktop app, 2026-11-12).
PLOT_WINDOW_S = 60


def legend_px(lines) -> int:
    """The legend as wide as its longest label, never LEGEND_PX for all.

    LEGEND_PX holds a 13-character label whole (160 cut it): about 11.5 px a
    character and 40 of swatch and padding. A plot labelled "gx" spent the same
    190 px, half a narrow column, on blank legend (2026-11-22)."""
    longest = max((len(s.get('label', '')) for s in lines), default=0)
    return max(112, min(LEGEND_PX, round(48 + 11.5 * longest)))


def plot(*lines: dict, y: tuple = ()) -> tuple:
    # Legend at the left: 'top' stacks one line per series above the plot and
    # squashed small plots flat (seen on Kutum); 'floating' covered the data.
    # Lichtblick's default legend is 240 px, half a plot in a narrow column
    # (desktop app, 2026-11-12); LEGEND_PX holds a label such as
    # "board heading" whole (160 cut it).
    # `y`: a bounded quantity keeps its whole range. Autoscaled, a steady
    # authority of 1.0 drew on a 0.94-1.06 axis and read as noise (Kutum,
    # 2026-11-22).
    cfg = {'paths': list(lines), 'showLegend': True, 'legendDisplay': 'left',
           'sidebarDimension': legend_px(lines), 'followingViewWidth': PLOT_WINDOW_S,
           'isSynced': True, 'xAxisVal': 'timestamp'}
    if y:
        cfg['minYValue'], cfg['maxYValue'] = y
    return 'Plot', cfg


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


def scene(pool: bool = False) -> tuple:
    """Lichtblick's 3D panel, following the vehicle in the odometry frame.

    `pool`: the pilot's one 3D view, pulled back to show Mongla in the pool:
    the course in 3D (course.pool_scene: outline, floor grid, every placed
    prop at its depth), the track it has run and the demand. Close in, as the
    cockpit's corner has it, the hull filled the panel and the pool was off
    it (owner, 2026-11-24)."""
    cfg = _scene()
    if pool:
        cfg['topics']['/mongla/batighor/pool_scene'] = {'visible': True}
        cfg['cameraState'] = {**cfg['cameraState'], 'distance': POOL_VIEW_M, 'phi': 50}
    return '3D', cfg


# From behind and above at this range a 45-degree view is about 6 m across:
# the props within a few metres and the track, with the hull still read.
POOL_VIEW_M = 6.0


def _scene() -> dict:
    return {'followTf': 'mongla', 'followMode': 'follow-position',
            # The track and the demand as an arrow on the hull (overlay.py).
            # Not /mongla/odom: the hull is placed by the same pose through
            # /tf, and the odometry's metre-long arrow ran off the panel as
            # a stray line (Chrome, 2026-11-25).
            'topics': {'/mongla/batighor/scene': {'visible': True}},
            # The hull says where the body is; seven frame names drawn
            # on one point say nothing. Preloading acts on a recording
            # only, and the panel alerts without it once /tf exists.
            # labelScaleFactor 1: a text's font_size is its size on screen.
            # At 0.3 the demand's 22 px caption drew at about 6 px and the
            # pool's 14 px names at about 4 px, unreadable (Chrome, 2026-11-25).
            'scene': {'backgroundColor': '#05070c', 'labelScaleFactor': 1.0,
                      'transforms': {'showLabel': False, 'enablePreloading': True}},
            # From behind and above, the way a pilot looks at a
            # vehicle. At 2.2 m a 45-degree view of a wide panel is
            # 3.4 m across and the 0.7 m hull filled a fifth of it;
            # at 1.3 m it is 2 m, room for the hull and the demand.
            # 0.95 m: at 1.3 the hull read small in the cockpit's
            # corner (owner, 2026-11-22); the demand arrow still fits.
            'cameraState': {'distance': 0.95, 'perspective': True, 'phi': 55,
                            'thetaOffset': 200, 'targetOffset': [0, 0, 0],
                            'target': [0, 0, 0], 'targetOrientation': [0, 0, 0, 1],
                            'fovy': 45, 'near': 0.01, 'far': 5000},
            # The hull, hung on the body frame /tf moves (model.py). A
            # root path, so it resolves against Kutum whatever its name.
            'layers': {'hull': {'visible': True, 'label': 'Mongla',
                                'layerId': 'foxglove.Urdf', 'instanceId': 'hull',
                                'sourceType': 'url', 'url': '/model/mongla.urdf',
                                'order': 1}}}


# The site's palette (docs/assets/site.css), for Lichtblick's own panels: red
# for what needs a hand now, amber for caution, ocean for a normal state, ink
# for unknown. The same words a reader of the site already knows.
RED_LIT, AMBER, SEA_LIT, TEAL, INK = '#ff2a14', '#f88701', '#5a8dff', '#22d3c5', '#3a4256'


def gauge(path: str, lo: float, hi: float, reverse: bool = False) -> tuple:
    """A dial with the site's ocean gradient; `reverse` when low is good."""
    return 'Gauge', {'path': path, 'minValue': lo, 'maxValue': hi, 'colorMode': 'gradient',
                     'colorMap': 'red-yellow-green', 'gradient': [INK, SEA_LIT] if not reverse
                     else [SEA_LIT, RED_LIT], 'reverse': False}


def indicator(path: str, rules: list, unknown: str = 'unknown') -> tuple:
    """A lamp: each rule (operator, value, colour, label), first match wins;
    nothing matched, or nothing heard, reads `unknown` on ink, never a state."""
    return 'Indicator', {'path': path, 'style': 'background', 'fallbackColor': INK,
                         'fallbackLabel': unknown,
                         'rules': [{'operator': o, 'rawValue': str(val), 'color': c, 'label': lab}
                                   for o, val, c, lab in rules]}


# MonglaState's tri-states (SAFETY_*), as an Indicator rule's operator and
# value. The manager decodes them; a lamp never re-reads the board's raw
# channels, where a 3 is "not initialised" and a 0 can mean "link lost".
CLEAR, TRIPPED = ('=', 1), ('=', 2)


def table(path: str) -> tuple:
    return 'Table', {'topicPath': path}


def pie(path: str, title: str) -> tuple:
    return 'PieChart', {'path': path, 'title': title}


def slider(var: str, lo: float, hi: float, step: float = 1.0) -> tuple:
    """Sets a global variable the other panels' paths read as `$var`."""
    return 'GlobalVariableSliderPanel', {'sliderProps': {'min': lo, 'max': hi, 'step': step},
                                         'globalVariableName': var}


HELM_SERVICE = '/mongla/batighor/helm'
TELEOP_TOPIC = '/mongla/batighor/teleop'


def call(cmd: str, colour: str, tooltip: str) -> tuple:
    """A one-press verb through the helm, which applies the seat's rules to it
    as to any browser: stop, disarm and surface are open to everyone."""
    payload = json.dumps({'who': 'button', 'op': 'command', 'cmd': cmd, 'fields_json': '',
                          'mission': '', 'confirms': []}, indent=1)
    return 'CallService', {'serviceName': HELM_SERVICE, 'requestPayload': payload,
                           'layout': 'vertical', 'buttonText': cmd.upper(),
                           'buttonTooltip': tooltip, 'buttonColor': colour}


def teleop() -> tuple:
    """Lichtblick's pad. The helm turns each press into one short step for
    the seated pilot (helm.py, `_on_teleop`)."""
    return 'Teleop', {'topic': TELEOP_TOPIC, 'publishRate': 4,
                      'upButton': {'field': 'linear-x', 'value': 1},
                      'downButton': {'field': 'linear-x', 'value': -1},
                      'leftButton': {'field': 'angular-z', 'value': 1},
                      'rightButton': {'field': 'angular-z', 'value': -1}}


def instrument(name: str, **config) -> tuple:
    """One of ours (panels/instruments, panels/control, panels/calibrate);
    `config` is what the panel reads as its initial state (Caption: camera)."""
    return PANEL + name, config


# A caption strip's share of a camera's column: one line of type, under the
# picture rather than drawn into it (owner, 2026-11-24).
CAPTION = 7


def seen(v: 'View', name: str, cam: str, panel: tuple) -> dict:
    """A camera panel with its caption under it: the camera's and the
    detector's state (overlay.caption, on <camera>/hud)."""
    return col((100 - CAPTION, v(name, panel)), (CAPTION, v(f'{name}_caption', instrument('Caption', camera=cam))))


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


# Each column of cameras is as wide as its pictures, stacked two high. The
# vehicle streams 16:9 (640x360, cameras.yaml); a column sized for 4:3 left
# a fifth of each panel black (test_views measures it against the profiles).
CAMERA_COLUMN = 37


def _cameras(v: View, *, synced: bool = False):
    extra = {'downward': ['flow_annotations', 'floor_annotations']}
    return col(*(seen(v, c, c, camera(c, *extra.get(c, []), synced=synced)) for c in CAMERAS))


def cockpit(v: View):
    """The default: both cameras, control, and what the vehicle sees and feels.

    Three columns, read left to right: the pictures; the Control Center whole
    (keys, helm, verb, mission, log: at a third of a column ARM and SEND were
    below the fold); the vehicle's numbers and attitude over what holds the
    target now and which stages run. The lock's history is in `vision`."""
    return row((CAMERA_COLUMN, _cameras(v)),
               (30, v('control', instrument('ControlCenter'))),
               # The Vehicle card carries the board's six lines under its gauges;
               # at 10 of 21 its last row was cut (Kutum, 2026-11-23).
               (33, col((11, row(v('vehicle', instrument('Vehicle')), v('hull', scene()))),
                        (7, v('vision', instrument('Vision'))),
                        (3, v('authority', plot(*(series(f'{V}/{c}/lock{RUNG}.score', c) for c in CAMERAS),
                                             y=(0, 1.05)))))))


def vision(v: View):
    """Detection and tracking in depth: every rung, its authority, the pose solved."""
    fwd, dwn = f'{V}/forward', f'{V}/downward'
    # Two columns. Now: both cameras' rungs and stages side by side (at a
    # sixth of a column the instrument scrolled and hid the downward camera)
    # over the ladder's history, every row in view. Then: the detectors'
    # settings, read against what the ladder does, over the numbers the lock
    # produces, one unit per plot (metres, degrees and pixels on one axis
    # flattened all but the largest).
    # Bearing and reprojection are per-shot numbers: the fuse leaves them NaN,
    # so they come from the solve itself (`target_pose`), not the fused pose.
    # The Vision instrument is a few rows (at half the column it was mostly
    # empty, beside a Detectors panel that scrolled): the lock's authority
    # sits under it, beside the ladder it scores.
    now = col((3, v('vision', instrument('Vision'))),
              (4, v('ladder', states(*_ladder_lines()))),
              (3, v('authority', plot(*(series(f'{V}/{c}/lock{RUNG}.score', c) for c in CAMERAS),
                                   y=(0, 1.05)))))
    numbers = col(
        (5, v('detectors', instrument('Detectors'))),
        (3, row(v('target_range', plot(series(f'{fwd}/target_pose_fused.range_m', 'range m'))),
                v('pose', plot(series(f'{fwd}/target_pose.bearing_deg', 'bearing deg'),
                               series(f'{fwd}/target_pose_fused.yaw_deg', 'yaw deg'))))),
        (3, v('anchor', plot(series(f'{dwn}/anchor_decomposition.vector.x', 'rotation'),
                             series(f'{dwn}/anchor_decomposition.vector.y', 'scale')))),
        (3, v('align', align('align x px', 'align y px',
                             series(f'{fwd}/target_pose.reproj_px', 'reproj px')))))
    right = row(now, numbers)
    return row((CAMERA_COLUMN, col(*(seen(v, c, c, camera(c, synced=True)) for c in CAMERAS))),
               (100 - CAMERA_COLUMN, right))


def flow(v: View):
    """The downward camera as a speed sensor: the floor tracked, and what it measured."""
    d = f'{V}/downward'
    return row(
        # The floor camera at its 16:9 shape; under it how far it says it went
        # and how many points it held.
        (62, col((80, v('floorcam', image(f'{d}/stream/compressed', f'{d}/flow_annotations',
                                          f'{d}/floor_annotations'))),
                 (20, row(v('distance', plot(series(f'{d}/distance_traveled.data', 'distance m',
                                                    True))),
                          v('tracks', plot(series(f'{d}/distance_debug.data[2]', 'tracks',
                                                  True))))))),
        (38, col(
            (3, v('velocity', plot(series(f'{d}/velocity.twist.twist.linear.x', 'flow vx m/s', True),
                               series(f'{d}/velocity.twist.twist.linear.y', 'flow vy m/s', True),
                               series('/mongla/odom.twist.twist.linear.x', 'filter vx m/s'),
                               series('/mongla/odom.twist.twist.linear.y', 'filter vy m/s')))),
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
                                  series(f'{d}/floor_height.range', 'floor m')))))))

def localization(v: View):
    """Navigation: where the filter puts the vehicle on the pool, and why.

    The localization map on the left (each fix with the correction it made,
    the filter's and the flow's velocity); over time on the right."""
    d = f'{V}/downward'
    return row(
        # The map most of the column: at 3/5 the pool drew 220 px tall under
        # its table (2026-11-22); the track and the hull share the rest.
        (50, col((7, v('locmap', instrument('LocalizationMap'))),
                 (3, row(v('navigation', instrument('Navigation')), v('nav', scene()))))),
        (50, col(
            # Motion is on the localization map and the mode on the status
            # strip; as rows here the strip needed 350 px and showed one row.
            (6, v('aiding', states(series('/mongla/localization/state.usable', 'usable', True),
                                   series('/mongla/localization/state.reason', 'why not', True),
                                   series('/mongla/localization/aiding.data', 'aiding', True)))),
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
        # What it cost: the pack the thrusters draw (PM2), not the electronics rail.
        (50, col(v('power', plot(series('/mongla/thruster_pack.voltage', 'thrusters V'))),
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
                 v('battery', plot(series('/mongla/thruster_pack.voltage', 'thrusters V'),
                                   series('/mongla/state.battery_voltage', 'electronics V'))),
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
    return row((CAMERA_COLUMN, col(*(seen(v, c, c, camera(c)) for c in CAMERAS))),
               (100 - CAMERA_COLUMN, col(
                   (7, v('chain', instrument('Chain'))),
                   # Each vision command's frame age when it left (bead a2c):
                   # empty until a vision verb commands.
                   (3, v('f2c', plot(series('/mongla/command_frame.vector.x', 'seconds')))))))


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
        # Flow view's. Here the boxes sit on their own frame, the panel as
        # tall as the 16:9 picture, and under it the error an alignment closes.
        (62, col((80, seen(v, 'down', 'downward', camera('downward', synced=True))),
                 (20, v('align', align('align x px', 'align y px'))))),
        (38, col(
            (5, v('vision', instrument('Vision'))),
            (4, v('ladder', states(series(f'{d}/lock{RUNG}.class_id', 'downward rung'),
                                   series(f'{V}/active_camera.data', 'live camera', True)))),
            (3, v('floor', plot(series(f'{d}/floor_height.range', 'floor m'),
                                series('/mongla/state.depth_m', 'depth m')))),
            (3, v('anchor', plot(series(f'{d}/anchor_decomposition.vector.x', 'rotation'),
                                 series(f'{d}/anchor_decomposition.vector.y', 'scale')))))))


def forward(v: View):
    """Everything ahead: the forward camera at full height with its boxes,
    trails and aim, and beside it what holds the target, the ladder, the
    target's pose and every detection of this frame as a table. Gates,
    markers and anything met head-on are flown from here."""
    f = f'{V}/forward'
    return row(
        (62, col((80, seen(v, 'fwd', 'forward', camera('forward', synced=True))),
                 (20, v('align', align('align x px', 'align y px'))))),
        (38, col(
            (3, v('vision', instrument('Vision'))),
            (4, v('ladder', states(series(f'{f}/lock{RUNG}.class_id', 'forward rung'),
                                   series(f'{V}/active_camera.data', 'live camera', True)))),
            (3, v('target_range', plot(series(f'{f}/target_pose_fused.range_m', 'range m')))),
            (4, v('dets', table(f'{f}/detections.detections'))))))


def vehicle(v: View):
    """The vehicle at a glance, in Lichtblick's own instruments: dials for
    depth and both packs, lamps for arming, mode, leak, kill and the
    barometer, and the board's diagnostics row by row with their values."""
    lamps = row(
        # Each label names its lamp: a lamp's title is cut to one letter in a
        # narrow panel (seen on the ScreenPad), so "unknown" alone said nothing.
        v('armed', indicator('/mongla/state.armed', [('=', 'true', RED_LIT, 'ARMED'),
                                                     ('=', 'false', SEA_LIT, 'disarmed')],
                             unknown='arming not heard')),
        v('mode', indicator('/mongla/state.mode', [('=', 'SURFACE', AMBER, 'SURFACE'),
                                                   ('=', 'AUTO', TEAL, 'AUTO'),
                                                   ('=', 'DEPTH_HOLD', TEAL, 'DEPTH HOLD'),
                                                   ('=', 'STABILIZE', SEA_LIT, 'STABILIZE'),
                                                   ('=', 'MANUAL', AMBER, 'MANUAL')],
                                                  unknown='mode not heard')),
        v('leak', indicator('/mongla/safety.leak', [(*TRIPPED, RED_LIT, 'LEAK'),
                                                   (*CLEAR, SEA_LIT, 'dry')],
                            unknown='leak unknown')),
        v('kill', indicator('/mongla/safety.kill_switch', [(*TRIPPED, RED_LIT, 'KILLED'),
                                                          (*CLEAR, SEA_LIT, 'kill clear')],
                            unknown='kill unseen')),
        v('baro', indicator('/mongla/safety.baro', [(*TRIPPED, RED_LIT, 'BARO FAULT: no moves'),
                                                   (*CLEAR, SEA_LIT, 'baro ok')],
                            unknown='baro unknown')))
    dials = row(v('depth_gauge', gauge('/mongla/state.depth_m', -4.0, 0.0)),
                v('pack_gauge', gauge('/mongla/thruster_pack.voltage', 13.2, 16.8)),
                v('rail_gauge', gauge('/mongla/state.battery_voltage', 0.0, 5.5)))
    return row((50, col((2, lamps), (3, dials),
                        (4, v('link_pie', pie('/mongla/board_link.rates[:].hz',
                                              'what fills the link, frames per second'))))),
               (50, col((4, v('diag_summary', ('DiagnosticSummary', {'topicToRender': '/diagnostics',
                                                                    'sortByLevel': True}))),
                        (6, v('diag_detail', ('DiagnosticStatusPanel',
                                              {'topicToRender': '/diagnostics'}))))))


def pilot(v: View):
    """The main screen when Vitals has the screen under it: the pictures as
    big as the screen allows, side by side across the top, and under them
    what Mongla makes of them: what holds the target now, the hull's
    attitude, and the pool with Mongla on it. The keys and the vehicle's
    state are on Vitals; helm, verbs and missions have their own tabs (the
    console's Helm and Mission), so none of them crowds the pictures."""
    return col((64, row(seen(v, 'forward', 'forward', camera('forward')),
                        seen(v, 'downward', 'downward', camera('downward', 'flow_annotations', 'floor_annotations')))),
               (36, row((32, v('vision', instrument('Vision'))),
                        (44, v('pool', scene(pool=True))),
                        (24, v('mission', instrument('Mission'))))))


def vitals(v: View):
    """How the vehicle is, for the touchscreen under the main screen (the
    Zenbook Duo's ScreenPad, 1920 x 550), in its own window: read at a glance
    while the eyes stay on the cameras above. Left, the keys, tall enough for
    a thumb; then the Vitals drawing (depth, heading, the demand as water past
    the hull, each ESC turning, arming, mode, packs, position and the board's
    six lines); then where Mongla is in the pool, north up. Each thing once:
    a first build repeated the vehicle's state as lamps, traces and a card
    side by side, and a 3D hull here and above both showed a zoomed-in hull
    and nothing else (seen on the Duo, 2026-11-24). The one 3D view is the
    pilot's, on the big screen."""
    return row((11, v('keys', instrument('Keys'))),
               (59, v('vitals', instrument('Vitals'))),
               (30, v('map', instrument('Map'))))


def tune(v: View):
    """Following one track, and the browser's own scripts. The slider sets
    `$track`; the plot follows that track's box on both cameras. The user
    script beside them derives a series in the browser, at no cost to Kutum."""
    t = '.detections[:]{id==$track}'
    return row((55, col((2, v('track', slider('track', 0, 3000))),
                        (5, v('track_xy', plot(
                            series(f'{V}/forward/tracks{t}.bbox.center.position.x', 'fwd x px'),
                            series(f'{V}/forward/tracks{t}.bbox.center.position.y', 'fwd y px'),
                            series(f'{V}/downward/tracks{t}.bbox.center.position.x', 'dwn x px'),
                            series(f'{V}/downward/tracks{t}.bbox.center.position.y', 'dwn y px')))),
                        (4, v('per_frame', plot(
                            series('/studio_script/mongla_detections_per_frame.forward', 'forward', True),
                            series('/studio_script/mongla_detections_per_frame.downward', 'downward', True)))))),
               (45, v('scripts', ('NodePlayground', {'selectedNodeId': 'mongla-detections-per-frame',
                                                     'autoFormatOnSave': True}))))


def helm(v: View, center: str = 'ControlCenter'):
    """Driving by hand: the Control Center, Lichtblick's pad for short steps,
    and the helm's request typed whole (prefilled with stop), for when a
    button is not enough to see what the helm answers. All through the seat.
    The hull is Vitals' and the cockpit's, not repeated here."""
    return row((50, v('control', instrument(center))),
               (50, col((5, v('pad', teleop())),
                        (3, v('raw_helm', call('stop', AMBER, 'any helm request, typed as JSON'))))))


def command(v: View):
    """The helm for the screen above Vitals: its keys are on Vitals."""
    return helm(v, 'Command')


def calibrate(v: View):
    """A camera calibration: the board found, the next pose wanted, the fit."""
    c = f'{V}/calibration'
    return row((62, col((65, v('camera', image(f'{c}/stream/compressed', f'{c}/annotations'))),
                        (35, v('undistorted', image(f'{c}/undistorted/compressed'))))),
               (38, v('calibrate', instrument('Calibrate'))))


def weak_link(v: View):
    """For a slow connection: the small previews, and the vehicle's numbers."""
    return row((CAMERA_COLUMN, col(*(seen(v, c, c, camera(c, stream='preview')) for c in CAMERAS))),
               (25, v('vehicle', instrument('Vehicle'))),
               (38, v('control', instrument('ControlCenter'))))


def inspect(v: View):
    """Everything on the wire, for when a number looks wrong: the chain from
    photons to thrust drawn stage by stage, every topic with its type and
    rate, and the vehicle's state message whole, field by field. Parameters
    stay closed to a browser (batighor.launch.py, BRIDGE_LIMITS)."""
    # The chain drawn, where Lichtblick's topic graph was: that put ~90 nodes
    # and topics in one band, unreadable even across a 3824 px screen
    # (2026-11-24, bead 318.2). Grouped by stage, each box says its rate and
    # age, and what the board does not send is drawn as the gap it is.
    return col((58, v('chain', instrument('Chain'))),
               (42, row((50, v('topics', ('SourceInfo', {}))),
                        (50, v('raw', ('RawMessages', {'topicPath': '/mongla/state',
                                                       'diffEnabled': False,
                                                       'expansion': 'all'}))))))


def files(v: View):
    """The missions and the deck as Kutum holds them, read only (shelves.py):
    what a run will execute and start with, without a shell on the vehicle.
    Beside it, the mission running now, line by line."""
    return row((64, v('files', instrument('Files'))),
               (36, v('text', instrument('MissionText'))))


def log_(v: View):
    """What every node says, newest last: the ROS log at info and above, beside
    the helm's own record of who asked for what (its keys and verbs are the
    Helm tab's, not repeated here)."""
    return row((68, v('rosout', ('RosOut', {'searchTerms': [], 'minLogLevel': 2,
                                            'topicToRender': '/rosout'}))),
               (32, v('helm_log', instrument('HelmLog'))))


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
                                             forward, downward, vehicle, tune, helm,
                                             calibrate, weak_link, inspect, log_,
                                             pilot, vitals, command, files)}
FILE = {'cockpit': 'mongla_layout'}            # the default keeps the name links use

# The console: every view as a tab of one layout, in the order a run is read.
# Each view stays its own layout too (links and the layout menu name them);
# here they are one place, and only the open tab's panels subscribe.
CONSOLE = 'batighor'
# One tab per aspect, and no two tabs showing the same thing: 18 tabs did not
# fit one row at the working scale and their labels ran together, and five
# of them repeated another (2026-11-24). The rest stay layouts of their own,
# in the layout menu: map (the Mission tab has the map), chain (Inspect
# draws it; this one sets it beside the cameras), forward and downward (one
# camera at full height; Vision has both), and vehicle (Lichtblick's own
# gauges, a cross-check on ours).
TABS = [('cockpit', 'Cockpit'), ('mission', 'Mission'), ('helm', 'Helm'),
        ('localization', 'Navigation'), ('vision', 'Vision'), ('flow', 'Flow'),
        ('thrust', 'Thrust'), ('board_health', 'Health'), ('board_link', 'Link'),
        ('log', 'Log'), ('files', 'Files'), ('tune', 'Tune'), ('calibrate', 'Calibrate'),
        ('inspect', 'Inspect')]
# The same console for the main screen when Vitals has the screen under it:
# Pilot first, and no Cockpit tab and no status strip, since the keys and the
# vehicle's state are on Vitals already. The tab bar stays: it is the index
# to every view.
PILOT_CONSOLE = 'batighor_pilot'
PILOT_TABS = [('pilot', 'Pilot')] + [('command', 'Helm') if t[0] == 'helm' else t
                                     for t in TABS if t[0] != 'cockpit']
CONSOLES = {CONSOLE: TABS, PILOT_CONSOLE: PILOT_TABS}


# Lichtblick user scripts, run in the browser on what it already receives, so
# a derived series costs the vehicle nothing. Each publishes on
# /studio_script/<name>.
USER_SCRIPTS = {'mongla-detections-per-frame': {
    'name': 'detections per frame',
    'sourceCode': """import { Input } from "./types";

// How many boxes each camera's detector gave on its latest frame: a count
// that drops to zero while the picture runs is a detector that lost the target.
export const inputs = ["/mongla/vision/forward/detections", "/mongla/vision/downward/detections"];
export const output = "/studio_script/mongla_detections_per_frame";

const last = { forward: 0, downward: 0 };

export default function script(
  event: Input<"/mongla/vision/forward/detections"> | Input<"/mongla/vision/downward/detections">,
): { forward: number; downward: number } {
  const cam = event.topic.includes("/forward/") ? "forward" : "downward";
  last[cam] = event.message.detections.length;
  return { ...last };
}
"""}}


def build(name: str) -> dict:
    if name in CONSOLES:
        v, tabs = View(), []
        for view, title in CONSOLES[name]:
            v.prefix = f'{view}.'
            tabs.append({'title': title, 'layout': VIEWS[view](v)})
        v.config['Tab!console'] = {'activeTabIdx': 0, 'tabs': tabs}
        # The status strip on every tab: a split is a percentage of the window,
        # and 5 % is one line of cells on a laptop's 1000 px of height.
        v.prefix = ''
        # The pilot console's strip keeps only what Vitals, below, does not
        # show; either strip marks the tabs and reports this screen's lag.
        strip = 'Status' if name == CONSOLE else 'Watch'
        tree = col((5, v('status', instrument(strip))), (95, 'Tab!console'))
    else:
        v = View()
        tree = VIEWS[name](v)
    return {'configById': v.config, 'globalVariables': {'track': 0}, 'userNodes': USER_SCRIPTS,
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
    for name in [*CONSOLES, *VIEWS]:
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
