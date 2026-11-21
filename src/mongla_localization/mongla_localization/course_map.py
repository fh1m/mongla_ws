"""Where the props are, per course. The prior half of perceive-then-move.

What this is for. A mission that searches for every prop from scratch spends
its run turning. BumblebeeAS do not: their mothers dead-reckon to roughly where
a prop should be and let perception take over for the last few metres -- and on
the task where perception was hardest, the slalom, they shipped plain waypoints
and never ran the 3,000-line particle filter they had built. A prior map is the
cheapest autonomy in the sport.

And what makes it safe here. Every number in a course file is a claim about a
pool we may not have measured. So a course entry is INCOMPLETE BY DEFAULT: the
loader refuses to answer for a prop whose position was never set, rather than
handing back a zero that a mission would drive to. `measured: false` is a
statement that the value is a rulebook nominal, not our pool -- our props are
not the venue's, and a range or a bearing off by a metre is a mission that
arrives somewhere else with total confidence.

One number, one place. A prop's `bearing_deg` -- the compass direction its
face points -- is what `mongla.anchor_heading()` needs to turn a landmark into
an absolute heading, AND what a waypoint needs to approach it from the front.
Writing it twice is how the two come to disagree, so the course file is the
single source and the DSL reads the anchor bearing from here.

Pure loading and validation. No ROS.
"""
from __future__ import annotations

import difflib
import math
import os
import pathlib
from dataclasses import dataclass
from typing import Dict, Optional

import yaml

_PKG_COURSES = pathlib.Path(__file__).resolve().parent / 'courses'

# Deck-side override, same contract as the geometry table: our pool is not the
# venue, and a course must be re-measurable without a colcon build.
_ENV = 'MONGLA_COURSE_DIR'
_HOME = pathlib.Path.home() / '.mongla' / 'courses'

# Every key a course file may hold. Anything else is refused: a typo such as
# `bearing:` for `bearing_deg:` would otherwise load as "unset" and surface
# minutes later as a refusal that names the right key, on the venue's day.
TOP_KEYS = ('course', 'frame', 'pool', 'props')
PROP_KEYS = ('x_m', 'y_m', 'depth_m', 'bearing_deg', 'detect_class', 'measured',
             'note', 'width_m')
POOL_KEYS = ('depth_m', 'depth_min_m', 'depth_max_m', 'floor', 'tile_m',
             'lane_lines', 'shield_max_depth_m')

# The venue's numbers that are bringup launch arguments: pool key -> argument.
# The course is their ONE place. A deck file that also sets one is refused at
# launch (two copies of one fact), and a typed name:=value still wins.
VENUE_LAUNCH = {'depth_m': 'pool_depth_m', 'depth_min_m': 'pool_depth_min_m',
                'depth_max_m': 'pool_depth_max_m', 'tile_m': 'tile_m',
                'lane_lines': 'lane_lines'}


@dataclass(frozen=True)
class Prop:
    """One prop's prior. Any field may be None -- unknown is not zero."""
    name: str
    x_m: Optional[float] = None          # pool frame, +x along the course
    y_m: Optional[float] = None
    depth_m: Optional[float] = None      # NEGATIVE below surface, this stack's sign
    bearing_deg: Optional[float] = None  # compass bearing its FACE points along
    detect_class: str = ''               # the class the detector emits for it
    width_m: Optional[float] = None      # what the detector BOXES, for metric range
    measured: bool = False               # false = rulebook nominal, not our pool
    note: str = ''

    @property
    def has_position(self) -> bool:
        return self.x_m is not None and self.y_m is not None

    @property
    def has_bearing(self) -> bool:
        return self.bearing_deg is not None


class Course:
    """A named set of prop priors, with refusals instead of defaults."""

    def __init__(self, name: str, props: Dict[str, Prop], source: str = '',
                 pool: Optional[Dict[str, Optional[float]]] = None):
        self.name = name
        self.props = props
        self.source = source
        # The venue's floor: `depth_m`, and `depth_min_m`/`depth_max_m` for a
        # floor that slopes or steps. None where unset, never a default.
        self.pool = dict(pool or {})

    def pool_depth(self):
        """(depth, min, max) in metres, min and max NaN when unset; None when
        the course gives no depth at all, so flow keeps what it was given."""
        d = self.pool.get('depth_m')
        if d is None:
            return None
        lo, hi = self.pool.get('depth_min_m'), self.pool.get('depth_max_m')
        nan = float('nan')
        return (float(d), nan if lo is None else float(lo),
                nan if hi is None else float(hi))

    def launch_values(self) -> Dict[str, str]:
        """{bringup argument: value} for every venue number this course sets."""
        out = {}
        for key, arg in VENUE_LAUNCH.items():
            v = self.pool.get(key)
            if v is None:
                continue
            out[arg] = ('true' if v else 'false') if isinstance(v, bool) else repr(float(v))
        return out

    def shield_max_depth_m(self) -> Optional[float]:
        """The depth the hull must never pass in this venue, or None."""
        return self.pool.get('shield_max_depth_m')

    def widths_by_class(self) -> Dict[str, float]:
        """{detector class: width_m} for every prop whose size was measured here."""
        return {(p.detect_class or p.name): p.width_m
                for p in self.props.values() if p.width_m is not None}

    def pool_floor(self):
        """Surveyed floor points [(x, y, depth), ...]; [] when not surveyed."""
        return list(self.pool.get('floor') or [])

    def prop(self, name: str) -> Prop:
        key = str(name).strip().lower()
        if key not in self.props:
            raise KeyError(
                f'{self.name!r} has no prop {name!r}. Known: '
                f'{sorted(self.props)}')
        return self.props[key]

    def position_of(self, name: str) -> tuple:
        """(x, y) or a raised error naming what is missing.

        Refusing is the whole point: a prop whose position was never measured
        would otherwise dead-reckon the vehicle to the pool's origin.
        """
        p = self.prop(name)
        if not p.has_position:
            raise ValueError(
                f'{self.name}/{name}: position is unset, so there is nothing to '
                f'dead-reckon to. Measure it and set x_m/y_m, or let the '
                f'mission search instead.')
        return float(p.x_m), float(p.y_m)

    def bearing_of(self, name: str) -> float:
        p = self.prop(name)
        if not p.has_bearing:
            raise ValueError(
                f'{self.name}/{name}: bearing_deg is unset, so it cannot anchor '
                f'a heading. A guessed bearing writes a wrong heading zero, '
                f'which is worse than having none.')
        return float(p.bearing_deg)

    def unmeasured(self) -> list:
        """Props carrying rulebook nominals rather than our own measurements."""
        return sorted(n for n, p in self.props.items() if not p.measured)


def _search_dirs() -> list:
    out = []
    env = os.environ.get(_ENV, '').strip()
    if env:
        out.append(pathlib.Path(env).expanduser())
    out.append(_HOME)
    out.append(_PKG_COURSES)
    return out


def load_course(name: str) -> Course:
    """First `<name>.yaml` found: env override, then ~/.mongla, then the package.

    Search order puts the deck FIRST on purpose. A course re-measured at the
    venue must win over the one committed months earlier, and it must do so
    without a rebuild.
    """
    stem = str(name).strip()
    for d in _search_dirs():
        path = d / f'{stem}.yaml'
        if not path.is_file():
            continue
        raw = yaml.safe_load(path.read_text()) or {}
        _refuse_unknown(raw, TOP_KEYS, path, 'the top level')
        _refuse_unknown(raw.get('pool') or {}, POOL_KEYS, path, 'pool')
        props = {}
        for key, body in (raw.get('props') or {}).items():
            body = body or {}
            _refuse_unknown(body, PROP_KEYS, path, f'props.{key}')
            measured = body.get('measured', False)
            if not isinstance(measured, bool):
                raise ValueError(
                    f'{path}: props.{key}.measured is {measured!r}; write true or '
                    f'false unquoted. A quoted "false" is a string, and a string '
                    f'reads as measured.')
            props[str(key).strip().lower()] = Prop(
                name=str(key).strip().lower(),
                x_m=_opt_float(body.get('x_m')),
                y_m=_opt_float(body.get('y_m')),
                depth_m=_opt_float(body.get('depth_m')),
                bearing_deg=_opt_float(body.get('bearing_deg')),
                detect_class=str(body.get('detect_class', '') or ''),
                measured=measured,
                note=str(body.get('note', '') or ''),
                width_m=_opt_float(body.get('width_m')))
        pool = {k: _opt_float((raw.get('pool') or {}).get(k))
                for k in ('depth_m', 'depth_min_m', 'depth_max_m')}
        rp = raw.get('pool') or {}
        pool['floor'] = _floor_points(rp.get('floor'), path)
        pool['tile_m'] = _opt_float(rp.get('tile_m'))
        pool['shield_max_depth_m'] = _opt_float(rp.get('shield_max_depth_m'))
        if 'lane_lines' in rp:
            if not isinstance(rp['lane_lines'], bool):
                raise ValueError(f'{path}: pool.lane_lines is {rp["lane_lines"]!r}; '
                                 f'write true or false unquoted')
            pool['lane_lines'] = rp['lane_lines']
        return Course(stem, props, source=str(path), pool=pool)
    raise FileNotFoundError(
        f'no course {stem!r}. Looked in: '
        f'{[str(d) for d in _search_dirs()]}')


def _floor_points(raw, path):
    """`pool: floor:` as [(x, y, depth), ...], depth positive; [] when unset.

    Surveyed points of the floor in the course's pool frame: a tape or a pole
    at a few places, more where it slopes. Anything that is not a list of
    three positive-depth numbers is refused, because a mis-typed survey is a
    wrong height on every velocity.
    """
    if raw is None:
        return []
    if not isinstance(raw, list):
        raise ValueError(f'{path}: pool.floor must be a list of [x_m, y_m, depth_m]')
    pts = []
    for i, row in enumerate(raw):
        if not (isinstance(row, (list, tuple)) and len(row) == 3):
            raise ValueError(f'{path}: pool.floor[{i}] is {row!r}; write '
                             f'[x_m, y_m, depth_m]')
        x, y, d = (float(v) for v in row)
        if not d > 0.0:
            raise ValueError(f'{path}: pool.floor[{i}] depth {d!r} must be positive '
                             f'metres (the floor is BELOW the surface)')
        pts.append((x, y, d))
    return pts


def _refuse_unknown(section, allowed, path, where) -> None:
    """Raise on a key the loader does not read, naming the nearest it does."""
    if not isinstance(section, dict):
        raise ValueError(f'{path}: {where} must be a mapping, got {section!r}')
    for k in section:
        if k not in allowed:
            near = difflib.get_close_matches(str(k), allowed, n=1)
            hint = f'; did you mean {near[0]!r}?' if near else ''
            raise ValueError(f'{path}: {where} has {k!r}, which nothing reads{hint} '
                             f'Known: {", ".join(allowed)}')


def _opt_float(v):
    """None stays None. An unset number must never become 0.0."""
    if v is None or v == '':
        return None
    f = float(v)
    if math.isnan(f):
        return None
    return f


def bearing_to(from_xy: tuple, to_xy: tuple) -> float:
    """Compass bearing from one pool point to another, in [0, 360).

    Pool frame convention, stated once: +x is along the course and is bearing
    0, +y is to the right of it and is bearing 90. Any other reading of these
    numbers puts the vehicle on a different heading with nothing logging a
    fault.
    """
    dx = float(to_xy[0]) - float(from_xy[0])
    dy = float(to_xy[1]) - float(from_xy[1])
    deg = math.degrees(math.atan2(dy, dx))
    return deg + 360.0 if deg < 0.0 else deg


def range_to(from_xy: tuple, to_xy: tuple) -> float:
    return math.hypot(float(to_xy[0]) - float(from_xy[0]),
                      float(to_xy[1]) - float(from_xy[1]))
