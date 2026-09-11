"""Where the props are, per course. The prior half of perceive-then-move.

⛔ WHAT THIS IS FOR. A mission that searches for every prop from scratch spends
its run turning. BumblebeeAS do not: their mothers dead-reckon to roughly where
a prop should be and let perception take over for the last few metres -- and on
the task where perception was hardest, the slalom, they shipped plain waypoints
and never ran the 3,000-line particle filter they had built. A prior map is the
cheapest autonomy in the sport.

⛔ AND WHAT MAKES IT SAFE HERE. Every number in a course file is a claim about a
pool we may not have measured. So a course entry is INCOMPLETE BY DEFAULT: the
loader refuses to answer for a prop whose position was never set, rather than
handing back a zero that a mission would drive to. `measured: false` is a
statement that the value is a rulebook nominal, not our pool -- our props are
not the venue's, and a range or a bearing off by a metre is a mission that
arrives somewhere else with total confidence.

⛔ ONE NUMBER, ONE PLACE. A prop's `bearing_deg` -- the compass direction its
face points -- is what `duburi.anchor_heading()` needs to turn a landmark into
an absolute heading, AND what a waypoint needs to approach it from the front.
Writing it twice is how the two come to disagree, so the course file is the
single source and the DSL reads the anchor bearing from here.

Pure loading and validation. No ROS.
"""
from __future__ import annotations

import math
import os
import pathlib
from dataclasses import dataclass
from typing import Dict, Optional

import yaml

_PKG_COURSES = pathlib.Path(__file__).resolve().parent / 'courses'

# Deck-side override, same contract as the geometry table: our pool is not the
# venue, and a course must be re-measurable without a colcon build.
_ENV = 'DUBURI_COURSE_DIR'
_HOME = pathlib.Path.home() / '.duburi' / 'courses'


@dataclass(frozen=True)
class Prop:
    """One prop's prior. Any field may be None -- unknown is not zero."""
    name: str
    x_m: Optional[float] = None          # pool frame, +x along the course
    y_m: Optional[float] = None
    depth_m: Optional[float] = None      # NEGATIVE below surface, this stack's sign
    bearing_deg: Optional[float] = None  # compass bearing its FACE points along
    detect_class: str = ''               # the class the detector emits for it
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

    def __init__(self, name: str, props: Dict[str, Prop], source: str = ''):
        self.name = name
        self.props = props
        self.source = source

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
    """First `<name>.yaml` found: env override, then ~/.duburi, then the package.

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
        props = {}
        for key, body in (raw.get('props') or {}).items():
            body = body or {}
            props[str(key).strip().lower()] = Prop(
                name=str(key).strip().lower(),
                x_m=_opt_float(body.get('x_m')),
                y_m=_opt_float(body.get('y_m')),
                depth_m=_opt_float(body.get('depth_m')),
                bearing_deg=_opt_float(body.get('bearing_deg')),
                detect_class=str(body.get('detect_class', '') or ''),
                measured=bool(body.get('measured', False)),
                note=str(body.get('note', '') or ''))
        return Course(stem, props, source=str(path))
    raise FileNotFoundError(
        f'no course {stem!r}. Looked in: '
        f'{[str(d) for d in _search_dirs()]}')


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
