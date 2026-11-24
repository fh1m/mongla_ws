"""``ros2 run mongla_localization course_survey`` -- write MEASURED prop positions.

The packaged course files ship with every position unset (a prior nobody
measured dead-reckons the vehicle somewhere wrong, confidently). This is how the
numbers get in at the venue: one prop per call, written to the DECK copy
(`$MONGLA_COURSE_DIR` or `~/mongla/deck/courses/<course>.yaml`), never the package.
The loader reads the deck copy first, so no rebuild is needed.

    course_survey --course sauvc26 --list
    course_survey --course sauvc26 --prop final_gate --x 16.0 --y 0.0 --bearing 0
    course_survey --course sauvc26 --prop flare_red --x 9.5 --y 2.1 --depth -0.8
    course_survey --course sauvc26 --prop final_gate --width 1.50
    course_survey --course sauvc26 --floor 0 0 1.28 --floor 17.5 0 2.0 --shield 1.5
    course_survey --course sauvc26 --tile 0.25
    course_survey --course pool --corner 0 -3 --corner 0 3 --corner 25 3 --corner 25 -3

`--width` is what the detector BOXES, measured on the real prop: the pose path
ranges on it. `--floor X Y DEPTH` adds one floor sounding (depth POSITIVE,
metres below the surface) and may be repeated; flow's height follows them.
`--shield` is the depth the hull must never pass here. `--corner X Y`, given
in order round the pool, replaces its outline: Batighor's map draws it, and
nothing steers by it.

The CLI re-reads the file through `load_course` after writing, so a malformed
value is caught here rather than in the water.
"""
from __future__ import annotations

import argparse
import os
import pathlib
import sys

import yaml

from mongla_localization import course_map as cm

_FIELDS = (('x', 'x_m'), ('y', 'y_m'), ('depth', 'depth_m'), ('bearing', 'bearing_deg'),
           ('width', 'width_m'))


def deck_dir() -> pathlib.Path:
    env = os.environ.get(cm._ENV, '').strip()
    # The loader's first place after the env var, or "wrote" names a file a
    # deck copy shadows (vl_1 #2).
    return pathlib.Path(env).expanduser() if env else cm._deck()


def survey(course: str, prop: str, values: dict, *, detect_class: str = '',
           out_dir: pathlib.Path | None = None, measured: bool = True,
           note: str = '') -> pathlib.Path:
    """Set fields on one prop in the deck copy; return the file written.

    `measured` says where the numbers came from. True (the CLI) is a tape
    survey: from then on a vision placement only checks it. False is a
    vision placement (`place_prop(commit=True)`), which a later placement may
    refine. Writing a placement as measured froze one pose's drift into the
    course for good, because nothing overwrites a measured prop.
    """
    out_dir = pathlib.Path(out_dir) if out_dir else deck_dir()
    out = out_dir / f'{course}.yaml'
    if out.is_file():
        raw = yaml.safe_load(out.read_text()) or {}
    else:
        src = cm._PKG_COURSES / f'{course}.yaml'
        if not src.is_file():
            raise FileNotFoundError(f'no packaged course {course!r} to start from')
        raw = yaml.safe_load(src.read_text()) or {}
    props = raw.setdefault('props', {}) or {}
    raw['props'] = props
    key = str(prop).strip().lower()
    if key not in props and not detect_class:
        raise KeyError(f'{course!r} has no prop {prop!r} (known: {sorted(props)}); '
                       f'pass --detect-class to add one')
    body = props.get(key) or {}
    for arg, field in _FIELDS:
        v = values.get(arg)
        if v is not None:
            body[field] = float(v)
    if detect_class:
        body['detect_class'] = detect_class
    body['measured'] = bool(measured)
    if note:
        body['note'] = str(note)
    props[key] = body
    out_dir.mkdir(parents=True, exist_ok=True)
    tmp = out.with_suffix('.yaml.tmp')
    tmp.write_text('# Written by course_survey: MEASURED at the venue. The packaged\n'
                   '# copy is the unmeasured template; this deck copy wins.\n'
                   + yaml.safe_dump(raw, sort_keys=False))
    os.replace(tmp, out)
    return out


def survey_pool(course: str, *, floor=(), tile_m=None, shield_m=None,
                lane_lines=None, corners=(), out_dir: pathlib.Path | None = None) -> pathlib.Path:
    """Add floor soundings and set the venue numbers under `pool:`; return the file."""
    out_dir = pathlib.Path(out_dir) if out_dir else deck_dir()
    out = out_dir / f'{course}.yaml'
    src = out if out.is_file() else cm._PKG_COURSES / f'{course}.yaml'
    if not src.is_file():
        raise FileNotFoundError(f'no packaged course {course!r} to start from')
    raw = yaml.safe_load(src.read_text()) or {}
    pool = raw.get('pool') or {}
    pts = list(pool.get('floor') or [])
    for x, y, d in floor:
        if not d > 0.0:
            raise ValueError(f'floor depth {d} must be POSITIVE metres below the surface')
        pts.append([float(x), float(y), float(d)])
    if pts:
        pool['floor'] = pts
    for key, v in (('tile_m', tile_m), ('shield_max_depth_m', shield_m)):
        if v is not None:
            if not v > 0.0:
                raise ValueError(f'{key} must be positive, got {v}')
            pool[key] = float(v)
    if lane_lines is not None:
        pool['lane_lines'] = bool(lane_lines)
    if corners:
        if len(corners) < 3:
            raise ValueError(f'an outline needs three corners or more, got {len(corners)}')
        pool['outline'] = [[float(x), float(y)] for x, y in corners]
    raw['pool'] = pool
    out_dir.mkdir(parents=True, exist_ok=True)
    tmp = out.with_suffix('.yaml.tmp')
    tmp.write_text('# Written by course_survey: MEASURED at the venue. The packaged\n'
                   '# copy is the unmeasured template; this deck copy wins.\n'
                   + yaml.safe_dump(raw, sort_keys=False))
    os.replace(tmp, out)
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--course', required=True)
    ap.add_argument('--prop')
    ap.add_argument('--x', type=float)
    ap.add_argument('--y', type=float)
    ap.add_argument('--depth', type=float, help='metres, NEGATIVE below the surface; shown on the Batighor map, '
                    'not read by a mission')
    ap.add_argument('--bearing', type=float, help='compass bearing the prop face points')
    ap.add_argument('--detect-class', default='')
    ap.add_argument('--width', type=float,
                    help='metres: what the detector boxes, measured on the real prop')
    ap.add_argument('--floor', type=float, nargs=3, action='append', default=[],
                    metavar=('X', 'Y', 'DEPTH'),
                    help='a floor sounding in the pool frame, depth POSITIVE; repeatable')
    ap.add_argument('--tile', type=float, help='floor tile pitch, metres')
    ap.add_argument('--shield', type=float,
                    help='the depth (positive metres) the hull must never pass')
    ap.add_argument('--lane-lines', choices=('true', 'false'))
    ap.add_argument('--corner', type=float, nargs=2, action='append', default=[],
                    metavar=('X', 'Y'),
                    help='a corner of the pool, in order round it; together they replace the outline')
    ap.add_argument('--list', action='store_true')
    a = ap.parse_args(argv)
    if a.depth is not None and a.depth > 0.0:
        print('depth must be NEGATIVE below the surface (this stack\'s sign)')
        return 2
    if a.prop:
        try:
            path = survey(a.course, a.prop,
                          {'x': a.x, 'y': a.y, 'depth': a.depth, 'bearing': a.bearing,
                           'width': a.width},
                          detect_class=a.detect_class)
        except (KeyError, FileNotFoundError, ValueError) as exc:
            print(f'refused: {exc}')
            return 2
        print(f'wrote {path}')
    if a.floor or a.tile is not None or a.shield is not None or a.lane_lines or a.corner:
        try:
            path = survey_pool(a.course, floor=a.floor, tile_m=a.tile, shield_m=a.shield,
                               lane_lines=None if a.lane_lines is None
                               else a.lane_lines == 'true', corners=a.corner)
        except (FileNotFoundError, ValueError) as exc:
            print(f'refused: {exc}')
            return 2
        print(f'wrote {path}')
    course = cm.load_course(a.course)
    pool = course.pool
    floor = pool.get('floor') or []
    print(f'pool: depth {pool.get("depth_m")}, floor soundings {len(floor)}, '
          f'outline corners {len(pool.get("outline") or [])}, '
          f'tile {pool.get("tile_m")}, lane lines {pool.get("lane_lines")}, '
          f'depth floor {pool.get("shield_max_depth_m")}')
    print(f'{course.name}  ({course.source})')
    for name, p in sorted(course.props.items()):
        pos = f'({p.x_m:+.2f}, {p.y_m:+.2f})' if p.has_position else 'unset'
        brg = f'{p.bearing_deg:.0f} deg' if p.has_bearing else '-'
        wid = f'{p.width_m:.3f} m' if p.width_m is not None else '-'
        print(f'  {name:16s} {"MEASURED" if p.measured else "nominal ":8s} '
              f'pos {pos:18s} bearing {brg:8s} width {wid}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
