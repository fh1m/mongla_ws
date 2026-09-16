"""``ros2 run duburi_localization course_survey`` -- write MEASURED prop positions.

The packaged course files ship with every position unset (a prior nobody
measured dead-reckons the vehicle somewhere wrong, confidently). This is how the
numbers get in at the venue: one prop per call, written to the DECK copy
(`$DUBURI_COURSE_DIR` or `~/.duburi/courses/<course>.yaml`), never the package.
The loader reads the deck copy first, so no rebuild is needed.

    course_survey --course sauvc26 --list
    course_survey --course sauvc26 --prop final_gate --x 16.0 --y 0.0 --bearing 0
    course_survey --course sauvc26 --prop flare_red --x 9.5 --y 2.1 --depth -0.8

The file is re-read through `load_course` after writing, so a malformed value is
caught here rather than in the water.
"""
from __future__ import annotations

import argparse
import os
import pathlib
import sys

import yaml

from duburi_localization import course_map as cm

_FIELDS = (('x', 'x_m'), ('y', 'y_m'), ('depth', 'depth_m'), ('bearing', 'bearing_deg'))


def deck_dir() -> pathlib.Path:
    env = os.environ.get(cm._ENV, '').strip()
    return pathlib.Path(env).expanduser() if env else cm._HOME


def survey(course: str, prop: str, values: dict, *, detect_class: str = '',
           out_dir: pathlib.Path | None = None) -> pathlib.Path:
    """Set measured fields on one prop in the deck copy; return the file written."""
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
    body['measured'] = True
    props[key] = body
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
    ap.add_argument('--depth', type=float, help='metres, NEGATIVE below the surface')
    ap.add_argument('--bearing', type=float, help='compass bearing the prop face points')
    ap.add_argument('--detect-class', default='')
    ap.add_argument('--list', action='store_true')
    a = ap.parse_args(argv)
    if a.depth is not None and a.depth > 0.0:
        print('depth must be NEGATIVE below the surface (this stack\'s sign)')
        return 2
    if a.prop:
        try:
            path = survey(a.course, a.prop,
                          {'x': a.x, 'y': a.y, 'depth': a.depth, 'bearing': a.bearing},
                          detect_class=a.detect_class)
        except (KeyError, FileNotFoundError, ValueError) as exc:
            print(f'refused: {exc}')
            return 2
        print(f'wrote {path}')
    course = cm.load_course(a.course)
    print(f'{course.name}  ({course.source})')
    for name, p in sorted(course.props.items()):
        pos = f'({p.x_m:+.2f}, {p.y_m:+.2f})' if p.has_position else 'unset'
        brg = f'{p.bearing_deg:.0f} deg' if p.has_bearing else '-'
        print(f'  {name:16s} {"MEASURED" if p.measured else "nominal ":8s} '
              f'pos {pos:18s} bearing {brg}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
