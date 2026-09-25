#!/usr/bin/env python3
"""Every package's unreachable modules, not just `mongla_vision`'s.

⛔ WHY THIS EXISTS. `test_no_capability_is_built_and_unreachable.py` is the
repo's guard against its oldest and most expensive defect -- a module that is
written, tested, and imported by nothing -- and it covers **one package**.
Five others had never been swept: `mongla_control` (1 283 tests),
`mongla_localization`, `mongla_planner`, `mongla_sensors`, `mongla_manager`.

Swept for the first time on 2026-09-25, it found four:

    mongla_control/geometric_allocation.py   336 lines
    mongla_control/hydrodynamics.py          294 lines
    mongla_control/nav_filter.py             138 lines
    mongla_manager/estimator/thrust_model.py 107 lines

⚠ AN ORPHAN IS NOT AUTOMATICALLY A DEFECT, and this tool does not pretend
otherwise. `thrust_model.py` is unreachable because **no non-zero ESC RPM has
ever crossed our wire** -- it waits on the thruster pack, says so in its own
docstring, and wiring it to a signal that is always 0 would be worse than
leaving it. That is a debt with a name, which is the standard CLAUDE.md §9
asks for. The defect is a module with NO stated reason, and the deeper defect
is nobody looking.

⭐ THE CLASS OF BUG THIS CATCHES is the one that cost this session a day: the
lock ladder never subscribed to `/mongla/state`, so vision did not know the
hull's heading and the object-permanence rung was unreachable for want of one
number the board already published. That is invisible to every test, because
each piece works perfectly alone.

⚠ Three exemptions, each a real reachability path an import grep cannot see:
ROS nodes are LAUNCHED not imported, `console_scripts` are invoked by name,
and `missions/` is loaded dynamically.

    python3 tools/orphan_sweep.py
    python3 tools/orphan_sweep.py --package mongla_control
"""
from __future__ import annotations

import argparse
import pathlib
import re
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
SRC = ROOT / 'src'


def entry_points() -> set:
    """`console_scripts` are invoked by NAME, so no import points at them."""
    out = set()
    for sp in SRC.rglob('setup.py'):
        try:
            txt = sp.read_text()
        except OSError:
            continue
        out |= {m.rsplit('.', 1)[-1]
                for m in re.findall(r'=\s*([a-z_0-9.]+):main', txt)}
    return out


def is_capability(path: pathlib.Path, rel: str, entries: set) -> bool:
    if '/test' in rel or path.name == '__init__.py':
        return False
    if '/missions/' in rel or '/launch/' in rel:
        return False
    if path.stem in entries:
        return False                       # invoked by name
    try:
        txt = path.read_text()
    except (OSError, UnicodeDecodeError):
        return False
    if 'def main(' in txt and 'rclpy' in txt:
        return False                       # a ROS node: launched, not imported
    return True


def importers(stem: str, exclude: pathlib.Path) -> list:
    """Files that IMPORT this module -- not ones that merely mention it.

    ⚠ A substring grep is useless: "approach" and "health" appear in prose all
    over this codebase and both read as wired when they were not.
    """
    pat = re.compile(
        rf'^\s*(from\s+\S*\b{stem}\b\s+import|'
        rf'from\s+\S+\s+import\s+[^\n]*\b{stem}\b|'
        rf'import\s+\S*\b{stem}\b)', re.M)
    hits = []
    for q in SRC.rglob('*.py'):
        if q == exclude or '/test' in str(q):
            continue
        try:
            if pat.search(q.read_text()):
                hits.append(str(q.relative_to(SRC)))
        except (OSError, UnicodeDecodeError):
            continue
    return hits


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--package', default='')
    ap.add_argument('--strict', action='store_true',
                    help='exit non-zero when any orphan is found')
    a = ap.parse_args()

    entries = entry_points()
    pkgs = ([a.package] if a.package
            else sorted(p.name for p in SRC.iterdir() if p.is_dir()))
    total = 0
    for pkg in pkgs:
        root = SRC / pkg / pkg
        if not root.is_dir():
            continue
        found = []
        for p in sorted(root.rglob('*.py')):
            rel = str(p.relative_to(SRC))
            if not is_capability(p, rel, entries):
                continue
            if not importers(p.stem, p):
                found.append((rel, sum(1 for _ in p.open('rb'))))
        if found:
            total += len(found)
            print(f'\n{pkg}: {len(found)} unreachable')
            for rel, n in found:
                print(f'   {n:5d} lines  {rel.split(pkg + "/", 1)[-1]}')

    if not total:
        print('no unreachable capability modules')
        return 0
    print(f'\n{total} unreachable module(s).\n'
          '⚠ An orphan is not automatically a defect -- some wait on hardware '
          'that does not exist yet, and say so in their own docstring. The '
          'defect is one with NO stated reason. Give each a named blocker or '
          'a consumer; there is no third state (CLAUDE.md §9).')
    return 1 if a.strict else 0


if __name__ == '__main__':
    sys.exit(main())
