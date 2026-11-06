#!/usr/bin/env python3
"""Every package's unreachable modules, not just `mongla_vision`'s.

⛔ WHY THIS EXISTS. `test_no_capability_is_built_and_unreachable.py` is the
repo's guard against its oldest and most expensive defect -- a module that is
written, tested, and imported by nothing -- and it covers **one package**.
Five others had never been swept: `mongla_control` (1 283 tests),
`mongla_localization`, `mongla_planner`, `mongla_manager`.

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
import ast
import functools
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


def module_name(path: pathlib.Path) -> str:
    """`src/mongla_vision/mongla_vision/anchor/draw.py` -> `mongla_vision.anchor.draw`.

    The outer directory is the colcon package, the inner one the Python package;
    the importable name starts at the inner one. Returns '' outside `src/`.
    """
    try:
        parts = path.resolve().relative_to(SRC.resolve()).with_suffix('').parts
    except ValueError:
        return ''
    parts = parts[1:]
    if parts and parts[-1] == '__init__':
        parts = parts[:-1]
    return '.'.join(parts)


def imported_modules(path: pathlib.Path) -> frozenset:
    """Cached on (path, mtime): a guard asks this of every file once per module."""
    p = path.resolve()
    try:
        key = (p, p.stat().st_mtime_ns)
    except OSError:
        return frozenset()
    return _imported_modules(*key)


@functools.lru_cache(maxsize=None)
def _imported_modules(path: pathlib.Path, _mtime: int) -> frozenset:
    """Every dotted module name this file's import statements could name.

    Read with `ast`, and relative imports are resolved against this file's own
    package. `from .x import y` yields both `pkg.x` and `pkg.x.y`, because `y`
    may be a submodule or a name inside `x`; over-reporting a candidate costs
    nothing when the question is "does anything import THIS module".
    """
    try:
        tree = ast.parse(path.read_text())
    except (OSError, SyntaxError, UnicodeDecodeError):
        return frozenset()
    own = module_name(path).split('.') if module_name(path) else []
    pkg = own if path.name == '__init__.py' else own[:-1]
    out = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            out |= {al.name for al in node.names}
        elif isinstance(node, ast.ImportFrom):
            if node.level:
                if not pkg or node.level - 1 > len(pkg):
                    continue
                base = pkg[:len(pkg) - (node.level - 1)]
            else:
                base = []
            mod = base + (node.module.split('.') if node.module else [])
            if mod:
                out.add('.'.join(mod))
            out |= {'.'.join(mod + [al.name]) for al in node.names}
    return frozenset(out)


def importers(target: pathlib.Path, *, tools: bool = True) -> list:
    """Files that IMPORT this exact module -- not ones that mention it, and not
    a different module that happens to share its file name.

    ⛔ THE STEM MATCH THIS REPLACES HID A WHOLE MODULE. It matched on the file
    name, and `mongla_vision` has two `draw.py`: the HUD one is imported, so
    `anchor/draw.py` -- 143 lines nothing on the vehicle ever ran -- read as
    imported for its whole life (removed 2026-11-06). Resolving the dotted name is the only answer.

    ⛔ `tools/` IS SCANNED TOO by default. Scanning only `src/` reported
    `draw_strip.py` as unreachable, and `tools/render_real_stills.py` imports it
    to render the site's HUD imagery. A sweep that recommends deleting live
    code is worse than no sweep. Guards about the VEHICLE pass `tools=False`.
    """
    want = module_name(target)
    if not want:
        return []
    me = target.resolve()
    return [rel for q, rel, mods in _index(tools) if q != me and want in mods]


@functools.lru_cache(maxsize=None)
def _index(tools: bool) -> tuple:
    """(resolved path, path relative to ROOT, modules it imports) per non-test file.
    Built once per process: the tree does not change while a sweep runs."""
    files = list(SRC.rglob('*.py'))
    if tools:
        files += list((ROOT / 'tools').rglob('*.py'))
    out = []
    for q in files:
        rel = str(q.relative_to(ROOT))
        if '/test' in rel:
            continue
        out.append((q.resolve(), rel, imported_modules(q)))
    return tuple(out)


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
            if not importers(p):
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
