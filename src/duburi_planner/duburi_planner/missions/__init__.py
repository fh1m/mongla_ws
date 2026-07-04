"""Mission registry -- auto-discovers any module under this package.

Drop a file in this folder named `<my_mission>.py`, expose

    def run(duburi, log):
        ...

and it will show up in `ros2 run duburi_planner mission --list` on the
next process start. There is NO registry table to update; the mission's
file name *is* its CLI name.

Convention
----------
* Files starting with `_` (e.g. `_helpers.py`) are treated as private
  implementation modules and skipped.
* A file is a mission iff it exposes a top-level callable named `run`
  with signature `run(duburi, log)`.
* Mission name = file stem. So `missions/follow_gate.py` becomes
  `ros2 run duburi_planner mission follow_gate`.

Hot-reload (no colcon build needed)
------------------------------------
`discover()` first tries to load mission files directly from the source
tree. When you save a mission file and re-run the mission, the updated
source is loaded immediately -- no `colcon build` or `source` required.

  * Source tree is found automatically by splitting the installed
    `__file__` path on ``/install/``.
  * Override the source dir with ``DUBURI_MISSIONS_DIR=/path/to/missions``.
  * Falls back to the installed package when the source tree is absent
    (production / container deployments).

Note: hot-reload covers ONLY mission ``.py`` files. Changes to DSL code
(``duburi_dsl.py``, ``client.py``, etc.) still require a rebuild because
those are imported as regular packages, not loaded by path.

The runner ([mission.py](../mission.py)) calls `discover()` once at
startup and dispatches into the matching `run`.
"""

import importlib
import importlib.util
import os
import pkgutil
import sys
import traceback
from pathlib import Path
from typing import Callable, Dict


def _warn_bad_mission(name: str, exc: Exception) -> None:
    """Loud stderr warning for a mission file that failed to import.

    We SKIP it and keep the rest of the registry runnable -- on competition day
    one stale/half-edited file (e.g. a scratch `test.py`) must never brick
    `mission <the-good-one>`. The warning keeps the dev-time "author notices"
    property without the fatal "whole runner dies".
    """
    print(f'\033[33m[missions] SKIPPED {name!r}: {type(exc).__name__}: {exc}\033[0m',
          file=sys.stderr)
    # Full traceback at DEBUG-ish verbosity so the author can still fix it.
    if os.environ.get('DUBURI_MISSIONS_VERBOSE'):
        traceback.print_exc()


def _find_src_missions_dir() -> Path | None:
    """Locate the source-tree missions directory for hot-reload.

    Priority:
      1. ``DUBURI_MISSIONS_DIR`` environment variable (explicit override)
      2. Split the installed ``__file__`` path on ``/install/`` to
         recover the workspace root, then probe the expected source path.

    Returns ``None`` if the source tree cannot be found (installed-only
    environments or non-standard workspace layouts).
    """
    env = os.environ.get('DUBURI_MISSIONS_DIR')
    if env:
        p = Path(env)
        return p if p.is_dir() else None

    # Installed path looks like:
    #   <ws>/install/duburi_planner/lib/python3.x/site-packages/duburi_planner/missions/__init__.py
    this = Path(__file__).resolve()
    path_str = str(this)
    if '/install/' in path_str:
        ws_root = Path(path_str.split('/install/')[0])
        src_dir = (ws_root / 'src' / 'duburi_planner'
                   / 'duburi_planner' / 'missions')
        if src_dir.is_dir():
            return src_dir

    return None


def discover() -> Dict[str, Callable]:
    """Walk missions and return {mission_name: run_callable}.

    Skips private modules (``_*.py``) and modules that don't expose a
    callable ``run``. A module that FAILS to import is SKIPPED with a loud
    stderr warning (not fatal) so one broken/stale file can't brick the whole
    registry on competition day -- the other missions stay runnable. Set
    ``DUBURI_MISSIONS_VERBOSE=1`` for the full traceback.

    Loads from the source tree when found (hot-reload); falls back to
    the installed package otherwise.
    """
    src_dir = _find_src_missions_dir()

    if src_dir is not None:
        found: Dict[str, Callable] = {}
        for py_file in sorted(src_dir.glob('*.py')):
            name = py_file.stem
            if name.startswith('_'):
                continue
            try:
                spec = importlib.util.spec_from_file_location(
                    f'duburi_planner.missions.{name}', py_file)
                if spec is None or spec.loader is None:
                    continue
                module = importlib.util.module_from_spec(spec)
                spec.loader.exec_module(module)  # type: ignore[union-attr]
            except Exception as exc:   # noqa: BLE001 -- one bad file must not brick the registry
                _warn_bad_mission(name, exc)
                continue
            run = getattr(module, 'run', None)
            if callable(run):
                found[name] = run
        return found

    # Fallback: installed package (no hot-reload)
    found = {}
    for module_info in pkgutil.iter_modules(__path__):
        name = module_info.name
        if name.startswith('_'):
            continue
        try:
            module = importlib.import_module(f'{__name__}.{name}')
        except Exception as exc:   # noqa: BLE001 -- one bad file must not brick the registry
            _warn_bad_mission(name, exc)
            continue
        run = getattr(module, 'run', None)
        if callable(run):
            found[name] = run
    return found


__all__ = ['discover']
