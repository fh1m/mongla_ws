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

The runner ([mission.py](../mission.py)) calls `discover()` once at
startup and dispatches into the matching `run`.
"""

import importlib
import pkgutil
from typing import Callable, Dict


def discover() -> Dict[str, Callable]:
    """Walk this package and return {mission_name: run_callable}.

    Skips private modules (`_*.py`) and modules that don't expose a
    callable `run`. Import errors propagate -- a broken mission file
    should fail loudly so authors notice immediately.
    """
    found: Dict[str, Callable] = {}
    for module_info in pkgutil.iter_modules(__path__):
        name = module_info.name
        if name.startswith('_'):
            continue
        module = importlib.import_module(f'{__name__}.{name}')
        run = getattr(module, 'run', None)
        if callable(run):
            found[name] = run
    return found


__all__ = ['discover']
