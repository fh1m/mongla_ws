"""Where the two Mongla workspaces live — one implementation, three callers.

Mongla is **one repo, two colcon workspaces**::

    duburi_ws/        <- duburi_ws_root()   autonomy: manager, planner, vision
      src/duburi_manager/ ...
      sim/            <- sim_ws_root()      this workspace: Gazebo + SITL + lab
        src/duburi_sim_bridge/ ...

Until 2026-08-26 the sim was a *sibling* tree literally named ``duburi-sim_ws``
and three places guessed at that layout independently:
``record_cameras._workspace_root``, ``duburi_sim_web.server._workspace_root``,
and ``scripts/duburi_sim``. Two of the three matched on the directory *name*,
which no longer exists, and both fell through to ``Path.cwd()`` on a miss — so a
wrong answer wrote ``datasets/`` into whatever directory you happened to launch
from instead of failing. Silent-wrong is the failure mode this module exists to
delete: **every function here raises rather than guessing.**

``scripts/duburi_sim`` mirrors the same env → marker-walk → error chain in shell
(it cannot import Python before the overlay is sourced) and points back here.
"""
import os
import tempfile
from pathlib import Path
from typing import Optional

#: A directory is that workspace iff it contains this marker.
SIM_MARKER = Path('src') / 'duburi_sim_bridge'
AUTONOMY_MARKER = Path('src') / 'duburi_manager'

SIM_WS_ENV = 'DUBURI_SIM_WS'
DUBURI_WS_ENV = 'DUBURI_WS'


def _walk_up(start: Path, marker: Path) -> Optional[Path]:
    """First ancestor of `start` (inclusive) holding `marker`, else None."""
    for candidate in (start, *start.parents):
        if (candidate / marker).is_dir():
            return candidate
    return None


def _ament_share(package: str) -> Optional[Path]:
    try:
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory(package)).resolve()
    except Exception:
        return None


def sim_ws_root() -> Path:
    """Root of the simulator workspace (the directory holding ``src/``).

    Order: ``$DUBURI_SIM_WS`` → walk up from this file → walk up from the
    installed share dir. The middle step covers both the source tree and a
    ``sim/install/...`` overlay, because ``install/`` lives inside ``sim/``.
    """
    env = os.environ.get(SIM_WS_ENV)
    if env:
        root = Path(env).expanduser().resolve()
        if (root / SIM_MARKER).is_dir():
            return root
        raise RuntimeError(
            f'{SIM_WS_ENV}={env} does not contain {SIM_MARKER} — '
            f'point it at the directory holding sim/src/.'
        )

    found = _walk_up(Path(__file__).resolve().parent, SIM_MARKER)
    if found:
        return found

    share = _ament_share('duburi_sim_bridge')
    if share:
        found = _walk_up(share, SIM_MARKER)
        if found:
            return found

    raise RuntimeError(
        'cannot locate the simulator workspace: no ancestor of '
        f'{Path(__file__).resolve()} contains {SIM_MARKER}. '
        f'Set {SIM_WS_ENV}=<path to duburi_ws/sim>.'
    )


def duburi_ws_root() -> Path:
    """Root of the autonomy workspace (the directory holding ``src/duburi_manager``).

    In-repo this is simply ``sim_ws_root().parent``; the walk keeps a
    sibling checkout working too.
    """
    env = os.environ.get(DUBURI_WS_ENV)
    if env:
        root = Path(env).expanduser().resolve()
        if (root / AUTONOMY_MARKER).is_dir():
            return root
        raise RuntimeError(
            f'{DUBURI_WS_ENV}={env} does not contain {AUTONOMY_MARKER} — '
            f'point it at the duburi_ws checkout root.'
        )

    found = _walk_up(sim_ws_root(), AUTONOMY_MARKER)
    if found:
        return found

    raise RuntimeError(
        f'cannot locate the autonomy workspace above {sim_ws_root()}. '
        f'Set {DUBURI_WS_ENV}=<path to duburi_ws>.'
    )


def runtime_dir() -> Path:
    """Per-user scratch dir for the lab's side-channel files.

    The lab and the ``duburi_sim`` shell CLI agree on the active course, the lab
    port and three logs through files in ``/tmp``. They were unprefixed and
    world-writable: on a shared box any user could pre-create
    ``/tmp/duburi_lab_active_course.txt`` and the lab would either fail to write
    it or read someone else's course and restart Gazebo into the wrong world.

    ``/tmp/duburi-$USER/`` at mode 0700 fixes both. ``scripts/duburi_sim``
    hardcodes the same expression — change one, change both, or the CLI and the
    lab stop agreeing on the active course.
    """
    user = os.environ.get('USER') or os.environ.get('LOGNAME') or str(os.getuid())
    d = Path(tempfile.gettempdir()) / f'duburi-{user}'
    d.mkdir(mode=0o700, exist_ok=True)
    return d


def _self_check() -> None:
    sim = sim_ws_root()
    ws = duburi_ws_root()
    assert (sim / SIM_MARKER).is_dir(), sim
    assert (ws / AUTONOMY_MARKER).is_dir(), ws
    # The whole point of the absorption: the sim sits inside the autonomy repo.
    assert sim.parent == ws, f'expected {sim} directly under {ws}'
    rt = runtime_dir()
    assert rt.is_dir(), rt
    print(f'ok  sim_ws_root()={sim}\nok  duburi_ws_root()={ws}\nok  runtime_dir()={rt}')


if __name__ == '__main__':
    _self_check()
