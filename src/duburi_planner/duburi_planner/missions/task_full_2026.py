"""Full RoboSub 2026 competition run: Gate → Slalom → Bin → Torpedo → Return+Roll.

Detected-paradigm combinator (flat, non-FSM). For the YASMIN FSM version:
    ros2 run duburi_planner mission fsm_full_2026

Launch (detected-paradigm):
    ros2 run duburi_planner mission task_full_2026

Chunks are loaded at run() time (not module import), so pool-day edits to
chunk files take effect without colcon build when launched from the source tree.

Fill competition_config.py before each run:
  SLALOM_HEADING_DEG, BIN_HEADING_DEG, TORPEDO_HEADING_DEG, RETURN_HEADING_DEG
"""

import importlib.util
from pathlib import Path

from duburi_planner.missions.competition_config import GATE_SEARCH_DEPTH_M
from duburi_planner.resilience import never_fails


def _chunk(name: str):
    """Load a sibling mission file by stem for hot-reload support."""
    py = Path(__file__).parent / f'{name}.py'
    spec = importlib.util.spec_from_file_location(
        f'duburi_planner.missions.{name}', py)
    if spec is None or spec.loader is None:
        raise ImportError(f"Cannot load chunk: {py}")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)   # type: ignore[union-attr]
    return mod


def run(duburi, log=None):
    duburi.mission_reset()   # clear heading lock + abort from any previous run
    # Load chunks here so pool-day edits take effect without colcon build.
    gate    = _chunk('task_gate')
    slalom  = _chunk('task_slalom')
    _bin    = _chunk('task_bin')
    torpedo = _chunk('task_torpedo')
    _return = _chunk('task_return')

    try:
        # 10-second window to remove tether before thrusters arm
        duburi.pause(10.0)
        duburi.arm()
        duburi.set_depth(GATE_SEARCH_DEPTH_M, timeout=30)
        duburi.lock_heading(0.0, timeout=600)   # BNO085 heading lock for full run

        for name, chunk in [
            ('gate',    gate),
            ('slalom',  slalom),
            ('bin',     _bin),
            ('torpedo', torpedo),
            ('return',  _return),
        ]:
            try:
                chunk.run(duburi)
            except Exception as exc:
                if log:
                    log(f'[MISSION] {name} FAILED: {exc} — continuing to next task')

    except Exception as exc:
        if log:
            log(f'[MISSION] ABORT: {exc}')
        raise
    finally:
        # ⛔ CLEANUP RAN AS ONE BLOCK, SO THE FIRST FAILURE ATE THE REST.
        # `release_heading()` raising meant `stop()` and `disarm()` never ran,
        # and the exception that surfaced was the cleanup's rather than the
        # mission's. Contain each step so one cannot cost the others.
        never_fails(duburi.release_heading, log=log, name='release_heading')
        never_fails(duburi.stop, log=log, name='stop')
        # ⛔ DISARM IS DELIBERATELY NOT CONTAINED. Every other step here is
        # tidying; this one is the safety path, and a failed disarm that
        # reported success is the single failure this stack cannot tolerate.
        # It raises, loudly, even from a finally.
        duburi.disarm()
