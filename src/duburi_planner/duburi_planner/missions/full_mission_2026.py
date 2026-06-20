"""Full RoboSub 2026 competition run: Gate → Slalom → Bin → Torpedo → Return+Roll.

Launch:
    ros2 launch duburi_vision full_mission.launch.py
    ros2 run duburi_planner mission full_mission_2026

Chunks are loaded at run() time (not module import), so pool-day edits to
chunk files take effect without colcon build when launched from the source tree.

Fill competition_config.py before each run:
  SLALOM_HEADING_DEG, BIN_HEADING_DEG, TORPEDO_HEADING_DEG, RETURN_HEADING_DEG
"""

import importlib.util
from pathlib import Path

from duburi_planner.missions.competition_config import GATE_SEARCH_DEPTH_M


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
    # Load chunks here so pool-day edits take effect without colcon build.
    gate    = _chunk('gate_task')
    slalom  = _chunk('slalom_task')
    _bin    = _chunk('bin_task')
    torpedo = _chunk('torpedo_task')
    _return = _chunk('return_task')

    try:
        # 10-second window to remove tether before thrusters arm
        duburi.pause(10.0)
        duburi.arm()
        duburi.set_depth(GATE_SEARCH_DEPTH_M, timeout=30)
        duburi.lock_heading(target=0.0, timeout=600)   # BNO085 heading lock for full run

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
        duburi.unlock_heading()
        duburi.stop()
        duburi.disarm()
