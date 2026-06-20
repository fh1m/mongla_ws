"""Slalom task — weave through red pipes.

Standalone test (slalom_red_pipe.pt required — logic tested with yolo11n placeholder):
    ros2 run duburi_planner mission slalom_task

Called by full_mission_2026 combinator after gate_task.
Expects heading lock active from combinator; only needs lateral + yaw corrections.

Pool-day: set SLALOM_HEADING_DEG and SLALOM_PIPE_OFFSET_PX in competition_config.py.
"""

from duburi_planner.missions.competition_config import (
    SLALOM_HEADING_DEG,
    SLALOM_PIPE_OFFSET_PX,
    SEARCH_FORWARD_GAIN,
    SEARCH_MAX_STEPS,
)


def run(duburi, log=None):
    if SLALOM_HEADING_DEG is not None:
        duburi.turn(SLALOM_HEADING_DEG)

    duburi.resume_detector('forward')
    duburi.set_model('slalom_red_pipe', node='/duburi_detector_fwd')
    duburi.set_classes('red_pipe', node='/duburi_detector_fwd')

    # ── Search for first pipe ─────────────────────────────────────────────────
    for _ in range(SEARCH_MAX_STEPS):
        if duburi.detected('red_pipe', stale_after=1.0):
            break
        duburi.move_forward(duration=0.5, gain=SEARCH_FORWARD_GAIN)
    else:
        result = duburi.vision.scan(
            target='red_pipe', camera='forward',
            step=15, dwell=1.5, speed=40, duration=60)
        if not result.success:
            if log:
                log('[slalom_task] red_pipe not found in scan — skipping task')
            duburi.pause_detector('forward')
            return

    # ── Slalom: approach with lateral offset, flip side per pipe ──────────────
    # ponytail: simple two-pass (left then right). Add more pipes if needed.
    for sign in (+1, -1):
        duburi.vision.home(
            target='red_pipe', camera='forward',
            yaw=True, lat=True,
            offset_x=sign * SLALOM_PIPE_OFFSET_PX,
            forward=True, on_lost='hold', duration=25)
        duburi.pause(0.5)

    duburi.pause_detector('forward')
