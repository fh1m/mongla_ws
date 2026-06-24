"""Slalom task — weave through red pipes.

Two-verb vision: align() holds a signed lateral pixel offset so the AUV
sits beside each pipe, then move() closes in on bbox HEIGHT (pipes are
tall+thin, so height is the right fill metric). A sweep fallback hunts
for the next pipe between passes.

Standalone test (slalom_red_pipe.pt — classes: red_pipe):
    ros2 run duburi_planner mission task_slalom

Pool-day: set SLALOM_HEADING_DEG and SLALOM_PIPE_OFFSET_PX in competition_config.py.
"""

from duburi_planner.missions.competition_config import (
    SLALOM_HEADING_DEG,
    SLALOM_PIPE_OFFSET_PX,
    SLALOM_FWD_FILL,
    ALIGN_ERR_PX,
    ALIGN_GAIN,
    APPROACH_GAIN,
    SEARCH_FORWARD_GAIN,
    SEARCH_CREEP_S,
    SEARCH_YAW_STEP_DEG,
)


def run(duburi, log=None):
    duburi.mission_reset()   # clear heading lock + abort from any previous run
    if SLALOM_HEADING_DEG is not None:
        duburi.turn(SLALOM_HEADING_DEG)

    duburi.resume_detector('forward')
    duburi.set_model('slalom_red_pipe', node='/duburi_detector_fwd')
    duburi.set_classes('red_pipe', node='/duburi_detector_fwd')

    # ── Weave: sit beside each pipe (signed lateral offset), then close in ────
    # ponytail: simple two-pass (right of pipe, then left). Add passes if needed.
    for sign in (+1, -1):
        duburi.vision.align(
            'red_pipe', camera='forward',
            yaw=0, lat=sign * SLALOM_PIPE_OFFSET_PX,
            err=ALIGN_ERR_PX, gain=ALIGN_GAIN, duration=25,
            fallback=sweep_for_pipe)
        duburi.vision.move(
            'red_pipe', camera='forward',
            fwd=SLALOM_FWD_FILL, mode='height',
            maintain=sign * SLALOM_PIPE_OFFSET_PX,
            gain=APPROACH_GAIN, duration=15,
            fallback=creep_forward)
        duburi.pause(0.5)

    duburi.pause_detector('forward')


# ── Mission-authored fallback search patterns (pure control) ────────────────────
def creep_forward(duburi):
    """One short forward creep, then return so the vision loop retries."""
    duburi.move_forward(SEARCH_CREEP_S, gain=SEARCH_FORWARD_GAIN)


def sweep_for_pipe(duburi, should_stop):
    """Yaw-sweep left/right looking for the next pipe; bail when it reappears."""
    heading = duburi.head()
    for step in (SEARCH_YAW_STEP_DEG, -2 * SEARCH_YAW_STEP_DEG,
                 2 * SEARCH_YAW_STEP_DEG):
        duburi.turn(heading + step)
        if should_stop():
            duburi.turn(heading)
            return
    duburi.turn(heading)
