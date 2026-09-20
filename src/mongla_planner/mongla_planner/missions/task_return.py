"""Return task — find gate again, pass back through, style roll.

Two-verb vision: align() re-centres on the gate (creeping to find it),
move() drives back through on bbox-height fill, then a style roll.

Standalone test (gate_rescue_repair.pt present):
    ros2 run mongla_planner mission task_return

Called by full_mission_2026 combinator after torpedo_task.
"""

from mongla_planner.missions.competition_config import (
    RETURN_HEADING_DEG,
    GATE_PASS_DEPTH_M,
    GATE_SEARCH_DEPTH_M,
    GATE_PASS_FWD_FILL,
    ALIGN_ERR_PX,
    ALIGN_GAIN,
    APPROACH_GAIN,
    STYLE_ROLL_HEADROOM_M,
    STYLE_ROLL_GAIN,
    SEARCH_FORWARD_GAIN,
    SEARCH_CREEP_S,
)

_FWD = '/mongla_detector_forward'


def run(mongla, log=None):
    mongla.mission_reset()
    if RETURN_HEADING_DEG is not None:
        mongla.turn(RETURN_HEADING_DEG)

    mongla.resume_detector('forward')
    mongla.set_model('gate_rescue_repair', node=_FWD)
    mongla.set_classes('gate', node=_FWD)

    # ── Centre on the return gate, descend, drive through ─────────────────────
    mongla.vision.align(
        'gate', camera='forward', yaw=0, lat=0,
        err=ALIGN_ERR_PX, gain=ALIGN_GAIN, duration=15,
        fallback=creep_forward)
    mongla.set_depth(GATE_PASS_DEPTH_M, timeout=20)
    mongla.vision.move(
        'gate', camera='forward',
        fwd=GATE_PASS_FWD_FILL, mode='height',
        gain=APPROACH_GAIN, duration=25,
        fallback=creep_forward)

    mongla.pause_detector('forward')
    mongla.pause(1.0)

    # ── Surface, then style roll ───────────────────────────────────────────────
    mongla.set_depth(GATE_SEARCH_DEPTH_M, timeout=20)
    mongla.style_roll(flips=1, headroom=STYLE_ROLL_HEADROOM_M, gain=STYLE_ROLL_GAIN)


# ── Mission-authored fallback search patterns (pure control) ────────────────────
def creep_forward(mongla):
    """One short forward creep, then return so the vision loop retries."""
    mongla.move_forward(SEARCH_CREEP_S, gain=SEARCH_FORWARD_GAIN)
