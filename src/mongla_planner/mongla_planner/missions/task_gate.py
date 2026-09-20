"""Gate task — find gate, align to rescue/repair marker, pass through.

Two-verb vision: align() centres (creeping forward via its fallback until
the gate appears), then move() drives through on bbox-height fill. No
command aborts the mission — a miss just logs and the run continues.

Standalone test (gate_rescue_repair.pt present):
    ros2 run mongla_planner mission task_gate

Called by full_mission_2026 combinator after arm + depth set.
"""

from mongla_planner.missions.competition_config import (
    GATE_PASS_DEPTH_M,
    GATE_PASS_FWD_FILL,
    ALIGN_ERR_PX,
    ALIGN_GAIN,
    APPROACH_GAIN,
    SEARCH_FORWARD_GAIN,
    SEARCH_CREEP_S,
)

_FWD = '/mongla_detector_forward'


def run(mongla, log=None):
    mongla.mission_reset()   # clear heading lock + abort from any previous run
    mongla.resume_detector('forward')
    mongla.set_model('gate_rescue_repair', node=_FWD)
    mongla.set_classes('gate,rescue,repair', node=_FWD)

    # ── Centre on gate (yaw+lat); creep forward to find it if not yet seen ────
    mongla.vision.align(
        'gate', camera='forward', yaw=0, lat=0,
        err=ALIGN_ERR_PX, gain=ALIGN_GAIN, duration=30,
        fallback=creep_forward)

    # ── Slide onto the rescue/repair marker (heading fixed) ───────────────────
    #     fallback so a marker flicker re-searches (creep) instead of silently
    #     LOSTing and sliding past -- the rescue/repair side is a scoring choice.
    mongla.set_classes('rescue,repair', node=_FWD)
    mongla.vision.align(
        'rescue', camera='forward', lat=0,
        err=ALIGN_ERR_PX, gain=ALIGN_GAIN, duration=10,
        fallback=creep_forward)

    # ── Descend to pass depth, re-filter for the gate outline, drive through ──
    mongla.set_depth(GATE_PASS_DEPTH_M, timeout=20)
    mongla.set_classes('gate', node=_FWD)
    mongla.vision.move(
        'gate', camera='forward',
        fwd=GATE_PASS_FWD_FILL, mode='height',
        gain=APPROACH_GAIN, duration=25,
        fallback=creep_forward)

    mongla.pause(2.0)
    mongla.pause_detector('forward')


# ── Mission-authored fallback search patterns (pure control) ────────────────────
def creep_forward(mongla):
    """One short forward creep, then return so the vision loop retries."""
    mongla.move_forward(SEARCH_CREEP_S, gain=SEARCH_FORWARD_GAIN)
