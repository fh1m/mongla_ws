"""Return task — find gate again, pass back through, style roll.

Standalone test (gate_rescue_repair.pt present):
    ros2 run duburi_planner mission return_task

Called by full_mission_2026 combinator after torpedo_task.
"""

from duburi_planner.missions.competition_config import (
    RETURN_HEADING_DEG,
    GATE_PASS_DEPTH_M,
    GATE_SEARCH_DEPTH_M,
    GATE_PASS_BBOX_FRAC,
    STYLE_ROLL_HEADROOM_M,
    STYLE_ROLL_GAIN,
    SEARCH_FORWARD_GAIN,
    SEARCH_MAX_STEPS,
)

_FWD = '/duburi_detector_fwd'


def run(duburi, log=None):
    if RETURN_HEADING_DEG is not None:
        duburi.turn(RETURN_HEADING_DEG)

    duburi.resume_detector('forward')
    duburi.set_model('gate_rescue_repair', node=_FWD)
    duburi.set_classes('gate', node=_FWD)

    # ── Search for return gate ────────────────────────────────────────────────
    for _ in range(SEARCH_MAX_STEPS):
        if duburi.detected('gate', stale_after=1.0):
            break
        duburi.move_forward(duration=0.5, gain=SEARCH_FORWARD_GAIN)
    else:
        result = duburi.vision.scan(
            target='gate', camera='forward',
            step=20, dwell=1.5, speed=40, duration=60)
        if not result.success:
            if log:
                log('[return_task] gate not found in scan — skipping task')
            duburi.pause_detector('forward')
            return

    # ── Centre on gate, descend, pass through ─────────────────────────────────
    duburi.vision.home(
        target='gate', camera='forward',
        yaw=True, lat=True, on_lost='hold', duration=15)
    duburi.set_depth(GATE_PASS_DEPTH_M, timeout=20)
    duburi.vision.approach(
        target='gate', camera='forward',
        dist=GATE_PASS_BBOX_FRAC, metric='height',
        on_lost='hold', duration=25, lock_mode='pursue')

    duburi.pause_detector('forward')
    duburi.pause(1.0)

    # ── Surface, then style roll ───────────────────────────────────────────────
    duburi.set_depth(GATE_SEARCH_DEPTH_M, timeout=20)
    duburi.style_roll(flips=1, headroom=STYLE_ROLL_HEADROOM_M, gain=STYLE_ROLL_GAIN)
