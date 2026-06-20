"""Gate task — find gate, align to rescue/repair marker, pass through.

Standalone test (gate_rescue_repair.pt present):
    ros2 run duburi_planner mission gate_task

Called by full_mission_2026 combinator after arm + depth set.
"""

from duburi_planner.missions.competition_config import (
    GATE_PASS_DEPTH_M,
    GATE_PASS_BBOX_FRAC,
    SEARCH_FORWARD_GAIN,
    SEARCH_MAX_STEPS,
)

_FWD = '/duburi_detector_fwd'


def run(duburi, log=None):
    duburi.resume_detector('forward')
    duburi.set_model('gate_rescue_repair', node=_FWD)
    duburi.set_classes('gate,rescue,repair', node=_FWD)

    # ── Search for gate ────────────────────────────────────────────────────────
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
                log('[gate_task] gate not found in scan — skipping task')
            duburi.pause_detector('forward')
            return

    # ── Coarse align: centre on gate ──────────────────────────────────────────
    duburi.vision.home(
        target='gate', camera='forward',
        yaw=True, lat=True, on_lost='hold', duration=15)

    # ── Lateral slide: align sub with rescue/repair marker ────────────────────
    duburi.set_classes('rescue,repair', node=_FWD)
    duburi.vision.home(
        target='rescue', camera='forward',
        yaw=False, lat=True, on_lost='hold', duration=10)

    # ── Descend to pass depth, then re-filter for gate outline ────────────────
    duburi.set_depth(GATE_PASS_DEPTH_M, timeout=20)
    duburi.set_classes('gate', node=_FWD)

    # ── Drive through — exits when gate fills GATE_PASS_BBOX_FRAC of frame ────
    duburi.vision.approach(
        target='gate', camera='forward',
        dist=GATE_PASS_BBOX_FRAC, metric='height',
        on_lost='hold', duration=25, lock_mode='pursue')

    duburi.pause(2.0)
    duburi.pause_detector('forward')
