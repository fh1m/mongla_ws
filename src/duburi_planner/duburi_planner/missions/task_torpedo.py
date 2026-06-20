"""Torpedo task — align to blood marker then lock and fire at hole.

Standalone test (torpedo_blood_hole.pt — classes: torpedo(0) blood(1) hole(2)):
    ros2 run duburi_planner mission task_torpedo

Called by full_mission_2026 combinator after bin_task.
fire_channel=1 → torpedo_1 (ESP32 channel 1). Always explicit.
"""

from duburi_planner.missions.competition_config import (
    TORPEDO_HEADING_DEG,
    TORPEDO_DEPTH_M,
    SEARCH_FORWARD_GAIN,
    SEARCH_MAX_STEPS,
)

_FWD = '/duburi_detector_fwd'


def run(duburi, log=None):
    assert TORPEDO_DEPTH_M is not None, (
        'TORPEDO_DEPTH_M not set — edit competition_config.py before pool day')

    if TORPEDO_HEADING_DEG is not None:
        duburi.turn(TORPEDO_HEADING_DEG)
    duburi.set_depth(TORPEDO_DEPTH_M, timeout=30)

    duburi.resume_detector('forward')
    duburi.set_model('torpedo_blood_hole', node=_FWD)
    duburi.set_classes('torpedo', node=_FWD)

    # ── Search for torpedo board ───────────────────────────────────────────────
    for _ in range(SEARCH_MAX_STEPS):
        if duburi.detected('torpedo', stale_after=1.0):
            break
        duburi.move_forward(0.5, gain=SEARCH_FORWARD_GAIN)
    else:
        result = duburi.vision.scan(
            target='torpedo', camera='forward',
            step=15, dwell=1.5, speed=40, duration=60)
        if not result.success:
            if log:
                log('[torpedo_task] torpedo board not found in scan — skipping task')
            duburi.pause_detector('forward')
            return

    # ── Coarse align on full board ────────────────────────────────────────────
    duburi.vision.home(
        target='torpedo', camera='forward',
        yaw=True, lat=True, depth=True, on_lost='hold', duration=15)

    # ── Fine align: blood marker is larger, easier to centre on first ─────────
    duburi.set_classes('blood,hole', node=_FWD)
    duburi.vision.home(
        target='blood', camera='forward',
        yaw=True, lat=True, depth=True, on_lost='hold', duration=12)

    # ── Stable lock + fire at hole ────────────────────────────────────────────
    duburi.vision.vision_lock_fire(
        target='hole', camera='forward',
        fire_channel=1,       # torpedo_1 — always explicit
        yaw=True, lat=True, depth=True, forward=False,
        stable_lock_s=3.0, max_attempts=3,
        deadband=0.04,
        kp_yaw=80.0, kp_lat=80.0, kp_depth=0.08,
        duration=60)

    duburi.pause(2.0)
    duburi.pause_detector('forward')
