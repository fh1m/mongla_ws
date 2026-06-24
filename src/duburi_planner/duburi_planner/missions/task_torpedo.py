"""Torpedo task — align on board, approach via blood, fine-lock hole, fire.

Sequence:
  1. Coarse align on 'torpedo' board (yaw + lat + depth, no forward)
  2. Blood approach: forward drive + micro yaw/lat corrections until close
  3. Switch to 'hole': fine 3-axis lock, stable for 3s, fire

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
    duburi.mission_reset()   # clear heading lock + abort from any previous run
    assert TORPEDO_DEPTH_M is not None, (
        'TORPEDO_DEPTH_M not set — edit competition_config.py before pool day')

    if TORPEDO_HEADING_DEG is not None:
        duburi.turn(TORPEDO_HEADING_DEG)
    duburi.set_depth(TORPEDO_DEPTH_M, timeout=30)

    duburi.resume_detector('forward')
    duburi.set_model('torpedo_blood_hole', node=_FWD)
    duburi.set_classes('torpedo,blood,hole', node=_FWD)

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
                log('[torpedo_task] torpedo board not found — skipping task')
            duburi.pause_detector('forward')
            return

    # ── 1. Coarse align: centre on board from distance, NO forward approach ────
    # High speed OK — far from board, large corrections are safe.
    # Exits once centred; does not advance toward board.
    duburi.vision.home(
        target='torpedo', camera='forward',
        yaw=True, lat=True, depth=True, forward=False,
        speed=0.55, h_frac_close=0.35, deadband=0.12,
        on_lost='hold', duration=15, lock_mode='settle')

    # ── 2. Blood approach: drive forward + micro yaw+lat corrections ───────────
    # lock_mode='pursue' keeps driving until blood fills 30% of frame height.
    # lat-priority gating (motion_vision.py) suppresses forward when laterally
    # off-centre — keeps path straight, no diagonal drift.
    duburi.vision.home(
        target='blood', camera='forward',
        yaw=True, lat=True, depth=False, forward=True,
        dist=0.30, metric='height',
        speed=0.45, h_frac_close=0.20, deadband=0.10,
        on_lost='hold', duration=30, lock_mode='pursue')

    # ── 3. Fine hole lock: yaw + lat + depth, slow + proximity-scaled ─────────
    duburi.set_classes('hole', node=_FWD)
    duburi.vision.home(
        target='hole', camera='forward',
        yaw=True, lat=True, depth=True, forward=False,
        speed=0.20, h_frac_close=0.25, deadband=0.04,
        on_lost='hold', duration=25, lock_mode='settle')

    # ── 4. Stable lock + fire ─────────────────────────────────────────────────
    duburi.vision.vision_lock_fire(
        target='hole', camera='forward',
        fire_channel=1,       # torpedo_1 — always explicit
        yaw=True, lat=True, depth=True, forward=False,
        stable_lock_s=3.0, max_attempts=3,
        deadband=0.03,        # tighter than align phase — dead-centre required
        kp_yaw=60.0, kp_lat=60.0, kp_depth=0.05,
        speed=0.12, h_frac_close=0.25,
        duration=60)

    duburi.pause(2.0)
    duburi.pause_detector('forward')
