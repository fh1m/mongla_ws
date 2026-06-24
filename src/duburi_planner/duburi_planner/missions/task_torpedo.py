"""Torpedo task — align on board, approach via blood, fine-lock hole, fire.

Two-verb vision:
  1. align() on the 'torpedo' board (yaw+lat+depth) from distance
  2. move() forward until 'blood' fills the frame (height metric)
  3. align() a tight lock on 'hole' (small err, low gain) -> fire(1)

Firing is just ``if align(...): fire(1)`` — align()'s VisionResult is
truthy only when the hole is centred within err, so no dedicated
lock-and-fire verb is needed.

Standalone test (torpedo_blood_hole.pt — classes: torpedo(0) blood(1) hole(2)):
    ros2 run duburi_planner mission task_torpedo

fire_channel=1 → torpedo_1 (ESP32 channel 1). Always explicit.
"""

from duburi_planner.missions.competition_config import (
    TORPEDO_HEADING_DEG,
    TORPEDO_DEPTH_M,
    TORPEDO_BLOOD_FWD_FILL,
    ALIGN_ERR_PX,
    FINE_ERR_PX,
    ALIGN_GAIN,
    APPROACH_GAIN,
    SEARCH_FORWARD_GAIN,
    SEARCH_CREEP_S,
)

_FWD = '/duburi_detector_fwd'
_FINE_GAIN = 12   # slow + precise for the fire lock


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

    # ── 1. Coarse align on the board (yaw+lat+depth), no forward ──────────────
    duburi.vision.align(
        'torpedo', camera='forward', yaw=0, lat=0, depth=0,
        err=ALIGN_ERR_PX, gain=ALIGN_GAIN, duration=15,
        fallback=creep_forward)

    # ── 2. Blood approach: drive forward until blood fills the frame ──────────
    duburi.set_classes('blood,hole', node=_FWD)
    duburi.vision.move(
        'blood', camera='forward',
        fwd=TORPEDO_BLOOD_FWD_FILL, mode='height',
        gain=APPROACH_GAIN, duration=30,
        fallback=creep_forward)

    # ── 3. Fine hole lock (tight err, slow), then fire on a confirmed lock ────
    duburi.set_classes('hole', node=_FWD)
    locked = duburi.vision.align(
        'hole', camera='forward', yaw=0, lat=0, depth=0,
        err=FINE_ERR_PX, gain=_FINE_GAIN, duration=25,
        fallback=creep_forward)
    if locked:
        duburi.fire(1)       # torpedo_1 — always explicit
    elif log:
        log('[torpedo_task] hole never locked — holding fire')

    duburi.pause(2.0)
    duburi.pause_detector('forward')


# ── Mission-authored fallback search patterns (pure control) ────────────────────
def creep_forward(duburi):
    """One short forward creep, then return so the vision loop retries."""
    duburi.move_forward(SEARCH_CREEP_S, gain=SEARCH_FORWARD_GAIN)
