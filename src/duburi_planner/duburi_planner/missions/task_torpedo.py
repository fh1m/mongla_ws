"""Torpedo task — align on board, approach via blood, fine-lock hole, fire.

Two-verb vision, phased so the close-in shot is robust (see
``.claude/context/precision-alignment.md``):
  1. COARSE align() on the 'torpedo' board (yaw+lat+depth) from distance
  2. lock_heading() on that nulled heading, then move() forward until 'blood'
     fills the frame (height metric) -- heading held by the lock, not vision
  3. TERMINAL align() a tight lat+depth lock on 'hole' (NO yaw -> heading_lock
     holds Ch4, killing the close-in yaw wobble) with lock_on=True (a 2nd hole
     can't steal the aim) and a MID-HOLD fire (the torpedo leaves while the loop
     is still glued, not after an align-then-fire drift gap)

Pool-day tuning (live ROS params, apply on the next goal -- start here if the
hull oscillates on the hole or jumps to the wrong opening):
    ros2 param set /duburi_manager vision.range_gain_floor 0.35   # soften close-in gain
    ros2 param set /duburi_manager vision.ctrl_conf        0.55   # reject low-score boxes
    ros2 param set /duburi_manager vision.ki_lat           0.4    # null steady current (after damping)

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

_FWD = '/duburi_detector_forward'
_FINE_GAIN = 12   # slow + precise for the fire lock
_FINE_HOLD_S = 4.0   # station-keep on the hole while the shot leaves
_FINE_FIRE_T = 1.0   # fire this many s into the hold (must be < _FINE_HOLD_S)


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

    # Heading is now nulled on the board. Hand yaw to the background heading lock
    # so the approach + terminal lock hold heading WITHOUT vision-yaw (Ch4 stays
    # steady -> no close-in yaw limit-cycle on the 20 kg hull). move() and the
    # terminal align() omit yaw, so they take the release_yaw path automatically.
    duburi.lock_heading(timeout=120)

    # ── 2. Blood approach: drive forward until blood fills the frame ──────────
    duburi.set_classes('blood,hole', node=_FWD)
    duburi.vision.move(
        'blood', camera='forward',
        fwd=TORPEDO_BLOOD_FWD_FILL, mode='height',
        gain=APPROACH_GAIN, duration=30,
        fallback=creep_forward)

    # ── 3. Terminal hole lock: lat+depth only (yaw held by the lock), continuity
    #       lock so a 2nd hole can't steal the aim, fire MID-HOLD while glued ───
    duburi.set_classes('hole', node=_FWD)
    locked = duburi.vision.align(
        'hole', camera='forward', lat=0, depth=0,         # NO yaw -> heading_lock
        err=FINE_ERR_PX, gain=_FINE_GAIN, duration=25,
        lock_on=True, hold=_FINE_HOLD_S,
        fire=1, fire_t=_FINE_FIRE_T, brake=False,         # torpedo_1, mid-hold
        fallback=creep_forward)
    if (not locked) and log:
        log('[torpedo_task] hole never locked — fire was withheld (gated on lock)')

    duburi.unlock_heading()
    duburi.pause(2.0)
    duburi.pause_detector('forward')


# ── Mission-authored fallback search patterns (pure control) ────────────────────
def creep_forward(duburi):
    """One short forward creep, then return so the vision loop retries."""
    duburi.move_forward(SEARCH_CREEP_S, gain=SEARCH_FORWARD_GAIN)
