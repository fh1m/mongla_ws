"""Torpedo task — align on board, approach via blood, standoff-lock hole, fire.

Two-verb vision, phased so the close-in shot is robust (see
``.claude/context/missions/precision-alignment.md``):
  1. COARSE align() on the 'torpedo' board (yaw+lat) from distance, with
     settle= so it exits squared up.
  2. move() forward until 'blood' fills the frame (height metric). The board
     holds the heading the align left it on. brake=False so the approach
     COASTS into range (no reverse-kick exit).
  3. TERMINAL align() = the STANDOFF SHOT on 'hole': lat centring PLUS a
     forward range-hold (fwd=) that drives to a repeatable firing standoff and
     HOLDS it, with lock_on=True (a second hole cannot steal the aim) and a
     MID-HOLD fire (the torpedo leaves while the loop is still glued to the
     hole and parked at the standoff).

The vertical aim is the depth setpoint. The board owns depth, so no vision
axis moves the hull up or down: the hole's height is met by TORPEDO_DEPTH_M.
Read the terminal align's `end_y_px` on the first runs and correct the depth.

Pool-day tuning (live ROS params, apply on the next goal):
    ros2 param set /mongla_manager vision.range_gain_floor 0.35   # soften close-in gain
    ros2 param set /mongla_manager vision.ctrl_conf        0.55   # reject low-score boxes
    ros2 param set /mongla_manager vision.ki_lat           0.4    # null steady current
The terminal fire-lock deliberately has no settle= -- settle would gate the
mid-hold fire. Standoff range is TORPEDO_STANDOFF_FILL in competition_config.py
(smaller fill = the hull parks further back).

Standalone test (torpedo_blood_hole -- classes: torpedo(0) blood(1) hole(2)):
    ros2 run mongla_planner mission task_torpedo

fire channel = TORPEDO_1_CHANNEL from competition_config (UNASSIGNED until the
payload is wired -- B53).
"""

from mongla_planner.missions.competition_config import (
    TORPEDO_1_CHANNEL,
    TORPEDO_HEADING_DEG,
    TORPEDO_DEPTH_M,
    TORPEDO_BLOOD_FWD_FILL,
    TORPEDO_STANDOFF_FILL,
    TORPEDO_STANDOFF_HOLD_S,
    TORPEDO_FIRE_T,
    FIRE_GAP_S,
    ALIGN_ERR_PX,
    FINE_ERR_PX,
    ALIGN_GAIN,
    APPROACH_GAIN,
    SEARCH_FORWARD_GAIN,
    SEARCH_CREEP_S,
)

_FWD = '/mongla_detector_forward'
_FINE_GAIN = 12   # slow + precise for the fire lock


def run(mongla, log=None):
    mongla.mission_reset()   # clear the abort flag and any demand from a previous run
    assert TORPEDO_DEPTH_M is not None, (
        'TORPEDO_DEPTH_M not set — edit competition_config.py before pool day')

    if TORPEDO_HEADING_DEG is not None:
        mongla.turn(TORPEDO_HEADING_DEG)
    mongla.set_depth(TORPEDO_DEPTH_M, timeout=30)

    mongla.resume_detector('forward')
    mongla.set_model('torpedo_blood_hole', node=_FWD)
    mongla.set_classes('torpedo,blood,hole', node=_FWD)

    # ── 1. Coarse align on the board (yaw+lat), no forward ────────────────────
    mongla.vision.align(
        'torpedo', camera='forward', yaw=0, lat=0,
        err=ALIGN_ERR_PX, gain=ALIGN_GAIN, duration=15, settle=8,
        fallback=creep_forward)

    # ── 2. Coarse approach: COAST forward until blood fills the frame ─────────
    # The terminal forward-close is capped by the fine `gain` below (12 %). If
    # the hull cannot reach the standoff inside the budget (fire then withheld),
    # raise TORPEDO_BLOOD_FWD_FILL so the coast lands closer.
    mongla.set_classes('blood,hole', node=_FWD)
    mongla.vision.move(
        'blood', camera='forward',
        fwd=TORPEDO_BLOOD_FWD_FILL, mode='height',
        gain=APPROACH_GAIN, duration=30, brake=False,
        fallback=creep_forward)

    # ── 3. Standoff shot: forward-standoff + lat + hold + mid-hold fire ───────
    # No yaw axis: the board holds heading, so the launcher does not jitter.
    mongla.set_classes('hole', node=_FWD)
    locked = mongla.vision.align(
        'hole', camera='forward', lat=0,
        fwd=TORPEDO_STANDOFF_FILL, fwd_mode='height',       # drive to + hold standoff
        err=FINE_ERR_PX, gain=_FINE_GAIN, duration=25,
        lock_on=True, hold=TORPEDO_STANDOFF_HOLD_S,
        fire=TORPEDO_1_CHANNEL, fire_t=TORPEDO_FIRE_T, fire_gap=FIRE_GAP_S,
        brake=False,
        fallback=creep_forward)
    if (not locked) and log:
        log('[torpedo_task] hole standoff never locked — fire was withheld '
            '(gated on lock + standoff)')

    mongla.pause(2.0)
    mongla.pause_detector('forward')


# ── Mission-authored fallback search patterns (pure control) ────────────────────
def creep_forward(mongla):
    """One short forward creep, then return so the vision loop retries."""
    mongla.move_forward(SEARCH_CREEP_S, gain=SEARCH_FORWARD_GAIN)
