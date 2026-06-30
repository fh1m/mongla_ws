"""Torpedo task — align on board, approach via blood, standoff-lock hole, fire.

Two-verb vision, phased so the close-in shot is robust (see
``.claude/context/precision-alignment.md``):
  1. COARSE align() on the 'torpedo' board (yaw+lat+depth) from distance
  2. lock_heading() on that nulled heading, then move() forward until 'blood'
     fills the frame (height metric) -- heading held by the lock, not vision.
     brake=False so the coarse approach COASTS into range (no reverse-kick exit).
  3. TERMINAL align() = the UNIFIED STANDOFF SHOT on 'hole': lat+depth centering
     PLUS a forward range-hold (fwd=) that drives the hull to a repeatable firing
     standoff and HOLDS it, with lock_on=True (a 2nd hole can't steal the aim) and
     a MID-HOLD fire (the torpedo leaves while the loop is still glued to the hole
     AND parked at the standoff -- no align-then-fire drift, no creeping into the
     board). RoboSub awards bonus points for firing further from the board, and a
     large+stable bbox at standoff is far easier to hold than point-blank -- so we
     fire from the standoff, not from a fragile point-blank creep.

Pool-day tuning (live ROS params, apply on the next goal -- ENABLE these on pool
day if the hull oscillates on the hole or jumps to the wrong opening):
    ros2 param set /duburi_manager vision.range_gain_floor 0.35   # soften close-in gain
    ros2 param set /duburi_manager vision.ctrl_conf        0.55   # reject low-score boxes
    ros2 param set /duburi_manager vision.ki_lat           0.4    # null steady current (after damping)
Standoff range itself is TORPEDO_STANDOFF_FILL in competition_config.py (smaller
fill = the hull parks further back). Read the live `[ align … fwd>=…% ]` line to
calibrate it to ~0.3-0.46m off the board.

Standalone test (torpedo_blood_hole.pt — classes: torpedo(0) blood(1) hole(2)):
    ros2 run duburi_planner mission task_torpedo

fire_channel=1 → torpedo_1 (ESP32 channel 1). Always explicit.
"""

from duburi_planner.missions.competition_config import (
    TORPEDO_HEADING_DEG,
    TORPEDO_DEPTH_M,
    TORPEDO_BLOOD_FWD_FILL,
    TORPEDO_STANDOFF_FILL,
    TORPEDO_STANDOFF_HOLD_S,
    TORPEDO_FIRE_T,
    ALIGN_ERR_PX,
    FINE_ERR_PX,
    ALIGN_GAIN,
    APPROACH_GAIN,
    SEARCH_FORWARD_GAIN,
    SEARCH_CREEP_S,
)

_FWD = '/duburi_detector_forward'
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

    # Heading is now nulled on the board. Hand yaw to the background heading lock
    # so the approach + terminal lock hold heading WITHOUT vision-yaw (Ch4 stays
    # steady -> no close-in yaw limit-cycle on the 20 kg hull). move() and the
    # terminal align() omit yaw, so they take the release_yaw path automatically.
    duburi.lock_heading(timeout=120)
    try:
        # ── 2. Coarse approach: COAST forward until blood fills the frame ──────
        #     brake=False -> no reverse-kick exit; the terminal verb closes the rest.
        # NOTE: the terminal forward-close is capped by the fine `gain` below (12%)
        # -- there is no separate forward gain. If the hull can't reach the standoff
        # inside the budget (fire then withheld), raise TORPEDO_BLOOD_FWD_FILL so the
        # coast lands closer, or raise the terminal gain (and set
        # vision.range_gain_floor to keep lat/depth gentle close-in).
        duburi.set_classes('blood,hole', node=_FWD)
        duburi.vision.move(
            'blood', camera='forward',
            fwd=TORPEDO_BLOOD_FWD_FILL, mode='height',
            gain=APPROACH_GAIN, duration=30, brake=False,
            fallback=creep_forward)

        # ── 3. UNIFIED STANDOFF SHOT: forward-standoff + lat/depth + hold + fire ──
        #     ONE verb drives to the firing standoff (fwd=, height), centres lat+depth
        #     (NO yaw -> heading_lock holds Ch4), continuity-locks the hole, and fires
        #     MID-HOLD while still glued AND parked at the standoff. brake=False (a
        #     fire-from-lock shot is never disturbed by a pre-shot kick).
        duburi.set_classes('hole', node=_FWD)
        locked = duburi.vision.align(
            'hole', camera='forward', lat=0, depth=0,          # NO yaw -> heading_lock
            fwd=TORPEDO_STANDOFF_FILL, fwd_mode='height',       # drive to + hold standoff
            err=FINE_ERR_PX, gain=_FINE_GAIN, duration=25,
            lock_on=True, hold=TORPEDO_STANDOFF_HOLD_S,
            fire=1, fire_t=TORPEDO_FIRE_T, brake=False,         # torpedo_1, mid-hold
            fallback=creep_forward)
        if (not locked) and log:
            log('[torpedo_task] hole standoff never locked — fire was withheld '
                '(gated on lock + standoff)')
    finally:
        # Always release the heading lock -- this chunk is chained inside
        # task_full_2026 with no per-chunk mission_reset, so a leaked lock would
        # drive Ch4 into the next task.
        duburi.unlock_heading()

    duburi.pause(2.0)
    duburi.pause_detector('forward')


# ── Mission-authored fallback search patterns (pure control) ────────────────────
def creep_forward(duburi):
    """One short forward creep, then return so the vision loop retries."""
    duburi.move_forward(SEARCH_CREEP_S, gain=SEARCH_FORWARD_GAIN)
