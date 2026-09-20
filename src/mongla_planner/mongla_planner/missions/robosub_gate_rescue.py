#!/usr/bin/env python3
"""robosub_gate_rescue — Gate Task 1 + Rescue Approach (Mongla_agile).

Platform : Mongla_agile 2.0  — no DVL, BNO085 heading, forward camera only.
Paradigm : detected()    — open-loop search + vision closed-loop alignment.
Model    : gate_rescue_repair  (classes: 0=gate  1=rescue  2=repair)
           → src/mongla_vision/models/gate_rescue_repair.pt

──────────────────────────────────────────────────────────────────────────────
Mission phases:
  0. Countdown      tether-removal window (COUNTDOWN_S)
  1. Startup        arm → dive → BNO085 heading lock
  2. FindGate       forward march → lateral wiggle → yaw sweep (both dirs)
  3. AlignGate      vision.align(gate, yaw+lat) — heading correction
  4. SlideRescue    vision.align(rescue, lat only) — slide to rescue side
  5. CommitPass     vision.move(gate, fwd fill)
  6. ClearGate      timed burst to ensure full clearance
  7. FindRescue     forward march → lateral wiggle → yaw sweep (both dirs)
  8. ApproachRescue vision.move(rescue, fwd fill) to bbox threshold
  9. DiveUnder      deeper depth → forward → return to mission depth
  10. Style         roll_rock (360° angle-confirmed) + yaw circle
  11. Surface       release heading → stop → surface → disarm

Launch (BNO085, no DVL):
  ros2 launch mongla_manager bringup.launch.py \\
      vision:=true yaw_source:=bno085 \\
      model:=gate_rescue_repair classes:=gate,rescue,repair conf:=0.45

Launch (sim/bench):
  ros2 launch mongla_manager bringup.launch.py \\
      vision:=true yaw_source:=mavlink_ahrs \\
      model:=gate_rescue_repair classes:=gate,rescue,repair
──────────────────────────────────────────────────────────────────────────────
"""

# ── Tunable constants (edit per pool session) ─────────────────────────────────

# Startup
COUNTDOWN_S        = 2        # tether-removal window (seconds; increase for comp)
MISSION_DEPTH      = -0.70    # dive depth in metres (negative = below surface)
DEPTH_TIMEOUT      = 35.0     # max seconds to reach MISSION_DEPTH
DEPTH_SETTLE       = 1.5      # settle time after depth reached

# Search — used by both gate and rescue search phases
SEARCH_GAIN        = 30       # forward/lateral thrust % during search
STEP_S             = 0.5      # seconds per forward step (short = low overshoot)
LATERAL_STEP_S     = 0.8      # seconds per lateral wiggle step
MAX_GATE_STEPS     = 5        # forward-march budget (adjust for pool distance)
MAX_RESCUE_STEPS   = 50       # forward-march budget for rescue sign
MAX_LATERAL_WIGGLES= 3        # left/right wiggles per search (3 = ±3 half-widths)
SWEEP_DEG          = 20       # yaw step size in degrees
MAX_SWEEP_STEPS    = 18       # 18×20° = full 360° sweep

# Vision alignment (pixel-native, two-verb API)
ALIGN_ERR_PX       = 40       # "centred" pixel tolerance for vision.align
ALIGN_GAIN         = 30       # max speed while centring
APPROACH_GAIN      = 45       # max speed while driving forward

# Gate alignment & pass
GATE_ALIGN_DUR     = 12.0     # max seconds for yaw-only gate alignment
RESCUE_SLIDE_DUR   =  8.0     # max seconds sliding laterally to rescue side
GATE_PASS_DUR      = 20.0     # max seconds for committed gate pass
GATE_PASS_FILL     = 50       # drive through until gate area fills 50% of frame
GATE_CLEAR_S       =  3.0     # timed burst after pass to clear gate bars
GATE_CLEAR_GAIN    = 65

# Rescue approach
RESCUE_ALIGN_DUR   = 12.0     # max seconds centring on rescue sign
RESCUE_HOME_DUR    = 25.0     # max seconds approaching rescue sign
RESCUE_FWD_FILL    = 45       # stop when rescue fills 45% of frame height

# Dive under rescue sign
DIVE_EXTRA         = -0.35    # extra depth below MISSION_DEPTH (more negative)
DIVE_SETTLE        =  1.5     # settle after depth change before driving
DIVE_PASS_S        =  3.5     # forward thrust seconds while at dive depth
DIVE_PASS_GAIN     = 50

# Style maneuver
ROLL_GAIN          = 60       # ACRO roll rate % for style_roll
ROLL_TIMEOUT       = 20.0     # max seconds for 360° roll (ACRO, BNO-confirmed)
YAW_STYLE_DEG      = 90.0     # degrees per yaw step
YAW_STYLE_FLIPS    =  1       # number of full 360° yaw rotations
YAW_STYLE_SETTLE   =  1.0     # settle between yaw steps (seconds)

# ──────────────────────────────────────────────────────────────────────────────


def run(mongla, log):
    mongla.mission_reset()
    # Register model — strict: wrong class name raises AttributeError
    mongla.models(robosub=('gate_rescue_repair', ['gate', 'rescue', 'repair']))
    mongla.camera = 'forward'

    gate   = mongla.models.robosub.gate
    rescue = mongla.models.robosub.rescue

    # ── 0. Countdown ──────────────────────────────────────────────────── #
    mongla.countdown(COUNTDOWN_S)

    # ── 1. Startup ────────────────────────────────────────────────────── #
    log('=== Startup ===')
    arm_result = mongla.arm()
    if not arm_result.success:
        mongla.pause(2.0)              # one retry
        arm_result = mongla.arm()
    if not arm_result.success:
        log('WARN: arm failed — aborting')
        return

    mongla.set_depth(MISSION_DEPTH, timeout=DEPTH_TIMEOUT, settle=DEPTH_SETTLE)
    mongla.lock_heading(mongla.head(), timeout=300)
    # No dvl_connect — Mongla_agile has no DVL

    # ── 2. FindGate ───────────────────────────────────────────────────── #
    log('=== FindGate ===')
    mongla.set_classes('gate,rescue,repair')

    found = (
        creep_forward(mongla, gate, MAX_GATE_STEPS)
        or lateral_wiggle(mongla, gate, MAX_LATERAL_WIGGLES)
        or yaw_sweep(mongla, gate, MAX_SWEEP_STEPS)
    )
    if not found:
        log('WARN: gate not found — aborting')
        surface_and_disarm(mongla)
        return

    log('FindGate: detected')

    # ── 3. AlignGate — yaw only, no forward yet ───────────────────────── #
    # Correct heading to face gate straight-on before lateral offset.
    # yaw=0 only touches Ch4 (the heading lock owns it via release_yaw);
    # there is no forward motion in this phase.
    log('=== AlignGate ===')
    mongla.set_classes('gate')
    mongla.vision.align(
        gate, yaw=0,
        err=ALIGN_ERR_PX, gain=ALIGN_GAIN, duration=GATE_ALIGN_DUR,
        fallback=creep_search)

    # ── 4. SlideRescue — lateral only, heading stays locked ───────────── #
    # yaw=False: don't undo the heading correction from phase 3.
    # Slides until rescue sign is centred in frame (rescue side of gate).
    log('=== SlideToRescue ===')
    mongla.set_classes('gate,rescue')
    mongla.vision.align(
        rescue, lat=0,
        err=ALIGN_ERR_PX, gain=ALIGN_GAIN, duration=RESCUE_SLIDE_DUR)

    # ── 5. CommitPass — advance through gate on rescue side ───────────── #
    # move() drives pure forward (no re-centre) until the gate fills
    # GATE_PASS_FILL of the frame; gain caps the approach speed.
    log('=== CommitPass ===')
    mongla.set_classes('gate')
    gate_result = mongla.vision.move(
        gate, fwd=GATE_PASS_FILL, mode='area',
        gain=APPROACH_GAIN, duration=GATE_PASS_DUR,
        fallback=creep_search)
    if not gate_result.ok:
        log('WARN: gate pass incomplete — continuing with timed clear')

    # ── 6. ClearGate — ensure full clearance past bars ────────────────── #
    log('=== ClearGate ===')
    mongla.move_forward(GATE_CLEAR_S, gain=GATE_CLEAR_GAIN)
    mongla.pause(0.8)   # brief stabilise before rescue search

    # ── 7. FindRescue ─────────────────────────────────────────────────── #
    log('=== FindRescue ===')
    mongla.set_classes('gate,rescue,repair')

    found = (
        creep_forward(mongla, rescue, MAX_RESCUE_STEPS)
        or lateral_wiggle(mongla, rescue, MAX_LATERAL_WIGGLES)
        or yaw_sweep(mongla, rescue, MAX_SWEEP_STEPS)
    )
    if not found:
        log('WARN: rescue sign not found — doing style and surfacing')
        do_style(mongla, log)
        surface_and_disarm(mongla)
        return

    log('FindRescue: detected')

    # ── 8. ApproachRescue — vision-locked to bbox threshold ───────────── #
    # Forward until rescue sign height fills RESCUE_HOME_DIST of frame.
    log('=== ApproachRescue ===')
    mongla.set_classes('rescue')
    mongla.vision.align(
        rescue, yaw=0, lat=0,
        err=ALIGN_ERR_PX, gain=ALIGN_GAIN, duration=RESCUE_ALIGN_DUR,
        fallback=creep_search)
    mongla.vision.move(
        rescue, fwd=RESCUE_FWD_FILL, mode='height',
        gain=APPROACH_GAIN, duration=RESCUE_HOME_DUR,
        fallback=creep_search)

    # ── 9. DiveUnder — descend, pass under sign, re-ascend ────────────── #
    log('=== DiveUnder ===')
    dive_depth = MISSION_DEPTH + DIVE_EXTRA   # more negative = deeper
    mongla.set_depth(dive_depth,      timeout=20.0, settle=DIVE_SETTLE)
    mongla.move_forward(DIVE_PASS_S,  gain=DIVE_PASS_GAIN)
    mongla.set_depth(MISSION_DEPTH,   timeout=20.0, settle=1.0)

    # ── 10. Style ─────────────────────────────────────────────────────── #
    do_style(mongla, log)

    # ── 11. Surface ───────────────────────────────────────────────────── #
    surface_and_disarm(mongla)


# ── Helpers ───────────────────────────────────────────────────────────────────

def creep_search(mongla):
    """Fallback for vision.align/move: one short forward creep, then return."""
    mongla.move_forward(STEP_S, gain=SEARCH_GAIN)


def creep_forward(mongla, target, max_steps) -> bool:
    """March forward in STEP_S bursts until target detected. Returns True on find."""
    for _ in range(max_steps):
        if mongla.detected(target, stale_after=0.5):
            return True
        mongla.move_forward(STEP_S, gain=SEARCH_GAIN)
    return False


def lateral_wiggle(mongla, target, max_wiggles) -> bool:
    """Zigzag left then right to sweep horizontal field of view.

    Each wiggle: move left → check → move right (2×) → check → move left (recover).
    Both directions searched — more effective than forward-only when target
    is slightly off-axis (likely given gate is roughly in front of us).
    """
    for _ in range(max_wiggles):
        mongla.move_left(LATERAL_STEP_S, gain=SEARCH_GAIN)
        if mongla.detected(target, stale_after=0.5):
            return True
        mongla.move_right(LATERAL_STEP_S * 2, gain=SEARCH_GAIN)  # cross centre
        if mongla.detected(target, stale_after=0.5):
            return True
        mongla.move_left(LATERAL_STEP_S, gain=SEARCH_GAIN)        # return centre
    return False


def yaw_sweep(mongla, target, max_steps) -> bool:
    """Yaw sweep alternating right/left. Both directions covered.

    Pattern: right, left×2, right×2, left×2 ... (alternating).
    """
    direction = 1   # +1 = right, -1 = left
    for _ in range(max_steps):
        if mongla.detected(target, stale_after=0.5):
            return True
        if direction > 0:
            mongla.yaw_right(SWEEP_DEG)
        else:
            mongla.yaw_left(SWEEP_DEG)
        mongla.pause(0.5)
        direction *= -1   # alternate
    return False


def do_style(mongla, log) -> None:
    """Style maneuver: 360° roll (ACRO, BNO-confirmed) + 360° yaw spin (ALT_HOLD).

    Roll: style_roll — ACRO mode, ACRO_BAL_ROLL/TRAINER zeroed, BNO angle tracking.
          Depth re-acquired automatically inside style_roll.
    Yaw: style_yaw — 4×90° in ALT_HOLD, heading lock active throughout.
    """
    log('=== Style (style_roll + style_yaw) ===')
    mongla.release_heading()
    mongla.style_roll(gain=ROLL_GAIN, timeout=ROLL_TIMEOUT)
    # style_roll re-acquires depth internally; lock heading at post-roll heading
    mongla.lock_heading(mongla.head(), timeout=30)
    mongla.style_yaw(flips=YAW_STYLE_FLIPS, deg_per_step=YAW_STYLE_DEG,
                     settle=YAW_STYLE_SETTLE)


def surface_and_disarm(mongla) -> None:
    """Release heading → stop → surface → disarm."""
    mongla.release_heading()
    mongla.stop()
    mongla.set_depth(0.0, timeout=60.0)
    mongla.disarm()
