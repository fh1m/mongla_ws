#!/usr/bin/env python3
"""robosub_gate_rescue — Gate Task 1 + Rescue Approach (Dubomini).

Platform : Dubomini 2.0  — no DVL, BNO085 heading, forward camera only.
Paradigm : detected()    — open-loop search + vision closed-loop alignment.
Model    : gate_rescue_repair  (classes: 0=gate  1=rescue  2=repair)
           → src/duburi_vision/models/gate_rescue_repair.pt

──────────────────────────────────────────────────────────────────────────────
Mission phases:
  0. Countdown      tether-removal window (COUNTDOWN_S)
  1. Startup        arm → dive → BNO085 heading lock
  2. FindGate       forward march → lateral wiggle → yaw sweep (both dirs)
  3. AlignGate      vision.home(gate, yaw+gate_guard) — heading correction
  4. SlideRescue    vision.home(rescue, lat only)     — slide to rescue side
  5. CommitPass     vision.home(gate, yaw+fwd+gate_guard+pass_at)
  6. ClearGate      timed burst to ensure full clearance
  7. FindRescue     forward march → lateral wiggle → yaw sweep (both dirs)
  8. ApproachRescue vision.home(rescue, yaw+lat+fwd) to bbox threshold
  9. DiveUnder      deeper depth → forward → return to mission depth
  10. Style         roll_rock (360° angle-confirmed) + yaw circle
  11. Surface       release heading → stop → surface → disarm

Gate angle guard:
  gate_guard=True suppresses forward when gate bbox w/h < GATE_GUARD_FRAC
  — prevents bar collision on angled approach.
  Phase 3 corrects heading first so Phase 5 rarely needs gate_guard.

Launch (BNO085, no DVL):
  ros2 launch duburi_manager bringup.launch.py \\
      vision:=true yaw_source:=bno085 \\
      model:=gate_rescue_repair classes:=gate,rescue,repair conf:=0.45

Launch (sim/bench):
  ros2 launch duburi_manager bringup.launch.py \\
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

# Gate alignment & pass
GATE_ALIGN_DUR     = 12.0     # max seconds for yaw-only gate alignment
GATE_ALIGN_SETTLE  = 0.3      # settle after alignment before slide
RESCUE_SLIDE_DUR   =  8.0     # max seconds sliding laterally to rescue side
RESCUE_SLIDE_SETTLE=  0.3     # settle after slide
GATE_PASS_DUR      = 20.0     # max seconds for committed gate pass
GATE_PASS_AT       =  0.40    # commit forward when gate area ≥ 40% of frame
GATE_PASS_GAIN     = 65       # forward % after pass_at triggers
GATE_GUARD_FRAC    =  0.30    # suppress forward if gate bbox w/h < 0.30
GATE_CLEAR_S       =  3.0     # timed burst after pass to clear gate bars
GATE_CLEAR_GAIN    = 65

# Rescue approach
RESCUE_HOME_DUR    = 25.0     # max seconds approaching rescue sign
RESCUE_HOME_DIST   =  0.45    # stop when rescue fills 45% of frame height
RESCUE_HOME_SETTLE =  0.5     # settle after approach

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


def run(duburi, log):
    # Register model — strict: wrong class name raises AttributeError
    duburi.models(robosub=('gate_rescue_repair', ['gate', 'rescue', 'repair']))
    duburi.camera = 'forward'

    gate   = duburi.models.robosub.gate
    rescue = duburi.models.robosub.rescue

    # ── 0. Countdown ──────────────────────────────────────────────────── #
    duburi.countdown(COUNTDOWN_S)

    # ── 1. Startup ────────────────────────────────────────────────────── #
    log('=== Startup ===')
    arm_result = duburi.arm()
    if not arm_result.success:
        duburi.pause(2.0)              # one retry
        arm_result = duburi.arm()
    if not arm_result.success:
        log('WARN: arm failed — aborting')
        return

    duburi.set_depth(MISSION_DEPTH, timeout=DEPTH_TIMEOUT, settle=DEPTH_SETTLE)
    duburi.lock_heading(duburi.head(), timeout=300)
    # No dvl_connect — Dubomini has no DVL

    # ── 2. FindGate ───────────────────────────────────────────────────── #
    log('=== FindGate ===')
    duburi.set_classes('gate,rescue,repair')

    found = (
        creep_forward(duburi, gate, MAX_GATE_STEPS)
        or lateral_wiggle(duburi, gate, MAX_LATERAL_WIGGLES)
        or yaw_sweep(duburi, gate, MAX_SWEEP_STEPS)
    )
    if not found:
        log('WARN: gate not found — aborting')
        surface_and_disarm(duburi)
        return

    log('FindGate: detected')

    # ── 3. AlignGate — yaw only, no forward yet ───────────────────────── #
    # Correct heading to face gate straight-on before lateral offset.
    # gate_guard prevents forward if gate still appears narrow (angled approach).
    log('=== AlignGate ===')
    duburi.set_classes('gate')
    duburi.vision.home(
        target=gate,
        yaw=True, lat=False, forward=False,
        gate_guard=True, gate_guard_min_w_frac=GATE_GUARD_FRAC,
        duration=GATE_ALIGN_DUR, settle=GATE_ALIGN_SETTLE,
        on_lost='hold',
    )

    # ── 4. SlideRescue — lateral only, heading stays locked ───────────── #
    # yaw=False: don't undo the heading correction from phase 3.
    # Slides until rescue sign is centred in frame (rescue side of gate).
    log('=== SlideToRescue ===')
    duburi.set_classes('gate,rescue')
    duburi.vision.home(
        target=rescue,
        yaw=False, lat=True, forward=False,
        duration=RESCUE_SLIDE_DUR, settle=RESCUE_SLIDE_SETTLE,
        on_lost='hold',
    )

    # ── 5. CommitPass — advance through gate on rescue side ───────────── #
    # lat=False: hold rescue-side position (don't drift back to gate centre).
    # pass_at: commits full-speed forward once gate fills GATE_PASS_AT of frame.
    log('=== CommitPass ===')
    duburi.set_classes('gate')
    gate_result = duburi.vision.home(
        target=gate,
        yaw=True, lat=False, forward=True,
        gate_guard=True, gate_guard_min_w_frac=GATE_GUARD_FRAC,
        pass_at=GATE_PASS_AT, pass_at_gain=GATE_PASS_GAIN,
        dist=0.45, metric='area',
        duration=GATE_PASS_DUR, settle=0.0,
        on_lost='hold',
    )
    if not gate_result.success:
        log('WARN: gate pass incomplete — continuing with timed clear')

    # ── 6. ClearGate — ensure full clearance past bars ────────────────── #
    log('=== ClearGate ===')
    duburi.move_forward(GATE_CLEAR_S, gain=GATE_CLEAR_GAIN)
    duburi.pause(0.8)   # brief stabilise before rescue search

    # ── 7. FindRescue ─────────────────────────────────────────────────── #
    log('=== FindRescue ===')
    duburi.set_classes('gate,rescue,repair')

    found = (
        creep_forward(duburi, rescue, MAX_RESCUE_STEPS)
        or lateral_wiggle(duburi, rescue, MAX_LATERAL_WIGGLES)
        or yaw_sweep(duburi, rescue, MAX_SWEEP_STEPS)
    )
    if not found:
        log('WARN: rescue sign not found — doing style and surfacing')
        do_style(duburi, log)
        surface_and_disarm(duburi)
        return

    log('FindRescue: detected')

    # ── 8. ApproachRescue — vision-locked to bbox threshold ───────────── #
    # Forward until rescue sign height fills RESCUE_HOME_DIST of frame.
    log('=== ApproachRescue ===')
    duburi.set_classes('rescue')
    duburi.vision.home(
        target=rescue,
        yaw=True, lat=True, forward=True,
        dist=RESCUE_HOME_DIST, metric='height',
        duration=RESCUE_HOME_DUR, settle=RESCUE_HOME_SETTLE,
        on_lost='hold',
    )

    # ── 9. DiveUnder — descend, pass under sign, re-ascend ────────────── #
    log('=== DiveUnder ===')
    dive_depth = MISSION_DEPTH + DIVE_EXTRA   # more negative = deeper
    duburi.set_depth(dive_depth,      timeout=20.0, settle=DIVE_SETTLE)
    duburi.move_forward(DIVE_PASS_S,  gain=DIVE_PASS_GAIN)
    duburi.set_depth(MISSION_DEPTH,   timeout=20.0, settle=1.0)

    # ── 10. Style ─────────────────────────────────────────────────────── #
    do_style(duburi, log)

    # ── 11. Surface ───────────────────────────────────────────────────── #
    surface_and_disarm(duburi)


# ── Helpers ───────────────────────────────────────────────────────────────────

def creep_forward(duburi, target, max_steps) -> bool:
    """March forward in STEP_S bursts until target detected. Returns True on find."""
    for _ in range(max_steps):
        if duburi.detected(target, stale_after=0.5):
            return True
        duburi.move_forward(STEP_S, gain=SEARCH_GAIN)
    return False


def lateral_wiggle(duburi, target, max_wiggles) -> bool:
    """Zigzag left then right to sweep horizontal field of view.

    Each wiggle: move left → check → move right (2×) → check → move left (recover).
    Both directions searched — more effective than forward-only when target
    is slightly off-axis (likely given gate is roughly in front of us).
    """
    for _ in range(max_wiggles):
        duburi.move_left(LATERAL_STEP_S, gain=SEARCH_GAIN)
        if duburi.detected(target, stale_after=0.5):
            return True
        duburi.move_right(LATERAL_STEP_S * 2, gain=SEARCH_GAIN)  # cross centre
        if duburi.detected(target, stale_after=0.5):
            return True
        duburi.move_left(LATERAL_STEP_S, gain=SEARCH_GAIN)        # return centre
    return False


def yaw_sweep(duburi, target, max_steps) -> bool:
    """Yaw sweep alternating right/left. Both directions covered.

    Pattern: right, left×2, right×2, left×2 ... (alternating).
    """
    direction = 1   # +1 = right, -1 = left
    for _ in range(max_steps):
        if duburi.detected(target, stale_after=0.5):
            return True
        if direction > 0:
            duburi.yaw_right(SWEEP_DEG)
        else:
            duburi.yaw_left(SWEEP_DEG)
        duburi.pause(0.5)
        direction *= -1   # alternate
    return False


def do_style(duburi, log) -> None:
    """Style maneuver: 360° roll (ACRO, BNO-confirmed) + 360° yaw spin (ALT_HOLD).

    Roll: style_roll — ACRO mode, ACRO_BAL_ROLL/TRAINER zeroed, BNO angle tracking.
          Depth re-acquired automatically inside style_roll.
    Yaw: style_yaw — 4×90° in ALT_HOLD, heading lock active throughout.
    """
    log('=== Style (style_roll + style_yaw) ===')
    duburi.release_heading()
    duburi.style_roll(gain=ROLL_GAIN, timeout=ROLL_TIMEOUT)
    # style_roll re-acquires depth internally; lock heading at post-roll heading
    duburi.lock_heading(duburi.head(), timeout=30)
    duburi.style_yaw(flips=YAW_STYLE_FLIPS, deg_per_step=YAW_STYLE_DEG,
                     settle=YAW_STYLE_SETTLE)


def surface_and_disarm(duburi) -> None:
    """Release heading → stop → surface → disarm."""
    duburi.release_heading()
    duburi.stop()
    duburi.set_depth(0.0, timeout=60.0)
    duburi.disarm()
