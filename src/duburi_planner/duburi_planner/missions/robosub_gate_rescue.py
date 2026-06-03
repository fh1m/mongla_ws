#!/usr/bin/env python3
"""robosub_gate_rescue — RoboSub Gate Task 1 + Rescue Approach for Dubomini.

Target platform: Dubomini 2.0 (no DVL, BNO085 heading, forward camera only).
Paradigm: detected() — open-loop search steps + vision closed-loop alignment.

Model: gate_rescue_repair  (classes: 0=gate  1=rescue  2=repair)
  weights → src/duburi_vision/models/gate_rescue_repair.pt

──────────────────────────────────────────────────────────────────────────────
Mission phases:
  0. Countdown        10s tether-removal window with banner countdown
  1. Startup          arm → dive to mission depth → lock BNO085 heading
  2. FindGate         creep forward in 0.5s steps; yaw-sweep fallback
  3. AlignGate        vision.home(gate, yaw+gate_guard only) — heading correction
  4. SlideToRescue    vision.home(rescue, lat only) — slide to rescue side
  5. CommitPass       vision.home(gate, yaw+fwd+gate_guard+pass_at) — gate pass
  6. ClearGate        timed forward burst to ensure full clearance
  7. FindRescue       creep forward; yaw-sweep fallback
  8. ApproachRescue   vision.home(rescue, yaw+lat+fwd) to bbox size threshold
  9. DiveUnder        deeper depth → forward → return to mission depth
  10. Style           4×yaw_right(90°) = 360° = 4 style bonus points
  11. Surface         release heading → stop → surface → disarm

RoboSub 2026 scoring:
  Gate pass (rescue side) = Begin Assessment base points
  Role selection (rescue side chosen) = role-bonus points
  Rescue sign approach + dive-under = additional task credit
  Style (4×90°) = 4 style bonus increments

Gate angle guard:
  gate_guard=True suppresses forward thrust when gate bbox width/height ratio
  is below _GATE_GUARD_FRAC — prevents driving into gate bars when approaching
  from an angle. Phase 3 corrects heading first, so by Phase 5 the gate
  should appear nearly straight-on (gate_guard rarely fires).

Launch commands:
  # Dubomini pool (BNO085 heading, no DVL):
  ros2 launch duburi_manager bringup.launch.py \\
      vision:=true \\
      yaw_source:=bno085 \\
      model:=gate_rescue_repair \\
      classes:=gate,rescue,repair \\
      conf:=0.45

  # Sim / bench (ArduSub AHRS heading):
  ros2 launch duburi_manager bringup.launch.py \\
      vision:=true \\
      yaw_source:=mavlink_ahrs \\
      model:=gate_rescue_repair \\
      classes:=gate,rescue,repair
──────────────────────────────────────────────────────────────────────────────
"""

# ══ Tunable constants — edit per pool session ════════════════════════════════

# Startup
_COUNTDOWN_S         = 10       # tether-removal window (seconds)
_MISSION_DEPTH_M     = -0.70    # dive depth (negative = below surface, m)
_DEPTH_TIMEOUT_S     = 35.0     # max time to reach mission depth

# Gate search
_SEARCH_GAIN         = 30       # forward thrust % for open-loop search steps
_SEARCH_STEP_S       = 0.5      # seconds per step  (short = less overshoot)
_MAX_GATE_STEPS      = 60       # budget: 60×0.5s = 30s forward march
_SWEEP_YAW_DEG       = 20       # degrees per yaw-sweep step
_MAX_SWEEP_STEPS     = 18       # 18×20° = 360° sweep

# Gate alignment & pass
_GATE_ALIGN_DUR_S    = 12.0     # max seconds for yaw-only gate alignment
_RESCUE_SLIDE_DUR_S  =  8.0     # max seconds sliding laterally to rescue side
_GATE_PASS_DUR_S     = 20.0     # max seconds for gate pass (incl. committed drive)
_GATE_PASS_AT        =  0.40    # commit forward when gate area ≥ 40% of frame
_GATE_PASS_GAIN      = 65       # forward % after pass_at triggers
_GATE_GUARD_FRAC     =  0.30    # suppress forward if gate w/h < 0.30
_GATE_CLEAR_S        =  3.0     # timed clear burst after pass_at
_GATE_CLEAR_GAIN     = 65

# Rescue search
_MAX_RESCUE_STEPS    = 50       # budget: 50×0.5s = 25s forward march
_MAX_RESCUE_SWEEP    = 18       # rescue yaw sweep budget

# Rescue approach
_RESCUE_HOME_DIST    =  0.45    # stop when rescue sign fills 45% of frame height
_RESCUE_HOME_DUR_S   = 25.0     # max seconds for approach

# Dive under rescue sign
_DIVE_EXTRA_M        = -0.35    # additional depth below mission depth (deeper)
_DIVE_SETTLE_S       =  1.5     # wait for depth to stabilise before moving
_DIVE_PASS_S         =  3.5     # forward thrust seconds at dive depth
_DIVE_PASS_GAIN      = 50

# Style
_STYLE_YAW_DEG       = 90       # degrees per style yaw step
_STYLE_STEPS         =  4       # 4×90° = 360° = 4 style increments

# ═════════════════════════════════════════════════════════════════════════════


def run(duburi, log):
    # ── Model registration (strict — wrong class name → AttributeError) ── #
    duburi.models(robosub=('gate_rescue_repair', ['gate', 'rescue', 'repair']))
    duburi.camera = 'forward'

    _gate   = duburi.models.robosub.gate
    _rescue = duburi.models.robosub.rescue

    # ── 0. Countdown ─────────────────────────────────────────────────── #
    duburi.countdown(_COUNTDOWN_S)

    # ── 1. Startup ───────────────────────────────────────────────────── #
    log.info('=== Startup ===')
    duburi.arm()
    duburi.set_depth(_MISSION_DEPTH_M, timeout=_DEPTH_TIMEOUT_S, settle=1.5)
    duburi.lock_heading(target=0.0, timeout=300)
    # No dvl_connect — Dubomini has no DVL

    # ── 2. FindGate ──────────────────────────────────────────────────── #
    log.info('=== FindGate ===')
    duburi.set_classes('gate,rescue,repair')

    if not _search_for(duburi, _gate, _MAX_GATE_STEPS, log, 'gate'):
        # Yaw sweep before giving up
        log.warn('FindGate: not in forward march — yaw sweeping')
        if not _sweep_for(duburi, _gate, _MAX_SWEEP_STEPS, log, 'gate'):
            log.warn('FindGate: gate not found — aborting mission')
            _surface_and_disarm(duburi)
            return

    log.info('FindGate: gate detected')

    # ── 3. AlignGate — yaw-only heading correction ───────────────────── #
    # Correct heading to face gate squarely before any lateral offset.
    # gate_guard suppresses forward if gate still appears narrow/angled.
    # lat=False: do NOT slide yet (we haven't chosen rescue side yet).
    log.info('=== AlignGate ===')
    duburi.set_classes('gate')
    duburi.vision.home(
        target=_gate,
        yaw=True, lat=False, forward=False,
        gate_guard=True, gate_guard_min_w_frac=_GATE_GUARD_FRAC,
        duration=_GATE_ALIGN_DUR_S,
        on_lost='hold',
    )

    # ── 4. SlideToRescue — lateral only, no yaw ──────────────────────── #
    # Heading is now locked to gate direction via BNO085 + phase 3 yaw.
    # Slide laterally to centre on the rescue sign (left side of gate).
    # yaw=False: don't undo the heading correction from phase 3.
    log.info('=== SlideToRescue ===')
    duburi.set_classes('gate,rescue')
    duburi.vision.home(
        target=_rescue,
        yaw=False, lat=True, forward=False,
        duration=_RESCUE_SLIDE_DUR_S,
        on_lost='hold',
    )

    # ── 5. CommitPass — advance through gate on rescue side ──────────── #
    # Now aligned to gate heading + positioned on rescue side.
    # yaw=True: maintain heading alignment while advancing.
    # lat=False: keep rescue-side position (don't drift back to gate centre).
    # gate_guard: still active — suppresses forward if gate appears angled.
    # pass_at=0.40: commits full-speed forward when gate fills 40% of frame.
    log.info('=== CommitPass ===')
    duburi.set_classes('gate')
    gate_pass = duburi.vision.home(
        target=_gate,
        yaw=True, lat=False, forward=True,
        gate_guard=True, gate_guard_min_w_frac=_GATE_GUARD_FRAC,
        pass_at=_GATE_PASS_AT, pass_at_gain=_GATE_PASS_GAIN,
        dist=0.45, metric='area',
        duration=_GATE_PASS_DUR_S,
        on_lost='hold',
    )
    if not gate_pass.success:
        log.warn('CommitPass: home() incomplete — continuing with timed clear')

    # ── 6. ClearGate — timed burst to ensure full clearance ──────────── #
    log.info('=== ClearGate ===')
    duburi.move_forward(_GATE_CLEAR_S, gain=_GATE_CLEAR_GAIN)

    # ── 7. FindRescue — search downstream for rescue sign ────────────── #
    log.info('=== FindRescue ===')
    duburi.set_classes('gate,rescue,repair')

    rescue_found = (
        _search_for(duburi, _rescue, _MAX_RESCUE_STEPS, log, 'rescue')
        or _sweep_for(duburi, _rescue, _MAX_RESCUE_SWEEP, log, 'rescue')
    )

    if not rescue_found:
        log.warn('FindRescue: rescue sign not found — skipping approach, doing style+surface')
        _do_style(duburi, log)
        _surface_and_disarm(duburi)
        return

    log.info('FindRescue: rescue sign detected')

    # ── 8. ApproachRescue — vision-locked approach to bbox threshold ──── #
    # Approach until rescue sign height = _RESCUE_HOME_DIST fraction of frame.
    # heading + lateral + forward closed loop.
    log.info('=== ApproachRescue ===')
    duburi.set_classes('rescue')
    duburi.vision.home(
        target=_rescue,
        yaw=True, lat=True, forward=True,
        dist=_RESCUE_HOME_DIST, metric='height',
        duration=_RESCUE_HOME_DUR_S,
        on_lost='hold',
    )

    # ── 9. DiveUnder — descend, pass under sign, ascend ──────────────── #
    # Descend below the rescue sign mounting height → thrust forward → ascend.
    log.info('=== DiveUnder ===')
    dive_depth = _MISSION_DEPTH_M + _DIVE_EXTRA_M   # more negative = deeper
    duburi.set_depth(dive_depth, timeout=20.0, settle=_DIVE_SETTLE_S)
    duburi.move_forward(_DIVE_PASS_S, gain=_DIVE_PASS_GAIN)
    duburi.set_depth(_MISSION_DEPTH_M, timeout=20.0, settle=1.0)

    # ── 10. Style ────────────────────────────────────────────────────── #
    _do_style(duburi, log)

    # ── 11. Surface ──────────────────────────────────────────────────── #
    _surface_and_disarm(duburi)


# ── Helpers ──────────────────────────────────────────────────────────────────

def _search_for(duburi, target, max_steps: int, log, name: str) -> bool:
    """Creep forward in short steps until target detected. Returns True on find."""
    for _ in range(max_steps):
        if duburi.detected(target, stale_after=0.5):
            return True
        duburi.move_forward(_SEARCH_STEP_S, gain=_SEARCH_GAIN)
    return False


def _sweep_for(duburi, target, max_steps: int, log, name: str) -> bool:
    """Yaw sweep until target detected. Returns True on find."""
    for _ in range(max_steps):
        if duburi.detected(target, stale_after=0.5):
            return True
        duburi.yaw_right(_SWEEP_YAW_DEG)
        duburi.pause(0.8)
    return False


def _do_style(duburi, log) -> None:
    """Style maneuver: 4×90° yaw right = 360° = 4 style increment points."""
    log.info(f'=== Style ({_STYLE_STEPS}×{_STYLE_YAW_DEG}° yaw) ===')
    duburi.release_heading()
    for _ in range(_STYLE_STEPS):
        duburi.yaw_right(_STYLE_YAW_DEG)
        duburi.pause(0.3)
    duburi.lock_heading(target=0.0, timeout=30)


def _surface_and_disarm(duburi) -> None:
    """Release heading → stop → surface → disarm."""
    duburi.release_heading()
    duburi.stop()
    duburi.set_depth(0.0, timeout=60.0)
    duburi.disarm()
