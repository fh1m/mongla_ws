#!/usr/bin/env python3
"""gate_flare_autonomous -- reactive gate+flare+return using duburi.detected().

This mission demonstrates the detected()-paradigm: the AUV executes open-loop
maneuvers until a target appears in frame, then switches to vision-closed-loop
control. Each while/for loop is a proto-state that maps directly to a future
YASMIN FSM node.

Mission phases:
  0. Countdown    -- operator removes tether
  1. Startup      -- arm, depth, DVL connect (warn if offline)
  2. FindGate     -- while not detected('gate'): move_forward(0.5s steps)
  3. HomeGate     -- vision.home(yaw+lat+gate_guard+pass_at)
  4. PassGate     -- DVL move_forward_dist(GATE_PASS_DIST_M)
  5. FindFlare    -- sweep: yaw + detected('flare') at each stop
  6. HomeFlare    -- vision.home(yaw+forward+depth, height metric)
  7. OrbitFlare   -- yaw_right(20°) × 18, break when detected('gate')
  8. HomeReturn   -- vision.home on gate (if re-found)
  9. ReturnPass   -- DVL move_forward_dist(GATE_RETURN_M)
  10. Surface     -- set_depth(0), disarm

Key design properties:
  • Short open-loop steps (0.5s) keep overshoot below ~0.15m at gain=30.
  • Safety budgets (MAX_*) prevent infinite loops if detector goes offline.
  • Class filter is restored with set_classes('gate,flare') before every
    orbit loop — prevents the "orbit trap" where vision.home(flare) silently
    filters the detector so detected('gate') can never return True.
  • All distance moves use DVL closed-loop (falls back to open-loop if DVL
    is offline, with a WARNING logged).

Recommended launch (BNO085 heading + DVL distance):
    ros2 launch duburi_manager bringup.launch.py \\
        vision:=true \\
        yaw_source:=bno085_dvl \\
        model:=gate_flare_medium_100ep \\
        classes:=gate,flare \\
        conf:=0.45

Fallback without DVL: yaw_source:=bno085
"""

# ── Mission-level constants ─────────────────────────────────────────────────

_SEARCH_GAIN      = 30     # forward thrust % for search steps
_SEARCH_STEP_S    = 0.5    # seconds per open-loop search step
_SWEEP_PAUSE_S    = 0.8    # observation dwell at each flare-search yaw stop
_SWEEP_YAW_DEG    = 10     # degrees per flare search yaw step (36 × 10° = 360°)
_ORBIT_YAW_DEG    = 20     # degrees per orbit yaw step (18 × 20° = 360°)
_ORBIT_DWELL_S    = 1.0    # observation dwell at each orbit stop

_GATE_PASS_DIST_M = 3.0    # DVL forward distance to pass through gate
_GATE_RETURN_M    = 1.5    # DVL forward distance after orbit gate pass
_POOL_DEPTH_M     = -0.8   # mission dive depth (negative = below surface)
_DEPTH_TIMEOUT_S  = 30.0

_MAX_GATE_STEPS   = 60     # safety budget for gate search (60 × 0.5s = 30s)
_MAX_SWEEP_STEPS  = 36     # flare sweep budget (36 × 10° = full 360°)
_MAX_ORBIT_STEPS  = 18     # orbit budget (18 × 20° = full 360°)


def run(duburi, log):
    duburi.camera = 'forward'
    duburi.models(gate='gate_flare_medium_100ep')

    # ── 0. Countdown ────────────────────────────────────────────────────── #
    duburi.countdown(10)

    # ── 1. Startup ──────────────────────────────────────────────────────── #
    duburi.arm()
    duburi.set_depth(_POOL_DEPTH_M, timeout=_DEPTH_TIMEOUT_S, settle=1.5)
    duburi.lock_heading(target=0.0, timeout=300)

    dvl_result = duburi.dvl_connect()
    if not dvl_result.success:
        log.warn('WARNING: DVL connect failed — distance moves will be open-loop (time-based)')

    # Enable both classes for all detected() checks
    duburi.set_classes('gate,flare')

    # ── 2. FindGate — creep forward until gate visible ───────────────────── #
    log.info('=== FindGate ===')
    gate_found = False
    for _ in range(_MAX_GATE_STEPS):
        if duburi.detected(duburi.models.gate.gate, stale_after=0.5):
            gate_found = True
            break
        duburi.move_forward(_SEARCH_STEP_S, gain=_SEARCH_GAIN)
    if not gate_found:
        log.warn('gate not found in search budget — aborting mission')
        _surface_and_disarm(duburi)
        return

    # ── 3. HomeGate — align and commit ──────────────────────────────────── #
    log.info('=== HomeGate ===')
    gate_result = duburi.vision.home(
        target=duburi.models.gate.gate,
        yaw=True, lat=True,
        gate_guard=True, gate_guard_min_w_frac=0.35,
        pass_at=0.38, pass_at_gain=55,
        dist=0.40, metric='area',
        duration=20,
        on_lost='hold',
    )
    if not gate_result.success:
        log.warn('gate alignment failed — attempting open-loop passage')

    # ── 4. PassGate — DVL forward through gate ──────────────────────────── #
    log.info('=== PassGate ===')
    duburi.move_forward_dist(_GATE_PASS_DIST_M, gain=60)

    # ── 5. FindFlare — yaw sweep until flare visible ─────────────────────── #
    log.info('=== FindFlare ===')
    # Restore both classes before sweep (vision.home above may have set classes='gate')
    duburi.set_classes('gate,flare')
    flare_found = False
    for _ in range(_MAX_SWEEP_STEPS):
        if duburi.detected(duburi.models.gate.flare, stale_after=0.5):
            flare_found = True
            break
        duburi.yaw_right(_SWEEP_YAW_DEG)
        duburi.pause(_SWEEP_PAUSE_S)

    if not flare_found:
        log.warn('flare not found in sweep — attempting return through gate directly')
        _return_through_gate(duburi, log)
        _surface_and_disarm(duburi)
        return

    # ── 6. HomeFlare — 3-axis lock on flare ──────────────────────────────── #
    log.info('=== HomeFlare ===')
    duburi.vision.home(
        target=duburi.models.gate.flare,
        yaw=True, forward=True, depth=True,
        dist=0.38, metric='height',
        duration=20,
        on_lost='hold',
    )

    # ── 7. OrbitFlare — yaw steps, break when gate re-appears ───────────── #
    log.info('=== OrbitFlare ===')
    # CRITICAL: vision.home above called set_classes('flare').
    # Restore both classes BEFORE the orbit loop or detected('gate') can never be True.
    duburi.set_classes('gate,flare')
    gate_reacquired = False
    for _ in range(_MAX_ORBIT_STEPS):
        if duburi.detected(duburi.models.gate.gate, stale_after=0.3):
            gate_reacquired = True
            break
        duburi.yaw_right(_ORBIT_YAW_DEG)
        duburi.pause(_ORBIT_DWELL_S)

    if not gate_reacquired:
        log.warn('gate not re-acquired during orbit — attempting blind return')

    _return_through_gate(duburi, log)
    _surface_and_disarm(duburi)


def _return_through_gate(duburi, log):
    """Phase 8+9: home on gate if visible, then DVL pass through."""
    if duburi.detected(duburi.models.gate.gate, stale_after=0.5):
        log.info('=== HomeReturn ===')
        duburi.vision.home(
            target=duburi.models.gate.gate,
            yaw=True, lat=True,
            gate_guard=True,
            dist=0.40, metric='area',
            duration=15,
            on_lost='hold',
        )

    log.info('=== ReturnPass ===')
    duburi.move_forward_dist(_GATE_RETURN_M, gain=60)


def _surface_and_disarm(duburi):
    duburi.release_heading()
    duburi.stop()
    duburi.set_depth(0.0, timeout=60.0)
    duburi.disarm()
