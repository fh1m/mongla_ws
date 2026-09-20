#!/usr/bin/env python3
"""gate_flare_autonomous -- reactive gate+flare+return using mongla.detected().

This mission demonstrates the detected()-paradigm: the AUV executes open-loop
maneuvers until a target appears in frame, then switches to vision-closed-loop
control. Each while/for loop is a proto-state that maps directly to a future
YASMIN FSM node.

Mission phases:
  0. Countdown    -- operator removes tether
  1. Startup      -- arm, depth, DVL connect (warn if offline)
  2. FindGate     -- while not detected('gate'): move_forward(0.5s steps)
  3. HomeGate     -- vision.align(gate, yaw+lat)
  4. PassGate     -- vision.move(gate, fwd fill)
  5. FindFlare    -- sweep: yaw + detected('flare') at each stop
  6. HomeFlare    -- vision.align(flare, yaw) + vision.move(flare)
  7. OrbitFlare   -- yaw_right(20°) × 18, break when detected('gate')
  8. HomeReturn   -- vision.align on gate (if re-found)
  9. ReturnPass   -- vision.move(gate) / DVL move_forward_dist(GATE_RETURN_M)
  10. Surface     -- set_depth(0), disarm

Key design properties:
  • Short open-loop steps (0.5s) keep overshoot below ~0.15m at gain=30.
  • Safety budgets (MAX_*) prevent infinite loops if detector goes offline.
  • Class filter is restored with set_classes('gate,flare') before every
    orbit loop — prevents the "orbit trap" where vision.align(flare) silently
    filters the detector so detected('gate') can never return True.
  • All distance moves use DVL closed-loop (falls back to open-loop if DVL
    is offline, with a WARNING logged).

Recommended launch (BNO085 heading + DVL distance):
    ros2 launch mongla_manager bringup.launch.py \\
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

# Vision (two-verb, pixel-native)
_ALIGN_ERR_PX     = 40     # "centred" pixel tolerance
_ALIGN_GAIN       = 30     # max speed while centring
_APPROACH_GAIN    = 45     # max speed while driving forward
_FLARE_FWD_FILL   = 38     # flare height % of frame at end of approach


def run(mongla, log):
    mongla.mission_reset()   # clear heading lock + abort from any previous run
    mongla.camera = 'forward'
    mongla.models(gate='gate_flare_medium_100ep')

    # ── 0. Countdown ────────────────────────────────────────────────────── #
    mongla.countdown(10)

    # ── 1. Startup ──────────────────────────────────────────────────────── #
    mongla.arm()
    mongla.set_depth(_POOL_DEPTH_M, timeout=_DEPTH_TIMEOUT_S, settle=1.5)
    mongla.lock_heading(target=0.0, timeout=300)

    dvl_result = mongla.dvl_connect()
    if not dvl_result.success:
        log('WARN: DVL connect failed — distance moves will be open-loop (time-based)')

    # Enable both classes for all detected() checks
    mongla.set_classes('gate,flare')

    # ── 2. FindGate — creep forward until gate visible ───────────────────── #
    log('=== FindGate ===')
    gate_found = False
    for _ in range(_MAX_GATE_STEPS):
        if mongla.detected(mongla.models.gate.gate, stale_after=0.5):
            gate_found = True
            break
        mongla.move_forward(_SEARCH_STEP_S, gain=_SEARCH_GAIN)
    if not gate_found:
        log('WARN: gate not found in search budget — aborting mission')
        _surface_and_disarm(mongla)
        return

    # ── 3. HomeGate — align and commit ──────────────────────────────────── #
    log('=== HomeGate ===')
    gate_result = mongla.vision.align(
        mongla.models.gate.gate, yaw=0, lat=0,
        err=_ALIGN_ERR_PX, gain=_ALIGN_GAIN, duration=20,
        fallback=_creep)
    if not gate_result.ok:
        log('WARN: gate alignment failed — attempting open-loop passage')

    # ── 4. PassGate — DVL forward through gate ──────────────────────────── #
    log('=== PassGate ===')
    mongla.move_forward_dist(_GATE_PASS_DIST_M, gain=60)

    # ── 5. FindFlare — yaw sweep until flare visible ─────────────────────── #
    log('=== FindFlare ===')
    # Restore both classes before sweep (the gate align/move above set classes='gate'
    # via its ClassRef target)
    mongla.set_classes('gate,flare')
    flare_found = False
    for _ in range(_MAX_SWEEP_STEPS):
        if mongla.detected(mongla.models.gate.flare, stale_after=0.5):
            flare_found = True
            break
        mongla.yaw_right(_SWEEP_YAW_DEG)
        mongla.pause(_SWEEP_PAUSE_S)

    if not flare_found:
        log('WARN: flare not found in sweep — attempting return through gate directly')
        _return_through_gate(mongla, log)
        _surface_and_disarm(mongla)
        return

    # ── 6. HomeFlare — 3-axis lock on flare ──────────────────────────────── #
    log('=== HomeFlare ===')
    mongla.vision.align(
        mongla.models.gate.flare, yaw=0, depth=0,
        err=_ALIGN_ERR_PX, gain=_ALIGN_GAIN, duration=15,
        fallback=_creep)
    mongla.vision.move(
        mongla.models.gate.flare, fwd=_FLARE_FWD_FILL, mode='height',
        gain=_APPROACH_GAIN, duration=20, fallback=_creep)

    # ── 7. OrbitFlare — yaw steps, break when gate re-appears ───────────── #
    log('=== OrbitFlare ===')
    # CRITICAL: the flare align/move above set classes='flare' via its ClassRef.
    # Restore both classes BEFORE the orbit loop or detected('gate') can never be True.
    mongla.set_classes('gate,flare')
    gate_reacquired = False
    for _ in range(_MAX_ORBIT_STEPS):
        if mongla.detected(mongla.models.gate.gate, stale_after=0.3):
            gate_reacquired = True
            break
        mongla.yaw_right(_ORBIT_YAW_DEG)
        mongla.pause(_ORBIT_DWELL_S)

    if not gate_reacquired:
        log('WARN: gate not re-acquired during orbit — attempting blind return')

    _return_through_gate(mongla, log)
    _surface_and_disarm(mongla)


def _return_through_gate(mongla, log):
    """Phase 8+9: home on gate if visible, then DVL pass through."""
    if mongla.detected(mongla.models.gate.gate, stale_after=0.5):
        log('=== HomeReturn ===')
        mongla.vision.align(
            mongla.models.gate.gate, yaw=0, lat=0,
            err=_ALIGN_ERR_PX, gain=_ALIGN_GAIN, duration=15,
            fallback=_creep)

    log('=== ReturnPass ===')
    mongla.move_forward_dist(_GATE_RETURN_M, gain=60)


def _creep(mongla):
    """Fallback for vision.align/move: one short forward creep, then return."""
    mongla.move_forward(_SEARCH_STEP_S, gain=_SEARCH_GAIN)


def _surface_and_disarm(mongla):
    mongla.release_heading()
    mongla.stop()
    mongla.set_depth(0.0, timeout=60.0)
    mongla.disarm()
