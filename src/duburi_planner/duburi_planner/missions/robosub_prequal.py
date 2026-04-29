#!/usr/bin/env python3
"""robosub_prequal -- RoboNation pre-qualification sequence.

Course layout:
  Start → [3 m] → Gate (2 m wide, 1 m tall, ~1 m below surface)
                → [10 m] → Marker/Flare (vertical yellow pipe)

Mission phases (each = one future YASMIN state):
  1.  Startup     -- arm, ALT_HOLD, descend
  2.  FindGate    -- drive forward while watching for 'gate'
  3.  HomeGate    -- yaw + forward; 'area' metric; approach to standoff
  4.  PassGate    -- strafe left + drive forward through gate
  5.  FindFlare   -- sweep right while watching for 'flare'
  6.  HomeFlare   -- 3-axis: yaw + forward + depth; 'height' metric
  7.  OrbitFlare  -- 12 × (yaw_left 30° + re-track) ≈ 360° polygon
  8.  Return      -- yaw 180°, find gate, home, pass through
  9.  Surface     -- stop, surface, disarm

Tune live (between runs, no rebuild):
  ros2 param set /duburi_manager vision.kp_yaw      60.0
  ros2 param set /duburi_manager vision.kp_forward  200.0
  ros2 param set /duburi_manager vision.deadband     0.12

WARNING: this mission arms the vehicle.
"""

CAMERA = 'forward'

# ── Depth ────────────────────────────────────────────────────────────────────
DIVE_DEPTH_M   = -1.0
DEPTH_SETTLE_S = 2.0

# ── Gate approach ────────────────────────────────────────────────────────────
GATE_SEARCH_GAIN    = 40.0
GATE_SEARCH_T       = 45.0
GATE_STANDOFF       = 0.45   # 45% area at standoff
GATE_ALIGN_T        = 20.0

# ── Gate pass (strafe + drive) ───────────────────────────────────────────────
GATE_STRAFE_T    = 4.0
GATE_STRAFE_GAIN = 55.0
GATE_DRIVE_T     = 3.0
GATE_DRIVE_GAIN  = 60.0

# ── Flare approach ───────────────────────────────────────────────────────────
FLARE_SEARCH_T       = 35.0
FLARE_SEARCH_YAW     = 25.0
FLARE_STANDOFF       = 0.40
FLARE_ALIGN_T        = 20.0

# ── Flare orbit ──────────────────────────────────────────────────────────────
ORBIT_STEP_DEG = 30.0
ORBIT_STEPS    = 12
ORBIT_STEP_T   = 10.0
ORBIT_SETTLE_S = 0.3
ORBIT_TRACK_T  = 3.0

# ── Return ───────────────────────────────────────────────────────────────────
RETURN_SEARCH_T  = 30.0
RETURN_ALIGN_T   = 20.0
RETURN_DRIVE_T   = 5.0
RETURN_DRIVE_GAIN = 60.0


def run(duburi, log):
    duburi.camera = CAMERA

    m = duburi.models(
        gate=('gate_flare_medium_100ep', ['gate', 'flare']),
    )

    # ── Phase 1: Startup ────────────────────────────────────────────────────
    log('Phase 1: arming and diving')
    duburi.arm()
    duburi.set_mode('ALT_HOLD')
    duburi.set_depth(DIVE_DEPTH_M, settle=DEPTH_SETTLE_S)

    # ── Phase 2: FindGate ───────────────────────────────────────────────────
    log('Phase 2: searching for gate (forward)')
    duburi.set_classes('gate')
    duburi.vision.find(
        target=m.gate.gate,
        move='forward',
        gain=GATE_SEARCH_GAIN,
        timeout=GATE_SEARCH_T)

    # ── Phase 3: HomeGate ───────────────────────────────────────────────────
    # 'area' metric is more stable for a wide, short gate than 'height' alone.
    log('Phase 3: homing on gate (yaw + forward)')
    duburi.vision.home(
        target=m.gate.gate,
        yaw=True, forward=True,
        dist=GATE_STANDOFF, metric='area',
        duration=GATE_ALIGN_T,
        on_lost='hold',
        lock_mode='settle')

    # ── Phase 4: PassGate ───────────────────────────────────────────────────
    log('Phase 4: passing through gate (left side)')
    duburi.move_left(GATE_STRAFE_T, gain=GATE_STRAFE_GAIN)
    duburi.move_forward(GATE_DRIVE_T, gain=GATE_DRIVE_GAIN)

    # ── Phase 5: FindFlare ──────────────────────────────────────────────────
    log('Phase 5: searching for flare (sweep right)')
    duburi.set_classes('flare')
    duburi.vision.find(
        target=m.gate.flare,
        move='yaw_right',
        yaw_rate_pct=FLARE_SEARCH_YAW,
        gain=GATE_SEARCH_GAIN,
        timeout=FLARE_SEARCH_T)

    # ── Phase 6: HomeFlare ──────────────────────────────────────────────────
    log('Phase 6: homing on flare (3-axis)')
    duburi.vision.home(
        target=m.gate.flare,
        yaw=True, forward=True, depth=True,
        dist=FLARE_STANDOFF, metric='height',
        duration=FLARE_ALIGN_T,
        on_lost='hold',
        lock_mode='settle')

    # ── Phase 7: OrbitFlare ─────────────────────────────────────────────────
    log('Phase 7: orbiting flare (12 × 30°)')
    for step in range(ORBIT_STEPS):
        log(f'  orbit step {step + 1}/{ORBIT_STEPS} '
            f'({(step + 1) * ORBIT_STEP_DEG:.0f}° total)')
        duburi.yaw_left(ORBIT_STEP_DEG, timeout=ORBIT_STEP_T, settle=ORBIT_SETTLE_S)
        duburi.vision.track(
            target=m.gate.flare,
            yaw=True, forward=True, depth=True,
            dist=FLARE_STANDOFF,
            duration=ORBIT_TRACK_T,
            on_lost='hold')

    # ── Phase 8: Return ──────────────────────────────────────────────────────
    log('Phase 8: yaw 180° to return heading')
    duburi.yaw_right(180.0, timeout=20.0)

    log('Phase 8: searching for gate (return leg)')
    duburi.set_classes('gate')
    duburi.vision.find(
        target=m.gate.gate,
        move='yaw_right',
        yaw_rate_pct=20.0,
        gain=0.0,
        timeout=RETURN_SEARCH_T)

    log('Phase 8: homing on gate for return pass')
    duburi.vision.home(
        target=m.gate.gate,
        yaw=True, forward=True,
        dist=GATE_STANDOFF, metric='area',
        duration=RETURN_ALIGN_T,
        on_lost='hold',
        lock_mode='settle')

    log('Phase 8: passing through gate (return leg)')
    duburi.move_left(GATE_STRAFE_T, gain=GATE_STRAFE_GAIN)
    duburi.move_forward(RETURN_DRIVE_T, gain=RETURN_DRIVE_GAIN)

    # ── Phase 9: Surface ────────────────────────────────────────────────────
    log('Phase 9: surfacing and disarming')
    duburi.stop()
    duburi.set_depth(0.0)
    duburi.disarm()
