#!/usr/bin/env python3
"""gate_flare_prequal -- fully autonomous RoboSub gate+flare pre-qualification.

Course layout (from 2026 briefing image):
  Start → [~3 m] → Gate (2 m wide, ~1 m tall, ~1 m below surface)
         → [~10 m] → Flare/marker (vertical pipe, centre of field)
         → [return ~13 m] → Gate → finish at start

Mission phases (each corresponds to one future YASMIN state):
  0.  Countdown  -- operator removes tether
  1.  Startup    -- arm, ALT_HOLD, descend
  2.  FindGate   -- drive forward while watching for 'gate'
  3.  FaceGate   -- turn to centre gate horizontally
  4.  HomeGate   -- yaw + lateral + gate_guard; committed pass once close
  5.  PassGate   -- DVL forward 3.5 m through gate
  6.  FindFlare  -- drive forward while watching for 'flare'
  7.  HomeFlare  -- 3-axis lock: yaw + forward + depth
  8.  OrbitFlare -- 12 × (yaw_left 30° + re-track)
  9.  Return     -- yaw 180°, find gate, home, pass through
  10. Surface    -- stop, surface, disarm

Recommended launch (BNO085 heading + DVL distance):
  ros2 launch duburi_manager bringup.launch.py \\
      vision:=true \\
      yaw_source:=bno085_dvl \\
      model:=gate_flare_medium_100ep \\
      classes:=gate \\
      conf:=0.45

  Fallback without DVL: yaw_source:=bno085
  Sim / bench:          yaw_source:=mavlink_ahrs

Pre-flight:
  ros2 run duburi_manager bringup_check
  ros2 run duburi_vision vision_check --camera forward --require-class gate
  ros2 run duburi_vision vision_thrust_check --camera forward --duration 4

Live tuning (between runs, no rebuild):
  ros2 param set /duburi_manager vision.kp_yaw      70.0
  ros2 param set /duburi_manager vision.kp_forward  180.0
  ros2 param set /duburi_manager vision.deadband     0.08

WARNING: this mission arms the vehicle and removes the tether.
         Run in Gazebo sim first. Have a safety diver / kill switch ready.
"""

# ── Camera ──────────────────────────────────────────────────────────────────
CAMERA = 'forward'

# ── Depth ───────────────────────────────────────────────────────────────────
DIVE_DEPTH_M   = -1.0   # gate sits ~1 m below surface
DEPTH_SETTLE_S = 2.0

# ── Tether countdown ─────────────────────────────────────────────────────────
COUNTDOWN_S = 10

# ── Gate search ──────────────────────────────────────────────────────────────
GATE_SEARCH_GAIN = 40.0   # forward thrust % while scanning
GATE_SEARCH_T    = 45.0   # abort after this many seconds

# ── Gate alignment ───────────────────────────────────────────────────────────
GATE_STANDOFF  = 0.42   # gate fills ~42% of frame AREA at standoff
GATE_ALIGN_T   = 20.0
GATE_KP_YAW    = 70.0
GATE_KP_FWD    = 160.0
GATE_DEADBAND  = 0.08

# ── Gate passage ─────────────────────────────────────────────────────────────
GATE_PASS_M    = 3.5    # DVL metres to clear gate
GATE_PASS_GAIN = 60.0

# ── Flare search ─────────────────────────────────────────────────────────────
FLARE_SEARCH_GAIN = 30.0
FLARE_SEARCH_T    = 40.0

# ── Flare approach ───────────────────────────────────────────────────────────
FLARE_STANDOFF = 0.38   # flare fills ~38% of frame height (tall narrow pipe)
FLARE_ALIGN_T  = 20.0
FLARE_KP_YAW   = 65.0
FLARE_KP_FWD   = 180.0

# ── Flare orbit (12 × 30° = 360° polygon) ───────────────────────────────────
ORBIT_STEP_DEG = 30.0
ORBIT_STEPS    = 12
ORBIT_STEP_T   = 10.0
ORBIT_SETTLE_S = 0.3
ORBIT_TRACK_T  = 3.0   # re-track duration after each step

# ── Return ───────────────────────────────────────────────────────────────────
RETURN_SEARCH_T  = 30.0
RETURN_PASS_M    = 3.5
RETURN_PASS_GAIN = 60.0


def run(duburi, log):
    duburi.camera = CAMERA

    m = duburi.models(
        gate=('gate_flare_medium_100ep', ['gate', 'flare']),
    )

    # ── Phase 0: Tether removal window ──────────────────────────────────────
    log('Phase 0: tether removal countdown')
    duburi.countdown(
        COUNTDOWN_S,
        message='Wire removed  --  Duburi is now autonomous. Good luck.')

    # ── Phase 1: Startup ────────────────────────────────────────────────────
    log('Phase 1: arm + ALT_HOLD + descend')
    duburi.arm()
    duburi.set_mode('ALT_HOLD')
    duburi.set_depth(DIVE_DEPTH_M, settle=DEPTH_SETTLE_S)

    duburi.dvl_connect()   # no-op when yaw_source is not dvl/bno085_dvl

    # ── Phase 2: FindGate ───────────────────────────────────────────────────
    log('Phase 2: searching for gate (forward)')
    duburi.set_classes('gate')
    duburi.vision.find(
        target=m.gate.gate,
        move='forward',
        gain=GATE_SEARCH_GAIN,
        timeout=GATE_SEARCH_T)

    # ── Phase 3: FaceGate ───────────────────────────────────────────────────
    # Yaw-only centering before the multi-axis home step. Reduces approach angle.
    log('Phase 3: facing gate (yaw-only turn)')
    duburi.vision.turn(
        target=m.gate.gate,
        duration=6.0,
        kp_yaw=GATE_KP_YAW,
        deadband=GATE_DEADBAND,
        on_lost='hold')

    # ── Phase 4: HomeGate ───────────────────────────────────────────────────
    # yaw + lateral simultaneously with gate_guard to prevent angled collision.
    # pass_at commits to a straight drive-through once the gate fills >38% area.
    log('Phase 4: homing on gate (yaw + lateral + guard)')
    duburi.vision.home(
        target=m.gate.gate,
        yaw=True, lat=True, forward=True,
        dist=GATE_STANDOFF, metric='area',
        gate_guard=True,
        pass_at=0.38, pass_at_gain=55.0,
        duration=GATE_ALIGN_T,
        on_lost='hold',
        kp_yaw=GATE_KP_YAW,
        kp_forward=GATE_KP_FWD,
        deadband=GATE_DEADBAND)

    # ── Phase 5: PassGate ───────────────────────────────────────────────────
    log('Phase 5: passing through gate (DVL forward)')
    duburi.move_forward_dist(GATE_PASS_M, gain=GATE_PASS_GAIN)

    # ── Phase 6: FindFlare ──────────────────────────────────────────────────
    log('Phase 6: searching for flare (forward)')
    duburi.set_classes('flare')
    duburi.vision.find(
        target=m.gate.flare,
        move='forward',
        gain=FLARE_SEARCH_GAIN,
        timeout=FLARE_SEARCH_T)

    # ── Phase 7: HomeFlare ──────────────────────────────────────────────────
    # 3-axis: yaw + forward + depth. 'height' metric for the tall narrow pipe.
    log('Phase 7: homing on flare (3-axis)')
    duburi.vision.home(
        target=m.gate.flare,
        yaw=True, forward=True, depth=True,
        dist=FLARE_STANDOFF, metric='height',
        duration=FLARE_ALIGN_T,
        on_lost='hold',
        kp_yaw=FLARE_KP_YAW,
        kp_forward=FLARE_KP_FWD)

    # ── Phase 8: OrbitFlare ─────────────────────────────────────────────────
    # 12 × 30° = 360° polygon orbit. Each step: yaw_left → flare drifts
    # off-centre → track() re-centres and holds for ORBIT_TRACK_T seconds.
    log('Phase 8: orbiting flare (12 × 30°)')
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

    # ── Phase 9: Return ──────────────────────────────────────────────────────
    log('Phase 9: return — yaw 180°')
    duburi.yaw_right(180.0, timeout=25.0, settle=0.5)

    log('Phase 9: searching for gate (return, stationary sweep)')
    duburi.set_classes('gate')
    duburi.vision.find(
        target=m.gate.gate,
        move='yaw_right',
        gain=0.0,
        timeout=RETURN_SEARCH_T)

    log('Phase 9: homing on gate for return pass')
    duburi.vision.home(
        target=m.gate.gate,
        yaw=True, lat=True, forward=True,
        dist=GATE_STANDOFF, metric='area',
        gate_guard=True,
        pass_at=0.38, pass_at_gain=55.0,
        duration=GATE_ALIGN_T,
        on_lost='hold',
        kp_yaw=GATE_KP_YAW,
        kp_forward=GATE_KP_FWD,
        deadband=GATE_DEADBAND)

    log('Phase 9: passing back through gate (DVL forward)')
    duburi.move_forward_dist(RETURN_PASS_M, gain=RETURN_PASS_GAIN)

    # ── Phase 10: Surface ────────────────────────────────────────────────────
    log('Phase 10: surfacing and disarming')
    duburi.stop()
    duburi.set_depth(0.0, timeout=30.0)
    duburi.disarm()
    log('Mission complete.')
