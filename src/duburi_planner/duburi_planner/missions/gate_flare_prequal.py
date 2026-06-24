#!/usr/bin/env python3
"""gate_flare_prequal -- fully autonomous RoboSub gate+flare pre-qualification.

Course layout (from 2026 briefing image):
  Start → [~3 m] → Gate (2 m wide, ~1 m tall, ~1 m below surface)
         → [~10 m] → Flare/marker (vertical pipe, centre of field)
         → [return ~13 m] → Gate → finish at start

Mission phases (each corresponds to one future YASMIN state):
  0.  Countdown  -- operator removes tether
  1.  Startup    -- arm, ALT_HOLD, descend
  2.  HomeGate   -- align (yaw+lat) then move forward to standoff ('area')
  3.  PassGate   -- DVL forward 3.5 m through gate
  4.  HomeFlare  -- align (yaw+depth) then move in ('height')
  5.  OrbitFlare -- 12 × (yaw_left 30° + re-align)
  6.  Return     -- yaw 180°, re-align gate, DVL pass through
  7.  Surface    -- stop, surface, disarm

Recommended launch (BNO085 heading + DVL distance):
  ros2 launch duburi_manager bringup.launch.py \\
      vision:=true yaw_source:=bno085_dvl \\
      model:=gate_flare_medium_100ep classes:=gate conf:=0.45

Live tuning (between runs, no rebuild):
  ros2 param set /duburi_manager vision.kp_yaw      70.0
  ros2 param set /duburi_manager vision.kp_forward  180.0

WARNING: this mission arms the vehicle and removes the tether.
         Run in Gazebo sim first. Have a safety diver / kill switch ready.
"""

CAMERA = 'forward'

DIVE_DEPTH_M   = -1.0   # gate sits ~1 m below surface
DEPTH_SETTLE_S = 2.0
COUNTDOWN_S    = 10

# ── Vision (pixel-native) ─────────────────────────────────────────────────────
ALIGN_ERR_PX   = 40
ALIGN_GAIN     = 30
APPROACH_GAIN  = 45
SEARCH_GAIN    = 40
SEARCH_CREEP_S = 0.6

# ── Gate ───────────────────────────────────────────────────────────────────────
GATE_FWD_FILL  = 42     # gate area % at standoff
GATE_ALIGN_T   = 20.0
GATE_MOVE_T    = 20.0
GATE_PASS_M    = 3.5
GATE_PASS_GAIN = 60.0

# ── Flare ──────────────────────────────────────────────────────────────────────
FLARE_FWD_FILL = 38     # tall narrow pipe -> height metric
FLARE_ALIGN_T  = 20.0
FLARE_MOVE_T   = 20.0

# ── Orbit ────────────────────────────────────────────────────────────────────
ORBIT_STEP_DEG = 30.0
ORBIT_STEPS    = 12
ORBIT_STEP_T   = 10.0
ORBIT_SETTLE_S = 0.3
ORBIT_TRACK_T  = 4.0

# ── Return ───────────────────────────────────────────────────────────────────
RETURN_ALIGN_T   = 20.0
RETURN_PASS_M    = 3.5
RETURN_PASS_GAIN = 60.0


def run(duburi, log):
    duburi.mission_reset()
    duburi.camera = CAMERA
    duburi.models(gate='gate_flare_medium_100ep')
    gate  = duburi.models.gate.gate
    flare = duburi.models.gate.flare

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

    dvl_result = duburi.dvl_connect()
    if not dvl_result.success:
        log('WARNING: DVL connect failed — distance moves will be open-loop (time-based)')

    # ── Phase 2: HomeGate ───────────────────────────────────────────────────
    log('Phase 2: homing on gate')
    duburi.vision.align(gate, yaw=0, lat=0, err=ALIGN_ERR_PX, gain=ALIGN_GAIN,
                        duration=GATE_ALIGN_T, fallback=creep_forward)
    duburi.vision.move(gate, fwd=GATE_FWD_FILL, mode='area',
                      gain=APPROACH_GAIN, duration=GATE_MOVE_T,
                      fallback=creep_forward)

    # ── Phase 3: PassGate ───────────────────────────────────────────────────
    log('Phase 3: passing through gate (DVL forward)')
    duburi.move_forward_dist(GATE_PASS_M, gain=GATE_PASS_GAIN)

    # ── Phase 4: HomeFlare ──────────────────────────────────────────────────
    log('Phase 4: homing on flare')
    duburi.vision.align(flare, yaw=0, depth=0, err=ALIGN_ERR_PX, gain=ALIGN_GAIN,
                        duration=FLARE_ALIGN_T, fallback=creep_forward)
    duburi.vision.move(flare, fwd=FLARE_FWD_FILL, mode='height',
                      gain=APPROACH_GAIN, duration=FLARE_MOVE_T,
                      fallback=creep_forward)

    # ── Phase 5: OrbitFlare ─────────────────────────────────────────────────
    log('Phase 5: orbiting flare (12 × 30°)')
    for step in range(ORBIT_STEPS):
        log(f'  orbit step {step + 1}/{ORBIT_STEPS}')
        duburi.yaw_left(ORBIT_STEP_DEG, timeout=ORBIT_STEP_T, settle=ORBIT_SETTLE_S)
        duburi.vision.align(flare, yaw=0, err=ALIGN_ERR_PX, gain=ALIGN_GAIN,
                            duration=ORBIT_TRACK_T)

    # ── Phase 6: Return ──────────────────────────────────────────────────────
    log('Phase 6: return — yaw 180° and re-align gate')
    duburi.yaw_right(180.0, timeout=25.0, settle=0.5)
    duburi.vision.align(gate, yaw=0, lat=0, err=ALIGN_ERR_PX, gain=ALIGN_GAIN,
                        duration=RETURN_ALIGN_T, fallback=sweep_right)
    duburi.move_forward_dist(RETURN_PASS_M, gain=RETURN_PASS_GAIN)

    # ── Phase 7: Surface ────────────────────────────────────────────────────
    log('Phase 7: surfacing and disarming')
    duburi.stop()
    duburi.set_depth(0.0, timeout=60.0)
    duburi.disarm()
    log('Mission complete.')


# ── Mission-authored fallback search patterns (pure control) ────────────────────
def creep_forward(duburi):
    duburi.move_forward(SEARCH_CREEP_S, gain=SEARCH_GAIN)


def sweep_right(duburi, should_stop):
    for _ in range(8):
        duburi.yaw_right(25.0)
        if should_stop():
            return
