#!/usr/bin/env python3
"""gate_prequal -- RoboSub gate prequalification with DVL square pattern.

Course layout:
  AUV start → [forward] → Gate (2 m wide) → [3 m] → Flare area → [return] → Gate

Mission phases:
  1.  Startup    -- arm, ALT_HOLD, descend, DVL connect
  2.  HomeGate   -- align (yaw) then move forward on 'area' fill
  3.  PassGate   -- DVL forward 3 m (timed fallback if DVL unavailable)
  4.  DVLSquare  -- 4 × (forward 2 m + yaw_right 90°) circles flare area
  5.  Return     -- yaw 180°, re-align gate, DVL pass back through
  6.  Surface    -- stop, surface, disarm

Pre-flight:
  ros2 launch mongla_manager bringup.launch.py vision:=true
  ros2 run mongla_planner mission gate_prequal

Live tuning:
  ros2 param set /mongla_manager vision.kp_yaw 70.0
  ros2 param set /mongla_manager vision.kp_forward 150.0

WARNING: this mission arms the vehicle.
"""

CAMERA       = 'forward'
TARGET_CLASS = 'gate'

DIVE_DEPTH_M = -1.0

# Vision (pixel-native)
ALIGN_ERR_PX   = 40
ALIGN_GAIN     = 30
APPROACH_GAIN  = 45
SEARCH_GAIN    = 35
SEARCH_CREEP_S = 0.6

# Gate
GATE_FWD_FILL   = 35     # gate area % at standoff
ALIGN_TIMEOUT_S = 20.0
MOVE_TIMEOUT_S  = 20.0
GATE_PASS_DIST_M = 3.0
GATE_PASS_GAIN   = 60.0

# DVL square around flare area
SQUARE_SIDE_M = 2.0
SQUARE_GAIN   = 50.0

# Return
RETURN_PASS_DIST = 3.5
RETURN_PASS_GAIN = 60.0


def run(mongla, log):
    mongla.mission_reset()
    mongla.camera = CAMERA
    mongla.target = TARGET_CLASS

    # ── Phase 1: Startup ────────────────────────────────────────────────────
    log('Phase 1: arm + descend')
    mongla.arm()
    mongla.set_mode('ALT_HOLD')
    mongla.set_depth(DIVE_DEPTH_M, settle=1.5)
    mongla.dvl_connect()

    # ── Phase 2: HomeGate ───────────────────────────────────────────────────
    log('Phase 2: homing on gate')
    mongla.vision.align(TARGET_CLASS, yaw=0, err=ALIGN_ERR_PX, gain=ALIGN_GAIN,
                        duration=ALIGN_TIMEOUT_S, fallback=creep_forward)
    mongla.vision.move(TARGET_CLASS, fwd=GATE_FWD_FILL, mode='area',
                      gain=APPROACH_GAIN, duration=MOVE_TIMEOUT_S,
                      fallback=creep_forward)

    # ── Phase 3: PassGate ───────────────────────────────────────────────────
    log('Phase 3: passing through gate (DVL forward)')
    mongla.move_forward_dist(GATE_PASS_DIST_M, gain=GATE_PASS_GAIN)

    # ── Phase 4: DVL square around flare area ───────────────────────────────
    log('Phase 4: DVL square pattern')
    for side in range(4):
        log(f'  square leg {side + 1}/4')
        mongla.move_forward_dist(SQUARE_SIDE_M, gain=SQUARE_GAIN)
        mongla.yaw_right(90.0, timeout=20.0, settle=0.3)

    # ── Phase 5: Return ──────────────────────────────────────────────────────
    log('Phase 5: yaw 180° and return through gate')
    mongla.yaw_right(180.0, timeout=25.0, settle=0.5)
    mongla.vision.align(TARGET_CLASS, yaw=0, err=ALIGN_ERR_PX, gain=ALIGN_GAIN,
                        duration=ALIGN_TIMEOUT_S, fallback=sweep_right)
    mongla.move_forward_dist(RETURN_PASS_DIST, gain=RETURN_PASS_GAIN)

    # ── Phase 6: Surface ────────────────────────────────────────────────────
    log('Phase 6: surfacing')
    mongla.stop()
    mongla.set_depth(0.0, timeout=30.0)
    mongla.disarm()
    log('Mission complete.')


# ── Mission-authored fallback search patterns (pure control) ────────────────────
def creep_forward(mongla):
    mongla.move_forward(SEARCH_CREEP_S, gain=SEARCH_GAIN)


def sweep_right(mongla, should_stop):
    for _ in range(8):
        mongla.yaw_right(25.0)
        if should_stop():
            return
