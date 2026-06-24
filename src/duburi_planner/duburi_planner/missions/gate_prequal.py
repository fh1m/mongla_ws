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
  ros2 launch duburi_manager bringup.launch.py vision:=true
  ros2 run duburi_planner mission gate_prequal

Live tuning:
  ros2 param set /duburi_manager vision.kp_yaw 70.0
  ros2 param set /duburi_manager vision.kp_forward 150.0

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


def run(duburi, log):
    duburi.mission_reset()
    duburi.camera = CAMERA
    duburi.target = TARGET_CLASS

    # ── Phase 1: Startup ────────────────────────────────────────────────────
    log('Phase 1: arm + descend')
    duburi.arm()
    duburi.set_mode('ALT_HOLD')
    duburi.set_depth(DIVE_DEPTH_M, settle=1.5)
    duburi.dvl_connect()

    # ── Phase 2: HomeGate ───────────────────────────────────────────────────
    log('Phase 2: homing on gate')
    duburi.vision.align(TARGET_CLASS, yaw=0, err=ALIGN_ERR_PX, gain=ALIGN_GAIN,
                        duration=ALIGN_TIMEOUT_S, fallback=creep_forward)
    duburi.vision.move(TARGET_CLASS, fwd=GATE_FWD_FILL, mode='area',
                      gain=APPROACH_GAIN, duration=MOVE_TIMEOUT_S,
                      fallback=creep_forward)

    # ── Phase 3: PassGate ───────────────────────────────────────────────────
    log('Phase 3: passing through gate (DVL forward)')
    duburi.move_forward_dist(GATE_PASS_DIST_M, gain=GATE_PASS_GAIN)

    # ── Phase 4: DVL square around flare area ───────────────────────────────
    log('Phase 4: DVL square pattern')
    for side in range(4):
        log(f'  square leg {side + 1}/4')
        duburi.move_forward_dist(SQUARE_SIDE_M, gain=SQUARE_GAIN)
        duburi.yaw_right(90.0, timeout=20.0, settle=0.3)

    # ── Phase 5: Return ──────────────────────────────────────────────────────
    log('Phase 5: yaw 180° and return through gate')
    duburi.yaw_right(180.0, timeout=25.0, settle=0.5)
    duburi.vision.align(TARGET_CLASS, yaw=0, err=ALIGN_ERR_PX, gain=ALIGN_GAIN,
                        duration=ALIGN_TIMEOUT_S, fallback=sweep_right)
    duburi.move_forward_dist(RETURN_PASS_DIST, gain=RETURN_PASS_GAIN)

    # ── Phase 6: Surface ────────────────────────────────────────────────────
    log('Phase 6: surfacing')
    duburi.stop()
    duburi.set_depth(0.0, timeout=30.0)
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
