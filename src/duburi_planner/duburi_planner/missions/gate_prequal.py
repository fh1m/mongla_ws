#!/usr/bin/env python3
"""gate_prequal -- RoboSub gate prequalification with DVL square pattern.

Course layout:
  AUV start → [forward] → Gate (2 m wide) → [3 m] → Flare area → [return] → Gate

Mission phases:
  1.  Startup    -- arm, ALT_HOLD, descend
  2.  FindGate   -- forward sweep until 'gate' seen
  3.  HomeGate   -- yaw + forward; 'area' metric; approach to standoff
  4.  PassGate   -- DVL forward 3 m (timed fallback if DVL unavailable)
  5.  DVLSquare  -- 4 × (forward 2 m + yaw_right 90°) circles flare area
  6.  Return     -- yaw 180°, find gate, home, pass back through
  7.  Surface    -- stop, surface, disarm

Pre-flight:
  ros2 launch duburi_manager bringup.launch.py vision:=true
  ros2 run duburi_planner mission gate_prequal

Live tuning:
  ros2 param set /duburi_manager vision.kp_yaw 70.0
  ros2 param set /duburi_manager vision.kp_forward 150.0
  ros2 param set /duburi_manager vision.deadband 0.08

WARNING: this mission arms the vehicle.
"""

CAMERA       = 'forward'
TARGET_CLASS = 'gate'

DIVE_DEPTH_M = -1.0

# Search
SEARCH_GAIN      = 35.0
SEARCH_TIMEOUT_S = 45.0

# Gate alignment
GATE_STANDOFF  = 0.35   # 35% frame area at standoff
ALIGN_TIMEOUT_S = 20.0
ALIGN_KP_YAW    = 70.0
ALIGN_KP_FWD    = 150.0
ALIGN_DEADBAND  = 0.08

# Gate passage
GATE_PASS_DIST_M = 3.0
GATE_PASS_GAIN   = 60.0

# DVL square around flare area
SQUARE_SIDE_M = 2.0
SQUARE_GAIN   = 50.0

# Return
RETURN_SEARCH_T  = 30.0
RETURN_PASS_DIST = 3.5
RETURN_PASS_GAIN = 60.0


def run(duburi, log):
    duburi.camera = CAMERA
    duburi.target = TARGET_CLASS

    # ── Phase 1: Startup ────────────────────────────────────────────────────
    log('Phase 1: arm + descend')
    duburi.arm()
    duburi.set_mode('ALT_HOLD')
    duburi.set_depth(DIVE_DEPTH_M, settle=1.5)
    duburi.dvl_connect()

    # ── Phase 2: FindGate ───────────────────────────────────────────────────
    log('Phase 2: searching for gate (forward)')
    duburi.vision.find(
        move='forward',
        gain=SEARCH_GAIN,
        timeout=SEARCH_TIMEOUT_S)

    # ── Phase 3: HomeGate ───────────────────────────────────────────────────
    # 'area' metric is stable for a wide short gate; on_lost='hold' rides
    # out detection flickers in turbid or lit pool water.
    log('Phase 3: homing on gate (yaw + forward)')
    duburi.vision.home(
        yaw=True, forward=True,
        dist=GATE_STANDOFF, metric='area',
        duration=ALIGN_TIMEOUT_S,
        on_lost='hold',
        kp_yaw=ALIGN_KP_YAW,
        kp_forward=ALIGN_KP_FWD,
        deadband=ALIGN_DEADBAND)

    # ── Phase 4: PassGate ───────────────────────────────────────────────────
    log('Phase 4: passing through gate (DVL forward)')
    duburi.move_forward_dist(GATE_PASS_DIST_M, gain=GATE_PASS_GAIN)

    # ── Phase 5: DVL square around flare area ───────────────────────────────
    log('Phase 5: DVL square pattern')
    for side in range(4):
        log(f'  square leg {side + 1}/4')
        duburi.move_forward_dist(SQUARE_SIDE_M, gain=SQUARE_GAIN)
        duburi.yaw_right(90.0, timeout=20.0, settle=0.3)

    # ── Phase 6: Return ──────────────────────────────────────────────────────
    log('Phase 6: yaw 180° to return heading')
    duburi.yaw_right(180.0, timeout=25.0, settle=0.5)

    log('Phase 6: searching for gate (stationary sweep)')
    duburi.vision.find(
        move='yaw_right',
        gain=0.0,
        timeout=RETURN_SEARCH_T)

    log('Phase 6: homing on gate for return pass')
    duburi.vision.home(
        yaw=True, forward=True,
        dist=GATE_STANDOFF, metric='area',
        duration=ALIGN_TIMEOUT_S,
        on_lost='hold',
        kp_yaw=ALIGN_KP_YAW,
        kp_forward=ALIGN_KP_FWD,
        deadband=ALIGN_DEADBAND)

    log('Phase 6: passing back through gate')
    duburi.move_forward_dist(RETURN_PASS_DIST, gain=RETURN_PASS_GAIN)

    # ── Phase 7: Surface ────────────────────────────────────────────────────
    log('Phase 7: surfacing')
    duburi.stop()
    duburi.set_depth(0.0, timeout=30.0)
    duburi.disarm()
    log('Mission complete.')
