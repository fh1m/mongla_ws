#!/usr/bin/env python3
"""robosub_prequal -- RoboNation pre-qualification sequence (two-verb vision).

Course layout:
  Start → [3 m] → Gate (2 m wide, 1 m tall, ~1 m below surface)
                → [10 m] → Marker/Flare (vertical yellow pipe)

Mission phases (each = one future YASMIN state):
  1.  Startup     -- arm, ALT_HOLD, descend
  2.  HomeGate    -- align (yaw) then move forward through gate ('area' fill)
  3.  PassGate    -- strafe left + drive forward through gate
  4.  HomeFlare   -- align (yaw+depth) then move in ('height' fill)
  5.  OrbitFlare  -- 12 × (yaw_left 30° + re-align)
  6.  Return      -- yaw 180°, re-align gate, pass through
  7.  Surface     -- stop, surface, disarm

Tune live (between runs, no rebuild):
  ros2 param set /mongla_manager vision.kp_yaw      60.0
  ros2 param set /mongla_manager vision.kp_forward  200.0
  ros2 param set /mongla_manager vision.lost_grace_s 1.5

WARNING: this mission arms the vehicle.
"""

CAMERA = 'forward'

# ── Depth ────────────────────────────────────────────────────────────────────
DIVE_DEPTH_M   = -1.0
DEPTH_SETTLE_S = 2.0

# ── Vision (pixel-native) ─────────────────────────────────────────────────────
ALIGN_ERR_PX   = 40
ALIGN_GAIN     = 30
APPROACH_GAIN  = 45
SEARCH_GAIN    = 40
SEARCH_CREEP_S = 0.6

# ── Gate ───────────────────────────────────────────────────────────────────────
GATE_FWD_FILL  = 45     # gate area % at standoff
GATE_ALIGN_T   = 20.0
GATE_MOVE_T    = 20.0
GATE_STRAFE_T    = 4.0
GATE_STRAFE_GAIN = 55.0
GATE_DRIVE_T     = 3.0
GATE_DRIVE_GAIN  = 60.0

# ── Flare ──────────────────────────────────────────────────────────────────────
FLARE_FWD_FILL = 40
FLARE_ALIGN_T  = 20.0
FLARE_MOVE_T   = 20.0

# ── Orbit ────────────────────────────────────────────────────────────────────
ORBIT_STEP_DEG = 30.0
ORBIT_STEPS    = 12
ORBIT_STEP_T   = 10.0
ORBIT_SETTLE_S = 0.3
ORBIT_TRACK_T  = 4.0

# ── Return ───────────────────────────────────────────────────────────────────
RETURN_ALIGN_T    = 20.0
RETURN_DRIVE_T    = 5.0
RETURN_DRIVE_GAIN = 60.0


def run(mongla, log):
    mongla.mission_reset()
    mongla.camera = CAMERA
    mongla.models(gate='gate_flare_medium_100ep')
    gate  = mongla.models.gate.gate
    flare = mongla.models.gate.flare

    # ── Phase 1: Startup ────────────────────────────────────────────────────
    log('Phase 1: arming and diving')
    mongla.arm()
    mongla.set_mode('ALT_HOLD')
    mongla.set_depth(DIVE_DEPTH_M, settle=DEPTH_SETTLE_S)

    # ── Phase 2: HomeGate (align yaw, then drive in on area fill) ───────────
    log('Phase 2: homing on gate')
    mongla.vision.align(gate, yaw=0, err=ALIGN_ERR_PX, gain=ALIGN_GAIN,
                        duration=GATE_ALIGN_T, fallback=creep_forward)
    mongla.vision.move(gate, fwd=GATE_FWD_FILL, mode='area',
                      gain=APPROACH_GAIN, duration=GATE_MOVE_T,
                      fallback=creep_forward)

    # ── Phase 3: PassGate ───────────────────────────────────────────────────
    log('Phase 3: passing through gate (left side)')
    mongla.move_left(GATE_STRAFE_T, gain=GATE_STRAFE_GAIN)
    mongla.move_forward(GATE_DRIVE_T, gain=GATE_DRIVE_GAIN)

    # ── Phase 4: HomeFlare (align yaw+depth, then drive in on height fill) ──
    log('Phase 4: homing on flare')
    mongla.vision.align(flare, yaw=0, depth=0, err=ALIGN_ERR_PX,
                        gain=ALIGN_GAIN, duration=FLARE_ALIGN_T,
                        fallback=sweep_right)
    mongla.vision.move(flare, fwd=FLARE_FWD_FILL, mode='height',
                      gain=APPROACH_GAIN, duration=FLARE_MOVE_T,
                      fallback=creep_forward)

    # ── Phase 5: OrbitFlare ─────────────────────────────────────────────────
    log('Phase 5: orbiting flare (12 × 30°)')
    for step in range(ORBIT_STEPS):
        log(f'  orbit step {step + 1}/{ORBIT_STEPS}')
        mongla.yaw_left(ORBIT_STEP_DEG, timeout=ORBIT_STEP_T, settle=ORBIT_SETTLE_S)
        mongla.vision.align(flare, yaw=0, err=ALIGN_ERR_PX, gain=ALIGN_GAIN,
                            duration=ORBIT_TRACK_T)

    # ── Phase 6: Return ──────────────────────────────────────────────────────
    log('Phase 6: yaw 180° and return through gate')
    mongla.yaw_right(180.0, timeout=20.0)
    mongla.vision.align(gate, yaw=0, err=ALIGN_ERR_PX, gain=ALIGN_GAIN,
                        duration=RETURN_ALIGN_T, fallback=sweep_right)
    mongla.move_left(GATE_STRAFE_T, gain=GATE_STRAFE_GAIN)
    mongla.move_forward(RETURN_DRIVE_T, gain=RETURN_DRIVE_GAIN)

    # ── Phase 7: Surface ────────────────────────────────────────────────────
    log('Phase 7: surfacing and disarming')
    mongla.stop()
    mongla.set_depth(0.0)
    mongla.disarm()


# ── Mission-authored fallback search patterns (pure control) ────────────────────
def creep_forward(mongla):
    mongla.move_forward(SEARCH_CREEP_S, gain=SEARCH_GAIN)


def sweep_right(mongla, should_stop):
    for _ in range(8):
        mongla.yaw_right(25.0)
        if should_stop():
            return
