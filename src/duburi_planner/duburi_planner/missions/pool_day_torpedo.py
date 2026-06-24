"""Pool day torpedo practice: align → approach → fine-lock → fire.

Sequence
────────
  1. Arm + depth → 3 s forward drive to start position
  2. Search: short forward sweeps until 'torpedo' board detected
  3. Coarse align on 'torpedo' board (yaw + lat + depth, NO forward approach)
  4. Blood approach: switch to 'blood', drive forward with micro yaw+lat corrections
     until blood fills BLOOD_TARGET_H_FRAC (lat-priority gating keeps path straight)
  5. Switch to 'hole': fine 3-axis lock (yaw + lat + depth, no forward)
  6. Stable-lock + fire: hold for FIRE_STABLE_LOCK_S then trigger ESP32 channel

All tunables are in the OPERATOR TUNABLES block. Edit here, no rebuild needed.

Launch:
    ros2 run duburi_planner mission pool_day_torpedo

Live gain tuning (between runs, no rebuild):
    ros2 param set /duburi_manager vision.kp_yaw 70.0
    ros2 param set /duburi_manager vision.kp_lat 60.0
    ros2 param set /duburi_manager vision.kp_depth 0.05

Ctrl-C at any time → AUV stops and disarms cleanly.
"""
import time

# ┌─────────────────────────────────────────────────────────────────────────────┐
# │                         OPERATOR TUNABLES                                   │
# └─────────────────────────────────────────────────────────────────────────────┘

# ── STARTUP ───────────────────────────────────────────────────────────────────
TETHER_REMOVE_PAUSE_S  = 5.0    # remove tether before AUV arms

# ── DEPTH + HEADING ───────────────────────────────────────────────────────────
TORPEDO_DEPTH_M        = None   # ★ SET AT POOL — e.g. -0.8 (negative = below surface)
TORPEDO_HEADING_DEG    = None   # None = skip, or compass bearing to face the board

# ── LAUNCH DRIVE (open-loop, puts AUV into starting range) ───────────────────
LAUNCH_FORWARD_S       = 3.0    # seconds
LAUNCH_FORWARD_GAIN    = 35     # % thrust — slow, just to clear start position

# ── SEARCH (forward creep until 'torpedo' detected) ───────────────────────────
SEARCH_FORWARD_GAIN    = 40     # % thrust per step
SEARCH_STEP_S          = 0.8    # seconds per forward step
SEARCH_MAX_STEPS       = 8      # steps before yaw scan fallback

# ── COARSE BOARD ALIGN (yaw + lat + depth on 'torpedo', no forward) ──────────
# AUV centres on the whole board from distance before approaching.
BOARD_SPEED            = 0.55   # fast — board is large target, far away
BOARD_H_FRAC_CLOSE     = 0.35   # proximity scaling starts when board fills 35% height
BOARD_DEADBAND         = 0.12   # coarse — board bbox is big
BOARD_ALIGN_S          = 20.0   # max seconds to centre

# ── BLOOD APPROACH (forward + micro yaw+lat corrections) ─────────────────────
# Uses lock_mode='pursue': AUV drives forward while keeping yaw+lat centred on
# the blood marker. lat-priority gating in motion_vision.py suppresses forward
# when lateral error is large — path stays straight, no diagonal drift.
# Exits when blood bbox height fraction ≥ BLOOD_TARGET_H_FRAC.
BLOOD_APPROACH_SPEED   = 0.45   # moderate forward rate during approach
BLOOD_H_FRAC_CLOSE     = 0.20   # proximity scaling starts this early on approach
BLOOD_TARGET_H_FRAC    = 0.30   # stop approach when blood fills 30% of frame height
BLOOD_DEADBAND         = 0.10   # medium — micro corrections, not precision lock
BLOOD_APPROACH_S       = 35.0   # max seconds to reach distance

# ── HOLE FINE LOCK (yaw + lat + depth, no forward — AUV is now close) ────────
# Tighter deadband than blood phase; proximity-scaled gains keep it very still.
HOLE_SPEED             = 0.20   # slow — close to hole, tiny corrections needed
HOLE_H_FRAC_CLOSE      = 0.25   # proximity scaling threshold for hole
HOLE_DEADBAND          = 0.04   # tighter than BOARD_DEADBAND — requires real centre
HOLE_ALIGN_S           = 30.0   # max seconds to dial in

# ── FIRE ─────────────────────────────────────────────────────────────────────
FIRE_CHANNEL           = 1      # 1=torpedo_1  2=torpedo_2  (always explicit)
FIRE_STABLE_LOCK_S     = 3.0    # ★ TUNE AT POOL — seconds of stillness before fire
FIRE_DEADBAND          = 0.03   # tightest — must be dead-centre to count stable
FIRE_SPEED             = 0.12   # barely correcting when near-locked
FIRE_H_FRAC_CLOSE      = 0.25   # proximity scaling for fire phase
FIRE_MAX_ATTEMPTS      = 3      # retries before fallback-fire

_FWD = '/duburi_detector_fwd'


def run(duburi, log=None):
    duburi.mission_reset()   # clear heading lock + abort from any previous run

    def info(msg):
        if log:
            log(msg)

    assert TORPEDO_DEPTH_M is not None, (
        'TORPEDO_DEPTH_M not set — measure hole depth at pool and set it above')

    # ── 0. Tether removal window ───────────────────────────────────────────────
    info(f'Torpedo practice — remove tether in {TETHER_REMOVE_PAUSE_S:.0f}s ...')
    time.sleep(TETHER_REMOVE_PAUSE_S)

    # ── 1. Arm + depth ────────────────────────────────────────────────────────
    duburi.arm()
    duburi.set_depth(TORPEDO_DEPTH_M)
    info(f'Armed. Descending to {TORPEDO_DEPTH_M}m — waiting for depth settle')
    time.sleep(3.0)

    # ── 2. Optional heading turn to face board ────────────────────────────────
    if TORPEDO_HEADING_DEG is not None:
        info(f'Turning to heading {TORPEDO_HEADING_DEG}°')
        duburi.turn(TORPEDO_HEADING_DEG)

    # ── 3. Launch: short forward drive to starting range ──────────────────────
    info(f'Launch drive: {LAUNCH_FORWARD_S:.0f}s forward at {LAUNCH_FORWARD_GAIN}% gain')
    duburi.move_forward(LAUNCH_FORWARD_S, gain=LAUNCH_FORWARD_GAIN)

    # ── 4. Detector setup ─────────────────────────────────────────────────────
    duburi.resume_detector('forward')
    duburi.set_model('torpedo_blood_hole', node=_FWD)
    duburi.set_classes('torpedo,blood,hole', node=_FWD)

    # ── 5. Search for torpedo board ───────────────────────────────────────────
    info('Searching for torpedo board ...')
    for _ in range(SEARCH_MAX_STEPS):
        if duburi.detected('torpedo', stale_after=1.0):
            break
        duburi.move_forward(SEARCH_STEP_S, gain=SEARCH_FORWARD_GAIN)
    else:
        result = duburi.vision.scan(
            target='torpedo', camera='forward',
            step=15, dwell=1.5, speed=35, duration=60)
        if not result.success:
            info('[torpedo] board not found in scan — aborting')
            duburi.pause_detector('forward')
            duburi.disarm()
            return

    # ── 6. Coarse align: centre on torpedo board, NO forward approach ─────────
    # High speed is fine — AUV is far from board, large bbox corrections are safe.
    # depth=True corrects vertical misalignment on the full board.
    # Exits once centred (lock_mode='settle') — does not drive toward board.
    info('Coarse align on torpedo board (yaw + lat + depth, no approach) ...')
    duburi.vision.home(
        target='torpedo', camera='forward',
        yaw=True, lat=True, depth=True, forward=False,
        speed=BOARD_SPEED,
        h_frac_close=BOARD_H_FRAC_CLOSE,
        deadband=BOARD_DEADBAND,
        duration=BOARD_ALIGN_S,
        on_lost='hold',
        lock_mode='settle',
    )

    # ── 7. Blood approach: drive forward + micro yaw+lat corrections ──────────
    # Blood is a medium-sized marker — easier to track during movement than 'hole'.
    # lock_mode='pursue' keeps driving forward until blood fills BLOOD_TARGET_H_FRAC.
    # lat-priority gating (motion_vision.py) suppresses forward when lateral error
    # is large → AUV stays centred then advances, never moves diagonally.
    # depth=False here: ArduSub ALT_HOLD holds depth during the drive.
    info('Approach: blood class, forward + micro yaw/lat corrections ...')
    duburi.vision.home(
        target='blood', camera='forward',
        yaw=True, lat=True, depth=False, forward=True,
        dist=BLOOD_TARGET_H_FRAC, metric='height',
        speed=BLOOD_APPROACH_SPEED,
        h_frac_close=BLOOD_H_FRAC_CLOSE,
        deadband=BLOOD_DEADBAND,
        duration=BLOOD_APPROACH_S,
        on_lost='hold',
        lock_mode='pursue',
    )

    # ── 8. Switch to hole class ───────────────────────────────────────────────
    info('Close enough — switching to hole class for fine lock')
    duburi.set_classes('hole', node=_FWD)
    time.sleep(0.3)   # let one detection tick arrive

    # ── 9. Fine hole lock: yaw + lat + depth, slow, proximity-scaled ──────────
    # AUV is now close: at full proximity, effective gain ≈ HOLE_SPEED × 0.2 = 4%.
    # HOLE_DEADBAND < BLOOD_DEADBAND — must be properly centred to count.
    # No forward — already at firing distance.
    info('Fine lock on hole (yaw + lat + depth, no approach) ...')
    duburi.vision.home(
        target='hole', camera='forward',
        yaw=True, lat=True, depth=True, forward=False,
        speed=HOLE_SPEED,
        h_frac_close=HOLE_H_FRAC_CLOSE,
        deadband=HOLE_DEADBAND,
        duration=HOLE_ALIGN_S,
        on_lost='hold',
        lock_mode='settle',
    )

    # ── 10. Stable-lock + fire ────────────────────────────────────────────────
    # Holds all axes until FIRE_STABLE_LOCK_S continuous seconds inside FIRE_DEADBAND
    # then triggers duburi.fire(FIRE_CHANNEL) via ESP32 serial.
    # FIRE_DEADBAND is tightest threshold — AUV must be dead-centre to hold stable.
    info(f'Lock-fire: {FIRE_STABLE_LOCK_S:.1f}s stable hold → ESP32 channel {FIRE_CHANNEL}')
    duburi.vision.vision_lock_fire(
        target='hole', camera='forward',
        fire_channel=FIRE_CHANNEL,
        yaw=True, lat=True, depth=True, forward=False,
        stable_lock_s=FIRE_STABLE_LOCK_S,
        max_attempts=FIRE_MAX_ATTEMPTS,
        deadband=FIRE_DEADBAND,
        kp_yaw=60.0,
        kp_lat=60.0,
        kp_depth=0.05,
        speed=FIRE_SPEED,
        h_frac_close=FIRE_H_FRAC_CLOSE,
        duration=60.0,
    )

    # ── 11. Done ──────────────────────────────────────────────────────────────
    info('Torpedo practice complete. Disarming.')
    duburi.pause_detector('forward')
    duburi.stop()
    duburi.disarm()
