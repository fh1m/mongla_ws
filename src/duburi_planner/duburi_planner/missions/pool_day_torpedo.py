"""Pool day torpedo practice: align → approach → fine-lock → fire.

Two-verb vision:
  1. Arm + depth → short launch drive into starting range
  2. align() on 'torpedo' board (yaw + lat + depth) — creep fallback finds it
  3. move() forward on 'blood' until it fills the frame (height metric)
  4. align() a tight lock on 'hole' (small err, slow) → fire on a confirmed lock

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

# ── ALIGNMENT (pixels) + SPEED CAPS (% thrust) ────────────────────────────────
BOARD_ERR_PX           = 40     # coarse board centring tolerance
HOLE_ERR_PX            = 14     # tight tolerance for the fire lock
BOARD_GAIN             = 30     # max speed while centring on the board
APPROACH_GAIN          = 35     # max speed while driving in on blood
FINE_GAIN              = 12     # slow + precise for the hole lock

# ── FILL TARGET (% of frame the blood bbox fills to end the approach) ─────────
BLOOD_FWD_FILL         = 30     # blood height % of frame at end of approach

# ── TIME BUDGETS (seconds) ────────────────────────────────────────────────────
BOARD_ALIGN_S          = 20.0
BLOOD_APPROACH_S       = 35.0
HOLE_LOCK_S            = 30.0

# ── FIRE ─────────────────────────────────────────────────────────────────────
FIRE_CHANNEL           = 1      # 1=torpedo_1  2=torpedo_2  (always explicit)

# ── SEARCH (mission-authored fallback) ────────────────────────────────────────
SEARCH_FORWARD_GAIN    = 40     # % thrust per creep
SEARCH_CREEP_S         = 0.8    # seconds of forward creep per fallback cycle

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

    if TORPEDO_HEADING_DEG is not None:
        info(f'Turning to heading {TORPEDO_HEADING_DEG}°')
        duburi.turn(TORPEDO_HEADING_DEG)

    # ── 2. Launch: short forward drive to starting range ──────────────────────
    info(f'Launch drive: {LAUNCH_FORWARD_S:.0f}s forward at {LAUNCH_FORWARD_GAIN}% gain')
    duburi.move_forward(LAUNCH_FORWARD_S, gain=LAUNCH_FORWARD_GAIN)

    duburi.resume_detector('forward')
    duburi.set_model('torpedo_blood_hole', node=_FWD)
    duburi.set_classes('torpedo,blood,hole', node=_FWD)

    # ── 3. Coarse align on the board (yaw + lat + depth) ──────────────────────
    info('Coarse align on torpedo board ...')
    duburi.vision.align(
        'torpedo', camera='forward', yaw=0, lat=0, depth=0,
        err=BOARD_ERR_PX, gain=BOARD_GAIN, duration=BOARD_ALIGN_S,
        fallback=creep_forward)

    # ── 4. Blood approach: drive forward until blood fills the frame ──────────
    info('Approach: drive in on blood marker ...')
    duburi.set_classes('blood,hole', node=_FWD)
    duburi.vision.move(
        'blood', camera='forward', fwd=BLOOD_FWD_FILL, mode='height',
        gain=APPROACH_GAIN, duration=BLOOD_APPROACH_S, fallback=creep_forward)

    # ── 5. Fine hole lock, then fire on a confirmed lock ──────────────────────
    info('Fine lock on hole ...')
    duburi.set_classes('hole', node=_FWD)
    locked = duburi.vision.align(
        'hole', camera='forward', yaw=0, lat=0, depth=0,
        err=HOLE_ERR_PX, gain=FINE_GAIN, duration=HOLE_LOCK_S,
        fallback=creep_forward)
    if locked:
        info(f'Locked — firing ESP32 channel {FIRE_CHANNEL}')
        duburi.fire(FIRE_CHANNEL)
    else:
        info('Hole never locked — holding fire')

    # ── 6. Done ───────────────────────────────────────────────────────────────
    info('Torpedo practice complete. Disarming.')
    duburi.pause(2.0)
    duburi.pause_detector('forward')
    duburi.stop()
    duburi.disarm()


# ── Mission-authored fallback search patterns (pure control) ────────────────────
def creep_forward(duburi):
    """One short forward creep, then return so the vision loop retries."""
    duburi.move_forward(SEARCH_CREEP_S, gain=SEARCH_FORWARD_GAIN)
