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
(The coarse board align uses a per-call settle= to exit squared-up; the terminal
fire-lock deliberately does NOT -- settle would gate the mid-hold fire.)

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

# ── FILL TARGETS (% of frame the bbox fills) ──────────────────────────────────
BLOOD_FWD_FILL         = 30     # blood height % at end of COARSE approach (gets in range)
# Firing STANDOFF: the terminal verb drives forward to this HOLE-height fill and
# HOLDS it while firing. Smaller = hull parks FURTHER back (RoboSub awards bonus
# points for firing further from the board; a large+stable bbox there is also
# easier to hold than point-blank). Read the live `[ align … fwd>=…% ]` line to
# calibrate it to ~0.3-0.46m off the board.
HOLE_STANDOFF_FILL     = 35     # hole height % of frame at the firing standoff

# ── TIME BUDGETS (seconds) ────────────────────────────────────────────────────
BOARD_ALIGN_S          = 20.0
BLOOD_APPROACH_S       = 35.0
HOLE_LOCK_S            = 30.0
HOLE_HOLD_S            = 4.0    # station-keep (forward+lat+depth) while firing
FIRE_T                 = 1.5    # seconds into the hold to fire (must be < HOLE_HOLD_S)

# ── FIRE ─────────────────────────────────────────────────────────────────────
FIRE_CHANNEL           = 1      # 1=torpedo_1  2=torpedo_2  (always explicit)

# ── SEARCH (mission-authored fallback) ────────────────────────────────────────
SEARCH_FORWARD_GAIN    = 40     # % thrust per creep
SEARCH_CREEP_S         = 0.8    # seconds of forward creep per fallback cycle

_FWD = '/duburi_detector_forward'


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
    # Everything after arm is wrapped so a mid-run exception ALWAYS releases the
    # heading lock and disarms -- a leaked lock would keep driving Ch4, and a
    # leaked arm would keep the thrusters live.
    try:
        duburi.set_depth(TORPEDO_DEPTH_M)
        info(f'Armed. Descending to {TORPEDO_DEPTH_M}m — waiting for depth settle')
        time.sleep(3.0)

        if TORPEDO_HEADING_DEG is not None:
            info(f'Turning to heading {TORPEDO_HEADING_DEG}°')
            duburi.turn(TORPEDO_HEADING_DEG)

        # ── 2. Launch: short forward drive to starting range ──────────────────
        info(f'Launch drive: {LAUNCH_FORWARD_S:.0f}s forward at {LAUNCH_FORWARD_GAIN}% gain')
        duburi.move_forward(LAUNCH_FORWARD_S, gain=LAUNCH_FORWARD_GAIN)

        duburi.resume_detector('forward')
        duburi.set_model('torpedo_blood_hole', node=_FWD)
        duburi.set_classes('torpedo,blood,hole', node=_FWD)

        # ── 3. Coarse align on the board (yaw + lat + depth) ──────────────────
        info('Coarse align on torpedo board ...')
        # settle= -> exit SETTLED (hull stopped) so lock_heading captures a clean heading.
        duburi.vision.align(
            'torpedo', camera='forward', yaw=0, lat=0, depth=0,
            err=BOARD_ERR_PX, gain=BOARD_GAIN, duration=BOARD_ALIGN_S, settle=8,
            fallback=creep_forward)

        # Hand yaw to the background heading lock so the approach + terminal lock
        # hold heading WITHOUT vision-yaw (no close-in yaw limit-cycle).
        duburi.lock_heading(timeout=120)

        # ── 4. Coarse approach: COAST forward until blood fills the frame ──────
        #     brake=False -> no reverse-kick exit; the terminal verb closes the rest.
        info('Approach: coast in on blood marker ...')
        duburi.set_classes('blood,hole', node=_FWD)
        duburi.vision.move(
            'blood', camera='forward', fwd=BLOOD_FWD_FILL, mode='height',
            gain=APPROACH_GAIN, duration=BLOOD_APPROACH_S, brake=False,
            fallback=creep_forward)

        # ── 5. UNIFIED STANDOFF SHOT: forward-standoff + lat/depth + hold + fire ──
        #     ONE verb drives to the firing standoff (fwd=, height), centres lat+depth
        #     (NO yaw -> heading_lock holds Ch4), continuity-locks the hole, and fires
        #     MID-HOLD while still glued AND parked at the standoff. brake=False.
        #     NOTE: the forward-close is capped by the fine gain (FINE_GAIN, 12%) --
        #     no separate forward gain -- so budget HOLE_LOCK_S to reach the standoff,
        #     or raise BLOOD_FWD_FILL so the coast lands closer.
        info('Standoff lock on hole + mid-hold fire ...')
        duburi.set_classes('hole', node=_FWD)
        locked = duburi.vision.align(
            'hole', camera='forward', lat=0, depth=0,          # NO yaw -> heading_lock
            fwd=HOLE_STANDOFF_FILL, fwd_mode='height',          # drive to + hold standoff
            err=HOLE_ERR_PX, gain=FINE_GAIN, duration=HOLE_LOCK_S,
            lock_on=True, hold=HOLE_HOLD_S,
            fire=FIRE_CHANNEL, fire_t=FIRE_T, brake=False,      # mid-hold, while glued
            fallback=creep_forward)
        if locked:
            info(f'Standoff lock held — fired ESP32 channel {FIRE_CHANNEL} mid-hold')
        else:
            info('Hole standoff never locked — fire was withheld (gated on lock)')
    finally:
        # ── 6. Done / safe-stop (runs on any exit, incl. exception) ───────────
        info('Torpedo practice complete. Disarming.')
        duburi.unlock_heading()
        duburi.pause_detector('forward')
        duburi.stop()
        duburi.disarm()


# ── Mission-authored fallback search patterns (pure control) ────────────────────
def creep_forward(duburi):
    """One short forward creep, then return so the vision loop retries."""
    duburi.move_forward(SEARCH_CREEP_S, gain=SEARCH_FORWARD_GAIN)
