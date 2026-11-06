"""Competition pool-day constants for RoboSub 2026.

Edit this file at the pool before a run — no colcon build needed (hot-reload applies).
None values = fill at pool after compass survey and depth sighting.

All vision tuning here is in the two-verb pixel-native vocabulary:
  * ``*_ERR_PX``   -- pixel tolerance for vision.align (aligned when |err| <= this)
  * ``*_FWD_FILL`` -- % of frame the bbox must fill for vision.move to stop
  * ``*_GAIN``     -- hard max-speed cap (% thrust) for that phase
  * ``*_OFFSET_PX``-- signed lateral pixel offset to hold
"""

# ── Depths ────────────────────────────────────────────────────────────────────
GATE_SEARCH_DEPTH_M   = -0.4    # initial descent depth for whole run
GATE_PASS_DEPTH_M     = -0.6    # depth to pass through gate opening
BIN_DEPTH_M           = -1.0    # downward camera must see bin clearly
TORPEDO_DEPTH_M       = None    # align with hole center; fill at pool

# ── Headings (degrees) — fill after compass survey ─────────────────────────────
# ⚠ SROT YAW SENSE CHANGED AT FIRMWARE REV 10 (2026-08-07). Before rev 10 an
# improper axis transform left the frame left-handed and yaw SILENTLY INVERTED
# while roll and pitch read correctly -- turning right made yaw DECREASE, measured
# in water. Any heading written down against a rev <= 9 board is NEGATED on a rev
# >= 10 board. These four are still None (nothing recorded, so nothing to
# convert), but check the board's rev before transcribing a heading from an older
# pool-day note. `SrotFC.read_behaviour_rev()` reports it.
SLALOM_HEADING_DEG    = None    # compass heading to slalom course
BIN_HEADING_DEG       = None    # compass heading to bin
TORPEDO_HEADING_DEG   = None    # compass heading to torpedo board
RETURN_HEADING_DEG    = None    # compass heading back to gate

# ── Alignment tolerances (pixels) ──────────────────────────────────────────────
ALIGN_ERR_PX          = 40      # default "centred" tolerance
FINE_ERR_PX           = 14      # tight tolerance for the torpedo hole / fire lock
ALIGN_GAIN            = 30      # max speed during centring
APPROACH_GAIN         = 35      # max speed while driving forward

# ── Gate ───────────────────────────────────────────────────────────────────────
GATE_PASS_FWD_FILL    = 80      # gate bbox height % of frame that means "through gate"

# ── Slalom ────────────────────────────────────────────────────────────────────
SLALOM_PIPE_OFFSET_PX = 80      # lateral pixel offset from pipe centre (positive=right)
SLALOM_FWD_FILL       = 60      # pipe bbox height % of frame at closest pass

# ── Bin (DOWNWARD camera: image-X → lateral, image-Y → surge) ─────────────────
BIN_CENTRE_ERR_PX     = 30      # how tightly to centre over the bin before dropping
BIN_DROPPER_CHANNEL   = None    # set below, from DROPPER_1_CHANNEL (B53)

# ── Degradation policy -- CONFIGURED, not coded (plan Block 1D) ──────────────────
#
# What a task does when the camera it steers on is ABSENT. BumblebeeAS ship
# `claw_required: false` -- "degrades to front-only if absent" -- as a config
# value, so the pool-side answer is an edit to one table, not to every mission.
#
#   True   the run cannot continue without it: ABORT loudly
#   False  SKIP the task that needs it, log why, and let every other task score
#
# Read by `mongla.camera_usable(name, task=...)` at each task's entry. Before
# this, only task_bin asked; sauvc_target_acquisition, pool_day_practice and
# demo_dual_camera called the vision verb anyway, and `_ensure_detector` aborted
# the WHOLE run over one dead downward USB camera.
CAMERA_REQUIRED = {
    'forward':  True,     # the gate, the flare, the torpedo board: nothing scores blind
    'downward': False,    # the bin / drum drop is one task; lose it, keep the run
}

# ── Payload channels -- ONE place, and UNASSIGNED (B53) ──────────────────────────
#
# ⛔ 0 = NOT ASSIGNED. The old map (1,2 = torpedoes, 3,4 = droppers) was invented:
# read live off the board, 2026-09-22 and again 2026-10-01, those channels are
# SERVO (the on-board arm) or unroled, and `fire()` refuses every one of them --
# so every torpedo and every drop would have scored zero while the mission ran.
#
#     FIREABLE (SWITCH)   9, 10, 12, 13, 14, 15, 16
#     SERVO (the arm)     1, 2, 3, 5, 6, 7, 8
#     unroled             4, 11
#
# The roles are FIRMWARE state set in Bondor, and nothing is wired to the
# payload yet. So these stay 0 until the payload is wired to a SWITCH channel
# and confirmed: run `ros2 run mongla_manager connect`, take the channel from
# its FIREABLE list, and set it HERE -- every mission reads these names.
# `fire(0)` is refused with a reason that says exactly this; it is never a
# silent no-op, and no mission may hard-code a channel number
# (`test_missions_never_hard_code_a_payload_channel.py`).
TORPEDO_1_CHANNEL     = 0
TORPEDO_2_CHANNEL     = 0
DROPPER_1_CHANNEL     = 0
DROPPER_2_CHANNEL     = 0

# ── Payload fire ─────────────────────────────────────────────────────────────────
FIRE_GAP_S            = 1.0     # seconds BETWEEN channels when firing a LIST (fire=[1,4]/[2,3]).
                                # The solenoid launcher misfires if two go together -- space them.

# ── Torpedo ──────────────────────────────────────────────────────────────────────
TORPEDO_BLOOD_FWD_FILL = 30     # blood bbox height % at end of COARSE approach (gets in range)
# Firing STANDOFF: the terminal align('hole', fwd=…) drives forward to this hole-height
# fill and HOLDS it while firing. RoboSub awards bonus points for firing FURTHER from the
# board (far 0.3m / farther 0.46m) AND a large+stable bbox at standoff is far easier to
# hold steady than point-blank -- so tune this so the hull parks ~0.3-0.46m off the board
# (smaller fill = further back). Read the live `[ align … fwd>=…% ]` line to calibrate.
TORPEDO_STANDOFF_FILL  = 35     # hole bbox height % of frame at the firing standoff
TORPEDO_STANDOFF_HOLD_S = 4.0   # seconds to station-keep (forward+lat) while firing
TORPEDO_FIRE_T         = 1.0    # seconds into the hold to fire (must be < STANDOFF_HOLD_S;
                                # leaves ~3 s of hold after the shot)

# ── Search behaviour (mission-authored fallback functions) ──────────────────────
SEARCH_FORWARD_GAIN   = 40      # % gain for forward creep between detections
SEARCH_CREEP_S        = 0.6     # seconds of forward creep per fallback cycle
SEARCH_YAW_STEP_DEG   = 20      # yaw step per sweep look

# ══════════════════════════════════════════════════════════════════════════════
#  SAUVC — everything above this line is RoboSub. Kept in one file so the
#  operator edits one file at the pool, but the two competitions share NO
#  constant: a different pool and a different rulebook, so a value tuned for
#  one is wrong for the other.
#  Source for every number below: https://sauvc.org/rulebook/ as published
#  30 Oct 2026 (re-read 2026-11-05). ⛔ The venue CHANGED between rulebooks:
#  the 2026-09-09 read described a one-off 25x16 m pool with a 1.6/1.2 m floor
#  and a 1.0 m gate 16 m out; the current one is an olympic 50x25 m pool whose
#  floor runs 1.28 m at the qualification wall, slopes to 2 m by 17.5 m, and is
#  2 m across most of the main arena, with a 1.5 m gate ~12 m out. Every depth
#  below was re-derived from the current figure (`courses/sauvc_olympic.yaml`).
# ══════════════════════════════════════════════════════════════════════════════

# ── Run budget ────────────────────────────────────────────────────────────────
# "Each team is given 15 minutes to complete the tasks." That 900 s covers ALL
# attempts AND setup, and the bonus is `(900 - RUN_TIME) * 0.03` once at least
# two tasks are done -- so a 300 s run is worth 18 bonus points, MORE than the
# 15-point Navigation task itself. Elapsed time is a scored resource, not slack.
# ⛔ The bonus needs TWO tasks. As shipped, `sauvc_navigation.run()` surfaces and
# disarms at the end, so a navigation-only run scores 20 (15 + 5 surfacing) and
# earns NO timing bonus. Chaining navigation into target acquisition on one dive
# is what makes the 18 points real, and that combinator does not exist yet.
SAUVC_RUN_BUDGET_S      = 900.0
# Navigation is one of the tasks; spending a third of the run on it is the
# ceiling, not the plan. test_sauvc_navigation.py asserts the mission's own
# `duration=` values sum under this.
SAUVC_NAV_BUDGET_S      = 300.0

# ── Depths (negative = below surface) ─────────────────────────────────────────
# The floor SLOPES: 1.28 m for the first 1.8 m off the qualification wall,
# down to 2 m at 17.5 m, flat 2 m beyond. The main arena starts at 10 m, where
# the slope puts the floor at ~1.66 m -- and the new Surfacing Square floats
# right there, so a finals run DOES visit the shallowest main-arena water.
# Every finals depth is checked against 1.57 m (1.66 less the rulebook's ±5 %
# margin on field dimensions), not the 2 m most of the arena has, because
# "touching the bottom of the pool or wall" is -5 points PER OCCURRENCE, the
# harshest penalty in the rulebook. Qualification runs in shallower water and
# has its own guard below.
SAUVC_MAX_DEPTH_M       = -1.35  # floor guard: 0.2 m of water under the hull at
                                 # the SHALLOWEST main-arena point. No finals
                                 # setpoint may go deeper (guard test pins this).
SAUVC_SEARCH_DEPTH_M    = -0.6   # transit/search depth while the gate is not yet
                                 # located -- shallow, so a heading error that
                                 # walks us toward the slope cannot ground us.
# The Navigation gate is "150cm wide and 150cm tall", standing on the bottom on
# a line "approximately 12m away from the starting zone", in the flat part of
# the main arena. With the floor at 1.9 m there (2 m less the ±5 % margin) the
# opening runs from ~0.4 m below the surface to the floor; its centre is ~1.15 m.
# We pass ABOVE that centre deliberately: touching the gate costs -2 (and no
# longer fails the task), touching the bottom costs -5, so the asymmetry says
# bias high. -1.0 m leaves ~0.5 m under the top bar and ~0.8 m over the floor
# for a 0.17 m tall hull. (Was -0.85 for the old venue's 1.0 m gate.)
SAUVC_GATE_PASS_DEPTH_M = -1.0

# ── Vision tuning ─────────────────────────────────────────────────────────────
SAUVC_ALIGN_ERR_PX      = 40     # "centred" tolerance on the gate
SAUVC_ALIGN_GAIN        = 30     # max % thrust while centring
SAUVC_APPROACH_GAIN     = 40     # max % thrust on the transit; higher than the
                                 # RoboSub 35 because the timing bonus pays for it
                                 # and the gate opening is 1.5 m wide, not a hole.
# Pass-through commit: vision.move(fwd=None) drives until the gate LEAVES the
# frame and then keeps driving for this long to physically clear it. The default
# is 2.0 s; 4.0 s here because failing to pass forfeits the mandatory task and
# with it every other point in the run, while clipping the gate costs only -2.
SAUVC_GATE_COMMIT_S     = 4.0

# ── Phase budgets (seconds) — these are what the guard test sums ──────────────
SAUVC_GATE_ALIGN_S      = 60.0   # centre on the gate, creeping forward to find it
SAUVC_GATE_MOVE_S       = 90.0   # ~12 m of transit plus the commit window

# ── Blind transit: the Selector[precise, always_act] terminal branch ──────────
# Navigation is MANDATORY and gates every other task, so a perception miss must
# not zero the run. If the gate is never locked, dead-reckon the leg the rulebook
# itself gives us: "approximately 12m away from the starting zone".
SAUVC_BLIND_TRANSIT_ENABLED = True
SAUVC_BLIND_TRANSIT_S       = 40.0  # ⚠ UNMEASURED on this hull. 12 m at ~0.35 m/s
                                    # is ~34 s; 40 s carries the hull ~2 m past
                                    # the gate line so the back-most part clears.
                                    # Measure a timed straight leg in the pool and
                                    # correct this before trusting the blind branch.
SAUVC_BLIND_GAIN            = 40

# ── Pre-arm ───────────────────────────────────────────────────────────────────
SAUVC_TETHER_PAUSE_S    = 10.0   # window to pull the tether before thrusters arm

# ── SAUVC Qualification (its own run, in its own lane) ─────────────────────────
# "To qualify, an AUV must swim from the qualification starting line, pass
#  completely through the qualification gate, perform a U-turn, and completely
#  pass through the gate again." 1 point for the forward pass, 2 only after
# both. "The AUV must complete the entire course without surfacing, touching
# the bottom, walls, or the qualification gate" -- and touching ANY of the
# gate's 3 sides fails the attempt. Only the top 15 qualifiers reach the finals,
# so this run gates every finals point the way Navigation gates every task.
#
# The gate is "150cm wide and ~100cm deep ... hanging from the water surface":
# the opening is the top ~1 m of water, bounded by two posts and a bottom bar.
# Surfacing and a bar touch both FAIL, so unlike the finals gate nothing argues
# for a bias: centre the hull in the 0.95 m worst-case opening.
SAUVC_QUAL_PASS_DEPTH_M  = -0.5   # hull 0.17 m tall: ~0.41 m to the surface,
                                  # ~0.36 m to the bottom bar at its shallowest
# The lane's floor is the shallowest water in the pool: 1.28 m off the wall
# (1.22 m with the ±5 % margin), sloping deeper toward the gate.
SAUVC_QUAL_MAX_DEPTH_M   = -1.0   # 0.2 m under the hull at the wall
SAUVC_QUAL_GATE_ALIGN_S  = 60.0   # centre on the gate, creeping forward to find it
SAUVC_QUAL_GATE_MOVE_S   = 75.0   # ~10 m out, plus the commit window
# After the forward pass the hull is barely through. A 1.5 m gate overfills a
# 46.7° in-water frame inside ~2.5 m (1.5 / (2 tan 23.35°) / 0.7), so turning
# in place leaves an align with no whole gate to centre on. Swim on first.
SAUVC_QUAL_STANDOFF_S    = 8.0    # ⚠ UNMEASURED. ~2.8 m at ~0.35 m/s
SAUVC_QUAL_TURN_DEG      = 180.0  # yaw_right: CLOCKWISE, which is also the turn
                                  # the qualification video must show
SAUVC_QUAL_TURN_TIMEOUT_S = 30.0
# Selector[precise, always_act], as in Navigation: a perception miss must not
# zero the qualifier. The rulebook gives the outbound leg ("approximately 10m");
# the return leg is the standoff plus the commit distance, ~4.5 m.
SAUVC_QUAL_BLIND_ENABLED = True
SAUVC_QUAL_BLIND_OUT_S   = 32.0   # ⚠ UNMEASURED. 10 m at ~0.35 m/s is ~29 s
SAUVC_QUAL_BLIND_BACK_S  = 14.0   # ⚠ UNMEASURED. ~4.5 m at ~0.35 m/s is ~13 s
# Ceiling for the whole run. Qualification time is a tie-breaker only, so this
# is a bound on a stuck mission, not a score.
SAUVC_QUAL_BUDGET_S      = 420.0

# ── SAUVC Task 2: Target Acquisition (drop a ball into a drum) ────────────────
# "There are 4 colored drums in the arena. One of the drums, chosen at random,
#  will be blue in color, while the rest are red in color." -- "60cm in diameter
#  and 30cm in depth."  Blue 30 pts, red-with-pinger 50, other red 10.
#
# ⛔ WE TARGET THE BLUE DRUM, AND ONLY THE BLUE DRUM. The 50-point drum is
# identified ACOUSTICALLY ("RJE International Pinger Model No. ULB-362B/45 kHz");
# the rulebook describes no visual marking at all, and no hydrophone is fitted to
# this vehicle. The `drum_red_pinger` class exists in sauvc_sim.yaml only because
# the SIMULATOR paints a yellow band on that drum -- an artifact with no real-
# world counterpart (see target_geometry.yaml). Steering on it would put the ball
# in a 10-point drum while believing it scored 50. Blue is the one drum a camera
# can actually tell apart, so blue is the target.
SAUVC_DRUM_CLASS         = 'drum_blue'
SAUVC_DRUM_HOVER_DEPTH_M = -0.7   # hover height for the drop. Drums stand on the
                                  # 2 m floor and are 0.30 m deep, so their mouths
                                  # are ~1.7 m down and this hovers ~1.0 m above
                                  # them. ⚠ Kept from the old venue (mouths ~0.9 m
                                  # down then): a 1 m ball drop is a longer fall
                                  # than it was. Lowering it needs the downward
                                  # camera's in-water footprint at that height --
                                  # measure before moving it.
SAUVC_DRUM_CENTRE_ERR_PX = 30     # how tightly to centre over the 0.60 m mouth
SAUVC_DRUM_ALIGN_S       = 60.0   # budget for the downward centring
SAUVC_DRUM_SETTLE_S      = 3.0    # settle over the drum before releasing
SAUVC_DROPPER_CHANNEL    = None   # set below, from DROPPER_1_CHANNEL (B53)

# ── SAUVC combinator (sauvc_full) ─────────────────────────────────────────────
SAUVC_RESERVE_S          = 45.0   # surface + disarm allowance, never offered to a task
SAUVC_DRUM_WORST_S       = 90.0   # hover dive + downward align + settle + release

# Task 4, Communication & Localization: 20 per bump flare, +60 for all three in
# the order sent over LoRa. ⛔ OFF by default. A 16 mm pole at the 10 px floor is
# detectable at <= ~0.9 m on the forward camera (measured-bars.md, P6 DATA OPEN),
# so an align on a flare from transit is expected NOT to acquire. Enable after
# the pool measurement, not before.
SAUVC_FLARES_ENABLED     = False
SAUVC_FLARE_LISTEN_S     = 45.0   # hold for the LoRa order; None -> default order
SAUVC_FLARE_DEFAULT_ORDER = ('red', 'yellow', 'blue')   # bump-all when no order
SAUVC_FLARE_ALIGN_S      = 40.0   # ⚠ UNMEASURED
SAUVC_FLARE_MOVE_S       = 30.0   # ⚠ UNMEASURED
SAUVC_FLARE_STOP_FILL    = 85     # % of frame HEIGHT: a 0.8 m pole fills it at contact range
SAUVC_FLARE_GAIN         = 25     # slow: a bump, not a ram
SAUVC_FLARE_PUSH_S       = 2.0    # ⚠ UNMEASURED short push to tip the ball
SAUVC_FLARE_BACKOFF_S    = 3.0    # ⚠ UNMEASURED
SAUVC_FLARES_WORST_S     = (SAUVC_FLARE_LISTEN_S + 3 * (
    SAUVC_FLARE_ALIGN_S + SAUVC_FLARE_MOVE_S + SAUVC_FLARE_PUSH_S + SAUVC_FLARE_BACKOFF_S))

# ⛔ Task 3 (Target Reacquisition, 60 pts) is NOT implemented and has no config
# here. "The AUV has to hold on to the ball till the end of attempt" -- that needs
# a gripper or a retaining mechanism this vehicle does not carry. Recording the
# absence so it is not mistaken for an oversight.

# The droppers the bin and SAUVC tasks use -- aliases, never second copies.
BIN_DROPPER_CHANNEL = DROPPER_1_CHANNEL
SAUVC_DROPPER_CHANNEL = DROPPER_1_CHANNEL
