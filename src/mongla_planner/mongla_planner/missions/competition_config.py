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

# ── Bin (DOWNWARD camera: image-X→lat Ch6, image-Y→surge Ch5, fill→depth) ────────
BIN_CENTRE_ERR_PX     = 30      # how tightly to centre over the bin before dropping
BIN_SURGE_SIGN        = +1      # NOTE: no longer read by task_bin -- the downward Ch5
                                # fore/aft polarity is now the PERMANENT `vision.surge_sign`
                                # deck default (-1) in mongla_manager/vision_tunables.py, so
                                # missions omit surge_sign entirely. Kept here for reference;
                                # to change the mount polarity edit vision.surge_sign (or set
                                # it live: `ros2 param set /mongla_manager vision.surge_sign -1`).
                                # VERIFY DISARMED: vision_thrust_check --camera downward.
BIN_DESCEND_FILL      = 0       # optional: descend until the bin fills this % (height) for a
                                # closer drop. 0 = OFF (hold BIN_DEPTH_M; core path). When >0,
                                # bounded by BIN_MAX_DEPTH_M so it can't drive into the floor.
BIN_MAX_DEPTH_M       = -1.6    # deepest allowed setpoint for BIN_DESCEND_FILL (negative m)
BIN_DEPTH_CEILING_M   = -0.4    # SHALLOWEST setpoint on the downward align -- surface guard so
                                # ratio/alignment can't lift the hull out of the water (negative m).
BIN_DROPPER_CHANNEL   = 3       # 3 = dropper_1, 4 = dropper_2 (marker drop)

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
TORPEDO_STANDOFF_HOLD_S = 4.0   # seconds to station-keep (forward+lat+depth) while firing
TORPEDO_FIRE_T         = 1.0    # seconds into the hold to fire (must be < STANDOFF_HOLD_S;
                                # leaves ~3 s for a CH340-reconnect-delayed shot to still land)

# ── Search behaviour (mission-authored fallback functions) ──────────────────────
SEARCH_FORWARD_GAIN   = 40      # % gain for forward creep between detections
SEARCH_CREEP_S        = 0.6     # seconds of forward creep per fallback cycle
SEARCH_YAW_STEP_DEG   = 20      # yaw step per sweep look

# ── Style roll ────────────────────────────────────────────────────────────────
STYLE_ROLL_HEADROOM_M = 0.4     # depth below surface for ACRO roll clearance
STYLE_ROLL_GAIN       = 60      # roll speed gain

# ══════════════════════════════════════════════════════════════════════════════
#  SAUVC 2026 — everything above this line is RoboSub. Kept in one file so the
#  operator edits one file at the pool, but the two competitions share NO
#  constant: a SAUVC pool is 25x16 m with a SLOPING 1.6/1.2 m floor and a
#  different rulebook, so a value tuned for one is wrong for the other.
#  Source for every number below: https://sauvc.org/rulebook/ (fetched
#  2026-09-09) and sim/.../spec/sauvc.yaml, which cites the same rulebook.
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
# The SAUVC floor SLOPES: 1.6 m at the pool centre rising to 1.2 m at both ends.
# Every depth here is checked against the SHALLOW end (1.2 m), not the centre,
# because a setpoint that clears the centre still grounds at the ends -- and
# "touching the bottom of the pool or wall" is -5 points PER OCCURRENCE, the
# harshest penalty in the rulebook.
SAUVC_MAX_DEPTH_M       = -1.0   # floor guard: 0.2 m of water under the hull at
                                 # the SHALLOWEST point in the pool. No SAUVC
                                 # setpoint may go deeper (guard test pins this).
SAUVC_SEARCH_DEPTH_M    = -0.6   # transit/search depth while the gate is not yet
                                 # located -- shallow, so a heading error that
                                 # walks us to a pool end cannot ground us.
# The Navigation gate is "150cm wide and 100cm tall", standing ON the bottom
# "approximately 16m away from the starting zone". At 16 m along a 25 m pool the
# floor interpolates to about 1.49 m, so the opening runs from the floor up to
# roughly 0.49 m below the surface and its centre is near 1.0 m.
# We pass ABOVE that centre deliberately: touching the gate costs -2, touching
# the bottom costs -5, so the asymmetry says bias high. -0.85 m leaves ~0.36 m
# under the top bar and ~0.64 m over the floor.
SAUVC_GATE_PASS_DEPTH_M = -0.85

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
SAUVC_GATE_MOVE_S       = 90.0   # ~16 m of transit plus the commit window

# ── Blind transit: the Selector[precise, always_act] terminal branch ──────────
# Navigation is MANDATORY and gates every other task, so a perception miss must
# not zero the run. If the gate is never locked, dead-reckon the leg the rulebook
# itself gives us: "approximately 16m away from the starting zone".
SAUVC_BLIND_TRANSIT_ENABLED = True
SAUVC_BLIND_TRANSIT_S       = 45.0  # ⚠ UNMEASURED on this hull. 16 m at ~0.35 m/s.
                                    # Measure a timed straight leg in the pool and
                                    # correct this before trusting the blind branch.
SAUVC_BLIND_GAIN            = 40

# ── Pre-arm ───────────────────────────────────────────────────────────────────
SAUVC_TETHER_PAUSE_S    = 10.0   # window to pull the tether before thrusters arm

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
                                  # floor and are 0.30 m deep, so their mouths are
                                  # ~0.25 m off the bottom; this keeps the hull
                                  # inside SAUVC_MAX_DEPTH_M with the mouth in
                                  # clear view of the downward camera.
SAUVC_DRUM_CENTRE_ERR_PX = 30     # how tightly to centre over the 0.60 m mouth
SAUVC_DRUM_ALIGN_S       = 60.0   # budget for the downward centring
SAUVC_DRUM_SETTLE_S      = 3.0    # settle over the drum before releasing
SAUVC_DRUM_DESCEND_FILL  = 0      # 0 = OFF: hold the hover depth. A fill-driven
                                  # descent needs the DOWNWARD camera's in-water
                                  # FOV, which is not measured on this hull -- a
                                  # guessed FOV here descends by an unknown amount
                                  # toward a -5 bottom touch. Measure first.
SAUVC_DRUM_DEPTH_CEILING_M = -0.4 # surface guard on the downward align
SAUVC_DROPPER_CHANNEL    = 3      # 3 = dropper_1, 4 = dropper_2

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
