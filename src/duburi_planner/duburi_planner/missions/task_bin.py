"""Bin drop task — DOWNWARD camera, hover over the bin, drop a marker.

RoboSub Task 3 (Recon/Bins). The AUV flies ABOVE the bin looking straight down,
so the downward-camera frame rotates the body axes and the align() KWARGS remap
(gated on ``camera='downward'``; full table: .claude/context/downward-camera.md):

    image-X  -> Ch6 LATERAL strafe   (left/right over the bin)   -- align ``lat``
    image-Y  -> Ch5 SURGE fore/aft   (forward/back over the bin) -- align ``fwd``
    bbox fill-> DEPTH descent        (get closer for the drop)   -- align ``depth`` (+fwd_mode)
    fire     -> DROPPER (3/4)

So ``align('fire', lat=0, fwd=0, depth=<fill%>)`` centres the hull over the bin in
BOTH horizontal axes (``lat`` = left/right Ch6, ``fwd`` = fore/aft Ch5 SURGE, two-
sided + braked) and DESCENDS to the fill target on ``depth``, while ArduSub holds
``BIN_DEPTH_M`` on Ch3. (``fwd`` is always the fore/aft joystick, ``depth`` always
the real depth axis -- the forward-cam ``fwd``/``depth`` meanings swap here.) The
verb auto-switches the live detector to 'downward' (pausing forward -- one detector
at a time on the Jetson) and flips the HUD to the downward view.

★ Before an armed run, VERIFY the surge sign DISARMED:
    ros2 run duburi_vision vision_thrust_check --camera downward
  A bin AHEAD in the image must drive the hull FORWARD (Ch5>1500). If it reverses,
  set BIN_SURGE_SIGN = -1 in competition_config.py. A wrong sign is positive feedback.

Standalone test (bin_fire_blood.pt — classes: blood(0) fire(1)):
    ros2 run duburi_planner mission task_bin
"""

from duburi_planner.missions.competition_config import (
    BIN_HEADING_DEG,
    BIN_DEPTH_M,
    BIN_CENTRE_ERR_PX,
    BIN_DESCEND_FILL,
    BIN_MAX_DEPTH_M,
    BIN_DEPTH_CEILING_M,
    BIN_DROPPER_CHANNEL,
    ALIGN_GAIN,
    SEARCH_FORWARD_GAIN,
    SEARCH_CREEP_S,
)

_DWN = '/duburi_detector_downward'


def run(duburi, log=None):
    duburi.mission_reset()
    if BIN_HEADING_DEG is not None:
        duburi.turn(BIN_HEADING_DEG)
    duburi.set_depth(BIN_DEPTH_M, timeout=30)

    # ⛔ THE EYE THIS TASK NEEDS MAY NOT BE THERE. A camera that fails to open
    # no longer takes the vision stack down with it -- the other camera keeps
    # running and this one's detector is simply absent. Calling a vision verb
    # anyway makes `_ensure_detector` abort the WHOLE run over one dead camera,
    # taking every later task with it. Skip this task instead and let the rest
    # of the mission score.
    if not duburi.camera_available('downward'):
        if log:
            log('[bin_task] downward camera absent — SKIPPING the bin drop. '
                'Every other task in the run still scores.')
        return

    # Point everything at the downward camera. use_camera() auto-switches the live
    # detector (pauses forward, resumes downward) + flips the HUD; set the model +
    # classes for the bin task on that node.
    duburi.use_camera('downward')
    # ── Centre the AUV over the bin, then drop ────────────────────────────────
    # DOWNWARD kwargs (see downward-camera.md): lat = left/right (Ch6),
    # fwd = fore/aft SURGE (Ch5, image-Y, two-sided + braked), depth = DESCENT to a
    # bbox fill %% (BIN_DESCEND_FILL, measured by fwd_mode='height'). ArduSub holds
    # BIN_DEPTH_M on Ch3; the descent (bounded by the floor set below) gets closer
    # for the drop. Creep-search finds the bin on target loss.
    #
    # The downward setup (model/classes + the depth-bound tunables) lives INSIDE the
    # try so that if any of it raises, the finally still restores the forward camera
    # + forward-safe depth defaults -- never leave the downward detector live or the
    # floor/ceiling clamping a later forward task.
    try:
        duburi.set_model('bin_fire_blood', node=_DWN)
        duburi.set_classes('fire,blood', node=_DWN)
        # Depth bounds are PER-MISSION now (vision tunables), not per-align kwargs:
        # set the descent floor + surface guard ONCE here. `surge_sign` is the
        # permanent vision.surge_sign default (-1) so the align below omits it.
        duburi.set_vision_param('max_depth_m', BIN_MAX_DEPTH_M)       # floor + enables descent
        duburi.set_vision_param('depth_ceiling', BIN_DEPTH_CEILING_M)  # surface guard

        aligned = duburi.vision.align(
            'fire', camera='downward', lat=0, fwd=0,          # lat+surge centre over bin
            depth=(BIN_DESCEND_FILL or None), fwd_mode='height',  # descend to fill%
            err=BIN_CENTRE_ERR_PX, gain=ALIGN_GAIN, duration=25,
            fallback=creep_forward)

        # Drop ONLY when centred -- a blind drop wastes the marker into empty water.
        if aligned:
            duburi.pause(3.0)                   # settle over the bin before the drop
            duburi.fire(BIN_DROPPER_CHANNEL)    # dropper — channel always explicit
            duburi.pause(2.0)                   # confirm drop complete
        elif log:
            log('[bin_task] never centred over the bin — marker HELD (no blind drop); '
                'check the downward detector resumed and vision.surge_sign')
    finally:
        # Restore the forward camera AND the forward-safe depth-bound defaults, so a
        # later FORWARD task in the same session (e.g. the torpedo standoff) is never
        # clamped by this bin run's floor/ceiling. Best-effort: never mask the exit.
        for _p in ('max_depth_m', 'depth_ceiling'):
            try:
                duburi.set_vision_param(_p, 0.0)
            except Exception:   # noqa: BLE001 -- reset is best-effort
                pass
        duburi.use_camera('forward')


# ── Mission-authored fallback search patterns (pure control) ────────────────────
def creep_forward(duburi):
    """One short forward creep, then return so the vision loop retries."""
    duburi.move_forward(SEARCH_CREEP_S, gain=SEARCH_FORWARD_GAIN)
