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
    BIN_SURGE_SIGN,
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

    # Point everything at the downward camera. use_camera() auto-switches the live
    # detector (pauses forward, resumes downward) + flips the HUD; set the model +
    # classes for the bin task on that node.
    duburi.use_camera('downward')
    duburi.set_model('bin_fire_blood', node=_DWN)
    duburi.set_classes('fire,blood', node=_DWN)

    # ── Centre the AUV over the bin, then drop ────────────────────────────────
    # DOWNWARD kwargs (see downward-camera.md): lat = left/right (Ch6),
    # fwd = fore/aft SURGE (Ch5, image-Y, two-sided + braked), depth = DESCENT to a
    # bbox fill %% (BIN_DESCEND_FILL, measured by fwd_mode='height'). ArduSub holds
    # BIN_DEPTH_M on Ch3; the descent (bounded by BIN_MAX_DEPTH_M) gets closer for
    # the drop. Creep-search finds the bin on target loss.
    try:
        aligned = duburi.vision.align(
            'fire', camera='downward', lat=0, fwd=0,          # lat+surge centre over bin
            depth=(BIN_DESCEND_FILL or None), fwd_mode='height',  # descend to fill%
            err=BIN_CENTRE_ERR_PX, gain=ALIGN_GAIN, duration=25,
            surge_sign=BIN_SURGE_SIGN, max_depth_m=BIN_MAX_DEPTH_M,
            depth_ceiling=BIN_DEPTH_CEILING_M,   # never surface during alignment
            fallback=creep_forward)

        # Drop ONLY when centred -- a blind drop wastes the marker into empty water.
        if aligned:
            duburi.pause(3.0)                   # settle over the bin before the drop
            duburi.fire(BIN_DROPPER_CHANNEL)    # dropper — channel always explicit
            duburi.pause(2.0)                   # confirm drop complete
        elif log:
            log('[bin_task] never centred over the bin — marker HELD (no blind drop); '
                'check the downward detector resumed and BIN_SURGE_SIGN')
    finally:
        # Always restore the forward camera (pauses the downward detector) even if a
        # fire/pause above raised -- never leave the downward detector live.
        duburi.use_camera('forward')


# ── Mission-authored fallback search patterns (pure control) ────────────────────
def creep_forward(duburi):
    """One short forward creep, then return so the vision loop retries."""
    duburi.move_forward(SEARCH_CREEP_S, gain=SEARCH_FORWARD_GAIN)
