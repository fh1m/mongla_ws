"""Bin drop task — DOWNWARD camera, hover over the bin, drop a marker.

RoboSub Task 3 (Recon/Bins). The AUV flies ABOVE the bin looking straight down,
so the downward-camera frame rotates the body axes (handled in the align engine,
gated on ``camera='downward'``):

    image-X  -> Ch6 LATERAL strafe   (left/right over the bin)   -- align lat axis
    image-Y  -> Ch5 SURGE fore/aft   (forward/back over the bin) -- align depth axis
    bbox fill-> DEPTH descent        (get closer for the drop)   -- align fwd= (optional)
    fire     -> DROPPER (3/4)

So the SAME ``align('fire', lat=0, depth=0)`` call now centres the hull over the bin
in BOTH horizontal axes (lat = left/right, depth-axis = fore/aft SURGE, two-sided +
braked -- not the old depth-setpoint hack), while ArduSub holds ``BIN_DEPTH_M`` on Ch3.
The verb auto-switches the live detector to 'downward' (pausing forward -- one detector
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
    # lat = left/right (Ch6), depth axis = fore/aft SURGE (Ch5, two-sided + braked).
    # ArduSub holds BIN_DEPTH_M. Optional BIN_DESCEND_FILL>0 descends for a closer
    # drop (bounded by BIN_MAX_DEPTH_M). Creep-search finds the bin on target loss.
    aligned = duburi.vision.align(
        'fire', camera='downward', lat=0, depth=0,
        fwd=(BIN_DESCEND_FILL or None), fwd_mode='height',
        err=BIN_CENTRE_ERR_PX, gain=ALIGN_GAIN, duration=25,
        surge_sign=BIN_SURGE_SIGN, max_depth_m=BIN_MAX_DEPTH_M,
        fallback=creep_forward)

    # Drop ONLY when centred -- a blind drop wastes the marker into empty water.
    if aligned:
        duburi.pause(3.0)                   # settle over the bin before the drop
        duburi.fire(BIN_DROPPER_CHANNEL)    # dropper — channel always explicit
        duburi.pause(2.0)                   # confirm drop complete
    elif log:
        log('[bin_task] never centred over the bin — marker HELD (no blind drop); '
            'check the downward detector resumed and BIN_SURGE_SIGN')

    duburi.use_camera('forward')        # back to forward (pauses downward detector)


# ── Mission-authored fallback search patterns (pure control) ────────────────────
def creep_forward(duburi):
    """One short forward creep, then return so the vision loop retries."""
    duburi.move_forward(SEARCH_CREEP_S, gain=SEARCH_FORWARD_GAIN)
