"""Bin drop task — DOWNWARD camera, hover over the bin, drop a marker.

RoboSub Task 3 (Recon/Bins). The AUV flies ABOVE the bin looking straight down,
so the downward camera's image axes map onto the hull's horizontal axes (full
table: .claude/context/perception/downward-camera.md):

    image-X  -> LATERAL strafe   (left/right over the bin)   -- align ``lat``
    image-Y  -> SURGE fore/aft   (forward/back over the bin) -- align ``fwd``
    fire     -> DROPPER channel

So ``align('fire', camera='downward', lat=0, fwd=0)`` centres the hull over the
bin in both horizontal axes (two-sided + braked), while the board holds
``BIN_DEPTH_M``. The verb switches the live detector to 'downward' (pausing
forward -- one detector at a time on the Hailo) and flips the HUD.

Before an armed run, VERIFY the surge sign DISARMED:
    ros2 run mongla_vision vision_thrust_check --camera downward
  A bin AHEAD in the image must drive the hull FORWARD. If it reverses, set
  `vision.surge_sign` the other way. A wrong sign is positive feedback.

Standalone test (bin_fire_blood -- classes: blood(0) fire(1)):
    ros2 run mongla_planner mission task_bin
"""

from mongla_planner.missions.competition_config import (
    BIN_HEADING_DEG,
    BIN_DEPTH_M,
    BIN_CENTRE_ERR_PX,
    BIN_DROPPER_CHANNEL,
    ALIGN_GAIN,
    SEARCH_FORWARD_GAIN,
    SEARCH_CREEP_S,
)

_DWN = '/mongla_detector_downward'


def run(mongla, log=None):
    mongla.mission_reset()
    if BIN_HEADING_DEG is not None:
        mongla.turn(BIN_HEADING_DEG)
    mongla.set_depth(BIN_DEPTH_M, timeout=30)

    # The eye this task needs may not be there. A camera that fails to open
    # does not take the vision stack down with it -- the other camera keeps
    # running and this one's detector is simply absent. Calling a vision verb
    # anyway makes `_ensure_detector` abort the WHOLE run over one dead camera,
    # taking every later task with it. Skip this task instead and let the rest
    # of the mission score.
    if not mongla.camera_usable('downward', task='bin drop'):
        return

    mongla.use_camera('downward')
    # The downward setup lives INSIDE the try so that if any of it raises, the
    # finally still restores the forward camera -- never leave the downward
    # detector live for a later forward task.
    try:
        mongla.set_model('bin_fire_blood', node=_DWN)
        mongla.set_classes('fire,blood', node=_DWN)
        aligned = mongla.vision.align(
            'fire', camera='downward', lat=0, fwd=0,          # lat + surge centre over bin
            err=BIN_CENTRE_ERR_PX, gain=ALIGN_GAIN, duration=25,
            fallback=creep_forward)

        # Drop ONLY when centred -- a blind drop wastes the marker into empty water.
        if aligned:
            mongla.pause(3.0)                   # settle over the bin before the drop
            mongla.fire(BIN_DROPPER_CHANNEL)    # dropper — channel always explicit
            mongla.pause(2.0)                   # confirm drop complete
        elif log:
            log('[bin_task] never centred over the bin — marker HELD (no blind drop); '
                'check the downward detector resumed and vision.surge_sign')
    finally:
        mongla.use_camera('forward')


# ── Mission-authored fallback search patterns (pure control) ────────────────────
def creep_forward(mongla):
    """One short forward creep, then return so the vision loop retries."""
    mongla.move_forward(SEARCH_CREEP_S, gain=SEARCH_FORWARD_GAIN)
