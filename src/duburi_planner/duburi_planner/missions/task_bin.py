"""Bin drop task — switch to downward camera, locate bin, drop marker.

Two-verb vision on the DOWNWARD camera: align() centres the AUV over the
bin in the lat (Ch6 strafe) and depth-as-fore/aft sense using pixel
error, then we drop. On the downward camera, image-Y maps to fore/aft;
align()'s lat axis handles left/right and we let ArduSub hold depth.

Standalone test (bin_fire_blood.pt — classes: blood(0) fire(1)):
    ros2 run duburi_planner mission task_bin
"""

from duburi_planner.missions.competition_config import (
    BIN_HEADING_DEG,
    BIN_DEPTH_M,
    BIN_CENTRE_ERR_PX,
    ALIGN_GAIN,
    SEARCH_FORWARD_GAIN,
    SEARCH_CREEP_S,
)


def run(duburi, log=None):
    duburi.mission_reset()
    if BIN_HEADING_DEG is not None:
        duburi.turn(BIN_HEADING_DEG)
    duburi.set_depth(BIN_DEPTH_M, timeout=30)

    duburi.use_camera('downward')
    duburi.resume_detector('downward')
    duburi.set_model('bin_fire_blood', node='/duburi_detector_dwn')
    duburi.set_classes('fire,blood', node='/duburi_detector_dwn')

    # ── Centre the AUV over the bin (lat + fore/aft via the depth axis) ───────
    # Downward cam: ey>0 = target aft, so the depth axis nudge drives the AUV
    # fore/aft to centre it; lat handles left/right. Creep to find it first.
    duburi.vision.align(
        'fire', camera='downward', lat=0, depth=0,
        err=BIN_CENTRE_ERR_PX, gain=ALIGN_GAIN, duration=25,
        fallback=creep_forward)

    duburi.pause(3.0)   # 3s stability confirmation before drop
    duburi.fire(3)      # dropper_1 — channel always explicit
    duburi.pause(2.0)   # confirm drop complete

    duburi.pause_detector('downward')
    duburi.use_camera('forward')


# ── Mission-authored fallback search patterns (pure control) ────────────────────
def creep_forward(duburi):
    """One short forward creep, then return so the vision loop retries."""
    duburi.move_forward(SEARCH_CREEP_S, gain=SEARCH_FORWARD_GAIN)
