"""video -- dual-camera vision pipeline driven by DATASET VIDEO FILES.

A video-tuned preset over ``vision_dual.launch.py``: point the forward camera
at a gate clip and the downward camera at a bin clip, and the full pipeline
(camera -> detector -> tracker -> HUD) runs against real recorded detections
with no hardware. Use it to exercise vision verbs (``vision.align`` /
``vision.move``) and rehearse autonomous mission *logic* before pool day.

    # Forward = gate clip, downward = bin clip (both loop, detectors live):
    ros2 launch duburi_vision video.launch.py \\
        fwd_video:=/path/to/gate.mp4 dwn_video:=/path/to/bin.mp4

    # Single video on the forward camera only:
    ros2 launch duburi_vision video.launch.py fwd_video:=/path/to/gate.mp4

    # Single pass (no loop) for a clean one-shot mission-sequence run:
    ros2 launch duburi_vision video.launch.py \\
        fwd_video:=/path/to/gate.mp4 loop:=false

    # Custom models / classes to match your clips:
    ros2 launch duburi_vision video.launch.py \\
        fwd_video:=/tmp/gate.mp4 fwd_model:=gate_flare_medium_100ep fwd_classes:=gate,flare

Defaults vs vision_dual: detectors start LIVE (paused:=false) so verbs see
detections immediately, the HUD enables video playback controls + warm-up
splash, and both videos loop by default.

What this validates -- and what it does NOT (read before trusting a result)
--------------------------------------------------------------------------
Video testing is **open-loop on vision**: the clip plays on its own timeline
and does NOT react to commanded thrust. So it validates:
  * detections firing on real imagery (model/conf/class tuning),
  * mission LOGIC -- detected() transitions, verb dispatch, fallback search,
    model/class switching, the downward-camera handoff,
  * thrust DIRECTION/SIGN for a given bbox position.
It does NOT validate closed-loop dynamics -- convergence, overshoot, the
align(hold=) station-keep, or arrival braking -- because the scene won't move
when the AUV does. align/move will chase a bbox that moves on the file's
schedule, never truly converging unless the clip happens to show a centred
target. For dynamics, use Gazebo/SITL; for detection + sequencing, use this.

Pair with control in separate terminals (SITL for /duburi/state + the manager)
exactly as documented in .claude/context/video-testing.md.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _setup(context, *_args, **_kwargs):
    fwd = LaunchConfiguration('fwd_video').perform(context).strip()
    dwn = LaunchConfiguration('dwn_video').perform(context).strip()
    if not fwd and not dwn:
        raise RuntimeError(
            "video.launch.py needs at least one video source -- pass "
            "fwd_video:=/path/to/gate.mp4 and/or dwn_video:=/path/to/bin.mp4. "
            "(For a single LIVE camera use vision.launch.py instead.)")

    dual_path = os.path.join(
        get_package_share_directory('duburi_vision'),
        'launch', 'vision_dual.launch.py')

    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(dual_path),
        launch_arguments={
            'fwd_video':   fwd,
            'dwn_video':   dwn,
            'fwd_model':   LaunchConfiguration('fwd_model'),
            'fwd_classes': LaunchConfiguration('fwd_classes'),
            'dwn_model':   LaunchConfiguration('dwn_model'),
            'dwn_classes': LaunchConfiguration('dwn_classes'),
            'fwd_conf':    LaunchConfiguration('conf'),
            'dwn_conf':    LaunchConfiguration('conf'),
            'fwd_loop':    LaunchConfiguration('loop'),
            'dwn_loop':    LaunchConfiguration('loop'),
            'paused':      'false',     # video testing wants live detections
            'viewer':      LaunchConfiguration('viewer'),
            'tracking':    LaunchConfiguration('tracking'),
            'anchor':      LaunchConfiguration('anchor'),
            'imgsz':       LaunchConfiguration('imgsz'),
            'max_det':     LaunchConfiguration('max_det'),
        }.items(),
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('fwd_video',   default_value='',
                              description='Forward-camera video (e.g. a gate dataset clip). '
                                          'At least one of fwd_video / dwn_video is required.'),
        DeclareLaunchArgument('dwn_video',   default_value='',
                              description='Downward-camera video (e.g. a bin dataset clip).'),
        DeclareLaunchArgument('fwd_model',   default_value='gate_rescue_repair',
                              description='YOLO model stem for the forward detector.'),
        DeclareLaunchArgument('fwd_classes', default_value='gate,rescue,repair',
                              description='Class filter for the forward detector.'),
        DeclareLaunchArgument('dwn_model',   default_value='bin_fire_blood',
                              description='YOLO model stem for the downward detector.'),
        DeclareLaunchArgument('dwn_classes', default_value='fire,blood',
                              description='Class filter for the downward detector.'),
        DeclareLaunchArgument('conf',        default_value='0.35',
                              description='Detection confidence threshold (both detectors).'),
        DeclareLaunchArgument('loop',        default_value='true',
                              description='Loop both videos at EOF. true = detections keep flowing '
                                          'for verb/gain tuning; false = single pass for a clean '
                                          'mission-sequence run (a loop re-shows the target at EOF, '
                                          'which can re-trigger detected() acquisition mid-run).'),
        DeclareLaunchArgument('viewer',      default_value='true',
                              description='Open the HUD (viewer:=false = headless autonomous run).'),
        DeclareLaunchArgument('tracking',    default_value='true'),
        DeclareLaunchArgument('anchor',      default_value='false',
                              description='Start anchor_node (XFeat superglue) for homography '
                                          'lock testing against the video sources.'),
        DeclareLaunchArgument('imgsz',       default_value='640'),
        DeclareLaunchArgument('max_det',     default_value='100'),
        OpaqueFunction(function=_setup),
    ])
