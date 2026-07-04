"""vision_dual -- forward + downward cameras, the RoboSub 2026 competition vision bringup.

Brings up both cameras, each with its own detector (+ tracker) and a
shared HUD viewer. Detector nodes are named ``duburi_detector_forward``
and ``duburi_detector_downward`` -- exactly the names the mission DSL
targets via ``duburi.set_model(... camera='forward')`` /
``resume_detector('downward')`` -- so per-task model/class hot-switching
just works.

Both detectors start PAUSED by default (saves Jetson GPU); each task
chunk calls ``duburi.resume_detector(<cam>)`` then ``pause_detector``.

Start it AFTER the control stack (``duburi start`` / bringup).

Usage:
    # Competition defaults (gate_rescue_repair fwd, bin_fire_blood dwn, paused):
    ros2 launch duburi_vision vision_dual.launch.py

    # Headless pool day:
    ros2 launch duburi_vision vision_dual.launch.py viewer:=false

    # Single model per camera:
    ros2 launch duburi_vision vision_dual.launch.py \\
        fwd_model:=gate_flare_medium_100ep fwd_classes:=gate,flare \\
        dwn_model:=bin_fire_blood dwn_classes:=fire,blood \\
        fwd_device:=0 dwn_device:=4

    # MULTI-MODEL (registry) -- forward switches gate->slalom->torpedo at
    # runtime; a mission's ClassRef / set_model() needs the model pre-loaded
    # here or it errors "not in registry". Pass the class UNION; per-camera conf:
    ros2 launch duburi_vision vision_dual.launch.py \\
        fwd_models:=gate_rescue_repair,slalom_red_pipe,torpedo_blood_hole \\
        fwd_classes:=gate,rescue,repair,red_pipe,torpedo,blood,hole fwd_conf:=0.60 \\
        dwn_models:=bin_fire_blood dwn_classes:=fire,blood dwn_conf:=0.70

    # Detectors live from the start (no per-task resume):
    ros2 launch duburi_vision vision_dual.launch.py paused:=false

    # Dataset VIDEO sources instead of live cameras (gate clip fwd, bin clip dwn).
    # For a video-tuned preset (detectors live, splash, loop) use video.launch.py:
    ros2 launch duburi_vision vision_dual.launch.py \\
        fwd_video:=/path/gate.mp4 dwn_video:=/path/bin.mp4 paused:=false

Viewer keys: f=forward  d=downward  D=depth-map  (the HUD DISPLAYS only one
camera at a time -- the side-by-side view was removed so it never subscribes both).
Camera/detector/tracker log at WARN; the viewer logs at INFO.

NOTE: this bringup OPENS BOTH camera streams (paused only gates detector
inference, not the MJPEG stream). Two 1080p USB-2 cameras on one 480 Mbps bus is
bandwidth-heavy and can trip USB enumeration; for a single-task run prefer the
one-camera vision.launch.py (camera:=forward|downward). See
.claude/context/dual-camera-setup.md "two-cameras-at-once" for the guard rails.
"""

from launch                  import LaunchDescription
from launch.actions          import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions       import IfCondition
from launch.event_handlers   import OnProcessExit
from launch.events           import Shutdown
from launch.substitutions    import LaunchConfiguration, PythonExpression
from launch_ros.actions      import Node

_QUIET = ['--log-level', 'warn']


def generate_launch_description():
    args = [
        DeclareLaunchArgument('fwd_device',   default_value='0',
                              description='/dev/videoN index for the forward camera (fallback '
                                          'when fwd_device_path is empty)'),
        DeclareLaunchArgument('dwn_device',   default_value='4',
                              description='/dev/videoN index for the downward camera (fallback '
                                          'when dwn_device_path is empty)'),
        # PORT-STABLE identity for two IDENTICAL cameras (same VID/PID). Set these to the
        # /dev/v4l/by-path/…-video-index0 symlinks so forward/downward never swap on
        # reboot/re-plug. Empty => fall back to the int index above. Non-empty wins.
        # See .claude/context/dual-camera-setup.md for how to find the by-path values.
        DeclareLaunchArgument('fwd_device_path', default_value='',
                              description='by-path symlink for the forward camera (port-stable)'),
        DeclareLaunchArgument('dwn_device_path', default_value='',
                              description='by-path symlink for the downward camera (port-stable)'),
        # Forward camera -- gate / slalom / torpedo tasks
        DeclareLaunchArgument('fwd_model',    default_value='gate_rescue_repair',
                              description='Single YOLO model stem (used only when '
                                          'fwd_models is empty)'),
        # REGISTRY mode: CSV of model stems the forward detector loads AND can
        # switch between at runtime (missions do this via ClassRef / set_model).
        # A bare stem registers under its own name, so a mission switches with
        # set_model("slalom_red_pipe"). Empty -> single-model mode (fwd_model).
        # The FIRST stem is the startup/active model.
        DeclareLaunchArgument('fwd_models',   default_value='',
                              description='CSV of model stems for runtime switching '
                                          '(e.g. gate_rescue_repair,slalom_red_pipe,'
                                          'torpedo_blood_hole). Empty = single fwd_model.'),
        DeclareLaunchArgument('fwd_classes',  default_value='gate,rescue,repair',
                              description='Class filter for the forward detector'),
        # Downward camera -- bin / drop tasks
        DeclareLaunchArgument('dwn_model',    default_value='bin_fire_blood',
                              description='Single YOLO model stem (used only when '
                                          'dwn_models is empty)'),
        DeclareLaunchArgument('dwn_models',   default_value='',
                              description='CSV of model stems for runtime switching '
                                          'on the downward detector. Empty = single dwn_model.'),
        DeclareLaunchArgument('dwn_classes',  default_value='fire,blood',
                              description='Class filter for the downward detector'),
        DeclareLaunchArgument('fwd_conf',     default_value='0.35'),
        DeclareLaunchArgument('dwn_conf',     default_value='0.35'),
        DeclareLaunchArgument('device_cls',   default_value='cuda:0',
                              description='Inference device for YOLO (cuda:0 | cpu)'),
        DeclareLaunchArgument('imgsz',        default_value='640',
                              description='Inference square size (both detectors). NOTE: a TensorRT '
                                          '.engine bakes imgsz at export -- this only re-scales the '
                                          '.pt fallback. For TRT, re-export to match: '
                                          'export_engine --all --imgsz <N>, then imgsz:=<N> here.'),
        DeclareLaunchArgument('max_det',      default_value='100',
                              description='Post-NMS detection cap, both detectors (runtime; lower '
                                          'toward ~10 if NMS is the FPS bottleneck on busy frames).'),
        DeclareLaunchArgument('paused',       default_value='true',
                              description='Start both detectors paused (resume_detector per task)'),
        DeclareLaunchArgument('viewer',       default_value='true'),
        DeclareLaunchArgument('tracking',     default_value='true'),
        DeclareLaunchArgument('tracker_type', default_value='ocsort',
                              description='Tracker engine: ocsort (default) | bytetrack | legacy_bytetrack'),
        # Per-camera video sources. A non-empty path runs that camera off a
        # dataset clip instead of the live webcam (forward = gate clip, downward
        # = bin clip) -- lets a full dual-camera mission be exercised against real
        # detections with no hardware. Empty = live webcam (back-compat default).
        DeclareLaunchArgument('fwd_video',    default_value='',
                              description='Video file for the FORWARD camera (e.g. a gate clip). '
                                          'Set => forward runs off the file; empty => live webcam.'),
        DeclareLaunchArgument('dwn_video',    default_value='',
                              description='Video file for the DOWNWARD camera (e.g. a bin clip). '
                                          'Set => downward runs off the file; empty => live webcam.'),
        DeclareLaunchArgument('fwd_loop',     default_value='true',
                              description='Loop the forward video at EOF (video source only).'),
        DeclareLaunchArgument('dwn_loop',     default_value='true',
                              description='Loop the downward video at EOF (video source only).'),
    ]

    def camera(profile: str, device_arg: str, video_arg: str, loop_arg: str,
               device_path_arg: str) -> Node:
        # Source-aware: a non-empty video path runs this camera off a file
        # (source=video_file, profile cleared so camera_node takes the path);
        # empty falls back to the live webcam profile. The node NAME stays the
        # profile ('forward'/'downward') either way, so the detector namespace
        # /duburi/vision/<name>/* and the manager's _vision_state_for(<name>)
        # resolve identically to live -- vision verbs need no change.
        video = LaunchConfiguration(video_arg)
        src   = PythonExpression(["'video_file' if '", video, "' else ''"])
        prof  = PythonExpression(["'' if '", video, "' else '", profile, "'"])
        return Node(
            package='duburi_vision', executable='camera_node',
            name=f'duburi_camera_{profile}', output='screen', ros_arguments=_QUIET,
            parameters=[{
                'profile': prof,
                'source':  src,
                'name':    profile,
                'device':  LaunchConfiguration(device_arg),
                'device_path': LaunchConfiguration(device_path_arg),
                'path':    video,
                'loop':    LaunchConfiguration(loop_arg),
            }],
        )

    def detector(profile: str, model_arg: str, models_arg: str,
                 classes_arg: str, conf_arg: str) -> Node:
        return Node(
            package='duburi_vision', executable='detector_node',
            name=f'duburi_detector_{profile}', output='screen', ros_arguments=_QUIET,
            parameters=[{
                'camera':              profile,
                'model_path':          LaunchConfiguration(model_arg),
                # Registry mode: when non-empty the detector loads ALL these
                # models and can set_model() between them at runtime; model_path
                # is then ignored. Empty -> single model_path (back-compat).
                'models':              LaunchConfiguration(models_arg),
                'device':              LaunchConfiguration('device_cls'),
                'classes':             LaunchConfiguration(classes_arg),
                'conf':                LaunchConfiguration(conf_arg),
                'imgsz':               LaunchConfiguration('imgsz'),
                'max_det':             LaunchConfiguration('max_det'),
                'half':                True,
                'paused':              LaunchConfiguration('paused'),
                'publish_debug_image': True,
                'debug_image_hz':      10.0,
            }],
        )

    def tracker(profile: str) -> Node:
        return Node(
            package='duburi_vision', executable='tracker_node',
            name=f'duburi_tracker_{profile}', output='screen', ros_arguments=_QUIET,
            parameters=[{'camera': profile,
                         'tracker_type': LaunchConfiguration('tracker_type')}],
            condition=IfCondition(LaunchConfiguration('tracking')),
        )

    # Enable the HUD's video playback controls + warm-up splash whenever either
    # camera is a file source (Space=pause, ,/.=frame-step, arrows=seek).
    any_video = PythonExpression([
        "True if ('", LaunchConfiguration('fwd_video'), "' or '",
        LaunchConfiguration('dwn_video'), "') else False"])
    viewer = Node(
        package='duburi_vision', executable='vision_display',
        name='duburi_display', output='screen',
        parameters=[{'camera': 'forward', 'video_file_mode': any_video}],
        condition=IfCondition(LaunchConfiguration('viewer')),
    )

    shutdown_on_exit = RegisterEventHandler(
        OnProcessExit(target_action=viewer, on_exit=[EmitEvent(event=Shutdown())])
    )

    return LaunchDescription(args + [
        camera('forward',  'fwd_device', 'fwd_video', 'fwd_loop', 'fwd_device_path'),
        camera('downward', 'dwn_device', 'dwn_video', 'dwn_loop', 'dwn_device_path'),
        detector('forward',  'fwd_model', 'fwd_models', 'fwd_classes', 'fwd_conf'),
        detector('downward', 'dwn_model', 'dwn_models', 'dwn_classes', 'dwn_conf'),
        tracker('forward'),
        tracker('downward'),
        viewer,
        shutdown_on_exit,
    ])
