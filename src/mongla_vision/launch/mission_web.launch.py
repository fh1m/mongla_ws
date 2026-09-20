"""mission_web -- the ONE command for pool-day monitoring + mission planning.

Brings up camera(s) + detector(s) + web_video_server (MJPEG video) + the
mission_web console node, which auto-opens the browser. Detectors start LIVE
(paused:=false) so streams show immediately; the console's per-camera Pause and
"Make live cam" (exclusive) controls let you drop to one detector if the Jetson
GPU is bound. Video (annotated image_debug) is on :video_port; the console + SSE
data on :web_port.

`cameras:=` selects the rig -- both (default), forward, or downward. A single
camera uses the one-camera vision.launch.py (so a box with only ONE camera never
crashes trying to open an absent second device), and the console shows just that
panel; both uses vision_dual.

    # BOTH cameras, competition (default):
    ros2 launch mongla_vision mission_web.launch.py

    # SINGLE camera (only one plugged in / one task):
    ros2 launch mongla_vision mission_web.launch.py cameras:=forward
    ros2 launch mongla_vision mission_web.launch.py cameras:=downward dwn_device:=0

    # Dataset videos, no hardware (dev-box end-to-end test):
    ros2 launch mongla_vision mission_web.launch.py \\
        fwd_video:=/path/gate.mp4 dwn_video:=/path/bin.mp4
    ros2 launch mongla_vision mission_web.launch.py cameras:=forward fwd_video:=/path/gate.mp4

    # Registry (runtime model switching from the UI dropdown / DSL):
    ros2 launch mongla_vision mission_web.launch.py \\
        fwd_models:=gate_rescue_repair,slalom_red_pipe,torpedo_blood_hole \\
        fwd_classes:=gate,rescue,repair,red_pipe,torpedo,blood,hole

Port-forward :web_port (console) and :video_port (streams) to a dev-box browser,
or just open http://localhost:<web_port> on the Jetson NoMachine desktop.
"""

import os

from launch                    import LaunchDescription
from launch.actions            import (DeclareLaunchArgument, IncludeLaunchDescription,
                                        SetEnvironmentVariable, LogInfo)
from launch.conditions         import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions      import LaunchConfiguration, PythonExpression
from launch_ros.actions        import Node
from ament_index_python.packages import (get_package_share_directory,
                                          PackageNotFoundError)


# Pass-through args forwarded to vision_dual (curated subset -- power users can
# run vision_dual directly for the full arg surface).
_PASSTHRU = ('fwd_device', 'dwn_device', 'fwd_device_path', 'dwn_device_path',
             'fwd_model', 'fwd_models', 'fwd_classes', 'fwd_conf', 'fwd_model_conf',
             'dwn_model', 'dwn_models', 'dwn_classes', 'dwn_conf', 'dwn_model_conf',
             'fwd_video', 'dwn_video', 'imgsz', 'max_det', 'tracking', 'debug_image_hz')


def generate_launch_description():
    share = get_package_share_directory('mongla_vision')
    vision_dual   = os.path.join(share, 'launch', 'vision_dual.launch.py')
    vision_single = os.path.join(share, 'launch', 'vision.launch.py')

    args = [
        DeclareLaunchArgument('cameras',    default_value='both',
                              description="rig: both (vision_dual) | forward | downward "
                                          "(single camera via vision.launch.py)"),
        DeclareLaunchArgument('web_port',   default_value='8090',
                              description='HTTP port for the mission console + SSE'),
        DeclareLaunchArgument('video_port', default_value='8080',
                              description='web_video_server MJPEG port'),
        DeclareLaunchArgument('no_browser', default_value='false',
                              description='true = do not auto-open the browser'),
        # Viewer wants live streams by default (image_debug only publishes while a
        # detector infers). Flip paused:=true to start dark and resume from the UI.
        DeclareLaunchArgument('paused',     default_value='false'),
        # Browser-stream smoothness: default higher than vision_dual's 10 (a web
        # viewer wants smooth motion; the detector caps it at real inference FPS).
        DeclareLaunchArgument('debug_image_hz', default_value='15.0'),
        DeclareLaunchArgument('viewer',     default_value='false',
                              description='also open the OpenCV HUD window (web console is primary)'),
        # Forwarded to vision_dual with its own defaults when unset.
        DeclareLaunchArgument('fwd_device', default_value='0'),
        DeclareLaunchArgument('dwn_device', default_value='4'),
        DeclareLaunchArgument('fwd_device_path', default_value=''),
        DeclareLaunchArgument('dwn_device_path', default_value=''),
        DeclareLaunchArgument('fwd_model',   default_value='gate_rescue_repair'),
        DeclareLaunchArgument('fwd_models',  default_value=''),
        DeclareLaunchArgument('fwd_classes', default_value='gate,rescue,repair'),
        DeclareLaunchArgument('fwd_conf',    default_value='0.35'),
        DeclareLaunchArgument('fwd_model_conf', default_value=''),
        DeclareLaunchArgument('dwn_model',   default_value='bin_fire_blood'),
        DeclareLaunchArgument('dwn_models',  default_value=''),
        DeclareLaunchArgument('dwn_classes', default_value='fire,blood'),
        DeclareLaunchArgument('dwn_conf',    default_value='0.35'),
        DeclareLaunchArgument('dwn_model_conf', default_value=''),
        DeclareLaunchArgument('fwd_video',   default_value=''),
        DeclareLaunchArgument('dwn_video',   default_value=''),
        DeclareLaunchArgument('imgsz',       default_value='640'),
        DeclareLaunchArgument('max_det',     default_value='100'),
        DeclareLaunchArgument('tracking',    default_value='true'),
    ]

    # --- BOTH: vision_dual (unchanged path) ---
    dual_args = {k: LaunchConfiguration(k) for k in _PASSTHRU}
    dual_args['paused'] = LaunchConfiguration('paused')
    dual_args['viewer'] = LaunchConfiguration('viewer')
    vision_both = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(vision_dual),
        launch_arguments=dual_args.items(),
        condition=LaunchConfigurationEquals('cameras', 'both'),
    )

    # --- SINGLE: one camera via vision.launch.py. Maps the matching fwd_*/dwn_*
    # args onto vision's flat arg surface so the same console kwargs work either way.
    def single(cam, dev, dev_path, model, models, classes, conf, video):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(vision_single),
            condition=LaunchConfigurationEquals('cameras', cam),
            launch_arguments={
                'camera':         cam,
                'device':         LaunchConfiguration(dev),
                'device_path':    LaunchConfiguration(dev_path),
                'model':          LaunchConfiguration(model),
                'models':         LaunchConfiguration(models),
                'classes':        LaunchConfiguration(classes),
                'conf':           LaunchConfiguration(conf),
                'video_file':     LaunchConfiguration(video),
                'imgsz':          LaunchConfiguration('imgsz'),
                'max_det':        LaunchConfiguration('max_det'),
                'tracking':       LaunchConfiguration('tracking'),
                'paused':         LaunchConfiguration('paused'),
                'debug_image_hz': LaunchConfiguration('debug_image_hz'),
                'viewer':         LaunchConfiguration('viewer'),
            }.items(),
        )
    vision_fwd = single('forward', 'fwd_device', 'fwd_device_path', 'fwd_model',
                        'fwd_models', 'fwd_classes', 'fwd_conf', 'fwd_video')
    vision_dwn = single('downward', 'dwn_device', 'dwn_device_path', 'dwn_model',
                        'dwn_models', 'dwn_classes', 'dwn_conf', 'dwn_video')

    # web_video_server is an apt package (ros-humble-web-video-server), not a repo
    # dep -- resolve it defensively. If it's ABSENT, a Node(package=...) action would
    # throw at launch-evaluation time and kill the WHOLE launch (camera+detector+
    # console never start). Instead skip it with a loud hint: the console + data
    # pipeline still come up; only the video tiles are blank ("stream not available").
    try:
        get_package_share_directory('web_video_server')
        video_actions = [Node(
            package='web_video_server', executable='web_video_server',
            name='mongla_web_video', output='screen',
            parameters=[{'port': LaunchConfiguration('video_port'),
                         'address': '0.0.0.0'}],
        )]
    except PackageNotFoundError:
        video_actions = [LogInfo(msg=(
            '[mission_web] web_video_server NOT installed -- console + detections '
            'run, but video tiles will be blank. Install: '
            'sudo apt install ros-humble-web-video-server'))]

    # The console subscribes only the selected camera(s) -> no phantom panel for a
    # camera that isn't launched. 'both' -> forward,downward; single -> just that one.
    cams_param = PythonExpression(
        ["'forward,downward' if '", LaunchConfiguration('cameras'),
         "' == 'both' else '", LaunchConfiguration('cameras'), "'"])
    console = Node(
        package='mongla_vision', executable='mission_web',
        name='mongla_mission_web', output='screen',
        parameters=[{'web_port':   LaunchConfiguration('web_port'),
                     'video_port': LaunchConfiguration('video_port'),
                     'cameras':    cams_param}],
    )
    no_browser_env = SetEnvironmentVariable(
        'MISSION_WEB_NO_BROWSER', '1',
        condition=IfCondition(LaunchConfiguration('no_browser')))

    return LaunchDescription(args + [no_browser_env, vision_both, vision_fwd,
                                     vision_dwn] + video_actions + [console])
