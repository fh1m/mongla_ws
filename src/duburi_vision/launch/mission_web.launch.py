"""mission_web -- the ONE command for pool-day monitoring + mission planning.

Brings up both cameras + both detectors (via vision_dual) + web_video_server
(MJPEG video) + the mission_web console node, which auto-opens the browser.
Detectors start LIVE (paused:=false) so both streams show immediately; the
console's per-camera Pause and "Make live cam" (exclusive) controls let you drop
to one detector if the Jetson GPU is bound. Video (annotated image_debug) is on
:video_port; the console + SSE data on :web_port.

    # Live cameras (competition):
    ros2 launch duburi_vision mission_web.launch.py

    # Dataset videos, no hardware (dev-box end-to-end test):
    ros2 launch duburi_vision mission_web.launch.py \\
        fwd_video:=/path/gate.mp4 dwn_video:=/path/bin.mp4

    # Registry (runtime model switching from the UI dropdown / DSL):
    ros2 launch duburi_vision mission_web.launch.py \\
        fwd_models:=gate_rescue_repair,slalom_red_pipe,torpedo_blood_hole \\
        fwd_classes:=gate,rescue,repair,red_pipe,torpedo,blood,hole

Port-forward :web_port (console) and :video_port (streams) to a dev-box browser,
or just open http://localhost:<web_port> on the Jetson NoMachine desktop.
"""

import os

from launch                    import LaunchDescription
from launch.actions            import (DeclareLaunchArgument, IncludeLaunchDescription,
                                        SetEnvironmentVariable)
from launch.conditions         import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions      import LaunchConfiguration
from launch_ros.actions        import Node
from ament_index_python.packages import get_package_share_directory


# Pass-through args forwarded to vision_dual (curated subset -- power users can
# run vision_dual directly for the full arg surface).
_PASSTHRU = ('fwd_device', 'dwn_device', 'fwd_device_path', 'dwn_device_path',
             'fwd_model', 'fwd_models', 'fwd_classes', 'fwd_conf', 'fwd_model_conf',
             'dwn_model', 'dwn_models', 'dwn_classes', 'dwn_conf', 'dwn_model_conf',
             'fwd_video', 'dwn_video', 'imgsz', 'max_det', 'tracking', 'debug_image_hz')


def generate_launch_description():
    vision_dual = os.path.join(
        get_package_share_directory('duburi_vision'), 'launch', 'vision_dual.launch.py')

    args = [
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

    forwarded = {k: LaunchConfiguration(k) for k in _PASSTHRU}
    forwarded['paused'] = LaunchConfiguration('paused')
    forwarded['viewer'] = LaunchConfiguration('viewer')

    vision = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(vision_dual),
        launch_arguments=forwarded.items(),
    )

    video_server = Node(
        package='web_video_server', executable='web_video_server',
        name='duburi_web_video', output='screen',
        parameters=[{'port': LaunchConfiguration('video_port'), 'address': '0.0.0.0'}],
    )

    # --no-browser is a process arg (not a ROS param); pass it via an env var the
    # node also honours, gated on the launch arg.
    console = Node(
        package='duburi_vision', executable='mission_web',
        name='duburi_mission_web', output='screen',
        parameters=[{'web_port':   LaunchConfiguration('web_port'),
                     'video_port': LaunchConfiguration('video_port')}],
    )
    no_browser_env = SetEnvironmentVariable(
        'MISSION_WEB_NO_BROWSER', '1',
        condition=IfCondition(LaunchConfiguration('no_browser')))

    return LaunchDescription(args + [no_browser_env, vision, video_server, console])
