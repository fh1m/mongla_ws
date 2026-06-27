"""vision -- single-camera vision subsystem (camera + detector + tracker + viewer).

The one launch file for any single-camera setup: live USB webcam, a
pre-recorded video file, or a ROS image topic (Gazebo / re-published
stream). Brings up the whole perception chain and names the detector
node ``duburi_detector_<camera>`` so the mission DSL
(``duburi.set_model(... camera='forward')`` etc.) and
``duburi.vision.*`` verbs reach it with no node-name guessing.

Start it AFTER the control stack (``duburi start`` / bringup), then run
vision missions.

Usage:
    # Live forward camera, pool gate model:
    ros2 launch duburi_vision vision.launch.py camera:=forward \\
        model:=gate_rescue_repair classes:=gate,rescue,repair conf:=0.4

    # Headless (mission mode, no OpenCV window):
    ros2 launch duburi_vision vision.launch.py camera:=forward viewer:=false

    # Replay a recorded run:
    ros2 launch duburi_vision vision.launch.py video_file:=/tmp/pool_run.mp4 classes:=gate

    # Gazebo / re-published image topic:
    ros2 launch duburi_vision vision.launch.py camera:=sim_front \\
        topic:=/duburi/sim/front_camera/image_raw

    # Start paused (resume per task to save GPU):
    ros2 launch duburi_vision vision.launch.py camera:=forward paused:=true

Nodes: duburi_camera_<camera>, duburi_detector_<camera>,
       duburi_tracker_<camera>, duburi_image_view (viewer).
Camera/detector/tracker log at WARN; the viewer logs at INFO.
"""

from launch                       import LaunchDescription
from launch.actions               import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions            import IfCondition
from launch.event_handlers        import OnProcessExit
from launch.events                import Shutdown
from launch.substitutions         import LaunchConfiguration, PythonExpression
from launch_ros.actions           import Node

# Quiet by default: camera/tracker/depth emit only warnings+ so the console
# isn't flooded during a mission (YOLO itself is already verbose=False). The
# bare 'warn' is the process default -- it silences rcl/rmw framework chatter
# too. The detector is the exception: it owns the always-on operator alignment
# line, so its own logger is pinned back to info (see _DET_QUIET below).
_QUIET = ['--log-level', 'warn']


def generate_launch_description():
    args = [
        DeclareLaunchArgument('camera',        default_value='forward',
                              description='Camera profile (forward|downward|sim_front|laptop|...). '
                                          'Drives node names: duburi_detector_<camera>.'),
        DeclareLaunchArgument('device',        default_value='-1',
                              description='/dev/videoN index; -1 = use profile default'),
        DeclareLaunchArgument('width',         default_value='640'),
        DeclareLaunchArgument('height',        default_value='480'),
        DeclareLaunchArgument('fps',           default_value='30'),
        DeclareLaunchArgument('video_file',    default_value='',
                              description='Path to a video file; when set, replaces the live webcam'),
        DeclareLaunchArgument('topic',         default_value='',
                              description='ROS image topic to consume (Gazebo / re-published stream). '
                                          'Ignored when video_file is set.'),
        DeclareLaunchArgument('loop',          default_value='true',
                              description='Loop the video file at EOF (video_file only)'),
        DeclareLaunchArgument('model',         default_value='yolov11n',
                              description='Model stem (models/) or .pt path (single-model mode). '
                                          'Pool: gate_rescue_repair / gate_flare_medium_100ep.'),
        DeclareLaunchArgument('models',        default_value='',
                              description='CSV name=stem registry for hot model switching: '
                                          '"gate=gate_nano_100ep,combined=gate_flare_medium_100ep"'),
        DeclareLaunchArgument('active_model',  default_value='',
                              description='Registry key to start with (requires models:="...")'),
        DeclareLaunchArgument('classes',       default_value='',
                              description='CSV class filter; empty = all model classes'),
        DeclareLaunchArgument('conf',          default_value='0.35'),
        DeclareLaunchArgument('iou',           default_value='0.5'),
        DeclareLaunchArgument('device_cls',    default_value='cuda:0',
                              description='Inference device for YOLO (cuda:0 | cpu)'),
        DeclareLaunchArgument('imgsz',         default_value='640',
                              description='Inference square size. NOTE: a TensorRT .engine bakes '
                                          'imgsz at export -- this arg only re-scales the .pt '
                                          'fallback. To bench a size on TRT, re-export to match: '
                                          'export_engine --all --imgsz <N>, then imgsz:=<N> here.'),
        DeclareLaunchArgument('max_det',       default_value='100',
                              description='Post-NMS detection cap (runtime; lower toward ~10 if '
                                          'NMS is the FPS bottleneck on busy frames).'),
        DeclareLaunchArgument('paused',        default_value='false',
                              description='Start the detector paused (resume_detector(camera) per task)'),
        DeclareLaunchArgument('viewer',        default_value='true',
                              description='Open the OpenCV vision_display HUD (viewer:=false = headless)'),
        DeclareLaunchArgument('tracking',      default_value='true',
                              description='Start tracker_node (ByteTrack + Kalman)'),
        DeclareLaunchArgument('track_buffer',  default_value='30'),
        DeclareLaunchArgument('min_hits',      default_value='1'),
        DeclareLaunchArgument('max_predict',   default_value='10'),
        DeclareLaunchArgument('depth',         default_value='false',
                              description='Start depth_estimation_node (monocular vis_range)'),
        DeclareLaunchArgument('depth_model',   default_value='',
                              description='DA V2-Small ONNX path; empty = bbox-area fallback'),
    ]

    cam        = LaunchConfiguration('camera')
    video_file = LaunchConfiguration('video_file')
    topic      = LaunchConfiguration('topic')

    # Source: file > topic > live webcam. The camera_node always runs and
    # republishes to /duburi/vision/<camera>/image_raw so detector + viewer
    # use one topic contract regardless of source.
    source_expr = PythonExpression([
        "'video_file' if '", video_file, "' else ('ros_topic' if '", topic, "' else '')"])
    # A named profile only applies to a live webcam; file/topic sources skip it.
    profile_expr = PythonExpression([
        "'' if ('", video_file, "' or '", topic, "') else '", cam, "'"])
    video_file_mode = PythonExpression(["True if '", video_file, "' else False"])

    # Per-camera node names -- the single naming rule the DSL relies on.
    cam_node = ['duburi_camera_',  cam]
    det_node = ['duburi_detector_', cam]
    trk_node = ['duburi_tracker_',  cam]

    # Detector: process default warn (kills framework gibberish) but pin THIS
    # node's own logger to info so its always-on operator alignment line shows.
    _DET_QUIET = ['--log-level', 'warn', '--log-level', det_node + [':=info']]

    camera_node = Node(
        package='duburi_vision', executable='camera_node', name=cam_node,
        output='screen', ros_arguments=_QUIET,
        parameters=[{
            'profile':         profile_expr,
            'source':          source_expr,
            'name':            cam,
            'topic':           topic,
            'device':          LaunchConfiguration('device'),
            'path':            video_file,
            'loop':            LaunchConfiguration('loop'),
            'width':           LaunchConfiguration('width'),
            'height':          LaunchConfiguration('height'),
            'fps':             LaunchConfiguration('fps'),
            'publish_rate_hz': LaunchConfiguration('fps'),
        }],
    )

    detector_node = Node(
        package='duburi_vision', executable='detector_node', name=det_node,
        output='screen', ros_arguments=_DET_QUIET,
        parameters=[{
            'camera':              cam,
            'model_path':          LaunchConfiguration('model'),
            'models':              LaunchConfiguration('models'),
            'active_model':        LaunchConfiguration('active_model'),
            'device':              LaunchConfiguration('device_cls'),
            'classes':             LaunchConfiguration('classes'),
            'conf':                LaunchConfiguration('conf'),
            'iou':                 LaunchConfiguration('iou'),
            'imgsz':               LaunchConfiguration('imgsz'),
            'max_det':             LaunchConfiguration('max_det'),
            'half':                True,
            'paused':              LaunchConfiguration('paused'),
            'publish_debug_image': True,
            'debug_image_hz':      10.0,
        }],
    )

    tracker_node = Node(
        package='duburi_vision', executable='tracker_node', name=trk_node,
        output='screen', ros_arguments=_QUIET,
        parameters=[{
            'camera':             cam,
            'track_buffer':       LaunchConfiguration('track_buffer'),
            'min_hits':           LaunchConfiguration('min_hits'),
            'max_predict_frames': LaunchConfiguration('max_predict'),
        }],
        condition=IfCondition(LaunchConfiguration('tracking')),
    )

    depth_node = Node(
        package='duburi_vision', executable='depth_estimation_node',
        name=['duburi_depth_', cam], output='screen', ros_arguments=_QUIET,
        parameters=[{
            'camera':            cam,
            'model_path':        LaunchConfiguration('depth_model'),
            'use_tracks':        LaunchConfiguration('tracking'),
            'publish_depth_map': True,
        }],
        condition=IfCondition(LaunchConfiguration('depth')),
    )

    image_viewer = Node(
        package='duburi_vision', executable='vision_display',
        name='duburi_image_view', output='screen',
        parameters=[{
            'camera':          cam,
            'video_file_mode': video_file_mode,
        }],
        condition=IfCondition(LaunchConfiguration('viewer')),
    )

    # Closing the viewer (Q) tears down the whole group so nodes don't orphan.
    shutdown_on_viewer_exit = RegisterEventHandler(
        OnProcessExit(target_action=image_viewer,
                      on_exit=[EmitEvent(event=Shutdown())]),
    )

    return LaunchDescription(
        args + [camera_node, detector_node, tracker_node, depth_node,
                image_viewer, shutdown_on_viewer_exit]
    )
