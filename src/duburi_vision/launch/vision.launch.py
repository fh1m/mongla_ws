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
    # Live forward camera, pool gate model. On the Jetson pass the PORT-STABLE
    # by-path symlink (the int device index 0/2 is wrong there -> camera_node
    # crashes at startup without it):
    ros2 launch duburi_vision vision.launch.py camera:=forward \\
        device_path:=/dev/duburi_cam_forward \\
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

# Non-ROS stderr/stdout noise that BYPASSES the --log-level filter above (it is
# not rcl logging, so the level pin can't touch it). Squelch at the source via
# env so the console shows only Mongla [DET]/[YOLO]/[VIS]/[DISP] lines:
#   PYTHONWARNINGS=ignore -> kills numpy's aarch64 "smallest subnormal is zero"
#       UserWarning (4×/node) and the supervision/trackers target=None
#       FutureWarning.
#   EGL_LOG_LEVEL=fatal   -> kills libEGL "DRI3: failed to query the version"
#       (emitted by the GL stack under `ssh -X` forwarding).
# TensorRT's one-shot [TRT] startup lines come from its own C++ logger inside
# ultralytics and intentionally stay -- they print once and the cross-device
# "engine plan across different models" line is a real heads-up worth seeing.
# To get the raw warnings back for debugging, run the node via `ros2 run`
# (which doesn't inherit this env) or `PYTHONWARNINGS=default`.
_QUIET_ENV = {
    'PYTHONWARNINGS': 'ignore',
    'EGL_LOG_LEVEL':  'fatal',
}


def generate_launch_description():
    args = [
        DeclareLaunchArgument('camera',        default_value='forward',
                              description='Camera profile (forward|downward|sim_front|laptop|...). '
                                          'Drives node names: duburi_detector_<camera>.'),
        DeclareLaunchArgument('device',        default_value='-1',
                              description='/dev/videoN index; -1 = use profile default'),
        # PORT-STABLE identity, same as vision_dual. On the Jetson the forward/downward
        # USB cameras do NOT enumerate at a stable int index (the raw /dev/videoN
        # renumbers on reboot/re-plug and silently swaps the two identical cameras).
        # Leave this EMPTY and the node auto-binds /dev/duburi_cam_<camera> WHEN THAT
        # SYMLINK EXISTS (Jetson) -- so a single-cam launch always opens the right
        # physical camera. A non-empty value forces a specific device (overrides the
        # symlink); on a dev box with no symlink it falls back to the int `device`.
        DeclareLaunchArgument('device_path',   default_value='',
                              description='override device (empty = auto /dev/duburi_cam_<camera> '
                                          'symlink if present, else int `device`)'),
        DeclareLaunchArgument('width',         default_value='640'),
        DeclareLaunchArgument('height',        default_value='480'),
        # 0 = take the fps from the named camera profile. A non-zero value
        # overrides it. This defaulted to 30 and was ALSO wired into
        # publish_rate_hz, so every named profile's fps was silently
        # overridden by this default -- `camera:=pi_forward` (210 fps)
        # published at 29.999 Hz and nothing said so.
        DeclareLaunchArgument('fps',           default_value='0',
                              description='0 = use the camera profile fps'),
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
        DeclareLaunchArgument('debug_image_hz', default_value='10.0',
                              description='Annotated image_debug publish rate (raise toward inference '
                                          'FPS for a smoother web/mission_web stream)'),
        DeclareLaunchArgument('viewer',        default_value='true',
                              description='Open the OpenCV vision_display HUD (viewer:=false = headless)'),
        DeclareLaunchArgument('tracking',      default_value='true',
                              description='Start tracker_node (Roboflow OC-SORT/ByteTrack + Kalman)'),
        DeclareLaunchArgument('tracker_type',  default_value='ocsort',
                              description='Tracker engine: ocsort (default) | bytetrack | legacy_bytetrack'),
        DeclareLaunchArgument('track_buffer',  default_value='30'),
        DeclareLaunchArgument('min_hits',      default_value='1'),
        # max_predict is the 4th rung of the coast ladder: the Kalman smoother
        # drops a track (filters it off /tracks) after this many predicted
        # frames, so it MUST exceed vision.coast_s in frames or the control coast
        # truncates early. 30 frames = 1.5 s at 20 Hz, headroom over coast_s~0.8.
        DeclareLaunchArgument('max_predict',   default_value='30'),
        DeclareLaunchArgument('depth',         default_value='false',
                              description='Start depth_estimation_node (monocular vis_range)'),
        DeclareLaunchArgument('depth_model',   default_value='',
                              description='DA V2-Small ONNX path; empty = bbox-area fallback'),
        # Downward optical-flow distance estimator (DVL-free). distance:=true starts
        # it so calc_distance('start'/'stop') works (use with camera:=downward).
        DeclareLaunchArgument('distance',        default_value='false',
                              description='Start the optical-flow distance node (calc_distance).'),
        DeclareLaunchArgument('pool_depth_m',    default_value='4.0',
                              description='Water column surface->floor (m); metric scale for flow.'),
        DeclareLaunchArgument('camera_focal_px', default_value='500.0',
                              description='Camera f_px (intrinsics calibration; distance scale rides on it).'),
        DeclareLaunchArgument('hud_distance',    default_value='true',
                              description='HUD pre-arms the distance panel.'),
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
        output='screen', ros_arguments=_QUIET, additional_env=_QUIET_ENV,
        parameters=[{
            'profile':         profile_expr,
            'source':          source_expr,
            'name':            cam,
            'topic':           topic,
            'device':          LaunchConfiguration('device'),
            # PORT-STABLE identity: an empty device_path falls back to the
            # /dev/duburi_cam_<camera> udev symlink WHEN IT EXISTS (Jetson), else
            # '' -> the int device index (dev box). Keeps forward/downward immune
            # to /dev/videoN renumbering on reboot/re-plug (the raw index is NOT
            # stable — the enumeration order flips, swapping the two identical cams).
            'device_path':     PythonExpression([
                "'", LaunchConfiguration('device_path'), "'",
                " or (lambda p: p if __import__('os').path.exists(p) else '')"
                "('/dev/duburi_cam_' + '", cam, "')"]),
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
        output='screen', ros_arguments=_DET_QUIET, additional_env=_QUIET_ENV,
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
            'debug_image_hz':      LaunchConfiguration('debug_image_hz'),
        }],
    )

    tracker_node = Node(
        package='duburi_vision', executable='tracker_node', name=trk_node,
        output='screen', ros_arguments=_QUIET, additional_env=_QUIET_ENV,
        parameters=[{
            'camera':             cam,
            'tracker_type':       LaunchConfiguration('tracker_type'),
            # Clamp the tracker's confidence gates to the detector's floor.
            # Above it NO track is created and /tracks stays empty while every
            # node looks healthy -- measured at 0.0 % presence on real footage.
            'detector_conf':      LaunchConfiguration('conf'),
            'track_buffer':       LaunchConfiguration('track_buffer'),
            'min_hits':           LaunchConfiguration('min_hits'),
            'max_predict_frames': LaunchConfiguration('max_predict'),
        }],
        condition=IfCondition(LaunchConfiguration('tracking')),
    )

    depth_node = Node(
        package='duburi_vision', executable='depth_estimation_node',
        name=['duburi_depth_', cam], output='screen', ros_arguments=_QUIET,
        additional_env=_QUIET_ENV,
        parameters=[{
            'camera':            cam,
            'model_path':        LaunchConfiguration('depth_model'),
            'use_tracks':        LaunchConfiguration('tracking'),
            'publish_depth_map': True,
        }],
        condition=IfCondition(LaunchConfiguration('depth')),
    )

    distance_node = Node(
        package='duburi_vision', executable='distance_estimation_node',
        name='duburi_distance_estimator', output='screen', ros_arguments=_QUIET,
        additional_env=_QUIET_ENV,
        parameters=[{
            'camera':          cam,
            'pool_depth_m':    LaunchConfiguration('pool_depth_m'),
            'camera_focal_px': LaunchConfiguration('camera_focal_px'),
        }],
        condition=IfCondition(LaunchConfiguration('distance')),
    )

    image_viewer = Node(
        package='duburi_vision', executable='vision_display',
        name='duburi_image_view', output='screen', additional_env=_QUIET_ENV,
        parameters=[{
            'camera':          cam,
            'video_file_mode': video_file_mode,
            'hud_distance':    LaunchConfiguration('hud_distance'),
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
                distance_node, image_viewer, shutdown_on_viewer_exit]
    )
