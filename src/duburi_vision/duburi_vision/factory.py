"""Single dispatch point: source-name -> Camera instance.

Used by `camera_node` and `vision_node` at startup. Adding a new camera
source = one line in `BUILDERS` plus the source class itself; nothing else
changes. Mirrors `duburi_sensors.factory.make_yaw_source` exactly.
"""


def _build_webcam(*, device=0, width=640, height=480, fps=30,
                  frame_id='laptop_cam', name='laptop', logger=None, **_):
    from .cameras.webcam import WebcamCamera
    return WebcamCamera(
        device=device, width=width, height=height, fps=fps,
        frame_id=frame_id, name=name, logger=logger)


def _build_ros_topic(*, node=None, topic=None, frame_id='ros_cam', name='ros_topic',
                     expected_width=0, expected_height=0, expected_fps=30.0,
                     logger=None, **_):
    if node is None:
        raise ValueError(
            "source='ros_topic' requires node=<rclpy.node.Node>. "
            "Build this from inside a ROS node (camera_node).")
    if not topic:
        raise ValueError("source='ros_topic' requires topic=<str>")
    from .cameras.ros_topic import RosTopicCamera
    return RosTopicCamera(
        node=node, topic=topic, frame_id=frame_id, name=name,
        expected_width=expected_width, expected_height=expected_height,
        expected_fps=expected_fps, logger=logger)


def _build_jetson_stub(**_):
    from .cameras.jetson_stub import JetsonCamera
    return JetsonCamera()


def _build_blueos_stub(**_):
    from .cameras.blueos_stub import BlueOSCamera
    return BlueOSCamera()


def _build_mavlink_stub(**_):
    from .cameras.mavlink_stub import MavlinkCamera
    return MavlinkCamera()


def _build_video_file(*, path='', loop=True, width=0, height=0, fps=0,
                      frame_id='video_cam', name='video', logger=None, **_):
    if not path:
        raise ValueError(
            "source='video_file' requires path=<str>. "
            "Pass video_file:=<path> in the launch file or path=<str> in Python.")
    from .cameras.video_file import VideoFileCamera
    return VideoFileCamera(
        path=path, width=width, height=height, fps=fps,
        loop=loop, frame_id=frame_id, name=name, logger=logger)


# Stubs are wired up so users hitting `source='jetson'` get the per-stub
# "not implemented yet" message instead of a generic "unknown source"
# error from make_camera. Same UX as duburi_sensors.factory.
BUILDERS = {
    'webcam':      _build_webcam,
    'ros_topic':   _build_ros_topic,
    'video_file':  _build_video_file,
    'jetson':      _build_jetson_stub,
    'blueos':      _build_blueos_stub,
    'mavlink':     _build_mavlink_stub,
}


def make_camera(source_name, /, **kwargs):
    """Return a configured Camera for the given source name.

    Parameters
    ----------
    source_name : str
        One of `BUILDERS` keys: 'webcam', 'ros_topic', 'jetson', ...
        Positional-only so kwargs can carry their own `name` (camera
        instance label) without colliding with the dispatch key.
    **kwargs
        Source-specific kwargs (see each `_build_*` for the contract).

    Raises
    ------
    ValueError
        If `source_name` is unknown or required kwargs are missing.
    NotImplementedError
        If `source_name` points at a stub source (jetson / blueos / mavlink).
    Exception
        Hardware/driver errors propagate (e.g. cv2 fails to open). Caller
        is responsible for failing loudly at startup -- no silent fallback.
    """
    key = (source_name or '').strip().lower()
    if key not in BUILDERS:
        known = ', '.join(sorted(BUILDERS))
        raise ValueError(f"unknown camera source {source_name!r}. known: {known}")
    return BUILDERS[key](**kwargs)


def make_camera_from_profile(profile, *, node=None, logger=None):
    """Convenience: build from a CAMERA_PROFILES dict (or a copy thereof).

    The profile's `source` key picks the builder; everything else is passed
    through as kwargs (including `name` -- the camera-instance label, which
    is what the source classes use to populate `info()['name']`). `node`
    and `logger` are injected so the caller doesn't have to remember which
    sources need them.
    """
    pf = dict(profile)
    source = pf.pop('source', None)
    if source is None:
        raise ValueError("profile must contain a 'source' key")
    return make_camera(source, node=node, logger=logger, **pf)
