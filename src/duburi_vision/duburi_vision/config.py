"""Named camera profiles.

The yaml file at `share/duburi_vision/config/cameras.yaml` is the user-facing
copy operators edit. This dict is the in-code default — handy for tests and
for `make_camera('laptop')` style one-liners that don't need to read a file.

If you add a profile here, add the same row to cameras.yaml so launch files
can find it without code changes.
"""


CAMERA_PROFILES = {
    # ---- Laptop dev (webcam) -------------------------------------------- #
    'laptop': {
        'source':   'webcam',
        'device':   0,
        'width':    640,
        'height':   480,
        'fps':      30,
        'frame_id': 'laptop_cam',
    },

    # ---- Gazebo SITL: forward / down -- topics depend on the world ----- #
    # The bluerov2_gz model used today does NOT ship a camera plugin; you
    # need to add a <sensor type="camera"> + <plugin filename="ignition-..."/>
    # block to model.sdf. Until then point this at whatever Image topic is
    # available (e.g. a re-published webcam over a ros_gz bridge).
    'sim_front': {
        'source':         'ros_topic',
        'topic':          '/duburi/sim/front_camera/image_raw',
        'expected_width': 640,
        'expected_height': 480,
        'expected_fps':    30,
        'frame_id':       'front_cam',
    },
    'sim_bottom': {
        'source':         'ros_topic',
        'topic':          '/duburi/sim/bottom_camera/image_raw',
        'expected_width': 640,
        'expected_height': 480,
        'expected_fps':    30,
        'frame_id':       'bottom_cam',
    },

    # ---- Future: on-vehicle (raise NotImplementedError today) ---------- #
    'jetson_front':  {'source': 'jetson'},
    'jetson_bottom': {'source': 'jetson'},
    'blueos':        {'source': 'blueos'},
    'mavlink':       {'source': 'mavlink'},
}


def get_profile(name):
    """Return a copy of the named profile, or raise ValueError listing
    the known profiles."""
    if name not in CAMERA_PROFILES:
        known = ', '.join(sorted(CAMERA_PROFILES))
        raise ValueError(f"unknown camera profile {name!r}. known: {known}")
    return dict(CAMERA_PROFILES[name])
