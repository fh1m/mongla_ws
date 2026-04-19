"""JetsonCamera — placeholder for the on-vehicle Jetson Orin Nano cameras.

Planned implementation: the two Blue Robotics Low-Light HD USB cams on the
Duburi 4.2. Same V4L2 path as `webcam` plus Jetson-specific bits — locked
exposure, MJPG pixel format, and an explicit `gst-launch` pipeline so the
ISP is the one doing the format conversion (not python-opencv on the CPU).

Until that lands, asking for source='jetson' fails loudly so nobody runs a
mission thinking they have a forward camera.
"""

from .camera import Camera


class JetsonCamera(Camera):
    name        = 'jetson'
    source_kind = 'jetson'

    def __init__(self, *_, **__):
        raise NotImplementedError(
            "jetson camera not implemented yet. "
            "Use source='webcam' (laptop dev) or source='ros_topic' (Gazebo / BlueOS).")
