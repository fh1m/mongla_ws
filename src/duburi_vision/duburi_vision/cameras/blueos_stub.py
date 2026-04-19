"""BlueOSCamera — placeholder for BlueOS' MJPEG / RTSP camera streams.

BlueOS exposes its USB cams via the camera manager service on the Pi:
    http://192.168.2.1:8554/streamN              (RTSP)
    http://192.168.2.1:6020/mavlink-camera-manager (REST)

Planned implementation: open the RTSP URL with cv2.VideoCapture under a
GStreamer backend, surface the same `Camera` interface. The factory hands
this source the URL via kwargs.

Until that lands, asking for source='blueos' fails loudly.
"""

from .camera import Camera


class BlueOSCamera(Camera):
    name        = 'blueos'
    source_kind = 'blueos'

    def __init__(self, *_, **__):
        raise NotImplementedError(
            "blueos camera not implemented yet. "
            "Use source='webcam' (laptop dev) or source='ros_topic' (Gazebo / BlueOS).")
