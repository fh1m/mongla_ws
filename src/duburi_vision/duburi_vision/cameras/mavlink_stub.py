"""MavlinkCamera — placeholder for MAVLink CAMERA_* protocol.

ArduSub / BlueOS expose CAMERA_INFORMATION + CAMERA_TRACKING_IMAGE_STATUS
+ VIDEO_STREAM_INFORMATION over MAVLink. Some payloads (e.g. SIYI ZR10)
publish their video over MAVLink-FTP or a paired RTSP URL fetched via
VIDEO_STREAM_INFORMATION.

Planned implementation: query VIDEO_STREAM_INFORMATION at startup, fetch
the RTSP URL, then delegate to the same path BlueOS uses. The factory
will need a `pixhawk` kwarg to share the existing MAVLink connection.

Until that lands, asking for source='mavlink' fails loudly.
"""

from .camera import Camera


class MavlinkCamera(Camera):
    name        = 'mavlink'
    source_kind = 'mavlink'

    def __init__(self, *_, **__):
        raise NotImplementedError(
            "mavlink camera not implemented yet. "
            "Use source='webcam' (laptop dev) or source='ros_topic' (Gazebo / BlueOS).")
