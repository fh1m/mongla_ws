"""Keyboard handling for the vision display: camera switching and video playback.

The keys arrive in the display process; playback lives in the camera process,
so each key becomes a topic or service call the camera answers.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from std_msgs.msg import Float32, Int32
from std_srvs.srv import SetBool

if TYPE_CHECKING:
    from .display_node import VisionDisplayNode


# Arrow key codes from cv2.waitKey on Linux (after & 0xFF they become 81-84).
# Use waitKeyEx() codes instead — waitKeyEx returns full 32-bit extended codes.
_KEY_LEFT  = 0xFF51  # left  arrow (XK_Left)
_KEY_RIGHT = 0xFF53  # right arrow (XK_Right)
_KEY_UP    = 0xFF52  # up    arrow (XK_Up)
_KEY_DOWN  = 0xFF54  # down  arrow (XK_Down)


# ---------------------------------------------------------------------------
# Video playback helpers
# ---------------------------------------------------------------------------

def _send_seek(node: VisionDisplayNode, seconds: float) -> None:
    if node._seek_pub is not None:
        node._seek_pub.publish(Float32(data=float(seconds)))


def _send_seek_frame(node: VisionDisplayNode, frames: int) -> None:
    if node._seek_frame_pub is not None:
        node._seek_frame_pub.publish(Int32(data=int(frames)))


# The ladder lives with the camera that owns playback speed. This file used
# to carry its own identical copy, and `VideoFileCamera` carried step-up and
# step-down methods that nothing called -- two copies of one tuple, in two
# processes, with a comment in the camera admitting it ("same values understood
# by display_node keybinds"). The STEPPING has to happen here, because the
# camera is a different process and the keypress arrives here; the LADDER does
# not.
from mongla_vision.cameras.video_file import SPEED_STEPS as _SPEED_STEPS


def _speed_step_up(cur: float) -> float:
    for s in _SPEED_STEPS:
        if s > cur + 0.01:
            return s
    return _SPEED_STEPS[-1]


def _speed_step_down(cur: float) -> float:
    for s in reversed(_SPEED_STEPS):
        if s < cur - 0.01:
            return s
    return _SPEED_STEPS[0]


def _send_speed(node: VisionDisplayNode, speed: float) -> None:
    node._video_speed = speed
    if node._speed_pub is not None:
        node._speed_pub.publish(Float32(data=float(speed)))


def _send_pause(node: VisionDisplayNode, pause: bool) -> None:
    """Update pause state optimistically then fire service call."""
    node._is_paused = pause          # update now — don't wait for service ACK
    if node._pause_client is None or not node._pause_client.service_is_ready():
        return
    req = SetBool.Request()
    req.data = pause
    node._pause_client.call_async(req)


def _handle_camera_keys(node: VisionDisplayNode, key: int) -> None:
    """Handle runtime camera switching keys.

    Subscription create/destroy is deferred to _process_pending_sub_action (timer cb on
    the executor thread) to avoid wait-set races with rclpy.spin on the daemon thread.
    """
    if key < 0:
        return
    if key == ord('f'):
        node._pending_sub_action = 'forward'
    elif key == ord('d'):
        node._pending_sub_action = 'downward'
    elif key == ord('D'):
        node._show_depth_map = not node._show_depth_map


def _handle_video_keys(node: VisionDisplayNode, key: int) -> None:
    """Process video playback keyboard shortcuts — called in BOTH empty and frame paths."""
    if key < 0 or not node._video_mode:
        return
    if key == ord(' '):
        _send_pause(node, not node._is_paused)
    elif key == _KEY_LEFT:
        _send_seek(node, -1.0)
    elif key == _KEY_RIGHT:
        _send_seek(node, 1.0)
    elif key == _KEY_UP:
        _send_seek(node, 10.0)
    elif key == _KEY_DOWN:
        _send_seek(node, -10.0)
    elif key == ord(','):
        _send_seek_frame(node, -1)
    elif key == ord('.'):
        _send_seek_frame(node, 1)
    elif key == ord('['):
        _send_speed(node, _speed_step_down(node._video_speed))
    elif key == ord(']'):
        _send_speed(node, _speed_step_up(node._video_speed))
