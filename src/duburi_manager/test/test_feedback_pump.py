"""FeedbackPump vision_provider wiring.

The pump streams Move.Feedback while a goal runs. A vision verb feeds live
signed target-from-centre px via a provider; the pump must publish them as
err_x_px/err_y_px (NaN when there's no fresh vision tick / non-vision verb).
A fake pixhawk + goal_handle keep it ROS-free.
"""

import math
import time

from duburi_manager.auv_manager_node import FeedbackPump


class _FakePixhawk:
    def get_attitude(self):
        return {'yaw': 12.0, 'depth': -0.5}


class _FakeGoalHandle:
    def __init__(self):
        self.feedbacks = []

    def publish_feedback(self, fb):
        self.feedbacks.append(fb)


def _pump_once(vision_provider):
    gh = _FakeGoalHandle()
    pump = FeedbackPump(_FakePixhawk(), gh, vision_provider=vision_provider)
    with pump:
        # let the 0.4s-interval worker publish at least once
        deadline = time.monotonic() + 1.5
        while not gh.feedbacks and time.monotonic() < deadline:
            time.sleep(0.02)
    assert gh.feedbacks, 'pump never published feedback'
    return gh.feedbacks[-1]


def test_feedback_carries_vision_px_when_provided():
    fb = _pump_once(lambda: (-42.0, 8.0))
    assert fb.err_x_px == -42.0
    assert fb.err_y_px == 8.0
    assert 'VIS:' in fb.status_line


def test_feedback_px_nan_when_provider_returns_none():
    fb = _pump_once(lambda: None)
    assert math.isnan(fb.err_x_px) and math.isnan(fb.err_y_px)
    assert 'VIS:' not in fb.status_line


def test_feedback_px_nan_when_no_provider():
    fb = _pump_once(None)
    assert math.isnan(fb.err_x_px) and math.isnan(fb.err_y_px)
