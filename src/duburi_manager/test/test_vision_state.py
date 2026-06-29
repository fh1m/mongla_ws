"""Selection tests for VisionState.bbox_error -- the continuity lock + conf floor.

The control loop steers on whatever box bbox_error returns, so the close-in
"last-moment misclassification" fix lives here: with a `near` hint + gate it must
return the box NEAREST the last centre (not the largest), drop boxes below
`min_score`, and stay byte-compatible (largest-area) when given no hint.

VisionState needs an rclpy Node only to create subscriptions; a tiny fake node
satisfies that so the selection math unit-tests without a running graph.
"""

import time
from types import SimpleNamespace

from duburi_manager.vision_state import VisionState


class _Log:
    def info(self, *_a, **_k): pass
    def warning(self, *_a, **_k): pass
    def debug(self, *_a, **_k): pass
    def error(self, *_a, **_k): pass


class _FakeNode:
    """Minimal Node: VisionState.__init__ only needs create_subscription + logger."""
    def create_subscription(self, *_a, **_k):
        return object()

    def get_logger(self):
        return _Log()


def _det(cx, cy, w, h, cls='hole', score=0.9):
    bbox = SimpleNamespace(center=SimpleNamespace(x=cx, y=cy), size_x=w, size_y=h)
    hyp = SimpleNamespace(hypothesis=SimpleNamespace(class_id=cls, score=score))
    return SimpleNamespace(bbox=bbox, results=[hyp])


def _vstate(dets, size=(640, 480)):
    vs = VisionState(_FakeNode(), camera='t', default_image_size=size)
    vs._latest_array = SimpleNamespace(detections=dets)
    vs._image_size = size
    vs._latest_stamp = time.monotonic()
    vs._info_seen = True
    return vs


# centre cx=320 -> ex=0.0 ; cx=560 -> ex=0.75 (640 wide)

def test_default_picks_largest_area():
    # A small centred hole + a big right-hand hole -> default returns the BIG one.
    vs = _vstate([_det(320, 240, 40, 40), _det(560, 240, 200, 200)])
    s = vs.bbox_error('hole')
    assert s is not None
    assert s.ex > 0.5            # the right-hand (large) box


def test_continuity_lock_picks_nearest_not_largest():
    # Same two boxes, but locked near centre -> returns the SMALL centred one,
    # ignoring the larger off-centre distractor (the second-hole case).
    vs = _vstate([_det(320, 240, 40, 40), _det(560, 240, 200, 200)])
    s = vs.bbox_error('hole', near=(0.0, 0.0), gate_norm=0.3)
    assert s is not None
    assert abs(s.ex) < 0.1       # the centred box, not the big one at ex~0.75


def test_continuity_lock_none_outside_gate():
    # Locked far to the right but only centred boxes exist -> nothing inside the
    # gate -> None (a transient loss the control loop rides on its grace timer).
    vs = _vstate([_det(320, 240, 40, 40), _det(300, 240, 60, 60)])
    assert vs.bbox_error('hole', near=(0.9, 0.0), gate_norm=0.1) is None


def test_min_score_floor_rejects_low_conf():
    # A big LOW-score box + a smaller HIGH-score box; min_score drops the big
    # low-score one so control steers on the trustworthy detection.
    vs = _vstate([_det(560, 240, 200, 200, score=0.40),
                  _det(320, 240, 60, 60, score=0.95)])
    s = vs.bbox_error('hole', min_score=0.6)
    assert s is not None
    assert abs(s.ex) < 0.1       # the high-score centred box
    assert s.score >= 0.6


def test_no_detection_returns_none():
    vs = _vstate([_det(320, 240, 40, 40, cls='gate')])
    assert vs.bbox_error('hole') is None        # class mismatch -> None
