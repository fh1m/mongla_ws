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

import pytest

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


# --------------------------------------------------------------------------- #
#  Coast layer (Part B) -- gap-bridging via /tracks, opt-in (coast_s>0).       #
#  Anti-bug invariants: a live /detections box ALWAYS wins; a coasted box of   #
#  the LOCKED id is conf-exempt; a coasted box of a DIFFERENT id is ignored;   #
#  coast-age is the TRUE time since the last real detection; coast_s=0 -> off. #
# --------------------------------------------------------------------------- #
def _trk(cx, cy, w, h, tid, cls='hole', score=0.9):
    """A /tracks Detection2D: `.id`=str(track_id); score 0.0 == coasted/predicted."""
    d = _det(cx, cy, w, h, cls=cls, score=score)
    d.id = str(tid)
    return d


def _with_tracks(vs, tracks):
    vs._latest_tracks = SimpleNamespace(detections=tracks)
    return vs


def test_coast_off_is_byte_identical():
    # coast_s=0 (default): even with a coasted box available, a no-live frame
    # returns None exactly as before -- the proven path is untouched.
    vs = _with_tracks(_vstate([]), [_trk(320, 240, 50, 50, tid=7, score=0.0)])
    assert vs.bbox_error('hole', locked_id=7, coast_s=0.0) is None


def test_live_detection_overrides_coast():
    # /detections HAS the target AND /tracks has a (stale) coasted box: the LIVE
    # box wins, tagged with its track id, coasted=False. (Fixes "stopped despite
    # a live detection".)
    vs = _vstate([_det(320, 240, 60, 60)])
    _with_tracks(vs, [_trk(320, 240, 60, 60, tid=7, score=0.9),       # real, matches live
                      _trk(560, 240, 50, 50, tid=9, score=0.0)])      # a coast of another id
    s = vs.bbox_error('hole', locked_id=9, coast_s=0.8)
    assert s is not None and s.coasted is False
    assert s.track_id == 7 and abs(s.ex) < 0.1


def test_coast_fills_gap_for_locked_id():
    # No live detection; /tracks has a predicted box of the locked id; the id was
    # seen real recently -> return a coasted Sample (conf-exempt, true age).
    vs = _with_tracks(_vstate([]), [_trk(560, 240, 50, 50, tid=7, score=0.0)])
    vs._last_real[7] = (time.monotonic() - 0.2, 0.9)   # real 0.2 s ago
    s = vs.bbox_error('hole', locked_id=7, coast_s=0.8)
    assert s is not None and s.coasted is True
    assert s.track_id == 7 and s.ex > 0.5
    assert s.age_s == pytest.approx(0.2, abs=0.05)     # TRUE time since last real
    assert s.score == 0.9                              # conf-exempt: last real score


def test_coast_only_for_the_locked_id():
    # A predicted box of a DIFFERENT id must NOT be coasted (would steer onto the
    # wrong same-class object). (Fixes "phantom on the wrong target".)
    vs = _with_tracks(_vstate([]), [_trk(560, 240, 50, 50, tid=3, score=0.0)])
    vs._last_real[7] = (time.monotonic() - 0.2, 0.9)
    assert vs.bbox_error('hole', locked_id=7, coast_s=0.8) is None


def test_coast_window_elapsed_returns_none():
    # Gap longer than coast_s -> loss (None), so the control grace timer fires.
    vs = _with_tracks(_vstate([]), [_trk(560, 240, 50, 50, tid=7, score=0.0)])
    vs._last_real[7] = (time.monotonic() - 1.2, 0.9)   # 1.2 s > coast_s=0.8
    assert vs.bbox_error('hole', locked_id=7, coast_s=0.8) is None


def test_coast_requires_a_prior_lock():
    # locked_id<0 (never acquired) must not coast a random predicted box.
    vs = _with_tracks(_vstate([]), [_trk(560, 240, 50, 50, tid=7, score=0.0)])
    vs._last_real[7] = (time.monotonic() - 0.2, 0.9)
    assert vs.bbox_error('hole', locked_id=-1, coast_s=0.8) is None


# =========================================================================== #
#  age_s means AGE SINCE CAPTURE, not age since the message landed
# =========================================================================== #
"""`_on_detections` used to stamp `time.monotonic()` on arrival, so every
consumer of `age_s` -- `_freshness`, the coast ladder, `is_new_frame` (which
gates the mid-hold torpedo FIRE), `align_stable_frames` -- measured age since
ARRIVAL. The whole capture -> inference -> transport chain, ~32 ms median and
48 p95 measured on the Pi, was invisible to the loop that exists to react to
it. `detector_node` had been passing the capture stamp through correctly the
whole time; the control host resampled it against a local clock.
"""


class _CapturingLog(_Log):
    def __init__(self):
        self.warnings = []

    def warn(self, msg):
        self.warnings.append(msg)

    def warning(self, msg):
        self.warnings.append(msg)


def _msg_stamped_wall(t_wall, dets=()):
    """A Detection2DArray-shaped object whose header carries a wall stamp."""
    sec = int(t_wall)
    return SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(sec=sec,
                                  nanosec=int((t_wall - sec) * 1e9))),
        detections=list(dets))


def _bare_vstate(log=None):
    vs = VisionState(_FakeNode(), camera='t', default_image_size=(640, 480))
    if log is not None:
        vs._log = log
    return vs


def test_age_reflects_the_capture_instant_not_the_arrival():
    """THE FIX. A frame captured 250 ms ago must report ~250 ms of age even
    though the message arrives now -- that delay is exactly what the freshness
    decay exists to respond to."""
    vs = _bare_vstate()
    vs._on_detections(_msg_stamped_wall(time.time() - 0.25))
    age = time.monotonic() - vs._latest_stamp
    assert 0.24 < age < 0.30, f'age {age * 1000:.1f} ms -- the delay was lost'


def test_a_fresh_frame_still_reads_as_fresh():
    """The other half: the fix must not manufacture age out of nothing."""
    vs = _bare_vstate()
    vs._on_detections(_msg_stamped_wall(time.time()))
    assert (time.monotonic() - vs._latest_stamp) < 0.02


def test_a_stamp_slightly_in_the_future_never_yields_a_negative_age():
    """Two hosts' `time.time()` differ by sub-millisecond jitter. A negative
    age would read to the control loop as impossibly fresh and, worse, sail
    through every freshness gate it has."""
    vs = _bare_vstate()
    vs._on_detections(_msg_stamped_wall(time.time() + 0.002))
    assert time.monotonic() - vs._latest_stamp >= 0.0


def test_an_absurd_stamp_falls_back_to_arrival_and_says_so_ONCE():
    """`use_sim_time`, an NTP step, or a publisher still stamping `now()` --
    all produce a stamp we cannot interpret. Arrival time is then the honest
    answer (the pre-fix behaviour) and it must be announced, not silently
    substituted. Once: a per-message warning at 36 Hz is a DoS on the log."""
    log = _CapturingLog()
    vs = _bare_vstate(log)
    for _ in range(5):
        vs._on_detections(_msg_stamped_wall(time.time() - 3600.0))
    assert len(log.warnings) == 1, log.warnings
    assert (time.monotonic() - vs._latest_stamp) < 0.02, \
        'fallback must use arrival time, not the absurd stamp'


def test_a_missing_stamp_falls_back_rather_than_raising():
    """A publisher with no header at all must not take down the callback."""
    vs = _bare_vstate()
    vs._on_detections(SimpleNamespace(detections=[]))
    assert (time.monotonic() - vs._latest_stamp) < 0.02


def test_a_zero_stamp_is_treated_as_absent():
    """An unset builtin_interfaces/Time is 0, which as a wall clock is 1970 --
    it must read as 'no stamp', not as 56 years of age."""
    vs = _bare_vstate()
    vs._on_detections(_msg_stamped_wall(0.0))
    assert (time.monotonic() - vs._latest_stamp) < 0.02


def test_stamps_stay_ordered_so_is_new_frame_still_works():
    """`motion_vision` derives `is_new_frame` from this value increasing.
    Capture stamps increase per frame just as arrival times did, and the fix
    makes the check mean what its docstring already claimed -- a distinct
    FRAME rather than a distinct MESSAGE."""
    vs = _bare_vstate()
    base = time.time() - 0.20
    seen = []
    for i in range(4):
        vs._on_detections(_msg_stamped_wall(base + i * 0.028))
        seen.append(vs._latest_stamp)
    assert seen == sorted(seen) and len(set(seen)) == 4


# =========================================================================== #
#  The coast registry must be bounded, and teardown must be complete
# =========================================================================== #
def test_last_real_is_bounded():
    """`_last_real` was written and never pruned.

    Two consequences, and the second is the dangerous one: unbounded growth
    over a mission, and a RECYCLED tracker id inheriting the previous
    object's sighting timestamp -- so a coast could start from a sighting
    belonging to something else, at that object's score. Both tracker
    backends prune their own registries against exactly this hazard; this
    dict did not.
    """
    vs = _bare_vstate()
    cap = VisionState._LAST_REAL_MAX
    old = time.monotonic() - VisionState._LAST_REAL_HORIZON_S - 1.0
    for i in range(cap * 2):
        vs._last_real[i] = (old, 0.5)
    vs._last_real[999] = (time.monotonic(), 0.9)     # one fresh entry
    vs._evict_last_real()
    assert len(vs._last_real) <= cap, len(vs._last_real)
    assert 999 in vs._last_real, 'the FRESH sighting must survive eviction'


def test_eviction_keeps_the_newest_when_all_are_live():
    """Many live ids is id churn, not staleness. Trim to the newest and warn
    rather than growing in silence."""
    warned = []
    vs = _bare_vstate(_CapturingLog())
    vs._log.warn = warned.append
    now = time.monotonic()
    cap = VisionState._LAST_REAL_MAX
    for i in range(cap + 50):
        vs._last_real[i] = (now - (cap + 50 - i) * 0.001, 0.5)
    vs._evict_last_real()
    assert len(vs._last_real) == cap
    assert (cap + 49) in vs._last_real, 'the newest must be kept'
    assert warned, 'trimming live ids must be announced'


def test_eviction_is_a_no_op_below_the_cap():
    """The common case must not pay for the rare one."""
    vs = _bare_vstate()
    vs._last_real[1] = (time.monotonic() - 1e6, 0.5)     # ancient, but alone
    vs._evict_last_real()
    assert 1 in vs._last_real, 'a lone old entry is not a leak'


def test_close_tears_down_every_subscription():
    """One `try` around all five meant the first failure skipped the rest --
    and it failed every time, on `_sub_img`, a leftover from when this class
    subscribed to `image_raw`. `_sub_vr` was therefore never destroyed."""
    destroyed = []

    class _Node(_FakeNode):
        def destroy_subscription(self, sub):
            destroyed.append(sub)

    vs = VisionState(_Node(), camera='t', default_image_size=(640, 480))
    vs.close()
    assert len(destroyed) == 4, (
        f'{len(destroyed)} of 4 subscriptions torn down -- teardown is '
        f'partial again')
