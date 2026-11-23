"""Every gate on a loop closure, because a wrong one makes the filter CONFIDENT.

A bad position fix is not an ordinary error. `update_position` shrinks the
covariance, so a wrong closure is believed, and a filter that is confidently in
the wrong place is worse than one honestly drifting. These tests are therefore
almost all refusals.

No ROS, no vehicle, no camera. That is deliberate: the vehicle cannot test this
at all today -- it has one USB camera and a barometer reporting "not
initialised" -- so the decision layer is pure precisely so it can be tested
anyway.
"""
import numpy as np
import pytest

from mongla_vision.anchor import loop_closure as lc


class _Pose:
    """The AnchorPose fields `consider` actually reads."""
    def __init__(self, H=None):
        self.H = H


class _Match:
    """The BankPose fields `consider` actually reads."""
    def __init__(self, *, ok=True, label='place:floor', inliers=200,
                 ref_position=(3.0, 4.0), index=0, H='identity'):
        self.ok = ok
        self.label = label
        self.inliers = inliers
        self.ref_position = ref_position
        self.index = index
        # `isinstance`, not `==`: comparing a numpy array to a string raises
        # "truth value of an array is ambiguous", and the sentinel default is
        # the only way to let a test pass H=None explicitly.
        if isinstance(H, str) and H == 'identity':
            H = np.eye(3)
        self.pose = _Pose(H)


def _ctx(**kw):
    base = dict(ref_age_s=120.0, travel_m=5.0, ref_sigma_m=0.30,
                m_per_px=0.002, bank_size=32, centre=(160.0, 120.0))
    base.update(kw)
    return base


# --------------------------------------------------------------------------- #
#  It must be able to say yes, or the refusals below prove nothing.
# --------------------------------------------------------------------------- #
def test_a_good_match_closes():
    c = lc.consider(_Match(), **_ctx())
    assert c.ok, c.reason
    assert c.xy == (3.0, 4.0)
    assert c.sigma > 0


def test_the_fix_is_the_stored_position_not_a_displaced_one():
    """Proximity is claimed, displacement is not -- resolving the offset into
    world x/y would need YAW, and a yaw error rotates the fix somewhere
    confidently wrong."""
    c = lc.consider(_Match(ref_position=(7.5, -2.25)), **_ctx())
    assert c.ok and c.xy == (7.5, -2.25)


# --------------------------------------------------------------------------- #
#  Identity and evidence
# --------------------------------------------------------------------------- #
def test_a_prop_is_not_a_position():
    """Today's forward bank is full of crops of a PERSON who walks about."""
    c = lc.consider(_Match(label='person'), **_ctx())
    assert not c.ok and 'not a place' in c.reason


def test_a_reference_with_no_position_refuses_and_says_why():
    """The state this module was born into: nothing passed position= to enrol,
    so every reference carried None and no closure could ever have fired."""
    c = lc.consider(_Match(ref_position=None), **_ctx())
    assert not c.ok and 'no position' in c.reason


@pytest.mark.parametrize('n', [0, 15, 16, 99])
def test_below_the_bar_refuses(n):
    """15 is the TRACKING bar and sits inside section 24's far distribution
    (p50 9-16). Ample to follow a target, useless to assert a position."""
    c = lc.consider(_Match(inliers=n), **_ctx())
    assert not c.ok, f'{n} inliers must not close a loop'


def test_at_the_bar_closes():
    assert lc.consider(_Match(inliers=lc.CLOSURE_INLIERS), **_ctx()).ok


def test_the_closure_bar_clears_the_far_distribution():
    """Section 24: far p50 9-16, near p50 76-192."""
    assert lc.CLOSURE_INLIERS > 16, 'the bar is inside the FAR distribution'
    assert lc.CLOSURE_INLIERS >= 6 * 15, 'the bar has drifted toward tracking'


# --------------------------------------------------------------------------- #
#  Self-closure: the one that would quietly destroy the filter
# --------------------------------------------------------------------------- #
def test_a_fresh_reference_cannot_close_on_itself():
    """Seconds after enrolment the frame matches the reference made from it at
    hundreds of inliers. The filter would get its own estimate back as a
    low-sigma measurement and shrink its covariance on zero information."""
    c = lc.consider(_Match(inliers=900), **_ctx(ref_age_s=2.0))
    assert not c.ok and 'close on itself' in c.reason


def test_holding_station_cannot_close():
    """Old enough, but the vehicle never went anywhere. Age alone is not
    evidence of a revisit."""
    c = lc.consider(_Match(), **_ctx(ref_age_s=600.0, travel_m=0.1))
    assert not c.ok and 'travelled' in c.reason


def test_the_newest_references_are_excluded_outright():
    c = lc.consider(_Match(index=31), **_ctx(bank_size=32))
    assert not c.ok and 'newest' in c.reason
    assert lc.consider(_Match(index=29), **_ctx(bank_size=32)).ok


# --------------------------------------------------------------------------- #
#  Scale: the measurement that does not exist on this vehicle today
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize('bad', [0.0, -1.0, float('nan')])
def test_no_altitude_refuses_rather_than_assuming_one(bad):
    """Pixels become metres only with a height above the floor, and there is
    no height without a working barometer. A constant here would put a
    plausible number where a measurement is missing."""
    c = lc.consider(_Match(), **_ctx(m_per_px=bad))
    assert not c.ok and 'altitude' in c.reason


def test_no_homography_refuses():
    c = lc.consider(_Match(H=None), **_ctx())
    assert not c.ok and 'homography' in c.reason


def test_a_distant_match_refuses():
    """Recognising the place from far away is not being at it."""
    big = np.eye(3); big[0, 2] = 5000.0          # 5000 px * 0.002 m/px = 10 m
    c = lc.consider(_Match(H=big), **_ctx())
    assert not c.ok and 'too far' in c.reason


def test_the_offset_is_read_from_the_warp_not_from_H_translation():
    """H[0,2]/H[1,2] are the translation ONLY for a pure shift about the
    origin. Every prototype in this lineage read them anyway."""
    H = np.eye(3)
    H[0, 2], H[1, 2] = 100.0, 0.0
    H[2, 0] = 0.01                                # projective: warp != H[:,2]
    off = lc.offset_metres(_Pose(H), 0.002, (160.0, 120.0))
    naive = np.hypot(100.0, 0.0) * 0.002
    assert abs(off - naive) > 1e-9, 'offset is being read off H[:,2]'


# --------------------------------------------------------------------------- #
#  Sigma: closures compose, so a closure inherits its reference's error
# --------------------------------------------------------------------------- #
def test_sigma_inherits_the_reference_uncertainty():
    """A closure onto a badly-localised checkpoint must BE bad. Chains of
    closures onto one drifting anchor is how a filter convinces itself."""
    tight = lc.consider(_Match(), **_ctx(ref_sigma_m=0.05))
    loose = lc.consider(_Match(), **_ctx(ref_sigma_m=3.00))
    assert tight.ok and loose.ok
    assert loose.sigma > tight.sigma * 5


def test_sigma_grows_with_the_apparent_offset():
    near = np.eye(3)
    far = np.eye(3); far[0, 2] = 300.0            # 0.6 m at 0.002 m/px
    a = lc.consider(_Match(H=near), **_ctx())
    b = lc.consider(_Match(H=far), **_ctx())
    assert b.sigma > a.sigma, (
        'offset must reach sigma; it is deliberately NOT applied to the '
        'position, so it has to be paid for somewhere')


def test_sigma_is_never_smaller_than_the_reference_sigma():
    """The fix cannot be more certain than the thing it is measured against."""
    for s in (0.05, 0.3, 1.0, 4.0):
        c = lc.consider(_Match(), **_ctx(ref_sigma_m=s))
        assert c.sigma >= s


def test_every_refusal_says_why():
    """A closure that silently never fires is indistinguishable from one that
    was never wired up -- which is the state this module was written into."""
    for kw in (dict(label='x'), dict(ref_position=None), dict(inliers=3),
               dict(H=None)):
        c = lc.consider(_Match(**kw), **_ctx())
        assert not c.ok and c.reason.strip(), f'silent refusal for {kw}'
    for ctx in (dict(ref_age_s=1.0), dict(travel_m=0.0), dict(m_per_px=0.0)):
        c = lc.consider(_Match(), **_ctx(**ctx))
        assert not c.ok and c.reason.strip(), f'silent refusal for {ctx}'


def test_an_accepted_closure_also_explains_itself():
    c = lc.consider(_Match(), **_ctx())
    assert c.ok and 'inliers' in c.reason and 'off' in c.reason


# --------------------------------------------------------------------------- #
#  The offset must be a DISPLACEMENT of the centre, not a landing point.
#  The first version of offset_metres warped the ORIGIN -- and H @ [0,0,1] is
#  literally H's third column, so it was the very read this file forbids,
#  wearing a hat. Caught by the test above.
# --------------------------------------------------------------------------- #
def test_a_pure_identity_homography_is_zero_offset():
    """The vehicle is exactly where the reference was taken."""
    assert lc.offset_metres(_Pose(np.eye(3)), 0.002, (160.0, 120.0)) == pytest.approx(0.0)


def test_a_pure_shift_moves_the_centre_by_that_shift():
    """Sanity: with no rotation or perspective, the centre moves by H[:,2]."""
    H = np.eye(3)
    H[0, 2] = 250.0
    assert lc.offset_metres(_Pose(H), 0.002, (160.0, 120.0)) == pytest.approx(0.5)


def test_a_rotation_about_the_centre_is_not_a_displacement():
    """The vehicle turned on the spot over the same patch of floor.

    Reading H[:,2] calls that a large translation and hands the filter a fix
    from somewhere it never went. Measuring the CENTRE's displacement calls it
    what it is: zero.
    """
    import math
    cx, cy = (160.0, 120.0)
    th = math.radians(30.0)
    c, s = math.cos(th), math.sin(th)
    R = np.array([[c, -s, cx - c * cx + s * cy],
                  [s,  c, cy - s * cx - c * cy],
                  [0.0, 0.0, 1.0]])
    assert lc.offset_metres(_Pose(R), 0.002, (cx, cy)) == pytest.approx(0.0, abs=1e-9)
    assert np.hypot(R[0, 2], R[1, 2]) > 10.0, (
        'the fixture needs a large H[:,2], or it cannot tell the two readings '
        'apart and proves nothing')


# --------------------------------------------------------------------------- #
#  The guards are about the MATCHED place (truth known by construction)
# --------------------------------------------------------------------------- #
def _swim(log, path, dt=0.5):
    """Drive a PlaceLog the way `_place_tick` does: step, then maybe take a
    place. Returns {position: index} for the places taken."""
    taken = {}
    for i, xy in enumerate(path):
        t = i * dt
        log.step(xy)
        if log.should_enrol(t, xy):
            log.note(t, xy, 0.25)
            taken[xy] = len(taken)
    return taken, (len(path) - 1) * dt


def _loop():
    """Out 4 m along x, across 4 m, back, and home to 0.1 m short of the
    start, 80 s later. Short, because the node asks before it enrols: a place
    taken on the very spot first would be the newest, and a tie resolves to
    the newest place, which refuses."""
    out = [(0.1 * i, 0.0) for i in range(40)]
    across = [(4.0, 0.1 * i) for i in range(40)]
    back = [(4.0 - 0.1 * i, 4.0) for i in range(40)]
    home = [(0.0, 4.0 - 0.1 * i) for i in range(40)]
    return out + across + back + home


def test_a_revisit_of_an_old_place_closes():
    """The case the whole path exists for. With the guards reading the
    NEWEST place instead (age <= 5 s, travel < 1 m) this refused every time:
    measured by putting the old expression back, which fails here."""
    log = lc.PlaceLog()
    taken, now = _swim(log, _loop())
    first = next(iter(taken))
    age, travel, sigma = log.since(first, now)
    assert age == pytest.approx(79.5)
    assert travel == pytest.approx(4.0 * 2 ** 0.5, rel=0.05)
    assert sigma == 0.25
    c = lc.consider(_Match(ref_position=first, index=taken[first]),
                    ref_age_s=age, travel_m=travel, ref_sigma_m=sigma,
                    m_per_px=0.002, bank_size=len(taken),
                    centre=(160.0, 120.0))
    assert c.ok, c.reason


def test_holding_station_over_an_old_place_still_cannot_close():
    """Old but never away: the jitter of a filter at rest is not travel."""
    log = lc.PlaceLog()
    rng = np.random.default_rng(3)
    path = [tuple(rng.normal(0.0, 0.05, 2)) for _ in range(200)]
    _swim(log, path)
    age, travel, _sigma = log.since(log._taken[0][1:3], 99.5)
    assert age > lc.MIN_REF_AGE_S
    assert travel < lc.MIN_TRAVEL_M


def test_a_place_this_run_did_not_take_has_no_history():
    """A bank loaded from disk: no age, no travel, no sigma, so it refuses."""
    log = lc.PlaceLog()
    age, travel, sigma = log.since((7.0, 7.0), 10.0)
    assert age != age and travel != travel and sigma != sigma
    c = lc.consider(_Match(ref_position=(7.0, 7.0)), ref_age_s=age,
                    travel_m=travel, ref_sigma_m=sigma, m_per_px=0.002,
                    bank_size=32, centre=(160.0, 120.0))
    assert not c.ok


def test_the_offset_is_measured_at_the_backend_s_own_centre():
    """The centre was a constant for a 320x240 grid. On a 640x480 export a
    rotation about the true centre read as metres of travel, so a revisit
    was refused as too far, or a near miss passed as one."""
    import math
    cx, cy = 320.0, 240.0
    th = math.radians(30.0)
    c, s = math.cos(th), math.sin(th)
    R = np.array([[c, -s, cx - c * cx + s * cy],
                  [s,  c, cy - s * cx - c * cy],
                  [0.0, 0.0, 1.0]])
    assert lc.offset_metres(_Pose(R), 0.002, (cx, cy)) == pytest.approx(0.0, abs=1e-9)
    assert lc.offset_metres(_Pose(R), 0.002, (160.0, 120.0)) > 0.1


def test_the_lock_passes_its_bank_s_grid_centre(monkeypatch):
    """Drives the lock's own closure step, which no test reached: the centre
    `consider` is given must be the bank backend's, not a 320x240 guess."""
    import types
    from mongla_vision.lock_inputs import LockInputsMixin
    seen = {}

    def fake_consider(bp, **kw):
        seen.update(kw)
        return lc.Closure(False, 'no match')
    monkeypatch.setattr(lc, 'consider', fake_consider)
    anchor = types.SimpleNamespace(
        size=3, grid_centre=(320.0, 240.0),
        locate=lambda *a, **k: None,
        enrol=lambda *a, **k: types.SimpleNamespace(accepted=False))
    node = types.SimpleNamespace(
        _closer_on=True, _anchor=anchor, _odom_xy=(1.0, 2.0), _odom_t=99.9,
        _ODOM_MAX_AGE_S=LockInputsMixin._ODOM_MAX_AGE_S,
        _odom_sigma=float('nan'), _places=lc.PlaceLog(),
        _m_per_px=lambda: 0.002, _closure_refusals=__import__('collections').Counter())
    node._count_refusal = lambda k: LockInputsMixin._count_refusal(node, k)
    LockInputsMixin._place_look(node, np.zeros((480, 640), np.uint8), None, 100.0)
    assert seen['centre'] == (320.0, 240.0)


def test_stale_odometry_neither_searches_nor_enrols():
    """The lock stored when odometry arrived and never read it, so a dead
    localiser left closure searching and enrolling at a position however old.
    /mongla/odom comes from a 10 Hz timer (worst gap 102.88 ms on Kutum,
    measured-bars section 91); older than the bar, nothing runs."""
    import collections
    import types
    from mongla_vision.lock_inputs import LockInputsMixin
    calls = []
    anchor = types.SimpleNamespace(
        size=3, grid_centre=(320.0, 240.0),
        locate=lambda *a, **k: calls.append('locate'),
        enrol=lambda *a, **k: calls.append('enrol') or types.SimpleNamespace(accepted=False))
    places = lc.PlaceLog()
    places.should_enrol = lambda now, xy: True
    node = types.SimpleNamespace(
        _closer_on=True, _anchor=anchor, _odom_xy=(1.0, 2.0), _odom_sigma=0.1,
        _places=places, _m_per_px=lambda: 0.002,
        _closure_refusals=collections.Counter(),
        _ODOM_MAX_AGE_S=LockInputsMixin._ODOM_MAX_AGE_S)
    node._count_refusal = lambda k: LockInputsMixin._count_refusal(node, k)
    gray = np.zeros((480, 640), np.uint8)
    node._odom_t = 100.0 - 2 * LockInputsMixin._ODOM_MAX_AGE_S
    LockInputsMixin._place_look(node, gray, None, 100.0)
    assert calls == []
    assert node._closure_refusals['stale odometry'] == 1
    node._odom_t = 100.0 - 0.1
    LockInputsMixin._place_look(node, gray, None, 100.0)
    assert 'enrol' in calls


def test_odometry_that_arrives_on_time_with_a_frozen_stamp_is_stale(monkeypatch):
    """The localiser publishes on a 10 Hz timer whether or not its inputs
    still arrive, and stamps each message with its newest input. Arriving on
    time with the same stamp is a position that stopped updating, so the age
    is taken from the last time the stamp moved."""
    import types
    from mongla_vision import lock_inputs as li
    clock = [100.0]
    monkeypatch.setattr(li.time, 'monotonic', lambda: clock[0])
    node = types.SimpleNamespace(_odom_stamp=None, _odom_t=0.0)

    def odom(stamp_s):
        return types.SimpleNamespace(
            header=types.SimpleNamespace(stamp=types.SimpleNamespace(
                sec=int(stamp_s), nanosec=int((stamp_s % 1) * 1e9))),
            pose=types.SimpleNamespace(
                pose=types.SimpleNamespace(position=types.SimpleNamespace(x=1.0, y=2.0, z=0.5)),
                covariance=[0.01] * 36))
    li.LockInputsMixin._on_odom(node, odom(50.0))
    assert node._odom_t == 100.0
    for _ in range(10):                        # a second of on-time, frozen messages
        clock[0] += 0.1
        li.LockInputsMixin._on_odom(node, odom(50.0))
    assert clock[0] - node._odom_t > li.ODOM_MAX_AGE_S
    clock[0] += 0.1
    li.LockInputsMixin._on_odom(node, odom(50.1))
    assert node._odom_t == clock[0]
