"""A planar target's mirror branch, resolved by geometry -- never by a vote.

Issue #55. Every test here is projectPoints -> the shipped `solve_pnp` ->
`PoseCluster` -> `anchor_from`. Nothing constructs a yaw by hand: the old tests
did, and that is how they came to encode "the false branch slopes +1" -- a
claim a single projected frame disproves (under pure rotation BOTH slope -1).

Scene: a 0.30 m planar target 3 m ahead, its normal turned 25 deg. The camera
yaws by psi (FRD, right positive) and may translate sideways. Truth: the
target's world yaw is +25 deg in every frame, and its camera-relative yaw now
is 25 - psi_now.

What would FALSIFY the rule, test by test, is stated in each docstring.
"""
import math

import numpy as np
import pytest

cv2 = pytest.importorskip('cv2')

from mongla_localization.heading_anchor import anchor_from   # noqa: E402
from mongla_localization.pose_cluster import (               # noqa: E402
    MIRROR_SLOPE, RULE_VIEWPOINT, PoseCluster, PoseSample)
from mongla_vision.anchor.pose import solve_pnp              # noqa: E402

OBJ = np.array([(x, y, 0.0) for x in np.linspace(-.15, .15, 5)
                for y in np.linspace(-.15, .15, 5)], np.float64)
K = np.array([[600., 0, 320], [0, 600., 180], [0, 0, 1.]])
THETA = 25.0


def _frames(*, lateral=0.0, yaw_sweep=0.0, n=10, rng=3.0, pitch=0.0,
            noise=0.0, seed=0, swap=()):
    """One `PoseSample` per frame, from the shipped producer.

    `swap` lists frames where the producer's PRIMARY is the other branch --
    the controlled wrong-majority injection #55 uses. It is fault injection,
    not a measurement of how often a detector does this.
    """
    g = np.random.default_rng(seed)
    Rt, _ = cv2.Rodrigues(np.array([math.radians(pitch), math.radians(THETA), 0.]))
    T = np.array([0., 0., rng])
    out = []
    for i in range(n):
        f = i / max(n - 1, 1)
        psi = yaw_sweep * f
        C = np.array([lateral * f, 0., 0.])
        Rc, _ = cv2.Rodrigues(np.array([0., math.radians(-psi), 0.]))
        rv, _ = cv2.Rodrigues(Rc @ Rt)
        img, _ = cv2.projectPoints(OBJ, rv, Rc @ (T - C), K, np.zeros(5))
        img = img.reshape(-1, 2) + g.normal(0.0, noise, (len(OBJ), 2))
        tp = solve_pnp(OBJ, img, K)
        assert tp.ok, tp.reason
        # A producer cannot report a branch it never had: no swap without one.
        do_swap = i in swap and math.isfinite(tp.alt_yaw_deg)
        y, a = (tp.alt_yaw_deg, tp.yaw_deg) if do_swap else (tp.yaw_deg, tp.alt_yaw_deg)
        out.append(PoseSample(t=float(i), yaw_deg=y, range_m=tp.range_m,
                              ambiguity=tp.ambiguity, reproj_px=tp.reproj_px,
                              n_points=tp.n_points, vehicle_yaw_deg=psi,
                              alt_yaw_deg=a, bearing_deg=tp.bearing_deg))
    return out


def _fuse(samples):
    c = PoseCluster()
    for s in samples:
        c.add(s)
    return c.fuse(now=samples[-1].t)


def _true_now(samples):
    return THETA - samples[-1].vehicle_yaw_deg


# ── the producer ─────────────────────────────────────────────────────────────
def test_the_producer_reports_both_branches_and_the_bearing():
    """Falsified if the alternate is the reported yaw again (chosen by index
    after the polish walked branches), or if the bearing has the wrong sign."""
    s = _frames(n=1)[0]
    assert s.yaw_deg == pytest.approx(THETA, abs=0.5)
    assert s.alt_yaw_deg == pytest.approx(-THETA, abs=0.5)
    right = _frames(n=2, lateral=-0.6)[-1]       # camera moves LEFT
    assert right.bearing_deg > 5.0, 'target must appear to the RIGHT'


# ── pure rotation: unresolvable, whatever the majority ──────────────────────
@pytest.mark.parametrize('swap', [(0, 1, 3, 4, 6, 7, 9), (2, 5, 8), (0, 2, 4, 6, 8)],
                         ids=['7-3 wrong', '7-3 right', '5-5'])
def test_a_turn_in_place_leaves_the_mirror_pair_UNRESOLVED(swap):
    """Both branches are constant in the world when only the heading changes,
    so both fit every frame. Falsified by ANY decision here -- the old fuser
    decided the 7-3-wrong case and anchored 54 deg against a true 9."""
    s = _frames(yaw_sweep=9.0, swap=swap)
    out = _fuse(s)
    assert not out.decided, f'decided {out.yaw_deg:+.1f} by {out.rule!r}'
    assert 'mirror pair unresolved' in out.reason
    got = sorted(out.candidates)
    assert got[0] == pytest.approx(-THETA - 9.0, abs=1.0)
    assert got[1] == pytest.approx(THETA - 9.0, abs=1.0)


def test_the_unresolved_pair_cannot_anchor_a_heading():
    out = _fuse(_frames(yaw_sweep=9.0, swap=(0, 1, 3, 4, 6, 7, 9)))
    a = anchor_from(out, 9.0, 205.0)
    assert not a.ok and 'mirror pair unresolved' in a.reason


# ── a change of viewpoint resolves it, whichever branch the solver picked ───
@pytest.mark.parametrize('lateral', [0.3, -0.3, 0.6])
def test_a_sideways_translation_resolves_to_the_TRUE_branch(lateral):
    """Falsified if it picks the mirror, or refuses with 5.7+ deg of bearing
    change and no noise."""
    s = _frames(lateral=lateral)
    out = _fuse(s)
    assert out.decided and out.rule == RULE_VIEWPOINT, out.reason
    assert out.yaw_deg == pytest.approx(_true_now(s), abs=0.5)
    assert out.slope == pytest.approx(0.0, abs=0.05)


def test_a_WRONG_majority_while_translating_still_resolves_true():
    """#55's injection, now on a moving camera. Falsified if the 7 wrong
    frames carry the answer -- the producer's preference must cast no vote."""
    s = _frames(lateral=0.45, yaw_sweep=6.0, swap=(0, 1, 3, 4, 6, 7, 9))
    out = _fuse(s)
    assert out.decided and out.rule == RULE_VIEWPOINT, out.reason
    assert out.yaw_deg == pytest.approx(_true_now(s), abs=0.5)
    a = anchor_from(out, 6.0, 205.0)
    assert a.ok


def test_it_survives_ten_degrees_of_pitch():
    """`2*beta - theta` is a yaw-only approximation; falsified if pitch breaks it."""
    s = _frames(lateral=0.6, pitch=10.0)
    out = _fuse(s)
    assert out.decided and out.yaw_deg == pytest.approx(_true_now(s), abs=1.0)


@pytest.mark.parametrize('seed', range(5))
def test_with_half_a_pixel_of_noise_it_decides_right_or_not_at_all(seed):
    """Falsified by a WRONG decision. Refusing is allowed; picking the mirror
    is not (measured: 0 wrong in 16 000 trials at the shipped SE gate)."""
    s = _frames(lateral=0.45, noise=0.5, seed=seed)
    out = _fuse(s)
    if out.decided:
        assert out.yaw_deg == pytest.approx(_true_now(s), abs=2.0)


def test_the_mirror_really_swings_at_slope_two():
    """The constant the rule is built on, measured through the producer."""
    from mongla_localization.pose_cluster import resolve_mirror
    r = resolve_mirror(_frames(lateral=0.6))
    slopes = sorted([r['slope_a'], r['slope_b']])
    assert slopes[0] == pytest.approx(0.0, abs=0.05)
    assert slopes[1] == pytest.approx(MIRROR_SLOPE, abs=0.05)


# ── the legacy path, and #55's own reproduction ──────────────────────────────
def test_issue_55_reproduction_is_now_refused():
    """The exact case filed: one branch per frame (no alternate sent), a 7/3
    wrong majority, a turn in place. It used to anchor 54 deg against 9."""
    s = [PoseSample(t=x.t, yaw_deg=x.yaw_deg, range_m=x.range_m,
                    vehicle_yaw_deg=x.vehicle_yaw_deg)
         for x in _frames(yaw_sweep=9.0, swap=(0, 1, 3, 4, 6, 7, 9))]
    out = _fuse(s)
    assert not out.decided, f'decided {out.yaw_deg:+.1f} by {out.rule!r}'
    assert not anchor_from(out, 9.0, 205.0).ok


def test_a_counted_decision_never_anchors_a_heading():
    """A single-cluster legacy stream may still be decided by 'support', but
    `anchor_from` refuses anything not resolved by viewpoint."""
    s = [PoseSample(t=x.t, yaw_deg=x.yaw_deg, range_m=x.range_m,
                    vehicle_yaw_deg=x.vehicle_yaw_deg)
         for x in _frames(lateral=0.3)]
    out = _fuse(s)
    assert out.decided and out.rule == 'support'
    a = anchor_from(out, 0.0, 205.0)
    assert not a.ok and 'viewpoint' in a.reason


def test_a_frame_where_IPPE_returns_NaN_is_solved_by_SQPnP_not_refused():
    """Measured: frame 9 of a 0.45 m / 6 deg sweep, noiseless, made IPPE
    return NaN for both branches and the producer refused a solvable frame."""
    Rt, _ = cv2.Rodrigues(np.array([0., math.radians(THETA), 0.]))
    Rc, _ = cv2.Rodrigues(np.array([0., math.radians(-6.0), 0.]))
    rv, _ = cv2.Rodrigues(Rc @ Rt)
    img, _ = cv2.projectPoints(OBJ, rv, Rc @ (np.array([0., 0., 3.]) - [0.45, 0, 0]),
                               K, np.zeros(5))
    n, rs, ts, e = cv2.solvePnPGeneric(OBJ, img.reshape(-1, 2), K, np.zeros(5),
                                       flags=cv2.SOLVEPNP_IPPE)
    tp = solve_pnp(OBJ, img.reshape(-1, 2), K)
    assert tp.ok, tp.reason
    assert tp.yaw_deg == pytest.approx(THETA - 6.0, abs=0.5)
    if not np.all(np.isfinite(np.ravel(e))):          # the case this guards
        assert math.isnan(tp.alt_yaw_deg), 'no alternate can come from a NaN solve'


# ── the wire, end to end ────────────────────────────────────────────────────
def test_the_wire_carries_both_branches_only_when_they_exist():
    """`to_msg` scrubs NaN to 0.0, so the flag is the only truth about the
    alternate. Falsified if a one-branch pose arrives with branches_valid."""
    from mongla_localization.pnp_node import to_msg
    from mongla_vision.anchor.pose import TargetPose as TP
    two = to_msg(TP(ok=True, yaw_deg=25.0, alt_yaw_deg=-25.0, bearing_deg=3.0,
                    range_m=3.0))
    assert two.branches_valid and two.alt_yaw_deg == pytest.approx(-25.0)
    one = to_msg(TP(ok=True, yaw_deg=25.0, range_m=3.0))
    assert not one.branches_valid


def _node_with(pub_sink):
    from mongla_localization.pose_fuse_node import PoseFuseNode
    n = PoseFuseNode.__new__(PoseFuseNode)
    n._cluster = PoseCluster()
    n._yaw_hist = []
    n._pub = type('P', (), {'publish': lambda _s, m: pub_sink.append(m)})()
    return n


def _feed_node(n, samples):
    from mongla_localization.pnp_node import to_msg
    from mongla_vision.anchor.pose import TargetPose as TP
    from std_msgs.msg import Header
    for s in samples:
        n._yaw_hist.append((s.t, s.vehicle_yaw_deg))
        h = Header()
        h.stamp.sec = int(s.t)
        h.stamp.nanosec = int((s.t - int(s.t)) * 1e9)
        n._on_pose(to_msg(TP(ok=True, yaw_deg=s.yaw_deg, alt_yaw_deg=s.alt_yaw_deg,
                             bearing_deg=s.bearing_deg, range_m=s.range_m,
                             reproj_px=s.reproj_px, ambiguity=s.ambiguity,
                             n_points=s.n_points), header=h))


def test_the_fuse_node_publishes_the_DISAGREEMENT_not_a_vote():
    """A turn in place through the real node: ok=False, both candidates in the
    reason, their separation in yaw_spread_deg. Falsified by ok=True."""
    out = []
    n = _node_with(out)
    _feed_node(n, _frames(yaw_sweep=9.0, swap=(0, 1, 3, 4, 6, 7, 9)))
    last = out[-1]
    assert not last.ok
    assert 'mirror pair unresolved' in last.reason
    assert last.yaw_spread_deg == pytest.approx(2 * THETA, abs=1.0)


def test_the_fuse_node_resolves_a_translating_approach():
    out = []
    n = _node_with(out)
    s = _frames(lateral=0.45, swap=(0, 1, 3, 4, 6, 7, 9))
    _feed_node(n, s)
    last = out[-1]
    assert last.ok and last.reason == RULE_VIEWPOINT
    assert last.yaw_deg == pytest.approx(_true_now(s), abs=0.5)


def test_the_DSL_hands_back_the_refusal_with_both_candidates():
    """`anchor_heading` used to wait for ok=True and time out as 'no fused
    pose', throwing the reason away. Falsified if the mission log cannot see
    which two headings were in dispute."""
    from mongla_interfaces.msg import TargetPose as Msg
    from mongla_planner.mongla_dsl import MonglaMission
    out = []
    n = _node_with(out)
    _feed_node(n, _frames(yaw_sweep=9.0))
    m = MonglaMission.__new__(MonglaMission)
    m._fused_subs = {'forward': object()}
    m._fused_pose = {'forward': out[-1]}
    m.client = type('C', (), {'node': None})()
    import mongla_planner.mongla_dsl as D
    real = D.rclpy.spin_once
    D.rclpy.spin_once = lambda *a, **k: None
    try:
        fused = MonglaMission._wait_fused_pose(m, 'forward', timeout=0.05)
    finally:
        D.rclpy.spin_once = real
    assert fused is not None and not fused.decided
    assert 'mirror pair unresolved' in fused.reason
    a = anchor_from(fused, 9.0, 205.0)
    assert not a.ok and '+16' in a.reason and '-34' in a.reason
