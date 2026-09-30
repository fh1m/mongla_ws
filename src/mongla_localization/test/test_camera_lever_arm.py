"""A camera that is not at the IMU sees the IMU's velocity PLUS omega x r.

Issue #27. The downward camera is the vehicle's only velocity sensor, and it
does not sit at the board's IMU. Rigid-body kinematics, exactly:

    v_cam = v_imu + omega x r            r = IMU -> camera, body FRD

The filter's state is the IMU's velocity (it predicts on the board's IMU), and
`update_body_velocity_xy` used to fuse the camera's velocity as if it were the
IMU's. So every turn wrote `omega x r` of velocity that does not exist into
the estimate: at 0.5 rad/s of yaw with the lens 0.25 m ahead of the IMU, that
is 0.125 m/s sideways, fused at the flow node's millimetre-per-second sigma.

⛔ THIS IS NOT THE DEROTATION GAIN. measured-bars §12 fits the IMAGE flow a
pitch or roll produces (`f * omega * dt`) and has its own open 12 % excess. The
lever arm is the camera's physical TRANSLATION during a rotation, which flow
measures correctly and which the filter must attribute to rotation rather than
to the hull. A yaw about a vertical axis through the IMU produces no
derotation term at all and a large lever-arm term, which is why the truth case
below is a yaw.

⛔ THE SIGN IS PINNED TWICE. `flow_node.py` records a pivot lever arm that
"came out WRONG" once already. So the case is run with r and with -r: the
right sign cancels the error, the wrong sign DOUBLES it, and a reader can see
which is which without trusting the algebra.

Premise, verified on the bench board on 2026-09-30: yaw right (clockwise seen
from above) is +z in FRD -- a ~90 deg clockwise turn moved heading +87.5 deg,
and `SCALED_IMU2.zgyro` agreed in sign with d(heading)/dt in 33 of 35 moving
samples.
"""
import math

import numpy as np
import pytest

from mongla_localization.inekf import RIEKF

G = 9.80665
ACCEL_LEVEL = np.array([0.0, 0.0, -G])   # a level, unaccelerated hull (FRD)
DT = 0.02
YAW_RATE = 0.5                           # rad/s -- about a `yaw_right 90`
R_CAM = np.array([0.25, 0.0, 0.10])      # lens 25 cm ahead, 10 cm below the IMU


def _spin(lever_arm, r_true=R_CAM, seconds=5.0):
    """A hull turning in place about its IMU. Truth: v_imu = 0 throughout.

    The camera, 0.25 m forward, is carried around a circle and reports the
    translation that produces -- which is real, and is exactly `omega x r`.
    """
    f = RIEKF()
    w = np.array([0.0, 0.0, YAW_RATE])
    v_cam = np.cross(w, r_true)             # what the flow node sees, body FRD
    for i in range(int(seconds / DT)):
        f.predict(w, ACCEL_LEVEL, DT)
        if i % 5 == 0:                      # 10 Hz flow
            f.update_depth(0.0, sigma=0.05)
            f.update_body_velocity_xy(v_cam[0], v_cam[1], 1e-6, 1e-6,
                                      lever_arm=lever_arm)
    return f


def _speed(f):
    return float(np.linalg.norm(f.X.v[:2]))


def test_the_truth_case_is_not_trivial():
    """The camera really does move: 0.125 m/s sideways, to the RIGHT (+y).

    A lens ahead of the IMU, turning right, swings right. If this is zero the
    test below cannot fail and proves nothing."""
    v_cam = np.cross([0.0, 0.0, YAW_RATE], R_CAM)
    assert v_cam[0] == pytest.approx(0.0, abs=1e-12)
    assert v_cam[1] == pytest.approx(+0.125, abs=1e-12)


def test_without_the_lever_arm_a_turn_in_place_reads_as_sideways_motion():
    """The defect, as it stood: the camera's velocity fused as the IMU's."""
    f = _spin(lever_arm=None)
    assert _speed(f) > 0.10, (
        f'{_speed(f):.4f} m/s: an uncorrected turn in place no longer reads as '
        f'motion -- then this test is not demonstrating #27 any more')


def test_with_the_lever_arm_a_turn_in_place_is_still():
    f = _spin(lever_arm=R_CAM)
    assert _speed(f) < 0.01, f'{_speed(f):.4f} m/s on a hull turning in place'


def test_the_WRONG_sign_doubles_the_error_rather_than_hiding_it():
    """The sign check `flow_node.py` says was once got wrong."""
    wrong = _speed(_spin(lever_arm=-R_CAM))
    none = _speed(_spin(lever_arm=None))
    assert wrong == pytest.approx(2.0 * none, rel=0.10), (
        f'-r gave {wrong:.4f} m/s against {none:.4f} uncorrected; the wrong '
        f'sign must be exactly twice as bad, or the model is not omega x r')


def test_the_lever_arm_does_nothing_when_nothing_turns():
    """Straight-line flight: omega = 0, so omega x r = 0 and r must not matter."""
    out = []
    for arm in (None, R_CAM):
        f = RIEKF()
        f.X.v = np.array([0.3, 0.0, 0.0])
        for i in range(int(5.0 / DT)):
            f.predict(np.zeros(3), ACCEL_LEVEL, DT)
            if i % 5 == 0:
                f.update_depth(0.0, sigma=0.05)
                f.update_body_velocity_xy(0.3, 0.0, 1e-4, 1e-4, lever_arm=arm)
        out.append(f.X.v.copy())
    assert np.allclose(out[0], out[1], atol=1e-9)


def test_the_rate_used_is_the_bias_corrected_one():
    """omega x r must use gyro MINUS bias, the same omega `predict` integrates.

    A 0.02 rad/s gyro bias on a still hull is 5 mm/s of phantom lever-arm
    velocity at r = 0.25 m if the raw gyro is used."""
    f = RIEKF()
    f.X.bg = np.array([0.0, 0.0, 0.02])
    f.predict(np.array([0.0, 0.0, 0.02]), ACCEL_LEVEL, DT)   # truth: not turning
    assert np.allclose(f.last_rate(), np.zeros(3), atol=1e-12)


def test_a_LATE_flow_sample_is_corrected_with_the_rate_AT_ITS_STAMP():
    """⛔ Flow reaches the filter through the replay path, often late. The
    rate used for `omega x r` must be the rate when the image pair was taken,
    not the rate when the message arrived.

    Truth: the hull turns in place at 0.5 rad/s for 2 s, then holds still. A
    flow sample taken DURING the turn (camera really moving at 0.125 m/s)
    arrives 0.5 s AFTER the turn stopped. Replayed at its own stamp, it must
    be corrected with 0.5 rad/s and read as a still hull. Corrected with the
    rate at arrival -- zero -- it would read as 0.125 m/s of translation."""
    from mongla_localization.retro import Retrodictor

    f = RIEKF()
    rd = Retrodictor(f, horizon_s=2.0)
    w_turn = np.array([0.0, 0.0, YAW_RATE])
    v_cam_during_turn = np.cross(w_turn, R_CAM)
    t = 0.0
    for i in range(int(2.0 / DT)):              # turning
        t += DT
        rd.run(t, lambda g, w=w_turn: g.predict(w, ACCEL_LEVEL, DT))
    t_sample = t - 0.1                          # taken mid-turn
    for i in range(int(0.5 / DT)):              # stopped
        t += DT
        rd.run(t, lambda g: g.predict(np.zeros(3), ACCEL_LEVEL, DT))
    assert np.allclose(f.last_rate(), 0.0), 'the live rate is zero on arrival'
    assert rd.run(t_sample, lambda g: g.update_body_velocity_xy(
        v_cam_during_turn[0], v_cam_during_turn[1], 1e-6, 1e-6,
        lever_arm=R_CAM)), 'the late sample was not within the replay horizon'
    assert _speed(f) < 0.01, (
        f'{_speed(f):.4f} m/s: the late sample was corrected with the rate at '
        f'ARRIVAL (zero), not the rate at its stamp')


def test_the_node_passes_its_configured_lever_arm_to_the_filter():
    """No capability ships unreachable: the node, not just the filter, must
    use the arm. A turn in place fed through `_on_flow` with an arm set reads
    as still; with it unset, as the old sideways motion."""
    import sys
    sys.path.insert(0, str(__import__('pathlib').Path(__file__).parent))
    import test_localization_node as T
    from geometry_msgs.msg import TwistWithCovarianceStamped

    def run(arm):
        n = T._node()
        n._flow_lever_arm = arm
        w = np.array([0.0, 0.0, YAW_RATE])
        v_cam = np.cross(w, R_CAM)
        for i in range(250):
            n._filter.predict(w, ACCEL_LEVEL, DT)
            if i % 5 == 0:
                n._filter.update_depth(0.0, sigma=0.05)
                m = TwistWithCovarianceStamped()
                m.twist.twist.linear.x = float(v_cam[0])
                m.twist.twist.linear.y = float(v_cam[1])
                m.twist.covariance[0] = m.twist.covariance[7] = 1e-6
                n._on_flow(m)
        return _speed(n._filter)

    assert run(None) > 0.10
    assert run(tuple(R_CAM)) < 0.01


def test_the_shipped_frames_file_loads_and_says_what_is_missing():
    """The file is valid, and until it is taped every offset reads unmeasured.

    When someone fills one in, this test is where they learn it was read."""
    from mongla_localization import frames
    o = frames.load()
    for name in o.unmeasured():
        assert getattr(o, name) is None


@pytest.mark.parametrize('bad', ['[1, 2]', '[250, 0, 100]', '"0.25"'])
def test_a_malformed_offset_is_refused_not_guessed(tmp_path, bad):
    """Millimetres typed as metres is the likeliest tape error; refuse it."""
    from mongla_localization import frames
    p = tmp_path / 'frames.yaml'
    p.write_text(f'imu_to:\n  downward_cam: {bad}\n')
    with pytest.raises(ValueError):
        frames.load(p)
