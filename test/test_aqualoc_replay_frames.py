"""The AQUALOC replay converts image motion to camera velocity correctly.

tools/studies/aqualoc_replay.py feeds real underwater data through the shipped
flow chain and RIEKF. Its one hand-derived step is the mapping from image shift
and gyro rate to camera velocity; the self-test moves a synthetic camera over a
synthetic floor with a known velocity and rotation and must recover it, and
must fail with the rotation sign flipped.
"""
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / 'tools' / 'studies'))
import aqualoc_replay  # noqa: E402


def test_the_self_test_passes():
    assert aqualoc_replay._self_test() == 0


def test_it_fails_with_the_rotation_sign_flipped(monkeypatch):
    orig = aqualoc_replay.flow_velocity

    def flipped(*a, roll_rate=0.0, pitch_rate=0.0, **k):
        return orig(*a, roll_rate=-roll_rate, pitch_rate=-pitch_rate, **k)

    monkeypatch.setattr(aqualoc_replay, 'flow_velocity', flipped)
    assert aqualoc_replay._self_test() == 1
