"""The focal length the de-rotation uses must be the WATER one, from the one
medium parameter -- and absent, not guessed, when either input is missing."""
from types import SimpleNamespace

from duburi_manager.vision_state import VisionState


def _vs(K, n_fn):
    vs = VisionState.__new__(VisionState)
    vs.calibration = lambda: (K, None)
    vs._node = SimpleNamespace(_uplink_n=n_fn) if n_fn else SimpleNamespace()
    return vs


def test_water_focal_is_fx_times_n():
    K = [741.0, 0, 320, 0, 741.0, 240, 0, 0, 1]
    assert abs(_vs(K, lambda: 1.333).px_per_rad() - 741.0 * 1.333) < 1e-9


def test_uncalibrated_zero_k_is_none():
    assert _vs([0.0] * 9, lambda: 1.333).px_per_rad() is None
    assert _vs(None, lambda: 1.333).px_per_rad() is None


def test_no_medium_source_is_none_not_air():
    assert _vs([741.0, 0, 320, 0, 741.0, 240, 0, 0, 1], None).px_per_rad() is None
