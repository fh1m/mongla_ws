"""The compile refuses every silent optimisation downgrade (issue #47).

The DFC drops AdaRound/fine-tuning below 1024 calibration frames OR with no
GPU, warns, and builds a HEF with degraded scores anyway. `downgrade_reasons`
is the one place that decides; these pin each way in.
"""
import importlib.util
import pathlib

_TOOL = pathlib.Path(__file__).resolve().parents[3] / 'tools' / 'hailo_compile.py'
_spec = importlib.util.spec_from_file_location('_hailo_compile', _TOOL)
hc = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(hc)


def test_a_full_build_on_a_gpu_has_no_reason_to_refuse():
    assert hc.downgrade_reasons(frames=1024, gpu=True) == []


def test_frames_DELIVERED_count_not_frames_requested():
    """`calib_frames` accepts as few as half of what was asked."""
    assert hc.downgrade_reasons(frames=600, gpu=True)


def test_no_gpu_is_a_downgrade_and_unknown_is_left_to_the_log():
    assert hc.downgrade_reasons(frames=1024, gpu=False)
    assert hc.downgrade_reasons(frames=1024, gpu=None) == []


def test_the_dfcs_own_warning_is_caught_whatever_its_cause():
    line = ("[warning] Reducing optimization level to 0 (the accuracy won't be "
            "optimized and compression won't be used) because there's no "
            "available GPU")
    got = hc.downgrade_reasons(frames=2048, gpu=None, log_lines=['ok', line])
    assert got and 'level to 0' in got[0]
