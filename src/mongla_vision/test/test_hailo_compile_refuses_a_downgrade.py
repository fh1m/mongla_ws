"""The compile refuses every silent optimisation downgrade (issue #47).

The DFC drops AdaRound/fine-tuning below 1024 calibration frames OR with no
GPU, warns, and builds a HEF with degraded scores anyway. `downgrade_reasons`
is the one place that decides; these pin each way in.
"""
import importlib.util
import pathlib

_TOOL = pathlib.Path(__file__).resolve().parents[3] / 'tools' / 'bench' / 'hailo_compile.py'
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


# ── the optimiser degrading a FULL build (2026-11-06) ─────────────────────────
# gate_sharks passed every downgrade check and its fine-tune diverged: median
# INT8 shift 0.339 on the emulator against -0.010 for a level-0 build.

def test_a_diverged_full_build_is_refused():
    why = hc.int8_shift_reasons({'n': 133, 'median': 0.339, 'p90': 0.555})
    assert why and 'degraded' in why[0]


def test_the_healthy_builds_measured_so_far_pass():
    for m in (-0.010, 0.04, 0.06):
        assert hc.int8_shift_reasons({'n': 133, 'median': m, 'p90': 0.1}) == []


def test_too_few_scored_frames_is_unverified_not_passed():
    """Archive frames without a target score below 0.25 on the float path;
    a median over three of them is not a measurement."""
    why = hc.int8_shift_reasons({'n': 3, 'median': 0.0, 'p90': 0.0})
    assert why and 'UNVERIFIED' in why[0]


def test_the_bar_sits_between_the_healthy_and_the_diverged_builds():
    assert 0.06 < hc.MAX_INT8_SHIFT < 0.339


def test_a_short_clip_gives_its_keyframes_once_and_the_rest_moves_on():
    # A clip with 5 keyframes asked for a tenth of 30 must not repeat frames
    # to fill its share; the longer clips take the remainder (side note
    # 2026-11-22: repeats would still read as the full count).
    take = hc.allocate([5, 100, 100], 30)
    assert take[0] == 5 and sum(take) == 30
    assert all(t <= c for t, c in zip(take, [5, 100, 100]))


def test_an_archive_too_small_gives_fewer_not_repeated():
    assert hc.allocate([3, 3, 3], 20) == [3, 3, 3]
    assert hc.allocate([0, 50], 10) == [0, 10]


def test_the_same_footage_filed_twice_is_read_once(tmp_path):
    # The archive holds two such pairs; each copy took a share and gave the
    # same frames, so 1024 frames requested came back 975 distinct.
    a, b, c = tmp_path / 'a.mkv', tmp_path / 'b' / 'a.mkv', tmp_path / 'c.mkv'
    b.parent.mkdir()
    a.write_bytes(b'x' * 5000)
    b.write_bytes(b'x' * 5000)
    c.write_bytes(b'y' * 5000)
    assert hc.unique_clips([a, b, c]) == [a, c]
