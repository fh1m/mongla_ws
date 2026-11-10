"""The benchmark must distinguish failed calls from measured chip execution."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest

_PATH = Path(__file__).resolve().parents[3] / 'tools/studies/composed_lock_harness.py'
_spec = importlib.util.spec_from_file_location('composed_lock_harness', _PATH)
harness = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(harness)


def test_failed_calls_are_not_successful_latency_samples():
    class Backend:
        def detect(self, gray):
            raise RuntimeError('chip failed')

    original = Backend.detect
    with harness.measure_calls(Backend) as measured:
        with pytest.raises(RuntimeError, match='chip failed'):
            Backend().detect(None)
    assert measured['ms'] == []
    assert measured['errors'] == ['RuntimeError: chip failed']
    assert Backend.detect is original


def test_successful_calls_return_the_original_result_and_restore_patch():
    result = object()

    class Backend:
        def detect(self, gray):
            return result

    original = Backend.detect
    with harness.measure_calls(Backend) as measured:
        assert Backend().detect(None) is result
    assert len(measured['ms']) == 1 and measured['ms'][0] >= 0
    assert measured['errors'] == []
    assert Backend.detect is original


def test_patch_is_restored_even_if_the_run_raises():
    class Backend:
        def detect(self, gray):
            return None

    original = Backend.detect
    with pytest.raises(ValueError):
        with harness.measure_calls(Backend):
            raise ValueError('startup failed')
    assert Backend.detect is original


def test_one_success_cannot_hide_later_chip_failures():
    checks = harness.score_run('XFeatHailo', 2, [10.0], 0.5,
                                ['RuntimeError: chip failed'])
    assert not all(ok for _, ok in checks)


def test_cpu_backend_is_a_falsifier_even_with_fast_calls():
    checks = harness.score_run('XFeatONNX', 2, [10.0], 0.5, [])
    assert not all(ok for _, ok in checks)


def test_no_calls_cannot_pass():
    assert not all(ok for _, ok in harness.score_run(
        'XFeatHailo', 2, [], 0.5, []))


def test_healthy_run_passes():
    assert all(ok for _, ok in harness.score_run(
        'XFeatHailo', 2, [10.0, 12.0], 0.5, []))
