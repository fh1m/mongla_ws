"""`tools/td_measure.py` scores the camera<->gyro offset on the bench; its
scorer must find a delay of KNOWN sign before its verdict on the rig counts
(CLAUDE.md §9: the scorer before the run)."""
import importlib.util
import pathlib

_TOOL = pathlib.Path(__file__).resolve().parents[3] / 'tools' / 'td_measure.py'


def test_the_scorer_recovers_injected_offsets_and_refuses_a_steady_turn():
    spec = importlib.util.spec_from_file_location('_td_measure', _TOOL)
    tm = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(tm)
    assert tm.self_test() == 0
