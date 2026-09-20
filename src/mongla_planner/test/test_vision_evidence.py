"""A vision verb's scoreboard row says WHERE and HOW it ended, and whether it fired.

The server always reports success for a vision verb, so before this a run that
never saw the gate and one that centred it wrote the same scorecard row.
"""
import math
import threading
import time
from types import SimpleNamespace
from unittest.mock import MagicMock

from mongla_control.vision_verbs import _fired_suffix
from mongla_planner.mongla_dsl import MonglaMission
from mongla_planner.vision_dsl import VisionResult, _VisionDSL, ALIGNED


def test_the_fire_suffix_reports_what_left():
    assert _fired_suffix([], {'thread': None, 'outcomes': []}) == ''
    assert _fired_suffix([1], {'thread': None, 'outcomes': []}) == ' fired=none'
    th = threading.Thread(target=lambda: None)
    th.start()
    assert _fired_suffix([1, 2], {'thread': th, 'outcomes': ['ch1:FIRED', 'ch2:DENIED']}) \
        == ' fired=ch1:FIRED,ch2:DENIED'


def test_a_fire_still_running_is_reported_pending_not_waited_on_forever(monkeypatch):
    import mongla_control.vision_verbs as vv
    monkeypatch.setattr(vv, '_FIRE_REPORT_WAIT_S', 0.05)
    th = threading.Thread(target=lambda: time.sleep(0.5), daemon=True)
    th.start()
    t0 = time.monotonic()
    assert vv._fired_suffix([1], {'thread': th, 'outcomes': []}) == ' fired=pending'
    assert time.monotonic() - t0 < 0.4


def _vision(message, x=12.0):
    dsl = MagicMock()
    dsl.__dict__['_scoreboard'] = [{'cmd': 'vision_align', 'success': True,
                                    'elapsed': 3.0, 'msg': message}]
    dsl.__dict__['_vinfo'] = {'forward': (('gate_rescue_repair',), 2)}
    dsl._record_vision = lambda *a: MonglaMission._record_vision(dsl, *a)
    v = _VisionDSL.__new__(_VisionDSL)
    v._dsl = dsl
    dsl.log = MagicMock()
    result = SimpleNamespace(final_value=float(ALIGNED), error_value=4.0, end_x_px=x,
                             end_y_px=-3.0, fill_frac=0.0, elapsed_s=3.0, message=message)
    return v, dsl, result


def test_the_result_and_the_scoreboard_row_carry_the_evidence():
    v, dsl, result = _vision('vision_align: ALIGNED fired=ch1:FIRED')
    res = v._orchestrate('align', 'gate', 'forward', 5.0, None, lambda remaining: result)
    assert res.fired == 'ch1:FIRED'
    row = dsl.__dict__['_scoreboard'][0]['vision']
    assert row == {'target': 'gate', 'camera': 'forward', 'outcome': 'ALIGNED',
                   'saw_target': True, 'x_px': 12.0, 'y_px': -3.0, 'fill': 0.0,
                   'fired': 'ch1:FIRED', 'model': ['gate_rescue_repair']}


def test_no_fire_requested_reads_none_and_never_seen_reads_null():
    v, dsl, result = _vision('vision_align: ALIGNED', x=math.nan)
    res = v._orchestrate('align', 'gate', 'forward', 5.0, None, lambda remaining: result)
    assert res.fired is None
    assert dsl.__dict__['_scoreboard'][0]['vision']['x_px'] is None


def test_the_align_verb_appends_the_fire_outcome_to_its_result():
    """Structural: the suffix must be on the RESULT message the DSL parses."""
    import ast
    import pathlib
    import mongla_control.vision_verbs as vv
    src = pathlib.Path(vv.__file__).read_text()
    body = src[src.index('    def vision_align('):src.index('    def _fire_async(')]
    assert '_fired_suffix(channels, fire_state)' in body
    assert "outcomes=fire_state['outcomes']" in body


def test_an_evidence_frame_is_written_as_a_viewable_ppm(tmp_path):
    from mongla_planner.mongla_dsl import _write_ppm
    img = SimpleNamespace(encoding='bgr8', width=2, height=1, step=6,
                          data=bytes([1, 2, 3, 4, 5, 6]))
    path = _write_ppm(img, str(tmp_path), 'align_gate_forward')
    raw = open(path, 'rb').read()
    assert raw.startswith(b'P6 2 1 255\n')
    assert raw.endswith(bytes([3, 2, 1, 6, 5, 4]))      # BGR -> RGB
    assert _write_ppm(SimpleNamespace(encoding='mono8'), str(tmp_path), 'x') is None
