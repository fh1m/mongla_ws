"""A stopped calibration reads stopped, one Send is one goal, and Arm checks
the pilot at the click (audit batighor_4 F8, batighor_5 #7, #17)."""
import pathlib

PANELS = pathlib.Path(__file__).resolve().parents[1] / 'panels'
NODE = (pathlib.Path(__file__).resolve().parents[2] / 'mongla_vision' / 'mongla_vision'
        / 'calibration' / 'calibrate_node.py')


def test_the_calibrate_page_ages_its_state_and_the_node_keeps_it_fresh():
    js = (PANELS / 'calibrate' / 'extension.js').read_text()
    assert 'Date.now() - stAt > STALE_MS' in js
    assert 'now - self._last_t >= 1.0' in NODE.read_text()


def test_send_is_one_goal_per_press():
    js = (PANELS / 'control' / 'extension.js').read_text()
    assert 'if (sending) return;' in js


def test_arm_rechecks_the_pilot_at_the_click():
    js = (PANELS / 'control' / 'extension.js').read_text()
    arm = js[js.index('armGo.addEventListener'):][:400]
    assert 'helm.pilot === me()' in arm
