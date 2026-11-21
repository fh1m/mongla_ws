"""The deck kutum/setup.sh writes on a fresh vehicle launches as it is.

A starter file that names an unknown key would stop the first bringup on a
test day. The blank pool course it names loads, and grades as unmeasured:
bringup_check warns and a mission refuses until the floor is surveyed.
"""
import os
import pathlib
import re

ROOT = pathlib.Path(__file__).resolve().parents[3]
BRINGUP = ROOT / 'src' / 'mongla_manager' / 'launch' / 'bringup.launch.py'


def test_the_starter_deck_launches_and_says_what_is_unmeasured(tmp_path, monkeypatch):
    from mongla_runtime import launch_defaults as ld
    from mongla_manager import bringup_grade as G, bringup_host as H
    deck = ROOT / 'kutum' / 'deck-bringup.yaml'
    declared = set(re.findall(r"DeclareLaunchArgument\(\s*'([a-z_0-9]+)'", BRINGUP.read_text()))
    vision_pi = ROOT / 'src' / 'mongla_vision' / 'launch' / 'vision_pi.launch.py'
    declared |= set(re.findall(r"DeclareLaunchArgument\(\s*'([a-z_0-9]+)'", vision_pi.read_text()))
    values = ld.read(deck, declared)
    assert values == {'course': 'pool'}
    # every commented example key is real, so uncommenting it cannot stop a launch
    for key in re.findall(r'^# ([a-z_]+): ', deck.read_text(), re.M):
        assert key in declared, key
    monkeypatch.setenv('MONGLA_COURSE_DIR', str(tmp_path))       # only the packaged pool
    verdict, msg = H.flow_height_verdict(values)
    assert verdict == G.WARN and 'refuse' in msg
    assert H.depth_floor_verdict({}, 'pool')[0] == G.WARN
    assert os.path.isfile(ROOT / 'kutum' / 'deck-README.txt')
