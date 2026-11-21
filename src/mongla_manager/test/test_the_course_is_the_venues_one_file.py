"""The course is the venue's one file: pool, floor, tiles, lane lines, the
depth floor and every prop's measured width, read by every package.

A venue number in the course AND in the deck file is refused at launch: two
copies of one fact, and nobody can tell which one the run used.
"""
import pytest

COURSE = """course: venue
frame: pool
pool:
  depth_m: 1.6
  depth_min_m: 1.28
  depth_max_m: 2.0
  floor: [[0, 0, 1.28], [17.5, 0, 2.0]]
  tile_m: 0.25
  lane_lines: true
  shield_max_depth_m: 1.5
props:
  gate:
    detect_class: gate
    width_m: 1.5
  flare:
    detect_class: orange_flare
    width_m: 0.15
"""


@pytest.fixture
def venue(tmp_path, monkeypatch):
    monkeypatch.setenv('MONGLA_COURSE_DIR', str(tmp_path))
    (tmp_path / 'venue.yaml').write_text(COURSE)
    deck = tmp_path / 'bringup.yaml'
    deck.write_text('course: venue\n')
    monkeypatch.setenv('MONGLA_BRINGUP', str(deck))
    return tmp_path


def test_every_venue_number_is_read(venue):
    from mongla_localization.course_map import load_course
    c = load_course('venue')
    assert c.launch_values() == {
        'pool_depth_m': '1.6', 'pool_depth_min_m': '1.28', 'pool_depth_max_m': '2.0',
        'tile_m': '0.25', 'lane_lines': 'true'}
    assert c.shield_max_depth_m() == 1.5
    assert c.widths_by_class() == {'gate': 1.5, 'orange_flare': 0.15}


def test_a_quoted_bool_is_refused(venue):
    from mongla_localization.course_map import load_course
    (venue / 'venue.yaml').write_text(COURSE.replace('lane_lines: true', "lane_lines: 'no'"))
    with pytest.raises(ValueError, match='lane_lines'):
        load_course('venue')


def test_the_course_widths_reach_the_pose_path(venue):
    from mongla_vision import target_geometry as tg
    tg._CACHE.clear()
    tg._STAMPS.clear()
    assert tg.width_for('orange_flare') == 0.15
    assert 'course venue' in str(tg.describe('orange_flare'))


def test_bringup_takes_the_venue_from_the_course(venue):
    import importlib.util
    import os
    path = os.path.join(os.path.dirname(__file__), '..', 'launch', 'bringup.launch.py')
    spec = importlib.util.spec_from_file_location('bringup_launch', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    from launch.actions import DeclareLaunchArgument
    args = [DeclareLaunchArgument('pool_depth_m', default_value='nan'),
            DeclareLaunchArgument('tile_m', default_value='0.0')]
    out, note, manager = mod._apply_course(args, {'course': 'venue'}, venue / 'bringup.yaml')
    got = {a.name: a.default_value[0].text for a in out}
    assert got == {'pool_depth_m': '1.6', 'tile_m': '0.25'}
    assert manager == {'shield_max_depth_m': 1.5}
    with pytest.raises(RuntimeError, match='pool_depth_m'):
        mod._apply_course(args, {'course': 'venue', 'pool_depth_m': '2.0'},
                          venue / 'bringup.yaml')


def test_a_typed_course_brings_its_venue_too(venue, monkeypatch):
    """`course:=venue` typed with no deck `course:` sets the venue numbers and
    the depth floor, as the deck's would (B-146): it used to set none."""
    import importlib.util
    import os
    import sys
    path = os.path.join(os.path.dirname(__file__), '..', 'launch', 'bringup.launch.py')
    spec = importlib.util.spec_from_file_location('bringup_launch', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    from launch.actions import DeclareLaunchArgument
    args = [DeclareLaunchArgument('pool_depth_m', default_value='nan')]
    monkeypatch.setattr(sys, 'argv', ['ros2', 'launch', 'mongla_manager', 'bringup.launch.py',
                                      'course:=venue'])
    out, _, manager = mod._apply_course(args, {}, None)
    assert {a.name: a.default_value[0].text for a in out} == {'pool_depth_m': '1.6'}
    assert manager == {'shield_max_depth_m': 1.5}
