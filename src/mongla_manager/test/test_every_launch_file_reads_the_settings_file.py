"""Every launch file an operator types reads ~/.mongla/bringup.yaml, not only bringup.

A setting that can only be typed is a setting that gets mistyped under time
pressure, and `ros2 launch` ignores a misspelt `name:=value` without a word.
So each vision launch file reads its own section, `launch: <name>:`, with the
same strictness as bringup's top level -- and only when it is the file on the
command line, so bringup's includes are never re-defaulted behind its back.
"""
from __future__ import annotations

import pathlib
import sys

import pytest

from mongla_localization import launch_defaults as LD

ROOT = pathlib.Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'tools'))
from launch_render import render, source_tree  # noqa: E402

LAUNCH = {p.name.removesuffix('.launch.py'): p
          for p in (ROOT / 'src').glob('*/launch/*.launch.py')}


@pytest.fixture
def settings(tmp_path, monkeypatch):
    def write(text):
        p = tmp_path / 'bringup.yaml'
        p.write_text(text)
        monkeypatch.setenv(LD.ENV, str(p))
        return p
    return write


def _typed(monkeypatch, name):
    monkeypatch.setattr(sys, 'argv', ['ros2', 'launch', 'pkg', f'{name}.launch.py'])


def test_the_sections_are_exactly_the_launch_files_other_than_bringup():
    assert set(LD.SECTIONS) == set(LAUNCH) - {'bringup'}


@pytest.mark.parametrize('name', sorted(set(LAUNCH) - {'bringup'}))
def test_each_launch_file_asks_for_its_section(name):
    assert 'launch_defaults.for_launch_file(args, __file__)' in LAUNCH[name].read_text()


def test_a_typed_file_takes_its_section(settings, monkeypatch):
    """The real path: video.launch.py refuses with no footage, and starts
    when the footage is in its section."""
    settings('launch:\n  video:\n    fwd_video: /footage/gate.mp4\n')
    _typed(monkeypatch, 'video')
    with source_tree():
        assert render({}, LAUNCH['video'])


def test_an_included_file_ignores_its_section(settings, monkeypatch):
    settings('launch:\n  video:\n    fwd_video: /footage/gate.mp4\n')
    _typed(monkeypatch, 'bringup')          # video.launch.py is not the one typed
    with source_tree(), pytest.raises(RuntimeError, match='fwd_video:='):
        render({}, LAUNCH['video'])


def test_the_command_line_still_wins(settings, monkeypatch):
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument
    settings('launch:\n  vision_dual:\n    fwd_model: from_file\n')
    _typed(monkeypatch, 'vision_dual')
    args, _ = LD.for_launch_file([DeclareLaunchArgument('fwd_model', default_value='x')],
                                 'vision_dual.launch.py')
    ctx = LaunchContext()
    ctx.launch_configurations['fwd_model'] = 'typed'
    args[0].visit(ctx)
    assert ctx.launch_configurations['fwd_model'] == 'typed'


def test_a_misspelt_section_stops_every_launch(settings, monkeypatch):
    settings('launch:\n  vison_dual:\n    fwd_model: m\n')
    _typed(monkeypatch, 'vision')
    with pytest.raises(RuntimeError, match="'vison_dual'.*did you mean 'vision_dual'"):
        LD.for_launch_file([], 'vision.launch.py')
    with pytest.raises(RuntimeError, match="'vison_dual'"):
        LD.apply([])                         # bringup refuses it too


def test_a_misspelt_key_in_a_section_names_the_real_one(settings, monkeypatch):
    from launch.actions import DeclareLaunchArgument
    settings('launch:\n  vision_dual:\n    fwd_modle: m\n')
    _typed(monkeypatch, 'vision_dual')
    with pytest.raises(RuntimeError, match="vision_dual.launch.py argument.*did you mean 'fwd_model'"):
        LD.for_launch_file([DeclareLaunchArgument('fwd_model', default_value='x')],
                           'vision_dual.launch.py')


def test_bringup_reads_the_top_level_and_not_the_sections(settings):
    from launch.actions import DeclareLaunchArgument
    settings('vision: true\nlaunch:\n  vision_pi:\n    fwd_model: m\n')
    _, values, _ = LD.apply([DeclareLaunchArgument('vision', default_value='false')])
    assert values == {'vision': 'true'}


def test_the_template_section_for_each_file_is_accepted_by_that_file(tmp_path, monkeypatch):
    """Uncommenting any line of a section in the example must not stop the
    launch: the template and the files it describes cannot drift apart."""
    import yaml
    from launch_render import declared
    text = (ROOT / 'src/mongla_manager/config/bringup.example.yaml').read_text()
    i, j = text.index('# launch:'), text.index('# ---- node parameters')
    block = '\n'.join(ln[2:] for ln in text[i:j].splitlines() if ln.startswith('# '))
    sections = yaml.safe_load(block)['launch']
    assert set(sections) == set(LD.SECTIONS)
    p = tmp_path / 'bringup.yaml'
    p.write_text(yaml.safe_dump({'launch': sections}))
    with source_tree():
        for name, values in sections.items():
            got = LD.read(p, set(declared(LAUNCH[name])), name)
            assert set(got) == set(values), name
