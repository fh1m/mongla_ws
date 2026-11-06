"""Bringup settings from ~/.mongla/bringup.yaml: the file sets defaults, the
command line still wins, and a typo stops the launch instead of vanishing.

The command line cannot be made strict (`ros2 launch` ignores unknown names);
the file can, and these pin that it is.
"""
import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument

from mongla_manager import launch_defaults as LD


def _args():
    return [DeclareLaunchArgument('vision', default_value='false'),
            DeclareLaunchArgument('medium', default_value='water',
                                  choices=['water', 'air'], description='m'),
            DeclareLaunchArgument('pool_depth_m', default_value='nan')]


def _resolve(arg, given=None):
    ctx = LaunchContext()
    if given is not None:
        ctx.launch_configurations[arg.name] = given
    arg.visit(ctx)
    return ctx.launch_configurations[arg.name]


@pytest.fixture
def settings(tmp_path, monkeypatch):
    def write(text):
        p = tmp_path / 'bringup.yaml'
        p.write_text(text)
        monkeypatch.setenv(LD.ENV, str(p))
        return p
    return write


def test_no_file_changes_nothing(monkeypatch, tmp_path):
    monkeypatch.setenv('HOME', str(tmp_path))
    monkeypatch.setattr(LD, 'DEFAULT_PATH', tmp_path / 'absent.yaml')
    monkeypatch.delenv(LD.ENV, raising=False)
    args = _args()
    out, values, path = LD.apply(args)
    assert out == args and values == {} and path is None


def test_the_file_sets_the_default(settings):
    settings('vision: true\npool_depth_m: 1.2\nmedium: air\n')
    out, values, _ = LD.apply(_args())
    got = {a.name: _resolve(a) for a in out}
    assert got == {'vision': 'true', 'medium': 'air', 'pool_depth_m': '1.2'}


def test_the_command_line_beats_the_file(settings):
    settings('vision: true\n')
    out, _, _ = LD.apply(_args())
    vision = next(a for a in out if a.name == 'vision')
    assert _resolve(vision, given='false') == 'false'


def test_a_misspelt_key_stops_the_launch_and_names_the_real_one(settings):
    settings('vison: true\n')
    with pytest.raises(RuntimeError, match="'vison'.*did you mean 'vision'"):
        LD.apply(_args())


def test_a_value_outside_the_choices_stops_the_launch(settings):
    p = settings('medium: mud\n')
    with pytest.raises(RuntimeError, match=f"{p}.*medium.*'mud'"):
        LD.apply(_args())


def test_pointing_at_a_missing_file_is_an_error_not_a_no_op(monkeypatch, tmp_path):
    monkeypatch.setenv(LD.ENV, str(tmp_path / 'typo_in_the_path.yaml'))
    with pytest.raises(RuntimeError, match='no such file'):
        LD.config_path()


def test_none_switches_the_file_off(settings, monkeypatch):
    settings('vision: true\n')
    monkeypatch.setenv(LD.ENV, 'none')
    assert LD.config_path() is None


def test_every_key_in_the_shipped_example_is_a_real_bringup_argument():
    """The example is generated; if it drifts from bringup it must fail here."""
    import ast
    import pathlib
    import re
    import yaml
    pkg = pathlib.Path(__file__).resolve().parents[1]
    text = (pkg / 'config' / 'bringup.example.yaml').read_text()
    # Every setting ships commented out; uncomment them all and it must parse.
    example = yaml.safe_load('\n'.join(
        line[2:] for line in text.splitlines() if re.match(r'# [a-z_0-9]+: ', line)))
    tree = ast.parse((pkg / 'launch' / 'bringup.launch.py').read_text())
    declared = {n.args[0].value for n in ast.walk(tree)
                if isinstance(n, ast.Call)
                and getattr(n.func, 'id', '') == 'DeclareLaunchArgument'
                and isinstance(n.args[0], ast.Constant)}
    assert set(example) == declared
