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
    declared = {a.name for a in _launch_ast().declarations(pkg / 'launch' / 'bringup.launch.py')}
    assert set(example) == declared


def _launch_ast():
    import importlib.util
    import pathlib
    import sys
    if 'launch_ast' not in sys.modules:
        path = pathlib.Path(__file__).resolve().parents[3] / 'tools' / 'launch_ast.py'
        spec = importlib.util.spec_from_file_location('launch_ast', path)
        sys.modules['launch_ast'] = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(sys.modules['launch_ast'])
    return sys.modules['launch_ast']


def test_bringup_passes_through_exactly_what_vision_pi_declares_and_it_does_not():
    """Every vehicle-vision argument is reachable from bringup, once.

    Missing one hides a setting from bringup.yaml (it is refused as unknown)
    and from the generated example; listing one bringup also declares would
    shadow bringup's own meaning of the name (`vision`, `lock`).
    """
    import pathlib
    la = _launch_ast()
    pkg = pathlib.Path(__file__).resolve().parents[1]
    bringup = pkg / 'launch' / 'bringup.launch.py'
    own = {a.name for a in la._own(bringup)}
    theirs = [a.name for a in la._own(la.find_launch('vision_pi.launch.py'))]
    assert la.passthrough(bringup)['vision_pi.launch.py'] == tuple(
        n for n in theirs if n not in own)


# --------------------------------------------------------------------------- #
#  `nodes:` -- node parameters in the same file                               #
# --------------------------------------------------------------------------- #
def test_nodes_is_not_mistaken_for_a_misspelt_argument(settings):
    settings('vision: true\nnodes:\n  mongla_manager:\n    vision.kp_yaw: 80.0\n')
    out, values, _ = LD.apply(_args())
    assert values == {'vision': 'true'}


def test_nodes_section_is_read_flat_or_nested(settings):
    p = settings('nodes:\n  mongla_manager:\n    vision:\n      kp_yaw: 80.0\n'
                 '  /mongla_localization:\n    zupt_window_s: 0.5\n')
    nodes = LD.read_nodes(p)
    assert nodes == {'mongla_manager': {'vision': {'kp_yaw': 80.0}},
                     'mongla_localization': {'zupt_window_s': 0.5}}


@pytest.mark.parametrize('body, match', [
    ("nodes:\n  '/**':\n    use_sim_time: true\n", 'wildcard'),
    ('nodes:\n  mongla_manager:\n    ros__parameters:\n      a: 1\n', 'ros__parameters'),
    ('nodes: [mongla_manager]\n', 'must map node names'),
])
def test_a_nodes_shape_ros_would_misread_stops_the_launch(settings, body, match):
    p = settings(body)
    with pytest.raises(RuntimeError, match=match):
        LD.read_nodes(p)


def test_node_actions_hand_the_section_to_ros_in_its_own_form(settings, monkeypatch):
    import yaml
    from launch_ros.actions import SetParametersFromFile
    from mongla_localization.params_file import APPLIED_ENV
    settings('nodes:\n  mongla_manager:\n    vision.kp_yaw: 80.0\n')
    acts = LD.node_actions()
    assert any(isinstance(a, SetParametersFromFile) for a in acts)
    env = next(a for a in acts if getattr(a, 'name', None) is not None
               and 'SetEnvironmentVariable' in type(a).__name__)
    ros_file = ''.join(s.text for s in env.value)
    assert ''.join(s.text for s in env.name) == APPLIED_ENV
    assert yaml.safe_load(open(ros_file)) == {
        'mongla_manager': {'ros__parameters': {'vision.kp_yaw': 80.0}}}


def test_no_nodes_section_adds_no_actions(settings):
    settings('vision: true\n')
    assert LD.node_actions() == []


# --------------------------------------------------------------------------- #
#  The node half: a real rclpy node, the file as ROS hands it over             #
# --------------------------------------------------------------------------- #
def _node_with(tmp_path, monkeypatch, section, *, inline=None):
    import rclpy
    import yaml
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from mongla_localization.params_file import APPLIED_ENV
    f = tmp_path / 'nodes.yaml'
    f.write_text(yaml.safe_dump({'probe': {'ros__parameters': section}}))
    monkeypatch.setenv(APPLIED_ENV, str(f))
    ctx = rclpy.Context()
    rclpy.init(context=ctx, args=['--ros-args', '--params-file', str(f)])
    node = Node('probe', context=ctx,
                parameter_overrides=[Parameter(k, value=v) for k, v in (inline or {}).items()])
    node.declare_parameter('kp_yaw', 60.0)
    node.declare_parameter('mixer_aware', True)
    return node, ctx


def _close(node, ctx):
    import rclpy
    node.destroy_node()
    rclpy.shutdown(context=ctx)


def test_a_real_node_accepts_a_declared_parameter(tmp_path, monkeypatch):
    from mongla_localization.params_file import refuse_unknown
    node, ctx = _node_with(tmp_path, monkeypatch, {'kp_yaw': 80.0})
    try:
        refuse_unknown(node)
        assert node.get_parameter('kp_yaw').value == 80.0
    finally:
        _close(node, ctx)


def test_a_real_node_refuses_a_misspelt_parameter(tmp_path, monkeypatch):
    from mongla_localization.params_file import refuse_unknown
    node, ctx = _node_with(tmp_path, monkeypatch, {'kp_yaww': 80.0})
    try:
        with pytest.raises(RuntimeError, match="'kp_yaww'.*did you mean 'kp_yaw'"):
            refuse_unknown(node)
    finally:
        _close(node, ctx)


def test_a_real_node_refuses_a_value_the_launch_overrode(tmp_path, monkeypatch):
    """Launch passes its own values after the file. The file's value loses
    without a word, so the node says so and names the fix."""
    from mongla_localization.params_file import refuse_unknown
    node, ctx = _node_with(tmp_path, monkeypatch, {'mixer_aware': False},
                           inline={'mixer_aware': True})
    try:
        with pytest.raises(RuntimeError, match='launch argument sets this'):
            refuse_unknown(node)
    finally:
        _close(node, ctx)


def test_without_bringup_nothing_is_checked(monkeypatch):
    from mongla_localization.params_file import APPLIED_ENV, refuse_unknown
    monkeypatch.delenv(APPLIED_ENV, raising=False)
    refuse_unknown(object())         # never touches the node
