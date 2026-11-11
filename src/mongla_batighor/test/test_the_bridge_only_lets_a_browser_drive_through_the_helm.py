"""A browser drives through the helm, or not at all.

foxglove_bridge cannot tell one browser from another, and by default it lets
each one call any service, set any parameter and publish on any topic. With
hidden topics on, that includes the action's own send_goal service, so a
watcher could have moved the vehicle past the helm. BRIDGE_LIMITS in
batighor.launch.py closes that; this reads them off the built launch action,
and checks that bringup starts the bridge only through that file.

The bridge matches a parameter as `<node>.<name>` with the node's leading
slash, and a service by its full name (foxglove-sdk
ros/src/foxglove_bridge/src/parameter_interface.cpp).
"""
import importlib.util
import re
from pathlib import Path

LAUNCH = Path(__file__).resolve().parents[1] / 'launch' / 'batighor.launch.py'
BRINGUP = Path(__file__).resolve().parents[2] / 'mongla_manager' / 'launch' / 'bringup.launch.py'


def _launch():
    spec = importlib.util.spec_from_file_location('_batighor_launch', LAUNCH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _allowed(patterns, name):
    return any(re.search(p, name) for p in patterns)


def test_the_launched_bridge_carries_the_limits():
    from launch_ros.actions import Node
    mod = _launch()
    bridge = [e for e in mod.generate_launch_description().entities
              if isinstance(e, Node) and e.node_executable == 'foxglove_bridge']
    assert len(bridge) == 1
    # launch_ros keeps each name as substitutions, and each list item as
    # substitutions of YAML text it parses at run time; read the same way.
    import yaml

    def text(x):
        return ''.join(s.text for s in x) if isinstance(x, (tuple, list)) else x
    given = {text(k): [yaml.safe_load(text(i)) for i in v]
             for d in bridge[0]._Node__parameters for k, v in d.items()
             if text(k) in mod.BRIDGE_LIMITS}
    for key, want in mod.BRIDGE_LIMITS.items():
        assert given.get(key) == want, (key, given.get(key))


def test_bringup_starts_the_bridge_only_through_batighor_launch():
    """A second bridge declared in bringup would carry its own limits, or none."""
    text = BRINGUP.read_text()
    assert "'foxglove_bridge'" not in text and 'BRIDGE_LIMITS' not in text
    assert "'batighor.launch.py'" in text


def test_only_the_helm_is_a_service_a_browser_may_call():
    lim = _launch().BRIDGE_LIMITS
    assert _allowed(lim['service_whitelist'], '/mongla/batighor/helm')
    for name in ('/mongla/move/_action/send_goal', '/mongla/move/_action/cancel_goal',
                 '/mongla_manager/set_parameters', '/mongla/batighor/helm_x'):
        assert not _allowed(lim['service_whitelist'], name), name


def test_a_browser_sets_no_parameter_itself():
    """Tuning goes through the helm, which closes it during a mission; the
    bridge cannot know when one runs, so it allows none."""
    lim = _launch().BRIDGE_LIMITS
    assert 'parameters' not in lim['capabilities']
    for name in ('/mongla_detector_forward.conf', '/mongla_manager.shield_max_depth_m'):
        assert not _allowed(lim['param_whitelist'], name), name


def test_a_browser_publishes_nothing():
    lim = _launch().BRIDGE_LIMITS
    assert 'clientPublish' not in lim['capabilities']
    assert not _allowed(lim['client_topic_whitelist'], '/mongla/mission_alive')
