"""The bench's stand-in filter refuses to run beside the real one.

Started beside a running filter, both published `/mongla/localization/state`,
and Batighor drew the vehicle anchored at the origin and 13 m away by turns,
five seconds apart (seen in the desktop app, 2026-11-21). The emulator now
looks for another publisher before it publishes, and stops if one appears.
"""
import os
import re
import subprocess
import sys
import time
from pathlib import Path

import pytest

TOOL = Path(__file__).resolve().parents[1] / 'tools' / 'bench' / 'emulate_localization.py'


def test_the_emulator_refuses_while_a_filter_publishes(monkeypatch):
    rclpy = pytest.importorskip('rclpy')
    msgs = pytest.importorskip('mongla_interfaces.msg')
    # One machine, its own domain: on a host with several interfaces the
    # default multicast discovery did not find a second process at all.
    env = dict(os.environ, ROS_DOMAIN_ID=str(150 + os.getpid() % 80), ROS_LOCALHOST_ONLY='1')
    monkeypatch.setenv('ROS_DOMAIN_ID', env['ROS_DOMAIN_ID'])
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    rclpy.init()
    try:
        filt = rclpy.create_node('localization_filter')
        pub = filt.create_publisher(msgs.LocalizationState, '/mongla/localization/state', 10)
        proc = subprocess.Popen([sys.executable, str(TOOL)], env=env, text=True,
                                stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        end = time.monotonic() + 30
        while proc.poll() is None and time.monotonic() < end:
            pub.publish(msgs.LocalizationState())
            rclpy.spin_once(filt, timeout_sec=0.1)
        if proc.poll() is None:
            proc.kill()
        out, err = proc.communicate(timeout=10)
        assert proc.returncode == 1, (out, err)
        # Named once discovery has the name; on CI the endpoint came first
        # and the graph reported a placeholder, which is said in words.
        assert re.search(r'(localization_filter|a node not yet named) on '
                         r'/mongla/localization/state', err), err
        assert '_NODE_NAME_UNKNOWN_' not in err
        assert 'localization:=false' in err
        filt.destroy_node()
    finally:
        rclpy.shutdown()


def test_the_emulator_says_everything_the_filter_says():
    """A stand-in that leaves out a topic the filter publishes leaves the
    panels reading it blank on the bench: Navigation said `aided by --`
    beside a map saying `aided (emulated)` (desktop app, 2026-11-22). The
    filter's own list is `SAYS` in its construction test, read here so there
    is one list. `/tf_static` is left out: the emulator draws no fixed frames,
    the bringup's frame publisher does."""
    import ast
    real = Path(__file__).resolve().parents[1] / 'src' / 'mongla_localization' / 'test' / \
        'test_the_real_node_constructs.py'
    tree = ast.parse(real.read_text())
    says = next(ast.literal_eval(n.value) for n in tree.body
                if isinstance(n, ast.Assign) and getattr(n.targets[0], 'id', '') == 'SAYS')
    src = TOOL.read_text()
    published = set(re.findall(r"create_publisher\(\s*\w+\s*,\s*'([^']+)'", src))
    if 'TransformBroadcaster(' in src:
        published.add('/tf')
    assert says - {'/tf_static'} <= published, says - {'/tf_static'} - published
