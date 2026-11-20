"""The real LocalizationNode, built through rclpy, as the launch file builds it.

Every other node test stubs the ROS machinery out (`test_localization_node._node`),
which is why the constructor (parameters, subscriptions, the filter's own
configuration) had no test executing it. A topic renamed on one side, or a
filter built without the shipped priors, passes every stubbed test.
"""
import pytest

rclpy = pytest.importorskip('rclpy')

from mongla_localization.inekf import FLOW_SCALE_SIGMA  # noqa: E402
from mongla_localization.localization_node import (  # noqa: E402
    ACCEL_BIAS_PRIOR, LocalizationNode)

HEARS = {'/mongla/imu', '/mongla/state', '/mongla/demand',
         '/mongla/vision/downward/velocity', '/mongla/vision/downward/flow_calibration',
         '/mongla/vision/downward/floor_grid_deg', '/mongla/vision/downward/lane_heading_deg',
         '/mongla/localization/fix', '/mongla/localization/heading'}
SAYS = {'/mongla/odom', '/mongla/localization/state', '/mongla/localization/aiding',
        '/mongla/localization/motion', '/tf', '/tf_static'}


@pytest.fixture
def node():
    started = not rclpy.ok()
    if started:
        rclpy.init()
    n = LocalizationNode()
    try:
        yield n
    finally:
        n.destroy_node()
        if started:
            rclpy.shutdown()


def test_it_hears_and_says_the_topics_the_stack_uses(node):
    assert {s.topic_name for s in node.subscriptions} == HEARS
    assert SAYS <= {p.topic_name for p in node.publishers}


def test_the_filter_is_built_with_the_shipped_priors(node):
    f = node._filter
    assert f.odometry_scale_sigma == FLOW_SCALE_SIGMA
    assert f.P[12, 12] == pytest.approx(ACCEL_BIAS_PRIOR ** 2)
    assert node._zupt_enabled
    assert node._n['fix'] == 0 and node._n['fix_refused'] == 0


def test_retrodict_on_builds_the_replay_around_the_same_filter():
    from rclpy.parameter import Parameter
    started = not rclpy.ok()
    if started:
        rclpy.init()
    n = LocalizationNode(parameter_overrides=[Parameter('retrodict', value=True)])
    try:
        assert n._retro is not None
        assert n._retro.filt is n._filter
    finally:
        n.destroy_node()
        if started:
            rclpy.shutdown()


def test_the_survey_runs_as_a_program(tmp_path):
    """`ros2 run mongla_localization course_survey` is the module's __main__."""
    import os
    import subprocess
    import sys
    env = dict(os.environ, MONGLA_COURSE_DIR=str(tmp_path))
    out = subprocess.run([sys.executable, '-m', 'mongla_localization.course_survey',
                          '--course', 'robosub26', '--list'],
                         capture_output=True, text=True, env=env, timeout=60)
    assert out.returncode == 0, out.stderr
    assert 'robosub26' in out.stdout
