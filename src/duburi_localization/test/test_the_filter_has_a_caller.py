"""The filter must be REACHABLE, not merely correct.

This file exists because `inekf.py` shipped as 19 passing tests and zero
callers: a converging estimator that no process ever constructed. Every
assertion here is about reachability -- entry point, launch file, build
script, DSL -- and each one is a way that work has silently scored nothing
before in this repo.

"Does anything actually CALL this, or am I admiring effort?" is the check.
"""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
PKG = ROOT / 'src' / 'duburi_localization'


def test_the_node_is_an_entry_point():
    """Without this, `ros2 run duburi_localization localization_node` and the
    launch file's `executable=` both fail at runtime, not at build."""
    setup = (PKG / 'setup.py').read_text()
    assert 'localization_node:main' in setup


def test_the_node_is_in_the_vehicle_launch():
    """A node nobody launches is the same as a node nobody wrote."""
    launch = (ROOT / 'src' / 'duburi_manager' / 'launch'
              / 'bringup.launch.py').read_text()
    assert "package='duburi_localization'" in launch
    assert "executable='localization_node'" in launch
    # And it must be RETURNED, not merely constructed -- a Node object left out
    # of the LaunchDescription is created, assigned, and never started.
    body = launch.split('return LaunchDescription')[-1]
    assert 'localization_node' in body


def test_the_package_is_in_the_build_script():
    """Dev tests pass from source; the vehicle runs what colcon built. A
    package missing here never builds there and nothing local can tell."""
    build = (ROOT / 'build_dubomini.sh').read_text()
    assert 'duburi_localization' in build


def test_the_filter_is_actually_constructed_by_the_node():
    """Guards against the node importing the filter and never using it, which
    is what a refactor does when it moves logic out and forgets to move it in."""
    src = (PKG / 'duburi_localization' / 'localization_node.py').read_text()
    assert 'RIEKF()' in src
    for method in ('predict', 'update_depth', 'update_body_velocity_xy',
                   'update_position', 'update_attitude',
                   'update_zero_velocity'):
        assert f'.{method}(' in src, f'{method} has no caller in the node'


def test_the_resection_fix_is_published_to_the_filter():
    """`fix_position()` computed a pool position, logged it, and dropped it.

    `update_position` is the only channel that bounds horizontal drift, so its
    caller is the difference between a filter that converges and one that
    dead-reckons with confidence.
    """
    dsl = (ROOT / 'src' / 'duburi_planner' / 'duburi_planner'
           / 'duburi_dsl.py').read_text()
    assert '_publish_fix' in dsl
    assert '/duburi/localization/fix' in dsl


def test_the_fused_pose_is_readable_from_a_mission():
    """If it is not in the DSL it cannot be used by a mission, which is the
    operator's stated bar for a capability existing at all."""
    dsl = (ROOT / 'src' / 'duburi_planner' / 'duburi_planner'
           / 'duburi_dsl.py').read_text()
    assert 'def pose(self' in dsl
    assert '/duburi/odom' in dsl


def test_the_manager_publishes_the_imu_the_filter_predicts_on():
    """The one input with no alternative source. Without `/duburi/imu` the
    filter never calls `predict` and is a measurement blender, not an
    estimator -- and it would still publish a confident pose."""
    mgr = (ROOT / 'src' / 'duburi_manager' / 'duburi_manager'
           / 'auv_manager_node.py').read_text()
    assert "'/duburi/imu'" in mgr
    assert '_publish_imu' in mgr
    fc = (ROOT / 'src' / 'duburi_control' / 'duburi_control' / 'fc'
          / 'srot_fc.py').read_text()
    assert 'def get_imu(self)' in fc


def test_the_single_prop_fix_exists_and_is_wired():
    """`fix_position()` needs TWO props 12 degrees apart; one prop of known
    width at a surveyed position pins the hull on its own. That is the
    difference between localising when the course cooperates and localising
    whenever anything is in view."""
    dsl = (ROOT / 'src' / 'duburi_planner' / 'duburi_planner'
           / 'duburi_dsl.py').read_text()
    assert 'def fix_from_prop(self' in dsl
    assert 'def range_to(self' in dsl
    # The fix must carry its own sigma: pose error from a planar target grows
    # with the SQUARE of range, so one constant cannot serve near and far.
    assert 'sigma=sigma' in dsl


def test_the_filter_reports_what_it_rejected():
    """A filter silently discarding measurements looks exactly like one that
    is merely drifting."""
    src = (PKG / 'duburi_localization' / 'localization_node.py').read_text()
    assert 'rejected' in src and 'lockout_breaks' in src
