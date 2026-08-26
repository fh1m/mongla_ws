"""The sim and the autonomy stack are one unit — prove they still agree.

Mongla is one repo with two colcon workspaces: ``duburi_ws/`` (this one, the same
binaries that fly on the vehicle) and ``duburi_ws/sim/`` (Gazebo Harmonic +
ArduSub SITL + the operator lab). They are built and sourced separately, so
nothing at build time notices when one drifts away from the other.

This file is the referee, and it is deliberately the same idea as
``duburi_control/test/test_srot_protocol_drift.py``: read the OTHER side's source
directly and assert the shared surface still matches, rather than trusting two
copies of a constant to stay in step by good intentions.

**Why source-text/AST reading rather than importing the sim:** the sim packages
need ``rclpy`` + Gazebo on the path and are not built when this suite runs. The
whole point is to catch drift without a Gazebo install, which is also what makes
it safe to run on the Jetson.

Every test **skips cleanly when ``sim/`` is absent** — a sparse checkout that
excludes the sim (the documented 15 W Jetson recipe) must not fail this suite.
"""
import ast
import re
from pathlib import Path

import pytest

from duburi_manager.connection_config import PROFILES

REPO = Path(__file__).resolve().parents[3]
SIM = REPO / 'sim'

pytestmark = pytest.mark.skipif(
    not (SIM / 'src' / 'duburi_sim_bringup').is_dir(),
    reason='sim/ not present in this checkout (sparse clone) -- nothing to referee',
)

STACK_LAUNCH = SIM / 'src/duburi_sim_bringup/launch/stack.launch.py'
SIM_LAUNCH = SIM / 'src/duburi_sim_bringup/launch/sim.launch.py'
CONTRACT_CHECK = SIM / 'src/duburi_sim_bridge/duburi_sim_bridge/contract_check.py'
BRIDGE_LAUNCH = SIM / 'src/duburi_sim_bridge/launch/bridge.launch.py'
TELEOP = SIM / 'src/duburi_sim_web/duburi_sim_web/teleop.py'
SUB_PARM = SIM / 'src/duburi_sim_bringup/config/duburi_sub.parm'
VEHICLE_SDF = SIM / 'src/duburi_sim_description/models/duburi_heavy/model.sdf'

MANAGER_LAUNCH = REPO / 'src/duburi_manager/launch/bringup.launch.py'
VISION_LAUNCH = REPO / 'src/duburi_vision/launch/vision.launch.py'

# Launch arguments the sim passes that a given autonomy branch may not declare.
# An UNDECLARED argument is NOT an error at runtime -- launch's
# IncludeLaunchDescription.execute() raises only for *missing required* args and
# silently turns extras into launch configurations nobody reads. That asymmetry
# is why this test exists: without it, a typo'd or branch-only argument is a
# no-op with no log line anywhere.
#
#   flight_controller -- declared on `srot` (selects SROT-USB vs Pixhawk/ArduSub).
#     On `main` there is no such argument and the Pixhawk path is the only path,
#     so passing it is inert and harmless. Keep it in stack.launch.py so the same
#     sim drives both branches.
BRANCH_SPECIFIC_MANAGER_ARGS = {'flight_controller'}


def _declared_args(launch_file: Path) -> set:
    """Names passed to DeclareLaunchArgument('<name>', ...) in a launch file."""
    tree = ast.parse(launch_file.read_text())
    names = set()
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        fn = node.func
        if getattr(fn, 'id', None) == 'DeclareLaunchArgument' or \
                getattr(fn, 'attr', None) == 'DeclareLaunchArgument':
            if node.args and isinstance(node.args[0], ast.Constant):
                names.add(node.args[0].value)
    return names


def _launch_argument_dicts(launch_file: Path) -> list:
    """Every `launch_arguments={...}.items()` dict literal, as {key: value|None}."""
    tree = ast.parse(launch_file.read_text())
    out = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.keyword) or node.arg != 'launch_arguments':
            continue
        value = node.value
        # launch_arguments={...}.items() -> unwrap the .items() call
        if isinstance(value, ast.Call) and getattr(value.func, 'attr', None) == 'items':
            value = value.func.value
        if not isinstance(value, ast.Dict):
            continue
        d = {}
        for k, v in zip(value.keys, value.values):
            if isinstance(k, ast.Constant):
                d[k.value] = v.value if isinstance(v, ast.Constant) else None
        out.append(d)
    return out


def _profile_port(name: str) -> int:
    conn = PROFILES[name]['conn']
    return int(conn.rsplit(':', 1)[1])


# ---------------------------------------------------------------- launch args

def test_every_manager_arg_the_sim_passes_is_real():
    """A silently-ignored launch argument is the trap this whole file guards.

    If the sim asks the manager for `mode:=sim` and someone renames that
    argument, nothing raises -- the manager just comes up on its own default
    (`pool` on the launch file, which binds the same UDP port but prints the
    wrong banner and skips the sim sanity hints).
    """
    declared = _declared_args(MANAGER_LAUNCH)
    passed = set()
    for d in _launch_argument_dicts(STACK_LAUNCH):
        if d.get('mode') == 'sim':
            passed |= set(d)
    assert passed, 'stack.launch.py no longer includes the manager with mode:=sim'
    unknown = passed - declared - BRANCH_SPECIFIC_MANAGER_ARGS
    assert not unknown, (
        f'stack.launch.py passes {sorted(unknown)} to duburi_manager/bringup.launch.py, '
        f'which does not declare them. Launch will NOT error -- it will silently '
        f'ignore them. Declare the argument, fix the name, or add it to '
        f'BRANCH_SPECIFIC_MANAGER_ARGS with a note saying why being inert is safe.'
    )


def test_every_vision_arg_the_sim_passes_is_real():
    declared = _declared_args(VISION_LAUNCH)
    passed = set()
    for d in _launch_argument_dicts(STACK_LAUNCH):
        if 'camera' in d and 'topic' in d:
            passed |= set(d)
    assert passed, 'stack.launch.py no longer includes the vision launch'
    unknown = passed - declared
    assert not unknown, (
        f'stack.launch.py passes {sorted(unknown)} to duburi_vision/vision.launch.py, '
        f'which does not declare them -- they would be silently ignored, and the '
        f'detector would come up on its default model with no error.'
    )


def test_sim_vision_uses_the_camera_name_missions_expect():
    """Missions look for `/duburi_detector_forward`; `camera:=` names the node."""
    for d in _launch_argument_dicts(STACK_LAUNCH):
        if 'camera' in d and 'topic' in d:
            assert d['camera'] == 'forward', (
                f"stack.launch.py launches vision as camera:={d['camera']!r}. "
                f"That names the node /duburi_detector_{d['camera']} while every "
                f"mission and the DSL look for /duburi_detector_forward."
            )
            return
    pytest.fail('no vision include found in stack.launch.py')


# -------------------------------------------------------------------- MAVLink

def test_sim_pushes_mavlink_to_the_port_the_sim_profile_binds():
    src = SIM_LAUNCH.read_text()
    primary = int(re.search(r'MAVLINK_PRIMARY_PORT\s*=\s*(\d+)', src).group(1))
    assert primary == _profile_port('sim'), (
        f'ArduSub SITL pushes to {primary} but PROFILES["sim"] binds '
        f'{_profile_port("sim")} -- the manager would sit on a silent socket.'
    )
    assert 'udpclient:127.0.0.1:{MAVLINK_PRIMARY_PORT}' in src, (
        'ArduSub must be the udp CLIENT on --serial0 and the manager the '
        'listener (PROFILES["sim"] is a udpin bind); two listeners connect to '
        'nothing and the manager waits forever on a silent socket.'
    )


def test_lab_teleop_does_not_steal_the_managers_socket():
    """The single rule that breaks arming: two things on 14550.

    The lab drives RC overrides over ArduSub's SITL TCP serial instead, so an
    operator can fly the sub by hand while the manager keeps its own link.
    """
    # DEFAULT_ENDPOINT = os.environ.get('DUBURI_TELEOP_ENDPOINT', 'tcp:...')
    # -- take the literal default, which is the one an operator actually gets.
    m = re.search(r"DEFAULT_ENDPOINT\s*=.*?,\s*'([^']+)'\s*\)", TELEOP.read_text())
    assert m, 'teleop.py no longer defines DEFAULT_ENDPOINT with a literal default'
    endpoint = m.group(1)
    assert str(_profile_port('sim')) not in endpoint, (
        f'lab teleop endpoint {endpoint} collides with the manager sim profile. '
        f'Both binds succeed (SO_REUSEADDR) and only one socket receives.'
    )


# --------------------------------------------------------------------- camera

def test_camera_topics_agree_across_sim_bridge_stack_and_check():
    src = STACK_LAUNCH.read_text()
    topic = re.search(r"default_value='(/duburi/sim/\S+?)'", src).group(1)
    for name, path in (('contract_check', CONTRACT_CHECK), ('bridge.launch', BRIDGE_LAUNCH)):
        assert topic in path.read_text(), (
            f'stack.launch.py feeds vision {topic}, but {name}.py never mentions it. '
            f'The detector would subscribe to a topic nobody publishes and simply '
            f'never fire -- no error, no detections.'
        )


def test_camera_resolution_agrees_between_the_model_and_the_checker():
    w = int(re.search(r'EXPECTED_WIDTH\s*=\s*(\d+)', CONTRACT_CHECK.read_text()).group(1))
    h = int(re.search(r'EXPECTED_HEIGHT\s*=\s*(\d+)', CONTRACT_CHECK.read_text()).group(1))
    sdf = VEHICLE_SDF.read_text()
    assert f'<width>{w}</width>' in sdf and f'<height>{h}</height>' in sdf, (
        f'contract_check demands {w}x{h} but model.sdf renders something else. '
        f'model.sdf is GENERATED -- fix configs.yaml and re-run '
        f'scripts/generate_model.py, never the .sdf by hand.'
    )


# --------------------------------------------------------------------- thrust

def test_thruster_pwm_range_agrees_between_the_model_and_ardusub():
    parm = SUB_PARM.read_text()
    lo = int(re.search(r'MOT_PWM_MIN\s+(\d+)', parm).group(1))
    hi = int(re.search(r'MOT_PWM_MAX\s+(\d+)', parm).group(1))
    sdf = VEHICLE_SDF.read_text()
    assert f'<servo_min>{lo}</servo_min>' in sdf, (
        f'duburi_sub.parm says MOT_PWM_MIN={lo}; the Gazebo plugin disagrees. '
        f'A mismatch silently rescales every thrust command, so gains tuned in '
        f'sim will not transfer to the pool.'
    )
    assert f'<servo_max>{hi}</servo_max>' in sdf, (
        f'duburi_sub.parm says MOT_PWM_MAX={hi}; the Gazebo plugin disagrees.'
    )
