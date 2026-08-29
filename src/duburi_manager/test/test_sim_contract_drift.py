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
import sys
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
VISION_DUAL_LAUNCH = REPO / 'src/duburi_vision/launch/vision_dual.launch.py'

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
    """The sim now drives BOTH cameras, so it includes vision_dual, not vision.

    Same trap as the manager args above: an argument vision_dual does not
    declare is silently dropped and the detector comes up on its default model
    with no error anywhere.
    """
    declared = _declared_args(VISION_DUAL_LAUNCH)
    passed = set()
    for d in _launch_argument_dicts(STACK_LAUNCH):
        if 'fwd_topic' in d and 'dwn_topic' in d:
            passed |= set(d)
    assert passed, 'stack.launch.py no longer includes the dual-camera vision launch'
    unknown = passed - declared
    assert not unknown, (
        f'stack.launch.py passes {sorted(unknown)} to '
        f'duburi_vision/vision_dual.launch.py, which does not declare them -- '
        f'they would be silently ignored, and the detector would come up on its '
        f'default model with no error.'
    )


def test_sim_vision_drives_both_cameras_from_the_sim_topics():
    """Front -> forward, bottom -> downward.

    The camera NAME is what missions and the DSL resolve
    (`/duburi_detector_forward`, `_vision_state_for('downward')`), and the sim
    publishes `front_camera`/`bottom_camera`. Getting that mapping backwards
    would point the bin detector at the gate camera and detect nothing, without
    an error.
    """
    # The include passes LaunchConfiguration references, so the mapping lives in
    # the DEFAULTS of the arguments it forwards, not in the include itself.
    src = STACK_LAUNCH.read_text()
    found = False
    for d in _launch_argument_dicts(STACK_LAUNCH):
        if 'fwd_topic' in d and 'dwn_topic' in d:
            # The VALUES are LaunchConfiguration references, which the source
            # parser cannot evaluate and reports as None. Key presence is the
            # only thing checkable here; the actual mapping is asserted from
            # the declared defaults below.
            found = True
    assert found, 'no dual-camera vision include found in stack.launch.py'

    fwd_default = re.search(
        r"'image_topic',\s*\n\s*default_value='([^']+)'", src)
    dwn_default = re.search(
        r"'bottom_image_topic',\s*\n\s*default_value='([^']+)'", src)
    assert fwd_default and 'front_camera' in fwd_default.group(1), (
        'the forward detector must default to the sim FRONT camera')
    assert dwn_default and 'bottom_camera' in dwn_default.group(1), (
        'the downward detector must default to the sim BOTTOM camera')


def test_the_manager_include_is_scoped():
    """`vision:=true` silently did nothing for the life of this file.

    IncludeLaunchDescription does NOT scope its launch_arguments. The manager
    include passes `vision: 'false'` (correctly -- the manager must not start its
    own vision), and that leaked into the outer scope and overwrote this file's
    own `vision` argument. The vision include then evaluated
    IfCondition(LaunchConfiguration('vision')) as false and skipped itself, with
    no error, for ANY value of vision:=.

    A GroupAction(scoped=True) is what confines it. Without the scope the bug
    returns and the only symptom is that no vision node starts.
    """
    src = STACK_LAUNCH.read_text()
    assert 'GroupAction(' in src and 'scoped=True' in src, (
        'the manager include is no longer scoped -- vision:=true will silently '
        'start nothing again'
    )


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


def test_every_course_has_a_turbidity_sidecar_beside_its_world():
    """`lighting:` must reach the image, not just the world file.

    gz-sim 8 ignores <scene><fog> on camera renders (measured 2026-08-28:
    fog_end 18 m -> 3 m left a 25 m wall pixel-identical), so a course's
    turbidity is carried by <course>.fx.yaml and loaded into underwater_fx by
    bridge.launch.py. A world without its sidecar silently falls back to the
    package default and the preset goes back to being decorative -- which is the
    exact bug this pair replaced, so it is worth a test rather than a habit.
    """
    worlds = sorted((SIM / 'src/duburi_sim_worlds/worlds').glob('*.world'))
    assert worlds, 'no generated worlds found'
    missing = [w.name for w in worlds if not w.with_suffix('.fx.yaml').exists()]
    assert not missing, (
        f'worlds with no turbidity sidecar: {missing}. '
        'Run scripts/gen_world.py --all rather than hand-editing worlds/.'
    )


def test_the_sidecar_carries_the_params_underwater_fx_declares():
    """A key the node does not declare is silently ignored by rclpy."""
    import yaml

    fx_node = (SIM / 'src/duburi_sim_bridge/duburi_sim_bridge/'
                     'underwater_fx.py').read_text()
    declared = set(re.findall(r"declare_parameter\(\s*'([^']+)'", fx_node))
    assert declared, 'could not parse underwater_fx parameters'

    sidecar = SIM / 'src/duburi_sim_worlds/worlds/sauvc26_final.fx.yaml'
    keys = set(yaml.safe_load(sidecar.read_text())['/**']['ros__parameters'])
    assert keys, 'sidecar declares nothing'
    assert keys <= declared, (
        f'sidecar sets parameters underwater_fx does not declare: '
        f'{sorted(keys - declared)}'
    )


def test_the_murky_preset_is_actually_murkier_than_the_clear_one():
    """Guards the ordering, not the numbers -- the presets are pool-tunable."""
    import yaml

    def turbidity(course):
        path = SIM / f'src/duburi_sim_worlds/worlds/{course}.fx.yaml'
        return yaml.safe_load(path.read_text())['/**']['ros__parameters']['turbidity']

    # sauvc26_final is `murky`, task_navigation is `competition`.
    assert turbidity('sauvc26_final') > turbidity('task_navigation')


def test_dvl_beam_visuals_are_off_in_the_generated_model():
    """The most expensive flag in the model, and it looks harmless.

    <visualize> on the DVL draws debug beams on the SAME Ogre2 render thread
    the cameras use. Measured 2026-08-28 on the SAUVC final course, headless:
    12.75 Hz / 6 ms frame jitter with it off, 2.83 Hz / 435 ms with it on. The
    jitter is what an operator sees as laggy teleop video and juddery recorded
    datasets, so a well-meaning flip of this flag is a real regression that no
    other test would catch. dvl_bridge.py publishes the same beams to RViz.
    """
    sdf = (SIM / 'src/duburi_sim_description/models/duburi_heavy/'
                 'model.sdf').read_text()
    dvl = sdf[sdf.index('<sensor name="dvl"'):sdf.index('</sensor>',
                                                       sdf.index('<sensor name="dvl"'))]
    assert '<visualize>false</visualize>' in dvl, (
        'DVL beam visuals are ON in the generated model. Set '
        'dvl.visualize_beams: false and regenerate, or accept 4.5x fewer frames.'
    )


def _prop_library():
    import importlib.util
    path = SIM / 'src/duburi_sim_worlds/scripts/prop_library.py'
    spec = importlib.util.spec_from_file_location('prop_library', path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules['prop_library'] = mod
    spec.loader.exec_module(mod)
    return mod


def test_each_competition_has_its_own_pool_and_textures():
    """Two competitions, two pools, and nothing shared that is size-dependent.

    RoboSub's pool is 2.1 m deep and SAUVC's is 1.6 m. Depth-spanning props bake
    pool depth in at generation time, and the floor/wall textures are sized from
    the pool, so a shared spec or a shared texture model produces geometry and
    tile pitch that are wrong with no error anywhere -- the class of failure
    this repo keeps meeting.
    """
    pl = _prop_library()
    comps = pl.competitions()
    assert {'sauvc', 'robosub'} <= set(comps)

    depths, tex = {}, set()
    for c in comps:
        spec = pl.load_spec(competition=c)
        depths[c] = spec['pool']['depth']
        tex.add(pl.texture_model(c))
    assert depths['sauvc'] != depths['robosub'], (
        'the two pools have the same depth; if that is now true on purpose, '
        'this test is what should change')
    assert len(tex) == len(comps), 'competitions share a texture model'


def test_a_world_is_generated_against_its_own_competition():
    """The course's `competition:` must reach the pool, not just the spec.

    A RoboSub world carrying `sauvc_pool` and `model://sauvc_textures` is the
    exact bug this caught during the refactor: pool DIMENSIONS came from the
    right spec while the model name and textures came from the default, because
    `--spec` defaulted to the SAUVC file and overrode the per-course choice.
    """
    import re
    worlds = SIM / 'src/duburi_sim_worlds/worlds'
    courses = SIM / 'src/duburi_sim_worlds/courses'
    for course in sorted(courses.glob('*.yaml')):
        world = worlds / f'{course.stem}.world'
        if not world.exists():
            continue
        text = course.read_text()
        m = re.search(r'^competition:\s*(\S+)', text, re.M)
        comp = m.group(1) if m else 'sauvc'
        w = world.read_text()
        assert f'<model name="{comp}_pool"' in w, f'{world.name}: wrong pool model'
        others = {f'model://{c}_textures' for c in ('sauvc', 'robosub')} - {
            f'model://{comp}_textures'}
        for wrong in others:
            assert wrong not in w, f'{world.name} references {wrong}'


def test_lab_pool_profile_matches_the_arena_specs():
    """The lab's altitude readout duplicates the pool geometry in JS.

    App.jsx has no route to spec/*.yaml, so it carries its own copy of each
    pool's length/depth/edge-depth to turn ground-truth z into an altitude. An
    operator reads that number to decide whether the vehicle is about to
    ground, and a stale copy is wrong by up to the full slope -- 0.4 m at
    SAUVC's ends. Nothing but this test keeps the two in step.
    """
    import re

    pl = _prop_library()
    js = (SIM / 'src/duburi_sim_web/frontend/src/App.jsx').read_text()
    block = re.search(r'const POOL_PROFILE = \{(.*?)\n\}', js, re.S)
    assert block, 'POOL_PROFILE missing from App.jsx'

    for comp in ('sauvc', 'robosub'):
        row = re.search(
            comp + r':\s*\{\s*length:\s*([\d.]+),\s*depth:\s*([\d.]+),\s*'
            r'edge:\s*([\d.]+|null)', block.group(1))
        assert row, f'{comp} missing from POOL_PROFILE'
        pool = pl.load_spec(competition=comp)['pool']
        assert float(row.group(1)) == pytest.approx(pool['length'])
        assert float(row.group(2)) == pytest.approx(pool['depth'])
        edge = pool.get('floor_edge_depth')
        if row.group(3) == 'null':
            assert not edge, f'{comp} slopes in the spec but is flat in the lab'
        else:
            assert float(row.group(3)) == pytest.approx(edge)


def test_floor_anchored_props_sit_on_the_sloped_floor():
    """A flat -depth placement leaves props hanging over a sloped floor.

    SAUVC's target zone is 2 m from the far wall where the floor has risen to
    1.26 m; a drum placed at a flat -1.6 m floats 0.34 m with nothing under it.
    """
    import re

    pl = _prop_library()
    pool = pl.load_spec(competition='sauvc')['pool']
    world = (SIM / 'src/duburi_sim_worlds/worlds/sauvc26_final.world').read_text()
    for name in ('drum_blue', 'orange_flare', 'gate'):
        m = re.search(rf'<name>{name}</name>\s*<pose>(\S+) (\S+) (\S+)', world)
        assert m, name
        x, z = float(m.group(1)), float(m.group(3))
        assert z == pytest.approx(-pl.floor_depth_at(pool, x), abs=1e-3), (
            f'{name} at x={x} sits at {z}, not on the floor')


def test_courses_stay_within_a_render_budget():
    """Props cost RENDER time, not collision time -- and that is the surprise.

    Measured 2026-08-29 on sauvc26_final with ArduSub: the full course ran at
    RTF 0.37-0.65 while the SAME world with every prop stripped ran at 1.00.
    Cutting collision shapes 101 -> 37 changed nothing. Halving the drums' draw
    calls (a 20-segment interior liner became one cylinder) took it to
    0.71-0.91.

    So the number to watch is VISUALS, not collisions, and it is paid four
    times over -- two cameras plus two bounding-box cameras all render the
    scene. A prop that adds twenty visuals adds eighty draw calls per step.

    The budget is deliberately loose; it exists to catch a prop that quietly
    adds a ring of fifty segments, not to police careful work.
    """
    import re

    worlds = SIM / 'src/duburi_sim_worlds/worlds'
    models = SIM / 'src/duburi_sim_worlds/models'
    counts = {d.name: (d / 'model.sdf').read_text().count('<visual')
              for d in models.iterdir() if (d / 'model.sdf').is_file()}

    for world in sorted(worlds.glob('*.world')):
        text = world.read_text()
        used = re.findall(r'<uri>model://([a-z_0-9]+)</uri>', text)
        total = sum(counts.get(m, 0) for m in used)
        assert total <= 260, (
            f'{world.name} draws {total} prop visuals; every one is rendered by '
            f'both cameras AND both bounding-box cameras. Simplify the worst '
            f'offender rather than raising this number.')
