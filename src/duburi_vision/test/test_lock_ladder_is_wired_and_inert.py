"""The lock ladder must be LAUNCHED, and must not steer until asked.

⛔ WHY THIS FILE EXISTS. `lock_node` -- the follower + XFeat anchor ladder --
was built, measured (its own `anchor_hz` comment carries a Pi table, and the
anchor was recorded holding 175 consecutive frames the detector lost), given
a console_scripts entry point, and then appeared in **no launch file**. It
had never run in a mission. Its consumer existed too: `vision_state.py`
subscribes to `<ns>/lock` and `<ns>/target_pose`.

Two halves, and they fail in opposite directions:

  * NOT WIRED -- the capability is dead weight. Nothing errors, because
    nothing asks for it.
  * WIRED AND CONSUMED AT ONCE -- the control loop would start steering on
    followed and anchored boxes the moment the launch changed, with no deck
    measurement in between. `vision.lock_s` gates that, and it stays 0: the
    ladder publishes evidence first, exactly as the firmware vision uplink is
    staged (echo the numbers, confirm they match, THEN actuate).

Text-level: no ROS runtime, no camera.
"""
import pathlib
import re

_PKG = pathlib.Path(__file__).resolve().parents[1]
_PI_LAUNCH = _PKG / 'launch' / 'vision_pi.launch.py'
# The launch `bringup.launch.py` actually includes. The ladder was wired into
# vision_pi first, which the docs and bringup do NOT use -- a capability
# reachable only from a path no mission runs is the state this file exists to
# end, so both are checked.
_MAIN_LAUNCH = _PKG / 'launch' / 'vision.launch.py'
_TUNABLES = (_PKG.parents[0] / 'duburi_manager' / 'duburi_manager'
             / 'vision_tunables.py')


def test_the_ladder_is_actually_LAUNCHED():
    for path in (_PI_LAUNCH, _MAIN_LAUNCH):
        assert "executable='lock_node'" in path.read_text(), (
            f'lock_node is absent from {path.name}, so the follower and '
            f'XFeat anchor never run on that path. bringup.launch.py '
            f'includes vision.launch.py, so wiring only vision_pi leaves the '
            f'capability unreachable from the documented mission command.')


def test_the_ladder_is_reachable_from_the_launch_BRINGUP_INCLUDES():
    """Names the coupling explicitly, so a future edit to bringup's include
    does not quietly strand the ladder again."""
    bringup = (_PKG.parents[0] / 'duburi_manager' / 'launch'
               / 'bringup.launch.py').read_text()
    included = 'vision.launch.py' in bringup
    assert included, (
        'bringup no longer includes vision.launch.py -- update this test and '
        'make sure the ladder is wired into whatever it includes now.')
    assert "executable='lock_node'" in _MAIN_LAUNCH.read_text()


def test_the_ladder_can_be_turned_off_without_editing_the_launch():
    """Bumblebee's reflex, and ours: a fallback that needs a code edit is not
    a fallback. On a pool deck it has to be an argument."""
    src = _PI_LAUNCH.read_text()
    assert re.search(r"DeclareLaunchArgument\(\s*\n?\s*'lock'", src), (
        'the ladder has no `lock` launch argument, so disabling it on the '
        'deck means editing a launch file.')
    i = src.index("executable='lock_node'")
    window = src[i:i + 1200]
    assert "IfCondition(LaunchConfiguration('lock'))" in window, (
        'lock_node is not gated on the `lock` argument, so the argument is '
        'inert -- declared and unread, the defect class this package has '
        'produced four times.')


def test_control_does_NOT_consume_the_ladder_by_default():
    """The staging gate. `vision.lock_s` > 0 makes `bbox_error()` answer from
    the ladder instead of live detections; until the ladder is watched on the
    deck it must stay 0, so wiring the node changes what is PUBLISHED and
    nothing about what is STEERED."""
    src = _TUNABLES.read_text()
    m = re.search(r"'vision\.lock_s':\s*([0-9.]+)", src)
    assert m, "vision.lock_s is gone from vision_tunables"
    assert float(m.group(1)) == 0.0, (
        f'vision.lock_s defaults to {m.group(1)}, not 0. That makes the '
        f'control loop steer on followed/anchored boxes by default, which is '
        f'a vehicle-behaviour change that no deck measurement has cleared. '
        f'Raise it deliberately, per mission, after watching /lock.')


def test_the_ladder_publishes_lock_NOT_detections():
    """The invariant that keeps 'the vehicle has a position' from becoming
    'the vehicle saw the target'."""
    node = (_PKG / 'duburi_vision' / 'lock_node.py').read_text()
    pubs = re.findall(r"create_publisher\([^,]+,\s*f?['\"]?\{?ns\}?([^'\",]*)",
                      node)
    assert pubs, 'no publishers found in lock_node -- has it been rewritten?'
    for topic in pubs:
        assert 'detections' not in topic, (
            f'lock_node publishes to {topic!r}, which collides with the '
            f'detector topic. A followed box there is indistinguishable from '
            f'one the detector actually saw.')


def test_the_anchor_model_follows_DUBURI_HEF_DIR():
    """The model directory is one truth. The detector resolves models through
    DUBURI_HEF_DIR; the anchor hardcoded ~/hailo_models, so pointing the env
    var elsewhere moved the detector and silently not the anchor -- costing
    the long-horizon rung with one WARN and no error."""
    node = (_PKG / 'duburi_vision' / 'lock_node.py').read_text()
    # `def _build_anchor`, not `_build_anchor`: the first occurrence of the
    # bare name is the CALL SITE ~20 lines earlier, so a window anchored on it
    # covers the wrong region and this test failed against correct code.
    i = node.index('def _build_anchor')
    window = node[i:i + 1600]
    # ⛔ Match the CALL, not the name. A first draft asserted
    # `'DUBURI_HEF_DIR' in window` and passed against code where the lookup
    # had been replaced by '' -- because the comment ABOVE it still said
    # DUBURI_HEF_DIR. The test was reading prose. Injection is what exposed
    # it: the defect went in and nothing failed.
    assert "os.environ.get('DUBURI_HEF_DIR'" in window, (
        'the anchor does not READ DUBURI_HEF_DIR (a comment mentioning it is '
        'not a lookup), so it and the detector can resolve models from '
        'different directories -- the anchor rung then vanishes with one '
        'WARN and no error.')
