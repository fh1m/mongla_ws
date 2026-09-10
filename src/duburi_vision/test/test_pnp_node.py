"""The bag must carry the EVIDENCE, or "re-solvable from a bag" is a slogan.

`pnp_node`'s own numeric behaviour is driven end to end in
`test_refraction_in_the_pose_path.py` -- a ray-traced square in, a range in
metres out, with air-optics and unstated-resolution negative controls. What
lives here is the wiring around it, because every part of that wiring was
either missing or wrong when it was first checked:

  * the record allowlist did not include `correspondences`, so a recorded run
    could not have been re-solved at all;
  * a ladder can be launched without the solver that gives it a pose.
"""
import re
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[3]
_RECORD = _ROOT / 'scripts' / 'pool_record.sh'
_LAUNCH = (Path(__file__).resolve().parents[1] / 'launch'
           / 'vision_pi.launch.py')


def _allowlists():
    src = _RECORD.read_text()
    out = {}
    for name in ('ALLOW_DEBUG', 'ALLOW_FULL'):
        m = re.search(name + r"='([^']+)'", src)
        assert m, f'{name} not found in {_RECORD}'
        out[name] = m.group(1)
    return out


def test_a_run_records_the_evidence_not_only_the_conclusion():
    """A pose is a lossy summary of the points it was fitted to.

    Record only the pose and a missed shot is unexplainable afterwards: bad
    evidence and a bad solve look identical. This was genuinely absent --
    neither allowlist matched `correspondences` when it was first checked.
    """
    for name, rx in _allowlists().items():
        for topic in ('/duburi/vision/forward/correspondences',
                      '/duburi/vision/forward/target_pose',
                      '/duburi/vision/downward/correspondences'):
            assert re.match(rx, topic), f'{name} does not record {topic}'


def test_the_allowlist_is_still_an_allowlist():
    """The negative control: a regex that matches everything records nothing
    useful and fills the card. Widening it is how that happens."""
    for name, rx in _allowlists().items():
        for topic in ('/rosout', '/parameter_events', '/tf'):
            assert not re.match(rx, topic), f'{name} now records {topic}'


def test_the_solver_launches_wherever_the_ladder_does():
    """`lock_node` publishes correspondences and no longer solves.

    So a ladder started without its solver leaves `target_pose` silent while
    every node looks healthy -- the capability-present-but-unreachable failure
    this stack keeps hitting. Both are gated on the same `lock` condition.
    """
    src = _LAUNCH.read_text()
    assert "executable='pnp_node'" in src, 'no solver in the vision launch'
    assert src.count("solver('") == src.count("ladder('"), (
        'a ladder is launched without a solver, or the reverse')
    assert src.count("condition=IfCondition(LaunchConfiguration('lock'))") >= 2


def test_one_medium_argument_reaches_all_three_nodes():
    """Flow, ladder and solver each apply the flat-port map.

    The argument was `flow_medium` when only the flow node read it. Three nodes
    reading three separately-defaulted values is how two of them come to
    disagree about whether the vehicle is in water -- and each would report a
    plausible number.
    """
    src = _LAUNCH.read_text()
    # The USE, not the word: the argument's own description mentions the old
    # name deliberately, so an operator with `flow_medium:=` in a script finds
    # out what replaced it instead of getting an unknown-argument error.
    assert "LaunchConfiguration('flow_medium')" not in src, (
        'the old flow-only name is still wired to a node')
    assert "DeclareLaunchArgument(\n            'flow_medium'" not in src
    assert src.count("LaunchConfiguration('medium')") >= 3, (
        'medium does not reach flow_node, lock_node and pnp_node')
    parent = (_ROOT / 'src' / 'duburi_manager' / 'launch'
              / 'bringup.launch.py').read_text()
    assert "DeclareLaunchArgument('medium'" in parent
    assert "'medium':        LaunchConfiguration('medium')" in parent, (
        'bringup declares medium but does not forward it to the vision launch')
