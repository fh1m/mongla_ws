"""Props that move must be jointed the way the real ones are moored.

Two live bugs prompted these, and both looked like collision problems and were
not:

  * the RoboSub gate's role markers would not move when the hull hit them --
    the gate was `<static>true</static>`, and a static model welds every link to
    the world however much collision it has;
  * the three slalom pipes moved as one -- they were fixed-jointed into a single
    rigid body, so clipping the left pipe swung the centre and right ones too.
"""
import os
import re

import pytest

MODELS = os.path.join(os.path.dirname(__file__), '..', '..',
                      'duburi_sim_worlds', 'models')


def _sdf(name):
    path = os.path.join(MODELS, name, 'model.sdf')
    if not os.path.isfile(path):
        pytest.skip(f'{name} not generated')
    with open(path) as fh:
        return fh.read()


def _joints(sdf):
    return dict(re.findall(r'<joint name="([^"]+)" type="(\w+)"', sdf))


def _static(sdf):
    return re.search(r'<static>(\w+)</static>', sdf).group(1) == 'true'


# ---------------------------------------------------------------- gate
def test_gate_is_not_static():
    """The whole reason the markers could not be pushed."""
    assert not _static(_sdf('robosub_gate'))


def test_gate_frame_is_pinned_to_the_world():
    """Dynamic must not mean driftable: it is a moored 3 m structure, and a gate
    that wanders moves the geometry the scorer measures the run against."""
    sdf = _sdf('robosub_gate')
    assert _joints(sdf).get('gate_mooring') == 'fixed'
    assert '<parent>world</parent>' in sdf


def test_gate_boards_are_hinged():
    joints = _joints(_sdf('robosub_gate'))
    for name in ('sign_survey_repair_hinge', 'sign_search_rescue_hinge',
                 'divider_hinge'):
        assert joints.get(name) == 'revolute', name


def test_gate_boards_have_a_restoring_spring():
    """Gravity alone does NOT bring these back. Measured: with weight only the
    board sat at -5.3 deg 21.5 s after a knock, and raising damping made it
    worse (-41.3 deg at 15.4 s) because the missing term was the spring, not the
    damper. With the spring: home and still by 7.2 s."""
    sdf = _sdf('robosub_gate')
    springs = [float(v) for v in
               re.findall(r'<spring_stiffness>([\d.]+)</spring_stiffness>', sdf)]
    assert springs and max(springs) > 0.0


def test_gate_boards_are_damped():
    """A hinge with no drag rings at its own frequency indefinitely."""
    sdf = _sdf('robosub_gate')
    assert 'gz-sim-hydrodynamics-system' in sdf


# -------------------------------------------------------------- slalom
def test_slalom_pipes_are_not_welded_to_each_other():
    """The bug: one rigid body, so a hit on one pipe moved all three."""
    joints = _joints(_sdf('robosub_slalom'))
    for a in ('pipe_left', 'pipe_centre', 'pipe_right'):
        for b in ('pipe_left', 'pipe_centre', 'pipe_right'):
            if a == b:
                continue
            assert f'{a}_{b}_fix' not in joints, f'{a} welded to {b}'


def test_each_slalom_pipe_is_moored_and_hinged():
    """Un-welding alone is not enough and was measured: a free pipe took the hit,
    travelled 2.29 m in 3.7 s and then diverged the solver. Each pipe is moored
    at the floor and hinges about its base, which is what the handbook
    describes -- it can be pushed over, not pushed away."""
    joints = _joints(_sdf('robosub_slalom'))
    for pipe in ('pipe_left', 'pipe_centre', 'pipe_right'):
        assert joints.get(f'{pipe}_mooring') == 'fixed', pipe
        assert joints.get(f'{pipe}_hinge') == 'universal', pipe


def test_slalom_hinges_take_a_push_from_any_bearing():
    """A single-axis hinge yields along one heading and stands rigid along the
    other; a hull can brush a pipe from any bearing."""
    sdf = _sdf('robosub_slalom')
    assert sdf.count('<axis2>') == 3
