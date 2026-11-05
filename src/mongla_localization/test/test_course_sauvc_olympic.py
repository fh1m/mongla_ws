"""The olympic-pool SAUVC course ships as structure, never as invented points.

The rulebook gives zones, and its arena figure is "not to scale", so a number
typed into this file would be a guess the mission then drives to. The
loader's refusal of an unmeasured prop is what keeps a guess off the vehicle;
these pin the file to that shape.
"""
from mongla_localization import course_map as cm


def _olympic():
    # Read the PACKAGE copy: a deck copy surveyed at the venue is correct
    # there and must not decide what the repository ships.
    import yaml
    body = yaml.safe_load((cm._PKG_COURSES / 'sauvc_olympic.yaml').read_text())
    return body['props']


def test_the_course_loads_through_the_real_loader():
    c = cm.load_course('sauvc_olympic')
    assert {'final_gate', 'qual_gate', 'surfacing_square'} <= set(c.props)


def test_every_prop_ships_unmeasured_and_without_a_position():
    for name, body in _olympic().items():
        assert body.get('measured') is False, name
        for k in ('x', 'y', 'depth_m', 'bearing_deg'):
            assert body.get(k) is None, f'{name}.{k} is a guess, not a survey'


def test_the_two_venues_name_the_same_model_classes():
    """One detection model serves both venues; a class only one file names is a
    typo or a class no model emits."""
    import yaml
    old = yaml.safe_load((cm._PKG_COURSES / 'sauvc26.yaml').read_text())['props']
    new = {b['detect_class'] for b in _olympic().values() if b['detect_class']}
    assert new == {b['detect_class'] for b in old.values() if b.get('detect_class')}


def test_the_surfacing_square_claims_no_class():
    """No shipped model emits it. Naming one here would let a mission steer on a
    class that never arrives -- the silent `[]` this repo has shipped before."""
    assert _olympic()['surfacing_square']['detect_class'] == ''
