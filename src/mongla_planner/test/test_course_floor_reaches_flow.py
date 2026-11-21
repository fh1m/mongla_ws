"""A course's surveyed floor reaches flow as numbers, and a bad survey is refused.

`use_course` hands `pool: floor:` to flow as `pool_floor_xyz`. Before the
double-array case in `_param_value`, a list went over as its str() and the
node refused it, so a surveyed floor would never have scaled anything.
"""
import pytest
from rcl_interfaces.msg import ParameterType

from mongla_planner.dsl_support import _param_value


def test_a_list_of_numbers_is_a_double_array():
    v = _param_value([0.0, 0.0, 1.28, 17.5, 0.0, 2])
    assert v.type == ParameterType.PARAMETER_DOUBLE_ARRAY
    assert list(v.double_array_value) == [0.0, 0.0, 1.28, 17.5, 0.0, 2.0]


def test_the_course_loader_reads_and_checks_the_floor(tmp_path, monkeypatch):
    from mongla_localization import course_map
    monkeypatch.setenv('MONGLA_COURSE_DIR', str(tmp_path))
    (tmp_path / 'slope.yaml').write_text(
        'course: slope\nframe: pool\n'
        'pool:\n  depth_m: 1.28\n  floor: [[0, 0, 1.28], [17.5, 0, 2.0]]\n'
        'props: {}\n')
    c = course_map.load_course('slope')
    assert c.pool_floor() == [(0.0, 0.0, 1.28), (17.5, 0.0, 2.0)]
    (tmp_path / 'bad.yaml').write_text(
        'course: bad\nframe: pool\npool:\n  floor: [[0, 0, -1.2]]\nprops: {}\n')
    with pytest.raises(ValueError, match='positive'):
        course_map.load_course('bad')
