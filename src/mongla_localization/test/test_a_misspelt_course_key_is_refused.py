"""A course or frames file key the loader does not read is refused at load.

Both loaders took what they knew and dropped the rest. A course surveyed with
`bearing: 90` for `bearing_deg: 90` loaded as "bearing unset", and the anchor
refused minutes later with a message naming the right key, as if nobody had
measured it. A quoted `measured: "false"` became True, because `bool` of any
non-empty string is True, and an unmeasured nominal was reported as ours. A
taped lever arm under `downwards_cam:` loaded as "not measured" and was never
applied. Each is a measurement typed in on the day and silently thrown away.
"""
import pytest

from mongla_localization import course_map, frames


def _course(tmp_path, monkeypatch, text):
    (tmp_path / 'venue.yaml').write_text(text)
    monkeypatch.setenv(course_map._ENV, str(tmp_path))
    return lambda: course_map.load_course('venue')


def test_a_good_course_still_loads(tmp_path, monkeypatch):
    load = _course(tmp_path, monkeypatch, """
course: venue
pool: {depth_m: 1.6}
props:
  gate: {x_m: 16.0, y_m: 0.0, bearing_deg: 0, detect_class: gate, measured: true}
""")
    c = load()
    assert c.position_of('gate') == (16.0, 0.0)
    assert c.prop('gate').measured is True


def test_a_misspelt_prop_key_is_refused_with_the_right_name(tmp_path, monkeypatch):
    load = _course(tmp_path, monkeypatch, """
props:
  gate: {x_m: 16.0, y_m: 0.0, bearing: 90}
""")
    with pytest.raises(ValueError, match="'bearing'.*did you mean 'bearing_deg'"):
        load()


def test_a_quoted_false_is_not_read_as_measured(tmp_path, monkeypatch):
    load = _course(tmp_path, monkeypatch, """
props:
  gate: {x_m: 16.0, y_m: 0.0, measured: "false"}
""")
    with pytest.raises(ValueError, match='measured'):
        load()


def test_an_unknown_top_level_or_pool_key_is_refused(tmp_path, monkeypatch):
    load = _course(tmp_path, monkeypatch, "pool: {depth: 1.6}\nprops: {}\n")
    with pytest.raises(ValueError, match="'depth'"):
        load()
    load = _course(tmp_path, monkeypatch, "prop: {}\n")
    with pytest.raises(ValueError, match="'prop'"):
        load()


def test_every_shipped_course_loads(monkeypatch, tmp_path):
    monkeypatch.setenv(course_map._ENV, str(tmp_path))      # nothing there
    monkeypatch.setattr(course_map, '_HOME', tmp_path)
    shipped = sorted(course_map._PKG_COURSES.glob('*.yaml'))
    assert shipped
    for p in shipped:
        course_map.load_course(p.stem)


def test_a_misspelt_lever_arm_is_refused(tmp_path):
    f = tmp_path / 'frames.yaml'
    f.write_text('imu_to:\n  downwards_cam: [0.1, 0.0, 0.05]\n')
    with pytest.raises(ValueError, match='downwards_cam'):
        frames.load(f)


def test_the_shipped_frames_file_loads():
    assert frames.load().unmeasured() == ['downward_cam', 'forward_cam', 'baro']
