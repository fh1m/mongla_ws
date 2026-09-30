"""The `sensor_offsets` health line: unmeasured is DEGRADED and says the cost."""
from mongla_localization.frames import Offsets
from mongla_manager import health_reporters as hr
from mongla_manager.health import State


def test_nothing_measured_is_degraded_and_names_the_cost():
    h = hr.sensor_offsets(Offsets(None, None, None))
    assert h.state is State.DEGRADED
    assert 'omega x r' in h.evidence and 'baro' in h.evidence


def test_all_measured_is_ok():
    h = hr.sensor_offsets(Offsets((0.25, 0, 0.1), (0.3, 0, 0), (0, 0, -0.05)))
    assert h.state is State.OK


def test_an_unreadable_file_is_unknown_not_ok():
    assert hr.sensor_offsets(None).state is State.UNKNOWN


def test_the_shipped_file_is_what_the_vehicle_reports():
    """The live line reads the real file; today that is DEGRADED."""
    from mongla_manager.auv_manager_node import _load_sensor_offsets
    o = _load_sensor_offsets()
    assert o is not None, 'the manager cannot read frames.yaml'
    assert hr.sensor_offsets(o).state in (State.OK, State.DEGRADED)
