"""Yaw-source registry."""
import sys as _sys_ct
import pathlib as _pl_ct
_sys_ct.path.insert(0, str(_pl_ct.Path(__file__).resolve().parents[3] / 'tools'))
from code_text import code_of, code_of_file  # noqa: E402  (issue #22)

def test_bno085_sim_dvl_is_registered():
    """The sim twin of the pool's own configuration.

    `bno085_dvl` (BNO heading + Nucleus position) is what the vehicle flies. In
    simulation the Nucleus is unreachable, so without this entry the pool
    configuration cannot be rehearsed at all -- the composite comes up
    heading-only and every *_dist verb is dead.
    """
    from mongla_sensors.factory import BUILDERS

    assert 'bno085_sim_dvl' in BUILDERS
    # It must pair a BNO with the GAZEBO dvl, not the Nucleus -- otherwise it
    # is just bno085_dvl under a different name.
    import inspect
    src = code_of(BUILDERS['bno085_sim_dvl'])
    assert 'SimDvlSource' in src
    assert 'NucleusDVLSource' not in src
    assert 'BNO085Source' in src
