"""Yaw-source registry."""

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
    src = inspect.getsource(BUILDERS['bno085_sim_dvl'])
    assert 'SimDvlSource' in src
    assert 'NucleusDVLSource' not in src
    assert 'BNO085Source' in src
