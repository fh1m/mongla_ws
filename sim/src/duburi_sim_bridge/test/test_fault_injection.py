"""Fault injection: the parts that do not need a running simulator.

The faults themselves are measured against a live sim (see FAULTS.md); these
pin the invariants that would otherwise break silently.
"""

import os
import re

import pytest

from duburi_sim_bridge.fault_injection import (
    CAMERA_PROCESSES,
    DVL_OUT,
    DVL_RAW,
    FaultInjection,
    _MavlinkRelay,
)
from duburi_sim_bridge.t200_curve import T200Curve


HERE = os.path.dirname(os.path.abspath(__file__))


def test_dvl_interposition_names_do_not_collide():
    """The sensor must publish the RAW topic and the node the plain one.

    If these were ever the same string the node would subscribe to its own
    output: an echo loop that looks like a working DVL until it saturates.
    """
    assert DVL_RAW != DVL_OUT
    assert DVL_OUT == '/dvl/velocity', (
        'SimDvlSource in duburi_sensors reads this exact topic')


def test_the_vehicle_model_publishes_the_raw_dvl_topic():
    """The interposition only works if the sensor was actually renamed.

    Left at `dvl/velocity`, the sensor and this node would both publish the
    topic the stack reads, a dropout would do nothing, and the fault would
    quietly pass.
    """
    sdf = os.path.join(HERE, '..', '..', 'duburi_sim_description', 'models',
                       'duburi_heavy', 'model.sdf')
    if not os.path.exists(sdf):
        pytest.skip('generated vehicle model not present')
    with open(sdf) as f:
        body = f.read()
    assert '<topic>dvl/velocity_raw</topic>' in body
    assert '<topic>dvl/velocity</topic>' not in body


def test_camera_match_cannot_hit_the_injector_itself():
    """A fault injector that SIGSTOPs itself needs a sim restart to undo."""
    own = ' '.join(['python3', __file__])
    assert not any(p in own for p in CAMERA_PROCESSES)


def test_dead_thruster_parsing():
    """`[0]` is the no-fault sentinel; an empty array cannot round-trip."""
    assert T200Curve._parse_dead([0]) == set()
    assert T200Curve._parse_dead([]) == set()
    assert T200Curve._parse_dead(None) == set()
    assert T200Curve._parse_dead([3]) == {3}
    assert T200Curve._parse_dead([3, 5, 0]) == {3, 5}


def test_relay_starts_unblocked():
    """A relay that came up blocked would look like a dead autopilot link."""
    relay = _MavlinkRelay(14559, 14550, _NullLogger())
    assert relay.blocked is False


def test_relay_ports_differ():
    """Listening and forwarding on one port is a self-send loop."""
    import inspect

    src = inspect.getsource(FaultInjection.__init__)
    listen = re.search(r"'relay_listen_port',\s*(\d+)", src)
    forward = re.search(r"'relay_forward_port',\s*(\d+)", src)
    assert listen and forward
    assert listen.group(1) != forward.group(1)
    assert forward.group(1) == '14550', 'the manager binds udpin:14550'


class _NullLogger:
    def info(self, *_):
        pass

    def warn(self, *_):
        pass

    def error(self, *_):
        pass
