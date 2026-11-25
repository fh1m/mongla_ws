"""After the link goes quiet, /mongla/state must say ABSENT, not the last value.

pymavlink keeps the last message of every type forever. So after a USB drop
`SrotFC.get_attitude` kept returning the final yaw/depth the board ever sent,
and `_effective_yaw_deg` -- when the configured yaw source correctly withheld a
stale sample -- fell straight back to that same cached attitude. /mongla/state
then published a frozen, confident vehicle: safety rule 6's "plausible number
standing in for an absent measurement".

Real `SrotFC` on a fake master, real `BoardYaw`, the node's real
methods run unbound on a stub; only the age of the cached ATTITUDE is varied.
"""
import math

import pytest
import pathlib
import sys
import time
from types import SimpleNamespace

from builtin_interfaces.msg import Time

from mongla_manager import auv_manager_node as amn
from mongla_control.fc import srot_fc as srot_mod
from mongla_manager.yaw_source import BoardYaw, STALE_SECONDS

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2]
                       / 'mongla_control' / 'test'))
from test_srot_fc import _fc   # noqa: E402


def _fc_with_attitude(age_s):
    fc = _fc()
    fc.master.messages['ATTITUDE'] = SimpleNamespace(
        yaw=math.radians(123.0), roll=0.0, pitch=0.0,
        _timestamp=time.time() - age_s)
    fc.master.messages['VFR_HUD'] = SimpleNamespace(_timestamp=time.time(), alt=-1.7)
    return fc


class _Pub:
    def __init__(self):
        self.msgs = []

    def publish(self, m):
        self.msgs.append(m)


class _StubNode:
    def __init__(self, fc):
        self.fc = fc
        self.yaw_source = BoardYaw(fc)
        self.state_publisher = _Pub()
        self.thruster_pack_publisher = _Pub()
        self.safety_publisher = _Pub()

    def _state_stamp(self):
        return Time()


def test_a_fresh_attitude_is_still_returned():
    att = _fc_with_attitude(0.0).get_attitude()
    assert att is not None
    assert att['yaw'] == 123.0 or abs(att['yaw'] - 123.0) < 1e-6
    assert att['depth'] == -1.7


def test_an_attitude_older_than_the_link_window_is_absent():
    fc = _fc_with_attitude(srot_mod._LINK_STALE_S + 2.0)
    assert fc.get_attitude() is None, 'a frozen ATTITUDE was reported as live'


def test_a_withheld_yaw_is_not_backfilled_from_the_cached_attitude():
    """The source says stale (older than its 250 ms contract); the node must
    not answer with the very sample the source just refused."""
    age = (STALE_SECONDS + srot_mod._LINK_STALE_S) / 2.0   # stale for the source
    node = _StubNode(_fc_with_attitude(age))
    assert node.yaw_source.read_yaw() is None
    yaw, label = amn.AUVManagerNode._effective_yaw_deg(node, node.fc.get_attitude())
    assert yaw is None and label == 'N/A', (
        f'stale yaw {yaw} ({label}) published after the source withheld it')


def test_state_after_a_usb_drop_is_NaN_not_the_last_value():
    node = _StubNode(_fc_with_attitude(10.0))
    att = node.fc.get_attitude()
    yaw, _ = amn.AUVManagerNode._effective_yaw_deg(node, att)
    amn.AUVManagerNode._publish_state(
        node, att, {'voltage': 14.7}, 'STABILIZE', True, yaw)
    (msg,) = node.state_publisher.msgs
    assert math.isnan(msg.yaw_deg), f'frozen yaw {msg.yaw_deg} published'
    assert math.isnan(msg.depth_m), f'frozen depth {msg.depth_m} published'


def test_a_stale_depth_is_absent_even_beside_a_fresh_attitude():
    """Cloud audit, 2026-11-07. ATTITUDE was age-gated and VFR_HUD was not, so
    a depth stream that stopped kept publishing its last value -- a confident
    -1.7 m from a board that had said nothing for a minute -- as long as the
    IMU stream lived. INJECTION-VERIFIED: read VFR_HUD through `_cache`
    instead of `_fresh` and this fails."""
    fc = _fc_with_attitude(age_s=0.0)
    fc.master.messages['VFR_HUD'] = SimpleNamespace(
        _timestamp=time.time() - 60.0, alt=-1.7)
    att = fc.get_attitude()
    assert att is not None and math.isnan(att['depth'])
    assert math.isnan(fc.telemetry().depth_m)


def test_a_stale_heartbeat_is_an_unknown_mode():
    fc = _fc_with_attitude(age_s=0.0)
    fc.master.messages['HEARTBEAT'] = SimpleNamespace(
        base_mode=0, autopilot=0, custom_mode=0, _timestamp=time.time() - 60.0)
    assert fc.get_mode() == 'UNKNOWN'


def test_the_demand_says_heave_is_unknown_not_zero():
    """The board reports surge and sway demand only. The message's z was left
    at its default 0.0, and Batighor plotted it as a heave demand of exactly
    zero: a confident number nothing measured."""
    from geometry_msgs.msg import Vector3Stamped
    from mongla_manager.node_telemetry import TelemetryMixin
    for d in ((0.4, -0.2), None):
        pub = _Pub()
        stub = SimpleNamespace(
            fc=SimpleNamespace(demand=lambda d=d: d), demand_publisher=pub,
            _Vector3Stamped=Vector3Stamped,
            get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=Time)))
        TelemetryMixin._publish_demand(stub)
        v = pub.msgs[-1].vector
        if d is None:
            assert math.isnan(v.x) and math.isnan(v.y)
        else:
            assert (v.x, v.y) == pytest.approx(d)
        assert math.isnan(v.z)


def test_the_thruster_pack_is_on_the_state_and_absent_is_nan():
    """PM2, the pack the thrusters draw, reached only the console; it is
    /mongla/thruster_pack (MonglaState kept its shape, so old bags decode). Batighor's
    Thrust view plotted PM1 (the electronics rail) as 'what it cost'."""
    node = _StubNode(_fc_with_attitude(0.0))
    att = node.fc.get_attitude()
    amn.AUVManagerNode._publish_state(node, att, None, 'MANUAL', False, 0.0)
    pack = node.thruster_pack_publisher.msgs[-1]
    assert math.isnan(pack.voltage) and not pack.present
    node.fc.note_battery(SimpleNamespace(id=1, voltages=[14630], current_battery=-1))
    amn.AUVManagerNode._publish_state(node, att, None, 'MANUAL', False, 0.0)
    pack = node.thruster_pack_publisher.msgs[-1]
    assert abs(pack.voltage - 14.63) < 1e-3 and pack.present


def test_the_safety_states_go_out_with_the_state_and_unheard_is_unknown():
    """Leak, kill and the barometer ride /mongla/safety under the state's own
    header (MonglaState keeps its shape). A board that has said nothing about
    them reads UNKNOWN, never clear."""
    from mongla_interfaces.msg import MonglaSafety
    node = _StubNode(_fc_with_attitude(0.0))
    amn.AUVManagerNode._publish_state(node, node.fc.get_attitude(), None, 'MANUAL', False, 0.0)
    (state,), (safety,) = node.state_publisher.msgs, node.safety_publisher.msgs
    assert safety.header == state.header
    u = MonglaSafety.SAFETY_UNKNOWN
    assert (safety.leak, safety.kill_switch, safety.baro, safety.baro_text) == (u, u, u, '')
