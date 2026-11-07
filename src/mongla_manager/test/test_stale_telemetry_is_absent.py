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
