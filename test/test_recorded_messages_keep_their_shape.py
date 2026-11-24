"""A message the bags hold keeps its fields, or every bag before it stops decoding.

ROS 2 does not version a message: rosbag2 replays the bytes against today's
definition. Adding `thruster_voltage` to MonglaState (2026-11-22) left the
bench rest bag decoding 0 of 39534 `/mongla/state` messages on Kutum. New data
goes on its own topic (`/mongla/thruster_pack`); changing a field here means
re-recording every fixture and pool-day bag, said in the commit.
"""
import pathlib

ROOT = pathlib.Path(__file__).resolve().parents[1]

FROZEN = {
    'MonglaState.msg': ['std_msgs/Header header', 'bool armed', 'string mode',
                        'float32 yaw_deg', 'float32 depth_m', 'float32 battery_voltage'],
    # Frozen from its first bag: new safety data goes on another topic.
    'MonglaSafety.msg': ['std_msgs/Header header', 'int8 SAFETY_UNKNOWN', 'int8 SAFETY_CLEAR',
                         'int8 SAFETY_TRIPPED', 'int8 leak', 'int8 kill_switch', 'int8 baro',
                         'string baro_text'],
}


def _fields(path):
    out = []
    for line in path.read_text().splitlines():
        line = line.split('#', 1)[0].strip()
        if line:
            out.append(' '.join(line.split()[:2]))
    return out


def test_recorded_messages_keep_their_fields():
    for name, fields in FROZEN.items():
        got = _fields(ROOT / 'src' / 'mongla_interfaces' / 'msg' / name)
        assert got == fields, f'{name} changed shape: every bag recorded before it stops decoding'
