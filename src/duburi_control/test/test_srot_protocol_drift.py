"""Drift guard: our pinned SROT wire constants vs the firmware's own headers.

The SROT protocol is hand-mirrored in FOUR places -- the firmware (`include/config.h`,
`src/comms/mav_commands.cpp`), the LoRa struct (`shared/lora_telem_proto.h`), Bondor's
`bondor/src/shared/protocol.ts`, and this workspace's `fc/srot_protocol.py`. Keeping
them in sync by hand is a deliberate choice (three separate repos, three languages);
what is NOT acceptable is drift going unnoticed.

The failure mode is why this file exists: a mismatched constant raises nothing. A wrong
move-type code sends a valid command that does the wrong thing; a wrong mode int gets
DENIED with no clue why; a wrong `PCA_RELAY_BASE_CH` fires the wrong payload channel.
There is no exception to catch -- only a vehicle behaving oddly in the water.

Skips cleanly when the sibling repo is not checked out (CI, a fresh clone, the Jetson),
so this can never block a build. It is a canary, not a dependency.
"""

import re
from pathlib import Path

import pytest

from duburi_control.fc import srot_protocol as sp

# The firmware repo lives under Mongla_others/, a sibling of Ros_workspaces/.
# Walk up rather than counting parents, so this survives the workspace being
# moved or nested differently on the Jetson. SROT_FW_DIR overrides.
def _find_firmware() -> Path | None:
    import os
    override = os.environ.get('SROT_FW_DIR')
    if override:
        return Path(override)
    for parent in Path(__file__).resolve().parents:
        candidate = parent / 'Mongla_others' / 'srot-control-board'
        if (candidate / 'include' / 'config.h').is_file():
            return candidate
    return None


_FW = _find_firmware()

pytestmark = pytest.mark.skipif(
    _FW is None,
    reason='SROT firmware repo not checked out beside this workspace')


def _read(*parts) -> str:
    return (_FW.joinpath(*parts)).read_text(errors='ignore')


def _define(text: str, name: str):
    """Value of a `#define NAME <int>` (ignores trailing comments/casts)."""
    m = re.search(rf'^\s*#define\s+{re.escape(name)}\s+\(?(\d+)', text, re.M)
    return int(m.group(1)) if m else None


def test_srot_move_command_id():
    """31000 is the one custom verb; everything the vehicle does routes through it."""
    src = _read('src', 'comms', 'mav_commands.cpp')
    assert re.search(r'MAV_CMD_SROT_MOVE\s*=?\s*31000|case\s+31000', src) or \
        '31000' in src, 'MAV_CMD_SROT_MOVE 31000 not found in the firmware dispatch'
    assert sp.CMD_SROT_MOVE == 31000


def test_move_type_codes_match_the_firmware_enum():
    """p1 is a 0-based WIRE code; the firmware stores movement::Type = p1 + 1.

    If that +1 offset ever changes, every single motion verb silently becomes a
    different manoeuvre -- `forward` would strafe. This is the highest-consequence
    constant in the whole protocol.
    """
    hdr = _read('src', 'control', 'movement.h')
    m = re.search(r'enum\s+class\s+Type\s*[^{]*\{([^}]*)\}', hdr, re.S)
    assert m, 'movement::Type enum not found'
    names = [n.strip().split('=')[0].strip()
             for n in m.group(1).split(',') if n.strip()]
    # NONE=0, then FWD, BACK, LEFT, RIGHT, TURN, DIVE, STOP, HOLD, STYLE, ARC
    assert names[0] == 'NONE', f'movement::Type no longer starts at NONE: {names}'
    order = names[1:]
    ours = [
        (sp.MOVE_FORWARD,  'FWD'),   (sp.MOVE_BACK,     'BACK'),
        (sp.MOVE_STRAFE_L, 'LEFT'),  (sp.MOVE_STRAFE_R, 'RIGHT'),
        (sp.MOVE_TURN,     'TURN'),  (sp.MOVE_DIVE,     'DIVE'),
        (sp.MOVE_STOP,     'STOP'),  (sp.MOVE_HOLD,     'HOLD'),
        (sp.MOVE_STYLE,    'STYLE'), (sp.MOVE_ARC,      'ARC'),
    ]
    for wire_code, fw_name in ours:
        assert order[wire_code] == fw_name, (
            f'wire p1={wire_code} maps to movement::Type {order[wire_code]}, '
            f'but srot_protocol calls it {fw_name}')


def test_flight_mode_ints_match():
    """A wrong mode int is a silent DENIED, or worse, the WRONG mode engaged."""
    hdr = _read('include', 'state_types.h')
    m = re.search(r'enum\s+class\s+FlightMode\s*[^{]*\{([^}]*)\}', hdr, re.S)
    assert m, 'FlightMode enum not found'
    # Scan name=value pairs directly: the entries carry // comments (which contain
    # commas), so splitting on ',' loses everything after the first one.
    fw = {name: int(value)
          for name, value in re.findall(r'(\w+)\s*=\s*(\d+)', m.group(1))}
    for name, value in (('STABILIZE', sp.MODE_STABILIZE), ('ACRO', sp.MODE_ACRO),
                        ('DEPTH_HOLD', sp.MODE_DEPTH_HOLD), ('SURFACE', sp.MODE_SURFACE),
                        ('MANUAL', sp.MODE_MANUAL), ('AUTO', sp.MODE_AUTO)):
        assert fw.get(name) == value, (
            f'FlightMode::{name} is {fw.get(name)} on the board, {value} here')


def test_payload_and_link_constants_match():
    cfg = _read('include', 'config.h')
    # Relay instance n -> PCA channel PCA_RELAY_BASE_CH + n. Off by one = wrong payload.
    assert _define(cfg, 'PCA_RELAY_BASE_CH') == sp.PCA_RELAY_BASE_CH
    assert _define(cfg, 'PCA9685_NUM_CH') == sp.PCA9685_NUM_CH
    # Baud: a mismatch here is a link that never opens (loud, but cheap to catch).
    assert _define(cfg, 'MAVLINK_BAUD') == sp.BAUD
    # GCS failsafe window -- the board surfaces after this much heartbeat silence.
    assert _define(cfg, 'GCS_FAILSAFE_MS') == sp.GCS_FAILSAFE_MS


def test_vehicle_ids_match():
    cfg = _read('include', 'config.h')
    assert _define(cfg, 'MAV_SYSTEM_ID') == sp.VEHICLE_SYSID
    assert _define(cfg, 'MAV_COMPONENT_ID') in (None, sp.VEHICLE_COMPID)


def test_stop_still_does_not_apply_reverse_thrust():
    """Guards our host-side brake against becoming a DOUBLE brake.

    `SrotFC._brake_last_leg` exists purely because the wire-reachable MOVE_STOP
    coasts: movement::start() zeroes s_uf/s_ul/s_speed before the STOP case, so
    PH_BRAKE computes `-0 * gain * 0`. If the firmware ever fixes that (it is item
    2 on our JETSON_FEEDBACK list), this test fails and tells us to drop the host
    brake instead of kicking the hull twice.
    """
    src = _read('src', 'control', 'movement.cpp')
    m = re.search(r'case\s+Type::STOP\s*:(.*?)(?=case\s+Type::|\n\s*\})', src, re.S)
    assert m, 'Type::STOP case not found in movement::start()'
    body = m.group(1)
    assert 'abort()' not in body, (
        'MOVE_STOP now routes through movement::abort(), which brakes correctly '
        'on-board -- remove SrotFC._brake_last_leg or it will brake twice.')


def test_esc_status_291_is_absent_from_our_dialect():
    """Pins the reason /duburi/esc_rpm cannot be built on ESC_STATUS today.

    pymavlink discards any msgid missing from its CRC-extra table SILENTLY. Upstream
    removed the WIP messages 290/291 from `common`, so the board's ESC_STATUS never
    reaches `master.messages` -- zero RPM that looks exactly like an ESC/wiring fault.
    Bondor hit this and lost every RPM packet (their connection.ts:40-51).

    If this test starts FAILING, pymavlink gained the message: delete the
    ESC_TELEMETRY fallback in SrotFC.telemetry() and read 291 directly.
    """
    from pymavlink.dialects.v20 import ardupilotmega as dialect
    assert 291 not in dialect.mavlink_map, (
        'ESC_STATUS (291) is now in the dialect -- simplify SrotFC.telemetry()')
    # The fallback we rely on instead must exist, or there is no RPM path at all.
    assert 11030 in dialect.mavlink_map and 11031 in dialect.mavlink_map, \
        'ESC_TELEMETRY_1_TO_4 / 5_TO_8 missing -- no usable RPM message remains'
