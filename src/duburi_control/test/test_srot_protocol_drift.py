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


def test_every_firmware_flight_mode_has_a_name():
    """The other direction: we must not be MISSING a mode the board can report.

    The check above compares the modes we already declare -- so it stays green when
    the firmware ADDS one, which is exactly how STUNT (100) and PATTERN (101) sat
    unmapped. An unmapped custom_mode surfaces on /duburi/state as 'UNKNOWN(100)',
    which reads like a comms fault rather than a real mode the hull is flying.

    Asserting on the firmware enum as a SET, not on the entries we happened to
    write down, is what makes this catch an addition instead of only a change.
    """
    hdr = _read('include', 'state_types.h')
    m = re.search(r'enum\s+class\s+FlightMode\s*[^{]*\{([^}]*)\}', hdr, re.S)
    assert m, 'FlightMode enum not found'
    fw = {name: int(value)
          for name, value in re.findall(r'(\w+)\s*=\s*(\d+)', m.group(1))}
    missing = {name: value for name, value in fw.items()
               if value not in sp.MODE_NAMES}
    assert not missing, (
        f'firmware FlightMode values with no MODE_NAMES entry: {missing}. '
        f'Add them to srot_protocol.MODE_NAMES -- and to MODE_NOT_SETTABLE too if '
        f'DO_SET_MODE cannot select them.')


def test_movement_phase_codes_match_the_firmware_enum():
    """MV_STATE: 6 is HOLD and 7 is DONE, not 6 = done.

    Both this repo and JETSON_COMMS.md stopped at `6: done` for a while, so a
    station-keeping HOLD leg was reported as finished while the vehicle was still
    holding. Harmless for completion (the terminal ACK is the authority) which is
    precisely why it survived unnoticed -- it only ever showed up in a log line.
    """
    src = _read('src', 'control', 'movement.cpp')
    m = re.search(r'enum\s*\{\s*(PH_IDLE[^}]*)\}', src)
    assert m, 'movement phase enum not found in movement.cpp'
    # `enum { PH_IDLE = 0, PH_CRUISE, PH_BRAKE, ... }` -- implicit increment after
    # the first, so position IS the value.
    names = [p.strip().split('=')[0].strip() for p in m.group(1).split(',') if p.strip()]
    expected = {
        'PH_IDLE': sp.MV_IDLE,   'PH_CRUISE': sp.MV_CRUISE, 'PH_BRAKE': sp.MV_BRAKE,
        'PH_TURN': sp.MV_TURN,   'PH_DIVE': sp.MV_DIVE,     'PH_STYLE': sp.MV_STYLE,
        'PH_HOLD': sp.MV_HOLD,   'PH_DONE': sp.MV_DONE,
    }
    for index, fw_name in enumerate(names):
        assert expected.get(fw_name) == index, (
            f'firmware {fw_name} == {index}, srot_protocol has {expected.get(fw_name)}')
    assert sp.mv_state_name(6) == 'hold' and sp.mv_state_name(7) == 'done'


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


def test_firmware_behaviour_rev_is_new_enough():
    """The board must be at or above the behaviour revision this host assumes.

    Replaces `test_stop_still_does_not_apply_reverse_thrust`, which asserted on the
    SHAPE of the firmware's C++ -- it grepped the `Type::STOP` case for `abort()`
    to decide whether MOVE_STOP still coasted.

    That failed silently in the way that matters. The firmware fixed MOVE_STOP by
    restoring the outgoing leg's axis and speed (which `start()` had zeroed before
    the switch), so the string never appeared, the test stayed GREEN, and the
    behaviour changed underneath it. Had we trusted it we would have kept
    `_brake_last_leg` and braked the hull twice. Their own suggested fix -- routing
    STOP through `abort()` -- would not have worked either, for the same reason:
    `abort()` does not restore those fields.

    Source text is a brittle proxy for behaviour. `SROT_FW_BEHAVIOUR_REV` is a
    number the board bumps deliberately, with each revision's meaning documented
    beside it in `include/config.h`.
    """
    src = _read('include', 'config.h')
    m = re.search(r'#define\s+SROT_FW_BEHAVIOUR_REV\s+(\d+)', src)
    assert m, (
        'SROT_FW_BEHAVIOUR_REV not found in include/config.h -- firmware predates '
        'the behaviour-revision signal (added 2026-08-01). Anything older than '
        'rev 2 COASTS on MOVE_STOP and we no longer carry a host brake.')
    rev = int(m.group(1))
    assert rev >= sp.FW_BEHAVIOUR_REV_REQUIRED, (
        f'firmware behaviour rev {rev} < required {sp.FW_BEHAVIOUR_REV_REQUIRED}. '
        f'Rev 1 coasts on MOVE_STOP and strands a move on IN_PROGRESS when a '
        f'failsafe displaces AUTO. Either flash rev >= '
        f'{sp.FW_BEHAVIOUR_REV_REQUIRED} or restore the host-side workarounds.')


def test_esc_status_291_is_absent_from_our_dialect():
    """Pins the reason /duburi/esc_rpm cannot be built on ESC_STATUS today.

    pymavlink discards any msgid missing from its CRC-extra table SILENTLY. Upstream
    removed the WIP messages 290/291 from `common`, so the board's ESC_STATUS never
    reaches `master.messages` -- zero RPM that looks exactly like an ESC/wiring fault.
    Bondor hit this and lost every RPM packet (their connection.ts:40-51).

    If this test starts FAILING, pymavlink gained the message: delete the
    ESC_TELEMETRY fallback in SrotFC.telemetry() and read 291 directly.

    ASSERTED AGAINST THE RUNTIME DIALECT, not against `dialects.v20.ardupilotmega`.
    This test used to import the v20 module directly and check ITS map -- which is a
    different object from the one the vehicle decodes with. pymavlink binds ONE dialect
    at import time from the environment, and the DEFAULT is `dialects.v10.ardupilotmega`
    (MAVLink 1), where 11030/11031 do not exist. So the old assertion passed while a
    default-configured connection decoded nothing: green test, zero RPM.

    Measured on the board in-vehicle: with the default dialect the ESC frames arrive and
    are reported as UNKNOWN_291 / UNKNOWN_11030 / UNKNOWN_11031. Importing `srot_fc`
    (which sets MAVLINK20 before importing pymavlink, as pixhawk.py does) is what makes
    the map below the real one. Test what runs.
    """
    from duburi_control.fc import srot_fc            # noqa: F401  -- sets MAVLINK20
    from pymavlink import mavutil
    runtime_map = mavutil.mavlink.mavlink_map

    assert mavutil.mavlink.__name__.startswith('pymavlink.dialects.v20'), (
        f'runtime dialect is {mavutil.mavlink.__name__}, not v20 -- MAVLINK20 was not '
        f'set before pymavlink was imported, and the ESC telemetry will silently vanish')
    assert 291 not in runtime_map, (
        'ESC_STATUS (291) is now in the dialect -- simplify SrotFC.telemetry()')
    # The fallback we rely on instead must exist, or there is no RPM path at all.
    assert 11030 in runtime_map and 11031 in runtime_map, \
        'ESC_TELEMETRY_1_TO_4 / 5_TO_8 missing -- no usable RPM message remains'
