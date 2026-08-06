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
    # Guard the PARSE before trusting it. Position-as-value only holds while the
    # firmware assigns PH_IDLE = 0 and lets the rest increment implicitly; an
    # explicit value or a gap would shift everything silently. And a regex that
    # matched only the first two entries would sail through the loop below having
    # checked almost nothing -- which is the same "green test, changed behaviour"
    # failure this whole file exists to prevent.
    assert len(names) == 8, f'expected 8 movement phases, parsed {len(names)}: {names}'
    assert '=' not in m.group(1).split(',', 1)[1], (
        'a later phase now carries an explicit value -- position is no longer the '
        'value, so this test must parse name=value pairs instead')
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


# --------------------------------------------------------------------------- #
#  MAV_RESULT values -- pinned against the firmware's OWN vendored enum         #
# --------------------------------------------------------------------------- #
# Added 2026-08-03 after `ACK_TEMPORARILY_REJECTED` was found to be 3, which is
# MAV_RESULT_UNSUPPORTED. Nothing caught it because this file pinned command ids
# and channel constants but never the RESULT codes -- and a wrong result code is
# the quietest possible bug: the board answers correctly, we misread the answer.
#
# The two directions it broke, neither visible in a log:
#   * a real mutex miss (wire 1) was not in TERMINAL_ACKS, so the move ACK loop
#     polled to the end of its budget and reported a bogus TIMEOUT + braked; and
#   * a genuine UNSUPPORTED (wire 3) was reported as "board busy, safe to retry",
#     advice that can never come true.

def test_mav_result_values_match_the_firmware_enum():
    """Parse `MAV_RESULT_*=<n>` out of the firmware's vendored common.h."""
    common = _FW / 'lib' / 'mavlink' / 'common' / 'common.h'
    if not common.is_file():
        pytest.skip('vendored common.h not present in the firmware checkout')
    text = common.read_text(errors='ignore')
    fw = {m.group(1): int(m.group(2))
          for m in re.finditer(r'MAV_RESULT_([A-Z_]+)\s*=\s*(\d+)', text)}
    assert fw, 'could not parse any MAV_RESULT_* from common.h'

    ours = {
        'ACCEPTED':             sp.ACK_ACCEPTED,
        'TEMPORARILY_REJECTED': sp.ACK_TEMPORARILY_REJECTED,
        'DENIED':               sp.ACK_DENIED,
        'UNSUPPORTED':          sp.ACK_UNSUPPORTED,
        'FAILED':               sp.ACK_FAILED,
        'IN_PROGRESS':          sp.ACK_IN_PROGRESS,
        'CANCELLED':            sp.ACK_CANCELLED,
    }
    for name, mine in ours.items():
        if name in fw:
            assert mine == fw[name], (
                f'ACK_{name} is {mine} here but MAV_RESULT_{name}={fw[name]} in the '
                f'firmware. A wrong result code misreads a correct answer.')


def test_the_ack_codes_are_distinct():
    """The bug was two names sharing value 3. Their CALLER ADVICE is opposite --
    TEMPORARILY_REJECTED means retry, UNSUPPORTED means never retry -- so a
    collision here is not cosmetic."""
    codes = [sp.ACK_ACCEPTED, sp.ACK_TEMPORARILY_REJECTED, sp.ACK_DENIED,
             sp.ACK_UNSUPPORTED, sp.ACK_FAILED, sp.ACK_IN_PROGRESS,
             sp.ACK_CANCELLED]
    assert len(set(codes)) == len(codes), f'duplicate ACK code: {codes}'


def test_in_progress_is_the_only_non_terminal_ack():
    assert sp.ACK_IN_PROGRESS not in sp.TERMINAL_ACKS
    for code in (sp.ACK_ACCEPTED, sp.ACK_DENIED, sp.ACK_FAILED, sp.ACK_CANCELLED,
                 sp.ACK_TEMPORARILY_REJECTED, sp.ACK_UNSUPPORTED):
        assert code in sp.TERMINAL_ACKS


# --------------------------------------------------------------------------- #
#  The +1 offset ITSELF -- the gap the ordering test above cannot see           #
# --------------------------------------------------------------------------- #

def test_the_wire_to_movement_type_offset_is_still_plus_one():
    """`test_move_type_codes_match_the_firmware_enum` asserts the ENUM ORDERING in
    movement.h. It says nothing about the line that applies the offset.

    So if `c.mv_type = (uint8_t)(wire + 1)` in mav_commands.cpp ever became
    `(uint8_t)wire`, or `wire + 2`, that test stays GREEN and every motion verb
    shifts by one -- `forward` becomes `back`, `stop` becomes `hold`. The enum and
    the offset are two independent facts and only one of them was pinned.
    """
    src = _read('src', 'comms', 'mav_commands.cpp')
    m = re.search(r'mv_type\s*=\s*\(\s*uint8_t\s*\)\s*\(\s*wire\s*\+\s*(\d+)\s*\)', src)
    assert m, ('could not find `mv_type = (uint8_t)(wire + N)` in mav_commands.cpp -- '
               'either it was reformatted (update this regex) or the offset moved '
               '(update srot_protocol AND every MOVE_* consumer)')
    assert int(m.group(1)) == 1, (
        f'the wire->movement::Type offset is now +{m.group(1)}, not +1. Every SROT_MOVE '
        f'verb this host sends is off by {int(m.group(1)) - 1}.')


def test_the_firmware_still_rejects_p1_outside_our_valid_range():
    """VALID_MOVE_TYPES must match the firmware's dispatch bound, or we either send
    something it DENIEs (annoying) or refuse something it would accept (a verb we
    cannot reach)."""
    src = _read('src', 'comms', 'mav_commands.cpp')
    m = re.search(r'wire\s*<\s*0\s*\|\|\s*wire\s*>\s*(\d+)', src)
    assert m, 'could not find the SROT_MOVE p1 range check in mav_commands.cpp'
    assert int(m.group(1)) == max(sp.VALID_MOVE_TYPES), (
        f'firmware accepts p1 up to {m.group(1)}, we allow up to '
        f'{max(sp.VALID_MOVE_TYPES)}')


# --------------------------------------------------------------------------- #
#  Which modes DO_SET_MODE will actually accept                                 #
# --------------------------------------------------------------------------- #

def test_mode_not_settable_matches_the_firmware_switch():
    """`MODE_NAMES` says a mode EXISTS; `MODE_NOT_SETTABLE` says we may not ask for
    it. Only the second is about DO_SET_MODE, and nothing pinned it.

    Both directions are failures: asking for a mode the board refuses is a DENIED
    with no explanation, and refusing one it would accept is a verb we can never
    reach from a mission.
    """
    src = _read('src', 'comms', 'mav_commands.cpp')
    m = re.search(r'isSupportedMode[^{]*\{(.*?)\n\}', src, re.S)
    assert m, 'isSupportedMode() not found in mav_commands.cpp'
    accepted = set(re.findall(r'case\s+\(?uint8_t\)?\s*\)?\s*FlightMode::(\w+)', m.group(1)))
    if not accepted:                       # tolerate a different case-label spelling
        accepted = set(re.findall(r'FlightMode::(\w+)', m.group(1)))
    assert accepted, 'parsed no mode labels out of isSupportedMode()'

    name_to_int = {v: k for k, v in sp.MODE_NAMES.items()}
    for fw_name in accepted:
        mode_int = name_to_int.get(fw_name)
        if mode_int is None:
            continue                       # covered by test_every_firmware_flight_mode_has_a_name
        assert mode_int not in sp.MODE_NOT_SETTABLE, (
            f'{fw_name} is in MODE_NOT_SETTABLE but the firmware ACCEPTS it -- '
            f'we are refusing a mode we could enter')
    for mode_int in sp.MODE_NOT_SETTABLE:
        fw_name = sp.MODE_NAMES.get(mode_int)
        if fw_name:
            assert fw_name not in accepted, (
                f'{fw_name} is settable on the board but we list it NOT_SETTABLE')


# --------------------------------------------------------------------------- #
#  Command ids other than 31000                                                 #
# --------------------------------------------------------------------------- #

def test_the_other_command_ids_are_still_what_we_send():
    """Only SROT_MOVE was pinned. A wrong id is answered UNSUPPORTED at best; at
    worst it is a DIFFERENT implemented command."""
    src = _read('src', 'comms', 'mav_commands.cpp')
    for name, value in (('MAV_CMD_USER_1', sp.CMD_USER_1),
                        ('MAV_CMD_USER_2', sp.CMD_USER_2),
                        ('MAV_CMD_USER_3', sp.CMD_USER_3),
                        ('MAV_CMD_USER_4', sp.CMD_USER_4),
                        ('MAV_CMD_USER_5', sp.CMD_USER_5)):
        assert name in src, f'{name} no longer dispatched by the firmware'
    common = _read('lib', 'mavlink', 'common', 'common.h')
    for enum_name, ours in (('MAV_CMD_DO_SET_SERVO', sp.CMD_DO_SET_SERVO),
                            ('MAV_CMD_DO_SET_RELAY', sp.CMD_DO_SET_RELAY)):
        m = re.search(rf'{enum_name}\s*=\s*(\d+)', common)
        if m:
            assert int(m.group(1)) == ours, f'{enum_name} is {m.group(1)}, we send {ours}'


# --------------------------------------------------------------------------- #
#  NAMED_VALUE_FLOAT names                                                      #
# --------------------------------------------------------------------------- #

def test_the_named_values_we_read_are_still_emitted():
    """A renamed NAMED_VALUE_FLOAT is COMPLETELY silent: `_named_value()` returns
    None forever and `telemetry()` reports the sensor as absent. LEAK is in this
    list -- "no leak reported" and "we stopped listening for leaks" look identical.

    ⚠ This asserts on SOURCE TEXT, which this file's preamble argues against. The
    exception is deliberate and narrow: for message NAMES there is no runtime value
    to compare -- a rename IS a source-text change, and grep is the only instrument
    that can see it. It does not assert the value is ever SENT (DEPTH_ERR/DEPTH_OUT
    sit inside a conditional), only that the name still exists to be sent.
    """
    src = _read('src', 'comms', 'mav_stream.cpp')
    for name in ('LEAK', 'WTEMP', 'GAIN', 'KILL', 'MAGACC', 'DEPTH_ERR', 'DEPTH_OUT'):
        assert re.search(rf'"{name}"', src), (
            f'the firmware no longer emits NAMED_VALUE_FLOAT "{name}", but srot_fc '
            f'still reads it -- that sensor will silently read as absent forever')


# --------------------------------------------------------------------------- #
#  The GCS-failsafe identity coupling                                           #
# --------------------------------------------------------------------------- #

def test_our_source_ids_match_the_firmware_failsafe_defaults():
    """FS_GCS_SYSID/FS_GCS_COMPID scope the board's GCS failsafe to US specifically.
    If they and our SOURCE_* diverge, the board stops counting our heartbeats and
    SURFACES mid-mission -- which on the deck looks exactly like a link fault, not
    like a config mismatch."""
    src = _read('src', 'comms', 'params.cpp')
    for pname, ours in (('FS_GCS_SYSID', sp.SOURCE_SYSID),
                        ('FS_GCS_COMPID', sp.SOURCE_COMPID)):
        m = re.search(rf'"{pname}"[^}}]*?(\d+)\.0f', src)
        if m is None:
            continue                       # spelled differently; the grep is the guard
        assert int(m.group(1)) == ours, (
            f'firmware {pname} defaults to {m.group(1)} but we present as {ours}; '
            f'the board will not recognise our heartbeat and will surface')


# --------------------------------------------------------------------------- #
#  SERVOn_FUNCTION -- the payload identity enum, mirrored in THREE repos         #
# --------------------------------------------------------------------------- #

_FUNCS = (('NONE', 'PCA_FUNC_NONE'), ('TORPEDO', 'PCA_FUNC_TORPEDO'),
          ('DROPPER', 'PCA_FUNC_DROPPER'), ('GRIPPER', 'PCA_FUNC_GRIPPER'),
          ('LIGHT', 'PCA_FUNC_LIGHT'), ('CAMERA', 'PCA_FUNC_CAMERA'),
          ('AUX', 'PCA_FUNC_AUX'))


def test_servo_function_enum_matches_the_firmware():
    cfg = _read('include', 'config.h')
    for fw_suffix, host_attr in _FUNCS:
        fw = _define(cfg, f'SROT_SERVO_FUNC_{fw_suffix}')
        assert fw is not None, f'SROT_SERVO_FUNC_{fw_suffix} missing from config.h'
        assert fw == getattr(sp, host_attr), (
            f'SROT_SERVO_FUNC_{fw_suffix}={fw} but {host_attr}={getattr(sp, host_attr)} '
            f'-- a payload would be mislabelled')


def test_the_firmware_defines_no_payload_function_we_lack():
    """The append-only guard. A firmware that gains SROT_SERVO_FUNC_SONAR while we
    do not know about it renders that channel as an unknown integer -- and worse,
    someone inserting a value in the middle renames every payload after it."""
    cfg = _read('include', 'config.h')
    fw_names = set(re.findall(r'#define\s+SROT_SERVO_FUNC_(\w+)', cfg))
    fw_names.discard('MAX')                # a bound, not a value
    ours = {n for n, _ in _FUNCS}
    assert fw_names <= ours, (
        f'firmware defines payload functions we do not mirror: {sorted(fw_names - ours)}')


def test_bondor_mirrors_the_same_payload_function_enum():
    """Bondor is the THIRD copy and the only edge with no automated check of its own
    (its gate is `npm run typecheck`, which cannot see the firmware). duburi_ws
    referees a constant it does not own, because otherwise nothing does.

    Skips cleanly when the ground-station repo is not checked out.
    """
    gs = _FW.parent / 'srot-ground-station' / 'bondor' / 'src' / 'shared' / 'protocol.ts'
    if not gs.is_file():
        pytest.skip('Bondor repo not checked out beside this workspace')
    text = gs.read_text(errors='ignore')
    if 'SERVO_FUNC' not in text:
        pytest.skip('Bondor has not adopted the payload-function enum yet')
    # Match the whole `{ ... }` entry containing the name, then pull `value` out of
    # it -- so reordering the keys (prettier, a hand-edit) cannot make a present enum
    # look missing. An order-dependent regex here would fail as "Bondor is missing
    # TORPEDO", sending someone to look for a deleted constant that is right there.
    for fw_suffix, host_attr in _FUNCS:
        entry = re.search(rf"\{{[^{{}}]*name:\s*'{fw_suffix}'[^{{}}]*\}}", text)
        assert entry, f'Bondor is missing payload function {fw_suffix}'
        m = re.search(r'value:\s*(\d+)', entry.group(0))
        assert m, f"Bondor's {fw_suffix} entry has no value: {entry.group(0)}"
        assert int(m.group(1)) == getattr(sp, host_attr), (
            f'Bondor has {fw_suffix}={m.group(1)}, we have {getattr(sp, host_attr)}')
