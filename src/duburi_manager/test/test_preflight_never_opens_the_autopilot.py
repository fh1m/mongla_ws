"""B47 -- the preflight must not reboot the flight controller.

Two independent facts collide on this hull:

  * The SROT board is a CH340 and enumerates as **1a86:7523** -- byte for byte
    the payload DevKit's VID/PID. Measured on the vehicle, it is the ONLY such
    device present.
  * Opening that serial port REBOOTS the board (`fc/port_guard.py` carries the
    measurements; it is why PortGuard exists at all).

`bringup_check` section I auto-detected the payload board by VID/PID with an
explicitly EMPTY exclude set, then OPENED what it found to "verify the serial
link". On an SROT hull that is the autopilot. The tool the operator is told to
run "at the start of every session" rebooted the flight controller and reported
PASS on it.

It got there because its backend default was hand-typed: `srot = '--srot' in
argv` stayed opt-IN long after `flight_controller` defaulted to srot, so the
bare command took the pixhawk branch on an srot vehicle. Same class as B46 --
two tables describing one vehicle, and nothing comparing them.

Both halves are pinned here: the default must come from the shared constant,
and the payload probe must never be handed the flight-controller link even when
someone deliberately asks for the pixhawk path.
"""
import os
import re
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_WS   = os.path.abspath(os.path.join(_HERE, '..', '..', '..'))
for _p in ('duburi_manager', 'duburi_control'):
    _s = os.path.join(_WS, 'src', _p)
    if _s not in sys.path:
        sys.path.insert(0, _s)

_CHECK = os.path.join(_WS, 'src', 'duburi_manager',
                      'duburi_manager', 'bringup_check.py')


def test_the_preflight_backend_is_not_a_hand_typed_default():
    """The bare command must follow the stack, not a literal someone typed."""
    src = open(_CHECK).read()
    assert "srot = '--srot' in argv" not in src, (
        "bringup_check's backend is opt-in again -- the bare command will take "
        "the pixhawk branch on an srot hull and section I will open the autopilot")
    assert 'DEFAULT_FLIGHT_CONTROLLER' in src, (
        'bringup_check must read the shared backend constant')


def test_the_shared_constant_is_the_one_the_manager_declares():
    """One value, two readers -- the property B46 exists to keep."""
    from duburi_manager.connection_config import DEFAULT_FLIGHT_CONTROLLER
    node = open(os.path.join(_WS, 'src', 'duburi_manager', 'duburi_manager',
                             'auv_manager_node.py')).read()
    assert "declare_parameter('flight_controller', DEFAULT_FLIGHT_CONTROLLER)" in node, (
        'the manager re-typed its backend default instead of using the constant')
    assert DEFAULT_FLIGHT_CONTROLLER in ('srot', 'pixhawk')


def test_both_overrides_are_still_honoured():
    """A default is not a lock: --srot forces on, --pixhawk forces off."""
    src = open(_CHECK).read()
    assert "if '--srot' in argv" in src
    assert "elif '--pixhawk' in argv" in src


def test_the_payload_probe_is_never_handed_an_empty_exclude():
    """The exact line that opened the autopilot."""
    src = open(_CHECK).read()
    assert 'auto_detect_port(exclude=set())' not in src, (
        'the payload probe is scanning with an empty exclude again -- on this '
        'hull that returns the SROT board and the verify step opens it')


def test_the_flight_controller_link_is_excluded_from_the_payload_scan():
    """Behavioural, not textual: run the real matcher over a faked USB bus.

    The board and the payload share a VID/PID, so the ONLY thing that can tell
    them apart here is the exclude set.
    """
    from duburi_control.payload import PayloadDriver

    class _Info:
        def __init__(self, device, vid, pid):
            self.device, self.vid, self.pid = device, vid, pid

    import serial.tools.list_ports as lp
    fc = '/dev/ttyUSB0'                       # the SROT board, as measured
    saved = lp.comports
    lp.comports = lambda: [_Info(fc, 0x1a86, 0x7523)]
    try:
        # what the old code did
        assert PayloadDriver.auto_detect_port(exclude=set()) == fc
        # what the fixed code does
        assert PayloadDriver.auto_detect_port(exclude={fc}) is None
    finally:
        lp.comports = saved


def test_the_exclude_survives_a_by_id_symlink():
    """The FC is resolved by its stable by-id name; the bus reports /dev/ttyUSB*.

    `auto_detect_port` realpaths both sides. If that ever stops, the exclude
    silently stops matching and the probe opens the autopilot again -- with the
    code still *looking* correct.
    """
    from duburi_control.payload import PayloadDriver
    src = open(os.path.join(_WS, 'src', 'duburi_control', 'duburi_control',
                            'payload.py')).read()
    assert 'os.path.realpath' in src, (
        'auto_detect_port no longer canonicalises paths -- a by-id exclude will '
        'not match a /dev/ttyUSB device')


def test_the_srot_branch_still_skips_the_payload_section_entirely():
    """Defence in depth, not a replacement: srot must not reach section I."""
    src = open(_CHECK).read()
    m = re.search(r'\n    else:\n        section\(.I\. Payload board', src)
    assert m, 'the payload section is no longer behind the srot/else split'
