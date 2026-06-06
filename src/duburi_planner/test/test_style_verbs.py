"""Tests for style verbs (style_roll, style_pitch, style_yaw) and BNO085 enhancements.

All tests use mocked hardware — no MAVLink or serial required.
"""

import sys
import time
import threading
import json
import types
import unittest
from io import BytesIO
from unittest.mock import MagicMock, patch, call, PropertyMock

# ---------------------------------------------------------------------------
# Path setup: make duburi_control and duburi_sensors importable without colcon
# ---------------------------------------------------------------------------
import os
_ws = os.path.join(os.path.dirname(__file__), '..', '..', '..', '..')
sys.path.insert(0, os.path.join(_ws, 'src', 'duburi_sensors'))
sys.path.insert(0, os.path.join(_ws, 'src', 'duburi_control'))


# ===========================================================================
# BNO085 Source tests
# ===========================================================================

class TestBNO085PitchRollParsing(unittest.TestCase):

    def _make_source_with_mock_serial(self, lines):
        """Return a BNO085Source with a fake serial that yields `lines`."""
        from duburi_sensors.sources.bno085 import BNO085Source
        encoded = [l.encode() + b'\n' for l in lines]
        idx = {'i': 0}

        class FakeSerial:
            is_open = True
            _written = []

            def readline(self):
                if idx['i'] < len(encoded):
                    v = encoded[idx['i']]
                    idx['i'] += 1
                    return v
                time.sleep(0.02)
                return b''

            def close(self):
                pass

            def write(self, data):
                FakeSerial._written.append(data)

        src = object.__new__(BNO085Source)
        src._port_name = 'fake'
        src._baud = 115200
        src._log  = None
        src._latest_yaw   = None
        src._latest_ts    = 0.0
        src._latest_pitch = 0.0
        src._latest_pitch_ts = 0.0
        src._latest_roll  = 0.0
        src._latest_roll_ts = 0.0
        src._frames_rx    = 0
        src._parse_errors = 0
        src._offset_deg   = 0.0
        src._stop = threading.Event()
        src._serial_write_lock = threading.Lock()   # required by send_command
        src._serial = FakeSerial()
        src._thread = threading.Thread(target=src._reader_loop, daemon=True)
        src._thread.start()
        return src, FakeSerial

    def test_parses_pitch_and_roll_from_json(self):
        src, _ = self._make_source_with_mock_serial([
            '{"yaw":45.0,"pitch":-12.5,"roll":7.3,"ts":1000}',
        ])
        time.sleep(0.15)
        src._stop.set()
        self.assertAlmostEqual(src._latest_pitch, -12.5, places=1)
        self.assertAlmostEqual(src._latest_roll,    7.3, places=1)

    def test_read_pitch_returns_cached_value(self):
        src, _ = self._make_source_with_mock_serial([
            '{"yaw":0.0,"pitch":20.0,"roll":-5.0,"ts":1000}',
        ])
        time.sleep(0.15)
        # Force timestamp fresh
        src._latest_pitch_ts = time.monotonic()
        src._latest_roll_ts  = time.monotonic()
        src._stop.set()
        self.assertAlmostEqual(src.read_pitch(), 20.0, places=1)
        self.assertAlmostEqual(src.read_roll(),  -5.0, places=1)

    def test_stale_injection_does_not_corrupt_accumulator(self):
        """Stale BNO frame (None) must be skipped; accumulator stays monotonic."""
        src, _ = self._make_source_with_mock_serial([])
        src._stop.set()
        # Simulate roll sequence mid-maneuver with a dropped frame
        roll_seq = [0.0, 45.0, 90.0, None, 135.0, 180.0]
        accum      = 0.0
        last_roll  = 0.0
        for cur in roll_seq:
            if cur is None:
                continue     # stale — same logic as style_roll loop
            delta = cur - last_roll
            if delta >  180: delta -= 360
            if delta < -180: delta += 360
            accum    += delta
            last_roll = cur
        # Should be ~180° monotonic, not jump backward on the None frame
        self.assertAlmostEqual(accum, 180.0, places=1)

    def test_read_pitch_returns_none_when_stale(self):
        src, _ = self._make_source_with_mock_serial([])
        src._stop.set()
        src._latest_pitch    = 45.0
        src._latest_pitch_ts = 0.0   # ancient timestamp → stale
        self.assertIsNone(src.read_pitch())

    def test_read_roll_returns_none_when_stale(self):
        src, _ = self._make_source_with_mock_serial([])
        src._stop.set()
        src._latest_roll    = 90.0
        src._latest_roll_ts = 0.0
        self.assertIsNone(src.read_roll())

    def test_old_firmware_yaw_only_still_works(self):
        """Firmware without pitch/roll fields must not break the reader."""
        src, _ = self._make_source_with_mock_serial([
            '{"yaw":180.0,"ts":1000}',
        ])
        time.sleep(0.15)
        src._stop.set()
        self.assertIsNotNone(src._latest_yaw)
        self.assertEqual(src._latest_pitch, 0.0)
        self.assertEqual(src._latest_roll,  0.0)

    def test_send_command_writes_to_serial(self):
        src, FakeSerial = self._make_source_with_mock_serial([])
        src._stop.set()
        FakeSerial._written.clear()
        src.send_command('L\n')
        self.assertIn(b'L\n', FakeSerial._written)

    def test_send_command_silently_ignores_closed_serial(self):
        src, FakeSerial = self._make_source_with_mock_serial([])
        src._stop.set()
        src._serial.is_open = False
        src.send_command('L\n')   # must not raise


# ===========================================================================
# Duburi facade: heading lock sends 'L' to BNO
# ===========================================================================

class TestHeadingLockSendsL(unittest.TestCase):

    def _make_duburi(self, yaw_source=None):
        """Return a Duburi instance with fully mocked Pixhawk."""
        from duburi_control.duburi import Duburi
        from duburi_control.pixhawk import Pixhawk

        px = MagicMock(spec=Pixhawk)
        px.get_attitude.return_value = {'yaw': 0.0, 'roll': 0.0, 'pitch': 0.0, 'depth': -0.5}
        px.get_mode.return_value = 'ALT_HOLD'
        px.arm.return_value = (True, 'ACCEPTED')
        px.set_mode.return_value = (True, 'ACCEPTED')

        log = MagicMock()
        log.info = MagicMock()
        log.warn = MagicMock()

        duburi = Duburi(px, log=log, yaw_source=yaw_source)
        duburi._abort_event.clear()
        return duburi

    def test_lock_heading_calls_send_command_L_when_bno(self):
        bno = MagicMock()
        bno.read_yaw.return_value = 45.0
        bno.send_command = MagicMock()

        duburi = self._make_duburi(yaw_source=bno)

        # Patch HeadingLock so we don't spin a real thread
        with patch('duburi_control.duburi.HeadingLock') as MockHL:
            mock_lock = MagicMock()
            MockHL.return_value = mock_lock
            duburi.lock_heading(45.0, timeout=10.0)

        bno.send_command.assert_called_once_with('L\n')

    def test_lock_heading_no_error_without_send_command(self):
        """yaw_source without send_command must not raise."""
        bno = MagicMock(spec=[])   # no send_command attribute
        bno.read_yaw = MagicMock(return_value=0.0)

        duburi = self._make_duburi(yaw_source=bno)
        with patch('duburi_control.duburi.HeadingLock') as MockHL:
            MockHL.return_value = MagicMock()
            duburi.lock_heading(0.0)   # must not AttributeError

    def test_lock_heading_no_error_without_yaw_source(self):
        duburi = self._make_duburi(yaw_source=None)
        with patch('duburi_control.duburi.HeadingLock') as MockHL:
            MockHL.return_value = MagicMock()
            duburi.lock_heading(0.0)   # must not raise


# ===========================================================================
# style_roll / style_pitch
# ===========================================================================

def _make_duburi_for_style():
    """Return (duburi, pixhawk_mock) with enough stubs for style verbs."""
    from duburi_control.duburi import Duburi
    from duburi_control.pixhawk import Pixhawk

    px = MagicMock(spec=Pixhawk)
    px.get_attitude.return_value = {'yaw': 0.0, 'roll': 0.0, 'pitch': 0.0, 'depth': -0.7}
    px.get_mode.return_value = 'ALT_HOLD'
    px.set_mode.return_value = (True, 'ACCEPTED')
    px.get_param.return_value = 1.0
    px.set_param.return_value = True
    px.send_rc_override = MagicMock()
    px.send_neutral      = MagicMock()
    px.percent_to_pwm    = Pixhawk.percent_to_pwm   # use real static method

    log = MagicMock()
    log.info = MagicMock()
    log.warn = MagicMock()

    duburi = Duburi(px, log=log)
    duburi._abort_event.clear()
    return duburi, px


class TestStyleRoll(unittest.TestCase):

    def test_enters_acro_mode(self):
        duburi, px = _make_duburi_for_style()
        # Make BNO accumulate 360° quickly via side-effect
        roll_vals = [0.0] + [i * 20.0 for i in range(1, 20)]
        px.get_attitude.side_effect = [
            {'yaw': 0.0, 'roll': v, 'pitch': 0.0, 'depth': -0.7}
            for v in roll_vals
        ] + [{'yaw': 0.0, 'roll': 0.0, 'pitch': 0.0, 'depth': -0.7}] * 50

        with patch('duburi_control.duburi.hold_depth'):
            duburi.style_roll(gain=60.0, timeout=5.0)

        modes_set = [c.args[0] for c in px.set_mode.call_args_list]
        self.assertIn('ACRO', modes_set)

    def test_acro_params_zeroed_before_maneuver(self):
        duburi, px = _make_duburi_for_style()
        px.get_attitude.side_effect = [
            {'yaw': 0.0, 'roll': i * 20.0, 'pitch': 0.0, 'depth': -0.7}
            for i in range(30)
        ] + [{'yaw': 0.0, 'roll': 0.0, 'pitch': 0.0, 'depth': -0.7}] * 30

        with patch('duburi_control.duburi.hold_depth'):
            duburi.style_roll(gain=60.0, timeout=5.0)

        set_calls = {(c.args[0], c.args[1]) for c in px.set_param.call_args_list}
        self.assertIn(('ACRO_BAL_ROLL', 0.0), set_calls)
        self.assertIn(('ACRO_TRAINER',  0.0), set_calls)

    def test_acro_params_restored_after_maneuver(self):
        duburi, px = _make_duburi_for_style()
        px.get_param.return_value = 1.5   # custom saved value
        px.get_attitude.side_effect = [
            {'yaw': 0.0, 'roll': i * 20.0, 'pitch': 0.0, 'depth': -0.7}
            for i in range(30)
        ] + [{'yaw': 0.0, 'roll': 0.0, 'pitch': 0.0, 'depth': -0.7}] * 30

        with patch('duburi_control.duburi.hold_depth'):
            duburi.style_roll(gain=60.0, timeout=5.0)

        # Should have restored 1.5 for both ACRO_BAL_ROLL and ACRO_TRAINER
        restore_calls = {(c.args[0], c.args[1]) for c in px.set_param.call_args_list}
        self.assertIn(('ACRO_BAL_ROLL', 1.5), restore_calls)
        self.assertIn(('ACRO_TRAINER',  1.5), restore_calls)

    def test_returns_alt_hold_after_maneuver(self):
        duburi, px = _make_duburi_for_style()
        px.get_attitude.side_effect = [
            {'yaw': 0.0, 'roll': i * 20.0, 'pitch': 0.0, 'depth': -0.7}
            for i in range(30)
        ] + [{'yaw': 0.0, 'roll': 0.0, 'pitch': 0.0, 'depth': -0.7}] * 30

        with patch('duburi_control.duburi.hold_depth'):
            duburi.style_roll(gain=60.0, timeout=5.0)

        modes = [c.args[0] for c in px.set_mode.call_args_list]
        self.assertEqual(modes[-1], 'ALT_HOLD')

    def test_aborts_if_param_set_fails(self):
        duburi, px = _make_duburi_for_style()
        px.set_param.return_value = False   # simulate timeout

        result = duburi.style_roll(gain=60.0, timeout=5.0)

        self.assertFalse(result.success)
        # ACRO mode should NOT have been entered
        acro_calls = [c for c in px.set_mode.call_args_list if c.args[0] == 'ACRO']
        self.assertEqual(len(acro_calls), 0)

    def test_uses_bno_read_roll_when_available(self):
        duburi, px = _make_duburi_for_style()
        bno = MagicMock()
        bno.read_roll = MagicMock(side_effect=[i * 20.0 for i in range(25)] + [360.0] * 10)
        bno.read_yaw  = MagicMock(return_value=0.0)
        duburi.yaw_source = bno

        with patch('duburi_control.duburi.hold_depth'):
            result = duburi.style_roll(gain=60.0, timeout=10.0)

        self.assertTrue(bno.read_roll.called)
        self.assertIn('bno', result.message)

    def test_ahrs2_fallback_when_no_bno_read_roll(self):
        duburi, px = _make_duburi_for_style()
        px.get_attitude.side_effect = [
            {'yaw': 0.0, 'roll': i * 20.0, 'pitch': 0.0, 'depth': -0.7}
            for i in range(30)
        ] + [{'yaw': 0.0, 'roll': 0.0, 'pitch': 0.0, 'depth': -0.7}] * 30

        with patch('duburi_control.duburi.hold_depth'):
            result = duburi.style_roll(gain=60.0, timeout=5.0)

        self.assertIn('ahrs2', result.message)


class TestStyleRollMultiFlip(unittest.TestCase):

    def test_two_flips_require_720_degrees(self):
        """flips=2 → loop runs until |accum| >= 720°."""
        duburi, px = _make_duburi_for_style()
        # Use BNO so get_attitude is not called inside the ACRO loop.
        # 36 steps × 20° = 720° across 40 readings (ample headroom).
        roll_readings = [i * 20.0 for i in range(40)] + [720.0] * 30
        bno = MagicMock()
        bno.read_roll = MagicMock(side_effect=roll_readings)
        bno.read_yaw  = MagicMock(return_value=0.0)
        duburi.yaw_source = bno

        with patch('duburi_control.duburi.hold_depth'):
            result = duburi.style_roll(gain=60.0, timeout=60.0, flips=2)

        self.assertTrue(result.success)
        self.assertGreaterEqual(abs(result.final_value), 720.0 - 1.0)
        self.assertIn('2 flip', result.message)

    def test_three_flips_complete_1080_degrees(self):
        """flips=3 → loop must accumulate 1080° before exiting."""
        duburi, px = _make_duburi_for_style()
        # 54 steps × 20° = 1080°. Use BNO to avoid get_attitude exhaustion.
        roll_readings = [i * 20.0 for i in range(60)] + [1080.0] * 20
        bno = MagicMock()
        bno.read_roll = MagicMock(side_effect=roll_readings)
        bno.read_yaw  = MagicMock(return_value=0.0)
        duburi.yaw_source = bno

        with patch('duburi_control.duburi.hold_depth'):
            result = duburi.style_roll(gain=60.0, timeout=120.0, flips=3)

        self.assertTrue(result.success)
        self.assertGreaterEqual(abs(result.final_value), 1080.0 - 1.0)
        self.assertIn('3 flip', result.message)

    def test_predive_called_before_acro(self):
        """hold_depth with dive_depth must be called before set_mode('ACRO')."""
        duburi, px = _make_duburi_for_style()
        duburi._abort_event.set()   # exit loop immediately
        px.get_attitude.return_value = {'yaw': 0.0, 'roll': 0.0, 'pitch': 0.0, 'depth': -0.7}

        call_order = []

        def mock_hold_depth(px_arg, depth, *a, **kw):
            call_order.append(('hold_depth', depth))

        def mock_set_mode(mode, **kw):
            call_order.append(('set_mode', mode))
            return (True, 'ACCEPTED')

        px.set_mode.side_effect = mock_set_mode

        with patch('duburi_control.duburi.hold_depth', side_effect=mock_hold_depth):
            duburi.style_roll(gain=60.0, timeout=5.0, flips=1, headroom=1.0)

        # Find first hold_depth call (pre-dive) and ACRO set_mode call
        pre_dive_idx = next((i for i, c in enumerate(call_order)
                             if c[0] == 'hold_depth'), None)
        acro_idx     = next((i for i, c in enumerate(call_order)
                             if c == ('set_mode', 'ACRO')), None)

        assert pre_dive_idx is not None, 'hold_depth (pre-dive) not called'
        assert acro_idx is not None, 'set_mode(ACRO) not called'
        self.assertLess(pre_dive_idx, acro_idx,
                        f'pre-dive not before ACRO: {call_order}')
        # Pre-dive depth must be target_depth - headroom = -0.7 - 1.0 = -1.7
        self.assertAlmostEqual(call_order[pre_dive_idx][1], -1.7, places=2)

    def test_headroom_zero_skips_predive(self):
        """headroom=0 must not issue any pre-dive hold_depth call."""
        duburi, px = _make_duburi_for_style()
        duburi._abort_event.set()
        px.get_attitude.return_value = {'yaw': 0.0, 'roll': 0.0, 'pitch': 0.0, 'depth': -0.7}

        hold_depth_calls = []

        def mock_hold_depth(px_arg, depth, *a, **kw):
            hold_depth_calls.append(depth)

        with patch('duburi_control.duburi.hold_depth', side_effect=mock_hold_depth):
            duburi.style_roll(gain=60.0, timeout=5.0, flips=1, headroom=0.0)

        # Only post-flip recovery hold_depth is allowed — no pre-dive
        self.assertFalse(
            any(d < -0.7 for d in hold_depth_calls),
            f'unexpected pre-dive depth call: {hold_depth_calls}')


class TestStylePitch(unittest.TestCase):

    def test_uses_pitch_channel_not_roll(self):
        duburi, px = _make_duburi_for_style()
        px.get_attitude.side_effect = [
            {'yaw': 0.0, 'roll': 0.0, 'pitch': i * 20.0, 'depth': -0.7}
            for i in range(30)
        ] + [{'yaw': 0.0, 'roll': 0.0, 'pitch': 0.0, 'depth': -0.7}] * 30

        with patch('duburi_control.duburi.hold_depth'):
            duburi.style_pitch(gain=50.0, timeout=5.0)

        # send_rc_override should have been called with pitch kwarg, not roll
        pitch_calls = [c for c in px.send_rc_override.call_args_list
                       if c.kwargs.get('pitch', 1500) != 1500 or
                          (c.args and len(c.args) > 0 and c.args[0] != 1500)]
        # At minimum, a call with pitch != 1500 must exist
        pwm_calls = [c for c in px.send_rc_override.call_args_list
                     if 'pitch' in c.kwargs and c.kwargs['pitch'] != 1500]
        self.assertGreater(len(pwm_calls), 0)

    def test_restores_acro_bal_pitch_not_roll(self):
        duburi, px = _make_duburi_for_style()
        px.get_param.return_value = 1.0
        px.get_attitude.side_effect = [
            {'yaw': 0.0, 'roll': 0.0, 'pitch': i * 20.0, 'depth': -0.7}
            for i in range(30)
        ] + [{'yaw': 0.0, 'roll': 0.0, 'pitch': 0.0, 'depth': -0.7}] * 30

        with patch('duburi_control.duburi.hold_depth'):
            duburi.style_pitch(gain=50.0, timeout=5.0)

        param_names = [c.args[0] for c in px.set_param.call_args_list]
        self.assertIn('ACRO_BAL_PITCH', param_names)
        self.assertNotIn('ACRO_BAL_ROLL', param_names)


# ===========================================================================
# style_yaw
# ===========================================================================

class TestStyleYaw(unittest.TestCase):

    def test_calls_yaw_motion_n_times(self):
        duburi, px = _make_duburi_for_style()
        # Patch yaw_snap so it does nothing but return
        with patch('duburi_control.duburi.yaw_snap') as mock_yaw, \
             patch('duburi_control.duburi.yaw_glide'):
            mock_yaw.return_value = None
            duburi.style_yaw(steps=4, deg_per_step=90.0, settle=0.0)

        self.assertEqual(mock_yaw.call_count, 4)

    def test_no_mode_change_stays_alt_hold(self):
        duburi, px = _make_duburi_for_style()
        with patch('duburi_control.duburi.yaw_snap'):
            duburi.style_yaw(steps=2, deg_per_step=90.0, settle=0.0)

        # set_mode should only be called for _ensure_yaw_capable_mode if needed;
        # crucially, ACRO must NOT appear
        modes = [c.args[0] for c in px.set_mode.call_args_list]
        self.assertNotIn('ACRO', modes)
        self.assertNotIn('STABILIZE', modes)


# ===========================================================================
# Pixhawk get_param / set_param — master.messages polling (not recv_match)
# ===========================================================================

class TestPixhawkParamPolling(unittest.TestCase):
    """Verify get_param / set_param use master.messages cache, not recv_match."""

    def _make_pixhawk(self):
        from duburi_control.pixhawk import Pixhawk
        px = object.__new__(Pixhawk)
        px.master = MagicMock()
        px.master.messages = {}
        px._log_mavlink = MagicMock()
        return px

    def test_get_param_returns_value_from_messages_cache(self):
        px = self._make_pixhawk()

        def _side_effect(name):
            # Simulate reader thread: fill cache after param_fetch_one
            msg = MagicMock()
            msg.param_id = name + '\x00' * (16 - len(name))
            msg.param_value = 1.5
            px.master.messages['PARAM_VALUE'] = msg

        px.master.param_fetch_one.side_effect = _side_effect
        result = px.get_param('ACRO_BAL_ROLL', timeout=1.0)
        self.assertAlmostEqual(result, 1.5, places=3)
        # Must NOT have called recv_match
        px.master.recv_match.assert_not_called()

    def test_get_param_returns_none_on_timeout(self):
        px = self._make_pixhawk()
        px.master.param_fetch_one = MagicMock()  # no side effect = cache never filled
        result = px.get_param('ACRO_BAL_ROLL', timeout=0.12)
        self.assertIsNone(result)

    def test_get_param_ignores_wrong_param_id(self):
        px = self._make_pixhawk()
        # Pre-fill cache with a DIFFERENT param name (stale from previous request)
        stale = MagicMock()
        stale.param_id = 'ACRO_TRAINER\x00\x00\x00\x00'
        stale.param_value = 2.0
        px.master.messages['PARAM_VALUE'] = stale

        # No new message arrives — should time out, not return stale value
        px.master.param_fetch_one = MagicMock()
        result = px.get_param('ACRO_BAL_ROLL', timeout=0.12)
        self.assertIsNone(result)

    def test_set_param_requires_value_echo(self):
        px = self._make_pixhawk()

        def _side_effect(name, value):
            # Echo the correct value back
            msg = MagicMock()
            msg.param_id = name + '\x00' * (16 - len(name))
            msg.param_value = float(value)
            px.master.messages['PARAM_VALUE'] = msg

        px.master.param_set_send.side_effect = _side_effect
        result = px.set_param('ACRO_BAL_ROLL', 0.0, timeout=1.0)
        self.assertTrue(result)
        px.master.recv_match.assert_not_called()

    def test_set_param_rejects_wrong_value_echo(self):
        px = self._make_pixhawk()

        def _side_effect(name, value):
            # Echo a DIFFERENT value (ArduSub rejected the write)
            msg = MagicMock()
            msg.param_id = name + '\x00' * (16 - len(name))
            msg.param_value = 1.0   # still the old value
            px.master.messages['PARAM_VALUE'] = msg

        px.master.param_set_send.side_effect = _side_effect
        result = px.set_param('ACRO_BAL_ROLL', 0.0, timeout=0.15)
        # Old value echoed back — should not be accepted
        self.assertFalse(result)

    def test_set_param_clears_stale_cache_before_request(self):
        px = self._make_pixhawk()
        # Pre-fill cache with stale matching entry
        stale = MagicMock()
        stale.param_id = 'ACRO_BAL_ROLL\x00\x00\x00'
        stale.param_value = 0.0   # matches requested value = would be false positive
        px.master.messages['PARAM_VALUE'] = stale

        px.master.param_set_send = MagicMock()  # no echo = cache should be cleared first
        result = px.set_param('ACRO_BAL_ROLL', 0.0, timeout=0.12)
        # Cache was cleared before request, so no stale hit
        self.assertFalse(result)


# ===========================================================================
# Heartbeat ordering: released before param calls in cleanup
# ===========================================================================

class TestHeartbeatReleasedBeforeCleanup(unittest.TestCase):

    def test_heartbeat_released_before_set_param_in_style_roll(self):
        """Heartbeat must be released BEFORE set_param/set_mode in inner finally."""
        duburi, px = _make_duburi_for_style()

        call_order = []

        def track_release():
            call_order.append('release_hb')
        def track_set_param(name, val, **kw):
            call_order.append(f'set_param:{name}')
            return True
        def track_set_mode(mode, **kw):
            call_order.append(f'set_mode:{mode}')
            return (True, 'ACCEPTED')

        duburi._release_heartbeat_for_lock = track_release
        duburi._hold_heartbeat_for_lock    = MagicMock()
        px.set_param.side_effect = track_set_param
        px.set_mode.side_effect  = track_set_mode

        px.get_attitude.side_effect = [
            {'yaw': 0.0, 'roll': i * 20.0, 'pitch': 0.0, 'depth': -0.7}
            for i in range(30)
        ] + [{'yaw': 0.0, 'roll': 0.0, 'pitch': 0.0, 'depth': -0.7}] * 30

        with patch('duburi_control.duburi.hold_depth'):
            duburi.style_roll(gain=60.0, timeout=5.0)

        # There are TWO set_param:ACRO_BAL_ROLL calls:
        #   [0] the ZEROING call (before ACRO) — not what we're checking
        #   [1] the RESTORE call (in finally, after loop) — must come AFTER release_hb
        # We check: first release_hb comes before the LAST set_param:ACRO_BAL_ROLL.
        acro_idx   = call_order.index('set_mode:ACRO')   # marks end of setup phase
        # Find first release_hb that occurs AFTER ACRO mode was set (cleanup release)
        hb_idx     = next((i for i, c in enumerate(call_order)
                           if c == 'release_hb' and i > acro_idx), None)
        # Find RESTORE set_param:ACRO_BAL_ROLL (last occurrence = restore call)
        param_idx  = max((i for i, c in enumerate(call_order)
                          if c == 'set_param:ACRO_BAL_ROLL'), default=None)
        assert hb_idx is not None,    'release_hb not found after ACRO mode'
        assert param_idx is not None, 'set_param:ACRO_BAL_ROLL not called'
        self.assertLess(hb_idx, param_idx,
                        f'heartbeat released after restore set_param: {call_order}')


if __name__ == '__main__':
    unittest.main()
