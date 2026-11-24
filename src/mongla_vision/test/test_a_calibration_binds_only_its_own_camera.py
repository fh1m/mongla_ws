"""A camera uses only a calibration measured on the unit plugged in (audit
vl_5 F6, vl_6 F9; owner 2026-11-24: forward is the Fantech C30, downward the
Sonix global shutter)."""
import json
import pathlib

from mongla_vision.calibration import binding

CAL = pathlib.Path(__file__).resolve().parents[1] / 'config' / 'calibration'
NODE = pathlib.Path(__file__).resolve().parents[1] / 'mongla_vision' / 'camera_node.py'
FANTECH, SONIX = ('1d6c', '0103'), ('0c45', '636d')


def test_the_verdicts():
    rec = {'usb_vid': '1d6c', 'usb_pid': '0103'}
    assert binding.pairing(rec, FANTECH) == 'match'
    assert binding.pairing(rec, SONIX) == 'different'
    assert binding.pairing({}, SONIX) == 'unverified'
    assert binding.pairing(rec, ('', '')) == 'unknown'


def test_the_shipped_files_pair_with_the_vehicles_cameras():
    fwd = json.loads((CAL / 'pi_forward_1280x720.json').read_text())
    dwn = json.loads((CAL / 'pi_downward_1280x720.json').read_text())
    assert binding.pairing(fwd, FANTECH) == 'match'
    assert binding.pairing(fwd, SONIX) == 'different'      # never on the downward
    assert binding.pairing(dwn, SONIX) == 'unverified'     # no id recorded: retake


def test_the_node_refuses_another_units_calibration():
    src = NODE.read_text()
    i = src.index("if verdict == 'different':")
    assert 'return None' in src[i:i + 600]


def test_an_unreadable_device_is_unknown():
    assert binding.device_usb_id('/dev/no_such_video_device') == ('', '')
