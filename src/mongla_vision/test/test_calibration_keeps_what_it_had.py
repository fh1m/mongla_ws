"""Calibration never loses a fit or a view it already had (audit vl_5 F3, F4, F5, F8, F9)."""
import json
import pathlib
import subprocess

from mongla_vision.calibration import guide_capture, guide_files

SRC = pathlib.Path(__file__).resolve().parents[1] / 'mongla_vision' / 'calibration'


def test_no_v4l2_ctl_is_false_not_a_raise(monkeypatch):
    def boom(*a, **k):
        raise FileNotFoundError('v4l2-ctl')
    monkeypatch.setattr(subprocess, 'run', boom)
    assert guide_capture.set_camera('/dev/null', 100, 0) is False


def test_an_unmeasured_blur_is_not_sharp():
    src = (SRC / 'guide_capture.py').read_text()
    assert 'oks = sharp is not None and sharp <= MAX_SHARPNESS_PX' in src


def test_a_capture_never_lands_on_a_later_view():
    src = (SRC / 'guide_capture.py').read_text()
    assert "while os.path.exists(os.path.join(\n                            st.outdir, f'cal_{st.i:03d}.png'))" in src


def test_solve_keeps_the_calibration_it_replaces():
    src = (SRC / 'solver.py').read_text()
    assert "os.replace(dest, dest + '.bak-'" in src


def test_reapplying_a_file_to_its_own_name_keeps_a_copy(tmp_path, monkeypatch):
    name = 'pi_downward_1280x720.json'
    (tmp_path / name).write_text(json.dumps({
        'image_width': 1280, 'image_height': 720, 'medium': 'air',
        'usb_vid': 'aaaa', 'camera_matrix': [[1, 0, 0], [0, 1, 0], [0, 0, 1]]}))
    monkeypatch.setattr(guide_files, 'camera_identity',
                        lambda dev: {'usb_vid': 'bbbb', 'usb_pid': '1'})
    got, err = guide_files.apply_calibration(str(tmp_path), name, '/dev/x', 'pi_downward')
    assert err is None, err
    baks = list(tmp_path.glob(name + '.bak-*'))
    assert len(baks) == 1 and json.loads(baks[0].read_text())['usb_vid'] == 'aaaa'


def test_a_capture_is_written_only_for_the_step_it_was_taken_for():
    """vl_5 F10: index checked and write result read, under the lock."""
    src = (SRC / 'guide_capture.py').read_text()
    assert 'if st.i != i:' in src and 'if not cv2.imwrite(fn, f):' in src
