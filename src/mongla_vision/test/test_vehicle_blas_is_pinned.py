"""The vehicle's BLAS: OpenBLAS checked at provisioning, one thread per call.

measured-bars §28.5: the Pi's numpy linked the Netlib REFERENCE BLAS, and the
XFeat matcher's 1024x1024x64 matmul took 58.6 ms there against 6.4 ms on
OpenBLAS. OpenBLAS's own pool defaults to every core in every process, which
under the matcher's deliberate 2-thread split is 8 threads on 4 cores.
"""
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'tools'))
from code_text import code_of_file  # noqa: E402

LAUNCHES = (
    ROOT / 'src/mongla_vision/launch/vision_pi.launch.py',
    ROOT / 'src/mongla_manager/launch/bringup.launch.py',
)


def _first_action_index(src: str) -> int:
    # bringup puts a LogInfo (`file_note`, the settings file it read) and the
    # `nodes:` actions (`params`: a log line, an env var, a parameter file)
    # between the arguments and the nodes; none of them starts a library.
    m = re.search(r'return LaunchDescription\(args \+ (?:file_note \+ )?(?:params \+ )?\[', src)
    assert m, 'no `return LaunchDescription(args + [...` to anchor on'
    return m.start()


def test_every_vehicle_launch_pins_one_blas_thread():
    for p in LAUNCHES:
        src = code_of_file(p)
        assert "SetEnvironmentVariable('OPENBLAS_NUM_THREADS', '1')" in src, p.name


def test_the_pin_comes_BEFORE_any_node():
    """OpenBLAS reads the variable when the library loads. A node started
    before the action runs keeps the default pool."""
    for p in LAUNCHES:
        src = code_of_file(p)
        i = _first_action_index(src)
        body = src[i:]
        pin = body.index("SetEnvironmentVariable('OPENBLAS_NUM_THREADS'")
        first_entry = body.split('[', 1)[1].lstrip()
        assert first_entry.startswith('SetEnvironmentVariable'), (
            f'{p.name}: the BLAS pin is not the first action ({pin})')


def test_kutum_installs_openblas_and_the_doctor_checks_for_it():
    setup = (ROOT / 'kutum/setup.sh').read_text()
    doctor = (ROOT / 'kutum/doctor.sh').read_text()
    assert 'libopenblas0-pthread' in setup.split('PKGS=', 1)[1].split('"', 2)[1]
    assert 'libblas.so.3 | grep -q openblas' in doctor
    assert 'libopenblas0-pthread' in doctor   # the WARNING names the fix
