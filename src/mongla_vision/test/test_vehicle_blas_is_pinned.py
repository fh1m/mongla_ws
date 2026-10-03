"""The vehicle's BLAS: OpenBLAS checked at provisioning, one thread per call.

measured-bars §28.5: the Pi's numpy linked the Netlib REFERENCE BLAS, and the
XFeat matcher's 1024x1024x64 matmul took 58.6 ms there against 6.4 ms on
OpenBLAS. OpenBLAS's own pool defaults to every core in every process, which
under the matcher's deliberate 2-thread split is 8 threads on 4 cores.
"""
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
    return src.index('return LaunchDescription(args + [')


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


def test_provisioning_checks_for_openblas_and_names_the_fix():
    sh = (ROOT / 'scripts/provision_vehicle.sh').read_text()
    assert 'libblas.so.3 | grep -q openblas' in sh
    assert 'sudo apt-get install -y libopenblas0-pthread' in sh
    # Checked BEFORE --check-only stops, or the check never runs on its own.
    assert sh.index('grep -q openblas') < sh.index('check-only: stopping')
