"""Every package declares, in package.xml, what its code imports.

`rosdep install --from-paths src` reads package.xml and nothing else, so an
import a package does not declare is a module a blank card never gets, and the
first sign of it is an ImportError on the vehicle. Kutum's `setup.sh` happens
to install most of these by hand today; this keeps package.xml true on its own.

pymavlink is the one deliberate exception: it has no rosdep key, and it is
pinned in requirements.txt and kutum/requirements.txt.
"""
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PACKAGES = ['mongla_control', 'mongla_localization', 'mongla_manager',
            'mongla_planner', 'mongla_vision']
ROSDEP_KEY = {'numpy': 'python3-numpy', 'cv2': 'python3-opencv', 'yaml': 'python3-yaml',
              'serial': 'python3-serial', 'PIL': 'python3-pil', 'scipy': 'python3-scipy'}
ROS_PACKAGES = {'rclpy', 'std_msgs', 'sensor_msgs', 'geometry_msgs', 'nav_msgs',
                'vision_msgs', 'tf2_ros', 'cv_bridge', 'rcl_interfaces',
                'builtin_interfaces', 'diagnostic_msgs', 'visualization_msgs',
                'action_msgs'}
IMPORT = re.compile(r'^\s*(?:import|from)\s+([A-Za-z_][A-Za-z0-9_]*)', re.M)


def _imports(pkg):
    found = set()
    for py in (ROOT / 'src' / pkg / pkg).rglob('*.py'):
        found |= set(IMPORT.findall(py.read_text(encoding='utf-8')))
    return found


def _declared(pkg):
    xml = (ROOT / 'src' / pkg / 'package.xml').read_text()
    return set(re.findall(r'<(?:exec_)?depend>([^<]+)</', xml))


def test_every_import_is_declared():
    missing = []
    for pkg in PACKAGES:
        imported = _imports(pkg)
        assert imported, f'{pkg}: parsed no imports -- the pattern broke'
        need = {ROSDEP_KEY[m] for m in imported if m in ROSDEP_KEY}
        need |= imported & ROS_PACKAGES
        missing += [f'{pkg}: {k}' for k in sorted(need - _declared(pkg))]
    assert not missing, ('imported but not in package.xml, so rosdep never installs '
                         'it:\n  ' + '\n  '.join(missing))
